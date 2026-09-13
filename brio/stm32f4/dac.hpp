/*
 * dac.hpp
 *
 * The STM32F4's digital-to-analog converter (RM0090 ch. 14, RM0390
 * ch. 14): `Dac`, a MONOSTATE resource, because a part either has one DAC
 * block or none at all - and the F401, F411 and F412 have none, which
 * their device header says by declaring no DAC_BASE. On such a part the
 * register-facing half of this file is not compiled and a `Dac` spelled
 * there is a compile error naming the reason; the vocabulary above it -
 * the triggers, the waves, the channel config and its validity - is every
 * part's.
 *
 * WHAT THIS CONVERTER IS. Two 12-bit channels, DAC_OUT1 on PA4 and
 * DAC_OUT2 on PA5 (14.3's table of DAC pins), each with its own trigger,
 * its own DMA request and its own output buffer.
 *
 * AND WHERE ITS OUTPUT CAN GO: TO A PAD, AND NOWHERE ELSE. This family's
 * DAC has no MCR and no internal route - 14.3's own note says the pin is
 * "automatically connected to the analog converter output" as soon as the
 * channel is enabled, and there is no code that keeps it off the pad. So
 * the only way from this converter to the ADC is the BOND PAD the two
 * share (PA4 is ADC12_IN4, PA5 is ADC12_IN5 on every datasheet of the
 * family), which is what makes a wireless DAC-into-ADC experiment
 * possible here at all - and what makes "configure the pad to analog
 * FIRST" a real obligation and not hygiene: a pad still in its reset
 * state (input floating, with its input buffer live) sits across a driven
 * analog node.
 *
 * FIVE FACTS THAT SHAPE THIS FILE.
 *
 * 1. CHANNELS ARE 0-BASED HERE AND 1-BASED IN THE MANUAL. `ch = 0` is
 *    DAC_CH1 / DAC_OUT1 / PA4 and `ch = 1` is DAC_CH2 / DAC_OUT2 / PA5.
 *    Every channel verb of this stratum counts from zero, and one file
 *    counting differently would be worse than one offset stated loudly.
 *    The two channels' fields are the same bits sixteen apart in CR, and
 *    that is what makes every verb below one body.
 *
 * 2. THE DATA REGISTER IS NOT THE OUTPUT REGISTER. A write lands in a
 *    holding register and reaches DAC_DORx one APB1 cycle later with no
 *    trigger, three cycles after the trigger with one, and one with the
 *    SOFTWARE trigger (14.3.4 and 14.3.6's note). So `output(ch)` reads
 *    DOR and `code(ch)` reads back what was asked for: a caller that
 *    confuses them measures its own write. And DOR is READ-ONLY - 14.3.4's
 *    first sentence - so there is no verb here that writes it.
 *
 * 3. THE BUFFER IS DISABLED BY A ONE. BOFFx is an output buffer DISABLE
 *    (14.5.1), the opposite polarity of everything around it, so the
 *    config field is `buffered` and the inversion happens once, here. A
 *    buffered output drives a load and cannot reach either rail; an
 *    unbuffered one reaches them and cannot drive anything - including,
 *    measurably, the ADC's own sampling capacitor.
 *
 * 4. A TRIGGER IS AN EDGE AND THE WAVE GENERATORS NEED ONE. 14.5.1:
 *    WAVEx is "only used if TENx = 1", so noise and triangle are refused
 *    without a trigger rather than quietly doing nothing; and TSELx
 *    cannot be written with ENx set, nor MAMPx after the channel is
 *    enabled - which is why `configure()` drops the enable first and puts
 *    nothing back.
 *
 * 5. THE DMA REQUEST COMES FROM A HARDWARE TRIGGER AND FROM NOTHING
 *    ELSE - 14.3.7's own parenthesis, "an external trigger (but not a
 *    software trigger)" - which is why a DMA-fed channel with the
 *    software trigger selected is refused here rather than left silent.
 *    Its cell is DAC1 on DMA1's stream 5, DAC2 on stream 6, channel 7
 *    both, and `engine_placed()` is what an application asserts against
 *    the reserve's copy of that table.
 *
 * 6. THE DMA REQUEST IS NOT QUEUED. 14.3.7: a second trigger arriving
 *    before the first request was acknowledged raises DMAUDRx and the
 *    channel keeps converting the OLD datum - so the flag is the count of
 *    samples a stream did not deliver, and the chapter's own recovery is
 *    to clear it, stop the stream and re-initialize both. This driver
 *    reports; the owner decides.
 *
 * WHAT THIS FAMILY'S DAC HAS NOT, and the STM32G0's has: an offset
 * calibration (no DAC_CCR, no CEN, no trim), sample-and-hold (no SHSR /
 * SHHR / SHRR), and any mode bits at all (no DAC_MCR). Three registers
 * fewer and one decision fewer.
 *
 * The reference is the ADC's: 14.3.5's transfer function is
 * VREF+ x DOR / 4096, the same pad `stm32f4/adc.hpp`'s `Ref` names.
 *
 * ERRATA (ES0206 Rev 24 2.6, ES0298 Rev 8 2.7 - the F411's ES0287 files
 * the same two items though that part has no DAC):
 *  - "DMA request not automatically cleared by clearing DMAEN": stopping
 *    a DMA-to-DAC stream leaves a pending request behind, to be served
 *    the moment the DAC is enabled again. The workaround is a SEQUENCE
 *    over two peripherals (check DMAUDR, clear DMAEN, close the DAC's bus
 *    clock, reconfigure both) and its middle step is this driver's:
 *    `stop_dma()` does the DAC's half and says what the DMA's half is.
 *  - "DMA underrun flag not set when an internal trigger is detected on
 *    the clock cycle of the DMA request acknowledge": no workaround, and
 *    it only bites where software and hardware triggers are used
 *    together. Stated on `underrun()`.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// DAC_CR.TSELx - table 94's eight triggers, by the code that selects
/// them. Unlike the ADC's tables there are no gaps: all eight codes carry
/// something, and what varies across the family is whether the TIMER
/// behind a code exists (`dac_trigger_valid()`).
enum class DacTrigger : uint8_t {
    tim6_trgo = 0,
    tim8_trgo = 1,
    tim7_trgo = 2,
    tim5_trgo = 3,
    tim2_trgo = 4,
    tim4_trgo = 5,
    exti9 = 6,
    software = 7,
};

/// Which timer a trigger code names, 0 for the two that name none.
constexpr uint8_t dac_trigger_timer(DacTrigger t) {
    switch (t) {
        case DacTrigger::tim6_trgo: return 6;
        case DacTrigger::tim8_trgo: return 8;
        case DacTrigger::tim7_trgo: return 7;
        case DacTrigger::tim5_trgo: return 5;
        case DacTrigger::tim2_trgo: return 2;
        case DacTrigger::tim4_trgo: return 4;
        default: return 0;
    }
}

/// A trigger code is usable where the timer behind it exists - the F410
/// has TIM1, TIM5, TIM6 and TIM9 alone, so four of the six timer codes
/// are dead silicon there, and the reserve reads that off the timers' own
/// base macros.
constexpr bool dac_trigger_valid(DacTrigger t) {
    const uint8_t tim = dac_trigger_timer(t);
    return tim == 0u || tim_present(tim);
}

/// The EXTI line the seventh code names - stated here so that an
/// application arming it never spells the number itself.
inline constexpr uint8_t dac_exti_line = 9;

/// DAC_CR.WAVEx (14.3.8, 14.3.9). Both generators advance on a trigger
/// and on nothing else; the manual spells the triangle code "1x", so two
/// bit patterns mean it and this enum offers the canonical one.
enum class DacWave : uint8_t { none = 0, noise = 1, triangle = 2 };

/// DAC_CR.MAMPx: the LFSR's mask, or the triangle's amplitude. The value
/// is 2^(code + 1) - 1 in both readings (14.5.1), and every code from 11
/// up means the full 4095.
constexpr uint16_t dac_wave_amplitude(uint8_t mamp) {
    const uint8_t c = mamp > 11u ? 11u : mamp;
    return static_cast<uint16_t>((1u << (c + 1u)) - 1u);
}

/// The LFSR's preloaded value (14.3.8), which is what a noise channel
/// puts out on its FIRST trigger with a zero holding register - the one
/// deterministic thing about a pseudo-random generator.
inline constexpr uint16_t dac_lfsr_preload = 0x0AAAu;

/// The pads the two channels drive, on every part of the family (14.3's
/// note): DAC_OUT1 is PA4, DAC_OUT2 is PA5. Spelled here so that an
/// application never names them itself.
constexpr char dac_pad_port(uint8_t ch) { return ch < 2u ? 'A' : 0; }
constexpr uint8_t dac_pad_pin(uint8_t ch) {
    return ch == 0u ? 4u : (ch == 1u ? 5u : 0xFFu);
}

/// What one channel is configured with.
struct DacChannelConfig {
    /// BOFFx, inverted: true drives a load and cannot reach the rails,
    /// false reaches them and drives nothing (14.3.2).
    bool buffered = true;
    /// TENx + TSELx. `software` with `triggered` true is 14.3.6's SWTRIG;
    /// `triggered` false ignores the selection entirely and a write
    /// reaches the output one APB1 cycle later on its own.
    bool triggered = false;
    DacTrigger trigger = DacTrigger::software;
    DacWave wave = DacWave::none;
    uint8_t amplitude = 0;             ///< MAMPx, 0..15 (11 and up are all 4095)
    bool dma = false;                  ///< DMAENx
    bool underrun_interrupt = false;   ///< DMAUDRIEx
};

constexpr bool dac_channel_config_valid(const DacChannelConfig& c) {
    if (!dac_trigger_valid(c.trigger)) {
        return false;
    }
    if (c.amplitude > 15u) {
        return false;   // MAMPx is four bits
    }
    if (c.wave != DacWave::none && !c.triggered) {
        return false;   // 14.5.1: WAVEx is only used with TENx set
    }
    if (c.dma && !c.triggered) {
        return false;   // 14.3.7: the request comes from a trigger, and only a hardware one
    }
    if (c.dma && c.trigger == DacTrigger::software) {
        return false;   // 14.3.7: "an external trigger (but not a software trigger)"
    }
    return true;
}

// The register-facing half is compiled only where the device header
// declares the block.
#if defined(DAC_BASE)

/// DAC_SR's per-channel bit (14.5.14), by channel index. The two sit
/// sixteen apart like everything else in this peripheral.
struct DacFlag {
    static constexpr uint32_t underrun(uint8_t ch) {
        return ch == 0u ? DAC_SR_DMAUDR1 : DAC_SR_DMAUDR2;
    }
};

// =============================================================================
// The resource
// =============================================================================

class Dac {
public:
    static_assert(dac_present(),
                  "brio Dac: this device has no DAC (the F401, F411 and F412 have none, and "
                  "their device header declares no DAC_BASE)");

    Dac() = delete;

    /// How many outputs this part really has - the reference manual's
    /// number and not the header's, which declares both channels' bits
    /// everywhere (stm32f4/device_tables.hpp says why).
    static constexpr uint8_t channels = dac_channel_facts().channels;
    /// Whether that number comes from a manual that was read.
    static constexpr bool channels_known = dac_channel_facts().known;

    /// The NVIC line - the underrun interrupt's only route, SHARED WITH
    /// TIM6 (the vector is spelled TIM6_DAC on every part that has a DAC),
    /// so a handler bound here is a dispatcher.
    static constexpr IRQn_Type irq() { return dac_irq(); }

    static DAC_TypeDef& regs() { return *reinterpret_cast<DAC_TypeDef*>(dac_base()); }

    /// The converter's full scale, for util/analog.hpp's dac_code() /
    /// dac_mv(). Twelve bits always: the 8-bit format is a PLACEMENT, not
    /// a resolution (14.3.3 stores it in DHRx[11:4]).
    static constexpr uint32_t steps = 4096;

    // ---- the block ---------------------------------------------------------------

    static void bus_clock(bool on) { Rcc::apb1_clock(dac_clock_mask(), on); }
    static bool bus_clock() { return Rcc::apb1_clock(dac_clock_mask()); }
    static void reset() { Rcc::apb1_reset(dac_reset_mask()); }

    /// Bus clock on, block reset, both channels off. The PADS are not
    /// touched: which of them this program wants in analog mode is the
    /// caller's, through `claim_pad()`.
    static void init() {
        bus_clock(true);
        reset();
        bus_clock(true);
    }

    static void release() {
        regs().CR = 0;
        reset();
        bus_clock(false);
    }

    /**
     * The pad a channel drives, in the mode 14.3's note demands: analog,
     * which on this family is NOT the reset state (stm32f4/pin.hpp fact
     * 3) - so this is a real store and skipping it leaves a live input
     * buffer across a driven analog node.
     *
     * Spelled as a verb over the Pin type so that an application names
     * `Pin<'A', 4>` at most once; `dac_pad_pin()` says which pad a channel
     * is if it would rather not name it at all.
     */
    template <class P>
    static void claim_pad() { P::analog(); }
    template <class P>
    static void release_pad() { P::analog(); }

    // ---- per-channel configuration --------------------------------------------------

    static constexpr bool channel_valid(uint8_t ch) { return ch < channels; }

    /**
     * Configure one channel and leave it DISABLED.
     *
     * 14.3.6's note makes TSELx unwritable with ENx set and 14.3.9's makes
     * MAMPx unchangeable once the channel is enabled, so this verb drops
     * the enable first and does not put it back: a caller that wants the
     * channel running calls `enable(ch, true)` after, which is also where
     * tWAKEUP is spent.
     */
    static bool configure(uint8_t ch, const DacChannelConfig& c) {
        if (!channel_valid(ch) || !dac_channel_config_valid(c)) {
            return false;
        }
        cfg_[ch] = c;
        const uint32_t shift = channel_shift(ch);
        uint32_t cr = regs().CR & ~(channel_cr_mask() << shift);
        if (!c.buffered) {
            cr |= DAC_CR_BOFF1 << shift;
        }
        if (c.triggered) {
            cr |= DAC_CR_TEN1 << shift;
            cr |= static_cast<uint32_t>(c.trigger) << (DAC_CR_TSEL1_Pos + shift);
        }
        cr |= static_cast<uint32_t>(c.wave) << (DAC_CR_WAVE1_Pos + shift);
        cr |= static_cast<uint32_t>(c.amplitude) << (DAC_CR_MAMP1_Pos + shift);
        if (c.dma) {
            cr |= DAC_CR_DMAEN1 << shift;
        }
        if (c.underrun_interrupt) {
            cr |= DAC_CR_DMAUDRIE1 << shift;
        }
        regs().CR = cr;
        return true;
    }

    static const DacChannelConfig& config(uint8_t ch) { return cfg_[ch < 2u ? ch : 0]; }

    /// ENx. 14.3.1: the channel is usable after tWAKEUP, and the DIGITAL
    /// interface works with the bit clear - which is what lets a holding
    /// register be primed before the analog part is on.
    static bool enable(uint8_t ch, bool on) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t bit = DAC_CR_EN1 << channel_shift(ch);
        regs().CR = on ? (regs().CR | bit) : (regs().CR & ~bit);
        return true;
    }

    static bool enabled(uint8_t ch) {
        return channel_valid(ch) && (regs().CR & (DAC_CR_EN1 << channel_shift(ch))) != 0u;
    }

    /// The wave generator alone, which is the one part of a configured
    /// channel 14.3.8 lets a program stop without disabling anything:
    /// "it is possible to reset the wave generation by resetting the
    /// WAVEx bits", and that also reloads the LFSR.
    static bool wave(uint8_t ch, DacWave w) {
        if (!channel_valid(ch) || (w != DacWave::none && !cfg_[ch].triggered)) {
            return false;
        }
        cfg_[ch].wave = w;
        const uint32_t shift = channel_shift(ch);
        regs().CR = (regs().CR & ~(DAC_CR_WAVE1_Msk << shift)) |
                    (static_cast<uint32_t>(w) << (DAC_CR_WAVE1_Pos + shift));
        return true;
    }

    static DacWave wave(uint8_t ch) {
        if (!channel_valid(ch)) {
            return DacWave::none;
        }
        const uint32_t w = (regs().CR >> (DAC_CR_WAVE1_Pos + channel_shift(ch))) & 0x3u;
        return w >= 2u ? DacWave::triangle : static_cast<DacWave>(w);
    }

    // ---- data ------------------------------------------------------------------------

    /// 12-bit right-aligned (DAC_DHR12Rx), the natural format.
    static bool write(uint8_t ch, uint16_t code) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t v = code & 0x0FFFu;
        if (ch == 0u) {
            regs().DHR12R1 = v;
        } else {
            regs().DHR12R2 = v;
        }
        return true;
    }

    /// 12-bit LEFT-aligned (DAC_DHR12Lx): the datum sits in bits 15:4,
    /// which is what a 16-bit sample stream already looks like.
    static bool write_left(uint8_t ch, uint16_t value) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t v = value & 0xFFF0u;
        if (ch == 0u) {
            regs().DHR12L1 = v;
        } else {
            regs().DHR12L2 = v;
        }
        return true;
    }

    /// 8-bit right-aligned (DAC_DHR8Rx): stored into DHRx[11:4], so the
    /// four low bits of the converter are zero and the full scale is 255
    /// steps of sixteen.
    static bool write8(uint8_t ch, uint8_t code) {
        if (!channel_valid(ch)) {
            return false;
        }
        if (ch == 0u) {
            regs().DHR8R1 = code;
        } else {
            regs().DHR8R2 = code;
        }
        return true;
    }

    /// Both channels in ONE store (DAC_DHR12RD, 14.4): the register that
    /// makes a two-channel waveform one bus access - and, with one
    /// DMAENx, one DMA request.
    static bool write_dual(uint16_t code0, uint16_t code1) {
        if (channels < 2u) {
            return false;
        }
        regs().DHR12RD = (static_cast<uint32_t>(code1 & 0x0FFFu) << 16) | (code0 & 0x0FFFu);
        return true;
    }

    /// DAC_DHR12LD - the dual left-aligned register, whose halves are at
    /// bits 19:4 and 31:20 (14.5.10) and NOT sixteen apart like every
    /// other pair in this peripheral.
    static bool write_dual_left(uint16_t value0, uint16_t value1) {
        if (channels < 2u) {
            return false;
        }
        regs().DHR12LD = (static_cast<uint32_t>(value1 & 0xFFF0u) << 16) | (value0 & 0xFFF0u);
        return true;
    }

    /// DAC_DHR8RD - both channels as two bytes of one half-word.
    static bool write_dual8(uint8_t code0, uint8_t code1) {
        if (channels < 2u) {
            return false;
        }
        regs().DHR8RD = (static_cast<uint32_t>(code1) << 8) | code0;
        return true;
    }

    /// What was ASKED for: the 12-bit right-aligned holding register,
    /// before any trigger. The other two formats write the SAME internal
    /// holding register, so a code written 8-bit or left-aligned reads
    /// back here shifted into place.
    static uint16_t code(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint16_t>((ch == 0u ? regs().DHR12R1 : regs().DHR12R2) & 0x0FFFu);
    }

    /// What the converter is PRODUCING: DAC_DORx, read-only (14.3.4),
    /// which the holding register reaches one APB1 cycle later untriggered
    /// and three after a hardware trigger. With a wave generator running
    /// this is the holding register PLUS the generator's own value, which
    /// is how a program watches an LFSR or a triangle without a meter.
    static uint16_t output(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint16_t>((ch == 0u ? regs().DOR1 : regs().DOR2) & 0x0FFFu);
    }

    /// SWTRIGx (14.5.2), the software trigger: write-only, self-clearing
    /// one APB1 cycle later once the holding register has been taken.
    static bool trigger(uint8_t ch) {
        if (!channel_valid(ch)) {
            return false;
        }
        regs().SWTRIGR = ch == 0u ? DAC_SWTRIGR_SWTRIG1 : DAC_SWTRIGR_SWTRIG2;
        return true;
    }

    /// Both at once - 14.4.6's simultaneous software start.
    static void trigger_both() {
        regs().SWTRIGR = DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
    }

    // ---- the DMA -----------------------------------------------------------------------

    /// Where a DMA stream writes a sample. THE FORMAT IS THE CALLER'S
    /// CHOICE and it decides the beat width: DHR12Rx wants a halfword,
    /// DHR8Rx a byte, DHR12RD a word for both channels - and a stream
    /// whose element type disagrees with the register it was pointed at
    /// writes a number the converter never meant.
    static volatile void* data_address_12r(uint8_t ch) {
        return ch == 0u ? static_cast<volatile void*>(&regs().DHR12R1)
                        : static_cast<volatile void*>(&regs().DHR12R2);
    }
    static volatile void* data_address_12l(uint8_t ch) {
        return ch == 0u ? static_cast<volatile void*>(&regs().DHR12L1)
                        : static_cast<volatile void*>(&regs().DHR12L2);
    }
    static volatile void* data_address_8r(uint8_t ch) {
        return ch == 0u ? static_cast<volatile void*>(&regs().DHR8R1)
                        : static_cast<volatile void*>(&regs().DHR8R2);
    }
    static volatile void* data_address_dual_12r() { return &regs().DHR12RD; }

    /// Whether an engine sits on a CELL of the request mapping that really
    /// carries this channel's request - `uart_engine_placed()`'s question
    /// for a DAC channel, with the same refusal where the part class's
    /// table was not read (stm32f4/device_tables.hpp).
    template <typename E>
    static constexpr bool engine_placed(uint8_t ch) {
        if constexpr (!E::present) {
            return true;
        } else {
            return dac_dma_placement_valid(ch, E::controller, E::stream, E::channel);
        }
    }

    static constexpr DmaPlacements dma_placements(uint8_t ch) { return dac_dma_placements(ch); }

    /**
     * The DAC's half of the errata's stop sequence (ES0206 2.6.1, ES0298
     * 2.7.1): a request already asserted is NOT withdrawn by clearing
     * DMAENx, and it will be served the moment the converter is enabled
     * again. So this clears the underrun flag, drops DMAENx and drops the
     * channel's enable - and the caller still owes the OTHER half, which
     * this driver cannot do: the DMA stream disabled and re-initialized
     * before anything is enabled again.
     */
    static bool stop_dma(uint8_t ch) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t shift = channel_shift(ch);
        regs().SR = DacFlag::underrun(ch);
        regs().CR &= ~((DAC_CR_DMAEN1 | DAC_CR_EN1) << shift);
        cfg_[ch].dma = false;
        return true;
    }

    // ---- flags and interrupts -------------------------------------------------------------

    static uint32_t flags() { return regs().SR; }
    /// DAC_SR's two flags are write-1-to-clear (14.5.14) - the ADC's SR,
    /// three registers away, is the opposite.
    static void clear_flags(uint32_t mask) { regs().SR = mask; }

    /**
     * DMAUDRx. 14.3.7: a trigger that arrives before the previous request
     * was acknowledged is LOST and the channel keeps converting the old
     * datum - so this is the count of samples a stream did not deliver.
     *
     * ES0206 2.6.2 / ES0298 2.7.2, no workaround: where SOFTWARE and
     * HARDWARE triggers are used together, an internal trigger landing on
     * the same clock cycle as a request acknowledge raises no flag at all.
     * A stream whose triggers come from one source only is not affected.
     */
    static bool underrun(uint8_t ch) {
        return channel_valid(ch) && (regs().SR & DacFlag::underrun(ch)) != 0u;
    }
    static bool clear_underrun(uint8_t ch) {
        if (!channel_valid(ch)) {
            return false;
        }
        regs().SR = DacFlag::underrun(ch);
        return true;
    }

    static void interrupts(uint8_t ch, bool on) {
        if (!channel_valid(ch)) {
            return;
        }
        const uint32_t bit = DAC_CR_DMAUDRIE1 << channel_shift(ch);
        regs().CR = on ? (regs().CR | bit) : (regs().CR & ~bit);
    }

    /**
     * The ISR BODY: the underrun flags this converter has ARMED, cleared
     * and handed back. The vector is TIM6's too, so it answers 0 when the
     * DAC did not speak.
     *
     * DMAUDRIEx sits in CR and the flag in SR, so the arming has to be
     * looked up rather than masked in one register - which is why this
     * body is four lines and not one.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t cr = regs().CR;
        uint32_t armed = 0;
        if ((cr & DAC_CR_DMAUDRIE1) != 0u) {
            armed |= DAC_SR_DMAUDR1;
        }
        if (channels > 1u && (cr & DAC_CR_DMAUDRIE2) != 0u) {
            armed |= DAC_SR_DMAUDR2;
        }
        const uint32_t hit = regs().SR & armed;
        if (hit != 0u) {
            regs().SR = hit;
        }
        return hit;
    }

private:
    /// The two channels' fields are the same bits sixteen apart in CR.
    static constexpr uint32_t channel_shift(uint8_t ch) { return ch == 0u ? 0u : 16u; }

    static constexpr uint32_t channel_cr_mask() {
        return DAC_CR_EN1 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_TSEL1_Msk | DAC_CR_WAVE1_Msk |
               DAC_CR_MAMP1_Msk | DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1;
    }

    inline static DacChannelConfig cfg_[2]{};
};

#endif // DAC_BASE

} // namespace brio
