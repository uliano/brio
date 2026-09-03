/*
 * dac.hpp
 *
 * The STM32G0's digital-to-analog converter (RM0444 ch. 16): `Dac`, a
 * MONOSTATE resource, because there is exactly one DAC on every part
 * that has one at all (16.3's own table - the G031/G041 have none, and
 * their device header declares no DAC1_BASE, which is what
 * stm32g0/device_tables.hpp reads).
 *
 * WHAT THIS CONVERTER IS. Two 12-bit channels, DAC1_OUT1 on PA4 and
 * DAC1_OUT2 on PA5 (16.3), each with its own trigger, its own DMA
 * request and its own output buffer - and each able to leave the pad
 * ALONE and reach only the on-chip peripherals (DAC_MCR.MODEx, 16.7.16).
 * On this family "on-chip peripherals" means THE COMPARATORS: figure 36
 * of the ADC chapter shows no DAC among the ADC's nineteen inputs, so
 * the DAC reaches the ADC through the PAD and nothing else, while it
 * reaches a comparator's inverting input with no pad at all. That
 * asymmetry is the shape of every wireless experiment on this silicon
 * and it is stated here because neither chapter says it.
 *
 * FOUR FACTS THAT SHAPE THIS FILE.
 *
 * 1. CHANNELS ARE 0-BASED HERE AND 1-BASED IN THE MANUAL. `ch = 0` is
 *    DAC_CH1 / DAC1_OUT1 / PA4 and `ch = 1` is DAC_CH2 / DAC1_OUT2 /
 *    PA5. Every channel verb of this stratum counts from zero
 *    (stm32g0/tim.hpp says so for the timers) and one file counting
 *    differently would be worse than one offset stated loudly.
 *
 * 2. THE DATA REGISTER IS NOT THE OUTPUT REGISTER. A write lands in
 *    DAC_DHRx and reaches DAC_DORx one dac_pclk cycle later with no
 *    trigger, or THREE cycles after the trigger with one (16.4.5). So
 *    `output(ch)` reads DOR and `code(ch)` reads back what was asked
 *    for: a caller that confuses them measures its own write.
 *
 * 3. A TRIGGER IS AN EDGE AND THE FIRST DATUM MUST PRECEDE IT. 16.4.8:
 *    "the very first data has to be written to the DAC_DHRx before the
 *    first trigger event occurs" - which is why a DMA-fed stream on this
 *    converter starts by filling the holding register, exactly as the
 *    SAM's did, and why the underrun flag exists at all. DMAUDRx is
 *    write-1-to-clear and 16.4.8 says recovering from it means
 *    re-initializing the DMA channel and the converter both: `underrun()`
 *    reports, and the owner decides.
 *
 * 4. THE WAVE GENERATORS NEED A TRIGGER. WAVEx is "only used if TENx = 1"
 *    (16.7.1), so noise and triangle are refused without one rather than
 *    quietly doing nothing - the samc dac.hpp ruling on dithering,
 *    reached again from another chapter.
 *
 * The reference is stm32g0/vref.hpp's `Ref`: 16.4.6's transfer function
 * is VREF+ x DOR / 4096, the same rail the ADC measures against.
 *
 * ERRATA: ES0548 Rev 3 has NO item touching the DAC on either silicon
 * revision.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/vref.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// DAC_CR.TSELx - table 85's interconnect, by the code that selects it.
/// The gaps are real: trg4, trg7, trg9, trg10 and trg14/15 carry nothing
/// on this family, so the enum spells only what exists and
/// `dac_trigger_valid()` refuses the rest.
enum class DacTrigger : uint8_t {
    software = 0,
    tim1_trgo = 1,
    tim2_trgo = 2,
    tim3_trgo = 3,
    tim6_trgo = 5,
    tim7_trgo = 6,
    tim15_trgo = 8,
    lptim1_out = 11,
    lptim2_out = 12,
    exti9 = 13,
};

constexpr bool dac_trigger_valid(DacTrigger t) {
    switch (t) {
        case DacTrigger::software:
        case DacTrigger::tim1_trgo:
        case DacTrigger::tim2_trgo:
        case DacTrigger::tim3_trgo:
        case DacTrigger::tim6_trgo:
        case DacTrigger::tim7_trgo:
        case DacTrigger::tim15_trgo:
        case DacTrigger::lptim1_out:
        case DacTrigger::lptim2_out:
        case DacTrigger::exti9: return true;
        default: return false;
    }
}

/**
 * DAC_MCR.MODEx (16.7.16): where the channel's output goes and whether
 * the buffer drives it. The three questions the field answers are the
 * PAD, the ON-CHIP path, and the BUFFER, and the eight codes are their
 * combinations plus sample-and-hold.
 *
 * A buffered output cannot reach the rails (DS13560's buffer output
 * swing) and an unbuffered one cannot drive a load; on this board the
 * pad route feeds an ADC input of a few tens of kiloohms and the LED,
 * so `pin_and_internal_buffered` is the useful default and the
 * unbuffered codes are there for the comparator path, which draws
 * nothing.
 */
enum class DacMode : uint8_t {
    pin_buffered = 0,
    pin_and_internal_buffered = 1,
    pin_unbuffered = 2,
    internal_unbuffered = 3,
    sample_hold_pin_buffered = 4,
    sample_hold_pin_and_internal_buffered = 5,
    sample_hold_pin_and_internal_unbuffered = 6,
    sample_hold_internal_unbuffered = 7,
};

constexpr bool dac_mode_drives_pin(DacMode m) {
    return m != DacMode::internal_unbuffered && m != DacMode::sample_hold_internal_unbuffered;
}
constexpr bool dac_mode_buffered(DacMode m) {
    return m == DacMode::pin_buffered || m == DacMode::pin_and_internal_buffered ||
           m == DacMode::sample_hold_pin_buffered ||
           m == DacMode::sample_hold_pin_and_internal_buffered;
}
constexpr bool dac_mode_sample_hold(DacMode m) {
    return static_cast<uint8_t>(m) >= 4u;
}

/// DAC_CR.WAVEx (16.4.9, 16.4.10). Both generators need a hardware or
/// software trigger to advance.
enum class DacWave : uint8_t { none = 0, noise = 1, triangle = 2 };

/// DAC_CR.MAMPx: the LFSR mask, or the triangle's amplitude. The value
/// is 2^(code+1) - 1 in both readings (16.7.1), and codes above 11 all
/// mean the full 4095.
constexpr uint16_t dac_wave_amplitude(uint8_t mamp) {
    const uint8_t c = mamp > 11u ? 11u : mamp;
    return static_cast<uint16_t>((1u << (c + 1u)) - 1u);
}

/// What one channel is configured with.
struct DacChannelConfig {
    DacMode mode = DacMode::pin_and_internal_buffered;
    /// TENx + TSELx. `software` with `triggered` true is 16.4.7's
    /// SWTRIG; `triggered` false ignores the selection entirely and a
    /// write reaches the output on its own.
    bool triggered = false;
    DacTrigger trigger = DacTrigger::software;
    DacWave wave = DacWave::none;
    uint8_t amplitude = 0;          ///< MAMPx, 0..11
    bool dma = false;               ///< DMAENx
    bool underrun_interrupt = false;///< DMAUDRIEx
};

constexpr bool dac_channel_config_valid(const DacChannelConfig& c) {
    if (!dac_trigger_valid(c.trigger)) {
        return false;
    }
    if (c.amplitude > 11u) {
        return false;
    }
    if (c.wave != DacWave::none && !c.triggered) {
        return false;   // 16.7.1: WAVEx is only used with TENx set
    }
    return true;
}

/// DAC_SR's per-channel bits (16.7.14), by channel index.
struct DacFlag {
    static constexpr uint32_t underrun(uint8_t ch) {
        return ch == 0 ? DAC_SR_DMAUDR1 : DAC_SR_DMAUDR2;
    }
    static constexpr uint32_t calibration(uint8_t ch) {
        return ch == 0 ? DAC_SR_CAL_FLAG1 : DAC_SR_CAL_FLAG2;
    }
    /// BWSTx: a sample-and-hold register write is still crossing into
    /// the low-power clock domain.
    static constexpr uint32_t busy(uint8_t ch) {
        return ch == 0 ? DAC_SR_BWST1 : DAC_SR_BWST2;
    }
};

// =============================================================================
// The resource
// =============================================================================

class Dac {
public:
    static_assert(dac_present(),
                  "brio Dac: this device has no DAC (16.3: the G031/G041 have none, "
                  "and their device header declares no DAC1_BASE)");

    Dac() = delete;

    static constexpr uint8_t channels = dac_channels();

    /// The NVIC line - the underrun interrupt's only route, SHARED WITH
    /// TIM6 AND LPTIM1 (table 61), so a handler bound here is a
    /// dispatcher.
    static constexpr IRQn_Type irq() { return dac_irq(); }

    static DAC_TypeDef& regs() { return *reinterpret_cast<DAC_TypeDef*>(dac_base()); }

    /**
     * The DMAMUX request ids of the two channels (table 55 rows 8 and 9),
     * to be handed to a stm32g0/dma.hpp engine's arm(). They live here
     * rather than in the reserve for the reason this stratum settled with
     * the timers: no device header of this pack declares one, and a
     * peripheral owns its own vocabulary.
     */
    static constexpr uint8_t dma_request(uint8_t ch) {
        return ch == 0u ? 8u : (ch == 1u ? 9u : 0u);
    }

    /**
     * Where a DMA channel writes a sample. THE FORMAT IS THE CALLER'S
     * CHOICE and it decides the beat width: DHR12Rx wants a halfword,
     * DHR8Rx a byte, and a stream whose element type disagrees with the
     * register it was pointed at writes a number the converter never
     * meant (the samc campaign's 24-bit lesson, in a smaller key).
     */
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
    /// The DUAL holding register: one 32-bit write carries both channels,
    /// which is what makes a two-channel stream one DMA channel (16.4.14).
    static volatile void* data_address_dual_12r() { return &regs().DHR12RD; }

    // ---- the block -----------------------------------------------------------

    static void bus_clock(bool on) { Rcc::apb1_clock(dac_clock_mask(), on); }
    static bool bus_clock() { return Rcc::apb1_clock(dac_clock_mask()); }
    static void reset() { Rcc::apb1_reset(dac_reset_mask()); }

    /// Bus clock on, block reset, both channels off. The pads are NOT
    /// touched: which of them this program wants in analog mode is the
    /// caller's, and a channel in an internal-only mode wants none.
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

    /// The pad a channel drives, in the mode 16.4.2 and 7.3.1 both want:
    /// analog, which is also its reset state. Spelled as a verb so an
    /// application never names PA4 or PA5 itself.
    template <class P>
    static void claim_pad() { P::analog(); }
    template <class P>
    static void release_pad() { P::analog(); }

    // ---- per-channel configuration -------------------------------------------

    static constexpr bool channel_valid(uint8_t ch) { return ch < channels; }

    /**
     * Configure one channel and leave it DISABLED.
     *
     * 16.7.16: MODEx is writable only with ENx and CENx both clear, and
     * 16.7.1 says the same of TSELx - so this verb disables the channel
     * first, writes MCR, then CR. A caller that wants the channel running
     * calls `enable(ch, true)` after.
     */
    static bool configure(uint8_t ch, const DacChannelConfig& c) {
        if (!channel_valid(ch) || !dac_channel_config_valid(c)) {
            return false;
        }
        cfg_[ch] = c;
        const uint32_t shift = channel_shift(ch);
        // ENx and CENx down first: MODEx and TSELx are both protected by
        // them, and a write that lands nowhere is the trap 16.7.16 warns
        // about ("If EN1 = 1 or CEN1 = 1 the write operation is ignored").
        regs().CR = regs().CR & ~((DAC_CR_EN1 | DAC_CR_CEN1) << shift);
        regs().MCR = (regs().MCR & ~(DAC_MCR_MODE1_Msk << shift)) |
                     (static_cast<uint32_t>(c.mode) << (DAC_MCR_MODE1_Pos + shift));
        uint32_t cr = regs().CR & ~(channel_cr_mask() << shift);
        if (c.triggered) {
            cr |= DAC_CR_TEN1 << shift;
            cr |= static_cast<uint32_t>(c.trigger) << (DAC_CR_TSEL1_Pos + shift);
        }
        cr |= static_cast<uint32_t>(c.wave) << (DAC_CR_WAVE1_Pos + shift);
        cr |= static_cast<uint32_t>(c.amplitude) << (DAC_CR_MAMP1_Pos + shift);
        if (c.dma) cr |= DAC_CR_DMAEN1 << shift;
        if (c.underrun_interrupt) cr |= DAC_CR_DMAUDRIE1 << shift;
        regs().CR = cr;
        return true;
    }

    static const DacChannelConfig& config(uint8_t ch) { return cfg_[ch < channels ? ch : 0]; }

    /// ENx. 16.4.3: the channel is usable after tWAKEUP; the DIGITAL
    /// interface works with the bit clear, which is what lets a holding
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

    // ---- data ----------------------------------------------------------------

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
    /// four low bits of the converter are zero and the full scale is 255.
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

    /// Both channels in ONE store (DAC_DHR12RD, 16.4.14) - the register
    /// that makes a two-channel waveform one DMA request.
    static bool write_dual(uint16_t code0, uint16_t code1) {
        if (channels < 2u) {
            return false;
        }
        regs().DHR12RD = (static_cast<uint32_t>(code1 & 0x0FFFu) << 16) | (code0 & 0x0FFFu);
        return true;
    }

    /// What was ASKED for: the holding register, before any trigger.
    static uint16_t code(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint16_t>((ch == 0u ? regs().DHR12R1 : regs().DHR12R2) & 0x0FFFu);
    }

    /// What the converter is PRODUCING: DAC_DORx, which the holding
    /// register reaches one dac_pclk cycle later untriggered and three
    /// after a trigger (16.4.5). Not writable - 16.4.5's first sentence.
    static uint16_t output(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint16_t>((ch == 0u ? regs().DOR1 : regs().DOR2) & 0x0FFFu);
    }

    /// The converter's full scale, for util/analog.hpp's dac_code() /
    /// dac_mv(). Twelve bits always: the 8-bit format is a placement, not
    /// a resolution (16.4.4).
    static constexpr uint32_t steps = 4096;

    /// SWTRIGx (16.7.2), the software trigger. Self-clearing once the
    /// holding register has been taken.
    static bool trigger(uint8_t ch) {
        if (!channel_valid(ch)) {
            return false;
        }
        regs().SWTRIGR = ch == 0u ? DAC_SWTRIGR_SWTRIG1 : DAC_SWTRIGR_SWTRIG2;
        return true;
    }
    static void trigger_both() {
        regs().SWTRIGR = DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
    }

    // ---- sample and hold (16.4.11) -------------------------------------------

    /**
     * The three times of sample-and-hold mode, in cycles of dac_hold_ck -
     * which is LSI, selected in the RCC (table 85) and NOT this driver's
     * to turn on.
     *
     * AND WITH LSI STOPPED NOTHING SAYS SO. An earlier version of this
     * comment claimed such a caller "gets a channel that never samples";
     * measured (test_stm32_analog letter o, with LSIRDY read clear) it
     * gets the opposite - the pad carries the value to within a count,
     * exactly as the plain buffered mode does, the times land, and BWST
     * never stands. So the mode degrades to plain buffered IN SILENCE
     * and there is no bit an application can read to tell the two apart.
     * Whoever selects a sample-and-hold mode owns the LSI.
     *
     * Sample and hold are 10-bit fields, refresh is 8. The write crosses
     * into the low-power domain, so `busy(ch)` can stand until it lands
     * (16.7.14's BWSTx) - though on this silicon it was never observed
     * to, at either state of the clock.
     */
    static bool sample_hold_times(uint8_t ch, uint16_t sample, uint16_t hold,
                                  uint8_t refresh) {
        if (!channel_valid(ch) || sample > 0x3FFu || hold > 0x3FFu) {
            return false;
        }
        if (ch == 0u) {
            regs().SHSR1 = sample;
            regs().SHHR = (regs().SHHR & ~DAC_SHHR_THOLD1_Msk) | hold;
            regs().SHRR = (regs().SHRR & ~DAC_SHRR_TREFRESH1_Msk) | refresh;
        } else {
            regs().SHSR2 = sample;
            regs().SHHR = (regs().SHHR & ~DAC_SHHR_THOLD2_Msk) |
                          (static_cast<uint32_t>(hold) << DAC_SHHR_THOLD2_Pos);
            regs().SHRR = (regs().SHRR & ~DAC_SHRR_TREFRESH2_Msk) |
                          (static_cast<uint32_t>(refresh) << DAC_SHRR_TREFRESH2_Pos);
        }
        return true;
    }

    static bool busy(uint8_t ch) {
        return channel_valid(ch) && (regs().SR & DacFlag::busy(ch)) != 0u;
    }

    // ---- the offset calibration (16.4.12) ------------------------------------

    /// DAC_CCR.OTRIMx, loaded at reset with the factory trim. Writable,
    /// and the user procedure below is what a caller runs when VDDA,
    /// VREF+ or the temperature have moved away from the factory
    /// conditions.
    static uint8_t trim(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint8_t>((regs().CCR >> (ch == 0u ? DAC_CCR_OTRIM1_Pos
                                                             : DAC_CCR_OTRIM2_Pos)) &
                                    0x1Fu);
    }

    static bool set_trim(uint8_t ch, uint8_t value) {
        if (!channel_valid(ch) || value > 31u) {
            return false;
        }
        const uint32_t pos = ch == 0u ? DAC_CCR_OTRIM1_Pos : DAC_CCR_OTRIM2_Pos;
        regs().CCR = (regs().CCR & ~(0x1FUL << pos)) | (static_cast<uint32_t>(value) << pos);
        return true;
    }

    /**
     * CENx - calibration mode. 16.4.12: legal only with ENx clear and
     * only in a BUFFERED mode (in an unbuffered one it has no effect at
     * all, which is a silent nothing this verb refuses instead).
     */
    static bool calibration_mode(uint8_t ch, bool on) {
        if (!channel_valid(ch) || enabled(ch)) {
            return false;
        }
        if (on && !dac_mode_buffered(cfg_[ch].mode)) {
            return false;
        }
        const uint32_t bit = DAC_CR_CEN1 << channel_shift(ch);
        regs().CR = on ? (regs().CR | bit) : (regs().CR & ~bit);
        return true;
    }

    /// CAL_FLAGx: the buffer, acting as a comparator against VREF+/2,
    /// says the trim has crossed. Only meaningful in calibration mode,
    /// and only after the tTRIM delay 16.4.12 demands between the write
    /// and this read - which the CALLER spends, because only it knows
    /// what its clock is worth.
    static bool calibration_flag(uint8_t ch) {
        return channel_valid(ch) && (regs().SR & DacFlag::calibration(ch)) != 0u;
    }

    // ---- flags and interrupts -------------------------------------------------

    static uint32_t flags() { return regs().SR; }
    /// DAC_SR's flags are write-1-to-clear (16.7.14).
    static void clear_flags(uint32_t mask) { regs().SR = mask; }

    /**
     * DMAUDRx. 16.4.8: a trigger that arrives before the previous
     * request was served is LOST and the channel keeps converting the old
     * datum - so this is the count of samples a stream did not deliver,
     * and the chapter's own recovery is to clear the flag, stop the DMA
     * and re-initialize both. This driver reports; the owner decides,
     * because only it knows what its stream was.
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

    /**
     * The ISR BODY: the underrun flags this converter has ARMED, cleared
     * and handed back. The vector is TIM6's and LPTIM1's too, so it
     * answers 0 when the DAC did not speak.
     *
     * DMAUDRIEx sits in CR and the flag in SR, so the arming has to be
     * looked up rather than masked in one register - which is why this
     * body is three lines and not one.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t cr = regs().CR;
        uint32_t armed = 0;
        if ((cr & DAC_CR_DMAUDRIE1) != 0u) armed |= DAC_SR_DMAUDR1;
        if (channels > 1u && (cr & DAC_CR_DMAUDRIE2) != 0u) armed |= DAC_SR_DMAUDR2;
        const uint32_t hit = regs().SR & armed;
        if (hit != 0u) {
            regs().SR = hit;
        }
        return hit;
    }

private:
    /// The two channels' fields are the same bits sixteen apart in CR,
    /// MCR and SR alike - which is what makes every verb above one body.
    static constexpr uint32_t channel_shift(uint8_t ch) { return ch == 0u ? 0u : 16u; }

    static constexpr uint32_t channel_cr_mask() {
        return DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_Msk | DAC_CR_WAVE1_Msk |
               DAC_CR_MAMP1_Msk | DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 | DAC_CR_CEN1;
    }

    inline static DacChannelConfig cfg_[2]{};
};

} // namespace brio
