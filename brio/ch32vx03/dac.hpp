/*
 * dac.hpp
 *
 * The digital-to-analog converter of the CH32V303 (RM ch. 17): `Dac`, a
 * MONOSTATE resource, because a part has one DAC block or none - every
 * CH32V303 has one with two channels, and no CH32V203 has any (datasheet
 * table 2-1-1 against the CH32V203's table 2-1; device::has_dac). On a part
 * without it `Dac` is a compile error naming the reason.
 *
 * WHAT THIS CONVERTER IS. Two 12-bit channels, DAC1 on PA4 and DAC2 on PA5
 * (the datasheet's DAC1_OUT and DAC2_OUT, table 3-4), each with its own
 * enable, its own output buffer, its own trigger, its own wave generator
 * and its own DMA request - and no status register and no interrupt at
 * all: RM 17.4's register table has thirteen words and not one of them is
 * a flag. So there is no underrun here to report, no ISR body, and
 * nothing a program waits on but the output register itself.
 *
 * AND WHERE ITS OUTPUT GOES: TO A PAD, AND NOWHERE ELSE. No register of
 * this chapter routes a channel inside the chip, and PA4 and PA5 are also
 * the converter's inputs 4 and 5 (ADC_IN4 and ADC_IN5 on every package of
 * the datasheet's table) - so the ADC reads a DAC through the BOND PAD the
 * two share, which is what makes a DAC-into-ADC measurement possible with
 * no wire at all. 17.2.2.1 asks for the pad in analog mode BEFORE the
 * channel is enabled; `DacOut<ch>::claim()` is that store, and PA4/PA5 are
 * also SPI1's pads, so a program with both blocks decides which one owns
 * them.
 *
 * FIVE FACTS THAT SHAPE THIS FILE.
 *
 * 1. CHANNELS ARE NUMBERED AS THE MANUAL NUMBERS THEM: 1 and 2, the
 *    DAC1/DAC2 of the pad names and of the DMA table (11-3's "DAC1" on
 *    DMA2's channel 3, "DAC2" on its channel 4). The two channels' fields
 *    are the same bits sixteen apart in DAC_CTLR, and that is what makes
 *    every verb one body.
 *
 * 2. THE DATA REGISTER IS NOT THE OUTPUT REGISTER. A write lands in a
 *    holding register (nine spellings of it, 17.4.3 to 17.4.11: three per
 *    channel and three for both) and reaches DAC_DORx one PB1 cycle later
 *    with no trigger or after the software trigger, and THREE PB1 cycles
 *    after a hardware trigger (17.2.2.5, 17.2.3). So `output(ch)` reads DOR and
 *    `code(ch)` reads the 12-bit right-aligned holding register back, and
 *    DOR is READ-ONLY - there is no verb here that writes it.
 *
 * 3. THE BUFFER IS DISABLED BY A ONE. BOFFx is an output buffer DISABLE
 *    (17.4.1), so the config field is `buffered` and the inversion happens
 *    once, here. The datasheet (table 4-45) rates a load of 5 kOhm with
 *    the buffer and 15 kOhm without, and an unbuffered output reaching
 *    closer to the rails (0 to 3 mV against 0 to 8 mV at the bottom).
 *
 * 4. A TRIGGER IS AN EDGE, AND THE WAVE GENERATORS AND THE DMA NEED ONE.
 *    17.2.4's and 17.2.5's notes make TENx a precondition of both
 *    generators; 17.2.2.4 raises the DMA request on a trigger event and
 *    "software trigger not included" - so a wave without a trigger and a
 *    DMA without a hardware one are REFUSED here rather than left silent.
 *    TSELx cannot change with ENx set (17.2.2.5's note) and MAMPx must be
 *    set before the enable (17.2.4's note 2), which is why `configure()`
 *    drops the enable first and does not put it back.
 *
 * 5. A TRIGGER CODE NAMES A TIMER THE PART MAY NOT HAVE. Table 17-1's six
 *    timer codes are TIM6, TIM8, TIM7, TIM5, TIM2 and TIM4: every CH32V303
 *    has TIM2 and TIM4, the CH32V303RC and VC have the other four, and the
 *    128 KB CH32V303CB and RB have none of TIM5..TIM8 (table 2-1-1). A code
 *    whose timer is absent is refused - by a static_assert where the
 *    configuration is a constant, by false where it is not.
 *
 * THE DMA REQUEST IS ON DMA2: DAC1 on its channel 3, DAC2 on its channel 4
 * (table 11-3, ch32vx03/dma_engine.hpp), and `claim_stream<ch, Engine>()`
 * arms an engine on the holding register its element width selects and
 * sets DMAENx in one verb, refusing at compile time an engine on any other
 * slot - an engine there would wait for a request that never comes. The
 * DUAL registers take one word for both channels, and WCH's own dual
 * example feeds DAC_RD12BDHR from DAC1's request alone; `claim_dual_stream`
 * does the same.
 *
 * WHAT THIS REGISTER SET HAS NOT, against the STM32F4's DAC this chapter
 * is the relative of: the underrun flag and its interrupt (no DAC_SR), and
 * the TIM6-shared vector that would carry it. RM V2.3's 17.4.1 also closes
 * with a note that with BOTH channels enabled "the same wave is output to
 * 2 hardware channels according to the configuration of channel1"; RM V2.5
 * deletes that note and 17.3's eleven dual modes say the opposite, and the
 * silicon sides with the later text - on the CH32V303VCT6 each pad followed
 * its own channel's configuration with both enabled - so this driver
 * writes each channel's own fields (docs/ch32vx03/dac.md).
 *
 * THE REFERENCE is the converter's: 17.2.3 gives the output as VDDA x
 * DOR / 4096, and the datasheet (table 4-45) as VREF+, which is VDDA on
 * every package but the LQFP100, whose VREF+ pad the board ties
 * (ch32vx03/adc.hpp's `adc_reference`). util/analog.hpp's `dac_code()` and
 * `dac_mv()` are the arithmetic over it.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma_engine.hpp"
#include "ch32vx03/pin.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// The registers (RM table 17-2)
// =============================================================================

struct DacRegs {
    volatile uint32_t CTLR;       ///< 0x00 both channels' control, 16 bits apart
    volatile uint32_t SWTR;       ///< 0x04 the software triggers, write-only
    volatile uint32_t R12BDHR1;   ///< 0x08 channel 1, 12 bits right-aligned
    volatile uint32_t L12BDHR1;   ///< 0x0c channel 1, 12 bits left-aligned (15:4)
    volatile uint32_t R8BDHR1;    ///< 0x10 channel 1, 8 bits right-aligned
    volatile uint32_t R12BDHR2;   ///< 0x14 channel 2, 12 bits right-aligned
    volatile uint32_t L12BDHR2;   ///< 0x18 channel 2, 12 bits left-aligned
    volatile uint32_t R8BDHR2;    ///< 0x1c channel 2, 8 bits right-aligned
    volatile uint32_t RD12BDHR;   ///< 0x20 both, 12 bits right-aligned: 11:0 and 27:16
    volatile uint32_t LD12BDHR;   ///< 0x24 both, 12 bits left-aligned: 15:4 and 31:20
    volatile uint32_t RD8BDHR;    ///< 0x28 both, 8 bits: 7:0 and 15:8
    volatile uint32_t DOR1;       ///< 0x2c channel 1's output, read-only
    volatile uint32_t DOR2;       ///< 0x30 channel 2's output, read-only
};

inline DacRegs* dac_regs() { return reinterpret_cast<DacRegs*>(dac_base); }

// DAC_CTLR (17.4.1): channel 1's fields at 0, channel 2's at 16.
inline constexpr uint32_t dac_en          = 1UL << 0;
inline constexpr uint32_t dac_boff        = 1UL << 1;    ///< the buffer DISABLE
inline constexpr uint32_t dac_ten         = 1UL << 2;
inline constexpr uint32_t dac_tsel_shift  = 3;
inline constexpr uint32_t dac_tsel_mask   = 7UL << 3;
inline constexpr uint32_t dac_wave_shift  = 6;
inline constexpr uint32_t dac_wave_mask   = 3UL << 6;
inline constexpr uint32_t dac_mamp_shift  = 8;
inline constexpr uint32_t dac_mamp_mask   = 0xFUL << 8;
inline constexpr uint32_t dac_dmaen       = 1UL << 12;
inline constexpr uint32_t dac_channel_fields = 0x1FFFUL;   ///< bits 12:0 of a channel
// DAC_SWTR (17.4.2)
inline constexpr uint32_t dac_swtrig1 = 1UL << 0;
inline constexpr uint32_t dac_swtrig2 = 1UL << 1;

// =============================================================================
// The vocabulary
// =============================================================================

/// DAC_CTLR.TSELx - table 17-1's eight triggers, by the code that selects
/// them. All eight codes carry something; what varies across the parts is
/// whether the TIMER behind a code exists (`dac_trigger_valid()`).
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
        case DacTrigger::exti9:
        case DacTrigger::software: return 0;
    }
    return 0;
}

/// A trigger code is usable where the timer behind it exists on the part
/// (parts/<part>.hpp's masks, read through the DMA table's helper): TIM6,
/// TIM7 and TIM8 are the 256 KB CH32V303's, TIM5 too.
constexpr bool dac_trigger_valid(DacTrigger t) {
    const uint8_t tim = dac_trigger_timer(t);
    return tim == 0u || dma_timer_present(tim);
}

/// The EXTI line the seventh code names, and the one hardware trigger that
/// needs no timer. It reaches the converter through the line's EVENT
/// enable and not its interrupt enable (measured on the CH32V303VCT6, as
/// the ADC's code 110 does): a program configures the line's event.
inline constexpr uint8_t dac_exti_line = 9;

/// DAC_CTLR.WAVEx (17.2.4, 17.2.5). The manual spells the triangle's code
/// "1x", so two patterns mean it and this enum offers the canonical one.
enum class DacWave : uint8_t { none = 0, noise = 1, triangle = 2 };

/// DAC_CTLR.MAMPx: the LFSR's unmasked bits, or the triangle's amplitude -
/// the same number 2^(code + 1) - 1 in both readings, every code from 11 up
/// meaning the full 4095 (17.4.1).
constexpr uint16_t dac_wave_amplitude(uint8_t mamp) {
    const uint8_t c = mamp > 11u ? 11u : mamp;
    return static_cast<uint16_t>((1u << (c + 1u)) - 1u);
}

/// The LFSR's preloaded value (17.2.5) - what a noise channel adds on its
/// FIRST trigger, the one deterministic thing about the generator.
inline constexpr uint16_t dac_lfsr_preload = 0x0AAAu;

/// The three placements of a datum in a holding register (17.2.2.3). The
/// 8-bit one is a PLACEMENT and not a resolution: it lands in DHR[11:4].
enum class DacFormat : uint8_t { right12, left12, right8 };

/// How wide the element is that feeds a format - which is what a DMA
/// stream's element type must be (the element type IS the beat).
constexpr uint8_t dac_format_bytes(DacFormat f) { return f == DacFormat::right8 ? 1u : 2u; }
/// And the dual registers: a word for the two 12-bit placements, a
/// half-word for the two bytes.
constexpr uint8_t dac_dual_format_bytes(DacFormat f) { return f == DacFormat::right8 ? 2u : 4u; }

/// When a datum written to a holding register reaches DOR, in PB1 cycles
/// (17.2.2.5, 17.2.3): one with no trigger enabled and one after the
/// software trigger, three after a hardware trigger.
constexpr uint8_t dac_dor_latency_pb1_cycles(bool triggered, DacTrigger t) {
    return (!triggered || t == DacTrigger::software) ? 1u : 3u;
}

/// What one channel is configured with.
struct DacChannelConfig {
    /// BOFFx, inverted: true drives a load and stops short of the rails
    /// (datasheet table 4-45), false reaches them and drives less.
    bool buffered = true;
    /// TENx + TSELx. With `triggered` false the selection is not written
    /// and a datum reaches DOR one PB1 cycle after it is held.
    bool triggered = false;
    DacTrigger trigger = DacTrigger::software;
    DacWave wave = DacWave::none;
    uint8_t amplitude = 0;             ///< MAMPx, 0..15 (11 and up are all 4095)
    bool dma = false;                  ///< DMAENx
};

constexpr bool dac_channel_config_valid(const DacChannelConfig& c) {
    if (c.triggered && !dac_trigger_valid(c.trigger)) {
        return false;   // table 17-1's timer is not on this part
    }
    if (c.amplitude > 15u) {
        return false;   // MAMPx is four bits
    }
    if (c.wave != DacWave::none && !c.triggered) {
        return false;   // 17.2.4's note 1, 17.2.5's note: the generators need TENx
    }
    if (c.dma && (!c.triggered || c.trigger == DacTrigger::software)) {
        return false;   // 17.2.2.4: "software trigger not included"
    }
    return true;
}

/// The DMA slot of a channel's request: DMA2's channel 3 and 4 (table 11-3).
constexpr DmaSlot dac_dma_slot(uint8_t ch) {
    return ch == 1u   ? dma_request_channel(DmaRequest::dac1)
           : ch == 2u ? dma_request_channel(DmaRequest::dac2)
                      : DmaSlot{};
}

// =============================================================================
// The pads
// =============================================================================

/// The pad a channel drives - PA4 and PA5 on every package (datasheet table
/// 3-4) - and the converter input it is also: ADC_IN4 and ADC_IN5.
constexpr Pad dac_pad(uint8_t ch) {
    return ch == 1u ? Pad{'A', 4} : ch == 2u ? Pad{'A', 5} : Pad{};
}
constexpr uint8_t dac_adc_channel(uint8_t ch) { return ch == 1u ? 4u : ch == 2u ? 5u : 0xFFu; }

/**
 * DacOut<ch>: one channel's output pad. `claim()` puts it in analog mode,
 * which 17.2.2.1 asks for BEFORE the channel is enabled, and which is also
 * the mode the ADC reads it through.
 */
template <uint8_t ch>
struct DacOut {
    static_assert(device::has_dac,
                  "brio DacOut: this part has no DAC (every CH32V303 has one, no CH32V203 has "
                  "any - device::has_dac)");
    static_assert(ch == 1u || ch == 2u, "brio DacOut: the DAC has channels 1 and 2");
    static constexpr uint8_t channel = ch;
    static constexpr Pad pad = dac_pad(ch);
    static constexpr uint8_t adc_channel = dac_adc_channel(ch);
    using pin = Pin<pad.port, pad.pin>;

    static void claim() { pin::analog(); }
    static void release() { pin::analog(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * The converter, a monostate spelled `Dac`. It is a class template behind
 * that name only so that the refusal on a part without a DAC fires where
 * the converter is USED and not where this header is included: a program
 * for both series includes this file and drives the converter from a
 * template whose parameter is `device::has_dac`, spelling it
 * `DacUnit<has>` there so that a part without it discards the branch
 * unread.
 */
template <bool present = device::has_dac>
class DacUnit {
public:
    static_assert(present,
                  "brio Dac: this part has no DAC - the two-channel converter of RM ch. 17 is "
                  "every CH32V303's and no CH32V203's (datasheet tables 2-1-1 and 2-1, "
                  "device::has_dac)");

    DacUnit() = delete;

    static constexpr uint8_t channels = 2;
    /// The full scale, for util/analog.hpp's dac_code() / dac_mv(). Twelve
    /// bits always: the 8-bit format is a placement.
    static constexpr uint32_t steps = 4096;
    static constexpr uint16_t max_code = 4095;

    static DacRegs& regs() { return *dac_regs(); }

    // ---- the block ---------------------------------------------------------------

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb1, rcc_pb1_dac);
        } else {
            Rcc::disable(Bus::pb1, rcc_pb1_dac);
        }
    }
    static bool bus_clock() { return Rcc::enabled(Bus::pb1, rcc_pb1_dac); }
    static void reset() { Rcc::reset(Bus::pb1, rcc_pb1_dac); }

    /// Gate open, block reset, both channels off. The PADS are the
    /// caller's, through `DacOut<ch>::claim()`.
    static void init() {
        bus_clock(true);
        reset();
    }

    static void release() {
        regs().CTLR = 0;
        reset();
        bus_clock(false);
    }

    // ---- per-channel configuration --------------------------------------------------

    static constexpr bool channel_valid(uint8_t ch) { return ch == 1u || ch == 2u; }
    static constexpr uint32_t shift_of(uint8_t ch) { return ch == 2u ? 16u : 0u; }

    /**
     * Configure one channel and leave it DISABLED. TSELx is not writable
     * with ENx set and MAMPx must be written before the enable (17.2.2.5,
     * 17.2.4), so this drops the enable first and does not put it back:
     * the caller enables after, which is also where the channel's wake-up
     * time (datasheet table 4-45's tWAKEUP, 6.5 to 10 us) is spent.
     */
    static bool configure(uint8_t ch, const DacChannelConfig& c) {
        if (!channel_valid(ch) || !dac_channel_config_valid(c)) {
            return false;
        }
        const uint32_t shift = shift_of(ch);
        uint32_t ctlr = regs().CTLR & ~(dac_channel_fields << shift);
        regs().CTLR = ctlr;   // the enable dropped before TSEL and MAMP move
        if (!c.buffered) {
            ctlr |= dac_boff << shift;
        }
        if (c.triggered) {
            ctlr |= dac_ten << shift;
            ctlr |= static_cast<uint32_t>(c.trigger) << (dac_tsel_shift + shift);
        }
        ctlr |= static_cast<uint32_t>(c.wave) << (dac_wave_shift + shift);
        ctlr |= static_cast<uint32_t>(c.amplitude) << (dac_mamp_shift + shift);
        if (c.dma) {
            ctlr |= dac_dmaen << shift;
        }
        regs().CTLR = ctlr;
        return true;
    }

    /// The same where the configuration is a constant: a timer the part
    /// has not got, or a generator or a DMA without the trigger they need,
    /// is a compile error on the line that asked.
    template <uint8_t ch, DacChannelConfig c>
    static void configure() {
        static_assert(ch == 1u || ch == 2u, "brio Dac: the DAC has channels 1 and 2");
        static_assert(!c.triggered || dac_trigger_valid(c.trigger),
                      "brio Dac: this trigger names a timer this part has not got - table 17-1's "
                      "TIM5, TIM6, TIM7 and TIM8 are the 256 KB CH32V303's (datasheet table "
                      "2-1-1)");
        static_assert(c.amplitude <= 15u, "brio Dac: MAMPx is four bits");
        static_assert(c.wave == DacWave::none || c.triggered,
                      "brio Dac: the noise and triangle generators step on a trigger (17.2.4's "
                      "note 1, 17.2.5's note) - enable one");
        static_assert(!c.dma || (c.triggered && c.trigger != DacTrigger::software),
                      "brio Dac: the DMA request is raised by a HARDWARE trigger, the software "
                      "one not included (17.2.2.4)");
        (void)configure(ch, c);
    }

    /// The configuration as DAC_CTLR holds it.
    static DacChannelConfig configuration(uint8_t ch) {
        DacChannelConfig c{};
        if (!channel_valid(ch)) {
            return c;
        }
        const uint32_t v = regs().CTLR >> shift_of(ch);
        c.buffered = (v & dac_boff) == 0u;
        c.triggered = (v & dac_ten) != 0u;
        c.trigger = static_cast<DacTrigger>((v & dac_tsel_mask) >> dac_tsel_shift);
        const uint32_t w = (v & dac_wave_mask) >> dac_wave_shift;
        c.wave = w >= 2u ? DacWave::triangle : static_cast<DacWave>(w);
        c.amplitude = static_cast<uint8_t>((v & dac_mamp_mask) >> dac_mamp_shift);
        c.dma = (v & dac_dmaen) != 0u;
        return c;
    }

    /// ENx: the analog power to the channel (17.2.2.1), usable after the
    /// wake-up time. The holding registers work with the bit clear, which
    /// is what lets a datum be primed before the output is on.
    static bool enable(uint8_t ch, bool on) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t bit = dac_en << shift_of(ch);
        regs().CTLR = on ? (regs().CTLR | bit) : (regs().CTLR & ~bit);
        return true;
    }
    static bool enabled(uint8_t ch) {
        return channel_valid(ch) && (regs().CTLR & (dac_en << shift_of(ch))) != 0u;
    }

    /// The wave generator alone. 17.2.4 and 17.2.5: writing 00 resets the
    /// generator (the triangle's counter, the LFSR back to its preload); a
    /// generator is refused on a channel with no trigger enabled.
    static bool wave(uint8_t ch, DacWave w) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t shift = shift_of(ch);
        if (w != DacWave::none && (regs().CTLR & (dac_ten << shift)) == 0u) {
            return false;
        }
        regs().CTLR = (regs().CTLR & ~(dac_wave_mask << shift)) |
                      (static_cast<uint32_t>(w) << (dac_wave_shift + shift));
        return true;
    }
    static DacWave wave(uint8_t ch) { return configuration(ch).wave; }

    /// DMAENx on its own: the request raised by each hardware trigger.
    static bool dma(uint8_t ch, bool on) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t bit = dac_dmaen << shift_of(ch);
        regs().CTLR = on ? (regs().CTLR | bit) : (regs().CTLR & ~bit);
        return true;
    }
    static bool dma(uint8_t ch) {
        return channel_valid(ch) && (regs().CTLR & (dac_dmaen << shift_of(ch))) != 0u;
    }

    // ---- data ------------------------------------------------------------------------

    /// 12 bits right-aligned (DAC_R12BDHRx), the natural format.
    static bool write(uint8_t ch, uint16_t code) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t v = code & 0x0FFFu;
        if (ch == 1u) {
            regs().R12BDHR1 = v;
        } else {
            regs().R12BDHR2 = v;
        }
        return true;
    }

    /// 12 bits LEFT-aligned (DAC_L12BDHRx): the datum in bits 15:4, which is
    /// what a 16-bit sample stream already looks like.
    static bool write_left(uint8_t ch, uint16_t value) {
        if (!channel_valid(ch)) {
            return false;
        }
        const uint32_t v = value & 0xFFF0u;
        if (ch == 1u) {
            regs().L12BDHR1 = v;
        } else {
            regs().L12BDHR2 = v;
        }
        return true;
    }

    /// 8 bits right-aligned (DAC_R8BDHRx), landing in DOR[11:4]: the four
    /// low bits of the converter zero, 256 steps of sixteen.
    static bool write8(uint8_t ch, uint8_t code) {
        if (!channel_valid(ch)) {
            return false;
        }
        if (ch == 1u) {
            regs().R8BDHR1 = code;
        } else {
            regs().R8BDHR2 = code;
        }
        return true;
    }

    /// Both channels in ONE store (DAC_RD12BDHR, 17.3): channel 1 in 11:0
    /// and channel 2 in 27:16.
    static void write_dual(uint16_t code1, uint16_t code2) {
        regs().RD12BDHR = (static_cast<uint32_t>(code2 & 0x0FFFu) << 16) | (code1 & 0x0FFFu);
    }
    /// DAC_LD12BDHR: the halves at 15:4 and 31:20, NOT sixteen apart like
    /// every other pair of this block.
    static void write_dual_left(uint16_t value1, uint16_t value2) {
        regs().LD12BDHR = (static_cast<uint32_t>(value2 & 0xFFF0u) << 16) | (value1 & 0xFFF0u);
    }
    /// DAC_RD8BDHR: the two bytes of one half-word.
    static void write_dual8(uint8_t code1, uint8_t code2) {
        regs().RD8BDHR = (static_cast<uint32_t>(code2) << 8) | code1;
    }

    /// What was ASKED for: the channel's 12-bit right-aligned holding
    /// register, before any trigger. The other formats write the same
    /// internal register, so a datum written 8-bit or left-aligned reads
    /// back here shifted into place.
    static uint16_t code(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint16_t>((ch == 1u ? regs().R12BDHR1 : regs().R12BDHR2) & 0x0FFFu);
    }

    /// What the converter is PRODUCING: DAC_DORx, read-only. With a wave
    /// generator running this is the holding register PLUS the generator's
    /// value, which is how a program watches an LFSR or a triangle without
    /// a meter.
    static uint16_t output(uint8_t ch) {
        if (!channel_valid(ch)) {
            return 0;
        }
        return static_cast<uint16_t>((ch == 1u ? regs().DOR1 : regs().DOR2) & 0x0FFFu);
    }

    /// SWTRIGx (17.4.2): write-only, cleared by the hardware one PB1 cycle
    /// later once the holding register has been taken.
    static bool software_trigger(uint8_t ch) {
        if (!channel_valid(ch)) {
            return false;
        }
        regs().SWTR = ch == 1u ? dac_swtrig1 : dac_swtrig2;
        return true;
    }
    /// Both at once - 17.3.6's simultaneous software start.
    static void software_trigger_both() { regs().SWTR = dac_swtrig1 | dac_swtrig2; }

    // ---- the DMA -----------------------------------------------------------------------

    /// The register a DMA stream writes for one channel in one format. THE
    /// FORMAT DECIDES THE BEAT: the 12-bit registers want a half-word, the
    /// 8-bit one a byte.
    static volatile void* data_address(uint8_t ch, DacFormat f) {
        DacRegs& r = regs();
        if (ch == 2u) {
            return f == DacFormat::right12   ? static_cast<volatile void*>(&r.R12BDHR2)
                   : f == DacFormat::left12  ? static_cast<volatile void*>(&r.L12BDHR2)
                                             : static_cast<volatile void*>(&r.R8BDHR2);
        }
        return f == DacFormat::right12   ? static_cast<volatile void*>(&r.R12BDHR1)
               : f == DacFormat::left12  ? static_cast<volatile void*>(&r.L12BDHR1)
                                         : static_cast<volatile void*>(&r.R8BDHR1);
    }
    /// The dual registers: a word for the two 12-bit placements, a
    /// half-word for the two bytes.
    static volatile void* dual_data_address(DacFormat f) {
        DacRegs& r = regs();
        return f == DacFormat::right12   ? static_cast<volatile void*>(&r.RD12BDHR)
               : f == DacFormat::left12  ? static_cast<volatile void*>(&r.LD12BDHR)
                                         : static_cast<volatile void*>(&r.RD8BDHR);
    }

    static constexpr DmaSlot dma_slot(uint8_t ch) { return dac_dma_slot(ch); }

    /**
     * Hand channel `ch`'s request to a DMA engine: the engine armed on the
     * holding register of `format` and DMAENx set - the two things every
     * stream user must do, as one verb. The channel's configuration must
     * already name a hardware trigger (17.2.2.4), which `configure()` has
     * refused to leave out.
     *
     * REFUSED AT COMPILE TIME for an engine on the wrong slot - DAC1's
     * request is DMA2's channel 3 and DAC2's its channel 4 (table 11-3), and
     * an engine anywhere else would wait for a request that never comes -
     * and for an element whose width is not the format's beat. Templated on
     * the engine so that a program with no stream never includes
     * ch32vx03/dma.hpp.
     */
    template <uint8_t ch, typename Engine, DacFormat format = DacFormat::right12>
    static void claim_stream() {
        static_assert(ch == 1u || ch == 2u, "brio Dac: the DAC has channels 1 and 2");
        static_assert(Engine::present, "brio Dac: an empty engine slot cannot carry a stream");
        static_assert(dma_engine_slot<Engine>() == dac_dma_slot(ch),
                      "brio Dac: the CHANNEL IS THE REQUEST (RM table 11-3) - DAC1 raises its "
                      "request on DMA2's channel 3 and DAC2 on DMA2's channel 4, and an engine "
                      "anywhere else would never see a trigger "
                      "(DmaRequestOf<DmaRequest::dac1> is how to spell it)");
        static_assert(sizeof(typename Engine::element) == dac_format_bytes(format),
                      "brio Dac: the element type is the beat - a 12-bit holding register takes "
                      "a half-word and the 8-bit one a byte");
        Engine::arm(data_address(ch, format));
        (void)dma(ch, true);
    }

    /**
     * Both channels from ONE stream: DAC1's request feeding a dual holding
     * register, a word (or, for the 8-bit pair, a half-word) a trigger -
     * the arrangement of WCH's own dual example. Both channels should take
     * the same trigger; only DAC1's DMAENx is set, so DAC2's request is
     * never raised and no engine waits on it.
     */
    template <typename Engine, DacFormat format = DacFormat::right12>
    static void claim_dual_stream() {
        static_assert(Engine::present, "brio Dac: an empty engine slot cannot carry a stream");
        static_assert(dma_engine_slot<Engine>() == dac_dma_slot(1),
                      "brio Dac: a dual stream rides DAC1's request, DMA2's channel 3 (RM table "
                      "11-3)");
        static_assert(sizeof(typename Engine::element) == dac_dual_format_bytes(format),
                      "brio Dac: a dual 12-bit holding register takes a word, the dual 8-bit one "
                      "a half-word");
        Engine::arm(dual_data_address(format));
        (void)dma(1, true);
    }

    // ---- the arithmetic ------------------------------------------------------------------

    /// util/analog.hpp's dac_code()/dac_mv() over this converter's full
    /// scale and a reference in millivolts - the caller's, measured
    /// through the ADC's VREFINT where it matters.
    static constexpr uint16_t code_for_mv(uint16_t mv, uint16_t ref_mv_) {
        return dac_code(mv, steps, ref_mv_);
    }
    static constexpr uint16_t mv_for_code(uint16_t code, uint16_t ref_mv_) {
        return dac_mv(code, steps, ref_mv_);
    }
};

/// The DAC of this part.
using Dac = DacUnit<>;

// =============================================================================
// The chapter, pinned at compile time
// =============================================================================

static_assert(sizeof(DacRegs) == 0x34);
static_assert(dac_base == 0x40007400UL);
static_assert(dac_wave_amplitude(0) == 1u && dac_wave_amplitude(3) == 15u);
static_assert(dac_wave_amplitude(10) == 2047u && dac_wave_amplitude(11) == 4095u);
static_assert(dac_wave_amplitude(15) == 4095u);
static_assert(dac_trigger_timer(DacTrigger::tim6_trgo) == 6 &&
              dac_trigger_timer(DacTrigger::tim8_trgo) == 8);
static_assert(dac_trigger_timer(DacTrigger::exti9) == 0 &&
              dac_trigger_timer(DacTrigger::software) == 0);
static_assert(dac_trigger_valid(DacTrigger::software) && dac_trigger_valid(DacTrigger::exti9));
static_assert(!dac_channel_config_valid(DacChannelConfig{.wave = DacWave::noise}));
static_assert(!dac_channel_config_valid(DacChannelConfig{.amplitude = 16}));
static_assert(!dac_channel_config_valid(
    DacChannelConfig{.triggered = true, .trigger = DacTrigger::software, .dma = true}));
static_assert(!dac_channel_config_valid(DacChannelConfig{.dma = true}));
static_assert(dac_channel_config_valid(
    DacChannelConfig{.triggered = true, .trigger = DacTrigger::software,
                     .wave = DacWave::triangle, .amplitude = 11}));
static_assert(dac_pad(1) == Pad{'A', 4} && dac_pad(2) == Pad{'A', 5});
static_assert(dac_adc_channel(1) == 4u && dac_adc_channel(2) == 5u);
static_assert(dac_format_bytes(DacFormat::right8) == 1u && dac_format_bytes(DacFormat::left12) == 2u);
static_assert(dac_dual_format_bytes(DacFormat::right12) == 4u);
static_assert(dac_dor_latency_pb1_cycles(false, DacTrigger::tim2_trgo) == 1u);
static_assert(dac_dor_latency_pb1_cycles(true, DacTrigger::software) == 1u);
static_assert(dac_dor_latency_pb1_cycles(true, DacTrigger::tim4_trgo) == 3u);

} // namespace brio
