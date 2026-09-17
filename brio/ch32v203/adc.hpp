/*
 * adc.hpp
 *
 * The analog-to-digital converters of the CH32V203 (RM ch. 12): TWO
 * 12-bit successive-approximation units over sixteen pads and two
 * internal sources, a regular group of up to sixteen conversions and an
 * injected group of four that preempts it, scan, continuous and
 * discontinuous modes, an analog watchdog, external triggers from the
 * timers and from an EXTI line, one DMA request, a calibration the
 * chapter asks for at every power-up - and the DUAL modes, in which
 * ADC1 leads and ADC2 follows. It is the STM32F1's ADC under WCH's
 * names, with two additions of WCH's own.
 *
 *   Adc<1|2>       the RESOURCE, one per converter: the gate, the
 *                  power-up, the calibration, the sample times, both
 *                  groups, the triggers, the watchdog, the flags, the
 *                  ISR body - and util/analog_sampler.hpp's converter
 *                  surface (select / start / selected / input_code).
 *
 *   AnalogIn<Pin>  a pad handed to its channel. The map is the family's
 *                  (ADC_IN0..7 are PA0..PA7, 8 and 9 are PB0 and PB1,
 *                  10..15 are PC0..PC5 - datasheet 3.2) and the PART
 *                  decides which of those pads its package bonds, which
 *                  is what `Pin` itself refuses.
 *
 *   AdcInput       the two internal sources: the temperature sensor on
 *                  channel 16 and VREFINT on channel 17, both woken by
 *                  ONE bit (CTLR2.TSVREFE) and both ADC1's alone.
 *
 * HOW MANY CONVERTERS, AND HOW MANY CHANNELS, IS THE PART'S (datasheet
 * table 2-1, `device::adc_count` and `device::adc_channel_count`): two
 * converters and nine or ten bonded channels up to the CH32V203C8, ONE
 * converter and sixteen channels on the CH32V203RB. `Adc<2>` does not
 * compile there, and neither does a dual mode.
 *
 * THE CLOCK IS THE ONE PLACE A LEGAL TREE LEAVES THIS PERIPHERAL OUT OF
 * SPECIFICATION. ADCCLK is PCLK2 divided by 2, 4, 6 or 8 and nothing
 * else (RM 3.4.2's ADCPRE), the converter is rated at 14 MHz, and this
 * family's PCLK2 is HCLK undivided - so above 112 MHz of HCLK there is
 * no code that keeps the converter in range, and at the 144 MHz this
 * stratum's own console runs at the slowest divider still gives 18 MHz.
 * ch32v203/clock.hpp programs ADCPRE in `init()` and publishes
 * `Clock::adc_hz` and `Clock::adc_in_spec`; THIS DRIVER REFUSES A CLOCK
 * THAT IS OUT OF SPECIFICATION AT COMPILE TIME - `Adc<n>::init(clock)`
 * static_asserts it, the F4's pattern of a ceiling the caller must
 * meet. A program that wants the converter picks a tree that keeps it:
 * the PLL on the HSI at 96 MHz gives ADCCLK 12 MHz.
 *
 * THE CALIBRATION, AND THE ORDER THAT IS NOT OBVIOUS. 12.2.2 asks for a
 * calibration at every power-up: RSTCAL until the hardware clears it,
 * then CAL until the hardware clears it, with the converter powered for
 * at least two ADCCLK cycles first; the code lands in RDATAR. And
 * CTLR1.BUFEN's own note adds the trap: setting TSVREFE (or the TKEY
 * enable) turns the input buffer ON and it cannot be turned off again,
 * so THE CALIBRATION MUST HAPPEN BEFORE THEM, WITH THE BUFFER OFF.
 * `init()` runs that order: gate, reset, control words with the buffer
 * and the gain still off, power-up, tSTAB, calibrate, and only then the
 * buffer, the gain and the internal sources.
 *
 * WHAT WCH ADDED TO THE F1's CHAPTER: an input BUFFER (CTLR1.BUFEN) for
 * sources above the impedance the sampling switch tolerates, and in
 * front of it a PROGRAMMABLE GAIN of 1, 4, 16 or 64 (CTLR1.PGA) that
 * amplifies a small signal into the converter's range; the buffer must
 * be on for the gain to mean anything. The TKEY bits (TKENABLE,
 * TKITUNE) of the same register belong to RM ch. 13 and are DECLINED
 * here - touch sensing is an application and not a driver - so nothing
 * in this file writes them, and a program that wants them is writing
 * its own chapter.
 *
 * WHAT BELONGS TO ANOTHER CLASS AND IS NOT HERE. The AUX register at
 * offset 0x54 (the short sample times 2.5..5.5 cycles) names
 * CH32F20x_D8, CH32F20x_D8C, CH32V30x_D8, CH32V30x_D8C and
 * CH32V31x_D8C in its own note - no CH32V20x, so no part of this
 * family: the map carries the word, no verb writes it. The trigger
 * code 110 is EXTI's line on this family and nothing else: the TIM8
 * alternative, and the four AFIO remap bits that would select it
 * (ADC1/ADC2_ETRGREG_RM and _ETRGINJ_RM, 10.2.11.8), name the same
 * other classes - which is why there is no remap verb in this file and
 * the trigger enumerators say `exti11` and `exti15`.
 *
 * ONLY ADC1 HAS A DMA REQUEST (12.2.7's note 2) and only ADC1 has
 * TSVREFE and the dual-mode field; the injected group takes no DMA on
 * either converter (12.2.2's note). ADC2's data reaches memory in a
 * dual mode alone, in the upper half of ADC1's RDATAR.
 *
 * THE FLAGS ARE WRITE-ZERO-TO-CLEAR (STATR's RW0, the timers' INTFR
 * discipline), and EOC is cleared by READING RDATAR as well.
 *
 * NO DMA-FED STREAM SLEEPS ON THIS FAMILY. In the Sleep of RM 2.4 no
 * bus master but the core gets a cycle (docs/ch32v203/dma.md), so an
 * ADC stream held by a DMA channel stalls for the whole sleep: a
 * program that runs one holds itself awake.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "ch32v203/clock.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/dma_engine.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "util/analog.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// The registers (RM tables 12-5 and 12-6)
// =============================================================================

/// One layout for both converters: the F1's block to RDATAR, the word
/// the map leaves at 0x50, and at 0x54 the AUX register whose own note
/// names other device classes (the file header) - carried so a reader
/// can hold table 12-5 beside this, written by nothing.
struct AdcRegs {
    volatile uint32_t STATR;      ///< 0x00 write zero to clear
    volatile uint32_t CTLR1;      ///< 0x04
    volatile uint32_t CTLR2;      ///< 0x08
    volatile uint32_t SAMPTR1;    ///< 0x0c SMP10..SMP17
    volatile uint32_t SAMPTR2;    ///< 0x10 SMP0..SMP9
    volatile uint32_t IOFR[4];    ///< 0x14..0x20 the injected offsets
    volatile uint32_t WDHTR;      ///< 0x24 the watchdog's high threshold
    volatile uint32_t WDLTR;      ///< 0x28 its low one
    volatile uint32_t RSQR1;      ///< 0x2c L and SQ13..SQ16
    volatile uint32_t RSQR2;      ///< 0x30 SQ7..SQ12
    volatile uint32_t RSQR3;      ///< 0x34 SQ1..SQ6
    volatile uint32_t ISQR;       ///< 0x38 JL and JSQ1..JSQ4
    volatile uint32_t IDATAR[4];  ///< 0x3c..0x48 the injected results
    volatile uint32_t RDATAR;     ///< 0x4c the regular result (and ADC2's, in dual mode)
    uint32_t RESERVED0;           ///< 0x50
    volatile uint32_t AUX;        ///< 0x54 another class's short sample times
};

/// ADC1 at 0x40012400, ADC2 a kilobyte above it.
constexpr uint32_t adc_base_for(uint8_t n) {
    return n == 1u ? pb2_base + 0x2400u : n == 2u ? pb2_base + 0x2800u : 0u;
}

inline AdcRegs* adc_regs(uint8_t n) { return reinterpret_cast<AdcRegs*>(adc_base_for(n)); }

// STATR (12.3.1) - every bit RW0
inline constexpr uint32_t adc_awd    = 1UL << 0;
inline constexpr uint32_t adc_eoc    = 1UL << 1;
inline constexpr uint32_t adc_jeoc   = 1UL << 2;
inline constexpr uint32_t adc_jstrt  = 1UL << 3;
inline constexpr uint32_t adc_strt   = 1UL << 4;
// CTLR1 (12.3.2)
inline constexpr uint32_t adc_awdch_mask    = 0x1FUL << 0;
inline constexpr uint32_t adc_eocie         = 1UL << 5;
inline constexpr uint32_t adc_awdie         = 1UL << 6;
inline constexpr uint32_t adc_jeocie        = 1UL << 7;
inline constexpr uint32_t adc_scan          = 1UL << 8;
inline constexpr uint32_t adc_awdsgl        = 1UL << 9;
inline constexpr uint32_t adc_jauto         = 1UL << 10;
inline constexpr uint32_t adc_discen        = 1UL << 11;
inline constexpr uint32_t adc_jdiscen       = 1UL << 12;
inline constexpr uint32_t adc_discnum_mask  = 7UL << 13;
inline constexpr uint32_t adc_discnum_shift = 13;
inline constexpr uint32_t adc_dualmod_mask  = 0xFUL << 16;
inline constexpr uint32_t adc_dualmod_shift = 16;
inline constexpr uint32_t adc_jawden        = 1UL << 22;
inline constexpr uint32_t adc_awden         = 1UL << 23;
inline constexpr uint32_t adc_tkenable      = 1UL << 24;   ///< RM ch. 13, declined (the file header)
inline constexpr uint32_t adc_tkitune       = 1UL << 25;   ///< the same
inline constexpr uint32_t adc_bufen         = 1UL << 26;
inline constexpr uint32_t adc_pga_mask      = 3UL << 27;
inline constexpr uint32_t adc_pga_shift     = 27;
// CTLR2 (12.3.3)
inline constexpr uint32_t adc_adon          = 1UL << 0;
inline constexpr uint32_t adc_cont          = 1UL << 1;
inline constexpr uint32_t adc_cal           = 1UL << 2;
inline constexpr uint32_t adc_rstcal        = 1UL << 3;
inline constexpr uint32_t adc_dma           = 1UL << 8;
inline constexpr uint32_t adc_align         = 1UL << 11;
inline constexpr uint32_t adc_jextsel_mask  = 7UL << 12;
inline constexpr uint32_t adc_jextsel_shift = 12;
inline constexpr uint32_t adc_jexttrig      = 1UL << 15;
inline constexpr uint32_t adc_extsel_mask   = 7UL << 17;
inline constexpr uint32_t adc_extsel_shift  = 17;
inline constexpr uint32_t adc_exttrig       = 1UL << 20;
inline constexpr uint32_t adc_jswstart      = 1UL << 21;
inline constexpr uint32_t adc_swstart       = 1UL << 22;
inline constexpr uint32_t adc_tsvrefe       = 1UL << 23;   ///< ADC1's alone

// =============================================================================
// The vocabulary
// =============================================================================

inline constexpr uint8_t adc_bits = 12;
inline constexpr uint32_t adc_steps = 1UL << adc_bits;   ///< util/analog.hpp's full scale
inline constexpr uint16_t adc_max_count = static_cast<uint16_t>(adc_steps - 1u);
/// Sixteen pads and two internal sources (12.2.2).
inline constexpr uint8_t adc_channels = 18;
inline constexpr uint8_t adc_pad_channels = 16;
inline constexpr uint8_t adc_temperature_channel = 16;
inline constexpr uint8_t adc_vrefint_channel = 17;
/// The longest regular sequence and the longest injected one.
inline constexpr uint8_t adc_regular_slots = 16;
inline constexpr uint8_t adc_injected_slots = 4;

/// Datasheet table 4-26: the internal reference, 1.17 to 1.23 V.
inline constexpr uint16_t adc_vrefint_mv = 1200;
/// Datasheet table 4-30, the temperature sensor: 1.40 V at 25 degrees
/// and a NEGATIVE coefficient of 4.3 mV per degree (3.8 to 4.7), which
/// is why the arithmetic below subtracts the reading from V25 and not
/// the other way round. The spread is why a reading is a temperature
/// CHANGE unless the part has been characterized: 12.2.6 says so, and
/// this family's electronic signature (RM ch. 31) holds the flash
/// capacity and the unique id and no calibration word, so there is
/// nothing here of the STM32F4's AdcFactory.
inline constexpr uint16_t adc_temperature_v25_mv = 1400;
inline constexpr uint16_t adc_temperature_slope_uv_per_c = 4300;
/// 12.2.6's recommended sampling time for the sensor.
inline constexpr uint32_t adc_temperature_sample_ns = 17'100;

/// util/analog.hpp's vocabulary on this target. No package of this
/// series brings out a VREF+ pad - the datasheet's pin tables have VDDA
/// and VSSA and nothing between them - so the converter's reference IS
/// the analog supply, and what that supply is in millivolts is the
/// BOARD's to state (the RP2040's and the STM32F4's arrangement).
/// `Adc<1>::vdda_mv()` measures it from VREFINT instead of assuming it.
enum class Ref : uint8_t { vdda };
constexpr uint16_t ref_mv(Ref, uint16_t vdda_mv = 3300) { return vdda_mv; }

/// The two internal sources, as TAGS (12.2.2). One bit wakes both.
enum class AdcInput : uint8_t {
    temperature = adc_temperature_channel,
    vrefint = adc_vrefint_channel,
};

/// SMPx[2:0] (12.3.4): the sampling time in ADCCLK cycles.
enum class AdcSampleTime : uint8_t {
    cycles1_5 = 0, cycles7_5 = 1, cycles13_5 = 2, cycles28_5 = 3,
    cycles41_5 = 4, cycles55_5 = 5, cycles71_5 = 6, cycles239_5 = 7,
};
inline constexpr AdcSampleTime adc_sample_shortest = AdcSampleTime::cycles1_5;
inline constexpr AdcSampleTime adc_sample_longest = AdcSampleTime::cycles239_5;

/// The sampling time in HALF ADCCLK cycles, so the .5 stays exact.
constexpr uint32_t adc_sample_half_cycles(AdcSampleTime t) {
    constexpr uint16_t table[8] = {3, 15, 27, 57, 83, 111, 143, 479};
    return table[static_cast<uint8_t>(t) & 7u];
}
/// tCONV = the sampling time + 11 ADCCLK cycles (12.2.2), in halves.
inline constexpr uint32_t adc_conversion_tail_half_cycles = 22;
constexpr uint32_t adc_conversion_half_cycles(AdcSampleTime t) {
    return adc_sample_half_cycles(t) + adc_conversion_tail_half_cycles;
}
/// The same in nanoseconds at a stated ADCCLK, for a program sizing a
/// sequence against a trigger rate.
constexpr uint32_t adc_conversion_ns(AdcSampleTime t, uint32_t adcclk_hz) {
    if (adcclk_hz == 0u) {
        return 0;
    }
    return static_cast<uint32_t>((adc_conversion_half_cycles(t) * 500'000'000ULL) / adcclk_hz);
}

/**
 * Datasheet table 4-28: the largest SOURCE IMPEDANCE each sampling time
 * can settle to within a quarter of an LSB, at the converter's full
 * 14 MHz. Zero where the table says "Invalid" - the number the formula
 * gives is beyond the 50 kOhm the converter is rated for at all (RAIN,
 * table 4-27), so the answer is the rating and not the settling.
 *
 * It is the one piece of the analog chapter a digital program can act
 * on: an internal pull is 30 to 50 kOhm (datasheet table 4-19), so a
 * pad held by its own pull is a 40 kOhm source and only the longest
 * sampling times read it true.
 */
constexpr uint32_t adc_max_source_ohms(AdcSampleTime t) {
    constexpr uint32_t table[8] = {400, 5900, 11400, 25200, 37200, 50000, 0, 0};
    return table[static_cast<uint8_t>(t) & 7u];
}

/// CTLR1.PGA (12.3.2): the gain in front of the converter. It needs the
/// input buffer (BUFEN), which is why AdcConfig refuses a gain without
/// one.
enum class AdcGain : uint8_t { x1 = 0, x4 = 1, x16 = 2, x64 = 3 };
constexpr uint8_t adc_gain_factor(AdcGain g) {
    return g == AdcGain::x1 ? 1u : g == AdcGain::x4 ? 4u : g == AdcGain::x16 ? 16u : 64u;
}

/// Table 12-1, the regular group's triggers (CTLR2.EXTSEL). Code 110 is
/// EXTI's line 11 on this family and nothing else (the file header).
enum class AdcTrigger : uint8_t {
    tim1_cc1 = 0, tim1_cc2 = 1, tim1_cc3 = 2, tim2_cc2 = 3,
    tim3_trgo = 4, tim4_cc4 = 5, exti11 = 6, software = 7,
};
/// Table 12-2, the injected group's (CTLR2.JEXTSEL). Code 110 is EXTI's
/// line 15.
enum class AdcInjectedTrigger : uint8_t {
    tim1_trgo = 0, tim1_cc4 = 1, tim2_trgo = 2, tim2_cc1 = 3,
    tim3_cc4 = 4, tim4_trgo = 5, exti15 = 6, software = 7,
};
/// The EXTI lines those two codes name, for a program that has to
/// configure the line before the trigger can arrive.
inline constexpr uint8_t adc_regular_exti_line = 11;
inline constexpr uint8_t adc_injected_exti_line = 15;

/// CTLR1.DUALMOD (12.3.2), ADC1's field: ten of the sixteen codes are
/// modes, the rest reserved. ADC1 leads, ADC2 follows (12.2.7).
enum class AdcDualMode : uint8_t {
    independent = 0,
    regular_and_injected_simultaneous = 1,
    regular_simultaneous_and_alternate_trigger = 2,
    injected_simultaneous_and_fast_interleaved = 3,
    injected_simultaneous_and_slow_interleaved = 4,
    injected_simultaneous = 5,
    regular_simultaneous = 6,
    fast_interleaved = 7,
    slow_interleaved = 8,
    alternate_trigger = 9,
};
constexpr bool adc_dual_mode_valid(AdcDualMode m) { return static_cast<uint8_t>(m) <= 9u; }

struct AdcFlag {
    static constexpr uint32_t watchdog = adc_awd;
    static constexpr uint32_t converted = adc_eoc;
    static constexpr uint32_t injected = adc_jeoc;
    static constexpr uint32_t injected_started = adc_jstrt;
    static constexpr uint32_t started = adc_strt;
    static constexpr uint32_t all = adc_awd | adc_eoc | adc_jeoc | adc_jstrt | adc_strt;
};

/**
 * What init() writes. The PRESCALER IS NOT HERE: ADCPRE belongs to the
 * clock tree and ch32v203/clock.hpp's `Clock::init()` programs it, so
 * the converter asks the clock what it is fed rather than setting it.
 */
struct AdcConfig {
    bool left_aligned = false;         ///< CTLR2.ALIGN
    bool continuous = false;           ///< CTLR2.CONT
    bool scan = false;                 ///< CTLR1.SCAN: the whole group, conversion after conversion
    bool auto_injected = false;        ///< CTLR1.JAUTO: the injected group after the regular one
    bool dma = false;                  ///< CTLR2.DMA: a request per regular conversion (ADC1's alone)
    bool input_buffer = false;         ///< CTLR1.BUFEN
    AdcGain gain = AdcGain::x1;        ///< CTLR1.PGA, which needs the buffer
    /// Discontinuous mode: 0 = off, 1..8 = the short sequence's length
    /// (CTLR1.DISCEN + DISCNUM).
    uint8_t discontinuous = 0;
    bool injected_discontinuous = false;   ///< CTLR1.JDISCEN
    /// CTLR2.TSVREFE: the temperature sensor and VREFINT. ADC1's alone,
    /// and set AFTER the calibration, because it forces the buffer on
    /// (the file header).
    bool internal_sources = false;
};

constexpr bool adc_config_valid(const AdcConfig& c) {
    if (c.discontinuous > 8u) {
        return false;
    }
    if (c.discontinuous != 0u && c.injected_discontinuous) {
        return false;   // 12.2.4's note 3: one group or the other
    }
    if (c.auto_injected && (c.discontinuous != 0u || c.injected_discontinuous)) {
        return false;   // 12.2.4's note 2
    }
    if (c.gain != AdcGain::x1 && !c.input_buffer) {
        return false;   // 12.3.2's PGA note: the gain wants the buffer
    }
    return true;
}

/**
 * The analog watchdog (12.2.5). Table 12-4 is four bits in one picture:
 * AWDEN and JAWDEN say WHICH GROUPS are guarded, AWDSGL and AWDCH
 * narrow that to one channel.
 */
struct AdcWatchdogConfig {
    uint16_t low = 0;
    uint16_t high = adc_max_count;
    /// One channel (AWDSGL + AWDCH), or every channel of the groups below.
    std::optional<uint8_t> channel{};
    bool regular_group = true;    ///< AWDEN
    bool injected_group = false;  ///< JAWDEN
    bool interrupt = false;       ///< AWDIE - and 12.3.2's note: in scan mode it ABORTS the scan
};

constexpr bool adc_watchdog_config_valid(const AdcWatchdogConfig& w) {
    return w.low <= adc_max_count && w.high <= adc_max_count && w.low <= w.high &&
           (!w.channel || *w.channel < adc_channels);
}

/**
 * The pads and their channels (datasheet 3.2's pin tables): the same
 * sixteen on every part of the series, with the PACKAGE deciding which
 * of them exist - `Pin` refuses an unbonded pad before this map is even
 * asked, so nothing here repeats the part table.
 */
constexpr uint8_t adc_channel_of(Pad p) {
    if (p.port == 'A' && p.pin <= 7u) { return p.pin; }               // ADC_IN0..7
    if (p.port == 'B' && p.pin == 0u) { return 8; }
    if (p.port == 'B' && p.pin == 1u) { return 9; }
    if (p.port == 'C' && p.pin <= 5u) { return static_cast<uint8_t>(10u + p.pin); }   // ADC_IN10..15
    return 0xFF;
}

/**
 * AnalogIn<Pin>: the claim that this pad is an analog input - refused
 * at compile time for a pad that is not one. `claim()` puts the pad in
 * analog mode, which on this family is CNF 00 MODE 00: the input
 * driver off AND THE PULL GONE WITH IT (RM 10.2.7), so a program that
 * wants a pad's own pull as its analog source keeps the pad a pulled
 * INPUT and never calls this.
 */
template <class P>
struct AnalogIn {
    static_assert(adc_channel_of(P::pad) != 0xFFu,
                  "brio AnalogIn: this pad is not an ADC input on the CH32V203 (PA0..PA7 are "
                  "channels 0..7, PB0 and PB1 are 8 and 9, PC0..PC5 are 10..15)");
    using pin = P;
    static constexpr uint8_t channel = adc_channel_of(P::pad);

    static void claim() { P::analog(); }
    static void release() { P::analog(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Adc<n>: one converter.
 *
 *   using Vin = brio::AnalogIn<brio::Pin<'A', 1>>;   // channel 1
 *   Vin::claim();
 *   brio::Adc<1>::init(clock, {.internal_sources = true});
 *   brio::Adc<1>::sample_time_all(brio::adc_sample_longest);
 *   brio::Adc<1>::select(Vin{});
 *   const uint16_t counts = brio::Adc<1>::read();
 *
 * util/analog_sampler.hpp drives `select()`, `start()`, `selected()`
 * and `input_code()`; the rest is the chapter's.
 */
template <uint8_t n>
class Adc {
public:
    Adc() = delete;

    static_assert(n == 1u || n == 2u,
                  "brio Adc: this family has ADC1 and ADC2 and no third converter");
    static_assert(n <= device::adc_count,
                  "brio Adc: this part has one converter (datasheet table 2-1's "
                  "channel@unit count - the 128 KB part trades the second unit for six "
                  "more channels), so only Adc<1> exists here");

    static constexpr uint8_t instance = n;
    static constexpr uint8_t channels = adc_channels;
    static constexpr uint32_t steps = adc_steps;

    /// What this converter alone can do. Only ADC1 has a DMA request
    /// (12.2.7's note 2), the internal sources (TSVREFE is "only applied
    /// for ADC1") and the dual-mode field (reserved in ADC2's CTLR1).
    static constexpr bool has_dma = (n == 1u);
    static constexpr bool has_internal_sources = (n == 1u);
    static constexpr bool has_dual_mode = (n == 1u) && (device::adc_count >= 2u);

    /// ONE VECTOR SERVES BOTH CONVERTERS (table 9-2's entry 34), so a
    /// handler that owns both reads both status registers.
    static constexpr Irq irq() { return Irq::adc1_2; }
    /// The channel the regular group's request is wired to, asked of
    /// the one place in this stratum that writes table 11-5's rows
    /// (ch32v203/dma_engine.hpp) rather than repeated here.
    static constexpr uint8_t dma_channel = DmaRequestOf<DmaRequest::adc1>::channel;

    static AdcRegs& regs() { return *adc_regs(n); }
    /// The register a DMA engine reads: the regular data register, whose
    /// upper half carries ADC2's conversion in a dual mode.
    static volatile void* data_address() { return &regs().RDATAR; }

    static constexpr uint32_t gate_bit = (n == 1u) ? rcc_pb2_adc1 : rcc_pb2_adc2;

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb2, gate_bit);
        } else {
            Rcc::disable(Bus::pb2, gate_bit);
        }
    }
    static bool bus_clock() { return Rcc::enabled(Bus::pb2, gate_bit); }
    static void reset() { Rcc::reset(Bus::pb2, gate_bit); }

    /**
     * Bring the converter up. The order is 12.2.2's, and the one step
     * that is not obvious is the CALIBRATION's place in it: the input
     * buffer and the internal sources are written AFTER it, because
     * TSVREFE forces the buffer on and a calibration wants it off
     * (12.3.2's BUFEN note).
     *
     * False for a configuration the chapter refuses, for a request no
     * instance can serve (a DMA or internal sources on ADC2), or for a
     * calibration that never completed.
     */
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& c = {}) {
        static_assert(Clock::is_static,
                      "brio Adc: every sampling time and every conversion this driver reports "
                      "is in ADCCLK cycles, and ADCCLK follows PCLK2 - a DynamicClock would "
                      "move them all, so the converter takes a static clock");
        static_assert(Clock::adc_in_spec,
                      "brio Adc: this clock leaves the converter OUT OF SPECIFICATION - "
                      "ADCCLK is PCLK2 divided by 2, 4, 6 or 8 (RM 3.4.2) and the part is "
                      "rated at 14 MHz, so HCLK above 112 MHz has no divider that keeps it. "
                      "Clock::adc_hz says what this tree would feed it; the PLL on the HSI "
                      "at 96 MHz gives 12 MHz");
        (void)clock;
        if (!adc_config_valid(c)) {
            return false;
        }
        if constexpr (!has_dma) {
            if (c.dma) {
                return false;
            }
        }
        if constexpr (!has_internal_sources) {
            if (c.internal_sources) {
                return false;
            }
        }
        Pfic::disable(irq());
        bus_clock(true);
        reset();
        cfg_ = c;

        AdcRegs& r = regs();
        uint32_t c1 = 0;
        if (c.scan) { c1 |= adc_scan; }
        if (c.auto_injected) { c1 |= adc_jauto; }
        if (c.discontinuous != 0u) {
            c1 |= adc_discen |
                  (static_cast<uint32_t>(c.discontinuous - 1u) << adc_discnum_shift);
        }
        if (c.injected_discontinuous) { c1 |= adc_jdiscen; }
        r.CTLR1 = c1;   // the buffer and the gain stay off for the calibration

        // Both software triggers armed: SWSTART only starts a regular
        // conversion with EXTSEL = 111 and EXTTRIG set, so `start()` is
        // one spelling whatever the group.
        uint32_t c2 = adc_exttrig | adc_extsel_mask | adc_jexttrig | adc_jextsel_mask;
        if (c.left_aligned) { c2 |= adc_align; }
        if (c.continuous) { c2 |= adc_cont; }
        if (c.dma) { c2 |= adc_dma; }
        r.CTLR2 = c2;

        // One conversion of channel 0 until a select().
        r.RSQR1 = 0;
        r.RSQR2 = 0;
        r.RSQR3 = 0;
        selected_ = 0;

        r.CTLR2 = r.CTLR2 | adc_adon;   // the wake-up write
        stab_spin();
        if (!calibrate()) {
            return false;
        }

        if (c.input_buffer) {
            r.CTLR1 = r.CTLR1 | adc_bufen |
                      (static_cast<uint32_t>(c.gain) << adc_pga_shift);
        }
        if constexpr (has_internal_sources) {
            if (c.internal_sources) {
                r.CTLR2 = r.CTLR2 | adc_tsvrefe;
                stab_spin();
            }
        }
        r.STATR = 0;
        return true;
    }

    static void release() {
        Pfic::disable(irq());
        regs().CTLR2 = regs().CTLR2 & ~adc_adon;
        reset();
        bus_clock(false);
    }

    /**
     * THE CALIBRATION (12.2.2, step 4): RSTCAL until the hardware clears
     * it, then CAL until the hardware clears it, the converter powered
     * for at least two ADCCLK cycles first. The code lands in RDATAR,
     * which `calibration_code()` reads. False when either bit never
     * cleared.
     */
    static bool calibrate(uint32_t spins = 0x100000UL) {
        AdcRegs& r = regs();
        r.CTLR2 = r.CTLR2 | adc_rstcal;
        uint32_t k = spins;
        while ((r.CTLR2 & adc_rstcal) != 0u && k-- != 0u) {
        }
        if ((r.CTLR2 & adc_rstcal) != 0u) {
            return false;
        }
        r.CTLR2 = r.CTLR2 | adc_cal;
        k = spins;
        while ((r.CTLR2 & adc_cal) != 0u && k-- != 0u) {
        }
        return (r.CTLR2 & adc_cal) == 0u;
    }
    /// What the last calibration left in the data register (12.2.2).
    static uint16_t calibration_code() { return static_cast<uint16_t>(regs().RDATAR & 0xFFFFu); }
    static bool calibrating() { return (regs().CTLR2 & (adc_cal | adc_rstcal)) != 0u; }

    static bool powered() { return (regs().CTLR2 & adc_adon) != 0u; }
    /// ADON: the first write wakes the converter (tSTAB, 1 us by the
    /// datasheet), a second one with nothing else changing would START a
    /// conversion - which is why every other verb here writes CTLR2 as a
    /// read-modify-write and the software trigger is SWSTART.
    static void power(bool on) {
        AdcRegs& r = regs();
        if (on) {
            if (!powered()) {
                r.CTLR2 = r.CTLR2 | adc_adon;
                stab_spin();
            }
        } else {
            r.CTLR2 = r.CTLR2 & ~adc_adon;
        }
    }

    // ---- the sampling times ---------------------------------------------------

    /// SMPx of one channel: SAMPTR2 holds 0..9, SAMPTR1 holds 10..17.
    static bool sample_time(uint8_t ch, AdcSampleTime t) {
        if (ch >= channels) {
            return false;
        }
        AdcRegs& r = regs();
        if (ch < 10u) {
            const uint32_t shift = 3u * ch;
            r.SAMPTR2 = (r.SAMPTR2 & ~(7UL << shift)) | (static_cast<uint32_t>(t) << shift);
        } else {
            const uint32_t shift = 3u * (ch - 10u);
            r.SAMPTR1 = (r.SAMPTR1 & ~(7UL << shift)) | (static_cast<uint32_t>(t) << shift);
        }
        return true;
    }
    static AdcSampleTime sample_time(uint8_t ch) {
        const AdcRegs& r = regs();
        return ch < 10u
                   ? static_cast<AdcSampleTime>((r.SAMPTR2 >> (3u * ch)) & 7u)
                   : static_cast<AdcSampleTime>((r.SAMPTR1 >> (3u * (ch - 10u))) & 7u);
    }
    static void sample_time_all(AdcSampleTime t) {
        uint32_t v = 0;
        for (uint8_t i = 0; i < 10u; ++i) {
            v |= static_cast<uint32_t>(t) << (3u * i);
        }
        regs().SAMPTR2 = v;
        v = 0;
        for (uint8_t i = 0; i < 8u; ++i) {
            v |= static_cast<uint32_t>(t) << (3u * i);
        }
        regs().SAMPTR1 = v;
    }
    /// tCONV of channel `ch` as the registers stand, in half ADCCLK cycles.
    static uint32_t conversion_half_cycles(uint8_t ch) {
        return adc_conversion_half_cycles(sample_time(ch));
    }

    // ---- the regular group ----------------------------------------------------

    /// The regular sequence: `count` channels in `order`, 1..16 (12.3.9
    /// to 12.3.11). L is count - 1.
    static bool sequence(const uint8_t* order, uint8_t count) {
        if (order == nullptr || count == 0u || count > adc_regular_slots) {
            return false;
        }
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] >= channels) {
                return false;
            }
        }
        uint32_t r1 = static_cast<uint32_t>(count - 1u) << 20;
        uint32_t r2 = 0;
        uint32_t r3 = 0;
        for (uint8_t i = 0; i < count; ++i) {
            const uint32_t ch = order[i];
            if (i < 6u) {
                r3 |= ch << (5u * i);
            } else if (i < 12u) {
                r2 |= ch << (5u * (i - 6u));
            } else {
                r1 |= ch << (5u * (i - 12u));
            }
        }
        AdcRegs& r = regs();
        r.RSQR3 = r3;
        r.RSQR2 = r2;
        r.RSQR1 = r1;
        selected_ = order[0];
        return true;
    }
    static uint8_t sequence_length() {
        return static_cast<uint8_t>(((regs().RSQR1 >> 20) & 0xFu) + 1u);
    }
    /// The channel in one slot of the sequence, 0-based.
    static uint8_t sequence_channel(uint8_t slot) {
        if (slot >= adc_regular_slots) {
            return 0xFF;
        }
        const AdcRegs& r = regs();
        const uint32_t word = slot < 6u ? r.RSQR3 : slot < 12u ? r.RSQR2 : r.RSQR1;
        const uint32_t shift = 5u * (slot < 6u ? slot : slot < 12u ? (slot - 6u) : (slot - 12u));
        return static_cast<uint8_t>((word >> shift) & 0x1Fu);
    }

    // ---- util/analog_sampler.hpp's converter surface ---------------------------

    template <class P>
    static constexpr uint8_t input_code(AnalogIn<P>) { return AnalogIn<P>::channel; }
    static constexpr uint8_t input_code(AdcInput in) { return static_cast<uint8_t>(in); }
    static constexpr uint8_t input_code(uint8_t ch) { return ch; }

    /// A one-conversion regular sequence on this channel.
    template <class P>
    static void select(AnalogIn<P>) { select_channel(AnalogIn<P>::channel); }
    static void select(AdcInput in) { select_channel(static_cast<uint8_t>(in)); }
    static void select_channel(uint8_t ch) {
        if (ch >= channels) {
            return;
        }
        AdcRegs& r = regs();
        r.RSQR1 = 0;
        r.RSQR3 = ch;
        selected_ = ch;
    }
    /// The first channel of the regular sequence in force - which is
    /// what a result belongs to when the sequence is one long.
    static uint8_t selected() { return selected_; }

    /// SWSTART: the regular group, now. Void, for the sampler.
    static void start() { regs().CTLR2 = regs().CTLR2 | adc_swstart; }
    static bool converting() { return (regs().STATR & adc_strt) != 0u; }
    static bool ready() { return (regs().STATR & adc_eoc) != 0u; }
    /// The regular data register whole - the upper half is ADC2's datum
    /// in a dual mode, zero otherwise.
    static uint32_t data() { return regs().RDATAR; }
    /// The result, which clears EOC by the read (the F1 lineage).
    static uint16_t result() { return static_cast<uint16_t>(regs().RDATAR & 0xFFFFu); }
    /// Twelve bits whatever the alignment.
    static uint16_t result_counts() {
        const uint16_t v = result();
        return cfg_.left_aligned ? static_cast<uint16_t>(v >> 4)
                                 : static_cast<uint16_t>(v & adc_max_count);
    }
    /// The follower's datum, from the upper half of this register
    /// (12.3.14): meaningful in a dual mode and on ADC1 alone.
    static uint16_t follower_counts() {
        const uint16_t v = static_cast<uint16_t>((regs().RDATAR >> 16) & 0xFFFFu);
        return cfg_.left_aligned ? static_cast<uint16_t>(v >> 4)
                                 : static_cast<uint16_t>(v & adc_max_count);
    }

    /// One conversion, polled: start, wait, read. False rather than a
    /// hang if the result never comes.
    static bool read(uint16_t& out, uint32_t spins = 0x100000UL) {
        clear_flags(adc_eoc);
        start();
        while (!ready() && spins-- != 0u) {
        }
        if (!ready()) {
            return false;
        }
        out = result_counts();
        return true;
    }
    static uint16_t read(uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        (void)read(v, spins);
        return v;
    }
    /// Convert `count` times and keep the last: what a program does
    /// after a select, so the datum it reports is not the one the
    /// sampling capacitor still carried from the previous channel.
    static uint16_t read_settled(uint8_t count, uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        for (uint8_t i = 0; i < count; ++i) {
            (void)read(v, spins);
        }
        return v;
    }

    // ---- what the internal sources are worth -----------------------------------

    /// The analog supply in millivolts, from a reading of VREFINT: the
    /// reference is 1.2 V nominal (datasheet table 4-26), so a reading
    /// of `counts` says the full scale is that much. It is a
    /// MEASUREMENT of a 1.17..1.23 V part, not a calibration: this
    /// family's signature carries no trimmed word.
    static uint16_t vdda_mv(uint16_t vrefint_counts) {
        if (vrefint_counts == 0u) {
            return 0;
        }
        return static_cast<uint16_t>(
            (static_cast<uint32_t>(adc_vrefint_mv) * adc_steps + vrefint_counts / 2u) /
            vrefint_counts);
    }
    /// util/analog.hpp's adc_mv on a reading, against a supply in mV.
    static uint16_t millivolts(uint16_t counts, uint16_t vdda) {
        return adc_mv(counts, adc_steps, vdda);
    }
    /**
     * The die temperature in HUNDREDTHS of a degree Celsius, from a
     * reading of channel 16 and the supply that scaled it. 12.2.6's
     * formula with the datasheet's typical numbers - and its own
     * warning: the slope and the offset vary from part to part
     * (3.8..4.7 mV per degree, 1.34..1.46 V at 25 degrees), so this is
     * a temperature CHANGE to be trusted and an absolute temperature to
     * be doubted.
     */
    static int32_t temperature_centi_c(uint16_t ts_counts, uint16_t vdda) {
        if (vdda == 0u) {
            return 0;
        }
        const int32_t mv = static_cast<int32_t>(adc_mv(ts_counts, adc_steps, vdda));
        const int32_t delta_uv = (static_cast<int32_t>(adc_temperature_v25_mv) - mv) * 1000;
        return 2500 + (delta_uv * 100) / static_cast<int32_t>(adc_temperature_slope_uv_per_c);
    }

    // ---- the triggers -----------------------------------------------------------

    /// The regular group's start source (table 12-1). EXTTRIG stays set:
    /// software is a trigger too, code 111.
    static void trigger(AdcTrigger t) {
        AdcRegs& r = regs();
        r.CTLR2 = (r.CTLR2 & ~adc_extsel_mask) |
                  (static_cast<uint32_t>(t) << adc_extsel_shift) | adc_exttrig;
    }
    static AdcTrigger trigger() {
        return static_cast<AdcTrigger>((regs().CTLR2 & adc_extsel_mask) >> adc_extsel_shift);
    }
    /// CTLR2.EXTTRIG on its own: what a program drops to make the
    /// regular group answer to nothing while it reprograms.
    static void trigger_enable(bool on) {
        AdcRegs& r = regs();
        r.CTLR2 = on ? (r.CTLR2 | adc_exttrig) : (r.CTLR2 & ~adc_exttrig);
    }

    /// The injected group's (table 12-2).
    static void injected_trigger(AdcInjectedTrigger t) {
        AdcRegs& r = regs();
        r.CTLR2 = (r.CTLR2 & ~adc_jextsel_mask) |
                  (static_cast<uint32_t>(t) << adc_jextsel_shift) | adc_jexttrig;
    }
    static AdcInjectedTrigger injected_trigger() {
        return static_cast<AdcInjectedTrigger>((regs().CTLR2 & adc_jextsel_mask) >>
                                               adc_jextsel_shift);
    }
    /// JAUTO forbids the injected group's external trigger (12.2.4):
    /// this is the bit to drop for it.
    static void injected_trigger_enable(bool on) {
        AdcRegs& r = regs();
        r.CTLR2 = on ? (r.CTLR2 | adc_jexttrig) : (r.CTLR2 & ~adc_jexttrig);
    }

    static void continuous(bool on) {
        AdcRegs& r = regs();
        r.CTLR2 = on ? (r.CTLR2 | adc_cont) : (r.CTLR2 & ~adc_cont);
    }
    static bool continuous() { return (regs().CTLR2 & adc_cont) != 0u; }

    /// CTLR2.DMA, ADC1's alone: a request per regular conversion. False
    /// on ADC2, whose conversions reach memory in a dual mode only.
    static bool dma(bool on) {
        if constexpr (!has_dma) {
            (void)on;
            return false;
        } else {
            AdcRegs& r = regs();
            r.CTLR2 = on ? (r.CTLR2 | adc_dma) : (r.CTLR2 & ~adc_dma);
            return true;
        }
    }
    static bool dma() { return (regs().CTLR2 & adc_dma) != 0u; }

    /**
     * Hand the regular group's stream to a DMA engine: the engine is
     * armed on this converter's data register and CTLR2.DMA is set, so
     * the two things every stream user must do are one verb.
     *
     * REFUSED AT COMPILE TIME for an engine on the wrong channel. THE
     * CHANNEL IS THE REQUEST on this controller (dma_engine.hpp): the
     * regular group raises its request on channel 1 and nowhere else,
     * so an engine named on any other channel would wait for a datum
     * that never comes - which is a wedge, not an error, and therefore
     * worth a refusal. Templated on the engine so that a program with
     * no stream never drags the controller in: only the caller names an
     * engine type, and only then is ch32v203/dma.hpp included anywhere.
     */
    template <typename Engine>
    static void claim_stream() {
        static_assert(has_dma,
                      "brio Adc: only ADC1 has a DMA request (12.2.7's note 2) - the second "
                      "converter's data reaches memory in a dual mode, through the master's "
                      "own register");
        static_assert(Engine::present,
                      "brio Adc: an empty engine slot cannot carry a stream");
        static_assert(Engine::channel == dma_channel,
                      "brio Adc: the CHANNEL IS THE REQUEST on this controller (RM table "
                      "11-5) and the ADC's regular group raises its request on channel 1 - "
                      "an engine on any other channel would never see a conversion "
                      "(DmaRequestOf<DmaRequest::adc1>::channel is how to spell it)");
        Engine::arm(data_address());
        (void)dma(true);
    }

    /// CTLR2.TSVREFE, ADC1's alone: ONE bit wakes both the temperature
    /// sensor and VREFINT, and it also forces the input buffer on for
    /// good (12.3.2's BUFEN note), which is why init() writes it after
    /// the calibration.
    static bool internal_sources(bool on) {
        if constexpr (!has_internal_sources) {
            (void)on;
            return false;
        } else {
            AdcRegs& r = regs();
            r.CTLR2 = on ? (r.CTLR2 | adc_tsvrefe) : (r.CTLR2 & ~adc_tsvrefe);
            if (on) {
                stab_spin();
            }
            return true;
        }
    }
    static bool internal_sources() { return (regs().CTLR2 & adc_tsvrefe) != 0u; }

    // ---- the gain and its buffer -------------------------------------------------

    /// CTLR1.BUFEN and PGA. A gain without the buffer is refused; and a
    /// buffer that TSVREFE has forced on cannot be taken off again, so
    /// `input_buffer()` reads back what the silicon says and not what
    /// was asked.
    static bool gain(AdcGain g, bool buffer = true) {
        if (g != AdcGain::x1 && !buffer) {
            return false;
        }
        AdcRegs& r = regs();
        uint32_t c1 = r.CTLR1 & ~(adc_pga_mask | adc_bufen);
        if (buffer) { c1 |= adc_bufen; }
        c1 |= static_cast<uint32_t>(g) << adc_pga_shift;
        r.CTLR1 = c1;
        return true;
    }
    static AdcGain gain() {
        return static_cast<AdcGain>((regs().CTLR1 & adc_pga_mask) >> adc_pga_shift);
    }
    static bool input_buffer() { return (regs().CTLR1 & adc_bufen) != 0u; }

    // ---- the injected group ------------------------------------------------------

    /**
     * The injected sequence, 1..4 channels (12.3.12). THE SLOTS FILL
     * FROM THE END - a length of two uses JSQ3 and JSQ4 - while the
     * results and the offsets count from the START, in conversion
     * order. More than one conversion needs SCAN (AdcConfig::scan):
     * without it only the first channel converts.
     */
    static bool injected_sequence(const uint8_t* order, uint8_t count) {
        if (order == nullptr || count == 0u || count > adc_injected_slots) {
            return false;
        }
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] >= channels) {
                return false;
            }
        }
        uint32_t v = static_cast<uint32_t>(count - 1u) << 20;
        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t slot = static_cast<uint8_t>(adc_injected_slots - count + i);
            v |= static_cast<uint32_t>(order[i]) << (5u * slot);
        }
        regs().ISQR = v;
        return true;
    }
    static uint8_t injected_length() {
        return static_cast<uint8_t>(((regs().ISQR >> 20) & 0x3u) + 1u);
    }
    /// The per-conversion offset subtracted from the raw datum
    /// (12.3.6), twelve bits. `slot` is the conversion's ORDER, 0..3.
    static bool injected_offset(uint8_t slot, uint16_t offset) {
        if (slot >= adc_injected_slots || offset > adc_max_count) {
            return false;
        }
        regs().IOFR[slot] = offset;
        return true;
    }
    static uint16_t injected_offset(uint8_t slot) {
        return slot < adc_injected_slots ? static_cast<uint16_t>(regs().IOFR[slot] & adc_max_count)
                                         : uint16_t{0};
    }
    static void injected_start() { regs().CTLR2 = regs().CTLR2 | adc_jswstart; }
    static bool injected_ready() { return (regs().STATR & adc_jeoc) != 0u; }
    static bool injected_converting() { return (regs().STATR & adc_jstrt) != 0u; }
    /**
     * The SIGNED result of one injected conversion: the datum less its
     * offset, which is why the register carries a sign bit (12.2.2's
     * figures 12-2 and 12-3). `slot` counts in CONVERSION order, so the
     * first conversion is IDATAR1 whatever JSQ slot it sat in.
     */
    static int16_t injected_result(uint8_t slot) {
        if (slot >= adc_injected_slots) {
            return 0;
        }
        const uint32_t raw = regs().IDATAR[slot];
        if (cfg_.left_aligned) {
            // SIGNB in bit 15, the datum below it: an arithmetic shift
            // of the signed halfword carries the sign down.
            return static_cast<int16_t>(static_cast<int16_t>(raw & 0xFFFFu) >> 3);
        }
        // SIGNB replicated from bit 12 up: sign-extend from twelve bits.
        const int32_t v = static_cast<int32_t>(raw & 0xFFFFu);
        return static_cast<int16_t>((v & 0x1000) != 0 ? (v | ~0xFFF) : (v & 0xFFF));
    }
    /// One injected conversion, polled.
    static bool injected_read(int16_t& out, uint8_t slot = 0, uint32_t spins = 0x100000UL) {
        clear_flags(adc_jeoc);
        injected_start();
        while (!injected_ready() && spins-- != 0u) {
        }
        if (!injected_ready()) {
            return false;
        }
        out = injected_result(slot);
        return true;
    }

    // ---- the analog watchdog -------------------------------------------------------

    static bool watchdog(const AdcWatchdogConfig& w) {
        if (!adc_watchdog_config_valid(w)) {
            return false;
        }
        AdcRegs& r = regs();
        r.WDHTR = w.high;
        r.WDLTR = w.low;
        uint32_t c1 = r.CTLR1 & ~(adc_awdch_mask | adc_awdsgl | adc_awden | adc_jawden | adc_awdie);
        if (w.channel) { c1 |= adc_awdsgl | *w.channel; }
        if (w.regular_group) { c1 |= adc_awden; }
        if (w.injected_group) { c1 |= adc_jawden; }
        if (w.interrupt) { c1 |= adc_awdie; }
        r.CTLR1 = c1;
        return true;
    }
    static void watchdog_off() {
        regs().CTLR1 = regs().CTLR1 & ~(adc_awden | adc_jawden | adc_awdie | adc_awdsgl);
    }
    /// The thresholds, which 12.3.7's note says may be changed during a
    /// conversion and take effect at the next one.
    static void watchdog_thresholds(uint16_t low, uint16_t high) {
        AdcRegs& r = regs();
        r.WDLTR = low & adc_max_count;
        r.WDHTR = high & adc_max_count;
    }
    static uint16_t watchdog_low() { return static_cast<uint16_t>(regs().WDLTR & adc_max_count); }
    static uint16_t watchdog_high() { return static_cast<uint16_t>(regs().WDHTR & adc_max_count); }
    static uint8_t watchdog_channel() {
        return static_cast<uint8_t>(regs().CTLR1 & adc_awdch_mask);
    }

    // ---- the dual modes --------------------------------------------------------------

    /**
     * CTLR1.DUALMOD, ADC1's alone (12.2.7): ADC1 is the master and ADC2
     * the slave, and the code says how their two groups are joined.
     * Refused on ADC2 and on a part with one converter, and refused
     * while the change would be made with a dual mode already standing,
     * which 12.3.2's own note forbids.
     *
     * THERE IS NO util SHAPE FOR THIS. Two converters in lockstep are a
     * capability no other stratum in this tree has, and one family does
     * not make a contract: the verb is this driver's and the finding is
     * the document's.
     */
    static bool dual(AdcDualMode m) {
        static_assert(has_dual_mode,
                      "brio Adc: the dual-mode field is ADC1's - 12.3.2 says DUALMOD is "
                      "reserved in ADC2, and a part with one converter has no follower to "
                      "lead. The master's verb is not a spelling both instances share");
        if (!adc_dual_mode_valid(m)) {
            return false;
        }
        AdcRegs& r = regs();
        if (m != AdcDualMode::independent && (r.CTLR1 & adc_dualmod_mask) != 0u) {
            return false;   // 12.3.2: change it with dual mode disabled
        }
        r.CTLR1 = (r.CTLR1 & ~adc_dualmod_mask) |
                  (static_cast<uint32_t>(m) << adc_dualmod_shift);
        return true;
    }
    static AdcDualMode dual() {
        return static_cast<AdcDualMode>((regs().CTLR1 & adc_dualmod_mask) >> adc_dualmod_shift);
    }

    // ---- flags and interrupts ----------------------------------------------------------

    static uint32_t flags() { return regs().STATR & AdcFlag::all; }
    static bool flag(uint32_t mask) { return (regs().STATR & mask) != 0u; }
    /// Write-zero-to-clear: the bits NOT in the mask are written back as
    /// ones, which leaves them standing.
    static void clear_flags(uint32_t mask) { regs().STATR = ~mask; }

    static constexpr uint32_t converted_interrupt = adc_eocie;
    static constexpr uint32_t injected_interrupt = adc_jeocie;
    static constexpr uint32_t watchdog_interrupt = adc_awdie;
    static void interrupts(uint32_t mask, bool on) {
        AdcRegs& r = regs();
        r.CTLR1 = on ? (r.CTLR1 | mask) : (r.CTLR1 & ~mask);
    }
    static uint32_t interrupts() {
        return regs().CTLR1 & (adc_eocie | adc_jeocie | adc_awdie);
    }

    /**
     * The ISR body: the flags that are both raised and enabled, cleared,
     * handed back. ONE VECTOR SERVES BOTH CONVERTERS, so a program that
     * uses both calls this for each and unions the answers - and it
     * returns zero for the converter that had nothing, which is what
     * makes that safe.
     *
     * EOC is also cleared by reading RDATAR, which a handler that takes
     * the result does anyway.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        AdcRegs& r = regs();
        const uint32_t c1 = r.CTLR1;
        uint32_t armed = 0;
        if ((c1 & adc_eocie) != 0u) { armed |= adc_eoc; }
        if ((c1 & adc_jeocie) != 0u) { armed |= adc_jeoc; }
        if ((c1 & adc_awdie) != 0u) { armed |= adc_awd; }
        const uint32_t hit = r.STATR & armed;
        if (hit != 0u) {
            r.STATR = ~hit;
        }
        return hit;
    }

    static const AdcConfig& config() { return cfg_; }

private:
    /// tSTAB: one microsecond by datasheet table 4-27, and this spins
    /// well past it at every rate the tree can run - the converter is
    /// woken once, so the cost is paid once.
    static void stab_spin() {
        for (uint32_t i = 0; i < 2'000u; ++i) {
            asm volatile("");
        }
    }

    static inline AdcConfig cfg_{};
    static inline uint8_t selected_ = 0;
};

// =============================================================================
// The chapter's arithmetic, pinned at compile time
// =============================================================================

static_assert(sizeof(AdcRegs) == 0x58);
static_assert(adc_base_for(1) == 0x40012400UL && adc_base_for(2) == 0x40012800UL);
static_assert(adc_steps == 4096u && adc_max_count == 4095u);
// 12.2.2: tCONV = the sampling time + 11 cycles. The two ends of it.
static_assert(adc_conversion_half_cycles(adc_sample_shortest) == 25u);    // 1.5 + 11 = 12.5
static_assert(adc_conversion_half_cycles(adc_sample_longest) == 501u);    // 239.5 + 11 = 250.5
// Datasheet table 4-27 gives the same two ends as 14 and 252 ADCCLK: the
// manual's 11-cycle tail and the datasheet's totals differ by a cycle
// and a half at each end, which is the sample-and-hold the datasheet
// counts and the manual does not. Named here so a reader who checks is
// not left wondering which document this file followed.
static_assert(adc_conversion_ns(AdcSampleTime::cycles239_5, 12'000'000UL) == 20'875u);
static_assert(adc_max_source_ohms(AdcSampleTime::cycles1_5) == 400u);
static_assert(adc_max_source_ohms(AdcSampleTime::cycles55_5) == 50'000u);
static_assert(adc_max_source_ohms(AdcSampleTime::cycles239_5) == 0u);
static_assert(adc_gain_factor(AdcGain::x64) == 64u);
static_assert(adc_channel_of(Pad{'A', 0}) == 0u && adc_channel_of(Pad{'A', 7}) == 7u);
static_assert(adc_channel_of(Pad{'B', 0}) == 8u && adc_channel_of(Pad{'B', 1}) == 9u);
static_assert(adc_channel_of(Pad{'C', 0}) == 10u && adc_channel_of(Pad{'C', 5}) == 15u);
static_assert(adc_channel_of(Pad{'B', 2}) == 0xFFu && adc_channel_of(Pad{'C', 13}) == 0xFFu);
static_assert(adc_channel_of(Pad{'D', 0}) == 0xFFu);
static_assert(!adc_config_valid(AdcConfig{.discontinuous = 9}));
static_assert(!adc_config_valid(AdcConfig{.auto_injected = true, .discontinuous = 2}));
static_assert(!adc_config_valid(AdcConfig{.discontinuous = 2, .injected_discontinuous = true}));
static_assert(!adc_config_valid(AdcConfig{.gain = AdcGain::x4}));
static_assert(adc_config_valid(AdcConfig{.input_buffer = true, .gain = AdcGain::x4}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.low = 100, .high = 50}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.high = adc_max_count + 1u}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.channel = adc_channels}));
static_assert(adc_watchdog_config_valid(AdcWatchdogConfig{.channel = adc_vrefint_channel}));
static_assert(!adc_dual_mode_valid(static_cast<AdcDualMode>(10)));

} // namespace brio
