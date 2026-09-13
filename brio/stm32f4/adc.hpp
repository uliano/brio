/*
 * adc.hpp
 *
 * The STM32F4's analog-to-digital converters (RM0090 ch. 13, RM0390
 * ch. 13, RM0383 ch. 11) in this stratum's faces:
 *
 *   AdcCommon        the BLOCK - one per device: the prescaler every
 *                    converter shares, the two internal-source switches,
 *                    the multi-ADC mode with its own DMA modes, the
 *                    common status and data registers, and the ONE reset
 *                    line all the converters hang from.
 *   Adc<n>           one converter, n in 1..3. `Adc<1>` is the MASTER -
 *                    the internal channels and every multi-ADC trigger
 *                    are its alone.
 *   AnalogIn<Pin,ch> a pad handed to a channel. The channel defaults to
 *                    the ADC1/ADC2 map every datasheet of this family
 *                    gives (PA0..PA7 -> IN0..IN7, PB0/PB1 -> IN8/IN9,
 *                    PC0..PC5 -> IN10..IN15) and is stated by the caller
 *                    for anything else - ADC3's own inputs on port F, or
 *                    a pad a package bonds differently.
 *   AdcInput         the three sources that are not pads, as TAGS: their
 *                    channel NUMBERS differ across the family and come
 *                    from the reserve.
 *
 * SIX FACTS THAT SHAPE THIS FILE.
 *
 * 1. THERE ARE UP TO THREE CONVERTERS AND ONE CLOCK. ADCCLK is PCLK2
 *    divided by 2, 4, 6 or 8 - four codes, no bypass, and the divider is
 *    in the COMMON register, so it is the whole block's (13.3.3). fADC
 *    has a ceiling of 36 MHz at 2.4..3.6 V and 18 MHz below (the
 *    datasheets' table of ADC characteristics), which at this family's
 *    top rate makes PCLK2/2 illegal and PCLK2/4 the fastest legal
 *    division: 22.5 MHz from a 90 MHz APB2. `init()` is where that is
 *    judged, because only it knows the clock.
 *
 * 2. THERE IS NO CALIBRATION AND NO REGULATOR. Unlike the STM32G0's
 *    converter this one has no ADCAL and no ADVREGEN: ADON powers it up
 *    and tSTAB (2..3 us, the datasheets' power-up time) is the whole
 *    bring-up. What ST measured on the die is elsewhere - VREFINT_CAL and
 *    the two temperature points in the system memory - and is a
 *    MEASUREMENT, not a trim: nothing is written back (see `AdcFactory`).
 *
 * 3. TWO GROUPS, AND THE INJECTED ONE PREEMPTS. A REGULAR sequence is up
 *    to sixteen conversions into ONE data register (so a sequence longer
 *    than one needs the DMA or a reader fast enough), and an INJECTED
 *    sequence is up to four into four registers of their own, each with a
 *    subtractable offset and a sign. An injected trigger INTERRUPTS a
 *    regular conversion and the regular sequence resumes where it was
 *    (13.3.10) - which is the whole point of the second group and has no
 *    counterpart on the two earlier targets. JSQR's four slots are filled
 *    FROM THE TAIL (13.13.12's note: with JL = 0 the converter runs JSQ4
 *    alone), a trap `injected_sequence()` hides by placing the list
 *    itself.
 *
 * 4. THE STATUS REGISTER IS rc_w0, AND THAT DECIDES HOW IT IS CLEARED.
 *    ADC_SR's six flags are "cleared by software" by writing ZERO
 *    (13.13.1), the opposite of every other flag register in this
 *    stratum. So `clear_flags(mask)` stores `~mask` - ones everywhere
 *    else, which KEEP - and never a read-modify-write, which would clear
 *    a flag that rose between the read and the write.
 *
 * 5. EOC IS CLEARED BY READING THE DATA. 13.13.1: EOC goes down on a
 *    read of ADC_DR as well as on a write of zero, which is what makes a
 *    polling loop one read - and what makes `result()` unusable from a
 *    place that wants the flag left standing. Overrun detection exists
 *    only with DMA = 1 or EOCS = 1 (13.13.3), so a program that converts
 *    without reading and without the DMA is not told it lost anything -
 *    and that is a CHOICE the chapter offers (13.8.3), not an oversight.
 *
 * 6. THE MULTI-ADC MODES ARE THE MASTER'S. Dual and triple modes are
 *    selected in the common register, the trigger comes from ADC1's own
 *    multiplexer and the slaves' triggers must be OFF (13.9), and the
 *    results come back through the common data register. This file
 *    programs them; what a DMA stream does with ADC_CDR is the DMA
 *    chapter's.
 *
 * 7. A SEQUENCE LONGER THAN ONE NEEDS THE DMA, and the request is a CELL
 *    of the fabric's mapping table rather than a number: ADC1 is DMA2's
 *    stream 0 or stream 4 on channel 0, ADC2 stream 2 or 3 on channel 1,
 *    ADC3 stream 0 or 1 on channel 2 - so ADC1 and ADC3 contend for one
 *    stream with two channels. The reserve carries the analog slice of
 *    that table and `engine_placed()` is what an application asserts
 *    against it; there is no engine SLOT here, because a converter has no
 *    transport to hold one - its DMA is a caller-side composition, a scan
 *    into a buffer the caller owns.
 *
 * THE REFERENCE IS A PAD AND NOTHING ELSE. This family has no reference
 * buffer and no reference selector: VREF+ is an input, tied to VDDA on
 * every board this stratum has met, and the only way to learn what it is
 * worth is VREFINT and the factory value. So `Ref` has one enumerator and
 * `ref_mv()` takes the board's millivolts, exactly as on the RP2040 - and
 * `Adc<1>::vdda_mv()` is what makes the guess unnecessary.
 *
 * ERRATA (ES0206 Rev 24 for the F42x/F43x, ES0298 Rev 8 for the F446,
 * ES0287 Rev 6 for the F411 - the same two items under three numbers):
 *  - "ADC sequencer modification during conversion" (ES0206 2.5.1,
 *    ES0298 2.6.1, ES0287 2.4.1, every silicon revision): with a SOFTWARE
 *    start, a write to ADC_SQRx or ADC_JSQR during a conversion resets
 *    that conversion and the converter does NOT restart by itself.
 *    ANSWERED STRUCTURALLY: every sequence verb here refuses while a
 *    software-started conversion is in flight, and
 *    `regular_sequence_unchecked()` exists so a suite can stage the
 *    erratum deliberately.
 *  - "Internal noise impacting the ADC accuracy" (ES0206 2.2.8, ES0298
 *    2.2.8, ES0287 2.2.8): noise on VDD propagates inside and costs
 *    accuracy whatever the power mode. The workaround is a SYSTEM one -
 *    the flash accelerator's prefetch OFF with both caches on, plus
 *    averaging - so it is not this driver's to apply: stm32f4/flash.hpp
 *    owns those bits, the application decides, and the cost of each
 *    choice is measured in docs/stm32f4/adc.md.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "util/analog.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// The reference
// =============================================================================

/// This family has no reference block: VREF+ is a pad (13.2's table of
/// ADC pins), tied to VDDA on every board here, and there is nothing to
/// select. One enumerator, so that util/analog.hpp's vocabulary has the
/// same shape as everywhere else.
enum class Ref : uint8_t { vref_pin };

/// What the pin carries is the BOARD's - 3300 on the three this stratum
/// runs on. `Adc<1>::vdda_mv()` measures it instead of assuming it.
constexpr uint16_t ref_mv(Ref, uint16_t vref_pin_mv = 3300) { return vref_pin_mv; }

// =============================================================================
// Vocabulary
// =============================================================================

/// ADC_CR1.RES (13.13.2). The DATA register is always 16 bits wide; what
/// changes is how many of them are significant and how long the SAR takes.
enum class AdcRes : uint8_t { bits12 = 0, bits10 = 1, bits8 = 2, bits6 = 3 };

/// The full scale of one conversion at this resolution - util/analog.hpp's
/// `steps`.
constexpr uint32_t adc_sample_steps(AdcRes r) {
    switch (r) {
        case AdcRes::bits12: return 4096;
        case AdcRes::bits10: return 1024;
        case AdcRes::bits8: return 256;
        case AdcRes::bits6: return 64;
        default: return 0;
    }
}

/// tSAR in whole ADCCLK cycles (13.7: 12, 10, 8, 6 - one per bit, and
/// nothing rounded, which is why this family needs no half-cycle
/// arithmetic).
constexpr uint8_t adc_sar_cycles(AdcRes r) {
    switch (r) {
        case AdcRes::bits12: return 12;
        case AdcRes::bits10: return 10;
        case AdcRes::bits8: return 8;
        case AdcRes::bits6: return 6;
        default: return 0;
    }
}

/// ADC_SMPR1/SMPR2's per-channel SMP field (13.5): every channel picks
/// its own, which is what lets a slow internal source and a fast pad
/// share one sequence.
enum class AdcSampleTime : uint8_t {
    cycles3 = 0, cycles15 = 1, cycles28 = 2, cycles56 = 3,
    cycles84 = 4, cycles112 = 5, cycles144 = 6, cycles480 = 7,
};

constexpr uint16_t adc_sample_cycles(AdcSampleTime s) {
    switch (s) {
        case AdcSampleTime::cycles3: return 3;
        case AdcSampleTime::cycles15: return 15;
        case AdcSampleTime::cycles28: return 28;
        case AdcSampleTime::cycles56: return 56;
        case AdcSampleTime::cycles84: return 84;
        case AdcSampleTime::cycles112: return 112;
        case AdcSampleTime::cycles144: return 144;
        case AdcSampleTime::cycles480: return 480;
        default: return 0;
    }
}

/// tCONV in whole ADCCLK cycles: the sampling time plus the SAR's own
/// (13.5's "sampling time + 12 cycles", generalized to the four
/// resolutions by 13.7).
constexpr uint16_t adc_conversion_cycles(AdcRes r, AdcSampleTime s) {
    return static_cast<uint16_t>(adc_sample_cycles(s) + adc_sar_cycles(r));
}

/// ADC_CCR.ADCPRE (13.13.16) - the ONE divider between PCLK2 and every
/// converter. There is no bypass: the fastest ADCCLK is half the bus.
enum class AdcPrescaler : uint8_t { div2 = 0, div4 = 1, div6 = 2, div8 = 3 };

constexpr uint8_t adc_prescaler_divisor(AdcPrescaler p) {
    return static_cast<uint8_t>(2u * (static_cast<uint8_t>(p) + 1u));
}

/// The datasheets' fADC ceilings (the ADC characteristics table, the same
/// numbers on the F429, F446 and F411): 36 MHz at 2.4..3.6 V, 18 MHz on a
/// supply below that. The boards here run at 3.3 V, so the first is the
/// one `init()` judges against, and the second is offered for a program
/// that knows its own rail.
inline constexpr uint32_t adc_max_hz = 36'000'000u;
inline constexpr uint32_t adc_max_hz_low_supply = 18'000'000u;
/// The chapter's other end: fADC has a MINIMUM too, 0.6 MHz.
inline constexpr uint32_t adc_min_hz = 600'000u;

/// The smallest division that keeps PCLK2 / div at or below `ceiling`, or
/// div8 when none does - a pure function of the pair, so the caller can
/// ask before it commits and `init()` can refuse afterwards.
constexpr AdcPrescaler adc_prescaler_for(uint32_t pclk2_hz, uint32_t ceiling = adc_max_hz) {
    for (uint8_t code = 0; code < 3; ++code) {
        const AdcPrescaler p = static_cast<AdcPrescaler>(code);
        if (pclk2_hz / adc_prescaler_divisor(p) <= ceiling) {
            return p;
        }
    }
    return AdcPrescaler::div8;
}

constexpr uint32_t adc_clock_hz(uint32_t pclk2_hz, AdcPrescaler p) {
    return pclk2_hz / adc_prescaler_divisor(p);
}

/// ADC_CR2.EXTSEL - table 87's sixteen regular triggers. Fifteen are a
/// timer's capture/compare or TRGO output and the last is a pad's EXTI
/// line; which of them carry a signal is a per-part question the TIMERS
/// answer (`adc_trigger_valid()`).
enum class AdcTrigger : uint8_t {
    tim1_cc1 = 0, tim1_cc2 = 1, tim1_cc3 = 2, tim2_cc2 = 3,
    tim2_cc3 = 4, tim2_cc4 = 5, tim2_trgo = 6, tim3_cc1 = 7,
    tim3_trgo = 8, tim4_cc4 = 9, tim5_cc1 = 10, tim5_cc2 = 11,
    tim5_cc3 = 12, tim8_cc1 = 13, tim8_trgo = 14, exti11 = 15,
};

/// ADC_CR2.JEXTSEL - table 88's sixteen injected triggers, a different
/// set of the same timers plus EXTI line 15.
enum class AdcInjectedTrigger : uint8_t {
    tim1_cc4 = 0, tim1_trgo = 1, tim2_cc1 = 2, tim2_trgo = 3,
    tim3_cc2 = 4, tim3_cc4 = 5, tim4_cc1 = 6, tim4_cc2 = 7,
    tim4_cc3 = 8, tim4_trgo = 9, tim5_cc4 = 10, tim5_trgo = 11,
    tim8_cc2 = 12, tim8_cc3 = 13, tim8_cc4 = 14, exti15 = 15,
};

/// The EXTI lines the last code of each table names - stated here so that
/// an application arming one never spells the number itself.
inline constexpr uint8_t adc_regular_exti_line = 11;
inline constexpr uint8_t adc_injected_exti_line = 15;

/// Which timer a regular trigger code names, 0 for the EXTI one.
constexpr uint8_t adc_trigger_timer(AdcTrigger t) {
    switch (t) {
        case AdcTrigger::tim1_cc1:
        case AdcTrigger::tim1_cc2:
        case AdcTrigger::tim1_cc3: return 1;
        case AdcTrigger::tim2_cc2:
        case AdcTrigger::tim2_cc3:
        case AdcTrigger::tim2_cc4:
        case AdcTrigger::tim2_trgo: return 2;
        case AdcTrigger::tim3_cc1:
        case AdcTrigger::tim3_trgo: return 3;
        case AdcTrigger::tim4_cc4: return 4;
        case AdcTrigger::tim5_cc1:
        case AdcTrigger::tim5_cc2:
        case AdcTrigger::tim5_cc3: return 5;
        case AdcTrigger::tim8_cc1:
        case AdcTrigger::tim8_trgo: return 8;
        default: return 0;
    }
}

constexpr uint8_t adc_trigger_timer(AdcInjectedTrigger t) {
    switch (t) {
        case AdcInjectedTrigger::tim1_cc4:
        case AdcInjectedTrigger::tim1_trgo: return 1;
        case AdcInjectedTrigger::tim2_cc1:
        case AdcInjectedTrigger::tim2_trgo: return 2;
        case AdcInjectedTrigger::tim3_cc2:
        case AdcInjectedTrigger::tim3_cc4: return 3;
        case AdcInjectedTrigger::tim4_cc1:
        case AdcInjectedTrigger::tim4_cc2:
        case AdcInjectedTrigger::tim4_cc3:
        case AdcInjectedTrigger::tim4_trgo: return 4;
        case AdcInjectedTrigger::tim5_cc4:
        case AdcInjectedTrigger::tim5_trgo: return 5;
        case AdcInjectedTrigger::tim8_cc2:
        case AdcInjectedTrigger::tim8_cc3:
        case AdcInjectedTrigger::tim8_cc4: return 8;
        default: return 0;
    }
}

/// Whether this device has what a trigger code names. RM0383 table 42
/// spells the F411's two TIM8 codes "Reserved" and the F410 loses most of
/// the table - and the reserve reads that off the TIMERS' own base
/// macros rather than off a per-part list. The EXTI codes are every
/// part's.
constexpr bool adc_trigger_valid(AdcTrigger t) {
    const uint8_t tim = adc_trigger_timer(t);
    return tim == 0u || tim_present(tim);
}

constexpr bool adc_trigger_valid(AdcInjectedTrigger t) {
    const uint8_t tim = adc_trigger_timer(t);
    return tim == 0u || tim_present(tim);
}

/// ADC_CR2.EXTEN / JEXTEN - table 86. `none` is what makes SWSTART (or
/// JSWSTART) the only way in.
enum class AdcEdge : uint8_t { none = 0, rising = 1, falling = 2, both = 3 };

/// The three sources that are not pads, as TAGS and not as channel
/// numbers: the temperature sensor sits on channel 16 in one part class
/// and on 18 in the others (RM0090 13.3.4), so a numeric enumerator would
/// be a per-part lie. `adc_input_channel()` is the reserve's answer.
enum class AdcInput : uint8_t { temperature, vrefint, vbat };

/// The channel `in` reaches on THIS part, 0xFF where the reserve does not
/// know the class (see stm32f4/device_tables.hpp: the manual is the only
/// authority and only four classes' are on the desk).
constexpr uint8_t adc_input_channel(AdcInput in) {
    constexpr AdcInternalFacts f = adc_internal_facts();
    switch (in) {
        case AdcInput::temperature: return f.temperature_channel;
        case AdcInput::vrefint: return f.vrefint_channel;
        case AdcInput::vbat: return f.vbat_channel;
        default: return 0xFFu;
    }
}

constexpr bool adc_input_valid(AdcInput in) { return adc_input_channel(in) < adc_channel_count; }

/**
 * The ADC1/ADC2 pad map, and it is a DATASHEET table with no header
 * symbol: DS10693 table 11, the F429 datasheet's table 12 and DS10314's
 * table 9 all give the same sixteen - PA0..PA7 are IN0..IN7, PB0 and PB1
 * are IN8 and IN9, PC0..PC5 are IN10..IN15 - and the first four are
 * ADC3's too. Anything else (ADC3's IN4..IN9, IN14 and IN15, which live
 * on port F) is stated by the caller, exactly as stm32f4/pin.hpp's AF
 * numbers are: the device header carries no analog pin table at all, so
 * the bench is the check.
 *
 * 0xFF for a pad that is not one of the sixteen.
 */
constexpr uint8_t adc12_channel_of(char port, uint8_t pin) {
    if (port == 'A' && pin <= 7u) {
        return pin;
    }
    if (port == 'B' && pin <= 1u) {
        return static_cast<uint8_t>(8u + pin);
    }
    if (port == 'C' && pin <= 5u) {
        return static_cast<uint8_t>(10u + pin);
    }
    return 0xFFu;
}

/// ADC3's own map (the same three datasheets, which agree): IN0..IN3 on
/// PA0..PA3 and IN10..IN13 on PC0..PC3 as for the other two, and IN4..IN9
/// plus IN14/IN15 on port F, which only the big packages bond.
constexpr uint8_t adc3_channel_of(char port, uint8_t pin) {
    if (port == 'A' && pin <= 3u) {
        return pin;
    }
    if (port == 'C' && pin <= 3u) {
        return static_cast<uint8_t>(10u + pin);
    }
    if (port == 'F') {
        switch (pin) {
            case 3: return 9;
            case 4: return 14;
            case 5: return 15;
            case 6: return 4;
            case 7: return 5;
            case 8: return 6;
            case 9: return 7;
            case 10: return 8;
            default: return 0xFFu;
        }
    }
    return 0xFFu;
}

/// ADC_SR's flags (13.13.1). ONE set of constants, used as a flag mask
/// and translated into ADC_CR1's enable bits by `interrupts()` - unlike
/// the STM32G0's converter, whose two registers share a layout, this
/// family scatters the four enables across CR1 and the translation has to
/// be written out.
struct AdcFlag {
    static constexpr uint32_t watchdog = ADC_SR_AWD;
    static constexpr uint32_t converted = ADC_SR_EOC;
    static constexpr uint32_t injected_converted = ADC_SR_JEOC;
    static constexpr uint32_t injected_started = ADC_SR_JSTRT;
    static constexpr uint32_t started = ADC_SR_STRT;
    static constexpr uint32_t overrun = ADC_SR_OVR;
    /// The four that can interrupt (table 89); JSTRT and STRT cannot.
    static constexpr uint32_t interrupting = watchdog | converted | injected_converted | overrun;
    static constexpr uint32_t all =
        watchdog | converted | injected_converted | injected_started | started | overrun;
};

/// ADC_CCR.MULTI (13.13.16). The dual codes pair ADC1 with ADC2 and leave
/// ADC3 independent; the triple ones drive all three. Every code the
/// chapter leaves out is Reserved and `adc_multi_valid()` refuses it.
enum class AdcMulti : uint8_t {
    independent = 0x00,
    dual_regular_injected = 0x01,
    dual_regular_alternate = 0x02,
    dual_injected = 0x05,
    dual_regular = 0x06,
    dual_interleaved = 0x07,
    dual_alternate = 0x09,
    triple_regular_injected = 0x11,
    triple_regular_alternate = 0x12,
    triple_injected = 0x15,
    triple_regular = 0x16,
    triple_interleaved = 0x17,
    triple_alternate = 0x19,
};

/// How many converters a mode needs - and therefore whether this device
/// can be asked for it.
constexpr uint8_t adc_multi_converters(AdcMulti m) {
    const uint8_t code = static_cast<uint8_t>(m);
    if (code == 0u) {
        return 1;
    }
    return (code & 0x10u) != 0u ? 3u : 2u;
}

constexpr bool adc_multi_valid(AdcMulti m) {
    switch (m) {
        case AdcMulti::independent:
        case AdcMulti::dual_regular_injected:
        case AdcMulti::dual_regular_alternate:
        case AdcMulti::dual_injected:
        case AdcMulti::dual_regular:
        case AdcMulti::dual_interleaved:
        case AdcMulti::dual_alternate:
        case AdcMulti::triple_regular_injected:
        case AdcMulti::triple_regular_alternate:
        case AdcMulti::triple_injected:
        case AdcMulti::triple_regular:
        case AdcMulti::triple_interleaved:
        case AdcMulti::triple_alternate:
            return adc_multi_converters(m) <= adc_instances();
        default: return false;
    }
}

/// ADC_CCR.DMA - the multi-ADC DMA modes (13.9): one half-word per
/// request, two as a word, or two bytes as a half-word. Which one a mode
/// wants is the chapter's, not this driver's, and 13.9 says dual mode
/// does not support mode 1 at all.
enum class AdcMultiDma : uint8_t { off = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

/// What one converter is configured with. The defaults are "one 12-bit
/// conversion of one channel when I ask for it".
struct AdcConfig {
    AdcRes resolution = AdcRes::bits12;
    /// ADC_CR2.ALIGN. Left alignment puts a 12-bit regular result in bits
    /// 15:4 and a 6-bit one in bits 7:2 (13.4's own special case).
    bool left_aligned = false;

    bool scan = false;           ///< CR1.SCAN: walk the sequence, one conversion each
    bool continuous = false;     ///< CR2.CONT
    /// CR2.EOCS: EOC at every conversion instead of at the end of the
    /// sequence - and, with it, OVERRUN DETECTION (13.13.3).
    bool eoc_per_conversion = true;

    bool discontinuous = false;  ///< CR1.DISCEN, regular group
    uint8_t discontinuous_count = 1;   ///< DISCNUM + 1, 1..8
    bool injected_discontinuous = false;   ///< CR1.JDISCEN
    /// CR1.JAUTO: the injected group runs by itself after the regular
    /// one. 13.3.10 forbids an injected external trigger with it, and
    /// forbids it with either discontinuous mode.
    bool auto_injected = false;

    bool dma = false;            ///< CR2.DMA (single-ADC mode)
    /// CR2.DDS: keep asking after the last transfer, which is what a
    /// circular DMA needs; without it the DMA bit must be cycled by hand.
    bool dma_continuous_requests = false;

    AdcTrigger trigger = AdcTrigger::tim1_cc1;
    AdcEdge trigger_edge = AdcEdge::none;
    AdcInjectedTrigger injected_trigger = AdcInjectedTrigger::tim1_cc4;
    AdcEdge injected_trigger_edge = AdcEdge::none;
};

/// Everything the chapter refuses, and nothing the chapter does not.
constexpr bool adc_config_valid(const AdcConfig& c) {
    if (adc_sample_steps(c.resolution) == 0u) {
        return false;
    }
    if (c.discontinuous_count == 0u || c.discontinuous_count > 8u) {
        return false;   // DISCNUM is three bits, 13.13.2
    }
    if (c.discontinuous && c.injected_discontinuous) {
        return false;   // 13.3.11: never both groups at once
    }
    if (c.auto_injected && (c.discontinuous || c.injected_discontinuous)) {
        return false;   // 13.3.10's own note
    }
    if (c.auto_injected && c.injected_trigger_edge != AdcEdge::none) {
        return false;   // 13.3.10: "external trigger on injected channels must be disabled"
    }
    if (c.trigger_edge != AdcEdge::none && !adc_trigger_valid(c.trigger)) {
        return false;
    }
    if (c.injected_trigger_edge != AdcEdge::none && !adc_trigger_valid(c.injected_trigger)) {
        return false;
    }
    if (c.dma_continuous_requests && !c.dma) {
        return false;
    }
    return true;
}

/// The full scale of what lands in ADC_DR under this config -
/// util/analog.hpp's `steps`. There is no oversampler on this converter,
/// so it is the resolution's and nothing else.
constexpr uint32_t adc_result_steps(const AdcConfig& c) { return adc_sample_steps(c.resolution); }

// =============================================================================
// The factory measurements (the datasheets' calibration tables)
// =============================================================================

/**
 * The three numbers ST measured on this die and left in the system
 * memory: DS10693 tables 81 and 84, the F429 datasheet's and DS10314's
 * twins - all three at the SAME addresses and the SAME conditions, which
 * is why one struct serves the family.
 *
 * They are ADC RESULTS at VDDA = VREF+ = 3.3 V (note the difference from
 * the STM32G0, whose points are taken at 3.0 V), the temperature ones at
 * 30 and 110 degrees. Nothing is written back anywhere: they are the
 * arithmetic's input, and `Adc<1>::vdda_mv()` / `temperature_centi_c()`
 * are the whole of their purpose.
 *
 * No device header of this pack declares the addresses, which is why they
 * are spelled here - the peripheral that uses a number owns it.
 */
struct AdcFactory {
    static constexpr uint16_t characterization_mv = 3300;
    static constexpr int16_t ts_cal1_celsius = 30;
    static constexpr int16_t ts_cal2_celsius = 110;

    static constexpr uint32_t vrefint_cal_address = 0x1FFF7A2AUL;
    static constexpr uint32_t ts_cal1_address = 0x1FFF7A2CUL;
    static constexpr uint32_t ts_cal2_address = 0x1FFF7A2EUL;

    static uint16_t vrefint_cal() { return read(vrefint_cal_address); }
    static uint16_t ts_cal1() { return read(ts_cal1_address); }
    static uint16_t ts_cal2() { return read(ts_cal2_address); }

    /// A blank or erased pair reads 0xFFFF and a zero would divide by
    /// nothing, so every consumer asks first - and so does a part class
    /// whose manual nobody read, where the channels themselves are a
    /// guess (stm32f4/device_tables.hpp).
    static bool plausible() {
        if (!adc_internal_facts().known) {
            return false;
        }
        const uint16_t v = vrefint_cal();
        const uint16_t a = ts_cal1();
        const uint16_t b = ts_cal2();
        return v != 0u && v != 0xFFFFu && a != 0u && a != 0xFFFFu && b != 0u && b != 0xFFFFu &&
               a != b;
    }

    /// The datasheet's OTHER answer for the temperature - the typical
    /// slope and the voltage at 25 degrees (the temperature sensor
    /// characteristics table: 2.5 mV/C, 760 mV) - kept beside the
    /// calibration because it is what a part with no calibration values
    /// would have to use, and because the two disagreeing is a finding.
    static constexpr uint16_t typical_v25_mv = 760;
    static constexpr uint16_t typical_slope_uv_per_c = 2500;

private:
    static uint16_t read(uint32_t address) {
        return *reinterpret_cast<const volatile uint16_t*>(address);
    }
};

// =============================================================================
// A pad on a channel
// =============================================================================

/**
 * AnalogIn<Pin, channel>: the claim that this pad is that ADC channel.
 *
 * The channel DEFAULTS to `adc12_channel_of()` - the sixteen pads every
 * datasheet of this family maps the same way - so `AnalogIn<Pin<'A', 4>>`
 * is channel 4 with nothing to look up. A pad outside that map (ADC3's
 * port F inputs) states its channel: `AnalogIn<Pin<'F', 6>, 4>`.
 *
 * `claim()` is analog mode, which is the one mode 8.3.12 turns the input
 * buffer OFF in - so a claimed analog pad reads zero on IDR, and that is
 * not a fault. It is NOT the reset state on this family (the reset state
 * is input floating), which is why claiming is a real store.
 */
template <class P, uint8_t channel_ = adc12_channel_of(P::port_letter, P::pin_number)>
struct AnalogIn {
    static_assert(channel_ != 0xFFu,
                  "brio AnalogIn: this pad is not one of the sixteen ADC1/ADC2 inputs every "
                  "datasheet of this family maps the same way, so no channel can be derived "
                  "from it - state the channel (ADC3's own inputs on port F are the case that "
                  "needs it)");
    static_assert(channel_ < adc_channel_count,
                  "brio AnalogIn: no such ADC channel on this family - the chapter documents "
                  "0..18 and everything above is Reserved");

    using pin = P;
    static constexpr uint8_t channel = channel_;

    static void claim() { P::analog(); }
    static void release() { P::analog(); }
};

// =============================================================================
// The common block
// =============================================================================

/**
 * AdcCommon: what the converters SHARE - the prescaler, the two
 * internal-source switches, the multi-ADC machinery, and the one reset.
 *
 * It is a monostate because there is exactly one such block on every part
 * (13.14's register map puts it at ADC1's base + 0x300, whether the part
 * has one converter or three), and it is a TYPE OF ITS OWN rather than a
 * corner of `Adc<n>` because everything in it belongs to all of them: a
 * prescaler written through `Adc<3>` would change `Adc<1>`'s rate, and
 * the reset line pulses every converter at once.
 */
class AdcCommon {
public:
    static_assert(adc_common_base() != 0u,
                  "brio AdcCommon: this device declares no ADC common block");

    AdcCommon() = delete;

    static ADC_Common_TypeDef& regs() {
        return *reinterpret_cast<ADC_Common_TypeDef*>(adc_common_base());
    }

    static constexpr uint8_t instances = adc_instances();

    /// ONE reset line for every converter and this block (7.3.7). Pulsing
    /// it while a sibling is converting stops the sibling too, which is
    /// why it is spelled here and not on the instance.
    static void reset() { Rcc::apb2_reset(adc_reset_mask); }

    // ---- the shared clock ----------------------------------------------------

    static void prescaler(AdcPrescaler p) {
        regs().CCR = (regs().CCR & ~ADC_CCR_ADCPRE_Msk) |
                     (static_cast<uint32_t>(p) << ADC_CCR_ADCPRE_Pos);
    }
    static AdcPrescaler prescaler() {
        return static_cast<AdcPrescaler>((regs().CCR & ADC_CCR_ADCPRE_Msk) >> ADC_CCR_ADCPRE_Pos);
    }

    /// fADC as the register holds it, from the bus rate the caller states.
    static uint32_t adc_hz(uint32_t pclk2_hz) { return adc_clock_hz(pclk2_hz, prescaler()); }

    // ---- the two internal-source switches (13.10, 13.11) ----------------------

    /**
     * TSVREFE: ONE bit wakes BOTH the temperature sensor and VREFINT
     * (13.10's own note), which is why there is one verb and not two.
     * 13.10 advises raising it together with ADON so the two start-up
     * times overlap.
     */
    static void internal_sources(bool on) { ccr_bit(ADC_CCR_TSVREFE, on); }
    static bool internal_sources() { return (regs().CCR & ADC_CCR_TSVREFE) != 0u; }

    /**
     * VBATE: the divider bridge onto the shared channel. 13.11 and
     * 13.13.16: where the sensor and the battery share a channel - every
     * class but the F405's - setting both switches gives the BATTERY, and
     * the sensor is simply not what is converted. The rule is stated
     * rather than enforced: the two bits are legal together and the
     * silicon picks, so a driver that refused would be inventing a rule
     * the chapter does not have.
     */
    static void vbat(bool on) { ccr_bit(ADC_CCR_VBATE, on); }
    static bool vbat() { return (regs().CCR & ADC_CCR_VBATE) != 0u; }

    /// Whether, on THIS part, the two switches fight over one channel.
    static constexpr bool sensor_shares_vbat() { return adc_internal_facts().sensor_shares_vbat; }
    /// What the bridge divides the battery by before the channel sees it
    /// (2 on the F405 class, 4 on the rest); 0 where the reserve does not
    /// know the class.
    static constexpr uint8_t vbat_divider() { return adc_internal_facts().vbat_divider; }

    // ---- multi-ADC mode (13.9) -------------------------------------------------

    static constexpr bool multi_valid(AdcMulti m) { return adc_multi_valid(m); }

    /**
     * Select a dual or triple mode. REFUSED when this device has fewer
     * converters than the mode needs, and when the code is one of the
     * Reserved ones.
     *
     * 13.13.16's own note: "a change of channel configuration generates
     * an abort" in multi mode, so the way to reconfigure is to come back
     * to `independent` first - which is also what 13.9.3 demands after a
     * sequence is interrupted. The obligation is the caller's; this verb
     * writes the field.
     */
    static bool multi(AdcMulti m) {
        if (!adc_multi_valid(m)) {
            return false;
        }
        regs().CCR = (regs().CCR & ~ADC_CCR_MULTI_Msk) |
                     (static_cast<uint32_t>(m) << ADC_CCR_MULTI_Pos);
        return true;
    }
    static AdcMulti multi() {
        return static_cast<AdcMulti>((regs().CCR & ADC_CCR_MULTI_Msk) >> ADC_CCR_MULTI_Pos);
    }

    /// ADC_CCR.DMA and DDS - the multi-ADC transfer mode, which is a
    /// different field from each converter's own CR2.DMA.
    static void multi_dma(AdcMultiDma mode, bool continuous_requests) {
        uint32_t v = regs().CCR & ~(ADC_CCR_DMA_Msk | ADC_CCR_DDS);
        v |= static_cast<uint32_t>(mode) << ADC_CCR_DMA_Pos;
        if (continuous_requests) {
            v |= ADC_CCR_DDS;
        }
        regs().CCR = v;
    }
    static AdcMultiDma multi_dma() {
        return static_cast<AdcMultiDma>((regs().CCR & ADC_CCR_DMA_Msk) >> ADC_CCR_DMA_Pos);
    }

    /**
     * ADC_CCR.DELAY - the gap between two sampling phases in interleaved
     * mode, 5 to 20 ADCCLK cycles (13.13.16). Refused outside that range
     * rather than truncated. 13.9.3's caveat: the silicon lengthens the
     * gap by itself where the other converter is still sampling, so this
     * is a MINIMUM and not a period.
     */
    static bool interleave_delay(uint8_t cycles) {
        if (cycles < 5u || cycles > 20u) {
            return false;
        }
        regs().CCR = (regs().CCR & ~ADC_CCR_DELAY_Msk) |
                     (static_cast<uint32_t>(cycles - 5u) << ADC_CCR_DELAY_Pos);
        return true;
    }
    static uint8_t interleave_delay() {
        return static_cast<uint8_t>(((regs().CCR & ADC_CCR_DELAY_Msk) >> ADC_CCR_DELAY_Pos) + 5u);
    }

    // ---- the common status and data -------------------------------------------

    /**
     * ADC_CSR - every converter's flags in one read-only word (13.13.15),
     * six bits per converter starting at 0, 8 and 16. It CANNOT clear
     * anything: a flag goes down in its own ADC_SR.
     */
    static uint32_t status() { return regs().CSR; }
    static uint32_t status(uint8_t instance) {
        if (instance < 1u || instance > 3u) {
            return 0;
        }
        return (regs().CSR >> (8u * (instance - 1u))) & AdcFlag::all;
    }

    /// ADC_CDR (13.13.17): the pair of results a dual or triple mode
    /// delivers in one 32-bit read, the master's in the low half.
    static uint32_t data() { return regs().CDR; }
    static uint16_t data_low() { return static_cast<uint16_t>(regs().CDR & 0xFFFFu); }
    static uint16_t data_high() { return static_cast<uint16_t>(regs().CDR >> 16); }

    /// Where a DMA stream reads a multi-ADC pair from.
    static volatile void* data_address() { return &regs().CDR; }

    /// Everything this block turned on, off again - the internal sources
    /// down (13.10's advice for a low-power mode), the multi mode back to
    /// independent, the prescaler at its reset value.
    static void release() {
        regs().CCR = 0;
    }

private:
    static void ccr_bit(uint32_t mask, bool on) {
        regs().CCR = on ? (regs().CCR | mask) : (regs().CCR & ~mask);
    }
};

// =============================================================================
// One converter
// =============================================================================

template <uint8_t n>
class Adc {
public:
    static_assert(n >= 1 && n <= 3, "brio Adc: this family numbers its converters 1..3");
    static_assert(n > 3 || adc_present(n),
                  "brio Adc: this device does not have that converter (the F401, F410, F411, "
                  "F412 and F413/F423 have ADC1 alone, and their headers declare no ADC2_BASE)");

    Adc() = delete;

    static constexpr uint8_t instance = n;
    /// ADC1 is the master: the internal channels are its alone (13.3.4's
    /// note) and every multi-ADC trigger comes from its multiplexer.
    static constexpr bool is_master = n == 1;
    static constexpr uint8_t channels = adc_channel_count;

    /// The NVIC line, SHARED BY EVERY CONVERTER, so a handler bound to it
    /// answers for all of them.
    static constexpr IRQn_Type irq() { return adc_irq(); }

    static ADC_TypeDef& regs() { return *reinterpret_cast<ADC_TypeDef*>(adc_base(n)); }

    /// Where a DMA stream reads this converter's regular result from.
    static volatile void* data_address() { return &regs().DR; }

    /**
     * Whether an engine sits on a CELL of the request mapping that really
     * carries this converter's request - the same question
     * `uart_engine_placed()` asks for a serial instance, and the same
     * refusal on a part class whose table was not read.
     *
     * There is no task here to hold an engine slot: a converter's DMA is
     * a caller-side composition (a scan into a buffer the caller owns),
     * so this is offered as a check an application static_asserts rather
     * than as a template parameter it fills in.
     */
    template <typename E>
    static constexpr bool engine_placed() {
        if constexpr (!E::present) {
            return true;
        } else {
            return adc_dma_placement_valid(n, E::controller, E::stream, E::channel);
        }
    }

    /// The cells this converter's request is wired to, for a program that
    /// would rather ask than look the table up.
    static constexpr DmaPlacements dma_placements() { return adc_dma_placements(n); }

    // ---- the bus clock ---------------------------------------------------------

    static void bus_clock(bool on) { Rcc::apb2_clock(adc_clock_mask(n), on); }
    static bool bus_clock() { return Rcc::apb2_clock(adc_clock_mask(n)); }

    // ---- power (13.3.1) --------------------------------------------------------

    static bool powered() { return (regs().CR2 & ADC_CR2_ADON) != 0u; }

    /**
     * ADON, and the stabilization time that goes with it. 13.3.7: the
     * converter needs tSTAB before it converts accurately, which the
     * datasheets give as 2 us typical and 3 us maximum - so `power_on()`
     * takes the CLOCK and spends it, and nothing else in this file has to
     * think about it again. 13.10's advice - raise TSVREFE at the same
     * time so the sensor's start-up overlaps - is the caller's to follow
     * through `AdcCommon`.
     */
    template <typename Clock>
    static bool power_on(Clock clock) {
        if (powered()) {
            return true;
        }
        regs().CR2 |= ADC_CR2_ADON;
        return delay_us(clock, 4);
    }

    /// Clearing ADON stops the conversions and puts the converter in
    /// power-down, where 13.3.1 says it draws a few microamps.
    static void power_off() { regs().CR2 &= ~ADC_CR2_ADON; }

    // ---- configuration ---------------------------------------------------------

    static constexpr bool config_valid(const AdcConfig& c) { return adc_config_valid(c); }

    /**
     * Write CR1 and CR2 from `c`, leaving ADON exactly as it was: this
     * verb configures, it does not power.
     *
     * There is no disabled-state rule to keep here - unlike the STM32G0's
     * CFGR1, this family's CR1 and CR2 carry no note forbidding a write
     * while the converter is on - so the refusal is the CONFIG's own
     * validity and nothing more. What must not move under a running
     * conversion is the SEQUENCE, and that is the sequence verbs' rule
     * (the errata item in this file's header).
     */
    static bool configure(const AdcConfig& c) {
        if (!adc_config_valid(c)) {
            return false;
        }
        cfg_ = c;
        const uint32_t adon = regs().CR2 & ADC_CR2_ADON;
        regs().CR1 = cr1_word(c) | (regs().CR1 & watchdog_cr1_mask);
        regs().CR2 = cr2_word(c) | adon;
        return true;
    }

    static const AdcConfig& config() { return cfg_; }

    /**
     * The whole bring-up in one verb: the bus clock, the block's
     * prescaler for this rate, the configuration, ADON and its
     * stabilization.
     *
     * `pclk2_hz` comes from the clock the caller hands over
     * (`apb_hz(clock, true)`), because the converter hangs off APB2 and
     * the CORE rate is not what divides. REFUSES when no prescaler code
     * keeps fADC under `ceiling` - which at 180 MHz on this family means
     * PCLK2/2 is out and PCLK2/4 is the fastest legal division.
     *
     * The RESET is deliberately NOT taken here: one line resets every
     * converter and the common block together, so an `Adc<2>::init()`
     * would silently stop an `Adc<1>` mid-sequence. `AdcCommon::reset()`
     * is the verb, and the program says when.
     */
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& c, uint32_t ceiling = adc_max_hz) {
        static_assert(clock_follows<Clock, Adc>(),
                      "brio Adc: initialized with a DynamicClock that does not list it among "
                      "its Users - the ADC prescaler would not follow the new bus rate");
        if (!adc_config_valid(c)) {
            return false;
        }
        const uint32_t pclk2 = apb_hz(clock, true);
        const AdcPrescaler p = adc_prescaler_for(pclk2, ceiling);
        const uint32_t f = adc_clock_hz(pclk2, p);
        if (f > ceiling || f < adc_min_hz) {
            return false;
        }
        bus_clock(true);
        AdcCommon::prescaler(p);
        if (!configure(c)) {
            return false;
        }
        return power_on(clock);
    }

    /// Everything this converter turned on, off again. The common block
    /// is left alone - it is the siblings' too.
    static void release() {
        regs().CR1 = 0;
        regs().CR2 = 0;
        clear_flags(AdcFlag::all);
        bus_clock(false);
    }

    // ---- sampling times (13.5) --------------------------------------------------

    /**
     * The sampling time of ONE channel: SMPR2 holds channels 0..9 and
     * SMPR1 channels 10..18, three bits each. 13.13.4's own warning -
     * "during sampling cycles the channel selection bits must remain
     * unchanged" - is about the SEQUENCE and not about this register.
     */
    static bool sample_time(uint8_t channel, AdcSampleTime s) {
        if (channel >= channels) {
            return false;
        }
        if (channel < 10u) {
            const uint32_t shift = 3u * channel;
            regs().SMPR2 = (regs().SMPR2 & ~(0x7UL << shift)) |
                           (static_cast<uint32_t>(s) << shift);
        } else {
            const uint32_t shift = 3u * (channel - 10u);
            regs().SMPR1 = (regs().SMPR1 & ~(0x7UL << shift)) |
                           (static_cast<uint32_t>(s) << shift);
        }
        return true;
    }

    static AdcSampleTime sample_time(uint8_t channel) {
        if (channel >= channels) {
            return AdcSampleTime::cycles3;
        }
        const uint32_t reg = channel < 10u ? regs().SMPR2 : regs().SMPR1;
        const uint32_t shift = 3u * (channel < 10u ? channel : channel - 10u);
        return static_cast<AdcSampleTime>((reg >> shift) & 0x7u);
    }

    /// The same for every channel at once - what a program that samples
    /// one pad and one internal source at the same rate writes.
    static void sample_time_all(AdcSampleTime s) {
        uint32_t w = 0;
        for (uint8_t i = 0; i < 10u; ++i) {
            w |= static_cast<uint32_t>(s) << (3u * i);
        }
        regs().SMPR2 = w;
        w = 0;
        for (uint8_t i = 0; i < 9u; ++i) {
            w |= static_cast<uint32_t>(s) << (3u * i);
        }
        regs().SMPR1 = w;
    }

    /// tCONV of `channel` in ADCCLK cycles, under the resolution in force.
    static uint16_t conversion_cycles(uint8_t channel) {
        return adc_conversion_cycles(cfg_.resolution, sample_time(channel));
    }

    // ---- the regular sequence (13.3.4) ------------------------------------------

    /// util/analog_sampler.hpp's `input_code`: the channel a result was
    /// taken on, as `selected()` reports it.
    template <class P, uint8_t ch>
    static constexpr uint8_t input_code(AnalogIn<P, ch>) { return ch; }
    static constexpr uint8_t input_code(AdcInput in) { return adc_input_channel(in); }

    /**
     * Select ONE channel - a regular sequence of length one.
     *
     * VOID, because util/analog_sampler.hpp's AnalogConverter concept asks
     * for a void select(); `select_sync()` is the same thing with the
     * answer a caller that can act on a failure wants.
     */
    template <class P, uint8_t ch>
    static void select(AnalogIn<P, ch>) { (void)select_channel(ch); }
    static void select(AdcInput in) { (void)select_channel(adc_input_channel(in)); }

    template <class P, uint8_t ch>
    static bool select_sync(AnalogIn<P, ch>) { return select_channel(ch); }
    static bool select_sync(AdcInput in) { return select_channel(adc_input_channel(in)); }

    static bool select_channel(uint8_t channel) {
        const uint8_t one[1] = {channel};
        return regular_sequence(one, 1);
    }

    /**
     * The regular sequence: up to sixteen channels in the order given,
     * written into SQR3 (slots 1..6), SQR2 (7..12) and SQR1 (13..16) with
     * the length in SQR1's L field.
     *
     * REFUSED while a software-started conversion is in flight - which is
     * the structural answer to ES0206 2.5.1 / ES0298 2.6.1 / ES0287 2.4.1
     * (the write resets that conversion and nothing restarts it). What
     * "in flight" means on a converter with no busy bit is spelled out on
     * `converting()`. `regular_sequence_unchecked()` is the staging
     * ground for a suite that wants the erratum to happen.
     */
    static bool regular_sequence(const uint8_t* order, uint8_t count) {
        if (converting()) {
            return false;
        }
        return regular_sequence_unchecked(order, count);
    }

    /// The same store with the refusal removed. Named for what it is so
    /// that it cannot be reached for by accident.
    static bool regular_sequence_unchecked(const uint8_t* order, uint8_t count) {
        if (order == nullptr || count == 0u || count > 16u) {
            return false;
        }
        uint32_t sqr[3] = {0, 0, 0};   // SQR3, SQR2, SQR1
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] >= channels) {
                return false;
            }
            sqr[i / 6u] |= static_cast<uint32_t>(order[i]) << (5u * (i % 6u));
        }
        regs().SQR3 = sqr[0];
        regs().SQR2 = sqr[1];
        regs().SQR1 = sqr[2] | (static_cast<uint32_t>(count - 1u) << ADC_SQR1_L_Pos);
        selected_ = order[0];
        length_ = count;
        return true;
    }

    /// How many conversions the sequence holds, as SQR1's L reads.
    static uint8_t sequence_length() {
        return static_cast<uint8_t>(((regs().SQR1 & ADC_SQR1_L_Msk) >> ADC_SQR1_L_Pos) + 1u);
    }

    /// The channel in slot `slot` (1..16) of the regular sequence.
    static uint8_t sequence_channel(uint8_t slot) {
        if (slot < 1u || slot > 16u) {
            return 0xFFu;
        }
        const uint8_t i = static_cast<uint8_t>(slot - 1u);
        const uint32_t reg = i < 6u ? regs().SQR3 : (i < 12u ? regs().SQR2 : regs().SQR1);
        return static_cast<uint8_t>((reg >> (5u * (i % 6u))) & 0x1Fu);
    }

    /**
     * The channel the LAST result was taken on, which is what
     * util/analog_sampler.hpp labels a sample with.
     *
     * THE SILICON DOES NOT REPORT IT. There is no current-channel
     * register on this converter either: the sequencer walks a list the
     * driver wrote, so the driver is what knows where it is. With a
     * one-channel selection - the sampler's whole usage - that is exact;
     * inside a longer sequence it is the sequence's FIRST channel, and a
     * caller walking a sequence labels its own results by position.
     */
    static uint8_t selected() { return selected_; }

    // ---- the injected sequence (13.3.10) -----------------------------------------

    /**
     * The injected sequence: up to four channels, and JSQR IS FILLED FROM
     * THE TAIL. 13.13.12's note: with JL = 0 the converter runs JSQ4
     * alone, with JL = 1 it runs JSQ3 then JSQ4, and only a full four
     * starts at JSQ1. A caller that wrote its list at JSQ1 and set JL
     * would convert channels it never named - so this verb does the
     * placement and the trap disappears.
     *
     * Refused while a software-started conversion is in flight, for the
     * same erratum as the regular sequence.
     */
    static bool injected_sequence(const uint8_t* order, uint8_t count) {
        if (converting() || converting_injected()) {
            return false;
        }
        return injected_sequence_unchecked(order, count);
    }

    static bool injected_sequence_unchecked(const uint8_t* order, uint8_t count) {
        if (order == nullptr || count == 0u || count > 4u) {
            return false;
        }
        uint32_t w = static_cast<uint32_t>(count - 1u) << ADC_JSQR_JL_Pos;
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] >= channels) {
                return false;
            }
            const uint8_t slot = static_cast<uint8_t>(4u - count + i);   // the tail rule
            w |= static_cast<uint32_t>(order[i]) << (5u * slot);
        }
        regs().JSQR = w;
        injected_length_ = count;
        return true;
    }

    static uint8_t injected_length() {
        return static_cast<uint8_t>(((regs().JSQR & ADC_JSQR_JL_Msk) >> ADC_JSQR_JL_Pos) + 1u);
    }

    /// The channel in JSQ slot `slot` (1..4) as the register holds it -
    /// the raw slot, not the conversion order, so that a caller can see
    /// the tail placement for itself.
    static uint8_t injected_slot_channel(uint8_t slot) {
        if (slot < 1u || slot > 4u) {
            return 0xFFu;
        }
        return static_cast<uint8_t>((regs().JSQR >> (5u * (slot - 1u))) & 0x1Fu);
    }

    /**
     * ADC_JOFRx (13.13.6): a 12-bit value subtracted from the raw result
     * of the k-th injected conversion (k = 1..4), which is what makes an
     * injected result SIGNED - the only signed datapath this converter
     * has.
     */
    static bool injected_offset(uint8_t k, uint16_t offset) {
        if (k < 1u || k > 4u || offset > 0x0FFFu) {
            return false;
        }
        (&regs().JOFR1)[k - 1u] = offset;
        return true;
    }
    static uint16_t injected_offset(uint8_t k) {
        if (k < 1u || k > 4u) {
            return 0;
        }
        return static_cast<uint16_t>((&regs().JOFR1)[k - 1u] & 0x0FFFu);
    }

    /// ADC_JDRx as a SIGNED value: 13.4 says the offset can push a result
    /// below zero and SEXT carries the sign, so the register's sixteen
    /// bits are read as an int16_t rather than masked.
    static int16_t injected_result(uint8_t k) {
        if (k < 1u || k > 4u) {
            return 0;
        }
        return static_cast<int16_t>(static_cast<uint16_t>((&regs().JDR1)[k - 1u] & 0xFFFFu));
    }

    static void start_injected() { regs().CR2 |= ADC_CR2_JSWSTART; }
    static bool injected_ready() { return (regs().SR & AdcFlag::injected_converted) != 0u; }

    /**
     * One injected sequence, polled: start, wait for JEOC, hand back the
     * first result. False rather than a hang when it never arrives.
     *
     * A JDRx read acknowledges NOTHING - only ADC_DR has that property -
     * so this verb takes JEOC and JSTRT down itself, for the reason
     * `converting()` gives.
     */
    static bool read_injected(int16_t& out, uint32_t spins = 0x100000UL) {
        clear_flags(AdcFlag::injected_converted | AdcFlag::injected_started);
        start_injected();
        for (uint32_t i = 0; i < spins; ++i) {
            if (injected_ready()) {
                out = injected_result(1);
                clear_flags(AdcFlag::injected_converted | AdcFlag::injected_started);
                return true;
            }
        }
        return false;
    }

    // ---- conversions --------------------------------------------------------------

    /**
     * Whether a conversion is IN FLIGHT - and this is a DERIVED answer,
     * because the chapter gives this converter no busy bit: STRT says a
     * regular conversion has STARTED and stays up until software clears
     * it, EOC says one has finished and goes down when the datum is read.
     * So "started and not yet finished" is the proxy.
     *
     * IT IS ONLY EXACT BECAUSE `result()` ACKNOWLEDGES. Reading ADC_DR
     * clears EOC by itself, and this driver clears STRT in the same verb
     * - otherwise STRT, which nothing but software takes down, would
     * stand for the rest of the program and every sequence verb would
     * refuse for ever after the first conversion (measured: it does, and
     * the refusal is silent). A caller that starts and polls by hand owes
     * that acknowledgement itself.
     */
    static bool converting() {
        const uint32_t sr = regs().SR;
        return (sr & AdcFlag::started) != 0u && (sr & AdcFlag::converted) == 0u;
    }

    /// The same question for the injected group, whose own pair of flags
    /// is JSTRT and JEOC.
    static bool converting_injected() {
        const uint32_t sr = regs().SR;
        return (sr & AdcFlag::injected_started) != 0u &&
               (sr & AdcFlag::injected_converted) == 0u;
    }

    /// SWSTART. With EXTEN != none this is still the software way in -
    /// the two are independent - and 13.13.3 warns that it does nothing at
    /// all with ADON clear. Void, because util/analog_sampler.hpp asks for
    /// a void start().
    static void start() { regs().CR2 |= ADC_CR2_SWSTART; }

    static bool ready() { return (regs().SR & AdcFlag::converted) != 0u; }
    static bool started() { return (regs().SR & AdcFlag::started) != 0u; }
    static bool overrun() { return (regs().SR & AdcFlag::overrun) != 0u; }

    /**
     * ADC_DR. READING IT CLEARS EOC (13.13.1), which is the
     * acknowledgement the converter counts - so a caller that wants the
     * flag left standing reads `flags()` and not this.
     *
     * AND IT CLEARS STRT TOO, which the silicon does not: that bit is
     * set when a conversion starts and taken down by nothing but a write,
     * so a driver that left it standing would have no way of ever saying
     * again that a conversion is in flight (see `converting()`). One
     * rc_w0 store, and the pair means what it reads like.
     */
    static uint16_t result() {
        const uint16_t v = static_cast<uint16_t>(regs().DR);
        regs().SR = ~AdcFlag::started;
        return v;
    }

    /// One conversion, polled: clear EOC, start, wait, read.
    static bool read(uint16_t& out, uint32_t spins = 0x100000UL) {
        clear_flags(AdcFlag::converted);
        start();
        for (uint32_t i = 0; i < spins; ++i) {
            if (ready()) {
                out = result();
                return true;
            }
        }
        return false;
    }

    static uint16_t read(uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        (void)read(v, spins);
        return v;
    }

    /// Convert `count` times and keep the last - what a caller wants after
    /// waking an internal source or moving a pad.
    static uint16_t read_settled(uint8_t count, uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        for (uint8_t i = 0; i < count; ++i) {
            (void)read(v, spins);
        }
        return v;
    }

    /**
     * Stop a continuous run: CONT down, then one more conversion allowed
     * to finish and its result read away, so the next caller does not
     * find a stale EOC. Nothing here clears ADON - the converter stays
     * powered and configured.
     */
    static void stop(uint32_t spins = 0x100000UL) {
        regs().CR2 &= ~ADC_CR2_CONT;
        for (uint32_t i = 0; i < spins; ++i) {
            if (ready()) {
                break;
            }
        }
        (void)result();
        clear_flags(AdcFlag::all);
    }

    // ---- the arithmetic (util/analog.hpp's `steps` for this converter) ------------

    static uint32_t result_steps() { return adc_result_steps(cfg_); }

    /**
     * VREF+ in millivolts, MEASURED: the ratio the datasheet's calibration
     * value defines, VREF+ = 3.3 V x VREFINT_CAL / VREFINT_DATA. On a
     * board that ties VREF+ to VDDA - which is every board this stratum
     * has met - that is the analog supply.
     *
     * The caller supplies the VREFINT reading (a 12-bit right-aligned
     * one), because taking one here would mean this verb owning the
     * channel selection of whoever called it. Zero when the factory
     * values are not usable.
     */
    static uint16_t vdda_mv(uint16_t vrefint_data) {
        if (vrefint_data == 0u || !AdcFactory::plausible()) {
            return 0;
        }
        const uint32_t num = static_cast<uint32_t>(AdcFactory::characterization_mv) *
                             AdcFactory::vrefint_cal();
        return static_cast<uint16_t>((num + vrefint_data / 2u) / vrefint_data);
    }

    /**
     * The junction temperature in HUNDREDTHS of a degree Celsius, from the
     * two calibration points. Centi-degrees because no driver in this
     * stratum has floating point and one degree is a coarse answer for a
     * sensor whose own linearity is +/- 1 to 2 C.
     *
     * `ts_data` must be a 12-bit right-aligned reading and `vdda` the
     * supply it was taken at: ST's points are at 3.3 V and a count taken
     * at another supply is on another scale, so it is rescaled first.
     * That rescaling is the step the chapter's own formula leaves out.
     */
    static int32_t temperature_centi_c(uint16_t ts_data, uint16_t vdda) {
        if (!AdcFactory::plausible() || vdda == 0u) {
            return 0;
        }
        const int32_t cal1 = static_cast<int32_t>(AdcFactory::ts_cal1());
        const int32_t cal2 = static_cast<int32_t>(AdcFactory::ts_cal2());
        const int32_t scaled = static_cast<int32_t>(
            (static_cast<uint32_t>(ts_data) * vdda + AdcFactory::characterization_mv / 2u) /
            AdcFactory::characterization_mv);
        const int32_t span = (AdcFactory::ts_cal2_celsius - AdcFactory::ts_cal1_celsius) * 100;
        return ((scaled - cal1) * span) / (cal2 - cal1) + AdcFactory::ts_cal1_celsius * 100;
    }

    /**
     * The chapter's OWN formula instead of the calibration's (13.10):
     * (VSENSE - V25) / Avg_Slope + 25, in centi-degrees, with the
     * datasheet's typical numbers. It is here because a part class whose
     * calibration values nobody has read has nothing else, and because
     * the two answers disagreeing on one die is worth being able to see.
     */
    static int32_t temperature_centi_c_typical(uint16_t ts_mv) {
        const int32_t delta_uv =
            (static_cast<int32_t>(ts_mv) - static_cast<int32_t>(AdcFactory::typical_v25_mv)) * 1000;
        return (delta_uv * 100) / static_cast<int32_t>(AdcFactory::typical_slope_uv_per_c) + 2500;
    }

    // ---- the analog watchdog (13.3.8) -----------------------------------------------

    /**
     * ONE watchdog per converter, on one channel or on all of them, and
     * table 85 says which of AWDEN / JAWDEN / AWDSGL spell which case.
     * This verb takes the case as two booleans and the channel, and
     * writes the thresholds with them.
     *
     * 13.3.8: THE THRESHOLDS ARE COMPARED BEFORE ALIGNMENT, so they are
     * 12-bit numbers whatever ALIGN says - and, at a resolution below 12
     * bits, they are still compared against the converter's own 12-bit
     * datapath. Both refused above 0xFFF rather than truncated.
     */
    static bool watchdog(uint16_t low, uint16_t high, bool regular, bool injected, bool single,
                         uint8_t channel = 0) {
        if (low > 0x0FFFu || high > 0x0FFFu || (single && channel >= channels)) {
            return false;
        }
        if (!regular && !injected) {
            return false;   // table 85's first row is watchdog_off()
        }
        regs().HTR = high;
        regs().LTR = low;
        uint32_t c = regs().CR1 & ~watchdog_cr1_mask;
        if (regular) {
            c |= ADC_CR1_AWDEN;
        }
        if (injected) {
            c |= ADC_CR1_JAWDEN;
        }
        if (single) {
            c |= ADC_CR1_AWDSGL | (static_cast<uint32_t>(channel) << ADC_CR1_AWDCH_Pos);
        }
        regs().CR1 = c;
        return true;
    }

    static void watchdog_off() { regs().CR1 &= ~watchdog_cr1_mask; }

    /**
     * The thresholds alone, which are the one part of a watchdog that is
     * really live: 13.13.7's note says a write while a conversion runs
     * takes effect at the next one, "with a write delay that can create
     * uncertainty on the effective time". So a control loop moves its
     * limits without stopping the converter, and only the CHANNEL costs a
     * CR1 write.
     */
    static bool watchdog_thresholds(uint16_t low, uint16_t high) {
        if (low > 0x0FFFu || high > 0x0FFFu) {
            return false;
        }
        regs().HTR = high;
        regs().LTR = low;
        return true;
    }

    static uint16_t watchdog_high() { return static_cast<uint16_t>(regs().HTR & 0x0FFFu); }
    static uint16_t watchdog_low() { return static_cast<uint16_t>(regs().LTR & 0x0FFFu); }
    static uint8_t watchdog_channel() {
        return static_cast<uint8_t>((regs().CR1 & ADC_CR1_AWDCH_Msk) >> ADC_CR1_AWDCH_Pos);
    }

    // ---- flags and interrupts -------------------------------------------------------

    static uint32_t flags() { return regs().SR; }
    static bool flag(uint32_t mask) { return (regs().SR & mask) != 0u; }

    /**
     * ADC_SR is rc_w0 (13.13.1): a flag goes down when a ZERO is written
     * over it and stays up under a one. So this stores `~mask` - never a
     * read-modify-write, which would take a flag that rose between the
     * read and the write down with it.
     */
    static void clear_flags(uint32_t mask) { regs().SR = ~mask; }

    /**
     * The four interrupts of table 89, named by their FLAG so that one
     * mask serves both here and at `flags()`. The enables live in CR1 at
     * positions of their own (AWDIE 6, EOCIE 5, JEOCIE 7, OVRIE 26), which
     * is why this is a translation and not a store.
     */
    static void interrupts(uint32_t flag_mask, bool on) {
        const uint32_t bits = enable_bits(flag_mask);
        regs().CR1 = on ? (regs().CR1 | bits) : (regs().CR1 & ~bits);
    }

    /// Which FLAGS are armed right now - the inverse translation, and what
    /// the ISR body masks with.
    static uint32_t armed() {
        const uint32_t c = regs().CR1;
        uint32_t m = 0;
        if ((c & ADC_CR1_AWDIE) != 0u) m |= AdcFlag::watchdog;
        if ((c & ADC_CR1_EOCIE) != 0u) m |= AdcFlag::converted;
        if ((c & ADC_CR1_JEOCIE) != 0u) m |= AdcFlag::injected_converted;
        if ((c & ADC_CR1_OVRIE) != 0u) m |= AdcFlag::overrun;
        return m;
    }

    /**
     * The ISR BODY: the flags this converter has ARMED, cleared and handed
     * back. The vector is shared by every converter, so a handler is a
     * dispatcher and this answers 0 when this one did not speak.
     *
     * IT DOES NOT READ ADC_DR - the result is the handler's to take, and
     * reading it here would clear EOC behind the handler's back. Which
     * means a handler that reports EOC and never reads the data leaves
     * EOC down and the datum standing, exactly as intended.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t hit = regs().SR & armed();
        if (hit != 0u) {
            regs().SR = ~hit;
        }
        return hit;
    }

private:
    /// The CR1 bits the watchdog owns - kept across a `configure()`,
    /// because arming a window and configuring the converter are two
    /// decisions and neither should undo the other.
    static constexpr uint32_t watchdog_cr1_mask =
        ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDSGL | ADC_CR1_AWDCH_Msk;

    static constexpr uint32_t enable_bits(uint32_t flag_mask) {
        uint32_t bits = 0;
        if ((flag_mask & AdcFlag::watchdog) != 0u) bits |= ADC_CR1_AWDIE;
        if ((flag_mask & AdcFlag::converted) != 0u) bits |= ADC_CR1_EOCIE;
        if ((flag_mask & AdcFlag::injected_converted) != 0u) bits |= ADC_CR1_JEOCIE;
        if ((flag_mask & AdcFlag::overrun) != 0u) bits |= ADC_CR1_OVRIE;
        return bits;
    }

    static constexpr uint32_t cr1_word(const AdcConfig& c) {
        uint32_t w = static_cast<uint32_t>(c.resolution) << ADC_CR1_RES_Pos;
        if (c.scan) w |= ADC_CR1_SCAN;
        if (c.discontinuous) {
            w |= ADC_CR1_DISCEN;
            w |= static_cast<uint32_t>(c.discontinuous_count - 1u) << ADC_CR1_DISCNUM_Pos;
        }
        if (c.injected_discontinuous) w |= ADC_CR1_JDISCEN;
        if (c.auto_injected) w |= ADC_CR1_JAUTO;
        return w;
    }

    static constexpr uint32_t cr2_word(const AdcConfig& c) {
        uint32_t w = 0;
        if (c.left_aligned) w |= ADC_CR2_ALIGN;
        if (c.eoc_per_conversion) w |= ADC_CR2_EOCS;
        if (c.dma) w |= ADC_CR2_DMA;
        if (c.dma_continuous_requests) w |= ADC_CR2_DDS;
        if (c.continuous) w |= ADC_CR2_CONT;
        w |= static_cast<uint32_t>(c.trigger) << ADC_CR2_EXTSEL_Pos;
        w |= static_cast<uint32_t>(c.trigger_edge) << ADC_CR2_EXTEN_Pos;
        w |= static_cast<uint32_t>(c.injected_trigger) << ADC_CR2_JEXTSEL_Pos;
        w |= static_cast<uint32_t>(c.injected_trigger_edge) << ADC_CR2_JEXTEN_Pos;
        return w;
    }

    inline static AdcConfig cfg_{};
    inline static uint8_t selected_ = 0;
    inline static uint8_t length_ = 1;
    inline static uint8_t injected_length_ = 1;
};

} // namespace brio
