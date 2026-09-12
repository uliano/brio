/*
 * adc.hpp
 *
 * The ADC of the CH32V00x (RM ch. 9 of each part's manual): one
 * successive-approximation converter over eight pads and two internal
 * sources, a rule group of up to sixteen conversions and an injection
 * group of four, timer triggers a group, an analog watchdog, one DMA
 * request - the STM32F1's ADC1 under WCH's names, 12-bit on the
 * CH32V006 with a third control register of its own, 10-bit on the
 * CH32V003 with the F1's calibration and a trigger delay register. In
 * the two strata every brio target uses:
 *
 *  Adc            the RESOURCE, a monostate (every part has exactly one):
 *                 the clock, the power-up, the sample times, both
 *                 groups, the triggers, the watchdogs, the flags and
 *                 the ISR body - and util/analog_sampler.hpp's converter
 *                 surface (select / start / selected / input_code).
 *
 *  AnalogIn<Pin>  a pad handed to its channel: the channel NUMBER comes
 *                 from the pad (DS table 2-1-1, the same eight pads and
 *                 channels on both parts), stated here once.
 *
 *  AdcInput       the two internal sources: VREFINT on channel 8 (1.2 V
 *                 nominal, DS table 3-5) on both parts; channel 9 is
 *                 the OPA's output on the CH32V006 and Vcal, a fraction
 *                 of AVDD, on the CH32V003 - each part's own enum.
 *
 * WHAT THE F1 HAS NOT, ON THE CH32V006. ADC_CTLR3: a LOW-POWER bit
 * (ADC_LP, set at reset, the mode for rates under a megasample - and
 * the mode the SAMPLE TIME TABLE depends on, 9.3.4 giving two
 * columns), a clock duty-cycle knob, and two more watchdogs (AWD1 and
 * AWD2 with their own threshold registers, compare results in CTLR3
 * cleared by writing zero) beside the F1's AWD0 - EACH OF THE THREE
 * ABLE TO RESET THE SYSTEM when its comparison fails (AWDx_RST_EN; the
 * boot reads ADCRSTF in RCC_RSTSCKR, reset.hpp's ResetFlag::adc),
 * which is what 9's opening paragraph calls "the reset and protection
 * system". What the three guard is the chapter's least clear sentence
 * and the bench's finding: under AWD_SCAN each watchdog takes ONE RANK
 * of a scanned rule sequence - the first conversion against watchdog
 * 0's thresholds, the second against watchdog 1's, the third against
 * watchdog 2's. Also an INPUT BUFFER (CTLR1.BUFEN) for sources above
 * the 50 kOhm the sampling switch tolerates. What the F1 has and this
 * part has not: a calibration - no CAL bit, no procedure, the vendor's
 * own init runs none; and a second sample-time register, the ten
 * channels fitting in SAMPTR2 alone.
 *
 * WHAT THE CH32V003 HAS INSTEAD (device::adc_*): the F1's whole shape
 * - 10 bits, a conversion of the sample time plus 11 ADCCLK cycles
 * with the sample table in whole cycles (3 to 241), the CALIBRATION
 * (RSTCAL, then CAL, each cleared by the hardware; the chapter
 * recommends one at every power-up and init() runs it), the Vcal
 * source on channel 9 at 2/4 or 3/4 of AVDD (CTLR1.CALVOL, 01 at
 * reset and "invalid" at 00 - init() writes it, never zero), a
 * 10-bit watchdog 0 and nothing above it, and at 0x50 the DELAY
 * register DLYR (9.3.15: an external trigger delayed by up to 511
 * ADCCLK cycles, for the rule or the injection group) where the
 * CH32V006 has CTLR3. No CTLR3 means no low-power mode, no input
 * buffer, no watchdog 1 or 2, no watchdog reset - each of those verbs
 * answers false or writes nothing there, and a config that asks is
 * refused. CTLR2 has no TGREGU/TGINJE either: the OPA is not a trigger
 * source. The injection group's TIM3 triggers name a timer the part has
 * not (table 9-4's codes 100 and 101 are blank) and are refused.
 *
 * THE CLOCK. ADCCLK is HCLK through RCC_CFGR0.ADCPRE, a five-bit code
 * whose dividers are a two-level table (/2 to /128, twelve distinct
 * values), the same encoding on both parts; the datasheet bounds fADC
 * at 48 MHz on the CH32V006 and 12 MHz on the CH32V003
 * (device::adc_max_clock_hz). A conversion is the sample time plus
 * 12.5 ADCCLK cycles on the CH32V006 (9.2.2) and plus 11 on the
 * CH32V003 (its 9.2.2) - adc_conversion_half_cycles() says which.
 *
 * THE POWER-UP IS THE F1's: ADON written once wakes the converter,
 * tSTAB passes, and ADON written AGAIN with nothing else changed is a
 * START. A software start is therefore either that second write (no
 * trigger armed) or SWSTART under EXTSEL = 111 with EXTTRIG set - this
 * driver arms the software trigger in init(), so `start()` is one
 * spelling whatever the group.
 *
 * THE INJECTION GROUP subtracts a per-slot offset and stores a SIGNED
 * result (table 9-1/9-2's SIGNB), so `injected_result()` returns
 * int16_t; the rule group's RDATAR is unsigned. The injection group
 * takes no DMA (9.2.2's note).
 *
 * THE STATUS FLAGS are write-zero-to-clear (STATR's RW0), the same
 * discipline as the timers' INTFR.
 *
 * NOT COVERED YET: TouchKey (TKENABLE/TKITUNE, the DRV bits of CTLR3 -
 * an application-level mode, declined until one wants it); the OPA as
 * a trigger source (TGREGU/TGINJE; the OPA chapter's); the discontinuous
 * mode beyond its configuration bits (no user); the external trigger
 * pads (PD3/PC2 for the rule group, PD1/PA2 for the injection one) as
 * a measured path - a pad and a source - and with them the CH32V003's
 * trigger delay, which only an external trigger exercises.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "util/analog.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// The registers (table 9-7 of each manual)
// =============================================================================

/// One layout for both parts: the F1's block to RDATAR, then the word
/// at 0x50 the two parts spend differently and the two words the
/// CH32V006 alone has. The driver reaches the part-specific words under
/// `if constexpr` on the facts alone.
struct AdcRegs {
    volatile uint32_t STATR;      ///< 0x00 write zero to clear
    volatile uint32_t CTLR1;      ///< 0x04
    volatile uint32_t CTLR2;      ///< 0x08
    volatile uint32_t SAMPTR1;    ///< 0x0c the F1's channels 10..17: none on either part (reserved on the CH32V006)
    volatile uint32_t SAMPTR2;    ///< 0x10 SMP0..SMP9
    volatile uint32_t IOFR[4];    ///< 0x14..0x20 the injection offsets
    volatile uint32_t WDHTR;      ///< 0x24 watchdog 0's high threshold
    volatile uint32_t WDLTR;      ///< 0x28 watchdog 0's low threshold
    volatile uint32_t RSQR1;      ///< 0x2c L and SQ13..16
    volatile uint32_t RSQR2;      ///< 0x30 SQ7..12
    volatile uint32_t RSQR3;      ///< 0x34 SQ1..6
    volatile uint32_t ISQR;       ///< 0x38 JL and JSQ1..4
    volatile uint32_t IDATAR[4];  ///< 0x3c..0x48 the injection results
    volatile uint32_t RDATAR;     ///< 0x4c the rule result
    volatile uint32_t CTLR3_DLYR; ///< 0x50 CTLR3 on the CH32V006, DLYR on the CH32V003
    volatile uint32_t WDTR1;      ///< 0x54 watchdog 1's thresholds (CH32V006 alone)
    volatile uint32_t WDTR2;      ///< 0x58 watchdog 2's thresholds (CH32V006 alone)
};

inline constexpr uint32_t adc_base = pb2_base + 0x2400;
inline AdcRegs* adc() { return reinterpret_cast<AdcRegs*>(adc_base); }

// STATR (9.3.1)
inline constexpr uint32_t adc_awd    = 1UL << 0;
inline constexpr uint32_t adc_eoc    = 1UL << 1;
inline constexpr uint32_t adc_jeoc   = 1UL << 2;
inline constexpr uint32_t adc_jstrt  = 1UL << 3;
inline constexpr uint32_t adc_strt   = 1UL << 4;
// CTLR1 (9.3.2)
inline constexpr uint32_t adc_awdch_mask = 0x1FUL;
inline constexpr uint32_t adc_eocie  = 1UL << 5;
inline constexpr uint32_t adc_awdie  = 1UL << 6;
inline constexpr uint32_t adc_jeocie = 1UL << 7;
inline constexpr uint32_t adc_scan   = 1UL << 8;
inline constexpr uint32_t adc_awdsgl = 1UL << 9;
inline constexpr uint32_t adc_jauto  = 1UL << 10;
inline constexpr uint32_t adc_discen = 1UL << 11;
inline constexpr uint32_t adc_jdiscen = 1UL << 12;
inline constexpr uint32_t adc_discnum_mask = 7UL << 13;
inline constexpr uint32_t adc_jawden = 1UL << 22;
inline constexpr uint32_t adc_awden  = 1UL << 23;
inline constexpr uint32_t adc_tkenable = 1UL << 24;   ///< CH32V006
inline constexpr uint32_t adc_tkitune = 1UL << 25;    ///< CH32V006
inline constexpr uint32_t adc_bufen  = 1UL << 26;     ///< CH32V006
inline constexpr uint32_t adc_calvol_mask = 3UL << 25;   ///< CH32V003: the Vcal fraction
// CTLR2 (9.3.3)
inline constexpr uint32_t adc_adon   = 1UL << 0;
inline constexpr uint32_t adc_cont   = 1UL << 1;
inline constexpr uint32_t adc_cal    = 1UL << 2;      ///< CH32V003
inline constexpr uint32_t adc_rstcal = 1UL << 3;      ///< CH32V003
inline constexpr uint32_t adc_tgregu = 1UL << 4;      ///< CH32V006
inline constexpr uint32_t adc_tginje = 1UL << 5;      ///< CH32V006
inline constexpr uint32_t adc_dma    = 1UL << 8;
inline constexpr uint32_t adc_align  = 1UL << 11;
inline constexpr uint32_t adc_jextsel_mask = 7UL << 12;
inline constexpr uint32_t adc_jexttrig = 1UL << 15;
inline constexpr uint32_t adc_extsel_mask = 7UL << 17;
inline constexpr uint32_t adc_exttrig = 1UL << 20;
inline constexpr uint32_t adc_jswstart = 1UL << 21;
inline constexpr uint32_t adc_swstart = 1UL << 22;
// CTLR3 (9.3.14 of the CH32V006's manual)
inline constexpr uint32_t adc_lp     = 1UL << 0;
inline constexpr uint32_t adc_dutyen = 1UL << 1;
inline constexpr uint32_t adc_drven  = 1UL << 2;
inline constexpr uint32_t adc_awd_scan = 1UL << 3;
inline constexpr uint32_t adc_awd0_rst_en = 1UL << 4;
inline constexpr uint32_t adc_awd1_rst_en = 1UL << 5;
inline constexpr uint32_t adc_awd2_rst_en = 1UL << 6;
inline constexpr uint32_t adc_awd0_res = 1UL << 8;
inline constexpr uint32_t adc_awd1_res = 1UL << 9;
inline constexpr uint32_t adc_awd2_res = 1UL << 10;
// DLYR (9.3.15 of the CH32V003's manual)
inline constexpr uint32_t adc_dlyvlu_mask = 0x1FFUL;
inline constexpr uint32_t adc_dlysrc = 1UL << 9;      ///< the delay is the injection group's

// =============================================================================
// The vocabulary
// =============================================================================

inline constexpr uint8_t adc_channels = 10;      ///< 0..7 the pads, 8 VREFINT, 9 the part's
inline constexpr uint8_t adc_pad_channels = 8;
inline constexpr uint32_t adc_bits = device::adc_bits;
inline constexpr uint32_t adc_steps = 1UL << adc_bits;   ///< util/analog.hpp's full scale
inline constexpr uint16_t adc_max_count = static_cast<uint16_t>(adc_steps - 1u);
inline constexpr uint16_t adc_vrefint_mv = 1200; ///< DS table 3-5, 1.18..1.22 V (1.17..1.23 on the CH32V003)

/// The two internal sources (9.2.2), each part's own channel 9.
enum class AdcInputCh32v006 : uint8_t { vrefint = 8, opa = 9 };
enum class AdcInputCh32v003 : uint8_t { vrefint = 8, vcal = 9 };
using AdcInput = std::conditional_t<device::part == Ch32Part::v003, AdcInputCh32v003, AdcInputCh32v006>;

/// CTLR1.CALVOL (the CH32V003): what Vcal is, as a fraction of AVDD.
enum class AdcCalVoltage : uint8_t { half = 1, three_quarters = 2 };
constexpr uint32_t adc_cal_voltage_numerator(AdcCalVoltage v) { return v == AdcCalVoltage::half ? 2u : 3u; }   // over 4

/// util/analog.hpp's vocabulary on this target: the converter's
/// reference IS the supply, and there is no other.
enum class Ref : uint8_t { vdd };

/// SMPx[2:0]: the sample time in ADCCLK cycles, each part's own table.
/// The CH32V006's depends on CTLR3.ADC_LP (9.3.4): the codes are the
/// same, the cycles differ at three of them - named by the low-power
/// column, the reset one. The CH32V003's is in whole cycles.
enum class AdcSampleTimeCh32v006 : uint8_t {
    cycles3_5 = 0, cycles7_5 = 1, cycles13_5 = 2, cycles28_5 = 3,
    cycles41_5 = 4, cycles55_5 = 5, cycles71_5 = 6, cycles239_5 = 7,
};
enum class AdcSampleTimeCh32v003 : uint8_t {
    cycles3 = 0, cycles9 = 1, cycles15 = 2, cycles30 = 3,
    cycles43 = 4, cycles57 = 5, cycles73 = 6, cycles241 = 7,
};
using AdcSampleTime =
    std::conditional_t<device::part == Ch32Part::v003, AdcSampleTimeCh32v003, AdcSampleTimeCh32v006>;
/// The two ends of either table, for a program that wants a sample
/// time without naming a count.
inline constexpr AdcSampleTime adc_sample_shortest = static_cast<AdcSampleTime>(0);
inline constexpr AdcSampleTime adc_sample_longest = static_cast<AdcSampleTime>(7);

/// The sample time in HALF ADCCLK cycles, so the CH32V006's .5 stays exact.
constexpr uint32_t adc_sample_half_cycles(AdcSampleTime t, bool low_power = device::adc_has_ctlr3) {
    if constexpr (device::part == Ch32Part::v003) {
        constexpr uint16_t table[8] = {6, 18, 30, 60, 86, 114, 146, 482};
        (void)low_power;
        return table[static_cast<uint8_t>(t) & 7u];
    } else {
        constexpr uint16_t lp[8] = {7, 15, 27, 57, 83, 111, 143, 479};
        constexpr uint16_t hp[8] = {7, 15, 23, 39, 71, 111, 143, 479};
        return (low_power ? lp : hp)[static_cast<uint8_t>(t) & 7u];
    }
}
/// The conversion's own cycles beyond the sample, in halves: 12.5 on
/// the CH32V006 (9.2.2), 11 on the CH32V003 (its 9.2.2).
inline constexpr uint32_t adc_conversion_tail_half_cycles = device::part == Ch32Part::v003 ? 22u : 25u;
/// tCONV in half ADCCLK cycles.
constexpr uint32_t adc_conversion_half_cycles(AdcSampleTime t, bool low_power = device::adc_has_ctlr3) {
    return adc_sample_half_cycles(t, low_power) + adc_conversion_tail_half_cycles;
}

/// RCC_CFGR0.ADCPRE (3.4.2 of both manuals): a two-level code. The
/// twelve dividers it can produce, each by its lowest code.
struct AdcPrescaler {
    uint8_t code;
    uint8_t divider;
};
inline constexpr AdcPrescaler adc_prescalers[] = {
    {0x00, 2}, {0x08, 4}, {0x10, 6}, {0x18, 8}, {0x14, 12}, {0x1C, 16},
    {0x15, 24}, {0x1D, 32}, {0x16, 48}, {0x1E, 64}, {0x17, 96}, {0x1F, 128},
};

/// The divider a code means.
constexpr uint8_t adc_prescaler_divider(uint8_t code) {
    const uint8_t hi = static_cast<uint8_t>((code >> 2) & 7u);
    const uint8_t lo = static_cast<uint8_t>(code & 3u);
    if ((hi & 1u) == 0u) {
        return static_cast<uint8_t>(2u + (hi >> 1) * 2u);   // 000 /2, 010 /4, 100 /6, 110 /8
    }
    return static_cast<uint8_t>((4u + (hi >> 1) * 4u) << lo);   // 001 /4, 011 /8, 101 /12, 111 /16, then x2, x4, x8
}

constexpr uint32_t adc_clock_hz(uint32_t hclk, uint8_t code) { return hclk / adc_prescaler_divider(code); }

/// The smallest divider whose ADCCLK is at most `max_hz`.
constexpr std::optional<AdcPrescaler> adc_prescaler_for(uint32_t hclk, uint32_t max_hz) {
    for (const AdcPrescaler& p : adc_prescalers) {
        if (hclk / p.divider <= max_hz) {
            return p;
        }
    }
    return {};
}

/// DS: fADC at most 48 MHz on the CH32V006, 12 MHz on the CH32V003.
inline constexpr uint32_t adc_max_clock_hz = device::adc_max_clock_hz;

/// Table 9-3: the rule group's triggers (EXTSEL), the same on both parts.
enum class AdcTrigger : uint8_t {
    tim1_trgo = 0, tim1_cc1 = 1, tim1_cc2 = 2, tim2_trgo = 3, tim2_cc1 = 4, tim2_cc2 = 5,
    external = 6,   ///< the PD3/PC2 pad, or the OPA under TGREGU (CH32V006)
    software = 7,
};
/// Table 9-4: the injection group's (JEXTSEL). The TIM3 codes are the
/// CH32V006's; the CH32V003's table leaves them blank and the driver
/// refuses them there.
enum class AdcInjectedTrigger : uint8_t {
    tim1_cc3 = 0, tim1_cc4 = 1, tim2_cc3 = 2, tim2_cc4 = 3, tim3_cc1 = 4, tim3_cc2 = 5,
    external = 6,   ///< the PD1/PA2 pad, or the OPA under TGINJE (CH32V006)
    software = 7,
};
constexpr bool adc_injected_trigger_valid(AdcInjectedTrigger t) {
    return device::has_tim3 || (t != AdcInjectedTrigger::tim3_cc1 && t != AdcInjectedTrigger::tim3_cc2);
}

struct AdcFlag {
    static constexpr uint32_t watchdog = adc_awd;
    static constexpr uint32_t converted = adc_eoc;
    static constexpr uint32_t injected = adc_jeoc;
    static constexpr uint32_t injected_started = adc_jstrt;
    static constexpr uint32_t started = adc_strt;
    static constexpr uint32_t all = adc_awd | adc_eoc | adc_jeoc | adc_jstrt | adc_strt;
};

struct AdcConfig {
    /// ADCPRE's code (adc_prescaler_for). /8 at 48 MHz is 6 MHz, the
    /// vendor's own choice on both parts; /2 is 24 MHz, legal on the
    /// CH32V006 alone.
    uint8_t prescaler_code = 0x18;
    bool left_aligned = false;         ///< CTLR2.ALIGN
    bool continuous = false;           ///< CTLR2.CONT
    bool scan = false;                 ///< CTLR1.SCAN: the whole group, one conversion after another
    bool auto_injected = false;        ///< CTLR1.JAUTO: the injection group after the rule group
    /// CTLR3.ADC_LP (CH32V006): the reset value, for rates under 1 Msps.
    /// The CH32V003 has no such mode: false is the only value there.
    bool low_power = device::adc_has_ctlr3;
    bool input_buffer = false;         ///< CTLR1.BUFEN (CH32V006)
    bool dma = false;                  ///< CTLR2.DMA: a request per rule conversion
    /// Discontinuous mode: 0 = off, 1..8 = the short sequence's length.
    uint8_t discontinuous = 0;
    bool injected_discontinuous = false;
    /// CTLR1.CALVOL (CH32V003): what channel 9 reads. The CH32V006 has
    /// no such field: `half`, the register's reset code, is the only
    /// value there.
    AdcCalVoltage cal_voltage = AdcCalVoltage::half;
};

constexpr bool adc_config_valid(const AdcConfig& c) {
    if (c.discontinuous > 8u) {
        return false;
    }
    if (c.discontinuous != 0u && c.injected_discontinuous) {
        return false;   // 9.2.4's note 3: one group or the other
    }
    if (c.auto_injected && (c.discontinuous != 0u || c.injected_discontinuous)) {
        return false;   // note 2
    }
    if constexpr (!device::adc_has_ctlr3) {
        if (c.low_power || c.input_buffer) {
            return false;   // no CTLR3, no BUFEN
        }
    }
    if constexpr (!device::adc_has_calibration) {
        if (c.cal_voltage != AdcCalVoltage::half) {
            return false;   // no CALVOL
        }
    }
    return true;
}

/// The watchdogs (9.2.5; the CH32V006's 9.3.14). Watchdog 0 is the
/// F1's, with its channel selection; on the CH32V006 1 and 2 have
/// thresholds of their own and their results in CTLR3.
struct AdcWatchdogConfig {
    uint16_t low = 0;                  ///< adc_bits wide
    uint16_t high = adc_max_count;
    /// Watchdog 0's scope: every channel of the groups it guards, or one
    /// channel (AWDSGL + AWDCH).
    std::optional<uint8_t> channel{};
    bool rule_group = true;            ///< AWDEN
    bool injected_group = false;       ///< JAWDEN
    bool interrupt = false;            ///< AWDIE
    /// AWDx_RST_EN (CH32V006): a comparison that fails RESETS THE CHIP.
    /// Refused on the CH32V003, which has no such reset.
    bool reset_on_fault = false;
};

constexpr bool adc_watchdog_config_valid(const AdcWatchdogConfig& w) {
    return w.low <= adc_max_count && w.high <= adc_max_count && w.low <= w.high &&
           (!w.channel || *w.channel < adc_channels) && (device::adc_has_ctlr3 || !w.reset_on_fault);
}

/// The pads and their channels (DS table 2-1-1 of each part, the same
/// eight): the pad SAYS the channel, so a wrong one cannot be written.
constexpr uint8_t adc_channel_of(Pad p) {
    if (p == Pad{'A', 2}) { return 0; }
    if (p == Pad{'A', 1}) { return 1; }
    if (p == Pad{'C', 4}) { return 2; }
    if (p == Pad{'D', 2}) { return 3; }
    if (p == Pad{'D', 3}) { return 4; }
    if (p == Pad{'D', 5}) { return 5; }
    if (p == Pad{'D', 6}) { return 6; }
    if (p == Pad{'D', 4}) { return 7; }
    return 0xFF;
}

/**
 * AnalogIn<Pin>: the claim that this pad is an ADC input - refused at
 * compile time for a pad that is not one. `claim()` puts the pad in
 * analog mode (the reset state, the input buffer off).
 */
template <class P>
struct AnalogIn {
    static_assert(adc_channel_of(P::pad) != 0xFFu,
                  "brio AnalogIn: this pad is not an ADC input on the CH32V00x (PA2, PA1, "
                  "PC4, PD2, PD3, PD5, PD6, PD4 are, channels 0..7 in that order)");
    using pin = P;
    static constexpr uint8_t channel = adc_channel_of(P::pad);

    static void claim() { P::analog(); }
    static void release() { P::analog(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Adc: the one converter, a monostate.
 *
 *   using Vin = brio::AnalogIn<brio::Pin<'D', 2>>;   // channel 3
 *   Adc::init(clock, {.prescaler_code = brio::adc_prescaler_for(48'000'000, 6'000'000)->code});
 *   Adc::sample_time(Vin::channel, brio::adc_sample_longest);
 *   Adc::select(Vin{});
 *   const uint16_t counts = Adc::read();
 *
 * util/analog_sampler.hpp drives `select()`, `start()`, `selected()`
 * and `input_code()`; the rest is the chapter's.
 */
struct Adc {
    Adc() = delete;

    static constexpr uint8_t channels = adc_channels;
    static constexpr uint8_t vrefint_channel = 8;
    /// The channel the OPA's output is on: 9 on the CH32V006 (an
    /// internal route), 7 on the CH32V003 (PD4's own, through the pad).
    static constexpr uint8_t opa_channel = device::opa_adc_channel;
    /// Vcal, the CH32V003's channel 9; nothing of that name on the CH32V006.
    static constexpr std::optional<uint8_t> vcal_channel =
        device::adc_has_calibration ? std::optional<uint8_t>{9} : std::nullopt;
    static constexpr Irq irq() { return Irq::adc; }
    /// Table 8-2 of both manuals: the rule group's request reaches DMA channel 1.
    static constexpr uint8_t dma_channel = 1;
    static volatile void* data_address() { return &adc()->RDATAR; }

    static AdcRegs& regs() { return *adc(); }

    static void bus_clock(bool on) {
        if (on) { rcc()->PB2PCENR |= rcc_pb2_adc1; } else { rcc()->PB2PCENR &= ~rcc_pb2_adc1; }
    }
    static void reset() {
        rcc()->PB2PRSTR |= rcc_pb2_adc1;
        rcc()->PB2PRSTR &= ~rcc_pb2_adc1;
    }

    /// RCC_CFGR0.ADCPRE.
    static void prescaler(uint8_t code) {
        rcc()->CFGR0 = (rcc()->CFGR0 & ~rcc_adcpre_mask) | (static_cast<uint32_t>(code & 0x1Fu) << 11);
    }
    static uint8_t prescaler() { return static_cast<uint8_t>((rcc()->CFGR0 & rcc_adcpre_mask) >> 11); }
    /// ADCCLK as it stands, from the clock the caller states.
    template <typename Clock>
    static uint32_t adcclk_hz(Clock clock) { return adc_clock_hz(clock_hz_of(clock), prescaler()); }

    /// Bring the converter up: gate, reset, the prescaler, the control
    /// words, the power-up write and tSTAB, the software trigger armed
    /// - and on the CH32V003 the calibration the chapter recommends at
    /// every power-up. False for a config the chapter refuses, an
    /// ADCCLK above the datasheet's bound, or a calibration that never
    /// completed.
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& c = {}) {
        if (!adc_config_valid(c) || adc_clock_hz(clock_hz_of(clock), c.prescaler_code) > adc_max_clock_hz) {
            return false;
        }
        Pfic::disable(irq());
        bus_clock(true);
        reset();
        prescaler(c.prescaler_code);
        cfg_ = c;
        AdcRegs& r = regs();
        uint32_t c1 = 0;
        if (c.scan) { c1 |= adc_scan; }
        if (c.auto_injected) { c1 |= adc_jauto; }
        if (c.discontinuous != 0u) {
            c1 |= adc_discen | (static_cast<uint32_t>(c.discontinuous - 1u) << 13);
        }
        if (c.injected_discontinuous) { c1 |= adc_jdiscen; }
        if (c.input_buffer) { c1 |= adc_bufen; }
        if constexpr (device::adc_has_calibration) {
            c1 |= static_cast<uint32_t>(c.cal_voltage) << 25;   // never the invalid 00
        }
        r.CTLR1 = c1;
        uint32_t c2 = adc_exttrig | adc_extsel_mask | adc_jexttrig | adc_jextsel_mask;   // both software triggers
        if (c.left_aligned) { c2 |= adc_align; }
        if (c.continuous) { c2 |= adc_cont; }
        if (c.dma) { c2 |= adc_dma; }
        r.CTLR2 = c2;
        if constexpr (device::adc_has_ctlr3) {
            r.CTLR3_DLYR = c.low_power ? (r.CTLR3_DLYR | adc_lp) : (r.CTLR3_DLYR & ~adc_lp);
        }
        // One conversion of channel 0, the reset sequence, until a select.
        r.RSQR1 = 0;
        r.RSQR3 = 0;
        selected_ = 0;
        r.CTLR2 |= adc_adon;   // the wake-up write
        stab_spin();
        if constexpr (device::adc_has_calibration) {
            if (!calibrate()) {
                return false;
            }
        }
        r.STATR = 0;
        return true;
    }

    static void release() {
        Pfic::disable(irq());
        regs().CTLR2 &= ~adc_adon;
        reset();
        bus_clock(false);
    }

    /// THE CALIBRATION (the CH32V003's 9.2.2 step 4): RSTCAL until the
    /// hardware clears it, then CAL until the hardware clears it, with
    /// the converter powered for at least two ADCCLK cycles first;
    /// the calibration code lands in RDATAR. False when either bit
    /// never cleared - and on the CH32V006, which has no calibration
    /// (device::adc_has_calibration), false without a write.
    static bool calibrate(uint32_t spins = 0x100000UL) {
        if constexpr (device::adc_has_calibration) {
            AdcRegs& r = regs();
            r.CTLR2 |= adc_rstcal;
            uint32_t n = spins;
            while ((r.CTLR2 & adc_rstcal) != 0u && n-- != 0u) {
            }
            if ((r.CTLR2 & adc_rstcal) != 0u) {
                return false;
            }
            r.CTLR2 |= adc_cal;
            n = spins;
            while ((r.CTLR2 & adc_cal) != 0u && n-- != 0u) {
            }
            return (r.CTLR2 & adc_cal) == 0u;
        } else {
            (void)spins;
            return false;
        }
    }

    /// THE STALL, AND ITS REMEDY. With a rule group paced by a hardware
    /// trigger and served by the DMA, and the CPU accessing the
    /// peripheral buses meanwhile, the converter has been seen to STOP
    /// CONVERTING (STRT standing, no EOC ever again) within the first
    /// conversions of a run - at rates from none to a tenth of the runs
    /// depending on the CPU's access pattern, and once in two hundred
    /// with an EOC interrupt reader and no DMA. Neither a read of RDATAR
    /// nor a software start revives it; a power cycle of ADON does.
    /// This verb is that cycle; a stream that must not die watches its
    /// own progress and calls it.
    static void recover() {
        power(false);
        power(true);
    }

    static bool powered() { return (regs().CTLR2 & adc_adon) != 0u; }
    static void power(bool on) {
        if (on) {
            if (!powered()) {
                regs().CTLR2 |= adc_adon;
                stab_spin();
            }
        } else {
            regs().CTLR2 &= ~adc_adon;
        }
    }

    // ---- the sample times ----------------------------------------------------

    static bool sample_time(uint8_t ch, AdcSampleTime t) {
        if (ch >= channels) {
            return false;
        }
        AdcRegs& r = regs();
        const uint32_t shift = 3u * ch;
        r.SAMPTR2 = (r.SAMPTR2 & ~(7UL << shift)) | (static_cast<uint32_t>(t) << shift);
        return true;
    }
    static AdcSampleTime sample_time(uint8_t ch) {
        return static_cast<AdcSampleTime>((regs().SAMPTR2 >> (3u * ch)) & 7u);
    }
    static void sample_time_all(AdcSampleTime t) {
        uint32_t v = 0;
        for (uint8_t ch = 0; ch < channels; ++ch) {
            v |= static_cast<uint32_t>(t) << (3u * ch);
        }
        regs().SAMPTR2 = v;
    }
    /// tCONV of channel `ch` in half ADCCLK cycles under the config in force.
    static uint32_t conversion_half_cycles(uint8_t ch) {
        return adc_conversion_half_cycles(sample_time(ch), cfg_.low_power);
    }

    // ---- the rule group ------------------------------------------------------

    /// The rule sequence: `count` channels in `order`, 1..16 (9.3.8).
    static bool sequence(const uint8_t* order, uint8_t count) {
        if (count == 0u || count > 16u) {
            return false;
        }
        uint32_t r1 = static_cast<uint32_t>(count - 1u) << 20;
        uint32_t r2 = 0;
        uint32_t r3 = 0;
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] >= channels) {
                return false;
            }
            const uint32_t ch = order[i];
            if (i < 6u) { r3 |= ch << (5u * i); }
            else if (i < 12u) { r2 |= ch << (5u * (i - 6u)); }
            else { r1 |= ch << (5u * (i - 12u)); }
        }
        AdcRegs& r = regs();
        r.RSQR3 = r3;
        r.RSQR2 = r2;
        r.RSQR1 = r1;
        selected_ = order[0];
        return true;
    }
    static uint8_t sequence_length() { return static_cast<uint8_t>(((regs().RSQR1 >> 20) & 0xFu) + 1u); }

    // ---- util/analog_sampler.hpp's converter surface --------------------------

    template <class P>
    static constexpr uint8_t input_code(AnalogIn<P>) { return AnalogIn<P>::channel; }
    static constexpr uint8_t input_code(AdcInput in) { return static_cast<uint8_t>(in); }
    static constexpr uint8_t input_code(uint8_t ch) { return ch; }

    /// A one-conversion rule sequence on this channel.
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
    /// The first channel of the rule sequence in force - what a result
    /// belongs to when the sequence is one long.
    static uint8_t selected() { return selected_; }

    /// SWSTART: the rule group, now. Void for the sampler.
    static void start() { regs().CTLR2 |= adc_swstart; }
    static bool converting() { return (regs().STATR & adc_strt) != 0u; }
    static bool ready() { return (regs().STATR & adc_eoc) != 0u; }
    /// The result, which clears EOC by the read (the F1 lineage).
    static uint16_t result() { return static_cast<uint16_t>(regs().RDATAR & 0xFFFFu); }
    /// The part's bits whatever the alignment.
    static uint16_t result_counts() {
        const uint16_t v = result();
        return cfg_.left_aligned ? static_cast<uint16_t>(v >> (16u - adc_bits))
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
    /// Convert `count` times and keep the last.
    static uint16_t read_settled(uint8_t count, uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        for (uint8_t i = 0; i < count; ++i) {
            (void)read(v, spins);
        }
        return v;
    }

    /// The supply, from a reading of VREFINT: 1.2 V nominal is `counts`
    /// of the full scale at VDD.
    static uint16_t supply_mv(uint16_t vrefint_counts) {
        if (vrefint_counts == 0u) {
            return 0;
        }
        return static_cast<uint16_t>((static_cast<uint32_t>(adc_vrefint_mv) * adc_steps + vrefint_counts / 2u) /
                                     vrefint_counts);
    }
    /// util/analog.hpp's adc_mv on a reading, against a supply in mV.
    static uint16_t millivolts(uint16_t counts, uint16_t vdd_mv) { return adc_mv(counts, adc_steps, vdd_mv); }

    // ---- the triggers ---------------------------------------------------------

    /// The rule group's start source: a timer event, the pad, or
    /// software (the default). EXTTRIG stays set: software is a trigger
    /// too (table 9-3's 111). The OPA in the pad's place is the
    /// CH32V006's TGREGU: refused on the CH32V003, whose CTLR2 has no
    /// such bit.
    static bool trigger(AdcTrigger t, bool opa_instead_of_pad = false) {
        if (opa_instead_of_pad && !device::adc_has_ctlr3) {
            return false;
        }
        AdcRegs& r = regs();
        uint32_t v = r.CTLR2 & ~(adc_extsel_mask | adc_tgregu);
        v |= static_cast<uint32_t>(t) << 17;
        if (opa_instead_of_pad) { v |= adc_tgregu; }
        r.CTLR2 = v;
        return true;
    }
    static AdcTrigger trigger() { return static_cast<AdcTrigger>((regs().CTLR2 & adc_extsel_mask) >> 17); }
    /// The injection group's: refused for a TIM3 event on the part
    /// without a TIM3, and for the OPA's path (TGINJE) on the CH32V003.
    static bool injected_trigger(AdcInjectedTrigger t, bool opa_instead_of_pad = false) {
        if (!adc_injected_trigger_valid(t) || (opa_instead_of_pad && !device::adc_has_ctlr3)) {
            return false;
        }
        AdcRegs& r = regs();
        uint32_t v = r.CTLR2 & ~(adc_jextsel_mask | adc_tginje);
        v |= static_cast<uint32_t>(t) << 12;
        if (opa_instead_of_pad) { v |= adc_tginje; }
        r.CTLR2 = v;
        return true;
    }
    /// JAUTO forbids the injection group's external trigger (9.2.4):
    /// this drops JEXTTRIG for it and restores it otherwise.
    static void injected_trigger_enable(bool on) {
        AdcRegs& r = regs();
        r.CTLR2 = on ? (r.CTLR2 | adc_jexttrig) : (r.CTLR2 & ~adc_jexttrig);
    }
    /// DLYR (the CH32V003's 9.3.15): an EXTERNAL trigger - the pad's,
    /// or a timer's - delayed by `adcclk_cycles` (0..511) before the
    /// group starts, for the rule group or the injection one. False
    /// beyond nine bits, and on the CH32V006, which has no such
    /// register (its 0x50 is CTLR3).
    static bool trigger_delay(bool injected_group, uint16_t adcclk_cycles) {
        if constexpr (device::adc_has_trigger_delay) {
            if (adcclk_cycles > adc_dlyvlu_mask) {
                return false;
            }
            regs().CTLR3_DLYR = (injected_group ? adc_dlysrc : 0u) | adcclk_cycles;
            return true;
        } else {
            (void)injected_group;
            (void)adcclk_cycles;
            return false;
        }
    }
    static void continuous(bool on) {
        AdcRegs& r = regs();
        r.CTLR2 = on ? (r.CTLR2 | adc_cont) : (r.CTLR2 & ~adc_cont);
    }
    static void dma(bool on) {
        AdcRegs& r = regs();
        r.CTLR2 = on ? (r.CTLR2 | adc_dma) : (r.CTLR2 & ~adc_dma);
    }

    // ---- the injection group -------------------------------------------------

    /// The injection sequence, 1..4 channels (9.3.11: the slots fill
    /// from the END - a length of 2 uses JSQ3 and JSQ4; the results and
    /// the offsets count from the START, in conversion order). MORE THAN
    /// ONE CONVERSION NEEDS SCAN (AdcConfig::scan): without it the first
    /// channel alone converts.
    static bool injected_sequence(const uint8_t* order, uint8_t count) {
        if (count == 0u || count > 4u) {
            return false;
        }
        uint32_t v = static_cast<uint32_t>(count - 1u) << 20;
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] >= channels) {
                return false;
            }
            const uint8_t slot = static_cast<uint8_t>(4u - count + i);   // 0-based JSQ index
            v |= static_cast<uint32_t>(order[i]) << (5u * slot);
        }
        regs().ISQR = v;
        return true;
    }
    static bool injected_offset(uint8_t slot, uint16_t offset) {
        if (slot >= 4u || offset > adc_max_count) {
            return false;
        }
        regs().IOFR[slot] = offset;
        return true;
    }
    static void injected_start() { regs().CTLR2 |= adc_jswstart; }
    static bool injected_ready() { return (regs().STATR & adc_jeoc) != 0u; }
    /// The signed result of the injection group's conversion `slot`
    /// (0..3 IN CONVERSION ORDER: the first conversion lands in IDATAR1
    /// whatever JSQ it sat in, and IOFR1 is its offset): the conversion
    /// less the offset, sign-extended from SIGNB.
    static int16_t injected_result(uint8_t slot) {
        if (slot >= 4u) {
            return 0;
        }
        const uint32_t raw = regs().IDATAR[slot];
        if (cfg_.left_aligned) {
            // SIGNB in bit 15, the datum below it down to bit 15 - adc_bits.
            return static_cast<int16_t>(static_cast<int16_t>(raw & 0xFFFFu) >> (15u - adc_bits));
        }
        return static_cast<int16_t>(raw & 0xFFFFu);   // SIGNB replicated down to bit adc_bits
    }

    // ---- the watchdogs --------------------------------------------------------

    /// Watchdog 0, the F1's: the thresholds, the scope, the interrupt,
    /// the reset (CH32V006).
    static bool watchdog0(const AdcWatchdogConfig& w) {
        if (!adc_watchdog_config_valid(w)) {
            return false;
        }
        AdcRegs& r = regs();
        r.WDHTR = w.high;
        r.WDLTR = w.low;
        uint32_t c1 = r.CTLR1 & ~(adc_awdch_mask | adc_awdsgl | adc_awden | adc_jawden | adc_awdie);
        if (w.channel) { c1 |= adc_awdsgl | *w.channel; }
        if (w.rule_group) { c1 |= adc_awden; }
        if (w.injected_group) { c1 |= adc_jawden; }
        if (w.interrupt) { c1 |= adc_awdie; }
        r.CTLR1 = c1;
        if constexpr (device::adc_has_ctlr3) {
            r.CTLR3_DLYR = w.reset_on_fault ? (r.CTLR3_DLYR | adc_awd0_rst_en) : (r.CTLR3_DLYR & ~adc_awd0_rst_en);
        }
        return true;
    }
    static void watchdog0_off() {
        AdcRegs& r = regs();
        r.CTLR1 &= ~(adc_awden | adc_jawden | adc_awdie);
        if constexpr (device::adc_has_ctlr3) {
            r.CTLR3_DLYR &= ~adc_awd0_rst_en;
        }
    }
    /// Watchdogs 1 and 2 (WDTR1/WDTR2, the CH32V006's): thresholds and
    /// the reset enable. THEY GUARD RANKS, under the watchdog scan
    /// (measured, the vendor's own example's arrangement): with
    /// AWD_SCAN set and a SCANNED rule sequence, watchdog 0 compares
    /// the FIRST conversion against WDHTR/WDLTR, watchdog 1 the SECOND
    /// against WDTR1, watchdog 2 the THIRD against WDTR2 - 9.3.15's
    /// "only applicable to watchdog channel 1" is that rank. Without
    /// the scan they compare nothing. The results are CTLR3's AWDx_RES,
    /// write-zero-to-clear; the AWD flag of STATR is watchdog 0's alone
    /// and stays down under the scan. On the CH32V003 there is no such
    /// watchdog: false, nothing written.
    static bool watchdog(uint8_t n, uint16_t low, uint16_t high, bool reset_on_fault = false) {
        if constexpr (device::adc_has_ctlr3) {
            if (n < 1u || n > 2u || low > adc_max_count || high > adc_max_count || low > high) {
                return false;
            }
            AdcRegs& r = regs();
            const uint32_t v = (static_cast<uint32_t>(high) << 16) | low;
            if (n == 1u) { r.WDTR1 = v; } else { r.WDTR2 = v; }
            const uint32_t rst = n == 1u ? adc_awd1_rst_en : adc_awd2_rst_en;
            r.CTLR3_DLYR = reset_on_fault ? (r.CTLR3_DLYR | rst) : (r.CTLR3_DLYR & ~rst);
            return true;
        } else {
            (void)n; (void)low; (void)high; (void)reset_on_fault;
            return false;
        }
    }
    /// The CH32V006's AWDx_RES; false on the CH32V003, whose watchdog 0
    /// answers in STATR's AWD alone.
    static bool watchdog_result(uint8_t n) {
        if constexpr (device::adc_has_ctlr3) {
            const uint32_t bit = n == 0u ? adc_awd0_res : n == 1u ? adc_awd1_res : adc_awd2_res;
            return (regs().CTLR3_DLYR & bit) != 0u;
        } else {
            (void)n;
            return false;
        }
    }
    static void clear_watchdog_result(uint8_t n) {
        if constexpr (device::adc_has_ctlr3) {
            const uint32_t bit = n == 0u ? adc_awd0_res : n == 1u ? adc_awd1_res : adc_awd2_res;
            regs().CTLR3_DLYR = regs().CTLR3_DLYR & ~bit;   // RW0 beside RW bits: the store keeps the rest
        } else {
            (void)n;
        }
    }
    /// AWD_SCAN (the CH32V006's 9.3.14): one watchdog per rank of the
    /// scanned rule sequence (the note on watchdog()). Nothing on the
    /// CH32V003.
    static void watchdog_scan(bool on) {
        if constexpr (device::adc_has_ctlr3) {
            AdcRegs& r = regs();
            r.CTLR3_DLYR = on ? (r.CTLR3_DLYR | adc_awd_scan) : (r.CTLR3_DLYR & ~adc_awd_scan);
        } else {
            (void)on;
        }
    }

    // ---- flags and interrupts ---------------------------------------------------

    static uint32_t flags() { return regs().STATR & AdcFlag::all; }
    static bool flag(uint32_t mask) { return (regs().STATR & mask) != 0u; }
    /// Write-zero-to-clear.
    static void clear_flags(uint32_t mask) { regs().STATR = ~mask; }

    static constexpr uint32_t converted_interrupt = adc_eocie;
    static constexpr uint32_t injected_interrupt = adc_jeocie;
    static constexpr uint32_t watchdog_interrupt = adc_awdie;
    static void interrupts(uint32_t mask, bool on) {
        AdcRegs& r = regs();
        r.CTLR1 = on ? (r.CTLR1 | mask) : (r.CTLR1 & ~mask);
    }

    /// The ISR body: the flags that are both raised and enabled, cleared,
    /// handed back. EOC is also cleared by reading RDATAR, which a
    /// handler that takes the result does anyway.
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
    template <typename Clock>
    static constexpr uint32_t clock_hz_of(Clock clock) {
        static_assert(Clock::is_static,
                      "brio Adc: the converter's clock and every sample time it holds are in HCLK "
                      "cycles - a DynamicClock would move them all; a rescaling program keeps the "
                      "ADC on a static clock");
        return brio::clock_hz(clock);   // the free function, not a member of this name
    }

    /// tSTAB: the datasheet gives no number this file trusts; a few
    /// microseconds of HCLK are spent.
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

static_assert(adc_prescaler_divider(0x00) == 2u && adc_prescaler_divider(0x08) == 4u &&
              adc_prescaler_divider(0x10) == 6u && adc_prescaler_divider(0x18) == 8u);
static_assert(adc_prescaler_divider(0x04) == 4u && adc_prescaler_divider(0x0C) == 8u &&
              adc_prescaler_divider(0x14) == 12u && adc_prescaler_divider(0x1C) == 16u);
static_assert(adc_prescaler_divider(0x05) == 8u && adc_prescaler_divider(0x15) == 24u &&
              adc_prescaler_divider(0x1D) == 32u && adc_prescaler_divider(0x16) == 48u &&
              adc_prescaler_divider(0x1F) == 128u);
static_assert(adc_prescaler_for(48'000'000UL, 24'000'000UL)->divider == 2u);
static_assert(adc_prescaler_for(48'000'000UL, 10'000'000UL)->divider == 6u);
static_assert(adc_prescaler_for(48'000'000UL, 1'000'000UL)->divider == 48u);
static_assert(!adc_prescaler_for(48'000'000UL, 100'000UL).has_value());
static_assert(adc_channel_of(Pad{'D', 4}) == 7u && adc_channel_of(Pad{'A', 2}) == 0u &&
              adc_channel_of(Pad{'C', 0}) == 0xFFu);
static_assert(!adc_config_valid(AdcConfig{.discontinuous = 9}));
static_assert(!adc_config_valid(AdcConfig{.auto_injected = true, .discontinuous = 2}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.low = 100, .high = 50}));
static_assert(!adc_watchdog_config_valid(AdcWatchdogConfig{.high = adc_max_count + 1u}));
static_assert(sizeof(AdcRegs) == 0x5C);

// Each part's own numbers, under the same `if constexpr` the verbs branch on.
constexpr bool adc_part_pinned() {
    if constexpr (device::part == Ch32Part::v003) {
        return adc_steps == 1024u && adc_max_count == 1023u &&
               adc_conversion_half_cycles(adc_sample_shortest) == 28u &&      // 3 + 11 cycles
               adc_conversion_half_cycles(adc_sample_longest) == 504u &&      // 241 + 11 cycles
               adc_max_clock_hz == 12'000'000UL &&
               !adc_config_valid(AdcConfig{.low_power = true}) &&
               !adc_config_valid(AdcConfig{.input_buffer = true}) &&
               adc_config_valid(AdcConfig{.cal_voltage = AdcCalVoltage::three_quarters}) &&
               !adc_watchdog_config_valid(AdcWatchdogConfig{.reset_on_fault = true}) &&
               !adc_injected_trigger_valid(AdcInjectedTrigger::tim3_cc1) &&
               Adc::vcal_channel.has_value() && Adc::opa_channel == 7u;
    } else {
        return adc_steps == 4096u && adc_max_count == 4095u &&
               adc_conversion_half_cycles(adc_sample_shortest) == 32u &&      // 16 cycles
               adc_conversion_half_cycles(adc_sample_longest) == 504u &&      // 252 cycles
               adc_conversion_half_cycles(static_cast<AdcSampleTime>(3), false) == 64u &&   // 19.5 + 12.5 in the other column
               adc_max_clock_hz == 48'000'000UL &&
               adc_config_valid(AdcConfig{.low_power = true}) &&
               !adc_config_valid(AdcConfig{.cal_voltage = AdcCalVoltage::three_quarters}) &&
               adc_watchdog_config_valid(AdcWatchdogConfig{.reset_on_fault = true}) &&
               adc_injected_trigger_valid(AdcInjectedTrigger::tim3_cc1) &&
               !Adc::vcal_channel.has_value() && Adc::opa_channel == 9u;
    }
}
static_assert(adc_part_pinned());

} // namespace brio
