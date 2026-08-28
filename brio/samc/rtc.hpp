/*
 * rtc.hpp
 *
 * The SAM C21 Real-Time Counter (DS60001479M ch. 24): one 32-bit counter
 * behind a 10-bit prescaler, wearing three different faces.
 *
 *   Rtc              the RESOURCE and the whole chapter: the three modes
 *                    with their overlaid register views, the prescaler,
 *                    the compares and the calendar alarm, the read
 *                    synchronization, the frequency correction, both
 *                    event directions, the one interrupt vector and the
 *                    enable/reset synchronization.
 *
 *   RtcClockValue    mode 2's date and time as a struct rather than a
 *                    packed word, with the chapter's own leap-year rule.
 *
 * There are no TASKS here on purpose. An alarm clock, a slow periodic
 * source, a power-pass timebase: each of those is a policy, and the AVR
 * precedent (`avrdx/rtc.hpp`, which built its resources and left its
 * tasks unwritten) is the one being followed. They get born with their
 * first user.
 *
 * SEVEN FACTS THAT SHAPE THE FILE.
 *
 * 1. THE CLOCK IS NOT THIS DRIVER'S TO CHOOSE, and that is not an
 *    omission. The RTC has no generic clock channel; its source is
 *    picked by OSC32KCTRL.RTCCTRL.RTCSEL, one register in another
 *    chapter, and `samc/osc32kctrl.hpp` owns it (`Osc32kctrl::
 *    rtc_clock()`). Nothing here writes it. What this header does is
 *    state the obligation 21.6.7 makes - "it is highly recommended to
 *    disable the RTC module first, before the RTC clock source
 *    selection is changed" - and take the chosen RATE as an argument
 *    wherever arithmetic needs one. The order a caller keeps is:
 *    select the clock, then `init()`.
 *
 *    The corollary is sharper than it looks: SWRST and ENABLE are
 *    synchronized INTO that clock's domain, so resetting the RTC with
 *    no source running leaves SYNCBUSY standing forever. The reset
 *    default (RTCSEL = ULP1K) is always available because OSCULP32K
 *    never stops - which is why this is a caveat and not a hazard, and
 *    why every wait here is bounded and reports.
 *
 * 2. THE REGISTER MAP IS THREE OVERLAID VIEWS, exactly as the TC's is,
 *    and it is handled the same way (samc/tc.hpp, fact 4). CTRLA,
 *    EVCTRL, INTENSET/CLR, INTFLAG, DBGCTRL, SYNCBUSY and FREQCORR sit
 *    at the same offsets in all three; only what follows differs -
 *    COUNT32 + COMP0, COUNT16 + PER + COMP0/1, or CLOCK + ALARM +
 *    MASK. So the control surface is written once against the MODE0
 *    view and the width-carrying verbs come in explicit flavours
 *    (`count32`/`count16`/`clock_value`, `comp32`/`comp16`). Nothing
 *    dispatches at run time on a mode the caller chose at compile time.
 *
 * 3. ERRATUM 1.16.3 IS LIVE ON EVERY REVISION OF THIS FAMILY, THIS ONE
 *    INCLUDED: "an 8-bit or 16-bit write access for a 32-bit register,
 *    or an 8-bit write access for a 16-bit register can fail" - for
 *    COUNT in either counter mode and for CLOCK. The workaround is to
 *    write each register at its full width, and the workaround here is
 *    STRUCTURAL: there is no verb that writes a byte or a half of any
 *    of them. Every write goes through the device header's own
 *    `uint32_t` or `uint16_t` member, and no caller is offered a way to
 *    get it wrong.
 *
 * 4. READING THE COUNTER IS NOT FREE, AND THE BIT THAT MAKES IT
 *    POSSIBLE IS NOT ON AT RESET. COUNT (and CLOCK) are
 *    READ-synchronized, gated by CTRLA.COUNTSYNC / CTRLA.CLOCKSYNC:
 *    with the bit clear, 24.8.1 says plainly that "disabling the
 *    synchronization will prevent reading valid values". So
 *    `RtcConfig::read_sync` defaults TRUE, and `count32()` and friends
 *    wait SYNCBUSY.COUNT before loading. The raw accessors exist,
 *    spelled `*_raw()`, and say what they are. What the synchronization
 *    COSTS, and what a read without it actually shows, is measured -
 *    see docs/samc/rtc.md.
 *
 *    Two errata sit on this bit and NEITHER applies to this silicon,
 *    both being read-the-row traps: 1.16.1 ("COUNTSYNC/CLOCKSYNC has no
 *    effect, read synchronization is always enabled") is E/G/J revision
 *    B ONLY, and 1.16.2 (the first COUNT read after enabling COUNTSYNC
 *    is wrong) is E/G/J revisions B..E - the marks under F and H on
 *    that item belong to the N-FAMILY ROW. The bench suite looks for
 *    1.16.2's symptom anyway, because "not this silicon" is worth
 *    confirming with behaviour and not only with a matrix.
 *
 * 5. ENABLE-PROTECTION AND WRITE-SYNCHRONIZATION ARE DIFFERENT THINGS,
 *    and CTRLA carries both. MODE, PRESCALER, MATCHCLR and CLKREP are
 *    ENABLE-protected and NOT synchronized; ENABLE and SWRST are
 *    synchronized and not protected; COUNTSYNC/CLOCKSYNC is
 *    synchronized and explicitly NOT enable-protected (24.8.1), which
 *    is why `read_sync(bool)` exists as a live verb beside the
 *    configuration field. EVCTRL is enable-protected as a whole.
 *    `configure()` and `event_config()` refuse while the RTC is
 *    enabled rather than writing where the write would land nowhere.
 *
 * 6. THE PRESCALER IS NOT ONLY A DIVIDER - IT IS THE PERIODIC EVENT
 *    SOURCE, AND TURNING IT OFF TURNS THOSE OFF TOO. PEREO[7:0] tap
 *    prescaler bits 2..9, giving f_source / 2^(n+3), and they are
 *    "independent of the prescaler setting used by the RTC counter,
 *    except if CTRLA.PRESCALER is zero" (24.6.8.1) - PRESCALER = OFF
 *    silently kills all eight. `event_config()` refuses that
 *    combination; the same is true of the periodic INTERRUPTS and the
 *    interrupt verbs say so.
 *
 *    THE ARITHMETIC CONSEQUENCE WORTH KNOWING BEFORE ANY CALENDAR
 *    CODE: mode 2 needs a 1 Hz counter clock, and the prescaler stops
 *    at /1024, so a 32.768 kHz source CANNOT REACH 1 Hz. The
 *    clock/calendar has to run off one of the 1.024 kHz oscillator
 *    outputs (ULP1K / OSC1K / XOSC1K) with PRESCALER = DIV1024. That
 *    is nowhere in chapter 24; `rtc_prescaler_for_hz()` is where it
 *    lives here, and it returns nothing for a source it cannot divide.
 *
 * 7. FREQCORR NEEDS A PRESCALER. "Frequency correction requires that
 *    CTRLA.PRESCALER is greater than 1" (24.6.8.2) - greater than the
 *    FIELD VALUE 1, i.e. DIV2 or slower, since both OFF and DIV1 pass
 *    the source straight through and there is no prescaler count to
 *    add or skip. `set_frequency_correction()` reads the field and
 *    refuses. The correction itself adds or skips one prescaler count
 *    once every 4096 source cycles, VALUE times over 240 such periods:
 *    VALUE / 983040, i.e. 1.017 ppm a step and +-129 ppm at the ends.
 *
 * NOT BUILT (docs/samc/rtc.md carries the list): tasks of any kind (see
 * above); SleepWalking and wake-from-sleep, which belong to the power
 * pass together with erratum 1.8.7's caveat that a DMA WRITE to
 * RTC.COUNT during standby may not land; and the RTC as the kernel
 * timebase, which stays SysTick's job (samc/ticker.hpp says why).
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvic.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// CTRLA.MODE (24.8.1). The three faces of the same counter.
enum class RtcMode : uint8_t {
    /// Mode 0: one 32-bit counter, one 32-bit compare, TOP fixed at
    /// 0xFFFFFFFF.
    count32 = RTC_MODE0_CTRLA_MODE_COUNT32_Val,
    /// Mode 1: a 16-bit counter whose TOP is the PER register, with two
    /// 16-bit compares.
    count16 = RTC_MODE0_CTRLA_MODE_COUNT16_Val,
    /// Mode 2: the clock/calendar, with one masked alarm.
    clock = RTC_MODE0_CTRLA_MODE_CLOCK_Val,
};

/// CTRLA.PRESCALER. `off` and `div1` are BOTH divide-by-one - but `off`
/// also stops the periodic events and interrupts (24.8.1), which is the
/// whole difference between them and the reason both are named.
enum class RtcPrescaler : uint8_t {
    off = RTC_MODE0_CTRLA_PRESCALER_OFF_Val,
    div1 = RTC_MODE0_CTRLA_PRESCALER_DIV1_Val,
    div2 = RTC_MODE0_CTRLA_PRESCALER_DIV2_Val,
    div4 = RTC_MODE0_CTRLA_PRESCALER_DIV4_Val,
    div8 = RTC_MODE0_CTRLA_PRESCALER_DIV8_Val,
    div16 = RTC_MODE0_CTRLA_PRESCALER_DIV16_Val,
    div32 = RTC_MODE0_CTRLA_PRESCALER_DIV32_Val,
    div64 = RTC_MODE0_CTRLA_PRESCALER_DIV64_Val,
    div128 = RTC_MODE0_CTRLA_PRESCALER_DIV128_Val,
    div256 = RTC_MODE0_CTRLA_PRESCALER_DIV256_Val,
    div512 = RTC_MODE0_CTRLA_PRESCALER_DIV512_Val,
    div1024 = RTC_MODE0_CTRLA_PRESCALER_DIV1024_Val,
};

/// The divisor a prescaler code stands for. `off` divides by one like
/// `div1` - the codes differ in what they do to the periodic events,
/// not in what they do to the counter.
constexpr uint16_t rtc_prescaler_divisor(RtcPrescaler p) {
    switch (p) {
    case RtcPrescaler::off:
    case RtcPrescaler::div1: return 1;
    case RtcPrescaler::div2: return 2;
    case RtcPrescaler::div4: return 4;
    case RtcPrescaler::div8: return 8;
    case RtcPrescaler::div16: return 16;
    case RtcPrescaler::div32: return 32;
    case RtcPrescaler::div64: return 64;
    case RtcPrescaler::div128: return 128;
    case RtcPrescaler::div256: return 256;
    case RtcPrescaler::div512: return 512;
    case RtcPrescaler::div1024: return 1024;
    }
    return 1;
}

/**
 * The prescaler that turns `source_hz` into `wanted_hz` exactly, or
 * nothing when no code does.
 *
 * THIS IS WHERE MODE 2's UNWRITTEN CONSTRAINT LIVES. The clock/calendar
 * requires a 1 Hz counter clock (24.6.2.5) and the prescaler stops at
 * /1024, so a 32768 Hz source cannot reach it and this returns nothing;
 * a 1024 Hz source reaches it with DIV1024. The RTC's clock select
 * (OSC32KCTRL.RTCCTRL, which is not this driver's) offers a 1.024 kHz
 * output on every one of the three oscillators for exactly that reason.
 *
 * `off` is never returned: it divides by one but kills the periodic
 * events, so a caller asking for a divide-by-one gets `div1`, the code
 * that only does what was asked.
 */
constexpr std::optional<RtcPrescaler> rtc_prescaler_for_hz(uint32_t source_hz,
                                                           uint32_t wanted_hz) {
    if (source_hz == 0u || wanted_hz == 0u || source_hz % wanted_hz != 0u) {
        return std::nullopt;
    }
    const uint32_t d = source_hz / wanted_hz;
    for (uint8_t code = static_cast<uint8_t>(RtcPrescaler::div1);
         code <= static_cast<uint8_t>(RtcPrescaler::div1024); ++code) {
        const RtcPrescaler p = static_cast<RtcPrescaler>(code);
        if (rtc_prescaler_divisor(p) == d) {
            return p;
        }
    }
    return std::nullopt;
}

/// The frequency of the periodic event/interrupt PERn, in millihertz so
/// that the slow ones do not round to zero. PERn taps prescaler bit
/// n + 2, so it toggles at f_source / 2^(n+3) (24.6.8.1): PER0 every
/// eight source cycles, PER7 every 1024.
constexpr uint32_t rtc_periodic_mhz(uint32_t source_hz, uint8_t n) {
    return n > 7u ? 0u
                  : static_cast<uint32_t>((static_cast<uint64_t>(source_hz) *
                                           1000ULL) >>
                                          (n + 3u));
}

/**
 * What one step of FREQCORR.VALUE is worth, in parts per BILLION.
 *
 * 24.6.8.2: the circuit adds or skips one prescaler count every 4096
 * source cycles, VALUE times over 240 of those periods, so the
 * correction is VALUE / (4096 x 240) = VALUE / 983040. In ppm the
 * chapter quotes a 1.017 ppm resolution; ppb is what makes the whole
 * range expressible in integers, and 127 steps come to 129 199 ppb.
 */
constexpr uint32_t rtc_correction_ppb(uint8_t value) {
    return static_cast<uint32_t>((static_cast<uint64_t>(value) * 1'000'000'000ULL) /
                                 983'040ULL);
}

/// MASK.SEL (24.12.11): which fields of ALARM take part in the
/// comparison. Every level includes the ones below it - there is no way
/// to match hours while ignoring seconds.
enum class RtcAlarmMask : uint8_t {
    off = RTC_MODE2_MASK_SEL_OFF_Val,
    second = RTC_MODE2_MASK_SEL_SS_Val,
    minute_second = RTC_MODE2_MASK_SEL_MMSS_Val,
    hour_minute_second = RTC_MODE2_MASK_SEL_HHMMSS_Val,
    day_and_below = RTC_MODE2_MASK_SEL_DDHHMMSS_Val,
    month_and_below = RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val,
    year_and_below = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val,
};

/// SEL 7 is Reserved (24.12.11); `off` is not - it is how an alarm is
/// disarmed without disturbing the ALARM register itself.
constexpr bool rtc_alarm_mask_valid(RtcAlarmMask m) {
    return static_cast<uint8_t>(m) <=
           static_cast<uint8_t>(RtcAlarmMask::year_and_below);
}

/// THE CHAPTER'S OWN LEAP RULE, and it is not the Gregorian one:
/// "the year is considered a leap year if YEAR[1:0] is zero"
/// (24.12.9). YEAR is an OFFSET from a reference year the software
/// picks, and 24.6.2.5 requires that reference to BE a leap year
/// (2016, 2020, ...) for this to mean anything. The rule has no century
/// exception - which is harmless over the register's 64-year span as
/// long as that span does not cross a non-leap century.
constexpr bool rtc_is_leap(uint8_t year) { return (year & 0x3u) == 0u; }

constexpr uint8_t rtc_days_in_month(uint8_t month, uint8_t year) {
    switch (month) {
    case 1: case 3: case 5: case 7: case 8: case 10: case 12: return 31;
    case 4: case 6: case 9: case 11: return 30;
    case 2: return rtc_is_leap(year) ? 29 : 28;
    default: return 0;
    }
}

/**
 * Mode 2's CLOCK / ALARM word, unpacked.
 *
 * `year` is an OFFSET (0..63) from a reference year the SOFTWARE
 * chooses and the silicon never sees - so this struct deliberately does
 * not carry a calendar year. `hour` is 0..23 in the 24-hour
 * representation; in the 12-hour one it is 1..12 and `pm` is HOUR[4].
 */
struct RtcClockValue {
    uint8_t second = 0;
    uint8_t minute = 0;
    uint8_t hour = 0;
    uint8_t day = 1;      ///< 1..31, and the month decides the top
    uint8_t month = 1;    ///< 1 = January
    uint8_t year = 0;     ///< offset from the reference year, 0..63
    bool pm = false;      ///< HOUR[4], meaningful only with CTRLA.CLKREP

    constexpr uint32_t to_register() const {
        return RTC_MODE2_CLOCK_SECOND(second) | RTC_MODE2_CLOCK_MINUTE(minute) |
               RTC_MODE2_CLOCK_HOUR(static_cast<uint32_t>(hour) |
                                    (pm ? 0x10u : 0u)) |
               RTC_MODE2_CLOCK_DAY(day) | RTC_MODE2_CLOCK_MONTH(month) |
               RTC_MODE2_CLOCK_YEAR(year);
    }

    /**
     * Unpack a CLOCK or ALARM word.
     *
     * `twelve_hour` IS NOT OPTIONAL INFORMATION, it is what the HOUR
     * field MEANS. In the 24-hour representation HOUR is five bits
     * holding 0..23 - bit 4 is part of the number, and hours 16 to 23
     * have it set. In the 12-hour one HOUR[3:0] holds 1..12 and HOUR[4]
     * is AM/PM. The same bits, two readings, and only CTRLA.CLKREP
     * decides - which is why `Rtc::clock_value()` asks the register
     * rather than the caller, and never this default. The default that
     * DOES exist is not a guess: false is CLKREP's own reset value, so
     * a bare call reads the word the way an untouched RTC would.
     */
    static constexpr RtcClockValue from_register(uint32_t r,
                                                 bool twelve_hour = false) {
        const uint8_t h = static_cast<uint8_t>(
            (r & RTC_MODE2_CLOCK_HOUR_Msk) >> RTC_MODE2_CLOCK_HOUR_Pos);
        return RtcClockValue{
            .second = static_cast<uint8_t>((r & RTC_MODE2_CLOCK_SECOND_Msk) >>
                                           RTC_MODE2_CLOCK_SECOND_Pos),
            .minute = static_cast<uint8_t>((r & RTC_MODE2_CLOCK_MINUTE_Msk) >>
                                           RTC_MODE2_CLOCK_MINUTE_Pos),
            .hour = static_cast<uint8_t>(twelve_hour ? (h & 0x0Fu) : h),
            .day = static_cast<uint8_t>((r & RTC_MODE2_CLOCK_DAY_Msk) >>
                                        RTC_MODE2_CLOCK_DAY_Pos),
            .month = static_cast<uint8_t>((r & RTC_MODE2_CLOCK_MONTH_Msk) >>
                                          RTC_MODE2_CLOCK_MONTH_Pos),
            .year = static_cast<uint8_t>((r & RTC_MODE2_CLOCK_YEAR_Msk) >>
                                         RTC_MODE2_CLOCK_YEAR_Pos),
            .pm = twelve_hour && (h & 0x10u) != 0u,
        };
    }

    /// A date the counter could actually hold. `twelve_hour` selects
    /// which hour range applies - the two are different fields' worth of
    /// legality on the same bits.
    constexpr bool valid(bool twelve_hour = false) const {
        if (second > 59u || minute > 59u || year > 63u) {
            return false;
        }
        if (twelve_hour ? (hour < 1u || hour > 12u) : (hour > 23u || pm)) {
            return false;
        }
        if (month < 1u || month > 12u) {
            return false;
        }
        return day >= 1u && day <= rtc_days_in_month(month, year);
    }

    constexpr bool operator==(const RtcClockValue&) const = default;
};

// =============================================================================
// Configuration
// =============================================================================

/**
 * CTRLA, all of it. The four enable-protected fields and the one that
 * is not (`read_sync`) travel together because they are written
 * together at initialization; `Rtc::read_sync(bool)` exists for the one
 * that can also move later.
 */
struct RtcConfig {
    RtcMode mode = RtcMode::count32;
    RtcPrescaler prescaler = RtcPrescaler::div1;

    /// CTRLA.MATCHCLR: clear the counter on a COMP0 / ALARM0 match.
    /// "Valid only in Mode 0 (COUNT32) and Mode 2 (CLOCK)" (24.12.1),
    /// and refused elsewhere. Note the chapter's own warning: with this
    /// set, the compare flag and the OVERFLOW flag are raised together.
    bool match_clear = false;

    /// CTRLA.CLKREP: 12-hour representation with AM/PM in HOUR[4].
    /// Mode 2 only.
    bool twelve_hour = false;

    /// CTRLA.COUNTSYNC in the counter modes, CLOCKSYNC in mode 2 - the
    /// same bit under two names. DEFAULTS TRUE because with it clear,
    /// 24.8.1 says reading COUNT does not give valid values at all.
    bool read_sync = true;
};

/// EVCTRL, which is enable-protected as a whole (24.8.2).
struct RtcEventConfig {
    /// PEREO[7:0], bit n = periodic interval n. NOTE that all eight go
    /// silent when the prescaler is OFF (24.6.8.1), which is what
    /// `rtc_event_config_valid()` refuses.
    uint8_t periodic_out = 0;
    /// CMPEOn in the counter modes, ALARMEO in mode 2: bit 0 is COMP0 /
    /// ALARM0, bit 1 is COMP1 and exists in mode 1 alone.
    uint8_t compare_out = 0;
    bool overflow_out = false;
};

/// How many compare (or alarm) channels a mode has. Mode 0 has one
/// 32-bit COMP0, mode 1 has two 16-bit ones, mode 2 has one ALARM0.
constexpr uint8_t rtc_compare_count(RtcMode m) {
    return m == RtcMode::count16 ? 2u : 1u;
}

/// A configuration's legality, and every refusal is a sentence of the
/// chapter: a Reserved prescaler code, MATCHCLR outside modes 0 and 2
/// (24.12.1), CLKREP outside mode 2 (24.12.1).
constexpr bool rtc_config_valid(const RtcConfig& c) {
    if (static_cast<uint8_t>(c.prescaler) >
        static_cast<uint8_t>(RtcPrescaler::div1024)) {
        return false;
    }
    if (c.match_clear && c.mode == RtcMode::count16) {
        return false;
    }
    return !(c.twelve_hour && c.mode != RtcMode::clock);
}

/**
 * The two rules that live BETWEEN the configuration and the event
 * configuration, so they get a function of their own:
 *  - a compare-event bit past the mode's channel count (there is no
 *    CMPEO1 outside mode 1);
 *  - a periodic event asked for with the prescaler OFF, which the
 *    silicon accepts and then never honours (24.6.8.1, 24.8.1).
 */
constexpr bool rtc_event_config_valid(const RtcConfig& c,
                                      const RtcEventConfig& e) {
    if (e.compare_out >= (1u << rtc_compare_count(c.mode))) {
        return false;
    }
    return !(e.periodic_out != 0u && c.prescaler == RtcPrescaler::off);
}

/// INTFLAG / INTENSET / INTENCLR. Bit 8 is CMP0 in the counter modes
/// and ALARM0 in the calendar - the same bit, so both names are here
/// and they are equal by construction.
struct RtcFlag {
    static constexpr uint16_t overflow = RTC_MODE0_INTFLAG_OVF_Msk;
    static constexpr uint16_t compare0 = RTC_MODE0_INTFLAG_CMP0_Msk;
    static constexpr uint16_t compare1 = RTC_MODE1_INTFLAG_CMP1_Msk;
    static constexpr uint16_t alarm0 = RTC_MODE2_INTFLAG_ALARM0_Msk;
    static constexpr uint16_t periodic(uint8_t n) {
        return static_cast<uint16_t>(1u << n);
    }
    static constexpr uint16_t compare(uint8_t n) {
        return static_cast<uint16_t>(compare0 << n);
    }
    static constexpr uint16_t periodic_all = 0x00FFu;
    static constexpr uint16_t all =
        static_cast<uint16_t>(periodic_all | compare0 | compare1 | overflow);
};

static_assert(RtcFlag::compare0 == RtcFlag::alarm0,
              "CMP0 and ALARM0 are the same INTFLAG bit under two names");

// =============================================================================
// The resource
// =============================================================================

/**
 * The RTC. One instance on every member of this family, so this is a
 * monostate struct and not a template - there is no index to pass.
 */
struct Rtc {
    Rtc() = delete;

    static constexpr IRQn_Type irq() { return RTC_IRQn; }

    /// The counter is 32 bits wide in mode 0 and 16 in mode 1; mode 2's
    /// top is a date, not a number (24.6.2.5).
    static constexpr uint32_t count32_max = 0xFFFFFFFFUL;
    static constexpr uint16_t count16_max = 0xFFFFu;
    /// FREQCORR.VALUE is seven bits plus a sign.
    static constexpr uint8_t correction_max = 0x7Fu;

    // ---- the EVSYS vocabulary this peripheral publishes --------------------
    //
    // evsys.hpp owns the fabric and not the vocabulary (see its header),
    // so the codes live here. They come from the device header's own
    // EVENT_ID_GEN_RTC_* constants rather than from a table this file
    // would keep - and the RTC is one instance on every variant, so
    // there is nothing per-package to probe and nothing for the
    // device-tables reserve to hold.
    //
    // THE RTC IS A GENERATOR ONLY. It consumes no events: there is no
    // EVENT_ID_USER_RTC_* symbol in the header, and 24.6.5 lists
    // outputs alone.

    /// Generator: COMP0 in the counter modes, ALARM0 in mode 2 - the
    /// header spells both `RTC_CMP_0`.
    static constexpr uint8_t compare_generator(uint8_t n) {
        return static_cast<uint8_t>(EVENT_ID_GEN_RTC_CMP_0 + n);
    }
    static constexpr uint8_t alarm_generator = EVENT_ID_GEN_RTC_CMP_0;
    static constexpr uint8_t overflow_generator = EVENT_ID_GEN_RTC_OVF;
    /// Generator: periodic interval n, n = 0..7.
    static constexpr uint8_t periodic_generator(uint8_t n) {
        return static_cast<uint8_t>(EVENT_ID_GEN_RTC_PER_0 + n);
    }

    // ---- the three overlaid views ------------------------------------------
    //
    // Everything up to and including FREQCORR is common; the control
    // surface below is written against MODE0 and the width-carrying
    // verbs name their mode.

    static rtc_mode0_registers_t& regs0() { return RTC_REGS->MODE0; }
    static rtc_mode1_registers_t& regs1() { return RTC_REGS->MODE1; }
    static rtc_mode2_registers_t& regs2() { return RTC_REGS->MODE2; }

    static constexpr bool config_valid(const RtcConfig& c) {
        return rtc_config_valid(c);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_RTC_Msk, on); }

    static uint32_t sync_flags() { return regs0().RTC_SYNCBUSY; }
    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs0().RTC_SYNCBUSY, mask, false, spins);
    }

    /**
     * CTRLA.SWRST. 24.6.2.2: "the RTC must be disabled before resetting
     * it", so this disables first rather than trusting the caller.
     *
     * BOTH WAITS ARE INTO THE RTC's OWN CLOCK DOMAIN, which is the one
     * OSC32KCTRL.RTCCTRL selects - so this returns false, rather than
     * hanging, when that clock is not running. DBGCTRL survives a reset
     * (24.8.6), which is the one register the chapter exempts.
     */
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs0().RTC_CTRLA = 0u;
        if (!sync_wait(RTC_MODE0_SYNCBUSY_ENABLE_Msk, spins)) {
            return false;
        }
        regs0().RTC_CTRLA = RTC_MODE0_CTRLA_SWRST_Msk;
        return sync_wait(RTC_MODE0_SYNCBUSY_SWRST_Msk, spins);
    }

    static bool enabled() {
        return (regs0().RTC_CTRLA & RTC_MODE0_CTRLA_ENABLE_Msk) != 0u;
    }
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint16_t v =
            static_cast<uint16_t>(regs0().RTC_CTRLA & ~RTC_MODE0_CTRLA_ENABLE_Msk);
        regs0().RTC_CTRLA =
            on ? static_cast<uint16_t>(v | RTC_MODE0_CTRLA_ENABLE_Msk) : v;
        return sync_wait(RTC_MODE0_SYNCBUSY_ENABLE_Msk, spins);
    }

    /**
     * APB clock on, interrupt off, software reset. The RTC is left
     * DISABLED and unconfigured, because CTRLA's fields and EVCTRL are
     * enable-protected and the caller has them to write.
     *
     * THE CLOCK MUST ALREADY BE CHOSEN AND RUNNING: this driver never
     * writes OSC32KCTRL.RTCCTRL (see the header essay, fact 1), and the
     * reset below synchronizes into whatever that register selected.
     * The reset default, ULP1K off the always-running OSCULP32K, is
     * what makes the naive order work anyway.
     */
    static bool init(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        bus_clock(true);
        return reset(spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        bus_clock(false);
    }

    // ---- configuration (enable-protected) ----------------------------------

    /**
     * CTRLA's whole field set. REFUSED while the RTC is enabled: MODE,
     * PRESCALER, MATCHCLR and CLKREP are enable-protected (24.6.2.1)
     * and a write that lands nowhere is worse than a false return.
     *
     * COUNTSYNC/CLOCKSYNC is the one bit here that is NOT
     * enable-protected but IS write-synchronized, which is why this
     * ends in a wait that the other four fields would not need.
     */
    static bool configure(const RtcConfig& c, uint32_t spins = 0xFFFFu) {
        if (!config_valid(c) || enabled()) {
            return false;
        }
        regs0().RTC_CTRLA = static_cast<uint16_t>(
            RTC_MODE0_CTRLA_MODE(static_cast<uint32_t>(c.mode)) |
            RTC_MODE0_CTRLA_PRESCALER(static_cast<uint32_t>(c.prescaler)) |
            (c.match_clear ? RTC_MODE0_CTRLA_MATCHCLR_Msk : 0u) |
            (c.twelve_hour ? RTC_MODE2_CTRLA_CLKREP_Msk : 0u) |
            (c.read_sync ? RTC_MODE0_CTRLA_COUNTSYNC_Msk : 0u));
        return sync_wait(RTC_MODE0_SYNCBUSY_COUNTSYNC_Msk, spins);
    }

    /// The compile-time twin, following avrdx's `Adc::init<cfg>()` and
    /// this stratum's `Vref::configure<cfg>()`: an impossible
    /// configuration is a compile error and not a false return.
    template <RtcConfig c>
    static bool configure(uint32_t spins = 0xFFFFu) {
        static_assert(rtc_config_valid(c),
                      "this RTC configuration is refused by chapter 24: a "
                      "Reserved prescaler code, MATCHCLR outside modes 0 and "
                      "2, or CLKREP outside mode 2");
        return configure(c, spins);
    }

    static uint16_t ctrla() { return regs0().RTC_CTRLA; }
    static RtcMode mode() {
        return static_cast<RtcMode>((regs0().RTC_CTRLA & RTC_MODE0_CTRLA_MODE_Msk) >>
                                    RTC_MODE0_CTRLA_MODE_Pos);
    }
    static RtcPrescaler prescaler() {
        return static_cast<RtcPrescaler>(
            (regs0().RTC_CTRLA & RTC_MODE0_CTRLA_PRESCALER_Msk) >>
            RTC_MODE0_CTRLA_PRESCALER_Pos);
    }
    static bool match_clear() {
        return (regs0().RTC_CTRLA & RTC_MODE0_CTRLA_MATCHCLR_Msk) != 0u;
    }
    static bool twelve_hour() {
        return (regs0().RTC_CTRLA & RTC_MODE2_CTRLA_CLKREP_Msk) != 0u;
    }

    /**
     * COUNTSYNC / CLOCKSYNC on its own, live: the one CTRLA bit that is
     * write-synchronized and explicitly NOT enable-protected (24.8.1),
     * so it can be moved under a running counter. That is what makes it
     * measurable, and what the bench suite uses it for.
     */
    static bool read_sync(bool on, uint32_t spins = 0xFFFFu) {
        const uint16_t v = static_cast<uint16_t>(regs0().RTC_CTRLA &
                                                 ~RTC_MODE0_CTRLA_COUNTSYNC_Msk);
        regs0().RTC_CTRLA =
            on ? static_cast<uint16_t>(v | RTC_MODE0_CTRLA_COUNTSYNC_Msk) : v;
        return sync_wait(RTC_MODE0_SYNCBUSY_COUNTSYNC_Msk, spins);
    }
    static bool read_sync() {
        return (regs0().RTC_CTRLA & RTC_MODE0_CTRLA_COUNTSYNC_Msk) != 0u;
    }

    /// EVCTRL, enable-protected like the rest. Takes the RtcConfig too,
    /// because the rules that matter live between them - see
    /// `rtc_event_config_valid()`.
    static bool event_config(const RtcConfig& c, const RtcEventConfig& e) {
        if (!rtc_event_config_valid(c, e) || enabled()) {
            return false;
        }
        // THE COMPARE FIELD IS WRITTEN THROUGH THE MODE1 MACRO ON
        // PURPOSE. `RTC_MODE0_EVCTRL_CMPEO_Msk` is ONE bit wide - mode 0
        // has one compare - so writing a mode-1 configuration through it
        // would silently drop CMPEO1. The mode 1 macro is two bits at
        // the same position, a superset of all three modes, and
        // `rtc_event_config_valid()` has already refused bit 1 wherever
        // it does not exist. The bench suite checks that both bits land.
        regs0().RTC_EVCTRL =
            RTC_MODE0_EVCTRL_PEREO(e.periodic_out) |
            RTC_MODE1_EVCTRL_CMPEO(e.compare_out) |
            (e.overflow_out ? RTC_MODE0_EVCTRL_OVFEO_Msk : 0u);
        return true;
    }
    static uint32_t evctrl() { return regs0().RTC_EVCTRL; }

    // ---- the counter, mode 0 -----------------------------------------------
    //
    // ERRATUM 1.16.3: COUNT is written at its FULL WIDTH and never in
    // pieces. There is no byte-wise verb here and there will not be one.

    static uint32_t count32_raw() { return regs0().RTC_COUNT; }
    /// The synchronized read: SYNCBUSY.COUNT out, then the load. Without
    /// CTRLA.COUNTSYNC this is not a valid counter value at all
    /// (24.8.1) - which is why `RtcConfig::read_sync` defaults true.
    static uint32_t count32(uint32_t spins = 0xFFFFu) {
        (void)sync_wait(RTC_MODE0_SYNCBUSY_COUNT_Msk, spins);
        return regs0().RTC_COUNT;
    }
    static bool set_count32(uint32_t v, uint32_t spins = 0xFFFFu) {
        regs0().RTC_COUNT = v;
        return sync_wait(RTC_MODE0_SYNCBUSY_COUNT_Msk, spins);
    }

    static uint32_t comp32() { return regs0().RTC_COMP; }
    static bool set_comp32(uint32_t v, uint32_t spins = 0xFFFFu) {
        regs0().RTC_COMP = v;
        return sync_wait(RTC_MODE0_SYNCBUSY_COMP0_Msk, spins);
    }

    // ---- the counter, mode 1 -----------------------------------------------

    static uint16_t count16_raw() { return regs1().RTC_COUNT; }
    static uint16_t count16(uint32_t spins = 0xFFFFu) {
        (void)sync_wait(RTC_MODE1_SYNCBUSY_COUNT_Msk, spins);
        return regs1().RTC_COUNT;
    }
    static bool set_count16(uint16_t v, uint32_t spins = 0xFFFFu) {
        regs1().RTC_COUNT = v;
        return sync_wait(RTC_MODE1_SYNCBUSY_COUNT_Msk, spins);
    }

    /// PER: mode 1's TOP, and the reason mode 1 exists at all - modes 0
    /// and 2 have no period register and wrap at their own maximum.
    static uint16_t period16() { return regs1().RTC_PER; }
    static bool set_period16(uint16_t v, uint32_t spins = 0xFFFFu) {
        regs1().RTC_PER = v;
        return sync_wait(RTC_MODE1_SYNCBUSY_PER_Msk, spins);
    }

    static uint16_t comp16(uint8_t n) { return regs1().RTC_COMP[n]; }
    static bool set_comp16(uint8_t n, uint16_t v, uint32_t spins = 0xFFFFu) {
        if (n >= rtc_compare_count(RtcMode::count16)) {
            return false;
        }
        regs1().RTC_COMP[n] = v;
        return sync_wait(RTC_MODE1_SYNCBUSY_COMP0_Msk << n, spins);
    }

    // ---- the calendar, mode 2 ----------------------------------------------

    static uint32_t clock_register_raw() { return regs2().RTC_CLOCK; }
    static uint32_t clock_register(uint32_t spins = 0xFFFFu) {
        (void)sync_wait(RTC_MODE2_SYNCBUSY_CLOCK_Msk, spins);
        return regs2().RTC_CLOCK;
    }
    /// The unpacked date and time. CTRLA.CLKREP is asked, not the
    /// caller: it is what decides whether HOUR's bit 4 is part of the
    /// number or the AM/PM flag.
    static RtcClockValue clock_value(uint32_t spins = 0xFFFFu) {
        return RtcClockValue::from_register(clock_register(spins), twelve_hour());
    }

    /// Set the date and time. False - and nothing written - for a date
    /// the counter could not hold; the 12/24-hour question is asked of
    /// CTRLA rather than of the caller, because the register is what
    /// decides how HOUR is read.
    static bool set_clock(const RtcClockValue& v, uint32_t spins = 0xFFFFu) {
        if (!v.valid(twelve_hour())) {
            return false;
        }
        regs2().RTC_CLOCK = v.to_register();
        return sync_wait(RTC_MODE2_SYNCBUSY_CLOCK_Msk, spins);
    }

    static uint32_t alarm_register() { return regs2().RTC_ALARM; }
    static RtcClockValue alarm() {
        return RtcClockValue::from_register(regs2().RTC_ALARM, twelve_hour());
    }
    static bool set_alarm(const RtcClockValue& v, uint32_t spins = 0xFFFFu) {
        if (!v.valid(twelve_hour())) {
            return false;
        }
        regs2().RTC_ALARM = v.to_register();
        return sync_wait(RTC_MODE2_SYNCBUSY_ALARM0_Msk, spins);
    }

    static RtcAlarmMask alarm_mask() {
        return static_cast<RtcAlarmMask>(regs2().RTC_MASK & RTC_MODE2_MASK_SEL_Msk);
    }
    /// MASK.SEL. Refuses the Reserved code 7; `off` is a legal value and
    /// is how an alarm is disarmed without disturbing ALARM itself.
    static bool set_alarm_mask(RtcAlarmMask m, uint32_t spins = 0xFFFFu) {
        if (!rtc_alarm_mask_valid(m)) {
            return false;
        }
        regs2().RTC_MASK =
            static_cast<uint8_t>(RTC_MODE2_MASK_SEL(static_cast<uint32_t>(m)));
        return sync_wait(RTC_MODE2_SYNCBUSY_MASK0_Msk, spins);
    }

    // ---- frequency correction ----------------------------------------------

    /**
     * FREQCORR: the digital trim, in units of one prescaler count per
     * 4096 source cycles applied `value` times in 240 such periods -
     * `rtc_correction_ppb()` turns that into parts per billion.
     *
     * `negative` is FREQCORR.SIGN, and the polarity is worth restating
     * because the register's own wording inverts twice: SIGN = 0 means
     * "the correction value is POSITIVE, i.e. frequency will be
     * DECREASED" (24.8.8) - counts are ADDED to the prescaler, so a
     * period gets longer. `negative = true` speeds the counter up.
     *
     * FALSE - AND NOTHING WRITTEN - WITH THE PRESCALER AT OFF OR DIV1.
     * 24.6.8.2 requires CTRLA.PRESCALER greater than 1, and both of
     * those codes pass the source straight through: there is no
     * prescaler count to add or skip, so the correction would be
     * configured and then do nothing at all.
     */
    static bool set_frequency_correction(bool negative, uint8_t value,
                                         uint32_t spins = 0xFFFFu) {
        if (value > correction_max) {
            return false;
        }
        if (static_cast<uint8_t>(prescaler()) <=
            static_cast<uint8_t>(RtcPrescaler::div1)) {
            return false;
        }
        regs0().RTC_FREQCORR = static_cast<uint8_t>(
            RTC_FREQCORR_VALUE(value) | (negative ? RTC_FREQCORR_SIGN_Msk
                                                        : 0u));
        return sync_wait(RTC_MODE0_SYNCBUSY_FREQCORR_Msk, spins);
    }

    static uint8_t correction_value() {
        return static_cast<uint8_t>(regs0().RTC_FREQCORR &
                                    RTC_FREQCORR_VALUE_Msk);
    }
    static bool correction_negative() {
        return (regs0().RTC_FREQCORR & RTC_FREQCORR_SIGN_Msk) != 0u;
    }

    // ---- interrupts --------------------------------------------------------
    //
    // One vector for every source (24.6.4), so a handler asks INTFLAG
    // which one it was. NOTE that the eight PERIODIC interrupts go
    // silent with the prescaler OFF, exactly as the periodic events do.

    static uint16_t flags() { return regs0().RTC_INTFLAG; }
    static void clear_flags(uint16_t mask = RtcFlag::all) {
        regs0().RTC_INTFLAG = mask;
    }
    static uint16_t armed() { return regs0().RTC_INTENSET; }
    static void arm(uint16_t mask) { regs0().RTC_INTENSET = mask; }
    static void disarm(uint16_t mask = RtcFlag::all) {
        regs0().RTC_INTENCLR = mask;
    }

    /// The ISR body; the app binds RTC_Handler. Masked with INTENSET
    /// like every other one-vector-many-sources block in this stratum,
    /// and it returns what it acknowledged.
    [[gnu::always_inline]] static uint16_t isr() {
        const uint16_t p = static_cast<uint16_t>(flags() & armed());
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    /// DBGCTRL.DBGRUN: keep counting while an external debugger holds
    /// the CPU. This register is the ONE the software reset does not
    /// touch (24.8.6).
    static void debug_run(bool on) {
        regs0().RTC_DBGCTRL =
            static_cast<uint8_t>(on ? RTC_DBGCTRL_DBGRUN_Msk : 0u);
    }
    static bool debug_run() {
        return (regs0().RTC_DBGCTRL & RTC_DBGCTRL_DBGRUN_Msk) != 0u;
    }
};

} // namespace brio
