/*
 * rtc.hpp
 *
 * The RP2040's real-time clock (datasheet 4.8): a calendar of seven
 * binary fields - a 12-bit year, the month, the day, the day of the
 * week, the hour, the minute, the second - counting seconds from a
 * reference the block makes by dividing clk_rtc by an integer
 * (CLKDIV_M1), a leap-year rule of "divisible by four" with a bit to
 * force it off, and ONE alarm: a match on any subset of the seven
 * fields, the interrupt of the block and the wake of the chip's
 * dormant state. In the two strata every brio target uses:
 *
 *  Rtc            the RESOURCE, a monostate (the chip has one): the
 *                 clock, the reference divider, the set and the read,
 *                 the load while running, the alarm and its enables,
 *                 the interrupt and the ISR body.
 *
 *  RtcDateTime    the seven fields in ORDINARY NUMBERS, the chapter's
 *                 encoding (the year 0..4095, Sunday 0); the calendar
 *                 arithmetic the silicon has not: the days of a month,
 *                 the weekday of a date, the century years the
 *                 silicon's leap rule gets wrong.
 *
 *  RtcAlarm       the seven fields with a match flag each: a field not
 *                 matched is a wildcard, so an alarm on the second
 *                 alone repeats every minute (4.8.3).
 *
 * THE CLOCK IS THE BLOCK'S OWN. clk_rtc is a generator of its own with
 * an aux mux and a 24.8 divider (clock.hpp's Clocks::rtc_select), and
 * the block wants it at any whole rate from 1 to 65536 Hz (4.8.4):
 * init() puts the crystal over 256 on it, 46875 Hz, and CLKDIV_M1 at
 * 46874 - the chapter's own example - or takes a one-pulse-per-second
 * source on a GPIO input with both dividers at one. The block's reset
 * completes without its clock (measured - the converter's does not),
 * and the clock comes first all the same.
 *
 * THE TWO CLOCK DOMAINS. Every register is written and read from
 * clk_sys and synchronized across: a write takes two clk_rtc periods
 * to land (4.8.4's note - 43 us at 46875 Hz, two seconds on a 1 Hz
 * reference), a value crosses back to the read path some time after
 * RTC_ACTIVE rises, and a coherent reading is RTC_0 THEN RTC_1 - the
 * first read latches the second (4.8.5.3). AND THE ENABLE TICKS: a
 * calendar enabled from a stop counts one second at once, some 130 us
 * after RTC_ACTIVE, whatever the divider's phase was (measured: the
 * value set reads one second on, the next rollover a whole second
 * after that tick; a LOAD into a running calendar ticks nothing). So
 * set() waits for that tick in the read path, loads the value again,
 * and returns once the read path shows it: the first second is then
 * a whole one and the value the one asked for.
 *
 * WHAT THE SILICON DOES NOT DO (4.8.1.1, 4.8.2): it checks no field's
 * range - an illegal value is undefined behaviour, so set() refuses
 * one -; it computes no day of the week - it increments the one given,
 * so `rtc_weekday_of` is offered for the caller to fill it -; and its
 * leap year is "divisible by four", right until 2100, where
 * CTRL.FORCE_NOTLEAPYEAR is the caller's to set (any time between 1
 * March 2096 and 28 February 2100, the note says).
 *
 * THE ALARM IS A LEVEL: MATCH_ACTIVE and the interrupt stand for the
 * whole second the calendar matches (or the whole minute, hour ...
 * of a coarser match), so a handler that leaves the match armed is
 * re-entered until the second passes - the vendor's own handler
 * disarms it at every entry. The ISR body here disarms the match and
 * says so; a repeating alarm is re-armed by the application once the
 * matching period is over (arm_alarm()), the ISR body's own
 * `rearm_after` doing that from the next second.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "rp2040/device.hpp"

#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/resets.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

/// The seven fields, the chapter's encoding (table 551): the year
/// 0..4095, the month 1..12, the day 1..31, the weekday 0 = Sunday ..
/// 6 = Saturday (ISO 8601 mod 7), the hour 0..23, the minute and the
/// second 0..59.
struct RtcDateTime {
    uint16_t year = 0;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t weekday = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    constexpr bool operator==(const RtcDateTime&) const = default;
};

/// The silicon's leap rule: divisible by four, and nothing else.
constexpr bool rtc_silicon_leap(uint16_t year) { return (year % 4u) == 0u; }
/// The Gregorian rule.
constexpr bool rtc_gregorian_leap(uint16_t year) {
    return (year % 4u) == 0u && ((year % 100u) != 0u || (year % 400u) == 0u);
}
/// A year the silicon would give a 29 February it has not: the
/// century years, 2100 the next - CTRL.FORCE_NOTLEAPYEAR's year.
constexpr bool rtc_needs_force_not_leap(uint16_t year) { return rtc_silicon_leap(year) && !rtc_gregorian_leap(year); }

/// Days in a month under the Gregorian rule.
constexpr uint8_t rtc_days_in_month(uint8_t month, uint16_t year) {
    switch (month) {
        case 1: case 3: case 5: case 7: case 8: case 10: case 12: return 31;
        case 4: case 6: case 9: case 11: return 30;
        case 2: return rtc_gregorian_leap(year) ? 29 : 28;
        default: return 0;
    }
}

/// The weekday of a Gregorian date, 0 = Sunday (Sakamoto's method).
constexpr uint8_t rtc_weekday_of(uint16_t year, uint8_t month, uint8_t day) {
    constexpr uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    uint32_t y = year;
    if (month < 3u) {
        y -= 1u;
    }
    return static_cast<uint8_t>((y + y / 4u - y / 100u + y / 400u + t[(month - 1u) % 12u] + day) % 7u);
}

/// Table 551's ranges, the day against its month.
constexpr bool rtc_datetime_valid(const RtcDateTime& d) {
    return d.year <= 4095u && d.month >= 1u && d.month <= 12u && d.day >= 1u &&
           d.day <= rtc_days_in_month(d.month, d.year) && d.weekday <= 6u && d.hour <= 23u && d.minute <= 59u &&
           d.second <= 59u;
}

/// The alarm: a field is matched when its flag is set, a wildcard
/// otherwise (4.8.3: fewer matched fields, a repeating alarm).
struct RtcAlarm {
    RtcDateTime at{};
    bool match_year = false;
    bool match_month = false;
    bool match_day = false;
    bool match_weekday = false;
    bool match_hour = false;
    bool match_minute = false;
    bool match_second = false;
};

/// An alarm repeats when not every field is matched.
constexpr bool rtc_alarm_repeats(const RtcAlarm& a) {
    return !(a.match_year && a.match_month && a.match_day && a.match_hour && a.match_minute && a.match_second);
}
/// The matched fields in range, the rest ignored.
constexpr bool rtc_alarm_valid(const RtcAlarm& a) {
    return (!a.match_year || a.at.year <= 4095u) && (!a.match_month || (a.at.month >= 1u && a.at.month <= 12u)) &&
           (!a.match_day || (a.at.day >= 1u && a.at.day <= 31u)) && (!a.match_weekday || a.at.weekday <= 6u) &&
           (!a.match_hour || a.at.hour <= 23u) && (!a.match_minute || a.at.minute <= 59u) &&
           (!a.match_second || a.at.second <= 59u);
}

/// The register pair of a date-time (SETUP_0 / SETUP_1 and their twins).
constexpr uint32_t rtc_word0_of(const RtcDateTime& d) {
    return (static_cast<uint32_t>(d.year) << RTC_SETUP_0_YEAR_LSB) | (static_cast<uint32_t>(d.month) << RTC_SETUP_0_MONTH_LSB) |
           (static_cast<uint32_t>(d.day) << RTC_SETUP_0_DAY_LSB);
}
constexpr uint32_t rtc_word1_of(const RtcDateTime& d) {
    return (static_cast<uint32_t>(d.weekday) << RTC_SETUP_1_DOTW_LSB) | (static_cast<uint32_t>(d.hour) << RTC_SETUP_1_HOUR_LSB) |
           (static_cast<uint32_t>(d.minute) << RTC_SETUP_1_MIN_LSB) | (static_cast<uint32_t>(d.second) << RTC_SETUP_1_SEC_LSB);
}
constexpr RtcDateTime rtc_datetime_of(uint32_t w0, uint32_t w1) {
    return RtcDateTime{
        .year = static_cast<uint16_t>((w0 & RTC_RTC_1_YEAR_BITS) >> RTC_RTC_1_YEAR_LSB),
        .month = static_cast<uint8_t>((w0 & RTC_RTC_1_MONTH_BITS) >> RTC_RTC_1_MONTH_LSB),
        .day = static_cast<uint8_t>((w0 & RTC_RTC_1_DAY_BITS) >> RTC_RTC_1_DAY_LSB),
        .weekday = static_cast<uint8_t>((w1 & RTC_RTC_0_DOTW_BITS) >> RTC_RTC_0_DOTW_LSB),
        .hour = static_cast<uint8_t>((w1 & RTC_RTC_0_HOUR_BITS) >> RTC_RTC_0_HOUR_LSB),
        .minute = static_cast<uint8_t>((w1 & RTC_RTC_0_MIN_BITS) >> RTC_RTC_0_MIN_LSB),
        .second = static_cast<uint8_t>((w1 & RTC_RTC_0_SEC_BITS) >> RTC_RTC_0_SEC_LSB),
    };
}

/// Where clk_rtc comes from (init()).
enum class RtcClock : uint8_t {
    crystal_div256,   ///< the crystal over 256: 46875 Hz on a 12 MHz crystal
    gpin0_1hz,        ///< one pulse per second on GPIO 20, both dividers at one
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Rtc: the one calendar, a monostate.
 *
 *   Rtc::init(clock);                          // clk_rtc = crystal / 256, CLKDIV_M1 = 46874
 *   Rtc::set({.year = 2026, .month = 9, .day = 13, .weekday = brio::rtc_weekday_of(2026, 9, 13),
 *             .hour = 12, .minute = 30, .second = 0});
 *   if (const auto now = Rtc::read()) { ... }
 *   Rtc::alarm({.at = {.second = 0}, .match_second = true});   // every minute at :00
 *   Rtc::interrupt(true);
 *   extern "C" void isr_rtc() { if (Rtc::isr()) { tick(); Rtc::rearm_after(); } }
 */
struct Rtc {
    Rtc() = delete;

    static constexpr IRQn_Type irq() { return RTC_IRQ_IRQn; }
    static constexpr uint32_t reset_bit = ResetBlock::rtc;
    static constexpr uint32_t crystal_divider = 256;

    static RTC_Type& regs() { return *RTC; }

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the block up: clk_rtc selected (the crystal over 256, or
    /// the one-pulse-per-second input), the block out of reset - only
    /// under its clock -, CLKDIV_M1 for a one-second reference. The
    /// calendar is NOT started: set() does that. `clock` is the app's
    /// Clock tag, the one truth of the crystal's rate. False when the
    /// crystal over 256 is not a whole rate under 65537 Hz or the block
    /// did not come up.
    template <typename Clock>
    static bool init(Clock clock, RtcClock src = RtcClock::crystal_div256) {
        (void)clock;
        Nvic::disable(irq());
        if (src == RtcClock::crystal_div256) {
            if ((Clock::xtal_hz % crystal_divider) != 0u || Clock::xtal_hz / crystal_divider > 65536u) {
                return false;
            }
            Clocks::rtc_select(RtcAux::xosc, crystal_divider);
            clk_hz_ = Clock::xtal_hz / crystal_divider;
        } else {
            Clocks::rtc_select(RtcAux::gpin0, 1);
            clk_hz_ = 1;
        }
        if (!Resets::cycle(reset_bit)) {
            return false;
        }
        regs().CLKDIV_M1 = (clk_hz_ - 1u) & RTC_CLKDIV_M1_BITS;
        return true;
    }
    /// clk_rtc's rate as init() left it.
    static uint32_t clock_hz() { return clk_hz_; }
    /// The reference divider as the register holds it, plus one.
    static uint32_t reference_divider() { return (regs().CLKDIV_M1 & RTC_CLKDIV_M1_BITS) + 1u; }

    static void release() {
        Nvic::disable(irq());
        interrupt(false);
        regs().CTRL = 0;
        Resets::hold(reset_bit);
        Clocks::rtc_stop();
    }

    // ---- the calendar -------------------------------------------------------

    /// CTRL.RTC_ACTIVE: counting.
    static bool running() { return (regs().CTRL & RTC_CTRL_RTC_ACTIVE_BITS) != 0u; }
    static bool enabled() { return (regs().CTRL & RTC_CTRL_RTC_ENABLE_BITS) != 0u; }

    /// Set the calendar and start it (4.8.5.2's sequence: disabled and
    /// waited inactive, the two words, LOAD, enabled and waited active)
    /// - AND WAITED FOR IN THE READ PATH: RTC_ACTIVE rises before the
    /// value has crossed back to RTC_0 / RTC_1 (measured: a read at
    /// RTC_ACTIVE still answers the old calendar), so set() returns
    /// once read() shows what was written. Refused for a value outside
    /// table 551; the silicon checks none.
    static bool set(const RtcDateTime& d, uint32_t spins = 1'000'000u) {
        if (!rtc_datetime_valid(d)) {
            return false;
        }
        regs().CTRL = regs().CTRL & RTC_CTRL_FORCE_NOTLEAPYEAR_BITS;
        while (running() && spins-- != 0u) {
        }
        if (running()) {
            return false;
        }
        regs().SETUP_0 = rtc_word0_of(d);
        regs().SETUP_1 = rtc_word1_of(d);
        regs().CTRL = (regs().CTRL & RTC_CTRL_FORCE_NOTLEAPYEAR_BITS) | RTC_CTRL_LOAD_BITS;
        regs().CTRL = (regs().CTRL & RTC_CTRL_FORCE_NOTLEAPYEAR_BITS) | RTC_CTRL_RTC_ENABLE_BITS;
        while (!running() && spins-- != 0u) {
        }
        if (!running()) {
            return false;
        }
        // The value seen in the read path (the old calendar stands there
        // until it lands), then the enable's tick seen there too and
        // undone by a second LOAD (the file header), then the value
        // again. A tick that does not show within its bound is not
        // waited for.
        if (!wait_shown(d, spins)) {
            return false;
        }
        uint32_t tick_spins = 4'000u;
        while (tick_spins-- != 0u) {
            const auto back = read();
            if (back && back->second != d.second) {
                hw_set(regs().CTRL, RTC_CTRL_LOAD_BITS);
                break;
            }
        }
        return wait_shown(d, spins);
    }
    /// A new value into a RUNNING calendar (4.8.5.2's note): the two
    /// words, then LOAD; it lands two clk_rtc periods later.
    static bool adjust(const RtcDateTime& d) {
        if (!rtc_datetime_valid(d)) {
            return false;
        }
        regs().SETUP_0 = rtc_word0_of(d);
        regs().SETUP_1 = rtc_word1_of(d);
        hw_set(regs().CTRL, RTC_CTRL_LOAD_BITS);
        return true;
    }
    /// Stop or restart the count where it stands.
    static void enable(bool on) {
        if (on) { hw_set(regs().CTRL, RTC_CTRL_RTC_ENABLE_BITS); } else { hw_clear(regs().CTRL, RTC_CTRL_RTC_ENABLE_BITS); }
    }
    /// The calendar now, RTC_0 read before RTC_1 (the first latches
    /// the second, 4.8.5.3); nullopt while it is not running.
    static std::optional<RtcDateTime> read() {
        if (!running()) {
            return {};
        }
        const uint32_t w1 = regs().RTC_0;
        const uint32_t w0 = regs().RTC_1;
        return rtc_datetime_of(w0, w1);
    }
    /// CTRL.FORCE_NOTLEAPYEAR: the silicon's leap rule switched off,
    /// for a century year (4.8.2's note on when).
    static void force_not_leap_year(bool on) {
        if (on) { hw_set(regs().CTRL, RTC_CTRL_FORCE_NOTLEAPYEAR_BITS); } else { hw_clear(regs().CTRL, RTC_CTRL_FORCE_NOTLEAPYEAR_BITS); }
    }
    static bool force_not_leap_year() { return (regs().CTRL & RTC_CTRL_FORCE_NOTLEAPYEAR_BITS) != 0u; }

    // ---- the alarm ------------------------------------------------------------

    /// The alarm's fields and enables, written with the match disarmed
    /// (IRQ_SETUP_0's rule: nothing else changes under MATCH_ENA), then
    /// armed. Refused for a matched field out of range.
    static bool alarm(const RtcAlarm& a) {
        if (!rtc_alarm_valid(a)) {
            return false;
        }
        disarm_alarm();
        regs().IRQ_SETUP_0 = rtc_word0_of(a.at) | (a.match_year ? RTC_IRQ_SETUP_0_YEAR_ENA_BITS : 0u) |
                             (a.match_month ? RTC_IRQ_SETUP_0_MONTH_ENA_BITS : 0u) |
                             (a.match_day ? RTC_IRQ_SETUP_0_DAY_ENA_BITS : 0u);
        regs().IRQ_SETUP_1 = rtc_word1_of(a.at) | (a.match_weekday ? RTC_IRQ_SETUP_1_DOTW_ENA_BITS : 0u) |
                             (a.match_hour ? RTC_IRQ_SETUP_1_HOUR_ENA_BITS : 0u) |
                             (a.match_minute ? RTC_IRQ_SETUP_1_MIN_ENA_BITS : 0u) |
                             (a.match_second ? RTC_IRQ_SETUP_1_SEC_ENA_BITS : 0u);
        arm_alarm();
        return true;
    }
    /// MATCH_ENA up: the alarm compares from here on.
    static void arm_alarm() { hw_set(regs().IRQ_SETUP_0, RTC_IRQ_SETUP_0_MATCH_ENA_BITS); }
    static void disarm_alarm() { hw_clear(regs().IRQ_SETUP_0, RTC_IRQ_SETUP_0_MATCH_ENA_BITS); }
    static bool alarm_armed() { return (regs().IRQ_SETUP_0 & RTC_IRQ_SETUP_0_MATCH_ENA_BITS) != 0u; }
    /// MATCH_ACTIVE: the calendar matches the alarm right now.
    static bool alarm_matching() { return (regs().IRQ_SETUP_0 & RTC_IRQ_SETUP_0_MATCH_ACTIVE_BITS) != 0u; }
    static RtcAlarm alarm() {
        const uint32_t w0 = regs().IRQ_SETUP_0;
        const uint32_t w1 = regs().IRQ_SETUP_1;
        return RtcAlarm{
            .at = rtc_datetime_of(w0, w1),
            .match_year = (w0 & RTC_IRQ_SETUP_0_YEAR_ENA_BITS) != 0u,
            .match_month = (w0 & RTC_IRQ_SETUP_0_MONTH_ENA_BITS) != 0u,
            .match_day = (w0 & RTC_IRQ_SETUP_0_DAY_ENA_BITS) != 0u,
            .match_weekday = (w1 & RTC_IRQ_SETUP_1_DOTW_ENA_BITS) != 0u,
            .match_hour = (w1 & RTC_IRQ_SETUP_1_HOUR_ENA_BITS) != 0u,
            .match_minute = (w1 & RTC_IRQ_SETUP_1_MIN_ENA_BITS) != 0u,
            .match_second = (w1 & RTC_IRQ_SETUP_1_SEC_ENA_BITS) != 0u,
        };
    }

    // ---- the interrupt ----------------------------------------------------------

    static void interrupt(bool on) {
        if (on) { hw_set(regs().INTE, RTC_INTE_RTC_BITS); } else { hw_clear(regs().INTE, RTC_INTE_RTC_BITS); }
    }
    static bool raw_pending() { return (regs().INTR & RTC_INTR_RTC_BITS) != 0u; }
    static bool pending() { return (regs().INTS & RTC_INTS_RTC_BITS) != 0u; }
    static void force(bool on) {
        if (on) { hw_set(regs().INTF, RTC_INTF_RTC_BITS); } else { hw_clear(regs().INTF, RTC_INTF_RTC_BITS); }
    }
    /// The ISR body: true when the alarm raised the line, WITH THE LINE
    /// MASKED AND THE MATCH DISARMED - the interrupt is the match's
    /// level and stands for the whole matching period otherwise (the
    /// file header), and the disarm itself lands two clk_rtc periods
    /// later, during which the line would re-enter the handler
    /// (measured: 124 entries in 43 us), so INTE is cleared first, in
    /// this domain. A repeating alarm is re-armed by the application
    /// when the period is over: rearm_after(), which unmasks too.
    [[gnu::always_inline]] static bool isr() {
        if (!pending()) {
            return false;
        }
        interrupt(false);
        disarm_alarm();
        return true;
    }
    /// Re-arm the match once the calendar has left the matching value
    /// and unmask the line: polled here, bounded by `spins` (a
    /// second-matched alarm leaves within a second; a coarser match
    /// takes its period - which is the application's to know and to
    /// wait for elsewhere). True when re-armed with the match no
    /// longer active; false with the match left disarmed and the line
    /// masked.
    static bool rearm_after(uint32_t spins = 100'000u) {
        arm_alarm();
        while (alarm_matching() && spins-- != 0u) {
            disarm_alarm();
            arm_alarm();
        }
        if (alarm_matching()) {
            disarm_alarm();
            return false;
        }
        interrupt(true);
        return true;
    }

private:
    static bool wait_shown(const RtcDateTime& d, uint32_t& spins) {
        while (spins-- != 0u) {
            const auto back = read();
            if (back && back->year == d.year && back->month == d.month && back->day == d.day && back->hour == d.hour &&
                back->minute == d.minute && back->second == d.second) {
                return true;
            }
        }
        return false;
    }

    static inline uint32_t clk_hz_ = 0;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

static_assert(rtc_silicon_leap(2100) && !rtc_gregorian_leap(2100) && rtc_needs_force_not_leap(2100));
static_assert(rtc_gregorian_leap(2000) && rtc_gregorian_leap(2024) && !rtc_gregorian_leap(2023) && !rtc_needs_force_not_leap(2024));
static_assert(rtc_days_in_month(2, 2024) == 29u && rtc_days_in_month(2, 2100) == 28u && rtc_days_in_month(4, 2026) == 30u);
// 13 September 2026 is a Sunday; 1 January 2000 a Saturday; 29 February 2024 a Thursday.
static_assert(rtc_weekday_of(2026, 9, 13) == 0u && rtc_weekday_of(2000, 1, 1) == 6u && rtc_weekday_of(2024, 2, 29) == 4u);
static_assert(rtc_datetime_valid({.year = 2026, .month = 9, .day = 13, .weekday = 0, .hour = 23, .minute = 59, .second = 59}));
static_assert(!rtc_datetime_valid({.year = 4096}) && !rtc_datetime_valid({.month = 13}) && !rtc_datetime_valid({.month = 2, .day = 30}) &&
              !rtc_datetime_valid({.weekday = 7}) && !rtc_datetime_valid({.hour = 24}));
static_assert(rtc_alarm_repeats({.match_second = true}) &&
              !rtc_alarm_repeats({.match_year = true, .match_month = true, .match_day = true, .match_hour = true, .match_minute = true,
                                  .match_second = true}));
static_assert(rtc_alarm_valid({.at = {.month = 13}}) && !rtc_alarm_valid({.at = {.month = 13}, .match_month = true}));
static_assert(rtc_datetime_of(rtc_word0_of({.year = 2026, .month = 9, .day = 13}), rtc_word1_of({.weekday = 6, .hour = 23, .minute = 59, .second = 58})) ==
              RtcDateTime{.year = 2026, .month = 9, .day = 13, .weekday = 6, .hour = 23, .minute = 59, .second = 58});

} // namespace brio
