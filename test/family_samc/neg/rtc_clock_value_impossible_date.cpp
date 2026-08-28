// mcu: samc21e18a samc21g18a samc21j18a
// The calendar counts real days: February has 28 or 29 of them, decided
// by the chapter's own leap rule (YEAR[1:0] == 0, 24.12.9) and not by
// the Gregorian one. A date the counter could never reach must not be
// writable into CLOCK or ALARM.

#include "samc/rtc.hpp"

using namespace brio;

constexpr RtcClockValue bad{.day = 30, .month = 2, .year = 0};
static_assert(bad.valid(),
              "this assertion is meant to FAIL: there is no 30 February, leap "
              "year or not");
