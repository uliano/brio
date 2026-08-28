// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.CLKREP "is valid only in Mode 2" (24.12.1) - there are no hours
// in a plain counter, so a 12-hour representation of one is nonsense.

#include "samc/rtc.hpp"

using namespace brio;

constexpr RtcConfig bad{.mode = RtcMode::count32, .twelve_hour = true};
static_assert(rtc_config_valid(bad),
              "this assertion is meant to FAIL: CLKREP belongs to mode 2");
