// mcu: samc21e18a samc21g18a samc21j18a
// MASK.SEL 0x7 is Reserved (24.12.11). The six that exist are a LADDER -
// each level includes the ones below it - and there is no seventh rung.

#include "samc/rtc.hpp"

using namespace brio;

static_assert(rtc_alarm_mask_valid(static_cast<RtcAlarmMask>(7)),
              "this assertion is meant to FAIL: SEL 7 is Reserved");
