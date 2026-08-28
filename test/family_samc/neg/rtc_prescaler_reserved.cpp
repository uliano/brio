// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.PRESCALER codes 0xC..0xF are Reserved (24.8.1). The enum names
// the eleven that exist; a cast past them must not reach the register.

#include "samc/rtc.hpp"

using namespace brio;

constexpr RtcConfig bad{.prescaler = static_cast<RtcPrescaler>(0xC)};
static_assert(rtc_config_valid(bad),
              "this assertion is meant to FAIL: 0xC is a Reserved prescaler "
              "code");
