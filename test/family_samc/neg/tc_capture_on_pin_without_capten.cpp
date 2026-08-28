// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.COPENx routes a capture channel's input to its WO pad, but the
// channel only captures at all if CTRLA.CAPTENx is set (35.6.2.8). A
// configuration with COPEN and no CAPTEN captures nothing and must not
// compile.

#include "samc/tc.hpp"

using namespace brio;

constexpr TcConfig bad_cfg{.capture_on_pin = 0x1};
static_assert(tc_config_valid(0, bad_cfg),
              "this assertion is meant to FAIL: capture-on-pin needs the "
              "channel to be a capture channel");
