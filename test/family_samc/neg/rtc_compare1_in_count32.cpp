// mcu: samc21e18a samc21g18a samc21j18a
// Mode 0 has ONE 32-bit compare; the second one exists in mode 1 alone
// (24.7 against 24.9). EVCTRL bit 9 is not drawn in the mode 0 register
// summary at all, so an event asked for on compare 1 there is an event
// that can never happen.

#include "samc/rtc.hpp"

using namespace brio;

constexpr RtcConfig cfg{.mode = RtcMode::count32};
constexpr RtcEventConfig ev{.compare_out = 0x2};
static_assert(rtc_event_config_valid(cfg, ev),
              "this assertion is meant to FAIL: mode 0 has one compare");
