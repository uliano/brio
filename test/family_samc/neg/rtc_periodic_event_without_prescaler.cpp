// mcu: samc21e18a samc21g18a samc21j18a
// "Periodic events are independent of the prescaler setting used by the
// RTC counter, except if CTRLA.PRESCALER is zero. Then, no periodic
// events will be generated" (24.6.8.1), and 24.8.1 says the same of the
// interrupts. A PEREO bit asked for with the prescaler OFF is written
// happily by the silicon and then never honoured - the worst kind of
// configuration, so it is refused instead.

#include "samc/rtc.hpp"

using namespace brio;

constexpr RtcConfig cfg{.prescaler = RtcPrescaler::off};
constexpr RtcEventConfig ev{.periodic_out = 0x04};
static_assert(rtc_event_config_valid(cfg, ev),
              "this assertion is meant to FAIL: the prescaler OFF silences "
              "every periodic event");
