// mcu: samc21j18a
// 21.6.4 asks for EN32K or EN1K to be enabled before a GCLK or the RTC
// is pointed at the oscillator. A configuration enabling NEITHER would
// start an oscillator no consumer can reach - a silent failure, so it is
// refused in a constant expression.

#include "samc/osc32kctrl.hpp"

using namespace brio;

constexpr Osc32kConfig bad_cfg{.enable_32k = false, .enable_1k = false};
static_assert(brio::Osc32k::config_valid(bad_cfg),
              "this assertion is meant to FAIL: an oscillator with both "
              "outputs off reaches nothing");
