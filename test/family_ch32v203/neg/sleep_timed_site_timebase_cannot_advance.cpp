// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A TIMED SITE OVER A TIMEBASE THAT CANNOT BE RESYNCHRONIZED. What the
// timed site is FOR is repairing kernel time after a Stop that froze
// it, and the repair is one verb: advance(). A timebase without it
// cannot be caught up, so the site refuses the platform rather than
// letting a program believe its deadlines survive a Stop.
#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/sleep.hpp"

/// A timebase with every verb the platform needs and NOT advance().
struct UnrepairableTicker {
    static constexpr uint16_t ticks_per_second = 1000;
    static uint32_t ticks() { return 0; }
    static void pause() {}
    static void resume() {}
};

using Clk = brio::Clock<brio::ClockSource::internal, 8'000'000>;
using P = brio::Ch32v203Platform<UnrepairableTicker>;
using Timed = brio::Ch32v203TimedSleepSite<P, Clk>;

void f() {
    (void)Timed::init();
}
