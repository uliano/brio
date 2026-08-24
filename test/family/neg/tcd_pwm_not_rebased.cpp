// mcu: avr128db48 avr128da48
// A TcdPwm clocked from CLK_PER and fed a DynamicClock that does not
// list it would silently change frequency at the next set().
#include "avrdx/clock.hpp"
#include "avrdx/tcd.hpp"
using namespace brio;
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot>;
void f() {
    (void)TcdPwm<TcdRoute::def>::init(Dyn{}, {.hz = 50'000, .dead_time_ticks = 4});
}
