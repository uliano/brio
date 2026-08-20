// mcu: avr128db48
// A DynamicClock that does NOT list the task must be refused by its
// init (clock_follows): the microseconds would silently go stale.
#include "avrdx/clock.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot>;
void f() { (void)Timeout<Tcb<1>>::init(Dyn{}, 1000, EventChannel<2>{}); }
