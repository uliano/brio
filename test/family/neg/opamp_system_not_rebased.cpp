// mcu: avr128db48 avr128db64
// The OPAMP block init'ed with a DynamicClock that does not list it
// among its users: TIMEBASE would keep the old rate and every settle
// time would mean the wrong number of microseconds.
#include "avrdx/clock.hpp"
#include "avrdx/opamp.hpp"
using namespace brio;
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot>;
void f() { OpampSystem::init(Dyn{}); }
