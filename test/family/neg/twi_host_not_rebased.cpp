// mcu: avr128db48 avr128da48
// A I2cHost init'ed with a DynamicClock that does not list it among its
// users: MBAUD is derived from CLK_PER and would go stale.
#include "avrdx/clock.hpp"
#include "avrdx/twi.hpp"
using namespace brio;
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot>;
void f() { (void)I2cHost<0>::init(Dyn{}); }
