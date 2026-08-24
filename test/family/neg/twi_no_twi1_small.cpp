// mcu: avr128db28 avr128da28
// TWI1 does not exist below 32 pins: the instance itself is refused,
// not just its routes.
#include "avrdx/clock.hpp"
#include "avrdx/twi.hpp"
using namespace brio;
void f() { (void)TwiHost<1>::init(Clock<ClockSource::internal, 24'000'000>{}); }
