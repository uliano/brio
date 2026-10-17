// mcu: avr64ea48 avr64ea28
// On the AVR EA the OSCCFG fuse chooses OSCHF at 16 or 20 MHz: an
// internal clock claiming another rate is refused.
#include "avrdx/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::internal, 10'000'000>::init(); }
