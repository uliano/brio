// mcu: avr128db48 avr128da48
// A SpiHost init'ed with a DynamicClock that does not list it among its
// users: its cs_setup timing and its SCK ceiling would go stale.
#include "avrdx/clock.hpp"
#include "avrdx/spi.hpp"
using namespace brio;
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot>;
void f() { (void)SpiHost<0>::init(Dyn{}, 1'000'000u); }
