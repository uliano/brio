// mcu: ch32v203f8 ch32v203g8
// Two packages of the series bring out neither OSC_IN nor OSC_OUT, so
// they have no HSE in either form: a crystal tree is refused there, and
// the HSI with the PLL on it is their whole clock.
#include "ch32v203/clock.hpp"

using Crystal = brio::Clock<brio::ClockSource::crystal, 8'000'000, 8'000'000>;
void f() { (void)Crystal::init(); }
