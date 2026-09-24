// A part whose frequency ladder the reserve has not read runs at its
// 16 MHz reset rate and nothing above it.
// mcu: stm32f401xe stm32f410cx stm32f412rx stm32f413xx
#include "stm32f4/clock.hpp"
using C = brio::Clock<brio::ClockSource::pll_hsi, 84'000'000>;
void f() { (void)C::init(); }
