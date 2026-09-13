// An HSE root needs its rate.
// mcu: stm32f429xx
#include "stm32f4/clock.hpp"
using C = brio::Clock<brio::ClockSource::pll_hse, 180'000'000>;
void f() { (void)C::init(); }
