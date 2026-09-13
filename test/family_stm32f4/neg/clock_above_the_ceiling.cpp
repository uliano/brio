// 180 MHz is the F42x/F43x and F446 ladders' top; the F411's stops at
// 100 MHz and the F405's at 168.
// mcu: stm32f411xe stm32f407xx
#include "stm32f4/clock.hpp"
using C = brio::Clock<brio::ClockSource::pll_hsi, 180'000'000>;
void f() { (void)C::init(); }
