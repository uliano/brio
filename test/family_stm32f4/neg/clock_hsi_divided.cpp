// The hsi root is 16 MHz undivided: the AHB prescaler is declared and
// not built.
// mcu: stm32f429xx stm32f411xe
#include "stm32f4/clock.hpp"
using C = brio::Clock<brio::ClockSource::hsi, 8'000'000>;
void f() { (void)C::init(); }
