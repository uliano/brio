// The generator runs on the 48 MHz domain, which on this family is the
// main PLL's Q output: a rate with no PLL gives it no clock at all, and
// the program is told so where it names the rate rather than later by
// CECS.
// mcu: stm32f429xx stm32f405xx
#include "stm32f4/rng.hpp"
void f() { (void)brio::Rng::init(brio::Clock<brio::ClockSource::hsi, 16'000'000>{}); }
