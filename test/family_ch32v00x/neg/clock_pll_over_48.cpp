// mcu: ch32v006k8 ch32v003f4
// The PLL doubles its source and SYSCLK stops at 48 MHz: the PLL from a
// 25 MHz crystal must be REFUSED.
#include "ch32v00x/clock.hpp"

using Pll = brio::Clock<brio::ClockSource::pll, 48'000'000, 25'000'000>;
void f() { (void)Pll::init(); }
