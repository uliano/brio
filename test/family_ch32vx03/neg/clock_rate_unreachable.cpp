// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The PLL multiplies by x2..x16 or x18, from the 8 MHz HSI whole or
// halved: 100 MHz is not one of the rates that makes.
#include "ch32vx03/clock.hpp"

using Odd = brio::Clock<brio::ClockSource::pll, 100'000'000>;
void f() { (void)Odd::init(); }
