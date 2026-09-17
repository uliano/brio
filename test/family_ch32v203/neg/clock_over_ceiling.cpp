// mcu: ch32v203f6 ch32v203g6 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// 200 MHz is a rate the PLL does make from a 25 MHz crystal (x8), and
// the part is not rated for it: the ceiling is 144 MHz.
#include "ch32v203/clock.hpp"

using TooFar = brio::Clock<brio::ClockSource::pll, 200'000'000, 25'000'000>;
void f() { (void)TooFar::init(); }
