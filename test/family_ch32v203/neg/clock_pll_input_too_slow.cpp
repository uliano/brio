// mcu: ch32v203f6 ch32v203g6 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The PLL's INPUT has a floor of its own (the datasheets' PLL tables,
// 4-15 and 4-16), and a rate the multiplier can make is not the same as
// a rate the PLL can take: 19.5 MHz is a 3 MHz crystal halved times
// thirteen, which asks the PLL to lock onto 1.5 MHz.
#include "ch32v203/clock.hpp"

using TooSlow = brio::Clock<brio::ClockSource::pll, 19'500'000, 3'000'000>;
void f() { (void)TooSlow::init(); }
