// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The PLL's OUTPUT has a floor too: 18 MHz on the CH32V20x_D6 parts and
// 40 on the CH32V203RB (datasheet table 4-15). 16 MHz is the whole HSI
// times two - a multiplier the ladder has - and is below both.
#include "ch32v203/clock.hpp"

using TooSlow = brio::Clock<brio::ClockSource::pll, 16'000'000>;
void f() { (void)TooSlow::init(); }
