// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The host/device controller must be fed exactly 48 MHz, and only a PLL
// rate of 48, 96 or 144 divides to it through USBPRE: a tree that cannot
// is refused where the driver is told to use it, not at run time.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/usbfs.hpp"

using Wrong = brio::Clock<brio::ClockSource::pll, 72'000'000>;
void f() { (void)brio::Usbfs<>::init(Wrong{}); }
