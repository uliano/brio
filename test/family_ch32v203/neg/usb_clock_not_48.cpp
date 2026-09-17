// mcu: ch32v203f6 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The device controller must be fed exactly 48 MHz, and only a PLL rate
// of 48, 96 or 144 divides to it: a tree that cannot is refused where
// the driver is told to use it, not at run time.
#include "ch32v203/clock.hpp"
#include "ch32v203/usb.hpp"

using Wrong = brio::Clock<brio::ClockSource::pll, 72'000'000>;
void f() { (void)brio::Usbd<>::init(Wrong{}); }
