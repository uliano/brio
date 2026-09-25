// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A TREE THAT FEEDS THE CONTROLLER ITS 48 MHz AND RUNS THE BUS BELOW
// THE FLOOR THIS BLOCK WAS MEASURED AT. The block reaches its buffers over
// the bus matrix, and on the CH32V303VC it carried an enumeration and a
// whole pour with HCLK at 12 MHz, lost bytes of the pour at 6 and carried
// neither at 1.5: usbfs_min_hclk_hz holds the 12 MHz, and init() refuses
// a static tree below it.
//
// The tree is written out by hand for usb_hclk_below_floor.cpp's reason:
// clock.hpp's `Clock` cannot express it - on every PLL tree of this
// family SYSCLK is the rate asked for and HPRE leaves it whole - while
// the register can (HPRE divides HCLK under a running PLL, RM 3.4.2).
#include "ch32vx03/clock.hpp"
#include "ch32vx03/usbfs.hpp"

struct DividedBus {
    static constexpr bool is_static = true;
    static constexpr uint32_t hz = 3'000'000UL;
    static constexpr uint32_t usb_hz = brio::usb_required_hz;
};

void f() { (void)brio::Usbfs<>::init(DividedBus{}); }
