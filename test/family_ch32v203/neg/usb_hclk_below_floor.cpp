// mcu: ch32v203f6 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A TREE THAT FEEDS THE CONTROLLER ITS 48 MHz AND RUNS THE BUS BELOW
// THE MEASURED FLOOR. The block reaches its packet memory over the bus
// matrix and what it needs there is cycles: it carries data with HCLK
// at 96, 48 and 24 MHz and loses packets at 12, so init() refuses
// anything under usbd_min_hclk_hz.
//
// The tree is written out here rather than taken from clock.hpp's
// `Clock`, because that task cannot express it: on every PLL tree of
// this family SYSCLK is the rate asked for and HPRE leaves it whole, so
// a Clock whose USB divider makes 48 MHz runs the core at 48, 96 or
// 144. What CAN make this shape is the register - HPRE divides HCLK
// under a running PLL (RM 3.4.2) - and so could a later clock task, and
// the refusal is stated where the driver, not the tree, is the one that
// knows the floor.
#include "ch32v203/clock.hpp"
#include "ch32v203/usb.hpp"

struct DividedBus {
    static constexpr uint32_t hz = 12'000'000UL;
    static constexpr uint32_t usb_hz = brio::usb_required_hz;
};

void f() { (void)brio::Usbd<>::init(DividedBus{}); }
