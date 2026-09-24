// mcu: ch32v203f8 ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The CH32V203F8's TSSOP20 bonds neither PA11 nor PA12 and the datasheet
// gives it no USB device controller, and the CH32V303 has none at all -
// its one full-speed controller is the host/device USBFS of RM ch. 23:
// the block is refused.
#include "ch32vx03/usb.hpp"

using Device = brio::Usbd<>;
void f() { Device::connect(true); }
