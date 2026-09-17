// mcu: ch32v203f8
// The CH32V203F8's TSSOP20 bonds neither PA11 nor PA12 and the datasheet
// gives it no USB device controller: the block is refused.
#include "ch32v203/usb.hpp"

using Device = brio::Usbd<>;
void f() { Device::connect(true); }
