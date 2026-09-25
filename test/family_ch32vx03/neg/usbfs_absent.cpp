// mcu: ch32v203f6 ch32v203g6 ch32v203k6 ch32v203k8
// The CH32V203F6, G6, K6 and K8 have no host/device controller - the
// datasheet's table 2-1 counts no USBHD for them, and the three with
// PB6/PB7 bonded give those pads no USB function: the block is refused.
#include "ch32vx03/usbfs.hpp"

using Device = brio::Usbfs<>;
void f() { Device::connect(true); }
