// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A full-speed bulk or interrupt packet is at most 64 bytes, and so is
// every buffer this driver hands out: a 65-byte endpoint is refused.
#include "ch32vx03/usbfs.hpp"

bool f() {
    return brio::Usbfs<>::configure_endpoint<brio::usb_ep_out(2), brio::UsbEndpointType::bulk, 65>();
}
