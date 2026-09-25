// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// Isochronous endpoints - the "no response" codes of RM 23.2.2.8 and
// 23.2.2.9, endpoint 3's 1023 bytes - are not offered: the claim is
// refused.
#include "ch32vx03/usbfs.hpp"

bool f() {
    return brio::Usbfs<>::configure_endpoint<brio::usb_ep_in(3), brio::UsbEndpointType::isochronous, 64>();
}
