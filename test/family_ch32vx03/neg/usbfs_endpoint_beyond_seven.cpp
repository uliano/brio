// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The block has eight sets of endpoint registers, 0..7; RM 23.2.2 maps
// the numbers 8..15 onto the registers of 1..7, which this driver does
// not offer: endpoint 8 is refused.
#include "ch32vx03/usbfs.hpp"

bool f() {
    return brio::Usbfs<>::configure_endpoint<brio::usb_ep_in(8), brio::UsbEndpointType::bulk, 64>();
}
