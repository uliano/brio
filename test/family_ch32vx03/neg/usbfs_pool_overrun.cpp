// mcu: ch32v203f8 ch32v203g8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A POOL OVERRUN, caught where it can be: a pool of 64 bytes holds
// endpoint zero's buffer and nothing else, so a claim named at compile
// time could not fit even with nothing else claimed - refused there,
// where the run-time claim could only answer false.
#include "ch32vx03/usbfs.hpp"

using Zero = brio::Usbfs<64>;
bool f() {
    return Zero::configure_endpoint<brio::usb_ep_in(1), brio::UsbEndpointType::bulk, 64>();
}
