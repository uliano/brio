// Must FAIL: a device with no class answers enumeration and nothing else.
#include "rp2040/usb.hpp"
#include "util/usb/device.hpp"

struct D {
    static constexpr auto device = brio::usb_device_descriptor({.vendor_id = 1, .product_id = 1});
    static constexpr auto configuration = brio::usb_configuration_head(9, 0);
    static std::span<const uint8_t> string(uint8_t) { return {}; }
};
using Dev = brio::UsbDevice<brio::Usb, D>;
void f() { Dev::start(); }
