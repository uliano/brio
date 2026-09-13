// Must FAIL: the notification endpoint and the bulk pair are two numbers.
#include "rp2040/platform.hpp"
#include "rp2040/usb.hpp"
#include "util/usb/cdc.hpp"

using Cdc = brio::UsbCdcAcm<brio::Usb, brio::Rp2040Platform<>, 0, 2, 2>;
bool f() { return Cdc::configure(); }
