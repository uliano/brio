// Must FAIL: endpoint zero is the control endpoint, a class cannot own it.
#include "rp2040/platform.hpp"
#include "rp2040/usb.hpp"
#include "util/usb/cdc.hpp"

using Cdc = brio::UsbCdcAcm<brio::Usb, brio::Rp2040Platform<>, 0, 1, 0>;
bool f() { return Cdc::configure(); }
