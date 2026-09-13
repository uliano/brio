// The F410 has no USB at all - neither core - and the driver is not
// there to be named on it.
// mcu: stm32f410cx stm32f410rx stm32f410tx
#include "stm32f4/usb.hpp"
void f() { (void)brio::UsbFs::endpoint_count; }
