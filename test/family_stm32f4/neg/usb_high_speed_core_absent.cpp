// The OTG_HS core is the F405 class's, the F42x/F43x's, the F446's and
// the F469/F479's; every other part has the full-speed core alone and
// its header declares no second base address.
// mcu: stm32f411xe stm32f401xe stm32f412zx stm32f413xx stm32f423xx
#include "stm32f4/usb.hpp"
void f() { (void)brio::UsbHs::endpoint_count; }
