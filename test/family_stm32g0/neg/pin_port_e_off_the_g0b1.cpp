// mcu: stm32g071xx stm32g031xx
// GPIO port E is bonded on the G0B1/G0C1 class only (the device header
// declares no GPIOE_BASE elsewhere): a Pin on it must be refused.
#include "stm32g0/pin.hpp"
using namespace brio;
void f() { Pin<'E', 0>::output(); }
