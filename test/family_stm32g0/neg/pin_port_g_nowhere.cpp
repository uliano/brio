// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// No STM32G0 has a port G.
#include "stm32g0/pin.hpp"
using namespace brio;
void f() { Pin<'G', 0>::output(); }
