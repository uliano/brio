// mcu: stm32g071xx stm32g031xx
// An EXTI line reached through a port E pad: the EXTICR code for E
// exists only where the port does (the G0B1/G0C1 class), so this must
// be refused on every smaller part.
#include "stm32g0/exti.hpp"
using namespace brio;
void f() { (void)ExtInt<Pin<'E', 3>>::claim(); }
