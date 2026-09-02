// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// There are sixteen EXTI GPIO lines because there are sixteen pins in a
// port: a seventeenth is refused, on every part.
#include "stm32g0/exti.hpp"
using namespace brio;
void f() { (void)ExtInt<Pin<'A', 16>>::claim(); }
