// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 34.4.7: LPUARTDIV must be at least 0x300, which is the same statement
// as "fck must be at least 3 x the baud rate". 19200 baud on the LSE's
// 32768 Hz is past it - and 9600 is the chapter's own stated ceiling.
#include "stm32g0/lpuart.hpp"
static_assert(brio::lpuart_brr(32768, 19200).value() == 0);
void f() {}
