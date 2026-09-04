// mcu: stm32g030xx stm32g050xx stm32g070xx stm32g0b0xx
// The x0 value line has no LPUART at all (no LPUART1_BASE and no
// USART_BRR_LPUART - the twenty-bit divisor mask exists only where the
// peripheral with the twenty-bit divisor does), so lpuart.hpp compiles
// its resource away there; the baud arithmetic above it stays.
#include "stm32g0/lpuart.hpp"
void f() { brio::Lpuart<1>::reset(); }
