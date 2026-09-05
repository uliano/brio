// mcu: stm32g030xx stm32g050xx stm32g070xx stm32g0b0xx
// The tickless timebase rides an LPTIM, and the value line has none:
// lptim_ticker.hpp gates the ticker on the same symbol lptim.hpp gates
// its resource on, so the name is unknown there and a program on the
// x0 line keeps the SysTick ticker.
#include "stm32g0/lptim_ticker.hpp"
#include "stm32g0/platform.hpp"
using P = brio::Stm32g0Platform<brio::LptimTicker<>>;
void f() { P::idle(); }
