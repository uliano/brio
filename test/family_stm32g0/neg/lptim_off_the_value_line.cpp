// mcu: stm32g030xx stm32g050xx stm32g070xx stm32g0b0xx
// The x0 value line has no LPTIM at all (no LPTIM1_BASE, no LPTIM_TypeDef,
// none of the CFGR bit names): lptim.hpp compiles its register half away
// and an Lptim<1> does not exist there - the vocabulary above it does.
#include "stm32g0/lptim.hpp"
void f() { brio::Lptim<1>::init(); }
