// mcu: stm32g030xx stm32g070xx stm32g0b0xx
// The x0 value line has no VREFBUF (its header declares no VREFBUF_BASE
// and no VREFBUF_TypeDef), so vref.hpp compiles its register half away
// there and a Vref spelled on such a part does not exist. The reference
// vocabulary above it still does - ref_valid() answers false for the two
// buffer codes, which test/family_stm32g0/adc.cpp asserts.
#include "stm32g0/vref.hpp"
void f() { brio::Vref::init(); }
