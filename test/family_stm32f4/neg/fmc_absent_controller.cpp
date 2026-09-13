// The F401, the F410, the F411 and the 48-pin F412 have no external
// memory controller of either kind, so the SDRAM banks do not exist to
// be named.
// mcu: stm32f401xe stm32f411xe stm32f410rx stm32f412cx
#include "stm32f4/fmc.hpp"
void f() { (void)brio::FmcSdram<1>::base; }
