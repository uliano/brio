// The F401, F411 and F412 have no DAC at all, and their device header
// declares no DAC_BASE - so the resource does not exist there rather than
// answering false.
// mcu: stm32f411xe stm32f401xe stm32f412zx
#include "stm32f4/dac.hpp"
void f() { (void)brio::Dac::steps; }
