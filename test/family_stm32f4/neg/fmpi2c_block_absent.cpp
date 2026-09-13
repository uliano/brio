// The F401, the F405 class, the F411, the F42x/F43x and the F469 have no
// FMPI2C at all, and their device header declares no FMPI2C1_BASE - so the
// resource and its tasks do not exist there rather than answering false,
// which is the shape stm32f4/dac.hpp uses for a block a part class has not
// got. The chapter's pure arithmetic still compiles everywhere; only the
// register half is behind the guard.
// mcu: stm32f429xx stm32f411xe stm32f405xx stm32f401xe
#include "stm32f4/fmpi2c.hpp"
void f() { (void)brio::FmpI2c<1>::number; }
