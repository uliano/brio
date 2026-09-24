// The F401, the F405 class, the F411 and the F42x/F43x have no Quad-SPI
// interface at all, and their device header declares no QSPI_R_BASE - so
// the resource does not exist there rather than answering false.
// mcu: stm32f411xe stm32f401xe stm32f429xx stm32f407xx
#include "stm32f4/quadspi.hpp"
void f() { (void)brio::Quadspi::busy(); }
