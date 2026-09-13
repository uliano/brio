// The F405 class, the F412 and the F413/F423 carry the FSMC and not the
// FMC: the same static banks under other register names, and no SDRAM
// controller anywhere. Nothing of fmc.hpp exists there.
// mcu: stm32f407xx stm32f405xx stm32f412zx stm32f413xx
#include "stm32f4/fmc.hpp"
void f() { brio::Fmc::clock(true); }
