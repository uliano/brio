// The SDRAM controller has two banks - FMC banks 5 and 6, spelled 1 and
// 2 by SDCR1/SDCR2 and by SDNE0/SDNE1. There is no third.
// mcu: stm32f429xx stm32f446xx stm32f469xx
#include "stm32f4/fmc.hpp"
void f() { (void)brio::FmcSdram<3>::base; }
