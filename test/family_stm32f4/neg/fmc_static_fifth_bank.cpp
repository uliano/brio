// FMC bank 1 is split into four sub-banks with a chip select each
// (FMC_NE1..NE4); banks 2..4 of the controller are the NAND and PC Card
// halves and are not reached through this type.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/fmc.hpp"
void f() { (void)brio::FmcNorPsram<5>::base; }
