// SPI6 is the 144-pin F42x/F43x class's (and the F469's); the F446, the
// F411 and the F405 class have nothing at that number.
// mcu: stm32f446xx stm32f411xe stm32f405xx
#include "stm32f4/spi.hpp"
void f() { brio::Spi<6>::enable(); }
