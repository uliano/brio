// A part class whose reference manual was not read has no I2S instance
// list, and the audio face is refused there rather than assumed from the
// neighbouring class - SPI2 carries it on every class that IS read.
// mcu: stm32f412zx stm32f401xe stm32f469xx
#include "stm32f4/spi.hpp"
void f() { brio::I2s<2>::enable(); }
