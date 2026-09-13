// Which instances wear the I2S face is the reference manual's memory map
// and not the device header, and on the F42x/F43x and the F446 it is
// SPI2 and SPI3 alone - SPI1 is an SPI and nothing else there.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/spi.hpp"
void f() { brio::I2s<1>::enable(); }
