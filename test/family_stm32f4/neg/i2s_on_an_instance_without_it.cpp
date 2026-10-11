// Which instances wear the I2S face is the reference manual's memory map
// and not the device header. On the F42x/F43x it is SPI2 and SPI3 alone
// and on the F446 SPI1, SPI2 and SPI3 (RM0390 table 165): SPI4 is an SPI
// and nothing else on both.
// mcu: stm32f429xx stm32f446xx
#include "stm32f4/spi.hpp"
void f() { brio::I2s<4>::enable(); }
