// mcu: stm32g071xx stm32g031xx
// Table 205's footnote: SPI3 "applies to STM32G0B1xx and STM32G0C1xx
// only". Every smaller part declares no SPI3_BASE, and the resource
// refuses the instance rather than pointing at an address the device
// has not got.
#include "stm32g0/spi.hpp"
using Third = brio::Spi<3>;
static_assert(sizeof(Third) >= 0, "instantiate it");
void use() { (void)Third::regs(); }
