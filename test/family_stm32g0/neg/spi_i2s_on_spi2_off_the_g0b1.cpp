// mcu: stm32g071xx stm32g031xx
// Table 205's footnote again, the other half: I2S2 exists only on the
// G0B1/G0C1, so on every smaller part SPI2 is an SPI and nothing else.
#include "stm32g0/spi.hpp"
static_assert(brio::Spi<2>::has_i2s_mode,
              "this static_assert MUST fail: this part's SPI2 carries no I2S");
