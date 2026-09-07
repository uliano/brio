// mcu: stm32g0b1xx
// Table 205's I2S row: SPI3 has none, on any part that has a SPI3 at
// all. The reserve says so and the resource's constant is what a
// program is allowed to believe.
#include "stm32g0/spi.hpp"
static_assert(brio::Spi<3>::has_i2s_mode,
              "this static_assert MUST fail: SPI3 carries no I2S (table 205)");
