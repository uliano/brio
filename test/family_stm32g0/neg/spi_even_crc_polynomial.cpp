// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 35.9.5: "The polynomial value should be odd only. No even value is
// supported" - and the silicon does not check.
#include "stm32g0/spi.hpp"
constexpr brio::SpiConfig cfg{.crc = true, .crc_polynomial = 0x1020u};
static_assert(brio::spi_config_valid(cfg), "must be refused");
