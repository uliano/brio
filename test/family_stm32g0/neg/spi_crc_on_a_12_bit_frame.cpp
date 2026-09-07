// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 35.5.14: the CRC "can be fixed to 8-bit or 16-bit. For all the other
// data frame lengths, no CRC is available."
#include "stm32g0/spi.hpp"
constexpr brio::SpiConfig cfg{.bits = brio::SpiDataSize::bits12, .crc = true};
static_assert(brio::spi_config_valid(cfg), "must be refused");
