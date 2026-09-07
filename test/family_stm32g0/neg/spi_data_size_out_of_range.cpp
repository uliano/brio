// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 35.5.6: "The minimum data length is 4 bits. If a data length of less
// than 4 bits is selected, it is forced to an 8-bit data frame size" -
// a register that lies about what it holds, so the driver refuses the
// code instead of writing it.
#include "stm32g0/spi.hpp"
constexpr brio::SpiConfig cfg{.bits = static_cast<brio::SpiDataSize>(2)};
static_assert(brio::spi_config_valid(cfg), "must be refused");
