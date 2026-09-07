// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 35.5.5: NSS output enable "is only used when the MCU is set as
// master" - a client's NSS pad is its chip select input.
#include "stm32g0/spi.hpp"
constexpr brio::SpiConfig cfg{.role = brio::SpiRole::client,
                              .nss = brio::SpiNss::hardware_output};
static_assert(brio::spi_config_valid(cfg), "must be refused");
