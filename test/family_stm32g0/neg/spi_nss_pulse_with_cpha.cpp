// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// 35.5.12: NSS pulse mode "takes effect only if the SPI interface is
// configured as Motorola SPI master (FRF = 0) with capture on the first
// edge (CPHA = 0)".
#include "stm32g0/spi.hpp"
constexpr brio::SpiConfig cfg{.mode = brio::SpiMode::mode1,
                              .nss = brio::SpiNss::hardware_pulse};
static_assert(brio::spi_config_valid(cfg), "must be refused");
