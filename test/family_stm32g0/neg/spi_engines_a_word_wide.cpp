// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The SPI data register takes a byte or a half-word (35.9.4): engines whose
// element is a word would move two frames a beat, and the host refuses them.
#include <stdint.h>
#include "stm32g0/dma.hpp"
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{
    .sck = {'B', 3, brio::PinFunction::af0},
    .miso = {'B', 4, brio::PinFunction::af0},
    .mosi = {'B', 5, brio::PinFunction::af0},
    .nss = {},
};
using Wide = brio::SpiHost<1, pins, brio::DmaTxEngine<1, 2, uint32_t>,
                           brio::DmaRxEngine<1, 3, uint32_t>>;
static_assert(Wide::has_engines);
