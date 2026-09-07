// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A DMA channel moves data ONE way; pointing both directions at it
// would have each re-programming the other's block.
#include "stm32g0/dma.hpp"
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {}};
using Same = brio::SpiHost<1, pins, brio::DmaTxEngine<1, 3>, brio::DmaRxEngine<1, 3>>;
void use() { (void)Same::has_engines; }
