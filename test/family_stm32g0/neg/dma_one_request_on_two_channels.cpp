// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// One channel a request: the DMAMUX may route a request id to two channels
// only where they are never active together (RM0444 11.4.4, the caution),
// which is in-turn use, and the rule forbids that too. SPI1's transmit on
// channel 1 through one host type and on channel 3 through another is one
// request on two slots, refused (util/dma_owner.hpp).
#include "stm32g0/dma.hpp"
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {}};
using First = brio::SpiHost<1, pins, brio::DmaTxEngine<1, 1>, brio::DmaRxEngine<1, 2>>;
using Second = brio::SpiHost<1, pins, brio::DmaTxEngine<1, 3>, brio::DmaRxEngine<1, 2>>;
void use() {
    (void)First::has_dma;
    (void)Second::has_dma;
}
