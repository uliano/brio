// One stream a request: SPI1's transmit is wired to DMA2's stream 3 and
// stream 5, both on cell 3 (RM0090 table 44), and a request offered two
// streams would be served by both if both were bound to it. One host type
// on stream 3 and another on stream 5 is one request on two slots,
// refused (util/dma_owner.hpp).
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using First = SpiHost<1, pins, DmaTxEngine<2, 3, 3>, DmaRxEngine<2, 2, 3>>;
using Second = SpiHost<1, pins, DmaTxEngine<2, 5, 3>, DmaRxEngine<2, 2, 3>>;
void f() {
    (void)First::init(Clock<ClockSource::hsi, 16'000'000>{});
    (void)Second::init(Clock<ClockSource::hsi, 16'000'000>{});
}
