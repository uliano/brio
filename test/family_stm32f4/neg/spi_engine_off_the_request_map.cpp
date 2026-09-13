// SPI1's transmit request is DMA2 stream 3 or stream 5, both on channel
// 3: an engine on channel 4 of stream 3 is refused before the board is
// powered rather than sitting on a stream nothing asks.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'A', 5, PinFunction::af5},
                       .miso = {'A', 6, PinFunction::af5},
                       .mosi = {'A', 7, PinFunction::af5}};
using Bus = SpiHost<1, pins, DmaTxEngine<2, 3, 4>, DmaRxEngine<2, 0, 3>>;
void f() { (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{}); }
