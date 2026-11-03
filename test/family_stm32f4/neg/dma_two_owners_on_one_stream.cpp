// One owner a stream, for the whole image (docs/design/dma.md, "The
// ownership rule"), and the key is the stream, never the channel cell:
// SPI2's transmit (DMA1 stream 4, cell 0) and UART4's transmit (DMA1
// stream 4, cell 4) are two cells of one stream, which cannot run at once
// (RM0090 10.3.3) - two owners on one slot, refused where the second
// transport is instantiated (util/dma_owner.hpp).
// mcu: stm32f429xx stm32f446xx stm32f469xx
#include "stm32f4/dma.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/usart.hpp"
using namespace brio;
constexpr SpiPins spi_pins{.sck = {'B', 13, PinFunction::af5},
                           .miso = {'B', 14, PinFunction::af5},
                           .mosi = {'B', 15, PinFunction::af5}};
constexpr UartPins uart_pins{.tx = {'A', 0, PinFunction::af8}, .rx = {'A', 1, PinFunction::af8}};
using Bus = SpiHost<2, spi_pins, DmaTxEngine<1, 4, 0>, DmaRxEngine<1, 3, 0>>;
using Serial = Uart<4, uart_pins, 64, 256, DmaTxEngine<1, 4, 4>>;
void f() {
    (void)Bus::init(Clock<ClockSource::hsi, 16'000'000>{});
    (void)Serial::init(Clock<ClockSource::hsi, 16'000'000>{}, 9600);
}
