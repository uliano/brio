// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// One owner a channel, for the whole image (docs/design/dma.md, "The
// ownership rule"): SPI1's transmit and USART2's transmit both named on
// DMA1's channel 1 is two owners on one slot, refused where the second
// transport is instantiated (util/dma_owner.hpp) - however the program
// meant to take the channel in turn.
#include "stm32g0/dma.hpp"
#include "stm32g0/spi.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::SpiPins spi_pins{.sck = {'B', 3, brio::PinFunction::af0},
                                 .miso = {'B', 4, brio::PinFunction::af0},
                                 .mosi = {'B', 5, brio::PinFunction::af0},
                                 .nss = {}};
constexpr brio::UartPins uart_pins{.tx = {'A', 2, brio::PinFunction::af1},
                                   .rx = {'A', 3, brio::PinFunction::af1}};
using Bus = brio::SpiHost<1, spi_pins, brio::DmaTxEngine<1, 1>, brio::DmaRxEngine<1, 2>>;
using Serial = brio::Uart<2, uart_pins, 64, 256, brio::DmaTxEngine<1, 1>>;
void use() {
    (void)Bus::has_dma;
    (void)Serial::has_tx_engine;
}
