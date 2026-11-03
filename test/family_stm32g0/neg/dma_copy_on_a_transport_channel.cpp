// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A copy engine owns its channel through its own tag (no request paces
// it), so a copy on the channel USART2's receive engine takes is two
// owners on one slot (docs/design/dma.md, "The ownership rule").
#include "stm32g0/dma.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins pins{.tx = {'A', 2, brio::PinFunction::af1},
                              .rx = {'A', 3, brio::PinFunction::af1}};
using Serial = brio::Uart<2, pins, 64, 256, brio::DmaTxEngine<1, 1>, brio::DmaRxEngine<1, 2>>;
using Copy = brio::DmaCopyEngine<1, 2>;
void use() {
    (void)Serial::has_tx_engine;
    Copy::arm();
}
