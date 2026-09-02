// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A DMA channel moves data in ONE direction. Naming the same channel for
// both of a transport's engine slots would have each re-programming the
// other's block.
#include "stm32g0/dma.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins pins{.tx = {'A', 2, brio::PinFunction::af1},
                              .rx = {'A', 3, brio::PinFunction::af1}};
using Clash = brio::Uart<2, pins, 64, 256, brio::DmaTxEngine<1, 1>, brio::DmaRxEngine<1, 1>>;
static_assert(Clash::has_tx_engine);
