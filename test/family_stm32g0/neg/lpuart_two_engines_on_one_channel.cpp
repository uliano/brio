// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The Uart negative's twin over the LPUART: a DMA channel moves data one
// way, so the two directions cannot name the same one.
#include "stm32g0/dma.hpp"
#include "stm32g0/lpuart.hpp"
constexpr brio::UartPins p{.tx = {'C', 1, brio::PinFunction::af1},
                           .rx = {'C', 0, brio::PinFunction::af1}};
using Clash = brio::LpUart<1, p, 64, 256, brio::DmaTxEngine<1, 1>, brio::DmaRxEngine<1, 1>>;
void f() { (void)sizeof(Clash); }
