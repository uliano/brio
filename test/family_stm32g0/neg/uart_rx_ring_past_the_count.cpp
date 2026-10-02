// mcu: stm32g0b1xx
// With a receive engine the transport's receive ring is ONE circular
// block, and CNDTR counts 65535 elements at most (RM0444 10.6.4): a ring
// of 65536 bytes is refused where the transport is named, not wrapped at
// the wrong length by a count register sixteen bits wide.
#include "stm32g0/dma.hpp"
#include "stm32g0/usart.hpp"
constexpr brio::UartPins pins{.tx = {'A', 2, brio::PinFunction::af1},
                              .rx = {'A', 3, brio::PinFunction::af1}};
using Big = brio::Uart<2, pins, 65536, 256, brio::NoDmaEngine, brio::DmaRxEngine<1, 1>>;
void f();
void f() { (void)Big::harvest(); }
