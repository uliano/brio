// mcu: stm32g0b1xx stm32g031xx
// The task's placement is util/placement.hpp's InFlash or InRam and
// nothing else: a tag of the program's own invention is refused where the
// port is named, never taken for one of the two.
#include "stm32g0/usart.hpp"
struct InTheCloud {};
constexpr brio::UartPins pins{.tx = {'A', 2, brio::PinFunction::af1},
                              .rx = {'A', 3, brio::PinFunction::af1}};
using U = brio::Uart<2, pins, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine, brio::UartOptions{},
                     InTheCloud>;
void f();
void f() { (void)U::isr(); }
