// mcu: samc21e18a samc21g18a samc21j18a
// CTRLC.BRKLEN is the LIN host's break length (31.8.3): a break length on
// the plain USART frame is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = (1u << 13) | 78u,
    .sampling = UartSampling::x16_fractional,
    .lin_break = 2,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
