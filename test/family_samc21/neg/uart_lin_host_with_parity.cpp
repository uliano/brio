// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.FORM has no LIN host code with parity (31.8.1, FORM 0x2 alone):
// a parity bit on the LIN host is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .format = {.parity = UartParity::even},
    .baud = (1u << 13) | 78u,
    .sampling = UartSampling::x16_fractional,
    .form = UartForm::lin_host,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
