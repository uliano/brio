// mcu: samc21e18a samc21g18a samc21j18a
// The IrDA coder works asynchronous at 16x with SAMPR[0] = 0
// (DS60001479M 31.6.3.3) - the arithmetic generator; the fractional one
// is refused at compile time.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 13,
    .sampling = UartSampling::x16_fractional,
    .irda = true,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
