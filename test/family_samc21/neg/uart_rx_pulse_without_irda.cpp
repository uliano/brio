// mcu: samc21e18a samc21g18a samc21j18a
// RXPL is the IrDA receiver's minimum pulse (DS60001479M 31.8.5) and
// means something under CTRLB.ENC alone: a non-zero RXPL on a plain
// asynchronous link is refused at compile time.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 60503,
    .rx_pulse = 19,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
