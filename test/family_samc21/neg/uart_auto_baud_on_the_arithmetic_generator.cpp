// mcu: samc21e18a samc21g18a samc21j18a
// Break detection and auto-baud want 16x on the FRACTIONAL generator
// (31.6.3.4, CTRLA.SAMPR = 0x1): the arithmetic one is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 60503,
    .form = UartForm::auto_baud,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
