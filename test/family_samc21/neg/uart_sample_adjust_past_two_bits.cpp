// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.SAMPA is two bits (DS60001479M 31.8.1, 31.6.3.10): four voting
// windows and no fifth.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 60503,
    .sample_adjust = 4,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
