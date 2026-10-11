// mcu: samc21e18a samc21g18a samc21j18a
// CTRLC.GTIME is RS-485's guard time (31.8.3): a guard time on the plain
// pinout is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 60503,
    .guard_time = 2,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
