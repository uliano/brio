// mcu: samc21e18a samc21g18a samc21j18a
// TE is an output on PAD[2] under the RS-485 pinout (31.8.1): RxD on that
// pad would listen to the driver enable, and is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad2,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 6, PinFunction::d}},
    .baud = 60503,
    .pinout = UartPinout::rs485,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
