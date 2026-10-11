// mcu: samc21e18a samc21g18a samc21j18a
// The synchronous host uses the EIGHT low bits of BAUD (DS60001479M
// 31.6.2.3): f_xck = f_ref / (2 (BAUD + 1)) with BAUD 0..255, so a wider
// value is a configuration the chapter does not offer.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad2, .rx = SercomPad::pad0,
             .tx_pin = {'A', 6, PinFunction::d}, .rx_pin = {'A', 4, PinFunction::d}},
    .baud = 256,
    .synchronous = true,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
