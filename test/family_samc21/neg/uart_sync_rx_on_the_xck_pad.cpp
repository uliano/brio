// mcu: samc21e18a samc21g18a samc21j18a
// In a synchronous configuration XCK lands on the pad TXPO puts beside
// TxD (PAD[3] next to a PAD[2] transmitter, DS60001479M 31.8.1), so RxD
// cannot be there too: refused at compile time.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad2, .rx = SercomPad::pad3,
             .tx_pin = {'A', 6, PinFunction::d}, .rx_pin = {'A', 7, PinFunction::d}},
    .baud = 11,
    .synchronous = true,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
