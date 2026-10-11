// mcu: samc21e18a samc21g18a samc21j18a
// The RS-485 pinout (TXPO 0x3) puts TxD on PAD[0] and TE on PAD[2]
// (DS60001479M 31.8.1): naming TxD on PAD[2] with it is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad2, .rx = SercomPad::pad0,
             .tx_pin = {'A', 6, PinFunction::d}, .rx_pin = {'A', 4, PinFunction::d}},
    .baud = 60503,
    .pinout = UartPinout::rs485,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
