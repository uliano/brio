// mcu: samc21e18a samc21g18a samc21j18a
// The Uart's placement moves ONE path, the transmit engine's completion
// (samc21/sercom.hpp's file header): a port with no engine has nothing it
// would place, so InRam there is refused rather than accepted and
// ignored. The per-byte isr() moves to SRAM with the vector the program
// places in `.fast_text`.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

void f() { (void)Uart<0, pads, 64, 256, NoDmaEngine, InRam>::tx_idle(); }
