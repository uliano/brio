// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.TXPO (31.8.1) offers TxD on SERCOM PAD[0] or PAD[2] and on no
// other pad. A two-wire link therefore cannot simply swap its two pads
// when RX and TX turn out crossed on a board: PAD[1] is a receive pad,
// full stop, and asking for a transmitter there is refused here rather
// than on the wire.
#include "samc/sercom.hpp"
using namespace brio;

constexpr UartPads swapped{
    .tx = SercomPad::pad1,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 5, PinFunction::d},
    .rx_pin = {'A', 4, PinFunction::d},
};

void f() { (void)Uart<0, swapped>::tx_idle(); }
