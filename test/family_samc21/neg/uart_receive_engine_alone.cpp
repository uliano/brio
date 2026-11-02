// mcu: samc21e18a samc21g18a samc21j18a
// A Uart's receive engine comes with its transmit engine, the pair holding
// both channels of the budget (samc21/sercom.hpp): a receive block on an idle
// line has no end, and the transport must know no other channel runs when it
// lets its own go (docs/samc21/dmac.md, "The rule"). The receive engine alone
// fails to compile.
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

void f() { (void)Uart<0, pads, 64, 256, NoDmaEngine, DmaRxEngine<0>>::tx_idle(); }
