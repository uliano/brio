// mcu: samc21e18a samc21g18a samc21j18a
// A DMA channel moves bytes in ONE direction: it has one descriptor,
// one source and one destination. Pointing a Uart's transmit and receive
// engines at the same channel would have each direction re-programming
// the other's descriptor - the transfers would not merely interleave
// badly, they would destroy each other. Refused at the template
// argument, where the number is typed.
#include "samc/dmac.hpp"
#include "samc/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

using Bad = Uart<0, pads, 64, 256, DmaTxEngine<3>, DmaRxEngine<3>>;

void f() { (void)Bad::tx_idle(); }
