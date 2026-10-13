// mcu: samc21e18a samc21g18a samc21j18a
// The Uart's last parameter is a placement (util/placement.hpp): InFlash,
// the default, or InRam. Any other type is refused where the port is
// named.
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

struct InTheCloud {};

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

void f() { (void)Uart<0, pads, 64, 256, DmaTxEngine, InTheCloud>::tx_idle(); }
