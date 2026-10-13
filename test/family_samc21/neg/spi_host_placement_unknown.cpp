// mcu: samc21e18a samc21g18a samc21j18a
// The SPI host's last parameter is a placement (util/placement.hpp):
// InFlash, the default, or InRam. Any other type is refused where the
// host is named - never an orphan section the linker would put wherever
// its defaults say.
#include "samc21/spi.hpp"
using namespace brio;

struct InTheCloud {};

constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

void f() { (void)SpiHost<0, host_pads, 0, InTheCloud>::status(); }
