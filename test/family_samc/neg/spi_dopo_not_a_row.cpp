// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.DOPO (32.8.1) fixes a whole (DO, SCK, SS) TRIPLE and has only
// four rows. This wiring - DO on PAD[1], SCK on PAD[0] - is one of the
// twenty orderings the silicon does not offer, and no amount of PORT
// configuration can produce it.

#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads bad_pads{
    .data_out = SercomPad::pad1,
    .sck = SercomPad::pad0,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 5, PinFunction::d},
    .sck_pin = {'A', 4, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

using Bad = SpiHost<0, bad_pads>;
void use() { (void)Bad::start({}); }
