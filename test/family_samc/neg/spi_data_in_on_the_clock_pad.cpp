// mcu: samc21e18a samc21g18a samc21j18a
// CTRLA.DIPO takes any of the four pads, so this one is a DRIVER
// refusal and says so: reading data off the SCK pad is not an
// arrangement any part of 32.6.3 describes. (DI == DO is a different
// matter and is ALLOWED - that is 32.6.3.4's loop-back.)

#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads bad_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad1,          // the clock
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 5, PinFunction::d},
};
constexpr SpiConfig bad_cfg{.pads = bad_pads, .role = SpiRole::host};

void use() { (void)Spi<0>::configure<bad_cfg>(); }
