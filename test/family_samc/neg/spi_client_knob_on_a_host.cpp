// mcu: samc21j18a
// CTRLB.PLOADEN and CTRLB.SSDE are the CLIENT's (32.8.2, 32.6.3.2,
// 32.6.3.6) and CTRLB.MSSEN is the HOST's. A knob asked for on the
// wrong side of the wire is a misunderstanding worth catching at
// compile time rather than a bit the silicon quietly ignores.

#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads pads{
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};
constexpr SpiConfig bad_cfg{
    .pads = pads,
    .role = SpiRole::host,
    .preload = true,
};

void use() { (void)Spi<0>::configure<bad_cfg>(); }
