// mcu: samc21j18a
// CTRLA.FORM (32.8.1) implements TWO of its sixteen codes: 0x0 (SPI
// frame) and 0x2 (SPI frame with address). 0x1 and 0x3..0xF are
// Reserved, and a Reserved code written into a register is not a
// refusal - it is undefined silicon.

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
    .role = SpiRole::client,
    .form = static_cast<SpiForm>(1),
};

void use() { (void)Spi<0>::configure<bad_cfg>(); }
