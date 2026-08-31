// mcu: samc21j18a
// 32.6.3.1's own sentence: "Preload must be disabled (CTRLB.PLOADEN=0)
// in order to use this mode." Address recognition and preloading both
// want to own the client's FIRST character of a transaction, and the
// silicon cannot give it to both.

#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads pads{
    .data_out = SercomPad::pad3,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 7, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 4, PinFunction::d},
};
constexpr SpiConfig bad_cfg{
    .pads = pads,
    .role = SpiRole::client,
    .form = SpiForm::with_address,
    .preload = true,
};

void use() { (void)Spi<0>::configure<bad_cfg>(); }
