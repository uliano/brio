// mcu: samc21e18a samc21g18a samc21j18a
// 32.5.1's table 32-2: a client's DI pad is its MOSI. A client that
// cannot receive has nothing to answer and cannot even see the address
// it is supposed to match, so `has_data_in = false` - which is legal
// and useful on a transmit-only HOST (32.5.1: "If the receiver is
// disabled, the data input pin can be used for other purposes") - is
// refused on this side of the wire.

#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads deaf{
    .data_out = SercomPad::pad3,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 7, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 4, PinFunction::d},
    .has_data_in = false,
};

using Bad = SpiClient<0, deaf>;
void use() { (void)Bad::selected(); }
