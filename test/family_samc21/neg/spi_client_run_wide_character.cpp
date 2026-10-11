// mcu: samc21j18a
// A client's character is eight bits or nine (32.8.2, CTRLB.CHSIZE):
// a run carries uint8_t or uint16_t, the element being the beat. A run
// of 32-bit words names a character this SERCOM cannot shift.

#include <span>

#include "samc21/spi.hpp"

using namespace brio;

constexpr SpiPads pads{
    .data_out = SercomPad::pad2,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 18, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};
using Peer = SpiClient<1, pads>;

void use() {
    static const uint32_t words[2] = {1, 2};
    (void)Peer::answer(std::span<const uint32_t>(words), 1000u);
}
