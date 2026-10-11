// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A frame of this block is 4..16 bits (RM0444 35.9.2, CR2.DS): a client
// run carries uint8_t (frames up to eight bits, packed in pairs) or
// uint16_t, the element being the beat. A run of 32-bit words names a
// frame this shifter cannot hold.
#include <span>
#include "stm32g0/spi.hpp"
constexpr brio::SpiPins pins{.sck = {'B', 3, brio::PinFunction::af0},
                             .miso = {'B', 4, brio::PinFunction::af0},
                             .mosi = {'B', 5, brio::PinFunction::af0},
                             .nss = {'A', 15, brio::PinFunction::af0}};
using Peer = brio::SpiClient<1, pins>;
void use() {
    static const uint32_t words[2] = {1, 2};
    (void)Peer::answer(std::span<const uint32_t>(words), 1000u);
}
