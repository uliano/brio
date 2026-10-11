// A frame of this block is eight bits or sixteen (RM0390 26.3.9, CR1.DFF):
// a client run carries uint8_t or uint16_t, the element being the beat. A
// run of 32-bit words names a frame this shifter cannot hold.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include <span>
#include "stm32f4/spi.hpp"
using namespace brio;
constexpr SpiPins pins{.sck = {'B', 13, PinFunction::af5}, .miso = {'B', 14, PinFunction::af5},
                       .mosi = {'B', 15, PinFunction::af5}, .nss = {'B', 12, PinFunction::af5}};
using Peer = SpiClient<2, pins>;
void f() {
    static const uint32_t words[2] = {1, 2};
    (void)Peer::answer(std::span<const uint32_t>(words), 1000u);
}
