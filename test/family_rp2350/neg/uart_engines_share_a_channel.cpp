// Two engines of one transport must name DIFFERENT DMA channels: a
// transmit run and a receive run on one channel would each abort the
// other. The DMA chapter is not written here yet, so the pair is staged
// with a tag that says only what the check reads.
#include "rp2350/uart.hpp"
struct OneChannel {
    OneChannel() = delete;
    static constexpr bool present = true;
    static constexpr uint8_t channel = 3;
};
constexpr brio::UartPins pins{.tx = {4, brio::PinFunction::uart},
                              .rx = {5, brio::PinFunction::uart}};
using Bad = brio::Uart<1, pins, 64, 64, OneChannel, OneChannel>;
void f() { (void)sizeof(Bad); }
