// mcu: samc21e18a samc21g18a samc21j18a
// A request has one channel for the whole image (docs/design/dma.md, "The
// ownership rule"): two transports of SERCOM0 that name different channels for
// its transmit request fail to compile, the error naming the owner,
// DmaOwner<sercom, 0, tx>, and both slots (util/dma_owner.hpp) - while two
// transport types of one SERCOM on the same channels pass (the family
// fixtures sercom.cpp and spi.cpp).
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

void f() {
    (void)Uart<0, pads, 64, 256, DmaTxEngine<0>>::tx_idle();
    (void)Uart<0, pads, 512, 512, DmaTxEngine<1>>::tx_idle();
}
