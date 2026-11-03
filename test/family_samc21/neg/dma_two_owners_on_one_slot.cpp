// mcu: samc21e18a samc21g18a samc21j18a
// A DMA channel has one owner for the whole image, the peripheral request it
// serves (docs/design/dma.md, "The ownership rule"): two transports of two
// SERCOMs that name the same channel - SERCOM1's port taking channel 0 while
// SERCOM0's holds it, in turn or at once - fail to compile, the error naming
// the slot, DmaSlotId<0, 0>, and both owners (util/dma_owner.hpp).
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

constexpr UartPads pads0{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};
constexpr UartPads pads1{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 16, PinFunction::c},
    .rx_pin = {'A', 17, PinFunction::c},
};

void f() {
    (void)Uart<0, pads0, 64, 256, DmaTxEngine<0>>::tx_idle();
    (void)Uart<1, pads1, 64, 256, DmaTxEngine<0>>::tx_idle();
}
