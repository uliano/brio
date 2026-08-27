// mcu: samc21e18a samc21g18a samc21j18a
// A Uart's optional DMA engines are named by CHANNEL, and a channel past
// DMAC_CH_NUM does not exist. The refusal has to reach through the
// engine and the task: a Uart is where an application actually spells
// the number, so that is where getting it wrong must fail - not deep
// inside a DmaChannel instantiation nobody wrote by hand.
#include "samc/dmac.hpp"
#include "samc/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

using Bad = Uart<0, pads, 64, 256, DmaTxEngine<DMAC_CH_NUM>>;

void f() { (void)Bad::tx_idle(); }
