// mcu: samc21e18a samc21g18a samc21j18a
// A Uart's two engines ride the budget's two channels, one each: two engines
// on one channel would program each other's block (samc21/sercom.hpp). They
// fail to compile.
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

void f() { (void)Uart<0, pads, 64, 256, DmaTxEngine<0>, DmaRxEngine<0>>::tx_idle(); }
