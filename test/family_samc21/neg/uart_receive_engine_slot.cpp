// mcu: samc21e18a samc21g18a samc21j18a
// The Uart has ONE engine slot, the transmit one: this stratum drives the
// DMAC for one user on one channel (samc21/dmac.hpp, erratum 1.10.4 - the
// Uart's own transmit and receive pair was measured writing the SERCOM's
// registers). A receive engine is a template argument the class does not
// take, so naming one fails to compile.
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"
using namespace brio;

struct SomeReceiveEngine {
    static constexpr bool present = true;
};

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 5, PinFunction::d},
};

void f() { (void)Uart<0, pads, 64, 256, NoDmaEngine, SomeReceiveEngine>::tx_idle(); }
