// mcu: samc21e18a samc21g18a samc21j18a
// An SPI host's receive engine comes with its transmit engine: the transmit
// channel's beats are what clock the wire, and a host that writes alone names
// the transmit engine alone (samc21/spi.hpp). The receive engine alone fails
// to compile.
#include "samc21/dmac.hpp"
#include "samc21/spi.hpp"
using namespace brio;

constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};

void f() { (void)SpiHost<0, host_pads, 0, NoDmaEngine, DmaRxEngine<1>>::status(); }
