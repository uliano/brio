// mcu: samc21e18a samc21g18a samc21j18a
// The SPI host has no engine slot: its data phase is the polled loop or
// the pump, the DMAC being driven for the Uart's transmitter alone
// (samc21/dmac.hpp, erratum 1.10.4 - the full-duplex pair hung beside any
// third active channel). An engine argument fails to compile.
#include "samc21/spi.hpp"
using namespace brio;

struct SomeEngine {
    static constexpr bool present = true;
    static constexpr uint8_t channel = 0;
};

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

void f() { (void)SpiHost<0, host_pads, 0, SomeEngine, SomeEngine>::status(); }
