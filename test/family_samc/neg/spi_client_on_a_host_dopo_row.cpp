// mcu: samc21e18a samc21g18a samc21j18a
// The role decides WHICH SIGNAL each pad carries (table 32-2): on a
// client DO is MISO and DI is MOSI, so the same four wires need a
// DIFFERENT DOPO row from the host's. A client built on the host's row
// 0x0 would drive its answer onto the host's MOSI line.
//
// What refuses it here is the driver's own client rule and not the
// DOPO table (row 0x0 is a legal row): a client's SS pad is claimed,
// and DI on a claimed SS pad is refused - which is exactly what
// swapping the roles without swapping the row produces.

#include "samc/spi.hpp"

using namespace brio;

constexpr SpiPads host_row{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad2,
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 6, PinFunction::d},
};

using Bad = SpiClient<0, host_row>;
void use() { (void)Bad::selected(); }
