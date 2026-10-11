// mcu: samc21e18a samc21g18a samc21j18a
// The fractional generator (CTRLA.SAMPR 0x1 or 0x3) divides by BAUD + FP/8
// with a 13-bit integer part that "must be greater than or equal to 1"
// (DS60001479M 30.6.2.3): a register whose low thirteen bits are zero -
// here FP alone - is refused at compile time.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 5, PinFunction::d}},
    .baud = 0x2000,
    .sampling = UartSampling::x16_fractional,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
