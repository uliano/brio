// mcu: samc21e18a samc21g18a samc21j18a
// A COLDEN configuration is written only with its two clocks stated
// (erratum 1.17.12, DS80000740S): the clockless compile-time configure is
// refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad0,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 4, PinFunction::d}},
    .baud = 60503,
    .collision_detect = true,
};

void f() { (void)Sercom<0>::configure<cfg>(); }
