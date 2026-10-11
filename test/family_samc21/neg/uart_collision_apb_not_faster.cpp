// mcu: samc21e18a samc21g18a samc21j18a
// Erratum 1.17.12 (DS80000740S): collision detection does not abort the
// transfer when the SERCOM's APB clock is not higher than its core clock -
// an APB at the core clock's own rate is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad0,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 4, PinFunction::d}},
    .baud = 60503,
    .collision_detect = true,
};

void f() { (void)Sercom<0>::configure<cfg, SercomClocks{48'000'000, 48'000'000}>(); }
