// mcu: samc21e18a samc21g18a samc21j18a
// Collision detection compares the receiver with the transmitter, and
// 31.6.3.7 wants both enabled: COLDEN with the receiver off is refused.
#include "samc21/sercom.hpp"
using namespace brio;

constexpr SercomUartConfig cfg{
    .pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad0,
             .tx_pin = {'A', 4, PinFunction::d}, .rx_pin = {'A', 4, PinFunction::d}},
    .baud = 60503,
    .rx = false,
    .collision_detect = true,
};

void f() { (void)Sercom<0>::configure<cfg, SercomClocks{48'000'000, 24'000'000}>(); }
