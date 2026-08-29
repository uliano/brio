// mcu: samc21e18a samc21g18a samc21j18a
// EVCTRL.PIDm is five bits and a PORT group has 32 pins, so the
// compile-time twin refuses a pin number the register cannot hold.
#include "samc/pin.hpp"
using namespace brio;
void f() {
    (void)Port<'A'>::configure_event<0, PortEventConfig{.pin = 32, .enable = true}>();
}
