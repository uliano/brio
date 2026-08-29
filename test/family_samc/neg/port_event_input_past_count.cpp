// mcu: samc21e18a samc21g18a samc21j18a
// This family has FOUR PORT event inputs per group (PORT_EV_NUM); the
// compile-time twin refuses a fifth rather than storing past the
// register.
#include "samc/pin.hpp"
using namespace brio;
void f() {
    (void)Port<'A'>::configure_event<4, PortEventConfig{.pin = 0, .enable = true}>();
}
