// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// PORTB is absent on 28/32 pins: a route on it reached through
// port_by_letter() is refused, not driven onto PORTA.
#include "avrdx/pin.hpp"
using namespace brio;
void f() { pinctrl_of('B', 4) = 0; }
