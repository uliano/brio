// mcu: avr128db48 avr128da48
// Same route, same reason: no XDIR pin (PB7) for the RS-485 drive enable.
#include "avrdx/usart.hpp"
using namespace brio;
void f() {
    Usart<3>::init<UsartConfig{.route = UsartRoute::alt1, .baud = 833, .rs485 = true}>();
}
