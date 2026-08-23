// mcu: avr128db48 avr128db28 avr128da64
// LBME (and ODME) act on the TXD pad: with PORTMUX at NONE there is no
// pad, and a pinless loop-back receives nothing.
#include "avrdx/usart.hpp"
using namespace brio;
void f() {
    Usart<0>::init<UsartConfig{.route = UsartRoute::none, .baud = 833, .loop_back = true}>();
}
