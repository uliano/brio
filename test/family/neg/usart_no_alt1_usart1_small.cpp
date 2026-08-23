// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// USART1's ALT1 is PC4..PC7: PORTC stops at PC3 on 28/32 pins.
#include "avrdx/usart.hpp"
using namespace brio;
void f() { Usart<1>::init<UsartConfig{.route = UsartRoute::alt1, .baud = 833}>(); }
