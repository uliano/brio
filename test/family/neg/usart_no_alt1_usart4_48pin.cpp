// mcu: avr128db48 avr128da48
// USART4's ALT1 is PE4..PE7: a 48-pin package stops at PE3, so the
// route does not exist at all there.
#include "avrdx/usart.hpp"
using namespace brio;
void f() { Usart<4>::init<UsartConfig{.route = UsartRoute::alt1, .baud = 833}>(); }
