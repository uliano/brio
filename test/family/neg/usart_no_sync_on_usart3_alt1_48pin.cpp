// mcu: avr128db48 avr128da48
// USART3 ALT1 bonds TXD/RXD (PB4/PB5) but no XCK (PB6) on 48 pins:
// asynchronous works there, synchronous cannot.
#include "avrdx/usart.hpp"
using namespace brio;
void f() {
    Usart<3>::init<UsartConfig{.route = UsartRoute::alt1, .mode = UsartMode::sync,
                               .baud = 64u << 6}>();
}
