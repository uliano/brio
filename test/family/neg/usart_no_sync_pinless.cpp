// mcu: avr128db48 avr128db28 avr128da64
// A pinless route has no XCK: the synchronous modes need one.
#include "avrdx/usart.hpp"
using namespace brio;
void f() {
    Usart<0>::init<UsartConfig{.route = UsartRoute::none, .mode = UsartMode::mspi,
                               .baud = 64u << 6}>();
}
