// mcu: avr128db48 avr128db28 avr128da64
// The non-normal receiver modes are asynchronous only (27.5.7).
#include "avrdx/usart.hpp"
using namespace brio;
void f() {
    Usart<0>::init<UsartConfig{.mode = UsartMode::sync, .rx_mode = UsartRxMode::clk2x,
                               .baud = 64u << 6}>();
}
