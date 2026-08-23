// mcu: avr128db48 avr128da48
// USART5 is 64-pin only.
#include "avrdx/usart.hpp"
using namespace brio;
void f() { Usart<5>::enable_rx(true); }
