// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// USART3 exists from 48 pins up.
#include "avrdx/usart.hpp"
using namespace brio;
void f() { Usart<3>::enable_rx(true); }
