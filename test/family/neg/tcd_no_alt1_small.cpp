// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// TCD ALT1 is PORTB (PB4/PB5, plus PB6/PB7 at 64 pins): 28- and 32-pin
// packages have no PORTB at all, and their headers list no ALT1 code.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() { (void)Tcd<0>::init<TcdConfig{.route = TcdRoute::alt1}>(); }
