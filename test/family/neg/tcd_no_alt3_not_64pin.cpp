// mcu: avr128db28 avr128db32 avr128db48 avr128da28 avr128da32 avr128da48
// TCD ALT3 is PG4..PG7: PORTG exists on the 64-pin packages only.
#include "avrdx/tcd.hpp"
using namespace brio;
void f() { (void)Tcd<0>::init<TcdConfig{.route = TcdRoute::alt3}>(); }
