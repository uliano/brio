// mcu: avr64ea48 avr64ea28
// Two pin event slots per port on the AVR EA: 0 and 1.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EventChannel<0>::source(EvPin<Pin<'A', 2>, 2>{}); }
