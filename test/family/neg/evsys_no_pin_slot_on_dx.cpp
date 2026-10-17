// mcu: avr128db48 avr128da28
// A pin event slot other than 0 on a part without PORTx.EVGENCTRLA.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EventChannel<0>::source(EvPin<Pin<'A', 2>, 1>{}); }
