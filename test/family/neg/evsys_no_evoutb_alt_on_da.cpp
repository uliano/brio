// mcu: avr128da64
// DS80000882C 2.6.1: EVOUT on PB7 (EVOUTB ALT1) never reaches the pin on
// a DA - refused there.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EvOut<Pin<'B', 7>>::listen(EventChannel<0>{}); }
