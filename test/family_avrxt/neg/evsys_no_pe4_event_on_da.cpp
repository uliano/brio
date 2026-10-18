// mcu: avr128da64
// DS80000882C 2.6.1: PB6/PB7 and PE4..PE7 are not connected to the event
// system on any DA revision - a pin event there must be refused.
#include "avrxt/evsys.hpp"
using namespace brio;
void f() { EventChannel<4>::source(EvPin<Pin<'E', 4>>{}); }
