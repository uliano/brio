// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// No TCA1 on 28/32 pins: its event users would write a USER register
// no timer listens to.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EvTcaCntA<1>::listen(EventChannel<0>{}); }
