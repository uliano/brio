// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// No TCA1 on 28/32 pins: its event generators name an instance that is
// not there.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { (void)EvTcaOvf<1>::code; }
