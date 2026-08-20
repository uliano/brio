// mcu: avr128db28 avr128da28
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { Tcb<3>::enable(); }
