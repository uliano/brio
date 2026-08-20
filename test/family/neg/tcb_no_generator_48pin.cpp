// mcu: avr128db48 avr128da48
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EventChannel<0>::source(EvTcbCapt<4>{}); }
