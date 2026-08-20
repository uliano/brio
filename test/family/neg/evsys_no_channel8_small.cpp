// mcu: avr128db28 avr128da28
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EventChannel<8>::pulse(); }
