// mcu: avr128db48 avr128da48
#include "avrdx/tcb.hpp"
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EvTcbCaptIn<4>::listen(EventChannel<0>{}); }
