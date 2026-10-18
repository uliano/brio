// mcu: avr128db28 avr128da28
#include "avrxt/tcb.hpp"
#include "avrxt/evsys.hpp"
using namespace brio;
void f() { EvTcbCaptIn<3>::listen(EventChannel<0>{}); }
