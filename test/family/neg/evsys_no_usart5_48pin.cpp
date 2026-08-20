// mcu: avr128db48 avr128da48
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EvUsartIrda<5>::listen(EventChannel<0>{}); }
