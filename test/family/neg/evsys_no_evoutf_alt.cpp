// mcu: avr128db48 avr128db28
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EvOut<Pin<'F', 7>>::listen(EventChannel<0>{}); }
