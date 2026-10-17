// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// The 28/32-pin packages carry ZCD0 alone: the zero-cross generator of
// ZCD1 names an instance that is not there.
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { (void)EvZcdOut<1>::code; }
