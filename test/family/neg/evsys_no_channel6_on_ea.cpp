// mcu: avr64ea48 avr64ea28
// Six event channels on the AVR EA (DS40002443A 16.4).
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EventChannel<6>::pulse(); }
