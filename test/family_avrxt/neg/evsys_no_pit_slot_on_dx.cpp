// mcu: avr128db48 avr128da48
// A PIT event slot other than 0 on a part without RTC.PITEVGENCTRLA.
#include "avrxt/evsys.hpp"
using namespace brio;
void f() { EventChannel<0>::source(EvPitDiv<1024, 1>{}); }
