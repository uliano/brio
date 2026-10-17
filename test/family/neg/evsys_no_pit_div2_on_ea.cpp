// mcu: avr64ea48 avr64ea28
// The PIT event slots divide CLK_RTC by 4..32768 (DS40002443A 24.13.17).
#include "avrdx/evsys.hpp"
using namespace brio;
void f() { EventChannel<0>::source(EvPitDiv<2, 0>{}); }
