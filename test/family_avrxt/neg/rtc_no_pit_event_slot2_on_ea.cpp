// mcu: avr64ea48 avr64ea28
// Two PIT event slots on the AVR EA (RTC.PITEVGENCTRLA): there is no
// third.
#include "avrxt/rtc.hpp"
using namespace brio;
void f() { Pit::event_divider<2>(PitEventDiv::div64); }
