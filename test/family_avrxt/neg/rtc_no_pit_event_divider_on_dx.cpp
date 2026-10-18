// mcu: avr128db48 avr128da28
// The AVR DA/DB has no PIT event slots (no RTC.PITEVGENCTRLA): its PIT
// dividers are fixed event generators, and the RTC-side verb is not a
// name there.
#include "avrxt/rtc.hpp"
using namespace brio;
void f() { Pit::event_divider<0>(PitEventDiv::div64); }
