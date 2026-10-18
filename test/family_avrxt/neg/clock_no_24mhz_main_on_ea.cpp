// mcu: avr64ea48 avr64ea28
// CLK_MAIN tops at 20 MHz on the AVR EA (DS40002443A 35.10.5): a 24 MHz
// source is refused even prescaled to a CLK_PER the part reaches.
#include "avrxt/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::external, 24'000'000, ClockDiv::div6, 16'000'000>::init(); }
