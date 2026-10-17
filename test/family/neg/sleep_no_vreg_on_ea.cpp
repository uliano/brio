// mcu: avr64ea48 avr64ea28
// The AVR EA has no regulator control in SLPCTRL (DS40002443A 13.4).
#include "avrdx/sleep.hpp"
using namespace brio;
void f() { Vreg::power(VregPower::performance); }
