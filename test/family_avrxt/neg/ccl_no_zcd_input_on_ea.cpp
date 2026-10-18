// mcu: avr64ea28 avr64ea48
// The AVR EA has no ZCD: its LUT input menu has no ZCD code, and the
// input is not a name there.
#include "avrxt/ccl.hpp"
using namespace brio;
void f() { Lut<0>::init<LutConfig{.in0 = LutInput::zcd, .truth = 0xAA}>(); }
