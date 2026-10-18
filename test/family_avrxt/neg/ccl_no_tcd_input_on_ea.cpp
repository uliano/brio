// mcu: avr64ea28 avr64ea48
// The AVR EA has no TCD: no TCD0 code in its LUT input menu.
#include "avrxt/ccl.hpp"
using namespace brio;
void f() { Lut<0>::init<LutConfig{.in0 = LutInput::tcd0, .truth = 0xAA}>(); }
