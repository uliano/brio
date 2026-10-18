// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// No TCA1 on 28/32 pins, and no TCA1 code in the LUT input menu: the
// input is not a name there.
#include "avrxt/ccl.hpp"
using namespace brio;
void f() { Lut<0>::init<LutConfig{.in0 = LutInput::tca1, .truth = 0xAA}>(); }
