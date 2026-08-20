// mcu: avr128db48 avr128db28
#include "avrdx/ccl.hpp"
using namespace brio;
void f() { Lut<3>::init<LutConfig{.in0 = LutInput::mask, .truth = 1, .output_pin = true, .alt_pin = true}>(); }
