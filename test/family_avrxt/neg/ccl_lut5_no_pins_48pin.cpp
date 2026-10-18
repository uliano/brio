// mcu: avr128db48 avr128da48
#include "avrxt/ccl.hpp"
using namespace brio;
void f() { Lut<5>::init<LutConfig{.in0 = LutInput::mask, .truth = 1, .output_pin = true}>(); }
