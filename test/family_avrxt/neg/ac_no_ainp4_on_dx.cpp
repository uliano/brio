// mcu: avr128db48 avr128da48
// AINP4 (PC3) is the AVR EA's fifth positive input; the DA/DB has four.
#include "avrxt/ac.hpp"
using namespace brio;
void f() { Ac<0>::init<AcConfig{.positive = AcPos::ainp4}>(); }
