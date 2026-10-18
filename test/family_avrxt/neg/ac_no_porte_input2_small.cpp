// mcu: avr128db28 avr128da28
#include "avrxt/ac.hpp"
using namespace brio;
void f() { Ac<2>::init<AcConfig{.positive = AcPos::ainp2}>(); }
