// mcu: avr128db28 avr128da28
#include "avrdx/ac.hpp"
using namespace brio;
void f() { Ac<2>::init<AcConfig{.positive = AcPos::ainp2}>(); }
