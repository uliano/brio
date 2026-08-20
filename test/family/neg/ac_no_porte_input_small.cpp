// mcu: avr128db28 avr128da28
#include "avrdx/ac.hpp"
using namespace brio;
void f() { Ac<0>::init<AcConfig{.positive = AcPos::ainp1}>(); }
