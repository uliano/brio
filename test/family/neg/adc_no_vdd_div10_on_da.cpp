// mcu: avr128da48 avr128da28
// The DA has no MVIO: the DB-only mux codes must not exist there.
#include "avrdx/adc.hpp"
using namespace brio;
void f() { Adc<0>::select(AdcInput::vdd_div10); }
