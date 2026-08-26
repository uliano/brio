// mcu: avr128db28 avr128db32 avr128db48 avr128db64
// MUXPOS LINKOUT is "OP[n-1] output" and 35.5.8 grants it to OP1 and
// OP2 only - OP0 has no previous op amp on the POSITIVE mux.
#include "avrdx/opamp.hpp"
using namespace brio;
void f() { Opamp<0>::init<OpampConfig{.positive = OpampPos::link_out}>(); }
