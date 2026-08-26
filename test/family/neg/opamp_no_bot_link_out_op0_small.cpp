// mcu: avr128db28 avr128db32
// MUXBOT LINKOUT of OP0 is OP2's output (35.5.7 note 1): on a package
// without OP2 the bottom of OP0's ladder would hang on nothing.
#include "avrdx/opamp.hpp"
using namespace brio;
void f() { Opamp<0>::init<OpampConfig{.bottom = OpampBot::link_out}>(); }
