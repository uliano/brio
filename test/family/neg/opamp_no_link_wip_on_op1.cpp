// mcu: avr128db48 avr128db64
// MUXPOS LINKWIP (OP0's ladder wiper) is OP2's alone (35.5.8).
#include "avrdx/opamp.hpp"
using namespace brio;
void f() { Opamp<1>::init<OpampConfig{.positive = OpampPos::link_wip}>(); }
