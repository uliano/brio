// mcu: avr128db28 avr128db32
// OP2 exists on 48- and 64-pin DB parts only: its registers are absent
// from the smaller packages' OPAMP_t, and opamp_count says 2.
#include "avrdx/opamp.hpp"
using namespace brio;
void f() { (void)Opamp<2>::init(OpampConfig{}); }
