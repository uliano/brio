// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// THE HIGH-SPEED MODE OF A CH32V203's AMPLIFIER. EXTEN_CTR2 and its four
// OPAn_HSMD bits are the CH32F20x_D8's, D8C's, CH32V30x_D8's, D8C's and
// CH32V31x_D8C's (RM 33.2.2's note) - no CH32V20x - so the bit is refused
// on every CH32V203, whose amplifiers have one speed.
#include "ch32vx03/opa.hpp"

void f() { brio::Opa<2>::high_speed(true); }
