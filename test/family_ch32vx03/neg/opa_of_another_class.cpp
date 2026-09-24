// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// THE THIRD AMPLIFIER, WHICH IS ANOTHER FAMILY'S. OPA_CTLR carries four
// amplifiers and every bit description of the upper two names
// CH32F20x_D8, CH32F20x_D8C, CH32V30x_D8, CH32V30x_D8C and
// CH32V31x_D8C (RM 30.3.1) - no CH32V20x. Two amplifiers is the whole
// of this chapter here, on every part.
#include "ch32vx03/opa.hpp"

using Third = brio::Opa<3>;
void f() { Third::enable(true); }
