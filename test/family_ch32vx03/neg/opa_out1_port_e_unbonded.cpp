// mcu: ch32v303cb ch32v303rb ch32v303rc
// A SECOND OUTPUT ON A PACKAGE WITHOUT PORT E. On the CH32V303 every
// amplifier's OUT1 is a port-E pad (PE15, PE14, PE7, PE8 - table 3-4), and
// only the LQFP100 bonds port E, so on the CB, the RB and the RC the pad
// type is refused by `Pin` itself on the line that named it.
#include "ch32vx03/opa.hpp"

using Second = brio::OpaOut<3, brio::OpaPin::out1>;
void f() { Second::claim(); }
