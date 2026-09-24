// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The block has twenty-two lines, 0..21: every register of 9.5.1 carries
// bits [21:0] and nothing above them.
#include "ch32v203/exti.hpp"

using Beyond = brio::ExtiLine<22>;
void f() { (void)Beyond::arm(true); }
