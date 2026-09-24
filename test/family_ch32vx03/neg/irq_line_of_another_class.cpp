// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A VECTOR OF ANOTHER DEVICE CLASS. TIM8's four lines are the CH32V303's
// (entries 59..62 of its table); the CH32V203's tails put other lines
// there, so device.hpp's Irq table states TIM8's as none on these parts
// and irq_present<>() refuses the name where it is used, instead of
// handing a driver the reset jump's index.
#include "ch32vx03/device.hpp"

void f() { (void)brio::irq_present<brio::Irq::tim8_brk>(); }
