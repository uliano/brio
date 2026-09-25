// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// HSRXEN2 ON A CH32V203. The second high-speed read mode is the
// CH32F20x_D8's, the CH32V30x_D8's and the D8C's (20.4.10's note), on some
// of their lots; on a CH32V203 bit 2 of HSCR is no field at all. Refused
// at compile time.
#include "ch32vx03/spi.hpp"

using One = brio::Spi<1>;
void f() { (void)One::high_speed_read2(true, 144'000'000UL); }
