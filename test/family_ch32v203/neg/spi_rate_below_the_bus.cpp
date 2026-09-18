// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A CEILING THE BUS CANNOT MAKE: BR's slowest code is the instance's own
// bus clock over 256 (RM 20.4.1), and a rate is never rounded UP - a
// device whose datasheet limit is below that cannot sit on this bus at
// all, and the program must see it rather than clock the device too
// fast. SPI2 divides PB1, which this stratum runs at 72 MHz: its floor
// is 281.25 kHz.
#include "ch32v203/spi.hpp"

using TooSlow = brio::SpiRateOf<72'000'000UL, 200'000UL>;
void f() { (void)TooSlow::clock; }
