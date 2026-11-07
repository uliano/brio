// mcu: ch32v006f8 ch32v006d8
// SPI1's column 3 sits on PB0..PB2 and PC0: the CH32V006's 20- and 12-pin
// packages bond no port B, so the host must be REFUSED there.
#include "ch32v00x/spi.hpp"

using Host = brio::SpiHost<1, brio::spi1_pins_for(3)>;
void f() { Host::release(); }
