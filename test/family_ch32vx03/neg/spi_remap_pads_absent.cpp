// mcu: ch32v203f6 ch32v203f8 ch32v203g8
// A COLUMN THIS PACKAGE HAS NOT GOT: SPI1's second column (table 10-32)
// is PA15/PB3/PB4/PB5, and these three packages bond neither PA15 nor
// PB3 nor PB4 - the column carries no clock here, which is an absence
// and not a remap. The parts that do bond it are family_ch32vx03/
// spi_remap.cpp's.
#include "ch32vx03/spi.hpp"

using Moved = brio::SpiHost<1, brio::spi_pins_for(1, 1)>;
void f() { (void)Moved::status(); }
