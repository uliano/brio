// mcu: avr128db28 avr128db32 avr128da28 avr128da32
// SPI1's ALT1 is PC4..PC7: PORTC stops at PC3 on 28/32 pins (and SPI0's
// ALT1 is PORTE, which those packages do not bond at all).
#include "avrdx/spi.hpp"
using namespace brio;
void f() { (void)Spi<1>::init<SpiConfig{.route = SpiRoute::alt1}>(); }
