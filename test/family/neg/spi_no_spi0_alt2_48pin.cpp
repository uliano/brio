// mcu: avr128db48 avr128da48
// SPI0 ALT2 is PG4..PG7: PORTG exists on 64-pin packages only.
#include "avrdx/spi.hpp"
using namespace brio;
void f() { (void)Spi<0>::init<SpiConfig{.route = SpiRoute::alt2}>(); }
