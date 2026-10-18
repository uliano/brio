// mcu: avr64ea48 avr64ea28
// SPI0 ALT6 puts SS on PF7, the UPDI pin: refused by name on the AVR EA.
#include "avrxt/spi.hpp"
using namespace brio;
void f() { (void)Spi<0>::init<SpiConfig{.route = SpiRoute::alt6}>(); }
