// mcu: avr64ea48 avr64ea28
// SPI0 code 2 is reserved on the AVR EA (DS40002443A 17.3.5): the route
// has no name there.
#include "avrxt/spi.hpp"
using namespace brio;
void f() { (void)Spi<0>::init<SpiConfig{.route = SpiRoute::alt2}>(); }
