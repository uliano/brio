// mcu: avr128db48 avr128da48
// SPI1 ALT2 on a 48-pin package: the header lists MOSI PB4 / MISO PB5
// and no SCK or SS, and DB errata 2.11.1 declares the whole position
// non-functional there. Refused, header or no header.
#include "avrdx/spi.hpp"
using namespace brio;
void f() { (void)Spi<1>::init<SpiConfig{.route = SpiRoute::alt2}>(); }
