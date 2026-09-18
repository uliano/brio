// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6
// ONE SPI ON THIS PART (datasheet table 2-1): every part of the series
// below the CH32V203C8 carries SPI1 alone, so the second instance must
// be refused rather than answered at an address nothing decodes.
#include "ch32v203/spi.hpp"

using Second = brio::Spi<2>;
void f() { Second::bus_clock(true); }
