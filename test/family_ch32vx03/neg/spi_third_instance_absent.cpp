// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb
// SPI3 WHERE THE PART HAS NONE. The third synchronous port is the
// CH32V303RC's and VC's (the CH32V303 datasheet's table 2-1-1: two SPI
// on the 128 KB parts, three on the 256 KB ones); no CH32V203 has more
// than two. Refused rather than answered at an address the part may not
// decode.
#include "ch32vx03/spi.hpp"

using Third = brio::Spi<3>;
void f() { Third::bus_clock(true); }
