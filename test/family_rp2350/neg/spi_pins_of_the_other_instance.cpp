// The pads are fixed per instance (datasheet 9.4, table 645): GP18/19/16
// are SPI0's, so an SPI1 asked to use them must not compile.
#include "rp2350/spi.hpp"
constexpr brio::SpiPins zero_pins{.sck = 18, .tx = 19, .rx = 16};
using Bad = brio::SpiHost<1, zero_pins>;
void f() { (void)sizeof(Bad); }
