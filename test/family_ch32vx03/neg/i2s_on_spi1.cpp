// mcu: ch32v303rc ch32v303vc
// I2S ON SPI1. The audio face is SPI2's and SPI3's (20.3.1): table 20-1
// gives SPI1 no I2SPR, and without a prescaler there is no master; the
// datasheet names I2S2 and I2S3 alone. Refused even on a part whose other
// two instances carry the face.
#include "ch32vx03/spi.hpp"

using Wrong = brio::I2s<1>;
void f() { Wrong::enable(); }
