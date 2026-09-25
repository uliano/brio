// mcu: ch32v303rc ch32v303vc
// A 24-BIT DATUM IN A 16-BIT CHANNEL. 20.4.8: CHLEN "only when DATLEN =
// 00 ... is meaningful, otherwise the channel length is fixed to 32 bits
// by hardware" - a wider datum in a 16-bit channel is not a frame this
// block makes, and a program that asked for one would count the wrong
// bit clock. A constant configuration naming it is refused.
#include "ch32vx03/spi.hpp"

using Out = brio::I2s<2>;
void f() {
    Out::configure<brio::I2sConfig{.data = brio::I2sDataLength::bits24,
                                   .channel = brio::I2sChannelLength::bits16}>();
}
