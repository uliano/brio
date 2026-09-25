// mcu: ch32v303rc ch32v303vc
// THE MASTER CLOCK ON A PAD THAT IS NOT THE INSTANCE'S. I2S2's MCK is
// PC6 and I2S3's PC7 (the CH32V303 datasheet's table 3-4), each on a pad
// of its own that no remap moves; a column naming PC7 for I2S2 would hand
// a pad to a function it does not carry. On the two parts with the face
// both pads are bonded, so the same check refuses an unbonded MCK pad on
// no part today - it is the pad's identity that is refused here.
#include "ch32vx03/spi.hpp"

using Out = brio::I2s<2>;
inline constexpr brio::I2sPins wrong{.ws = {'B', 12}, .ck = {'B', 13}, .sd = {'B', 15},
                                     .mck = {'C', 7}};
void f() { (void)Out::claim_pads<wrong>(brio::I2sMode::host_transmit); }
