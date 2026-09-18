// REFUSED IN THE QFN-60: a round-robin over nine channels. That mask is
// the QFN-80's - eight pads and the sensor - and this package has five,
// four pads and the sensor on AINSEL 4 (datasheet 12.4.2.1). The
// register field is nine bits wide on both, the die being one; what
// differs is what a bit of it reaches.
#include "rp2350/adc.hpp"

void mask() { brio::Adc::round_robin<0x1FFu>(); }
