// REFUSED: a round-robin mask with a bit above the channels the chip
// has. CS.RROBIN is nine bits wide here against the RP2040's five, and
// nine is all there is even in the larger package - eight pads and the
// temperature sensor (datasheet 12.4.3.3).
#include "rp2350/adc.hpp"

void mask() { brio::Adc::round_robin<0x200u>(); }
