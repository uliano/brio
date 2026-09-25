// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE ELEMENT TYPE IS THE BEAT. A 12-bit holding register takes a
// half-word a request; a stream of bytes pointed at DAC_R12BDHR1 would
// write eight bits of every sample and leave the top four whatever the
// bus left there, so the pairing is refused.
#include "ch32vx03/dac.hpp"
#include "ch32vx03/dma.hpp"

using Bytes = brio::DmaLoopEngine<2, 3, uint8_t>;
void f() { brio::Dac::claim_stream<1, Bytes, brio::DacFormat::right12>(); }
