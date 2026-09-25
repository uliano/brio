// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A BLOCK ENGINE ON A CHANNEL THAT IS NOT THE ADC's. There is no
// request multiplexer here: RM table 11-5 wires the regular group's
// request to DMA channel 1 and to no other, so an engine named on
// another channel would sit waiting for a datum that never arrives -
// a wedge with no error flag, which is exactly what a refusal is for.
#include "ch32vx03/adc.hpp"
#include "ch32vx03/dma.hpp"

using WrongChannel = brio::DmaPingPongEngine<1, 3, uint16_t>;
void f() { brio::Adc<1>::claim_stream<WrongChannel>(); }
