// mcu: ch32v303rc ch32v303vc
// A channel has one owner for the whole image (docs/design/dma.md, "The
// ownership rule"), on the second controller as on the first. Table 11-3
// wires DAC1's request and UART4's receive request to DMA2's channel 3, so
// a DAC1 stream and a UART4 transport with its receive engine cannot share
// an image: the second claim names DmaSlotId<2, 3> and both owners. The
// CH32V303 parts with UART4 (the RC and the VC).
#include "ch32vx03/dac.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using P = brio::Ch32vx03Platform<>;
using Dac1Row = brio::DmaRequestOf<brio::DmaRequest::dac1>;
using Player = brio::DmaLoopEngine<Dac1Row::controller, Dac1Row::channel, uint16_t>;
using Uart4Rx = brio::DmaRequestOf<brio::DmaRequest::uart4_rx>;
using Uart4 = brio::Uart<4, P, 64, 64, brio::UartFormat{}, brio::NoDmaEngine,
                         brio::DmaRxEngine<Uart4Rx::controller, Uart4Rx::channel>>;

void both() {
    brio::Dac::claim_stream<1, Player>();
    Uart4::release();
}
