// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// A STREAM FROM THE SECOND CONVERTER OF A CH32V203. 12.2.7's note 2 gives
// the DMA request to ADC1 alone, and the tables of this class have no row
// for ADC2 - its data reaches memory in a dual mode, through the master's
// register - so a stream claimed on it is refused. (The CH32V303's
// table 11-3 does give ADC2 a row, on DMA2.)
#include "ch32vx03/adc.hpp"
#include "ch32vx03/dma.hpp"

using Any = brio::DmaPingPongEngine<1, 1, uint16_t>;
void f() { brio::Adc<2>::claim_stream<Any>(); }
