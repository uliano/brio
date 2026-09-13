// A DMA engine has to sit on a CELL of the request mapping the converter
// is really wired to: ADC1 is DMA2's stream 0 or stream 4 on channel 0,
// and stream 1 carries ADC3's request and not its own.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include "stm32f4/adc.hpp"
#include "stm32f4/dma.hpp"
static_assert(brio::Adc<1>::engine_placed<brio::DmaRxEngine<2, 1, 0, uint16_t>>(),
              "ADC1's request is not on DMA2 stream 1");
