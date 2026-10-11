// mcu: ch32x035f8 ch32x035f7
// The 20-pin CH32X035 parts offer no USART1 (parts/<part>.hpp): its
// requests have no channel there, and naming one is refused.
#include "ch32x035/dma_engine.hpp"

constexpr uint8_t ch = brio::DmaRequestOf<brio::DmaRequest::usart1_tx>::channel;
