// mcu: ch32x035f7
// The CH32X035F7P6 reaches no USART4 column (parts/ch32x035f7.hpp): its
// requests have no channel there.
#include "ch32x035/dma_engine.hpp"

constexpr uint8_t ch = brio::DmaRequestOf<brio::DmaRequest::usart4_rx>::channel;
