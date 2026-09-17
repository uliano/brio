// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A channel moves data ONE WAY - one direction bit, one pair of
// addresses, one count - so the two engines of one transport must name
// two channels, which is what the request table gives them.
#include "ch32v203/dma.hpp"

static_assert(brio::dma_engines_distinct<brio::DmaTxEngine<7>, brio::DmaRxEngine<7>>(),
              "two engines of one transport must not share a DMA channel");
