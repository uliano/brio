// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// util/block_stream.hpp's BlockSource is not "anything with a DMA
// channel": it is caller-owned blocks handed over whole, with the
// accounting. A byte-transport engine has none of that surface, and this
// is the refusal that makes the fixture's positive claim mean something.
#include "stm32g0/dma.hpp"
#include "util/block_stream.hpp"
static_assert(brio::BlockSource<brio::DmaTxEngine<1, 1>>,
              "this static_assert MUST fail: a DmaTxEngine is not a BlockSource");
