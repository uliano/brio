// A channel programmed by hand cannot take a slot an engine owns: the
// program's tag on DMA1's channel 2 beside SPI1's receive engine there is
// refused, the error naming the slot, the tag and the request. (The tag on
// a channel nothing else claims compiles.)
// refused: new declaration of [^ ]*constexpr .*ScopeCapture brio::dma_owner_of.DmaSlotId<1, 2>.*old declaration [^ ]*constexpr brio::DmaOwner<brio::DmaPeriph::spi, 1, brio::DmaSignal::rx> brio::dma_owner_of.DmaSlotId<1, 2>
#include <stdint.h>
#include "util/dma_owner.hpp"

template <uint8_t ch, typename Elem, typename Owner>
struct Engine {
    static_assert(brio::dma_claim<brio::DmaSlotId<1, ch>, Owner>);
    static void start(Elem*, uint16_t) {}
};

/// The raw channel's register surface, claimed by whoever programs it.
template <uint8_t ch, typename Owner>
struct HandChannel {
    static_assert(brio::dma_claim<brio::DmaSlotId<1, ch>, Owner>);
    static void enable() {}
};

using Spi1Rx = brio::DmaOwner<brio::DmaPeriph::spi, 1, brio::DmaSignal::rx>;
struct ScopeCapture {};

void f(uint16_t* p) {
    Engine<2, uint16_t, Spi1Rx>::start(p, 1);
    HandChannel<2, ScopeCapture>::enable();
}
