// Two requests on one slot do not compile: DMA1's channel 7 given to
// USART2's transmit and then to I2C1's receive - the pair the CH32V203's
// request map ORs onto that channel - is refused, the error naming the slot
// and both owners. (Either engine alone compiles.)
// refused: new declaration of [^ ]*constexpr brio::DmaOwner<brio::DmaPeriph::i2c, 1, brio::DmaSignal::rx> brio::dma_owner_of.DmaSlotId<1, 7>.*old declaration [^ ]*constexpr brio::DmaOwner<brio::DmaPeriph::usart, 2, brio::DmaSignal::tx> brio::dma_owner_of.DmaSlotId<1, 7>
#include <stdint.h>
#include "util/dma_owner.hpp"

template <uint8_t ch, typename Elem, typename Owner>
struct Engine {
    static_assert(brio::dma_claim<brio::DmaSlotId<1, ch>, Owner>);
    static void start(const Elem*, uint16_t) {}
};

using Usart2Tx = brio::DmaOwner<brio::DmaPeriph::usart, 2, brio::DmaSignal::tx>;
using I2c1Rx = brio::DmaOwner<brio::DmaPeriph::i2c, 1, brio::DmaSignal::rx>;

void f(const uint8_t* p) {
    Engine<7, uint8_t, Usart2Tx>::start(p, 1);
    Engine<7, uint8_t, I2c1Rx>::start(p, 1);
}
