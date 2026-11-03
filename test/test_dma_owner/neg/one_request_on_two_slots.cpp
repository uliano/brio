// One request bound to two slots does not compile: UART0's transmit on
// channel 10 and then on channel 11 of an RP family's DMA - the binding the
// RP datasheets forbid (RP2040 2.5.3.2, RP2350 12.6.4.2) and the STM32G0's
// DMAMUX cautions against (RM0444 11.4.4) - is refused, the error naming the
// request and both slots. (Either binding alone compiles.)
// refused: new declaration of [^ ]*constexpr brio::DmaSlotId<0, 11> brio::dma_slot_of.DmaOwner<brio::DmaPeriph::uart, 0, brio::DmaSignal::tx>.*old declaration [^ ]*constexpr brio::DmaSlotId<0, 10> brio::dma_slot_of.DmaOwner<brio::DmaPeriph::uart, 0, brio::DmaSignal::tx>
#include <stdint.h>
#include "util/dma_owner.hpp"

template <uint8_t ch, typename Owner>
struct PacedChannel {
    static_assert(brio::dma_claim<brio::DmaSlotId<0, ch>, Owner>);
    static void arm() {}
};

using Uart0Tx = brio::DmaOwner<brio::DmaPeriph::uart, 0, brio::DmaSignal::tx>;

void f() {
    PacedChannel<10, Uart0Tx>::arm();
    PacedChannel<11, Uart0Tx>::arm();
}
