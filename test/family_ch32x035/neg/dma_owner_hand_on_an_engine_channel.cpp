// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// Whoever programs a channel owns it: the SPI host's receive engine owns
// channel 2 for SPI1's receive, so a block programmed by hand on channel 2
// through DmaChannel's verbs (the hand tag, its default owner) is a second
// owner - REFUSED, the error naming DmaSlotId<1, 2>.
#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

using Host = brio::SpiHost<1, brio::spi1_default_pins, brio::DmaTxEngine<3>, brio::DmaRxEngine<2>>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
    brio::DmaChannel<2>::stop();
}
