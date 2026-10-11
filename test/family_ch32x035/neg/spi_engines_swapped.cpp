// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7
// RM 9.2.3: the transmit engine on channel 2 and the receive one on 3 -
// the two requests' channels crossed - is refused.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

using Host = brio::SpiHost<1, brio::spi1_default_pins, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
