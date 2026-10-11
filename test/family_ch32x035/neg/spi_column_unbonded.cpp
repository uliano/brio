// mcu: ch32x035f8 ch32x035f7 ch32x035g8u ch32x035g8r ch32x033f8
// SPI1_RM 11 puts SCK on PC5, which these packages do not bring out: the
// column is refused.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

using Host = brio::SpiHost<1, brio::spi1_pins_for(3)>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
