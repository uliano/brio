// mcu: ch32x035f8 ch32x035f7 ch32x035g8u ch32x035g8r ch32x033f8
// SPI1_RM 11's pads PC4..PC7 are not all brought out by these packages:
// a client on that column is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/spi.hpp"

using Client = brio::SpiClient<1, brio::spi1_pins_for(3)>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Client::init(clock);
}
