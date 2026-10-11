// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The series has SPI1 alone (RM ch. 16): a second instance is refused.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

using Host = brio::SpiHost<2>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
