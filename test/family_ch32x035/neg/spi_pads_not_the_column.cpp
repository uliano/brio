// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// SPI1_RM moves a whole column: MOSI named on PA6, the column's MISO pad,
// is no link - refused.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

using Host = brio::SpiHost<1, brio::SpiPins{.sck = {'A', 5}, .mosi = {'A', 6}, .miso = {'A', 7}, .nss = {'A', 4}}>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
