// mcu: ch32x033f8
// The CH32X033F8P6 bonds PA7 and PB0 to one pin (datasheet table 2-1,
// note 7) and forbids both as outputs: a host driving MOSI on PA7 is
// refused.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

using Host = brio::SpiHost<1>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
