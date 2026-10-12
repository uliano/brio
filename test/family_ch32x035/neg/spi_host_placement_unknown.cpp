// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7
// A placement that is neither of util/placement.hpp's two tags: refused
// where the host is named.
#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/spi.hpp"

struct InTightlyCoupled {};
using Host = brio::SpiHost<1, brio::spi1_default_pins, brio::NoDmaEngine, brio::NoDmaEngine,
                           brio::spi_default_hold_off_cycles, InTightlyCoupled>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Host::init(clock);
}
