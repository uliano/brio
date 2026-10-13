// mcu: ch32v006k8 ch32v003f4
// A placement that is neither of util/placement.hpp's two tags: refused
// where the host is named.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/spi.hpp"

struct InTightlyCoupled {};
using Host = brio::SpiHost<1, brio::spi1_default_pins, brio::NoDmaEngine, brio::NoDmaEngine,
                           brio::spi_default_hold_off_cycles, InTightlyCoupled>;
void f() { (void)Host::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}); }
