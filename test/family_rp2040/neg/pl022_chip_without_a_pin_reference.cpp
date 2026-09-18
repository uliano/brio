// Must FAIL: a host's chip select is a pin of the APPLICATION's, named
// at run time in the Request, so a chip traits type whose PinRef cannot
// be driven high and low is not a Pl022Chip - the concept names the two
// verbs.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

namespace {
struct NoSelect : brio::SimPl022 {
    using PinRef = int;
};

constexpr brio::SimPl022Pins pins{.sck = 0, .tx = 1, .rx = 2};
using Host = brio::Pl022Host<NoSelect, 0, pins, brio::SimPl022NoEngine, brio::SimPl022NoEngine>;
}   // namespace

void f() { (void)Host::status(); }
