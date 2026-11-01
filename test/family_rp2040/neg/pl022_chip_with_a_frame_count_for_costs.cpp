// Must FAIL: a chip traits type whose costs are a bare frame count is not
// a Pl022Chip - the host draws the short asynchronous request from the
// engines' fixed cost over the polled loop's and from the loops' turns,
// which a single count cannot state - and the concept says which member
// is wrong.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

namespace {
struct CountForCosts : brio::SimPl022 {
    static constexpr uint16_t costs = 13;
};
}   // namespace

void f() { (void)brio::Pl022Ssp<CountForCosts, 0>::enabled(); }
