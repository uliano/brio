// Must FAIL: a chip traits type with no way to take the block in and
// out of reset is not a Pl022Chip, and the concept says which member is
// missing.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

namespace {
struct NoReset : brio::SimPl022 {
    template <uint8_t i>
    static bool reset() = delete;
};
}   // namespace

void f() { (void)brio::Pl022Ssp<NoReset, 0>::enabled(); }
