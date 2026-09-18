// Must FAIL: a chip traits type that cannot set a register's bits
// without a read-modify-write is not a Pl022Chip - the enable and the
// two live bits share SSPCR1.
#include "host/sim_pl022.hpp"
#include "pl022/spi.hpp"

namespace {
struct NoBitVerbs : brio::SimPl022 {
    static void set_bits(volatile uint32_t& reg, uint32_t bits) = delete;
};
}   // namespace

void f() { (void)brio::Pl022Ssp<NoBitVerbs, 0>::enabled(); }
