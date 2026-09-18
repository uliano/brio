// Must FAIL: a chip traits type that cannot set a register's bits
// without a read-modify-write is not a Pl011Chip - the interrupt mask
// is touched from two contexts.
#include "host/sim_pl011.hpp"
#include "pl011/uart.hpp"

namespace {
struct NoBitVerbs : brio::SimPl011 {
    static void set_bits(volatile uint32_t& reg, uint32_t bits) = delete;
};
}   // namespace

void f() { (void)brio::Pl011Uart<NoBitVerbs, 0>::enabled(); }
