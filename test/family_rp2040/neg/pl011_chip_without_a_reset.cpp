// Must FAIL: a chip traits type with no way to take the block in and
// out of reset is not a Pl011Chip, and the concept says which member is
// missing.
#include "host/sim_pl011.hpp"
#include "pl011/uart.hpp"

namespace {
struct NoReset : brio::SimPl011 {
    template <uint8_t i>
    static bool reset() = delete;
};
}   // namespace

void f() { (void)brio::Pl011Uart<NoReset, 0>::enabled(); }
