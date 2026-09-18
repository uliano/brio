// Must FAIL: a chip traits type with no microsecond ruler is not a
// DwApbI2cChip - the unstick paces its half-bits by one, and the concept
// names it.
#include "dw_apb_i2c/i2c.hpp"
#include "host/sim_dw_apb_i2c.hpp"

namespace {
struct NoRuler : brio::SimDwApbI2c {
    static void spin_us(SpinRate rate, uint32_t us) = delete;
};
}   // namespace

void f() { (void)brio::DwApbI2cBlock<NoRuler, 0>::enabled(); }
