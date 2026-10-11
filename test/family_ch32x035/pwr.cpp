// Pwr family smoke TU: the voltage detector's four thresholds and its two
// verbs, on every part.
#include "ch32x035/pwr.hpp"

using namespace brio;

static_assert(pvd_rising_mv(PvdLevel::v2_1) == 2120 && pvd_falling_mv(PvdLevel::v2_1) == 2100);
static_assert(pvd_rising_mv(PvdLevel::v3_0) == 3020 && pvd_falling_mv(PvdLevel::v3_0) == 3000);
static_assert(pvd_rising_mv(PvdLevel::v4_0) - pvd_falling_mv(PvdLevel::v4_0) == 20);
static_assert(pwr_pls_mask == 0x60u && pwr_pvd0 == 0x4u);

bool smoke_pvd() {
    Pwr::pvd(PvdLevel::v3_0);
    const bool low = Pwr::supply_low();
    return Pwr::pvd() == PvdLevel::v3_0 && !low;
}
