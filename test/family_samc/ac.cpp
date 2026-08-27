// Family smoke TU for samc/ac.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three). The AC block
// is identical across the family at the register level; what differs is
// pad bonding (COMP2/3 inputs on the small packages), which is the
// full campaign's per-package legality job, not this TU's.

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/clock.hpp"
#include "samc/platform_sam.hpp"

using namespace brio;

static_assert(Ac::comparator_count == 4);
static_assert(AcComparator<0>::flag == 0x01);
static_assert(AcComparator<3>::flag == 0x08);
static_assert(AcComparator<2>::sync_mask == AC_SYNCBUSY_COMPCTRL2_Msk);

// The config word is constexpr-composable: the fixture pins one shape.
constexpr AcConfig probe_cfg{
    .positive = AcPositive::pin0,
    .negative = AcNegative::vscale,
    .speed = AcSpeed::high,
    .out = AcOut::synchronous,
};
static_assert((ac_compctrl(probe_cfg) & AC_COMPCTRL_OUT_Msk) == AC_COMPCTRL_OUT_SYNC);
static_assert((ac_compctrl(probe_cfg) & AC_COMPCTRL_ENABLE_Msk) == 0u,
              "configure() owns ENABLE; the config word must never carry it");

void block_verbs() {
    (void)Ac::init(4);
    (void)Ac::enabled();
    (void)Ac::flags();
    Ac::clear_flags(0x3F);
    (void)Ac::take_flags();
    Ac::release();
}

void comparator_verbs() {
    using C = AcComparator<1>;
    (void)C::configure(probe_cfg);
    C::scaler(31);
    (void)C::enable(true);
    (void)C::ready();
    (void)C::state();
    C::start();
    (void)C::flag_set();
    C::clear_flag();
    C::arm(true);
    (void)C::armed();
    (void)C::enable(false);
}
