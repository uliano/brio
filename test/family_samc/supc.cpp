// SUPC family smoke TU: the block, both brown-out detectors, the
// regulator and the bandgap. The supply controller is identical across
// the E/G/J variants, so what this TU proves is that every verb and
// every constraint instantiates from the device header alone.
#include "samc/supc.hpp"

using namespace brio;

// The sampling clock's nominal rate, in millihertz off a 1.024 kHz
// source (22.6.3.6).
static_assert(bod_sample_mhz(BodPrescaler::div2) == 512'000);
static_assert(bod_sample_mhz(BodPrescaler::div65536) == 15);

static_assert(BodVdd::config_valid(BodVddConfig{.level = 8}));
static_assert(BodVdd::config_valid(BodVddConfig{.level = BodVdd::level_max}));
static_assert(!BodVdd::config_valid(BodVddConfig{.level = 64}));

// The production fuse setting, as a register word: level 8, action
// RESET, enabled.
static_assert((BodVdd::word(BodVddConfig{.level = 8, .action = BodAction::reset},
                            true) &
               SUPC_BODVDD_ENABLE_Msk) != 0u);
static_assert((BodVdd::word(BodVddConfig{.level = 8}, false) &
               SUPC_BODVDD_ENABLE_Msk) == 0u);
static_assert((BodVdd::word(BodVddConfig{.level = 44}, false) &
               SUPC_BODVDD_LEVEL_Msk) == SUPC_BODVDD_LEVEL(44));

static_assert(vref_mv(VrefLevel::v1_024) == 1024);
static_assert(vref_mv(VrefLevel::v4_096) == 4096);
static_assert(Vref::config_valid(VrefConfig{.level = VrefLevel::v2_048}));
static_assert(!Vref::config_valid(VrefConfig{.level = static_cast<VrefLevel>(1)}));
static_assert((Vref::word(VrefConfig{.output_enable = true}) &
               SUPC_VREF_VREFOE_Msk) != 0u);

void block_verbs() {
    (void)Supc::irq();
    Supc::bus_clock(true);
    (void)Supc::status();
    (void)Supc::flags();
    (void)Supc::armed();
    Supc::arm(SupcFlag::bodvdd_detect);
    Supc::disarm(SupcFlag::all);
    Supc::clear_flags();
    (void)Supc::isr();
}

void bodvdd_verbs() {
    (void)BodVdd::configure(BodVddConfig{.level = 8,
                                         .action = BodAction::none,
                                         .hysteresis = true,
                                         .sampled = true,
                                         .run_standby = true,
                                         .sampled_in_standby = true,
                                         .prescaler = BodPrescaler::div1024},
                            true, 100);
    (void)BodVdd::configure(BodVddConfig{.level = 64}, true, 100);  // refused
    (void)BodVdd::configure(BodVddConfig{}, false, 100);            // configure only
    (void)BodVdd::enable(true, 100);
    (void)BodVdd::enabled();
    (void)BodVdd::level();
    (void)BodVdd::action();
    (void)BodVdd::hysteresis();
    (void)BodVdd::ready();
    (void)BodVdd::detected();
    (void)BodVdd::sync_ready();
    (void)BodVdd::wait_sync(100);
    (void)BodVdd::matches_fuses();
    (void)BodVdd::reg();
}

void bodcore_verbs() {
    // Read-only by construction: there is no setter to call.
    (void)BodCore::reg();
    (void)BodCore::enabled();
    (void)BodCore::action();
    (void)BodCore::hysteresis();
    (void)BodCore::ready();
    (void)BodCore::detected();
    (void)BodCore::sync_ready();
}

void vreg_and_vref_verbs() {
    (void)Vreg::enabled();
    Vreg::run_standby(true);
    (void)Vreg::run_standby();

    (void)Vref::configure<VrefConfig{.level = VrefLevel::v2_048,
                                     .output_enable = true}>();
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v4_096,
                                     .on_demand = true,
                                     .run_standby = true});
    Vref::output_enable(false);
    (void)Vref::level();
    (void)Vref::output_enabled();
    (void)Vref::reg();
}
