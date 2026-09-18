// PWR family smoke TU: the power controller whole - the mode pair, the
// Stop's two price bits with the interlock the chapter imposes, the two
// flags and their write-one clears, the wake-up pad, the supply monitor
// with both of its threshold tables, what a Standby keeps of the RAM,
// the regulator trims that live in EXTEN, and the debugger's three
// low-power bits read and never written.
//
// Two things here ARE per part and are asked of the part table rather
// than named: which RAM the first retention bit covers (the whole array
// on the CH32V20x_D6, the low 2 KB on the D8) and whether the second
// bank exists at all. The PVD's millivolts are NOT one of them - they
// are the DIE's, read at run time out of FEATURE_SIGN - which is why
// the constexpr forms take the supply level as an argument.
#include <stddef.h>

#include "ch32v203/pfic.hpp"
#include "ch32v203/pwr.hpp"

using namespace brio;

// ---- the register map (RM table 2-2) ---------------------------------------
static_assert(offsetof(PwrRegs, CTLR) == 0x00);
static_assert(offsetof(PwrRegs, CSR) == 0x04);

// ---- the bits (2.4.1, 2.4.2) -----------------------------------------------
static_assert(pwr_lpds == (1u << 0));
static_assert(pwr_pdds == (1u << 1));
static_assert(pwr_cwuf == (1u << 2));
static_assert(pwr_csbf == (1u << 3));
static_assert(pwr_pvde == (1u << 4));
static_assert(pwr_pls_mask == (0x7u << 5) && pwr_pls_shift == 5);
static_assert(pwr_dbp == (1u << 8));   // device.hpp's: the RTC's door
static_assert(pwr_r2ksty == (1u << 16) && pwr_r30ksty == (1u << 17));
static_assert(pwr_r2kvbat == (1u << 18) && pwr_r30kvbat == (1u << 19));
static_assert(pwr_ramlv == (1u << 20));
static_assert(pwr_retention_bits ==
              (pwr_r2ksty | pwr_r30ksty | pwr_r2kvbat | pwr_r30kvbat | pwr_ramlv));
// The retention field and the mode field are disjoint: 2.4.1's closing
// note resets one by a Standby wake and the other only by the backup
// domain.
static_assert((pwr_retention_bits & (pwr_lpds | pwr_pdds | pwr_pvde | pwr_pls_mask)) == 0u);
static_assert(pwr_wuf == (1u << 0) && pwr_sbf == (1u << 1));
static_assert(pwr_pvdo == (1u << 2) && pwr_ewup == (1u << 8));

// ---- the modes -------------------------------------------------------------
static_assert(!pwr_mode_resets(PwrMode::sleep));
static_assert(!pwr_mode_resets(PwrMode::stop));
static_assert(pwr_mode_resets(PwrMode::standby));
static_assert(!pwr_mode_stops_clocks(PwrMode::sleep));
static_assert(pwr_mode_stops_clocks(PwrMode::stop));
static_assert(pwr_mode_stops_clocks(PwrMode::standby));

// RAMLV is valid only with the low-power regulator (the bit's own note).
static_assert(stop_config_valid(StopConfig{}));
static_assert(stop_config_valid(StopConfig{StopRegulator::low_power, true}));
static_assert(!stop_config_valid(StopConfig{StopRegulator::main, true}));

// ---- the supply monitor ----------------------------------------------------
static_assert(pvd_level_count == 8);
static_assert(pvd_level_valid(pvd_level_highest));
static_assert(!pvd_level_valid(static_cast<PvdLevel>(8)));

// 2.4.1's two tables, both ends of each, and the hysteresis between the
// rising threshold and the falling one.
static_assert(pvd_rising_mv(PvdLevel::level0, PwrSupplyLevel::from_2v4) == 2370);
static_assert(pvd_rising_mv(PvdLevel::level7, PwrSupplyLevel::from_2v4) == 3290);
static_assert(pvd_rising_mv(PvdLevel::level0, PwrSupplyLevel::from_1v8) == 2190);
static_assert(pvd_rising_mv(PvdLevel::level7, PwrSupplyLevel::from_1v8) == 2880);
static_assert(pvd_falling_mv(PvdLevel::level0, PwrSupplyLevel::from_2v4) == 2290);
static_assert(pvd_falling_mv(PvdLevel::level7, PwrSupplyLevel::from_1v8) == 2790);
static_assert(pvd_falling_mv(PvdLevel::level5, PwrSupplyLevel::from_2v4) <
              pvd_rising_mv(PvdLevel::level5, PwrSupplyLevel::from_2v4));

// The two tables are ordered and never cross: a level of the 1.8 V die
// is always the lower threshold of the pair.
static_assert(pvd_rising_mv(PvdLevel::level4, PwrSupplyLevel::from_1v8) <
              pvd_rising_mv(PvdLevel::level4, PwrSupplyLevel::from_2v4));

// The PVD's route out is EXTI line 16, which exti.hpp names and this
// file only uses.
static_assert(Pwr::pvd_exti_line == Exti::line_pvd);
static_assert(Exti::implemented(Pwr::pvd_exti_line));

// ---- the per-part facts ----------------------------------------------------
// The wake-up pad is PA0 on every package of this series.
static_assert(Pwr::wakeup_port == 'A' && Pwr::wakeup_pin_number == 0);
static_assert(Pwr::wakeup_pad_bonded);

// The retention bits, per class: one bank on the D6, two on the D8.
// AND THE MANUAL'S NUMBER IS THE CLASS'S LARGEST ARRAY: the D6's note
// names "the 20K RAM" while four parts of this series carry ten, so
// what the bit covers is stated bounded by what the part HAS and can
// never exceed it.
static_assert(Pwr::has_upper_ram_retention == device::is_d8_class);
static_assert(Pwr::ram_retention_bytes <= device::sram_bytes);
static_assert(Pwr::ram_retention_bytes ==
              (device::is_d8_class ? 2048u
                                   : (device::sram_bytes < 20480u ? device::sram_bytes : 20480u)));

// ---- every verb ------------------------------------------------------------
void pwr_verbs() {
    Pwr::bus_clock(true);
    (void)Pwr::bus_clock();
    Pwr::open();
    (void)Pwr::ctlr();
    (void)Pwr::csr();

    // the mode pair, both ways, and the read-back
    (void)Pwr::arm(PwrMode::sleep);
    (void)Pwr::arm(PwrMode::stop);
    (void)Pwr::arm(PwrMode::stop, StopConfig{StopRegulator::low_power, true});
    (void)Pwr::arm(PwrMode::standby);
    (void)Pwr::mode();
    (void)Pwr::stop_config(StopConfig{});
    (void)Pwr::stop_config();

    // the flags
    (void)Pwr::wakeup_flag();
    (void)Pwr::standby_flag();
    Pwr::clear_wakeup_flag();
    Pwr::clear_standby_flag();
    Pwr::clear_flags();

    // the pad
    (void)Pwr::wakeup_pin(true);
    (void)Pwr::wakeup_pin();

    // the supply monitor, run-time and compile-time forms
    (void)Pwr::pvd(true, PvdLevel::level5);
    (void)Pwr::pvd<PvdLevel::level7>(true);
    (void)Pwr::pvd();
    (void)Pwr::pvd_level();
    (void)Pwr::supply_low();
    (void)Pwr::supply_level();
    (void)Pwr::pvd_rising_mv(PvdLevel::level2);
    (void)Pwr::pvd_falling_mv(PvdLevel::level2);
    (void)Pwr::arm_pvd(true);
    (void)Pwr::arm_pvd(true, ExtiSense::falling);
    (void)Pwr::arm_pvd_event(true);
    (void)Pwr::pvd_isr();
    (void)feature_sign();
    (void)pwr_supply_level();

    // what a Standby keeps
    Pwr::retain_ram(true);
    (void)Pwr::retain_ram();
    Pwr::retain_ram_on_vbat(true);
    (void)Pwr::retain_ram_on_vbat();
    Pwr::retain_upper_ram(true);
    (void)Pwr::retain_upper_ram();
    Pwr::retain_upper_ram_on_vbat(true);
    (void)Pwr::retain_upper_ram_on_vbat();

    // the regulator's trims, which live in EXTEN
    Pwr::core_voltage(2);
    (void)Pwr::core_voltage();
    Pwr::low_power_voltage(2);
    (void)Pwr::low_power_voltage();

    // what the debugger left behind: reads only
    (void)Pwr::debug_cr();
    (void)Pwr::debug_in_sleep();
    (void)Pwr::debug_in_stop();
    (void)Pwr::debug_in_standby();
    (void)Pwr::debug_holds_clocks();
}

/// The deliberate one-shot and the two instructions behind it, kept out
/// of the verb sweep because one of them does not come back.
void pwr_stop_here() {
    Pwr::wait_for_event();
    Pwr::wait_for_interrupt();
    Pwr::enter(PwrMode::standby);
}

/// The PVD's own vector, as an application binds it.
extern "C" BRIO_CH32_INTERRUPT void pvd_handler() {
    if (Pwr::pvd_isr()) {
        Pwr::clear_flags();
    }
}
