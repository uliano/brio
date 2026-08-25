// SLPCTRL family smoke TU: every package must compile this
// (instantiation only). SLPCTRL is one register pair with the same
// fields on every DA/DB package - SMODE/SEN in CTRLA, PMODE/HTLLEN in
// VREGCTRL - so there are no tiers to gate; what this TU proves is that
// the surface instantiates everywhere, including the HTLLEN interlock,
// whose TWI half must compile both where TWI1 exists (48/64-pin) and
// where it does not (28/32-pin).
//
// NO NEGATIVES accompany this chapter, deliberately: nothing in it has
// a combination that can be refused at compile time. The one rule that
// must be enforced - HTLLEN against an enabled TWI client or CCL - is a
// property of two registers at run time, so its enforcement is
// high_temp_low_leakage()'s false return, exercised by test a of
// test_avr_sleep.
#include "avrdx/sleep.hpp"

using namespace brio;

// The codes ARE the register's, in both fields.
static_assert(static_cast<uint8_t>(SleepMode::idle) == SLPCTRL_SMODE_IDLE_gc);
static_assert(static_cast<uint8_t>(SleepMode::standby) == SLPCTRL_SMODE_STDBY_gc);
static_assert(static_cast<uint8_t>(SleepMode::power_down) == SLPCTRL_SMODE_PDOWN_gc);
static_assert(static_cast<uint8_t>(VregPower::normal) == SLPCTRL_PMODE_AUTO_gc);
static_assert(static_cast<uint8_t>(VregPower::performance) == SLPCTRL_PMODE_FULL_gc);

// The SLEEP instruction is compiled, never executed by this TU: a smoke
// build has no wake-up source and would stop the device for good.
[[gnu::unused]] static void never_called() {
    Sleep::sleep();
    Sleep::enter(SleepMode::idle);
    Sleep::enter(SleepMode::standby);
    Sleep::enter(SleepMode::power_down);
}

void sleep_verbs() {
    Sleep::arm(SleepMode::idle);
    Sleep::arm(SleepMode::standby);
    Sleep::arm(SleepMode::power_down);
    if (Sleep::armed() && Sleep::armed_mode() == SleepMode::standby) {
        Sleep::disarm();
    }
    Sleep::disarm();
}

void vreg_verbs() {
    Vreg::power(VregPower::performance);
    Vreg::power(VregPower::normal);
    if (Vreg::power() == VregPower::normal) {
        // Enabling can be refused (an enabled TWI client or CCL);
        // disabling never is.
        if (!Vreg::high_temp_low_leakage(true)) {
            (void)Vreg::high_temp_low_leakage(false);
        }
    }
    (void)Vreg::high_temp_low_leakage();
}
