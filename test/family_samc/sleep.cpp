// Family smoke TU for samc/sleep.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The PM is one instance with one register layout on every member of the
// family, so there is no package gating to check. What this fixture pins
// is the part of the driver that is a DECISION rather than a reading of
// the header: the three implemented sleep codes, the Reserved regulator
// mode, and above all THE LADDER MAPPING - which is where this target
// differs from the AVR one and where util/power.hpp's never-deeper rule
// finally earns its keep.

#include <stdint.h>

#include "samc/platform_sam.hpp"
#include "samc/sleep.hpp"

using namespace brio;

// ---- the codes are the register's, and only three exist ---------------------

static_assert(static_cast<uint8_t>(SleepMode::idle0) == PM_SLEEPCFG_SLEEPMODE_IDLE0_Val);
static_assert(static_cast<uint8_t>(SleepMode::idle2) == PM_SLEEPCFG_SLEEPMODE_IDLE2_Val);
static_assert(static_cast<uint8_t>(SleepMode::standby) ==
              PM_SLEEPCFG_SLEEPMODE_STANDBY_Val);

static_assert(sleep_mode_valid(SleepMode::idle0));
static_assert(sleep_mode_valid(SleepMode::idle2));
static_assert(sleep_mode_valid(SleepMode::standby));
// 0x1, 0x3 and 0x5..0x7 are Reserved (19.8.1). There is no IDLE1.
static_assert(!sleep_mode_valid(static_cast<SleepMode>(1)));
static_assert(!sleep_mode_valid(static_cast<SleepMode>(3)));
static_assert(!sleep_mode_valid(static_cast<SleepMode>(5)));
static_assert(!sleep_mode_valid(static_cast<SleepMode>(7)));

// The field is three bits wide and the header agrees.
static_assert(PM_SLEEPCFG_SLEEPMODE_Msk == 0x07u);

// ---- STDBYCFG: two fields, one of them with a Reserved code -----------------

static_assert(standby_config_valid(StandbyConfig{}));
static_assert(standby_config_valid(
    StandbyConfig{.regulator = VregStandbyMode::performance}));
static_assert(standby_config_valid(StandbyConfig{.regulator = VregStandbyMode::low_power}));
static_assert(!standby_config_valid(
    StandbyConfig{.regulator = static_cast<VregStandbyMode>(3)}));

// THE DEFAULT IS THE RESET VALUE, and it is not zero: BBIASHS comes up
// set (19.8.2, reset 0x0400), which is what makes erratum 1.8.13's
// preconditions the default state of a brio program.
static_assert(StandbyConfig{}.back_bias);
static_assert(Pm::word(StandbyConfig{}) == PM_STDBYCFG_RESETVALUE);
static_assert(PM_STDBYCFG_RESETVALUE == 0x0400u);

// Only bits 6, 7 and 10 exist in that register.
static_assert(PM_STDBYCFG_Msk == 0x04C0u);
static_assert((Pm::word(StandbyConfig{.regulator = VregStandbyMode::low_power,
                                      .back_bias = false}) &
               ~static_cast<uint16_t>(PM_STDBYCFG_Msk)) == 0u);

// ---- the ladder mapping, which is this target's own decision ----------------
//
// AVR DA/DB realizes all four rungs and its mapping is the identity.
// This family has three modes for four rungs: `deep` maps DOWN to
// standby (never deeper than asked), and `light` is IDLE2 - the deepest
// idle - so that the three armed rungs are three DISTINCT register
// codes and armed() can stay a pure read of the silicon. See the
// SamSleepSite comment for what that costs (a CAN wake from `light`).

static_assert(SleepSite<SamSleepSite>);
static_assert(rung(SleepDepth::none) < rung(SleepDepth::light));
static_assert(shallower(SleepDepth::deep, SleepDepth::standby) == SleepDepth::standby);
static_assert(is_deep_mode(SleepDepth::standby) && !is_deep_mode(SleepDepth::light));

// ---- every verb instantiates ------------------------------------------------

void pm_verbs() {
    Pm::bus_clock(true);
    (void)Pm::bus_clock();

    (void)Pm::sleepcfg();
    (void)Pm::sleep_mode();
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    (void)Pm::set_sleep_mode<SleepMode::idle2>();
    (void)Pm::set_sleep_mode<SleepMode::standby>();

    (void)Pm::stdbycfg();
    (void)Pm::regulator_mode();
    (void)Pm::back_bias();
    (void)Pm::configure_standby(StandbyConfig{});
    (void)Pm::configure_standby<StandbyConfig{.regulator = VregStandbyMode::performance,
                                              .back_bias = false}>();
}

void sleep_verbs() {
    // Both of these really stop the CPU; naming them here is what checks
    // they compile, and a family TU is never run.
    Pm::sleep();
    (void)Pm::enter(SleepMode::idle0);
}

void site_verbs() {
    (void)SamSleepSite::arm(SleepDepth::none);
    (void)SamSleepSite::arm(SleepDepth::light);
    (void)SamSleepSite::arm(SleepDepth::standby);
    (void)SamSleepSite::arm(SleepDepth::deep);
    SamSleepSite::disarm();
    (void)SamSleepSite::armed();
}

// The erratum guard is a plain RAII object over a core register; it must
// instantiate outside any ticker instantiation.
void systick_guard() {
    SysTickInterruptGuard guard;
    (void)guard;
}

// And the manager itself must instantiate against this site - the whole
// point of the campaign is that util/power.hpp compiles UNCHANGED here.
using Pmgr = PowerManager<SamPlatform, SamSleepSite>;

void manager_verbs() {
    Pmgr::init();
    (void)Pmgr::ceiling();
    (void)Pmgr::armed_depth();
    PowerLock lock = Pmgr::restrict(SleepDepth::light);
    lock.release();
}

// ---- the timed site ---------------------------------------------------------
// The v2 site: the RTC as alarm and witness, the power MODEL untouched -
// the same PowerManager must take it through the same concept with no
// new member, which is this instantiation's whole claim.
using TimedSite = SamTimedSleepSite<SamPlatform>;
static_assert(SleepSite<TimedSite>);

using TimedPmgr = PowerManager<SamPlatform, TimedSite>;

void timed_site_verbs() {
    (void)TimedSite::init();
    (void)TimedSite::arm(SleepDepth::standby);
    (void)TimedSite::alarm_armed();
    (void)TimedSite::last_advance();
    TimedSite::disarm();
    TimedPmgr::init();
}

// A measured-rate configuration compiles at any plausible 32 kHz value.
constexpr TimedSleepConfig measured{.rtc_hz = 32907, .clock = RtcClock::ulp_32k};
using MeasuredSite = SamTimedSleepSite<SamPlatform, measured>;
static_assert(SleepSite<MeasuredSite>);

// And the ticker's resync verb exists at every rate the family builds.
void advance_verb() { Ticker::advance(123u); }
