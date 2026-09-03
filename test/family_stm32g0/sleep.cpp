// PWR + the sleep sites family smoke TU (stm32g0/pwr.hpp,
// stm32g0/sleep.hpp): chapter 4's whole register surface and the two
// util/power.hpp adapters over it. What is per-variant here is the
// SPARSE set of wake-up pins and the pull registers that follow the GPIO
// bonding, both answered by the reserve - so this TU also proves that a
// verb handed a pin or a port the part has not got refuses instead of
// writing a bit that is not there.
#include "kernel/kernel.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/sleep.hpp"
#include "util/power.hpp"

using namespace brio;

// ---- chapter 4's own vocabulary --------------------------------------------

static_assert(pwr_mode_valid(PwrMode::sleep));
static_assert(pwr_mode_valid(PwrMode::stop0));
static_assert(pwr_mode_valid(PwrMode::stop1));
static_assert(pwr_mode_valid(PwrMode::standby));
static_assert(pwr_mode_valid(PwrMode::shutdown));
// LPMS 010 is Reserved (4.4.1) and is not a spelling this driver has.
static_assert(!pwr_mode_valid(static_cast<PwrMode>(2)));
static_assert(!pwr_mode_valid(static_cast<PwrMode>(7)));

// The one distinction anything above this file cares about: does the
// program come back through the reset vector?
static_assert(!pwr_mode_resets(PwrMode::sleep));
static_assert(!pwr_mode_resets(PwrMode::stop0));
static_assert(!pwr_mode_resets(PwrMode::stop1));
static_assert(pwr_mode_resets(PwrMode::standby));
static_assert(pwr_mode_resets(PwrMode::shutdown));
static_assert(!pwr_mode_stops_clocks(PwrMode::sleep));
static_assert(pwr_mode_stops_clocks(PwrMode::stop0));

// 4.2.2: the falling threshold must sit below the rising one, and the
// pad mode ignores the falling field altogether.
static_assert(pvd_config_valid(PvdConfig{}));
static_assert(!pvd_config_valid({.rising = PvdRising::v2_1,
                                 .falling = PvdFalling::v2_9}));
static_assert(pvd_config_valid({.rising = PvdRising::pvd_in,
                                .falling = PvdFalling::v2_9}));

// ---- the per-part geometry the reserve answers for --------------------------

// WKUP1, WKUP2, WKUP4 and WKUP6 are bonded on every part of the family;
// 3 and 5 are not, which is exactly the sort of fact a driver must not
// hard-code.
static_assert(Pwr::wakeup_pin_present(1));
static_assert(Pwr::wakeup_pin_present(2));
static_assert(Pwr::wakeup_pin_present(4));
static_assert(Pwr::wakeup_pin_present(6));
static_assert(!Pwr::wakeup_pin_present(0));
static_assert(!Pwr::wakeup_pin_present(7));
static_assert(pwr_wakeup_pin_count() >= 4 && pwr_wakeup_pin_count() <= 6);

// ---- the ladder -------------------------------------------------------------

using SysClock = Clock<ClockSource::pll, 64'000'000>;
using Site = Stm32SleepSite<SysClock>;
using TimedSite = Stm32TimedSleepSite<Stm32Platform, SysClock>;

static_assert(SleepSite<Site>);
static_assert(SleepSite<TimedSite>);

// The Clock task decides which SWS value means "the Stop did not
// happen": a PLL program has one to detect, an HSISYS program has none.
static_assert(Site::expected_source == SysclkSource::pllrclk);
static_assert(Stm32SleepSite<Clock<ClockSource::internal, 16'000'000>>::
                  expected_source == SysclkSource::hsisys);

// The timed site's own arithmetic, at the default over-estimate.
static_assert(timed_sleep_config_valid(TimedSleepConfig{}));
static_assert(!timed_sleep_config_valid({.rtcclk_hz = 512}));
static_assert(!timed_sleep_config_valid({.fast_clock = RtcWakeupClock::ck_spre}));
// ck_wut is rounded UP, which is the direction that makes a placed alarm
// land late rather than early.
static_assert(TimedSite::fast_hz == (33'000 + 15) / 16);
static_assert(TimedSite::prescalers.async != 0xFF);
static_assert(rtc_ck_spre_hz(TimedSite::prescalers, 33'000) == 1);
// A 16-bit reload at that rate covers half a minute of kernel ticks.
static_assert(TimedSite::fast_span_ticks > 25'000);

void pwr_verbs() {
    Pwr::bus_clock(true);
    (void)Pwr::bus_clock();
    (void)Pwr::cr1();
    (void)Pwr::cr2();
    (void)Pwr::cr3();
    (void)Pwr::cr4();
    (void)Pwr::sr1();
    (void)Pwr::sr2();
    (void)Pwr::range();
    (void)Pwr::range(1);
    (void)Pwr::range_changing();
    Pwr::rtc_domain_unlock(true);
    (void)Pwr::rtc_domain_unlocked();
    Pwr::flash_power_down_stop(false);
    (void)Pwr::flash_power_down_stop();
    Pwr::flash_power_down_lp_sleep(false);
    Pwr::flash_power_down_lp_run(false);
    (void)Pwr::flash_ready();
    (void)Pwr::low_power_run();
    (void)Pwr::low_power_run(false);
    (void)Pwr::on_low_power_regulator();
    (void)Pwr::low_power_regulator_ready();
    Pwr::deep_sleep(false);
    (void)Pwr::deep_sleep();
    (void)Pwr::lpms();
    (void)Pwr::arm(PwrMode::sleep);
    (void)Pwr::mode();
    (void)Pwr::stop_hsidiv_hazard();
    (void)Pwr::wakeup_pin(1, false);
    (void)Pwr::wakeup_pin_enabled(1);
    (void)Pwr::wakeup_flag(1);
    (void)Pwr::standby_flag();
    (void)Pwr::internal_wakeup_flag();
    Pwr::internal_wakeup(true);
    (void)Pwr::internal_wakeup();
    Pwr::clear_wakeup_flags();
    Pwr::sram_retention(true);
    (void)Pwr::sram_retention();
    Pwr::sampled_supply_monitor(false);
    (void)Pwr::pvd_config(PvdConfig{});
    Pwr::pvd_enable(false);
    (void)Pwr::pvd_enabled();
    (void)Pwr::pvd_below();
    Pwr::dac_supply_monitor(false);
    (void)Pwr::dac_supply_low();
    Pwr::apply_pulls(false);
    (void)Pwr::apply_pulls();
    (void)Pwr::standby_pull('A', 0, true, false);
    (void)Pwr::standby_pull('G', 0, true, false);   // no such port anywhere
    static_assert(Pwr::pvd_exti_line == 16);
    if (false) {
        Pwr::enter(PwrMode::standby);   // one-way: named, not called
    }
}

void site_verbs() {
    (void)Site::arm(SleepDepth::standby);
    Site::disarm();
    (void)Site::armed();
    (void)Site::resume_clock();

    (void)TimedSite::init();
    (void)TimedSite::ready();
    (void)TimedSite::arm(SleepDepth::deep);
    TimedSite::disarm();
    (void)TimedSite::armed();
    (void)TimedSite::alarm_armed();
    (void)TimedSite::last_advance();
    (void)TimedSite::last_reload();
    (void)TimedSite::last_alarm_was_fast();
    (void)TimedSite::place_alarm(500);
    TimedSite::resync();
    TimedSite::isr();
}

// ---- the THIRD site: the same lift, on an LPTIM ------------------------------
//
// Its config is checked against the same three rules on every header,
// and the negatives lptim_site_on_pclk.cpp and
// lptim_site_slower_than_the_tick.cpp are the other half of the claim.

static_assert(lptim_sleep_rate_hz(LptimTimedSleepConfig{}) == 32'768u,
              "the LSE default is the crystal's exact rate");
static_assert(lptim_sleep_rate_hz(LptimTimedSleepConfig{.source = LptimClock::lsi}) ==
                  34'000u,
              "and the LSI default is DS13560's UPPER bound - the direction the "
              "rate rule wants");
static_assert(lptim_sleep_counter_hz(LptimTimedSleepConfig{}) == 1024u,
              "32768 / 32");
static_assert(lptim_timed_sleep_config_valid(LptimTimedSleepConfig{}, 1000u));
static_assert(lptim_timed_sleep_config_valid(
    LptimTimedSleepConfig{.instance = 2, .source = LptimClock::lsi,
                          .rate_hz = 33'000}, 1000u));
static_assert(!lptim_timed_sleep_config_valid(
                  LptimTimedSleepConfig{.source = LptimClock::pclk}, 1000u),
              "PCLK stops with the VCORE domain");
static_assert(!lptim_timed_sleep_config_valid(
                  LptimTimedSleepConfig{.source = LptimClock::hsi16}, 1000u),
              "and HSI16 is a clock request ES0548 2.2.4 breaks");
static_assert(!lptim_timed_sleep_config_valid(
                  LptimTimedSleepConfig{.prescaler = LptimPrescaler::div64}, 1000u),
              "512 counts a second cannot repair a 1 kHz tick");
static_assert(!lptim_timed_sleep_config_valid(
                  LptimTimedSleepConfig{.instance = 3}, 1000u));

using LptimSite = Stm32LptimTimedSleepSite<Stm32Platform, SysClock>;
static_assert(SleepSite<LptimSite>,
              "the third site over util/power.hpp's unchanged concept");
static_assert(LptimSite::counter_hz == 1024u);
static_assert(LptimSite::rate_hz == 32'768u);
static_assert(LptimSite::min_alarm_counts >= 1u);
static_assert(LptimSite::span_ticks > 60'000u,
              "one lap of a 1024 Hz counter is about 64 seconds");

// The same site on the OTHER instance and the OTHER oscillator, since
// both are legal and the vector differs per header.
using LptimSite2 =
    Stm32LptimTimedSleepSite<Stm32Platform, SysClock,
                             LptimTimedSleepConfig{.instance = 2,
                                                   .source = LptimClock::lsi,
                                                   .rate_hz = 33'000,
                                                   .prescaler = LptimPrescaler::div16}>;
static_assert(SleepSite<LptimSite2>);

void lptim_site_verbs() {
    (void)LptimSite::init();
    (void)LptimSite::ready();
    (void)LptimSite::arm(SleepDepth::deep);
    LptimSite::disarm();
    (void)LptimSite::armed();
    (void)LptimSite::alarm_armed();
    (void)LptimSite::last_advance();
    (void)LptimSite::last_cmp();
    (void)LptimSite::last_counts();
    (void)LptimSite::cmp_completions();
    (void)LptimSite::laps();
    (void)LptimSite::count32();
    (void)LptimSite::place_alarm(500);
    LptimSite::resync();
    LptimSite::isr();

    (void)LptimSite2::init();
    (void)LptimSite2::arm(SleepDepth::standby);
    LptimSite2::disarm();
    LptimSite2::isr();
}

// A manager over the site, which is what proves the concept fits.
struct Voter : Fsm<Voter, PrepareSleep, SleepVote, WakeReport> {
    static inline EventQueue<Event, 4, Stm32Platform> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](const PrepareSleep& p) { p.reply.send(SleepVote{true}); return handled(); },
            [](SleepVote) { return handled(); },
            [](WakeReport) { return handled(); });
    }
};
using Manager = PowerManager<Stm32Platform, TimedSite, PowerConfig{}, Voter>;
using K = Kernel<Stm32Platform, Voter, Manager>;

void kernel_over_the_site() {
    K::init_all();
    (void)K::step();
}

using LptimManager = PowerManager<Stm32Platform, LptimSite, PowerConfig{}, Voter>;
using LptimK = Kernel<Stm32Platform, Voter, LptimManager>;

void kernel_over_the_lptim_site() {
    LptimK::init_all();
    (void)LptimK::step();
}
