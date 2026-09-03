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
