// Power family smoke TU: the whole of the PWR chapter (stm32f4/pwr.hpp),
// the two util/power.hpp sites over it (stm32f4/sleep.hpp) and the
// DYNAMIC CLOCK the deep rungs make necessary (stm32f4/clock.hpp's
// Rates<> + DynamicClock), on every header of the pack.
//
// What differs across the family here: one VOS bit or two, the over-drive
// pair, the under-drive field, the low-voltage-in-deep-sleep pair, the
// FISSR/FMSSR pair, ADCDC1, the backup SRAM behind BRE, how many WKUPx
// pins are bonded and which pads they are, and the PVD's thresholds in
// volts. Every one of them is the reserve's, so the verbs below compile
// unchanged everywhere and answer false where the silicon has nothing.
#include "stm32f4/clock.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/sleep.hpp"

using namespace brio;

// ---- the reserve's own facts, header-independent -------------------------------

// Pin 1 exists on every part of the pack, under one spelling or the other.
static_assert(pwr_wakeup_pin_mask(1) != 0u);
static_assert(pwr_wakeup_pin_count() >= 1u && pwr_wakeup_pin_count() <= 3u);
static_assert(pwr_wakeup_pin_mask(0) == 0u && pwr_wakeup_pin_mask(4) == 0u);
// WKUP1 is PA0 on every datasheet read.
static_assert(pwr_wakeup_pad(1).known && pwr_wakeup_pad(1).port == 'A' &&
              pwr_wakeup_pad(1).pin == 0);
static_assert(!pwr_wakeup_pad(0).known);
// A pin the part has not got is never a pad.
static_assert(pwr_wakeup_pin_count() >= 2u || !pwr_wakeup_pad(2).known);
// The under-drive field and the over-drive pair are the two halves of one
// option: no header of this pack has one without the other.
static_assert(pwr_has_under_drive() == pwr_has_over_drive());
// The PVD's line is the same on every part.
static_assert(pwr_pvd_exti_line == 16);
// The backup SRAM's gate and the classes that carry it.
static_assert(Pwr::has_backup_sram == (pwr_backup_sram_clock_mask() != 0u));

// The thresholds, where a manual was read: the top code is 2.9 V on both
// tables, and the bottom differs.
static_assert(!pwr_pvd_levels().known || pwr_pvd_levels().mv[7] == 2900);
static_assert(!pwr_pvd_levels().known || pwr_pvd_levels().mv[0] == 2000 ||
              pwr_pvd_levels().mv[0] == 2200);
static_assert(Pwr::pvd_levels_known == pwr_pvd_levels().known);
static_assert(Pwr::pvd_level_mv(8) == 0);

// ---- the mode vocabulary --------------------------------------------------------
static_assert(!pwr_mode_resets(PwrMode::sleep) && !pwr_mode_resets(PwrMode::stop) &&
              pwr_mode_resets(PwrMode::standby));
static_assert(!pwr_mode_stops_clocks(PwrMode::sleep) && pwr_mode_stops_clocks(PwrMode::stop) &&
              pwr_mode_stops_clocks(PwrMode::standby));

// The default Stop is legal on every part; the low-voltage and
// under-drive variants only where the part has the bits.
static_assert(stop_config_valid(StopConfig{}));
static_assert(stop_config_valid(StopConfig{StopRegulator::low_power, true, false, false}));
static_assert(stop_config_valid(StopConfig{StopRegulator::main, false, true, false}) ==
              pwr_has_low_voltage_stop());
static_assert(stop_config_valid(StopConfig{StopRegulator::main, true, true, true}) ==
              pwr_has_under_drive());
// Under-drive is a MODIFIER of the low-voltage mode and never a mode of
// its own - refused even where the field exists.
static_assert(!stop_config_valid(StopConfig{StopRegulator::main, true, false, true}));

// ---- the sites' configuration ---------------------------------------------------
static_assert(sleep_site_config_valid(SleepSiteConfig{}));
static_assert(timed_sleep_config_valid(TimedSleepConfig{}));
static_assert(!timed_sleep_config_valid(TimedSleepConfig{.rtcclk_hz = 512}));
static_assert(!timed_sleep_config_valid(TimedSleepConfig{.fast_clock = RtcWakeupClock::ck_spre}));
// 32768 Hz split the stopwatch way: PREDIV_A 0, PREDIV_S 32767.
static_assert(rtc_prescalers_for_resolution(32'768u).async == 0 &&
              rtc_prescalers_for_resolution(32'768u).sync == 32'767u);

// ---- the APB rule, one statement --------------------------------------------------
static_assert(apb_divider_at(16'000'000u, false) == 1 && apb_divider_at(16'000'000u, true) == 1);
static_assert(apb_hz_at(16'000'000u, false) == 16'000'000u);
static_assert(apb_divider_at(16'000'000u, false) ==
              Clock<ClockSource::hsi, 16'000'000>::apb1_div);

// ---- the sites and the dynamic clock, on the reset rate ---------------------------
//
// Every header runs the 16 MHz HSI with no ladder read, so the SITES are
// instantiated on it unconditionally; the PLL rates and the ladder they
// need are behind the template below.

using P = Stm32f4Platform<>;
using Boot = Clock<ClockSource::hsi, 16'000'000>;
using Site = Stm32f4SleepSite<Boot>;
using TimedSite = Stm32f4TimedSleepSite<P, Boot>;

static_assert(SleepSite<Site>);
static_assert(SleepSite<TimedSite>);
static_assert(Site::pauses_tick);

using BootRates = Rates<Boot>;
using BootDynamic = DynamicClock<BootRates, Ticker>;
static_assert(!BootDynamic::is_static && BootDynamic::rate_count == 1);
static_assert(BootDynamic::rate_hz(0) == 16'000'000u);
static_assert(BootDynamic::can_run_at(16'000'000u) && !BootDynamic::can_run_at(48'000'000u));
static_assert(BootDynamic::index_of(48'000'000u) == BootDynamic::rate_count);
static_assert(BootDynamic::rate_source(0) == SysclkSource::hsi);
static_assert(BootDynamic::rate_pclk1_hz(0) == 16'000'000u);
static_assert(BootDynamic::rate_wait_states(0) == 0);
static_assert(BootDynamic::rebases<Ticker> && !BootDynamic::rebases<Boot>);
static_assert(clock_follows<BootDynamic, Ticker>());
static_assert(!clock_follows<BootDynamic, Boot>());

/// The ladder-dependent half: a two-rate pack from the HSI's PLL, and the
/// sites over it. Behind a template parameter because a discarded branch
/// of a non-template is still checked.
template <bool known = sysclk_ladder().known>
void ladder_dynamic() {
    if constexpr (known) {
        using Fast = Clock<ClockSource::pll_hsi, known ? 84'000'000 : 16'000'000>;
        using Ladder = DynamicClock<Rates<Fast, Boot>, Ticker>;
        static_assert(Ladder::rate_count == 2);
        static_assert(Ladder::rate_source(0) == SysclkSource::pll);
        static_assert(Ladder::rate_source(1) == SysclkSource::hsi);
        static_assert(Ladder::index_of(16'000'000u) == 1);
        (void)Ladder::init();
        (void)Ladder::template set<16'000'000u>();
        (void)Ladder::set(84'000'000u);
        (void)Ladder::template set_index<0>();
        (void)Ladder::set_index(1);
        (void)Ladder::restore();
        (void)Ladder::switching();
        (void)Ladder::hz();
        (void)Ladder::pclk1_hz();
        (void)Ladder::pclk2_hz();
        (void)Ladder::usb_hz();
        (void)Ladder::rate_index();
        (void)Ladder::regime();

        using LadderSite = Stm32f4SleepSite<Ladder>;
        using LadderTimed = Stm32f4TimedSleepSite<P, Ladder>;
        static_assert(SleepSite<LadderSite>);
        static_assert(SleepSite<LadderTimed>);
        (void)LadderSite::arm(SleepDepth::deep);
        LadderSite::disarm();
        (void)LadderTimed::init();
        (void)LadderTimed::arm(SleepDepth::standby);
        LadderTimed::disarm();
        LadderTimed::isr();
    }
}

/// The Stop flavours a part with the under-drive field can name, again
/// behind a parameter: the config is refused at compile time where the
/// bits do not exist, which is what neg/ proves.
template <bool deep_bits = pwr_has_under_drive()>
void under_drive_site() {
    // The two flags DEPEND on the parameter, or the type below would be
    // instantiated in the discarded branch too and its static_assert would
    // fire on every part without the field.
    constexpr SleepSiteConfig cfg{
        .standby = StopConfig{StopRegulator::main, false, false, false},
        .deep = StopConfig{StopRegulator::low_power, true, deep_bits, deep_bits},
    };
    if constexpr (deep_bits) {
        using Deepest = Stm32f4SleepSite<Boot, Ticker, cfg>;
        static_assert(SleepSite<Deepest>);
        (void)Deepest::arm(SleepDepth::deep);
        Deepest::disarm();
        (void)Deepest::armed();
        static_assert(Deepest::config.deep.under_drive);
    }
}

// ---- every verb of the resource ---------------------------------------------------

void pwr_verbs() {
    Pwr::bus_clock(true);
    (void)Pwr::bus_clock();
    (void)Pwr::cr();
    (void)Pwr::csr();

    // the regulator (the clock chapter's half, exercised again here)
    (void)Pwr::scale(VoltageScale::scale2);
    (void)Pwr::scale();
    (void)Pwr::scale_ready();
    (void)Pwr::scale_exists(VoltageScale::scale3);
    (void)Pwr::has_over_drive();
    (void)Pwr::over_drive_enter();
    (void)Pwr::over_drive_ready();
    (void)Pwr::over_drive_active();
    (void)Pwr::over_drive_exit();

    // the mode
    Pwr::deep_sleep(false);
    (void)Pwr::deep_sleep();
    Pwr::sleep_on_exit(false);
    (void)Pwr::sleep_on_exit();
    Pwr::event_on_pending(false);
    (void)Pwr::event_on_pending();
    Pwr::power_down_deep_sleep(false);
    (void)Pwr::power_down_deep_sleep();
    (void)Pwr::stop_config(StopConfig{});
    (void)Pwr::stop_config();
    (void)Pwr::arm(PwrMode::sleep);
    (void)Pwr::arm(PwrMode::stop, StopConfig{});
    (void)Pwr::mode();
    Pwr::wait_for_interrupt();
    Pwr::wait_for_event();
    Pwr::enter(PwrMode::sleep);
    Pwr::enter(PwrMode::sleep, true);

    // the wake-up pins
    (void)Pwr::wakeup_pin_count;
    (void)Pwr::wakeup_pin_present(1);
    (void)Pwr::wakeup_pad(1);
    (void)Pwr::wakeup_pin(1, true);
    (void)Pwr::wakeup_pin_enabled(1);
    (void)Pwr::wakeup_pin(2, false);
    (void)Pwr::wakeup_pin(3, false);
    Pwr::wakeup_pins(false);

    // the flags
    (void)Pwr::wakeup_flag();
    Pwr::clear_wakeup_flag();
    (void)Pwr::standby_flag();
    Pwr::clear_standby_flag();
    Pwr::clear_wakeup_flags();
    Pwr::prepare_standby();

    // the detector
    (void)Pwr::pvd_level(3);
    (void)Pwr::pvd_level();
    (void)Pwr::pvd_level_mv(3);
    Pwr::pvd_enable(false);
    (void)Pwr::pvd_enabled();
    (void)Pwr::pvd_below();
    (void)Pwr::pvd_exti_line;

    // the backup regulator and its array
    (void)Pwr::backup_sram_clock(true);
    (void)Pwr::backup_sram_clock();
    (void)Pwr::backup_regulator(false);
    (void)Pwr::backup_regulator();
    (void)Pwr::backup_regulator_ready();

    // under-drive, the flash-while-running pair and AN4073's bit
    (void)Pwr::has_under_drive;
    (void)Pwr::under_drive_flag();
    Pwr::clear_under_drive_flag();
    (void)Pwr::has_flash_stop_while_run;
    (void)Pwr::flash_interface_stop_in_run(false);
    (void)Pwr::flash_interface_stop_in_run();
    (void)Pwr::flash_stop_in_run(false);
    (void)Pwr::flash_stop_in_run();
    (void)Pwr::has_adc_dc1;
    (void)Pwr::adc_dc1(false);
    (void)Pwr::adc_dc1();

    // the debugger's three bits
    (void)Pwr::debug_in_sleep();
    Pwr::debug_in_sleep(false);
    (void)Pwr::debug_in_stop();
    Pwr::debug_in_stop(false);
    (void)Pwr::debug_in_standby();
    Pwr::debug_in_standby(false);
    Pwr::debug_low_power(false);
}

void site_verbs() {
    (void)Site::arm(SleepDepth::none);
    (void)Site::arm(SleepDepth::light);
    (void)Site::arm(SleepDepth::standby);
    (void)Site::arm(SleepDepth::deep);
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
    (void)TimedSite::place_alarm(100);
    (void)TimedSite::time_of_hour_ms();
    (void)TimedSite::elapsed_ms(0, 1);
    TimedSite::resync();
    TimedSite::isr();

    (void)BootDynamic::init();
    (void)BootDynamic::set<16'000'000u>();
    (void)BootDynamic::restore();

    ladder_dynamic();
    under_drive_site();
}
