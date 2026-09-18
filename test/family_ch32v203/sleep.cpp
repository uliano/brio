// Sleep family smoke TU: the two sites util/power.hpp asks for - the
// plain one that maps the depth ladder onto this family's modes, and
// the timed one that places the RTC's alarm where the deadline is and
// hands the frozen span back to the ticker - plus the count of active
// bus masters both of them and the platform's idle path consult.
//
// The arithmetic is checked where it is decided: the alarm's counts are
// rounded UP and the witness's ticks DOWN, which is the one direction
// that makes a wake late and a resync short rather than either of them
// early. The configuration's default is the crystal over 32, a tick of
// 1024 Hz, and the static_asserts below are what a program gets when it
// asks for something else.
#include <concepts>

#include "ch32v203/bus_activity.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/sleep.hpp"

using namespace brio;

using Boot = Clock<ClockSource::internal, 8'000'000>;
using Fast = Clock<ClockSource::pll, 96'000'000>;
using Dyn = DynamicClock<Rates<Boot, Fast>>;
using P = Ch32v203Platform<>;

using Site = Ch32v203SleepSite<Fast>;
using DynSite = Ch32v203SleepSite<Dyn>;
using Timed = Ch32v203TimedSleepSite<P, Fast>;

// ---- both sites are what the model asks for --------------------------------
static_assert(SleepSite<Site>);
static_assert(SleepSite<DynSite>);
static_assert(SleepSite<Timed>);

// ---- the two Stop flavours -------------------------------------------------
static_assert(sleep_site_config_valid(SleepSiteConfig{}));
static_assert(Site::config.standby.regulator == StopRegulator::main);
static_assert(Site::config.deep.regulator == StopRegulator::low_power);
// The default asks for neither RAM low-voltage mode: what it costs the
// retention a Stop is chosen for is nowhere in the chapter.
static_assert(!Site::config.standby.ram_low_voltage);
static_assert(!Site::config.deep.ram_low_voltage);
static_assert(!sleep_site_config_valid(
    SleepSiteConfig{StopConfig{StopRegulator::main, true}, StopConfig{}}));

// ---- the timed site's arithmetic -------------------------------------------
// The default: the crystal over 32, one count every 976 us.
static_assert(timed_sleep_divider(TimedSleepConfig{}) == 32);
static_assert(timed_sleep_tr_hz(TimedSleepConfig{}) == 1024);
static_assert(Timed::prescaler == 31);
static_assert(Timed::tr_hz == 1024);
static_assert(timed_sleep_config_valid(TimedSleepConfig{}));

// A rate that does not divide whole still answers an UPPER bound on the
// tick it makes, which is the direction both halves of the site want:
// 39 kHz over 38 is 1027 Hz stated against 1026.3 real.
static_assert(timed_sleep_divider(TimedSleepConfig{.rtcclk_hz = 39'000}) == 38);
static_assert(timed_sleep_tr_hz(TimedSleepConfig{.rtcclk_hz = 39'000}) == 1027);

// What the config refuses: a source that is not one, a tick faster than
// the clock, and a division the twenty-bit prescaler cannot hold.
static_assert(!timed_sleep_config_valid(
    TimedSleepConfig{.source = RtcClockSource::none}));
static_assert(!timed_sleep_config_valid(
    TimedSleepConfig{.rtcclk_hz = 1024, .tick_hz = 32'768}));
static_assert(!timed_sleep_config_valid(TimedSleepConfig{.rtcclk_hz = 0}));
static_assert(!timed_sleep_config_valid(TimedSleepConfig{.tick_hz = 0}));

// The alarm's counts round UP and are floored, so a deadline is never
// placed nearer than the write can land; the witness's ticks round DOWN.
static_assert(Timed::counts_for(1000) == 1024);
static_assert(Timed::counts_for(1) == Timed::alarm_floor_counts);
static_assert(Timed::counts_for(0) == Timed::alarm_floor_counts);
static_assert(Timed::counts_for(3) == 4);
static_assert(Timed::counts_for(500) == 512);
static_assert(Timed::ticks_for(1024) == 1000);
static_assert(Timed::ticks_for(1) == 0);
static_assert(Timed::ticks_for(Timed::counts_for(500)) >= 499);

// Never early, at every deadline the guard lets through: the counts
// placed are worth at least the ticks asked for.
static_assert(Timed::ticks_for(Timed::counts_for(2)) >= 2);
static_assert(Timed::ticks_for(Timed::counts_for(37)) >= 37);
static_assert(Timed::ticks_for(Timed::counts_for(60'000)) >= 60'000);

// ---- the count of bus masters ----------------------------------------------
static_assert(requires { { BusActivity::active() } -> std::same_as<uint8_t>; });
static_assert(requires { { P::bus_masters_active() } -> std::same_as<uint8_t>; });

// ---- every verb ------------------------------------------------------------
void sleep_verbs() {
    (void)Site::arm(SleepDepth::none);
    (void)Site::arm(SleepDepth::light);
    (void)Site::arm(SleepDepth::standby);
    (void)Site::arm(SleepDepth::deep);
    (void)Site::armed();
    (void)Site::resume_clock();
    Site::disarm();

    (void)DynSite::resume_clock();
    (void)DynSite::arm(SleepDepth::deep);
    DynSite::disarm();

    (void)Timed::init();
    (void)Timed::ready();
    (void)Timed::arm(SleepDepth::deep);
    (void)Timed::armed();
    (void)Timed::alarm_armed();
    (void)Timed::place_alarm(500);
    Timed::resync();
    Timed::disarm();
    (void)Timed::last_advance();
    (void)Timed::last_counts();

    BusActivity::entered();
    (void)BusActivity::active();
    BusActivity::left();
    (void)P::bus_masters_active();
}

/// The door the ladder has not, kept out of the verb sweep because it
/// is not meant to come back.
void sleep_standby() { Site::enter_standby(); }

/// The RTC alarm's own vector, as an application binds it: the four
/// acts, in one call.
extern "C" BRIO_CH32_INTERRUPT void rtc_alarm_handler() { Timed::isr(); }
