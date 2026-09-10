// Sleep family smoke TU: PWR's verbs, the AWU's arithmetic, the two
// sites satisfying util/power.hpp's SleepSite over a static and a
// dynamic clock, and a PowerManager over each.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/exti.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/sleep.hpp"
#include "kernel/active_object.hpp"
#include "util/power.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using Fast = Clock<ClockSource::pll, 48'000'000>;
struct Follower { static void rebase(uint32_t) {} };
using Dyn = DynamicClock<Fast, Follower>;

static_assert(awu_prescaler_divider(0) == 1 && awu_prescaler_divider(1) == 1);
static_assert(awu_prescaler_divider(2) == 2 && awu_prescaler_divider(13) == 4096);
static_assert(awu_prescaler_divider(14) == 10240 && awu_prescaler_divider(15) == 61440);
static_assert(Awu::period_us(13, 63, 128'000) == 2'048'000);   // the longest at the nominal rate
static_assert(Awu::period_us(0, 0, 128'000) == 7);
static_assert(exti_line_awu == 9 && exti_line_pvd == 8);

using Plain = Ch32SleepSite<Fast>;
using Timed = Ch32TimedSleepSite<P, Fast>;
using TimedDyn = Ch32TimedSleepSite<P, Dyn>;
static_assert(SleepSite<Plain> && SleepSite<Timed> && SleepSite<TimedDyn>);

using Manager = PowerManager<P, Timed>;
static_assert(ActiveObject<Manager>);

using Button = ExtInt<'D', 4>;

void sleep_verbs() {
    Pwr::standby(true);
    (void)Pwr::standby();
    Pwr::ldo(pwr_ldo_saving);
    (void)Pwr::ldo();
    Pwr::pvd(true, PvdLevel::v2_43);
    (void)Pwr::pvd();
    (void)Pwr::supply_low();
    Pwr::flash_low_power(true);
    (void)Awu::init(true);
    Awu::arm(13, 63);
    (void)Awu::enabled();
    (void)Awu::fired();
    Awu::clear();
    Awu::disarm();
    (void)Plain::arm(SleepDepth::deep);
    (void)Plain::armed();
    Plain::disarm();
    (void)Timed::init();
    (void)Timed::lsi_hz();
    (void)Timed::arm(SleepDepth::standby);
    Timed::disarm();
    (void)Timed::last_advance();
    (void)TimedDyn::init();
    Button::init(true, false);
    Button::interrupt(true);
    Button::soft();
    (void)Button::flag();
    Button::clear();
    Exti::port(3, 2);
    (void)Exti::flags();
    Manager::init();
}
