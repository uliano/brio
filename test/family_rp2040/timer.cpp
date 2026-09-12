// Timer + watchdog + reset family smoke TU: every verb instantiated.
#include "rp2040/clock.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/reset.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/watchdog.hpp"

using namespace brio;
using SysClock = Clock<ClockSource::pll, 125'000'000>;

static_assert(Timer::alarm_count == 4);
static_assert(Timer::irq(3) == TIMER_IRQ_3_IRQn);
static_assert(Watchdog::max_timeout_us == 0x7fffffu);
static_assert(ResetCause::all == 0x1fu);

void timer_verbs() {
    constexpr SysClock clock;
    (void)Timer::init(clock);
    (void)Timer::now();
    (void)Timer::now_low();
    (void)Timer::now_latched();
    Timer::alarm(0, 1000u);
    Timer::alarm_in(1, 500u);
    (void)Timer::armed(0);
    Timer::disarm(0);
    Timer::interrupt(2, true);
    (void)Timer::raised(2);
    (void)Timer::pending(2);
    Timer::clear(2);
    Timer::force(3, true);
    Timer::force(3, false);
    Timer::pause(true);
    (void)Timer::paused();
    Timer::debug_pause(false, false);
    (void)WatchdogTick::start(12);
    (void)WatchdogTick::running();
    (void)WatchdogTick::cycles();
    (void)WatchdogTick::count();
    WatchdogTick::stop();
    Watchdog::start(100'000u);
    Watchdog::kick();
    (void)Watchdog::running();
    (void)Watchdog::remaining_us();
    (void)Watchdog::reason();
    Watchdog::stop();
    Scratch<0>::write(1u);
    (void)Scratch<3>::read();
    (void)Reset::causes();
}
[[noreturn]] void reset_verbs(bool which) {
    if (which) { Watchdog::force_reset(); }
    Reset::software();
    Reset::core();
}
[[noreturn]] void fault_body() { fault_reset<Rp2040Platform<>>(0x77); }
[[noreturn]] void panic_body() { panic<Rp2040Platform<>, ResetReporter>(PanicCode::assert_failed, 1); }
