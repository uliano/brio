// Sleep family smoke TU: the gate sets a sleeping program composes, the
// dormant-wake destination of the IO bank, and both sites of
// util/power.hpp over them - the plain ladder on either oscillator and
// the timed site with the always-on timer as alarm and witness.
#include "rp2350/clock.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/sleep.hpp"
#include "util/power.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
using P = Rp2350Platform<>;

using CrystalSite = Rp2350SleepSite<SysClock, DormantSource::xosc>;
using RingSite = Rp2350SleepSite<SysClock, DormantSource::rosc>;
using TimedSite = Rp2350TimedSleepSite<P, SysClock, DormantSource::xosc>;
using TimedRingSite = Rp2350TimedSleepSite<P, SysClock, DormantSource::rosc>;

// Both satisfy the contract the manager is written against.
static_assert(SleepSite<CrystalSite>);
static_assert(SleepSite<RingSite>);
static_assert(SleepSite<TimedSite>);
static_assert(SleepSite<TimedRingSite>);

// The sets are disjoint where they name different blocks, and the
// always-on block's pair is what an alarm needs kept.
static_assert((sleep_clocks_powman.en0 & sleep_clocks_core.en0) == 0u);
static_assert((sleep_clocks_uart0.en1 & sleep_clocks_uart1.en1) == 0u);
static_assert(sleep_clocks_powman.en1 == 0u);
static_assert(TimedSite::alarm_gates == sleep_clocks_powman);

// The two conversions, in the directions the header promises: a deadline
// rounded UP into milliseconds, a span rounded DOWN into ticks. At this
// target's 1000 Hz tick the two units are the same and the rounding is
// visible only in the arithmetic.
static_assert(TimedSite::ticks_to_ms(0u) == 0u);
static_assert(TimedSite::ticks_to_ms(1u) == 1u);
static_assert(TimedSite::ticks_to_ms(300u) == 300u);
static_assert(TimedSite::ms_to_ticks(999u) == 999u);

// The manager over the timed site: the composition an application writes.
using Power = PowerManager<P, TimedSite, PowerConfig{}>;

void sleep_gate_sets() {
    Clocks::sleep_enables(sleep_clocks_core | sleep_clocks_powman | sleep_clocks_timer |
                          sleep_clocks_uart0 | sleep_clocks_uart1 | sleep_clocks_dma |
                          sleep_clocks_pwm | sleep_clocks_watchdog);
    Clocks::sleep_enables(sleep_clocks_all);
    Clocks::sleep_enables(sleep_clocks_none);
    (void)Clocks::sleep_enables();
    (void)Clocks::wake_enables();
    (void)Clocks::enabled();
}

void sleep_dormant_wake() {
    (void)DormantWake::enable(19u, pin_events(PinEvent::level_high));
    (void)DormantWake::enable(8u, pin_edges);
    DormantWake::disable(19u, pin_events(PinEvent::level_high));
    (void)DormantWake::enabled(8u);
    (void)DormantWake::pending(8u);
    (void)DormantWake::raw(8u);
    DormantWake::acknowledge(8u, pin_edges);
    (void)DormantWake::any_enabled();
    DormantWake::disable_all();
    static_assert(DormantWake::target == PinIrqTarget::dormant_wake);
}

void sleep_plain_site() {
    CrystalSite::standby_clocks(sleep_clocks_core | sleep_clocks_powman);
    (void)CrystalSite::standby_clocks();
    (void)CrystalSite::arm(SleepDepth::none);
    (void)CrystalSite::arm(SleepDepth::light);
    (void)CrystalSite::arm(SleepDepth::standby);
    (void)CrystalSite::arm(SleepDepth::deep);
    (void)CrystalSite::armed();
    (void)CrystalSite::dormant_wake_ready();
    (void)CrystalSite::dormants();
    CrystalSite::disarm();

    (void)RingSite::arm(SleepDepth::deep);
    (void)RingSite::dormant_wake_ready();
    RingSite::disarm();
}

/// The hook the idle path takes instead of a sleep instruction, named
/// here so the linker keeps it: nothing calls it in a compile.
void sleep_dormant_entry() {
    CrystalSite::go_dormant();
    RingSite::go_dormant();
}

void sleep_timed_site() {
    (void)TimedSite::init();
    TimedSite::standby_clocks(sleep_clocks_core | sleep_clocks_timer | sleep_clocks_uart0);
    (void)TimedSite::standby_clocks();
    (void)TimedSite::arm(SleepDepth::light);
    (void)TimedSite::arm(SleepDepth::standby);
    (void)TimedSite::arm(SleepDepth::deep);
    (void)TimedSite::armed();
    TimedSite::resync();
    (void)TimedSite::isr();
    (void)TimedSite::ready();
    (void)TimedSite::alarm_armed();
    (void)TimedSite::last_advance();
    TimedSite::disarm();

    (void)TimedRingSite::init();
    (void)TimedRingSite::arm(SleepDepth::deep);
    TimedRingSite::disarm();
}

void sleep_manager() {
    Power::init();
    (void)Power::ceiling();
    (void)Power::armed_depth();
    PowerLock lock = Power::restrict(SleepDepth::light);
    lock.release();
    Power::dispatch(Power::Event{SleepRequested{SleepDepth::standby, {}}});
}
