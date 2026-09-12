// Sleep family smoke TU: the dormant-wake logic, the two sites over
// util/power.hpp's contract, the clock gates, the manager over the
// timed site.
#include "kernel/kernel.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/sleep.hpp"
#include "util/power.hpp"

using namespace brio;

using P = Rp2040Platform<>;
using SysClock = Clock<ClockSource::pll, 125'000'000>;
using Plain = Rp2040SleepSite<SysClock>;
using PlainRosc = Rp2040SleepSite<SysClock, DormantSource::rosc>;
using Timed = Rp2040TimedSleepSite<P, SysClock>;
using TimedRosc = Rp2040TimedSleepSite<P, SysClock, DormantSource::rosc, 2>;

static_assert(SleepSite<Plain> && SleepSite<PlainRosc> && SleepSite<Timed> && SleepSite<TimedRosc>);
static_assert((sleep_clocks_core | sleep_clocks_uart0).en1 & CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS);
static_assert((sleep_clocks_all & ~sleep_clocks_pwm).en0 == ~CLOCKS_SLEEP_EN0_CLK_SYS_PWM_BITS);
static_assert(sleep_clocks_none.en1 == 0u && sleep_clocks_all.en1 == CLOCKS_SLEEP_EN1_BITS);
static_assert((DormantWakeEvent::edge_low | DormantWakeEvent::level_high) == 0x06u);

struct Voter : Fsm<Voter, SleepVote, PrepareSleep, WakeReport> {
    static inline EventQueue<Event, 4, P> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e, [](Entry) { return handled(); }, [](Exit) { return handled(); },
                     [](SleepVote) { return handled(); },
                     [](const PrepareSleep& p) { p.reply.send(SleepVote{true}); return handled(); },
                     [](WakeReport) { return handled(); });
    }
};
using Manager = PowerManager<P, Timed, PowerConfig{}, Voter>;
using K = Kernel<P, Voter, Manager>;

void sleep_verbs() {
    (void)DormantWake::enable(1, DormantWakeEvent::edge_low | DormantWakeEvent::level_low, true);
    (void)DormantWake::enabled(1);
    (void)DormantWake::raised(1);
    (void)DormantWake::pending(1);
    DormantWake::acknowledge(1, dormant_wake_bits(DormantWakeEvent::edge_low));
    (void)DormantWake::any_enabled();
    DormantWake::disable_all();

    (void)Plain::arm(SleepDepth::light);
    (void)Plain::arm(SleepDepth::standby);
    (void)Plain::arm(SleepDepth::deep);
    (void)Plain::armed();
    (void)Plain::dormant_wake_ready();
    (void)Plain::dormants();
    Plain::disarm();
    (void)PlainRosc::arm(SleepDepth::deep);
    PlainRosc::disarm();

    (void)Timed::init();
    (void)Timed::arm(SleepDepth::standby);
    (void)Timed::arm(SleepDepth::deep);
    (void)Timed::armed();
    Timed::resync();
    Timed::isr();
    (void)Timed::rtc_isr();
    (void)Timed::ready();
    (void)Timed::alarm_armed();
    (void)Timed::rtc_alarm_armed();
    (void)Timed::last_advance();
    Timed::disarm();
    (void)TimedRosc::init();
    (void)TimedRosc::arm(SleepDepth::deep);
    TimedRosc::disarm();

    Clocks::sleep_enables(sleep_clocks_core | sleep_clocks_uart0 | Timed::timer_gates);
    (void)Clocks::sleep_enables();
    Clocks::wake_enables(sleep_clocks_all);
    (void)Clocks::wake_enables();
    (void)Clocks::enabled();
    Xosc::dormant();
    Rosc::dormant();
    P::sleep_hook = nullptr;

    K::init_all();
    K::idle_if_empty();
}
