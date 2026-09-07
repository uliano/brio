// Family smoke TU: stm32g0/lptim_ticker.hpp - the tickless kernel
// timebase - instantiated on both LPTIMs on every G0x1 header, and on
// the x0 value line (no LPTIM) compiled to the absence check and
// nothing else. Instantiation only - no main(), no hardware.
//
// What it pins: the rate arithmetic (32768 >> prescaler, a power of
// two; 1024 at the default), the concept (an LptimTicker is Tickless
// and the SysTick Ticker is not), the platform on it (Platform, with
// idle_until, at 1024 ticks per second where the kernel's ms
// conversions stop folding to the identity), a Kernel plus a
// PowerManager over the plain site on that platform (the tickless
// branch of idle_if_empty and the pause-less site, instantiated), and
// every verb of the ticker on both instances.

#include <stdint.h>

#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/lptim_ticker.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/sleep.hpp"
#include "util/power.hpp"

using namespace brio;

#if !defined(LPTIM1_BASE)

static_assert(!lptim_present(1), "no LPTIM, no LptimTicker, on the x0 value line");

#else

using SysClock = Clock<ClockSource::pll, 64'000'000>;

// ---- the rate ----------------------------------------------------------------
static_assert(lptim_ticker_hz(LptimTickerConfig{}) == 1024u);
static_assert(lptim_ticker_hz({.shift = 0}) == 32'768u);
static_assert(lptim_ticker_hz({.shift = 10}) == 32u);
static_assert(lptim_ticker_config_valid(LptimTickerConfig{}));
static_assert(lptim_ticker_config_valid({.instance = 2}));
static_assert(!lptim_ticker_config_valid({.instance = 3}));
static_assert(!lptim_ticker_config_valid({.shift = 11}), "fewer than 32 ticks a second is refused");

// ---- the default ticker, and the concept ---------------------------------------
using Tb = LptimTicker<>;
static_assert(Tickless<Tb>, "the whole point");
static_assert(!Tickless<Ticker>, "the SysTick ticker has no wake to place");
static_assert(Tb::ticks_per_second == 1024u);
static_assert(Tb::shift == 5u && Tb::counts_per_tick == 32u);
static_assert(Tb::lap_counts == 0x10000u && Tb::lap_ticks == 2048u, "a lap is two seconds");
static_assert(Tb::min_counts_ahead == 6u);
static_assert(Tb::irq() == lptim_irq(1));

// A second one on the other instance at the crystal's own rate - the
// bench's lap witness (a lap of two seconds).
using Witness = LptimTicker<LptimTickerConfig{.instance = 2, .shift = 0}>;
static_assert(Tickless<Witness>);
static_assert(Witness::ticks_per_second == 32'768u);
static_assert(Witness::shift == 0u && Witness::lap_ticks == 0x10000u);
static_assert(Witness::irq() == lptim_irq(2));
static_assert(Tb::parked_cmp == 0xFFFFu, "parked on the lap: one wake per lap, not two");

// ---- the LSI source: a stated rate, the band, the arithmetic ------------------
using OnLsi = LptimTicker<LptimTickerConfig{
    .instance = 2, .shift = 5, .source = LptimTickerSource::lsi, .lsi_hz = 32'586}>;
static_assert(Tickless<OnLsi>);
static_assert(!OnLsi::on_crystal && Tb::on_crystal);
static_assert(OnLsi::count_hz == 32'586u && OnLsi::ticks_per_second == 1018u);
static_assert(OnLsi::lap_ticks == 2048u, "the lap is the counter's, not the clock's");
static_assert(lptim_ticker_hz({.source = LptimTickerSource::lsi}) == (34'000u >> 5),
              "the default statement is table 46's ceiling: never early");
static_assert(lptim_ticker_config_valid({.source = LptimTickerSource::lsi, .lsi_hz = 29'500}));
static_assert(!lptim_ticker_config_valid({.source = LptimTickerSource::lsi, .lsi_hz = 29'499}));
static_assert(!lptim_ticker_config_valid({.source = LptimTickerSource::lsi, .lsi_hz = 34'001}));
static_assert(lptim_ticker_config_valid({.lsi_hz = 40'000}), "lsi_hz is not read on the crystal");

// ---- the platform on it --------------------------------------------------------
using P = Stm32g0Platform<Tb>;
static_assert(Platform<P>);
static_assert(P::ticks_per_second == 1024u);
static_assert(std::same_as<P::Timebase, Tb>);
template <class Q>
constexpr bool has_idle_until = requires { Q::idle_until(std::optional<uint32_t>{}); };
static_assert(has_idle_until<P>);
static_assert(!has_idle_until<Stm32g0Platform<>>);

// The kernel's conversions at 1024 Hz: CEIL, no longer the identity.
static_assert(ticks_from_ms<P>(1000) == 1024);
static_assert(ticks_from_ms<P>(1) == 2);
static_assert(ticks_from_ms<P>(500) == 512);
static_assert(ticks_from_secs<P>(3) == 3072);

// ---- a kernel and a manager over the plain site, on the tickless platform ------
using Site = Stm32g0SleepSite<SysClock, Tb>;
static_assert(SleepSite<Site>);
static_assert(!Site::pauses_tick, "nothing to pause on this timebase");

struct Probe : Fsm<Probe, PrepareSleep, SleepVote, WakeReport> {
    static inline EventQueue<Event, 4, P> queue;
    static inline TimeEvent<P, Probe, SleepVote> alarm{SleepVote{true}};
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
using Manager = PowerManager<P, Site, PowerConfig{}, Probe>;
using K = Kernel<P, Probe, Manager>;

void tickless_kernel() {
    K::init_all();
    Probe::alarm.arm(10);
    (void)K::step();
    K::idle_if_empty();                                   // the idle_until branch
    (void)TimeEvents<P>::next_deadline();
    P::idle_until(std::nullopt);
    P::idle_until(std::optional<uint32_t>{P::now() + 5u});
}

// ---- every verb, both instances ------------------------------------------------
template <class T>
void ticker_verbs() {
    constexpr SysClock clock;
    (void)T::init(clock);
    (void)T::ticks();
    (void)T::count();
    (void)T::millis();
    (void)T::secs();
    TimeStamp stamp{};
    T::now(stamp);
    (void)T::arm_wake(T::ticks(), T::ticks() + 100u);
    (void)T::park();
    (void)T::isr();
    (void)T::laps();
    (void)T::cmp_reg();
    (void)T::write_pending();
    (void)T::deferrals();
    (void)T::floor_declines();
    (void)T::stores();
    (void)T::write_timeouts();
    (void)T::stop_waits();
    (void)OnLsi::init(clock);
    (void)OnLsi::millis();
    (void)OnLsi::secs();
    OnLsi::now(stamp);
}

void all_verbs() {
    ticker_verbs<Tb>();
    ticker_verbs<Witness>();
}

#endif // LPTIM1_BASE
