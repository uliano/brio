// test_avr_power - the POWER-MANAGEMENT test SUITE for the AVR DA/DB
// target: util/power.hpp (the depth ladder, the vote round, the standing
// restrictions, the deadline guard and the first-event-after-wake
// contract), avrdx/sleep.hpp's AvrSleepSite that realizes it on SLPCTRL,
// and the kernel half that makes the whole thing work with no new hook -
// AvrPlatform::idle() taking an already-armed deeper mode instead of
// imposing IDLE, and TimeEvents::ticks_to_next().
//
// Reference test of those (docs/design/power.md, docs/avrdx/platform.md):
// keep it passing.
//
// NOTHING TO WIRE. The instrument is the chip: a TCB pair cascaded at
// CLK_PER is the 32-bit stopwatch (RUNSTDBY on both halves, so it counts
// through the sleep it is timing), the PIT is both the kernel's 1024 Hz
// timebase and the wake source, and a fake bus engine gives the arbiter
// something to be busy with.
//
// UNLIKE EVERY OTHER SUITE HERE, THIS ONE RUNS THE KERNEL. The object
// under test is an active object, so the rounds go through real queues,
// real dispatch and the real Kernel pack (Probe, Bus, Pm) - only the
// LOOP is the suite's: each test pumps Kernel::step() itself, and where
// a sleep is the point it calls Kernel::idle_if_empty(), which is the
// same hook run() would call. The console loop between tests is the
// usual polled one and never sleeps, so a mode left armed by a test is
// inert until the next quiesce() clears it.
//
// PINS IT CLAIMS: none but the console (USART2 ALT1, PF4/PF5, 460800).
//
// Commands: ? | a the ladder and what it arms | b a real standby round
// | c the deadline guard | d the voters | e the standing restrictions
// | z = a..e

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "avrdx/rtc.hpp"
#include "avrdx/sleep.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "kernel/kernel.hpp"
#include "util/bus_master.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = AvrPlatform;
using SysClock = Clock<ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

// ---- the instruments ------------------------------------------------------------
// The 32-bit CLK_PER stopwatch. RUNSTDBY on both halves is what lets it
// TIME a standby sleep: the peripheral's own flag revives its whole
// clock chain (docs/avrdx/platform.md), so the counter keeps running
// while the CPU is stopped. That also means this sleep keeps the
// oscillator alive - it is a measurement of the STOP, not of the lowest
// current the part can reach.
using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;

constexpr uint32_t crystal_hz = SysClock::hz;
/// CLK_PER cycles in one PIT tick at 24 MHz (24e6 / 1024).
constexpr uint32_t cycles_per_tick = crystal_hz / Ticker::ticks_per_second;

// ---- shared with the ISRs ---------------------------------------------------------
volatile bool pit_ran = false;
volatile uint32_t pit_stamp = 0;
volatile uint16_t pit_irqs = 0;

uint32_t cycles_now() { return Watch::read(); }

// ---- the system under test --------------------------------------------------------

/// A bus engine that never touches a wire. What test d exercises is the
/// ARBITER above it - the BusMaster, which is the voter - so start()
/// always goes asynchronous and the transfer stays in flight until the
/// test posts its TransferDone. That is exactly the state a deep sleep
/// must not be allowed into: the completion interrupt is what a gated
/// clock domain would swallow.
struct FakeBus {
    struct Request {
        uint8_t tag;
        ReplyTo<BusDone> reply;
    };
    static inline uint8_t started = 0;
    static bool start(const Request&) {
        ++started;
        return false;
    }
};

using Bus = BusMaster<FakeBus, P>;

/// The payload of the time event the deadline guard looks at.
struct Blip {};

/// The other stakeholder, and the one that asks: it requests the sleeps,
/// hears the votes come back, votes itself, listens for the WakeReport
/// and owns the time event test c arms. One AO in both roles is the
/// normal shape - a stakeholder usually has an opinion AND a schedule.
struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline bool accept = true;          ///< how this voter votes
    static inline uint8_t votes = 0;           ///< replies to our requests
    static inline bool last_ok = false;        ///< the last one's verdict
    static inline uint8_t asked = 0;           ///< PrepareSleeps received
    static inline SleepDepth asked_depth = SleepDepth::none;
    static inline uint8_t wakes = 0;           ///< WakeReports received
    static inline SleepDepth woke_from = SleepDepth::none;
    static inline uint16_t blips = 0;

    static void init() { start(&only); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](SleepVote v) {
                ++votes;
                last_ok = v.ok;
                return handled();
            },
            [](const PrepareSleep& p) {
                ++asked;
                asked_depth = p.depth;
                p.reply.send(SleepVote{accept});
                return handled();
            },
            [](WakeReport w) {
                ++wakes;
                woke_from = w.was;
                return handled();
            },
            [](Blip) {
                ++blips;
                return handled();
            });
    }
};

using Pm = PowerManager<P, AvrSleepSite, PowerConfig{}, Bus, Probe>;
using K = Kernel<P, Probe, Bus, Pm>;

// ---- the test harness -------------------------------------------------------------
TestBench<Serial> bench;

void verdict(const char* name, bool ok) { bench.verdict(name, ok); }

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);            // the shift register, generously
}

/// The kernel loop, minus the sleep: serve matured time events and every
/// queued event until the system is quiet. Bounded, so a test that has
/// built a self-feeding loop reports it as a failed verdict instead of
/// hanging the bench.
void pump() {
    for (uint16_t i = 0; i < 500; ++i) {
        TimeEvents<P>::process();
        if (!K::step()) {
            return;
        }
    }
}

/// Post a request through the kernel and let the round run to its end.
void ask(SleepDepth d) {
    post<Pm>(SleepRequested{d, reply_to<Probe, SleepVote>()});
    pump();
}

/// The wake acknowledgement: "awake, no new request". Any event would
/// disarm the site; this is the one an app's wake path posts when it has
/// nothing else to say.
void release_sleep() { ask(SleepDepth::none); }

/// (Re)build the stopwatch with RUNSTDBY on both halves. The cascade
/// task does not expose the flag, so the two configurations are written
/// through the resource's own config struct - the same two the task
/// writes, plus run_standby.
void watch_init() {
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});
    WatchHi::init({.mode = TcbMode::capture, .clock = TcbClock::event, .compare = 0,
                   .event_input = true, .cascade = true, .run_standby = true});
    WatchLo::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0,
                   .event_input = true, .run_standby = true});
    Watch::reset();
}

/// The state every letter starts from: the site disarmed, every queue
/// drained, every AO re-init'ed, the stopwatch rebuilt, no time event
/// armed and no restriction standing.
void quiesce() {
    cli();
    Probe::deadline.disarm();
    TimeEvents<P>::clear_all();
    while (Probe::queue.pop().has_value()) {
    }
    while (Bus::queue.pop().has_value()) {
    }
    while (Pm::queue.pop().has_value()) {
    }
    Probe::accept = true;
    Probe::votes = Probe::asked = Probe::wakes = 0;
    Probe::last_ok = false;
    Probe::asked_depth = SleepDepth::none;
    Probe::woke_from = SleepDepth::none;
    Probe::blips = 0;
    FakeBus::started = 0;
    pit_irqs = 0;
    pit_ran = false;
    sei();
    K::init_all();                 // Pm::init() disarms the site
    watch_init();
}

const char* depth_name(SleepDepth d) {
    switch (d) {
    case SleepDepth::none: return "none";
    case SleepDepth::light: return "light";
    case SleepDepth::standby: return "standby";
    case SleepDepth::deep: return "deep";
    }
    return "?";
}

// ---- a: the ladder and what it arms ------------------------------------------------

void ta_ladder() {
    quiesce();

    struct Leg {
        const char* what;
        SleepDepth asked;
        uint8_t ctrla;
        SleepDepth site;
        SleepDepth manager;
        bool ok;
    };
    Leg legs[4];
    const SleepDepth ladder[4] = {SleepDepth::none, SleepDepth::light,
                                  SleepDepth::standby, SleepDepth::deep};
    for (uint8_t i = 0; i < 4; ++i) {
        Probe::last_ok = false;
        ask(ladder[i]);
        // Read the register back at the moment of the measurement: a
        // store that did not happen shows up in the table instead of
        // being assumed.
        legs[i] = Leg{depth_name(ladder[i]), ladder[i], SLPCTRL.CTRLA,
                      AvrSleepSite::armed(), Pm::armed_depth(), Probe::last_ok};
        release_sleep();
    }

    for (const Leg& l : legs) {
        print(serial, "  ", l.what, ": SLPCTRL.CTRLA=", hex(l.ctrla), " site=",
              depth_name(l.site), " manager=", depth_name(l.manager),
              l.ok ? " (accepted)" : " (refused)", crlf);
    }

    verdict("none arms nothing and still replies ok",
            legs[0].ctrla == 0 && legs[0].site == SleepDepth::none &&
            legs[0].manager == SleepDepth::none && legs[0].ok);
    verdict("light is SMODE=IDLE with SEN set",
            legs[1].ctrla == (SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm) && legs[1].ok);
    verdict("standby is SMODE=STDBY with SEN set",
            legs[2].ctrla == (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm) && legs[2].ok);
    verdict("deep is SMODE=PDOWN with SEN set",
            legs[3].ctrla == (SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm) && legs[3].ok);

    // The model lets a target map a rung it lacks to the nearest
    // SHALLOWER one. This family has all four, so the mapping is the
    // identity here and the site reads back exactly what was asked - the
    // rule earns its keep on a target with fewer modes (host suite).
    bool identity = true;
    for (const Leg& l : legs) {
        identity = identity && l.site == l.asked && l.manager == l.asked;
    }
    verdict("this target realizes every rung: the mapping is the identity", identity);

    verdict("and the platform agrees the site is disarmed again",
            !P::sleep_armed() && AvrSleepSite::armed() == SleepDepth::none);

    quiesce();
}

// ---- b: a real standby round -------------------------------------------------------

/// t0 -> the next PIT interrupt's own stamp, best of eight, starting
/// right after a tick edge so both legs measure a whole period. `stop`
/// takes the kernel's idle hook (which takes whatever is ARMED); the
/// other leg just spins. The difference is what stopping cost.
uint32_t tick_leg(bool stop) {
    uint32_t best = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8; ++i) {
        cli();
        pit_ran = false;
        sei();
        while (!pit_ran) {                       // sync to a tick edge
        }
        cli();
        pit_ran = false;
        const uint32_t t0 = cycles_now();
        if (stop) {
            K::idle_if_empty();                  // sei + SLEEP, then masked again
        } else {
            sei();
        }
        while (!pit_ran) {
        }
        cli();
        const uint32_t d = pit_stamp - t0;
        sei();
        if (d < best) {
            best = d;
        }
    }
    return best;
}

void tb_standby() {
    quiesce();

    // The round itself, timed.
    console_drain();
    const uint32_t r0 = cycles_now();
    ask(SleepDepth::standby);
    const uint32_t r1 = cycles_now();
    const uint8_t ctrla = SLPCTRL.CTRLA;

    verdict("a two-voter round arms standby",
            Probe::last_ok && ctrla == (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm));
    verdict("both stakeholders were asked, at the depth requested",
            Probe::asked == 1 && Probe::asked_depth == SleepDepth::standby);
    print(serial, "  the round (post -> armed, two voters through the kernel) cost ",
          r1 - r0, " CLK_PER cycles = ", (r1 - r0) / (crystal_hz / 1'000'000u), " us",
          crlf);

    // The CPU really stops. Over the same wall-clock span, a loop that
    // only turns when the CPU runs turns thousands of times awake and
    // once per wake asleep.
    console_drain();
    volatile uint32_t work_busy = 0;
    volatile uint32_t work_sleep = 0;
    sei();
    {
        const uint32_t t0 = P::now();
        while (P::now() - t0 < 32u) {
            work_busy = work_busy + 1;
        }
    }
    const uint32_t c0 = cycles_now();
    {
        const uint32_t t0 = P::now();
        for (;;) {
            cli();
            if (P::now() - t0 >= 32u) {
                sei();
                break;
            }
            K::idle_if_empty();          // the ARMED mode, not IDLE
            work_sleep = work_sleep + 1;
        }
    }
    const uint32_t c1 = cycles_now();

    print(serial, "  over 32 ticks: ", work_busy, " loop turns awake, ", work_sleep,
          " asleep (one per wake)", crlf);
    print(serial, "  the RUNSTDBY stopwatch counted ", c1 - c0,
          " CLK_PER ticks across the sleeping window (nominal ", 32u * cycles_per_tick,
          ")", crlf);
    verdict("the loop is frozen while the CPU sleeps", work_sleep <= 40);
    verdict("the same span awake turns it thousands of times",
            work_busy > 100u * (work_sleep ? work_sleep : 1u));
    verdict("and the clock chain kept running for the peripheral that asked",
            c1 - c0 > 31u * cycles_per_tick && c1 - c0 < 33u * cycles_per_tick);
    verdict("the site is still armed: a wake that says nothing changes nothing",
            SLPCTRL.CTRLA == (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm) &&
            Pm::armed_depth() == SleepDepth::standby);

    // What stopping and restarting cost, against the same span spun.
    console_drain();
    const uint32_t slept = tick_leg(true);
    const uint32_t spun = tick_leg(false);
    print(serial, "  a whole PIT period from t0 to the wake ISR's own stamp: asleep ",
          slept, " CLK_PER, spinning ", spun, " (standby costs ",
          static_cast<int32_t>(slept) - static_cast<int32_t>(spun),
          " cycles beside a clock the stopwatch keeps alive)", crlf);
    verdict("the standby wake costs less than a microsecond here",
            slept >= spun && slept - spun < crystal_hz / 1'000'000u * 2u);

    // The first event after the wake.
    const uint8_t wakes_before = Probe::wakes;
    release_sleep();
    print(serial, "  WakeReport: ", Probe::wakes - wakes_before, " received, was=",
          depth_name(Probe::woke_from), crlf);
    verdict("the first event after the wake disarms",
            SLPCTRL.CTRLA == 0 && !P::sleep_armed() &&
            AvrSleepSite::armed() == SleepDepth::none);
    verdict("and publishes the WakeReport of the depth that was armed",
            Probe::wakes == wakes_before + 1 && Probe::woke_from == SleepDepth::standby);
    verdict("the no-op request behind it still replies ok", Probe::last_ok);
    verdict("the manager forgot the round", Pm::armed_depth() == SleepDepth::none);

    quiesce();
}

// ---- c: the deadline guard ---------------------------------------------------------

void tc_deadline() {
    quiesce();

    Probe::deadline.arm_every(1);          // a deadline every tick: always near
    const uint32_t near_ticks = TimeEvents<P>::ticks_to_next().value_or(0xFFFFFFFFu);
    ask(SleepDepth::deep);
    print(serial, "  nearest deadline ", near_ticks, " ticks away, min_deep_ticks 2: ",
          Probe::last_ok ? "accepted" : "refused", crlf);
    verdict("a deadline nearer than min_deep_ticks refuses a deep request",
            !Probe::last_ok && !P::sleep_armed() &&
            Pm::armed_depth() == SleepDepth::none);
    verdict("and nobody is even asked", Probe::asked == 0);

    // Light is not deep: the guard does not apply to it.
    ask(SleepDepth::light);
    verdict("the same near deadline lets a light request through",
            Probe::last_ok && SLPCTRL.CTRLA == (SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm));
    release_sleep();

    Probe::deadline.disarm();
    Probe::deadline.arm_every(1000);
    Probe::asked = 0;
    const uint32_t far_ticks = TimeEvents<P>::ticks_to_next().value_or(0);
    ask(SleepDepth::deep);
    print(serial, "  nearest deadline ", far_ticks, " ticks away: ",
          Probe::last_ok ? "accepted" : "refused", crlf);
    verdict("a distant deadline lets the same request through",
            Probe::last_ok && SLPCTRL.CTRLA == (SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm));
    verdict("and this time the stakeholders were asked", Probe::asked == 1);
    release_sleep();

    Probe::deadline.disarm();
    verdict("with nothing armed, ticks_to_next has nothing to say",
            !TimeEvents<P>::ticks_to_next().has_value());
    ask(SleepDepth::deep);
    verdict("and an empty armed list does not stand in the way",
            Probe::last_ok && SLPCTRL.CTRLA == (SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm));
    release_sleep();

    quiesce();
}

// ---- d: the voters -----------------------------------------------------------------

void td_voters() {
    quiesce();

    ask(SleepDepth::standby);
    verdict("an idle bus votes yes and the round arms",
            Probe::last_ok && SLPCTRL.CTRLA == (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm));
    release_sleep();

    // A transfer in flight: the engine's start() went asynchronous, so
    // its completion interrupt is still owed.
    post<Bus>(FakeBus::Request{1, {}});
    pump();
    verdict("the fake engine took the transfer and kept it", FakeBus::started == 1);

    Probe::asked = 0;
    ask(SleepDepth::standby);
    verdict("a bus mid-transfer refuses and the round aborts",
            !Probe::last_ok && !P::sleep_armed() &&
            Pm::armed_depth() == SleepDepth::none);
    verdict("the other stakeholder was still asked - unanimity, not first-no",
            Probe::asked == 1);

    post<Bus>(TransferDone{bus_ok});
    pump();
    Probe::asked = 0;
    ask(SleepDepth::standby);
    verdict("with the transfer finished the same request is accepted",
            Probe::last_ok && SLPCTRL.CTRLA == (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm));
    release_sleep();

    // The other stakeholder refusing has the same weight.
    Probe::accept = false;
    Probe::asked = 0;
    ask(SleepDepth::standby);
    verdict("any single refusal ends the round, whoever casts it",
            !Probe::last_ok && !P::sleep_armed() && Probe::asked == 1);
    Probe::accept = true;

    quiesce();
}

// ---- e: the standing restrictions ---------------------------------------------------

void te_locks() {
    quiesce();

    {
        PowerLock lock = Pm::restrict(SleepDepth::light);
        verdict("a lock is held and names its ceiling",
                static_cast<bool>(lock) && lock.level() == SleepDepth::light &&
                Pm::ceiling() == SleepDepth::light);

        ask(SleepDepth::deep);
        print(serial, "  restrict(light) + request(deep): SLPCTRL.CTRLA=",
              hex(SLPCTRL.CTRLA), " manager=", depth_name(Pm::armed_depth()),
              " voters asked at ", depth_name(Probe::asked_depth), crlf);
        verdict("a deep request is forced down to light",
                Probe::last_ok &&
                SLPCTRL.CTRLA == (SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm) &&
                Pm::armed_depth() == SleepDepth::light);
        verdict("and the stakeholders vote on the CLAMPED depth",
                Probe::asked_depth == SleepDepth::light);
    }
    verdict("the lock's scope ended, the ladder is whole again",
            Pm::ceiling() == SleepDepth::deep);

    // This request is also the wake: it disarms the light round, reports
    // it, and is then judged against the machine as it is now.
    const uint8_t wakes_before = Probe::wakes;
    ask(SleepDepth::deep);
    verdict("the request that arrives armed is first of all a wake",
            Probe::wakes == wakes_before + 1 && Probe::woke_from == SleepDepth::light);
    verdict("and the fresh round arms the full depth",
            SLPCTRL.CTRLA == (SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm));
    release_sleep();

    // Nesting: the shallowest live restriction wins.
    PowerLock standby_lock = Pm::restrict(SleepDepth::standby);
    verdict("one lock alone sets the ceiling", Pm::ceiling() == SleepDepth::standby);
    PowerLock light_lock = Pm::restrict(SleepDepth::light);
    verdict("the shallowest live restriction wins", Pm::ceiling() == SleepDepth::light);

    Probe::asked_depth = SleepDepth::none;
    ask(SleepDepth::deep);
    verdict("a deep request under both locks arms the shallowest",
            SLPCTRL.CTRLA == (SLPCTRL_SMODE_IDLE_gc | SLPCTRL_SEN_bm) &&
            Probe::asked_depth == SleepDepth::light);
    release_sleep();

    light_lock.release();
    verdict("releasing it falls back to the other", Pm::ceiling() == SleepDepth::standby);
    light_lock.release();
    verdict("releasing twice is not releasing someone else's",
            Pm::ceiling() == SleepDepth::standby);

    {
        PowerLock moved = static_cast<PowerLock&&>(standby_lock);
        verdict("a moved-from lock holds nothing",
                !static_cast<bool>(standby_lock) && static_cast<bool>(moved));
        verdict("but the right moved, it did not die",
                Pm::ceiling() == SleepDepth::standby);
    }
    verdict("and the mover's scope ends the restriction",
            Pm::ceiling() == SleepDepth::deep);

    quiesce();
}

// ---- menu ---------------------------------------------------------------------------

void register_tests() {
    bench.letter('a', "the ladder and what it arms", ta_ladder);
    bench.letter('b', "a real standby round through the kernel", tb_standby);
    bench.letter('c', "the deadline guard", tc_deadline);
    bench.letter('d', "the voters", td_voters);
    bench.letter('e', "the standing restrictions", te_locks);
}

void help() {
    print(serial, "test_avr_power:", crlf);
    bench.menu();
}

} // namespace

// The console. Its DRE interrupt is also a wake source, which is why
// every measurement here is preceded by console_drain().
ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

// The timebase AND the wake source. The stamp is taken FIRST so it
// measures the wake, not the bookkeeping behind it.
ISR(RTC_PIT_vect) {
    pit_stamp = Watch::read();
    brio::Ticker::pit();
    ++pit_irqs;
    pit_ran = true;
}

// All four TCB vectors are bound as a net: an unbound vector on this
// core is a jump to 0, i.e. a silent reset loop, and a suite that sleeps
// is exactly where a stray flag would find one.
ISR(TCB0_INT_vect) { (void)brio::Tcb<0>::take_flags(); }
ISR(TCB1_INT_vect) { (void)WatchLo::take_flags(); }
ISR(TCB2_INT_vect) { (void)WatchHi::take_flags(); }
ISR(TCB3_INT_vect) { (void)brio::Tcb<3>::take_flags(); }

int main() {
    const ResetFlags why = Reset::take_flags();

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    watch_init();
    K::init_all();
    sei();

    auto board = board_id();
    if (board.empty()) {
        board = "?";
    }
    print(serial, crlf, "test_avr_power - power-management test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ", hex(SYSCFG.REVID),
          ", RSTFR=", hex(why.raw), ")", crlf);
    print(serial, "  timebase ", Ticker::ticks_per_second, " Hz = ", cycles_per_tick,
          " CLK_PER cycles per tick; the PIT is also the wake source", crlf);
    register_tests();
    help();

    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            help();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "? for help", crlf);
        }
        bench.prompt();
    }
}
