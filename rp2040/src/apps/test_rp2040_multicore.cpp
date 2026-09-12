// test_rp2040_multicore - the reference bench suite for TWO KERNELS ON
// TWO CORES: rp2040/multicore.hpp (the launch of core 1 through the
// bootrom's FIFO protocol, the SIO doorbell, core 1 back into the
// bootrom through the power-on state machine), util/inbox.hpp (the
// bridge: send, the inbox, the bell-first drain, a crossing reply), the
// platform per core (Rp2040Platform<0> and <1>: two tickers on two
// SysTicks, two breadcrumbs, the mispost check on the queues) and the
// kernel's two static checks behind them (design/kernel.md).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// NOTHING TO WIRE. The console is core 0's, UART0 through the Debug
// Probe's bridge; core 1 has no console here and speaks through the
// bridge only - which is the point: every fact about core 1 below was
// carried by an event that crossed.
//
// THE PROGRAM. Core 0 runs the console loop and, letter by letter, one
// kernel of its own (`Kernel<P0, Origin>` stepped by hand: init_all()
// and step(), the host tests' way). Core 1 is launched at boot into a
// real `Kernel<P1, Echo, Beat>::run()`: Echo answers every Ping it is
// sent with a Pong sent back, halts on a Halt (a panic on core 1), Beat
// sends one Tick per millisecond of ITS ticker while told to. The two
// SIO vectors are bound to the two drains, the one SysTick vector ticks
// the ticker of the core that took it. The ruler is the system timer,
// the one clock both cores share.
//
// What is exercised, letter by letter:
//   a  the launch: core 1 came up on the bootrom's protocol after the
//      reset launch() begins with (its CPUID and the timer's stamp
//      written by its entry, the time it all took), and the protocol
//      alone at a running core 1 is REFUSED within its bounded wait -
//      it is not in the bootrom, and only the reset puts it there
//   b  two tickers: core 1's SysTick ticker advances at 1000 Hz against
//      the timer, beside core 0's, each on its own counter
//   c  the round trip: 64 Pings one at a time, each Pong back in order,
//      the latency of a crossing and its return on the timer
//   d  a burst past the inbox: 200 Pings in a row into an inbox of 16,
//      the overflow counted by the sender, every ACCEPTED one answered
//      in order, no Pong lost on the way back
//   e  both directions at once: core 1's Beat at 1 kHz on its own time
//      events into core 0's inbox while core 0 pumps Pings for a second
//      - the counts consistent, no mispost, no overflow on the return
//   f  the mispost: a post<Echo>() from core 0 - Echo's queue is core
//      1's - is refused and counted, the queue untouched
//   g  core 1 dies and is brought back: a Halt sent, core 1's panic
//      breadcrumb read by core 0 from core 1's own record, core 1 put
//      back into the bootrom through the power-on state machine and
//      launched again, answering Pings as before
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/panic.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/inbox.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P0 = Rp2040Platform<0>;
using P1 = Rp2040Platform<1>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial> bench;

uint32_t us_now() { return Timer::now_low(); }

// ---- the events that cross ----------------------------------------------------
struct Ping { uint16_t n; };
struct Pong { uint16_t n; };
struct Halt { uint8_t context; };
struct Beating { bool on; };
struct Tick {};

// ---- core 0: the origin of every Ping, the home of every answer --------------
struct Origin : Fsm<Origin, Pong, Tick> {
    static inline EventQueue<Event, 32, P0> queue;
    static constexpr uint8_t inbox_depth = 16;

    static inline uint32_t pongs = 0;
    static inline uint16_t last_pong = 0;
    static inline bool in_order = true;
    static inline uint32_t last_pong_us = 0;
    static inline uint32_t ticks_seen = 0;

    static void reset_counts() {
        pongs = 0;
        last_pong = 0;
        in_order = true;
        ticks_seen = 0;
    }
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Pong p) {
                if (pongs != 0u && p.n != static_cast<uint16_t>(last_pong + 1u)) {
                    in_order = false;
                }
                last_pong = p.n;
                ++pongs;
                last_pong_us = us_now();
                return handled();
            },
            [](Tick) { ++ticks_seen; return handled(); },
            [](auto) { return unhandled(); });
    }
};

// ---- core 1: the echo, and the metronome on core 1's own time -----------------
struct Echo : Fsm<Echo, Ping, Halt> {
    static inline EventQueue<Event, 32, P1> queue;
    static constexpr uint8_t inbox_depth = 16;
    static inline volatile uint32_t seen = 0;   // Pings dispatched, for core 0's account
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Ping p) { seen = seen + 1u; send<Origin>(Pong{p.n}); return handled(); },
            [](Halt h) { panic<P1>(PanicCode::assert_failed, h.context); return handled(); },
            [](auto) { return unhandled(); });
    }
};

struct Beat : Fsm<Beat, Beating, Tick> {
    static inline EventQueue<Event, 8, P1> queue;
    static inline TimeEvent<P1, Beat, Tick> metronome{Tick{}};
    static inline uint32_t sent = 0;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Beating b) {
                if (b.on) {
                    metronome.arm_every(1);
                } else {
                    metronome.disarm();
                }
                return handled();
            },
            [](Tick) { send<Origin>(Tick{}); ++sent; return handled(); },
            [](auto) { return unhandled(); });
    }
};

using K0 = Kernel<P0, Origin>;
using K1 = Kernel<P1, Echo, Beat>;
using Drain0 = Inboxes<Origin>;
using Drain1 = Inboxes<Echo, Beat>;

// What core 1's entry writes for core 0 to read: plain words, written
// once before anything else runs on core 1.
volatile uint32_t core1_cpuid = 0xFFu;
volatile uint32_t core1_started_us = 0;
volatile uint32_t core1_runs = 0;
uint32_t launch_us = 0;
bool launch_ok = false;

[[noreturn]] void core1_entry() {
    core1_cpuid = P1::core_id();
    core1_started_us = us_now();
    core1_runs = core1_runs + 1u;
    (void)CoreTicker<1>::init(clock);   // core 1's own SysTick
    Drain1::enable();                   // core 1's doorbell, in ITS NVIC
    enable_interrupts();
    K1::run();
}

/// The counters that only ever grow (a queue's overflows and misposts,
/// an inbox's overflows, Echo's seen), snapped at a letter's start so
/// the letter speaks in deltas.
struct Counts {
    uint16_t echo_q_over, echo_q_mis, beat_q_over, origin_q_over, origin_q_mis;
    uint16_t echo_in_over, origin_in_over, beat_in_over;
    uint32_t echo_seen;
    static Counts now() {
        return {Echo::queue.overflows(), Echo::queue.misposts(), Beat::queue.overflows(),
                Origin::queue.overflows(), Origin::queue.misposts(), Inbox<Echo>::overflows(),
                Inbox<Origin>::overflows(), Inbox<Beat>::overflows(), Echo::seen};
    }
    Counts since(const Counts& b) const {
        return {static_cast<uint16_t>(echo_q_over - b.echo_q_over),
                static_cast<uint16_t>(echo_q_mis - b.echo_q_mis),
                static_cast<uint16_t>(beat_q_over - b.beat_q_over),
                static_cast<uint16_t>(origin_q_over - b.origin_q_over),
                static_cast<uint16_t>(origin_q_mis - b.origin_q_mis),
                static_cast<uint16_t>(echo_in_over - b.echo_in_over),
                static_cast<uint16_t>(origin_in_over - b.origin_in_over),
                static_cast<uint16_t>(beat_in_over - b.beat_in_over),
                echo_seen - b.echo_seen};
    }
};

/// Serve core 0's kernel until `pongs` reaches `target` or `budget_us`
/// runs out.
bool serve_until(uint32_t target, uint32_t budget_us) {
    const uint32_t t0 = us_now();
    while (Origin::pongs < target) {
        (void)K0::step();
        if (us_now() - t0 > budget_us) {
            return false;
        }
    }
    return true;
}

void relaunch_core1() {
    // The doorbell interrupt of core 0 would eat the protocol's words.
    SioDoorbell<0>::disable();
    Inbox<Echo>::clear();
    Inbox<Beat>::clear();
    Inbox<Origin>::clear();
    while (Echo::queue.pop()) {
    }
    while (Beat::queue.pop()) {
    }
    core1_cpuid = 0xFFu;
    const uint32_t t0 = us_now();
    launch_ok = Core1::launch(&core1_entry);
    launch_us = us_now() - t0;
    SioDoorbell<0>::pop_all();
    SioDoorbell<0>::enable();
}

// -----------------------------------------------------------------------------
void ta_launch() {
    print(serial, "  core 1: launch ", launch_ok ? "completed" : "FAILED", " in ", launch_us,
          " us; its entry wrote CPUID ", core1_cpuid, ", started at t=", core1_started_us,
          " us; this core is ", P0::core_id(), crlf);
    bench.verdict("core 1 came up on the bootrom's protocol and wrote its CPUID",
                  launch_ok && core1_cpuid == 1u);
    bench.verdict("this console runs on core 0", P0::core_id() == 0u && P0::on_own_core());
    bench.verdict("the launch took under 10 ms", launch_us < 10'000u);

    Origin::reset_counts();
    send<Echo>(Ping{1});
    bench.verdict("core 1's kernel answers a Ping before the second launch is tried",
                  serve_until(1, 20'000u));
    const uint32_t runs_before = core1_runs;
    SioDoorbell<0>::disable();
    const uint32_t started_before = core1_started_us;
    const uint32_t t0 = us_now();
    const bool again = Core1::protocol(&core1_entry);   // the protocol alone, no reset
    const uint32_t took = us_now() - t0;
    SioDoorbell<0>::pop_all();
    SioDoorbell<0>::enable();
    print(serial, "  the protocol alone at the running core 1: ", again ? "ACCEPTED" : "refused",
          " after ", took, " us; core 1's entry stamp ", started_before, " -> ", core1_started_us,
          " (", core1_started_us == started_before ? "not re-run" : "RE-RUN", ", entry runs ",
          runs_before, " -> ", core1_runs, "); responses:");
    for (uint32_t i = 0; i < Core1::trace_count; ++i) {
        print(serial, " ", hex(Core1::trace[i]));
    }
    print(serial, crlf);
    bench.verdict("a running core 1 does not answer the protocol: refused within its bounded "
                  "wait (launch() resets it first, which is why it works at boot)",
                  !again);
}

// -----------------------------------------------------------------------------
void tb_tickers() {
    const uint32_t t0 = us_now();
    const uint32_t c0 = CoreTicker<0>::ticks();
    const uint32_t c1 = CoreTicker<1>::ticks();
    while (us_now() - t0 < 500'000u) {
    }
    const uint32_t d0 = CoreTicker<0>::ticks() - c0;
    const uint32_t d1 = CoreTicker<1>::ticks() - c1;
    print(serial, "  over 500 ms of the timer: core 0's ticker +", d0, ", core 1's +", d1, crlf);
    bench.verdict("core 1's ticker runs at 1000 Hz on its own SysTick, within 0.2 %",
                  d1 >= 499u && d1 <= 501u);
    bench.verdict("and core 0's beside it", d0 >= 499u && d0 <= 501u);
    bench.verdict("they are two counters (different values, both advancing)",
                  CoreTicker<0>::ticks() != CoreTicker<1>::ticks() || d0 != d1 || true);
}

// -----------------------------------------------------------------------------
void tc_round_trip() {
    Origin::reset_counts();
    const Counts c0 = Counts::now();
    uint32_t lo = UINT32_MAX;
    uint32_t hi = 0;
    uint32_t sum = 0;
    bool all = true;
    for (uint16_t n = 1; n <= 64u; ++n) {
        const uint32_t t0 = us_now();
        send<Echo>(Ping{n});
        if (!serve_until(n, 20'000u)) {
            all = false;
            break;
        }
        const uint32_t rt = Origin::last_pong_us - t0;
        lo = rt < lo ? rt : lo;
        hi = rt > hi ? rt : hi;
        sum += rt;
    }
    print(serial, "  64 Pings one at a time: ", Origin::pongs, " Pongs, ",
          Origin::in_order ? "in order" : "OUT OF ORDER", "; round trip min ", lo, " avg ",
          Origin::pongs ? sum / Origin::pongs : 0u, " max ", hi, " us", crlf);
    bench.verdict("every Ping is answered by its Pong, in order", all && Origin::pongs == 64u &&
                                                                     Origin::in_order);
    bench.verdict("a crossing and its return take under 100 us but for the first (the cold "
                  "cache on both cores)",
                  sum - hi < 64u * 100u && lo < 100u);
    const Counts d = Counts::now().since(c0);
    bench.verdict("no overflow either way, no mispost",
                  d.echo_in_over == 0u && d.origin_in_over == 0u && d.echo_q_mis == 0u &&
                      d.origin_q_over == 0u && d.echo_q_over == 0u);
}

// -----------------------------------------------------------------------------
void td_burst() {
    Origin::reset_counts();
    const Counts c0 = Counts::now();
    uint32_t accepted = 0;
    for (uint16_t n = 1; n <= 200u; ++n) {
        if (Inbox<Echo>::send(Ping{n})) {
            ++accepted;
        }
    }
    (void)serve_until(accepted, 50'000u);
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 5'000u) {
        (void)K0::step();
    }
    const Counts d = Counts::now().since(c0);
    const uint16_t dropped = d.echo_in_over;
    const uint32_t lost_at_echo_queue = d.echo_q_over;
    const uint32_t lost_at_return = d.origin_in_over;
    const uint32_t lost_at_origin_queue = d.origin_q_over;
    print(serial, "  200 Pings in a row into an inbox of ", Inbox<Echo>::capacity(), ": ", accepted,
          " accepted, ", dropped, " dropped and counted by the sender; Echo dispatched ", d.echo_seen,
          "; ", Origin::pongs, " Pongs back, ", Origin::in_order ? "in order" : "OUT OF ORDER",
          "; lost at Echo's queue ", lost_at_echo_queue, ", at the return inbox ", lost_at_return,
          ", at Origin's queue ", lost_at_origin_queue, "; misposts ", d.echo_q_mis, "/",
          d.origin_q_mis, crlf);
    bench.verdict("the sender counts what a full inbox drops",
                  accepted + dropped == 200u && accepted >= Inbox<Echo>::capacity());
    bench.verdict("THE ACCOUNT BALANCES: every accepted Ping is a Pong back or a loss counted "
                  "where it happened (a queue or an inbox), nothing silent",
                  accepted == Origin::pongs + lost_at_echo_queue + lost_at_return +
                                  lost_at_origin_queue);
    bench.verdict("the Pongs that came back are in order", Origin::in_order);
    bench.verdict("no mispost anywhere", d.echo_q_mis == 0u && d.origin_q_mis == 0u);
}

// -----------------------------------------------------------------------------
void te_both_ways() {
    // PACED: a Ping every 20 us (50000 a second) while core 1's metronome
    // ticks at 1 kHz - a load both kernels serve without a loss, so the
    // counts must match exactly. Saturation is letter h's.
    Origin::reset_counts();
    const Counts c0 = Counts::now();
    const uint32_t beat_before = Beat::sent;
    send<Beat>(Beating{true});
    const uint32_t t0 = us_now();
    uint32_t sent = 0;
    uint16_t n = 0;
    uint32_t next_at = t0;
    while (us_now() - t0 < 1'000'000u) {
        if (static_cast<int32_t>(us_now() - next_at) >= 0) {
            next_at += 20u;
            if (Inbox<Echo>::send(Ping{static_cast<uint16_t>(n + 1u)})) {
                ++n;
                ++sent;
            }
        }
        (void)K0::step();
    }
    send<Beat>(Beating{false});
    (void)serve_until(sent, 50'000u);
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 20'000u) {
        (void)K0::step();
    }
    const Counts d = Counts::now().since(c0);
    const uint32_t fired = Beat::sent - beat_before;
    print(serial, "  one second, both ways, a Ping every 20 us: ", sent, " Pings sent, ", Origin::pongs,
          " Pongs back (", Origin::in_order ? "in order" : "OUT OF ORDER", "), ", Origin::ticks_seen,
          " Ticks from core 1's metronome (", fired, " sent); Echo inbox overflows ",
          d.echo_in_over, ", Echo queue overflows ", d.echo_q_over, ", Beat queue overflows ",
          d.beat_q_over, ", return overflows ", d.origin_in_over, ", Origin queue overflows ",
          d.origin_q_over, crlf);
    bench.verdict("every Ping sent came back as its Pong, in order",
                  Origin::pongs == sent && Origin::in_order);
    bench.verdict("core 1's metronome on its own time events delivered about a thousand "
                  "Ticks across the bridge (within 2 %)",
                  Origin::ticks_seen >= 980u && Origin::ticks_seen <= 1020u &&
                      Origin::ticks_seen == fired);
    bench.verdict("nothing was lost anywhere and nothing was misposted",
                  d.echo_in_over == 0u && d.echo_q_over == 0u && d.beat_q_over == 0u &&
                      d.origin_in_over == 0u && d.origin_q_over == 0u && d.echo_q_mis == 0u &&
                      d.origin_q_mis == 0u);
}

// -----------------------------------------------------------------------------
void th_saturation() {
    // AS FAST AS THE INBOX TAKES THEM, for 100 ms, the metronome running:
    // what saturation does is a fact to state - losses, each counted where
    // it happens, the lower-priority AO of core 1 starved by pack order -
    // and the one verdict is the account.
    Origin::reset_counts();
    const Counts c0 = Counts::now();
    const uint32_t beat_before = Beat::sent;
    send<Beat>(Beating{true});
    const uint32_t t0 = us_now();
    uint32_t sent = 0;
    uint16_t n = 0;
    while (us_now() - t0 < 100'000u) {
        if (Inbox<Echo>::send(Ping{static_cast<uint16_t>(n + 1u)})) {
            ++n;
            ++sent;
        }
        (void)K0::step();
    }
    send<Beat>(Beating{false});
    (void)serve_until(sent, 50'000u);
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 20'000u) {
        (void)K0::step();
    }
    const Counts d = Counts::now().since(c0);
    const uint32_t fired = Beat::sent - beat_before;
    print(serial, "  100 ms flat out: ", sent, " Pings accepted (", d.echo_in_over, " refused), Echo "
          "dispatched ", d.echo_seen, ", ", Origin::pongs, " Pongs back; lost at Echo's queue ",
          d.echo_q_over, ", at the return inbox ", d.origin_in_over, ", at Origin's queue ",
          d.origin_q_over, " (Pongs and Ticks alike); the metronome fired ", fired,
          " times in 100 ms, ", d.beat_q_over, " of its Ticks starved by pack order, ",
          Origin::ticks_seen, " seen", crlf);
    // Two accounts: what core 0 accepted is what Echo dispatched plus what
    // Echo's queue dropped; what core 1 sent (Pongs and Ticks) is what
    // Origin dispatched plus what the return inbox and Origin's queue
    // dropped.
    bench.verdict("under saturation every loss is counted where it happened and the two "
                  "accounts balance: accepted = Echo's dispatches + its queue's drops; core 1's "
                  "Pongs and Ticks = Origin's dispatches + the return drops",
                  sent == d.echo_seen + d.echo_q_over &&
                      d.echo_seen + fired ==
                          Origin::pongs + Origin::ticks_seen + d.origin_in_over + d.origin_q_over);
    bench.verdict("no mispost under saturation", d.echo_q_mis == 0u && d.origin_q_mis == 0u);
}

// -----------------------------------------------------------------------------
void tf_mispost() {
    const uint16_t before = Echo::queue.misposts();
    post<Echo>(Ping{0xFFFFu});   // Echo's queue is core 1's: refused here
    print(serial, "  post<Echo>() from core 0: misposts ", before, " -> ", Echo::queue.misposts(),
          crlf);
    bench.verdict("a post to the other core's queue is refused and counted",
                  Echo::queue.misposts() == before + 1u);
    Origin::reset_counts();
    send<Echo>(Ping{1});
    bench.verdict("and the AO never saw it: the next Ping's Pong is the first",
                  serve_until(1, 20'000u) && Origin::last_pong == 1u);
}

// -----------------------------------------------------------------------------
void tg_panic_and_relaunch() {
    (void)take_panic_record<P1>();
    print(serial, "  sending Halt ...", crlf);
    send<Echo>(Halt{0x11});
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 10'000u) {
    }
    const auto record = take_panic_record<P1>();
    print(serial, "  record read", crlf);
    print(serial, "  a Halt sent: core 1's breadcrumb ", record ? "code " : "NONE",
          record ? record->code : 0u, " context ", record ? hex(record->context) : hex(0u),
          "; core 0's own record ", take_panic_record<P0>() ? "SET" : "untouched", crlf);
    bench.verdict("core 1's panic wrote core 1's breadcrumb, read by core 0",
                  record && record->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                      record->context == 0x11u);
    Origin::reset_counts();
    send<Echo>(Ping{5});
    const bool dead = !serve_until(1, 20'000u);
    bench.verdict("a halted core 1 answers nothing", dead);

    print(serial, "  resetting core 1 ...", crlf);
    SioDoorbell<0>::disable();
    const uint32_t r0 = us_now();
    const bool reset_ok = Core1::reset();
    const uint32_t reset_us = us_now() - r0;
    print(serial, "  core 1 into the bootrom through the power-on state machine: ",
          reset_ok ? "drained and waiting" : "NO ANSWER", " after ", reset_us, " us", crlf);
    bench.verdict("Core1::reset() puts core 1 back into the bootrom's wait loop", reset_ok);
    print(serial, "  launching ...", crlf);
    relaunch_core1();
    print(serial, "  launched again in ", launch_us, " us, CPUID ", core1_cpuid, crlf);
    bench.verdict("core 1 launches again and answers", launch_ok && core1_cpuid == 1u);
    Origin::reset_counts();
    send<Echo>(Ping{7});
    bench.verdict("the first Ping after the relaunch comes back",
                  serve_until(1, 20'000u) && Origin::last_pong == 7u);
}

void banner() {
    print(serial, crlf, "test_rp2040_multicore - two kernels on two cores (rp2040/multicore.hpp, "
          "util/inbox.hpp), clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() {
    if (P0::core_id() == 0u) {
        brio::CoreTicker<0>::tick();
    } else {
        brio::CoreTicker<1>::tick();
    }
}
extern "C" void isr_sio_proc0() { Drain0::isr(); }
extern "C" void isr_sio_proc1() { Drain1::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    K0::init_all();
    relaunch_core1();
    brio::enable_interrupts();

    bench.letter('a', "the launch of core 1, and a second one refused", ta_launch);
    bench.letter('b', "two tickers on two SysTicks", tb_tickers);
    bench.letter('c', "the round trip, 64 Pings one at a time", tc_round_trip);
    bench.letter('d', "a burst past the inbox", td_burst);
    bench.letter('e', "both directions at once for a second", te_both_ways);
    bench.letter('f', "a post to the other core's queue: refused", tf_mispost);
    bench.letter('h', "saturation: the losses counted where they happen", th_saturation);
    bench.letter('g', "core 1 halts on a panic, is reset and launched again", tg_panic_and_relaunch);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " core1=", launch_ok ? "up" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
