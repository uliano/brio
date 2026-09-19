// test_rp2350_multicore - the reference bench suite for TWO KERNELS ON
// TWO CORES, ON EITHER INSTRUCTION SET: rp2350/multicore.hpp (the launch
// of core 1 through the bootrom's FIFO protocol, the SIO doorbell, core
// 1 back into the ROM through the power-on state machine),
// util/inbox.hpp (the bridge: send, the inbox, the bell-first drain, a
// crossing reply), the platform per core (Rp2350Platform<0> and <1>: two
// tickers, two breadcrumbs, the mispost check on the queues) and the
// kernel's two static checks behind them (design/kernel.md section 12).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// Every letter runs on both architectures and is judged the same way.
// Where a verdict must say something different of the two halves, its own
// text says it - which happens once here, in letter b: on the Cortex-M33
// half the two tickers are two SysTicks, core-private counters of clk_sys
// cycles; on the Hazard3 half they are two comparators against ONE shared
// microsecond counter, which is also this suite's ruler.
//
// NOTHING TO WIRE. The console is core 0's, UART0 through the Debug
// Probe's bridge on GP0/GP1; core 1 has no console here and speaks
// through the bridge only - which is the point: every fact about core 1
// below was carried by an event that crossed. A second console for core
// 1 is the chip's own USB CDC port, whose chapter is not written yet.
//
// THE PROGRAM. Core 0 runs the console loop and, letter by letter, one
// kernel of its own (`Tenuto<P0, Origin>` stepped by hand: init_all()
// and step(), the host tests' way). Core 1 is launched at boot into a
// real `Tenuto<P1, Echo, Beat>::run()`: Echo answers every Ping it is
// sent with a Pong sent back, halts on a Halt (a panic on core 1), Beat
// sends one Tick per millisecond of ITS ticker while told to. ONE bell
// vector name serves both cores and both architectures - `isr_sio_bell`,
// interrupt 26, core-local - and the one timebase vector ticks the
// ticker of the core that took it. The ruler is the platform timer, the
// microsecond counter both cores and both architectures share.
//
// THE FAULT VECTORS ARE DELIBERATELY NOT BOUND: letter g kills core 1
// with a panic, and what must happen then is that core 1 alone stops -
// the crt's weak handler spinning - while core 0 goes on reporting. An
// app that bound fault_reset() would reboot the chip instead.
//
// What is exercised, letter by letter:
//   a  the launch: core 1 came up on the bootrom's protocol after the
//      reset launch() begins with (its CPUID and the ruler's stamp
//      written by its entry, the time it all took), and the protocol
//      alone at a running core 1 is REFUSED within its bounded wait -
//      it is not in the ROM, and only the reset puts it there
//   b  two tickers: core 1's ticker advances at 1000 Hz against the
//      ruler, beside core 0's, each on its own counter
//   c  the round trip: 64 Pings one at a time, each Pong back in order,
//      the latency of a crossing and its return on the ruler
//   d  a burst at the inbox: 200 Pings in a row into an inbox of 16,
//      whatever a full one drops counted by the sender, every ACCEPTED
//      one answered in order, no Pong lost on the way back - and the
//      account that says where the rest went, which on this chip is the
//      receiving AO's own QUEUE and not the ring
//   e  both directions at once: core 1's Beat at 1 kHz on its own time
//      events into core 0's inbox while core 0 pumps Pings for a second
//      - the counts consistent, no mispost, no overflow on the return
//   f  the mispost: a post<Echo>() from core 0 - Echo's queue is core
//      1's - is refused and counted, the queue untouched
//   g  core 1 dies and is brought back: a Halt sent, core 1's panic
//      breadcrumb read by core 0 from core 1's own record, core 1 put
//      back into the ROM through the power-on state machine and
//      launched again, answering Pings as before
//   h  saturation: as fast as the inbox takes them, the losses counted
//      where they happen and the two accounts balancing - a run bounded
//      by a COUNT, because an overflow counter is sixteen bits and
//      saturates, and a loss it could not count is a loss the account
//      cannot see
//   j  THE TWO CHANNELS ARE TWO: the bell's eight flags and their
//      acknowledge, the mailbox FIFO's depth MEASURED (the datasheet
//      says four in one place and eight in another), the launch refused
//      while the mailbox interrupt would eat its echoes - and a whole
//      relaunch with the bell interrupt left ENABLED, which is what the
//      doorbell buys over the RP2040's FIFO bell
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/panic.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/multicore.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/inbox.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P0 = Rp2350Platform<0>;
using P1 = Rp2350Platform<1>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial> bench;

uint32_t us_now() { return Mtime::micros(); }

const char* arch_name() {
    return core_kind == CoreKind::hazard3 ? "RISC-V Hazard3" : "Arm Cortex-M33";
}

// ---- the events that cross ----------------------------------------------------
struct Ping {
    uint16_t n;
};
struct Pong {
    uint16_t n;
};
struct Halt {
    uint8_t context;
};
struct Beating {
    bool on;
};
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
        return match(
            e, [](Entry) { return handled(); },
            [](Pong p) {
                if (pongs != 0u && p.n != static_cast<uint16_t>(last_pong + 1u)) {
                    in_order = false;
                }
                last_pong = p.n;
                ++pongs;
                last_pong_us = us_now();
                return handled();
            },
            [](Tick) {
                ++ticks_seen;
                return handled();
            },
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
        return match(
            e, [](Entry) { return handled(); },
            [](Ping p) {
                seen = seen + 1u;
                send<Origin>(Pong{p.n});
                return handled();
            },
            [](Halt h) {
                panic<P1>(PanicCode::assert_failed, h.context);
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

struct Beat : Fsm<Beat, Beating, Tick> {
    static inline EventQueue<Event, 8, P1> queue;
    static inline TimeEvent<P1, Beat, Tick> metronome{Tick{}};
    static inline uint32_t sent = 0;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(
            e, [](Entry) { return handled(); },
            [](Beating b) {
                if (b.on) {
                    metronome.arm_every(1);
                } else {
                    metronome.disarm();
                }
                return handled();
            },
            [](Tick) {
                send<Origin>(Tick{});
                ++sent;
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

using K0 = Tenuto<P0, Origin>;
using K1 = Tenuto<P1, Echo, Beat>;
using Drain0 = Inboxes<Origin>;
using Drain1 = Inboxes<Echo, Beat>;

// What core 1's entry writes for core 0 to read: plain words, written
// once before anything else runs on core 1.
volatile uint32_t core1_cpuid = 0xFFu;
volatile uint32_t core1_started_us = 0;
volatile uint32_t core1_runs = 0;
volatile uint32_t core1_vectors = 0;
uint32_t launch_us = 0;
bool launch_ok = false;

/// A stack top of the suite's own, for the two letters that PROBE the
/// launch protocol at a core 1 that is running: protocol() writes its
/// sixteen-byte frame below whatever it is given before the first word
/// goes out, and core 1's real stack is in use.
alignas(16) uint32_t probe_stack[8];
uint32_t probe_stack_top() {
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(probe_stack + 8));
}

[[noreturn]] void core1_entry() {
    core1_cpuid = P1::core_id();
    core1_started_us = us_now();
    core1_runs = core1_runs + 1u;
    core1_vectors = core_vector_base();   // what the shim adopted, read on core 1
    // Core 1's own timebase. On the Hazard3 half this restarts the tick
    // generator the shared microsecond counter rides on, so a span
    // measured across a launch is short by that restart - microseconds,
    // and the ruler stays monotonic.
    (void)CoreTicker<1>::init(clock);
    Drain1::enable();   // core 1's bell, in ITS controller
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
        return {Echo::queue.overflows(),   Echo::queue.misposts(),   Beat::queue.overflows(),
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

/// Core 1 reset and launched again, WITH CORE 0'S BELL LEFT ENABLED: the
/// launch protocol is the mailbox FIFO's and the bell is another line
/// (rp2350/multicore.hpp), so nothing has to be masked around it. The
/// inboxes and core 1's queues are emptied because the AOs behind them
/// are about to start their lives over.
void relaunch_core1() {
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
}

// =============================================================================
// a - the launch, and a second one refused
// =============================================================================
void ta_launch() {
    print(serial, "  core 1: launch ", launch_ok ? "completed" : "FAILED", " in ", launch_us,
          " us; its entry wrote CPUID ", core1_cpuid, ", started at t=", core1_started_us,
          " us, with its vector base at ", hex(core1_vectors), "; this core is ", P0::core_id(),
          " on ", arch_name(), crlf);
    bench.verdict("core 1 came up on the bootrom's protocol and wrote its CPUID",
                  launch_ok && core1_cpuid == 1u);
    bench.verdict("the entry shim ran on core 1: Core1::entered()", Core1::entered());
    bench.verdict("core 1's vector base is this core's - one table, two cores",
                  core1_vectors == core_vector_base() && core1_vectors != 0u);
    bench.verdict("this console runs on core 0", P0::core_id() == 0u && P0::on_own_core());
    bench.verdict("the launch took under 10 ms", launch_us < 10'000u);

    Origin::reset_counts();
    send<Echo>(Ping{1});
    bench.verdict("core 1's kernel answers a Ping before the second launch is tried",
                  serve_until(1, 20'000u));
    const uint32_t runs_before = core1_runs;
    const uint32_t started_before = core1_started_us;
    const uint32_t t0 = us_now();
    // The protocol alone, no reset - and onto a stack of this suite's,
    // because core 1 is standing on its own.
    const bool again = Core1::protocol(&core1_entry, probe_stack_top());
    const uint32_t took = us_now() - t0;
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
                  !again && core1_runs == runs_before);
}

// =============================================================================
// b - two tickers
// =============================================================================
void tb_tickers() {
    const uint32_t t0 = us_now();
    const uint32_t c0 = CoreTicker<0>::ticks();
    const uint32_t c1 = CoreTicker<1>::ticks();
    while (us_now() - t0 < 500'000u) {
    }
    const uint32_t d0 = CoreTicker<0>::ticks() - c0;
    const uint32_t d1 = CoreTicker<1>::ticks() - c1;
    print(serial, "  over 500 ms of the ruler: core 0's ticker +", d0, ", core 1's +", d1,
          "; the two run on ",
          core_kind == CoreKind::hazard3
              ? "two comparators against the one shared microsecond counter"
              : "two SysTicks, each counting its own core's clk_sys",
          crlf);
    bench.verdict("core 1's ticker runs at 1000 Hz on its own counter, within 0.2 %",
                  d1 >= 499u && d1 <= 501u);
    bench.verdict("and core 0's beside it", d0 >= 499u && d0 <= 501u);

    // THE PROOF THAT THEY ARE TWO: stop this core's and watch the other
    // go on. pause() is the calling core's own - SysTick's interrupt
    // enable on one half, mie.MTIE on the other - so core 1's is
    // untouched by it. Core 0 has no time event running here, and the
    // console does not read the clock: 100 ms of stopped kernel time
    // costs this suite nothing.
    const uint32_t p0 = CoreTicker<0>::ticks();
    const uint32_t p1 = CoreTicker<1>::ticks();
    CoreTicker<0>::pause();
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 100'000u) {
    }
    const uint32_t frozen = CoreTicker<0>::ticks() - p0;
    const uint32_t ran = CoreTicker<1>::ticks() - p1;
    CoreTicker<0>::resume();
    const uint32_t p2 = CoreTicker<0>::ticks();
    const uint32_t t2 = us_now();
    while (us_now() - t2 < 20'000u) {
    }
    const uint32_t again = CoreTicker<0>::ticks() - p2;
    print(serial, "  core 0's ticker paused for 100 ms of the ruler: it advanced ", frozen,
          " ticks, core 1's ", ran, "; after the resume core 0's advanced ", again,
          " in 20 ms", crlf);
    bench.verdict("they are two timebases and not one: core 0's stops where core 0 stops it "
                  "and core 1's runs on",
                  frozen == 0u && ran >= 99u && ran <= 101u);
    bench.verdict("and core 0's takes up again after the pause", again >= 19u && again <= 21u);
}

// =============================================================================
// c - the round trip
// =============================================================================
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
    bench.verdict("every Ping is answered by its Pong, in order",
                  all && Origin::pongs == 64u && Origin::in_order);
    bench.verdict("a crossing and its return take under 100 us but for the first (the cold "
                  "cache on both cores)",
                  sum - hi < 64u * 100u && lo < 100u);
    const Counts d = Counts::now().since(c0);
    bench.verdict("no overflow either way, no mispost",
                  d.echo_in_over == 0u && d.origin_in_over == 0u && d.echo_q_mis == 0u &&
                      d.origin_q_over == 0u && d.echo_q_over == 0u);
}

// =============================================================================
// d - a burst at the inbox
// =============================================================================
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
          " accepted, ", dropped, " dropped and counted by the sender; Echo dispatched ",
          d.echo_seen, "; ", Origin::pongs, " Pongs back, ",
          Origin::in_order ? "in order" : "OUT OF ORDER", "; lost at Echo's queue ",
          lost_at_echo_queue, ", at the return inbox ", lost_at_return, ", at Origin's queue ",
          lost_at_origin_queue, "; misposts ", d.echo_q_mis, "/", d.origin_q_mis, crlf);
    bench.verdict("the sender counts what a full inbox drops",
                  accepted + dropped == 200u && accepted >= Inbox<Echo>::capacity());
    bench.verdict("THE ACCOUNT BALANCES: every accepted Ping is a Pong back or a loss counted "
                  "where it happened (a queue or an inbox), nothing silent",
                  accepted == Origin::pongs + lost_at_echo_queue + lost_at_return +
                                  lost_at_origin_queue);
    bench.verdict("the Pongs that came back are in order", Origin::in_order);
    bench.verdict("no mispost anywhere", d.echo_q_mis == 0u && d.origin_q_mis == 0u);
}

// =============================================================================
// e - both directions at once
// =============================================================================
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
    print(serial, "  one second, both ways, a Ping every 20 us: ", sent, " Pings sent, ",
          Origin::pongs, " Pongs back (", Origin::in_order ? "in order" : "OUT OF ORDER", "), ",
          Origin::ticks_seen, " Ticks from core 1's metronome (", fired,
          " sent); Echo inbox overflows ", d.echo_in_over, ", Echo queue overflows ",
          d.echo_q_over, ", Beat queue overflows ", d.beat_q_over, ", return overflows ",
          d.origin_in_over, ", Origin queue overflows ", d.origin_q_over, crlf);
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

// =============================================================================
// f - the mispost
// =============================================================================
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

// =============================================================================
// g - core 1 halts on a panic, is reset and launched again
// =============================================================================
void tg_panic_and_relaunch() {
    (void)take_panic_record<P1>();
    print(serial, "  sending Halt ...", crlf);
    send<Echo>(Halt{0x11});
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 10'000u) {
    }
    const auto record = take_panic_record<P1>();
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
    const uint32_t r0 = us_now();
    const bool reset_ok = Core1::reset();
    const uint32_t reset_us = us_now() - r0;
    print(serial, "  core 1 into the ROM through the power-on state machine: ",
          reset_ok ? "drained and waiting" : "NO ANSWER", " after ", reset_us, " us; FRCE_OFF now ",
          hex(Psm::held()), crlf);
    bench.verdict("Core1::reset() puts core 1 back into the bootrom's wait loop", reset_ok);
    bench.verdict("and leaves the power-on state machine holding nothing - erratum RP2350-E19 "
                  "wants every FRCE_OFF bit but PROC1 clear at a reboot, and this leaves that "
                  "one clear too",
                  Psm::held() == 0u);
    print(serial, "  launching ...", crlf);
    relaunch_core1();
    print(serial, "  launched again in ", launch_us, " us, CPUID ", core1_cpuid, crlf);
    bench.verdict("core 1 launches again and answers", launch_ok && core1_cpuid == 1u);
    Origin::reset_counts();
    send<Echo>(Ping{7});
    bench.verdict("the first Ping after the relaunch comes back",
                  serve_until(1, 20'000u) && Origin::last_pong == 7u);
}

// =============================================================================
// h - saturation
// =============================================================================
void th_saturation() {
    // AS FAST AS THE INBOX TAKES THEM, the metronome running: what
    // saturation does is a fact to state - losses, each counted where it
    // happens, the lower-priority AO of core 1 starved by pack order -
    // and the one verdict is the account.
    //
    // THE RUN IS BOUNDED BY A COUNT AND NOT BY A TIME, because an
    // overflow counter is sixteen bits and SATURATES rather than wrap
    // (kernel/event_queue.hpp): a loss it could not count would make the
    // account below false for a reason that is not a fault. Flat out,
    // core 0 offers some forty Pings for every one core 1 dispatches, so
    // this many leaves the drop counts a comfortable margin under
    // 65535 - and the last verdict checks that margin held rather than
    // assuming it. The counters are monotone, so this letter is a
    // measurement ONCE PER BOOT: asked twice it spends the margin, and
    // then says so instead of pretending.
    constexpr uint32_t offered = 40'000;
    Origin::reset_counts();
    const Counts c0 = Counts::now();
    const uint32_t beat_before = Beat::sent;
    send<Beat>(Beating{true});
    const uint32_t t0 = us_now();
    uint32_t sent = 0;
    uint16_t n = 0;
    while (sent < offered && us_now() - t0 < 500'000u) {
        if (Inbox<Echo>::send(Ping{static_cast<uint16_t>(n + 1u)})) {
            ++n;
            ++sent;
        }
        (void)K0::step();
    }
    const uint32_t flat_out_us = us_now() - t0;
    send<Beat>(Beating{false});
    (void)serve_until(sent, 50'000u);
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 20'000u) {
        (void)K0::step();
    }
    const Counts d = Counts::now().since(c0);
    const uint32_t fired = Beat::sent - beat_before;
    print(serial, "  ", offered, " Pings offered flat out in ", flat_out_us, " us: ", sent,
          " accepted (", d.echo_in_over, " refused), Echo dispatched ", d.echo_seen, ", ",
          Origin::pongs, " Pongs back; lost at Echo's queue ", d.echo_q_over,
          ", at the return inbox ", d.origin_in_over, ", at Origin's queue ", d.origin_q_over,
          " (Pongs and Ticks alike); Beat dispatched ", fired, " Ticks, ", d.beat_q_over,
          " starved by pack order, ", Origin::ticks_seen, " seen", crlf);
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
    bench.verdict("and no counter saturated, so the account above counted every loss there was "
                  "- an overflow counter stops at 65535 rather than wrap",
                  Echo::queue.overflows() != UINT16_MAX && Beat::queue.overflows() != UINT16_MAX &&
                      Origin::queue.overflows() != UINT16_MAX &&
                      Inbox<Echo>::overflows() != UINT16_MAX &&
                      Inbox<Origin>::overflows() != UINT16_MAX);
}

// =============================================================================
// j - the bell and the mailbox are two channels
// =============================================================================
void tj_two_channels() {
    // 1. The bell, watched with its interrupt off so it can be seen
    //    standing at all - with it on, the handler answers it in the
    //    store's own shadow.
    (void)SioDoorbell<0>::disable();
    const bool clear_before = !SioDoorbell<0>::pending();
    SioDoorbell<0>::ring();   // core 0 ringing core 0: DOORBELL_IN_SET
    const bool stood = SioDoorbell<0>::pending();
    const uint32_t raised = SioDoorbell<0>::pop_all();
    const bool gone = !SioDoorbell<0>::pending();
    const bool re_enabled = SioDoorbell<0>::enable();
    print(serial, "  the bell of core 0, rung by core 0: was ", clear_before ? "clear" : "SET",
          ", stood ", stood ? "yes" : "NO", ", pop_all took ", hex(raised), " of ",
          hex(SioDoorbell<0>::flags), ", now ", gone ? "clear" : "STILL SET", crlf);
    bench.verdict("a ring raises this core's own bell flag and pop_all acknowledges exactly it",
                  clear_before && stood && raised == SioDoorbell<0>::bell && gone && re_enabled);
    bench.verdict("the bell of the OTHER core cannot be enabled from here: one line number "
                  "serves both cores, so the verb refuses rather than arm the wrong one",
                  !SioDoorbell<1>::enable());

    // 2. The other core's bell, read from here through the outbound
    //    register's read-back: core 1 answers its bells, so after a round
    //    trip none stands.
    Origin::reset_counts();
    send<Echo>(Ping{1});
    const bool answered = serve_until(1, 20'000u);
    const bool core1_bell_clear = !SioDoorbell<1>::pending();
    bench.verdict("core 1 acknowledges its bell: after a round trip nothing stands in the "
                  "outbound register's read-back",
                  answered && core1_bell_clear);

    // 3. The mailbox FIFO, whose depth the datasheet gives twice and
    //    differently (3.1.5 says four entries, the FIFO_ST register
    //    description says eight): MEASURE it. The words stay unread -
    //    core 1 is in its kernel, not in the ROM - and the reset below
    //    is what clears them, core 1's own boot code draining its FIFO
    //    before it answers.
    SioMailbox::clear_errors();
    uint32_t depth = 0;
    while (SioMailbox::push(0xA5A50000u + depth) && depth < 64u) {
        ++depth;
    }
    const bool refused = !SioMailbox::push(0xDEADBEEFu);
    const bool no_wof = !SioMailbox::write_on_full();
    print(serial, "  the mailbox FIFO towards core 1 took ", depth,
          " words before it was full (the datasheet says four in 3.1.5 and eight in the "
          "FIFO_ST register description); the next push was ",
          refused ? "refused" : "ACCEPTED", ", the sticky write-on-full flag ",
          no_wof ? "clear" : "SET", crlf);
    bench.verdict("the outgoing mailbox has room for a whole number of words and then says so",
                  depth >= 4u && depth <= 8u);
    bench.verdict("brio's push refuses on a full FIFO instead of raising the sticky "
                  "write-on-full flag a bare store would",
                  refused && no_wof);

    // 4. The launch refuses while this core's mailbox interrupt is on -
    //    its handler would eat the protocol's echoes. Safe to try: the
    //    check is the first thing protocol() does, so no word is pushed,
    //    and the line is raised by the INCOMING FIFO, which is empty. The
    //    pending bit is cleared first, because on the Arm half the
    //    controller LATCHED every assertion of that line while it was
    //    disabled - and the launches above asserted it at every echo.
    bool guard_ok = false;
    if (!SioMailbox::readable() && !SioMailbox::read_on_empty() && !SioMailbox::write_on_full()) {
        Irq::clear_pending(SioMailbox::irq());
        Irq::enable(SioMailbox::irq());
        guard_ok = !Core1::protocol(&core1_entry, probe_stack_top());
        Irq::disable(SioMailbox::irq());
    }
    bench.verdict("the launch refuses to run while the mailbox interrupt is enabled on this core",
                  guard_ok);

    // 5. THE WHOLE RELAUNCH WITH THE BELL INTERRUPT LEFT ENABLED. On the
    //    RP2040 the bell was the mailbox and this had to be masked; here
    //    the two are two lines and two registers.
    const bool bell_on_before = Irq::enabled(SioDoorbell<0>::irq());
    relaunch_core1();
    const bool bell_on_after = Irq::enabled(SioDoorbell<0>::irq());
    Origin::reset_counts();
    send<Echo>(Ping{9});
    const bool alive = serve_until(1, 20'000u) && Origin::last_pong == 9u;
    print(serial, "  relaunched in ", launch_us, " us with core 0's bell interrupt ",
          bell_on_before ? "enabled" : "DISABLED", " throughout (", bell_on_after ? "still on" : "OFF",
          " after); core 1 ", alive ? "answers" : "IS SILENT", crlf);
    bench.verdict("a launch completes with the bell interrupt enabled: the protocol is the "
                  "mailbox's channel and the bell is another line",
                  bell_on_before && bell_on_after && launch_ok && alive);
}

void banner() {
    print(serial, crlf,
          "test_rp2350_multicore - two kernels on two cores (rp2350/multicore.hpp, "
          "util/inbox.hpp) on ",
          arch_name(), ", clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}   // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() {
    if (brio::core_id() == 0u) {
        brio::CoreTicker<0>::tick();
    } else {
        brio::CoreTicker<1>::tick();
    }
}
/// ONE NAME, BOTH CORES: SIO_IRQ_BELL is core-local and carries the same
/// number on each, so the core that took it decides which drain runs.
extern "C" void isr_sio_bell() {
    if (brio::core_id() == 0u) {
        Drain0::isr();
    } else {
        Drain1::isr();
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output();
    K0::init_all();
    relaunch_core1();
    Drain0::enable();
    brio::enable_interrupts();

    bench.letter('a', "the launch of core 1, and a second one refused", ta_launch);
    bench.letter('b', "two tickers, one per core", tb_tickers);
    bench.letter('c', "the round trip, 64 Pings one at a time", tc_round_trip);
    bench.letter('d', "a burst at the inbox, and where the losses land", td_burst);
    bench.letter('e', "both directions at once for a second", te_both_ways);
    bench.letter('f', "a post to the other core's queue: refused", tf_mispost);
    bench.letter('g', "core 1 halts on a panic, is reset and launched again",
                 tg_panic_and_relaunch);
    bench.letter('h', "saturation: the losses counted where they happen", th_saturation);
    bench.letter('j', "the bell and the mailbox are two channels", tj_two_channels);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    " core1=", launch_ok ? "up" : "FAILED", brio::crlf);
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
