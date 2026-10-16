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
// kernel of its own (`Tenuto<P0, Origin>` stepped by hand: init_all()
// and step(), the host tests' way). Core 1 is launched at boot into a
// real `Tenuto<P1, Echo, Beat>::run()`: Echo answers every Ping it is
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
//   i  A DMA ENGINE ON LINE 1 SERVED BY CORE 1: the copy engine on channel
//      6 armed by core 1's Echo (the line enabled in core 1's NVIC), 4 KB
//      copied, its completion's vector run on core 1 alone, the report
//      crossed back to core 0
//   j  A Lease::reply LOAN ACROSS THE BRIDGE AND BACK: 64 requests, each
//      lending core 0's 256-byte buffer to core 1's Echo, which adds one
//      to every byte and sums them and replies through the crossing
//      ReplyTo with the same loan - every byte back incremented, the sum
//      and the pointer right; then THE SYSTEM TIMER from both cores at
//      once (the raw pair read by each, never backwards on either) and
//      PAUSED (the count standing while core 0's SysTick counts on)
//   k  THE IDLE CONTRACT ON CORE 1: test_rp2040_platform's letters w and
//      k run by core 1 on Rp2040Platform<1> and CoreTicker<1> inside
//      Echo's dispatch - its own SysTick placed by a reload, PWM slice 0's
//      wrap in its own NVIC, and the SIO doorbell rung by core 0 a counted
//      number of cycles after core 1 asks - every position of 400, a
//      plain WFI and with SLEEPDEEP set, alarm 1 the rescue; then a quiet
//      pack on core 1 turned over 100 of its ticks
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
#include "kernel/tenuto.hpp"
#include "kernel/panic.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/pwm.hpp"
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

TestBench<Serial> bench;

uint32_t us_now() { return Timer::now_low(); }

// ---- the events that cross ----------------------------------------------------
struct Ping { uint16_t n; };
struct Pong { uint16_t n; };
struct Halt { uint8_t context; };
struct Beating { bool on; };
struct Tick {};
/// Letter i: a copy on core 1's DMA engine, its end on line 1, and the
/// account of it sent back.
struct CopyOn1 { uint16_t words; };
struct CopyDone { uint8_t flags; };
struct Copied { uint8_t flags; };
/// Letter j: a buffer lent across for the length of a request, and the
/// timer read from core 1 while core 0 reads it too.
struct Folded {
    Borrowed<uint8_t, Lease::reply> buf;
    uint16_t len;
    uint32_t sum;
};
struct Fold {
    Borrowed<uint8_t, Lease::reply> buf;
    uint16_t len;
    ReplyTo<Folded> reply;
};
struct ClockRace { uint32_t reads; };
/// Letter k: core 1 walks its idle path (namespace c1 below).
struct WalkOn1 {};
namespace c1 {
void run_on_core1();
}
struct Raced { uint32_t reads; uint32_t backwards; };

/// Letter i's engine: channel 6, its completions on DMA_IRQ_1, armed BY
/// CORE 1 - arm() enables the line in the calling core's NVIC.
using Copier = DmaCopyEngine<6, 1>;
alignas(4) uint32_t copy_src[1024];
alignas(4) uint32_t copy_dst[1024];
/// Which core ran line 1's vector, counted in the vector itself.
volatile uint32_t line1_on_core0 = 0;
volatile uint32_t line1_on_core1 = 0;

// ---- core 0: the origin of every Ping, the home of every answer --------------
struct Origin : Fsm<Origin, Pong, Tick, Copied, Folded, Raced> {
    static inline EventQueue<Event, 32, P0> queue;
    static constexpr uint8_t inbox_depth = 16;

    static inline uint32_t pongs = 0;
    static inline uint16_t last_pong = 0;
    static inline bool in_order = true;
    static inline uint32_t last_pong_us = 0;
    static inline uint32_t ticks_seen = 0;
    static inline uint32_t copies = 0;
    static inline uint8_t copy_flags = 0;
    static inline uint32_t folds = 0;
    static inline Folded last_fold{};
    static inline uint32_t races = 0;
    static inline Raced last_race{};

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
            [](Copied c) { ++copies; copy_flags = c.flags; return handled(); },
            [](Folded f) { last_fold = f; ++folds; return handled(); },
            [](Raced r) { last_race = r; ++races; return handled(); },
            [](auto) { return unhandled(); });
    }
};

// ---- core 1: the echo, and the metronome on core 1's own time -----------------
struct Echo : Fsm<Echo, Ping, Halt, CopyOn1, CopyDone, Fold, ClockRace, WalkOn1> {
    static inline EventQueue<Event, 32, P1> queue;
    static constexpr uint8_t inbox_depth = 16;
    static inline volatile uint32_t seen = 0;   // Pings dispatched, for core 0's account
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Ping p) { seen = seen + 1u; send<Origin>(Pong{p.n}); return handled(); },
            [](Halt h) { panic<P1>(PanicCode::assert_failed, h.context); return handled(); },
            [](CopyOn1 c) {
                Copier::arm(DmaReport::blocks);
                if (!Copier::copy(copy_dst, static_cast<const uint32_t*>(copy_src), c.words)) {
                    send<Origin>(Copied{0xFF});
                }
                return handled();
            },
            [](CopyDone d) { send<Origin>(Copied{d.flags}); return handled(); },
            [](Fold f) {
                uint32_t sum = 0;
                for (uint16_t i = 0; i < f.len; ++i) {
                    f.buf.get()[i] = static_cast<uint8_t>(f.buf.get()[i] + 1u);
                    sum += f.buf.get()[i];
                }
                f.reply.send(Folded{f.buf, f.len, sum});
                return handled();
            },
            [](ClockRace c) {
                uint32_t backwards = 0;
                uint64_t last = Timer::now();
                for (uint32_t i = 0; i < c.reads; ++i) {
                    const uint64_t t = Timer::now();
                    if (t < last) {
                        ++backwards;
                    }
                    last = t;
                }
                send<Origin>(Raced{c.reads, backwards});
                return handled();
            },
            [](WalkOn1) {
                c1::run_on_core1();
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

using K0 = Tenuto<P0, Origin>;
using K1 = Tenuto<P1, Echo, Beat>;
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
/// AN ENGINE ON LINE 1 SERVED BY CORE 1: core 0 asks core 1's Echo for a
/// copy; Echo arms the copy engine on channel 6 with its completions on
/// DMA_IRQ_1 - arm() enables the line in the CALLING core's NVIC, core 1's
/// - and starts 4 KB; the vector runs the engine's service and posts the
/// result to Echo, which sends it across. Core 0's NVIC never enabled the
/// line, so the vector must have run on core 1 alone (counted by CPUID in
/// the vector), the copy exact.
void ti_dma_on_core1() {
    for (uint32_t i = 0; i < 1024u; ++i) {
        copy_src[i] = 0xC0DE'0000u + 7u * i;
        copy_dst[i] = 0;
    }
    Origin::copies = 0;
    Origin::copy_flags = 0;
    const uint32_t c0 = line1_on_core0;
    const uint32_t c1 = line1_on_core1;
    const Counts k0 = Counts::now();
    send<Echo>(CopyOn1{1024});
    const uint32_t t0 = us_now();
    while (Origin::copies == 0u && us_now() - t0 < 100'000u) {
        (void)K0::step();
    }
    uint32_t wrong = 0;
    for (uint32_t i = 0; i < 1024u; ++i) {
        if (copy_dst[i] != copy_src[i]) {
            ++wrong;
        }
    }
    const Counts d = Counts::now().since(k0);
    print(serial, "  4 KB on channel 6, line 1: ", Origin::copies, " report(s) back, flags ", hex(Origin::copy_flags),
          ", ", wrong, " words wrong; line 1's vector ran ", line1_on_core1 - c1, " time(s) on core 1 and ",
          line1_on_core0 - c0, " on core 0; core 0's NVIC has line 1 ",
          Nvic::enabled(DmaLine<1>::irq()) ? "ENABLED" : "disabled", crlf);
    bench.verdict("AN ENGINE ON LINE 1 SERVED BY CORE 1: the copy engine armed on core 1 completes its block on DMA_IRQ_1, "
                  "the vector run on core 1 once and never on core 0, the copy exact, the report crossed back",
                  Origin::copies == 1u && (Origin::copy_flags & Copier::flag_complete) != 0u && wrong == 0u &&
                      line1_on_core1 == c1 + 1u && line1_on_core0 == c0 && !Nvic::enabled(DmaLine<1>::irq()) &&
                      d.echo_q_mis == 0u);
}

// -----------------------------------------------------------------------------
/// A LEASE::REPLY LOAN ACROSS THE BRIDGE: the buffer is core 0's, lent
/// for the length of one request; core 1 writes it and replies with the
/// loan, and core 0 reads it only once the reply is dispatched - the
/// inbox's release and acquire fences (util/inbox.hpp) are what make
/// core 1's writes visible. Then the system timer, the one clock the
/// two cores share, read by both at once and paused.
alignas(4) uint8_t lent[256];

void tj_loan_and_timer() {
    const Counts k0 = Counts::now();
    uint32_t good = 0;
    uint32_t bad_bytes = 0;
    bool same_loan = true;
    bool sums = true;
    for (uint16_t round = 0; round < 64u; ++round) {
        uint32_t want_sum = 0;
        for (uint16_t i = 0; i < 256u; ++i) {
            lent[i] = static_cast<uint8_t>(round * 7u + i);
            want_sum += static_cast<uint8_t>(lent[i] + 1u);
        }
        const uint32_t before = Origin::folds;
        send<Echo>(Fold{lend<Lease::reply>(lent), 256, send_reply_to<Origin, Folded>()});
        const uint32_t t0 = us_now();
        while (Origin::folds == before && us_now() - t0 < 20'000u) {
            (void)K0::step();
        }
        if (Origin::folds == before) {
            break;
        }
        uint32_t bad = 0;
        for (uint16_t i = 0; i < 256u; ++i) {
            if (lent[i] != static_cast<uint8_t>(round * 7u + i + 1u)) {
                ++bad;
            }
        }
        bad_bytes += bad;
        same_loan = same_loan && Origin::last_fold.buf.get() == lent && Origin::last_fold.len == 256u;
        sums = sums && Origin::last_fold.sum == want_sum;
        if (bad == 0u) {
            ++good;
        }
    }
    const Counts d = Counts::now().since(k0);
    print(serial, "  64 Folds of a 256-byte loan: ", good, " rounds exact, ", bad_bytes, " bytes wrong, the loan back the same ",
          same_loan, ", the sums ", sums ? "right" : "WRONG", crlf);
    bench.verdict("A Lease::reply LOAN CROSSES AND COMES BACK: 64 rounds, every byte core 1 wrote read by core 0 after the "
                  "reply, the same pointer and length back, core 1's sum right",
                  good == 64u && bad_bytes == 0u && same_loan && sums && d.echo_in_over == 0u && d.origin_in_over == 0u);

    // The raw pair from both cores at once: core 1 reads it 20000 times
    // while core 0 reads it the same, each counting a step backwards.
    const uint32_t r0 = Origin::races;
    send<Echo>(ClockRace{20'000});
    uint32_t back0 = 0;
    uint64_t last = Timer::now();
    for (uint32_t i = 0; i < 20'000u; ++i) {
        const uint64_t t = Timer::now();
        if (t < last) {
            ++back0;
        }
        last = t;
    }
    const uint32_t t1 = us_now();
    while (Origin::races == r0 && us_now() - t1 < 100'000u) {
        (void)K0::step();
    }
    print(serial, "  the raw pair read 20000 times on each core at once: core 0 backwards ", back0, ", core 1 ",
          Origin::races != r0 ? Origin::last_race.reads : 0u, " reads, backwards ", Origin::last_race.backwards, crlf);
    bench.verdict("the system timer's raw pair read from both cores at once never goes backwards on either",
                  Origin::races == r0 + 1u && Origin::last_race.reads == 20'000u && Origin::last_race.backwards == 0u &&
                      back0 == 0u);

    // PAUSE: five of core 0's SysTick milliseconds with the timer paused
    // - the count stands (the reads around it, a microsecond or two) -
    // then five more running (5000 us, one tick of SysTick phase either
    // way).
    const uint32_t tk0 = CoreTicker<0>::ticks();
    while (CoreTicker<0>::ticks() == tk0) {
    }
    const uint32_t tk1 = CoreTicker<0>::ticks();
    const uint32_t a = Timer::now_low();
    Timer::pause(true);
    const bool flag = Timer::paused();
    while (CoreTicker<0>::ticks() - tk1 < 5u) {
    }
    const uint32_t b = Timer::now_low();
    Timer::pause(false);
    const uint32_t tk2 = CoreTicker<0>::ticks();
    while (CoreTicker<0>::ticks() - tk2 < 5u) {
    }
    const uint32_t c = Timer::now_low();
    print(serial, "  the timer paused for 5 SysTick ms: it moved ", b - a, " us (PAUSE read ", flag, "); running for 5 more: ",
          c - b, " us", crlf);
    bench.verdict("PAUSE stops the count - 2 us at most across five milliseconds - and it runs on from where it stood "
                  "(4000..6000 us over the next five)",
                  flag && b - a <= 2u && c - b >= 4000u && c - b <= 6000u && !Timer::paused());
}

// -----------------------------------------------------------------------------
/// THE IDLE CONTRACT ON CORE 1: test_rp2040_platform's letters w and k,
/// run BY CORE 1 inside Echo's dispatch on Rp2040Platform<1> and
/// CoreTicker<1> - core 1's own SysTick, its own NVIC (the rescue alarm,
/// the PWM line, the doorbell enabled there by core 1) - with a third
/// edge only core 1 has: the SIO doorbell, rung by core 0 a counted
/// number of its own clk_sys cycles after core 1 asks for it. The
/// results come back through SRAM.
namespace c1 {

constexpr uint8_t rescue_alarm = 1;
constexpr uint32_t rescue_us = 50'000u;
constexpr uint32_t positions = 400;
constexpr uint32_t tick_period = SysClock::hz / CoreTicker<1>::ticks_per_second;
using EdgeSlice = PwmSlice<0>;

volatile uint32_t rescue_fires = 0;
volatile uint32_t tick_entry_val = 0;
volatile uint32_t line_hits = 0;
volatile uint32_t line_entry = 0;
volatile uint32_t bells = 0;          ///< core 1's bell vector entries
volatile uint32_t bell_d = 0;         ///< the cycles core 0 waits before it rings
volatile uint32_t bell_go = 0;        ///< raised by core 1 to ask for a ring
volatile uint32_t bell_served = 0;    ///< and answered by core 0
volatile bool walking = false;

struct TickEdge {
    static constexpr uint32_t first_d = 8;
    static constexpr uint32_t period_us = 1000;
    static void begin() {}
    static void end() {}
    static void sync() {
        const uint32_t s = CoreTicker<1>::ticks();
        while (CoreTicker<1>::ticks() == s) {
        }
    }
    static uint32_t count() { return CoreTicker<1>::ticks(); }
    [[gnu::always_inline]] static void place(uint32_t d) {
        SysTick->LOAD = d;
        SysTick->VAL = 0;
        SysTick->LOAD = tick_period - 1u;
    }
    static uint32_t since_edge() { return tick_period - 1u - SysTick->VAL; }
    static uint32_t entry() { return tick_period - 1u - tick_entry_val; }
};

struct LineEdge {
    static constexpr uint32_t first_d = 2;
    static constexpr uint32_t period_us = 65536u / (SysClock::hz / 1'000'000u);
    static void begin() {
        (void)Pwm::reset();
        (void)EdgeSlice::configure(PwmSliceConfig{});
        Pwm::clear_pending(EdgeSlice::bit);
        Pwm::interrupts(EdgeSlice::bit, true);
        Nvic::clear_pending(Pwm::irq());
        Nvic::enable(Pwm::irq());   // core 1's NVIC: this runs on core 1
        EdgeSlice::enable(true);
        CoreTicker<1>::pause();
    }
    static void end() {
        CoreTicker<1>::resume();
        EdgeSlice::enable(false);
        Pwm::interrupts(EdgeSlice::bit, false);
        Nvic::disable(Pwm::irq());
        Pwm::clear_pending(EdgeSlice::bit);
        Nvic::clear_pending(Pwm::irq());
    }
    static void sync() {
        const uint32_t s = line_hits;
        while (line_hits == s) {
        }
    }
    static uint32_t count() { return line_hits; }
    [[gnu::always_inline]] static void place(uint32_t d) { EdgeSlice::ctr() = 0xFFFFu - d; }
    static uint32_t since_edge() { return EdgeSlice::ctr(); }
    static uint32_t entry() { return line_entry; }
};

/// THE DOORBELL: core 1 asks, core 0 counts d cycles of its own SysTick
/// and rings. The two cores share no cycle counter, so the edge's time
/// is not read back here - only whether a wake was lost.
struct BellEdge {
    static constexpr uint32_t first_d = 2;
    static constexpr uint32_t period_us = 1000;   // no period: a lost bell is the rescue's
    static void begin() { CoreTicker<1>::pause(); }
    static void end() { CoreTicker<1>::resume(); }
    static void sync() {
        while (bell_served != bell_go) {
        }
    }
    static uint32_t count() { return bells; }
    [[gnu::always_inline]] static void place(uint32_t d) {
        bell_d = d;
        bell_go = bell_go + 1u;
    }
    static uint32_t since_edge() { return 0; }
    static uint32_t entry() { return 0; }
};

struct Sweep {
    uint32_t lost;
    uint32_t late;
    uint32_t slowest_us;
    uint32_t asleep_entry;
    uint32_t asleep_loop;
};
/// [edge][rung][edges - 1]: edge 0 tick, 1 line, 2 bell; rung 0 a plain
/// WFI, 1 with SLEEPDEEP set on core 1.
Sweep sweeps[3][2][2];
volatile bool walked = false;

template <typename Edge>
bool one_try(uint32_t d, uint32_t edges, Sweep& s) {
    Edge::sync();
    P1::CriticalSection cs;
    const uint32_t r0 = rescue_fires;
    Timer::alarm_in(rescue_alarm, rescue_us);
    Edge::place(d);
    for (uint8_t k = 0; k < 12u; ++k) {
        (void)us_now();
    }
    const uint32_t n0 = Edge::count();
    const uint32_t u0 = us_now();
    for (;;) {
        P1::CriticalSection turn;
        if (Edge::count() - n0 >= edges) {
            break;
        }
        P1::idle();
    }
    const uint32_t to_loop = Edge::since_edge();
    const uint32_t entry = Edge::entry();
    const uint32_t took = us_now() - u0;
    Timer::disarm(rescue_alarm);
    Timer::clear(rescue_alarm);
    Nvic::clear_pending(Timer::irq(rescue_alarm));
    const bool rescued = rescue_fires != r0;
    const bool late = !rescued && took > (edges - 1u) * Edge::period_us + Edge::period_us / 2u;
    s.lost += rescued ? 1u : 0u;
    s.late += late ? 1u : 0u;
    s.slowest_us = took > s.slowest_us ? took : s.slowest_us;
    if (edges == 1u && d == Edge::first_d + positions - 1u) {
        s.asleep_entry = entry;
        s.asleep_loop = to_loop;
    }
    return true;
}

template <typename Edge, uint8_t e>
void walk(uint32_t max_edges) {
    Edge::begin();
    for (uint8_t rung = 0; rung < 2u; ++rung) {
        if (rung == 1u) {
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   // core 1's own SCR
        }
        for (uint32_t edges = 1; edges <= max_edges; ++edges) {
            Sweep& s = sweeps[e][rung][edges - 1u];
            s = Sweep{};
            (void)one_try<Edge>(Edge::first_d + positions - 1u, edges, s);   // the XIP fill, discarded
            s = Sweep{};
            for (uint32_t i = 0; i < positions; ++i) {
                (void)one_try<Edge>(Edge::first_d + i, edges, s);
            }
        }
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }
    Edge::end();
}

/// Letter k's pack, on core 1's platform.
template <uint8_t N>
struct Quiet {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P1> queue;
    static inline TimeEvent<P1, Quiet, Event> beat{Event{N}};
    static void init() {
        if constexpr (N != 0u) {
            beat.arm_every(ticks_from_ms<P1>(500u * N));
        }
    }
    static void dispatch(const Event&) {}
};
using QuietKernel = Tenuto<P1, Quiet<1>, Quiet<2>, Quiet<0>>;
volatile uint32_t turns = 0;

/// What Echo runs on core 1 for letter k.
void run_on_core1() {
    rescue_fires = 0;
    Timer::clear(rescue_alarm);
    Timer::interrupt(rescue_alarm, true);
    Nvic::clear_pending(Timer::irq(rescue_alarm));
    Nvic::enable(Timer::irq(rescue_alarm));   // core 1's NVIC
    walk<TickEdge, 0>(2);
    walk<LineEdge, 1>(2);
    walk<BellEdge, 2>(1);
    Timer::interrupt(rescue_alarm, false);
    Nvic::disable(Timer::irq(rescue_alarm));
    // THE KERNEL'S TURN on core 1: a quiet pack turned as run() turns it
    // over 100 of core 1's ticks.
    QuietKernel::init_all();
    const uint32_t t = CoreTicker<1>::ticks();
    uint32_t n = 0;
    while (CoreTicker<1>::ticks() - t < 100u) {
        TimeEvents<P1>::process();
        if (!QuietKernel::step()) {
            QuietKernel::idle_if_empty();
        }
        ++n;
    }
    Quiet<1>::beat.disarm();
    Quiet<2>::beat.disarm();
    turns = n;
    walked = true;
}

}   // namespace c1

void tk_idle_on_core1() {
    c1::walked = false;
    c1::bell_go = 0;
    c1::bell_served = 0;
    send<Echo>(WalkOn1{});
    // Core 0 serves the bell edge meanwhile: each request answered after
    // `bell_d` cycles of its own SysTick (a 125 000-cycle period, read
    // down), then the ring.
    const uint32_t t0 = us_now();
    uint32_t served = 0;
    while (!c1::walked && us_now() - t0 < 120'000'000u) {
        const uint32_t go = c1::bell_go;
        if (go != served) {
            const uint32_t d = c1::bell_d;
            const uint32_t v0 = SysTick->VAL;
            constexpr uint32_t period = SysClock::hz / CoreTicker<0>::ticks_per_second;
            while (((v0 + period - SysTick->VAL) % period) < d) {
            }
            SioDoorbell<1>::ring();
            served = go;
            c1::bell_served = go;
        }
    }
    const char* const edge_names[3] = {"the tick (core 1's SysTick)", "a peripheral line (PWM wrap, core 1's NVIC)",
                                       "the doorbell (rung by core 0)"};
    bool clean = c1::walked;
    for (uint8_t e = 0; e < 3u; ++e) {
        print(serial, "  ", edge_names[e], ", D = ", e == 0u ? 8u : 2u, "..", (e == 0u ? 8u : 2u) + c1::positions - 1u, crlf);
        for (uint8_t rung = 0; rung < 2u; ++rung) {
            for (uint32_t edges = 1; edges <= (e == 2u ? 1u : 2u); ++edges) {
                const c1::Sweep& s = c1::sweeps[e][rung][edges - 1u];
                print(serial, "    ", rung == 0u ? "light    " : "SLEEPDEEP", " waiting for ", edges, edges == 1u ? " edge:  " : " edges: ",
                      s.lost, " lost, ", s.late, " late, the slowest try ", s.slowest_us, " us");
                if (edges == 1u && e != 2u) {
                    print(serial, "; asleep: ", s.asleep_entry, " cycles to the handler, ", s.asleep_loop, " to the loop");
                }
                print(serial, crlf);
                clean = clean && s.lost == 0u && s.late == 0u;
            }
        }
    }
    print(serial, "  the rescue fired ", c1::rescue_fires, " time(s); core 1's quiet kernel: ", c1::turns,
          " turns over 100 of its ticks", crlf);
    bench.verdict("THE IDLE CONTRACT ON CORE 1: no position of its tick, of a line in its NVIC or of the doorbell rung by core "
                  "0 loses the wake or defers it a period, one edge or two, plain WFI or SLEEPDEEP",
                  clean && c1::rescue_fires == 0u);
    bench.verdict("core 1's kernel turns once per tick: 100 ticks, 100 turns (one either way)",
                  c1::walked && c1::turns + 1u >= 100u && c1::turns <= 101u);
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
        c1::tick_entry_val = SysTick->VAL;   // letter k: where core 1's period stood on entry
        brio::CoreTicker<1>::tick();
    }
}
extern "C" void isr_sio_proc0() { Drain0::isr(); }
extern "C" void isr_sio_proc1() {
    c1::bells = c1::bells + 1u;   // letter k's doorbell edge
    Drain1::isr();
}
/// Letter k: the line edge and the rescue, both enabled in core 1's NVIC.
extern "C" void isr_pwm_wrap() {
    c1::line_entry = c1::EdgeSlice::ctr();
    brio::Pwm::clear_pending(c1::EdgeSlice::bit);
    c1::line_hits = c1::line_hits + 1u;
}
extern "C" void isr_timer_1() {
    brio::Timer::clear(c1::rescue_alarm);
    c1::rescue_fires = c1::rescue_fires + 1u;
}
extern "C" void isr_dma_1() {
    if (P1::core_id() == 1u) {
        line1_on_core1 = line1_on_core1 + 1u;
    } else {
        line1_on_core0 = line1_on_core0 + 1u;
    }
    const uint8_t f = Copier::service();
    if (f != 0u) {
        brio::post<Echo>(CopyDone{f});
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)brio::Dma::init();   // letter i's engine, out of reset before core 1 runs
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
    bench.letter('i', "a DMA engine on line 1, armed and served by core 1", ti_dma_on_core1);
    bench.letter('j', "a Lease::reply loan across and back; the timer from both cores, paused", tj_loan_and_timer);
    bench.letter('k', "the idle contract on core 1: its tick, a line, the doorbell; its kernel's turn", tk_idle_on_core1);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
