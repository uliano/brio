// test_rp2350_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the RP2350: Rp2350RubatoPlatform and this
// target's scheduling request (rp2350/schedule.hpp) on BOTH processor
// pairs from one source - the core stratum's PendSV and SVC on the
// Cortex-M33, the machine software interrupt through the crt's trampoline
// and the entry's own mret on Hazard3 - against the model the host suite
// specifies (test/test_rubato).
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs INSIDE the kernel it tests: the program is a Rubato pack of
// five objects, from the highest priority down - Top, High, Mid, Low and
// Console - and the Console, the lowest, owns the UART and runs the
// letters in its own dispatch. Its print is the only writer of the port;
// the other objects write what they saw into a trace in memory, and the
// Console judges and prints it after. A post from a letter is therefore a
// post from the Console's dispatch: to an object above it, the kernel's
// road 2.
//
// ONE SOURCE, TWO HALVES, apart only where rp2350/core.hpp tells them
// apart: the scheduler's vectors (one macro, BRIO_RP2350_SCHEDULER_VECTORS,
// two vectors on one half and one on the other), the stack pointer's
// read, the request's pending bit, and the one letter each half has alone
// - j, the FPU across a preemption, on the Cortex-M33 (Hazard3 here has
// no FPU); k, the software interrupt's own facts, on Hazard3.
//
// NOTHING TO WIRE, NOTHING LIT. The console is the Debug Probe's UART
// bridge on GP0 (TX) / GP1 (RX) = UART0; the suite configures no other
// pad, and the board's LED on GP25 is never driven (letter i checks its
// pad). The interrupt every road-1 letter lands is TIMER1's alarm 0,
// served by core 0 under the one name `isr_timer1_0` on both halves.
//
// THE RULER is TIMER1 itself with its SOURCE on clk_sys (rp2350/timer.hpp,
// datasheet 12.8.1.1): a count a clk_sys cycle on both architectures, read
// in ONE load through the APB bridge (2.2.4: three cycles at least), and
// the counter the alarm matches - so an edge placed `d` counts ahead is
// placed `d` core cycles ahead, and the stamps on either side of a path
// are differences on one counter. The core's own counters are the
// alternatives letter d weighs and prints: on the Cortex-M33 the kernel
// ticker's cycles() (cortexm/ticker.hpp's BasicTicker, SysTick's position
// composed with the tick count, five reads and a check), on Hazard3
// mcycle (one CSR read, a cycle each, off by default in mcountinhibit and
// switched on for that letter alone). The kernel timebase is SysTick on
// one half and the platform timer's comparator on the other
// (rp2350/ticker.hpp); neither is TIMER1.
//
// WHERE THE SCHEDULER RUNS: in thread mode on both halves, outside every
// trap or exception, so an interrupt's post preempts the dispatch it
// lands in at any depth - the Console's own dispatch included, since the
// UART's interrupt starts it at the loop's base.
//
// THE COST MODELS every bound is drawn with, over the release listing of
// this image at 150 MHz, warm (each timed letter runs its path once
// uncounted first, and the XIP cache, 4.4.1, then holds it):
//  - the Cortex-M33: the datasheet gives no cycle counts for the core
//    (3.7 refers to ARM's documents) and ARM's Cortex-M33 TRM is not on
//    this desk, so the model is the Cortex-M4's table (DDI 0439B table
//    3-1: an instruction 1, a load 2, a store 1, a taken branch or a call
//    2 to 4 with the refill - counted at 3 for a branch, 4 for a call
//    and 2 for a BX -, PUSH and POP 1 + N, a POP into the PC 4 + N, MRS
//    and MSR 1 to 2 - counted at 2 -, DSB and ISB up to 4) and its
//    exception figures (3.9.1: an entry 12, a return 12, a tail-chain
//    6), the M33 being the same three-stage design class; the bench
//    judges the model;
//  - Hazard3: datasheet 3.8.7 - an ALU instruction or a CSR access 1, a
//    load 1 (2 when the next instruction uses it), a store 1, `jal` and
//    `jalr` 2 (3 onto a 32-bit instruction that is not 32-bit aligned), a
//    branch 1 predicted and 2 not - counted at 1 not taken and 2 taken,
//    3 onto such an instruction -, `mret` 2, `ecall` 3 to mtvec; an
//    interrupt's entry is not given, and the model takes `ecall`'s 3 for
//    it, the vector's jump 2 after it; cm.push and cm.pop of N
//    registers N + 1, cm.popret N + 3 with its return;
//  - on both, a read of TIMER1 is 3 cycles at least (2.2.4) and the SIO
//    answers in one (3.1).
// A bound is the path's count under its model, plus what may land in it
// (a kernel tick), plus a fifth of the count, rounded up, as the margin.
//
// The letters (each bound's arithmetic beside its verdict in the code):
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms the alarm 2000 cycles ahead and spins 20000 at
//      a mark; the interrupt posts to High. The trace says High's dispatch
//      landed before Low resumes, and every entry reads the ceiling at its
//      own object's index; the Console reads its own, 4, after the chain,
//      and the UART's receive interrupt read 255 (the base) when the
//      letter's key arrived. Ten runs, all alike.
//   b  ROAD 2: Low posts to Mid, Mid to High, inside the handlers; High
//      posts to Low. The trace nests High inside Mid inside Low, and
//      Low's second event is served after its first dispatch ends, never
//      inside it. Ten runs, all alike.
//   c  THE LATENCY OF ROAD 1, the Console's wait loop interrupted: the
//      ruler at the interrupt handler's last statement and at High's
//      handler's first, min, median and max over 100 events after one
//      warm-up, the console drained first; the bound beside it.
//   d  THE RULERS, THE SCAN AND THE POST: the ruler's two-read cost, the
//      core's own counter's beside it and their agreement over a span;
//      activate() with nothing ready below the Console's ceiling (four
//      empty queues, the fifth object at the ceiling), and from Mid's
//      handler one post that asks for nothing (to Low, below it) against
//      one that runs High's dispatch inside it (road 2), each the least of
//      16, the ruler's own cost taken off.
//   e  DEPTH 2: High's handler - started by road 2 (the Console's post),
//      then by road 1 (an interrupt over the Console) - spins at a mark
//      while the alarm posts to Top. Top's entry lands before High resumes;
//      the latency printed and bounded. Ten runs each.
//   f  TIME EVENTS UNDER PREEMPTION, in three parts. (1) Three one-shots
//      matured in one turn, the lowest object's armed last so walked
//      first, posted the way Rubato::run() posts them - the walk with the
//      ceiling held at 0, then the scheduler: nothing is dispatched during
//      the walk and the three are served highest first. (2) Low's handler
//      runs 200 turns of arm(), arm(), disarm(), most of each turn inside
//      a masked window; the alarm lands there and its High preempts at the
//      window's end, arming two timers and disarming the one Low keeps
//      re-arming: the list stays whole - the next turns fire every timer
//      due once, the disarmed one never, nothing twice. (3) The cycles of
//      one arm(), its masked window inside them. The turns are run()'s,
//      written out in the Console's dispatch (base_turn(); the kernel loop
//      itself is below the Console and does not turn while a letter runs).
//   g  THE REQUEST UNDER THE MASK (PRIMASK on one half, mstatus.MIE on the
//      other): a post to a higher object made inside a critical section
//      leaves the request pending and is NOT served inside the section -
//      the receiver's entry follows the section's last mark - and is
//      served when the section ends, before the statement after it; from
//      the Console (to Mid) and from Low's dispatch (to High), ten runs
//      each, with the request's pending bit read inside the section and
//      the cycles from the section's last statement to the receiver's
//      first.
//   j  (the Cortex-M33) THE FPU ACROSS A PREEMPTION, the STM32F4 suite's
//      letter on this core: Low's handler sums a float series with a mark
//      inside the loop; High, started by the interrupt at the mark, sums
//      another; a second run has the interrupt handler compute a float
//      too. Every value bit-exact against the same functions run without
//      preemption; FPCCR's ASPEN and LSPEN read and judged first, since
//      the lazy context save the core stratum leans on is theirs.
//   k  (Hazard3) THE SOFTWARE INTERRUPT'S OWN FACTS. (1) How many
//      instructions retire after the store that raises the flag before the
//      trap is taken, with nothing waiting: a sled of eight stores behind
//      the raw store, Top's dispatch counting the ones that ran - and the
//      same sled behind request_scheduler(), where the count must be 0,
//      the request having waited. (2) The registers across an entry and a
//      nested one: fifteen caller-saved registers loaded with patterns, the
//      alarm landing while they are live and starting High by road 1, High
//      doing the same while the alarm starts Top over it: every pattern
//      found again on both levels. (3) Inside a dispatch the software
//      interrupt started: the flag already lowered, mip.MSIP clear,
//      mstatus.MIE set, mscratch moved by one.
//   i  DARK: every object but the Console idle, no time event armed, the
//      alarm disarmed and its line quiet, the LED's pad not an output, the
//      ceiling the Console's own, interrupts enabled; and the request as
//      scheduler_init() set it (PendSV at the lowest level and not pending
//      on one half, mie.MSIE set and the flag low on the other).
//   h  THE STACK, the last letter: main() painted the free part of core
//      0's 4 KB stack bank below its own frame; the high-water mark
//      against the deepest stack pointer the dispatches and the interrupt
//      sampled in the letters before plus what lies below a sample, the
//      arithmetic beside the verdict.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <atomic>
#include <bit>

#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/flash.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Rp2350RubatoPlatform<>;
using Edge = Timer<1>;

constexpr bool hazard3 = core_kind == CoreKind::hazard3;

/// TIMER1 on clk_sys: one load, a count a cycle. A compiler barrier on
/// either side, so that no plain load or store of the program is
/// scheduled across a stamp (the volatile read orders against volatile
/// accesses alone).
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t t = Edge::now_low();
        std::atomic_signal_fence(std::memory_order_seq_cst);
        return t;
    }
};

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;   // the board's user LED (docs/boards/weact-rp2350b.md): never driven

TestBench<Serial> bench;

// =============================================================================
// The trace, the stamps, the stack samples
// =============================================================================

struct Mark {
    char who;        ///< 'T', 'H', 'M', 'L', 'C'
    char what;       ///< 's' start, 'm' the mark, 'p' after a post, 'r' resumed, 'e' end, 'a' after
    uint8_t ceiling; ///< P::ceiling when it was written
};

constexpr uint8_t trace_capacity = 48;
Mark trace[trace_capacity];
volatile uint8_t trace_len = 0;
volatile bool tracing = true;

/// One entry, from a dispatch or a handler: the append is the decision a
/// mask covers, three stores.
void note(char who, char what) {
    InterruptGuard g;
    const uint8_t n = trace_len;
    if (n < trace_capacity) {
        trace[n] = Mark{who, what, P::ceiling};
        trace_len = static_cast<uint8_t>(n + 1u);
    }
}

void trace_clear() {
    InterruptGuard g;
    trace_len = 0;
}

void print_trace() {
    print(serial, "  trace:");
    for (uint8_t i = 0; i < trace_len; ++i) {
        print(serial, " ", trace[i].who, trace[i].what, trace[i].ceiling);
    }
    print(serial, crlf);
}

/// The trace equals `want`, written as "Ls3 Lm3 Hs1 ..." (who, what,
/// ceiling) with single spaces.
bool trace_is(const char* want) {
    uint8_t i = 0;
    const char* p = want;
    while (*p != '\0') {
        if (i >= trace_len) {
            return false;
        }
        const Mark& m = trace[i];
        if (p[0] != m.who || p[1] != m.what || static_cast<uint8_t>(p[2] - '0') != m.ceiling) {
            return false;
        }
        p += 3;
        if (*p == ' ') {
            ++p;
        }
        ++i;
    }
    return i == trace_len;
}

/// The lowest stack pointer a dispatch or the interrupt saw.
volatile uint32_t sp_low_dispatch = 0xFFFF'FFFFu;
volatile uint32_t sp_low_isr = 0xFFFF'FFFFu;

[[gnu::always_inline]] inline uint32_t stack_pointer() {
#if defined(BRIO_RP2350_CORE_M33)
    return __get_MSP();
#else
    uint32_t sp;
    __asm__ volatile("mv %0, sp" : "=r"(sp));
    return sp;
#endif
}

[[gnu::always_inline]] inline void sample_sp(volatile uint32_t& low) {
    const uint32_t sp = stack_pointer();
    if (sp < low) {
        low = sp;
    }
}

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

/// TIMER1's alarm 0 `d` cycles from now: one edge (the match disarms it),
/// the handler clears the raised flag.
[[gnu::always_inline]] inline void edge_in(uint32_t d) {
    Edge::alarm<0>(Edge::now_low() + d);
}

// =============================================================================
// The pack
// =============================================================================

struct Hit {
    uint8_t n;
};

/// Top, High, Mid and Low: one shape. The handler's first statement
/// stamps the ruler (letter c's far end); then the stack sample, the
/// trace's start entry, the letter's action, the end entry. An event
/// numbered 0 writes no entry (the flush's, below).
template <uint8_t id>
struct Node {
    using Event = Hit;
    static constexpr char name = "THML"[id];
    static inline EventQueue<Event, 4, P> queue;
    static inline void (*action)(uint8_t) = nullptr;
    static inline volatile uint32_t entered = 0;
    static inline volatile uint32_t runs = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        entered = Ruler::now();
        runs = runs + 1u;
        sample_sp(sp_low_dispatch);
        const bool traced = tracing && e.n != 0u;
        if (traced) {
            note(name, 's');
        }
        if (action != nullptr) {
            action(e.n);
        }
        if (traced) {
            note(name, 'e');
        }
    }
};

using Top = Node<0>;
using High = Node<1>;
using Mid = Node<2>;
using Low = Node<3>;

struct ConsoleEvent {
    uint8_t kind;
};
constexpr uint8_t console_rx = 0;

void console_banner();
void console_turn();

/// The lowest object: the UART's owner and the letters' runner.
struct Console {
    using Event = ConsoleEvent;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {
        console_banner();
        bench.prompt();
    }
    static void dispatch(const Event& e) {
        if (e.kind == console_rx) {
            console_turn();
        }
    }
};

using K = Rubato<P, Top, High, Mid, Low, Console>;
constexpr uint8_t console_index = 4;

/// The ceiling the UART's receive interrupt read when a byte arrived.
volatile uint8_t rx_ceiling = 0;

// =============================================================================
// The interrupt the road-1 letters land
// =============================================================================

volatile uint8_t edge_to = 1;           ///< 0: post to Top, 1: to High
volatile uint32_t edge_last = 0;        ///< the ruler at the handler's last statement
volatile uint32_t edge_hits = 0;
volatile bool edge_float = false;       ///< letter j: the handler computes a float too
volatile float edge_acc = 0.0f;

void reset_scenario() {
    trace_clear();
    Top::action = nullptr;
    High::action = nullptr;
    Mid::action = nullptr;
    Low::action = nullptr;
    tracing = true;
}

/// Wait until `runs` moves past `before`, at most `cycles`: true when it
/// did.
bool wait_runs(const volatile uint32_t& runs, uint32_t before, uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (runs == before) {
        if (Ruler::now() - t0 >= cycles) {
            return false;
        }
    }
    return true;
}

/// A post from the Console to Low with nothing to do and nothing traced:
/// its activate() serves whatever waits above the Console first.
void flush() {
    post<Low>(Hit{0});
}

/// Let the console's transmitter fall silent (at most 50 ms; 230 bytes at
/// 115200 are 20 ms) and its last byte leave the wire (87 us, waited 100):
/// its interrupt, landing in a measured path, would be measured.
void console_drain() {
    const uint32_t t0 = Ruler::now();
    while (!Serial::tx_idle() && Ruler::now() - t0 < SysClock::hz / 20u) {
    }
    spin(SysClock::hz / 10'000u);
}

uint32_t ruler_cost() {
    uint32_t best = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t a = Ruler::now();
        const uint32_t b = Ruler::now();
        if (b - a < best) {
            best = b - a;
        }
    }
    return best;
}

/// One millisecond of clk_sys: the letters' wait for a dispatch that
/// should have run within a few hundred cycles.
constexpr uint32_t one_ms = SysClock::hz / 1000u;

/// The bound a verdict is judged against, one figure per half: the
/// arithmetic is beside each use.
constexpr uint32_t per_half(uint32_t m33, uint32_t h3) { return hazard3 ? h3 : m33; }

// =============================================================================
// a - the ceiling, road 1 at depth 1
// =============================================================================
void ta_ceiling() {
    print(serial, "  the UART's receive interrupt read the ceiling ", rx_ceiling,
          " when this letter's key arrived", crlf);
    bench.verdict("the base's ceiling is 255 (read by the receive interrupt as the key arrived)",
                  rx_ceiling == 255u);
    bench.verdict("the Console's dispatch reads its own index, 4", P::ceiling == console_index);

    const char* const want = "Ls3 Lm3 Hs1 He1 Lr3 Le3";
    uint8_t alike = 0;
    bool ceiling_back = true;
    bool once = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        Low::action = [](uint8_t) {
            note('L', 'm');
            edge_to = 1;
            edge_in(2000u);      // 13 us into a 133 us spin: the edge lands inside it
            spin(20000u);
            note('L', 'r');
        };
        const uint32_t h0 = High::runs;
        post<Low>(Hit{1});
        if (trace_is(want)) {
            ++alike;
        } else if (alike == run) {
            print_trace();       // the first that differs
        }
        ceiling_back = ceiling_back && P::ceiling == console_index;
        once = once && High::runs == h0 + 1u;
    }
    reset_scenario();
    print(serial, "  ", alike, " of 10 runs traced ", want, crlf);
    bench.verdict("High's dispatch starts inside Low's, before Low resumes, every ceiling its "
                  "own index (10 runs)",
                  alike == 10u);
    bench.verdict("High was dispatched once per edge", once);
    bench.verdict("the ceiling is the Console's again after every chain", ceiling_back);
}

// =============================================================================
// b - road 2
// =============================================================================
void tb_road2() {
    // L1 posts to Mid, Mid to High, High to Low (below it: waits), and the
    // second Low event is served after the first Low dispatch has ended.
    const char* const want = "Ls3 Ms2 Hs1 Hm1 He1 Mr2 Me2 Lr3 Le3 Ls3 Le3";
    uint8_t alike = 0;
    bool back = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        Low::action = [](uint8_t n) {
            if (n == 1u) {
                post<Mid>(Hit{1});
                note('L', 'r');
            }
        };
        Mid::action = [](uint8_t) {
            post<High>(Hit{1});
            note('M', 'r');
        };
        High::action = [](uint8_t) {
            post<Low>(Hit{2});
            note('H', 'm');
        };
        post<Low>(Hit{1});
        if (trace_is(want)) {
            ++alike;
        } else if (alike == run) {
            print_trace();
        }
        back = back && P::ceiling == console_index;
    }
    reset_scenario();
    print(serial, "  ", alike, " of 10 runs traced ", want, crlf);
    bench.verdict("High inside Mid inside Low, each served inside the post that asked for it, "
                  "every ceiling its own index (10 runs)",
                  alike == 10u);
    bench.verdict("High's post to Low is served after High returns and after Low's own "
                  "dispatch ends, never inside it",
                  alike == 10u);
    bench.verdict("the ceiling is the Console's again", back);
}

// =============================================================================
// c - the latency of road 1
// =============================================================================
constexpr uint32_t latency_events = 100;
uint32_t latency[latency_events];
bool lengthened[latency_events];   ///< per event: a tick, a refill or the console line inside

void sort(uint32_t* v, uint32_t n) {
    for (uint32_t i = 1; i < n; ++i) {
        const uint32_t x = v[i];
        uint32_t j = i;
        while (j > 0 && v[j - 1u] > x) {
            v[j] = v[j - 1u];
            --j;
        }
        v[j] = x;
    }
}

/// The road-1 path from the interrupt handler's last statement (its stamp's
/// load) to High's first (its stamp's load), by the release listing of each
/// half under its cost model (the file header).
///
/// The Cortex-M33 (each taken branch at three, its refill included):
///   the handler's tail after the stamp (a literal, a store, the
///   frame's release, a pop of the PC)                               9
///   the tail-chain into PendSV                                      6
///   PendSV's thirteen instructions to the BX and the BX (two pushes,
///   two stores, a literal: 17; no barrier on this core,
///   cortexm/schedule.hpp) and its return into thread mode (12)     29
///   the entry: cpsie, the call                                      5
///   activate(): nine registers saved and the frame, two literals,
///   the ceiling's load and test (19), Top's visit (the queue's two
///   cursors loaded and compared in line, the branch not taken: 6)
///   and the index's compare with its branch taken (4), High's (the
///   queue's address a literal, the two cursors, the compare, the
///   taken branch: 10), High's serve to the stamp (the ceiling's and
///   the dispatching index's stores and the call to take(): 13;
///   take(): 19; the slot and the stamp's literal: 6)               77
///                                                                 ---
///                                                                 126
///
/// Hazard3:
///   the handler's tail after the stamp (a store, cm.popret: 5), the
///   return into the crt's dispatch and its jump back onto a 32-bit
///   instruction not 4-aligned (3), `meinext` read and its sign
///   branch taken (3), the sixteen loads, the frame and the `mret`
///   (19)                                                           30
///   the software interrupt's entry and the table's jump (3 + 2)     5
///   the trampoline: the frame, sixteen stores, the call            19
///   the body: sixteen instructions, the return                     18
///   the trampoline's sixteen loads, the frame, the `mret`          19
///   the entry: the frame, sixteen stores, csrsi, the call          20
///   activate(): cm.push of thirteen (14), four setups, the ceiling's
///   load and test (21); Top's visit (the queue's two cursors loaded
///   and compared in line, the branch not taken: 4) and the index's
///   compare with its branch taken (4); High's (5); High's serve to
///   the stamp (the ceiling's and the dispatching index's stores and
///   the call to take(): 6; take() with its return: 16; the slot and
///   the stamp's address: 5)                                        61
///                                                                 ---
///                                                                 172
/// of which the crt's two dispatches - the external interrupt's tail
/// (30) and the trampoline's save and restore around the body (38) -
/// are 68, and the entry's own save of the same sixteen registers 17.
///
/// On top of each, at most one kernel tick landing in the path: SysTick
/// on the Cortex-M33 (its entry 12, its fourteen instructions with three
/// literals 22, its return 12 and, landing on an exception return, that
/// return abandoned and begun again 12: 58), the platform timer's trap
/// on Hazard3 (its entry and jump 5, the trampoline's save 19, the tick's
/// twenty-one instructions and return 25, the restore and `mret` 19:
/// 68, taken as 70). And a fifth of the path's count, rounded up: 26 and
/// 35. The bound is 126 + 58 + 26 = 210 on the Cortex-M33 and 172 + 70 +
/// 35 = 277 on Hazard3; the letter reads the kernel tick around each
/// event, and every event over the fastest by more than the margin below
/// must have had a tick in it.
constexpr uint32_t latency_count = per_half(126u, 172u);
constexpr uint32_t latency_bound = per_half(210u, 277u);

/// The XIP cache's misses so far (both halves read the same counters):
/// a refill between two reads of a span is read as letter c reads a tick.
uint32_t xip_misses() { return Xip::accesses() - Xip::hits(); }

/// The console line's handler entries so far: on Hazard3 the external
/// trap serves every pending line before its mret (3.8.6.1's meinext), so
/// a console byte's interrupt pending beside the alarm's lengthens the
/// alarm handler's trap by that handler, and on both halves a handler
/// landing on the path lengthens it; read around an event as the tick is.
volatile uint32_t uart_interrupts = 0;

/// Everything that can lengthen a measured path without being part of it,
/// read before and after: a kernel tick, an XIP refill, the console line.
struct Lengtheners {
    uint32_t ticks, misses, uart;
    static Lengtheners now() { return {Ticker::ticks(), xip_misses(), uart_interrupts}; }
    bool moved() const {
        const Lengtheners n = now();
        return n.ticks != ticks || n.misses != misses || n.uart != uart;
    }
};

void tc_latency() {
    reset_scenario();
    tracing = false;
    // One warm-up event (the XIP cache) not counted, then 100; each waits
    // at most 1 ms for High's first statement.
    console_drain();
    uint32_t missed = 0;
    // An event slower than the path's own count is one a tick landed in,
    // or an XIP refill (the suite is larger than the cache's share a run
    // keeps warm): the kernel tick and the cache's miss counter read
    // around each say so.
    uint32_t slow = 0;
    uint32_t slow_ticked = 0;
    for (uint32_t i = 0; i <= latency_events; ++i) {
        const uint32_t r = High::runs;
        const Lengtheners before = Lengtheners::now();
        edge_to = 1;
        edge_in(400u);
        if (!wait_runs(High::runs, r, one_ms)) {
            ++missed;
            flush();
        }
        const bool ticked = before.moved();
        if (i > 0u) {
            latency[i - 1u] = High::entered - edge_last;
            lengthened[i - 1u] = ticked;
        }
    }
    // The events that something lengthened are told apart from the
    // fastest, not from the model's count: the count is a cost model of
    // the listing (on Hazard3 3 cycles under the fastest event, on the
    // Cortex-M33 23 over it), and a transfer's cost in that model
    // is uncertain by a cycle or two, so an event within the fastest plus
    // 18 (nine taken transfers at two) is the path itself and one beyond
    // that had something else in it.
    uint32_t lo_raw = 0xFFFF'FFFFu;
    for (uint32_t i = 0; i < latency_events; ++i) {
        lo_raw = latency[i] < lo_raw ? latency[i] : lo_raw;
    }
    for (uint32_t i = 0; i < latency_events; ++i) {
        if (latency[i] > lo_raw + 18u) {
            ++slow;
            if (lengthened[i]) {
                ++slow_ticked;
            }
        }
    }
    sort(latency, latency_events);
    const uint32_t lo = latency[0];
    const uint32_t med = latency[latency_events / 2u];
    const uint32_t hi = latency[latency_events - 1u];
    print(serial, "  road 1 over the Console's wait: the interrupt's last statement to High's "
          "first, min ", lo, " median ", med, " max ", hi, " cycles over ", latency_events,
          " events (", missed, " missed); the three slowest ", latency[latency_events - 3u],
          " ", latency[latency_events - 2u], " ", hi, crlf);
    print(serial, "  the path's count ", latency_count, " cycles, its bound ", latency_bound,
          "; events over the fastest + 18: ", slow, ", a kernel tick, an XIP refill or the "
          "console line's handler inside ", slow_ticked, " of them", crlf);
    bench.verdict("every interrupt's post started High within 1 ms", missed == 0u);
    bench.verdict("every event over the fastest + 18 had a tick, a cache refill or the console "
                  "line's handler in it",
                  slow == slow_ticked);
    bench.verdict("the latency stays under the path's bound (the arithmetic beside it)",
                  hi <= latency_bound);
    reset_scenario();
}

// =============================================================================
// d - the rulers, the scan and the post
// =============================================================================
volatile uint32_t post_quiet = 0;
volatile uint32_t post_road2 = 0;

/// The core's own counter, the alternative ruler: the ticker's composed
/// cycles() on the Cortex-M33, mcycle on Hazard3. Both count clk_sys.
[[gnu::always_inline]] inline uint32_t core_cycles() {
#if defined(BRIO_RP2350_CORE_M33)
    return Ticker::cycles();
#else
    return csr_read<RVCSR_MCYCLE_OFFSET>();
#endif
}

uint32_t core_ruler_cost() {
    uint32_t best = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t a = core_cycles();
        const uint32_t b = core_cycles();
        if (b - a < best) {
            best = b - a;
        }
    }
    return best;
}

void td_scan() {
    reset_scenario();
    tracing = false;
#if !defined(BRIO_RP2350_CORE_M33)
    // mcycle counts only with mcountinhibit.CY clear (3.8.9: set at reset
    // to save power); on for this letter, put back after it.
    const uint32_t inhibit = csr_read<RVCSR_MCOUNTINHIBIT_OFFSET>();
    csr_clear<RVCSR_MCOUNTINHIBIT_OFFSET>(RVCSR_MCOUNTINHIBIT_CY_BITS);
#endif
    const uint32_t ruler = ruler_cost();
    const uint32_t core_ruler = core_ruler_cost();
    // The two rulers over spans of 1.0 to 1.06 ms (a SysTick reload
    // crossed on the Cortex-M33), each pair of reads taken back to back in
    // the same order.
    // Each pair under the platform's mask, so no handler lands between
    // its two reads, and the cache's miss counter read around each pair:
    // a refill inside a pair skews it by the line's cost (measured on the
    // Hazard3 half: a span so lengthened 110 cycles off against the
    // others' 0 to 1), so such a span is reported and left out of the
    // bound, which judges the clean ones. The first span is flagged in
    // every run (its second pair's code is cold after the spin), so
    // spans are taken until four are clean or sixteen have been taken,
    // the lengths stepping through 1.00, 1.02, 1.04 and 1.06 ms: a run
    // whose four spans were all flagged would leave none to judge.
    constexpr uint8_t spans_wanted = 4;
    constexpr uint8_t spans_most = 16;
    uint32_t agree_hi = 0;
    uint8_t spans_clean = 0;
    uint8_t spans_taken = 0;
    uint32_t diffs[spans_most];
    bool clean[spans_most];
    for (uint8_t i = 0; i < spans_most && spans_clean < spans_wanted; ++i) {
        uint32_t c0, r0, c1, r1;
        const uint32_t m0 = xip_misses();
        {
            P::CriticalSection cs;
            c0 = core_cycles();
            r0 = Ruler::now();
        }
        const uint32_t m1 = xip_misses();
        spin(one_ms + 3000u * static_cast<uint32_t>(i % 4u));
        const uint32_t m2 = xip_misses();
        {
            P::CriticalSection cs;
            c1 = core_cycles();
            r1 = Ruler::now();
        }
        const uint32_t m3 = xip_misses();
        const uint32_t dc = c1 - c0;
        const uint32_t dr = r1 - r0;
        diffs[i] = dc > dr ? dc - dr : dr - dc;
        clean[i] = m1 == m0 && m3 == m2;
        ++spans_taken;
        if (clean[i]) {
            ++spans_clean;
            if (diffs[i] > agree_hi) {
                agree_hi = diffs[i];
            }
        }
    }
#if !defined(BRIO_RP2350_CORE_M33)
    csr_set<RVCSR_MCOUNTINHIBIT_OFFSET>(inhibit & RVCSR_MCOUNTINHIBIT_CY_BITS);
#endif
    uint32_t scan = 0xFFFF'FFFFu;
    uint32_t scan_hi = 0;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        const bool served = K::activate();
        const uint32_t t1 = Ruler::now();
        const uint32_t c = t1 - t0 - ruler;
        if (!served && c < scan) {
            scan = c;
        }
        if (!served && c > scan_hi) {
            scan_hi = c;
        }
    }
    uint32_t quiet = 0xFFFF'FFFFu;
    uint32_t road2 = 0xFFFF'FFFFu;
    Mid::action = [](uint8_t) {
        const uint32_t r = ruler_cost();
        const uint32_t t0 = Ruler::now();
        post<Low>(Hit{0});        // below Mid: pushed, nothing asked
        const uint32_t t1 = Ruler::now();
        post<High>(Hit{0});       // above: High's dispatch inside the post
        const uint32_t t2 = Ruler::now();
        post_quiet = t1 - t0 - r;
        post_road2 = t2 - t1 - r;
    };
    for (int i = 0; i < 16; ++i) {
        post<Mid>(Hit{0});
        if (post_quiet < quiet) {
            quiet = post_quiet;
        }
        if (post_road2 < road2) {
            road2 = post_road2;
        }
    }
    reset_scenario();
    print(serial, "  the ruler's two reads (TIMER1 on clk_sys) ", ruler, " cycles; the core's ",
          hazard3 ? "mcycle" : "ticker's composed cycles()", " two reads ", core_ruler,
          " cycles", crlf);
    print(serial, "  over 1.00..1.06 ms the two rulers differ by");
    for (uint8_t i = 0; i < spans_taken; ++i) {
        print(serial, " ", diffs[i], clean[i] ? "" : "*");
    }
    print(serial, " cycles (* a cache refill inside a pair of reads); ", spans_taken,
          " spans taken, at most ", agree_hi, " on the ", spans_clean, " clean ones", crlf);
    print(serial, "  activate() with nothing ready below the Console: ", scan, "..", scan_hi,
          " cycles (four empty queues, the fifth at the ceiling)", crlf);
    print(serial, "  from Mid: a post to Low (asks nothing) ", quiet,
          " cycles; a post to High (road 2, High's dispatch inside) ", road2,
          " cycles; the scheduling's share ", road2 > quiet ? road2 - quiet : 0u, crlf);
    // The rulers. Each pair of reads in the span is taken back to back, so
    // the two deltas differ by the skew between the pairs: at most the
    // slower read's own length plus, on the Cortex-M33, one retry of the
    // composed read's loop when a reload falls between its reads - twice
    // the slower two-read cost, and the ruler's own on top.
    const uint32_t slower = ruler > core_ruler ? ruler : core_ruler;
    bench.verdict("the two rulers agree within twice the slower two-read cost and the ruler's "
                  "over every span whose pairs missed no cache line, at least one (both count "
                  "clk_sys)",
                  spans_clean != 0u && agree_hi <= 2u * slower + ruler);
    // The bounds below are the release listing's count under the file
    // header's cost model, and a fifth of it as the margin.
    // The scan, the call to the stamp after it included. The Cortex-M33:
    // the call (4); activate()'s nine registers, frame, two literals,
    // the ceiling's load and test (19); Top's visit, the queue's two
    // cursors compared in line, and the index's compare with its branch
    // taken (10); High's and Mid's (the queue's address a literal, the
    // two cursors, the compare, the index's compare, two branches not
    // taken: 10 each); Low's, its branch taken on the Console's index
    // (12); the frame's release and the pop of nine with the PC (15):
    // 80, and with the fifth 96. Hazard3: the call (2); cm.push, the
    // setups, the ceiling's load and test (21); Top's visit in line and
    // the index's branch taken (8); High's (6) and Mid's (7); Low's, its
    // branch taken (8); the result's move and the cm.popret of thirteen
    // with its return onto the stamp's load, a 32-bit instruction not
    // 4-aligned (18): 70, and with the fifth 84.
    constexpr uint32_t scan_bound = per_half(96u, 84u);
    // The quiet post: push() and the ceiling's compare ask no queue
    // whether it is empty. The Cortex-M33: the event's store and the
    // call (6); post<Low>()'s prefix (10); the queue's push() - the core
    // check's SIO load, the guard's mrs and cpsid, the cursor's
    // arithmetic, the copy, msr, the pop of the PC (46); the ceiling's
    // compare, its branch and the pop (16): 78, 94 with the fifth.
    // Hazard3: the event's store and the call (4); post<Low>()'s
    // cm.push, the setups and the call (7); push() - the core check, the
    // guard's csrrci, the cursor, the copy, csrsi, the return (30); the
    // ceiling's compare, its branch mispredicted and cm.popret (10): 51,
    // 62 with the fifth.
    constexpr uint32_t quiet_bound = per_half(94u, 62u);
    // The road-2 post, from Mid's ceiling to High's empty dispatch and
    // back. The Cortex-M33: the event's store and the call (6);
    // post<High>()'s prefix and the call to push() (10); push() - the
    // core check, the guard, the cursor, the copy, msr, the pop of the
    // PC (47); the ceiling's compare, its branch not taken and the call
    // (13); the request's store, DSB and ISB (12); PendSV's entry (12),
    // body (17) and return into thread mode (12); the entry (5);
    // activate() at Mid's ceiling - the prologue and Top's visit (29),
    // High's (10), High's serve - the raises and the call to take()
    // (13), take() (19), the slot, the stamp, the count, the stack
    // sample and the trace's test (34), the action's test (7), the
    // trace's again (4), the drop flag and release() (10 + 13), the
    // restore and its branch (9) - the rescan - the served flag and the
    // jump (4), the ceiling and Top's visit (13), High's, empty now, and
    // Mid's index (12) - the frame's release and the pop (15): 192; the
    // entry's FP context cleared, its ISB and the SVC (10), the SVC's
    // entry (12), three instructions (6) and return into the request
    // (12); the request's and the post's returns (8): 374, 449 with the
    // fifth. Hazard3: the event's store and the call (4); post<High>()'s
    // cm.push, the setups and the call to push() (8); push() - the core
    // check, the guard's csrrci, the cursor, the copy, csrsi, the return
    // (30); the ceiling's compare, its branch not taken, cm.pop and the
    // tail jump (11); the request to its spin, one turn (11); the trap -
    // its entry and the table's jump (5), the trampoline's save (19),
    // the body (18), the restore and the `mret` (19); the entry's save
    // (20); activate() at Mid's ceiling - the prologue and the ceiling's
    // test (21), Top's visit and the index's branch (8), High's (5),
    // High's serve - the raises and the call (6), take() (16), the slot,
    // the stamp, the count, the stack sample and the trace's test (22),
    // the action's test (4), the trace's again (2), the drop flag and
    // release() (7 + 11), the restore and its branch (5) - the rescan -
    // the served flag and the jump (4), the ceiling and Top's visit (7),
    // the index's branch (4), High's, empty now, and Mid's index (7) -
    // the result's move and the cm.popret (17): 146; the entry's
    // restore, mstatus and mepc written, the `mret` (25); the spin's
    // last turn and the return (4): 320, 384 with the fifth.
    constexpr uint32_t road2_bound = per_half(449u, 384u);
    print(serial, "  bounds: the scan ", scan_bound, ", a quiet post ", quiet_bound,
          ", a road-2 post ", road2_bound, crlf);
    bench.verdict("the scan of four empty queues, the least of 16, under its bound", scan < scan_bound);
    bench.verdict("a post that asks nothing under its bound (the push and one compare)",
                  quiet < quiet_bound);
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    bench.verdict("and under its bound, High's empty dispatch included", road2 < road2_bound);
}

// =============================================================================
// e - depth 2
// =============================================================================
void te_depth2() {
    const char* const want = "Hs1 Hm1 Ts0 Te0 Hr1 He1";
    // Letter c's path with activate() at High's ceiling, where Top's
    // visit is the take. The Cortex-M33: the handler's tail, the
    // tail-chain, PendSV and the entry (49); activate()'s prologue and
    // the ceiling's test (19), Top's two cursors compared in line with
    // the branch taken (8), the ceiling's and the dispatching index's
    // stores and the call to take() (13), take() (19), the slot and the
    // stamp's literal (6): 114; with a tick (58) and a fifth rounded up
    // (23), 195. Hazard3: the external dispatch's tail, the software
    // interrupt's trap, the trampoline, the body and the entry (111);
    // activate()'s prologue and the ceiling's test (21), Top's two
    // cursors compared in line with the branch taken (5), the two
    // stores and the call (6), take() (16), the slot and the stamp's
    // address (5): 164; with a tick (70) and a fifth rounded up (33),
    // 267.
    constexpr uint32_t depth2_bound = per_half(195u, 267u);
    for (int road = 2; road >= 1; --road) {
        console_drain();
        uint8_t alike = 0;
        uint32_t least = 0xFFFF'FFFFu;
        uint32_t most = 0;
        uint32_t most_clean = 0;   ///< the longest run no tick and no refill lengthened
        uint8_t slow = 0;          ///< runs over the bound
        uint8_t explained = 0;     ///< of them, with a tick or a refill inside
        for (int run = 0; run < 10; ++run) {
            reset_scenario();
            High::action = [](uint8_t) {
                note('H', 'm');
                edge_to = 0;
                edge_in(2000u);
                spin(20000u);
                note('H', 'r');
            };
            const uint32_t r = High::runs;
            // A kernel tick or an XIP refill landing between the edge and
            // Top's first statement lengthens the run: both are read as
            // letter c reads them, and such a run is explained, not judged.
            const Lengtheners before = Lengtheners::now();
            if (road == 2) {
                post<High>(Hit{1});
            } else {
                edge_to = 1;     // the first edge starts High over the Console
                edge_in(400u);
                if (!wait_runs(High::runs, r, one_ms)) {
                    flush();
                }
            }
            const bool lengthened = before.moved();
            const uint32_t d = Top::entered - edge_last;
            if (d < least) {
                least = d;
            }
            if (d > most) {
                most = d;
            }
            if (!lengthened && d > most_clean) {
                most_clean = d;
            }
            if (d > depth2_bound) {
                ++slow;
                if (lengthened) {
                    ++explained;
                }
            }
            if (trace_is(want)) {
                ++alike;
            } else if (alike == run) {
                print_trace();
            }
        }
        print(serial, "  High started by road ", road, ": ", alike, " of 10 runs traced ", want,
              "; Top's first statement ", least, "..", most,
              " cycles after the interrupt's last (bound ", depth2_bound, "); runs over it ",
              slow, ", a kernel tick or an XIP refill inside ", explained, crlf);
        bench.verdict(road == 2 ? "road 1 at depth 2 (High by road 2): Top runs before High "
                                  "resumes (10 runs)"
                                : "road 1 at depth 2 (High by road 1): Top runs before High "
                                  "resumes (10 runs)",
                      alike == 10u);
        bench.verdict("and Top's first statement within its bound of the interrupt's last "
                      "(the arithmetic beside it) in every run nothing lengthened, every "
                      "lengthened one with a tick or a refill inside",
                      most_clean <= depth2_bound && slow == explained);
    }
    reset_scenario();
}

// =============================================================================
// f - time events under preemption
// =============================================================================
using TE = TimeEvents<P>;
TimeEvent<P, High, Hit> te_high{Hit{21}};    // part 1: three due in one turn
TimeEvent<P, Mid, Hit> te_mid{Hit{31}};
TimeEvent<P, Low, Hit> te_low{Hit{41}};      // armed last: walked first
TimeEvent<P, Mid, Hit> te_a{Hit{51}};        // part 2: re-armed by Low, disarmed by High
TimeEvent<P, Mid, Hit> te_b{Hit{52}};        // armed and disarmed by Low: never fires
TimeEvent<P, Mid, Hit> te_c{Hit{53}};        // armed by High, due at once
TimeEvent<P, Mid, Hit> te_d{Hit{54}};        // armed by High, due at once
TimeEvent<P, Mid, Hit> te_m{Hit{55}};        // part 3: the measured arm()

uint8_t fired[64];
volatile uint32_t arm_cycles = 0;
volatile uint32_t low_iteration = 0;          ///< Low's loop, counted
volatile uint32_t high_landed = 0;            ///< the iteration High's dispatch landed in

/// One turn of Rubato::run()'s loop at the Console's level, as run()
/// writes it: the matured time events posted with the ceiling held at 0,
/// then the ceiling back (here the Console's own index, at the base 255)
/// and the scheduler, which serves them in priority order.
void base_turn() {
    const uint8_t c = P::ceiling;
    P::ceiling = 0;
    std::atomic_signal_fence(std::memory_order_seq_cst);
    TE::process();
    std::atomic_signal_fence(std::memory_order_seq_cst);
    P::ceiling = c;
    (void)K::activate();
}

/// Wait until the kernel tick has moved `ticks` on.
void wait_ticks(uint32_t ticks) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ticks) {
    }
}

void tf_timers() {
    reset_scenario();
    TE::clear_all();
    for (uint8_t& f : fired) {
        f = 0;
    }
    High::action = [](uint8_t n) { ++fired[n]; };
    Mid::action = [](uint8_t n) { ++fired[n]; };
    Low::action = [](uint8_t n) { ++fired[n]; };

    // 1. Three matured in one turn, the lowest object's armed last (so
    //    walked first): nothing is dispatched during the walk, and the
    //    scheduler after it serves High, Mid, Low.
    te_high.arm(0);
    te_mid.arm(0);
    te_low.arm(0);
    {
        const uint8_t c = P::ceiling;
        P::ceiling = 0;
        std::atomic_signal_fence(std::memory_order_seq_cst);
        TE::process();
        std::atomic_signal_fence(std::memory_order_seq_cst);
        P::ceiling = c;
    }
    const bool walk_quiet = trace_len == 0u && !High::queue.empty() && !Mid::queue.empty() &&
                            !Low::queue.empty();
    (void)K::activate();
    const bool in_order = trace_is("Hs1 He1 Ms2 Me2 Ls3 Le3");
    if (!in_order) {
        print_trace();
    }
    const bool part1 = fired[21] == 1u && fired[31] == 1u && fired[41] == 1u;

    // 2. Low's handler arms and disarms in a loop, most of each turn inside
    //    arm()'s or disarm()'s masked window; the alarm lands there and its
    //    High preempts at the window's end, arming two timers and
    //    disarming the one Low keeps re-arming.
    trace_clear();
    low_iteration = 0;
    high_landed = 0;
    Low::action = [](uint8_t n) {
        if (n != 60u) {
            ++fired[n];
            return;
        }
        note('L', 'm');
        edge_to = 1;
        edge_in(3000u);          // inside the loop's first tenth: 200 turns of three list verbs
        for (uint32_t i = 0; i < 200u; ++i) {
            low_iteration = i;
            te_a.arm(3);
            te_b.arm(3);
            te_b.disarm();
        }
        note('L', 'r');
    };
    High::action = [](uint8_t n) {
        if (n != 1u) {
            ++fired[n];
            return;
        }
        high_landed = low_iteration;
        te_c.arm(0);
        te_a.disarm();           // Low's next turn arms it again
        te_d.arm(0);
    };
    post<Low>(Hit{60});
    const bool preempted = trace_is("Ls3 Lm3 Hs1 He1 Lr3 Le3");
    if (!preempted) {
        print_trace();
    }
    High::action = [](uint8_t n) { ++fired[n]; };
    Low::action = [](uint8_t n) { ++fired[n]; };
    base_turn();                           // te_c and te_d: due at once
    const bool turn1 = fired[51] == 0u && fired[52] == 0u && fired[53] == 1u && fired[54] == 1u;
    wait_ticks(4u);                        // te_a's three ticks, and one more
    base_turn();
    const bool turn2 = fired[51] == 1u && fired[52] == 0u && fired[53] == 1u && fired[54] == 1u;
    base_turn();
    const bool turn3 = fired[51] == 1u && fired[52] == 0u && fired[53] == 1u && fired[54] == 1u;
    const bool empty = !TE::next_deadline().has_value();

    // 3. One arm() timed from the Console, the least of 16: the masked
    //    window is inside it.
    const uint32_t r = ruler_cost();
    uint32_t least = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        te_m.arm(1000);
        const uint32_t t1 = Ruler::now();
        if (t1 - t0 - r < least) {
            least = t1 - t0 - r;
        }
    }
    arm_cycles = least;
    te_m.disarm();
    TE::clear_all();
    reset_scenario();

    print(serial, "  part 1: fires High ", fired[21], ", Mid ", fired[31], ", Low ", fired[41],
          "; part 2: High landed in Low's turn ", high_landed, " of 200; fires a ", fired[51],
          ", b ", fired[52], ", c ", fired[53], ", d ", fired[54], "; one arm() ", arm_cycles,
          " cycles", crlf);
    bench.verdict("three timers matured in one turn: nothing dispatched during the walk "
                  "(the ceiling held at 0), every one queued",
                  walk_quiet);
    bench.verdict("and the scheduler after the walk served them highest first, once each",
                  in_order && part1);
    bench.verdict("High preempted Low's arm/disarm loop by road 1, inside it",
                  preempted && high_landed < 199u);
    bench.verdict("the next turn: the two High armed fired, the one it disarmed (re-armed by "
                  "Low for three ticks) and the one Low disarmed did not",
                  turn1);
    bench.verdict("three ticks on: the re-armed one fired once; a turn more: nothing "
                  "twice, the list empty",
                  turn2 && turn3 && empty);
    // arm() over a list of one node (itself, re-armed), by the release
    // listing. The Cortex-M33: the arguments and the call (7); the
    // frame, mrs and cpsid, the call to the unlink (14); the unlink, out
    // of line - the head's load, the walk's test, the two stores of the
    // unlink and the node's two, the return (23); now()'s load, the
    // deadline and the node's stores, the link at the list's head, the
    // tail's test, msr, the pop of the PC (34): 78, the masked window the
    // 60 from cpsid to msr; 94 with the fifth. Hazard3: the arguments and
    // the call (4); cm.push, two spills, csrrci, the call (8); the unlink
    // (14); the deadline, the node's stores, the link, csrsi, cm.popret
    // (22): 48, the masked window the 38 from csrrci to csrsi; 58 with
    // the fifth.
    constexpr uint32_t arm_bound = per_half(94u, 58u);
    bench.verdict("one arm(), the least of 16, under its bound, its masked window inside",
                  arm_cycles < arm_bound);
}

// =============================================================================
// g - the request under the mask
// =============================================================================
volatile bool pend_inside = true;       ///< the request read pending inside every section
volatile bool ran_inside = false;       ///< the receiver's runs moved inside a section
volatile uint32_t section_last = 0;     ///< the ruler at a section's last statement

/// The request's pending bit: PendSV's in ICSR on one half, the hart's
/// RISCV_SOFTIRQ flag on the other - read at the SIO rather than in mip,
/// which a load after the raising store reads with the store done.
[[gnu::always_inline]] inline bool request_pending() {
#if defined(BRIO_RP2350_CORE_M33)
    return (SCB->ICSR & SCB_ICSR_PENDSVSET_Msk) != 0u;
#else
    return (riscv_softirq() & (1UL << core_id())) != 0u;
#endif
}

/// The section a post is made in: masked by the platform's own critical
/// section, the post, the marks, the ruler last; the section's end is
/// the guard's restore, and the 'a' mark is the statement after it.
template <class Receiver>
[[gnu::always_inline]] inline void post_in_section(char who) {
    const uint32_t r = Receiver::runs;
    {
        P::CriticalSection cs;
        note(who, 'm');
        post<Receiver>(Hit{1});
        const bool pending = request_pending();
        note(who, 'p');
        pend_inside = pend_inside && pending;
        ran_inside = ran_inside || Receiver::runs != r;
        section_last = Ruler::now();
    }
    note(who, 'a');
}

void tg_mask() {
    console_drain();
    pend_inside = true;
    ran_inside = false;
    // From the Console's dispatch (ceiling 4) to Mid, then from Low's (3)
    // to High: the receiver's entry follows the section's last mark and
    // precedes the mark after the section.
    const char* const want_console = "Cm4 Cp4 Ms2 Me2 Ca4";
    const char* const want_low = "Ls3 Lm3 Lp3 Hs1 He1 La3 Le3";
    uint8_t alike_console = 0;
    uint8_t alike_low = 0;
    constexpr uint32_t section_bound = per_half(168u, 186u);   // derived below the loop
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    uint32_t most_clean = 0;   ///< the longest run no tick and no refill lengthened
    uint8_t slow = 0;          ///< runs over the bound
    uint8_t explained = 0;     ///< of them, with a tick or a refill inside
    bool back = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        // A tick that matures inside the section runs its handler at the
        // unmask, before the scheduler's trap (it outranks it on both
        // halves), and an XIP refill on the path adds its lines: both are
        // read as letter c reads them, and such a run is explained, not
        // judged against the warm bound (measured on the Cortex-M33: one
        // run of ten at 1275 cycles against 134 for the others).
        const Lengtheners before = Lengtheners::now();
        post_in_section<Mid>('C');
        {
            const bool lengthened = before.moved();
            const uint32_t d = Mid::entered - section_last;
            least = d < least ? d : least;
            most = d > most ? d : most;
            if (!lengthened && d > most_clean) {
                most_clean = d;
            }
            if (d >= section_bound) {
                ++slow;
                if (lengthened) {
                    ++explained;
                }
            }
        }
        if (trace_is(want_console)) {
            ++alike_console;
        } else if (alike_console == run) {
            print_trace();
        }
        reset_scenario();
        Low::action = [](uint8_t) { post_in_section<High>('L'); };
        post<Low>(Hit{1});
        if (trace_is(want_low)) {
            ++alike_low;
        } else if (alike_low == run) {
            print_trace();
        }
        back = back && P::ceiling == console_index && P::interrupts_enabled();
    }
    reset_scenario();
    print(serial, "  from the Console to Mid: ", alike_console, " of 10 runs traced ",
          want_console, crlf);
    print(serial, "  from Low to High: ", alike_low, " of 10 runs traced ", want_low, crlf);
    // From the stamp's load to Mid's, by the release listing. The
    // Cortex-M33: the stamp's store with its literal (3), the saved
    // mask's load and the guard's msr that lifts the mask (4); PendSV's
    // entry from thread mode (12), its body (17) and its return into
    // thread mode (12); the entry (5); activate() at the Console's
    // ceiling - the prologue, the ceiling's test and Top's visit with
    // the index's branch (29), High's (10), Mid's with its branch taken
    // (10), Mid's serve to the stamp (13 + 19 + 6: 38): 140, 168 with
    // the fifth. Hazard3: the stamp's store, the guard's test and the
    // csrsi that lifts the mask (4), the trap taken at once on the
    // instruction after it (the pending bit has stood since the post;
    // letter k measures the raise's own latency); the trap, the
    // trampoline, the body, the restore (61); the entry (20); activate()
    // at the Console's ceiling - the prologue and the ceiling's test
    // (21), Top's visit with the index's branch (8), High's (6), Mid's
    // with its branch taken (7), Mid's serve to the stamp (7 + 16 + 5:
    // 28): 155, 186 with the fifth.
    print(serial, "  the section's last statement to Mid's first: ", least, "..", most,
          " cycles (bound ", section_bound, "); runs at or over it ", slow,
          ", a kernel tick or an XIP refill inside ", explained, crlf);
    bench.verdict(hazard3 ? "the hart's software interrupt flag stood raised inside every "
                            "section, and no receiver ran inside one"
                          : "PendSV stood pending inside every section, and no receiver ran "
                            "inside one",
                  pend_inside && !ran_inside);
    bench.verdict("the Console's post to Mid under the mask is served when the section ends, "
                  "before the statement after it (10 runs)",
                  alike_console == 10u);
    bench.verdict("Low's post to High under the mask likewise, inside Low's dispatch (10 runs)",
                  alike_low == 10u);
    bench.verdict("the section's end to the receiver's first statement under its bound "
                  "(the arithmetic beside it) in every run nothing lengthened, every "
                  "lengthened one with a tick or a refill inside",
                  most_clean < section_bound && slow == explained);
    bench.verdict("the ceiling is the Console's again, interrupts enabled", back);
}

#if defined(BRIO_RP2350_CORE_M33)
// =============================================================================
// j - the FPU across a preemption (the Cortex-M33)
// =============================================================================
/// A float series with a mark inside the loop: at i == mark the edge is
/// armed and the loop spins with every accumulator live. No call inside
/// the loop, so the accumulators can stay in s0-s15, the registers lazy
/// stacking covers.
[[gnu::noinline]] float series_low(uint32_t n, uint32_t mark) {
    float a = 0.0f;
    float b = 1.0f;
    float c = 0.5f;
    for (uint32_t i = 0; i < n; ++i) {
        a += b / static_cast<float>(i + 1u);
        b = b * 0.999f + c;
        c = c * 0.5f + 0.25f;
        if (i == mark) {
            edge_to = 1;
            edge_in(500u);
            spin(20000u);
        }
    }
    return a + b + c;
}

[[gnu::noinline]] float series_high(uint32_t n) {
    float a = 0.0f;
    float b = 3.0f;
    for (uint32_t i = 0; i < n; ++i) {
        a += b * static_cast<float>(i) / 7.0f;
        b = b * 0.75f + 1.0f;
    }
    return a + b;
}

volatile float low_sum = 0.0f;
volatile float high_sum = 0.0f;

void tj_fpu() {
    // The lazy context save the core stratum leans on (cortexm/
    // schedule.hpp, "THE FPU") is FPCCR's: ASPEN makes an FP instruction
    // mark the context active so an exception reserves its words, LSPEN
    // defers their save. ARM's architecture resets both to 1 in the
    // Secure state; this chip's datasheet (FPCCR's table, M33 registers)
    // lists both at 0 - the read below says which the silicon holds after
    // the bootrom.
    const uint32_t fpccr = FPU->FPCCR;
    const bool aspen = (fpccr & FPU_FPCCR_ASPEN_Msk) != 0u;
    const bool lspen = (fpccr & FPU_FPCCR_LSPEN_Msk) != 0u;
    print(serial, "  FPCCR ", hex(fpccr), ": ASPEN ", aspen ? 1 : 0, ", LSPEN ", lspen ? 1 : 0,
          crlf);
    bench.verdict("FPCCR.ASPEN and LSPEN are set: the FP context is stacked, lazily, by every "
                  "exception taken over FP code",
                  aspen && lspen);

    reset_scenario();
    tracing = false;
    const float ref_low = series_low(400u, 0xFFFF'FFFFu);
    const float ref_high = series_high(300u);
    const float ref_edge = 2.0f * 1.5f + 1.0f;
    Low::action = [](uint8_t) { low_sum = series_low(400u, 200u); };
    High::action = [](uint8_t) { high_sum = series_high(300u); };
    bool exact = true;
    bool edge_exact = true;
    bool ran = true;
    for (int run = 0; run < 2; ++run) {
        edge_float = run == 1;
        edge_acc = 2.0f;
        low_sum = 0.0f;
        high_sum = 0.0f;
        const uint32_t h = High::runs;
        post<Low>(Hit{1});
        ran = ran && High::runs == h + 1u;
        const bool ok = std::bit_cast<uint32_t>(static_cast<float>(low_sum)) ==
                            std::bit_cast<uint32_t>(ref_low) &&
                        std::bit_cast<uint32_t>(static_cast<float>(high_sum)) ==
                            std::bit_cast<uint32_t>(ref_high);
        exact = exact && ok;
        if (run == 1) {
            edge_exact = std::bit_cast<uint32_t>(static_cast<float>(edge_acc)) ==
                         std::bit_cast<uint32_t>(ref_edge);
        }
        print(serial, "  run ", run, run == 1 ? " (the handler's float too)" : "", ": Low ",
              hex(std::bit_cast<uint32_t>(static_cast<float>(low_sum))), " (",
              hex(std::bit_cast<uint32_t>(ref_low)), "), High ",
              hex(std::bit_cast<uint32_t>(static_cast<float>(high_sum))), " (",
              hex(std::bit_cast<uint32_t>(ref_high)), ")", crlf);
    }
    edge_float = false;
    reset_scenario();
    bench.verdict("High preempted Low's series once per run", ran);
    bench.verdict("Low's and High's sums are bit-exact across the preemption (thread mode)",
                  exact);
    bench.verdict("and the handler's own float, computed over Low's live registers, is exact",
                  edge_exact);
}
#else
// =============================================================================
// k - the software interrupt's own facts (Hazard3)
// =============================================================================
volatile uint32_t sled[8];             ///< part 1: the stores behind the raise
volatile uint32_t sled_seen = 0;       ///< how many Top found stored
volatile uint32_t top_runs_k = 0;

/// Top's action in part 1: count the sled's stores already done.
void count_sled(uint8_t) {
    uint32_t n = 0;
    for (const volatile uint32_t& s : sled) {
        n += s != 0u ? 1u : 0u;
    }
    sled_seen = n;
    top_runs_k = top_runs_k + 1u;
}

void sled_clear() {
    for (volatile uint32_t& s : sled) {
        s = 0u;
    }
}

/// Top's event queued with no request (the ceiling held at 0 across the
/// post), so that the next raise of the flag is what serves it.
void queue_top_quietly() {
    const uint8_t c = P::ceiling;
    P::ceiling = 0;
    std::atomic_signal_fence(std::memory_order_seq_cst);
    post<Top>(Hit{0});
    std::atomic_signal_fence(std::memory_order_seq_cst);
    P::ceiling = c;
}

/// The raw raise: the store to CORE0_SET, then eight stores of 1 into the
/// sled, nothing between them.
[[gnu::noinline]] void raise_then_sled() {
    __asm__ volatile(
        "sw %[bit], 0(%[reg])\n\t"
        "sw %[one], 0(%[s])\n\t"  "sw %[one], 4(%[s])\n\t"  "sw %[one], 8(%[s])\n\t"
        "sw %[one], 12(%[s])\n\t" "sw %[one], 16(%[s])\n\t" "sw %[one], 20(%[s])\n\t"
        "sw %[one], 24(%[s])\n\t" "sw %[one], 28(%[s])\n\t"
        ::[bit] "r"(1UL), [reg] "r"(riscv_softirq_address), [one] "r"(1UL), [s] "r"(&sled[0])
        : "memory");
}

/// The request, then the same eight stores.
[[gnu::noinline]] void request_then_sled() {
    request_scheduler();
    __asm__ volatile(
        "sw %[one], 0(%[s])\n\t"  "sw %[one], 4(%[s])\n\t"  "sw %[one], 8(%[s])\n\t"
        "sw %[one], 12(%[s])\n\t" "sw %[one], 16(%[s])\n\t" "sw %[one], 20(%[s])\n\t"
        "sw %[one], 24(%[s])\n\t" "sw %[one], 28(%[s])\n\t"
        ::[one] "r"(1UL), [s] "r"(&sled[0])
        : "memory");
}

/// Fifteen caller-saved registers (t0..t6, a0..a7) loaded with patterns,
/// a spin with them live until `*done` is non-zero (at most `limit`
/// turns), then the fifteen stored into `out`. True when every pattern
/// came back and the spin ended on `done`.
constexpr uint32_t pattern_base = 0x5A00'0000u;
[[gnu::noinline]] bool registers_survive(const volatile uint32_t* done, uint32_t limit,
                                         uint32_t salt) {
    uint32_t out[16];
    __asm__ volatile(
        "mv s1, %[done]\n\t"
        "mv s2, %[limit]\n\t"
        "mv s4, %[out]\n\t"
        "li t0, 0x00000001\n\t"  "add t0, t0, %[salt]\n\t"
        "li t1, 0x00000102\n\t"  "add t1, t1, %[salt]\n\t"
        "li t2, 0x00000203\n\t"  "add t2, t2, %[salt]\n\t"
        "li t3, 0x00000304\n\t"  "add t3, t3, %[salt]\n\t"
        "li t4, 0x00000405\n\t"  "add t4, t4, %[salt]\n\t"
        "li t5, 0x00000506\n\t"  "add t5, t5, %[salt]\n\t"
        "li t6, 0x00000607\n\t"  "add t6, t6, %[salt]\n\t"
        "li a0, 0x00000708\n\t"  "add a0, a0, %[salt]\n\t"
        "li a1, 0x00000809\n\t"  "add a1, a1, %[salt]\n\t"
        "li a2, 0x0000090a\n\t"  "add a2, a2, %[salt]\n\t"
        "li a3, 0x00000a0b\n\t"  "add a3, a3, %[salt]\n\t"
        "li a4, 0x00000b0c\n\t"  "add a4, a4, %[salt]\n\t"
        "li a5, 0x00000c0d\n\t"  "add a5, a5, %[salt]\n\t"
        "li a6, 0x00000d0e\n\t"  "add a6, a6, %[salt]\n\t"
        "li a7, 0x00000e0f\n\t"  "add a7, a7, %[salt]\n\t"
        "1:\n\t"
        "lw s3, 0(s1)\n\t"
        "bnez s3, 2f\n\t"
        "addi s2, s2, -1\n\t"
        "bnez s2, 1b\n"
        "2:\n\t"
        "sw t0, 0(s4)\n\t"  "sw t1, 4(s4)\n\t"  "sw t2, 8(s4)\n\t"  "sw t3, 12(s4)\n\t"
        "sw t4, 16(s4)\n\t" "sw t5, 20(s4)\n\t" "sw t6, 24(s4)\n\t" "sw a0, 28(s4)\n\t"
        "sw a1, 32(s4)\n\t" "sw a2, 36(s4)\n\t" "sw a3, 40(s4)\n\t" "sw a4, 44(s4)\n\t"
        "sw a5, 48(s4)\n\t" "sw a6, 52(s4)\n\t" "sw a7, 56(s4)\n\t" "sw s3, 60(s4)\n\t"
        ::[done] "r"(done), [limit] "r"(limit), [out] "r"(&out[0]), [salt] "r"(salt)
        : "t0", "t1", "t2", "t3", "t4", "t5", "t6", "a0", "a1", "a2", "a3", "a4", "a5", "a6",
          "a7", "s1", "s2", "s3", "s4", "memory");
    bool same = out[15] != 0u;
    for (uint32_t i = 0; i < 15u; ++i) {
        const uint32_t want = (0x0001u + 0x0101u * i) + salt;
        same = same && out[i] == want;
    }
    return same;
}

volatile uint32_t high_done = 0;       ///< part 2: High's dispatch ran
volatile uint32_t top_done = 0;        ///< part 2: Top's dispatch ran
volatile bool high_kept = false;       ///< High's registers across Top's nested entry
volatile uint32_t in_flag = 1;         ///< part 3: the flag as High's dispatch found it
volatile uint32_t in_mip = 1;          ///< mip.MSIP likewise
volatile uint32_t in_mie = 0;          ///< mstatus.MIE likewise
volatile uint32_t in_served = 0;       ///< mscratch likewise

void tk_softirq() {
    reset_scenario();
    tracing = false;
    // 1. The raw raise and the request, ten of each.
    uint32_t raw_lo = 0xFFFF'FFFFu;
    uint32_t raw_hi = 0;
    uint32_t req_hi = 0;
    bool each_once = true;
    Top::action = count_sled;
    for (int run = 0; run < 10; ++run) {
        sled_clear();
        sled_seen = 0xFFu;
        uint32_t t = top_runs_k;
        queue_top_quietly();
        raise_then_sled();
        each_once = each_once && top_runs_k == t + 1u;
        raw_lo = sled_seen < raw_lo ? sled_seen : raw_lo;
        raw_hi = sled_seen > raw_hi ? sled_seen : raw_hi;
        sled_clear();
        sled_seen = 0xFFu;
        t = top_runs_k;
        queue_top_quietly();
        request_then_sled();
        each_once = each_once && top_runs_k == t + 1u;
        req_hi = sled_seen > req_hi ? sled_seen : req_hi;
    }
    Top::action = nullptr;
    print(serial, "  the raw raise: ", raw_lo, "..", raw_hi,
          " of the eight stores behind it ran before the trap; behind request_scheduler(): ",
          req_hi, crlf);
    // The raw figure is the bus's fact the request does not depend on (rp2350/
    // schedule_hazard3.hpp): printed, and judged by nothing but the run
    // itself - every raise served once. The guarded figure is the
    // request's promise.
    bench.verdict("each raise served Top exactly once (20 raises)", each_once);
    bench.verdict("request_scheduler() returns after the scheduler it started: no store "
                  "behind it ran first (10 runs)",
                  req_hi == 0u);

    // 2. and 3. The registers across an entry and a nested one, and what a
    //    dispatch the software interrupt started finds.
    console_drain();
    uint8_t outer_kept = 0;
    uint8_t inner_kept = 0;
    bool flag_low = true;
    bool mip_low = true;
    bool mie_set = true;
    bool served_once = true;
    High::action = [](uint8_t) {
        in_flag = riscv_softirq() & (1UL << core_id());
        in_mip = csr_read<RVCSR_MIP_OFFSET>() & RVCSR_MIP_MSIP_BITS;
        in_mie = csr_read<RVCSR_MSTATUS_OFFSET>() & mstatus_mie;
        in_served = csr_read<RVCSR_MSCRATCH_OFFSET>();
        // Top over High: the alarm lands while High's registers are live.
        top_done = 0;
        edge_to = 0;
        edge_in(400u);
        high_kept = registers_survive(&top_done, one_ms, 0x0010'0000u);
        high_done = 1;
    };
    Top::action = [](uint8_t) { top_done = 1; };
    for (int run = 0; run < 10; ++run) {
        high_done = 0;
        high_kept = false;
        const uint32_t served0 = csr_read<RVCSR_MSCRATCH_OFFSET>();
        edge_to = 1;
        edge_in(400u);
        const bool outer = registers_survive(&high_done, one_ms, pattern_base);
        outer_kept = static_cast<uint8_t>(outer_kept + (outer ? 1u : 0u));
        inner_kept = static_cast<uint8_t>(inner_kept + (high_kept ? 1u : 0u));
        flag_low = flag_low && in_flag == 0u;
        mip_low = mip_low && in_mip == 0u;
        mie_set = mie_set && in_mie != 0u;
        served_once = served_once && in_served == served0 + 1u;
    }
    reset_scenario();
    print(serial, "  registers kept: the Console's across High's entry ", outer_kept,
          " of 10, High's across Top's nested entry ", inner_kept, " of 10", crlf);
    bench.verdict("fifteen caller-saved registers found again after an entry, the Console's "
                  "(10 runs)",
                  outer_kept == 10u);
    bench.verdict("and after a nested entry, High's under Top's (10 runs)", inner_kept == 10u);
    bench.verdict("inside the dispatch it started, the flag already lowered and mip.MSIP clear",
                  flag_low && mip_low);
    bench.verdict("the dispatch runs with mstatus.MIE set, and the served count moved by one "
                  "for its entry",
                  mie_set && served_once);
}
#endif

// =============================================================================
// i - dark
// =============================================================================
void ti_dark() {
    reset_scenario();
    bench.verdict("Top, High, Mid and Low are idle: every queue empty",
                  Top::queue.empty() && High::queue.empty() && Mid::queue.empty() &&
                      Low::queue.empty());
    bench.verdict("no time event is armed", !TE::next_deadline().has_value());
    bench.verdict("TIMER1's alarm 0 is disarmed and its line quiet",
                  !Edge::armed<0>() && !Edge::raised<0>() && !Irq::pending(Edge::irq<0>()));
    bench.verdict("the LED's pad GP25 is not an output", !Led::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
#if defined(BRIO_RP2350_CORE_M33)
    const uint32_t prio = NVIC_GetPriority(PendSV_IRQn);
    print(serial, "  PendSV's priority ", prio, " of 0..15", crlf);
    bench.verdict("PendSV at the lowest of the sixteen levels and not pending",
                  prio == 15u && !request_pending());
#else
    bench.verdict("mie.MSIE set and the hart's software interrupt flag low",
                  (csr_read<RVCSR_MIE_OFFSET>() & RVCSR_MIE_MSIE_BITS) != 0u &&
                      !request_pending());
#endif
}

// =============================================================================
// h - the stack
// =============================================================================
constexpr uint32_t paint = 0xC5AC'CE55u;
uint32_t paint_top = 0;      ///< the highest painted word's address + 4
uint32_t paint_bottom = 0;   ///< the lowest painted word's address

void th_stack() {
    uint32_t lowest = paint_top;
    for (uint32_t a = paint_bottom; a < paint_top; a += 4u) {
        if (*reinterpret_cast<const uint32_t*>(a) != paint) {
            lowest = a;
            break;
        }
    }
    const uint32_t used = paint_top - lowest;
    const uint32_t deepest_dispatch = paint_top - sp_low_dispatch;
    const uint32_t deepest_isr = paint_top - sp_low_isr;
    print(serial, "  painted ", paint_top - paint_bottom, " bytes below main's frame (",
          hex(paint_bottom), "..", hex(paint_top), "); used ", used,
          " bytes; the deepest stack pointer sampled: a dispatch at ", deepest_dispatch,
          ", the interrupt at ", deepest_isr, crlf);
    // A dispatch's sample is taken inside activate(), whose saved
    // registers and frame are above it (64 bytes on the Cortex-M33, 96 on
    // Hazard3). Below the deepest one lies at most, by the release
    // listing: the letter's action and what it calls (the Cortex-M33's
    // widest, letter d's, 32 bytes calling a post, 8, and the queue's
    // push, 8 - 48, taken as 64 for the FPU letter's series; Hazard3's,
    // letter k's High action calling registers_survive(), 16 and 96:
    // 112); then an interrupt landing there - on the Cortex-M33 an
    // extended frame of 26 words and its alignment word (108), the FPU
    // letter's context being active, then either the deepest handler's
    // own frames (the alarm's 16, its post's 8 and the push's 8: 32) or,
    // the handler gone and its request pending, PendSV's two words and an
    // activate() that finds nothing (8 + 64), with a basic frame and a
    // handler over that (36 + 32): 64 + 108 + 140 = 312; on Hazard3 the
    // trampoline's 64 and the handler's frames (the alarm's 32, its
    // post's 16: 48), or, the handler gone, the entry's 80 and an
    // activate() that finds nothing (96) with an interrupt over that
    // (64 + 48): 112 + 288 = 400.
    constexpr uint32_t below = per_half(312u, 400u);
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    print(serial, "  the bound below the deepest sample ", below, " bytes", crlf);
    bench.verdict("the paint reached the deepest sampled stack pointer", used >= deepest);
    bench.verdict("the high-water mark is within the bound of the deepest sample (the arithmetic "
                  "beside it)",
                  used <= deepest + below);
    bench.verdict("and inside the 4 KB bank, with the paint's floor untouched",
                  lowest > paint_bottom);
}

// =============================================================================
// the Console's turn
// =============================================================================
bool clock_ok = false;
bool tick_ok = false;
bool ruler_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED", " tick=",
          tick_ok ? (hazard3 ? "mtime" : "SysTick") : "FAILED", " ruler=",
          ruler_ok ? "TIMER1" : "FAILED", crlf);
    print(serial, "test_rp2350_rubato - the preemptive kernel on the RP2350's ",
          hazard3 ? "Hazard3" : "Cortex-M33", ", the scheduler in thread mode", crlf);
    bench.menu();
}

void console_turn() {
    uint8_t c = 0;
    while (Serial::read_byte(c)) {
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            console_banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES, for every vector but the scheduler's, which
// the macro binds for the half the image is built for.
extern "C" void isr_uart0() {
    uart_interrupts = uart_interrupts + 1u;
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void isr_systick() { brio::Ticker::tick(); }

BRIO_RP2350_SCHEDULER_VECTORS(K)

/// TIMER1's alarm 0: the raised flag cleared, the post, and the ruler as
/// the last statement - letter c's near end.
extern "C" void isr_timer1_0() {
    Edge::clear<0>();
    sample_sp(sp_low_isr);
    edge_hits = edge_hits + 1u;
#if defined(BRIO_RP2350_CORE_M33)
    if (edge_float) {
        edge_acc = edge_acc * 1.5f + 1.0f;
    }
#endif
    if (edge_to == 0u) {
        brio::post<Top>(Hit{1});
    } else {
        brio::post<High>(Hit{1});
    }
    edge_last = Ruler::now();
}

/// Core 0's stack: the whole of the upper unstriped bank, scratch_y, 4 KB
/// (rp2350/ld/rp2350_16m.ld), its top the linker's symbol.
extern "C" uint32_t __stack_top;
constexpr uint32_t stack_bank_bytes = 4096u;

int main() {
    // Paint the free stack: from 64 bytes below main's own stack pointer
    // down to the bank's floor, 64 bytes of it left as a guard.
    {
        const uint32_t sp = stack_pointer();
        const uint32_t top = reinterpret_cast<uint32_t>(&__stack_top);
        paint_bottom = top - stack_bank_bytes + 64u;
        paint_top = (sp - 64u) & ~3u;
        for (uint32_t a = paint_bottom; a < paint_top; a += 4u) {
            *reinterpret_cast<volatile uint32_t*>(a) = paint;
        }
    }

    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    ruler_ok = Edge::init(clock);
    Edge::source(brio::TimerSource::sysclk);
    Edge::interrupt<0>(true);
    brio::Irq::enable(Edge::irq<0>());
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", ta_ceiling);
    bench.letter('b', "road 2: the chain inside the posts", tb_road2);
    bench.letter('c', "the latency of road 1", tc_latency);
    bench.letter('d', "the rulers, the scan and the post, in cycles", td_scan);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", te_depth2);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", tf_timers);
    bench.letter('g', "the request under the mask: served when the section ends", tg_mask);
#if defined(BRIO_RP2350_CORE_M33)
    bench.letter('j', "the FPU's registers across a preemption, FPCCR's lazy save", tj_fpu);
#else
    bench.letter('k', "the software interrupt: its latency, the registers, the flag", tk_softirq);
#endif
    bench.letter('i', "dark: every object idle, nothing lit, the request as set", ti_dark);
    bench.letter('h', "the stack's high-water mark", th_stack);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
