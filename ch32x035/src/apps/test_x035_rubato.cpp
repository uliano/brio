// test_x035_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the CH32X035: Ch32x035RubatoPlatform and the
// QingKe V4C's scheduling request (ch32x035/schedule.hpp), against the
// model the host suite specifies (test/test_rubato).
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
// NOTHING TO WIRE, NOTHING LIT. The console is the probe's own serial
// (USART2 on PA2/PA3); the suite configures no other pad, and the LED's
// pad PA0 stays undriven (letter i checks it). The interrupt every road-1
// letter lands is TIM2's capture-compare line (TIM2 is an advanced timer
// on this chip, with four lines of its own; ch32x035/tim.hpp), on TIM2's
// clock, which is HCLK here (the static_assert below), so a delay placed
// in its counts is a delay in core cycles. The core has no floating-point
// unit (RV32IMAC), so there is no float letter.
//
// THE RULER is the STK counter itself, read bare: CNTL counts HCLK and
// restarts at the ticker's compare every millisecond, so two reads less
// than a period apart are a cycle count with the restart folded in
// (`between()` below) - one load. The ticker's composed cycles()
// (ch32x035/ticker.hpp: the tick count and the counter read under a
// consistency check) is the other ruler weighed; letter d prints both
// costs and checks one against the other, and the composed one times only
// the waits longer than a period. TIM2 is not the console's (USART2) nor
// the ticker's (STK).
//
// THE CODE RUNS FROM THE FLASH, behind two wait states at 48 MHz with no
// prefetch buffer: a fetch reads one word, and a taken branch costs what
// its target's address modulo four makes it (docs/ch32x035/platform.md,
// "A handler in SRAM"). Every bound below is drawn over this image's
// release listing with that fetch model (letter c states it).
//
// WHERE THE SCHEDULER RUNS is a constant of the image,
// `scheduler_in_thread_mode` below, and the letters whose verdict depends
// on it judge against it. Here it is true: the software interrupt returns
// into the scheduler in thread mode and the scheduler leaves through the
// machine-mode environment call (ch32x035/schedule.hpp), so the software
// interrupt is not active while the dispatches run, and an interrupt's
// post preempts the dispatch it lands in at any depth - the Console's own
// dispatch included, since the UART's interrupt starts it at the loop's
// base. The letters also carry what they expect of a scheduler run INSIDE
// the software interrupt's handler (the constant false): there every
// dispatch runs inside that handler, the Console's too, an interrupt's
// request waits for the dispatch it landed in to return and the
// activate() around it to rescan, and road 2 nests only because a request
// made inside the handler calls the scheduler; letters a, c, e, f and g
// show that shape's limit.
//
// The letters (each bound's arithmetic beside its verdict in the code):
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms TIM2 2000 cycles ahead and spins 20000 at a
//      mark; the interrupt posts to High. The trace says where High's
//      dispatch landed - before Low resumes where road 1 is kept, after
//      Low's end in the handler shape - and every entry reads the ceiling
//      at its own object's index; the Console reads its own, 4, after the
//      chain, and the UART's receive interrupt read 255 (the base) when
//      the letter's key arrived. Ten runs, all alike.
//   b  ROAD 2: Low posts to Mid, Mid to High, inside the handlers; High
//      posts to Low. The trace nests High inside Mid inside Low, and
//      Low's second event is served after its first dispatch ends, never
//      inside it. Ten runs, all alike.
//   c  THE LATENCY OF ROAD 1, the Console's wait loop interrupted: the
//      ruler at the interrupt handler's last statement and at High's
//      handler's first, min, median and max over 100 events after one
//      warm-up, the console drained first; the bound beside it. Where road
//      1 is not kept, the letter measures and judges the deferral instead.
//   d  THE RULERS, THE SCAN AND THE POST: the two rulers' two-read costs
//      and their agreement over a span; activate() with nothing ready
//      below the Console's ceiling (four empty queues, the fifth object at
//      the ceiling), and from Mid's handler one post that asks for nothing
//      (to Low, below it) against one that runs High's dispatch inside it
//      (road 2), each the least of 16, the ruler's own cost taken off.
//   e  DEPTH 2: High's handler - started by road 2 (the Console's post),
//      then by road 1 (an interrupt over the Console) - spins at a mark
//      while TIM2 posts to Top. Top's entry lands before High resumes
//      where road 1 is kept at depth 2, after High's end in the handler
//      shape; the latency (or the deferral) printed. Ten runs each.
//   f  TIME EVENTS UNDER PREEMPTION, in three parts. (1) Three one-shots
//      matured in one turn, the lowest object's armed last so walked
//      first, posted the way Rubato::run() posts them - the walk with the
//      ceiling held at 0, then the scheduler: nothing is dispatched during
//      the walk and the three are served highest first. (2) Low's handler
//      runs 200 turns of arm(), arm(), disarm(), most of each turn inside
//      a masked window; TIM2's interrupt lands there and its High
//      preempts at the window's end (in the handler shape: after Low's
//      end), arming two timers and disarming the one Low keeps re-arming:
//      the list stays whole - the next turns fire every timer due once,
//      the disarmed one never, nothing twice. (3) The cycles of one arm(),
//      its masked window inside them. The turns are run()'s, written out
//      in the Console's dispatch (base_turn(); the kernel loop itself is
//      below the Console and does not turn while a letter runs).
//   g  THE REQUEST UNDER THE MASK: a post to a higher object made inside a
//      critical section pends the software interrupt and is NOT served
//      inside the section - the receiver's entry follows the section's
//      last mark - and is served when the section ends, before the
//      statement after it (in the handler shape: when the poster's
//      dispatch has ended); from the Console (to Mid) and from Low's
//      dispatch (to High), ten runs each, with the line's pending bit read
//      inside the section and the cycles from the section's last statement
//      to the receiver's first.
//   i  DARK: every object but the Console idle, no time event armed,
//      TIM2's line quiet, the LED's pad not an output, the ceiling the
//      Console's own, interrupts enabled; and the nesting the platform
//      set: INESTEN, the software interrupt at its priority, every other
//      line at 0.
//   h  THE STACK, the last letter: the crt painted the free RAM from the
//      end of the image's RAM sections to the stack top before the first
//      call; the high-water mark against the deepest stack pointer the
//      dispatches and the interrupt sampled in letters a..g plus what lies
//      below a sample, the arithmetic beside the verdict.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/schedule.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/tim.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

/// Where the scheduler runs in this image: true, in thread mode, the
/// software interrupt returning into it (ch32x035/schedule.hpp); false
/// would state a scheduler run inside the software interrupt's handler
/// (the file header).
constexpr bool scheduler_in_thread_mode = true;

using P = Ch32x035RubatoPlatform<>;
using Edge = Tim<2>;
static_assert(tim_clock_hz(clock) == SysClock::hz,
              "TIM2 counts HCLK here: a delay in its counts is a delay in core cycles");

/// The STK counter read bare: HCLK cycles within the ticker's period.
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return stk()->CNTL; }
};

/// The cycles from `from` to `to`, two bare reads less than a period
/// apart, the restart at the compare folded in.
[[gnu::always_inline]] inline uint32_t between(uint32_t from, uint32_t to) {
    constexpr uint32_t period = SysClock::hz / Ticker::ticks_per_second;
    return to >= from ? to - from : to + period - from;
}

using Serial = Uart<2, P>;
constexpr Serial serial;

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

[[gnu::always_inline]] inline void sample_sp(volatile uint32_t& low) {
    uint32_t sp;
    __asm__ volatile("mv %0, sp" : "=r"(sp));
    if (sp < low) {
        low = sp;
    }
}

/// A busy wait of `cycles` HCLK cycles, under a ticker period.
[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (between(t0, Ruler::now()) < cycles) {
    }
}

/// TIM2's compare `d` counts of HCLK from now, its interrupt armed: one
/// edge, the handler disarms it.
[[gnu::always_inline]] inline void edge_in(uint16_t d) {
    Edge::enable(false);
    Edge::set_count(0);
    Edge::clear_flags(Edge::compare_flag(0));
    (void)Edge::set_compare(0, d);
    Edge::interrupts(Edge::compare_interrupt(0), true);
    Edge::enable(true);
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

void reset_scenario() {
    trace_clear();
    Top::action = nullptr;
    High::action = nullptr;
    Mid::action = nullptr;
    Low::action = nullptr;
    tracing = true;
}

/// Wait until `runs` moves past `before`, at most `cycles` (the ticker's
/// composed cycles, so a wait may outlast a period): true when it did.
bool wait_runs(const volatile uint32_t& runs, uint32_t before, uint32_t cycles) {
    const uint32_t t0 = Ticker::cycles();
    while (runs == before) {
        if (Ticker::cycles() - t0 >= cycles) {
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
/// 115200 are 20 ms) and its last byte leave the wire: its interrupt,
/// landing in a measured path, would be measured.
void console_drain() {
    const uint32_t t0 = Ticker::cycles();
    while (!Serial::tx_idle() && Ticker::cycles() - t0 < SysClock::hz / 20u) {
    }
    spin(SysClock::hz / 10'000u);   // 100 us: the last byte's own 87 on the wire
}

uint32_t ruler_cost() {
    uint32_t best = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t a = Ruler::now();
        const uint32_t b = Ruler::now();
        if (between(a, b) < best) {
            best = between(a, b);
        }
    }
    return best;
}

/// One millisecond of HCLK: the letters' wait for a dispatch that should
/// have run within a few hundred cycles.
constexpr uint32_t one_ms = SysClock::hz / 1000u;

// =============================================================================
// a - the ceiling, road 1 at depth 1
// =============================================================================
void ta_ceiling() {
    print(serial, "  the UART's receive interrupt read the ceiling ", rx_ceiling,
          " when this letter's key arrived", crlf);
    bench.verdict("the base's ceiling is 255 (read by the receive interrupt as the key arrived)",
                  rx_ceiling == 255u);
    bench.verdict("the Console's dispatch reads its own index, 4", P::ceiling == console_index);

    // Where road 1 is kept, High's dispatch lands inside Low's spin; in the
    // handler shape it waits for Low's end and the rescan.
    const char* const want = scheduler_in_thread_mode ? "Ls3 Lm3 Hs1 He1 Lr3 Le3"
                                                      : "Ls3 Lm3 Lr3 Le3 Hs1 He1";
    uint8_t alike = 0;
    bool ceiling_back = true;
    bool once = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        Low::action = [](uint8_t) {
            note('L', 'm');
            edge_to = 1;
            edge_in(2000u);      // 42 us into a 417 us spin: the edge lands inside it
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
    if constexpr (scheduler_in_thread_mode) {
        bench.verdict("High's dispatch starts inside Low's, before Low resumes, every ceiling its "
                      "own index (10 runs)",
                      alike == 10u);
    } else {
        bench.verdict("handler shape: High's dispatch starts after Low's has ended - road 1 does "
                      "not preempt a dispatch inside the trap - every ceiling its own index "
                      "(10 runs)",
                      alike == 10u);
    }
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
[[maybe_unused]] uint32_t latency[latency_events];

[[maybe_unused]] void sort(uint32_t* v, uint32_t n) {
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

/// THE COST MODEL every bound of this suite is drawn with, over this
/// image's release listing (build-cmake/ch32x035f8-release/
/// test_x035_rubato.lst). The code runs from the flash, at two wait states
/// and with no prefetch buffer, and the core waits for its fetch
/// (docs/ch32x035/platform.md, "A handler in SRAM"): straight-line code
/// issues as fast as its words arrive, one four-byte word every 4 cycles -
/// 4 cycles a 32-bit instruction, 2 a compressed one, and a load from SRAM
/// or a peripheral, a store or a CSR access nothing beyond its fetch; a
/// taken transfer (a branch, a jump, a call, a return, an MRET into code,
/// a fence.i) issues its target 8 cycles after it, 12 when the target is a
/// 32-bit instruction across two words; a trap's entry (the store that
/// pends the software interrupt, the unmask that finds it pending, an
/// ecall, an interrupt line) issues the handler's first instruction 20
/// cycles after the instruction that triggers it, and an MRET into a
/// pending interrupt (the tail) 24 after the MRET. Those figures are this
/// core's, and test_x035_platform's letter i measures them - hand-placed
/// sequences timed on the bare STK, each judged to the cycle, every trap
/// the same in 256 trips, the same with the hardware prologue and without
/// it (its push costs nothing measurable here); the fence.i's 8 is the one
/// figure above that letter does not time, and no path below runs one. The
/// platform suite's letter e agrees with them to the cycle (its entry, 36,
/// is the raiser's own 12 and 24 into the handler).
///
/// ONE THING THE MODEL LEAVES OUT: the branch predictor corecfgr's 0x1F
/// turns on (the crt writes WCH's value; docs/ch32x035/platform.md). Its
/// right guess costs nothing against the model, and its wrong one, by the
/// platform suite's letter i, up to 8 cycles at one branch (one guessed
/// taken that falls through) and within 4 a predicted site over every
/// window that letter times, its call and return counted - the allowance
/// below, a sum over a path's sites and not a figure of each: measured on
/// this core, road 1 below took the same
/// cycles in every event with corecfgr cleared and up to 16 more with 0x1F,
/// in steps of 4, as the predictor's history (another code's branches
/// aliasing the path's) stood at each event. So every run's figure is the
/// model's and at most the PREDICTOR'S SPREAD, 4 cycles at each predicted
/// site of the path; and a least of several carries it too, since two of
/// the path's own sites can share an entry and be wrong in every run.
///
/// The road-1 path from the interrupt handler's last statement to High's
/// first, by the listing:
///   TIM2's handler after the stamp's load: a load, a store, the stack's
///   release, its MRET (4 instructions)                               12
///   the tail into the software interrupt, its body to its MRET: the
///   block, mepc and mstatus read and stored, the entry's address into
///   mepc (9)                                                    24 + 28
///   its MRET into the entry; the entry's sixteen stores, csrsi, the call
///   (19)                                                              48
///   activate(): its frame and thirteen stores, the ceiling's load, Top's
///   visit - the call to the queue's out-of-line empty(), seven
///   instructions with its return, the index compare (32)            100
///   High's visit: the call to empty() and its answer (11)             52
///   the ceiling's store, the dispatching flag, the call to take(),
///   take() to its return, the stamp's load (25)                       92
///                                                                    ---
///                                                                    356
/// (101 instructions, 10 taken transfers; the queue's ring wrapping in
/// take() changes the instructions and not the cycles.) On top of it: the
/// predictor's spread at the path's 14 predicted sites (7 conditional
/// branches, 4 calls, 3 returns), 56; and at most one tick landing in the
/// path - the ticker's line is at the peripheral level, so it preempts
/// the software interrupt's handler, the entry and activate(), never
/// TIM2's handler - its entry (20), its thirteen instructions to its MRET
/// (40), its MRET into a 32-bit instruction across two words (12) and its
/// own branch's spread (4), 76. The bound is the three, 488, and a fifth
/// of the path's count, 71: 559. An event more than the spread, 56, over
/// the fastest is one a tick landed in (a tick costs at least 64: its
/// entry, its body and an MRET, less the fetch the interrupted instruction
/// had anyway), and the letter reads the kernel tick around each event to
/// say so.
///
/// AN IMAGE BUILT WITHOUT THE HARDWARE PROLOGUE (CH32X035_HPE=OFF) pays
/// on every path through a handler that saves in software, not in the
/// traps themselves (the transitions measured the same), and its bounds
/// are drawn the same way over that image's listing: TIM2's handler,
/// attributed and saving in software, reloads after the stamp its sixteen
/// integer registers (44 cycles where 12 were); the software interrupt's
/// body saves and reloads its scratch register (two instructions, 56
/// where 52 were); the rest, the same instructions at other addresses,
/// costs the same 292. Road 1 then counts 392 (119 instructions), its
/// spread the same 56, and a tick landing in it 92 (its frame of sixteen
/// saves: 56 cycles of body where 40 were): 540, and 618 with a fifth of
/// the count.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
constexpr bool hpe_build = true;
#else
constexpr bool hpe_build = false;
#endif
constexpr uint32_t latency_count = hpe_build ? 356u : 392u;
constexpr uint32_t latency_bound = hpe_build ? 559u : 618u;
constexpr uint32_t latency_spread = 56u;

void tc_latency() {
    reset_scenario();
    tracing = false;
    if constexpr (scheduler_in_thread_mode) {
        // One warm-up event not counted, then 100; each waits at most 1 ms
        // for High's first statement.
        console_drain();
        uint32_t missed = 0;
        // An event slower than the fastest by more than the spread is one
        // a tick landed in: the kernel tick read around each says so.
        bool ticked_at[latency_events];
        for (uint32_t i = 0; i <= latency_events; ++i) {
            const uint32_t r = High::runs;
            const uint32_t t0 = Ticker::ticks();
            edge_to = 1;
            edge_in(400u);
            if (!wait_runs(High::runs, r, one_ms)) {
                ++missed;
                flush();
            }
            const bool ticked = Ticker::ticks() != t0;
            if (i > 0u) {
                latency[i - 1u] = between(edge_last, High::entered);
                ticked_at[i - 1u] = ticked;
            }
        }
        uint32_t fastest = 0xFFFF'FFFFu;
        for (uint32_t i = 0; i < latency_events; ++i) {
            fastest = latency[i] < fastest ? latency[i] : fastest;
        }
        uint32_t slow = 0;
        uint32_t slow_ticked = 0;
        for (uint32_t i = 0; i < latency_events; ++i) {
            if (latency[i] > fastest + latency_spread) {
                ++slow;
                slow_ticked += ticked_at[i] ? 1u : 0u;
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
              "; events over the fastest + ", latency_spread, ": ", slow,
              ", a kernel tick inside ", slow_ticked, " of them", crlf);
        bench.verdict("every interrupt's post started High within 1 ms", missed == 0u);
        bench.verdict("every event over the fastest by more than the path's spread had a tick "
                      "land in it",
                      slow == slow_ticked);
        bench.verdict("the latency stays under the path's bound (the arithmetic beside it)",
                      hi <= latency_bound);
    } else {
        // The handler shape: the Console's dispatch runs inside the trap,
        // so the interrupt's request waits. 1 ms of wait (48000 cycles, a
        // hundred times the path's own length) and High has not run;
        // the flush serves it, and the deferral is the wait plus the flush.
        uint32_t deferred = 0;
        uint32_t least = 0xFFFF'FFFFu;
        for (uint32_t i = 0; i < 10u; ++i) {
            const uint32_t r = High::runs;
            edge_to = 1;
            const uint32_t c0 = Ticker::cycles();
            edge_in(400u);
            const bool ran = wait_runs(High::runs, r, one_ms);
            if (!ran) {
                ++deferred;
                flush();
            }
            // The deferral from the arming, on the composed ruler (it
            // outlasts a period): the edge 400 counts after it.
            const uint32_t d = Ticker::cycles() - c0;
            if (d < least) {
                least = d;
            }
        }
        // The wait starts before the edge is armed 400 counts ahead, so a
        // deferred High starts at least 48000 cycles after the arming;
        // 47000 is that with a thousand to spare.
        print(serial, "  handler shape: ", deferred, " of 10 interrupts left High waiting for "
              "1 ms; the least deferral ", least, " cycles from the arming", crlf);
        bench.verdict("handler shape: an interrupt's post over the Console's dispatch is served "
                      "only when that dispatch posts again or returns (10 of 10 deferred past "
                      "1 ms)",
                      deferred == 10u && least >= 47'000u);
    }
    reset_scenario();
}

// =============================================================================
// d - the rulers, the scan and the post
// =============================================================================
volatile uint32_t post_quiet = 0;
volatile uint32_t post_road2 = 0;

uint32_t ticker_ruler_cost() {
    uint32_t best = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t a = Ticker::cycles();
        const uint32_t b = Ticker::cycles();
        if (b - a < best) {
            best = b - a;
        }
    }
    return best;
}

void td_scan() {
    reset_scenario();
    tracing = false;
    const uint32_t ruler = ruler_cost();
    const uint32_t ticker_ruler = ticker_ruler_cost();
    // The two rulers over spans of 0.5 to 0.8 ms (under the period the
    // bare read folds one restart of), each pair of reads taken back to
    // back in the same order: the bare read's delta (the restart folded
    // in) against the composed cycles' delta.
    uint32_t agree_hi = 0;
    for (int i = 0; i < 4; ++i) {
        const uint32_t c0 = Ticker::cycles();
        const uint32_t r0 = Ruler::now();
        spin(one_ms / 2u + one_ms / 10u * static_cast<uint32_t>(i));
        const uint32_t c1 = Ticker::cycles();
        const uint32_t r1 = Ruler::now();
        const uint32_t dc = c1 - c0;
        const uint32_t dr = between(r0, r1);
        const uint32_t diff = dc > dr ? dc - dr : dr - dc;
        if (diff > agree_hi) {
            agree_hi = diff;
        }
    }
    uint32_t scan = 0xFFFF'FFFFu;
    uint32_t scan_hi = 0;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        const bool served = K::activate();
        const uint32_t t1 = Ruler::now();
        const uint32_t c = between(t0, t1) - ruler;
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
        post_quiet = between(t0, t1) - r;
        post_road2 = between(t1, t2) - r;
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
    print(serial, "  the ruler's two reads (STK's CNTL, bare) ", ruler,
          " cycles; the ticker's composed cycles() two reads ", ticker_ruler, " cycles", crlf);
    print(serial, "  over 0.5..0.8 ms the two rulers differ by at most ", agree_hi, " cycles",
          crlf);
    print(serial, "  activate() with nothing ready below the Console: ", scan, "..", scan_hi,
          " cycles (four empty queues, the fifth at the ceiling)", crlf);
    print(serial, "  from Mid: a post to Low (asks nothing) ", quiet,
          " cycles; a post to High (road 2, High's dispatch inside) ", road2,
          " cycles; the scheduling's share ", road2 > quiet ? road2 - quiet : 0u, crlf);
    bench.verdict("the bare read is cheaper than the ticker's composed cycles()",
                  ruler < ticker_ruler);
    // Each pair of reads in the span is taken back to back, so the two
    // deltas differ by the skew between the pairs: at most the composed
    // read's own length plus one retry of its loop when a restart falls
    // between its reads - twice its two-read cost.
    bench.verdict("the two rulers agree over the span within twice the composed read's cost "
                  "(both count HCLK)",
                  agree_hi <= 2u * ticker_ruler);
    // The bounds below are the release listing's count under letter c's
    // cost model, read to read less the ruler's own two reads (by the
    // model 4 cycles in this image, 0 in the image without the hardware
    // prologue, where the two loads share a word; read 3 and 0 to 1 - the
    // model's word is one cycle coarse inside a word), the predictor's
    // spread at the path's predicted sites (letter c), and a fifth of the
    // count as the margin; each is a least of 16, so no tick enters it.
    // The scan: the read and the call (4); activate()'s frame and thirteen
    // stores, the ceiling's load, Top's visit - the call to the queue's
    // out-of-line empty(), seven instructions with its return - and the
    // index compare (100); High's, Mid's and Low's visits (40, 52, 52);
    // the Console's index ending the pass (20); the thirteen loads and the
    // return of the epilogue (36); the second read (8): 312 (90
    // instructions), 308 without the ruler; the spread at its 19 sites
    // (76) and the fifth (61): 445. Without the hardware prologue the same
    // instructions at other addresses: 316, and 455.
    constexpr uint32_t scan_bound = hpe_build ? 445u : 455u;
    bench.verdict("the scan of four empty queues, the least of 16, under its bound (the "
                  "arithmetic beside it)",
                  scan < scan_bound);
    // The quiet post: the read, the event's store and the call (4);
    // post<Low>()'s frame and the call to the queue's push() (20); push()
    // under its guard to the unmask and its return (76); the ceiling's
    // compare taken, the epilogue and the return (32); the instruction
    // before the second read (12): 144 (41 instructions; the queue's ring
    // wrapping changes the instructions and not the cycles), 140 without
    // the ruler; the spread at its 9 sites (36) and the fifth (28): 204.
    // Without the hardware prologue 156, and 223.
    constexpr uint32_t quiet_bound = hpe_build ? 204u : 223u;
    bench.verdict("a post that asks nothing, under its bound (the push and one compare)",
                  quiet < quiet_bound);
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    // The road-2 post: the read, post<High>()'s frame and push() as the
    // quiet post's (100); the compare not taken, the tail jump to the
    // request and its four instructions to the store that pends the
    // software interrupt (44); the trap's entry (20) and the software
    // interrupt's body to its MRET (28); the thread entry's stores and
    // call (48); activate() at Mid's ceiling to High's take() (228);
    // High's dispatch with nothing to do, the release, the pass served
    // (152); the rescan of Top and High and Mid's index ending it (128);
    // the epilogue (36); the entry's reloads and its ecall (44); the
    // ecall's entry (20) and the exit's body to its MRET (16); the MRET
    // into the request's return, the return and the second read (16):
    // 880 (244 instructions), 884 where the rings wrap (239), 880 the
    // costlier without the ruler; the spread at its 44 sites (176) and
    // the fifth (176): 1232. Without the hardware prologue (letter c) the two
    // bodies' scratch saves and the same instructions at other addresses:
    // 920, and 1280.
    constexpr uint32_t road2_bound = hpe_build ? 1232u : 1280u;
    print(serial, "  bounds: the scan ", scan_bound, ", a quiet post ", quiet_bound,
          ", a road-2 post ", road2_bound, crlf);
    bench.verdict("and under its bound (the arithmetic beside it), High's empty dispatch included",
                  road2 < road2_bound);
}

// =============================================================================
// e - depth 2
// =============================================================================
void te_depth2() {
    const char* const want = scheduler_in_thread_mode ? "Hs1 Hm1 Ts0 Te0 Hr1 He1"
                                                      : "Hs1 Hm1 Hr1 He1 Ts0 Te0";
    for (int road = 2; road >= 1; --road) {
        console_drain();
        uint8_t alike = 0;
        uint32_t least = 0xFFFF'FFFFu;
        uint32_t most = 0;
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
            if (road == 2) {
                post<High>(Hit{1});
            } else {
                edge_to = 1;     // the first edge starts High over the Console
                edge_in(400u);
                if (!wait_runs(High::runs, r, one_ms)) {
                    flush();
                }
            }
            const uint32_t d = between(edge_last, Top::entered);
            if (d < least) {
                least = d;
            }
            if (d > most) {
                most = d;
            }
            if (trace_is(want)) {
                ++alike;
            } else if (alike == run) {
                print_trace();
            }
        }
        print(serial, "  High started by road ", road, ": ", alike, " of 10 runs traced ", want,
              "; Top's first statement ", least, "..", most,
              " cycles after the interrupt's last", crlf);
        if constexpr (scheduler_in_thread_mode) {
            bench.verdict(road == 2 ? "road 1 at depth 2 (High by road 2): Top runs before High "
                                      "resumes (10 runs)"
                                    : "road 1 at depth 2 (High by road 1): Top runs before High "
                                      "resumes (10 runs)",
                          alike == 10u);
            // Letter c's path with activate() at High's ceiling, where
            // Top's visit is the take: TIM2's tail (12), the software
            // interrupt's (52), the thread entry (48), activate()'s frame
            // and Top's empty() answering no (96), the call to take(),
            // take() and the stamp (84) - 292 under letter c's model, 87
            // instructions - and the predictor's spread at its 10
            // predicted sites (40), a tick landing in it (76, letter c),
            // a fifth of the count (58): 466. Without the hardware
            // prologue letter c's additions make it 328, and 525.
            constexpr uint32_t depth2_bound = hpe_build ? 466u : 525u;
            bench.verdict("and Top's first statement within its bound of the interrupt's last "
                          "(the arithmetic beside it)",
                          most <= depth2_bound);
        } else {
            bench.verdict(road == 2 ? "handler shape, High by road 2: Top runs after High's end, "
                                      "the stage's limit (10 runs)"
                                    : "handler shape, High by road 1: Top runs after High's end, "
                                      "the stage's limit (10 runs)",
                          alike == 10u);
        }
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
    TE::process();
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
        TE::process();
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
    //    arm()'s or disarm()'s masked window; TIM2's interrupt lands there
    //    and its High preempts at the window's end, arming two timers and
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
    // In the handler shape High waits for Low's end, and its disarm of
    // te_a is then the last word: te_a never fires.
    const bool preempted = trace_is(scheduler_in_thread_mode ? "Ls3 Lm3 Hs1 He1 Lr3 Le3"
                                                             : "Ls3 Lm3 Lr3 Le3 Hs1 He1");
    constexpr uint8_t a_fires = scheduler_in_thread_mode ? 1u : 0u;
    if (!preempted) {
        print_trace();
    }
    High::action = [](uint8_t n) { ++fired[n]; };
    Low::action = [](uint8_t n) { ++fired[n]; };
    base_turn();                           // te_c and te_d: due at once
    const bool turn1 = fired[51] == 0u && fired[52] == 0u && fired[53] == 1u && fired[54] == 1u;
    wait_ticks(4u);                        // te_a's three ticks, and one more
    base_turn();
    const bool turn2 = fired[51] == a_fires && fired[52] == 0u && fired[53] == 1u && fired[54] == 1u;
    base_turn();
    const bool turn3 = fired[51] == a_fires && fired[52] == 0u && fired[53] == 1u && fired[54] == 1u;
    const bool empty = !TE::next_deadline().has_value();

    // 3. One arm() timed from the Console, the least of 16: the masked
    //    window is inside it.
    const uint32_t r = ruler_cost();
    uint32_t least = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        te_m.arm(1000);
        const uint32_t t1 = Ruler::now();
        if (between(t0, t1) - r < least) {
            least = between(t0, t1) - r;
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
    if constexpr (scheduler_in_thread_mode) {
        bench.verdict("High preempted Low's arm/disarm loop by road 1, inside it",
                      preempted && high_landed < 199u);
    } else {
        bench.verdict("handler shape: High ran after Low's arm/disarm loop had ended",
                      preempted && high_landed == 199u);
    }
    bench.verdict("the next turn: the two High armed fired, the one it disarmed (re-armed by "
                  "Low for three ticks) and the one Low disarmed did not",
                  turn1);
    bench.verdict(scheduler_in_thread_mode
                      ? "three ticks on: the re-armed one fired once; a turn more: nothing "
                        "twice, the list empty"
                      : "three ticks on: the one High disarmed last stayed silent; a turn more: "
                        "nothing twice, the list empty",
                  turn2 && turn3 && empty);
    // arm() over a list of one node (itself, re-armed), by the release
    // listing: the read, a store and the call (8); the frame, the guard's
    // csrrci and the call to the unlink (24); the unlink, out of line -
    // the head's load, the walk's test, the two stores of the unlink and
    // the node's two, the return (52); now()'s load, the deadline and the
    // node's stores, the link at the list's head, the walk's test (48);
    // the csrsi, the epilogue and the return (20); the instruction before
    // the second read (16): 168 under letter c's model (45 instructions),
    // 164 without the ruler, the masked window the 31 instructions from
    // the csrrci to the csrsi, 116 cycles; the predictor's spread at its 9
    // sites (36, letter c) and a fifth (32): 232. The least of 16 may be
    // the first, over an empty list (the unlink finds no head): 144.
    // Without the hardware prologue the same instructions at other
    // addresses: 156, and 223.
    constexpr uint32_t arm_bound = hpe_build ? 232u : 223u;
    print(serial, "  the bound of one arm() ", arm_bound, " cycles", crlf);
    bench.verdict("one arm(), the least of 16, under its bound, its masked window inside (the "
                  "arithmetic beside it)",
                  arm_cycles < arm_bound);
}

// =============================================================================
// g - the request under the mask
// =============================================================================
volatile bool pend_inside = true;       ///< the line read pending inside every section
volatile bool ran_inside = false;       ///< the receiver's runs moved inside a section
volatile uint32_t section_last = 0;     ///< the ruler at a section's last statement

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
        const bool pending = Pfic::pending(Irq::software);
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
    // precedes the mark after the section - in the handler shape, it
    // follows the poster's end (the Console's: the flush below, which the
    // letter makes once its marks are down).
    const char* const want_console = scheduler_in_thread_mode ? "Cm4 Cp4 Ms2 Me2 Ca4"
                                                              : "Cm4 Cp4 Ca4 Ms2 Me2";
    const char* const want_low = scheduler_in_thread_mode ? "Ls3 Lm3 Lp3 Hs1 He1 La3 Le3"
                                                          : "Ls3 Lm3 Lp3 La3 Le3 Hs1 He1";
    uint8_t alike_console = 0;
    uint8_t alike_low = 0;
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    bool back = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        post_in_section<Mid>('C');
        if constexpr (!scheduler_in_thread_mode) {
            flush();
        }
        {
            const uint32_t d = between(section_last, Mid::entered);
            least = d < least ? d : least;
            most = d > most ? d : most;
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
    print(serial, "  the section's last statement to Mid's first: ", least, "..", most,
          " cycles", crlf);
    bench.verdict("the software interrupt stood pending inside every section, and no receiver "
                  "ran inside one",
                  pend_inside && !ran_inside);
    if constexpr (scheduler_in_thread_mode) {
        bench.verdict("the Console's post to Mid under the mask is served when the section "
                      "ends, before the statement after it (10 runs)",
                      alike_console == 10u);
        bench.verdict("Low's post to High under the mask likewise, inside Low's dispatch "
                      "(10 runs)",
                      alike_low == 10u);
        // From the stamp's load to Mid's, by the release listing: the
        // stamp's store, the guard's test and its csrsi that lifts the
        // mask (12); the trap's entry taken at the unmask (20) and the
        // software interrupt's body (28); the thread entry (48);
        // activate() at the Console's ceiling - Top's visit (100), High's
        // (40), Mid's empty() and its answer (64), the ceiling's store,
        // the call to take(), take() and the stamp (92): 404 under letter
        // c's model, 114 instructions. The most of ten runs carries the
        // predictor's spread at the path's 19 predicted sites (76) and,
        // the path being some 8 us of the ticker's 1 ms period, a tick
        // landing in one of the ten runs about one time in twelve: its 76
        // (letter c); a fifth of the count (80): 636. Without the hardware
        // prologue letter c's additions make it 408, and 657.
        constexpr uint32_t section_bound = hpe_build ? 636u : 657u;
        bench.verdict("the section's end to the receiver's first statement under its bound "
                      "(the arithmetic beside it)",
                      most < section_bound);
    } else {
        bench.verdict("handler shape: the Console's post to Mid under the mask waits for the "
                      "Console's next request, the stage's limit (10 runs)",
                      alike_console == 10u);
        bench.verdict("handler shape: Low's post to High under the mask is served after Low's "
                      "end (10 runs)",
                      alike_low == 10u);
    }
    bench.verdict("the ceiling is the Console's again, interrupts enabled", back);
}

// =============================================================================
// i - dark
// =============================================================================
uint32_t read_intsyscr() {
    uint32_t v;
    __asm__ volatile("csrr %0, 0x804" : "=r"(v));
    return v;
}

void ti_dark() {
    reset_scenario();
    bench.verdict("Top, High, Mid and Low are idle: every queue empty",
                  Top::queue.empty() && High::queue.empty() && Mid::queue.empty() &&
                      Low::queue.empty());
    bench.verdict("no time event is armed", !TE::next_deadline().has_value());
    bench.verdict("TIM2's line is quiet (nothing pending)", !Pfic::pending(Edge::cc_irq()));
    bench.verdict("the LED's pad PA0 is not an output (its MODE bits 00)",
                  (Pin<'A', 0>::nibble() & 0x3u) == 0u);
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
    // The priority bytes, one a line (PFIC_IPRIORx, the device header's
    // IPRIOR array).
    const volatile uint8_t* prio = pfic()->IPRIOR;
    uint32_t others = 0;
    for (uint32_t n = 0; n < vector_count; ++n) {
        if (n != static_cast<uint8_t>(Irq::software) && prio[n] != 0u) {
            ++others;
        }
    }
    const uint32_t intsyscr = read_intsyscr();
    print(serial, "  INTSYSCR ", hex(intsyscr), ", the software interrupt's priority ",
          hex(prio[static_cast<uint8_t>(Irq::software)]), ", lines at another priority ", others,
          " of ", vector_count, crlf);
    bench.verdict("nesting on (INTSYSCR.INESTEN), the software interrupt alone at 0x80, every "
                  "other line at 0",
                  (intsyscr & intsyscr_inesten) != 0u &&
                      prio[static_cast<uint8_t>(Irq::software)] == scheduler_priority &&
                      others == 0u);
}

// =============================================================================
// h - the stack
// =============================================================================
void th_stack() {
    const uint32_t top = reinterpret_cast<uint32_t>(__stack_top);
    const uint32_t floor = reinterpret_cast<uint32_t>(__stack_floor);
    const uint32_t used = top - floor - stack_untouched();
    const uint32_t deepest_dispatch = top - sp_low_dispatch;
    const uint32_t deepest_isr = top - sp_low_isr;
    print(serial, "  painted by the crt ", top - floor, " bytes (", hex(floor), "..", hex(top),
          "); used ", used, " bytes; the deepest stack pointer sampled: a dispatch at ",
          deepest_dispatch, ", the interrupt at ", deepest_isr, crlf);
    // A dispatch's sample is taken inside activate(), whose 96-byte frame
    // is above it. Below the deepest one lies at most, by the release
    // listing: the letter's action and what it calls (the widest, letter
    // d's and letter g's, a 48-byte frame calling a post, 16, the queue's
    // push and the request, none: 64); and below that either an
    // interrupt handler's own frames - the hardware prologue pushes to its
    // internal stack, not to this one: TIM2's 32 with a post's 16, 48, the
    // UART's 16 and the tick's none below it - or, the handler gone and
    // its request pending, the software interrupt's block (16), the
    // entry's sixteen registers (64) and an activate() frame that finds
    // nothing to serve (96), with a handler over that (48): 224. 64 + 224
    // = 288: the high-water mark lies within 288 bytes of the deeper of
    // the two samples. Without the hardware prologue the handlers' frames
    // carry their saves: TIM2's 96 and a post's 16 (the UART's 80, the
    // tick's 16), so 352.
    constexpr uint32_t below = hpe_build ? 288u : 352u;
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    bench.verdict("the paint reached the deepest sampled stack pointer", used >= deepest);
    print(serial, "  the bound below the deepest sample ", below, " bytes", crlf);
    bench.verdict("the high-water mark is within the bound of the deepest sample (the arithmetic "
                  "beside it)",
                  used <= deepest + below);
}

// =============================================================================
// the Console's turn
// =============================================================================
bool clock_ok = false;
bool tick_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
          tick_ok ? "STK" : "FAILED", " ruler=STK", crlf);
    print(serial, "test_x035_rubato - the preemptive kernel on the QingKe V4C, the scheduler ",
          scheduler_in_thread_mode ? "in thread mode" : "inside the software interrupt",
          crlf);
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
BRIO_CH32_VECTOR(usart2_handler) {
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }

/// TIM2's capture-compare line: the edge disarmed, the post, and the
/// ruler as the last statement - letter c's near end.
BRIO_CH32_VECTOR(tim2_cc_handler) {
    Edge::interrupts(Edge::compare_interrupt(0), false);
    Edge::clear_flags(Edge::compare_flag(0));
    sample_sp(sp_low_isr);
    edge_hits = edge_hits + 1u;
    if (edge_to == 0u) {
        brio::post<Top>(Hit{1});
    } else {
        brio::post<High>(Hit{1});
    }
    edge_last = Ruler::now();
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    Edge::init();
    (void)Edge::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Pfic::enable(Edge::cc_irq());
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", ta_ceiling);
    bench.letter('b', "road 2: the chain inside the posts", tb_road2);
    bench.letter('c', "the latency of road 1", tc_latency);
    bench.letter('d', "the rulers, the scan and the post, in cycles", td_scan);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", te_depth2);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", tf_timers);
    bench.letter('g', "the request under the mask: served when the section ends", tg_mask);
    bench.letter('i', "dark: every object idle, nothing lit, the nesting as set", ti_dark);
    bench.letter('h', "the stack's high-water mark", th_stack);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
