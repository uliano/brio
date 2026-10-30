// test_vx03_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the CH32V203 and the CH32V303:
// Ch32vx03RubatoPlatform and the QingKe V4's scheduling request
// (ch32vx03/schedule.hpp) on the V4B and the V4F, against the model the
// host suite specifies (test/test_rubato).
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
// (USART1 on PA9/PA10); the suite configures no other pad, and the LED's
// pad PB2 stays an input (letter i checks it). The interrupt every road-1
// letter lands is TIM2's compare, a PFIC line of its own on TIM2's clock,
// which is HCLK here (the static_assert below), so a delay placed in its
// counts is a delay in core cycles. On a part with the FPU letter j adds
// TIM3's compare, bound by the float trampoline (pfic.hpp), for a handler
// that does float work over the dispatch it interrupts.
//
// THE RULER is the STK counter itself, read bare: CNTL counts HCLK and
// restarts at the ticker's compare every millisecond, so two reads less
// than a period apart are a cycle count with the restart folded in
// (`between()` below) - one load. The ticker's composed cycles()
// (ch32vx03/ticker.hpp: five reads and a consistency check) is the
// other ruler weighed; letter d prints both costs and checks one against
// the other, and the composed one times only the waits longer than a
// period. TIM2 is not the console's (USART1) nor the ticker's (STK).
//
// WHERE THE SCHEDULER RUNS is a constant of the image,
// `scheduler_in_thread_mode` below, and the letters whose verdict depends
// on it judge against it. Here it is true: the software interrupt returns
// into the scheduler in thread mode and the scheduler leaves through the
// machine-mode environment call (ch32vx03/schedule.hpp), so the software
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
//      chain, and the UART's receive interrupt read the loop's ceiling
//      when the letter's key arrived - 255 at the base, or 0 while the
//      base turn walks the time events. Ten runs, all alike.
//   b  ROAD 2: Low posts to Mid, Mid to High, inside the handlers; High
//      posts to Low. The trace nests High inside Mid inside Low, and
//      Low's second event is served after its first dispatch ends, never
//      inside it. Ten runs, all alike.
//   c  THE LATENCY OF ROAD 1, the Console's wait loop interrupted: the
//      ruler at the interrupt handler's last statement and at High's
//      handler's first, min, median and max over 100 events after twelve
//      of warm-up, the console drained first; the bound beside it. Where
//      road 1 is not kept, the letter measures and judges the deferral
//      instead.
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
//   j  (a part with the FPU) THE FLOATING-POINT UNIT ACROSS A PREEMPTION:
//      Low's handler sums a float series under round-toward-zero with a
//      mark inside the loop, where it clears fflags and the edge lands;
//      High, started by it, sums another and raises NX (read at its end).
//      Twenty runs, ten
//      with the float handler (TIM3) computing over Low's live registers:
//      every value bit-exact against the same functions run without
//      preemption, Low's fflags still clear after the preemption and its
//      fcsr at its series' end as without it, High finding FS Clean. Then
//      the cost of the entry's save: road 1 over Low with its unit Dirty
//      and Clean, the edge to High's first statement and High's last to
//      Low's first, sixteen of each.
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
// build: boards = v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <bit>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/schedule.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

/// Where the scheduler runs in this image: true, in thread mode, the
/// software interrupt returning into it (ch32vx03/schedule.hpp); false
/// would state a scheduler run inside the software interrupt's handler
/// (the file header).
constexpr bool scheduler_in_thread_mode = true;

using P = Ch32vx03RubatoPlatform<>;
using Edge = Tim<2>;
static_assert(Edge::clock_hz(clock) == SysClock::hz,
              "TIM2 counts HCLK here: a delay in its counts is a delay in core cycles");
static_assert(Tim<3>::clock_hz(clock) == SysClock::hz, "and so does letter j's TIM3");

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

using Serial = Uart<1, P>;
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
[[gnu::always_inline]] inline void edge_in(uint32_t d) {
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

/// The scheduler run by hand from the Console's dispatch, as the
/// platform's entry and exit run it (kernel/platform.hpp): the ceiling
/// replaced with 0, activate() at the Console's own, the ceiling put
/// back. Whether it served. Unlike the platform's exit it does not compare
/// the count of quiet posts after the run, and here it need not: a post
/// that lands in the run's last choice, with the ceiling at 0, is counted
/// quiet and waits, and the only handler that posts while the letters
/// call this - letters d and f - is the UART's receive edge, to the
/// Console, whose index is the base itself and which no run started here
/// may serve; TIM2's and TIM3's edges are armed only inside a dispatch and
/// disarm themselves there, and the tick posts nothing.
bool activate_here() {
    const uint8_t base = P::ceiling;
    P::ceiling = 0;
    const bool served = K::activate(base).served;
    P::ceiling = base;
    return served;
}
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
    // The key arrives while the kernel's loop runs below every object, and
    // the loop holds one of two ceilings there: 255, its base, or 0 while
    // its base turn walks the matured time events (kernel/rubato.hpp,
    // run(): a fire then only queues; init_all() holds 0 the same way
    // around the inits). 0 is also Top's index, so a 0 would be Top's
    // dispatch as well - but nothing can dispatch Top when this key
    // arrives: the suite arms no time event on Top (its timers are High's,
    // Mid's and Low's), and the one post to Top is TIM2's handler with
    // edge_to at 0, which only letter e's High sets, arming one edge that
    // lands inside its own spin; that handler's first statement disarms
    // the compare interrupt, so after letter e no edge is armed, and
    // letter i checks the line quiet before the prompt (TIM3's edge, letter
    // j's, posts to High alone and disarms itself the same way). So a 0
    // here is the walk, never a dispatch.
    const bool at_base = rx_ceiling == 255u;
    const bool in_walk = rx_ceiling == 0u;
    print(serial, "  the UART's receive interrupt read the ceiling ", rx_ceiling,
          at_base   ? " (the loop's base)"
          : in_walk ? " (the base turn's walk of the time events, held at 0)"
                    : " (neither the base nor the walk)",
          " when this letter's key arrived", crlf);
    bench.verdict("the ceiling the receive interrupt read as the key arrived is the loop's: 255 "
                  "at its base or 0 during its walk of the time events (no dispatch of Top can "
                  "be running then)",
                  at_base || in_walk);
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
            edge_in(2000u);      // 14 us into a 139 us spin: the edge lands inside it
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

/// THE COST MODEL every bound of this suite is drawn with, over the release
/// listing of this image: an instruction one cycle (the QingKe V4 is a
/// three-stage single-issue pipeline; 64 straight `addi` and a call take
/// 72, docs/ch32vx03/platform.md), a load two, a taken branch two (a
/// four-instruction loop with its taken branch, 5 cycles a turn), a call
/// or a return three (an empty call in a loop, 9 cycles with the loop's
/// branch); and the trap's own transitions as the platform suite's letter
/// e measured them on this core with the hardware prologue (the
/// software interrupt raised by hand): an entry 16 cycles from the store
/// that pends it to the handler's first statement, an MRET 25 from the
/// handler's last statement to the interrupted code's next - each figure
/// with a few instructions of the raiser's own inside it, so an upper
/// estimate here. The QingKe V4 manual gives no cycle count for either,
/// only that the hardware prologue pushes and pops in one cycle (3.4).
///
/// The road-1 path from the interrupt handler's last statement to High's
/// first, by the listing:
///   TIM2's handler after the stamp's load: a load, a store, the stack's
///   release (3 instructions)                                        4
///   its MRET into the software interrupt, pending: an MRET and an
///   entry                                                        25 + 16
///   the software interrupt's body: the trap's mstatus read and MIE
///   cleared in one instruction, the block, that mstatus with MIE clear
///   and mepc stored, the ceiling loaded into the block and 0 stored in
///   its place, the entry's address into mepc (12 instructions, one
///   load)                                                          13
///   its MRET into the entry                                        25
///   the entry: the frame, sixteen stores, `base` loaded from the
///   block, csrsi, the call (20)                                    23
///   activate(): its frame and thirteen stores, `base` kept, the
///   queues' addresses (18 instructions)                            18
///   the choice's head: the count of quiet posts loaded, `base`
///   tested against 0 (3)                                            4
///   the scan's registers: the limit, the hold, the indices (9)      9
///   Top's visit, its two cursors loaded and compared in line, its
///   branch taken (5)                                                8
///   the limit's compare for High, not taken (1)                     1
///   High's visit, in line, its branch not taken (5)                 7
///   the raise: the ceiling's store, the hold's test, the count
///   loaded again and compared, the queue's address, the
///   dispatching index's load and store, the call to take() (9)    13
///   take(), fifteen with its return                               20
///   its result tested, the stamp's address and load (5)            6
///                                                                 ---
///                                                                 192
/// (105 instructions outside the traps, 70 of them in activate().) That
/// is the CH32V203's listing; the CH32V303's, built with F, runs the same
/// functions instruction for instruction and adds two things to every
/// path through the entry: a vector that calls out is a trampoline there
/// (pfic.hpp), TIM2's body reloading its return address and returning to
/// the MRET (a load and a return, 5), and the entry tests mstatus.FS
/// before the call (four instructions, the branch taken where there is
/// nothing to save, 5): 202. On top of it, at most one tick landing in the
/// path - the ticker's line is at the peripheral level, so it preempts the
/// software interrupt's handler at its first instruction, the entry and
/// activate(), never TIM2's handler: its entry (16), its twelve
/// instructions (14) and its MRET (25), 55. The bound is the two, 247
/// (257), and a fifth of the path's count, 38 (40): 285 (297). And the
/// spread: two events no other interrupt touches run the same
/// instructions, and differ by the predictor's state at the path's taken
/// transfers, two cycles each at most - four on the CH32V203 (the call to
/// activate(), Top's branch, the call to take() and its return), six on
/// the CH32V303 (the body's return and the FS test's branch besides): an
/// event more than 8 (12) cycles over the fastest is one a tick landed
/// in, and the letter reads the kernel tick around each event to say so.
///
/// AN IMAGE BUILT WITHOUT THE HARDWARE PROLOGUE (CH32VX03_HPE=OFF) pays
/// more on every path through a trap, and its bounds are drawn the same
/// way: letter e's figures for that build, an entry 19 and an MRET 32
/// (three and seven more, 17 over the path's three transitions); each
/// scheduler body's save of its two scratch registers (two stores and two
/// loads, 6 cycles a body); and TIM2's handler, attributed and saving in
/// software, reloading after the stamp its sixteen integer registers on
/// the CH32V203 (36 cycles where 4 were, 32 more) and its twenty
/// f-registers too on the CH32V303 (79 where the trampoline's tail took
/// 9, 70 more). Road 1 then counts 247 (295), and a tick landing in it
/// costs 76 (its entry 19, its twenty instructions 25, its MRET 32): 372
/// (430) with a fifth of the count. That handler is no trampoline, so the
/// body's return leaves the CH32V303's path: five taken transfers, a
/// spread of 10.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
constexpr bool hpe_build = true;
#else
constexpr bool hpe_build = false;
#endif
constexpr bool fpu_part = device::has_fpu;
constexpr uint32_t latency_count = hpe_build ? (fpu_part ? 202u : 192u) : (fpu_part ? 295u : 247u);
constexpr uint32_t latency_bound = hpe_build ? (fpu_part ? 297u : 285u) : (fpu_part ? 430u : 372u);
constexpr uint32_t latency_spread = fpu_part ? (hpe_build ? 12u : 10u) : 8u;
/// The events letter c runs before it counts (the comment in tc_latency()).
constexpr uint32_t latency_warmup = 12;

void tc_latency() {
    reset_scenario();
    tracing = false;
    if constexpr (scheduler_in_thread_mode) {
        // Twelve warm-up events not counted, then 100; each waits at most
        // 1 ms for High's first statement. The warm-up takes High's queue
        // cursor twice round its six positions: on the CH32V303VCT6 the
        // first two events whose take() wraps the cursor ran 13 to 14
        // cycles over the fastest, every run alike, and the later wraps 1
        // or 2 (read over the debug port, in an image that differed from
        // this one by its sort skipped) - the wrap path cold in the core's
        // predictor after the letters before, a cost of the first passes
        // that the model's two cycles a taken transfer does not describe
        // and that the letter's figure is not about.
        console_drain();
        uint32_t missed = 0;
        // An event slower than the fastest by more than the spread is one
        // a tick landed in: the kernel tick read around each says so.
        bool ticked_at[latency_events];
        for (uint32_t i = 0; i < latency_warmup + latency_events; ++i) {
            const uint32_t r = High::runs;
            const uint32_t t0 = Ticker::ticks();
            edge_to = 1;
            edge_in(400u);
            if (!wait_runs(High::runs, r, one_ms)) {
                ++missed;
                flush();
            }
            const bool ticked = Ticker::ticks() != t0;
            if (i >= latency_warmup) {
                latency[i - latency_warmup] = between(edge_last, High::entered);
                ticked_at[i - latency_warmup] = ticked;
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
        // so the interrupt's request waits. 1 ms of wait (144000 cycles, a
        // few hundred times the path's own length) and High has not run;
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
        // deferred High starts at least 144000 cycles after the arming;
        // 143000 is that with a thousand to spare.
        print(serial, "  handler shape: ", deferred, " of 10 interrupts left High waiting for "
              "1 ms; the least deferral ", least, " cycles from the arming", crlf);
        bench.verdict("handler shape: an interrupt's post over the Console's dispatch is served "
                      "only when that dispatch posts again or returns (10 of 10 deferred past "
                      "1 ms)",
                      deferred == 10u && least >= 143'000u);
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
    // The two rulers over spans of 0.5 to 0.8 ms, each pair of reads taken
    // back to back in the same order: the bare read's delta (the restart
    // folded in) against the composed cycles' delta.
    uint32_t agree_hi = 0;
    for (int i = 0; i < 4; ++i) {
        const uint32_t c0 = Ticker::cycles();
        const uint32_t r0 = Ruler::now();
        spin(one_ms / 2u + 14'400u * static_cast<uint32_t>(i));
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
        const bool served = activate_here();
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
    // cost model, and a fifth of it as the margin.
    // The scan, through activate_here() - the platform's entry and exit
    // written out, so its few instructions are the scan's here: the call
    // (3); activate_here()'s frame, the ceiling loaded and 0 stored,
    // `base` moved, the call to activate() (8 instructions, 11 cycles);
    // activate()'s frame and thirteen stores, `base` kept, the queues'
    // addresses (18, 18); the choice's head, the count loaded and `base`
    // tested (3, 4); the scan's registers (9, 9); per empty object its two
    // cursors loaded and compared in line, no call, its branch taken
    // (Top's, High's and Mid's 8, Low's 9 with its queue's address), and
    // the limit's compare after each (1 not taken after Top, High and
    // Mid; after Low the Console's index ends the scan, its constant and
    // the taken branch, 3); no hold, so the choice ends (a taken branch,
    // 2); the epilogue - the result packed, thirteen loads and the return
    // (18, 33); activate_here()'s three loads, the ceiling put back, the
    // result's byte and the return (8, 13): 92 instructions, 132 cycles;
    // 158 with the fifth.
    constexpr uint32_t scan_bound = 158u;
    bench.verdict("the scan of four empty queues, the least of 16, costs under 158 cycles",
                  scan < scan_bound);
    // The quiet post: the event's store and the call (2 instructions, 4
    // cycles), post<Low>()'s frame and the call to the queue's push() (6,
    // 8), push() under its guard to the unmask and the return (22, 30),
    // the ceiling's compare and its taken branch (4, 7), the count of quiet
    // posts loaded, incremented and stored inside the epilogue (7, 12),
    // and the instruction before the second read (1): 42 instructions, 62
    // cycles; 74 with the fifth. Its path asks no queue whether it is
    // empty: the same instructions with empty() in line or out of it.
    constexpr uint32_t quiet_bound = 74u;
    bench.verdict("a post that asks nothing costs under 74 cycles (the push, one compare and "
                  "the count)",
                  quiet < quiet_bound);
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    // The road-2 post: the event's store and the call, post<High>()'s
    // frame and the call to push() (8 instructions, 12 cycles), push() as
    // in the quiet post (22, 30), the compare not taken and the tail jump
    // to the request (8, 13), the request's four instructions to its
    // fence.i (4), the trap's entry (16), the software interrupt's twelve
    // (13), its MRET (25), the entry's twenty (23); activate() at Mid's
    // ceiling, 151 instructions and 208 cycles - the frame, the choice's
    // head, the scan's registers, Top's and High's visits in line and the
    // raise with the count compared to the call to take() (50, 60), take()
    // (15, 20), High's dispatch with nothing to do and the call to
    // release() with its body (35, 51), the dispatching index restored,
    // the served flag, the ceiling 0 and the jump back (8, 10), the second
    // choice - its head, its registers, Top and High empty in line, Mid's
    // index ending the scan, no hold (25, 34), the epilogue (18, 33); the
    // entry's eighteen after the call, the count stored in the block
    // (34), the ecall's entry (16), the exit's eleven to its MRET - MIE
    // cleared, the ceiling back, the count compared and found still,
    // mepc and mstatus back, the block popped (16), its MRET (25), the
    // request's return (3): 255 instructions, 356 cycles, and the four
    // transitions 82: 438, and with the entry's FS test where there is F
    // (letter c) 443; 526 (532) with the fifth. Without the hardware
    // prologue (letter c) the two bodies' scratch saves (12) and the four
    // transitions' 20 more: 470 (475), 564 (570).
    constexpr uint32_t road2_bound =
        hpe_build ? (fpu_part ? 532u : 526u) : (fpu_part ? 570u : 564u);
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
            // Top's visit is the take: TIM2's tail (4), the software
            // interrupt's twelve (13), the entry's twenty (23), activate()'s
            // frame, the choice's head and the scan's registers (30
            // instructions, 31 cycles), Top's visit in line, its branch not
            // taken (5, 7), the raise to 0 with the hold's test and the count
            // compared, the queue's address, the dispatching index's load
            // and store and the call to take() (10, 14), take() (15, 20),
            // its result and the stamp, the ruler's address already in a
            // register (4, 5) - 99 instructions, 117 cycles under letter c's
            // model - and the three transitions (66): 183, 193 with letter
            // c's two additions where there is F. With a tick landing in it
            // (55) and a fifth (37, 39): 275 (287). Without the hardware
            // prologue letter c's additions make it 238 (286), and with a
            // tick (76) and a fifth 362 (419).
            constexpr uint32_t depth2_bound =
                hpe_build ? (fpu_part ? 287u : 275u) : (fpu_part ? 419u : 362u);
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
    (void)activate_here();
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
    (void)activate_here();
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
    // listing: a store and the call (2), the frame, the guard's csrrci and
    // the call to the unlink (7), the unlink, out of line - the head's
    // load, the walk's test, the two stores of the unlink and the node's
    // two, the return (12) - now()'s load, the deadline and the node's
    // stores, the link at the list's head, the walk's test (15), the
    // csrsi, the epilogue and the return (7), the instruction before the
    // second read: 44 instructions, 64 cycles under letter c's model, the
    // masked window the 30 instructions from the csrrci to the csrsi; 77
    // with a fifth.
    bench.verdict("one arm(), the least of 16, costs under 77 cycles, its masked window inside",
                  arm_cycles < 77u);
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
        // mask, the two instructions after it the V4B runs before the
        // trap (6, this core's shadow behind the unmask, ch32vx03/
        // schedule.hpp); the trap's entry (16), the software interrupt's
        // twelve (13) and its MRET (25); the entry's twenty (23);
        // activate() at the Console's ceiling - its frame, the choice's
        // head and the scan's registers (30 instructions, 31 cycles),
        // Top's and High's visits in line, empty, each with the limit's
        // compare after it (12, 18), Mid's visit (5, 7), the raise with the
        // hold's test and the count compared, to the call to take() (10,
        // 14), take() (15, 20) and the stamp (5, 6) - 77 instructions, 96
        // cycles: 115 instructions, 138 cycles under letter c's model, and
        // the two transitions 41: 179, 184 with the entry's FS test where
        // there is F; 215 (221) with a fifth. Without the hardware prologue
        // the software interrupt's scratch saves (6) and the two
        // transitions' 10 more: 195 (200), 234 (240).
        constexpr uint32_t section_bound =
            hpe_build ? (fpu_part ? 221u : 215u) : (fpu_part ? 240u : 234u);
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
// j - the floating-point unit across a preemption (a part with the FPU)
// =============================================================================
using FloatEdge = Tim<3>;                ///< letter j's second edge: a float handler
volatile bool float_edge_float = false;  ///< the float handler computes over Low's registers
volatile float float_edge_acc = 0.0f;
volatile uint32_t low_mark_fflags = 0;   ///< fflags Low reads after the preemption
volatile uint32_t low_fcsr_after = 0;    ///< fcsr Low reads at its series' end
volatile uint32_t high_fs = 0;           ///< mstatus.FS as High's dispatch finds it
volatile uint32_t high_fflags = 0;       ///< fflags at the end of High's series
volatile float dirt = 0.0f;              ///< what Low writes to dirty its unit
volatile uint32_t high_left = 0;         ///< the ruler at High's action's last statement
volatile uint32_t low_back = 0;          ///< and at Low's first after it resumed
volatile bool dirty_run = false;         ///< letter j's timed half: Low dirties its unit

[[gnu::always_inline]] inline uint32_t read_fflags() {
    uint32_t f;
    __asm__ volatile("csrr %0, fflags" : "=r"(f)::"memory");
    return f;
}
[[gnu::always_inline]] inline void write_fflags(uint32_t f) {
    __asm__ volatile("csrw fflags, %0" ::"r"(f) : "memory");
}
[[gnu::always_inline]] inline uint32_t read_fcsr() {
    uint32_t f;
    __asm__ volatile("frcsr %0" : "=r"(f)::"memory");
    return f;
}
[[gnu::always_inline]] inline void write_fcsr(uint32_t f) {
    __asm__ volatile("fscsr %0" ::"r"(f) : "memory");
}

/// A float series with a mark inside the loop: at i == mark fflags is
/// cleared, the edge is armed and the loop spins with every accumulator
/// live, and fflags is read back. No call inside the loop, so the
/// accumulators stay in caller-saved f-registers, the ones the entry
/// saves.
template <bool fpu = device::has_fpu>
[[gnu::noinline]] float series_low(uint32_t n, uint32_t mark, bool float_edge) {
    float a = 0.0f;
    float b = 1.0f;
    float c = 0.5f;
    for (uint32_t i = 0; i < n; ++i) {
        a += b / static_cast<float>(i + 1u);
        b = b * 0.999f + c;
        c = c * 0.5f + 0.25f;
        if (i == mark) {
            if constexpr (fpu) {
                write_fflags(0);
            }
            if (float_edge) {
                FloatEdge::enable(false);
                FloatEdge::set_count(0);
                FloatEdge::clear_flags(FloatEdge::compare_flag(0));
                (void)FloatEdge::set_compare(0, 500u);
                FloatEdge::interrupts(FloatEdge::compare_interrupt(0), true);
                FloatEdge::enable(true);
            } else {
                edge_to = 1;
                edge_in(500u);
            }
            spin(20000u);
            if constexpr (fpu) {
                low_mark_fflags = read_fflags();
            }
        }
    }
    if constexpr (fpu) {
        low_fcsr_after = read_fcsr();
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
volatile bool low_float_edge = false;

/// Low runs its series under round-toward-zero, fcsr 0x20; High inherits
/// the mode it preempts.
constexpr uint32_t series_mode = 0x20u;

template <bool fpu = device::has_fpu>
void tj_fpu() {
    if constexpr (fpu) {
        reset_scenario();
        tracing = false;
        // The references are taken the same way with no preemption: one
        // fcsr for the program, as under Tenuto, which the entry saves
        // and gives back to the code it preempted.
        const uint32_t fcsr0 = read_fcsr();
        write_fcsr(series_mode);
        const uint32_t ref_low = std::bit_cast<uint32_t>(series_low(400u, 0xFFFF'FFFFu, false));
        const uint32_t ref_fcsr = low_fcsr_after;
        const uint32_t ref_high = std::bit_cast<uint32_t>(series_high(300u));
        write_fcsr(fcsr0);
        Low::action = [](uint8_t) {
            write_fcsr(series_mode);
            low_sum = series_low(400u, 200u, low_float_edge);
        };
        High::action = [](uint8_t) {
            uint32_t st;
            __asm__ volatile("csrr %0, mstatus" : "=r"(st));
            high_fs = (st >> 13) & 3u;
            high_sum = series_high(300u);
            high_fflags = read_fflags();
        };
        bool exact = true;
        bool flags_kept = true;
        bool fcsr_kept = true;
        bool edge_exact = true;
        bool fs_clean = true;
        bool high_nx = true;
        for (int run = 0; run < 20; ++run) {
            low_float_edge = run >= 10;
            float_edge_float = run >= 10;
            float_edge_acc = 2.0f;
            low_sum = 0.0f;
            high_sum = 0.0f;
            const uint32_t h = High::runs;
            post<Low>(Hit{1});
            const bool ran = High::runs == h + 1u;
            const uint32_t got_low = std::bit_cast<uint32_t>(static_cast<float>(low_sum));
            const uint32_t got_high = std::bit_cast<uint32_t>(static_cast<float>(high_sum));
            exact = exact && ran && got_low == ref_low && got_high == ref_high;
            flags_kept = flags_kept && low_mark_fflags == 0u;
            fcsr_kept = fcsr_kept && low_fcsr_after == ref_fcsr;
            fs_clean = fs_clean && high_fs == 2u;
            high_nx = high_nx && (high_fflags & 0x01u) != 0u;
            if (run >= 10) {
                edge_exact = edge_exact && std::bit_cast<uint32_t>(static_cast<float>(
                                               float_edge_acc)) == std::bit_cast<uint32_t>(4.0f);
            }
            if (run == 0 || run == 10) {
                print(serial, "  ", run == 10 ? "a float handler over Low: " : "", "Low ",
                      hex(got_low), " (", hex(ref_low), "), High ", hex(got_high), " (",
                      hex(ref_high), ") with fflags ", hex(high_fflags),
                      ", Low's fflags after the preemption ", low_mark_fflags,
                      ", its fcsr at the end ", hex(low_fcsr_after), " (", hex(ref_fcsr),
                      "), FS as High found it ", high_fs, crlf);
            }
        }
        // The entry's save and restore, timed: road 1 over Low with its
        // unit Dirty (one float written before the edge) and Clean
        // (nothing written since the save that started it), the edge to
        // High's first statement and High's last to Low's first after it.
        uint32_t in_dirty = 0xFFFF'FFFFu;
        uint32_t in_clean = 0xFFFF'FFFFu;
        uint32_t out_dirty = 0xFFFF'FFFFu;
        uint32_t out_clean = 0xFFFF'FFFFu;
        uint32_t in_dirty_hi = 0;
        uint32_t in_clean_hi = 0;
        uint32_t out_dirty_hi = 0;
        uint32_t out_clean_hi = 0;
        console_drain();
        Low::action = [](uint8_t) {
            if (dirty_run) {
                dirt = dirt + 1.0f;
            }
            const uint32_t r = High::runs;
            edge_to = 1;
            edge_in(400u);
            while (High::runs == r) {
            }
            low_back = Ruler::now();
        };
        High::action = [](uint8_t) { high_left = Ruler::now(); };
        for (int run = 0; run < 32; ++run) {
            dirty_run = (run & 1) != 0;
            post<Low>(Hit{1});
            const uint32_t in = between(edge_last, High::entered);
            const uint32_t out = between(high_left, low_back);
            if (dirty_run) {
                in_dirty = in < in_dirty ? in : in_dirty;
                out_dirty = out < out_dirty ? out : out_dirty;
                in_dirty_hi = in > in_dirty_hi ? in : in_dirty_hi;
                out_dirty_hi = out > out_dirty_hi ? out : out_dirty_hi;
            } else {
                in_clean = in < in_clean ? in : in_clean;
                out_clean = out < out_clean ? out : out_clean;
                in_clean_hi = in > in_clean_hi ? in : in_clean_hi;
                out_clean_hi = out > out_clean_hi ? out : out_clean_hi;
            }
        }
        reset_scenario();
        print(serial, "  road 1 over Low, the edge to High's first statement: FS Dirty ",
              in_dirty, "..", in_dirty_hi, ", Clean ", in_clean, "..", in_clean_hi,
              " cycles; High's last to Low's first: Dirty ", out_dirty, "..", out_dirty_hi,
              ", Clean ", out_clean, "..", out_clean_hi, crlf);
        if constexpr (hpe_build) {
            print(serial, "  the save on the way in ", in_dirty - in_clean,
                  " cycles, the restore on the way out ", out_dirty - out_clean, crlf);
        }
        bench.verdict("Low's and High's sums bit-exact across 20 preemptions, ten of them with a "
                      "float handler over Low's live registers",
                      exact);
        bench.verdict("the float handler's own result exact (2 x 1.5 + 1)", edge_exact);
        bench.verdict("fcsr given back: Low's flags cleared before the preemption still clear "
                      "after it though High's series raised NX, its rounding mode and flags at "
                      "the end as without the preemption",
                      flags_kept && fcsr_kept && high_nx);
        bench.verdict("the scheduler runs with FS Clean (the entry saved and cleaned it, or found "
                      "nothing to save)",
                      fs_clean);
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
        bench.verdict("the save is paid where FS is Dirty and skipped where it is Clean",
                      in_dirty > in_clean && out_dirty > out_clean);
        // The bounds of the two, from what the float trampoline's own set
        // costs on this core (test_vx03_platform's letter l: its twenty
        // stores, fcsr's read and its store, a frame - 23 instructions -
        // 33 cycles of entry over a leaf's; its fcsr written back, twenty
        // loads, the frame - 23 - 26 cycles of exit over a leaf's): the
        // save here is the same 23 and FS cleaned (two), 35; the restore
        // the same 23 and the jump over the clean path (two), 28. Each is
        // the difference of two paths' least, and the two paths are two
        // layouts, not one path with a block skipped: the Dirty path takes
        // its jump where the Clean one falls through, and its branch falls
        // through where the Clean one's is taken, so each least carries
        // its own transfers' alignment and predictor state - the spread
        // letter c derives for a path on this core, latency_spread (12 on
        // this part), which the difference inherits. Measured, the restore
        // read 20 to 35 and the save 25 to 33 across three images of this
        // suite that differ in their layout alone, the model inside both
        // ranges. So: the model and that spread, 47 and 40.
        bench.verdict("the save within 47 cycles and the restore within 40 (the model and the "
                      "layout spread)",
                      in_dirty - in_clean <= 35u + latency_spread &&
                          out_dirty - out_clean <= 28u + latency_spread);
#else
        // Without the hardware prologue the edge's handler is the plain
        // attributed one, which saves the twenty caller-saved f-registers in
        // software around its call and reloads them: the reload leaves FS
        // Dirty, so the entry finds it Dirty on both paths and saves on
        // both. The two paths then run the same instructions and differ by
        // letter c's spread at most (10 in this build).
        const uint32_t d_in = in_dirty > in_clean ? in_dirty - in_clean : in_clean - in_dirty;
        const uint32_t d_out = out_dirty > out_clean ? out_dirty - out_clean : out_clean - out_dirty;
        bench.verdict("without the hardware prologue the edge's handler leaves FS Dirty and the "
                      "entry saves on both paths: the two within 10 cycles",
                      d_in <= latency_spread && d_out <= latency_spread);
#endif
    }
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
    bench.verdict("TIM2's line is quiet (nothing pending)", !Pfic::pending(Edge::irq()));
    bench.verdict("the LED's pad PB2 is not an output (its MODE bits 00)",
                  (Pin<'B', 2>::nibble() & 0x3u) == 0u);
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
    // The priority bytes read through the block's base, one byte a line:
    // the device header's array covers the first 64 lines.
    const volatile uint8_t* prio = reinterpret_cast<const volatile uint8_t*>(0xE000E400UL);
    uint32_t others = 0;
    for (uint32_t n = 0; n < device::vector_count; ++n) {
        if (n != static_cast<uint8_t>(Irq::software) && prio[n] != 0u) {
            ++others;
        }
    }
    const uint32_t intsyscr = read_intsyscr();
    print(serial, "  INTSYSCR ", hex(intsyscr), ", the software interrupt's priority ",
          hex(prio[static_cast<uint8_t>(Irq::software)]), ", lines at another priority ", others,
          " of ", device::vector_count, crlf);
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
    // internal stack, not to this one: TIM2's 32 with a post's 16, 48 - or,
    // the handler gone and its request pending, a scheduler run that
    // serves nothing (one that serves would sample deeper): at most two
    // software interrupt blocks (16 each) - the second where a handler
    // landing on the trap's first instruction, before its csrci, pended
    // the request again, taken at the entry's first instruction as a run
    // at a ceiling of 0 (qingke/schedule.hpp, RE-ENTRANCY; kernel/
    // rubato.hpp, "The stack") - then that run's entry's sixteen registers
    // (64) and its activate() frame (96), with a handler over that (48):
    // 240. 64 + 240 = 304: the high-water mark lies within 304 bytes of
    // the deeper of the two samples. Where there is F, the entry's
    // f-register save adds its 96 bytes to the second branch, and letter
    // j's float handler is the deepest handler - the float trampoline's 96
    // bytes, its body's 32 and a post's 16, 144: 64 + 32 + 64 + 96 + 96 +
    // 144 = 496. Without the hardware prologue the handlers' frames carry
    // their saves: TIM2's 96 (192 with the f-registers) and a post's 16,
    // so 368 (560).
    constexpr uint32_t below = hpe_build ? (fpu_part ? 496u : 304u) : (fpu_part ? 560u : 368u);
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
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED", " tick=",
          tick_ok ? "STK" : "FAILED", " ruler=STK", crlf);
    print(serial, "test_vx03_rubato - the preemptive kernel on the QingKe V4, the scheduler ",
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
BRIO_CH32_VECTOR(usart1_handler) {
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }

/// TIM2's compare: the edge disarmed, the post, and the ruler as the last
/// statement - letter c's near end.
BRIO_CH32_VECTOR(tim2_handler) {
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

/// Letter j's float edge (TIM3's compare): a float computed over the
/// live registers of the dispatch it interrupts, so the float trampoline,
/// then the post to High. Only the parts with the FPU enable it.
BRIO_CH32_VECTOR_FLOAT(tim3_handler) {
    FloatEdge::interrupts(FloatEdge::compare_interrupt(0), false);
    FloatEdge::clear_flags(FloatEdge::compare_flag(0));
    if constexpr (device::has_fpu) {
        if (float_edge_float) {
            float_edge_acc = float_edge_acc * 1.5f + 1.0f;
        }
    }
    brio::post<High>(Hit{1});
    edge_last = Ruler::now();
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    Edge::init();
    (void)Edge::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Pfic::enable(Edge::irq());
    if constexpr (device::has_fpu) {
        FloatEdge::init();
        (void)FloatEdge::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
        brio::Pfic::enable(FloatEdge::irq());
    }
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", ta_ceiling);
    bench.letter('b', "road 2: the chain inside the posts", tb_road2);
    bench.letter('c', "the latency of road 1", tc_latency);
    bench.letter('d', "the rulers, the scan and the post, in cycles", td_scan);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", te_depth2);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", tf_timers);
    bench.letter('g', "the request under the mask: served when the section ends", tg_mask);
    if constexpr (device::has_fpu) {
        bench.letter('j', "the FPU's registers and fcsr across a preemption, the save's cost",
                     tj_fpu<>);
    }
    bench.letter('i', "dark: every object idle, nothing lit, the nesting as set", ti_dark);
    bench.letter('h', "the stack's high-water mark", th_stack);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
