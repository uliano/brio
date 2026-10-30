// test_stm32f4_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the STM32F4: Stm32f4RubatoPlatform and the
// core's scheduling request (cortexm/schedule.hpp) against the model the
// host suite specifies (test/test_rubato).
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs INSIDE the kernel it tests: the program is a Rubato pack of
// five objects, from the highest priority down - Top, High, Mid, Low and
// Console. The Console runs the letters in its own dispatch and prints
// through Log, the console's LineWriter (util/line_writer.hpp), the one
// writer of the UART and no object of the pack: the Console is the object
// whose lines may wait for the ring, so a letter's lines leave as it
// prints them. The other objects write what they saw into a trace in
// memory, and the Console judges and prints it after - letters j and k
// excepted, whose Low and High print lines of their own. A post from a
// letter is therefore a post from the Console's dispatch: to an object
// above it, the kernel's road 2.
//
// NOTHING TO WIRE, NOTHING LIT. The console is the Nucleo-F446RE's
// ST-LINK virtual COM port (USART2 on PA2/PA3); the suite configures no
// other pad, and the LED stays dark (letter i checks its pad). The
// interrupt every road-1 letter lands is TIM9's compare, an NVIC line on
// APB2 whose timer clock is HCLK (180 MHz here), so a delay placed in its
// counts is a delay in core cycles; the ruler is the DWT's CYCCNT
// (stm32f4/dwt.hpp), one load, 5.6 ns a count.
//
// WHERE THE SCHEDULER RUNS is a constant of the image,
// `scheduler_in_thread_mode` below, and the letters whose verdict depends
// on it judge against it. Here it is true: PendSV returns into the
// scheduler in thread mode through a second exception frame and the
// scheduler leaves through SVC (cortexm/schedule.hpp), so PendSV is not
// active while the dispatches run, and an interrupt's post preempts the
// dispatch it lands in at any depth - the Console's own dispatch
// included, since the UART's interrupt starts it at the loop's base. The
// letters also carry what they expect of a scheduler run INSIDE PendSV,
// in handler mode (the constant false): there every dispatch runs inside
// PendSV, the Console's too, an interrupt's PendSV waits for the dispatch
// it landed in to return and the activate() around it to rescan, and
// road 2 nests only because a request made inside PendSV calls the
// scheduler; letters a, c and e show that shape's limit at depth 1 and 2.
//
// The letters (each bound's arithmetic beside its verdict in the code):
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms TIM9 2000 cycles ahead and spins 20000 at a
//      mark; the interrupt posts to High. The trace says where High's
//      dispatch landed - before Low resumes where road 1 is kept, after
//      Low's end in the handler-mode shape - and every entry reads the
//      ceiling at its own object's index; the Console reads its own, 4,
//      after the chain, and the UART's receive interrupt read the loop's
//      ceiling when the letter's key arrived - 255 at the base, or 0
//      while the base turn walks the time events. Ten runs, all alike.
//   b  ROAD 2: Low posts to Mid, Mid to High, inside the handlers; High
//      posts to Low. The trace nests High inside Mid inside Low, and
//      Low's second event is served after its first dispatch ends, never
//      inside it. Ten runs, all alike.
//   c  THE LATENCY OF ROAD 1, the Console's wait loop interrupted: CYCCNT
//      at the interrupt handler's last statement and at High's handler's
//      first, min, median and max over 100 events after one warm-up, the
//      console drained first; the bound is the sum of the M4's own cycle
//      counts along the path, written beside it. Where road 1 is not kept,
//      the letter measures and judges the deferral instead.
//   d  THE SCAN AND THE POST: activate() with nothing ready below the
//      Console's ceiling (four empty queues, the fifth object at the
//      ceiling), and from Mid's handler one post that asks for nothing
//      (to Low, below it) against one that runs High's dispatch inside it
//      (road 2), each the least of 16, the ruler's own two-read cost taken
//      off.
//   e  DEPTH 2: High's handler - started by road 2 (the Console's post),
//      then by road 1 (an interrupt over the Console) - spins at a mark
//      while TIM9 posts to Top. Top's entry lands before High resumes
//      where road 1 is kept at depth 2, after High's end in the handler-
//      mode shape; the latency (or the deferral) printed. Ten runs each.
//   f  TIME EVENTS UNDER PREEMPTION, in three parts. (1) Three one-shots
//      matured in one turn, the lowest object's armed last so walked
//      first, posted the way Rubato::run() posts them - the walk with the
//      ceiling held at 0, then the scheduler: nothing is dispatched during
//      the walk and the three are served highest first. (2) Low's handler
//      runs 200 turns of arm(), arm(), disarm(), most of each turn inside
//      a masked window; TIM9's interrupt lands there and its High
//      preempts at the window's end, arming two timers and disarming the
//      one Low keeps re-arming: the list stays whole - the next turns
//      fire every timer due once, the disarmed one never, nothing twice.
//      (3) The cycles of one arm(), its masked window inside them, by
//      CYCCNT around the call. The turns are run()'s, written out in the
//      Console's dispatch (base_turn(); the kernel loop itself is below
//      the Console and does not turn while a letter runs).
//   g  THE FPU: Low's handler sums a float series with a mark inside the
//      loop; High, started by the interrupt at the mark, sums another;
//      a second run has the interrupt handler compute a float too. Every
//      value bit-exact against the same functions run without preemption
//      - lazy stacking proved by the values, in the mode the scheduler
//      runs in.
//   i  DARK: every object but the Console idle, no time event armed,
//      TIM9's line quiet, the LED's pad not an output, the ceiling the
//      Console's own.
//   j  THE WRITER'S COST: a verdict-shaped line (60 bytes) printed four
//      times from Low's dispatch - the first after the letter's other
//      work, with the UART's handler inside (the run starts the idle
//      transmitter), the second right behind it, the third with
//      interrupts masked, the fourth in two halves, the texts and the
//      end - each the whole cost of the line, formatted into Low's stage
//      and published into the ring; against the same line printed
//      straight into the transport with interrupts masked and the
//      transport's run alone (2 and 60 bytes), least and most over 16
//      runs; since boot, the lines and bytes the writer published, the
//      most stages it held at once and the lines it lost.
//   k  TWO LINES INTERLEAVED: the Console composes a line in five pieces
//      with spins between them while TIM9's interrupt starts High, which
//      prints a whole line of its own; ten times, the console drained
//      before each so that High's line finds room in the ring (a line of
//      High's, an object that may not wait, finding none would be lost).
//      Both lines carry a "#<" mark, and bin/brio's judge reads every
//      marked line whole and twenty of them.
//   h  THE STACK, the last letter: main() painted the free stack below
//      its own frame down to the end of the image's RAM sections; the
//      high-water mark against the deepest stack pointer the dispatches
//      and the interrupt sampled in letters a..g plus what lies below a
//      sample (an exception frame with the FPU's 18 words, the handler's
//      own frames), the arithmetic beside the verdict.
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <bit>

#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/line_writer.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock;

namespace {

using namespace brio;

/// Where the scheduler runs in this image: true, in thread mode, PendSV
/// returning into it (cortexm/schedule.hpp); false would state a
/// scheduler run inside PendSV (the file header).
constexpr bool scheduler_in_thread_mode = true;

using P = Stm32f4RubatoPlatform<>;
using Ruler = CycleCounter;
using Edge = Tim<9>;

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;

/// The Console's index in the pack (the pack is below).
constexpr uint8_t console_index = 4;

/// The console's writer, the one writer of the UART: the default pool (a
/// long stage of 192 bytes, two small ones of 96), the Console the object
/// whose lines may wait - the letters run in its dispatch - and the
/// sizing counters on for letter j.
using Log = LineWriter<Serial, P,
                       LineWriterOptions{.max_sources = 8, .wait_from = console_index, .peak = true}>;
/// The writer's pool: its long stage and its two small ones.
constexpr uint8_t log_stages = 3;

TestBench<Log> bench;

// =============================================================================
// The trace, the stamps, the stack samples
// =============================================================================

struct Mark {
    char who;        ///< 'T', 'H', 'M', 'L'
    char what;       ///< 's' start, 'm' the mark, 'r' resumed, 'e' end
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

/// The trace, sixteen entries a line: at most 9 + 16 x 6 = 105 bytes, well
/// inside the Console's stage of 192.
void print_trace() {
    const uint8_t n = trace_len;
    for (uint8_t i = 0; i < n; ++i) {
        if (i % 16u == 0u) {
            print(Log{}, i == 0u ? "  trace:" : "        ");
        }
        print(Log{}, " ", trace[i].who, trace[i].what, trace[i].ceiling);
        if (i % 16u == 15u || i + 1u == n) {
            print(Log{}, crlf);
        }
    }
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

/// The lowest main-stack pointer a dispatch or the interrupt saw.
volatile uint32_t sp_low_dispatch = 0xFFFF'FFFFu;
volatile uint32_t sp_low_isr = 0xFFFF'FFFFu;

[[gnu::always_inline]] inline void sample_sp(volatile uint32_t& low) {
    const uint32_t sp = __get_MSP();
    if (sp < low) {
        low = sp;
    }
}

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

/// TIM9's compare `d` counts of HCLK from now, its interrupt armed: one
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
/// stamps CYCCNT (letter c's far end); then the stack sample, the trace's
/// start entry, the letter's action, the end entry. An event numbered 0
/// writes no entry (the flush's, below).
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
/// back. Whether it served.
bool activate_here() {
    const uint8_t base = P::ceiling;
    P::ceiling = 0;
    const bool served = K::activate(base).served;
    P::ceiling = base;
    return served;
}

/// The ceiling the UART's receive interrupt read when a byte arrived.
volatile uint8_t rx_ceiling = 0;

// =============================================================================
// The interrupt the road-1 letters land
// =============================================================================

volatile uint8_t edge_to = 1;           ///< 0: post to Top, 1: to High
volatile uint32_t edge_last = 0;        ///< CYCCNT at the handler's last statement
volatile uint32_t edge_hits = 0;
volatile bool edge_float = false;       ///< letter g: the handler computes a float too
volatile float edge_acc = 0.0f;

void reset_scenario() {
    trace_clear();
    Top::action = nullptr;
    High::action = nullptr;
    Mid::action = nullptr;
    Low::action = nullptr;
    tracing = true;
    edge_float = false;
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
/// its activate() serves whatever waits above the Console first - where
/// road 1 is not kept, the dispatch an interrupt's post left waiting.
void flush() {
    post<Low>(Hit{0});
}

/// Let the console's transmitter fall silent (at most 50 ms, 230 bytes at
/// 115200): its interrupt, landing in a measured path, would be measured.
void console_drain() {
    const uint32_t t0 = Ruler::now();
    while (!Serial::tx_idle() && Ruler::now() - t0 < 9'000'000u) {
    }
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
    // Mid's and Low's), the UART's handler posts to the Console alone,
    // SysTick's only counts the tick and Log, no object of the pack,
    // posts nothing; the one post to Top is TIM9's handler with edge_to
    // at 0, which only letter e's High sets, arming one edge that lands
    // inside its own spin, and in either shape Top's dispatch ends inside
    // letter e's (in the handler-mode shape at High's end and the
    // rescan); that handler's first statement disarms the compare
    // interrupt, so no edge outlives letter e (letter i checks the line
    // quiet). So a 0 here is the walk, never a dispatch.
    const bool at_base = rx_ceiling == 255u;
    const bool in_walk = rx_ceiling == 0u;
    print(Log{}, "  the UART's receive interrupt read the ceiling ", rx_ceiling,
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
    // handler-mode shape it waits for Low's end and the rescan.
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
            edge_in(2000u);      // 11 us into a 111 us spin: the edge lands inside it
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
    print(Log{}, "  ", alike, " of 10 runs traced ", want, crlf);
    if constexpr (scheduler_in_thread_mode) {
        bench.verdict("High's dispatch starts inside Low's, before Low resumes, every ceiling its "
                      "own index (10 runs)",
                      alike == 10u);
    } else {
        bench.verdict("handler mode: High's dispatch starts after Low's has ended - road 1 does "
                      "not preempt a dispatch inside PendSV - every ceiling its own index (10 runs)",
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
    print(Log{}, "  ", alike, " of 10 runs traced ", want, crlf);
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

/// The road-1 path from the interrupt handler's last statement to High's
/// first, in the M4's own cycle counts (DDI 0439B 3.9.1 and table 3-1)
/// over the release listing, each taken branch's refill at its three:
///   the handler's tail after the stamp (its store, the stack's
///   release, a pop of the PC)                                     9
///   the tail-chain into PendSV                                    6
///   PendSV's fourteen instructions (two pushes, two stores, a DSB)
///   and its return into thread mode, twelve                      32
///   the entry: cpsie, the call                                    5
///   activate(): nine registers saved and its frame (11), two
///   literals (4), the served flag cleared (1), Top's visit (the
///   ceiling's load and test, the queue's two cursors loaded and
///   compared in line, the branches: 14), High's (the queue's
///   address a literal, the two cursors, the compare, the taken
///   branch: 11), High's dispatch up to the stamp (the ceiling's
///   and the index's stores, the call to take(): 42)              83
///                                                               ---
///                                                               135
/// at zero wait states, the code warm in the ART's cache after the
/// uncounted first event, every refill taken at its most, every load at
/// two cycles. The count is short of the path: measured, 143 on every
/// event of a run no other interrupt touches but one in a hundred at
/// 148, 8 and 13 cycles over the count. So the quiet path is the
/// measured 143 to 148, not the count; an event over latency_quiet below
/// is one an interrupt landed in. On top of it, at most one SysTick
/// handler landing in the path - its entry (12), its fifteen
/// instructions (20), its return (12) and, when it lands on an exception
/// return, that return abandoned and started again (12): 58, measured
/// +45 and +58 - and two of the ART's line misses (12): 148 + 58 + 12 =
/// 218, the bound, from the quiet path's slowest measured event (the
/// count being short of it).
/// The letter reads the kernel tick around each event, and every event
/// over latency_quiet must have had a tick in it.
constexpr uint32_t latency_bound = 218u;

/// The quiet path's measured 143 to 148 and 11 to 16 cycles of margin:
/// well below the least a tick adds (45, so 188), above the path's own
/// spread (5).
constexpr uint32_t latency_quiet = 159u;

void tc_latency() {
    reset_scenario();
    tracing = false;
    if constexpr (scheduler_in_thread_mode) {
        // One warm-up event (the caches, the prefetch) not counted, then
        // 100; each waits at most 1 ms for High's first statement.
        console_drain();
        uint32_t missed = 0;
        // An event slower than the quiet path (latency_quiet) is one a
        // tick landed in: the kernel tick read around each says so.
        uint32_t slow = 0;
        uint32_t slow_ticked = 0;
        for (uint32_t i = 0; i <= latency_events; ++i) {
            const uint32_t r = High::runs;
            const uint32_t t0 = Ticker::ticks();
            edge_to = 1;
            edge_in(400u);
            if (!wait_runs(High::runs, r, 180'000u)) {
                ++missed;
                flush();
            }
            const bool ticked = Ticker::ticks() != t0;
            if (i > 0u) {
                const uint32_t d = High::entered - edge_last;
                latency[i - 1u] = d;
                if (d > latency_quiet) {
                    ++slow;
                    if (ticked) {
                        ++slow_ticked;
                    }
                }
            }
        }
        sort(latency, latency_events);
        const uint32_t lo = latency[0];
        const uint32_t med = latency[latency_events / 2u];
        const uint32_t hi = latency[latency_events - 1u];
        print(Log{}, "  road 1 over the Console's wait: the interrupt's last statement to High's "
              "first, min ", lo, " median ", med, " max ", hi);
        print(Log{}, " cycles over ", latency_events, " events (", missed,
              " missed); the three slowest ", latency[latency_events - 3u]);
        print(Log{}, " ", latency[latency_events - 2u], " ", hi, crlf);
        print(Log{}, "  events over ", latency_quiet, " cycles: ", slow, ", a kernel tick inside ",
              slow_ticked, " of them", crlf);
        bench.verdict("every interrupt's post started High within 1 ms", missed == 0u);
        bench.verdict("every event over the path's own count had a tick land in it",
                      slow == slow_ticked);
        bench.verdict("the latency stays under the path's bound (the arithmetic beside it)",
                      hi <= latency_bound);
    } else {
        // Handler mode: the Console's dispatch runs inside PendSV, so the
        // interrupt's PendSV waits. 1 ms of wait (180000 cycles, a hundred
        // times the path's own length) and High has not run; the flush
        // serves it, and the deferral is the wait plus the flush.
        uint32_t deferred = 0;
        uint32_t least = 0xFFFF'FFFFu;
        for (uint32_t i = 0; i < 10u; ++i) {
            const uint32_t r = High::runs;
            edge_to = 1;
            edge_in(400u);
            const bool ran = wait_runs(High::runs, r, 180'000u);
            if (!ran) {
                ++deferred;
                flush();
            }
            const uint32_t d = High::entered - edge_last;
            if (d < least) {
                least = d;
            }
        }
        // The wait starts before the edge is armed 400 counts ahead, so a
        // deferred High starts at least 180000 - 400 - the handler's own
        // cycles after the handler's last statement: 179000 is that, with
        // the handler's 600 cycles to spare.
        print(Log{}, "  handler mode: ", deferred, " of 10 interrupts left High waiting for "
              "1 ms; the least deferral ", least, " cycles", crlf);
        bench.verdict("handler mode: an interrupt's post over the Console's dispatch is served "
                      "only when that dispatch posts again or returns (10 of 10 deferred past "
                      "1 ms)",
                      deferred == 10u && least >= 179'000u);
    }
    reset_scenario();
}

// =============================================================================
// d - the scan and the post
// =============================================================================
volatile uint32_t post_quiet = 0;
volatile uint32_t post_road2 = 0;

void td_scan() {
    reset_scenario();
    tracing = false;
    const uint32_t ruler = ruler_cost();
    uint32_t scan = 0xFFFF'FFFFu;
    uint32_t scan_hi = 0;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        const bool served = activate_here();
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
    print(Log{}, "  the ruler's two reads ", ruler, " cycles", crlf);
    print(Log{}, "  activate() with nothing ready below the Console: ", scan, "..", scan_hi,
          " cycles (four empty queues, the fifth at the ceiling)", crlf);
    print(Log{}, "  from Mid: a post to Low (asks nothing) ", quiet,
          " cycles; a post to High (road 2, High's dispatch inside) ", road2,
          " cycles; the scheduling's share ", road2 > quiet ? road2 - quiet : 0u, crlf);
    // The scan, as the release listing has it: activate()'s entry saves
    // nine registers and opens a 28-byte frame, its exit closes the frame
    // and restores them with the return (10 + 1 and 1 + 13 cycles, DDI
    // 0439B table 3-1: 1 + N for an STM or LDM of N, 3 more for a load of
    // the PC: 25), two literal loads (4), the served flag cleared and
    // returned (2), the ceiling's load and test (3); per empty object the
    // queue's two cursors loaded and compared in line - empty() is forced
    // inline, no call - the compare's branch and the ceiling compared and
    // branched on: 8 cycles at two a load, 2 more for the queue's address,
    // a literal, on the three after Top (whose address the entry loaded),
    // 3 more for Top's taken branch to the next visit: 11 + 10 + 10, and
    // Low's 10 plus the taken branch that ends the pass, the Console's
    // index against the ceiling (3): 13. That is 25 + 4 + 2 + 3 + 11 + 10
    // + 10 + 13 = 78 at zero wait states, and the least of 16 runs from
    // the ART's cache: 104 is the sum with a third for refills and fetch
    // beyond the table's counts.
    bench.verdict("the scan of four empty queues, the least of 16, costs under 104 cycles",
                  scan < 104u);
    // The quiet post: post<Low>() out of line, 13 instructions with the
    // ceiling's compare, and the queue type's push(), 17 with the guard
    // (mrs, cpsid, msr) - 30 instructions and two calls and returns, four
    // pipeline refills of up to 3 cycles: about 50 at zero wait states.
    // 100 is twice that, the ART's misses (five wait states a line) and
    // the stores to the ruler's volatile results included. No scan and
    // no queue's empty() lie on this path: the push and the compare.
    bench.verdict("a post that asks nothing costs under 100 cycles (the push and one compare)",
                  quiet < 100u);
    // The road-2 post, the quiet one's 50 plus: the request's call, its
    // store, DSB and ISB (10); PendSV's entry, body and return into thread
    // mode (12 + 20 + 12); the entry (5); activate() at Mid's ceiling -
    // its frame (25: nine registers saved and restored with the return,
    // the 28-byte frame opened and closed), Top's and High's visits, each
    // queue's cursors compared in line (Top's 14: the ceiling's load and
    // test, the two cursors, the compares, its taken branch; High's 41:
    // the literal, the two cursors, the taken branch, 11, and the
    // dispatch's start up to take()'s return, 30), High's dispatch with
    // nothing to do (its stamps and counters, 30), the release (15), the
    // rescan of Top and High and Mid's index ending the pass (14 + 13);
    // the FP context cleared and the SVC's entry, three instructions and
    // return (7 + 12 + 6 + 12): 50 + 10 + 44 + 5 + 25 + 14 + 41 + 30 + 15
    // + 27 + 37 = 298 at zero wait states. The post's own part, the 50,
    // is the quiet post's count, and the quiet post measures 78 to 85
    // against it (the same instructions, the push and the compare): that
    // part is taken at the quiet post's own bound, 100 (its arithmetic
    // above), and the rest, 248 by the count, with a third for the
    // refills and the ART's misses (330.7, so 331): 100 + 331 = 431.
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    bench.verdict("and under 431 cycles, High's empty dispatch included", road2 < 431u);
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
                if (!wait_runs(High::runs, r, 180'000u)) {
                    flush();
                }
            }
            const uint32_t d = Top::entered - edge_last;
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
        print(Log{}, "  High started by road ", road, ": ", alike, " of 10 runs traced ", want,
              "; Top's first statement ", least, "..", most,
              " cycles after the interrupt's last", crlf);
        if constexpr (scheduler_in_thread_mode) {
            bench.verdict(road == 2 ? "road 1 at depth 2 (High by road 2): Top runs before High "
                                      "resumes (10 runs)"
                                    : "road 1 at depth 2 (High by road 1): Top runs before High "
                                      "resumes (10 runs)",
                          alike == 10u);
        } else {
            bench.verdict(road == 2 ? "handler mode, High by road 2: Top runs after High's end, "
                                      "the stage's limit (10 runs)"
                                    : "handler mode, High by road 1: Top runs after High's end, "
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
    //    arm()'s or disarm()'s masked window; TIM9's interrupt lands there
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
    // In handler mode High waits for Low's end, and its disarm of te_a is
    // then the last word: te_a never fires.
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
        if (t1 - t0 - r < least) {
            least = t1 - t0 - r;
        }
    }
    arm_cycles = least;
    te_m.disarm();
    TE::clear_all();
    reset_scenario();

    print(Log{}, "  part 1: fires High ", fired[21], ", Mid ", fired[31], ", Low ", fired[41]);
    print(Log{}, "; part 2: High landed in Low's turn ", high_landed, " of 200; fires a ", fired[51],
          ", b ", fired[52]);
    print(Log{}, ", c ", fired[53], ", d ", fired[54], "; one arm() ", arm_cycles, " cycles", crlf);
    bench.verdict("three timers matured in one turn: nothing dispatched during the walk "
                  "(the ceiling held at 0), every one queued",
                  walk_quiet);
    bench.verdict("and the scheduler after the walk served them highest first, once each",
                  in_order && part1);
    if constexpr (scheduler_in_thread_mode) {
        bench.verdict("High preempted Low's arm/disarm loop by road 1, inside it",
                      preempted && high_landed < 199u);
    } else {
        bench.verdict("handler mode: High ran after Low's arm/disarm loop had ended",
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
    // arm() over a list of one node (itself, re-armed): the call and its
    // frame, the guard (mrs, cpsid, msr: 3), now()'s volatile load, the
    // unlink walk to the node and its two stores, the four stores of the
    // link and the walk's test - 35 instructions by the listing, about 50
    // cycles with the refills; 100 is twice that, the ART's misses
    // included. The masked window is the guard's span, inside it.
    bench.verdict("one arm(), the least of 16, costs under 100 cycles, its masked window inside",
                  arm_cycles < 100u);
}

// =============================================================================
// g - the FPU
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

void tg_fpu() {
    reset_scenario();
    tracing = false;
    const float ref_low = series_low(400u, 0xFFFF'FFFFu);
    const float ref_high = series_high(300u);
    const float ref_edge = 2.0f * 1.5f + 1.0f;
    Low::action = [](uint8_t) { low_sum = series_low(400u, 200u); };
    High::action = [](uint8_t) { high_sum = series_high(300u); };
    bool exact = true;
    bool edge_exact = true;
    for (int run = 0; run < 2; ++run) {
        edge_float = run == 1;
        edge_acc = 2.0f;
        low_sum = 0.0f;
        high_sum = 0.0f;
        const uint32_t h = High::runs;
        post<Low>(Hit{1});
        (void)wait_runs(High::runs, h, 1000u);
        const bool ok = std::bit_cast<uint32_t>(static_cast<float>(low_sum)) ==
                            std::bit_cast<uint32_t>(ref_low) &&
                        std::bit_cast<uint32_t>(static_cast<float>(high_sum)) ==
                            std::bit_cast<uint32_t>(ref_high);
        exact = exact && ok;
        if (run == 1) {
            edge_exact = std::bit_cast<uint32_t>(static_cast<float>(edge_acc)) ==
                         std::bit_cast<uint32_t>(ref_edge);
        }
        print(Log{}, "  run ", run, run == 1 ? " (the handler's float too)" : "", ": Low ",
              hex(std::bit_cast<uint32_t>(static_cast<float>(low_sum))), " (",
              hex(std::bit_cast<uint32_t>(ref_low)), "), High ",
              hex(std::bit_cast<uint32_t>(static_cast<float>(high_sum))), " (",
              hex(std::bit_cast<uint32_t>(ref_high)), ")", crlf);
    }
    reset_scenario();
    bench.verdict(scheduler_in_thread_mode
                      ? "Low's and High's sums are bit-exact across the preemption (thread mode)"
                      : "Low's and High's sums are bit-exact, Low inside PendSV in handler mode",
                  exact);
    bench.verdict("and the handler's own float, computed over Low's live registers, is exact",
                  edge_exact);
}

// =============================================================================
// i - dark
// =============================================================================
void ti_dark() {
    reset_scenario();
    bench.verdict("Top, High, Mid and Low are idle: every queue empty",
                  Top::queue.empty() && High::queue.empty() && Mid::queue.empty() &&
                      Low::queue.empty());
    bench.verdict("no time event is armed", !TE::next_deadline().has_value());
    bench.verdict("TIM9's line is quiet (nothing pending)", !Nvic::pending(Edge::irq()));
    bench.verdict("the LED's pad PA5 is not an output", !Pin<'A', 5>::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// j - the writer's cost
// =============================================================================
/// A verdict-shaped line, the bench grammar's shape: "  cost  <50 characters>"
/// and CRLF, 60 bytes on the wire.
const char* const cost_name = "the ruler reads delay_us(999) at least hz x 999 us";
/// The same 60 bytes as one run, for the transport's run alone.
const char cost_line[] = "  cost  the ruler reads delay_us(999) at least hz x 999 us\r\n";
static_assert(sizeof cost_line == 61u, "the line is 60 bytes and its NUL");
volatile uint32_t line_cycles[3] = {0, 0, 0};
volatile uint32_t split_cycles[2] = {0, 0};
volatile uint32_t line_irq[4] = {0, 0, 0, 0};     ///< the UART handler's entries inside each line
volatile uint32_t line_ticks[4] = {0, 0, 0, 0};   ///< the kernel ticks inside each line
volatile uint32_t uart_entries = 0;               ///< the UART handler's entries since boot
volatile uint32_t cost_ruler = 0;

/// The least of a sample, and its most.
struct Spread {
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    void add(uint32_t v) {
        least = v < least ? v : least;
        most = v > most ? v : most;
    }
};

void tj_writer() {
    reset_scenario();
    tracing = false;
    cost_ruler = ruler_cost();
    // Low prints the line from its own dispatch, an object that may not
    // wait: each print formats into one of the small stages and, at its
    // crlf, publishes it into the ring - the whole cost of the line, there
    // is no dispatch after it. The first after the letter's other work, the
    // second right behind it, a third right behind with interrupts masked
    // (the core's own cycles), and a fourth in two halves - the four texts
    // into the stage, then the line's end that publishes it. The first
    // line's run starts the idle transmitter and the UART's handler lands
    // inside it, handing over the first byte; the transmitter takes that
    // byte into its shift register a bit time later (1560 cycles) and the
    // handler lands again, inside the second line - the 180 cycles the
    // second line measures over the third and the fourth. The counters
    // below read both, and the kernel tick, around each line.
    // Around each line, the UART handler's entries and the kernel ticks
    // that fell inside it: what the line's cycles carry beyond the core's.
    Low::action = [](uint8_t) {
        for (uint32_t k = 0; k < 2u; ++k) {
            const uint32_t i0 = uart_entries;
            const uint32_t k0 = Ticker::ticks();
            const uint32_t t0 = Ruler::now();
            print(Log{}, "  ", "cost", "  ", cost_name, crlf);
            line_cycles[k] = Ruler::now() - t0 - cost_ruler;
            line_irq[k] = uart_entries - i0;
            line_ticks[k] = Ticker::ticks() - k0;
        }
        {
            InterruptGuard g;
            const uint32_t i0 = uart_entries;
            const uint32_t k0 = Ticker::ticks();
            const uint32_t t0 = Ruler::now();
            print(Log{}, "  ", "cost", "  ", cost_name, crlf);
            line_cycles[2] = Ruler::now() - t0 - cost_ruler;
            line_irq[2] = uart_entries - i0;
            line_ticks[2] = Ticker::ticks() - k0;
        }
        const uint32_t i0 = uart_entries;
        const uint32_t k0 = Ticker::ticks();
        const uint32_t t0 = Ruler::now();
        print(Log{}, "  ", "cost", "  ", cost_name);
        const uint32_t t1 = Ruler::now();
        print(Log{}, crlf);
        const uint32_t t2 = Ruler::now();
        split_cycles[0] = t1 - t0 - cost_ruler;
        split_cycles[1] = t2 - t1 - cost_ruler;
        line_irq[3] = uart_entries - i0;
        line_ticks[3] = Ticker::ticks() - k0;
    };
    Spread cold;
    Spread warm;
    Spread warm_masked;
    Spread texts;
    Spread end;
    Spread direct_masked;
    Spread run_two;
    Spread run_sixty;
    bool written_all = true;
    uint32_t most_irq[4] = {0, 0, 0, 0};
    uint32_t most_ticks[4] = {0, 0, 0, 0};
    const uint16_t lost0 = Log::lost();
    for (int i = 0; i < 16; ++i) {
        // The ring empty, then the four lines from Low's dispatch.
        console_drain();
        const uint32_t b0 = Log::bytes_written();
        post<Low>(Hit{0});
        written_all = written_all && Log::bytes_written() - b0 == 240u;
        cold.add(line_cycles[0]);
        warm.add(line_cycles[1]);
        warm_masked.add(line_cycles[2]);
        texts.add(split_cycles[0]);
        end.add(split_cycles[1]);
        for (uint32_t k = 0; k < 4u; ++k) {
            most_irq[k] = line_irq[k] > most_irq[k] ? line_irq[k] : most_irq[k];
            most_ticks[k] = line_ticks[k] > most_ticks[k] ? line_ticks[k] : most_ticks[k];
        }
        // The oracle: the same line printed straight into the transport,
        // warm and masked - a write past the writer, for the measurement
        // alone, with the ring empty. A debug build checks the writer's
        // claim on every write, so there these writes say they are the
        // writer's (a release build compiles nothing of it).
        console_drain();
        if constexpr (debug_checks) {
            WriterClaim<Serial>::writing = true;
        }
        for (uint32_t k = 0; k < 2u; ++k) {
            InterruptGuard g;
            const uint32_t t0 = Ruler::now();
            print(Serial{}, "  ", "dirc", "  ", cost_name, crlf);
            const uint32_t d = Ruler::now() - t0 - cost_ruler;
            if (k == 1u) {
                direct_masked.add(d);
            }
        }
        // And the transport's run alone, warm and masked: a run of two
        // bytes and the line's sixty as one run, the shape of a publish.
        console_drain();
        for (uint32_t k = 0; k < 2u; ++k) {
            InterruptGuard g;
            uint32_t t0 = Ruler::now();
            print(Serial{}, std::string_view("  ", 2));
            const uint32_t two = Ruler::now() - t0 - cost_ruler;
            t0 = Ruler::now();
            print(Serial{}, std::string_view(cost_line, 60));
            const uint32_t sixty = Ruler::now() - t0 - cost_ruler;
            if (k == 1u) {
                run_two.add(two);
                run_sixty.add(sixty);
            }
        }
        if constexpr (debug_checks) {
            WriterClaim<Serial>::writing = false;
        }
    }
    reset_scenario();
    print(Log{}, "  a verdict-shaped line, 60 bytes, over 16 runs: printed from Low's dispatch ",
          cold.least, "..", cold.most, " cycles, warm ", warm.least, "..", warm.most,
          ", warm and masked ", warm_masked.least, "..", warm_masked.most, crlf);
    print(Log{}, "  of a warm one: the four texts into the stage ", texts.least, "..", texts.most,
          " cycles, the line's end that publishes it ", end.least, "..", end.most, crlf);
    print(Log{}, "  inside the four lines (cold, warm, masked, split), the most over 16 runs: the UART "
          "handler's entries ", most_irq[0], ' ', most_irq[1], ' ', most_irq[2], ' ', most_irq[3],
          ", kernel ticks ", most_ticks[0], ' ', most_ticks[1], ' ', most_ticks[2], ' ', most_ticks[3],
          crlf);
    print(Log{}, "  print() straight into the transport, warm and masked ", direct_masked.least, "..",
          direct_masked.most, " cycles", crlf);
    print(Log{}, "  the transport's run, warm and masked: 2 bytes ", run_two.least, "..", run_two.most,
          ", 60 bytes ", run_sixty.least, "..", run_sixty.most, " cycles", crlf);
    print(Log{}, "  since boot: ", Log::lines_written(), " lines, ", Log::bytes_written(),
          " bytes published; the most stages held at once ", Log::peak_stages(), " of ", log_stages,
          "; lost ", Log::lost(), crlf);
    bench.verdict("each of Low's four lines was published, 60 bytes each", written_all);
    bench.verdict("nothing lost while measuring", Log::lost() == lost0);
    bench.verdict("the stage peak within the pool", Log::peak_stages() >= 1u && Log::peak_stages() <= log_stages);
}

// =============================================================================
// k - two lines interleaved under preemption
// =============================================================================
volatile uint32_t high_lines = 0;

void tk_interleave() {
    reset_scenario();
    tracing = false;
    high_lines = 0;
    High::action = [](uint8_t) {
        print(Log{}, "#<H ", high_lines, ": a whole line from High's dispatch, started by TIM9 :H>#", crlf);
        high_lines = high_lines + 1u;
    };
    const uint16_t lost0 = Log::lost();
    uint32_t inside = 0;
    for (uint32_t run = 0; run < 10u; ++run) {
        // The ring empty first: High may not wait, and its line must find
        // the room the Console's lines would otherwise have taken.
        console_drain();
        const uint32_t h0 = High::runs;
        print(Log{}, "#<C ", run, ": the Console composes this line in pieces");
        // TIM9 in 111 us, inside the four 56 us spins between the pieces:
        // High's dispatch preempts the Console's and prints a whole line.
        edge_to = 1;
        edge_in(20000u);
        for (uint32_t piece = 0; piece < 4u; ++piece) {
            spin(10000u);
            print(Log{}, ", piece ", piece);
        }
        const bool landed = High::runs != h0;
        print(Log{}, " :C>#", crlf);
        inside += landed ? 1u : 0u;
    }
    reset_scenario();
    // What bin/brio's judge counts: every line marked "#<" whole, twenty.
    print(Log{}, "#count 20", crlf);
    bench.verdict("High's dispatch landed inside every one of the Console's lines (10 of 10)",
                  inside == 10u && high_lines == 10u);
    bench.verdict("no line lost", Log::lost() == lost0);
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
    print(Log{}, "  painted ", paint_top - paint_bottom, " bytes below main's frame; used ", used,
          " bytes; the deepest stack pointer sampled: a dispatch at ", deepest_dispatch,
          ", the interrupt at ", deepest_isr, crlf);
    // A dispatch's sample is taken inside activate() (the dispatch is
    // inlined there), whose nine saved registers and 28-byte frame (64
    // bytes) are above it. Below the deepest one lies at most, by the
    // release listing: the letter's action and what it calls - the
    // widest is letter j's, which saves 32 bytes and prints through the
    // writer, whose deepest path is end_line() 16, publish() 64 (nine
    // registers and a 28-byte frame), put() 24, the transport's
    // write_bulk() 32 and the runtime's memcpy, a leaf: 168; an
    // interrupt landing there, an extended frame of 26 words and its
    // alignment word (108); and below that either the deepest handler's
    // own frames (the UART's 32, its post and push 16: 48) or, the
    // handler gone and its post pending PendSV, PendSV's two words and an
    // activate() frame that finds nothing to serve (8 + 64 = 72; the fake
    // frame's 32 are popped before activate() pushes). 168 + 108 + 72 =
    // 348: the high-water mark lies within 350 bytes of the deeper of the
    // two samples.
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    bench.verdict("the paint reached the deepest sampled stack pointer", used >= deepest);
    bench.verdict("the high-water mark is within 350 bytes of the deepest sample",
                  used <= deepest + 350u);
}

// =============================================================================
// the Console's turn
// =============================================================================
bool clock_ok = false;
bool tick_ok = false;
bool ruler_ok = false;

void console_banner() {
    print(Log{}, crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
          tick_ok ? "SysTick" : "FAILED", " ruler=", ruler_ok ? "DWT" : "FAILED", crlf);
    print(Log{}, "test_stm32f4_rubato - the preemptive kernel on the STM32F4, the scheduler ",
          scheduler_in_thread_mode ? "in thread mode" : "inside PendSV (handler mode)", crlf);
    bench.menu();
}

void console_turn() {
    uint8_t c = 0;
    while (Serial::read_byte(c)) {
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(Log{}, static_cast<char>(c), crlf);
        if (c == '?') {
            console_banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(Log{}, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART2_IRQHandler() {
    uart_entries = uart_entries + 1u;
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

/// TIM9's compare (its line shared with TIM1's break, which nothing here
/// enables): the edge disarmed, the post, and CYCCNT as the last
/// statement - letter c's near end.
extern "C" void TIM1_BRK_TIM9_IRQHandler() {
    Edge::interrupts(Edge::compare_interrupt(0), false);
    Edge::clear_flags(Edge::compare_flag(0));
    sample_sp(sp_low_isr);
    edge_hits = edge_hits + 1u;
    if (edge_float) {
        edge_acc = edge_acc * 1.5f + 1.0f;
    }
    if (edge_to == 0u) {
        brio::post<Top>(Hit{1});
    } else {
        brio::post<High>(Hit{1});
    }
    edge_last = Ruler::now();
}

extern "C" uint32_t __bss_end;

int main() {
    // Paint the free stack: from 64 bytes below main's own stack pointer
    // down to the end of the image's RAM sections (.bss, then .noinit with
    // the panic record: the linker script names no stack limit, so the
    // higher of the two ends, 8-aligned, plus a guard of 64 bytes).
    {
        const uint32_t sp = __get_MSP();
        uint32_t bottom = reinterpret_cast<uint32_t>(&__bss_end);
        const uint32_t noinit_end = reinterpret_cast<uint32_t>(&P::panic_record() + 1);
        if (noinit_end > bottom) {
            bottom = noinit_end;
        }
        bottom = ((bottom + 7u) & ~7u) + 64u;
        paint_top = (sp - 64u) & ~3u;
        paint_bottom = bottom;
        for (uint32_t a = bottom; a < paint_top; a += 4u) {
            *reinterpret_cast<volatile uint32_t*>(a) = paint;
        }
    }

    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    Log::init();
    tick_ok = brio::Ticker::init(clock);
    ruler_ok = Ruler::init();
    Edge::init();
    (void)Edge::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Nvic::enable(Edge::irq());
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", ta_ceiling);
    bench.letter('b', "road 2: the chain inside the posts", tb_road2);
    bench.letter('c', "the latency of road 1", tc_latency);
    bench.letter('d', "the scan and the post, in cycles", td_scan);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", te_depth2);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", tf_timers);
    bench.letter('g', "the FPU's registers across a preemption", tg_fpu);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('j', "the writer's cost: a line's cycles, the pool's peak, the losses", tj_writer);
    bench.letter('k', "two lines interleaved under preemption, each whole", tk_interleave);
    bench.letter('h', "the stack's high-water mark", th_stack);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
