// test_stm32_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the STM32G0: Stm32g0RubatoPlatform and the
// core's scheduling request (cortexm/schedule.hpp) on the Cortex-M0+,
// against the model the host suite specifies (test/test_rubato).
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
// NOTHING TO WIRE, NOTHING LIT. The console is the Nucleo-G0B1RE's
// ST-LINK virtual COM port (USART2 on PA2/PA3, AF1); the suite configures
// no other pad, and the LED stays dark (letter i checks its pad). The
// interrupt every road-1 letter lands is TIM14's compare, an NVIC line of
// its own on TIMPCLK, which is HCLK here (stm32g0/clock.hpp's Clock<>
// pins the APB prescaler at 1, tim.hpp's "The clock"), so a delay placed
// in its counts is a delay in core cycles.
//
// THE RULER. This core has no DWT cycle counter (ARMv6-M). Two rulers
// were weighed, and letter d prints both costs: the kernel ticker's
// BasicTicker::cycles(), SysTick's position composed with the tick count
// (cortexm/ticker.hpp, five reads and a consistency check, a cycle of
// resolution), and TIM2, this family's one 32-bit timer, free-running at
// prescaler 0 on TIMPCLK = HCLK: one load of CNT over the APB, a cycle of
// resolution, a wrap every 67 s. TIM2 is the cheaper read by a wide
// margin and is the ruler here (`Ruler` below); letter d checks it
// against the ticker's composed cycles over a span, both being HCLK.
// TIM2 and TIM14 are not the console's (USART2) nor the ticker's
// (SysTick).
//
// WHERE THE SCHEDULER RUNS: in thread mode, PendSV returning into it
// through a second exception frame and the scheduler leaving through SVC
// (cortexm/schedule.hpp), so PendSV is not active while the dispatches
// run, and an interrupt's post preempts the dispatch it lands in at any
// depth - the Console's own dispatch included, since the UART's interrupt
// starts it at the loop's base.
//
// The letters (each bound's arithmetic beside its verdict in the code):
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms TIM14 2000 cycles ahead and spins 20000 at a
//      mark; the interrupt posts to High. The trace says High's dispatch
//      landed before Low resumes, and every entry reads the ceiling at its
//      own object's index; the Console reads its own, 4, after the chain,
//      and the UART's receive interrupt read the loop's ceiling when the
//      letter's key arrived - 255 at the base, or 0 while the base turn
//      walks the time events. Ten runs, all alike.
//   b  ROAD 2: Low posts to Mid, Mid to High, inside the handlers; High
//      posts to Low. The trace nests High inside Mid inside Low, and
//      Low's second event is served after its first dispatch ends, never
//      inside it. Ten runs, all alike.
//   c  THE LATENCY OF ROAD 1, the Console's wait loop interrupted: the
//      ruler at the interrupt handler's last statement and at High's
//      handler's first, min, median and max over 100 events after one
//      warm-up, the console drained first; the bound is the sum of the
//      M0+'s own cycle counts along the path, written beside it.
//   d  THE RULERS, THE SCAN AND THE POST: the two rulers' two-read costs
//      and their agreement over a span; activate() with nothing ready
//      below the Console's ceiling (four empty queues, the fifth object at
//      the ceiling), and from Mid's handler one post that asks for nothing
//      (to Low, below it) against one that runs High's dispatch inside it
//      (road 2), each the least of 16, the ruler's own cost taken off.
//   e  DEPTH 2: High's handler - started by road 2 (the Console's post),
//      then by road 1 (an interrupt over the Console) - spins at a mark
//      while TIM14 posts to Top. Top's entry lands before High resumes;
//      the latency printed. Ten runs each.
//   f  TIME EVENTS UNDER PREEMPTION, in three parts. (1) Three one-shots
//      matured in one turn, the lowest object's armed last so walked
//      first, posted the way Rubato::run() posts them - the walk with the
//      ceiling held at 0, then the scheduler: nothing is dispatched during
//      the walk and the three are served highest first. (2) Low's handler
//      runs 200 turns of arm(), arm(), disarm(), most of each turn inside
//      a masked window; TIM14's interrupt lands there and its High
//      preempts at the window's end, arming two timers and disarming the
//      one Low keeps re-arming: the list stays whole - the next turns
//      fire every timer due once, the disarmed one never, nothing twice.
//      (3) The cycles of one arm(), its masked window inside them. The
//      turns are run()'s, written out in the Console's dispatch
//      (base_turn(); the kernel loop itself is below the Console and does
//      not turn while a letter runs).
//   g  THE REQUEST UNDER PRIMASK (this core has no FPU, so no FPU letter):
//      a post to a higher object made inside a critical section pends
//      PendSV and is NOT served inside the section - the receiver's entry
//      follows the section's last mark - and is served when the section
//      ends, before the statement after it; from the Console (to Mid) and
//      from Low's dispatch (to High), ten runs each, with PENDSVSET read
//      inside the section and the cycles from the section's last
//      statement to the receiver's first, the kernel tick read around
//      each event as in letter c: the events no tick landed in judged
//      against the path's bound, those a tick did reported and judged
//      against the bound with one tick's cost on it.
//   i  DARK: every object but the Console idle, no time event armed,
//      TIM14's line quiet, the LED's pad not an output, the ceiling the
//      Console's own, interrupts enabled.
//   h  THE STACK, the last letter: main() painted the free stack below
//      its own frame down to the end of the image's RAM sections; the
//      high-water mark against the deepest stack pointer the dispatches
//      and the interrupt sampled in letters a..g plus what lies below a
//      sample (a basic exception frame, the handlers' own frames), the
//      arithmetic beside the verdict.
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32g0RubatoPlatform<>;
using Edge = Tim<14>;
using Clock32 = Tim<2>;

/// TIM2 free-running at HCLK: one load of CNT, a count a cycle.
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return Clock32::count(); }
};

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}};
using Serial = Uart<2, console_pins>;
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

/// TIM14's compare `d` counts of HCLK from now, its interrupt armed: one
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
/// back. Whether it served.
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

/// Let the console's transmitter fall silent (at most 50 ms, 3.2 M
/// cycles; 230 bytes at 115200 are 20 ms): its interrupt, landing in a
/// measured path, would be measured.
void console_drain() {
    const uint32_t t0 = Ruler::now();
    while (!Serial::tx_idle() && Ruler::now() - t0 < 3'200'000u) {
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
    // Mid's and Low's), the UART's handler posts to the Console alone and
    // SysTick's only counts the tick, and the one post to Top is TIM14's
    // handler with edge_to at 0, which only letter e's High sets, arming
    // one edge that lands inside its own spin; that handler's first
    // statement disarms the compare interrupt, so no edge outlives letter
    // e (letter i checks the line quiet). So a 0 here is the walk, never
    // a dispatch.
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

    const char* const want = "Ls3 Lm3 Hs1 He1 Lr3 Le3";
    uint8_t alike = 0;
    bool ceiling_back = true;
    bool once = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        Low::action = [](uint8_t) {
            note('L', 'm');
            edge_to = 1;
            edge_in(2000u);      // 31 us into a 312 us spin: the edge lands inside it
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

/// The road-1 path from the interrupt handler's last statement to High's
/// first, in the Cortex-M0+'s own cycle counts over the release listing.
/// The instruction counts are the core's table (the RP2040 datasheet's
/// 2.4.3.3, table 81, the copy of the Cortex-M0+'s figures this desk
/// has: a load or store 2, BL 3, BX 2, a taken branch 2, PUSH and POP
/// 1 + N, a POP into the PC 3 + N, MRS and MSR 3, DSB and ISB 3); the
/// exception's are the Cortex-M0+'s published figures - an entry of
/// 15 cycles, which 2.4.3.6.1 of the same datasheet quotes, a return of
/// 10 and a tail-chain of 11, the last two taken from the design brief
/// and not read in the core's TRM (DDI 0484C is not on this desk):
///   the handler's tail after the stamp's load (a literal, the store,
///   a pop of four into the PC)                                    11
///   the tail-chain into PendSV                                    11
///   PendSV's twelve instructions to the BX (two pushes, two
///   stores, a literal: 17; the body carries no barrier on this core,
///   cortexm/schedule.hpp - the count below keeps the three the DSB
///   took when this bound was derived, so the bound stands by three),
///   the BX and the return into thread mode (12)                  32
///   the entry: cpsie, the call                                     4
///   activate(): five registers saved and the frame (8), the
///   ceiling's load and test (12), Top's visit (the queue's two
///   cursors loaded and compared in line, the branch not taken: 8),
///   High's (the ceiling's compare and its taken branch, the cursors
///   and the taken branch: 14), the raise of the ceiling and of the
///   index of the object being dispatched (13), the call to take()
///   and its body (25), take()'s result tested and the stamp's
///   address (6)                                                   86
///                                                                ---
///                                                                144
/// at zero wait states. The flash answers at two (RM0444 3.3.4) behind
/// the prefetch and two cache lines (3.3.5), so each of the path's 9
/// discontinuities - the exception return and the vector fetch, the BX
/// into thread mode, two calls (activate() and take()) and take()'s
/// return, High's two taken branches and take()'s own when its cursor
/// wraps - refills a line at up to 3 cycles, and each of its six
/// literal loads (the handler's tail's, PendSV's, and activate()'s
/// four: the ceiling's address, the two queues, the dispatching
/// index's) reads the flash at 2: 144 + 27 + 12 = 183, the count with
/// every refill paid (measured 180..181 on every event no other
/// interrupt touches). On top of it, at most one SysTick handler landing
/// in the path: its entry (15), its thirteen instructions with three
/// literals (27), its return (10), its vector and first line refilled
/// (6) and, landing on an exception return, that return abandoned and
/// begun again (12): 70. The bound is the two, 253, and a fifth of the
/// path's own count (36.6, taken as 37): 290. The letter reads the
/// kernel tick around each event, and every event over 183 must have
/// had a tick in it.
constexpr uint32_t latency_count = 183u;
constexpr uint32_t latency_bound = 290u;

void tc_latency() {
    reset_scenario();
    tracing = false;
    // One warm-up event (the cache, the prefetch) not counted, then 100;
    // each waits at most 1 ms for High's first statement.
    console_drain();
    uint32_t missed = 0;
    // An event slower than the path's own count is one a tick landed in:
    // the kernel tick read around each says so.
    uint32_t slow = 0;
    uint32_t slow_ticked = 0;
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
            const uint32_t d = High::entered - edge_last;
            latency[i - 1u] = d;
            if (d > latency_count) {
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
    print(serial, "  road 1 over the Console's wait: the interrupt's last statement to High's "
          "first, min ", lo, " median ", med, " max ", hi, " cycles over ", latency_events,
          " events (", missed, " missed); the three slowest ", latency[latency_events - 3u],
          " ", latency[latency_events - 2u], " ", hi, crlf);
    print(serial, "  events over ", latency_count, " cycles: ", slow, ", a kernel tick inside ",
          slow_ticked, " of them", crlf);
    bench.verdict("every interrupt's post started High within 1 ms", missed == 0u);
    bench.verdict("every event over the path's own count had a tick land in it",
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
    // The two rulers over one span of about 1 ms of HCLK, crossing a
    // reload: TIM2's delta against the composed cycles' delta, each pair
    // of reads taken back to back in the same order.
    uint32_t agree_hi = 0;
    for (int i = 0; i < 4; ++i) {
        const uint32_t c0 = Ticker::cycles();
        const uint32_t r0 = Ruler::now();
        spin(one_ms + 1000u * static_cast<uint32_t>(i));
        const uint32_t c1 = Ticker::cycles();
        const uint32_t r1 = Ruler::now();
        const uint32_t dc = c1 - c0;
        const uint32_t dr = r1 - r0;
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
    print(serial, "  the ruler's two reads (TIM2's CNT) ", ruler,
          " cycles; the ticker's composed cycles() two reads ", ticker_ruler, " cycles", crlf);
    print(serial, "  over 1..1.05 ms the two rulers differ by at most ", agree_hi, " cycles", crlf);
    print(serial, "  activate() with nothing ready below the Console: ", scan, "..", scan_hi,
          " cycles (four empty queues, the fifth at the ceiling)", crlf);
    print(serial, "  from Mid: a post to Low (asks nothing) ", quiet,
          " cycles; a post to High (road 2, High's dispatch inside) ", road2,
          " cycles; the scheduling's share ", road2 > quiet ? road2 - quiet : 0u, crlf);
    // The rulers. Each pair of reads in the span is taken back to back,
    // so the two deltas differ by the skew between the pairs: at most the
    // composed read's own length (its two-read cost, about 65) plus one
    // retry of its loop when a reload falls between its reads (another
    // 65); 160 is the two with a fifth.
    bench.verdict("TIM2 reads cheaper than the ticker's composed cycles()", ruler < ticker_ruler);
    bench.verdict("the two rulers agree over the span within 160 cycles (both count HCLK)",
                  agree_hi <= 160u);
    // The bounds below are the release listing's count at zero wait
    // states (the core's table, letter c's comment), plus every refill
    // the path can pay - up to 3 cycles at each call, return and taken
    // branch (two flash wait states behind the prefetch), 2 at each
    // literal load - and a fifth of the whole as the margin.
    // The scan: the call (3); activate()'s frame and the ceiling's test
    // (20); per empty object the queue's two cursors loaded and compared
    // in line, with the ceiling's compare (Top 8, High, Mid and Low 13
    // each); the Console's index ending the pass (6) and the return (11):
    // 87. Its 6 discontinuities - the call, High's, Mid's and Low's taken
    // branches, the jump to the exit, the return - (18) and 5 literals
    // (10): 115; 138 with the fifth.
    bench.verdict("the scan of four empty queues, the least of 16, costs under 138 cycles",
                  scan < 138u);
    // The quiet post, a path that asks no queue whether it is empty: the
    // event's setup and the call between the two reads (10), post<Low>()'s
    // prefix (a push, a literal, two moves, the call: 10), the queue's
    // push() under its guard (mrs, cpsid, msr: 47), the ceiling's compare
    // and the return (16): 83, less the one load the ruler's cost takes
    // off: 81. Its 7 discontinuities (21) and 2 literals (4): 106; 127
    // with the fifth.
    bench.verdict("a post that asks nothing costs under 127 cycles (the push and one compare)",
                  quiet < 127u);
    // The road-2 post, the quiet one's 81 plus: the request's call and
    // body (15); PendSV's entry, body and return into thread mode (15 +
    // 20 + 12); the entry (4); activate() at Mid's ceiling - its frame and
    // test (20), Top's visit (8), High's to the dispatch (the visit 14,
    // the raises of the ceiling and of the dispatching index 13, take()
    // 25, its result and the stamp's address 6: 58), High's dispatch with
    // nothing to do, the release and the two restores (87), the rescan of
    // Top and High and Mid's index ending the pass with the return (49);
    // the SVC's entry, three instructions and return (32); the request's
    // and the post's returns (7): 408. Its 31 discontinuities at most -
    // 15 outside activate() (the post's, the push's, the request's calls,
    // branches and returns, the two exception entries and returns, the
    // entry's call and activate()'s return), 16 inside it (the visits'
    // taken branches, take() and release() each called, returning and
    // branching on a wrapped cursor, the dispatch's four taken branches,
    // the two jumps back to the pass) - (93) and 18 literals (36): 537;
    // 645 with the fifth (644.4).
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    bench.verdict("and under 645 cycles, High's empty dispatch included", road2 < 645u);
}

// =============================================================================
// e - depth 2
// =============================================================================
void te_depth2() {
    const char* const want = "Hs1 Hm1 Ts0 Te0 Hr1 He1";
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
        print(serial, "  High started by road ", road, ": ", alike, " of 10 runs traced ", want,
              "; Top's first statement ", least, "..", most,
              " cycles after the interrupt's last", crlf);
        bench.verdict(road == 2 ? "road 1 at depth 2 (High by road 2): Top runs before High "
                                  "resumes (10 runs)"
                                : "road 1 at depth 2 (High by road 1): Top runs before High "
                                  "resumes (10 runs)",
                      alike == 10u);
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
    //    arm()'s or disarm()'s masked window; TIM14's interrupt lands there
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
    // listing: the call (5); the frame and the guard's mrs and cpsid
    // (11); the unlink, out of line - the head's load, the walk's test,
    // the two stores of the unlink and the node's three, the return (27);
    // now()'s volatile load, the deadline and the node's four stores, the
    // link at the list's tail, the msr and the return (44): 87 at zero
    // wait states, the masked window the 74 from the cpsid to the msr.
    // Its 5 discontinuities (15) and 5 literals (10): 112; 135 with a
    // fifth (letter d's comment says how the bounds are drawn).
    bench.verdict("one arm(), the least of 16, costs under 135 cycles, its masked window inside",
                  arm_cycles < 135u);
}

// =============================================================================
// g - the request under PRIMASK
// =============================================================================
volatile bool pend_inside = true;       ///< PENDSVSET read inside every section
volatile bool ran_inside = false;       ///< the receiver's runs moved inside a section
volatile uint32_t section_last = 0;     ///< the ruler at a section's last statement

[[gnu::always_inline]] inline bool pendsv_pending() {
    return (SCB->ICSR & SCB_ICSR_PENDSVSET_Msk) != 0u;
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
        const bool pending = pendsv_pending();
        note(who, 'p');
        pend_inside = pend_inside && pending;
        ran_inside = ran_inside || Receiver::runs != r;
        section_last = Ruler::now();
    }
    note(who, 'a');
}

/// From the stamp's load to Mid's, by the release listing: the stamp's
/// store with its literal (4) and the guard's msr (3) that lifts the
/// mask; PendSV's entry from thread mode (15), its body and its return
/// into thread mode (32); the entry (4); activate() at the Console's
/// ceiling - its frame and test (20), Top's visit (8), High's (13),
/// Mid's to the stamp (the visit 14, the raises 13, take() 25, its result
/// and the stamp's address 6: 58): 157 at zero wait states. Its 8
/// discontinuities - PendSV's vector and its return into thread mode,
/// the call to activate(), High's and Mid's taken branches, take()'s
/// call, its branch on a wrapped cursor and its return - (24) and 7
/// literals (the stamp's, PendSV's, the ceiling's address, three queues,
/// the dispatching index's: 14): 195, the path's own count with every
/// refill paid; 234 with a fifth (letter d's comment says how the bounds
/// are drawn). A kernel tick (1 ms, 64000 cycles, so at most one in a
/// path of two hundred) lengthens an event in one of two ways. Matured
/// inside the section, SysTick (priority 0, above PendSV) is taken at the
/// msr before PendSV, which then tail-chains in (11) instead of entering
/// from thread mode (15): the handler's entry (15), its body (27) and
/// its first line refilled (6) less the 4 saved, 44. Landing in the path
/// after the msr, it costs letter c's 70 (its entry, body, return,
/// refills and an exception return begun again). The worse, 70, on the
/// bound: 304. The letter reads the kernel tick around each event:
/// the events nothing lengthened are judged against 234, every event
/// over 195 must have had a tick in it, and a ticked event is judged
/// against 304.
constexpr uint32_t primask_count = 195u;
constexpr uint32_t primask_bound = 234u;
constexpr uint32_t primask_tick_bound = 304u;

void tg_primask() {
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
    // The events no tick landed in, and those one did.
    uint32_t clean = 0;
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    // Printed after the loop: a print inside it would put the UART's
    // interrupt in the next events.
    uint32_t ticked_cycles[10];
    uint32_t ticked = 0;
    uint32_t ticked_most = 0;
    // An event slower than the path's own count is one a tick landed in.
    uint32_t slow = 0;
    uint32_t slow_ticked = 0;
    bool back = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        const uint32_t t0 = Ticker::ticks();
        post_in_section<Mid>('C');
        {
            const bool tick_in = Ticker::ticks() != t0;
            const uint32_t d = Mid::entered - section_last;
            if (tick_in) {
                ticked_cycles[ticked] = d;
                ++ticked;
                ticked_most = d > ticked_most ? d : ticked_most;
            } else {
                ++clean;
                least = d < least ? d : least;
                most = d > most ? d : most;
            }
            if (d > primask_count) {
                ++slow;
                if (tick_in) {
                    ++slow_ticked;
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
    if (clean > 0u) {
        print(serial, "  the section's last statement to Mid's first: ", least, "..", most,
              " cycles over the ", clean, " events no tick landed in", crlf);
    }
    print(serial, "  events over ", primask_count, " cycles: ", slow, ", a kernel tick inside ",
          slow_ticked, " of them; ", ticked, " of 10 events with a tick inside", crlf);
    for (uint32_t k = 0; k < ticked; ++k) {
        print(serial, "  an event a kernel tick landed in: ", ticked_cycles[k], " cycles", crlf);
    }
    bench.verdict("PendSV stood pending inside every section, and no receiver ran inside one",
                  pend_inside && !ran_inside);
    bench.verdict("the Console's post to Mid under PRIMASK is served when the section ends, "
                  "before the statement after it (10 runs)",
                  alike_console == 10u);
    bench.verdict("Low's post to High under PRIMASK likewise, inside Low's dispatch (10 runs)",
                  alike_low == 10u);
    // The bounds' arithmetic is beside primask_count above.
    bench.verdict("the section's end to the receiver's first statement under 234 cycles, "
                  "in every event no tick landed in",
                  most < primask_bound);
    bench.verdict("every event over the path's own count (195) had a tick land in it",
                  slow == slow_ticked);
    bench.verdict("an event a tick landed in under 304 cycles (the bound and one tick)",
                  ticked_most < primask_tick_bound);
    bench.verdict("the ceiling is the Console's again, interrupts enabled", back);
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
    bench.verdict("TIM14's line is quiet (nothing pending)", !Nvic::pending(Edge::irq()));
    bench.verdict("the LED's pad PA5 is not an output", !Pin<'A', 5>::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
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
    // A dispatch's sample is taken inside activate(), whose five saved
    // registers and 36-byte frame (56 bytes) are above it. Below the
    // deepest one lies at most, by the release listing: the letter's
    // action and what it calls (the deepest, letter d's, a 40-byte frame
    // calling a post, 8, and the queue's push, 8: 56; letter g's saves 32
    // and calls note(), 20: 52); an interrupt landing there, a
    // basic frame of 8 words and its alignment word (36: this core has
    // no FPU words); and below that either the deepest handler's own
    // frames (the UART's 32 with its post inline, TIM14's 16 with a
    // post's 8 and a push's 8: 32) or, the handler gone and its post
    // pending PendSV, PendSV's two words and an activate() frame that
    // finds nothing to serve (8 + 56 = 64; the fake frame's 32 are
    // popped before activate() pushes). 56 + 36 + 64 = 156: the
    // high-water mark lies within 156 bytes of the deeper of the two
    // samples.
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    bench.verdict("the paint reached the deepest sampled stack pointer", used >= deepest);
    bench.verdict("the high-water mark is within 156 bytes of the deepest sample",
                  used <= deepest + 156u);
}

// =============================================================================
// the Console's turn
// =============================================================================
bool clock_ok = false;
bool tick_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED", " tick=",
          tick_ok ? "SysTick" : "FAILED", " ruler=TIM2", crlf);
    print(serial, "test_stm32_rubato - the preemptive kernel on the STM32G0's Cortex-M0+, the "
          "scheduler in thread mode", crlf);
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
extern "C" void BRIO_STM32G0_USART2_HANDLER() {
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

/// TIM14's compare: the edge disarmed, the post, and the ruler as the
/// last statement - letter c's near end.
extern "C" void TIM14_IRQHandler() {
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

extern "C" uint32_t __bss_end;

int main() {
    // Paint the free stack: from 64 bytes below main's own stack pointer
    // down to the end of the image's RAM sections (.bss, then .noinit with
    // the panic record: stm32g0/ld/stm32g0b1re.ld names no stack limit,
    // the stack growing down from __stack_top, the top of the 144 KB, so
    // the higher of the two ends, 8-aligned, plus a guard of 64 bytes).
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
    tick_ok = brio::Ticker::init(clock);
    Clock32::init();
    (void)Clock32::configure(TimConfig{.prescaler = 0, .period = 0xFFFF'FFFFu});
    Clock32::enable(true);
    Edge::init();
    (void)Edge::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Nvic::enable(Edge::irq());
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", ta_ceiling);
    bench.letter('b', "road 2: the chain inside the posts", tb_road2);
    bench.letter('c', "the latency of road 1", tc_latency);
    bench.letter('d', "the rulers, the scan and the post, in cycles", td_scan);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", te_depth2);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", tf_timers);
    bench.letter('g', "the request under PRIMASK: served when the section ends", tg_primask);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('h', "the stack's high-water mark", th_stack);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
