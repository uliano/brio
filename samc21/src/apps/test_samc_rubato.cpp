// test_samc_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the SAM C21: SamRubatoPlatform and the core's
// scheduling request (cortexm/schedule.hpp) on the Cortex-M0+, against
// the model the host suite specifies (test/test_rubato).
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs INSIDE the kernel it tests: the program is a Rubato pack of
// five objects, from the highest priority down - Top, High, Mid, Low and
// Console - and the Console, the lowest, owns the UART and runs the
// letters in its own dispatch. Its print is the only writer of the port;
// the other objects write what they saw into a trace in memory, and the
// Console judges and prints it after. A post from a letter is therefore a
// post from the Console's dispatch: to an object above it, the kernel's
// road 2. The letters, their verdicts and their trace strings are the
// STM32G0's suite's (test_stm32_rubato) wherever the letter is the same,
// so the two outputs read side by side; letter d's rulers and letter w
// are this family's own.
//
// NOTHING TO WIRE, NOTHING LIT. The console is the board's CH340 on
// SERCOM5 (PB30/PB31, function D); the suite configures no other pad,
// and the LED on PB23 stays dark (letter i checks its pad). The
// interrupt every road-1 letter lands is TC4's compare match on channel
// 0: TC4 is the one TC that cannot pair (DS60001479M 35.6.2.4) and has a
// generic clock channel of its own (35.5.3, TC4_GCLK_ID), so taking it
// takes no half of a 32-bit pair and moves no sibling's clock; the
// console is a SERCOM and the ruler is SysTick, so neither uses a TC.
// TC4 counts generator 0 undivided, the 48 MHz of OSC48M the CPU runs
// on, so a delay placed in its counts is a delay in core cycles.
//
// THE RULER. This core has no DWT cycle counter (ARMv6-M), and the ruler
// is the kernel ticker's BasicTicker::cycles(): SysTick's position in its
// period composed with the tick count and the pending flag
// (cortexm/ticker.hpp, util/cycle_count.hpp) - CPU cycles, a cycle of
// resolution, a wrap every 89 s at 48 MHz. Its own cost is five reads, a
// consistency check and the compose: two back-to-back reads were 68
// cycles apart on the STM32G0B1RE at two flash wait states, a figure
// this suite re-measures (letter d prints it) and takes off every span
// it reports, so the spans are the path between the two stamps, as the
// STM32G0's one-load ruler gives them. `Ruler::now()` is always_inline
// and flatten, so the whole read lands inline at every stamp.
//
// WHERE THE SCHEDULER RUNS: in thread mode, PendSV returning into it
// through a second exception frame and the scheduler leaving through SVC
// (cortexm/schedule.hpp), so PendSV is not active while the dispatches
// run, and an interrupt's post preempts the dispatch it lands in at any
// depth - the Console's own dispatch included, since the UART's interrupt
// starts it at the loop's base. The SVC vector is entry 11 of this crt's
// table, SVCall_Handler (samc21/platform.hpp says why the name matters).
//
// THE BOUNDS. Every cycle bound is the release listing's instruction
// count at zero wait states by the Cortex-M0+'s table, plus what this
// flash adds, plus a stated margin. The table is the RP2040 datasheet's
// 2.4.3.3, table 81, the copy of the core's figures this desk has: a
// load or store 2, BL 3, BX 2, a taken branch 2, PUSH and POP 1 + N, a
// POP into the PC 3 + N with the PC among the N, MRS and MSR 3, DSB and
// ISB 3, MULS 1 (this part's multiplier is the fast one, DS60001479M
// table 10-1). An exception entry is 15 cycles, which 2.4.3.6.1 of the
// same datasheet quotes; a return of 10 and a tail-chain of 11 are the
// figures the STM32G0's suite takes, not read in the core's TRM (DDI
// 0484C is not on this desk). At 48 MHz the flash answers behind two
// wait states (DS60001479M table 45-41)
// through NVMCTRL's cache, direct-mapped, eight lines of 64 bits
// (27.6.7), in its reset read mode NO_MISS_PENALTY (27.8.2): 64 bytes,
// less than any path measured here, so a path's lines come again from
// the flash at two wait states - four Thumb instructions for 2 cycles
// more - and its literals likewise (docs/samc21/platform.md, "A handler
// in SRAM"). The fetch is counted with EVERY LINE PAID AT THE WORST
// ALIGNMENT: each straight run of B bytes of code the path executes is
// charged the most lines it can span from any halfword offset,
// ceil((B + 6) / 8), at 2 cycles a line, and every literal and vector
// it reads 2 more - as if the cache never hit and the linker had placed
// every run at its worst. The same count over the loop benchmark.md's
// SAM C21 row measures the ruler with gives 79 cycles a read where the
// row reads 65: the cache does hit, and the runs do not all sit at their
// worst. A span with the ruler taken off is the path between the two
// stamps' reads of SysTick's VAL less the ruler's own two reads at their
// best (26 instructions in one run of 52 bytes, 36 cycles at zero wait
// states and seven lines, 14: 50 - the loop that measures them is longer
// than the cache, so there every line is paid); the reads measure 48
// (letter d prints them). THE COUNT IS NOT AN UPPER BOUND on this flash:
// road 1, the scan and letter g measure 4 to 5 cycles over their counts,
// while the same instructions at 16 MHz, the flash behind no wait state,
// measured 1 to 4 under their zero-wait counts in a diagnostic build (not
// kept) - the shortfall is the fetch's, a literal that misses costing more
// than the two wait states the count charges it, the mechanism inside the
// flash controller not established. Every bound adds its fifth on top of the count;
// letter c's threshold for a slow event, which has no margin of its own,
// stands on the measured quiet path instead (its comment). The
// instruction counts are THIS image's listing
// (build-cmake/samc21j-release/test_samc_rubato.lst).
//
// The letters:
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms TC4 2000 cycles ahead and spins 20000 at a
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
//      handler's first, the ruler's own cost taken off, min, median and
//      max over 100 events after one warm-up, the console drained first;
//      the bound is the quiet path measured plus what a kernel tick adds,
//      counted by the M0+'s own cycles and this flash's fetch, written
//      beside it.
//   d  THE RULER, THE SCAN AND THE POST: the ruler's two-read cost and
//      its count across four tick reloads; activate() with nothing ready
//      below the Console's ceiling (four empty queues, the fifth object at
//      the ceiling), and from Mid's handler one post that asks for nothing
//      (to Low, below it) against one that runs High's dispatch inside it
//      (road 2), each the least of 16, the ruler's own cost taken off.
//   e  DEPTH 2: High's handler - started by road 2 (the Console's post),
//      then by road 1 (an interrupt over the Console) - spins at a mark
//      while TC4 posts to Top. Top's entry lands before High resumes;
//      the latency printed. Ten runs each.
//   f  TIME EVENTS UNDER PREEMPTION, in three parts. (1) Three one-shots
//      matured in one turn, the lowest object's armed last so walked
//      first, posted the way Rubato::run() posts them - the walk with the
//      ceiling held at 0, then the scheduler: nothing is dispatched during
//      the walk and the three are served highest first. (2) Low's handler
//      runs 200 turns of arm(), arm(), disarm(), most of each turn inside
//      a masked window; TC4's interrupt lands there and its High
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
//      statement to the receiver's first.
//   w  THE REQUEST PENDING AT THE IDLE'S WFI: with interrupts masked as
//      the kernel's idle path masks them, a post to Mid pends PendSV and
//      the platform's own idle() follows - its DSB, its WFI under the mode
//      PM.SLEEPCFG holds, its unmask. ARMv6-M's WFI ends on a pending
//      exception that would preempt with PRIMASK clear, and this letter
//      asks the SAM's PM whether a pending PendSV ends it as a pending
//      NVIC line does (docs/samc21/platform.md measures the line): in
//      IDLE0 and in IDLE2, ten runs each, each started just after a tick
//      edge so that the tick, the only other wake source armed, is a whole
//      millisecond away. Mid's first statement before that tick says the
//      WFI ended on the request; its cycles from the stamp before idle()
//      are printed and judged in IDLE0. The PM is handed back at IDLE0.
//      Not walked: STANDBY, where the tick's interrupt is held off across
//      the WFI (erratum 1.8.13's guard) and a WFI the request did not end
//      would have no wake at all.
//   i  DARK: every object but the Console idle, no time event armed,
//      TC4's line quiet, the LED's pad not an output, the ceiling the
//      Console's own, interrupts enabled.
//   h  THE STACK, the last letter: main() painted the free stack below
//      its own frame down to the end of the image's RAM sections; the
//      high-water mark against the deepest stack pointer the dispatches
//      and the interrupt sampled in letters a..w plus what lies below a
//      sample (a basic exception frame, the handlers' own frames), the
//      arithmetic beside the verdict.
//
// RUNNING IT: `brio flash <board> test_samc_rubato`, then
// `brio run <board> z` (one image: the SAMC21J18A's 256 KB and 32 KB
// take the whole suite), each letter also by name with
// `brio run <board> <letter> --expect="->"`. The board is a c21j in the
// manifest, the Atmel-ICE's cable in its SAM port. What a run settles
// that this source cannot: the ruler's own cost on this flash (letter
// d), and whether a pending PendSV ends the WFI (letter w).
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "samc21/sleep.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = SamRubatoPlatform;
using Edge = Tc<4>;

/// The kernel ticker read as CPU cycles (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
};

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
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

/// TC4's compare `d` counts of the 48 MHz generator from about now, its
/// interrupt armed: one edge, the handler disarms it. COUNT and CC0 are
/// write-synchronized (35.6.2.1), so each write waits its SYNCBUSY bit
/// out; the counter has counted those waits when the interrupt is armed,
/// and the edge lands `d` counts after the COUNT write, a few tens of
/// cycles before `d` have passed since the call returned - the letters
/// place their edges hundreds of cycles out.
[[gnu::always_inline]] inline void edge_in(uint16_t d) {
    Edge::disarm(Edge::match_flag(0));
    (void)Edge::set_count16(0);
    (void)Edge::set_cc16(0, d);
    Edge::clear_flags(Edge::match_flag(0));
    Edge::arm(Edge::match_flag(0));
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

/// Let the console's transmitter fall silent (at most 50 ms, 2.4 M
/// cycles at 48 MHz; 230 bytes at 115200 are 20 ms): its interrupt,
/// landing in a measured path, would be measured.
void console_drain() {
    const uint32_t t0 = Ruler::now();
    while (!Serial::tx_idle() && Ruler::now() - t0 < 2'400'000u) {
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

/// A span between two stamps with the ruler's own two-read cost taken
/// off, never below zero.
[[gnu::always_inline]] inline uint32_t net(uint32_t span, uint32_t ruler) {
    return span > ruler ? span - ruler : 0u;
}

/// One millisecond of CPU cycles: the letters' wait for a dispatch that
/// should have run within a few hundred cycles.
constexpr uint32_t one_ms = SysClock::hz / 1000u;

/// The ruler's two-read cost, measured at the start of every letter that
/// takes it off a span.
uint32_t ruler = 0;

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
    // Mid's and Low's), and the one post to Top is TC4's handler with
    // edge_to at 0, which only letter e's High sets, arming one edge that
    // lands inside its own spin; that handler's first statement disarms
    // the match interrupt, so after letter e no edge is armed, and letter
    // i checks the line quiet before the prompt. So a 0 here is the walk,
    // never a dispatch.
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
            edge_in(2000u);      // 42 us into a 417 us spin at 48 MHz: the edge lands inside it
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

/// The road-1 path from the interrupt handler's stamp to High's, by the
/// release listing (the file header says how a bound is drawn): the
/// stamp's tail and the handler's exit (25 instructions, a pop of five
/// into the PC); the tail-chain into PendSV (11) and its vector's read;
/// PendSV's thirteen instructions (no barrier on this core,
/// cortexm/schedule.hpp) and the return into thread mode (10);
/// the entry's cpsie and call; activate() at the Console's ceiling - its
/// frame and the ceiling's test, Top's visit (the queue's two cursors
/// loaded and compared in line, no call), High's, the raise of the
/// ceiling and of the dispatching index, take() on the path whose cursor
/// wraps (the longer of its two, by a cycle) and its result, to the
/// stamp's read of VAL. 100 instructions, 183 cycles at zero wait
/// states and 94 of fetch (ten literals and the vector among them): 277,
/// less the ruler's 50, 227 (147 at zero wait states) - the path's own
/// count, which the quiet events measure 231 to 232: SHORT by 4 to 5 (the
/// file header says why). So the threshold for a slow event is the
/// fastest quiet event measured, 231, plus two cycles for each of the
/// path's nine taken transfers (the most a branch target's line can move
/// with where the code lands): 249. On top of the slowest quiet event
/// measured, 232, at most one SysTick landing in the path: its entry (15)
/// and its vector, its handler on the longer branch, a second's boundary,
/// and its return (19 instructions, 57 cycles, 26 of fetch: 83), and
/// either the stamp's retry when it lands between the five reads (13
/// instructions, 30) or an exception return abandoned and begun again
/// (12): 113. The bound is the two, 345, and a fifth of the quiet path
/// (47): 392. The letter reads the kernel tick around each event, and
/// every event over 249 must have had a tick in it.
constexpr uint32_t latency_quiet = 249u;
constexpr uint32_t latency_bound = 392u;

void tc_latency() {
    reset_scenario();
    tracing = false;
    // One warm-up event (the cache, the bridge) not counted, then 100;
    // each waits at most 1 ms for High's first statement.
    console_drain();
    ruler = ruler_cost();
    uint32_t missed = 0;
    // An event slower than the quiet path's threshold is one a tick
    // landed in: the kernel tick read around each says so.
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
            const uint32_t d = net(High::entered - edge_last, ruler);
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
    print(serial, "  road 1 over the Console's wait: the interrupt's last statement to High's "
          "first, min ", lo, " median ", med, " max ", hi, " cycles over ", latency_events,
          " events (", missed, " missed); the three slowest ", latency[latency_events - 3u],
          " ", latency[latency_events - 2u], " ", hi, "; the ruler's ", ruler,
          " taken off each", crlf);
    print(serial, "  events over ", latency_quiet, " cycles: ", slow, ", a kernel tick inside ",
          slow_ticked, " of them", crlf);
    bench.verdict("every interrupt's post started High within 1 ms", missed == 0u);
    bench.verdict("every event over the quiet path's threshold had a tick land in it",
                  slow == slow_ticked);
    bench.verdict("the latency stays under the path's bound (the arithmetic beside it)",
                  hi <= latency_bound);
    reset_scenario();
}

// =============================================================================
// d - the ruler, the scan and the post
// =============================================================================
volatile uint32_t post_quiet = 0;
volatile uint32_t post_road2 = 0;

/// The ruler's two reads, VAL to VAL in ruler_cost()'s loop, by the
/// release listing: 26 instructions, 36 cycles at zero wait states and
/// 16 of fetch at the worst alignment, 52, and a fifth (11): 63. The
/// STM32G0B1RE's same two reads measured 68 at its two wait states
/// behind a prefetch, its listing another.
constexpr uint32_t ruler_bound = 63u;
/// The ruler across four reloads, against 4 x 48 000. Each end is a tick
/// handler's exit, a poll that sees the count move, and the reads to
/// VAL; what differs between the two ends bounds the difference: the
/// two ends' code from the poll's exit to VAL (10 instructions, 16 to 26
/// cycles without and with their fetch, against 12, 19 to 31: at most
/// 15), the handler's two branches (62 and 83 cycles: 21), a poll turn
/// (9 and 10 cycles: 10) and the instruction the tick's entry waits for
/// (a load and a line, 3): 49, and a fifth (10): 59.
constexpr uint32_t span_bound = 59u;

void td_scan() {
    reset_scenario();
    tracing = false;
    console_drain();
    ruler = ruler_cost();
    // The ruler across four tick reloads: each span starts at the first
    // reading after one tick and ends at the first after the fourth
    // tick on, so it must read four periods of SysTick, 4 x 48 000.
    uint32_t span_hi = 0;
    for (int i = 0; i < 4; ++i) {
        uint32_t t = Ticker::ticks();
        while (Ticker::ticks() == t) {
        }
        const uint32_t c0 = Ruler::now();
        t = t + 1u;
        while (Ticker::ticks() - t < 4u) {
        }
        const uint32_t c1 = Ruler::now();
        const uint32_t dc = c1 - c0;
        constexpr uint32_t want = 4u * one_ms;
        const uint32_t diff = dc > want ? dc - want : want - dc;
        if (diff > span_hi) {
            span_hi = diff;
        }
    }
    uint32_t scan = 0xFFFF'FFFFu;
    uint32_t scan_hi = 0;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        const bool served = K::activate();
        const uint32_t t1 = Ruler::now();
        const uint32_t c = net(t1 - t0, ruler);
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
        const uint32_t t0 = Ruler::now();
        post<Low>(Hit{0});        // below Mid: pushed, nothing asked
        const uint32_t t1 = Ruler::now();
        post<High>(Hit{0});       // above: High's dispatch inside the post
        const uint32_t t2 = Ruler::now();
        post_quiet = net(t1 - t0, ruler);
        post_road2 = net(t2 - t1, ruler);
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
    print(serial, "  the ruler's two reads (the ticker's composed cycles()) ", ruler,
          " cycles", crlf);
    print(serial, "  across four tick reloads the ruler reads 4 x ", one_ms, " cycles within ",
          span_hi, crlf);
    print(serial, "  activate() with nothing ready below the Console: ", scan, "..", scan_hi,
          " cycles (four empty queues, the fifth at the ceiling)", crlf);
    print(serial, "  from Mid: a post to Low (asks nothing) ", quiet,
          " cycles; a post to High (road 2, High's dispatch inside) ", road2,
          " cycles; the scheduling's share ", road2 > quiet ? road2 - quiet : 0u, crlf);
    bench.verdict("the ruler's two reads cost under its bound (the arithmetic beside it)",
                  ruler < ruler_bound);
    bench.verdict("the ruler counts four tick periods across four reloads, within its bound",
                  span_hi <= span_bound);
    // The bounds below are the release listing's count with every line
    // paid at the worst alignment, the ruler's 50 taken off, and a fifth
    // as the margin (the file header says how they are drawn).
    // The scan: the stamp's tail and the call; activate()'s frame and the
    // ceiling's test; per empty object the ceiling's compare and the
    // queue's two cursors loaded and compared in line (no call); the
    // Console's index ending the pass and the return; the second stamp
    // to VAL (activate() alone is 46 instructions, 88 cycles at zero wait
    // states). 78 instructions, 135 cycles and 78 of fetch: 213, less 50,
    // 163 (99 at zero wait states); 196 with the fifth.
    bench.verdict("the scan of four empty queues, the least of 16, costs under 196 cycles",
                  scan < 196u);
    // The quiet post: the stamp's tail, the event's setup and the call;
    // post<Low>()'s thirteen instructions on the path that asks nothing,
    // the queue's push() under its guard (twenty-two: mrs, cpsid, the
    // cursors, the slot, msr); the second stamp to VAL. No queue is asked
    // whether it is empty on this path. 66 instructions, 108 cycles and
    // 60 of fetch: 168, less 50, 118 (72 at zero wait states); 142 with
    // the fifth.
    bench.verdict("a post that asks nothing costs under 142 cycles (the push and one compare)",
                  quiet < 142u);
    // The road-2 post: the quiet one's shape to the compare, then the
    // request's call and body; PendSV's entry from thread mode (15) and
    // its vector's read, its body and the return into thread mode (10);
    // the entry; activate() at Mid's ceiling - Top's visit and High's,
    // each queue's cursors compared in line, the raises, take() and the
    // release each on the path whose cursor wraps, High's dispatch with
    // nothing to do (its stamp, the stack sample, the trace's and the
    // action's tests), the two restores, the rescan of Top and High and
    // Mid's index ending the pass, the return; the SVC's entry (15) and
    // its vector's read, three instructions and the return into the real
    // frame (10); the request's and the post's returns; the second stamp
    // to VAL. 249 instructions, 464 cycles and 260 of fetch (24 literals
    // and the two vectors among them): 724, less 50, 674 (428 at zero
    // wait states); 809 with the fifth.
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    bench.verdict("and under 809 cycles, High's empty dispatch included", road2 < 809u);
}

// =============================================================================
// e - depth 2
// =============================================================================
void te_depth2() {
    const char* const want = "Hs1 Hm1 Ts0 Te0 Hr1 He1";
    ruler = ruler_cost();
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
            const uint32_t d = net(Top::entered - edge_last, ruler);
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
    //    arm()'s or disarm()'s masked window; TC4's interrupt lands there
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
    console_drain();
    ruler = ruler_cost();
    uint32_t least = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        const uint32_t t0 = Ruler::now();
        te_m.arm(1000);
        const uint32_t t1 = Ruler::now();
        if (net(t1 - t0, ruler) < least) {
            least = net(t1 - t0, ruler);
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
    // listing: the stamp's tail and the call; arm()'s frame, the guard's
    // mrs and cpsid; the unlink, out of line - the head's load, the walk's
    // test, the two stores of the unlink and the node's two, the return;
    // now()'s volatile load, the deadline and the node's stores, the link
    // at the list's tail, the msr and the return; the second stamp to
    // VAL. 75 instructions, 128 cycles and 64 of fetch: 192, less 50, 142
    // (92 at zero wait states); 171 with the fifth (letter d's comment
    // says how the bounds are drawn).
    bench.verdict("one arm(), the least of 16, costs under 171 cycles, its masked window inside",
                  arm_cycles < 171u);
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

void tg_primask() {
    console_drain();
    ruler = ruler_cost();
    pend_inside = true;
    ran_inside = false;
    // From the Console's dispatch (ceiling 4) to Mid, then from Low's (3)
    // to High: the receiver's entry follows the section's last mark and
    // precedes the mark after the section.
    const char* const want_console = "Cm4 Cp4 Ms2 Me2 Ca4";
    const char* const want_low = "Ls3 Lm3 Lp3 Hs1 He1 La3 Le3";
    uint8_t alike_console = 0;
    uint8_t alike_low = 0;
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    bool back = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        post_in_section<Mid>('C');
        {
            const uint32_t d = net(Mid::entered - section_last, ruler);
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
    bench.verdict("PendSV stood pending inside every section, and no receiver ran inside one",
                  pend_inside && !ran_inside);
    bench.verdict("the Console's post to Mid under PRIMASK is served when the section ends, "
                  "before the statement after it (10 runs)",
                  alike_console == 10u);
    bench.verdict("Low's post to High under PRIMASK likewise, inside Low's dispatch (10 runs)",
                  alike_low == 10u);
    // From the stamp's read to Mid's, by the release listing: the stamp's
    // tail and store, the guard's msr that lifts the mask; PendSV's entry
    // from thread mode (15) and its vector's read, its body and the
    // return into thread mode (10); the entry; activate() at the
    // Console's ceiling - Top's visit and High's, each queue's cursors
    // compared in line, Mid's, the raises, take() on the path whose
    // cursor wraps and its result, to the stamp's read. 108 instructions,
    // 197 cycles and 106 of fetch: 303, less 50, 253 (161 at zero wait
    // states); 304 with the fifth (letter d's comment says how the bounds
    // are drawn).
    bench.verdict("the section's end to the receiver's first statement under 304 cycles",
                  most < 304u);
    bench.verdict("the ceiling is the Console's again, interrupts enabled", back);
}

// =============================================================================
// w - the request pending at the idle's WFI
// =============================================================================

/// One leg of letter w in the mode `mode`: ten runs, each started just
/// after a tick edge, of the kernel's idle shape with a request pending.
/// Returns how many ran Mid before the next tick; `least` and `most` are
/// the cycles from the stamp before idle() to Mid's first statement.
uint8_t wfi_leg(SleepMode mode, bool& pending_all, uint32_t& least, uint32_t& most) {
    least = 0xFFFF'FFFFu;
    most = 0;
    uint8_t woke = 0;
    if (!Pm::set_sleep_mode(mode)) {
        return 0;
    }
    for (int run = 0; run < 10; ++run) {
        const uint32_t t = Ticker::ticks();
        while (Ticker::ticks() == t) {
        }
        const uint32_t r = Mid::runs;
        disable_interrupts();              // the mask the kernel's idle path holds
        post<Mid>(Hit{0});                 // below the Console's ceiling: PendSV pended
        pending_all = pending_all && pendsv_pending();
        const uint32_t t0 = Ruler::now();
        P::idle();                         // DSB, WFI at the armed mode, cpsie
        const bool same_tick = Ticker::ticks() == t + 1u;
        if (Mid::runs != r && same_tick) {
            ++woke;
            const uint32_t d = net(Mid::entered - t0, ruler);
            least = d < least ? d : least;
            most = d > most ? d : most;
        }
    }
    return woke;
}

/// From the stamp before idle() to Mid's first statement, the WFI
/// falling through on the pending request, by the release listing: the
/// stamp's tail; idle()'s call, the SLEEPCFG read and its test, the DSB,
/// the WFI, the branch to the cpsie; PendSV's entry from thread mode (15)
/// and its vector's read, its body and the return into thread mode
/// (10); the entry; activate() at the Console's ceiling - Top's visit
/// and High's, each queue's cursors compared in line, Mid's, the raises,
/// take() on the path whose cursor wraps and its result, to the stamp's
/// read. 106 instructions, 199 cycles and 112 of fetch: 311, less the
/// ruler's 50, 261 (163 at zero wait states). The count puts idle()'s
/// own path at 38 cycles, where test_samc_platform measured it at 30 to
/// 45 with a pending NVIC line (docs/samc21/platform.md, "The idle path,
/// measured": the SLEEPCFG read crosses a bridge): 7 more, 268; 322 with
/// a fifth (letter d's comment says how the bounds are drawn).
constexpr uint32_t wfi_bound = 322u;

void tw_wfi() {
    reset_scenario();
    tracing = false;
    console_drain();
    ruler = ruler_cost();
    bool pending0 = true;
    bool pending2 = true;
    uint32_t least0 = 0;
    uint32_t most0 = 0;
    uint32_t least2 = 0;
    uint32_t most2 = 0;
    const uint8_t woke0 = wfi_leg(SleepMode::idle0, pending0, least0, most0);
    const uint8_t woke2 = wfi_leg(SleepMode::idle2, pending2, least2, most2);
    const bool handed_back = Pm::set_sleep_mode(SleepMode::idle0);
    reset_scenario();
    print(serial, "  IDLE0: ", woke0, " of 10 runs served Mid before the next tick, ", least0,
          "..", most0, " cycles from the stamp before idle() to Mid's first statement", crlf);
    print(serial, "  IDLE2: ", woke2, " of 10 runs served Mid before the next tick, ", least2,
          "..", most2, " cycles", crlf);
    bench.verdict("PendSV stood pending at every WFI", pending0 && pending2);
    bench.verdict("a pending PendSV ends the WFI in IDLE0: Mid served at the unmask, before "
                  "the next tick (10 runs)",
                  woke0 == 10u);
    bench.verdict("and within the path's bound (the arithmetic beside it)",
                  woke0 == 10u && most0 <= wfi_bound);
    bench.verdict("a pending PendSV ends the WFI in IDLE2 too, before the next tick (10 runs)",
                  woke2 == 10u);
    bench.verdict("the PM handed back at IDLE0", handed_back && Pm::sleep_mode() == SleepMode::idle0);
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
    bench.verdict("TC4's line is quiet (nothing pending)", !Nvic::pending(Edge::irq()));
    bench.verdict("the LED's pad PB23 is not an output", !Pin<'B', 23>::is_output());
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
    // registers and 44-byte frame (64 bytes) are above it. Below the
    // deepest one lies at most, by the release listing: the letter's
    // action and what it calls (the deepest, letter g's in Low, saves
    // five registers and 20 bytes, 40, and calls note(), 20: 60; letter
    // d's, 40, calls a post, 8, and the queue's push, 8: 56); an
    // interrupt landing there, a basic frame of 8 words and its
    // alignment word (36: this core has no FPU words); and below that
    // either the deepest handler's own frames (TC4's 40 with a post's 8
    // and a push's 8: 56; the UART's 40 with its out-of-line receive
    // paths) or, the handler gone and its post pending PendSV, PendSV's
    // two words and an activate() frame that finds nothing to serve
    // (8 + 64 = 72; the fake frame's 32 are popped before activate()
    // pushes). 60 + 36 + 72 = 168: the high-water mark lies within 168
    // bytes of the deeper of the two samples.
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    bench.verdict("the paint reached the deepest sampled stack pointer", used >= deepest);
    bench.verdict("the high-water mark is within 168 bytes of the deepest sample",
                  used <= deepest + 168u);
}

// =============================================================================
// the Console's turn
// =============================================================================
bool clock_ok = false;
bool tick_ok = false;
bool edge_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED", " tick=",
          tick_ok ? "SysTick" : "FAILED", " edge=", edge_ok ? "TC4" : "FAILED",
          " ruler=SysTick cycles", crlf);
    print(serial, "test_samc_rubato - the preemptive kernel on the SAM C21's Cortex-M0+, the "
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
//
// An unbound vector here is a SILENT death - the crt's default handler is
// a spin loop - so every line this suite raises is bound, and the SVC
// vector by the crt's own name, SVCall_Handler.
extern "C" void SERCOM5_Handler() {
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVCall_Handler() { brio::scheduler_return(); }

/// TC4's compare: the edge disarmed, the post, and the ruler as the last
/// statement - letter c's near end.
extern "C" void TC4_Handler() {
    Edge::disarm(Edge::match_flag(0));
    Edge::clear_flags(Edge::match_flag(0));
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
    // the panic record: samc21/ld/samc21j18a.ld names no stack limit, the
    // stack growing down from __stack_top, the top of the 32 KB, so the
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
    tick_ok = brio::Ticker::init(clock);
    edge_ok = Edge::init(0) &&
              Edge::configure(TcConfig{.mode = TcMode::count16, .prescaler = TcPrescaler::div1}) &&
              Edge::enable(true);
    brio::Nvic::enable(Edge::irq());
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", ta_ceiling);
    bench.letter('b', "road 2: the chain inside the posts", tb_road2);
    bench.letter('c', "the latency of road 1", tc_latency);
    bench.letter('d', "the ruler, the scan and the post, in cycles", td_scan);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", te_depth2);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", tf_timers);
    bench.letter('g', "the request under PRIMASK: served when the section ends", tg_primask);
    bench.letter('w', "the request pending at the idle's WFI, in IDLE0 and IDLE2", tw_wfi);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('h', "the stack's high-water mark", th_stack);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
