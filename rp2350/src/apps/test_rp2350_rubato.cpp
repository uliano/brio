// test_rp2350_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the RP2350: Rp2350RubatoPlatform<core> and this
// target's scheduling request (rp2350/schedule.hpp) on BOTH processor
// pairs from one source - the core stratum's PendSV and SVC on the
// Cortex-M33, the machine software interrupt through the crt's trampoline
// and the entry's own mret on Hazard3 - and the one scheduler entry that
// serves a kernel on each core (the pair body), against the model the
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
// road 2. Core 1 is held in the bootrom for every letter but one: letter
// l launches it into a Rubato pack of its own and puts it back when it
// ends.
//
// ONE SOURCE, TWO HALVES, apart only where rp2350/core.hpp tells them
// apart: the scheduler's vectors (one macro,
// BRIO_RP2350_SCHEDULER_VECTORS_PAIR, two vectors on one half and one on
// the other), the stack pointer's read, the request's pending bit, and
// the one letter each half has alone - j, the FPU across a preemption, on
// the Cortex-M33 (Hazard3 here has no FPU); k, the software interrupt's
// own facts, on Hazard3.
//
// THE ONE SCHEDULER ENTRY IS THE PAIR BODY in this image, for every
// letter: letter l runs a Rubato on each core, the two cores of a pair
// share one table, and a vector is bound once (rp2350/schedule.hpp). On
// core 0 the pair body takes the single body's path with the core asked
// in place of the entry's literal - four cycles more on the Cortex-M33 by
// the model below, two on Hazard3 - and every bound below is drawn for
// it.
//
// NOTHING TO WIRE, NOTHING LIT. The console is the Debug Probe's UART
// bridge on GP0 (TX) / GP1 (RX) = UART0; the suite configures no other
// pad, and the board's LED on GP25 is never driven (letter i checks its
// pad). The interrupt every road-1 letter of core 0 lands is TIMER1's
// alarm 0, served by core 0 under the one name `isr_timer1_0` on both
// halves; core 1's, in letter l, is TIMER1's alarm 1, enabled in core
// 1's controller alone, and the doorbell's line, `isr_sio_bell`.
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
// uncounted first, and the XIP cache, 4.4.1, then holds it), are the
// figures test_rp2350_platform's letter i measures on this chip: each
// primitive timed there as a run of identical instructions in the flash,
// the cache warm, on this suite's ruler.
//  - The Cortex-M33 (ARM's TRM for the core, 100230, gives no instruction
//    timing): an instruction 1; a load 1, and 2 when the next instruction
//    reads what it loaded as an operand or an address (not as a store's
//    data), a literal's included; a load over the APB bridge, the ruler's,
//    3; a store 1; a taken branch, a BL or a BX 2, one more onto a 32-bit
//    instruction off its word; a not-taken conditional branch 1, and
//    nothing after a 16-bit instruction, which it issues with (the core's
//    dual issue, TRM 1.4.1), an IT nothing likewise; PUSH and POP of N
//    registers N, a POP into the PC N + 3; MRS, MSR, CPSID and CPSIE 1;
//    DSB and ISB 2. The exceptions, each as the letter times it: an SVC to
//    its handler's first instruction 13, the SVC's 1 and the entry's 12;
//    after the instruction that loads EXC_RETURN into the PC, the return
//    to thread mode 8 (10 with a BX) and 7 after a POP of N (N + 10 in
//    all), and into a pending exception, the tail-chain, 5 and 4 (7 with a
//    BX, N + 7 with the POP); PendSV pended under the mask and taken at the
//    MSR that lifts it 12 - the MSR, the one instruction that retires
//    behind it and the entry's 10 -; requested by the store to ICSR, its
//    DSB and its ISB 14 - the three, 5, and the entry's 9 -, and the return
//    onto the instruction after that ISB 3 longer than the return's 8.
//  - Hazard3: datasheet 3.8.7, which the letter confirms - an ALU
//    instruction or a CSR access 1, a load 1 and 2 when the next
//    instruction reads it (not when it only stores it), a store 1, `jal`
//    and `jalr` 2, a branch 1 not taken and 2 taken, every transfer one
//    more onto a 32-bit instruction not 32-bit aligned, cm.push and cm.pop
//    of N registers N + 1, cm.popret N + 2 (4 for one), each counted so
//    here - but for `mret`, 3 where the table gives 2, and the trap
//    entries the section does not give, measured to the handler's first
//    instruction through the table's jump: an `ecall` 5, an `mret` into a
//    pending software interrupt 6, the `csrsi` that finds it pending 5, the
//    store that raises it with MIE set 7 (the store, the two instructions
//    that retire behind it, the entry's 2 and the jump's 2) - the entry 1
//    after an `mret`, 2 after a `csrsi` or a store, where `ecall`'s own is
//    3;
//  - on both, a read of TIMER1 is 3 cycles (2.2.4: three at least) and
//    the SIO answers in one (3.1), a load or a store.
// A bound is the path's count under its model, plus what may land in it
// (a kernel tick), plus a fifth of the count, rounded up, as the margin.
// Against the measurement the counts sit within two cycles on the paths
// that start and end in thread mode (the scan, the two posts, `arm()`):
// exact on Hazard3, and on the Cortex-M33 the scan 2 under and the two
// posts 2 over; and 3 to 5 cycles under it on those that start in an
// interrupt handler or under the mask and end at a dispatch's first
// statement (letters c, e, g and l): a residue the letter's rows, each
// matching its model, do not place.
//
// The letters (each bound's arithmetic beside its verdict in the code):
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms the alarm 2000 cycles ahead and spins 20000 at
//      a mark; the interrupt posts to High. The trace says High's dispatch
//      landed before Low resumes, and every entry reads the ceiling at its
//      own object's index; the Console reads its own, 4, after the chain,
//      and the UART's receive interrupt read the loop's ceiling when the
//      letter's key arrived - 255 at the base, or 0 through the base turn
//      (its walk of the time events and its choice). Ten runs, all alike.
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
//      same sled behind the platform's request, where the count must be 0,
//      the request having waited. (2) The registers across an entry and a
//      nested one: fifteen caller-saved registers loaded with patterns, the
//      alarm landing while they are live and starting High by road 1, High
//      doing the same while the alarm starts Top over it: every pattern
//      found again on both levels. (3) Inside a dispatch the software
//      interrupt started: the flag already lowered, mip.MSIP clear,
//      mstatus.MIE set, mscratch moved by one.
//   l  A RUBATO ON EACH CORE: core 1 launched into a pack of three of its
//      own (Top1, High1, Low1) on Rp2350RubatoPlatform<1>, its request set
//      up by its own scheduler_init(), every preemption on both cores
//      through the one pair body. Core 0's Low sends to core 1's Low1
//      (road 1 from core 1's doorbell handler, at its base); then both
//      cores run the same shape at once: a spin at a mark, the core's own
//      alarm posting to its High, which posts to its Top inside its
//      handler (road 2). Each core writes its own trace with its own
//      ceiling; read after both settle, each shows its own indices, and
//      the ruler shows each core's preemption landing inside the other
//      core's preempted dispatch. Ten runs after one warm-up. Inside
//      Low1's dispatch a post to core 0's High, which the queue refuses
//      and counts, with core 1's request read not raised. Before those
//      runs, with core 0 asleep, core 1's two road-1 latencies over
//      sixteen crossings - the doorbell handler's last statement to
//      Low1's first, alarm 1's to High1's - against their counts; on the
//      Cortex-M33 core 1's FPCCR. Then core 1's stack: painted by its
//      entry, its high-water mark against the deepest stack pointer its
//      dispatches and its interrupts sampled, and core 1 back in the
//      bootrom.
//   i  DARK: every object of core 0 idle, core 1 in the bootrom, no time
//      event armed, both alarms disarmed and alarm 0's line quiet, the
//      LED's pad not an output, the ceiling the Console's own, interrupts
//      enabled; and the request as
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
#include "rp2350/multicore.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/inbox.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Rp2350RubatoPlatform<>;
using P1 = Rp2350RubatoPlatform<1>;   // core 1 under Rubato (letter l)
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

/// `len` marks equal `want`, written as "Ls3 Lm3 Hs1 ..." (who, what,
/// ceiling) with single spaces.
bool marks_are(const Mark* marks, uint8_t len, const char* want) {
    uint8_t i = 0;
    const char* p = want;
    while (*p != '\0') {
        if (i >= len) {
            return false;
        }
        const Mark& m = marks[i];
        if (p[0] != m.who || p[1] != m.what || static_cast<uint8_t>(p[2] - '0') != m.ceiling) {
            return false;
        }
        p += 3;
        if (*p == ' ') {
            ++p;
        }
        ++i;
    }
    return i == len;
}

/// Core 0's trace equals `want`.
bool trace_is(const char* want) { return marks_are(trace, trace_len, want); }

/// Core 1's trace (letter l): written by core 1 alone, with core 1's
/// ceiling and under core 1's mask, and read by core 0 once core 1 has
/// settled at its base.
Mark trace1[trace_capacity];
volatile uint8_t trace1_len = 0;

void note1(char who, char what) {
    InterruptGuard g;
    const uint8_t n = trace1_len;
    if (n < trace_capacity) {
        trace1[n] = Mark{who, what, P1::ceiling};
        trace1_len = static_cast<uint8_t>(n + 1u);
    }
}

void print_trace1() {
    print(serial, "  core 1's trace:");
    for (uint8_t i = 0; i < trace1_len; ++i) {
        print(serial, " ", trace1[i].who, trace1[i].what, trace1[i].ceiling);
    }
    print(serial, crlf);
}

/// The lowest stack pointer a dispatch or the interrupt saw, per core.
volatile uint32_t sp_low_dispatch = 0xFFFF'FFFFu;
volatile uint32_t sp_low_isr = 0xFFFF'FFFFu;
volatile uint32_t sp1_low_dispatch = 0xFFFF'FFFFu;
volatile uint32_t sp1_low_isr = 0xFFFF'FFFFu;

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

/// The scheduler run by hand from the Console's dispatch, as the
/// platform's entry and exit run it (kernel/platform.hpp): the ceiling
/// replaced with 0, activate() at the Console's own, the ceiling put
/// back. Whether it served. One step of the platform's exit is left out:
/// the compare of the quiet-post count that activate() returns, which
/// starts the scheduler again when a post landed after its last scan.
/// No post can land there in this suite: the kernel tick posts nothing
/// (the time events are walked in the base turn, base_turn() here),
/// alarm 0 is armed only by a letter that waits for its post before
/// going on, the Console's own received byte is a post at the base's ceiling
/// that activate() would not serve anyway, and core 1's doorbell and
/// alarm 1 live in letter l alone, where core 0 calls this nowhere.
bool activate_here() {
    const uint8_t base = P::ceiling;
    P::ceiling = 0;
    const bool served = K::activate(base).served;
    P::ceiling = base;
    return served;
}
constexpr uint8_t console_index = 4;

// =============================================================================
// Core 1's pack (letter l)
// =============================================================================

uint32_t xip_misses();

/// Core 1's three, the shape of core 0's Node with core 1's trace and
/// stack samples; Low1 receives across the cores, through its inbox. The
/// XIP cache's miss counter and core 0's tick count are read right after
/// the stamp, so that a latency on core 1 can be told lengthened by a
/// refill or by core 0's tick handler (letter l). The
/// action is written by core 0 and read by core 1, so it is volatile: a
/// plain store core 0's compiler sees overwritten before anything of
/// core 0 reads it is a dead store, and gone.
template <uint8_t id>
struct Node1 {
    using Event = Hit;
    static constexpr char name = "thl"[id];
    static inline EventQueue<Event, 4, P1> queue;
    static constexpr uint8_t inbox_depth = 4;
    static inline void (*volatile action)(uint8_t) = nullptr;
    static inline volatile uint32_t entered = 0;
    static inline volatile uint32_t entered_misses = 0;
    static inline volatile uint32_t entered_ticks = 0;
    static inline volatile uint32_t runs = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        entered = Ruler::now();
        entered_misses = xip_misses();
        entered_ticks = Ticker::ticks();
        runs = runs + 1u;
        sample_sp(sp1_low_dispatch);
        note1(name, 's');
        if (action != nullptr) {
            action(e.n);
        }
        note1(name, 'e');
    }
};

using Top1 = Node1<0>;
using High1 = Node1<1>;
using Low1 = Node1<2>;
using K1 = Rubato<P1, Top1, High1, Low1>;

/// What core 1's doorbell handler drains: Low1 alone receives across.
using Drain1 = Inboxes<Low1>;

/// Which program core 1 runs: 0 none (the bootrom), 2 its Rubato.
/// Written by core 1's entry and by core 0's park, read by core 0.
volatile uint32_t core1_stage = 0;

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
    // The key arrives while the kernel's loop runs below every object, and
    // the loop holds one of two ceilings there: 255, its base, or 0 through
    // its base turn (kernel/rubato.hpp, run(): 0 from the walk of the
    // matured time events, where a fire only queues, through activate()'s
    // choice, until idle_if_empty() puts 255 back; init_all() holds 0 the
    // same way around the inits). 0 is also Top's index, so a 0 would be Top's
    // dispatch as well - but nothing can dispatch Top when this key
    // arrives: the suite arms no time event on Top (its timers are High's,
    // Mid's and Low's), and Top's senders all run inside a letter or
    // through High. Alarm 0's handler posts to Top with edge_to at 0, which
    // only High's actions in letters e and k (Hazard3) set, each arming
    // one alarm that lands inside that dispatch (the match disarms the
    // alarm); letter k's quiet post to Top is served by the raise that
    // follows it, inside the letter; and High's dispatch in letter l posts
    // to Top, its action outliving the letter - but nothing posts to High
    // between letters: alarm 0 is armed only inside a letter, its post
    // waited for, and core 1 sends nothing to core 0 (its one post to High
    // is refused by High's queue, and the doorbell is enabled on core 1
    // alone) and is parked in the bootrom before letter l ends. Letter i
    // checks the alarms disarmed and core 1 parked before the prompt. So a
    // 0 here is the base turn, never a dispatch.
    const bool at_base = rx_ceiling == 255u;
    const bool in_walk = rx_ceiling == 0u;
    print(serial, "  the UART's receive interrupt read the ceiling ", rx_ceiling,
          at_base   ? " (the loop's base)"
          : in_walk ? " (the base turn, held at 0 through its walk of the time events and its choice)"
                    : " (neither the base nor the base turn)",
          " when this letter's key arrived", crlf);
    bench.verdict("the ceiling the receive interrupt read as the key arrived is the loop's: 255 "
                  "at its base or 0 during its turn, the walk of the time events and the choice "
                  "(no dispatch of Top can be running then)",
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
bool refilled[latency_events];     ///< per event: a refill or the console line inside

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
/// The Cortex-M33:
///   the handler's tail after the stamp (a literal, a store, the frame's
///   release, the pop of the PC that loads EXC_RETURN)               8
///   the tail-chain into PendSV                                      4
///   PendSV's pair body, sixteen instructions to the BX and the BX
///   (two pushes, two stores, the SIO's CPUID and the table's entry
///   loaded: 19; no barrier on this core, cortexm/schedule.hpp) and
///   its return into thread mode (8)                                27
///   the entry, under PendSV's PRIMASK until its cpsie: the ceiling's
///   literal, which its load waits a cycle on, the load and the store
///   of 0 (5); cpsie, `base` into the stacked filler, the call (4)   9
///   activate(): nine registers saved, the served flag, the frame, two
///   literals and `base` stored (14); the count's load, `base` reloaded
///   and its test, the branch not taken (5); the no-hold value, the
///   hold flag and four addresses (6); Top's visit (the queue's two
///   cursors loaded and compared in line, the branch taken: 6) and the
///   compare with `above` (2); High's look (the two cursors, the
///   compare, the branch not taken, folded: 4); the ceiling raised to
///   High's index (1); the count read again and compared, its branches
///   folded (4); the dispatching index saved and raised and the call to
///   take() (6); take() (15); the slot, its test, and the stamp's
///   literal, which the stamp's load waits a cycle on (6)           69
///                                                                 ---
///                                                                 117
///
/// Hazard3:
///   the handler's tail after the stamp (a store, cm.popret: 5), the
///   return into the crt's dispatch and its jump back onto a 32-bit
///   instruction not 4-aligned (3), `meinext` read and its sign
///   branch taken (3), the sixteen loads, the frame and the `mret`
///   (20)                                                           31
///   the software interrupt's entry behind that `mret` and the
///   table's jump (1 + 2)                                            3
///   the trampoline: the frame, sixteen stores, the call            19
///   the pair body: eighteen instructions, the return               20
///   the trampoline's sixteen loads, the frame, the `mret`          20
///   the entry, MIE clear until its csrsi: the frame and sixteen
///   stores (17); the ceiling's load, the store of 0 and `base` into
///   the frame's byte 0 (3); csrsi, the call (3)                    23
///   activate(): cm.push of thirteen (14), four setups, the count's
///   load and `base`'s test (6), nine setups (9); Top's visit (the
///   queue's two cursors loaded and compared in line, the branch taken
///   onto a 32-bit instruction not 4-aligned: 6) and the compare with
///   `above` (1); High's look, the branch not taken (4); the raise (1);
///   the count read again and compared (4); the dispatching index
///   saved and raised and the call (6); take() with its return (16);
///   the slot, its test and the stamp's address (5)                 72
///                                                                 ---
///                                                                 188
/// of which the crt's two dispatches - the external interrupt's tail
/// (31) and the trampoline's save and restore around the body (39) -
/// are 70, and the entry's own save of the same sixteen registers 17.
///
/// On top of each, at most one kernel tick landing in the path: SysTick
/// on the Cortex-M33 (its entry 12, its nineteen instructions with three
/// literals 24, its BX and return 10 and, landing on an exception return,
/// that return abandoned and begun again 8: 54), the platform timer's trap
/// on Hazard3 (its entry 3 and the table's jump 2, the trampoline's save
/// and the call 19, the tick's instructions with its seconds' rollover and
/// return 30, the restore and `mret` 20: 74). And a fifth of the path's
/// count, rounded up: 24 and 38. The bound is 117 + 54 + 24 = 195 on the
/// Cortex-M33 and 188 + 74 + 38 = 300 on Hazard3; the letter reads the
/// kernel tick around each event, and every event over the fastest by more
/// than the margin below must have had a tick in it.
constexpr uint32_t latency_count = per_half(117u, 188u);
constexpr uint32_t latency_bound = per_half(195u, 300u);

/// The XIP cache's misses so far (both halves read the same counters):
/// a refill between two reads of a span is read as letter c reads a tick.
uint32_t xip_misses() { return Xip::accesses() - Xip::hits(); }

/// Every letter is registered through this: the XIP cache's two counters
/// cleared as the letter starts. They are 32-bit saturating counters
/// (rp2350/flash.hpp's Xip), and xip_misses() is their difference: left
/// uncleared, they stand at all ones after 2^32 accesses - measured, some
/// 225 s of letter l run back to back -, the difference stops moving,
/// and a refilled event reads as a clean one and is judged against a
/// bound that does not allow for it. A letter lasts well under a second.
/// Cleared for every letter, so that one added later cannot forget.
template <void (*letter)()>
void xip_cleared() {
    Xip::reset_counters();
    letter();
}

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
    /// The two a path's bound does not allow for: a refill, the console.
    bool refilled() const {
        const Lengtheners n = now();
        return n.misses != misses || n.uart != uart;
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
        const bool refill = before.refilled();
        if (i > 0u) {
            latency[i - 1u] = High::entered - edge_last;
            lengthened[i - 1u] = ticked;
            refilled[i - 1u] = refill;
        }
    }
    // The events that something lengthened are told apart from the
    // fastest, not from the model's count: the count is a cost model of
    // the listing (4 cycles under the fastest event on either half), and
    // a transfer's cost in that model is uncertain by a cycle or two, so
    // an event within the fastest plus 18 (nine taken transfers at two) is
    // the path itself and one beyond that had something else in it.
    uint32_t lo_raw = 0xFFFF'FFFFu;
    for (uint32_t i = 0; i < latency_events; ++i) {
        lo_raw = latency[i] < lo_raw ? latency[i] : lo_raw;
    }
    uint32_t hi_bounded = 0;   ///< the slowest event no refill and no console line lengthened
    for (uint32_t i = 0; i < latency_events; ++i) {
        if (latency[i] > lo_raw + 18u) {
            ++slow;
            if (lengthened[i]) {
                ++slow_ticked;
            }
        }
        if (!refilled[i] && latency[i] > hi_bounded) {
            hi_bounded = latency[i];
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
          "console line's handler inside ", slow_ticked, " of them; the slowest with no refill "
          "and no console line inside ", hi_bounded, crlf);
    bench.verdict("every interrupt's post started High within 1 ms", missed == 0u);
    bench.verdict("every event over the fastest + 18 had a tick, a cache refill or the console "
                  "line's handler in it",
                  slow == slow_ticked);
    // The bound allows for one kernel tick and not for a refill or the
    // console line, which are reported and not judged (the suite's method,
    // as letters e and g).
    bench.verdict("the latency stays under the path's bound (the arithmetic beside it) in every "
                  "event no refill and no console line lengthened",
                  hi_bounded <= latency_bound);
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
    // The scan, through activate_here() - the platform's entry and exit
    // written by hand - the call to the stamp after it included. The
    // Cortex-M33: the call (2); activate_here()'s push of six, the
    // ceiling's literal, load and store of 0, the move and the call (14);
    // activate()'s prologue to Top's look (25, letter c's); Top's visit
    // in line, its branch taken, and the compare with `above` (8); High's
    // and Mid's likewise (8 and 8); Low's, its queue's address a literal
    // (8); the compare with `above` at the Console's index, its branch
    // taken (3); the no-hold test, the result packed, the frame's release
    // and the pop of nine with the PC (20): 80 in activate(); the result's
    // unpack, the ceiling put back, the frame's release and the pop of
    // three with the PC (9): 105, and with the fifth 126. Hazard3: the
    // call (2); activate_here()'s cm.push, the ceiling's load and store of
    // 0, the move and the call (9); activate()'s prologue (29, letter
    // c's); Top's visit and the compare with `above` (7); High's, its
    // branch onto a 32-bit instruction not 4-aligned (7); Mid's (6);
    // Low's, its queue's address set up (6), and the compare with `above`
    // at the Console's index, its branch taken (3); the no-hold test, the
    // result packed and the cm.popret of thirteen (20): 78 in activate();
    // the result's unpack, the ceiling put back and the cm.popret onto
    // the stamp's load, a 32-bit instruction not 4-aligned (9): 98, and
    // with the fifth 118.
    constexpr uint32_t scan_bound = per_half(126u, 118u);
    // The quiet post: push() and the ceiling's compare ask no queue
    // whether it is empty. The Cortex-M33: the event's setup and the call
    // (5); post<Low>()'s prefix (6); the queue's push() - the core
    // check's SIO load and its branch (7), the guard's mrs and cpsid and
    // the cursor's arithmetic (12), the copy and the cursor's store (10),
    // msr (3), the pop of the PC (5): 37; the ceiling's compare, its
    // branch taken onto a 32-bit instruction off its word (8); the pop and
    // the tail call (4); the count of quiet posts raised - its literal,
    // the load, the add, the store, the return (8): 68, 82 with the fifth.
    // Hazard3: the event's store and the call (4); post<Low>()'s cm.push,
    // the setups and the call (8); push() - the core check, the guard's
    // csrrci, the cursor, the copy, csrsi, the return (31); the ceiling's
    // compare and its branch taken (6), cm.pop and the tail jump (5); the
    // count raised and the return (6): 60, 72 with the fifth.
    constexpr uint32_t quiet_bound = per_half(82u, 72u);
    // The road-2 post, from Mid's ceiling to High's empty dispatch and
    // back. The Cortex-M33: the event's store and the call (4);
    // post<High>()'s prefix and the call to push() (6); push() (37); the
    // ceiling's compare, its branch not taken and the call (7); the
    // request - the core check (the SIO's address, CPUID, the branch not
    // taken), the store, DSB and ISB (12); PendSV's entry (9), the pair
    // body (19) and the return into thread mode (8); the entry (9, letter
    // c's); activate() at Mid's ceiling - the prologue and Top's visit
    // (33), High's look, the raise and the count compared (9), the serve
    // - the dispatching index and the call to take() (6), take() (15),
    // the slot (4), the stamp (a load over the bridge, 3), the count, the
    // stack sample (21 in all), the trace's test (7), the action's test
    // (6), the trace's again (3), the drop flag and release() (8 + 13),
    // the branch back (2), the served flag, the dispatching index put
    // back and the ceiling's store of 0 (11) - the next choice: the count
    // read and the setups (11), Top's visit (8), High's, empty now (6),
    // `above` found at Mid's index (3), the no-hold test, the result
    // packed and the pop of nine with the PC (20) - 186 in activate(); the
    // entry's exit - the count kept, the FP context cleared, its ISB, the
    // two literals and the SVC (9) -, the SVC's entry (12), its handler -
    // the frame's three words and the filler loaded, the ceiling put
    // back, the count compared, the frame's release and the BX (13) - and
    // the return into the request, onto the instruction after its ISB
    // (11); the request's and the post's returns (7): 349, 419 with the
    // fifth. Hazard3: the event's store and the call (4); post<High>()'s
    // cm.push, the setups and the call to push() (8); push() - the core
    // check, the guard's csrrci, the cursor, the copy, csrsi, the return
    // (30); the ceiling's compare, its branch not taken, cm.pop and the
    // tail jump (10); the request - the hart check, its branch not taken -
    // to its spin, one turn (12); the trap - its entry behind the store
    // and the table's jump (2 + 2), the trampoline's save (19), the pair
    // body (20), the restore and the `mret` (20); the entry (23, letter
    // c's); activate() at Mid's ceiling - the prologue (29), Top's visit
    // and the compare with `above` (7), High's look, the raise and the
    // count compared (9), the serve - the dispatching index and the call
    // (6), take() (16), the slot (4), the stamp (3 over the bridge), the
    // count and the stack sample (14 in all), the trace's test (4), the
    // action's test (5), the trace's again (3), the drop flag and the
    // jump to release() (7), release() (2 + 11), the jump back (2), the
    // served flag, the dispatching index put back and the ceiling's store
    // of 0 (8) - the next choice: the count read and the setups (11),
    // Top's visit (7), High's, empty now (6), `above` found at Mid's
    // index (2), the no-hold test, the result packed and the cm.popret
    // (20) - 173 in activate(); the entry's exit - thirteen registers
    // restored, `base` and the count's low byte, mstatus written, which
    // clears MIE, the ceiling put back, the count compared, mepc and the
    // last three registers restored, the frame and the `mret` (32); the
    // spin's last turn and the return (4): 359, 431 with the fifth.
    // (Where a branch's figure differs from the same branch elsewhere,
    // its target is or is not a 32-bit instruction off its 4-byte
    // alignment in this image: the model's one cycle.)
    constexpr uint32_t road2_bound = per_half(419u, 431u);
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
    // look finds the event. The Cortex-M33: the handler's tail, the
    // tail-chain, the pair body, its return and the entry (48);
    // activate()'s prologue (25), Top's two cursors compared in line, the
    // branch not taken and folded (4), the raise, the count read again and
    // compared and the hold flag's moves (8), the dispatching index and
    // the call to take() (6), take() (15), the slot and the stamp's
    // literal (6): 112; with a tick (54) and a fifth rounded up (23),
    // 189. Hazard3: the external dispatch's tail, the software
    // interrupt's trap, the trampoline, the pair body and the entry
    // (116); activate()'s prologue (29), Top's two cursors compared in
    // line, the branch not taken (4), the raise (1), the count read again
    // and compared (5), the dispatching index and the call (5), take()
    // (16), the slot (4): 180; with a tick (74) and a fifth rounded up
    // (36), 290.
    constexpr uint32_t depth2_bound = per_half(189u, 290u);
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
        std::atomic_signal_fence(std::memory_order_seq_cst);
        TE::process();
        std::atomic_signal_fence(std::memory_order_seq_cst);
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
    // listing. The Cortex-M33: the stamp's store, the arguments and the
    // call (5); the frame, mrs and cpsid, the call to the unlink (10); the
    // unlink, out of line - the head's load, the walk's test, the two
    // stores of the unlink and the node's two, the return (19); now()'s
    // load, the deadline and the node's stores, the link at the list's
    // head, the tail's test, msr, the pop of the PC (27): 61, the masked
    // window the 42 from cpsid to msr; 74 with the fifth. Hazard3: the
    // stamp's store, the argument and the call (4); cm.push, two spills,
    // csrrci, the call (9); the unlink (14); the deadline, the node's
    // stores, the link, csrsi, cm.popret onto the stamp's address, a
    // 32-bit instruction not 4-aligned (23), that address (1): 51, the
    // masked window the 35 from csrrci to csrsi; 62 with the fifth. The
    // first of the 16 finds the list empty and its unlink stops at the
    // head, 53 and 45 by the same count: the least when that call's path
    // is already warm, and the bound, drawn on the longer path, holds
    // either.
    constexpr uint32_t arm_bound = per_half(74u, 62u);
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
    constexpr uint32_t section_bound = per_half(159u, 204u);   // derived below the loop
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
    // Cortex-M33: the stamp's store with its literal, the saved mask's
    // load and the guard's msr that lifts the mask (6); the instruction
    // that retires behind the msr and PendSV's entry (11), the pair body
    // (19) and its return into thread mode (8); the entry (9, letter c's);
    // activate() at the Console's ceiling - the prologue (25), Top's visit
    // and the compare with `above` (8), High's (8), Mid's look (4), the
    // raise and the count compared (6), the dispatching index and the call
    // to take() (7), take() (15), the slot and the stamp's literal (6):
    // 132, 159 with the fifth. Hazard3: the stamp's store, the guard's
    // test and the csrsi that lifts the mask (4), the trap taken at once
    // on the instruction after it (the pending bit has stood since the
    // post; letter k measures the raise's own latency); the trap's entry
    // behind the csrsi and the table's jump, the trampoline, the pair
    // body, the restore (2 + 2 + 19 + 20 + 20: 63); the entry (23, letter
    // c's); activate() at the Console's ceiling - the prologue (29), Top's
    // visit and the compare with `above` (7), High's, its branch onto a
    // 32-bit instruction not 4-aligned (7), Mid's look (4), the raise and
    // the count compared (5), the dispatching index and the call (7),
    // take() (16), the slot and the stamp's address (5): 170, 204 with
    // the fifth.
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

/// The platform's request, then the same eight stores.
[[gnu::noinline]] void request_then_sled() {
    P::pend_schedule();
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
          " of the eight stores behind it ran before the trap; behind the platform's request: ",
          req_hi, crlf);
    // The raw figure is the bus's fact the request does not depend on (rp2350/
    // schedule_hazard3.hpp): printed, and judged by nothing but the run
    // itself - every raise served once. The guarded figure is the
    // request's promise.
    bench.verdict("each raise served Top exactly once (20 raises)", each_once);
    bench.verdict("the platform's request returns after the scheduler it started: no store "
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
    bench.verdict("core 1 is in the bootrom, no program of this image running there",
                  core1_stage == 0u);
    bench.verdict("no time event is armed", !TE::next_deadline().has_value());
    bench.verdict("TIMER1's alarms 0 and 1 are disarmed, alarm 0's line quiet",
                  !Edge::armed<0>() && !Edge::armed<1>() && !Edge::raised<0>() &&
                      !Irq::pending(Edge::irq<0>()));
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
    // (64 + 48): 112 + 288 = 400. One run lies deeper than the dispatch
    // it precedes, on the Cortex-M33 alone: an interrupt taken at PendSV's
    // first instruction, before its cpsid, posts with the old ceiling and
    // asks again, and the request is taken at the entry's cpsie, after its
    // store of 0, as a run at ceiling 0 that serves nothing
    // (kernel/rubato.hpp, "The stack") - a basic frame, PendSV's two words
    // and an activate() below the delayed run's entry, 44 bytes under that
    // run's own dispatch sample, and an interrupt over it 36 + 32 more:
    // 112, inside 312. Hazard3 masks from the trap to the entry's store,
    // and has no such run.
    constexpr uint32_t below = per_half(312u, 400u);
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    print(serial, "  the bound below the deepest sample ", below, " bytes", crlf);
    // And above it: the words of activate()'s frame below its saved
    // registers, written only on the paths that use them - 28 bytes on the
    // Cortex-M33 (nine registers pushed, then 28), 44 on Hazard3 (cm.push
    // of thirteen into 96) - so the paint's edge may stand up to that far
    // above a dispatch's sample, by the release listing (letter l's Top,
    // at depth three, is such a sample: no path below it writes the
    // frame's lowest words).
    constexpr uint32_t frame_unwritten = per_half(28u, 44u);
    bench.verdict("the paint reached the deepest sampled stack pointer, but for the words of its "
                  "frame no path wrote",
                  used + frame_unwritten >= deepest);
    bench.verdict("the high-water mark is within the bound of the deepest sample (the arithmetic "
                  "beside it)",
                  used <= deepest + below);
    bench.verdict("and inside the 4 KB bank, with the paint's floor untouched",
                  lowest > paint_bottom);
}

// =============================================================================
// l - a Rubato on each core
// =============================================================================
volatile uint32_t paint1_top = 0;      ///< core 1's highest painted word's address + 4
volatile uint32_t paint1_bottom = 0;   ///< core 1's lowest painted word's address
volatile uint32_t core1_fpccr = 0;   ///< the Cortex-M33's FPCCR as core 1 read it

/// Core 1's stack bank: scratch_x, 4 KB below core 1's stack top
/// (rp2350/ld/rp2350_16m.ld).
constexpr uint32_t core1_bank_bytes = 4096u;

/// Core 1's program: its stack painted, its request set up in its own
/// registers, alarm 1's line and the doorbell's enabled in its own
/// controller, its kernel. No ticker of its own: core 1's pack arms no
/// time event, and the ruler is TIMER1, which both cores read - so core 0's
/// tick and its handler stay what letters a to k measured.
[[noreturn]] void core1_rubato() {
    {
        const uint32_t sp = stack_pointer();
        const uint32_t bottom = Core1::default_stack_top() - core1_bank_bytes + 64u;
        const uint32_t top = (sp - 64u) & ~3u;
        for (uint32_t a = bottom; a < top; a += 4u) {
            *reinterpret_cast<volatile uint32_t*>(a) = paint;
        }
        paint1_bottom = bottom;
        paint1_top = top;
    }
#if defined(BRIO_RP2350_CORE_M33)
    core1_fpccr = FPU->FPCCR;
#endif
    scheduler_init();
    Edge::clear<1>();
    Edge::interrupt<1>(true);
    Irq::clear_pending(Edge::irq<1>());
    Irq::enable(Edge::irq<1>());
    Drain1::enable();
    core1_stage = 2;
    enable_interrupts();
    K1::run();
}

/// Core 1 back into the bootrom, alarm 1 quiet and Low1's inbox emptied.
bool core1_park() {
    Edge::interrupt<1>(false);
    Edge::disarm<1>();
    Edge::clear<1>();
    const bool ok = Core1::reset();
    core1_stage = 0;
    Inbox<Low1>::clear();
    return ok;
}

/// Core 1 reset, its kernel's state put back while it is held (the
/// ceiling at the base, the trace empty), then launched into its Rubato;
/// true when its entry reported within 10 ms.
bool core1_launch() {
    bool ok = Core1::reset();
    core1_stage = 0;
    Inbox<Low1>::clear();
    P1::ceiling = 255;
    trace1_len = 0;
    ok = ok && Core1::protocol(&core1_rubato);
    const uint32_t t0 = Ruler::now();
    while (ok && core1_stage != 2u) {
        if (Ruler::now() - t0 >= 10u * one_ms) {
            return false;
        }
    }
    return ok;
}

/// Core 1's ceiling read from core 0: through a volatile access, since the
/// other core writes it.
uint8_t core1_ceiling() { return *static_cast<volatile uint8_t*>(&P1::ceiling); }

/// Both cores' windows on the ruler: the preempted dispatch's mark and its
/// resumption, written by the core whose dispatch it is.
volatile uint32_t w0_mark = 0;
volatile uint32_t w0_resumed = 0;
volatile uint32_t w1_mark = 0;
volatile uint32_t w1_resumed = 0;
volatile uint32_t k1_done = 0;              ///< Low1's actions ended, counted by core 1
volatile uint32_t bell1_last = 0;           ///< the ruler at core 1's doorbell handler's last statement
volatile uint32_t edge1_last = 0;           ///< the ruler at alarm 1's handler's last statement
volatile uint32_t bell1_misses = 0;         ///< the XIP cache's misses before the doorbell's stamp
volatile uint32_t edge1_misses = 0;         ///< and before alarm 1's
volatile uint32_t bell1_ticks = 0;          ///< core 0's tick count before the doorbell's stamp
volatile uint32_t edge1_ticks = 0;          ///< and before alarm 1's
volatile bool mispost_raised = true;        ///< core 1's request read after its mispost

/// Core 1's spin and the alarm's distance, in clk_sys cycles: an alarm 100
/// us into a 500 us spin on each core, so that each core's preemption
/// lands while the other core's preempted dispatch still spins - core 1's
/// mark trails core 0's by the crossing alone, a few microseconds warm.
constexpr uint32_t both_edge = 15'000u;
constexpr uint32_t both_spin = 75'000u;

/// What core 0 waits after core 1's last sign of a dispatch's end (Low1's
/// counter, or its ceiling back at 255) before it sends again: the rest
/// of core 1's way back to its base - the action's return, the trace's
/// last entry, release(), the next choice, activate()'s return and the
/// scheduler entry's exit - reads as about 160 cycles in the Cortex-M33's
/// release listing and about 150 in Hazard3's (a reading, not a count
/// instruction by instruction), and 3000 cycles (20 us) is many times
/// that. Sent sooner, the doorbell lands on that way back, where its post
/// to Low1 asks for nothing and is served by the same run, or by the run
/// the exit starts again in place when the count of quiet posts moved
/// (kernel/platform.hpp, the entry and the exit): correct, and no deeper,
/// but the crossing's latency would then hold the rest of the one before,
/// which is not what this letter measures.
constexpr uint32_t core1_settle = 3000u;

/// Core 1's road 1 from its doorbell handler's last statement to Low1's
/// first, at core 1's base, by the release listing under the file
/// header's models. The Cortex-M33: the handler's tail after the stamp
/// (a literal, a store, the frame's release, the pop of three with the
/// PC: 10), the tail-chain into PendSV (4), the pair body (19) and its
/// return into thread mode (8), K1's entry (9, letter c's), K1's
/// activate() - twelve registers saved, `base` kept, the served flag,
/// two literals, the count's load and `base`'s test, the no-hold value,
/// the hold flag and four addresses (26), Top1's two cursors compared in
/// line, the branch taken, and the compare with `above` (8), High1's
/// likewise, its compare folded (7), Low1's look (4), the raise (2), the
/// count read again and compared (4), the dispatching index and the call
/// to take() (7), take() (15), the slot and the stamp's literal (4):
/// 127, and with a fifth rounded up, 153. Hazard3: the handler's tail
/// (the store, cm.popret of six: 9), the crt's jump back, `meinext` and
/// its branch, the sixteen loads and the `mret` (3 + 3 + 20), the
/// software interrupt's entry and the table's jump (1 + 2), the
/// trampoline (19), the pair body (20), the restore (20), K1's entry
/// (23, letter c's), K1's activate() - cm.push of thirteen and the
/// setups (29), Top1's visit, its branch onto a 32-bit instruction not
/// 4-aligned, and the compare with `above` (7), High1's (6), Low1's look
/// (4), the raise (1), the count compared (4), the dispatching index and
/// the call (7), take() (16), the slot and the stamp's address (4): 198,
/// and with the fifth 238. No kernel tick lands on core 1: its pack arms
/// no time event and core 1 runs no ticker (core1_rubato()).
constexpr uint32_t bell_bound = per_half(153u, 238u);
/// And from alarm 1's handler's last statement to High1's first, over
/// Low1's dispatch at K1's ceiling 2: the same with alarm 1's tail (8 on
/// the Cortex-M33 - a pop of the PC alone -, 5 on Hazard3 - cm.popret of
/// two) and activate() taking High1 (its look, the raise and the count
/// compared: 9 and 9; the serve 6 and 5, take() 15 and 16, to the
/// stamp's literal 4 and 4): 116 and 186, and with the fifth 140 and 224.
constexpr uint32_t alarm1_bound = per_half(140u, 224u);
/// What lies below core 1's deepest stack sample, by the release
/// listing. A dispatch's sample is taken inside K1's activate(), whose
/// frame is above it. Below it at most: the dispatch's action and what it
/// calls (the Cortex-M33's widest is Low1's action in the second phase,
/// 24 bytes, calling note1(), 20: 44; Hazard3's Low1 and High1 actions,
/// 32, whose callees take no frame: 32); an interrupt landing there - on
/// the Cortex-M33 a basic frame and its alignment word (36: no FP
/// instruction runs on core 1, by the listing, so its FP context is never
/// active), on Hazard3 the crt's trap frame (64) -; and below that either
/// the deepest handler's frames (the doorbell's 24 and its push()'s 8 on
/// the Cortex-M33: 32; 48 on Hazard3, whose push() takes none) or, the
/// handler gone and its request pending, PendSV's two words and an
/// activate() not yet dispatching (8 + 48, twelve registers saved: 56)
/// on the Cortex-M33, and on Hazard3 in place of the trap frame and the
/// handler the entry's frame and such an activate() (80 + 80, cm.push of
/// thirteen into 80: 160, against 64 + 48). 44 + 36 + 56 = 136 on the
/// Cortex-M33, 32 + 160 = 192 on Hazard3. On the Cortex-M33 the run at
/// ceiling 0 an interrupt before PendSV's cpsid can leave at the entry
/// (letter h's comment) lies 44 bytes under the dispatch sample of the
/// run it delays, with an interrupt over it 36 + 32 more: 112, inside
/// 136.
constexpr uint32_t below1 = per_half(136u, 192u);

void tl_both_cores() {
    reset_scenario();
    sp1_low_dispatch = 0xFFFF'FFFFu;
    sp1_low_isr = 0xFFFF'FFFFu;
    const bool up = core1_launch();
    print(serial, "  core 1 launched into its Rubato pack: ", up ? "up" : "FAILED", crlf);
    bench.verdict("core 1 came up under Rubato (its entry reported within 10 ms)", up);
    if (!up) {
        (void)core1_park();
        return;
    }
#if defined(BRIO_RP2350_CORE_M33)
    const bool aspen1 = (core1_fpccr & FPU_FPCCR_ASPEN_Msk) != 0u;
    const bool lspen1 = (core1_fpccr & FPU_FPCCR_LSPEN_Msk) != 0u;
    print(serial, "  core 1's FPCCR ", hex(static_cast<uint32_t>(core1_fpccr)), ": ASPEN ",
          aspen1 ? 1 : 0, ", LSPEN ", lspen1 ? 1 : 0, crlf);
    bench.verdict("core 1's FPCCR.ASPEN and LSPEN are set, as core 0's: the pair body's FP part "
                  "rests on the same lazy save there",
                  aspen1 && lspen1);
#endif
    // Core 1's two road-1 latencies first, with core 0 asleep: core 0's
    // send to Low1, road 1 on core 1 from
    // its doorbell handler at its base (the handler's last statement to
    // Low1's first); then Low1 arms alarm 1 2000 cycles ahead and spins
    // 20000, and the alarm's post starts High1 over it (alarm 1's
    // handler's last statement to High1's first). One warm-up and sixteen,
    // each latency lengthened when the XIP cache missed between the
    // handler's read of the miss counter, just before its stamp, and the
    // dispatch's, just after its own - or when core 0's kernel tick count
    // moved between the same two points, the tick being the one interrupt
    // that wakes core 0 here: its
    // handler runs through the same cache and bus (measured on Hazard3
    // before the tick was read: one crossing of 160 at 120 cycles over
    // the others with no miss counted in its window; which of the tick's
    // effects lengthened it is not established). With core 0 asleep, the
    // rest is core 1's path.
    constexpr uint8_t crossings = 16;
    Low1::action = [](uint8_t) {
        Edge::alarm<1>(Ruler::now() + 2000u);
        spin(20000u);
        k1_done = k1_done + 1u;
    };
    High1::action = nullptr;
    console_drain();   // the lines above still leaving: core 0's UART handler would miss lines
    uint32_t bell_lo = 0xFFFF'FFFFu, bell_hi = 0, bell_clean_hi = 0;
    uint32_t alarm_lo = 0xFFFF'FFFFu, alarm_hi = 0, alarm_clean_hi = 0;
    uint8_t bell_lengthened = 0;
    uint8_t alarm_lengthened = 0;
    uint8_t crossing_missed = 0;
    for (uint8_t i = 0; i <= crossings; ++i) {
        const uint32_t r = k1_done;
        const uint32_t h = High1::runs;
        send<Low1>(Hit{1});
        // Core 0 asleep while core 1's two paths run, until its next
        // interrupt (the kernel tick at the latest): a core 0 spinning on the ruler and on the flash measured them
        // longer, 2 to 6 cycles on the Cortex-M33 and 11 to 14 on Hazard3,
        // and its own fetches fell in the window of the miss counter.
        wait_for_interrupt();
        const bool ran = wait_runs(k1_done, r, one_ms) && High1::runs == h + 1u;
        spin(core1_settle);
        if (!ran) {
            ++crossing_missed;
            continue;
        }
        if (i == 0u) {
            continue;
        }
        const uint32_t b = Low1::entered - bell1_last;
        bell_lo = b < bell_lo ? b : bell_lo;
        bell_hi = b > bell_hi ? b : bell_hi;
        if (Low1::entered_ticks != bell1_ticks || Low1::entered_misses != bell1_misses) {
            ++bell_lengthened;
        } else if (b > bell_clean_hi) {
            bell_clean_hi = b;
        }
        const uint32_t a = High1::entered - edge1_last;
        alarm_lo = a < alarm_lo ? a : alarm_lo;
        alarm_hi = a > alarm_hi ? a : alarm_hi;
        if (High1::entered_ticks != edge1_ticks || High1::entered_misses != edge1_misses) {
            ++alarm_lengthened;
        } else if (a > alarm_clean_hi) {
            alarm_clean_hi = a;
        }
    }
    // Core 1's side: Low1, started by core 0's send (road 1 from core 1's
    // doorbell handler at its base), posts to core 0's High under its own
    // mask (refused by the queue, nothing raised), arms alarm 1 and spins;
    // the alarm posts to High1 (road 1 over Low1), which posts to Top1
    // inside its dispatch (road 2).
    Low1::action = [](uint8_t) {
        note1('l', 'm');
        w1_mark = Ruler::now();
        {
            InterruptGuard g;
            post<High>(Hit{0});
            mispost_raised = request_pending();
        }
        Edge::alarm<1>(Ruler::now() + both_edge);
        spin(both_spin);
        w1_resumed = Ruler::now();
        note1('l', 'r');
        k1_done = k1_done + 1u;
    };
    High1::action = [](uint8_t) { post<Top1>(Hit{1}); };
    const char* const want0 = "Ls3 Lm3 Hs1 Ts0 Te0 He1 Lr3 Le3";
    const char* const want1 = "ls2 lm2 hs1 ts0 te0 he1 lr2 le2";
    uint8_t alike0 = 0;
    uint8_t alike1 = 0;
    uint8_t overlapped = 0;
    bool settled = true;
    bool refused = true;          ///< every mispost counted by High's queue, High not run by it
    bool quiet = true;            ///< and core 1's request not raised by it
    // Run 0 is the warm-up, not judged: core 1's first crossing of a cold
    // program pays the XIP cache's fill for both cores.
    console_drain();
    for (int run = 0; run <= 10; ++run) {
        reset_scenario();
        trace1_len = 0;          // core 1 is idle at its base: nothing writes it now
        mispost_raised = true;
        // Core 0's side: the same shape on alarm 0.
        Low::action = [](uint8_t) {
            note('L', 'm');
            w0_mark = Ruler::now();
            send<Low1>(Hit{1});
            edge_to = 1;
            edge_in(both_edge);
            spin(both_spin);
            w0_resumed = Ruler::now();
            note('L', 'r');
        };
        High::action = [](uint8_t) { post<Top>(Hit{1}); };
        const uint16_t mis0 = High::queue.misposts();
        const uint32_t h0runs = High::runs;
        const uint32_t d0 = k1_done;
        post<Low>(Hit{1});
        // Both settle: core 1's Low1 ended its action (within 2 ms of the
        // post's return, its spin being 500 us), then its dispatch - the
        // trace's last entry written, then the scheduler's exit putting
        // back 255, the ceiling of core 1's base (kernel/platform.hpp, the
        // entry and the exit) - within 1 ms. Seen once is settled, and the
        // read is not taken again: core 1's loop then turns once more with
        // the ceiling at 0 (its base turn, kernel/rubato.hpp's run()) before
        // its idle puts 255 back, and a second read can land in that turn.
        const bool done = wait_runs(k1_done, d0, 2u * one_ms);
        const uint32_t t0 = Ruler::now();
        bool back1 = core1_ceiling() == 255u;
        while (!back1 && Ruler::now() - t0 < one_ms) {
            back1 = core1_ceiling() == 255u;
        }
        const bool back0 = P::ceiling == console_index;
        if (!(done && back1 && back0)) {   // printed for the warm-up too, judged after it
            print(serial, "  run ", run, " did not settle: Low1's end ", done ? "seen" : "NOT seen",
                  ", core 1's ceiling ", core1_ceiling(), ", core 0's ", P::ceiling, crlf);
        }
        spin(core1_settle);
        if (run == 0) {
            continue;
        }
        settled = settled && done && back1 && back0;
        refused = refused && High::queue.misposts() == static_cast<uint16_t>(mis0 + 1u) &&
                  High::runs == h0runs + 1u;
        quiet = quiet && !mispost_raised;
        if (trace_is(want0)) {
            ++alike0;
        } else if (alike0 + 1 == run) {
            print_trace();
        }
        if (marks_are(trace1, trace1_len, want1)) {
            ++alike1;
        } else if (alike1 + 1 == run) {
            print_trace1();
        }
        // Each core's preemption inside the other core's preempted
        // dispatch, on the ruler both read: core 0's High started between
        // core 1's mark and its resumption, and core 1's High1 between
        // core 0's.
        const uint32_t h0 = High::entered;
        const uint32_t h1 = High1::entered;
        const bool inside0 = h0 - w1_mark <= w1_resumed - w1_mark;
        const bool inside1 = h1 - w0_mark <= w0_resumed - w0_mark;
        if (inside0 && inside1) {
            ++overlapped;
        }
    }
    Low1::action = nullptr;
    High1::action = nullptr;
    reset_scenario();
    print(serial, "  core 0: ", alike0, " of 10 runs traced ", want0, crlf);
    print(serial, "  core 1: ", alike1, " of 10 runs traced ", want1, crlf);
    print(serial, "  each core's preemption inside the other's preempted dispatch in ", overlapped,
          " of 10 runs; core 1's ceiling read after each settle ", core1_ceiling(), crlf);
    print(serial, "  core 1, road 1 from its doorbell at its base: the handler's last statement "
          "to Low1's first ", bell_lo, "..", bell_hi, " cycles over ", crossings, " crossings (",
          crossing_missed, " missed), at most ", bell_clean_hi, " on the ", crossings - bell_lengthened,
          " no XIP refill and no tick of core 0's lengthened (bound ", bell_bound, ")", crlf);
    print(serial, "  core 1, road 1 from alarm 1 over Low1: the handler's last statement to "
          "High1's first ", alarm_lo, "..", alarm_hi, " cycles, at most ", alarm_clean_hi,
          " on the ", crossings - alarm_lengthened, " no XIP refill and no tick of core 0's lengthened (bound ",
          alarm1_bound, ")", crlf);
    bench.verdict("core 0 under Rubato beside core 1's: road 1 then road 2, every ceiling its own "
                  "index (10 runs)",
                  alike0 == 10u);
    bench.verdict("core 1 under Rubato through the same scheduler entry: road 1 from its doorbell, "
                  "road 1 from its alarm, road 2, every ceiling its own index (10 runs)",
                  alike1 == 10u);
    bench.verdict("the two cores preempted at once, each inside the other's preempted dispatch "
                  "(10 runs)",
                  overlapped == 10u);
    bench.verdict("both settle: core 1's ceiling 255 at its base, core 0's the Console's (10 runs)",
                  settled);
    bench.verdict("core 1's post to core 0's High was refused and counted by High's queue, and "
                  "ran nothing (10 runs)",
                  refused);
    bench.verdict(hazard3 ? "and raised no software interrupt flag on core 1 (10 runs)"
                          : "and left PendSV not pending on core 1 (10 runs)",
                  quiet);
    bench.verdict("every crossing started Low1 on core 1 and its alarm High1, within 1 ms (16 "
                  "crossings)",
                  crossing_missed == 0u);
    bench.verdict("core 1's road 1 from its doorbell under its bound (the arithmetic beside it) "
                  "in every crossing no refill and no tick lengthened, at least one",
                  bell_lengthened < crossings && bell_clean_hi <= bell_bound);
    bench.verdict("core 1's road 1 from alarm 1 under its bound (the arithmetic beside it) in "
                  "every crossing no refill and no tick lengthened, at least one",
                  alarm_lengthened < crossings && alarm_clean_hi <= alarm1_bound);

    // Core 1's stack, read by core 0 with core 1 idle at its base.
    uint32_t lowest = paint1_top;
    for (uint32_t a = paint1_bottom; a < paint1_top; a += 4u) {
        if (*reinterpret_cast<const uint32_t*>(a) != paint) {
            lowest = a;
            break;
        }
    }
    const uint32_t used = paint1_top - lowest;
    const uint32_t deepest_dispatch = paint1_top - sp1_low_dispatch;
    const uint32_t deepest_isr = paint1_top - sp1_low_isr;
    print(serial, "  core 1: painted ", paint1_top - paint1_bottom, " bytes below its entry's frame (",
          hex(paint1_bottom), "..", hex(paint1_top), "); used ", used,
          " bytes; the deepest stack pointer sampled: a dispatch at ", deepest_dispatch,
          ", an interrupt at ", deepest_isr, "; the bound below the deepest sample ", below1,
          " bytes", crlf);
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    // A dispatch's sample is the stack pointer inside activate(), and the
    // words of that frame below its saved registers are written only on
    // the paths that use them: none on the Cortex-M33 (core 1's activate()
    // pushes twelve registers, r0 to r2 as its locals' slots, and
    // allocates nothing more), seven on Hazard3 (cm.push of thirteen
    // registers into 80 bytes, the 28 below them its spill slots). So the
    // paint's edge may stand that far above the sample, 0 and 28 bytes, by
    // the release listing.
    constexpr uint32_t frame_unwritten = per_half(0u, 28u);
    bench.verdict("core 1's paint reached its deepest sampled stack pointer, but for the words "
                  "of its frame no path wrote",
                  used + frame_unwritten >= deepest);
    bench.verdict("core 1's high-water mark is within the bound of its deepest sample (the "
                  "arithmetic beside it)",
                  used <= deepest + below1);
    const bool parked = core1_park();
    bench.verdict("core 1 back in the bootrom", parked);
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
// the macro binds for the half the image is built for - the pair body, for
// a Rubato on each core.
extern "C" void isr_uart0() {
    uart_interrupts = uart_interrupts + 1u;
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void isr_systick() { brio::Ticker::tick(); }

BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K, K1)

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

/// TIMER1's alarm 1, core 1's in letter l: the raised flag cleared, the
/// post to High1, and the ruler as the last statement.
extern "C" void isr_timer1_1() {
    Edge::clear<1>();
    sample_sp(sp1_low_isr);
    brio::post<High1>(Hit{1});
    edge1_misses = xip_misses();
    edge1_ticks = brio::Ticker::ticks();
    edge1_last = Ruler::now();
}

/// The doorbell, one name for both cores (3.1.6: SIO_IRQ_BELL is
/// core-local): enabled on core 1 alone, whose crossing it lands as a
/// local post to Low1, and the ruler as the last statement.
extern "C" void isr_sio_bell() {
    if (brio::core_id() == 1u) {
        sample_sp(sp1_low_isr);
        Drain1::isr();
        bell1_misses = xip_misses();
        bell1_ticks = brio::Ticker::ticks();
        bell1_last = Ruler::now();
    }
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

    bench.letter('a', "the ceiling, road 1 at depth 1", xip_cleared<ta_ceiling>);
    bench.letter('b', "road 2: the chain inside the posts", xip_cleared<tb_road2>);
    bench.letter('c', "the latency of road 1", xip_cleared<tc_latency>);
    bench.letter('d', "the rulers, the scan and the post, in cycles", xip_cleared<td_scan>);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", xip_cleared<te_depth2>);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", xip_cleared<tf_timers>);
    bench.letter('g', "the request under the mask: served when the section ends", xip_cleared<tg_mask>);
#if defined(BRIO_RP2350_CORE_M33)
    bench.letter('j', "the FPU's registers across a preemption, FPCCR's lazy save", xip_cleared<tj_fpu>);
#else
    bench.letter('k', "the software interrupt: its latency, the registers, the flag", xip_cleared<tk_softirq>);
#endif
    bench.letter('l', "a Rubato on each core, one scheduler entry, two ceilings", xip_cleared<tl_both_cores>);
    bench.letter('i', "dark: every object idle, core 1 parked, nothing lit, the request as set",
                 xip_cleared<ti_dark>);
    bench.letter('h', "the stack's high-water mark", xip_cleared<th_stack>);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
