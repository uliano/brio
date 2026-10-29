// test_rp2040_rubato - the bench suite of Rubato, the preemptive kernel
// (kernel/rubato.hpp), on the RP2040: Rp2040RubatoPlatform<core>, its
// scheduling request (PendSV, cortexm/schedule.hpp's, asked only from the
// platform's own core) and the thread-mode scheduler on the Cortex-M0+,
// the one PendSV entry that serves a kernel on each core
// (rp2040/schedule.hpp's scheduler_exception_pair), against the model the
// host suite specifies (test/test_rubato).
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs INSIDE the kernel it tests: core 0's program is a Rubato pack
// of five objects, from the highest priority down - Top, High, Mid, Low
// and Console - and the Console, the lowest, owns the UART and runs the
// letters in its own dispatch. Its print is the only writer of the port;
// the other objects write what they saw into a trace in memory, and the
// Console judges and prints it after. A post from a letter is therefore a
// post from the Console's dispatch: to an object above it, the kernel's
// road 2. Core 1 is held in the bootrom for every letter but two: letter
// j launches it into a small Tenuto program, letter k into a Rubato pack
// of its own, and each puts it back when it ends.
//
// NOTHING TO WIRE, NOTHING LIT. The console is UART0 on GP0/GP1 through
// the Debug Probe's bridge; the suite configures no other pad, and GP25,
// the LED's pad on both boards, stays dark (letter i checks it). The
// interrupt every road-1 letter of core 0 lands is alarm 0 of the system
// timer, a line of its own (TIMER_IRQ_0) enabled in core 0's NVIC alone;
// core 1's, in letter k, is alarm 1 in core 1's NVIC. An alarm is placed
// in whole microseconds (rp2040/timer.hpp), 125 cycles of clk_sys each,
// which places an edge to within one microsecond: enough for an edge
// inside a spin of hundreds of microseconds, and the latencies are read
// from the handler's own last statement, not from the alarm.
//
// THE RULER. This core has no DWT cycle counter (ARMv6-M), and this chip
// no 32-bit timer on clk_sys. The ruler is the kernel ticker's
// BasicTicker::cycles() over core 0's SysTick (cortexm/ticker.hpp): the
// tick count and SysTick's position composed, five reads and a
// consistency check, a cycle of resolution; its two-read cost was 68
// cycles on the STM32G0B1RE (64 MHz, its flash at two wait states), and
// letter d measures it here and checks it against the system timer over
// a millisecond. Core 1's ruler, in letter k, is its own SysTick's,
// CoreTicker<1>::cycles(); what crosses the cores is read on the system
// timer, the one clock both share.
//
// WHERE THE SCHEDULER RUNS: in thread mode, PendSV returning into it
// through a second exception frame and the scheduler leaving through SVC
// (cortexm/schedule.hpp), so PendSV is not active while the dispatches
// run, and an interrupt's post preempts the dispatch it lands in at any
// depth - the Console's own dispatch included, since the UART's interrupt
// starts it at the loop's base. THE ONE PendSV ENTRY IS THE PAIR BODY in
// this image, for every letter: letter k runs a Rubato on each core, the
// two cores share one vector table, and a vector is bound once
// (rp2040/schedule.hpp). On core 0 the pair body takes the path the
// single body would take with six instructions in place of one literal
// and no DSB, two cycles more (that file's header); every bound below is
// drawn for the pair body.
//
// The letters (each bound's arithmetic beside its verdict in the code):
//   a  THE CEILING, ROAD 1 AT DEPTH 1. Low's handler, started by the
//      Console's post, arms alarm 0 16 us ahead and spins 20000 cycles
//      (160 us) at a mark; the interrupt posts to High. The trace says
//      High's dispatch landed before Low resumes, and every entry reads
//      the ceiling at its own object's index; the Console reads its own,
//      4, after the chain, and the UART's receive interrupt read the
//      loop's ceiling when the letter's key arrived - 255 at the base, or
//      0 while the base turn walks the time events. Ten runs, all alike.
//   b  ROAD 2: Low posts to Mid, Mid to High, inside the handlers; High
//      posts to Low. The trace nests High inside Mid inside Low, and
//      Low's second event is served after its first dispatch ends, never
//      inside it. Ten runs, all alike.
//   c  THE LATENCY OF ROAD 1, the Console's wait loop interrupted: the
//      ruler at the interrupt handler's last statement and at High's
//      handler's first, min, median and max over 100 events after one
//      warm-up, the console drained first; the bound is the sum of the
//      M0+'s own cycle counts along the path, written beside it.
//   d  THE RULER, THE SCAN AND THE POST: the ruler's two-read cost and its
//      agreement with the system timer over a millisecond; activate() with
//      nothing ready below the Console's ceiling (four empty queues, the
//      fifth object at the ceiling), and from Mid's handler one post that
//      asks for nothing (to Low, below it) against one that runs High's
//      dispatch inside it (road 2), each the least of 16, the ruler's own
//      cost taken off.
//   e  DEPTH 2: High's handler - started by road 2 (the Console's post),
//      then by road 1 (an interrupt over the Console) - spins at a mark
//      while alarm 0 posts to Top. Top's entry lands before High resumes;
//      the latency printed. Ten runs each.
//   f  TIME EVENTS UNDER PREEMPTION, in three parts. (1) Three one-shots
//      matured in one turn, the lowest object's armed last so walked
//      first, posted the way Rubato::run() posts them - the walk with the
//      ceiling held at 0, then the scheduler: nothing is dispatched during
//      the walk and the three are served highest first. (2) Low's handler
//      runs 200 turns of arm(), arm(), disarm(), most of each turn inside
//      a masked window; alarm 0's interrupt lands there and its High
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
//   j  ROAD 1 ACROSS THE CORES: core 1 launched into a Tenuto program of
//      one object, Relay, which answers every event core 0 sends it with a
//      send to High on core 0. First a mispost: Relay posts to High, and
//      the queue refuses and counts it with no PendSV pended on core 1.
//      Then Low's handler sends and spins; core 0's doorbell handler posts
//      to High, and High's dispatch starts after the doorbell handler's
//      last statement and before Low resumes (ten runs, traced, the
//      handler's own entry in the trace); then the latency from that last
//      statement to High's first over 100 events over the Console's wait,
//      against letter c's count with the doorbell handler's tail in place
//      of the alarm's and core 1's share of the XIP cache added.
//   k  A RUBATO ON EACH CORE: core 1 launched into a pack of three of its
//      own (Top1, High1, Low1) on Rp2040RubatoPlatform<1>, PendSV's
//      priority set in its own SHPR3, every preemption through the one
//      pair body. Core 0's Low sends to core 1's Low1 (road 1 from core
//      1's doorbell at its base), then both cores run the same shape at
//      once: a spin at a mark, the core's own alarm posting to its High,
//      which posts to its Top inside its handler (road 2). Each core
//      writes its own trace with its own ceiling; read after both settle,
//      each shows its own indices, and the system timer shows each core's
//      preemption landing inside the other core's preempted dispatch. Ten
//      runs. Then core 1's stack: painted by its entry, its high-water
//      mark against the deepest stack pointer its dispatches and its
//      interrupt sampled.
//   i  DARK: every object of core 0 idle, core 1 back in the bootrom, no
//      time event armed, no alarm armed and alarm 0's line quiet, GP25 not
//      an output, the ceiling the Console's own, interrupts enabled.
//   h  THE STACK, the last letter: main() painted core 0's free stack
//      below its own frame down to the bottom of its bank; the high-water
//      mark against the deepest stack pointer the dispatches and the
//      interrupts sampled in letters a..k plus what lies below a sample (a
//      basic exception frame, the handlers' own frames), the arithmetic
//      beside the verdict.
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/flash.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/schedule.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/inbox.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Rp2040RubatoPlatform<0>;
using P1 = Rp2040RubatoPlatform<1>;   // core 1 under Rubato (letter k)
using P1T = Rp2040Platform<1>;        // core 1 under Tenuto (letter j)

/// Core 0's SysTick composed with its tick count: a count a cycle.
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return CoreTicker<0>::cycles(); }
};

/// The alarm of the system timer core 0's road-1 letters land.
constexpr uint8_t edge_alarm = 0;
/// Core 1's, in letter k.
constexpr uint8_t edge_alarm1 = 1;

constexpr UartPins console_pins{.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

/// One millisecond of clk_sys: the letters' wait for a dispatch that
/// should have run within a few hundred cycles.
constexpr uint32_t one_ms = SysClock::hz / 1000u;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;
static_assert(SysClock::hz % 1'000'000u == 0u, "a whole number of cycles per microsecond");

// =============================================================================
// The trace, the stamps, the stack samples
// =============================================================================

struct Mark {
    char who;        ///< 'T', 'H', 'M', 'L', 'C', 'D' (the doorbell); 't', 'h', 'l' on core 1
    char what;       ///< 's' start, 'm' the mark, 'p' after a post, 'r' resumed, 'e' end, 'a' after, 'x' exit
    uint8_t ceiling; ///< the writing core's ceiling when it was written
};

constexpr uint8_t trace_capacity = 48;

/// A trace written by one core, with that core's ceiling.
template <class Plat>
struct Trace {
    static inline Mark marks[trace_capacity];
    static inline volatile uint8_t len = 0;

    /// One entry, from a dispatch or a handler: the append is the
    /// decision a mask covers, three stores.
    static void note(char who, char what) {
        InterruptGuard g;
        const uint8_t n = len;
        if (n < trace_capacity) {
            marks[n] = Mark{who, what, Plat::ceiling};
            len = static_cast<uint8_t>(n + 1u);
        }
    }

    static void clear() {
        InterruptGuard g;
        len = 0;
    }

    static void print_all() {
        print(serial, "  trace:");
        for (uint8_t i = 0; i < len; ++i) {
            print(serial, " ", marks[i].who, marks[i].what, marks[i].ceiling);
        }
        print(serial, crlf);
    }

    /// The trace equals `want`, written as "Ls3 Lm3 Hs1 ..." (who, what,
    /// ceiling) with single spaces.
    static bool is(const char* want) {
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
};

using Trace0 = Trace<P>;
using Trace1 = Trace<P1>;

volatile bool tracing = true;

void note(char who, char what) { Trace0::note(who, what); }

/// The lowest main-stack pointer a dispatch or an interrupt saw, per core.
volatile uint32_t sp_low_dispatch = 0xFFFF'FFFFu;
volatile uint32_t sp_low_isr = 0xFFFF'FFFFu;
volatile uint32_t sp1_low_dispatch = 0xFFFF'FFFFu;
volatile uint32_t sp1_low_isr = 0xFFFF'FFFFu;

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

/// Alarm 0 `us` microseconds from now: one edge, the handler clears it
/// (the match disarms the alarm by itself, rp2040/timer.hpp).
[[gnu::always_inline]] inline void edge_in(uint32_t us) { Timer::alarm_in(edge_alarm, us); }

// =============================================================================
// Core 0's pack
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
    static inline volatile uint32_t entered_us = 0;
    static inline volatile uint32_t runs = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        entered = Ruler::now();
        entered_us = Timer::now_low();
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

/// What core 0's doorbell handler drains: High alone receives across.
using Drain0 = Inboxes<High>;

/// The ceiling the UART's receive interrupt read when a byte arrived.
volatile uint8_t rx_ceiling = 0;

// =============================================================================
// Core 1: a Tenuto program (letter j) and a Rubato pack (letter k)
// =============================================================================

/// Letter j's event to core 1: answer with a send to High - or, with
/// `mispost`, with a post to High, which is core 0's and refused.
struct Fire {
    uint8_t n;
    bool mispost;
};

/// Core 1's one object under Tenuto: every Fire answered by a Hit sent to
/// core 0's High; a mispost made with core 1's interrupts masked, so that
/// a PendSV it pended would stand pending where the dispatch reads it.
struct Relay {
    using Event = Fire;
    static inline EventQueue<Event, 4, P1T> queue;
    static constexpr uint8_t inbox_depth = 4;
    static inline volatile uint32_t runs = 0;
    static inline volatile uint32_t misposted = 0;
    static inline volatile bool pended = false;
    static void init() {}
    static void dispatch(const Event& e) {
        runs = runs + 1u;
        if (e.mispost) {
            InterruptGuard g;
            post<High>(Hit{e.n});
            pended = (SCB->ICSR & SCB_ICSR_PENDSVSET_Msk) != 0u;
            misposted = misposted + 1u;
        } else {
            send<High>(Hit{e.n});
        }
    }
};
using K1T = Tenuto<P1T, Relay>;

/// Core 1's ruler, its own SysTick's.
struct Ruler1 {
    [[gnu::always_inline]] static uint32_t now() { return CoreTicker<1>::cycles(); }
};

/// Core 1's three under Rubato, the shape of core 0's Node with core 1's
/// trace and stack samples.
template <uint8_t id>
struct Node1 {
    using Event = Hit;
    static constexpr char name = "thl"[id];
    static inline EventQueue<Event, 4, P1> queue;
    static constexpr uint8_t inbox_depth = 4;
    static inline void (*action)(uint8_t) = nullptr;
    static inline volatile uint32_t entered_us = 0;
    static inline volatile uint32_t runs = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        entered_us = Timer::now_low();
        runs = runs + 1u;
        sample_sp(sp1_low_dispatch);
        Trace1::note(name, 's');
        if (action != nullptr) {
            action(e.n);
        }
        Trace1::note(name, 'e');
    }
};

using Top1 = Node1<0>;
using High1 = Node1<1>;
using Low1 = Node1<2>;
using K1 = Rubato<P1, Top1, High1, Low1>;

/// Core 1's doorbell drains, one per program it may run.
using Drain1T = Inboxes<Relay>;
using Drain1R = Inboxes<Low1>;

/// Which program core 1 runs: 0 none (the bootrom), 1 Tenuto, 2 Rubato.
/// Written by core 1's entry, read by its doorbell handler and core 0.
volatile uint32_t core1_stage = 0;

// =============================================================================
// The interrupt the road-1 letters land, and the doorbell's
// =============================================================================

volatile uint8_t edge_to = 1;           ///< 0: post to Top, 1: to High
volatile uint32_t edge_last = 0;        ///< the ruler at the alarm handler's last statement
volatile uint32_t edge_hits = 0;
volatile uint32_t bell_last = 0;        ///< the ruler at the doorbell handler's last statement
volatile uint32_t bell_hits = 0;

void reset_scenario() {
    Trace0::clear();
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

/// Let the console's transmitter fall silent (at most 50 ms, 6.25 M
/// cycles; 230 bytes at 115200 are 20 ms): its interrupt, landing in a
/// measured path, would be measured.
void console_drain() {
    const uint32_t t0 = Ruler::now();
    while (!Serial::tx_idle() && Ruler::now() - t0 < 50u * one_ms) {
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

/// The XIP cache's misses so far, both cores' (the counters are the
/// cache's, rp2040/flash.hpp's Xip): what tells an event a line refill
/// landed in from one that a tick did.
uint32_t xip_misses() { return Xip::accesses() - Xip::hits(); }

/// Every letter is registered through this: the XIP cache's two counters
/// cleared as the letter starts. They are 32-bit saturating counters
/// (rp2040/flash.hpp's Xip), and xip_misses() is their difference: left
/// uncleared, they stand at all ones after 2^32 accesses, the difference
/// stops moving, and a refilled event reads as a clean one and is judged
/// against a bound that does not allow for it. A letter lasts well under a second.
/// Cleared for every letter, so that one added later cannot forget.
template <void (*letter)()>
void xip_cleared() {
    Xip::reset_counters();
    letter();
}

// =============================================================================
// Core 1's launches
// =============================================================================

constexpr uint32_t paint = 0xC5AC'CE55u;
uint32_t paint1_top = 0;      ///< core 1's highest painted word's address + 4
uint32_t paint1_bottom = 0;   ///< core 1's lowest painted word's address

/// The bottom of core 1's stack: its bank, 4 KB below its top
/// (rp2040/ld/rp2040_2m.ld's scratch_x).
uint32_t core1_bank_bottom() { return Core1::default_stack_top() - 4096u; }

[[noreturn]] void core1_tenuto() {
    CoreTicker<1>::init(clock);   // core 1's own SysTick
    Drain1T::enable();            // core 1's doorbell, in ITS NVIC
    core1_stage = 1;
    enable_interrupts();
    K1T::run();
}

[[noreturn]] void core1_rubato() {
    // Paint core 1's free stack from 64 bytes below this frame down to 64
    // bytes above its bank's floor.
    {
        const uint32_t sp = __get_MSP();
        paint1_bottom = core1_bank_bottom() + 64u;
        paint1_top = (sp - 64u) & ~3u;
        for (uint32_t a = paint1_bottom; a < paint1_top; a += 4u) {
            *reinterpret_cast<volatile uint32_t*>(a) = paint;
        }
    }
    CoreTicker<1>::init(clock);
    scheduler_init();             // PendSV's priority in core 1's own SHPR3
    Timer::clear(edge_alarm1);
    Timer::interrupt(edge_alarm1, true);
    Nvic::clear_pending(Timer::irq(edge_alarm1));
    Nvic::enable(Timer::irq(edge_alarm1));   // core 1's NVIC
    Drain1R::enable();
    core1_stage = 2;
    enable_interrupts();
    K1::run();
}

/// Core 1 out of whatever it runs and into the bootrom; with core 0's
/// doorbell off around it (the reset's word comes through the FIFO).
bool core1_park() {
    SioDoorbell<0>::disable();
    const bool ok = Core1::reset();
    core1_stage = 0;
    Inbox<High>::clear();
    Inbox<Relay>::clear();
    Inbox<Low1>::clear();
    SioDoorbell<0>::pop_all();
    Nvic::clear_pending(SioDoorbell<0>::irq());
    SioDoorbell<0>::enable();
    return ok;
}

/// Core 1 reset, the inboxes emptied while nobody sends, then launched
/// into `entry`; true when its entry reported `stage` within 10 ms.
bool core1_launch(void (*entry)(), uint32_t stage) {
    SioDoorbell<0>::disable();
    bool ok = Core1::reset();
    core1_stage = 0;
    Inbox<High>::clear();
    Inbox<Relay>::clear();
    Inbox<Low1>::clear();
    ok = ok && Core1::protocol(entry);
    SioDoorbell<0>::pop_all();
    Nvic::clear_pending(SioDoorbell<0>::irq());
    SioDoorbell<0>::enable();
    const uint32_t t0 = Ruler::now();
    while (ok && core1_stage != stage) {
        if (Ruler::now() - t0 >= 10u * one_ms) {
            return false;
        }
    }
    return ok;
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
    // Mid's and Low's), and Top has two senders. One is alarm 0's
    // handler with edge_to at 0, which only letter e's High sets, arming
    // one alarm that lands inside its own spin; an alarm disarms itself at
    // its match (rp2040/timer.hpp). The other is High's dispatch in letter
    // k, whose action outlives the letter - but nothing posts to High
    // between letters: alarm 0 is armed only inside a letter, tens of
    // microseconds out, the letter waiting for its post, and the one
    // sender across, core 1 (core 0's doorbell drains High alone), is
    // parked in the bootrom with the inboxes cleared before letters j and
    // k end. Letter i checks both, no alarm armed and core 1 parked,
    // before the prompt. So a 0 here is the walk, never a dispatch.
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
            edge_in(16u);        // 16 us into a 160 us spin: the edge lands inside it
            spin(20000u);
            note('L', 'r');
        };
        const uint32_t h0 = High::runs;
        post<Low>(Hit{1});
        if (Trace0::is(want)) {
            ++alike;
        } else if (alike == run) {
            Trace0::print_all();   // the first that differs
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
        if (Trace0::is(want)) {
            ++alike;
        } else if (alike == run) {
            Trace0::print_all();
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

/// THE PATH'S COUNT, the road-1 path from an interrupt handler's last
/// statement to High's first, in the Cortex-M0+'s own cycle counts over
/// the release listing (table 81 of the RP2040 datasheet, 2.4.3.3: a
/// load or store 2, one to the single-cycle IO port 1, BL 3, BX 2, a
/// taken branch 2, PUSH and POP 1 + N, a POP into the PC 3 + N, MRS and
/// MSR 3, DSB and ISB 3; the exception's entry 15, 2.4.3.6.1's worst case
/// for a zero-wait system; its return 10 and a tail-chain 11, the
/// Cortex-M0+'s published figures, which this datasheet does not give and
/// the STM32G0's suite takes from the same place). The stamp at each end
/// is a call of the ruler, cycles(), out of line: 51 cycles, 22 to its
/// sample of SysTick's VAL and 29 after it, so its share in the
/// difference is the part of the first call after its sample and the
/// part of the second before it:
///   the alarm handler's tail from the first stamp's sample - the rest
///   of cycles(): the second ICSR read, the tick count again, the
///   consistency test, the composition and the pop into the PC (29);
///   the stamp's store with its literal (4); the pop of four into the
///   PC (7)                                                          40
///   the tail-chain into PendSV                                      11
///   the pair body to its BX (22), the BX and the return into thread
///   mode (12)                                                       34
///   the entry: cpsie, the call                                       4
///   activate() to High's dispatch: five registers saved, the frame,
///   the ceiling's address and value kept on the stack and the
///   ceiling's test (20); Top's visit - the queue's two cursors loaded
///   and compared in line (empty() is forced inline), the compare with
///   the ceiling (13); High's cursors compared (9)                   42
///   High's dispatch to its first stamp's sample: the ceiling's raise,
///   the dispatching index's raise and the call of take() (16), take()
///   on its longer path, the cursor not wrapping (22); the slot kept
///   and the call of cycles() (8); cycles() to its sample - its frame,
///   three literals, LOAD, the tick count, the first ICSR read and VAL
///   (22)                                                            68
///                                                                  ---
///                                                                  199
/// with every fetch a hit of the XIP cache, which answers in one cycle
/// (2.6.3.1) - zero wait states, so 199 is the warm count. A line the
/// cache misses costs its fetch over the QSPI bus, 50 to 80 cycles at
/// 125 MHz (docs/rp2040/platform.md, "A handler in SRAM"); the letter
/// reads the cache's miss counter around each event, as it reads the
/// kernel tick, and every event over the warm count must have had one or
/// the other in its window. The bound for the slowest: the warm count;
/// one SysTick handler landing in the path - its entry (15), its body on
/// core 0 (27 by the listing), its return (10) and, landing on an
/// exception return, that return begun again (12): 64; one line missed
/// (80); and a fifth of the warm count (40): 383.
constexpr uint32_t latency_count = 199u;
constexpr uint32_t latency_bound = 383u;

/// One latency run: `events` edges from `edge`, each followed by a wait
/// for High's first statement; returns the misses, fills `latency[]`,
/// counts the slow events and which of them a tick or a line refill
/// explains.
struct Slow {
    uint32_t missed;
    uint32_t slow;
    uint32_t explained;
};

template <class Place, class Stamp>
Slow latency_run(Place place, Stamp stamp, uint32_t warm_count) {
    Slow s{0, 0, 0};
    // The cache's counters saturate (rp2040/flash.hpp), at 2^32 accesses,
    // about a minute of execution: cleared here, a run lasts some 100 ms.
    Xip::reset_counters();
    // One warm-up event (the cache) not counted, then 100; each waits at
    // most 1 ms for High's first statement.
    for (uint32_t i = 0; i <= latency_events; ++i) {
        const uint32_t r = High::runs;
        const uint32_t t0 = CoreTicker<0>::ticks();
        const uint32_t m0 = xip_misses();
        place();
        if (!wait_runs(High::runs, r, one_ms)) {
            ++s.missed;
            flush();
        }
        const bool ticked = CoreTicker<0>::ticks() != t0;
        const bool refilled = xip_misses() != m0;
        if (i > 0u) {
            const uint32_t d = High::entered - stamp();
            latency[i - 1u] = d;
            if (d > warm_count) {
                ++s.slow;
                if (ticked || refilled) {
                    ++s.explained;
                }
            }
        }
    }
    return s;
}

void tc_latency() {
    reset_scenario();
    tracing = false;
    console_drain();
    const Slow s = latency_run([] { edge_to = 1; edge_in(4u); }, [] { return edge_last; },
                               latency_count);
    sort(latency, latency_events);
    const uint32_t lo = latency[0];
    const uint32_t med = latency[latency_events / 2u];
    const uint32_t hi = latency[latency_events - 1u];
    print(serial, "  road 1 over the Console's wait: the interrupt's last statement to High's "
          "first, min ", lo, " median ", med, " max ", hi, " cycles over ", latency_events,
          " events (", s.missed, " missed); the three slowest ", latency[latency_events - 3u],
          " ", latency[latency_events - 2u], " ", hi, crlf);
    print(serial, "  events over ", latency_count, " cycles: ", s.slow,
          ", a kernel tick or an XIP refill inside ", s.explained, " of them", crlf);
    bench.verdict("every interrupt's post started High within 1 ms", s.missed == 0u);
    bench.verdict("every event over the path's warm count had a tick or a cache refill in it",
                  s.slow == s.explained);
    bench.verdict("the latency stays under the path's bound (the arithmetic beside it)",
                  hi <= latency_bound);
    reset_scenario();
}

// =============================================================================
// d - the ruler, the scan and the post
// =============================================================================
volatile uint32_t post_quiet = 0;
volatile uint32_t post_road2 = 0;

void td_scan() {
    reset_scenario();
    tracing = false;
    console_drain();
    const uint32_t ruler = ruler_cost();
    // The ruler against the system timer over about a millisecond,
    // crossing a reload: the cycles' delta against the microseconds'
    // times 125, each pair of reads taken back to back in the same order
    // under the platform's own mask, so that no handler lands between a
    // pair's two reads (both counters run through the mask, and cycles()
    // reads a reload the handler has not counted yet off the pending bit;
    // measured unmasked: a tick's handler between the reads put 282 to
    // 607 cycles on one side in three runs of ten). The first pair after
    // the console's drain runs from a cold XIP cache, and the refills of
    // the pair's own code land between its two reads (measured: the first
    // span 73 to 513 cycles off, the three after it 6 to 30), so one
    // unmeasured pair warms the path first, as letter c warms its own.
    {
        P::CriticalSection cs;
        (void)Ruler::now();
        (void)Timer::now_low();
    }
    // A refill that lands inside one pair and not the other skews that
    // pair's two reads by the line's cost (measured: spans with a refill
    // in a pair 89 to 326 cycles off, spans without 4 to 35), so each
    // pair's misses are read as letter c reads them, and a span one
    // lengthened is reported and left out of the bound. Spans are taken
    // until four are clean or sixteen have been taken: with four spans
    // alone, a refill was flagged in every one of the four in 1 run of 20
    // (the first span is always flagged, its second pair's code cold; of
    // the three after it 12 in 60 were flagged, the differences of the
    // flagged spans of that run 34 to 69 cycles), and that run had no
    // span left to judge. The lengths step through 1, 1.001, 1.002 and
    // 1.003 ms as before.
    constexpr uint8_t spans_wanted = 4;
    constexpr uint8_t spans_most = 16;
    uint32_t agree_hi = 0;        ///< the worst span whose pairs missed no line
    uint8_t spans_clean = 0;
    uint8_t spans_taken = 0;
    uint32_t diffs[spans_most];
    bool clean[spans_most];
    for (uint8_t i = 0; i < spans_most && spans_clean < spans_wanted; ++i) {
        uint32_t c0, u0, c1, u1;
        const uint32_t m0 = xip_misses();
        {
            P::CriticalSection cs;
            c0 = Ruler::now();
            u0 = Timer::now_low();
        }
        const uint32_t m1 = xip_misses();
        spin(one_ms + 1000u * static_cast<uint32_t>(i % 4u));
        const uint32_t m2 = xip_misses();
        {
            P::CriticalSection cs;
            c1 = Ruler::now();
            u1 = Timer::now_low();
        }
        const uint32_t m3 = xip_misses();
        const uint32_t dc = c1 - c0;
        const uint32_t du = (u1 - u0) * cycles_per_us;
        diffs[i] = dc > du ? dc - du : du - dc;
        clean[i] = m1 == m0 && m3 == m2;
        ++spans_taken;
        if (clean[i]) {
            ++spans_clean;
            if (diffs[i] > agree_hi) {
                agree_hi = diffs[i];
            }
        }
    }
    print(serial, "  the spans' ruler-against-timer differences:");
    for (uint8_t i = 0; i < spans_taken; ++i) {
        print(serial, " ", diffs[i], clean[i] ? "" : "*");
    }
    print(serial, " cycles (* a cache refill inside a pair of reads); ", spans_taken,
          " spans taken, ", spans_clean, " clean", crlf);
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
    print(serial, "  the ruler's two reads (core 0's composed cycles()) ", ruler, " cycles", crlf);
    print(serial, "  over 1..1.003 ms the ruler and the system timer differ by at most ", agree_hi,
          " cycles on the clean spans", crlf);
    print(serial, "  activate() with nothing ready below the Console: ", scan, "..", scan_hi,
          " cycles (four empty queues, the fifth at the ceiling)", crlf);
    print(serial, "  from Mid: a post to Low (asks nothing) ", quiet,
          " cycles; a post to High (road 2, High's dispatch inside) ", road2,
          " cycles; the scheduling's share ", road2 > quiet ? road2 - quiet : 0u, crlf);
    // The ruler's two reads by the release listing: the first call from
    // its sample of VAL to its return (29), the move that keeps the first
    // value and the second call (4), the second call to its sample (22,
    // letter c's comment): 55 warm, 66 with a fifth. The STM32G0B1RE
    // measured 68 for the same two reads from a flash at two wait states.
    bench.verdict("the ruler's two reads cost under 66 cycles (the listing's 55 and a fifth)",
                  ruler < 66u);
    // The ruler and the timer, both off the crystal: clk_sys is the PLL's
    // exact 125 MHz (docs/rp2040/clock.md) and the timer's tick clk_ref
    // over 12 - so over the span the timer's delta in cycles differs from
    // the true span between its two reads by less than one microsecond
    // (125 cycles, two floors), and that span from the ruler's by the
    // difference of the two skews from a ruler's sample to the timer's
    // read: the listing's two pairs differ by a literal and a move (3),
    // and nothing lands between a pair's reads under the mask (a retry
    // of cycles() comes before its sample and moves nothing). 128, 154
    // with a fifth.
    bench.verdict("the ruler agrees with the system timer within 154 cycles over every span "
                  "whose pairs of reads missed no cache line (at least one such span)",
                  spans_clean != 0u && agree_hi <= 154u);
    // The bounds below are the release listing's count with every fetch
    // a hit of the XIP cache (letter c's comment), the least of 16 taken
    // so the cache is warm, and a fifth of the whole as the margin; the
    // ruler's two-read cost taken off holds one move the measured span
    // does not, so each count below is the span's less one.
    // The scan: activate()'s frame and the ceiling's test (20); Top's
    // visit, the queue's two cursors loaded and compared in line (8),
    // and the compare with the ceiling (5); High's (8 and 4), Mid's (9
    // and 4) and Low's (9, then the last compare with the ceiling and
    // the branch out, 6); the frame's release and the return (11): 84;
    // the call and the result kept (4): 88; 106 with the fifth.
    bench.verdict("the scan of four empty queues, the least of 16, costs under 106 cycles",
                  scan < 106u);
    // The quiet post: the event's setup and the call between the two
    // reads (8), post<Low>()'s prefix (a push, a literal, two moves, the
    // call: 10), the queue's push() - the core check, an IOPORT load of
    // CPUID, then the copy under the guard's mrs, cpsid and msr (48) -
    // the ceiling's compare and the return (16): 82, less one: 81; 98
    // with the fifth. Its path asks no queue whether it is empty.
    bench.verdict("a post that asks nothing costs under 98 cycles (the push and one compare)",
                  quiet < 98u);
    // The road-2 post: the setup and the call (9); post<High>()'s prefix
    // (10) and push() (48); the ceiling's compare and the call of the
    // request (13); the request - the core's test, the store, the DSB
    // and the ISB (17, Rp2040RubatoPlatform::pend_schedule()); PendSV's
    // entry from thread mode (15); the pair body and the return into
    // thread mode (34); the entry (4); activate() at Mid's ceiling to
    // High's dispatch (42, letter c's); High's dispatch with nothing to
    // do (191: the two raises and take(), 38, as in letter c; the slot
    // kept and the ruler's whole call, 8 and 51; the stamp's store, the
    // timer's low word over the APB bridge, whose read adds two cycles
    // to the core's load (2.1.3), its store, the count and the stack
    // sample, 32; the tests, 19; the call of release() and release(),
    // 23; the ceiling and the index restored and the branch to the
    // second pass, 20); the second pass, which compares Top's and High's
    // cursors in line and finds both empty, and the return (49); the
    // SVC's entry, its three instructions and its return (31); the
    // request's and the post's returns (7): 470, less one: 469; 563 with
    // the fifth.
    bench.verdict("a road-2 post costs more than a quiet one (the receiver ran inside it)",
                  road2 > quiet);
    bench.verdict("and under 563 cycles, High's empty dispatch included", road2 < 563u);
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
                edge_in(16u);
                spin(20000u);
                note('H', 'r');
            };
            const uint32_t r = High::runs;
            if (road == 2) {
                post<High>(Hit{1});
            } else {
                edge_to = 1;     // the first edge starts High over the Console
                edge_in(4u);
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
            if (Trace0::is(want)) {
                ++alike;
            } else if (alike == run) {
                Trace0::print_all();
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

/// Wait until core 0's kernel tick has moved `ticks` on.
void wait_ticks(uint32_t ticks) {
    const uint32_t t0 = CoreTicker<0>::ticks();
    while (CoreTicker<0>::ticks() - t0 < ticks) {
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
    const bool walk_quiet = Trace0::len == 0u && !High::queue.empty() && !Mid::queue.empty() &&
                            !Low::queue.empty();
    (void)K::activate();
    const bool in_order = Trace0::is("Hs1 He1 Ms2 Me2 Ls3 Le3");
    if (!in_order) {
        Trace0::print_all();
    }
    const bool part1 = fired[21] == 1u && fired[31] == 1u && fired[41] == 1u;

    // 2. Low's handler arms and disarms in a loop, most of each turn inside
    //    arm()'s or disarm()'s masked window; alarm 0's interrupt lands
    //    there and its High preempts at the window's end, arming two
    //    timers and disarming the one Low keeps re-arming.
    Trace0::clear();
    low_iteration = 0;
    high_landed = 0;
    Low::action = [](uint8_t n) {
        if (n != 60u) {
            ++fired[n];
            return;
        }
        note('L', 'm');
        edge_to = 1;
        edge_in(24u);            // inside the loop's first tenth: 200 turns of three list verbs
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
    const bool preempted = Trace0::is("Ls3 Lm3 Hs1 He1 Lr3 Le3");
    if (!preempted) {
        Trace0::print_all();
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
    // listing: the arguments and the call (9); the frame, the guard's mrs
    // and cpsid and the call of the out-of-line unlink (14); the unlink -
    // the head's load and test, the two stores of the unlink and the
    // node's two, the tail's test, the return (29); now()'s volatile
    // load, the deadline, the node's stores, the link at the list's tail,
    // the msr and the return (38): 90, less one (letter d's comment): 89
    // warm, the masked window the 63 from the cpsid to the msr's end; 107
    // with a fifth.
    bench.verdict("one arm(), the least of 16, costs under 107 cycles, its masked window inside",
                  arm_cycles < 107u);
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
    uint8_t slow = 0;        ///< runs over the path's warm count
    uint8_t explained = 0;   ///< of them, with a kernel tick or an XIP refill inside
    uint32_t most_clean = 0; ///< the longest run nothing lengthened
    bool back = true;
    for (int run = 0; run < 10; ++run) {
        reset_scenario();
        // A tick that matures inside the section runs its handler at the
        // unmask, before PendSV (SysTick outranks it), and an XIP refill
        // on the path adds its lines: both are read as letter c reads them,
        // and a run they lengthen is explained, not judged against the
        // warm bound.
        const uint32_t t0 = CoreTicker<0>::ticks();
        const uint32_t m0 = xip_misses();
        post_in_section<Mid>('C');
        {
            const uint32_t d = Mid::entered - section_last;
            least = d < least ? d : least;
            most = d > most ? d : most;
            const bool lengthened = CoreTicker<0>::ticks() != t0 || xip_misses() != m0;
            if (!lengthened) {
                most_clean = d > most_clean ? d : most_clean;
            }
            if (d > 214u) {
                ++slow;
                if (lengthened) {
                    ++explained;
                }
            }
        }
        if (Trace0::is(want_console)) {
            ++alike_console;
        } else if (alike_console == run) {
            Trace0::print_all();
        }
        reset_scenario();
        Low::action = [](uint8_t) { post_in_section<High>('L'); };
        post<Low>(Hit{1});
        if (Trace0::is(want_low)) {
            ++alike_low;
        } else if (alike_low == run) {
            Trace0::print_all();
        }
        back = back && P::ceiling == console_index && P::interrupts_enabled();
    }
    reset_scenario();
    print(serial, "  from the Console to Mid: ", alike_console, " of 10 runs traced ",
          want_console, crlf);
    print(serial, "  from Low to High: ", alike_low, " of 10 runs traced ", want_low, crlf);
    print(serial, "  the section's last statement to Mid's first: ", least, "..", most,
          " cycles; runs over 214: ", slow, ", a kernel tick or an XIP refill inside ",
          explained, crlf);
    bench.verdict("PendSV stood pending inside every section, and no receiver ran inside one",
                  pend_inside && !ran_inside);
    bench.verdict("the Console's post to Mid under PRIMASK is served when the section ends, "
                  "before the statement after it (10 runs)",
                  alike_console == 10u);
    bench.verdict("Low's post to High under PRIMASK likewise, inside Low's dispatch (10 runs)",
                  alike_low == 10u);
    // From the section's stamp's sample to Mid's, by the release listing:
    // the rest of the ruler's call (29), the stamp's store with its
    // literal and the saved mask's load (6) and the guard's msr that
    // lifts it (3); PendSV's entry from thread mode (15), the pair body
    // and its return into thread mode (34); the entry (4); activate() at
    // the Console's ceiling - its frame and test (20), Top's visit (13),
    // High's (12), Mid's to its dispatch (10), each queue's cursors
    // compared in line; Mid's dispatch to its stamp's sample (68, High's
    // in letter c): 214 warm; 257 with a fifth. A run a tick's handler or a refill lengthened is explained
    // by the counters read around it (measured: one run of ten at 604
    // cycles, a tick inside), so the bound judges the runs nothing
    // lengthened, and every lengthened one must be explained.
    bench.verdict("the section's end to the receiver's first statement under 257 cycles "
                  "in every run nothing lengthened",
                  most_clean < 257u);
    bench.verdict("every run over the path's warm count had a tick or a cache refill in it",
                  slow == explained);
    bench.verdict("the ceiling is the Console's again, interrupts enabled", back);
}

// =============================================================================
// j - road 1 across the cores
// =============================================================================
void tj_across() {
    reset_scenario();
    const bool up = core1_launch(&core1_tenuto, 1u);
    print(serial, "  core 1 launched into its Tenuto program: ", up ? "up" : "FAILED", crlf);
    bench.verdict("core 1 came up under Tenuto (its entry reported within 10 ms)", up);
    if (!up) {
        (void)core1_park();
        return;
    }
    // The mispost under Rubato: core 1 posts to High instead of sending.
    // The queue refuses and counts it; the request after the push - the
    // receiver's index is below core 0's ceiling - is not made from core
    // 1 (Rp2040RubatoPlatform::pend_schedule()), so no PendSV stands
    // pending on core 1 inside its masked section, and High never runs.
    {
        const uint16_t m0 = High::queue.misposts();
        const uint32_t h0 = High::runs;
        const uint32_t d0 = Relay::misposted;
        send<Relay>(Fire{1, true});
        const bool done = wait_runs(Relay::misposted, d0, one_ms);
        print(serial, "  a post from core 1 to High: misposts ", m0, " -> ", High::queue.misposts(),
              "; core 1's PendSV ", Relay::pended ? "PENDED" : "not pended", crlf);
        bench.verdict("a post from core 1 to core 0's object is refused and counted, pends no PendSV "
                      "on core 1, and the object never sees it",
                      done && High::queue.misposts() == m0 + 1u && !Relay::pended &&
                          High::runs == h0);
    }
    // The trace: Low sends to Relay and spins 62500 cycles (500 us) at a
    // mark; core 1 answers with a send to High, and core 0's doorbell
    // handler, landing in the spin, writes its own exit entry 'D' with
    // the ceiling it found (Low's, 3) before High starts. A crossing and
    // its return is 10 us (docs/rp2040/multicore.md), the first of a cold
    // program 163 us: the first run is the warm-up, not counted.
    const char* const want = "Ls3 Lm3 Dx3 Hs1 He1 Lr3 Le3";
    uint8_t alike = 0;
    bool once = true;
    uint32_t cross_lo = 0xFFFF'FFFFu;
    uint32_t cross_hi = 0;
    static volatile uint32_t sent_us = 0;
    for (int run = 0; run <= 10; ++run) {
        reset_scenario();
        Low::action = [](uint8_t) {
            note('L', 'm');
            sent_us = Timer::now_low();
            send<Relay>(Fire{1, false});
            spin(62500u);
            note('L', 'r');
        };
        const uint32_t h0 = High::runs;
        post<Low>(Hit{1});
        if (run == 0) {
            continue;
        }
        const uint32_t cross = High::entered_us - sent_us;
        cross_lo = cross < cross_lo ? cross : cross_lo;
        cross_hi = cross > cross_hi ? cross : cross_hi;
        if (Trace0::is(want)) {
            ++alike;
        } else if (alike + 1 == run) {
            Trace0::print_all();
        }
        once = once && High::runs == h0 + 1u;
    }
    reset_scenario();
    print(serial, "  ", alike, " of 10 runs traced ", want, "; Low's send to High's first statement ",
          cross_lo, "..", cross_hi, " us across core 1 and back", crlf);
    bench.verdict("ROAD 1 ACROSS THE CORES: High's dispatch starts after the doorbell handler's "
                  "exit and before Low resumes, every ceiling its own index (10 runs)",
                  alike == 10u);
    bench.verdict("High was dispatched once per crossing", once);

    // The latency, over the Console's wait: the doorbell handler's last
    // statement to High's first. The path is letter c's with the doorbell
    // handler's tail in place of the alarm's, and the tails are the same
    // by the listing - the rest of cycles() (29), the stamp's store with
    // its literal (4), a pop of four into the PC (7) - so the warm count
    // is letter c's 199. One thing more runs beside it: core 1, which
    // after its ring returns from its send and its dispatch and turns its
    // loop to the WFI, fetching from the same XIP cache, whose port
    // serves one master a cycle; so each of the path's fetches from the
    // cache - by the listing 58 words of instructions, 8 literals, the
    // pair body's table entry and PendSV's vector - may wait one cycle
    // behind core 1's: 68 more. An event is slow over 267, and the bound
    // is letter c's with those 68: 451.
    constexpr uint32_t bell_count = latency_count + 68u;
    constexpr uint32_t bell_bound = latency_bound + 68u;
    tracing = false;
    console_drain();
    const Slow s = latency_run([] { send<Relay>(Fire{0, false}); }, [] { return bell_last; },
                               bell_count);
    sort(latency, latency_events);
    const uint32_t lo = latency[0];
    const uint32_t med = latency[latency_events / 2u];
    const uint32_t hi = latency[latency_events - 1u];
    print(serial, "  road 1 from core 0's doorbell: its handler's last statement to High's first, "
          "min ", lo, " median ", med, " max ", hi, " cycles over ", latency_events, " crossings (",
          s.missed, " missed); over ", bell_count, ": ", s.slow,
          ", a kernel tick or an XIP refill inside ", s.explained, crlf);
    bench.verdict("every crossing started High within 1 ms", s.missed == 0u);
    bench.verdict("every crossing over the path's warm count and core 1's share of the cache had "
                  "a tick or a cache refill in it",
                  s.slow == s.explained);
    bench.verdict("the doorbell's latency stays under the path's bound (the arithmetic beside it)",
                  hi <= bell_bound);
    reset_scenario();
    const bool parked = core1_park();
    bench.verdict("core 1 back in the bootrom", parked);
}

// =============================================================================
// k - a Rubato on each core
// =============================================================================
/// Core 0's and core 1's windows on the system timer: the preempted
/// dispatch's mark and its resumption, and the preemptor's start.
volatile uint32_t w0_mark = 0;
volatile uint32_t w0_resumed = 0;
volatile uint32_t w1_mark = 0;
volatile uint32_t w1_resumed = 0;
volatile uint32_t k1_done = 0;          ///< Low1's actions ended, counted by core 1

/// Core 1's ceiling read from core 0: through a volatile access, since
/// the other core writes it and a plain read in a loop could be hoisted.
uint8_t core1_ceiling() { return *static_cast<volatile uint8_t*>(&P1::ceiling); }

void tk_both_cores() {
    reset_scenario();
    sp1_low_dispatch = 0xFFFF'FFFFu;
    sp1_low_isr = 0xFFFF'FFFFu;
    const bool up = core1_launch(&core1_rubato, 2u);
    print(serial, "  core 1 launched into its Rubato pack: ", up ? "up" : "FAILED", crlf);
    bench.verdict("core 1 came up under Rubato (its entry reported within 10 ms)", up);
    if (!up) {
        (void)core1_park();
        return;
    }
    // Core 1's side: Low1, started by core 0's send (road 1 from core 1's
    // doorbell at its base), arms alarm 1 100 us ahead and spins 62500 of
    // its cycles (500 us); the alarm posts to High1, which posts to Top1
    // inside its dispatch (road 2).
    Low1::action = [](uint8_t) {
        Trace1::note('l', 'm');
        w1_mark = Timer::now_low();
        Timer::alarm_in(edge_alarm1, 100u);
        const uint32_t t0 = Ruler1::now();
        while (Ruler1::now() - t0 < 62500u) {
        }
        w1_resumed = Timer::now_low();
        Trace1::note('l', 'r');
        k1_done = k1_done + 1u;
    };
    High1::action = [](uint8_t) { post<Top1>(Hit{1}); };
    // Core 0's side: the same shape on alarm 0.
    const char* const want0 = "Ls3 Lm3 Hs1 Ts0 Te0 He1 Lr3 Le3";
    const char* const want1 = "ls2 lm2 hs1 ts0 te0 he1 lr2 le2";
    uint8_t alike0 = 0;
    uint8_t alike1 = 0;
    uint8_t overlapped = 0;
    bool settled = true;
    // Run 0 is the warm-up, not judged: core 1's first crossing of a cold
    // program pays the XIP cache's fill on both cores (163 us once,
    // docs/rp2040/multicore.md), longer than the 100 us before core 0's
    // alarm, which would put core 0's preemption before core 1's mark.
    for (int run = 0; run <= 10; ++run) {
        reset_scenario();
        Trace1::clear();     // core 1 is idle at its base: nothing writes it now
        Low::action = [](uint8_t) {
            note('L', 'm');
            w0_mark = Timer::now_low();
            send<Low1>(Hit{1});
            edge_to = 1;
            edge_in(100u);
            spin(62500u);
            w0_resumed = Timer::now_low();
            note('L', 'r');
        };
        High::action = [](uint8_t) { post<Top>(Hit{1}); };
        const uint32_t d0 = k1_done;
        post<Low>(Hit{1});
        // Both settle: core 1's Low1 ended its action (within 2 ms of the
        // post's return, its spin being 500 us), then its dispatch -
        // the trace's last entry written before the ceiling is restored
        // to 255, its base (kernel/rubato.hpp's visit()), within 1 ms.
        const bool done = wait_runs(k1_done, d0, 2u * one_ms);
        const uint32_t t0 = Ruler::now();
        while (core1_ceiling() != 255u && Ruler::now() - t0 < one_ms) {
        }
        settled = settled && done && core1_ceiling() == 255u && P::ceiling == console_index;
        if (run == 0) {
            continue;
        }
        if (Trace0::is(want0)) {
            ++alike0;
        } else if (alike0 + 1 == run) {
            Trace0::print_all();
        }
        if (Trace1::is(want1)) {
            ++alike1;
        } else if (alike1 + 1 == run) {
            Trace1::print_all();
        }
        // Each core's preemption inside the other core's preempted
        // dispatch, on the system timer: core 0's High started between
        // core 1's mark and its resumption, and core 1's High1 between
        // core 0's.
        const uint32_t h0 = High::entered_us;
        const uint32_t h1 = High1::entered_us;
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
          " of 10 runs; core 1's ceiling read after each settle ", P1::ceiling, crlf);
    bench.verdict("core 0 under Rubato beside core 1's: road 1 then road 2, every ceiling its own "
                  "index (10 runs)",
                  alike0 == 10u);
    bench.verdict("core 1 under Rubato through the same PendSV entry: road 1 from its doorbell, "
                  "road 1 from its alarm, road 2, every ceiling its own index (10 runs)",
                  alike1 == 10u);
    bench.verdict("the two cores preempted at once, each inside the other's preempted dispatch "
                  "(10 runs)",
                  overlapped == 10u);
    bench.verdict("both settle: core 1's ceiling 255 at its base, core 0's the Console's", settled);

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
          ", the interrupt at ", deepest_isr, crlf);
    // A dispatch's sample on core 1 is taken inside its activate(),
    // whose five saved registers and 20-byte frame (40 bytes) are above
    // it. Below the
    // deepest one lies at most, by the release listing: the dispatch's
    // action and what it calls (Low1's, 16, calling cycles() or the
    // trace's note(), 20: 36; High1's, 16, with a push, 8: 24); an
    // interrupt landing there, a basic frame and its alignment word (36);
    // and below that either the deepest handler's own frames (the
    // doorbell's 40 with a push, 8: 48; the alarm's 16 with a push: 24)
    // or, the handler gone and its post pending PendSV, PendSV's two
    // words and an activate() frame that finds nothing (8 + 40 = 48; the
    // fake frame's 32 below the two words are popped before activate()
    // pushes, 48 too). 36 + 36 + 48 = 120: core 1's high-water mark lies
    // within 120 bytes of the deeper of its two samples.
    const uint32_t deepest = deepest_dispatch > deepest_isr ? deepest_dispatch : deepest_isr;
    bench.verdict("core 1's paint reached its deepest sampled stack pointer", used >= deepest);
    bench.verdict("core 1's high-water mark is within 120 bytes of its deepest sample",
                  used <= deepest + 120u);
    Timer::interrupt(edge_alarm1, false);
    Timer::disarm(edge_alarm1);
    Timer::clear(edge_alarm1);
    const bool parked = core1_park();
    bench.verdict("core 1 back in the bootrom", parked);
}

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
    bench.verdict("no alarm is armed, alarm 0's line is quiet (nothing raised, nothing pending)",
                  !Timer::armed(edge_alarm) && !Timer::armed(edge_alarm1) &&
                      !Timer::raised(edge_alarm) && !Nvic::pending(Timer::irq(edge_alarm)));
    bench.verdict("the LED's pad GP25 is not an output", !Pin<25>::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// h - the stack
// =============================================================================
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
          ", an interrupt at ", deepest_isr, crlf);
    // A dispatch's sample is taken inside activate(), whose five saved
    // registers and frame (56 bytes) are above it. Below the deepest one
    // lies at most, by the release listing: the letter's action and what
    // it calls (the deepest, letter d's, a 32-byte frame calling
    // ruler_cost(), 16, which calls cycles(), 20: 68); an interrupt
    // landing there, a basic frame of 8 words and its alignment word (36:
    // this core has no FPU words); and below that either the deepest
    // handler's own frames (the alarm's 16 and the doorbell's 16, each
    // calling cycles(), 20, or the trace's note(), 20: 36; the UART's 32,
    // its post inline) or, the handler gone and its post pending PendSV,
    // PendSV's two words and an activate() frame that finds nothing to
    // serve (8 + 56 = 64; the fake frame's 32 are popped before
    // activate() pushes). 68 + 36 + 64 = 168: the high-water mark lies
    // within 168 bytes of the deeper of the two samples.
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
bool timer_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=",
          timer_ok ? "1us" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " ruler=cycles()", crlf);
    print(serial, "test_rp2040_rubato - the preemptive kernel on the RP2040's Cortex-M0+, the "
          "scheduler in thread mode, one PendSV entry for both cores", crlf);
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
extern "C" void isr_uart0() {
    if (Serial::isr()) {
        rx_ceiling = P::ceiling;
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

/// One SysTick vector, two cores: each ticks its own ticker.
extern "C" void isr_systick() {
    if (P::core_id() == 0u) {
        brio::CoreTicker<0>::tick();
    } else {
        brio::CoreTicker<1>::tick();
    }
}

/// One PendSV entry, two kernels (rp2040/schedule.hpp), and the SVC
/// body both cores leave the scheduler through.
extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception_pair<K, K1>(); }
extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

/// Alarm 0, core 0's: the flag cleared, the post, and the ruler as the
/// last statement - letter c's near end.
extern "C" void isr_timer_0() {
    brio::Timer::clear(edge_alarm);
    sample_sp(sp_low_isr);
    edge_hits = edge_hits + 1u;
    if (edge_to == 0u) {
        brio::post<Top>(Hit{1});
    } else {
        brio::post<High>(Hit{1});
    }
    edge_last = Ruler::now();
}

/// Alarm 1, core 1's in letter k: the post to High1.
extern "C" void isr_timer_1() {
    brio::Timer::clear(edge_alarm1);
    sample_sp(sp1_low_isr);
    brio::post<High1>(Hit{1});
}

/// Core 0's doorbell: the crossing posted to High, the handler's own exit
/// entry, and the ruler as the last statement - letter j's near end.
extern "C" void isr_sio_proc0() {
    sample_sp(sp_low_isr);
    Drain0::isr();
    bell_hits = bell_hits + 1u;
    if (tracing) {
        note('D', 'x');
    }
    bell_last = Ruler::now();
}

/// Core 1's doorbell, for whichever program core 1 runs.
extern "C" void isr_sio_proc1() {
    if (core1_stage == 2u) {
        sample_sp(sp1_low_isr);
        Drain1R::isr();
    } else {
        Drain1T::isr();
    }
}

int main() {
    // Paint core 0's free stack: from 64 bytes below main's own stack
    // pointer down to 64 bytes above the floor of its bank, the upper
    // unstriped one, whose floor is core 1's stack top (rp2040/ld/).
    {
        const uint32_t sp = __get_MSP();
        paint_bottom = Core1::default_stack_top() + 64u;
        paint_top = (sp - 64u) & ~3u;
        for (uint32_t a = paint_bottom; a < paint_top; a += 4u) {
            *reinterpret_cast<volatile uint32_t*>(a) = paint;
        }
    }

    clock_ok = SysClock::init();
    timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::CoreTicker<0>::init(clock);
    brio::Timer::clear(edge_alarm);
    brio::Timer::interrupt(edge_alarm, true);
    brio::Nvic::clear_pending(brio::Timer::irq(edge_alarm));
    brio::Nvic::enable(brio::Timer::irq(edge_alarm));   // core 0's NVIC alone
    Drain0::enable();
    brio::scheduler_init();

    bench.letter('a', "the ceiling, road 1 at depth 1", xip_cleared<ta_ceiling>);
    bench.letter('b', "road 2: the chain inside the posts", xip_cleared<tb_road2>);
    bench.letter('c', "the latency of road 1", xip_cleared<tc_latency>);
    bench.letter('d', "the ruler, the scan and the post, in cycles", xip_cleared<td_scan>);
    bench.letter('e', "depth 2: an interrupt over a preempting dispatch", xip_cleared<te_depth2>);
    bench.letter('f', "time events: the base turn's order, arm and disarm preempted", xip_cleared<tf_timers>);
    bench.letter('g', "the request under PRIMASK: served when the section ends", xip_cleared<tg_primask>);
    bench.letter('j', "road 1 across the cores: core 1's send preempts core 0's dispatch", xip_cleared<tj_across>);
    bench.letter('k', "a Rubato on each core, one PendSV entry, two ceilings", xip_cleared<tk_both_cores>);
    bench.letter('i', "dark: every object idle, core 1 parked, nothing lit", xip_cleared<ti_dark>);
    bench.letter('h', "the stack's high-water mark", xip_cleared<th_stack>);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
