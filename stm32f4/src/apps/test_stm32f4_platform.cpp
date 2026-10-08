// test_stm32f4_platform - the reference bench suite for the STM32F4's
// PLATFORM: the running half (the PRIMASK critical section, the idle
// hook, the SysTick timebase), stm32f4/delay.hpp (the microsecond
// busy-wait), the clock tree as the registers hold it against what the
// Clock task's constants say, and the panic breadcrumb without a reset.
// The failing half - the reset causes, the watchdogs, the breadcrumb
// across a real reset - is the reset chapter's suite to come.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The console is the board's ST-LINK virtual COM port
// (or the probe's bridge on the black pill, as console.cpp says) and
// every clock this suite measures is inside the chip. THERE IS NO
// INDEPENDENT RULER on this family yet (the RTC and the timers are
// chapters of their own), so the SysTick tick judges delay_us in bulk
// and delay_us is judged for its at-least contract against SysTick's
// own counter; the tick's own rate is judged by the reload's arithmetic
// and, on the desk, by the console's uptime against a clock.
//
// What is exercised, letter by letter:
//   a  the boot story: the device id and revision, the unique id, the
//      flash size, no breadcrumb pending
//   b  the critical section: PRIMASK saved and restored, nesting, and
//      the idle hook returning on the tick
//   c  the SysTick timebase: the reload, ticks/millis/secs/now
//      coherence, and the VAL-delta accumulation that delay.hpp is
//      built on checked against the interrupt count
//   d  delay_us: a thousand 100 us waits against the kernel tick, never
//      early on SysTick's counter, the cap, the no-counter refusal
//   e  the clock tree: SWS, the PLL's lock, the HSE's mode, the
//      regulator scale and over-drive, the flash latency and the
//      accelerator, the APB dividers - each register against the task's
//      constant
//   g  the panic breadcrumb WITHOUT a reset: written, taken once, gone
//   w  THE IDLE HOOK AGAINST AN INTERRUPT'S EDGE, PLACED TO THE CYCLE:
//      SysTick's next edge put D cycles after a write, D = 1..400 (LOAD
//      set to D around the VAL write that reloads it, then back), then
//      the kernel's own shape - a masked check, idle(), until one tick
//      (and, a second pass, two) - so the edge walks across every
//      instruction of the idle path and of the loop around it, the WFI's
//      entry included; a third pass places the edge of an NVIC LINE
//      (TIM9's compare on HCLK) the same way with the tick's interrupt
//      held off. TIM2 at 1 MHz is the clock of each try and its RESCUE:
//      a try that takes its 50 ms period is a wake idle() lost for good,
//      one that ends a whole tick late a wake lost until the next tick.
//      The placements shorten the ticks they land in, so kernel time
//      runs fast during the letter. Prints the cycles from an edge that
//      finds the core asleep to the handler and to the caller's loop
//      (the tick handler stamps VAL at its first and last statements),
//      and what an idle() costs when a pending line returns it at once.
//   k  THE KERNEL'S TURN: a Tenuto pack of three quiet AOs, two with a
//      periodic time event, turned as Tenuto::run() turns it over 100
//      ticks with the tick the only interrupt - one turn per tick - and
//      what one quiet turn costs when a pending line makes its idle()
//      return at once, against that idle() alone.
//   t  the runtime's seven functions (rt/rt.cpp) over every alignment,
//      length and overlap: rt/selftest.hpp's cases, the host suite's own,
//      against the symbols this image links - in `z`
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/panic.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "rt/selftest.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx) || defined(STM32F469xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

#if defined(STM32F429xx)
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F469xx)
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
#elif defined(STM32F411xE)
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

std::optional<PanicRecord> boot_record;

/// Let the console's own transmitter fall silent: its interrupt would
/// otherwise wake every idle() below.
void console_drain() {
    const uint32_t t0 = Ticker::ticks();
    while (!Serial::tx_idle() && Ticker::ticks() - t0 < 200u) {
    }
}

// =============================================================================
// a - the boot story
// =============================================================================
void ta_boot() {
    const DeviceIdcode id = DeviceIdcode::read();
    const DeviceUid uid = DeviceUid::read();
    print(serial, "  DEV_ID ", hex(id.dev_id), " REV_ID ", hex(id.rev_id), " flash ",
          flash_size_kbytes(), " KB uid ", hex(uid.word[0]), "-", hex(uid.word[1]), "-",
          hex(uid.word[2]), crlf);
#if defined(STM32F429xx)
    bench.verdict("DEV_ID 0x419: an STM32F42x/F43x", id.dev_id == 0x419u);
    bench.verdict("the flash size register says 2048 KB", flash_size_kbytes() == 2048u);
#elif defined(STM32F446xx)
    bench.verdict("DEV_ID 0x421: an STM32F446", id.dev_id == 0x421u);
    bench.verdict("the flash size register says 512 KB", flash_size_kbytes() == 512u);
#elif defined(STM32F411xE)
    bench.verdict("DEV_ID 0x431: an STM32F411", id.dev_id == 0x431u);
    bench.verdict("the flash size register says 512 KB", flash_size_kbytes() == 512u);
#elif defined(STM32F469xx)
    bench.verdict("DEV_ID 0x434: an STM32F469/F479", id.dev_id == 0x434u);
    bench.verdict("the flash size register says 2048 KB", flash_size_kbytes() == 2048u);
#endif
    bench.verdict("the unique id is not blank", (uid.word[0] | uid.word[1] | uid.word[2]) != 0u);
    bench.verdict("no breadcrumb is pending on a clean start", !boot_record);
}

// =============================================================================
// b - the critical section and the idle hook
// =============================================================================
void tb_critical() {
    bench.verdict("interrupts are enabled when a letter runs", P::interrupts_enabled());

    bool inside = true, nested = true, after_inner = false, after_outer = false;
    {
        P::CriticalSection cs;
        inside = P::interrupts_enabled();
        {
            P::CriticalSection inner;
            nested = P::interrupts_enabled();
        }
        after_inner = P::interrupts_enabled();
    }
    after_outer = P::interrupts_enabled();
    bench.verdict("a critical section masks", !inside);
    bench.verdict("a nested one is still masked", !nested);
    bench.verdict("leaving the INNER scope does not unmask (PRIMASK is saved "
                  "and restored, not cleared)",
                  !after_inner);
    bench.verdict("leaving the outer scope unmasks", after_outer);

    // The tick keeps counting through a masked window - the interrupt is
    // pending, not lost - but a masked window LOSES ticks: SysTick's
    // interrupt is a pending bit, so five periods under the mask deliver
    // one tick when the mask lifts.
    const uint32_t period = SysTick->LOAD + 1u;
    const uint32_t t0 = Ticker::ticks();
    uint32_t cycles = 0;
    {
        P::CriticalSection cs;
        uint32_t last = SysTick->VAL;
        while (cycles < 5u * period) {
            const uint32_t now = SysTick->VAL;
            cycles += (last >= now) ? (last - now) : (last + period - now);
            last = now;
        }
    }
    // The pending interrupt is taken AFTER the mask lifts, not inside the
    // instruction that lifts it: give the core the ISB the architecture
    // asks for before reading what the handler delivered (PM0214 2.1.3).
    __ISB();
    const uint32_t through_mask = Ticker::ticks() - t0;
    print(serial, "  five SysTick periods under the mask advanced the tick by ", through_mask,
          " ms", crlf);
    bench.verdict("a masked window LOSES ticks (one delivered, the rest coalesce)",
                  through_mask >= 1u && through_mask <= 2u);

    // idle(): WFI, then unmask. Any enabled interrupt wakes it, so the
    // console has to be silent first.
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        P::idle();
        ++calls;
    }
    print(serial, "  with the console silent, ", calls, " idle() call(s) reached the next tick",
          crlf);
    bench.verdict("idle() sleeps and the tick is what brings it back", calls >= 1u && calls < 20u);
    bench.verdict("and it comes back with interrupts enabled", P::interrupts_enabled());
}

// =============================================================================
// w - the idle hook against an interrupt's edge, placed to the cycle
// =============================================================================
/// The clock of each try and its RESCUE: 1 MHz, a 50 ms period.
using Rescue = Tim<2>;
/// The NVIC line whose edge is placed: TIM9 on APB2, whose timer clock is
/// HCLK on every bench board (APB2 at HCLK / 2 doubled, or undivided on
/// the F411 - letter w says so before it trusts it), so a compare lands
/// to the core's cycle. TIM9 is on every part of the family the suite
/// builds for.
using Edge = Tim<9>;

volatile uint32_t rescue_periods = 0;
volatile uint32_t edge_hits = 0;
volatile uint32_t edge_entry = 0;   ///< Edge's count at its handler's first statement
volatile uint32_t tick_entry = 0;   ///< SysTick VAL at the tick handler's first statement
volatile uint32_t tick_exit = 0;    ///< and at its last

/// Cycles since SysTick's 1 -> 0 edge, from a VAL read in the period that
/// edge's reload began: the reload lands one cycle after the edge.
uint32_t since_tick_edge(uint32_t val) { return SysTick->LOAD + 1u - val; }

/// SysTick's next edge `d` cycles from now (d >= 1): a write to VAL clears
/// it and the counter reloads from LOAD at the next clock, the exception
/// pending at its 1 -> 0 transition (ARMv7-M ARM B3.3.1, PM0214 4.5.2);
/// LOAD goes back at once so the reload AT the edge is the program's
/// period again. False
/// when the placement missed: the counter above d and no tick pending.
bool place_tick_edge(uint32_t d, uint32_t reload) {
    SysTick->LOAD = d;
    SysTick->VAL = 0;
    SysTick->LOAD = reload;
    const uint32_t v = SysTick->VAL;
    return v <= d || (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0u;
}

constexpr uint32_t walk_positions = 400;

struct EdgeWalk {
    uint32_t lost = 0;          ///< rescued (the rescue's period) or a tick late
    uint32_t first_lost = 0;    ///< the first position that was
    uint32_t misplaced = 0;     ///< tries whose edge was not where it was asked
    uint32_t slowest_us = 0;
    uint32_t asleep_to_loop = 0;     ///< edge -> the caller's loop, the core asleep (least)
    uint32_t asleep_to_loop_hi = 0;  ///< and most, over the far half of the walk
    uint32_t asleep_entry = 0;       ///< edge -> the handler's first statement
    uint32_t asleep_handler_end = 0; ///< edge -> the handler's last statement
    /// The nearest position whose edge still found the core asleep: the
    /// walk runs from D = 400 down, the far half's edge-to-loop figures
    /// (all of them asleep) giving the band, and the first try whose
    /// figure leaves the band by more than two cycles is an edge that
    /// came before the WFI.
    uint32_t asleep_from = 0;
};

/// Positions are walked from the far end down, so the first try is the
/// asleep reference the nearer ones are compared with - after one more
/// try beyond the far end, which warms the caches and the prefetch on
/// the handler's and the loop's code and is judged but not measured.
void note_position(EdgeWalk& w, uint32_t d, uint32_t to_loop) {
    if (d > walk_positions) {
        return;   // the warm-up try: its figures carry the caches' misses
    }
    if (d == walk_positions) {
        w.asleep_to_loop = to_loop;
        w.asleep_to_loop_hi = to_loop;
    }
    if (d > walk_positions / 2u) {
        if (to_loop < w.asleep_to_loop) {
            w.asleep_to_loop = to_loop;
        }
        if (to_loop > w.asleep_to_loop_hi) {
            w.asleep_to_loop_hi = to_loop;
        }
        w.asleep_from = d;
        return;
    }
    if (w.asleep_from == d + 1u && to_loop + 2u >= w.asleep_to_loop &&
        to_loop <= w.asleep_to_loop_hi + 2u) {
        w.asleep_from = d;
    }
}

/// The kernel's own shape against SysTick's edge: a masked check, idle(),
/// until `edges` ticks have been taken. The first edge walks across every
/// cycle from the placement to D = 400; with two, the second comes a
/// whole period later and finds the sleep entered after a tick was taken.
EdgeWalk walk_tick(uint32_t edges) {
    EdgeWalk w;
    const uint32_t reload = SysTick->LOAD;
    for (uint32_t d = walk_positions + 1u; d >= 1u; --d) {
        uint32_t to_loop = 0;
        bool rescued = false;
        bool placed = false;
        uint32_t took_us = 0;
        {
            P::CriticalSection cs;
            const uint32_t r0 = rescue_periods;
            Rescue::set_count(0);
            const uint32_t t = Ticker::ticks();
            placed = place_tick_edge(d, reload);
            for (;;) {
                P::CriticalSection turn;   // the kernel's own shape
                if (Ticker::ticks() - t >= edges) {
                    break;
                }
                P::idle();
            }
            to_loop = SysTick->VAL;
            took_us = Rescue::count();
            rescued = rescue_periods != r0;
        }
        if (!placed) {
            ++w.misplaced;
        }
        if (took_us > w.slowest_us) {
            w.slowest_us = took_us;
        }
        // A wake lost for good waits for the rescue; one lost until the
        // NEXT tick ends a whole period late. The edges asked for land
        // within 10 us of the period they need: the bound sits a quarter
        // period past that.
        if (rescued || took_us > edges * 1000u - 750u) {
            if (w.lost == 0u) {
                w.first_lost = d;
            }
            ++w.lost;
        }
        note_position(w, d, since_tick_edge(to_loop));
        if (d == walk_positions) {
            w.asleep_entry = since_tick_edge(tick_entry);
            w.asleep_handler_end = since_tick_edge(tick_exit);
        }
    }
    return w;
}

/// The same shape against an NVIC LINE's edge - TIM9's compare, D counts
/// of HCLK after its counter starts - with the tick's interrupt held off
/// (TICKINT clear) so that nothing but the line and the rescue can wake
/// the core: a lost wake here is lost for good.
EdgeWalk walk_line() {
    EdgeWalk w;
    SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_TICKINT_Msk;
    for (uint32_t d = walk_positions + 1u; d >= 1u; --d) {
        uint32_t to_loop = 0;
        bool rescued = false;
        uint32_t took_us = 0;
        {
            P::CriticalSection cs;
            const uint32_t r0 = rescue_periods;
            const uint32_t h0 = edge_hits;
            Edge::enable(false);
            Edge::set_count(0);
            Edge::clear_flags(Edge::compare_flag(0));
            (void)Edge::set_compare(0, d);
            Edge::interrupts(Edge::compare_interrupt(0), true);
            Rescue::set_count(0);
            Edge::enable(true);
            // A read of the block waits for the enable's store to land, so
            // the count starts HERE in the core's stream and not wherever
            // the write buffer delivers it - the DSB of idle() would
            // otherwise hold the core until it lands, and no edge would
            // ever come before the WFI.
            (void)Edge::count();
            for (;;) {
                P::CriticalSection turn;
                if (edge_hits != h0) {
                    break;
                }
                P::idle();
            }
            to_loop = Edge::count();
            took_us = Rescue::count();
            rescued = rescue_periods != r0;
        }
        if (took_us > w.slowest_us) {
            w.slowest_us = took_us;
        }
        if (rescued || took_us > 100u) {
            if (w.lost == 0u) {
                w.first_lost = d;
            }
            ++w.lost;
        }
        note_position(w, d, to_loop - d);
        if (d == walk_positions) {
            w.asleep_entry = edge_entry - d;
        }
    }
    Edge::enable(false);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_TICKINT_Msk;
    return w;
}

/// The same edges finding the core AWAKE - spinning unmasked on the count
/// the handler moves - for the entry the sleep is weighed against: the
/// least of 8 each.
struct AwakeEntry {
    uint32_t tick_entry = 0xFFFF'FFFFu;
    uint32_t tick_to_loop = 0xFFFF'FFFFu;
    uint32_t line_entry = 0xFFFF'FFFFu;
    uint32_t line_to_loop = 0xFFFF'FFFFu;
};

AwakeEntry awake_entry() {
    AwakeEntry a;
    const uint32_t reload = SysTick->LOAD;
    for (int i = 0; i < 8; ++i) {
        const uint32_t t = Ticker::ticks();
        (void)place_tick_edge(200u, reload);
        while (Ticker::ticks() == t) {
        }
        const uint32_t to_loop = since_tick_edge(SysTick->VAL);
        const uint32_t entry = since_tick_edge(tick_entry);
        if (entry < a.tick_entry) {
            a.tick_entry = entry;
        }
        if (to_loop < a.tick_to_loop) {
            a.tick_to_loop = to_loop;
        }
    }
    SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_TICKINT_Msk;
    for (int i = 0; i < 8; ++i) {
        const uint32_t h0 = edge_hits;
        Edge::enable(false);
        Edge::set_count(0);
        Edge::clear_flags(Edge::compare_flag(0));
        (void)Edge::set_compare(0, 200u);
        Edge::interrupts(Edge::compare_interrupt(0), true);
        Edge::enable(true);
        while (edge_hits == h0) {
        }
        const uint32_t to_loop = Edge::count() - 200u;
        const uint32_t entry = edge_entry - 200u;
        if (entry < a.line_entry) {
            a.line_entry = entry;
        }
        if (to_loop < a.line_to_loop) {
            a.line_to_loop = to_loop;
        }
    }
    Edge::enable(false);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_TICKINT_Msk;
    return a;
}

/// An idle() a pending line makes return at once - the whole cost, the
/// line's handler round trip and the two VAL reads included - the least
/// of 16.
uint32_t idle_at_once_cycles() {
    const uint32_t period = SysTick->LOAD + 1u;
    uint32_t best = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        P::CriticalSection cs;
        Nvic::set_pending(Edge::irq());
        const uint32_t v0 = SysTick->VAL;
        P::idle();
        const uint32_t v1 = SysTick->VAL;
        disable_interrupts();
        const uint32_t c = v0 >= v1 ? v0 - v1 : v0 + period - v1;
        if (c < best) {
            best = c;
        }
    }
    return best;
}

void rescue_up() {
    Rescue::init();
    (void)Rescue::configure(TimConfig{
        .prescaler = static_cast<uint16_t>(Rescue::clock_hz(clock) / 1'000'000u - 1u),
        .period = 49'999u});
    Rescue::clear_flags(Rescue::update_flag);
    Rescue::interrupts(Rescue::update_interrupt, true);
    Nvic::enable(Rescue::irq());
    Rescue::enable(true);
    Edge::init();
    (void)Edge::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
    Nvic::enable(Edge::irq());
}

void rescue_down() {
    Nvic::disable(Edge::irq());
    Edge::release();
    Rescue::enable(false);
    Rescue::interrupts(Rescue::update_interrupt, false);
    Nvic::disable(Rescue::irq());
    Rescue::release();
}

void tw_edge() {
    bench.verdict("TIM9 counts HCLK (the placed line's edge is in core cycles)",
                  Edge::clock_hz(clock) == SysClock::hz);
    rescue_up();
    console_drain();

    const EdgeWalk one = walk_tick(1);
    const EdgeWalk two = walk_tick(2);
    const EdgeWalk line = walk_line();
    const uint32_t at_once = idle_at_once_cycles();
    const AwakeEntry awake = awake_entry();
    rescue_down();

    print(serial, "  ", walk_positions, " edge positions: lost waiting for one tick ", one.lost,
          " (first at D=", one.first_lost, "), for two ", two.lost, " (first at D=", two.first_lost,
          "), for TIM9's line ", line.lost, " (first at D=", line.first_lost, ")", crlf);
    print(serial, "  misplaced tick edges ", one.misplaced + two.misplaced, "; the slowest try: ",
          one.slowest_us, " us for one tick, ", two.slowest_us, " us for two, ", line.slowest_us,
          " us for the line", crlf);
    print(serial, "  the edge found the core asleep from D=", one.asleep_from, " (the tick) and D=",
          line.asleep_from, " (the line) on; nearer, it came before the WFI", crlf);
    print(serial, "  a tick edge that finds the core asleep: handler entered at ", one.asleep_entry,
          ", its last statement at ", one.asleep_handler_end, ", the caller's loop at ",
          one.asleep_to_loop, "..", one.asleep_to_loop_hi, " cycles", crlf);
    print(serial, "  a line edge that finds the core asleep: handler entered at ", line.asleep_entry,
          ", the caller's loop at ", line.asleep_to_loop, "..",
          line.asleep_to_loop_hi, " cycles", crlf);
    print(serial, "  the same edges finding the core awake (spinning unmasked): the tick's handler "
          "entered at ", awake.tick_entry, ", the spin out at ", awake.tick_to_loop,
          "; the line's handler at ", awake.line_entry, ", the spin out at ", awake.line_to_loop,
          " cycles", crlf);
    print(serial, "  an idle() a pending line returns at once: ", at_once,
          " cycles (the line's handler and two VAL reads included)", crlf);
    bench.verdict("every tick edge was placed where it was asked", one.misplaced + two.misplaced == 0u);
    bench.verdict("and the walk crossed the sleep's entry: the nearest edges came before the WFI",
                  one.asleep_from > 1u && line.asleep_from > 1u);
    bench.verdict("no edge position loses the wake, waiting for one tick", one.lost == 0u);
    bench.verdict("nor waiting for two (the sleep after a tick was taken)", two.lost == 0u);
    bench.verdict("nor on an NVIC line, the tick held off", line.lost == 0u);
}

// =============================================================================
// k - the kernel's turn: how many per interrupt, and what one costs
// =============================================================================
// A pack shaped like a small program's: three AOs, two of them with a
// periodic time event (500 ms and 1000 ms, so none fires in the 100 ms
// this letter idles), none of them ever posted to.
template <uint8_t N>
struct Quiet {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Quiet, Event> beat{Event{N}};
    static void init() {
        if constexpr (N != 0u) {
            beat.arm_every(ticks_from_ms<P>(500u * N));
        }
    }
    static void dispatch(const Event&) {}
};
using QuietKernel = Tenuto<P, Quiet<1>, Quiet<2>, Quiet<0>>;

/// One turn of Tenuto::run()'s loop, as it is written there.
[[gnu::always_inline]] inline void kernel_turn() {
    TimeEvents<P>::process();
    if (!QuietKernel::step()) {
        QuietKernel::idle_if_empty();
    }
}

void tk_turns() {
    Edge::init();
    Nvic::enable(Edge::irq());
    console_drain();
    QuietKernel::init_all();

    // Turns over 100 quiet ticks: the tick is the only interrupt.
    const uint32_t t = Ticker::ticks();
    uint32_t turns = 0;
    while (Ticker::ticks() - t < 100u) {
        kernel_turn();
        ++turns;
    }

    // A quiet turn whose idle() a pending line returns at once, against
    // that idle() alone: the difference is the turn's own work.
    const uint32_t period = SysTick->LOAD + 1u;
    uint32_t turn_cycles = 0xFFFF'FFFFu;
    for (int i = 0; i < 16; ++i) {
        disable_interrupts();
        Nvic::set_pending(Edge::irq());
        const uint32_t c0 = SysTick->VAL;
        kernel_turn();   // its idle() wakes at once and takes the line
        const uint32_t c1 = SysTick->VAL;
        enable_interrupts();
        const uint32_t c = c0 >= c1 ? c0 - c1 : c0 + period - c1;
        if (c < turn_cycles) {
            turn_cycles = c;
        }
    }
    const uint32_t at_once = idle_at_once_cycles();
    TimeEvents<P>::clear_all();
    Nvic::disable(Edge::irq());
    Edge::release();

    print(serial, "  100 quiet ticks: ", turns, " kernel turns", crlf);
    print(serial, "  a quiet turn whose idle() a pending line returns at once: ", turn_cycles,
          " cycles; that idle() alone ", at_once, "; the turn's own work ",
          turn_cycles > at_once ? turn_cycles - at_once : 0u, " cycles", crlf);
    bench.verdict("the kernel loop turns once per interrupt (100 ticks, 100 turns, one more "
                  "allowed at each end of the window)",
                  turns >= 99u && turns <= 102u);
}

// =============================================================================
// c - the SysTick timebase
// =============================================================================
void tc_ticker() {
    const uint32_t reload = SysTick->LOAD;
    print(serial, "  SysTick CTRL=", hex(SysTick->CTRL), " LOAD=", reload, " (",
          SysClock::hz / Ticker::ticks_per_second - 1u, " expected)", crlf);
    bench.verdict("the reload is Clock::hz / tps - 1",
                  reload == SysClock::hz / Ticker::ticks_per_second - 1u);
    bench.verdict("the counter runs on the processor clock with its interrupt armed",
                  (SysTick->CTRL & (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                                    SysTick_CTRL_ENABLE_Msk)) ==
                      (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                       SysTick_CTRL_ENABLE_Msk));

    const uint32_t ticks = Ticker::ticks();
    const uint32_t ms = Ticker::millis();
    const uint32_t secs = Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    print(serial, "  ticks=", ticks, " millis=", ms, " secs=", secs, " now=", stamp, crlf);
    bench.verdict("millis() is ticks() at 1000 Hz (no correction, no drift)",
                  ms >= ticks && ms - ticks <= 2u);
    bench.verdict("secs() is the whole-second part of the same counter",
                  secs == ticks / 1000u || secs == (ticks + 2u) / 1000u);
    bench.verdict("now() agrees with both counters",
                  stamp.seconds == secs || stamp.seconds == secs + 1u);

    // The VAL-delta accumulation delay.hpp is built on, against the
    // interrupt count over 200 wraps.
    const uint32_t target = 200;
    const uint32_t period = reload + 1u;
    uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() == t1) {
    }
    t1 = Ticker::ticks();
    uint32_t last = SysTick->VAL;
    uint32_t accumulated = 0;
    while (Ticker::ticks() - t1 < target) {
        const uint32_t now = SysTick->VAL;
        accumulated += (last >= now) ? (last - now) : (last + period - now);
        last = now;
    }
    const uint32_t expected = target * period;
    const uint32_t cerr = accumulated > expected ? accumulated - expected : expected - accumulated;
    print(serial, "  200 ticks = ", accumulated, " cycles by VAL deltas, ", expected,
          " expected, error ", cerr, " cycles", crlf);
    bench.verdict("the VAL-delta accumulation tracks the interrupt count over "
                  "200 wraps to under a period's worth of error",
                  cerr < period);
}

// =============================================================================
// d - delay_us on the SysTick counter
// =============================================================================
void td_delay() {
    const uint32_t period = SysTick->LOAD + 1u;
    const uint32_t cycles_per_us = delay_rate(SysClock::hz).cycles_per_us;

    // Each wait measured on SysTick's own counter (the same ruler the
    // wait reads, so this judges the ARITHMETIC: at least, never early).
    static const uint32_t spans[] = {5, 30, 100, 500, 900};
    bool all_at_least = true;
    for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        const uint32_t v0 = SysTick->VAL;
        const bool ok = delay_us(clock, spans[i]);
        const uint32_t v1 = SysTick->VAL;
        const uint32_t took = (v0 >= v1) ? (v0 - v1) : (v0 + period - v1);
        const uint32_t took_us = took / cycles_per_us;
        print(serial, "  delay_us(", spans[i], ") -> ", took_us, " us (", took, " cycles)", crlf);
        if (!ok || took < spans[i] * cycles_per_us) {
            all_at_least = false;
        }
    }
    bench.verdict("delay_us serves 5..900 us AT LEAST on its own counter", all_at_least);

    const uint32_t t0 = Ticker::ticks();
    for (uint16_t k = 0; k < 1000; ++k) {
        (void)delay_us(clock, 100u);
    }
    const uint32_t bulk_ms = Ticker::ticks() - t0;
    print(serial, "  1000 x delay_us(100) = ", bulk_ms, " ms of kernel tick (100 due)", crlf);
    bench.verdict("a thousand 100 us waits are 100 ms of kernel time, never less, "
                  "and the per-call overhead costs under 10%",
                  bulk_ms >= 100u && bulk_ms <= 110u);

    bool refused = true;
    for (uint8_t k = 0; k < 4; ++k) {
        refused = refused && !delay_us(clock, 1000u);
    }
    bench.verdict("a wait of one SysTick period or more is REFUSED (TimeEvent territory)",
                  refused);
    bench.verdict("a wait of 65536 us is refused too", !delay_us(clock, 65536u));
}

/// Whether this program's clock names the HSE in bypass (the Nucleo's
/// MCO) or as a crystal - a constexpr of the board file's line.
constexpr bool hse_is_bypass() {
#if defined(STM32F446xx)
    return true;
#else
    return false;
#endif
}

// =============================================================================
// e - the clock tree as the registers hold it
// =============================================================================
void te_clock() {
    print(serial, "  SWS=", static_cast<uint8_t>(Rcc::sysclk_status()), " PLL ",
          Rcc::pll_ready() ? "locked" : "OFF", " HSE ",
          Rcc::hse_ready() ? (Rcc::hse_bypassed() ? "bypass" : "crystal") : "OFF",
          " VOS scale", static_cast<uint8_t>(Pwr::scale()), " OD ",
          Pwr::over_drive_active() ? "on" : "off", " WS ", FlashWaitStates::get(),
          " PPRE1 /", Rcc::apb1_divider(), " PPRE2 /", Rcc::apb2_divider(), crlf);
    bench.verdict("SYSCLK is the PLL (SWS reads 10)", Rcc::sysclk_status() == SysclkSource::pll);
    bench.verdict("the PLL is locked", Rcc::pll_ready());
    bench.verdict("the HSE root is ready, in the mode the board file states",
                  Rcc::hse_ready() && Rcc::hse_bypassed() == hse_is_bypass());
    bench.verdict("the regulator scale is the one the ladder picked for this rate",
                  Pwr::scale() == SysClock::regime.scale);
    bench.verdict("over-drive is active exactly where the rate needs it",
                  Pwr::over_drive_active() == SysClock::regime.over_drive);
    bench.verdict("the flash latency is the task's wait states",
                  FlashWaitStates::get() == SysClock::wait_states);
    bench.verdict("the ART accelerator is on (prefetch, both caches)",
                  FlashAccel::prefetch() && FlashAccel::icache() && FlashAccel::dcache());
    bench.verdict("the APB dividers are the task's",
                  Rcc::apb1_divider() == SysClock::apb1_div && Rcc::apb2_divider() == SysClock::apb2_div);
    bench.verdict("HPRE is 1: HCLK is SYSCLK", Rcc::ahb_undivided());
    bench.verdict("PLLCFGR holds the ratio the task searched",
                  (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos == SysClock::pll.m &&
                  (RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos == SysClock::pll.n &&
                  ((RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos) == (SysClock::pll.p / 2u - 1u) &&
                  (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos == SysClock::pll.q);
}

// =============================================================================
// g - the panic breadcrumb, no reset
// =============================================================================
void tg_breadcrumb() {
    print(serial, "  panic record at ", hex(reinterpret_cast<uintptr_t>(&P::panic_record())), crlf);
    bench.verdict("the .noinit record sits in the main SRAM",
                  reinterpret_cast<uintptr_t>(&P::panic_record()) >= 0x20000000u &&
                      reinterpret_cast<uintptr_t>(&P::panic_record()) < 0x20040000u);
    bench.verdict("nothing is pending", !take_panic_record<P>());
    P::panic_record() = PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::assert_failed), 0x42};
    const auto taken = take_panic_record<P>();
    bench.verdict("a written record is taken with its code and context",
                  taken && taken->code == static_cast<uint8_t>(PanicCode::assert_failed) &&
                      taken->context == 0x42);
    bench.verdict("and taken only once", !take_panic_record<P>());
}

void banner() {
    print(serial, crlf, "test_stm32f4_platform - the STM32F4 platform, SysTick, delay_us, "
          "the clock tree, the breadcrumb", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#elif defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
/// The tick, with two stamps of SysTick's VAL - letter w's split of an
/// edge's way back to the loop into entry, body and exit. Each is one
/// load and one store; every other letter only sees them as time.
extern "C" void SysTick_Handler() {
    tick_entry = SysTick->VAL;
    brio::Ticker::tick();
    tick_exit = SysTick->VAL;
}

/// Letter w's rescue and clock: TIM2's update, counted.
extern "C" void TIM2_IRQHandler() {
    Rescue::clear_flags(Rescue::update_flag);
    rescue_periods = rescue_periods + 1u;
}

/// Letter w's placed edge and letter k's pending line: TIM9 (its line
/// shared with TIM1's break, which nothing here enables), its count
/// stamped first, the compare's interrupt closed so that one edge is one
/// entry (a pend from the NVIC alone takes the same path).
extern "C" void TIM1_BRK_TIM9_IRQHandler() {
    edge_entry = Edge::count();
    Edge::interrupts(Edge::compare_interrupt(0), false);
    Edge::clear_flags(Edge::compare_flag(0));
    edge_hits = edge_hits + 1u;
}

namespace {

/// t: the runtime's seven functions (rt/rt.cpp) over every alignment,
/// length and overlap - rt/selftest.hpp's cases, the ones the host suite
/// runs, against the symbols this image really links.
void tt_runtime() {
    brio::rt_selftest_all(brio::rt_linked_functions(),
                          [](const char* name, const brio::RtSelftestResult& r) {
        print(serial, "  ", r.cases, " cases", crlf);
        if (r.failures != 0u) {
            print(serial, "  ", r.failures, " failed, the first: ", r.first_what, " ",
                  r.first_a, " ", r.first_b, crlf);
        }
        bench.verdict(name, r.failures == 0u);
    });
}

}  // namespace

int main() {
    boot_record = brio::take_panic_record<P>();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the boot story", ta_boot);
    bench.letter('b', "the critical section and the idle hook", tb_critical);
    bench.letter('c', "the SysTick timebase and its VAL arithmetic", tc_ticker);
    bench.letter('d', "delay_us on the SysTick counter", td_delay);
    bench.letter('e', "the clock tree against the task's constants", te_clock);
    bench.letter('g', "the panic breadcrumb, no reset", tg_breadcrumb);
    bench.letter('w', "the idle hook against an interrupt's edge, placed to the cycle", tw_edge);
    bench.letter('k', "the kernel's turn: how many per interrupt, what one costs", tk_turns);
    bench.letter('t', "the runtime's seven functions at every alignment (rt/rt.cpp)", tt_runtime);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
