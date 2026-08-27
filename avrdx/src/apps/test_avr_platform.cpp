// test_avr_platform - the PLATFORM test SUITE for the AVR DA/DB target:
// everything the kernel stands on that is not a peripheral driver. The
// short-wait role (avrdx/delay.hpp), the Platform concept's AVR
// realization (avrdx/platform_avr.hpp: critical section, idle/sleep,
// the timebase, atomic_width, the panic breadcrumb) and the reset side
// of the story (avrdx/reset.hpp: RSTCTRL and the watchdog).
//
// Reference test of those three headers (docs/avrdx/platform.md): keep
// it passing.
//
// Every timing here is COUNTED IN CLK_PER CYCLES, not estimated: a TCB
// pair (TCB1+TCB2) cascaded into one 32-bit counter at CLK_PER is the
// stopwatch, latched by a software event on its snapshot channel. Each
// measurement subtracts the cost of two back-to-back stamps, so what is
// reported is the cycle cost of the code between them. Interrupts are
// masked around the fine measurements: the numbers are exact, not
// statistical.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. No pin is driven, nothing to wire.
// Event channels 4 (the cascade's carry) and 5 (its snapshot).
//
// Test i RESETS THE BOARD four times on purpose and carries its own
// verdicts across the resets in a .noinit token, so `z` still ends with
// one ALL: line. It is always run last.
//
// Commands: ? | a folded delay | b runtime delay | c delay across a
// rebase | d delay_cycles | e critical section | f idle | g ring and
// queue on the silicon | h timebase | i panic breadcrumb across real
// resets | z = all

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>
#include <optional>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "util/print.hpp"
#include "util/ring.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

/// The suite's own reset-surviving breadcrumb: test i spans four resets
/// and its verdicts, its tallies and the record it expects to find have
/// to cross them, so `z` can still close with one ALL: line.
///
/// It lives at namespace scope and is `inline` on purpose. gcc gives an
/// INLINE variable with a custom section attribute a COMDAT group and a
/// plain one none, and the two section types cannot be merged: the
/// platform's own panic_record_ is a static inline member, so this must
/// be inline too or the link fails with a section type conflict.
struct Token {
    uint16_t magic;
    uint16_t canary;
    uint8_t phase;        ///< which reset we are waiting for (0 = none pending)
    uint8_t in_all;       ///< the run was `z`: close with the ALL: line
    uint8_t note;         ///< phase 2: 1 = the window violation did NOT reset us
    uint8_t code;         ///< the PanicCode written before the reset
    uint8_t context;      ///< its context byte
    uint16_t all_pass;    ///< tallies banked by the tests before this one
    uint16_t all_fail;
    uint16_t i_pass;      ///< test i's own tallies so far
    uint16_t i_fail;
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using P = AvrPlatform;
using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;
using DynClock = DynamicClock<SysClock, Serial>;

// ---- the instrument ----------------------------------------------------------
using Alarm = Tcb<0>;                ///< f's wake source, g's producer pace
using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;

constexpr uint32_t crystal_hz = SysClock::hz;
/// CLK_PER cycles in one PIT tick at 24 MHz (24e6 / 1024).
constexpr uint32_t cycles_per_tick = crystal_hz / Ticker::ticks_per_second;

/// The 32-bit CLK_PER stamp: a software event latches both halves.
uint32_t cycles_now() { return Watch::read(); }

void stopwatch_init() {
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});
    Watch::reset();
}

// ---- shared with the ISRs -----------------------------------------------------
volatile uint8_t alarm_mode = 0;      ///< 0 off, 1 = f's alarm, 2 = g's producer
volatile uint32_t alarm_stamp = 0;
volatile bool alarm_ran = false;

Ring<uint16_t, 64, P> ring;
static_assert(P::atomic_width == 1, "the AVR core moves one byte atomically");
static_assert(decltype(ring)::lock_free,
              "an 8-bit index fits atomic_width: this ring must compile to the "
              "LOCK-FREE path, with no interrupt masking in push/pop");
volatile uint16_t produce_seq = 0;
volatile uint16_t produce_limit = 0;
volatile uint16_t produce_rejects = 0;

/// The kernel's own queue over the same platform (test g).
EventQueue<uint8_t, 4, P> queue;

// ---- the breadcrumb token (spans the resets of test i) -------------------------
constexpr uint16_t token_magic = 0x5A17;
constexpr uint16_t token_canary = 0xC3A5;

// What this boot was caused by, and what it inherited. Sampled by main()
// before anything else can disturb either.
ResetFlags boot_reset{};
std::optional<PanicRecord> boot_record;
bool boot_record_twice = false;

/// A reporter that ends the panic in a software reset: the composition
/// kernel/panic.hpp advertises ("reset now and report at next boot").
struct ResetReporter {
    [[noreturn]] static void report(PanicCode, uint8_t) { Reset::software(); }
};

// ---- tiny test harness --------------------------------------------------------
uint16_t passed = 0, failed = 0;
bool in_all_run = false;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

/// The cost of two back-to-back stamps: what every measurement below
/// subtracts. Constant to the cycle (interrupts masked, min of eight).
uint32_t stamp_cost() {
    uint32_t best = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8; ++i) {
        P::CriticalSection cs;
        const uint32_t a = cycles_now();
        const uint32_t b = cycles_now();
        if (b - a < best) best = b - a;
    }
    return best;
}

/// CLK_PER cycles spent inside f(), interrupts masked, best of eight.
template <typename F>
uint32_t cycles_of(uint32_t cost, F f) {
    uint32_t best = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8; ++i) {
        P::CriticalSection cs;
        const uint32_t a = cycles_now();
        f();
        const uint32_t b = cycles_now();
        if (b - a < best) best = b - a;
    }
    return best - cost;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);          // the shifter, generously
}

void quiesce() {
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    (void)Watchdog::off();
    Ticker::init();
    stopwatch_init();
    ring.clear();
    produce_rejects = 0;
    produce_limit = produce_rejects;
    produce_seq = produce_limit;
}

// ---- a the folded path ---------------------------------------------------------
// A static Clock and a compile-time `us`: delay_us folds the cycle count
// and __builtin_avr_delay_cycles emits the tightest loop. The measured
// cost must be the nominal cycle count plus a small constant, and must
// never be SHORT - "at least" is the whole contract.
void ta_folded() {
    print(serial, "a delay_us, folded path (static clock, constant us)", crlf);
    quiesce();
    const uint32_t cost = stamp_cost();
    print(serial, "  two back-to-back stamps cost ", cost, " CLK_PER cycles "
                  "(subtracted from every figure below)", crlf);

    struct Row { uint32_t us; uint32_t measured; };
    Row rows[6] = {
        {1, cycles_of(cost, [] { delay_us(clock, 1); })},
        {2, cycles_of(cost, [] { delay_us(clock, 2); })},
        {5, cycles_of(cost, [] { delay_us(clock, 5); })},
        {10, cycles_of(cost, [] { delay_us(clock, 10); })},
        {50, cycles_of(cost, [] { delay_us(clock, 50); })},
        {100, cycles_of(cost, [] { delay_us(clock, 100); })},
    };
    bool at_least = true;
    int32_t lo = 0x7FFFFFFF, hi = -0x7FFFFFFF;
    for (const Row& r : rows) {
        const uint32_t nominal = r.us * (crystal_hz / 1'000'000u);
        const int32_t over = static_cast<int32_t>(r.measured) - static_cast<int32_t>(nominal);
        if (over < 0) at_least = false;
        if (over < lo) lo = over;
        if (over > hi) hi = over;
        print(serial, "  ", r.us, " us: ", r.measured, " cycles, nominal ", nominal,
              ", overhead ", over, crlf);
    }
    verdict("every folded delay is at least as long as asked", at_least);
    print(serial, "  path overhead spans ", lo, " .. ", hi, " cycles", crlf);
    verdict("the overhead is a small constant, not a per-us cost", hi - lo <= 4);

    // The fold really happened: the same 10 us through a value the
    // compiler cannot see is measurably more expensive (the runtime
    // path computes its loop count). firmware.lst confirms the shape.
    volatile uint32_t opaque = 10;
    const uint32_t runtime10 = cycles_of(cost, [&] { delay_us(clock, opaque); });
    print(serial, "  10 us folded=", rows[3].measured, " vs the same call with an "
                  "opaque us=", runtime10, " cycles", crlf);
    verdict("the constant call took the folded path (cheaper than the loop)",
            rows[3].measured < runtime10);
    quiesce();
}

// ---- b the runtime path --------------------------------------------------------
// A volatile defeats __builtin_constant_p, so delay_us takes the
// fixed-point branch of delay_us_at<hz>: loops = ceil(us * mult / 2^12)
// with mult = ceil(hz/4e6 in Q4.12) folded from the static rate - a
// 16x16 multiply and a shift, never a division. Nominal is unchanged
// (ceil to whole 4-cycle loops).
void tb_runtime() {
    print(serial, "b delay_us, runtime path (the 4-cycle loop)", crlf);
    quiesce();
    const uint32_t cost = stamp_cost();
    const uint8_t cpu = cycles_per_us(crystal_hz);
    verdict("cycles_per_us(24 MHz) = 24", cpu == 24);

    struct Row { uint32_t us; uint32_t measured; };
    volatile uint32_t v = 0;
    Row rows[6];
    const uint32_t list[6] = {1, 2, 5, 10, 50, 100};
    for (uint8_t i = 0; i < 6; ++i) {
        v = list[i];
        rows[i] = {list[i], cycles_of(cost, [&] { delay_us(clock, v); })};
    }
    bool at_least = true;
    int32_t lo = 0x7FFFFFFF, hi = -0x7FFFFFFF;
    for (const Row& r : rows) {
        const uint32_t nominal = r.us * cpu;
        const uint32_t loops = (r.us * cpu + 3u) / 4u;
        const int32_t over = static_cast<int32_t>(r.measured) - static_cast<int32_t>(nominal);
        if (over < 0) at_least = false;
        if (over < lo) lo = over;
        if (over > hi) hi = over;
        print(serial, "  ", r.us, " us: ", r.measured, " cycles, nominal ", nominal,
              " (", loops, " loops of 4), overhead ", over, crlf);
    }
    verdict("every runtime delay is at least as long as asked", at_least);
    print(serial, "  path overhead spans ", lo, " .. ", hi, " cycles", crlf);
    verdict("the overhead is a constant, not a per-us cost", hi - lo <= 6);

    // The ceil is what makes 1 us honest: 24 cycles is exactly 6 loops.
    verdict("1 us at 24 MHz is exactly 6 loops", (1u * cpu + 3u) / 4u == 6u);
    // The bare loop, without the delay_us wrapper: the pure cost of
    // _delay_loop_2's own iteration.
    const uint32_t bare = cycles_of(cost, [] { delay_us_runtime(24, 1); });
    print(serial, "  delay_us_runtime(24, 1) alone: ", bare, " cycles (6 loops = 24)", crlf);
    verdict("the bare loop is at least its nominal 24 cycles", bare >= 24);
    quiesce();
}

// ---- c across a clock rebase -----------------------------------------------------
// A DynamicClock's rate is one of a DISCRETE set the type knows
// (source_hz over the twelve prescalers), so delay_us dispatches on the
// current rate INDEX into per-rate branches folded at compile time: no
// division ever runs at wait time, and the per-rate Q4.12 factor makes
// the microsecond arithmetic exact even at sub-MHz rates. The delay
// must stay honest at each rate, the fixed cost must be small (the
// ceiling verdict bars the old runtime division from coming back), and
// at 1.5 MHz the old whole-cycles-per-us 4/3 overshoot must be GONE.
void tc_rebase() {
    print(serial, "c delay_us under DynamicClock: the index dispatch, 24 -> 12 -> 24 MHz and 1.5 MHz exact", crlf);
    quiesce();
    const bool dyn_ok = DynClock::init();
    console_drain();                  // the console is about to be re-owned
    Serial::init(DynClock{}, 460800);
    verdict("DynamicClock init (boot = the crystal)", dyn_ok);
    const uint32_t cost = stamp_cost();

    // Each rate is measured TWICE, at 1000 and 2000 us. The difference
    // is the delay's per-microsecond cost with the call's fixed cost
    // removed; 2*d1 - d2 is that fixed cost. Milliseconds, not
    // microseconds: on this path the fixed part is large enough to hide
    // the thing being measured.
    struct Leg { uint32_t hz; uint32_t d1; uint32_t d2; };
    Leg legs[3];
    volatile uint32_t v = 1000;

    legs[0].hz = 24'000'000u;
    legs[0].d1 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
    v = 2000;
    legs[0].d2 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    legs[1].hz = 12'000'000u;
    v = 1000;
    legs[1].d1 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
    v = 2000;
    legs[1].d2 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    legs[2].hz = 24'000'000u;
    v = 1000;
    legs[2].d1 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
    v = 2000;
    legs[2].d2 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });

    bool honest = true, slope_exact = true, fixed_same = true;
    uint32_t fixed0 = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        const Leg& l = legs[i];
        const uint32_t us = static_cast<uint32_t>(
            (static_cast<uint64_t>(l.d1) * 1'000'000u) / l.hz);
        const uint32_t slope = l.d2 - l.d1;
        const uint32_t slope_us = static_cast<uint32_t>(
            (static_cast<uint64_t>(slope) * 1'000'000u) / l.hz);
        const uint32_t fixed = 2u * l.d1 - l.d2;
        if (us < 1000u) honest = false;
        if (!near(static_cast<int32_t>(slope_us), 1000, 5)) slope_exact = false;
        if (i == 0) fixed0 = fixed;
        else if (!near(static_cast<int32_t>(fixed), static_cast<int32_t>(fixed0), 8)) {
            fixed_same = false;
        }
        print(serial, "  at ", l.hz, " Hz: 1000 us -> ", l.d1, " cycles (", us,
              " us), 2000 us -> ", l.d2, "; per-1000-us slope ", slope, " cycles = ",
              slope_us, " us; fixed call cost ", fixed, " cycles", crlf);
    }
    verdict("1000 us stays at least 1000 us at every rate", honest);
    verdict("the per-us cost is exact at every whole-MHz rate", slope_exact);
    verdict("the fixed call cost does not depend on the rate", fixed_same);
    print(serial, "  that fixed cost is the index dispatch plus the shared "
                  "fixed-point tail - no division runs at wait time.", crlf);
    verdict("the fixed cost stays under 200 cycles (the runtime division "
            "must not come back)", fixed0 < 200u);

    // A CONSTANT us under the dynamic clock: each dispatch branch folds
    // its own exact loop, so the only runtime cost is the index chain.
    const uint32_t k1000 = cycles_of(cost, [] { delay_us(DynClock{}, 1000); });
    print(serial, "  constant 1000 us at 24 MHz through the dispatch: ", k1000,
          " cycles (nominal 24000)", crlf);
    verdict("a constant us under DynamicClock is at least nominal", k1000 >= 24000u);
    verdict("and its overhead is the dispatch alone, under 100 cycles",
            k1000 - 24000u < 100u);

    // The sub-MHz honesty. 1.5 MHz is 24 MHz / 16, a rate the prescaler
    // really reaches. The old whole-cycles-per-us rounding (still what
    // the stored-byte helper does: cycles_per_us(1.5 MHz) = 2) ran every
    // delay at 4/3 of nominal; the dispatch's Q4.12 factor for this rate
    // is exact (1536/4096 = 0.375 loops/us), so the slope must now be
    // 1000 us, not 1333. The console cannot survive 1.5 MHz (BAUD would
    // have to go below its floor of 64), so the leg is measured in
    // silence and printed after the climb back.
    verdict("the driver knows 460800 is unreachable at 1.5 MHz",
            !Serial::can_baud(1'500'000u, 460800u));
    verdict("cycles_per_us still rounds 1.5 up to 2 (the stored-byte helper)",
            cycles_per_us(1'500'000u) == 2);
    verdict("the dispatch's Q4.12 factor for 1.5 MHz is exact",
            delay_mult(1'500'000u) == 1536u);
    console_drain();
    uint32_t s1 = 0, s2 = 0;
    bool slow_ok = DynClock::set(1'500'000u);
    if (slow_ok) {
        v = 1000;
        s1 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
        v = 2000;
        s2 = cycles_of(cost, [&] { delay_us(DynClock{}, v); });
        slow_ok = DynClock::set(24'000'000u);
    }
    Serial::init(clock, 460800);
    verdict("the rate really reached 1.5 MHz and came back", slow_ok);
    const uint32_t slow_us = (s1 * 1'000'000u) / 1'500'000u;
    const uint32_t slope_us = ((s2 - s1) * 1'000'000u) / 1'500'000u;
    print(serial, "  at 1500000 Hz: 1000 us -> ", s1, " cycles (", slow_us,
          " us), per-1000-us slope ", s2 - s1, " cycles = ", slope_us,
          " us: the per-rate factor is exact, the old 4/3 overshoot is gone", crlf);
    verdict("1000 us at 1.5 MHz is at least 1000 us", slow_us >= 1000u);
    verdict("and the slope is the exact 1000 us, not the old ceil's 1333",
            near(static_cast<int32_t>(slope_us), 1000, 10));
    quiesce();
}

// ---- d delay_cycles ---------------------------------------------------------------
void td_cycles() {
    print(serial, "d delay_cycles: rounding to 4 and the 16-bit chunking", crlf);
    quiesce();
    const uint32_t cost = stamp_cost();

    struct Row { uint32_t k; uint32_t measured; };
    Row rows[5] = {
        {4, cycles_of(cost, [] { delay_cycles(4); })},
        {8, cycles_of(cost, [] { delay_cycles(8); })},
        {100, cycles_of(cost, [] { delay_cycles(100); })},
        {1000, cycles_of(cost, [] { delay_cycles(1000); })},
        // 65538 loops: one full 0xFFFF chunk, then a remainder of 3.
        {65536u * 4u + 8u, cycles_of(cost, [] { delay_cycles(65536u * 4u + 8u); })},
    };
    bool at_least = true;
    for (const Row& r : rows) {
        const uint32_t loops = (r.k + 3u) / 4u;
        const uint32_t nominal = loops * 4u;
        const int32_t over = static_cast<int32_t>(r.measured) - static_cast<int32_t>(nominal);
        if (over < 0) at_least = false;
        print(serial, "  ", r.k, " cycles asked: ", r.measured, " measured, ", loops,
              " loops = ", nominal, " nominal, overhead ", over, crlf);
    }
    verdict("every delay_cycles is at least the multiple of 4 it rounds to", at_least);
    // The chunked case must not cost more than its own arithmetic: the
    // extra loop iteration count is exact, the only additions are the
    // subtraction and the second call.
    const uint32_t chunked = rows[4].measured;
    const uint32_t chunk_nominal = ((65536u * 4u + 8u + 3u) / 4u) * 4u;
    print(serial, "  the chunk boundary adds ", chunked - chunk_nominal,
          " cycles over the two loops' own ", chunk_nominal, crlf);
    verdict("crossing 0xFFFF loops costs only its own arithmetic",
            chunked >= chunk_nominal && chunked - chunk_nominal < 60);
    quiesce();
}

// ---- e the critical section ---------------------------------------------------------
void te_critical() {
    print(serial, "e CriticalSection: nesting, restore, cost", crlf);
    quiesce();
    const uint32_t cost = stamp_cost();

    sei();
    bool masked_in_outer = false, masked_in_inner = false;
    bool masked_after_inner = false, restored_after_outer = false;
    {
        P::CriticalSection outer;
        masked_in_outer = !P::interrupts_enabled();
        {
            P::CriticalSection inner;
            masked_in_inner = !P::interrupts_enabled();
        }
        masked_after_inner = !P::interrupts_enabled();
    }
    restored_after_outer = P::interrupts_enabled();
    verdict("the guard masks interrupts", masked_in_outer);
    verdict("a nested guard masks them too", masked_in_inner);
    verdict("the INNER exit leaves them masked (it restores, not enables)",
            masked_after_inner);
    verdict("the OUTER exit restores the caller's state", restored_after_outer);

    // Entered with interrupts already masked: the exit must NOT enable
    // them. This is the whole difference between saving SREG and a
    // blind sei, and the reason panic() can hold a guard forever.
    cli();
    bool still_masked = false;
    {
        P::CriticalSection cs;
        still_masked = !P::interrupts_enabled();
    }
    still_masked = still_masked && !P::interrupts_enabled();
    sei();
    verdict("a guard entered with interrupts masked leaves them masked", still_masked);

    const uint32_t empty = cycles_of(cost, [] { P::CriticalSection cs; });
    const uint32_t nested = cycles_of(cost, [] {
        P::CriticalSection a;
        P::CriticalSection b;
    });
    print(serial, "  one empty guard costs ", empty, " CLK_PER cycles, two nested ",
          nested, crlf);
    verdict("the guard is a handful of cycles, not a call", empty <= 8);
    verdict("nesting costs the same again", near(static_cast<int32_t>(nested),
                                                 2 * static_cast<int32_t>(empty), 2));
    print(serial, "  the memory-barrier half of the contract is a COMPILE-time "
                  "property: it is exercised, not timed, by every ISR-shared "
                  "counter in this suite (f, g, h).", crlf);
    quiesce();
}

// ---- f idle() ------------------------------------------------------------------------
void tf_idle() {
    print(serial, "f idle(): sleep until the next interrupt", crlf);
    quiesce();
    const uint32_t cost = stamp_cost();

    // 1. woken by the timebase, and disarmed on the way out. The
    // console must be silent first: an unfinished line leaves the DRE
    // interrupt armed, and THAT would be the interrupt doing the waking
    // (measured: a wake every ~250 cycles, one per frame at 460800).
    console_drain();
    uint32_t lo = 0xFFFFFFFFu, hi = 0, sum = 0;
    bool woke_on_interrupt = true, disarmed = true, enabled_on_return = true;
    constexpr uint8_t n = 16;
    for (uint8_t i = 0; i < n; ++i) {
        cli();
        const uint32_t before = P::now();
        const uint32_t t0 = cycles_now();
        P::idle();
        const bool armed = P::sleep_armed();
        const bool ints = P::interrupts_enabled();
        cli();
        const uint32_t t1 = cycles_now();
        sei();
        if (armed) disarmed = false;
        if (!ints) enabled_on_return = false;
        if (P::now() == before) woke_on_interrupt = false;
        const uint32_t d = t1 - t0 - cost;
        sum += d;
        if (d < lo) lo = d;
        if (d > hi) hi = d;
    }
    print(serial, "  woken by the 1024 Hz tick: ", lo, " .. ", hi, " cycles (mean ",
          sum / n, ") of a ", cycles_per_tick, "-cycle tick period", crlf);
    verdict("idle() returned only after an interrupt actually fired", woke_on_interrupt);
    verdict("the wait never exceeds one tick period", hi <= cycles_per_tick + cycles_per_tick / 8);
    verdict("SLPCTRL is disarmed on return (SEN back to 0)", disarmed);
    verdict("interrupts are enabled on return", enabled_on_return);

    // 2. the wake-up penalty. The same alarm fires at a known cycle; the
    // ISR stamps the clock itself, so the interrupt entry cost cancels
    // and what is left between the two variants is the cost of having
    // been ASLEEP (13.3.3.2: six CLK_PER cycles, plus idle()'s own NOP,
    // arming store and sei).
    console_drain();
    Ticker::pause();
    alarm_mode = 1;
    (void)Alarm::init({.mode = TcbMode::periodic, .clock = TcbClock::div1, .compare = 24000});
    Alarm::enable_capt_interrupt(true);
    uint32_t slept = 0xFFFFFFFFu, spun = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8; ++i) {
        cli();
        alarm_ran = false;
        Alarm::count(0);
        Alarm::clear_capt();
        uint32_t t0 = cycles_now();
        P::idle();
        while (!alarm_ran) {           // any other interrupt is not the end
        }
        cli();
        uint32_t d = alarm_stamp - t0;
        if (d < slept) slept = d;

        alarm_ran = false;
        Alarm::count(0);
        Alarm::clear_capt();
        t0 = cycles_now();
        sei();
        while (!alarm_ran) {
        }
        cli();
        d = alarm_stamp - t0;
        if (d < spun) spun = d;
        sei();
    }
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    Ticker::resume();
    print(serial, "  same alarm, stamped inside its ISR: asleep ", slept,
          " cycles vs spinning ", spun, " -> waking costs ",
          static_cast<int32_t>(slept) - static_cast<int32_t>(spun), " cycles", crlf);
    verdict("sleeping delays the ISR by only a handful of cycles",
            slept >= spun && slept - spun <= 20);

    // 3. the CPU really stopped. Over the same wall-clock span, a loop
    // that only runs when the CPU runs turns thousands of times awake
    // and once per tick asleep.
    volatile uint32_t work_busy = 0, work_sleep = 0;
    console_drain();
    sei();
    {
        const uint32_t t0 = P::now();
        while (P::now() - t0 < 32u) {
            work_busy = work_busy + 1;
        }
    }
    {
        const uint32_t t0 = P::now();
        for (;;) {
            cli();
            if (P::now() - t0 >= 32u) {
                sei();
                break;
            }
            P::idle();
            work_sleep = work_sleep + 1;
        }
    }
    print(serial, "  over 32 ticks: ", work_busy, " loop turns awake, ", work_sleep,
          " asleep (one per wake)", crlf);
    verdict("the counter is frozen while the CPU sleeps", work_sleep <= 40);
    verdict("the same span awake turns it thousands of times",
            work_busy > 100u * (work_sleep ? work_sleep : 1u));
    quiesce();
}

// ---- g the ring and the queue on the silicon --------------------------------------
// util/ring.hpp's lock-free path is the one compiled here (the
// static_assert at the top of this file states it): an 8-bit index fits
// AvrPlatform::atomic_width, so push() and pop() touch no interrupt
// mask. A real ISR produces, main consumes, and the sequence proves it.
void tg_ring() {
    print(serial, "g Ring (lock-free) and EventQueue on the silicon", crlf);
    quiesce();
    verdict("the ring compiled to the lock-free path", decltype(ring)::lock_free);
    verdict("capacity is size - 1", ring.capacity() == 63);

    constexpr uint16_t total = 50000;
    ring.clear();
    produce_seq = 0;
    produce_rejects = 0;
    produce_limit = total;
    // Nothing may print between arming the producer and the consume
    // loop: a console line is 60 slots' worth of production.
    const bool started = PeriodicTick<Alarm>::init(clock, 20'000u);
    alarm_mode = 2;
    const uint32_t t0 = cycles_now();

    uint16_t expect = 0;
    uint32_t got = 0, stall = 0;
    bool exact = true;
    while (got < total) {
        const std::optional<uint16_t> v = ring.pop();
        if (v) {
            if (*v != expect) exact = false;
            ++expect;
            ++got;
            stall = 0;
        } else if (++stall > 2'000'000u) {
            break;
        }
    }
    const uint32_t elapsed = cycles_now() - t0;
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    print(serial, "  ", got, " elements through the ring in ", elapsed / (crystal_hz / 1000u),
          " ms, ", produce_rejects, " pushes refused", crlf);
    verdict("the producer tick started", started);
    verdict("every element arrived", got == total);
    verdict("in order and uncorrupted", exact);
    verdict("nothing was refused while the consumer kept up", produce_rejects == 0);
    verdict("the ring is empty at the end", ring.empty());

    // A deliberate overrun: the producer runs, nobody consumes.
    ring.clear();
    produce_seq = 0;
    produce_rejects = 0;
    produce_limit = 300;
    alarm_mode = 2;
    (void)PeriodicTick<Alarm>::init(clock, 20'000u);
    delay_us(clock, 50000);
    alarm_mode = 0;
    Alarm::enable_capt_interrupt(false);
    Alarm::disable();
    const uint16_t rejects = produce_rejects;
    const uint16_t seq = produce_seq;
    print(serial, "  overrun: ", seq, " pushed, ", rejects, " refused, ", ring.count(),
          " queued", crlf);
    verdict("a full ring refuses instead of overwriting", rejects > 0);
    verdict("it holds exactly its capacity", ring.count() == ring.capacity());
    verdict("full() agrees", ring.full());
    verdict("the producer stopped at capacity", seq == ring.capacity());
    bool contiguous = true;
    for (uint16_t i = 0; i < 63; ++i) {
        const std::optional<uint16_t> v = ring.pop();
        if (!v || *v != i) contiguous = false;
    }
    verdict("and what survived is the OLDEST 63, in order", contiguous && ring.empty());

    // The kernel's own queue over the same platform: the overflow
    // counter saturates instead of wrapping.
    for (uint8_t i = 0; i < 4; ++i) queue.push(i);
    verdict("the queue takes its depth", queue.size() == 4 && queue.overflows() == 0);
    for (uint8_t i = 0; i < 3; ++i) queue.push(i);
    verdict("overflow is counted, not fatal", queue.overflows() == 3 && queue.size() == 4);
    for (uint32_t i = 0; i < 70'000u; ++i) queue.push(0);
    verdict("the counter saturates instead of lying by wrapping",
            queue.overflows() == 0xFFFFu);
    quiesce();
}

// ---- h the timebase ------------------------------------------------------------------
void th_timebase() {
    print(serial, "h now() / ticks_per_second, and break_here()", crlf);
    quiesce();
    verdict("ticks_per_second is the PIT's 1024", P::ticks_per_second == 1024u);
    verdict("and it is the Ticker's own rate",
            P::ticks_per_second == Ticker::ticks_per_second);

    // 1024 ticks against the crystal.
    uint32_t t = P::now();
    while (P::now() == t) {
    }
    const uint32_t k0 = P::now();
    const uint32_t c0 = cycles_now();
    while (P::now() - k0 < 1024u) {
    }
    const uint32_t c1 = cycles_now();
    const uint32_t cycles = c1 - c0;
    const int32_t ppm = static_cast<int32_t>(
        (static_cast<int64_t>(cycles) - crystal_hz) * 1'000'000 / crystal_hz);
    print(serial, "  1024 ticks = ", cycles, " crystal cycles (nominal ", crystal_hz,
          ") -> the timebase is ", -ppm, " ppm fast", crlf);
    verdict("one second within the OSC32K's +-10 % spec",
            near(static_cast<int32_t>(cycles), static_cast<int32_t>(crystal_hz),
                 static_cast<int32_t>(crystal_hz / 10)));
    verdict("and within the +-3 % this desk's OSC32K really shows",
            near(static_cast<int32_t>(cycles), static_cast<int32_t>(crystal_hz),
                 static_cast<int32_t>(crystal_hz / 33)));

    // Monotonic across the low byte's wrap: the 32-bit counter is read
    // whole under a critical section, so no word may ever interleave.
    uint32_t prev = P::now();
    const uint32_t start = prev;
    uint16_t wraps = 0;
    bool monotonic = true, single_step = true;
    while (prev - start < 600u) {
        const uint32_t cur = P::now();
        const uint32_t d = cur - prev;
        if (cur < prev) monotonic = false;
        if (d > 1u) single_step = false;
        if (d == 1u && (prev & 0xFFu) == 0xFFu) ++wraps;
        prev = cur;
    }
    print(serial, "  600 ticks sampled, ", wraps, " low-byte wraps crossed", crlf);
    verdict("now() never goes backwards", monotonic);
    verdict("it advances one tick at a time", single_step);
    verdict("at least two low-byte wraps were crossed", wraps >= 2);

    P::break_here();
    verdict("break_here() falls through as a NOP with no debugger attached", true);
    print(serial, "  the with-OCD half of break_here() needs a debug session and is "
                  "not verifiable from this console.", crlf);
    quiesce();
}

// ---- i the breadcrumb across real resets ----------------------------------------------
// This test spans four resets. Its state lives in `token`, a .noinit
// object of this app, and every leg is entered from main() at boot.

void bank(uint8_t phase) {
    token.magic = token_magic;
    token.canary = token_canary;
    token.phase = phase;
    token.in_all = in_all_run ? 1 : 0;
    token.i_pass = passed;
    token.i_fail = failed;
}

void report_boot() {
    print(serial, "  boot: RSTFR=", hex(boot_reset.raw),
          boot_reset.power_on ? " POR" : "",
          boot_reset.brown_out ? " BOR" : "",
          boot_reset.external ? " EXT" : "",
          boot_reset.watchdog ? " WDT" : "",
          boot_reset.software ? " SW" : "",
          boot_reset.updi ? " UPDI" : "",
          boot_record ? "; a panic record was pending" : "; no panic record",
          crlf);
}

/// Phase 1: write the breadcrumb through panic() and let the watchdog
/// take the board down under it.
[[noreturn]] void arm_watchdog_panic() {
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = 0x5A;
    bank(1);
    print(serial, "  arming WDT 8 ms and panicking (the board will reset) ...", crlf);
    console_drain();
    (void)Watchdog::arm(WdtTime::ms8);
    panic<P, HaltReporter>(PanicCode::assert_failed, 0x5A);
}

void ti_reset() {
    print(serial, "i the panic breadcrumb across real resets", crlf);
    report_boot();
    verdict("this boot names a reset source", boot_reset.any());
    verdict("no panic record is pending on a clean start", !boot_record);
    verdict("the WDT is unlocked (WDTCFG fuse OFF)", !Watchdog::locked());
    verdict("the WDT starts disabled", !Watchdog::enabled());
    if (Watchdog::locked() || Watchdog::enabled()) {
        print(serial, "  the watchdog is not ours to drive on this board: "
                      "the reset legs are SKIPPED", crlf);
        return;
    }
    verdict("arm(8 ms) accepted", Watchdog::arm(WdtTime::ms8));
    verdict("PERIOD and WINDOW read back",
            Watchdog::period() == WdtTime::ms8 && Watchdog::window() == WdtTime::off);
    verdict("wdt_time_us(ms8) is the data sheet's 7812 us", wdt_time_us(WdtTime::ms8) == 7812u);
    verdict("arm(8 s, window 8 ms) accepted",
            Watchdog::arm(WdtTime::s8, WdtTime::ms8));
    verdict("both fields read back",
            Watchdog::period() == WdtTime::s8 && Watchdog::window() == WdtTime::ms8);
    verdict("off() disables it again", Watchdog::off() && !Watchdog::enabled());
    arm_watchdog_panic();
}

/// Everything after a reset: called from main() instead of the banner.
void ti_resume() {
    passed = token.i_pass;
    failed = token.i_fail;
    in_all_run = token.in_all != 0;
    const uint8_t phase = token.phase;
    print(serial, crlf, "i (continued after reset ", phase, " of 4)", crlf);
    report_boot();

    if (phase == 1) {
        verdict("RSTFR names a WATCHDOG reset", boot_reset.watchdog);
        verdict("and no other source", boot_reset.raw == 0x08u);
        verdict("the breadcrumb survived the reset", boot_record.has_value());
        verdict("its magic is panic_magic",
                boot_record && boot_record->magic == panic_magic);
        verdict("its code is the one panic() was given",
                boot_record && boot_record->code == token.code);
        verdict("its context byte came through untouched",
                boot_record && boot_record->context == token.context);
        verdict("a second take_panic_record() returns nothing", !boot_record_twice);

        // Phase 2: a WINDOW violation. PERIOD 8 s would take forever;
        // the closed window makes the WDR itself the reset.
        bank(2);
        token.note = 0;
        print(serial, "  arming WDT PERIOD=8 s WINDOW=64 ms, one WDR to activate the "
                      "window and a second one inside it ...", crlf);
        console_drain();
        (void)Watchdog::arm(WdtTime::s8, WdtTime::ms64);
        (void)Watchdog::sync();            // the window is in force only now
        Watchdog::clear();                 // activates the window (22.3.3.2)
        delay_us(clock, 5'000u);           // let that WDR synchronize
        Watchdog::clear();                 // 5 ms into a 62.5 ms closed window
        delay_us(clock, 200'000u);         // window mode did not act
        token.note = 1;
        (void)Watchdog::off();
        Reset::software();
    }

    if (phase == 2) {
        verdict("a WDR inside the closed window reset the device", token.note == 0);
        if (token.note == 0) {
            verdict("RSTFR names a WATCHDOG reset, not a software one",
                    boot_reset.watchdog && !boot_reset.software);
        }
        verdict("a fresh boot after a clean take reports NO pending record",
                !boot_record.has_value());

        // Phase 3: the same breadcrumb, ended by a SOFTWARE reset from
        // inside the reporter.
        token.code = static_cast<uint8_t>(PanicCode::kernel_fault);
        token.context = 0xA5;
        bank(3);
        print(serial, "  panicking into a reporter that resets in software ...", crlf);
        console_drain();
        panic<P, ResetReporter>(PanicCode::kernel_fault, 0xA5);
    }

    if (phase == 3) {
        verdict("RSTFR names a SOFTWARE reset", boot_reset.software);
        verdict("and no watchdog reset", !boot_reset.watchdog);
        verdict("the breadcrumb survived a software reset too",
                boot_record.has_value());
        verdict("code and context are the reporter run's",
                boot_record && boot_record->code == token.code &&
                    boot_record->context == token.context);
        verdict("a second take_panic_record() returns nothing", !boot_record_twice);

        // Phase 4: the watchdog lock, then a plain software reset with
        // no panic at all.
        verdict("the WDT is unlocked again after a reset", !Watchdog::locked());
        verdict("and disabled", Watchdog::off() && !Watchdog::enabled());
        Watchdog::lock();
        verdict("LOCK reads back", Watchdog::locked());
        verdict("arm() refuses while CTRLA is locked", !Watchdog::arm(WdtTime::ms8));
        verdict("and nothing was written", !Watchdog::enabled());
        bank(4);
        print(serial, "  resetting in software with no panic at all ...", crlf);
        console_drain();
        Reset::software();
    }

    // phase 4: the last boot.
    verdict("RSTFR names a SOFTWARE reset", boot_reset.software);
    verdict("the reset released the watchdog lock", !Watchdog::locked());
    verdict("a plain reset leaves no panic record", !boot_record.has_value());
    verdict("the .noinit token crossed all four resets intact",
            token.canary == token_canary);
    token.magic = 0;
    token.phase = 0;
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
    if (in_all_run) {
        print(serial, "ALL: ", static_cast<uint16_t>(token.all_pass + passed), " pass, ",
              static_cast<uint16_t>(token.all_fail + failed), " fail", crlf);
    }
}

// ---- the menu -----------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_folded}, {'b', tb_runtime}, {'c', tc_rebase}, {'d', td_cycles},
    {'e', te_critical}, {'f', tf_idle}, {'g', tg_ring}, {'h', th_timebase},
    {'i', ti_reset},
};
constexpr char all_keys[] = "abcdefghi";

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    in_all_run = true;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            token.all_pass = tp;          // banked before the test that may reset
            token.all_fail = tf;
            run(t.fn);
            tp += passed;
            tf += failed;
        }
    }
    in_all_run = false;
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

void help() {
    print(serial, "test_avr_platform: a delay_us folded | b delay_us runtime | "
                  "c delay across a rebase | d delay_cycles | e critical section | "
                  "f idle | g ring and queue | h timebase | i panic breadcrumb "
                  "across four REAL resets    -> z = all of a..i", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }

ISR(TCB0_INT_vect) {
    (void)Alarm::take_flags();
    if (alarm_mode == 1) {
        alarm_stamp = cycles_now();
        alarm_ran = true;
    } else if (alarm_mode == 2) {
        for (uint8_t k = 0; k < 4; ++k) {
            const uint16_t next = produce_seq;
            if (next >= produce_limit) {
                break;
            }
            if (ring.push(next)) {
                produce_seq = static_cast<uint16_t>(next + 1);
            } else {
                // Refused, not lost: the same value goes again next tick.
                produce_rejects = static_cast<uint16_t>(produce_rejects + 1);
                break;
            }
        }
    }
}

int main() {
    // FIRST: the two things a later line would destroy. RSTFR
    // accumulates until someone clears it, and the breadcrumb is a
    // fetch-and-clear.
    boot_reset = Reset::take_flags();
    boot_record = take_panic_record<P>();
    boot_record_twice = take_panic_record<P>().has_value();

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    stopwatch_init();
    sei();

    if (token.magic == token_magic && token.phase != 0) {
        ti_resume();
    } else {
        token.magic = 0;
        auto board = board_id();
        if (board.empty()) board = "?";
        print(serial, crlf, "test_avr_platform - platform test suite (board ", board,
              ", clk=", xtal ? "XTAL" : "OSCHF",
              " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
        report_boot();
        help();
    }
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'z' || c == 'Z') { run_set(all_keys); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
