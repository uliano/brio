/*
 * test_wait.hpp
 *
 * The waits a bench test suite needs, written once: wait for a condition
 * with a time-out, measure how long a condition took to come true, spin
 * for a fixed time, wait on the kernel tick, let the console's last byte
 * leave the wire, and tell whether a measured span was the measured code
 * alone. A suite (a `test_<target>_<subject>` app, util/testbench.hpp)
 * includes this header and names three things it already has: a ruler,
 * the free-running cycle counter util/bench.hpp calls a CycleRuler; its
 * kernel platform; and its console transport. Nothing here knows a
 * target, so every family's suites share one implementation and the host
 * tests it (test/test_bench_wait).
 *
 * The smallest use, an excerpt from a letter whose suite has its family's
 * Ticker, its clock type SysClock and its TestBench `bench` in scope,
 * Peer::ready() standing for whatever the letter waits on:
 *
 *     struct Ruler {
 *         static uint32_t now() { return Ticker::cycles(); }
 *         static constexpr uint32_t hz() { return SysClock::hz; }
 *     };
 *     ...
 *     if (!brio::wait_until<Ruler>([] { return Peer::ready(); }, 5'000)) {
 *         bench.verdict("the peer answered within 5 ms", false);
 *         return;
 *     }
 *
 * The rules a test letter follows, which these verbs exist to make easy:
 *
 *  - A time-out is a time on a ruler, never a loop count. A loop of
 *    600000 turns is "about 50 ms" on one clock, one wait-state setting
 *    and one compiler; a budget in microseconds is 50 ms on every chip.
 *  - A wait that can run out returns a bool, and the letter acts on it:
 *    a verdict, an early return, a skip (TestBench::skip()). The verbs
 *    that can run out are marked [[nodiscard]], so the compiler warns
 *    where the answer is dropped; a deliberate drop is written
 *    `(void)` with its reason beside it.
 *  - A timed window opens after console_drain() and prints after it
 *    closes. A console line going out while a span is measured puts the
 *    transmit interrupt inside the span: take the readings first, print
 *    them afterwards.
 *  - A derived bound judges only the runs the witness calls quiet. A
 *    bound computed from the code's cost describes the code alone; a run
 *    that a kernel tick (or another interrupt) lengthened is reported
 *    with its figure and left out of the judgement (TickWitness below).
 *  - A wait inside a measured path adds its poll turn to the figure.
 *    Where a letter times the path a wait guards (a transfer's cost, a
 *    period's spread), the turn - one condition call and one ruler read
 *    - is part of what it measures: poll the flag bare and check the
 *    budget every so many turns, or state the turn in the bound.
 *
 * Who calls what. Every verb runs in main context - setup, a letter, the
 * handler of an active object - and never in an interrupt handler. None
 * masks interrupts of its own; each turn of a poll is one call of the
 * condition and one read of the ruler (or of the tick count), and the
 * ruler's read masks interrupts only if the ruler does (one over the
 * AVR's two cascaded counters reads them under a mask, for a few loads).
 * Interrupts must be
 * enabled while a verb runs when the ruler is built on the kernel tick,
 * as a ticker's cycles() is, and always for wait_ticks() and
 * TickWitness: with them masked the tick stops and a time-out never
 * comes. The condition is any callable returning bool - a lambda
 * usually - called in the caller's context, as often as the loop turns;
 * whatever it spends is added to the turn, and to how late a time-out is
 * seen, so it should not wait itself.
 *
 * The ruler's rate. The conversion from microseconds to cycles is a
 * multiplication by the whole number of cycles in one microsecond,
 * computed at compile time from R::hz(): no 64-bit multiply or divide
 * reaches an image, which the CH32V003 (RV32EC, no multiplier, 16 KB)
 * could not afford. So a ruler's hz() must be a constant expression
 * (declared constexpr: a hz() that returns a constant without the
 * keyword is not one) and a whole number of megahertz; a ruler that is
 * neither is refused at compile time, with the reason. A ruler on a clock whose rate changes at run time (a
 * DynamicClock) is not a ruler these verbs can convert for.
 *
 * Half the wrap. A difference of two ruler reads is exact under 2^32
 * cycles, and a poll notices the end of its budget one turn late, or
 * later when an interrupt handler runs in between; so a budget is held
 * to half the ruler's range, 2^31 cycles (about 44 s at 48 MHz, 12 s at
 * 180 MHz), leaving the other half to the late read. A constant budget
 * longer than that is a compile error where the optimizer sees the
 * constant (every optimized build; a -O0 build cannot ask), and a run-
 * time budget longer than that saturates to it. The same rule holds for
 * wait_ticks() on the tick count, and span_fits() states it for a
 * counter narrower than 32 bits.
 *
 * The console. console_drain() waits on the console's tx_idle(), which
 * on every transport brio has answers for the shift register as well as
 * the ring, so no extra delay "for the shifter" belongs after it: the
 * AVR's USART (TXCIF), the SAM C21's SERCOM (TXC), the STM32G0's USART
 * and LPUART and the STM32F4's USART (TC), the CH32V00x's, the
 * CH32V203/CH32V303's and the CH32X035's USART (STATR.TC), the PL011 of
 * the RP2040 and the RP2350 (BUSY), each with its transmit engine's busy
 * flag where it has one; and the USB CDC port (util/usb/cdc.hpp), where
 * idle means the host has taken the last packet.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <optional>
#include <type_traits>

#include "kernel/platform.hpp"
#include "util/bench.hpp"

namespace brio {

/// Half the range of a 32-bit counter: the longest budget a wait takes,
/// in cycles or in ticks (the file header, "Half the wrap").
inline constexpr uint32_t half_wrap = 0x8000'0000UL;

/// A console a drain can wait on: tx_idle() answers whether the ring is
/// empty and the last frame has left the wire (the file header, "The
/// console").
template <typename C>
concept TxIdleSink = requires {
    { C::tx_idle() } -> std::same_as<bool>;
};

namespace test_wait_detail {

/// Whether R::hz() is a constant expression: only then can the rate be
/// read at compile time.
template <typename R>
concept ConstantRate = requires { typename std::integral_constant<uint32_t, R::hz()>; };

/// What the conversion needs of ruler R, computed once at compile time:
/// the whole cycles in a microsecond and the longest budget that stays
/// under half the wrap.
template <CycleRuler R>
struct Rate {
    static_assert(ConstantRate<R>,
                  "brio::test_wait: the ruler's hz() must be a constant expression "
                  "(constexpr), so that microseconds convert to cycles at compile time");
    static constexpr uint32_t hz = [] {
        if constexpr (ConstantRate<R>) {
            return R::hz();
        } else {
            return uint32_t{1'000'000};
        }
    }();
    static_assert(hz >= 1'000'000UL && hz % 1'000'000UL == 0u,
                  "brio::test_wait: the ruler's rate must be a whole number of megahertz "
                  "(a whole number of cycles per microsecond)");
    static constexpr uint32_t per_us = hz >= 1'000'000UL ? hz / 1'000'000UL : 1u;
    static constexpr uint32_t max_us = half_wrap / per_us;
};

/// Never defined: a call that survives optimization stops the build with
/// this message (gcc's error attribute), and the call is only reached
/// with a constant budget past half the wrap.
[[gnu::error("brio::test_wait: a constant budget longer than half the counter's wrap "
             "(2^31 cycles or ticks)")]]
void budget_past_half_the_wrap();

/// The budget check every verb opens with: a constant past the limit is a
/// compile error (in a constant evaluation by the non-constexpr call,
/// elsewhere by the error attribute once the optimizer sees the
/// constant), a run-time one saturates to the limit.
[[gnu::always_inline]] constexpr uint32_t clamp_budget(uint32_t value, uint32_t limit) {
    if consteval {
        if (value > limit) {
            budget_past_half_the_wrap();
        }
    } else {
        if (__builtin_constant_p(value) && value > limit) {
            budget_past_half_the_wrap();
        }
    }
    return value > limit ? limit : value;
}

/// wait_until()'s loop over a budget already in cycles.
template <CycleRuler R, typename F>
bool wait_cycles(F& done, uint32_t budget) {
    const uint32_t t0 = R::now();
    while (!done()) {
        if (R::now() - t0 >= budget) {
            return done();
        }
    }
    return true;
}

/// cycles_until()'s loop over a budget already in cycles.
template <CycleRuler R, typename F>
std::optional<uint32_t> count_cycles(F& done, uint32_t budget) {
    const uint32_t t0 = R::now();
    for (;;) {
        const bool held = done();
        const uint32_t elapsed = R::now() - t0;
        if (held) {
            return elapsed;
        }
        if (elapsed >= budget) {
            if (done()) {
                return R::now() - t0;
            }
            return std::nullopt;
        }
    }
}

/// spin_us()'s loop over a span already in cycles.
template <CycleRuler R>
void spin_cycles(uint32_t span) {
    const uint32_t t0 = R::now();
    while (R::now() - t0 < span) {
    }
}

/// wait_ticks()'s loop over a budget already clamped.
template <Platform P, typename F>
bool tick_loop(F& done, uint32_t ticks) {
    const uint32_t t0 = P::now();
    while (!done()) {
        if (P::now() - t0 > ticks) {
            return done();
        }
    }
    return true;
}

} // namespace test_wait_detail

/// Microseconds as cycles of ruler R: a multiplication by the constant
/// cycles per microsecond, the microseconds first held to the most that
/// stays within half the wrap (2^31 cycles); a constant past that is a
/// compile error (the file header, "Half the wrap"). Any context; never
/// waits, never masks.
template <CycleRuler R>
[[gnu::always_inline]] constexpr uint32_t us_cycles(uint32_t us) {
    using Rate = test_wait_detail::Rate<R>;
    return test_wait_detail::clamp_budget(us, Rate::max_us) * Rate::per_us;
}

/**
 * Wait until done() returns true or budget_us microseconds have passed on
 * ruler R. True when the condition held; false when the budget ran out
 * with it still false - the condition is asked once more after the
 * budget, so a condition that came true during the last turn is not
 * reported as a time-out. Main context; never masks; each turn of the
 * poll is one done() and one R::now(). Waits at most the budget plus one
 * turn (plus whatever interrupt handlers take meanwhile).
 */
template <CycleRuler R, typename F>
[[nodiscard, gnu::always_inline]] inline bool wait_until(F&& done, uint32_t budget_us) {
    return test_wait_detail::wait_cycles<R>(done, us_cycles<R>(budget_us));
}

/**
 * The same wait, answering how long it took: the cycles of ruler R from
 * the start to the turn that saw done() true, or nothing when the budget
 * ran out (the condition asked once more after it, as wait_until() does).
 * For a letter that times the wait itself; the reading is late by at most
 * one turn of the poll - one done() and one R::now() - which the letter's
 * bound states. Main context; never masks.
 */
template <CycleRuler R, typename F>
[[nodiscard, gnu::always_inline]] inline std::optional<uint32_t> cycles_until(
    F&& done, uint32_t budget_us) {
    return test_wait_detail::count_cycles<R>(done, us_cycles<R>(budget_us));
}

/// Spin for at least `us` microseconds on ruler R: a fixed span, the
/// replacement for a delay loop or a loop count. Main context; never
/// masks; each turn one R::now().
template <CycleRuler R>
[[gnu::always_inline]] inline void spin_us(uint32_t us) {
    test_wait_detail::spin_cycles<R>(us_cycles<R>(us));
}

/**
 * Wait until done() returns true or `ticks` whole periods of the kernel
 * tick have passed on platform P (P::now()): for waits longer than a
 * ruler can hold, and for a suite whose ruler is busy elsewhere (the
 * AVR's counters, when the letter tests them). True when the condition
 * held, asked once more after the budget as wait_until() asks it. The
 * time-out comes after at least `ticks` and at most `ticks` + 1 periods,
 * since the wait starts somewhere inside a period; kernel/time.hpp's
 * ticks_from_ms() converts milliseconds. The budget is held to half the
 * tick counter's wrap as a ruler's is. Main context with interrupts
 * enabled (masked, the tick stops and the time-out never comes); masks
 * nothing of its own, and P::now() masks only where its platform says
 * so; each turn one done() and one P::now().
 */
template <Platform P, typename F>
[[nodiscard, gnu::always_inline]] inline bool wait_ticks(F&& done, uint32_t ticks) {
    return test_wait_detail::tick_loop<P>(done,
                                          test_wait_detail::clamp_budget(ticks, half_wrap - 1u));
}

/**
 * Wait until every byte the console has queued is off the wire, its last
 * stop bit included (Console::tx_idle(), the file header's "The
 * console"), for at most budget_us microseconds on ruler R. True when the
 * console went idle; false on the budget - a wedged console reported
 * rather than hung on. The default budget, 200 ms, covers a 2 KB ring at
 * 115200 baud with ten bits a byte (178 ms). What a letter calls before it opens a timed window. Main
 * context; never masks; each turn one tx_idle() and one R::now().
 */
template <CycleRuler R, TxIdleSink Console>
[[nodiscard, gnu::always_inline]] inline bool console_drain(uint32_t budget_us = 200'000u) {
    auto idle = [] { return Console::tx_idle(); };
    return test_wait_detail::wait_cycles<R>(idle, us_cycles<R>(budget_us));
}

/**
 * The witness a timing letter keeps beside a measured span: the kernel
 * ticks that landed inside it. Construct it right before the span opens
 * (after console_drain()), read quiet() or ticks() right after the span
 * closes and before anything prints. A run the witness does not call
 * quiet carried a tick handler that is not part of the code measured: the
 * letter reports that run with its figure and judges its derived bound
 * on the quiet runs alone. The tick is one thing that lengthens a span;
 * a suite keeps its own counters beside the witness for the others it
 * knows of (the RP chips' flash cache misses, a vector's entry count).
 * On a platform whose tick count is a hardware counter the core does not
 * service (a tickless timebase), a moved count is a conservative answer:
 * the run is called lengthened though no handler may have run. Main
 * context with interrupts enabled; masks nothing of its own, and P::now()
 * masks only where its platform says so; each read is one P::now().
 */
template <Platform P>
class TickWitness {
public:
    [[gnu::always_inline]] TickWitness() : start_(P::now()) {}

    /// No tick since the witness was built (or restarted).
    [[gnu::always_inline]] bool quiet() const { return P::now() == start_; }

    /// The ticks since the witness was built (or restarted).
    [[gnu::always_inline]] uint32_t ticks() const { return P::now() - start_; }

    /// Start counting again from now, for the next run.
    [[gnu::always_inline]] void restart() { start_ = P::now(); }

private:
    uint32_t start_;
};

/// The span from `start` to `now` on a counter that may read a little
/// behind (stale-low) at either end, such as a real-time counter whose
/// register lags its prescaler: a reading of `now` behind `start` is a
/// span of zero, never a span of four billion. Only for spans under 2^31
/// counts, which read as themselves; a span past that reads as zero.
/// Any context; pure arithmetic.
constexpr uint32_t since_clamped(uint32_t now, uint32_t start) {
    const uint32_t d = now - start;
    return d >= half_wrap ? 0u : d;
}

/// Whether a counter `Bits` wide can time a wait of `budget_counts`
/// counts: the budget within half its wrap, the other half left to the
/// late read (the file header, "Half the wrap"). Written where the budget
/// is chosen, as `static_assert(brio::span_fits<16>(budget))`; a narrow
/// counter's difference is taken modulo its width, `(now - start) &
/// mask`.
template <unsigned Bits>
constexpr bool span_fits(uint32_t budget_counts) {
    static_assert(Bits >= 2u && Bits <= 32u, "brio::span_fits: a counter of 2 to 32 bits");
    return budget_counts <= (uint32_t{1} << (Bits - 1u));
}

} // namespace brio
