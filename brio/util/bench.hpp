/*
 * bench.hpp
 *
 * The benchmark grammar: what a bench_<family> app prints, one line per
 * operation and size, the same on every family, so that a number read on
 * one chip means what the same number means on another and the vendor's
 * library doing the same operation on the same board prints a line the
 * same eyes read.
 *
 *     bench <op> n=<bytes> wall=<cycles> busy=<cycles> irq=<count> isr=<cycles> rate=<B/s> wire=<B/s> x=<wall/wire>
 *
 *  - wall: ruler cycles from the operation's start to its completion,
 *    the idle included.
 *  - busy: the cycles the core was not idle (below).
 *  - irq: the interrupts the operation took.
 *  - isr: the cycles spent inside their handlers, a part of busy that
 *    is told apart because it is the number a transport's shape decides
 *    (one handler per byte, per FIFO level, per block) where busy also
 *    holds the thread's own spin on a blocking print.
 *  - rate: bytes per second achieved, from wall.
 *  - wire: bytes per second the configured medium allows - the app
 *    states it (bits per byte over the line rate for a transport, the
 *    bus for a memory operation) and says why.
 *  - x: wall over the wire's time. The number the documents carried as a
 *    fact and overview.md's rules now call a finding above about 1.5:
 *    the harness prints it so that no measurement is filed unexplained.
 *
 * NOT A TEST. The only verdict of a bench letter is that it ran;
 * TestBench still frames the app (the menu, the ALL: line bin/brio waits
 * for), and the bench lines are the payload between the verdicts.
 *
 * THE RULER. A CycleRuler is a free-running count of CORE cycles,
 * wrapping at 2^32, readable from any context, and it states its own
 * rate because the counters differ by core: SysTick's cycles() on the
 * M0+ parts and the M4 kept to its DWT, the system timer counting
 * clk_sys on the RP2350 (one ruler for both of its instruction sets),
 * the STK on the QingKe parts, two cascaded TCBs on the AVR. A
 * difference of two reads is exact under 2^32 cycles, which bounds one
 * measurement at about 90 s at 48 MHz and 24 s at 180 MHz.
 *
 * BUSY. The core is busy when it is not idle, and the kernel's idle path
 * is one call the platform owns - so BenchIdle<P, R> is a platform that
 * forwards everything to P and stamps the ruler around P::idle(), the
 * kernel being templated on the platform and compiling nothing new. An
 * idle window measured that way holds the interrupt that ended it: its
 * handler ran inside the window, so its cycles are not idle and must be
 * added back; a handler that preempted the thread ran inside wall and
 * outside every window, and is already counted. IsrMeter<R, I> is the
 * stamp pair a bound vector carries, keeping the two kinds apart by
 * asking the idle adapter whether a window is open:
 *
 *     busy = wall - idle_windows + handler_cycles_inside_them
 *
 * THE COST OF THE INSTRUMENT. A stamp pair is two ruler reads and the
 * arithmetic (SysTick's cycles() is five register reads and a compose,
 * some 30 to 40 cycles on an M0+), and an idle window costs the same:
 * the bench app's letter r measures both and prints them once per run,
 * and every bench line carries RAW numbers. Two seams are stated rather
 * than closed: the latency of entering and leaving the handler (a few
 * cycles either side of the stamps) is counted as idle, and an
 * interrupt that lands between P::idle()'s return and the window's close
 * is counted in both - a window of a few cycles once per idle turn.
 *
 * THE COUNTERS ARE READ QUIESCENT. bench_counters() reads the idle
 * adapter's and the meters' accumulators with no guard: a bench app
 * reads them before the operation starts and after it has completed,
 * when nothing of the operation is still in flight. On a core whose
 * atomic_width is below 4 the app takes the counters under its
 * platform's CriticalSection, as it would any 32-bit value an ISR
 * writes.
 */
#pragma once
#include <stdint.h>
#include <concepts>
#include <optional>
#include "kernel/platform.hpp"
#include "util/print.hpp"

namespace brio {

/// A free-running count of core cycles, wrapping at 2^32, read from any
/// context its own header admits (a ticker's cycles() composed from the
/// tick count is one period low inside the tick handler's own body - the
/// meter on THAT vector takes the counter's position instead); hz() is
/// the rate it counts at (a Clock's hz, or the rate a dynamic clock
/// stands at when the line is printed).
template <typename R>
concept CycleRuler = requires {
    { R::now() } -> std::same_as<uint32_t>;
    { R::hz() } -> std::same_as<uint32_t>;
};

namespace bench_detail {

/// The optional members of a platform that is one core of several
/// (kernel/platform.hpp), carried through the adapter where P has them
/// and absent otherwise, so the kernel's detection sees P's answer.
template <typename P>
struct BenchIdleCores {};

template <typename P>
    requires requires {
        typename P::Doorbell;
        { P::on_own_core() } -> std::same_as<bool>;
    }
struct BenchIdleCores<P> {
    using Doorbell = typename P::Doorbell;
    static bool on_own_core() { return P::on_own_core(); }
};

/// The idle path's hook a platform may offer (a low-power state that is
/// not a sleep instruction): the adapter's name is P's own variable, so a
/// sleep site that writes it through the adapter arms P.
template <typename P>
struct BenchIdleHook {};

template <typename P>
    requires requires { P::sleep_hook = nullptr; }
struct BenchIdleHook<P> {
    static inline void (*&sleep_hook)() = P::sleep_hook;
};

} // namespace bench_detail

/// Elapsed cycles since start(), exact under 2^32 (the unsigned wrap).
/// Both verbs are forced inline: a Stopwatch line measures the ruler's
/// read and nothing else, not a call and a return -Os would add. Both
/// are ORDERING POINTS: the barrier keeps the measured code between the
/// two reads, where the compiler would otherwise move a load above the
/// first or a store below the second (measured on the M33 and Hazard3).
template <CycleRuler R>
class Stopwatch {
public:
    [[gnu::always_inline]] void start() {
        start_ = R::now();
        asm volatile("" ::: "memory");
    }
    [[gnu::always_inline]] uint32_t elapsed() const {
        asm volatile("" ::: "memory");
        return R::now() - start_;
    }

private:
    uint32_t start_ = 0;
};

/// What a handler meter asks of the idle adapter: is a window open?
template <typename I>
concept IdleWindow = requires {
    { I::in_idle() } -> std::same_as<bool>;
};

/// The idle adapter of a program that never idles (a polled loop with
/// no kernel): no window is ever open, every handler preempts the thread.
struct NoIdleWindow {
    static bool in_idle() { return false; }
};

/// The platform a bench app names: P with its idle path stamped. The
/// kernel sees a Platform - and the optional members exactly where P
/// offers them: idle_until(), on_own_core() and Doorbell, sleep_hook -
/// the app sees the windows' total and their count.
template <Platform P, CycleRuler R>
struct BenchIdle : bench_detail::BenchIdleCores<P>, bench_detail::BenchIdleHook<P> {
    using CriticalSection = typename P::CriticalSection;
    static constexpr auto ticks_per_second = P::ticks_per_second;
    static constexpr auto atomic_width = P::atomic_width;

    static void break_here() { P::break_here(); }
    static uint32_t now() { return P::now(); }
    static PanicRecord& panic_record() { return P::panic_record(); }

    /// Called with interrupts masked, as P::idle() is; the window opens
    /// before the ruler is read and closes after, so the window holds
    /// the sleep, the wake and the handler that caused it.
    static void idle() {
        in_idle_ = true;
        const uint32_t t0 = R::now();
        P::idle();
        close_window(t0);
    }

    /// The tickless door, forwarded where P has it and absent otherwise,
    /// so the kernel's detection sees exactly what it would see on P.
    static void idle_until(std::optional<uint32_t> deadline)
        requires requires(std::optional<uint32_t> d) { P::idle_until(d); }
    {
        in_idle_ = true;
        const uint32_t t0 = R::now();
        P::idle_until(deadline);
        close_window(t0);
    }

    static bool in_idle() { return in_idle_; }
    /// Cycles spent inside idle windows since the program started.
    static uint32_t idle_cycles() { return idle_cycles_; }
    /// How many windows there were.
    static uint32_t idle_turns() { return idle_turns_; }

private:
    [[gnu::always_inline]] static void close_window(uint32_t t0) {
        idle_cycles_ += R::now() - t0;
        ++idle_turns_;
        in_idle_ = false;
    }

    static inline volatile bool in_idle_ = false;  ///< written by the loop, read by handlers
    static inline uint32_t idle_cycles_ = 0;
    static inline uint32_t idle_turns_ = 0;
};

/// The stamp pair a bound vector carries: enter() first, leave() last.
/// A handler body runs to completion and no interrupt nests over another
/// on any target (design/kernel.md section 1), so one start stamp serves
/// every vector that shares the meter.
template <CycleRuler R, IdleWindow I = NoIdleWindow>
class IsrMeter {
public:
    /// The barrier after the read keeps the body's loads below the stamp:
    /// without it the compiler schedules them above the ruler's read, and
    /// a few cycles of the body are not metered (measured on the M4).
    [[gnu::always_inline]] void enter() {
        t0_ = R::now();
        asm volatile("" ::: "memory");
    }

    /// The barrier before the read keeps the body's stores above it.
    [[gnu::always_inline]] void leave() {
        asm volatile("" ::: "memory");
        const uint32_t d = R::now() - t0_;
        ++count_;
        cycles_ += d;
        if (I::in_idle()) {
            in_idle_ += d;
        }
    }

    uint32_t count() const { return count_; }
    uint32_t cycles() const { return cycles_; }
    /// The part of cycles() spent inside idle windows: what busy adds back.
    uint32_t cycles_in_idle() const { return in_idle_; }

private:
    uint32_t t0_ = 0;
    uint32_t count_ = 0;
    uint32_t cycles_ = 0;
    uint32_t in_idle_ = 0;
};

/// The accumulators at one instant: the idle adapter's windows and the
/// meters' sums. Two of these, before and after, make a sample.
struct BenchCounters {
    uint32_t idle_cycles;
    uint32_t isr_cycles;
    uint32_t isr_in_idle;
    uint32_t irq;
};

/// Read the counters quiescent (the header's rule); `Idle` is the bench
/// app's BenchIdle, the meters its IsrMeters.
template <typename Idle, typename... Meters>
BenchCounters bench_counters(const Meters&... m) {
    return {Idle::idle_cycles(), (0u + ... + m.cycles()), (0u + ... + m.cycles_in_idle()),
            (0u + ... + m.count())};
}

/// One operation's numbers.
struct BenchSample {
    uint32_t wall;
    uint32_t busy;
    uint32_t irq;
    uint32_t isr;
};

/// The busy formula over a wall and two readings of the counters.
constexpr BenchSample bench_sample(uint32_t wall, const BenchCounters& before,
                                   const BenchCounters& after) {
    const uint32_t idle = after.idle_cycles - before.idle_cycles;
    const uint32_t added_back = after.isr_in_idle - before.isr_in_idle;
    return {wall, wall - idle + added_back, after.irq - before.irq,
            after.isr_cycles - before.isr_cycles};
}

/// The line. `hz` is the ruler's rate, `wire_bps` the medium's limit in
/// bytes per second; rate and x are computed here and nowhere else.
template <ByteSink S>
void bench_line(S s, const char* op, uint32_t bytes, const BenchSample& m, uint32_t hz,
                uint32_t wire_bps) {
    print(s, "bench ", op, " n=", bytes, " wall=", m.wall, " busy=", m.busy, " irq=", m.irq,
          " isr=", m.isr, " rate=");
    if (m.wall == 0u) {
        print(s, "-");
    } else {
        const uint64_t rate = static_cast<uint64_t>(bytes) * hz / m.wall;
        print(s, static_cast<uint32_t>(rate));
    }
    print(s, " wire=", wire_bps, " x=");
    const uint64_t wire_cycles =
        wire_bps == 0u ? 0u : static_cast<uint64_t>(bytes) * hz / wire_bps;
    if (wire_cycles == 0u) {
        print(s, "-", crlf);
        return;
    }
    const uint64_t x100 = static_cast<uint64_t>(m.wall) * 100u / wire_cycles;
    const uint32_t whole = static_cast<uint32_t>(x100 / 100u);
    const uint8_t hundredths = static_cast<uint8_t>(x100 % 100u);
    print(s, whole, '.', static_cast<char>('0' + hundredths / 10u),
          static_cast<char>('0' + hundredths % 10u), crlf);
}

} // namespace brio
