/*
 * ticker.hpp
 *
 * Written by Uliano Guerrini; rewritten with Claude (Anthropic).
 *
 * The time of a brio program on the AVR DA, DB and EA: a counter that
 * the RTC's periodic interrupt timer (PIT) advances 1024 times a second.
 * It is the kernel's timebase - AvrPlatform::now() (avrxt/platform.hpp)
 * returns Ticker::ticks(), and every time event a program arms counts
 * these ticks - and it also gives a program milliseconds, seconds and a
 * timestamp to read. A program does two things with it: it binds the
 * PIT's interrupt vector to pit(), and it calls init() once at start-up.
 * The smallest use, as the maintained example `blink` spells it:
 *
 *     using P = brio::AvrPlatform;
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *
 *     ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init();     // RTC clock select, PIT period, its interrupt on
 *         sei();
 *         brio::Tenuto<P, Blinker, Supervisor>::run();   // blink's two active objects
 *     }
 *
 * `ISR(RTC_PIT_vect)` is avr-libc's way of writing the interrupt handler
 * of that vector; `brio::Ticker` is `BasicTicker<1024>`, the alias at the
 * bottom of this file; `init()` picks the RTC's clock, programs the PIT
 * period for 1024 Hz and enables the PIT interrupt; `sei()` lets the first
 * interrupt in; from then on every PIT interrupt runs pit(), which
 * advances the counters, and the kernel loop that run() enters reads
 * them through P (Blinker and Supervisor are blink's active objects:
 * components of a brio program with their own event queues, served one
 * event at a time by the kernel, docs/learn/introduction.md). init()
 * enables the interrupt and does not check that the vector is bound:
 * the binding is the program's. A vector left unbound is avr-libc's
 * __bad_interrupt, a `jmp 0` in the release listings, so the first tick
 * restarts the program from its first instruction, without a hardware
 * reset and without the RETI that clears CPUINT's LVL0EX, so the
 * restarted program takes no level-0 interrupt again (15.5.2, inferred,
 * not measured); a handler that does not call pit() leaves the counters
 * still, and no time event ever fires.
 *
 * Who calls what, and from where:
 *  - init() and init(source) are setup: main(), after the clock's
 *    SysClock::init() and before sei(). They reset the counters and wait
 *    for the RTC's busy flags (avrxt/rtc.hpp's rtc_wait and rtc_pit_wait,
 *    each at most 0xFFFF polls and a last read; the flags clear in about
 *    2.8 CLK_RTC periods, measured, docs/avrxt/rtc.md). A wait that runs
 *    out (CLK_RTC not running) is not reported: init() writes on;
 *  - pit() is the interrupt body, called from the program's
 *    ISR(RTC_PIT_vect) and from nowhere else; it runs to completion with
 *    no other handler interrupting it, because no vector is raised to
 *    CPUINT's level 1 (the mechanism is avrxt/platform.hpp's), and it
 *    waits for nothing;
 *  - ticks(), millis(), secs() and now() may be called from any context:
 *    each masks interrupts (ATOMIC_BLOCK with ATOMIC_RESTORESTATE, so the
 *    previous state comes back) for the few loads that copy a counter;
 *  - pause() and resume() are one store each to PITINTCTRL: the interrupt
 *    off and on, the PIT itself (PITEN) left running.
 * The tick is as accurate as CLK_RTC, which is the internal OSC32K
 * unless init() finds a 32.768 kHz crystal running (init() below says
 * what "running" takes); that oscillator's
 * error, measured per die, is in docs/avrxt/rtc.md.
 *
 * Why a monostate: a monostate is a class whose state is static, with no
 * instances. Here every data member is a `static inline` (one shared
 * copy, no instances, header-only) and every method is static, so the
 * ISR body reaches its counters with direct lds/sts on absolute
 * addresses instead of loading a pointer first, there is no lazy-init
 * branch or guard to pay for, and the class needs no translation unit
 * of its own.
 *
 * State lives in .bss (zero-initialized before main, no runtime ctor). The
 * tick frequency is a template parameter, so alternative rates are distinct
 * types checked at compile time; the canonical instantiation for this
 * project is the `Ticker` alias at the bottom of this file.
 *
 * ## Hardware foundation
 * The PIT is part of the RTC peripheral and counts a 32.768 kHz clock
 * (internal OSC32K or a 32k crystal, see init()). The silicon is driven
 * through the `Pit` and `RtcClock` resources of avrxt/rtc.hpp; this class
 * is the tick arithmetic above them, and it owns the block's one clock
 * select (the counter half, `Rtc`, shares whatever init() picked).
 * Supported tick rates map onto the PIT period dividers:
 *
 *   1024 Hz (~0.977 ms) ... 16 Hz (~62.5 ms), powers of two only.
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49 days @1024 Hz).
 *  - millis(): approximate milliseconds, long-term exact against the tick
 *              (wraps ~49.7 days); the tick itself is as accurate as
 *              CLK_RTC, above.
 *  - secs():   exact seconds (wraps ~136 years).
 *  - now():    TimeStamp = whole seconds + millisecond fraction (exact
 *              seconds; the fraction is floor(ticks*1000/tps), ~1 ms res).
 *
 * ## Millisecond correction
 * 32768 Hz does not divide into decimal milliseconds: at 1024 ticks/s each
 * tick is 0.9765625 ms, so blindly adding 1 ms per tick would gain 24 ms
 * every second. The ISR therefore skips the millis increment at positions
 * 0x00, 0x2A and 0x55 of every 128-tick window (3 skips per 128 ticks,
 * evenly spread): (1024 - 24) * 1 ms = exactly 1000 ms per second, and the
 * same average holds for every supported tick rate. millis() consequently
 * jitters by up to ~1 tick but never drifts. Position 0x00 belongs in that
 * list: it comes round at every multiple of 128 and not only at the second
 * boundaries, and leaving it unskipped gains 0.7% at 1024 Hz.
 *
 * ## Usage
 * ```cpp
 * #include "avrxt/ticker.hpp"
 *
 * ISR(RTC_PIT_vect) { brio::Ticker::pit(); }   // ISR body: update counters
 *
 * int main() {
 *     SysClock::init();                        // brio::Clock<...>, avrxt/clock.hpp
 *     brio::Ticker::init();                    // RTC clock select + PIT config
 *     sei();
 *     uint32_t t0 = brio::Ticker::millis();
 *     ...
 * }
 * ```
 */

#pragma once
#include <avr/io.h>
#include <util/atomic.h>
#include <stdint.h>
#include <bit>

#include "avrxt/rtc.hpp"
#include "util/timestamp.hpp"

namespace brio {

/**
 * Static (monostate) time tracker driven by the RTC PIT interrupt.
 * `tps` is the tick frequency in Hz: a power of two, 16..1024.
 *
 * All members are static: there is exactly one time base per program, backed
 * by the single hardware PIT. Use the `Ticker` alias below; instantiating a
 * second rate would configure the same hardware differently, so pick one.
 *
 * ## Concurrency model
 * pit() is the ISR body and runs to completion: the AVRxt core does not
 * clear the I bit on entering a handler (DS40002247B and DS40002443A
 * 7.6.3), and what keeps another handler out is CPUINT - every vector at
 * level 0, none raised to level 1, an NMI aside (avrxt/platform.hpp).
 * The public getters copy multi-byte counters inside ATOMIC_BLOCK, whose
 * cli/sei memory barriers also force the compiler to re-read memory, which
 * is why the counters do not need to be volatile.
 */
template <uint16_t tps = 1024>
class BasicTicker {
    static_assert(std::has_single_bit(tps), "ticks per second must be a power of 2");
    static_assert(tps >= 16, "ticks per second must be >= 16");
    static_assert(tps <= 1024, "ticks per second must be <= 1024");

private:
    /// Milliseconds to add per tick (for the millis() approximation)
    static constexpr uint16_t millis_per_tick = 1024 / tps;

    /// Bitmask extracting the fractional tick within a second
    static constexpr uint16_t mask = tps - 1;

    // Counters shared with the ISR (C++17 inline statics, live in .bss).
    static inline uint32_t m_ticks = 0;
    static inline uint32_t m_millis = 0;
    static inline uint32_t m_secs = 0;

    /// PIT period for the requested tick rate (compile time), counting
    /// cycles of a 32.768 kHz CLK_RTC
    static constexpr PitPeriod pit_period() {
        switch (tps) {
            case 16:   return PitPeriod::cyc2048;  // ~62.5 ms
            case 32:   return PitPeriod::cyc1024;  // ~31.25 ms
            case 64:   return PitPeriod::cyc512;   // ~15.625 ms
            case 128:  return PitPeriod::cyc256;   // ~7.813 ms
            case 256:  return PitPeriod::cyc128;   // ~3.906 ms
            case 512:  return PitPeriod::cyc64;    // ~1.953 ms
            default:   return PitPeriod::cyc32;    // 1024 -> ~0.977 ms
        }
    }

public:
    BasicTicker() = delete;  // static-only class: no instances

    /// Tick frequency in Hz (exposed for period computations)
    static constexpr uint16_t ticks_per_second = tps;

    /**
     * Configure the RTC clock source and start the PIT interrupt.
     *
     * The Ticker owns the RTC block's one clock select (rtc.hpp): it
     * takes what RtcClock::preferred() names - XOSC32K when a 32k crystal
     * was started and is still enabled and running, the internal OSC32K
     * otherwise. "Running" is XOSC32KS, which a crystal nothing requests
     * yet sets only when it was started forced on (Xosc32k's run_standby
     * argument, avrxt/clock.hpp): one started without it, or still in its
     * start-up, is passed over in silence for OSC32K. Resets all counters, waits for the RTC's pending writes
     * (bounded, the answer not checked), programs the PIT period for
     * ticks_per_second and enables its interrupt. The RTC counter half
     * (brio::Rtc) is left alone and shares whatever source is selected
     * here.
     *
     * Setup: call once in main(), after clock init and before sei(). The
     * RTC_PIT_vect ISR must call pit().
     */
    static void init() { init(RtcClock::preferred()); }

    /// The same, with the CLK_RTC source named: an application that
    /// knows its board (a crystal, an external clock, the 1.024 kHz
    /// OSC1K for a slow tick) says so instead of letting init() guess.
    /// The tick rate then follows the source: tps counts cycles of a
    /// 32.768 kHz CLK_RTC, so OSC1K divides every rate by 32 - and
    /// ticks_per_second, millis() and secs() do not follow: they keep
    /// counting as if CLK_RTC were 32.768 kHz, so kernel time runs 32
    /// times slow on OSC1K.
    static void init(RtcSource source) {
        m_ticks = 0;
        m_millis = 0;
        m_secs = 0;

        (void)Rtc::sync();            // no configuration write in flight
        RtcClock::select(source);
        Pit::init(pit_period());
    }

    /**
     * PIT interrupt body - call from ISR(RTC_PIT_vect) and from nowhere
     * else.
     *
     * Clears the interrupt flag, advances the tick counter, then derives
     * seconds (exact) and milliseconds (skip-corrected, see file header).
     */
    // always_inline: an ISR body has exactly one call site (the vector
    // binding in the app), so inlining costs no flash and lets the compiler
    // save only the registers actually used instead of the full ABI
    // call-clobbered set (measured on this function: 8 pushes vs 16).
    // Survives the -fno-inline debug profile like _delay_ms does.
    [[gnu::always_inline]] static void pit() {
        Pit::clear_flag();

        ++m_ticks;
        const uint16_t low = static_cast<uint16_t>(m_ticks);

        if ((low & mask) == 0) {
            ++m_secs;  // full second boundary
        }

        // Skip 3 positions per 128-tick window so millis() averages exactly
        // 1000 ms per second (second boundaries land on position 0x00 too).
        const uint8_t pos = low & 0x7F;
        if (pos != 0x00 && pos != 0x2A && pos != 0x55) {
            m_millis += millis_per_tick;
        }
    }

    /// Stop the periodic interrupt without losing the counters: for the
    /// rare moments when the CPU runs too slowly to serve a 1024 Hz tick
    /// (a 32 kHz main clock: the ISR would never return). Time stands
    /// still while paused; resume() picks up from where it was.
    static void pause() { Pit::enable_interrupt(false); }
    static void resume() { Pit::enable_interrupt(true); }

    /// Current timestamp: whole seconds + fractional ticks - the most
    /// precise representation (no millis jitter); seconds and ticks are
    /// read atomically so they belong to the same instant.
    static void now(TimeStamp &out) {
        uint16_t frac;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            out.seconds = m_secs;
            frac = static_cast<uint16_t>(m_ticks) & mask;
        }
        // tick fraction -> milliseconds (0..999): TimeStamp is
        // target-independent, the tick unit is not (see util/timestamp.hpp).
        out.millis = static_cast<uint16_t>(
            (static_cast<uint32_t>(frac) * 1000u) / tps);
    }

    /// Raw 32-bit tick count since init() (wraps: 49 days @ 1024 Hz);
    /// what AvrPlatform::now() returns. Any context.
    static uint32_t ticks() {
        uint32_t tmp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tmp = m_ticks;
        }
        return tmp;
    }

    /// Approximate milliseconds since init(): ~1 tick jitter, no long-term
    /// drift (wraps ~49.7 days)
    static uint32_t millis() {
        uint32_t tmp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tmp = m_millis;
        }
        return tmp;
    }

    /// Exact seconds since init() (wraps after ~136 years)
    static uint32_t secs() {
        uint32_t tmp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tmp = m_secs;
        }
        return tmp;
    }
};

/// The project-wide time base: 1024 ticks/s (~0.977 ms resolution).
/// Change the rate here; everything (time events, apps) follows the alias.
using Ticker = BasicTicker<1024>;

} // namespace brio
