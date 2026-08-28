/**
 * @file ticker.hpp
 * @author uliano
 * @brief Time tracking on the AVR RTC Periodic Interrupt Timer (PIT)
 * @date Created on April 26, 2024; rewritten as a static class on 07/21/2026
 *
 * Rewrite of the AVR-Multislope singleton Ticker as a "monostate" class
 * template: every data member is a C++17 `static inline` (one shared copy,
 * no instances, header-only) and every method is static. Compared to the
 * previous `Ticker::instance()` / `Ticker::ptr` singleton this removes:
 *
 *  - the global pointer indirection in the ISR (the compiler emits direct
 *    lds/sts on absolute addresses instead of loading a pointer first),
 *  - the Meyers-singleton lazy-init branch and guard,
 *  - the separate ticker.cpp translation unit.
 *
 * State lives in .bss (zero-initialized before main, no runtime ctor). The
 * tick frequency is a template parameter, so alternative rates are distinct
 * types checked at compile time; the canonical instantiation for this
 * project is the `Ticker` alias at the bottom of this file.
 *
 * ## Hardware foundation
 * The PIT is part of the RTC peripheral and counts a 32.768 kHz clock
 * (internal OSC32K or a 32k crystal, see init()). The silicon is driven
 * through the `Pit` and `RtcClock` resources of avrdx/rtc.hpp; this class
 * is the tick arithmetic above them, and it OWNS the block's one clock
 * select (the counter half, `Rtc`, shares whatever init() picked).
 * Supported tick rates map onto the PIT period dividers:
 *
 *   1024 Hz (~0.977 ms) ... 16 Hz (~62.5 ms), powers of two only.
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49 days @1024 Hz).
 *  - millis(): approximate milliseconds, long-term exact (wraps ~49.7 days).
 *  - secs():   exact seconds (wraps ~136 years).
 *  - now():    TimeStamp = whole seconds + millisecond fraction (exact
 *              seconds; the fraction is floor(ticks*1000/tps), ~1 ms res).
 *
 * ## Millisecond correction
 * 32768 Hz does not divide into decimal milliseconds: at 1024 ticks/s each
 * tick is 0.9765625 ms, so blindly adding 1 ms per tick would gain 24 ms
 * every second. The ISR therefore SKIPS the millis increment at positions
 * 0x00, 0x2A and 0x55 of every 128-tick window (3 skips per 128 ticks,
 * evenly spread): (1024 - 24) * 1 ms = exactly 1000 ms per second, and the
 * same average holds for every supported tick rate. millis() consequently
 * jitters by up to ~1 tick but never drifts.
 *
 * NOTE - two bugs of the original AVR-Multislope implementation are fixed:
 *  1. the H/L split of the tick counter was declared in the wrong order for
 *     a little-endian layout, so ticks() interleaved its words (the counter
 *     effectively advanced by 65536 per tick); the union is gone, the ISR
 *     now increments the plain 32-bit counter.
 *  2. position 0x00 of the 128-tick window was NOT skipped (the code assumed
 *     it could not occur, but it does at every multiple of 128 that is not a
 *     second boundary), so millis() gained 0.7% at 1024 Hz.
 *
 * ## Usage
 * ```cpp
 * #include "avrdx/ticker.hpp"
 *
 * ISR(RTC_PIT_vect) { brio::Ticker::pit(); }   // ISR body: update counters
 *
 * int main() {
 *     SysClock::init();                        // brio::Clock<...>, avrdx/clock.hpp
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

#include "avrdx/rtc.hpp"
#include "util/timestamp.hpp"

namespace brio {

/**
 * @class BasicTicker
 * @brief Static (monostate) time tracker driven by the RTC PIT interrupt
 * @tparam tps Tick frequency in Hz: power of two, 16..1024
 *
 * All members are static: there is exactly one time base per program, backed
 * by the single hardware PIT. Use the `Ticker` alias below; instantiating a
 * second rate would configure the same hardware differently, so pick one.
 *
 * ## Concurrency model
 * pit() is the ISR body and runs with interrupts disabled (AVR default).
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
     * @brief Configure the RTC clock source and start the PIT interrupt
     *
     * The Ticker OWNS the RTC block's one clock select (rtc.hpp): it
     * takes XOSC32K when the clock init found a running 32k crystal, the
     * internal OSC32K otherwise. Resets all counters, programs the PIT
     * period for ticks_per_second and enables its interrupt. The RTC
     * counter half (brio::Rtc) is left alone and shares whatever source
     * is selected here.
     *
     * Call once after clock init and before sei(). The RTC_PIT_vect ISR
     * must call pit().
     */
    static void init() { init(RtcClock::preferred()); }

    /// The same, with the CLK_RTC source named: an application that
    /// knows its board (a crystal, an external clock, the 1.024 kHz
    /// OSC1K for a slow tick) says so instead of letting init() guess.
    /// The tick rate then follows the source: tps counts cycles of a
    /// 32.768 kHz CLK_RTC, so OSC1K divides every rate by 32.
    static void init(RtcSource source) {
        m_ticks = 0;
        m_millis = 0;
        m_secs = 0;

        (void)Rtc::sync();            // no configuration write in flight
        RtcClock::select(source);
        Pit::init(pit_period());
    }

    /**
     * @brief PIT interrupt body - call from ISR(RTC_PIT_vect)
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

    /**
     * @brief Current timestamp: whole seconds + fractional ticks
     *
     * The most precise representation (no millis jitter); seconds and ticks
     * are read atomically so they belong to the same instant.
     */
    /// Stop the periodic interrupt without losing the counters: for the
    /// rare moments when the CPU runs too slowly to serve a 1024 Hz tick
    /// (a 32 kHz main clock: the ISR would never return). Time stands
    /// still while paused; resume() picks up from where it was.
    static void pause() { Pit::enable_interrupt(false); }
    static void resume() { Pit::enable_interrupt(true); }

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

    /// Raw 32-bit tick count since init() (wraps: 49 days @ 1024 Hz)
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
