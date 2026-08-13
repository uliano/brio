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
 * (internal OSC32K or a 32k crystal, see init()). Supported tick rates map
 * onto the PIT period dividers:
 *
 *   1024 Hz (~0.977 ms) ... 16 Hz (~62.5 ms), powers of two only.
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49 days @1024 Hz).
 *  - millis(): approximate milliseconds, long-term exact (wraps ~49.7 days).
 *  - secs():   exact seconds (wraps ~136 years).
 *  - now():    TimeStamp = whole seconds + fractional ticks (no jitter).
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
 * #include "ticker.hpp"
 *
 * ISR(RTC_PIT_vect) { brio::Ticker::pit(); }   // ISR body: update counters
 *
 * int main() {
 *     brio::init_clock_24mhz();                // or init_clocks()
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

namespace brio {

/**
 * @struct TimeStamp
 * @brief High-precision timestamp: whole seconds + fractional ticks
 *
 * fraction = ticks / ticks_per_second. Unlike millis(), this representation
 * carries no jitter from the decimal-millisecond correction.
 */
struct TimeStamp {
    uint32_t seconds;  ///< Whole seconds elapsed (wraps after ~136 years)
    uint16_t ticks;    ///< Fractional second in ticks (0 .. ticks_per_second-1)
};

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

    /// PIT period divider bits for the requested tick rate (compile time)
    static constexpr uint8_t pit_period_bits() {
        switch (tps) {
            case 16:   return RTC_PERIOD_CYC2048_gc;  // ~62.5 ms
            case 32:   return RTC_PERIOD_CYC1024_gc;  // ~31.25 ms
            case 64:   return RTC_PERIOD_CYC512_gc;   // ~15.625 ms
            case 128:  return RTC_PERIOD_CYC256_gc;   // ~7.813 ms
            case 256:  return RTC_PERIOD_CYC128_gc;   // ~3.906 ms
            case 512:  return RTC_PERIOD_CYC64_gc;    // ~1.953 ms
            default:   return RTC_PERIOD_CYC32_gc;    // 1024 -> ~0.977 ms
        }
    }

public:
    BasicTicker() = delete;  // static-only class: no instances

    /// Tick frequency in Hz (exposed for period computations)
    static constexpr uint16_t ticks_per_second = tps;

    /**
     * @brief Configure the RTC clock source and start the PIT interrupt
     *
     * Selects XOSC32K when the clock init found a running 32k crystal
     * (MCLKSTATUS flag), the internal OSC32K otherwise; resets all counters;
     * programs the PIT period for ticks_per_second and enables its interrupt.
     *
     * Call once after clock init and before sei(). The RTC_PIT_vect ISR
     * must call pit().
     */
    static void init() {
        m_ticks = 0;
        m_millis = 0;
        m_secs = 0;

        while (RTC.STATUS > 0) {}  // wait for register synchronization

        // 32k crystal if the clock init started one, internal OSC32K otherwise.
        if (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm) {
            RTC.CLKSEL = RTC_CLKSEL_XOSC32K_gc;
        } else {
            RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;
        }

        while (RTC.PITSTATUS > 0) {}  // wait for PIT register synchronization
        RTC.PITCTRLA = pit_period_bits() | RTC_PITEN_bm;
        RTC.PITINTCTRL = RTC_PI_bm;   // enable the periodic interrupt
    }

    /**
     * @brief PIT interrupt body - call from ISR(RTC_PIT_vect)
     *
     * Clears the interrupt flag, advances the tick counter, then derives
     * seconds (exact) and milliseconds (skip-corrected, see file header).
     */
    static void pit() {
        RTC.PITINTFLAGS = RTC_PI_bm;  // clear interrupt flag

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
    static void now(TimeStamp &out) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            out.seconds = m_secs;
            out.ticks = static_cast<uint16_t>(m_ticks) & mask;
        }
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
/// Change the rate here; everything (Timer<>, apps) follows the alias.
using Ticker = BasicTicker<1024>;

} // namespace brio
