// Ported from uliano/AVR-Multislope (lib/core/src/ticker.hpp).
/**
 * @file ticker.hpp
 * @author uliano
 * @brief Time tracking system using AVR RTC Periodic Interrupt Timer
 * @date Created on April 26, 2024, 9:01 AM
 * @revised 01/9/2026
 *
 * This file implements a singleton-based time tracking system that provides multiple
 * time representations with different trade-offs between precision, range, and stability.
 *
 * ## Hardware Foundation
 * The system is built on the AVR's PIT (Periodic Interrupt Timer) which is part of the RTC
 * peripheral and ALWAYS runs at 32.768 kHz (standard watch crystal frequency). The PIT can
 * generate interrupts at these periods:
 * - CYC32   (32 cycles):   ~0.977 ms  (1024 ticks/sec)
 * - CYC64   (64 cycles):   ~1.953 ms  (512 ticks/sec)
 * - CYC128  (128 cycles):  ~3.906 ms  (256 ticks/sec)
 * - CYC256  (256 cycles):  ~7.813 ms  (128 ticks/sec)
 * - CYC512  (512 cycles):  ~15.625 ms (64 ticks/sec)
 * - CYC1024 (1024 cycles): ~31.25 ms  (32 ticks/sec)
 * - CYC2048 (2048 cycles): ~62.5 ms   (16 ticks/sec)
 *
 * ## Time Representations
 * The system provides four different time representations:
 *
 * 1. **ticks()** - Raw 32-bit tick counter
 *    - Wraps: 49 days (1024 ticks/sec) to 8.5 years (16 ticks/sec)
 *    - Use for: High-precision intervals when `ticks_per_second` is known
 *    - Changes behavior when `ticks_per_second` is modified
 *
 * 2. **millis()** - Approximate millisecond counter (32-bit)
 *    - Wraps: ~49.7 days
 *    - Use for: Short-term timing, delays, timeouts
 *    - Advantage: Stable across `ticks_per_second` changes
 *    - Note: Not exact due to 32.768 kHz clock frequency mismatch with decimal
 *    - Special jitter compensation skips 3 ticks per 128 to maintain accuracy
 *
 * 3. **secs()** - Second counter (32-bit)
 *    - Wraps: ~136 years
 *    - Use for: Long-term timing, timestamping events
 *    - Most stable for durations > 1 second
 *
 * 4. **TimeStamp** - Composite seconds + fractional ticks
 *    - No practical wrap limit
 *    - Most precise representation
 *    - Use for: Data logging, event timestamping requiring maximum accuracy
 *    - No jitter from millisecond correction
 *
 * ## Usage Pattern
 * ```cpp
 * // 1. Initialize RTC clock source (in main or startup code)
 * init_ticker();  // Sets up RTC to use 32.768 kHz oscillator and initializes Ticker
 *
 * // 2. Access time from anywhere via the global pointer
 * uint32_t current_time = Ticker::ptr->millis();
 *
 * // 3. In ISR (must be defined by user):
 * ISR(RTC_PIT_vect) {
 *     Ticker::ptr->pit();  // Updates all time counters
 * }
 * ```
 *
 * @note The init_ticker() function handles both RTC setup and Ticker initialization,
 *       setting Ticker::ptr to enable ISR access.
 */

#pragma once
#include <avr/io.h>
#include <util/atomic.h>
#include "utils.hpp"

/**
 * @brief Initialize the RTC peripheral and Ticker singleton
 *
 * This function:
 * 1. Waits for RTC register synchronization
 * 2. Configures RTC to use the 32.768 kHz oscillator (OSC32K)
 * 3. Creates and initializes the Ticker singleton
 * 4. Sets up Ticker::ptr for ISR access
 * 5. Enables the PIT interrupt
 *
 * @note Must be called once during system initialization, before enabling global interrupts
 */
void init_ticker(void);

/**
 * @struct TimeStamp
 * @brief High-precision timestamp with seconds and fractional tick components
 *
 * This structure provides the most accurate time representation by separating
 * the integer seconds from the fractional part expressed in ticks.
 *
 * The `ticks` field represents the fractional second as:
 * fraction = ticks / ticks_per_second
 *
 * For example, with ticks_per_second=1024:
 * - ticks=512 represents 0.5 seconds
 * - ticks=256 represents 0.25 seconds
 *
 * Unlike millis(), this representation doesn't accumulate jitter from
 * the decimal conversion corrections.
 */
typedef struct {
    uint32_t seconds;  ///< Whole seconds elapsed (wraps after ~136 years)
    uint16_t ticks;    ///< Fractional second in ticks (0 to ticks_per_second-1)
} TimeStamp;


/**
 * @class Ticker
 * @brief Singleton class providing system-wide time tracking via RTC PIT interrupts
 *
 * This class maintains multiple synchronized time counters updated from the
 * RTC Periodic Interrupt Timer running at 32.768 kHz.
 *
 * ## Configuration
 * The `ticks_per_second` constant determines the PIT interrupt frequency:
 * - Higher values (512, 1024): Better time resolution, more CPU overhead
 * - Lower values (16, 32): Lower resolution, less CPU overhead
 *
 * ## Thread Safety
 * All public accessor methods use ATOMIC_BLOCK to ensure consistent reads
 * of multi-byte values that can be modified by the ISR.
 *
 * ## Millisecond Correction Algorithm
 * The 32.768 kHz clock doesn't divide evenly into milliseconds. To compensate:
 * - At 1024 ticks/sec: each tick should be ~0.9765625 ms (not 1 ms)
 * - The pit() method skips millisecond increments at specific tick positions
 * - This creates slight jitter in millis() but maintains long-term accuracy
 * - The secs() counter is always precise (no jitter)
 *
 * @note This is a singleton - use Ticker::instance() or Ticker::ptr to access
 */
class Ticker {

    // ============================================================================
    // CONFIGURATION: Change tick frequency here
    // ============================================================================
    // To modify the system tick rate, change the value below.
    // Valid values: 16, 32, 64, 128, 256, 512, 1024 (must be power of 2)
    //
    // Higher values = better time resolution but more CPU overhead
    // Lower values  = lower resolution but less interrupt overhead
    //
    // Examples:
    //   1024 Hz = ~0.977 ms per tick  (default, maximum resolution)
    //    512 Hz = ~1.953 ms per tick
    //    256 Hz = ~3.906 ms per tick
    //    128 Hz = ~7.813 ms per tick
    //     64 Hz = ~15.625 ms per tick
    //     32 Hz = ~31.25 ms per tick
    //     16 Hz = ~62.5 ms per tick (minimum overhead)
    // ============================================================================
    static constexpr uint16_t ticks_per_second = 1024;
    // ============================================================================

    static_assert(is_powerof2(ticks_per_second), "ticks_per_second must be a power of 2");
    static_assert(ticks_per_second >= 16, "ticks_per_second must be >= 16");
    static_assert(ticks_per_second <= 1024, "ticks_per_second must be <= 1024");

    private:

    /// Milliseconds to add per tick (for millis() approximation)
    static constexpr uint16_t millis_per_tick = 1024 / ticks_per_second;

    /// Bitmask to extract fractional tick within a second (ticks_per_second - 1)
    static constexpr uint16_t mask = ticks_per_second - 1;

    uint32_t m_secs;    ///< Seconds counter (wraps ~136 years)
    uint32_t m_millis;  ///< Approximate milliseconds counter (wraps ~49.7 days)

    /**
     * @brief Union for efficient 32-bit tick counter access
     *
     * Allows atomic update of lower 16 bits in ISR while also providing
     * full 32-bit access for reading. The ISR increments m_ticksL and
     * checks for overflow to increment m_ticksH.
     */
    union {
        uint32_t m_ticks;   ///< Full 32-bit tick counter
        struct {
            uint16_t m_ticksH;  ///< High 16 bits of tick counter
            uint16_t m_ticksL;  ///< Low 16 bits of tick counter (updated in ISR)
        };
    };

    public:

    /// Global pointer for fast ISR access (set by init())
    static inline Ticker* ptr = nullptr;

    /**
     * @brief Get the singleton instance of Ticker
     * @return Reference to the global Ticker instance
     *
     * This follows the Meyers' Singleton pattern. The static instance is
     * created on first call and persists for the program's lifetime.
     *
     * @note Prefer using Ticker::ptr for ISR access (faster, no function call overhead)
     */
    static Ticker& instance(void) {
        static Ticker global_instance;
        return global_instance;
    }

    /**
     * @brief Initialize the Ticker and configure the RTC PIT
     *
     * This method:
     * 1. Sets Ticker::ptr for fast ISR access
     * 2. Resets all time counters to zero
     * 3. Waits for RTC peripheral synchronization
     * 4. Configures PIT period based on ticks_per_second (compile-time selection)
     * 5. Enables the PIT interrupt
     *
     * The PIT period is selected via compile-time if-constexpr to generate
     * optimal code with no runtime overhead.
     *
     * @note Must be called after RTC clock source is configured (see init_ticker())
     *       and before enabling global interrupts
     */
    inline void init() {
        ptr = this;  // Enable ISR access via global pointer
        m_ticks = 0;
        m_secs = 0;
        m_millis = 0;

        while (RTC.PITSTATUS > 0);  // Wait for register synchronization

        // Compile-time selection of PIT period based on ticks_per_second
        if constexpr (ticks_per_second == 16) {
            RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc | 1 << RTC_PITEN_bp;  // ~62.5 ms
        }
        if constexpr (ticks_per_second == 32) {
            RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | 1 << RTC_PITEN_bp;  // ~31.25 ms
        }
        if constexpr (ticks_per_second == 64) {
            RTC.PITCTRLA = RTC_PERIOD_CYC512_gc | 1 << RTC_PITEN_bp;   // ~15.625 ms
        }
        if constexpr (ticks_per_second == 128) {
            RTC.PITCTRLA = RTC_PERIOD_CYC256_gc | 1 << RTC_PITEN_bp;   // ~7.813 ms
        }
        if constexpr (ticks_per_second == 256) {
            RTC.PITCTRLA = RTC_PERIOD_CYC128_gc | 1 << RTC_PITEN_bp;   // ~3.906 ms
        }
        if constexpr (ticks_per_second == 512) {
            RTC.PITCTRLA = RTC_PERIOD_CYC64_gc | 1 << RTC_PITEN_bp;    // ~1.953 ms
        }
        if constexpr (ticks_per_second == 1024) {
            RTC.PITCTRLA = RTC_PERIOD_CYC32_gc | 1 << RTC_PITEN_bp;    // ~0.977 ms
        }

        RTC.PITINTCTRL = 1 << RTC_PI_bp;  // Enable periodic interrupt
    }

    /**
     * @brief PIT interrupt handler - must be called from RTC_PIT_vect ISR
     *
     * This method updates all time counters and implements the millisecond
     * jitter correction algorithm. It should be the ONLY code in your ISR:
     *
     * ```cpp
     * ISR(RTC_PIT_vect) {
     *     Ticker::ptr->pit();
     * }
     * ```
     *
     * ## Update Sequence
     * 1. Clear the PIT interrupt flag
     * 2. Increment tick counter (m_ticksL)
     * 3. If a full second elapsed: increment m_secs
     * 4. Otherwise: update m_millis with jitter correction
     * 5. Handle tick counter overflow (m_ticksL -> m_ticksH)
     *
     * ## Millisecond Jitter Correction
     * To convert 32.768 kHz ticks to decimal milliseconds accurately:
     * - With 1024 ticks/sec: real period is 0.9765625 ms, not 1.0 ms
     * - Without correction: 1 second = 1024 ms (2.4% error!)
     * - Solution: Skip millis increment at positions 0x2A (42) and 0x55 (85)
     *            within each 128-tick window
     * - Result: Skips 3 ticks per 128, maintaining ~1000ms per second
     *
     * The algorithm checks the lower 7 bits of the tick counter to identify
     * these skip positions, ensuring millis() stays synchronized with real time.
     *
     * @note This method is designed for minimal ISR execution time
     * @note The tick counter never misses a count - only millis has corrections
     */
    inline void pit(void) {
        RTC.PITINTFLAGS = RTC_PI_bm;  // Clear interrupt flag

        ++m_ticksL;  // Increment low 16 bits of tick counter

        if ((m_ticksL & mask) == 0) {
            // Full second boundary reached
            ++m_secs;
        } else {
            // Fractional second: update millis with jitter correction
            // Skip 3 ticks per 128 to maintain decimal millisecond accuracy
            uint8_t tmp = m_ticksL & 0x7F;  // Position within 128-tick window
            if (tmp != 0x2A && tmp != 0x55) {  // Skip at positions 42 and 85
                m_millis += millis_per_tick;   // (tmp cannot be 0x00 here)
            }
        }

        if (m_ticksL == 0) {
            ++m_ticksH;  // Handle overflow of lower 16 bits
        }
    }

    /**
     * @brief Get current timestamp with second and fractional tick components
     * @param[out] now TimeStamp structure to fill with current time
     *
     * This provides the most accurate time representation, unaffected by
     * the jitter corrections applied to millis(). Ideal for data logging
     * where precision and consistency are critical.
     *
     * The atomic block ensures the seconds and ticks values are from the
     * same moment in time (prevents ISR interruption during read).
     */
    inline void now(TimeStamp & now) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            now.seconds = m_secs;
            now.ticks = m_ticksL & mask;  // Extract fractional ticks within second
        }
    }

    /**
     * @brief Get raw tick counter value
     * @return 32-bit tick count since initialization
     *
     * Provides the highest resolution time measurement. The wrap period
     * depends on ticks_per_second:
     * - 1024 ticks/sec: wraps after ~49 days
     * - 16 ticks/sec: wraps after ~8.5 years
     *
     * Useful for precise interval measurements when the tick rate is known.
     *
     * @note Atomic read protects against torn reads during ISR updates
     */
    inline uint32_t ticks(void) {
        uint32_t tmp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tmp = m_ticks;
        }
        return tmp;
    }

    /**
     * @brief Get approximate millisecond counter
     * @return 32-bit millisecond count (wraps ~49.7 days)
     *
     * This is the most commonly used time representation for delays, timeouts,
     * and short-term timing. It remains consistent when ticks_per_second changes.
     *
     * ## Important Notes
     * - Not perfectly accurate due to 32.768 kHz crystal frequency
     * - Has slight jitter (~+-1ms) due to skip-tick correction algorithm
     * - Long-term accuracy is good (maintains ~1000ms/second average)
     * - Use secs() for durations > 1 second if jitter matters
     *
     * @note Atomic read protects against torn reads during ISR updates
     */
    inline uint32_t millis(void) {
        uint32_t tmp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tmp = m_millis;
        }
        return tmp;
    }

    /**
     * @brief Get second counter value
     * @return 32-bit second count (wraps after ~136 years)
     *
     * The most stable time representation with no jitter. Increments exactly
     * once per second based on the tick counter reaching ticks_per_second.
     *
     * Ideal for:
     * - Long-term timing (minutes, hours, days)
     * - Events that need second-level precision
     * - Uptime tracking
     *
     * @note Atomic read protects against torn reads during ISR updates
     */
    inline uint32_t secs(void) {
        uint32_t tmp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tmp = m_secs;
        }
        return tmp;
    }

    private:
    /**
     * @brief Private constructor (singleton pattern)
     *
     * Construction is handled by instance(). The actual initialization
     * of hardware and counters happens in init().
     */
    Ticker() { /* Initialization done in init() */ }

    /// Destructor not needed - singleton lives for program lifetime
    ~Ticker() = default;

    /// Disable copy construction - singleton cannot be copied
    Ticker(const Ticker&) = delete;

    /// Disable assignment - singleton cannot be assigned
    Ticker& operator=(const Ticker&) = delete;
};
