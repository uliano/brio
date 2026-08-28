/*
 * reset.hpp
 *
 * RSTCTRL (chapter 14) and the WDT (chapter 22) of the AVR DA/DB: why
 * the program is running, and how to end it on purpose.
 *
 * These two blocks are the other half of the panic breadcrumb. The
 * kernel writes a PanicRecord into reset-surviving storage
 * (kernel/panic.hpp, AvrPlatform::panic_record) and hands over to a
 * reporter whose job may be "reset now and report at the next boot";
 * panic.hpp's own contract then says to "cross-check the reset-cause
 * register on the target for the full story". `Reset::take_flags()` IS
 * that cross-check, `Reset::software()` is one way of performing the
 * reset, and `Watchdog` is the other - all three in the target stratum,
 * so no app has to reach for a register.
 *
 * Two facts of this silicon shape everything here:
 *
 *  - RSTFR ACCUMULATES. A flag is set by its reset source and cleared
 *    only by writing a one to it (14.5.1) - except that a power-on
 *    reset clears every flag but PORF and a brown-out reset clears
 *    every flag but PORF and BORF. Software that never clears the
 *    register therefore reads the whole history since power-up, not
 *    the last reset. take_flags() reads and clears in one verb: call
 *    it once, first thing at boot, and keep the answer.
 *  - SRAM is NOT promised across a reset. The data sheet states that
 *    RAM is kept in sleep (13.2) and lists the logic domains each
 *    reset clears (table 14-1, corrected by DS80000915F 3.3.1) - SRAM
 *    is in neither list, for any source including power-on. A .noinit
 *    breadcrumb is therefore only ever trustworthy behind a magic
 *    word, which is exactly what take_panic_record() checks.
 *
 * The WDT runs from the 1.024 kHz output of OSC32K, asynchronously to
 * CLK_PER, and keeps counting in every sleep mode and through a main
 * clock failure (22.2). Its period and its optional closed window use
 * the SAME encoding, 8 ms to 8 s. CTRLA and the LOCK bit are CCP
 * protected (22.3.7), CTRLA must not be written while SYNCBUSY is set
 * (22.3.6), and a lock is one-way: only a debugger (or a reset) takes
 * it off again, so a locked watchdog cannot be disabled from software.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

namespace brio {

// =============================================================================
// RSTCTRL
// =============================================================================

/// The six sources of RSTCTRL.RSTFR (14.5.1), as one snapshot. `raw`
/// keeps the register byte for printing; `any` is false only on the
/// impossible-looking boot where nothing is flagged (the flags were
/// cleared by earlier software without a reset in between).
struct ResetFlags {
    bool power_on;    ///< PORF - and the only flag a power-on reset leaves standing
    bool brown_out;   ///< BORF
    bool external;    ///< EXTRF - the RESET pin
    bool watchdog;    ///< WDRF - a time-out or a window violation
    bool software;    ///< SWRF - a write to RSTCTRL.SWRR
    bool updi;        ///< UPDIRF - the debugger/programmer
    uint8_t raw;

    constexpr bool any() const { return raw != 0; }
};

/// The reset controller: which reset brought us here, and one way to
/// cause the next one.
struct Reset {
    Reset() = delete;

    /// Peek at RSTFR without clearing it.
    static ResetFlags flags() { return decode(RSTCTRL.RSTFR); }

    /// Read RSTFR and clear exactly the bits that were set (write-one-
    /// to-clear, 14.5.1). The verb to call once at boot: what it
    /// returns is the reset that started THIS run, and the register is
    /// left clean for the next one.
    static ResetFlags take_flags() {
        const uint8_t f = RSTCTRL.RSTFR;
        RSTCTRL.RSTFR = f;
        return decode(f);
    }

    /// Clear every flag without reading them.
    static void clear_flags() { RSTCTRL.RSTFR = 0x3F; }

    /// Software reset: a CCP-protected write of SWRST (14.3.2.1.5).
    /// The reset sequence starts immediately (tSWR ~150 ns), so the
    /// loop below is only there to make the never-returns promise true
    /// for the compiler.
    [[noreturn]] static void software() {
        _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRST_bm);
        for (;;) {
        }
    }

private:
    static constexpr ResetFlags decode(uint8_t f) {
        return ResetFlags{
            (f & RSTCTRL_PORF_bm) != 0,  (f & RSTCTRL_BORF_bm) != 0,
            (f & RSTCTRL_EXTRF_bm) != 0, (f & RSTCTRL_WDRF_bm) != 0,
            (f & RSTCTRL_SWRF_bm) != 0,  (f & RSTCTRL_UPDIRF_bm) != 0, f};
    }
};

// =============================================================================
// WDT
// =============================================================================

/// PERIOD and WINDOW share one encoding (22.5.1); the names are the
/// data sheet's nominal times, the values ARE the register codes.
/// `off` in PERIOD disables the watchdog, `off` in WINDOW disables
/// window mode.
enum class WdtTime : uint8_t {
    off = 0x0,
    ms8 = 0x1,     ///< 8 OSC32K/1024 cycles, 7.8125 ms nominal
    ms16 = 0x2,
    ms32 = 0x3,
    ms64 = 0x4,
    ms125 = 0x5,
    ms250 = 0x6,
    ms500 = 0x7,
    s1 = 0x8,
    s2 = 0x9,
    s4 = 0xA,
    s8 = 0xB,
};

/// Nominal duration in microseconds of a WdtTime (0 for `off`): the
/// counter is 8 << (code - 1) cycles of a 1.024 kHz clock, so the
/// nominal times are the data sheet's 7.8125 ms .. 8 s and not the
/// round numbers the enum is named after.
constexpr uint32_t wdt_time_us(WdtTime t) {
    const uint8_t code = static_cast<uint8_t>(t);
    // us = cycles * 1e6 / 1024, written as * 15625 / 16 so the widest
    // case (8192 cycles) stays inside 32 bits.
    return code == 0 ? 0u
                     : (static_cast<uint32_t>(8) << (code - 1)) * 15'625u / 16u;
}

/// The watchdog. Its clock is the 1.024 kHz output of OSC32K, running
/// in every sleep mode and independent of CLK_PER (22.2).
struct Watchdog {
    Watchdog() = delete;

    /// Restart the time-out period: the WDR instruction (22.2). In
    /// window mode this is also the instruction that must land INSIDE
    /// the open window - too early resets the device just as too late
    /// does. Two details the code around it has to respect: the FIRST
    /// WDR after window mode is enabled (or after leaving Debug mode)
    /// is what ACTIVATES the window and is never judged (22.3.3.2), and
    /// a WDR needs two to three WDT cycles to synchronize, so two of
    /// them back to back are one event, not two.
    [[gnu::always_inline]] static void clear() { __asm__ __volatile__("wdr"); }

    /// STATUS.SYNCBUSY: a CTRLA write is still crossing into the WDT
    /// clock domain. Writing CTRLA while this is set is not allowed
    /// (22.3.6).
    static bool busy() { return (WDT.STATUS & WDT_SYNCBUSY_bm) != 0; }

    /// STATUS.LOCK: CTRLA is write-protected. Set by the WDTCFG fuse at
    /// boot when the fuse enables the watchdog, or by lock() - and only
    /// a debugger or a reset ever clears it (22.3.3.3).
    static bool locked() { return (WDT.STATUS & WDT_LOCK_bm) != 0; }

    static WdtTime period() {
        return static_cast<WdtTime>(WDT.CTRLA & WDT_PERIOD_gm);
    }
    static WdtTime window() {
        return static_cast<WdtTime>((WDT.CTRLA & WDT_WINDOW_gm) >> WDT_WINDOW_gp);
    }
    static bool enabled() { return period() != WdtTime::off; }

    /// Bounded wait for STATUS.SYNCBUSY. The configuration a CTRLA
    /// write asks for is IN FORCE only once this returns: the register
    /// lives in CLK_PER's domain and the counter in the 1.024 kHz one,
    /// so an arm() is followed by two to three WDT cycles (~3 ms) in
    /// which the OLD configuration is still the one running. That is
    /// invisible in Normal mode (the period is milliseconds anyway) and
    /// decisive in Window mode, where a WDR issued during those cycles
    /// is judged by the configuration being replaced. arm() does NOT
    /// wait afterwards on purpose - a panic reporter arming the
    /// watchdog on its way out must not block for three milliseconds.
    /// False when the wait ran out (a stopped OSC32K).
    static bool sync() { return wait_sync(); }

    /// Program PERIOD and WINDOW in the one CCP-protected store CTRLA
    /// wants. `window` other than off arms window mode: a WDR before
    /// the closed period expires resets the device, exactly as a
    /// missing one does. False (and nothing written) when CTRLA is
    /// locked or the synchronization of an earlier write never
    /// finished. Returning does not mean the new configuration is
    /// running yet - see sync().
    static bool arm(WdtTime period, WdtTime window = WdtTime::off) {
        if (locked()) {
            return false;
        }
        if (!wait_sync()) {
            return false;
        }
        _PROTECTED_WRITE(WDT.CTRLA, static_cast<uint8_t>(
            static_cast<uint8_t>(period) |
            static_cast<uint8_t>(static_cast<uint8_t>(window) << WDT_WINDOW_gp)));
        return true;
    }

    /// Stop the watchdog (PERIOD = OFF, window mode off with it).
    static bool off() { return arm(WdtTime::off, WdtTime::off); }

    /// Write-protect CTRLA for good: after this, arm() and off() refuse
    /// and only a reset (or a debugger) gives the register back. The
    /// one-way trip is the point - it is what keeps a runaway program
    /// from disabling its own watchdog (22.3.3.3).
    static void lock() {
        _PROTECTED_WRITE(WDT.STATUS, WDT_LOCK_bm);
    }

private:
    /// Bounded wait for SYNCBUSY. The flag lives two to three cycles of
    /// a 1.024 kHz clock (~3 ms); the bound is generous and exists only
    /// so a stopped OSC32K cannot hang the caller.
    static bool wait_sync() {
        for (uint32_t i = 0; i < 2'000'000u; ++i) {
            if (!busy()) {
                return true;
            }
        }
        return false;
    }
};

} // namespace brio
