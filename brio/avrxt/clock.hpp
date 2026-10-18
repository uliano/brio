/*
 * clock.hpp
 *
 * The clock controller (CLKCTRL) of the AVR DA/DB (DS40002247B ch. 12)
 * and of the AVR EA (DS40002443A ch. 12) in two strata, as
 * docs/avrxt/clkctrl.md describes:
 *
 *  RESOURCES - one monostate per block of the controller, each a thin
 *  typed view of its registers (CCP-protected writes, status bits):
 *    Oschf        internal HF oscillator: on the DA/DB 1/2/3/4/8/12/16/
 *                 20/24 MHz by FRQSEL, on the EA ONE rate the OSCCFG fuse
 *                 chose (16 or 20 MHz, no register field); RUNSTDBY,
 *                 manual tune (-32..+31 steps of ~0.4 % on the DB, 0.2 %
 *                 typical on the EA), auto-tune against a 32.768 kHz crystal
 *    Osc32k       internal 32.768 kHz ULP oscillator (+-10 %): RUNSTDBY
 *    Xosc32k      32.768 kHz crystal on PF0/PF1 or external clock on PF0:
 *                 enable, source, start-up time, low-power mode, RUNSTDBY
 *    Xoschf       HF crystal on PA0/PA1 or external clock on PA0: DB 4-24
 *                 MHz crystal, 32 MHz clock, with a frequency range; EA
 *                 4-20 MHz crystal, 20 MHz clock, no range field. Enable,
 *                 source, start-up time, RUNSTDBY (DB and EA)
 *    Pll          x2 / x3 from OSCHF or XOSCHF, 16-24 MHz in, 32-48 MHz
 *                 out - it clocks the TCD only, never the main clock
 *                 (DA/DB; the EA has none)
 *    MainClock    CLKSEL (OSCHF / OSC32K / XOSC32K / EXTCLK), the 1..64
 *                 prescaler, CLKOUT on PA7, the "switching" status
 *    ClockTimebase  MCLKTIMEBASE, CLK_PER cycles per microsecond rounded
 *                 up (EA): the time base the ADC and the TWI's bus
 *                 time-out count in, owned by the clock and written by
 *                 every Clock init and every DynamicClock switch
 *    ClockFailure the clock failure detector: source, interrupt (normal
 *                 or NMI), test bit, flag, ISR body (DB and EA)
 *
 *  TASKS - what an application names:
 *    Clock<source, source_hz, div>   the static main clock: ONE constexpr
 *                 truth `hz` every driver derives from; init() composes the
 *                 resources and reports whether the requested source runs
 *    DynamicClock<Boot, Users...>    the runtime regime: set<hz>()/set(hz)
 *                 move the prescaler and rebase the listed users first
 *                 (docs/design/clock.md)
 *
 * Facts that shape the code on the DA/DB (12.3, 39.10, errata
 * DS80000915F 2.5.x):
 *  - after any reset the device runs from OSCHF at 4 MHz (or OSC32K,
 *    per the OSCCFG fuse); every CTRL register is CCP-protected
 *    (_PROTECTED_WRITE); CLK_MAIN max 24 MHz at any VDD (1.8-5.5 V);
 *  - internal sources start when requested; crystals must be ENABLED
 *    and take their start-up time (XOSCHF 256/1k/4k cycles, XOSC32K 1k..
 *    64k cycles = 300 ms typ., 1 s in low-power mode); an external
 *    source that never toggles leaves a pending switch that only a
 *    reset clears - so the source is started and its status waited for
 *    BEFORE it is selected;
 *  - RUNSTDBY keeps an oscillator running in standby (and removes its
 *    start-up time from a later request); on rev A4 it does not work
 *    for external sources (2.5.2) and EXTS is not set for EXTCLK +
 *    RUNSTDBY without a requester (2.5.1);
 *  - the PLL runs only when a peripheral (TCD) requests it; PLLS never
 *    sets with RUNSTDBY and no requester (2.5.3, A4/A5); it does not
 *    run from an XOSCHF crystal, only from an external clock (2.5.4);
 *  - OSCHF accuracy: calibrated +-2..5 % at >= 4 MHz over VDD and
 *    temperature, 1-3 MHz +-6..10 %; OSC32K +-10 %. The manual tune is
 *    not 0.4 %/step across its range (bench, A5, at 16 MHz: -32 steps =
 *    -8.8 %, +31 steps = +12.4 %: ~0.28 %/step down, 0.4 %/step up);
 *  - MCLKSTATUS follows the REQUEST: OSCHFS reads 0 while OSCHF is not
 *    the main clock and nobody asks for it, RUNSTDBY notwithstanding
 *    (bench); a CFD fallback really resets OSCHF to 4 MHz (FRQSEL 0x3)
 *    and its interrupt re-fires every 10 OSC32K cycles (305 us, bench).
 *
 * And on the EA (DS40002443A 12.3, 8.8.2.3, 35.10; DS80001048D has no
 * CLKCTRL item):
 *  - after any reset CLK_MAIN is OSCHF at the FUSE's rate behind the
 *    prescaler at /6 (12.3.3; MCLKCTRLB resets to 0x11): 3.33 MHz from a
 *    20 MHz fuse. The CPU reads FUSE.OSCCFG and cannot write it; the
 *    Clock type names the rate as a claim and init() checks it against
 *    the fuse with one read (matches_fuse());
 *  - CLK_MAIN max 20 MHz, and only from VDD 4.5 V (35.10.5): the type
 *    asserts the 20 MHz, the supply rows are the reader's (clkctrl.md);
 *  - OSCHFCTRLA resets to 0x0C with bits 3:2 undocumented (12.5.8): it is
 *    only ever read-modify-written here, never written whole;
 *  - MCLKTIMEBASE (12.3.6, 12.5.7, not under CCP) must cover at least one
 *    microsecond of CLK_PER - too small makes the ADC's and the TWI's
 *    internal delays short - so it is written with every rate change, on
 *    the side of the switch that keeps it covering 1 us throughout.
 *
 * The RTC's own clock (CLK_RTC: OSC32K or XOSC32K) is selected by the
 * RTC driver (Ticker); CLKCTRL only provides the oscillators - start
 * Xosc32k here before Ticker::init() when a 32k crystal is fitted.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrxt/device_tables.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// Resources
// =============================================================================

/// Poll one MCLKSTATUS bit with a bounded spin. Internal helper of this
/// file; the resources expose named status() functions.
inline bool clkctrl_wait(uint8_t mask, uint32_t spins) {
    while (spins--) {
        if (CLKCTRL.MCLKSTATUS & mask) {
            return true;
        }
    }
    return false;
}

#if !defined(CLKCTRL_XOSCHFCTRLA)
/// DA: whether a clock arrives on PA0 (EXTCLK), asked without committing
/// the main clock. EXTS is set only while EXTCLK is REQUESTED (DS40002183A
/// 11.3.3.2.2: the pin becomes EXTCLK "if any peripheral requests this
/// clock"; measured on an AVR128DA48: unrequested, EXTS stays clear with a
/// clock on the pin; requested, it is set within three spins). The main
/// clock cannot be the requester of a question: a switch asked of a
/// source that never toggles stays pending until a reset (11.3.1). TCD0,
/// on EXTCLK and enabled, is a requester that commits nothing - it is
/// disabled again at once, CTRLA back to its reset value. With no clock
/// on the pin its enable never synchronizes and TCD0 is left waiting for
/// one; the caller - a Clock init, first thing in main() - uses no TCD
/// yet.
inline bool extclk_seen(uint32_t spins) {
    auto enable_ready = [] {
        for (uint16_t i = 0; i < 0xFFFFu; ++i) {
            if (TCD0.STATUS & TCD_ENRDY_bm) {
                return true;
            }
        }
        return false;
    };
    if (!enable_ready()) {
        return false;
    }
    TCD0.CTRLA = TCD_CLKSEL_EXTCLK_gc;
    TCD0.CTRLA = TCD_CLKSEL_EXTCLK_gc | TCD_ENABLE_bm;
    const bool seen = clkctrl_wait(CLKCTRL_EXTS_bm, spins);
    if (enable_ready()) {
        TCD0.CTRLA = TCD_CLKSEL_EXTCLK_gc;
        if (enable_ready()) {
            TCD0.CTRLA = 0;
        }
    }
    return seen;
}
#endif

// ---- OSCHF ------------------------------------------------------------------

#if defined(CLKCTRL_FRQSEL_gm)
/// OSCHF FRQSEL code for a rate it can produce, 0xFF otherwise. Only
/// where OSCHF's rate is a register field (device tables'
/// has_oschf_frqsel); where a fuse chooses it there is no such table.
constexpr uint8_t oschf_frqsel(uint32_t hz) {
    switch (hz) {
    case 1'000'000: return CLKCTRL_FRQSEL_1M_gc;
    case 2'000'000: return CLKCTRL_FRQSEL_2M_gc;
    case 3'000'000: return CLKCTRL_FRQSEL_3M_gc;
    case 4'000'000: return CLKCTRL_FRQSEL_4M_gc;
    case 8'000'000: return CLKCTRL_FRQSEL_8M_gc;
    case 12'000'000: return CLKCTRL_FRQSEL_12M_gc;
    case 16'000'000: return CLKCTRL_FRQSEL_16M_gc;
    case 20'000'000: return CLKCTRL_FRQSEL_20M_gc;
    case 24'000'000: return CLKCTRL_FRQSEL_24M_gc;
    default: return 0xFF;
    }
}
#endif // CLKCTRL_FRQSEL_gm

/// The internal high-frequency oscillator.
struct Oschf {
    Oschf() = delete;

#if defined(CLKCTRL_FRQSEL_gm)
    /// Set the output frequency (one oschf_frqsel() can produce); keeps
    /// the RUNSTDBY and AUTOTUNE bits. Takes effect immediately, also
    /// while OSCHF is the main clock. False, touching nothing, for a
    /// rate OSCHF does not produce (a runtime verb must not write a
    /// reserved FRQSEL).
    static bool set_hz(uint32_t hz) {
        const uint8_t sel = oschf_frqsel(hz);
        if (sel == 0xFF) return false;
        const uint8_t keep = static_cast<uint8_t>(CLKCTRL.OSCHFCTRLA &
                                                  (CLKCTRL_RUNSTDBY_bm | CLKCTRL_AUTOTUNE_bm));
        _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, static_cast<uint8_t>(sel | keep));
        return true;
    }
#endif
    static void run_standby(bool on) { flag(CLKCTRL_RUNSTDBY_bm, on); }

    /// Auto-tune the frequency against a running 32.768 kHz crystal
    /// (Xosc32k must be started and stable). Locks the manual tune.
    ///
    /// Clearing it copies the auto-tune's last value into TUNE (11.5.6),
    /// and the copy lands AFTER the protected write: measured on an
    /// AVR128DA48, a read in the next instructions still returns the old
    /// TUNE and one 3 us later the new one - a tune() written in between
    /// was overwritten. The auto-tune runs on OSCHF's internal 1 MHz
    /// clock (11.3.5) and the copy crosses from it, so the clear waits
    /// 120 CPU cycles - 5 us at the fastest CLK_PER, longer at any slower
    /// one - before it returns.
    ///
    /// On the EA the copy lands within 0.75 us and three CLK_MAIN cycles
    /// (DS40002443A 12.5.9): the same 120 cycles cover it at every CLK_PER
    /// the part reaches (6 us at 20 MHz). The field is the device tables'
    /// (one bit on the DA/DB, a two-bit field with code XOSC32K on the EA).
    static void autotune(bool on) {
        const uint8_t a = CLKCTRL.OSCHFCTRLA;
        _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, static_cast<uint8_t>(
            on ? (a | oschf_autotune_on) : (a & ~oschf_autotune_field)));
        if (!on) {
            __builtin_avr_delay_cycles(120);
        }
    }

    /// Manual tune: -32 .. +31 steps (~0.4 % on the DB, 0.2 % typical on
    /// the EA, 35.10.1); a 6-bit two's complement in OSCHFTUNE, ignored
    /// while AUTOTUNE is on. The EA mirrors bit 5 into bits 7:6 (12.5.9).
    static void tune(int8_t steps) {
        if (steps < -32) steps = -32;
        if (steps > 31) steps = 31;
        CLKCTRL.OSCHFTUNE = static_cast<uint8_t>(steps & 0x3F);
    }
    static int8_t tune() {
        const uint8_t v = CLKCTRL.OSCHFTUNE & 0x3F;
        return static_cast<int8_t>(v & 0x20 ? static_cast<int8_t>(v) - 64 : v);
    }

    static bool stable() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_OSCHFS_bm) != 0; }

private:
    static void flag(uint8_t bit, bool on) {
        const uint8_t v = static_cast<uint8_t>(on ? (CLKCTRL.OSCHFCTRLA | bit)
                                                  : (CLKCTRL.OSCHFCTRLA & ~bit));
        _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, v);
    }
};

// ---- OSC32K -----------------------------------------------------------------

/// The internal 32.768 kHz ULP oscillator (29.5 .. 36 kHz: +-10 %).
struct Osc32k {
    Osc32k() = delete;
    static void run_standby(bool on) {
        _PROTECTED_WRITE(CLKCTRL.OSC32KCTRLA, static_cast<uint8_t>(on ? CLKCTRL_RUNSTDBY_bm : 0));
    }
    static bool stable() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_OSC32KS_bm) != 0; }
};

// ---- XOSC32K ----------------------------------------------------------------

enum class Xosc32kStartup : uint8_t { cycles1k = 0, cycles16k = 1, cycles32k = 2, cycles64k = 3 };

/// The 32.768 kHz crystal oscillator on PF0/PF1, or an external 32 kHz
/// clock on PF0. Start-up ~300 ms typ. (1 s in low-power mode).
///
/// ENABLE ALONE DOES NOT RUN IT. With RUNSTDBY clear the oscillator runs
/// only while something REQUESTS it - the RTC, the main clock, the PLL -
/// and XOSC32KS stays clear until then (measured on an AVR128DA48: a
/// fitted crystal, enabled, never stable; forced on, stable within about
/// a second). So a program that wants to KNOW the crystal runs before
/// anything uses it starts it with run_standby = true - the bit forces
/// the oscillator on in Active and every sleep mode - and may clear the
/// force with force_on(false) once a user requests it.
struct Xosc32k {
    Xosc32k() = delete;

    /// Crystal on PF0/PF1. CSUT/SEL are read-only while ENABLE (or the
    /// oscillator is in use): a start on an enabled oscillator stops it
    /// first, so the new configuration always lands (the restart pays
    /// the start-up time again).
    static void start_crystal(Xosc32kStartup sut = Xosc32kStartup::cycles64k,
                              bool low_power = false, bool run_standby = false) {
        if (enabled()) stop();
        _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm |
            (static_cast<uint8_t>(sut) << CLKCTRL_CSUT_gp) |
            (low_power ? CLKCTRL_LPMODE_bm : 0) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// External 32.768 kHz clock on PF0 (start-up two cycles).
    static void start_external(bool run_standby = false) {
        if (enabled()) stop();                     // SEL is latched while enabled
        _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm | CLKCTRL_SEL_bm | (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// Off; PF0/PF1 return to the PORT.
    static void stop() { _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, 0); }
    /// RUNSTDBY on a running oscillator (the one field of the register
    /// that is not write-protected while ENABLE is set): forced on, or
    /// running only while requested.
    static void force_on(bool on) {
        const uint8_t a = CLKCTRL.XOSC32KCTRLA;
        _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, static_cast<uint8_t>(
            on ? (a | CLKCTRL_RUNSTDBY_bm) : (a & ~CLKCTRL_RUNSTDBY_bm)));
    }
    static bool forced_on() { return (CLKCTRL.XOSC32KCTRLA & CLKCTRL_RUNSTDBY_bm) != 0; }
    static bool enabled() { return (CLKCTRL.XOSC32KCTRLA & CLKCTRL_ENABLE_bm) != 0; }
    static bool stable() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm) != 0; }
    /// Wait for stable() with a bound (a few cycles per spin; a 1 s
    /// crystal start-up at 24 MHz needs millions). Meaningful only while
    /// the oscillator is forced on or requested (above).
    static bool wait_stable(uint32_t spins = 0x00FFFFFFu) {
        return clkctrl_wait(CLKCTRL_XOSC32KS_bm, spins);
    }
};

// ---- XOSCHF -----------------------------------------------------------------

// Whether this part has the crystal oscillator is device_tables.hpp's
// has_xoschf: a program choosing its HF source asks that value.

#if defined(CLKCTRL_XOSCHFCTRLA)
enum class XoschfStartup : uint8_t { cycles256 = 0, cycles1k = 1, cycles4k = 2 };

/// The high-frequency crystal oscillator on PA0/PA1, or an external clock
/// on PA0: DB 4-24 MHz crystal and 32 MHz clock, with a frequency range
/// (the oscillator's drive) to choose; EA 4-20 MHz crystal (8-20 MHz from
/// 2.5 V) and 20 MHz clock, and no range field (DS40002443A 12.5.12,
/// 35.10.3-4). SELHF is bit 1 on both (crystal 0, clock 1).
struct Xoschf {
    Xoschf() = delete;

#if defined(CLKCTRL_FRQRANGE_gm)
    /// Frequency range code for a crystal rate (the oscillator's drive).
    static constexpr uint8_t range_for(uint32_t hz) {
        return hz <= 8'000'000 ? CLKCTRL_FRQRANGE_8M_gc
             : hz <= 16'000'000 ? CLKCTRL_FRQRANGE_16M_gc
             : hz <= 24'000'000 ? CLKCTRL_FRQRANGE_24M_gc
                                : CLKCTRL_FRQRANGE_32M_gc;
    }
#else
    /// No range field on this part: nothing to choose for a rate.
    static constexpr uint8_t range_for(uint32_t) { return 0; }
#endif

    /// Crystal on PA0/PA1 at `hz`; start-up 4k cycles by default.
    /// SELHF/FRQRANGE/CSUTHF are read-only while ENABLE (or EXTS): a
    /// start on an enabled oscillator stops it first, so a
    /// crystal-to-external (or range) change always lands.
    static void start_crystal(uint32_t hz, XoschfStartup sut = XoschfStartup::cycles4k,
                              bool run_standby = true) {
        if (enabled()) stop();
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm | range_for(hz) |
            (static_cast<uint8_t>(sut) << CLKCTRL_CSUTHF_gp) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// External clock on PA0 (start-up two cycles).
    static void start_external(uint32_t hz, bool run_standby = true) {
        if (enabled()) stop();                     // the config is latched while enabled
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm | CLKCTRL_SELHF_bm | range_for(hz) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// Off; PA0/PA1 return to the PORT.
    static void stop() { _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, 0); }
    static bool enabled() { return (CLKCTRL.XOSCHFCTRLA & CLKCTRL_ENABLE_bm) != 0; }
    /// Crystal stable / external clock running (EXTS). DB errata 2.5.1
    /// (A4): not set for EXTCLK + RUNSTDBY without a requester.
    static bool stable() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm) != 0; }
    static bool wait_stable(uint32_t spins = 0xFFFFu) { return clkctrl_wait(CLKCTRL_EXTS_bm, spins); }
};
#endif

// ---- PLL --------------------------------------------------------------------
// Only where the part has one (its status bit is the block, device
// tables' has_pll); without it, Pll and its vocabulary do not exist.
#if defined(CLKCTRL_PLLS_bm)

enum class PllSource : uint8_t { oschf = 0, xoschf = 1 };
enum class PllMultiplier : uint8_t {
    off = CLKCTRL_MULFAC_DISABLE_gc, x2 = CLKCTRL_MULFAC_2x_gc, x3 = CLKCTRL_MULFAC_3x_gc
};

/// The PLL: 16-24 MHz in, 32-48 MHz out, lock ~10 us. It clocks the TCD
/// only (CLK_TCD); it cannot be the main clock. It runs only when the
/// TCD requests it: PLLS is not observable without a requester (and
/// never sets with RUNSTDBY and no requester: errata 2.5.3 A4/A5, DA
/// 2.4.1 every revision).
///
/// ERRATA 2.5.4 AS CODE (DB A4/A5): with SOURCE = XOSCHF the PLL runs
/// only if XOSCHF is configured for an external CLOCK (SELHF = 1) - it
/// does not work with a crystal, and there is no workaround. start()
/// therefore REFUSES that combination and writes nothing: a bench
/// crystal on PA0/PA1 cannot feed the PLL, and a driver that pretended
/// otherwise would leave the TCD without a clock and its ENRDY closed
/// forever. The DA errata has no twin item and needs none: the DA has
/// no XOSCHF crystal oscillator at all (no CLKCTRL.XOSCHFCTRLA in its
/// header), so SOURCE = 1 there IS the external clock and the check
/// compiles out.
struct Pll {
    Pll() = delete;

    /// False (writing nothing) when the source cannot drive the PLL on
    /// this silicon: see errata 2.5.4 above.
    static bool start(PllSource src, PllMultiplier mul, bool run_standby = false) {
        if (!source_ok(src)) return false;
        _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, static_cast<uint8_t>(
            static_cast<uint8_t>(mul) |
            (src == PllSource::xoschf ? CLKCTRL_SOURCE_bm : 0) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
        return true;
    }
    /// Can this source drive the PLL as the block stands right now?
    static bool source_ok(PllSource src) {
#if defined(CLKCTRL_XOSCHFCTRLA)
        if (src == PllSource::xoschf) {
            return (CLKCTRL.XOSCHFCTRLA & CLKCTRL_SELHF_bm) != 0;   // external clock only
        }
#else
        (void)src;
#endif
        return true;
    }
    static void stop() { _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, 0); }
    static bool locked() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_PLLS_bm) != 0; }
    /// The multiplier in force (off when the PLL is disabled).
    static PllMultiplier multiplier() {
        return static_cast<PllMultiplier>(CLKCTRL.PLLCTRLA & CLKCTRL_MULFAC_gm);
    }
    static PllSource source() {
        return (CLKCTRL.PLLCTRLA & CLKCTRL_SOURCE_bm) ? PllSource::xoschf : PllSource::oschf;
    }
};

/// What the PLL makes of an input rate (39.10.5: 16-24 MHz in, 32-48
/// MHz out); 0 when the combination is out of specification.
constexpr uint32_t pll_output_hz(uint32_t in_hz, PllMultiplier mul) {
    if (in_hz < 16'000'000u || in_hz > 24'000'000u) return 0;
    const uint32_t out = mul == PllMultiplier::x2 ? in_hz * 2u
                       : mul == PllMultiplier::x3 ? in_hz * 3u : 0u;
    return out > 48'000'000u ? 0u : out;
}
#endif // CLKCTRL_PLLS_bm

// ---- main clock -------------------------------------------------------------

/// One MCLKCTRLB value: PDIV's code in place, the prescaler enabled.
constexpr uint8_t clock_pdiv(uint8_t code) {
    return static_cast<uint8_t>((code << CLKCTRL_PDIV_gp) | CLKCTRL_PEN_bm);
}

/// Main prescaler (CLKCTRL.MCLKCTRLB). div1 = prescaler disabled. The PDIV
/// codes are the same on every part (DS40002247B 12.5.2, DS40002443A
/// 12.5.2: DIV2 0x0 .. DIV64 0x5, DIV6 0x8 .. DIV48 0xC) under two sets of
/// enumerator names (CLKCTRL_PDIV_2X_gc against CLKCTRL_PDIV_DIV2_gc), so
/// they are written here as the chapter's numbers and test/family_avrxt/clock.cpp
/// holds them against each header's names.
enum class ClockDiv : uint8_t {
    div1 = 0,
    div2 = clock_pdiv(0x0),
    div4 = clock_pdiv(0x1),
    div6 = clock_pdiv(0x8),
    div8 = clock_pdiv(0x2),
    div10 = clock_pdiv(0x9),
    div12 = clock_pdiv(0xA),
    div16 = clock_pdiv(0x3),
    div24 = clock_pdiv(0xB),
    div32 = clock_pdiv(0x4),
    div48 = clock_pdiv(0xC),
    div64 = clock_pdiv(0x5),
};

constexpr uint32_t clock_divisor(ClockDiv d) {
    switch (d) {
    case ClockDiv::div1: return 1;
    case ClockDiv::div2: return 2;
    case ClockDiv::div4: return 4;
    case ClockDiv::div6: return 6;
    case ClockDiv::div8: return 8;
    case ClockDiv::div10: return 10;
    case ClockDiv::div12: return 12;
    case ClockDiv::div16: return 16;
    case ClockDiv::div24: return 24;
    case ClockDiv::div32: return 32;
    case ClockDiv::div48: return 48;
    case ClockDiv::div64: return 64;
    }
    return 0;
}

/// Every main prescaler, in one canonical order. This array is the
/// DISCRETE SET behind the runtime regime: a DynamicClock can only ever
/// run at source_hz divided by one of these, so its current rate is an
/// INDEX into this list - which is what lets avrxt/delay.hpp expand
/// per-rate arithmetic at compile time and dispatch on the index
/// (division-free delays at every reachable rate).
inline constexpr ClockDiv clock_divs[] = {
    ClockDiv::div1,  ClockDiv::div2,  ClockDiv::div4,  ClockDiv::div6,
    ClockDiv::div8,  ClockDiv::div10, ClockDiv::div12, ClockDiv::div16,
    ClockDiv::div24, ClockDiv::div32, ClockDiv::div48, ClockDiv::div64};
inline constexpr uint8_t clock_div_count = sizeof(clock_divs) / sizeof(clock_divs[0]);

/// Position of a prescaler in clock_divs (the index delay dispatch uses).
constexpr uint8_t clock_div_index(ClockDiv d) {
    for (uint8_t i = 0; i < clock_div_count; ++i) {
        if (clock_divs[i] == d) return i;
    }
    return 0;
}

/// The main prescaler that turns source_hz into hz exactly, or div1
/// with `ok` false when no prescaler does.
struct DivFor { ClockDiv div; bool ok; };
constexpr DivFor div_for(uint32_t source_hz, uint32_t hz) {
    for (ClockDiv d : clock_divs) {
        if (hz != 0 && source_hz / clock_divisor(d) == hz && source_hz % clock_divisor(d) == 0) {
            return {d, true};
        }
    }
    return {ClockDiv::div1, false};
}

/// What CLK_MAIN can come from (MCLKCTRLA.CLKSEL).
enum class MainSource : uint8_t {
    oschf = CLKCTRL_CLKSEL_OSCHF_gc,
    osc32k = CLKCTRL_CLKSEL_OSC32K_gc,
    xosc32k = CLKCTRL_CLKSEL_XOSC32K_gc,
    extclk = CLKCTRL_CLKSEL_EXTCLK_gc,   ///< XOSCHF: crystal or external clock per SELHF
};

/// The main clock mux, prescaler, CLKOUT and switching status. The
/// source must be running (and an external one stable) BEFORE it is
/// selected: a selected source that never toggles is a pending switch
/// only a reset clears (12.3.2).
struct MainClock {
    MainClock() = delete;

    static void prescale(ClockDiv d) { _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, static_cast<uint8_t>(d)); }

    /// Select the source; keeps CLKOUT as it is. Returns false when the
    /// switch did not complete within the spin bound (SOSC still set).
    static bool select(MainSource s, uint32_t spins = 0xFFFFu) {
        const uint8_t out = static_cast<uint8_t>(CLKCTRL.MCLKCTRLA & CLKCTRL_CLKOUT_bm);
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, static_cast<uint8_t>(static_cast<uint8_t>(s) | out));
        while (spins--) {
            if (!switching()) return true;
        }
        return false;
    }
    static MainSource source() {
        return static_cast<MainSource>(CLKCTRL.MCLKCTRLA & CLKCTRL_CLKSEL_gm);
    }
    /// SOSC: a source switch is in progress.
    static bool switching() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) != 0; }

    /// CLK_PER on the CLKOUT pin (PA7): the test instrument of this
    /// peripheral (an oscilloscope reads every rate directly). Cleared
    /// by hardware when a clock failure switches the main clock.
    static void clkout(bool on) {
        const uint8_t sel = static_cast<uint8_t>(CLKCTRL.MCLKCTRLA & CLKCTRL_CLKSEL_gm);
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, static_cast<uint8_t>(sel | (on ? CLKCTRL_CLKOUT_bm : 0)));
        if (on) {
            PORTA.DIRSET = PIN7_bm;
        }
    }
};

// ---- the time base (EA) --------------------------------------------------------

/// MCLKTIMEBASE for a CLK_PER rate: the count of CLK_PER cycles that is
/// one microsecond or more, rounded UP (DS40002443A 12.3.6, 12.5.7: a
/// value too small makes the delays timed in it short - the unsafe side;
/// the data sheet's own example, 20 MHz / 6 = 3333333 Hz, gives 4). Five
/// bits: 31 covers 31 MHz, above any CLK_PER the part reaches.
constexpr uint8_t clock_timebase_for(uint32_t hz) {
    const uint32_t t = (hz + 999'999u) / 1'000'000u;
    return static_cast<uint8_t>(t > 31u ? 31u : t);
}

#if defined(CLKCTRL_TIMEBASE_gm)
/// CLKCTRL.MCLKTIMEBASE: the time base the ADC's and the PGA's internal
/// timings and the TWI's bus time-out count in (DS40002443A 31.3.2,
/// 31.3.3.4, 27.3.3.1). It is a CLKCTRL register and a pure function of
/// CLK_PER, so the CLOCK owns it: every Clock init writes it, every
/// DynamicClock switch rewrites it, and no driver writes it. Not under
/// CCP: one STS.
struct ClockTimebase {
    ClockTimebase() = delete;
    static void write(uint8_t cycles) { CLKCTRL.MCLKTIMEBASE = cycles; }
    static uint8_t read() { return static_cast<uint8_t>(CLKCTRL.MCLKTIMEBASE & CLKCTRL_TIMEBASE_gm); }
};
#endif

// ---- clock failure detection ----------------------------------------------------
// The CFD exists on the DB and the EA; on the DA the whole block is
// compiled out (the device header is the authority: no CLKCTRL_CFDEN_bm
// there).
#ifdef CLKCTRL_CFDEN_bm

enum class CfdSource : uint8_t {
    main = CLKCTRL_CFDSRC_CLKMAIN_gc,
    xoschf = CLKCTRL_CFDSRC_XOSCHF_gc,
    xosc32k = CLKCTRL_CFDSRC_XOSC32K_gc,
};

/// The clock failure detector (DS40002247B 12.3.7, DS40002443A 12.3.5):
/// watches one source for edges; on silence it raises the CFD flag,
/// optionally an interrupt (regular or NMI), and - when the MAIN clock is
/// watched - forces CLKSEL back to the start-up source "at its reset
/// frequency" (OSCHF at 4 MHz on the DB; on the EA OSCHF at the fuse's
/// rate); CLKOUT disabled. Once enabled with the NMI interrupt, its registers are
/// read-only until a reset (the safety lock-in). A test bit forces a
/// failure. The flag re-triggers every ten OSC32K cycles while the
/// condition holds; clear it (or fix the clock) to stop.
///
/// ISR body: `cfd()` clears the flag and returns the source in effect
/// - the app binds CLKCTRL_CFD_vect (or NMI_vect) and decides what to
/// do: a brio app posts to a supervisor AO; the kernel keeps running
/// from the start-up clock, with every driver now off-rate until a
/// rebase/re-init.
struct ClockFailure {
    ClockFailure() = delete;

    static void watch(CfdSource src) {
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLC,
                         static_cast<uint8_t>(static_cast<uint8_t>(src) | CLKCTRL_CFDEN_bm));
    }
    static void stop() { _PROTECTED_WRITE(CLKCTRL.MCLKCTRLC, 0); }
    /// Interrupt on failure: regular (CLKCTRL_CFD_vect) or non-maskable
    /// (NMI_vect). NMI + enabled + watching locks the CFD configuration
    /// until reset.
    static void interrupt(bool on, bool nmi = false) {
        _PROTECTED_WRITE(CLKCTRL.MCLKINTCTRL, static_cast<uint8_t>(
            (on ? CLKCTRL_CFD_bm : 0) |
            static_cast<uint8_t>(nmi ? CLKCTRL_INTTYPE_NMI_gc : CLKCTRL_INTTYPE_INT_gc)));
    }
    /// Force a failure condition (CFDTST). With the main clock watched
    /// this switches it to the start-up source. Clear with test(false).
    static void test(bool force) {
        const uint8_t keep = static_cast<uint8_t>(CLKCTRL.MCLKCTRLC & (CLKCTRL_CFDSRC_gm | CLKCTRL_CFDEN_bm));
        _PROTECTED_WRITE(CLKCTRL.MCLKCTRLC, static_cast<uint8_t>(keep | (force ? CLKCTRL_CFDTST_bm : 0)));
    }
    static bool failed() { return (CLKCTRL.MCLKINTFLAGS & CLKCTRL_CFD_bm) != 0; }
    static void clear() { CLKCTRL.MCLKINTFLAGS = CLKCTRL_CFD_bm; }

    /// ISR body: acknowledge, report which main source is now in effect.
    [[gnu::always_inline]] static MainSource cfd() {
        clear();
        return MainClock::source();
    }
};

#endif // CLKCTRL_CFDEN_bm

// =============================================================================
// Tasks
// =============================================================================

enum class ClockSource : uint8_t {
    internal,   ///< OSCHF at a rate it can produce (the EA: the fuse's)
    crystal,    ///< XOSCHF crystal on PA0/PA1 (DB and EA)
    external,   ///< external clock on PA0 (the DB and EA through XOSCHF in
                ///< its external-clock mode, the DA directly: its only HF input)
    osc32k,     ///< the internal 32.768 kHz oscillator as main clock
    xosc32k,    ///< the 32.768 kHz crystal (or clock on PF0) as main clock
};

/// The static main clock. `src_hz` is the source's rate and `div` the main
/// prescaler: hz = src_hz / div is CLK_PER, the one truth every driver
/// derives from. `oschf_hz` is the rate OSCHF runs at - the fallback that
/// keeps hz true when an external source fails to start:
///  - on a part whose OSCHF rate is a register field (FRQSEL, the DA/DB)
///    it is src_hz and nothing else: init() sets OSCHF to it;
///  - on a part whose OSCHF rate the OSCCFG fuse chooses (the EA) it is a
///    CLAIM about the fuse - 16 or 20 MHz - which the type cannot check
///    and init() does (matches_fuse(), one read of FUSE.OSCCFG). For an
///    internal source it is the rate itself; for a crystal or an external
///    clock it defaults to src_hz, and a source at another rate names the
///    fuse's explicitly. The fallback is then OSCHF over the prescaler
///    that gives the same CLK_PER, resolved here at compile time.
template <ClockSource src, uint32_t src_hz, ClockDiv div = ClockDiv::div1,
          uint32_t oschf_hz = src_hz>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t source_hz = src_hz;
    static constexpr uint32_t divisor = clock_divisor(div);
    static constexpr uint32_t hz = src_hz / divisor;   ///< CLK_PER = CLK_CPU
    static constexpr bool is_static = true;

    static constexpr bool hf = src == ClockSource::internal || src == ClockSource::crystal ||
                               src == ClockSource::external;
    static_assert(divisor > 0, "invalid main prescaler");
#if defined(CLKCTRL_FRQSEL_gm)
    static_assert(!hf || oschf_frqsel(src_hz) != 0xFF,
                  "brio Clock: a high-frequency source rate must be one OSCHF can "
                  "produce (1/2/3/4/8/12/16/20/24 MHz) - it is the fallback that keeps "
                  "Clock::hz true when an external source fails to start");
    static_assert(!hf || oschf_hz == src_hz,
                  "brio Clock: on this part OSCHF's rate is set by the driver, and the "
                  "fallback is OSCHF at the source's own rate - leave oschf_hz alone");
#else
    static_assert(!hf || oschf_hz == oschf_fuse_hz_clear || oschf_hz == oschf_fuse_hz_set,
                  "brio Clock: on this part the OSCCFG fuse chooses OSCHF's rate - 16 or "
                  "20 MHz (DS40002443A 8.8.2.3); name that rate as the claim (the internal "
                  "source's own rate, or the fourth argument beside a crystal or a clock)");
    static_assert(src != ClockSource::internal || oschf_hz == src_hz,
                  "brio Clock: an internal source runs at the fuse's rate - src_hz is that claim");
    static_assert(!hf || div_for(oschf_hz, hz).ok,
                  "brio Clock: no main prescaler brings OSCHF at the fuse's rate to "
                  "Clock::hz - the fallback that keeps hz true when the external source "
                  "fails to start would run at another rate");
#endif
    static_assert(hf || src_hz == 32'768, "brio Clock: the 32 kHz sources are 32768 Hz");
    static_assert(src_hz % divisor == 0, "source rate not divisible by the prescaler");
    static_assert(hz <= clk_main_max_hz && (!hf || src_hz <= clk_main_max_hz),
                  "brio Clock: CLK_MAIN must not exceed the part's maximum (24 MHz on "
                  "the AVR DA/DB, 20 MHz on the AVR EA - the latter from VDD 4.5 V)");
#if defined(F_CPU)
    static_assert(hz == F_CPU,
                  "brio Clock: this build defines F_CPU with a value different "
                  "from Clock::hz. Either unflag -DF_CPU (this project does) or "
                  "make it equal: never two truths");
#endif
#if !defined(CLKCTRL_XOSCHFCTRLA)
    static_assert(src != ClockSource::crystal,
                  "brio Clock: the HF crystal (XOSCHF) exists on the DB and the EA; the "
                  "DA takes an external CLOCK on PA0 directly (ClockSource::external)");
#endif

    /// Whether the part runs OSCHF at the rate this type claims. Where the
    /// rate is a register field the driver sets, always; where the OSCCFG
    /// fuse chooses it, one read of FUSE.OSCCFG (the CPU reads the fuses
    /// at their data-space addresses, DS40002443A 8.8).
    static bool matches_fuse() {
#if defined(CLKCTRL_FRQSEL_gm)
        return true;
#else
        if constexpr (!hf) {
            return true;
        } else {
            const uint32_t fused = (FUSE.OSCCFG & FUSE_OSCHFFRQ_bm) ? oschf_fuse_hz_set
                                                                     : oschf_fuse_hz_clear;
            return fused == oschf_hz;
        }
#endif
    }

    /// Bring CLK_PER to `hz`. Returns true when running from the
    /// requested source, false when an external source failed to start
    /// and the fallback is running instead: OSCHF at the same CLK_PER for
    /// the high-frequency sources, OSC32K for xosc32k. On a part whose
    /// OSCHF rate is a fuse's, false too when the fuse does not match the
    /// claim - and then NOTHING is written: the part keeps its reset clock
    /// (the fuse's rate over /6) and Clock::hz is not true. Call first in
    /// main().
    static bool init() {
#if defined(CLKCTRL_FRQSEL_gm)
        if constexpr (hf) {
            // Baseline and fallback: OSCHF at the target rate, prescaled.
            (void)Oschf::set_hz(src_hz);          // rate static_asserted above
            Oschf::run_standby(true);
            MainClock::prescale(div);
            (void)MainClock::select(MainSource::oschf);
            (void)clkctrl_wait(CLKCTRL_OSCHFS_bm, 0xFFFFu);
            if constexpr (src == ClockSource::internal) {
                return true;
            } else {
#if defined(CLKCTRL_XOSCHFCTRLA)
                if constexpr (src == ClockSource::crystal) {
                    Xoschf::start_crystal(src_hz);
                } else {
                    Xoschf::start_external(src_hz);
                }
                if (!Xoschf::wait_stable()) {
                    Xoschf::stop();                         // release PA0/PA1, stay on OSCHF
                    return false;
                }
                if (!MainClock::select(MainSource::extclk)) {
                    (void)MainClock::select(MainSource::oschf);
                    return false;
                }
                return true;
#else
                // DA: EXTCLK is a direct input on PA0 - nothing to
                // enable, and EXTS only answers while the clock is
                // requested: ask through extclk_seen() BEFORE selecting,
                // because a selected source that never toggles leaves a
                // switch pending that only a reset clears.
                if (!extclk_seen(0xFFFFu)) {
                    return false;                   // stay on the OSCHF fallback
                }
                if (!MainClock::select(MainSource::extclk)) {
                    (void)MainClock::select(MainSource::oschf);
                    return false;
                }
                return true;
#endif
            }
        } else {
            return init_32k();
        }
#else
        // The EA: OSCHF's rate is the fuse's. Every write below is to a
        // register the fuse leaves alone, and the order keeps CLK_PER at
        // or below its final value at every step (CLK_MAIN never exceeds
        // the 20 MHz any source here is bounded by).
        if constexpr (hf) {
            if (!matches_fuse()) {
                return false;                       // reset clock kept: fuse/6
            }
            constexpr ClockDiv fb = div_for(oschf_hz, hz).div;   // the fallback's prescaler
            Oschf::run_standby(true);               // RMW: bits 3:2 untouched (12.5.8)
            MainClock::prescale(fb);
            (void)MainClock::select(MainSource::oschf);
            (void)clkctrl_wait(CLKCTRL_OSCHFS_bm, 0xFFFFu);
            ClockTimebase::write(clock_timebase_for(hz));
            if constexpr (src == ClockSource::internal) {
                return true;
            } else {
                if constexpr (src == ClockSource::crystal) {
                    Xoschf::start_crystal(src_hz);
                } else {
                    Xoschf::start_external(src_hz);
                }
                if (!Xoschf::wait_stable()) {
                    Xoschf::stop();                 // release PA0/PA1, stay on OSCHF
                    return false;
                }
                // CLK_PER is hz on both sides of the switch; where the two
                // prescalers differ, the one that lowers CLK_PER in between
                // goes first.
                if constexpr (fb == div) {
                    if (MainClock::select(MainSource::extclk)) return true;
                } else if constexpr (src_hz > oschf_hz) {
                    MainClock::prescale(div);
                    if (MainClock::select(MainSource::extclk)) return true;
                } else {
                    if (MainClock::select(MainSource::extclk)) {
                        MainClock::prescale(div);
                        return true;
                    }
                }
                MainClock::prescale(fb);
                (void)MainClock::select(MainSource::oschf);
                return false;
            }
        } else {
            const bool ok = init_32k();
            ClockTimebase::write(clock_timebase_for(hz));
            return ok;
        }
#endif
    }

private:
    /// The 32 kHz sources as the main clock (inlined: one body, the init
    /// of every part).
    [[gnu::always_inline]] static bool init_32k() {
        MainClock::prescale(div);
        if constexpr (src == ClockSource::osc32k) {
            Osc32k::run_standby(true);
            (void)clkctrl_wait(CLKCTRL_OSC32KS_bm, 0xFFFFu);
            return MainClock::select(MainSource::osc32k);
        } else {
            // Forced on while it is waited for (nothing requests it
            // yet, so XOSC32KS would never be set), then selected -
            // the main clock is a requester - and the force dropped
            // unless whoever started it had asked for it.
            const bool was_forced = Xosc32k::enabled() && Xosc32k::forced_on();
            if (!Xosc32k::enabled()) {
                Xosc32k::start_crystal(Xosc32kStartup::cycles64k, false, true);
            } else {
                Xosc32k::force_on(true);
            }
            if (!Xosc32k::wait_stable()) {
                Xosc32k::stop();
                (void)MainClock::select(MainSource::osc32k);   // fallback: the internal 32k
                return false;
            }
            const bool ok = MainClock::select(MainSource::xosc32k);
            if (!was_forced) {
                Xosc32k::force_on(false);
            }
            return ok;
        }
    }
};

/**
 * The runtime regime. Boot is a static Clock<...> naming the source and
 * its rate; set<hz>() / set(hz) name the NEW RATE (the app speaks Hz -
 * the prescaler that produces it is this silicon's detail, resolved by
 * div_for; a rate no prescaler reaches is a compile error / a false),
 * fan it out to Users (each a ClockUser: `static void rebase(uint32_t
 * hz)`, checked by the concept where the list is written) in list
 * order, synchronously, and THEN reprogram the main prescaler - so a
 * user can drain what it has in flight at the old rate before adopting
 * the new one. Call set() only when nothing that depends on the rate
 * is mid-transfer (a driver's rebase() may wait for its own hardware
 * to go idle; a bus transaction in flight is the caller's problem - in
 * an AO system, ask the bus AOs before switching). F_CPU must not be
 * defined for this type: no rate is fixed any more.
 *
 *   using Boot = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   using SysClock = brio::DynamicClock<Boot, Serial, Twi0>;
 *   constexpr SysClock clock;
 *   ...
 *   SysClock::init();                 // Boot's init: 24 MHz
 *   SysClock::set<4'000'000>();       // Serial and Twi0 rebased, then 4 MHz
 */
template <typename Boot, ClockUser... Users>
struct DynamicClock {
    static constexpr ClockSource source = Boot::source;
    static constexpr uint32_t source_hz = Boot::source_hz;
    static constexpr bool is_static = false;
    static_assert(Boot::divisor == 1, "DynamicClock: give Boot the source rate with div1; "
                                      "the prescaler is what set() changes");
#if defined(F_CPU)
    static_assert(false, "brio DynamicClock: F_CPU must not be defined with a "
                         "runtime clock - the rate has one truth, Clock::hz");
#endif

    static uint32_t hz() { return hz_; }

    /// The DISCRETE-RATE surface (what avrxt/delay.hpp dispatches on).
    /// A dynamic clock only ever runs at source_hz / clock_divs[i]: the
    /// whole set is known from the TYPE, so per-rate arithmetic can be
    /// expanded and folded per index at compile time, and the one
    /// runtime fact is WHICH index is current - a byte, not a division.
    static constexpr uint8_t rate_count = clock_div_count;
    static constexpr uint32_t rate_hz(uint8_t i) {
        return source_hz / clock_divisor(clock_divs[i]);
    }
    static uint8_t rate_index() { return idx_; }

    /// Is U one of the users that set() rebases? Drivers assert this in
    /// init(clock): a clocked driver forgotten in the list would keep
    /// running at the old rate in silence - a compile error instead.
    template <typename U>
    static constexpr bool rebases = (std::same_as<U, Users> || ...);

    /// Boot configuration, then Boot::hz. See Clock::init for the return.
    static bool init() {
        const bool ok = Boot::init();
        hz_ = Boot::hz;
        idx_ = 0;
        return ok;
    }

    /// Can this clock run at `hz` (source_hz / an existing prescaler)?
    static constexpr bool can_run_at(uint32_t hz) { return div_for(source_hz, hz).ok; }

    /// Switch to a rate known at compile time (checked: unreachable
    /// rates do not compile).
    template <uint32_t hz>
    static void set() {
        static_assert(can_run_at(hz),
                      "DynamicClock: this rate is not source_hz divided by an "
                      "available main prescaler");
        apply(div_for(source_hz, hz).div, hz);
    }

    /// Switch to a rate chosen at run time; false (nothing changed) when
    /// the rate is not reachable.
    static bool set(uint32_t hz) {
        const DivFor d = div_for(source_hz, hz);
        if (!d.ok) {
            return false;
        }
        apply(d.div, hz);
        return true;
    }

private:
    /// Rebase every user for the new rate (each may first drain what
    /// it has in flight at the OLD rate), then switch the prescaler.
    /// Where the clock owns MCLKTIMEBASE it moves on the side of the
    /// switch that keeps it covering a microsecond throughout: before the
    /// prescaler when the rate rises, after it when the rate falls. One
    /// STS of a value per rate index, a table built from the type.
    static void apply(ClockDiv d, uint32_t next) {
        (Users::rebase(next), ...);
#if defined(CLKCTRL_TIMEBASE_gm)
        const uint8_t i = clock_div_index(d);
        if (next > hz_) {
            ClockTimebase::write(timebase_.v[i]);
            MainClock::prescale(d);
        } else {
            MainClock::prescale(d);
            ClockTimebase::write(timebase_.v[i]);
        }
        hz_ = next;
        idx_ = i;
#else
        MainClock::prescale(d);
        hz_ = next;
        idx_ = clock_div_index(d);
#endif
    }

#if defined(CLKCTRL_TIMEBASE_gm)
    struct TimebaseTable { uint8_t v[clock_div_count]; };
    static constexpr TimebaseTable timebase_table() {
        TimebaseTable t{};
        for (uint8_t i = 0; i < clock_div_count; ++i) {
            t.v[i] = clock_timebase_for(rate_hz(i));
        }
        return t;
    }
    static constexpr TimebaseTable timebase_ = timebase_table();
#endif

    static inline uint32_t hz_ = Boot::hz;
    static inline uint8_t idx_ = 0;
};

} // namespace brio
