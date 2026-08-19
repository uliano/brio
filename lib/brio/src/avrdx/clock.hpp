/*
 * clock.hpp
 *
 * The AVR DA/DB clock controller (CLKCTRL, DS40002247B ch. 12) in two
 * strata, as docs/design/clkctrl.md describes:
 *
 *  RESOURCES - one monostate per block of the controller, each a thin
 *  typed view of its registers (CCP-protected writes, status bits):
 *    Oschf        internal HF oscillator: 1/2/3/4/8/12/16/20/24 MHz,
 *                 RUNSTDBY, manual tune (-32..+31 steps of ~0.4 %),
 *                 auto-tune against a 32.768 kHz crystal
 *    Osc32k       internal 32.768 kHz ULP oscillator (+-10 %): RUNSTDBY
 *    Xosc32k      32.768 kHz crystal on PF0/PF1 or external clock on PF0:
 *                 enable, source, start-up time, low-power mode, RUNSTDBY
 *    Xoschf       HF crystal on PA0/PA1 (4-24 MHz) or external clock on
 *                 PA0 (up to 32 MHz): enable, source, frequency range,
 *                 start-up time, RUNSTDBY (DB only)
 *    Pll          x2 / x3 from OSCHF or XOSCHF, 16-24 MHz in, 32-48 MHz
 *                 out - it clocks the TCD only, never the main clock
 *    MainClock    CLKSEL (OSCHF / OSC32K / XOSC32K / EXTCLK), the 1..64
 *                 prescaler, CLKOUT on PA7, the "switching" status
 *    ClockFailure the clock failure detector: source, interrupt (normal
 *                 or NMI), test bit, flag, ISR body
 *
 *  TASKS - what an application names:
 *    Clock<source, source_hz, div>   the static main clock: ONE constexpr
 *                 truth `hz` every driver derives from; init() composes the
 *                 resources and reports whether the requested source runs
 *    DynamicClock<Boot, Users...>    the runtime regime: set<hz>()/set(hz)
 *                 move the prescaler and rebase the listed users first
 *                 (docs/design/clock.md)
 *
 * Facts that shape the code (12.3, 39.10, errata DS80000915F 2.5.x):
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
 * The RTC's own clock (CLK_RTC: OSC32K or XOSC32K) is selected by the
 * RTC driver (Ticker); CLKCTRL only provides the oscillators - start
 * Xosc32k here before Ticker::init() when a 32k crystal is fitted.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

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

// ---- OSCHF ------------------------------------------------------------------

/// OSCHF FRQSEL code for a rate it can produce, 0xFF otherwise.
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

/// The internal high-frequency oscillator.
struct Oschf {
    Oschf() = delete;

    /// Set the output frequency (one oschf_frqsel() can produce); keeps
    /// the RUNSTDBY and AUTOTUNE bits. Takes effect immediately, also
    /// while OSCHF is the main clock.
    static void set_hz(uint32_t hz) {
        const uint8_t keep = static_cast<uint8_t>(CLKCTRL.OSCHFCTRLA &
                                                  (CLKCTRL_RUNSTDBY_bm | CLKCTRL_AUTOTUNE_bm));
        _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, static_cast<uint8_t>(oschf_frqsel(hz) | keep));
    }
    static void run_standby(bool on) { flag(CLKCTRL_RUNSTDBY_bm, on); }

    /// Auto-tune the frequency against a running 32.768 kHz crystal
    /// (Xosc32k must be started and stable). Locks the manual tune.
    static void autotune(bool on) { flag(CLKCTRL_AUTOTUNE_bm, on); }

    /// Manual tune: -32 .. +31 steps of ~0.4 % (6-bit two's complement
    /// in OSCHFTUNE; ignored while AUTOTUNE is on).
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
/// clock on PF0. Start-up ~300 ms typ. (1 s in low-power mode): enable
/// it early and check stable() before selecting it for the RTC or as
/// the main clock.
struct Xosc32k {
    Xosc32k() = delete;

    /// Crystal on PF0/PF1.
    static void start_crystal(Xosc32kStartup sut = Xosc32kStartup::cycles64k,
                              bool low_power = false, bool run_standby = false) {
        _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm |
            (static_cast<uint8_t>(sut) << CLKCTRL_CSUT_gp) |
            (low_power ? CLKCTRL_LPMODE_bm : 0) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// External 32.768 kHz clock on PF0 (start-up two cycles).
    static void start_external(bool run_standby = false) {
        _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm | CLKCTRL_SEL_bm | (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// Off; PF0/PF1 return to the PORT.
    static void stop() { _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, 0); }
    static bool enabled() { return (CLKCTRL.XOSC32KCTRLA & CLKCTRL_ENABLE_bm) != 0; }
    static bool stable() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm) != 0; }
    /// Wait for stable() with a bound (a few cycles per spin; a 1 s
    /// crystal start-up at 24 MHz needs millions).
    static bool wait_stable(uint32_t spins = 0x00FFFFFFu) {
        return clkctrl_wait(CLKCTRL_XOSC32KS_bm, spins);
    }
};

// ---- XOSCHF -----------------------------------------------------------------

#if defined(CLKCTRL_XOSCHFCTRLA)
enum class XoschfStartup : uint8_t { cycles256 = 0, cycles1k = 1, cycles4k = 2 };

/// The high-frequency crystal oscillator on PA0/PA1 (4-24 MHz), or an
/// external clock on PA0 (up to 32 MHz). DB family only.
struct Xoschf {
    Xoschf() = delete;

    /// Frequency range code for a crystal rate (the oscillator's drive).
    static constexpr uint8_t range_for(uint32_t hz) {
        return hz <= 8'000'000 ? CLKCTRL_FRQRANGE_8M_gc
             : hz <= 16'000'000 ? CLKCTRL_FRQRANGE_16M_gc
             : hz <= 24'000'000 ? CLKCTRL_FRQRANGE_24M_gc
                                : CLKCTRL_FRQRANGE_32M_gc;
    }

    /// Crystal on PA0/PA1 at `hz`; start-up 4k cycles by default.
    static void start_crystal(uint32_t hz, XoschfStartup sut = XoschfStartup::cycles4k,
                              bool run_standby = true) {
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm | static_cast<uint8_t>(CLKCTRL_SELHF_XTAL_gc) | range_for(hz) |
            (static_cast<uint8_t>(sut) << CLKCTRL_CSUTHF_gp) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// External clock on PA0 (start-up two cycles).
    static void start_external(uint32_t hz, bool run_standby = true) {
        _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, static_cast<uint8_t>(
            CLKCTRL_ENABLE_bm | static_cast<uint8_t>(CLKCTRL_SELHF_EXTCLOCK_gc) | range_for(hz) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    /// Off; PA0/PA1 return to the PORT.
    static void stop() { _PROTECTED_WRITE(CLKCTRL.XOSCHFCTRLA, 0); }
    static bool enabled() { return (CLKCTRL.XOSCHFCTRLA & CLKCTRL_ENABLE_bm) != 0; }
    /// Crystal stable / external clock running (EXTS). Errata 2.5.1 (A4):
    /// not set for EXTCLK + RUNSTDBY without a requester.
    static bool stable() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm) != 0; }
    static bool wait_stable(uint32_t spins = 0xFFFFu) { return clkctrl_wait(CLKCTRL_EXTS_bm, spins); }
};
#endif

// ---- PLL --------------------------------------------------------------------

enum class PllSource : uint8_t { oschf = 0, xoschf = 1 };
enum class PllMultiplier : uint8_t {
    off = CLKCTRL_MULFAC_DISABLE_gc, x2 = CLKCTRL_MULFAC_2x_gc, x3 = CLKCTRL_MULFAC_3x_gc
};

/// The PLL: 16-24 MHz in, 32-48 MHz out, lock ~10 us. It clocks the TCD
/// only (CLK_TCD); it cannot be the main clock. It runs only when the
/// TCD requests it: PLLS is not observable without a requester (and
/// never sets with RUNSTDBY and no requester: errata 2.5.3, A4/A5);
/// it does not run from an XOSCHF crystal, only from an external clock
/// on PA0 (2.5.4, A4/A5).
struct Pll {
    Pll() = delete;
    static void start(PllSource src, PllMultiplier mul, bool run_standby = false) {
        _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, static_cast<uint8_t>(
            static_cast<uint8_t>(mul) |
            (src == PllSource::xoschf ? CLKCTRL_SOURCE_bm : 0) |
            (run_standby ? CLKCTRL_RUNSTDBY_bm : 0)));
    }
    static void stop() { _PROTECTED_WRITE(CLKCTRL.PLLCTRLA, 0); }
    static bool locked() { return (CLKCTRL.MCLKSTATUS & CLKCTRL_PLLS_bm) != 0; }
};

// ---- main clock -------------------------------------------------------------

/// Main prescaler (CLKCTRL.MCLKCTRLB). div1 = prescaler disabled.
enum class ClockDiv : uint8_t {
    div1 = 0,
    div2 = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm,
    div4 = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm,
    div6 = CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm,
    div8 = CLKCTRL_PDIV_8X_gc | CLKCTRL_PEN_bm,
    div10 = CLKCTRL_PDIV_10X_gc | CLKCTRL_PEN_bm,
    div12 = CLKCTRL_PDIV_12X_gc | CLKCTRL_PEN_bm,
    div16 = CLKCTRL_PDIV_16X_gc | CLKCTRL_PEN_bm,
    div24 = CLKCTRL_PDIV_24X_gc | CLKCTRL_PEN_bm,
    div32 = CLKCTRL_PDIV_32X_gc | CLKCTRL_PEN_bm,
    div48 = CLKCTRL_PDIV_48X_gc | CLKCTRL_PEN_bm,
    div64 = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm,
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

/// The main prescaler that turns source_hz into hz exactly, or div1
/// with `ok` false when no prescaler does.
struct DivFor { ClockDiv div; bool ok; };
constexpr DivFor div_for(uint32_t source_hz, uint32_t hz) {
    constexpr ClockDiv all[] = {ClockDiv::div1, ClockDiv::div2, ClockDiv::div4,
                                ClockDiv::div6, ClockDiv::div8, ClockDiv::div10,
                                ClockDiv::div12, ClockDiv::div16, ClockDiv::div24,
                                ClockDiv::div32, ClockDiv::div48, ClockDiv::div64};
    for (ClockDiv d : all) {
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

// ---- clock failure detection ----------------------------------------------------

enum class CfdSource : uint8_t {
    main = CLKCTRL_CFDSRC_CLKMAIN_gc,
    xoschf = CLKCTRL_CFDSRC_XOSCHF_gc,
    xosc32k = CLKCTRL_CFDSRC_XOSC32K_gc,
};

/// The clock failure detector (12.3.7): watches one source for edges;
/// on silence it raises the CFD flag, optionally an interrupt (regular
/// or NMI), and - when the MAIN clock is watched - forces CLKSEL back to
/// the start-up source (OSCHF at its reset frequency, 4 MHz; CLKOUT
/// disabled). Once enabled with the NMI interrupt, its registers are
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

// =============================================================================
// Tasks
// =============================================================================

enum class ClockSource : uint8_t {
    internal,   ///< OSCHF at a rate it can produce
    crystal,    ///< XOSCHF crystal on PA0/PA1 (DB only)
    external,   ///< external clock on PA0 (XOSCHF in EXTCLK mode, DB only)
    osc32k,     ///< the internal 32.768 kHz oscillator as main clock
    xosc32k,    ///< the 32.768 kHz crystal (or clock on PF0) as main clock
};

template <ClockSource src, uint32_t src_hz, ClockDiv div = ClockDiv::div1>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t source_hz = src_hz;
    static constexpr uint32_t divisor = clock_divisor(div);
    static constexpr uint32_t hz = src_hz / divisor;   ///< CLK_PER = CLK_CPU
    static constexpr bool is_static = true;

    static constexpr bool hf = src == ClockSource::internal || src == ClockSource::crystal ||
                               src == ClockSource::external;
    static_assert(divisor > 0, "invalid main prescaler");
    static_assert(!hf || oschf_frqsel(src_hz) != 0xFF,
                  "brio Clock: a high-frequency source rate must be one OSCHF can "
                  "produce (1/2/3/4/8/12/16/20/24 MHz) - it is the fallback that keeps "
                  "Clock::hz true when an external source fails to start");
    static_assert(hf || src_hz == 32'768, "brio Clock: the 32 kHz sources are 32768 Hz");
    static_assert(src_hz % divisor == 0, "source rate not divisible by the prescaler");
    static_assert(hz <= 24'000'000, "CLK_MAIN must not exceed 24 MHz");
#if defined(F_CPU)
    static_assert(hz == F_CPU,
                  "brio Clock: this build defines F_CPU with a value different "
                  "from Clock::hz. Either unflag -DF_CPU (this project does) or "
                  "make it equal: never two truths");
#endif
#if !defined(CLKCTRL_XOSCHFCTRLA)
    static_assert(src != ClockSource::crystal && src != ClockSource::external,
                  "brio Clock: XOSCHF exists only on the DB family");
#endif

    /// Bring CLK_PER to `hz`. Returns true when running from the
    /// requested source, false when an external source failed to start
    /// and the fallback is running instead: OSCHF at the same rate for
    /// the high-frequency sources, OSC32K for xosc32k. Call first in
    /// main().
    static bool init() {
        if constexpr (hf) {
            // Baseline and fallback: OSCHF at the target rate, prescaled.
            Oschf::set_hz(src_hz);
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
                return false;
#endif
            }
        } else {
            MainClock::prescale(div);
            if constexpr (src == ClockSource::osc32k) {
                Osc32k::run_standby(true);
                (void)clkctrl_wait(CLKCTRL_OSC32KS_bm, 0xFFFFu);
                return MainClock::select(MainSource::osc32k);
            } else {
                if (!Xosc32k::enabled()) {
                    Xosc32k::start_crystal();
                }
                if (!Xosc32k::wait_stable()) {
                    Xosc32k::stop();
                    (void)MainClock::select(MainSource::osc32k);   // fallback: the internal 32k
                    return false;
                }
                return MainClock::select(MainSource::xosc32k);
            }
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
                         "runtime clock (build_unflags = -DF_CPU)");
#endif

    static uint32_t hz() { return hz_; }

    /// Is U one of the users that set() rebases? Drivers assert this in
    /// init(clock): a clocked driver forgotten in the list would keep
    /// running at the old rate in silence - a compile error instead.
    template <typename U>
    static constexpr bool rebases = (std::same_as<U, Users> || ...);

    /// Boot configuration, then Boot::hz. See Clock::init for the return.
    static bool init() {
        const bool ok = Boot::init();
        hz_ = Boot::hz;
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
    static void apply(ClockDiv d, uint32_t next) {
        (Users::rebase(next), ...);
        MainClock::prescale(d);
        hz_ = next;
    }

    static inline uint32_t hz_ = Boot::hz;
};

} // namespace brio
