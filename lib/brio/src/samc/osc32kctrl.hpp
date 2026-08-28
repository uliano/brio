/*
 * osc32kctrl.hpp
 *
 * The SAM C21 32.768 kHz oscillators controller (DS60001479M ch. 21):
 * the three slow clock roots, the one register that chooses the RTC's
 * clock, and the crystal's failure detector.
 *
 *   Osc32kctrl   the BLOCK - status, the three interrupt sources, and
 *                RTCCTRL, which is where the RTC's clock is chosen even
 *                though the RTC lives in another chapter.
 *   Osculp32k    the ultra-low-power RC: always running, never
 *                configured, only trimmed.
 *   Osc32k       the accurate internal RC: OFF at reset, and it must be
 *                given its factory trim by hand before it means
 *                anything.
 *   Xosc32k      the external crystal, with the clock-failure detector
 *                that can switch away from it on its own.
 *
 * THE FACT THAT MOST NEEDS SAYING: OSC32K IS NOT CALIBRATED UNTIL
 * SOFTWARE CALIBRATES IT. 21.5.9 is blunt - "the OSC32K calibration
 * value from the production test MUST be loaded from the NVM Software
 * Calibration Area into the OSC32K register (OSC32K.CALIB) by software
 * to achieve specified accuracy" - so an OSC32K enabled straight out of
 * reset runs at whatever an untrimmed RC happens to do. The value lives
 * in the area samc/nvm.hpp already reads, which is why `factory_calib()`
 * below exists rather than leaving each caller to remember. The bench
 * suite measures both settings, and the difference is not small.
 *
 * FOUR MORE RULES OF THIS CHAPTER, each with code behind it.
 *
 * 1. AN OSCILLATOR'S OUTPUT IS SEPARATE FROM THE OSCILLATOR. EN32K and
 *    EN1K gate the two outputs, and 21.6.4 requires the one a consumer
 *    wants to be enabled BEFORE the GCLK or the RTC is pointed at it.
 *    An enabled oscillator with both outputs off is a clock nobody can
 *    reach - a silent failure this driver refuses to let happen by
 *    accident.
 *
 * 2. CALIB IS COMMITTED ON READY. "When writing the calibration bits,
 *    the user must wait for the STATUS.OSC32KRDY bit to go high before
 *    the new value is committed to the oscillator" (21.6.4), so writing
 *    the trim and immediately measuring reads the OLD frequency.
 *
 * 3. WRTLOCK IS ONE-WAY UNTIL A POWER-ON RESET, on both internal
 *    oscillators. Not a watchdog-style hazard - the worst it does is
 *    freeze a working configuration - but a locked oscillator cannot be
 *    reconfigured by anything short of unplugging the board, so every
 *    configuring verb here checks first and refuses rather than writing
 *    into a lock.
 *
 * 4. THE RTC'S CLOCK IS CHOSEN HERE, NOT IN THE RTC. RTCCTRL.RTCSEL
 *    picks among all six oscillator outputs, and 21.6.7 asks for the RTC
 *    to be disabled before the selection changes. This header owns the
 *    register; the ordering is the RTC driver's to keep when it exists.
 *
 * ERRATA: NEITHER ITEM TOUCHING THIS CHAPTER APPLIES TO THIS SILICON,
 * and both are the kind that a careless read of the matrix would apply
 * anyway. 1.1.1 (the CFD's automatic switch does not work when XOSC32K
 * is requested by the GCLK) is marked for E/G/J revision B ONLY, and
 * this chip is rev F. 1.22.1 (the CFD cannot switch to the safe clock
 * when the input is stuck high) is marked only in the N-family row -
 * the same trap dmac.md records for the DMAC items: read the row, not
 * the column.
 *
 * NOT BUILT (docs/samc/osc32kctrl.md carries the list): the CFD's event
 * output (EVCTRL.CFDEO - no EVSYS driver on this target), and XOSC32K
 * itself is written and family-compiled but cannot be exercised here
 * because the bench board carries no 32 kHz crystal.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/nvm.hpp"

namespace brio {

/// INTFLAG / INTENSET / INTENCLR, and the first two bits are also
/// STATUS bits with the same meaning.
struct Osc32kFlag {
    static constexpr uint32_t xosc32k_ready = OSC32KCTRL_INTFLAG_XOSC32KRDY_Msk;
    static constexpr uint32_t osc32k_ready = OSC32KCTRL_INTFLAG_OSC32KRDY_Msk;
    static constexpr uint32_t clock_failure = OSC32KCTRL_INTFLAG_CLKFAIL_Msk;
    static constexpr uint32_t all =
        xosc32k_ready | osc32k_ready | clock_failure;
};

/// RTCCTRL.RTCSEL: which oscillator output clocks the RTC. Every one of
/// the six is a legal choice (21.6.7); which are AVAILABLE depends on
/// what has been enabled, and on whether the board has a crystal.
///
/// NOTE the device header's own typo, kept in view rather than copied:
/// its comment on RTCSEL_XOSC1K reads "1.024kHz from 32.768kHz INTERNAL
/// oscillator", where the value plainly means the external one.
enum class RtcClock : uint8_t {
    ulp_1k = OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K_Val,
    ulp_32k = OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K_Val,
    osc_1k = OSC32KCTRL_RTCCTRL_RTCSEL_OSC1K_Val,
    osc_32k = OSC32KCTRL_RTCCTRL_RTCSEL_OSC32K_Val,
    xosc_1k = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC1K_Val,
    xosc_32k = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K_Val,
};

/// STARTUP, shared in shape by OSC32K and XOSC32K: how many cycles the
/// output stays masked after an enable. The two chapters give different
/// cycle counts for the same field value, so the code is passed through
/// and named by its field, not by a time this header would have to
/// invent.
using Osc32kStartup = uint8_t;

// =============================================================================
// The block
// =============================================================================

struct Osc32kctrl {
    Osc32kctrl() = delete;

    static constexpr IRQn_Type irq() { return OSC32KCTRL_IRQn; }

    static uint32_t status() { return OSC32KCTRL_REGS->OSC32KCTRL_STATUS; }
    static bool osc32k_ready() { return (status() & Osc32kFlag::osc32k_ready) != 0u; }
    static bool xosc32k_ready() { return (status() & Osc32kFlag::xosc32k_ready) != 0u; }
    /// STATUS.CLKFAIL: the detector currently sees the crystal failing.
    static bool clock_failing() { return (status() & Osc32kFlag::clock_failure) != 0u; }
    /// STATUS.CLKSW: the CFD has switched the clock to the safe source.
    static bool clock_switched() {
        return (status() & OSC32KCTRL_STATUS_CLKSW_Msk) != 0u;
    }

    static uint32_t flags() { return OSC32KCTRL_REGS->OSC32KCTRL_INTFLAG; }
    static uint32_t armed() { return OSC32KCTRL_REGS->OSC32KCTRL_INTENSET; }
    static void clear_flags(uint32_t mask = Osc32kFlag::all) {
        OSC32KCTRL_REGS->OSC32KCTRL_INTFLAG = mask;
    }
    static void arm(uint32_t mask) { OSC32KCTRL_REGS->OSC32KCTRL_INTENSET = mask; }
    static void disarm(uint32_t mask) { OSC32KCTRL_REGS->OSC32KCTRL_INTENCLR = mask; }

    /// The ISR body; the app binds the handler. NOTE that this line is
    /// SHARED - IRQ 0 carries MCLK, OSCCTRL, OSC32KCTRL, PAC and SUPC
    /// together - so a handler must ask each block in turn, and this one
    /// answers only for its own.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t p = flags() & armed();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    // ---- the RTC's clock ---------------------------------------------------

    /**
     * Choose the RTC's clock. 21.6.7 asks for the RTC to be DISABLED
     * before this changes - a rule this header cannot enforce, having no
     * RTC driver to ask, and which is therefore the caller's to keep.
     * It is stated here because a silent change under a running counter
     * is exactly the kind of thing that is never noticed.
     */
    static void rtc_clock(RtcClock which) {
        OSC32KCTRL_REGS->OSC32KCTRL_RTCCTRL =
            OSC32KCTRL_RTCCTRL_RTCSEL(static_cast<uint32_t>(which));
    }
    static RtcClock rtc_clock() {
        return static_cast<RtcClock>(
            (OSC32KCTRL_REGS->OSC32KCTRL_RTCCTRL & OSC32KCTRL_RTCCTRL_RTCSEL_Msk) >>
            OSC32KCTRL_RTCCTRL_RTCSEL_Pos);
    }
};

// =============================================================================
// OSCULP32K - always there, only trimmable
// =============================================================================

/**
 * The ultra-low-power RC. It is enabled by a power-on reset and runs
 * until the next one; there is nothing to start and nothing to wait for.
 * What it has is a trim, loaded from flash factory calibration at
 * start-up and overridable - and a one-way lock.
 *
 * This is the oscillator the watchdog runs on (21.6.6), so a program
 * that trims it moves every watchdog timeout with it.
 */
struct Osculp32k {
    Osculp32k() = delete;

    static constexpr uint8_t calib_max = 0x1F;   ///< CALIB is five bits

    static uint8_t calib() {
        return static_cast<uint8_t>(
            (OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K & OSC32KCTRL_OSCULP32K_CALIB_Msk) >>
            OSC32KCTRL_OSCULP32K_CALIB_Pos);
    }

    static bool locked() {
        return (OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K &
                OSC32KCTRL_OSCULP32K_WRTLOCK_Msk) != 0u;
    }

    /// Override the factory trim. False - and nothing written - for a
    /// value past the field, or once the lock is on.
    static bool calib(uint8_t value) {
        if (value > calib_max || locked()) {
            return false;
        }
        OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K = OSC32KCTRL_OSCULP32K_CALIB(value);
        return true;
    }

    /// One way, until a power-on reset. Nothing here can undo it.
    static void lock() {
        OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K =
            OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K | OSC32KCTRL_OSCULP32K_WRTLOCK_Msk;
    }
};

// =============================================================================
// OSC32K - the accurate internal RC
// =============================================================================

struct Osc32kConfig {
    /// OSC32K.CALIB, seven bits. THE FACTORY VALUE IS NOT LOADED FOR
    /// YOU (21.5.9) - `Osc32k::factory_calib()` is where to get it.
    uint8_t calib = 0;

    /// The two outputs. At least one must be on or the oscillator runs
    /// where nothing can reach it, which `config_valid()` refuses.
    bool enable_32k = true;
    bool enable_1k = false;

    Osc32kStartup startup = 0;
    bool run_standby = false;
    /// ONDEMAND clear keeps it running across resets other than a POR
    /// (21.6.4), which is what a timekeeping application wants.
    bool on_demand = false;
};

struct Osc32k {
    Osc32k() = delete;

    static constexpr uint8_t calib_max = 0x7F;    ///< CALIB is seven bits
    static constexpr uint8_t startup_max = 0x7;

    /// The production trim, straight out of the NVM Software Calibration
    /// Area (9.4). This is the coupling 21.5.9 demands, made explicit:
    /// samc/nvm.hpp reads the area, this hands the value to the
    /// oscillator, and a caller that skips it gets an untrimmed RC.
    static uint8_t factory_calib() {
        return NvmCalibration::read().osc32k_calib();
    }

    static constexpr bool config_valid(const Osc32kConfig& c) {
        return c.calib <= calib_max && c.startup <= startup_max &&
               (c.enable_32k || c.enable_1k);
    }

    static uint32_t reg() { return OSC32KCTRL_REGS->OSC32KCTRL_OSC32K; }
    static bool enabled() { return (reg() & OSC32KCTRL_OSC32K_ENABLE_Msk) != 0u; }
    static bool locked() { return (reg() & OSC32KCTRL_OSC32K_WRTLOCK_Msk) != 0u; }
    static bool ready() { return Osc32kctrl::osc32k_ready(); }

    static uint8_t calib() {
        return static_cast<uint8_t>((reg() & OSC32KCTRL_OSC32K_CALIB_Msk) >>
                                    OSC32KCTRL_OSC32K_CALIB_Pos);
    }

    /**
     * Configure and start the oscillator, and wait until it is ready.
     *
     * The wait is not politeness: 21.6.4 says the calibration value is
     * committed to the oscillator when STATUS.OSC32KRDY goes high, so a
     * caller that measures before this returns measures the old trim.
     *
     * False - and nothing written - for an impossible configuration, a
     * locked oscillator, or a ready flag that never came.
     */
    static bool init(const Osc32kConfig& cfg, uint32_t spins = 2'000'000UL) {
        if (!config_valid(cfg) || locked()) {
            return false;
        }
        OSC32KCTRL_REGS->OSC32KCTRL_OSC32K =
            OSC32KCTRL_OSC32K_CALIB(cfg.calib) |
            OSC32KCTRL_OSC32K_STARTUP(cfg.startup) |
            (cfg.enable_32k ? OSC32KCTRL_OSC32K_EN32K_Msk : 0u) |
            (cfg.enable_1k ? OSC32KCTRL_OSC32K_EN1K_Msk : 0u) |
            (cfg.run_standby ? OSC32KCTRL_OSC32K_RUNSTDBY_Msk : 0u) |
            (cfg.on_demand ? OSC32KCTRL_OSC32K_ONDEMAND_Msk : 0u) |
            OSC32KCTRL_OSC32K_ENABLE_Msk;
        while (!ready() && spins-- != 0u) {
        }
        return ready();
    }

    /// Re-trim a running oscillator, waiting for the commit. The rest of
    /// the register is preserved, which is what makes this usable as a
    /// tuning step rather than a reconfiguration.
    static bool retrim(uint8_t value, uint32_t spins = 2'000'000UL) {
        if (value > calib_max || locked()) {
            return false;
        }
        const uint32_t kept = reg() & ~OSC32KCTRL_OSC32K_CALIB_Msk;
        Osc32kctrl::clear_flags(Osc32kFlag::osc32k_ready);
        OSC32KCTRL_REGS->OSC32KCTRL_OSC32K = kept | OSC32KCTRL_OSC32K_CALIB(value);
        while (!ready() && spins-- != 0u) {
        }
        return ready();
    }

    static void stop() {
        OSC32KCTRL_REGS->OSC32KCTRL_OSC32K =
            reg() & ~static_cast<uint32_t>(OSC32KCTRL_OSC32K_ENABLE_Msk);
    }

    /// One way, until a power-on reset.
    static void lock() {
        OSC32KCTRL_REGS->OSC32KCTRL_OSC32K = reg() | OSC32KCTRL_OSC32K_WRTLOCK_Msk;
    }
};

// =============================================================================
// XOSC32K - the external crystal, and the detector that watches it
// =============================================================================

struct Xosc32kConfig {
    /// XTALEN: a crystal between XIN32 and XOUT32 when set, an external
    /// clock on XIN32 alone when clear.
    bool crystal = true;
    bool enable_32k = true;
    bool enable_1k = false;
    Osc32kStartup startup = 0;
    bool run_standby = false;
    bool on_demand = false;
};

struct Xosc32k {
    Xosc32k() = delete;

    static constexpr uint8_t startup_max = 0x7;

    static uint16_t reg() { return OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K; }
    static bool enabled() { return (reg() & OSC32KCTRL_XOSC32K_ENABLE_Msk) != 0u; }
    static bool locked() { return (reg() & OSC32KCTRL_XOSC32K_WRTLOCK_Msk) != 0u; }
    static bool ready() { return Osc32kctrl::xosc32k_ready(); }

    static constexpr bool config_valid(const Xosc32kConfig& c) {
        return c.startup <= startup_max && (c.enable_32k || c.enable_1k);
    }

    /// Start the crystal and wait for it. A crystal's start-up is
    /// hundreds of milliseconds, so the default bound is generous and a
    /// false return means the crystal is not oscillating - which, on a
    /// board that has none, is the correct answer rather than a fault.
    static bool init(const Xosc32kConfig& cfg, uint32_t spins = 40'000'000UL) {
        if (!config_valid(cfg) || locked()) {
            return false;
        }
        OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K = static_cast<uint16_t>(
            OSC32KCTRL_XOSC32K_STARTUP(cfg.startup) |
            (cfg.crystal ? OSC32KCTRL_XOSC32K_XTALEN_Msk : 0u) |
            (cfg.enable_32k ? OSC32KCTRL_XOSC32K_EN32K_Msk : 0u) |
            (cfg.enable_1k ? OSC32KCTRL_XOSC32K_EN1K_Msk : 0u) |
            (cfg.run_standby ? OSC32KCTRL_XOSC32K_RUNSTDBY_Msk : 0u) |
            (cfg.on_demand ? OSC32KCTRL_XOSC32K_ONDEMAND_Msk : 0u) |
            OSC32KCTRL_XOSC32K_ENABLE_Msk);
        while (!ready() && spins-- != 0u) {
        }
        return ready();
    }

    static void stop() {
        OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K =
            static_cast<uint16_t>(reg() & ~OSC32KCTRL_XOSC32K_ENABLE_Msk);
    }

    static void lock() {
        OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K =
            static_cast<uint16_t>(reg() | OSC32KCTRL_XOSC32K_WRTLOCK_Msk);
    }

    // ---- the clock failure detector ---------------------------------------

    /**
     * CFDCTRL: watch the crystal against OSCULP32K and fall back to it
     * when the crystal stops. `prescaler` is CFDPRESC, the divider on
     * the monitoring clock.
     *
     * Neither erratum touching this feature applies to this silicon -
     * 1.1.1 is revision B only and 1.22.1 is the N family only - but
     * both are about the SWITCH failing rather than the detection, so a
     * program that relies on the automatic fallback should still check
     * `Osc32kctrl::clock_switched()` rather than assume it happened.
     */
    static void failure_detector(bool on, bool prescale = false) {
        OSC32KCTRL_REGS->OSC32KCTRL_CFDCTRL = static_cast<uint8_t>(
            (on ? OSC32KCTRL_CFDCTRL_CFDEN_Msk : 0u) |
            (prescale ? OSC32KCTRL_CFDCTRL_CFDPRESC_Msk : 0u));
    }
    static bool failure_detector() {
        return (OSC32KCTRL_REGS->OSC32KCTRL_CFDCTRL &
                OSC32KCTRL_CFDCTRL_CFDEN_Msk) != 0u;
    }

    /// SWBACK: go back to the crystal after a failure, once it is
    /// oscillating again. Self-clearing.
    static void switch_back() {
        OSC32KCTRL_REGS->OSC32KCTRL_CFDCTRL =
            OSC32KCTRL_REGS->OSC32KCTRL_CFDCTRL | OSC32KCTRL_CFDCTRL_SWBACK_Msk;
    }
};

} // namespace brio
