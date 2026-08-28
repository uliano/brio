/*
 * clock.hpp
 *
 * The SAM C21 clock tree in two strata, the same split avrdx/clock.hpp
 * uses (docs/design/clock.md):
 *
 *  RESOURCES - one monostate per block, a thin typed view of its
 *  registers with the synchronization discipline built in:
 *    Oscctrl      the BLOCK (OSCCTRL, ch. 20): the STATUS register all
 *                 three oscillators report into, the seven interrupt
 *                 sources behind one shared vector, and the CFD's event
 *                 output enable
 *    Osc48m       the internal 48 MHz oscillator (OSCCTRL, ch. 20):
 *                 ENABLE/ONDEMAND/RUNSTDBY, the 1..16 output divider
 *                 with its OSC48MSYNCBUSY wait, start-up time, ready
 *    Xosc         the external multipurpose crystal oscillator, 0.4 to
 *                 32 MHz: crystal or external clock, the gain the
 *                 chapter makes mandatory, the start-up counter, and
 *                 the clock failure detector with its safe clock
 *    Fdpll        the fractional 96 MHz DPLL: three reference sources,
 *                 the LDR/LDRFRAC ratio with an actual_hz readback, the
 *                 output prescaler, lock and its time-out
 *    Gclk<n>      one of the nine generic clock generators (GCLK,
 *                 ch. 16): source, 1..65536 divider in both DIVSEL
 *                 regimes, output to a pin, RUNSTDBY, with its
 *                 GCLK_SYNCBUSY wait
 *    GclkChannel  the 41 PERIPHERAL clock channels: which generator
 *                 feeds a given peripheral, the write lock
 *    Mclk         the main clock controller (MCLK, ch. 17): CPUDIV and
 *                 the AHB/APB bus-clock masks
 *
 *  TASK - what an application names:
 *    Clock<source, hz>   the static main clock: ONE constexpr truth
 *                 `hz` every driver derives from (there is no F_CPU in
 *                 this build, exactly as on AVR); init() composes the
 *                 resources and reports whether the requested source
 *                 really runs.
 *
 * SCOPE, honestly. All three OSCCTRL roots are now RESOURCES with their
 * full register description; what is still single-rooted is the TASK.
 * `Clock<source, hz>` implements ClockSource::internal (OSC48M) only:
 * pointing CLK_MAIN at the crystal or at the DPLL is a main-clock DESIGN
 * decision - which rate is the compile-time truth, who is told when it
 * moves, what a DynamicClock looks like on a target whose peripherals
 * each have a generator of their own - and that decision is not this
 * file's to take by accident. The resources below are what the decision
 * will be built out of, and the bench suite proves the CPU really can
 * run from the DPLL through them. The 32 kHz roots live in
 * samc/osc32kctrl.hpp; see ticker.hpp for the one consequence a
 * DynamicClock has to remember when it arrives.
 *
 * Facts that shape the code (DS60001479M ch. 20, 16, 17, 27, 45.11 and
 * errata DS80000740S, silicon rev F on the bench chip):
 *  - out of reset the device runs OSC48M divided by 12 = 4 MHz, and
 *    GCLK generator 0 is already sourced from OSC48M and enabled - which
 *    is why a bare "set the divider" is enough to reach 48 MHz and why
 *    the rest of init() is confirmation rather than construction;
 *  - flash reads limit the CPU frequency per wait state (table 45-41,
 *    VDD > 2.7 V column: 19 MHz at 0 WS, 38 at 1, 64 at 2), and 27.5.2
 *    orders the wait states adapted BEFORE a frequency rise (and after a
 *    fall, which is why set_for_hz() below looks at the current value);
 *  - erratum 1.2.2: writing OSC48MDIV while OSC48M runs UNREQUESTED
 *    leaves OSC48MSYNCBUSY.OSC48MDIV stuck - clear ONDEMAND first;
 *  - erratum 1.2.3: a rare no-start at power-up on parts built before
 *    2025-01, avoided by keeping ENABLE = 1 and ONDEMAND = 0. Both are
 *    what init() writes, which is also the erratum 1.2.2 mitigation;
 *  - erratum 1.25.1, LIVE ON EVERY REVISION: below 25 C the FDPLL
 *    reports SPURIOUS unlocks, and because the lock signal gates the
 *    output clock, the DPLL's consumers lose their clock for the
 *    duration. The workaround is DPLLCTRLB.LBYPASS, which ungates the
 *    output - `FdpllConfig::lock_bypass` defaults to TRUE for exactly
 *    that reason, and a caller who wants the gate back has to ask;
 *  - erratum 1.3.3: an on-the-fly ratio change does NOT set
 *    STATUS.DPLLLDRTO although INTFLAG.DPLLLDRTO does rise, so
 *    `ratio_updated()` reads the INTFLAG;
 *  - erratum 1.3.4: an on-the-fly ratio change needs GCLK_DPLL_32K
 *    running (the block's internal lock timer), which is a peripheral
 *    channel of its own - `Fdpll::lock_timer_clock()` is the verb, and
 *    `set_ratio()` says so where it can only be a caller obligation.
 *
 * A NOTE ON NAMES. Chapter 20's register summary calls the XOSC failure
 * bits STATUS.CLKFAIL and STATUS.CLKSW; the device header calls them
 * XOSCFAIL and XOSCCKSW. The header wins in code (house rule), and the
 * verbs here are named for what they mean.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/nvm.hpp"

namespace brio {

// =============================================================================
// Resources
// =============================================================================

/// Bounded spin on a synchronization or status bit. Every wait in this
/// file is bounded: a clock that never becomes ready must be reported,
/// never hung on.
inline bool clock_wait(const volatile uint32_t& reg, uint32_t mask, bool want_set,
                       uint32_t spins) {
    while (spins-- != 0u) {
        if (((reg & mask) != 0u) == want_set) {
            return true;
        }
    }
    return false;
}

// The flash read wait states used below are `FlashWaitStates`, and they
// live in samc/nvm.hpp: CTRLB.RWS is NVMCTRL's register. The QUESTION is
// this file's, because nothing may raise the CPU frequency without
// answering it first - so the type is included here and called here, and
// owned there.

/// OSC48M output divider ratio -> the OSC48MDIV.DIV field (n divides by
/// n + 1). Only ratios that divide 48 MHz EXACTLY are named: Clock::hz is
/// a truth, and 48/5 = 9.6 MHz cannot be spelled in whole hertz.
constexpr uint8_t osc48m_div_field(uint32_t ratio) {
    return static_cast<uint8_t>(ratio - 1u);
}

/// The field value producing `hz`, or 0xFF when no exact ratio does.
constexpr uint8_t osc48m_div_for(uint32_t hz) {
    for (uint32_t n = 1; n <= 16; ++n) {
        if (48'000'000UL % n == 0u && 48'000'000UL / n == hz) {
            return osc48m_div_field(n);
        }
    }
    return 0xFF;
}

/// The internal 48 MHz oscillator (20.6.4). It is the reset-time source
/// of GCLK generator 0, so on a running program it is by definition
/// requested; the ONDEMAND/ENABLE discipline below matters exactly when
/// it is not.
struct Osc48m {
    Osc48m() = delete;

    static bool enabled() {
        return (OSCCTRL_REGS->OSCCTRL_OSC48MCTRL & OSCCTRL_OSC48MCTRL_ENABLE_Msk) != 0u;
    }
    static void enable(bool on) { set_ctrl_bit(OSCCTRL_OSC48MCTRL_ENABLE_Msk, on); }

    /// ONDEMAND: the oscillator runs only while some generator asks for
    /// it. Cleared by Clock::init() and left cleared - erratum 1.2.3's
    /// mitigation, and the precondition erratum 1.2.2 needs before any
    /// divider write.
    static bool on_demand() {
        return (OSCCTRL_REGS->OSCCTRL_OSC48MCTRL & OSCCTRL_OSC48MCTRL_ONDEMAND_Msk) != 0u;
    }
    static void on_demand(bool on) { set_ctrl_bit(OSCCTRL_OSC48MCTRL_ONDEMAND_Msk, on); }

    static void run_standby(bool on) { set_ctrl_bit(OSCCTRL_OSC48MCTRL_RUNSTDBY_Msk, on); }

    /// STATUS.OSC48MRDY: the output is stable and usable.
    static bool ready() {
        return (OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_OSC48MRDY_Msk) != 0u;
    }
    static bool wait_ready(uint32_t spins = 0xFFFFu) {
        return clock_wait(OSCCTRL_REGS->OSCCTRL_STATUS, OSCCTRL_STATUS_OSC48MRDY_Msk,
                          true, spins);
    }

    /// Output divider, as the RATIO 1..16 (the register holds ratio - 1).
    /// Refuses a ratio outside the field and reports a synchronization
    /// that never completed - see erratum 1.2.2 for how that happens.
    static bool divider(uint32_t ratio, uint32_t spins = 0xFFFFu) {
        if (ratio < 1u || ratio > 16u) {
            return false;
        }
        OSCCTRL_REGS->OSCCTRL_OSC48MDIV = OSCCTRL_OSC48MDIV_DIV(osc48m_div_field(ratio));
        return wait_sync(spins);
    }
    static uint32_t divider() {
        return (OSCCTRL_REGS->OSCCTRL_OSC48MDIV & OSCCTRL_OSC48MDIV_DIV_Msk) + 1u;
    }

    static bool sync_busy() {
        return (OSCCTRL_REGS->OSCCTRL_OSC48MSYNCBUSY &
                OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV_Msk) != 0u;
    }
    static bool wait_sync(uint32_t spins = 0xFFFFu) {
        return clock_wait(OSCCTRL_REGS->OSCCTRL_OSC48MSYNCBUSY,
                          OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV_Msk, false, spins);
    }

    /// Start-up time in OSC48M cycles before the output is released,
    /// as the STUP field value (20.8.10).
    static void startup(uint8_t stup) {
        OSCCTRL_REGS->OSCCTRL_OSC48MSTUP = OSCCTRL_OSC48MSTUP_STARTUP(stup);
    }

private:
    static void set_ctrl_bit(uint8_t mask, bool on) {
        const uint8_t v = OSCCTRL_REGS->OSCCTRL_OSC48MCTRL;
        OSCCTRL_REGS->OSCCTRL_OSC48MCTRL =
            static_cast<uint8_t>(on ? (v | mask) : (v & static_cast<uint8_t>(~mask)));
    }
};

// =============================================================================
// OSCCTRL, the block: one status register, one interrupt vector
// =============================================================================

/// OSCCTRL INTFLAG / INTENSET / INTENCLR. The first four are also STATUS
/// bits with the same meaning; STATUS additionally carries XOSCCKSW,
/// which has no flag because a switch is not an event.
///
/// NOTE the device header's own asymmetry, kept in view rather than
/// smoothed over: the time-out bit is INTFLAG.DPLLLTO but STATUS.DPLLTO.
struct OscctrlFlag {
    static constexpr uint32_t xosc_ready = OSCCTRL_INTFLAG_XOSCRDY_Msk;
    static constexpr uint32_t xosc_fail = OSCCTRL_INTFLAG_XOSCFAIL_Msk;
    static constexpr uint32_t osc48m_ready = OSCCTRL_INTFLAG_OSC48MRDY_Msk;
    static constexpr uint32_t dpll_lock_rise = OSCCTRL_INTFLAG_DPLLLCKR_Msk;
    static constexpr uint32_t dpll_lock_fall = OSCCTRL_INTFLAG_DPLLLCKF_Msk;
    static constexpr uint32_t dpll_timeout = OSCCTRL_INTFLAG_DPLLLTO_Msk;
    static constexpr uint32_t dpll_ratio_done = OSCCTRL_INTFLAG_DPLLLDRTO_Msk;
    static constexpr uint32_t all = OSCCTRL_INTFLAG_Msk;
};

/**
 * The oscillators controller as a block: the STATUS register all three
 * roots report into, the interrupt surface, and the CFD's event output.
 *
 * There is no reset and no enable here - OSCCTRL is always powered and
 * its APB clock is on out of reset (17.6.2.6). What the block owns is
 * the shared vocabulary; each oscillator's own control register belongs
 * to its own resource below.
 */
struct Oscctrl {
    Oscctrl() = delete;

    /// IRQ 0 is SHARED - MCLK, OSCCTRL, OSC32KCTRL, PAC and SUPC all
    /// arrive on it - so a handler must ask each block in turn.
    static constexpr IRQn_Type irq() { return OSCCTRL_IRQn; }

    static uint32_t status() { return OSCCTRL_REGS->OSCCTRL_STATUS; }

    static uint32_t flags() { return OSCCTRL_REGS->OSCCTRL_INTFLAG; }
    static uint32_t armed() { return OSCCTRL_REGS->OSCCTRL_INTENSET; }
    static void clear_flags(uint32_t mask = OscctrlFlag::all) {
        OSCCTRL_REGS->OSCCTRL_INTFLAG = mask;
    }
    static void arm(uint32_t mask) { OSCCTRL_REGS->OSCCTRL_INTENSET = mask; }
    static void disarm(uint32_t mask) { OSCCTRL_REGS->OSCCTRL_INTENCLR = mask; }

    /// The ISR body; the app binds the handler. Answers only for this
    /// block, because IRQ 0 is shared - see irq().
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t p = flags() & armed();
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    /// EVCTRL.CFDEO: the clock-failure event on the EVSYS. The generator
    /// code below is what an event channel is pointed at.
    static void failure_event(bool on) {
        OSCCTRL_REGS->OSCCTRL_EVCTRL =
            static_cast<uint8_t>(on ? OSCCTRL_EVCTRL_CFDEO_Msk : 0u);
    }
    static bool failure_event() {
        return (OSCCTRL_REGS->OSCCTRL_EVCTRL & OSCCTRL_EVCTRL_CFDEO_Msk) != 0u;
    }

    /// This block's one EVSYS generator code. The driver publishes it;
    /// samc/evsys.hpp owns the fabric and knows nothing of oscillators.
    /// 20.6.8: the event is NOT generated once the CFD has switched to
    /// the safe clock, only on the failure itself.
    static constexpr uint8_t failure_generator = EVENT_ID_GEN_OSCCTRL_XOSC_FAIL;
};

// =============================================================================
// XOSC - the external multipurpose crystal oscillator, and its watchdog
// =============================================================================

/// XOSCCTRL.GAIN (20.8.5). The value is a RECOMMENDED MAXIMUM crystal
/// frequency, not a range: pick the smallest one that covers the crystal.
/// The chapter's table says 32 MHz for the top code where the device
/// header's enumerator is named GAIN30; the numbers below are the
/// chapter's, the codes are the header's.
enum class XoscGain : uint8_t {
    up_to_2mhz = OSCCTRL_XOSCCTRL_GAIN_GAIN2_Val,
    up_to_4mhz = OSCCTRL_XOSCCTRL_GAIN_GAIN4_Val,
    up_to_8mhz = OSCCTRL_XOSCCTRL_GAIN_GAIN8_Val,
    up_to_16mhz = OSCCTRL_XOSCCTRL_GAIN_GAIN16_Val,
    up_to_32mhz = OSCCTRL_XOSCCTRL_GAIN_GAIN30_Val,
};

/// The smallest GAIN code that covers `hz`, so a caller states the
/// crystal it soldered and not a table lookup. A frequency past the
/// oscillator's own 32 MHz ceiling still returns the top code - it is
/// `Xosc::config_valid()` that refuses it, in one place.
constexpr XoscGain xosc_gain_for(uint32_t hz) {
    if (hz <= 2'000'000UL) return XoscGain::up_to_2mhz;
    if (hz <= 4'000'000UL) return XoscGain::up_to_4mhz;
    if (hz <= 8'000'000UL) return XoscGain::up_to_8mhz;
    if (hz <= 16'000'000UL) return XoscGain::up_to_16mhz;
    return XoscGain::up_to_32mhz;
}

/// XOSCCTRL.STARTUP: the output stays masked for 2^n OSCULP32K cycles
/// plus three XOSC cycles (table 20-5). The field value is passed
/// through - the table's microseconds assume a nominal OSCULP32K, and
/// this family's OSCULP32K measures several per mille fast
/// (osc32kctrl.md), so a time named here would be a number this header
/// cannot keep.
using XoscStartup = uint8_t;

/// The approximate masking time of a STARTUP code, in microseconds, as
/// table 20-5 prints it. Approximate is the chapter's own word: the
/// counter runs on OSCULP32K.
constexpr uint32_t xosc_startup_us(XoscStartup code) {
    // 2^code OSCULP32K cycles at a nominal 32.768 kHz, rounded as the
    // table rounds: 31, 61, 122, 244, ... doubling each step.
    uint32_t us = 31;
    for (uint8_t i = 0; i < code; ++i) {
        us *= 2u;
    }
    return us;
}

/// CFDPRESC: the safe clock is OSC48M divided by 2^CFDPRESC (20.8.6),
/// and 20.6.3 requires it to be no faster than the monitored XOSC.
using CfdPrescaler = uint8_t;

/// The smallest CFDPRESC whose safe clock is at or below `xosc_hz`,
/// given the OSC48M output rate actually in force. Returns 7 (the
/// field's maximum) when even that is too fast, which is the honest
/// answer for a very slow crystal.
constexpr CfdPrescaler cfd_prescaler_for(uint32_t xosc_hz, uint32_t osc48m_hz) {
    for (uint8_t p = 0; p < 7; ++p) {
        if ((osc48m_hz >> p) <= xosc_hz) {
            return p;
        }
    }
    return 7;
}

struct XoscConfig {
    /// The crystal or external clock frequency in hertz. It is not
    /// written anywhere - the oscillator has no idea what is attached -
    /// but it is what GAIN and the CFD prescaler are chosen from, and
    /// what config_valid() judges against the chapter's 0.4..32 MHz.
    uint32_t hz = 0;

    /// XTALEN: a crystal across XIN/XOUT when set, an external clock on
    /// XIN alone when clear. In external-clock mode XOUT stays a GPIO
    /// (20.6.2), and GAIN/AMPGC are meaningless.
    bool crystal = true;

    /// GAIN is MANDATORY in crystal mode - 20.8.5 says so twice, once
    /// under AMPGC and once under GAIN itself - so it is not defaulted
    /// but derived from `hz` unless the caller overrides it.
    bool gain_from_hz = true;
    XoscGain gain = XoscGain::up_to_2mhz;

    /// AMPGC: automatic amplitude control, lower power in most cases.
    /// It does NOT replace GAIN.
    bool automatic_gain = false;

    XoscStartup startup = 0;

    /// ONDEMAND clear = the oscillator runs whether or not a generator
    /// asks for it, which is what a program that measures it needs.
    bool on_demand = false;
    bool run_standby = false;

    /// CFDEN, with the safe-clock prescaler. 20.6.3 requires OSC48M -
    /// the safe clock's source - to be running BEFORE the detector is
    /// enabled, which on this family it always is (clock.hpp's init
    /// leaves it free-running); `init()` checks anyway.
    bool failure_detector = false;
    CfdPrescaler cfd_prescaler = 0;
};

/**
 * The external multipurpose crystal oscillator (20.6.2).
 *
 * The pads are NOT this driver's to claim: 20.5.1 says XIN and XOUT are
 * taken over by OSCCTRL when the oscillator is enabled, and given back
 * when it is disabled - there is no PORT configuration to make and none
 * is made here. In external-clock mode only XIN is taken.
 *
 * THE TEARDOWN ORDER IS THE ONE TRAP OF THIS FILE. A generic clock
 * generator releases its old source only once the new one is ready
 * (16.6.2.6), so a generator still pointed at a STOPPED oscillator can
 * never be moved again - the GENCTRL write never synchronizes. Point
 * every generator somewhere that is running BEFORE calling stop(). The
 * comment on stop() says it again where it matters.
 */
struct Xosc {
    Xosc() = delete;

    /// The chapter's own range (20.2): 0.4 to 32 MHz.
    static constexpr uint32_t min_hz = 400'000UL;
    static constexpr uint32_t max_hz = 32'000'000UL;
    static constexpr uint8_t startup_max = 0xF;
    static constexpr uint8_t cfd_prescaler_max = 0x7;

    static constexpr bool config_valid(const XoscConfig& c) {
        return c.hz >= min_hz && c.hz <= max_hz && c.startup <= startup_max &&
               c.cfd_prescaler <= cfd_prescaler_max;
    }

    static uint16_t reg() { return OSCCTRL_REGS->OSCCTRL_XOSCCTRL; }
    static bool enabled() { return (reg() & OSCCTRL_XOSCCTRL_ENABLE_Msk) != 0u; }

    /// STATUS.XOSCRDY: the crystal is stable and usable as a source.
    static bool ready() {
        return (Oscctrl::status() & OSCCTRL_STATUS_XOSCRDY_Msk) != 0u;
    }
    /// STATUS.XOSCFAIL (the chapter's CLKFAIL): the detector sees the
    /// XOSC failing RIGHT NOW - it follows the current activity, it does
    /// not latch (20.6.3). The latching copy is INTFLAG.
    static bool failing() {
        return (Oscctrl::status() & OSCCTRL_STATUS_XOSCFAIL_Msk) != 0u;
    }
    /// STATUS.XOSCCKSW (the chapter's CLKSW): the detector has replaced
    /// the XOSC output with the safe clock. It stays set until a switch
    /// back succeeds.
    static bool switched_to_safe_clock() {
        return (Oscctrl::status() & OSCCTRL_STATUS_XOSCCKSW_Msk) != 0u;
    }

    static constexpr uint32_t cfd_word(const XoscConfig& c) {
        return OSCCTRL_CFDPRESC_CFDPRESC(c.cfd_prescaler);
    }

    static constexpr uint16_t ctrl_word(const XoscConfig& c) {
        const XoscGain g = c.gain_from_hz ? xosc_gain_for(c.hz) : c.gain;
        return static_cast<uint16_t>(
            OSCCTRL_XOSCCTRL_STARTUP(c.startup) |
            (c.crystal ? OSCCTRL_XOSCCTRL_XTALEN_Msk : 0u) |
            (c.crystal ? OSCCTRL_XOSCCTRL_GAIN(static_cast<uint32_t>(g)) : 0u) |
            ((c.crystal && c.automatic_gain) ? OSCCTRL_XOSCCTRL_AMPGC_Msk : 0u) |
            (c.failure_detector ? OSCCTRL_XOSCCTRL_CFDEN_Msk : 0u) |
            (c.run_standby ? OSCCTRL_XOSCCTRL_RUNSTDBY_Msk : 0u) |
            (c.on_demand ? OSCCTRL_XOSCCTRL_ONDEMAND_Msk : 0u) |
            OSCCTRL_XOSCCTRL_ENABLE_Msk);
    }

    /**
     * Start the oscillator and wait for STATUS.XOSCRDY.
     *
     * False - and nothing written - for a frequency or a field outside
     * the chapter, for a failure detector asked for while OSC48M (its
     * safe clock) is not ready, or for a ready flag that never came. A
     * crystal that is not oscillating is exactly that last case: a
     * bounded wait and a false return, never a hang.
     *
     * The default bound is generous because a crystal's start-up is the
     * STARTUP field's business and can be a whole second at the top
     * code.
     */
    static bool init(const XoscConfig& cfg, uint32_t spins = 40'000'000UL) {
        if (!config_valid(cfg)) {
            return false;
        }
        if (cfg.failure_detector && !Osc48m::ready()) {
            return false;   // 20.6.3: the safe clock must run FIRST
        }
        // The prescaler before CFDEN, so the safe clock is never briefly
        // faster than the crystal it stands in for.
        OSCCTRL_REGS->OSCCTRL_CFDPRESC = static_cast<uint8_t>(cfd_word(cfg));
        Oscctrl::clear_flags(OscctrlFlag::xosc_ready | OscctrlFlag::xosc_fail);
        OSCCTRL_REGS->OSCCTRL_XOSCCTRL = ctrl_word(cfg);
        while (!ready() && spins-- != 0u) {
        }
        return ready();
    }

    /// The same thing with the configuration known at compile time, so
    /// an impossible one is a compile error naming what is wrong rather
    /// than a false return at run time. The house pattern (avrdx's
    /// `Adc::init<cfg>()`): the runtime overload stays for a
    /// configuration built from runtime values.
    template <XoscConfig cfg>
    static bool init(uint32_t spins = 40'000'000UL) {
        static_assert(cfg.hz >= min_hz && cfg.hz <= max_hz,
                      "brio Xosc: XOSCConfig::hz must name the crystal or "
                      "external clock actually attached, and 20.2 bounds it at "
                      "0.4 to 32 MHz");
        static_assert(cfg.startup <= startup_max,
                      "brio Xosc: STARTUP is four bits (table 20-5)");
        static_assert(cfg.cfd_prescaler <= cfd_prescaler_max,
                      "brio Xosc: CFDPRESC is three bits (20.8.6)");
        return init(cfg, spins);
    }

    /**
     * Stop the oscillator and give XIN/XOUT back to PORT.
     *
     * CALLER OBLIGATION, and the one that bites: no generic clock
     * generator may still be sourced from the XOSC when this returns.
     * 16.6.2.6 releases a generator's old source only once the NEW one
     * is ready, and a stopped oscillator never becomes ready, so such a
     * generator is unroutable until the next reset.
     */
    static void stop() {
        OSCCTRL_REGS->OSCCTRL_XOSCCTRL =
            static_cast<uint16_t>(reg() & ~OSCCTRL_XOSCCTRL_ENABLE_Msk);
    }

    // ---- the clock failure detector ---------------------------------------

    /// CFDEN on a running oscillator, without disturbing anything else
    /// in XOSCCTRL. The detector does not judge until the start-up time
    /// has elapsed (20.6.3).
    static void failure_detector(bool on) {
        const uint16_t v = reg();
        OSCCTRL_REGS->OSCCTRL_XOSCCTRL = static_cast<uint16_t>(
            on ? (v | OSCCTRL_XOSCCTRL_CFDEN_Msk)
               : (v & static_cast<uint16_t>(~OSCCTRL_XOSCCTRL_CFDEN_Msk)));
    }
    static bool failure_detector() {
        return (reg() & OSCCTRL_XOSCCTRL_CFDEN_Msk) != 0u;
    }

    /// The safe clock's divider: OSC48M / 2^p (20.8.6).
    static void cfd_prescaler(CfdPrescaler p) {
        OSCCTRL_REGS->OSCCTRL_CFDPRESC = OSCCTRL_CFDPRESC_CFDPRESC(p);
    }
    static CfdPrescaler cfd_prescaler() {
        return static_cast<CfdPrescaler>(OSCCTRL_REGS->OSCCTRL_CFDPRESC &
                                         OSCCTRL_CFDPRESC_CFDPRESC_Msk);
    }

    /**
     * SWBEN: go back to the crystal once it recovers. The bit is
     * cleared by hardware when the switch happens (20.8.5), so it is
     * written and then observed through switched_to_safe_clock().
     *
     * ERRATUM 1.22.1 is about exactly this and does NOT apply to this
     * silicon: the failure-to-switch with the input stuck high is marked
     * for the N family only, at revisions E and F, and the E/G/J row is
     * empty. The trap the errata matrix sets is reading the COLUMN; the
     * row is what counts. A program that depends on the automatic
     * switch should still check switched_to_safe_clock() rather than
     * assume, which is what the workaround amounts to anyway.
     */
    static void switch_back() {
        OSCCTRL_REGS->OSCCTRL_XOSCCTRL =
            static_cast<uint16_t>(reg() | OSCCTRL_XOSCCTRL_SWBEN_Msk);
    }
    static bool switch_back_pending() {
        return (reg() & OSCCTRL_XOSCCTRL_SWBEN_Msk) != 0u;
    }
};


/// GENCTRL.SRC codes (16.8.3). The names are the device header's.
enum class GclkSource : uint8_t {
    xosc = GCLK_GENCTRL_SRC_XOSC_Val,
    gclk_in = GCLK_GENCTRL_SRC_GCLKIN_Val,
    gclk_gen1 = GCLK_GENCTRL_SRC_GCLKGEN1_Val,
    osculp32k = GCLK_GENCTRL_SRC_OSCULP32K_Val,
    osc32k = GCLK_GENCTRL_SRC_OSC32K_Val,
    xosc32k = GCLK_GENCTRL_SRC_XOSC32K_Val,
    osc48m = GCLK_GENCTRL_SRC_OSC48M_Val,
    dpll96m = GCLK_GENCTRL_SRC_DPLL96M_Val,
};

/// One generator's whole GENCTRL, written in a single store.
///
/// `div` is the raw DIVISION FACTOR field, read according to `div_pow2`:
/// with div_pow2 false the source is divided by div (0 and 1 both mean
/// "not divided", 16.6.2.7), with it true by 2^(div + 1). Two regimes,
/// one field - the chapter's own shape, kept rather than hidden, because
/// only the caller knows which one expresses its intent exactly.
///
/// The FIELD WIDTH is per generator (table 16-3): sixteen bits on
/// generator 1, eight on every other including 0. Bits written outside
/// the range are ignored by the hardware, so a linear divisor past 255
/// only means anything on generator 1.
struct GclkConfig {
    GclkSource source = GclkSource::osc48m;
    uint16_t div = 0;
    /// DIVSEL: the divisor becomes 2^(DIV + 1).
    ///
    /// 16.8.3's wording invites another reading - "divided by 2^(N+1),
    /// where N is the Division Factor Bits for the selected generator" -
    /// which would make the divisor a FIXED 512 on the eight-bit
    /// generators and 131072 on generator 1, with DIV ignored. An
    /// earlier note in this file said exactly that. IT IS WRONG, and
    /// the correction is measured, not argued: test_samc_clock letter f
    /// counts generator 5 against generator 0 with the frequency meter -
    /// both fed by OSC48M, so the count IS the divisor and no reference
    /// error can enter - and gets 2, 16 and 512 for DIV = 0, 3 and 8.
    /// Generator 1, the sixteen-bit one, gives 512 for the same DIV = 8,
    /// so the field's WIDTH plays no part at all.
    bool div_pow2 = false;
    bool improve_duty = false;   ///< IDC: 50 % duty for an odd divider (16.6.2.8)
    bool output_enable = false;  ///< OE: the generator on its GCLK_IO pad
    bool output_off_value = false;  ///< OOV: the pad's level while disabled
    bool run_standby = false;
};

constexpr uint32_t gclk_genctrl_word(const GclkConfig& c) {
    return GCLK_GENCTRL_SRC(static_cast<uint32_t>(c.source)) |
           GCLK_GENCTRL_DIV(c.div) |
           (c.div_pow2 ? GCLK_GENCTRL_DIVSEL_Msk : 0u) |
           (c.improve_duty ? GCLK_GENCTRL_IDC_Msk : 0u) |
           (c.output_enable ? GCLK_GENCTRL_OE_Msk : 0u) |
           (c.output_off_value ? GCLK_GENCTRL_OOV_Msk : 0u) |
           (c.run_standby ? GCLK_GENCTRL_RUNSTDBY_Msk : 0u) |
           GCLK_GENCTRL_GENEN_Msk;
}

/// Generic clock generator n. GCLK_GEN_NUM comes from the device header
/// (nine on this family); generator 0 is CPU/bus clock, generator 1 is
/// the only one that can feed another generator.
template <uint8_t n>
struct Gclk {
    Gclk() = delete;
    static_assert(n < GCLK_GEN_NUM, "this generic clock generator does not exist");

    static constexpr uint8_t index = n;
    static constexpr uint32_t sync_mask = GCLK_SYNCBUSY_GENCTRL0_Msk << n;

    /// Configure AND enable in one store, then wait out the write
    /// synchronization. Safe on a running generator: a source change is
    /// taken on the fly and the old source is released only once the new
    /// one is ready (16.6.2.6) - which is what lets Clock::init() re-state
    /// generator 0 while executing from it. NOTE for the day this target
    /// has a second clock source: 16.6.2.6 asks for ONDEMAND on the
    /// OUTGOING source before generator 0 really CHANGES source.
    /// Re-stating the source it already has is not that case.
    static bool configure(const GclkConfig& cfg, uint32_t spins = 0xFFFFu) {
        GCLK_REGS->GCLK_GENCTRL[n] = gclk_genctrl_word(cfg);
        return wait_sync(spins);
    }

    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = GCLK_REGS->GCLK_GENCTRL[n];
        GCLK_REGS->GCLK_GENCTRL[n] =
            on ? (v | GCLK_GENCTRL_GENEN_Msk) : (v & ~GCLK_GENCTRL_GENEN_Msk);
        return wait_sync(spins);
    }
    static bool enabled() {
        return (GCLK_REGS->GCLK_GENCTRL[n] & GCLK_GENCTRL_GENEN_Msk) != 0u;
    }

    static GclkSource source() {
        return static_cast<GclkSource>(
            (GCLK_REGS->GCLK_GENCTRL[n] & GCLK_GENCTRL_SRC_Msk) >> GCLK_GENCTRL_SRC_Pos);
    }

    static bool sync_busy() { return (GCLK_REGS->GCLK_SYNCBUSY & sync_mask) != 0u; }
    static bool wait_sync(uint32_t spins = 0xFFFFu) {
        return clock_wait(GCLK_REGS->GCLK_SYNCBUSY, sync_mask, false, spins);
    }
};

/// The peripheral clock channels (PCHCTRL, 16.8.4): each peripheral that
/// needs a clock of its own names ONE generator here. The channel index
/// is the device header's <PERIPHERAL>_GCLK_ID_* constant - a fact of the
/// peripheral, so its driver passes it; this resource only wires it.
///
/// PCHCTRL has no synchronization: 16.6.3.3 requires the channel to be
/// DISABLED before its generator is changed, which is what connect()
/// does rather than trusting a hot swap.
struct GclkChannel {
    GclkChannel() = delete;

    /// Returns false when the channel refused to go quiet - which on
    /// this register means WRTLOCK, since PCHCTRL has no synchronization
    /// to wait out. Bounded like every other wait in this file.
    static bool connect(uint8_t channel, uint8_t generator, uint32_t spins = 0xFFFFu) {
        GCLK_REGS->GCLK_PCHCTRL[channel] &= ~GCLK_PCHCTRL_CHEN_Msk;
        if (!clock_wait(GCLK_REGS->GCLK_PCHCTRL[channel], GCLK_PCHCTRL_CHEN_Msk,
                        false, spins)) {
            return false;
        }
        GCLK_REGS->GCLK_PCHCTRL[channel] =
            GCLK_PCHCTRL_GEN(generator) | GCLK_PCHCTRL_CHEN_Msk;
        return true;
    }

    static void disconnect(uint8_t channel) {
        GCLK_REGS->GCLK_PCHCTRL[channel] &= ~GCLK_PCHCTRL_CHEN_Msk;
    }

    static bool connected(uint8_t channel) {
        return (GCLK_REGS->GCLK_PCHCTRL[channel] & GCLK_PCHCTRL_CHEN_Msk) != 0u;
    }
    static uint8_t generator(uint8_t channel) {
        return static_cast<uint8_t>(GCLK_REGS->GCLK_PCHCTRL[channel] & GCLK_PCHCTRL_GEN_Msk);
    }

    /// WRTLOCK is ONE-WAY (16.6.3.4): the channel's configuration is
    /// frozen until the next reset.
    static void lock(uint8_t channel) {
        GCLK_REGS->GCLK_PCHCTRL[channel] |= GCLK_PCHCTRL_WRTLOCK_Msk;
    }
    static bool locked(uint8_t channel) {
        return (GCLK_REGS->GCLK_PCHCTRL[channel] & GCLK_PCHCTRL_WRTLOCK_Msk) != 0u;
    }
};

// =============================================================================
// FDPLL96M - the fractional digital phase-locked loop
// =============================================================================

/// DPLLCTRLB.REFCLK (20.8.14). GCLK means the peripheral channel
/// OSCCTRL_GCLK_ID_FDPLL, i.e. whatever generator that channel names.
enum class DpllReference : uint8_t {
    xosc32k = OSCCTRL_DPLLCTRLB_REFCLK_XOSC32K_Val,
    xosc = OSCCTRL_DPLLCTRLB_REFCLK_XOSC_Val,
    gclk = OSCCTRL_DPLLCTRLB_REFCLK_GCLK_Val,
};

/// DPLLCTRLB.FILTER: the PI controller. DEFAULT is adjusted by the
/// hardware and is what a caller should leave alone (20.6.5.1.5).
enum class DpllFilter : uint8_t {
    default_filter = 0,
    low_bandwidth = 1,
    high_bandwidth = 2,
    high_damping = 3,
};

/// DPLLCTRLB.LTIME: how long the lock is allowed to take before
/// INTFLAG.DPLLLTO is raised. `automatic` is the chapter's DEFAULT -
/// no time-out, the lock signal comes from the block's own status.
enum class DpllLockTime : uint8_t {
    automatic = 0,
    ms8 = 4,
    ms9 = 5,
    ms10 = 6,
    ms11 = 7,
};

/// DPLLPRESC.PRESC: the output prescaler, applied AFTER the DCO. The
/// 48..96 MHz range is the DCO's, not this output's.
enum class DpllPrescaler : uint8_t {
    div1 = OSCCTRL_DPLLPRESC_PRESC_DIV1_Val,
    div2 = OSCCTRL_DPLLPRESC_PRESC_DIV2_Val,
    div4 = OSCCTRL_DPLLPRESC_PRESC_DIV4_Val,
};

struct FdpllConfig {
    DpllReference reference = DpllReference::gclk;

    /// The reference's frequency in hertz BEFORE the XOSC divider - the
    /// crystal's own rate for DpllReference::xosc, the generator's rate
    /// for gclk, 32768 for xosc32k. Nothing is written from it; it is
    /// what the arithmetic and config_valid() reason about.
    uint32_t reference_hz = 0;

    /// DPLLCTRLB.DIV, the divider on the XOSC reference ONLY: the
    /// reference becomes f_xosc / (2 x (DIV + 1)). It is ignored by the
    /// other two references, and config_valid() refuses a non-zero value
    /// there rather than letting it look effective.
    uint16_t xosc_div = 0;

    /// DPLLRATIO. The DCO runs at f_ref x (LDR + 1 + LDRFRAC/16), so
    /// `ldr` is the multiplier MINUS ONE - the register's own shape,
    /// kept rather than hidden, because `ratio_for()` below is where a
    /// caller should be spelling a frequency anyway.
    uint16_t ldr = 0;
    uint8_t ldr_frac = 0;

    DpllPrescaler prescaler = DpllPrescaler::div1;
    DpllFilter filter = DpllFilter::default_filter;
    DpllLockTime lock_time = DpllLockTime::automatic;

    /// WUF: release the output at the end of the start-up time rather
    /// than at lock. Saves milliseconds and hands out an unstable clock
    /// while the loop acquires (table 20-3).
    bool wake_up_fast = false;

    /// LPEN: bypass the time-to-digital converter. Less power, more
    /// jitter (20.6.5.1.5).
    bool low_power = false;

    /// LBYPASS: the lock signal stops gating the output clock.
    ///
    /// DEFAULT TRUE, AND THAT IS ERRATUM 1.25.1, live on every silicon
    /// revision: below 25 C the DPLL reports spurious unlocks while
    /// still meeting its specification, and each one HALTS the output
    /// clock. Anything clocked from the DPLL then stops for the
    /// duration. The erratum's own workaround is this bit, so it is
    /// what this driver does unless told otherwise - and a caller that
    /// clears it is choosing the gate back, not getting it by accident.
    bool lock_bypass = true;
};

/// The (ldr, ldr_frac) pair whose DCO frequency is nearest `target_hz`
/// from a reference of `reference_hz`, and the frequency it really
/// produces. The DCO steps in sixteenths of the reference, so the
/// arithmetic is exact in units of reference_hz/16 and the rounding is
/// to the nearest step. Compile-time, like every other ratio in this
/// stratum.
struct DpllRatio {
    uint16_t ldr = 0;
    uint8_t frac = 0;
    uint32_t actual_hz = 0;
    bool exact = false;   ///< the requested rate is hit to the hertz
};

constexpr DpllRatio dpll_ratio_for(uint32_t reference_hz, uint32_t target_hz) {
    DpllRatio r{};
    if (reference_hz == 0u) {
        return r;
    }
    // steps = round(16 x target / reference), the multiplier in
    // sixteenths. 64 bits because 16 x 96 MHz overflows nothing but the
    // intermediate product with a slow reference does not, and the width
    // is named rather than hoped for.
    const uint64_t num = 16ULL * static_cast<uint64_t>(target_hz) +
                         (static_cast<uint64_t>(reference_hz) / 2u);
    uint64_t steps = num / reference_hz;
    if (steps < 16u) {
        steps = 16u;   // the multiplier cannot go below 1
    }
    const uint64_t mult = steps / 16u;
    if (mult > 4096u) {
        return r;   // LDR is twelve bits; LDR + 1 tops out at 4096
    }
    r.ldr = static_cast<uint16_t>(mult - 1u);
    r.frac = static_cast<uint8_t>(steps % 16u);
    r.actual_hz = static_cast<uint32_t>(
        (steps * static_cast<uint64_t>(reference_hz)) / 16u);
    r.exact = r.actual_hz == target_hz;
    return r;
}

/**
 * The fractional 96 MHz DPLL (20.6.5).
 *
 * WHAT THE ARITHMETIC IS. The DCO runs at
 *
 *     f_dco = f_ref x (LDR + 1 + LDRFRAC / 16)
 *
 * and CLK_DPLL is f_dco divided by the output prescaler. The 48..96 MHz
 * limit of table 45-52 is on f_dco, so the prescaler is how a slower
 * output is reached - and `config_valid()` judges the DCO, not the
 * output, which is the distinction the chapter's block diagram makes
 * and its text does not repeat.
 *
 * TWO CLOCKS, NOT ONE. Besides the reference the block has an internal
 * lock timer on its own peripheral channel, GCLK_DPLL_32K. It is not
 * needed to LOCK, but erratum 1.3.4 says an on-the-fly ratio change
 * does not work without it - so `lock_timer_clock()` exists and
 * `set_ratio()` names the obligation it cannot enforce.
 */
struct Fdpll {
    Fdpll() = delete;

    /// The two peripheral clock channels, from the device header.
    static constexpr uint8_t gclk_reference = OSCCTRL_GCLK_ID_FDPLL;      // 0
    static constexpr uint8_t gclk_lock_timer = OSCCTRL_GCLK_ID_FDPLL32K;  // 1

    /// Table 45-52: the reference and the DCO output.
    static constexpr uint32_t min_reference_hz = 32'000UL;
    static constexpr uint32_t max_reference_hz = 2'000'000UL;
    static constexpr uint32_t min_output_hz = 48'000'000UL;
    static constexpr uint32_t max_output_hz = 96'000'000UL;
    static constexpr uint16_t ldr_max = 0xFFF;
    static constexpr uint8_t frac_max = 0xF;
    static constexpr uint16_t xosc_div_max = 0x7FF;

    /// The reference frequency the loop actually sees: the XOSC divider
    /// applies to DpllReference::xosc alone (20.8.14).
    static constexpr uint32_t divided_reference_hz(const FdpllConfig& c) {
        if (c.reference != DpllReference::xosc) {
            return c.reference_hz;
        }
        return c.reference_hz / (2u * (static_cast<uint32_t>(c.xosc_div) + 1u));
    }

    /// The DCO frequency a configuration produces.
    static constexpr uint32_t dco_hz(const FdpllConfig& c) {
        const uint64_t ref = divided_reference_hz(c);
        const uint64_t steps = 16ULL * (static_cast<uint64_t>(c.ldr) + 1u) + c.ldr_frac;
        return static_cast<uint32_t>((ref * steps) / 16u);
    }

    /// CLK_DPLL: the DCO after the output prescaler. This is what a GCLK
    /// generator sourced from DPLL96M receives.
    static constexpr uint32_t output_hz(const FdpllConfig& c) {
        return dco_hz(c) >> static_cast<uint32_t>(c.prescaler);
    }

    /**
     * Every constraint the chapter and table 45-52 put on a
     * configuration, in one place:
     *  - the DIVIDED reference inside 32 kHz .. 2 MHz;
     *  - the DCO inside 48 .. 96 MHz (the prescaler is not a way out of
     *    the bottom of that range - it divides what the DCO made);
     *  - the fields inside their widths;
     *  - and DPLLCTRLB.DIV asked for only where it exists, which is the
     *    XOSC reference alone.
     */
    static constexpr bool config_valid(const FdpllConfig& c) {
        if (c.ldr > ldr_max || c.ldr_frac > frac_max || c.xosc_div > xosc_div_max) {
            return false;
        }
        if (c.reference != DpllReference::xosc && c.xosc_div != 0u) {
            return false;
        }
        const uint32_t ref = divided_reference_hz(c);
        if (ref < min_reference_hz || ref > max_reference_hz) {
            return false;
        }
        const uint32_t dco = dco_hz(c);
        return dco >= min_output_hz && dco <= max_output_hz;
    }

    // ---- clocks ------------------------------------------------------------

    /// Point GCLK_DPLL (the reference channel) at a generator. Only
    /// meaningful for DpllReference::gclk.
    static bool reference_clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_reference, generator, spins);
    }

    /// Point GCLK_DPLL_32K (the internal lock timer) at a generator.
    /// Erratum 1.3.4: without it an on-the-fly ratio change does not
    /// work. A generator on any 32 kHz root is what it wants.
    static bool lock_timer_clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_lock_timer, generator, spins);
    }

    // ---- state -------------------------------------------------------------

    static bool enabled() {
        return (OSCCTRL_REGS->OSCCTRL_DPLLCTRLA & OSCCTRL_DPLLCTRLA_ENABLE_Msk) != 0u;
    }

    /// DPLLSTATUS.LOCK: the loop is at its target frequency.
    static bool locked() {
        return (OSCCTRL_REGS->OSCCTRL_DPLLSTATUS & OSCCTRL_DPLLSTATUS_LOCK_Msk) != 0u;
    }
    /// DPLLSTATUS.CLKRDY: the output clock is actually being issued.
    /// With LBYPASS set this is the bit that matters and LOCK is only a
    /// report - see FdpllConfig::lock_bypass and erratum 1.25.1.
    static bool clock_ready() {
        return (OSCCTRL_REGS->OSCCTRL_DPLLSTATUS & OSCCTRL_DPLLSTATUS_CLKRDY_Msk) != 0u;
    }

    static bool sync_busy(uint8_t mask = OSCCTRL_DPLLSYNCBUSY_Msk) {
        return (OSCCTRL_REGS->OSCCTRL_DPLLSYNCBUSY & mask) != 0u;
    }
    static bool wait_sync(uint8_t mask = OSCCTRL_DPLLSYNCBUSY_Msk,
                          uint32_t spins = 0xFFFFu) {
        while (sync_busy(mask) && spins-- != 0u) {
        }
        return !sync_busy(mask);
    }

    /**
     * INTFLAG.DPLLLTO - and it does NOT mean what its name says.
     *
     * The chapter calls it "DPLL Lock Timer Time-out", which reads as a
     * failure to lock. Measured (test_samc_clock letter d), a loop
     * configured with LTIME = 8 ms comes up with CLKRDY set, LOCK set
     * and THIS FLAG SET as well: the flag marks the lock TIMER reaching
     * zero, which in that mode is simply how the output is released
     * (table 20-3), not a failure. It is only ever raised when
     * DpllLockTime is not `automatic`, and it says nothing about the
     * loop - `locked()` is the bit that does.
     */
    static bool timed_out() {
        return (Oscctrl::flags() & OscctrlFlag::dpll_timeout) != 0u;
    }

    /**
     * INTFLAG.DPLLLDRTO: an on-the-fly ratio change has been taken by
     * the analog cell.
     *
     * ERRATUM 1.3.3, live on this silicon: STATUS.DPLLLDRTO is NOT set
     * when the ratio changes although the change happens; the INTFLAG
     * copy is. So this reads the flag, and nothing in this file reads
     * that status bit.
     */
    static bool ratio_updated() {
        return (Oscctrl::flags() & OscctrlFlag::dpll_ratio_done) != 0u;
    }

    // ---- configuration -----------------------------------------------------

    static constexpr uint32_t ctrlb_word(const FdpllConfig& c) {
        return OSCCTRL_DPLLCTRLB_REFCLK(static_cast<uint32_t>(c.reference)) |
               OSCCTRL_DPLLCTRLB_FILTER(static_cast<uint32_t>(c.filter)) |
               OSCCTRL_DPLLCTRLB_LTIME(static_cast<uint32_t>(c.lock_time)) |
               OSCCTRL_DPLLCTRLB_DIV(c.xosc_div) |
               (c.wake_up_fast ? OSCCTRL_DPLLCTRLB_WUF_Msk : 0u) |
               (c.low_power ? OSCCTRL_DPLLCTRLB_LPEN_Msk : 0u) |
               (c.lock_bypass ? OSCCTRL_DPLLCTRLB_LBYPASS_Msk : 0u);
    }

    static constexpr uint32_t ratio_word(uint16_t ldr, uint8_t frac) {
        return OSCCTRL_DPLLRATIO_LDR(ldr) | OSCCTRL_DPLLRATIO_LDRFRAC(frac);
    }

    /**
     * Configure the loop and start it, waiting until the output clock is
     * being issued.
     *
     * ORDER, and it is the chapter's: DPLLCTRLB is ENABLE-PROTECTED
     * (20.8.14), so everything static is written with the block disabled
     * and ENABLE comes last. The wait is on DPLLSTATUS.CLKRDY rather
     * than on LOCK, because CLKRDY is what says a consumer will receive
     * a clock - under LBYPASS (the default, erratum 1.25.1) LOCK may
     * flicker without the output ever stopping.
     *
     * False - and the loop left disabled - for a configuration outside
     * the chapter or an output that never appeared.
     */
    static bool init(const FdpllConfig& cfg, uint32_t spins = 4'000'000UL) {
        if (!config_valid(cfg)) {
            return false;
        }
        // Disabled first: CTRLB is enable-protected, and a running loop
        // would ignore the writes and raise a PAC error.
        OSCCTRL_REGS->OSCCTRL_DPLLCTRLA = 0;
        if (!wait_sync(OSCCTRL_DPLLSYNCBUSY_ENABLE_Msk, spins)) {
            return false;
        }
        OSCCTRL_REGS->OSCCTRL_DPLLCTRLB = ctrlb_word(cfg);
        OSCCTRL_REGS->OSCCTRL_DPLLRATIO = ratio_word(cfg.ldr, cfg.ldr_frac);
        if (!wait_sync(OSCCTRL_DPLLSYNCBUSY_DPLLRATIO_Msk, spins)) {
            return false;
        }
        OSCCTRL_REGS->OSCCTRL_DPLLPRESC =
            OSCCTRL_DPLLPRESC_PRESC(static_cast<uint32_t>(cfg.prescaler));
        if (!wait_sync(OSCCTRL_DPLLSYNCBUSY_DPLLPRESC_Msk, spins)) {
            return false;
        }

        Oscctrl::clear_flags(OscctrlFlag::dpll_lock_rise | OscctrlFlag::dpll_lock_fall |
                             OscctrlFlag::dpll_timeout | OscctrlFlag::dpll_ratio_done);

        // ONDEMAND is cleared: a loop that stops whenever nothing asks
        // for it cannot be measured, and erratum 1.25.2 makes ONDEMAND
        // non-functional in standby anyway. RUNSTDBY is left clear -
        // this file takes no position on sleep.
        OSCCTRL_REGS->OSCCTRL_DPLLCTRLA = OSCCTRL_DPLLCTRLA_ENABLE_Msk;
        if (!wait_sync(OSCCTRL_DPLLSYNCBUSY_ENABLE_Msk, spins)) {
            return false;
        }
        uint32_t left = spins;
        while (!clock_ready() && left-- != 0u) {
        }
        return clock_ready();
    }

    /// The compile-time twin, one static_assert per constraint so a
    /// configuration that cannot work says WHICH rule it broke.
    template <FdpllConfig cfg>
    static bool init(uint32_t spins = 4'000'000UL) {
        static_assert(cfg.ldr <= ldr_max && cfg.ldr_frac <= frac_max &&
                          cfg.xosc_div <= xosc_div_max,
                      "brio Fdpll: LDR is 12 bits, LDRFRAC 4, DPLLCTRLB.DIV 11");
        static_assert(cfg.reference == DpllReference::xosc || cfg.xosc_div == 0u,
                      "brio Fdpll: DPLLCTRLB.DIV divides the XOSC reference and "
                      "nothing else (20.8.14) - asking for it on the GCLK or "
                      "XOSC32K reference would silently do nothing");
        static_assert(divided_reference_hz(cfg) >= min_reference_hz &&
                          divided_reference_hz(cfg) <= max_reference_hz,
                      "brio Fdpll: the reference the loop sees must be inside "
                      "32 kHz .. 2 MHz (table 45-52) - divide the XOSC down "
                      "with DPLLCTRLB.DIV, or a generator down with its own");
        static_assert(dco_hz(cfg) >= min_output_hz && dco_hz(cfg) <= max_output_hz,
                      "brio Fdpll: the DCO must run at 48 .. 96 MHz (table "
                      "45-52). The output prescaler divides what the DCO made "
                      "and is not a way below that floor");
        return init(cfg, spins);
    }

    /// Wait for DPLLSTATUS.LOCK, separately from the output being
    /// issued. Worth asking when the jitter matters; not worth hanging
    /// on, hence the bound and the bool.
    static bool wait_locked(uint32_t spins = 4'000'000UL) {
        while (!locked() && spins-- != 0u) {
        }
        return locked();
    }

    /**
     * Change the loop divider ratio ON THE FLY (20.6.5.1.4) and wait
     * for the analog cell to take it.
     *
     * TWO ERRATA MEET HERE, both live on this silicon. 1.3.4: this does
     * not work at all unless GCLK_DPLL_32K is running, which is
     * `lock_timer_clock()` and is the CALLER's to have done - the block
     * offers no way to ask. 1.3.3: the completion shows up in
     * INTFLAG.DPLLLDRTO and NOT in the status bit of the same name, so
     * that is what is waited on.
     */
    static bool set_ratio(uint16_t ldr, uint8_t frac, uint32_t spins = 4'000'000UL) {
        if (ldr > ldr_max || frac > frac_max) {
            return false;
        }
        Oscctrl::clear_flags(OscctrlFlag::dpll_ratio_done);
        OSCCTRL_REGS->OSCCTRL_DPLLRATIO = ratio_word(ldr, frac);
        if (!wait_sync(OSCCTRL_DPLLSYNCBUSY_DPLLRATIO_Msk, spins)) {
            return false;
        }
        uint32_t left = spins;
        while (!ratio_updated() && left-- != 0u) {
        }
        return ratio_updated();
    }

    /// The output prescaler on a running loop (20.6.5.1.3).
    static bool prescaler(DpllPrescaler p, uint32_t spins = 0xFFFFu) {
        OSCCTRL_REGS->OSCCTRL_DPLLPRESC =
            OSCCTRL_DPLLPRESC_PRESC(static_cast<uint32_t>(p));
        return wait_sync(OSCCTRL_DPLLSYNCBUSY_DPLLPRESC_Msk, spins);
    }
    static DpllPrescaler prescaler() {
        return static_cast<DpllPrescaler>(OSCCTRL_REGS->OSCCTRL_DPLLPRESC &
                                          OSCCTRL_DPLLPRESC_PRESC_Msk);
    }

    /**
     * Stop the loop.
     *
     * The same caller obligation as Xosc::stop(): no generator may still
     * be sourced from DPLL96M, or 16.6.2.6 leaves it unroutable.
     */
    static bool stop(uint32_t spins = 0xFFFFu) {
        OSCCTRL_REGS->OSCCTRL_DPLLCTRLA = 0;
        return wait_sync(OSCCTRL_DPLLSYNCBUSY_ENABLE_Msk, spins);
    }
};
// NOTE ON PLACEMENT: the DPLL is an OSCCTRL block like the two above,
// but it sits here, after GclkChannel, because BOTH its clocks - the
// reference and the internal lock timer - are peripheral clock
// channels, and a resource may only use what is already declared.

/// The main clock controller (ch. 17): generator 0 arrives here, CPUDIV
/// divides it into CLK_CPU/CLK_AHB/CLK_APBx, and the four masks gate the
/// bus clock of each peripheral.
struct Mclk {
    Mclk() = delete;

    /// CPUDIV is a ONE-HOT power of two (1, 2, 4, ... 128), not a count
    /// (17.8.5).
    static bool cpu_div(uint8_t ratio) {
        switch (ratio) {
            case 1: case 2: case 4: case 8:
            case 16: case 32: case 64: case 128:
                MCLK_REGS->MCLK_CPUDIV = MCLK_CPUDIV_CPUDIV(ratio);
                return true;
            default:
                return false;
        }
    }
    static uint8_t cpu_div() { return MCLK_REGS->MCLK_CPUDIV; }

    // Bus-clock masks (17.6.2.6). The bit of a peripheral is the device
    // header's MCLK_APBxMASK_<PERIPHERAL>_Msk - its driver's business to
    // know, never this resource's.
    static void ahb(uint32_t mask, bool on) { apply(MCLK_REGS->MCLK_AHBMASK, mask, on); }
    static void apb_a(uint32_t mask, bool on) { apply(MCLK_REGS->MCLK_APBAMASK, mask, on); }
    static void apb_b(uint32_t mask, bool on) { apply(MCLK_REGS->MCLK_APBBMASK, mask, on); }
    static void apb_c(uint32_t mask, bool on) { apply(MCLK_REGS->MCLK_APBCMASK, mask, on); }

private:
    static void apply(volatile uint32_t& reg, uint32_t mask, bool on) {
        reg = on ? (reg | mask) : (reg & ~mask);
    }
};

// =============================================================================
// Task: the main clock
// =============================================================================

/// Where CLK_MAIN comes from. Only `internal` is implemented; the rest
/// name the tree's other roots so the vocabulary does not change under
/// an application when they are built, and so asking for one today is a
/// compile error with an explanation instead of a wrong clock.
enum class ClockSource : uint8_t {
    internal,   ///< OSC48M, the internal 48 MHz oscillator
    crystal,    ///< XOSC crystal (this board: 24 MHz on PA14/PA15)
    external,   ///< XOSC in external-clock mode
    osc32k,     ///< the internal 32.768 kHz oscillator
    xosc32k,    ///< the 32.768 kHz crystal
    dpll,       ///< FDPLL96M, from any of the above
};

/**
 * The static main clock: `hz` is the ONE compile-time truth about
 * CLK_CPU that every driver of this target derives from - there is no
 * F_CPU in this build, and no second place a rate can be stated.
 *
 * The rate is produced by the OSC48M output divider alone: GCLK
 * generator 0 and MCLK CPUDIV are written to "no division" and stay
 * there. One rate, one knob - a second knob would let two settings spell
 * the same `hz` and immediately raise the question which of them a
 * driver's own divider should assume. Generator 0 gains a divider on the
 * day a peripheral needs the bus faster or slower than the CPU.
 */
template <ClockSource src, uint32_t src_hz>
struct Clock {
    static constexpr ClockSource source = src;
    static constexpr uint32_t hz = src_hz;   ///< CLK_CPU = CLK_AHB = CLK_APBx
    static constexpr bool is_static = true;

    static_assert(src == ClockSource::internal,
                  "brio Clock: only ClockSource::internal (OSC48M) is implemented on "
                  "the SAM C21 today - the crystal, the 32 kHz oscillators and the "
                  "FDPLL96M are declared and arrive with their first consumer");
    static_assert(osc48m_div_for(src_hz) != 0xFF,
                  "brio Clock: an OSC48M rate must be 48 MHz divided by 1..16 with no "
                  "remainder - 48, 24, 16, 12, 9.6, 8, 6, 4.8, 4, 3.2 and 3 MHz. The "
                  "ratios 7, 9, 11, 13 and 14 leave one, and Clock::hz has to be exact");
    static_assert(src_hz <= 48'000'000UL, "CLK_CPU must not exceed 48 MHz");

    /// The OSC48M divider ratio this rate needs (1..16).
    static constexpr uint32_t divider = 48'000'000UL / src_hz;

    /// Bring CLK_CPU to `hz`. Returns false when the oscillator did not
    /// report ready or a synchronization did not complete - the caller
    /// then knows the rate is NOT the one `hz` claims. Call first in
    /// main(), before any driver init.
    static bool init() {
        // Wait states BEFORE a frequency rise, AFTER a fall (27.5.2):
        // the flash must never be read faster than its current setting
        // allows, in either direction.
        constexpr uint8_t rws = FlashWaitStates::for_hz(hz);
        const bool raising = rws > FlashWaitStates::get();
        if (raising) {
            FlashWaitStates::set(rws);
        }

        // ENABLE = 1 with ONDEMAND = 0 (errata 1.2.3 and 1.2.2): the
        // oscillator free-runs, which is both the start-up mitigation
        // and the precondition of the divider write below.
        Osc48m::enable(true);
        Osc48m::on_demand(false);

        bool ok = Osc48m::divider(divider);
        ok = Osc48m::wait_ready() && ok;

        // Generator 0 and CPUDIV are RE-STATED, not assumed: both are
        // already what we want out of reset, but `hz` is a promise, and
        // a promise that rests on whatever a debugger or a bootloader
        // left behind is not one. Re-stating generator 0 while running
        // from it is safe - see Gclk::configure().
        ok = Gclk<0>::configure({.source = GclkSource::osc48m}) && ok;
        ok = Mclk::cpu_div(1) && ok;

        if (!raising) {
            FlashWaitStates::set(rws);
        }
        return ok;
    }
};

} // namespace brio
