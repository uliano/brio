/*
 * clock.hpp
 *
 * The SAM C21 clock tree in two strata, the same split avrdx/clock.hpp
 * uses (docs/design/clock.md):
 *
 *  RESOURCES - one monostate per block, a thin typed view of its
 *  registers with the synchronization discipline built in:
 *    Osc48m       the internal 48 MHz oscillator (OSCCTRL, ch. 20):
 *                 ENABLE/ONDEMAND/RUNSTDBY, the 1..16 output divider
 *                 with its OSC48MSYNCBUSY wait, start-up time, ready
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
 * SCOPE, honestly. Only ClockSource::internal (OSC48M) is IMPLEMENTED.
 * The other enumerators are declared so the vocabulary is stable and an
 * unimplemented choice is a compile error with a message, not a silent
 * wrong clock; the XOSC crystal, the 32 kHz oscillators and the FDPLL96M
 * arrive with their first consumer (the board's 24 MHz crystal on
 * PA14/PA15 is deliberately unused today). Neither is there a
 * DynamicClock for this target yet - see ticker.hpp for the one
 * consequence that has to be remembered when it arrives.
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
 *    what init() writes, which is also the erratum 1.2.2 mitigation.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

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

/// The flash read wait states. They belong to NVMCTRL (ch. 27), not to
/// the clock controller - but they are a FUNCTION of the CPU frequency
/// and nothing may raise that frequency without setting them first, so
/// they live here until samc/nvm.hpp is born and takes them over.
struct FlashWaitStates {
    FlashWaitStates() = delete;

    static uint8_t get() {
        return static_cast<uint8_t>(
            (NVMCTRL_REGS->NVMCTRL_CTRLB & NVMCTRL_CTRLB_RWS_Msk) >>
            NVMCTRL_CTRLB_RWS_Pos);
    }

    static void set(uint8_t rws) {
        NVMCTRL_REGS->NVMCTRL_CTRLB =
            (NVMCTRL_REGS->NVMCTRL_CTRLB & ~NVMCTRL_CTRLB_RWS_Msk) |
            NVMCTRL_CTRLB_RWS(rws);
    }

    /// Wait states the flash needs at `hz` (table 45-41, VDD > 2.7 V:
    /// the conservative column - the 5 V one only buys 1 MHz at 0 WS).
    static constexpr uint8_t for_hz(uint32_t hz) {
        if (hz <= 19'000'000UL) return 0;
        if (hz <= 38'000'000UL) return 1;
        return 2;
    }
};

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
/// one field -
/// the chapter's own shape, kept rather than hidden, because only the
/// caller knows which one expresses its intent exactly.
struct GclkConfig {
    GclkSource source = GclkSource::osc48m;
    uint16_t div = 0;
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
