/*
 * ac.hpp
 *
 * The SAM C21 Analog Comparators (DS60001479M ch. 40) - the MINIMAL
 * resource surface, built to answer one bench question first (the
 * latency of the GCLK_AC-synchronized output path, which the chapter
 * never quantifies) and shaped so the full AC campaign can grow on it
 * rather than replace it:
 *
 *  Ac              the BLOCK: APB clock, the one GCLK channel
 *                  (GCLK_AC clocks ALL the digital - sampling, filter,
 *                  edge detection, the SYNC output; the comparators
 *                  themselves are analog and free-running), reset and
 *                  enable with their SYNCBUSY waits, INTFLAG.
 *
 *  AcComparator<n> one of the four comparators: configuration written
 *                  only while ITS enable is low (every COMPCTRL field
 *                  is enable-protected, 40.8.13), the per-comparator
 *                  VDD scaler, single-shot start, READY/STATE, flag
 *                  and arming verbs.
 *
 * WHAT THE CHAPTER SAYS ABOUT TIME, collected here because the bench
 * question lives exactly in the gaps between these clauses:
 *  - every figure of 40.6.2.4 draws a "Sampled Comparator Output" and
 *    the filter section states the sampling rate IS GCLK_AC;
 *  - COMPCTRLn.OUT routes either "the asynchronous output" or "the
 *    synchronous output (including filtering)" to the CMPn pad - with
 *    NO cycle count given for the synchronous path anywhere;
 *  - the only counted delay is the ENABLE/START command propagation
 *    ("2-3 cycles" in figures 40-3/40-4, before t_STARTUP) and the
 *    filter's own "N-1 sampling cycles";
 *  - edge detection for interrupts compares "the current and previous
 *    sample" (40.6.2.4.1), so a flag may trail the output by a sample;
 *  - the electrical tables measure ONLY the analog propagation delay
 *    (45-34 note 4: "ACOUT (AC direct output)... only analog
 *    propagation delay"; typ 38 ns high speed) and the startup
 *    (2-3 us high speed, 6.5-8.5 us low power).
 *  What a sampled edge costs in whole GCLK_AC cycles is therefore a
 *  silicon question; docs/samc/ac.md carries the measured answer.
 *
 * Pads, from the I/O multiplexing table (function B for the analog
 * inputs, function H for the digital outputs; the device header's
 * MUX_* constants are the authority): COMP0/1 inputs AIN[0..3] =
 * PA04..PA07; COMP2/3 inputs AIN[4..7] = PA02, PA03, PB05*, PB06*
 * (*J package only - and on the E and G packages COMP2/3 bond only
 * AIN[5:4], 40.1). Outputs CMP0 = PA12 or PA18, CMP1 = PA13 or PA19,
 * CMP2 = PA24 or PB30, CMP3 = PA25 or PB31 - on the bench board
 * PB30/PB31 are the console, one more reason this driver's first
 * user lives on COMP0.
 *
 * NOT BUILT YET (the full campaign's list, docs/samc/ac.md): window
 * mode (WINCTRL/WSTATE, the WIN flags), the event surface (EVCTRL is
 * not written - no EVSYS driver exists on this target), sleep
 * (RUNSTDBY is a config field, the 40.6.14 sequences have no owner),
 * per-package input legality as compile-time refusals, DAC as the
 * negative input (no DAC driver yet).
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvic.hpp"

namespace brio {

// =============================================================================
// Vocabulary (the register codes, named)
// =============================================================================

/// COMPCTRLn.MUXPOS. PIN0..3 are AIN[0..3] for COMP0/1 and AIN[4..7]
/// for COMP2/3 (the pair, not the comparator, owns the pads).
enum class AcPositive : uint8_t {
    pin0 = AC_COMPCTRL_MUXPOS_PIN0_Val,
    pin1 = AC_COMPCTRL_MUXPOS_PIN1_Val,
    pin2 = AC_COMPCTRL_MUXPOS_PIN2_Val,
    pin3 = AC_COMPCTRL_MUXPOS_PIN3_Val,
    vscale = AC_COMPCTRL_MUXPOS_VSCALE_Val,   ///< this comparator's own VDD scaler
};

/// COMPCTRLn.MUXNEG.
enum class AcNegative : uint8_t {
    pin0 = AC_COMPCTRL_MUXNEG_PIN0_Val,
    pin1 = AC_COMPCTRL_MUXNEG_PIN1_Val,
    pin2 = AC_COMPCTRL_MUXNEG_PIN2_Val,
    pin3 = AC_COMPCTRL_MUXNEG_PIN3_Val,
    ground = AC_COMPCTRL_MUXNEG_GND_Val,
    vscale = AC_COMPCTRL_MUXNEG_VSCALE_Val,
    bandgap = AC_COMPCTRL_MUXNEG_BANDGAP_Val,
    dac = AC_COMPCTRL_MUXNEG_DAC_Val,
};

/// COMPCTRLn.SPEED: bias current, so propagation delay AND startup time
/// (40.6.7). Only the two documented codes exist.
enum class AcSpeed : uint8_t {
    low_power = AC_COMPCTRL_SPEED_LOW_Val,
    high = AC_COMPCTRL_SPEED_HIGH_Val,
};

/// COMPCTRLn.FLEN: the majority filter, sampled at GCLK_AC. Costs N-1
/// sampling cycles of extra latency (40.6.8).
enum class AcFilter : uint8_t {
    off = AC_COMPCTRL_FLEN_OFF_Val,
    majority3 = AC_COMPCTRL_FLEN_MAJ3_Val,
    majority5 = AC_COMPCTRL_FLEN_MAJ5_Val,
};

/// COMPCTRLn.OUT: what reaches the CMPn pad. The pad also needs its
/// PMUX (function H on this family) - routing here does not claim it.
enum class AcOut : uint8_t {
    off = AC_COMPCTRL_OUT_OFF_Val,
    asynchronous = AC_COMPCTRL_OUT_ASYNC_Val,   ///< the raw comparator
    synchronous = AC_COMPCTRL_OUT_SYNC_Val,     ///< sampled, filter included
};

/// COMPCTRLn.INTSEL: which output change raises INTFLAG.COMPn. Edge
/// detection compares the current sample with the previous one.
enum class AcInterrupt : uint8_t {
    toggle = AC_COMPCTRL_INTSEL_TOGGLE_Val,
    rising = AC_COMPCTRL_INTSEL_RISING_Val,
    falling = AC_COMPCTRL_INTSEL_FALLING_Val,
    end_of_comparison = AC_COMPCTRL_INTSEL_EOC_Val,   ///< single-shot only
};

/// One comparator's whole configuration - everything COMPCTRLn holds
/// except ENABLE, which is a separate deliberate step (the fields are
/// writable only while that bit is low).
struct AcConfig {
    AcPositive positive = AcPositive::pin0;
    AcNegative negative = AcNegative::ground;
    bool single_shot = false;       ///< SINGLE: idle until start()
    AcInterrupt interrupt_on = AcInterrupt::toggle;
    AcSpeed speed = AcSpeed::high;
    bool hysteresis = false;        ///< HYSTEN: continuous mode only
    AcFilter filter = AcFilter::off;
    AcOut out = AcOut::off;
    bool swap = false;              ///< SWAP: swap inputs AND invert output
    bool run_standby = false;
};

constexpr uint32_t ac_compctrl(const AcConfig& c) {
    return AC_COMPCTRL_MUXPOS(static_cast<uint32_t>(c.positive)) |
           AC_COMPCTRL_MUXNEG(static_cast<uint32_t>(c.negative)) |
           (c.single_shot ? AC_COMPCTRL_SINGLE_Msk : 0u) |
           AC_COMPCTRL_INTSEL(static_cast<uint32_t>(c.interrupt_on)) |
           AC_COMPCTRL_SPEED(static_cast<uint32_t>(c.speed)) |
           (c.hysteresis ? AC_COMPCTRL_HYSTEN_Msk : 0u) |
           AC_COMPCTRL_FLEN(static_cast<uint32_t>(c.filter)) |
           AC_COMPCTRL_OUT(static_cast<uint32_t>(c.out)) |
           (c.swap ? AC_COMPCTRL_SWAP_Msk : 0u) |
           (c.run_standby ? AC_COMPCTRL_RUNSTDBY_Msk : 0u);
}

// =============================================================================
// The block
// =============================================================================

class Ac {
public:
    Ac() = delete;

    static constexpr uint8_t comparator_count = 4;

    static constexpr IRQn_Type irq() { return AC_IRQn; }

    static ac_registers_t& regs() { return *AC_REGS; }

    /// CLK_AC_APB (APBC bit) - registers only. The digital machinery
    /// runs on GCLK_AC, which clock() wires.
    static void bus_clock(bool on) { Mclk::apb_c(MCLK_APBCMASK_AC_Msk, on); }

    /// Point GCLK_AC (peripheral channel AC_GCLK_ID) at a generator.
    /// The generator's rate is the sampling rate of everything digital
    /// in this peripheral - which is exactly the knob the latency
    /// measurement turns (slow it down and the quantization becomes
    /// visible to a software stopwatch).
    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(AC_GCLK_ID, generator, spins);
    }

    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().AC_SYNCBUSY, mask, false, spins);
    }

    /// Everything back to reset (DBGCTRL excepted). Write-synchronized:
    /// the wait is bounded and reported like every wait in this stratum.
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().AC_CTRLA = AC_CTRLA_SWRST_Msk;
        return sync_wait(AC_SYNCBUSY_SWRST_Msk, spins);
    }

    /// CTRLA.ENABLE. Disabling the block stops every comparator but
    /// does NOT clear their COMPCTRLn.ENABLE bits (40.6.2.1) - they
    /// resume when the block comes back.
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        regs().AC_CTRLA = on ? AC_CTRLA_ENABLE_Msk : 0u;
        return sync_wait(AC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() { return (regs().AC_CTRLA & AC_CTRLA_ENABLE_Msk) != 0u; }

    /// Clock, reset, enable: the block up with no comparator running.
    static bool init(uint8_t gclk_generator, uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        bus_clock(true);
        if (!clock(gclk_generator, spins)) {
            return false;
        }
        if (!reset(spins)) {
            return false;
        }
        return enable(true, spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)enable(false, spins);
        GclkChannel::disconnect(AC_GCLK_ID);
        bus_clock(false);
    }

    // ---- flags (one register for all four comparators + both windows) ------

    static uint8_t flags() { return regs().AC_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().AC_INTFLAG = mask; }

    /// The ISR body: read-and-clear, the app binds AC_Handler and
    /// dispatches on the returned mask (COMPn = bit n, WINn = bit 4+n).
    [[gnu::always_inline]] static uint8_t take_flags() {
        const uint8_t f = regs().AC_INTFLAG;
        if (f != 0u) {
            regs().AC_INTFLAG = f;
        }
        return f;
    }
};

// =============================================================================
// One comparator
// =============================================================================

template <uint8_t n>
class AcComparator {
    static_assert(n < Ac::comparator_count,
                  "the SAM C21 AC implements four comparators, numbered from zero");

public:
    AcComparator() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint8_t flag = static_cast<uint8_t>(1u << n);   ///< INTFLAG.COMPn
    static constexpr uint32_t sync_mask = AC_SYNCBUSY_COMPCTRL0_Msk << n;

    /**
     * Write the whole COMPCTRLn. Every field is enable-protected
     * (writable only while COMPCTRLn.ENABLE is zero, 40.8.13), so this
     * disables first and leaves the comparator DISABLED: enable() is a
     * separate, deliberate step, exactly as with the SERCOM and the
     * DMAC channels.
     */
    static bool configure(const AcConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!enable(false, spins)) {
            return false;
        }
        Ac::regs().AC_COMPCTRL[n] = ac_compctrl(cfg);
        return true;
    }

    /// COMPCTRLn.ENABLE - write-synchronized, both directions waited.
    /// After an enable the comparator is comparing only once t_STARTUP
    /// has passed: ready() is the honest question, not this return.
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = Ac::regs().AC_COMPCTRL[n];
        Ac::regs().AC_COMPCTRL[n] =
            on ? (v | AC_COMPCTRL_ENABLE_Msk) : (v & ~AC_COMPCTRL_ENABLE_Msk);
        return Ac::sync_wait(sync_mask, spins);
    }
    static bool enabled() {
        return (Ac::regs().AC_COMPCTRL[n] & AC_COMPCTRL_ENABLE_Msk) != 0u;
    }

    /// This comparator's own 64-step VDD scaler: VDD x (value+1) / 64
    /// (40.8.12). Not enable-protected - a threshold may move live.
    static void scaler(uint8_t value) {
        Ac::regs().AC_SCALER[n] = AC_SCALER_VALUE(value);
    }

    /// Single-shot start (CTRLB is write-only: a plain store of this
    /// comparator's START bit). Clears READY; hardware sets it again
    /// when the comparison completes (40.6.2.4.2).
    static void start() {
        Ac::regs().AC_CTRLB = static_cast<uint8_t>(AC_CTRLB_START0_Msk << n);
    }

    /// STATUSB.READYn - the output is valid. STATE read before this is
    /// one is undefined by the chapter.
    static bool ready() {
        return (Ac::regs().AC_STATUSB & (AC_STATUSB_READY0_Msk << n)) != 0u;
    }

    /// STATUSA.STATEn - the sampled comparator output as the register
    /// interface sees it. How this readback relates in time to the
    /// SYNC pad output is one of the measured facts in docs/samc/ac.md.
    static bool state() {
        return (Ac::regs().AC_STATUSA & (AC_STATUSA_STATE0_Msk << n)) != 0u;
    }

    static bool flag_set() { return (Ac::regs().AC_INTFLAG & flag) != 0u; }
    static void clear_flag() { Ac::regs().AC_INTFLAG = flag; }

    /// INTENSET/INTENCLR are set-only and clear-only: plain stores.
    static void arm(bool on) {
        if (on) {
            Ac::regs().AC_INTENSET = flag;
        } else {
            Ac::regs().AC_INTENCLR = flag;
        }
    }
    static bool armed() { return (Ac::regs().AC_INTENSET & flag) != 0u; }
};

} // namespace brio
