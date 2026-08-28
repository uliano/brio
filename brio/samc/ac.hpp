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
 * WINDOW MODE (40.6.4) is the second face of every comparator PAIR:
 * COMP0/COMP1 form window 0 and COMP2/COMP3 window 1. The pair shares
 * one positive input and its two negative inputs are the window's
 * limits, in either order; WSTATE says above / inside / below and
 * WINTSEL chooses which of those four conditions raises the window's
 * own interrupt flag. The chapter asks for two things the registers do
 * not enforce - both comparators of the pair in the same measurement
 * mode (COMPCTRLn.SINGLE) and the SAME I/O pin as positive input - so
 * `AcWindow<w>::pair_consistent()` asks the silicon whether they hold.
 *
 * THE EVENT SURFACE (40.6.13) runs both ways and this header publishes
 * both vocabularies, evsys.hpp owning only the fabric:
 *  - OUT: `comparator_generator(n)` (a copy of the comparator status)
 *    and `window_generator(w)` (a copy of the inside/outside status),
 *    enabled by EVCTRL.COMPEOx / WINEOx;
 *  - IN: `start_user(n)`, the SOCn user that starts a comparison,
 *    enabled by EVCTRL.COMPEIx and optionally inverted by INVEIx. Table
 *    29-3 marks all four SOC users ASYNCHRONOUS PATH ONLY, which is a
 *    fact about the fabric a caller has to honour when it connects the
 *    channel.
 * EVCTRL is ENABLE-PROTECTED (block level), so `event_config()` refuses
 * while the block is enabled rather than storing into a dead register.
 *
 * PER-PACKAGE INPUT LEGALITY: the pair, not the comparator, owns the
 * pads - COMP0/1 take AIN[0..3] and COMP2/3 take AIN[4..7] - and 40.1
 * says the E and G variants bond only AIN[5:4] for COMP2/3. The device
 * header says the same thing in symbols (PIN_PB05B_AC_AIN6 and
 * PIN_PB06B_AC_AIN7 exist on the J alone), so that is the authority
 * here: `ac_config_valid()` refuses a pin input the package does not
 * bond, and `AcComparator<n>::configure()` returns false on it.
 *
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F. Of six AC
 * items plus one device-level one, TWO are this silicon and neither can
 * be fixed in a register:
 *  - 1.5.3 Analog Pins (all revisions): the AC and the PTC share pads,
 *    which can cost the PTC accuracy. The workaround is to give the AC
 *    a non-pin input (scaler, DAC or bandgap) while the PTC measures -
 *    a system-level choice, and there is no PTC driver here to
 *    coordinate with.
 *  - 1.5.6 Spurious COMP Interrupt (all revisions): enabling with
 *    MUXNEG = bandgap can raise a spurious COMPn flag. Microchip's
 *    workaround is to use the VDD scaler instead. This driver does NOT
 *    refuse the bandgap - it is a real and useful input - so the
 *    obligation is the caller's: after enabling a comparator whose
 *    negative input is the bandgap, clear its flag before arming.
 *    `AcNegative::bandgap` carries the note.
 *  - NOT this silicon: 1.5.1 and 1.5.4 (hysteresis, revisions B..E and
 *    B), 1.5.2 (low-power mode WITH hysteresis, B..E - the pairing is
 *    legal here), 1.5.5 (power consumption figures, B..E), and the
 *    device-level 1.8.2, which says GCLK_AC is not functional and the
 *    AC must borrow GCLK_ADC1's channel - REVISION B ONLY, so `clock()`
 *    uses AC_GCLK_ID as the header declares.
 *
 * NOT BUILT YET (docs/samc/ac.md carries the list): sleep (RUNSTDBY is
 * a config field, the 40.6.14 sequences have no owner - the power pass
 * does), and the two internal negative inputs that need a peripheral
 * this stratum has not got: the DAC, and the bandgap, whose INTREF
 * output has to be turned on in SUPC.VREF first (22.6.2.2).
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
    /// INTREF, supplied by the bandgap - and it needs SUPC.VREF.VREFOE
    /// turned on first (22.6.2.2), which no driver in this stratum does
    /// yet. ERRATUM 1.5.6, EVERY REVISION: enabling a comparator with
    /// this negative input can raise a SPURIOUS COMPn flag, so clear
    /// that flag after the enable and before arming the interrupt.
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

// ---- which analog inputs this package actually bonds --------------------------
//
// The PAIR owns the pads: COMP0/1 take AIN[0..3], COMP2/3 take
// AIN[4..7]. 40.1 says the E and G variants bond only AIN[5:4] for
// COMP2/3, and the device header says the same in symbols - the
// PIN_P<pad>B_AC_AIN<k> constants exist for exactly the bonded ones -
// so the header is the authority and no package list is kept here.

/// Which AINx a comparator's PINn code selects (40.6.3: the pair owns
/// the pads, not the comparator).
constexpr uint8_t ac_ain_of(uint8_t comparator, uint8_t pin) {
    return static_cast<uint8_t>((comparator < 2u ? 0u : 4u) + pin);
}

/// Whether this device bonds a given AIN pad at all.
constexpr bool ac_ain_exists(uint8_t ain) {
    switch (ain) {
#ifdef PIN_PA04B_AC_AIN0
    case 0: return true;
#endif
#ifdef PIN_PA05B_AC_AIN1
    case 1: return true;
#endif
#ifdef PIN_PA06B_AC_AIN2
    case 2: return true;
#endif
#ifdef PIN_PA07B_AC_AIN3
    case 3: return true;
#endif
#ifdef PIN_PA02B_AC_AIN4
    case 4: return true;
#endif
#ifdef PIN_PA03B_AC_AIN5
    case 5: return true;
#endif
#ifdef PIN_PB05B_AC_AIN6
    case 6: return true;
#endif
#ifdef PIN_PB06B_AC_AIN7
    case 7: return true;
#endif
    default: return false;
    }
}

/// The MUXPOS/MUXNEG pin codes are the same four values; this turns one
/// into the pin index, or 0xFF for a code that is not a pin at all.
constexpr uint8_t ac_pin_index(AcPositive p) {
    switch (p) {
    case AcPositive::pin0: return 0;
    case AcPositive::pin1: return 1;
    case AcPositive::pin2: return 2;
    case AcPositive::pin3: return 3;
    default: return 0xFF;
    }
}
constexpr uint8_t ac_pin_index(AcNegative n) {
    switch (n) {
    case AcNegative::pin0: return 0;
    case AcNegative::pin1: return 1;
    case AcNegative::pin2: return 2;
    case AcNegative::pin3: return 3;
    default: return 0xFF;
    }
}

// ---- window mode vocabulary (40.6.4, 40.8.10, 40.8.7) ------------------------

/// WINCTRL.WINTSELw: which of the four window conditions raises
/// INTFLAG.WINw. Events are generated from the inside/outside state
/// regardless of this selection (40.6.13).
enum class AcWindowInterrupt : uint8_t {
    above = AC_WINCTRL_WINTSEL0_ABOVE_Val,
    inside = AC_WINCTRL_WINTSEL0_INSIDE_Val,
    below = AC_WINCTRL_WINTSEL0_BELOW_Val,
    outside = AC_WINCTRL_WINTSEL0_OUTSIDE_Val,
};

/// STATUSA.WSTATEw. `unknown` is the register's reserved 0x3, which the
/// chapter does not define and the silicon should not produce.
enum class AcWindowState : uint8_t {
    above = AC_STATUSA_WSTATE0_ABOVE_Val,
    inside = AC_STATUSA_WSTATE0_INSIDE_Val,
    below = AC_STATUSA_WSTATE0_BELOW_Val,
    unknown = 3,
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

/**
 * Is this configuration legal for THIS comparator on THIS package?
 *
 * Three refusals, each with a sentence of the chapter behind it:
 *  - an input pin the package does not bond (40.1 and the device
 *    header's own AIN symbols) - which is why the comparator index has
 *    to be an argument: the pair decides which AINx a PINn means;
 *  - hysteresis in single-shot mode: "Hysteresis is available only in
 *    continuous mode (COMPCTRLx.SINGLE=0)" (40.6.6);
 *  - the end-of-comparison interrupt in continuous mode: INTSEL's own
 *    table calls EOC "single-shot mode only" (40.8.12).
 */
constexpr bool ac_config_valid(uint8_t comparator, const AcConfig& c) {
    const uint8_t pos = ac_pin_index(c.positive);
    if (pos != 0xFFu && !ac_ain_exists(ac_ain_of(comparator, pos))) {
        return false;
    }
    const uint8_t neg = ac_pin_index(c.negative);
    if (neg != 0xFFu && !ac_ain_exists(ac_ain_of(comparator, neg))) {
        return false;
    }
    if (c.hysteresis && c.single_shot) {
        return false;
    }
    return !(c.interrupt_on == AcInterrupt::end_of_comparison && !c.single_shot);
}

/// The block's event control (EVCTRL, 40.8.3) in one value: the four
/// comparator outputs, the two window outputs, and the four start-a-
/// comparison inputs with their optional inversion.
struct AcEventControl {
    uint8_t comparator_out = 0;   ///< COMPEO[3:0], bit n = comparator n
    uint8_t window_out = 0;       ///< WINEO[1:0], bit w = window w
    uint8_t start_in = 0;         ///< COMPEI[3:0]: an event starts comparison n
    uint8_t invert_in = 0;        ///< INVEI[3:0]: that event is inverted first
};

constexpr uint16_t ac_evctrl(const AcEventControl& e) {
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(e.comparator_out & 0x0Fu) << AC_EVCTRL_COMPEO0_Pos) |
        (static_cast<uint32_t>(e.window_out & 0x03u) << AC_EVCTRL_WINEO0_Pos) |
        (static_cast<uint32_t>(e.start_in & 0x0Fu) << AC_EVCTRL_COMPEI0_Pos) |
        (static_cast<uint32_t>(e.invert_in & 0x0Fu) << AC_EVCTRL_INVEI0_Pos));
}

/// INVEIx only means something for a comparator whose event input is
/// enabled - inverting an event nothing listens to is a configuration
/// asking for something the silicon cannot do.
constexpr bool ac_event_control_valid(const AcEventControl& e) {
    return (e.invert_in & ~e.start_in & 0x0Fu) == 0u;
}

// =============================================================================
// The block
// =============================================================================

class Ac {
public:
    Ac() = delete;

    static constexpr uint8_t comparator_count = 4;
    /// Comparators are grouped in pairs, and a pair is a window (40.1).
    static constexpr uint8_t window_count = comparator_count / 2;

    static constexpr IRQn_Type irq() { return AC_IRQn; }

    // ---- the EVSYS vocabulary this peripheral publishes ---------------------
    //
    // evsys.hpp owns the FABRIC and not the vocabulary, so the codes of
    // ch. 29's tables that belong to the AC live here.

    /// Generator: a copy of comparator n's status (COMP0 is 0x49).
    static constexpr uint8_t comparator_generator(uint8_t n) {
        return static_cast<uint8_t>(0x49u + n);
    }
    /// Generator: a copy of window w's inside/outside status (WIN0 is
    /// 0x4D). Generated from that state whether or not the window's
    /// interrupt is enabled (40.6.13).
    static constexpr uint8_t window_generator(uint8_t w) {
        return static_cast<uint8_t>(0x4Du + w);
    }
    /// User: SOCn, "start comparator n" (SOC0 is user 34). TABLE 29-3
    /// MARKS ALL FOUR ASYNCHRONOUS PATH ONLY - a fact about the fabric
    /// that the channel's configuration has to honour.
    static constexpr uint8_t start_user(uint8_t n) {
        return static_cast<uint8_t>(34u + n);
    }

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

    // ---- events (EVCTRL, enable-protected) ---------------------------------

    /**
     * Write the whole EVCTRL: both event directions in one store.
     *
     * ENABLE-PROTECTED (40.8.3), so this refuses while the block is
     * enabled instead of storing into a register the silicon ignores -
     * the same discipline the comparators' own enable-protected fields
     * get.
     */
    static bool event_config(const AcEventControl& e) {
        if (enabled() || !ac_event_control_valid(e)) {
            return false;
        }
        regs().AC_EVCTRL = ac_evctrl(e);
        return true;
    }

    static AcEventControl event_config() {
        const uint16_t v = regs().AC_EVCTRL;
        return AcEventControl{
            .comparator_out =
                static_cast<uint8_t>((v & AC_EVCTRL_COMPEO_Msk) >> AC_EVCTRL_COMPEO0_Pos),
            .window_out =
                static_cast<uint8_t>((v & AC_EVCTRL_WINEO_Msk) >> AC_EVCTRL_WINEO0_Pos),
            .start_in =
                static_cast<uint8_t>((v & AC_EVCTRL_COMPEI_Msk) >> AC_EVCTRL_COMPEI0_Pos),
            .invert_in =
                static_cast<uint8_t>((v & AC_EVCTRL_INVEI_Msk) >> AC_EVCTRL_INVEI0_Pos),
        };
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

    /// This comparator's own EVSYS codes, for a caller that has the
    /// comparator type but not its index to hand.
    static constexpr uint8_t event_generator = Ac::comparator_generator(n);
    static constexpr uint8_t start_event_user = Ac::start_user(n);

    /// Whether a configuration is legal for THIS comparator on THIS
    /// package - the pair decides which AINx a `pinK` code means, so
    /// the index has to be part of the question. See ac_config_valid().
    static constexpr bool config_valid(const AcConfig& cfg) {
        return ac_config_valid(n, cfg);
    }

    /**
     * Write the whole COMPCTRLn. Every field is enable-protected
     * (writable only while COMPCTRLn.ENABLE is zero, 40.8.13), so this
     * disables first and leaves the comparator DISABLED: enable() is a
     * separate, deliberate step, exactly as with the SERCOM and the
     * DMAC channels.
     *
     * A configuration this package cannot honour is REFUSED rather than
     * written - an input pin the device does not bond, hysteresis in
     * single-shot mode, or the end-of-comparison interrupt in
     * continuous mode.
     */
    static bool configure(const AcConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!config_valid(cfg)) {
            return false;
        }
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

    /// COMPCTRLn's SINGLE bit as the silicon holds it - what
    /// AcWindow<w>::pair_consistent() asks both comparators of a pair.
    static bool single_shot() {
        return (Ac::regs().AC_COMPCTRL[n] & AC_COMPCTRL_SINGLE_Msk) != 0u;
    }
    /// COMPCTRLn.MUXPOS as the silicon holds it, likewise.
    static AcPositive positive() {
        return static_cast<AcPositive>(
            (Ac::regs().AC_COMPCTRL[n] & AC_COMPCTRL_MUXPOS_Msk) >>
            AC_COMPCTRL_MUXPOS_Pos);
    }
};

// =============================================================================
// One window: a comparator PAIR working together (40.6.4)
// =============================================================================

/**
 * `AcWindow<0>` is COMP0 and COMP1, `AcWindow<1>` is COMP2 and COMP3.
 *
 * The pair shares one positive input - the signal - and its two
 * negative inputs are the window's limits, in EITHER order ("the window
 * will also work in the opposite configuration with COMP0 lower and
 * COMP1 higher"). WSTATE reports above / inside / below; WINTSEL says
 * which of four conditions raises INTFLAG.WINw, while the window's
 * EVSYS event is generated from the inside/outside state whether the
 * interrupt is enabled or not (40.6.13).
 *
 * The individual comparators keep working throughout: their own STATE,
 * flags, interrupts and events are unaffected by window mode.
 *
 * WINCTRL is WRITE-SYNCHRONIZED but NOT enable-protected (40.8.10), so
 * a window may be turned on and its interrupt condition changed under a
 * running block - unlike everything in COMPCTRLn.
 */
template <uint8_t w>
class AcWindow {
    static_assert(w < Ac::window_count,
                  "this family pairs its four comparators into two windows");

public:
    AcWindow() = delete;

    static constexpr uint8_t index = w;
    using lower_comparator = AcComparator<static_cast<uint8_t>(2u * w)>;
    using upper_comparator = AcComparator<static_cast<uint8_t>(2u * w + 1u)>;

    /// INTFLAG.WINw - the window flags sit above the four comparator
    /// ones in the same register.
    static constexpr uint8_t flag = static_cast<uint8_t>(AC_INTFLAG_WIN0_Msk << w);
    /// This window's EVSYS generator code.
    static constexpr uint8_t event_generator = Ac::window_generator(w);

    /**
     * Enable the window and choose its interrupt condition in one
     * write-synchronized store. Only this window's two bits move; the
     * other window's are preserved.
     */
    static bool configure(bool on, AcWindowInterrupt on_condition,
                          uint32_t spins = 0xFFFFu) {
        const uint8_t keep = static_cast<uint8_t>(
            Ac::regs().AC_WINCTRL &
            ~static_cast<uint8_t>(AC_WINCTRL_WEN0_Msk << (4u * w) |
                                  AC_WINCTRL_WINTSEL0_Msk << (4u * w)));
        const uint8_t mine = static_cast<uint8_t>(
            ((on ? AC_WINCTRL_WEN0_Msk : 0u) |
             (static_cast<uint32_t>(on_condition) << AC_WINCTRL_WINTSEL0_Pos))
            << (4u * w));
        Ac::regs().AC_WINCTRL = static_cast<uint8_t>(keep | mine);
        return Ac::sync_wait(AC_SYNCBUSY_WINCTRL_Msk, spins);
    }

    static bool enabled() {
        return (Ac::regs().AC_WINCTRL & (AC_WINCTRL_WEN0_Msk << (4u * w))) != 0u;
    }
    static AcWindowInterrupt interrupt_on() {
        return static_cast<AcWindowInterrupt>(
            (Ac::regs().AC_WINCTRL >> (AC_WINCTRL_WINTSEL0_Pos + 4u * w)) & 0x3u);
    }

    /// STATUSA.WSTATEw. Meaningful only once BOTH comparators are
    /// ready, which is what ready() answers.
    static AcWindowState state() {
        return static_cast<AcWindowState>(
            (Ac::regs().AC_STATUSA >> (AC_STATUSA_WSTATE0_Pos + 2u * w)) & 0x3u);
    }

    static bool ready() {
        return lower_comparator::ready() && upper_comparator::ready();
    }

    /**
     * The two things 40.6.4 REQUIRES and no register enforces: both
     * comparators of the pair in the same measurement mode, and the
     * same positive input on both so the pair really does see one
     * signal. Asked of the silicon rather than of a configuration
     * struct, because the two comparators are configured separately and
     * nothing else would notice them drifting apart.
     */
    static bool pair_consistent() {
        return lower_comparator::single_shot() == upper_comparator::single_shot() &&
               lower_comparator::positive() == upper_comparator::positive();
    }

    /// In single-shot mode either comparator's START starts BOTH
    /// measurements (40.6.4), so one verb is the honest surface.
    static void start() { lower_comparator::start(); }

    static bool flag_set() { return (Ac::regs().AC_INTFLAG & flag) != 0u; }
    static void clear_flag() { Ac::regs().AC_INTFLAG = flag; }
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
