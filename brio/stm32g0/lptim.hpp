/*
 * lptim.hpp
 *
 * The STM32G0's low-power timers (RM0444 ch. 26) in the two strata every
 * brio target uses (docs/design/overview.md, "Target strata"):
 *
 *  Lptim<n>        the RESOURCE - one LPTIMx block: the kernel-clock
 *                  multiplexer, the prescaler, the counter with its two
 *                  reset mechanisms, the compare and auto-reload with
 *                  their write handshake, the trigger multiplexer, the
 *                  waveform generator, the glitch filters, encoder mode,
 *                  the flags, the ISR body and the wake line.
 *
 *  LptimPad<sel>   a pad handed to IN1, IN2, ETR or OUT (the AF number
 *                  is the DATASHEET's - the tim.hpp rule again).
 *
 *  TASKS           LptimPwm (util/pwm_channel.hpp's PwmChannel, its
 *                  FOURTH implementation), LptimPeriodicTick,
 *                  LptimCounter, LptimPulseCounter, LptimTimeout,
 *                  LptimEncoder.
 *
 * SIX FACTS THAT SHAPE THIS FILE.
 *
 * 1. NOTHING HERE EVER CLEARS ENABLE. ES0548 2.8.1 (revision Z): if the
 *    application clears CR.ENABLE "within a small time window around one
 *    LPTIM interrupt occurrence", the wake-up signal that leaves this
 *    block can freeze in its active state - after which the device
 *    cannot enter Stop at all and the firmware sits in the interrupt
 *    routine for ever. The erratum's own workaround is not a sequence
 *    but a substitution: "do not clear its ENABLE bit... instead, reset
 *    the whole LPTIMx peripheral via the RCC controller". So `disable()`
 *    IS an RCC reset pulse, `reset()` is the same pulse spelled for the
 *    init path, and there is no verb in this file that stores a zero
 *    into CR.ENABLE. The cost is that a disable also forgets the
 *    configuration - which is why every task's setup() configures from
 *    scratch.
 *
 * 2. THE FLAG CLEAR IS ORDERED, AND ONLY THE HANDLER MAY DO IT. ES0548
 *    2.8.2 (revision Z): with at least one interrupt enabled in
 *    LPTIM_IER, clearing a flag whose interrupt is DISABLED at the same
 *    instant as a new event is detected leaves the interrupt signal
 *    "permanently stuck high". The workaround has three parts and all
 *    three are code here:
 *      - clear a flag only when its interrupt is enabled, or
 *      - clear the disabled-interrupt flags FIRST, in the same routine;
 *      - and never clear outside the interrupt routine at all.
 *    So `clear_flags()` REFUSES (false, nothing written) when IER is
 *    nonzero and the core is not in handler mode - `__get_IPSR() != 0`,
 *    which is one instruction - and `isr()` does the two-step clear
 *    itself: the pending flags whose enable is CLEAR first, then those
 *    whose enable is SET. With IER == 0 the erratum's precondition is
 *    absent and `clear_flags()` is unrestricted, which is what makes the
 *    polling tasks (LptimPwm's compare handshake) legal.
 *    `clear_flags_raw()` is the unguarded store, named for what it is:
 *    the ISR body's own verb and the bench's staging one (the rtc.hpp
 *    raw-verb precedent).
 *
 * 3. THE TWO SIDES OF THE BLOCK HAVE OPPOSITE ENABLE RULES, and getting
 *    them backwards is the chapter's easiest mistake. CFGR, CFGR2 and
 *    IER may be written ONLY WHILE DISABLED (26.7.3, 26.7.4, 26.7.9);
 *    CMP and ARR may be written ONLY WHILE ENABLED (26.7.6, 26.7.7);
 *    CNTSTRT/SNGSTRT/COUNTRST/RSTARE are discarded while disabled
 *    (26.4.7, 26.7.5). Every verb here checks its own side and returns
 *    false rather than storing into a register the silicon will ignore.
 *
 * 4. A WRITE TO CMP OR ARR IS NOT DONE WHEN THE STORE RETURNS. The APB
 *    interface and the kernel logic run on different clocks, so 26.4.11
 *    forbids a second write "before respectively the ARROK flag or the
 *    CMPOK flag be set" and promises "unpredictable results" otherwise.
 *    This driver therefore separates the STORE (set_cmp/set_arr, which
 *    neither clear nor wait) from the OBSERVATION (cmp_ok/arr_ok reads,
 *    wait_cmp_ok/wait_arr_ok bounded polls). The handshake is the
 *    caller's to spend, and every task in this file spends it.
 *
 * 5. A COUNTER READ IS TWO READS. 26.7.8: with an asynchronous kernel
 *    clock a single read of CNT "may return unreliable values" and two
 *    consecutive reads must agree. `count()` does that and returns
 *    std::optional - nothing when they never agreed inside the bound.
 *    `count_raw()` is the single read, which exists because 26.4.14 says
 *    the double read is IMPOSSIBLE with RSTARE set (the first read
 *    resets the counter), and the two reset mechanisms are exclusive by
 *    the same paragraph's warning.
 *
 * 6. TWO INSTANCES, ONE LPTIM_TypeDef, AND A TABLE. Table 135 gives
 *    encoder mode to LPTIM1 alone, and figure 271's footnote gives
 *    LPTIM2 only one input channel - but the device header declares
 *    CFGR.ENC, ISR.UP/DOWN and CFGR2.IN2SEL once, for the struct both
 *    share, so `LPTIM2->CFGR |= LPTIM_CFGR_ENC` compiles and writes a
 *    Reserved bit. What an instance IS therefore comes from
 *    stm32g0/device_tables.hpp with its citation (the tim.hpp geometry
 *    precedent), every verb that names a missing feature refuses, and
 *    LptimEncoder static_asserts.
 *
 * THE KERNEL CLOCK. RCC_CCIPR.LPTIMnSEL (5.4.21) offers PCLK, LSI, HSI16
 * and LSE to BOTH instances on every part of this pack. Only LSI and LSE
 * keep counting through Stop 0/1 with no further ceremony (26.5, table
 * 145); HSI16 is a peripheral clock REQUEST (5.3 lists the LPTIMs among
 * the blocks that can ask for HSI16 in Stop), and a request is exactly
 * what ES0548 2.2.4 breaks when RCC_CR.HSIDIV is nonzero - so an LPTIM
 * on HSI16 as a wake source on a divided HSI is a configuration this
 * file names and does not recommend. PCLK stops with the VCORE domain
 * and is a Sleep-mode-only clock.
 *
 * THE WAKE PATH. Table 65 gives LPTIM1 EXTI line 29 and LPTIM2 line 30,
 * both DIRECT: no trigger selection, no pending bit of the EXTI's own -
 * the peripheral's own ISR flag IS the pending state - and the line's
 * IMR bit must be set for the interrupt to bring the core out of Stop.
 * `wake_line(true)` is that one bit (the rtc.hpp precedent).
 *
 * THE VECTOR. On the G0B1/G0C1 and the G071 class LPTIM1 shares
 * TIM6_DAC_LPTIM1_IRQn with TIM6 and the DAC and LPTIM2 shares
 * TIM7_LPTIM2_IRQn with TIM7; on the G031 class each has a line of its
 * own. `Lptim<n>::irq()` reads that off the reserve; `isr()` returns the
 * mask it served, so a shared handler can tell "not mine" (0) from
 * "mine" - the shared-vector contract of this stratum.
 *
 * THE PAD MAP IS THE DATASHEET'S AND NOTHING CHECKS IT (DS13560 tables
 * 13..24), exactly as for the timers: `LptimPad<sel>` is the caller's
 * claim and the bench is the only check. test_stm32_lptim measures four
 * of those claims.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// RCC_CCIPR.LPTIMnSEL (5.4.21): which clock the kernel logic counts.
/// The codes are the register's own.
enum class LptimClock : uint8_t {
    pclk = 0,    ///< the APB clock - stops in Stop
    lsi = 1,     ///< runs in Stop 0/1 (26.5)
    hsi16 = 2,   ///< a clock REQUEST in Stop - see the file header
    lse = 3,     ///< runs in Stop 0/1 (26.5)
};

/// Does this kernel clock survive a Stop with no further condition?
/// 26.5's table 145: "No effect when LPTIM is clocked by LSE or LSI".
constexpr bool lptim_clock_runs_in_stop(LptimClock c) {
    return c == LptimClock::lsi || c == LptimClock::lse;
}

/// CFGR.CKSEL: where the counter's clock comes from.
enum class LptimClockSource : uint8_t {
    internal = 0,          ///< the kernel clock above
    external_input1 = 1,   ///< IN1 IS the clock - no oscillator needed (26.4.12)
};

/// CFGR.PRESC (table 143): the power-of-two prescaler.
enum class LptimPrescaler : uint8_t {
    div1 = 0, div2 = 1, div4 = 2, div8 = 3,
    div16 = 4, div32 = 5, div64 = 6, div128 = 7,
};

constexpr uint32_t lptim_prescaler_divider(LptimPrescaler p) {
    return 1UL << static_cast<uint8_t>(p);
}

/// The rate the 16-bit counter really advances at.
constexpr uint32_t lptim_counter_hz(uint32_t kernel_hz, LptimPrescaler p) {
    return kernel_hz / lptim_prescaler_divider(p);
}

/// CFGR.CKPOL. With an EXTERNAL clock these are the counting edges
/// (26.4.12); in encoder mode they select the sub-mode of table 144;
/// with an internal clock and COUNTMODE = 0 the field means nothing.
/// Code 11 is "not allowed" (26.7.4) and is not spelled.
enum class LptimClockPolarity : uint8_t {
    rising = 0,
    falling = 1,
    both = 2,
};

/// CFGR.CKFLT and CFGR.TRGFLT: how many consecutive equal samples make a
/// transition. The two fields share this vocabulary and, per 26.4.5, are
/// controlled BY GROUP - one sensitivity for every external input, one
/// for every trigger.
enum class LptimFilter : uint8_t {
    none = 0,       ///< any level change is a transition
    samples2 = 1,
    samples4 = 2,
    samples8 = 3,
};

/**
 * CFGR.TRIGSEL, tables 138 and 139.
 *
 * Rows 0..4, 6 and 7 are the same signal on both instances. ROW 5 IS
 * NOT: it is COMP3_OUT on LPTIM1 and TAMP_TRG3 on LPTIM2. Two
 * enumerators with the same value would be indistinguishable to a
 * checker, so the LPTIM2 spelling carries a marker bit that
 * `lptim_trigger_code()` strips and `lptim_trigger_valid()` reads - the
 * row is then named honestly on both instances AND refusable on the
 * wrong one.
 */
enum class LptimTrigger : uint8_t {
    etr_pad = 0,        ///< lptim_ext_trig0: the LPTIMx_ETR alternate function
    rtc_alarm_a = 1,    ///< lptim_ext_trig1
    rtc_alarm_b = 2,    ///< lptim_ext_trig2
    tamp1 = 3,          ///< lptim_ext_trig3: TAMP1 input detection
    tamp2 = 4,          ///< lptim_ext_trig4: TAMP2 input detection
    comp3_out = 5,          ///< lptim_ext_trig5 - LPTIM1 ONLY
    tamp_trg3 = 5 | 0x80,   ///< lptim_ext_trig5 - LPTIM2 ONLY
    comp1_out = 6,      ///< lptim_ext_trig6
    comp2_out = 7,      ///< lptim_ext_trig7
};

constexpr uint8_t lptim_trigger_code(LptimTrigger t) {
    return static_cast<uint8_t>(static_cast<uint8_t>(t) & 0x07u);
}

/// Whether instance `n` really has that trigger row - which is two
/// questions: the row-5 split above, and whether the COMPARATOR a row
/// names exists on this part at all (the G031 class has none, and the
/// third is the G0B1/G0C1's alone - stm32g0/device_tables.hpp).
constexpr bool lptim_trigger_valid(uint8_t n, LptimTrigger t) {
    switch (t) {
        case LptimTrigger::comp3_out: return n == 1u && comp_present(3);
        case LptimTrigger::tamp_trg3: return n == 2u;
        case LptimTrigger::comp1_out: return comp_present(1);
        case LptimTrigger::comp2_out: return comp_present(2);
        default: break;
    }
    return static_cast<uint8_t>(t) <= 4u;
}

/// CFGR.TRIGEN: whether an external trigger starts the counter, and on
/// which edge. `software` is TRIGEN = 00, where CNTSTRT/SNGSTRT start it
/// directly and TRIGSEL means nothing.
enum class LptimTriggerEdge : uint8_t {
    software = 0,
    rising = 1,
    falling = 2,
    both = 3,
};

/// CFGR.WAVE (26.4.10). `pwm_or_pulse` is WAVE = 0, where WHICH of the
/// two it is depends on the start verb: CNTSTRT gives PWM, SNGSTRT gives
/// one pulse. WAVE = 1 is Set-once: one pulse, then the output stays at
/// its last level and every later trigger is discarded.
enum class LptimWaveform : uint8_t {
    pwm_or_pulse = 0,
    set_once = 1,
};

/**
 * CFGR2.IN1SEL, tables 140 and 142 - AND THE TWO TABLES DO NOT AGREE,
 * which is why the enumerators are named for the SIGNAL and the checker
 * takes the instance:
 *   LPTIM1: mux0 pad, mux1 COMP1_OUT, mux2 and mux3 NOT CONNECTED;
 *   LPTIM2: mux0 pad, mux1 COMP1_OUT, mux2 COMP2_OUT, mux3 either.
 */
enum class LptimInput1 : uint8_t {
    pad = 0,
    comp1_out = 1,
    comp2_out = 2,        ///< LPTIM2 only
    comp1_or_comp2 = 3,   ///< LPTIM2 only
};

constexpr bool lptim_input1_valid(uint8_t n, LptimInput1 s) {
    switch (s) {
        case LptimInput1::pad: return true;
        case LptimInput1::comp1_out: return comp_present(1);
        case LptimInput1::comp2_out: return n == 2u && comp_present(2);
        case LptimInput1::comp1_or_comp2:
            return n == 2u && comp_present(1) && comp_present(2);
    }
    return false;
}

/// CFGR2.IN2SEL, table 141 - LPTIM1's alone, and only two of its four
/// codes are connected to anything.
enum class LptimInput2 : uint8_t {
    pad = 0,
    comp2_out = 1,
};

constexpr bool lptim_input2_valid(uint8_t n, LptimInput2 s) {
    if (!lptim_has_input2(n)) {
        return s == LptimInput2::pad;   // the field is Reserved; only "leave it" is legal
    }
    return s == LptimInput2::pad || comp_present(2);
}

/// The flags of LPTIM_ISR (26.7.1). The SAME bit positions serve
/// LPTIM_ICR (26.7.2) and LPTIM_IER (26.7.3), which is what lets isr()
/// mask one register with another and what makes one `interrupts` mask
/// in the config mean the same thing as one `clear_flags` mask.
struct LptimFlag {
    static constexpr uint32_t cmpm = LPTIM_ISR_CMPM;         ///< compare match
    static constexpr uint32_t arrm = LPTIM_ISR_ARRM;         ///< auto-reload match
    static constexpr uint32_t exttrig = LPTIM_ISR_EXTTRIG;   ///< external trigger edge
    static constexpr uint32_t cmpok = LPTIM_ISR_CMPOK;       ///< CMP write completed
    static constexpr uint32_t arrok = LPTIM_ISR_ARROK;       ///< ARR write completed
    static constexpr uint32_t up = LPTIM_ISR_UP;             ///< encoder: down to up
    static constexpr uint32_t down = LPTIM_ISR_DOWN;         ///< encoder: up to down
    static constexpr uint32_t all = cmpm | arrm | exttrig | cmpok | arrok | up | down;
    /// The two that only exist where encoder mode does (26.7.1's notes).
    static constexpr uint32_t encoder_only = up | down;
};

static_assert(LPTIM_ISR_CMPM == LPTIM_IER_CMPMIE && LPTIM_ISR_ARRM == LPTIM_IER_ARRMIE &&
                  LPTIM_ISR_CMPOK == LPTIM_IER_CMPOKIE &&
                  LPTIM_ISR_ARROK == LPTIM_IER_ARROKIE &&
                  LPTIM_ISR_EXTTRIG == LPTIM_IER_EXTTRIGIE &&
                  LPTIM_ISR_CMPM == LPTIM_ICR_CMPMCF &&
                  LPTIM_ISR_ARRM == LPTIM_ICR_ARRMCF,
              "brio lptim: LPTIM_ISR, LPTIM_ICR and LPTIM_IER are assumed to share "
              "bit positions (RM0444 26.7.1..26.7.3) - isr() ANDs them");

/**
 * Everything CFGR, CFGR2 and IER hold. All three are DISABLED-ONLY
 * registers, so they are one verb: a configuration is a state the block
 * is put into while it is off, and there is no half of it that can be
 * changed later.
 */
struct LptimConfig {
    LptimClockSource clock = LptimClockSource::internal;   ///< CKSEL
    /// COUNTMODE: with an internal clock, count the EDGES on IN1 sampled
    /// by that clock instead of the clock itself. 26.4.12 makes this
    /// mode require PRESC = /1 and an input slower than the kernel clock.
    bool count_external = false;
    LptimPrescaler prescaler = LptimPrescaler::div1;       ///< PRESC
    LptimClockPolarity clock_polarity = LptimClockPolarity::rising;   ///< CKPOL
    /// CKFLT / TRGFLT. THE OBLIGATION THIS DRIVER CANNOT ENFORCE
    /// (26.4.5): "before activating the digital filters, an internal
    /// clock source should first be provided to the LPTIM" - and whether
    /// the selected kernel clock is actually RUNNING is an RCC question,
    /// not a CFGR one. `Lptim<n>::kernel_clock_running()` is the read
    /// that answers it; with no internal clock both fields must be
    /// `none` or 26.4.5 promises nothing about the filters at all.
    LptimFilter clock_filter = LptimFilter::none;
    LptimFilter trigger_filter = LptimFilter::none;
    LptimTrigger trigger = LptimTrigger::etr_pad;          ///< TRIGSEL
    LptimTriggerEdge trigger_edge = LptimTriggerEdge::software;   ///< TRIGEN
    /// TIMOUT: a trigger arriving while the counter runs RESETS it
    /// instead of being ignored - the timeout function of 26.4.9.
    bool timeout = false;
    LptimWaveform waveform = LptimWaveform::pwm_or_pulse;  ///< WAVE
    bool output_inverted = false;                          ///< WAVPOL
    /// PRELOAD: CMP and ARR taken at the end of the current period
    /// rather than at once (26.4.11). A PWM whose duty may change while
    /// it runs wants this set; a counter being programmed does not.
    bool preload = false;
    bool encoder = false;                                  ///< ENC - LPTIM1 only
    LptimInput1 input1 = LptimInput1::pad;                 ///< CFGR2.IN1SEL
    LptimInput2 input2 = LptimInput2::pad;                 ///< CFGR2.IN2SEL
    /// LPTIM_IER, as a set of LptimFlag bits. Nonzero here is what turns
    /// on ES0548 2.8.2's precondition, so a polling user leaves it 0.
    uint32_t interrupts = 0;
};

/**
 * Whether a configuration is legal for instance `n` - the same judgment
 * configure() makes, constexpr so `init<cfg>()` and the tasks can
 * static_assert on it. The rules, each with its citation:
 *
 *  - CKPOL 11 is "not allowed" (26.7.4);
 *  - both edges with an EXTERNAL clock is refused outright: 26.4.12 says
 *    the counter can be updated "either on rising edges or falling edges
 *    ... but not on both", and 26.4.4's both-edges case additionally
 *    demands an internal clock at four times the external rate, which
 *    nothing in this driver can know. With COUNTMODE = 1 (an internal
 *    clock sampling IN1) both edges ARE legal and that same 4x rule is
 *    the caller's obligation;
 *  - COUNTMODE = 1 requires PRESC = /1 (26.4.12, in so many words);
 *  - encoder mode requires the instance to have it (table 135), an
 *    internal clock and PRESC = /1 (26.4.15's Caution), and it wants the
 *    second input, so it is LPTIM1's alone twice over;
 *  - a trigger row and an input multiplexer code must exist on THIS
 *    instance and on THIS part (tables 138..142 plus which comparators
 *    the part bonds);
 *  - the UP/DOWN interrupt enables are Reserved where encoder mode is
 *    (26.7.3's notes), and no bit outside LptimFlag::all may be set.
 */
constexpr bool lptim_config_valid(uint8_t n, const LptimConfig& c) {
    if (!lptim_present(n)) {
        return false;
    }
    if (static_cast<uint8_t>(c.clock_polarity) > 2u) {
        return false;
    }
    const bool external = c.clock == LptimClockSource::external_input1;
    if (external && c.clock_polarity == LptimClockPolarity::both) {
        return false;
    }
    if (c.count_external && c.prescaler != LptimPrescaler::div1) {
        return false;
    }
    if (c.encoder) {
        if (!lptim_has_encoder(n) || external ||
            c.prescaler != LptimPrescaler::div1) {
            return false;
        }
    }
    if (c.trigger_edge != LptimTriggerEdge::software &&
        !lptim_trigger_valid(n, c.trigger)) {
        return false;
    }
    if (!lptim_input1_valid(n, c.input1) || !lptim_input2_valid(n, c.input2)) {
        return false;
    }
    if ((c.interrupts & ~LptimFlag::all) != 0u) {
        return false;
    }
    if (!lptim_has_encoder(n) && (c.interrupts & LptimFlag::encoder_only) != 0u) {
        return false;
    }
    return true;
}

/// A pad handed to an LPTIM signal.
///
///   constexpr brio::PinSel out{'B', 0, brio::PinFunction::af5};  // LPTIM1_OUT
///   using Wave = brio::LptimPad<out>;
///   Wave::claim();
///
/// The AF NUMBER IS THE DATASHEET'S (DS13560 tables 13..24) and no
/// symbol of the device header can check it. The pad's input buffer
/// stays live in alternate mode (7.3.1), so an output is readable on
/// IDR and an EXTI line can watch it - and an INPUT pad still follows
/// its own internal pull, which is what makes this chapter measurable
/// with no wire.
template <PinSel sel>
struct LptimPad {
    LptimPad() = delete;

    static_assert(sel.valid(),
                  "brio LptimPad: this device has no such pad (port absent, or a "
                  "pin number past 15)");

    using pin = Pin<sel.port, sel.pin>;
    static constexpr PinSel selection = sel;

    /// Hand the pad to the LPTIM as the OUTPUT of that alternate
    /// function (LPTIMx_OUT).
    static void claim(PinSpeed speed = PinSpeed::low, bool open_drain = false) {
        pin::function(sel.function, {.open_drain = open_drain, .speed = speed});
    }
    /// The same handover for an INPUT (IN1, IN2, ETR), with a pull if
    /// the pad needs one to rest somewhere.
    static void claim_input(PinPull pull = PinPull::none) {
        pin::function(sel.function, {.pull = pull});
    }
    static void release() { pin::release(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Lptim<n>: one LPTIMx block.
 *
 *   using Tick = brio::Lptim<1>;
 *   Tick::init();                                   // clock on, RCC reset
 *   Tick::kernel_clock(brio::LptimClock::lse);
 *   Tick::configure({.prescaler = brio::LptimPrescaler::div32,
 *                    .interrupts = brio::LptimFlag::arrm});
 *   Tick::enable();
 *   Tick::set_arr(1023);
 *   (void)Tick::wait_arr_ok();
 *   Tick::start_continuous();
 *
 * Every verb that names a feature this instance does not have returns
 * false and writes NOTHING (fact 6 of the file header); every verb on
 * the wrong side of the enable rule does the same (fact 3).
 */
template <uint8_t n>
class Lptim {
public:
    Lptim() = delete;

    static_assert(lptim_present(n),
                  "brio Lptim: this device has no such low-power timer (every "
                  "STM32G0x1 of the pack has LPTIM1 and LPTIM2, and no part has "
                  "a third)");

    static constexpr uint8_t instance = n;

    // ---- what this instance IS (stm32g0/device_tables.hpp) ------------------
    static constexpr bool has_encoder = lptim_has_encoder(n);
    static constexpr bool has_input2 = lptim_has_input2(n);
    /// EXTI line 29 (LPTIM1) or 30 (LPTIM2), both DIRECT.
    static constexpr uint8_t exti_line = lptim_exti_line(n);
    /// Table 56's DMAMUX TRIGGER input this timer's output drives - a
    /// trigger for a request GENERATOR, not a request line: an LPTIM has
    /// no DMA request of its own on this family. Published HERE per the
    /// stratum's ruling that a fabric driver owns the fabric and a
    /// peripheral publishes its own codes.
    static constexpr uint8_t dmamux_generator_input = lptim_dmamux_trigger(n);

    static constexpr IRQn_Type irq() { return lptim_irq(n); }

    static LPTIM_TypeDef& regs() {
        return *reinterpret_cast<LPTIM_TypeDef*>(lptim_base(n));
    }

    // ---- the bus clock, the kernel clock and the reset -----------------------
    //
    // 5.2.17: a peripheral whose enable bit is clear does not answer
    // register reads at all, so init() opens the gate first. The readback
    // after the store is the two-cycle stall the chapter asks software to
    // account for - Rcc's own verbs pay it.

    static void bus_clock(bool on) {
        Rcc::apb1_clock(lptim_bus_clock_mask(n), on);
    }
    static bool bus_clock() { return Rcc::apb1_clock(lptim_bus_clock_mask(n)); }

    /// RCC_CCIPR.LPTIMnSEL. Written here and not in clock.hpp for the
    /// same reason the USART's is written in usart.hpp: the multiplexer
    /// belongs to the peripheral that counts its output. The field is in
    /// the RCC, so it SURVIVES this block's reset - which is why init()
    /// does not touch it and a caller sets it once.
    static void kernel_clock(LptimClock c) {
        Rcc::kernel_clock(lptim_clock_select_pos(n), static_cast<uint8_t>(c));
    }
    static LptimClock kernel_clock() {
        return static_cast<LptimClock>(Rcc::kernel_clock(lptim_clock_select_pos(n)));
    }

    /// Is the selected kernel clock actually RUNNING? The read 26.4.5's
    /// filter obligation needs and `lptim_config_valid()` cannot make:
    /// PCLK always is (this program is executing), the two 32 kHz roots
    /// and HSI16 report their own ready bits.
    static bool kernel_clock_running() {
        switch (kernel_clock()) {
            case LptimClock::pclk: return true;
            case LptimClock::lsi: return Rcc::lsi_ready();
            case LptimClock::hsi16: return Rcc::hsi_ready();
            case LptimClock::lse: return (RCC->BDCR & RCC_BDCR_LSERDY) != 0u;
        }
        return false;
    }

    /**
     * Pulse RCC_APBRSTR1: every register back to the value table 147
     * prints, and THE ONLY WAY THIS DRIVER TURNS AN LPTIM OFF (fact 1 of
     * the file header, ES0548 2.8.1). The bus clock and the CCIPR
     * selection are not touched - neither lives in this block.
     */
    static void reset() { Rcc::apb1_reset(lptim_reset_mask(n)); }

    /// The erratum's substitution, spelled for the reader who is looking
    /// for a disable: it is the same pulse as reset(), and it forgets the
    /// configuration - which a CR.ENABLE clear would not have. That is
    /// the price of the workaround and there is no cheaper one.
    static void disable() { reset(); }

    /// Clock on, then reset. The whole of "bring this timer up".
    static void init() {
        bus_clock(true);
        reset();
    }

    /// Block reset and its clock closed. Pads are NOT touched - a pad is
    /// the caller's claim and LptimPad::release() is its release.
    static void release() {
        reset();
        bus_clock(false);
    }

    // ---- configuration (26.7.3, 26.7.4, 26.7.9) -----------------------------

    /// Whether a configuration is legal for this instance - the free
    /// function, bound to n.
    static constexpr bool config_valid(const LptimConfig& c) {
        return lptim_config_valid(n, c);
    }

    /**
     * CFGR, CFGR2 and IER in one store each, REFUSED while the block is
     * enabled (fact 3) and for a configuration `config_valid()` rejects.
     *
     * CFGR2's IN2SEL half is written only on an instance that HAS a
     * second input; on the other it is left at zero, because 26.7.9's
     * note makes those bits Reserved there.
     *
     * ONE REFUSAL HERE HAS NO COMPILE-TIME TWIN, and cannot have one:
     * 26.4.5 says "before activating the digital filters, an internal
     * clock source should first be provided to the LPTIM", and whether
     * the selected kernel clock is RUNNING is an RCC fact, not a
     * configuration one. So a nonzero CKFLT or TRGFLT is refused HERE,
     * where kernel_clock_running() can be asked, and `lptim_config_valid
     * ()` - which knows nothing of the RCC - lets it through.
     */
    static bool configure(const LptimConfig& c) {
        if (enabled() || !config_valid(c)) {
            return false;
        }
        if ((c.clock_filter != LptimFilter::none ||
             c.trigger_filter != LptimFilter::none) &&
            !kernel_clock_running()) {
            return false;   // 26.4.5's own note, as far as it is checkable
        }
        LPTIM_TypeDef& r = regs();
        uint32_t cfgr =
            (c.clock == LptimClockSource::external_input1 ? LPTIM_CFGR_CKSEL : 0u) |
            (static_cast<uint32_t>(c.clock_polarity) << LPTIM_CFGR_CKPOL_Pos) |
            (static_cast<uint32_t>(c.clock_filter) << LPTIM_CFGR_CKFLT_Pos) |
            (static_cast<uint32_t>(c.trigger_filter) << LPTIM_CFGR_TRGFLT_Pos) |
            (static_cast<uint32_t>(c.prescaler) << LPTIM_CFGR_PRESC_Pos) |
            (static_cast<uint32_t>(lptim_trigger_code(c.trigger)) << LPTIM_CFGR_TRIGSEL_Pos) |
            (static_cast<uint32_t>(c.trigger_edge) << LPTIM_CFGR_TRIGEN_Pos) |
            (c.timeout ? LPTIM_CFGR_TIMOUT : 0u) |
            (static_cast<uint32_t>(c.waveform) << LPTIM_CFGR_WAVE_Pos) |
            (c.output_inverted ? LPTIM_CFGR_WAVPOL : 0u) |
            (c.preload ? LPTIM_CFGR_PRELOAD : 0u) |
            (c.count_external ? LPTIM_CFGR_COUNTMODE : 0u);
        if constexpr (has_encoder) {
            cfgr |= c.encoder ? LPTIM_CFGR_ENC : 0u;
        }
        r.CFGR = cfgr;
        uint32_t cfgr2 = static_cast<uint32_t>(c.input1) << LPTIM_CFGR2_IN1SEL_Pos;
        if constexpr (has_input2) {
            cfgr2 |= static_cast<uint32_t>(c.input2) << LPTIM_CFGR2_IN2SEL_Pos;
        }
        r.CFGR2 = cfgr2;
        r.IER = c.interrupts;
        return true;
    }

    /// The compile-time twin (the reset.hpp Iwdg/Wwdg spelling): the
    /// same store, with the rule a bad configuration broke named in the
    /// assertion.
    template <LptimConfig c>
    static bool configure() {
        static_assert(lptim_config_valid(n, c),
                      "brio Lptim: illegal configuration - CKPOL 11 does not exist; "
                      "both edges are refused with an external clock (26.4.12); "
                      "COUNTMODE = 1 needs PRESC = /1 (26.4.12); encoder mode needs "
                      "LPTIM1, an internal clock and PRESC = /1 (table 135, "
                      "26.4.15); a trigger row or input code must exist on this "
                      "instance and this part (tables 138..142); and the UP/DOWN "
                      "interrupts are Reserved without encoder mode (26.7.3)");
        return configure(c);
    }

    /// LPTIM_IER alone, on its own terms: disabled-only like the rest of
    /// the configuration, and refused otherwise.
    static bool interrupts(uint32_t mask, bool on) {
        if (enabled() || (mask & ~LptimFlag::all) != 0u) {
            return false;
        }
        if constexpr (!has_encoder) {
            if ((mask & LptimFlag::encoder_only) != 0u) {
                return false;
            }
        }
        regs().IER = on ? (regs().IER | mask) : (regs().IER & ~mask);
        return true;
    }
    static uint32_t interrupts() { return regs().IER; }

    // ---- enable and start (26.4.7, 26.4.8, 26.4.13) -------------------------

    /// Set CR.ENABLE. 26.4.13: the block is not really enabled until TWO
    /// COUNTER CLOCKS later, which at 32768 Hz is 61 us and at PCLK is
    /// nothing - the caller (or the task) spends that wait before
    /// starting. There is no matching disable: fact 1.
    static void enable() { regs().CR = regs().CR | LPTIM_CR_ENABLE; }
    static bool enabled() { return (regs().CR & LPTIM_CR_ENABLE) != 0u; }

    /**
     * CNTSTRT: run for ever (26.4.8). With TRIGEN = 00 the counter
     * starts here, three kernel clocks later (26.4.7's second note);
     * with an external trigger selected this ARMS it and the trigger
     * starts it.
     *
     * Refused while the block is disabled - 26.4.7: "any write on these
     * bits when the timer is disabled will be discarded by hardware", so
     * a store that would vanish is a false here instead.
     *
     * It is also the ON-THE-FLY switch out of one-shot mode: 26.4.8 says
     * a CNTSTRT written during a single-pulse count makes the counter
     * restart at the next ARR match instead of stopping.
     */
    static bool start_continuous() {
        if (!enabled()) {
            return false;
        }
        regs().CR = regs().CR | LPTIM_CR_CNTSTRT;
        return true;
    }

    /// SNGSTRT: run once, stop at ARR. The mirror of the above, and the
    /// on-the-fly switch the other way: written during a continuous
    /// count it makes the counter stop at the next ARR match.
    static bool start_single() {
        if (!enabled()) {
            return false;
        }
        regs().CR = regs().CR | LPTIM_CR_SNGSTRT;
        return true;
    }

    /// Is a count in progress? There is no register bit for it - CNTSTRT
    /// and SNGSTRT are cleared by hardware as soon as they are taken -
    /// so this is deliberately NOT offered. What a caller can ask is
    /// whether the counter is MOVING, which is two reads of count().

    // ---- the compare and the auto-reload (26.4.11, 26.7.6, 26.7.7) ----------
    //
    // Fact 4: the store and the observation are separate verbs, because
    // the completion is what the CALLER must not write over.

    /// Store CMP. Refused while the block is DISABLED (26.7.6's Caution
    /// is the opposite way round from CFGR's). Does not clear CMPOK and
    /// does not wait: 26.4.11 makes that the caller's handshake.
    static bool set_cmp(uint16_t v) {
        if (!enabled()) {
            return false;
        }
        regs().CMP = v;
        return true;
    }
    static uint16_t cmp() { return static_cast<uint16_t>(regs().CMP); }

    /// Store ARR. Same rule, same caveat. 26.4.10: for a waveform ARR
    /// "must be strictly greater than" CMP - a rule about the two
    /// registers TOGETHER, so it is the task's and not this verb's.
    static bool set_arr(uint16_t v) {
        if (!enabled()) {
            return false;
        }
        regs().ARR = v;
        return true;
    }
    static uint16_t arr() { return static_cast<uint16_t>(regs().ARR); }

    static bool cmp_ok() { return (regs().ISR & LptimFlag::cmpok) != 0u; }
    static bool arr_ok() { return (regs().ISR & LptimFlag::arrok) != 0u; }

    /// Bounded poll for CMPOK. Returns the flag's state - true when the
    /// write landed, false when it never did inside the bound - and does
    /// NOT clear it: with an interrupt enabled only the handler may
    /// (fact 2), and with none enabled the caller clears it itself.
    static bool wait_cmp_ok(uint32_t spins = write_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (cmp_ok()) {
                return true;
            }
        }
        return cmp_ok();
    }
    static bool wait_arr_ok(uint32_t spins = write_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (arr_ok()) {
                return true;
            }
        }
        return arr_ok();
    }

    // ---- the counter (26.4.14, 26.7.8) --------------------------------------

    /// TWO CONSECUTIVE READS THAT AGREE (fact 5), bounded. Nothing when
    /// they never agreed - which on an asynchronous kernel clock is a
    /// real possibility and not a defensive nicety.
    static std::optional<uint16_t> count(uint32_t spins = read_spins) {
        uint16_t a = static_cast<uint16_t>(regs().CNT);
        for (uint32_t i = 0; i < spins; ++i) {
            const uint16_t b = static_cast<uint16_t>(regs().CNT);
            if (a == b) {
                return a;
            }
            a = b;
        }
        return std::nullopt;
    }

    /// ONE read. The verb for the RSTARE case, where 26.4.14 says the
    /// double read is impossible because the first read is the reset -
    /// and for a counter clocked from PCLK, where there is no
    /// asynchronous domain to cross.
    static uint16_t count_raw() { return static_cast<uint16_t>(regs().CNT); }

    /**
     * COUNTRST: the SYNCHRONOUS reset (26.4.14). Refused while disabled
     * (the bit "can be set only when the LPTIM is enabled") and refused
     * while it still reads set - 26.7.5's Caution: "COUNTRST must never
     * be set to 1 by software before it is already cleared to 0 by
     * hardware", which is a rule with no hardware behind it and
     * therefore has to be this verb's.
     *
     * It costs three kernel clocks to cross into the counter's domain,
     * so the counter takes A FEW MORE PULSES before it lands - the
     * chapter says so and the suite counts them.
     */
    static bool reset_count() {
        if (!enabled() || (regs().CR & LPTIM_CR_COUNTRST) != 0u) {
            return false;
        }
        regs().CR = regs().CR | LPTIM_CR_COUNTRST;
        return true;
    }
    static bool count_reset_pending() { return (regs().CR & LPTIM_CR_COUNTRST) != 0u; }

    /**
     * RSTARE: the ASYNCHRONOUS reset - every read of CNT zeroes it.
     * Enabled-only, like its synchronous twin.
     *
     * THE TWO MECHANISMS ARE EXCLUSIVE and nothing in the silicon says
     * so: 26.4.14's Warning - "there is no mechanism inside the LPTIM
     * that prevents the two reset mechanisms from being used
     * simultaneously" - makes it the caller's rule. And with this bit
     * set, count()'s double read is impossible by construction, so
     * count_raw() is the only legal reader.
     */
    static bool reset_on_read(bool on) {
        if (!enabled()) {
            return false;
        }
        regs().CR = on ? (regs().CR | LPTIM_CR_RSTARE) : (regs().CR & ~LPTIM_CR_RSTARE);
        return true;
    }
    static bool reset_on_read() { return (regs().CR & LPTIM_CR_RSTARE) != 0u; }

    // ---- flags (26.7.1, 26.7.2) and ES0548 2.8.2 ----------------------------

    static uint32_t status() { return regs().ISR; }

    /**
     * Clear flags - AND THIS IS THE ERRATUM AS CODE (fact 2, ES0548
     * 2.8.2).
     *
     * With ANY interrupt enabled in IER, a flag may be cleared only
     * inside the interrupt routine, so this verb REFUSES (false, nothing
     * written) when IER is nonzero and the core is in thread mode. The
     * test is `__get_IPSR() != 0`, one instruction.
     *
     * With IER == 0 the erratum's own precondition ("the LPTIM is
     * configured in interrupt mode - at least one interrupt is enabled")
     * is absent, and this is an ordinary W1C register again: that is
     * what makes the polling users of this driver - LptimPwm's compare
     * handshake above all - legal rather than lucky.
     */
    static bool clear_flags(uint32_t mask) {
        if (regs().IER != 0u && __get_IPSR() == 0u) {
            return false;
        }
        regs().ICR = mask & LptimFlag::all;
        return true;
    }

    /// The unguarded store: the ISR body's own verb, and the bench's
    /// staging one. Named for what it is so that a caller reaching for
    /// it knows it is stepping past 2.8.2's workaround on purpose.
    [[gnu::always_inline]] static void clear_flags_raw(uint32_t mask) {
        regs().ICR = mask & LptimFlag::all;
    }

    /**
     * The ISR body an application binds to this instance's vector.
     * Returns the mask it SERVED - 0 when nothing of this block's was
     * pending, which is how a handler on a shared vector answers only
     * for itself (the file header's shared-vector contract).
     *
     * The clear is ES0548 2.8.2's ORDER, not a single store: the pending
     * flags whose interrupt is DISABLED go first, then those whose
     * interrupt is enabled. Only the second group is returned, because
     * only that group is what the interrupt was for; the first is swept
     * because the erratum says it must be swept here and nowhere else.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        LPTIM_TypeDef& r = regs();
        const uint32_t ier = r.IER;
        const uint32_t pending = r.ISR & LptimFlag::all;
        const uint32_t unarmed = pending & ~ier;
        const uint32_t armed = pending & ier;
        if (unarmed != 0u) {
            r.ICR = unarmed;
        }
        if (armed != 0u) {
            r.ICR = armed;
        }
        return armed;
    }

    // ---- the wake line (table 65) -------------------------------------------

    /// EXTI line 29 (LPTIM1) or 30 (LPTIM2): a DIRECT line, so there is
    /// no trigger to select and no pending bit to clear - but its IMR
    /// bit must stand or the interrupt does not bring the core out of a
    /// Stop. Idempotent (rtc.hpp's wake_line_open() precedent).
    static bool wake_line(bool on) { return Exti::interrupt(exti_line, on); }
    static bool wake_line() { return Exti::interrupt(exti_line); }

    /// What "pending" means for a direct line: this peripheral's own
    /// flags. The EXTI has none for line 29 or 30.
    static uint32_t pending_wake() { return status() & regs().IER; }

    // ---- debug (26.4.16) ----------------------------------------------------

    /// DBG_APB_FZ1.DBG_LPTIMn_STOP: whether the counter freezes while a
    /// debugger holds the core. Two things this register needs, both
    /// learned in reset.hpp: it answers a store only with
    /// RCC_APBENR1.DBGEN set - which is CLEAR at reset - so this verb
    /// opens that gate itself; and 40.10.3 says it is not reset by a
    /// system reset, so whatever a debug session left is still there.
    static void debug_freeze(bool on) {
        Rcc::apb1_clock(RCC_APBENR1_DBGEN, true);
        const uint32_t bit = n == 1u ? DBG_APB_FZ1_DBG_LPTIM1_STOP
                                     : DBG_APB_FZ1_DBG_LPTIM2_STOP;
        DBG->APBFZ1 = on ? (DBG->APBFZ1 | bit) : (DBG->APBFZ1 & ~bit);
    }
    static bool debug_freeze() {
        const uint32_t bit = n == 1u ? DBG_APB_FZ1_DBG_LPTIM1_STOP
                                     : DBG_APB_FZ1_DBG_LPTIM2_STOP;
        return (DBG->APBFZ1 & bit) != 0u;
    }

    /// How many spins a bounded wait may cost. Sized for the worst
    /// case this block has: a CMP write crossing into a 32 kHz kernel
    /// clock from a 64 MHz core is a few thousand cycles, and a wait
    /// that gives up is reported rather than hidden.
    static constexpr uint32_t write_spins = 200'000;
    static constexpr uint32_t read_spins = 64;
};

// =============================================================================
// Tasks
// =============================================================================

/**
 * LptimPwm<L, top>: one PWM output on LPTIMx_OUT, and util/pwm_channel.hpp's
 * PwmChannel on its FOURTH implementation (the AVR's TCA, the SAM's TC
 * and TCC, this).
 *
 *   using Lamp = brio::LptimPwm<brio::Lptim<1>, 1000>;
 *   Lamp::setup(brio::LptimPrescaler::div1);
 *   Lamp::duty(250);            // a quarter
 *
 * THE ARITHMETIC, MEASURED (test_stm32_lptim letter c) because 26.4.10
 * describes the waveform in words and prints no formula:
 *
 *   period    = ARR + 1 counter ticks (the counter runs 0..ARR);
 *   high time = ARR - CMP + 1 counter ticks - ONE MORE than the
 *               chapter's two sentences ("set as soon as the counter
 *               EXCEEDS the compare", "reset as soon as a match occurs
 *               between ARR and CNT") read on their own, which is why it
 *               was measured and not derived;
 *   EXCEPT at CMP >= ARR, the pair 26.4.10 forbids, where the output is
 *               a FLAT LOW and not the one tick the formula would give.
 *
 * So this task maps duty v out of `max` = ARR onto CMP = max - v, and
 * BOTH ENDPOINTS COME OUT EXACT BY TWO DIFFERENT ROUTES: v = max puts
 * CMP at 0 and the output high for the whole period (1000 per mille
 * measured), while v = 0 puts CMP on ARR and lands on the forbidden
 * pair, whose flat low IS zero duty (0 per mille measured). In between
 * the duty is (v + 1) / (max + 1), one tick generous - the mapping says
 * 251, 501 and 751 per mille for v = 250, 500 and 750 out of 999 and the
 * pad reads 251, 502 and 752, the last per mille being the sampler's own
 * (test_stm32_lptim letter c counts pad levels; it does not count
 * ticks).
 *
 * PRELOAD IS SET. A PwmChannel::duty() that can cut the pulse being
 * produced is not one a generic actuator can use (the tim.hpp ruling),
 * so 26.4.11's end-of-period update is on and a duty change lands at the
 * next period boundary.
 *
 * NO INTERRUPT IS ENABLED, and that is load-bearing rather than
 * economical: with IER == 0, ES0548 2.8.2's precondition is absent and
 * duty()'s CMPOK handshake - wait for the previous write, clear the
 * flag, store the next - is legal from thread mode. A caller that arms
 * an interrupt on this instance takes that handshake away.
 */
template <class L, uint16_t top = 0xFFFF>
struct LptimPwm {
    LptimPwm() = delete;
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    /**
     * Bring the timer up as a continuous PWM generator. The order is the
     * chapter's: configure while disabled, enable, spend the enable
     * latency, write ARR and wait for ARROK, write CMP and wait for
     * CMPOK, then start.
     *
     * False when any of those steps was refused or never completed.
     */
    static bool setup(LptimPrescaler prescaler = LptimPrescaler::div1,
                      bool inverted = false) {
        if (!L::configure({.prescaler = prescaler,
                           .waveform = LptimWaveform::pwm_or_pulse,
                           .output_inverted = inverted,
                           .preload = true})) {
            return false;
        }
        L::enable();
        if (!L::set_arr(top)) {
            return false;
        }
        if (!L::wait_arr_ok()) {
            return false;
        }
        (void)L::clear_flags(LptimFlag::arrok);
        if (!L::set_cmp(top)) {   // duty 0
            return false;
        }
        if (!L::wait_cmp_ok()) {
            return false;
        }
        return L::start_continuous();
    }

    /// The duty, in counts out of `max`. Spends 26.4.11's handshake: the
    /// PREVIOUS write must have completed before this one is stored, or
    /// the chapter promises "unpredictable results".
    static void duty(uint16_t v) {
        (void)L::wait_cmp_ok();
        (void)L::clear_flags(LptimFlag::cmpok);
        const uint16_t compare = v >= max ? 0u : static_cast<uint16_t>(max - v);
        (void)L::set_cmp(compare);
    }
    static uint16_t duty() {
        const uint16_t compare = L::cmp();
        return compare >= max ? 0u : static_cast<uint16_t>(max - compare);
    }
};

static_assert(PwmChannel<LptimPwm<Lptim<1>, 1000>>,
              "brio LptimPwm must satisfy util/pwm_channel.hpp's PwmChannel");

/**
 * LptimPeriodicTick<L>: an ARRM interrupt every `period + 1` counter
 * ticks - and, on LSI or LSE, THE ONE PERIODIC INTERRUPT OF THIS FAMILY
 * THAT SURVIVES A STOP without owning the RTC (26.5, table 145).
 *
 *   using Beat = brio::LptimPeriodicTick<brio::Lptim<1>>;
 *   brio::Lptim<1>::init();
 *   brio::Lptim<1>::kernel_clock(brio::LptimClock::lse);
 *   Beat::setup(brio::LptimPrescaler::div32, 1023);   // 1 Hz off a crystal
 *
 * `setup()` is a SETUP-TIME verb: it writes ARR before any interrupt is
 * armed... it cannot be, since IER is disabled-only and therefore
 * written by the same configure() - so the ARROK handshake here is the
 * FIRST one after a reset, where the flag starts clear and nothing has
 * to be cleared. A later period change is `set_period()`, which says in
 * its own comment what it can and cannot observe.
 */
template <class L>
struct LptimPeriodicTick {
    LptimPeriodicTick() = delete;

    static constexpr uint32_t flag = LptimFlag::arrm;

    static bool setup(LptimPrescaler prescaler, uint16_t period,
                      bool interrupt = true) {
        if (!L::configure({.prescaler = prescaler,
                           .interrupts = interrupt ? LptimFlag::arrm : 0u})) {
            return false;
        }
        L::enable();
        if (!L::set_arr(period)) {
            return false;
        }
        return L::wait_arr_ok() && L::start_continuous();
    }

    /**
     * Change the period of a running tick. THE HANDSHAKE IS ONLY HALF
     * OBSERVABLE here and the caller must know it: with ARRMIE enabled,
     * ES0548 2.8.2 forbids clearing ARROK outside the handler, so a
     * standing ARROK from the previous write cannot be cleared by this
     * verb and `wait_arr_ok()` would answer for that old write. So this
     * stores and returns whether the store was ACCEPTED, not whether it
     * COMPLETED - and 26.4.11 makes spacing two such writes the caller's
     * job. An application that changes the period often should clear
     * ARROK in its own handler, which is where the erratum allows it.
     */
    static bool set_period(uint16_t period) { return L::set_arr(period); }

    /// Stop the tick. Fact 1: this is an RCC reset, so everything above
    /// has to be set up again afterwards.
    static void stop() { L::disable(); }
};

/**
 * LptimCounter<L>: the free-running 32-bit count - a 16-bit counter at
 * ARR = 0xFFFF plus a high word the ARRM interrupt carries.
 *
 * This is what a timebase across a Stop is built on (stm32g0/sleep.hpp's
 * third site), and the read is the classic two-word one: read high, read
 * CNT, read high again; if the high word moved between the two reads,
 * take the NEW high word with a FRESH counter reading - which cannot
 * have wrapped again in the few cycles that separate them.
 *
 * The app binds the instance's vector to `isr()`, not to `L::isr()`:
 * this body is the resource's plus the accumulation.
 */
template <class L>
struct LptimCounter {
    LptimCounter() = delete;

    /// Continuous counting at ARR = 0xFFFF with ARRM armed. False when
    /// any step was refused.
    static bool setup(LptimPrescaler prescaler = LptimPrescaler::div1) {
        high_ = 0;
        if (!L::configure({.prescaler = prescaler,
                           .interrupts = LptimFlag::arrm})) {
            return false;
        }
        L::enable();
        if (!L::set_arr(0xFFFFu)) {
            return false;
        }
        return L::wait_arr_ok() && L::start_continuous();
    }

    /// The counter, 32 bits wide. std::nullopt when the underlying
    /// double read never agreed (fact 5).
    static std::optional<uint32_t> count32() {
        for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
            const uint32_t hi = high_;
            const std::optional<uint16_t> low = L::count();
            if (!low.has_value()) {
                return std::nullopt;
            }
            if (high_ == hi) {
                return (hi << 16) | *low;
            }
        }
        return std::nullopt;
    }

    /// The 16-bit half alone - what a span shorter than one lap needs,
    /// and the reading that costs no interrupt at all.
    static std::optional<uint16_t> count() { return L::count(); }

    static uint32_t laps() { return high_; }

    /// Put the lap accumulator back to zero. `setup()` does it; this is
    /// the verb for a task that configures the block ITSELF and only
    /// borrows the counting half (LptimPulseCounter does exactly that),
    /// and for a caller restarting a count without a reconfiguration.
    static void reset_laps() { high_ = 0; }

    /// The ISR body: the resource's ordered clear, plus the lap
    /// accumulation. Returns what the resource served.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t served = L::isr();
        if ((served & LptimFlag::arrm) != 0u) {
            high_ = high_ + 1u;
        }
        return served;
    }

private:
    static inline volatile uint32_t high_ = 0;
};

/**
 * LptimPulseCounter<L>: 26.1's "pulse counter" - the counter clocked BY
 * ITS OWN INPUT, with no oscillator running at all.
 *
 * Two arrangements, and they are not the same thing:
 *
 *  - `setup_external()` is CKSEL = 1: IN1 is the LPTIM's clock, no
 *    internal clock is needed (except by the glitch filters), and this
 *    is the configuration that counts through a Stop with every
 *    oscillator off. Its price is 26.4.12's own sentence: "the first
 *    five active edges on the LPTIM external Input1 (after LPTIM is
 *    enable) are lost", because the input is also what clocks the
 *    block's own logic. The suite counts them.
 *  - `setup_sampled()` is CKSEL = 0 with COUNTMODE = 1: the internal
 *    clock samples IN1, so nothing is lost at the start, both edges
 *    become legal, and the input must be slower than the kernel clock
 *    and the prescaler must be /1.
 *
 * The 32-bit count is LptimCounter's, and so is the vector body.
 */
template <class L>
struct LptimPulseCounter {
    LptimPulseCounter() = delete;

    using Counter = LptimCounter<L>;

    /// The input IS the clock. `polarity` may not be `both` here
    /// (26.4.12) and lptim_config_valid() refuses it.
    static bool setup_external(LptimClockPolarity polarity = LptimClockPolarity::rising,
                               LptimInput1 input = LptimInput1::pad,
                               LptimFilter filter = LptimFilter::none) {
        high_reset();
        if (!L::configure({.clock = LptimClockSource::external_input1,
                           .clock_polarity = polarity,
                           .clock_filter = filter,
                           .input1 = input,
                           .interrupts = LptimFlag::arrm})) {
            return false;
        }
        return start();
    }

    /// The internal clock samples the input. The prescaler is /1 by the
    /// chapter's rule, so it is not an argument.
    static bool setup_sampled(LptimClockPolarity polarity = LptimClockPolarity::rising,
                              LptimInput1 input = LptimInput1::pad,
                              LptimFilter filter = LptimFilter::none) {
        high_reset();
        if (!L::configure({.count_external = true,
                           .clock_polarity = polarity,
                           .clock_filter = filter,
                           .input1 = input,
                           .interrupts = LptimFlag::arrm})) {
            return false;
        }
        return start();
    }

    static std::optional<uint32_t> count32() { return Counter::count32(); }
    static std::optional<uint16_t> count() { return Counter::count(); }
    [[gnu::always_inline]] static uint32_t isr() { return Counter::isr(); }

    /// 26.4.12's own number, stated so an application can subtract it:
    /// the edges lost between enabling an externally clocked LPTIM and
    /// the first count. The bench measures whether it is really five.
    static constexpr uint8_t documented_lost_edges = 5;

private:
    static void high_reset() { Counter::reset_laps(); }
    static bool start() {
        L::enable();
        if (!L::set_arr(0xFFFFu)) {
            return false;
        }
        return L::wait_arr_ok() && L::start_continuous();
    }
};

/**
 * LptimTimeout<L>: 26.4.9's timeout function - "the first trigger event
 * will start the timer, any successive trigger event will reset the
 * counter and the timer will restart", so a CMPM means NO TRIGGER
 * ARRIVED within the compare value.
 *
 * THE FEED IS THE TRIGGER, AND ONLY THE TRIGGER. A software start
 * (TRIGEN = 00) cannot feed a timeout: 26.4.9's mechanism is the trigger
 * multiplexer's, so `setup()` refuses a software trigger edge and the
 * thing that keeps the timeout alive is whatever table 138/139 row the
 * caller named - an ETR pad, an RTC alarm, a comparator.
 */
template <class L>
struct LptimTimeout {
    LptimTimeout() = delete;

    static constexpr uint32_t flag = LptimFlag::cmpm;

    /**
     * Arm a timeout of `counts` counter ticks. The trigger both starts
     * and feeds it; the CMPM interrupt is the expiry.
     *
     * False for a software trigger edge (there is nothing to feed it
     * with), for a counts value that leaves no room under ARR - 26.4.10
     * wants ARR strictly greater than CMP - or for any refused step.
     */
    static bool setup(LptimTrigger trigger, LptimTriggerEdge edge, uint16_t counts,
                      LptimPrescaler prescaler = LptimPrescaler::div1,
                      LptimFilter trigger_filter = LptimFilter::none) {
        if (edge == LptimTriggerEdge::software || counts == 0xFFFFu) {
            return false;
        }
        if (!L::configure({.prescaler = prescaler,
                           .trigger_filter = trigger_filter,
                           .trigger = trigger,
                           .trigger_edge = edge,
                           .timeout = true,
                           .interrupts = LptimFlag::cmpm})) {
            return false;
        }
        L::enable();
        if (!L::set_arr(0xFFFFu)) {
            return false;
        }
        if (!L::wait_arr_ok()) {
            return false;
        }
        if (!L::set_cmp(counts)) {
            return false;
        }
        return L::wait_cmp_ok() && L::start_continuous();
    }

    /// Has the timeout expired since the last look? The flag is the
    /// peripheral's; with CMPMIE armed only the handler may clear it
    /// (ES0548 2.8.2), so this is a read and the clear belongs to isr().
    static bool expired() { return (L::status() & LptimFlag::cmpm) != 0u; }

    [[gnu::always_inline]] static uint32_t isr() { return L::isr(); }

    static void stop() { L::disable(); }
};

/**
 * LptimEncoder<L>: 26.4.15's quadrature interface - LPTIM1's alone
 * (table 135), because it is the instance with a second input.
 *
 * The counter runs between 0 and ARR and its content IS the position;
 * the UP and DOWN flags of LPTIM_ISR report a change of DIRECTION, not
 * a direction, which is 26.4.15's own wording and the thing that
 * surprises. The three sub-modes are CKPOL's three legal codes and their
 * counting scenarios are table 144's.
 *
 * The chapter's own Caution is enforced by lptim_config_valid(): an
 * internal clock and PRESC = /1, or nothing.
 */
template <class L>
struct LptimEncoder {
    LptimEncoder() = delete;
    static_assert(L::has_encoder,
                  "brio LptimEncoder: this instance has no encoder mode - table 135 "
                  "gives it to LPTIM1 alone, and LPTIM2 has only one input channel "
                  "anyway (figure 271's footnote)");

    /// `sub_mode` is CKPOL: rising = sub-mode 1, falling = sub-mode 2,
    /// both = sub-mode 3 (26.7.4's CKPOL description).
    static bool setup(uint16_t period,
                      LptimClockPolarity sub_mode = LptimClockPolarity::rising,
                      LptimInput1 input1 = LptimInput1::pad,
                      LptimInput2 input2 = LptimInput2::pad,
                      bool direction_interrupt = false) {
        if (!L::configure({.clock_polarity = sub_mode,
                           .encoder = true,
                           .input1 = input1,
                           .input2 = input2,
                           .interrupts = direction_interrupt
                                             ? LptimFlag::encoder_only
                                             : 0u})) {
            return false;
        }
        L::enable();
        if (!L::set_arr(period)) {
            return false;
        }
        // 26.4.15: "LPTIM_ARR must be configured before starting the
        // counter", and continuous mode is the only one encoder mode
        // runs in.
        return L::wait_arr_ok() && L::start_continuous();
    }

    /// The position - the counter, read the reliable way.
    static std::optional<uint16_t> position() { return L::count(); }

    /// The two direction-change flags, as read. Clearing them is
    /// clear_flags()' business and, with the interrupt armed, the
    /// handler's (ES0548 2.8.2).
    static bool went_up() { return (L::status() & LptimFlag::up) != 0u; }
    static bool went_down() { return (L::status() & LptimFlag::down) != 0u; }

    [[gnu::always_inline]] static uint32_t isr() { return L::isr(); }

    static void stop() { L::disable(); }
};

} // namespace brio
