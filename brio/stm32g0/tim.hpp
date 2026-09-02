/*
 * tim.hpp
 *
 * The STM32G0's timers (RM0444 ch. 21..25) in the two strata every brio
 * target uses (docs/design/overview.md, "Target strata"):
 *
 *  Tim<n>          the RESOURCE - one TIMx block: the time base
 *                  (prescaler, counter, auto-reload with its preload),
 *                  the counting modes, the capture/compare channels in
 *                  both their faces, the slave controller and the master
 *                  TRGO, the break/dead-time unit, the input
 *                  multiplexer, the flags and the ISR body.
 *
 *  TimPad<sel>     a pad handed to a timer channel (the AF number is the
 *                  DATASHEET's - see below).
 *
 *  TASKS           TimPwm / TimPairPwm (util/pwm_channel.hpp's
 *                  PwmChannel, one and two outputs), TimPeriodMeter /
 *                  TimIntervalMeter (what util/meter_sampler.hpp's
 *                  MeterLatch is fed from), TimEventCounter /
 *                  TimGatedCounter (one timer measuring another with no
 *                  wire and no CPU), TimPeriodicTick, TimOnePulse.
 *
 * FIVE FACTS THAT SHAPE THIS FILE.
 *
 * 1. THERE IS ONE TIM_TypeDef AND TEN DIFFERENT TIMERS. The device
 *    header declares every register as a member of one struct, so
 *    `TIM14->SMCR` compiles and writes a hole in the address map: the
 *    silicon's TIM14 has no slave controller (24.4), TIM2/3/4 have no
 *    break unit (22.4), TIM6/7 have no channel at all (23.4). NOTHING IN
 *    THE HEADER SAYS SO. What a timer IS - counter width, channels,
 *    complementary outputs, slave controller, master mode, BDTR, RCR,
 *    centre-aligned counting, TISEL, ETR - therefore comes from
 *    stm32g0/device_tables.hpp, where those facts are named as what they
 *    are: the DOCUMENTS' (DS13560 table 7 and each chapter's ".2 main
 *    features"), keyed by instance, and reached only for an instance the
 *    header says exists. Every verb that touches a register an instance
 *    does not implement REFUSES - returns false and writes nothing -
 *    rather than storing into a hole, and the tasks that need a feature
 *    static_assert on it, which is what makes a dead-time request on a
 *    TIM3 a compile error instead of a silent nothing.
 *
 * 2. THE STATUS REGISTER IS rc_w0, NOT W1C. A flag of TIMx_SR is cleared
 *    by writing ZERO to it and a write of one has no effect (21.4.5), so
 *    clearing is `SR = ~flags` - a plain store, no read-modify-write, and
 *    a flag that arrives between the read and the store SURVIVES. Both
 *    other brio targets clear their flags by writing ONES (the AVR's
 *    INTFLAGS, the SAM's INTFLAG, this family's own EXTI_RPR1), so this
 *    is the one register in the stratum where the reflex is wrong.
 *
 * 3. THE PRESCALER AND THE AUTO-RELOAD ARE SHADOWED. PSC is copied into
 *    the working register at the next UPDATE event and never before
 *    (21.4.11); ARR is too when CR1.ARPE is set, and is taken at once
 *    when it is clear. So a configuration is only in force after an
 *    update, which is why configure() ends with EGR.UG - a software
 *    update that loads both shadows - and then clears the UIF that
 *    update raised. A caller that changes the period of a RUNNING timer
 *    and wants it now uses the same verb; one that wants the change at
 *    the end of the current period sets ARPE and writes ARR.
 *
 * 4. A COMPARE REGISTER IS PRELOADED BY DEFAULT HERE, AND THAT IS A
 *    CHOICE. CCyPE is clear out of reset (a write to CCRy acts at once,
 *    which can produce a runt pulse if it lands past the current count);
 *    every output channel this driver configures sets it, because a
 *    PwmChannel::duty() that can glitch is not one an actuator above can
 *    use. 21.3.10's "PWM mode" recipe asks for it too. The unbuffered
 *    behaviour is still reachable - TimChannelConfig::preload = false.
 *
 * 5. THE PAD MAP IS THE DATASHEET'S AND NOTHING CHECKS IT. Which AF
 *    number carries TIM2_CH1 on PA5 is DS13560 table 13 (AF2), and the
 *    device header has no symbol for it - stm32g0/pin.hpp states that
 *    once for the whole stratum. So a timer pad is a `PinSel` the CALLER
 *    writes, `TimPad<sel>` is the claim, and the bench is the only check
 *    there is. Three of those claims are measured in test_stm32_tim.
 *
 * THE CLOCK. TIMPCLK is PCLK when the APB prescaler is 1 and twice PCLK
 * otherwise (5.2.13); stm32g0/clock.hpp's `Clock<>` pins that prescaler
 * at 1, so TIMPCLK == pclk_hz == hz and `tim_clock_hz(clock)` is that
 * one number. `Tim<n>::clock_ok()` asks the silicon whether the
 * assumption still holds, so a future task that divides the buses cannot
 * make this file lie silently. TIM1 and TIM15 can additionally be fed
 * from PLLQCLK (5.2.13, RCC_CCIPR.TIM1SEL/TIM15SEL) - NOT built, because
 * this stratum's PLL configuration drives the R output only.
 *
 * ERRATA, ES0548 Rev 3 read on the bench chip's revision Z column - all
 * three TIM items apply:
 *  - 2.7.1 one-pulse mode trigger not detected in master-slave reset +
 *    trigger with MSM set. Its own workaround is "keep MSM reset", so
 *    TimSlaveConfig::master_slave DEFAULTS FALSE and the caller that
 *    sets it is told what it costs; the driver cannot refuse it, since
 *    cycle-accurate cascading is a legitimate thing to want.
 *  - 2.7.2 consecutive compare event missed when CCR changes between two
 *    adjacent counter cycles - no workaround, and it is STAGED at the
 *    bench (test_stm32_tim letter k) rather than described.
 *  - 2.7.3 output-compare clear (ocref_clr) with an external counter
 *    reset. This driver exposes OCyCE and the OCCS selection, so the
 *    combination is reachable; the obligation is stated on the verb.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// CR1.DIR - which way an edge-aligned counter runs (21.4.1). Only
/// TIM1..TIM4 have the bit; every other timer of this family counts up
/// and the driver refuses to claim otherwise.
enum class TimDirection : uint8_t { up = 0, down = 1 };

/// CR1.CMS - edge-aligned, or centre-aligned with the compare flag set
/// while counting down, up, or both (21.4.1). The three centre modes
/// differ ONLY in when CCyIF is raised; the waveform is the same.
enum class TimAlignment : uint8_t {
    edge = 0,
    center_down = 1,   ///< CCyIF set while down-counting
    center_up = 2,     ///< CCyIF set while up-counting
    center_both = 3,
};

/// CR1.CKD - the ratio between the timer clock and tDTS, the sampling
/// clock of the digital filters and the unit of the dead-time generator
/// (21.4.1). Code 3 is reserved.
enum class TimClockDivision : uint8_t { div1 = 0, div2 = 1, div4 = 2 };

/// SMCR.TS - what the slave controller listens to (21.4.3). Which timer
/// ITR0..ITR3 mean is per instance: tables 119 (TIM1), 123 (TIM2/3/4) and
/// 130 (TIM15), reproduced by `tim_internal_trigger()` below.
enum class TimTrigger : uint8_t {
    itr0 = 0, itr1 = 1, itr2 = 2, itr3 = 3,
    ti1_edge = 4,    ///< TI1F_ED: one pulse per TRANSITION of TI1 (never gate on it)
    ti1 = 5,         ///< TI1FP1
    ti2 = 6,         ///< TI2FP2
    etr = 7,         ///< ETRF, the external trigger input
};

/// SMCR.SMS - what the trigger does to the counter (21.4.3). Codes above
/// 1000 are reserved on this family.
enum class TimSlaveMode : uint8_t {
    disabled = 0,
    encoder1 = 1,
    encoder2 = 2,
    encoder3 = 3,
    reset = 4,               ///< a trigger edge reinitializes the counter
    gated = 5,               ///< the counter runs while the trigger is HIGH
    trigger = 6,             ///< a trigger edge starts the counter (does not reset it)
    external_clock1 = 7,     ///< the trigger's rising edges ARE the counter clock
    combined_reset_trigger = 8,
};

/// CR2.MMS - what this timer publishes on TRGO (21.4.2).
enum class TimMasterMode : uint8_t {
    reset = 0,          ///< EGR.UG
    enable = 1,         ///< the counter-enable signal
    update = 2,         ///< the update event - the frequency divider a slave counts
    compare_pulse = 3,  ///< a pulse when a capture or a compare 1 happened
    oc1ref = 4,         ///< the channel-1 waveform ITSELF, high time and all
    oc2ref = 5,
    oc3ref = 6,
    oc4ref = 7,
};

/// CCMRx.OCyM - what a compare match does to the output (21.4.7). The
/// field is four bits, split by the silicon across bit 16 and bits 6:4.
enum class TimOutputMode : uint8_t {
    frozen = 0,
    active_on_match = 1,
    inactive_on_match = 2,
    toggle = 3,
    force_inactive = 4,
    force_active = 5,
    pwm1 = 6,               ///< active while CNT < CCRy
    pwm2 = 7,               ///< the complement of pwm1
    retriggerable_opm1 = 8,
    retriggerable_opm2 = 9,
    combined_pwm1 = 12,
    combined_pwm2 = 13,
    asymmetric_pwm1 = 14,
    asymmetric_pwm2 = 15,
};

/// CCMRx.CCyS - a channel is an OUTPUT or an input mapped on one of
/// three sources (21.4.7). `direct` is this channel's own TI, `indirect`
/// its neighbour's - the pair that makes PWM input mode.
enum class TimChannelSelect : uint8_t {
    output = 0,
    direct = 1,
    indirect = 2,
    trc = 3,     ///< the TRC signal (needs the slave controller)
};

/// CCER.CCyP/CCyNP for an INPUT channel: 00 rising, 01 falling, 11 both
/// (21.4.9; 10 is reserved).
enum class TimCapturePolarity : uint8_t { rising = 0, falling = 1, both = 3 };

/// CCMRx.ICyPSC - capture one edge in 1, 2, 4 or 8 (21.4.7).
enum class TimCapturePrescaler : uint8_t { every = 0, every2 = 1, every4 = 2, every8 = 3 };

/// The time base. `period` is ARR and is REFUSED at zero: 21.4.12 says a
/// null auto-reload leaves the counter blocked, which is a stopped timer
/// wearing the face of a running one.
struct TimConfig {
    uint16_t prescaler = 0;                ///< PSC: the counter clock is TIMPCLK / (PSC + 1)
    uint32_t period = 0xFFFF;              ///< ARR (32-bit on TIM2 only)
    TimDirection direction = TimDirection::up;
    TimAlignment alignment = TimAlignment::edge;
    TimClockDivision clock_division = TimClockDivision::div1;
    bool auto_reload_preload = false;      ///< CR1.ARPE
    bool one_pulse = false;                ///< CR1.OPM: stop at the next update
    bool update_disable = false;           ///< CR1.UDIS
    bool update_on_overflow_only = false;  ///< CR1.URS: EGR.UG and a slave reset raise no UIF
    bool uif_remap = false;                ///< CR1.UIFREMAP: UIF copied into CNT bit 31
    uint8_t repetition = 0;                ///< RCR (break-capable timers only)
};

/// One capture/compare channel as an OUTPUT.
struct TimChannelConfig {
    TimOutputMode mode = TimOutputMode::frozen;
    uint32_t compare = 0;                  ///< CCRy
    bool preload = true;                   ///< CCMRx.OCyPE - see fact 4 in the file header
    bool fast = false;                     ///< CCMRx.OCyFE
    bool clear_on_ocref_clr = false;       ///< CCMRx.OCyCE (ES0548 2.7.3 applies)
    bool active_low = false;               ///< CCER.CCyP
    bool complementary_active_low = false; ///< CCER.CCyNP
    bool enable = true;                    ///< CCER.CCyE
    bool complementary_enable = false;     ///< CCER.CCyNE
    bool idle_high = false;                ///< CR2.OISy  (break-capable timers)
    bool complementary_idle_high = false;  ///< CR2.OISyN
};

/// One capture/compare channel as an INPUT.
struct TimCaptureConfig {
    TimChannelSelect select = TimChannelSelect::direct;
    TimCapturePolarity polarity = TimCapturePolarity::rising;
    TimCapturePrescaler prescaler = TimCapturePrescaler::every;
    /// ICyF, 0..15: how many consecutive samples at the rate table 105
    /// gives must agree before an edge is believed. The sampling clock
    /// is tDTS or the timer clock, per the code.
    uint8_t filter = 0;
    bool enable = true;                    ///< CCER.CCyE
};

/// The slave controller, written as one configuration because SMS and TS
/// belong together: 21.4.3's own note says TS must be changed only while
/// the mode is not using it.
struct TimSlaveConfig {
    TimSlaveMode mode = TimSlaveMode::disabled;
    TimTrigger trigger = TimTrigger::itr0;
    /// SMCR.MSM: delay this timer's reaction so cascaded slaves line up
    /// cycle for cycle. DEFAULTS FALSE, and that default is ES0548
    /// 2.7.1's own workaround - see the file header.
    bool master_slave = false;
};

/// BDTR, the break and dead-time unit of the four timers that have one
/// (21.4.18). LOCK IS ONE-WAY: once written non-zero, the level's
/// registers are read-only until the next peripheral reset - which is
/// why it is spelled here and defaulted to none.
struct TimBreakDeadTime {
    /// DTG, raw: 21.4.18's four ranges, 0xx = DTG x tDTG, 10x =
    /// (64 + DTG[5:0]) x 2, 110 = (32 + DTG[4:0]) x 8, 111 =
    /// (32 + DTG[4:0]) x 16, all in tDTS units. `tim_dead_time_ticks()`
    /// below decodes it and `tim_dead_time_code()` searches for a code.
    uint8_t dead_time = 0;
    bool main_output_enable = true;     ///< MOE - without it no output is driven
    bool automatic_output_enable = false;  ///< AOE
    bool break_enable = false;          ///< BKE
    bool break_active_high = false;     ///< BKP
    uint8_t break_filter = 0;           ///< BKF, 0..15
    bool off_state_run = false;         ///< OSSR
    bool off_state_idle = false;        ///< OSSI
    uint8_t lock = 0;                   ///< LOCK 0..3, ONE-WAY (see above)
};

/// The dead time a DTG code produces, in tDTS ticks (21.4.18's four
/// ranges). tDTS is the timer clock divided by CR1.CKD.
constexpr uint16_t tim_dead_time_ticks(uint8_t dtg) {
    if ((dtg & 0x80u) == 0u) {
        return dtg;
    }
    if ((dtg & 0xC0u) == 0x80u) {
        return static_cast<uint16_t>((64u + (dtg & 0x3Fu)) * 2u);
    }
    if ((dtg & 0xE0u) == 0xC0u) {
        return static_cast<uint16_t>((32u + (dtg & 0x1Fu)) * 8u);
    }
    return static_cast<uint16_t>((32u + (dtg & 0x1Fu)) * 16u);
}

/// The smallest DTG code whose dead time reaches `ticks` tDTS units, or
/// 0xFF when the ranges cannot: the generator is coarse above 127 ticks
/// and a caller that asks for 200 gets 208, never 192 - the "at least"
/// direction every waiting verb in brio takes.
constexpr uint8_t tim_dead_time_code(uint16_t ticks) {
    for (uint16_t c = 0; c < 256u; ++c) {
        if (tim_dead_time_ticks(static_cast<uint8_t>(c)) >= ticks) {
            return static_cast<uint8_t>(c);
        }
    }
    return 0xFFu;
}

/// Which timer feeds ITR0..ITR3 of instance `n` (tables 119, 123, 130),
/// as an instance NUMBER; 0 where the entry is not a timer's TRGO but a
/// channel output (TIM14_OC1, TIM16_OC1, TIM17_OC1 - named separately by
/// `tim_internal_trigger_is_oc1`), or where there is no such link.
///
/// THIS IS THE MANUAL'S TABLE and not the header's, like every other
/// geometric fact of this chapter; it is here rather than in the reserve
/// because it is a fact about TIMERS and not about this DEVICE - it is
/// the same on every part of the family, and an instance that does not
/// exist is caught by tim_present() one line up.
constexpr uint8_t tim_internal_trigger(uint8_t n, uint8_t itr) {
    if (itr > 3u) {
        return 0u;
    }
    // Table 119 (TIM1), table 123 (TIM2/3/4), table 130 (TIM15).
    constexpr uint8_t none = 0u;
    switch (n) {
        case 1:
            return itr == 0u ? 15u : itr == 1u ? 2u : itr == 2u ? 3u : 17u;
        case 2:
            return itr == 0u ? 1u : itr == 1u ? 15u : itr == 2u ? 3u : 14u;
        case 3:
        case 4:
            return itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 15u : 14u;
        case 15:
            return itr == 0u ? 2u : itr == 1u ? 3u : itr == 2u ? 16u : 17u;
        default:
            return none;
    }
}

/// Whether that link is the master's OC1 OUTPUT rather than its TRGO -
/// the one-channel timers have no master mode at all (25.4.24), so what
/// reaches a slave is their waveform, and its pulse must be at least two
/// of the SLAVE's clock cycles wide for the slave to see it.
constexpr bool tim_internal_trigger_is_oc1(uint8_t n, uint8_t itr) {
    const uint8_t src = tim_internal_trigger(n, itr);
    return src == 14u || src == 16u || src == 17u;
}

/// The ITRx index by which slave `n` reaches master `m`, or 0xFF when
/// there is no such link - the direction a caller actually thinks in.
constexpr uint8_t tim_trigger_index_for(uint8_t n, uint8_t master) {
    for (uint8_t i = 0; i < 4u; ++i) {
        if (tim_internal_trigger(n, i) == master) {
            return i;
        }
    }
    return 0xFFu;
}

/// TIMPCLK: the clock every timer of this family counts. PCLK when the
/// APB prescaler is 1, twice PCLK otherwise (5.2.13); Clock<> pins the
/// prescaler at 1, and Tim<n>::clock_ok() checks that at run time.
template <typename Clock>
constexpr uint32_t tim_clock_hz(Clock) {
    return Clock::pclk_hz;
}

/// A pad handed to a timer channel.
///
///   constexpr brio::PinSel led{'A', 5, brio::PinFunction::af2};  // TIM2_CH1
///   using LedOut = brio::TimPad<led>;
///   LedOut::claim();
///
/// The AF NUMBER IS THE DATASHEET'S (DS13560 tables 13..24) and no
/// symbol of the device header can check it - fact 5 of the file header.
/// The pad's input buffer stays live in alternate mode (7.3.1), so a
/// driven waveform is readable on IDR and an EXTI line can watch it,
/// which is what makes this chapter measurable with no wire.
template <PinSel sel>
struct TimPad {
    TimPad() = delete;

    static_assert(sel.valid(),
                  "brio TimPad: this device has no such pad (port absent, or a pin "
                  "number past 15)");

    using pin = Pin<sel.port, sel.pin>;
    static constexpr PinSel selection = sel;

    /// Hand the pad to the timer as an OUTPUT of that alternate function.
    static void claim(PinSpeed speed = PinSpeed::low, bool open_drain = false) {
        pin::function(sel.function, {.open_drain = open_drain, .speed = speed});
    }
    /// The same handover for a CAPTURE channel: the AF input path, with a
    /// pull if the pad needs one to rest somewhere.
    static void claim_input(PinPull pull = PinPull::none) {
        pin::function(sel.function, {.pull = pull});
    }
    static void release() { pin::release(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Tim<n>: one TIMx block.
 *
 *   using Pwm = brio::Tim<2>;
 *   Pwm::init();                                   // clock on, reset
 *   Pwm::configure({.prescaler = 63, .period = 999});
 *   Pwm::output_channel(0, {.mode = brio::TimOutputMode::pwm1,
 *                           .compare = 500});
 *   Pwm::enable(true);
 *
 * Every verb that names a feature this instance does not have returns
 * false and writes NOTHING (fact 1 of the file header). Every verb that
 * names a channel checks it against `channels` the same way.
 */
template <uint8_t n>
class Tim {
public:
    Tim() = delete;

    static_assert(tim_present(n),
                  "brio Tim: this device has no such timer (the device header "
                  "declares no TIMn_BASE for it; the G0B1/G0C1 has TIM1..4, 6, 7, "
                  "14..17, the G071 class the same without TIM4, the G031 class "
                  "TIM1, 2, 3, 14, 16 and 17)");

    static constexpr uint8_t instance = n;

    // ---- what this instance IS (stm32g0/device_tables.hpp) ------------------
    static constexpr uint8_t counter_bits = tim_counter_bits(n);
    static constexpr uint32_t max_period = tim_max_period(n);
    static constexpr uint8_t channels = tim_channels(n);
    static constexpr uint8_t complementary_channels = tim_complementary_channels(n);
    static constexpr bool has_slave_mode = tim_has_slave_mode(n);
    static constexpr bool has_master_mode = tim_has_master_mode(n);
    static constexpr bool has_break = tim_has_break(n);
    static constexpr bool has_repetition = tim_has_repetition(n);
    static constexpr bool has_direction = tim_has_direction(n);
    static constexpr bool has_center_aligned = tim_has_center_aligned(n);
    static constexpr bool has_tisel = tim_has_tisel(n);
    static constexpr bool has_dma_burst = tim_has_dma_burst(n);
    static constexpr bool has_external_trigger = tim_has_external_trigger(n);
    static constexpr bool has_split_vector = tim_has_split_vector(n);

    static constexpr IRQn_Type irq() { return tim_irq(n); }
    /// TIM1's capture/compare vector; every other timer's own vector.
    static constexpr IRQn_Type cc_irq() { return tim_cc_irq(n); }

    static TIM_TypeDef& regs() { return *reinterpret_cast<TIM_TypeDef*>(tim_base(n)); }

    // ---- the bus clock and the reset ---------------------------------------
    //
    // 5.2.17 again: a peripheral whose enable bit is clear does not even
    // answer register reads, so init() opens the gate before it touches
    // anything, and the reset that follows is what makes the state this
    // driver assumes true whatever a bootloader left behind.

    static void bus_clock(bool on) {
        constexpr TimBusClock bc = tim_bus_clock(n);
        if constexpr (bc.apb2) {
            Rcc::apb2_clock(bc.enable_mask, on);
        } else {
            Rcc::apb1_clock(bc.enable_mask, on);
        }
    }
    static bool bus_clock() {
        constexpr TimBusClock bc = tim_bus_clock(n);
        if constexpr (bc.apb2) {
            return Rcc::apb2_clock(bc.enable_mask);
        } else {
            return Rcc::apb1_clock(bc.enable_mask);
        }
    }
    /// Pulse the block's reset line: every register back to the value the
    /// chapter's register map prints.
    static void reset() {
        constexpr TimBusClock bc = tim_bus_clock(n);
        if constexpr (bc.apb2) {
            Rcc::apb2_reset(bc.reset_mask);
        } else {
            Rcc::apb1_reset(bc.reset_mask);
        }
    }
    /// Clock on, then reset. The whole of "bring this timer up".
    static void init() {
        bus_clock(true);
        reset();
    }
    /// Counter stopped, outputs off, pads NOT touched (a pad is the
    /// caller's claim - TimPad::release() is its release), block reset
    /// and its clock closed.
    static void release() {
        enable(false);
        reset();
        bus_clock(false);
    }

    /// Is the APB prescaler still 1, so that TIMPCLK == PCLK (5.2.13)?
    /// The one assumption tim_clock_hz() makes about the clock tree.
    static bool clock_ok() { return Rcc::bus_prescalers_are_unity(); }

    // ---- the time base (21.4.1, 21.4.11, 21.4.12) ---------------------------

    /// Whether a configuration is legal for THIS instance - the same
    /// judgment configure() makes, available at compile time so a task
    /// can static_assert on it.
    static constexpr bool config_valid(const TimConfig& c) {
        if (c.period == 0u || c.period > max_period) {
            return false;
        }
        if (c.direction != TimDirection::up && !has_direction) {
            return false;
        }
        if (c.alignment != TimAlignment::edge && !has_center_aligned) {
            return false;
        }
        if (c.clock_division == static_cast<TimClockDivision>(3)) {
            return false;
        }
        if (c.repetition != 0u && !has_repetition) {
            return false;
        }
        return true;
    }

    /**
     * Write the time base and leave the counter STOPPED.
     *
     * The order is the chapter's: the counter is stopped first, PSC and
     * ARR are written, CR1 is assembled in one store, and then EGR.UG
     * loads both shadow registers (fact 3 of the file header) - after
     * which the update flag that generated is cleared, because it is
     * ours and not the application's. CNT is zeroed too, so a
     * reconfigured timer starts where the caller thinks it does.
     */
    static bool configure(const TimConfig& c) {
        if (!config_valid(c)) {
            return false;
        }
        TIM_TypeDef& t = regs();
        t.CR1 &= ~TIM_CR1_CEN;
        t.PSC = c.prescaler;
        t.ARR = c.period;
        if constexpr (has_repetition) {
            t.RCR = c.repetition;
        }
        uint32_t cr1 = 0;
        if constexpr (has_direction) {
            if (c.direction == TimDirection::down) {
                cr1 |= TIM_CR1_DIR;
            }
        }
        if constexpr (has_center_aligned) {
            cr1 |= (static_cast<uint32_t>(c.alignment) << TIM_CR1_CMS_Pos) & TIM_CR1_CMS_Msk;
        }
        cr1 |= (static_cast<uint32_t>(c.clock_division) << TIM_CR1_CKD_Pos) & TIM_CR1_CKD_Msk;
        if (c.auto_reload_preload) { cr1 |= TIM_CR1_ARPE; }
        if (c.one_pulse) { cr1 |= TIM_CR1_OPM; }
        if (c.update_disable) { cr1 |= TIM_CR1_UDIS; }
        if (c.update_on_overflow_only) { cr1 |= TIM_CR1_URS; }
        if (c.uif_remap) { cr1 |= TIM_CR1_UIFREMAP; }
        t.CR1 = cr1;
        t.CNT = 0;
        t.EGR = TIM_EGR_UG;
        clear_flags(TIM_SR_UIF);
        return true;
    }

    /// CR1.CEN. Starting a timer is one bit and nothing else - everything
    /// that has to be true first is configure()'s.
    static void enable(bool on) {
        TIM_TypeDef& t = regs();
        t.CR1 = on ? (t.CR1 | TIM_CR1_CEN) : (t.CR1 & ~TIM_CR1_CEN);
    }
    static bool enabled() { return (regs().CR1 & TIM_CR1_CEN) != 0u; }

    /// The counter. 32 bits on TIM2 and 16 everywhere else, masked so a
    /// UIFREMAP copy in bit 31 (21.4.13) never reads back as count.
    static uint32_t count() {
        const uint32_t v = regs().CNT;
        return counter_bits == 32u ? v : (v & 0xFFFFu);
    }
    static void set_count(uint32_t v) { regs().CNT = v & max_period; }
    /// CNT bit 31 when CR1.UIFREMAP is set: the update flag, readable in
    /// the same access as the count - which is how a 16-bit counter is
    /// read without a race against its own wrap.
    static bool count_update_flag() {
        return counter_bits == 16u && (regs().CNT & 0x80000000UL) != 0u;
    }

    static uint16_t prescaler() { return static_cast<uint16_t>(regs().PSC); }
    /// PSC takes effect at the next update event, not now (21.4.11).
    static void set_prescaler(uint16_t p) { regs().PSC = p; }

    static uint32_t period() { return regs().ARR & max_period; }
    /// ARR: at once with ARPE clear, at the next update with it set
    /// (21.4.12). Refuses zero and anything past this counter's width.
    static bool set_period(uint32_t v) {
        if (v == 0u || v > max_period) {
            return false;
        }
        regs().ARR = v;
        return true;
    }
    static bool auto_reload_preload() { return (regs().CR1 & TIM_CR1_ARPE) != 0u; }

    static uint8_t repetition() { return has_repetition ? static_cast<uint8_t>(regs().RCR) : 0u; }
    static bool set_repetition(uint8_t v) {
        if constexpr (!has_repetition) {
            (void)v;
            return false;
        } else {
            regs().RCR = v;
            return true;
        }
    }

    /// EGR: raise an event by software (21.4.6). `update()` is the one
    /// that reloads the shadow registers.
    static void update() { regs().EGR = TIM_EGR_UG; }
    static bool capture_compare_event(uint8_t ch) {
        if (ch >= channels) {
            return false;
        }
        regs().EGR = TIM_EGR_CC1G << ch;
        return true;
    }
    static bool trigger_event() {
        if constexpr (!has_slave_mode) {
            return false;
        } else {
            regs().EGR = TIM_EGR_TG;
            return true;
        }
    }
    /// EGR.BG: a break by software, which is the only break this board
    /// can raise with no wire. It clears MOE exactly as a pad would.
    static bool break_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().EGR = TIM_EGR_BG;
            return true;
        }
    }

    // ---- flags (21.4.5) -----------------------------------------------------
    //
    // rc_w0, see fact 2 of the file header: `SR = ~mask` clears exactly
    // the named flags and cannot swallow one that arrives meanwhile.

    static constexpr uint32_t update_flag = TIM_SR_UIF;
    static constexpr uint32_t trigger_flag = TIM_SR_TIF;
    static constexpr uint32_t break_flag = TIM_SR_BIF;
    static constexpr uint32_t break2_flag = TIM_SR_B2IF;
    static constexpr uint32_t commutation_flag = TIM_SR_COMIF;
    static constexpr uint32_t compare_flag(uint8_t ch) { return TIM_SR_CC1IF << ch; }
    /// CCyOF: a capture arrived while the previous one was still unread.
    /// The NEW value is lost, not the old one (21.4.5).
    static constexpr uint32_t overcapture_flag(uint8_t ch) { return TIM_SR_CC1OF << ch; }

    static uint32_t flags() { return regs().SR; }
    static bool flag(uint32_t mask) { return (regs().SR & mask) != 0u; }
    static void clear_flags(uint32_t mask) { regs().SR = ~mask; }

    // ---- interrupts and DMA requests (21.4.4) -------------------------------
    //
    // DIER's interrupt enables sit at the SAME BIT POSITIONS as the SR
    // flags they gate, which is what lets isr() mask one with the other
    // in one AND. Asserted rather than assumed:
    static_assert(TIM_DIER_UIE == TIM_SR_UIF && TIM_DIER_CC1IE == TIM_SR_CC1IF &&
                      TIM_DIER_CC4IE == TIM_SR_CC4IF && TIM_DIER_COMIE == TIM_SR_COMIF &&
                      TIM_DIER_TIE == TIM_SR_TIF && TIM_DIER_BIE == TIM_SR_BIF,
                  "TIMx_DIER's interrupt enables and TIMx_SR's flags must share their "
                  "bit positions (RM0444 21.4.4, 21.4.5) - isr() ANDs them");

    static constexpr uint32_t update_interrupt = TIM_DIER_UIE;
    static constexpr uint32_t trigger_interrupt = TIM_DIER_TIE;
    static constexpr uint32_t break_interrupt = TIM_DIER_BIE;
    static constexpr uint32_t compare_interrupt(uint8_t ch) { return TIM_DIER_CC1IE << ch; }
    static constexpr uint32_t update_dma = TIM_DIER_UDE;
    static constexpr uint32_t trigger_dma = TIM_DIER_TDE;
    static constexpr uint32_t compare_dma(uint8_t ch) { return TIM_DIER_CC1DE << ch; }

    static void interrupts(uint32_t mask, bool on) {
        TIM_TypeDef& t = regs();
        t.DIER = on ? (t.DIER | mask) : (t.DIER & ~mask);
    }
    static uint32_t interrupts() { return regs().DIER; }

    /**
     * The body of this timer's vector - and on this family a vector is
     * usually SHARED (TIM3 with TIM4, TIM6 with the DAC and LPTIM1,
     * TIM16 and TIM17 with the FDCAN lines), so a handler calls one body
     * per owner and each answers only for its own flags.
     *
     * Returns the flags that were BOTH set and enabled, having cleared
     * exactly those. A flag whose interrupt is not enabled is left
     * standing for a poller to read - unlike this family's EXTI, where
     * an unarmed line has no pending bit at all.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        TIM_TypeDef& t = regs();
        const uint32_t dier = t.DIER;
        uint32_t enabled_flags = dier & 0xFFu;
        if ((dier & TIM_DIER_BIE) != 0u) {
            enabled_flags |= TIM_SR_B2IF;   // the second break has no enable of its own
        }
        const uint32_t hit = t.SR & enabled_flags;
        if (hit != 0u) {
            t.SR = ~hit;
        }
        return hit;
    }

    // ---- capture/compare channels (21.4.7 .. 21.4.9) ------------------------

    /// CCRy. Reading a CAPTURE channel's register is what clears its
    /// CCyIF (21.4.5), so this verb is the acknowledgement as well as
    /// the reading - a handler that clears the flag without reading
    /// throws the measurement away.
    static uint32_t compare(uint8_t ch) {
        if (ch >= channels) {
            return 0;
        }
        const uint32_t v = ccr(ch);
        return counter_bits == 32u ? v : (v & 0xFFFFu);
    }
    static bool set_compare(uint8_t ch, uint32_t v) {
        if (ch >= channels || v > max_period) {
            return false;
        }
        set_ccr(ch, v);
        return true;
    }

    /// Configure channel `ch` as an OUTPUT. Writes CCMR, CCR, CR2's idle
    /// levels (where there are any) and CCER, in that order, so the
    /// enable is the last store - the pad is never driven by a
    /// half-written configuration.
    static bool output_channel(uint8_t ch, const TimChannelConfig& c) {
        if (ch >= channels || c.compare > max_period) {
            return false;
        }
        if (c.complementary_enable && ch >= complementary_channels) {
            return false;
        }
        TIM_TypeDef& t = regs();
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        const uint32_t mode = static_cast<uint32_t>(c.mode);
        uint32_t v = 0;
        v |= (mode & 0x7u) << (shift + 4u);
        v |= ((mode >> 3u) & 0x1u) << (16u + shift);
        if (c.fast) { v |= 1u << (shift + 2u); }
        if (c.preload) { v |= 1u << (shift + 3u); }
        if (c.clear_on_ocref_clr) { v |= 1u << (shift + 7u); }
        write_ccmr(ch, v);

        set_ccr(ch, c.compare);

        if constexpr (has_break) {
            // OISy / OISyN live in CR2 and are write-protected by
            // BDTR.LOCK level 1 and up (21.4.18) - a caller that has
            // locked the timer gets what it locked, not an error.
            const uint32_t ois_shift = 8u + 2u * ch;
            uint32_t cr2 = t.CR2 & ~(0x3u << ois_shift);
            if (c.idle_high) { cr2 |= 1u << ois_shift; }
            if (c.complementary_idle_high && ch < complementary_channels) {
                cr2 |= 2u << ois_shift;
            }
            t.CR2 = cr2;
        }

        write_ccer(ch, c.enable, c.active_low, c.complementary_enable && ch < complementary_channels,
                   c.complementary_active_low);
        return true;
    }

    /// Configure channel `ch` as an INPUT. The channel is disabled in
    /// CCER first, because 21.4.9 makes CCyS writable only then - the
    /// silicon's own rule, and the reason this is not one store.
    static bool capture_channel(uint8_t ch, const TimCaptureConfig& c) {
        if (ch >= channels || c.select == TimChannelSelect::output) {
            return false;
        }
        if (c.filter > 15u) {
            return false;
        }
        if (c.select == TimChannelSelect::trc && !has_slave_mode) {
            return false;   // TRC is the slave controller's signal
        }
        write_ccer(ch, false, false, false, false);

        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint32_t v = 0;
        v |= static_cast<uint32_t>(c.select) << shift;
        v |= static_cast<uint32_t>(c.prescaler) << (shift + 2u);
        v |= static_cast<uint32_t>(c.filter) << (shift + 4u);
        write_ccmr(ch, v);

        const uint8_t pol = static_cast<uint8_t>(c.polarity);
        write_ccer(ch, c.enable, (pol & 1u) != 0u, false, (pol & 2u) != 0u);
        return true;
    }

    /// CCER.CCyE alone, for a channel already configured.
    static bool channel_enable(uint8_t ch, bool on) {
        if (ch >= channels) {
            return false;
        }
        TIM_TypeDef& t = regs();
        const uint32_t bit = TIM_CCER_CC1E << (4u * ch);
        t.CCER = on ? (t.CCER | bit) : (t.CCER & ~bit);
        return true;
    }
    static bool channel_enabled(uint8_t ch) {
        return ch < channels && (regs().CCER & (TIM_CCER_CC1E << (4u * ch))) != 0u;
    }
    static bool complementary_enable(uint8_t ch, bool on) {
        if (ch >= complementary_channels) {
            return false;
        }
        TIM_TypeDef& t = regs();
        const uint32_t bit = TIM_CCER_CC1NE << (4u * ch);
        t.CCER = on ? (t.CCER | bit) : (t.CCER & ~bit);
        return true;
    }

    // ---- the slave controller and the master output (21.4.2, 21.4.3) --------

    static bool slave(const TimSlaveConfig& c) {
        if constexpr (!has_slave_mode) {
            (void)c;
            return false;
        } else {
            if (static_cast<uint8_t>(c.mode) > 8u) {
                return false;
            }
            if (c.trigger == TimTrigger::etr && !has_external_trigger) {
                return false;
            }
            if (c.mode == TimSlaveMode::gated && c.trigger == TimTrigger::ti1_edge) {
                return false;   // 21.4.3's own note: TI1F_ED has no level to gate on
            }
            const uint32_t sms = static_cast<uint32_t>(c.mode);
            const uint32_t ts = static_cast<uint32_t>(c.trigger);
            TIM_TypeDef& t = regs();
            uint32_t v = t.SMCR & ~(TIM_SMCR_SMS_Msk | TIM_SMCR_TS_Msk | TIM_SMCR_MSM);
            v |= (sms & 0x7u) | (((sms >> 3u) & 0x1u) << 16u);
            v |= ((ts & 0x7u) << 4u) | (((ts >> 3u) & 0x3u) << 20u);
            if (c.master_slave) { v |= TIM_SMCR_MSM; }
            t.SMCR = v;
            return true;
        }
    }
    static TimSlaveMode slave_mode() {
        if constexpr (!has_slave_mode) {
            return TimSlaveMode::disabled;
        } else {
            const uint32_t v = regs().SMCR;
            return static_cast<TimSlaveMode>((v & 0x7u) | (((v >> 16u) & 0x1u) << 3u));
        }
    }
    static TimTrigger slave_trigger() {
        if constexpr (!has_slave_mode) {
            return TimTrigger::itr0;
        } else {
            const uint32_t v = regs().SMCR;
            return static_cast<TimTrigger>(((v >> 4u) & 0x7u) | (((v >> 20u) & 0x3u) << 3u));
        }
    }

    /// CR2.MMS - what goes out on TRGO. 21.4.2's own note, worth
    /// repeating because it is the commonest cascade bug: the SLAVE's
    /// clock must already be enabled before the master starts sending.
    static bool master(TimMasterMode m) {
        if constexpr (!has_master_mode) {
            (void)m;
            return false;
        } else {
            if (static_cast<uint8_t>(m) > 3u && channels == 0u) {
                return false;   // a basic timer has no OCxREF to publish
            }
            if (static_cast<uint8_t>(m) > 3u &&
                static_cast<uint8_t>(m) - 4u >= channels) {
                return false;
            }
            TIM_TypeDef& t = regs();
            t.CR2 = (t.CR2 & ~TIM_CR2_MMS_Msk) |
                    ((static_cast<uint32_t>(m) << TIM_CR2_MMS_Pos) & TIM_CR2_MMS_Msk);
            return true;
        }
    }
    static TimMasterMode master() {
        return static_cast<TimMasterMode>((regs().CR2 & TIM_CR2_MMS_Msk) >> TIM_CR2_MMS_Pos);
    }

    // ---- break and dead time (21.4.18) --------------------------------------

    static bool break_dead_time(const TimBreakDeadTime& c) {
        if constexpr (!has_break) {
            (void)c;
            return false;
        } else {
            if (c.break_filter > 15u || c.lock > 3u) {
                return false;
            }
            uint32_t v = c.dead_time;
            v |= (static_cast<uint32_t>(c.lock) << TIM_BDTR_LOCK_Pos) & TIM_BDTR_LOCK_Msk;
            if (c.off_state_idle) { v |= TIM_BDTR_OSSI; }
            if (c.off_state_run) { v |= TIM_BDTR_OSSR; }
            if (c.break_enable) { v |= TIM_BDTR_BKE; }
            if (c.break_active_high) { v |= TIM_BDTR_BKP; }
            if (c.automatic_output_enable) { v |= TIM_BDTR_AOE; }
            if (c.main_output_enable) { v |= TIM_BDTR_MOE; }
            v |= (static_cast<uint32_t>(c.break_filter) << TIM_BDTR_BKF_Pos) & TIM_BDTR_BKF_Msk;
            regs().BDTR = v;
            return true;
        }
    }

    /// BDTR.MOE on its own: the master switch every output of a
    /// break-capable timer passes through. A break clears it in hardware.
    static bool main_output(bool on) {
        if constexpr (!has_break) {
            (void)on;
            return false;
        } else {
            TIM_TypeDef& t = regs();
            t.BDTR = on ? (t.BDTR | TIM_BDTR_MOE) : (t.BDTR & ~TIM_BDTR_MOE);
            return true;
        }
    }
    static bool main_output() { return has_break && (regs().BDTR & TIM_BDTR_MOE) != 0u; }

    // ---- the input multiplexer (21.4.29 and its twins) ----------------------

    /**
     * TIMx_TISEL: which SOURCE feeds TIy. Code 0 is always the channel's
     * own pad; what the other codes are is per timer and is the
     * chapter's table - TIM16 reaches LSI, LSE and the RTC wake-up,
     * TIM17 HSI48/256, HSE/32 and the MCOs, TIM14 the RTC clock, HSE/32
     * and the MCOs, TIM1/2/3/4 the comparator outputs, TIM15's TI2 the
     * capture signals of TIM2 and TIM3.
     *
     * THIS IS WHAT MAKES A CAPTURE CHANNEL MEASURABLE WITH NO PAD AT
     * ALL, which is why the driver exposes the raw code and names no
     * source: the vocabulary of what code 1 means belongs to the
     * peripheral that owns the signal (the samc EVSYS ruling), and half
     * of these sources have no driver in this stratum yet.
     */
    static bool input_select(uint8_t ch, uint8_t code) {
        if constexpr (!has_tisel) {
            (void)ch; (void)code;
            return false;
        } else {
            if (ch >= channels || code > 15u) {
                return false;
            }
            const uint32_t shift = 8u * ch;
            TIM_TypeDef& t = regs();
            t.TISEL = (t.TISEL & ~(0xFu << shift)) | (static_cast<uint32_t>(code) << shift);
            return true;
        }
    }
    static uint8_t input_select(uint8_t ch) {
        if constexpr (!has_tisel) {
            (void)ch;
            return 0;
        } else {
            return ch >= channels ? 0u : static_cast<uint8_t>((regs().TISEL >> (8u * ch)) & 0xFu);
        }
    }

private:
    static volatile uint32_t& ccr_ref(uint8_t ch) {
        // CCR1..CCR4 are four consecutive words at 0x34 (21.4.13..16).
        return *(&regs().CCR1 + ch);
    }
    static uint32_t ccr(uint8_t ch) { return ccr_ref(ch); }
    static void set_ccr(uint8_t ch, uint32_t v) { ccr_ref(ch) = v; }

    static void write_ccmr(uint8_t ch, uint32_t value) {
        TIM_TypeDef& t = regs();
        const uint32_t mask = 0x00FF00FFu << (8u * (ch & 1u));
        if (ch < 2u) {
            t.CCMR1 = (t.CCMR1 & ~mask) | value;
        } else {
            t.CCMR2 = (t.CCMR2 & ~mask) | value;
        }
    }

    /// CCER's four bits of one channel: E, P, NE, NP at 4 x ch. Channel 4
    /// has no NE bit (21.4.9 leaves it reserved), and `ne` is only ever
    /// true for a channel with a complementary output, which channel 4
    /// never is.
    static void write_ccer(uint8_t ch, bool e, bool p, bool ne, bool np) {
        TIM_TypeDef& t = regs();
        const uint32_t shift = 4u * ch;
        uint32_t v = t.CCER & ~(0xFu << shift);
        if (e) { v |= 1u << shift; }
        if (p) { v |= 2u << shift; }
        if (ne) { v |= 4u << shift; }
        if (np) { v |= 8u << shift; }
        t.CCER = v;
    }
};

// =============================================================================
// Tasks
// =============================================================================

/**
 * TimPwm<T, ch, top>: one PWM output, a `PwmChannel` (util/pwm_channel.hpp)
 * whose `max` is the PERIOD the timer runs at.
 *
 *   using Led = brio::TimPwm<brio::Tim<2>, 0, 1000>;
 *   Led::setup(63);           // 64 MHz / 64 / 1001 = about 1 kHz
 *   Led::duty(250);           // a quarter
 *
 * `top` is a template parameter because PwmChannel requires `max` to be
 * a compile-time constant: a full scale that can move under a generic
 * actuator is not one it can scale against (the samc TcPwm8 ruling).
 *
 * The frequency belongs to the TIMER and the duty to the CHANNEL, which
 * is the concept's own division of labour - so `setup()` takes the
 * prescaler and `duty()` takes nothing else.
 */
template <class T, uint8_t ch, uint16_t top = 0xFFFF>
struct TimPwm {
    TimPwm() = delete;
    static_assert(ch < T::channels,
                  "brio TimPwm: this timer has no such capture/compare channel");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    /// Bring the timer up as an edge-aligned PWM generator on this
    /// channel. On a break-capable timer MOE is raised too, without which
    /// nothing reaches the pad at all (21.4.18) - the trap that makes a
    /// first TIM1 PWM look dead.
    static bool setup(uint16_t prescaler = 0, TimOutputMode mode = TimOutputMode::pwm1,
                      bool active_low = false) {
        if (!T::configure({.prescaler = prescaler,
                           .period = top,
                           .auto_reload_preload = true})) {
            return false;
        }
        if (!T::output_channel(ch, {.mode = mode, .compare = 0, .active_low = active_low})) {
            return false;
        }
        if constexpr (T::has_break) {
            (void)T::main_output(true);
        }
        T::enable(true);
        return true;
    }

    /// The duty, in counts out of `max`. A plain store into a PRELOADED
    /// compare register: it is taken at the next update, so a change
    /// never cuts the pulse that is being produced.
    static void duty(uint16_t v) { (void)T::set_compare(ch, v > max ? max : v); }
    static uint16_t duty() { return static_cast<uint16_t>(T::compare(ch)); }
};

/**
 * TimPairPwm<T, ch, top>: a channel and its COMPLEMENT, with the
 * dead time the silicon inserts between them - the shape only the four
 * break-capable timers of this family have (TIM1's channels 1..3,
 * TIM15/16/17's channel 1).
 *
 * `max` and `duty()` are the PwmChannel contract again, on the pair: the
 * two outputs are one actuator and one duty.
 */
template <class T, uint8_t ch, uint16_t top = 0xFFFF>
struct TimPairPwm {
    TimPairPwm() = delete;
    static_assert(T::has_break,
                  "brio TimPairPwm: this timer has no break/dead-time unit, so it has "
                  "no complementary output either (TIM1, TIM15, TIM16 and TIM17 have "
                  "one; TIM2/3/4, TIM6/7 and TIM14 do not)");
    static_assert(ch < T::complementary_channels,
                  "brio TimPairPwm: this channel has no complementary output");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    /// `dead_time` is a raw DTG code - `tim_dead_time_code(ticks)` turns
    /// a wanted number of tDTS ticks into one, always rounding UP.
    static bool setup(uint16_t prescaler = 0, uint8_t dead_time = 0,
                      TimClockDivision division = TimClockDivision::div1) {
        if (!T::configure({.prescaler = prescaler,
                           .period = top,
                           .clock_division = division,
                           .auto_reload_preload = true})) {
            return false;
        }
        if (!T::output_channel(ch, {.mode = TimOutputMode::pwm1,
                                    .compare = 0,
                                    .complementary_enable = true})) {
            return false;
        }
        if (!T::break_dead_time({.dead_time = dead_time, .main_output_enable = true})) {
            return false;
        }
        T::enable(true);
        return true;
    }

    static void duty(uint16_t v) { (void)T::set_compare(ch, v > max ? max : v); }
    static uint16_t duty() { return static_cast<uint16_t>(T::compare(ch)); }

    /// The dead time in force, in tDTS ticks (which are timer clock ticks
    /// divided by CR1.CKD).
    static uint16_t dead_time_ticks() {
        return tim_dead_time_ticks(static_cast<uint8_t>(T::regs().BDTR & TIM_BDTR_DTG_Msk));
    }
};

/**
 * TimPeriodMeter<T>: the PERIOD and the HIGH TIME of a signal on TI1, in
 * one capture arrangement - the chapter's "PWM input mode" (21.3.6).
 *
 * Two channels watch the same input: channel 1 directly on the rising
 * edge (the period), channel 2 indirectly on the falling one (the
 * width), and the slave controller RESETS the counter on every rising
 * edge so both readings are measured from the same origin. So this
 * needs a timer with a slave controller and two channels - TIM1, TIM2,
 * TIM3, TIM4 and TIM15 - and it costs BOTH of them.
 *
 * `period_ticks()` and `width_ticks()` READ the capture registers, which
 * is what clears their flags; a handler that clears CCyIF without
 * reading has thrown the measurement away.
 */
template <class T>
struct TimPeriodMeter {
    TimPeriodMeter() = delete;
    static_assert(T::channels >= 2,
                  "brio TimPeriodMeter: PWM input mode watches one input with TWO "
                  "channels (RM0444 21.3.6)");
    static_assert(T::has_slave_mode,
                  "brio TimPeriodMeter: PWM input mode resets the counter on every "
                  "rising edge, which is the slave controller's job");

    /// `invert` measures the LOW time and the period from falling edges
    /// instead (the whole arrangement mirrored, 21.3.6's own note).
    static bool setup(uint16_t prescaler = 0, uint8_t filter = 0, bool invert = false) {
        if (!T::configure({.prescaler = prescaler, .period = T::max_period})) {
            return false;
        }
        const TimCapturePolarity direct = invert ? TimCapturePolarity::falling
                                                 : TimCapturePolarity::rising;
        const TimCapturePolarity indirect = invert ? TimCapturePolarity::rising
                                                   : TimCapturePolarity::falling;
        if (!T::capture_channel(0, {.select = TimChannelSelect::direct,
                                    .polarity = direct,
                                    .filter = filter})) {
            return false;
        }
        if (!T::capture_channel(1, {.select = TimChannelSelect::indirect,
                                    .polarity = indirect,
                                    .filter = filter})) {
            return false;
        }
        if (!T::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::ti1})) {
            return false;
        }
        T::enable(true);
        return true;
    }

    /// The counter is reset ON the edge and the capture is taken AT it,
    /// so a period reads as its own tick count and not one less - the
    /// opposite of the samc TC's capture, which clears and latches
    /// together and always reads one short.
    static uint32_t period_ticks() { return T::compare(0); }
    static uint32_t width_ticks() { return T::compare(1); }

    static constexpr uint32_t period_flag = T::compare_flag(0);
    static constexpr uint32_t width_flag = T::compare_flag(1);
    static constexpr uint32_t overrun_flag = T::overcapture_flag(0);
    static constexpr uint32_t period_interrupt = T::compare_interrupt(0);
};

/**
 * TimIntervalMeter<T, ch>: the interval between consecutive edges of one
 * input, on ONE channel and a free-running counter - what a timer with a
 * single channel (TIM14, TIM16, TIM17) can measure, and what a capture
 * ISR hands to a util/meter_sampler.hpp MeterLatch.
 *
 *   using Lsi = brio::TimIntervalMeter<brio::Tim<16>, 0>;
 *   extern "C" void TIM16_FDCAN_IT0_IRQHandler() {
 *       if (brio::Tim<16>::isr() & Lsi::capture_flag) {
 *           if (auto d = Lsi::interval()) { Latch::store(*d); }
 *       }
 *   }
 *
 * The subtraction is done in the counter's own modulus, so a wrap
 * between two edges costs nothing as long as the interval is shorter
 * than one full counter period - which is the meter's whole contract and
 * is the caller's to arrange with the prescaler.
 *
 * `interval()` keeps ONE value of state and is meant for the capture
 * handler alone: there is no critical section in it, because the
 * handler is the only writer and the only reader.
 */
template <class T, uint8_t ch = 0>
struct TimIntervalMeter {
    TimIntervalMeter() = delete;
    static_assert(ch < T::channels,
                  "brio TimIntervalMeter: this timer has no such capture channel");

    static bool setup(uint16_t prescaler = 0, uint8_t filter = 0,
                      TimCapturePolarity polarity = TimCapturePolarity::rising,
                      TimCapturePrescaler divider = TimCapturePrescaler::every) {
        have_last_ = false;
        if (!T::configure({.prescaler = prescaler, .period = T::max_period})) {
            return false;
        }
        if (!T::capture_channel(ch, {.select = TimChannelSelect::direct,
                                     .polarity = polarity,
                                     .prescaler = divider,
                                     .filter = filter})) {
            return false;
        }
        T::enable(true);
        return true;
    }

    /// Read the capture (which acknowledges it) and return the distance
    /// from the previous one - nothing on the first edge, there being no
    /// interval yet.
    static std::optional<uint32_t> interval() {
        const uint32_t now = T::compare(ch);
        const uint32_t last = last_;
        const bool had = have_last_;
        last_ = now;
        have_last_ = true;
        if (!had) {
            return std::nullopt;
        }
        return (now - last) & T::max_period;
    }

    /// Throw the previous edge away: the next interval() returns nothing
    /// and starts a fresh pair. A measurement that spans a
    /// reconfiguration is not a measurement.
    static void restart() { have_last_ = false; }

    static constexpr uint32_t capture_flag = T::compare_flag(ch);
    static constexpr uint32_t overrun_flag = T::overcapture_flag(ch);
    static constexpr uint32_t capture_interrupt = T::compare_interrupt(ch);

private:
    static inline uint32_t last_ = 0;
    static inline bool have_last_ = false;
};

/**
 * TimEventCounter<T>: a timer whose CLOCK is another timer's trigger -
 * external clock mode 1 on an ITRx link (21.3.15). With the master
 * publishing its update event on TRGO, the count IS the number of master
 * periods, so a frequency is measured with no wire, no pad and no CPU in
 * the path.
 *
 *   TimEventCounter<Tim<3>>::setup(TimTrigger::itr1);   // TIM3 counts TIM2
 *
 * WHICH TIMER ITRx MEANS is per instance: `tim_trigger_index_for(slave,
 * master)` looks the link up in the manual's tables so a caller names
 * the MASTER and not a number.
 */
template <class T>
struct TimEventCounter {
    TimEventCounter() = delete;
    static_assert(T::has_slave_mode,
                  "brio TimEventCounter: counting a trigger is the slave "
                  "controller's external clock mode 1");

    static bool setup(TimTrigger trigger, uint32_t period = T::max_period) {
        if (!T::configure({.prescaler = 0, .period = period})) {
            return false;
        }
        if (!T::slave({.mode = TimSlaveMode::external_clock1, .trigger = trigger})) {
            return false;
        }
        T::enable(true);
        return true;
    }

    static uint32_t count() { return T::count(); }
    static void restart() { T::set_count(0); }
};

/**
 * TimGatedCounter<T>: a timer that counts ITS OWN clock only while the
 * trigger is HIGH (slave gated mode, 21.3.15). Pointed at a master whose
 * TRGO is OC1REF - the PWM waveform itself - the count over a known
 * window IS the high time, so a DUTY CYCLE is measured internally, with
 * no pad and no sampling.
 *
 * Gated mode must never be used with TimTrigger::ti1_edge (21.4.3's own
 * note: an edge detector has no level to gate on), which Tim::slave()
 * refuses.
 */
template <class T>
struct TimGatedCounter {
    TimGatedCounter() = delete;
    static_assert(T::has_slave_mode,
                  "brio TimGatedCounter: gating is the slave controller's job");

    static bool setup(TimTrigger trigger, uint16_t prescaler = 0,
                      uint32_t period = T::max_period) {
        if (!T::configure({.prescaler = prescaler, .period = period})) {
            return false;
        }
        if (!T::slave({.mode = TimSlaveMode::gated, .trigger = trigger})) {
            return false;
        }
        // In gated mode CEN is the software half of an AND with the
        // trigger level: the counter runs only while both are up.
        T::enable(true);
        return true;
    }

    static uint32_t count() { return T::count(); }
    static void restart() { T::set_count(0); }
};

/**
 * TimPeriodicTick<T>: the plainest use of a timer - an update event
 * every `period + 1` counter ticks, and an interrupt on it. What the
 * basic timers (TIM6, TIM7) exist for, and what any other timer does
 * with no channel configured.
 */
template <class T>
struct TimPeriodicTick {
    TimPeriodicTick() = delete;

    static bool setup(uint16_t prescaler, uint32_t period, bool interrupt = true) {
        if (!T::configure({.prescaler = prescaler, .period = period})) {
            return false;
        }
        T::interrupts(T::update_interrupt, interrupt);
        T::enable(true);
        return true;
    }
    static void stop() { T::enable(false); }
    static constexpr uint32_t flag = T::update_flag;
};

/**
 * TimOnePulse<T, ch>: one pulse of a chosen width, after a chosen delay,
 * on a trigger - CR1.OPM plus a compare channel (21.3.7). The counter
 * stops itself at the next update, so the pulse happens once per
 * trigger and the timer costs nothing between them.
 *
 * `delay` is where the pulse STARTS (the compare value) and the pulse
 * ends at the period, so width = period - delay. The trigger is the
 * caller's: a software start (T::enable(true)), a slave trigger mode on
 * an ITRx, or a TI edge.
 */
template <class T, uint8_t ch = 0>
struct TimOnePulse {
    TimOnePulse() = delete;
    static_assert(ch < T::channels,
                  "brio TimOnePulse: this timer has no such capture/compare channel");

    static bool setup(uint16_t prescaler, uint32_t delay, uint32_t width,
                      bool active_low = false) {
        if (width == 0u) {
            return false;
        }
        if (!T::configure({.prescaler = prescaler,
                           .period = delay + width,
                           .one_pulse = true})) {
            return false;
        }
        // PWM2 is active while CNT >= CCR, so the output rises at the
        // compare and falls at the update that stops the counter - the
        // chapter's own recipe for a delayed pulse.
        if (!T::output_channel(ch, {.mode = TimOutputMode::pwm2,
                                    .compare = delay,
                                    .active_low = active_low})) {
            return false;
        }
        if constexpr (T::has_break) {
            (void)T::main_output(true);
        }
        return true;
    }

    /// Arm the pulse for a trigger that STARTS the counter (slave
    /// trigger mode), or fire it now with fire().
    static bool arm(TimTrigger trigger) {
        return T::slave({.mode = TimSlaveMode::trigger, .trigger = trigger});
    }
    static void fire() {
        T::set_count(0);
        T::enable(true);
    }
    static bool busy() { return T::enabled(); }
};

} // namespace brio
