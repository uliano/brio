/*
 * tim.hpp
 *
 * The STM32F4's timers (RM0090 ch. 17..20, RM0390 ch. 15..18, RM0383
 * ch. 12..14) in the two strata every brio target uses
 * (docs/design/overview.md, "Target strata"):
 *
 *  Tim<n>          the RESOURCE - one TIMx block: the time base
 *                  (prescaler, counter, auto-reload with its preload),
 *                  the counting modes, the capture/compare channels in
 *                  both their faces, the slave controller and the master
 *                  TRGO, the external trigger, the break/dead-time unit,
 *                  the option register, the DMA burst engine, the flags
 *                  and the ISR bodies.
 *
 *  TimPad<sel>     a pad handed to a timer channel (the AF number is the
 *                  DATASHEET's - see below).
 *
 *  TASKS           TimPwm / TimPairPwm (util/pwm_channel.hpp's
 *                  PwmChannel, one output and a complementary pair),
 *                  TimPeriodMeter / TimIntervalMeter (what
 *                  util/meter_sampler.hpp's MeterLatch is fed from),
 *                  TimEventCounter / TimGatedCounter (one timer
 *                  measuring another with no wire and no CPU),
 *                  TimPeriodicTick, TimOnePulse, TimEncoder.
 *
 * SIX FACTS THAT SHAPE THIS FILE.
 *
 * 1. THERE IS ONE TIM_TypeDef AND FOURTEEN DIFFERENT TIMERS. The device
 *    header declares every register as a member of one struct, so
 *    `TIM11->SMCR` compiles and writes a hole in the address map: the
 *    silicon's TIM10/11/13/14 have no slave controller and no CR2 at all
 *    (RM0383 14.5), TIM9/12 have a slave controller but no encoder mode
 *    and no ETR (14.4.2), TIM6/7 have no channel (RM0090 20.4). NOTHING
 *    IN THE HEADER SAYS SO. What a timer IS - counter width, channels,
 *    complementary outputs, slave controller, master mode, BDTR, RCR,
 *    centre-aligned counting, ETR, DMA, an option register - therefore
 *    comes from stm32f4/device_tables.hpp, where those facts are named
 *    as what they are: the MANUALS', keyed by instance, and reached only
 *    for an instance the header says exists. Every verb that touches a
 *    register an instance does not implement REFUSES - returns false and
 *    writes nothing - rather than storing into a hole, and the tasks that
 *    need a feature static_assert on it, which is what makes a dead-time
 *    request on a TIM3 a compile error instead of a silent nothing.
 *
 * 2. THE STATUS REGISTER IS rc_w0, NOT W1C. A flag of TIMx_SR is cleared
 *    by writing ZERO to it and a write of one has no effect (17.4.5), so
 *    clearing is `SR = ~flags` - a plain store, no read-modify-write, and
 *    a flag that arrives between the read and the store SURVIVES. Every
 *    other flag register in this stratum - the EXTI's PR, the RTC's ISR -
 *    is cleared by writing ONES or ZEROES into a read-modify-write, so
 *    this is the one place where that reflex is wrong.
 *
 * 3. THE PRESCALER AND THE AUTO-RELOAD ARE SHADOWED. PSC is copied into
 *    the working register at the next UPDATE event and never before
 *    (17.4.11); ARR is too when CR1.ARPE is set, and is taken at once
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
 *    use. 17.3.9's "PWM mode" recipe asks for it too. The unbuffered
 *    behaviour is still reachable - TimChannelConfig::preload = false.
 *
 * 5. THE COUNTER'S CLOCK IS NOT THE BUS CLOCK. RM0383 6.2 and RM0090
 *    7.2's own rule: with the APB prescaler at 1 a timer counts HCLK;
 *    with it divided the timer counts TWICE its APB clock - and on every
 *    part but the F405 class a bit of RCC_DCKCFGR, TIMPRE, moves that to
 *    "HCLK at a prescaler of 1 or 2, four times PCLK otherwise". So
 *    `Tim<n>::clock_hz(clock)` derives the rate from the CLOCK TASK's own
 *    dividers (they are constexpr, so every period and prescaler folds),
 *    and `clock_hz_now(hclk)` reads the same number back out of the
 *    silicon - which is what stops this file from lying if a future
 *    dynamic clock moves a prescaler. TIMPRE itself is RCC's:
 *    stm32f4/clock.hpp's `Rcc::timpre()`.
 *
 * 6. THE PAD MAP IS THE DATASHEET'S AND NOTHING CHECKS IT. Which AF
 *    number carries TIM3_CH1 on PA6 is DS10314 table 9 (AF2), and the
 *    device header has no symbol for it - stm32f4/pin.hpp states that
 *    once for the whole stratum. So a timer pad is a `PinSel` the CALLER
 *    writes, `TimPad<sel>` is the claim, and the bench is the only check
 *    there is.
 *
 * MEASURING WITH NO WIRE. Three of this chapter's own features make a
 * timer observable with nothing attached, and they are why the reference
 * suite needs no jumper:
 *  - THE INTERNAL TRIGGERS. Every four-channel timer publishes TRGO and
 *    every slave-capable one listens on ITR0..ITR3; which timer each ITRx
 *    IS is `tim_internal_trigger()` below (RM0090 tables 94, 98 and 101).
 *    A master on "update" and a slave in external clock mode 1 count each
 *    other exactly.
 *  - THE OPTION REGISTERS. TIM5_OR routes the LSI, the LSE or the RTC
 *    wake-up into TIM5's channel 4 and TIM11_OR routes HSE_RTC into
 *    TIM11's channel 1 (RM0383 6.2.11 calls this out as the intended use):
 *    a capture channel weighs an oscillator against the core clock with no
 *    pad at all.
 *  - A PAD THE TIMER ITSELF DRIVES. The chapter's FORCED OUTPUT MODE
 *    (17.3.7) puts OCyREF under CCMRx instead of under the counter, so
 *    the CPU writes a level onto the pin through the timer's own output
 *    stage - `output_mode(ch, force_active/force_inactive)` - and TI1 and
 *    TI2 see it, because a channel's input path is live whatever CCyS
 *    says. That is what stimulates a capture or a quadrature interface
 *    here. A GPIO in OUTPUT mode does NOT do it: the alternate-function
 *    input multiplexer is opened by MODER and not by the AF nibble alone
 *    (measured - and the opposite of what the EXTI does with the same
 *    pad, which reads the input data path directly).
 *
 * ERRATA - ES0287 2.6, ES0206 2.7 and ES0298 2.6 carry the SAME four
 * items, so they are the family's:
 *  - "PWM re-enabled in automatic output enable mode despite of system
 *    break": with BDTR.AOE set a system break does not latch. Its own
 *    workaround is to leave AOE clear and use OCxCE for cycle-by-cycle
 *    control, so TimBreakDeadTime::automatic_output_enable DEFAULTS FALSE
 *    and the verb says what setting it costs.
 *  - "TRGO and TRGO2 trigger output failure": the SLAVE's clock must
 *    already be running before the master sends, and must not be changed
 *    while it does. Stated on master() and obeyed by every task here that
 *    cascades - the slave is configured and enabled first.
 *  - "Consecutive compare event missed in specific conditions": two
 *    matches in adjacent counter cycles lose the second. No workaround;
 *    it is staged at the bench rather than described.
 *  - "Output compare clear not working with external counter reset":
 *    OCxCE together with a slave reset/combined mode. This driver exposes
 *    both, so the combination is reachable and the obligation is stated
 *    on the verb.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// CR1.DIR - which way an edge-aligned counter runs (17.4.1). Only the
/// four-channel timers have the bit; every other timer of this family
/// counts up and the driver refuses to claim otherwise.
enum class TimDirection : uint8_t { up = 0, down = 1 };

/// CR1.CMS - edge-aligned, or centre-aligned with the compare flag set
/// while counting down, up, or both (17.4.1). The three centre modes
/// differ ONLY in when CCyIF is raised; the waveform is the same.
enum class TimAlignment : uint8_t {
    edge = 0,
    center_down = 1,   ///< CCyIF set while down-counting
    center_up = 2,     ///< CCyIF set while up-counting
    center_both = 3,
};

/// CR1.CKD - the ratio between the timer clock and tDTS, the sampling
/// clock of the digital filters and the unit of the dead-time generator
/// (17.4.1). Code 3 is reserved.
enum class TimClockDivision : uint8_t { div1 = 0, div2 = 1, div4 = 2 };

/// SMCR.TS - what the slave controller listens to (17.4.3). Which timer
/// ITR0..ITR3 mean is per instance: RM0090 tables 94, 98 and 101,
/// reproduced by `tim_internal_trigger()` below.
enum class TimTrigger : uint8_t {
    itr0 = 0, itr1 = 1, itr2 = 2, itr3 = 3,
    ti1_edge = 4,    ///< TI1F_ED: one pulse per TRANSITION of TI1 (never gate on it)
    ti1 = 5,         ///< TI1FP1
    ti2 = 6,         ///< TI2FP2
    etr = 7,         ///< ETRF, the external trigger input
};

/// SMCR.SMS - what the trigger does to the counter (17.4.3). The field is
/// THREE bits on this family: there is no combined reset+trigger mode and
/// no retriggerable one-pulse mode.
enum class TimSlaveMode : uint8_t {
    disabled = 0,
    encoder1 = 1,            ///< counts on TI2 edges only, direction from TI1
    encoder2 = 2,            ///< counts on TI1 edges only, direction from TI2
    encoder3 = 3,            ///< counts on both, four counts per quadrature cycle
    reset = 4,               ///< a trigger edge reinitializes the counter
    gated = 5,               ///< the counter runs while the trigger is HIGH
    trigger = 6,             ///< a trigger edge starts the counter (does not reset it)
    external_clock1 = 7,     ///< the trigger's rising edges ARE the counter clock
};

/// CR2.MMS - what this timer publishes on TRGO (17.4.2). The basic timers
/// have the first three codes and no more, having no channel to publish.
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

/// CCMRx.OCyM - what a compare match does to the output (17.4.7). THREE
/// bits on this family: the combined, asymmetric and retriggerable modes
/// of the later families do not exist here.
enum class TimOutputMode : uint8_t {
    frozen = 0,
    active_on_match = 1,
    inactive_on_match = 2,
    toggle = 3,
    force_inactive = 4,
    force_active = 5,
    pwm1 = 6,               ///< active while CNT < CCRy
    pwm2 = 7,               ///< the complement of pwm1
};

/// CCMRx.CCyS - a channel is an OUTPUT or an input mapped on one of
/// three sources (17.4.7). `direct` is this channel's own TI, `indirect`
/// its neighbour's - the pair that makes PWM input mode.
enum class TimChannelSelect : uint8_t {
    output = 0,
    direct = 1,
    indirect = 2,
    trc = 3,     ///< the TRC signal (needs the slave controller)
};

/// CCER.CCyP/CCyNP for an INPUT channel: 00 rising, 01 falling, 11 both
/// (17.4.9; 10 is reserved, and 11 must not be used in encoder mode).
enum class TimCapturePolarity : uint8_t { rising = 0, falling = 1, both = 3 };

/// CCMRx.ICyPSC - capture one edge in 1, 2, 4 or 8 (17.4.7).
enum class TimCapturePrescaler : uint8_t { every = 0, every2 = 1, every4 = 2, every8 = 3 };

/**
 * TIMx_DCR.DBA - the DMA burst engine's base, as a WORD OFFSET from
 * TIMx_CR1 (17.4.19's own example). Named rather than computed, because
 * the offsets are the register map's and an application that arithmetics
 * them itself has written a bug no header can catch.
 *
 * The list is the FULL timer's; a smaller instance simply has fewer of
 * these registers implemented, and a burst that walks past its last one
 * reads zero and writes nowhere - the silicon's answer, not this
 * driver's, and `dma_burst()` refuses only what the FIELD cannot hold.
 */
enum class TimBurstBase : uint8_t {
    cr1 = 0, cr2 = 1, smcr = 2, dier = 3, sr = 4, egr = 5,
    ccmr1 = 6, ccmr2 = 7, ccer = 8, cnt = 9, psc = 10, arr = 11,
    rcr = 12, ccr1 = 13, ccr2 = 14, ccr3 = 15, ccr4 = 16, bdtr = 17,
};

/// TIM2_OR.ITR1_RMP (18.4.19) - what TIM2's ITR1 listens to instead of
/// TIM8's TRGO. The Ethernet and USB entries exist where those
/// controllers do; the driver stores the code and does not judge that,
/// the peripherals being other chapters' business.
enum class Tim2Trigger1 : uint8_t {
    tim8_trgo = 0,      ///< the reset value, and Reserved on a part with no TIM8
    eth_ptp = 1,        ///< the Ethernet precision-time-protocol trigger output
    otg_fs_sof = 2,     ///< the USB OTG FS start-of-frame
    otg_hs_sof = 3,     ///< the USB OTG HS start-of-frame
};

/// TIM5_OR.TI4_RMP (18.4.20) - what TIM5's channel 4 captures. Three of
/// the four are INTERNAL, which is what makes an oscillator weighable
/// with no pad (RM0383 6.2.11).
enum class Tim5Input4 : uint8_t {
    pad = 0,            ///< the datasheet's TIM5_CH4 pin
    lsi = 1,            ///< the low-speed internal oscillator
    lse = 2,            ///< the 32768 Hz crystal of the backup domain
    rtc_wakeup = 3,     ///< the RTC wake-up interrupt (which must be enabled)
};

/// TIM11_OR.TI1_RMP (19.5.11) - what TIM11's channel 1 captures. Only
/// code 2 is a source of its own; 0, 1 and 3 all mean the pad.
enum class Tim11Input1 : uint8_t {
    pad = 0,
    hse_rtc = 2,        ///< the HSE divided by RCC_CFGR's RTCPRE
};

/// The time base. `period` is ARR and is REFUSED at zero: 17.4.12 says a
/// null auto-reload leaves the counter blocked, which is a stopped timer
/// wearing the face of a running one.
struct TimConfig {
    uint16_t prescaler = 0;                ///< PSC: the counter clock is TIMxCLK / (PSC + 1)
    uint32_t period = 0xFFFF;              ///< ARR (32-bit on TIM2 and TIM5)
    TimDirection direction = TimDirection::up;
    TimAlignment alignment = TimAlignment::edge;
    TimClockDivision clock_division = TimClockDivision::div1;
    bool auto_reload_preload = false;      ///< CR1.ARPE
    bool one_pulse = false;                ///< CR1.OPM: stop at the next update
    bool update_disable = false;           ///< CR1.UDIS
    bool update_on_overflow_only = false;  ///< CR1.URS: EGR.UG and a slave reset raise no UIF
    uint8_t repetition = 0;                ///< RCR (the advanced-control timers only)
};

/// One capture/compare channel as an OUTPUT.
struct TimChannelConfig {
    TimOutputMode mode = TimOutputMode::frozen;
    uint32_t compare = 0;                  ///< CCRy
    bool preload = true;                   ///< CCMRx.OCyPE - see fact 4 in the file header
    bool fast = false;                     ///< CCMRx.OCyFE
    bool clear_on_ocref_clr = false;       ///< CCMRx.OCyCE (the ES0287 2.6.4 item applies)
    bool active_low = false;               ///< CCER.CCyP
    bool complementary_active_low = false; ///< CCER.CCyNP
    bool enable = true;                    ///< CCER.CCyE
    bool complementary_enable = false;     ///< CCER.CCyNE
    bool idle_high = false;                ///< CR2.OISy  (the advanced-control timers)
    bool complementary_idle_high = false;  ///< CR2.OISyN
};

/// One capture/compare channel as an INPUT.
struct TimCaptureConfig {
    TimChannelSelect select = TimChannelSelect::direct;
    TimCapturePolarity polarity = TimCapturePolarity::rising;
    TimCapturePrescaler prescaler = TimCapturePrescaler::every;
    /// ICyF, 0..15: how many consecutive samples at the rate table 68
    /// gives must agree before an edge is believed. The sampling clock is
    /// tDTS or the timer clock, per the code.
    uint8_t filter = 0;
    bool enable = true;                    ///< CCER.CCyE
};

/// The slave controller, written as one configuration because SMS and TS
/// belong together: 17.4.3's own note says TS must be changed only while
/// the mode is not using it.
struct TimSlaveConfig {
    TimSlaveMode mode = TimSlaveMode::disabled;
    TimTrigger trigger = TimTrigger::itr0;
    /// SMCR.MSM: delay this timer's reaction so cascaded slaves line up
    /// cycle for cycle.
    bool master_slave = false;
};

/// SMCR's external-trigger half (17.4.3): the ETR input's polarity,
/// prescaler and filter, and ECE - external clock mode 2, ETRF as the
/// counter's clock with no trigger selection at all.
struct TimEtrConfig {
    bool inverted = false;     ///< ETP: active low / falling edge
    uint8_t prescaler = 0;     ///< ETPS: 0 off, 1 /2, 2 /4, 3 /8 - 17.4.3 wants ETRP at or below TIMxCLK/4
    uint8_t filter = 0;        ///< ETF 0..15, the same table as a channel's ICxF
    bool clock_mode2 = false;  ///< ECE: the counter clocks on ETRF's rising edges
};

constexpr bool tim_etr_config_valid(const TimEtrConfig& c) {
    return c.prescaler <= 3u && c.filter <= 15u;
}

/// BDTR, the break and dead-time unit of the two timers that have one
/// (17.4.18). LOCK IS ONE-WAY: once written non-zero, the level's
/// registers are read-only until the next peripheral reset - which is
/// why it is spelled here and defaulted to none.
struct TimBreakDeadTime {
    /// DTG, raw: 17.4.18's four ranges, 0xx = DTG x tDTG, 10x =
    /// (64 + DTG[5:0]) x 2, 110 = (32 + DTG[4:0]) x 8, 111 =
    /// (32 + DTG[4:0]) x 16, all in tDTS units. `tim_dead_time_ticks()`
    /// below decodes it and `tim_dead_time_code()` searches for a code.
    uint8_t dead_time = 0;
    bool main_output_enable = true;     ///< MOE - without it no output is driven
    /// AOE: the outputs come back on their own at the next update after a
    /// break. DEFAULTS FALSE because a SYSTEM break (the clock security
    /// system's) is not latched with it set - the first errata item of the
    /// file header, whose own workaround is to leave this clear.
    bool automatic_output_enable = false;
    bool break_enable = false;          ///< BKE
    bool break_active_high = false;     ///< BKP
    bool off_state_run = false;         ///< OSSR
    bool off_state_idle = false;        ///< OSSI
    uint8_t lock = 0;                   ///< LOCK 0..3, ONE-WAY (see above)
};

/// The quadrature interface (17.3.12): which edges are counted, and each
/// input's polarity - which is what "inverted" means in encoder mode, not
/// an edge selection. Both channels are filtered by the same code here
/// because an encoder's two tracks come off one disc.
struct TimEncoderConfig {
    TimSlaveMode mode = TimSlaveMode::encoder3;
    uint8_t filter = 0;             ///< ICxF for both inputs, 0..15
    bool invert_a = false;          ///< CC1P: TI1FP1 inverted
    bool invert_b = false;          ///< CC2P: TI2FP2 inverted
};

/// The dead time a DTG code produces, in tDTS ticks (17.4.18's four
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

/**
 * Which timer feeds ITR0..ITR3 of instance `n`, as an instance NUMBER; 0
 * where there is no such link or where the master is a timer this device
 * has not got.
 *
 * RM0090 tables 94, 98 and 101, RM0390 tables 108, 112 and 115, RM0383
 * tables 49, 53 and 56 - the SAME table, and the only entries the three
 * disagree on are the three that name TIM8, which the F401/F410/F411
 * manuals print as Reserved because those parts have no TIM8. So the
 * table is one table and the presence probe is what removes the TIM8
 * rows: no device name is spelled here.
 *
 * TIM2's ITR1 is the one entry an OPTION REGISTER can move: TIM2_OR's
 * ITR1_RMP points it at the Ethernet or a USB start-of-frame instead
 * (`Tim2Trigger1`), and this table describes its reset value.
 */
constexpr uint8_t tim_internal_trigger(uint8_t n, uint8_t itr) {
    if (itr > 3u) {
        return 0u;
    }
    uint8_t src = 0;
    switch (n) {
        case 1:  src = itr == 0u ? 5u : itr == 1u ? 2u : itr == 2u ? 3u : 4u; break;
        case 2:  src = itr == 0u ? 1u : itr == 1u ? 8u : itr == 2u ? 3u : 4u; break;
        case 3:  src = itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 5u : 4u; break;
        case 4:  src = itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 3u : 8u; break;
        case 5:  src = itr == 0u ? 2u : itr == 1u ? 3u : itr == 2u ? 4u : 8u; break;
        case 8:  src = itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 4u : 5u; break;
        case 9:  src = itr == 0u ? 2u : itr == 1u ? 3u : itr == 2u ? 10u : 11u; break;
        case 12: src = itr == 0u ? 4u : itr == 1u ? 5u : itr == 2u ? 13u : 14u; break;
        default: return 0u;
    }
    return tim_present(src) ? src : 0u;
}

/// Whether that link is the master's OC1 OUTPUT rather than its TRGO -
/// the one-channel timers have no CR2 and so no master mode at all, so
/// what reaches TIM9 and TIM12 is their WAVEFORM, and its pulse must be
/// at least two of the slave's clock cycles wide to be seen.
constexpr bool tim_internal_trigger_is_oc(uint8_t n, uint8_t itr) {
    const uint8_t src = tim_internal_trigger(n, itr);
    return src != 0u && !tim_has_master_mode(src);
}

/// The ITRx index by which slave `n` reaches master `m`, or 0xFF when
/// there is no such link - the direction a caller actually thinks in.
constexpr uint8_t tim_trigger_index_for(uint8_t n, uint8_t master) {
    if (master == 0u) {
        return 0xFFu;
    }
    for (uint8_t i = 0; i < 4u; ++i) {
        if (tim_internal_trigger(n, i) == master) {
            return i;
        }
    }
    return 0xFFu;
}

/**
 * TIMxCLK, the rate a timer's prescaler divides, from a Clock task's own
 * constexpr dividers (fact 5 of the file header):
 *  - TIMPRE clear (the reset state): HCLK when the timer's APB prescaler
 *    is 1, twice PCLK otherwise;
 *  - TIMPRE set: HCLK when that prescaler is 1 or 2, four times PCLK
 *    otherwise.
 * `on_apb2` picks the bus - `Tim<n>::clock_hz(clock)` knows which its
 * instance is on and asks for it.
 */
template <typename Clock>
constexpr uint32_t tim_clock_hz(Clock, bool on_apb2, bool timpre = false) {
    static_assert(Clock::is_static,
                  "brio Tim: a timer counts TIMxCLK and every period, prescaler and "
                  "capture it holds is in those cycles - a DynamicClock would move all "
                  "of them at once and no rebase() can promise the same periods (the "
                  "prescaler's granularity), so the timers take a static clock only. "
                  "A rescaling program keeps its timers on what does not move: the RTC, "
                  "an LPTIM on LSE - docs/design/clock.md");
    const uint8_t div = on_apb2 ? Clock::apb2_div : Clock::apb1_div;
    if (timpre) {
        return div <= 2u ? Clock::hz : (Clock::hz / div) * 4u;
    }
    return div == 1u ? Clock::hz : (Clock::hz / div) * 2u;
}

/**
 * A pad handed to a timer channel.
 *
 *   constexpr brio::PinSel wave{'A', 6, brio::PinFunction::af2};  // TIM3_CH1
 *   using Wave = brio::TimPad<wave>;
 *   Wave::claim();
 *
 * The AF NUMBER IS THE DATASHEET'S (DS10314 table 9 and its twins) and no
 * symbol of the device header can check it - fact 6 of the file header.
 */
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
    /**
     * The pad as a GPIO OUTPUT with the timer's AF nibble already
     * selected: the program drives the level, and MODER decides who else
     * sees it. The EXTI does (it taps the input data path, which is live
     * in every mode but analog) - THE TIMER DOES NOT: the
     * alternate-function input multiplexer is opened by MODER, and a
     * capture channel behind a pad in output mode sees nothing at all
     * (measured). A capture's stimulus on a board with no wire is the
     * timer's OWN output stage instead: `Tim<n>::output_mode()`'s forced
     * modes.
     *
     * The order matters: pin::function() writes AFRL/AFRH, and
     * pin::output() writes MODER without touching the nibble.
     */
    static void drive(bool level = false) {
        pin::function(sel.function);
        pin::output(level);
    }
    static void set() { pin::set(); }
    static void clear() { pin::clear(); }
    static bool read() { return pin::read(); }
    static void release() { pin::release(); }
};

// =============================================================================
// The resource
// =============================================================================

/**
 * Tim<n>: one TIMx block.
 *
 *   using Wave = brio::Tim<3>;
 *   Wave::init();                                  // clock on, reset
 *   Wave::configure({.prescaler = 99, .period = 999});
 *   Wave::output_channel(0, {.mode = brio::TimOutputMode::pwm1,
 *                            .compare = 500});
 *   Wave::enable(true);
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
                  "brio Tim: this device has no such timer (the device header declares "
                  "no TIMn_BASE for it; the F42x/F43x, F446, F405 class, F412, F413 and "
                  "F469 have TIM1..TIM14, the F401 and F411 TIM1..TIM5 and TIM9..TIM11, "
                  "the F410 TIM1, TIM5, TIM6, TIM9 and TIM11)");

    static constexpr uint8_t instance = n;

    // ---- what this instance IS (stm32f4/device_tables.hpp) ------------------
    static constexpr uint8_t counter_bits = tim_counter_bits(n);
    static constexpr uint32_t max_period = tim_max_period(n);
    static constexpr uint8_t channels = tim_channels(n);
    static constexpr uint8_t complementary_channels = tim_complementary_channels(n);
    static constexpr bool has_slave_mode = tim_has_slave_mode(n);
    static constexpr bool has_encoder = tim_has_encoder(n);
    static constexpr bool has_master_mode = tim_has_master_mode(n);
    static constexpr bool has_break = tim_has_break(n);
    static constexpr bool has_repetition = tim_has_repetition(n);
    static constexpr bool has_direction = tim_has_direction(n);
    static constexpr bool has_center_aligned = tim_has_center_aligned(n);
    static constexpr bool has_clock_division = tim_has_clock_division(n);
    static constexpr bool has_external_trigger = tim_has_external_trigger(n);
    static constexpr bool has_ti1_xor = tim_has_ti1_xor(n);
    static constexpr bool has_dma_request = tim_has_dma_request(n);
    static constexpr bool has_dma_burst = tim_has_dma_burst(n);
    static constexpr bool has_option_register = tim_has_option_register(n);
    static constexpr bool has_split_vectors = tim_has_split_vectors(n);
    static constexpr bool on_apb2 = tim_bus_clock(n).apb2;

    /// The four vectors. On a timer with one line all four are the same
    /// enumerator, so a handler can bind them without asking.
    static constexpr IRQn_Type irq() { return tim_irq(n); }
    static constexpr IRQn_Type cc_irq() { return tim_cc_irq(n); }
    static constexpr IRQn_Type break_irq() { return tim_break_irq(n); }
    static constexpr IRQn_Type trigger_irq() { return tim_trigger_irq(n); }

    static TIM_TypeDef& regs() { return *reinterpret_cast<TIM_TypeDef*>(tim_base(n)); }

    // ---- the bus clock and the reset ---------------------------------------
    //
    // A peripheral whose enable bit is clear does not answer register
    // reads at all, so init() opens the gate before it touches anything,
    // and the reset that follows is what makes the state this driver
    // assumes true whatever a bootloader left behind.

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

    // ---- the counter's clock (fact 5) ---------------------------------------

    /// TIMxCLK for this instance, from the clock task's own dividers.
    /// `timpre` is RCC_DCKCFGR's bit, which is clear out of reset.
    template <typename Clock>
    static constexpr uint32_t clock_hz(Clock c, bool timpre = false) {
        return tim_clock_hz(c, on_apb2, timpre);
    }

    /// The same number read back out of the SILICON - RCC_CFGR's own
    /// prescaler field and RCC_DCKCFGR's TIMPRE - given the HCLK the
    /// program believes it runs at. The witness clock_hz() has not got.
    static uint32_t clock_hz_now(uint32_t hclk) {
        const uint8_t div = on_apb2 ? Rcc::apb2_divider() : Rcc::apb1_divider();
        if (rcc_has_timpre() && Rcc::timpre()) {
            return div <= 2u ? hclk : (hclk / div) * 4u;
        }
        return div == 1u ? hclk : (hclk / div) * 2u;
    }

    // ---- the time base (17.4.1, 17.4.11, 17.4.12) ---------------------------

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
        if (c.clock_division != TimClockDivision::div1 && !has_clock_division) {
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
        if constexpr (has_clock_division) {
            cr1 |= (static_cast<uint32_t>(c.clock_division) << TIM_CR1_CKD_Pos) & TIM_CR1_CKD_Msk;
        }
        if (c.auto_reload_preload) { cr1 |= TIM_CR1_ARPE; }
        if (c.one_pulse) { cr1 |= TIM_CR1_OPM; }
        if (c.update_disable) { cr1 |= TIM_CR1_UDIS; }
        if (c.update_on_overflow_only) { cr1 |= TIM_CR1_URS; }
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

    /// The counter. 32 bits on TIM2 and TIM5, 16 everywhere else.
    static uint32_t count() {
        const uint32_t v = regs().CNT;
        return counter_bits == 32u ? v : (v & 0xFFFFu);
    }
    static void set_count(uint32_t v) { regs().CNT = v & max_period; }

    /// CR1.DIR read back - in encoder mode this is the SILICON's answer
    /// to which way the shaft is turning, not the caller's setting.
    static TimDirection direction() {
        return (has_direction && (regs().CR1 & TIM_CR1_DIR) != 0u) ? TimDirection::down
                                                                   : TimDirection::up;
    }

    static uint16_t prescaler() { return static_cast<uint16_t>(regs().PSC); }
    /// PSC takes effect at the next update event, not now (17.4.11).
    static void set_prescaler(uint16_t p) { regs().PSC = p; }

    static uint32_t period() { return regs().ARR & max_period; }
    /// ARR: at once with ARPE clear, at the next update with it set
    /// (17.4.12). Refuses zero and anything past this counter's width.
    static bool set_period(uint32_t v) {
        if (v == 0u || v > max_period) {
            return false;
        }
        regs().ARR = v;
        return true;
    }
    static bool auto_reload_preload() { return (regs().CR1 & TIM_CR1_ARPE) != 0u; }

    /// RCR: one update event in `repetition + 1` counter periods (17.4.13).
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

    /// EGR: raise an event by software (17.4.6). `update()` is the one
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
    /// EGR.COMG: a commutation by software - what transfers the preloaded
    /// channel configuration when CR2.CCPC is set (17.4.6).
    static bool commutation_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().EGR = TIM_EGR_COMG;
            return true;
        }
    }
    /// EGR.BG: a break by software, which is the only break a board with
    /// no wire can raise. It clears MOE exactly as a pad would.
    static bool break_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().EGR = TIM_EGR_BG;
            return true;
        }
    }

    // ---- flags (17.4.5) -----------------------------------------------------
    //
    // rc_w0, see fact 2 of the file header: `SR = ~mask` clears exactly
    // the named flags and cannot swallow one that arrives meanwhile.

    static constexpr uint32_t update_flag = TIM_SR_UIF;
    static constexpr uint32_t trigger_flag = TIM_SR_TIF;
    static constexpr uint32_t break_flag = TIM_SR_BIF;
    static constexpr uint32_t commutation_flag = TIM_SR_COMIF;
    static constexpr uint32_t compare_flag(uint8_t ch) { return TIM_SR_CC1IF << ch; }
    /// CCyOF: a capture arrived while the previous one was still unread.
    /// The NEW value is lost, not the old one (17.4.5).
    static constexpr uint32_t overcapture_flag(uint8_t ch) { return TIM_SR_CC1OF << ch; }
    /// The flags an INTERRUPT can be raised for - TIMx_SR's low byte, at
    /// the very bit positions DIER's enables sit at. The overcapture
    /// flags are NOT among them: they have no enable and no vector, and
    /// are read and cleared by whoever reads the capture.
    static constexpr uint32_t interrupt_flags = TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF |
                                                TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF |
                                                TIM_SR_TIF | TIM_SR_BIF;
    /// Every flag this chapter has, for flags() and clear_flags().
    static constexpr uint32_t all_flags = interrupt_flags | TIM_SR_CC1OF | TIM_SR_CC2OF |
                                          TIM_SR_CC3OF | TIM_SR_CC4OF;

    static uint32_t flags() { return regs().SR; }
    static bool flag(uint32_t mask) { return (regs().SR & mask) != 0u; }
    static void clear_flags(uint32_t mask) { regs().SR = ~mask; }

    /**
     * The flags vector `v` answers for on THIS instance - what a handler
     * hands isr() so that one of four vectors cannot consume another's
     * event. On a timer with one line every group is that one line's, so
     * this is `all_flags` for any of its four enumerators; on TIM1 and
     * TIM8 it is the split RM0090 table 61 makes.
     *
     * Zero for a vector that is not this timer's at all - which is what a
     * SHARED handler (TIM1_BRK_TIM9 and its five siblings) relies on: it
     * calls both owners' bodies and each answers for its own half.
     */
    static constexpr uint32_t vector_flags(IRQn_Type v) {
        if constexpr (!has_split_vectors) {
            return v == tim_irq(n) ? interrupt_flags : 0u;
        } else {
            if (v == tim_irq(n)) {
                return TIM_SR_UIF;
            }
            if (v == tim_cc_irq(n)) {
                return TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF;
            }
            if (v == tim_break_irq(n)) {
                return TIM_SR_BIF;
            }
            if (v == tim_trigger_irq(n)) {
                return TIM_SR_TIF | TIM_SR_COMIF;
            }
            return 0u;
        }
    }

    // ---- interrupts and DMA requests (17.4.4) -------------------------------
    //
    // DIER's interrupt enables sit at the SAME BIT POSITIONS as the SR
    // flags they gate, which is what lets isr() mask one with the other
    // in one AND. Asserted rather than assumed:
    static_assert(TIM_DIER_UIE == TIM_SR_UIF && TIM_DIER_CC1IE == TIM_SR_CC1IF &&
                      TIM_DIER_CC4IE == TIM_SR_CC4IF && TIM_DIER_COMIE == TIM_SR_COMIF &&
                      TIM_DIER_TIE == TIM_SR_TIF && TIM_DIER_BIE == TIM_SR_BIF,
                  "TIMx_DIER's interrupt enables and TIMx_SR's flags must share their "
                  "bit positions (RM0090 17.4.4, 17.4.5) - isr() ANDs them");

    static constexpr uint32_t update_interrupt = TIM_DIER_UIE;
    static constexpr uint32_t trigger_interrupt = TIM_DIER_TIE;
    static constexpr uint32_t break_interrupt = TIM_DIER_BIE;
    static constexpr uint32_t commutation_interrupt = TIM_DIER_COMIE;
    static constexpr uint32_t compare_interrupt(uint8_t ch) { return TIM_DIER_CC1IE << ch; }
    static constexpr uint32_t update_dma = TIM_DIER_UDE;
    static constexpr uint32_t trigger_dma = TIM_DIER_TDE;
    static constexpr uint32_t commutation_dma = TIM_DIER_COMDE;
    static constexpr uint32_t compare_dma(uint8_t ch) { return TIM_DIER_CC1DE << ch; }

    static void interrupts(uint32_t mask, bool on) {
        TIM_TypeDef& t = regs();
        t.DIER = on ? (t.DIER | mask) : (t.DIER & ~mask);
    }
    static uint32_t interrupts() { return regs().DIER; }

    /**
     * The body of one of this timer's vectors.
     *
     * `mask` says which flags this vector answers for: `vector_flags(v)`
     * on an advanced-control timer whose four event groups have four
     * lines, and `interrupt_flags` (the default) on every other, whose
     * one line answers for everything.
     *
     * DIER IS MASKED to its low byte before the AND, and that is not
     * tidiness: DIER's capture/compare DMA enables sit at bits 9..12,
     * exactly where SR's OVERCAPTURE flags do, so a channel with a DMA
     * request armed would otherwise make this body eat a CCyOF that
     * belongs to whoever reads the capture.
     *
     * Returns the flags that were BOTH set and enabled, having cleared
     * exactly those. A flag whose interrupt is not enabled is left
     * standing for a poller to read - which is what makes every
     * measurement in this chapter possible with no handler at all.
     */
    [[gnu::always_inline]] static uint32_t isr(uint32_t mask = interrupt_flags) {
        TIM_TypeDef& t = regs();
        const uint32_t hit = t.SR & (t.DIER & interrupt_flags) & mask;
        if (hit != 0u) {
            t.SR = ~hit;
        }
        return hit;
    }

    // ---- capture/compare channels (17.4.7 .. 17.4.9) ------------------------

    /// CCRy. Reading a CAPTURE channel's register is what clears its
    /// CCyIF (17.4.5), so this verb is the acknowledgement as well as
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

    /**
     * Configure channel `ch` as an OUTPUT. The channel is disabled in
     * CCER first, then CCMR, CCR, CR2's idle levels (where there are any)
     * and CCER are written in that order, so the enable is the last store
     * and the pad is never driven by a half-written configuration.
     *
     * THE DISABLE IS NOT TIDINESS, IT IS 17.4.7's RULE: "CCyS is writable
     * only when the channel is OFF (CCyE = 0)". Without it a channel that
     * is currently an INPUT stays one - the CCyS field is dropped in
     * silence and the timer goes on capturing a pad the caller believes
     * it is driving. Measured, in exactly that shape.
     */
    static bool output_channel(uint8_t ch, const TimChannelConfig& c) {
        if (ch >= channels || c.compare > max_period) {
            return false;
        }
        if (c.complementary_enable && ch >= complementary_channels) {
            return false;
        }
        if (c.clear_on_ocref_clr && !has_external_trigger) {
            return false;   // ocref_clr IS ETRF: no ETR, no clear
        }
        write_ccer(ch, false, false, false, false);

        TIM_TypeDef& t = regs();
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint32_t v = 0;
        v |= (static_cast<uint32_t>(c.mode) & 0x7u) << (shift + 4u);
        if (c.fast) { v |= 1u << (shift + 2u); }
        if (c.preload) { v |= 1u << (shift + 3u); }
        if (c.clear_on_ocref_clr) { v |= 1u << (shift + 7u); }
        write_ccmr(ch, v);

        set_ccr(ch, c.compare);

        if constexpr (has_break) {
            // OISy / OISyN live in CR2 and are write-protected by
            // BDTR.LOCK level 1 and up (17.4.18) - a caller that has
            // locked the timer gets what it locked, not an error.
            const uint32_t ois_shift = 8u + 2u * ch;
            uint32_t cr2 = t.CR2 & ~(0x3u << ois_shift);
            if (c.idle_high) { cr2 |= 1u << ois_shift; }
            if (c.complementary_idle_high && ch < complementary_channels) {
                cr2 |= 2u << ois_shift;
            }
            t.CR2 = cr2;
        }

        write_ccer(ch, c.enable, c.active_low,
                   c.complementary_enable && ch < complementary_channels,
                   c.complementary_active_low);
        return true;
    }

    /// Configure channel `ch` as an INPUT. The channel is disabled in
    /// CCER first, because 17.4.9 makes CCyS writable only then - the
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

    /**
     * CCMRx.OCyM alone, for a channel already configured as an output -
     * the chapter's "forced output mode" (17.3.7) reached in one store.
     *
     * `force_active` and `force_inactive` are what make an output channel
     * A PAD THE PROGRAM DRIVES while the pad stays in alternate function:
     * OCyREF follows the field and not the counter, so the CPU can put a
     * level on the pin through the timer's own output stage. That is the
     * stimulus a capture or an encoder input has on a board with no wire,
     * the GPIO's own output mode not reaching a timer's input.
     */
    static bool output_mode(uint8_t ch, TimOutputMode m) {
        if (ch >= channels) {
            return false;
        }
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        TIM_TypeDef& t = regs();
        volatile uint32_t& r = ch < 2u ? t.CCMR1 : t.CCMR2;
        r = (r & ~(0x7u << (shift + 4u))) |
            ((static_cast<uint32_t>(m) & 0x7u) << (shift + 4u));
        return true;
    }
    static TimOutputMode output_mode(uint8_t ch) {
        if (ch >= channels) {
            return TimOutputMode::frozen;
        }
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        const uint32_t r = ch < 2u ? regs().CCMR1 : regs().CCMR2;
        return static_cast<TimOutputMode>((r >> (shift + 4u)) & 0x7u);
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
    static bool complementary_enabled(uint8_t ch) {
        return ch < complementary_channels &&
               (regs().CCER & (TIM_CCER_CC1NE << (4u * ch))) != 0u;
    }

    // ---- the slave controller and the master output (17.4.2, 17.4.3) --------

    static bool slave(const TimSlaveConfig& c) {
        if constexpr (!has_slave_mode) {
            (void)c;
            return false;
        } else {
            const uint8_t sms = static_cast<uint8_t>(c.mode);
            if (sms > 7u) {
                return false;
            }
            if (sms >= 1u && sms <= 3u && !has_encoder) {
                return false;   // SMS 001..011 are Reserved on TIM9/TIM12
            }
            if (c.trigger == TimTrigger::etr && !has_external_trigger) {
                return false;
            }
            if (c.mode == TimSlaveMode::gated && c.trigger == TimTrigger::ti1_edge) {
                return false;   // 17.4.3's own note: TI1F_ED has no level to gate on
            }
            TIM_TypeDef& t = regs();
            uint32_t v = t.SMCR & ~(TIM_SMCR_SMS_Msk | TIM_SMCR_TS_Msk | TIM_SMCR_MSM);
            v |= static_cast<uint32_t>(sms) << TIM_SMCR_SMS_Pos;
            v |= static_cast<uint32_t>(c.trigger) << TIM_SMCR_TS_Pos;
            if (c.master_slave) { v |= TIM_SMCR_MSM; }
            t.SMCR = v;
            return true;
        }
    }
    static TimSlaveMode slave_mode() {
        if constexpr (!has_slave_mode) {
            return TimSlaveMode::disabled;
        } else {
            return static_cast<TimSlaveMode>((regs().SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
        }
    }
    static TimTrigger slave_trigger() {
        if constexpr (!has_slave_mode) {
            return TimTrigger::itr0;
        } else {
            return static_cast<TimTrigger>((regs().SMCR & TIM_SMCR_TS_Msk) >> TIM_SMCR_TS_Pos);
        }
    }

    /**
     * CR2.MMS - what goes out on TRGO. The errata item the file header
     * calls "TRGO and TRGO2 trigger output failure" is the obligation on
     * this verb: THE SLAVE'S CLOCK MUST ALREADY BE ENABLED before the
     * master starts sending, and must not be changed while it does.
     */
    static bool master(TimMasterMode m) {
        if constexpr (!has_master_mode) {
            (void)m;
            return false;
        } else {
            const uint8_t code = static_cast<uint8_t>(m);
            if (code > 7u) {
                return false;
            }
            if (code >= 3u && (channels == 0u || (code >= 4u && code - 4u >= channels))) {
                return false;   // no OCyREF to publish
            }
            TIM_TypeDef& t = regs();
            t.CR2 = (t.CR2 & ~TIM_CR2_MMS_Msk) |
                    ((static_cast<uint32_t>(code) << TIM_CR2_MMS_Pos) & TIM_CR2_MMS_Msk);
            return true;
        }
    }
    static TimMasterMode master() {
        if constexpr (!has_master_mode) {
            return TimMasterMode::reset;
        } else {
            return static_cast<TimMasterMode>((regs().CR2 & TIM_CR2_MMS_Msk) >> TIM_CR2_MMS_Pos);
        }
    }

    /// CR2.TI1S (17.3.17): TI1 becomes the XOR of the first three channel
    /// inputs - a Hall-sensor interface, and a way to watch three pads on
    /// one capture.
    static bool ti1_xor(bool on) {
        if constexpr (!has_ti1_xor) {
            (void)on;
            return false;
        } else {
            TIM_TypeDef& t = regs();
            t.CR2 = on ? (t.CR2 | TIM_CR2_TI1S) : (t.CR2 & ~TIM_CR2_TI1S);
            return true;
        }
    }
    static bool ti1_xor() { return has_ti1_xor && (regs().CR2 & TIM_CR2_TI1S) != 0u; }

    /// CR2.CCPC and CR2.CCUS (17.4.2): the channel configuration
    /// PRELOADED, transferred on a commutation - the three-phase motor
    /// shape. `on_trigger` also lets the trigger input commutate.
    static bool preload_channels(bool on, bool on_trigger = false) {
        if constexpr (!has_break) {
            (void)on; (void)on_trigger;
            return false;
        } else {
            TIM_TypeDef& t = regs();
            uint32_t v = t.CR2 & ~(TIM_CR2_CCPC | TIM_CR2_CCUS);
            if (on) { v |= TIM_CR2_CCPC; }
            if (on_trigger) { v |= TIM_CR2_CCUS; }
            t.CR2 = v;
            return true;
        }
    }

    /// CR2.CCDS: a capture/compare DMA request is issued on the CC event
    /// (false, the reset state) or on the UPDATE event (true) - 17.4.2.
    static bool compare_dma_on_update(bool on) {
        if constexpr (!has_dma_request) {
            (void)on;
            return false;
        } else {
            TIM_TypeDef& t = regs();
            t.CR2 = on ? (t.CR2 | TIM_CR2_CCDS) : (t.CR2 & ~TIM_CR2_CCDS);
            return true;
        }
    }

    // ---- break and dead time (17.4.18) --------------------------------------

    static bool break_dead_time(const TimBreakDeadTime& c) {
        if constexpr (!has_break) {
            (void)c;
            return false;
        } else {
            if (c.lock > 3u) {
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
            regs().BDTR = v;
            return true;
        }
    }

    /// BDTR.MOE on its own: the master switch every output of an
    /// advanced-control timer passes through. A break clears it in
    /// hardware, and that is the whole of what a break does.
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

    /// The dead time in force, in tDTS ticks (timer clock ticks divided
    /// by CR1.CKD).
    static uint16_t dead_time_ticks() {
        if constexpr (!has_break) {
            return 0;
        } else {
            return tim_dead_time_ticks(static_cast<uint8_t>(regs().BDTR & TIM_BDTR_DTG_Msk));
        }
    }

    // ---- the external trigger input (17.4.3) --------------------------------

    /// ETP / ETPS / ETF / ECE, the SMCR half the slave configuration
    /// leaves alone. Refused on a timer without ETR and for a code
    /// outside its field. 17.4.3: "the external clock frequency must be
    /// at most a quarter of TIMxCLK" AFTER the prescaler - the caller's
    /// arithmetic, since this file does not know the source's rate.
    static bool external_trigger(const TimEtrConfig& c) {
        if constexpr (!has_external_trigger) {
            (void)c;
            return false;
        } else {
            if (!tim_etr_config_valid(c)) {
                return false;
            }
            TIM_TypeDef& t = regs();
            uint32_t v = t.SMCR & ~(TIM_SMCR_ETP | TIM_SMCR_ETPS_Msk | TIM_SMCR_ETF_Msk |
                                    TIM_SMCR_ECE);
            if (c.inverted) { v |= TIM_SMCR_ETP; }
            v |= static_cast<uint32_t>(c.prescaler) << TIM_SMCR_ETPS_Pos;
            v |= static_cast<uint32_t>(c.filter) << TIM_SMCR_ETF_Pos;
            if (c.clock_mode2) { v |= TIM_SMCR_ECE; }
            t.SMCR = v;
            return true;
        }
    }
    static bool external_clock_mode2() {
        return has_external_trigger && (regs().SMCR & TIM_SMCR_ECE) != 0u;
    }

    // ---- the option register (18.4.19, 18.4.20, 19.5.11) --------------------
    //
    // Only three instances have one, and each has ONE field in a
    // different place - so the raw verbs below take the field's value and
    // the three typed ones name the sources. This is where a capture
    // channel is pointed at something that is not a pad at all, which is
    // what makes an oscillator weighable with nothing attached.

    /// TIMx_OR's field, raw. Refused on an instance with no option
    /// register, and for a code past the two bits every one of them has.
    static bool option(uint8_t code) {
        if constexpr (!has_option_register) {
            (void)code;
            return false;
        } else {
            if (code > 3u) {
                return false;
            }
            TIM_TypeDef& t = regs();
            t.OR = (t.OR & ~(0x3UL << option_pos)) |
                   (static_cast<uint32_t>(code) << option_pos);
            return true;
        }
    }
    static uint8_t option() {
        if constexpr (!has_option_register) {
            return 0;
        } else {
            return static_cast<uint8_t>((regs().OR >> option_pos) & 0x3u);
        }
    }

    /// TIM2_OR.ITR1_RMP - what this timer's ITR1 listens to.
    static bool trigger1_source(Tim2Trigger1 s) {
        static_assert(n == 2, "brio Tim: ITR1_RMP is TIM2's option register alone");
        return option(static_cast<uint8_t>(s));
    }
    static Tim2Trigger1 trigger1_source() {
        static_assert(n == 2, "brio Tim: ITR1_RMP is TIM2's option register alone");
        return static_cast<Tim2Trigger1>(option());
    }

    /// TIM5_OR.TI4_RMP - what this timer's channel 4 captures.
    static bool input4_source(Tim5Input4 s) {
        static_assert(n == 5, "brio Tim: TI4_RMP is TIM5's option register alone");
        return option(static_cast<uint8_t>(s));
    }
    static Tim5Input4 input4_source() {
        static_assert(n == 5, "brio Tim: TI4_RMP is TIM5's option register alone");
        return static_cast<Tim5Input4>(option());
    }

    /// TIM11_OR.TI1_RMP - what this timer's channel 1 captures. Codes 0,
    /// 1 and 3 all mean the pad, so the reader answers `pad` for each.
    static bool input1_source(Tim11Input1 s) {
        static_assert(n == 11, "brio Tim: TI1_RMP is TIM11's option register alone");
        return option(static_cast<uint8_t>(s));
    }
    static Tim11Input1 input1_source() {
        static_assert(n == 11, "brio Tim: TI1_RMP is TIM11's option register alone");
        const uint8_t code = option();
        return code == 2u ? Tim11Input1::hse_rtc : Tim11Input1::pad;
    }

    // ---- the DMA burst engine (TIMx_DCR, TIMx_DMAR) -------------------------
    //
    // ONE REQUEST, SEVERAL REGISTERS. The plain requests (DIER's UDE/TDE/
    // CCxDE) move one datum into one register; the burst engine turns each
    // request into a WALK of `length` consecutive registers starting at
    // `base`, all through the single address TIMx_DMAR - so a DMA stream
    // whose peripheral address never changes writes ARR, then CCR1, then
    // CCR2, and starts again at the next update event (17.4.19, 17.4.20).
    //
    // THE BASE IS A WORD OFFSET FROM TIMx_CR1 and not an address, which is
    // why the offsets are named rather than computed.

    /// Point the burst engine at `length` registers starting at `base`.
    /// 17.4.19's DBL is the length MINUS ONE and this argument is the
    /// LENGTH, so nobody has to remember which.
    static bool dma_burst(TimBurstBase base, uint8_t length) {
        if (!has_dma_burst || length < 1u || length > 18u) {
            return false;
        }
        const uint8_t dba = static_cast<uint8_t>(base);
        if (dba > 31u) {
            return false;
        }
        regs().DCR = (static_cast<uint32_t>(dba) << TIM_DCR_DBA_Pos) |
                     (static_cast<uint32_t>(length - 1u) << TIM_DCR_DBL_Pos);
        return true;
    }
    static TimBurstBase burst_base() {
        return static_cast<TimBurstBase>((regs().DCR & TIM_DCR_DBA_Msk) >> TIM_DCR_DBA_Pos);
    }
    static uint8_t burst_length() {
        return static_cast<uint8_t>(((regs().DCR & TIM_DCR_DBL_Msk) >> TIM_DCR_DBL_Pos) + 1u);
    }
    /// The burst engine off: DBL and DBA back to zero, which is CR1 with a
    /// length of one and is also the reset value.
    static void dma_burst_off() { regs().DCR = 0; }

    /// The ONE address a burst stream is pointed at: every access to it
    /// lands on `base + index` and the index is the controller's own.
    /// Null where this timer has no burst engine.
    static volatile void* dmar_address() {
        return has_dma_burst ? static_cast<volatile void*>(&regs().DMAR) : nullptr;
    }
    /// Where a DMA stream writes a duty into, and reads a capture from:
    /// CCRy itself. The address arithmetic is ccr_ref()'s, so a stream and
    /// a CPU write cannot disagree about which register a channel is.
    ///
    /// WHICH STREAM AND CHANNEL serves this timer is RM0090 tables 43 and
    /// 44 - a slice of the request mapping this file does not carry,
    /// because no task here names a DMA engine slot yet.
    static volatile void* ccr_address(uint8_t ch) {
        return ch < channels ? static_cast<volatile void*>(&ccr_ref(ch)) : nullptr;
    }

private:
    /// TIM2_OR bits 11:10, TIM5_OR bits 7:6, TIM11_OR bits 1:0 - the
    /// reserve's, because the header declares each *_Pos macro only where
    /// its instance exists.
    static constexpr uint32_t option_pos = tim_option_pos(n) == 0xFFu ? 0u : tim_option_pos(n);

    static volatile uint32_t& ccr_ref(uint8_t ch) {
        // CCR1..CCR4 are four consecutive words at 0x34 (17.4.14..17).
        return *(&regs().CCR1 + ch);
    }
    static uint32_t ccr(uint8_t ch) { return ccr_ref(ch); }
    static void set_ccr(uint8_t ch, uint32_t v) { ccr_ref(ch) = v; }

    static void write_ccmr(uint8_t ch, uint32_t value) {
        TIM_TypeDef& t = regs();
        const uint32_t mask = 0xFFu << (8u * (ch & 1u));
        if (ch < 2u) {
            t.CCMR1 = (t.CCMR1 & ~mask) | value;
        } else {
            t.CCMR2 = (t.CCMR2 & ~mask) | value;
        }
    }

    /// CCER's four bits of one channel: E, P, NE, NP at 4 x ch. Channel 4
    /// has no NE bit (17.4.9 leaves it reserved), and `ne` is only ever
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
 * TimPwm<T, ch, top>: one PWM output, a `PwmChannel`
 * (util/pwm_channel.hpp) whose `max` is the PERIOD the timer runs at.
 *
 *   using Lamp = brio::TimPwm<brio::Tim<3>, 0, 1000>;
 *   Lamp::setup(99);          // 100 MHz / 100 / 1001 = about 1 kHz
 *   Lamp::duty(250);          // a quarter
 *
 * `top` is a template parameter because PwmChannel requires `max` to be a
 * compile-time constant: a full scale that can move under a generic
 * actuator is not one it can scale against.
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
    /// channel. On an advanced-control timer MOE is raised too, without
    /// which nothing reaches the pad at all (17.4.18) - the trap that
    /// makes a first TIM1 PWM look dead.
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
 * TimPairPwm<T, ch, top>: a channel and its COMPLEMENT, with the dead
 * time the silicon inserts between them - the shape only the two
 * advanced-control timers of this family have (TIM1 and TIM8, channels
 * 1..3).
 *
 * `max` and `duty()` are the PwmChannel contract again, on the pair: the
 * two outputs are one actuator and one duty.
 */
template <class T, uint8_t ch, uint16_t top = 0xFFFF>
struct TimPairPwm {
    TimPairPwm() = delete;
    static_assert(T::has_break,
                  "brio TimPairPwm: this timer has no break/dead-time unit, so it has "
                  "no complementary output either (TIM1 and TIM8 have one; every other "
                  "timer of this family does not)");
    static_assert(ch < T::complementary_channels,
                  "brio TimPairPwm: this channel has no complementary output (channels "
                  "1..3 have one, channel 4 has not)");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    /// `dead_time` is a raw DTG code - `tim_dead_time_code(ticks)` turns a
    /// wanted number of tDTS ticks into one, always rounding UP. tDTS is
    /// TIMxCLK divided by `division`, and it is NOT divided by the
    /// counter's prescaler: the dead time is the same absolute width
    /// whatever the PWM frequency.
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

    static uint16_t dead_time_ticks() { return T::dead_time_ticks(); }
};

/**
 * TimPeriodMeter<T>: the PERIOD and the HIGH TIME of a signal on TI1, in
 * one capture arrangement - the chapter's "PWM input mode" (17.3.6).
 *
 * Two channels watch the same input: channel 1 directly on the rising
 * edge (the period), channel 2 indirectly on the falling one (the width),
 * and the slave controller RESETS the counter on every rising edge so
 * both readings are measured from the same origin. So this needs a timer
 * with a slave controller and two channels - TIM1..TIM5, TIM8, TIM9,
 * TIM12 - and it costs BOTH of them.
 *
 * `period_ticks()` and `width_ticks()` READ the capture registers, which
 * is what clears their flags; a handler that clears CCyIF without reading
 * has thrown the measurement away.
 */
template <class T>
struct TimPeriodMeter {
    TimPeriodMeter() = delete;
    static_assert(T::channels >= 2,
                  "brio TimPeriodMeter: PWM input mode watches one input with TWO "
                  "channels (RM0090 17.3.6)");
    static_assert(T::has_slave_mode,
                  "brio TimPeriodMeter: PWM input mode resets the counter on every "
                  "rising edge, which is the slave controller's job");

    /// `invert` measures the LOW time and the period from falling edges
    /// instead (the whole arrangement mirrored, 17.3.6's own note).
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
    /// so a period reads as its own tick count and not one less.
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
 * single channel (TIM10, TIM11, TIM13, TIM14) can measure, and what a
 * capture ISR hands to a util/meter_sampler.hpp MeterLatch.
 *
 *   using Lse = brio::TimIntervalMeter<brio::Tim<5>, 3>;
 *   extern "C" void TIM5_IRQHandler() {
 *       if (brio::Tim<5>::isr() & Lse::capture_flag) {
 *           if (auto d = Lse::interval()) { Latch::store(*d); }
 *       }
 *   }
 *
 * The subtraction is done in the counter's own modulus, so a wrap between
 * two edges costs nothing as long as the interval is shorter than one
 * full counter period - which is the meter's whole contract and is the
 * caller's to arrange with the prescaler.
 *
 * `interval()` keeps ONE value of state and is meant for the capture
 * handler alone: there is no critical section in it, because the handler
 * is the only writer and the only reader.
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
 * external clock mode 1 on an ITRx link (17.3.15). With the master
 * publishing its update event on TRGO, the count IS the number of master
 * periods, so a frequency is measured with no wire, no pad and no CPU in
 * the path.
 *
 *   TimEventCounter<Tim<3>>::setup(TimTrigger::itr1);   // TIM3 counts TIM2
 *
 * WHICH TIMER ITRx MEANS is per instance: `tim_trigger_index_for(slave,
 * master)` looks the link up so a caller names the MASTER and not a
 * number. The errata obligation is the caller's the other way round: this
 * task enables the SLAVE, so the master must be started after it.
 */
template <class T>
struct TimEventCounter {
    TimEventCounter() = delete;
    static_assert(T::has_slave_mode,
                  "brio TimEventCounter: counting a trigger is the slave controller's "
                  "external clock mode 1");

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
 * trigger is HIGH (slave gated mode, 17.3.15). Pointed at a master whose
 * TRGO is OC1REF - the PWM waveform itself - the count over a known
 * window IS the high time, so a DUTY CYCLE is measured internally, with
 * no pad and no sampling.
 *
 * Gated mode must never be used with TimTrigger::ti1_edge (17.4.3's own
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
 * TimPeriodicTick<T>: the plainest use of a timer - an update event every
 * `period + 1` counter ticks, and an interrupt on it. What the basic
 * timers (TIM6, TIM7) exist for, and what any other timer does with no
 * channel configured.
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
 * on a trigger - CR1.OPM plus a compare channel (17.3.10). The counter
 * stops itself at the next update, so the pulse happens once per trigger
 * and the timer costs nothing between them.
 *
 * `delay` is where the pulse STARTS (the compare value) and the pulse
 * ends at the period, so width = period - delay. The trigger is the
 * caller's: a software start (fire()), a slave trigger mode on an ITRx
 * (arm()), or a TI edge.
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
        // THE COMPARE IS PRELOADED (fact 4 of the file header), and a
        // one-pulse timer sees no update between here and its trigger -
        // so without this the pulse would run against the ACTIVE compare
        // register's old value and come out the whole period wide,
        // measured. The update loads it and its own flag is cleared.
        T::update();
        T::clear_flags(T::update_flag);
        if constexpr (T::has_break) {
            (void)T::main_output(true);
        }
        return true;
    }

    /// Arm the pulse for a trigger that STARTS the counter (slave trigger
    /// mode), or fire it now with fire().
    static bool arm(TimTrigger trigger) {
        return T::slave({.mode = TimSlaveMode::trigger, .trigger = trigger});
    }
    static void fire() {
        T::set_count(0);
        T::enable(true);
    }
    static bool busy() { return T::enabled(); }
};

/**
 * TimEncoder<T>: the quadrature interface (17.3.12) - two channel inputs
 * as the two tracks of an incremental encoder, the counter following the
 * shaft in both directions with no software in the path.
 *
 *   using Knob = brio::TimEncoder<brio::Tim<4>>;
 *   Knob::setup();                      // x4 counting, no filter
 *   const uint32_t at = Knob::count();
 *   const bool backwards = Knob::reversing();
 *
 * `encoder3` (the default) counts BOTH edges of both tracks - four counts
 * per quadrature cycle and the finest resolution; `encoder1` and
 * `encoder2` count one track's edges only. The counter runs modulo ARR in
 * both directions, so `period` is the encoder's own count per revolution
 * where there is one, and the whole 16 or 32 bits otherwise.
 *
 * A quadrature input must never use TimCapturePolarity::both (17.4.9's
 * own note), which is why this task takes an INVERSION per track and not
 * a polarity.
 *
 * TI1FP1 and TI2FP2 come off the pads BEFORE the CCyS multiplexer, so a
 * channel can drive its own pad and be counted on it at the same time:
 * setup() leaves both channels as inputs, and a caller that wants the
 * timer to make its own quadrature puts them back into forced output mode
 * afterwards (`output_channel` then `output_mode`). That is what the
 * reference suite does, having no wire.
 */
template <class T>
struct TimEncoder {
    TimEncoder() = delete;
    static_assert(T::has_encoder,
                  "brio TimEncoder: SMS codes 001..011 are Reserved on this timer - the "
                  "quadrature interface is TIM1..TIM5's and TIM8's");
    static_assert(T::channels >= 2,
                  "brio TimEncoder: an encoder has two tracks and needs two channels");

    static bool setup(const TimEncoderConfig& c = {}, uint32_t period = T::max_period) {
        const uint8_t mode = static_cast<uint8_t>(c.mode);
        if (mode < 1u || mode > 3u || c.filter > 15u) {
            return false;
        }
        if (!T::configure({.prescaler = 0, .period = period})) {
            return false;
        }
        if (!T::capture_channel(0, {.select = TimChannelSelect::direct,
                                    .polarity = c.invert_a ? TimCapturePolarity::falling
                                                           : TimCapturePolarity::rising,
                                    .filter = c.filter})) {
            return false;
        }
        if (!T::capture_channel(1, {.select = TimChannelSelect::direct,
                                    .polarity = c.invert_b ? TimCapturePolarity::falling
                                                           : TimCapturePolarity::rising,
                                    .filter = c.filter})) {
            return false;
        }
        if (!T::slave({.mode = c.mode})) {
            return false;
        }
        T::enable(true);
        return true;
    }

    static uint32_t count() { return T::count(); }
    static void set_count(uint32_t v) { T::set_count(v); }
    /// CR1.DIR as the SILICON keeps it: which way the last edge moved the
    /// counter.
    static bool reversing() { return T::direction() == TimDirection::down; }
};

} // namespace brio
