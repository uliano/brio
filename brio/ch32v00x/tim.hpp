/*
 * tim.hpp
 *
 * The CH32V00x's timers (RM ch. 11, 12, 13) in the two strata every
 * brio target uses (docs/design/overview.md, "Target strata"):
 *
 *  Tim<n>          the RESOURCE, n = 1 or 2 - one TIMx block: the time
 *                  base (prescaler, counter, auto-reload with its
 *                  preload), the counting modes, the four capture/
 *                  compare channels in both their faces, the slave
 *                  controller and the master TRGO, the external
 *                  trigger, the break/dead-time unit (TIM1) or the
 *                  dead-time pairs (TIM2), the flags and the ISR body.
 *
 *  Tim3            the STREAMLINED timer of the CH32V006/007 (ch. 13),
 *                  a different block: a counter with four compare
 *                  registers and NO pad, whose matches are internal
 *                  pulses (the ADC's triggers) and DMA requests.
 *
 *  TimPad<pad>     a pad handed to a timer channel: the default pad of
 *                  the datasheet's table, or the pad a remap column
 *                  gives it (afio.hpp's afio_tim1_pads / afio_tim2_pads,
 *                  with Tim<n>::remap(code) writing the column).
 *
 *  TASKS           TimPwm / TimPairPwm (util/pwm_channel.hpp's
 *                  PwmChannel, one and two outputs), TimPeriodMeter /
 *                  TimIntervalMeter (what util/meter_sampler.hpp's
 *                  MeterLatch is fed from), TimEventCounter /
 *                  TimGatedCounter (one timer measuring another with
 *                  no wire and no CPU), TimPeriodicTick, TimOnePulse.
 *
 * THE LINEAGE, AND WHAT DIFFERS. TIM1 and TIM2 are the STM32F1's
 * advanced and general-purpose timers under WCH's register names
 * (CTLR1, SMCFGR, DMAINTENR, INTFR, SWEVGR, CHCTLRx, CCER, ATRLR,
 * RPTCR, CHxCVR, BDTR, DMACFGR, DMAADR) at the F1's offsets, sixteen
 * bits at a four-byte stride except the four CHxCVR, which are 32
 * bits so that bit 16 can carry the CAPLVL level indication. What the
 * F1 has not: three bits at the top of CTLR1 - CAPLVL (the captured
 * level in bit 16 of CHxCVR under a dual-edge capture), CAPOV (a
 * capture after an overflow reads 0xFFFF) and OE_MODE (what the
 * output pads do when CEN is cleared: hold, or float) - and, on
 * TIM2, DEAD TIME WITHOUT A BREAK UNIT: TIM2_DTCR pairs channel 3
 * with channel 1 and channel 4 with channel 2 as complementary
 * outputs with their own dead-time fields (12.3.9), so a TIM2 pair
 * costs two channels where a TIM1 pair costs one and its OCxN. What
 * the F1 has and this family has not: TIM3 as a copy of TIM2 - here
 * TIM3 is the streamlined block above, on the CH32V006/007 alone.
 *
 * WHAT A TIMER IS comes from the chapters, stated once below as
 * constexpr facts keyed by instance (no vendor header exists in this
 * build to ask): TIM1 has four channels, three complementary outputs,
 * the break unit, the repetition counter and the slave controller;
 * TIM2 has four channels, the two dead-time pairs, the slave
 * controller and no repetition. Every verb that names a feature an
 * instance lacks REFUSES - false, nothing written - rather than store
 * into a reserved bit, and the tasks static_assert on the facts.
 *
 * THE STATUS REGISTER IS WRITE-ZERO-TO-CLEAR (11.4.5's RW0): a flag of
 * INTFR is cleared by writing ZERO to it and a one has no effect, so
 * clearing is `INTFR = ~flags`, a plain store, and a flag that arrives
 * between the read and the store survives. The same discipline as the
 * I2C's STAR1 and the opposite of the DMA's INTFCR.
 *
 * THE PRESCALER AND THE AUTO-RELOAD ARE SHADOWED. PSC is taken at the
 * next update event; ATRLR too under ARPE and at once without. So
 * configure() ends with SWEVGR.UG - a software update that loads both
 * shadows - and clears the UIF it raised.
 *
 * A COMPARE REGISTER IS PRELOADED BY DEFAULT HERE, a choice: OCxPE is
 * clear out of reset, and every output channel this driver configures
 * sets it, because a PwmChannel::duty() that can glitch is not one an
 * actuator above can use. TimChannelConfig::preload = false reaches
 * the unbuffered behaviour.
 *
 * THE CLOCK. CK_INT is HCLK - this family has no APB prescaler, so
 * `tim_clock_hz(clock)` is the Clock's hz, and a static clock only: a
 * DynamicClock would move every period at once and no rebase() can
 * promise the same periods.
 *
 * THE VECTORS. TIM1 has four (break, update, trigger/commutation,
 * capture-compare; Irq::tim1_brk..tim1_cc), TIM2 one (Irq::tim2), TIM3
 * none at all - its events are DMA requests and internal pulses. The
 * ISR body answers for the flags that are both raised and enabled
 * whichever vector it is called from.
 *
 * NOT COVERED YET: the DMA burst (DMACFGR/DMAADR) and the timers' DMA
 * requests as engine sources - born with a stream that needs them;
 * the encoder modes as a task (the resource has the three SMS codes;
 * a task needs an encoder on the desk); the COM event and the
 * commutation preload (CCPC/CCUS) - motor control's, no user; TIM3's
 * matches as ADC triggers - the ADC chapter's.
 */

#pragma once


#include <stdint.h>

#include <optional>

#include "ch32v00x/afio.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// The registers (tables 11-3, 12-3): sixteen bits at a four-byte stride,
// the four compare/capture registers 32 bits wide
// =============================================================================

struct TimRegs {
    volatile uint16_t CTLR1;      ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t CTLR2;      ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t SMCFGR;     ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t DMAINTENR;  ///< 0x0c
    uint16_t RESERVED3;
    volatile uint16_t INTFR;      ///< 0x10 write zero to clear
    uint16_t RESERVED4;
    volatile uint16_t SWEVGR;     ///< 0x14 write only
    uint16_t RESERVED5;
    volatile uint16_t CHCTLR1;    ///< 0x18 channels 1 and 2
    uint16_t RESERVED6;
    volatile uint16_t CHCTLR2;    ///< 0x1c channels 3 and 4
    uint16_t RESERVED7;
    volatile uint16_t CCER;       ///< 0x20
    uint16_t RESERVED8;
    volatile uint16_t CNT;        ///< 0x24
    uint16_t RESERVED9;
    volatile uint16_t PSC;        ///< 0x28
    uint16_t RESERVED10;
    volatile uint16_t ATRLR;      ///< 0x2c
    uint16_t RESERVED11;
    volatile uint16_t RPTCR;      ///< 0x30 TIM1 only
    uint16_t RESERVED12;
    volatile uint32_t CHCVR[4];   ///< 0x34..0x40, bit 16 the CAPLVL level
    volatile uint16_t BDTR;       ///< 0x44 TIM1; TIM2's DTCR sits here
    uint16_t RESERVED13;
    volatile uint16_t DMACFGR;    ///< 0x48
    uint16_t RESERVED14;
    volatile uint16_t DMAADR;     ///< 0x4c
    uint16_t RESERVED15;
};

inline constexpr uint32_t tim1_base = pb2_base + 0x2C00;
inline constexpr uint32_t tim2_base = pb1_base + 0x0000;
inline constexpr uint32_t tim3_base = pb1_base + 0x0800;

constexpr uint32_t tim_base_for(uint8_t n) {
    return n == 1 ? tim1_base : n == 2 ? tim2_base : 0;
}

// CTLR1 (11.4.1)
inline constexpr uint16_t tim_cen     = 1u << 0;
inline constexpr uint16_t tim_udis    = 1u << 1;
inline constexpr uint16_t tim_urs     = 1u << 2;
inline constexpr uint16_t tim_opm     = 1u << 3;
inline constexpr uint16_t tim_dir     = 1u << 4;
inline constexpr uint16_t tim_cms_mask = 3u << 5;
inline constexpr uint16_t tim_arpe    = 1u << 7;
inline constexpr uint16_t tim_ckd_mask = 3u << 8;
inline constexpr uint16_t tim_oe_mode = 1u << 13;
inline constexpr uint16_t tim_capov   = 1u << 14;
inline constexpr uint16_t tim_caplvl  = 1u << 15;
// CTLR2 (11.4.2)
inline constexpr uint16_t tim_ccpc    = 1u << 0;
inline constexpr uint16_t tim_ccus    = 1u << 2;
inline constexpr uint16_t tim_ccds    = 1u << 3;
inline constexpr uint16_t tim_mms_mask = 7u << 4;
inline constexpr uint16_t tim_ti1s    = 1u << 7;
// SMCFGR (11.4.3)
inline constexpr uint16_t tim_sms_mask = 7u << 0;
inline constexpr uint16_t tim_ts_mask  = 7u << 4;
inline constexpr uint16_t tim_msm     = 1u << 7;
inline constexpr uint16_t tim_etf_mask = 15u << 8;
inline constexpr uint16_t tim_etps_mask = 3u << 12;
inline constexpr uint16_t tim_ece     = 1u << 14;
inline constexpr uint16_t tim_etp     = 1u << 15;
// DMAINTENR (11.4.4) and INTFR (11.4.5) share their low positions
inline constexpr uint16_t tim_uie     = 1u << 0;
inline constexpr uint16_t tim_cc1ie   = 1u << 1;
inline constexpr uint16_t tim_comie   = 1u << 5;
inline constexpr uint16_t tim_tie     = 1u << 6;
inline constexpr uint16_t tim_bie     = 1u << 7;
inline constexpr uint16_t tim_ude     = 1u << 8;
inline constexpr uint16_t tim_cc1de   = 1u << 9;
inline constexpr uint16_t tim_comde   = 1u << 13;
inline constexpr uint16_t tim_tde     = 1u << 14;
inline constexpr uint16_t tim_uif     = 1u << 0;
inline constexpr uint16_t tim_cc1if   = 1u << 1;
inline constexpr uint16_t tim_comif   = 1u << 5;
inline constexpr uint16_t tim_tif     = 1u << 6;
inline constexpr uint16_t tim_bif     = 1u << 7;
inline constexpr uint16_t tim_cc1of   = 1u << 9;
// SWEVGR (11.4.6)
inline constexpr uint16_t tim_ug      = 1u << 0;
inline constexpr uint16_t tim_cc1g    = 1u << 1;
inline constexpr uint16_t tim_comg    = 1u << 5;
inline constexpr uint16_t tim_tg      = 1u << 6;
inline constexpr uint16_t tim_bg      = 1u << 7;
// CCER (11.4.9)
inline constexpr uint16_t tim_cc1e    = 1u << 0;
inline constexpr uint16_t tim_cc1p    = 1u << 1;
inline constexpr uint16_t tim_cc1ne   = 1u << 2;
inline constexpr uint16_t tim_cc1np   = 1u << 3;
// BDTR (11.4.18)
inline constexpr uint16_t tim_dtg_mask = 0xFFu;
inline constexpr uint16_t tim_lock_mask = 3u << 8;
inline constexpr uint16_t tim_ossi    = 1u << 10;
inline constexpr uint16_t tim_ossr    = 1u << 11;
inline constexpr uint16_t tim_bke     = 1u << 12;
inline constexpr uint16_t tim_bkp     = 1u << 13;
inline constexpr uint16_t tim_aoe     = 1u << 14;
inline constexpr uint16_t tim_moe     = 1u << 15;
// TIM2_DTCR (12.4.17)
inline constexpr uint16_t tim2_oc1n_en = 1u << 0;
inline constexpr uint16_t tim2_oc2n_en = 1u << 1;
inline constexpr uint16_t tim2_dt1_p   = 1u << 2;
inline constexpr uint16_t tim2_dt1n_p  = 1u << 3;
inline constexpr uint16_t tim2_dt2_p   = 1u << 4;
inline constexpr uint16_t tim2_dt2n_p  = 1u << 5;

// =============================================================================
// The facts, keyed by instance (11.1, 12.2.2)
// =============================================================================

constexpr bool tim_present(uint8_t n) { return n == 1 || n == 2; }
constexpr uint8_t tim_channels(uint8_t n) { return tim_present(n) ? 4u : 0u; }
/// OCxN outputs: TIM1's channels 1..3 through the break unit; TIM2's
/// channels 1 and 2 through DTCR, riding channels 3 and 4.
/// TIM1's three complementary outputs on both parts; TIM2's two pairs
/// under DTCR on the CH32V006 alone (the CH32V003's TIM2 has no
/// dead-time generator - device::tim2_has_dead_time).
constexpr uint8_t tim_complementary_channels(uint8_t n) {
    return n == 1 ? 3u : (n == 2 && device::tim2_has_dead_time) ? 2u : 0u;
}
constexpr bool tim_has_break(uint8_t n) { return n == 1; }
constexpr bool tim_has_repetition(uint8_t n) { return n == 1; }
constexpr bool tim_has_dead_time_pairs(uint8_t n) { return n == 2 && device::tim2_has_dead_time; }
constexpr bool tim_has_slave_mode(uint8_t n) { return tim_present(n); }
constexpr bool tim_has_external_trigger(uint8_t n) { return tim_present(n); }

/// Table 11-2: which timer each ITRx of an instance is wired to (0 =
/// nothing). TIM1's ITR1 is TIM2's TRGO; TIM2's ITR0 is TIM1's.
constexpr uint8_t tim_internal_trigger(uint8_t n, uint8_t itr) {
    if (n == 1 && itr == 1u) { return 2u; }
    if (n == 2 && itr == 0u) { return 1u; }
    return 0u;
}
/// The ITR index of `slave` that carries `master`'s TRGO, 0xFF if none.
constexpr uint8_t tim_trigger_index_for(uint8_t slave, uint8_t master) {
    for (uint8_t i = 0; i < 4u; ++i) {
        if (tim_internal_trigger(slave, i) == master) {
            return i;
        }
    }
    return 0xFFu;
}

// =============================================================================
// Vocabulary
// =============================================================================

enum class TimDirection : uint8_t { up = 0, down = 1 };

/// CTLR1.CMS.
enum class TimAlignment : uint8_t {
    edge = 0,
    center_down = 1,   ///< CCxIF set while down-counting
    center_up = 2,     ///< CCxIF set while up-counting
    center_both = 3,
};

/// CTLR1.CKD: the dead-time and filter sampling clock's divider.
enum class TimClockDivision : uint8_t { div1 = 0, div2 = 1, div4 = 2 };

/// SMCFGR.TS.
enum class TimTrigger : uint8_t {
    itr0 = 0, itr1 = 1, itr2 = 2, itr3 = 3,
    ti1_edge = 4,   ///< TI1F_ED: one pulse per TRANSITION of TI1 (never gate on it)
    ti1 = 5,        ///< TI1FP1
    ti2 = 6,        ///< TI2FP2
    etr = 7,        ///< ETRF
};

/// SMCFGR.SMS.
enum class TimSlaveMode : uint8_t {
    disabled = 0,
    encoder1 = 1,
    encoder2 = 2,
    encoder3 = 3,
    reset = 4,             ///< a trigger edge reinitializes the counter
    gated = 5,             ///< the counter runs while the trigger is HIGH
    trigger = 6,           ///< a trigger edge starts the counter
    external_clock1 = 7,   ///< the trigger's rising edges ARE the counter clock
};

/// CTLR2.MMS: what TRGO carries.
enum class TimMasterMode : uint8_t {
    reset = 0,
    enable = 1,
    update = 2,
    compare_pulse = 3,
    oc1ref = 4,
    oc2ref = 5,
    oc3ref = 6,
    oc4ref = 7,
};

/// CHCTLRx.OCxM.
enum class TimOutputMode : uint8_t {
    frozen = 0,
    active_on_match = 1,
    inactive_on_match = 2,
    toggle = 3,
    force_inactive = 4,
    force_active = 5,
    pwm1 = 6,   ///< active while CNT < CHxCVR (up-counting)
    pwm2 = 7,   ///< the complement of pwm1
};

/// CHCTLRx.CCxS.
enum class TimChannelSelect : uint8_t {
    output = 0,
    direct = 1,     ///< ICx on TIx
    indirect = 2,   ///< ICx on the neighbour's TI (1<->2, 3<->4)
    trc = 3,        ///< the TRC signal (the slave controller's)
};

enum class TimCapturePolarity : uint8_t { rising = 0, falling = 1 };
enum class TimCapturePrescaler : uint8_t { every = 0, every2 = 1, every4 = 2, every8 = 3 };

struct TimConfig {
    uint16_t prescaler = 0;               ///< PSC: the counter clock is HCLK / (PSC + 1)
    uint16_t period = 0xFFFF;             ///< ATRLR
    TimDirection direction = TimDirection::up;
    TimAlignment alignment = TimAlignment::edge;
    TimClockDivision clock_division = TimClockDivision::div1;
    bool auto_reload_preload = false;     ///< ARPE
    bool one_pulse = false;               ///< OPM
    bool update_disable = false;          ///< UDIS
    bool update_on_overflow_only = false; ///< URS
    uint8_t repetition = 0;               ///< RPTCR (TIM1)
    bool capture_level = false;           ///< CAPLVL: bit 16 of CHxCVR carries the captured level
    bool capture_overflow = false;        ///< CAPOV: a capture after an overflow reads 0xFFFF
    bool outputs_float_when_stopped = false;   ///< OE_MODE: the pads float with CEN clear
};

struct TimChannelConfig {
    TimOutputMode mode = TimOutputMode::frozen;
    uint16_t compare = 0;                  ///< CHxCVR
    bool preload = true;                   ///< OCxPE - see the file header
    bool fast = false;                     ///< OCxFE
    bool clear_on_etrf = false;            ///< OCxCE
    bool active_low = false;               ///< CCER.CCxP
    bool complementary_active_low = false; ///< CCER.CCxNP (TIM1) / DTCR.DTxN_P (TIM2)
    bool enable = true;                    ///< CCER.CCxE
    bool complementary_enable = false;     ///< CCER.CCxNE (TIM1) / DTCR.OCxN_EN (TIM2)
    bool idle_high = false;                ///< CTLR2.OISx (TIM1)
    bool complementary_idle_high = false;  ///< CTLR2.OISxN (TIM1)
};

struct TimCaptureConfig {
    TimChannelSelect select = TimChannelSelect::direct;
    TimCapturePolarity polarity = TimCapturePolarity::rising;
    TimCapturePrescaler prescaler = TimCapturePrescaler::every;
    uint8_t filter = 0;                    ///< ICxF 0..15
    bool enable = true;                    ///< CCER.CCxE
};

struct TimSlaveConfig {
    TimSlaveMode mode = TimSlaveMode::disabled;
    TimTrigger trigger = TimTrigger::itr0;
    bool master_slave = false;             ///< MSM
};

struct TimEtrConfig {
    bool inverted = false;   ///< ETP
    uint8_t prescaler = 0;   ///< ETPS: 0 off, 1 /2, 2 /4, 3 /8
    uint8_t filter = 0;      ///< ETF 0..15
    bool clock_mode2 = false;   ///< ECE
};

constexpr bool tim_etr_config_valid(const TimEtrConfig& c) {
    return c.prescaler <= 3u && c.filter <= 15u;
}

/// TIM1_BDTR.
struct TimBreakDeadTime {
    uint8_t dead_time = 0;               ///< DTG, the F1 encoding (tim_dead_time_ticks)
    bool main_output_enable = true;      ///< MOE
    bool automatic_output_enable = false;   ///< AOE
    bool break_enable = false;           ///< BKE
    bool break_active_high = false;      ///< BKP
    bool off_state_run = false;          ///< OSSR
    bool off_state_idle = false;         ///< OSSI
    uint8_t lock = 0;                    ///< LOCK 0..3, ONE-WAY until a reset
};

/// DTG's four ranges, in ticks of the dead-time clock (11.4.18).
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

/// The smallest code whose dead time is at least `ticks`; 0xFF when
/// none reaches it (the top is 1008 ticks).
constexpr uint8_t tim_dead_time_code(uint16_t ticks) {
    for (uint16_t c = 0; c < 256u; ++c) {
        if (tim_dead_time_ticks(static_cast<uint8_t>(c)) >= ticks) {
            return static_cast<uint8_t>(c);
        }
    }
    return 0xFFu;
}

/// TIM2_DTCR: one pair's dead time, 1..16 ticks of the module clock
/// (DTx = ticks - 1), and the two polarities.
struct TimPairDeadTime {
    uint8_t ticks = 1;                   ///< 1..16
    bool active_low = false;             ///< DTx_P
    bool complementary_active_low = false;   ///< DTxN_P
};

constexpr bool tim_pair_dead_time_valid(const TimPairDeadTime& d) {
    return d.ticks >= 1u && d.ticks <= 16u;
}

/// CK_INT is HCLK: no APB prescaler on this family.
template <typename Clock>
constexpr uint32_t tim_clock_hz(Clock) {
    static_assert(Clock::is_static,
                  "brio Tim: a timer counts HCLK and every period, prescaler and capture "
                  "it holds is in those cycles - a DynamicClock would move all of them at "
                  "once and no rebase() can promise the same periods, so the timers take "
                  "a static clock only");
    return Clock::hz;
}

/// A pad handed to a timer channel: the datasheet's default one.
template <Pad pad>
struct TimPad {
    TimPad() = delete;
    static_assert(pad.valid(), "brio TimPad: a pad must be named");

    using pin = Pin<pad.port, pad.pin>;
    static constexpr Pad selection = pad;

    static void claim(PinDrive drive = PinDrive::push_pull) { pin::function(drive); }
    /// An input channel's pad: a plain input (the alternate function of
    /// an input is the pad's input buffer), pulled as asked.
    static void claim_input(PinPull pull = PinPull::none) { pin::input(pull); }
    static void release() { pin::release(); }
};

/// The CH32V006's default timer pads (DS table 2-1-1), for a program
/// that wants the datasheet's own placement without restating it.
namespace tim_default_pads {
inline constexpr Pad tim1_ch1{'D', 2};
inline constexpr Pad tim1_ch2{'A', 1};
inline constexpr Pad tim1_ch3{'C', 3};
inline constexpr Pad tim1_ch4{'C', 4};
inline constexpr Pad tim1_ch1n{'D', 0};
inline constexpr Pad tim1_ch2n{'A', 2};
inline constexpr Pad tim1_ch3n{'D', 1};   ///< SWDIO's pad: a program that claims it loses the probe
inline constexpr Pad tim1_bkin{'C', 2};
inline constexpr Pad tim1_etr{'C', 5};
inline constexpr Pad tim2_ch1_etr{'D', 4};
inline constexpr Pad tim2_ch2{'D', 3};
inline constexpr Pad tim2_ch3{'C', 0};
inline constexpr Pad tim2_ch4{'D', 7};   ///< the reset pin on parts other than the K8
}  // namespace tim_default_pads

// =============================================================================
// Tim<n>: the resource
// =============================================================================

/**
 * Tim<n>: one TIMx block, n = 1 or 2.
 *
 *   using Pwm = brio::Tim<2>;
 *   Pwm::init();                                   // clock on, reset
 *   Pwm::configure({.prescaler = 47, .period = 999});
 *   Pwm::output_channel(0, {.mode = brio::TimOutputMode::pwm1, .compare = 500});
 *   Pwm::enable(true);
 *
 * Every verb that names a feature this instance lacks returns false
 * and writes NOTHING; every verb that names a channel checks it
 * against `channels`. Channels are 0-based here (channel 1 of the
 * chapter is 0), the other strata's convention.
 */
template <uint8_t n>
class Tim {
public:
    Tim() = delete;
    static_assert(tim_present(n), "brio Tim: this family has TIM1 and TIM2 (TIM3 is Tim3, a "
                                  "different block)");

    static constexpr uint8_t instance = n;
    static constexpr uint16_t max_period = 0xFFFF;
    static constexpr uint8_t channels = tim_channels(n);
    static constexpr uint8_t complementary_channels = tim_complementary_channels(n);
    static constexpr bool has_break = tim_has_break(n);
    static constexpr bool has_repetition = tim_has_repetition(n);
    static constexpr bool has_dead_time_pairs = tim_has_dead_time_pairs(n);
    static constexpr bool has_slave_mode = tim_has_slave_mode(n);
    static constexpr bool has_external_trigger = tim_has_external_trigger(n);

    /// TIM1's four vectors, TIM2's one: the update line is the one every
    /// task that ticks binds; the capture-compare line the meters'.
    static constexpr Irq update_irq() { return n == 1 ? Irq::tim1_up : Irq::tim2; }
    static constexpr Irq cc_irq() { return n == 1 ? Irq::tim1_cc : Irq::tim2; }
    static constexpr Irq trigger_irq() { return n == 1 ? Irq::tim1_trg_com : Irq::tim2; }
    static constexpr Irq break_irq() { return n == 1 ? Irq::tim1_brk : Irq::tim2; }

    static TimRegs& regs() { return *reinterpret_cast<TimRegs*>(tim_base_for(n)); }

    static void bus_clock(bool on) {
        if constexpr (n == 1) {
            if (on) { rcc()->PB2PCENR |= rcc_pb2_tim1; } else { rcc()->PB2PCENR &= ~rcc_pb2_tim1; }
        } else {
            if (on) { rcc()->PB1PCENR |= rcc_pb1_tim2; } else { rcc()->PB1PCENR &= ~rcc_pb1_tim2; }
        }
    }
    static bool bus_clock() {
        if constexpr (n == 1) {
            return (rcc()->PB2PCENR & rcc_pb2_tim1) != 0u;
        } else {
            return (rcc()->PB1PCENR & rcc_pb1_tim2) != 0u;
        }
    }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() {
        if constexpr (n == 1) {
            rcc()->PB2PRSTR |= rcc_pb2_tim1;
            rcc()->PB2PRSTR &= ~rcc_pb2_tim1;
        } else {
            rcc()->PB1PRSTR |= rcc_pb1_tim2;
            rcc()->PB1PRSTR &= ~rcc_pb1_tim2;
        }
    }
    static void init() {
        bus_clock(true);
        reset();
    }
    static void release() {
        enable(false);
        reset();
        bus_clock(false);
    }

    /// The whole instance's pads moved to a column of afio.hpp's table
    /// 7-8 (TIM1, ten codes) or 7-9-1 (TIM2, eight): the pads
    /// afio_tim1_pads(code) / afio_tim2_pads(code) name are then the
    /// ones a TimPad claims. Refused for a code the table has not.
    static bool remap(uint8_t code) {
        if constexpr (n == 1) {
            if (code >= afio_tim1_codes) {
                return false;
            }
            Afio::remap_tim1(code);
        } else {
            if (code >= afio_tim2_codes) {
                return false;
            }
            Afio::remap_tim2(code);
        }
        return true;
    }
    static uint8_t remap() { return n == 1 ? Afio::tim1_remap() : Afio::tim2_remap(); }

    // ---- the time base ---------------------------------------------------------

    static constexpr bool config_valid(const TimConfig& c) {
        if (c.period == 0u) {
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

    /// Write the time base and leave the counter STOPPED: PSC and ATRLR
    /// written, CTLR1 assembled in one store, the slave controller back
    /// at its reset (SMCFGR zero: a free-running counter until a verb
    /// after this one says otherwise - a TimPeriodMeter's reset-on-TI1
    /// must not outlive it into the next task's setup on the same timer,
    /// where every interval would read zero), CNT zeroed, then UG loads
    /// both shadows and the UIF it raised is cleared.
    static bool configure(const TimConfig& c) {
        if (!config_valid(c)) {
            return false;
        }
        TimRegs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(t.CTLR1 & ~tim_cen);
        t.SMCFGR = 0;
        t.PSC = c.prescaler;
        t.ATRLR = c.period;
        if constexpr (has_repetition) {
            t.RPTCR = c.repetition;
        }
        uint16_t v = 0;
        if (c.direction == TimDirection::down) { v |= tim_dir; }
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.alignment) << 5);
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.clock_division) << 8);
        if (c.auto_reload_preload) { v |= tim_arpe; }
        if (c.one_pulse) { v |= tim_opm; }
        if (c.update_disable) { v |= tim_udis; }
        if (c.update_on_overflow_only) { v |= tim_urs; }
        if (c.capture_level) { v |= tim_caplvl; }
        if (c.capture_overflow) { v |= tim_capov; }
        if (c.outputs_float_when_stopped) { v |= tim_oe_mode; }
        t.CTLR1 = v;
        t.CNT = 0;
        t.SWEVGR = tim_ug;
        clear_flags(tim_uif);
        return true;
    }

    static void enable(bool on) {
        TimRegs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(on ? (t.CTLR1 | tim_cen) : (t.CTLR1 & ~tim_cen));
    }
    static bool enabled() { return (regs().CTLR1 & tim_cen) != 0u; }
    static uint16_t count() { return regs().CNT; }
    static void set_count(uint16_t v) { regs().CNT = v; }
    static uint16_t prescaler() { return regs().PSC; }
    static void set_prescaler(uint16_t p) { regs().PSC = p; }
    static uint16_t period() { return regs().ATRLR; }
    static bool set_period(uint16_t v) {
        if (v == 0u) {
            return false;
        }
        regs().ATRLR = v;
        return true;
    }
    static bool auto_reload_preload() { return (regs().CTLR1 & tim_arpe) != 0u; }
    static uint8_t repetition() { return has_repetition ? static_cast<uint8_t>(regs().RPTCR) : 0u; }
    static bool set_repetition(uint8_t v) {
        if constexpr (!has_repetition) {
            (void)v;
            return false;
        } else {
            regs().RPTCR = v;
            return true;
        }
    }

    // ---- software events (11.4.6) ----------------------------------------------

    static void update() { regs().SWEVGR = tim_ug; }
    static bool capture_compare_event(uint8_t ch) {
        if (ch >= channels) {
            return false;
        }
        regs().SWEVGR = static_cast<uint16_t>(tim_cc1g << ch);
        return true;
    }
    static void trigger_event() { regs().SWEVGR = tim_tg; }
    static bool commutation_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().SWEVGR = tim_comg;
            return true;
        }
    }
    static bool break_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().SWEVGR = tim_bg;
            return true;
        }
    }

    // ---- flags and interrupts (11.4.4, 11.4.5) ---------------------------------

    static constexpr uint16_t update_flag = tim_uif;
    static constexpr uint16_t trigger_flag = tim_tif;
    static constexpr uint16_t break_flag = tim_bif;
    static constexpr uint16_t commutation_flag = tim_comif;
    static constexpr uint16_t compare_flag(uint8_t ch) { return static_cast<uint16_t>(tim_cc1if << ch); }
    static constexpr uint16_t overcapture_flag(uint8_t ch) { return static_cast<uint16_t>(tim_cc1of << ch); }

    static uint16_t flags() { return regs().INTFR; }
    static bool flag(uint16_t mask) { return (regs().INTFR & mask) != 0u; }
    /// Write-zero-to-clear: a plain store of the complement.
    static void clear_flags(uint16_t mask) { regs().INTFR = static_cast<uint16_t>(~mask); }

    static constexpr uint16_t update_interrupt = tim_uie;
    static constexpr uint16_t trigger_interrupt = tim_tie;
    static constexpr uint16_t break_interrupt = tim_bie;
    static constexpr uint16_t commutation_interrupt = tim_comie;
    static constexpr uint16_t compare_interrupt(uint8_t ch) { return static_cast<uint16_t>(tim_cc1ie << ch); }
    static constexpr uint16_t update_dma = tim_ude;
    static constexpr uint16_t trigger_dma = tim_tde;
    static constexpr uint16_t commutation_dma = tim_comde;
    static constexpr uint16_t compare_dma(uint8_t ch) { return static_cast<uint16_t>(tim_cc1de << ch); }

    static void interrupts(uint16_t mask, bool on) {
        TimRegs& t = regs();
        t.DMAINTENR = static_cast<uint16_t>(on ? (t.DMAINTENR | mask) : (t.DMAINTENR & ~mask));
    }
    static uint16_t interrupts() { return regs().DMAINTENR; }

    /// The ISR body, for any of this timer's vectors: the flags that
    /// were BOTH raised and enabled, cleared - exactly those - and handed
    /// back. A flag whose interrupt is off is left standing for a
    /// poller.
    [[gnu::always_inline]] static uint16_t isr() {
        TimRegs& t = regs();
        const uint16_t enabled_flags = static_cast<uint16_t>(t.DMAINTENR & 0xFFu);
        const uint16_t hit = static_cast<uint16_t>(t.INTFR & enabled_flags);
        if (hit != 0u) {
            t.INTFR = static_cast<uint16_t>(~hit);
        }
        return hit;
    }

    /// The compare/capture register's address, for a DMA engine.
    static volatile void* ccr_address(uint8_t ch) {
        return ch < channels ? static_cast<volatile void*>(&regs().CHCVR[ch]) : nullptr;
    }

    // ---- the channels ----------------------------------------------------------

    static uint16_t compare(uint8_t ch) {
        return ch < channels ? static_cast<uint16_t>(regs().CHCVR[ch] & 0xFFFFu) : uint16_t{0};
    }
    /// The whole 32-bit register: bit 16 is the CAPLVL level indication
    /// under a dual-edge capture (11.4.1).
    static uint32_t capture_raw(uint8_t ch) { return ch < channels ? regs().CHCVR[ch] : 0u; }
    static bool captured_level(uint8_t ch) { return ch < channels && (regs().CHCVR[ch] & (1UL << 16)) != 0u; }
    static bool set_compare(uint8_t ch, uint16_t v) {
        if (ch >= channels) {
            return false;
        }
        regs().CHCVR[ch] = v;
        return true;
    }

    /// The channel as an OUTPUT: mode, preload, polarity, the enables,
    /// the idle states (TIM1). A complementary enable on TIM2 is DTCR's
    /// OCxN_EN, which borrows channel ch + 2's pad (12.3.9).
    static bool output_channel(uint8_t ch, const TimChannelConfig& c) {
        if (ch >= channels) {
            return false;
        }
        if (c.complementary_enable && ch >= complementary_channels) {
            return false;
        }
        TimRegs& t = regs();
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint16_t v = 0;
        v |= static_cast<uint16_t>((static_cast<uint16_t>(c.mode) & 0x7u) << (shift + 4u));
        if (c.fast) { v |= static_cast<uint16_t>(1u << (shift + 2u)); }
        if (c.preload) { v |= static_cast<uint16_t>(1u << (shift + 3u)); }
        if (c.clear_on_etrf) { v |= static_cast<uint16_t>(1u << (shift + 7u)); }
        write_chctlr(ch, v);
        t.CHCVR[ch] = c.compare;
        if constexpr (has_break) {
            const uint8_t ois_shift = static_cast<uint8_t>(8u + 2u * ch);
            uint16_t cr2 = static_cast<uint16_t>(t.CTLR2 & ~(0x3u << ois_shift));
            if (c.idle_high) { cr2 |= static_cast<uint16_t>(1u << ois_shift); }
            if (c.complementary_idle_high && ch < complementary_channels) {
                cr2 |= static_cast<uint16_t>(2u << ois_shift);
            }
            t.CTLR2 = cr2;
            write_ccer(ch, c.enable, c.active_low, c.complementary_enable, c.complementary_active_low);
        } else {
            write_ccer(ch, c.enable, c.active_low, false, false);
            if constexpr (has_dead_time_pairs) {
                if (ch < complementary_channels) {
                    const uint16_t en = ch == 0u ? tim2_oc1n_en : tim2_oc2n_en;
                    const uint16_t np = ch == 0u ? tim2_dt1n_p : tim2_dt2n_p;
                    uint16_t d = static_cast<uint16_t>(t.BDTR & ~(en | np));
                    if (c.complementary_enable) { d |= en; }
                    if (c.complementary_active_low) { d |= np; }
                    t.BDTR = d;
                }
            }
        }
        return true;
    }

    /// The channel as an INPUT: which TI it watches, the edge, the
    /// prescaler, the filter.
    static bool capture_channel(uint8_t ch, const TimCaptureConfig& c) {
        if (ch >= channels || c.select == TimChannelSelect::output || c.filter > 15u) {
            return false;
        }
        write_ccer(ch, false, false, false, false);
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint16_t v = 0;
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.select) << shift);
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.prescaler) << (shift + 2u));
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.filter) << (shift + 4u));
        write_chctlr(ch, v);
        write_ccer(ch, c.enable, c.polarity == TimCapturePolarity::falling, false, false);
        return true;
    }

    static bool channel_enable(uint8_t ch, bool on) {
        if (ch >= channels) {
            return false;
        }
        TimRegs& t = regs();
        const uint16_t bit = static_cast<uint16_t>(tim_cc1e << (4u * ch));
        t.CCER = static_cast<uint16_t>(on ? (t.CCER | bit) : (t.CCER & ~bit));
        return true;
    }
    static bool channel_enabled(uint8_t ch) {
        return ch < channels && (regs().CCER & (tim_cc1e << (4u * ch))) != 0u;
    }
    static bool complementary_enable(uint8_t ch, bool on) {
        if (ch >= complementary_channels) {
            return false;
        }
        TimRegs& t = regs();
        if constexpr (has_break) {
            const uint16_t bit = static_cast<uint16_t>(tim_cc1ne << (4u * ch));
            t.CCER = static_cast<uint16_t>(on ? (t.CCER | bit) : (t.CCER & ~bit));
        } else {
            const uint16_t bit = ch == 0u ? tim2_oc1n_en : tim2_oc2n_en;
            t.BDTR = static_cast<uint16_t>(on ? (t.BDTR | bit) : (t.BDTR & ~bit));
        }
        return true;
    }

    /// TI1S: TI1 as the XOR of the three first inputs (11.4.2).
    static void ti1_xor(bool on) {
        TimRegs& t = regs();
        t.CTLR2 = static_cast<uint16_t>(on ? (t.CTLR2 | tim_ti1s) : (t.CTLR2 & ~tim_ti1s));
    }

    // ---- the slave controller, the master output, the ETR ----------------------

    static bool slave(const TimSlaveConfig& c) {
        if (c.mode == TimSlaveMode::gated && c.trigger == TimTrigger::ti1_edge) {
            return false;   // an edge detector has no level to gate on
        }
        if (static_cast<uint8_t>(c.trigger) < 4u &&
            tim_internal_trigger(n, static_cast<uint8_t>(c.trigger)) == 0u &&
            c.mode != TimSlaveMode::disabled) {
            return false;   // table 11-2: nothing is wired to this ITR
        }
        TimRegs& t = regs();
        uint16_t v = static_cast<uint16_t>(t.SMCFGR & ~(tim_sms_mask | tim_ts_mask | tim_msm));
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.mode) & 0x7u);
        v |= static_cast<uint16_t>((static_cast<uint16_t>(c.trigger) & 0x7u) << 4);
        if (c.master_slave) { v |= tim_msm; }
        t.SMCFGR = v;
        return true;
    }
    static TimSlaveMode slave_mode() { return static_cast<TimSlaveMode>(regs().SMCFGR & tim_sms_mask); }
    static TimTrigger slave_trigger() { return static_cast<TimTrigger>((regs().SMCFGR & tim_ts_mask) >> 4); }

    static bool master(TimMasterMode m) {
        TimRegs& t = regs();
        t.CTLR2 = static_cast<uint16_t>((t.CTLR2 & ~tim_mms_mask) | (static_cast<uint16_t>(m) << 4));
        return true;
    }
    static TimMasterMode master() { return static_cast<TimMasterMode>((regs().CTLR2 & tim_mms_mask) >> 4); }

    static bool external_trigger(const TimEtrConfig& c) {
        if (!tim_etr_config_valid(c)) {
            return false;
        }
        TimRegs& t = regs();
        uint16_t v = static_cast<uint16_t>(t.SMCFGR & ~(tim_etp | tim_etps_mask | tim_etf_mask | tim_ece));
        if (c.inverted) { v |= tim_etp; }
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.prescaler) << 12);
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.filter) << 8);
        if (c.clock_mode2) { v |= tim_ece; }
        t.SMCFGR = v;
        return true;
    }

    // ---- the break unit (TIM1) and the dead-time pairs (TIM2) --------------------

    static bool break_dead_time(const TimBreakDeadTime& c) {
        if constexpr (!has_break) {
            (void)c;
            return false;
        } else {
            if (c.lock > 3u) {
                return false;
            }
            uint16_t v = c.dead_time;
            v |= static_cast<uint16_t>(c.lock << 8);
            if (c.off_state_idle) { v |= tim_ossi; }
            if (c.off_state_run) { v |= tim_ossr; }
            if (c.break_enable) { v |= tim_bke; }
            if (c.break_active_high) { v |= tim_bkp; }
            if (c.automatic_output_enable) { v |= tim_aoe; }
            if (c.main_output_enable) { v |= tim_moe; }
            regs().BDTR = v;
            return true;
        }
    }
    /// MOE: without it no TIM1 output is driven. TIM2 has no such gate
    /// and the verb answers false there.
    static bool main_output(bool on) {
        if constexpr (!has_break) {
            (void)on;
            return false;
        } else {
            TimRegs& t = regs();
            t.BDTR = static_cast<uint16_t>(on ? (t.BDTR | tim_moe) : (t.BDTR & ~tim_moe));
            return true;
        }
    }
    static bool main_output() { return has_break && (regs().BDTR & tim_moe) != 0u; }
    static uint8_t dead_time_code() { return has_break ? static_cast<uint8_t>(regs().BDTR & tim_dtg_mask) : 0u; }

    /// TIM2's DTCR for one pair (pair 0 = channels 1 and 3, pair 1 =
    /// channels 2 and 4): the dead time in module clocks and the two
    /// polarities. Refused on TIM1, whose dead time is BDTR's.
    static bool pair_dead_time(uint8_t pair, const TimPairDeadTime& d) {
        if constexpr (!has_dead_time_pairs) {
            (void)pair; (void)d;
            return false;
        } else {
            if (pair >= complementary_channels || !tim_pair_dead_time_valid(d)) {
                return false;
            }
            TimRegs& t = regs();
            const uint8_t dt_shift = pair == 0u ? 8u : 12u;
            const uint16_t p_bit = pair == 0u ? tim2_dt1_p : tim2_dt2_p;
            const uint16_t np_bit = pair == 0u ? tim2_dt1n_p : tim2_dt2n_p;
            uint16_t v = static_cast<uint16_t>(t.BDTR & ~((0xFu << dt_shift) | p_bit | np_bit));
            v |= static_cast<uint16_t>((d.ticks - 1u) << dt_shift);
            if (d.active_low) { v |= p_bit; }
            if (d.complementary_active_low) { v |= np_bit; }
            t.BDTR = v;
            return true;
        }
    }
    static uint8_t pair_dead_time_ticks(uint8_t pair) {
        if constexpr (!has_dead_time_pairs) {
            (void)pair;
            return 0;
        } else {
            return pair >= complementary_channels
                       ? uint8_t{0}
                       : static_cast<uint8_t>(((regs().BDTR >> (pair == 0u ? 8u : 12u)) & 0xFu) + 1u);
        }
    }

private:
    static void write_chctlr(uint8_t ch, uint16_t value) {
        TimRegs& t = regs();
        const uint16_t mask = static_cast<uint16_t>(0x00FFu << (8u * (ch & 1u)));
        if (ch < 2u) {
            t.CHCTLR1 = static_cast<uint16_t>((t.CHCTLR1 & ~mask) | value);
        } else {
            t.CHCTLR2 = static_cast<uint16_t>((t.CHCTLR2 & ~mask) | value);
        }
    }
    static void write_ccer(uint8_t ch, bool e, bool p, bool ne, bool np) {
        TimRegs& t = regs();
        const uint8_t shift = static_cast<uint8_t>(4u * ch);
        uint16_t v = static_cast<uint16_t>(t.CCER & ~(0xFu << shift));
        if (e) { v |= static_cast<uint16_t>(1u << shift); }
        if (p) { v |= static_cast<uint16_t>(2u << shift); }
        if (ne) { v |= static_cast<uint16_t>(4u << shift); }
        if (np) { v |= static_cast<uint16_t>(8u << shift); }
        t.CCER = v;
    }
};

#if BRIO_CH32_HAS_TIM3
// =============================================================================
// Tim3: the streamlined timer (ch. 13), CH32V006/007
// =============================================================================

struct Tim3Regs {
    volatile uint16_t CTLR1;      ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t DMAINTENR;  ///< 0x04 OCxPE and CC3DE/CC4DE
    uint16_t RESERVED1;
    volatile uint16_t CNT;        ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t ATRLR;      ///< 0x0c
    uint16_t RESERVED3;
    volatile uint16_t CH1CVR;     ///< 0x10
    uint16_t RESERVED4;
    volatile uint16_t CH2CVR;     ///< 0x14
    uint16_t RESERVED5;
    volatile uint16_t CH3CVR;     ///< 0x18
    uint16_t RESERVED6;
    volatile uint16_t CH4CVR;     ///< 0x1c
    uint16_t RESERVED7;
};

/**
 * Tim3: a 16-bit counter with four compare registers and no pad, no
 * prescaler, no capture, no interrupt (13.1, 13.2.3): its matches are
 * internal pulses on channels 1 and 2 (the ADC's triggers) and DMA
 * requests on channels 3 and 4, and its clock is CK_INT or TIM1's
 * trigger (external clock mode 1, SMS = 111). The CTLR1 shape is
 * TIM2's but for the SMS field living in bits 10:8.
 */
struct Tim3 {
    Tim3() = delete;

    static constexpr uint8_t channels = 4;
    static constexpr uint16_t max_period = 0xFFFF;

    static Tim3Regs& regs() { return *reinterpret_cast<Tim3Regs*>(tim3_base); }

    static void bus_clock(bool on) {
        if (on) { rcc()->PB1PCENR |= rcc_pb1_tim3; } else { rcc()->PB1PCENR &= ~rcc_pb1_tim3; }
    }
    static void reset() {
        rcc()->PB1PRSTR |= rcc_pb1_tim3;
        rcc()->PB1PRSTR &= ~rcc_pb1_tim3;
    }
    static void init() {
        bus_clock(true);
        reset();
    }

    struct Config {
        uint16_t period = 0xFFFF;
        TimDirection direction = TimDirection::up;
        TimAlignment alignment = TimAlignment::edge;
        bool auto_reload_preload = false;
        bool update_disable = false;
        /// External clock mode 1: TIM1's trigger is the count clock
        /// (13.2.2.2, table 13-1).
        bool clocked_by_tim1 = false;
    };

    static bool configure(const Config& c) {
        if (c.period == 0u) {
            return false;
        }
        Tim3Regs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(t.CTLR1 & ~tim_cen);
        t.ATRLR = c.period;
        uint16_t v = 0;
        if (c.direction == TimDirection::down) { v |= tim_dir; }
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.alignment) << 5);
        if (c.auto_reload_preload) { v |= tim_arpe; }
        if (c.update_disable) { v |= tim_udis; }
        if (c.clocked_by_tim1) { v |= static_cast<uint16_t>(7u << 8); }
        t.CTLR1 = v;
        t.CNT = 0;
        return true;
    }

    static void enable(bool on) {
        Tim3Regs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(on ? (t.CTLR1 | tim_cen) : (t.CTLR1 & ~tim_cen));
    }
    static bool enabled() { return (regs().CTLR1 & tim_cen) != 0u; }
    static uint16_t count() { return regs().CNT; }
    static void set_count(uint16_t v) { regs().CNT = v; }
    static uint16_t period() { return regs().ATRLR; }

    static uint16_t compare(uint8_t ch) { return ch < channels ? chcvr(ch) : uint16_t{0}; }
    static bool set_compare(uint8_t ch, uint16_t v) {
        if (ch >= channels) {
            return false;
        }
        chcvr(ch) = v;
        return true;
    }
    /// OCxPE: the compare register's preload (13.4.2).
    static bool compare_preload(uint8_t ch, bool on) {
        if (ch >= channels) {
            return false;
        }
        Tim3Regs& t = regs();
        const uint16_t bit = static_cast<uint16_t>(1u << ch);
        t.DMAINTENR = static_cast<uint16_t>(on ? (t.DMAINTENR | bit) : (t.DMAINTENR & ~bit));
        return true;
    }
    /// CC3DE / CC4DE: a DMA request at the match of channel 3 or 4 -
    /// the two channels that reach the DMA (table 8-2: channels 1 and
    /// 4 of the controller on the CH32V006/007).
    static bool dma_request(uint8_t ch, bool on) {
        if (ch != 2u && ch != 3u) {
            return false;
        }
        Tim3Regs& t = regs();
        const uint16_t bit = static_cast<uint16_t>(1u << (9u + ch));
        t.DMAINTENR = static_cast<uint16_t>(on ? (t.DMAINTENR | bit) : (t.DMAINTENR & ~bit));
        return true;
    }

private:
    static volatile uint16_t& chcvr(uint8_t ch) {
        Tim3Regs& t = regs();
        switch (ch) {
            case 0: return t.CH1CVR;
            case 1: return t.CH2CVR;
            case 2: return t.CH3CVR;
            default: return t.CH4CVR;
        }
    }
};
#endif   // BRIO_CH32_HAS_TIM3

// =============================================================================
// The tasks
// =============================================================================

/**
 * TimPwm<T, ch, top>: one PWM output, a `PwmChannel` (util/pwm_channel.hpp)
 * whose `max` is the PERIOD the timer runs at.
 *
 *   using Led = brio::TimPwm<brio::Tim<2>, 0, 1000>;
 *   Led::setup(47);           // 48 MHz / 48 / 1001 = about 1 kHz
 *   Led::duty(250);           // a quarter
 */
template <class T, uint8_t ch, uint16_t top = 0xFFFF>
struct TimPwm {
    TimPwm() = delete;
    static_assert(ch < T::channels, "brio TimPwm: this timer has no such capture/compare channel");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    static bool setup(uint16_t prescaler = 0, TimOutputMode mode = TimOutputMode::pwm1,
                      bool active_low = false) {
        if (!T::configure({.prescaler = prescaler, .period = top, .auto_reload_preload = true})) {
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
    static void duty(uint16_t v) { (void)T::set_compare(ch, v > max ? max : v); }
    static uint16_t duty() { return T::compare(ch); }
};

/**
 * TimPairPwm<T, ch, top>: a channel and its COMPLEMENT with the dead
 * time the silicon inserts between them. On TIM1 the pair is OCx and
 * OCxN under the break unit (channels 1..3); on TIM2 it is channel ch
 * and channel ch + 2 under DTCR (12.3.9), so `setup()` takes the dead
 * time in the unit each has - BDTR's DTG code on TIM1 (tim_dead_time_
 * code), DTCR's 1..16 module clocks on TIM2.
 */
template <class T, uint8_t ch, uint16_t top = 0xFFFF>
struct TimPairPwm {
    TimPairPwm() = delete;
    static_assert(T::has_break || T::has_dead_time_pairs,
                  "brio TimPairPwm: this timer has no complementary output");
    static_assert(ch < T::complementary_channels,
                  "brio TimPairPwm: this channel has no complementary output (TIM1: 1..3, "
                  "TIM2: 1 and 2, whose complements ride channels 3 and 4)");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    static bool setup(uint16_t prescaler = 0, uint16_t dead_time = 0,
                      TimClockDivision division = TimClockDivision::div1) {
        if (!T::configure({.prescaler = prescaler, .period = top, .clock_division = division,
                           .auto_reload_preload = true})) {
            return false;
        }
        if (!T::output_channel(ch, {.mode = TimOutputMode::pwm1, .compare = 0, .complementary_enable = true})) {
            return false;
        }
        if constexpr (T::has_break) {
            if (dead_time > 255u) {
                return false;
            }
            if (!T::break_dead_time({.dead_time = static_cast<uint8_t>(dead_time), .main_output_enable = true})) {
                return false;
            }
        } else {
            if (!T::pair_dead_time(ch, {.ticks = static_cast<uint8_t>(dead_time == 0u ? 1u : dead_time)})) {
                return false;
            }
        }
        T::enable(true);
        return true;
    }
    static void duty(uint16_t v) { (void)T::set_compare(ch, v > max ? max : v); }
    static uint16_t duty() { return T::compare(ch); }
    /// The dead time in force, in ticks of its own clock.
    static uint16_t dead_time_ticks() {
        if constexpr (T::has_break) {
            return tim_dead_time_ticks(T::dead_time_code());
        } else {
            return T::pair_dead_time_ticks(ch);
        }
    }
};

/**
 * TimPeriodMeter<T>: the PERIOD and the HIGH TIME of a signal on TI1 -
 * the chapter's PWM input mode (11.3.4): channel 1 directly on the
 * rising edge, channel 2 indirectly on the falling one, the slave
 * controller resetting the counter on every rising edge. Costs both
 * channels. Reading a capture register is what clears its flag.
 */
template <class T>
struct TimPeriodMeter {
    TimPeriodMeter() = delete;
    static_assert(T::channels >= 2, "brio TimPeriodMeter: PWM input mode watches one input with two channels");

    static bool setup(uint16_t prescaler = 0, uint8_t filter = 0, bool invert = false) {
        if (!T::configure({.prescaler = prescaler, .period = T::max_period})) {
            return false;
        }
        const TimCapturePolarity direct = invert ? TimCapturePolarity::falling : TimCapturePolarity::rising;
        const TimCapturePolarity indirect = invert ? TimCapturePolarity::rising : TimCapturePolarity::falling;
        if (!T::capture_channel(0, {.select = TimChannelSelect::direct, .polarity = direct, .filter = filter})) {
            return false;
        }
        if (!T::capture_channel(1, {.select = TimChannelSelect::indirect, .polarity = indirect, .filter = filter})) {
            return false;
        }
        if (!T::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::ti1})) {
            return false;
        }
        T::enable(true);
        return true;
    }
    static uint16_t period_ticks() { return T::compare(0); }
    static uint16_t width_ticks() { return T::compare(1); }
    static constexpr uint16_t period_flag = T::compare_flag(0);
    static constexpr uint16_t width_flag = T::compare_flag(1);
    static constexpr uint16_t overrun_flag = T::overcapture_flag(0);
    static constexpr uint16_t period_interrupt = T::compare_interrupt(0);
};

/**
 * TimIntervalMeter<T, ch>: the interval between consecutive edges of
 * one input on ONE channel and a free-running counter - what a capture
 * ISR hands to a util/meter_sampler.hpp MeterLatch. The subtraction is
 * in the counter's modulus; `interval()` keeps one value of state and
 * is the capture handler's alone.
 */
template <class T, uint8_t ch = 0>
struct TimIntervalMeter {
    TimIntervalMeter() = delete;
    static_assert(ch < T::channels, "brio TimIntervalMeter: this timer has no such capture channel");

    static bool setup(uint16_t prescaler = 0, uint8_t filter = 0,
                      TimCapturePolarity polarity = TimCapturePolarity::rising,
                      TimCapturePrescaler divider = TimCapturePrescaler::every) {
        have_last_ = false;
        if (!T::configure({.prescaler = prescaler, .period = T::max_period})) {
            return false;
        }
        if (!T::capture_channel(ch, {.select = TimChannelSelect::direct, .polarity = polarity,
                                     .prescaler = divider, .filter = filter})) {
            return false;
        }
        T::enable(true);
        return true;
    }
    static std::optional<uint16_t> interval() {
        const uint16_t now = T::compare(ch);
        const uint16_t last = last_;
        const bool had = have_last_;
        last_ = now;
        have_last_ = true;
        if (!had) {
            return std::nullopt;
        }
        return static_cast<uint16_t>(now - last);
    }
    static void restart() { have_last_ = false; }
    static constexpr uint16_t capture_flag = T::compare_flag(ch);
    static constexpr uint16_t overrun_flag = T::overcapture_flag(ch);
    static constexpr uint16_t capture_interrupt = T::compare_interrupt(ch);

private:
    static inline uint16_t last_ = 0;
    static inline bool have_last_ = false;
};

/**
 * TimEventCounter<T>: a timer whose CLOCK is another timer's trigger -
 * external clock mode 1 on the one ITR link each instance has (table
 * 11-2: TIM2 counts TIM1 on ITR0, TIM1 counts TIM2 on ITR1). With the
 * master publishing its update on TRGO, the count IS the number of
 * master periods: a frequency measured with no wire and no CPU.
 */
template <class T>
struct TimEventCounter {
    TimEventCounter() = delete;

    static bool setup(TimTrigger trigger, uint16_t period = T::max_period) {
        if (!T::configure({.prescaler = 0, .period = period})) {
            return false;
        }
        if (!T::slave({.mode = TimSlaveMode::external_clock1, .trigger = trigger})) {
            return false;
        }
        T::enable(true);
        return true;
    }
    static uint16_t count() { return T::count(); }
    static void restart() { T::set_count(0); }
};

/**
 * TimGatedCounter<T>: a timer that counts ITS OWN clock only while the
 * trigger is HIGH (slave gated mode). Pointed at a master whose TRGO
 * is OCxREF - the PWM waveform itself - the count over a known window
 * IS the high time: a duty cycle measured with no pad.
 */
template <class T>
struct TimGatedCounter {
    TimGatedCounter() = delete;

    static bool setup(TimTrigger trigger, uint16_t prescaler = 0, uint16_t period = T::max_period) {
        if (!T::configure({.prescaler = prescaler, .period = period})) {
            return false;
        }
        if (!T::slave({.mode = TimSlaveMode::gated, .trigger = trigger})) {
            return false;
        }
        T::enable(true);
        return true;
    }
    static uint16_t count() { return T::count(); }
    static void restart() { T::set_count(0); }
};

/**
 * TimPeriodicTick<T>: an update event every `period + 1` counter ticks
 * and an interrupt on it.
 */
template <class T>
struct TimPeriodicTick {
    TimPeriodicTick() = delete;

    static bool setup(uint16_t prescaler, uint16_t period, bool interrupt = true) {
        if (!T::configure({.prescaler = prescaler, .period = period})) {
            return false;
        }
        T::interrupts(T::update_interrupt, interrupt);
        T::enable(true);
        return true;
    }
    static void stop() { T::enable(false); }
    static constexpr uint16_t flag = T::update_flag;
};

/**
 * TimOnePulse<T, ch>: one pulse of a chosen width after a chosen delay,
 * on a trigger - OPM plus a compare channel in PWM mode 2 (11.3.8).
 * `delay` is where the pulse starts and it ends at the period. The
 * trigger is the caller's: T::enable(true), a slave trigger mode, or a
 * TI edge.
 */
template <class T, uint8_t ch = 0>
struct TimOnePulse {
    TimOnePulse() = delete;
    static_assert(ch < T::channels, "brio TimOnePulse: this timer has no such capture/compare channel");

    static bool setup(uint16_t prescaler, uint16_t delay, uint16_t width, bool active_low = false) {
        if (width == 0u || static_cast<uint32_t>(delay) + width > T::max_period) {
            return false;
        }
        if (!T::configure({.prescaler = prescaler, .period = static_cast<uint16_t>(delay + width),
                           .one_pulse = true})) {
            return false;
        }
        if (!T::output_channel(ch, {.mode = TimOutputMode::pwm2, .compare = delay, .active_low = active_low})) {
            return false;
        }
        if constexpr (T::has_break) {
            (void)T::main_output(true);
        }
        // configure() issued its UG before the channel existed, and the
        // compare is preloaded (this file's default): without one more
        // update here the shadow compare is still 0 when the trigger
        // comes, PWM mode 2 is active from the first count, and the pad
        // rises at the trigger and falls at the period - a pulse of
        // delay + width + 1 (measured before this line existed).
        T::update();
        return true;
    }
    static void fire() { T::enable(true); }
    static bool busy() { return T::enabled(); }
};

// =============================================================================
// The chapters' arithmetic, pinned at compile time
// =============================================================================

static_assert(tim_dead_time_ticks(0x7F) == 127u && tim_dead_time_ticks(0x80) == 128u &&
              tim_dead_time_ticks(0xBF) == 254u && tim_dead_time_ticks(0xC0) == 256u &&
              tim_dead_time_ticks(0xDF) == 504u && tim_dead_time_ticks(0xE0) == 512u &&
              tim_dead_time_ticks(0xFF) == 1008u);
static_assert(tim_dead_time_code(100) == 100u && tim_dead_time_code(130) == 0x81u &&
              tim_dead_time_code(1008) == 0xFFu && tim_dead_time_code(1009) == 0xFFu);
static_assert(tim_internal_trigger(2, 0) == 1u && tim_internal_trigger(1, 1) == 2u &&
              tim_internal_trigger(1, 0) == 0u);
static_assert(tim_trigger_index_for(2, 1) == 0u && tim_trigger_index_for(1, 2) == 1u &&
              tim_trigger_index_for(1, 1) == 0xFFu);
static_assert(tim_complementary_channels(1) == 3u && tim_complementary_channels(2) == (device::tim2_has_dead_time ? 2u : 0u));
static_assert(!tim_pair_dead_time_valid({.ticks = 0}) && tim_pair_dead_time_valid({.ticks = 16}) &&
              !tim_pair_dead_time_valid({.ticks = 17}));
static_assert(sizeof(TimRegs) == 0x50 && offsetof(TimRegs, CHCVR) == 0x34 && offsetof(TimRegs, BDTR) == 0x44);

} // namespace brio
