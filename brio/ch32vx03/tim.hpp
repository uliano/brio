/*
 * tim.hpp
 *
 * The timers of the CH32V203 and the CH32V303 (RM ch. 14 for the
 * advanced-control block, ch. 15 for the general-purpose ones, ch. 16 for
 * the basic ones) in the two strata every brio target uses
 * (docs/design/overview.md, "Target strata"):
 *
 *  Tim<n>          the RESOURCE - one TIMx block: the time base
 *                  (prescaler, counter, auto-reload and their shadow
 *                  registers), the counting modes, the four
 *                  capture/compare channels in both their faces, the
 *                  slave controller and the master TRGO, the external
 *                  trigger, the break and dead-time unit of the
 *                  advanced timers, the dual-edge capture of the
 *                  CH32V303, the DMA burst engine, the flags and the ISR
 *                  bodies.
 *
 *  TimPad<pad>     a pad handed to a timer signal, its Pad taken from
 *                  afio.hpp's remap COLUMN (below).
 *
 *  TASKS           TimPwm / TimPairPwm (util/pwm_channel.hpp's
 *                  PwmChannel, one output and the complementary pair),
 *                  TimPeriodMeter / TimIntervalMeter (what a capture
 *                  body hands util/meter_sampler.hpp's MeterLatch),
 *                  TimEventCounter / TimGatedCounter (one timer
 *                  measuring another with no wire and no CPU),
 *                  TimPeriodicTick, TimOnePulse, TimEncoder.
 *
 * WHAT A PART HAS. The reference manual covers four families at once and
 * its timer chapters name TIM1/8/9/10 (advanced), TIM2/3/4/5
 * (general-purpose) and TIM6/7 (basic). The datasheets' tables 2-1 and
 * 2-1-1 are what say which of them a PART offers: every CH32V203 and the
 * two 128 KB CH32V303 have ONE advanced-control timer (TIM1) and THREE
 * general-purpose ones (TIM2..TIM4), the CH32V203RB a fourth - the
 * 32-bit TIM5 - and the CH32V303RC and VC the whole set: FOUR advanced
 * ones (TIM1, TIM8, TIM9, TIM10), FOUR general-purpose ones (TIM2..TIM5,
 * TIM5 sixteen bits there) and the TWO BASIC ones of chapter 16 (TIM6,
 * TIM7), which no CH32V203 has. Those facts are the part table's MASKS
 * (`device::advanced_timer_instances`, `general_timer_instances`,
 * `basic_timer_instances`), read by `tim_present()` and its siblings; no
 * file of this stratum but parts/ names a part.
 *
 * A BASIC TIMER IS A TIME BASE AND NOTHING ELSE (16.2.2): no channel, no
 * slave controller, no external trigger, no down-counting, no burst
 * engine - CTLR1, CTLR2's MMS, UIE/UDE, UIF, UG, CNT, PSC and ATRLR are
 * its whole register file (16.4), and its TRGO goes to the DAC and, on
 * this family, to TIM9's internal triggers. So `Tim<6>` and `Tim<7>` say
 * `channels == 0`, and a channel verb on one of them does not compile:
 * the registers it would write are not there.
 *
 * THE BLOCK IS THE STM32F1's, NOT THE CH32V00x's. Same register file,
 * same bit positions, WCH's names (CTLR1, SMCFGR, DMAINTENR, INTFR,
 * SWEVGR, CHCTLR1/2, CCER, ATRLR, RPTCR, CHxCVR, BDTR, DMACFGR). What
 * the sister family added to it - TIM2's own dead-time pairs in DTCR,
 * the CAPLVL/CAPOV bits that carry a capture's level, the OE_MODE bit -
 * IS NOT HERE: CTLR1 of this family ends at CKD, and a program ported
 * from the CH32V00x that wrote bits 13..15 of CTLR1 would write nothing
 * at all. What the manual's own chapter adds for the CH32V30x_D8 is the
 * DUAL-EDGE CAPTURE register TIMx_AUX (14.3.11, 15.3.9, 14.4.21): one
 * channel capturing both edges of its input and holding the pulse WIDTH
 * in its own register, on channels 2..4 of every timer but the basic
 * ones - and only "for lot numbers where the penultimate sixth bit is not
 * zero", so it is the class's verb and the silicon's answer:
 * `dual_edge_capture()` writes the channel's bit into TIMx_AUX, reads it
 * back and answers false, writing nothing else, where the lot kept no
 * bit - which is what the CH32V303VCT6 the reference suite ran on did.
 *
 * SIX FACTS THAT SHAPE THIS FILE.
 *
 * 1. THE FLAG REGISTER IS rc_w0. A flag of TIMx_INTFR is cleared by
 *    writing ZERO to it and a write of one has no effect (14.4.5), so
 *    clearing is `INTFR = ~flags` - a plain store, no read-modify-write,
 *    and a flag that arrives between a read and the store SURVIVES.
 *    Every other flag register of this stratum - the EXTI's INTFR, the
 *    RCC's INTR - is cleared by writing ONES, so this is the one place
 *    where that reflex is wrong.
 *
 * 2. THE PRESCALER AND THE AUTO-RELOAD ARE SHADOWED. PSC is copied into
 *    the working register at the next UPDATE event and never before
 *    (14.2.3); ATRLR is too when CTLR1.ARPE is set, and is taken at once
 *    when it is clear. So a configuration is only in force after an
 *    update, which is why configure() ends with SWEVGR.UG - a software
 *    update that loads both shadows - and then clears the UIF that
 *    update raised.
 *
 * 3. A COMPARE REGISTER IS PRELOADED BY DEFAULT HERE, AND THAT IS A
 *    CHOICE. OCyPE is clear out of reset (a write to CHxCVR acts at
 *    once, which can produce a runt pulse if it lands past the current
 *    count); every output channel this driver configures sets it,
 *    because a PwmChannel::duty() that can glitch is not one an actuator
 *    above can use. 14.3.5's own PWM recipe asks for it too. The
 *    unbuffered behaviour stays reachable - TimChannelConfig::preload =
 *    false.
 *
 * 4. CCyS IS WRITABLE ONLY WITH THE CHANNEL OFF (14.4.7's own note).
 *    output_channel() and capture_channel() therefore clear the
 *    channel's CCER bits BEFORE they write CHCTLRx: without that a
 *    channel that is currently an INPUT stays one, the CCyS field being
 *    dropped in silence, and the timer goes on capturing a pad the
 *    caller believes it is driving.
 *
 * 5. THE COUNTER'S CLOCK IS NOT THE BUS CLOCK. RM 3.3.1's own rule: a
 *    timer counts its bus clock when that bus's prescaler is 1 and TWICE
 *    it otherwise. The clock task states both numbers (`timclk1_hz`,
 *    `timclk2_hz`), so `Tim<n>::clock_hz(clock)` folds at compile time,
 *    and `clock_hz_now(hclk)` reads the same number back out of RCC -
 *    the witness that stops this file from lying if a prescaler moves.
 *
 * 6. THERE IS NO BOTH-EDGE CODE IN CCER. CCER carries CCyP for the
 *    capture polarity and nothing else: CCyNP is the COMPLEMENTARY
 *    OUTPUT's polarity on the advanced timers (14.4.9) and Reserved on
 *    the general-purpose ones (15.4.9). The later STM32 families' "11 =
 *    both edges" encoding does not exist here, so TimCapturePolarity has
 *    two values and a caller that wants both edges uses two channels on
 *    one input (TimPeriodMeter's arrangement) - or, on the CH32V303, the
 *    dual-edge capture above, which is a different register and not a
 *    CCER code.
 *
 * MEASURING WITH NO WIRE. Three of this chapter's own features make a
 * timer observable with nothing attached, which is what the reference
 * suite is built on:
 *  - THE INTERNAL TRIGGERS. Every timer here publishes TRGO and listens
 *    on ITR0..ITR3; which timer each ITRx IS comes from tables 14-2 and
 *    15-2, reproduced by `tim_internal_trigger()`. A master publishing
 *    its update event and a slave in external clock mode 1 count each
 *    other exactly, with no pad in the path.
 *  - A PAD THE TIMER ITSELF DRIVES. Forced output mode (14.3.3) puts
 *    OCyREF under CHCTLRx instead of under the counter, so the CPU
 *    drives a level onto the pin through the timer's own output stage -
 *    `output_mode(ch, force_active/force_inactive)` - and the channel
 *    inputs see it, a channel's input path being live whatever CCyS
 *    says. IT STOPS WORKING IN AN ENCODER MODE: measured with one
 *    variable, the same channel and the same forced mode drive the pad
 *    while SMS is zero and leave it low while SMS names an encoder
 *    mode, so this family's quadrature interface cannot be fed by the
 *    timer that reads it.
 *  - A PAD THE PORT DRIVES, which on this family reaches a timer's
 *    input too: sixteen edges written on a pad in PLAIN OUTPUT mode,
 *    with no alternate-function nibble at all, are sixteen captures
 *    (measured; the same experiment on the STM32F4 gives zero, its
 *    alternate-function input multiplexer being opened by the mode
 *    register). The alternate-function INPUT here is the pad's own
 *    input buffer (10.2.4), so the CPU can stimulate a capture, an
 *    external clock or a break input with a GPIO store - which is what
 *    the encoder's two tracks are driven with.
 *
 * THE PADS ARE REMAP COLUMNS, NOT AF NUMBERS. On this family a
 * peripheral's pads move as a COLUMN of the chapter's table (afio.hpp),
 * so a timer's pad is not a per-pin selector: `Tim<n>::remap(code)`
 * writes the column and `tim_channel_pad(n, code, ch)` says which pad
 * carries a signal in it. Both come from afio.hpp's tables - the one
 * source that also knows which columns this PACKAGE bonds and which
 * belong to another device class. A pad is then claimed with
 * `TimPad<pad>`: an output channel wants the alternate-function push-
 * pull nibble, a capture wants a plain input (on this family the
 * alternate-function INPUT is the pad's own input buffer, so an input
 * channel needs no AF nibble at all - 10.2.4).
 *
 * NO ERRATA SHEET EXISTS FOR THIS FAMILY (docs/ch32vx03/vendor/
 * README.md): what the bench finds is the errata list, and
 * docs/ch32vx03/tim.md carries it.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// The registers (tables 14-3..14-6, 15-3..15-6, 16-1, 16-2): sixteen-bit
// registers at a four-byte stride, with the counter, the auto-reload and
// the four capture/compare registers declared thirty-two bits wide -
// which is what they are on the CH32V203RB's 32-bit TIM5 and what their
// upper half reads as zero on every other instance. A basic timer has
// the registers up to ATRLR but for SMCFGR and the channels' three; the
// rest of its map is not there.
// =============================================================================

struct TimRegs {
    volatile uint16_t CTLR1;      ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t CTLR2;      ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t SMCFGR;     ///< 0x08 the slave controller and the external trigger
    uint16_t RESERVED2;
    volatile uint16_t DMAINTENR;  ///< 0x0c the interrupt and DMA request enables
    uint16_t RESERVED3;
    volatile uint16_t INTFR;      ///< 0x10 the flags - WRITE ZERO to clear (fact 1)
    uint16_t RESERVED4;
    volatile uint16_t SWEVGR;     ///< 0x14 write-only: raise an event by software
    uint16_t RESERVED5;
    volatile uint16_t CHCTLR1;    ///< 0x18 channels 1 and 2, in both their faces
    uint16_t RESERVED6;
    volatile uint16_t CHCTLR2;    ///< 0x1c channels 3 and 4
    uint16_t RESERVED7;
    volatile uint16_t CCER;       ///< 0x20 the enables and polarities
    uint16_t RESERVED8;
    volatile uint32_t CNT;        ///< 0x24
    volatile uint16_t PSC;        ///< 0x28
    uint16_t RESERVED9;
    volatile uint32_t ATRLR;      ///< 0x2c
    volatile uint16_t RPTCR;      ///< 0x30 the advanced timer's alone
    uint16_t RESERVED10;
    volatile uint32_t CHCVR[4];   ///< 0x34..0x40
    volatile uint16_t BDTR;       ///< 0x44 the advanced timer's alone
    uint16_t RESERVED11;
    volatile uint16_t DMACFGR;    ///< 0x48 the burst engine's base and length
    uint16_t RESERVED12;
    volatile uint16_t DMAADR;     ///< 0x4c the one address a burst walks through
    uint16_t RESERVED13;
    volatile uint16_t AUX;        ///< 0x50 the dual-edge capture (the CH32V30x_D8)
    uint16_t RESERVED14;
};

/// The instances this family addresses (tables 14-3..14-6, 15-3..15-6,
/// 16-1, 16-2). The four advanced-control timers are PB2's; the rest are
/// PB1's. WHICH of them a PART offers is `tim_present()`; TIM6..TIM10's
/// addresses are device.hpp's, where the CH32V303's blocks are named.
inline constexpr uint32_t tim1_base = pb2_base + 0x2c00;
inline constexpr uint32_t tim2_base = pb1_base + 0x0000;
inline constexpr uint32_t tim3_base = pb1_base + 0x0400;
inline constexpr uint32_t tim4_base = pb1_base + 0x0800;
inline constexpr uint32_t tim5_base = pb1_base + 0x0c00;

constexpr uint32_t tim_base_for(uint8_t n) {
    return n == 1    ? tim1_base
           : n == 2  ? tim2_base
           : n == 3  ? tim3_base
           : n == 4  ? tim4_base
           : n == 5  ? tim5_base
           : n == 6  ? tim6_base
           : n == 7  ? tim7_base
           : n == 8  ? tim8_base
           : n == 9  ? tim9_base
           : n == 10 ? tim10_base
                     : 0;
}

// ---- TIMx_CTLR1 (14.4.1, 15.4.1) -------------------------------------------
inline constexpr uint16_t tim_cen       = 1u << 0;
inline constexpr uint16_t tim_udis      = 1u << 1;
inline constexpr uint16_t tim_urs       = 1u << 2;
inline constexpr uint16_t tim_opm       = 1u << 3;
inline constexpr uint16_t tim_dir       = 1u << 4;
inline constexpr uint16_t tim_cms_mask  = 3u << 5;
inline constexpr uint8_t tim_cms_shift  = 5;
inline constexpr uint16_t tim_arpe      = 1u << 7;
inline constexpr uint16_t tim_ckd_mask  = 3u << 8;
inline constexpr uint8_t tim_ckd_shift  = 8;

// ---- TIMx_CTLR2 (14.4.2, 15.4.2) -------------------------------------------
inline constexpr uint16_t tim_ccpc      = 1u << 0;
inline constexpr uint16_t tim_ccus      = 1u << 2;
inline constexpr uint16_t tim_ccds      = 1u << 3;
inline constexpr uint16_t tim_mms_mask  = 7u << 4;
inline constexpr uint8_t tim_mms_shift  = 4;
inline constexpr uint16_t tim_ti1s      = 1u << 7;
/// OIS1 at bit 8, then OIS1N, OIS2, OIS2N, OIS3, OIS3N and OIS4 - two
/// bits a channel for the three that have a complementary output, one
/// for channel 4.
inline constexpr uint8_t tim_ois_shift  = 8;
inline constexpr uint16_t tim_ois_mask  = 0x7F00u;

// ---- TIMx_SMCFGR (14.4.3, 15.4.3) ------------------------------------------
inline constexpr uint16_t tim_sms_mask  = 7u << 0;
inline constexpr uint16_t tim_ts_mask   = 7u << 4;
inline constexpr uint8_t tim_ts_shift   = 4;
inline constexpr uint16_t tim_msm       = 1u << 7;
inline constexpr uint16_t tim_etf_mask  = 15u << 8;
inline constexpr uint8_t tim_etf_shift  = 8;
inline constexpr uint16_t tim_etps_mask = 3u << 12;
inline constexpr uint8_t tim_etps_shift = 12;
inline constexpr uint16_t tim_ece       = 1u << 14;
inline constexpr uint16_t tim_etp       = 1u << 15;

// ---- TIMx_DMAINTENR (14.4.4, 15.4.4) ---------------------------------------
inline constexpr uint16_t tim_uie       = 1u << 0;
inline constexpr uint16_t tim_cc1ie     = 1u << 1;
inline constexpr uint16_t tim_comie     = 1u << 5;
inline constexpr uint16_t tim_tie       = 1u << 6;
inline constexpr uint16_t tim_bie       = 1u << 7;
inline constexpr uint16_t tim_ude       = 1u << 8;
inline constexpr uint16_t tim_cc1de     = 1u << 9;
inline constexpr uint16_t tim_comde     = 1u << 13;
inline constexpr uint16_t tim_tde       = 1u << 14;

// ---- TIMx_INTFR (14.4.5, 15.4.5) -------------------------------------------
inline constexpr uint16_t tim_uif       = 1u << 0;
inline constexpr uint16_t tim_cc1if     = 1u << 1;
inline constexpr uint16_t tim_comif     = 1u << 5;
inline constexpr uint16_t tim_tif       = 1u << 6;
inline constexpr uint16_t tim_bif       = 1u << 7;
inline constexpr uint16_t tim_cc1of     = 1u << 9;

// ---- TIMx_SWEVGR (14.4.6, 15.4.6) ------------------------------------------
inline constexpr uint16_t tim_ug        = 1u << 0;
inline constexpr uint16_t tim_cc1g      = 1u << 1;
inline constexpr uint16_t tim_comg      = 1u << 5;
inline constexpr uint16_t tim_tg        = 1u << 6;
inline constexpr uint16_t tim_bg        = 1u << 7;

// ---- TIMx_CCER (14.4.9, 15.4.9) --------------------------------------------
// Four bits a channel: E, P, NE, NP. NE and NP exist on the advanced
// timer's channels 1..3 alone; on a general-purpose timer they are
// Reserved, and channel 4 has no complementary output anywhere.
inline constexpr uint16_t tim_cc1e      = 1u << 0;
inline constexpr uint16_t tim_cc1p      = 1u << 1;
inline constexpr uint16_t tim_cc1ne     = 1u << 2;
inline constexpr uint16_t tim_cc1np     = 1u << 3;

// ---- TIMx_BDTR (14.4.18) ---------------------------------------------------
inline constexpr uint16_t tim_dtg_mask  = 0x00FFu;
inline constexpr uint16_t tim_lock_mask = 3u << 8;
inline constexpr uint8_t tim_lock_shift = 8;
inline constexpr uint16_t tim_ossi      = 1u << 10;
inline constexpr uint16_t tim_ossr      = 1u << 11;
inline constexpr uint16_t tim_bke       = 1u << 12;
inline constexpr uint16_t tim_bkp       = 1u << 13;
inline constexpr uint16_t tim_aoe       = 1u << 14;
inline constexpr uint16_t tim_moe       = 1u << 15;

// ---- TIMx_DMACFGR (14.4.19) ------------------------------------------------
inline constexpr uint16_t tim_dba_mask  = 0x001Fu;
inline constexpr uint16_t tim_dbl_mask  = 0x1F00u;
inline constexpr uint8_t tim_dbl_shift  = 8;

// ---- TIMx_AUX (14.4.21, 15.4.25): CAP_ED_CH2..CAP_ED_CH4 at bits 0..2 --------
inline constexpr uint16_t tim_cap_ed_ch2 = 1u << 0;
inline constexpr uint16_t tim_cap_ed_mask = 0x7u;

// =============================================================================
// What a timer IS on this part
// =============================================================================

/// The three KINDS of timer, each a mask of the part table's (the
/// datasheets' tables 2-1 and 2-1-1): the advanced-control ones (TIM1,
/// and TIM8..TIM10 on the CH32V303RC and VC), the general-purpose ones
/// (TIM2..TIM4, and TIM5 on three parts) and the basic ones (TIM6 and
/// TIM7, the CH32V303RC's and VC's alone).
constexpr bool tim_advanced(uint8_t n) {
    return n < 16u && (device::advanced_timer_instances & static_cast<uint16_t>(1U << n)) != 0u;
}
constexpr bool tim_general(uint8_t n) {
    return n < 16u && (device::general_timer_instances & static_cast<uint16_t>(1U << n)) != 0u;
}
constexpr bool tim_basic(uint8_t n) {
    return n < 16u && (device::basic_timer_instances & static_cast<uint16_t>(1U << n)) != 0u;
}

/// The instances this PART offers.
constexpr bool tim_present(uint8_t n) { return tim_advanced(n) || tim_general(n) || tim_basic(n); }

/// Four capture/compare channels on every timer of this family but the
/// basic ones, which have none (16.2.2).
constexpr uint8_t tim_channels(uint8_t n) { return tim_present(n) && !tim_basic(n) ? 4u : 0u; }

/// The complementary outputs: channels 1..3 of the advanced timers, and
/// nowhere else (14.3.6).
constexpr uint8_t tim_complementary_channels(uint8_t n) { return tim_advanced(n) ? 3u : 0u; }

/// The break and dead-time unit, and the repetition counter: the
/// advanced timers' alone (14.4.13, 14.4.18).
constexpr bool tim_has_break(uint8_t n) { return tim_advanced(n); }
constexpr bool tim_has_repetition(uint8_t n) { return tim_advanced(n); }

/// The counter's width. Sixteen bits everywhere but the CH32V203RB's
/// TIM5: chapter 15's opening note gives thirty-two to the TIM5 of the
/// CH32V20x_D8 (and of two classes this stratum does not serve) and
/// sixteen to every other's - the CH32V303's included.
constexpr uint8_t tim_counter_bits(uint8_t n) {
    return (n == 5u && device::has_tim5 && device::device_class == DeviceClass::v20x_d8) ? 32u
                                                                                        : 16u;
}
constexpr uint32_t tim_max_period(uint8_t n) {
    return tim_counter_bits(n) == 32u ? 0xFFFFFFFFUL : 0xFFFFUL;
}

/// Every advanced and general-purpose timer of this family has the whole
/// slave controller, the encoder modes, the master output, the external
/// trigger, the TI1 XOR and the DMA burst engine: chapters 14 and 15
/// differ only in the break unit, the repetition counter and the
/// complementary outputs. A basic timer has none of them but the master
/// output - three of its codes, the ones that need no channel - and one
/// DMA request, the update's.
constexpr bool tim_has_slave_mode(uint8_t n) { return tim_present(n) && !tim_basic(n); }
constexpr bool tim_has_encoder(uint8_t n) { return tim_present(n) && !tim_basic(n); }
constexpr bool tim_has_master_mode(uint8_t n) { return tim_present(n); }
constexpr bool tim_has_external_trigger(uint8_t n) { return tim_present(n) && !tim_basic(n); }
constexpr bool tim_has_ti1_xor(uint8_t n) { return tim_present(n) && !tim_basic(n); }
constexpr bool tim_has_dma(uint8_t n) { return tim_present(n); }
constexpr bool tim_has_dma_burst(uint8_t n) { return tim_present(n) && !tim_basic(n); }
/// Down-counting, the centre-aligned modes and CKD: not a basic timer's
/// (16.4.1's CTLR1 has none of the three fields).
constexpr bool tim_has_up_down(uint8_t n) { return tim_present(n) && !tim_basic(n); }
/// The dual-edge capture register TIMx_AUX: the CH32V30x_D8's, on every
/// timer with channels (14.4.21, 15.4.25).
constexpr bool tim_has_dual_edge_capture(uint8_t n) {
    return tim_channels(n) != 0u && device::device_class == DeviceClass::v30x_d8;
}

/// Which bus the instance sits on, and the gate bit RCC holds for it.
constexpr Bus tim_bus(uint8_t n) { return tim_advanced(n) ? Bus::pb2 : Bus::pb1; }
constexpr uint32_t tim_gate(uint8_t n) {
    return n == 1u    ? rcc_pb2_tim1
           : n == 2u  ? rcc_pb1_tim2
           : n == 3u  ? rcc_pb1_tim3
           : n == 4u  ? rcc_pb1_tim4
           : n == 5u  ? rcc_pb1_tim5
           : n == 6u  ? rcc_pb1_tim6
           : n == 7u  ? rcc_pb1_tim7
           : n == 8u  ? rcc_pb2_tim8
           : n == 9u  ? rcc_pb2_tim9
           : n == 10u ? rcc_pb2_tim10
                      : 0u;
}

/**
 * Which timer feeds ITR0..ITR3 of instance `n`, as an instance NUMBER;
 * 0 where the link's master is a timer THIS PART has not got, and for a
 * basic timer, which has no slave controller.
 *
 * Tables 14-2 and 15-2, whole: TIM1 listens to TIM5/2/3/4, TIM8 to
 * TIM1/2/4/5, TIM9 to TIM10/5/6/7 - the one timer the two BASIC timers'
 * TRGO reaches - TIM10 to TIM9/2/4/5, TIM2 to TIM1/8/3/4, TIM3 to
 * TIM1/2/5/4, TIM4 to TIM1/2/3/8 and TIM5 to TIM2/3/4/8. Where the master
 * is a timer the part has not got - TIM5 and TIM8 on most of the
 * CH32V203, TIM8 on the CH32V203RB and the two 128 KB CH32V303 - the
 * presence fold removes the link, and no part is named here.
 *
 * TIM2's ITR1 is also the one link an AFIO field could move (PCFR1's
 * TIM2ITR1_RM, to the Ethernet's time stamp or the USB frame marker, and
 * table 15-2 writes "TIM8/USB/ETH" in its cell); that field is READ-ONLY
 * AT ZERO on the CH32V20x_D6, measured, and afio.hpp refuses it there -
 * so on that class this table describes the only connection there is.
 * On the CH32V303VCT6 the field takes a write, and TIM2's ITR1 counts
 * TIM8's TRGO at either value (measured, TIM8 as the master): this table
 * holds there too, and what else the field's second value connects no
 * measurement here has seen.
 */
constexpr uint8_t tim_internal_trigger(uint8_t n, uint8_t itr) {
    if (itr > 3u || tim_basic(n)) {
        return 0u;
    }
    uint8_t src = 0;
    switch (n) {
        case 1: src = itr == 0u ? 5u : itr == 1u ? 2u : itr == 2u ? 3u : 4u; break;
        case 2: src = itr == 0u ? 1u : itr == 1u ? 8u : itr == 2u ? 3u : 4u; break;
        case 3: src = itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 5u : 4u; break;
        case 4: src = itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 3u : 8u; break;
        case 5: src = itr == 0u ? 2u : itr == 1u ? 3u : itr == 2u ? 4u : 8u; break;
        case 8: src = itr == 0u ? 1u : itr == 1u ? 2u : itr == 2u ? 4u : 5u; break;
        case 9: src = itr == 0u ? 10u : itr == 1u ? 5u : itr == 2u ? 6u : 7u; break;
        case 10: src = itr == 0u ? 9u : itr == 1u ? 2u : itr == 2u ? 4u : 5u; break;
        default: return 0u;
    }
    return tim_present(src) ? src : 0u;
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

// =============================================================================
// Vocabulary
// =============================================================================

/// CTLR1.DIR - which way an edge-aligned counter runs (14.4.1). The bit
/// is ignored in a centre-aligned or encoder mode, where the silicon
/// owns the direction.
enum class TimDirection : uint8_t { up = 0, down = 1 };

/// CTLR1.CMS - edge-aligned, or centre-aligned with the compare flag
/// raised while counting down, up, or both. The three centre modes
/// differ ONLY in when CCyIF is raised; the waveform is the same.
enum class TimAlignment : uint8_t {
    edge = 0,
    center_down = 1,   ///< CCyIF set while down-counting
    center_up = 2,     ///< CCyIF set while up-counting
    center_both = 3,
};

/// CTLR1.CKD - the ratio between the timer clock and tDTS, the sampling
/// clock of the digital filters and the unit of the dead-time generator
/// (14.4.1). Code 3 is reserved.
enum class TimClockDivision : uint8_t { div1 = 0, div2 = 1, div4 = 2 };

/// SMCFGR.TS - what the slave controller listens to (14.4.3). Which
/// timer ITR0..ITR3 mean is per instance: `tim_internal_trigger()`.
enum class TimTrigger : uint8_t {
    itr0 = 0, itr1 = 1, itr2 = 2, itr3 = 3,
    ti1_edge = 4,    ///< TI1F_ED: a pulse per TRANSITION of TI1 (never gate on it)
    ti1 = 5,         ///< TI1FP1
    ti2 = 6,         ///< TI2FP2
    etr = 7,         ///< ETRF, the external trigger input
};

/// SMCFGR.SMS - what the trigger does to the counter (14.4.3).
enum class TimSlaveMode : uint8_t {
    disabled = 0,
    encoder1 = 1,            ///< counts TI2 edges only, direction from TI1
    encoder2 = 2,            ///< counts TI1 edges only, direction from TI2
    encoder3 = 3,            ///< counts both, four counts per quadrature cycle
    reset = 4,               ///< a trigger edge reinitializes the counter
    gated = 5,               ///< the counter runs while the trigger is HIGH
    trigger = 6,             ///< a trigger edge starts the counter (does not reset it)
    external_clock1 = 7,     ///< the trigger's rising edges ARE the counter clock
};

/// CTLR2.MMS - what this timer publishes on TRGO (14.4.2). A basic
/// timer has the first three codes and no more (16.4.2), having no
/// channel to publish.
enum class TimMasterMode : uint8_t {
    reset = 0,          ///< SWEVGR.UG
    enable = 1,         ///< the counter-enable signal
    update = 2,         ///< the update event - the frequency divider a slave counts
    compare_pulse = 3,  ///< a pulse when a capture or a compare 1 happened
    oc1ref = 4,         ///< the channel-1 waveform ITSELF, high time and all
    oc2ref = 5,
    oc3ref = 6,
    oc4ref = 7,
};

/// CHCTLRx.OCyM - what a compare match does to the output (14.4.7).
/// Three bits: the combined and asymmetric modes of the later STM32
/// families do not exist here.
enum class TimOutputMode : uint8_t {
    frozen = 0,
    active_on_match = 1,
    inactive_on_match = 2,
    toggle = 3,
    force_inactive = 4,
    force_active = 5,
    pwm1 = 6,               ///< active while CNT < CHxCVR
    pwm2 = 7,               ///< the complement of pwm1
};

/// CHCTLRx.CCyS - a channel is an OUTPUT or an input mapped on one of
/// three sources (14.4.7). `direct` is this channel's own TI,
/// `indirect` its neighbour's - the pair that makes PWM input mode.
enum class TimChannelSelect : uint8_t {
    output = 0,
    direct = 1,
    indirect = 2,
    trc = 3,     ///< the TRC signal (needs the slave controller's trigger)
};

/// CCER.CCyP for an INPUT channel: the rising edge or the falling one.
/// THERE IS NO BOTH-EDGE CODE on this family (fact 6 of the header).
enum class TimCapturePolarity : uint8_t { rising = 0, falling = 1 };

/// CHCTLRx.ICyPSC - capture one edge in 1, 2, 4 or 8 (14.4.7).
enum class TimCapturePrescaler : uint8_t { every = 0, every2 = 1, every4 = 2, every8 = 3 };

/**
 * TIMx_DMACFGR.DBA - the burst engine's base, as a WORD OFFSET from
 * TIMx_CTLR1 (14.4.19's own example). Named rather than computed,
 * because the offsets are the register map's and an application that
 * arithmetics them itself has written a bug no header can catch.
 */
enum class TimBurstBase : uint8_t {
    ctlr1 = 0, ctlr2 = 1, smcfgr = 2, dmaintenr = 3, intfr = 4, swevgr = 5,
    chctlr1 = 6, chctlr2 = 7, ccer = 8, cnt = 9, psc = 10, atrlr = 11,
    rptcr = 12, ch1cvr = 13, ch2cvr = 14, ch3cvr = 15, ch4cvr = 16, bdtr = 17,
};

/// The time base. `period` is ATRLR and is REFUSED at zero: 14.4.12 says
/// a null auto-reload stops the counter, which is a stopped timer
/// wearing the face of a running one.
struct TimConfig {
    uint16_t prescaler = 0;                ///< PSC: the counter clock is TIMxCLK / (PSC + 1)
    uint32_t period = 0xFFFF;              ///< ATRLR
    TimDirection direction = TimDirection::up;
    TimAlignment alignment = TimAlignment::edge;
    TimClockDivision clock_division = TimClockDivision::div1;
    bool auto_reload_preload = false;      ///< CTLR1.ARPE
    bool one_pulse = false;                ///< CTLR1.OPM: stop at the next update
    bool update_disable = false;           ///< CTLR1.UDIS
    bool update_on_overflow_only = false;  ///< CTLR1.URS: UG and a slave reset raise no UIF
    uint8_t repetition = 0;                ///< RPTCR (the advanced timer's alone)
};

/// One capture/compare channel as an OUTPUT.
struct TimChannelConfig {
    TimOutputMode mode = TimOutputMode::frozen;
    uint32_t compare = 0;                  ///< CHxCVR
    bool preload = true;                   ///< CHCTLRx.OCyPE - see fact 3 of the header
    bool fast = false;                     ///< CHCTLRx.OCyFE
    bool clear_on_etrf = false;            ///< CHCTLRx.OCyCE: a high ETRF clears OCyREF
    bool active_low = false;               ///< CCER.CCyP
    bool complementary_active_low = false; ///< CCER.CCyNP
    bool enable = true;                    ///< CCER.CCyE
    bool complementary_enable = false;     ///< CCER.CCyNE
    bool idle_high = false;                ///< CTLR2.OISy  (the advanced timer)
    bool complementary_idle_high = false;  ///< CTLR2.OISyN
};

/// One capture/compare channel as an INPUT.
struct TimCaptureConfig {
    TimChannelSelect select = TimChannelSelect::direct;
    TimCapturePolarity polarity = TimCapturePolarity::rising;
    TimCapturePrescaler prescaler = TimCapturePrescaler::every;
    /// ICyF, 0..15: how many consecutive samples at the rate 14.4.3's
    /// table gives must agree before an edge is believed. The sampling
    /// clock is tDTS or the timer clock, per the code.
    uint8_t filter = 0;
    bool enable = true;                    ///< CCER.CCyE
};

/// The slave controller, written as one configuration because SMS and
/// TS belong together: 14.4.3's own note says TS may be changed only
/// while SMS is zero.
struct TimSlaveConfig {
    TimSlaveMode mode = TimSlaveMode::disabled;
    TimTrigger trigger = TimTrigger::itr0;
    /// SMCFGR.MSM: delay this timer's reaction so cascaded slaves line
    /// up cycle for cycle.
    bool master_slave = false;
};

/// SMCFGR's external-trigger half (14.4.3): the ETR input's polarity,
/// prescaler and filter, and ECE - external clock mode 2, ETRF as the
/// counter's clock with no trigger selection at all.
struct TimEtrConfig {
    bool inverted = false;     ///< ETP: active low / falling edge
    uint8_t prescaler = 0;     ///< ETPS: 0 off, 1 /2, 2 /4, 3 /8
    uint8_t filter = 0;        ///< ETF 0..15, the same table as a channel's ICyF
    bool clock_mode2 = false;  ///< ECE: the counter clocks on ETRF's rising edges
};

constexpr bool tim_etr_config_valid(const TimEtrConfig& c) {
    return c.prescaler <= 3u && c.filter <= 15u;
}

/// BDTR, the break and dead-time unit of the advanced timer (14.4.18).
/// LOCK IS ONE-WAY: once written non-zero, the level's registers are
/// read-only until the next peripheral reset - which is why it is
/// spelled here and defaults to none.
struct TimBreakDeadTime {
    /// DTG, raw: 14.4.18's four ranges, decoded by
    /// `tim_dead_time_ticks()` and searched by `tim_dead_time_code()`.
    uint8_t dead_time = 0;
    bool main_output_enable = true;     ///< MOE - without it no output reaches a pad
    /// AOE: the outputs come back on their own at the next update after
    /// a break. Defaults FALSE, so that a break stands until the
    /// program says otherwise.
    bool automatic_output_enable = false;
    bool break_enable = false;          ///< BKE
    bool break_active_high = false;     ///< BKP
    bool off_state_run = false;         ///< OSSR
    bool off_state_idle = false;        ///< OSSI
    uint8_t lock = 0;                   ///< LOCK 0..3, ONE-WAY (see above)
};

/// The quadrature interface (14.3.9): which edges are counted, and each
/// input's polarity - which is what "inverted" means in encoder mode,
/// not an edge selection. Both channels take the same filter code here
/// because an encoder's two tracks come off one disc.
struct TimEncoderConfig {
    TimSlaveMode mode = TimSlaveMode::encoder3;
    uint8_t filter = 0;             ///< ICyF for both inputs, 0..15
    bool invert_a = false;          ///< CC1P: TI1FP1 inverted
    bool invert_b = false;          ///< CC2P: TI2FP2 inverted
};

/// The dead time a DTG code produces, in tDTS ticks (14.4.18's four
/// ranges). tDTS is the timer clock divided by CTLR1.CKD.
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
 * TIMxCLK, the rate a timer's prescaler divides, from a Clock task's own
 * constexpr dividers (fact 5 of the file header): the bus clock when
 * that bus's prescaler is 1, twice it otherwise (RM 3.3.1).
 */
template <typename Clock>
constexpr uint32_t tim_clock_hz(Clock, bool on_pb2) {
    static_assert(Clock::is_static,
                  "brio Tim: a timer counts TIMxCLK and every period, prescaler and "
                  "capture it holds is in those cycles - a DynamicClock would move all "
                  "of them at once and no rebase() can promise the same periods (the "
                  "prescaler's granularity), so the timers take a static clock only. "
                  "A rescaling program keeps its timers on what does not move: the RTC "
                  "on the crystal - docs/design/clock.md");
    return on_pb2 ? Clock::timclk2_hz : Clock::timclk1_hz;
}

// =============================================================================
// The pads
// =============================================================================

/// An advanced timer's column (TIM1, TIM8, TIM9, TIM10), from afio.hpp.
constexpr AdvancedTimPads tim_advanced_pads(uint8_t n, uint8_t code) {
    return !tim_advanced(n) ? AdvancedTimPads{}
           : n == 1u        ? afio_tim1_pads(code)
           : n == 8u        ? afio_tim8_pads(code)
           : n == 9u        ? afio_tim9_pads(code)
           : n == 10u       ? afio_tim10_pads(code)
                            : AdvancedTimPads{};
}

/// A general-purpose timer's column. TIM5 has none - its four channels
/// are PA0..PA3 under code 0 and nothing else (afio.hpp).
constexpr TimPads tim_general_pads(uint8_t n, uint8_t code) {
    return !tim_general(n) ? TimPads{}
           : n == 2u       ? afio_tim2_pads(code)
           : n == 3u       ? afio_tim3_pads(code)
           : n == 4u       ? afio_tim4_pads(code)
           : n == 5u       ? (code == 0u ? afio_tim5_pad_set : TimPads{})
                           : TimPads{};
}

/// Which pad carries channel `ch` (0-based) of timer `n` under remap
/// column `code`, from afio.hpp's own tables. An invalid Pad where the
/// column does not exist, the signal has no pad in it, or the timer has
/// no channel at all.
constexpr Pad tim_channel_pad(uint8_t n, uint8_t code, uint8_t ch) {
    if (ch >= 4u || tim_channels(n) == 0u) {
        return Pad{};
    }
    if (tim_advanced(n)) {
        const AdvancedTimPads p = tim_advanced_pads(n, code);
        return ch == 0u ? p.ch1 : ch == 1u ? p.ch2 : ch == 2u ? p.ch3 : p.ch4;
    }
    const TimPads p = tim_general_pads(n, code);
    return ch == 0u ? p.ch1 : ch == 1u ? p.ch2 : ch == 2u ? p.ch3 : p.ch4;
}

/// The complementary output of channel `ch` of timer `n` - an advanced
/// timer's channels 1..3 alone.
constexpr Pad tim_complementary_pad(uint8_t n, uint8_t code, uint8_t ch) {
    if (ch >= 3u || !tim_advanced(n)) {
        return Pad{};
    }
    const AdvancedTimPads p = tim_advanced_pads(n, code);
    return ch == 0u ? p.ch1n : ch == 1u ? p.ch2n : p.ch3n;
}

/// The external trigger's pad. TIM3's is PD2 in every column (table
/// 10-17's note 1), TIM4's is PE0 in both (the CH32V303 datasheet's
/// table 3-4), and TIM5 and the basic timers have none.
constexpr Pad tim_etr_pad(uint8_t n, uint8_t code) {
    if (!tim_present(n)) {
        return Pad{};
    }
    return tim_advanced(n) ? tim_advanced_pads(n, code).etr : tim_general_pads(n, code).etr;
}

/// The break input's pad - the advanced timers' alone.
constexpr Pad tim_break_pad(uint8_t n, uint8_t code) {
    return tim_advanced(n) ? tim_advanced_pads(n, code).bkin : Pad{};
}

/**
 * A pad handed to a timer signal.
 *
 *   constexpr brio::Pad wave = brio::tim_channel_pad(3, 0, 0);  // TIM3_CH1
 *   using Wave = brio::TimPad<wave>;
 *   Wave::claim();                     // alternate function, push-pull
 *
 * An OUTPUT channel wants the alternate-function nibble, which is what
 * `claim()` writes; a CAPTURE channel wants a plain input, which is
 * what `claim_input()` writes - on this family the peripheral's input
 * is the pad's own input buffer and there is no AF nibble for it
 * (10.2.4).
 */
template <Pad pad>
struct TimPad {
    TimPad() = delete;

    static_assert(pad.valid(), "brio TimPad: a pad must be named");
    static_assert(pad_bonded(pad),
                  "brio TimPad: this package does not bring that pad out - the remap "
                  "column carries no such signal here (afio.hpp's tables and "
                  "device::port_pins)");

    using pin = Pin<pad.port, pad.pin>;
    static constexpr Pad selection = pad;

    /// The pad as the timer's OUTPUT.
    static void claim(PinDrive drive = PinDrive::push_pull, PinSpeed speed = PinSpeed::fast) {
        pin::function(drive, speed);
    }
    /// The pad as a channel's INPUT, pulled as asked.
    static void claim_input(PinPull pull = PinPull::none) { pin::input(pull); }
    static bool read() { return pin::read(); }
    static void release() { pin::release(); }
};

// =============================================================================
// Tim<n>: the resource
// =============================================================================

/**
 * Tim<n>: one TIMx block.
 *
 *   using Wave = brio::Tim<3>;
 *   Wave::init();                                  // clock on, reset
 *   Wave::configure({.prescaler = 71, .period = 999});
 *   Wave::output_channel(0, {.mode = brio::TimOutputMode::pwm1,
 *                            .compare = 500});
 *   Wave::enable(true);
 *
 * Every verb that names a feature this instance has not got returns
 * false and writes NOTHING; every verb that names a channel checks it
 * against `channels`. Channels are 0-based here (channel 1 of the
 * chapter is 0), this project's convention on every target.
 */
template <uint8_t n>
class Tim {
public:
    Tim() = delete;

    static_assert(tim_present(n),
                  "brio Tim: this part has no such timer - every part carries TIM1 "
                  "(advanced-control) and TIM2..TIM4 (general-purpose); TIM5 is the "
                  "CH32V203RB's and the 256 KB CH32V303's, and TIM6..TIM10 the "
                  "CH32V303RC's and VC's alone (the datasheets' tables 2-1 and 2-1-1, "
                  "parts/<part>.hpp's masks)");

    static constexpr uint8_t instance = n;

    // ---- what this instance IS ---------------------------------------------
    static constexpr uint8_t counter_bits = tim_counter_bits(n);
    static constexpr uint32_t max_period = tim_max_period(n);
    static constexpr uint8_t channels = tim_channels(n);
    static constexpr uint8_t complementary_channels = tim_complementary_channels(n);
    static constexpr bool has_break = tim_has_break(n);
    static constexpr bool has_repetition = tim_has_repetition(n);
    static constexpr bool has_slave_mode = tim_has_slave_mode(n);
    static constexpr bool has_encoder = tim_has_encoder(n);
    static constexpr bool has_master_mode = tim_has_master_mode(n);
    static constexpr bool has_external_trigger = tim_has_external_trigger(n);
    static constexpr bool has_ti1_xor = tim_has_ti1_xor(n);
    static constexpr bool has_dma = tim_has_dma(n);
    static constexpr bool has_dma_burst = tim_has_dma_burst(n);
    static constexpr bool has_up_down = tim_has_up_down(n);
    static constexpr bool has_dual_edge_capture = tim_has_dual_edge_capture(n);
    static constexpr bool is_basic = tim_basic(n);
    static constexpr bool on_pb2 = tim_bus(n) == Bus::pb2;
    /// Whether this timer's outputs and inputs move as a REMAP COLUMN at
    /// all: TIM5 has no column (its one field moves channel 4 to the LSI)
    /// and the basic timers have no pad.
    static constexpr bool has_remap = n <= 4u || n >= 8u;
    /// The column's field, for the timers that have one (afio.hpp answers
    /// for the device class and the package).
    static constexpr Remap remap_field = n == 1u   ? Remap::tim1
                                         : n == 2u ? Remap::tim2
                                         : n == 3u ? Remap::tim3
                                         : n == 8u ? Remap::tim8
                                         : n == 9u ? Remap::tim9
                                         : n == 10u ? Remap::tim10
                                                    : Remap::tim4;

    /// The vectors. Each advanced timer has FOUR lines of its own and
    /// none of them is shared - TIM1's at 40..43, and on the CH32V303
    /// TIM8's at 59..62, TIM9's at 90..93 and TIM10's at 94..97 - and
    /// every other timer has one, so all four accessors answer with it.
    static constexpr Irq irq() {
        if constexpr (n == 1u) {
            return Irq::tim1_up;
        } else if constexpr (n == 8u) {
            return Irq::tim8_up;
        } else if constexpr (n == 9u) {
            return Irq::tim9_up;
        } else if constexpr (n == 10u) {
            return Irq::tim10_up;
        } else if constexpr (n == 2u) {
            return Irq::tim2;
        } else if constexpr (n == 3u) {
            return Irq::tim3;
        } else if constexpr (n == 4u) {
            return Irq::tim4;
        } else if constexpr (n == 5u) {
            return Irq::tim5;
        } else if constexpr (n == 6u) {
            return Irq::tim6;
        } else {
            return Irq::tim7;
        }
    }
    static constexpr Irq cc_irq() {
        return n == 1u    ? Irq::tim1_cc
               : n == 8u  ? Irq::tim8_cc
               : n == 9u  ? Irq::tim9_cc
               : n == 10u ? Irq::tim10_cc
                          : irq();
    }
    static constexpr Irq break_irq() {
        return n == 1u    ? Irq::tim1_brk
               : n == 8u  ? Irq::tim8_brk
               : n == 9u  ? Irq::tim9_brk
               : n == 10u ? Irq::tim10_brk
                          : irq();
    }
    static constexpr Irq trigger_irq() {
        return n == 1u    ? Irq::tim1_trg_com
               : n == 8u  ? Irq::tim8_trg_com
               : n == 9u  ? Irq::tim9_trg_com
               : n == 10u ? Irq::tim10_trg_com
                          : irq();
    }
    static constexpr bool has_split_vectors = tim_advanced(n);

    static TimRegs& regs() { return *reinterpret_cast<TimRegs*>(tim_base_for(n)); }

    // ---- the bus clock and the reset ---------------------------------------
    //
    // A peripheral whose gate is closed answers nothing, so init() opens
    // it before it touches a register, and the reset that follows makes
    // the state this driver assumes true whatever ran before.

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(tim_bus(n), tim_gate(n));
        } else {
            Rcc::disable(tim_bus(n), tim_gate(n));
        }
    }
    static bool bus_clock() { return Rcc::enabled(tim_bus(n), tim_gate(n)); }

    /// Pulse the block's reset line: every register back to the value
    /// the chapter's register map prints.
    static void reset() { Rcc::reset(tim_bus(n), tim_gate(n)); }

    /// Clock on, then reset. The whole of "bring this timer up".
    static void init() {
        bus_clock(true);
        reset();
    }

    /// Counter stopped, outputs off, pads NOT touched (a pad is the
    /// caller's claim - TimPad::release() is its release), block reset
    /// and its gate closed.
    static void release() {
        enable(false);
        reset();
        bus_clock(false);
    }

    // ---- the pads ----------------------------------------------------------

    /// Select this timer's remap COLUMN (afio.hpp). False - and nothing
    /// written - for a code this device class or this package has not
    /// got. TIM5 has no column of its own: its channel 4 can be fed the
    /// LSI instead of a pad, which is Remap::tim5_ch4's business; and a
    /// basic timer has no pad at all.
    static bool remap(uint8_t code) {
        if constexpr (!has_remap) {
            (void)code;
            return false;
        } else {
            return Afio::remap(remap_field, code);
        }
    }
    static uint8_t remap() {
        if constexpr (!has_remap) {
            return 0u;
        } else {
            return Afio::remap_code(remap_field);
        }
    }

    /// The pads of a column, for a program that wants the table rather
    /// than the register.
    static constexpr Pad channel_pad(uint8_t code, uint8_t ch) {
        return tim_channel_pad(n, code, ch);
    }
    static constexpr Pad complementary_pad(uint8_t code, uint8_t ch) {
        return tim_complementary_pad(n, code, ch);
    }
    static constexpr Pad etr_pad(uint8_t code) { return tim_etr_pad(n, code); }
    static constexpr Pad break_pad(uint8_t code) { return tim_break_pad(n, code); }

    // ---- the counter's clock (fact 5) --------------------------------------

    /// TIMxCLK for this instance, from the clock task's own dividers.
    template <typename Clock>
    static constexpr uint32_t clock_hz(Clock c) {
        return tim_clock_hz(c, on_pb2);
    }

    /// The same number read back out of the SILICON - RCC's own
    /// prescaler field - given the HCLK the program believes it runs
    /// at. The witness clock_hz() has not got.
    static uint32_t clock_hz_now(uint32_t hclk) {
        const uint8_t code = on_pb2 ? Rcc::ppre2_code() : Rcc::ppre1_code();
        return timclk_hz_at(hclk / ppre_divider(code), code);
    }

    // ---- the time base (14.4.1, 14.4.11, 14.4.12) --------------------------

    /// Whether a configuration is legal for THIS instance - the same
    /// judgment configure() makes, available at compile time so a task
    /// can static_assert on it.
    static constexpr bool config_valid(const TimConfig& c) {
        if (c.period == 0u || c.period > max_period) {
            return false;
        }
        if (c.repetition != 0u && !has_repetition) {
            return false;
        }
        if (static_cast<uint8_t>(c.clock_division) > 2u) {
            return false;
        }
        // A basic timer counts up, edge-aligned, and has no CKD (16.4.1).
        if (!has_up_down && (c.direction != TimDirection::up ||
                             c.alignment != TimAlignment::edge ||
                             c.clock_division != TimClockDivision::div1)) {
            return false;
        }
        return true;
    }

    /**
     * The time base. The counter is STOPPED first, the registers
     * written, and the configuration then forced into the shadow
     * registers by a software update event whose flag is cleared again
     * - fact 2 of the file header: without it PSC and (with ARPE) ATRLR
     * would stand until the first overflow of the OLD period. CNT is
     * zeroed too, so a reconfigured timer starts where the caller
     * thinks it does.
     */
    static bool configure(const TimConfig& c) {
        if (!config_valid(c)) {
            return false;
        }
        TimRegs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(t.CTLR1 & ~tim_cen);
        t.PSC = c.prescaler;
        t.ATRLR = c.period;
        if constexpr (has_repetition) {
            t.RPTCR = c.repetition;
        }
        uint16_t ctlr1 = 0;
        if (c.direction == TimDirection::down) {
            ctlr1 = static_cast<uint16_t>(ctlr1 | tim_dir);
        }
        ctlr1 = static_cast<uint16_t>(
            ctlr1 | ((static_cast<uint16_t>(c.alignment) << tim_cms_shift) & tim_cms_mask));
        ctlr1 = static_cast<uint16_t>(
            ctlr1 | ((static_cast<uint16_t>(c.clock_division) << tim_ckd_shift) & tim_ckd_mask));
        if (c.auto_reload_preload) { ctlr1 = static_cast<uint16_t>(ctlr1 | tim_arpe); }
        if (c.one_pulse) { ctlr1 = static_cast<uint16_t>(ctlr1 | tim_opm); }
        if (c.update_disable) { ctlr1 = static_cast<uint16_t>(ctlr1 | tim_udis); }
        if (c.update_on_overflow_only) { ctlr1 = static_cast<uint16_t>(ctlr1 | tim_urs); }
        t.CTLR1 = ctlr1;
        t.CNT = 0;
        t.SWEVGR = tim_ug;
        clear_flags(tim_uif);
        return true;
    }

    /// CTLR1.CEN. Starting a timer is one bit and nothing else -
    /// everything that has to be true first is configure()'s.
    static void enable(bool on) {
        TimRegs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(on ? (t.CTLR1 | tim_cen) : (t.CTLR1 & ~tim_cen));
    }
    static bool enabled() { return (regs().CTLR1 & tim_cen) != 0u; }

    /// The counter. Sixteen bits everywhere but the 32-bit TIM5.
    static uint32_t count() { return regs().CNT & max_period; }
    static void set_count(uint32_t v) { regs().CNT = v & max_period; }

    /// CTLR1.DIR read back - in a centre-aligned or encoder mode this is
    /// the SILICON's answer to which way it is counting, not the
    /// caller's setting.
    static TimDirection direction() {
        return (regs().CTLR1 & tim_dir) != 0u ? TimDirection::down : TimDirection::up;
    }

    static uint16_t prescaler() { return regs().PSC; }
    /// PSC takes effect at the next update event, not now (14.2.3).
    static void set_prescaler(uint16_t p) { regs().PSC = p; }

    static uint32_t period() { return regs().ATRLR & max_period; }
    /// ATRLR: at once with ARPE clear, at the next update with it set.
    /// Refuses zero and anything past this counter's width.
    static bool set_period(uint32_t v) {
        if (v == 0u || v > max_period) {
            return false;
        }
        regs().ATRLR = v;
        return true;
    }
    static bool auto_reload_preload() { return (regs().CTLR1 & tim_arpe) != 0u; }

    /// RPTCR: one update event in `repetition + 1` counter periods
    /// (14.4.13).
    static uint8_t repetition() {
        if constexpr (!has_repetition) {
            return 0u;
        } else {
            return static_cast<uint8_t>(regs().RPTCR & 0xFFu);
        }
    }
    static bool set_repetition(uint8_t v) {
        if constexpr (!has_repetition) {
            (void)v;
            return false;
        } else {
            regs().RPTCR = v;
            return true;
        }
    }

    /// SWEVGR: raise an event by software (14.4.6). `update()` is the
    /// one that reloads the shadow registers.
    static void update() { regs().SWEVGR = tim_ug; }
    static bool capture_compare_event(uint8_t ch) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if (ch >= channels) {
            return false;
        }
        regs().SWEVGR = static_cast<uint16_t>(tim_cc1g << ch);
        return true;
    }
    static bool trigger_event() {
        if constexpr (!has_slave_mode) {
            return false;
        } else {
            regs().SWEVGR = tim_tg;
            return true;
        }
    }
    /// SWEVGR.COMG: a commutation by software - what transfers the
    /// preloaded channel configuration when CTLR2.CCPC is set.
    static bool commutation_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().SWEVGR = tim_comg;
            return true;
        }
    }
    /// SWEVGR.BG: a break by software, which is the only break a board
    /// with no wire can raise. It clears MOE exactly as a pad would.
    static bool break_event() {
        if constexpr (!has_break) {
            return false;
        } else {
            regs().SWEVGR = tim_bg;
            return true;
        }
    }

    // ---- flags (14.4.5) -----------------------------------------------------
    //
    // rc_w0, fact 1 of the file header: `INTFR = ~mask` clears exactly
    // the named flags and cannot swallow one that arrives meanwhile.

    static constexpr uint16_t update_flag = tim_uif;
    static constexpr uint16_t trigger_flag = tim_tif;
    static constexpr uint16_t break_flag = tim_bif;
    static constexpr uint16_t commutation_flag = tim_comif;
    static constexpr uint16_t compare_flag(uint8_t ch) {
        return static_cast<uint16_t>(tim_cc1if << ch);
    }
    /// CCyOF: a capture arrived while the previous one was still unread.
    /// The NEW value is lost, not the old one (14.4.5).
    static constexpr uint16_t overcapture_flag(uint8_t ch) {
        return static_cast<uint16_t>(tim_cc1of << ch);
    }
    /// The flags an INTERRUPT can be raised for - INTFR's low byte, at
    /// the very bit positions DMAINTENR's enables sit at. The
    /// overcapture flags are NOT among them: they have no enable and no
    /// vector, and are read and cleared by whoever reads the capture. A
    /// basic timer has one flag, UIF (16.4.4).
    static constexpr uint16_t interrupt_flags =
        is_basic ? tim_uif
                 : static_cast<uint16_t>(
                       tim_uif | tim_cc1if | (tim_cc1if << 1) | (tim_cc1if << 2) |
                       (tim_cc1if << 3) |
                       (has_break ? static_cast<uint16_t>(tim_comif | tim_bif) : 0u) | tim_tif);
    /// Every flag this chapter has, for flags() and clear_flags().
    static constexpr uint16_t all_flags =
        is_basic ? tim_uif
                 : static_cast<uint16_t>(interrupt_flags | tim_cc1of | (tim_cc1of << 1) |
                                         (tim_cc1of << 2) | (tim_cc1of << 3));

    static uint16_t flags() { return regs().INTFR; }
    static bool flag(uint16_t mask) { return (regs().INTFR & mask) != 0u; }
    static void clear_flags(uint16_t mask) { regs().INTFR = static_cast<uint16_t>(~mask); }

    /**
     * The flags vector `v` answers for on THIS instance - what a handler
     * hands isr() so that one of four vectors cannot consume another's
     * event. On a timer with one line every group is that one line's;
     * on the advanced timer it is the split table 9-2 makes.
     *
     * Zero for a vector that is not this timer's at all.
     */
    static constexpr uint16_t vector_flags(Irq v) {
        if constexpr (!has_split_vectors) {
            return v == irq() ? interrupt_flags : static_cast<uint16_t>(0u);
        } else {
            if (v == irq()) {
                return tim_uif;
            }
            if (v == cc_irq()) {
                return static_cast<uint16_t>(tim_cc1if | (tim_cc1if << 1) | (tim_cc1if << 2) |
                                             (tim_cc1if << 3));
            }
            if (v == break_irq()) {
                return tim_bif;
            }
            if (v == trigger_irq()) {
                return static_cast<uint16_t>(tim_tif | tim_comif);
            }
            return static_cast<uint16_t>(0u);
        }
    }

    // ---- interrupts and DMA requests (14.4.4) -------------------------------

    static constexpr uint16_t update_interrupt = tim_uie;
    static constexpr uint16_t trigger_interrupt = tim_tie;
    static constexpr uint16_t break_interrupt = tim_bie;
    static constexpr uint16_t commutation_interrupt = tim_comie;
    static constexpr uint16_t compare_interrupt(uint8_t ch) {
        return static_cast<uint16_t>(tim_cc1ie << ch);
    }
    static constexpr uint16_t update_dma = tim_ude;
    static constexpr uint16_t trigger_dma = tim_tde;
    static constexpr uint16_t commutation_dma = tim_comde;
    static constexpr uint16_t compare_dma(uint8_t ch) {
        return static_cast<uint16_t>(tim_cc1de << ch);
    }

    static void interrupts(uint16_t mask, bool on) {
        TimRegs& t = regs();
        t.DMAINTENR = static_cast<uint16_t>(on ? (t.DMAINTENR | mask) : (t.DMAINTENR & ~mask));
    }
    static uint16_t interrupts() { return regs().DMAINTENR; }

    /**
     * The body of one of this timer's vectors.
     *
     * `mask` says which flags this vector answers for: `vector_flags(v)`
     * on the advanced timer, whose four event groups have four lines,
     * and `interrupt_flags` (the default) on every other, whose one
     * line answers for everything.
     *
     * DMAINTENR IS MASKED to its interrupt bits before the AND, and that
     * is not tidiness: its capture/compare DMA enables sit at bits
     * 9..12, exactly where INTFR's OVERCAPTURE flags do, so a channel
     * with a DMA request armed would otherwise make this body eat a
     * CCyOF that belongs to whoever reads the capture.
     *
     * Returns the flags that were BOTH set and enabled, having cleared
     * exactly those. A flag whose interrupt is not enabled is left
     * standing for a poller to read - which is what makes every
     * measurement in this chapter possible with no handler at all.
     *
     * A BREAK INPUT HELD AT ITS ACTIVE LEVEL SETS BIF AGAIN AS SOON AS IT
     * IS CLEARED (measured on the CH32V303VCT6's TIM8; 14.4.5 says only
     * "cleared by software"), so a break vector whose body returns while
     * the input stands is taken again at once, and the core does nothing
     * else until the pad falls. A program whose break input can stand
     * masks `break_interrupt` in its body until the input has returned;
     * this body does not choose that policy for it.
     */
    [[gnu::always_inline]] static uint16_t isr(uint16_t mask = interrupt_flags) {
        TimRegs& t = regs();
        const uint16_t hit =
            static_cast<uint16_t>(t.INTFR & (t.DMAINTENR & interrupt_flags) & mask);
        if (hit != 0u) {
            t.INTFR = static_cast<uint16_t>(~hit);
        }
        return hit;
    }

    // ---- capture/compare channels (14.4.7 .. 14.4.9) ------------------------

    /// CHxCVR. Reading a CAPTURE channel's register is what clears its
    /// CCyIF (14.4.5), so this verb is the acknowledgement as well as
    /// the reading - a handler that clears the flag without reading
    /// throws the measurement away.
    static uint32_t compare(uint8_t ch) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if (ch >= channels) {
            return 0;
        }
        return regs().CHCVR[ch] & max_period;
    }
    static bool set_compare(uint8_t ch, uint32_t v) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if (ch >= channels || v > max_period) {
            return false;
        }
        regs().CHCVR[ch] = v;
        return true;
    }

    /**
     * Configure channel `ch` as an OUTPUT. The channel is disabled in
     * CCER first, then CHCTLRx, CHxCVR, CTLR2's idle levels (where there
     * are any) and CCER are written in that order, so the enable is the
     * last store and the pad is never driven by a half-written
     * configuration.
     *
     * THE DISABLE IS NOT TIDINESS, IT IS 14.4.7's RULE: CCyS is
     * writable only when the channel is off. Without it a channel that
     * is currently an INPUT stays one - the field is dropped in silence
     * - and the timer goes on capturing a pad the caller believes it is
     * driving.
     */
    static bool output_channel(uint8_t ch, const TimChannelConfig& c) {
        static_assert(channels > 0u,
                      "brio Tim: a basic timer has no capture/compare channel (RM 16.2.2) - "
                      "the registers this verb writes are not in its map");
        if (ch >= channels || c.compare > max_period) {
            return false;
        }
        if (c.complementary_enable && ch >= complementary_channels) {
            return false;
        }
        if (c.clear_on_etrf && !has_external_trigger) {
            return false;   // OCyCE clears on ETRF: no ETR, no clear
        }
        write_ccer(ch, false, false, false, false);

        TimRegs& t = regs();
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint16_t v = 0;
        v = static_cast<uint16_t>(v | ((static_cast<uint16_t>(c.mode) & 0x7u) << (shift + 4u)));
        if (c.fast) { v = static_cast<uint16_t>(v | (1u << (shift + 2u))); }
        if (c.preload) { v = static_cast<uint16_t>(v | (1u << (shift + 3u))); }
        if (c.clear_on_etrf) { v = static_cast<uint16_t>(v | (1u << (shift + 7u))); }
        write_chctlr(ch, v);

        t.CHCVR[ch] = c.compare;

        if constexpr (has_break) {
            // OISy / OISyN live in CTLR2 and are write-protected by
            // BDTR.LOCK level 1 and up (14.4.18) - a caller that has
            // locked the timer gets what it locked, not an error.
            const uint8_t ois_shift = static_cast<uint8_t>(tim_ois_shift + 2u * ch);
            uint16_t ctlr2 = static_cast<uint16_t>(t.CTLR2 & ~(0x3u << ois_shift));
            if (c.idle_high) { ctlr2 = static_cast<uint16_t>(ctlr2 | (1u << ois_shift)); }
            if (c.complementary_idle_high && ch < complementary_channels) {
                ctlr2 = static_cast<uint16_t>(ctlr2 | (2u << ois_shift));
            }
            t.CTLR2 = ctlr2;
        }

        write_ccer(ch, c.enable, c.active_low,
                   c.complementary_enable && ch < complementary_channels,
                   c.complementary_active_low);
        return true;
    }

    /// Configure channel `ch` as an INPUT. The channel is disabled in
    /// CCER first, for 14.4.7's rule above - the silicon's, and the
    /// reason this is not one store.
    static bool capture_channel(uint8_t ch, const TimCaptureConfig& c) {
        static_assert(channels > 0u,
                      "brio Tim: a basic timer has no capture/compare channel (RM 16.2.2) - "
                      "the registers this verb writes are not in its map");
        if (ch >= channels || c.select == TimChannelSelect::output || c.filter > 15u) {
            return false;
        }
        if (c.select == TimChannelSelect::trc && !has_slave_mode) {
            return false;   // TRC is the slave controller's signal
        }
        write_ccer(ch, false, false, false, false);

        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint16_t v = 0;
        v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.select) << shift));
        v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.prescaler) << (shift + 2u)));
        v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.filter) << (shift + 4u)));
        write_chctlr(ch, v);

        write_ccer(ch, c.enable, c.polarity == TimCapturePolarity::falling, false, false);
        return true;
    }

    /**
     * CHCTLRx.OCyM alone, for a channel already configured as an output
     * - the chapter's forced output mode (14.3.3) reached in one store.
     *
     * `force_active` and `force_inactive` are what make an output
     * channel A PAD THE PROGRAM DRIVES while the pad stays in alternate
     * function: OCyREF follows the field and not the counter, so the
     * CPU puts a level on the pin through the timer's own output stage.
     * That is the stimulus a capture or an encoder input has on a board
     * with no wire.
     */
    static bool output_mode(uint8_t ch, TimOutputMode m) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if (ch >= channels) {
            return false;
        }
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        TimRegs& t = regs();
        volatile uint16_t& r = ch < 2u ? t.CHCTLR1 : t.CHCTLR2;
        r = static_cast<uint16_t>((r & ~(0x7u << (shift + 4u))) |
                                  ((static_cast<uint16_t>(m) & 0x7u) << (shift + 4u)));
        return true;
    }
    static TimOutputMode output_mode(uint8_t ch) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if (ch >= channels) {
            return TimOutputMode::frozen;
        }
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        const uint16_t r = ch < 2u ? regs().CHCTLR1 : regs().CHCTLR2;
        return static_cast<TimOutputMode>((r >> (shift + 4u)) & 0x7u);
    }

    /// CCER.CCyE alone, for a channel already configured.
    static bool channel_enable(uint8_t ch, bool on) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if (ch >= channels) {
            return false;
        }
        TimRegs& t = regs();
        const uint16_t bit = static_cast<uint16_t>(tim_cc1e << (4u * ch));
        t.CCER = static_cast<uint16_t>(on ? (t.CCER | bit) : (t.CCER & ~bit));
        return true;
    }
    static bool channel_enabled(uint8_t ch) {
        return ch < channels && (regs().CCER & static_cast<uint16_t>(tim_cc1e << (4u * ch))) != 0u;
    }
    static bool complementary_enable(uint8_t ch, bool on) {
        if (ch >= complementary_channels) {
            return false;
        }
        TimRegs& t = regs();
        const uint16_t bit = static_cast<uint16_t>(tim_cc1ne << (4u * ch));
        t.CCER = static_cast<uint16_t>(on ? (t.CCER | bit) : (t.CCER & ~bit));
        return true;
    }
    static bool complementary_enabled(uint8_t ch) {
        return ch < complementary_channels &&
               (regs().CCER & static_cast<uint16_t>(tim_cc1ne << (4u * ch))) != 0u;
    }

    // ---- the dual-edge capture (14.3.11, 14.4.21, 15.3.9, 15.4.25) ---------
    //
    // THE CH32V30x_D8's, channels 2..4 (0-based 1..3). The chapter's own
    // recipe: CCyS = 11 - the code that on every other channel means TRC -
    // and TIMx_AUX's CAP_ED_CHy set; the channel's register then holds
    // "the captured dual-edge pulse width value". And "only available for
    // lot numbers where the penultimate sixth bit is not zero": a LOT
    // rule, which no register states and the part number does not carry.
    // What a die without the register does is measured - on a CH32V303VC
    // TIMx_AUX reads zero whatever is written into it, on TIM1 and TIM2
    // alike - so the verb ASKS the die: it sets the bit, reads it back,
    // and answers false having written nothing else when the bit is not
    // there. What the width then means - which input, which pulse, which
    // unit - wants a die that has the register.

    /// Channel `ch` (1..3) as a DUAL-EDGE capture: CAP_ED_CHy set and read
    /// back, then the channel disabled in CCER, CCyS = 11 with the
    /// prescaler and filter, and CCyE and CCyP as asked. False on a timer
    /// or a channel that has not got the register - and on a DIE that has
    /// not, which is what the read-back says.
    static bool dual_edge_capture(uint8_t ch,
                                  TimCapturePolarity polarity = TimCapturePolarity::rising,
                                  uint8_t filter = 0,
                                  TimCapturePrescaler prescaler = TimCapturePrescaler::every) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        if constexpr (!has_dual_edge_capture) {
            (void)ch;
            (void)polarity;
            (void)filter;
            (void)prescaler;
            return false;
        } else {
            if (ch < 1u || ch >= channels || filter > 15u) {
                return false;
            }
            TimRegs& t = regs();
            const uint16_t bit = static_cast<uint16_t>(tim_cap_ed_ch2 << (ch - 1u));
            t.AUX = static_cast<uint16_t>(t.AUX | bit);
            if ((t.AUX & bit) == 0u) {
                return false;   // this die's lot has no TIMx_AUX (the note above)
            }
            write_ccer(ch, false, false, false, false);
            const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
            uint16_t v = static_cast<uint16_t>(0x3u << shift);
            v = static_cast<uint16_t>(v | (static_cast<uint16_t>(prescaler) << (shift + 2u)));
            v = static_cast<uint16_t>(v | (static_cast<uint16_t>(filter) << (shift + 4u)));
            write_chctlr(ch, v);
            write_ccer(ch, true, polarity == TimCapturePolarity::falling, false, false);
            return true;
        }
    }

    /// CAP_ED_CHy cleared: the channel is an ordinary one again (its
    /// CCyS is the caller's to rewrite).
    static bool dual_edge_capture_off(uint8_t ch) {
        if constexpr (!has_dual_edge_capture) {
            (void)ch;
            return false;
        } else {
            if (ch < 1u || ch >= channels) {
                return false;
            }
            TimRegs& t = regs();
            t.AUX = static_cast<uint16_t>(t.AUX & ~(tim_cap_ed_ch2 << (ch - 1u)));
            return true;
        }
    }

    /// Whether channel `ch` is in the dual-edge capture, read back.
    static bool dual_edge(uint8_t ch) {
        if constexpr (!has_dual_edge_capture) {
            (void)ch;
            return false;
        } else {
            return ch >= 1u && ch < channels &&
                   (regs().AUX & static_cast<uint16_t>(tim_cap_ed_ch2 << (ch - 1u))) != 0u;
        }
    }

    // ---- the slave controller and the master output (14.4.2, 14.4.3) --------

    static bool slave(const TimSlaveConfig& c) {
        if constexpr (!has_slave_mode) {
            (void)c;
            return false;
        } else {
            const uint8_t sms = static_cast<uint8_t>(c.mode);
            if (sms > 7u) {
                return false;
            }
            if (c.trigger == TimTrigger::etr && !has_external_trigger) {
                return false;
            }
            if (c.mode == TimSlaveMode::gated && c.trigger == TimTrigger::ti1_edge) {
                return false;   // 14.4.3's own note: TI1F_ED has no level to gate on
            }
            TimRegs& t = regs();
            uint16_t v = static_cast<uint16_t>(t.SMCFGR & ~(tim_sms_mask | tim_ts_mask | tim_msm));
            v = static_cast<uint16_t>(v | sms);
            v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.trigger) << tim_ts_shift));
            if (c.master_slave) { v = static_cast<uint16_t>(v | tim_msm); }
            t.SMCFGR = v;
            return true;
        }
    }
    static TimSlaveMode slave_mode() {
        if constexpr (!has_slave_mode) {
            return TimSlaveMode::disabled;
        } else {
            return static_cast<TimSlaveMode>(regs().SMCFGR & tim_sms_mask);
        }
    }
    static TimTrigger slave_trigger() {
        if constexpr (!has_slave_mode) {
            return TimTrigger::itr0;
        } else {
            return static_cast<TimTrigger>((regs().SMCFGR & tim_ts_mask) >> tim_ts_shift);
        }
    }

    /**
     * CTLR2.MMS - what goes out on TRGO. THE SLAVE'S CLOCK MUST ALREADY
     * BE RUNNING before the master starts sending: a slave whose gate
     * is closed does not see the trigger that arrives while it is, and
     * a cascade configured the other way round counts one event short.
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
            if (code >= 3u && channels == 0u) {
                return false;   // a basic timer publishes reset, enable or update (16.4.2)
            }
            if (code >= 4u && static_cast<uint8_t>(code - 4u) >= channels) {
                return false;   // no OCyREF to publish
            }
            TimRegs& t = regs();
            t.CTLR2 = static_cast<uint16_t>(
                (t.CTLR2 & ~tim_mms_mask) |
                ((static_cast<uint16_t>(code) << tim_mms_shift) & tim_mms_mask));
            return true;
        }
    }
    static TimMasterMode master() {
        if constexpr (!has_master_mode) {
            return TimMasterMode::reset;
        } else {
            return static_cast<TimMasterMode>((regs().CTLR2 & tim_mms_mask) >> tim_mms_shift);
        }
    }

    /// CTLR2.TI1S: TI1 becomes the XOR of the first three channel
    /// inputs - a Hall-sensor interface, and a way to watch three pads
    /// on one capture.
    static bool ti1_xor(bool on) {
        if constexpr (!has_ti1_xor) {
            (void)on;
            return false;
        } else {
            TimRegs& t = regs();
            t.CTLR2 = static_cast<uint16_t>(on ? (t.CTLR2 | tim_ti1s) : (t.CTLR2 & ~tim_ti1s));
            return true;
        }
    }
    static bool ti1_xor() { return has_ti1_xor && (regs().CTLR2 & tim_ti1s) != 0u; }

    /// CTLR2.CCPC and CTLR2.CCUS: the channel configuration PRELOADED,
    /// transferred on a commutation - the three-phase motor shape. Both
    /// bits are printed in the general-purpose timers' register map too
    /// (15.4.2) and have nothing to act on there, so this verb is the
    /// advanced timer's.
    static bool preload_channels(bool on, bool on_trigger = false) {
        if constexpr (!has_break) {
            (void)on;
            (void)on_trigger;
            return false;
        } else {
            TimRegs& t = regs();
            uint16_t v = static_cast<uint16_t>(t.CTLR2 & ~(tim_ccpc | tim_ccus));
            if (on) { v = static_cast<uint16_t>(v | tim_ccpc); }
            if (on_trigger) { v = static_cast<uint16_t>(v | tim_ccus); }
            t.CTLR2 = v;
            return true;
        }
    }

    /// CTLR2.CCDS: a capture/compare DMA request is issued on the CC
    /// event (false, the reset state) or on the UPDATE event (true).
    static bool compare_dma_on_update(bool on) {
        if constexpr (channels == 0u) {
            (void)on;
            return false;
        } else {
            TimRegs& t = regs();
            t.CTLR2 = static_cast<uint16_t>(on ? (t.CTLR2 | tim_ccds) : (t.CTLR2 & ~tim_ccds));
            return true;
        }
    }

    // ---- break and dead time (14.4.18) --------------------------------------

    static bool break_dead_time(const TimBreakDeadTime& c) {
        if constexpr (!has_break) {
            (void)c;
            return false;
        } else {
            if (c.lock > 3u) {
                return false;
            }
            uint16_t v = c.dead_time;
            v = static_cast<uint16_t>(v | ((static_cast<uint16_t>(c.lock) << tim_lock_shift) &
                                           tim_lock_mask));
            if (c.off_state_idle) { v = static_cast<uint16_t>(v | tim_ossi); }
            if (c.off_state_run) { v = static_cast<uint16_t>(v | tim_ossr); }
            if (c.break_enable) { v = static_cast<uint16_t>(v | tim_bke); }
            if (c.break_active_high) { v = static_cast<uint16_t>(v | tim_bkp); }
            if (c.automatic_output_enable) { v = static_cast<uint16_t>(v | tim_aoe); }
            if (c.main_output_enable) { v = static_cast<uint16_t>(v | tim_moe); }
            regs().BDTR = v;
            return true;
        }
    }

    /// BDTR.MOE on its own: the master switch every output of the
    /// advanced timer passes through. A break clears it in hardware,
    /// and that is the whole of what a break does.
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

    /// The dead time in force, in tDTS ticks (timer clock ticks divided
    /// by CTLR1.CKD).
    static uint16_t dead_time_ticks() {
        if constexpr (!has_break) {
            return 0;
        } else {
            return tim_dead_time_ticks(static_cast<uint8_t>(regs().BDTR & tim_dtg_mask));
        }
    }

    // ---- the external trigger input (14.4.3) --------------------------------

    /// ETP / ETPS / ETF / ECE, the SMCFGR half a slave configuration
    /// leaves alone. 14.4.3: ETRP must be at or below a quarter of
    /// TIMxCLK - the caller's arithmetic, this file not knowing the
    /// source's rate.
    static bool external_trigger(const TimEtrConfig& c) {
        if constexpr (!has_external_trigger) {
            (void)c;
            return false;
        } else {
            if (!tim_etr_config_valid(c)) {
                return false;
            }
            TimRegs& t = regs();
            uint16_t v = static_cast<uint16_t>(
                t.SMCFGR & ~(tim_etp | tim_etps_mask | tim_etf_mask | tim_ece));
            if (c.inverted) { v = static_cast<uint16_t>(v | tim_etp); }
            v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.prescaler) << tim_etps_shift));
            v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.filter) << tim_etf_shift));
            if (c.clock_mode2) { v = static_cast<uint16_t>(v | tim_ece); }
            t.SMCFGR = v;
            return true;
        }
    }
    static bool external_clock_mode2() {
        return has_external_trigger && (regs().SMCFGR & tim_ece) != 0u;
    }

    // ---- the DMA burst engine (14.4.19, 14.4.20) ----------------------------
    //
    // ONE REQUEST, SEVERAL REGISTERS. The plain requests (DMAINTENR's
    // UDE/TDE/CCyDE) move one datum into one register; the burst engine
    // turns each request into a WALK of `length` consecutive registers
    // starting at `base`, all through the single address TIMx_DMAADR -
    // so a DMA channel whose peripheral address never changes writes
    // ATRLR, then CH1CVR, then CH2CVR, and starts again at the next
    // request. THE BASE IS A WORD OFFSET FROM TIMx_CTLR1, which is why
    // the offsets are named rather than computed.

    /// Point the burst engine at `length` registers starting at `base`.
    /// 14.4.19's DBL is the length MINUS ONE and this argument is the
    /// LENGTH, so nobody has to remember which.
    static bool dma_burst(TimBurstBase base, uint8_t length) {
        if (!has_dma_burst || length < 1u || length > 18u) {
            return false;
        }
        const uint8_t dba = static_cast<uint8_t>(base);
        if (dba > 31u) {
            return false;
        }
        regs().DMACFGR = static_cast<uint16_t>(
            dba | ((static_cast<uint16_t>(length - 1u) << tim_dbl_shift) & tim_dbl_mask));
        return true;
    }
    static TimBurstBase burst_base() {
        if constexpr (!has_dma_burst) {
            return TimBurstBase::ctlr1;
        } else {
            return static_cast<TimBurstBase>(regs().DMACFGR & tim_dba_mask);
        }
    }
    static uint8_t burst_length() {
        if constexpr (!has_dma_burst) {
            return 0u;
        } else {
            return static_cast<uint8_t>(((regs().DMACFGR & tim_dbl_mask) >> tim_dbl_shift) + 1u);
        }
    }
    /// The burst engine off: DBL and DBA back to zero, which is CTLR1
    /// with a length of one and is also the reset value.
    static void dma_burst_off() {
        if constexpr (has_dma_burst) {
            regs().DMACFGR = 0;
        }
    }

    /// The ONE address a burst DMA is pointed at: every access to it
    /// lands on `base + index`, the index being the controller's own.
    static volatile void* dmaadr_address() {
        return has_dma_burst ? static_cast<volatile void*>(&regs().DMAADR) : nullptr;
    }
    /// Where a DMA channel writes a duty into, and reads a capture
    /// from: CHxCVR itself.
    static volatile void* chcvr_address(uint8_t ch) {
        static_assert(channels > 0u, "brio Tim: a basic timer has no capture/compare channel");
        return ch < channels ? static_cast<volatile void*>(&regs().CHCVR[ch]) : nullptr;
    }
    /// Where a DMA channel reads this counter from. The register is
    /// thirty-two bits wide in the map and only the low half carries a
    /// count on every instance but the 32-bit one, so a channel that
    /// samples it names the width the timer HAS (`counter_bits`).
    static volatile void* cnt_address() { return static_cast<volatile void*>(&regs().CNT); }

private:
    static void write_chctlr(uint8_t ch, uint16_t value) {
        TimRegs& t = regs();
        const uint16_t mask = static_cast<uint16_t>(0xFFu << (8u * (ch & 1u)));
        if (ch < 2u) {
            t.CHCTLR1 = static_cast<uint16_t>((t.CHCTLR1 & ~mask) | value);
        } else {
            t.CHCTLR2 = static_cast<uint16_t>((t.CHCTLR2 & ~mask) | value);
        }
    }

    /// CCER's four bits of one channel: E, P, NE, NP at 4 x ch. `ne` is
    /// only ever true for a channel with a complementary output.
    static void write_ccer(uint8_t ch, bool e, bool p, bool ne, bool np) {
        TimRegs& t = regs();
        const uint8_t shift = static_cast<uint8_t>(4u * ch);
        uint16_t v = static_cast<uint16_t>(t.CCER & ~(0xFu << shift));
        if (e) { v = static_cast<uint16_t>(v | (1u << shift)); }
        if (p) { v = static_cast<uint16_t>(v | (2u << shift)); }
        if (ne) { v = static_cast<uint16_t>(v | (4u << shift)); }
        if (np) { v = static_cast<uint16_t>(v | (8u << shift)); }
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
 *   Lamp::setup(71);          // 72 MHz / 72 / 1001 = about 1 kHz
 *   Lamp::duty(250);          // a quarter
 *
 * `top` is a template parameter because PwmChannel requires `max` to be
 * a compile-time constant: a full scale that can move under a generic
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
    /// channel. On the advanced timer MOE is raised too, without which
    /// nothing reaches the pad at all (14.4.18) - the trap that makes a
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
    /// never cuts the pulse being produced.
    static void duty(uint16_t v) { (void)T::set_compare(ch, v > max ? max : v); }
    static uint16_t duty() { return static_cast<uint16_t>(T::compare(ch)); }
};

/**
 * TimPairPwm<T, ch, top>: a channel and its COMPLEMENT, with the dead
 * time the silicon inserts between them - the shape only the advanced
 * timer has here (TIM1, channels 1..3).
 *
 * `max` and `duty()` are the PwmChannel contract again, on the pair:
 * the two outputs are one actuator and one duty.
 */
template <class T, uint8_t ch, uint16_t top = 0xFFFF>
struct TimPairPwm {
    TimPairPwm() = delete;
    static_assert(T::has_break,
                  "brio TimPairPwm: this timer has no break/dead-time unit, so it has "
                  "no complementary output either (TIM1 has one; the general-purpose "
                  "timers of this family do not)");
    static_assert(ch < T::complementary_channels,
                  "brio TimPairPwm: this channel has no complementary output (channels "
                  "1..3 have one, channel 4 has not)");
    static_assert(top > 0, "a PWM period of zero has no duty to set");

    static constexpr uint16_t max = top;

    /// `dead_time` is a raw DTG code - `tim_dead_time_code(ticks)` turns
    /// a wanted number of tDTS ticks into one, always rounding UP. tDTS
    /// is TIMxCLK divided by `division`, and it is NOT divided by the
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
 * one capture arrangement - the chapter's PWM input mode (14.3.4).
 *
 * Two channels watch the same input: channel 1 directly on the rising
 * edge (the period), channel 2 indirectly on the falling one (the
 * width), and the slave controller RESETS the counter on every rising
 * edge so both readings are measured from the same origin. It costs
 * both channels, and 14.3.4's own note is why it is TI1's: only TI1FP1
 * and TI2FP2 reach the slave controller.
 *
 * `period_ticks()` and `width_ticks()` READ the capture registers,
 * which is what clears their flags; a handler that clears CCyIF without
 * reading has thrown the measurement away. What they return is what a
 * util/meter_sampler.hpp MeterLatch is fed with.
 */
template <class T>
struct TimPeriodMeter {
    TimPeriodMeter() = delete;
    static_assert(T::channels >= 2,
                  "brio TimPeriodMeter: PWM input mode watches one input with TWO "
                  "channels (RM 14.3.4)");
    static_assert(T::has_slave_mode,
                  "brio TimPeriodMeter: PWM input mode resets the counter on every "
                  "rising edge, which is the slave controller's job");

    /// `invert` measures the LOW time and the period from falling edges
    /// instead (the whole arrangement mirrored).
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

    static constexpr uint16_t period_flag = T::compare_flag(0);
    static constexpr uint16_t width_flag = T::compare_flag(1);
    static constexpr uint16_t overrun_flag = T::overcapture_flag(0);
    static constexpr uint16_t period_interrupt = T::compare_interrupt(0);
};

/**
 * TimIntervalMeter<T, ch>: the interval between consecutive edges of one
 * input, on ONE channel and a free-running counter - what a capture ISR
 * hands a util/meter_sampler.hpp MeterLatch.
 *
 *   using Edge = brio::TimIntervalMeter<brio::Tim<2>, 1>;
 *   extern "C" BRIO_CH32_INTERRUPT void tim2_handler() {
 *       if (brio::Tim<2>::isr() & Edge::capture_flag) {
 *           if (auto d = Edge::interval()) { Latch::store(*d); }
 *       }
 *   }
 *
 * The subtraction is done in the counter's own modulus, so a wrap
 * between two edges costs nothing as long as the interval is shorter
 * than one full counter period - the meter's whole contract, and the
 * caller's to arrange with the prescaler.
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
    /// from the previous one - nothing on the first edge, there being
    /// no interval yet.
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

    /// Throw the previous edge away: the next interval() returns
    /// nothing and starts a fresh pair. A measurement that spans a
    /// reconfiguration is not a measurement.
    static void restart() { have_last_ = false; }

    static constexpr uint16_t capture_flag = T::compare_flag(ch);
    static constexpr uint16_t overrun_flag = T::overcapture_flag(ch);
    static constexpr uint16_t capture_interrupt = T::compare_interrupt(ch);

private:
    static inline uint32_t last_ = 0;
    static inline bool have_last_ = false;
};

/**
 * TimEventCounter<T>: a timer whose CLOCK is another timer's trigger -
 * external clock mode 1 on an ITRx link (14.2.2.2). With the master
 * publishing its update event on TRGO, the count IS the number of
 * master periods, so a frequency is measured with no wire, no pad and
 * no CPU in the path.
 *
 *   TimEventCounter<Tim<3>>::setup(TimTrigger::itr1);   // TIM3 counts TIM2
 *
 * WHICH TIMER ITRx MEANS is per instance: `tim_trigger_index_for(slave,
 * master)` looks the link up so a caller names the MASTER and not a
 * number. This task enables the SLAVE, so the master is started after
 * it.
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
 * trigger is HIGH (slave gated mode, 14.4.3). Pointed at a master whose
 * TRGO is OC1REF - the PWM waveform itself - the count over a known
 * window IS the high time, so a DUTY CYCLE is measured internally, with
 * no pad and no sampling.
 *
 * Gated mode must never be used with TimTrigger::ti1_edge (14.4.3's own
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
 * every `period + 1` counter ticks, and an interrupt on it. It is what a
 * BASIC timer is for, on the parts that have one (TIM6 and TIM7 of the
 * CH32V303RC and VC); everywhere else it costs one of the timers with
 * channels, and nothing else about it is different.
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
    static constexpr uint16_t flag = T::update_flag;
};

/**
 * TimOnePulse<T, ch>: one pulse of a chosen width, after a chosen
 * delay, on a trigger - CTLR1.OPM plus a compare channel (14.3.8). The
 * counter stops itself at the next update, so the pulse happens once
 * per trigger and the timer costs nothing between them.
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
        // PWM2 is active while CNT >= CHxCVR, so the output rises at the
        // compare and falls at the update that stops the counter - the
        // chapter's own recipe for a delayed pulse.
        if (!T::output_channel(ch, {.mode = TimOutputMode::pwm2,
                                    .compare = delay,
                                    .active_low = active_low})) {
            return false;
        }
        // THE COMPARE IS PRELOADED (fact 3 of the file header), and a
        // one-pulse timer sees no update between here and its trigger -
        // so without this the pulse would run against the ACTIVE
        // compare register's old value and come out the whole period
        // wide. The update loads it and its own flag is cleared.
        T::update();
        T::clear_flags(T::update_flag);
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

/**
 * TimEncoder<T>: the quadrature interface (14.3.9) - two channel inputs
 * as the two tracks of an incremental encoder, the counter following
 * the shaft in both directions with no software in the path.
 *
 *   using Knob = brio::TimEncoder<brio::Tim<4>>;
 *   Knob::setup();                      // x4 counting, no filter
 *   const uint32_t at = Knob::count();
 *   const bool backwards = Knob::reversing();
 *
 * `encoder3` (the default) counts BOTH edges of both tracks - four
 * counts per quadrature cycle and the finest resolution; `encoder1` and
 * `encoder2` count one track's edges only. The counter runs modulo
 * ATRLR in both directions, so `period` is the encoder's own count per
 * revolution where there is one, and the whole sixteen bits otherwise.
 *
 * TI1FP1 and TI2FP2 come off the PADS, before the CCyS multiplexer, so
 * what a program stimulates the interface with is the pads and not the
 * channels: setup() leaves both channels as inputs, and a caller that
 * wants the timer to count a quadrature of its own making drives those
 * two pads AS GPIO OUTPUTS (the file header's third wireless
 * instrument). The other strata's trick - the channels' own forced
 * output stages - is measured NOT to work here: in an encoder mode a
 * channel's output stage no longer reaches its pad.
 */
template <class T>
struct TimEncoder {
    TimEncoder() = delete;
    static_assert(T::has_encoder,
                  "brio TimEncoder: this timer has no quadrature interface");
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
    /// CTLR1.DIR as the SILICON keeps it: which way the last edge moved
    /// the counter.
    static bool reversing() { return T::direction() == TimDirection::down; }
};

} // namespace brio
