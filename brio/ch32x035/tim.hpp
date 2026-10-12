/*
 * tim.hpp
 *
 * The CH32X035's two ADVANCED-CONTROL timers, TIM1 and TIM2 (RM ch. 12):
 * one block twice, the STM32F1's advanced timer under WCH's register
 * names - sixteen-bit registers at a four-byte stride, the four CHxCVR 32
 * bits wide - with a SPEC register at 0x50 the F1 has not got. TIM3 is
 * chapter 13's general-purpose block, a different one, and has no driver
 * here.
 *
 *  Tim<n>      the RESOURCE, n = 1 or 2: the time base (prescaler,
 *              counter, auto-reload with its preload, the repetition
 *              counter, the counting modes, one-pulse), the four channels
 *              as OUTPUTS in every OCxM mode with their complementary
 *              outputs and MOE, the flags, the interrupt and DMA-request
 *              enables, the ISR body, the remap.
 *  TimPad<pad> a pad handed to a timer signal, the pad named from the
 *              column afio.hpp states (afio_tim1_pads / afio_tim2_pads),
 *              the same source Tim<n>::remap(code) programs.
 *
 * THE SUBSET. What is driven is what a waveform engine needs - output
 * compare with its toggle and its forced levels, one-pulse mode, the
 * complementary outputs behind MOE (on the QFN20 the only timer outputs
 * that reach PB0 and PB1 are TIM1's C2N and C3N under its code 001), a
 * compare's DMA request, the remap - and the time base under them. The
 * channels' INPUT face (capture, the filters, PWM input), the slave
 * controller and TRGO, the external trigger, the encoder modes, the
 * brake unit with the dead time and the off-state controls (OSSI, OSSR,
 * the idle levels OISx), the COM event, the DMA burst and SPEC's
 * alternate enable are not: each is born with its first user and named
 * in docs/ch32x035/tim.md's gap list. A configuration struct carries no
 * field for them, so nothing here writes their bits.
 *
 * THE OUTPUTS HOLD WHILE THE COUNTER IS STOPPED. Chapter 12 says nothing
 * of what an output does with CEN clear; the CH32X035F8U6 answered
 * (docs/ch32x035/tim.md): every output - a channel's and a complementary
 * one - keeps DRIVING the level it had when the counter stopped, and a
 * mode written meanwhile, a forced level included, reaches the pad only
 * once the counter runs. A timer that has never run since its reset holds
 * its outputs LOW. So a program that wants a forced level on a pad runs
 * the counter first - what the CH32V00x's TIM1 does too, measured there.
 *
 * MOE, THE GATE (12.4.18): set, the enabled outputs carry OCxREF; clear,
 * with OSSI at its reset value, they are LET GO - the pad floats (a level
 * charged onto it stays). That is CTLR1's BIT 13 at work: 12.4.1 calls it
 * "Reserved" with a reset value of zero, and on the CH32X035F8U6 it reads
 * ONE out of reset, writable both ways; cleared, an output MOE disables is
 * DRIVEN at its idle level OISx / OISxN instead - the float-or-drive
 * choice the CH32V00x documents as OE_MODE, in the same position, applied
 * here to the MOE-disabled state and not to the stopped counter (the hold
 * above is the same at either value). WCH's library keeps the bit as it
 * reads (TIM_TimeBaseInit is a read-modify-write), and so does
 * configure(): a reserved bit is not this driver's to move. The idle
 * levels, OSSI and OSSR belong to the brake unit and are not driven.
 * Bit 12 is CMP_BK, the brake from a comparator, TIM1's alone and not
 * driven. What the block has that the F1 has not: CAPLVL and CAPOV at the
 * top of CTLR1 (capture features, not driven) and the SPEC register (not
 * driven).
 *
 * A SOFTWARE COMPARE EVENT DOES NOTHING TO AN OUTPUT CHANNEL. 12.4.6 says
 * CCxG on an output channel sets CCxIF and raises its interrupt and DMA
 * request; on the CH32X035F8U6 it raised neither, counter stopped or
 * running (docs/ch32x035/tim.md), while a real match does both. The
 * driver has no verb for it: a compare's request comes from a match.
 *
 * THE COMPLEMENTARY OUTPUT ALONE. CCxNE set with CCxE clear drives OCxN
 * from OCxREF through the polarity CCxNP and no dead time - the F1's
 * output-control table, which chapter 12 does not print; the CH32X035F8U6
 * answered it: C2NE alone puts a forced-active channel's level, high, on
 * C2N, and a toggle moves it.
 *
 * THE STATUS REGISTER IS WRITE-ZERO-TO-CLEAR (12.4.5's RW0): clearing is
 * `INTFR = ~flags`, a plain store, and a flag that arrives between the
 * read and the store survives.
 *
 * THE PRESCALER AND THE AUTO-RELOAD ARE SHADOWED (12.2.3): PSC is taken
 * at the next update event, ATRLR too under ARPE. configure() ends with
 * SWEVGR.UG - a software update that loads both shadows - and clears the
 * UIF it raised.
 *
 * A COMPARE REGISTER IS PRELOADED BY DEFAULT HERE, a choice: OCxPE is
 * clear out of reset and every output channel this driver configures sets
 * it unless TimChannelConfig::preload says false - which is what a value
 * written by DMA at each match needs (12.3.3's step 4: "keeping OCxPE to
 * 0").
 *
 * THE CLOCK. CK_INT is HCLK - the series has no peripheral-bus prescaler
 * (RM 3.3) - so `tim_clock_hz(clock)` is the Clock's hz, and a static
 * clock only.
 *
 * THE VECTORS. Four a timer (table 7-1): TIM1 at 34..37 (brake, update,
 * trigger/commutation, capture-compare), TIM2 at 38 (update) and 51..53
 * (capture-compare, trigger/commutation, brake). THE DMA REQUESTS are
 * dma_engine.hpp's table: TIM1's channels 1..4 on DMA channels 2, 3, 6
 * and 4, its update on 5; TIM2's on 5, 7, 1 and 7, its update on 2.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/device.hpp"
#include "ch32x035/dma_engine.hpp"
#include "ch32x035/pin.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// The registers (tables 12-3, 12-4)
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
    volatile uint16_t RPTCR;      ///< 0x30
    uint16_t RESERVED12;
    volatile uint32_t CHCVR[4];   ///< 0x34..0x40, bit 16 the CAPLVL level
    volatile uint16_t BDTR;       ///< 0x44
    uint16_t RESERVED13;
    volatile uint16_t DMACFGR;    ///< 0x48
    uint16_t RESERVED14;
    volatile uint16_t DMAADR;     ///< 0x4c
    uint16_t RESERVED15;
    volatile uint16_t SPEC;       ///< 0x50
    uint16_t RESERVED16;
};

inline constexpr uint32_t tim1_base = pb2_base + 0x2C00;
inline constexpr uint32_t tim2_base = pb1_base + 0x0000;

constexpr uint32_t tim_base_for(uint8_t n) {
    return n == 1 ? tim1_base : n == 2 ? tim2_base : 0;
}

// CTLR1 (12.4.1)
inline constexpr uint16_t tim_cen      = 1u << 0;
inline constexpr uint16_t tim_udis     = 1u << 1;
inline constexpr uint16_t tim_urs      = 1u << 2;
inline constexpr uint16_t tim_opm      = 1u << 3;
inline constexpr uint16_t tim_dir      = 1u << 4;
inline constexpr uint16_t tim_cms_mask = 3u << 5;
inline constexpr uint16_t tim_arpe     = 1u << 7;
inline constexpr uint16_t tim_ckd_mask = 3u << 8;
inline constexpr uint16_t tim_cmp_bk   = 1u << 12;   ///< TIM1 only; not driven
inline constexpr uint16_t tim_ctlr1_reserved13 = 1u << 13;   ///< reads one out of reset (the file header)
// BDTR (12.4.18)
inline constexpr uint16_t tim_ossi     = 1u << 10;   ///< not driven
inline constexpr uint16_t tim_capov    = 1u << 14;   ///< not driven
inline constexpr uint16_t tim_caplvl   = 1u << 15;   ///< not driven
// CTLR2 (12.4.2): OISx at 8 + 2x, OISxN at 9 + 2x
inline constexpr uint16_t tim_ccds     = 1u << 3;
// DMAINTENR (12.4.4) and INTFR (12.4.5) share their low positions
inline constexpr uint16_t tim_uie      = 1u << 0;
inline constexpr uint16_t tim_cc1ie    = 1u << 1;
inline constexpr uint16_t tim_ude      = 1u << 8;
inline constexpr uint16_t tim_cc1de    = 1u << 9;
inline constexpr uint16_t tim_uif      = 1u << 0;
inline constexpr uint16_t tim_cc1if    = 1u << 1;
inline constexpr uint16_t tim_cc1of    = 1u << 9;
// SWEVGR (12.4.6)
inline constexpr uint16_t tim_ug       = 1u << 0;
inline constexpr uint16_t tim_cc1g     = 1u << 1;
// CCER (12.4.9): four bits a channel - E, P, NE, NP
inline constexpr uint16_t tim_cc1e     = 1u << 0;
inline constexpr uint16_t tim_cc1p     = 1u << 1;
inline constexpr uint16_t tim_cc1ne    = 1u << 2;
inline constexpr uint16_t tim_cc1np    = 1u << 3;
// BDTR (12.4.18)
inline constexpr uint16_t tim_moe      = 1u << 15;

// =============================================================================
// The facts, keyed by instance (12.1)
// =============================================================================

constexpr bool tim_present(uint8_t n) { return n == 1 || n == 2; }
constexpr uint8_t tim_channels(uint8_t n) { return tim_present(n) ? 4u : 0u; }
/// Channels 1..3 carry a complementary output; channel 4 has one pin
/// (12.3.7).
constexpr uint8_t tim_complementary_channels(uint8_t n) { return tim_present(n) ? 3u : 0u; }

/// The column a remap code puts an instance's signals on.
constexpr TimPadSet tim_pads_for(uint8_t n, uint8_t code) {
    return n == 1 ? afio_tim1_pads(code) : n == 2 ? afio_tim2_pads(code) : TimPadSet{};
}

/// Is that pad one of the two-wire debug port's (PC18 SWDIO, PC19 SWCLK)?
constexpr bool tim_pad_on_debug_port(Pad p) {
    return p.port == 'C' && (p.pin == 18u || p.pin == 19u);
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

/// CHCTLRx.OCxM (12.4.7): what OCxREF does.
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

struct TimConfig {
    uint16_t prescaler = 0;               ///< PSC: the counter clock is HCLK / (PSC + 1)
    uint16_t period = 0xFFFF;             ///< ATRLR
    TimDirection direction = TimDirection::up;
    TimAlignment alignment = TimAlignment::edge;
    TimClockDivision clock_division = TimClockDivision::div1;
    bool auto_reload_preload = false;     ///< ARPE
    bool one_pulse = false;               ///< OPM: the counter stops at the next update
    bool update_disable = false;          ///< UDIS
    bool update_on_overflow_only = false; ///< URS
    uint8_t repetition = 0;               ///< RPTCR
};

/// One channel as an OUTPUT.
struct TimChannelConfig {
    TimOutputMode mode = TimOutputMode::frozen;
    uint16_t compare = 0;                  ///< CHxCVR
    bool preload = true;                   ///< OCxPE - see the file header
    bool active_low = false;               ///< CCER.CCxP
    bool complementary_active_low = false; ///< CCER.CCxNP
    bool enable = true;                    ///< CCER.CCxE
    bool complementary_enable = false;     ///< CCER.CCxNE (channels 1..3)
};

/// CK_INT is HCLK: no peripheral-bus prescaler on this series.
template <typename Clock>
constexpr uint32_t tim_clock_hz(Clock) {
    static_assert(Clock::is_static,
                  "brio Tim: a timer counts HCLK and every period, prescaler and compare it "
                  "holds is in those cycles - a DynamicClock would move all of them at once "
                  "and no rebase() can promise the same periods, so the timers take a static "
                  "clock only");
    return Clock::hz;
}

/**
 * A pad handed to a timer signal. The pad is named from its column -
 * `TimPad<afio_tim1_pads(1).ch2n>` - so the program states the code it
 * remaps to and the pad that code gives in one place. A pad this package
 * does not bond does not compile; a pad of the debug port is REFUSED at
 * run time while the port is the probe's (claim() answers false and
 * touches nothing), the USART transport's rule for its columns.
 */
template <Pad pad>
struct TimPad {
    TimPad() = delete;
    static_assert(pad.valid(), "brio TimPad: that column carries no such signal");
    static_assert(pad_bonded(pad), "brio TimPad: this package does not bond that pad (parts/<part>.hpp)");

    using pin = Pin<pad.port, pad.pin>;
    static constexpr Pad selection = pad;

    /// The pad as the timer's output (alternate push-pull, 8.2.10's table
    /// 8-1 for a compare channel and a complementary one alike).
    static bool claim() {
        if (tim_pad_on_debug_port(pad) && Afio::debug_port_enabled()) {
            return false;
        }
        pin::function();
        return true;
    }
    static void release() { pin::release(); }
};

// =============================================================================
// Tim<n>: the resource
// =============================================================================

/**
 * Tim<n>: one advanced-control timer, n = 1 or 2.
 *
 *   using T1 = brio::Tim<1>;
 *   T1::init();                                          // gate on, reset
 *   (void)T1::remap(1);                                  // C2N on PB0
 *   (void)T1::configure({.prescaler = 47, .period = 999});
 *   (void)T1::output_channel(1, {.mode = brio::TimOutputMode::toggle, .compare = 500,
 *                                .enable = false, .complementary_enable = true});
 *   T1::main_output(true);
 *   T1::enable(true);                                    // running: the outputs follow
 *   (void)brio::TimPad<brio::afio_tim1_pads(1).ch2n>::claim();
 *
 * Channels are 0-based (channel 1 of the chapter is 0), brio's timer
 * surface; every verb that names a channel checks it and answers false,
 * writing nothing, for one the block has not got.
 */
template <uint8_t n>
class Tim {
public:
    Tim() = delete;
    static_assert(tim_present(n), "brio Tim: the CH32X035's advanced-control timers are TIM1 and TIM2 "
                                  "(TIM3 is chapter 13's general-purpose block, with no driver here)");

    static constexpr uint8_t instance = n;
    static constexpr uint16_t max_period = 0xFFFF;
    static constexpr uint8_t channels = tim_channels(n);
    static constexpr uint8_t complementary_channels = tim_complementary_channels(n);
    static constexpr Bus bus = n == 1 ? Bus::pb2 : Bus::pb1;
    static constexpr uint32_t gate = n == 1 ? rcc_pb2_tim1 : rcc_pb1_tim2;
    static constexpr Remap remap_field = n == 1 ? Remap::tim1 : Remap::tim2;

    static constexpr Irq update_irq() { return n == 1 ? Irq::tim1_up : Irq::tim2_up; }
    static constexpr Irq cc_irq() { return n == 1 ? Irq::tim1_cc : Irq::tim2_cc; }
    static constexpr Irq trigger_irq() { return n == 1 ? Irq::tim1_trg_com : Irq::tim2_trg_com; }
    static constexpr Irq break_irq() { return n == 1 ? Irq::tim1_brk : Irq::tim2_brk; }

    /// The DMA channel a compare's request lands on (dma_engine.hpp's
    /// table), 0 for a channel the block has not got.
    static constexpr uint8_t compare_dma_channel(uint8_t ch) {
        if constexpr (n == 1) {
            return ch == 0 ? dma_request_channel(DmaRequest::tim1_ch1)
                 : ch == 1 ? dma_request_channel(DmaRequest::tim1_ch2)
                 : ch == 2 ? dma_request_channel(DmaRequest::tim1_ch3)
                 : ch == 3 ? dma_request_channel(DmaRequest::tim1_ch4) : uint8_t{0};
        } else {
            return ch == 0 ? dma_request_channel(DmaRequest::tim2_ch1)
                 : ch == 1 ? dma_request_channel(DmaRequest::tim2_ch2)
                 : ch == 2 ? dma_request_channel(DmaRequest::tim2_ch3)
                 : ch == 3 ? dma_request_channel(DmaRequest::tim2_ch4) : uint8_t{0};
        }
    }
    static constexpr uint8_t update_dma_channel =
        n == 1 ? dma_request_channel(DmaRequest::tim1_up) : dma_request_channel(DmaRequest::tim2_up);

    static TimRegs& regs() { return *reinterpret_cast<TimRegs*>(tim_base_for(n)); }

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, gate);
        } else {
            Rcc::disable(bus, gate);
        }
    }
    static bool bus_clock() { return Rcc::enabled(bus, gate); }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() { (void)Rcc::reset(bus, gate); }
    static void init() {
        bus_clock(true);
        reset();
    }
    static void release() {
        enable(false);
        reset();
        bus_clock(false);
    }

    /// The instance's nine signals moved to a column of afio.hpp's table
    /// (TIM1_RM or TIM2_RM, 8.3.2.1): the pads tim_pads_for(n, code) names
    /// are then the ones a TimPad claims. Refused for a code the field has
    /// not got or whose column this package bonds no pad of.
    static bool remap(uint8_t code) { return Afio::remap(remap_field, code); }
    static uint8_t remap() { return Afio::remap_code(remap_field); }
    static constexpr TimPadSet column(uint8_t code) { return tim_pads_for(n, code); }

    // ---- the time base ---------------------------------------------------------

    static constexpr bool config_valid(const TimConfig& c) {
        return c.period != 0u && c.clock_division != static_cast<TimClockDivision>(3);
    }

    /// Write the time base and leave the counter STOPPED: PSC, ATRLR and
    /// RPTCR written, CTLR1 assembled in one store (the capture and
    /// comparator bits zero, the reserved bit 13 as it reads - the file
    /// header), the slave controller at its reset (SMCFGR zero: a
    /// free-running counter), CNT zeroed, then UG loads the shadows and the
    /// UIF it raised is cleared.
    static bool configure(const TimConfig& c) {
        if (!config_valid(c)) {
            return false;
        }
        TimRegs& t = regs();
        t.CTLR1 = static_cast<uint16_t>(t.CTLR1 & ~tim_cen);
        t.SMCFGR = 0;
        t.PSC = c.prescaler;
        t.ATRLR = c.period;
        t.RPTCR = c.repetition;
        uint16_t v = static_cast<uint16_t>(t.CTLR1 & tim_ctlr1_reserved13);
        if (c.direction == TimDirection::down) { v |= tim_dir; }
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.alignment) << 5);
        v |= static_cast<uint16_t>(static_cast<uint16_t>(c.clock_division) << 8);
        if (c.auto_reload_preload) { v |= tim_arpe; }
        if (c.one_pulse) { v |= tim_opm; }
        if (c.update_disable) { v |= tim_udis; }
        if (c.update_on_overflow_only) { v |= tim_urs; }
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
    /// PSC as written, taken at the next update event (12.2.3).
    static void set_prescaler(uint16_t p) { regs().PSC = p; }
    static uint16_t period() { return regs().ATRLR; }
    static bool set_period(uint16_t v) {
        if (v == 0u) {
            return false;
        }
        regs().ATRLR = v;
        return true;
    }
    static uint8_t repetition() { return static_cast<uint8_t>(regs().RPTCR); }
    static void set_repetition(uint8_t v) { regs().RPTCR = v; }
    static bool one_pulse() { return (regs().CTLR1 & tim_opm) != 0u; }

    // ---- software events (12.4.6) ------------------------------------------------

    /// UG: the counter and the prescaler reinitialized, the shadows
    /// loaded, UIF raised (URS clear) and, under UDE, the update's DMA
    /// request (measured: one item a UG on both timers).
    static void update() { regs().SWEVGR = tim_ug; }

    // ---- flags, interrupts and DMA requests (12.4.4, 12.4.5) -------------------

    static constexpr uint16_t update_flag = tim_uif;
    static constexpr uint16_t compare_flag(uint8_t ch) { return static_cast<uint16_t>(tim_cc1if << ch); }

    static uint16_t flags() { return regs().INTFR; }
    static bool flag(uint16_t mask) { return (regs().INTFR & mask) != 0u; }
    /// Write-zero-to-clear: a plain store of the complement.
    static void clear_flags(uint16_t mask) { regs().INTFR = static_cast<uint16_t>(~mask); }

    static constexpr uint16_t update_interrupt = tim_uie;
    static constexpr uint16_t compare_interrupt(uint8_t ch) { return static_cast<uint16_t>(tim_cc1ie << ch); }
    static constexpr uint16_t update_dma = tim_ude;
    /// CCxDE: the compare's DMA request, on compare_dma_channel(ch).
    static constexpr uint16_t compare_dma(uint8_t ch) { return static_cast<uint16_t>(tim_cc1de << ch); }

    /// Set or clear enables of DMAINTENR: the update and compare
    /// interrupts and DMA requests above (a read-modify-write of the one
    /// register; its other bits, the trigger's, the COM event's and the
    /// brake's, are not driven).
    static void interrupts(uint16_t mask, bool on) {
        TimRegs& t = regs();
        t.DMAINTENR = static_cast<uint16_t>(on ? (t.DMAINTENR | mask) : (t.DMAINTENR & ~mask));
    }
    static uint16_t interrupts() { return regs().DMAINTENR; }

    /// The ISR body, for any of this timer's vectors: the flags that were
    /// BOTH raised and enabled, cleared - exactly those - and handed back.
    [[gnu::always_inline]] static uint16_t isr() {
        TimRegs& t = regs();
        const uint16_t enabled_flags = static_cast<uint16_t>(t.DMAINTENR & 0xFFu);
        const uint16_t hit = static_cast<uint16_t>(t.INTFR & enabled_flags);
        if (hit != 0u) {
            t.INTFR = static_cast<uint16_t>(~hit);
        }
        return hit;
    }

    /// The compare register's address, for a DMA engine (the register is
    /// 32 bits; a word or a half-word beat lands in its value).
    static volatile void* ccr_address(uint8_t ch) {
        return ch < channels ? static_cast<volatile void*>(&regs().CHCVR[ch]) : nullptr;
    }

    // ---- the channels as outputs (12.3.3, 12.3.4, 12.3.7, 12.3.9) -------------------

    static uint16_t compare(uint8_t ch) {
        return ch < channels ? static_cast<uint16_t>(regs().CHCVR[ch] & 0xFFFFu) : uint16_t{0};
    }
    static bool set_compare(uint8_t ch, uint16_t v) {
        if (ch >= channels) {
            return false;
        }
        regs().CHCVR[ch] = v;
        return true;
    }

    /// The channel as an OUTPUT, whole: the mode and the preload (CCxS 00),
    /// the compare value, the polarities and the enables. A complementary
    /// enable or polarity on channel 4 is refused.
    static bool output_channel(uint8_t ch, const TimChannelConfig& c) {
        if (ch >= channels) {
            return false;
        }
        if ((c.complementary_enable || c.complementary_active_low) && ch >= complementary_channels) {
            return false;
        }
        TimRegs& t = regs();
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u));
        uint16_t v = static_cast<uint16_t>((static_cast<uint16_t>(c.mode) & 0x7u) << (shift + 4u));
        if (c.preload) { v |= static_cast<uint16_t>(1u << (shift + 3u)); }
        write_chctlr(ch, v);
        t.CHCVR[ch] = c.compare;
        write_ccer(ch, c.enable, c.active_low, c.complementary_enable, c.complementary_active_low);
        return true;
    }

    /// OCxM alone, the channel's other bits as they stand: what a waveform
    /// moves between toggling and a forced level, one read-modify-write of
    /// CHCTLRx and nothing else.
    static bool output_mode(uint8_t ch, TimOutputMode m) {
        if (ch >= channels) {
            return false;
        }
        TimRegs& t = regs();
        const uint8_t shift = static_cast<uint8_t>(8u * (ch & 1u) + 4u);
        volatile uint16_t& r = ch < 2u ? t.CHCTLR1 : t.CHCTLR2;
        r = static_cast<uint16_t>((r & ~(0x7u << shift)) | ((static_cast<uint16_t>(m) & 0x7u) << shift));
        return true;
    }
    static TimOutputMode output_mode(uint8_t ch) {
        if (ch >= channels) {
            return TimOutputMode::frozen;
        }
        const uint16_t r = ch < 2u ? regs().CHCTLR1 : regs().CHCTLR2;
        return static_cast<TimOutputMode>((r >> (8u * (ch & 1u) + 4u)) & 0x7u);
    }

    static bool channel_enable(uint8_t ch, bool on) {
        if (ch >= channels) {
            return false;
        }
        set_ccer_bit(static_cast<uint16_t>(tim_cc1e << (4u * ch)), on);
        return true;
    }
    static bool channel_enabled(uint8_t ch) {
        return ch < channels && (regs().CCER & (tim_cc1e << (4u * ch))) != 0u;
    }
    static bool complementary_enable(uint8_t ch, bool on) {
        if (ch >= complementary_channels) {
            return false;
        }
        set_ccer_bit(static_cast<uint16_t>(tim_cc1ne << (4u * ch)), on);
        return true;
    }
    static bool complementary_enabled(uint8_t ch) {
        return ch < complementary_channels && (regs().CCER & (tim_cc1ne << (4u * ch))) != 0u;
    }

    /// MOE (12.4.18): without it no output of this timer is driven - each
    /// enabled one is let go, floating, under the reset values of OSSI and
    /// of CTLR1's bit 13 (the file header).
    static void main_output(bool on) {
        TimRegs& t = regs();
        t.BDTR = static_cast<uint16_t>(on ? (t.BDTR | tim_moe) : (t.BDTR & ~tim_moe));
    }
    static bool main_output() { return (regs().BDTR & tim_moe) != 0u; }

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
    static void set_ccer_bit(uint16_t bit, bool on) {
        TimRegs& t = regs();
        t.CCER = static_cast<uint16_t>(on ? (t.CCER | bit) : (t.CCER & ~bit));
    }
};

} // namespace brio
