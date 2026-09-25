/*
 * rtc.hpp
 *
 * The real-time clock and the backup domain around it (RM ch. 6 for the
 * counter, ch. 4 for the backup registers and the tamper input, 3.3.3
 * and 3.4.9 for the oscillator and the clock select, 2.4.1 for the one
 * bit that unlocks all of it):
 *
 *   RtcDomain   the DOMAIN: PWR_CTLR's write enable and the whole of
 *               RCC_BDCTLR - the LSE with its bypass, RTCSEL one-way
 *               with BDRST the way back, RTCEN.
 *   Rtc         the COUNTER: a 32-bit up-counter behind a 20-bit
 *               prescaler, with an alarm, a second event and an
 *               overflow event.
 *   Bkp         the BACKUP REGISTERS, the tamper input that wipes them,
 *               and the three things this block can put on the TAMPER
 *               pad.
 *
 * THIS IS NOT THE CALENDAR RTC of the newer ST parts. There is no BCD
 * register, no date, no sub-second, no wake-up timer: what the silicon
 * keeps is a NUMBER, one that counts a tick whose period the prescaler
 * decides, and the calendar is the program's own arithmetic over it.
 * The counter and the prescaler live in the backup domain and survive
 * a system reset, a Standby wake and - with a battery on VBAT - the
 * loss of VDD; the CONTROL register does not, which is why a program
 * that comes back finds its count intact and its interrupts disarmed.
 *
 * THREE GATES BEFORE A SINGLE REGISTER ANSWERS. 4.2: the PWR block's
 * bus clock and the BKP block's must be open in RCC_PB1PCENR, and
 * PWR_CTLR.DBP must be set, before the backup registers and the RTC's
 * can be reached at all. `RtcDomain::unlock()` is the second of those
 * and opens the first on the way; `Bkp::open()` adds the third.
 *
 * TWO DISCIPLINES THE CHAPTER IMPOSES, AND EVERY VERB HERE OBEYS THEM
 * (6.2.3):
 *
 *  - WRITING is a WINDOW. Wait for RTOFF, set CNF, write one or more of
 *    the four backup-domain registers, clear CNF, wait for RTOFF again.
 *    Between the last two steps the write is crossing into the RTC's
 *    own clock domain, and a second write started there is lost. Every
 *    verb below that touches the prescaler, the counter or the alarm
 *    opens and closes that window; `configure()` opens ONE window for
 *    all three, which is what the chapter's "one or more" is for.
 *  - READING needs the domains SYNCHRONIZED. The RTC's registers are
 *    clocked by RTCCLK and read over PB1, so after a system reset, a
 *    power reset or a wake from Stop or Standby the values the bus
 *    sees are stale until an RTCCLK edge has passed. RSF is the flag
 *    that says an edge has: `synchronize()` clears it and waits for the
 *    hardware to set it, and that is what a program calls ONCE after a
 *    reset before it believes a count. Measured at 14 us on the
 *    crystal, inside the thirty a period would cost.
 *
 *    THE FLAGS CROSS THAT BOUNDARY FASTER THAN THE COUNTER DOES.
 *    Measured on the overflow: at the instant OWF stands, the count the
 *    bus reads is still the one before the wrap, and the wrapped value
 *    appears a moment later. A handler that wants to know where the
 *    counter is READS IT, and does not infer it from the event.
 *
 * THE COUNTER IS THIRTY-TWO BITS IN TWO SIXTEEN-BIT REGISTERS, and the
 * chapter does not say what happens when the low half rolls over
 * between the two reads. It is a carry race whatever the silicon
 * intends, so `count()` reads the high half, the low half and the high
 * half again and retries while the first and the last disagree - the
 * driver's own care, not a rule of the chapter. MEASURED at 32768 ticks
 * a second over four million reads and a roll-over of the low half: the
 * retry never fired and no read was ever torn - but THE COUNT DOES STEP
 * BACKWARDS by up to three ticks now and then, which is a read crossing
 * back out of the RTC's clock domain and seeing a stale copy. A program
 * that reads this counter faster than it ticks cannot assume the number
 * only grows.
 *
 * THE ALARM AND THE PRESCALER RELOAD ARE WRITE-ONLY: RTC_ALRMH/L and
 * RTC_PSCRH/L cannot be read back at all (6.3.3, 6.3.4, 6.3.9, 6.3.10).
 * What CAN be read is the prescaler's live DOWN-counter, RTC_DIVH/L,
 * which is `divider()` - and which is also the fraction of the current
 * tick, the finest time this block offers.
 *
 * THE ALARM REACHES TWO VECTORS. The second, alarm and overflow events
 * share the RTC's own line (`Irq::rtc`); the ALARM ALONE also reaches
 * EXTI line 17 and its own line (`Irq::rtc_alarm`), which is the path
 * that survives a Stop, because an EXTI line is asynchronous and needs
 * no clock. `wake_line` names that line, `arm_wake()` configures it
 * and `alarm_isr()` is the body of the handler on it - clearing the
 * EXTI flag through exti.hpp and the RTC's own ALRF, because a flag
 * left standing re-enters the handler for ever. MEASURED: the line
 * fires WITH RTC_CTLRH's own alarm enable CLEAR, so the event leaves
 * the peripheral by itself and the wake costs no vector of the RTC's -
 * the opposite direction of this family's other finding, that an EXTI
 * line INTO a peripheral needs that peripheral's event enable.
 *
 * AND THE ALARM FIRES AS THE COUNTER LEAVES THE VALUE IT WAS ARMED AT.
 * Measured with the handler reading the count: the number it sees is
 * ALR + 1, not ALR. A program that wants something done AT a count arms
 * the alarm at that count and finds the counter one further on.
 *
 * WHAT RTCCLK MAY BE, AND THE ONE NUMBER THAT IS NOT KNOWABLE. RTCSEL
 * takes the LSE, the LSI or the HSE divided (3.4.9). The first two are
 * plain; the third is not, because on the CH32V20x_D6 the division is
 * 512 OR 128 and the register description keys the choice on the LOT
 * NUMBER. `device::rtc_hse_div` states both, `rtc_hse_divider_known`
 * says whether they agree, and a program that clocks the RTC from the
 * crystal on a part where they do not agree MEASURES which one it has -
 * against the second event, which is the only ruler in the room. (One
 * CH32V203C8 measured on this bench divides by 128: a reload asked for
 * one hertz out of 15625 Hz ticked at four, which puts RTCCLK at
 * 62500 Hz out of an 8 MHz crystal. It is a fact of that die's lot and
 * not of the part, which is exactly why the driver measures instead of
 * choosing.)
 *
 * WHAT IS NOT HERE. PWR_CTLR's other bits - the sleep modes, the PVD,
 * the RAM retention - belong to the power chapter and its own file;
 * the register itself is device.hpp's, like RCC's, because two
 * chapters reach it and one of its bits is the door to this one.
 * The LSI stays clock.hpp's (`Rcc::lsi_start()`), because it is not a
 * backup-domain oscillator: only its SELECTION as RTCCLK is here.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/exti.hpp"

namespace brio {

// =============================================================================
// The domain (RM 3.4.9)
// =============================================================================

/// RCC_BDCTLR's bits (3.4.9). Every one of them lives in the backup
/// domain: they survive a system reset and are cleared only by a backup
/// domain reset.
inline constexpr uint32_t rcc_lseon       = 1UL << 0;
inline constexpr uint32_t rcc_lserdy      = 1UL << 1;
inline constexpr uint32_t rcc_lsebyp      = 1UL << 2;    ///< writable only with LSEON clear
inline constexpr uint32_t rcc_rtcsel_mask = 0x3UL << 8;
inline constexpr uint32_t rcc_rtcsel_shift = 8;
inline constexpr uint32_t rcc_rtcen       = 1UL << 15;
inline constexpr uint32_t rcc_bdrst       = 1UL << 16;

/// What RTCCLK may be (3.4.9's RTCSEL). `none` is the reset state, and
/// RTCEN is forced to zero by hardware while it stands.
enum class RtcClockSource : uint8_t {
    none = 0,
    lse = 1,           ///< the 32.768 kHz crystal in the backup domain
    lsi = 2,           ///< the internal RC, whose rate is nominal (device::lsi_*)
    hse_divided = 3,   ///< the crystal over device::rtc_hse_div
};

/// Whether this part's HSE-to-RTC division is a SINGLE number. It is
/// not on the CH32V20x_D6, where 3.4.9 gives 512 or 128 by lot number,
/// so a program that needs the rate on such a die measures it.
inline constexpr bool rtc_hse_divider_known = device::rtc_hse_div[0] == device::rtc_hse_div[1];

/// RTCCLK with RTCSEL on the HSE, for a crystal of `hse_hz` and the
/// divider this die turns out to have (index 0 or 1 of
/// device::rtc_hse_div - the same number twice where the manual states
/// one).
constexpr uint32_t rtc_hse_clock_hz(uint32_t hse_hz, uint8_t which = 0) {
    const uint32_t div = device::rtc_hse_div[which > 1u ? 1u : which];
    return div == 0u ? 0u : hse_hz / div;
}

/**
 * The backup domain: the write enable, the low-speed crystal, the
 * clock select and the domain's own reset.
 *
 *   brio::RtcDomain::unlock(true);
 *   brio::RtcDomain::lse_enable(true);
 *   if (!brio::RtcDomain::lse_wait_ready()) { ... no crystal on this board ... }
 *   brio::RtcDomain::open(brio::RtcClockSource::lse);
 *
 * RTCSEL IS ONE-WAY. 3.4.9: "once the RTC clock source has been
 * selected, it cannot be changed until the next backup domain is
 * reset". `select()` therefore answers false when a DIFFERENT source
 * already stands, and `reset()` is the only way out - at the cost of
 * the counter, the prescaler, the alarm and the backup data registers.
 */
struct RtcDomain {
    RtcDomain() = delete;

    /// A bound for the waits that are the silicon's own handful of
    /// cycles. Long enough for anything but an oscillator's start-up,
    /// short enough that a dead block is reported instead of hanging
    /// the program.
    static constexpr uint32_t ready_spins = 2'000'000UL;

    /// The bound for the LSE CRYSTAL, which is a different order of
    /// magnitude: a 32.768 kHz oscillator's start-up is specified in
    /// hundreds of milliseconds to seconds, not in cycles. Sized at
    /// twenty times `ready_spins`; `lse_wait_ready()` takes the bound
    /// as an argument for a board that wants to wait longer or give up
    /// sooner.
    static constexpr uint32_t lse_ready_spins = 40'000'000UL;

    // ---- the gates outside RCC_BDCTLR ---------------------------------------

    /// RCC_PB1PCENR's PWREN, clear at reset: with the bus clock shut,
    /// PWR_CTLR does not answer a write. Every verb that touches DBP
    /// opens it first.
    static void pwr_bus_clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb1, rcc_pb1_pwr);
        } else {
            Rcc::disable(Bus::pb1, rcc_pb1_pwr);
        }
    }
    static bool pwr_bus_clock() { return Rcc::enabled(Bus::pb1, rcc_pb1_pwr); }

    /**
     * PWR_CTLR.DBP (2.4.1): the door to the RTC's registers, the BKP
     * block's and the four backup-domain bits of RCC_BDCTLR. Returns
     * what the bit READS BACK as, so a caller can insist rather than
     * assume - with the bus clock shut the write goes nowhere and this
     * answers false.
     */
    static bool unlock(bool on) {
        pwr_bus_clock(true);
        pwr_ctlr_store(on ? (pwr()->CTLR | pwr_dbp) : (pwr()->CTLR & ~pwr_dbp));
        return unlocked() == on;
    }
    static bool unlocked() { return (pwr()->CTLR & pwr_dbp) != 0u; }

    // ---- RCC_BDCTLR ---------------------------------------------------------

    static uint32_t bdctlr() { return rcc()->BDCTLR; }

    /**
     * BDRST: reset the whole backup domain - the LSE's three bits,
     * RTCSEL, RTCEN, the RTC's prescaler, counter and alarm, and the
     * backup DATA registers (3.2.3, 4.2.4). The way back from every
     * one-way bit in this register, and the only one.
     *
     * The pulse is written, read back so that it is long enough, then
     * released and read back again. Needs DBP.
     */
    static void reset() {
        rcc()->BDCTLR = rcc_bdrst;
        (void)rcc()->BDCTLR;
        rcc()->BDCTLR = 0u;
        (void)rcc()->BDCTLR;
    }

    // ---- the low-speed crystal (3.3.3) --------------------------------------

    static void lse_enable(bool on) {
        rcc()->BDCTLR = on ? (rcc()->BDCTLR | rcc_lseon) : (rcc()->BDCTLR & ~rcc_lseon);
    }
    static bool lse_enabled() { return (rcc()->BDCTLR & rcc_lseon) != 0u; }
    /// LSERDY, set by hardware. 3.4.9 adds that after LSEON is cleared
    /// it takes six LSE cycles for this to fall.
    static bool lse_ready() { return (rcc()->BDCTLR & rcc_lserdy) != 0u; }

    /// Bounded wait for LSERDY. False = the crystal did not start in
    /// `spins` turns, which is a FACT ABOUT THE BOARD (no crystal
    /// fitted, or a bad one) and not an error to swallow.
    static bool lse_wait_ready(uint32_t spins = lse_ready_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (lse_ready()) {
                return true;
            }
        }
        return false;
    }

    /// LSEBYP: an external square wave into OSC32_IN instead of a
    /// crystal. "This bit can be written only when LSEON is 0" (3.4.9),
    /// which is the refusal.
    static bool lse_bypass(bool on) {
        if (lse_enabled() || lse_ready()) {
            return false;
        }
        rcc()->BDCTLR = on ? (rcc()->BDCTLR | rcc_lsebyp) : (rcc()->BDCTLR & ~rcc_lsebyp);
        return true;
    }
    static bool lse_bypass() { return (rcc()->BDCTLR & rcc_lsebyp) != 0u; }

    /// Whether this part's package brings out the 32 kHz oscillator
    /// pads at all: OSC32_IN and OSC32_OUT are PC14 and PC15, so a part
    /// with no port C has no crystal to start - and only the bypass
    /// form, which needs one pad, would be reachable if that pad
    /// existed.
    static constexpr bool has_lse_pins = (device::port_pins('C') & (3u << 14)) == (3u << 14);

    // ---- the select and the enable ------------------------------------------

    static RtcClockSource selected() {
        return static_cast<RtcClockSource>((rcc()->BDCTLR & rcc_rtcsel_mask) >> rcc_rtcsel_shift);
    }

    /**
     * Choose RTCCLK. ONE-WAY (3.4.9): false when a DIFFERENT source
     * already stands - the caller's way out is `reset()`, and saying so
     * is the whole reason this answers a bool. Re-selecting what is
     * already in force costs nothing and is true.
     */
    static bool select(RtcClockSource s) {
        const RtcClockSource now = selected();
        if (now == s) {
            return true;
        }
        if (now != RtcClockSource::none) {
            return false;
        }
        rcc()->BDCTLR = (rcc()->BDCTLR & ~rcc_rtcsel_mask) |
                        ((static_cast<uint32_t>(s) << rcc_rtcsel_shift) & rcc_rtcsel_mask);
        return selected() == s;
    }

    /// RTCEN. Hardware forces it to zero while RTCSEL is `none`
    /// (3.4.9), so the reader is what says whether it took.
    static void enable(bool on) {
        rcc()->BDCTLR = on ? (rcc()->BDCTLR | rcc_rtcen) : (rcc()->BDCTLR & ~rcc_rtcen);
    }
    static bool enabled() { return (rcc()->BDCTLR & rcc_rtcen) != 0u; }

    /**
     * The whole of 4.2's sequence in one verb: the PWR bus clock, DBP,
     * RTCSEL, RTCEN. False when the domain could not be unlocked, or
     * when the source could not be chosen because a different one
     * already stands - `wipe` is how a caller says "and reset the
     * domain first if you must", which is what a program wanting the
     * crystal has to do on a board whose domain came up on something
     * else.
     *
     * Does NOT start the LSE or the LSI: which oscillator to run and
     * how long to wait for it is the application's, and the
     * enable/wait pairs are right here and in clock.hpp.
     */
    static bool open(RtcClockSource s, bool wipe = false) {
        if (!unlock(true)) {
            return false;
        }
        if (wipe && selected() != s) {
            reset();
        }
        if (!select(s)) {
            return false;
        }
        enable(true);
        return enabled();
    }
};

// =============================================================================
// The counter (RM ch. 6)
// =============================================================================

/// Table 6-1: ten sixteen-bit registers on a four-byte stride, at
/// 0x4000 2800.
struct RtcRegs {
    volatile uint16_t CTLRH;  uint16_t RESERVED0;   ///< 0x00 the three interrupt enables
    volatile uint16_t CTLRL;  uint16_t RESERVED1;   ///< 0x04 RTOFF, CNF, RSF and the three flags
    volatile uint16_t PSCRH;  uint16_t RESERVED2;   ///< 0x08 PRL[19:16], write-only
    volatile uint16_t PSCRL;  uint16_t RESERVED3;   ///< 0x0c PRL[15:0], write-only
    volatile uint16_t DIVH;   uint16_t RESERVED4;   ///< 0x10 DIV[19:16], read-only
    volatile uint16_t DIVL;   uint16_t RESERVED5;   ///< 0x14 DIV[15:0], read-only
    volatile uint16_t CNTH;   uint16_t RESERVED6;   ///< 0x18 CNT[31:16]
    volatile uint16_t CNTL;   uint16_t RESERVED7;   ///< 0x1c CNT[15:0]
    volatile uint16_t ALRMH;  uint16_t RESERVED8;   ///< 0x20 ALR[31:16], write-only
    volatile uint16_t ALRML;  uint16_t RESERVED9;   ///< 0x24 ALR[15:0], write-only
};

inline RtcRegs* rtc_regs() { return reinterpret_cast<RtcRegs*>(pb1_base + 0x2800); }

/// RTC_CTLRH (6.3.1): the three interrupt enables.
inline constexpr uint16_t rtc_secie = 1U << 0;
inline constexpr uint16_t rtc_alrie = 1U << 1;
inline constexpr uint16_t rtc_owie  = 1U << 2;
inline constexpr uint16_t rtc_ctlrh_enables = rtc_secie | rtc_alrie | rtc_owie;

/// RTC_CTLRL (6.3.2). The three event flags and RSF are rw0 - a WRITTEN
/// ZERO clears one and a written one leaves it - so every store here is
/// a plain one with ones in the bits to keep.
inline constexpr uint16_t rtc_secf  = 1U << 0;
inline constexpr uint16_t rtc_alrf  = 1U << 1;
inline constexpr uint16_t rtc_owf   = 1U << 2;
inline constexpr uint16_t rtc_rsf   = 1U << 3;
inline constexpr uint16_t rtc_cnf   = 1U << 4;
inline constexpr uint16_t rtc_rtoff = 1U << 5;   ///< read-only
/// The four rw0 bits, which is what a store must carry as ones to keep.
inline constexpr uint16_t rtc_ctlrl_flags = rtc_secf | rtc_alrf | rtc_owf | rtc_rsf;

/// The prescaler's field is twenty bits and the divider it makes is
/// that plus one (6.3.4), so the longest tick this block can make is
/// RTCCLK over 2^20.
inline constexpr uint32_t rtc_prescaler_max = 0x000FFFFFUL;

/// What `rtc_prescaler_for()` answers when no reload makes the tick
/// asked for: a value the field cannot hold, so it can never be
/// mistaken for one.
inline constexpr uint32_t rtc_prescaler_none = 0xFFFFFFFFUL;

constexpr bool rtc_prescaler_valid(uint32_t reload) { return reload <= rtc_prescaler_max; }

/**
 * The reload that makes TR_CLK exactly `tick_hz` out of `rtcclk_hz` -
 * one second by default, which is what 6.3.4's own example computes
 * (0x7FFF from 32768 Hz).
 *
 * EXACT OR NOTHING: a ratio that is not whole, or one the twenty-bit
 * field cannot hold, answers `rtc_prescaler_none` rather than a rounded
 * reload, because a clock that is nearly right is a clock that drifts.
 */
constexpr uint32_t rtc_prescaler_for(uint32_t rtcclk_hz, uint32_t tick_hz = 1) {
    if (rtcclk_hz == 0u || tick_hz == 0u || (rtcclk_hz % tick_hz) != 0u) {
        return rtc_prescaler_none;
    }
    const uint32_t div = rtcclk_hz / tick_hz;
    return (div == 0u || div - 1u > rtc_prescaler_max) ? rtc_prescaler_none : div - 1u;
}

/// What TR_CLK a reload makes out of a stated RTCCLK - the inverse, for
/// a program that wants to print what it actually got.
constexpr uint32_t rtc_tick_hz(uint32_t rtcclk_hz, uint32_t reload) {
    return reload > rtc_prescaler_max ? 0u : rtcclk_hz / (reload + 1u);
}

/// What ONE window may set: the prescaler always, the counter and the
/// alarm when the program says so. The two flags exist because zero is
/// a legal count and a legal alarm.
struct RtcConfig {
    uint32_t prescaler = 0x7FFF;   ///< PRL: a divider of this plus one
    uint32_t count = 0;            ///< CNT, written when `set_count`
    uint32_t alarm = 0;            ///< ALR, written when `set_alarm`
    bool set_count = false;
    bool set_alarm = false;
};

constexpr bool rtc_config_valid(const RtcConfig& c) { return rtc_prescaler_valid(c.prescaler); }

/**
 * The counter, monostate.
 *
 *   brio::RtcDomain::open(brio::RtcClockSource::lse);
 *   brio::Rtc::synchronize();
 *   (void)brio::Rtc::configure({.prescaler = brio::rtc_prescaler_for(32768)});
 *   const uint32_t now = brio::Rtc::count();
 *
 * EVERY WRITE VERB ANSWERS A BOOL, and a false means the hardware never
 * reported the previous write finished (RTOFF stayed low) or the value
 * does not fit its field - never that the write was silently dropped.
 */
struct Rtc {
    Rtc() = delete;

    static RtcRegs& regs() { return *rtc_regs(); }

    /// A bound for RTOFF and RSF. A write crosses into RTCCLK, so the
    /// wait is three of ITS cycles - a hundred microseconds or so on
    /// the crystal, and the bound is generous against it.
    static constexpr uint32_t sync_spins = 2'000'000UL;

    /// The EXTI line the ALARM raises, and the one a wake from a
    /// low-power mode is armed on (9.4.2's table; `Exti::line_rtc_alarm`).
    static constexpr uint8_t wake_line = Exti::line_rtc_alarm;

    // ---- the two disciplines (6.2.3) ----------------------------------------

    /// RTOFF: the last write has reached the RTC's own domain. Read-only.
    static bool write_finished() { return (regs().CTLRL & rtc_rtoff) != 0u; }

    /// Bounded wait for RTOFF. What every write verb does first.
    static bool wait_write_finished(uint32_t spins = sync_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (write_finished()) {
                return true;
            }
        }
        return false;
    }

    /// RSF: an RTCCLK edge has passed since the flag was cleared, so
    /// the registers the bus reads are the RTC's own.
    static bool synchronized() { return (regs().CTLRL & rtc_rsf) != 0u; }

    /**
     * Clear RSF and wait for the hardware to set it again - the read
     * discipline of 6.2.3, which a program runs ONCE after a reset or a
     * wake before it believes a count. It costs up to one RTCCLK
     * period, which on the crystal is thirty microseconds.
     */
    static bool synchronize(uint32_t spins = sync_spins) {
        regs().CTLRL = static_cast<uint16_t>(rtc_ctlrl_flags & ~rtc_rsf);
        for (uint32_t i = 0; i < spins; ++i) {
            if (synchronized()) {
                return true;
            }
        }
        return false;
    }

    /// Open the configuration window: wait RTOFF, then CNF. A caller
    /// that opens it writes the registers itself and closes it with
    /// `end_config()`; `configure()` below is the verb that does all
    /// three.
    static bool begin_config(uint32_t spins = sync_spins) {
        if (!wait_write_finished(spins)) {
            return false;
        }
        regs().CTLRL = static_cast<uint16_t>(rtc_ctlrl_flags | rtc_cnf);
        return true;
    }

    /// Close it and wait for the write to land.
    static bool end_config(uint32_t spins = sync_spins) {
        regs().CTLRL = rtc_ctlrl_flags;
        return wait_write_finished(spins);
    }

    static bool in_config() { return (regs().CTLRL & rtc_cnf) != 0u; }

    // ---- the four backup-domain registers -----------------------------------

    /**
     * One window, everything the caller asked for. This is the verb a
     * program uses at boot: the chapter's own "write operation to one
     * or more RTC registers" means the prescaler, the counter and the
     * alarm cost one crossing between them and not three.
     */
    static bool configure(const RtcConfig& c) {
        if (!rtc_config_valid(c)) {
            return false;
        }
        if (!begin_config()) {
            return false;
        }
        regs().PSCRH = static_cast<uint16_t>((c.prescaler >> 16) & 0x000Fu);
        regs().PSCRL = static_cast<uint16_t>(c.prescaler);
        if (c.set_count) {
            regs().CNTH = static_cast<uint16_t>(c.count >> 16);
            regs().CNTL = static_cast<uint16_t>(c.count);
        }
        if (c.set_alarm) {
            regs().ALRMH = static_cast<uint16_t>(c.alarm >> 16);
            regs().ALRML = static_cast<uint16_t>(c.alarm);
        }
        return end_config();
    }

    /// The same, with the configuration a CONSTANT: a prescaler the
    /// twenty-bit field cannot hold is a compile error here, where the
    /// run-time form can only answer false.
    template <RtcConfig cfg>
    static bool configure() {
        static_assert(rtc_prescaler_valid(cfg.prescaler),
                      "brio Rtc: RTC_PSCR is TWENTY bits and the divider it makes is the value "
                      "plus one (RM 6.3.4), so the longest tick this block can make is RTCCLK "
                      "over 2^20 - rtc_prescaler_for() answers rtc_prescaler_none for a tick "
                      "this prescaler cannot reach");
        return configure(cfg);
    }

    /// The prescaler reload alone. WRITE-ONLY on this silicon: what can
    /// be read back is `divider()`, the live down-counter, and never
    /// this.
    static bool prescaler(uint32_t reload) {
        if (!rtc_prescaler_valid(reload)) {
            return false;
        }
        if (!begin_config()) {
            return false;
        }
        regs().PSCRH = static_cast<uint16_t>((reload >> 16) & 0x000Fu);
        regs().PSCRL = static_cast<uint16_t>(reload);
        return end_config();
    }

    /// The counter. Both halves inside one window, so the count never
    /// exists half-written.
    static bool count(uint32_t v) {
        if (!begin_config()) {
            return false;
        }
        regs().CNTH = static_cast<uint16_t>(v >> 16);
        regs().CNTL = static_cast<uint16_t>(v);
        return end_config();
    }

    /// The alarm. WRITE-ONLY, like the prescaler reload: a program that
    /// wants to know what it armed keeps the number itself.
    static bool alarm(uint32_t v) {
        if (!begin_config()) {
            return false;
        }
        regs().ALRMH = static_cast<uint16_t>(v >> 16);
        regs().ALRML = static_cast<uint16_t>(v);
        return end_config();
    }

    // ---- reading ------------------------------------------------------------

    /**
     * The 32-bit count. The two halves are two registers and the low
     * one can roll between the reads, so the high half is read, then
     * the low, then the high again, and the whole thing is taken again
     * while the two high reads disagree. The chapter says nothing about
     * this; it is arithmetic, not silicon.
     */
    static uint32_t count() {
        for (;;) {
            const uint16_t high = regs().CNTH;
            const uint16_t low = regs().CNTL;
            if (regs().CNTH == high) {
                return (static_cast<uint32_t>(high) << 16) | low;
            }
        }
    }

    /**
     * RTC_DIV, the prescaler's live DOWN-counter (6.3.5, 6.3.6): how
     * many RTCCLK edges are left before the next tick. Read the same
     * way as the count and for the same reason - and it is the finest
     * time this block offers, the fraction of the current second.
     */
    static uint32_t divider() {
        for (;;) {
            const uint16_t high = regs().DIVH;
            const uint16_t low = regs().DIVL;
            if (regs().DIVH == high) {
                return ((static_cast<uint32_t>(high) & 0x000Fu) << 16) | low;
            }
        }
    }

    // ---- the events (6.3.1, 6.3.2) ------------------------------------------

    /// Arm exactly the interrupts in `mask` (rtc_secie / rtc_alrie /
    /// rtc_owie) and disarm the rest. RTC_CTLRH is not one of the four
    /// backup-domain registers (6.2.2), so it needs no configuration
    /// window - but a write is still a write, and this waits for the
    /// previous one to land first.
    static bool interrupts(uint16_t mask) {
        if (!wait_write_finished()) {
            return false;
        }
        regs().CTLRH = mask & rtc_ctlrh_enables;
        return wait_write_finished();
    }
    static uint16_t interrupts() { return regs().CTLRH & rtc_ctlrh_enables; }

    /// The three event flags as they stand.
    static uint16_t flags() { return regs().CTLRL & (rtc_secf | rtc_alrf | rtc_owf); }
    static bool second() { return (regs().CTLRL & rtc_secf) != 0u; }
    static bool alarmed() { return (regs().CTLRL & rtc_alrf) != 0u; }
    static bool overflowed() { return (regs().CTLRL & rtc_owf) != 0u; }

    /// Clear exactly the flags in `mask`. The register is rw0, so the
    /// store carries ONES in every flag to keep - never a
    /// read-modify-write, which would acknowledge whatever arrived
    /// between the read and the write.
    static void clear(uint16_t mask) {
        regs().CTLRL = static_cast<uint16_t>(rtc_ctlrl_flags & ~(mask & rtc_ctlrl_flags));
    }

    /**
     * The RTC vector's body (`Irq::rtc`): the flags that stood,
     * cleared, and handed back for the program to read. The second, the
     * alarm and the overflow all arrive here.
     */
    [[gnu::always_inline]] static uint16_t isr() {
        const uint16_t f = flags();
        clear(f);
        return f;
    }

    // ---- the alarm's second path (EXTI line 17) ------------------------------

    /// Point EXTI line 17 at the alarm and arm it, which is what makes
    /// the alarm a WAKE: the line is asynchronous and needs no clock,
    /// where the RTC's own vector does. The sense is the rising edge -
    /// the alarm event is a pulse.
    static bool arm_wake(bool on) {
        if (!Exti::sense(wake_line, on ? ExtiSense::rising : ExtiSense::none)) {
            return false;
        }
        return Exti::interrupt(wake_line, on);
    }

    /// The same as an EVENT rather than an interrupt: what ends a WFE
    /// with no handler at all.
    static bool arm_wake_event(bool on) {
        if (!Exti::sense(wake_line, on ? ExtiSense::rising : ExtiSense::none)) {
            return false;
        }
        return Exti::event(wake_line, on);
    }

    /**
     * The body of the handler on the ALARM's own vector
     * (`Irq::rtc_alarm`): the EXTI line's flag cleared first and the
     * RTC's ALRF after it, because either one left standing re-enters
     * the handler for ever. Answers whether the alarm really stood.
     */
    [[gnu::always_inline]] static bool alarm_isr() {
        const bool stood = alarmed();
        (void)Exti::clear(wake_line);
        clear(rtc_alrf);
        return stood;
    }
};

// =============================================================================
// The backup registers and the tamper input (RM ch. 4)
// =============================================================================

/// One backup data register: sixteen bits on a four-byte stride.
struct BkpDataReg {
    volatile uint16_t D;
    uint16_t RESERVED;
};

/// Table 4-1. The second block of data registers (BKP_DATAR11..42)
/// belongs to the CH32V20x_D8 and its relatives; on the D6 the window
/// is there and `device::bkp_data_registers` is what says how many of
/// the cells mean anything.
struct BkpRegs {
    uint32_t RESERVED0;            ///< 0x00
    BkpDataReg DATAR[10];          ///< 0x04..0x28  BKP_DATAR1..BKP_DATAR10
    volatile uint16_t OCTLR;  uint16_t RESERVED1;   ///< 0x2c calibration and the pulse output
    volatile uint16_t TPCTLR; uint16_t RESERVED2;   ///< 0x30 the tamper enable and its level
    volatile uint16_t TPCSR;  uint16_t RESERVED3;   ///< 0x34 the tamper flags
    uint32_t RESERVED4[2];         ///< 0x38, 0x3c
    BkpDataReg DATAR_HIGH[32];     ///< 0x40..0xbc  BKP_DATAR11..BKP_DATAR42
};

inline BkpRegs* bkp_regs() { return reinterpret_cast<BkpRegs*>(pb1_base + 0x6C00); }

/// BKP_OCTLR (4.3.2)
inline constexpr uint16_t bkp_cal_mask = 0x007FU;   ///< CAL[6:0]
inline constexpr uint16_t bkp_cco      = 1U << 7;   ///< RTCCLK/64 on the TAMPER pad
inline constexpr uint16_t bkp_asoe     = 1U << 8;   ///< a pulse on the TAMPER pad
inline constexpr uint16_t bkp_asos     = 1U << 9;   ///< 1: the second pulse, 0: the alarm's

/// BKP_TPCTLR (4.3.3)
inline constexpr uint16_t bkp_tpe  = 1U << 0;   ///< the pad is the tamper input, not an I/O
inline constexpr uint16_t bkp_tpal = 1U << 1;   ///< 1: a LOW level is the tamper, 0: a HIGH one

/// BKP_TPCSR (4.3.4). TIF and TEF are read-only; CTI and CTE are
/// write-only ones that clear them.
inline constexpr uint16_t bkp_cte  = 1U << 0;
inline constexpr uint16_t bkp_cti  = 1U << 1;
inline constexpr uint16_t bkp_tpie = 1U << 2;
inline constexpr uint16_t bkp_tef  = 1U << 8;
inline constexpr uint16_t bkp_tif  = 1U << 9;

/// What the TAMPER pad puts out when it is not a tamper input (4.2.3).
enum class BkpPulse : uint8_t {
    alarm = 0,    ///< a pulse on every alarm event
    second = 1,   ///< a pulse on every second event
};

/**
 * The backup data registers and the block around them.
 *
 *   brio::Bkp::open();
 *   brio::Bkp::data(1, 0xBEEF);
 *   const auto kept = brio::Bkp::data(1);     // std::optional<uint16_t>
 *
 * WHAT THEY SURVIVE, AND WHAT THEY DO NOT. A system reset, a Standby
 * wake and - with a battery on VBAT - the loss of VDD (4.0). What wipes
 * them is a BACKUP DOMAIN reset (`RtcDomain::reset()`) or a TAMPER
 * event, and 4.2.4 is explicit that the block's own RCC reset line
 * (BKPRST) does NOT: the peripheral's reset and the domain's are two
 * different things here.
 *
 * HOW MANY THERE ARE IS THE DEVICE CLASS'S. Ten on the CH32V20x_D6,
 * forty-two on the D8 (`device::bkp_data_registers`), and a register
 * beyond the count is refused - by static_assert where the index is a
 * constant, by an empty optional where it is not.
 *
 * THE TAMPER PAD IS PC13, WHICH TWO-THIRDS OF THIS SERIES DO NOT BOND.
 * The block's registers answer on every part and every verb here
 * writes them; what a package decides is whether the pad comes out.
 * `has_tamper_pad` is that fact, and the pad itself is the PROGRAM's to
 * configure - nothing here claims a pin. AND THE BLOCK TAKES THE PAD
 * WHOLE when TPE is set: measured, the pad then reads zero and neither
 * its own output stage nor its pull reaches the detector, so a tamper
 * on this input can only come from OUTSIDE the chip.
 *
 * THE HARDWARE REMEMBERS AN EDGE IT WAS NOT WATCHING FOR (4.2.2): even
 * with TPE clear the level is sampled, so setting TPE over a pad
 * already at the active level raises a tamper event at once and wipes
 * the registers. `clear_event()` is what 4.2.2 says to call FIRST, and
 * `tamper()` calls it before it enables.
 */
struct Bkp {
    Bkp() = delete;

    static BkpRegs& regs() { return *bkp_regs(); }

    /// How many data registers this part's device class carries.
    static constexpr uint8_t count = device::bkp_data_registers;

    /// The TAMPER/RTC output pad, and whether this package brings it
    /// out. The pad is an ordinary GPIO until TPE, CCO or ASOE takes
    /// it.
    static constexpr char tamper_pad_port = 'C';
    static constexpr uint8_t tamper_pad_pin = 13;
    static constexpr bool has_tamper_pad =
        (device::port_pins(tamper_pad_port) & (1u << tamper_pad_pin)) != 0u;

    /// RCC_PB1PCENR's BKPEN, clear at reset (4.2's step 1).
    static void clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb1, rcc_pb1_bkp);
        } else {
            Rcc::disable(Bus::pb1, rcc_pb1_bkp);
        }
    }
    static bool clock() { return Rcc::enabled(Bus::pb1, rcc_pb1_bkp); }

    /// The block's own RCC reset line, which 4.2.4 says does NOT clear
    /// the data registers - and which, MEASURED, moves nothing in this
    /// register file at all: the calibration and the tamper pair
    /// survive it too. BKPRST resets the backup INTERFACE; what lies
    /// behind it belongs to the domain, and `RtcDomain::reset()` is the
    /// only thing that clears any of it.
    static void reset_block() { Rcc::reset(Bus::pb1, rcc_pb1_bkp); }

    /// 4.2's whole sequence: both bus clocks and DBP. False when the
    /// domain would not unlock.
    static bool open() {
        clock(true);
        return RtcDomain::unlock(true);
    }

    // ---- the data registers (4.3.1) -----------------------------------------

    /// Register `n`, numbered as the chapter numbers them: 1 to
    /// `count`. Nothing outside that range, and `std::optional` is how
    /// a run-time index says so.
    static std::optional<uint16_t> data(uint8_t n) {
        if (n < 1u || n > count) {
            return std::nullopt;
        }
        return cell(n).D;
    }

    /// False when the index is outside the range, and nothing written.
    /// A write is also dropped by the silicon while a tamper event
    /// stands (4.3.4's note on TEF), which `tamper_event()` is what
    /// asks.
    static bool data(uint8_t n, uint16_t v) {
        if (n < 1u || n > count) {
            return false;
        }
        cell(n).D = v;
        return true;
    }

    /// The same pair with the index a CONSTANT: a register this device
    /// class has not got is a compile error here.
    template <uint8_t N>
    static uint16_t data() {
        static_assert(N >= 1u && N <= count,
                      "brio Bkp: the backup data registers are numbered from one and how many "
                      "there are is the DEVICE CLASS's - ten on the CH32V20x_D6, forty-two on "
                      "the D8 (device::bkp_data_registers)");
        return cell(N).D;
    }

    template <uint8_t N>
    static void data(uint16_t v) {
        static_assert(N >= 1u && N <= count,
                      "brio Bkp: the backup data registers are numbered from one and how many "
                      "there are is the DEVICE CLASS's - ten on the CH32V20x_D6, forty-two on "
                      "the D8 (device::bkp_data_registers)");
        cell(N).D = v;
    }

    // ---- the calibration and the pad's outputs (4.3.2) ----------------------

    /// CAL[6:0]: how many RTCCLK pulses are SKIPPED every 2^20, which
    /// slows the clock by up to 121 ppm and can never speed it up
    /// (4.3.2). False for a value past the seven-bit field.
    static bool calibration(uint8_t cal) {
        if (cal > bkp_cal_mask) {
            return false;
        }
        regs().OCTLR = static_cast<uint16_t>((regs().OCTLR & ~bkp_cal_mask) | cal);
        return true;
    }
    static uint8_t calibration() { return static_cast<uint8_t>(regs().OCTLR & bkp_cal_mask); }

    /**
     * CCO: RTCCLK divided by 64 on the TAMPER pad, which is what a
     * program measures the oscillator against before it trims CAL.
     * 4.3.2's note 1 makes it exclusive with the tamper input, so this
     * REFUSES while TPE stands; note 2 adds that the bit dies with VDD
     * where the rest of the block does not.
     */
    static bool clock_output(bool on) {
        if (on && tamper_enabled()) {
            return false;
        }
        regs().OCTLR = on ? static_cast<uint16_t>(regs().OCTLR | bkp_cco)
                          : static_cast<uint16_t>(regs().OCTLR & ~bkp_cco);
        return true;
    }
    static bool clock_output() { return (regs().OCTLR & bkp_cco) != 0u; }

    /**
     * ASOE and ASOS: a pulse on the TAMPER pad at every alarm event or
     * at every second event (4.2.3). The same pad, so the same
     * exclusion with the tamper input.
     */
    static bool pulse_output(bool on, BkpPulse which = BkpPulse::second) {
        if (on && tamper_enabled()) {
            return false;
        }
        uint16_t v = static_cast<uint16_t>(regs().OCTLR & ~(bkp_asoe | bkp_asos));
        if (on) {
            v = static_cast<uint16_t>(v | bkp_asoe);
            if (which == BkpPulse::second) {
                v = static_cast<uint16_t>(v | bkp_asos);
            }
        }
        regs().OCTLR = v;
        return true;
    }
    static bool pulse_output() { return (regs().OCTLR & bkp_asoe) != 0u; }
    static BkpPulse pulse_source() {
        return (regs().OCTLR & bkp_asos) != 0u ? BkpPulse::second : BkpPulse::alarm;
    }

    // ---- the tamper input (4.2.2, 4.3.3, 4.3.4) -----------------------------

    static bool tamper_enabled() { return (regs().TPCTLR & bkp_tpe) != 0u; }
    /// TPAL as it stands: true when a LOW level is what counts as a
    /// tamper.
    static bool tamper_active_low() { return (regs().TPCTLR & bkp_tpal) != 0u; }

    /**
     * Enable or disable the tamper input, with the level that counts.
     *
     * THE ORDER IS 4.2.2's AND NOT A HABIT. The silicon samples the pad
     * whether or not TPE is set and REMEMBERS an edge that matches
     * TPAL, so enabling over a pad already at the active level fires at
     * once and wipes the data registers. So the level is written first
     * (legal only with TPE clear, 4.3.3's note), then the remembered
     * event is cleared, and only then is the input enabled.
     */
    static bool tamper(bool on, bool active_low = false) {
        regs().TPCTLR = static_cast<uint16_t>(regs().TPCTLR & ~bkp_tpe);
        uint16_t ctl = static_cast<uint16_t>(regs().TPCTLR & ~bkp_tpal);
        if (active_low) {
            ctl = static_cast<uint16_t>(ctl | bkp_tpal);
        }
        regs().TPCTLR = ctl;
        clear_event();
        if (on) {
            regs().TPCTLR = static_cast<uint16_t>(regs().TPCTLR | bkp_tpe);
        }
        return tamper_enabled() == on;
    }

    /**
     * TPIE. 4.3.4 notes two things about it: the interrupt cannot wake
     * the core from a low-power mode, and clearing the enable clears
     * TIF with it.
     *
     * A PLAIN STORE, NEVER A READ-MODIFY-WRITE, and that is the one
     * discipline this register demands: CTI and CTE are write-one bits
     * whose "value read out is invalid" (4.3.4), so carrying a read of
     * them back into a store would acknowledge a tamper nobody has
     * looked at. TPIE is the only bit here worth preserving, and
     * `tpcsr_keep()` is how the clearing verbs preserve it.
     */
    static void tamper_interrupt(bool on) { regs().TPCSR = on ? bkp_tpie : 0u; }
    static bool tamper_interrupt() { return (regs().TPCSR & bkp_tpie) != 0u; }

    /// TEF: a tamper event has happened. While it stands EVERY WRITE to
    /// a data register is dropped (4.3.4), which is the one thing that
    /// makes this flag worth reading outside a handler.
    static bool tamper_event() { return (regs().TPCSR & bkp_tef) != 0u; }
    /// TIF: the same event with the interrupt armed.
    static bool tamper_interrupt_flag() { return (regs().TPCSR & bkp_tif) != 0u; }

    /// CTE, write-only: clear TEF - and with it whatever edge the
    /// hardware remembered before TPE was set.
    static void clear_event() { regs().TPCSR = static_cast<uint16_t>(tpcsr_keep() | bkp_cte); }
    /// CTI, write-only: clear TIF.
    static void clear_interrupt() { regs().TPCSR = static_cast<uint16_t>(tpcsr_keep() | bkp_cti); }

    /**
     * The tamper vector's body (`Irq::tamper`): whether the event
     * stood, both flags cleared. The data registers are already gone by
     * the time this runs - that is what the block is for.
     */
    [[gnu::always_inline]] static bool isr() {
        const bool stood = tamper_interrupt_flag() || tamper_event();
        regs().TPCSR = static_cast<uint16_t>(tpcsr_keep() | bkp_cti | bkp_cte);
        return stood;
    }

private:
    static BkpDataReg& cell(uint8_t n) {
        return n <= 10u ? regs().DATAR[n - 1u] : regs().DATAR_HIGH[n - 11u];
    }

    /// The only bit of TPCSR a store must carry forward. TEF and TIF are
    /// read-only, and CTI and CTE are write-one bits whose read value
    /// 4.3.4 calls invalid - so a store that carried a READ of this
    /// register back into it could acknowledge a tamper nobody has
    /// looked at.
    static uint16_t tpcsr_keep() { return static_cast<uint16_t>(regs().TPCSR & bkp_tpie); }
};

} // namespace brio
