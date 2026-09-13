/*
 * rtc.hpp
 *
 * The real-time clock and the backup domain it lives in (RM0383 ch. 17,
 * RM0090 ch. 26, RM0390 ch. 22) - a BCD calendar, two alarms, a periodic
 * wake-up timer, two calibrators, a timestamp, one or two tamper inputs
 * and twenty words of memory that outlive every reset but the domain's
 * own.
 *
 *  RtcDomain  the gate and the clock: PWR_CR.DBP, the whole of RCC_BDCR
 *             (LSE with its bypass and drive mode, RTCSEL, RTCEN, BDRST),
 *             RCC_CSR's LSI and RCC_CFGR's RTCPRE - the HSE divider that
 *             only this chapter has a use for.
 *  Rtc        the peripheral: the write-protection keys, initialization
 *             mode, the two prescalers, the calendar, both alarms, the
 *             wake-up timer, smooth and coarse calibration, the shift,
 *             the timestamp, the tamper inputs, the RTC_OUT pad, the
 *             twenty backup registers, the flags and three ISR bodies.
 *
 * EIGHT FACTS OF THIS SILICON shape everything below.
 *
 * 1. THE DOMAIN IS LOCKED AT EVERY RESET, AND THERE ARE TWO LOCKS, not
 *    one. PWR_CR.DBP (RM0383 5.4.1) gates RCC_BDCR, every RTC register
 *    and every backup register, and is clear after a system reset; on top
 *    of it most RTC registers carry a SECOND lock that only the key
 *    sequence 0xCA then 0x53 into RTC_WPR opens (17.3.5), and "writing a
 *    wrong key reactivates the write protection". The two are
 *    independent: DBP is the domain's and is cleared by a system reset,
 *    the WPR lock is the peripheral's and "is not affected by system
 *    reset". THREE THINGS SIT OUTSIDE THE KEY and inside DBP alone -
 *    RTC_ISR[13:8] (the six event flags), RTC_TAFCR and RTC_BKPxR - which
 *    is why clear_flags(), the tamper verbs and backup() do not bracket
 *    themselves with the keys and every other configuring verb does.
 *    Writing DBP needs a READ BACK before its effect can be relied on
 *    (5.4.1's own note, and ES0206 2.2.9: the delay grows with the APB1
 *    prescaler); unlock() does that read.
 *
 * 2. RTCSEL IS ONE-WAY, AND THE WAY BACK IS A DOMAIN RESET. 6.2.8: "Once
 *    the RTCCLK clock source has been selected, the only possible way of
 *    modifying the selection is to reset the power domain." So select()
 *    returns false rather than pretending, and reset() - BDRST - is a
 *    deliberate, separately spelled verb: it stops the RTC and wipes the
 *    calendar, the prescalers, the alarms, the wake-up timer, both
 *    calibrations, the timestamp, TAFCR, THE TWENTY BACKUP REGISTERS and
 *    the clock choice.
 *
 * 3. THE RTC KEEPS RUNNING UNDER SYSTEM RESET, and that is the point of
 *    it. 17.3.7: on LSE or LSI the counter is not reset by anything but a
 *    domain reset; what a system reset clears is the three SHADOW
 *    registers and three bits of RTC_ISR (INIT, INITF, RSF). Hence INITS,
 *    the flag that says whether the calendar was ever set, and hence the
 *    backup registers as a legitimate place to leave a note.
 *
 * 4. THE CALENDAR IS READ THROUGH SHADOWS, OR NOT AT ALL - and on this
 *    silicon the shadow's locking is ERRATA-BOUND. With BYPSHAD = 0 the
 *    three readable registers are copies refreshed every two RTCCLK
 *    cycles, RSF says a copy has landed, and reading SSR or TR is
 *    supposed to LOCK the higher-order ones until DR is read (17.3.6).
 *    ES0287 2.8.2 (and its ES0206 2.9.2 / ES0298 2.9.2 twins) says the
 *    lock can be missed when the read is initiated one APB cycle before a
 *    shadow update, so the three may disagree; the workaround is either
 *    BYPSHAD = 1 or "read SSR again after reading SSR/TR/DR to confirm
 *    that SSR is still the same". read() does exactly that, per mode -
 *    the errata's re-read in shadow mode, the chapter's own read-twice in
 *    bypass mode - and is the only calendar reader this file offers.
 *    AND THE SHADOW IS NOT UPDATED IN STOP OR STANDBY: after such a sleep
 *    RSF must be cleared and awaited again before a shadow read means
 *    anything (17.3.2, 17.3.6).
 *    ONE MORE PRECONDITION, and it is arithmetic: a shadow read is legal
 *    only where fPCLK1 is at least seven times fRTCCLK (17.3.6).
 *    rtc_shadow_read_allowed() prices it; at 32768 Hz every rate this
 *    stratum can boot clears it by three orders of magnitude.
 *
 * 5. INITIALIZATION MODE IS A STOPPED CALENDAR, AND ENTERING IT TWICE IN
 *    A ROW IS AN ERRATUM. ES0287 2.8.4 (ES0206 2.9.4, ES0298 2.9.4): INIT
 *    set between one and two RTCCLK cycles after being cleared sets INITF
 *    immediately instead of waiting for the synchronization, and "a write
 *    during this critical period might result in the corruption of one or
 *    more calendar registers". The workaround is the chapter's own
 *    reading discipline, and this file applies it UNCONDITIONALLY on the
 *    way OUT: exit_init() clears INIT, then clears BYPSHAD if it was set,
 *    waits for RSF to rise and restores BYPSHAD - so a second
 *    enter_init() is safe by construction and no caller has to remember.
 *    The cost is one RTCCLK period (about 30 us at 32768 Hz) per exit,
 *    paid once per configuration. exit_init_raw() is the bare sequence,
 *    so that the erratum can be STAGED at a bench.
 *
 * 6. EVERY ARMED THING HAS A WRITE WINDOW, AND THE WINDOW IS A FLAG. The
 *    wake-up timer's reload and clock select may be written only with
 *    WUTE clear and WUTWF set; an alarm's registers only with its enable
 *    clear and its own write flag set, OR in initialization mode; and
 *    both flags take "1 to 2 RTCCLK clock cycles" to appear (17.3.5).
 *    Every configuring verb below therefore disables, waits for its own
 *    flag with a BOUND, writes and re-enables - and returns false if the
 *    flag never came, which on a stopped RTCCLK is what happens.
 *
 * 7. THE INTERRUPTS REACH THE NVIC THROUGH THE EXTI, AND THE LINES ARE
 *    CONFIGURABLE ONES. 17.5: alarm A and B share EXTI line 17 and the
 *    RTC_Alarm vector, the wake-up timer has line 22 and RTC_WKUP, and
 *    tamper and timestamp share line 21 and TAMP_STAMP. Unlike the
 *    STM32G0's direct lines these carry a trigger selection and a pending
 *    bit: the rising edge must be selected, the mask opened, and THE
 *    PENDING BIT CLEARED IN THE HANDLER or the vector re-enters for ever.
 *    The three EXTI operations this needs (IMR, RTSR, PR) are written
 *    here as private helpers with the EXTI's own register names; they are
 *    the pin-interrupt chapter's registers and will be reconciled with
 *    stm32f4/exti.hpp when that file exists.
 *    AND A SHARED LINE CAN SWALLOW AN EVENT: ES0287 2.8.3 (ES0206 2.9.3,
 *    ES0298 2.9.3) - one source's flag rising after the handler has
 *    checked it but before the EXTI's pending bit is really clear never
 *    raises the line again. The workaround is to check the sources again
 *    after clearing, which is why serve() below is a LOOP over the
 *    standing masked flags and not a single pass.
 *
 * 8. TWO CALIBRATORS, AND THEY ARE NOT TO BE USED TOGETHER. Smooth
 *    calibration (RTC_CALR, 17.3.11) masks or inserts individual RTCCLK
 *    pulses over a 32-second window, 0.954 ppm a step, -487.1 to
 *    +488.5 ppm, writable on the fly; COARSE calibration (RTC_CALIBR,
 *    17.3.10) adds or removes ck_apre cycles once a minute for the first
 *    2 x DC minutes of a 64-minute cycle, about +4 / -2 ppm a step, -63
 *    to +126 ppm, writable in initialization mode only and refused below
 *    PREDIV_A = 6. 17.3.10: "the application must select one of the two
 *    methods" - so each verb here refuses while the other calibrator is
 *    armed, rather than letting a program run two corrections at once.
 *
 * WHY RCC_BDCR, RCC_CFGR's RTCPRE AND RCC_CSR's LSI LIVE HERE AND NOT IN
 * clock.hpp. One register, one owner - and the owner is this chapter.
 * RCC_BDCR is unreachable without PWR_CR.DBP, so its access discipline is
 * the RTC DOMAIN's and not the clock tree's; RTCSEL is one-way, and a
 * clock verb whose consequence can only be undone by wiping a calendar
 * belongs beside the calendar; the register also carries RTCEN, LSE and
 * BDRST, which are this chapter's subject matter. RTCPRE divides the HSE
 * for one consumer only - RTCSEL's third choice - and "must be configured
 * before selecting the RTC clock source" (6.3.3), which is this file's
 * sequence. The LSI is NOT here: it is the RTC's and the watchdog's shared
 * root, an RC oscillator in the 1.2 V domain outside the backup domain,
 * and its verbs are Rcc's (clock.hpp) - RtcDomain selects it, Rcc runs it.
 * Nor are the EXTI lines' registers: exti.hpp owns the fabric, and the
 * wake_line_* verbs below are this chapter's SEQUENCE over its verbs.
 *
 * ERRATA, ES0287 Rev 6 on the bench part's revision (REV_ID 0x1000,
 * markings A/1/2 - every RTC item applies to all three), with the ES0206
 * and ES0298 twins named where a verb answers one:
 *  - 2.8.2 (the calendar shadow lock) is LIVE and is coded, see fact 4;
 *  - 2.8.3 (one RTC interrupt masked by another) is LIVE and is coded,
 *    see fact 7;
 *  - 2.8.4 (calendar initialization on consecutive INIT entries) is LIVE
 *    and is coded, see fact 5;
 *  - 2.8.1 (a spurious tamper detection when a falling-edge channel is
 *    disabled with its pin high) has NO workaround: tamper_disarm() says
 *    so and clears the flag it may have raised, which is all software
 *    can do;
 *  - 2.8.5 (an alarm flag repeatedly set while the core is halted in
 *    debug, when the sub-second field is compared) has no workaround
 *    either; serve()'s bounded loop keeps a handler from spinning for
 *    ever on it, and the fact is stated in docs/stm32f4/rtc.md;
 *  - 2.2.10 (PC13 transitions disturb LSE on the LQFP and UFQFPN
 *    packages) is LIVE and unfixable in software: PC13 is RTC_AF1, the
 *    pad every tamper, timestamp and RTC_OUT verb here reaches, so a
 *    program that runs LSE and toggles PC13 pays for it in the crystal's
 *    accuracy. Stated, measured (docs/stm32f4/rtc.md), not coded;
 *  - 2.2.12 (a missed backup-domain reset after a supply dip leaves the
 *    domain's registers unpredictable) is LIVE, and its workaround is
 *    this file's reset() called from a power-on reset flag - which is
 *    the reset chapter's flag to read.
 *
 * NOT BUILT: ONE FIELD OF THE CHAPTER, and it is RTC_CR's FMT. The
 * calendar is pinned to the 24-hour format by set_calendar(), and
 * RtcDateTime has no AM/PM - a binary hour and a PM bit would be two
 * truths about what "one o'clock" means, and the conversion belongs to
 * whoever displays it. Everything else in ch. 17's register description
 * has a verb here; docs/stm32f4/rtc.md carries what has not been
 * MEASURED, which is a different list.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/pwr.hpp"

namespace brio {

// =============================================================================
// The RTC domain: the gate, the clock and the one-way choices (RM0383 6.2.8)
// =============================================================================

/// RCC_BDCR.RTCSEL: what RTCCLK is. `none` is the reset value and means
/// the RTC has no clock at all - not an error, just a peripheral that
/// will never tick.
enum class RtcClockSource : uint8_t {
    none = 0,
    lse = 1,
    lsi = 2,
    hse_divided = 3,
};

/// RCC_BDCR.LSEMOD, where the part has it (rtc_has_lse_mode()): the
/// oscillator's drive. "Low power" is the reset value and what a watch
/// crystal on a clean board wants; "high drive" starts a reluctant one.
enum class LseMode : uint8_t { low_power = 0, high_drive = 1 };

/// The RTCPRE code that divides the HSE down to `want_hz` exactly, or 0
/// when no legal code does. The field is 5 bits and ITS VALUE IS THE
/// DIVIDER, with 0 and 1 both meaning "no clock" (6.3.3), so the search
/// is over 2..31. 6.2.8 wants 1 MHz at the RTC, which a 25 MHz crystal
/// reaches at /25 and an 8 MHz one at /8; an HSE above 31 MHz, or one
/// that is not a whole multiple of the wanted rate, reaches it at none.
constexpr uint8_t rtc_hse_divider_for(uint32_t hse_hz, uint32_t want_hz = 1'000'000u) {
    if (want_hz == 0u || hse_hz == 0u || hse_hz % want_hz != 0u) {
        return 0;
    }
    const uint32_t div = hse_hz / want_hz;
    return (div >= 2u && div <= 31u) ? static_cast<uint8_t>(div) : 0u;
}

/**
 * The RTC domain's gate and clock tree.
 *
 * ORDER, AND IT IS RM0383 5.1.2's: open the PWR bus clock, set DBP,
 * select RTCCLK, set RTCEN. `open()` is those four in one call; the
 * individual verbs exist because a boot that has to reset the domain
 * first does them in a different order, and because a suite has to be
 * able to prove each one.
 */
struct RtcDomain {
    RtcDomain() = delete;

    /// A bound for the ready and reset waits that are the silicon's own
    /// handful of cycles. At 100 MHz this is a few tens of milliseconds -
    /// long enough for anything but a crystal's start-up, and short
    /// enough that a dead oscillator is reported instead of hanging the
    /// program.
    static constexpr uint32_t ready_spins = 2'000'000UL;

    /// The bound for an LSE CRYSTAL, which is a different order of
    /// magnitude: the oscillator's start-up is specified in seconds, not
    /// cycles (DS10314 table 39 gives 2 s typical), and measured at
    /// 0.4 s on the bench board. Sized for four times that at 100 MHz;
    /// lse_wait_ready() takes the bound as an argument for a board that
    /// wants to wait longer or give up sooner.
    static constexpr uint32_t lse_ready_spins = 40'000'000UL;

    // ---- the gate outside BDCR ----------------------------------------------

    /// RCC_APB1ENR.PWREN - the PWR block's bus clock, clear at reset. A
    /// peripheral without its bus clock does not answer register reads,
    /// so this comes first. It is stm32f4/pwr.hpp's verb, called here
    /// rather than duplicated.
    static void pwr_bus_clock(bool on) { Pwr::bus_clock(on); }
    static bool pwr_bus_clock() { return Pwr::bus_clock(); }

    /**
     * PWR_CR.DBP - "disable backup domain write protection" (5.4.1).
     * Needs pwr_bus_clock() first.
     *
     * THE READ BACK IS THE CHAPTER'S OWN: "depending on the APB1
     * prescaler, there is a delay between writing to DBP and the
     * effective disabling/enabling of the backup domain protection.
     * Therefore, a dummy read operation to the PWR_CR register is
     * required just after writing to the DBP bit." ES0206 2.2.9 says the
     * same as an erratum. One read is not a proof that the protection has
     * moved - unlocked() is - but it is what the manual asks for, and the
     * verb answers the readback so a caller can insist.
     */
    static bool unlock(bool on) {
        Pwr::bus_clock(true);
        PWR->CR = on ? (PWR->CR | PWR_CR_DBP) : (PWR->CR & ~PWR_CR_DBP);
        return unlocked() == on;
    }
    static bool unlocked() { return (PWR->CR & PWR_CR_DBP) != 0u; }

    // ---- RCC_BDCR -----------------------------------------------------------

    static uint32_t bdcr() { return RCC->BDCR; }

    /**
     * Reset the whole backup domain - BDRST, the way back from every
     * one-way bit in this register.
     *
     * WHAT IT COSTS: the calendar, the prescalers, both alarms, the
     * wake-up timer, both calibrations, the timestamp, TAFCR, THE TWENTY
     * BACKUP REGISTERS and the clock choice (17.3.7). Needs DBP.
     *
     * It is also ES0287 2.2.12's workaround: a supply dip can leave the
     * domain without the power-on reset it was owed, and the way out is
     * this verb, called by a program that finds a power-on reset flag.
     *
     * The pulse is written, read back so it is long enough, then
     * released - ES0206 2.2.9's second measure ("wait for the end of the
     * operation ... via a polling loop on targeted registers") applied to
     * the register the pulse is in.
     */
    static void reset() {
        RCC->BDCR = RCC_BDCR_BDRST;
        (void)RCC->BDCR;
        RCC->BDCR = 0u;
        (void)RCC->BDCR;
    }

    // ---- LSE ----------------------------------------------------------------

    static void lse_enable(bool on) {
        RCC->BDCR = on ? (RCC->BDCR | RCC_BDCR_LSEON) : (RCC->BDCR & ~RCC_BDCR_LSEON);
    }
    static bool lse_enabled() { return (RCC->BDCR & RCC_BDCR_LSEON) != 0u; }
    static bool lse_ready() { return (RCC->BDCR & RCC_BDCR_LSERDY) != 0u; }

    /// Bounded wait for LSERDY. False = the crystal did not start in
    /// `spins` turns, which is a FACT about the board (no crystal fitted,
    /// or a bad one) and not an error to be swallowed.
    static bool lse_wait_ready(uint32_t spins = lse_ready_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (lse_ready()) {
                return true;
            }
        }
        return false;
    }

    /// LSEBYP - an external square wave on OSC32_IN instead of a crystal.
    /// "This bit can be written only when the LSE clock is disabled"
    /// (6.3.17), which is the refusal.
    static bool lse_bypass(bool on) {
        if (lse_enabled() || lse_ready()) {
            return false;
        }
        RCC->BDCR = on ? (RCC->BDCR | RCC_BDCR_LSEBYP) : (RCC->BDCR & ~RCC_BDCR_LSEBYP);
        return true;
    }
    static bool lse_bypass() { return (RCC->BDCR & RCC_BDCR_LSEBYP) != 0u; }

    /// Whether this part's RCC_BDCR carries LSEMOD at all.
    static constexpr bool has_lse_mode() { return rtc_has_lse_mode(); }

    /**
     * LSEMOD, the oscillator's drive (6.3.17). Written with the
     * oscillator STOPPED: the chapter attaches no rule of its own, but a
     * drive change under a running crystal is a change of the working
     * point of an oscillator whose start-up is already measured in
     * seconds, so this verb refuses it the way lse_bypass() refuses.
     * False on a part whose header has no LSEMOD, with nothing written.
     */
    static bool lse_mode(LseMode m) {
#if defined(RCC_BDCR_LSEMOD)
        if (lse_enabled() || lse_ready()) {
            return false;
        }
        RCC->BDCR = (m == LseMode::high_drive) ? (RCC->BDCR | RCC_BDCR_LSEMOD)
                                               : (RCC->BDCR & ~RCC_BDCR_LSEMOD);
        return true;
#else
        (void)m;
        return false;
#endif
    }
    static LseMode lse_mode() {
#if defined(RCC_BDCR_LSEMOD)
        return (RCC->BDCR & RCC_BDCR_LSEMOD) != 0u ? LseMode::high_drive : LseMode::low_power;
#else
        return LseMode::low_power;
#endif
    }

    // ---- the HSE divider (RCC_CFGR.RTCPRE) ----------------------------------

    /**
     * RTCPRE - the HSE division that feeds RTCSEL's third choice
     * (6.3.3). The value IS the divider: 2..31, with 0 and 1 meaning "no
     * clock". Refused outside that range, and refused once a source is
     * selected, because the chapter says these bits "must be configured
     * if needed before selecting the RTC clock source" and the selection
     * is one-way.
     */
    static bool hse_divider(uint8_t div) {
        if (div < 2u || div > 31u || selected() != RtcClockSource::none) {
            return false;
        }
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_RTCPRE_Msk) |
                    (static_cast<uint32_t>(div) << RCC_CFGR_RTCPRE_Pos);
        return true;
    }
    static uint8_t hse_divider() {
        return static_cast<uint8_t>((RCC->CFGR & RCC_CFGR_RTCPRE_Msk) >> RCC_CFGR_RTCPRE_Pos);
    }

    // ---- RTCSEL and RTCEN ---------------------------------------------------

    static RtcClockSource selected() {
        return static_cast<RtcClockSource>((RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >>
                                           RCC_BDCR_RTCSEL_Pos);
    }

    /**
     * Choose RTCCLK. ONE-WAY (6.2.8): false when a DIFFERENT source is
     * already selected - the caller's way out is reset(), and saying so
     * is the whole reason this returns a bool. Re-selecting what is
     * already in force is a no-op and true.
     */
    static bool select(RtcClockSource s) {
        const RtcClockSource now = selected();
        if (now == s) {
            return true;
        }
        if (now != RtcClockSource::none) {
            return false;
        }
        RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL_Msk) |
                    ((static_cast<uint32_t>(s) << RCC_BDCR_RTCSEL_Pos) & RCC_BDCR_RTCSEL_Msk);
        return selected() == s;
    }

    static void enable(bool on) {
        RCC->BDCR = on ? (RCC->BDCR | RCC_BDCR_RTCEN) : (RCC->BDCR & ~RCC_BDCR_RTCEN);
    }
    static bool enabled() { return (RCC->BDCR & RCC_BDCR_RTCEN) != 0u; }

    // ---- the composed boot verb ---------------------------------------------

    /**
     * The whole of 5.1.2's sequence: the PWR bus clock, DBP, RTCSEL,
     * RTCEN. False when the domain could not be unlocked, or when the
     * source could not be chosen because a different one already stands -
     * `wipe` is how a caller says "and reset the domain first if you
     * must", which is what a program wanting LSE has to do on a board
     * whose domain came up on something else (fact 2).
     *
     * Does NOT start LSE or LSI: which oscillator to run and how long to
     * wait for it is the application's, and the enable/wait pairs are
     * right there.
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
// The prescalers (RM0383 17.3.1, 17.6.5)
// =============================================================================

/// The two register FIELDS, not the divisors: PREDIV_A and PREDIV_S, each
/// one less than the ratio it makes. The defaults are the domain reset's,
/// which are the 32768 Hz pair.
struct RtcPrescalers {
    uint8_t async = 127;    ///< PREDIV_A, 7 bits: ck_apre = RTCCLK/(async+1)
    uint16_t sync = 255;    ///< PREDIV_S, 15 bits: ck_spre = ck_apre/(sync+1)
};

constexpr bool rtc_prescalers_valid(const RtcPrescalers& p) {
    return p.async <= 0x7Fu && p.sync <= 0x7FFFu;
}

/// ck_spre in hertz for a STATED RTCCLK - the calendar's own tick, and
/// also the wake-up timer's slowest clock. The rate is the caller's
/// argument and not a constant of this file: LSE is 32768 Hz by
/// construction, LSI is an uncalibrated RC and the HSE branch is whatever
/// RTCPRE makes of a board's crystal, and a driver that pretended to know
/// which one is running would be lying.
constexpr uint32_t rtc_ck_spre_hz(const RtcPrescalers& p, uint32_t rtcclk_hz) {
    return rtcclk_hz / ((static_cast<uint32_t>(p.async) + 1UL) *
                        (static_cast<uint32_t>(p.sync) + 1UL));
}

/// ck_apre - what the sub-second counter counts, i.e. the resolution of
/// every sub-second reading and sub-second alarm match.
constexpr uint32_t rtc_ck_apre_hz(const RtcPrescalers& p, uint32_t rtcclk_hz) {
    return rtcclk_hz / (static_cast<uint32_t>(p.async) + 1UL);
}

/// How many RTCCLK cycles one calendar second is: the product of the two
/// ratios, which is what a measurement of the second divides.
constexpr uint32_t rtc_cycles_per_second(const RtcPrescalers& p) {
    return (static_cast<uint32_t>(p.async) + 1UL) * (static_cast<uint32_t>(p.sync) + 1UL);
}

/// The EXACT pair that makes ck_spre 1 Hz from `rtcclk_hz`, with the
/// asynchronous factor as HIGH as possible (17.3.1's own recommendation:
/// "configure the asynchronous prescaler to a high value to minimize
/// consumption"). `async` comes back 0xFF - a value the 7-bit field
/// cannot hold - when no exact pair exists. 32768 gives 127/255, the
/// domain reset value and the chapter's example.
constexpr RtcPrescalers rtc_prescalers_for(uint32_t rtcclk_hz) {
    for (uint32_t a = 128; a >= 1; --a) {
        if (rtcclk_hz % a != 0u) {
            continue;
        }
        const uint32_t s = rtcclk_hz / a;
        if (s >= 1u && s <= 32768u) {
            return RtcPrescalers{static_cast<uint8_t>(a - 1u), static_cast<uint16_t>(s - 1u)};
        }
    }
    return RtcPrescalers{0xFFu, 0u};
}

/// The exact 1 Hz pair with the SMALLEST asynchronous factor - i.e. the
/// LARGEST PREDIV_S, which is what a caller who wants the sub-second
/// counter as a STOPWATCH asks for. It is the deliberate opposite of
/// rtc_prescalers_for()'s choice: 17.3.8 says "the resolution can be
/// improved by increasing the synchronous prescaler value" and prices it
/// in ck_apre current, and 30.52 us at 32768 Hz is what PREDIV_S = 0x7FFF
/// buys.
constexpr RtcPrescalers rtc_prescalers_for_resolution(uint32_t rtcclk_hz) {
    for (uint32_t a = 1; a <= 128u; ++a) {
        if (rtcclk_hz % a != 0u) {
            continue;
        }
        const uint32_t s = rtcclk_hz / a;
        if (s >= 1u && s <= 32768u) {
            return RtcPrescalers{static_cast<uint8_t>(a - 1u), static_cast<uint16_t>(s - 1u)};
        }
    }
    return RtcPrescalers{0xFFu, 0u};
}

/// The fraction of a second a sub-second reading means, in MILLISECONDS:
/// 17.6.11's own formula (PREDIV_S - SS) / (PREDIV_S + 1), scaled. SS can
/// exceed PREDIV_S after a shift operation, which the chapter says means
/// "one second less"; here that clamps to zero rather than wrapping.
constexpr uint16_t rtc_subsecond_ms(uint16_t ss, uint16_t prediv_s) {
    if (ss > prediv_s) {
        return 0;
    }
    return static_cast<uint16_t>((static_cast<uint32_t>(prediv_s - ss) * 1000UL) /
                                 (static_cast<uint32_t>(prediv_s) + 1UL));
}

/// 17.3.6's precondition on a SHADOW read: fPCLK1 must be at least seven
/// times fRTCCLK. Below that the calendar registers must be read with
/// BYPSHAD = 1 (17.6.3's note), which is what this predicate is for.
constexpr bool rtc_shadow_read_allowed(uint32_t pclk1_hz, uint32_t rtcclk_hz) {
    return rtcclk_hz != 0u && pclk1_hz >= 7UL * rtcclk_hz;
}

// =============================================================================
// The calendar (RM0383 17.3.2, 17.6.1, 17.6.2)
// =============================================================================

/// A whole calendar reading or setting, in ORDINARY NUMBERS. The
/// registers are BCD and this struct is not: the conversion is constexpr,
/// checked both ways by the family fixture, and is the only place a
/// nibble is shifted.
struct RtcDateTime {
    uint8_t hour = 0;      ///< 0..23 (24-hour format; FMT is pinned to it)
    uint8_t minute = 0;    ///< 0..59
    uint8_t second = 0;    ///< 0..59
    uint8_t day = 1;       ///< 1..31
    uint8_t month = 1;     ///< 1..12
    uint8_t year = 0;      ///< 0..99, the century being the caller's business
    uint8_t weekday = 1;   ///< 1 = Monday .. 7 = Sunday; 0 is "forbidden" (17.6.2)
};

constexpr uint8_t rtc_to_bcd(uint8_t v) {
    return static_cast<uint8_t>(((v / 10u) << 4) | (v % 10u));
}
constexpr uint8_t rtc_from_bcd(uint8_t v) {
    return static_cast<uint8_t>(((v >> 4) & 0x0Fu) * 10u + (v & 0x0Fu));
}

/// Days in a month for a two-digit year. THE LEAP RULE IS THE SILICON'S:
/// this calendar spans one century, so "divisible by four" is the whole
/// of it - there is no year 00 exception to apply, the field having no
/// century in it.
constexpr uint8_t rtc_days_in_month(uint8_t month, uint8_t year) {
    switch (month) {
        case 1: case 3: case 5: case 7: case 8: case 10: case 12: return 31;
        case 4: case 6: case 9: case 11: return 30;
        case 2: return (year % 4u == 0u) ? 29 : 28;
        default: return 0;
    }
}

constexpr bool rtc_datetime_valid(const RtcDateTime& d) {
    if (d.hour > 23u || d.minute > 59u || d.second > 59u) {
        return false;
    }
    if (d.month < 1u || d.month > 12u || d.year > 99u) {
        return false;
    }
    if (d.weekday < 1u || d.weekday > 7u) {
        return false;
    }
    return d.day >= 1u && d.day <= rtc_days_in_month(d.month, d.year);
}

/// RTC_TR for a setting (24-hour format, so PM is always 0).
constexpr uint32_t rtc_time_register(const RtcDateTime& d) {
    return (static_cast<uint32_t>(rtc_to_bcd(d.hour)) << RTC_TR_HU_Pos) |
           (static_cast<uint32_t>(rtc_to_bcd(d.minute)) << RTC_TR_MNU_Pos) |
           (static_cast<uint32_t>(rtc_to_bcd(d.second)) << RTC_TR_SU_Pos);
}

/// RTC_DR for a setting.
constexpr uint32_t rtc_date_register(const RtcDateTime& d) {
    return (static_cast<uint32_t>(rtc_to_bcd(d.year)) << RTC_DR_YU_Pos) |
           (static_cast<uint32_t>(d.weekday) << RTC_DR_WDU_Pos) |
           (static_cast<uint32_t>(rtc_to_bcd(d.month)) << RTC_DR_MU_Pos) |
           (static_cast<uint32_t>(rtc_to_bcd(d.day)) << RTC_DR_DU_Pos);
}

constexpr RtcDateTime rtc_decode(uint32_t tr, uint32_t dr) {
    RtcDateTime d{};
    d.hour = rtc_from_bcd(
        static_cast<uint8_t>((tr & (RTC_TR_HT_Msk | RTC_TR_HU_Msk)) >> RTC_TR_HU_Pos));
    d.minute = rtc_from_bcd(
        static_cast<uint8_t>((tr & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >> RTC_TR_MNU_Pos));
    d.second = rtc_from_bcd(
        static_cast<uint8_t>((tr & (RTC_TR_ST_Msk | RTC_TR_SU_Msk)) >> RTC_TR_SU_Pos));
    d.year = rtc_from_bcd(
        static_cast<uint8_t>((dr & (RTC_DR_YT_Msk | RTC_DR_YU_Msk)) >> RTC_DR_YU_Pos));
    d.weekday = static_cast<uint8_t>((dr & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos);
    d.month = rtc_from_bcd(
        static_cast<uint8_t>((dr & (RTC_DR_MT_Msk | RTC_DR_MU_Msk)) >> RTC_DR_MU_Pos));
    d.day = rtc_from_bcd(
        static_cast<uint8_t>((dr & (RTC_DR_DT_Msk | RTC_DR_DU_Msk)) >> RTC_DR_DU_Pos));
    return d;
}

/// One coherent look at the calendar: the three registers as one value.
struct RtcReading {
    RtcDateTime time{};
    uint16_t subsecond = 0;   ///< RTC_SSR, counting DOWN from PREDIV_S
};

// =============================================================================
// The alarms (RM0383 17.3.3, 17.6.8, 17.6.9, 17.6.18, 17.6.19)
// =============================================================================

enum class RtcAlarmId : uint8_t { a = 0, b = 1 };

/**
 * One alarm. Every field is compared unless its mask says otherwise, so
 * the DEFAULT here - everything masked, and no sub-second comparison - is
 * "every second", and a caller builds up from there.
 *
 * `subsecond_mask` is MASKSS as the chapter spells it (17.6.18): 0 means
 * no sub-second comparison at all (the alarm lands when the seconds unit
 * increments), 15 means all fifteen bits must match, and k in between
 * compares the low k bits.
 */
struct RtcAlarm {
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint8_t day = 1;                 ///< date 1..31, or weekday 1..7
    bool weekday_select = false;     ///< WDSEL: `day` is a weekday
    bool mask_date = true;           ///< MSK4
    bool mask_hours = true;          ///< MSK3
    bool mask_minutes = true;        ///< MSK2
    bool mask_seconds = true;        ///< MSK1
    uint8_t subsecond_mask = 0;      ///< MASKSS, 0..15
    uint16_t subsecond = 0;          ///< SS[14:0]
};

constexpr bool rtc_alarm_valid(const RtcAlarm& a) {
    if (a.subsecond_mask > 15u || a.subsecond > 0x7FFFu) {
        return false;
    }
    if (!a.mask_hours && a.hour > 23u) {
        return false;
    }
    if (!a.mask_minutes && a.minute > 59u) {
        return false;
    }
    if (!a.mask_seconds && a.second > 59u) {
        return false;
    }
    if (!a.mask_date) {
        if (a.weekday_select) {
            return a.day >= 1u && a.day <= 7u;
        }
        return a.day >= 1u && a.day <= 31u;
    }
    return true;
}

constexpr uint32_t rtc_alarm_register(const RtcAlarm& a) {
    uint32_t v = (static_cast<uint32_t>(rtc_to_bcd(a.hour)) << RTC_ALRMAR_HU_Pos) |
                 (static_cast<uint32_t>(rtc_to_bcd(a.minute)) << RTC_ALRMAR_MNU_Pos) |
                 (static_cast<uint32_t>(rtc_to_bcd(a.second)) << RTC_ALRMAR_SU_Pos);
    v |= a.weekday_select
             ? (RTC_ALRMAR_WDSEL | (static_cast<uint32_t>(a.day) << RTC_ALRMAR_DU_Pos))
             : (static_cast<uint32_t>(rtc_to_bcd(a.day)) << RTC_ALRMAR_DU_Pos);
    if (a.mask_date) {
        v |= RTC_ALRMAR_MSK4;
    }
    if (a.mask_hours) {
        v |= RTC_ALRMAR_MSK3;
    }
    if (a.mask_minutes) {
        v |= RTC_ALRMAR_MSK2;
    }
    if (a.mask_seconds) {
        v |= RTC_ALRMAR_MSK1;
    }
    return v;
}

constexpr uint32_t rtc_alarm_subsecond_register(const RtcAlarm& a) {
    return (static_cast<uint32_t>(a.subsecond_mask) << RTC_ALRMASSR_MASKSS_Pos) |
           (static_cast<uint32_t>(a.subsecond) & RTC_ALRMASSR_SS_Msk);
}

// =============================================================================
// The wake-up timer (RM0383 17.3.4, 17.6.6)
// =============================================================================

/// WUCKSEL. The four divided-RTCCLK codes give a short, fine timer (122 us
/// to 32 s at 32768 Hz); the two ck_spre codes give a long, one-second
/// one, the second of them adding 2^16 to the reload (17.6.6), which is
/// how the range reaches 36 hours.
enum class RtcWakeupClock : uint8_t {
    div16 = 0,
    div8 = 1,
    div4 = 2,
    div2 = 3,
    ck_spre = 4,
    ck_spre_high = 6,   ///< ck_spre with 2^16 added to the counter
};

/// The divider a code means, 0 for the two ck_spre codes (whose rate is
/// the prescalers' business and not this enum's).
constexpr uint8_t rtc_wakeup_divider(RtcWakeupClock c) {
    switch (c) {
        case RtcWakeupClock::div16: return 16;
        case RtcWakeupClock::div8: return 8;
        case RtcWakeupClock::div4: return 4;
        case RtcWakeupClock::div2: return 2;
        default: return 0;
    }
}

/// 17.6.6: WUTF is set every (WUT + 1) ck_wut cycles, and "setting
/// WUT[15:0] to 0x0000 with WUCKSEL[2:0] = 011 (RTCCLK/2) is forbidden".
/// The reload is a 16-bit field, so a caller asking for more has asked
/// for a different timer.
constexpr bool rtc_wakeup_valid(RtcWakeupClock c, uint32_t reload) {
    if (reload > 0xFFFFu) {
        return false;
    }
    if (c == RtcWakeupClock::div2 && reload == 0u) {
        return false;
    }
    switch (c) {
        case RtcWakeupClock::div16:
        case RtcWakeupClock::div8:
        case RtcWakeupClock::div4:
        case RtcWakeupClock::div2:
        case RtcWakeupClock::ck_spre:
        case RtcWakeupClock::ck_spre_high:
            return true;
    }
    return false;
}

/// ck_wut in hertz for a stated RTCCLK and prescaler pair - the ruler an
/// application converts a deadline with. Zero for a ck_spre code with a
/// prescaler pair that does not make a whole number.
constexpr uint32_t rtc_wakeup_clock_hz(RtcWakeupClock c, uint32_t rtcclk_hz,
                                       const RtcPrescalers& p) {
    const uint8_t div = rtc_wakeup_divider(c);
    if (div != 0u) {
        return rtcclk_hz / div;
    }
    return rtc_ck_spre_hz(p, rtcclk_hz);
}

// =============================================================================
// Calibration (RM0383 17.3.10, 17.3.11, 17.6.7, 17.6.16)
// =============================================================================

/// The SMOOTH calibration window, in seconds. The chapter's own three,
/// and the two shorter ones cost resolution: CALM[1:0] are stuck at 00 at
/// 8 s and CALM[0] at 16 s (17.6.16).
enum class RtcCalibrationWindow : uint8_t {
    seconds32 = 0,
    seconds16 = 1,
    seconds8 = 2,
};

/**
 * A smooth calibration setting. CALM masks out RTCCLK pulses (slowing the
 * calendar, 0.9537 ppm per step) and CALP inserts one every 2^11
 * (+488.5 ppm), so the two together reach roughly -487 .. +488 ppm.
 */
struct RtcCalibration {
    bool plus = false;                                        ///< CALP
    uint16_t minus = 0;                                       ///< CALM, 9 bits
    RtcCalibrationWindow window = RtcCalibrationWindow::seconds32;
};

constexpr bool rtc_calibration_valid(const RtcCalibration& c) {
    if (c.minus > 0x1FFu) {
        return false;
    }
    // 17.6.16's two stuck-bit notes, turned into refusals: a value the
    // hardware would quietly round is a value the caller did not ask for.
    if (c.window == RtcCalibrationWindow::seconds8 && (c.minus & 0x3u) != 0u) {
        return false;
    }
    if (c.window == RtcCalibrationWindow::seconds16 && (c.minus & 0x1u) != 0u) {
        return false;
    }
    return true;
}

/// The correction a smooth setting applies, in parts per billion, signed:
/// (512 x CALP - CALM) pulses in 2^20, scaled. Positive speeds the
/// calendar up. The chapter's exact formula divides by (2^20 + CALM -
/// 512 x CALP); the difference is under a part per million of the result
/// and this stays integer arithmetic on a 32-bit core.
constexpr int32_t rtc_calibration_ppb(const RtcCalibration& c) {
    const int32_t pulses = (c.plus ? 512 : 0) - static_cast<int32_t>(c.minus);
    return static_cast<int32_t>((static_cast<int64_t>(pulses) * 1'000'000'000LL) / (1LL << 20));
}

/**
 * A COARSE calibration setting - RTC_CALIBR, the older mechanism kept
 * "for compatibility reasons" (17.3.10) and covered here because it is
 * part of the chapter.
 *
 * It works at the ck_apre output, not on RTCCLK: with `negative` clear,
 * two ck_apre cycles are ADDED every minute for 2 x steps minutes of a
 * 64-minute cycle (+4.069 ppm a step); with it set, one is REMOVED
 * (-2.035 ppm a step). So the correction is neither smooth nor
 * symmetric, and it is why the smooth calibrator exists.
 */
struct RtcCoarseCalibration {
    bool negative = false;   ///< DCS
    uint8_t steps = 0;       ///< DC[4:0], 0..31
};

constexpr bool rtc_coarse_calibration_valid(const RtcCoarseCalibration& c) {
    return c.steps <= 31u;
}

/// The correction a coarse setting applies, in parts per billion, signed
/// - the chapter's own +4.069 / -2.035 ppm per step (17.3.10, "case of
/// RTCCLK = 32.768 kHz and PREDIV_A + 1 = 128", the only case it prices).
constexpr int32_t rtc_coarse_calibration_ppb(const RtcCoarseCalibration& c) {
    const int32_t steps = static_cast<int32_t>(c.steps);
    return c.negative ? -(steps * 2035) : (steps * 4069);
}

// =============================================================================
// The RTC_OUT pad (RM0383 17.3.14, 17.3.15)
// =============================================================================

/// OSEL - which flag, if any, is routed to RTC_ALARM on the RTC_OUT pad.
/// "Once RTC_ALARM is enabled, it has priority over RTC_CALIB", so this
/// and the calibration output are one pad's two claimants.
enum class RtcOutput : uint8_t {
    off = 0,
    alarm_a = 1,
    alarm_b = 2,
    wakeup = 3,
};

/// COSEL - which clock the calibration output carries. The frequencies
/// are the chapter's for RTCCLK at 32768 Hz: 512 Hz is RTCCLK/64 and
/// needs PREDIV_A = 0x7F, 1 Hz is RTCCLK/(256 x (PREDIV_A + 1)) and needs
/// PREDIV_S[7:0] = 0xFF.
enum class RtcCalibOutput : uint8_t { hz512 = 0, hz1 = 1 };

// =============================================================================
// Tamper detection (RM0383 17.3.13, 17.6.17)
// =============================================================================

/**
 * TAMPFLT - the block's detection MODE, not an input's.
 *
 * `edge` is the register's 0x0 and it is a different detector from the
 * other three: no filter, no sampling clock, no internal pull-up on the
 * pad and no latency between the pad and the flag. The three sample
 * counts are the LEVEL detector, which precharges the pad through the
 * internal pull-up, samples it at TAMPFREQ and fires after N consecutive
 * samples at the active level.
 *
 * One TAFCR serves both inputs, which is why the filter, the sampling
 * rate and the precharge are a BLOCK config here and only the trigger
 * polarity and the enable are per input.
 */
enum class TamperFilter : uint8_t {
    edge = 0,
    samples2 = 1,
    samples4 = 2,
    samples8 = 3,
};

/// TAMPFREQ - how often a filtered input is sampled, as RTCCLK divided by
/// a power of two from 32768 down to 256 (1 Hz to 128 Hz on a 32768 Hz
/// RTCCLK). It is the detection LATENCY's other factor and, through the
/// precharge, the pull-up's duty cycle.
enum class TamperSampling : uint8_t {
    div32768 = 0,
    div16384 = 1,
    div8192 = 2,
    div4096 = 3,
    div2048 = 4,
    div1024 = 5,
    div512 = 6,
    div256 = 7,
};

constexpr uint32_t tamper_sampling_divider(TamperSampling s) {
    return 32768UL >> static_cast<uint32_t>(s);
}

/// The sampling rate in hertz for a STATED RTCCLK - the rate is the
/// caller's argument, the rtc_ck_spre_hz() convention.
constexpr uint32_t tamper_sampling_hz(TamperSampling s, uint32_t rtcclk_hz) {
    return rtcclk_hz / tamper_sampling_divider(s);
}

/// TAMPPRCH - how long a pad is held by the internal pull-up before each
/// sample, in RTCCLK cycles. Larger capacitance on the input wants a
/// longer precharge; TamperConfig::pullup false turns the precharge off
/// entirely (TAMPPUDIS) and makes the sampler read whatever the outside
/// world holds.
enum class TamperPrecharge : uint8_t {
    cycles1 = 0,
    cycles2 = 1,
    cycles4 = 2,
    cycles8 = 3,
};

/**
 * TAMPxTRG - AND THE BIT MEANS OPPOSITE SENSES IN THE TWO MODES, which is
 * why neither value is called "active low" here.
 *
 * 17.6.17 spells it out per bit: with TAMPFLT != 00 a zero means "TAMPER
 * staying LOW triggers", and with TAMPFLT == 00 the same zero means
 * "RISING edge triggers". So the one register bit selects a low level in
 * the filtered detector and a rise in the edge detector - an application
 * that switches its filter on and keeps its trigger constant has just
 * inverted its own polarity. The names carry both readings so the trap
 * cannot be spelled away.
 */
enum class TamperTrigger : uint8_t {
    low_level_or_rising_edge = 0,
    high_level_or_falling_edge = 1,
};

/// The block-wide detection config - the filtered detector's half of
/// TAFCR.
struct TamperConfig {
    TamperFilter filter = TamperFilter::edge;
    TamperSampling sampling = TamperSampling::div32768;
    TamperPrecharge precharge = TamperPrecharge::cycles1;
    bool pullup = true;   ///< the precharge; TAMPPUDIS is its inverse
};

/**
 * One TAMP_INx, in the manual's own 1-based numbering.
 *
 * WHAT ARMING ONE COSTS, said here and not hidden: A DETECTED TAMPER
 * ERASES ALL TWENTY BACKUP REGISTERS (17.3.13). This block has no
 * per-input NOERASE bit and no mask - those are a later family's - so
 * there is no way to arm a tamper input and keep a breadcrumb.
 */
struct TamperInput {
    uint8_t index = 1;
    TamperTrigger trigger = TamperTrigger::low_level_or_rising_edge;
};

/// The index checked against what this part's manual declares (the
/// reserve's rtc_pad_facts(): the header cannot be asked, because ST
/// spells TAMP2E on every F4 header and only two of the four manuals read
/// have a second input).
constexpr bool tamper_input_valid(const TamperInput& t) {
    return t.index >= 1u && t.index <= rtc_pad_facts().tamper_inputs;
}

// =============================================================================
// Flags (RM0383 17.6.4)
// =============================================================================

/// One bit per event, all of them in RTC_ISR - there is no separate
/// status, masked-status and clear register on this family: the flags are
/// rc_w0 bits in the same register as INIT and the write flags.
struct RtcFlag {
    static constexpr uint32_t alarm_a = RTC_ISR_ALRAF;
    static constexpr uint32_t alarm_b = RTC_ISR_ALRBF;
    static constexpr uint32_t wakeup = RTC_ISR_WUTF;
    static constexpr uint32_t timestamp = RTC_ISR_TSF;
    static constexpr uint32_t timestamp_overflow = RTC_ISR_TSOVF;
    static constexpr uint32_t tamper1 = RTC_ISR_TAMP1F;
    /// THE SECOND TAMPER FLAG IS A MANUAL FACT, not a header one: bit 14
    /// is TAMP2F where a second input exists and Reserved where it does
    /// not, and the mask is zero there so that no expression over these
    /// ever writes a one into a reserved bit.
    static constexpr uint32_t tamper2 =
        rtc_pad_facts().tamper_inputs >= 2u ? RTC_ISR_TAMP2F : 0u;
    static constexpr uint32_t tamper = tamper1 | tamper2;
    static constexpr uint32_t all =
        alarm_a | alarm_b | wakeup | timestamp | timestamp_overflow | tamper;
};

// =============================================================================
// The peripheral
// =============================================================================

/**
 * The RTC.
 *
 * MONOSTATE, like every other single-instance driver in this stratum
 * (there is one RTC on every part of the family, and one domain for it to
 * live in).
 *
 * WHAT IT DOES NOT DO: it never writes RCC_BDCR - that is RtcDomain's
 * register, whole - and it never enables an NVIC line for you. It does
 * write the three EXTI bits its interrupts need (fact 7), because without
 * them no interrupt of this peripheral ever reaches the NVIC and leaving
 * that to the application would be leaving a trap.
 */
struct Rtc {
    Rtc() = delete;

    static constexpr uint32_t key_unlock1 = 0xCAu;
    static constexpr uint32_t key_unlock2 = 0x53u;
    static constexpr uint32_t key_lock = 0xFFu;   ///< any wrong key re-locks

    /// A bound on every synchronization wait. Sized for the slowest thing
    /// here - "1 to 2 RTCCLK cycles" at 32768 Hz is 61 us, and
    /// exit_init()'s RSF wait is a whole RTCCLK period - with room to
    /// spare, and small enough that a STOPPED RTCCLK is reported as false
    /// in a few milliseconds rather than hanging.
    static constexpr uint32_t sync_spins = 1'000'000UL;

    static constexpr uint8_t backup_count = rtc_backup_registers();
    static constexpr uint8_t tamper_inputs = rtc_pad_facts().tamper_inputs;
    static constexpr bool has_second_pad = rtc_pad_facts().has_af2;

    static constexpr uint8_t alarm_exti_line = rtc_alarm_exti_line;
    static constexpr uint8_t wakeup_exti_line = rtc_wakeup_exti_line;
    static constexpr uint8_t tamper_stamp_exti_line = rtc_tamper_stamp_exti_line;

    static constexpr IRQn_Type alarm_irq() { return rtc_alarm_irq(); }
    static constexpr IRQn_Type wakeup_irq() { return rtc_wakeup_irq(); }
    static constexpr IRQn_Type tamper_stamp_irq() { return rtc_tamper_stamp_irq(); }

    // ---- write protection (17.3.5) ------------------------------------------

    static void unlock() {
        RTC->WPR = key_unlock1;
        RTC->WPR = key_unlock2;
    }
    static void lock() { RTC->WPR = key_lock; }

    // ---- raw register readbacks ---------------------------------------------

    static uint32_t cr() { return RTC->CR; }
    static uint32_t isr() { return RTC->ISR; }
    static uint32_t prer() { return RTC->PRER; }
    static uint32_t wutr() { return RTC->WUTR; }
    static uint32_t calr() { return RTC->CALR; }
    static uint32_t calibr() { return RTC->CALIBR; }
    static uint32_t tafcr() { return RTC->TAFCR; }
    static uint32_t status() { return RTC->ISR & RtcFlag::all; }

    /// INITS - "the calendar year field is different from 0", i.e. the
    /// calendar has been set since the last domain reset (17.6.4). The
    /// only thing that tells a fresh boot whether the RTC it inherited
    /// means anything.
    static bool calendar_set() { return (RTC->ISR & RTC_ISR_INITS) != 0u; }
    static bool in_init() { return (RTC->ISR & RTC_ISR_INITF) != 0u; }
    static bool synchronized() { return (RTC->ISR & RTC_ISR_RSF) != 0u; }
    static bool shift_pending() { return (RTC->ISR & RTC_ISR_SHPF) != 0u; }
    static bool recalibration_pending() { return (RTC->ISR & RTC_ISR_RECALPF) != 0u; }
    static bool bypass_shadow() { return (RTC->CR & RTC_CR_BYPSHAD) != 0u; }

    // ---- initialization mode (17.3.5, ES0287 2.8.4) -------------------------

    /**
     * Enter initialization mode: the calendar counter stops and TR, DR
     * and PRER become writable. False when INITF never rose, which on a
     * stopped RTCCLK is always. Brackets its own write protection.
     */
    static bool enter_init() {
        unlock();
        RTC->ISR = RTC->ISR | RTC_ISR_INIT;
        for (uint32_t i = 0; i < sync_spins; ++i) {
            if (in_init()) {
                lock();
                return true;
            }
        }
        lock();
        return false;
    }

    /**
     * Leave initialization mode, ES0287 2.8.4's WORKAROUND INCLUDED.
     *
     * The erratum is that a second INIT entry within one to two RTCCLK
     * cycles of the first exit sets INITF immediately, and calendar
     * writes made in that window may be dropped or corrupted; the
     * workaround is to "clear the BYPSHAD bit (if set) then wait for RSF
     * to rise, before entering the initialization mode again". Doing it
     * HERE rather than at the next entry is what makes it unconditional:
     * a caller cannot forget, and cannot get it wrong by entering
     * initialization from a different code path.
     *
     * False = RSF never rose within the bound, which means the RTC is not
     * counting and the calendar written just now is not in force.
     */
    static bool exit_init() {
        unlock();
        RTC->ISR = RTC->ISR & ~RTC_ISR_INIT;
        const bool bypass = bypass_shadow();
        if (bypass) {
            RTC->CR = RTC->CR & ~RTC_CR_BYPSHAD;
        }
        RTC->ISR = clear_rsf_value();
        bool ok = false;
        for (uint32_t i = 0; i < sync_spins; ++i) {
            if (synchronized()) {
                ok = true;
                break;
            }
        }
        if (bypass) {
            RTC->CR = RTC->CR | RTC_CR_BYPSHAD;
        }
        lock();
        return ok;
    }

    /**
     * Leave initialization mode WITHOUT the erratum workaround - the
     * chapter's bare sequence, and nothing else.
     *
     * It exists for two reasons and neither is convenience: ES0287 2.8.4
     * cannot be STAGED at a bench without a path that does not dodge it
     * (and an app may not poke a register itself), and a caller that is
     * about to do the RSF wait for its own reasons should not pay for it
     * twice. Anything else calls exit_init().
     */
    static void exit_init_raw() {
        unlock();
        RTC->ISR = RTC->ISR & ~RTC_ISR_INIT;
        lock();
    }

    /// Clear RSF and wait for the next shadow copy - what 17.3.6 asks for
    /// after a system reset, an initialization, a shift, and AFTER EVERY
    /// STOP OR STANDBY. A no-op that answers true when the shadow
    /// registers are bypassed, there being nothing to synchronize.
    static bool wait_sync() {
        if (bypass_shadow()) {
            return true;
        }
        unlock();
        RTC->ISR = clear_rsf_value();
        lock();
        for (uint32_t i = 0; i < sync_spins; ++i) {
            if (synchronized()) {
                return true;
            }
        }
        return false;
    }

    // ---- configuration ------------------------------------------------------

    /// Write both prescalers. 17.6.5: "the initialization must be
    /// performed in two separate write accesses", the synchronous one
    /// first - the chapter's order, not an arbitrary one. Legal only in
    /// initialization mode, which is the refusal.
    static bool set_prescalers(const RtcPrescalers& p) {
        if (!rtc_prescalers_valid(p) || !in_init()) {
            return false;
        }
        unlock();
        RTC->PRER = static_cast<uint32_t>(p.sync) << RTC_PRER_PREDIV_S_Pos;
        RTC->PRER = (static_cast<uint32_t>(p.sync) << RTC_PRER_PREDIV_S_Pos) |
                    (static_cast<uint32_t>(p.async) << RTC_PRER_PREDIV_A_Pos);
        lock();
        return true;
    }

    /// The same write from a CONSTEXPR pair, refused at COMPILE time
    /// instead of at run time - the stratum's standing pattern (a
    /// configuration a program knows at build time should not be
    /// discovered wrong on a bench).
    template <RtcPrescalers p>
    static bool set_prescalers() {
        static_assert(rtc_prescalers_valid(p),
                      "brio Rtc: PREDIV_A is 7 bits and PREDIV_S is 15");
        return set_prescalers(p);
    }

    static RtcPrescalers prescalers() {
        const uint32_t v = RTC->PRER;
        return RtcPrescalers{
            static_cast<uint8_t>((v & RTC_PRER_PREDIV_A_Msk) >> RTC_PRER_PREDIV_A_Pos),
            static_cast<uint16_t>((v & RTC_PRER_PREDIV_S_Msk) >> RTC_PRER_PREDIV_S_Pos)};
    }

    /// BYPSHAD (fact 4). Writable outside initialization mode - 17.6.3's
    /// note about bits 7, 6 and 4 covers DCE, FMT and REFCKON, not this
    /// one.
    static void bypass_shadow(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_BYPSHAD) : (RTC->CR & ~RTC_CR_BYPSHAD);
        lock();
    }

    /**
     * Write the calendar - TR and DR - and pin the 24-hour format. Legal
     * ONLY in initialization mode (17.6.1 and 17.6.2 both say "must be
     * written in initialization mode only"), which is the refusal; init()
     * below is this verb plus the mode and the prescalers, and a caller
     * doing its own initialization sequence (staging ES0287 2.8.4, for
     * one) uses this.
     */
    static bool set_calendar(const RtcDateTime& d) {
        if (!rtc_datetime_valid(d) || !in_init()) {
            return false;
        }
        unlock();
        RTC->TR = rtc_time_register(d);
        RTC->DR = rtc_date_register(d);
        RTC->CR = RTC->CR & ~RTC_CR_FMT;   // 24-hour format, always
        lock();
        return true;
    }

    /**
     * The whole boot sequence of 17.3.5: enter initialization mode, set
     * the prescalers, set the calendar, leave - with the erratum
     * workaround on the way out. `time` is not optional in the sense a
     * caller might hope: there is no way to set the prescalers without
     * stopping the counter, so a program that wants to keep a calendar
     * that survived a reset passes the one it just read.
     */
    static bool init(const RtcPrescalers& p, const RtcDateTime& time) {
        if (!rtc_prescalers_valid(p) || !rtc_datetime_valid(time)) {
            return false;
        }
        if (!enter_init()) {
            return false;
        }
        if (!set_prescalers(p)) {
            (void)exit_init();
            return false;
        }
        if (!set_calendar(time)) {
            (void)exit_init();
            return false;
        }
        return exit_init();
    }

    // ---- reading the calendar (17.3.6, ES0287 2.8.2) ------------------------

    /**
     * ONE COHERENT LOOK at sub-second, time and date.
     *
     * Two modes, two disciplines:
     *  - shadow (BYPSHAD = 0): RSF first - an unsynchronized shadow is a
     *    reset's default value, not a time - then SSR, TR, DR and SSR
     *    AGAIN. The chapter says the first read locks the higher-order
     *    shadows until DR is read; ES0287 2.8.2 says the lock can be
     *    missed, and its own workaround is the second SSR read. A
     *    disagreement is retried.
     *  - bypass (BYPSHAD = 1): the counters are read live and an RTCCLK
     *    edge between two reads makes them disagree, so the whole triple
     *    is read twice and compared (17.3.6's own instruction).
     *
     * False = the reading could not be made coherent (a stopped RTCCLK in
     * bypass mode, an RSF that never rose in shadow mode). `out` is
     * untouched then.
     */
    static bool read(RtcReading& out) {
        if (!bypass_shadow()) {
            if (!synchronized()) {
                return false;
            }
            for (uint8_t attempt = 0; attempt < 4u; ++attempt) {
                const uint16_t ss0 = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
                const uint32_t tr = RTC->TR;
                const uint32_t dr = RTC->DR;
                const uint16_t ss1 = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
                if (ss0 == ss1) {
                    out.subsecond = ss0;
                    out.time = rtc_decode(tr, dr);
                    return true;
                }
            }
            return false;
        }
        uint16_t ss0 = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
        uint32_t tr0 = RTC->TR;
        uint32_t dr0 = RTC->DR;
        for (uint8_t attempt = 0; attempt < 4u; ++attempt) {
            const uint16_t ss1 = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
            const uint32_t tr1 = RTC->TR;
            const uint32_t dr1 = RTC->DR;
            if (ss0 == ss1 && tr0 == tr1 && dr0 == dr1) {
                out.subsecond = ss1;
                out.time = rtc_decode(tr1, dr1);
                return true;
            }
            ss0 = ss1;
            tr0 = tr1;
            dr0 = dr1;
        }
        return false;
    }

    /**
     * RTC_SSR alone - the synchronous prescaler's counter, counting DOWN
     * from PREDIV_S and reloading at every calendar update.
     *
     * read() gives the calendar; this gives the PHASE, in one load where
     * read() costs four plus a coherence check. It is what 17.3.8's
     * synchronization procedure actually needs (the offset to a remote
     * clock is computed from SS and nothing else), it is shift()'s own
     * precondition, and it is the only reading fine enough to catch the
     * 1 Hz edge AS IT HAPPENS: a value that has RISEN since the previous
     * read is a reload, located to whatever the caller's loop costs
     * rather than to the 1/(PREDIV_S + 1) second the value itself
     * resolves. With BYPSHAD clear it is a shadow reading like any other
     * and the caller owes wait_sync() first.
     *
     * IT IS TWO LOADS AND NOT ONE, and that is 17.3.6's instruction for
     * the bypass mode taken at its word: "the value of one of the
     * registers may be incorrect if an RTCCLK edge occurs during the read
     * operation ... alternatively, the software can just compare the two
     * results of the least-significant calendar register". A torn value
     * read out of the live counter is not a rare accident - a loop
     * polling this register continuously catches roughly nine of them a
     * second at 32768 Hz (measured), and a caller watching for the reload
     * edge would see every one of them as a second. So the confirmation
     * is here, bounded, rather than owed by every caller.
     */
    static uint16_t subsecond() {
        uint16_t v = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
        for (uint8_t i = 0; i < 4u; ++i) {
            const uint16_t again = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
            if (again == v) {
                break;
            }
            v = again;
        }
        return v;
    }

    // ---- daylight saving (17.3.5) -------------------------------------------

    /// ADD1H / SUB1H: one hour in one operation, without stopping the
    /// calendar. SUB1H "has no effect when current hour is 0" - the
    /// chapter's own sentence, not enforced here because the register
    /// enforces it. Both changes take effect in the next second.
    static void shift_hour(bool add) {
        unlock();
        RTC->CR = RTC->CR | (add ? RTC_CR_ADD1H : RTC_CR_SUB1H);
        lock();
    }

    /// The BKP bit of RTC_CR - one bit of memory for "I have already done
    /// the summer-time change", and nothing else.
    static void daylight_flag(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_BKP) : (RTC->CR & ~RTC_CR_BKP);
        lock();
    }
    static bool daylight_flag() { return (RTC->CR & RTC_CR_BKP) != 0u; }

    // ---- the sub-second shift (17.3.8) --------------------------------------

    /**
     * Shift the calendar by a fraction of a second - RTC_SHIFTR.
     *
     * `subfs` is ADDED to the synchronous prescaler counter, which counts
     * DOWN, so it DELAYS the clock by subfs / (PREDIV_S + 1) seconds. With
     * `add1s` the whole second is added first, so the pair ADVANCES the
     * clock by 1 - subfs / (PREDIV_S + 1) seconds in one atomic
     * operation. That asymmetry is the register's, not this verb's: there
     * is no "subtract a second" bit.
     *
     * FOUR REFUSALS, every one of them a sentence of 17.3.8 or 17.6.12:
     *  - `subfs` past the 15-bit field;
     *  - a shift already pending (SHPF), where the register says a write
     *    "has no effect" - a silent loss this verb turns into a false;
     *  - REFCKON set, which 17.3.8's caution forbids outright;
     *  - SS[15] set, the caution's overflow guard. SS is 16 bits and
     *    PREDIV_S is 15, so bit 15 can only be standing because an earlier
     *    shift put it there, and shifting again from that state is what
     *    the chapter tells the caller to avoid.
     *
     * It INITIATES and returns: the operation takes up to a second of
     * RTCCLK to land and shift_pending() is the completion. Writing SUBFS
     * also clears RSF, so a caller reading through the shadow registers
     * waits for synchronized() before believing what it reads.
     */
    static bool shift(bool add1s, uint16_t subfs) {
        if (subfs > 0x7FFFu) {
            return false;
        }
        if (shift_pending() || reference_clock()) {
            return false;
        }
        if ((RTC->SSR & 0x8000u) != 0u) {
            return false;
        }
        unlock();
        RTC->SHIFTR = (add1s ? RTC_SHIFTR_ADD1S : 0u) |
                      (static_cast<uint32_t>(subfs) << RTC_SHIFTR_SUBFS_Pos);
        lock();
        return true;
    }

    // ---- reference clock detection (17.3.9) ---------------------------------
    //
    // REFCKON does NOT clock the calendar - RTCCLK still does. What it
    // buys is a RELOAD of the asynchronous prescaler whenever an edge of
    // RTC_REFIN is seen inside the detection window around a calendar
    // update, which drags the 1 Hz edge onto the reference's and makes the
    // calendar as accurate as the reference instead of as accurate as the
    // crystal. The correction is therefore quantized at one ck_apre period
    // per second, and the window is 7 ck_apre periods hunting for the
    // first edge and 3 once locked.
    //
    // THREE RULES, all of them refusals here rather than notes:
    //  - REFCKON is bit 4 of RTC_CR, which 17.6.3's note says may be
    //    written in INITIALIZATION MODE ONLY;
    //  - the prescalers must be the default pair (PREDIV_A = 0x7F,
    //    PREDIV_S = 0xFF), because the detection is built on a 256 Hz
    //    ck_apre (17.3.9's own "must");
    //  - the coarse calibrator must be off, RTC_CALIBR at zero (17.3.9's
    //    caution).
    //
    // And one interlock from the other side: a shift must not be issued
    // while this is on (17.3.8's caution), which shift() enforces.

    static bool reference_clock() { return (RTC->CR & RTC_CR_REFCKON) != 0u; }

    static bool reference_clock(bool on) {
        if (!in_init()) {
            return false;
        }
        if (on) {
            const RtcPrescalers p = prescalers();
            if (p.async != 0x7Fu || p.sync != 0xFFu || RTC->CALIBR != 0u) {
                return false;
            }
        }
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_REFCKON) : (RTC->CR & ~RTC_CR_REFCKON);
        lock();
        return true;
    }

    // ---- alarms -------------------------------------------------------------

    static constexpr uint32_t alarm_enable_bit(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RTC_CR_ALRAE : RTC_CR_ALRBE;
    }
    static constexpr uint32_t alarm_interrupt_bit(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RTC_CR_ALRAIE : RTC_CR_ALRBIE;
    }
    static constexpr uint32_t alarm_write_flag(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RTC_ISR_ALRAWF : RTC_ISR_ALRBWF;
    }
    static constexpr uint32_t alarm_flag(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RtcFlag::alarm_a : RtcFlag::alarm_b;
    }

    static bool alarm_enabled(RtcAlarmId id) { return (RTC->CR & alarm_enable_bit(id)) != 0u; }

    /**
     * Program an alarm: 17.3.5's own four steps - clear the enable, poll
     * the write flag, write the two registers, set the enable again -
     * with the flag cleared and the interrupt enable set or left alone as
     * asked, and the EXTI line opened when it is set.
     *
     * THE CAUTION IS ENFORCED. 17.3.3: "if the seconds field is selected
     * (MSK1 bit reset ...), the synchronous prescaler division factor set
     * in the RTC_PRER register must be at least 3" - so an alarm
     * comparing seconds under a PREDIV_S below 2 is refused rather than
     * armed into an undefined comparison.
     */
    static bool set_alarm(RtcAlarmId id, const RtcAlarm& a, bool interrupt = true) {
        if (!rtc_alarm_valid(a)) {
            return false;
        }
        if (!a.mask_seconds && prescalers().sync < 2u) {
            return false;
        }
        unlock();
        RTC->CR = RTC->CR & ~(alarm_enable_bit(id) | alarm_interrupt_bit(id));
        lock();
        bool ready = in_init();
        for (uint32_t i = 0; !ready && i < sync_spins; ++i) {
            ready = (RTC->ISR & alarm_write_flag(id)) != 0u;
        }
        if (!ready) {
            return false;
        }
        unlock();
        if (id == RtcAlarmId::a) {
            RTC->ALRMAR = rtc_alarm_register(a);
            RTC->ALRMASSR = rtc_alarm_subsecond_register(a);
        } else {
            RTC->ALRMBR = rtc_alarm_register(a);
            RTC->ALRMBSSR = rtc_alarm_subsecond_register(a);
        }
        lock();
        clear_flags(alarm_flag(id));
        unlock();
        RTC->CR = RTC->CR | alarm_enable_bit(id) | (interrupt ? alarm_interrupt_bit(id) : 0u);
        lock();
        if (interrupt) {
            wake_line_open(alarm_exti_line);
        }
        return true;
    }

    /// The same alarm from a CONSTEXPR description, refused at compile
    /// time - a mask past MASKSS's four bits or a field past its range is
    /// a build error and not a bench surprise.
    template <RtcAlarm a>
    static bool set_alarm(RtcAlarmId id, bool interrupt = true) {
        static_assert(rtc_alarm_valid(a),
                      "brio Rtc: MASKSS is 0..15, SS is 15 bits, and an unmasked "
                      "field must hold a legal date or time");
        return set_alarm(id, a, interrupt);
    }

    /// Turn an alarm off, interrupt and all, and clear its flag.
    static void clear_alarm(RtcAlarmId id) {
        unlock();
        RTC->CR = RTC->CR & ~(alarm_enable_bit(id) | alarm_interrupt_bit(id));
        lock();
        clear_flags(alarm_flag(id));
    }

    // ---- the wake-up timer --------------------------------------------------

    static bool wakeup_enabled() { return (RTC->CR & RTC_CR_WUTE) != 0u; }
    static bool wakeup_write_allowed() { return (RTC->ISR & RTC_ISR_WUTWF) != 0u; }

    /**
     * Program the periodic wake-up timer: 17.3.5's sequence exactly -
     * clear WUTE, POLL WUTWF, write WUT and WUCKSEL, set WUTE.
     *
     * `reload` is the register value: the flag comes every reload + 1
     * ticks of ck_wut, which is what rtc_wakeup_clock_hz() prices. False
     * when the setting is impossible or when WUTWF never rose (a stopped
     * RTCCLK).
     *
     * The flag is cleared before the timer is enabled; 17.6.4's "WUTF
     * must be cleared by software at least 1.5 RTCCLK periods before WUTF
     * is set to 1 again" is then the caller's obligation in the handler,
     * and wakeup_isr() meets it by clearing at entry.
     */
    static bool set_wakeup(RtcWakeupClock c, uint32_t reload, bool interrupt = true) {
        if (!rtc_wakeup_valid(c, reload)) {
            return false;
        }
        unlock();
        RTC->CR = RTC->CR & ~(RTC_CR_WUTE | RTC_CR_WUTIE);
        lock();
        bool ready = false;
        for (uint32_t i = 0; !ready && i < sync_spins; ++i) {
            ready = wakeup_write_allowed();
        }
        if (!ready) {
            return false;
        }
        unlock();
        RTC->WUTR = reload & RTC_WUTR_WUT_Msk;
        RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL_Msk) |
                  (static_cast<uint32_t>(c) << RTC_CR_WUCKSEL_Pos);
        lock();
        clear_flags(RtcFlag::wakeup);
        unlock();
        RTC->CR = RTC->CR | RTC_CR_WUTE | (interrupt ? RTC_CR_WUTIE : 0u);
        lock();
        if (interrupt) {
            wake_line_open(wakeup_exti_line);
        }
        return true;
    }

    /// The same setting from CONSTEXPR arguments, refused at compile
    /// time: a reload past the 16-bit field, and 17.6.6's forbidden pair
    /// (RTCCLK/2 with a reload of zero), are build errors here.
    template <RtcWakeupClock c, uint32_t reload>
    static bool set_wakeup(bool interrupt = true) {
        static_assert(rtc_wakeup_valid(c, reload),
                      "brio Rtc: WUT is 16 bits, and RTCCLK/2 with WUT = 0 is forbidden");
        return set_wakeup(c, reload, interrupt);
    }

    /// Stop the wake-up timer and clear its flag. Returns once WUTE is
    /// written; WUTWF follows "up to 2 RTCCLK clock cycles" later
    /// (17.6.4), which is set_wakeup()'s wait and not this verb's.
    static void clear_wakeup() {
        unlock();
        RTC->CR = RTC->CR & ~(RTC_CR_WUTE | RTC_CR_WUTIE);
        lock();
        clear_flags(RtcFlag::wakeup);
    }

    // ---- smooth calibration (17.3.11) ---------------------------------------

    /**
     * Write RTC_CALR. RECALPF is waited out first: 17.6.4 says the
     * register "is blocked" while it stands, and 17.3.11's
     * re-calibration-on-the-fly makes that up to three ck_apre periods.
     * Refused while the COARSE calibrator is enabled (fact 8).
     */
    static bool calibrate(const RtcCalibration& c) {
        if (!rtc_calibration_valid(c)) {
            return false;
        }
        if ((RTC->CR & RTC_CR_DCE) != 0u) {
            return false;
        }
        for (uint32_t i = 0; i < sync_spins; ++i) {
            if (!recalibration_pending()) {
                break;
            }
        }
        if (recalibration_pending()) {
            return false;
        }
        uint32_t v = static_cast<uint32_t>(c.minus) & RTC_CALR_CALM_Msk;
        if (c.plus) {
            v |= RTC_CALR_CALP;
        }
        if (c.window == RtcCalibrationWindow::seconds8) {
            v |= RTC_CALR_CALW8;
        } else if (c.window == RtcCalibrationWindow::seconds16) {
            v |= RTC_CALR_CALW16;
        }
        unlock();
        RTC->CALR = v;
        lock();
        return true;
    }

    /// The same setting from a CONSTEXPR one, refused at compile time -
    /// including 17.6.16's stuck bits, which a shorter window makes
    /// unreachable rather than merely inaccurate.
    template <RtcCalibration c>
    static bool calibrate() {
        static_assert(rtc_calibration_valid(c),
                      "brio Rtc: CALM is 9 bits, and its low bits are stuck at zero "
                      "in the 16- and 8-second windows");
        return calibrate(c);
    }

    static RtcCalibration calibration() {
        const uint32_t v = RTC->CALR;
        RtcCalibration c{};
        c.plus = (v & RTC_CALR_CALP) != 0u;
        c.minus = static_cast<uint16_t>(v & RTC_CALR_CALM_Msk);
        c.window = (v & RTC_CALR_CALW8)    ? RtcCalibrationWindow::seconds8
                   : (v & RTC_CALR_CALW16) ? RtcCalibrationWindow::seconds16
                                           : RtcCalibrationWindow::seconds32;
        return c;
    }

    /**
     * Store into RTC_CALR WITHOUT opening the key window - the one verb in
     * this file that deliberately does not bracket itself.
     *
     * It exists so that the write protection can be PROVEN rather than
     * asserted: a lock is only evidenced by a store that does NOT land,
     * and an application may not reach a register itself (the stratum's
     * standing rule). CALR is the register chosen for it because the worst
     * a wrong value there can do is move the calendar by a few hundred
     * parts per million. Nothing but a bench suite should call this.
     */
    static void calr_unprotected(uint32_t v) { RTC->CALR = v; }

    // ---- coarse calibration (17.3.10) ---------------------------------------

    /**
     * RTC_CALIBR and the DCE bit that arms it - the older calibrator,
     * whole, because it is part of the chapter.
     *
     * THREE REFUSALS: initialization mode only (17.6.7 and 17.6.3's note
     * on bit 7); PREDIV_A at least 6, because 17.3.10's caution says the
     * calibration "may not work correctly" below it; and RTC_CALR at zero,
     * because the two calibrators are not to be used together (fact 8).
     * Disabling needs the mode too, and nothing else.
     */
    static bool coarse_calibrate(bool on, const RtcCoarseCalibration& c = {}) {
        if (!in_init()) {
            return false;
        }
        if (!on) {
            unlock();
            RTC->CR = RTC->CR & ~RTC_CR_DCE;
            RTC->CALIBR = 0u;
            lock();
            return true;
        }
        if (!rtc_coarse_calibration_valid(c) || prescalers().async < 6u || RTC->CALR != 0u) {
            return false;
        }
        unlock();
        RTC->CALIBR = (c.negative ? RTC_CALIBR_DCS : 0u) |
                      (static_cast<uint32_t>(c.steps) << RTC_CALIBR_DC_Pos);
        RTC->CR = RTC->CR | RTC_CR_DCE;
        lock();
        return true;
    }

    static bool coarse_calibration_enabled() { return (RTC->CR & RTC_CR_DCE) != 0u; }

    static RtcCoarseCalibration coarse_calibration() {
        const uint32_t v = RTC->CALIBR;
        return RtcCoarseCalibration{(v & RTC_CALIBR_DCS) != 0u,
                                    static_cast<uint8_t>((v & RTC_CALIBR_DC_Msk) >>
                                                         RTC_CALIBR_DC_Pos)};
    }

    // ---- the timestamp (17.3.12) --------------------------------------------

    /// TSE and TSEDGE. "TSE must be reset when TSEDGE is changed to avoid
    /// unwanted TSF setting" - so the edge is written with the enable
    /// down, always, and the enable goes back up in a second store.
    static void timestamp_enable(bool on, bool falling_edge = false) {
        unlock();
        uint32_t v = RTC->CR & ~(RTC_CR_TSE | RTC_CR_TSEDGE);
        if (falling_edge) {
            v |= RTC_CR_TSEDGE;
        }
        RTC->CR = v;
        if (on) {
            RTC->CR = v | RTC_CR_TSE;
        }
        lock();
    }
    static bool timestamp_enabled() { return (RTC->CR & RTC_CR_TSE) != 0u; }

    /// TSIE - the timestamp's own interrupt enable, on the tamper and
    /// timestamp vector.
    static void timestamp_interrupt(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_TSIE) : (RTC->CR & ~RTC_CR_TSIE);
        lock();
        if (on) {
            wake_line_open(tamper_stamp_exti_line);
        }
    }

    /// TSTR/TSDR/TSSSR as one reading. Valid only while TSF stands - the
    /// registers are frozen from the event until the flag is cleared
    /// (17.6.13).
    static RtcReading timestamp() {
        RtcReading r{};
        r.subsecond = static_cast<uint16_t>(RTC->TSSSR & RTC_TSSSR_SS_Msk);
        r.time = rtc_decode(RTC->TSTR, RTC->TSDR);
        return r;
    }

    // ---- the RTC_OUT pad (17.3.14, 17.3.15) ---------------------------------

    /**
     * RTC_ALARM on the pad: OSEL picks the flag, POL its polarity, and
     * ALARMOUTTYPE (in TAFCR, so outside the key) picks open drain or push
     * pull. RM0383's note is the reason no GPIO verb is called here: "when
     * RTC_CALIB or RTC_ALARM is selected, RTC_OUT is automatically
     * configured in output alternate function".
     *
     * The pad is RTC_AF1 - PC13 on every part whose manual was read - and
     * it is the same pad the tamper input, the timestamp input and the
     * calibration output claim. RTC_ALARM has priority over RTC_CALIB
     * (17.3.15), which is the silicon's rule and not this verb's.
     */
    static void alarm_output(RtcOutput sel, bool active_low = false, bool push_pull = false) {
        unlock();
        RTC->CR = (RTC->CR & ~(RTC_CR_OSEL_Msk | RTC_CR_POL)) |
                  (static_cast<uint32_t>(sel) << RTC_CR_OSEL_Pos) |
                  (active_low ? RTC_CR_POL : 0u);
        lock();
        RTC->TAFCR = push_pull ? (RTC->TAFCR | RTC_TAFCR_ALARMOUTTYPE)
                               : (RTC->TAFCR & ~RTC_TAFCR_ALARMOUTTYPE);
    }
    static RtcOutput alarm_output() {
        return static_cast<RtcOutput>((RTC->CR & RTC_CR_OSEL_Msk) >> RTC_CR_OSEL_Pos);
    }

    /**
     * RTC_CALIB on the pad: COE and COSEL. The two frequencies are the
     * chapter's, and each has a PRESCALER PRECONDITION which this verb
     * enforces rather than states - 512 Hz is RTCCLK/64 and wants
     * PREDIV_A = 0x7F, 1 Hz is RTCCLK/(256 x (PREDIV_A + 1)) and wants
     * PREDIV_S + 1 to be a non-zero multiple of 256, i.e.
     * PREDIV_S[7:0] = 0xFF. False when the running prescalers do not
     * satisfy the one asked for; nothing is written then.
     *
     * The output "is not impacted by the calibration value programmed in
     * RTC_CALIBR" and its duty cycle is irregular - "there is a light
     * jitter on falling edges", so a measurement counts RISING edges.
     */
    static bool calibration_output(bool on, RtcCalibOutput which = RtcCalibOutput::hz512) {
        if (on) {
            const RtcPrescalers p = prescalers();
            if (which == RtcCalibOutput::hz512 && p.async != 0x7Fu) {
                return false;
            }
            if (which == RtcCalibOutput::hz1 && (p.sync & 0xFFu) != 0xFFu) {
                return false;
            }
        }
        unlock();
        uint32_t v = RTC->CR & ~(RTC_CR_COE | RTC_CR_COSEL);
        if (which == RtcCalibOutput::hz1) {
            v |= RTC_CR_COSEL;
        }
        if (on) {
            v |= RTC_CR_COE;
        }
        RTC->CR = v;
        lock();
        return true;
    }
    static bool calibration_output() { return (RTC->CR & RTC_CR_COE) != 0u; }

    /**
     * TSINSEL and TAMP1INSEL - which pad the timestamp input and TAMPER1
     * read, RTC_AF1 or RTC_AF2. Refused on a part whose manual gives it
     * no RTC_AF2 (the reserve's rtc_pad_facts()), where the register's 1
     * is Reserved.
     *
     * 17.3.12's rule, which the caller owns because it is about two
     * features at once: "mapping the timestamp event on RTC_AF2 is not
     * allowed if RTC_AF1 is used as TAMPER".
     */
    static bool timestamp_pad(bool second_pad) {
        if (second_pad && !has_second_pad) {
            return false;
        }
        RTC->TAFCR = second_pad ? (RTC->TAFCR | RTC_TAFCR_TSINSEL)
                                : (RTC->TAFCR & ~RTC_TAFCR_TSINSEL);
        return true;
    }
    static bool tamper_pad(bool second_pad) {
        if (second_pad && !has_second_pad) {
            return false;
        }
        RTC->TAFCR = second_pad ? (RTC->TAFCR | RTC_TAFCR_TAMP1INSEL)
                                : (RTC->TAFCR & ~RTC_TAFCR_TAMP1INSEL);
        return true;
    }

    // ---- tamper detection (17.3.13) -----------------------------------------

    static bool tamper_armed(uint8_t index) {
        const uint32_t bit = tamper_enable_bit(index);
        return bit != 0u && (RTC->TAFCR & bit) != 0u;
    }
    static bool any_tamper_armed() {
        return (RTC->TAFCR & (RTC_TAFCR_TAMP1E | tamper2_enable_bit())) != 0u;
    }

    /**
     * The block's detection mode - the filter, the sampling rate, the
     * precharge and the pull-up.
     *
     * REFUSED WHILE ANY INPUT IS ARMED. Nothing in the silicon enforces
     * it, so a store into a live block would land and change what a
     * running detector means halfway through a sample train; and
     * TAMPxTRG's meaning depends on TAMPFLT (see TamperTrigger), so a
     * filter change under an armed input silently inverts its polarity.
     */
    static bool tamper_config(const TamperConfig& c) {
        if (any_tamper_armed()) {
            return false;
        }
        uint32_t v = RTC->TAFCR & ~(RTC_TAFCR_TAMPPUDIS | RTC_TAFCR_TAMPPRCH_Msk |
                                    RTC_TAFCR_TAMPFLT_Msk | RTC_TAFCR_TAMPFREQ_Msk);
        v |= static_cast<uint32_t>(c.sampling) << RTC_TAFCR_TAMPFREQ_Pos;
        v |= static_cast<uint32_t>(c.filter) << RTC_TAFCR_TAMPFLT_Pos;
        v |= static_cast<uint32_t>(c.precharge) << RTC_TAFCR_TAMPPRCH_Pos;
        if (!c.pullup) {
            v |= RTC_TAFCR_TAMPPUDIS;
        }
        RTC->TAFCR = v;
        return true;
    }

    static TamperFilter tamper_filter() {
        return static_cast<TamperFilter>((RTC->TAFCR & RTC_TAFCR_TAMPFLT_Msk) >>
                                         RTC_TAFCR_TAMPFLT_Pos);
    }

    /**
     * Arm one tamper input. THE TRIGGER LANDS BEFORE THE ENABLE, which is
     * 17.6.17's own caution ("when TAMPFLT = 0, TAMPxE must be reset when
     * TAMPxTRG is changed to avoid spuriously setting TAMPxF"), and
     * re-arming an armed input is refused for the same reason - the caller
     * disarms first, which is also 17.3.13's advice after a detection.
     *
     * AND IT MAY FIRE AT ONCE, BY DESIGN: in edge mode the detection
     * signal "is logically ANDed with TAMPxE", so an input already at the
     * active level when the enable goes up is a detection immediately -
     * which is the only way to stage a tamper with no wire, and a trap
     * for anybody who expects an edge.
     */
    static bool tamper_arm(const TamperInput& t) {
        if (!tamper_input_valid(t) || tamper_armed(t.index)) {
            return false;
        }
        const uint32_t trg = tamper_trigger_bit(t.index);
        uint32_t v = RTC->TAFCR & ~trg;
        if (t.trigger == TamperTrigger::high_level_or_falling_edge) {
            v |= trg;
        }
        RTC->TAFCR = v;
        RTC->TAFCR = v | tamper_enable_bit(t.index);
        return true;
    }

    /**
     * Disarm one input and clear its flag.
     *
     * ES0287 2.8.1 HAS NO WORKAROUND: disabling a falling-edge channel
     * while its pin is high raises a false detection - and a detection
     * erases the backup registers, which no software can undo. Clearing
     * the flag afterwards is all this verb can do, and it does it so that
     * a stale flag does not keep a vector alive.
     */
    static bool tamper_disarm(uint8_t index) {
        const uint32_t bit = tamper_enable_bit(index);
        if (bit == 0u) {
            return false;
        }
        RTC->TAFCR = RTC->TAFCR & ~bit;
        clear_flags(tamper_flag(index));
        return true;
    }

    /// TAMPIE - ONE interrupt enable for both inputs (there is no per
    /// input one on this family), on the tamper and timestamp vector.
    static void tamper_interrupt(bool on) {
        RTC->TAFCR = on ? (RTC->TAFCR | RTC_TAFCR_TAMPIE) : (RTC->TAFCR & ~RTC_TAFCR_TAMPIE);
        if (on) {
            wake_line_open(tamper_stamp_exti_line);
        }
    }
    static bool tamper_interrupt() { return (RTC->TAFCR & RTC_TAFCR_TAMPIE) != 0u; }

    /// TAMPTS - a tamper event fills the timestamp registers. "TAMPTS is
    /// valid even if TSE = 0", so this is the one timestamp source that
    /// needs no timestamp enable.
    static void timestamp_on_tamper(bool on) {
        RTC->TAFCR = on ? (RTC->TAFCR | RTC_TAFCR_TAMPTS) : (RTC->TAFCR & ~RTC_TAFCR_TAMPTS);
    }
    static bool timestamp_on_tamper() { return (RTC->TAFCR & RTC_TAFCR_TAMPTS) != 0u; }

    /// The flag of tamper input `index` (1-based), 0 for an index this
    /// part has not got.
    static constexpr uint32_t tamper_flag(uint8_t index) {
        if (index == 1u) {
            return RtcFlag::tamper1;
        }
        return (index == 2u) ? RtcFlag::tamper2 : 0u;
    }

    // ---- the backup registers (17.6.20) -------------------------------------
    //
    // Twenty 32-bit words in the backup domain: not reset by a system
    // reset, kept through Standby, powered from VBAT when VDD goes away -
    // and lost to a domain reset or a tamper detection. That makes them
    // this target's second kind of surviving storage, beside the .noinit
    // breadcrumb (a system reset only). They are outside the WPR key and
    // inside DBP (fact 1), so a write with the domain locked is dropped in
    // silence and the readback is how a caller learns that.

    /// Backup register `n`, 0 for an index this part has not got.
    static uint32_t backup(uint8_t n) {
        if (n >= backup_count) {
            return 0u;
        }
        return (&RTC->BKP0R)[n];
    }

    /// Write one. False for an index past the end - the only failure this
    /// verb can see.
    static bool backup(uint8_t n, uint32_t value) {
        if (n >= backup_count) {
            return false;
        }
        (&RTC->BKP0R)[n] = value;
        return true;
    }

    /// The same two by a CONSTEXPR index, refused at compile time: a
    /// program that names its own breadcrumb slots gets a build error
    /// rather than a zero and a dropped write on a part with fewer.
    template <uint8_t n>
    static uint32_t backup() {
        static_assert(n < backup_count, "brio Rtc: this part has no backup register n");
        return (&RTC->BKP0R)[n];
    }
    template <uint8_t n>
    static void backup(uint32_t value) {
        static_assert(n < backup_count, "brio Rtc: this part has no backup register n");
        (&RTC->BKP0R)[n] = value;
    }

    // ---- flags and the interrupts -------------------------------------------

    static bool flag(uint32_t mask) { return (RTC->ISR & mask) != 0u; }

    /**
     * Clear event flags. THE REGISTER IS rc_w0: writing a one to a flag
     * leaves it alone and writing a zero clears it, so the value written
     * is "every flag but these". INIT shares the register and is rw, so
     * it is carried over rather than zeroed; every other bit position is
     * read-only and takes no harm.
     *
     * No key: RTC_ISR[13:8] are outside the write protection (fact 1),
     * which is what lets an ISR body clear a flag without opening a
     * window. 17.6.4: the flags "are cleared 2 APB clock cycles after
     * programming them to 0".
     */
    static void clear_flags(uint32_t mask) {
        RTC->ISR = (RtcFlag::all & ~mask) | (RTC->ISR & RTC_ISR_INIT);
    }

    /// Which flags would raise a vector right now: the standing ones AND
    /// their enables. There is no masked-status register on this family,
    /// so the mask is built from RTC_CR's four interrupt enables and
    /// TAFCR's one.
    static uint32_t masked_status() { return RTC->ISR & enabled_mask(); }

    static uint32_t enabled_mask() {
        const uint32_t cr = RTC->CR;
        uint32_t m = 0;
        if ((cr & RTC_CR_ALRAIE) != 0u) {
            m |= RtcFlag::alarm_a;
        }
        if ((cr & RTC_CR_ALRBIE) != 0u) {
            m |= RtcFlag::alarm_b;
        }
        if ((cr & RTC_CR_WUTIE) != 0u) {
            m |= RtcFlag::wakeup;
        }
        if ((cr & RTC_CR_TSIE) != 0u) {
            m |= RtcFlag::timestamp | RtcFlag::timestamp_overflow;
        }
        if ((RTC->TAFCR & RTC_TAFCR_TAMPIE) != 0u) {
            m |= RtcFlag::tamper;
        }
        return m;
    }

    /// Open one of this peripheral's EXTI lines: the rising edge, then the
    /// interrupt mask (fact 7), through exti.hpp's verbs - the pending bit
    /// and its clear are Exti::pending(line) / Exti::clear(line), and a
    /// handler that does not clear it re-enters for ever.
    static void wake_line_open(uint8_t line) {
        (void)Exti::sense(line, ExtiSense::rising);
        (void)Exti::interrupt(line, true);
    }
    static bool wake_line_is_open(uint8_t line) {
        return Exti::interrupt(line) && Exti::sense(line) == ExtiSense::rising;
    }

    /**
     * The ISR body an app's RTC_Alarm_IRQHandler calls: both alarms, one
     * line, one vector. Returns the MASKED flags it served - so a handler
     * dispatches on what was really enabled, not on what merely happened -
     * and clears exactly those.
     */
    [[gnu::always_inline]] static uint32_t alarm_isr() {
        return serve(alarm_exti_line, RtcFlag::alarm_a | RtcFlag::alarm_b);
    }

    /// The ISR body for RTC_WKUP_IRQHandler: the periodic wake-up timer.
    [[gnu::always_inline]] static uint32_t wakeup_isr() {
        return serve(wakeup_exti_line, RtcFlag::wakeup);
    }

    /// The ISR body for TAMP_STAMP_IRQHandler: the timestamp, its
    /// overflow and the tamper inputs, which share line 21.
    [[gnu::always_inline]] static uint32_t tamper_stamp_isr() {
        return serve(tamper_stamp_exti_line,
                     RtcFlag::timestamp | RtcFlag::timestamp_overflow | RtcFlag::tamper);
    }

    // ---- debug ---------------------------------------------------------------

    /// DBGMCU_APB1_FZ.DBG_RTC_STOP: whether the calendar freezes when a
    /// debugger halts the core. Like the watchdogs' bits beside it, it is
    /// not reset by a system reset - whatever a debugger left there is
    /// what the next boot finds - and ES0287 2.8.5 says the sub-second
    /// alarm downcounter keeps running under a halt whatever this bit
    /// says.
    static bool debug_freeze() { return (DBGMCU->APB1FZ & DBGMCU_APB1_FZ_DBG_RTC_STOP) != 0u; }
    static void debug_freeze(bool on) {
        DBGMCU->APB1FZ = on ? (DBGMCU->APB1FZ | DBGMCU_APB1_FZ_DBG_RTC_STOP)
                            : (DBGMCU->APB1FZ & ~DBGMCU_APB1_FZ_DBG_RTC_STOP);
    }

private:
    /// RTC_ISR with RSF written as a zero and every event flag written as
    /// a one - clearing the synchronization flag without touching any
    /// other rc_w0 bit. RSF is bit 5, which the key DOES cover, so every
    /// caller of this holds the window open.
    static uint32_t clear_rsf_value() {
        return (RTC->ISR & ~RTC_ISR_RSF) | RtcFlag::all;
    }

    static constexpr uint32_t tamper2_enable_bit() {
        return rtc_pad_facts().tamper_inputs >= 2u ? RTC_TAFCR_TAMP2E : 0u;
    }
    static constexpr uint32_t tamper_enable_bit(uint8_t index) {
        if (index == 1u) {
            return RTC_TAFCR_TAMP1E;
        }
        return (index == 2u) ? tamper2_enable_bit() : 0u;
    }
    static constexpr uint32_t tamper_trigger_bit(uint8_t index) {
        if (index == 1u) {
            return RTC_TAFCR_TAMP1TRG;
        }
        return (index == 2u && rtc_pad_facts().tamper_inputs >= 2u) ? RTC_TAFCR_TAMP2TRG : 0u;
    }

    /**
     * One vector's worth of service: clear the EXTI's pending bit, then
     * serve the standing masked flags among `candidates` UNTIL NONE
     * STANDS.
     *
     * The loop is ES0287 2.8.3's workaround generalized. The erratum: a
     * second source on the same line whose flag rises after the handler
     * has checked it, but before the EXTI's pending bit is effectively
     * clear, never raises the line again and is lost for ever. The
     * erratum's own remedy is to check the sources a second time; this
     * checks them until they are quiet, which covers the two-source lines
     * (alarms A and B, tamper and timestamp) and costs one extra register
     * read on the one-source line.
     *
     * The inner spin is 17.6.4's other sentence - the flags clear two APB
     * cycles after the write - so that the next pass sees the truth
     * rather than the flag it just cleared. It is BOUNDED because ES0287
     * 2.8.5 can hold an alarm flag up under a debugger's halt, and a
     * handler that spun on that would never return.
     */
    [[gnu::always_inline]] static uint32_t serve(uint8_t line, uint32_t candidates) {
        (void)Exti::clear(line);
        uint32_t served = 0;
        for (uint8_t pass = 0; pass < 3u; ++pass) {
            const uint32_t standing = masked_status() & candidates;
            if (standing == 0u) {
                break;
            }
            clear_flags(standing);
            served |= standing;
            for (uint8_t i = 0; i < 8u && (RTC->ISR & standing) != 0u; ++i) {
            }
        }
        return served;
    }
};

} // namespace brio
