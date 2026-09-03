/*
 * rtc.hpp
 *
 * The real-time clock and the tamper/backup block (RM0444 ch. 30 and
 * ch. 31) - a calendar, two alarms, a periodic wake-up timer, a smooth
 * calibrator and five words of memory that outlive a reset, all of it
 * living in a POWER DOMAIN OF ITS OWN.
 *
 *  RtcDomain  the gate and the clock: PWR_CR1.DBP, RCC_APBENR1.RTCAPBEN
 *             and the whole of RCC_BDCR - LSE with its drive level,
 *             bypass and clock-security system, the RTCSEL multiplexer,
 *             RTCEN, the low-speed clock output, and BDRST, the one way
 *             back out of every one-way decision above.
 *  Rtc        the peripheral: the write-protection keys, initialization
 *             mode, the two prescalers, the BCD calendar, both alarms,
 *             the wake-up timer, smooth calibration, the timestamp, the
 *             flags and the ISR body.
 *  Tamp       ch. 31, and only the half this stratum can honestly own:
 *             the five backup registers, and the tamper configuration
 *             DECODED READ-ONLY.
 *
 * SEVEN FACTS OF THIS SILICON shape everything below.
 *
 * 1. THE DOMAIN IS LOCKED AT EVERY RESET, AND THERE ARE TWO LOCKS, not
 *    one. PWR_CR1.DBP (4.1.2) gates the whole domain - RCC_BDCR, every
 *    RTC register and every backup register - and is clear after a
 *    system reset. On top of it, most RTC registers carry a SECOND lock
 *    that only the key sequence 0xCA then 0x53 into RTC_WPR opens
 *    (30.3.8), and "writing a wrong key reactivates the write
 *    protection". The two are independent: DBP is the domain's, WPR is
 *    the peripheral's, and the WPR lock is "not affected by system
 *    reset" while DBP is cleared by one. Every configuring verb here
 *    brackets itself with unlock()/lock() so that a program cannot
 *    leave the peripheral open by forgetting; unlock() and lock() are
 *    public anyway, because a suite has to be able to prove the lock.
 *
 * 2. RTCSEL IS ONE-WAY, AND THE WAY BACK IS A DOMAIN RESET. 5.4.23:
 *    "Once the RTC clock source is selected, it cannot be changed
 *    anymore unless the RTC domain is reset". So `select()` returns
 *    false rather than pretending, and `RtcDomain::reset()` - BDRST -
 *    is a deliberate, separately spelled verb: it stops the RTC, wipes
 *    the calendar, the alarms, the prescalers, the backup registers and
 *    the clock choice. THIS BOARD NEEDS IT. Measured on the bench and
 *    recorded in docs/stm32g0/reset.md long before this file existed:
 *    RCC_BDCR comes up reading 0x8200 - RTCEN set, RTCSEL = LSI - after
 *    EVERY system reset, because the domain is not in the system reset's
 *    scope, so a program that wants LSE must reset the domain first.
 *    reset() follows ES0548 2.2.11's own spelling (write 0x0001_0000,
 *    READ IT BACK to make the pulse long enough, write 0).
 *
 * 3. THE RTC KEEPS RUNNING UNDER SYSTEM RESET, and that is the point of
 *    it. 5.2.12 and 30.3.10: with LSE or LSI as RTCCLK the counter is
 *    not reset by anything but a domain reset - what a system reset does
 *    clear is the SHADOW registers and three bits of RTC_ICSR. Hence
 *    INITS, the flag that says whether the calendar was ever set, and
 *    hence the backup registers as a legitimate place to leave a note.
 *
 * 4. THE CALENDAR IS READ THROUGH SHADOWS, OR NOT AT ALL. With
 *    BYPSHAD = 0 the three readable registers are copies refreshed every
 *    two RTCCLK cycles, RSF says a copy has landed, and reading SSR or
 *    TR LOCKS the higher-order ones until DR is read (30.3.9) - so a
 *    coherent reading is always three reads ending in DR. With
 *    BYPSHAD = 1 there is no copy and no waiting, which is what a
 *    low-power program wants, at the price stated by the chapter: the
 *    values may disagree if an RTCCLK edge lands between two reads, so
 *    "the software must read all the registers twice". read() below does
 *    exactly that, per mode, and is the only calendar reader this file
 *    offers.
 *    AND THE SHADOW IS NOT UPDATED IN STOP OR STANDBY: after such a
 *    sleep RSF must be cleared and awaited again before a shadow read
 *    means anything (30.3.5, 30.3.9). BYPSHAD = 1 deletes that problem,
 *    which is why the timed sleep site (stm32g0/sleep.hpp) sets it.
 *
 * 5. INITIALIZATION MODE IS A STOPPED CALENDAR, AND ENTERING IT TWICE IN
 *    A ROW IS AN ERRATUM. ES0548 2.9.1 (LIVE on both silicon revisions):
 *    INIT set between one and two RTCCLK cycles after being cleared sets
 *    INITF immediately instead of waiting for the synchronization, and
 *    "a write occurring during this critical period might result in the
 *    corruption of one or more calendar registers". The workaround is
 *    the chapter's own reading discipline, and this file applies it
 *    UNCONDITIONALLY on the way OUT: exit_init() clears INIT, then
 *    clears BYPSHAD if it was set, waits for RSF to rise, and restores
 *    BYPSHAD - so a second enter_init() is safe by construction and no
 *    caller has to remember. The cost is one RTCCLK period (about 30 us
 *    at 32 kHz) per exit, paid once per configuration.
 *
 * 6. EVERY ARMED THING HAS A WRITE WINDOW, AND THE WINDOW IS A FLAG.
 *    The wake-up timer's reload and clock select may be written only
 *    with WUTE clear and WUTWF set; alarm A's registers only with ALRAE
 *    clear and ALRAWF set; and both flags take "around 2 RTCCLK clock
 *    cycles" to appear. Every configuring verb below therefore disables,
 *    waits for its own flag with a BOUND, writes, and re-enables - and
 *    returns false if the flag never came, which on a stopped RTCCLK is
 *    what happens.
 *
 * 7. THE INTERRUPT IS ONE VECTOR AND THE EXTI IS ALREADY OPEN. Table 61
 *    gives RTC and TAMP one line (RTC_TAMP_IRQn), and table 65 makes
 *    EXTI 19 (RTC) and 21 (TAMP) DIRECT lines: no trigger selection, no
 *    pending bit, nothing to clear in the EXTI, and EXTI_IMR1 comes out
 *    of reset at 0xFFF8_0000 with both of them already unmasked
 *    (13.5.12). So 4.3.10's instruction to "configure the EXTI Line 19
 *    to be sensitive to rising edge" describes a register bit this
 *    device has not got; what a wake really needs is the RTC's own
 *    interrupt enable, the NVIC line, and the IMR bit that is already
 *    there. `wake_line_open()` is the one-bit assertion of that, and it
 *    is idempotent.
 *
 * WHY RCC_BDCR LIVES HERE AND NOT IN clock.hpp. On the SAM the RTC's
 * clock select sits in the oscillator block and samc/rtc.hpp never
 * touches it - one register, one owner. The same rule applies here and
 * lands the other way round, for three reasons: RCC_BDCR is unreachable
 * without PWR_CR1.DBP, so its access discipline is the RTC DOMAIN's and
 * not the clock tree's; RTCSEL is one-way, and a clock verb whose
 * consequence can only be undone by wiping a calendar belongs beside the
 * calendar; and the register also carries RTCEN, LSE and BDRST, which
 * are this chapter's subject matter and not chapter 5's. clock.hpp's
 * `Rcc` therefore never names BDCR, and `RtcDomain` owns all of it. The
 * day ClockSource::lse is built, the clock task will ASK this type for a
 * running LSE rather than start one behind its back.
 *
 * ERRATA, ES0548 Rev 3 on the bench chip's revision Z column:
 *  - 2.9.1 (calendar init on consecutive INIT entries) is LIVE and is
 *    coded, see fact 5;
 *  - 2.2.1 (unstable LSI when it clocks the RTC) is LIVE, has no fix in
 *    this file, and is a REASON TO PREFER LSE where a board has the
 *    crystal: its own workaround is to reset the RTC domain on every VDD
 *    power-up, which reset() spells and which docs/stm32g0/rtc.md tells
 *    an application to do from PWRRSTF;
 *  - 2.2.11 (a missed domain reset after a supply dip) is LIVE, and its
 *    workaround is the same reset() from the same flag;
 *  - 2.2.6 (PC13 transitions disturb LSE) is LIVE and unfixable in
 *    software: PC13 is the RTC_OUT1/TAMP_IN1 pad, so a program using LSE
 *    should leave it alone. Stated, not coded - there is nothing to
 *    code.
 *
 * NOT BUILT (docs/stm32g0/rtc.md carries the list): the tamper
 * DETECTION half of ch. 31 (the pads, the filters, the active tampers,
 * the erase-on-tamper of the backup registers) - decoded read-only here
 * because arming a tamper wipes the very registers this stratum uses as
 * a breadcrumb, and because nothing on the bench can drive TAMP_IN;
 * RTC_REFIN reference-clock detection (a mains input this board has
 * not got); the RTC_OUT1/OUT2 output pads and the calibration output
 * (they want PC13, which erratum 2.2.6 makes hostile to LSE); and
 * RTCSEL = HSE/32, which needs an HSE root this stratum does not build.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/device_tables.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"

namespace brio {

// =============================================================================
// The RTC domain: the gate, the clock and the one-way choices (5.4.23)
// =============================================================================

/// RCC_BDCR.RTCSEL: what RTCCLK is. `none` is the reset value and means
/// the RTC has no clock at all - not an error, just a peripheral that
/// will never tick.
enum class RtcClockSource : uint8_t {
    none = 0,
    lse = 1,
    lsi = 2,
    hse_div32 = 3,
};

/// RCC_BDCR.LSEDRV: how hard the oscillator drives the crystal (5.2.5).
/// It may be LOWERED under a running LSE and never raised, which is the
/// asymmetry `lse_drive()` refuses on.
enum class LseDrive : uint8_t {
    low = 0,
    medium_low = 1,
    medium_high = 2,
    high = 3,
};

/**
 * The RTC domain's gate and clock tree - the whole of RCC_BDCR, plus the
 * two enable bits outside it that a program must open first.
 *
 * ORDER, AND IT IS 4.1.2's: open the PWR bus clock, set DBP, choose
 * RTCSEL, set RTCEN. `open()` is those four in one call; the individual
 * verbs exist because a boot that has to reset the domain first does
 * them in a different order.
 */
struct RtcDomain {
    RtcDomain() = delete;

    /// A bound for every ready/reset wait in this file. At 64 MHz this
    /// is a few tens of milliseconds - long enough for an LSE crystal's
    /// start-up (hundreds of milliseconds is the datasheet's worst case,
    /// so a caller wanting to wait that long calls lse_wait_ready() in a
    /// loop of its own) and short enough that a dead oscillator is
    /// reported instead of hanging the program.
    static constexpr uint32_t ready_spins = 2'000'000UL;

    // ---- the two gates outside BDCR -----------------------------------------

    /// RCC_APBENR1.PWREN - the PWR block's bus clock. Clear at reset,
    /// and a peripheral without its bus clock does not answer register
    /// reads (5.2.17), so this comes first. Left open afterwards: the
    /// sleep site wants it anyway.
    static void pwr_bus_clock(bool on) {
        RCC->APBENR1 = on ? (RCC->APBENR1 | RCC_APBENR1_PWREN)
                          : (RCC->APBENR1 & ~RCC_APBENR1_PWREN);
        (void)RCC->APBENR1;
    }
    static bool pwr_bus_clock() {
        return (RCC->APBENR1 & RCC_APBENR1_PWREN) != 0u;
    }

    /// RCC_APBENR1.RTCAPBEN - the APB INTERFACE to the RTC and TAMP
    /// register banks. Distinct from RTCEN, which is the RTC's own
    /// clock: the counter can run with this bit clear (that is the whole
    /// point of a domain that survives), but nothing can read it.
    static void apb_clock(bool on) {
        RCC->APBENR1 = on ? (RCC->APBENR1 | RCC_APBENR1_RTCAPBEN)
                          : (RCC->APBENR1 & ~RCC_APBENR1_RTCAPBEN);
        (void)RCC->APBENR1;
    }
    static bool apb_clock() {
        return (RCC->APBENR1 & RCC_APBENR1_RTCAPBEN) != 0u;
    }

    /// PWR_CR1.DBP - "disable RTC domain write protection" (4.4.1).
    /// Needs pwr_bus_clock() first.
    static void unlock(bool on) {
        PWR->CR1 = on ? (PWR->CR1 | PWR_CR1_DBP) : (PWR->CR1 & ~PWR_CR1_DBP);
    }
    static bool unlocked() { return (PWR->CR1 & PWR_CR1_DBP) != 0u; }

    // ---- RCC_BDCR -----------------------------------------------------------

    static uint32_t bdcr() { return RCC->BDCR; }

    /**
     * Reset the whole RTC domain - BDRST, the way back from every
     * one-way bit in this register.
     *
     * WHAT IT COSTS: the calendar, the prescalers, both alarms, the
     * wake-up timer, the calibration, the timestamp, the FIVE BACKUP
     * REGISTERS and the clock choice. Everything in the domain except
     * LSCOSEL, LSCOEN and BDRST itself, which only a domain POWER-ON
     * reset clears (5.4.23).
     *
     * The sequence is ES0548 2.2.11's own: write the whole register with
     * BDRST alone (which also clears the bits "that might not be
     * reset"), READ IT BACK so the pulse is long enough, then write
     * zero. Needs DBP.
     */
    static void reset() {
        RCC->BDCR = RCC_BDCR_BDRST;
        (void)RCC->BDCR;
        RCC->BDCR = 0u;
        (void)RCC->BDCR;
    }

    // ---- LSE ----------------------------------------------------------------

    static void lse_enable(bool on) {
        RCC->BDCR = on ? (RCC->BDCR | RCC_BDCR_LSEON)
                       : (RCC->BDCR & ~RCC_BDCR_LSEON);
    }
    static bool lse_enabled() { return (RCC->BDCR & RCC_BDCR_LSEON) != 0u; }
    static bool lse_ready() { return (RCC->BDCR & RCC_BDCR_LSERDY) != 0u; }

    /// Bounded wait for LSERDY. False = the crystal did not start in
    /// `spins` turns, which is a FACT about the board (no crystal
    /// fitted, or a bad one) and not an error to be swallowed.
    static bool lse_wait_ready(uint32_t spins = ready_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            if (lse_ready()) {
                return true;
            }
        }
        return false;
    }

    /**
     * LSEDRV. 5.2.5: the drive "can be decreased to the lower drive
     * capability when the LSE is ON", but "once LSEDRV is selected, the
     * drive capability can not be increased if LSEON=1" - so an increase
     * under a running oscillator is REFUSED here rather than written
     * into a register that ignores it.
     */
    static bool lse_drive(LseDrive d) {
        const uint8_t want = static_cast<uint8_t>(d);
        if (lse_enabled() && want > static_cast<uint8_t>(lse_drive())) {
            return false;
        }
        RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_LSEDRV_Msk) |
                    ((static_cast<uint32_t>(want) << RCC_BDCR_LSEDRV_Pos) &
                     RCC_BDCR_LSEDRV_Msk);
        return true;
    }
    static LseDrive lse_drive() {
        return static_cast<LseDrive>(
            (RCC->BDCR & RCC_BDCR_LSEDRV_Msk) >> RCC_BDCR_LSEDRV_Pos);
    }

    /// LSEBYP - an external square wave on OSC32_IN instead of a
    /// crystal. "This bit can be written only when the external 32 kHz
    /// oscillator is disabled (LSEON=0 and LSERDY=0)" (5.4.23), which is
    /// the refusal below.
    static bool lse_bypass(bool on) {
        if (lse_enabled() || lse_ready()) {
            return false;
        }
        RCC->BDCR = on ? (RCC->BDCR | RCC_BDCR_LSEBYP)
                       : (RCC->BDCR & ~RCC_BDCR_LSEBYP);
        return true;
    }
    static bool lse_bypass() { return (RCC->BDCR & RCC_BDCR_LSEBYP) != 0u; }

    /**
     * The clock security system on LSE. ONE-WAY: 5.4.23 says it "cannot
     * be disabled, except after a LSE failure detection", and its
     * detection flag holds the oscillator under reset until a domain
     * reset. Its own preconditions are the chapter's and are enforced:
     * LSE on, LSE ready, RTCSEL already chosen.
     */
    static bool lse_css(bool on) {
        if (!on) {
            if (!lse_css_failed()) {
                return false;   // 5.4.23: only clearable after a failure
            }
            RCC->BDCR = RCC->BDCR & ~RCC_BDCR_LSECSSON;
            return true;
        }
        if (!lse_enabled() || !lse_ready() || selected() == RtcClockSource::none) {
            return false;
        }
        RCC->BDCR = RCC->BDCR | RCC_BDCR_LSECSSON;
        return true;
    }
    static bool lse_css() { return (RCC->BDCR & RCC_BDCR_LSECSSON) != 0u; }
    static bool lse_css_failed() {
        return (RCC->BDCR & RCC_BDCR_LSECSSD) != 0u;
    }

    /// The EXTI line the LSE clock-security system raises (table 65:
    /// line 31, DIRECT). Published here per the stratum's rule that a
    /// peripheral owns its own line number.
    static constexpr uint8_t css_exti_line = 31;

    /// TIM16_TISEL's code for LSE on TI1 (25.6.18). Published here for
    /// the same reason: the timer driver owns the multiplexer and this
    /// block owns the signal, so an application measuring the crystal
    /// writes `Tim<16>::input_select(0, RtcDomain::lse_tim16_ti1_code)`
    /// and neither driver has to know the other's table.
    static constexpr uint8_t lse_tim16_ti1_code = 2;

    // ---- RTCSEL and RTCEN ---------------------------------------------------

    static RtcClockSource selected() {
        return static_cast<RtcClockSource>(
            (RCC->BDCR & RCC_BDCR_RTCSEL_Msk) >> RCC_BDCR_RTCSEL_Pos);
    }

    /**
     * Choose RTCCLK. ONE-WAY (5.4.23): false when a DIFFERENT source is
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
                    ((static_cast<uint32_t>(s) << RCC_BDCR_RTCSEL_Pos) &
                     RCC_BDCR_RTCSEL_Msk);
        return selected() == s;
    }

    static void enable(bool on) {
        RCC->BDCR = on ? (RCC->BDCR | RCC_BDCR_RTCEN)
                       : (RCC->BDCR & ~RCC_BDCR_RTCEN);
    }
    static bool enabled() { return (RCC->BDCR & RCC_BDCR_RTCEN) != 0u; }

    // ---- the low-speed clock output (LSCO) ----------------------------------
    //
    // 5.4.23's two survivors of a domain reset. The pad is PA2 on this
    // family - the console's TX on this board - so this is offered and
    // never used by anything here.

    static void lsco(bool on, bool from_lse = false) {
        uint32_t v = RCC->BDCR & ~(RCC_BDCR_LSCOEN | RCC_BDCR_LSCOSEL);
        if (on) {
            v |= RCC_BDCR_LSCOEN;
        }
        if (from_lse) {
            v |= RCC_BDCR_LSCOSEL;
        }
        RCC->BDCR = v;
    }
    static bool lsco() { return (RCC->BDCR & RCC_BDCR_LSCOEN) != 0u; }

    // ---- the composed boot verb ---------------------------------------------

    /**
     * The whole of 4.1.2's sequence: PWR bus clock, DBP, the RTC's APB
     * interface, RTCSEL, RTCEN. False when the source could not be
     * chosen because a different one already stands - `wipe` is how a
     * caller says "and reset the domain first if you must", which is
     * what a program wanting LSE has to do on a board whose domain came
     * up on LSI (fact 2).
     *
     * Does NOT start LSE: which oscillator to run and how long to wait
     * for it is the application's, and lse_enable()/lse_wait_ready() are
     * right there.
     */
    static bool open(RtcClockSource s, bool wipe = false) {
        pwr_bus_clock(true);
        unlock(true);
        if (!unlocked()) {
            return false;
        }
        apb_clock(true);
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
// The prescalers (30.3.4, 30.6.5)
// =============================================================================

/// The two register FIELDS, not the divisors: PREDIV_A and PREDIV_S,
/// each one less than the ratio it makes.
struct RtcPrescalers {
    uint8_t async = 127;    ///< PREDIV_A, 7 bits: ck_apre = RTCCLK/(async+1)
    uint16_t sync = 255;    ///< PREDIV_S, 15 bits: ck_spre = ck_apre/(sync+1)
};

constexpr bool rtc_prescalers_valid(const RtcPrescalers& p) {
    return p.async <= 0x7Fu && p.sync <= 0x7FFFu;
}

/// ck_spre in hertz for a stated RTCCLK - the calendar's own tick, and
/// also the wake-up timer's slowest clock. The RATE IS THE CALLER'S
/// ARGUMENT and not a constant of this file, exactly as the IWDG's
/// nominal time-out takes an LSI rate: LSE is 32768 Hz by construction
/// and LSI is an uncalibrated RC, and a driver that pretended to know
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

/// The EXACT pair that makes ck_spre 1 Hz from `rtcclk_hz`, with the
/// asynchronous factor as HIGH as possible (30.3.4's own recommendation:
/// "configure the asynchronous prescaler to a high value to minimize
/// consumption"). `async` comes back 0xFF - a value the 7-bit field
/// cannot hold - when no exact pair exists, the hsidiv_for() convention.
///
/// 32768 gives 127/255, the reset value and the chapter's example.
constexpr RtcPrescalers rtc_prescalers_for(uint32_t rtcclk_hz) {
    for (uint32_t a = 128; a >= 1; --a) {
        if (rtcclk_hz % a != 0u) {
            continue;
        }
        const uint32_t s = rtcclk_hz / a;
        if (s >= 1u && s <= 32768u) {
            return RtcPrescalers{static_cast<uint8_t>(a - 1u),
                                 static_cast<uint16_t>(s - 1u)};
        }
    }
    return RtcPrescalers{0xFFu, 0u};
}

/// The exact 1 Hz pair with the SMALLEST asynchronous factor - i.e. the
/// LARGEST PREDIV_S, which is what a caller who wants the sub-second
/// counter as a STOPWATCH asks for.
///
/// It is the deliberate opposite of rtc_prescalers_for()'s choice.
/// 30.3.4 recommends a high asynchronous factor "to minimize
/// consumption", and that is the right default for a calendar; but
/// PREDIV_S is the RESOLUTION of every sub-second reading and every
/// sub-second alarm (30.3.11 says so in as many words: "the resolution
/// can be improved by increasing the synchronous prescaler value"), so
/// a timebase that has to measure a frozen span to better than a
/// millisecond wants this one and pays the ck_apre current for it.
constexpr RtcPrescalers rtc_prescalers_for_resolution(uint32_t rtcclk_hz) {
    for (uint32_t a = 1; a <= 128u; ++a) {
        if (rtcclk_hz % a != 0u) {
            continue;
        }
        const uint32_t s = rtcclk_hz / a;
        if (s >= 1u && s <= 32768u) {
            return RtcPrescalers{static_cast<uint8_t>(a - 1u),
                                 static_cast<uint16_t>(s - 1u)};
        }
    }
    return RtcPrescalers{0xFFu, 0u};
}

/// The fraction of a second a sub-second reading means, in
/// MILLISECONDS: 30.6.3's own formula (PREDIV_S - SS) / (PREDIV_S + 1),
/// scaled. SS can exceed PREDIV_S after a shift operation, which the
/// chapter says means "one second less"; here that clamps to zero and
/// the caller is told by read()'s own coherence check, not by a silent
/// wrap.
constexpr uint16_t rtc_subsecond_ms(uint16_t ss, uint16_t prediv_s) {
    if (ss > prediv_s) {
        return 0;
    }
    return static_cast<uint16_t>((static_cast<uint32_t>(prediv_s - ss) * 1000UL) /
                                 (static_cast<uint32_t>(prediv_s) + 1UL));
}

// =============================================================================
// The calendar (30.3.5, 30.6.1, 30.6.2)
// =============================================================================

/// A whole calendar reading or setting, in ORDINARY NUMBERS. The
/// registers are BCD and this struct is not: the conversion is
/// constexpr, checked both ways by the family fixture, and is the only
/// place a nibble is shifted.
struct RtcDateTime {
    uint8_t hour = 0;      ///< 0..23 (24-hour format; see FMT)
    uint8_t minute = 0;    ///< 0..59
    uint8_t second = 0;    ///< 0..59
    uint8_t day = 1;       ///< 1..31
    uint8_t month = 1;     ///< 1..12
    uint8_t year = 0;      ///< 0..99, the century being the caller's business
    uint8_t weekday = 1;   ///< 1 = Monday .. 7 = Sunday; 0 is "forbidden" (30.6.2)
};

constexpr uint8_t rtc_to_bcd(uint8_t v) {
    return static_cast<uint8_t>(((v / 10u) << 4) | (v % 10u));
}
constexpr uint8_t rtc_from_bcd(uint8_t v) {
    return static_cast<uint8_t>(((v >> 4) & 0x0Fu) * 10u + (v & 0x0Fu));
}

/// Days in a month for a two-digit year. THE LEAP RULE IS THE
/// SILICON'S: this calendar spans one century, so "divisible by four" is
/// the whole of it - there is no year 00 exception to apply, the field
/// having no century in it.
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
    d.hour = rtc_from_bcd(static_cast<uint8_t>((tr & (RTC_TR_HT_Msk | RTC_TR_HU_Msk)) >>
                                               RTC_TR_HU_Pos));
    d.minute = rtc_from_bcd(static_cast<uint8_t>(
        (tr & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >> RTC_TR_MNU_Pos));
    d.second = rtc_from_bcd(static_cast<uint8_t>(
        (tr & (RTC_TR_ST_Msk | RTC_TR_SU_Msk)) >> RTC_TR_SU_Pos));
    d.year = rtc_from_bcd(static_cast<uint8_t>(
        (dr & (RTC_DR_YT_Msk | RTC_DR_YU_Msk)) >> RTC_DR_YU_Pos));
    d.weekday = static_cast<uint8_t>((dr & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos);
    d.month = rtc_from_bcd(static_cast<uint8_t>(
        (dr & (RTC_DR_MT_Msk | RTC_DR_MU_Msk)) >> RTC_DR_MU_Pos));
    d.day = rtc_from_bcd(static_cast<uint8_t>(
        (dr & (RTC_DR_DT_Msk | RTC_DR_DU_Msk)) >> RTC_DR_DU_Pos));
    return d;
}

/// One coherent look at the calendar: the three registers as one value.
struct RtcReading {
    RtcDateTime time{};
    uint16_t subsecond = 0;   ///< RTC_SSR, counting DOWN from PREDIV_S
};

// =============================================================================
// The alarms (30.3.6, 30.6.14..30.6.17)
// =============================================================================

enum class RtcAlarmId : uint8_t { a = 0, b = 1 };

/**
 * One alarm. Every field is compared unless its mask says otherwise, so
 * the DEFAULT here - everything masked but the sub-second field, which
 * is masked by its own separate rule - is "every second", and a caller
 * builds up from there.
 *
 * `subsecond_mask` is MASKSS as the chapter spells it (30.6.15): 0 means
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
             ? (RTC_ALRMAR_WDSEL |
                (static_cast<uint32_t>(a.day) << RTC_ALRMAR_DU_Pos))
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
// The wake-up timer (30.3.7, 30.6.6)
// =============================================================================

/// WUCKSEL. The four divided-RTCCLK codes give a short, fine timer; the
/// two ck_spre codes give a long, one-second one, the second of them
/// adding 2^16 to the reload (30.6.6), which is how the range reaches
/// 36 hours.
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

/// 30.6.6: WUTF is set every (WUT + 1) ck_wut cycles, and "setting
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
// Smooth calibration (30.3.13, 30.6.9)
// =============================================================================

/// The calibration window, in seconds. The chapter's own three, and the
/// two shorter ones cost resolution: CALM[1:0] are stuck at 00 at 8 s
/// and CALM[0] at 16 s (30.6.9).
enum class RtcCalibrationWindow : uint8_t {
    seconds32 = 0,
    seconds16 = 1,
    seconds8 = 2,
};

/**
 * A calibration setting. CALM masks out pulses (slowing the calendar,
 * 0.9537 ppm per step) and CALP inserts one every 2^11 (+488.5 ppm), so
 * the two together reach roughly -488 .. +488 ppm.
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
    // 30.6.9's two stuck-bit notes, turned into refusals: a value the
    // hardware would quietly round is a value the caller did not ask for.
    if (c.window == RtcCalibrationWindow::seconds8 && (c.minus & 0x3u) != 0u) {
        return false;
    }
    if (c.window == RtcCalibrationWindow::seconds16 && (c.minus & 0x1u) != 0u) {
        return false;
    }
    return true;
}

/// The correction a setting applies, in parts per billion, signed:
/// (512 x CALP - CALM) pulses in 2^20, scaled. Positive speeds the
/// calendar up.
constexpr int32_t rtc_calibration_ppb(const RtcCalibration& c) {
    const int32_t pulses = (c.plus ? 512 : 0) - static_cast<int32_t>(c.minus);
    return static_cast<int32_t>((static_cast<int64_t>(pulses) * 1'000'000'000LL) /
                                (1LL << 20));
}

// =============================================================================
// Flags (30.6.18..30.6.20)
// =============================================================================

/// One bit per event, the shape RTC_SR, RTC_MISR and RTC_SCR all share.
struct RtcFlag {
    static constexpr uint32_t alarm_a = RTC_SR_ALRAF;
    static constexpr uint32_t alarm_b = RTC_SR_ALRBF;
    static constexpr uint32_t wakeup = RTC_SR_WUTF;
    static constexpr uint32_t timestamp = RTC_SR_TSF;
    static constexpr uint32_t timestamp_overflow = RTC_SR_TSOVF;
    static constexpr uint32_t internal_timestamp = RTC_SR_ITSF;
    static constexpr uint32_t all = alarm_a | alarm_b | wakeup | timestamp |
                                    timestamp_overflow | internal_timestamp;
};

// =============================================================================
// The peripheral
// =============================================================================

/**
 * The RTC.
 *
 * MONOSTATE, like every other single-instance driver in this stratum
 * (there is one RTC on every part of the family, and one domain for it
 * to live in).
 *
 * WHAT IT DOES NOT DO: it never writes RCC_BDCR - that is RtcDomain's
 * register, whole - and it never enables the NVIC line for you. It does
 * assert the EXTI mask bit (fact 7), because that is one idempotent bit
 * without which no interrupt of this peripheral ever leaves the block,
 * and because leaving it to the application would be leaving a trap.
 */
struct Rtc {
    Rtc() = delete;

    static constexpr uint32_t key_unlock1 = 0xCAu;
    static constexpr uint32_t key_unlock2 = 0x53u;
    static constexpr uint32_t key_lock = 0xFFu;   ///< any wrong key re-locks

    /// A bound on every synchronization wait. Sized for the slowest
    /// thing here - "around 2 RTCCLK cycles" at 32 kHz is 61 us, and
    /// exit_init()'s RSF wait is a whole RTCCLK period - with room to
    /// spare, and small enough that a STOPPED RTCCLK is reported as
    /// false in a few milliseconds rather than hanging.
    static constexpr uint32_t sync_spins = 1'000'000UL;

    static constexpr IRQn_Type irq() { return rtc_irq(); }
    static constexpr uint8_t exti_line = rtc_exti_line;

    /// TIM16_TISEL's code for "RTC wake-up" on TI1 (25.6.18) - the
    /// periodic wake-up signal as a capture source, which is how this
    /// stratum measures the RTC against the core clock with no wire.
    /// 25.6.18's own footnote is the obligation: the source "requires to
    /// enable the RTC interrupt", i.e. WUTIE, which set_wakeup() does by
    /// default.
    static constexpr uint8_t wakeup_tim16_ti1_code = 3;

    // ---- write protection (30.3.8) -----------------------------------------

    static void unlock() {
        RTC->WPR = key_unlock1;
        RTC->WPR = key_unlock2;
    }
    static void lock() { RTC->WPR = key_lock; }

    // ---- raw register readbacks --------------------------------------------

    static uint32_t cr() { return RTC->CR; }
    static uint32_t icsr() { return RTC->ICSR; }
    static uint32_t prer() { return RTC->PRER; }
    static uint32_t wutr() { return RTC->WUTR; }
    static uint32_t calr() { return RTC->CALR; }
    static uint32_t status() { return RTC->SR; }
    static uint32_t masked_status() { return RTC->MISR; }

    /// INITS - "the calendar has been initialized", i.e. the year field
    /// is not the domain reset's 0x00 (30.6.4). The only thing that
    /// tells a fresh boot whether the RTC it inherited means anything.
    static bool calendar_set() { return (RTC->ICSR & RTC_ICSR_INITS) != 0u; }
    static bool in_init() { return (RTC->ICSR & RTC_ICSR_INITF) != 0u; }
    static bool synchronized() { return (RTC->ICSR & RTC_ICSR_RSF) != 0u; }
    static bool shift_pending() { return (RTC->ICSR & RTC_ICSR_SHPF) != 0u; }
    static bool recalibration_pending() {
        return (RTC->ICSR & RTC_ICSR_RECALPF) != 0u;
    }

    static bool bypass_shadow() { return (RTC->CR & RTC_CR_BYPSHAD) != 0u; }

    // ---- initialization mode (30.3.8, ES0548 2.9.1) -------------------------

    /**
     * Enter initialization mode: the calendar counter stops and TR, DR
     * and PRER become writable. False when INITF never rose, which on a
     * stopped RTCCLK is always. Brackets its own write protection.
     */
    static bool enter_init() {
        unlock();
        RTC->ICSR = RTC->ICSR | RTC_ICSR_INIT;
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
     * Leave initialization mode, ES0548 2.9.1's WORKAROUND INCLUDED.
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
     * False = RSF never rose within the bound, which means the RTC is
     * not counting and the calendar written just now is not in force.
     */
    static bool exit_init() {
        unlock();
        RTC->ICSR = RTC->ICSR & ~RTC_ICSR_INIT;
        const bool bypass = bypass_shadow();
        if (bypass) {
            RTC->CR = RTC->CR & ~RTC_CR_BYPSHAD;
        }
        RTC->ICSR = RTC->ICSR & ~RTC_ICSR_RSF;
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
     * It exists for two reasons and neither is convenience: ES0548
     * 2.9.1 cannot be STAGED at a bench without a path that does not
     * dodge it (and an app may not poke a register itself), and a caller
     * that is about to do the RSF wait for its own reasons should not
     * pay for it twice. Anything else should call exit_init().
     */
    static void exit_init_raw() {
        unlock();
        RTC->ICSR = RTC->ICSR & ~RTC_ICSR_INIT;
        lock();
    }

    /// Clear RSF and wait for the next shadow copy - what 30.3.9 asks
    /// for after a system reset, an initialization, a shift, and AFTER
    /// EVERY STOP OR STANDBY. A no-op that answers true when the shadow
    /// registers are bypassed, there being nothing to synchronize.
    static bool wait_sync() {
        if (bypass_shadow()) {
            return true;
        }
        unlock();
        RTC->ICSR = RTC->ICSR & ~RTC_ICSR_RSF;
        lock();
        for (uint32_t i = 0; i < sync_spins; ++i) {
            if (synchronized()) {
                return true;
            }
        }
        return false;
    }

    // ---- configuration ------------------------------------------------------

    /// Write both prescalers. 30.6.5: "the initialization must be
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

    static RtcPrescalers prescalers() {
        const uint32_t v = RTC->PRER;
        return RtcPrescalers{
            static_cast<uint8_t>((v & RTC_PRER_PREDIV_A_Msk) >> RTC_PRER_PREDIV_A_Pos),
            static_cast<uint16_t>((v & RTC_PRER_PREDIV_S_Msk) >> RTC_PRER_PREDIV_S_Pos)};
    }

    /// BYPSHAD (fact 4). Writable outside initialization mode - the
    /// chapter's note about bits 6 and 4 covers FMT and REFCKON, not
    /// this one.
    static void bypass_shadow(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_BYPSHAD) : (RTC->CR & ~RTC_CR_BYPSHAD);
        lock();
    }

    // ---- reference clock detection (30.3.12) --------------------------------
    //
    // REFCKON does NOT clock the calendar - the LSE still does. What it
    // buys is a RELOAD of the asynchronous prescaler whenever an edge of
    // RTC_REFIN is seen inside the detection window around a calendar
    // update, which drags the 1 Hz edge onto the reference's and makes
    // the calendar as accurate as the reference instead of as accurate
    // as the crystal. The correction is therefore quantized at ONE
    // ck_apre period per second - the prescaler is reloaded, not
    // trimmed - and the window is 7 ck_apre periods hunting for the
    // first edge and 3 once locked.
    //
    // TWO RULES, and both are refusals here rather than notes:
    //  - REFCKON is bit 4 of RTC_CR, which 30.6.3's own note says may be
    //    written in INITIALIZATION MODE ONLY;
    //  - the prescalers must be the default pair (PREDIV_A = 0x7F,
    //    PREDIV_S = 0xFF), because the detection is built on a 256 Hz
    //    ck_apre. Enabling it under any other pair is 30.3.12's own
    //    "must", and a driver that let it through would be shipping a
    //    calendar that is silently wrong.
    //
    // And one INTERLOCK the chapter states from the other side: a shift
    // operation must not be issued while this is on (30.3.11's caution),
    // which shift() below enforces.

    static bool reference_clock() { return (RTC->CR & RTC_CR_REFCKON) != 0u; }

    static bool reference_clock(bool on) {
        if (!in_init()) {
            return false;
        }
        if (on) {
            const RtcPrescalers p = prescalers();
            if (p.async != 0x7Fu || p.sync != 0xFFu) {
                return false;
            }
        }
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_REFCKON) : (RTC->CR & ~RTC_CR_REFCKON);
        lock();
        return true;
    }

    /**
     * Write the calendar - TR and DR - and pin the 24-hour format.
     * Legal ONLY in initialization mode (30.6.1 and 30.6.2 both say
     * "must be written in initialization mode only"), which is the
     * refusal; init() below is this verb plus the mode and the
     * prescalers, and a caller doing its own initialization sequence
     * (staging ES0548 2.9.1, for one) uses this.
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
     * The whole boot sequence of 30.3.8: enter initialization mode, set
     * the prescalers, set the calendar, leave - with the erratum
     * workaround on the way out. `time` is optional in the sense that a
     * caller wanting to keep a calendar that survived a reset passes the
     * one it just read; there is no way to set the prescalers without
     * stopping the counter, so there is no verb that pretends otherwise.
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

    // ---- reading the calendar (30.3.9) --------------------------------------

    /**
     * ONE COHERENT LOOK at sub-second, time and date.
     *
     * Two modes, two disciplines, both the chapter's:
     *  - shadow (BYPSHAD = 0): read SSR, then TR, then DR. The first read
     *    LOCKS the higher-order shadows and the DR read releases them, so
     *    the three belong to one instant by construction. RSF is checked
     *    first - an unsynchronized shadow is a reset's default value, not
     *    a time.
     *  - bypass (BYPSHAD = 1): the counters are read live and an RTCCLK
     *    edge between two reads makes them disagree, so the whole triple
     *    is read twice and compared, and a third time if needed.
     *
     * False = the reading could not be made coherent (a stopped RTCCLK
     * in bypass mode, an RSF that never rose in shadow mode). `out` is
     * untouched in that case.
     */
    static bool read(RtcReading& out) {
        if (!bypass_shadow()) {
            if (!synchronized()) {
                return false;
            }
            const uint16_t ss = static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
            const uint32_t tr = RTC->TR;
            const uint32_t dr = RTC->DR;
            out.subsecond = ss;
            out.time = rtc_decode(tr, dr);
            return true;
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
     * The wall clock this stratum's sleep site measures a frozen span
     * with: milliseconds since the top of the hour, 0..3599999.
     *
     * WHY THE HOUR AND NOT THE DAY. A span is a difference taken modulo
     * the wrap, and every extra field costs a BCD decode on a path that
     * runs inside a wake. One hour is longer than any sleep a kernel
     * timebase can usefully be resynchronized across (the tick counter
     * itself is 49.7 days, but a program that slept an hour has other
     * problems), and the modulus is a round number a caller can reason
     * about. Longer spans want read() and a calendar of their own.
     *
     * 0xFFFFFFFF when the calendar could not be read coherently.
     */
    /**
     * RTC_SSR alone - the synchronous prescaler's counter, counting DOWN
     * from PREDIV_S and reloading at every calendar update.
     *
     * read() gives the calendar; this gives the PHASE, in one load where
     * read() costs three plus a coherence retry. It is what 30.3.11's
     * synchronization procedure actually needs (the offset to a remote
     * clock is computed from SS and nothing else), it is shift()'s own
     * precondition, and it is the only reading fine enough to catch the
     * 1 Hz edge AS IT HAPPENS: a value that has RISEN since the previous
     * read is a reload, located to whatever the caller's loop costs
     * rather than to the 1/(PREDIV_S+1) second the value itself
     * resolves. With BYPSHAD clear it is a shadow reading like any
     * other and the caller owes wait_sync() first.
     */
    static uint16_t subsecond() {
        return static_cast<uint16_t>(RTC->SSR & RTC_SSR_SS_Msk);
    }

    static uint32_t time_of_hour_ms() {
        RtcReading r{};
        if (!read(r)) {
            return 0xFFFFFFFFu;
        }
        const uint16_t sync = prescalers().sync;
        return (static_cast<uint32_t>(r.time.minute) * 60'000UL) +
               (static_cast<uint32_t>(r.time.second) * 1000UL) +
               rtc_subsecond_ms(r.subsecond, sync);
    }

    /// The difference between two time_of_hour_ms() readings, wrap-safe
    /// over the hour. Undefined (and meaningless) for spans past an hour.
    static constexpr uint32_t elapsed_ms(uint32_t from, uint32_t to) {
        constexpr uint32_t hour = 3'600'000UL;
        return (to >= from) ? (to - from) : (hour - from + to);
    }

    // ---- daylight saving (30.3.8) -------------------------------------------

    /// ADD1H / SUB1H: one hour in one operation, without stopping the
    /// calendar. SUB1H "has no effect when current hour is 0" - the
    /// chapter's own sentence, not enforced here because the register
    /// enforces it.
    static void shift_hour(bool add) {
        unlock();
        RTC->CR = RTC->CR | (add ? RTC_CR_ADD1H : RTC_CR_SUB1H);
        lock();
    }

    /// The BKP bit of RTC_CR - one bit of memory for "I have already
    /// done the summer-time change", and nothing else.
    static void daylight_flag(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_BKP) : (RTC->CR & ~RTC_CR_BKP);
        lock();
    }
    static bool daylight_flag() { return (RTC->CR & RTC_CR_BKP) != 0u; }

    // ---- the sub-second shift (30.3.11) -------------------------------------

    /**
     * Shift the calendar by a fraction of a second - RTC_SHIFTR.
     *
     * `subfs` is ADDED to the synchronous prescaler counter, which
     * counts DOWN, so it DELAYS the clock by subfs / (PREDIV_S + 1)
     * seconds. With `add1s` the whole second is added first, so the pair
     * ADVANCES the clock by 1 - subfs / (PREDIV_S + 1) seconds in one
     * atomic operation. That asymmetry is the register's, not this
     * verb's: there is no "subtract a second" bit.
     *
     * FOUR REFUSALS, every one of them a sentence of 30.3.11 or 30.6.10:
     *  - `subfs` past the 15-bit field;
     *  - a shift already pending (SHPF), where the register says a write
     *    "has no effect" - a silent loss this verb turns into a false;
     *  - REFCKON set, which 30.3.11's caution forbids outright;
     *  - SS[15] set, the caution's overflow guard. On this family SS is
     *    16 bits and PREDIV_S is 15, so bit 15 can only be standing
     *    because an earlier shift put it there, and shifting again from
     *    that state is what the chapter tells the caller to avoid.
     *
     * It INITIATES and returns: the operation takes up to a second of
     * RTCCLK to land and `shift_pending()` is the completion, exactly as
     * the flag's own description has it. Writing SUBFS also clears RSF,
     * so a caller reading through the shadow registers waits for
     * `synchronized()` before believing what it reads.
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

    // ---- alarms -------------------------------------------------------------

    static constexpr uint32_t alarm_enable_bit(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RTC_CR_ALRAE : RTC_CR_ALRBE;
    }
    static constexpr uint32_t alarm_interrupt_bit(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RTC_CR_ALRAIE : RTC_CR_ALRBIE;
    }
    static constexpr uint32_t alarm_write_flag(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RTC_ICSR_ALRAWF : RTC_ICSR_ALRBWF;
    }
    static constexpr uint32_t alarm_flag(RtcAlarmId id) {
        return id == RtcAlarmId::a ? RtcFlag::alarm_a : RtcFlag::alarm_b;
    }

    static bool alarm_enabled(RtcAlarmId id) {
        return (RTC->CR & alarm_enable_bit(id)) != 0u;
    }

    /**
     * Program an alarm: 30.3.8's own three steps - clear ALRxE, write
     * the two registers, set ALRxE again - with the write flag waited
     * for in between (fact 6), the flag cleared, and the interrupt
     * enable set or left alone as asked.
     *
     * THE CAUTION IS ENFORCED. 30.3.6: "if the seconds field is selected
     * (MSK1 reset), the synchronous prescaler division factor set in the
     * RTC_PRER register must be at least 3" - so an alarm comparing
     * seconds under a PREDIV_S below 2 is refused rather than armed into
     * an undefined comparison.
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
            ready = (RTC->ICSR & alarm_write_flag(id)) != 0u;
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
        RTC->SCR = alarm_flag(id);
        RTC->CR = RTC->CR | alarm_enable_bit(id) |
                  (interrupt ? alarm_interrupt_bit(id) : 0u);
        lock();
        if (interrupt) {
            wake_line_open();
        }
        return true;
    }

    /// Turn an alarm off, interrupt and all, and clear its flag.
    static void clear_alarm(RtcAlarmId id) {
        unlock();
        RTC->CR = RTC->CR & ~(alarm_enable_bit(id) | alarm_interrupt_bit(id));
        RTC->SCR = alarm_flag(id);
        lock();
    }

    // ---- the wake-up timer --------------------------------------------------

    static bool wakeup_enabled() { return (RTC->CR & RTC_CR_WUTE) != 0u; }
    static bool wakeup_write_allowed() {
        return (RTC->ICSR & RTC_ICSR_WUTWF) != 0u;
    }

    /**
     * Program the periodic wake-up timer: 30.3.8's sequence exactly -
     * clear WUTE, POLL WUTWF, write WUT and WUCKSEL, set WUTE.
     *
     * `reload` is the register value: the flag comes every reload + 1
     * ticks of ck_wut, which is what rtc_wakeup_clock_hz() prices. False
     * when the setting is impossible or when WUTWF never rose (a stopped
     * RTCCLK).
     *
     * The flag is cleared before the timer is enabled, and ES0548 has no
     * item here - the WUTF flag "must be cleared by software at least
     * 1.5 RTCCLK periods before WUTF is set to 1 again" (30.6.18) is the
     * caller's obligation in the ISR, and isr() below meets it by
     * clearing at entry.
     */
    static bool set_wakeup(RtcWakeupClock c, uint32_t reload,
                           bool interrupt = true) {
        if (!rtc_wakeup_valid(c, reload)) {
            return false;
        }
        unlock();
        RTC->CR = RTC->CR & ~(RTC_CR_WUTE | RTC_CR_WUTIE);
        lock();
        bool ready = in_init();
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
        RTC->SCR = RtcFlag::wakeup;
        RTC->CR = RTC->CR | RTC_CR_WUTE | (interrupt ? RTC_CR_WUTIE : 0u);
        lock();
        if (interrupt) {
            wake_line_open();
        }
        return true;
    }

    /// Stop the wake-up timer and clear its flag. Returns once WUTE is
    /// written; WUTWF follows "up to 2 RTCCLK clock cycles" later
    /// (30.3.8), which is set_wakeup()'s wait and not this verb's.
    static void clear_wakeup() {
        unlock();
        RTC->CR = RTC->CR & ~(RTC_CR_WUTE | RTC_CR_WUTIE);
        RTC->SCR = RtcFlag::wakeup;
        lock();
    }

    // ---- smooth calibration -------------------------------------------------

    /**
     * Write RTC_CALR. RECALPF is waited out first: 30.6.4 says the
     * register "is blocked" while it stands, and the chapter's
     * re-calibration-on-the-fly section makes that up to three ck_spre
     * periods - three SECONDS at the usual rate, which is why the bound
     * here is generous and why this verb is not something to call in a
     * loop.
     */
    static bool calibrate(const RtcCalibration& c) {
        if (!rtc_calibration_valid(c)) {
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

    /**
     * Store into RTC_CALR WITHOUT opening the key window - the one verb
     * in this file that deliberately does not bracket itself.
     *
     * It exists so that the write protection can be PROVEN rather than
     * asserted: a lock is only evidenced by a store that does NOT land,
     * and an application may not reach a register itself (the stratum's
     * standing rule). CALR is the register chosen for it because the
     * worst a wrong value there can do is move the calendar by a few
     * hundred parts per million. Nothing but a bench suite should call
     * this.
     */
    static void calr_unprotected(uint32_t v) { RTC->CALR = v; }

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

    // ---- the timestamp (30.3.14) --------------------------------------------
    //
    // The event source is the RTC_TS pad (PC13 or PA4) or a tamper, and
    // this bench drives neither, so what is offered is the whole
    // register surface and nothing that pretends to have been measured.

    static void timestamp_enable(bool on, bool falling_edge = false) {
        unlock();
        uint32_t v = RTC->CR & ~(RTC_CR_TSE | RTC_CR_TSEDGE);
        if (falling_edge) {
            v |= RTC_CR_TSEDGE;   // TSE must be clear when TSEDGE changes
        }
        RTC->CR = v;
        if (on) {
            RTC->CR = v | RTC_CR_TSE;
        }
        lock();
    }
    static bool timestamp_enabled() { return (RTC->CR & RTC_CR_TSE) != 0u; }

    /**
     * TAMPTS - a TAMPER event fills the timestamp registers.
     *
     * It is the timestamp source that needs no RTC_TS pad, and on this
     * board it is the only one that can be staged at all: RTC_TS is
     * PC13, which carries the user button and its own pull-up. 31.3.4
     * also prices it - with TAMPTS set a tamper flag is asserted 3
     * ck_apre cycles after the event instead of at once.
     */
    static void timestamp_on_tamper(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_TAMPTS) : (RTC->CR & ~RTC_CR_TAMPTS);
        lock();
    }
    static bool timestamp_on_tamper() { return (RTC->CR & RTC_CR_TAMPTS) != 0u; }

    /// ITSE - the INTERNAL timestamp, whose one event is the switch to
    /// the VBAT supply (30.3.14). Offered because it is one bit of the
    /// same register and a program that keeps a timestamp wants to know
    /// which source filled it; ITSF in RTC_SR is how the two are told
    /// apart. It cannot be staged on a board with VDD only.
    static void timestamp_internal(bool on) {
        unlock();
        RTC->CR = on ? (RTC->CR | RTC_CR_ITSE) : (RTC->CR & ~RTC_CR_ITSE);
        lock();
    }
    static bool timestamp_internal() { return (RTC->CR & RTC_CR_ITSE) != 0u; }

    /// TSTR/TSDR/TSSSR as one reading. Valid only while TSF stands - the
    /// registers are frozen from the event until the flag is cleared.
    static RtcReading timestamp() {
        RtcReading r{};
        r.subsecond = static_cast<uint16_t>(RTC->TSSSR & RTC_TSSSR_SS_Msk);
        r.time = rtc_decode(RTC->TSTR, RTC->TSDR);
        return r;
    }

    // ---- flags and the interrupt --------------------------------------------

    static bool flag(uint32_t mask) { return (RTC->SR & mask) != 0u; }
    static void clear_flags(uint32_t mask) { RTC->SCR = mask & RtcFlag::all; }

    /// Unmask this peripheral's EXTI line (fact 7). Idempotent, and a
    /// single-bit read-modify-write, because IMR1 comes out of reset
    /// with every DIRECT line unmasked and a whole-register write would
    /// silently close somebody else's wake-up.
    static bool wake_line_open() { return Exti::interrupt(exti_line, true); }

    /**
     * The ISR body an app's RTC_TAMP_IRQHandler calls. Returns the
     * MASKED flags that were standing - so a handler dispatches on what
     * was really enabled, not on what merely happened - and clears
     * exactly those.
     *
     * NOTHING TO CLEAR IN THE EXTI: line 19 is direct and has no pending
     * bit (fact 7). The peripheral's own flag is the whole
     * acknowledgement, and 30.6.18's note - "after clearing the flag,
     * read it until it is read at 0 before leaving the interrupt
     * routine" - is why the read-back loop is here and not left to the
     * caller: the flags clear "a few APB clock cycles" after the write,
     * and a handler that returns before that re-enters immediately.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t masked = RTC->MISR & RtcFlag::all;
        if (masked == 0u) {
            return 0u;
        }
        RTC->SCR = masked;
        for (uint32_t i = 0; i < 64u && (RTC->SR & masked) != 0u; ++i) {
        }
        return masked;
    }

    // ---- debug --------------------------------------------------------------

    /// DBG_APB_FZ1.DBG_RTC_STOP: whether the calendar freezes when a
    /// debugger halts the core. Needs RCC_APBENR1.DBGEN, the reset.hpp
    /// caveat again, and like the watchdogs' bit it is "not reset by
    /// system reset" - whatever a debug session left there is what the
    /// next boot finds.
    static bool debug_freeze() {
        return (DBG->APBFZ1 & DBG_APB_FZ1_DBG_RTC_STOP) != 0u;
    }
    static void debug_freeze(bool on) {
        DBG->APBFZ1 = on ? (DBG->APBFZ1 | DBG_APB_FZ1_DBG_RTC_STOP)
                         : (DBG->APBFZ1 & ~DBG_APB_FZ1_DBG_RTC_STOP);
    }
};

// =============================================================================
// TAMP (RM0444 ch. 31) - the backup registers and the tamper detection
// =============================================================================

/**
 * TAMPFLT (31.6.3) - the block's detection MODE, not an input's.
 *
 * `edge` is the register's 0x0 and it is a different peripheral from the
 * other three: no filter, no sampling clock, no internal pull-up on the
 * pads and no latency between the pad and the flag. The three sample
 * counts are the LEVEL detector, which precharges each pad through the
 * internal pull-up, samples it at TAMPFREQ and fires after N
 * consecutive samples at the active level.
 *
 * One FLTCR serves every TAMP_INx, which is why the filter, the
 * sampling rate and the precharge are a BLOCK config here and only the
 * trigger polarity is per input.
 */
enum class TamperFilter : uint8_t {
    edge = 0,
    samples2 = 1,
    samples4 = 2,
    samples8 = 3,
};

/// TAMPFREQ - how often a filtered input is sampled, as RTCCLK divided
/// by a power of two from 32768 down to 256 (1 Hz to 128 Hz on a 32768
/// Hz RTCCLK). It is the detection LATENCY's other factor and, through
/// the precharge, the pull-up's duty cycle: 31.3.4 calls the choice the
/// trade-off between latency and the current the pull-up costs.
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
/// caller's argument, the rtc_ck_spre_hz() convention, because LSE and
/// LSI are not the same number and this file must not pretend to know
/// which one the domain took.
constexpr uint32_t tamper_sampling_hz(TamperSampling s, uint32_t rtcclk_hz) {
    return rtcclk_hz / tamper_sampling_divider(s);
}

/// TAMPPRCH - how long a pad is held by the internal pull-up before
/// each sample, in RTCCLK cycles. Larger capacitance on the input wants
/// a longer precharge; TamperConfig::pullup false turns the precharge
/// off entirely (TAMPPUDIS) and makes the sampler read whatever the
/// outside world holds.
enum class TamperPrecharge : uint8_t {
    cycles1 = 0,
    cycles2 = 1,
    cycles4 = 2,
    cycles8 = 3,
};

/**
 * TAMPxTRG - and THE BIT MEANS OPPOSITE SENSES IN THE TWO MODES, which
 * is why neither value is called "active low" here.
 *
 * 31.6.2 spells it out per bit: with TAMPFLT != 00 a zero means "input
 * staying LOW triggers", and with TAMPFLT == 00 the same zero means
 * "RISING edge and high level triggers". So the one register bit selects
 * a low level in the filtered detector and a rise in the edge detector -
 * an application that switches its filter on and keeps its trigger
 * constant has just inverted its own polarity. The names carry both
 * readings so the trap cannot be spelled away.
 */
enum class TamperTrigger : uint8_t {
    low_level_or_rising_edge = 0,
    high_level_or_falling_edge = 1,
};

/// The block-wide detection config - one TAMP_FLTCR.
struct TamperConfig {
    TamperFilter filter = TamperFilter::edge;
    TamperSampling sampling = TamperSampling::div32768;
    TamperPrecharge precharge = TamperPrecharge::cycles1;
    bool pullup = true;   ///< the precharge; TAMPPUDIS is its inverse
};

/// One TAMP_INx, in the manual's own 1-based numbering.
struct TamperInput {
    uint8_t index = 1;
    TamperTrigger trigger = TamperTrigger::low_level_or_rising_edge;

    /// TAMPxNOERASE inverted. The DEFAULT ERASES THE BACKUP REGISTERS,
    /// which is the register's own reset state and the whole point of
    /// the block, and is stated here rather than hidden behind a
    /// friendlier default.
    bool erase_backups = true;

    /// TAMPxMSK: the flag is masked and cleared by hardware, the backup
    /// registers are NOT erased, and what survives is the trigger to the
    /// low-power timers. 31.3.4 restricts it to the FILTERED detector,
    /// and 31.6.2 forbids the interrupt with it - both are refusals in
    /// tamper_input_valid().
    bool masked = false;

    bool interrupt = false;   ///< TAMPxIE
};

/// The two illegal combinations, both of them sentences of ch. 31, and
/// the index checked against what the part declares.
constexpr bool tamper_input_valid(const TamperInput& t, TamperFilter f) {
    if (t.index < 1u || t.index > tamp_external_inputs()) {
        return false;
    }
    if (t.masked && t.interrupt) {
        return false;   // 31.6.2: "must not be enabled when TAMPxMSK is set"
    }
    if (t.masked && f == TamperFilter::edge) {
        return false;   // 31.3.4: level detection with filtering only
    }
    return true;
}

/// One bit per TAMP event - the shape TAMP_SR, TAMP_MISR and TAMP_SCR
/// all share, the RtcFlag convention.
struct TampFlag {
    static constexpr uint32_t tamper1 = TAMP_SR_TAMP1F;
    static constexpr uint32_t tamper2 = TAMP_SR_TAMP2F;
    /// THE THIRD INPUT IS THE G0B1'S ALONE - the G071's and the G031's
    /// headers declare TAMP1F and TAMP2F and stop, which is why
    /// tamp_external_inputs() is a probe and not a constant. Its mask is
    /// zero where the part has none, so every expression over these
    /// stays legal on every header.
#if defined(TAMP_SR_TAMP3F_Msk)
    static constexpr uint32_t tamper3 = TAMP_SR_TAMP3F;
#else
    static constexpr uint32_t tamper3 = 0u;
#endif
    static constexpr uint32_t external = tamper1 | tamper2 | tamper3;

    /// All three headers of this pack declare the four internal sources,
    /// and the probe is still a probe because a part without them would
    /// otherwise compile a mask that does not exist.
#if defined(TAMP_SR_ITAMP3F_Msk)
    static constexpr uint32_t lse_monitor = TAMP_SR_ITAMP3F;
    static constexpr uint32_t hse_monitor = TAMP_SR_ITAMP4F;
    static constexpr uint32_t calendar_overflow = TAMP_SR_ITAMP5F;
    static constexpr uint32_t manufacturer_readout = TAMP_SR_ITAMP6F;
#else
    static constexpr uint32_t lse_monitor = 0u;
    static constexpr uint32_t hse_monitor = 0u;
    static constexpr uint32_t calendar_overflow = 0u;
    static constexpr uint32_t manufacturer_readout = 0u;
#endif
    static constexpr uint32_t internal = lse_monitor | hse_monitor |
                                         calendar_overflow |
                                         manufacturer_readout;
    static constexpr uint32_t all = external | internal;
};

/// The flag of external tamper `index` (1-based), 0 for an index this
/// part has not got.
constexpr uint32_t tamper_flag(uint8_t index) {
    return (index >= 1u && index <= tamp_external_inputs())
               ? (1UL << (index - 1u))
               : 0u;
}

/// The flag of internal tamper `y` (the manual's ITAMP3..ITAMP6), 0
/// where the part has none.
constexpr uint32_t internal_tamper_flag(uint8_t y) {
    if (!tamp_has_internal_tampers() || y < tamp_internal_first ||
        y > tamp_internal_last) {
        return 0u;
    }
    return 1UL << (15u + y);   // ITAMP3F is bit 18
}

/**
 * The tamper and backup block.
 *
 * WHAT IS BUILT: the five backup registers, which are ordinary 32-bit
 * words in the RTC domain - they survive every system reset, every
 * Standby and every Shutdown, and they are lost only to a domain reset
 * or a loss of both VDD and VBAT. That makes them this target's third
 * kind of surviving storage, beside the .noinit breadcrumb (a system
 * reset only) and the flash journal (everything).
 *
 * AND THE TAMPER DETECTION, which is the rest of the chapter: three
 * external inputs with an edge detector and a filtered level detector,
 * four internal sources on the parts that declare them, the erase of the
 * backup registers, the mask that suppresses it, the timestamp
 * (RTC_CR.TAMPTS, over in Rtc) and the interrupt.
 *
 * WHAT ARMING ONE COSTS, said once and not hidden behind a default: a
 * detected tamper ERASES the five backup registers unless TAMPxNOERASE
 * or TAMPxMSK says otherwise, and those five words are the third kind of
 * surviving storage this stratum has. `any_armed()` stays exactly what
 * it was - the one question a program keeping something there needs
 * answered about the state it inherited.
 *
 * The domain gate is RtcDomain's: DBP for the write access, RTCAPBEN for
 * the register bank. The RTC's WPR key does NOT cover this block -
 * 30.3.8's list is the RTC's own registers - which the bench suite
 * checks rather than assumes.
 */
struct Tamp {
    Tamp() = delete;

    static constexpr uint8_t backup_count = tamp_backup_registers();
    static constexpr uint8_t exti_line = tamp_exti_line;
    static constexpr IRQn_Type irq() { return rtc_irq(); }

    /// Backup register `n`, 0 for an index this part has not got.
    static uint32_t backup(uint8_t n) {
        if (n >= backup_count) {
            return 0u;
        }
        return (&TAMP->BKP0R)[n];
    }

    /// Write one. False for an index past the end - the only failure
    /// this verb can see; a write with DBP clear is dropped by the
    /// silicon and the readback is how a caller learns that.
    static bool backup(uint8_t n, uint32_t value) {
        if (n >= backup_count) {
            return false;
        }
        (&TAMP->BKP0R)[n] = value;
        return true;
    }

    // ---- the read-only half -------------------------------------------------

    static uint32_t config1() { return TAMP->CR1; }
    static uint32_t config2() { return TAMP->CR2; }
    static uint32_t filter() { return TAMP->FLTCR; }
    static uint32_t interrupts() { return TAMP->IER; }
    static uint32_t status() { return TAMP->SR; }
    static uint32_t masked_status() { return TAMP->MISR; }

    /// Is any EXTERNAL tamper input armed? The low half of TAMP_CR1.
    static bool any_armed() { return (TAMP->CR1 & 0xFFFFu) != 0u; }

    /// Is any INTERNAL tamper armed? IT IS NOT A RHETORICAL QUESTION:
    /// 31.6.1 gives TAMP_CR1 the RTC domain reset value 0xFFFF0000, and
    /// bits 18..21 of that are ITAMP3E..ITAMP6E - so a part that has
    /// never been told anything about tampering comes out of a domain
    /// reset with LSE monitoring, HSE monitoring, the calendar overflow
    /// and the ST manufacturer readout ALL ARMED, measured on this
    /// silicon. None of them has a NOERASE bit.
    static bool any_internal_armed() {
        return (TAMP->CR1 & TampFlag::internal) != 0u;
    }

    /// THE QUESTION A PROGRAM KEEPING SOMETHING IN THE BACKUP REGISTERS
    /// ACTUALLY NEEDS ANSWERED: is anything at all armed that would
    /// erase them? Both halves, because the reset value arms the
    /// internal one and an application asking only about the pads would
    /// be told "no" by a chip that is one calendar overflow away from
    /// wiping its breadcrumb.
    static bool erase_source_armed() {
        return any_armed() || any_internal_armed();
    }

    /// Clear a tamper flag (TAMP_SCR). Offered because a flag standing
    /// from a previous life blocks a Standby entry (table 33's own
    /// precondition list), and clearing one is not arming anything.
    static void clear_flags(uint32_t mask) { TAMP->SCR = mask; }

    // ---- the detection half (31.3.4) ----------------------------------------

    static constexpr uint8_t input_count = tamp_external_inputs();
    static constexpr bool has_internal_tampers = tamp_has_internal_tampers();

    /**
     * The block's detection mode - TAMP_FLTCR.
     *
     * REFUSED WHILE ANY INPUT IS ARMED, and that is 31.6.1's own
     * footnote: the mode "must be configured before enabling the tamper
     * detection". Nothing in the silicon enforces it, so a store into a
     * live block would land and change what a running detector means
     * halfway through a sample train.
     */
    static bool filter_config(const TamperConfig& c) {
        if (any_armed()) {
            return false;
        }
        TAMP->FLTCR =
            (static_cast<uint32_t>(c.sampling) << TAMP_FLTCR_TAMPFREQ_Pos) |
            (static_cast<uint32_t>(c.filter) << TAMP_FLTCR_TAMPFLT_Pos) |
            (static_cast<uint32_t>(c.precharge) << TAMP_FLTCR_TAMPPRCH_Pos) |
            (c.pullup ? 0u : TAMP_FLTCR_TAMPPUDIS);
        return true;
    }

    static TamperFilter filter_mode() {
        return static_cast<TamperFilter>(
            (TAMP->FLTCR & TAMP_FLTCR_TAMPFLT_Msk) >> TAMP_FLTCR_TAMPFLT_Pos);
    }

    static bool armed(uint8_t index) {
        const uint32_t bit = tamper_flag(index);
        return bit != 0u && (TAMP->CR1 & bit) != 0u;
    }

    /**
     * Arm one external tamper input.
     *
     * The ORDER is the chapter's: the trigger, the erase policy, the
     * mask and the interrupt all land BEFORE TAMPxE, because the same
     * footnote that governs filter_config() governs TAMP_CR2. Re-arming
     * an already-armed input is refused for the same reason - the caller
     * disarms first, which is also 31.3.4's own advice after a detection
     * ("the TAMP_INx should be disabled and then re-enabled").
     */
    static bool arm(const TamperInput& t) {
        if (!tamper_input_valid(t, filter_mode()) || armed(t.index)) {
            return false;
        }
        const uint32_t n = t.index - 1u;
        uint32_t cr2 = TAMP->CR2;
        cr2 &= ~((1UL << n) | (1UL << (16u + n)) | (1UL << (24u + n)));
        if (!t.erase_backups) {
            cr2 |= 1UL << n;              // TAMPxNOERASE
        }
        if (t.masked) {
            cr2 |= 1UL << (16u + n);      // TAMPxMSK
        }
        if (t.trigger == TamperTrigger::high_level_or_falling_edge) {
            cr2 |= 1UL << (24u + n);      // TAMPxTRG
        }
        TAMP->CR2 = cr2;
        TAMP->IER = t.interrupt ? (TAMP->IER | (1UL << n))
                                : (TAMP->IER & ~(1UL << n));
        TAMP->CR1 = TAMP->CR1 | (1UL << n);
        return true;
    }

    /// Disarm one input, interrupt included - a disarmed input that can
    /// still raise a vector would be a trap of this driver's making.
    static bool disarm(uint8_t index) {
        const uint32_t bit = tamper_flag(index);
        if (bit == 0u) {
            return false;
        }
        TAMP->CR1 = TAMP->CR1 & ~bit;
        TAMP->IER = TAMP->IER & ~bit;
        return true;
    }

    /**
     * One INTERNAL tamper, by the manual's own index (ITAMP3 = LSE
     * monitoring, 4 = HSE monitoring, 5 = the calendar overflow, 6 = the
     * ST manufacturer readout).
     *
     * They are the parts of this chapter that need no pad at all, and
     * they ERASE THE BACKUP REGISTERS TOO: 31.3.4's erase is driven by
     * ITAMPyF exactly as it is by TAMPxF, and there is no NOERASE bit
     * for an internal source. False where the part declares none.
     */
    static bool internal_tamper(uint8_t y, bool on, bool interrupt = false) {
        const uint32_t bit = internal_tamper_flag(y);
        if (bit == 0u) {
            return false;
        }
        TAMP->CR1 = on ? (TAMP->CR1 | bit) : (TAMP->CR1 & ~bit);
        TAMP->IER = (on && interrupt) ? (TAMP->IER | bit)
                                      : (TAMP->IER & ~bit);
        return true;
    }

    static bool flag(uint32_t mask) { return (TAMP->SR & mask) != 0u; }

    /// Unmask this block's EXTI line (table 65's DIRECT line 21). The
    /// RTC's own wake_line_open() is the twin, and the two lines share
    /// one vector.
    static bool wake_line_open() { return Exti::interrupt(exti_line, true); }

    /**
     * The ISR body an app's RTC_TAMP_IRQHandler calls for this half.
     * Returns the MASKED flags it served and clears exactly those, the
     * Rtc::isr() convention - so one handler can call both bodies and
     * dispatch on the union.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t served = TAMP->MISR & TampFlag::all;
        if (served != 0u) {
            TAMP->SCR = served;
        }
        return served;
    }
};

} // namespace brio
