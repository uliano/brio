/*
 * pwr.hpp
 *
 * Power control (RM0444 ch. 4): the voltage regulator's two ranges, the
 * supply supervisor, the RTC domain's write gate, the six wake-up pins,
 * the Standby pull configuration, and - the reason the chapter exists -
 * WHICH LOW-POWER MODE THE NEXT `wfi` TAKES.
 *
 *  Pwr        the block, whole. Every register of 4.4, every field this
 *             part implements, and nothing that pretends to be a policy.
 *
 * THE SLEEP MODE IS TWO REGISTERS IN TWO PLACES, and that is the first
 * thing to know about this family. The Cortex-M0+ has one bit, SCR.
 * SLEEPDEEP, which says "the next WFI is a deep sleep"; PWR_CR1.LPMS
 * says WHICH deep sleep. Neither is the other's, and both must agree
 * before a WFI does anything but Sleep. `Pwr` owns them BOTH - SLEEPDEEP
 * is written here and nowhere else in this stratum, so that "what is
 * armed" is one question with one answer (`Pwr::mode()`), and so that
 * stm32g0/platform_stm32.hpp's idle() can stay what it is: a DSB, a WFI
 * and an unmask that take whatever somebody else armed.
 *
 * SEVEN FACTS OF THIS CHAPTER.
 *
 * 1. THE BLOCK HAS A BUS-CLOCK GATE AND IT IS CLEAR AT RESET.
 *    RCC_APBENR1.PWREN, like every APB peripheral of this family, and
 *    5.2.17 says a peripheral without its bus clock does not answer
 *    register reads. `bus_clock(true)` is therefore the first call in
 *    this file's usage, and stm32g0/clock.hpp's Clock::init() already
 *    makes it (it needs PWR_CR1.VOS for the flash latency table) and
 *    leaves it open.
 *
 * 2. SEVEN MODES, AND ONLY FOUR OF THEM ARE A LADDER A PROGRAM CAN
 *    RESUME FROM. Run, Low-power run, Sleep, Low-power sleep, Stop 0,
 *    Stop 1, Standby, Shutdown (4.3). Of the ones a WFI reaches: Sleep
 *    keeps every clock but the CPU's; Stop 0 and Stop 1 stop everything
 *    in the VCORE domain but KEEP SRAM AND REGISTERS, differing only in
 *    which regulator supplies VCORE; Standby and Shutdown power the
 *    VCORE domain OFF and come back through the reset vector. That last
 *    sentence is why `PwrMode` has all six codes and why
 *    stm32g0/sleep.hpp's SleepSite maps only four of them - see its
 *    header.
 *
 * 3. LOW-POWER RUN IS A WHOLE-PROGRAM DECISION, not a sleep. LPR
 *    switches VCORE to the low-power regulator and 4.3.2 requires the
 *    system clock to be at or below 2 MHz FIRST; leaving it means
 *    clearing LPR, WAITING FOR REGLPF, and only then raising the clock.
 *    The verbs are here because the register is; nothing in this stratum
 *    calls them, because putting a whole program at 2 MHz is not
 *    something a sleep site may do behind an application's back.
 *
 * 4. WHAT COMES BACK AFTER A STOP IS HSISYS, WHATEVER WAS RUNNING
 *    BEFORE. 4.3.6 and 5.3: "the system clock, when exiting Stop 0 or
 *    Stop 1 mode, is the HSISYS clock", and the PLL is off. A program
 *    running at 64 MHz on the PLL therefore resumes at 16 MHz with a
 *    SysTick reload and a USART baud rate meant for four times that -
 *    which is a fact of the chapter and not a defect, and which
 *    stm32g0/sleep.hpp's site answers by re-running the clock task in
 *    the first thing that executes after the wake. HSIDIV is NOT
 *    changed by a Stop, so a program on a divided HSISYS comes back on
 *    exactly the clock it left.
 *
 * 5. A STOP IS ONLY ENTERED IF NOTHING IS PENDING. Table 31's own note:
 *    every EXTI pending bit and "the peripheral flags generating wake-up
 *    interrupts must be cleared. Otherwise, the Stop mode entry
 *    procedure is ignored and program execution continues" - and the
 *    Cortex's WFI is itself a no-op with an interrupt pending. So a
 *    Stop that does not happen is NOT an error condition anywhere in
 *    the silicon: the instruction simply falls through. Everything that
 *    judges a sleep in this stratum therefore judges it by TIME ELAPSED
 *    and never by a flag.
 *
 * 6. STANDBY AND SHUTDOWN NEED THEIR OWN FLAGS CLEAR TO BE ENTERED AT
 *    ALL (tables 33 and 34): the WUFx wake-up flags in PWR_SR1, and the
 *    RTC flag matching whichever RTC event is meant to end the sleep. A
 *    stale flag from a previous life is an entry that silently does not
 *    happen - fact 5 again, with a second cause. `clear_wakeup_flags()`
 *    is the sweep, and `enter()` performs it.
 *
 * 7. THE I/O PULLS FOR STANDBY ARE A SEPARATE SET OF REGISTERS FROM THE
 *    GPIO'S. In Standby and Shutdown the GPIO block is unpowered, so a
 *    pad's state comes from PWR_PUCRx / PWR_PDCRx, applied only while
 *    PWR_CR3.APC is set (4.3.8). They are NOT reset by a wake from
 *    Standby, which makes them one more piece of state a program
 *    inherits.
 *
 * PER-PART VARIABILITY, all of it in stm32g0/device_tables.hpp: the six
 * wake-up pins are a SPARSE set (the G031 bonds 1, 2, 4 and 6, the G071
 * adds 5, only the G0B1/G0C1 has all six), the pull registers follow the
 * GPIO bonding (port E is the G0B1/G0C1's alone), and the VDDIO2 monitor
 * exists only where the second I/O supply does. Every verb below that
 * takes a pin number or a port letter checks the reserve and returns
 * false rather than writing a bit that is not there.
 *
 * ERRATA, ES0548 Rev 3 read on the bench chip's REVISION Z column:
 *  - 2.2.2 (a wake-up flag wrongly set while a WKUPx pin is being
 *    configured) is LIVE, and its workaround is one line: clear WUFx
 *    after configuring the pin. `wakeup_pin()` does exactly that, so
 *    the erratum cannot reach a caller of this file.
 *  - 2.2.4 (with HSIDIV != 0 a peripheral with clock-request capability
 *    fails to wake the device from Stop) is LIVE and has NO workaround.
 *    It is not refusable here: it does not touch the RTC, the EXTI or
 *    the wake-up pins, only the peripherals that ask for HSI16 while
 *    stopped (the USARTs, the LPUARTs, I2C1). `stop_hsidiv_hazard()` is
 *    the readable predicate an application asserts on, and the doc says
 *    what it covers.
 *  - 2.2.7 (SRAM corrupted on Standby entry) is REVISION A ONLY - "-"
 *    in the Z column - which is what makes Standby usable on this board
 *    at all.
 *  - 2.3.1 (a wake-up-capable GPIO not configurable after a Standby
 *    wake) is REVISION A ONLY as well.
 *  - 2.2.11 (a missed RTC domain reset after a supply dip) is LIVE, and
 *    belongs to the domain rather than to this chapter: its workaround
 *    is stm32g0/rtc.hpp's RtcDomain::reset(), keyed on PWRRSTF.
 *
 * NOT BUILT (docs/stm32g0/pwr.md carries the list): the VBAT charging
 * switch (PWR_CR4.VBE/VBRS - it drives current into a battery this desk
 * has not got); the BOR levels, which are OPTION BYTES and belong to the
 * flash chapter's provisioning verb, not to a register here; and
 * SLEEPONEXIT / SEVONPEND, two Cortex bits whose use is a kernel design
 * decision (brio's loop returns to the idle path rather than staying in
 * a handler) and not a power one.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/device_tables.hpp"

namespace brio {

/// PWR_CR1.LPMS, plus the Cortex bit that decides whether it is
/// consulted at all. `sleep` is not an LPMS code - it is SLEEPDEEP = 0,
/// which is what the four LPMS codes below have in common: they only
/// mean anything with SLEEPDEEP set.
enum class PwrMode : uint8_t {
    sleep = 0xFF,     ///< SLEEPDEEP = 0: the CPU clock stops and nothing else
    stop0 = 0,        ///< LPMS 000, main regulator on
    stop1 = 1,        ///< LPMS 001, low-power regulator
    standby = 3,      ///< LPMS 011, VCORE off; wakes through the reset vector
    shutdown = 4,     ///< LPMS 1xx, everything off but the RTC domain
};

/// LPMS 010 is Reserved (4.4.1) and 1xx is one mode with four
/// spellings; this driver writes 100 and refuses the rest.
constexpr bool pwr_mode_valid(PwrMode m) {
    switch (m) {
        case PwrMode::sleep:
        case PwrMode::stop0:
        case PwrMode::stop1:
        case PwrMode::standby:
        case PwrMode::shutdown:
            return true;
    }
    return false;
}

/// Does this mode power the VCORE domain off - i.e. does the program
/// come back through the reset vector instead of the instruction after
/// the WFI? The one distinction that matters to anything above this
/// file.
constexpr bool pwr_mode_resets(PwrMode m) {
    return m == PwrMode::standby || m == PwrMode::shutdown;
}

/// Does this mode stop the VCORE clocks - SysTick, every TIM, the buses?
constexpr bool pwr_mode_stops_clocks(PwrMode m) {
    return m != PwrMode::sleep;
}

/// PWR_CR2.PVDRT / PVDFT: the supply thresholds the detector compares
/// VDD against (4.2.2). The two fields are independent and the falling
/// one must sit below the rising one; the codes are the register's own.
enum class PvdRising : uint8_t {
    v2_1 = 0, v2_2 = 1, v2_5 = 2, v2_6 = 3, v2_7 = 4, v2_9 = 5, v3_0 = 6,
    pvd_in = 7,   ///< the PVD_IN pad against VREFINT; the falling field is ignored
};
enum class PvdFalling : uint8_t {
    v2_0 = 0, v2_2 = 1, v2_4 = 2, v2_5 = 3, v2_6 = 4, v2_8 = 5, v2_9 = 6,
};

struct PvdConfig {
    PvdRising rising = PvdRising::v2_9;
    PvdFalling falling = PvdFalling::v2_8;
};

/// 4.2.2: "VPVDFx should always be set to a lower voltage level than
/// VPVDRx" - a hysteresis the wrong way round is a detector that
/// chatters, so it is refused. The pad mode ignores the falling field
/// and is accepted whatever it holds.
constexpr bool pvd_config_valid(const PvdConfig& c) {
    if (c.rising == PvdRising::pvd_in) {
        return true;
    }
    return static_cast<uint8_t>(c.falling) <= static_cast<uint8_t>(c.rising);
}

/**
 * The PWR block.
 *
 * MONOSTATE: one instance on every part, one set of registers, no
 * handle. The verbs are register-level facts of chapter 4; the ORDERING
 * rules that involve other blocks (the flash latency before a range
 * change, the clock below 2 MHz before LPR, the re-init of the clock
 * after a Stop) belong to their tasks, so that a verb here never does
 * two things.
 */
struct Pwr {
    Pwr() = delete;

    /// A bound on every status wait here. The longest of them is the
    /// regulator's range change, tens of microseconds; at 64 MHz this is
    /// milliseconds, and it turns a dead flag into a false.
    static constexpr uint32_t ready_spins = 1'000'000UL;

    // ---- the bus clock (fact 1) --------------------------------------------

    static void bus_clock(bool on) {
        RCC->APBENR1 = on ? (RCC->APBENR1 | RCC_APBENR1_PWREN)
                          : (RCC->APBENR1 & ~RCC_APBENR1_PWREN);
        (void)RCC->APBENR1;
    }
    static bool bus_clock() { return (RCC->APBENR1 & RCC_APBENR1_PWREN) != 0u; }

    // ---- raw registers ------------------------------------------------------

    static uint32_t cr1() { return PWR->CR1; }
    static uint32_t cr2() { return PWR->CR2; }
    static uint32_t cr3() { return PWR->CR3; }
    static uint32_t cr4() { return PWR->CR4; }
    static uint32_t sr1() { return PWR->SR1; }
    static uint32_t sr2() { return PWR->SR2; }

    // ---- voltage scaling (4.1.4) --------------------------------------------

    /// PWR_CR1.VOS as the chapter numbers it: 1 = Range 1 (up to
    /// 64 MHz), 2 = Range 2 (up to 16 MHz). 0 and 3 "cannot be written
    /// (forbidden by hardware)" and never read back.
    static uint8_t range() {
        return static_cast<uint8_t>((PWR->CR1 & PWR_CR1_VOS_Msk) >> PWR_CR1_VOS_Pos);
    }

    /// PWR_SR2.VOSF: the regulator is still moving to the range asked
    /// for. 4.1.4's rise sequence waits for this to CLEAR before the
    /// wait states and the frequency go up.
    static bool range_changing() { return (PWR->SR2 & PWR_SR2_VOSF) != 0u; }

    /**
     * Write VOS. THE ORDER AROUND IT IS THE CALLER'S, and 4.1.4 gives
     * two different ones: down is frequency, then wait states, then this;
     * up is this, then VOSF, then wait states, then frequency. A verb
     * that guessed which direction the caller meant would be a task, and
     * the task for it is stm32g0/clock.hpp's.
     *
     * False for a code the hardware forbids, or when the regulator did
     * not report ready within the bound.
     */
    static bool range(uint8_t r) {
        if (r != 1u && r != 2u) {
            return false;
        }
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS_Msk) |
                   ((static_cast<uint32_t>(r) << PWR_CR1_VOS_Pos) & PWR_CR1_VOS_Msk);
        for (uint32_t i = 0; i < ready_spins; ++i) {
            if (!range_changing()) {
                return range() == r;
            }
        }
        return false;
    }

    // ---- the RTC domain gate (4.1.2) ----------------------------------------
    //
    // The bit lives in this register and the DOMAIN is
    // stm32g0/rtc.hpp's, which is why RtcDomain spells the same two
    // verbs: a program that never touches the RTC still has a right to
    // read this bit off the block that owns the register.

    static void rtc_domain_unlock(bool on) {
        PWR->CR1 = on ? (PWR->CR1 | PWR_CR1_DBP) : (PWR->CR1 & ~PWR_CR1_DBP);
    }
    static bool rtc_domain_unlocked() { return (PWR->CR1 & PWR_CR1_DBP) != 0u; }

    // ---- the flash power-down bits (4.4.1) ----------------------------------

    /// FPD_STOP: put the flash in power-down while stopped. Cheaper, and
    /// paid for on the way out - table 31 prices the wake at "the longest
    /// wake-up time between HSI16 and flash memory", and PWR_SR2's
    /// FLASH_RDY is when the array is back.
    static void flash_power_down_stop(bool on) {
        PWR->CR1 = on ? (PWR->CR1 | PWR_CR1_FPD_STOP)
                      : (PWR->CR1 & ~PWR_CR1_FPD_STOP);
    }
    static bool flash_power_down_stop() {
        return (PWR->CR1 & PWR_CR1_FPD_STOP) != 0u;
    }
    /// FPD_LPSLP / FPD_LPRUN: the same for the two low-power run modes.
    /// 4.4.1: legal "only when the user code is executed from SRAM",
    /// which nothing in this stratum does - offered, never used here.
    static void flash_power_down_lp_sleep(bool on) {
        PWR->CR1 = on ? (PWR->CR1 | PWR_CR1_FPD_LPSLP)
                      : (PWR->CR1 & ~PWR_CR1_FPD_LPSLP);
    }
    static void flash_power_down_lp_run(bool on) {
        PWR->CR1 = on ? (PWR->CR1 | PWR_CR1_FPD_LPRUN)
                      : (PWR->CR1 & ~PWR_CR1_FPD_LPRUN);
    }
    static bool flash_ready() { return (PWR->SR2 & PWR_SR2_FLASH_RDY) != 0u; }

    // ---- low-power run (fact 3) ---------------------------------------------

    static bool low_power_run() { return (PWR->CR1 & PWR_CR1_LPR) != 0u; }
    static bool on_low_power_regulator() {
        return (PWR->SR2 & PWR_SR2_REGLPF) != 0u;
    }
    static bool low_power_regulator_ready() {
        return (PWR->SR2 & PWR_SR2_REGLPS) != 0u;
    }

    /// Set or clear LPR. The 2 MHz precondition on the way in and the
    /// REGLPF wait on the way out are 4.3.2's, and only the second is
    /// something a register can answer - so the wait is here and the
    /// precondition is the caller's, stated and not enforced (this file
    /// does not know the system clock).
    static bool low_power_run(bool on) {
        PWR->CR1 = on ? (PWR->CR1 | PWR_CR1_LPR) : (PWR->CR1 & ~PWR_CR1_LPR);
        if (on) {
            return true;
        }
        for (uint32_t i = 0; i < ready_spins; ++i) {
            if (!on_low_power_regulator()) {
                return true;
            }
        }
        return false;
    }

    // ---- the sleep mode (fact 2) --------------------------------------------

    /// The Cortex's SLEEPDEEP, written HERE and nowhere else in this
    /// stratum (see the file header).
    static void deep_sleep(bool on) {
        SCB->SCR = on ? (SCB->SCR | SCB_SCR_SLEEPDEEP_Msk)
                      : (SCB->SCR & ~SCB_SCR_SLEEPDEEP_Msk);
    }
    static bool deep_sleep() { return (SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u; }

    static uint8_t lpms() {
        return static_cast<uint8_t>((PWR->CR1 & PWR_CR1_LPMS_Msk) >> PWR_CR1_LPMS_Pos);
    }

    /**
     * Arm the mode the next WFI will take: SLEEPDEEP and LPMS together,
     * which is the only pair that means anything (fact 2).
     *
     * ARMING IS NOT SLEEPING. Nothing stops here; the CPU stops when
     * something executes a WFI, which in a brio program is the kernel's
     * idle path. That split is what lets util/power.hpp's model exist
     * with no new kernel hook, and it is why this verb is named for what
     * it does.
     *
     * False for a Reserved code. Reading it back is `mode()`, and the
     * readback is the state - there is no shadow variable anywhere in
     * this stratum.
     */
    static bool arm(PwrMode m) {
        if (!pwr_mode_valid(m)) {
            return false;
        }
        if (m == PwrMode::sleep) {
            deep_sleep(false);
            return true;
        }
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS_Msk) |
                   ((static_cast<uint32_t>(m) << PWR_CR1_LPMS_Pos) & PWR_CR1_LPMS_Msk);
        (void)PWR->CR1;
        deep_sleep(true);
        return true;
    }

    /// What a following WFI would take. A pure read of the two
    /// registers: SLEEPDEEP clear is Sleep whatever LPMS holds, and the
    /// Reserved LPMS code 010 - which this driver never writes - reads
    /// back as Stop 1, the nearest implemented mode below it, so that
    /// this function is total.
    static PwrMode mode() {
        if (!deep_sleep()) {
            return PwrMode::sleep;
        }
        const uint8_t code = lpms();
        if ((code & 0x4u) != 0u) {
            return PwrMode::shutdown;
        }
        switch (code) {
            case 0: return PwrMode::stop0;
            case 1: return PwrMode::stop1;
            case 3: return PwrMode::standby;
            default: return PwrMode::stop1;
        }
    }

    /**
     * ES0548 2.2.4's condition, as a readable predicate: with
     * RCC_CR.HSIDIV different from zero, "peripherals with clock request
     * capability fail to wake the device up from Stop modes" and a Stop
     * entry from an HSE system clock does not happen at all. There is no
     * workaround, so this is not a refusal - the RTC, the EXTI lines and
     * the wake-up pins are unaffected, and a program whose only wake is
     * one of those is not touched by it.
     */
    static bool stop_hsidiv_hazard() {
        return (RCC->CR & RCC_CR_HSIDIV_Msk) != 0u;
    }

    // ---- wake-up pins (4.4.3, 4.4.4, 4.4.5) ---------------------------------

    static constexpr uint8_t wakeup_pin_count = 6;
    static constexpr bool wakeup_pin_present(uint8_t n) {
        return pwr_wakeup_pin_present(n);
    }

    /**
     * Enable or disable WKUPn with its polarity. The four registers the
     * pin appears in (EWUPn in CR3, WPn in CR4, WUFn in SR1, CWUFn in
     * SCR) all share bit n-1, which is why one probe answers for all of
     * them.
     *
     * ES0548 2.2.2 IS APPLIED HERE: configuring a wake-up pin can set
     * its own flag, and a standing WUFx blocks the very Standby entry
     * the pin was configured for (fact 6), so the flag is cleared after
     * the configuration and the erratum cannot reach a caller.
     *
     * False for a pin this part does not bond.
     */
    static bool wakeup_pin(uint8_t n, bool on, bool falling = false) {
        if (!wakeup_pin_present(n)) {
            return false;
        }
        const uint32_t bit = 1UL << (n - 1u);
        PWR->CR4 = falling ? (PWR->CR4 | bit) : (PWR->CR4 & ~bit);
        PWR->CR3 = on ? (PWR->CR3 | bit) : (PWR->CR3 & ~bit);
        PWR->SCR = bit;   // ES0548 2.2.2
        return true;
    }

    static bool wakeup_pin_enabled(uint8_t n) {
        return wakeup_pin_present(n) && (PWR->CR3 & (1UL << (n - 1u))) != 0u;
    }

    /// PWR_SR1.WUFn - did this pin wake the device? Survives a Standby
    /// wake (4.4.5: "neither reset upon exiting Standby mode nor with
    /// the PWRRST bit"), which is what makes it readable at the next
    /// boot.
    static bool wakeup_flag(uint8_t n) {
        return wakeup_pin_present(n) && (PWR->SR1 & (1UL << (n - 1u))) != 0u;
    }

    /// PWR_SR1.SBF - "the device entered Standby mode". Not cleared by a
    /// system reset either, so the boot after a Standby can tell itself
    /// apart from a boot after a reset. Shutdown clears everything,
    /// including this, which is how the two are told apart.
    static bool standby_flag() { return (PWR->SR1 & PWR_SR1_SBF) != 0u; }

    /// PWR_SR1.WUFI - a wake came from the INTERNAL wake-up line (the
    /// RTC, the TAMP), the one that has no pin. Read-only and not
    /// clearable: it stands while any internal source does.
    static bool internal_wakeup_flag() { return (PWR->SR1 & PWR_SR1_WUFI) != 0u; }

    /// PWR_CR3.EIWUL - whether that internal line may wake the device
    /// at all. SET at reset (PWR_CR3 resets to 0x8000), which is why an
    /// RTC alarm out of a Standby works with nothing configured here.
    static void internal_wakeup(bool on) {
        PWR->CR3 = on ? (PWR->CR3 | PWR_CR3_EIWUL) : (PWR->CR3 & ~PWR_CR3_EIWUL);
    }
    static bool internal_wakeup() { return (PWR->CR3 & PWR_CR3_EIWUL) != 0u; }

    /// Clear every wake-up flag and the Standby flag - the sweep tables
    /// 33 and 34 require before a Standby or Shutdown entry (fact 6).
    static void clear_wakeup_flags() {
        PWR->SCR = PWR_SCR_CSBF | PWR_SCR_CWUF1 | PWR_SCR_CWUF2 |
#if defined(PWR_SCR_CWUF3)
                   PWR_SCR_CWUF3 |
#endif
                   PWR_SCR_CWUF4 |
#if defined(PWR_SCR_CWUF5)
                   PWR_SCR_CWUF5 |
#endif
                   PWR_SCR_CWUF6;
    }

    // ---- SRAM retention and the supply sampler (4.4.3) ----------------------

    /// RRS: keep the SRAM alive from the low-power regulator through a
    /// Standby. Costs current, saves everything.
    static void sram_retention(bool on) {
        PWR->CR3 = on ? (PWR->CR3 | PWR_CR3_RRS) : (PWR->CR3 & ~PWR_CR3_RRS);
    }
    static bool sram_retention() { return (PWR->CR3 & PWR_CR3_RRS) != 0u; }

    /// ENB_ULP: sample the supply instead of watching it. The chapter's
    /// own Caution is the whole story - "if the supply voltage drops
    /// below the minimum operating condition between two samples, the
    /// reset condition is missed and no reset is generated" - so this is
    /// offered and never set by anything here.
    static void sampled_supply_monitor(bool on) {
        PWR->CR3 = on ? (PWR->CR3 | PWR_CR3_ENB_ULP)
                      : (PWR->CR3 & ~PWR_CR3_ENB_ULP);
    }

    // ---- the programmable voltage detector (4.2.2) ---------------------------

    static bool pvd_config(const PvdConfig& c) {
        if (!pvd_config_valid(c)) {
            return false;
        }
        PWR->CR2 = (PWR->CR2 & ~(PWR_CR2_PVDRT_Msk | PWR_CR2_PVDFT_Msk)) |
                   ((static_cast<uint32_t>(c.rising) << PWR_CR2_PVDRT_Pos) &
                    PWR_CR2_PVDRT_Msk) |
                   ((static_cast<uint32_t>(c.falling) << PWR_CR2_PVDFT_Pos) &
                    PWR_CR2_PVDFT_Msk);
        return true;
    }

    static void pvd_enable(bool on) {
        PWR->CR2 = on ? (PWR->CR2 | PWR_CR2_PVDE) : (PWR->CR2 & ~PWR_CR2_PVDE);
    }
    static bool pvd_enabled() { return (PWR->CR2 & PWR_CR2_PVDE) != 0u; }

    /// PWR_SR2.PVDO: 1 means VDD is BELOW the threshold in force.
    static bool pvd_below() { return (PWR->SR2 & PWR_SR2_PVDO) != 0u; }

    /// The EXTI line the detector's output raises - a CONFIGURABLE line
    /// (table 65), so a sense must be chosen before anything is pending.
    /// Published here per the stratum's rule that a peripheral owns its
    /// own line number.
    static constexpr uint8_t pvd_exti_line = 16;

    /// The DAC supply monitor (PVMENDAC / PVMODAC): 1.8 V on VDDA. A
    /// second detector with no threshold to choose.
    static void dac_supply_monitor(bool on) {
        PWR->CR2 = on ? (PWR->CR2 | PWR_CR2_PVMEN_DAC)
                      : (PWR->CR2 & ~PWR_CR2_PVMEN_DAC);
    }
    static bool dac_supply_low() { return (PWR->SR2 & PWR_SR2_PVMO_DAC) != 0u; }

    /// Does this part carry the second I/O supply and its monitor?
    static constexpr bool has_vddio2 = pwr_vddio2_present();

    // ---- the Standby pull configuration (fact 7) ----------------------------

    /// PWR_CR3.APC - whether the two registers below are applied at all.
    static void apply_pulls(bool on) {
        PWR->CR3 = on ? (PWR->CR3 | PWR_CR3_APC) : (PWR->CR3 & ~PWR_CR3_APC);
    }
    static bool apply_pulls() { return (PWR->CR3 & PWR_CR3_APC) != 0u; }

    /**
     * A pad's pull for Standby and Shutdown. `up` and `down` are the two
     * register bits; 4.4.8 makes pull-up conditional on the pull-down
     * bit being clear, so asking for both is a contradiction and is
     * refused rather than silently resolved.
     *
     * False for a port this part does not bond, a pin past 15, or that
     * contradiction.
     */
    static bool standby_pull(char port, uint8_t pin, bool up, bool down) {
        if (pin > 15u || (up && down)) {
            return false;
        }
        volatile uint32_t* pu = pwr_pullup_reg(port);
        volatile uint32_t* pd = pwr_pulldown_reg(port);
        if (pu == nullptr || pd == nullptr) {
            return false;
        }
        const uint32_t bit = 1UL << pin;
        *pd = down ? (*pd | bit) : (*pd & ~bit);
        *pu = up ? (*pu | bit) : (*pu & ~bit);
        return true;
    }

    // ---- entering a mode ----------------------------------------------------

    /**
     * Arm `m` and stop, here and now - the deliberate one-shot for the
     * modes a SleepSite will not touch (Standby and Shutdown, which come
     * back through the reset vector) and for a program with no kernel
     * under it.
     *
     * WHAT IT DOES BEYOND arm(): the flag sweep tables 33 and 34 demand
     * (fact 6), a DSB so the posted writes retire, and the WFI. It does
     * NOT clear the EXTI pending bits or the peripheral flags that also
     * block a Stop entry (fact 5) - those belong to whoever set them,
     * and a Stop that silently does not happen is measured by time and
     * not by a flag.
     *
     * NOT [[noreturn]] even for Standby: a Standby whose entry
     * conditions were not met falls through, and a caller that means to
     * end here says so with a spin of its own.
     */
    static void enter(PwrMode m) {
        if (!arm(m)) {
            return;
        }
        if (pwr_mode_resets(m)) {
            clear_wakeup_flags();
        }
        __DSB();
        __WFI();
    }
};

} // namespace brio
