/*
 * pwr.hpp
 *
 * The power controller (RM0090 ch. 5, RM0390 ch. 5, RM0383 ch. 5), whole:
 * the block's bus clock, the main regulator's VOLTAGE SCALE and the
 * OVER-DRIVE pair - the part the CLOCK TASK needs, and the reason this
 * file was opened - and then the Sleep/Stop/Standby ladder with the
 * regulator variants that price a Stop, the wake-up pins, the
 * programmable voltage detector, the backup regulator, and the three
 * DBGMCU bits that decide whether a low-power mode happens at all under
 * a probe. One chapter, one owner; util/power.hpp's SITES over it are
 * stm32f4/sleep.hpp's.
 *
 * THE MODE IS TWO REGISTERS IN TWO PLACES, and that is the first thing to
 * know. The Cortex-M4 has one bit, SCB->SCR.SLEEPDEEP, which says "the
 * next WFI is a deep sleep"; PWR_CR.PDDS says WHICH deep sleep (Stop or
 * Standby) and PWR_CR's LPDS, FPDS, MRLVDS/LPLVDS and UDEN say what a
 * Stop costs to leave. Neither register is the other's, and both must
 * agree before a WFI does anything but Sleep. `Pwr` owns them BOTH -
 * SLEEPDEEP is written here and nowhere else in this stratum, so that
 * "what is armed" is one question with one answer (`Pwr::mode()`), and so
 * that stm32f4/platform.hpp's idle() can stay what it is: a DSB, a WFI
 * and an unmask that take whatever somebody else armed.
 *
 * SIX FACTS OF THIS CHAPTER, besides the regulator's own below.
 *
 * 1. THREE MODES, AND ONLY TWO OF THEM ARE A LADDER A PROGRAM CAN RESUME
 *    FROM (5.3). Sleep stops the CPU clock and nothing else; Stop stops
 *    every clock in the 1.2 V domain, disables the PLLs, the HSI and the
 *    HSE, and KEEPS SRAM AND REGISTERS; Standby powers the 1.2 V domain
 *    off and comes back THROUGH THE RESET VECTOR with everything but the
 *    backup domain and PWR_CSR lost. That last sentence is why
 *    stm32f4/sleep.hpp's site maps no rung to Standby - see its header.
 *
 * 2. WHAT COMES BACK FROM A STOP IS NOT WHAT WENT IN. 5.3.5: "When
 *    exiting Stop mode by issuing an interrupt or a wakeup event, the HSI
 *    RC oscillator is selected as system clock"; the PLL and the HSE are
 *    off, the regulator is back at scale 3 ("when the microcontroller
 *    enters in Stop mode, the voltage scale 3 is automatically selected"),
 *    over-drive is off ("entering Stop mode disables the Over-drive mode,
 *    as well as the PLL") and any under-drive is off too. A program
 *    running at 180 MHz therefore resumes at 16 MHz with a SysTick reload
 *    and a USART divisor meant for eleven times that - a fact of the
 *    chapter and not a defect, and what stm32f4/sleep.hpp's site answers
 *    by re-running the clock task in the first thing that executes after
 *    the wake. Everything OUTSIDE the clock tree survives: the flash
 *    latency, the APB prescalers, every peripheral's registers and the
 *    whole of SRAM.
 *
 * 3. A STOP IS ONLY ENTERED IF NOTHING IS PENDING. Table 18's own note:
 *    "all EXTI Line pending bits ..., all peripheral interrupts pending
 *    bits, the RTC Alarms ..., RTC wakeup, RTC tamper, and RTC time stamp
 *    flags, must be reset. Otherwise, the Stop mode entry procedure is
 *    ignored and program execution continues" - and the Cortex's WFI is
 *    itself a no-op with an interrupt pending. So a Stop that does not
 *    happen is NOT an error condition anywhere in the silicon: the
 *    instruction simply falls through. Everything that judges a sleep in
 *    this stratum therefore judges it by TIME ELAPSED and never by a flag.
 *
 * 4. STANDBY NEEDS ITS OWN FLAG CLEAR TO BE ENTERED AT ALL (table 19):
 *    WUF in PWR_CSR, and the RTC flag matching whichever RTC event is
 *    meant to end the sleep. A stale flag from a previous life is an
 *    entry that silently does not happen - fact 3 again, with a second
 *    cause. AND THERE IS AN ERRATUM ON TOP OF IT: ES0298 2.2.4 (ES0206
 *    2.2.4, ES0287 2.2.4), "the various wake-up sources are logically
 *    OR-ed in front of the rising-edge detector"; a source held high while
 *    CWUF clears the flag "may mask further wake-up events on the input of
 *    the edge detector", and the device then never wakes. The workaround
 *    is the erratum's own four steps and `prepare_standby()` is them:
 *    disable every wake-up source, clear the flags, re-enable, enter.
 *
 * 5. THERE IS ONE WAKE-UP FLAG FOR EVERY SOURCE. Unlike the STM32G0's
 *    per-pin WUFx, this family has a single WUF raised by any WKUP pin
 *    OR any of the RTC's five events (5.4.2), so it says THAT the device
 *    was woken and never BY WHAT. SBF, beside it, says the device was in
 *    Standby and is cleared only by a power-on reset or by CSBF - which
 *    is what lets the boot after a Standby tell itself apart from the
 *    boot after a reset.
 *
 * 6. THE DEBUG BITS DECIDE WHETHER A LOW-POWER MODE HAPPENS. DBGMCU_CR's
 *    DBG_SLEEP, DBG_STOP and DBG_STANDBY (33.16.1) keep FCLK and HCLK
 *    alive through the mode so a probe can still reach the core; the
 *    register "is asynchronously reset by the PORESET (and not the system
 *    reset)", and OpenOCD's own stm32f4x.cfg writes all three at every
 *    connection - so a board that has seen a probe holds them without any
 *    program having asked. With DBG_STOP set the clocks are the internal
 *    RC's rather than none, and ES0298 2.2.1 adds the consequence that
 *    matters here: "if the SysTick timer interrupt is enabled during the
 *    Stop mode debug ..., it wakes up the system from Stop mode". A Stop
 *    that looks like it lasts a millisecond is a debugged board and not a
 *    silicon fact. The three verbs below read and clear the bits, no gate
 *    needed - DBGMCU is on the external private peripheral bus.
 *
 * WHAT IS NOT HERE. PWR_CR.DBP, the backup domain's write gate, is
 * stm32f4/rtc.hpp's `RtcDomain::unlock()`: one register bit, one owner,
 * and the owner is the chapter whose registers it unlocks. The BOR
 * levels are option bytes and belong to the flash chapter's provisioning
 * verb, not to a register here.
 *
 * WHY THE SCALE IS A CLOCK QUESTION. RM0090 5.1.4: the regulator's
 * output "can be scaled by software to different voltage values (scale
 * 1, scale 2, and scale 3 ...). The scale can be modified only when
 * the PLL is OFF and the HSI or HSE clock source is selected as system
 * clock source. The new value programmed is active only when the PLL is
 * ON." Each scale carries a HCLK ceiling (stm32f4/device_tables.hpp's
 * ladder), and the top of the ladder - 168 or 180 MHz on the parts
 * that reach it - is reached only in OVER-DRIVE, entered by a sequence
 * that belongs between the PLL's start and the switch to it (5.1.4,
 * "Entering Over-drive mode"): ODEN then wait ODRDY, ODSWEN then wait
 * ODSWRDY - the system is stalled during the switch. Entering Stop
 * disables the over-drive mode and the PLL; whoever brings the part
 * back does both again.
 *
 * ONE VOS BIT OR TWO. The F405/F407 class has a single VOS bit (scale 2
 * and scale 1, PWR_CR bit 14); every later part has two (scales 3, 2
 * and 1). The reserve says which (pwr_vos_two_bits()), and the code
 * for the scale is written once against that fact.
 *
 * THE OVER-DRIVE VERBS ARE PREPROCESSOR-GUARDED, not if-constexpr'd:
 * `Pwr` is a plain struct and a discarded branch of a non-template is
 * still checked, so a header without ODEN would not compile the name.
 * The one fact the guards ask - is PWR_CR_ODEN declared? - is the
 * reserve's own probe, so the two can never disagree.
 *
 * THE BLOCK IS BEHIND A GATE. PWR sits on APB1 behind RCC_APB1ENR.PWREN,
 * clear at reset; a register read through the closed gate answers zero
 * and a write is dropped in silence. Every verb here that touches a
 * register opens the gate first (bus_clock(true), idempotent), the
 * STM32G0 lesson.
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"

namespace brio {

/// The main regulator's scale in Run mode. The numbers are the
/// chapter's: scale 1 is the highest voltage (and the highest HCLK),
/// scale 3 the lowest.
enum class VoltageScale : uint8_t { scale3 = 3, scale2 = 2, scale1 = 1 };

/// What a following WFI (or WFE) takes: SLEEPDEEP and PDDS as the pair
/// they are. `sleep` is SLEEPDEEP = 0, which is what the other two have
/// in common - PDDS only means anything with SLEEPDEEP set.
enum class PwrMode : uint8_t {
    sleep = 0,     ///< SLEEPDEEP = 0: the CPU clock stops and nothing else
    stop = 1,      ///< SLEEPDEEP = 1, PDDS = 0: every 1.2 V clock stops, SRAM kept
    standby = 2,   ///< SLEEPDEEP = 1, PDDS = 1: the domain off; wakes through the reset vector
};

/// Does this mode power the 1.2 V domain off - i.e. does the program come
/// back through the reset vector instead of the instruction after the
/// WFI? The one distinction that matters to anything above this file.
constexpr bool pwr_mode_resets(PwrMode m) { return m == PwrMode::standby; }

/// Does this mode stop the 1.2 V clocks - SysTick, every TIM, the buses?
constexpr bool pwr_mode_stops_clocks(PwrMode m) { return m != PwrMode::sleep; }

/// Which regulator supplies the 1.2 V domain through a Stop (PWR_CR.LPDS).
/// The main one is the fastest to leave, the low-power one the cheapest to
/// hold; the datasheet's table of wake-up times prices both.
enum class StopRegulator : uint8_t { main = 0, low_power = 1 };

/**
 * What a Stop costs, as the four bits that price it (table 17).
 *
 *  regulator         LPDS: the main regulator, or the low-power one
 *  flash_power_down  FPDS: the flash array in power-down through the Stop
 *  low_voltage       MRLVDS or LPLVDS, WHICHEVER MATCHES THE REGULATOR -
 *                    RM0383 spells the pair LVDS and RM0390 spells the
 *                    same two bits UDS; the header uses the first
 *                    spelling, and one flag here writes the right one
 *  under_drive       UDEN = 11: the 1.2 V domain in REDUCED LEAKAGE. Only
 *                    legal on top of `low_voltage` (table 17's under-drive
 *                    rows all carry MRUDS or LPUDS), and only where the
 *                    part has the field at all
 *
 * The default is the cheapest Stop to LEAVE and not the cheapest to hold:
 * a sleep site names the deeper variants explicitly.
 */
struct StopConfig {
    StopRegulator regulator = StopRegulator::main;
    bool flash_power_down = false;
    bool low_voltage = false;
    bool under_drive = false;
};

/// The two rules of table 17 that a register cannot enforce: the
/// low-voltage pair and the under-drive field must exist on this part,
/// and under-drive is a MODIFIER of the low-voltage mode, never a mode of
/// its own.
constexpr bool stop_config_valid(const StopConfig& c) {
    if (c.low_voltage && !pwr_has_low_voltage_stop()) {
        return false;
    }
    if (c.under_drive && (!pwr_has_under_drive() || !c.low_voltage)) {
        return false;
    }
    return true;
}

/// PWR as a monostate resource.
struct Pwr {
    Pwr() = delete;

    static constexpr uint32_t ready_spins = 1'000'000u;

    /// Open (or close) the block's APB1 gate, with the readback that
    /// covers the enable's propagation (ES0206 2.2.7 and its twins: a
    /// peripheral register written right after its clock enable may not
    /// take the store; the readback is the recommended dummy access).
    static void bus_clock(bool on) {
        if (on) {
            RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        } else {
            RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
        }
        (void)RCC->APB1ENR;
    }
    static bool bus_clock() { return (RCC->APB1ENR & RCC_APB1ENR_PWREN) != 0u; }

    /// The two registers as they stand, for a program that wants to print
    /// what it found rather than ask thirty questions.
    static uint32_t cr() {
        bus_clock(true);
        return PWR->CR;
    }
    static uint32_t csr() {
        bus_clock(true);
        return PWR->CSR;
    }

    /// Whether `s` exists on this part: scale 3 does not on the one-bit
    /// VOS class.
    static constexpr bool scale_exists(VoltageScale s) {
        return pwr_vos_two_bits() || s != VoltageScale::scale3;
    }

    /// Program the regulator scale. The chapter's precondition - PLL off,
    /// HSI or HSE as SYSCLK - is the caller's (the clock task honours
    /// it); the value takes effect when the PLL turns on. False when the
    /// scale does not exist on this part; nothing written then.
    static bool scale(VoltageScale s) {
        if (!scale_exists(s)) {
            return false;
        }
        bus_clock(true);
        if constexpr (pwr_vos_two_bits()) {
            PWR->CR = (PWR->CR & ~PWR_CR_VOS_Msk) |
                      (static_cast<uint32_t>(vos_code(s)) << PWR_CR_VOS_Pos);
        } else {
            // One bit: set = scale 1, clear = scale 2 (RM0090 5.4.1).
            if (s == VoltageScale::scale1) {
                PWR->CR |= PWR_CR_VOS;
            } else {
                PWR->CR &= ~PWR_CR_VOS;
            }
        }
        return true;
    }

    /// The scale in force (or programmed, PLL still off).
    static VoltageScale scale() {
        bus_clock(true);
        if constexpr (pwr_vos_two_bits()) {
            const uint8_t code = static_cast<uint8_t>((PWR->CR & PWR_CR_VOS_Msk) >> PWR_CR_VOS_Pos);
            return scale_of(code);
        } else {
            return (PWR->CR & PWR_CR_VOS) != 0u ? VoltageScale::scale1 : VoltageScale::scale2;
        }
    }

    /// PWR_CSR.VOSRDY: the regulator has settled at the programmed
    /// scale - after the PLL is on and the value is in force.
    static bool scale_ready() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_VOSRDY) != 0u;
    }

    /// Whether this part has the over-drive mode at all (the reserve's
    /// PWR_CR_ODEN probe).
    static constexpr bool has_over_drive() { return pwr_has_over_drive(); }

    /// Enter over-drive: ODEN and wait for ODRDY, ODSWEN and wait for
    /// ODSWRDY (RM0090 5.1.4 steps 3..5). To be called with the PLL
    /// configured and ON but not yet the system clock, and BEFORE the
    /// peripheral clocks are enabled ("during the Over-drive switch
    /// activation, no peripheral clocks should be enabled"). Bounded
    /// waits: false when either flag does not rise. Always false, and
    /// nothing written, on a part without the mode.
    static bool over_drive_enter() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        PWR->CR |= PWR_CR_ODEN;
        if (!wait_csr(PWR_CSR_ODRDY)) {
            return false;
        }
        PWR->CR |= PWR_CR_ODSWEN;
        return wait_csr(PWR_CSR_ODSWRDY);
#else
        return false;
#endif
    }

    /// Leave over-drive, both bits at once, and WAIT FOR ODSWRDY TO FALL -
    /// 5.1.4's sequence 1 complete ("reset simultaneously the ODEN and the
    /// ODSW bits ...; wait for the ODWRDY flag of PWR_CSR to be reset").
    /// The caller has SYSCLK on HSI or HSE, per the chapter: the bits are
    /// writable only there. False when the flag never fell within the
    /// bound, and true - with nothing written - on a part that has no
    /// over-drive to leave.
    static bool over_drive_exit() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        PWR->CR &= ~(PWR_CR_ODEN | PWR_CR_ODSWEN);
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if ((PWR->CSR & PWR_CSR_ODSWRDY) == 0u) {
                return true;
            }
        }
        return false;
#else
        return true;
#endif
    }

    /// Whether the regulator is in over-drive right now (ODSWRDY).
    static bool over_drive_active() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_ODSWRDY) != 0u;
#else
        return false;
#endif
    }

    /// PWR_CSR.ODRDY - over-drive is READY (the first half of the entry
    /// sequence); `over_drive_active()` is the second.
    static bool over_drive_ready() {
#if defined(PWR_CR_ODEN)
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_ODRDY) != 0u;
#else
        return false;
#endif
    }

    // ---- the sleep mode (facts 1 and 2) --------------------------------------

    /// The Cortex's SLEEPDEEP, written HERE and nowhere else in this
    /// stratum (see the file header).
    static void deep_sleep(bool on) {
        SCB->SCR = on ? (SCB->SCR | SCB_SCR_SLEEPDEEP_Msk)
                      : (SCB->SCR & ~SCB_SCR_SLEEPDEEP_Msk);
    }
    static bool deep_sleep() { return (SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u; }

    /// SCR.SLEEPONEXIT: the core re-enters the armed mode on every return
    /// from the LAST nested handler instead of resuming thread mode
    /// (5.3.3's third entry path). Offered because the register has it;
    /// NOTHING IN THIS STRATUM SETS IT, because a brio program's idle path
    /// is the kernel loop's and an ISR that never returns to it would
    /// starve every queue. Clear at reset.
    static void sleep_on_exit(bool on) {
        SCB->SCR = on ? (SCB->SCR | SCB_SCR_SLEEPONEXIT_Msk)
                      : (SCB->SCR & ~SCB_SCR_SLEEPONEXIT_Msk);
    }
    static bool sleep_on_exit() { return (SCB->SCR & SCB_SCR_SLEEPONEXIT_Msk) != 0u; }

    /// SCR.SEVONPEND: a WFE is released by any interrupt becoming pending,
    /// "even the disabled ones" (5.3.3). Offered for the same reason and
    /// used by nothing here: brio's idle path is a WFI.
    static void event_on_pending(bool on) {
        SCB->SCR = on ? (SCB->SCR | SCB_SCR_SEVONPEND_Msk)
                      : (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk);
    }
    static bool event_on_pending() { return (SCB->SCR & SCB_SCR_SEVONPEND_Msk) != 0u; }

    /// PWR_CR.PDDS on its own - which deep sleep SLEEPDEEP means.
    static void power_down_deep_sleep(bool on) {
        bus_clock(true);
        PWR->CR = on ? (PWR->CR | PWR_CR_PDDS) : (PWR->CR & ~PWR_CR_PDDS);
    }
    static bool power_down_deep_sleep() {
        bus_clock(true);
        return (PWR->CR & PWR_CR_PDDS) != 0u;
    }

    /**
     * The four bits that price a Stop (table 17), written in one store.
     * False, and NOTHING written, for a configuration this part cannot
     * hold (`stop_config_valid()`); the bits a part has not got are simply
     * absent from the mask, so the same call on a smaller part writes the
     * subset it does have.
     */
    static bool stop_config(const StopConfig& c) {
        if (!stop_config_valid(c)) {
            return false;
        }
        bus_clock(true);
        uint32_t v = PWR->CR & ~(PWR_CR_LPDS | PWR_CR_FPDS | low_voltage_mask() |
                                 under_drive_mask());
        if (c.regulator == StopRegulator::low_power) {
            v |= PWR_CR_LPDS;
        }
        if (c.flash_power_down) {
            v |= PWR_CR_FPDS;
        }
        if (c.low_voltage) {
            v |= c.regulator == StopRegulator::low_power ? lp_low_voltage_mask()
                                                         : mr_low_voltage_mask();
        }
        if (c.under_drive) {
            v |= under_drive_mask();
        }
        PWR->CR = v;
        return true;
    }

    /// What the four bits hold now. The low-voltage flag reports the one
    /// that MATCHES the regulator in force, which is the only one the
    /// silicon consults.
    static StopConfig stop_config() {
        bus_clock(true);
        const uint32_t v = PWR->CR;
        StopConfig c{};
        c.regulator = (v & PWR_CR_LPDS) != 0u ? StopRegulator::low_power : StopRegulator::main;
        c.flash_power_down = (v & PWR_CR_FPDS) != 0u;
        const uint32_t lv = c.regulator == StopRegulator::low_power ? lp_low_voltage_mask()
                                                                    : mr_low_voltage_mask();
        c.low_voltage = lv != 0u && (v & lv) != 0u;
        c.under_drive = under_drive_mask() != 0u && (v & under_drive_mask()) == under_drive_mask();
        return c;
    }

    /**
     * Arm the mode the next WFI will take: SLEEPDEEP and PDDS together,
     * which is the only pair that means anything (the file header).
     *
     * ARMING IS NOT SLEEPING. Nothing stops here; the CPU stops when
     * something executes a WFI, which in a brio program is the kernel's
     * idle path. That split is what lets util/power.hpp's model exist with
     * no new kernel hook, and it is why this verb is named for what it
     * does.
     *
     * Reading it back is `mode()`, and the readback IS the state - there
     * is no shadow variable anywhere in this stratum.
     */
    static bool arm(PwrMode m) {
        bus_clock(true);
        if (m == PwrMode::sleep) {
            deep_sleep(false);
            return true;
        }
        power_down_deep_sleep(m == PwrMode::standby);
        (void)PWR->CR;
        deep_sleep(true);
        return true;
    }

    /// Arm a Stop and price it in one call. False, and nothing armed, for
    /// a configuration this part cannot hold.
    static bool arm(PwrMode m, const StopConfig& c) {
        if (!stop_config(c)) {
            return false;
        }
        return arm(m);
    }

    /// What a following WFI would take. A pure read of the two registers:
    /// SLEEPDEEP clear is Sleep whatever PDDS holds.
    static PwrMode mode() {
        if (!deep_sleep()) {
            return PwrMode::sleep;
        }
        return power_down_deep_sleep() ? PwrMode::standby : PwrMode::stop;
    }

    /// The two stopping instructions, spelled once so that a caller does
    /// not reach for a CMSIS intrinsic of its own. Both retire the posted
    /// writes first (the ARM recommendation).
    static void wait_for_interrupt() {
        __DSB();
        __WFI();
    }
    static void wait_for_event() {
        __DSB();
        __WFE();
    }

    /**
     * Arm `m` and stop, here and now - the deliberate one-shot for the
     * mode a SleepSite will not touch (Standby, which comes back through
     * the reset vector) and for a program with no kernel under it.
     *
     * WHAT IT DOES BEYOND arm(): for Standby, the flag sweep table 19
     * demands and the erratum's re-arming around it (`prepare_standby()`,
     * fact 4), then the instruction. It does NOT clear the EXTI pending
     * bits or the peripheral flags that also block a Stop entry (fact 3) -
     * those belong to whoever set them, and a Stop that silently does not
     * happen is measured by time and not by a flag.
     *
     * NOT [[noreturn]] even for Standby: a Standby whose entry conditions
     * were not met falls through, and a caller that means to end here says
     * so with a spin of its own.
     */
    static void enter(PwrMode m, bool use_wfe = false) {
        (void)arm(m);
        if (pwr_mode_resets(m)) {
            prepare_standby();
        }
        if (use_wfe) {
            wait_for_event();
        } else {
            wait_for_interrupt();
        }
    }

    // ---- wake-up pins (5.4.2) -------------------------------------------------

    /// How many WKUPx pins this part bonds, and whether `n` (1-based) is
    /// one of them - the reserve's reading of the header's EWUP masks.
    static constexpr uint8_t wakeup_pin_count = pwr_wakeup_pin_count();
    static constexpr bool wakeup_pin_present(uint8_t n) {
        return pwr_wakeup_pin_mask(n) != 0u;
    }
    /// Which pad the pin is, where a datasheet on the desk says so
    /// (`known == false` otherwise).
    static constexpr PwrWakeupPad wakeup_pad(uint8_t n) { return pwr_wakeup_pad(n); }

    /**
     * Enable or disable WKUPn. There is no polarity to choose on this
     * family: 5.4.2 says the pin is "forced in input pull down
     * configuration" and a RISING edge is the wake - so a program does not
     * configure the pad at all, and must not drive it.
     *
     * THE NOTE THAT COSTS A SLEEP: "an additional wakeup event is detected
     * if the WKUP pin is enabled ... when the WKUP pin level is already
     * high". The flag is therefore cleared AFTER the enable, which is also
     * the shape ES0298 2.2.4's workaround wants; a pin held high still
     * raises it again at once, which is the silicon and not a bug.
     *
     * False for a pin this part does not bond.
     */
    static bool wakeup_pin(uint8_t n, bool on) {
        const uint32_t bit = pwr_wakeup_pin_mask(n);
        if (bit == 0u) {
            return false;
        }
        bus_clock(true);
        PWR->CSR = on ? (PWR->CSR | bit) : (PWR->CSR & ~bit);
        clear_wakeup_flag();
        return true;
    }

    static bool wakeup_pin_enabled(uint8_t n) {
        const uint32_t bit = pwr_wakeup_pin_mask(n);
        if (bit == 0u) {
            return false;
        }
        bus_clock(true);
        return (PWR->CSR & bit) != 0u;
    }

    /// Every wake-up pin at once - what the erratum's step 1 and step 3
    /// need, and what a program that only wants the RTC to wake it calls
    /// once at boot.
    static void wakeup_pins(bool on) {
        for (uint8_t n = 1; n <= wakeup_pin_count; ++n) {
            (void)wakeup_pin(n, on);
        }
    }

    // ---- the flags (fact 5) ---------------------------------------------------

    /// PWR_CSR.WUF: SOMETHING woke the device - a WKUP pin, or one of the
    /// RTC's five events. One flag for every source on this family, so it
    /// never says which. Cleared by a system reset or by CWUF.
    static bool wakeup_flag() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_WUF) != 0u;
    }
    /// CWUF: "clear the WUF wakeup flag after 2 system clock cycles"
    /// (5.4.1), so the readback that follows a clear is two cycles away -
    /// the spin here is those cycles and no policy.
    static void clear_wakeup_flag() {
        bus_clock(true);
        PWR->CR = PWR->CR | PWR_CR_CWUF;
        for (uint8_t i = 0; i < 8u && (PWR->CSR & PWR_CSR_WUF) != 0u; ++i) {
        }
    }

    /// PWR_CSR.SBF: the device HAS BEEN in Standby. Not cleared by a
    /// system reset - "cleared only by a POR/PDR ... or by setting the
    /// CSBF bit" - which is what lets the boot after a Standby tell itself
    /// apart from the boot after a reset.
    static bool standby_flag() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_SBF) != 0u;
    }
    static void clear_standby_flag() {
        bus_clock(true);
        PWR->CR = PWR->CR | PWR_CR_CSBF;
    }

    /// Both, the sweep table 19 requires before a Standby entry.
    static void clear_wakeup_flags() {
        bus_clock(true);
        PWR->CR = PWR->CR | PWR_CR_CSBF | PWR_CR_CWUF;
        for (uint8_t i = 0; i < 8u && (PWR->CSR & PWR_CSR_WUF) != 0u; ++i) {
        }
    }

    /**
     * ES0298 2.2.4's four steps, in the erratum's own order: disable every
     * wake-up source, clear the flags, re-enable what was enabled, and
     * leave the caller to enter. Without it a source held high while CWUF
     * lands "may mask further wake-up events on the input of the edge
     * detector" and the device never wakes.
     *
     * Only the PINS are re-enabled here: the RTC's events are the RTC
     * chapter's enables, and 5.3.7's own "safe RTC alternate function
     * wakeup flag clearing sequence" is the same dance around them -
     * stm32f4/sleep.hpp's timed site performs it for the wake-up timer.
     */
    static void prepare_standby() {
        uint8_t was = 0;
        for (uint8_t n = 1; n <= wakeup_pin_count; ++n) {
            if (wakeup_pin_enabled(n)) {
                was = static_cast<uint8_t>(was | (1u << (n - 1u)));
            }
            (void)wakeup_pin(n, false);
        }
        clear_wakeup_flags();
        for (uint8_t n = 1; n <= wakeup_pin_count; ++n) {
            if ((was & (1u << (n - 1u))) != 0u) {
                (void)wakeup_pin(n, true);
            }
        }
    }

    // ---- the programmable voltage detector (5.2.3) ----------------------------

    /// The eight PLS codes in MILLIVOLTS, the reference manual's table -
    /// which differs by part class, so a class whose manual is not on the
    /// desk answers 0 and `pvd_levels_known` is false. The CODE is
    /// writable on every part all the same: the field is the same three
    /// bits everywhere.
    static constexpr bool pvd_levels_known = pwr_pvd_levels().known;
    static constexpr uint16_t pvd_level_mv(uint8_t code) {
        return code < 8u ? pwr_pvd_levels().mv[code] : 0u;
    }

    /// PLS[2:0]. False for a code past the field; nothing written then.
    static bool pvd_level(uint8_t code) {
        if (code > 7u) {
            return false;
        }
        bus_clock(true);
        PWR->CR = (PWR->CR & ~PWR_CR_PLS_Msk) |
                  ((static_cast<uint32_t>(code) << PWR_CR_PLS_Pos) & PWR_CR_PLS_Msk);
        return true;
    }
    static uint8_t pvd_level() {
        bus_clock(true);
        return static_cast<uint8_t>((PWR->CR & PWR_CR_PLS_Msk) >> PWR_CR_PLS_Pos);
    }

    static void pvd_enable(bool on) {
        bus_clock(true);
        PWR->CR = on ? (PWR->CR | PWR_CR_PVDE) : (PWR->CR & ~PWR_CR_PVDE);
    }
    static bool pvd_enabled() {
        bus_clock(true);
        return (PWR->CR & PWR_CR_PVDE) != 0u;
    }

    /// PWR_CSR.PVDO: 1 means VDD is BELOW the threshold in force. "Valid
    /// only if PVD is enabled by the PVDE bit", and 0 after a Standby or a
    /// reset until PVDE is set again.
    static bool pvd_below() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_PVDO) != 0u;
    }

    /// The EXTI line the detector's output raises - a CONFIGURABLE line on
    /// this family, so a sense must be chosen before anything is pending.
    /// The line's registers are exti.hpp's; this file owns the NUMBER.
    static constexpr uint8_t pvd_exti_line = pwr_pvd_exti_line;

    // ---- the backup regulator (5.1.2) -----------------------------------------

    /// Whether this part has the 4 KB backup SRAM the regulator keeps
    /// alive. PWR_CSR carries BRE and BRR on EVERY part of the pack, the
    /// F401, F410, F411 and F412/F413 classes included, where there is no
    /// backup SRAM behind them - the reserve reads the RCC's gate instead,
    /// which is the honest question.
    static constexpr bool has_backup_sram = pwr_backup_sram_clock_mask() != 0u;

    /// RCC_AHB1ENR.BKPSRAMEN, the array's own bus clock (5.1.2's second
    /// access recipe). Written through the CMSIS pointer with the 2.2.7
    /// readback, as bus_clock() above does: clock.hpp includes THIS file,
    /// so `Rcc` is not reachable from here. False on a part without the
    /// array.
    static bool backup_sram_clock(bool on) {
        if constexpr (has_backup_sram) {
            constexpr uint32_t m = pwr_backup_sram_clock_mask();
            RCC->AHB1ENR = on ? (RCC->AHB1ENR | m) : (RCC->AHB1ENR & ~m);
            (void)RCC->AHB1ENR;
            return true;
        } else {
            (void)on;
            return false;
        }
    }
    static bool backup_sram_clock() {
        constexpr uint32_t m = pwr_backup_sram_clock_mask();
        return m != 0u && (RCC->AHB1ENR & m) != 0u;
    }

    /**
     * PWR_CSR.BRE: keep the backup SRAM's content through Standby and
     * VBAT. THE BIT IS BEHIND PWR_CR.DBP - 5.4.1 lists "the BRE bit of the
     * PWR_CSR register" among what DBP protects - which is
     * stm32f4/rtc.hpp's RtcDomain::unlock(), the one owner of that bit; a
     * write here with the domain locked is dropped in silence, and
     * `backup_regulator()` read back says so.
     *
     * "Once set, the application must wait that the Backup Regulator Ready
     * flag (BRR) is set": the wait is here and bounded, and false means
     * the flag never came.
     */
    static bool backup_regulator(bool on) {
        bus_clock(true);
        PWR->CSR = on ? (PWR->CSR | PWR_CSR_BRE) : (PWR->CSR & ~PWR_CSR_BRE);
        if (!on) {
            return true;
        }
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if ((PWR->CSR & PWR_CSR_BRR) != 0u) {
                return true;
            }
        }
        return false;
    }
    static bool backup_regulator() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_BRE) != 0u;
    }
    static bool backup_regulator_ready() {
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_BRR) != 0u;
    }

    // ---- under-drive (table 17) -----------------------------------------------

    static constexpr bool has_under_drive = pwr_has_under_drive();

    /// PWR_CSR.UDRDY: the device HAS ENTERED and left a Stop in under-drive
    /// - "not set as long as the MCU has not entered stop mode yet". The
    /// field is rc_w1 and reads 11 when it stands.
    static bool under_drive_flag() {
#if defined(PWR_CSR_UDRDY)
        bus_clock(true);
        return (PWR->CSR & PWR_CSR_UDRDY) == PWR_CSR_UDRDY;
#else
        return false;
#endif
    }
    static void clear_under_drive_flag() {
#if defined(PWR_CSR_UDRDY)
        bus_clock(true);
        PWR->CSR = PWR->CSR | PWR_CSR_UDRDY;
#endif
    }

    // ---- stopping the flash while the system runs (5.4.1) ---------------------
    //
    // FISSR and FMSSR are the two bits of this chapter this stratum
    // OFFERS AND NEVER SETS, and the manual says why: "this bit could not
    // be set while executing with the Flash itself. It should be done with
    // a specific routine executed from RAM." Nothing in brio runs from
    // RAM on this family, so a program that wants them owns the .ram_text
    // routine and these two verbs; docs/stm32f4/pwr.md carries the fact.

    static constexpr bool has_flash_stop_while_run = pwr_has_flash_stop_while_run();

    static bool flash_interface_stop_in_run(bool on) {
#if defined(PWR_CR_FISSR)
        bus_clock(true);
        PWR->CR = on ? (PWR->CR | PWR_CR_FISSR) : (PWR->CR & ~PWR_CR_FISSR);
        return true;
#else
        (void)on;
        return false;
#endif
    }
    static bool flash_interface_stop_in_run() {
#if defined(PWR_CR_FISSR)
        bus_clock(true);
        return (PWR->CR & PWR_CR_FISSR) != 0u;
#else
        return false;
#endif
    }
    static bool flash_stop_in_run(bool on) {
#if defined(PWR_CR_FMSSR)
        bus_clock(true);
        PWR->CR = on ? (PWR->CR | PWR_CR_FMSSR) : (PWR->CR & ~PWR_CR_FMSSR);
        return true;
#else
        (void)on;
        return false;
#endif
    }
    static bool flash_stop_in_run() {
#if defined(PWR_CR_FMSSR)
        bus_clock(true);
        return (PWR->CR & PWR_CR_FMSSR) != 0u;
#else
        return false;
#endif
    }

    /// PWR_CR.ADCDC1, whose meaning is AN4073's and not this manual's -
    /// "refer to AN4073 for details on how to use this bit", legal only at
    /// 2.7..3.6 V with the prefetch off. Offered, never set here.
    static constexpr bool has_adc_dc1 = pwr_has_adcdc1();
    static bool adc_dc1(bool on) {
#if defined(PWR_CR_ADCDC1)
        bus_clock(true);
        PWR->CR = on ? (PWR->CR | PWR_CR_ADCDC1) : (PWR->CR & ~PWR_CR_ADCDC1);
        return true;
#else
        (void)on;
        return false;
#endif
    }
    static bool adc_dc1() {
#if defined(PWR_CR_ADCDC1)
        bus_clock(true);
        return (PWR->CR & PWR_CR_ADCDC1) != 0u;
#else
        return false;
#endif
    }

    // ---- the debugger's low-power bits (fact 6) -------------------------------
    //
    // DBGMCU_CR is on the EXTERNAL private peripheral bus at 0xE0042004
    // and needs no clock gate (unlike the STM32G0's, which sits behind
    // RCC_APBENR1.DBGEN): the reads and writes below are plain.

    static bool debug_in_sleep() { return (DBGMCU->CR & DBGMCU_CR_DBG_SLEEP) != 0u; }
    static void debug_in_sleep(bool on) {
        DBGMCU->CR = on ? (DBGMCU->CR | DBGMCU_CR_DBG_SLEEP) : (DBGMCU->CR & ~DBGMCU_CR_DBG_SLEEP);
    }
    static bool debug_in_stop() { return (DBGMCU->CR & DBGMCU_CR_DBG_STOP) != 0u; }
    static void debug_in_stop(bool on) {
        DBGMCU->CR = on ? (DBGMCU->CR | DBGMCU_CR_DBG_STOP) : (DBGMCU->CR & ~DBGMCU_CR_DBG_STOP);
    }
    static bool debug_in_standby() { return (DBGMCU->CR & DBGMCU_CR_DBG_STANDBY) != 0u; }
    static void debug_in_standby(bool on) {
        DBGMCU->CR = on ? (DBGMCU->CR | DBGMCU_CR_DBG_STANDBY)
                        : (DBGMCU->CR & ~DBGMCU_CR_DBG_STANDBY);
    }

    /// All three at once - what a program that means to MEASURE a sleep
    /// calls before it does, and what a suite prints as it FOUND them.
    static void debug_low_power(bool on) {
        constexpr uint32_t m = DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;
        DBGMCU->CR = on ? (DBGMCU->CR | m) : (DBGMCU->CR & ~m);
    }

private:
    /// The low-voltage-in-deep-sleep bits, empty where the part has none.
    /// RM0383 spells them MRLVDS/LPLVDS and RM0390 MRUDS/LPUDS; ST's
    /// headers use the first spelling wherever the bits exist.
    static constexpr uint32_t mr_low_voltage_mask() {
#if defined(PWR_CR_MRLVDS)
        return PWR_CR_MRLVDS;
#else
        return 0u;
#endif
    }
    static constexpr uint32_t lp_low_voltage_mask() {
#if defined(PWR_CR_LPLVDS)
        return PWR_CR_LPLVDS;
#else
        return 0u;
#endif
    }
    static constexpr uint32_t low_voltage_mask() {
        return mr_low_voltage_mask() | lp_low_voltage_mask();
    }
    static constexpr uint32_t under_drive_mask() {
#if defined(PWR_CR_UDEN)
        return PWR_CR_UDEN;
#else
        return 0u;
#endif
    }

    /// The two-bit VOS encoding (RM0090 5.4.1 for the F42x/F43x, RM0390,
    /// RM0383): 01 scale 3, 10 scale 2, 11 scale 1; 00 is Reserved and
    /// reads as scale 3.
    static constexpr uint8_t vos_code(VoltageScale s) {
        switch (s) {
            case VoltageScale::scale3: return 1;
            case VoltageScale::scale2: return 2;
            case VoltageScale::scale1: return 3;
        }
        return 3;
    }
    static constexpr VoltageScale scale_of(uint8_t code) {
        switch (code) {
            case 2: return VoltageScale::scale2;
            case 3: return VoltageScale::scale1;
            default: return VoltageScale::scale3;
        }
    }
    static bool wait_csr(uint32_t mask) {
        for (uint32_t spins = 0; spins < ready_spins; ++spins) {
            if ((PWR->CSR & mask) != 0u) {
                return true;
            }
        }
        return false;
    }
};

} // namespace brio
