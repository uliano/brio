/*
 * sleep.hpp
 *
 * PM, the Power Manager (DS60001479M ch. 19) - the whole of it, which is
 * two registers - plus `SamSleepSite`, the adapter that puts
 * `util/power.hpp`'s depth ladder on this silicon.
 *
 * The MECHANISM only: which sleep mode the next WFI will take, what the
 * regulator and the RAM do while the CPU is stopped, and the one
 * erratum that makes standby entry a hazard. The POLICY - when a program
 * may stop its clocks - is util/power.hpp's, and the design note at the
 * end of this comment says why the two are separate files.
 *
 * ## Three modes, and one of them is not a rung
 *
 * SLEEPCFG.SLEEPMODE has three implemented codes (19.8.1); 0x1, 0x3 and
 * 0x5..0x7 are Reserved and `sleep_mode_valid()` refuses them.
 *
 * IDLE0 (0x0) - the RESET value, so it is also what a machine that has
 * never touched this register takes on a bare WFI. The CPU stops; MCLK,
 * GCLK generator 0 and the clock source feeding it stay alive; every
 * AHB/APB clock a peripheral asks for keeps running; the CAN is clocked
 * and can wake the system. The regulator and the RAM stay in normal
 * mode. Waking costs a handful of cycles.
 *
 * IDLE2 (0x2) - IDLE0 except that the CAN is NOT clocked and therefore
 * cannot wake the system (19.6.3.3.1). That single difference is the
 * whole of it: there is no IDLE1 on this family.
 *
 * STANDBY (0x4) - the CPU and the peripherals stop. What survives is
 * exactly what asks to: a peripheral with RUNSTDBY set requests its
 * clock, that request wakes the GCLK generator behind it, and the
 * generator's request wakes the clock source (the chapter calls the
 * whole arrangement SleepWalking, table 19-2). Nothing else runs. The
 * RAM is back-biased and the low-power regulator supplies VDDCORE
 * unless something below says otherwise. Wake-up sources are the
 * asynchronous interrupts - anything raised on a generic clock, an
 * external clock or an external event - plus the synchronous interrupts
 * of the peripherals configured to run in standby (table 19-1).
 *
 * ## THE TICK STANDS STILL IN STANDBY
 *
 * This is the fact that shapes how a brio program on this target may
 * sleep, and it does NOT travel from the AVR. There the kernel timebase
 * is the RTC's PIT on a 32 kHz oscillator, and it runs through every
 * sleep mode the part has. Here the timebase is SysTick, SysTick is
 * clocked from the CPU clock (samc/ticker.hpp), and in standby the CPU
 * clock stops. So:
 *
 *   KERNEL TIME STOPS FOR EXACTLY AS LONG AS THE STANDBY LASTS, and
 *   every armed time event matures LATE by that amount.
 *
 * The v1 policy is an honest restriction rather than a correction:
 * standby is legitimate when the kernel has NO armed time event, and
 * `TimeEvents<P>::ticks_to_next()` is the question that answers it -
 * empty means nothing is waiting and nothing will be late. An
 * application that arms periodics and also wants standby needs a
 * timebase that survives standby (the RTC does, samc/rtc.hpp) and a
 * resynchronization of the tick counter after every wake; that is
 * designed work, it is named in docs/samc/platform.md's gap list, and
 * it is deliberately NOT hidden inside this header.
 *
 * IDLE is not affected: MCLK and GCLK0 keep running, so SysTick keeps
 * counting and time events mature on time.
 *
 * ## The erratum that makes standby entry a hazard: 1.8.13
 *
 * "When the systick interrupt is enabled and the standby back-bias
 * option is set (STDBYCFG.BBIAS = 1), a hard fault can occur upon
 * standby entry in the rare occasion when the systick interrupt
 * coincides exactly with the standby entry." LIVE on every silicon
 * revision of the E/G/J family, this one included, and its two
 * preconditions are the DEFAULT state of a brio program: BBIASHS reads
 * 1 out of reset (19.8.2, reset value 0x0400) and the SysTick interrupt
 * IS the kernel tick.
 *
 * The workaround the errata gives is "disable the systick interrupt
 * before entering standby and re-enable it after", and it costs
 * NOTHING here because of the paragraph above: the tick is frozen
 * across a standby whether or not its interrupt is enabled. So the
 * discipline is applied at the WFI, in two places and nowhere else:
 *
 *   - `SamPlatform::idle()` (samc/platform_sam.hpp), which is where the
 *     kernel loop sleeps - it reads SLEEPCFG and holds a
 *     `SysTickInterruptGuard` when the armed mode is STANDBY;
 *   - `Pm::sleep()` / `Pm::enter()` below, for a caller that sleeps
 *     outside the kernel loop.
 *
 * A caller that writes its own WFI owes itself the same guard, and
 * `SysTickInterruptGuard` (samc/ticker.hpp - the file that owns the
 * register) is it.
 *
 * ## The regulator and the RAM
 *
 * STDBYCFG carries the only two knobs standby has.
 *
 * VREGSMOD (19.6.4.2, table 19-4) picks which regulator supplies
 * VDDCORE while the device is in standby: AUTO leaves it to the
 * hardware (low-power when nothing sleepwalks, main when something
 * does), PERFORMANCE asks for the main regulator always, LP asks for
 * the low-power one always and 19-4's note limits it to a sleepwalking
 * task on a 32 kHz GCLK. 0x3 is Reserved.
 *
 * ERRATUM 1.8.14, live on this silicon: entering standby with
 * VREGSMOD = PERFORMANCE wrongly switches to the low-power regulator
 * AND keeps requesting GCLK0 - i.e. the setting achieves the opposite
 * of its name and costs the main clock's current on top. The workaround
 * is SUPC.VREG.RUNSTDBY = 1 (`Vreg::run_standby(true)`, samc/supc.hpp),
 * which forces the main regulator from the other end. This header does
 * not set that bit for the caller: it belongs to the supply controller,
 * an application may want it with VREGSMOD = AUTO as well, and a driver
 * that silently wrote another driver's register would be the kind of
 * hidden coupling this codebase refuses. `standby_config_valid()`
 * therefore ACCEPTS performance mode and the comment on the enumerator
 * carries the obligation.
 *
 * BBIASHS (19.6.4.1, table 19-3) back-biases the RAM in standby, which
 * is the low-power state and the reset default. Two consequences worth
 * knowing before clearing or keeping it: with the RAM back-biased THE
 * DMAC CANNOT ACCESS IT (19.6.4.1's own note - a sleepwalking DMA
 * transfer into SRAM needs BBIASHS = 0), and with it set erratum 1.8.13
 * above applies.
 *
 * ## The readback rule
 *
 * 19.6.3.3: "A small latency happens between the store instruction and
 * actual writing of the SLEEPCFG register due to bridges. Software must
 * ensure that the SLEEPCFG register reads the desired value before
 * issuing a WFI instruction." That is not a SYNCBUSY flag and not a
 * synchronized register in the usual sense - it is a plain write posted
 * across a bridge - so `set_sleep_mode()` reads the register back until
 * it agrees, and returns false if it never does. Every verb here that
 * can sleep goes through it.
 *
 * ## The rest of the chapter's own answers
 *
 * The PM has NO interrupt, NO event, NO DMA connection and NO software
 * reset (19.5.4 is about the interrupt CONTROLLER, 19.6.6 and 19.6.7
 * say "Not applicable", 19.6.3.2 says "always enabled and can not be
 * reset"). Its registers are optionally PAC write-protected (19.5.7);
 * brio has no PAC driver yet, so nothing here writes PAC and the
 * protection is left as reset leaves it - off.
 *
 * ITS BUS CLOCK IS ONE-WAY. 19.5.2: "If this clock is disabled, it can
 * only be re-enabled by a system reset." `bus_clock(false)` is
 * therefore a verb that leaves the device unable to sleep until the
 * next reset; it is exposed, with this sentence, rather than hidden.
 *
 * DEBUG CHANGES WHAT STANDBY IS. 19.5.6: with the CPU halted in debug
 * mode the PM keeps operating, and a standby requested while a debugger
 * is attached does NOT turn the power domains off. A sleep measured
 * under a debug session is not the sleep the silicon does on its own -
 * the AVR's chapter 13 says the same thing about its own OCD, and
 * tools/bench.py clears DHCSR.C_DEBUGEN at the end of every flash for
 * this reason among others.
 *
 * ## Errata that touch this chapter, at silicon revision F
 *
 * LIVE: 1.8.13 (above, coded), 1.8.14 (above, the obligation stated),
 * 1.8.7 (a DMA WRITE performed while sleepwalking may not land on some
 * peripheral registers - RTC.COUNT, TC/TCC control and count registers,
 * ADC/SDADC SWTRIG - with "use Idle instead of Standby" as the only
 * workaround; nothing here can enforce it, and the affected drivers'
 * docs carry it), 1.8.5 (increased power consumption in standby, no
 * workaround - a number, not a behaviour).
 *
 * NOT this silicon (revision B only - the read-the-row trap the whole
 * samc stratum has been bitten by): 1.8.1 (Idle does not stop the
 * AHB/APB clocks while the FDPLL runs as a GCLK source), 1.8.11
 * (VREGSMOD has no effect at all), 1.8.6 (the SysTick calibration
 * value, which the ticker never reads anyway).
 *
 * ## Design position
 *
 * The kernel sleeps by itself on a bare WFI, and with SLEEPCFG at its
 * reset value that is IDLE0 - the mode whose wake-up list is complete,
 * so "no events queued" is the whole of what the idle hook must know.
 * Standby gates clock domains and shortens that list, so entering it is
 * a decision about the whole application: which peripherals must
 * survive, which oscillator must stay up for them, what is allowed to
 * wake the program, and - here - whether anything is waiting on a
 * timebase that will stop. That decision belongs to a power-manager
 * active object (util/power.hpp); this header is the mechanism it is
 * built on. `SamSleepSite` at the end is the two-way adapter between
 * them, and it maps the ladder onto SLEEPMODE and nothing more.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/ticker.hpp"
#include "util/power.hpp"

namespace brio {

/// SLEEPCFG.SLEEPMODE (19.8.1). The values ARE the register codes.
/// IDLE0 is the reset value, so it is also "no mode armed".
enum class SleepMode : uint8_t {
    idle0 = PM_SLEEPCFG_SLEEPMODE_IDLE0_Val,
    idle2 = PM_SLEEPCFG_SLEEPMODE_IDLE2_Val,
    standby = PM_SLEEPCFG_SLEEPMODE_STANDBY_Val,
};

/// The three implemented codes; 0x1, 0x3 and 0x5..0x7 are Reserved.
constexpr bool sleep_mode_valid(SleepMode m) {
    return m == SleepMode::idle0 || m == SleepMode::idle2 || m == SleepMode::standby;
}

/// STDBYCFG.VREGSMOD (19.8.2, table 19-4): which regulator supplies
/// VDDCORE in standby.
enum class VregStandbyMode : uint8_t {
    /// Hardware decides: the low-power regulator when nothing
    /// sleepwalks, the main one when something does.
    automatic = PM_STDBYCFG_VREGSMOD_AUTO_Val,

    /// Ask for the main regulator always. ERRATUM 1.8.14, LIVE ON THIS
    /// SILICON: on its own this does the opposite - the system switches
    /// to the low-power regulator AND keeps requesting GCLK0. Set
    /// SUPC.VREG.RUNSTDBY (samc/supc.hpp's `Vreg::run_standby(true)`)
    /// alongside it, or use `automatic`.
    performance = PM_STDBYCFG_VREGSMOD_PERFORMANCE_Val,

    /// Ask for the low-power regulator always. Table 19-4's note 2:
    /// "must only be used when SleepWalking is running on GCLK with
    /// 32 kHz source" - the low-power regulator cannot carry a faster
    /// one.
    low_power = PM_STDBYCFG_VREGSMOD_LP_Val,
};

/// STDBYCFG, the whole of the standby configuration.
struct StandbyConfig {
    VregStandbyMode regulator = VregStandbyMode::automatic;

    /// BBIASHS: back-bias the RAM in standby. TRUE IS THE RESET VALUE
    /// (19.8.2, reset 0x0400) and the low-power state, and it carries
    /// two consequences: the DMAC cannot access a back-biased RAM
    /// (19.6.4.1), and erratum 1.8.13 makes a SysTick interrupt
    /// coinciding with standby entry a potential hard fault (which is
    /// why every WFI in this stratum masks that interrupt first - see
    /// the file header).
    bool back_bias = true;
};

/// VREGSMOD 0x3 is Reserved; nothing else in this register can be wrong.
constexpr bool standby_config_valid(const StandbyConfig& c) {
    return c.regulator == VregStandbyMode::automatic ||
           c.regulator == VregStandbyMode::performance ||
           c.regulator == VregStandbyMode::low_power;
}

// =============================================================================
// The block
// =============================================================================

/**
 * The power manager: which mode the next WFI takes, and what standby
 * does to the regulator and the RAM.
 *
 * A monostate struct and not a template - there is one PM.
 */
struct Pm {
    Pm() = delete;

    static pm_registers_t& regs() { return *PM_REGS; }

    /// CLK_PM_APB. ON out of reset. **Turning it off is one-way**: 19.5.2
    /// says it "can only be re-enabled by a system reset", so a device
    /// whose PM bus clock is off cannot be told to sleep again until it
    /// reboots.
    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_PM_Msk, on); }
    static bool bus_clock() {
        return (MCLK_REGS->MCLK_APBAMASK & MCLK_APBAMASK_PM_Msk) != 0u;
    }

    // ---- SLEEPCFG (19.8.1) ------------------------------------------------

    static uint8_t sleepcfg() { return regs().PM_SLEEPCFG; }

    /// What the next WFI would take. IDLE0 is the reset value, so this
    /// reads `idle0` on a machine that has never armed anything.
    static SleepMode sleep_mode() {
        return static_cast<SleepMode>(sleepcfg() & PM_SLEEPCFG_SLEEPMODE_Msk);
    }

    /**
     * Arm `m`, and DO NOT RETURN UNTIL THE REGISTER SAYS SO.
     *
     * 19.6.3.3: the store crosses a bridge and lands late, and software
     * "must ensure that the SLEEPCFG register reads the desired value
     * before issuing a WFI instruction" - so this reads it back. False
     * for a Reserved code (nothing written) or for a readback that
     * never agreed within `spins`; a caller that sleeps on a false
     * return sleeps in a mode it did not choose.
     */
    static bool set_sleep_mode(SleepMode m, uint32_t spins = 0xFFFFu) {
        if (!sleep_mode_valid(m)) {
            return false;
        }
        const uint8_t want = static_cast<uint8_t>(m);
        regs().PM_SLEEPCFG = want;
        while (sleepcfg() != want && spins-- != 0u) {
        }
        return sleepcfg() == want;
    }

    /// The compile-time twin, so a Reserved SLEEPMODE code is a compile
    /// error rather than a false at run time.
    template <SleepMode m>
    static bool set_sleep_mode(uint32_t spins = 0xFFFFu) {
        static_assert(sleep_mode_valid(m),
                      "brio Pm: SLEEPCFG.SLEEPMODE has three implemented codes "
                      "(19.8.1) - IDLE0 0x0, IDLE2 0x2 and STANDBY 0x4; 0x1, 0x3 "
                      "and 0x5..0x7 are Reserved");
        return set_sleep_mode(m, spins);
    }

    // ---- STDBYCFG (19.8.2) ------------------------------------------------

    static uint16_t stdbycfg() { return regs().PM_STDBYCFG; }

    static VregStandbyMode regulator_mode() {
        return static_cast<VregStandbyMode>((stdbycfg() & PM_STDBYCFG_VREGSMOD_Msk) >>
                                            PM_STDBYCFG_VREGSMOD_Pos);
    }
    static bool back_bias() { return (stdbycfg() & PM_STDBYCFG_BBIASHS_Msk) != 0u; }

    static constexpr uint16_t word(const StandbyConfig& c) {
        return static_cast<uint16_t>(
            PM_STDBYCFG_VREGSMOD(static_cast<uint32_t>(c.regulator)) |
            (c.back_bias ? PM_STDBYCFG_BBIASHS_Msk : 0u));
    }

    /// One store; STDBYCFG has no synchronization of its own. False and
    /// nothing written for the Reserved VREGSMOD code.
    static bool configure_standby(const StandbyConfig& c) {
        if (!standby_config_valid(c)) {
            return false;
        }
        regs().PM_STDBYCFG = word(c);
        return true;
    }

    /// The compile-time twin, so a Reserved regulator code is a compile
    /// error rather than a false at run time.
    template <StandbyConfig cfg>
    static bool configure_standby() {
        static_assert(standby_config_valid(cfg),
                      "brio Pm: STDBYCFG.VREGSMOD 0x3 is Reserved (19.8.2); the "
                      "three modes are AUTO, PERFORMANCE and LP");
        return configure_standby(cfg);
    }

    // ---- the sleep itself -------------------------------------------------

    /**
     * WFI at whatever SLEEPCFG currently holds.
     *
     * The `SysTickInterruptGuard` is erratum 1.8.13's workaround and is
     * taken only when the armed mode is STANDBY - see the file header
     * for why it costs nothing (the tick is frozen across a standby
     * either way) and why it is at the WFI rather than at the arming.
     *
     * WARNING (19.6.3.3): the wake-up source must be configured,
     * ENABLED, and REACHABLE IN THE ARMED MODE before this runs.
     * Standby's wake-up list is short - the asynchronous interrupts,
     * plus the synchronous ones of peripherals with RUNSTDBY - and
     * arming a mode nothing on that list can leave is how a program
     * stops for good. This verb does not touch PRIMASK: on this core a
     * pending interrupt wakes WFI even while masked, so there is no
     * lost-wakeup window to close and no sei-then-sleep pairing to get
     * right.
     */
    [[gnu::always_inline]] static void sleep() {
        if (sleep_mode() == SleepMode::standby) {
            SysTickInterruptGuard guard;
            __DSB();
            __WFI();
        } else {
            __DSB();
            __WFI();
        }
    }

    /// Arm and sleep: the bounded verb, for a caller that has already
    /// established its wake condition. False - and NO sleep - when the
    /// mode could not be armed (see set_sleep_mode()). The mode is left
    /// as it was armed; `set_sleep_mode(SleepMode::idle0)` puts the
    /// register back where reset left it.
    static bool enter(SleepMode m, uint32_t spins = 0xFFFFu) {
        if (!set_sleep_mode(m, spins)) {
            return false;
        }
        sleep();
        return true;
    }
};

// =============================================================================
// The SleepSite
// =============================================================================

/**
 * The SleepSite of this target: util/power.hpp's depth ladder expressed
 * in SLEEPCFG.SLEEPMODE.
 *
 *   none    -> IDLE0     the reset mode; the kernel's idle hook as ever
 *   light   -> IDLE2     the deepest IDLE this family has
 *   standby -> STANDBY
 *   deep    -> STANDBY   there is nothing deeper on this family
 *
 * TWO THINGS TO READ BEFORE USING IT.
 *
 * THE NEVER-DEEPER RULE EARNS ITS KEEP HERE, unlike on AVR DA/DB where
 * the three rungs matched three modes exactly. This family's deepest
 * stop is STANDBY, so `deep` maps to it and `armed()` reports
 * `standby` - what the target really took, which is what the manager
 * records. An application asking for `deep` gets the deepest stop this
 * silicon offers and never one that loses more than it agreed to lose.
 *
 * THE REGISTER IS THE STATE, and that is why `light` is IDLE2 rather
 * than IDLE0. There is no SEN bit here: IDLE0 is both a sleep mode and
 * the reset value, so a site that armed `light` as IDLE0 could not tell
 * "armed at light" from "nothing armed" without a variable shadowing
 * the register. Mapping `light` to IDLE2 makes the three rungs three
 * distinct codes, keeps `armed()` a pure read of the silicon, and costs
 * exactly one thing: the CAN cannot wake the device from `light`
 * (19.6.3.3.1 - IDLE2's only difference from IDLE0). brio has no CAN
 * driver; an application that grows one and wants a CAN wake from a
 * light sleep should arm IDLE0 through `Pm::enter()` directly.
 *
 * IT ONLY ARMS. The WFI is still the kernel loop's: SamPlatform::idle()
 * takes whatever SLEEPCFG holds instead of imposing its own, which is
 * what makes a power manager possible with no new kernel hook. So
 * everything chapter 19 warns about still applies to the caller - above
 * all that the wake-up source must be reachable in the armed mode, and
 * (this target's own addition) that KERNEL TIME STOPS across a standby,
 * so no armed time event may be waiting on it. See the file header.
 */
struct SamSleepSite {
    SamSleepSite() = delete;

    static bool arm(SleepDepth d) {
        switch (d) {
        case SleepDepth::none:
            return Pm::set_sleep_mode(SleepMode::idle0);
        case SleepDepth::light:
            return Pm::set_sleep_mode(SleepMode::idle2);
        case SleepDepth::standby:
        case SleepDepth::deep:
            return Pm::set_sleep_mode(SleepMode::standby);
        }
        return false;
    }

    static void disarm() { (void)Pm::set_sleep_mode(SleepMode::idle0); }

    static SleepDepth armed() {
        switch (Pm::sleep_mode()) {
        case SleepMode::idle0:
            return SleepDepth::none;
        case SleepMode::idle2:
            return SleepDepth::light;
        case SleepMode::standby:
            return SleepDepth::standby;
        }
        return SleepDepth::none;
    }
};

static_assert(SleepSite<SamSleepSite>);

} // namespace brio
