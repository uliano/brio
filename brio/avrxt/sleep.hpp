/*
 * sleep.hpp
 *
 * The sleep controller of the AVR DA, DB and EA (SLPCTRL, DS40002247B
 * ch. 13, DS40002443A ch. 13): what a program uses to stop the chip
 * deeper than the kernel does by itself. The kernel's idle hook already
 * sleeps in IDLE whenever every queue is empty (AvrPlatform::idle(),
 * avrxt/platform.hpp), so a program that only wants "nothing to do right
 * now" needs nothing from this header. STANDBY and POWER-DOWN stop
 * clocks and shorten the list of what can wake the chip, and this header
 * is the mechanism for them, in two shapes. Code that owns the moment
 * enters a mode itself, its wake source configured and enabled first
 * (the shape docs/avrxt/platform.md, "How to use it", spells out):
 *
 *     brio::Pit::init(brio::PitPeriod::cyc4096);        // 125 ms, interrupt on
 *     brio::Sleep::enter(brio::SleepMode::standby);     // interrupts on, wake armed
 *
 * and an active-object program names AvrSleepSite, at the end of this
 * file, in its power manager (util/power.hpp), which arms a mode after
 * every stakeholder agreed and leaves the SLEEP instruction to the
 * kernel loop's idle().
 *
 * Who calls what. Sleep::arm() and disarm() are one store of
 * SLPCTRL.CTRLA each, preceded by the erratum's NOP (below); armed() and
 * armed_mode() one load: none masks or waits. Sleep::sleep() and enter()
 * stop the CPU until an enabled interrupt the armed mode keeps comes,
 * and touch SREG not at all: global interrupts must already be on, and
 * with no wake source that reaches the armed mode only a reset or a
 * debugger comes back (13.3.1). They are for main context: a program's
 * own code before the kernel runs, or code that owns the moment - under
 * a kernel the loop's idle() executes the instruction. Vreg's verbs are
 * setup verbs: a read and a CCP-protected write of VREGCTRL each, and
 * high_temp_low_leakage(true) refuses (false, nothing written) while a
 * TWI client or the CCL is enabled. AvrSleepSite's arm() and disarm()
 * are the power manager's, called from its handler.
 *
 * VREGCTRL and the regulator control exist on the DA and the DB alone.
 * The AVR EA's SLPCTRL (DS40002443A ch. 13) is CTRLA alone, the same
 * three modes: no VREGCTRL and no regulator control anywhere, so Vreg
 * and VregPower do not exist there (device tables' has_vreg). What
 * follows is the AVR DB's chapter unless it names the EA. The mechanism
 * only: which mode is armed, what a SLEEP instruction then does, and
 * what the regulator is told to do while the CPU is stopped. The policy
 * - when a program may stop its clocks - is not here and is not the
 * kernel's either (see the design note at the end of this comment).
 *
 * Tables 13-2 (activity), 13-3 (clocks) and 13-4 (wake-up sources) are
 * rewritten by errata DS80000915F clarification 3.4.1; what follows is
 * the clarified version, which is the one the code and docs/avrxt/
 * platform.md follow.
 *
 * IDLE. The CPU stops and nothing else does: every peripheral keeps its
 * clock, every interrupt source wakes, and waking costs six CLK_PER
 * cycles (13.3.3.2, measured). This is the mode the kernel's idle hook
 * uses (AvrPlatform::idle()); a program that only wants "nothing to do
 * right now" wants IDLE and nothing from this header.
 *
 * STANDBY. CLK_CPU stops and every clock domain stops unless something
 * asks for it. RTC, CCL, AC, ADC, DAC, OPAMP, TCA and TCB run only with
 * their own RUNSTDBY set; WDT, BOD, EVSYS and NVM run regardless;
 * everything else is stopped - the TCD's clock never runs in standby at
 * all. The main clock source runs only if something requests it, and the
 * peripheral's flag alone is that request (measured): a TCB with
 * RUNSTDBY set counts through standby with the oscillator's own
 * RUNSTDBY on or off, and with it clear the whole chain stops whatever
 * the oscillator's flag says. An oscillator's RUNSTDBY (Oschf::
 * run_standby(), Osc32k::run_standby()) buys something else entirely:
 * the source stays alive unrequested, so the wake-up pays no start-up
 * time - and, because the regulator watches the same thing, it buys a
 * fast wake-up for the other source too (see "Wake time" below). Wake-up
 * sources: a PORT pin (in an asynchronously sensing configuration), BOD
 * VLM, MVIO, the RTC (counter functions with RUNSTDBY), a TWI address
 * match, the CCL with RUNSTDBY, USART start-of-frame detection, and
 * TCA/TCB/ADC/AC interrupts from the instances left running. A TWI
 * client's address match is the only wake-up its peripheral offers: the
 * data interrupts that carry the rest of the frame are not on this
 * list, so a program that sleeps again after the match stalls the
 * tenure with the host holding SCL.
 *
 * POWER-DOWN. Only the PIT, the WDT, a sampled BOD and the EVSYS remain
 * - the RTC's counter stops even with RUNSTDBY, the PIT half does not.
 * Wake-up sources: a PORT pin, BOD VLM, MVIO, the PIT, a TWI address
 * match, and the CCL only when its path is fully asynchronous
 * (FILTSEL = 0 and EDGEDET = 0) - more exactly, whenever the LUT's own
 * clock is one power-down keeps: a filtered LUT clocked from OSC32K
 * wakes, the same LUT clocked from CLK_PER does not (measured). With
 * VREGCTRL.HTLLEN set the list shrinks to the PORT pin, BOD VLM, MVIO
 * and the PIT: no TWI match, no CCL - which is why
 * high_temp_low_leakage(true) refuses while a TWI client or the CCL is
 * enabled instead of silently making their wake-up unpredictable (the
 * chapter's own warning).
 *
 * Wake time. Six CLK_PER cycles from IDLE. From standby or power-down
 * the same six cycles plus whatever the main clock source needs to come
 * back and the regulator to reach its normal drive - and those are two
 * separate bills, measured apart (platform.md carries the table):
 *
 *   the oscillator - OSCHF comes back in 24 us, a 24 MHz crystal in
 *   1.77 ms, and an oscillator kept alive by its own RUNSTDBY in no
 *   time at all (43 CLK_PER ticks of a 24 MHz counter, wake included);
 *
 *   the regulator - 290 us more, but only when the device really let
 *   go. VREGCTRL's AUTO profile drops to low power in standby and
 *   power-down "and whenever OSC32K is the only clock running"
 *   (13.3.5): an oscillator left running by its own RUNSTDBY keeps the
 *   regulator at full drive even when it is not the main clock source,
 *   so beside a running crystal an OSCHF restart out of standby costs
 *   its bare 24 us and PMODE changes nothing. Power-down stops every
 *   oscillator whatever RUNSTDBY says, so there the regulator is always
 *   paid for - and PMODE = FULL, which pays it in advance at the price
 *   of quiescent current, is the difference between 313 us and 24 us.
 *
 * A crystal's own start-up buries both: it costs the same 1.77 ms in
 * standby and in power-down, and no profile touches it.
 *
 * Two register facts. SLPCTRL.CTRLA is not under CCP; VREGCTRL is the
 * one CCP-protected register of the block (13.3.5, table 13-6). And
 * errata DS80000915F 2.2.4 (all revisions): a store to an address >= 64
 * immediately followed by a write to SLPCTRL.CTRLA loses that write, so
 * every CTRLA store below is preceded by a NOP - the documented
 * workaround. The AVR EA's twin is DS80001048D 2.2.3. The DA errata of
 * record predates the item and carries no twin; the NOP costs one cycle
 * and is emitted on every family.
 *
 * A debugger break wakes the device to Active mode whether or not an
 * interrupt is pending (13.3.4): a sleep measured under a debug session
 * is not the sleep the silicon does on its own.
 *
 * On the AVR EA (DS40002443A ch. 13; docs/avrxt/platform.md has the
 * measurements). CTRLA alone, the same three modes and codes (13.5.1).
 * STANDBY runs the RTC, CCL, ADC, TCA and TCB with their own RUNSTDBY
 * (table 13-1), and the AC and DAC with theirs (30.3.5, 32.3.3 - table
 * 13-1 omits them; the AC's interrupt measured waking the device), the
 * WDT, the BOD and the EVSYS regardless; POWER-DOWN the PIT, the WDT, the
 * BOD and the EVSYS. The wake-up sources are table 13-3's: a PORT pin
 * from every mode - and every pin of the EA senses fully asynchronously,
 * one edge of any of them waking the device from power-down (18.3.3.1,
 * measured on twelve pins) where the DA/DB need Px2/Px6 or both
 * edges -, the TWI address match and the BOD VLM from every mode, the
 * RTC with RUNSTDBY in standby and its PIT in power-down, the CCL with
 * RUNSTDBY in standby and on an asynchronous path in power-down, the
 * USART's start of frame in standby only, the TCA/TCB/ADC/AC left
 * running in standby. The wake pays six CLK_PER cycles and one
 * oscillator start-up (table 13-4) and, measured, nothing else: there is
 * no regulator start-up on this die - OSCHF restarts in the same 9.2 us
 * beside a running crystal and with every other oscillator stopped, out
 * of standby and out of power-down alike (13.3.2.2's "the time it takes
 * to start the regulator if it has been switched off" is never met) -
 * while a stopped crystal costs its build-up plus CSUTHF's count (0.61
 * to 0.67 ms cold for this board's 20 MHz quartz at 4096 cycles). FUSE.BODCFG's
 * ACTIVE = ENABLEWAIT halts a wake until the BOD runs (13.3.2.2): the
 * figures above are with the BOD continuous asleep too, where it has
 * nothing to wait for; a BOD sampled or off in sleep would add its
 * start-up (tBOD_ST, 30 us typical, table 35-15), not measured.
 *
 * Design position. The kernel sleeps by itself in IDLE and only in IDLE
 * - the wake-up list is complete there, so "no events queued" can mean
 * "stop the CPU" with nothing else to know. Standby and power-down gate
 * clock domains and shorten the wake-up list, so entering them is a
 * decision about the whole application: which peripherals must survive,
 * which oscillator must stay up for them, what is allowed to wake the
 * program. That decision belongs to a power-manager active object
 * (util/power.hpp), and this header is the mechanism it is built on -
 * not a policy. `AvrSleepSite` at the end is the two-way adapter between
 * them: it maps the model's depth ladder onto SMODE and nothing more.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrxt/device_tables.hpp"
#include "util/power.hpp"

namespace brio {

/// SLPCTRL.CTRLA.SMODE (13.5.1). The values are the register codes (the
/// device header spells standby STDBY; the names here are the modes as
/// chapter 13 calls them).
enum class SleepMode : uint8_t {
    idle = SLPCTRL_SMODE_IDLE_gc,
    standby = SLPCTRL_SMODE_STDBY_gc,
    power_down = SLPCTRL_SMODE_PDOWN_gc,
};

/// The sleep controller: which mode is armed, and the instruction that
/// takes it.
struct Sleep {
    Sleep() = delete;

    /// Arm `m`: one store of SMODE + SEN, preceded by the erratum's NOP
    /// (2.2.4, see the file header). Arming alone does nothing - "the
    /// SLEEP instruction must be executed to make the device go to
    /// sleep" (13.3.1).
    static void arm(SleepMode m) {
        __asm__ __volatile__("nop");
        SLPCTRL.CTRLA = static_cast<uint8_t>(static_cast<uint8_t>(m) | SLPCTRL_SEN_bm);
    }

    /// Disarm: a SLEEP instruction becomes a no-op again.
    static void disarm() {
        __asm__ __volatile__("nop");
        SLPCTRL.CTRLA = 0;
    }

    /// SEN: would a SLEEP instruction suspend the CPU?
    static bool armed() { return (SLPCTRL.CTRLA & SLPCTRL_SEN_bm) != 0; }

    /// SMODE, whether or not SEN is set.
    static SleepMode armed_mode() {
        return static_cast<SleepMode>(SLPCTRL.CTRLA & SLPCTRL_SMODE_gm);
    }

    /// The SLEEP instruction itself.
    ///
    /// Warning (13.3.1): the wake-up sources must be configured and
    /// enabled, and global interrupts on, before this runs. With no
    /// enabled interrupt that can reach the armed mode, only a reset
    /// (or a debugger) ever comes back. This verb does not touch SREG:
    /// keeping I set is the caller's business, and so is closing the
    /// lost-wake-up window - arm() first, then mask, test the condition
    /// and `sei(); sleep();` back to back, which is the sequence
    /// AvrPlatform::idle() emits for IDLE.
    ///
    /// The memory clobber keeps the compiler from moving loads of state
    /// an ISR writes across the instruction.
    [[gnu::always_inline]] static void sleep() {
        __asm__ __volatile__("sleep" ::: "memory");
    }

    /// Arm, sleep, disarm: the bounded verb, for a caller that has
    /// already established its wake condition (interrupts enabled, the
    /// source armed). Leaves SLPCTRL.CTRLA at 0 whatever woke it.
    static void enter(SleepMode m) {
        arm(m);
        sleep();
        disarm();
    }
};

// The regulator's sleep control exists only where SLPCTRL has VREGCTRL
// (device tables' has_vreg): elsewhere Vreg and VregPower are not named.
#if defined(SLPCTRL_PMODE_gm)

/// SLPCTRL.VREGCTRL.PMODE (13.5.2): what the internal voltage regulator
/// does while the device sleeps. The data sheet calls the two settings
/// "Normal" (AUTO: low-power mode in standby and power-down, and
/// whenever OSC32K is the only clock running) and "Performance" (FULL:
/// maximum drive in every mode, which buys a fast start-up out of the
/// deep modes at the price of quiescent current).
enum class VregPower : uint8_t {
    normal = SLPCTRL_PMODE_AUTO_gc,
    performance = SLPCTRL_PMODE_FULL_gc,
};

/// The voltage regulator's sleep behaviour. Its one register is CCP
/// protected (13.3.5), so every write here goes through the IOREG key.
struct Vreg {
    Vreg() = delete;

    /// Set PMODE, keeping HTLLEN as it is.
    static void power(VregPower p) {
        const uint8_t keep = static_cast<uint8_t>(SLPCTRL.VREGCTRL & SLPCTRL_HTLLEN_bm);
        _PROTECTED_WRITE(SLPCTRL.VREGCTRL, static_cast<uint8_t>(static_cast<uint8_t>(p) | keep));
    }

    static VregPower power() {
        return static_cast<VregPower>(SLPCTRL.VREGCTRL & SLPCTRL_PMODE_gm);
    }

    /// High-Temperature Low Leakage: cuts leakage in Power-Down at high
    /// junction temperatures, and shortens that mode's wake-up list to
    /// the PORT pin, BOD VLM, MVIO and the PIT.
    ///
    /// Enabling is refused while a TWI client or the CCL is enabled:
    /// chapter 13 requires the TWI address-match and CCL wake-up
    /// sources to be disabled when HTLLEN is active "to avoid
    /// unpredictable behavior", and a rule stated in a comment but not
    /// enforced is a rule the next program breaks. False means nothing
    /// was written - shut those blocks down first. Disabling always
    /// succeeds.
    static bool high_temp_low_leakage(bool on) {
        if (on && (twi_client_enabled() || ccl_enabled())) {
            return false;
        }
        const uint8_t keep = static_cast<uint8_t>(SLPCTRL.VREGCTRL & SLPCTRL_PMODE_gm);
        _PROTECTED_WRITE(SLPCTRL.VREGCTRL,
                         static_cast<uint8_t>(keep | (on ? SLPCTRL_HTLLEN_bm : 0)));
        return true;
    }

    static bool high_temp_low_leakage() {
        return (SLPCTRL.VREGCTRL & SLPCTRL_HTLLEN_bm) != 0;
    }

private:
    /// Any TWI instance this package has, with its client half enabled
    /// (SCTRLA.ENABLE) - the address match that HTLLEN must not race.
    /// Read through avr/io.h rather than avrxt/twi.hpp: the interlock is
    /// a fact about two registers, not a reason for this header to
    /// depend on a driver.
    static bool twi_client_enabled() {
        bool on = (TWI0.SCTRLA & TWI_ENABLE_bm) != 0;
#if defined(TWI1)
        on = on || (TWI1.SCTRLA & TWI_ENABLE_bm) != 0;
#endif
        return on;
    }

    static bool ccl_enabled() { return (CCL.CTRLA & CCL_ENABLE_bm) != 0; }
};

#endif // SLPCTRL_PMODE_gm

/**
 * The SleepSite of this target: the depth ladder of util/power.hpp
 * expressed in SMODE.
 *
 *   none    -> nothing armed; the kernel's idle hook behaves as always
 *   light   -> IDLE       standby -> STANDBY       deep -> PDOWN
 *
 * This family realizes every rung, so the "map an absent rung to the
 * nearest shallower one" rule of the model is the identity here and
 * armed() reads back exactly what was asked. A target with fewer modes
 * is where that rule earns its keep.
 *
 * It only arms. The SLEEP instruction is still the kernel loop's:
 * AvrPlatform::idle() leaves an already-armed deeper mode standing and
 * takes it instead of imposing IDLE, which is what makes a power manager
 * possible with no new kernel hook. Everything the chapter warns about
 * therefore still applies to the caller - above all that the wake-up
 * source must be configured, enabled and reachable in the armed mode
 * before the round is accepted: standby and power-down have short
 * wake-up lists (see the file header), and arming a mode nothing on the
 * list can leave is how a program stops for good.
 */
struct AvrSleepSite {
    AvrSleepSite() = delete;

    static bool arm(SleepDepth d) {
        switch (d) {
        case SleepDepth::none:
            Sleep::disarm();
            return true;
        case SleepDepth::light:
            Sleep::arm(SleepMode::idle);
            return true;
        case SleepDepth::standby:
            Sleep::arm(SleepMode::standby);
            return true;
        case SleepDepth::deep:
            Sleep::arm(SleepMode::power_down);
            return true;
        }
        return false;
    }

    static void disarm() { Sleep::disarm(); }

    static SleepDepth armed() {
        if (!Sleep::armed()) {
            return SleepDepth::none;
        }
        switch (Sleep::armed_mode()) {
        case SleepMode::idle:
            return SleepDepth::light;
        case SleepMode::standby:
            return SleepDepth::standby;
        case SleepMode::power_down:
            return SleepDepth::deep;
        }
        return SleepDepth::none;
    }
};

static_assert(SleepSite<AvrSleepSite>);

} // namespace brio
