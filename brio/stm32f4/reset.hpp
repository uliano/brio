/*
 * reset.hpp
 *
 * Why the program is running, and how to end it on purpose (STM32F4):
 * the RCC's reset flags (RM0090 7.1, 7.3.21 - RM0390 6.3.21, RM0383
 * 6.3.21), the two watchdogs this family carries - the independent one
 * on its own oscillator (RM0090 ch. 21) and the system window watchdog
 * on the APB1 clock (ch. 22) - the FOUR FAULT VECTORS a Cortex-M4 has
 * where an M0+ has one (PM0214 4.3.10..4.3.15), and the handler body
 * that turns a crash into a message the next boot can read.
 *
 * This is the other half of the panic breadcrumb. The kernel writes a
 * PanicRecord into reset-surviving storage (kernel/panic.hpp,
 * Stm32f4Platform::panic_record) and hands over to a Reporter whose job
 * may be "reset now and report at the next boot"; panic.hpp's contract
 * then says to cross-check the target's reset-cause register for the
 * full story. `Reset::take_flags()` IS that cross-check,
 * `Reset::software()` is one way of causing the reset, `Iwdg` and `Wwdg`
 * are two more, and `hard_fault_reset<P>()` is the body an app binds to
 * a fault vector so a fault leaves a note instead of a spin.
 *
 * SEVEN FACTS OF THIS SILICON shape everything below.
 *
 * 1. THE FLAGS ACCUMULATE, AND THERE IS NO "THE CAUSE". RCC_CSR's seven
 *    reset flags are set by hardware and every one of them is "cleared
 *    by writing to the RMVF bit" - nothing else clears them, and 7.3.21
 *    says the register is "reset by system reset, except reset flags by
 *    power reset only". So this register is a HISTORY and not a single
 *    cause, and the boot verb is read-and-clear: `take_flags()`. What a
 *    boot sees is the delta since the last clear. PINRSTF is described
 *    here as the pin's alone ("set by hardware when a reset from the
 *    NRST pin occurs"), but the pad is driven low by the internal reset
 *    sources, so a software reset raises it too (measured:
 *    docs/stm32f4/reset.md). `pin_only()` is therefore the only reading
 *    PINRSTF can be trusted for, and this file offers no `cause()` enum:
 *    with an accumulating register and a bit every source raises, the
 *    cause is not a function of the register's value.
 *
 * 2. RCC_CSR IS A SHARED REGISTER, and the two halves have two owners.
 *    Bits 31..24 are this chapter's (the seven flags and RMVF); bits
 *    1..0 are the clock tree's (LSION/LSIRDY) and belong to
 *    stm32f4/clock.hpp's `Rcc`, which is where the LSI verbs live -
 *    the IWDG runs on that oscillator and the RTC will too. Every write
 *    on either side is a read-modify-write that preserves the other's
 *    bits, which is safe in both directions because RMVF reads as zero
 *    and writing zero to it has no effect.
 *
 * 3. THE IWDG OF THIS FAMILY HAS NO WINDOW. Four registers - KR, PR,
 *    RLR, SR - and that is the whole chapter: no WINR, no WVU, and so
 *    no early-refresh reset (the reserve's `iwdg_has_window()` asks the
 *    device header and answers false on all twenty-three). A deliberate
 *    reset therefore cannot be spelled as a refresh into a closed
 *    window; `force_reset()` programs the shortest reload instead and
 *    refreshes into it.
 *
 * 4. THE IWDG IS ONE WAY IN SOFTWARE - AND A RESET DOES STOP IT. 21.3
 *    starts it with 0xCCCC in the key register, and the Stop/Standby
 *    summaries (RM0090 5.3.4) finish the sentence: "Once started it
 *    cannot be stopped except by a Reset". MEASURED, and worth stating
 *    because the family lore teaches the opposite: a boot that follows
 *    an IWDG reset outlives three of its own time-outs with nothing
 *    refreshing anything, so a program does NOT inherit a watchdog it
 *    must feed for ever. What a caller commits to is this run of the
 *    program. There is no bit that says whether it runs; `running()` is
 *    the only witness the silicon offers - the LSI a start forces on -
 *    and it is a witness and not a proof.
 *
 * 5. THE KEYED REGISTERS DO NOT UPDATE UNTIL THE WATCHDOG IS STARTED,
 *    and this is the finding chapter 21 never states. PR and RLR are
 *    writable only after the key 0x5555 (21.3.2), and any other key
 *    value - the 0xAAAA refresh included - locks them again. Each has a
 *    bit in IWDG_SR that hardware raises AT THE STORE (measured: the
 *    read right after it already sees the bit) and drops "when the
 *    update operation is completed in the VDD voltage domain (takes up
 *    to 5 RC 40 kHz cycles)", and a register read while its bit stands
 *    returns the OLD value (21.4.2 and 21.4.3 both say so). WITH THE
 *    WATCHDOG STOPPED THE BIT NEVER DROPS, LSION or no LSION - 7.2.9 is
 *    where the reason hides: "if the independent watchdog is started ...
 *    after the LSI oscillator temporization, the clock is provided to
 *    the IWDG", so the logic that performs the update has no clock until
 *    the start key is written. That is why arm() below starts before it
 *    configures, and why configure() on a stopped watchdog is bounded
 *    and answers false rather than hanging - which is also the shape the
 *    errata want.
 *
 * 6. THE WWDG IS THE OPPOSITE PERIPHERAL IN EVERY WAY. It counts PCLK1
 *    through a fixed /4096 and a programmable /2^WDGTB - TWO bits here,
 *    four codes, /1 to /8 - it has a bus clock enable that is CLEAR AT
 *    RESET (a peripheral with no clock does not answer register reads),
 *    and its activation bit WDGA is "set by software and only cleared by
 *    hardware after a reset" (22.6.1). Its down-counter is FREE-RUNNING
 *    "even if the watchdog is disabled" (22.3) and EWIF "is also set if
 *    the interrupt is not enabled" (22.6.3), which together make the
 *    whole TIMING path measurable with WDGA never set - the way this
 *    stratum's bench suite measures it, and the reason `configure()` and
 *    `refresh()` are separate verbs from `start()`. A window below 0x40
 *    can never be served (a refresh is legal only while the counter is
 *    at or below W AND above 0x3F), so `wwdg_config_valid()` refuses it
 *    rather than arm a watchdog nobody could feed; `force_reset()` is
 *    how a caller asks for the reset deliberately - WDGA with T6 clear,
 *    22.3's own note.
 *
 * 7. FOUR FAULT VECTORS, THREE OF THEM DISABLED AT RESET. MemManage,
 *    BusFault and UsageFault are configurable exceptions (PM0214 4.3.10:
 *    SHCSR's MEMFAULTENA, BUSFAULTENA, USGFAULTENA, all clear out of
 *    reset) that ESCALATE to HardFault while they are off - which is why
 *    a program that never touches SHCSR sees every fault at one vector,
 *    with HFSR.FORCED set. `Faults` is the switch and the status:
 *    enable() opens the three, the CCR traps turn a divide by zero and
 *    an unaligned access into UsageFaults that would otherwise be
 *    silent, and `read()` gathers CFSR / HFSR / MMFAR / BFAR into a
 *    twelve-byte `FaultRecord` a handler can bank. NO STORAGE IS OWNED
 *    HERE: where the record goes across the reset is the application's
 *    (its own .noinit), exactly as the kernel's PanicRecord is the
 *    platform's.
 *
 * ERRATA: the IWDG's update bits are the one item all three sheets carry
 * - ES0206 Rev 24 2.8.1..2.8.4, ES0298 Rev 8 2.9.1..2.9.4, ES0287 Rev 6
 * 2.7.1..2.7.4, on every silicon revision of every part: RVU and PVU are
 * never cleared if the device enters Stop while one stands, or if the
 * APB clock is below twice the IWDG clock. Neither can bite here - the
 * bounded `sync()` reports instead of hanging, the APB is megahertz
 * against the LSI's kilohertz, and Stop is the power chapter's, which
 * inherits the standing advice: leave no update in flight when arming a
 * sleep. No sheet has an item for the WWDG or for the reset flags.
 *
 * NOT BUILT (docs/stm32f4/reset.md carries the list): the OPTION BYTES
 * behind this chapter - WDG_SW (a hardware IWDG started before any
 * software runs), nRST_STOP and nRST_STDBY (what LPWRRSTF reports on)
 * and the BOR level behind BORRSTF - which are FLASH_OPTCR's and belong
 * to the flash chapter, because writing that register with a wrong value
 * bricks a board; and the debug freeze bits, which are READ ONLY here
 * because they are the debugger's - a program that fights them hides the
 * behaviour a suite is trying to measure.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"

namespace brio {

// =============================================================================
// The reset flags (RCC_CSR, RM0090 7.3.21)
// =============================================================================

/// One bit per reset source, exactly as RCC_CSR carries them. They
/// ACCUMULATE until RMVF is written, so these are masks and not an
/// enumeration of "the cause" (fact 1 in this file's header).
struct ResetFlag {
    /// LPWRRSTF: an illegal Stop / Standby entry, and only when the
    /// matching nRST_* option byte enables that reset.
    static constexpr uint32_t low_power = RCC_CSR_LPWRRSTF;
    /// WWDGRSTF: the window watchdog bit - the counter, or a refresh
    /// made above the window.
    static constexpr uint32_t window_watchdog = RCC_CSR_WWDGRSTF;
    /// IWDGRSTF: the independent watchdog reached zero.
    static constexpr uint32_t independent_watchdog = RCC_CSR_IWDGRSTF;
    /// SFTRSTF: the core's own SYSRESETREQ (Reset::software()).
    static constexpr uint32_t software = RCC_CSR_SFTRSTF;
    /// PORRSTF: a power-on / power-down reset.
    static constexpr uint32_t power_on = RCC_CSR_PORRSTF;
    /// PINRSTF: the NRST pin. The pad is driven low by the internal
    /// sources too, so this bit names the pin only when it stands ALONE
    /// (pin_only()).
    static constexpr uint32_t pin = RCC_CSR_PINRSTF;
    /// BORRSTF: the brown-out detector - and 7.3.21's own description
    /// says "set by hardware when a POR/PDR or BOR reset occurs", so it
    /// stands beside PORRSTF on a cold boot rather than instead of it.
    static constexpr uint32_t brown_out = RCC_CSR_BORRSTF;

    static constexpr uint32_t all = low_power | window_watchdog |
                                    independent_watchdog | software | power_on |
                                    pin | brown_out;

    /// The two watchdogs together, for a caller that only wants to know
    /// that one of them bit.
    static constexpr uint32_t watchdog = window_watchdog | independent_watchdog;

    /// The three a supply event raises together (fact 1: a cold boot is
    /// BOR + POR + PIN on this family, measured).
    static constexpr uint32_t supply = power_on | brown_out;
};

/**
 * The reset flags, and the CPU's own way of causing a reset.
 *
 * There is no reset CONTROLLER on this family: the flags live in the RCC
 * and the reset request lives in the core, so this type is the two of
 * them under one name, which is where an application looks for either.
 */
struct Reset {
    Reset() = delete;

    /// The seven flags as they stand. Non-destructive: reading changes
    /// nothing, so the boot code may sample this as often as it likes.
    static uint32_t flags() { return RCC->CSR & ResetFlag::all; }

    /// Clear every flag through RMVF, preserving the LSI bits this
    /// register shares with the clock tree (fact 2).
    ///
    /// RMVF is written set and then put back to zero, so that the next
    /// boot's read of the register is clean whichever way the silicon
    /// treats the bit (7.3.21 marks it rt_w and gives writing zero "no
    /// effect").
    static void clear_flags() {
        RCC->CSR = RCC->CSR | RCC_CSR_RMVF;
        RCC->CSR = RCC->CSR & ~RCC_CSR_RMVF;
    }

    /// Read-and-clear: the boot verb. What comes back is the history
    /// since the last clear, and the register is empty afterwards, so
    /// the NEXT boot reports only what happened in between.
    static uint32_t take_flags() {
        const uint32_t f = flags();
        clear_flags();
        return f;
    }

    /// True when `bits` (a sample of flags()) names only the NRST pin -
    /// the one reading PINRSTF can be trusted for, since the internal
    /// reset sources drive the same pad.
    static constexpr bool pin_only(uint32_t bits) {
        return (bits & ResetFlag::all) == ResetFlag::pin;
    }

    /**
     * Reset the device now, through the Cortex's SYSRESETREQ. Shows up
     * as SFTRSTF at the next boot.
     *
     * The mechanism is the core's, not the RCC's - which is why the verb
     * lives here but the code is CMSIS's. It carries the DSBs the ARM
     * ARM asks for.
     */
    [[noreturn]] static void software() {
        NVIC_SystemReset();
        for (;;) {  // NVIC_SystemReset is __NO_RETURN; this satisfies the
        }           // compiler on any CMSIS that forgets to say so.
    }
};

// =============================================================================
// IWDG (RM0090 ch. 21)
// =============================================================================

/// IWDG_PR: the LSI divider ahead of the 12-bit down-counter. Codes 6
/// and 7 BOTH mean /256 (21.4.2), so `div256` is the only spelling of
/// that ratio this driver offers and 7 is never written.
enum class IwdgPrescaler : uint8_t {
    div4 = 0,
    div8 = 1,
    div16 = 2,
    div32 = 3,
    div64 = 4,
    div128 = 5,
    div256 = 6,
};

/// 4 << code - the divider a prescaler field means.
constexpr uint32_t iwdg_divider(IwdgPrescaler p) {
    return 4UL << static_cast<uint8_t>(p);
}

/// The nominal time-out of a (prescaler, reload) pair, in milliseconds,
/// at a STATED LSI rate. The rate is the caller's argument and not a
/// constant of this file: LSI is an uncalibrated RC (the datasheets'
/// LSI table gives 17..47 kHz over the full range, 32 kHz typical) and a
/// driver that pretended to know it would be lying. At the 32 kHz
/// nominal this reproduces RM0090 table 107's max column (0x0FFF at /4 =
/// 512 ms); the table's min column is sub-millisecond and truncates to
/// zero here, which is the price of milliseconds and not an error.
constexpr uint32_t iwdg_nominal_ms(IwdgPrescaler p, uint16_t reload,
                                   uint32_t lsi_hz = 32'000UL) {
    return static_cast<uint32_t>(
        (iwdg_divider(p) * (static_cast<uint32_t>(reload) + 1UL) * 1000UL) / lsi_hz);
}

/// A whole IWDG setting. The reset values are the ones the silicon
/// starts from (RL 0x0FFF at /4: 512 ms at the nominal LSI). There is no
/// window on this family (fact 3), so this is the whole of it.
struct IwdgConfig {
    IwdgPrescaler prescaler = IwdgPrescaler::div4;
    /// IWDG_RLR.RL, 12 bits: the value loaded on every refresh.
    uint16_t reload = 0x0FFF;
};

/// RL is a twelve-bit field. A reload of zero is LEGAL and is table
/// 107's min column - the shortest time-out the part has, which is what
/// Iwdg::force_reset() programs on purpose.
constexpr bool iwdg_config_valid(const IwdgConfig& c) {
    return c.reload <= 0x0FFFu;
}

/**
 * The independent watchdog: one 12-bit down-counter on LSI, four
 * registers, and no way back in software once it is started.
 *
 * ORDER, AND WHY IT IS THE CHAPTER'S. 21.3 starts the watchdog with
 * 0xCCCC and then configures it behind 0x5555, and `arm()` is exactly
 * that: start, prescaler, reload, refresh. `configure()` is the same
 * body WITHOUT the start, which is legal (the key protecting PR and RLR
 * has nothing to do with the start key) and is how a program re-times a
 * watchdog that is already running.
 *
 * IT NEEDS TO BE STARTED TO CONFIGURE, not merely to have an LSI (fact
 * 5): the update bits drop only once the start key has given the block
 * its clock, so configure() on a stopped watchdog writes the registers,
 * watches the bits stand, and answers false in bounded time. That is the
 * honest report of a configuration that has not taken, and it is why the
 * only useful order is arm()'s.
 */
struct Iwdg {
    Iwdg() = delete;

    /// The three key values (21.4.1). Writing any of them locks the
    /// protected registers again unless it is the unlock key itself.
    static constexpr uint16_t key_refresh = 0xAAAAu;
    static constexpr uint16_t key_unlock = 0x5555u;
    static constexpr uint16_t key_start = 0xCCCCu;

    /// This peripheral has NO bus clock enable: there is no IWDGEN bit
    /// in any RCC_APBxENR, and it answers its registers out of reset.

    // ---- the keys ---------------------------------------------------------

    /// Reload the counter from RLR - the verb a program calls in its
    /// loop. Also RE-LOCKS PR and RLR, which is 21.3.2's own sentence
    /// ("a write access to this register with a different value breaks
    /// the sequence ... This implies that it is the case of the reload
    /// operation") and the reason configure() unlocks each time.
    [[gnu::always_inline]] static void refresh() { IWDG->KR = key_refresh; }

    /// Open the write window on PR and RLR.
    static void unlock() { IWDG->KR = key_unlock; }

    /// Start the watchdog. ONE WAY IN SOFTWARE: nothing this program can
    /// write will stop it again - a RESET does, measured (fact 4) - so
    /// what a caller commits to is this run of the program, not the
    /// board's remaining life.
    static void start() { IWDG->KR = key_start; }

    /**
     * The nearest thing this silicon has to "is the watchdog running":
     * the LSI standing READY with LSION clear, since a started IWDG
     * forces the oscillator on and cannot be stopped (7.2.9), and no
     * other verb of this chapter can bring that about.
     *
     * A WITNESS AND NOT A PROOF: a program that set LSION itself hides
     * the evidence, and nothing here can tell a watchdog started by the
     * WDG_SW option byte from one this program armed. The other
     * candidate requestor does NOT spoil it, measured: a backup domain
     * holding RTCEN with RTCSEL naming the LSI leaves LSIRDY clear on
     * this family, so an RTC's standing select is not mistaken for a
     * watchdog. A program that must KNOW keeps its own mark in
     * reset-surviving storage - which is what the bench suite does.
     */
    static bool running() { return Rcc::lsi_ready() && !Rcc::lsi_enabled(); }

    // ---- the update bits (21.4.4) ------------------------------------------

    static constexpr uint32_t update_mask = IWDG_SR_PVU_Msk | IWDG_SR_RVU_Msk;

    static uint32_t status() { return IWDG->SR; }
    static bool busy(uint32_t mask = update_mask) {
        return (IWDG->SR & mask) != 0u;
    }

    /// Bounded wait for an update to cross into the VDD domain. The
    /// bound is a safety net and not a timing model: five prescaled LSI
    /// cycles is at most a few hundred microseconds, and this loop's
    /// body is several instructions at 180 MHz. What it really buys is
    /// the two shapes fact 5 and the errata describe - a watchdog whose
    /// clock never comes, and the errata's flag that never clears -
    /// turning a hang into a false.
    static constexpr uint32_t sync_spin_limit = 4'000'000UL;

    static bool sync(uint32_t mask = update_mask) {
        for (uint32_t spin = 0; spin < sync_spin_limit; ++spin) {
            if (!busy(mask)) {
                return true;
            }
        }
        return false;
    }

    // ---- readback ----------------------------------------------------------
    //
    // Each of these is valid only while its own update bit is clear
    // (21.4.2 and 21.4.3 say so in two identical Notes); the caller
    // checks busy() or has just come back from a successful configure().

    static IwdgPrescaler prescaler() {
        const uint32_t pr = IWDG->PR & IWDG_PR_PR_Msk;
        // Code 7 is a second spelling of /256; report the one this
        // driver writes, so a readback can be compared with what was
        // asked for.
        return static_cast<IwdgPrescaler>(pr == 7u ? 6u : pr);
    }
    static uint8_t prescaler_bits() {
        return static_cast<uint8_t>(IWDG->PR & IWDG_PR_PR_Msk);
    }
    static uint16_t reload() {
        return static_cast<uint16_t>(IWDG->RLR & IWDG_RLR_RL_Msk);
    }

    // ---- configuration -----------------------------------------------------

    /**
     * Write the timing, WITHOUT starting the watchdog.
     *
     * False - and possibly a half-written configuration - when the
     * setting is impossible or when an update never crossed, which on a
     * STOPPED watchdog is always (fact 5: the block has no clock until
     * the start key). The order is the chapter's: wait for a previous
     * update, unlock, prescaler, reload, wait for both.
     */
    static bool configure(const IwdgConfig& cfg) {
        if (!iwdg_config_valid(cfg)) {
            return false;
        }
        if (!sync()) {
            return false;
        }
        unlock();
        IWDG->PR = static_cast<uint32_t>(cfg.prescaler);
        IWDG->RLR = cfg.reload;
        return sync();
    }

    /// Compile-time twin: an impossible setting is a build error.
    template <IwdgConfig cfg>
    static bool configure() {
        static_assert(iwdg_config_valid(cfg),
                      "IwdgConfig: RL is a twelve-bit field (RM0090 21.4.3). This "
                      "family's IWDG has no window register, so a reload is the "
                      "whole timing and any value from 0 to 0x0FFF is legal");
        return configure(cfg);
    }

    /**
     * Start the watchdog and give it this timing - the chapter's own
     * order, start first (21.3, and ST's own initialization sequence).
     *
     * ONE WAY UNTIL THE NEXT RESET. After this the device reboots unless
     * something refreshes inside the time-out. The closing refresh loads
     * the new reload into the counter, which is safe on this family
     * because there is no window to be above (fact 3).
     */
    static bool arm(const IwdgConfig& cfg) {
        if (!iwdg_config_valid(cfg)) {
            return false;
        }
        start();
        if (!configure(cfg)) {
            return false;
        }
        refresh();
        return true;
    }

    /**
     * Reset the device on purpose, the only way this family's IWDG can:
     * start it, program the SHORTEST time-out (/4 with a reload of zero,
     * table 107's 0.125 ms at the nominal LSI) and refresh into it.
     *
     * NOT INSTANTANEOUS and not [[noreturn]]: the counter is on the LSI,
     * so the CPU executes whatever follows for a few hundred
     * microseconds. A caller that means to end here says so with a spin
     * of its own.
     *
     * False when the reload could not be made to cross into the LSI
     * domain, in which case the watchdog is running with whatever timing
     * it had - which still ends in a reset, only later.
     */
    static bool force_reset() {
        start();
        const bool timed = configure(IwdgConfig{IwdgPrescaler::div4, 0});
        refresh();
        return timed;
    }

    // ---- debug -------------------------------------------------------------

    /// DBGMCU_APB1_FZ.DBG_IWDG_STOP (21.3.3): whether the counter
    /// freezes while the core is HALTED by a debugger. READ ONLY here,
    /// and deliberately: the bit is the debugger's - OpenOCD's
    /// stm32f4x.cfg sets it at every attach - a program that fights it
    /// hides the very behaviour a measurement is after, and a suite that
    /// measures a time-out needs to know which way it stands.
    ///
    /// It costs nothing to read: DBGMCU sits at 0xE0042000, in the
    /// core's private peripheral space, with no bus-clock gate in front
    /// of it (unlike the APB-mounted debug blocks of other families).
    /// It is reset by a power-on reset only (38.16), so what a debugger
    /// wrote stands across every warm boot.
    static bool debug_frozen() {
        return (DBGMCU->APB1FZ & DBGMCU_APB1_FZ_DBG_IWDG_STOP) != 0u;
    }
};

// =============================================================================
// WWDG (RM0090 ch. 22)
// =============================================================================

/// WWDG_CFR.WDGTB: a second divider after the fixed PCLK1/4096. TWO bits
/// on this family - four codes, and the reserve's
/// `wwdg_prescaler_codes()` reads the width off the device header.
enum class WwdgPrescaler : uint8_t {
    div1 = 0,
    div2 = 1,
    div4 = 2,
    div8 = 3,
};

static_assert(wwdg_prescaler_codes() == 4,
              "stm32f4/reset.hpp: WWDG_CFR.WDGTB is two bits wide on this family "
              "(RM0090 22.6.2) and WwdgPrescaler spells its four codes - a header "
              "with a wider field would need more of them");

/// PCLK1 cycles per decrement of the 7-bit counter: 4096 x 2^WDGTB.
constexpr uint32_t wwdg_step_cycles(WwdgPrescaler p) {
    return 4096UL << static_cast<uint8_t>(p);
}

/**
 * Microseconds from a counter value `t` down to the reset, at
 * `pclk1_hz` - 22.4's formula with T[5:0] + 1 spelled as t - 0x3F, which
 * is what it means for the T6-set values (0x40..0x7F) the register may
 * hold.
 *
 * KEPT IN 32 BITS AND IT NEED NOT COST ANYTHING: the whole cycle count
 * is formed first and divided by the megahertz last, and the widest it
 * can be is 4096 x 8 x 64 = 2097152 cycles. The only inexactness is the
 * truncation of the final divide, and a rate that is not a whole number
 * of megahertz is refused with a zero rather than answered wrongly.
 *
 * THE CHAPTER'S TABLE, NOT ITS WORKED EXAMPLE. This reproduces table
 * 109 exactly (30 MHz: 136 us at /1 with T[5:0] = 0, 69905 us at /8 with
 * T[5:0] = 0x3F, against a printed 136.53 us and 69.91 ms). 22.4's
 * worked example prints 21.85 ms where its own formula gives 87.38 ms
 * for the same three numbers; the table and the bench agree with the
 * formula (docs/stm32f4/reset.md).
 */
constexpr uint32_t wwdg_timeout_us(uint32_t pclk1_hz, WwdgPrescaler p, uint8_t t) {
    const uint32_t mhz = pclk1_hz / 1'000'000UL;
    if (mhz == 0u || t <= 0x3Fu) {
        return 0;
    }
    return wwdg_step_cycles(p) * (static_cast<uint32_t>(t) - 0x3FUL) / mhz;
}

/// A whole WWDG setting: the CFR fields. The counter itself is not here
/// - it is written by start() and refresh(), which is the register the
/// application touches at run time.
struct WwdgConfig {
    WwdgPrescaler prescaler = WwdgPrescaler::div1;
    /// WWDG_CFR.W: the high limit of the refresh window. 0x7F (the reset
    /// value) means "refresh whenever you like".
    uint8_t window = 0x7Fu;
    /// WWDG_CFR.EWI: interrupt when the counter reaches 0x40, one step
    /// before the reset. One-way like WDGA - "only cleared by hardware
    /// after a reset" (22.6.2).
    bool early_wakeup = false;
};

/// Seven-bit field, and a window below 0x40 can never be served: a
/// refresh is legal only while the counter is at or below W AND above
/// 0x3F (22.3), so W < 0x40 leaves no legal instant at all.
/// Wwdg::force_reset() is how a caller asks for a reset deliberately.
constexpr bool wwdg_config_valid(const WwdgConfig& c) {
    return c.window <= 0x7Fu && c.window >= 0x40u &&
           static_cast<uint8_t>(c.prescaler) < wwdg_prescaler_codes();
}

/**
 * The system window watchdog: a 7-bit down-counter on PCLK1, an
 * activation bit that only a reset clears, and an interrupt one step
 * before the end.
 *
 * WHAT MAKES IT MEASURABLE WITHOUT RISK: the counter free-runs whether
 * or not WDGA is set, and EWIF is raised at 0x40 whether or not the
 * interrupt is enabled (fact 6). So configure(), refresh(), counter(),
 * in_window() and the flag are all usable on a DISARMED block - the
 * whole timing path, with no reset anywhere - and start() is the
 * separate, one-way verb that puts the reset behind it.
 */
struct Wwdg {
    Wwdg() = delete;

    /// RCC_APB1ENR.WWDGEN, and it is CLEAR AT RESET: until this is on,
    /// the registers below read as nothing. (This family's option byte
    /// selects a hardware INDEPENDENT watchdog, not a hardware window
    /// one, so nothing but software ever sets this bit.)
    static void bus_clock(bool on) { Rcc::apb1_clock(RCC_APB1ENR_WWDGEN, on); }
    static bool bus_clock() { return (RCC->APB1ENR & RCC_APB1ENR_WWDGEN) != 0u; }

    /// The early-wakeup interrupt's line: position 0 of the vector
    /// table, the WWDG's alone on every part of this family (the
    /// reserve's wwdg_irq()).
    static constexpr IRQn_Type irq() { return wwdg_irq(); }

    // ---- state -------------------------------------------------------------

    static uint32_t cr() { return WWDG->CR; }
    static uint32_t cfr() { return WWDG->CFR; }

    /// WDGA. Set by software, cleared only by a reset (22.6.1).
    static bool enabled() { return (WWDG->CR & WWDG_CR_WDGA) != 0u; }
    /// T[6:0] as it stands - a live read of a free-running counter.
    static uint8_t counter() {
        return static_cast<uint8_t>(WWDG->CR & WWDG_CR_T_Msk);
    }
    static WwdgPrescaler prescaler() {
        return static_cast<WwdgPrescaler>((WWDG->CFR & WWDG_CFR_WDGTB_Msk) >>
                                          WWDG_CFR_WDGTB_Pos);
    }
    static uint8_t window() {
        return static_cast<uint8_t>((WWDG->CFR & WWDG_CFR_W_Msk) >> WWDG_CFR_W_Pos);
    }
    static bool early_wakeup_enabled() { return (WWDG->CFR & WWDG_CFR_EWI) != 0u; }

    /// Whether a refresh would be legal RIGHT NOW: the counter at or
    /// below the window and above 0x3F (22.3). A snapshot of a moving
    /// counter and therefore a hint, not a lock - a caller racing the
    /// window arranges its timing and does not poll this.
    static bool in_window() {
        const uint8_t t = counter();
        return t <= window() && t > 0x3Fu;
    }

    // ---- configuration -----------------------------------------------------

    /**
     * Write CFR: prescaler, window, and the early-wakeup enable.
     *
     * Legal at any time - CFR is not enable-protected and this chapter
     * has no synchronization at all, both registers being on PCLK1. EWI
     * is a set-only bit, so a configuration that clears it does not
     * disarm an interrupt already enabled; that is 22.6.2's silicon and
     * not this driver's choice, and it is why `early_wakeup` reads back
     * as a promise only in the direction that can be kept.
     */
    static bool configure(const WwdgConfig& cfg) {
        if (!wwdg_config_valid(cfg)) {
            return false;
        }
        WWDG->CFR = (static_cast<uint32_t>(cfg.prescaler) << WWDG_CFR_WDGTB_Pos) |
                    (static_cast<uint32_t>(cfg.window) << WWDG_CFR_W_Pos) |
                    (cfg.early_wakeup ? WWDG_CFR_EWI : 0u);
        return true;
    }

    /// Compile-time twin.
    template <WwdgConfig cfg>
    static bool configure() {
        static_assert(wwdg_config_valid(cfg),
                      "WwdgConfig: W is a seven-bit field, a window below 0x40 can "
                      "never be served - a refresh is legal only while the counter "
                      "is at or below W and above 0x3F (RM0090 22.3) - and WDGTB is "
                      "two bits wide here. Wwdg::force_reset() is how to ask for a "
                      "reset deliberately");
        return configure(cfg);
    }

    /**
     * Reload the counter - the verb a program calls in its loop. T6 is
     * forced set, which is 22.4's Warning ("always write 1 in the T6 bit
     * to avoid generating an immediate reset") made structural: a value
     * below 0x40 cannot be reached through this verb.
     *
     * WDGA is a set-only bit, so this write cannot disarm a running
     * watchdog and does not need to read CR first.
     *
     * A refresh made while the counter is ABOVE the window is a reset,
     * which is the whole point of the peripheral: the caller times it,
     * and nothing here checks (in_window() is there for a caller that
     * wants to look).
     */
    [[gnu::always_inline]] static void refresh(uint8_t counter_value = 0x7Fu) {
        WWDG->CR = static_cast<uint32_t>(counter_value) | WWDG_CR_T_6;
    }

    /**
     * Start the watchdog with the counter at `counter_value`. ONE WAY:
     * 22.3, "it cannot be disabled again except by a reset".
     */
    static void start(uint8_t counter_value = 0x7Fu) {
        WWDG->CR = WWDG_CR_WDGA | static_cast<uint32_t>(counter_value) | WWDG_CR_T_6;
    }

    /**
     * Reset the device now, by activating the watchdog with T6 CLEAR -
     * 22.3's own note ("the T6 bit can be used to generate a software
     * reset"). Distinct from Reset::software() in what the next boot is
     * told: this one raises WWDGRSTF, so two intentions can cross a
     * reset with no surviving RAM at all.
     *
     * Not [[noreturn]]: whether the CPU retires another instruction
     * first is the silicon's business, not this driver's promise.
     */
    static void force_reset() { WWDG->CR = WWDG_CR_WDGA; }

    // ---- the early-wakeup interrupt ----------------------------------------

    /// EWIF. Raised at 0x40 whether or not the interrupt is enabled
    /// (22.6.3), which is what lets a disarmed block be timed.
    static bool flag() { return (WWDG->SR & WWDG_SR_EWIF) != 0u; }

    /// rc_w0: the flag is cleared by writing ZERO to it, and writing one
    /// has no effect - the opposite discipline to every
    /// write-one-to-clear register in this stratum.
    static void clear_flag() { WWDG->SR = 0u; }

    /// The ISR body: the app binds WWDG_IRQHandler and calls this.
    /// Returns true when the early warning was what fired, so a handler
    /// can tell that from a shared line (this line is not shared here,
    /// but the shape is the stratum's).
    ///
    /// IT DOES NOT REFRESH. Whether the warning is a "recover and carry
    /// on" or a "log and let it reset" is 22.3's own alternative, and
    /// therefore the application's policy.
    [[gnu::always_inline]] static bool isr() {
        if (!flag()) {
            return false;
        }
        clear_flag();
        return true;
    }

    // ---- debug -------------------------------------------------------------

    /// DBGMCU_APB1_FZ.DBG_WWDG_STOP (22.5), the IWDG's twin bit, read
    /// only for the same reason (Iwdg::debug_frozen()).
    static bool debug_frozen() {
        return (DBGMCU->APB1FZ & DBGMCU_APB1_FZ_DBG_WWDG_STOP) != 0u;
    }
};

// =============================================================================
// The four fault vectors (PM0214 4.3.10 .. 4.3.15)
// =============================================================================

/// What the core recorded about a fault: the three status bytes, the
/// HardFault's own register and the faulting address. Twelve bytes, so
/// an application can bank it in the same .noinit word the panic record
/// crosses a reset in - this file owns no storage.
struct FaultRecord {
    /// SCB->CFSR: MMFSR in bits 7..0, BFSR in 15..8, UFSR in 31..16.
    uint32_t cfsr = 0;
    /// SCB->HFSR: VECTTBL, FORCED (an escalated configurable fault),
    /// DEBUGEVT (a BKPT with no debugger halted on it).
    uint32_t hfsr = 0;
    /// MMFAR or BFAR, whichever CFSR says is valid; zero when neither
    /// is (address_valid()).
    uint32_t address = 0;

    constexpr bool empty() const { return cfsr == 0u && hfsr == 0u; }
    constexpr bool mem_fault() const { return (cfsr & SCB_CFSR_MEMFAULTSR_Msk) != 0u; }
    constexpr bool bus_fault() const { return (cfsr & SCB_CFSR_BUSFAULTSR_Msk) != 0u; }
    constexpr bool usage_fault() const { return (cfsr & SCB_CFSR_USGFAULTSR_Msk) != 0u; }
    /// HFSR.FORCED: a configurable fault that was disabled (or masked)
    /// and escalated to the HardFault vector.
    constexpr bool escalated() const { return (hfsr & SCB_HFSR_FORCED_Msk) != 0u; }
    constexpr bool address_valid() const {
        return (cfsr & (SCB_CFSR_MMARVALID_Msk | SCB_CFSR_BFARVALID_Msk)) != 0u;
    }
};

/**
 * The three configurable faults: the switch, the two traps and the
 * status registers.
 *
 * They are DISABLED at reset and escalate to HardFault, which is why a
 * program that never comes here sees every fault at one vector with
 * HFSR.FORCED set. Enabling them buys two things: a handler per KIND,
 * and (with the CCR traps) two conditions the core otherwise ignores -
 * a divide by zero and an unaligned access.
 *
 * NOTHING HERE IS AUTOMATIC. The kernel promise is that no interrupt
 * nests over another (docs/design/kernel.md section 11) and a fault is
 * not an interrupt: it is synchronous with the instruction that caused
 * it, so enabling these vectors changes no priority and no masking.
 */
struct Faults {
    Faults() = delete;

    /// Enable or disable the three configurable fault exceptions
    /// (SHCSR). A fault whose vector is disabled escalates to HardFault
    /// instead - the reset state, and what every brio program has seen
    /// until it calls this.
    static void enable(bool mem, bool bus, bool usage) {
        uint32_t v = SCB->SHCSR & ~(SCB_SHCSR_MEMFAULTENA_Msk |
                                    SCB_SHCSR_BUSFAULTENA_Msk |
                                    SCB_SHCSR_USGFAULTENA_Msk);
        if (mem) { v |= SCB_SHCSR_MEMFAULTENA_Msk; }
        if (bus) { v |= SCB_SHCSR_BUSFAULTENA_Msk; }
        if (usage) { v |= SCB_SHCSR_USGFAULTENA_Msk; }
        SCB->SHCSR = v;
    }
    static bool mem_enabled() { return (SCB->SHCSR & SCB_SHCSR_MEMFAULTENA_Msk) != 0u; }
    static bool bus_enabled() { return (SCB->SHCSR & SCB_SHCSR_BUSFAULTENA_Msk) != 0u; }
    static bool usage_enabled() { return (SCB->SHCSR & SCB_SHCSR_USGFAULTENA_Msk) != 0u; }

    /// CCR.DIV_0_TRP: make an integer divide by zero a UsageFault
    /// instead of a silent zero result (PM0214 4.3.6).
    static void divide_by_zero_trap(bool on) { ccr_bit(SCB_CCR_DIV_0_TRP_Msk, on); }
    static bool divide_by_zero_trap() { return (SCB->CCR & SCB_CCR_DIV_0_TRP_Msk) != 0u; }

    /// CCR.UNALIGN_TRP: make an unaligned word or halfword access a
    /// UsageFault instead of letting the bus split it.
    static void unaligned_trap(bool on) { ccr_bit(SCB_CCR_UNALIGN_TRP_Msk, on); }
    static bool unaligned_trap() { return (SCB->CCR & SCB_CCR_UNALIGN_TRP_Msk) != 0u; }

    /// The status as it stands: non-destructive, so a handler may read
    /// it as often as it likes before deciding what to do.
    static FaultRecord read() {
        FaultRecord r{};
        r.cfsr = SCB->CFSR;
        r.hfsr = SCB->HFSR;
        if ((r.cfsr & SCB_CFSR_MMARVALID_Msk) != 0u) {
            r.address = SCB->MMFAR;
        } else if ((r.cfsr & SCB_CFSR_BFARVALID_Msk) != 0u) {
            r.address = SCB->BFAR;
        }
        return r;
    }

    /// Clear the sticky bits: both registers are write-one-to-clear, so
    /// writing back what was read takes down exactly what was seen.
    static void clear() {
        const uint32_t cfsr = SCB->CFSR;
        const uint32_t hfsr = SCB->HFSR;
        SCB->CFSR = cfsr;
        SCB->HFSR = hfsr;
    }

    /// Read then clear - the pair a handler wants when it banks the
    /// wreck and resets, so the next fault's record is its own.
    static FaultRecord take() {
        const FaultRecord r = read();
        clear();
        return r;
    }

private:
    static void ccr_bit(uint32_t mask, bool on) {
        SCB->CCR = on ? (SCB->CCR | mask) : (SCB->CCR & ~mask);
    }
};

// =============================================================================
// The breadcrumb across a reset
// =============================================================================

/**
 * A panic Reporter that ends the program with a system reset instead of
 * a spin, so the breadcrumb panic() has already written is reported at
 * the next boot rather than needing a debugger to be seen.
 *
 * panic() writes the record BEFORE any reporter runs, which is what
 * makes this composable at all: by the time report() is called the
 * information is already safe.
 */
struct ResetReporter {
    static void report(PanicCode, uint8_t) { Reset::software(); }
};

/**
 * The fault handler BODY: record the wreck and reset.
 *
 * An app binds it, the driver never names a vector:
 *
 *     extern "C" void HardFault_Handler() {
 *         brio::hard_fault_reset<brio::Stm32f4Platform<>>();
 *     }
 *
 * THE SAME BODY SERVES THE OTHER THREE FAULT VECTORS where a program
 * enables them (`Faults::enable`): what it does - leave a note and
 * reboot - does not depend on which of the four the core took, and a
 * handler that wants the detail reads `Faults::read()` into storage of
 * its own BEFORE calling this.
 *
 * IT DOES NOT GO THROUGH panic(). panic() calls P::break_here(), which
 * is BKPT, and a BKPT with no debugger halted on it is a debug event
 * that escalates - taken from INSIDE a fault handler at the same
 * priority that is a LOCKUP, and a locked-up core does not reach the
 * reset. So this writes the same record panic() would, by hand, and
 * resets immediately. A debugger that IS attached halts on the fault
 * itself long before this runs.
 *
 * The record survives because .noinit is NOLOAD and the crt neither
 * loads nor zeroes it - and because the magic word makes the claim
 * checkable rather than assumed: RM0090 promises nothing about SRAM
 * across a reset.
 *
 * AN EXISTING RECORD IS NOT OVERWRITTEN, and that is what makes this
 * compose with panic(). With no debugger attached, panic()'s closing
 * BKPT escalates into exactly this handler; clobbering the record here
 * would turn every panic into a kernel_fault and throw away the code the
 * caller actually reported. So a valid record standing means the fault
 * is a CONSEQUENCE of something already diagnosed, and the only thing
 * left to do is the reset.
 */
template <Platform P>
[[noreturn]] void hard_fault_reset(uint8_t context = 0) {
    PanicRecord& r = P::panic_record();
    if (r.magic != panic_magic) {
        r = PanicRecord{panic_magic,
                        static_cast<uint8_t>(PanicCode::kernel_fault), context};
    }
    Reset::software();
}

} // namespace brio
