/*
 * reset.hpp
 *
 * Why the program is running, and how to end it on purpose (STM32G0):
 * the RCC's reset flags (RM0444 5.1, 5.4.24), the two watchdogs the
 * family carries - the independent one on its own oscillator (ch. 28)
 * and the system window watchdog on the bus clock (ch. 29) - and the
 * fault handler body that turns a crash into a message the next boot
 * can read.
 *
 * This is the other half of the panic breadcrumb. The kernel writes a
 * PanicRecord into reset-surviving storage (kernel/panic.hpp,
 * Stm32Platform::panic_record) and hands over to a Reporter whose job
 * may be "reset now and report at the next boot"; panic.hpp's contract
 * then says to cross-check the target's reset-cause register for the
 * full story. `Reset::take_flags()` IS that cross-check,
 * `Reset::software()` is one way of causing the reset, `Iwdg` and
 * `Wwdg` are two more, and `hard_fault_reset<P>()` is the body an app
 * binds to HardFault_Handler so a fault leaves a note instead of a
 * spin.
 *
 * SIX FACTS OF THIS SILICON shape everything below.
 *
 * 1. THE FLAGS ACCUMULATE, AND THERE IS NO "THE CAUSE". RCC_CSR's seven
 *    reset flags are set by hardware and "cleared by setting the RMVF
 *    bit" - nothing else clears them, and 5.4.24 says the register is
 *    "reset upon system reset, except for reset flags that are only
 *    reset upon power reset". So this register is a HISTORY, the AVR's
 *    RSTFR habit and not the SAM's exclusive RCAUSE, and the verb is
 *    read-and-clear: `take_flags()`. Worse than a history: PINRSTF is
 *    a CATCH-ALL - 5.4.24 bit 26 sets it "when a reset from the
 *    PF2-NRST pin occurs OR when a system reset is triggered by any
 *    other source" - so a lone PINRSTF is the only reading that means
 *    "the pin", and every other source raises it alongside its own bit
 *    (measured: docs/stm32g0/reset.md). That is why this file offers no
 *    `cause()` enum: with an accumulating register and a catch-all bit,
 *    the cause is not a function of the register's value. An
 *    application clears the flags when it has read them, and what it
 *    sees at the next boot is the delta.
 *
 * 2. RCC_CSR IS A SHARED REGISTER, and the two halves have two owners.
 *    Bits 31..23 are this chapter's (the flags and RMVF); bits 1..0 are
 *    the clock tree's (LSION/LSIRDY) and belong to stm32g0/clock.hpp's
 *    `Rcc`, which is where the LSI verbs live - the samc precedent,
 *    where rtc.hpp never writes OSC32KCTRL's RTCCTRL. Every write here
 *    is a read-modify-write that preserves the other owner's bits.
 *
 * 3. THE IWDG IS ON ANOTHER CLOCK, AND SOFTWARE CANNOT STOP IT - BUT A
 *    RESET CAN. It counts LSI (about 32 kHz, min 29.5 / max 34 kHz over
 *    the full range - DS13560 table 46, so a watchdog margin here is a
 *    real margin, and the bench measured 32.5 kHz on this die), and
 *    28.3.1's "Once running, the IWDG cannot be stopped" is followed
 *    elsewhere in the manual by the four words that matter: "except
 *    upon a reset" (5.3's Stop and Standby summaries). MEASURED, and
 *    against the family lore as well as against the AVR's and the
 *    SAM's habits: a boot that follows an IWDG reset outlives its own
 *    time-out several times over with nothing refreshing anything, so
 *    the reset really does stop it and a program does NOT inherit a
 *    watchdog it must feed for ever. There is still no register bit
 *    that says whether it runs - and LSIRDY standing with LSION clear
 *    is NOT that bit, whatever 5.4.24's list of requestors invites: on
 *    the bench board the RTC domain survives every system reset with
 *    RTCEN set and RTCSEL = LSI, so the oscillator is forced by
 *    something that is not a watchdog.
 *
 * 4. THE IWDG'S KEYED REGISTERS DO NOT UPDATE UNTIL IT IS STARTED, and
 *    this is the finding this chapter never states. PR, RLR and WINR
 *    are writable only after the key 0x5555 (28.3.4), and any other key
 *    value - the 0xAAAA refresh included - locks them again. Each of
 *    the three has a bit in IWDG_SR that hardware raises AT THE STORE
 *    (measured: the read right after it already sees the bit) and drops
 *    when the value has crossed into the VDD domain, and a register
 *    read while its bit stands returns the OLD value (28.4.2/3/5 say so
 *    three times). WITH THE WATCHDOG STOPPED THE BIT NEVER DROPS,
 *    LSION or no LSION - 5.2.14 is where the reason hides: "if the IWDG
 *    is started ... after the LSI oscillator temporization, the clock
 *    is provided to the IWDG", so the logic that performs the update
 *    has no clock until the start key is written. That is why every
 *    sequence in 28.3.2 begins with 0xCCCC and why arm() below starts
 *    before it configures; configure() on a stopped watchdog is bounded
 *    and answers false rather than hanging.
 *
 * 5. THE WWDG IS THE OPPOSITE PERIPHERAL IN EVERY WAY. It counts PCLK
 *    through a fixed /4096 and a programmable /2^WDGTB, it has a bus
 *    clock enable that is CLEAR AT RESET (a peripheral with no clock
 *    does not answer register reads), and its activation bit WDGA is
 *    "set by software and only cleared by hardware after a reset"
 *    (29.5.1) - one-way in software, put away by the reset, exactly
 *    like the IWDG once fact 3 is read to the end. Its down-counter is
 *    FREE-RUNNING "even if the watchdog is disabled" (29.3.3) and EWIF
 *    "is also set if the interrupt is not enabled" (29.5.3), which
 *    together make the whole TIMING path measurable with WDGA never
 *    set - the way this stratum's bench suite measures it, and the
 *    reason `configure()` and `refresh()` are separate verbs from
 *    `start()`. What is NOT reachable that way is the interrupt: with
 *    WDGA clear the flag rises and NO REQUEST IS MADE (measured), so
 *    29.2's "triggered (if enabled and the watchdog activated)" is the
 *    exact sentence and 29.5.2's bit description is the loose one.
 *
 * 6. A WINDOW THAT CANNOT BE SERVED IS REFUSED, on both. The IWDG
 *    resets if a refresh happens while the counter is above WIN, so
 *    WIN = 0 is a configuration in which no refresh is ever legal; the
 *    WWDG's refresh is legal only while the counter is at or below W
 *    and above 0x3F, so any W below 0x40 is the same trap. Both are
 *    refused by the config_valid() predicates rather than armed, the
 *    samc WdtConfig precedent (a caller that asked for a serviceable
 *    watchdog must not silently get an unserviceable one). Provoking a
 *    reset ON PURPOSE has its own spelling on each: Iwdg::force_reset()
 *    refreshes into a closed window, Wwdg::force_reset() clears T6 with
 *    WDGA set (29.3.3's own note).
 *
 * ERRATA: ES0548 Rev 3 has NO item for the IWDG, the WWDG or the reset
 * flags on either revision - the errata's only LSI item, 2.2.1, is
 * conditioned on "LSI clocks the RTC, or it clocks the clock security
 * system on LSE", neither of which this file can bring about (and its
 * own workaround is keyed on PWRRSTF, a flag this file reads).
 *
 * NOT BUILT (docs/stm32g0/reset.md carries the list): the option bytes
 * that decide the watchdogs' hardware/software mode and their behaviour
 * in Stop and Standby (IWDG_SW, IWDG_STOP, IWDG_STDBY, WWDG_SW,
 * nRST_STOP/STDBY/SHDW behind LPWRRSTF) - read-only facts here, because
 * writing FLASH_OPTR is the flash campaign's and a bad option byte
 * bricks a board; the NRST pin's three modes; and the PWR side of
 * PWRRSTF (BOR levels), which arrives with the sleep pass.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "stm32g0/clock.hpp"

namespace brio {

// =============================================================================
// The reset flags (RCC_CSR, RM0444 5.4.24)
// =============================================================================

/// One bit per reset source, exactly as RCC_CSR carries them. They
/// ACCUMULATE until RMVF is written, so these are masks and not an
/// enumeration of "the cause" (see fact 1 in this file's header).
struct ResetFlag {
    /// LPWRRSTF: an illegal Stop / Standby / Shutdown entry, and only
    /// when the matching nRST_* option byte enables that reset.
    static constexpr uint32_t low_power = RCC_CSR_LPWRRSTF;
    /// WWDGRSTF: the window watchdog bit the counter or a bad refresh.
    static constexpr uint32_t window_watchdog = RCC_CSR_WWDGRSTF;
    /// IWDGRSTF: the independent watchdog reached zero, or was refreshed
    /// outside its window.
    static constexpr uint32_t independent_watchdog = RCC_CSR_IWDGRSTF;
    /// SFTRSTF: the core's own SYSRESETREQ (Reset::software()).
    static constexpr uint32_t software = RCC_CSR_SFTRSTF;
    /// PWRRSTF: a power-on reset or a brown-out.
    static constexpr uint32_t power = RCC_CSR_PWRRSTF;
    /// PINRSTF: the NRST pin - OR ANY OTHER SYSTEM RESET SOURCE. It is
    /// a catch-all (5.4.24), so it only names the pin when it stands
    /// ALONE.
    static constexpr uint32_t pin = RCC_CSR_PINRSTF;
    /// OBLRSTF: the option byte loader restarted the device.
    static constexpr uint32_t option_loader = RCC_CSR_OBLRSTF;

    static constexpr uint32_t all =
        low_power | window_watchdog | independent_watchdog | software |
        power | pin | option_loader;

    /// The two watchdogs together, for a caller that only wants to know
    /// that one of them bit.
    static constexpr uint32_t watchdog = window_watchdog | independent_watchdog;
};

/**
 * The reset flags, and the CPU's own way of causing a reset.
 *
 * There is no reset CONTROLLER on this family: the flags live in the
 * RCC and the reset request lives in the core, so this type is the two
 * of them under one name, which is where an application looks for
 * either.
 */
struct Reset {
    Reset() = delete;

    /// The seven flags as they stand. Non-destructive: reading changes
    /// nothing, so the boot code may sample this as often as it likes.
    static uint32_t flags() { return RCC->CSR & ResetFlag::all; }

    /// Clear every flag through RMVF, preserving the LSI bits this
    /// register shares with the clock tree (fact 2).
    ///
    /// RMVF is a plain rw bit, not a strobe: it is set, the flags go,
    /// and it is put back to zero so the next boot's read of the
    /// register is clean.
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
    /// the one reading PINRSTF can be trusted for, since every other
    /// system reset raises it too.
    static constexpr bool pin_only(uint32_t bits) {
        return (bits & ResetFlag::all) == ResetFlag::pin;
    }

    /**
     * Reset the device now, through the Cortex's SYSRESETREQ. Shows up
     * as SFTRSTF (beside PINRSTF) at the next boot.
     *
     * The mechanism is the core's, not the RCC's - which is why the
     * verb lives here but the code is CMSIS's. It carries the DSBs the
     * ARM ARM asks for.
     */
    [[noreturn]] static void software() {
        NVIC_SystemReset();
        for (;;) {  // NVIC_SystemReset is __NO_RETURN; this satisfies the
        }           // compiler on any CMSIS that forgets to say so.
    }
};

// =============================================================================
// IWDG (RM0444 ch. 28)
// =============================================================================

/// IWDG_PR: the LSI divider ahead of the 12-bit down-counter. Codes 6
/// and 7 BOTH mean /256 (28.4.2), so `div256` is the only spelling of
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
/// constant of this file, exactly as freqm's reference_hz is: LSI is an
/// uncalibrated RC (DS13560 table 46: 29.5..34 kHz over the full range)
/// and a driver that pretended to know it would be lying. At the 32 kHz
/// nominal this reproduces DS13560 table 73 (0x0FFF at /4 = 512 ms).
constexpr uint32_t iwdg_nominal_ms(IwdgPrescaler p, uint16_t reload,
                                   uint32_t lsi_hz = 32'000UL) {
    return static_cast<uint32_t>(
        (iwdg_divider(p) * (static_cast<uint32_t>(reload) + 1UL) * 1000UL) / lsi_hz);
}

/// A whole IWDG setting. The reset values are the ones the silicon
/// starts from (RL and WIN both 0x0FFF), i.e. the window disabled.
struct IwdgConfig {
    IwdgPrescaler prescaler = IwdgPrescaler::div4;
    /// IWDG_RLR.RL, 12 bits: the value loaded on every refresh.
    uint16_t reload = 0x0FFF;
    /// IWDG_WINR.WIN, 12 bits. A refresh is legal only while the
    /// counter is BELOW this and above zero, so `window >= reload`
    /// (the reset value) means "no window".
    uint16_t window = 0x0FFF;
};

/// Both fields must fit twelve bits, and a window of zero is refused:
/// no counter value is ever below it, so no refresh would ever be legal
/// and the reset would be certain. Iwdg::force_reset() is how a caller
/// asks for that on purpose.
constexpr bool iwdg_config_valid(const IwdgConfig& c) {
    return c.reload <= 0x0FFFu && c.window <= 0x0FFFu && c.window != 0u;
}

/**
 * The independent watchdog: one 12-bit down-counter on LSI, four
 * registers, and no way back once it is started.
 *
 * ORDER, AND WHY IT IS THE CHAPTER'S. 28.3.2 gives two sequences -
 * start, unlock, PR, RLR, wait for SR, then either WINR (window) or a
 * refresh (no window) - and `arm()` is exactly that. `configure()` is
 * the same body WITHOUT the start, which is legal (the key protecting
 * PR/RLR/WINR has nothing to do with the start key) and is how a
 * program prepares a watchdog it may never arm, or re-arms the timing
 * of one that is already running - the boot path of any program that
 * has ever started this thing (fact 3).
 *
 * IT NEEDS TO BE STARTED TO CONFIGURE, not merely clocked (fact 4):
 * the update bits drop only once the start key has given the block its
 * clock, so configure() on a stopped watchdog writes the registers,
 * watches the bits stand, and answers false in bounded time. That is
 * the honest report of a configuration that has not taken, and it is
 * why the only useful order is arm()'s.
 */
struct Iwdg {
    Iwdg() = delete;

    /// The three key values (28.4.1). Writing any of them locks the
    /// protected registers again unless it is the unlock key itself.
    static constexpr uint16_t key_refresh = 0xAAAAu;
    static constexpr uint16_t key_unlock = 0x5555u;
    static constexpr uint16_t key_start = 0xCCCCu;

    /// This peripheral has NO bus clock enable: it is not on a bridge
    /// the RCC gates (there is no IWDGEN bit), and it answers its
    /// registers out of reset.

    // ---- the keys ---------------------------------------------------------

    /// Reload the counter from RLR. Also RE-LOCKS PR/RLR/WINR, which is
    /// 28.3.4's own sentence and the reason configure() unlocks each
    /// time.
    ///
    /// A refresh made while the counter is ABOVE the window value is a
    /// reset (28.3.2), so this verb is only safe when the window is
    /// disabled or the caller knows where the counter is.
    [[gnu::always_inline]] static void refresh() { IWDG->KR = key_refresh; }

    /// Open the write window on PR, RLR and WINR.
    static void unlock() { IWDG->KR = key_unlock; }

    /// Start the watchdog. ONE WAY IN SOFTWARE: nothing this program
    /// can write will stop it again (28.3.1). A RESET does - measured,
    /// and the four words "except upon a reset" are the manual's own
    /// (fact 3) - so what a caller commits to is this run of the
    /// program, not the board's remaining life.
    static void start() { IWDG->KR = key_start; }

    /**
     * Reset the device by refreshing into a closed window.
     *
     * NOT INSTANTANEOUS and not [[noreturn]]: the refresh crosses into
     * the LSI domain, so the CPU executes whatever follows for a few
     * tens of microseconds. A caller that means to end here says so
     * with a spin of its own - the samc Watchdog::force_reset()
     * precedent, for the same reason.
     *
     * Requires a running watchdog: with the counter stopped there is
     * nothing to compare against a window.
     */
    static void force_reset() {
        unlock();
        IWDG->WINR = 0u;   // no counter value is below zero
        refresh();
    }

    // ---- the update bits (28.4.4) ------------------------------------------

    static constexpr uint32_t update_mask =
        IWDG_SR_PVU_Msk | IWDG_SR_RVU_Msk | IWDG_SR_WVU_Msk;

    static uint32_t status() { return IWDG->SR; }
    static bool busy(uint32_t mask = update_mask) {
        return (IWDG->SR & mask) != 0u;
    }

    /// Bounded wait for an update to cross into the VDD domain. The
    /// bound is a safety net and not a timing model: five prescaled LSI
    /// cycles at /256 is about 40 ms, which at 64 MHz is a few million
    /// CPU cycles, and this loop's body is several instructions. On a
    /// STOPPED watchdog it is what turns a hang into a false, and it
    /// costs about 0.6 s at 64 MHz to say so (measured).
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
    // (28.4.2/3/5 say so in three identical Notes); the caller checks
    // busy() or has just come back from a successful configure().

    static IwdgPrescaler prescaler() {
        const uint32_t pr = IWDG->PR & IWDG_PR_PR_Msk;
        // Code 7 is a second spelling of /256; report the one this
        // driver writes so a readback can be compared with what was
        // asked for.
        return static_cast<IwdgPrescaler>(pr == 7u ? 6u : pr);
    }
    static uint8_t prescaler_bits() {
        return static_cast<uint8_t>(IWDG->PR & IWDG_PR_PR_Msk);
    }
    static uint16_t reload() {
        return static_cast<uint16_t>(IWDG->RLR & IWDG_RLR_RL_Msk);
    }
    static uint16_t window() {
        return static_cast<uint16_t>(IWDG->WINR & IWDG_WINR_WIN_Msk);
    }

    // ---- configuration -----------------------------------------------------

    /**
     * Write the timing, WITHOUT starting the watchdog.
     *
     * False - and possibly a half-written configuration - when the
     * setting is impossible or when an update never crossed, which on a
     * STOPPED watchdog is always (fact 4: the block has no clock until
     * the start key). The order is 28.3.2's: unlock, prescaler, reload,
     * wait for both, then the window, whose own write performs a
     * reload.
     *
     * THE WINDOW IS WRITTEN LAST AND ON PURPOSE. Writing WINR reloads
     * the counter (28.3.2), so it is also the refresh that makes the
     * new timing take effect - and doing it in the other order would
     * leave a counter reloaded against the old window.
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
        if (!sync(IWDG_SR_PVU_Msk | IWDG_SR_RVU_Msk)) {
            return false;
        }
        unlock();
        IWDG->WINR = cfg.window;
        return sync(IWDG_SR_WVU_Msk);
    }

    /// Compile-time twin: an impossible setting is a build error.
    template <IwdgConfig cfg>
    static bool configure() {
        static_assert(iwdg_config_valid(cfg),
                      "IwdgConfig: RL and WIN are twelve-bit fields, and a window "
                      "of zero can never be served - no counter value is below it, "
                      "so every refresh would reset (RM0444 28.3.2). Iwdg::"
                      "force_reset() is how to ask for that deliberately");
        return configure(cfg);
    }

    /**
     * Start the watchdog and give it this timing - the chapter's own
     * sequence, start first (28.3.2).
     *
     * ONE WAY UNTIL THE NEXT RESET. After this the device reboots unless
     * something refreshes inside the window; the reset that follows is
     * what puts the watchdog away again (fact 3).
     *
     * The closing refresh is issued ONLY when the window is disabled:
     * writing WINR has just reloaded the counter to RL, so with a live
     * window (window < reload) an immediate refresh would be made with
     * the counter ABOVE the window - which is a reset, and would turn
     * this verb into a suicide note.
     */
    static bool arm(const IwdgConfig& cfg) {
        if (!iwdg_config_valid(cfg)) {
            return false;
        }
        start();
        if (!configure(cfg)) {
            return false;
        }
        if (cfg.window >= cfg.reload) {
            refresh();
        }
        return true;
    }

    // ---- debug -------------------------------------------------------------

    /// DBG_APB_FZ1.DBG_IWDG_STOP: whether the counter freezes when the
    /// core is halted by a debugger (28.3.5). Zero after a POWER-ON
    /// reset - the counter keeps running, which is what makes a
    /// single-stepped program reboot under a debugger.
    ///
    /// TWO THINGS THIS REGISTER NEEDS, both measured. It answers a
    /// store only with RCC_APBENR1.DBGEN set, and that bit is CLEAR AT
    /// RESET (5.2.17: a peripheral without its bus clock does not take
    /// one) - so a caller opens that gate first, exactly as it would
    /// for any other APB peripheral. And 40.10.3 says the register is
    /// "not reset by system reset": whatever set it last - a debug
    /// session, an earlier run of the program - is still what it holds
    /// at the next boot, so a program that cares must write it and not
    /// assume it.
    static bool debug_freeze() {
        return (DBG->APBFZ1 & DBG_APB_FZ1_DBG_IWDG_STOP) != 0u;
    }
    static void debug_freeze(bool on) {
        DBG->APBFZ1 = on ? (DBG->APBFZ1 | DBG_APB_FZ1_DBG_IWDG_STOP)
                         : (DBG->APBFZ1 & ~DBG_APB_FZ1_DBG_IWDG_STOP);
    }
};

// =============================================================================
// WWDG (RM0444 ch. 29)
// =============================================================================

/// WWDG_CFR.WDGTB: a second divider after the fixed PCLK/4096.
enum class WwdgPrescaler : uint8_t {
    div1 = 0,
    div2 = 1,
    div4 = 2,
    div8 = 3,
    div16 = 4,
    div32 = 5,
    div64 = 6,
    div128 = 7,
};

/// PCLK cycles per decrement of the 7-bit counter: 4096 x 2^WDGTB.
constexpr uint32_t wwdg_step_cycles(WwdgPrescaler p) {
    return 4096UL << static_cast<uint8_t>(p);
}

/**
 * Microseconds from a counter value `t` down to the reset, at `pclk_hz`
 * - 29.3.4's formula with T[5:0] + 1 spelled as t - 0x3F, which is what
 * it means for the T6-set values (0x40..0x7F) the register may hold.
 *
 * KEPT IN 32 BITS AND IT NEED NOT COST ANYTHING: the whole cycle count
 * is formed first and divided by the megahertz last, and the widest it
 * can be is 4096 x 128 x 64 = 33554432 cycles, which fits. A 64-bit
 * intermediate would be a libcall on this core; the only inexactness
 * left is the truncation of the final divide (RM0444 29.3.4's own
 * worked example, 48 MHz at WDGTB 3, comes out 43690 us against its
 * printed 43.69 ms), and a rate that is not a whole number of megahertz
 * is refused with a zero rather than answered wrongly.
 */
constexpr uint32_t wwdg_timeout_us(uint32_t pclk_hz, WwdgPrescaler p, uint8_t t) {
    const uint32_t mhz = pclk_hz / 1'000'000UL;
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
    /// WWDG_CFR.W: the high limit of the refresh window. 0x7F (the
    /// reset value) means "refresh whenever you like".
    uint8_t window = 0x7Fu;
    /// WWDG_CFR.EWI: interrupt when the counter reaches 0x40, one step
    /// before the reset. One-way like WDGA - "cleared by hardware after
    /// a reset" (29.5.2).
    bool early_wakeup = false;
};

/// Seven-bit field, and a window below 0x40 can never be served: a
/// refresh is legal only while the counter is at or below W AND above
/// 0x3F (29.3.3), so W < 0x40 leaves no legal instant at all.
/// Wwdg::force_reset() is how a caller asks for a reset deliberately.
constexpr bool wwdg_config_valid(const WwdgConfig& c) {
    return c.window <= 0x7Fu && c.window >= 0x40u;
}

/**
 * The system window watchdog: a 7-bit down-counter on PCLK, an
 * activation bit that only a reset clears, and an interrupt one step
 * before the end.
 *
 * WHAT MAKES IT MEASURABLE WITHOUT RISK: the counter free-runs whether
 * or not WDGA is set, and EWIF is raised at 0x40 whether or not the
 * interrupt is enabled (fact 5). So configure(), refresh(), counter()
 * and the flag are all usable on a DISARMED block - the whole timing
 * path, with no reset anywhere - and start() is the separate, one-way
 * verb that puts the reset behind it.
 */
struct Wwdg {
    Wwdg() = delete;

    /// RCC_APBENR1.WWDGEN, and it is CLEAR AT RESET: until this is on,
    /// the registers below read as nothing (5.2.17). The hardware
    /// option WWDG_SW = 0 sets the same bit in hardware, which is how a
    /// hardware watchdog runs before any software has said anything.
    static void bus_clock(bool on) { Rcc::apb1_clock(RCC_APBENR1_WWDGEN, on); }
    static bool bus_clock() { return (RCC->APBENR1 & RCC_APBENR1_WWDGEN) != 0u; }

    /// The early-wakeup interrupt's line. It is NOT shared on this
    /// family: position 0 of the vector table is the WWDG's alone
    /// (RM0444 table 61).
    static constexpr IRQn_Type irq() { return WWDG_IRQn; }

    // ---- state -------------------------------------------------------------

    static uint32_t cr() { return WWDG->CR; }
    static uint32_t cfr() { return WWDG->CFR; }

    /// WDGA. Set by software, cleared only by a reset (29.5.1).
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

    // ---- configuration -----------------------------------------------------

    /**
     * Write CFR: prescaler, window, and the early-wakeup enable.
     *
     * Legal at any time - CFR is not enable-protected and this chapter
     * has no synchronization at all, both registers being on PCLK. EWI
     * is a set-only bit, so a configuration that clears it does not
     * disarm an interrupt already enabled; that is 29.5.2's silicon and
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
                      "WwdgConfig: W is a seven-bit field, and a window below 0x40 "
                      "can never be served - a refresh is legal only while the "
                      "counter is at or below W and above 0x3F (RM0444 29.3.3). "
                      "Wwdg::force_reset() is how to ask for a reset deliberately");
        return configure(cfg);
    }

    /**
     * Reload the counter. T6 is forced set, which is 29.3.3's Warning
     * ("always write 1 in the T6 bit to avoid generating an immediate
     * reset") made structural: a value below 0x40 cannot be reached
     * through this verb.
     *
     * WDGA is a set-only bit, so this write cannot disarm a running
     * watchdog and does not need to read CR first.
     *
     * A refresh made while the counter is ABOVE the window is a reset,
     * which is the whole point of the peripheral: the caller times it.
     */
    [[gnu::always_inline]] static void refresh(uint8_t counter_value = 0x7Fu) {
        WWDG->CR = static_cast<uint32_t>(counter_value) | WWDG_CR_T_6;
    }

    /**
     * Start the watchdog with the counter at `counter_value`. ONE WAY:
     * 29.3.2, "it cannot be disabled again, except by a reset".
     */
    static void start(uint8_t counter_value = 0x7Fu) {
        WWDG->CR = WWDG_CR_WDGA | static_cast<uint32_t>(counter_value) | WWDG_CR_T_6;
    }

    /**
     * Reset the device now, by activating the watchdog with T6 CLEAR -
     * 29.3.3's own note ("the T6 bit can be used to generate a software
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
    /// (29.5.3), which is what lets a disarmed block be timed.
    static bool flag() { return (WWDG->SR & WWDG_SR_EWIF) != 0u; }

    /// rc_w0: the flag is cleared by writing ZERO to it, and writing
    /// one has no effect - the opposite discipline to every
    /// write-one-to-clear register in the other two strata.
    static void clear_flag() { WWDG->SR = 0u; }

    /// The ISR body: the app binds WWDG_IRQHandler and calls this.
    /// Returns true when the early warning was what fired, so a handler
    /// can tell that from a shared line (this line is not shared here,
    /// but the shape is the stratum's).
    ///
    /// IT DOES NOT REFRESH. Whether the warning is a "recover and carry
    /// on" or a "log and let it reset" is 29.4's own alternative, and
    /// therefore the application's policy.
    [[gnu::always_inline]] static bool isr() {
        if (!flag()) {
            return false;
        }
        clear_flag();
        return true;
    }

    // ---- debug -------------------------------------------------------------

    /// DBG_APB_FZ1.DBG_WWDG_STOP (29.3.5), the IWDG's twin bit - and
    /// with the same two conditions on it: RCC_APBENR1.DBGEN must be
    /// on, and 40.10.3's register survives a system reset.
    static bool debug_freeze() {
        return (DBG->APBFZ1 & DBG_APB_FZ1_DBG_WWDG_STOP) != 0u;
    }
    static void debug_freeze(bool on) {
        DBG->APBFZ1 = on ? (DBG->APBFZ1 | DBG_APB_FZ1_DBG_WWDG_STOP)
                         : (DBG->APBFZ1 & ~DBG_APB_FZ1_DBG_WWDG_STOP);
    }
};

// =============================================================================
// Faults, and the breadcrumb across a reset
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
 * The HardFault handler BODY: record the wreck and reset.
 *
 * An app binds it, the driver never names a vector:
 *
 *     extern "C" void HardFault_Handler() {
 *         brio::hard_fault_reset<brio::Stm32Platform>();
 *     }
 *
 * IT DOES NOT GO THROUGH panic(), and the reason is specific to this
 * core. panic() calls P::break_here(), which is BKPT, and on ARMv6-M a
 * BKPT with no debugger attached escalates to HardFault - taken from
 * INSIDE HardFault that is a LOCKUP, and a locked-up core does not
 * reach the reset. So this writes the same record panic() would, by
 * hand, and resets immediately. A debugger that IS attached halts on
 * the fault itself long before this runs.
 *
 * The record survives because .noinit is NOLOAD and the crt neither
 * loads nor zeroes it - and because the magic word makes the claim
 * checkable rather than assumed, which matters here as much as on the
 * SAM: RM0444 promises nothing about SRAM across a reset, and this
 * family can additionally be told by option byte to raise an NMI on the
 * first read of a never-written word (platform_stm32.hpp says where).
 *
 * AN EXISTING RECORD IS NOT OVERWRITTEN, and that is what makes this
 * compose with panic(). With no debugger attached, panic()'s closing
 * BKPT escalates into exactly this handler; clobbering the record here
 * would turn every panic into a kernel_fault and throw away the code
 * the caller actually reported. So a valid record standing means the
 * fault is a CONSEQUENCE of something already diagnosed, and the only
 * thing left to do is the reset.
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
