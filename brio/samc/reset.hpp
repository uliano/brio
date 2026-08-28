/*
 * reset.hpp
 *
 * RSTC (DS60001479M ch. 18) and the WDT (ch. 23) of the SAM C21: why the
 * program is running, and how to end it on purpose - plus the fault
 * handler body that turns a crash into a message the next boot can read.
 *
 * This is the other half of the panic breadcrumb. The kernel writes a
 * PanicRecord into reset-surviving storage (kernel/panic.hpp,
 * SamPlatform::panic_record) and hands over to a Reporter whose job may
 * be "reset now and report at the next boot"; panic.hpp's own contract
 * then says to cross-check the reset-cause register on the target for
 * the full story. `Reset::cause()` IS that cross-check,
 * `Reset::software()` is one way of causing the reset, `Watchdog` is
 * another, and `hard_fault_reset<P>()` is the body an app binds to
 * HardFault_Handler so that a fault leaves a note instead of a spin.
 *
 * FOUR FACTS OF THIS SILICON shape everything below, and the first is
 * the one that most needs saying because the AVR family taught the
 * opposite habit.
 *
 * 1. RCAUSE IS EXCLUSIVE, NOT CUMULATIVE. "When a Reset occurs, the bit
 *    corresponding to the Reset source is set to '1' and all other bits
 *    are written to '0'" (18.8.1). It is read-only, there is nothing to
 *    clear, and it always describes exactly one reset - the last one.
 *    The AVR's RSTFR accumulates history and needs a read-and-clear verb
 *    at boot; porting that habit here would be writing to a read-only
 *    register and reading a history that does not exist.
 *
 * 2. NOT EVERY RESET RESETS EVERYTHING (table 18-1). Only a power-supply
 *    reset (POR, BODVDD, BODCORE) clears the whole device. An external
 *    reset spares the RTC, OSC32KCTRL, RSTC itself and a WRTLOCK'd GCLK;
 *    a WDT reset or a system reset request spares those AND the debug
 *    logic. So a watchdog reset leaves a running RTC running - which is
 *    a feature when something must keep time across a recovery, and a
 *    trap for code that assumes a reset is a clean slate.
 *
 *    SRAM APPEARS IN NO ROW OF THAT TABLE, for any source including
 *    power-on. Nothing promises the breadcrumb survives; the magic word
 *    take_panic_record() checks is therefore necessary and not merely
 *    prudent, and the linker script's NOLOAD .noinit is what keeps the
 *    crt from zeroing it.
 *
 * 3. THE WDT RUNS ON ITS OWN OSCILLATOR AND ITS RESET VALUES ARE FUSES.
 *    The counter is clocked at 1.024 kHz from OSCULP32K, deliberately
 *    inaccurate ("the exact time-out period may vary from device-to-
 *    device", 23.5.3) - a watchdog margin here is a real margin, not a
 *    rounding. And CTRLA.ENABLE, CTRLA.ALWAYSON, CTRLA.WEN, CONFIG.PER,
 *    CONFIG.WINDOW and EWCTRL.EWOFFSET are all loaded from the NVM User
 *    Row at power-on (23.6.2.2): what this driver sees at boot is what
 *    the fuse row says, and samc/nvm.hpp's `NvmUserRow` is where to read
 *    that intent from. The two must agree, and the bench suite checks
 *    that they do.
 *
 * 4. THE CLEAR REGISTER IS A LOADED GUN. Writing 0xA5 restarts the
 *    period; "writing any other value than 0xA5 to CLEAR will issue an
 *    immediate system reset" (23.6.2.4). That makes a wrong key a second
 *    way to reset on purpose - and, unlike `Reset::software()`, one the
 *    reset controller attributes to the watchdog. `Watchdog::clear()`
 *    and `Watchdog::force_reset()` are the two halves of that, spelled
 *    apart so neither can happen by accident.
 *
 * ERRATA: neither RSTC nor the WDT has an item in DS80000740S - there is
 * no section for either. The watchdog does appear inside someone else's
 * workaround: 1.22.1 (XOSC/XOSC32K clock-failure detection cannot switch
 * to the safe clock when the input is stuck high) tells the application
 * to run the WDT and switch clocks in firmware after the WDT reset. That
 * matters to the clock pass, when XOSC exists.
 *
 * REGISTER ACCESS PROTECTION: RCAUSE and every writable WDT register but
 * INTFLAG are PAC write-protected (18.5.8, 23.5.8). PAC protection is off
 * out of reset and no brio driver enables it, so nothing here unlocks
 * anything; a future PAC driver must.
 *
 * NOT BUILT (docs/samc/platform.md carries the list): the SUPC side of
 * the story - BODVDD and BODCORE are reset SOURCES named here and
 * configured there, and SUPC has no driver yet - and the vector table
 * relocation (VTOR), which a bootloader would want and nothing else does.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "samc/clock.hpp"

namespace brio {

// =============================================================================
// RSTC
// =============================================================================

/// What brought the program here. RCAUSE names exactly one source, so
/// this is an enum and not a flag set - the difference from the AVR's
/// accumulating RSTFR is the point.
enum class ResetCause : uint8_t {
    unknown = 0,      ///< no bit set: not reachable on real silicon after a reset
    power_on,         ///< POR
    brown_out_core,   ///< BODCORE - the core regulator's detector
    brown_out_vdd,    ///< BODVDD - the supply's detector
    external,         ///< EXT - the RESET pin pulled low
    watchdog,         ///< WDT - a time-out, a window violation, or a bad CLEAR key
    system_request,   ///< SYST - the CPU's own SYSRESETREQ (Reset::software)
};

/**
 * The reset controller. One read-only register, no configuration, no
 * interrupts, no events, and active in every sleep mode (18.6.x): almost
 * everything this type offers is interpretation.
 */
struct Reset {
    Reset() = delete;

    /// RCAUSE as it stands. Read-only and never cleared by software.
    static uint8_t cause_bits() { return RSTC_REGS->RSTC_RCAUSE; }

    static ResetCause cause() {
        const uint8_t r = cause_bits();
        if (r & RSTC_RCAUSE_POR_Msk) return ResetCause::power_on;
        if (r & RSTC_RCAUSE_BODCORE_Msk) return ResetCause::brown_out_core;
        if (r & RSTC_RCAUSE_BODVDD_Msk) return ResetCause::brown_out_vdd;
        if (r & RSTC_RCAUSE_EXT_Msk) return ResetCause::external;
        if (r & RSTC_RCAUSE_WDT_Msk) return ResetCause::watchdog;
        if (r & RSTC_RCAUSE_SYST_Msk) return ResetCause::system_request;
        return ResetCause::unknown;
    }

    /// The two groups table 18-1 divides the sources into, because they
    /// answer different questions: a power-supply reset means the whole
    /// device was cleared and nothing at all survived, while a user
    /// reset left the RTC, OSC32KCTRL, RSTC and a WRTLOCK'd GCLK alone.
    static bool power_supply_reset() {
        return (cause_bits() & (RSTC_RCAUSE_POR_Msk | RSTC_RCAUSE_BODVDD_Msk |
                                RSTC_RCAUSE_BODCORE_Msk)) != 0u;
    }
    static bool user_reset() {
        return (cause_bits() & (RSTC_RCAUSE_EXT_Msk | RSTC_RCAUSE_WDT_Msk |
                                RSTC_RCAUSE_SYST_Msk)) != 0u;
    }

    /// True when this reset left the peripherals that survive a user
    /// reset still running - the RTC above all. Code that must not
    /// assume a clean slate asks this.
    static bool warm() { return user_reset(); }

    /**
     * Reset the device now, through the CPU's SYSRESETREQ. Shows up as
     * ResetCause::system_request at the next boot.
     *
     * This is the Cortex's reset, not RSTC's - RSTC has no control
     * register at all, which is why the verb lives here but the
     * mechanism is CMSIS's. It carries the DSB the ARM ARM asks for.
     */
    [[noreturn]] static void software() {
        NVIC_SystemReset();
        for (;;) {  // NVIC_SystemReset is __NO_RETURN; this satisfies the
        }           // compiler on any CMSIS that forgets to say so.
    }
};

// =============================================================================
// WDT
// =============================================================================

/// CONFIG.PER, CONFIG.WINDOW and EWCTRL.EWOFFSET all use this one
/// encoding: a count of 1.024 kHz CLK_WDT_OSC cycles. 8 cycles is about
/// 8 ms and 16384 about 16 s - "about", because OSCULP32K is not
/// accurate by design.
enum class WdtCycles : uint8_t {
    cyc8 = WDT_CONFIG_PER_CYC8_Val,
    cyc16 = WDT_CONFIG_PER_CYC16_Val,
    cyc32 = WDT_CONFIG_PER_CYC32_Val,
    cyc64 = WDT_CONFIG_PER_CYC64_Val,
    cyc128 = WDT_CONFIG_PER_CYC128_Val,
    cyc256 = WDT_CONFIG_PER_CYC256_Val,
    cyc512 = WDT_CONFIG_PER_CYC512_Val,
    cyc1024 = WDT_CONFIG_PER_CYC1024_Val,
    cyc2048 = WDT_CONFIG_PER_CYC2048_Val,
    cyc4096 = WDT_CONFIG_PER_CYC4096_Val,
    cyc8192 = WDT_CONFIG_PER_CYC8192_Val,
    cyc16384 = WDT_CONFIG_PER_CYC16384_Val,
};

/// Nominal milliseconds for a period field, at the oscillator's nominal
/// 1.024 kHz: 8 << field cycles, a cycle being 1/1.024 ms. ROUNDED to
/// nearest, which is what makes the endpoints read as the chapter's own
/// "8ms to 16s" (8 cycles is 7.8125 ms; truncation would print 7). It is
/// arithmetic about the nominal rate and NOT a promise about a
/// particular die - OSCULP32K is inaccurate by design, 23.5.3.
constexpr uint32_t wdt_nominal_ms(WdtCycles c) {
    return ((8UL << static_cast<uint8_t>(c)) * 1000UL + 512UL) / 1024UL;
}

/// INTFLAG / INTENSET / INTENCLR: the WDT has exactly one interrupt.
struct WdtFlag {
    static constexpr uint8_t early_warning = WDT_INTFLAG_EW_Msk;
};

/**
 * The watchdog's configuration. Every field here is enable-protected -
 * writable only while the WDT is disabled (23.6.2.1) - and every one of
 * them also has a power-on value taken from the NVM User Row.
 */
struct WdtConfig {
    /// CONFIG.PER: the time-out in Normal mode, the OPEN window in
    /// Window mode.
    WdtCycles period = WdtCycles::cyc16384;

    /// CTRLA.WEN. In Window mode a clear BEFORE the window opens is
    /// itself a reset, so the watchdog polices both edges of the loop.
    bool window_mode = false;
    /// CONFIG.WINDOW: the CLOSED window, only meaningful with window_mode.
    WdtCycles window = WdtCycles::cyc8;

    /// INTENSET.EW plus EWCTRL.EWOFFSET. In Normal mode the offset is
    /// counted from the START of the period, so it must be SHORTER than
    /// `period` or the reset arrives first and the interrupt never does
    /// (23.6.8.2). In Window mode the offset is ignored: the interrupt
    /// is raised when the window opens.
    bool early_warning = false;
    WdtCycles ew_offset = WdtCycles::cyc8;

    /**
     * CTRLA.ALWAYSON - and it is ONE-WAY: once written it is cleared
     * only by a power-on reset, CONFIG and EWCTRL become read-only, and
     * the watchdog can never be disabled again. A program that sets this
     * by accident makes a board that resets itself until it is
     * unplugged. Exposed because the chapter has it and because the fuse
     * row can set it anyway; never set by any test.
     */
    bool always_on = false;
};

/**
 * The watchdog.
 *
 * WHAT IS SYNCHRONIZED, AND WHAT THAT COSTS. CTRLA.ENABLE, CTRLA.WEN,
 * CTRLA.ALWAYSON and CLEAR cross into the 1.024 kHz domain, so each has
 * a SYNCBUSY bit and a write is not in force until it clears (23.6.7).
 * CONFIG and EWCTRL do not - they are read directly by the counter
 * logic, which is why they are enable-protected instead. Every verb
 * below that touches a synchronized bit waits for it, bounded: at
 * 1.024 kHz a synchronization is on the order of a few tens of
 * microseconds, and a wait that never ends is a false return rather than
 * a hang.
 */
struct Watchdog {
    Watchdog() = delete;

    /// This block's bit in MCLK.APBAMASK. On at reset (APBAMASK's reset
    /// value is 0xFFF, every bit of the A bridge), so nothing has to
    /// enable it - the verb exists for a power pass that wants it off.
    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_WDT_Msk, on); }

    // ---- synchronization --------------------------------------------------

    static constexpr uint32_t sync_mask =
        WDT_SYNCBUSY_ENABLE_Msk | WDT_SYNCBUSY_WEN_Msk |
        WDT_SYNCBUSY_ALWAYSON_Msk | WDT_SYNCBUSY_CLEAR_Msk;

    static bool busy(uint32_t mask = sync_mask) {
        return (WDT_REGS->WDT_SYNCBUSY & mask) != 0u;
    }

    /// Bounded wait. The bound is a safety net and not a timing model:
    /// one 1.024 kHz cycle is ~46000 CPU cycles at 48 MHz, so this is
    /// comfortably several of them.
    static constexpr uint32_t sync_spin_limit = 1'000'000UL;

    static bool sync(uint32_t mask = sync_mask) {
        for (uint32_t spin = 0; spin < sync_spin_limit; ++spin) {
            if (!busy(mask)) {
                return true;
            }
        }
        return false;
    }

    // ---- state ------------------------------------------------------------

    static uint8_t ctrla() { return WDT_REGS->WDT_CTRLA; }
    static bool enabled() { return (ctrla() & WDT_CTRLA_ENABLE_Msk) != 0u; }
    static bool window_mode() { return (ctrla() & WDT_CTRLA_WEN_Msk) != 0u; }
    /// CTRLA.ALWAYSON. Once true, nothing but a power-on reset changes it.
    static bool always_on() { return (ctrla() & WDT_CTRLA_ALWAYSON_Msk) != 0u; }

    static uint8_t config() { return WDT_REGS->WDT_CONFIG; }
    static WdtCycles period() {
        return static_cast<WdtCycles>((config() & WDT_CONFIG_PER_Msk) >>
                                      WDT_CONFIG_PER_Pos);
    }
    static WdtCycles window() {
        return static_cast<WdtCycles>((config() & WDT_CONFIG_WINDOW_Msk) >>
                                      WDT_CONFIG_WINDOW_Pos);
    }
    static WdtCycles ew_offset() {
        return static_cast<WdtCycles>((WDT_REGS->WDT_EWCTRL &
                                       WDT_EWCTRL_EWOFFSET_Msk) >>
                                      WDT_EWCTRL_EWOFFSET_Pos);
    }

    // ---- configuration ----------------------------------------------------

    /**
     * Is this a configuration the silicon can hold?
     *
     * The one combination that is legal to write and useless to run:
     * Normal mode with an early-warning offset at or past the period.
     * 23.6.8.2 spells out the consequence - "the watchdog time-out
     * system reset is generated prior to the Early Warning interrupt.
     * Consequently, the Early Warning interrupt will never be
     * generated" - so a caller that asked for a warning would silently
     * get none. Window mode is exempt: there the offset is not used at
     * all, the interrupt marking the window opening instead.
     */
    static constexpr bool config_valid(const WdtConfig& c) {
        return !(c.early_warning && !c.window_mode &&
                 static_cast<uint8_t>(c.ew_offset) >=
                     static_cast<uint8_t>(c.period));
    }

    /// Compile-time arming: an impossible configuration is a build error.
    template <WdtConfig cfg>
    static bool arm() {
        static_assert(config_valid(cfg),
                      "WdtConfig: in Normal mode the early-warning offset must "
                      "be SHORTER than the period, or the reset arrives first "
                      "and the interrupt never does (23.6.8.2)");
        return write_config(cfg);
    }

    /// Run-time arming. False - and nothing written - for an impossible
    /// configuration, for a watchdog already in always-on mode (CONFIG
    /// and EWCTRL are read-only there, 23.6.8.1), or for a
    /// synchronization that never completed.
    static bool arm(const WdtConfig& cfg) {
        if (!config_valid(cfg)) {
            return false;
        }
        return write_config(cfg);
    }

    /**
     * Disable the watchdog. Refused - false, nothing written - when
     * always-on is set: 23.6.2.3 says the WDT can be disabled only if
     * ALWAYSON is '0', and pretending otherwise would leave a caller
     * believing a live watchdog is off.
     */
    static bool disable() {
        if (always_on()) {
            return false;
        }
        if (!sync(WDT_SYNCBUSY_ENABLE_Msk)) {
            return false;
        }
        WDT_REGS->WDT_CTRLA = static_cast<uint8_t>(ctrla() & ~WDT_CTRLA_ENABLE_Msk);
        return sync(WDT_SYNCBUSY_ENABLE_Msk);
    }

    // ---- feeding it -------------------------------------------------------

    /**
     * Restart the time-out period. THE ONLY correct value is the key
     * 0xA5; this verb never writes anything else, which is why
     * force_reset() below is a separate name.
     *
     * Not synchronized on the way in - the write is posted - so a caller
     * that feeds the dog in a tight loop does not pay a synchronization
     * each time. `sync()` is there for a caller that needs to know the
     * clear has landed.
     */
    [[gnu::always_inline]] static void clear() {
        WDT_REGS->WDT_CLEAR = WDT_CLEAR_CLEAR_KEY_Val;
    }

    /**
     * Reset the device now, by writing a deliberately wrong key.
     * 23.6.2.4: "writing any other value than 0xA5 to CLEAR will issue
     * an immediate system reset."
     *
     * Distinct from Reset::software() in what the next boot is told: the
     * reset controller attributes this one to the WATCHDOG, so a program
     * can use the two to signal two different intentions across a reset
     * without any surviving RAM at all.
     *
     * TWO MEASURED FACTS, both wider than the chapter's sentence, which
     * sits inside the Normal-mode section and could be read as requiring
     * a running watchdog:
     *
     *  - it bites whether the watchdog is ENABLED OR NOT. A wrong key
     *    written with CTRLA.ENABLE clear resets the device just the
     *    same, and RCAUSE calls it a watchdog reset either way.
     *  - it is NOT instantaneous. CLEAR is one of the four
     *    write-synchronized registers (23.6.7), so the reset arrives a
     *    few CLK_WDT_OSC cycles later - a millisecond or three at
     *    1.024 kHz - and the CPU executes whatever follows in the
     *    meantime.
     *
     * Hence not [[noreturn]]: this verb DOES return, and a caller that
     * means to end here must say so itself, with a spin or by putting
     * nothing after it that matters.
     */
    static void force_reset() { WDT_REGS->WDT_CLEAR = 0x00u; }

    // ---- the early-warning interrupt --------------------------------------

    static uint8_t flags() { return WDT_REGS->WDT_INTFLAG; }
    static uint8_t armed() { return WDT_REGS->WDT_INTENSET; }
    /// Write-one-to-clear. INTFLAG is the one WDT register PAC never
    /// protects (23.5.8).
    static void clear_flags(uint8_t mask = WdtFlag::early_warning) {
        WDT_REGS->WDT_INTFLAG = mask;
    }
    static bool early_warning_flag() {
        return (flags() & WdtFlag::early_warning) != 0u;
    }
    static void arm_interrupt(bool on) {
        if (on) {
            WDT_REGS->WDT_INTENSET = WdtFlag::early_warning;
        } else {
            WDT_REGS->WDT_INTENCLR = WdtFlag::early_warning;
        }
    }

    /// The ISR body: the app binds WDT_Handler and calls this. Returns
    /// the flags it acknowledged, so a handler can tell "the warning
    /// fired" from "something else woke me".
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t p = static_cast<uint8_t>(flags() & armed());
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    /// The shared body of the two arm() forms. Order matters and is the
    /// chapter's: disable first (CONFIG and EWCTRL are enable-protected),
    /// then the unsynchronized registers, then ENABLE with the
    /// enable-protected CTRLA bits written alongside it - which
    /// 23.6.2.1 permits when ENABLE goes to '1', and only then.
    static bool write_config(const WdtConfig& cfg) {
        if (always_on()) {
            return false;
        }
        if (!disable()) {
            return false;
        }

        WDT_REGS->WDT_CONFIG = static_cast<uint8_t>(
            WDT_CONFIG_PER(static_cast<uint8_t>(cfg.period)) |
            WDT_CONFIG_WINDOW(static_cast<uint8_t>(cfg.window)));
        WDT_REGS->WDT_EWCTRL = static_cast<uint8_t>(
            WDT_EWCTRL_EWOFFSET(static_cast<uint8_t>(cfg.ew_offset)));

        arm_interrupt(cfg.early_warning);
        clear_flags();

        WDT_REGS->WDT_CTRLA = static_cast<uint8_t>(
            WDT_CTRLA_ENABLE_Msk |
            (cfg.window_mode ? WDT_CTRLA_WEN_Msk : 0u) |
            (cfg.always_on ? WDT_CTRLA_ALWAYSON_Msk : 0u));
        return sync();
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
 *         brio::hard_fault_reset<brio::SamPlatform>();
 *     }
 *
 * IT DOES NOT GO THROUGH panic(), and the reason is specific to this
 * core. panic() calls P::break_here(), which is BKPT, and on ARMv6-M a
 * BKPT with no debugger attached escalates to HardFault - taken from
 * INSIDE HardFault that is a LOCKUP, and a locked-up core does not reach
 * the reset. So this writes the same record panic() would, by hand, and
 * resets immediately. A debugger that IS attached halts on the fault
 * itself long before this runs.
 *
 * The record survives because .noinit is NOLOAD and the crt neither
 * loads nor zeroes it - and because the magic word makes the claim
 * checkable rather than assumed, which matters here more than anywhere:
 * table 18-1 promises nothing about SRAM.
 *
 * AN EXISTING RECORD IS NOT OVERWRITTEN, and that is what makes this
 * compose with panic(). panic() writes its diagnosis and then calls
 * P::break_here(), which is BKPT - and on ARMv6-M with no debugger
 * attached a BKPT escalates to exactly this handler. Clobbering the
 * record here would turn every panic into a kernel_fault and throw away
 * the code the caller actually reported. So: a valid record standing
 * means the fault is a CONSEQUENCE of something already diagnosed, and
 * the only thing left to do is the reset.
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
