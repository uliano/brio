/*
 * reset.hpp
 *
 * Why the chip last started, and how to make it start again (datasheet
 * 2.12 chip-level reset, 2.13 the power-on state machine, 4.7 the
 * watchdog's REASON, 2.8 the bootrom) - the failing half of the
 * platform beside rp2040/platform.hpp's running half. A program reads
 * the causes and the panic breadcrumb once at boot, binds the HardFault
 * body, and lets a panic reboot the chip (docs/rp2040/reset.md):
 *
 *     const uint32_t causes = brio::Reset::causes();
 *     const auto record = brio::take_panic_record<P>();     // once, at boot
 *
 *     extern "C" void isr_hardfault() { brio::fault_reset<P>(); }
 *     brio::panic<P, brio::ResetReporter>(brio::PanicCode::assert_failed, 7);
 *
 * Who calls what: causes() from any context, read-only (reading changes
 * nothing); Reset::software() and Reset::core() from any context and on
 * either core, neither returning and neither waiting for more than the
 * reset to land; fault_reset<P>() is the body of the program's
 * isr_hardfault and runs in the fault; ResetReporter is a panic()
 * argument, called by the panic path.
 *
 * The causes are spread over two blocks and read as one word here, and
 * the word is a history, not a cause (measured, test_rp2040_platform
 * letter i - a letter is one command key of the suite's console and the
 * group of verdicts it runs):
 *  - VREG_AND_CHIP_RESET.CHIP_RESET names the chip-level resets, the
 *    ones that took rst_n_psm low: HAD_POR (power-on or brown-out),
 *    HAD_RUN (the RUN pin), HAD_PSM_RESTART (the rescue debug port) -
 *    and they stand from that event on, for the life of the supply:
 *    HAD_POR reads set at every boot of a board that was powered on
 *    once, however many resets came after;
 *  - WATCHDOG.REASON names the last watchdog reset: TIMER (the
 *    countdown) or FORCE (CTRL.TRIGGER), each replacing the other, and
 *    it stands until a chip-level reset: a board reports TIMER at every
 *    boot after its first time-out, however many software resets came
 *    between.
 * This chip has no chip-wide SYSRESETREQ. SCB.AIRCR.SYSRESETREQ, the
 * Cortex-M's own request, "only resets the Cortex-M0+ processor core
 * (not the Debug or PMU)" - and only the core that asked (2.4.2): the
 * core restarts through the bootrom and the second stage, the other
 * core keeps running whatever it ran, the debug logic keeps whatever a
 * probe left in it. Measured: a program with a kernel on each core
 * reset through it on core 0 came back to a core 1 still in its old
 * life, deaf to a new launch (test_rp2040_multicore). It sets no flag
 * and clears none. So it is `Reset::core()` here, the verb for a
 * program that means exactly that, and `Reset::software()` - what
 * ResetReporter and fault_reset() call, what a program means by
 * "reboot" - is the watchdog's trigger with the power-on state
 * machine's WDSEL selecting everything but the two oscillators
 * (rp2040/watchdog.hpp): the whole chip through the bootrom as a
 * power-on would, both cores, the SRAM kept, REASON.FORCE at the next
 * boot as its mark - the one software reboot this chip has, which the
 * SDK's watchdog_reboot() is too.
 *
 * So "what caused this boot" is answered by comparing the word with the
 * one the previous life saw (a scratch register, as the bench suite
 * does): a reboot by software reads FORCE where the last watchdog event
 * stood, and the chip-level flags as they were. The registers are
 * read-only: reading changes nothing.
 *
 * `ResetReporter` is a panic Reporter that reboots instead of spinning,
 * so the breadcrumb is read at the next boot - on either core: a panic
 * on core 1 reboots the chip and core 0 with it, and core 0 reports
 * both records at the next boot (one per platform type,
 * `take_panic_record<P0>()` and `take_panic_record<P1>()`).
 * `fault_reset<P>()` is the HardFault body an app binds to
 * `isr_hardfault`.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "rp2040/watchdog.hpp"

namespace brio {

/// The causes, as one word.
struct ResetCause {
    static constexpr uint32_t power_on = 1u << 0;        ///< CHIP_RESET.HAD_POR
    static constexpr uint32_t run_pin = 1u << 1;         ///< CHIP_RESET.HAD_RUN
    static constexpr uint32_t rescue = 1u << 2;          ///< CHIP_RESET.HAD_PSM_RESTART
    static constexpr uint32_t watchdog_timer = 1u << 3;  ///< WATCHDOG.REASON.TIMER
    static constexpr uint32_t watchdog_force = 1u << 4;  ///< WATCHDOG.REASON.FORCE
    static constexpr uint32_t all = 0x1Fu;
};

struct Reset {
    Reset() = delete;

    /// ResetCause bits as the two registers hold them: the history the
    /// file header describes (the chip-level flags standing since their
    /// event, the last watchdog event standing until a chip-level reset),
    /// not the cause of this boot alone - a reboot by software reads
    /// watchdog_force beside whatever chip-level flag stands. Any context;
    /// reading changes nothing.
    static uint32_t causes() {
        const uint32_t chip = VREG_AND_CHIP_RESET->CHIP_RESET;
        const uint32_t wd = WATCHDOG->REASON;
        uint32_t c = 0;
        if ((chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) != 0u) { c |= ResetCause::power_on; }
        if ((chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS) != 0u) { c |= ResetCause::run_pin; }
        if ((chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) != 0u) { c |= ResetCause::rescue; }
        if ((wd & WATCHDOG_REASON_TIMER_BITS) != 0u) { c |= ResetCause::watchdog_timer; }
        if ((wd & WATCHDOG_REASON_FORCE_BITS) != 0u) { c |= ResetCause::watchdog_force; }
        return c;
    }

    /// Reboot the chip: the watchdog's trigger under the power-on state
    /// machine's selection (the file header) - both cores through the
    /// bootrom, the SRAM kept, REASON.FORCE at the next boot. Never
    /// returns.
    [[noreturn]] static void software() { Watchdog::force_reset(); }

    /// Reset this core alone through SYSRESETREQ (2.4.2): the calling
    /// core restarts through the bootrom and the second stage; the other
    /// core, the debug logic and every peripheral stay as they are, and
    /// no flag marks it. Never returns.
    [[noreturn]] static void core() {
        __DSB();
        SCB->AIRCR = (0x5FAul << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
        __DSB();
        for (;;) {
        }
    }
};

/// A panic Reporter that ends the program with a chip reboot, so the
/// breadcrumb panic() wrote is reported at the next boot - from either
/// core. panic() calls the reporter after its breakpoint
/// (kernel/panic.hpp): with no debugger attached that breakpoint raises
/// a HardFault first (Rp2040Platform::break_here()), so on a board
/// without a probe the reboot is the bound fault_reset()'s, which keeps
/// the panic's record, and this reporter's is the one a debugger reaches
/// by resuming the breakpoint.
struct ResetReporter {
    static void report(PanicCode, uint8_t) { Reset::software(); }
};

/**
 * The HardFault body: record the wreck and reset. An app binds it:
 *
 *     extern "C" void isr_hardfault() { brio::fault_reset<brio::Rp2040Platform<>>(); }
 *
 * Not through panic(): panic() ends in BKPT, which with no debugger
 * escalates to HardFault, and a BKPT taken inside HardFault is a
 * lockup. An existing record is not overwritten: a valid one standing
 * means this fault is the consequence of a panic already diagnosed.
 */
template <Platform P>
[[noreturn]] void fault_reset(uint8_t context = 0) {
    PanicRecord& r = P::panic_record();
    if (r.magic != panic_magic) {
        r = PanicRecord{panic_magic, static_cast<uint8_t>(PanicCode::kernel_fault), context};
    }
    Reset::software();
}

} // namespace brio
