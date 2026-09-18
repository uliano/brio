/*
 * reset.hpp
 *
 * Why the chip last started, and how to make it start again (datasheet
 * 7.1 the three tiers, 7.3 chip-level resets and their record, 7.4 the
 * power-on state machine, 12.9 the watchdog's REASON and trigger, 3.5.8
 * the rescue reset, 5.2 the bootrom) - the failing half of the platform
 * beside rp2350/platform.hpp's running half.
 *
 * THREE TIERS, AND THE WORD THAT RECORDS THEM. Chapter 7 divides every
 * reset of this chip into CHIP-level (the whole device: the power-on
 * reset, brown-out, the RUN pin, a debugger's request, the rescue port, a
 * glitch, a switched-core power-down, and a watchdog event configured to
 * reach that far), SYSTEM (the power-on state machine's stages, which is
 * what an ordinary reboot runs) and SUBSYSTEM (the peripheral blocks,
 * rp2350/resets.hpp). The source of the last chip-level reset is recorded
 * in POWMAN's CHIP_RESET - moved there from the RP2040's LDO block and
 * widened, because the power manager is in the ALWAYS-ON domain and so
 * the record outlives a power-down of the core.
 *
 * AND THE WORD IS NOT A CAUSE. WATCHDOG.REASON names the LAST watchdog
 * event, TIMER or FORCE, each replacing the other. What CHIP_RESET's
 * HAD_* half is exactly, the datasheet says two ways: 7.3.3 calls it "the
 * source of the last chip-level reset", which would make it one cause at
 * a time, while the bit names are latches and the RP2040's equivalent
 * word was measured to stand for the life of the supply. EITHER WAY a
 * program answers "what caused THIS boot" by comparing the word with the
 * one the previous life saw rather than by reading it, which is what the
 * bench suite does; the two readings differ only in what it expects to
 * find, and the suite reports which this silicon is.
 *
 * ONE MORE THING CLEARS REASON HERE, AND NOT ON THE RP2040 (12.9's own
 * note): a DEBUGGER WARM RESET of either core - the Arm SYSRESETREQ or
 * the RISC-V hartreset - clears WATCHDOG.REASON, so that code loaded
 * under a probe after a time-out does not go on reading the time-out.
 * `Reset::core()` is that same signal from software, so it wipes the
 * watchdog's half of the word and leaves the chip-level half standing.
 *
 * POWMAN IS READ AND NEVER WRITTEN HERE. Every register of that block up
 * to offset 0xac - CHIP_RESET among them - takes a write only with the
 * password 0x5AFE in its top half (6.4), and what those registers do is
 * the power chapter's business. The two flags a program might want to
 * clear, RESCUE_FLAG and DOUBLE_TAP, are the BOOTROM's: it reads the
 * rescue flag before anything else and clears it to acknowledge (5.2,
 * 3.5.8), so by the time this file runs the flag is already down and
 * `rescue_flag()` reports what is left rather than what happened. What
 * happened is `ResetCause::rescue`, the latch beside it.
 *
 * THE REBOOT. `software()` - what ResetReporter and fault_reset() call,
 * what a program means by "reboot" - is THE WATCHDOG'S TRIGGER with the
 * power-on state machine's WDSEL selecting every stage but the two
 * oscillators (rp2350/watchdog.hpp): the whole system through the
 * bootrom, both cores, the crystal kept, REASON.FORCE at the next boot as
 * its mark. `core()` is the processor's own reset request, which exists
 * on the Cortex-M33 half alone: on Hazard3 the hart reset controls
 * (dmcontrol.hartreset, dmcontrol.ndmreset) are IN THE DEBUG MODULE
 * (3.8.5.3), reachable by a debugger and by no instruction, so the verb
 * is refused at compile time there rather than spelled as something it is
 * not.
 *
 * `ResetReporter` is a panic Reporter that reboots instead of spinning,
 * so the breadcrumb is read at the next boot - on either core.
 * `fault_reset<P>()` is the body an app binds to the crt's fault entry,
 * which is `isr_hardfault` on the Arm half and `isr_riscv_exception` on
 * the RISC-V one: two names because the two crts have two tables, one
 * function because the wreck is the same.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "kernel/panic.hpp"
#include "kernel/platform.hpp"
#include "rp2350/core.hpp"
#include "rp2350/watchdog.hpp"

namespace brio {

/// The causes, as one word: POWMAN's CHIP_RESET latches and the
/// watchdog's REASON together, in 7.3.3's order of severity.
struct ResetCause {
    static constexpr uint32_t power_on = 1u << 0;        ///< CHIP_RESET.HAD_POR
    static constexpr uint32_t brown_out = 1u << 1;       ///< CHIP_RESET.HAD_BOR
    static constexpr uint32_t run_pin = 1u << 2;         ///< CHIP_RESET.HAD_RUN_LOW
    static constexpr uint32_t debug_port = 1u << 3;      ///< CHIP_RESET.HAD_DP_RESET_REQ
    static constexpr uint32_t rescue = 1u << 4;          ///< CHIP_RESET.HAD_RESCUE
    /// A watchdog event that reset the power manager, with or without
    /// clk_ref (the two POWMAN_WDSEL bits nothing in brio sets).
    static constexpr uint32_t watchdog_powman_async = 1u << 5;
    static constexpr uint32_t watchdog_powman = 1u << 6;
    /// A watchdog event that powered the switched core domain down.
    static constexpr uint32_t watchdog_swcore = 1u << 7;
    /// The switched core domain powered down for any other reason.
    static constexpr uint32_t swcore_powerdown = 1u << 8;
    static constexpr uint32_t glitch_detect = 1u << 9;   ///< CHIP_RESET.HAD_GLITCH_DETECT
    /// The RISC-V debug module's non-debug-module reset: both harts, no
    /// other hardware, and it is recorded here all the same.
    static constexpr uint32_t hazard3_sys_reset = 1u << 10;
    /// A watchdog event that ran the power-on state machine: THE ONE
    /// `software()` MAKES.
    static constexpr uint32_t watchdog_psm = 1u << 11;

    static constexpr uint32_t watchdog_timer = 1u << 12;  ///< WATCHDOG.REASON.TIMER
    static constexpr uint32_t watchdog_force = 1u << 13;  ///< WATCHDOG.REASON.FORCE

    /// Every bit above.
    static constexpr uint32_t all = 0x3FFFu;
    /// The chip-level half: CHIP_RESET's twelve HAD_* bits, in the
    /// always-on domain (the file header on what exactly they are).
    static constexpr uint32_t chip_level = 0x0FFFu;
    /// The watchdog's half: the two bits a debugger's warm reset, and
    /// `core()` with it, clears.
    static constexpr uint32_t watchdog = watchdog_timer | watchdog_force;
};

struct Reset {
    Reset() = delete;

private:
    /// One latch of CHIP_RESET beside the ResetCause bit it becomes: the
    /// register's order is the datasheet's, the word's is this file's,
    /// and the pairing is stated once here.
    struct CauseBit {
        uint32_t in_register;
        uint32_t cause;
    };
    static constexpr CauseBit chip_bits[] = {
        {POWMAN_CHIP_RESET_HAD_POR_BITS, ResetCause::power_on},
        {POWMAN_CHIP_RESET_HAD_BOR_BITS, ResetCause::brown_out},
        {POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS, ResetCause::run_pin},
        {POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS, ResetCause::debug_port},
        {POWMAN_CHIP_RESET_HAD_RESCUE_BITS, ResetCause::rescue},
        {POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_ASYNC_BITS,
         ResetCause::watchdog_powman_async},
        {POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_BITS, ResetCause::watchdog_powman},
        {POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS, ResetCause::watchdog_swcore},
        {POWMAN_CHIP_RESET_HAD_SWCORE_PD_BITS, ResetCause::swcore_powerdown},
        {POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS, ResetCause::glitch_detect},
        {POWMAN_CHIP_RESET_HAD_HZD_SYS_RESET_REQ_BITS, ResetCause::hazard3_sys_reset},
        {POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_PSM_BITS, ResetCause::watchdog_psm},
    };

public:
    /// ResetCause bits for this boot, read-only and unchanged by reading.
    static uint32_t causes() {
        const uint32_t chip = POWMAN->CHIP_RESET;
        uint32_t c = 0;
        for (const CauseBit& b : chip_bits) {
            if ((chip & b.in_register) != 0u) {
                c |= b.cause;
            }
        }
        const uint32_t wd = WATCHDOG->REASON;
        if ((wd & WATCHDOG_REASON_TIMER_BITS) != 0u) { c |= ResetCause::watchdog_timer; }
        if ((wd & WATCHDOG_REASON_FORCE_BITS) != 0u) { c |= ResetCause::watchdog_force; }
        return c;
    }

    /// CHIP_RESET.RESCUE_FLAG, the bit a rescue reset sets to tell the
    /// bootrom to halt before user code. The bootrom clears it to
    /// acknowledge, so a program that reads this ordinarily reads false
    /// even after a rescue - `causes() & ResetCause::rescue` is what
    /// answers "was this boot a rescue".
    static bool rescue_flag() {
        return (POWMAN->CHIP_RESET & POWMAN_CHIP_RESET_RESCUE_FLAG_BITS) != 0u;
    }

    /// CHIP_RESET.DOUBLE_TAP, set by tapping the RUN pin low twice: the
    /// bootrom's alternate-boot request. Read here for the record; the
    /// flag is POWMAN's and no verb of this stratum writes it.
    static bool double_tap() {
        return (POWMAN->CHIP_RESET & POWMAN_CHIP_RESET_DOUBLE_TAP_BITS) != 0u;
    }

    /// Reboot the SYSTEM: the watchdog's trigger under the power-on state
    /// machine's selection (the file header) - both cores through the
    /// bootrom, the oscillators kept, the SRAM and the scratch registers
    /// untouched by the tier this takes, REASON.FORCE at the next boot.
    /// Never returns.
    [[noreturn]] static void software() { Watchdog::force_reset(); }

    /// Reset THIS PROCESSOR through its own request. On the Cortex-M33
    /// half that is SCB.AIRCR.SYSRESETREQ, a warm reset that restarts the
    /// core through the bootrom and leaves the clock tree, the pads and
    /// every peripheral exactly as they were - and that CLEARS
    /// WATCHDOG.REASON on the way (12.9). Never returns.
    ///
    /// REFUSED AT COMPILE TIME ON THE HAZARD3 HALF, where the hart's
    /// reset controls live in the Debug Module and no instruction reaches
    /// them (3.8.5.3). The guard is on the ARGUMENT, so
    /// `Reset::core<CoreKind::hazard3>()` is refused by both compilers
    /// and a plain `Reset::core()` is refused by exactly the build that
    /// has not got it.
    template <CoreKind k = core_kind>
    [[noreturn]] static void core() {
        static_assert(k == CoreKind::cortex_m33,
                      "brio Reset::core(): a Hazard3 hart cannot reset itself - hartreset "
                      "and ndmreset are the Debug Module's (datasheet 3.8.5.3), a "
                      "debugger's and not a program's. Reset::software() is the reboot "
                      "both halves of this chip have");
#if defined(BRIO_RP2350_CORE_M33)
        __DSB();
        SCB->AIRCR = (0x5FAul << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
        __DSB();
#endif
        for (;;) {
        }
    }
};

/// A panic Reporter that ends the program with a reboot, so the
/// breadcrumb panic() wrote is reported at the next boot - from either
/// core and on either architecture.
struct ResetReporter {
    static void report(PanicCode, uint8_t) { Reset::software(); }
};

/**
 * The fault body: record the wreck and reset. An app binds it to the
 * crt's fault entry, which is one name per architecture:
 *
 *     extern "C" void isr_hardfault()       { brio::fault_reset<P>(); }   // Arm
 *     extern "C" void isr_riscv_exception() { brio::fault_reset<P>(); }   // RISC-V
 *
 * Not through panic(): panic() ends in a breakpoint instruction, which
 * with no debugger attached escalates to the very fault this body serves
 * - a lockup on the M33 and an endless re-entry on Hazard3, where an
 * exception handler that returns re-executes the instruction that
 * faulted. An existing record is not overwritten: a valid one standing
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
