/*
 * platform_sam.hpp
 *
 * SAM C21 (Cortex-M0+) implementation of the brio Platform concept - the
 * one header of this stratum the kernel templates are instantiated with.
 * Apps select it by including it and passing SamPlatform along.
 *
 * CriticalSection is samc/nvic.hpp's InterruptGuard: save PRIMASK,
 * cpsid i, restore on scope exit. Nesting-correct, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 * ARMv6-M has no BASEPRI, so this is all-or-nothing masking - there is
 * no priority-limited critical section to be had on this core.
 *
 * The other half of the panic breadcrumb - which reset actually happened
 * and how to cause one - is samc/reset.hpp (RSTC, the watchdog,
 * `ResetReporter` and the HardFault body); this header provides the
 * storage the record lives in, and the linker script the .noinit section
 * it needs.
 *
 * The STOPPING half is samc/sleep.hpp (PM) plus util/power.hpp's
 * manager. Only one thing about it lands here: `idle()` takes whatever
 * PM.SLEEPCFG holds, so a depth armed above stands - see the comment on
 * idle() for that and for the erratum it carries.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "kernel/platform.hpp"
#include "samc/nvic.hpp"
#include "samc/ticker.hpp"

namespace brio {

struct SamPlatform {
    using CriticalSection = InterruptGuard;

    /// Entered with interrupts MASKED and nothing to do: sleep until the
    /// next interrupt.
    ///
    /// WFI FIRST, UNMASK AFTER. On Cortex-M a pending interrupt wakes
    /// WFI even with PRIMASK set - the handler simply does not run until
    /// PRIMASK clears. So sleeping before unmasking closes the
    /// lost-wakeup window by construction, and it closes it more simply
    /// than AVR's sei-then-sleep pairing: an interrupt that becomes
    /// pending between the caller's queue check and the WFI does not put
    /// the core to sleep at all.
    ///
    /// THE ARMED MODE STANDS. This family selects the sleep depth in
    /// PM.SLEEPCFG, not in SCR.SLEEPDEEP, and the WFI instruction is the
    /// same one whatever depth is selected - so honoring a mode armed
    /// above (samc/sleep.hpp's `SamSleepSite`, driven by
    /// util/power.hpp's manager) costs this hook no arming logic at all.
    /// SCR.SLEEPDEEP is deliberately never written: chapter 19 does not
    /// use it, and out of reset it is 0. With SLEEPCFG at ITS reset value
    /// - IDLE0 - this is plain idle sleep: the CPU stops and every
    /// peripheral and interrupt source stays alive.
    ///
    /// THE ONE TEST A MANAGERLESS PROGRAM PAYS FOR is the SLEEPCFG read
    /// below, and it buys erratum 1.8.13's workaround: with the standby
    /// back-bias option set (STDBYCFG.BBIASHS, whose reset value is 1) a
    /// SysTick interrupt coinciding exactly with a standby entry can
    /// hard fault, so the kernel tick's interrupt is held off across the
    /// WFI whenever the armed mode is STANDBY. The register is read
    /// rather than a flag kept, because SLEEPCFG IS the state: it is what
    /// the WFI will obey. The guard costs no ticks - SysTick is clocked
    /// from the CPU clock, so it is frozen across a standby either way
    /// (samc/ticker.hpp's caveat, and samc/sleep.hpp's tick rule).
    ///
    /// The DSB is the ARM recommendation for WFI: it retires the posted
    /// writes - the SLEEPCFG store above all - before the core stops.
    static void idle() {
        if ((PM_REGS->PM_SLEEPCFG & PM_SLEEPCFG_SLEEPMODE_Msk) ==
            PM_SLEEPCFG_SLEEPMODE_STANDBY) {
            SysTickInterruptGuard guard;   // erratum 1.8.13
            __DSB();
            __WFI();
        } else {
            __DSB();
            __WFI();
        }
        __enable_irq();
    }

    /// PRIMASK readback: the one bit CriticalSection saves and restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Halt in the debugger.
    ///
    /// CAVEAT, ARMv6-M. Unlike v7-M, this core cannot ask whether a
    /// debugger is attached (DHCSR is debugger-access-only), so BKPT
    /// cannot be made conditional the way the AVR BREAK instruction is a
    /// NOP without an active OCD: with no debugger halted on it, this
    /// escalates to HardFault_Handler - which the startup file provides
    /// as a distinct spin loop precisely so the wreck is legible in a
    /// backtrace. Documented rather than guessed around; the breadcrumb
    /// -then-reset story belongs to the reset/panic pass.
    static void break_here() { __BKPT(0); }

    static uint32_t now() { return Ticker::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (samc/ticker.hpp).
    static constexpr uint32_t ticks_per_second = Ticker::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here.
    static constexpr unsigned atomic_width = 4;

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and startup neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot.
    /// SRAM survival is promised NOWHERE: table 18-1 lists what each
    /// reset cause resets and no row of it mentions SRAM, for any
    /// source including power-on - exactly the AVR situation. Hence
    /// take_panic_record()'s magic word: cold RAM is what makes that
    /// check necessary, not merely prudent.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // NOTE: gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` WITHOUT the group name, and gas
    // warns "group name for SHF_GROUP not specified". Harmless (the symbol
    // lands in .noinit, weak dedup works) and identical on the AVR side -
    // the same candidate upstream bug report, now seen on two back ends.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<SamPlatform>);

} // namespace brio
