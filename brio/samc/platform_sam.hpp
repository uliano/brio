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
 * and how to cause one (RSTC/WDT, the avrdx/reset.hpp analog) - is not
 * built for this target yet; this header only provides the storage the
 * record lives in, and the linker script the .noinit section it needs.
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
    /// SCR.SLEEPDEEP IS NOT TOUCHED. Out of reset it is 0, so this is
    /// plain sleep: the CPU stops, every peripheral and every interrupt
    /// source stays alive. Whatever depth is armed above stands - the
    /// exact analog of the AVR idle() honoring a standing SEN, and the
    /// reason the future power manager will need no new kernel hook.
    /// Arming (SCR.SLEEPDEEP + PM.SLEEPCFG) is that pass's job, not the
    /// kernel's "nothing to do" hook's.
    static void idle() {
        __WFI();
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
