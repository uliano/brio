/*
 * platform_stm32.hpp
 *
 * STM32G0 (Cortex-M0+) implementation of the brio Platform concept - the
 * one header of this stratum the kernel templates are instantiated with.
 * Apps select it by including it and passing Stm32Platform along.
 *
 * CriticalSection is stm32g0/nvic.hpp's InterruptGuard: save PRIMASK,
 * cpsid i, restore on scope exit. Nesting-correct, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 * ARMv6-M has no BASEPRI, so this is all-or-nothing masking.
 *
 * The other halves are not built yet and are named so nobody looks for
 * them here: which reset happened and how to cause one (RCC_CSR's reset
 * flags, SYSRESETREQ, the IWDG/WWDG) is a future stm32g0/reset.hpp; the
 * STOPPING half (PWR's Sleep/Stop/Standby/Shutdown ladder and the
 * util/power.hpp site over it) is a future stm32g0/sleep.hpp. This
 * header provides the storage the panic record lives in, and the linker
 * script the .noinit section it needs.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "kernel/platform.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/ticker.hpp"

namespace brio {

struct Stm32Platform {
    using CriticalSection = InterruptGuard;

    /// Entered with interrupts MASKED and nothing to do: sleep until the
    /// next interrupt.
    ///
    /// WFI FIRST, UNMASK AFTER. On Cortex-M a pending interrupt wakes
    /// WFI even with PRIMASK set - the handler simply does not run until
    /// PRIMASK clears. So sleeping before unmasking closes the
    /// lost-wakeup window by construction: an interrupt that becomes
    /// pending between the caller's queue check and the WFI does not put
    /// the core to sleep at all.
    ///
    /// SLEEP MODE, AND ONLY SLEEP MODE. This family selects the deeper
    /// modes with SCR.SLEEPDEEP plus PWR_CR1.LPMS (RM0444 4.3), and this
    /// hook never writes either: out of reset SLEEPDEEP is 0, so a WFI
    /// here is Sleep - the CPU clock stops, HCLK, SysTick and every
    /// peripheral keep running (5.3). The future sleep site arms the
    /// deeper modes above this hook, which then takes whatever is armed,
    /// exactly as the samc hook does with PM.SLEEPCFG.
    ///
    /// The DSB is the ARM recommendation for WFI: it retires the posted
    /// writes before the core stops.
    static void idle() {
        __DSB();
        __WFI();
        __enable_irq();
    }

    /// PRIMASK readback: the one bit CriticalSection saves and restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Halt in the debugger.
    ///
    /// CAVEAT, ARMv6-M. The core cannot ask whether a debugger is
    /// attached (DHCSR is debugger-access-only), so BKPT cannot be made
    /// conditional: with no debugger halted on it, this escalates to
    /// HardFault_Handler, which the crt provides as a distinct spin loop
    /// so the wreck is legible in a backtrace. And with C_DEBUGEN left
    /// set by a flashing session the core HALTS here in silence - the
    /// samc bench lesson, answered the same way: tools/bench.py clears
    /// DHCSR after every flash.
    static void break_here() { __BKPT(0); }

    static uint32_t now() { return Ticker::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (stm32g0/ticker.hpp).
    static constexpr uint32_t ticks_per_second = Ticker::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here.
    static constexpr unsigned atomic_width = 4;

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and the crt neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot.
    /// SRAM content is promised NOWHERE across a reset (RM0444 2.3 says
    /// only that the SRAM is not retained through Shutdown); the magic
    /// word take_panic_record() checks is what makes a cold word
    /// harmless. One more thing this family can do to a never-written
    /// word: with the SRAM PARITY CHECK enabled by option byte
    /// (FLASH_OPTR.RAM_PARITY_CHECK = 0; the factory default is 1,
    /// disabled) a read of uninitialized SRAM raises an NMI - the
    /// breadcrumb's first read after a power-on would. Kept in mind for
    /// the option-byte pass; today's boards ship with the check off.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name and gas
    // warns; harmless, identical on the AVR and SAM sides.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<Stm32Platform>);

} // namespace brio
