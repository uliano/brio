/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on a SAM C21: the SAM C21
 * (Cortex-M0+) realization of the kernel's Platform concept, the
 * compiler-checked set of requirements kernel/platform.hpp states, and
 * the one header of this stratum the kernel templates are instantiated
 * with. A program includes it and names it once, as the first argument
 * of its Tenuto:
 *
 *     using P = brio::SamPlatform;
 *     brio::Tenuto<P, Blinker, Supervisor>::run();
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time
 * (samc21/ticker.hpp's 1000 Hz SysTick `Ticker`), a breakpoint for a
 * panic and the storage a panic record survives a reset in. Who calls
 * what: the kernel loop calls idle() in main-loop context, with
 * interrupts already masked, and nothing else should; now() and
 * ticks_per_second may be read from any context; CriticalSection may be
 * opened from any context and nests; break_here() is the panic path's
 * and panic_record() the panic path's and the next boot's. A program
 * binds the timebase's vector itself (SysTick_Handler calling
 * Ticker::tick(), after SysClock::init() and Ticker::init(clock) in
 * main()); the kernel binds no vector, and a SysTick_Handler left
 * undefined is the crt's weak spin, where the program stops at its
 * first tick.
 *
 * CriticalSection is samc21/nvic.hpp's InterruptGuard: save PRIMASK,
 * cpsid i, restore on scope exit. Nesting-correct, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 * ARMv6-M has no BASEPRI, so this is all-or-nothing masking - there is
 * no priority-limited critical section to be had on this core.
 *
 * The other half of the panic breadcrumb - which reset actually happened
 * and how to cause one - is samc21/reset.hpp (RSTC, the watchdog,
 * `ResetReporter` and the HardFault body); this header provides the
 * storage the record lives in, and the linker script the .noinit section
 * it needs.
 *
 * The stopping half is samc21/sleep.hpp (PM) plus util/power.hpp's
 * manager. Only one thing about it lands here: `idle()` takes whatever
 * PM.SLEEPCFG holds, so a depth armed above stands - see the comment on
 * idle() for that and for the erratum it carries.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "kernel/platform.hpp"
#include "samc21/nvic.hpp"
#include "samc21/ticker.hpp"

namespace brio {

struct SamPlatform {
    using CriticalSection = InterruptGuard;

    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt and returns with interrupts enabled. Not for a program
    /// to call.
    ///
    /// WFI first, unmask after. On Cortex-M a pending interrupt wakes WFI
    /// even with PRIMASK set - the handler simply does not run until
    /// PRIMASK clears. So sleeping before unmasking closes the
    /// lost-wakeup window by construction: an interrupt that becomes
    /// pending between the caller's queue check and the WFI does not put
    /// the core to sleep at all. Measured with an edge placed at every
    /// cycle of this path, in IDLE0, IDLE2 and STANDBY: no wake lost or
    /// late, one return per interrupt taken (docs/samc21/platform.md).
    ///
    /// The armed mode stands. This family selects the sleep depth in
    /// PM.SLEEPCFG, not in SCR.SLEEPDEEP, and the WFI instruction is the
    /// same one whatever depth is selected - so honoring a mode armed
    /// above (samc21/sleep.hpp's `SamSleepSite`, driven by
    /// util/power.hpp's manager) costs this hook no arming logic at all.
    /// SCR.SLEEPDEEP is deliberately never written: chapter 19 does not
    /// use it, and out of reset it is 0. With SLEEPCFG at its reset value
    /// - IDLE0 - this is plain idle sleep: the CPU stops and every
    /// peripheral and interrupt source stays alive.
    ///
    /// The one test a program with no power manager pays for is the
    /// SLEEPCFG read below, and it buys erratum 1.8.13's workaround: with the standby
    /// back-bias option set (STDBYCFG.BBIASHS, whose reset value is 1) a
    /// SysTick interrupt coinciding exactly with a standby entry can
    /// hard fault, so the kernel tick's interrupt is held off across the
    /// WFI whenever the armed mode is STANDBY. The register is read
    /// rather than a flag kept, because SLEEPCFG is the state: it is what
    /// the WFI will obey. Kernel time stands still across a standby with
    /// the guard, whatever SysTick's counter does in there - and on the
    /// bench it runs (docs/samc21/platform.md), so the guard is also what
    /// keeps the tick from ending the standby every period (samc21/
    /// sleep.hpp's tick rule).
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

    /// Halt in the debugger: the panic path's breakpoint
    /// (kernel/panic.hpp), callable from any context.
    ///
    /// Caveat, ARMv6-M. Unlike v7-M, this core cannot ask whether a
    /// debugger is attached (DHCSR is debugger-access-only), so BKPT
    /// cannot be made conditional on one being present: with no debugger
    /// halted on it, this escalates to HardFault_Handler - which the
    /// startup file provides as a distinct spin loop precisely so the
    /// wreck is legible in a backtrace. Documented rather than guessed
    /// around; the breadcrumb-then-reset story is samc21/reset.hpp's.
    static void break_here() { __BKPT(0); }

    /// The kernel's clock: the ticker's tick count, readable from any
    /// context (what kernel/time_event.hpp arms deadlines against).
    static uint32_t now() { return Ticker::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (samc21/ticker.hpp).
    static constexpr uint32_t ticks_per_second = Ticker::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here.
    static constexpr unsigned atomic_width = 4;

    /// This family places `.fast_text` in SRAM: samc21/ld/samc21j18a.ld
    /// collects it first in .data, which the crt copies before main(), and
    /// the vector table holds absolute addresses, so a vector placed there
    /// is entered directly; a CALL between the flash and SRAM is out of a
    /// BL's 16 MB and goes through the linker's long-branch veneer
    /// (docs/samc21/platform.md, "A handler in SRAM"). A driver of this
    /// stratum may therefore be given util/placement.hpp's InRam.
    static constexpr bool places_fast_text = true;

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and startup neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot. SRAM
    /// survival is promised nowhere: table 18-1 lists what each reset
    /// cause resets and no row of it mentions SRAM, for any source
    /// including power-on. Hence take_panic_record()'s magic word: cold
    /// RAM is what makes that check necessary, not merely prudent.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name, and gas
    // warns "group name for SHF_GROUP not specified". Harmless (the symbol
    // lands in .noinit, weak dedup works) and not specific to this back
    // end - it appears on others too, so it is a candidate gcc bug.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<SamPlatform>);

} // namespace brio
