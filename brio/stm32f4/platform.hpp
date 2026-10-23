/*
 * platform.hpp
 *
 * STM32F4 (Cortex-M4F) implementation of the brio Platform concept - the
 * one header of this stratum the kernel templates are instantiated with.
 * Apps select it by including it and passing Stm32f4Platform<> along.
 *
 * ONE CLASS, ITS TIMEBASE A PARAMETER. The template parameter is the
 * kernel timebase, and the default is the SysTick BasicTicker of
 * stm32f4/ticker.hpp (`Ticker`, 1000 Hz). SysTick rides HCLK, and HCLK
 * stops in Stop mode - so on that timebase KERNEL TIME STANDS STILL
 * across a Stop; the sleep sites that repair it from an RTC witness are
 * the power chapter's. The parameter is kept for the day a timebase
 * that counts through Stop exists on this family (the STM32G0's LPTIM
 * shape); today there is no Tickless timebase here and the kernel's
 * optional idle_until() hook is not offered.
 *
 * CriticalSection is stm32f4/nvic.hpp's InterruptGuard: save PRIMASK,
 * cpsid i, restore on scope exit. Nesting-correct, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 * This core HAS BASEPRI, and this platform does not use it: the kernel
 * promise that no interrupt nests over another is kept by every line
 * sitting at one priority and PRIMASK being the one mask
 * (stm32f4/nvic.hpp).
 *
 * The other halves live next door: which reset happened and how to
 * cause one is the reset chapter's, the STOPPING half - PWR's
 * Sleep/Stop/Standby ladder and the util/power.hpp sites over it - is
 * stm32f4/pwr.hpp's to grow. This silicon selects the sleep depth in
 * SCB->SCR.SLEEPDEEP and PWR_CR's PDDS/LPDS, and the idle hook here
 * writes none of them, so a WFI here takes whatever somebody else
 * armed. This header provides the storage the panic record lives in,
 * and the linker script the .noinit section it needs.
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "kernel/platform.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/ticker.hpp"
#include "cortexm/schedule.hpp"

namespace brio {

template <class TB = Ticker>
struct Stm32f4Platform {
    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on - what now() reads and
    /// where the tick rate comes from.
    using Timebase = TB;

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
    /// WHATEVER IS ARMED. This family selects the deeper modes with
    /// SCR.SLEEPDEEP plus PWR_CR (RM0090 5.3), and this hook never
    /// writes either: out of reset SLEEPDEEP is 0, so a WFI here is
    /// Sleep - the CPU clock stops, HCLK, SysTick and every peripheral
    /// keep running (5.3.4) - and with a sleep site having armed a Stop,
    /// the same WFI is that Stop. The site arms above this hook and the
    /// hook takes what it finds.
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
    /// Unconditional, as on the other Cortex-M families: with no debugger
    /// halted on it a BKPT escalates to HardFault_Handler, which the crt
    /// provides as a distinct spin loop so the wreck is legible in a
    /// backtrace; with C_DEBUGEN left set by a flashing tool the core
    /// HALTS here in silence, which is why bin/brio clears DHCSR after
    /// every flash. (ARMv7-M lets software READ DHCSR's C_DEBUGEN, unlike
    /// ARMv6-M, so a conditional break is possible on this family; it is
    /// not taken, so that the three families' panics end the same way.)
    static void break_here() { __BKPT(0); }

    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (stm32f4/ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here.
    static constexpr unsigned atomic_width = 4;

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and the crt neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot. SRAM
    /// content is promised nowhere across a reset (RM0090 2.3.1 says
    /// only what Standby loses); the magic word take_panic_record()
    /// checks is what makes a cold word harmless.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name and gas
    // warns; harmless.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<Stm32f4Platform<>>);

/**
 * Stm32f4RubatoPlatform<TB>: the platform a Rubato kernel runs on
 * (kernel/rubato.hpp), the preemptive one. It is Stm32f4Platform<TB>
 * whole - the same critical section, idle, timebase, breakpoint and panic
 * storage - plus the three members kernel/platform.hpp's Preemptive
 * concept asks for:
 *
 *     using P = brio::Stm32f4RubatoPlatform<>;
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
 *     extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }
 *     int main() { ...; brio::scheduler_init(); brio::enable_interrupts(); K::run(); }
 *
 *  - `preemptive`, true;
 *  - `ceiling`, the byte the kernel keeps: the pack index of the object
 *    being dispatched, 255 at the loop's base. A static of the platform,
 *    so of the program: one kernel per platform type, which on this
 *    single-core family is one kernel;
 *  - `pend_schedule()`, the request for the scheduler, which is the core's
 *    and not ST's: cortexm/schedule.hpp's PendSV, at the lowest priority
 *    this core implements (the sixteenth level, below every NVIC line at
 *    its reset priority 0). PendSV returns into the scheduler in thread
 *    mode, and the scheduler leaves through SVC; that file says how, with
 *    the frames it builds, the FPU's part and the stack each level of
 *    preemption costs.
 *
 * The CriticalSection is still PRIMASK alone, and the promise that an
 * interrupt handler is not interrupted by another keeps its meaning for
 * the peripheral handlers, which still share one priority and never nest
 * among themselves; the scheduler's exception sits below them all, and
 * they do interrupt it. A program on this platform binds two vectors
 * more than a Tenuto program, PendSV_Handler and SVC_Handler, both naked,
 * executes no `svc` of its own, and calls scheduler_init() before it
 * enables interrupts.
 */
template <class TB = Ticker>
struct Stm32f4RubatoPlatform : Stm32f4Platform<TB> {
    static constexpr bool preemptive = true;

    /// The index of the object being dispatched; 255 at the loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler (cortexm/schedule.hpp).
    [[gnu::always_inline]] static void pend_schedule() { request_scheduler(); }
};

static_assert(Platform<Stm32f4RubatoPlatform<>>);
static_assert(Preemptive<Stm32f4RubatoPlatform<>>);
static_assert(!Preemptive<Stm32f4Platform<>>);

} // namespace brio
