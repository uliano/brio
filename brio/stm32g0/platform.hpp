/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on an STM32G0: the
 * STM32G0 (Cortex-M0+) realization of the kernel's Platform concept,
 * the compiler-checked set of requirements kernel/platform.hpp states.
 * A program names it once, as the first argument of its Tenuto, and
 * every kernel template is instantiated with it:
 *
 *     using P = brio::Stm32g0Platform<>;
 *     brio::Tenuto<P, Blinker, Supervisor>::run();
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time, a
 * breakpoint for a panic and the storage a panic record survives a reset
 * in. Who calls what: the kernel loop calls idle() and idle_until() in
 * main-loop context, with interrupts already masked, and nothing else
 * should; now() and ticks_per_second may be read from any context;
 * CriticalSection may be opened from any context and nests;
 * break_here() is the panic path's and panic_record() the panic path's
 * and the next boot's. A program binds the timebase's interrupt vector
 * itself (SysTick_Handler to Ticker::tick(), or the LPTIM's vector to
 * LptimTicker::isr()); the kernel binds no vector, and a program that
 * forgets it never advances time.
 *
 * One class, two timebases. The template parameter is the kernel
 * timebase, and the default is the SysTick BasicTicker of
 * stm32g0/ticker.hpp (`Ticker`, 1000 Hz). SysTick rides HCLK, and HCLK
 * stops in the Stop modes, so on that timebase kernel time stands still
 * across a Stop, and stm32g0/sleep.hpp keeps two timed sites that repair
 * it afterwards from an RTC or LPTIM witness. The other argument is
 * stm32g0/lptim_ticker.hpp's `LptimTicker`: kernel time counted on a
 * low-power timer clocked from the LSE crystal, which keeps counting
 * through Stop 0 and Stop 1. That timebase is tickless: there is no
 * periodic interrupt at all, and this platform then provides the
 * kernel's optional idle_until() hook (kernel/platform.hpp): the loop
 * hands it the nearest armed deadline, the timebase places its wake
 * there, and one WFI covers the whole wait whatever depth a sleep site
 * armed. The plain sleep site is the only site such a program takes (the
 * timed ones refuse it at compile time: nothing stands still, nothing
 * needs repairing), and SysTick keeps running interrupt-less as
 * cortexm/delay.hpp's cycle counter. The price: one LPTIM, its vector
 * bound by the app to the ticker's isr().
 *
 * Why the idle sleeps first and unmasks after: on Cortex-M a pending
 * interrupt wakes a WFI even under PRIMASK, so an interrupt that lands
 * between the kernel's empty check and the WFI does not put the core to
 * sleep, and no wake is lost; idle() below says it in full. Why the
 * masking is all-or-nothing: ARMv6-M has no BASEPRI, so CriticalSection
 * is stm32g0/nvic.hpp's InterruptGuard, which saves PRIMASK, executes
 * cpsid i and restores on scope exit; it nests correctly, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 *
 * The other halves live next door: which reset happened and how to cause
 * one is stm32g0/reset.hpp, and the stopping half (PWR's
 * Sleep/Stop/Standby/Shutdown ladder and the util/power.hpp sites over
 * it) is stm32g0/pwr.hpp plus stm32g0/sleep.hpp. This silicon selects
 * the sleep depth in SCB->SCR.SLEEPDEEP and PWR_CR1.LPMS, and neither
 * idle hook here writes either, so a WFI here takes whatever somebody
 * else armed. This header provides the storage the panic record lives
 * in, and the linker script the .noinit section it needs.
 */

#pragma once

#include <stdint.h>

#include <concepts>
#include <optional>

#include "stm32g0xx.h"

#include "kernel/platform.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/ticker.hpp"
#include "cortexm/schedule.hpp"

namespace brio {

/// A timebase that keeps counting while the core sleeps AND can place a
/// wake at an absolute tick: what Stm32g0Platform needs to offer the
/// kernel's idle_until(). `arm_wake(now, deadline)` is called with
/// interrupts masked, `now` being the timebase's own reading a moment
/// earlier and `deadline` strictly ahead of it; true means "the wake is
/// in place, sleep", false means "do not sleep this turn" (the timebase
/// could not place it yet - the loop turns and asks again). `park()` is
/// the same call with NO deadline: put the wake where it costs nothing
/// (for the LPTIM, on the counter's own lap edge), same answer. The
/// SysTick BasicTicker is deliberately NOT one of these: it has no wake
/// to place and its counting stops with the core.
template <class TB>
concept Tickless = requires(uint32_t now, uint32_t deadline) {
    { TB::ticks() } -> std::same_as<uint32_t>;
    requires TB::ticks_per_second > 0u;
    { TB::arm_wake(now, deadline) } -> std::same_as<bool>;
    { TB::park() } -> std::same_as<bool>;
};

template <class TB = Ticker>
struct Stm32g0Platform {
    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on - what now() reads and
    /// where the tick rate comes from. The sleep sites ask it whether it
    /// has a periodic interrupt to pause (stm32g0/sleep.hpp).
    using Timebase = TB;

    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt and returns with interrupts enabled. Not for a program
    /// to call.
    ///
    /// WFI first, unmask after. On Cortex-M a pending interrupt wakes
    /// WFI even with PRIMASK set - the handler simply does not run until
    /// PRIMASK clears. So sleeping before unmasking closes the
    /// lost-wakeup window by construction: an interrupt that becomes
    /// pending between the caller's queue check and the WFI does not put
    /// the core to sleep at all.
    ///
    /// Whatever is armed. This family selects the deeper modes with
    /// SCR.SLEEPDEEP plus PWR_CR1.LPMS (RM0444 4.3), and this hook never
    /// writes either: out of reset SLEEPDEEP is 0, so a WFI here is
    /// Sleep - the CPU clock stops, HCLK, SysTick and every peripheral
    /// keep running (5.3) - and with stm32g0/sleep.hpp's site having
    /// armed a Stop, the same WFI is that Stop. The site arms above this
    /// hook and the hook takes what it finds. On the SysTick timebase
    /// kernel time stops in a Stop (SysTick rides HCLK), which is what
    /// the timed sites exist to repair; on a Tickless timebase it does
    /// not, and the loop calls idle_until() below instead of this.
    ///
    /// The DSB is the ARM recommendation for WFI: it retires the posted
    /// writes before the core stops.
    static void idle() {
        __DSB();
        __WFI();
        __enable_irq();
    }

    /// The kernel's optional hook (kernel/platform.hpp), present only on
    /// a Tickless timebase: sleep until `deadline` - the absolute tick of
    /// the nearest armed time event - or until any other interrupt,
    /// whichever comes first; with no deadline, until any interrupt.
    ///
    /// Entered masked like idle(). A deadline already due, or one the
    /// timebase could not place this turn, means NO SLEEP: interrupts
    /// back on and return, and the loop's next process() fires what is
    /// due (or the next idle_until() places what was deferred). Never
    /// past the deadline: the timebase's wake is the same counter the
    /// deadline was written in, and its own contract is "at or after".
    static void idle_until(std::optional<uint32_t> deadline)
        requires Tickless<TB>
    {
        if (deadline.has_value()) {
            const uint32_t now = TB::ticks();
            if (static_cast<int32_t>(*deadline - now) <= 0 || !TB::arm_wake(now, *deadline)) {
                __enable_irq();
                return;
            }
        } else if (!TB::park()) {
            __enable_irq();
            return;
        }
        idle();
    }

    /// PRIMASK readback: the one bit CriticalSection saves and restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Whether the caller runs inside an exception handler: IPSR's
    /// exception number, zero in thread mode (cortexm/nvic.hpp's
    /// in_handler(); the optional member of kernel/platform.hpp).
    static bool in_handler() { return brio::in_handler(); }

    /// Halt in the debugger: the panic path's breakpoint
    /// (kernel/panic.hpp), callable from any context.
    ///
    /// Caveat, ARMv6-M. The core cannot ask whether a debugger is
    /// attached (DHCSR is debugger-access-only), so BKPT cannot be made
    /// conditional: with no debugger halted on it, this escalates to
    /// HardFault_Handler, which the crt provides as a distinct spin loop
    /// so the wreck is legible in a backtrace. And with C_DEBUGEN left
    /// set by a flashing tool the core HALTS here in silence, which
    /// is why bin/brio clears DHCSR after every flash.
    static void break_here() { __BKPT(0); }

    /// The kernel's clock: the timebase's tick count, readable from any
    /// context (what kernel/time_event.hpp arms deadlines against).
    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (stm32g0/ticker.hpp),
    /// 1024 Hz on the default LptimTicker (stm32g0/lptim_ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here.
    static constexpr unsigned atomic_width = 4;

    /// This family places `.fast_text` in SRAM: the three linker scripts
    /// (stm32g0/ld/) collect it first in .data, which the crt copies
    /// before main(), and the vector table holds absolute addresses, so a
    /// vector placed there is entered directly; a CALL between the flash
    /// and SRAM is out of a BL's 16 MB and goes through the linker's
    /// long-branch veneer (docs/stm32g0/platform.md, "Code in SRAM"). A
    /// driver of this stratum may therefore be given util/placement.hpp's
    /// InRam - on every part, the STM32G031's 8 KB included: the placement
    /// is the program's opt-in and its cost the program's to weigh, and a
    /// link that does not fit is refused by the script's ram region.
    static constexpr bool places_fast_text = true;

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and the crt neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot
    /// (written by the panic path, read by take_panic_record() at boot).
    /// SRAM content is promised nowhere across a reset (RM0444 2.3 says
    /// only that the SRAM is not retained through Shutdown); the magic
    /// word take_panic_record() checks is what makes a cold word
    /// harmless. One more thing this family can do to a never-written
    /// word: with the SRAM parity check enabled by option byte
    /// (FLASH_OPTR.RAM_PARITY_CHECK = 0; the factory default is 1,
    /// disabled) a read of uninitialized SRAM raises an NMI - the
    /// breadcrumb's first read after a power-on would. Nothing here
    /// enables the check, and the bench boards ship with it off.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name and gas
    // warns; harmless.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<Stm32g0Platform<>>);
static_assert(!Tickless<Ticker>);

/**
 * Stm32g0RubatoPlatform<TB>: the platform a Rubato kernel runs on
 * (kernel/rubato.hpp), the preemptive one. It is Stm32g0Platform<TB>
 * whole - the same critical section, idle, idle_until() where the
 * timebase is Tickless, timebase, breakpoint and panic storage - plus the
 * three members kernel/platform.hpp's Preemptive concept asks for:
 *
 *     using P = brio::Stm32g0RubatoPlatform<>;
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
 *    this core implements (the fourth of the Cortex-M0+'s four levels,
 *    the byte 0xC0, below every NVIC line at its reset priority 0).
 *    PendSV returns into the scheduler in thread mode, and the scheduler
 *    leaves through SVC; that file says how, with the frames it builds
 *    and the stack each level of preemption costs (a basic frame always:
 *    this core has no FPU).
 *
 * The CriticalSection is still PRIMASK alone - ARMv6-M has nothing finer
 * - and the promise that an interrupt handler is not interrupted by
 * another keeps its meaning for the peripheral handlers, which still
 * share one priority and never nest among themselves; the scheduler's
 * exception sits below them all, and they do interrupt it. A request
 * made with PRIMASK set is taken when the mask lifts, so a post inside a
 * critical section serves its receiver when the section ends. A program
 * on this platform binds two vectors more than a Tenuto program,
 * PendSV_Handler and SVC_Handler (the crt's weak aliases), both naked,
 * executes no `svc` of its own, and calls scheduler_init() before it
 * enables interrupts.
 */
template <class TB = Ticker>
struct Stm32g0RubatoPlatform : Stm32g0Platform<TB> {
    static constexpr bool preemptive = true;

    /// The index of the object being dispatched; 255 at the loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler (cortexm/schedule.hpp).
    [[gnu::always_inline]] static void pend_schedule() { request_scheduler(); }
};

static_assert(Platform<Stm32g0RubatoPlatform<>>);
static_assert(Preemptive<Stm32g0RubatoPlatform<>>);
static_assert(!Preemptive<Stm32g0Platform<>>);

} // namespace brio
