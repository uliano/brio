/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on an RP2040: the RP2040
 * (Cortex-M0+) realization of the kernel's Platform concept, the
 * compiler-checked set of requirements kernel/platform.hpp states. A
 * program names it once, as the first argument of its Tenuto, and every
 * kernel template is instantiated with it. In outline (Console,
 * SerialLines and Lamp stand for the program's active objects):
 *
 *     using P = brio::Rp2040Platform<>;
 *     extern "C" void isr_systick() { brio::Ticker::tick(); }
 *     ...
 *     SysClock::init();             // rp2040/clock.hpp, first
 *     brio::Ticker::init(clock);    // the SysTick timebase, rp2040/ticker.hpp
 *     brio::enable_interrupts();
 *     brio::Tenuto<P, Console, SerialLines, Lamp>::run();
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time, a
 * breakpoint for a panic and the storage a panic record survives a reset
 * in. Who calls what: the kernel loop calls idle() in main-loop context
 * with interrupts already masked, and nothing else should; now() and
 * ticks_per_second may be read from any context; CriticalSection may be
 * opened from any context and nests; break_here() is the panic path's
 * and panic_record() the panic path's and the next boot's; sleep_hook is
 * written by a dormant sleep site (rp2040/sleep.hpp) and by nothing
 * else. A program binds the timebase's vector itself (isr_systick to
 * the ticker's tick()); the kernel binds no vector, and the crt aliases
 * isr_systick, as it does every handler name but isr_hardfault (a weak
 * spin of its own), weakly to Default_Handler, a spin
 * (rp2040/src/glue/startup_rp2040.cpp), so a program that forgets it
 * stops in that spin at its first tick.
 *
 * One platform type per core. The first template parameter is the core,
 * 0 or 1: the kernel's statics are keyed by the platform type, so
 * `Tenuto<Rp2040Platform<0>, ...>` and `Tenuto<Rp2040Platform<1>, ...>`
 * are two kernels with two lists of active objects (an active object is
 * a component with its own event queue, served one event at a time by
 * its core's kernel), two timebases (the second parameter, the core's
 * own SysTick ticker by default) and two breadcrumbs, and the type is
 * the core identity everywhere the kernel asks P (the model:
 * util/inbox.hpp, design/kernel.md section 12, "Two cores: two kernels
 * and a bridge"). `Rp2040Platform<>` is core 0's, what a single-core
 * program names. The two optional members of a multi-core platform
 * (kernel/platform.hpp) are here: `on_own_core()` reads SIO's CPUID, so
 * a post to the other core's queue is refused and counted; `Doorbell`
 * is the SIO FIFO towards this core (rp2040/multicore.hpp), what a send
 * to an active object of this core rings.
 *
 * The platform offers no idle_until(): SysTick rides clk_sys, and
 * nothing of this chip's sleeping short of DORMANT stops clk_sys, so
 * the plain idle() path is the whole story here and the loop compiles
 * nothing else. What the idle path does offer is `sleep_hook`, the
 * one entry a DORMANT needs (below).
 *
 * CriticalSection is rp2040/nvic.hpp's InterruptGuard: save PRIMASK,
 * cpsid i, restore on scope exit. Nesting-correct, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 * ARMv6-M has no BASEPRI, so this is all-or-nothing masking - and it is
 * per core: PRIMASK is a register of the core that sets it, so a
 * critical section here excludes this core's handlers and nothing that
 * runs on the other core. Everything the kernel guards with it is
 * therefore correct within one core, which is what the kernel promises;
 * anything shared between two kernels is a bridge with its own
 * discipline, not a CriticalSection.
 *
 * core_id() reads SIO's CPUID: the core that is running, whatever the
 * platform type says it should be.
 *
 * Rp2040RubatoPlatform<core_index, TB>, at the end of this file, is the
 * same platform for the preemptive kernel, Rubato: one per core, as here.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "kernel/platform.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/ticker.hpp"
#include "cortexm/schedule.hpp"

namespace brio {

template <uint8_t core_index = 0, class TB = CoreTicker<core_index>>
struct Rp2040Platform {
    static_assert(core_index < 2u, "the RP2040 has two cores, 0 and 1");

    using CriticalSection = InterruptGuard;

    /// The core this platform is: the identity every kernel static of
    /// this type carries.
    static constexpr uint8_t core = core_index;

    /// The kernel timebase this program runs on - this core's.
    using Timebase = TB;

    /// The bell a send to an AO of this core rings (util/inbox.hpp).
    using Doorbell = SioDoorbell<core_index>;

    /// Whether the calling core is this one: SIO's CPUID against `core`.
    /// What EventQueue::push checks before it copies.
    static bool on_own_core() { return SIO->CPUID == core_index; }

    /// The kernel's idle: entered by the loop, in main-loop context on
    /// this platform's core, with interrupts masked and nothing to do;
    /// sleeps until the next interrupt (or runs sleep_hook instead) and
    /// returns with interrupts enabled. Not for a program to call.
    ///
    /// WFI first, unmask after. On Cortex-M a pending interrupt wakes
    /// WFI even with PRIMASK set - the handler simply does not run until
    /// PRIMASK clears. So sleeping before unmasking closes the
    /// lost-wakeup window by construction: an interrupt that becomes
    /// pending between the caller's queue check and the WFI does not put
    /// the core to sleep at all.
    ///
    /// What the sleep is on this chip: the core's clock stops, everything
    /// else runs; when both cores sleep and the DMA is idle the clock
    /// enables switch from the WAKE_EN to the SLEEP_EN registers (2.15.3.5,
    /// all clocks enabled at reset, so nothing changes until a power model
    /// decides otherwise). SCR.SLEEPDEEP is never written here: this core
    /// has no deeper mode of its own, the chip's DORMANT is a clocks
    /// matter for a sleep site to arm.
    ///
    /// The DSB is the ARM recommendation for WFI: it retires the posted
    /// writes before the core stops.
    static void idle() {
        if (sleep_hook != nullptr) {
            sleep_hook();
        } else {
            __DSB();
            __WFI();
        }
        __enable_irq();
    }

    /// The one stop that is not a WFI. The chip's DORMANT state is
    /// entered by a register write, not by the sleep instruction, so a
    /// site that arms it hands the idle path the function that does it
    /// (rp2040/sleep.hpp: the clocks onto the oscillator, the keyword,
    /// the tree restored after the wake) and takes it back when it
    /// disarms. Called with interrupts masked, like the WFI it replaces -
    /// but its wake list is the DORMANT's own: 2.11.3 ends the state on a
    /// GPIO event or the RTC's interrupt and on nothing else, so a line
    /// that was pending at the call, or turns pending on the way in, is
    /// served after one of those and not at once. That is the deep rung's
    /// bargain, the one the power model's round of votes exists to strike
    /// (util/power.hpp). Null = the WFI, which is every other rung of the
    /// ladder and keeps the wake of every line (docs/rp2040/platform.md).
    static inline void (*sleep_hook)() = nullptr;

    /// PRIMASK readback: the one bit CriticalSection saves and restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Whether the caller runs inside an exception handler: IPSR's
    /// exception number, zero in thread mode (cortexm/nvic.hpp's
    /// in_handler(); the optional member of kernel/platform.hpp).
    static bool in_handler() { return brio::in_handler(); }

    /// Halt in the debugger.
    ///
    /// Caveat, ARMv6-M. The core cannot ask whether a debugger is
    /// attached (DHCSR is debugger-access-only), so BKPT cannot be made
    /// conditional: with no debugger halted on it, this escalates to
    /// isr_hardfault, which the crt provides as a distinct spin loop so
    /// the wreck is legible in a backtrace. And with C_DEBUGEN left set
    /// by a flashing tool the core halts here in silence, which is why
    /// bin/brio clears DHCSR after every flash.
    static void break_here() { __BKPT(0); }

    /// Kernel time in ticks of the timebase; any context.
    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (rp2040/ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here -
    /// within one core. Between the two cores atomicity is not
    /// ordering, and the bridge that crosses carries its own fences.
    static constexpr unsigned atomic_width = 4;

    /// This family places `.fast_text` in SRAM: rp2040/ld/rp2040_2m.ld
    /// collects it first in .data, which the crt copies before main(), and
    /// the vector table holds absolute addresses, so a vector placed there
    /// is entered directly; a CALL between the flash and SRAM is out of a
    /// BL's 16 MB and goes through the linker's long-branch veneer. What it
    /// buys here is the XIP cache's fill on a cold path, not a warm path's
    /// speed (docs/rp2040/platform.md, "A handler in SRAM"). A driver of
    /// this stratum may therefore be given util/placement.hpp's InRam.
    static constexpr bool places_fast_text = true;

    /// Which core runs this: SIO.CPUID, 0 or 1 (2.3.1.1); any context.
    static uint8_t core_id() { return static_cast<uint8_t>(SIO->CPUID); }

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and the crt neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot. The
    /// SRAM is not powered down by any reset short of a power cycle, and
    /// the magic word take_panic_record() checks is what makes a cold
    /// word harmless. One record per core: a static of this template.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name and gas
    // warns; harmless.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<Rp2040Platform<>>);
static_assert(Platform<Rp2040Platform<1>>);

/**
 * Rp2040RubatoPlatform<core_index, TB>: the platform a Rubato kernel runs
 * on (kernel/rubato.hpp), the preemptive one, for one core. It is
 * Rp2040Platform<core_index, TB> whole - the same critical section, idle,
 * sleep hook, timebase, breakpoint, panic storage, core identity and
 * doorbell - plus the three members kernel/platform.hpp's Preemptive
 * concept asks for:
 *
 *  - `preemptive`, true;
 *  - `ceiling`, the byte the kernel keeps: the pack index of the object
 *    being dispatched, 255 at the loop's base. A static of the platform
 *    type, and the type is the core, so each core has its own ceiling
 *    and the two never meet;
 *  - `pend_schedule()`, the request for the scheduler, which is the core's
 *    and not Raspberry Pi's: cortexm/schedule.hpp's PendSV, pended in the
 *    ICSR of the core that asks and taken by that core, at the lowest of
 *    the Cortex-M0+'s four levels (the byte 0xC0), below every NVIC line
 *    at its reset priority 0 - and asked only from the platform's own
 *    core, so that a post from the other core, which the queue refuses,
 *    pends nothing (the member's comment). PendSV returns into the
 *    scheduler in thread mode, and the scheduler leaves through SVC; that
 *    file says how, with the frames it builds (always basic: this core
 *    has no FPU).
 *
 * WHAT THIS CHIP ADDS: ONE VECTOR TABLE FOR TWO CORES. Core 1 is launched
 * with core 0's VTOR (rp2040/multicore.hpp), so `isr_pendsv` and
 * `isr_svcall` are one entry each for both cores. A program with one
 * Rubato, on core 0, core 1 under Tenuto or not launched, binds them as
 * every Cortex-M program does - core 1 never pends PendSV: a Tenuto
 * kernel never asks, and a post of core 1's to an object of core 0 is
 * refused by the queue and asks nothing (pend_schedule() below):
 *
 *     using P = brio::Rp2040RubatoPlatform<>;
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception<K>(); }
 *     extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }
 *     int main() { ...; brio::scheduler_init(); brio::enable_interrupts(); K::run(); }
 *
 * A program with a Rubato on each core binds PendSV to the body that asks
 * which core took it, rp2040/schedule.hpp's scheduler_exception_pair<K0,
 * K1>(), and each core calls scheduler_init() itself, before it enables
 * its interrupts (PendSV's priority is a register of each core); the SVC
 * body serves both cores as it is. Which of two ways a two-kernel program
 * should take - that body, or a vector table per core - is an open
 * decision, rp2040/schedule.hpp's header states both with their costs.
 *
 * The CriticalSection is still PRIMASK alone, per core: the peripheral
 * handlers still share one priority and never nest among themselves; the
 * scheduler's exception sits below them all, and they interrupt it. A
 * request made with PRIMASK set is taken when the mask lifts, so a post
 * inside a critical section serves its receiver when the section ends. A
 * post from the other core is a send through the inbox (util/inbox.hpp),
 * whose doorbell handler posts locally: road 1 on the receiving core,
 * taken at that handler's exit. A program on this platform executes no
 * `svc` of its own.
 */
template <uint8_t core_index = 0, class TB = CoreTicker<core_index>>
struct Rp2040RubatoPlatform : Rp2040Platform<core_index, TB> {
    static constexpr bool preemptive = true;

    /// The index of the object being dispatched on this core; 255 at the
    /// loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler, made only when the calling core is
    /// this platform's. post() asks after the queue's push whenever the
    /// receiver's index is below this ceiling, and a post from the other
    /// core is a mispost the push has already refused and counted
    /// (kernel/event_queue.hpp); asking then would pend the posting
    /// core's PendSV, and with the single body bound - a Rubato on core
    /// 0, core 1 under Tenuto - PendSV taken on core 1 would run core 0's
    /// kernel there, one kernel on two cores. Asked from here only on its
    /// own core, a mispost stays what the queue made it: a count.
    ///
    /// The body is cortexm/schedule.hpp's request_scheduler() - PENDSVSET,
    /// then the DSB and the ISB, in a call the compiler cannot see into,
    /// for the reason that function gives - with SIO's CPUID compared
    /// first, inside the same opaque call: written as a test around a
    /// call of request_scheduler(), GCC 16 at -Os splits the test into a
    /// helper of its own, and the asking path pays a second call, a
    /// frame and its return around the test, 16 cycles in all by the
    /// release listing, where the test alone costs five here (the
    /// CPUID's address built and loaded over the IO port, the compare,
    /// the branch: table 81 of the datasheet).
    [[gnu::noinline, gnu::noipa]] static void pend_schedule() {
        if (SIO->CPUID == core_index) {
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;   // every other ICSR bit written 0: no effect
            __DSB();
            __ISB();
        }
    }
};

static_assert(Platform<Rp2040RubatoPlatform<>>);
static_assert(Preemptive<Rp2040RubatoPlatform<>>);
static_assert(Preemptive<Rp2040RubatoPlatform<1>>);
static_assert(!Preemptive<Rp2040Platform<>>);

} // namespace brio
