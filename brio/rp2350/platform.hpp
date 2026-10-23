/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on an RP2350: this chip's
 * realization of the kernel's Platform concept, the compiler-checked set
 * of requirements kernel/platform.hpp states. A program names it once,
 * as the first argument of its Tenuto, and every kernel template is
 * instantiated with it; the same type serves the Cortex-M33 pair and
 * the Hazard3 pair:
 *
 *     using P = brio::Rp2350Platform<>;
 *     extern "C" void isr_systick() { brio::Ticker::tick(); }
 *     ...
 *     SysClock::init();
 *     brio::Mtime::start(clock);
 *     brio::Ticker::init(clock);
 *     brio::enable_interrupts();
 *     brio::Tenuto<P, MyAo>::run();
 *
 * (SysClock and clock are the program's clock type and its tag object,
 * rp2350/clock.hpp, and SysClock::init() is the clock task, the call
 * that brings the clock tree up; Mtime is the ruler, the platform
 * timer's microsecond counter (rp2350/mtime.hpp), which the Hazard3
 * ticker counts on and which the Arm half needs only to measure or to
 * wait; MyAo stands for the program's active objects, the components
 * with an event queue of their own that the kernel serves one event at
 * a time; `extern "C"` gives the handler the plain C name the crt's
 * vector binds; docs/rp2350/platform.md reads the lines one by one.)
 *
 * It gives the kernel a critical section (interrupts masked on this
 * core for a scope), an idle that sleeps until the next interrupt
 * without losing one that arrives at the wrong moment, the program's
 * source of time, a breakpoint for a panic and the storage a panic
 * record survives a reset in. Who calls what: the kernel loop calls
 * idle() in main-loop context, with interrupts already masked, and
 * nothing else should; now() and ticks_per_second may be read from any
 * context; CriticalSection may be opened from any context and nests;
 * on_own_core() is what the kernel's queue asks before a post copies an
 * event; break_here() is the panic path's and panic_record() the panic
 * path's and the next boot's; sleep_hook is written by a sleep site
 * (rp2350/sleep.hpp) and read by idle(). A program binds the timebase's
 * interrupt vector itself (isr_systick to Ticker::tick(), the same name
 * on both architectures); the kernel binds no vector, and a program that
 * forgets it stops in the crt's default spin at the first tick. The
 * clock task runs first, because the SysTick reload and the ruler's
 * divisor are computed from the rates the clock type claims.
 *
 * One platform type per core, as on the RP2040. The first template
 * parameter is the core, 0 or 1: the kernel's statics are keyed by the
 * platform type, so `Tenuto<Rp2350Platform<0>, ...>` and
 * `Tenuto<Rp2350Platform<1>, ...>` are two kernels with two packs (a
 * pack is a program's list of active objects, the arguments of its
 * Tenuto), two timebases (the second parameter, the core's own ticker by
 * default) and two breadcrumbs, and the type is the core identity
 * everywhere the kernel asks P. `Rp2350Platform<>` is core 0's, what a
 * single-core program names. The two optional members of a platform
 * that is one core of several (kernel/platform.hpp) are here:
 * `on_own_core()` reads SIO's CPUID, so a post to the other core's queue
 * is refused and counted (the queue's misposts(), kernel/event_queue.hpp); `Doorbell` is the SIO doorbell towards this
 * core (rp2350/multicore.hpp), what a send to an active object of this
 * core rings.
 *
 * And one platform type for two architectures. Everything
 * architecture-specific here is a name from rp2350/core.hpp - the
 * critical section, the sleep, the breakpoint, the core number - so this
 * file is the same on the Cortex-M33 half and the Hazard3 half, and so
 * is every kernel and util header above it. What the two halves do
 * differently is one level down: PRIMASK against mstatus.MIE, WFI
 * against wfi, BKPT against ebreak, SysTick against the platform timer.
 *
 * The idle path is the same shape on both, and on both it is free of a
 * lost-wakeup window: an interrupt that becomes pending between the
 * caller's queue check and the sleep instruction does not put the core
 * to sleep at all. On the M33 that is ARM's rule (a pending interrupt
 * wakes WFI even with PRIMASK set); on Hazard3 it is datasheet 3.8.1.23's
 * (wfi ignores mstatus.MIE and respects every other interrupt control).
 * The two are the same promise in two spellings, which is why the
 * kernel's loop needs no target knowledge.
 *
 * The platform offers no idle_until(): the timebase runs while the core
 * sleeps in either architecture, in the plain idle() path and through a
 * standby, and the one state that stops it, DORMANT, is repaired after
 * the wake only by the timed sleep site's resync from the always-on
 * timer (Rp2350TimedSleepSite, rp2350/sleep.hpp); under the plain site
 * the deep rung has no witness, and the time a dormant spent is not
 * caught up. What the platform does offer is `sleep_hook`, the
 * entry a low-power state that is not a sleep instruction takes (the
 * RP2040's DORMANT was the first of those): rp2350/sleep.hpp's site - the
 * object a program arms with how deep the chip may sleep, on a ladder of
 * rungs, light, standby and deep - installs its DORMANT entry there on
 * core 0's platform while the deep rung is armed. A program that uses no
 * sleep site leaves sleep_hook alone. POWMAN's power-down states are not
 * reached through it: leaving one is a boot, not a wake
 * (rp2350/powman.hpp).
 *
 * The critical section is per core in both spellings: PRIMASK and
 * mstatus belong to the core that writes them, so a critical section
 * here excludes this core's handlers and nothing that runs on the other
 * core. Everything the kernel guards with it is therefore correct within
 * one core, which is what the kernel promises; anything shared between
 * two kernels is a bridge with its own discipline (util/inbox.hpp's
 * inbox, rp2350/multicore.hpp's doorbell).
 */

#pragma once

#include <stdint.h>

#include "rp2350/core.hpp"

#include "kernel/platform.hpp"
#include "rp2350/multicore.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/ticker.hpp"

namespace brio {

template <uint8_t core_index = 0, class TB = CoreTicker<core_index>>
struct Rp2350Platform {
    static_assert(core_index < 2u, "the RP2350 has two cores per architecture, 0 and 1");

    using CriticalSection = InterruptGuard;

    /// The core this platform is: the identity every kernel static of
    /// this type carries.
    static constexpr uint8_t core = core_index;

    /// The kernel timebase this program runs on - this core's.
    using Timebase = TB;

    /// The bell a send to an active object of this core rings
    /// (util/inbox.hpp).
    using Doorbell = SioDoorbell<core_index>;

    /// Whether the calling core is this one: SIO's CPUID against `core`.
    /// What EventQueue::push checks before it copies; callable from any
    /// context, one register read.
    static bool on_own_core() { return SIO->CPUID == core_index; }

    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt (the file header on why that has no lost-wakeup window on
    /// either architecture) - or, when a sleep site has installed
    /// sleep_hook, takes that state instead - and returns with interrupts
    /// enabled. Not for a program to call.
    static void idle() {
        if (sleep_hook != nullptr) {
            sleep_hook();
        } else {
            wait_for_interrupt();
        }
        enable_interrupts();
    }

    /// The one stop that is not a sleep instruction. A state entered by a
    /// register write rather than by wfi hands the idle path the function
    /// that does it, and takes it back when it disarms. Called by idle()
    /// with interrupts masked, like the sleep it replaces. Null = the
    /// plain sleep, which the light and standby rungs use;
    /// rp2350/sleep.hpp's site sets core 0's to its DORMANT entry when it
    /// arms the deep rung and back to null when it arms another rung or
    /// disarms. Written from the site's arm() and disarm() in main
    /// context, and from interrupt context too: the timed site's isr(),
    /// in the program's isr_powman_timer, calls disarm(). idle() reads it
    /// with interrupts masked, so no write lands between its read and the
    /// sleep.
    static inline void (*sleep_hook)() = nullptr;

    /// The global mask's readback: the one bit CriticalSection saves and
    /// restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Halt in the debugger: the panic path's breakpoint
    /// (kernel/panic.hpp), callable from any context.
    ///
    /// With no debugger attached this escalates - to a HardFault on the
    /// M33, to the crt's exception trap on Hazard3 - a fault entry the
    /// program binds to rp2350/reset.hpp's fault_reset() to reboot with
    /// the breadcrumb kept, or that spins where a debugger's backtrace
    /// names it. With a probe attached and halting debug enabled the core
    /// stops here instead, which is why the bench's flash verb (brio
    /// flash, docs/rp2350/README.md, "Upload") takes halting debug back
    /// down after programming an Arm image.
    static void break_here() { debug_break(); }

    /// The kernel's clock: the timebase's tick count, readable from any
    /// context (what kernel/time_event.hpp arms deadlines against).
    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on both halves
    /// (rp2350/ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here -
    /// within one core. Between two cores atomicity is not ordering, and
    /// the bridge that crosses carries its own fences.
    static constexpr unsigned atomic_width = 4;

    /// This family places `.fast_text` in SRAM, under both architectures:
    /// rp2350/ld/rp2350_16m.ld collects it first in .data, which both crts
    /// copy before main(); a vector there is entered directly (the Arm
    /// table and the Hazard3 dispatch's table hold absolute addresses),
    /// and a CALL from the flash into SRAM is, on the Cortex-M33, out of a
    /// BL's 16 MB and goes through the linker's veneer, while Hazard3's
    /// auipc and jalr reach it. What it buys here is the XIP cache's fill
    /// on a cold path, not a warm path's speed (docs/rp2350/platform.md,
    /// "A handler in SRAM"). A driver of this stratum may therefore be
    /// given util/placement.hpp's InRam.
    static constexpr bool places_fast_text = true;

    /// Which core runs this: SIO.CPUID, 0 or 1, whichever architecture
    /// asks.
    static uint8_t core_id() { return brio::core_id(); }

    /// Panic breadcrumb in .noinit: the linker script marks the section
    /// NOLOAD and the crt neither loads nor zeroes it, so the record
    /// survives a warm reset and can be reported at the next boot. One
    /// record per core: a static of this template.
    static PanicRecord& panic_record() { return panic_record_; }

private:
    // gcc emits the COMDAT section for an inline variable with a custom
    // section attribute without the group name and gas warns; harmless.
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

static_assert(Platform<Rp2350Platform<>>);
static_assert(Platform<Rp2350Platform<1>>);

/**
 * Rp2350RubatoPlatform<core, TB>: the platform a Rubato kernel runs on
 * (kernel/rubato.hpp), the preemptive one. It is Rp2350Platform<core, TB>
 * whole - the same critical section, idle, timebase, breakpoint, panic
 * storage, core identity and doorbell - plus the three members
 * kernel/platform.hpp's Preemptive concept asks for, and the same text on
 * both processor architectures:
 *
 *     using P = brio::Rp2350RubatoPlatform<>;
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     BRIO_RP2350_SCHEDULER_VECTORS(K)
 *     int main() { ...; brio::scheduler_init(); brio::enable_interrupts(); K::run(); }
 *
 *  - `preemptive`, true;
 *  - `ceiling`, the byte the kernel keeps: the pack index of the object
 *    being dispatched, 255 at the loop's base. A static of the platform
 *    type, so of the core;
 *  - `pend_schedule()`, the request for the scheduler (rp2350/
 *    schedule.hpp): on the Cortex-M33 the core stratum's PendSV at the
 *    lowest of the core's sixteen levels (0xF0, below every NVIC line at
 *    its reset priority 0), returning into the scheduler in thread mode,
 *    the scheduler leaving through SVC; on Hazard3 the machine software
 *    interrupt, the SIO flag of the asking hart, through the crt's
 *    trampoline into the scheduler's entry, which leaves by an `mret` of
 *    its own. Both files say how, with the stack each level costs.
 *
 * The CriticalSection is still PRIMASK on one half and mstatus.MIE on the
 * other, and the promise that an interrupt handler is not interrupted by
 * another keeps its meaning: on the Cortex-M33 the peripheral lines share
 * one priority and the scheduler's exception sits below them all; on
 * Hazard3 no handler ever runs with MIE set, and the scheduler runs
 * outside every trap. A request made with the mask set is taken when the
 * mask lifts, so a post inside a critical section serves its receiver
 * when the section ends. A program on this platform binds the
 * scheduler's vectors (the macro above: two on one half, one on the
 * other), executes no `svc` of its own on the Cortex-M33 and leaves
 * mscratch alone on Hazard3, and calls scheduler_init() before it
 * enables interrupts.
 *
 * CORE 0 ONLY. Both crts give the two cores of a pair one table, so a
 * second Rubato on core 1 would enter the same scheduler vectors with a
 * second kernel to serve; which kernel a vector serves is the two-kernel
 * question the RP2040 shares, and until it is answered core 1 runs
 * Tenuto (rp2350/multicore.hpp).
 */
template <uint8_t core_index = 0, class TB = CoreTicker<core_index>>
struct Rp2350RubatoPlatform : Rp2350Platform<core_index, TB> {
    static_assert(core_index == 0u,
                  "brio Rp2350RubatoPlatform: a Rubato kernel runs on core 0 alone - both cores "
                  "of a pair enter one table of scheduler vectors, and the two-kernel body is not "
                  "written (rp2350/schedule.hpp)");

    static constexpr bool preemptive = true;

    /// The index of the object being dispatched; 255 at the loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler (rp2350/schedule.hpp).
    [[gnu::always_inline]] static void pend_schedule() { request_scheduler(); }
};

static_assert(Platform<Rp2350RubatoPlatform<>>);
static_assert(Preemptive<Rp2350RubatoPlatform<>>);
static_assert(!Preemptive<Rp2350Platform<>>);

} // namespace brio
