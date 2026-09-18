/*
 * platform.hpp
 *
 * The RP2350's implementation of the brio Platform concept - the one
 * header of this stratum the kernel templates are instantiated with.
 * Apps select it by including it and passing Rp2350Platform<> along.
 *
 * ONE PLATFORM TYPE PER CORE, as on the RP2040. The first template
 * parameter is the core, 0 or 1: the kernel's statics are keyed by the
 * platform type, so `Tenuto<Rp2350Platform<0>, ...>` and
 * `Tenuto<Rp2350Platform<1>, ...>` are two kernels with two packs, two
 * timebases (the second parameter, the core's own ticker by default) and
 * two breadcrumbs, and the type IS the core identity everywhere the
 * kernel asks P. `Rp2350Platform<>` is core 0's, what a single-core
 * program names.
 *
 * AND ONE PLATFORM TYPE FOR TWO ARCHITECTURES. Everything
 * architecture-specific here is a name from rp2350/core.hpp - the
 * critical section, the sleep, the breakpoint, the core number - so this
 * file is the same on the Cortex-M33 half and the Hazard3 half, and so
 * is every kernel and util header above it. What the two halves do
 * differently is one level down: PRIMASK against mstatus.MIE, WFI
 * against wfi, BKPT against ebreak, SysTick against the platform timer.
 *
 * THE IDLE PATH IS THE SAME SHAPE ON BOTH, and on both it is free of a
 * lost-wakeup window: an interrupt that becomes pending between the
 * caller's queue check and the sleep instruction does not put the core
 * to sleep at all. On the M33 that is ARM's rule (a pending interrupt
 * wakes WFI even with PRIMASK set); on Hazard3 it is datasheet 3.8.5's
 * (wfi ignores mstatus.MIE and respects every other interrupt control).
 * The two are the same promise in two spellings, which is why the
 * kernel's loop needs no target knowledge.
 *
 * The platform offers no idle_until(): the timebase runs while the core
 * sleeps in either architecture, so the plain idle() path is the whole
 * story until the power chapter arrives with something deeper. What it
 * does offer is `sleep_hook`, the entry a low-power state that is NOT a
 * sleep instruction will need (the RP2040's DORMANT was the first of
 * those; POWMAN's states are this chip's, and a later chapter's).
 *
 * The critical section is PER CORE in both spellings: PRIMASK and
 * mstatus belong to the core that writes them, so a critical section
 * here excludes this core's handlers and nothing that runs on the other
 * core. Everything the kernel guards with it is therefore correct within
 * one core, which is what the kernel promises; anything shared between
 * two kernels is a bridge with its own discipline.
 */

#pragma once

#include <stdint.h>

#include "rp2350/core.hpp"

#include "kernel/platform.hpp"
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

    /// Whether the calling core is this one: SIO's CPUID against `core`.
    /// What EventQueue::push checks before it copies.
    static bool on_own_core() { return SIO->CPUID == core_index; }

    /// Entered with interrupts MASKED and nothing to do: sleep until the
    /// next interrupt (the file header on why that has no lost-wakeup
    /// window on either architecture), then unmask.
    static void idle() {
        if (sleep_hook != nullptr) {
            sleep_hook();
        } else {
            wait_for_interrupt();
        }
        enable_interrupts();
    }

    /// THE ONE STOP THAT IS NOT A SLEEP INSTRUCTION. A state entered by a
    /// register write rather than by wfi hands the idle path the function
    /// that does it, and takes it back when it disarms. Called with
    /// interrupts masked, like the sleep it replaces. Null = the plain
    /// sleep, which is what every rung of this target's ladder is until
    /// the power chapter offers a deeper one.
    static inline void (*sleep_hook)() = nullptr;

    /// The global mask's readback: the one bit CriticalSection saves and
    /// restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Halt in the debugger.
    ///
    /// With no debugger attached this escalates - to a HardFault on the
    /// M33, to the crt's exception trap on Hazard3 - which is the
    /// legible wreck a panic wants. With a probe attached and halting
    /// debug enabled the core stops here instead, which is why the flash
    /// verb takes halting debug back down after programming.
    static void break_here() { debug_break(); }

    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on both halves
    /// (rp2350/ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here -
    /// within one core. Between two cores atomicity is not ordering, and
    /// the bridge that crosses carries its own fences.
    static constexpr unsigned atomic_width = 4;

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

} // namespace brio
