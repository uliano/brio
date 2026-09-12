/*
 * platform.hpp
 *
 * RP2040 (Cortex-M0+) implementation of the brio Platform concept - the
 * one header of this stratum the kernel templates are instantiated with.
 * Apps select it by including it and passing Rp2040Platform<> along.
 *
 * ONE PLATFORM TYPE PER CORE. The first template parameter is the core,
 * 0 or 1: the kernel's statics are keyed by the platform type, so
 * `Kernel<Rp2040Platform<0>, ...>` and `Kernel<Rp2040Platform<1>, ...>`
 * are two kernels with two packs, two timebases (the second parameter,
 * the core's own SysTick ticker by default) and two breadcrumbs, and
 * the type IS the core identity everywhere the kernel asks P (the
 * model: util/inbox.hpp, design/kernel.md). `Rp2040Platform<>` is core
 * 0's, what a single-core program names. The two optional members of a
 * multi-core platform (kernel/platform.hpp) are here: `on_own_core()`
 * reads SIO's CPUID, so a post to the other core's queue is refused
 * and counted; `Doorbell` is the SIO FIFO towards this core
 * (rp2040/multicore.hpp), what a send to an AO of this core rings.
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
 * ARMv6-M has no BASEPRI, so this is all-or-nothing masking - AND IT IS
 * PER CORE: PRIMASK is a register of the core that sets it, so a
 * critical section here excludes this core's handlers and nothing that
 * runs on the other core. Everything the kernel guards with it is
 * therefore correct within one core, which is what the kernel promises;
 * anything shared between two kernels is a bridge with its own
 * discipline, not a CriticalSection.
 *
 * core_id() reads SIO's CPUID: the core that is running, whatever the
 * platform type says it should be.
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "kernel/platform.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/ticker.hpp"

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
    /// WHAT THE SLEEP IS on this chip: the core's clock stops, everything
    /// else runs; when BOTH cores sleep and the DMA is idle the clock
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

    /// THE ONE STOP THAT IS NOT A WFI. The chip's DORMANT state is
    /// entered by a register write, not by the sleep instruction, so a
    /// site that arms it hands the idle path the function that does it
    /// (rp2040/sleep.hpp: the clocks onto the oscillator, the keyword,
    /// the tree restored after the wake) and takes it back when it
    /// disarms. Called with interrupts masked, like the WFI it replaces,
    /// and with the same closure of the lost-wakeup window: a wake that
    /// is already pending ends the dormant state at once. Null = the
    /// WFI, which is every other rung of the ladder.
    static inline void (*sleep_hook)() = nullptr;

    /// PRIMASK readback: the one bit CriticalSection saves and restores.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Halt in the debugger.
    ///
    /// CAVEAT, ARMv6-M. The core cannot ask whether a debugger is
    /// attached (DHCSR is debugger-access-only), so BKPT cannot be made
    /// conditional: with no debugger halted on it, this escalates to
    /// isr_hardfault, which the crt provides as a distinct spin loop so
    /// the wreck is legible in a backtrace. And with C_DEBUGEN left set
    /// by a flashing tool the core HALTS here in silence, which is why
    /// bin/brio clears DHCSR after every flash.
    static void break_here() { __BKPT(0); }

    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (rp2040/ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path here -
    /// within one core. Between the two cores atomicity is not
    /// ordering, and the bridge that crosses carries its own fences.
    static constexpr unsigned atomic_width = 4;

    /// Which core runs this: SIO.CPUID, 0 or 1 (2.3.1.1).
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

} // namespace brio
