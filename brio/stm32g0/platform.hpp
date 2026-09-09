/*
 * platform.hpp
 *
 * STM32G0 (Cortex-M0+) implementation of the brio Platform concept - the
 * one header of this stratum the kernel templates are instantiated with.
 * Apps select it by including it and passing Stm32g0Platform<> along.
 *
 * ONE CLASS, TWO TIMEBASES. The template parameter is the kernel
 * timebase, and the default is the SysTick BasicTicker of
 * stm32g0/ticker.hpp (`Ticker`, 1000 Hz). SysTick rides HCLK, and HCLK
 * stops in the Stop modes - so on that
 * timebase KERNEL TIME STANDS STILL across a Stop, and stm32g0/sleep.hpp
 * keeps two timed sites that repair it afterwards from an RTC or LPTIM
 * witness. The other argument is stm32g0/lptim_ticker.hpp's
 * `LptimTicker`: kernel time counted on a low-power timer clocked from
 * the LSE crystal, which keeps counting through Stop 0 and Stop 1. That
 * timebase is TICKLESS - there is no periodic interrupt at all, and this
 * platform then provides the kernel's OPTIONAL idle_until() hook
 * (kernel/platform.hpp): the loop hands it the nearest armed deadline,
 * the timebase places its wake there, and one WFI covers the whole wait
 * whatever depth a sleep site armed. The plain sleep site is the only
 * site such a program takes (the timed ones refuse it at compile time:
 * nothing stands still, nothing needs repairing), and SysTick keeps
 * running interrupt-less as armv6m/delay.hpp's cycle counter. The
 * price: one LPTIM, its vector bound by the app to the ticker's isr().
 *
 * CriticalSection is stm32g0/nvic.hpp's InterruptGuard: save PRIMASK,
 * cpsid i, restore on scope exit. Nesting-correct, and the CMSIS
 * intrinsics behind it carry the "memory" clobbers the concept asks for.
 * ARMv6-M has no BASEPRI, so this is all-or-nothing masking.
 *
 * The other halves live next door: which reset happened and how to cause
 * one is stm32g0/reset.hpp, and the STOPPING half - PWR's
 * Sleep/Stop/Standby/Shutdown ladder and the util/power.hpp sites over
 * it - is stm32g0/pwr.hpp plus stm32g0/sleep.hpp. This silicon selects
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
    /// SCR.SLEEPDEEP plus PWR_CR1.LPMS (RM0444 4.3), and this hook never
    /// writes either: out of reset SLEEPDEEP is 0, so a WFI here is
    /// Sleep - the CPU clock stops, HCLK, SysTick and every peripheral
    /// keep running (5.3) - and with stm32g0/sleep.hpp's site having
    /// armed a Stop, the same WFI is that Stop. The site arms above this
    /// hook and the hook takes what it finds. On the SysTick timebase
    /// KERNEL TIME STOPS
    /// in a Stop (SysTick rides HCLK), which is what the timed sites
    /// exist to repair; on a Tickless timebase it does not, and the loop
    /// calls idle_until() below instead of this.
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

    /// Halt in the debugger.
    ///
    /// CAVEAT, ARMv6-M. The core cannot ask whether a debugger is
    /// attached (DHCSR is debugger-access-only), so BKPT cannot be made
    /// conditional: with no debugger halted on it, this escalates to
    /// HardFault_Handler, which the crt provides as a distinct spin loop
    /// so the wreck is legible in a backtrace. And with C_DEBUGEN left
    /// set by a flashing tool the core HALTS here in silence, which
    /// is why tools/bench.py clears DHCSR after every flash.
    static void break_here() { __BKPT(0); }

    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on SysTick (stm32g0/ticker.hpp),
    /// 1024 Hz on the default LptimTicker (stm32g0/lptim_ticker.hpp).
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

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

} // namespace brio
