/*
 * platform.hpp
 *
 * CH32V203 (QingKe V4B, RV32IMAC) implementation of the brio Platform
 * concept - the one header of this stratum the kernel templates are
 * instantiated with. An app selects it by including it and passing
 * Ch32v203Platform along.
 *
 * THE SECOND QINGKE FAMILY, AND THE BIGGER CORE OF THE TWO: thirty-two
 * registers where the CH32V00x has sixteen, a divider, an atomic
 * extension, user mode, a 64-bit system counter and up to 144 MHz. What
 * does NOT change is the contract above it: the kernel and util strata
 * compile here untouched, which is the fact this target exists to keep
 * true.
 *
 * CriticalSection is ch32v203/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * All-or-nothing masking. This core DOES have a priority threshold and
 * interrupt nesting, and brio uses neither: an ISR body runs to
 * completion here as everywhere (docs/design/kernel.md section 1).
 *
 * WHAT IS NOT HERE YET, and is deliberate rather than forgotten: no
 * sleep site (the Sleep, Stop and Standby of RM ch. 2 have no user), no
 * reset.hpp to say WHICH reset happened, and no idle_until() - this
 * platform is not tickless, its timebase is the core counter and stops
 * with the core. Each arrives with the driver that needs it.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/device.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/ticker.hpp"
#include "kernel/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32v203Platform {
    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on.
    using Timebase = TB;

    /**
     * Entered with interrupts MASKED and nothing to do: sleep until the
     * next interrupt.
     *
     * NOT A PLAIN WFI, AND THE REASON IS THE SISTER FAMILY'S. The
     * RISC-V privileged specification lets WFI resume when an interrupt
     * is pending whatever mstatus.MIE says, and the Cortex-M targets
     * lean on the same property to sleep first and unmask after. The
     * QingKe V2C does NOT have it - measured on the CH32V00x bench, a
     * `wfi` with MIE clear sleeps past every pending interrupt for ever
     * - and whether this core keeps the specification's promise has not
     * been measured here (docs/ch32v203/README.md's gap list owes it).
     *
     * So this hook uses the idiom that is correct under BOTH readings.
     * PFIC_SCTLR.WFITOWFE makes the NEXT wfi behave as a wait-for-EVENT,
     * and SEVONPEND makes every interrupt entering the pending state an
     * event, masked or not - with the event LATCHED, so an interrupt
     * that turned pending between the caller's queue check and this
     * instruction makes the wfi return at once instead of sleeping. The
     * core wakes and continues HERE, with MIE still clear; the unmask
     * that follows is what lets the interrupt be taken. The lost-wakeup
     * window is closed by the latch, not by instruction ordering.
     *
     * WFITOWFE is written before every wfi because it is spent on the
     * subsequent WFI; SEVONPEND is sticky but costs nothing to
     * re-assert in the same store.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this
     * is the plain Sleep of RM 2.4 - the core clock gated, the counter
     * and the wake logic alive. When a sleep site of this stratum
     * exists, this hook grows the step the other strata have: with a
     * deep mode armed the timebase is paused and its pending bit
     * cleared before the sleep, or the millisecond tick ends the sleep
     * before it begins.
     */
    static void idle() {
        pfic_sctlr() = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
        __asm__ volatile("wfi" ::: "memory");
        enable_interrupts();
    }

    /// mstatus.MIE readback: the one bit CriticalSection saves.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /**
     * Halt in the debugger.
     *
     * CAVEAT. With a debugger halted on it, the program stops here.
     * With NO debugger attached, `ebreak` raises the breakpoint
     * exception, which this core takes to the vector table's own
     * breakpoint entry - the crt provides that as a distinct spin loop,
     * so the wreck is legible instead of running on into nothing. A
     * reporter that must survive without a debugger cannot assume this
     * call returns.
     */
    static void break_here() { __asm__ volatile("ebreak"); }

    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on the core counter.
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible
    /// access, which is what lets util/ring.hpp take its lock-free path.
    static constexpr unsigned atomic_width = 4;

    /**
     * Panic breadcrumb in .noinit: the linker script places the section
     * and the crt neither loads nor zeroes it, so a record written just
     * before a reset can be reported at the next boot.
     *
     * What survives a reset on this silicon is not promised anywhere -
     * the SRAM keeps its contents through a warm reset in practice and
     * the magic word take_panic_record() checks is what makes a cold,
     * random word harmless.
     */
    static PanicRecord& panic_record() { return panic_record_; }

private:
    [[gnu::section(".noinit")]] static inline PanicRecord panic_record_;
};

/**
 * How many bytes of the stack no call has reached since reset. The crt
 * paints the free RAM between the last section the linker placed
 * (`__stack_floor`, one past .noinit) and the stack top with one
 * pattern; this walks up from the floor until the pattern breaks. A word
 * of the pattern legitimately on the stack ends the walk early: the
 * answer errs on the small side.
 */
extern "C" uint32_t __stack_floor[];
extern "C" uint32_t __stack_top[];
inline uint32_t stack_untouched() {
    const volatile uint32_t* p = __stack_floor;
    uint32_t n = 0;
    while (p < __stack_top && *p == 0xA5A5A5A5UL) {
        ++p;
        n += 4u;
    }
    return n;
}

static_assert(Platform<Ch32v203Platform<>>);

} // namespace brio
