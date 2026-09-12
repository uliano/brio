/*
 * platform.hpp
 *
 * CH32V00x (QingKe V2C, RV32EC) implementation of the brio Platform
 * concept - the one header of this stratum the kernel templates are
 * instantiated with. An app selects it by including it and passing
 * Ch32v00xPlatform along.
 *
 * THE SMALLEST MACHINE BRIO RUNS ON, and the one whose instruction set
 * has SIXTEEN registers instead of thirty-two (RV32E): everything the
 * kernel does here it does with half the register file, which is
 * exactly why this target is worth having - it prices the abstractions
 * honestly.
 *
 * CriticalSection is ch32v00x/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * All-or-nothing masking, like the AVR's I flag and ARMv6-M's PRIMASK;
 * this core has no priority threshold that generic code could use.
 *
 * WHAT IS NOT HERE YET, and is deliberate rather than forgotten: no
 * sleep site (the AWU and the standby modes of RM ch. 5 have no user),
 * no reset.hpp to say WHICH reset happened, and no idle_until() - this
 * platform is not tickless, its timebase is the core counter and stops
 * with the core. Each arrives with the driver that needs it.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/ticker.hpp"
#include "kernel/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32v00xPlatform {
    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on.
    using Timebase = TB;

    /**
     * Entered with interrupts MASKED and nothing to do: sleep until the
     * next interrupt.
     *
     * NOT A PLAIN WFI, AND THIS IS THE CORE'S OWN RULE. The RISC-V
     * privileged specification lets WFI resume when an interrupt is
     * pending whatever mstatus.MIE says, and the two Cortex-M0+ targets
     * lean on the same property to sleep first and unmask after. The
     * QingKe V2 does not have it: its manual (5.2) says a WFI sleep is
     * ended by "the interrupt source responded by the interrupt
     * controller", and nothing is responded while MIE is clear - so
     * `wfi` then `csrsi` sleeps past every pending interrupt for ever.
     * Measured on the bench: the tick and the USART both pending, the
     * core asleep with MIE clear, the console silent.
     *
     * The way out is the core's WFE. PFIC_SCTLR.WFITOWFE makes the NEXT
     * wfi behave as a wait-for-EVENT, and SEVONPEND makes every
     * interrupt entering the pending state an event, masked or not -
     * with the event LATCHED, so an interrupt that turned pending
     * between the caller's queue check and this instruction makes the
     * wfi return at once instead of sleeping (3.1's SCTLR: "if the WFE
     * instruction is not executed, the system will be woken up
     * immediately after the next execution"). The core wakes and
     * continues HERE, with MIE still clear; the unmask that follows is
     * what lets the interrupt be taken. The lost-wakeup window is
     * closed by the latch, not by instruction ordering. WCH's own
     * __WFI() is a WFE too - with a manual SETEVENT and a doubled wfi,
     * because it does not use SEVONPEND.
     *
     * WFITOWFE is written before every wfi because the manual says
     * "the subsequent WFI", one-shot, and the vendor's helpers re-set
     * it each time; SEVONPEND is sticky but costs nothing to re-assert
     * in the same store.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this
     * is the plain Sleep of QingKe 5.1 (the core clock gated, STK and
     * the wake logic alive), and with ch32v00x/sleep.hpp's site having
     * armed a Standby the same instruction is that Standby. The hook
     * takes what it finds, as on the other strata - with one thing it
     * does for a Standby, because only this hook can: THE TICK IS HELD
     * OFF ACROSS IT. The STK fires every millisecond, and a tick turning
     * pending is an event that ends the WFE before the Standby begins
     * (the SAM's SysTick had the same power, and its idle() holds it
     * off the same way). So with SLEEPDEEP armed the timebase is paused
     * and its pending bit cleared before the sleep, and resumed after;
     * kernel time stands still for exactly the slept span, which the
     * timed site hands back. A stale latched event (a USART interrupt
     * from just before) still ends the first WFE at once: the kernel
     * loop simply turns and sleeps again, which is why a program that
     * wants a Standby lets the loop idle and does not call this once.
     */
    static void idle() {
        const bool deep = (pfic_sctlr() & sctlr_sleepdeep) != 0u;
        if (deep) {
            TB::pause();
            stk()->SR = 0;
            Pfic::clear_pending(Irq::systick);
        }
        pfic_sctlr() = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
        __asm__ volatile("wfi" ::: "memory");
        if (deep) {
            TB::resume();
        }
        enable_interrupts();
    }

    /// mstatus.MIE readback: the one bit CriticalSection saves.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /**
     * Halt in the debugger.
     *
     * CAVEAT. With a debugger halted on it, the program stops here.
     * With NO debugger attached, `ebreak` raises the breakpoint
     * exception, which this core takes to the vector table's fault
     * entry - the crt provides that as a distinct spin loop, so the
     * wreck is legible instead of running on into nothing. A reporter
     * that must survive without a debugger cannot assume this call
     * returns.
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
 * pattern; this walks up from the floor until the pattern breaks. The
 * number is the margin a program has on a part with 2 KB of RAM - and
 * zero means the stack has already run into the data below it, which
 * no other symptom names. A word of the pattern legitimately on the
 * stack ends the walk early: the answer errs on the small side.
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

static_assert(Platform<Ch32v00xPlatform<>>);

} // namespace brio
