/*
 * platform.hpp
 *
 * CH32V203 (QingKe V4B, RV32IMAC) implementation of the brio Platform
 * concept - the one header of this stratum the kernel templates are
 * instantiated with. An app selects it by including it and passing
 * Ch32vx03Platform along.
 *
 * THE SECOND QINGKE FAMILY, AND THE BIGGER CORE OF THE TWO: thirty-two
 * registers where the CH32V00x has sixteen, a divider, an atomic
 * extension, user mode, a 64-bit system counter and up to 144 MHz. What
 * does NOT change is the contract above it: the kernel and util strata
 * compile here untouched, which is the fact this target exists to keep
 * true.
 *
 * CriticalSection is ch32vx03/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * All-or-nothing masking. This core DOES have a priority threshold and
 * interrupt nesting, and brio uses neither: an ISR body runs to
 * completion here as everywhere (docs/design/kernel.md section 1).
 *
 * IDLE() ON THIS FAMILY ASKS THE BUS FIRST, and that is this target's
 * one departure from the others. In a sleep of any depth here no bus
 * master but the core gets a cycle (ch32vx03/bus_activity.hpp carries
 * the measurements and the reasoning), so a sleep taken while a DMA
 * channel or the USB controller is working does not slow the program
 * down - it loses the transfer. The platform therefore reads the count
 * of active masters and SLEEPS ONLY AT ZERO; above zero it returns at
 * once with interrupts enabled, which the Platform contract allows (the
 * host's idle() returns at once too) and which leaves the loop
 * spinning and the bus served.
 *
 * WHAT IS NOT HERE YET, and is deliberate rather than forgotten: no
 * idle_until() - this platform is not tickless, its timebase is the
 * core counter and stops with the core.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/ticker.hpp"
#include "kernel/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32vx03Platform {
    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on.
    using Timebase = TB;

    /// Whether that timebase has a periodic interrupt to pause across a
    /// deep sleep. True of the STK ticker; a timebase that keeps time
    /// through a Stop would not need it and would not offer the verbs.
    static constexpr bool pauses_tick = requires {
        TB::pause();
        TB::resume();
    };

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
     * been measured here (docs/ch32vx03/README.md's gap list owes it).
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
     * AND IT SLEEPS ONLY WHILE THE CORE OWNS THE BUS. In a sleep of any
     * depth on this family no other master gets a cycle, so a DMA
     * channel or the USB controller that is working would not be slowed
     * by the sleep but broken by it (ch32vx03/bus_activity.hpp). With
     * masters active this hook therefore returns AT ONCE, interrupts
     * enabled, and the loop spins - a legal idle() under the Platform
     * contract, and the honest one here.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this
     * is the plain Sleep of RM 2.3.2 - the core clock gated, the
     * counter and the wake logic alive. With a DEEP mode armed by a
     * sleep site (ch32vx03/sleep.hpp) the timebase is paused and its
     * pending bit cleared first, because the STK stops with HCLK in a
     * Stop and because SEVONPEND makes a tick that is merely PENDING
     * end the sleep before it begins.
     *
     * AND THAT COSTS ONE TICK, KNOWINGLY. A tick that had already
     * fired when the deep sleep begins is dropped instead of served,
     * so kernel time is one tick short of the wall. A TIMED site
     * repairs it for free - its witness measures the wall and
     * subtracts the ticks the counter itself served, so one that was
     * not served is advanced instead - and without one a deep sleep is
     * legal only with no deadline armed, which is the model's own
     * restriction.
     */
    static void idle() {
        if (BusActivity::active() != 0u) {
            enable_interrupts();
            return;
        }
        const bool deep = (pfic_sctlr() & sctlr_sleepdeep) != 0u;
        if (deep) {
            if constexpr (pauses_tick) {
                TB::pause();
            }
            stk()->SR = 0;
            Pfic::clear_pending(Irq::systick);
        }
        pfic_sctlr() = (pfic_sctlr() | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
        __asm__ volatile("wfi" ::: "memory");
        if (deep) {
            if constexpr (pauses_tick) {
                TB::resume();
            }
        }
        enable_interrupts();
    }

    /// How many bus masters other than the core are working - the
    /// number idle() reads, published here so a program (and a suite)
    /// can ask the same question the sleep path asks.
    static uint8_t bus_masters_active() { return BusActivity::active(); }

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

static_assert(Platform<Ch32vx03Platform<>>);

} // namespace brio
