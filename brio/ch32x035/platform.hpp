/*
 * platform.hpp
 *
 * CH32X035 (QingKe V4C, RV32IMAC) implementation of the brio Platform
 * concept - the one header of this stratum the kernel templates are
 * instantiated with. An app selects it by including it and passing
 * Ch32x035Platform along.
 *
 * THE QINGKE V4C, THE CH32V203's CORE GENERATION: the V4C is the V4B
 * with a faster divider and the physical memory protection unit (the
 * reference manual's core table) - thirty-two registers, the atomic
 * extension, user mode, a 64-bit system counter - at a ceiling of 48 MHz.
 * What does NOT change is the contract above it: the kernel and util
 * strata compile here untouched.
 *
 * CriticalSection is ch32x035/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * All-or-nothing masking. This core DOES have a priority threshold and
 * two levels of interrupt nesting, and brio uses neither: an ISR body
 * runs to completion here as everywhere (docs/design/kernel.md section
 * 1).
 *
 * WHAT IS NOT HERE, and is deliberate rather than forgotten: no
 * idle_until() - this platform is not tickless, its timebase is the core
 * counter and stops with the core - and no count of bus masters. On the
 * CH32V203 and the CH32V303 no master but the core gets a bus cycle in a
 * sleep of any depth, so that stratum's idle path reads a count of
 * working DMA channels and USB controllers and sleeps only at zero
 * (brio/ch32vx03/bus_activity.hpp). Whether this part does the same is a
 * measurement, and until a driver of this stratum starts a bus master
 * (the DMA, the USB controllers) there is nothing for such a count to
 * hold.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/ticker.hpp"
#include "kernel/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32x035Platform {
    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on.
    using Timebase = TB;

    /// Whether that timebase has a periodic interrupt to pause across a
    /// deep sleep. True of the STK ticker.
    static constexpr bool pauses_tick = requires {
        TB::pause();
        TB::resume();
    };

    /**
     * Entered with interrupts MASKED and nothing to do: sleep until the
     * next interrupt.
     *
     * NOT A PLAIN WFI, AND THE REASON IS THE SISTER FAMILIES'. The RISC-V
     * privileged specification lets WFI resume when an interrupt is
     * pending whatever mstatus.MIE says, and the Cortex-M targets lean on
     * the same property to sleep first and unmask after. The QingKe V2C
     * does NOT have it - measured on the CH32V00x, a `wfi` with MIE clear
     * sleeps past every pending interrupt for ever - and the QingKe V4
     * manual (6.2) wakes a WFI on "the interrupt source responded by the
     * interrupt controller", a wording that does not say whether the
     * global mask must be set for it. Whether this core keeps the
     * specification's promise has not been measured.
     *
     * So this hook uses the idiom that is correct under BOTH readings.
     * PFIC_SCTLR.WFITOWFE makes the NEXT wfi behave as a wait-for-EVENT,
     * and SEVONPEND makes every interrupt entering the pending state an
     * event, masked or not - with the event LATCHED, so an interrupt that
     * turned pending between the caller's queue check and this
     * instruction makes the wfi return at once instead of sleeping (RM
     * 7.5.2.38: "if the WFE instruction is not executed, the system will
     * be woken up immediately after the next execution of the
     * instruction"). The core wakes and continues HERE, with MIE still
     * clear; the unmask that follows is what lets the interrupt be taken.
     * The lost-wakeup window is closed by the latch, not by instruction
     * ordering. WCH's own __WFI() for this series is a WFE as well, with
     * a SETEVENT and a doubled wfi.
     *
     * WFITOWFE is written before every wfi because it is spent on the
     * subsequent WFI; SEVONPEND is sticky but costs nothing to re-assert
     * in the same store.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this is
     * the plain Sleep of RM 2.3.2 - the core clock stopped, every
     * peripheral clock running. No verb of this stratum sets SLEEPDEEP;
     * a program that does has armed a Stop or a Standby (RM 2.3.3, 2.3.4),
     * and this hook then pauses the timebase and clears its pending bit
     * first, as the sister families' hooks do, because a tick that is
     * merely PENDING would end the sleep before it began under SEVONPEND.
     */
    static void idle() {
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

    /// mstatus.MIE readback: the one bit CriticalSection saves.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /**
     * Halt in the debugger.
     *
     * CAVEAT. With a debugger halted on it, the program stops here. With
     * NO debugger attached, `ebreak` raises the breakpoint exception,
     * which the CH32V203's core takes to the vector table's own
     * breakpoint entry (measured there); the crt of this project gives
     * that entry a weak spin loop of its own, apart from the exception
     * entry's and the unbound vectors', so a probe that halts the core
     * reads which wreck it was from the program counter. A reporter that
     * must survive without a debugger cannot assume this call returns.
     */
    static void break_here() { __asm__ volatile("ebreak"); }

    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on the core counter.
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path.
    static constexpr unsigned atomic_width = 4;

    /**
     * Panic breadcrumb in .noinit: the linker script places the section
     * and the crt neither loads nor zeroes it, so a record written just
     * before a reset can be reported at the next boot.
     *
     * What survives a reset on this silicon is not promised anywhere -
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
 * (`__stack_floor`, one past .noinit) and the stack top with one pattern;
 * this walks up from the floor until the pattern breaks. A word of the
 * pattern legitimately on the stack ends the walk early: the answer errs
 * on the small side.
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

static_assert(Platform<Ch32x035Platform<>>);

} // namespace brio
