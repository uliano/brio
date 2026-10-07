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
     * Entered with interrupts MASKED and nothing to do: unmask, then sleep
     * until the next interrupt, and return with interrupts enabled.
     *
     * NOT A PLAIN WFI. The RISC-V privileged specification lets WFI resume
     * when an interrupt is pending whatever mstatus.MIE says, and the
     * Cortex-M targets lean on the same property to sleep first and unmask
     * after. The QingKe V4 manual (6.2) wakes a WFI on "the interrupt
     * source responded by the interrupt controller", which does not
     * promise it, and a bare WFI is not what this hook executes, so the
     * question is left unasked here.
     *
     * The instruction is the core's WFE, executed with MIE SET.
     * PFIC_SCTLR.WFITOWFE makes `wfi` a wait-for-EVENT and SEVONPEND makes
     * every interrupt entering the pending state an event, LATCHED: "if
     * the WFE instruction is not executed, the system will be woken up
     * immediately after the next execution of the instruction" (RM
     * 7.5.2.38). The order is store, unmask, `wfi`, and each interleaving
     * with an interrupt is covered:
     *  - pending between the caller's check and the unmask: taken at the
     *    unmask, its pending edge latched - the WFE returns at once;
     *  - pending between the unmask and the `wfi`: taken there, latched
     *    the same way - the WFE returns and the caller's loop finds what
     *    the handler posted;
     *  - pending while asleep, or on the cycle the WFE goes to sleep: an
     *    ENABLED interrupt with MIE set wakes a WFE by V4 manual 6.2's
     *    second item, "woken up when an interrupt is generated, and after
     *    waking up, the microprocessor executes the interrupt function
     *    first" - a level, whatever the edge's timing.
     * With MIE clear, the only wake an interrupt gives a WFE is 6.2's third
     * item, SEVONPEND's "new interrupt pending signal" - an edge, and the
     * latch is all that stands between it and a sleep that never ends. The
     * QingKe V2 loses such an edge on the cycle its WFE goes to sleep
     * (docs/ch32v00x/platform.md); the unmasked order needs no edge at all.
     *
     * The store clears SLEEPONEXIT in the same instruction: with it set,
     * the waking handler's mret would put the core back to sleep (6.2) and
     * the caller's loop would not see what the handler posted. Nothing in
     * brio sets it; the idle path does not rely on that.
     *
     * ONE TURN PER WAKE: THE LATCH IS CONSUMED AFTER IT. The edge that wakes
     * the WFE is an interrupt entering the pending state, so SEVONPEND
     * latches it too, and left there it ends the NEXT idle() at once: the
     * caller's loop turns twice per interrupt, the second turn finding
     * nothing (measured on the CH32X035F8U6: 200 kernel turns over 100
     * quiet ticks, docs/ch32x035/platform.md). So after the wake the hook
     * writes SETEVENT - "set the event to wake up the WFE case" (RM
     * 7.5.2.38) - and executes one more `wfi`, which that event ends at
     * once and which leaves the latch clear whatever it held. MIE is SET
     * by then, so an enabled interrupt pending around the consume is taken
     * by its level (6.2's second item) and its handler runs before the
     * caller's next masked check; what the consume can eat is a latch whose
     * interrupt has been or is being taken, or the edge of a line the PFIC
     * does not enable, which wakes nothing the kernel waits for. It comes
     * AFTER the sleep, where WCH's `__WFE()` puts its SETEVENT and first
     * `wfi` BEFORE it: with the unmask first, an interrupt taken between
     * the unmask and a consume placed before the sleep would have its
     * latch eaten and its post found only at the next interrupt.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this is
     * the plain Sleep of RM 2.3.2 - the core clock stopped, every
     * peripheral clock running. No verb of this stratum sets SLEEPDEEP; a
     * program that does has armed a Stop or a Standby (RM 2.3.3, 2.3.4),
     * and the path is then idle_deep(), out of line.
     */
    static void idle() {
        const uint32_t sctlr = pfic_sctlr();
        if ((sctlr & sctlr_sleepdeep) != 0u) {
            idle_deep(sctlr);
            return;
        }
        const uint32_t wfe = (sctlr | sctlr_wfitowfe | sctlr_sevonpend) &
                             ~(sctlr_setevent | sctlr_sleeponexit);
        pfic_sctlr() = wfe;
        enable_interrupts();
        __asm__ volatile("wfi" ::: "memory");
        // The latch the wake left, consumed: SETEVENT makes the next WFE
        // return at once whatever the latch held, and that WFE clears it.
        pfic_sctlr() = wfe | sctlr_setevent;
        __asm__ volatile("wfi" ::: "memory");
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
    /**
     * idle() with SLEEPDEEP armed: the timebase paused and its pending bit
     * cleared first - the STK rides HCLK, which a Stop stops, and under
     * SEVONPEND a tick that is merely PENDING would end the sleep before it
     * began - then the same store, unmask and `wfi`, and the timebase
     * resumed after the wake. It does not consume the latch: a `wfi` with
     * SLEEPDEEP armed is a second entry into that mode.
     */
    [[gnu::noinline]] static void idle_deep(uint32_t sctlr) {
        if constexpr (pauses_tick) {
            TB::pause();
        }
        stk()->SR = 0;
        Pfic::clear_pending(Irq::systick);
        pfic_sctlr() = (sctlr | sctlr_wfitowfe | sctlr_sevonpend) &
                       ~(sctlr_setevent | sctlr_sleeponexit);
        enable_interrupts();
        __asm__ volatile("wfi" ::: "memory");
        if constexpr (pauses_tick) {
            TB::resume();
        }
    }

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
