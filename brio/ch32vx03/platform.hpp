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
     * Entered with interrupts MASKED and nothing to do: unmask, then
     * sleep until the next interrupt.
     *
     * NOT A PLAIN WFI. The RISC-V privileged specification lets WFI
     * resume when an interrupt is pending whatever mstatus.MIE says, and
     * the Cortex-M targets lean on the same property to sleep first and
     * unmask after. This core does NOT keep that promise: QingKe V4
     * manual 6.2 wakes a WFI on "the interrupt source responded by the
     * interrupt controller", and measured on the CH32V203C8, a `wfi`
     * with MIE clear slept past the STK and TIM2 both pending and
     * enabled until a debugger halted it.
     *
     * So the instruction is the core's WFE, executed with MIE SET.
     * PFIC_SCTLR.WFITOWFE makes wfi a wait-for-EVENT, and SEVONPEND
     * makes every interrupt entering the pending state an event,
     * LATCHED: a WFE executed after the event returns at once (the
     * manual's SCTLR table: "if the WFE instruction is not executed,
     * the system will be woken up immediately after the next
     * execution"). The order is store, unmask, wfi, and each
     * interleaving with an interrupt is covered:
     *  - pending between the caller's check and the unmask: taken at
     *    the unmask, and its pending edge latched - the WFE returns;
     *  - pending between the unmask and the wfi: taken there, latched
     *    the same way - the WFE returns, the caller's loop turns and
     *    finds what the handler posted;
     *  - pending while asleep, or on the cycle the WFE goes to sleep:
     *    an ENABLED interrupt with MIE set is a wake by 6.2's WFE item
     *    (2), "woken up when an interrupt is generated, and after
     *    waking up, the microprocessor executes the interrupt function
     *    first" - a level, whatever the edge's timing.
     * Measured on the CH32V203C8 with the STK's edge placed to the cycle
     * across the whole sequence, in twelve code layouts and with the
     * window between the unmask and the wfi widened by eight nops: no
     * position loses the wake.
     *
     * WHY NOT SLEEP MASKED. With MIE clear the only wake an interrupt
     * gives a WFE is 6.2's item (3), SEVONPEND's "NEW interrupt pending
     * signal (the previously generated pending signal does not take
     * effect)" - an EDGE, and the latch is all that stands between a
     * pending edge and a sleep that never ends. The QingKe V2 core of
     * the CH32V00x, whose manual carries the same words, loses an edge
     * arriving on the very cycle its WFE goes to sleep (measured there:
     * one cycle per sleep entry, the core then asleep for ever with the
     * tick pending and enabled). The same instrument found no such
     * cycle on this core in twenty-four layouts - but that is a fact
     * of one die's sleep entry and not of the manual, and the unmasked
     * order needs no edge at all. It is also the cheaper wake: the
     * handler runs straight out of the sleep instead of after an
     * unmask (measured: docs/ch32vx03/platform.md).
     *
     * The store also clears SLEEPONEXIT, in the same instruction: with
     * it set, the waking handler's mret would put the core back to
     * sleep (6.2) and the caller's loop would never see what the
     * handler posted. Nothing in brio sets it; the idle path does not
     * rely on that. WFITOWFE stays set after a wfi (measured: a wfi
     * with no store before it still waits for an event), so the store
     * is not a re-arm; it stays because a WFE is what makes the unmask
     * first safe, and this hook does not trust that nothing else wrote
     * the register - WCH's own `__WFI()` clears WFITOWFE.
     *
     * ONE TURN PER WAKE: THE LATCH IS CONSUMED AFTER IT. The edge that
     * wakes the WFE is an interrupt entering the pending state, so
     * SEVONPEND latches it too, and left there it would end the NEXT
     * idle() at once: the caller's loop would turn twice per interrupt,
     * the second turn finding nothing (measured on the CH32V203C8: 200
     * kernel turns over 100 quiet ticks, a wasted turn of a three-AO
     * pack 141 cycles). So after the wake the hook writes SETEVENT - the
     * SCTLR table's (QingKe V4 manual 3.1) "set the event to wake up the
     * WFE case" - and executes one more wfi, which that event ends at
     * once and which clears the latch whatever it held. This is safe
     * because MIE is SET by then: an enabled interrupt pending around the
     * consume is taken by its level (6.2's WFE item 2), its handler run
     * before the caller's next masked check - it is not the latch that
     * lets the caller see it. What the consume can eat is a latch whose
     * interrupt has been or is being taken, or the edge of a line the
     * PFIC does not enable, which wakes nothing the kernel waits for. It
     * comes AFTER the sleep and not before it, where WCH's `__WFE()`
     * puts its SETEVENT and first `wfi`: with the unmask first, an
     * interrupt taken between the unmask and a consume placed before the
     * sleep would have its latch eaten and its post found only at the
     * next interrupt. Measured with the edge placed to the cycle across
     * the whole sequence, the consume included: 101 turns over 100
     * ticks, nothing lost and no wake a tick late. The consume is three
     * instructions - an `ori`, the store, the `wfi` - and 5 to 7 cycles
     * a wake (docs/ch32vx03/platform.md), against the 141 of the turn it
     * saves.
     *
     * AND IT SLEEPS ONLY WHILE THE CORE OWNS THE BUS. In a sleep of any
     * depth on this family no other master gets a cycle, so a DMA
     * channel or the USB controller that is working would not be slowed
     * by the sleep but broken by it (ch32vx03/bus_activity.hpp). With
     * masters active this hook therefore returns AT ONCE, interrupts
     * enabled, and the loop spins - a legal idle() under the Platform
     * contract, and the honest one here, which turns the loop as often as
     * it can for as long as a master works: one turn per wake is the
     * promise of the sleep, not of the spin.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this
     * is the plain Sleep of RM 2.3.2 - the core clock gated, the
     * counter and the wake logic alive. With a DEEP mode armed by a
     * sleep site (ch32vx03/sleep.hpp) the path is idle_deep(), out of
     * line: the timebase is paused and its pending bit cleared first,
     * because the STK stops with HCLK in a Stop and because SEVONPEND
     * makes a tick that is merely PENDING end the sleep before it
     * begins. The deep path does not consume the latch: a `wfi` with
     * SLEEPDEEP armed is a second entry into that mode, and the one stale
     * turn after a deep sleep is worth less than a second pass through
     * that door.
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
    /**
     * idle() with SLEEPDEEP armed: the tick held off across the deep
     * sleep (idle()'s comment says why), then the same store, unmask
     * and wfi. The handler that ends the sleep runs first (6.2's WFE
     * item 2), with the timebase still paused and on the clock the wake
     * left; the timed site's alarm body needs neither the tick's
     * interrupt nor the program's clock to begin, and puts the clock
     * back as its first act (ch32vx03/sleep.hpp).
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
