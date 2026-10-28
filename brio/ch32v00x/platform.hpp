/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on a CH32V00x: the
 * realization of the kernel's Platform concept (the compiler-checked
 * set of requirements kernel/platform.hpp states) on the QingKe V2C of
 * the CH32V006 and the QingKe V2A of the CH32V003, both RV32EC. A
 * program names it once, as the first argument of its Tenuto, and every
 * kernel template is instantiated with it. The smallest use is the glue
 * every program of this project writes (docs/ch32v00x/platform.md, "How
 * to use it"):
 *
 *     using P = brio::Ch32v00xPlatform<>;
 *
 *     BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }
 *
 *     int main() {
 *         const auto record = brio::take_panic_record<P>();   // first, and once
 *         SysClock::init();
 *         brio::Ticker::init(clock);
 *         brio::enable_interrupts();
 *         brio::Tenuto<P, ...>::run();
 *     }
 *
 * The empty angle brackets take the default timebase, the STK Ticker of
 * ch32v00x/ticker.hpp (1000 Hz); the `...` stands for the program's
 * active objects in priority order (an active object is a component of
 * a brio program with its own event queue, served one event at a time
 * by the kernel). The kernel binds no vector: a program that forgets
 * the systick_handler binding compiles, but the crt points that vector
 * at its spin for unbound vectors, Ticker::init() enables the line, and
 * the first tick after enable_interrupts() enters the spin and the
 * program stops there.
 * BRIO_CH32_VECTOR is the one spelling of a vector binding on this
 * target (ch32v00x/pfic.hpp). fault_reset<P>() is the fault
 * vector's body (ch32v00x/reset.hpp).
 *
 * Who calls what. The kernel loop calls idle() in main-loop context,
 * with interrupts already masked, and nothing else should; it returns
 * with interrupts enabled. now() and ticks_per_second may be read from
 * any context. CriticalSection may be opened from any context and
 * nests. break_here() is the panic path's, and panic_record() the panic
 * path's and the next boot's (take_panic_record<P>() in kernel/panic.hpp
 * reads and clears it). Nothing here waits except the sleep itself, and
 * the only interrupts masked are the critical section's and, on the
 * CH32V003, idle()'s own sleep entry (below).
 *
 * The register file is half the usual one: RV32E has sixteen registers
 * instead of thirty-two, so everything the kernel does here it does with
 * half the register file, which is why this target is worth having: it
 * prices the abstractions honestly. The CH32V003 is the smallest machine
 * brio runs on (16 KB of flash, 2 KB of SRAM, no multiplier).
 *
 * CriticalSection is ch32v00x/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * The masking is all-or-nothing, one bit; this core has no priority
 * threshold that generic code could use.
 *
 * What is not here, on purpose: no idle_until(). This platform is not
 * tickless (a tickless timebase keeps counting while the core sleeps and
 * can place a wake at the next deadline); its timebase is the core
 * counter and stops with the core. The sleep sites are
 * ch32v00x/sleep.hpp's; which reset happened, the fault body and the
 * watchdogs are ch32v00x/reset.hpp's.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/exti.hpp"
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
     * The kernel's idle: entered by the loop, in main-loop context, with
     * interrupts masked and nothing to do; sleeps until the next
     * interrupt and returns with interrupts enabled, having let that
     * interrupt's handler run. Not for a program to call. On the
     * CH32V006 it unmasks, then sleeps; on the CH32V003 it sleeps masked
     * from SRAM (the CH32V003, below).
     *
     * Not a plain WFI, and this is the core's own rule. The RISC-V
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
     * So the instruction is the core's WFE, and it is executed with MIE
     * set. PFIC_SCTLR.WFITOWFE makes wfi a wait-for-event, and SEVONPEND
     * makes every interrupt entering the pending state an event,
     * latched: a WFE executed after the event returns at once (3.1's
     * SCTLR: "if the WFE instruction is not executed, the system will
     * be woken up immediately after the next execution"). The order is
     * store, unmask, wfi, and each interleaving with an interrupt is
     * covered:
     *  - pending between the caller's check and the unmask: taken at
     *    the unmask, and its pending edge latched - the WFE returns;
     *  - pending between the unmask and the wfi: taken there, latched
     *    the same way - the WFE returns, the caller's loop turns and
     *    finds what the handler posted;
     *  - pending while asleep, or on the cycle the WFE goes to sleep:
     *    an enabled interrupt with MIE set is a wake by 5.2's WFE item
     *    (2), "woken up when an interrupt is generated", and the handler
     *    runs before the instruction after the wfi.
     * Measured on the CH32V006 with the STK's edge placed to the cycle
     * across the whole sequence and the window between the unmask and
     * the wfi widened by eight nops: no position loses the wake.
     *
     * Why not sleep masked. With MIE clear the only wake left is 5.2's
     * item (3), SEVONPEND's "new interrupt pending signal" - an edge -
     * and the edge has a hole: a pending edge arriving on the very
     * cycle the WFE goes to sleep is neither latched for that WFE nor
     * seen by the sleeping core. Measured: exactly one cycle in each
     * sleep entry, every layout, with the vendor's doubled wfi too.
     * The pending bit then stands, the STK's CNTIF with it, so no new
     * edge ever comes: the core sleeps for ever with the tick pending
     * and enabled. A kernel loop entering sleep at a phase that walks
     * against the tick's (anything asynchronous to it) meets that cycle
     * sooner or later. The hole stays for a line the PFIC does not
     * enable (a SEVONPEND-only wake, measured on TIM2's update): such a
     * wake waits for its line's next edge.
     *
     * WFITOWFE and SEVONPEND read back set after a wfi (measured: the
     * manual's "the subsequent WFI" is not one-shot here), so the store
     * is not a re-arm; it stays because a WFE is what makes the unmask
     * first safe, and this hook does not trust that nothing else wrote
     * the register.
     *
     * One turn per wake: the latch is consumed after it. The edge that
     * wakes the WFE is an interrupt entering the pending state, so
     * SEVONPEND latches it too, and left there it would end the next
     * idle() at once: the caller's loop would turn twice per interrupt,
     * the second turn finding nothing (measured: 200 kernel turns over
     * 100 quiet ticks, a wasted turn of a three-AO pack 287 cycles).
     * So after the wake the hook writes SETEVENT - 3.1's "set the event
     * to wake up the WFE case" - and executes one more wfi, which that
     * event ends at once and which clears the latch whatever it held.
     * This is safe because MIE is set by then: an enabled interrupt
     * pending around the consume is taken by level, its handler run
     * before the caller's next masked check - it is not the latch that
     * lets the caller see it. What the consume can eat is a latch whose
     * interrupt has been or is being taken, or the edge of a line the
     * PFIC does not enable, which wakes nothing the kernel waits for (a
     * sleep site that wakes through such a line enables it instead:
     * ch32v00x/sleep.hpp). Measured with the edge placed to the cycle
     * across the whole sequence, the consume included, in twelve code
     * layouts: 101 turns over 100 ticks, nothing lost and no wake a
     * tick late. The consume is three instructions, 6 to 15 cycles a
     * wake by layout, against the 287 of the turn it saves.
     *
     * What depth. SLEEPDEEP as found: out of reset it is clear, so this
     * is the plain Sleep of QingKe 5.1 (the core clock gated, STK and
     * the wake logic alive), and with ch32v00x/sleep.hpp's site having
     * armed a Standby the same instruction is that Standby. The hook
     * takes what it finds, with one thing it does for a Standby,
     * because only this hook can: the tick is held off across it. The
     * STK fires every millisecond, and a tick turning pending is an
     * event that ends the WFE before the Standby begins (the SAM C21's
     * SysTick has the same power over its sleep, and its idle() holds
     * it off the same way). So with SLEEPDEEP armed the timebase is paused
     * and its pending bit cleared before the sleep, and resumed after;
     * kernel time stands still for exactly the slept span, which the
     * timed site hands back. A stale latched event (a USART interrupt
     * from just before) still ends the first WFE at once: the kernel
     * loop simply turns and sleeps again, which is why a program that
     * wants a Standby lets the loop idle and does not call this once.
     * The Standby is the cold path, out of line, so the Sleep the
     * kernel loop takes every turn is a leaf: one load, one store, the
     * unmask and the wfi, then the consume's store and wfi. The Standby
     * path does not consume: a wfi with SLEEPDEEP armed is a Standby
     * entry, and the one stale turn after a Standby is worth less than
     * a second pass through that door.
     */
    static void idle() {
        const uint32_t sctlr = pfic_sctlr();
        if ((sctlr & sctlr_sleepdeep) != 0u) {
            idle_deep(sctlr);
            return;
        }
        const uint32_t wfe = (sctlr | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
        if constexpr (device::sleep_entry_from_sram) {
            // The QingKe V2A (the CH32V003), whose sleep entry has a
            // wedge the V2C has not: see "The CH32V003" below. Masked
            // throughout, fetched from SRAM, the unmask back here - and
            // no sleep at all while a DMA channel works, which the V2A's
            // Sleep would stall (parts/ch32v003.hpp).
            // The latch emptied first: SETEVENT and a wfi it ends at
            // once.
            pfic_sctlr() = wfe | sctlr_setevent;
            wfi_from_sram();
            // What the emptied latch may have held, asked of the PFIC:
            // an enabled line pending means no sleep - the unmask below
            // takes it. From here on an edge latches for the sleep's wfi.
            // No sleep either while a DMA channel works, or when the
            // tick's match is too close to keep its edge off the entry.
            const PficRegs* const p = pfic();
            if (((p->IPR[0] & p->ISR[0]) | (p->IPR[1] & p->ISR[1])) == 0u &&
                !(device::sleep_stops_bus_masters && Dma::any_working()) &&
                TB::cycles_to_tick() >= sram_sleep_tick_guard_cycles) {
                pfic_sctlr() = wfe;
                wfi_from_sram();
            }
            enable_interrupts();
        } else {
            pfic_sctlr() = wfe;
            enable_interrupts();
            __asm__ volatile("wfi" ::: "memory");
            // The latch the wake left, consumed: SETEVENT makes the next
            // WFE return at once whatever the latch held, and that WFE
            // clears it.
            pfic_sctlr() = wfe | sctlr_setevent;
            __asm__ volatile("wfi" ::: "memory");
        }
    }

    /**
     * The CH32V003 (QingKe V2A) sleeps another way, because its sleep
     * entry wedges. Measured with the STK's edge placed to the cycle
     * across the sleep, an IWDG fed by the tick catching each hang: an
     * edge landing in a window four cycles wide at the wfi's entry
     * leaves the core in a state nothing ends - not that interrupt, not
     * a later line's edge, not a debugger's halt (with DBGMCU_CR.SLEEP
     * keeping HCLK on as well) - only a reset. The same window with the
     * V2C's sequence above, without its consume, with the wfi fetched
     * from SRAM, with the hardware prologue off, and in the masked order
     * fetched from flash (with a spin after the wfi or not). What ends
     * it is the shape of the vendor's own CH32V003 WFE (its EVT's
     * core_riscv.h, read as the oracle; the CH32V006's EVT has none of
     * it): the wfi run from SRAM with interrupts masked and a spin of 19
     * turns in SRAM after it - so that no vector and no instruction is
     * fetched from flash in the first cycles after the wake. In that
     * shape the window wedges nothing in 1600 tries (a spin of 1 turn
     * still wedged); one cycle of it loses the edge instead, the masked
     * WFE's own hole - 5.2's item (3) is an edge, and the edge on the
     * entry cycle is not seen (the CH32V006 measured the same of its
     * masked order).
     *
     * The masked order moves the consume: an interrupt is taken only at
     * the unmask after the sleep, and a latch emptied before that comes
     * back (measured: 200 turns over 100 ticks with the V2C's consume
     * after the wake). So the latch is emptied first - SETEVENT and a
     * wfi it ends at once, 5 cycles - and the PFIC is asked what it may
     * have held: an enabled line pending means no sleep, the caller's
     * unmask takes it. An edge after that question latches for the
     * sleep's own wfi, which then returns at once. And the tick's edge
     * is kept off the entry: if its match is closer than
     * sram_sleep_tick_guard_cycles there is no sleep either, and the
     * caller's loop turns until the tick is in. What stays: the edge of
     * another line that lands on the entry cycle itself is late - it
     * stands pending and masked, and the next wake (the tick's, at the
     * latest) lets the unmask take it.
     *
     * A V2A Sleep is not a light one: once entered it lasts at least
     * about 3150 cycles at 48 MHz (66 us) - an edge that comes sooner
     * after the entry runs the core again only then (measured with the
     * edge 100 to 2400 cycles after the entry, the sum constant to five
     * cycles) - so an interrupt that finds the core freshly asleep is
     * served up to 66 us late, where the V2C's is served in about 25
     * cycles.
     */
    static constexpr uint32_t sram_wake_spin_turns = 19;

    /// The tick guard. From the counter's read to the wfi are eleven
    /// instructions (the subtraction and compare, the SCTLR store, the
    /// call into SRAM), about thirty cycles at 48 MHz behind the flash's
    /// two wait states and fewer below; with the four-cycle window the
    /// guard is three times that. What it costs is the caller's loop
    /// turning instead of sleeping for at most this many cycles, once a
    /// tick period.
    static constexpr uint32_t sram_sleep_tick_guard_cycles = 96;

    /// mstatus.MIE readback: the one bit CriticalSection saves.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Whether the caller runs inside an interrupt handler: PFIC_GISR's
    /// GACTSTA (ch32v00x/pfic.hpp's Pfic::in_handler(), which says why not
    /// mstatus.MIE; the optional member of kernel/platform.hpp).
    static bool in_handler() { return Pfic::in_handler(); }

    /**
     * Halt in the debugger.
     *
     * Caveat. With a debugger attached, the program stops here.
     * With no debugger attached, `ebreak` raises the breakpoint
     * exception, which this core takes to the vector table's fault
     * entry - the crt provides that as a distinct spin loop, so the
     * wreck is legible instead of running on into nothing. A reporter
     * that must survive without a debugger cannot assume this call
     * returns.
     */
    static void break_here() { __asm__ volatile("ebreak"); }

    /// Kernel time: the timebase's tick count, readable from any context.
    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on the core counter.
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible
    /// access, which is what lets util/ring.hpp take its lock-free path.
    static constexpr unsigned atomic_width = 4;

    /// This family places `.fast_text` in SRAM, on both parts: the two
    /// linker scripts collect it first in .data, which the crt copies
    /// before main(), and the vector table holds absolute addresses, so
    /// code there is entered and called without a veneer
    /// (docs/ch32v00x/platform.md, "A handler in SRAM"). A driver of this
    /// stratum may therefore be given util/placement.hpp's InRam - an
    /// opt-in the program spends its SRAM on, 8 KB on the CH32V006 and
    /// 2 KB on the CH32V003, and a link that does not fit is refused by
    /// the scripts' `ram` region.
    static constexpr bool places_fast_text = true;

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
    /// The CH32V003's wfi ("The CH32V003", above): executed masked from
    /// SRAM with the spin after it, so that nothing is fetched from flash
    /// in the first cycles after a wake - twelve bytes of RAM. Code that
    /// must run from SRAM, so `.ram_text` (util/placement.hpp's two
    /// names), which ld/ch32v003f4.ld carries in .data - as
    /// `.ram_text.brio`, never cloned: a member of a class template is
    /// emitted in a COMDAT group, and GCC refuses one section name for a
    /// grouped function and a program's plain one in the same translation
    /// unit ("section type conflict", WCH's gcc 15.2), so the brio name is
    /// for header functions and a program's own names `.ram_text`.
    [[gnu::section(".ram_text.brio"), gnu::noinline, gnu::noclone]] static void wfi_from_sram() {
        uint32_t n = sram_wake_spin_turns;
        __asm__ volatile("wfi\n1:\taddi %0, %0, -1\n\tbnez %0, 1b" : "+r"(n)::"memory");
    }

    /// idle() with SLEEPDEEP armed: the tick held off across the
    /// Standby (idle()'s comment says why), then the same store, unmask
    /// and wfi - on the CH32V003 the masked SRAM entry instead, its
    /// EXTI lines that interrupt made events for the Standby's span
    /// (the sleep entry's wedge, "The CH32V003" above: masked, a line's
    /// interrupt is no wake, and an EXTI event is one - 5.2's item (1);
    /// the vendor's own deep WFE does the same).
    [[gnu::noinline]] static void idle_deep(uint32_t sctlr) {
        TB::pause();
        stk()->SR = 0;
        Pfic::clear_pending(Irq::systick);
        const uint32_t wfe = (sctlr | sctlr_wfitowfe | sctlr_sevonpend) & ~sctlr_setevent;
        if constexpr (device::sleep_entry_from_sram) {
            const uint32_t events = exti()->EVENR;
            exti()->EVENR = events | exti()->INTENR;
            pfic_sctlr() = wfe;
            wfi_from_sram();
            exti()->EVENR = events;
            enable_interrupts();
        } else {
            pfic_sctlr() = wfe;
            enable_interrupts();
            __asm__ volatile("wfi" ::: "memory");
        }
        TB::resume();
    }

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

static_assert(Platform<Ch32v00xPlatform<>> && HandlerAware<Ch32v00xPlatform<>>);

} // namespace brio
