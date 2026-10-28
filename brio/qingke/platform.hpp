/*
 * platform.hpp - the CORE stratum: the kernel's platform on a QingKe core.
 *
 * What the brio kernel stands on when it runs on a QingKe core: the
 * realization of the kernel's Platform concept (the compiler-checked set
 * of requirements kernel/platform.hpp states), written once for the
 * CH32V00x (the QingKe V2C of the CH32V006, the V2A of the CH32V003,
 * both RV32EC), the CH32V203 and the CH32V303 (the V4B, RV32IMAC, and the
 * V4F, the same core with a single-precision floating-point unit,
 * RV32IMAFC) and the CH32X035 (the V4C, RV32IMAC: the V4B with a faster
 * divider and the physical memory protection unit, the reference
 * manual's core table). It gives the kernel a critical section
 * (interrupts masked for a scope), an idle that sleeps until the next
 * interrupt without losing one that arrives at the wrong moment, the
 * program's source of time (the template parameter, by default the STK
 * `Ticker` at 1000 Hz, qingke/ticker.hpp), a breakpoint for a panic and
 * the storage a panic record survives a reset in.
 *
 * A program never names this file's type: it names its family's
 * platform - `Ch32v00xPlatform<>`, `Ch32vx03Platform<>`,
 * `Ch32x035Platform<>`, each a struct over QingkePlatform<Chip, TB> in
 * the family's platform.hpp - once, as the first argument of its Tenuto:
 *
 *     using P = brio::Ch32x035Platform<>;
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }   // where the family has reset.hpp
 *
 *     int main() {
 *         const auto record = brio::take_panic_record<P>();   // first, and once
 *         SysClock::init();
 *         brio::Ticker::init(clock);
 *         brio::enable_interrupts();
 *         brio::Tenuto<P, ...>::run();   // the program's active objects
 *     }
 *
 * The empty angle brackets take the default timebase; the `...` stands
 * for the program's active objects in priority order (an active object
 * is a component of a brio program with its own event queue, served one
 * event at a time by the kernel). The kernel binds no vector: a program
 * that forgets the systick_handler binding compiles, but the crt points
 * that vector at its spin for unbound vectors, Ticker::init() enables the
 * line, and the first tick after enable_interrupts() enters the spin and
 * the program stops there. fault_reset<P>() is the fault vector's body on
 * the CH32V00x and the CH32V203/CH32V303 (their reset.hpp).
 *
 * WHAT THE FAMILY SUPPLIES. A chip traits type, the first template
 * argument, whose members are the facts this file cannot know - the
 * `QingkeChip` concept below checks them:
 *  - `bus_masters_working()`, the question idle() asks before it sleeps:
 *    whether a bus master other than the core is working that a sleep
 *    would stall - the CH32V203's and the CH32V303's count of active DMA
 *    channels and USB controllers (ch32vx03/bus_activity.hpp), the
 *    CH32X035's DMA controller's own registers, the CH32V003's DMA
 *    controller, and on the CH32V006, whose Sleep the DMA works through,
 *    a constant false;
 *  - `sleep_entry_from_sram`, where the core's sleep entry must run
 *    masked from SRAM (the CH32V003's QingKe V2A, below), with the two
 *    verbs that make a Standby's wake sources events; absent, the entry
 *    is the ordinary one. A member left out is false rather than a
 *    member every family must spell, because the one part that has it
 *    is the one part whose entry differs;
 *  - `idle_leaves_sleeponexit`, where the idle store leaves SLEEPONEXIT
 *    as it finds it instead of clearing it: the CH32V00x's (idle(),
 *    below); absent, the store clears it, as the two V4 families' does.
 * The family's own platform struct adds what is its alone:
 * `places_fast_text` where its linker scripts place `.fast_text`, the
 * CH32V203's `bus_masters_active()`.
 *
 * Who calls what. The kernel loop calls idle() in main-loop context, with
 * interrupts already masked and nothing to do, and nothing else should;
 * idle() returns with interrupts enabled, after the sleep or at once
 * while a bus master works. now(), ticks_per_second and
 * interrupts_enabled() may be read from any context; CriticalSection may
 * be opened from any context and nests; break_here() is the panic path's,
 * and panic_record() the panic path's and the next boot's
 * (take_panic_record<P>() in kernel/panic.hpp reads and clears it);
 * stack_untouched() is a main-context read of memory. Nothing here waits
 * except the sleep itself, and the only interrupts masked are the
 * critical section's and, on the CH32V003, idle()'s own sleep entry.
 *
 * The register file differs and the contract above it does not: the
 * QingKe V2's RV32E has sixteen registers instead of thirty-two, which is
 * why the CH32V00x is worth having - it prices the abstractions honestly,
 * and the CH32V003 is the smallest machine brio runs on (16 KB of flash,
 * 2 KB of SRAM, no multiplier); the V4 has thirty-two, a divider, an
 * atomic extension, user mode and a 64-bit system counter, up to 144 MHz
 * on the CH32V203 and the CH32V303 and 48 MHz on the CH32X035. The kernel
 * and util strata compile on all of them untouched, which is the fact
 * these targets exist to keep true.
 *
 * CriticalSection is qingke/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * All-or-nothing masking. The V4 cores do have a priority threshold and
 * interrupt nesting (two levels on the V4B and the V4C), and under Tenuto
 * this platform uses neither: an ISR body runs to completion here as
 * everywhere (docs/design/kernel.md section 1, "The model in one page").
 * The preemptive kernel's platforms (each V4 family's RubatoPlatform)
 * turn nesting on for one purpose alone - the scheduling request one
 * level below every peripheral line, which still share one level and
 * never nest among themselves (qingke/schedule.hpp).
 *
 * Not here, and deliberate rather than forgotten: no idle_until(). This
 * platform is not tickless (a tickless timebase keeps counting while the
 * core sleeps and can place a wake at the next deadline); its timebase is
 * the core counter, which keeps counting in the plain Sleep and stops in
 * a deeper one (qingke/ticker.hpp). The sleep sites are the families'
 * sleep.hpp (the CH32V00x's and the CH32V203's); which reset happened,
 * the fault body and the watchdogs are their reset.hpp.
 *
 * Include-order contract: the family's device header, pfic.hpp and
 * ticker.hpp first (and on a V4 family its schedule.hpp), then the chip
 * traits, which the family's platform.hpp defines before this file.
 */

#pragma once

#include <stdint.h>

#include <concepts>

#include "kernel/platform.hpp"
#include "qingke/pfic.hpp"
#include "qingke/ticker.hpp"

namespace brio {

/// What a family's chip traits type gives the platform (the file header).
/// The CH32V003's sleep entry from SRAM brings two verbs with it: the
/// Standby's wake sources made events for the span of the masked sleep,
/// and given back after it.
template <class C>
concept QingkeChip =
    requires {
        { C::bus_masters_working() } -> std::same_as<bool>;
    } &&
    (!requires { requires C::sleep_entry_from_sram; } ||
     requires(uint32_t found) {
         { C::sram_deep_events_on() } -> std::same_as<uint32_t>;
         C::sram_deep_events_off(found);
     });

template <class Chip, class TB>
struct QingkePlatform {
    static_assert(QingkeChip<Chip>,
                  "brio QingkePlatform: the chip traits give bus_masters_working() -> bool, and with "
                  "sleep_entry_from_sram also sram_deep_events_on() -> uint32_t and "
                  "sram_deep_events_off(uint32_t) (qingke/platform.hpp)");

    using CriticalSection = InterruptGuard;

    /// The kernel timebase this program runs on.
    using Timebase = TB;

    /// Whether that timebase has a periodic interrupt to pause across a
    /// deep sleep. True of the STK ticker; a timebase that keeps time
    /// through a Stop or a Standby would not need it and would not offer
    /// the verbs.
    static constexpr bool pauses_tick = requires {
        TB::pause();
        TB::resume();
    };

    /// Whether the core's sleep entry runs masked from SRAM: the
    /// CH32V003's (idle(), "The CH32V003"), false where the chip traits
    /// leave the member out.
    static constexpr bool sleep_entry_from_sram = requires { requires Chip::sleep_entry_from_sram; };

    /// The SCTLR bits idle()'s store clears beside SETEVENT: SLEEPONEXIT,
    /// but where the chip traits say the store leaves it as found (the
    /// CH32V00x's, idle()).
    static constexpr uint32_t idle_clears =
        sctlr_setevent | (requires { requires Chip::idle_leaves_sleeponexit; } ? 0u : sctlr_sleeponexit);

    /**
     * The kernel's idle: entered by the loop, in main-loop context, with
     * interrupts masked and nothing to do; unmasks, then sleeps until the
     * next interrupt, and returns with interrupts enabled, having let that
     * interrupt's handler run. Not for a program to call. While a bus
     * master works that the sleep would stall it does not sleep at all:
     * it unmasks and returns at once. On the CH32V003 it sleeps masked
     * from SRAM instead ("The CH32V003", below).
     *
     * NOT A PLAIN WFI, and this is the cores' own rule. The RISC-V
     * privileged specification lets WFI resume when an interrupt is
     * pending whatever mstatus.MIE says, and the Cortex-M targets lean on
     * the same property to sleep first and unmask after. The QingKe cores
     * do not keep that promise: the V2 manual (5.2) and the V4 manual
     * (6.2) end a WFI sleep on "the interrupt source responded by the
     * interrupt controller", and nothing is responded while MIE is clear
     * - so `wfi` then `csrsi` sleeps past every pending interrupt for
     * ever. Measured on the CH32V006 (the tick and the USART both pending,
     * the core asleep with MIE clear, the console silent) and on the
     * CH32V203C8 (a `wfi` with MIE clear slept past the STK and TIM2 both
     * pending and enabled until a debugger halted it); on the CH32X035 the
     * manual's words do not promise it either, and a bare WFI is not what
     * this hook executes, so the question was left unasked there.
     *
     * So the instruction is the core's WFE, executed with MIE SET.
     * PFIC_SCTLR.WFITOWFE makes wfi a wait-for-EVENT, and SEVONPEND makes
     * every interrupt entering the pending state an event, LATCHED: a WFE
     * executed after the event returns at once (the SCTLR table, the V2
     * manual 3.1 and the V4 manual 3.1, and the CH32X035 reference manual
     * 7.5.2.38: "if the WFE instruction is not executed, the system will
     * be woken up immediately after the next execution of the
     * instruction"). The order is store, unmask, wfi, and each
     * interleaving with an interrupt is covered:
     *  - pending between the caller's check and the unmask: taken at the
     *    unmask, and its pending edge latched - the WFE returns;
     *  - pending between the unmask and the wfi: taken there, latched the
     *    same way - the WFE returns, the caller's loop turns and finds
     *    what the handler posted;
     *  - pending while asleep, or on the cycle the WFE goes to sleep: an
     *    ENABLED interrupt with MIE set is a wake by the WFE's item (2)
     *    (the V2 manual 5.2, the V4 manual 6.2), "woken up when an
     *    interrupt is generated, and after waking up, the microprocessor
     *    executes the interrupt function first" - a level, whatever the
     *    edge's timing, the handler run before the instruction after the
     *    wfi.
     * Measured with the STK's edge placed to the cycle across the whole
     * sequence and the window between the unmask and the wfi widened by
     * eight nops: no position loses the wake on the CH32V006, nor on the
     * CH32V203C8 in twelve code layouts, nor on the CH32X035F8U6 (no edge
     * position of 600, at 48 MHz and at 8 MHz:
     * docs/ch32x035/platform.md).
     *
     * WHY NOT SLEEP MASKED. With MIE clear the only wake an interrupt
     * gives a WFE is its item (3), SEVONPEND's "NEW interrupt pending
     * signal (the previously generated pending signal does not take
     * effect)" - an EDGE, and the latch is all that stands between a
     * pending edge and a sleep that never ends. The QingKe V2's edge has
     * a hole: a pending edge arriving on the very cycle the WFE goes to
     * sleep is neither latched for that WFE nor seen by the sleeping core
     * - measured on the CH32V006: exactly one cycle in each sleep entry,
     * every layout, with the vendor's doubled wfi too. The pending bit
     * then stands, the STK's CNTIF with it, so no new edge ever comes: the
     * core sleeps for ever with the tick pending and enabled, and a kernel
     * loop entering sleep at a phase that walks against the tick's
     * (anything asynchronous to it) meets that cycle sooner or later. The
     * hole stays for a line the PFIC does not enable (a SEVONPEND-only
     * wake, measured on the CH32V006's TIM2 update): such a wake waits for
     * its line's next edge. The same instrument found no such cycle on the
     * CH32V203C8 in twenty-four layouts - but that is a fact of one die's
     * sleep entry and not of the manual, and the unmasked order needs no
     * edge at all. It is also the cheaper wake: the handler runs straight
     * out of the sleep instead of after an unmask (measured:
     * docs/ch32vx03/platform.md).
     *
     * On the CH32V203, the CH32V303 and the CH32X035 the store also clears
     * SLEEPONEXIT, in the same instruction: with it set, the waking
     * handler's mret would put the core back to sleep (the V4 manual 6.2;
     * the V2 manual 5.2 says the same of the V2) and the caller's loop
     * would never see what the handler posted. Nothing in brio sets it;
     * that idle path does not rely on that. The CH32V00x's store leaves
     * the bit as it finds it (`Chip::idle_leaves_sleeponexit`), relying on
     * nothing in brio setting it: clearing it there too moves the
     * CH32V003's SRAM entry by an instruction (the first SCTLR value is no
     * longer one `ori` of what was read), and that entry's timing is what
     * "The CH32V003" below measured. WFITOWFE stays set after a wfi - measured on the CH32V006
     * (WFITOWFE and SEVONPEND read back set after a wfi: the manual's "the
     * subsequent WFI" is not one-shot there) and on the CH32V203 (a wfi
     * with no store before it still waits for an event) - so the store is
     * not a re-arm; it stays because a WFE is what makes the unmask first
     * safe, and this hook does not trust that nothing else wrote the
     * register - WCH's own `__WFI()` clears WFITOWFE.
     *
     * ONE TURN PER WAKE: THE LATCH IS CONSUMED AFTER IT. The edge that
     * wakes the WFE is an interrupt entering the pending state, so
     * SEVONPEND latches it too, and left there it would end the NEXT
     * idle() at once: the caller's loop would turn twice per interrupt,
     * the second turn finding nothing - measured: 200 kernel turns over
     * 100 quiet ticks on the CH32V006 (a wasted turn of a three-AO pack
     * 287 cycles), on the CH32V203C8 (141 cycles) and on the CH32X035F8U6
     * (docs/ch32x035/platform.md). So after the wake the hook writes
     * SETEVENT - the SCTLR table's "set the event to wake up the WFE
     * case" (the V2 and V4 manuals 3.1, the CH32X035 reference manual
     * 7.5.2.38) - and executes one more wfi, which that event ends at once
     * and which clears the latch whatever it held. This is safe because
     * MIE is SET by then: an enabled interrupt pending around the consume
     * is taken by its level (the WFE's item 2), its handler run before the
     * caller's next masked check - it is not the latch that lets the
     * caller see it. What the consume can eat is a latch whose interrupt
     * has been or is being taken, or the edge of a line the PFIC does not
     * enable, which wakes nothing the kernel waits for (a sleep site that
     * wakes through such a line enables it instead: ch32v00x/sleep.hpp).
     * It comes AFTER the sleep and not before it, where WCH's `__WFE()`
     * puts its SETEVENT and first `wfi`: with the unmask first, an
     * interrupt taken between the unmask and a consume placed before the
     * sleep would have its latch eaten and its post found only at the next
     * interrupt. Measured with the edge placed to the cycle across the
     * whole sequence, the consume included: 101 turns over 100 ticks,
     * nothing lost and no wake a tick late, on the CH32V006 in twelve code
     * layouts and on the CH32V203C8 (and one kernel turn per tick on the
     * CH32X035F8U6). The consume is three instructions - an `ori`, the
     * store, the `wfi` - 6 to 15 cycles a wake by layout on the CH32V006
     * and 5 to 7 on the CH32V203 (docs/ch32vx03/platform.md), against the
     * 287 and the 141 of the turn it saves.
     *
     * AND IT SLEEPS ONLY WHILE NO BUS MASTER IT WOULD STALL IS WORKING
     * (Chip::bus_masters_working()). In a sleep of any depth on the
     * CH32V203 and the CH32V303 no master but the core gets a cycle, so a
     * DMA channel or the USB controller that is working would not be
     * slowed by the sleep but broken by it (ch32vx03/bus_activity.hpp
     * carries the measurements and the reasoning). In the CH32X035's
     * Sleep (its reference manual 2.3.2) the DMA controller moves nothing
     * - measured on the CH32X035F8U6 (docs/ch32x035/dma.md): a 4096-item
     * copy started before a sleep had moved 68 items when the tick woke
     * the core 19 000 cycles later, and a block paced by a USART's
     * transmitter ran at a fiftieth of its pace. In the CH32V003's V2A
     * Sleep the DMA moves nothing either (parts/ch32v003.hpp); in the
     * CH32V006's V2C Sleep it works on, so there the question is a
     * constant and compiles to nothing. With such a master working this
     * hook returns AT ONCE, interrupts enabled, and the loop spins - a
     * legal idle() under the Platform contract (the host's idle() returns
     * at once too), and the honest one here, which turns the loop as often
     * as it can for as long as a master works: one turn per wake is the
     * promise of the sleep, not of the spin. A program with an engine at
     * work, a receive ring above all, so holds itself awake without having
     * to know that it does.
     *
     * WHAT DEPTH. SLEEPDEEP as found: out of reset it is clear, so this is
     * the plain Sleep - the core clock gated, the counter and the wake
     * logic alive (the QingKe V2 manual 5.1; the CH32V203's and the
     * CH32X035's reference manuals 2.3.2). With a deeper mode armed - the
     * CH32V00x's Standby by ch32v00x/sleep.hpp's site, a CH32V203's Stop
     * or Standby by ch32vx03/sleep.hpp's, a CH32X035 program's own
     * SLEEPDEEP for its reference manual's 2.3.3 and 2.3.4 (no verb of
     * that stratum sets it) - the path is idle_deep(), out of line, so the
     * Sleep the kernel loop takes every turn is a leaf: one load, one
     * store, the unmask and the wfi, then the consume's store and wfi. The
     * deep path holds the tick off, because only this hook can: the STK
     * fires every millisecond and stops with HCLK in a Stop, and under
     * SEVONPEND a tick that is merely PENDING is an event that ends the
     * WFE before the deep sleep begins (the SAM C21's SysTick has the same
     * power over its sleep, and its idle() holds it off the same way). So
     * with SLEEPDEEP armed the timebase is paused and its pending bit
     * cleared before the sleep, and resumed after; kernel time stands
     * still for exactly the slept span, which a timed site hands back. A
     * stale latched event (a USART interrupt from just before) still ends
     * the first WFE at once: the kernel loop simply turns and sleeps
     * again, which is why a program that wants a deep sleep lets the loop
     * idle and does not call this once. The deep path does not consume
     * the latch: a wfi with SLEEPDEEP armed is a second entry into that
     * mode, and the one stale turn after a deep sleep is worth less than a
     * second pass through that door.
     *
     * AND THAT COSTS ONE TICK, KNOWINGLY. A tick that had already fired
     * when the deep sleep begins is dropped instead of served, so kernel
     * time is one tick short of the wall. A TIMED site repairs it for free
     * - its witness measures the wall and subtracts the ticks the counter
     * itself served, so one that was not served is advanced instead - and
     * without one a deep sleep is legal only with no deadline armed, which
     * is the model's own restriction.
     */
    static void idle() {
        if constexpr (!sleep_entry_from_sram) {
            if (Chip::bus_masters_working()) {
                // A master at work: no sleep of any depth (above).
                enable_interrupts();
                return;
            }
        }
        const uint32_t sctlr = pfic_sctlr();
        if ((sctlr & sctlr_sleepdeep) != 0u) {
            idle_deep(sctlr);
            return;
        }
        const uint32_t wfe = (sctlr | sctlr_wfitowfe | sctlr_sevonpend) & ~idle_clears;
        if constexpr (sleep_entry_from_sram) {
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
                !Chip::bus_masters_working() &&
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
     * edge landing in a window four cycles wide at the wfi's entry leaves
     * the core in a state nothing ends - not that interrupt, not a later
     * line's edge, not a debugger's halt (with DBGMCU_CR.SLEEP keeping
     * HCLK on as well) - only a reset. The same window with the V2C's
     * sequence above, without its consume, with the wfi fetched from
     * SRAM, with the hardware prologue off, and in the masked order
     * fetched from flash (with a spin after the wfi or not). What ends it
     * is the shape of the vendor's own CH32V003 WFE (its EVT's
     * core_riscv.h, read as the oracle; the CH32V006's EVT has none of
     * it): the wfi run from SRAM with interrupts masked and a spin of 19
     * turns in SRAM after it - so that no vector and no instruction is
     * fetched from flash in the first cycles after the wake. In that
     * shape the window wedges nothing in 1600 tries (a spin of 1 turn
     * still wedged); one cycle of it loses the edge instead, the masked
     * WFE's own hole - the V2 manual 5.2's item (3) is an edge, and the
     * edge on the entry cycle is not seen (the CH32V006 measured the same
     * of its masked order).
     *
     * The masked order moves the consume: an interrupt is taken only at
     * the unmask after the sleep, and a latch emptied before that comes
     * back (measured: 200 turns over 100 ticks with the V2C's consume
     * after the wake). So the latch is emptied first - SETEVENT and a wfi
     * it ends at once, 5 cycles - and the PFIC is asked what it may have
     * held: an enabled line pending means no sleep, the caller's unmask
     * takes it. An edge after that question latches for the sleep's own
     * wfi, which then returns at once. And the tick's edge is kept off the
     * entry: if its match is closer than sram_sleep_tick_guard_cycles
     * there is no sleep either, and the caller's loop turns until the tick
     * is in. What stays: the edge of another line that lands on the entry
     * cycle itself is late - it stands pending and masked, and the next
     * wake (the tick's, at the latest) lets the unmask take it.
     *
     * A V2A Sleep is not a light one: once entered it lasts at least about
     * 3150 cycles at 48 MHz (66 us) - an edge that comes sooner after the
     * entry runs the core again only then (measured with the edge 100 to
     * 2400 cycles after the entry, the sum constant to five cycles) - so
     * an interrupt that finds the core freshly asleep is served up to
     * 66 us late, where the V2C's is served in about 25 cycles.
     *
     * The spin's turns, for the CH32V003's entry alone.
     */
    static constexpr uint32_t sram_wake_spin_turns = 19;

    /// The tick guard of the CH32V003's entry. From the counter's read to
    /// the wfi are eleven instructions (the subtraction and compare, the
    /// SCTLR store, the call into SRAM), about thirty cycles at 48 MHz
    /// behind the flash's two wait states and fewer below; with the
    /// four-cycle window the guard is three times that. What it costs is
    /// the caller's loop turning instead of sleeping for at most this many
    /// cycles, once a tick period.
    static constexpr uint32_t sram_sleep_tick_guard_cycles = 96;

    /// mstatus.MIE readback: the one bit CriticalSection saves.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Whether the caller runs inside an interrupt handler: PFIC_GISR's
    /// GACTSTA (qingke/pfic.hpp's Pfic::in_handler(), which says why not
    /// mstatus.MIE; the optional member of kernel/platform.hpp).
    static bool in_handler() { return Pfic::in_handler(); }

    /**
     * Halt in the debugger: the panic path's breakpoint
     * (kernel/panic.hpp), callable from any context.
     *
     * CAVEAT. With a debugger halted on it, the program stops here. With
     * NO debugger attached, `ebreak` raises the breakpoint exception, and
     * where it lands is the crt's: on the CH32V00x, the vector table's
     * fault entry, which that crt provides as a distinct spin loop; on the
     * CH32V203 and the CH32V303, the table's own breakpoint entry, index 9
     * (measured; docs/ch32vx03/platform.md) - the crt's
     * breakpoint_handler, a weak alias of its default_handler spin until
     * the program binds it (to fault_reset<P>() of ch32vx03/reset.hpp,
     * which resets); on the CH32X035, the breakpoint entry too (the V4's
     * vector table, measured on the CH32V203), which that project's crt
     * gives a weak spin loop of its own, apart from the exception entry's
     * and the unbound vectors', so a probe that halts the core reads which
     * wreck it was from the program counter. Either way the wreck stops
     * where a probe can see it instead of running on into nothing. A
     * reporter that must survive without a debugger cannot assume this
     * call returns.
     */
    static void break_here() { __asm__ volatile("ebreak"); }

    /// The kernel's clock: the timebase's tick count, readable from any
    /// context (what kernel/time_event.hpp arms deadlines against).
    static uint32_t now() { return TB::ticks(); }

    /// Tick rate of the timebase: 1000 Hz on the core counter.
    static constexpr uint32_t ticks_per_second = TB::ticks_per_second;

    /// 32-bit core: an aligned word moves in one uninterruptible access,
    /// which is what lets util/ring.hpp take its lock-free path.
    static constexpr unsigned atomic_width = 4;

    /**
     * Panic breadcrumb in .noinit: the linker script places the section
     * and the crt neither loads nor zeroes it, so a record written just
     * before a reset can be reported at the next boot (written by the
     * panic path, read by take_panic_record() at boot).
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
    /// for header functions and a program's own names `.ram_text`. Never
    /// instantiated on a part whose entry is the ordinary one.
    [[gnu::section(".ram_text.brio"), gnu::noinline, gnu::noclone]] static void wfi_from_sram() {
        uint32_t n = sram_wake_spin_turns;
        __asm__ volatile("wfi\n1:\taddi %0, %0, -1\n\tbnez %0, 1b" : "+r"(n)::"memory");
    }

    /**
     * idle() with SLEEPDEEP armed: the tick held off across the deep
     * sleep (idle()'s comment says why), then the same store, unmask and
     * wfi. The handler that ends the sleep runs first (the WFE's item 2),
     * with the timebase still paused and on the clock the wake left; a
     * timed site's alarm body needs neither the tick's interrupt nor the
     * program's clock to begin, and puts the clock back as its first act
     * (ch32vx03/sleep.hpp). On the CH32V003 the masked SRAM entry
     * instead, the EXTI lines that interrupt made events for the
     * Standby's span (the sleep entry's wedge, "The CH32V003" above:
     * masked, a line's interrupt is no wake, and an EXTI event is one -
     * the V2 manual 5.2's item (1); the vendor's own deep WFE does the
     * same).
     */
    [[gnu::noinline]] static void idle_deep(uint32_t sctlr) {
        if constexpr (pauses_tick) {
            TB::pause();
        }
        stk()->SR = 0;
        Pfic::clear_pending(Irq::systick);
        const uint32_t wfe = (sctlr | sctlr_wfitowfe | sctlr_sevonpend) & ~idle_clears;
        if constexpr (sleep_entry_from_sram) {
            const uint32_t events = Chip::sram_deep_events_on();
            pfic_sctlr() = wfe;
            wfi_from_sram();
            Chip::sram_deep_events_off(events);
            enable_interrupts();
        } else {
            pfic_sctlr() = wfe;
            enable_interrupts();
            __asm__ volatile("wfi" ::: "memory");
        }
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
 * this walks up from the floor until the pattern breaks. The number is
 * the margin a program has - on the CH32V003's 2 KB of RAM above all -
 * and zero means the stack has already run into the data below it, which
 * no other symptom names. A word of the pattern legitimately on the stack
 * ends the walk early: the answer errs on the small side. Main context; a
 * read of memory.
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

} // namespace brio
