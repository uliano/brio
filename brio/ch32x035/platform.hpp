/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on a CH32X035: the
 * CH32X035 (QingKe V4C, RV32IMAC) realization of the kernel's Platform
 * concept, the compiler-checked set of requirements kernel/platform.hpp
 * states. A program names it once, as the first argument of its Tenuto,
 * and every kernel template is instantiated with it:
 *
 *     using P = brio::Ch32x035Platform<>;
 *     BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     ...
 *     SysClock::init();
 *     brio::Ticker::init(clock);
 *     brio::enable_interrupts();
 *     brio::Tenuto<P, ...>::run();                       // the program's active objects
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time, a
 * breakpoint for a panic and the storage a panic record survives a reset
 * in. Who calls what: the kernel loop calls idle() in main-loop context,
 * with interrupts already masked, and nothing else should; now() and
 * ticks_per_second may be read from any context; CriticalSection may be
 * opened from any context and nests; break_here() is the panic path's
 * and panic_record() the panic path's and the next boot's;
 * stack_untouched() is a main-context read of memory. A program binds the
 * timebase's interrupt vector itself (systick_handler to Ticker::tick(),
 * with the binding BRIO_CH32_VECTOR, pfic.hpp); the kernel binds no
 * vector, and a program that forgets it never advances time.
 *
 * The QingKe V4C, the CH32V203's core generation: the V4C is the V4B
 * with a faster divider and the physical memory protection unit (the
 * reference manual's core table) - thirty-two registers, the atomic
 * extension, user mode, a 64-bit system counter - at a ceiling of 48 MHz.
 * What does not change is the contract above it: the kernel and util
 * strata compile here untouched.
 *
 * CriticalSection is ch32x035/pfic.hpp's InterruptGuard: read-and-clear
 * mstatus.MIE in one instruction, restore what was there on scope exit.
 * All-or-nothing masking. This core does have a priority threshold and
 * two levels of interrupt nesting, and under Tenuto brio uses neither:
 * an interrupt handler's body runs to completion here as everywhere
 * (docs/design/kernel.md section 1, "The model in one page"). The one
 * program that turns the nesting on is a Rubato one, through
 * ch32x035/schedule.hpp's scheduler_init(): its scheduling request, the
 * software interrupt, is put one preemption level below every peripheral
 * line, which still never nest among themselves (Ch32x035RubatoPlatform,
 * at the end of this file).
 *
 * What is not here, and is deliberate rather than forgotten: no
 * idle_until() - this platform is not tickless, its timebase is the core
 * counter and stops with the core.
 *
 * No sleep over a working DMA channel. In the Sleep of RM 2.3.2 the DMA
 * controller moves nothing - measured on the CH32X035F8U6
 * (docs/ch32x035/dma.md): a 4096-item copy started before a sleep had
 * moved 68 items when the tick woke the core 19 000 cycles later, and a
 * block paced by a USART's transmitter ran at a fiftieth of its pace -
 * the CH32V203's and the CH32V303's answer (brio/ch32vx03/bus_activity.hpp).
 * So idle() asks the controller first, `Dma::any_working()` - a channel
 * enabled and circular or with items left - and returns at once, unmasked,
 * while one is: a program with an engine at work, a receive ring above
 * all, holds itself awake without having to know that it does. The
 * silicon's own registers are the answer, read in the idle path alone:
 * one load with the controller's gate closed, two a channel with it open;
 * a count kept by the engines would cost their hot paths a guarded
 * increment for an answer the registers already hold. The USB
 * controllers, the other bus masters of the series, have no driver here.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/schedule.hpp"
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
     * The kernel's idle: entered by the loop, in main-loop context, with
     * interrupts masked and nothing to do; unmasks, then sleeps until the
     * next interrupt, and returns with interrupts enabled. Not for a
     * program to call. With a DMA channel at work it does not sleep at
     * all (the file header): it unmasks and returns at once.
     *
     * Not a plain WFI. The RISC-V privileged specification lets WFI resume
     * when an interrupt is pending whatever mstatus.MIE says, and the
     * Cortex-M targets lean on the same property to sleep first and unmask
     * after. The QingKe V4 manual (6.2) wakes a WFI on "the interrupt
     * source responded by the interrupt controller", which does not
     * promise it, and a bare WFI is not what this hook executes, so the
     * question is left unasked here.
     *
     * The instruction is the core's WFE, executed with MIE set.
     * PFIC_SCTLR.WFITOWFE makes `wfi` a wait-for-event and SEVONPEND makes
     * every interrupt entering the pending state an event, latched: "if
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
     *    enabled interrupt with MIE set wakes a WFE by V4 manual 6.2's
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
     * One turn per wake: the latch is consumed after it. The edge that wakes
     * the WFE is an interrupt entering the pending state, so SEVONPEND
     * latches it too, and left there it ends the next idle() at once: the
     * caller's loop turns twice per interrupt, the second turn finding
     * nothing (measured on the CH32X035F8U6: 200 kernel turns over 100
     * quiet ticks, docs/ch32x035/platform.md). So after the wake the hook
     * writes SETEVENT - "set the event to wake up the WFE case" (RM
     * 7.5.2.38) - and executes one more `wfi`, which that event ends at
     * once and which leaves the latch clear whatever it held. MIE is set
     * by then, so an enabled interrupt pending around the consume is taken
     * by its level (6.2's second item) and its handler runs before the
     * caller's next masked check; what the consume can eat is a latch whose
     * interrupt has been or is being taken, or the edge of a line the PFIC
     * does not enable, which wakes nothing the kernel waits for. It comes
     * after the sleep, where WCH's `__WFE()` puts its SETEVENT and first
     * `wfi` before it: with the unmask first, an interrupt taken between
     * the unmask and a consume placed before the sleep would have its
     * latch eaten and its post found only at the next interrupt.
     *
     * What depth. SLEEPDEEP as found: out of reset it is clear, so this is
     * the plain Sleep of RM 2.3.2 - the core clock stopped, every
     * peripheral clock running. No verb of this stratum sets SLEEPDEEP; a
     * program that does has armed a Stop or a Standby (RM 2.3.3, 2.3.4),
     * and the path is then idle_deep(), out of line.
     */
    static void idle() {
        if (Dma::any_working()) {
            // A channel at work: no sleep of any depth (the file header).
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

    /// mstatus.MIE readback: the one bit CriticalSection saves.
    static bool interrupts_enabled() { return brio::interrupts_enabled(); }

    /// Whether the caller runs inside an interrupt handler: PFIC_GISR's
    /// GACTSTA (ch32x035/pfic.hpp's Pfic::in_handler(), which says why not
    /// mstatus.MIE; the optional member of kernel/platform.hpp).
    static bool in_handler() { return Pfic::in_handler(); }

    /**
     * Halt in the debugger: the panic path's breakpoint
     * (kernel/panic.hpp), callable from any context.
     *
     * Caveat. With a debugger halted on it, the program stops here. With
     * no debugger attached, `ebreak` raises the breakpoint exception,
     * which the CH32V203's core takes to the vector table's own
     * breakpoint entry (measured there); the crt of this project gives
     * that entry a weak spin loop of its own, apart from the exception
     * entry's and the unbound vectors', so a probe that halts the core
     * reads which wreck it was from the program counter. A reporter that
     * must survive without a debugger cannot assume this call returns.
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

    /// This family places `.fast_text` in SRAM: the seven linker scripts
    /// collect it first in .data, which the crt copies before main(), and
    /// the vector table holds absolute addresses, so code there is entered
    /// and called without a veneer (docs/ch32x035/platform.md, "A handler
    /// in SRAM"). A driver of this stratum may therefore be given
    /// util/placement.hpp's InRam.
    static constexpr bool places_fast_text = true;

    /**
     * Panic breadcrumb in .noinit: the linker script places the section
     * and the crt neither loads nor zeroes it, so a record written just
     * before a reset can be reported at the next boot (written by the
     * panic path, read by take_panic_record() at boot).
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

static_assert(Platform<Ch32x035Platform<>> && HandlerAware<Ch32x035Platform<>>);

/**
 * Ch32x035RubatoPlatform<TB>: the platform a Rubato kernel runs on
 * (kernel/rubato.hpp), the preemptive one. It is Ch32x035Platform<TB>
 * whole - the same critical section, idle, timebase, breakpoint and panic
 * storage - plus the three members kernel/platform.hpp's Preemptive
 * concept asks for:
 *
 *     using P = brio::Ch32x035RubatoPlatform<>;
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
 *     BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }
 *     int main() { ...; brio::scheduler_init(); brio::enable_interrupts(); K::run(); }
 *
 *  - `preemptive`, true;
 *  - `ceiling`, the byte the kernel keeps: the pack index of the object
 *    being dispatched, 255 at the loop's base. A static of the platform,
 *    so of the program: one kernel per platform type, which on this
 *    single-core family is one kernel;
 *  - `pend_schedule()`, the request for the scheduler: ch32x035/
 *    schedule.hpp's software interrupt, below every other line, with
 *    interrupt nesting turned on so that the peripheral handlers, at one
 *    level among themselves, interrupt it. The software interrupt
 *    returns into the scheduler in thread mode, and the scheduler leaves
 *    through the machine-mode environment call; that file says how, with
 *    the block each level of preemption leaves on the stack.
 *
 * The CriticalSection is still mstatus.MIE alone, and the promise that an
 * interrupt handler is not interrupted by another keeps its meaning for
 * the peripheral handlers, which share one preemption level and never
 * nest among themselves; the scheduler's interrupt sits below them all,
 * and they do interrupt it. A program on this platform binds two vectors
 * more than a Tenuto program, the software interrupt's and the
 * machine-mode environment call's, both naked, executes no `ecall` of
 * its own, and calls scheduler_init() before it enables interrupts.
 */
template <class TB = Ticker>
struct Ch32x035RubatoPlatform : Ch32x035Platform<TB> {
    static constexpr bool preemptive = true;

    /// The index of the object being dispatched; 255 at the loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler (ch32x035/schedule.hpp).
    [[gnu::always_inline]] static void pend_schedule() { request_scheduler(); }
};

static_assert(Platform<Ch32x035RubatoPlatform<>> && HandlerAware<Ch32x035RubatoPlatform<>>);
static_assert(Preemptive<Ch32x035RubatoPlatform<>>);
static_assert(!Preemptive<Ch32x035Platform<>>);

} // namespace brio
