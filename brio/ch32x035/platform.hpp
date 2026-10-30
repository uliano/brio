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
 * The platform is the QingKe core stratum's (qingke/platform.hpp's
 * QingkePlatform, whose account it is: the critical section, the
 * WFE-shaped idle() and why it unmasks first, the latch consumed after
 * the wake, the deep path, the breakpoint, the panic breadcrumb,
 * stack_untouched()), over the one fact of this family `Ch32x035Qingke`
 * below gives it: the question idle() asks before it sleeps. In the
 * Sleep of RM 2.3.2 the DMA controller moves nothing - measured on the
 * CH32X035F8U6 (docs/ch32x035/dma.md) - the CH32V203's and the
 * CH32V303's answer (brio/ch32vx03/bus_activity.hpp). So idle() asks
 * the controller first, `Dma::any_working()` - a channel enabled and
 * circular or with items left - and returns at once, unmasked, while one
 * is. The silicon's own registers are the answer, read in the idle path
 * alone: one load with the controller's gate closed, two a channel with
 * it open; a count kept by the engines would cost their hot paths a
 * guarded increment for an answer the registers already hold. The USB
 * controllers, the other bus masters of the series, have no driver here.
 * The platform adds `places_fast_text`: the series' section layout
 * (ch32x035/ld/sections.ld), which every part's image links, places
 * `.fast_text`.
 *
 * Who calls what is QingkePlatform's. A program binds the timebase's
 * interrupt vector itself (systick_handler to Ticker::tick(), with the
 * binding BRIO_CH32_VECTOR, pfic.hpp); the kernel binds no vector, and a
 * program that forgets it stops in the crt's spin for unbound vectors at
 * the first tick after interrupts are enabled. No verb of this stratum
 * sets SLEEPDEEP; a program that does has armed a Stop or a Standby (RM
 * 2.3.3, 2.3.4), and idle() then takes the deep path.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/schedule.hpp"
#include "ch32x035/ticker.hpp"

namespace brio {

/// The fact of this family qingke/platform.hpp's QingkePlatform is
/// written over (the QingkeChip concept there names its members).
/// Monostate.
struct Ch32x035Qingke {
    Ch32x035Qingke() = delete;

    /// Whether a bus master other than the core is working, which the
    /// Sleep would stall (the file header) - here a DMA channel,
    /// `Dma::any_working()`, the controller's own registers. Idle path
    /// only; one load with the gate closed, two a channel with it open.
    [[gnu::always_inline]] static bool bus_masters_working() { return Dma::any_working(); }
};

} // namespace brio

#include "qingke/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32x035Platform : QingkePlatform<Ch32x035Qingke, TB> {
    /// This family places `.fast_text` in SRAM: the series' section layout
    /// (ch32x035/ld/sections.ld) collects it in .data, after the scheduler
    /// path where an image places it, which the crt copies before main(), and
    /// the vector table holds absolute addresses, so code there is entered
    /// and called without a veneer (docs/ch32x035/platform.md, "A handler
    /// in SRAM"). A driver of this stratum may therefore be given
    /// util/placement.hpp's InRam.
    static constexpr bool places_fast_text = true;
};

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
 *     BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }
 *     int main() { ...; brio::scheduler_init(); brio::enable_interrupts(); K::run(); }
 *
 *  - `preemptive`, true;
 *  - `ceiling`, the byte the kernel keeps: the pack index of the object
 *    being dispatched or just served, 0 while the scheduler chooses, 255
 *    at the loop's base (kernel/platform.hpp). A static of the platform,
 *    so of the program: one kernel per platform type, which on this
 *    single-core family is one kernel;
 *  - `pend_schedule()`, the request for the scheduler: qingke/
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

    /// The index of the object being dispatched or just served, 0 while
    /// the scheduler chooses; 255 at the loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler (qingke/schedule.hpp).
    [[gnu::always_inline]] static void pend_schedule() { request_scheduler(); }
};

static_assert(Platform<Ch32x035RubatoPlatform<>> && HandlerAware<Ch32x035RubatoPlatform<>>);
static_assert(Preemptive<Ch32x035RubatoPlatform<>>);
static_assert(!Preemptive<Ch32x035Platform<>>);

} // namespace brio
