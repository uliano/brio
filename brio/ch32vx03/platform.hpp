/*
 * platform.hpp
 *
 * What the brio kernel stands on when it runs on a CH32V203 or a
 * CH32V303: this stratum's realization of the kernel's Platform
 * concept, the compiler-checked set of requirements kernel/platform.hpp
 * states, on the QingKe V4B of the CH32V203 (RV32IMAC) and the QingKe
 * V4F of the CH32V303 (the same core with a single-precision
 * floating-point unit, RV32IMAFC). It is the one header of this stratum
 * the kernel templates are instantiated with: a program names it once,
 * as the first argument of its Tenuto (docs/ch32vx03/platform.md, "How
 * to use it"):
 *
 *     using P = brio::Ch32vx03Platform<>;
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     ...
 *     SysClock::init();
 *     brio::Ticker::init(clock);
 *     brio::enable_interrupts();
 *     brio::Tenuto<P, ...>::run();   // the program's active objects
 *
 * The platform is the QingKe core stratum's (qingke/platform.hpp's
 * QingkePlatform, whose account it is: the critical section, the
 * WFE-shaped idle() and why it unmasks first, the latch consumed after
 * the wake, the deep path that holds the tick off, the breakpoint, the
 * panic breadcrumb, stack_untouched()), over the one fact of this family
 * `Ch32vx03Qingke` below gives it: the question idle() asks before it
 * sleeps. In a sleep of any depth here no bus master but the core gets a
 * cycle (ch32vx03/bus_activity.hpp carries the measurements and the
 * reasoning), so a sleep taken while a DMA channel or the USB controller
 * is working does not slow the program down - it loses the transfer. The
 * platform therefore reads the count of active masters the drivers keep
 * and sleeps only at zero; above zero it returns at once with interrupts
 * enabled, which the Platform contract allows and which leaves the loop
 * spinning and the bus served. The platform adds `bus_masters_active()`,
 * that count, for a program and a suite to ask the same question; it
 * does not say `places_fast_text`, the code flash's window running at
 * zero wait (docs/ch32vx03/platform.md, "A handler in SRAM").
 *
 * Who calls what is QingkePlatform's; bus_masters_active() may be read
 * from any context. A program binds the timebase's interrupt vector
 * itself (systick_handler to Ticker::tick(), with one of
 * ch32vx03/pfic.hpp's three binding macros) and initializes the ticker
 * after the clock task; a deep mode is armed by a sleep site
 * (ch32vx03/sleep.hpp).
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/schedule.hpp"
#include "ch32vx03/ticker.hpp"

namespace brio {

/// The fact of this family qingke/platform.hpp's QingkePlatform is
/// written over (the QingkeChip concept there names its members).
/// Monostate.
struct Ch32vx03Qingke {
    Ch32vx03Qingke() = delete;

    /// Whether a bus master other than the core is working - a DMA
    /// channel or a USB controller, the count this family's drivers keep
    /// (ch32vx03/bus_activity.hpp) - which a sleep of any depth here would
    /// break rather than slow. Any context; one load.
    [[gnu::always_inline]] static bool bus_masters_working() { return BusActivity::active() != 0u; }
};

} // namespace brio

#include "qingke/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32vx03Platform : QingkePlatform<Ch32vx03Qingke, TB> {
    /// How many bus masters other than the core are working - the
    /// number idle() reads, published here so a program (and a suite)
    /// can ask the same question the sleep path asks.
    static uint8_t bus_masters_active() { return BusActivity::active(); }
};

static_assert(Platform<Ch32vx03Platform<>> && HandlerAware<Ch32vx03Platform<>>);

/**
 * Ch32vx03RubatoPlatform<TB>: the platform a Rubato kernel runs on
 * (kernel/rubato.hpp), the preemptive one. It is Ch32vx03Platform<TB>
 * whole - the same critical section, idle, timebase, breakpoint and panic
 * storage - plus the three members kernel/platform.hpp's Preemptive
 * concept asks for:
 *
 *     using P = brio::Ch32vx03RubatoPlatform<>;
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
struct Ch32vx03RubatoPlatform : Ch32vx03Platform<TB> {
    static constexpr bool preemptive = true;

    /// The index of the object being dispatched or just served, 0 while
    /// the scheduler chooses; 255 at the loop's base.
    static inline uint8_t ceiling = 255;

    /// The request for the scheduler (qingke/schedule.hpp).
    [[gnu::always_inline]] static void pend_schedule() { request_scheduler(); }
};

static_assert(Platform<Ch32vx03RubatoPlatform<>> && HandlerAware<Ch32vx03RubatoPlatform<>>);
static_assert(Preemptive<Ch32vx03RubatoPlatform<>>);
static_assert(!Preemptive<Ch32vx03Platform<>>);

} // namespace brio
