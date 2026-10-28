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
 * active objects in priority order. BRIO_CH32_VECTOR is the one spelling
 * of a vector binding on this target (ch32v00x/pfic.hpp);
 * fault_reset<P>() is the fault vector's body (ch32v00x/reset.hpp).
 *
 * The platform is the QingKe core stratum's (qingke/platform.hpp's
 * QingkePlatform, whose account it is: the critical section, the
 * WFE-shaped idle() and why it unmasks first, the CH32V003's masked
 * sleep entry from SRAM and the measurements behind it, the panic
 * breadcrumb, stack_untouched()), over the facts of this family, which
 * `Ch32v00xQingke` below gives it:
 *  - the question idle() asks before it sleeps: a working DMA channel on
 *    the CH32V003, whose V2A's Sleep stalls the controller
 *    (device::sleep_stops_bus_masters, parts/ch32v003.hpp), and nothing
 *    on the CH32V006, whose V2C's Sleep the DMA works through;
 *  - the CH32V003's sleep entry, masked from SRAM
 *    (device::sleep_entry_from_sram), and the EXTI lines that interrupt
 *    made events for a Standby's span, the wake a masked entry has.
 * The platform adds `places_fast_text`: both linker scripts place
 * `.fast_text`.
 *
 * What is not here, on purpose: no idle_until() - this platform is not
 * tickless, its timebase is the core counter and stops with the core in
 * a Standby. The sleep sites are ch32v00x/sleep.hpp's; which reset
 * happened, the fault body and the watchdogs are ch32v00x/reset.hpp's.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/exti.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/ticker.hpp"

namespace brio {

/// The facts of this family qingke/platform.hpp's QingkePlatform is
/// written over (the QingkeChip concept there names them). Monostate.
struct Ch32v00xQingke {
    Ch32v00xQingke() = delete;

    /// Whether a bus master other than the core is working that the
    /// Sleep would stall: a DMA channel on the CH32V003, never on the
    /// CH32V006, where the answer is a constant and the question
    /// compiles to nothing. Idle path only.
    [[gnu::always_inline]] static bool bus_masters_working() {
        return device::sleep_stops_bus_masters && Dma::any_working();
    }

    /// The CH32V003's sleep entry runs masked from SRAM (its part table).
    static constexpr bool sleep_entry_from_sram = device::sleep_entry_from_sram;

    /// The CH32V003's Standby, entered masked: the EXTI lines that
    /// interrupt made events for its span - masked, a line's interrupt is
    /// no wake, and an EXTI event is one (QingKe V2 manual 5.2's item
    /// (1); the vendor's own deep WFE does the same). Returns the event
    /// mask it found, for sram_deep_events_off().
    [[gnu::always_inline]] static uint32_t sram_deep_events_on() {
        const uint32_t events = exti()->EVENR;
        exti()->EVENR = events | exti()->INTENR;
        return events;
    }

    /// The event mask given back after the Standby.
    [[gnu::always_inline]] static void sram_deep_events_off(uint32_t events) { exti()->EVENR = events; }
};

} // namespace brio

#include "qingke/platform.hpp"

namespace brio {

template <class TB = Ticker>
struct Ch32v00xPlatform : QingkePlatform<Ch32v00xQingke, TB> {
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
};

static_assert(Platform<Ch32v00xPlatform<>> && HandlerAware<Ch32v00xPlatform<>>);

} // namespace brio
