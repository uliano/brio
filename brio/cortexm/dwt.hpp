/*
 * dwt.hpp - the CORE stratum: the DWT's cycle counter.
 *
 * CYCCNT is a free-running 32-bit count of processor clock cycles, kept
 * by the Data Watchpoint and Trace unit that ARMv7-M and ARMv8-M main
 * define (DDI 0403E C1.8.3): it wraps to zero transparently, it is read
 * in ONE load from the private peripheral bus, and it needs no interrupt
 * and no composition - which is what SysTick's cycles() has to do
 * (cortexm/ticker.hpp: the tick count, the position in the period and
 * the pending bit, five reads and a compose). A difference of two reads
 * is exact under 2^32 cycles, about 24 s at 180 MHz.
 *
 * WHAT TURNS IT ON. Two bits, in this order: DEMCR.TRCENA (C1.6.5), the
 * global enable of the DWT and the ITM - while it is 0 the DWT's
 * registers read UNKNOWN - and DWT_CTRL.CYCCNTENA (C1.8.7). The counter
 * is OPTIONAL in the architecture: DWT_CTRL.NOCYCCNT reads 1 where the
 * unit has none (C1.8.8), and init() then answers false and enables
 * nothing.
 *
 * WHAT A DEBUGGER DOES TO IT. A probe may set TRCENA itself (watchpoints
 * and trace live in the same unit), and nothing promises it leaves the
 * bit as it found it - so init() is IDEMPOTENT: it sets only what reads
 * clear, clears nothing a probe set, and never writes CYCCNT (a reader
 * takes differences, and a profiler on the probe's side keeps its own
 * base). A caller that cannot know what the probe did since calls it
 * again; on a running counter it is two reads and no write. And the
 * counter stops while the core is HALTED in Debug state (C1.8.8), so a
 * breakpoint inside a measured span removes the halt from the reading.
 *
 * THE COST OF A READ: one LDR from 0xE0001004 on the private peripheral
 * bus, two cycles on the Cortex-M4 when it does not pipeline with a
 * neighbouring load (DDI 0439B table 3-1, 3.3.2), plus the address it
 * needs in a register. now() is forced inline so that a stamp in a
 * handler is that load and nothing more.
 *
 * WHAT IT COUNTS THROUGH is a family's fact, not the core's: the
 * architecture counts the cycles spent in WFI inside CYCCNT (C1.8.4's
 * identity puts DWT_SLEEPCNT's cycles in it), but whether a family's
 * sleep keeps the counter's clock running is the family header's to say
 * (stm32f4/dwt.hpp); a sleep that stops the processor clock altogether
 * (an STM32's Stop) stops the counter with it.
 *
 * WHICH CORES. ARMv6-M has no cycle counter, and CMSIS's core_cm0plus.h
 * declares no DWT at all, so this file is refused on the M0 and the M0+
 * by the static_assert below. The registers are named by CMSIS-Core 5's
 * core headers for both the M4 and the M33 (DEMCR is the DCB's on
 * ARMv8-M, and CMSIS 5 keeps the CoreDebug name for it); the one family
 * that includes this file is the STM32F4's, and what ARMv8-M adds around
 * the counter - counting in Secure state under the authentication
 * interface - is not addressed here.
 *
 * Include-order contract: the family's device header first (it brings
 * the CMSIS core header with `DWT` and `CoreDebug`); the family's
 * dwt.hpp does.
 */

#pragma once

#include <stdint.h>

#if !defined(__CM0PLUS_REV) && !defined(__CM0_REV) && !defined(__CM4_REV) && !defined(__CM33_REV)
#error "cortexm/dwt.hpp: include the family's device header first (stm32f4/dwt.hpp does) - the CMSIS core header it brings is what this file is written against"
#endif

static_assert(__CORTEX_M >= 3u,
              "cortexm/dwt.hpp: the DWT's cycle counter is an ARMv7-M / ARMv8-M main unit; "
              "an ARMv6-M core (the M0, the M0+) has none - its cycle ruler is SysTick's "
              "cycles() (cortexm/ticker.hpp)");

// The definitions name CMSIS's DWT and CoreDebug, which an ARMv6-M core
// header does not declare: the static_assert above is then the one
// diagnostic such a build sees, rather than the first of a cascade.
#if __CORTEX_M >= 3u

namespace brio {

/// The DWT's cycle counter as a monostate: enable it once (and again
/// whenever a probe may have touched it), read it from any context.
struct CycleCounter {
    CycleCounter() = delete;

    /// TRCENA, then CYCCNTENA, each written only where it reads clear.
    /// True when the counter is implemented and its enable reads back
    /// set; false - and CYCCNTENA left alone - when DWT_CTRL.NOCYCCNT
    /// says the unit has no counter. CYCCNT itself is never written.
    static bool init() {
        if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
            CoreDebug->DEMCR = CoreDebug->DEMCR | CoreDebug_DEMCR_TRCENA_Msk;
        }
        if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u) {
            return false;
        }
        if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
            DWT->CTRL = DWT->CTRL | DWT_CTRL_CYCCNTENA_Msk;
        }
        return running();
    }

    /// Both enables read set: the counter is counting (outside a halt).
    static bool running() {
        return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u &&
               (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u;
    }

    /// The count: one load, from any context. Wraps at 2^32; take
    /// differences of two reads, unsigned.
    [[gnu::always_inline]] static uint32_t now() { return DWT->CYCCNT; }
};

} // namespace brio

#endif
