/*
 * core_m33.hpp
 *
 * The Arm half of this target's core names: what rp2350/core.hpp
 * provides when the image is built for the Cortex-M33 pair. A program
 * never includes this file; it includes core.hpp and uses the names
 * listed there (InterruptGuard, enable_interrupts(), Irq,
 * wait_for_interrupt(), debug_break(), ...), which this half and
 * rp2350/core_hazard3.hpp both export.
 *
 * Nobody includes this file directly: rp2350/core.hpp does, when the
 * processor in the socket is an M33, and that is the one place in the
 * stratum where the architecture is asked. A driver that needs a
 * critical section or an interrupt line includes core.hpp and gets the
 * same names whichever half answered.
 *
 * What it holds. A Cortex-M33 pair is, for everything brio asks of a
 * core, the programmer's model the other ARM families already share -
 * the NVIC's per-line enables, PRIMASK as the one global mask, WFI,
 * BKPT. So this file is the device header and then `cortexm/nvic.hpp`
 * (InterruptGuard, the global mask verbs, Nvic), exactly as
 * samc21/nvic.hpp and stm32f4/nvic.hpp are, plus the verbs that spell
 * this core's instructions and the names rp2350/core.hpp promises. The
 * contexts are core.hpp's: InterruptGuard from any context, masking this
 * core alone; Irq's verbs one register access each on this core's NVIC;
 * wait_for_interrupt() the platform's idle path, entered masked;
 * sleep_releases_power_request() a sleep site's, and empty here.
 *
 * What is deliberately not here. The M33's own extensions - the MPU, the
 * SAU and TrustZone, MSPLIM, the two coprocessor ports, BASEPRI - are
 * untouched: the kernel's promise is that no interrupt nests over
 * another (design/kernel.md sections 1, The model in one page, and 11,
 * Platform: what the machine provides), which is kept here as on every
 * other Cortex-M family by leaving every line at the reset priority and
 * masking with PRIMASK alone. `Nvic::priority()` exists because the core
 * stratum offers it; no driver of this stratum calls it, and `Irq`
 * (core.hpp's name for the controller) does not carry it, because the
 * RISC-V half's priorities run the other way - numerically higher is
 * more urgent there - and a verb that means two things is worse than no
 * verb.
 *
 * Everything an image runs here is Secure: the bootrom hands over in
 * Secure state and nothing in brio leaves it, so `SysTick`, `SCB` and
 * the NVIC are the Secure banked copies, which are the ones the vector
 * table in the crt belongs to.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "cortexm/nvic.hpp"

namespace brio {

/// The per-line interrupt controller, under the name both architectures
/// answer to. On this half it is the NVIC of the core that runs the
/// verb - there are two, one per M33, and a line reaches both.
using Irq = Nvic;

/// Sleep until an interrupt is pending: the platform's idle path, called
/// with interrupts masked (rp2350/platform.hpp). PRIMASK does not stop a
/// pending interrupt from waking the core (it only stops the handler from
/// running), which is what makes the kernel's mask-check-sleep-unmask
/// idle path free of a lost-wakeup window. The DSB is ARM's own
/// recommendation: the posted writes retire before the core stops.
[[gnu::always_inline]] inline void wait_for_interrupt() {
    __DSB();
    __WFI();
}

/// Whether a sleeping core releases its power request, which is half of
/// datasheet 6.5.2's condition for the chip's SLEEP state; set and read
/// by the sleep site (rp2350/sleep.hpp).
///
/// A Cortex-M33 does it by itself: its SLEEPING output is asserted for
/// the duration of a WFI and the clock controller reads that, so there
/// is nothing here to arm and this verb is empty (and the readback always
/// true). It exists because the other half of this chip has to be asked
/// (rp2350/core_hazard3.hpp), and the sleep sites are written once for
/// both.
[[gnu::always_inline]] inline void sleep_releases_power_request(bool) {}
[[gnu::always_inline]] inline bool sleep_releases_power_request() { return true; }

/// Halt in the debugger: the panic path's breakpoint, from any context.
/// With no debugger attached this escalates to a fault, which is the
/// legible wreck a panic wants.
[[gnu::always_inline]] inline void debug_break() { __BKPT(0); }

} // namespace brio
