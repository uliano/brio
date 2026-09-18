/*
 * core_m33.hpp
 *
 * The Arm half of this target's core: an Cortex-M33 pair, which for
 * everything brio asks of a core is the programmer's model the other ARM
 * families already share - the NVIC's per-line enables, PRIMASK as the
 * one global mask, WFI, BKPT. So this file is the device header and then
 * `cortexm/nvic.hpp`, exactly as samc21/nvic.hpp and stm32f4/nvic.hpp
 * are, plus the two verbs that spell this core's instructions and the
 * names rp2350/core.hpp promises.
 *
 * NOBODY INCLUDES THIS FILE DIRECTLY: rp2350/core.hpp does, when the
 * processor in the socket is an M33, and that is the ONE place in the
 * stratum where the architecture is asked. A driver that needs a
 * critical section or an interrupt line includes core.hpp and gets the
 * same names whichever half answered.
 *
 * WHAT IS DELIBERATELY NOT HERE. The M33's own extensions - the MPU, the
 * SAU and TrustZone, MSPLIM, the two coprocessor ports, BASEPRI - are
 * untouched: the kernel's promise is that no interrupt nests over
 * another (design/kernel.md sections 1 and 11), which is kept here as on
 * every other Cortex-M family by leaving every line at the reset
 * priority and masking with PRIMASK alone. `Nvic::priority()` exists
 * because the core stratum offers it; no driver of this stratum calls
 * it, and `Irq` (core.hpp's name for the controller) does not carry it,
 * because the RISC-V half's priorities run the OTHER WAY - numerically
 * higher is more urgent there - and a verb that means two things is
 * worse than no verb.
 *
 * Everything an image runs here is SECURE: the bootrom hands over in
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
/// answer to. On this half it IS the NVIC of the core that runs the
/// verb - there are two, one per M33, and a line reaches both.
using Irq = Nvic;

/// Sleep until an interrupt is pending. PRIMASK does not stop a pending
/// interrupt from waking the core (it only stops the handler from
/// running), which is what makes the kernel's mask-check-sleep-unmask
/// idle path free of a lost-wakeup window. The DSB is ARM's own
/// recommendation: the posted writes retire before the core stops.
[[gnu::always_inline]] inline void wait_for_interrupt() {
    __DSB();
    __WFI();
}

/// Halt in the debugger. With no debugger attached this escalates to a
/// fault, which is the legible wreck a panic wants.
[[gnu::always_inline]] inline void debug_break() { __BKPT(0); }

} // namespace brio
