/*
 * nvic.hpp
 *
 * Interrupt control on the Cortex-M0+: the core's own NVIC (enable /
 * disable / pend one peripheral line, and its priority) plus the single
 * global mask every brio critical section rides on, PRIMASK.
 *
 * ARCHITECTURE, not chip - and this is the SECOND Cortex-M0+ family in
 * brio, so this file is deliberately the samc/nvic.hpp twin line for
 * line: only the device header include differs, and the IRQn_Type value
 * list callers name lines by comes from that header. The standing naming
 * rule says an `armv6m/` core stratum is factored out at the second ARM
 * family, and it is factored AFTER this bring-up with both copies in
 * hand and a byte-identity gate on every samc image - a pass of its own
 * (docs/design/overview.md). Until then the two files are twins by
 * discipline, and any divergence between them is a finding for that pass.
 *
 * It sits at the BOTTOM of the stm32g0/ stratum on purpose: the guard is
 * needed both by ticker.hpp (multi-word counter reads) and by
 * platform_stm32.hpp (where the kernel finds its CriticalSection), and
 * platform_stm32.hpp includes ticker.hpp - so the guard can live in
 * neither of them.
 *
 * PRIMASK and nothing else: ARMv6-M has no BASEPRI, so masking is
 * all-or-nothing - there is no "mask everything below priority N". The
 * NVIC priorities below therefore order PREEMPTION between handlers,
 * never the reach of a critical section.
 *
 * NOT here (declared, not built): the fault/exception configuration,
 * NVIC_SystemReset and the vector-table relocation (VTOR exists on this
 * core, unlike the samc's promise; RM0444 12.1) - they belong with the
 * reset/panic pass, the stm32g0 analog of samc/reset.hpp.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

namespace brio {

/// Number of distinct NVIC priority levels: the core implements
/// __NVIC_PRIO_BITS high bits of an 8-bit field (2 on this family ->
/// four levels, 0 the most urgent). The device header is the authority.
inline constexpr uint8_t irq_priority_levels = 1u << __NVIC_PRIO_BITS;

/**
 * RAII interrupt guard: the constructor masks every maskable interrupt,
 * the destructor restores the PREVIOUS PRIMASK, so guards nest and an
 * inner one never re-enables interrupts an outer one had masked. This is
 * what Stm32Platform::CriticalSection is.
 *
 * Both CMSIS intrinsics used here carry a "memory" clobber (cpsid i on
 * entry, msr primask on exit), which is exactly the compiler barrier the
 * Platform contract asks for: data shared with a handler needs no
 * volatile inside a guarded section.
 */
class InterruptGuard {
public:
    InterruptGuard() : saved_(__get_PRIMASK()) { __disable_irq(); }
    ~InterruptGuard() { __set_PRIMASK(saved_); }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t saved_;
};

/// Unmask interrupts globally (PRIMASK = 0). What an app calls once, at
/// the end of main()'s setup, before handing control to the kernel - the
/// sei() of this target.
inline void enable_interrupts() { __enable_irq(); }

/// Mask interrupts globally. Prefer InterruptGuard: a bare disable has
/// no matching restore and does not nest.
inline void disable_interrupts() { __disable_irq(); }

/// PRIMASK readback: true when interrupts are not masked.
inline bool interrupts_enabled() { return __get_PRIMASK() == 0u; }

/**
 * The NVIC as a monostate resource. `irq` is always the device header's
 * IRQn_Type value (USART2_LPUART2_IRQn, TIM3_TIM4_IRQn, ...); the core
 * exceptions (negative values, SysTick_IRQn among them) accept
 * priority() and nothing else - they have no enable bit, being enabled
 * by the peripheral that raises them (SysTick's own TICKINT).
 *
 * SHARED LINES ARE THE RULE ON THIS FAMILY, not the exception: RM0444
 * table 61 gives 32 positions to more peripherals than that, so USART2
 * shares a line with LPUART2, USART3..6 with LPUART1, TIM3 with TIM4,
 * and so on. Enabling a line enables it for every peripheral on it; the
 * app's handler for that vector calls each owner's ISR body in turn.
 */
struct Nvic {
    Nvic() = delete;

    static void enable(IRQn_Type irq) { NVIC_EnableIRQ(irq); }
    static void disable(IRQn_Type irq) { NVIC_DisableIRQ(irq); }
    static bool enabled(IRQn_Type irq) { return NVIC_GetEnableIRQ(irq) != 0u; }

    /// Raise the line in software (the pending bit is the same one the
    /// peripheral sets; clearing it drops an interrupt that has not run).
    static void set_pending(IRQn_Type irq) { NVIC_SetPendingIRQ(irq); }
    static void clear_pending(IRQn_Type irq) { NVIC_ClearPendingIRQ(irq); }
    static bool pending(IRQn_Type irq) { return NVIC_GetPendingIRQ(irq) != 0u; }

    /// Preemption priority, 0 (most urgent) .. irq_priority_levels - 1.
    /// Returns false and changes nothing when the level does not exist on
    /// this core, rather than silently truncating it.
    static bool priority(IRQn_Type irq, uint8_t level) {
        if (level >= irq_priority_levels) {
            return false;
        }
        NVIC_SetPriority(irq, level);
        return true;
    }
    static uint8_t priority(IRQn_Type irq) {
        return static_cast<uint8_t>(NVIC_GetPriority(irq));
    }
};

} // namespace brio
