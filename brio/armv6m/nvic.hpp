/*
 * nvic.hpp - the CORE stratum, first file.
 *
 * Interrupt control on any ARMv6-M core (Cortex-M0/M0+): the core's own
 * NVIC (enable / disable / pend one peripheral line, and its priority)
 * plus the single global mask every brio critical section rides on,
 * PRIMASK. Nothing in this file knows a vendor: it is the part of a
 * Cortex-M0+ target that ARM designed, and it became a stratum of its
 * own at the SECOND ARM family in brio, as the naming rule foresaw -
 * `samc/nvic.hpp` and `stm32g0/nvic.hpp` were twins line for line, and
 * this is that one file, with the two families' headers reduced to the
 * include that selects the device.
 *
 * WHAT A FAMILY OWES BEFORE INCLUDING THIS: its device header. The
 * CMSIS core header this file relies on (core_cm0plus.h: the PRIMASK
 * intrinsics, the NVIC_* functions, IRQn_Type, __NVIC_PRIO_BITS) is
 * itself included BY the device header, after the device has declared
 * its IRQn enumerators and priority width - so a family's nvic.hpp
 * includes "sam.h" or "stm32g0xx.h" and then this file, and this file
 * refuses to be included first. That is the one include-order contract
 * of the armv6m stratum, and every file here states it.
 *
 * PRIMASK and nothing else: ARMv6-M has no BASEPRI, so masking is
 * all-or-nothing - there is no "mask everything below priority N". The
 * NVIC priorities below therefore order PREEMPTION between handlers,
 * never the reach of a critical section.
 *
 * NOT here (declared, not built): the fault/exception configuration,
 * NVIC_SystemReset and the vector-table relocation - they belong with
 * each family's reset/panic pass (a system reset request is core, the
 * reset CAUSE is the vendor's).
 */

#pragma once

#include <stdint.h>

#if !defined(__CM0PLUS_REV) && !defined(__CM0_REV)
#error "armv6m/nvic.hpp: include the family's device header first (samc/nvic.hpp and stm32g0/nvic.hpp do) - the CMSIS core header it brings is what this file is written against"
#endif

namespace brio {

/// Number of distinct NVIC priority levels: the core implements
/// __NVIC_PRIO_BITS high bits of an 8-bit field (2 on both families so
/// far -> four levels, 0 the most urgent). The device header is the
/// authority.
inline constexpr uint8_t irq_priority_levels = 1u << __NVIC_PRIO_BITS;

/**
 * RAII interrupt guard: the constructor masks every maskable interrupt,
 * the destructor restores the PREVIOUS PRIMASK, so guards nest and an
 * inner one never re-enables interrupts an outer one had masked. This is
 * what each family's Platform::CriticalSection is.
 *
 * Both CMSIS intrinsics used here carry a "memory" clobber (cpsid i on
 * entry, msr primask on exit), which is exactly the compiler barrier the
 * Platform contract asks for: data shared with a handler needs no
 * volatile inside a guarded section. No extra asm barrier is emitted -
 * one that adds nothing would only claim to.
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
/// sei() of these targets.
inline void enable_interrupts() { __enable_irq(); }

/// Mask interrupts globally. Prefer InterruptGuard: a bare disable has
/// no matching restore and does not nest.
inline void disable_interrupts() { __disable_irq(); }

/// PRIMASK readback: true when interrupts are not masked.
inline bool interrupts_enabled() { return __get_PRIMASK() == 0u; }

/**
 * The NVIC as a monostate resource. `irq` is always the device header's
 * IRQn_Type value; the core exceptions (negative values, SysTick_IRQn
 * among them) accept priority() and nothing else - they have no enable
 * bit, being enabled by the peripheral that raises them (SysTick's own
 * TICKINT).
 *
 * A LINE MAY BE SHARED by several peripherals (the rule on the STM32G0,
 * one line on the SAM C21): enabling it enables it for all of them, and
 * the app's handler for that vector calls each owner's ISR body in turn.
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
