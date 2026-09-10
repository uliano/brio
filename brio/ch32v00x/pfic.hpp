/*
 * pfic.hpp
 *
 * Interrupt control on the CH32V00x: the global mask that is a CSR bit,
 * and the controller that holds the per-line enables.
 *
 * TWO LAYERS, LIKE EVERY TARGET. `mstatus.MIE` is the RISC-V global
 * mask - one bit, all or nothing, the equivalent of the AVR's I flag
 * and of ARMv6-M's PRIMASK - and `InterruptGuard` is the kernel's
 * CriticalSection over it: read-and-clear on entry, restore the
 * PREVIOUS value on exit, so guards nest. `csrrci` does the read and
 * the clear in ONE instruction, which is what makes the guard correct
 * against an interrupt arriving between the two halves.
 *
 * The per-line enables belong to the PFIC, WCH's own controller (the
 * device header calls it NVIC as well; this stratum uses the name the
 * chapter uses). Its enable and disable registers are WRITE-ONE: a
 * write of 1 to a bit acts, a 0 does nothing, so there is no
 * read-modify-write and no guard needed around it.
 *
 * WHAT IS NOT HERE. The block's priorities, its four free vectored
 * entries (VTFIDR/VTFADDR) and the hardware stack are real features of
 * this core and none of them has a user yet - they arrive with one.
 * The interrupt NESTING and hardware-stacking bits of INTSYSCR stay at
 * their reset value (both off, RM 6.5.3.1): with them off, a handler is
 * an ordinary `[[gnu::interrupt]]` function that saves in software what
 * it uses, which is what upstream gcc emits and what keeps this stratum
 * compilable by a toolchain that is not WCH's.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"

namespace brio {

/// mstatus.MIE, the one global interrupt enable.
inline constexpr uint32_t mstatus_mie = 1UL << 3;

inline bool interrupts_enabled() {
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    return (mstatus & mstatus_mie) != 0u;
}

inline void enable_interrupts() {
    __asm__ volatile("csrsi mstatus, 8" ::: "memory");
}

inline void disable_interrupts() {
    __asm__ volatile("csrci mstatus, 8" ::: "memory");
}

/**
 * The kernel's CriticalSection on this target.
 *
 * The constructor clears mstatus.MIE and keeps what it found; the
 * destructor sets the bit again only if it WAS set, so an inner guard
 * inside an outer one leaves interrupts masked on the way out. Both
 * halves carry a "memory" clobber, which is the barrier the Platform
 * concept asks for: data shared with a handler needs no volatile of
 * its own inside a guarded region.
 */
class InterruptGuard {
public:
    InterruptGuard() {
        __asm__ volatile("csrrci %0, mstatus, 8" : "=r"(saved_) :: "memory");
    }

    ~InterruptGuard() {
        if ((saved_ & mstatus_mie) != 0u) {
            __asm__ volatile("csrsi mstatus, 8" ::: "memory");
        }
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t saved_;
};

/// The interrupt controller's per-line enables. Monostate: there is one
/// PFIC, and its two registers are write-one, so no verb here needs a
/// guard around it.
struct Pfic {
    Pfic() = delete;

    static void enable(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IENR[n >> 5] = 1UL << (n & 0x1fu);
    }

    static void disable(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IRER[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Is the line enabled? (The manual's ISR bank is the ENABLE status.)
    static bool enabled(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (pfic()->ISR[n >> 5] & (1UL << (n & 0x1fu))) != 0u;
    }

    /// Is the line asserted and waiting? (The IPR bank.)
    static bool pending(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (pfic()->IPR[n >> 5] & (1UL << (n & 0x1fu))) != 0u;
    }
};

} // namespace brio
