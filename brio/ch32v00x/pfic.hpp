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
 * THE HANDLER ATTRIBUTE IS ONE SPELLING FOR THE WHOLE IMAGE.
 * BRIO_CH32_INTERRUPT is what an app puts on a vector binding, and it
 * follows the project's CH32V00X_HPE option (ch32v00x/CMakeLists.txt):
 * with the core's hardware prologue/epilogue on, the crt has set
 * INTSYSCR.HWSTKEN and the attribute is WCH's "WCH-Interrupt-fast" - the
 * ten caller-saved registers are pushed to the stack by the hardware
 * on entry and popped on MRET (QingKe V2 manual 3.4), and gcc emits
 * no prologue at all; with it off, the attribute is the plain
 * `interrupt` and the handler saves what it clobbers itself. The two
 * must agree: a fast handler under an HPE that is off returns into a
 * program whose registers it clobbered. One macro, one option, no
 * way to spell them apart. Interrupt NESTING is off in both cases:
 * the kernel's rule (docs/design/kernel.md) that an ISR body runs to
 * completion holds on this target by INTSYSCR.INESTEN never being set.
 *
 * WHAT IS NOT HERE. The block's priorities and its two free vectored
 * entries (VTFIDR/VTFADDR) are real features of this core with no user
 * yet - they arrive with one.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"

/// The attribute of every interrupt handler of this target (see the
/// file header): what an app writes on a vector binding, and the ONE
/// place the choice between the core's hardware prologue and gcc's is
/// spelled. Where only the preprocessor can ask (a build option, an
/// attribute), a macro is the honest tool.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
#define BRIO_CH32_INTERRUPT [[gnu::interrupt("WCH-Interrupt-fast")]]
#else
#define BRIO_CH32_INTERRUPT [[gnu::interrupt]]
#endif

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

    /// Raise a line by hand - the software interrupt's own way in, and
    /// a test's way of firing any vector on demand. Write-one.
    static void set_pending(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IPSR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Withdraw a pending line. Write-one.
    static void clear_pending(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IPRR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Is the line being serviced right now? (The IACTR bank.)
    static bool active(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        return (pfic()->IACTR[n >> 5] & (1UL << (n & 0x1fu))) != 0u;
    }
};

} // namespace brio
