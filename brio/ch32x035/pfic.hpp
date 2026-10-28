/*
 * pfic.hpp
 *
 * Interrupt control on the CH32X035, for a program and for the drivers:
 * the global interrupt enable, the critical section the kernel masks
 * with, the per-line enables of the interrupt controller (the PFIC,
 * WCH's own), and the macro every interrupt handler of this target is
 * bound with. A program meets all four:
 *
 *     BRIO_CH32_VECTOR(usart2_handler) { (void)Serial::isr(); }
 *     brio::Pfic::enable(brio::Irq::usart2);     // a driver's init() does this itself
 *     brio::enable_interrupts();                  // once, before the kernel runs
 *     { brio::InterruptGuard guard; ... }         // a critical section, any context
 *
 * Who calls what: enable_interrupts() and disable_interrupts() are one
 * instruction each, legal from any context; InterruptGuard may be opened
 * from any context and nests; every Pfic verb is one store or one load
 * of a write-one bank, legal from any context with no guard; nothing
 * here waits.
 *
 * Two layers, like every target. `mstatus.MIE` is the RISC-V global mask
 * - one bit, all or nothing, the equivalent of the AVR's I flag and of
 * ARMv6-M's PRIMASK - and `InterruptGuard` is the kernel's
 * CriticalSection over it: read-and-clear on entry, restore the previous
 * value on exit, so guards nest. `csrrci` does the read and the clear in
 * one instruction, which is what makes the guard correct against an
 * interrupt arriving between the two halves. This file is the
 * CH32V203's (brio/ch32vx03/pfic.hpp) word for word where the two
 * QingKe V4 cores agree, which is everywhere this file reaches: the
 * machine-mode CSRs, the write-one enable banks, the handler binding.
 * With the CH32V00x's, that makes three QingKe families holding the same
 * file, and folding them into a `qingke/` core stratum is a change of its
 * own, made with every image of the three byte-identical across it.
 *
 * The per-line enables belong to the PFIC. Its enable and disable
 * registers are write-one: a write of 1 to a bit acts, a 0 does nothing,
 * so there is no read-modify-write and no guard needed around it.
 *
 * The vector binding is one spelling for the whole image.
 * BRIO_CH32_VECTOR(name) { body } is what an app writes to bind a vector
 * (an `extern "C"` function under the crt's name for it, with the
 * handler attribute), and it follows the project's CH32X035_HPE option
 * (ch32x035/CMakeLists.txt, on by default; the build passes it to the
 * compiler as BRIO_CH32_HPE):
 * with the core's hardware prologue/epilogue on, the crt has set
 * INTSYSCR.HWSTKEN and the attribute is WCH's "WCH-Interrupt-fast" - the
 * sixteen caller-saved integer registers pushed by the hardware on entry
 * and popped on MRET (QingKe V4 manual 3.4, two levels deep on the V4C)
 * and gcc emits no prologue at all; with it off, the attribute is the
 * plain `interrupt` and the handler saves what it clobbers itself. The
 * two must agree: a fast handler under an HPE that is off returns into a
 * program whose registers it clobbered. One macro, one option, no way to
 * spell them apart. The binding is the CH32V203's spelling
 * (brio/ch32vx03/pfic.hpp), whose CH32V303 images expand it to a
 * trampoline; this core has no floating-point unit, so its two siblings
 * there, BRIO_CH32_LEAF_VECTOR and BRIO_CH32_VECTOR_FLOAT, are
 * BRIO_CH32_VECTOR exactly here. A vector placed in a section of its own
 * (`.fast_text`, docs/ch32x035/platform.md, "A handler in SRAM") is
 * declared with the section first and then bound:
 *
 *     extern "C" [[gnu::section(".fast_text")]] void usart4_handler();
 *     BRIO_CH32_VECTOR(usart4_handler) { ... }
 *
 * and gcc merges the declaration's section into the definition (an
 * attribute cannot be written in front of the macro, whose expansion
 * begins with `extern "C"`).
 *
 * And the macro forbids gcc to merge two handlers. Two bindings with the
 * same body are what gcc's identical code folding looks for, and gcc
 * folds them - by turning the second into a handler that calls the first:
 * a call that never comes back, because the first ends in MRET, leaving
 * whatever the caller had put on the stack there (measured on the
 * CH32V303VCT6, docs/ch32vx03/platform.md). `no_icf` keeps every handler
 * whole, from this stratum's first line, under both attributes.
 *
 * What is not here. This core can nest interrupts - two levels, chosen in
 * INTSYSCR with the priority bytes in IPRIOR - and WCH's own startup file
 * turns that on. brio does not: the kernel's rule (docs/design/kernel.md
 * section 1, "The model in one page") is that an interrupt handler's body
 * runs to completion, on every target, and the crt leaves INESTEN clear.
 * The four free vectored entries (VTFIDR/VTFADDRR) are a real feature of
 * this core with no user; they arrive with one.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"

/// The binding of every vector of this target (see the file header):
/// `BRIO_CH32_VECTOR(usart2_handler) { ... }` at file scope, the one place
/// the choice between the core's hardware prologue and gcc's is spelled.
/// Every handler is aligned to four: from the flash, code with taken
/// branches costs what its address modulo four makes it (the fetch reads
/// a word; docs/ch32x035/platform.md, "A handler in SRAM"), and gcc
/// aligns a function to two under -Os - so without it a handler's cost
/// moved with whatever the image put before it, by up to ten cycles on
/// the USART's completion. Two bytes of padding a handler at most.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
#define BRIO_CH32_HANDLER_ATTRIBUTE [[gnu::interrupt("WCH-Interrupt-fast"), gnu::no_icf, gnu::aligned(4)]]
#else
#define BRIO_CH32_HANDLER_ATTRIBUTE [[gnu::interrupt, gnu::no_icf, gnu::aligned(4)]]
#endif
#define BRIO_CH32_VECTOR(name) extern "C" BRIO_CH32_HANDLER_ATTRIBUTE void name()
#define BRIO_CH32_VECTOR_FLOAT(name) BRIO_CH32_VECTOR(name)
#define BRIO_CH32_LEAF_VECTOR(name) BRIO_CH32_VECTOR(name)

namespace brio {

/// mstatus.MIE, the one global interrupt enable.
inline constexpr uint32_t mstatus_mie = 1UL << 3;

/// Is the global enable set? One CSR read, any context.
inline bool interrupts_enabled() {
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    return (mstatus & mstatus_mie) != 0u;
}

/// Set the global enable: what a program calls once, after every
/// driver's init() and before the kernel runs. One instruction.
inline void enable_interrupts() {
    __asm__ volatile("csrsi mstatus, 8" ::: "memory");
}

/// Clear the global enable. One instruction; the guard below restores.
inline void disable_interrupts() {
    __asm__ volatile("csrci mstatus, 8" ::: "memory");
}

/**
 * The kernel's CriticalSection on this target: interrupts masked from
 * the point it is declared to the end of the enclosing scope. Any
 * context, nests.
 *
 * The constructor clears mstatus.MIE and keeps what it found; the
 * destructor sets the bit again only if it was set, so an inner guard
 * inside an outer one leaves interrupts masked on the way out. Both
 * halves carry a "memory" clobber, which is the barrier the Platform
 * concept asks for: data shared with a handler needs no volatile of its
 * own inside a guarded region.
 */
class InterruptGuard {
public:
    [[gnu::always_inline]] InterruptGuard() {
        __asm__ volatile("csrrci %0, mstatus, 8" : "=r"(saved_) :: "memory");
    }

    [[gnu::always_inline]] ~InterruptGuard() {
        if ((saved_ & mstatus_mie) != 0u) {
            __asm__ volatile("csrsi mstatus, 8" ::: "memory");
        }
    }

    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;

private:
    uint32_t saved_;
};

/// PFIC_GISR.GACTSTA: an interrupt handler is executing (Pfic::in_handler()).
inline constexpr uint32_t pfic_gisr_gactsta = 1UL << 8;

/// The interrupt controller's per-line enables. Monostate: there is one
/// PFIC, and its registers are write-one, so no verb here needs a guard
/// around it.
struct Pfic {
    Pfic() = delete;

    /// Let a line reach its vector. A driver's init() enables its own
    /// lines; a program enables a line it binds itself. Write-one.
    static void enable(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IENR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Stop a line from reaching its vector. Write-one.
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

    /// Raise a line by hand - the software interrupt's own way in, and a
    /// test's way of firing any vector on demand. Write-one.
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

    /// Whether the caller runs inside an interrupt handler: PFIC_GISR's
    /// GACTSTA, bit 8 of the register at offset 0x4C, which the chapter
    /// describes as an interrupt being executed, whatever the nesting level
    /// (the reference manual 7.5.2.11 and the QingKe V4 manual 3.1). Not
    /// mstatus.MIE, which a critical section in thread code clears as a
    /// trap's entry does, so it cannot tell a handler from a guard. The
    /// platform's in_handler() (kernel/platform.hpp's optional member) is
    /// this read. Any context; one load and a test.
    static bool in_handler() { return (pfic()->GISR & pfic_gisr_gactsta) != 0u; }
};

} // namespace brio
