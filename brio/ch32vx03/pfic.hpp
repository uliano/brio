/*
 * pfic.hpp
 *
 * Interrupt control on the CH32V203: the global mask that is a CSR bit,
 * and the controller that holds the per-line enables.
 *
 * TWO LAYERS, LIKE EVERY TARGET. `mstatus.MIE` is the RISC-V global mask
 * - one bit, all or nothing, the equivalent of the AVR's I flag and of
 * ARMv6-M's PRIMASK - and `InterruptGuard` is the kernel's
 * CriticalSection over it: read-and-clear on entry, restore the PREVIOUS
 * value on exit, so guards nest. `csrrci` does the read and the clear in
 * ONE instruction, which is what makes the guard correct against an
 * interrupt arriving between the two halves. This half of the file is
 * the CH32V00x's word for word, because the two QingKe cores spell the
 * machine-mode CSRs the same way; it is NOT yet factored into a core
 * stratum, which is a measurement this family owes and not a guess (the
 * cortexm/ stratum was born that way, with two families in hand and
 * every image byte-identical across the move).
 *
 * The per-line enables belong to the PFIC, WCH's own controller. Its
 * enable and disable registers are WRITE-ONE: a write of 1 to a bit
 * acts, a 0 does nothing, so there is no read-modify-write and no guard
 * needed around it.
 *
 * THE VECTOR BINDING IS ONE SPELLING FOR THE WHOLE IMAGE.
 * BRIO_CH32_VECTOR(name) { body } is what an app writes to bind a
 * vector, and it follows two facts of the build: the project's
 * CH32VX03_HPE option (ch32vx03/CMakeLists.txt) and whether the image is
 * compiled with the F extension (the compiler's __riscv_flen, exactly
 * when the part table's ISA carries F - the CH32V303's images).
 *
 * WITHOUT F (every CH32V203 image) it is an attributed handler under the
 * name. With the core's hardware prologue/epilogue on, the crt has set
 * INTSYSCR.HWSTKEN and the attribute is WCH's "WCH-Interrupt-fast": the
 * sixteen caller-saved integer registers pushed by the hardware on entry
 * and popped on MRET (QingKe V4 manual 3.2, 3.4 note 3), gcc emitting no
 * prologue for them and ending the function with MRET; with it off, the
 * attribute is the plain `interrupt` and the handler saves what it
 * clobbers itself. The two must agree: a fast handler under an HPE that
 * is off returns into a program whose registers it clobbered. One
 * macro, one option, no way to spell them apart.
 *
 * WITH F AND THE HPE ON (the CH32V303's images as the project builds
 * them) the vector is a NAKED TRAMPOLINE: two instructions, a call of
 * an ordinary function `name_body` holding the braces that follow, and
 * MRET. The reason is what the hardware prologue does NOT save - an
 * f-register, ever (3.4 note 3) - and what gcc does about it: a function
 * with the interrupt attribute that calls out saves every f-register the
 * ABI lets a callee clobber, the twenty ft0..ft11 and fa0..fa7, without
 * looking at the callee, so on this core one call anywhere in a handler
 * costs eighty bytes of stack and forty memory operations on every
 * entry (docs/ch32vx03/platform.md measures them). A naked function gets
 * no prologue from gcc at all, and the body, an ordinary function under
 * ilp32f, saves only what the ABI asks of a callee - the callee-saved
 * registers it uses, the return address if it calls - and none of the
 * caller-saved ones, the sixteen integers being the hardware's. In the
 * release listings those saves are gcc's ordinary prologue at the
 * body's entry (the I2C host's event body: ra, s0, s1), and a body
 * whose last act is a call ends in a tail jump that returns straight
 * to the trampoline (the console's receive body and the kernel's
 * `post`), costing nothing more. The trampoline's own cost: the call
 * (one `jal`, two bytes compressed where the body is within reach, four
 * otherwise) and the body's `ret`, against no instruction at all in an
 * attributed leaf - measured on the CH32V303VCT6 (the platform suite's
 * letter e) three cycles on the way in and five on the way out, where a
 * handler that calls out sheds forty memory operations. With F and the
 * HPE OFF the binding stays the attributed handler of the plain
 * `interrupt` (gcc saves whatever a call might clobber, f-registers
 * included, in software): the f-register question is then the software
 * prologue's own, and a trampoline would buy nothing there.
 *
 * WHY A MACRO. What differs between the two expansions is not a type or
 * a value but the SHAPE of a declaration and the SYMBOL a vector table
 * entry names - an attributed function called `name`, or a naked one
 * called `name` and a second function called `name_body` - and no
 * template names a symbol. The trampoline's body is basic asm alone,
 * which is gcc's rule for a naked function.
 *
 * THE TRAMPOLINE'S PROMISE IS ENFORCED, NOT STATED. Nothing saves the
 * f-registers of the program a trampoline interrupts, so its body and
 * everything the body reaches must use none: `brio check ch32vx03` walks
 * the call graph of every trampoline's body in every linked image of
 * the F presets, from the vector table's own entries, and FAILS on a
 * floating-point instruction anywhere on it, or on an indirect call it
 * cannot resolve, naming the vector and the function
 * (cli/vector_guard.py). A body that NEEDS float binds with
 * BRIO_CH32_VECTOR_FLOAT(name) instead: the same trampoline with the
 * twenty caller-saved f-registers stored around the call - the set gcc
 * saves for a handler that calls out, so fcsr is not among them: a body
 * that changes the rounding mode or raises a flag leaves it changed for
 * the program it interrupted - and the guard leaves its body alone.
 * Without F, or with the HPE off, the two macros are one.
 *
 * AND THE ATTRIBUTED FORM FORBIDS gcc TO MERGE TWO HANDLERS. Two
 * bindings with the same body - one transport's two DMA channels, each
 * vector calling the same dma_isr() - are what gcc's identical code
 * folding looks for, and gcc folds them - upstream 16.2 and WCH's 15.2
 * alike, at -Os and not at -O2, with either attribute - by turning the
 * second into a handler that CALLS the first: a call that never comes
 * back, because the first ends in MRET. Whatever the caller had put on
 * the stack stays there - on the V4F the twenty f-registers it saves
 * before any call, eighty bytes - and the interrupted program resumes
 * with its stack pointer that far below its frame (measured on the
 * CH32V303VCT6: the next return jumped to a saved float). `no_icf` keeps
 * every attributed handler whole, the one spelling that holds under
 * both attributes and both ABIs. A trampoline needs no such guard: no
 * two of them are alike (each calls its own body), and two bodies that
 * fold are ordinary functions, for which a fold is an alias or a jump
 * that returns where it should; it carries `no_icf` all the same, so
 * that no reading of the rule depends on the asm text differing.
 *
 * WHAT IS NOT HERE. This core can NEST interrupts - two, four or eight
 * levels, chosen in INTSYSCR with the priority bytes in IPRIOR - and
 * WCH's own startup file turns that on. brio does not: the kernel's rule
 * (docs/design/kernel.md section 1) is that an ISR body runs to
 * completion, on every target, and the crt leaves INESTEN clear. The
 * four free vectored entries (VTFADDRR) are a real feature of this core
 * with no user yet; they arrive with one.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"

/// The binding of every vector of this target (see the file header):
/// `BRIO_CH32_VECTOR(usart1_handler) { Serial::isr(); }` - the ONE place
/// the handler's shape is chosen, from the hardware prologue option and
/// the image's floating-point ABI.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
#define BRIO_CH32_HANDLER_ATTRIBUTE [[gnu::interrupt("WCH-Interrupt-fast"), gnu::no_icf]]
#else
#define BRIO_CH32_HANDLER_ATTRIBUTE [[gnu::interrupt, gnu::no_icf]]
#endif

#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE && defined(__riscv_flen)
// The naked trampoline: the hardware has saved the sixteen caller-saved
// integer registers (ra among them, which the call overwrites and MRET
// puts back), the body saves what a callee must, MRET returns.
#define BRIO_CH32_VECTOR(name)                                   \
    extern "C" void name##_body();                               \
    extern "C" [[gnu::naked, gnu::no_icf]] void name() {         \
        __asm__("call " #name "_body\n\t"                         \
                "mret");                                         \
    }                                                            \
    extern "C" void name##_body()
// The same trampoline for a body that does float work: the twenty
// caller-saved f-registers (ft0..ft7 = f0..f7, fa0..fa7 = f10..f17,
// ft8..ft11 = f28..f31) stored on the user stack around the call, a
// frame of eighty bytes that keeps the stack's 16-byte alignment.
#define BRIO_CH32_VECTOR_FLOAT(name)                                                       \
    extern "C" void name##_body();                                                         \
    extern "C" [[gnu::naked, gnu::no_icf]] void name() {                                   \
        __asm__("addi sp, sp, -80\n\t"                                                     \
                "fsw ft0, 0(sp)\n\t"  "fsw ft1, 4(sp)\n\t"  "fsw ft2, 8(sp)\n\t"           \
                "fsw ft3, 12(sp)\n\t" "fsw ft4, 16(sp)\n\t" "fsw ft5, 20(sp)\n\t"          \
                "fsw ft6, 24(sp)\n\t" "fsw ft7, 28(sp)\n\t" "fsw fa0, 32(sp)\n\t"          \
                "fsw fa1, 36(sp)\n\t" "fsw fa2, 40(sp)\n\t" "fsw fa3, 44(sp)\n\t"          \
                "fsw fa4, 48(sp)\n\t" "fsw fa5, 52(sp)\n\t" "fsw fa6, 56(sp)\n\t"          \
                "fsw fa7, 60(sp)\n\t" "fsw ft8, 64(sp)\n\t" "fsw ft9, 68(sp)\n\t"          \
                "fsw ft10, 72(sp)\n\t" "fsw ft11, 76(sp)\n\t"                              \
                "call " #name "_body\n\t"                                                  \
                "flw ft0, 0(sp)\n\t"  "flw ft1, 4(sp)\n\t"  "flw ft2, 8(sp)\n\t"           \
                "flw ft3, 12(sp)\n\t" "flw ft4, 16(sp)\n\t" "flw ft5, 20(sp)\n\t"          \
                "flw ft6, 24(sp)\n\t" "flw ft7, 28(sp)\n\t" "flw fa0, 32(sp)\n\t"          \
                "flw fa1, 36(sp)\n\t" "flw fa2, 40(sp)\n\t" "flw fa3, 44(sp)\n\t"          \
                "flw fa4, 48(sp)\n\t" "flw fa5, 52(sp)\n\t" "flw fa6, 56(sp)\n\t"          \
                "flw fa7, 60(sp)\n\t" "flw ft8, 64(sp)\n\t" "flw ft9, 68(sp)\n\t"          \
                "flw ft10, 72(sp)\n\t" "flw ft11, 76(sp)\n\t"                              \
                "addi sp, sp, 80\n\t"                                                      \
                "mret");                                                                   \
    }                                                                                      \
    extern "C" void name##_body()
#else
#define BRIO_CH32_VECTOR(name) extern "C" BRIO_CH32_HANDLER_ATTRIBUTE void name()
#define BRIO_CH32_VECTOR_FLOAT(name) BRIO_CH32_VECTOR(name)
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
};

} // namespace brio
