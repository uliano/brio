/*
 * pfic.hpp
 *
 * Interrupt control on the CH32V203 and the CH32V303, for a program and
 * for the drivers: the global interrupt enable, the critical section the
 * kernel masks with, the per-line enables of the interrupt controller
 * (the PFIC, WCH's own), and the three macros that bind a vector - one
 * spelling a program writes whatever the build, the macro choosing the
 * handler's shape from the build. A program meets the macros and the
 * global enable as docs/ch32vx03/platform.md's "How to use it" writes
 * them:
 *
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }
 *     ...
 *     brio::enable_interrupts();
 *
 * and the rest from inside the drivers, as ticker.hpp's init() enables
 * its line and its now() masks:
 *
 *     Pfic::enable(Irq::systick);
 *     InterruptGuard guard;
 *
 * A binding stands at file scope, one per vector the program owns, under
 * the crt's name for the vector (the crt is the startup code that runs
 * before main(), ch32vx03/src/glue/startup_ch32vx03.S). Every handler
 * the crt's table names is a weak alias of its `default_handler`, a spin
 * loop (entry 0 is the jump to the reset path and the reserved words are
 * zero), so a line enabled with no binding stops the program there with
 * no fault and no message. `P` is the program's platform type
 * (`using P = brio::Ch32vx03Platform<>;`, platform.hpp). A body that
 * compiles to no call and touches no f-register - the tick's, whose
 * Ticker::tick() is always_inline and so leaves no call behind - binds
 * with the leaf form, a body that does float work with
 * BRIO_CH32_VECTOR_FLOAT, any other with BRIO_CH32_VECTOR; on the
 * CH32V303's images (the QingKe V4F core, the V4B's twin with a
 * single-precision floating-point unit) a BRIO_CH32_VECTOR body must
 * reach no floating-point instruction at all, and `brio check ch32vx03`
 * fails the image whose body does (below) - a check of the linked
 * images that an ordinary build does not run.
 * enable_interrupts() runs once in main(), after the drivers' init()
 * and before the kernel runs.
 *
 * Who calls what: enable_interrupts() and disable_interrupts() are one
 * instruction each and interrupts_enabled() one CSR read, legal from any
 * context; InterruptGuard may be declared in any context and nests,
 * masking interrupts from its declaration to the end of the enclosing
 * scope - for as long as that scope runs, which the guard does not
 * bound; every Pfic verb is one store into a write-one bank or one load
 * of a status bank, legal from any context with no guard. Nothing here
 * waits.
 *
 * Two layers, like every target. `mstatus.MIE` is the RISC-V global mask
 * - one bit, all or nothing, the equivalent of the AVR's I flag and of
 * ARMv6-M's PRIMASK - and `InterruptGuard` is the kernel's
 * CriticalSection over it: read-and-clear on entry, restore the previous
 * value on exit, so guards nest. `csrrci` does the read and the clear in
 * one instruction, which is what makes the guard correct against an
 * interrupt arriving between the two halves. The code of this half is
 * the CH32V00x's (brio/ch32v00x/pfic.hpp) token for token, because the
 * two QingKe cores spell the machine-mode CSRs the same way; it is not
 * yet factored into a core stratum, which is a measurement this family
 * owes and not a guess (the cortexm/ stratum was born that way, with
 * two families in hand and every image byte-identical across the move).
 *
 * The per-line enables belong to the PFIC, WCH's own controller. Its
 * enable and disable registers are write-one: a write of 1 to a bit
 * acts, a 0 does nothing, so there is no read-modify-write and no guard
 * needed around it.
 *
 * The vector binding is one spelling for the whole image.
 * BRIO_CH32_VECTOR(name) { body } is what an app writes to bind a
 * vector, and it follows two facts of the build: the project's
 * CH32VX03_HPE option (a CMake option of the ch32vx03/ project, chosen
 * at configure and on by default, ch32vx03/CMakeLists.txt; the build
 * passes it to the compiler as BRIO_CH32_HPE), which turns on the core's
 * hardware prologue/epilogue (the HPE: the core itself saving and
 * restoring registers around a handler), and whether the image is
 * compiled with the F extension (the compiler's __riscv_flen, exactly
 * when the project's part table, ch32vx03/cmake/ch32vx03-parts.cmake,
 * gives the part an ISA that carries F - the CH32V303's images).
 *
 * Without F (every CH32V203 image) it is an attributed handler under the
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
 * With F and the HPE on (the CH32V303's images as the project builds
 * them) the vector is a naked trampoline: two instructions, a call of
 * an ordinary function `name_body` holding the braces that follow, and
 * MRET. The reason is what the hardware prologue does not save - an
 * f-register, ever (3.4 note 3) - and what gcc does about it: a
 * function with the interrupt attribute that calls out saves every
 * f-register the ABI lets a callee clobber, the twenty ft0..ft11 and
 * fa0..fa7, without looking at the callee, so on this core one call
 * anywhere in a handler costs eighty bytes of stack and forty memory
 * operations on every entry (docs/ch32vx03/platform.md measures them).
 * A naked function gets no prologue from gcc at all, and the body, an
 * ordinary function under ilp32f, saves only what the ABI asks of a
 * callee - the callee-saved registers it uses, the return address if it
 * calls - and none of the caller-saved ones, the sixteen integers being
 * the hardware's. In the release listings those saves are gcc's
 * ordinary prologue at the body's entry (the I2C host's event body: ra,
 * s0, s1), and a body whose last act is a call ends in a tail jump that
 * returns straight to the trampoline (a serial port's receive body, and
 * the kernel's `post`), costing nothing more. The trampoline's own
 * cost: the call (one `jal`, two bytes compressed where the body is
 * within reach, four otherwise) and the body's `ret`, against no
 * instruction at all in an attributed leaf - measured on the
 * CH32V303VCT6 (the platform suite's letter e; a suite's letter is one
 * command key of its console and the group of verdicts it runs) two
 * cycles on the way in and two to five on the way out, where a handler
 * that calls out sheds forty memory operations. With F and the HPE off
 * the binding stays the attributed handler of the plain `interrupt`
 * (gcc saves whatever a call might clobber, f-registers included, in
 * software): the f-register question is then the software prologue's
 * own, and a trampoline would buy nothing there.
 *
 * Why a macro. What differs between the two expansions is not a type or
 * a value but the shape of a declaration and the symbol a vector table
 * entry names - an attributed function called `name`, or a naked one
 * called `name` and a second function called `name_body` - and no
 * template names a symbol. The trampoline's body is basic asm alone,
 * which is gcc's rule for a naked function.
 *
 * The trampoline's promise is enforced, not stated. Nothing saves the
 * f-registers of the program a trampoline interrupts, so its body and
 * everything the body reaches must use none: `brio check ch32vx03` walks
 * the call graph of every trampoline's body in every linked image of
 * the F presets, from the vector table's own entries, and fails on a
 * floating-point instruction anywhere on it, or on an indirect call it
 * cannot resolve, naming the vector and the function
 * (cli/vector_guard.py). A body that needs float binds with
 * BRIO_CH32_VECTOR_FLOAT(name) instead: the same trampoline with the
 * twenty caller-saved f-registers stored around the call - the set gcc
 * saves for a handler that calls out - and fcsr too, which gcc does not
 * save. The body starts from fcsr's reset state (round to nearest, no
 * flag), not from the interrupted program's: a handler's arithmetic is
 * then the same whatever it interrupts - a program that sets another
 * rounding mode for its own work does not change its handlers' results
 * with it - and the flags the body finds are the ones it raised; the
 * swap that clears fcsr is the instruction that reads it, so the
 * choice costs nothing over inheriting. The interrupted program gets
 * its own rounding mode and sticky flags back, whatever the body did to
 * them. The guard checks that trampoline's stores and loads, fcsr's
 * among them, and leaves its body alone; the platform suite's letter l
 * judges both halves on the CH32V303VCT6 (the body finds 0, the
 * program finds its 0x20 and its 0x81 after a body that raised NX).
 * fcsr's four instructions cost twelve cycles of round trip there, 123
 * against 111 in two images that differ in them alone.
 *
 * A leaf needs no trampoline. A body that calls nothing and touches no
 * f-register saves nothing under the fast attribute, F or not, so the
 * trampoline's call and return are its whole cost there.
 * BRIO_CH32_LEAF_VECTOR(name) { body } binds it as the attributed
 * handler of the hardware prologue even under F (without F, or with the
 * HPE off, it is BRIO_CH32_VECTOR exactly, so those images do not move);
 * measured on the CH32V303VCT6 in two images that differ in the
 * binding alone, letter e's software interrupt 53..66 cycles of round
 * trip against 57..70 under the trampoline (entry 16 against 18, exit
 * 25 against 27..30), and the tick's edge to its
 * handler's first statement 22 cycles against 24 (letter w). The
 * promise is the guard's too, both ways: a vector bound leaf that gcc
 * compiled with a call, a tail jump, an indirect transfer other than a
 * jump table into itself, or an f-register touched fails the check,
 * naming it; a trampoline whose body came out a leaf is reported as a
 * candidate for the leaf form (a line of the check's output, not a
 * failure). The binding's mark in the image is an absolute symbol,
 * `__brio_leaf_<name>`, which costs no byte of it. And since the three
 * macros are the only bindings the guard can hold to a promise, any
 * other entry of an F image's table - a handler attributed by hand
 * among them - fails the check too, the crt's default handler alone
 * excepted. Without F, or with the HPE off, the three macros are one.
 *
 * And the attributed form forbids gcc to merge two handlers. Two
 * bindings with the same body - one transport's two DMA channels, each
 * vector calling the same dma_isr() - are what gcc's identical code
 * folding looks for, and gcc folds them - upstream 16.2 and WCH's 15.2
 * alike, at -Os and not at -O2, with either attribute - by turning the
 * second into a handler that calls the first: a call that never comes
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
 * What is not here. This core can nest interrupts - two, four or eight
 * levels, chosen in INTSYSCR with the priority bytes in IPRIOR - and
 * WCH's own startup file turns that on. brio does not, as a rule: the
 * kernel's rule (docs/design/kernel.md section 1, "The model in one
 * page") is that an ISR body runs to completion, on every target, so the
 * crt leaves INESTEN clear and no driver writes a priority byte, and the
 * peripheral handlers never nest among themselves. The one program that
 * turns nesting on is a Rubato one, through ch32vx03/schedule.hpp's
 * scheduler_init(): its scheduling request, the software interrupt, is
 * put one preemption level below every peripheral line, which still
 * share one level; that file owns the write to INTSYSCR and to that one
 * priority byte. The four free vectored entries (VTFADDRR) are a real
 * feature of this core with no user yet; they arrive with one.
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"

/// The binding of every vector of this target (see the file header):
/// `BRIO_CH32_VECTOR(usart1_handler) { Serial::isr(); }` at file scope -
/// the one place the handler's shape is chosen, from the hardware
/// prologue option and the image's floating-point ABI - with its float
/// sibling BRIO_CH32_VECTOR_FLOAT and its leaf sibling
/// BRIO_CH32_LEAF_VECTOR.
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
// ft8..ft11 = f28..f31) and fcsr stored on the user stack around the
// call, a frame of ninety-six bytes that keeps the stack's 16-byte
// alignment. One csrrw reads the interrupted program's fcsr into t0 (a
// register the hardware has saved) and writes zero, so the body starts
// from fcsr's reset state - round to nearest, no flag raised - whatever
// the program it interrupted had set (the header says why), and the
// program finds its own rounding mode and flags again after MRET.
#define BRIO_CH32_VECTOR_FLOAT(name)                                                       \
    extern "C" void name##_body();                                                         \
    extern "C" [[gnu::naked, gnu::no_icf]] void name() {                                   \
        __asm__("addi sp, sp, -96\n\t"                                                     \
                "fsw ft0, 0(sp)\n\t"  "fsw ft1, 4(sp)\n\t"  "fsw ft2, 8(sp)\n\t"           \
                "fsw ft3, 12(sp)\n\t" "fsw ft4, 16(sp)\n\t" "fsw ft5, 20(sp)\n\t"          \
                "fsw ft6, 24(sp)\n\t" "fsw ft7, 28(sp)\n\t" "fsw fa0, 32(sp)\n\t"          \
                "fsw fa1, 36(sp)\n\t" "fsw fa2, 40(sp)\n\t" "fsw fa3, 44(sp)\n\t"          \
                "fsw fa4, 48(sp)\n\t" "fsw fa5, 52(sp)\n\t" "fsw fa6, 56(sp)\n\t"          \
                "fsw fa7, 60(sp)\n\t" "fsw ft8, 64(sp)\n\t" "fsw ft9, 68(sp)\n\t"          \
                "fsw ft10, 72(sp)\n\t" "fsw ft11, 76(sp)\n\t"                              \
                "csrrw t0, fcsr, zero\n\t"                                                  \
                "sw t0, 80(sp)\n\t"                                                         \
                "call " #name "_body\n\t"                                                  \
                "lw t0, 80(sp)\n\t"                                                         \
                "csrw fcsr, t0\n\t"                                                         \
                "flw ft0, 0(sp)\n\t"  "flw ft1, 4(sp)\n\t"  "flw ft2, 8(sp)\n\t"           \
                "flw ft3, 12(sp)\n\t" "flw ft4, 16(sp)\n\t" "flw ft5, 20(sp)\n\t"          \
                "flw ft6, 24(sp)\n\t" "flw ft7, 28(sp)\n\t" "flw fa0, 32(sp)\n\t"          \
                "flw fa1, 36(sp)\n\t" "flw fa2, 40(sp)\n\t" "flw fa3, 44(sp)\n\t"          \
                "flw fa4, 48(sp)\n\t" "flw fa5, 52(sp)\n\t" "flw fa6, 56(sp)\n\t"          \
                "flw fa7, 60(sp)\n\t" "flw ft8, 64(sp)\n\t" "flw ft9, 68(sp)\n\t"          \
                "flw ft10, 72(sp)\n\t" "flw ft11, 76(sp)\n\t"                              \
                "addi sp, sp, 96\n\t"                                                      \
                "mret");                                                                   \
    }                                                                                      \
    extern "C" void name##_body()
// The leaf form: the attributed handler of the hardware prologue even
// here, for a body that calls nothing and touches no f-register, which
// then saves nothing in software and needs no trampoline. The absolute
// symbol `__brio_leaf_<name>` is the binding's mark in the image, by
// which the vector guard holds the body to that promise (it costs no
// byte of the image: an absolute local symbol lives in the symbol table
// alone).
#define BRIO_CH32_LEAF_VECTOR(name)                              \
    __asm__(".set __brio_leaf_" #name ", 1");                    \
    extern "C" BRIO_CH32_HANDLER_ATTRIBUTE void name()
#else
#define BRIO_CH32_VECTOR(name) extern "C" BRIO_CH32_HANDLER_ATTRIBUTE void name()
#define BRIO_CH32_VECTOR_FLOAT(name) BRIO_CH32_VECTOR(name)
#define BRIO_CH32_LEAF_VECTOR(name) BRIO_CH32_VECTOR(name)
#endif

namespace brio {

/// mstatus.MIE, the one global interrupt enable.
inline constexpr uint32_t mstatus_mie = 1UL << 3;

/// Is the global enable set? One CSR read, any context.
inline bool interrupts_enabled() {
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    return (mstatus & mstatus_mie) != 0u;
}

/// Set the global enable: what a program calls once in main(), after
/// every driver's init() and before the kernel runs. One instruction.
inline void enable_interrupts() {
    __asm__ volatile("csrsi mstatus, 8" ::: "memory");
}

/// Clear the global enable. One instruction; it saves nothing, so a
/// section that must restore what it found takes InterruptGuard.
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

/// The interrupt controller's per-line enables. Monostate (a type with
/// only static members, never instantiated): there is one PFIC, and its
/// two registers are write-one, so no verb here needs a guard around it.
/// Every verb is one store or one load, legal from any context.
struct Pfic {
    Pfic() = delete;

    /// Enable a line - as ticker.hpp's init() enables its own, whose
    /// vector the program binds. Write-one.
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

    /// Whether the caller runs inside an interrupt handler: PFIC_GISR's
    /// GACTSTA, bit 8 of the register at offset 0x4C, which the chapter
    /// describes as an interrupt being executed, whatever the nesting level
    /// (the reference manual 9.5.2.11 and the QingKe V4 manual 3.1: the
    /// same register and the same bit on the V4B and the V4F). Not
    /// mstatus.MIE, which a critical section in thread code clears as a
    /// trap's entry does, so it cannot tell a handler from a guard. A
    /// dispatch the preemptive kernel starts at a handler's exit runs in
    /// thread mode, after the trap's MRET (ch32vx03/schedule.hpp), where
    /// the bit is expected clear: the manual's reading, not a measurement
    /// (docs/ch32vx03/platform.md). The platform's in_handler()
    /// (kernel/platform.hpp's optional member) is this read. Any context;
    /// one load and a test.
    static bool in_handler() { return (pfic()->GISR & pfic_gisr_gactsta) != 0u; }
};

} // namespace brio
