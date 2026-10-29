/*
 * pfic.hpp - the CORE stratum: interrupt control on a QingKe core.
 *
 * The global interrupt enable, the critical section the kernel masks
 * with, the per-line enables of the interrupt controller (the PFIC,
 * WCH's own), and the three macros that bind a vector - written once for
 * the three QingKe families brio runs on: the CH32V00x (the QingKe V2C
 * of the CH32V006, the V2A of the CH32V003), the CH32V203 and the
 * CH32V303 (the V4B and the V4F) and the CH32X035 (the V4C). A program
 * never includes this file: it includes its family's pfic.hpp
 * (ch32x035/pfic.hpp and its siblings), which is the family's device
 * header and then this file, or platform.hpp, which includes that. A
 * program meets the macros and the global enable:
 *
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }
 *     ...
 *     brio::enable_interrupts();
 *
 * and the rest from inside the drivers, as a ticker's init() enables its
 * line and its now() masks:
 *
 *     Pfic::enable(Irq::systick);
 *     InterruptGuard guard;
 *
 * Who calls what: enable_interrupts() and disable_interrupts() are one
 * instruction each and interrupts_enabled() one CSR read, legal from any
 * context; InterruptGuard may be declared in any context and nests,
 * masking interrupts from its declaration to the end of the enclosing
 * scope - for as long as that scope runs, which the guard does not
 * bound; every Pfic verb is one store into a write-one bank or one load
 * of a status bank, legal from any context with no guard. Nothing here
 * waits. enable_interrupts() runs once in main(), after the drivers'
 * init() and before the kernel runs.
 *
 * Two layers, like every target. `mstatus.MIE` is the RISC-V global mask
 * - one bit, all or nothing, the equivalent of the AVR's I flag and of
 * ARMv6-M's PRIMASK - and `InterruptGuard` is the kernel's
 * CriticalSection over it: read-and-clear on entry, restore the previous
 * value on exit, so guards nest. `csrrci` does the read and the clear in
 * one instruction, which is what makes the guard correct against an
 * interrupt arriving between the two halves. The QingKe V2 and V4 spell
 * the machine-mode CSRs the same way, which is why one file serves both.
 *
 * The per-line enables belong to the PFIC (WCH's register definitions for
 * the CH32V00x call it NVIC as well; brio uses the name the chapters
 * use). Its enable and disable registers are write-one: a write of 1 to a
 * bit acts, a 0 does nothing, so there is no read-modify-write and no
 * guard needed around it.
 *
 * THE VECTOR BINDING is one spelling for every image of the three
 * families. BRIO_CH32_VECTOR(name) { body } is what an app writes to bind
 * a vector: an `extern "C"` function under the crt's name for the vector
 * (the crt is the startup code that runs before main(), each project's
 * src/glue/startup_*.S), standing at file scope, one per vector the
 * program owns. Every handler the crt's table names is a weak alias of
 * its default handler, a spin loop, so a line enabled with no binding
 * stops the program there with no fault and no message. The macro
 * follows two facts of the build:
 *
 *  - THE CORE'S HARDWARE PROLOGUE/EPILOGUE (the HPE: the core itself
 *    saving and restoring registers around a handler), each project's
 *    option - CH32V00X_HPE, CH32VX03_HPE, CH32X035_HPE, chosen at
 *    configure and on by default in all three (the projects'
 *    CMakeLists.txt) - which the build passes to the compiler as
 *    BRIO_CH32_HPE. With it on, the crt has set INTSYSCR.HWSTKEN and the
 *    handler attribute is WCH's "WCH-Interrupt-fast": gcc emits no
 *    prologue for the registers the hardware saves and ends the function
 *    with MRET. On the QingKe V2 the hardware pushes the ten caller-saved
 *    registers of RV32E to the USER stack (QingKe V2 manual 3.4); on the
 *    V4 the sixteen caller-saved integer registers to a stack of its own
 *    (QingKe V4 manual 3.2, 3.4 note 3), two levels deep on the V4B and
 *    the V4C and three on the V4F (the V4 manual's table 1-1). With it
 *    off, the attribute is the plain `interrupt` and the handler saves
 *    what it clobbers itself. The two must agree: a fast handler under an
 *    HPE that is off returns into a program whose registers it
 *    clobbered. One macro, one option, no way to spell them apart.
 *  - WHETHER THE IMAGE IS COMPILED WITH THE F EXTENSION (the compiler's
 *    __riscv_flen): the CH32V303's QingKe V4F, the V4B's twin with a
 *    single-precision floating-point unit, exactly when the ch32vx03/
 *    project's part table (ch32vx03/cmake/ch32vx03-parts.cmake) gives
 *    the part an ISA that carries F. No other core here has F, and
 *    without it the three macros are one.
 *
 * With F and the HPE on (the CH32V303's images as the project builds
 * them) the vector is a NAKED TRAMPOLINE: two instructions, a call of an
 * ordinary function `name_body` holding the braces that follow, and
 * MRET. The reason is what the hardware prologue does not save - an
 * f-register, ever (V4 manual 3.4 note 3) - and what gcc does about it:
 * a function with the interrupt attribute that calls out saves every
 * f-register the ABI lets a callee clobber, the twenty ft0..ft11 and
 * fa0..fa7, without looking at the callee, so on that core one call
 * anywhere in a handler costs eighty bytes of stack and forty memory
 * operations on every entry (docs/ch32vx03/platform.md measures them).
 * A naked function gets no prologue from gcc at all, and the body, an
 * ordinary function under ilp32f, saves only what the ABI asks of a
 * callee - the callee-saved registers it uses, the return address if it
 * calls - and none of the caller-saved ones, the sixteen integers being
 * the hardware's. In the release listings those saves are gcc's ordinary
 * prologue at the body's entry (the I2C host's event body: ra, s0, s1),
 * and a body whose last act is a call ends in a tail jump that returns
 * straight to the trampoline (a serial port's receive body, and the
 * kernel's `post`), costing nothing more. The trampoline's own cost: the
 * call (one `jal`, two bytes compressed where the body is within reach,
 * four otherwise) and the body's `ret`, against no instruction at all in
 * an attributed leaf - measured on the CH32V303VCT6 (the platform suite's
 * letter e; a suite's letter is one command key of its console and the
 * group of verdicts it runs) two cycles on the way in and two to five on
 * the way out, where a handler that calls out sheds forty memory
 * operations. With F and the HPE off the binding stays the attributed
 * handler of the plain `interrupt` (gcc saves whatever a call might
 * clobber, f-registers included, in software): the f-register question is
 * then the software prologue's own, and a trampoline would buy nothing
 * there.
 *
 * Why a macro. What differs between the expansions is not a type or a
 * value but the shape of a declaration and the symbol a vector table
 * entry names - an attributed function called `name`, or a naked one
 * called `name` and a second function called `name_body` - and no
 * template names a symbol; a build option and an attribute are questions
 * only the preprocessor can ask. The trampoline's body is basic asm
 * alone, which is gcc's rule for a naked function.
 *
 * The trampoline's promise is enforced, not stated. Nothing saves the
 * f-registers of the program a trampoline interrupts, so its body and
 * everything the body reaches must use none: `brio check ch32vx03` walks
 * the call graph of every trampoline's body in every linked image of the
 * F presets, from the vector table's own entries, and fails on a
 * floating-point instruction anywhere on it, or on an indirect call it
 * cannot resolve, naming the vector and the function
 * (cli/vector_guard.py) - a check of the linked images that an ordinary
 * build does not run. A body that needs float binds with
 * BRIO_CH32_VECTOR_FLOAT(name) instead: the same trampoline with the
 * twenty caller-saved f-registers stored around the call - the set gcc
 * saves for a handler that calls out - and fcsr too, which gcc does not
 * save. The body starts from fcsr's reset state (round to nearest, no
 * flag), not from the interrupted program's: a handler's arithmetic is
 * then the same whatever it interrupts - a program that sets another
 * rounding mode for its own work does not change its handlers' results
 * with it - and the flags the body finds are the ones it raised; the swap
 * that clears fcsr is the instruction that reads it, so the choice costs
 * nothing over inheriting. The interrupted program gets its own rounding
 * mode and sticky flags back, whatever the body did to them. The guard
 * checks that trampoline's stores and loads, fcsr's among them, and
 * leaves its body alone; the platform suite's letter l judges both halves
 * on the CH32V303VCT6 (the body finds 0, the program finds its 0x20 and
 * its 0x81 after a body that raised NX). fcsr's four instructions cost
 * twelve cycles of round trip there, 123 against 111 in two images that
 * differ in them alone.
 *
 * A leaf needs no trampoline. A body that calls nothing and touches no
 * f-register saves nothing under the fast attribute, F or not, so the
 * trampoline's call and return are its whole cost there.
 * BRIO_CH32_LEAF_VECTOR(name) { body } binds it as the attributed handler
 * of the hardware prologue even under F (without F, or with the HPE off,
 * it is BRIO_CH32_VECTOR exactly, so those images do not move); measured
 * on the CH32V303VCT6 in two images that differ in the binding alone,
 * letter e's software interrupt 53..66 cycles of round trip against
 * 57..70 under the trampoline (entry 16 against 18, exit 25 against
 * 27..30), and the tick's edge to its handler's first statement 22 cycles
 * against 24 (letter w). The promise is the guard's too, both ways: a
 * vector bound leaf that gcc compiled with a call, a tail jump, an
 * indirect transfer other than a jump table into itself, or an
 * f-register touched fails the check, naming it; a trampoline whose body
 * came out a leaf is reported as a candidate for the leaf form (a line of
 * the check's output, not a failure). The binding's mark in the image is
 * an absolute symbol, `__brio_leaf_<name>`, which costs no byte of it.
 * And since the three macros are the only bindings the guard can hold to
 * a promise, any other entry of an F image's table - a handler attributed
 * by hand among them - fails the check too, the crt's default handler
 * alone excepted.
 *
 * THE ATTRIBUTED FORM FORBIDS GCC TO MERGE TWO HANDLERS. Two bindings
 * with the same body - one transport's two DMA channels, each vector
 * calling the same dma_isr() - are what gcc's identical code folding
 * looks for, and gcc folds them - upstream 16.2 and WCH's 15.2 alike, at
 * -Os and not at -O2, with either attribute - by turning the second into
 * a handler that calls the first: a call that never comes back, because
 * the first ends in MRET. Whatever the caller had put on the stack stays
 * there: nothing under the hardware prologue, where the merge would be
 * harmless by luck, the caller-saved registers under a plain `interrupt`,
 * and on the V4F the twenty f-registers it saves before any call, eighty
 * bytes - the interrupted program resuming with its stack pointer that
 * far below its frame (measured on the CH32V303VCT6: the next return
 * jumped to a saved float). `no_icf` keeps every attributed handler
 * whole, the one spelling that holds under both attributes and both ABIs.
 * A trampoline needs no such guard: no two of them are alike (each calls
 * its own body), and two bodies that fold are ordinary functions, for
 * which a fold is an alias or a jump that returns where it should; it
 * carries `no_icf` all the same, so that no reading of the rule depends
 * on the asm text differing.
 *
 * EVERY BINDING IS ALIGNED TO FOUR. gcc aligns a function to two bytes
 * under -Os, so without the attribute a vector's cost moved with whatever
 * the image put before it. From the CH32X035's flash, code with taken
 * branches costs what its address modulo four makes it (the fetch reads a
 * word; docs/ch32x035/platform.md, "A handler in SRAM"), by up to ten
 * cycles on the USART's completion there. On the other two families the
 * figures are smaller and point the same way, each measured in two
 * images of the platform suite that differ in the alignment alone, ten
 * runs each: on the CH32V006K8U6 the software interrupt's round trip 78
 * cycles against 81 (letter e) and the console's USART handler's 123
 * against 129 (letter u); on the CH32V203C8T6 the round trip 52 against
 * 56, and on the CH32V303VCT6 the round trip 52 to 62 against 53 to 63
 * and the float letter's three 48, 125 and 66 against 49, 126 and 66.
 * The price is the padding - two bytes a binding at most, 4 to 28 bytes
 * an image across the CH32V00x project's images, up to 36 across the
 * CH32V203's and 60 across the CH32V303's, whose trampolines are aligned
 * too - and what the alignment does not fix: where a function's own
 * branch targets fall in their words, which the V4B and the V4F price by
 * a cycle or two a taken branch (docs/ch32vx03/platform.md). In the
 * same two images, built while its scan still called each queue's
 * empty() out of line, the preemptive kernel's suite on the CH32V303VCT6 read
 * road 1 at 144 cycles against 140 to 141 and a road-2 post at 344 to 347
 * against 338 to 342 - measured, and explained no further than this:
 * the trampoline bodies behind the vectors moved with them (the timer's
 * from two modulo four to zero), and their branch targets with them.
 *
 * A vector placed in a section of its own (`.fast_text`, on the two
 * families whose scripts place it: docs/ch32v00x/platform.md and
 * docs/ch32x035/platform.md, "A handler in SRAM") is declared with the
 * section first and then bound:
 *
 *     extern "C" [[gnu::section(".fast_text")]] void usart4_handler();
 *     BRIO_CH32_VECTOR(usart4_handler) { ... }
 *
 * and gcc merges the declaration's section into the definition: an
 * attribute cannot be written in front of the macro, whose expansion
 * begins with `extern "C"`.
 *
 * What is not here. The V4 cores can nest interrupts - two levels on the
 * V4B and the V4C, up to eight on the V4F, chosen in INTSYSCR with the
 * priority bytes in IPRIOR - and WCH's own startup files turn that on.
 * brio does not, as a rule: the kernel's rule (docs/design/kernel.md
 * section 1, "The model in one page") is that an ISR body runs to
 * completion, on every target, so the crts leave INTSYSCR.INESTEN clear
 * and no driver writes a priority byte, and the peripheral handlers never
 * nest among themselves. The one program that turns nesting on is a
 * Rubato one on a V4 family, through qingke/schedule.hpp's
 * scheduler_init(): its scheduling request, the software interrupt, is
 * put one preemption level below every peripheral line, which still share
 * one level; that file owns the write to INTSYSCR and to that one
 * priority byte. The free vectored entries - two on the V2 (VTFIDR and
 * VTFADDR), four on the V4 (VTFIDR and VTFADDRR) - are a real feature of
 * these cores with no user yet; they arrive with one.
 *
 * Include-order contract: the family's device header first - it defines
 * `pfic()`, `PficRegs` and the family's `Irq` - which the family's
 * pfic.hpp does, then this file.
 */

#pragma once

#include <stdint.h>

static_assert(sizeof(brio::PficRegs) != 0u,
              "qingke/pfic.hpp: include the family's device header first (the family's pfic.hpp does) - "
              "pfic(), PficRegs and Irq are its");

/// The handler attribute of every binding (the file header): WCH's fast
/// attribute under the hardware prologue, gcc's plain one without it,
/// `no_icf` and the alignment to four either way.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
#define BRIO_CH32_HANDLER_ATTRIBUTE [[gnu::interrupt("WCH-Interrupt-fast"), gnu::no_icf, gnu::aligned(4)]]
#else
#define BRIO_CH32_HANDLER_ATTRIBUTE [[gnu::interrupt, gnu::no_icf, gnu::aligned(4)]]
#endif

/// The binding of every vector (see the file header):
/// `BRIO_CH32_VECTOR(usart1_handler) { Serial::isr(); }` at file scope -
/// the one place the handler's shape is chosen, from the hardware
/// prologue option and the image's floating-point ABI - with its float
/// sibling BRIO_CH32_VECTOR_FLOAT and its leaf sibling
/// BRIO_CH32_LEAF_VECTOR.
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE && defined(__riscv_flen)
// The naked trampoline: the hardware has saved the sixteen caller-saved
// integer registers (ra among them, which the call overwrites and MRET
// puts back), the body saves what a callee must, MRET returns.
#define BRIO_CH32_VECTOR(name)                                         \
    extern "C" void name##_body();                                     \
    extern "C" [[gnu::naked, gnu::no_icf, gnu::aligned(4)]] void name() { \
        __asm__("call " #name "_body\n\t"                               \
                "mret");                                               \
    }                                                                  \
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
    extern "C" [[gnu::naked, gnu::no_icf, gnu::aligned(4)]] void name() {                 \
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
 * The kernel's CriticalSection on these cores: interrupts masked from the
 * point it is declared to the end of the enclosing scope. Any context,
 * nests.
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
/// enable and disable registers are write-one, so no verb here needs a
/// guard around it. Every verb is one store or one load, legal from any
/// context.
struct Pfic {
    Pfic() = delete;

    /// Enable a line at the controller (IENR, write-one). A driver's
    /// init() enables its own lines, as a ticker's init() does; a line no
    /// driver opens - an EXTI pin group, or an ADC whose init() leaves its
    /// line closed - is the program's to enable.
    static void enable(Irq irq) {
        const uint32_t n = static_cast<uint32_t>(irq);
        pfic()->IENR[n >> 5] = 1UL << (n & 0x1fu);
    }

    /// Disable a line at the controller (IRER, write-one).
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
    /// GACTSTA, bit 8 of the register at offset 0x4C, which each chapter
    /// describes as an interrupt being executed, whatever the nesting
    /// level - the same register and the same bit on every part here (the
    /// CH32V00X reference manual 6.5.2.7 for the CH32V006, the CH32V003's
    /// own 6.5.2.7 and the QingKe V2 manual 3.1; the CH32V203 and
    /// CH32V303 reference manual 9.5.2.11 and the QingKe V4 manual 3.1;
    /// the CH32X035 reference manual 7.5.2.11). Not mstatus.MIE, which
    /// says nothing of a handler on these cores: interrupt entry leaves it
    /// set (the QingKe V2 manual 2.2, docs/ch32v00x/platform.md; the
    /// QingKe V4 manual 2.2, until the last level of nesting), and a
    /// critical section in thread code clears it.
    /// A dispatch the preemptive kernel starts at a handler's exit runs in
    /// thread mode, after the trap's MRET (qingke/schedule.hpp), where the
    /// bit is expected clear: the manual's reading, not a measurement
    /// (docs/ch32vx03/platform.md). The platform's in_handler()
    /// (kernel/platform.hpp's optional member) is this read. Any context;
    /// one load and a test.
    static bool in_handler() { return (pfic()->GISR & pfic_gisr_gactsta) != 0u; }
};

} // namespace brio
