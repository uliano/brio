/*
 * schedule.hpp - the CORE stratum: the scheduling request of the
 * preemptive kernel on the QingKe V4, and the thread-mode scheduler it
 * starts.
 *
 * What a QingKe V4 platform gives Rubato (kernel/rubato.hpp) beyond what
 * Tenuto takes from it: a way to ask for the scheduler, and the place the
 * scheduler then runs. kernel/platform.hpp states the contract - the
 * request, pend_schedule(), runs the scheduler at once when it is made
 * from a dispatch and at the handler's exit when it is made from an
 * interrupt handler - and this file is how the CH32V203 and the CH32V303
 * (the QingKe V4B and V4F) and the CH32X035 (the V4C) keep it. A program
 * never includes this file: the family's schedule.hpp defines the one
 * fact of its core this file asks (`scheduling_request_posted`, below)
 * and includes it, and the family's platform.hpp includes that. The
 * family's Rubato platform calls the request; a program binds the two
 * vectors this file needs to the bodies below, exactly as it binds every
 * other vector, and turns the nesting on once:
 *
 *     using P = brio::Ch32vx03RubatoPlatform<>;   // or Ch32x035RubatoPlatform<>
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
 *     BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }
 *     ...
 *     brio::scheduler_init();        // in main(), before interrupts are enabled
 *     K::run();
 *
 * Both vectors are NAKED: the bodies are assembly that moves the stack
 * pointer and returns from the trap itself, and a compiler-made prologue
 * would put a frame where they expect none. The bodies are forced inline,
 * so the vector's function is that assembly and nothing else, in a
 * release build and in a debug one. BRIO_CH32_SCHEDULER_VECTOR is the
 * binding: naked, `no_icf` and aligned to four like every binding of the
 * stratum (qingke/pfic.hpp says why), and marked a leaf in the image
 * (`__brio_leaf_<name>`, an absolute symbol that costs no byte), which is
 * what it is under the hardware prologue - no call, no f-register,
 * nothing saved in software - and what the CH32V303's vector guard then
 * holds it to in every image built with F (cli/vector_guard.py): neither
 * body calls anything, and the scheduler they lead to runs in thread
 * mode, outside every trap.
 *
 * THE REQUEST IS THE SOFTWARE INTERRUPT, line 14 of the vector table
 * (the QingKe V4 manual table 3-1; the CH32X035 reference manual 7.3,
 * table 7-1), pended by a store into the first pending-set register
 * (PFIC_IPSR1 in the CH32X035 reference manual 7.5.2.25; the V4 manual
 * 3.1: "set directly through the interrupt pending register to trigger
 * into the interrupt"), the PFIC's own way in for every line
 * (qingke/pfic.hpp's set_pending). The pending bit falls when the line is
 * taken (measured: the handler reads it clear), so nothing withdraws it.
 * What follows the store is a fact of the core, and the family states it
 * as `scheduling_request_posted`:
 *
 *  - ON THE V4B AND THE V4F THE STORE IS POSTED and the core runs on:
 *    measured on the CH32V203C8T6 and the CH32V303VCT6, three
 *    instructions after the store retire before the trap, two behind a
 *    `fence` or a load of the PFIC, and NONE behind a `fence.i` - the
 *    instruction the request ends with there;
 *  - THE V4C TAKES THE TRAP AT THE INSTRUCTION AFTER THE STORE WHEN THE
 *    STORE RUNS FROM THE FLASH: measured on the CH32X035F8U6 with
 *    hand-placed assembly reading mepc in the handler, the store at one
 *    address and mepc the next, the same after the `csrsi` that unmasks
 *    a pending request, and the same with a `fence.i` after the store
 *    (mepc the fence itself). There the request is the store and nothing
 *    else. The same store run from SRAM is posted like the V4B's: three
 *    compressed instructions retire behind it (two with the branch
 *    predictor off) and two behind the unmasking `csrsi`, measured on the
 *    same part with the sequence placed in SRAM, a `fence.i` after the
 *    store closing them for two cycles. The fact is the family's to state
 *    because it is a fact of where the family runs the request, not of
 *    the core alone - and the CH32X035 runs it from either memory, as the
 *    image's build line chooses, so its value follows that choice
 *    (ch32x035/schedule.hpp).
 *
 * Either way a request made from thread mode with MIE set is taken before
 * the instruction that follows it: that is what "at once" means for a
 * request made from a dispatch. A request made with MIE clear waits for
 * the unmask, so no dispatch ever starts inside a critical section; a
 * post made inside one serves its receiver when the section ends - on the
 * V4F, and on the V4C where the `csrsi` that ends it runs from the flash,
 * at that `csrsi` (on the CH32X035F8U6 at 48 MHz, from the section's last
 * statement to the receiver's first, 184 cycles with the scheduler path
 * in SRAM and 405 with it in the flash, test_x035_rubato's letter g,
 * whose section is in the flash in both images); on the V4B two
 * instructions after it (measured: the same pend taken after two
 * instructions behind the `csrsi`, none behind a `fence.i` there), and on
 * the V4C the same two where the `csrsi` runs from SRAM (measured with
 * the sequence placed in SRAM) - which the platform's critical section
 * does not pay a `fence.i` on every exit to close.
 *
 * NESTING, AT RUN TIME. The crt leaves INTSYSCR.INESTEN clear (an ISR body
 * runs to completion: docs/design/kernel.md section 1), and
 * scheduler_init() sets it, with the one preemption bit (PMTCFG = 01, two
 * levels), and puts the software interrupt's priority byte at 0x80: the
 * preemption bit set, so below every other line, which stay at the reset
 * priority 0. Peripheral handlers then still never nest among themselves
 * - one preemption level is theirs, and two lines of one level do not
 * preempt each other (the V4 manual 3.3) - and every one of them
 * interrupts the software interrupt's handler. Measured on the V4B and
 * the V4F: a line of priority 0 raised inside the software interrupt's
 * handler is taken there (PFIC_GISR's NESTSTA reads 0x03, the second
 * level), a line at 0x00 with the software interrupt at 0x00 too is not,
 * and the handler's mepc, mcause and mstatus read the same after the
 * nested level as before it - the controller keeps them per level, which
 * is what lets the handler below read and write mepc while a peripheral
 * line may nest over it. The V4B of the CH32V203 implements two nesting
 * levels (the V4 manual table 1-1) and ignores PMTCFG's write (INTSYSCR
 * reads 0x3 after 0x7), its nesting fixed at the one preemption bit; the
 * V4C of the CH32X035 implements the same two levels and a hardware
 * prologue two deep (table 1-1, the V4B's figures) and ignores the write
 * as the V4B does (INTSYSCR reads 0x3 after 0x7, measured by
 * test_x035_platform's letter i; 0x2 in an image built without the
 * prologue); the V4F of the CH32V303 implements eight and takes the write
 * (it reads 0x8003, PMTSTA saying one preemption bit), so the one store
 * asks the same of all three. Two levels are what the design needs and
 * all it can reach: the scheduler's trap and one peripheral handler over
 * it, inside the hardware prologue's depth on each (two on the V4B and
 * the V4C, three on the V4F). The environment call that ends a
 * scheduler's run is an exception and no line preempts it (measured on
 * the V4B and the V4F: a line pended inside its handler is taken after
 * its MRET). test_x035_rubato's letters a and e show the nesting at work
 * on the V4C: an interrupt's post preempts a dispatch at depth one and at
 * depth two.
 *
 * WHERE THE SCHEDULER RUNS: IN THREAD MODE. The software interrupt does
 * not run the scheduler; it makes the core return from the trap INTO it.
 * scheduler_trap<K>() moves the stack pointer down sixteen bytes, stores
 * there the interrupted code's mepc and mstatus, writes the entry's
 * address into mepc and executes MRET:
 *
 *     higher addresses
 *       the interrupted code's stack
 *       (two words unused)           the 16-byte block keeps the ABI's
 *       mstatus of the trap          alignment of the stack pointer
 *       mepc of the trap        <- sp at the entry
 *     lower addresses
 *
 * THE PER-LEVEL SAVE IS THE MAIN STACK, not an array of the kernel's
 * indexed by a depth: each preemption adds one level, as deep as the
 * pack's chain of priorities, and the stack is where the nesting already
 * keeps its order, so no bound is chosen and no index is kept - the same
 * two words below the frame the Cortex-M realization pushes
 * (cortexm/schedule.hpp). With the hardware prologue (the projects'
 * CH32VX03_HPE and CH32X035_HPE options, the default) the MRET pops the
 * sixteen caller-saved integer registers the hardware pushed at the
 * trap's entry (the V4 manual 3.4), so the entry, scheduler_thread<K>(),
 * lands in machine mode with the trap no longer active and the
 * INTERRUPTED CODE'S registers in the register file (measured on the V4B
 * and the V4F: sixteen patterns set before the request, all sixteen found
 * at the entry, all sixteen found again after the return). The entry
 * stores those sixteen on the stack - x1, x5..x7, x10..x17, x28..x31, the
 * set the hardware saves - sets MIE, calls K::activate(), loads them back
 * and executes `ecall`. scheduler_return(), the machine-mode environment
 * call's vector (line 5 of the table), loads mepc and mstatus from the
 * two words, moves the stack pointer back up over the block and executes
 * MRET: the hardware's push at the ecall and its pop at this MRET pair up
 * around the exit, and the interrupted code resumes with its registers,
 * its mstatus and its stack pointer as they were. Without the hardware
 * prologue (the option OFF) the same shape holds with the two bodies' one
 * scratch register, t0, saved and restored in the block's third word by
 * the bodies themselves: nothing else has been pushed, so the MRET has
 * nothing else to pop. s0..s11 and sp never leave the register file:
 * everything between the two traps is code that keeps them.
 *
 * THE UNMASK IN THE ENTRY is not decoration. Entry does not clear MIE on
 * these cores (the V4 manual 2.2), so the trap normally returns into the
 * entry with MIE set; but on the V4B an interrupt racing the platform
 * guard's `csrrci` is taken with mepc past it and MPIE clear
 * (docs/ch32vx03/platform.md, the mask's shadow), and the MRET then puts
 * MIE at 0. The entry sets it, so the dispatches run unmasked, and the
 * exit's MRET puts the section's own mask back with the saved mstatus.
 * The scheduler is then the work the request asked for before the section
 * began.
 *
 * THE FLOATING-POINT UNIT (an image built with F, the CH32V303's: the
 * compiler's __riscv_flen; the V4B and the V4C have none, RV32IMAC, and
 * there the entry has no f-registers to save and the file no second
 * shape). The hardware prologue saves no f-register (the V4 manual 3.4,
 * note 3), so the entry saves them: when mstatus.FS is Dirty (both bits
 * of 14:13, the V4 manual 8.2) the interrupted code has written an
 * f-register since its state was last saved, and before the call the
 * entry stores the twenty the ABI lets a callee clobber - ft0..ft11 and
 * fa0..fa7, the set BRIO_CH32_VECTOR_FLOAT stores (qingke/pfic.hpp) - and
 * fcsr, a frame of 96 bytes, then writes FS = Clean; after the call it
 * loads them back, fcsr first. fs0..fs11 are not in the frame for the
 * reason s0..s11 are not: everything the scheduler runs is code that
 * keeps them. Any other state means nothing has been written since a save
 * made it Clean - the dispatch the entry preempts has done no float work
 * - and the entry skips both halves; a dispatch that writes an f-register
 * makes the unit Dirty again for the next entry over it. The exit's
 * mstatus puts the interrupted code's own FS back. fcsr is the program's,
 * as under Tenuto, not an object's: a preempting dispatch starts from the
 * rounding mode it preempts, and the preempted one finds its own mode and
 * flags again. Measured on the CH32V303VCT6 (test_vx03_rubato's letter
 * j): sums held in f-registers bit-exact across preemptions, a float
 * handler's among them, flags raised by the preempting dispatch not seen
 * by the preempted one, and the save's price 27 to 30 cycles on the way
 * in and 29 on the way out. The save lives here, in thread mode,
 * and never in a vector's body: the vector guard (cli/vector_guard.py)
 * holds every vector of an F image to touching no f-register, and the
 * scheduler's two vectors touch none.
 *
 * WHAT THIS BUYS. While the dispatches run, the software interrupt is NOT
 * active: an interrupt handler's post pends it again, and it is taken at
 * that handler's exit, over whatever dispatch was running, at any depth -
 * the kernel's road 1. A dispatch's post to an object above it pends it
 * from thread mode, taken at once, inside post() - road 2. Each level of
 * nesting costs, on the stack, the block (16 bytes), the entry's sixteen
 * registers (64), the f-registers' frame where the entry saves them (96,
 * the V4F alone) and the scheduler's own frames.
 *
 * RE-ENTRANCY. The software interrupt can be pended at any moment and the
 * shapes all close: pended while a dispatch runs, it nests one level
 * more; pended while the entry stores its registers or between
 * activate()'s return and the `ecall`, it nests over the entry itself,
 * whose `ecall` then runs after the nested level has unwound; pended by a
 * peripheral handler nested over the software interrupt's own handler, it
 * is taken at the instruction after that handler's MRET - the entry's
 * first - and builds its block below the first. The block is written
 * after the stack pointer has moved, so a handler nested over the surgery
 * puts its frame below it.
 *
 * WHAT THIS FILE ASSUMES. Every line but the software interrupt stays at
 * priority 0 (no driver of the three strata writes PFIC_IPRIOR); nothing
 * but this file writes the software interrupt's priority or pends it; the
 * program executes no `ecall` of its own - ecall_m_handler is this
 * file's; and the program runs in machine mode, on one stack.
 *
 * WHICH CORES. The QingKe V4B, V4F and V4C: the same PFIC, the same
 * INTSYSCR and the same hardware prologue (the V4 manual covers the
 * series), the request's one difference stated by the family. The QingKe
 * V2 of the CH32V00x differs where this file leans: its hardware prologue
 * pushes ten registers to the USER stack rather than to an internal one
 * (docs/ch32vx03/platform.md), so the MRET into the entry would pop that
 * frame from below the block, and its RV32E has sixteen registers; the
 * shape would carry over with the frame's layout read from that core's
 * manual, and nothing of it has been measured there - the CH32V00x
 * family does not include this file.
 *
 * Include-order contract: the family's device header and its pfic.hpp
 * first, and `scheduling_request_posted` and `scheduler_vector_table_init()`
 * defined - the family's step of scheduler_init(), which keeps the vector
 * table where the family's image runs its scheduler path from - which the
 * family's schedule.hpp does before including this file.
 */

#pragma once

#include <stdint.h>

#include <type_traits>

#include "qingke/pfic.hpp"

static_assert(std::is_same_v<decltype(brio::scheduling_request_posted), const bool>,
              "qingke/schedule.hpp: the family's schedule.hpp defines "
              "`inline constexpr bool scheduling_request_posted` before including this file - "
              "true where the request's store is posted and must end with a fence.i");

/// The binding of the scheduler's two vectors (the file header): naked,
/// kept whole against identical code folding, aligned to four like every
/// vector of the stratum, marked a leaf in the image so that the vector
/// guard holds the body to being one, and given a section of its own,
/// `.text.brio_scheduler_vector.<name>`, by which a family's linker script
/// can place the scheduler's vectors apart from a handler a program binds
/// to the same line with BRIO_CH32_VECTOR (the CH32X035's does, for its
/// `scheduler = ram` images: ch32x035/ld/scheduler_ram/). Every other
/// script takes it with the rest of `.text.*`.
#define BRIO_CH32_SCHEDULER_VECTOR(name)                                  \
    __asm__(".set __brio_leaf_" #name ", 1");                             \
    extern "C" [[gnu::naked, gnu::no_icf, gnu::aligned(4),                \
                 gnu::section(".text.brio_scheduler_vector." #name)]] void name()

namespace brio {

/// INTSYSCR (CSR 0x804, QingKe V4 manual 3.2): the nesting enable and the
/// nesting-depth field's code for two levels, one preemption bit.
inline constexpr uint32_t intsyscr_inesten = 1UL << 1;
inline constexpr uint32_t intsyscr_pmtcfg_two_levels = 1UL << 2;

/// The software interrupt's priority byte: the preemption bit set, below
/// every line at the reset priority 0.
inline constexpr uint8_t scheduler_priority = 0x80;

/// The request: the software interrupt pended, taken before the next
/// instruction in thread mode with MIE set, at the handler's exit in a
/// handler - behind a `fence.i` where the family's core posts the store
/// (`scheduling_request_posted`, the file header), the store alone where
/// it does not.
///
/// A CALL THE COMPILER CANNOT SEE INTO, on purpose: noinline and noipa.
/// From a dispatch the request runs other dispatches before it returns,
/// through a trap the compiler does not know of, so to the caller it must
/// be a call to code that may read and write any static of the program -
/// which an out-of-line function with no interprocedural summary is. The
/// price is one call and return on the asking path only; a post that
/// asks nothing never reaches it. The store is volatile and the function
/// opaque, so nothing of the caller's moves across it.
[[gnu::noinline, gnu::noipa]] inline void request_scheduler() {
    pfic()->IPSR[0] = 1UL << static_cast<uint8_t>(Irq::software);
    if constexpr (scheduling_request_posted) {
        __asm__ volatile("fence.i" ::: "memory");
    }
}

/// The software interrupt below every other line, nesting on, the line
/// enabled - and first the family's own step, scheduler_vector_table_init()
/// (on the CH32X035 an image that runs the scheduler path from SRAM moves
/// the vector table there; elsewhere it does nothing). Call once in main(),
/// before interrupts are enabled.
inline void scheduler_init() {
    scheduler_vector_table_init();
    pfic()->IPRIOR[static_cast<uint8_t>(Irq::software)] = scheduler_priority;
    __asm__ volatile("csrs 0x804, %0" ::"r"(intsyscr_inesten | intsyscr_pmtcfg_two_levels)
                     : "memory");
    Pfic::enable(Irq::software);
}

/// The thread-mode entry the trap returns into (the file header): the
/// sixteen caller-saved integer registers stored (and in an image with F
/// the twenty caller-saved f-registers and fcsr, when FS is Dirty), MIE
/// set, the scheduler, the registers loaded back, and the way out through
/// `ecall`. Naked: it owns no frame but the one it builds, so the stack
/// pointer at its `ecall` is the one it started with, and it touches no
/// register the interrupted code keeps.
template <class Kernel>
[[gnu::naked, gnu::noinline]] void scheduler_thread() {
    __asm__ volatile(
        "addi sp, sp, -64\n\t"
        "sw ra, 0(sp)\n\t"   "sw t0, 4(sp)\n\t"   "sw t1, 8(sp)\n\t"   "sw t2, 12(sp)\n\t"
        "sw a0, 16(sp)\n\t"  "sw a1, 20(sp)\n\t"  "sw a2, 24(sp)\n\t"  "sw a3, 28(sp)\n\t"
        "sw a4, 32(sp)\n\t"  "sw a5, 36(sp)\n\t"  "sw a6, 40(sp)\n\t"  "sw a7, 44(sp)\n\t"
        "sw t3, 48(sp)\n\t"  "sw t4, 52(sp)\n\t"  "sw t5, 56(sp)\n\t"  "sw t6, 60(sp)\n\t"
#if defined(__riscv_flen)
        // The interrupted code's floating-point state, when it has one:
        // FS Dirty (both bits of mstatus 14:13) - the twenty caller-saved
        // f-registers and fcsr stored, FS then set Clean, the scheduler,
        // and the same loaded back; any other state - nothing written
        // since the last save made it Clean - the scheduler alone.
        "lui t1, 6\n\t"
        "csrr t0, mstatus\n\t"
        "and t0, t0, t1\n\t"
        "bne t0, t1, 1f\n\t"
        "addi sp, sp, -96\n\t"
        "fsw ft0, 0(sp)\n\t"   "fsw ft1, 4(sp)\n\t"   "fsw ft2, 8(sp)\n\t"   "fsw ft3, 12(sp)\n\t"
        "fsw ft4, 16(sp)\n\t"  "fsw ft5, 20(sp)\n\t"  "fsw ft6, 24(sp)\n\t"  "fsw ft7, 28(sp)\n\t"
        "fsw fa0, 32(sp)\n\t"  "fsw fa1, 36(sp)\n\t"  "fsw fa2, 40(sp)\n\t"  "fsw fa3, 44(sp)\n\t"
        "fsw fa4, 48(sp)\n\t"  "fsw fa5, 52(sp)\n\t"  "fsw fa6, 56(sp)\n\t"  "fsw fa7, 60(sp)\n\t"
        "fsw ft8, 64(sp)\n\t"  "fsw ft9, 68(sp)\n\t"  "fsw ft10, 72(sp)\n\t" "fsw ft11, 76(sp)\n\t"
        "frcsr t0\n\t"
        "sw t0, 80(sp)\n\t"
        "lui t1, 2\n\t"
        "csrc mstatus, t1\n\t"
        "csrsi mstatus, 8\n\t"
        "call %0\n\t"
        "lw t0, 80(sp)\n\t"
        "fscsr t0\n\t"
        "flw ft0, 0(sp)\n\t"   "flw ft1, 4(sp)\n\t"   "flw ft2, 8(sp)\n\t"   "flw ft3, 12(sp)\n\t"
        "flw ft4, 16(sp)\n\t"  "flw ft5, 20(sp)\n\t"  "flw ft6, 24(sp)\n\t"  "flw ft7, 28(sp)\n\t"
        "flw fa0, 32(sp)\n\t"  "flw fa1, 36(sp)\n\t"  "flw fa2, 40(sp)\n\t"  "flw fa3, 44(sp)\n\t"
        "flw fa4, 48(sp)\n\t"  "flw fa5, 52(sp)\n\t"  "flw fa6, 56(sp)\n\t"  "flw fa7, 60(sp)\n\t"
        "flw ft8, 64(sp)\n\t"  "flw ft9, 68(sp)\n\t"  "flw ft10, 72(sp)\n\t" "flw ft11, 76(sp)\n\t"
        "addi sp, sp, 96\n\t"
        "j 2f\n"
        "1:\n\t"
        "csrsi mstatus, 8\n\t"
        "call %0\n"
        "2:\n\t"
#else
        "csrsi mstatus, 8\n\t"
        "call %0\n\t"
#endif
        "lw ra, 0(sp)\n\t"   "lw t0, 4(sp)\n\t"   "lw t1, 8(sp)\n\t"   "lw t2, 12(sp)\n\t"
        "lw a0, 16(sp)\n\t"  "lw a1, 20(sp)\n\t"  "lw a2, 24(sp)\n\t"  "lw a3, 28(sp)\n\t"
        "lw a4, 32(sp)\n\t"  "lw a5, 36(sp)\n\t"  "lw a6, 40(sp)\n\t"  "lw a7, 44(sp)\n\t"
        "lw t3, 48(sp)\n\t"  "lw t4, 52(sp)\n\t"  "lw t5, 56(sp)\n\t"  "lw t6, 60(sp)\n\t"
        "addi sp, sp, 64\n\t"
        "ecall\n\t"
        "j .\n\t"            // never reached: the ecall's vector returns into the interrupted code
        ::"i"(&Kernel::activate));
}

/// The body the program binds to software_handler: the block with the
/// trap's mepc and mstatus, then the return into scheduler_thread<K>().
/// Nine instructions with the hardware prologue (the address two), eleven
/// without.
template <class Kernel>
[[gnu::always_inline]] inline void scheduler_trap() {
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
    __asm__ volatile(
        "addi sp, sp, -16\n\t"
        "csrr t0, mepc\n\t"
        "sw t0, 0(sp)\n\t"
        "csrr t0, mstatus\n\t"
        "sw t0, 4(sp)\n\t"
        "la t0, %0\n\t"
        "csrw mepc, t0\n\t"
        "mret\n\t"
        ::"i"(&scheduler_thread<Kernel>));
#else
    __asm__ volatile(
        "addi sp, sp, -16\n\t"
        "sw t0, 8(sp)\n\t"
        "csrr t0, mepc\n\t"
        "sw t0, 0(sp)\n\t"
        "csrr t0, mstatus\n\t"
        "sw t0, 4(sp)\n\t"
        "la t0, %0\n\t"
        "csrw mepc, t0\n\t"
        "lw t0, 8(sp)\n\t"
        "mret\n\t"
        ::"i"(&scheduler_thread<Kernel>));
#endif
}

/// The body the program binds to ecall_m_handler: mepc and mstatus from
/// the block, the block popped, the return into the interrupted code.
/// Six instructions with the hardware prologue, eight without.
[[gnu::always_inline]] inline void scheduler_return() {
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
    __asm__ volatile(
        "lw t0, 0(sp)\n\t"
        "csrw mepc, t0\n\t"
        "lw t0, 4(sp)\n\t"
        "csrw mstatus, t0\n\t"
        "addi sp, sp, 16\n\t"
        "mret\n\t");
#else
    __asm__ volatile(
        "sw t0, 8(sp)\n\t"
        "lw t0, 0(sp)\n\t"
        "csrw mepc, t0\n\t"
        "lw t0, 4(sp)\n\t"
        "csrw mstatus, t0\n\t"
        "lw t0, 8(sp)\n\t"
        "addi sp, sp, 16\n\t"
        "mret\n\t");
#endif
}

} // namespace brio
