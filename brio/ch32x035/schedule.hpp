/*
 * schedule.hpp - the scheduling request of the preemptive kernel on the
 * QingKe V4C of the CH32X035, and the thread-mode scheduler it starts.
 *
 * What a QingKe V4 platform gives Rubato (kernel/rubato.hpp) beyond what
 * Tenuto takes from it: a way to ask for the scheduler, and the place the
 * scheduler then runs. kernel/platform.hpp states the contract - the
 * request, pend_schedule(), runs the scheduler at once when it is made
 * from a dispatch and at the handler's exit when it is made from an
 * interrupt handler - and this file is how the CH32X035 keeps it. The
 * family's Rubato platform calls the request; a program binds the two
 * vectors this file needs to the bodies below, exactly as it binds every
 * other vector, and turns the nesting on once:
 *
 *     using P = brio::Ch32x035RubatoPlatform<>;
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
 * binding: naked, `no_icf` like every binding of this stratum (pfic.hpp
 * says why), aligned to four like every vector of this stratum (a fetch
 * from the flash reads a word), and marked a leaf in the image
 * (`__brio_leaf_<name>`, an absolute symbol that costs no byte), which is
 * what it is under the hardware prologue - no call, nothing saved in
 * software: neither body calls anything, and the scheduler they lead to
 * runs in thread mode, outside every trap.
 *
 * THE REQUEST IS THE SOFTWARE INTERRUPT, line 14 of the vector table
 * (the reference manual 7.3, table 7-1; the QingKe V4 manual table 3-1),
 * pended by a store into PFIC_IPSR1 (the reference manual 7.5.2.25; the
 * V4 manual 3.1: "set directly through the interrupt pending register to
 * trigger into the interrupt"), the PFIC's own way in for every line
 * (pfic.hpp's set_pending). The pending bit falls when the line is taken,
 * so nothing withdraws it. THIS CORE TAKES THE TRAP AT THE INSTRUCTION
 * AFTER THE STORE: measured on the CH32X035F8U6 with hand-placed assembly
 * reading mepc in the handler, the store at one address and mepc the
 * next, the same after the `csrsi` that unmasks a pending request, and
 * the same with a `fence.i` after the store (mepc the fence itself). So
 * the request is the store and nothing else, where the CH32V203's ends
 * with a `fence.i` because there the store is posted and three
 * instructions retire before the trap unless the fence closes them
 * (brio/ch32vx03/schedule.hpp): a request made from thread mode with MIE
 * set is taken before the instruction that follows it, which is what "at
 * once" means for a request made from a dispatch. A request made with MIE
 * clear waits for the unmask, so no dispatch ever starts inside a
 * critical section; a post made inside one serves its receiver when the
 * section ends, before the statement after it (the suite's letter g: 405
 * cycles from the section's last statement to the receiver's first, at
 * 48 MHz from the flash).
 *
 * NESTING, AT RUN TIME. The crt leaves INTSYSCR.INESTEN clear (an ISR body
 * runs to completion: docs/design/kernel.md section 1), and
 * scheduler_init() sets it, with the one preemption bit (PMTCFG = 01, two
 * levels), and puts the software interrupt's priority byte at 0x80: the
 * preemption bit set, so below every other line, which stay at the reset
 * priority 0. Peripheral handlers then still never nest among themselves
 * - one preemption level is theirs, and two lines of one level do not
 * preempt each other (the V4 manual 3.3) - and every one of them
 * interrupts the software interrupt's handler. The V4C implements two
 * nesting levels and a hardware prologue two deep (table 1-1), the
 * V4B's figures, and ignores PMTCFG's write as the V4B does (INTSYSCR
 * reads 0x3 after the write of 0x7, measured by the suite's letter i;
 * 0x2 in an image built without the prologue): the scheduler's trap and
 * one peripheral handler over it, inside that depth. The environment
 * call that ends a scheduler's run is an exception and no line preempts
 * it. The suite's letters a and e show the nesting at work: an
 * interrupt's post preempts a dispatch at depth one and at depth two.
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
 * (cortexm/schedule.hpp). With the hardware prologue (the CH32X035_HPE
 * option, the default) the MRET pops the sixteen caller-saved integer
 * registers the hardware pushed at the trap's entry (the V4 manual 3.4),
 * so the entry, scheduler_thread<K>(), lands in machine mode with the
 * trap no longer active and the INTERRUPTED CODE'S registers in the
 * register file. The entry stores those sixteen on the stack - x1,
 * x5..x7, x10..x17, x28..x31, the set the hardware saves - sets MIE, calls
 * K::activate(), loads them back and executes `ecall`. scheduler_return(),
 * the machine-mode environment call's vector (line 5 of the table), loads
 * mepc and mstatus from the two words, moves the stack pointer back up
 * over the block and executes MRET: the hardware's push at the ecall and
 * its pop at this MRET pair up around the exit, and the interrupted code
 * resumes with its registers, its mstatus and its stack pointer as they
 * were. Without the hardware prologue (CH32X035_HPE=OFF) the same shape
 * holds with the two bodies' one scratch register, t0, saved and restored
 * in the block's third word by the bodies themselves: nothing else has
 * been pushed, so the MRET has nothing else to pop. s0..s11 and sp never
 * leave the register file: everything between the two traps is code that
 * keeps them. This core has no floating-point unit (RV32IMAC), so the
 * entry has no f-registers to save and the file has no second shape.
 *
 * THE UNMASK IN THE ENTRY is not decoration. Entry does not clear MIE on
 * this core (the V4 manual 2.2), so the trap normally returns into the
 * entry with MIE set; but an interrupt racing the platform guard's
 * `csrrci` is taken with mepc past it and MPIE clear (the V4B's mask
 * shadow, docs/ch32vx03/platform.md), and the MRET then puts MIE at 0.
 * The entry sets it, so the dispatches run unmasked, and the exit's MRET
 * puts the section's own mask back with the saved mstatus. The scheduler
 * is then the work the request asked for before the section began.
 *
 * WHAT THIS BUYS. While the dispatches run, the software interrupt is
 * NOT active: an interrupt handler's post pends it again, and it is taken
 * at that handler's exit, over whatever dispatch was running, at any
 * depth - the kernel's road 1. A dispatch's post to an object above it
 * pends it from thread mode, taken at once, inside post() - road 2. Each
 * level of nesting costs, on the stack, the block (16 bytes), the entry's
 * sixteen registers (64) and the scheduler's own frames.
 *
 * RE-ENTRANCY. The software interrupt can be pended at any moment and the
 * shapes all close: pended while a dispatch runs, it nests one level
 * more; pended while the entry stores its registers or between
 * activate()'s return and the `ecall`, it nests over the entry itself,
 * whose `ecall` then runs after the nested level has unwound; pended by a
 * peripheral handler nested over the software interrupt's own handler,
 * it is taken at the instruction after that handler's MRET - the entry's
 * first - and builds its block below the first. The block is written
 * after the stack pointer has moved, so a handler nested over the
 * surgery puts its frame below it.
 *
 * WHAT THIS FILE ASSUMES. Every line but the software interrupt stays at
 * priority 0 (no driver of this stratum writes PFIC_IPRIOR); nothing but
 * this file writes the software interrupt's priority or pends it; the
 * program executes no `ecall` of its own - ecall_m_handler is this
 * file's; and the program runs in machine mode, on one stack.
 *
 * WHICH CORES. The QingKe V4C of the CH32X035 series. The file is the
 * CH32V203's and the CH32V303's (brio/ch32vx03/schedule.hpp) with that
 * file's f-register shape left out, the V4 manual covering the series:
 * the same PFIC, the same INTSYSCR, the same hardware prologue. Three
 * QingKe families now hold the same request and the same two bodies, and
 * folding them into a `qingke/` core stratum is a change of its own, made
 * with every image of the three byte-identical across it.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"
#include "ch32x035/pfic.hpp"

/// The binding of the scheduler's two vectors (the file header): naked,
/// kept whole against identical code folding, aligned to four like every
/// vector of this stratum, and marked a leaf in the image.
#define BRIO_CH32_SCHEDULER_VECTOR(name)                                  \
    __asm__(".set __brio_leaf_" #name ", 1");                             \
    extern "C" [[gnu::naked, gnu::no_icf, gnu::aligned(4)]] void name()

namespace brio {

/// INTSYSCR (CSR 0x804, QingKe V4 manual 3.2): the nesting enable and the
/// nesting-depth field's code for two levels, one preemption bit.
inline constexpr uint32_t intsyscr_inesten = 1UL << 1;
inline constexpr uint32_t intsyscr_pmtcfg_two_levels = 1UL << 2;

/// The software interrupt's priority byte: the preemption bit set, below
/// every line at the reset priority 0.
inline constexpr uint8_t scheduler_priority = 0x80;

/// The request: the software interrupt pended, taken before the next
/// instruction in thread mode with MIE set (the file header: measured,
/// this core needs no fence behind the store), at the handler's exit in
/// a handler.
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
}

/// The software interrupt below every other line, nesting on, the line
/// enabled. Call once in main(), before interrupts are enabled.
inline void scheduler_init() {
    pfic()->IPRIOR[static_cast<uint8_t>(Irq::software)] = scheduler_priority;
    __asm__ volatile("csrs 0x804, %0" ::"r"(intsyscr_inesten | intsyscr_pmtcfg_two_levels)
                     : "memory");
    Pfic::enable(Irq::software);
}

/// The thread-mode entry the trap returns into (the file header): the
/// sixteen caller-saved integer registers stored, MIE set, the scheduler,
/// the registers loaded back, and the way out through `ecall`. Naked: it
/// owns no frame but the one it builds, so the stack pointer at its
/// `ecall` is the one it started with, and it touches no register the
/// interrupted code keeps.
template <class Kernel>
[[gnu::naked, gnu::noinline]] void scheduler_thread() {
    __asm__ volatile(
        "addi sp, sp, -64\n\t"
        "sw ra, 0(sp)\n\t"   "sw t0, 4(sp)\n\t"   "sw t1, 8(sp)\n\t"   "sw t2, 12(sp)\n\t"
        "sw a0, 16(sp)\n\t"  "sw a1, 20(sp)\n\t"  "sw a2, 24(sp)\n\t"  "sw a3, 28(sp)\n\t"
        "sw a4, 32(sp)\n\t"  "sw a5, 36(sp)\n\t"  "sw a6, 40(sp)\n\t"  "sw a7, 44(sp)\n\t"
        "sw t3, 48(sp)\n\t"  "sw t4, 52(sp)\n\t"  "sw t5, 56(sp)\n\t"  "sw t6, 60(sp)\n\t"
        "csrsi mstatus, 8\n\t"
        "call %0\n\t"
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
