/*
 * schedule_hazard3.hpp
 *
 * The Hazard3 half of this target's scheduling request for the preemptive
 * kernel (kernel/rubato.hpp), and the thread-mode scheduler it starts.
 *
 * NOBODY INCLUDES THIS FILE DIRECTLY: rp2350/schedule.hpp does, when the
 * image is built for the Hazard3 pair, and binds the vector with
 * BRIO_RP2350_SCHEDULER_VECTORS. The names are the ones every brio
 * realization of the request exports - `request_scheduler()`,
 * `request_scheduler_on<core>()`, `scheduler_init()`,
 * `scheduler_thread<K>()` - and the engine below them is this core's,
 * written from datasheet 3.8.4 (traps), 3.8.4.2.1 (the software
 * interrupt), 3.8.6.1 (Xh3irq) and the SIO's RISCV_SOFTIRQ register
 * (3.1.11).
 *
 * THE REQUEST IS THE MACHINE SOFTWARE INTERRUPT. mip.MSIP is wired to the
 * SIO's RISCV_SOFTIRQ register, one flag per hart (3.8.4.2.1): writing
 * CORE<n>_SET raises hart n's flag, writing CORE<n>_CLR lowers it, and it
 * stays raised - a LEVEL - until it is cleared. The request raises the
 * flag of the hart that asks (mhartid, which equals the core number,
 * 3.1.2's note), so it is the calling core's own, as PendSV is on the
 * Cortex-M33; the platform's request_scheduler_on<core>() raises it only
 * when that hart is `core` (rp2350/schedule.hpp says why), and the test
 * costs the asking path the branch alone: the hart id is read anyway, and
 * the flag's bit becomes a constant. scheduler_init() sets mie.MSIE
 * (core_hazard3.hpp's machine_software_interrupt) and lowers a flag a
 * previous image may have left raised, both for the hart that calls it.
 *
 * "TAKEN BEFORE THE NEXT INSTRUCTION" IS A WAIT, NOT A BARRIER. 3.8.4
 * says when the core enters an interrupt (mstatus.MIE set, the bit
 * pending in mip, its enable in mie) and nothing about how many
 * instructions retire between the store that raises the flag and the
 * pending bit's arrival in mip: the store crosses the bus to the SIO, and
 * no fence of this core is defined to wait for an interrupt line. So the
 * request does not count on a latency. With MIE set it WAITS until the
 * scheduler has been served: the body below adds one to mscratch every
 * time it runs, and the request, having read mscratch before raising the
 * flag, spins until the count moves. The trap lands somewhere in the
 * request - after the store, in the read of mstatus or in the spin - and
 * the request returns only after every dispatch the post asked for has
 * run to its end, which is what "at once" means for a request made from a
 * dispatch. With MIE clear (a critical section, or a trap's handler) the
 * request returns at once: the flag stays raised and the core takes it
 * when the mask lifts - at the guard's `csrsi`, or at the handler's
 * `mret`. How many instructions retire after the store when nothing
 * waits is a fact of the bus this file does not depend on, and
 * test_rp2350_rubato's letter k measures it.
 *
 * PRIORITY AMONG THE ARCHITECTURE'S THREE SOURCES is 3.8.4.2's fixed
 * order, meip above msip above mtip, and it only orders interrupts
 * pending at the same instant: a trap clears mstatus.MIE and the crt's
 * dispatch never sets it again (startup_rp2350_riscv.S), so nothing
 * preempts a handler. A post made inside a peripheral handler raises the
 * flag and returns; the software interrupt is taken at that handler's
 * `mret`, after any other line still pending, and never inside it.
 *
 * NO NESTING IS TURNED ON, AND NONE IS NEEDED. Xh3irq's preemption
 * (3.8.6.1.4 and 3.8.6.1.5: MEIPRA's levels, MEICONTEXT's PREEMPT stack,
 * a meip handler that re-enables MIE) is for an interrupt handler
 * interrupting another, which the kernel's promise forbids (design/
 * kernel.md sections 1 and 11) and which Rubato does not ask for: what it
 * preempts is a DISPATCH, and a dispatch here runs outside every trap,
 * with MIE set. So MEIPRA stays at its reset value, as core_hazard3.hpp
 * says, and every handler still runs masked to its end. MEICONTEXT needs
 * no care either: the meip entry pushes its PREEMPT stack and sets
 * MRETEIRQ, the crt's `mret` pops it, and any other trap entry or `mret`
 * clears MRETEIRQ (3.8.6.1.5) - so the software interrupt's trap and the
 * entry's own `mret` below leave the stack where they found it.
 *
 * THE VECTOR IS REACHED THROUGH THE CRT'S TRAMPOLINE. mtvec is vectored,
 * and entry 3 of the table is `trap_soft_irq` (startup_rp2350_riscv.S):
 * it moves sp down 64 bytes, stores the sixteen caller-saved registers
 * there, CALLS `isr_riscv_softirq`, loads them back and executes `mret`.
 * The body bound to that name (scheduler_trap<K>(), naked) is therefore
 * a callee under a frame it does not own, and it does not run the
 * scheduler: it rewrites the trap so that the trampoline's `mret` lands
 * in the scheduler's entry instead of the interrupted code -
 *
 *  1. it lowers the hart's flag (the request is a level);
 *  2. it adds one to mscratch (the count the request waits on);
 *  3. it stores the interrupted code's mepc and mstatus in the two words
 *     just BELOW the trampoline's frame, free stack: nothing runs on this
 *     core between there and the entry, because MIE is clear all the
 *     way (point 4);
 *  4. it clears mstatus.MPIE as it reads it, so that the trampoline's
 *     `mret` - which copies MPIE into MIE (3.8.4) - lands in the entry
 *     with MIE still clear; the saved word keeps MPIE set;
 *  5. it writes the entry's address into mepc and returns to the
 *     trampoline, which restores the interrupted code's registers and
 *     executes its `mret`.
 *
 *     higher addresses
 *       the interrupted code's stack          <- sp of the interrupted code (I)
 *       the trampoline's 16 registers         I - 64 .. I - 4: dead once it
 *                                             has loaded them back
 *       mstatus of the trap                   I - 68 \  the body's two
 *       mepc of the trap                      I - 72 /  stores
 *       (two words unused)                    I - 80   <- sp in the entry
 *     lower addresses
 *
 * THE ENTRY, scheduler_thread<K>(), lands in machine mode, outside every
 * trap, with the interrupted code's registers in the register file, sp
 * back at I and MIE clear. It moves sp down 80 bytes - so the frame's
 * words 8 and 12 ARE the body's two stores, and nothing has to be copied
 * - stores the sixteen caller-saved registers in words 16 to 76 (the
 * set the trampoline saved, x1, x5..x7, x10..x17, x28..x31), and, still
 * with MIE clear, reads the kernel's ceiling into the frame's byte 0 and
 * stores 0 in its place: the entry's step of kernel/platform.hpp's
 * contract, made before any handler can run since the request was taken,
 * because nothing has run on this hart since the trap cleared MIE. It
 * then sets MIE and calls K::activate(base), which returns the count of
 * quiet posts its last choice read in a0's low byte. The exit follows: it
 * loads thirteen registers back with MIE still set, then writes mstatus
 * from word 12 - MIE clear, MPIE set, MPP machine, so from that
 * instruction on nothing can enter - stores `base` back into the ceiling,
 * compares the count with the kernel's and, when it moved, raises the
 * hart's flag again (CORE<n>_SET, the request's own store); then it
 * writes mepc from word 8, loads the last three registers, moves sp back
 * up and executes `mret`: the interrupted code resumes at its own pc,
 * with its registers, its stack pointer and its interrupt enable as they
 * were - and with the flag raised, the software interrupt is taken over
 * it as soon as the flag reaches mip, in place of the run that ended.
 * How many of its instructions run first is the bus's fact the request's
 * paragraph above declines to count on; none of them nests over the exit,
 * whose frame is given back. 80 and not 72 because the ABI keeps sp
 * 16-byte aligned, and the entry calls. s0..s11 and sp never leave the
 * register file: everything between the trap and the `mret` is code that
 * keeps them. Hazard3 on this chip has no floating-point unit, so there
 * is no f-register and no fcsr to save, and the entry saves none (3.8.8's
 * configuration enables no F extension, and the project's `-march` names
 * none).
 *
 * WHY THE ENTRY'S OWN `mret` AND NO EXIT VECTOR. `mret` is an ordinary
 * machine-mode instruction that "can be executed without any prior trap"
 * (3.8.4): it restores the privilege from MPP, MIE from MPIE, and jumps
 * to mepc. Nothing else is pending on it - this core pushes nothing at a
 * trap entry and keeps no "active" state per interrupt, so there is no
 * hardware frame to pop and no level to retire. A QingKe V4 needs an
 * `ecall` to leave its scheduler (ch32vx03/schedule.hpp), because its
 * hardware prologue's push must be paired with a pop inside a trap; here
 * the entry writes mepc and mstatus itself, and the exit is the entry's
 * own tail, its one step made whole by the MIE its write of mstatus
 * clears. The exception vector (`isr_riscv_exception`) is not touched,
 * and the program keeps the `ecall` it never executes.
 *
 * WHAT THIS BUYS. While the dispatches run, no trap is in progress: a
 * peripheral handler's post raises the flag again, and the software
 * interrupt is taken at that handler's `mret`, over whatever dispatch was
 * running, at any depth - the kernel's road 1. A dispatch's post to an
 * object above it raises the flag with MIE set, and the request waits for
 * the scheduler it starts - road 2. Each level of nesting costs, on the
 * stack, the entry's 80 bytes and the scheduler's own frames; the
 * trampoline's 64 are given back before the entry builds its frame. The
 * levels are the kernel's runs, at most one per object of the pack.
 *
 * TWO KERNELS: THE PAIR BODY. Core 1 is launched with core 0's mtvec
 * (rp2350/multicore.hpp), so both harts enter the one table and the one
 * trampoline, and `isr_riscv_softirq` must serve the kernel of the hart
 * that took it. scheduler_trap_pair<K0, K1>() is the body above with the
 * entry's address taken from a table of two after the body, indexed by
 * the mhartid the body reads anyway to lower the flag: the table's
 * address built where the single body builds the entry's, one `sh2add`
 * (Zba, which this core implements, 3.8.1, and the project's -march
 * names) and one load, placed so that no instruction waits on it (a load
 * is one cycle when the next instruction does not read it, 3.8.7). Two
 * instructions and two cycles over the single body, the same path on
 * both harts. mhartid and not SIO's CPUID: the same answer (3.1.2's
 * note), a CSR read of one cycle that the body makes already, where the
 * CPUID is a load from the SIO behind a second address. Everything else
 * the body and the entry touch is the hart's own: mepc, mstatus,
 * mscratch, the flag, the stack.
 *
 * RE-ENTRANCY. The flag can be raised at any moment and the shapes all
 * close. Raised while a dispatch runs, it nests one level more, at that
 * dispatch's index: a new trap, a trampoline frame and the body's two
 * words below the current sp, a new entry below them. Raised between the
 * entry's store of 0 and its write of mstatus, it cannot be: the ceiling
 * is 0 there, or a dispatch's index, and a post that finds 0 asks for
 * nothing. A line that became pending while the body or the entry's first
 * instructions ran (MIE clear, so nothing on this core can raise the flag
 * there) is taken at the entry's `csrsi`, after the store, and its post
 * asks for nothing. Raised by the exit itself, or after the entry has
 * written mstatus, it waits for the `mret` and is taken at the
 * interrupted code's next instruction, a level the outer one has already
 * left. So on this core no run starts at a ceiling of 0: from the request
 * being taken to the entry's store nothing runs (kernel/rubato.hpp, "The
 * stack"). Every block is written below a stack pointer that has already
 * moved, or with MIE clear, so no frame lands on another.
 *
 * WHAT THIS FILE ASSUMES. The crt's trampoline for the software interrupt
 * is the 64-byte frame and the `call` described above (the two words'
 * place is computed from it: a change there is a change here); the
 * stack pointer is 16-byte aligned wherever an interrupt lands, as the
 * ABI and the crt keep it; mscratch is this file's; nothing but this file
 * raises or lowers the flag of a hart running a Rubato kernel, or sets
 * mstatus.MIE inside a trap's handler; the program runs in machine mode,
 * on one stack per hart.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "rp2350/core.hpp"

#include "kernel/platform.hpp"

namespace brio {

/// The SIO's RISCV_SOFTIRQ register (3.1.11): CORE<n>_SET at bit n,
/// CORE<n>_CLR at bit 8 + n, each read back as the flag's state.
inline constexpr uint32_t riscv_softirq_address = SIO_BASE + SIO_RISCV_SOFTIRQ_OFFSET;
inline constexpr uint32_t riscv_softirq_clear_shift = 8;

[[gnu::always_inline]] inline volatile uint32_t& riscv_softirq() {
    return *reinterpret_cast<volatile uint32_t*>(riscv_softirq_address);
}

/// The raise and the wait both requests share: `set` written to
/// RISCV_SOFTIRQ, and with MIE set the spin until mscratch moves.
[[gnu::always_inline]] inline void raise_and_wait(uint32_t set) {
    const uint32_t served = csr_read<RVCSR_MSCRATCH_OFFSET>();
    riscv_softirq() = set;
    if ((csr_read<RVCSR_MSTATUS_OFFSET>() & mstatus_mie) != 0u) {
        while (csr_read<RVCSR_MSCRATCH_OFFSET>() == served) {
        }
    }
}

/// The request: the calling hart's flag raised and, with MIE set, the
/// wait until the scheduler it starts has been served (the file header).
///
/// A CALL THE COMPILER CANNOT SEE INTO, on purpose: noinline and noipa.
/// From a dispatch the request runs other dispatches before it returns,
/// through a trap the compiler does not know of, so to the caller it must
/// be a call to code that may read and write any static of the program -
/// which an out-of-line function with no interprocedural summary is. The
/// price is one call and return on the asking path only; a post that
/// asks nothing never reaches it.
[[gnu::noinline, gnu::noipa]] inline void request_scheduler() {
    raise_and_wait(1UL << csr_read<RVCSR_MHARTID_OFFSET>());
}

/// The request for hart `core`'s scheduler, made only when the calling
/// hart is `core` (rp2350/schedule.hpp says why): what
/// Rp2350RubatoPlatform<core>::pend_schedule() calls. Opaque for the
/// reason request_scheduler() gives, the test inside the same call.
template <uint8_t core>
[[gnu::noinline, gnu::noipa]] void request_scheduler_on() {
    if (csr_read<RVCSR_MHARTID_OFFSET>() == core) {
        raise_and_wait(1UL << core);
    }
}

/// The calling hart's flag lowered (a previous image may have left it
/// raised: the SIO is not reset with the core) and mie.MSIE set. Call
/// once in main(), before interrupts are enabled.
inline void scheduler_init() {
    riscv_softirq() = 1UL << (riscv_softirq_clear_shift + csr_read<RVCSR_MHARTID_OFFSET>());
    machine_software_interrupt(true);
}

/// The thread-mode entry the trampoline's `mret` lands in (the file
/// header): the frame whose words 8 and 12 the body has already written,
/// the sixteen caller-saved registers stored, the ceiling read into the
/// frame's byte 0 and 0 stored in its place with MIE still clear, MIE
/// set, the scheduler, thirteen registers loaded back unmasked; then the
/// exit with MIE clear again - mstatus written from word 12, the ceiling
/// put back, the count compared and the hart's flag raised again when it
/// moved - mepc written, the last three registers, the frame given back,
/// `mret`. Naked: it owns no frame but the one it builds, and it touches
/// no register the interrupted code keeps.
template <class Kernel>
[[gnu::naked, gnu::noinline]] void scheduler_thread() {
    __asm__ volatile(
        "addi sp, sp, -80\n\t"
        "sw ra, 16(sp)\n\t"  "sw t0, 20(sp)\n\t"  "sw t1, 24(sp)\n\t"  "sw t2, 28(sp)\n\t"
        "sw a0, 32(sp)\n\t"  "sw a1, 36(sp)\n\t"  "sw a2, 40(sp)\n\t"  "sw a3, 44(sp)\n\t"
        "sw a4, 48(sp)\n\t"  "sw a5, 52(sp)\n\t"  "sw a6, 56(sp)\n\t"  "sw a7, 60(sp)\n\t"
        "sw t3, 64(sp)\n\t"  "sw t4, 68(sp)\n\t"  "sw t5, 72(sp)\n\t"  "sw t6, 76(sp)\n\t"
        "lui t0, %%hi(%1)\n\t"
        "lbu a0, %%lo(%1)(t0)\n\t"   // base: the ceiling the request was taken at
        "sb zero, %%lo(%1)(t0)\n\t"  // the ceiling 0, MIE clear since the trap
        "sb a0, 0(sp)\n\t"           // base, for the exit
        "csrsi mstatus, 8\n\t"
        "call %0\n\t"                // activate(base): the count in a0's low byte
        "lw ra, 16(sp)\n\t"                                            "lw t2, 28(sp)\n\t"
                             "lw a1, 36(sp)\n\t"  "lw a2, 40(sp)\n\t"  "lw a3, 44(sp)\n\t"
        "lw a4, 48(sp)\n\t"  "lw a5, 52(sp)\n\t"  "lw a6, 56(sp)\n\t"  "lw a7, 60(sp)\n\t"
        "lw t3, 64(sp)\n\t"  "lw t4, 68(sp)\n\t"  "lw t5, 72(sp)\n\t"  "lw t6, 76(sp)\n\t"
        "lbu t1, 0(sp)\n\t"          // base
        "andi a0, a0, 255\n\t"       // the count
        "lw t0, 12(sp)\n\t"
        "csrw mstatus, t0\n\t"       // MIE clear from here: the exit is one step
        "lui t0, %%hi(%1)\n\t"
        "sb t1, %%lo(%1)(t0)\n\t"    // the ceiling back
        "lui t0, %%hi(%2)\n\t"
        "lbu t0, %%lo(%2)(t0)\n\t"
        "bne a0, t0, 2f\n"
        "1:\n\t"
        "lw t0, 8(sp)\n\t"
        "csrw mepc, t0\n\t"
        "lw a0, 32(sp)\n\t"
        "lw t1, 24(sp)\n\t"
        "lw t0, 20(sp)\n\t"
        "addi sp, sp, 80\n\t"
        "mret\n"
        "2:\n\t"
        "li t0, %3\n\t"              // a quiet post landed behind the last scan:
        "lui t1, %%hi(%4)\n\t"
        "sw t0, %%lo(%4)(t1)\n\t"    // the hart's flag raised, taken over the interrupted code
        "j 1b\n\t"
        ::"i"(&Kernel::activate), "i"(&Kernel::Platform::ceiling),
          "i"(&quiet_posts<typename Kernel::Platform>), "i"(1UL << Kernel::Platform::core),
          "i"(riscv_softirq_address));
}

/// The body the program binds to isr_riscv_softirq (through
/// BRIO_RP2350_SCHEDULER_VECTORS), called by the crt's trampoline with
/// MIE clear: the flag lowered, the served count moved, the interrupted
/// code's mepc and mstatus stored below the trampoline's frame with MPIE
/// cleared in the live mstatus, mepc pointed at scheduler_thread<K>(),
/// and the return to the trampoline. Sixteen lines of assembly, seventeen
/// instructions in the listing (`la` is two).
template <class Kernel>
[[gnu::always_inline]] inline void scheduler_trap() {
    __asm__ volatile(
        "csrr t1, mhartid\n\t"
        "li t0, %1\n\t"
        "sll t0, t0, t1\n\t"
        "lui t1, %%hi(%2)\n\t"
        "sw t0, %%lo(%2)(t1)\n\t"   // the flag lowered: a level, cleared before the mask lifts
        "csrr t0, mscratch\n\t"
        "addi t0, t0, 1\n\t"
        "csrw mscratch, t0\n\t"     // what the request waits on
        "csrr t0, mepc\n\t"
        "sw t0, -8(sp)\n\t"         // the entry's word 8
        "li t0, %3\n\t"
        "csrrc t1, mstatus, t0\n\t"
        "sw t1, -4(sp)\n\t"         // the entry's word 12, MPIE as the trap left it
        "la t0, %0\n\t"
        "csrw mepc, t0\n\t"
        "ret\n\t"
        ::"i"(&scheduler_thread<Kernel>), "i"(1UL << riscv_softirq_clear_shift),
          "i"(riscv_softirq_address), "i"(RVCSR_MSTATUS_MPIE_BITS));
}

/// The body the program binds to isr_riscv_softirq when each hart runs a
/// Rubato (through BRIO_RP2350_SCHEDULER_VECTORS_PAIR): scheduler_trap<K>()
/// with mepc pointed at scheduler_thread<K0>() on hart 0 and
/// scheduler_thread<K1>() on hart 1, from a table of two after the body
/// indexed by mhartid (the file header, "TWO KERNELS"). Eighteen lines
/// of assembly, nineteen instructions in the listing, the same on both
/// harts.
template <class K0, class K1>
[[gnu::always_inline]] inline void scheduler_trap_pair() {
    static_assert(rp2350_detail::kernel_pair_in_order<K0, K1>());
    __asm__ volatile(
        "csrr t2, mhartid\n\t"     // kept in t2 to the table's index
        "li t0, %2\n\t"
        "sll t0, t0, t2\n\t"
        "lui t1, %%hi(%3)\n\t"
        "sw t0, %%lo(%3)(t1)\n\t"   // the flag lowered: a level, cleared before the mask lifts
        "csrr t0, mscratch\n\t"
        "addi t0, t0, 1\n\t"
        "csrw mscratch, t0\n\t"     // what the request waits on
        "csrr t0, mepc\n\t"
        "sw t0, -8(sp)\n\t"         // the entry's word 8
        "la t3, 1f\n\t"
        "sh2add t3, t2, t3\n\t"
        "lw t3, 0(t3)\n\t"          // this hart's entry; the next instruction does not read it
        "li t0, %4\n\t"
        "csrrc t1, mstatus, t0\n\t"
        "sw t1, -4(sp)\n\t"         // the entry's word 12, MPIE as the trap left it
        "csrw mepc, t3\n\t"
        "ret\n\t"
        ".balign 4\n"
        "1:\n\t"
        ".word %0\n\t"              // hart 0
        ".word %1\n\t"              // hart 1
        ::"i"(&scheduler_thread<K0>), "i"(&scheduler_thread<K1>),
          "i"(1UL << riscv_softirq_clear_shift), "i"(riscv_softirq_address),
          "i"(RVCSR_MSTATUS_MPIE_BITS));
}

} // namespace brio
