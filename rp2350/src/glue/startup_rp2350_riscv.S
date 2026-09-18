/*
 * startup_rp2350_riscv.S - entry, image metadata, trap dispatch and reset
 * path for an RP2350 running its Hazard3 cores, compiled into every image
 * of the riscv presets (rp2350_add_app() lists it alongside the app's own
 * source). No SDK crt0: this file and the linker script are the whole
 * crt. Its Arm twin is startup_rp2350_arm.cpp, and the two agree on every
 * name an app binds.
 *
 * WHAT RUNS BEFORE THIS. The bootrom scans the flash for a block loop
 * containing a valid IMAGE_DEF; this one says RISC-V, so the bootrom
 * resets both cores into the RISC-V architecture (datasheet 3.9.1 -
 * ARCHSEL is sampled at processor reset, and an Arm image flashed over
 * this one switches them back, with no button and no OTP write) and
 * enters the image AT ITS LOWEST ADDRESS (5.9.5.2: there is no vector
 * table to enter through on this architecture, so the first instruction
 * of the image is the entry point - hence the jump below, before the
 * metadata).
 *
 * `reset run` from a debugger is a core reset and NOT a chip reset: the
 * clock tree, the pads and the peripherals stay as the previous image
 * left them. Nothing here assumes a reset state it did not make itself.
 *
 * THE TRAP TABLE (3.8.4.2). mtvec is written with the table's address and
 * its low bit set, which is this core's VECTORED mode: an interrupt of
 * cause N enters at mtvec + 4 * N, and an EXCEPTION enters at mtvec + 0.
 * The table must be aligned to its own size rounded up to a power of two
 * - 64 bytes here. Three interrupt causes exist on this core: 3 the
 * machine software interrupt (the SIO's RISCV_SOFTIRQ register, one bit
 * per core - the doorbell a second kernel will ring), 7 the machine timer
 * (the SIO's 64-bit MTIME against this core's MTIMECMP), 11 the external
 * interrupt, which is every one of the 52 system interrupt lines
 * multiplexed by Hazard3's own controller.
 *
 * THE NAMES ARE THE ARM TABLE'S (the file header of startup_rp2350_arm.cpp
 * says why): a system line N enters `isr_irqN`, which hardware/regs/
 * intctrl.h spells isr_uart0, isr_dma_0 and so on, so ONE vector binding
 * in an app is bound on both architectures. The machine timer enters
 * `isr_systick` - the name the kernel timebase's interrupt wears on this
 * target, whichever processor is in the socket: on Arm it is SysTick's own
 * vector, here it is the trap the platform timer raises, and the ticker
 * behind it is the architecture's own (brio/rp2350/ticker.hpp). The
 * machine software interrupt enters `isr_riscv_softirq`, which has no Arm
 * twin because on Arm that doorbell is a system line with a name of its
 * own. Every one of these is a WEAK label an app overrides by defining
 * the strong symbol, and one that nobody defines falls through to a spin,
 * as on every other target of this project.
 *
 * NO INTERRUPT NESTS OVER ANOTHER (design/kernel.md sections 1 and 11).
 * Hardware clears mstatus.MIE on trap entry and this dispatch NEVER SETS
 * IT AGAIN before the mret: the external handler loops on `meinext` and
 * runs each pending line's handler to completion, in descending priority
 * order, under one register save. That is the datasheet's own minimal
 * dispatch (3.8.6.1.6) and not the SDK's pre-emptive one - and because
 * nothing can preempt, mepc, mstatus and meicontext need no saving.
 *
 * THE REGISTERS SAVED are the caller-saved set an ordinary C function may
 * clobber: ra, t0..t6, a0..a7. The handlers are PLAIN C FUNCTIONS with no
 * interrupt attribute, which is what lets one save serve a whole run of
 * them.
 */

#include "hardware/platform_defs.h"

/* Hazard3's own CSRs (datasheet 3.8.6.1), by number: the assembler knows
   none of these names. */
#define RVCSR_MEINEXT 0xbe4

    .section .vectors, "ax", @progbits
    /* norvc so the jump is a full word, norelax so that no alignment
       request of the assembler's own is left for the linker to resolve:
       this section is to be exactly one instruction, and the metadata
       is to follow it immediately. */
    .option push
    .option norvc
    .option norelax
    .globl _start
    .type _start, %function
_start:
    j       reset_handler          /* the image's lowest address IS the entry point */
    .size _start, . - _start
    .option pop

/* THE IMAGE METADATA, right behind the entry jump and so well inside the
   first 4 kB the bootrom scans (datasheet 5.9.5.2): the twenty bytes that
   say "this is an executable, RISC-V, for this chip", as five
   little-endian words - a start marker, one one-word item naming the
   image type, the last-item terminator, a relative link to the next block
   in the loop (zero = a loop of this block alone) and an end marker. The
   word that decides the architecture is the second one, and it is the ONLY
   difference from the Arm crt's block. */
    .section .image_def, "a", @progbits
    .align 2
    .word   0xffffded3             /* PICOBIN_BLOCK_MARKER_START */
    .word   0x11010142             /* IMAGE_TYPE item: EXE, RISC-V, RP2350 */
    .word   0x000001ff             /* LAST item, one word */
    .word   0x00000000             /* next block in the loop: this one */
    .word   0xab123579             /* PICOBIN_BLOCK_MARKER_END */

    .section .text.crt, "ax", @progbits

/* ---- the trap table (mtvec, vectored) -------------------------------- */

    .p2align 6
    .globl riscv_trap_table
riscv_trap_table:
    .option push
    .option norvc
    .option norelax
    j       trap_exception         /* 0: every exception enters here */
    j       trap_unexpected        /* 1 */
    j       trap_unexpected        /* 2 */
    j       trap_soft_irq          /* 3: machine software interrupt */
    j       trap_unexpected        /* 4 */
    j       trap_unexpected        /* 5 */
    j       trap_unexpected        /* 6 */
    j       trap_timer             /* 7: machine timer interrupt */
    j       trap_unexpected        /* 8 */
    j       trap_unexpected        /* 9 */
    j       trap_unexpected        /* 10 */
    j       trap_external          /* 11: the 52 system lines, multiplexed */
    j       trap_unexpected        /* 12 */
    j       trap_unexpected        /* 13 */
    j       trap_unexpected        /* 14 */
    j       trap_unexpected        /* 15 */
    .option pop

/* Save and restore the caller-saved registers around a handler run: 64
   bytes of this trap's own stack frame, the set a C function may clobber. */
.macro save_caller_saved
    addi    sp, sp, -64
    sw      ra,  0(sp)
    sw      t0,  4(sp)
    sw      t1,  8(sp)
    sw      t2, 12(sp)
    sw      a0, 16(sp)
    sw      a1, 20(sp)
    sw      a2, 24(sp)
    sw      a3, 28(sp)
    sw      a4, 32(sp)
    sw      a5, 36(sp)
    sw      a6, 40(sp)
    sw      a7, 44(sp)
    sw      t3, 48(sp)
    sw      t4, 52(sp)
    sw      t5, 56(sp)
    sw      t6, 60(sp)
.endm

.macro restore_caller_saved
    lw      ra,  0(sp)
    lw      t0,  4(sp)
    lw      t1,  8(sp)
    lw      t2, 12(sp)
    lw      a0, 16(sp)
    lw      a1, 20(sp)
    lw      a2, 24(sp)
    lw      a3, 28(sp)
    lw      a4, 32(sp)
    lw      a5, 36(sp)
    lw      a6, 40(sp)
    lw      a7, 44(sp)
    lw      t3, 48(sp)
    lw      t4, 52(sp)
    lw      t5, 56(sp)
    lw      t6, 60(sp)
    addi    sp, sp, 64
.endm

/* The external interrupt: read meinext (the highest-priority pending and
   enabled line, already shifted left by two), call that line's handler,
   read again, until the MSB says there is none left. The UPDATE bit is
   deliberately not written: it exists to maintain the pre-emption context,
   and nothing here pre-empts. */
    .globl trap_external
    .type trap_external, %function
trap_external:
    save_caller_saved
1:
    csrr    a0, RVCSR_MEINEXT
    bltz    a0, 2f
    lui     a1, %hi(brio_rp2350_irq_table)
    add     a1, a1, a0
    lw      a1, %lo(brio_rp2350_irq_table)(a1)
    jalr    ra, a1
    j       1b
2:
    restore_caller_saved
    mret
    .size trap_external, . - trap_external

/* The machine timer: the kernel timebase on this architecture. */
    .type trap_timer, %function
trap_timer:
    save_caller_saved
    call    isr_systick
    restore_caller_saved
    mret

/* The machine software interrupt: the SIO's per-core RISCV_SOFTIRQ bit. */
    .type trap_soft_irq, %function
trap_soft_irq:
    save_caller_saved
    call    isr_riscv_softirq
    restore_caller_saved
    mret

/* Every exception (the vector table's entry 0). An exception handler that
   simply returned would re-execute the faulting instruction for ever, so
   the default spins; the reset chapter is what fills this in. */
    .type trap_exception, %function
trap_exception:
    save_caller_saved
    call    isr_riscv_exception
    restore_caller_saved
    mret

/* An interrupt cause this core does not have. */
    .type trap_unexpected, %function
trap_unexpected:
    j       trap_unexpected

/* ---- the reset path -------------------------------------------------- */

    .globl reset_handler
    .type reset_handler, %function
reset_handler:
    /* gp first, with relaxation off so the load is not turned into a
       gp-relative one before gp exists; then this core's stack. */
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop
    la      sp, __stack_top

    /* Interrupts off and every source masked: each of them is armed by
       the driver that owns it (the ticker sets mie.MTIE once MTIMECMP
       holds a deadline, the interrupt controller sets mie.MEIE with the
       first line it enables). A blanket arm here would take a timer trap
       on the spot, MTIME being enabled out of reset. */
    csrw    mstatus, zero
    csrw    mie, zero
    la      t0, riscv_trap_table
    ori     t0, t0, 1              /* the low bit: vectored mode */
    csrw    mtvec, t0

    /* .data (with .ram_text inside it) out of flash into SRAM. */
    la      t0, __data_load_start
    la      t1, __data_start
    la      t2, __data_end
1:  beq     t1, t2, 2f
    lw      t3, 0(t0)
    sw      t3, 0(t1)
    addi    t0, t0, 4
    addi    t1, t1, 4
    j       1b
2:
    /* .bss to zero. .noinit is neither loaded nor zeroed: the panic
       breadcrumb lives there and must survive a reset. */
    la      t0, __bss_start
    la      t1, __bss_end
3:  beq     t0, t1, 4f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       3b
4:
    /* The static constructors, in the two arrays the linker script
       collects (s0/s1 survive the calls: they are callee-saved). */
    la      s0, __preinit_array_start
    la      s1, __preinit_array_end
5:  beq     s0, s1, 6f
    lw      t0, 0(s0)
    addi    s0, s0, 4
    jalr    ra, t0
    j       5b
6:
    la      s0, __init_array_start
    la      s1, __init_array_end
7:  beq     s0, s1, 8f
    lw      t0, 0(s0)
    addi    s0, s0, 4
    jalr    ra, t0
    j       7b
8:
    call    main
9:  j       9b
    .size reset_handler, . - reset_handler

/* ---- the software vector table and its weak defaults ------------------ */

/* One word per system interrupt line, indexed by meinext's own value
   (the line number shifted left by two). In flash, read through the XIP
   window like any other constant. */
    .section .rodata.irq_table, "a", @progbits
    .p2align 2
    .globl brio_rp2350_irq_table
brio_rp2350_irq_table:
.altmacro
.macro table_entry n
    .word   isr_irq\n
.endm
.set IRQN, 0
.rept NUM_IRQS
    table_entry %IRQN
.set IRQN, IRQN + 1
.endr
.noaltmacro

    .section .text.crt, "ax", @progbits

/* Every name below is a WEAK label on the same spin loop, so an app binds
   a vector by defining the strong symbol and everything else lands
   somewhere harmless and legible. */
.macro weak_isr name
    .weak   \name
    .type   \name, %function
\name:
.endm

.altmacro
.macro weak_isr_n n
    weak_isr isr_irq\n
.endm
.set IRQN, 0
.rept NUM_IRQS
    weak_isr_n %IRQN
.set IRQN, IRQN + 1
.endr
.noaltmacro

weak_isr isr_systick
weak_isr isr_riscv_softirq
weak_isr isr_riscv_exception
    .globl brio_rp2350_default_isr
brio_rp2350_default_isr:
    j       brio_rp2350_default_isr

/* abort(): the one libc symbol a brio image references (libstdc++'s throw
   sites under -fno-exceptions). A spin, so the frame survives for the
   debugger, and so newlib's abort() does not drag the syscall stubs in. */
    .globl abort
    .type abort, %function
abort:
    j       abort
