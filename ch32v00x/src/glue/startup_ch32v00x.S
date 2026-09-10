/*
 * startup_ch32v00x.S - vector table + reset path for the CH32V00x,
 * compiled into EVERY image of this project (ch32_add_app() lists it
 * alongside the app's own source). No WCH startup template, no
 * SystemInit, no SystemCoreClock: this file and ld/<part>.ld are the
 * whole crt.
 *
 * THE TABLE'S FIRST WORD IS AN INSTRUCTION. The core fetches address 0
 * as code, so entry 0 is `j reset_handler`; every entry after it is a
 * handler ADDRESS, because mtvec is set below with both mode bits -
 * MODE1 = absolute addresses, MODE0 = vectored by interrupt number
 * (RM 6.5.3.2). Index 12 is the core's system counter and index 32 is
 * USART1: the numbers ch32v00x/device.hpp's Irq enum states, since on
 * this core the interrupt number IS the table index.
 *
 * THE APP BINDS THE VECTOR, RISC-V edition: every handler below is a
 * WEAK alias for default_handler (a spin), and an app binds one by
 * defining the strong symbol -
 *
 *   extern "C" [[gnu::interrupt]] void systick_handler() { brio::Ticker::tick(); }
 *   extern "C" [[gnu::interrupt]] void usart1_handler() { Serial::isr(); }
 *
 * The `interrupt` attribute is what makes gcc end the function with
 * MRET and save what it clobbers; it is required, because this crt
 * leaves INTSYSCR at its reset value - hardware stacking and interrupt
 * nesting OFF (RM 6.5.3.1). WCH's own startup turns both on and uses
 * their compiler's "WCH-Interrupt-fast" attribute instead; not doing so
 * keeps this stratum compilable by an upstream gcc, and the hardware
 * stack becomes a measured optimization rather than an assumption.
 *
 * THE NAMES ARE OURS. There is no vendor header in this build (see
 * brio/ch32v00x/device.hpp), so no vendor spelling is being honoured:
 * the handlers are named in the project's own style, one per line of
 * the table, from the reference manual's interrupt list.
 *
 * reset_handler does the minimum an image needs: the global pointer and
 * the stack, copy .data from flash, zero .bss, walk .preinit_array and
 * .init_array (a static constructor that never ran is a silent wrong
 * answer - the first image of this bring-up found exactly that), then
 * main(). It does NOT touch the clock: Clock::init() in main() owns the
 * tree, the way it does on the other three targets.
 */

    .section .vectors, "ax", @progbits
    .globl _start
    .align 2
_start:
    .option norvc
    j       reset_handler          /* 0  the core fetches this as code */
    .word   0                      /* 1  */
    .word   nmi_handler            /* 2  */
    .word   fault_handler          /* 3  exceptions land here, ebreak included */
    .word   0                      /* 4  */
    .word   0                      /* 5  */
    .word   0                      /* 6  */
    .word   0                      /* 7  */
    .word   0                      /* 8  */
    .word   0                      /* 9  */
    .word   0                      /* 10 */
    .word   0                      /* 11 */
    .word   systick_handler        /* 12 the core's system counter (STK) */
    .word   0                      /* 13 */
    .word   software_handler       /* 14 */
    .word   0                      /* 15 */
    .word   wwdg_handler           /* 16 */
    .word   pvd_handler            /* 17 */
    .word   flash_handler          /* 18 */
    .word   rcc_handler            /* 19 */
    .word   exti7_0_handler        /* 20 */
    .word   awu_handler            /* 21 */
    .word   dma1_channel1_handler  /* 22 */
    .word   dma1_channel2_handler  /* 23 */
    .word   dma1_channel3_handler  /* 24 */
    .word   dma1_channel4_handler  /* 25 */
    .word   dma1_channel5_handler  /* 26 */
    .word   dma1_channel6_handler  /* 27 */
    .word   dma1_channel7_handler  /* 28 */
    .word   adc_handler            /* 29 */
    .word   i2c1_ev_handler        /* 30 */
    .word   i2c1_er_handler        /* 31 */
    .word   usart1_handler         /* 32 */
    .word   spi1_handler           /* 33 */
    .word   tim1_brk_handler       /* 34 */
    .word   tim1_up_handler        /* 35 */
    .word   tim1_trg_com_handler   /* 36 */
    .word   tim1_cc_handler        /* 37 */
    .word   tim2_handler           /* 38 */
    .word   usart2_handler         /* 39 CH32V005/006/007 only */
    .word   opcm_handler           /* 40 CH32V005/006/007 only */
    .option rvc

    .section .text.startup, "ax", @progbits
    .globl reset_handler
    .type reset_handler, @function
reset_handler:
.option push
.option norelax
    la      gp, __global_pointer$
.option pop
    la      sp, __stack_top

    /* .data: flash image to RAM */
    la      a0, __data_load_start
    la      a1, __data_start
    la      a2, __data_end
    bgeu    a1, a2, 2f
1:  lw      t0, (a0)
    sw      t0, (a1)
    addi    a0, a0, 4
    addi    a1, a1, 4
    bltu    a1, a2, 1b
2:
    /* .bss to zero (.noinit is deliberately left alone) */
    la      a0, __bss_start
    la      a1, __bss_end
    bgeu    a0, a1, 2f
1:  sw      zero, (a0)
    addi    a0, a0, 4
    bltu    a0, a1, 1b
2:
    /* mstatus: machine mode, interrupts still masked - main() enables
       them when its drivers are up. mtvec: this table, vectored, with
       absolute addresses. */
    li      t0, 0x1880
    csrw    mstatus, t0
    la      t0, _start
    ori     t0, t0, 3
    csrw    mtvec, t0

    /* .preinit_array, then .init_array: the static constructors. s0/s1
       are callee-saved, so they survive each call. */
    la      s0, __preinit_array_start
    la      s1, __preinit_array_end
1:  bgeu    s0, s1, 2f
    lw      t0, (s0)
    addi    s0, s0, 4
    jalr    t0
    j       1b
2:
    la      s0, __init_array_start
    la      s1, __init_array_end
1:  bgeu    s0, s1, 2f
    lw      t0, (s0)
    addi    s0, s0, 4
    jalr    t0
    j       1b
2:
    jal     main

    /* main() returning is a program that ran out of program: spin
       where a debugger can see it rather than fetching rubbish. */
main_returned:
    j       main_returned

/* ---- the default handlers ------------------------------------------------
 * A vector an app never binds lands in a spin, which is legible under a
 * debugger and harmless without one. fault_handler is its own symbol:
 * on this core `ebreak` with no debugger attached escalates here, and
 * telling that wreck apart from an unbound peripheral vector is worth
 * one label. */
    .globl default_handler
    .type default_handler, @function
default_handler:
    j       default_handler

    .weak   nmi_handler
    .set    nmi_handler, default_handler
    .weak   fault_handler
    .set    fault_handler, default_handler
    .weak   systick_handler
    .set    systick_handler, default_handler
    .weak   software_handler
    .set    software_handler, default_handler
    .weak   wwdg_handler
    .set    wwdg_handler, default_handler
    .weak   pvd_handler
    .set    pvd_handler, default_handler
    .weak   flash_handler
    .set    flash_handler, default_handler
    .weak   rcc_handler
    .set    rcc_handler, default_handler
    .weak   exti7_0_handler
    .set    exti7_0_handler, default_handler
    .weak   awu_handler
    .set    awu_handler, default_handler
    .weak   dma1_channel1_handler
    .set    dma1_channel1_handler, default_handler
    .weak   dma1_channel2_handler
    .set    dma1_channel2_handler, default_handler
    .weak   dma1_channel3_handler
    .set    dma1_channel3_handler, default_handler
    .weak   dma1_channel4_handler
    .set    dma1_channel4_handler, default_handler
    .weak   dma1_channel5_handler
    .set    dma1_channel5_handler, default_handler
    .weak   dma1_channel6_handler
    .set    dma1_channel6_handler, default_handler
    .weak   dma1_channel7_handler
    .set    dma1_channel7_handler, default_handler
    .weak   adc_handler
    .set    adc_handler, default_handler
    .weak   i2c1_ev_handler
    .set    i2c1_ev_handler, default_handler
    .weak   i2c1_er_handler
    .set    i2c1_er_handler, default_handler
    .weak   usart1_handler
    .set    usart1_handler, default_handler
    .weak   spi1_handler
    .set    spi1_handler, default_handler
    .weak   tim1_brk_handler
    .set    tim1_brk_handler, default_handler
    .weak   tim1_up_handler
    .set    tim1_up_handler, default_handler
    .weak   tim1_trg_com_handler
    .set    tim1_trg_com_handler, default_handler
    .weak   tim1_cc_handler
    .set    tim1_cc_handler, default_handler
    .weak   tim2_handler
    .set    tim2_handler, default_handler
    .weak   usart2_handler
    .set    usart2_handler, default_handler
    .weak   opcm_handler
    .set    opcm_handler, default_handler
