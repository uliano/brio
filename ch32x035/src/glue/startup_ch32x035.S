/*
 * startup_ch32x035.S - vector table + reset path for the CH32X035 series,
 * compiled into EVERY image of this project (ch32x035_add_app() lists it
 * alongside the app's own source). No WCH startup template, no
 * SystemInit, no SystemCoreClock: this file and ld/<part>.ld are the
 * whole crt.
 *
 * THE TABLE'S FIRST WORD IS AN INSTRUCTION. The core fetches address 0
 * as code, so entry 0 is `j reset_handler`; every entry after it is a
 * handler ADDRESS, because mtvec is set below with both mode bits -
 * MODE1 = absolute addresses, MODE0 = vectored by interrupt number
 * (RM 7.5.3.2). Index 12 is the core's system counter and index 39 is
 * USART2: the numbers ch32x035/device.hpp's Irq enum states, since on
 * this core the interrupt number IS the table index.
 *
 * ONE TABLE FOR THE WHOLE SERIES, 55 WORDS. The reference manual's table
 * 7-1 has seven core entries and thirty-nine peripheral ones, 16 to 54,
 * with 19 reserved, and it is the same for every part: a part whose
 * package does not bring a block out still has the block's line. WCH's
 * own startup file for the series (EVT/EXAM/SRC/Startup) has the same
 * fifty-five words in the same order, and the table here agrees with it
 * word for word.
 *
 * THE APP BINDS THE VECTOR, RISC-V edition: every handler below is
 * WEAK - an alias for default_handler (a spin), or for the fault and the
 * breakpoint entries a spin of its own - and an app binds one by defining
 * the strong symbol -
 *
 *   extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
 *   extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { Serial::isr(); }
 *
 * The attribute is what makes gcc end the function with MRET, and WHICH
 * attribute is the project's CH32X035_HPE option (BRIO_CH32_HPE here):
 * with it, this crt sets INTSYSCR.HWSTKEN and the handlers are declared
 * "WCH-Interrupt-fast" - the caller-saved registers pushed and popped by
 * the hardware (QingKe V4 manual 3.4; two levels deep on the V4C, table
 * 1-1) and no prologue in the handler; without it, INTSYSCR stays at its
 * reset value and a handler saves what it clobbers in software. Interrupt
 * NESTING (INESTEN) is never set, which is where this crt parts company
 * with WCH's own - that file writes INTSYSCR = 3, the hardware stack AND
 * nesting: the kernel's rule (docs/design/kernel.md) is that an ISR body
 * runs to completion, on every target.
 *
 * THE MICROPROCESSOR CONFIGURATION REGISTER IS WRITTEN HERE, AND NEITHER
 * MANUAL SAYS WHAT IT HOLDS. corecfgr (CSR 0xBC0) "configures the
 * microprocessor pipeline, instruction prediction and other related
 * features" and "generally does not need to be operated" (QingKe V4
 * manual 8.3), with no bit table; this series' reference manual does not
 * name it at all. WCH's own startup file for the series writes 0x1F into
 * it before anything else runs, as it does on the CH32V203 and the
 * CH32V303, and this crt writes the same value for the same reason the
 * sibling crt does (ch32vx03/src/glue/startup_ch32vx03.S): it is what
 * the vendor's silicon is shipped running. What the five bits buy on
 * this core is a measurement this stratum owes.
 *
 * reset_handler does the minimum an image needs: the global pointer and
 * the stack, copy .data from flash, zero .bss, paint the free RAM, walk
 * .preinit_array and .init_array, then main(). It does NOT touch the
 * clock: Clock::init() in main() owns the tree, the way it does on every
 * other target. And it enters main() in MACHINE mode with interrupts
 * masked (mstatus 0x1880) - WCH's file writes mstatus 0x88 and enters
 * main() through an mret, whose MPP field of zero is user mode.
 */

    .section .vectors, "ax", @progbits
    .globl _start
    .align 2
_start:
    .option norvc
    j       reset_handler          /* 0  the core fetches this as code */
    .word   0                      /* 1  */
    .word   nmi_handler            /* 2  */
    .word   fault_handler          /* 3  the exception vector (HardFault) */
    .word   0                      /* 4  */
    .word   ecall_m_handler        /* 5  environment call, machine mode */
    .word   0                      /* 6  */
    .word   0                      /* 7  */
    .word   ecall_u_handler        /* 8  environment call, user mode */
    .word   breakpoint_handler     /* 9  ebreak */
    .word   0                      /* 10 */
    .word   0                      /* 11 */
    .word   systick_handler        /* 12 the core's system counter (STK) */
    .word   0                      /* 13 */
    .word   software_handler       /* 14 */
    .word   0                      /* 15 */
    .word   wwdg_handler           /* 16 */
    .word   pvd_handler            /* 17 the supply monitor, EXTI line 26 */
    .word   flash_handler          /* 18 */
    .word   0                      /* 19 reserved: this series' RCC has no interrupt */
    .word   exti7_0_handler        /* 20 */
    .word   awu_handler            /* 21 the auto-wakeup, EXTI line 27 */
    .word   dma1_channel1_handler  /* 22 */
    .word   dma1_channel2_handler  /* 23 */
    .word   dma1_channel3_handler  /* 24 */
    .word   dma1_channel4_handler  /* 25 */
    .word   dma1_channel5_handler  /* 26 */
    .word   dma1_channel6_handler  /* 27 */
    .word   dma1_channel7_handler  /* 28 */
    .word   adc1_handler           /* 29 */
    .word   i2c1_ev_handler        /* 30 */
    .word   i2c1_er_handler        /* 31 */
    .word   usart1_handler         /* 32 */
    .word   spi1_handler           /* 33 */
    .word   tim1_brk_handler       /* 34 */
    .word   tim1_up_handler        /* 35 */
    .word   tim1_trg_com_handler   /* 36 */
    .word   tim1_cc_handler        /* 37 */
    .word   tim2_up_handler        /* 38 TIM2 is an advanced timer here: four lines */
    .word   usart2_handler         /* 39 */
    .word   exti15_8_handler       /* 40 */
    .word   exti25_16_handler      /* 41 */
    .word   usart3_handler         /* 42 */
    .word   usart4_handler         /* 43 */
    .word   dma1_channel8_handler  /* 44 */
    .word   usbfs_handler          /* 45 */
    .word   usbfs_wakeup_handler   /* 46 EXTI line 28 */
    .word   pioc_handler           /* 47 */
    .word   opa_handler            /* 48 */
    .word   usbpd_handler          /* 49 */
    .word   usbpd_wakeup_handler   /* 50 EXTI line 29 */
    .word   tim2_cc_handler        /* 51 */
    .word   tim2_trg_com_handler   /* 52 */
    .word   tim2_brk_handler       /* 53 */
    .word   tim3_handler           /* 54 */
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
    /* The free RAM between the last placed section and the stack top is
       painted, so that stack_untouched() (brio/ch32x035/platform.hpp)
       can say how deep the stack ever reached. Nothing is on the stack
       yet: this runs before the first call. */
    la      a0, __stack_floor
    la      a1, __stack_top
    li      t0, 0xA5A5A5A5
    bgeu    a0, a1, 2f
1:  sw      t0, (a0)
    addi    a0, a0, 4
    bltu    a0, a1, 1b
2:
    /* corecfgr: the pipeline and branch prediction bits the file header
       explains - 0x1F under WCH's own crt. */
    li      t0, 0x1f
    csrw    0xbc0, t0

    /* mstatus: machine mode, interrupts still masked - main() enables
       them when its drivers are up. mtvec: this table, vectored, with
       absolute addresses. INTSYSCR: the hardware stack if the image was
       built for it, nesting never. */
    li      t0, 0x1880
    csrw    mstatus, t0
#if defined(BRIO_CH32_HPE) && BRIO_CH32_HPE
    li      t0, 0x1
    csrw    0x804, t0
#endif
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

    /* main() returning is a program that ran out of program: spin where
       a debugger can see it rather than fetching rubbish. */
main_returned:
    j       main_returned

/* ---- the default handlers ------------------------------------------------
 * A vector an app never binds lands in a spin, which is legible under a
 * debugger and harmless without one. fault_handler and breakpoint_handler
 * are spins of their OWN, weak like the rest: an exception and an ebreak
 * with no debugger attached are wrecks worth telling apart from an unbound
 * peripheral vector and from each other, and a probe that halts the core
 * reads which one by the program counter alone. */
    .globl default_handler
    .type default_handler, @function
default_handler:
    j       default_handler

    .weak   fault_handler
    .type   fault_handler, @function
fault_handler:
    j       fault_handler

    .weak   breakpoint_handler
    .type   breakpoint_handler, @function
breakpoint_handler:
    j       breakpoint_handler

.macro weak_vector name
    .weak   \name
    .set    \name, default_handler
.endm

    weak_vector nmi_handler
    weak_vector ecall_m_handler
    weak_vector ecall_u_handler
    weak_vector systick_handler
    weak_vector software_handler
    weak_vector wwdg_handler
    weak_vector pvd_handler
    weak_vector flash_handler
    weak_vector exti7_0_handler
    weak_vector awu_handler
    weak_vector dma1_channel1_handler
    weak_vector dma1_channel2_handler
    weak_vector dma1_channel3_handler
    weak_vector dma1_channel4_handler
    weak_vector dma1_channel5_handler
    weak_vector dma1_channel6_handler
    weak_vector dma1_channel7_handler
    weak_vector adc1_handler
    weak_vector i2c1_ev_handler
    weak_vector i2c1_er_handler
    weak_vector usart1_handler
    weak_vector spi1_handler
    weak_vector tim1_brk_handler
    weak_vector tim1_up_handler
    weak_vector tim1_trg_com_handler
    weak_vector tim1_cc_handler
    weak_vector tim2_up_handler
    weak_vector usart2_handler
    weak_vector exti15_8_handler
    weak_vector exti25_16_handler
    weak_vector usart3_handler
    weak_vector usart4_handler
    weak_vector dma1_channel8_handler
    weak_vector usbfs_handler
    weak_vector usbfs_wakeup_handler
    weak_vector pioc_handler
    weak_vector opa_handler
    weak_vector usbpd_handler
    weak_vector usbpd_wakeup_handler
    weak_vector tim2_cc_handler
    weak_vector tim2_trg_com_handler
    weak_vector tim2_brk_handler
    weak_vector tim3_handler
