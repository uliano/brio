/*
 * startup_ch32v203.S - vector table + reset path for the CH32V203,
 * compiled into EVERY image of this project (ch32v203_add_app() lists it
 * alongside the app's own source). No WCH startup template, no
 * SystemInit, no SystemCoreClock: this file and ld/<part>.ld are the
 * whole crt.
 *
 * THE TABLE'S FIRST WORD IS AN INSTRUCTION. The core fetches address 0
 * as code, so entry 0 is `j reset_handler`; every entry after it is a
 * handler ADDRESS, because mtvec is set below with both mode bits -
 * MODE1 = absolute addresses, MODE0 = vectored by interrupt number
 * (QingKe V4 manual 3.2). Index 12 is the core's system counter and
 * index 53 is USART1: the numbers ch32v203/device.hpp's Irq enum
 * states, since on this core the interrupt number IS the table index.
 *
 * THE TAIL OF THE TABLE IS THE PART'S. Entries 61 and up belong to the
 * device class: WCH's CH32V20x_D6 (every part up to the CH32V203C8)
 * ends at 62 with UART4 and the eighth DMA channel right after the
 * USBFS pair, while the CH32V203RB (their D8) puts the Ethernet pair
 * and two words of the wireless family's there, and runs to 69 with
 * TIM5, UART4, the eighth DMA channel and the 32 kHz oscillator's two
 * lines. The reference manual's own vector table (9.2) is the UNION of
 * this family with the CH32V30x and lists neither tail - it names TIM8
 * at 59..62, which is the larger family's - so the per-class truth is
 * stated here, from the part definition, and nowhere else.
 *
 * THE APP BINDS THE VECTOR, RISC-V edition: every handler below is a
 * WEAK alias for default_handler (a spin), and an app binds one by
 * defining the strong symbol -
 *
 *   extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
 *   extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { Serial::isr(); }
 *
 * The attribute is what makes gcc end the function with MRET, and WHICH
 * attribute is the project's CH32V203_HPE option (BRIO_CH32_HPE here):
 * with it, this crt sets INTSYSCR.HWSTKEN and the handlers are declared
 * "WCH-Interrupt-fast" - the caller-saved registers pushed and popped by
 * the hardware (QingKe V4 manual 3.2, HPE depth 3) and no prologue in the
 * handler; without it, INTSYSCR stays at its reset value and a handler
 * saves what it clobbers in software. Interrupt NESTING (INESTEN) is
 * never set, which is where this crt parts company with WCH's own: the
 * kernel's rule (docs/design/kernel.md) is that an ISR body runs to
 * completion, on every target.
 *
 * THE MICROPROCESSOR CONFIGURATION REGISTER IS WRITTEN HERE, AND THE
 * MANUAL DOES NOT SAY WHAT IT HOLDS. corecfgr (CSR 0xBC0) "configures
 * the microprocessor pipeline, instruction prediction and other related
 * features" and "generally does not need to be operated" (QingKe V4
 * manual 8.1), with no bit table anywhere; its value is 0 at the reset
 * vector (measured over the debug port) and WCH's own startup file
 * writes 0x1F into it before anything else runs, so every program built
 * with their tools runs with those five bits set and none of their
 * documents says what they do. This crt writes the same value for the
 * same reason a driver copies a factory calibration it cannot derive,
 * and the cost of not writing it is a measurement this stratum owes
 * (docs/ch32v203/README.md's gap list).
 *
 * reset_handler does the minimum an image needs: the global pointer and
 * the stack, copy .data from flash, zero .bss, paint the free RAM, walk
 * .preinit_array and .init_array, then main(). It does NOT touch the
 * clock: Clock::init() in main() owns the tree, the way it does on every
 * other target.
 */

    .section .vectors, "ax", @progbits
    .globl _start
    .align 2
_start:
    .option norvc
    j       reset_handler          /* 0  the core fetches this as code */
    .word   0                      /* 1  */
    .word   nmi_handler            /* 2  */
    .word   fault_handler          /* 3  the exception vector */
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
    .word   pvd_handler            /* 17 */
    .word   tamper_handler         /* 18 */
    .word   rtc_handler            /* 19 */
    .word   flash_handler          /* 20 */
    .word   rcc_handler            /* 21 */
    .word   exti0_handler          /* 22 */
    .word   exti1_handler          /* 23 */
    .word   exti2_handler          /* 24 */
    .word   exti3_handler          /* 25 */
    .word   exti4_handler          /* 26 */
    .word   dma1_channel1_handler  /* 27 */
    .word   dma1_channel2_handler  /* 28 */
    .word   dma1_channel3_handler  /* 29 */
    .word   dma1_channel4_handler  /* 30 */
    .word   dma1_channel5_handler  /* 31 */
    .word   dma1_channel6_handler  /* 32 */
    .word   dma1_channel7_handler  /* 33 */
    .word   adc1_2_handler         /* 34 */
    .word   usb_hp_can1_tx_handler /* 35 USBD high priority, or CAN1 TX */
    .word   usb_lp_can1_rx0_handler/* 36 USBD low priority, or CAN1 RX0 */
    .word   can1_rx1_handler       /* 37 */
    .word   can1_sce_handler       /* 38 */
    .word   exti9_5_handler        /* 39 */
    .word   tim1_brk_handler       /* 40 */
    .word   tim1_up_handler        /* 41 */
    .word   tim1_trg_com_handler   /* 42 */
    .word   tim1_cc_handler        /* 43 */
    .word   tim2_handler           /* 44 */
    .word   tim3_handler           /* 45 */
    .word   tim4_handler           /* 46 */
    .word   i2c1_ev_handler        /* 47 */
    .word   i2c1_er_handler        /* 48 */
    .word   i2c2_ev_handler        /* 49 */
    .word   i2c2_er_handler        /* 50 */
    .word   spi1_handler           /* 51 */
    .word   spi2_handler           /* 52 */
    .word   usart1_handler         /* 53 */
    .word   usart2_handler         /* 54 */
    .word   usart3_handler         /* 55 */
    .word   exti15_10_handler      /* 56 */
    .word   rtc_alarm_handler      /* 57 */
    .word   usb_wakeup_handler     /* 58 USBD wake-up, through EXTI */
    .word   usbfs_handler          /* 59 */
    .word   usbfs_wakeup_handler   /* 60 */
#if defined(CH32V203RB)
    .word   eth_handler            /* 61 */
    .word   eth_wakeup_handler     /* 62 */
    .word   0                      /* 63 the wireless family's radio pair */
    .word   0                      /* 64 */
    .word   tim5_handler           /* 65 */
    .word   uart4_handler          /* 66 */
    .word   dma1_channel8_handler  /* 67 */
    .word   osc32k_cal_handler     /* 68 */
    .word   osc32k_wakeup_handler  /* 69 */
#else
    .word   uart4_handler          /* 61 */
    .word   dma1_channel8_handler  /* 62 */
#endif
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
       painted, so that stack_untouched() (brio/ch32v203/platform.hpp)
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
       explains - zero at the reset vector, 0x1F under WCH's own crt. */
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
 * are their own symbols: an ebreak with no debugger attached is a wreck
 * worth telling apart from an unbound peripheral vector. */
    .globl default_handler
    .type default_handler, @function
default_handler:
    j       default_handler

.macro weak_vector name
    .weak   \name
    .set    \name, default_handler
.endm

    weak_vector nmi_handler
    weak_vector fault_handler
    weak_vector ecall_m_handler
    weak_vector ecall_u_handler
    weak_vector breakpoint_handler
    weak_vector systick_handler
    weak_vector software_handler
    weak_vector wwdg_handler
    weak_vector pvd_handler
    weak_vector tamper_handler
    weak_vector rtc_handler
    weak_vector flash_handler
    weak_vector rcc_handler
    weak_vector exti0_handler
    weak_vector exti1_handler
    weak_vector exti2_handler
    weak_vector exti3_handler
    weak_vector exti4_handler
    weak_vector dma1_channel1_handler
    weak_vector dma1_channel2_handler
    weak_vector dma1_channel3_handler
    weak_vector dma1_channel4_handler
    weak_vector dma1_channel5_handler
    weak_vector dma1_channel6_handler
    weak_vector dma1_channel7_handler
    weak_vector dma1_channel8_handler
    weak_vector adc1_2_handler
    weak_vector usb_hp_can1_tx_handler
    weak_vector usb_lp_can1_rx0_handler
    weak_vector can1_rx1_handler
    weak_vector can1_sce_handler
    weak_vector exti9_5_handler
    weak_vector tim1_brk_handler
    weak_vector tim1_up_handler
    weak_vector tim1_trg_com_handler
    weak_vector tim1_cc_handler
    weak_vector tim2_handler
    weak_vector tim3_handler
    weak_vector tim4_handler
    weak_vector i2c1_ev_handler
    weak_vector i2c1_er_handler
    weak_vector i2c2_ev_handler
    weak_vector i2c2_er_handler
    weak_vector spi1_handler
    weak_vector spi2_handler
    weak_vector usart1_handler
    weak_vector usart2_handler
    weak_vector usart3_handler
    weak_vector uart4_handler
    weak_vector exti15_10_handler
    weak_vector rtc_alarm_handler
    weak_vector usb_wakeup_handler
    weak_vector usbfs_handler
    weak_vector usbfs_wakeup_handler
#if defined(CH32V203RB)
    weak_vector eth_handler
    weak_vector eth_wakeup_handler
    weak_vector tim5_handler
    weak_vector osc32k_cal_handler
    weak_vector osc32k_wakeup_handler
#endif
