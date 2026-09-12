// startup_rp2040.cpp - vector table + reset path for the RP2040, compiled
// into EVERY image of this project (rp2040_add_app() lists it alongside
// the second-stage bootloader and the app's own source). No SDK crt0, no
// SystemInit, no SystemCoreClock: this file, the boot2 stage and the
// linker script are the whole crt.
//
// WHAT RUNS BEFORE THIS. Out of reset the bootrom runs core 0 on the
// ring oscillator (clk_sys and clk_ref at roughly 6.5 MHz, 2.7), finds
// the flash, copies its first 256 bytes to SRAM, checks their CRC and
// runs them: that stage programs the XIP SSI for the flash chip and
// leaves through the vector table at flash offset 0x100 - it writes
// VTOR with that address and loads SP and PC from its first two words
// (src/glue/boot2_*.S). So when Reset_Handler starts, VTOR already
// points at `vector_table` below, the stack pointer is __stack_top (the
// top of the upper unstriped SRAM bank, ld/), every peripheral the reset
// controller governs is still HELD IN RESET (2.14.1: software releases
// what it uses - the stratum's Resets does), and core 1 is asleep in the
// bootrom's launch protocol, waiting on the inter-core FIFO. Nothing
// here touches the clocks (SysClock::init() in main() owns the tree) or
// VTOR.
//
// The vector table is the chip's own: sixteen core entries and the 32
// NVIC lines of table 2.3.2 - 26 peripheral lines and six spare ones the
// software may pend. THE NAMES are the pico-sdk's, the spelling every
// Pico tool and user knows: the core exceptions as isr_nmi /
// isr_hardfault / isr_svcall / isr_pendsv / isr_systick and the
// peripheral lines as isr_<block>. THE PERIPHERAL NAMES ARE MACROS: the
// SDK's vector slots are the symbols isr_irq0..isr_irq31, and
// hardware/regs/intctrl.h spells isr_uart0 as `#define isr_uart0
// isr_irq20` and so on - so the symbol in the image is the slot's, and
// the friendly name is what one WRITES. This file includes that header
// for exactly that reason: the name it declares weak and the name an
// app defines strong must expand to the same symbol (an app reaches
// the macros through any header of the stratum). Every handler is a
// WEAK alias for Default_Handler (a spin), and an app binds a vector by
// defining the strong symbol itself -
//
//   extern "C" void isr_systick() { brio::Ticker::tick(); }
//   extern "C" void isr_uart0() { ... Serial::isr() ... }
//
// A name outside this table compiles, links, and lands in
// Default_Handler's silent spin. EVERY LINE REACHES BOTH CORES' NVICs
// (2.3.2): this table is fetched by whichever core has VTOR at it, and a
// line is served by the core that enabled it - by exactly one core, the
// rule the stratum keeps.
//
// Reset_Handler does the minimum an image needs: copy .data from flash
// (through the XIP window the stage just opened), zero .bss, walk
// .init_array itself, call main(). .noinit is neither loaded nor
// zeroed, so the PanicRecord breadcrumb survives a warm reset.
//
// HardFault gets its own weak spin rather than folding into
// Default_Handler: on ARMv6-M a BKPT with no debugger escalates here,
// and a distinct symbol makes the wreck legible in a backtrace.

#include <stdint.h>

#include "hardware/platform_defs.h"
#include "hardware/regs/intctrl.h"

extern "C" {

extern uint32_t __data_load_start;
extern uint32_t __data_start;
extern uint32_t __data_end;
extern uint32_t __bss_start;
extern uint32_t __bss_end;
extern void (*__preinit_array_start[])();
extern void (*__preinit_array_end[])();
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();

// Declared as a FUNCTION so its address can sit in the Handler array
// with no reinterpret_cast (constexpr initialization would reject one).
// Entry 0 of the table is the initial stack pointer; nothing calls it.
void __stack_top();

}  // extern "C"

int main();

extern "C" {

void Default_Handler()
{
    for (;;) {}
}

// WEAK and a DEFINITION (not an alias): an app that binds isr_hardfault
// must not hit a multiple-definition error.
__attribute__((weak)) void isr_hardfault()
{
    for (;;) {}
}

// abort(): the one libc symbol a brio image references (libstdc++'s
// throw sites under -fno-exceptions). A spin, so the frame survives for
// the debugger, and so newlib's abort() does not drag the syscall stubs
// in.
[[noreturn]] void abort()
{
    for (;;) {}
}

[[noreturn]] void Reset_Handler()
{
    const uint32_t* src = &__data_load_start;
    for (uint32_t* dst = &__data_start; dst != &__data_end; ) {
        *dst++ = *src++;
    }
    for (uint32_t* dst = &__bss_start; dst != &__bss_end; ) {
        *dst++ = 0u;
    }
    for (auto** p = __preinit_array_start; p != __preinit_array_end; ++p) {
        (*p)();
    }
    for (auto** p = __init_array_start; p != __init_array_end; ++p) {
        (*p)();
    }
    main();
    for (;;) {}
}

// Core exceptions (the SDK's spelling).
void isr_nmi()        __attribute__((weak, alias("Default_Handler")));
void isr_svcall()     __attribute__((weak, alias("Default_Handler")));
void isr_pendsv()     __attribute__((weak, alias("Default_Handler")));
void isr_systick()    __attribute__((weak, alias("Default_Handler")));

// The 32 NVIC lines, in IRQn order (RP2040.h / datasheet table 2.3.2).
void isr_timer_0()    __attribute__((weak, alias("Default_Handler")));  // 0
void isr_timer_1()    __attribute__((weak, alias("Default_Handler")));  // 1
void isr_timer_2()    __attribute__((weak, alias("Default_Handler")));  // 2
void isr_timer_3()    __attribute__((weak, alias("Default_Handler")));  // 3
void isr_pwm_wrap()   __attribute__((weak, alias("Default_Handler")));  // 4
void isr_usbctrl()    __attribute__((weak, alias("Default_Handler")));  // 5
void isr_xip()        __attribute__((weak, alias("Default_Handler")));  // 6
void isr_pio0_0()     __attribute__((weak, alias("Default_Handler")));  // 7
void isr_pio0_1()     __attribute__((weak, alias("Default_Handler")));  // 8
void isr_pio1_0()     __attribute__((weak, alias("Default_Handler")));  // 9
void isr_pio1_1()     __attribute__((weak, alias("Default_Handler")));  // 10
void isr_dma_0()      __attribute__((weak, alias("Default_Handler")));  // 11
void isr_dma_1()      __attribute__((weak, alias("Default_Handler")));  // 12
void isr_io_bank0()   __attribute__((weak, alias("Default_Handler")));  // 13
void isr_io_qspi()    __attribute__((weak, alias("Default_Handler")));  // 14
void isr_sio_proc0()  __attribute__((weak, alias("Default_Handler")));  // 15
void isr_sio_proc1()  __attribute__((weak, alias("Default_Handler")));  // 16
void isr_clocks()     __attribute__((weak, alias("Default_Handler")));  // 17
void isr_spi0()       __attribute__((weak, alias("Default_Handler")));  // 18
void isr_spi1()       __attribute__((weak, alias("Default_Handler")));  // 19
void isr_uart0()      __attribute__((weak, alias("Default_Handler")));  // 20
void isr_uart1()      __attribute__((weak, alias("Default_Handler")));  // 21
void isr_adc_fifo()   __attribute__((weak, alias("Default_Handler")));  // 22
void isr_i2c0()       __attribute__((weak, alias("Default_Handler")));  // 23
void isr_i2c1()       __attribute__((weak, alias("Default_Handler")));  // 24
void isr_rtc()        __attribute__((weak, alias("Default_Handler")));  // 25
void isr_spare_0()    __attribute__((weak, alias("Default_Handler")));  // 26
void isr_spare_1()    __attribute__((weak, alias("Default_Handler")));  // 27
void isr_spare_2()    __attribute__((weak, alias("Default_Handler")));  // 28
void isr_spare_3()    __attribute__((weak, alias("Default_Handler")));  // 29
void isr_spare_4()    __attribute__((weak, alias("Default_Handler")));  // 30
void isr_spare_5()    __attribute__((weak, alias("Default_Handler")));  // 31

}  // extern "C"

using Handler = void (*)();

__attribute__((section(".vectors"), used))
constexpr Handler vector_table[] = {
    __stack_top,                               // 0: initial SP
    Reset_Handler,                             // 1: reset
    isr_nmi,                                   // 2
    isr_hardfault,                             // 3
    nullptr, nullptr, nullptr, nullptr,        // 4..7 reserved on v6-M
    nullptr, nullptr, nullptr,                 // 8..10 reserved
    isr_svcall,                                // 11
    nullptr, nullptr,                          // 12..13 reserved
    isr_pendsv,                                // 14
    isr_systick,                               // 15
    // NVIC lines 0..31:
    isr_timer_0, isr_timer_1, isr_timer_2, isr_timer_3,
    isr_pwm_wrap, isr_usbctrl, isr_xip, isr_pio0_0,
    isr_pio0_1, isr_pio1_0, isr_pio1_1, isr_dma_0,
    isr_dma_1, isr_io_bank0, isr_io_qspi, isr_sio_proc0,
    isr_sio_proc1, isr_clocks, isr_spi0, isr_spi1,
    isr_uart0, isr_uart1, isr_adc_fifo, isr_i2c0,
    isr_i2c1, isr_rtc, isr_spare_0, isr_spare_1,
    isr_spare_2, isr_spare_3, isr_spare_4, isr_spare_5,
};
