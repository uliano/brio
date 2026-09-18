// startup_rp2350_arm.cpp - vector table, image metadata and reset path for
// an RP2350 running its Cortex-M33 cores, compiled into every image of the
// arm presets (rp2350_add_app() lists it alongside the app's own source).
// No SDK crt0, no SystemInit, no SystemCoreClock: this file and the linker
// script are the whole crt. Its RISC-V twin is startup_rp2350_riscv.S, and
// the two agree on every name an app binds.
//
// WHAT RUNS BEFORE THIS. The bootrom runs core 0 out of reset, scans the
// flash for a block loop containing a valid IMAGE_DEF, sets the QSPI
// interface up well enough to execute from (03h serial reads at CLKDIV 12,
// datasheet 5.1.4 - there is no second-stage bootloader on this chip,
// 5.9.5), and enters the image. The IMAGE_DEF below says the image is an
// Arm one, so the bootrom leaves both cores in the Arm architecture
// (3.9.1: ARCHSEL is sampled at processor reset, and a RISC-V image
// flashed over this one switches them back with no button and no OTP
// write). With no explicit entry point in the metadata the bootrom takes
// the image for a Cortex-M vector table and enters through its first two
// words: the stack pointer and the reset handler.
//
// WHAT IS NOT GUARANTEED. `reset run` from a debugger is SYSRESETREQ, and
// that resets the cores and NOT the clock tree, the pads or the
// peripherals: an image may start on the previous image's PLL, with pads
// a previous program configured. Nothing here assumes a reset state it
// did not make itself, and neither may a driver (brio/rp2350/clock.hpp's
// init() walks the tree back to a source it started before it touches a
// PLL, for exactly this reason).
//
// THE THREE THINGS RESET_HANDLER DOES BEFORE .data:
//  - VTOR is written with this table's address. The bootrom enters
//    through the table without necessarily pointing the core at it, and
//    every later exception must find it.
//  - MSPLIM is zeroed. ARMv8-M has a stack-limit register the bootrom or
//    a debugger may leave set; brio does not use it (a limit nothing
//    arms is a fault waiting at an inherited value), and zero is what a
//    cold core has.
//  - CPACR gets full access to CP10 and CP11, the FPU, because this
//    project builds the M33 with the HARD-FLOAT ABI: the first function
//    with a float argument executes a coprocessor instruction, and an
//    unenabled FPU turns that into a UsageFault. The same act as the
//    stm32f4 crt's, for the same reason.
//
// THE VECTOR TABLE is the chip's own: sixteen core entries (the M33's
// four configurable faults among them, where the RP2040's M0+ had
// reserved words) and the 52 system interrupt lines of datasheet 3.2.
// THE NAMES are the pico-sdk's, the spelling every Pico tool and user
// knows: the core exceptions as isr_nmi / isr_hardfault / isr_memmanage /
// ... / isr_systick, the peripheral lines as isr_<block>. THE PERIPHERAL
// NAMES ARE MACROS: the slots are the symbols isr_irq0..isr_irq51 and
// hardware/regs/intctrl.h spells isr_uart0 as `#define isr_uart0
// isr_irq33`, so the symbol in the image is the slot's and the friendly
// name is what one writes. This file includes that header for exactly
// that reason: the name declared weak here and the name an app defines
// strong must expand to the same symbol.
//
//   extern "C" void isr_systick() { brio::Ticker::tick(); }
//   extern "C" void isr_uart0() { Serial::isr(); }
//
// AND THOSE TWO LINES COMPILE UNCHANGED ON THE RISC-V PRESET: the other
// crt binds the same names, isr_systick to the machine timer trap and
// isr_irqN through Hazard3's own dispatch. One app source, two
// architectures, one set of vector bindings.
//
// A name outside this table compiles, links, and lands in
// Default_Handler's silent spin. EVERY LINE REACHES BOTH CORES' NVICs:
// this table is fetched by whichever core has VTOR at it, and a line is
// served by the core that enabled it - by exactly one core, the rule the
// stratum keeps.
//
// .noinit is neither loaded nor zeroed, so the PanicRecord breadcrumb
// survives a warm reset.

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

// Declared as a FUNCTION so its address can sit in the Handler array with
// no reinterpret_cast. Entry 0 of the table is the initial stack pointer;
// nothing calls it.
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

// abort(): the one libc symbol a brio image references (libstdc++'s throw
// sites under -fno-exceptions). A spin, so the frame survives for the
// debugger, and so newlib's abort() does not drag the syscall stubs in.
[[noreturn]] void abort()
{
    for (;;) {}
}

[[noreturn]] void Reset_Handler();

}  // extern "C"

using Handler = void (*)();

extern "C" {

// Core exceptions (the SDK's spelling; the four configurable faults are
// the M33's own, where the RP2040's table had reserved words).
void isr_nmi()           __attribute__((weak, alias("Default_Handler")));
void isr_memmanage()     __attribute__((weak, alias("Default_Handler")));
void isr_busfault()      __attribute__((weak, alias("Default_Handler")));
void isr_usagefault()    __attribute__((weak, alias("Default_Handler")));
void isr_securefault()   __attribute__((weak, alias("Default_Handler")));
void isr_svcall()        __attribute__((weak, alias("Default_Handler")));
void isr_debugmonitor()  __attribute__((weak, alias("Default_Handler")));
void isr_pendsv()        __attribute__((weak, alias("Default_Handler")));
void isr_systick()       __attribute__((weak, alias("Default_Handler")));

// The 52 system interrupt lines, in IRQn order (RP2350.h / datasheet 3.2).
void isr_timer0_0()      __attribute__((weak, alias("Default_Handler")));  // 0
void isr_timer0_1()      __attribute__((weak, alias("Default_Handler")));  // 1
void isr_timer0_2()      __attribute__((weak, alias("Default_Handler")));  // 2
void isr_timer0_3()      __attribute__((weak, alias("Default_Handler")));  // 3
void isr_timer1_0()      __attribute__((weak, alias("Default_Handler")));  // 4
void isr_timer1_1()      __attribute__((weak, alias("Default_Handler")));  // 5
void isr_timer1_2()      __attribute__((weak, alias("Default_Handler")));  // 6
void isr_timer1_3()      __attribute__((weak, alias("Default_Handler")));  // 7
void isr_pwm_wrap_0()    __attribute__((weak, alias("Default_Handler")));  // 8
void isr_pwm_wrap_1()    __attribute__((weak, alias("Default_Handler")));  // 9
void isr_dma_0()         __attribute__((weak, alias("Default_Handler")));  // 10
void isr_dma_1()         __attribute__((weak, alias("Default_Handler")));  // 11
void isr_dma_2()         __attribute__((weak, alias("Default_Handler")));  // 12
void isr_dma_3()         __attribute__((weak, alias("Default_Handler")));  // 13
void isr_usbctrl()       __attribute__((weak, alias("Default_Handler")));  // 14
void isr_pio0_0()        __attribute__((weak, alias("Default_Handler")));  // 15
void isr_pio0_1()        __attribute__((weak, alias("Default_Handler")));  // 16
void isr_pio1_0()        __attribute__((weak, alias("Default_Handler")));  // 17
void isr_pio1_1()        __attribute__((weak, alias("Default_Handler")));  // 18
void isr_pio2_0()        __attribute__((weak, alias("Default_Handler")));  // 19
void isr_pio2_1()        __attribute__((weak, alias("Default_Handler")));  // 20
void isr_io_bank0()      __attribute__((weak, alias("Default_Handler")));  // 21
void isr_io_bank0_ns()   __attribute__((weak, alias("Default_Handler")));  // 22
void isr_io_qspi()       __attribute__((weak, alias("Default_Handler")));  // 23
void isr_io_qspi_ns()    __attribute__((weak, alias("Default_Handler")));  // 24
void isr_sio_fifo()      __attribute__((weak, alias("Default_Handler")));  // 25
void isr_sio_bell()      __attribute__((weak, alias("Default_Handler")));  // 26
void isr_sio_fifo_ns()   __attribute__((weak, alias("Default_Handler")));  // 27
void isr_sio_bell_ns()   __attribute__((weak, alias("Default_Handler")));  // 28
void isr_sio_mtimecmp()  __attribute__((weak, alias("Default_Handler")));  // 29
void isr_clocks()        __attribute__((weak, alias("Default_Handler")));  // 30
void isr_spi0()          __attribute__((weak, alias("Default_Handler")));  // 31
void isr_spi1()          __attribute__((weak, alias("Default_Handler")));  // 32
void isr_uart0()         __attribute__((weak, alias("Default_Handler")));  // 33
void isr_uart1()         __attribute__((weak, alias("Default_Handler")));  // 34
void isr_adc_fifo()      __attribute__((weak, alias("Default_Handler")));  // 35
void isr_i2c0()          __attribute__((weak, alias("Default_Handler")));  // 36
void isr_i2c1()          __attribute__((weak, alias("Default_Handler")));  // 37
void isr_otp()           __attribute__((weak, alias("Default_Handler")));  // 38
void isr_trng()          __attribute__((weak, alias("Default_Handler")));  // 39
void isr_proc0_cti()     __attribute__((weak, alias("Default_Handler")));  // 40
void isr_proc1_cti()     __attribute__((weak, alias("Default_Handler")));  // 41
void isr_pll_sys()       __attribute__((weak, alias("Default_Handler")));  // 42
void isr_pll_usb()       __attribute__((weak, alias("Default_Handler")));  // 43
void isr_powman_pow()    __attribute__((weak, alias("Default_Handler")));  // 44
void isr_powman_timer()  __attribute__((weak, alias("Default_Handler")));  // 45
void isr_spare_0()       __attribute__((weak, alias("Default_Handler")));  // 46
void isr_spare_1()       __attribute__((weak, alias("Default_Handler")));  // 47
void isr_spare_2()       __attribute__((weak, alias("Default_Handler")));  // 48
void isr_spare_3()       __attribute__((weak, alias("Default_Handler")));  // 49
void isr_spare_4()       __attribute__((weak, alias("Default_Handler")));  // 50
void isr_spare_5()       __attribute__((weak, alias("Default_Handler")));  // 51

}  // extern "C"

__attribute__((section(".vectors"), used))
constexpr Handler vector_table[] = {
    __stack_top,                               // 0: initial SP
    Reset_Handler,                             // 1: reset
    isr_nmi,                                   // 2
    isr_hardfault,                             // 3
    isr_memmanage,                             // 4
    isr_busfault,                              // 5
    isr_usagefault,                            // 6
    isr_securefault,                           // 7
    nullptr, nullptr, nullptr,                 // 8..10 reserved
    isr_svcall,                                // 11
    isr_debugmonitor,                          // 12
    nullptr,                                   // 13 reserved
    isr_pendsv,                                // 14
    isr_systick,                               // 15
    // The 52 system interrupt lines:
    isr_timer0_0, isr_timer0_1, isr_timer0_2, isr_timer0_3,
    isr_timer1_0, isr_timer1_1, isr_timer1_2, isr_timer1_3,
    isr_pwm_wrap_0, isr_pwm_wrap_1, isr_dma_0, isr_dma_1,
    isr_dma_2, isr_dma_3, isr_usbctrl, isr_pio0_0,
    isr_pio0_1, isr_pio1_0, isr_pio1_1, isr_pio2_0,
    isr_pio2_1, isr_io_bank0, isr_io_bank0_ns, isr_io_qspi,
    isr_io_qspi_ns, isr_sio_fifo, isr_sio_bell, isr_sio_fifo_ns,
    isr_sio_bell_ns, isr_sio_mtimecmp, isr_clocks, isr_spi0,
    isr_spi1, isr_uart0, isr_uart1, isr_adc_fifo,
    isr_i2c0, isr_i2c1, isr_otp, isr_trng,
    isr_proc0_cti, isr_proc1_cti, isr_pll_sys, isr_pll_usb,
    isr_powman_pow, isr_powman_timer, isr_spare_0, isr_spare_1,
    isr_spare_2, isr_spare_3, isr_spare_4, isr_spare_5,
};

static_assert(sizeof(vector_table) / sizeof(vector_table[0]) == 16 + NUM_IRQS,
              "the table is sixteen core entries and one per system interrupt line");

// THE IMAGE METADATA, right behind the vector table and so well inside
// the first 4 kB the bootrom scans (datasheet 5.9.5.1): the twenty bytes
// that say "this is an executable, Secure, Arm, for this chip", as five
// little-endian words - a start marker, one one-word item naming the
// image type, the last-item terminator, a relative link to the next
// block in the loop (zero = a loop of this block alone) and an end
// marker. It is a CONSTANT, not a structure: the bootrom reads bytes.
// The word that decides the architecture is the second one, and it is
// the ONLY difference from the RISC-V crt's block.
__attribute__((section(".image_def"), used))
constexpr uint32_t image_def[] = {
    0xffffded3u,   // PICOBIN_BLOCK_MARKER_START
    0x10210142u,   // IMAGE_TYPE item: EXE, Secure, Arm, RP2350
    0x000001ffu,   // LAST item, one word
    0x00000000u,   // next block in the loop: this one
    0xab123579u,   // PICOBIN_BLOCK_MARKER_END
};

extern "C" {

[[noreturn]] void Reset_Handler()
{
    // The three acts before any C++ state exists (the file header says
    // why each): the table, the stack limit, the FPU.
    constexpr uint32_t ppb_base = 0xe0000000u;
    *reinterpret_cast<volatile uint32_t*>(ppb_base + 0xed08u) =   // VTOR
        reinterpret_cast<uint32_t>(&vector_table[0]);
    __asm__ volatile("msr msplim, %0" :: "r"(0u));
    volatile uint32_t& cpacr = *reinterpret_cast<volatile uint32_t*>(ppb_base + 0xed88u);
    cpacr = cpacr | (0xfu << 20);                                 // CP10, CP11: full access
    __asm__ volatile("dsb" ::: "memory");
    __asm__ volatile("isb" ::: "memory");

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

// The ELF entry point the linker script names, so that a debugger and
// `program` agree with the bootrom about where the image starts.
[[noreturn]] void _start() __attribute__((alias("Reset_Handler")));

}  // extern "C"
