// startup_stm32f411.cpp - vector table + reset path for the STM32F411,
// compiled into EVERY image of this project (stm32_add_app() lists it
// alongside the app's own source). No ST startup template, no SystemInit,
// no SystemCoreClock: this file and the part's ld/ script are the whole crt.
//
// The app-binds-the-vector rule, ARM edition: every handler below is a
// WEAK alias for Default_Handler (a spin), and an app binds a vector by
// defining the strong symbol itself -
//
//   extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
//   extern "C" void USART1_IRQHandler() { ... Serial::isr() ... }
//
// THE NAMES. The device header declares the IRQn ENUMERATORS
// (stm32f411xe.h) but no handler names at all; the spelling every STM32
// tool, template and user knows comes from ST's own startup file
// (cmsis-device-f4 Source/Templates/gcc/startup_stm32f411xe.s, the
// authority cited here, deliberately not vendored): the core exceptions
// in CMSIS-classic form (NMI_Handler, HardFault_Handler,
// MemManage_Handler, BusFault_Handler, UsageFault_Handler, SVC_Handler,
// DebugMon_Handler, PendSV_Handler, SysTick_Handler) and the peripheral
// lines as <acronym>_IRQHandler, with the ACRONYM of RM0383 table 38. On this
// family a line is mostly ONE peripheral's - the sharing the table does
// show is the advanced timers' (TIM1_BRK_TIM9, TIM1_UP_TIM10,
// TIM1_TRG_COM_TIM11, and TIM8's with TIM12/13/14), TIM6 with the DAC,
// the EXTI lines 5..9 and 10..15 grouped, and the I2C's two vectors per
// instance (EV and ER). A handler for a shared line asks each of its
// peripherals in turn. A name this table does not reference compiles,
// links, and lands in Default_Handler's silent spin. The table has
// holes: the holes (19-22 CAN1, 39 USART3, 43-46 TIM8, 48 FMC, 52-55 UART4/5, TIM6/7, 61-66 ETH and CAN2, 74-80 OTG_HS, DCMI, CRYP, HASH_RNG, 82-83 UART7/8) are other parts' of the family; the F411 has no peripheral there.
//
// Reset_Handler does the minimum an image needs - and one thing an
// ARMv7-M with an FPU adds: ENABLE THE COPROCESSOR FIRST. This project
// builds with the hard-float ABI, so the compiler may keep a float in an
// s register anywhere, including the initializers .init_array runs; with
// CPACR's CP10/CP11 fields at their reset value (no access) the first
// such instruction is a UsageFault. Full access is written, then DSB and
// ISB so the write is in force before the next instruction (PM0214
// 4.6.1). Lazy stacking (FPCCR.ASPEN/LSPEN) is left at its reset value,
// on: an exception stacks the FPU context only if the handler uses the
// FPU. Then .data from flash, .bss zeroed, .init_array walked, main().
// It does NOT touch the clock (SysClock::init() in main() owns the tree)
// and does NOT write VTOR (the table is fetched through the boot alias
// at 0 - see the linker script). .noinit is neither loaded nor zeroed,
// so the PanicRecord breadcrumb survives a warm reset.
//
// HardFault gets its own weak spin rather than folding into
// Default_Handler: a BKPT with no debugger escalates here, and a
// distinct symbol makes the wreck legible in a backtrace. The three
// configurable faults (MemManage, BusFault, UsageFault) are DISABLED at
// reset (SHCSR) and escalate to HardFault too; they are weak aliases
// of Default_Handler until the fault-handling pass gives them a body.

#include <stdint.h>

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

// WEAK and a DEFINITION (not an alias): an app that binds HardFault_Handler
// must not hit a multiple-definition error.
__attribute__((weak)) void HardFault_Handler()
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
    // CPACR at 0xE000ED88: CP10 (bits 21:20) and CP11 (bits 23:22) to
    // full access - the hard-float ABI's precondition, before any C++.
    *reinterpret_cast<volatile uint32_t*>(0xE000ED88u) |= (3u << 20) | (3u << 22);
    asm volatile("dsb\n\tisb" ::: "memory");

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

// Core exceptions (CMSIS-classic spelling, the ST startup's).
void NMI_Handler()        __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler()  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler()   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler() __attribute__((weak, alias("Default_Handler")));
void SVC_Handler()        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler()   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler()     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler()    __attribute__((weak, alias("Default_Handler")));

// Peripheral lines, in IRQn order (stm32f411xe.h / RM0383 table 38).
void WWDG_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 0
void PVD_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 1
void TAMP_STAMP_IRQHandler()                    __attribute__((weak, alias("Default_Handler")));  // 2
void RTC_WKUP_IRQHandler()                      __attribute__((weak, alias("Default_Handler")));  // 3
void FLASH_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 4
void RCC_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 5
void EXTI0_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 6
void EXTI1_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 7
void EXTI2_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 8
void EXTI3_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 9
void EXTI4_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 10
void DMA1_Stream0_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 11
void DMA1_Stream1_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 12
void DMA1_Stream2_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 13
void DMA1_Stream3_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 14
void DMA1_Stream4_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 15
void DMA1_Stream5_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 16
void DMA1_Stream6_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 17
void ADC_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 18
void EXTI9_5_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 23
void TIM1_BRK_TIM9_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  // 24
void TIM1_UP_TIM10_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  // 25
void TIM1_TRG_COM_TIM11_IRQHandler()            __attribute__((weak, alias("Default_Handler")));  // 26
void TIM1_CC_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 27
void TIM2_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 28
void TIM3_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 29
void TIM4_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 30
void I2C1_EV_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 31
void I2C1_ER_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 32
void I2C2_EV_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 33
void I2C2_ER_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 34
void SPI1_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 35
void SPI2_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 36
void USART1_IRQHandler()                        __attribute__((weak, alias("Default_Handler")));  // 37
void USART2_IRQHandler()                        __attribute__((weak, alias("Default_Handler")));  // 38
void EXTI15_10_IRQHandler()                     __attribute__((weak, alias("Default_Handler")));  // 40
void RTC_Alarm_IRQHandler()                     __attribute__((weak, alias("Default_Handler")));  // 41
void OTG_FS_WKUP_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  // 42
void DMA1_Stream7_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 47
void SDIO_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 49
void TIM5_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 50
void SPI3_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 51
void DMA2_Stream0_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 56
void DMA2_Stream1_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 57
void DMA2_Stream2_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 58
void DMA2_Stream3_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 59
void DMA2_Stream4_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 60
void OTG_FS_IRQHandler()                        __attribute__((weak, alias("Default_Handler")));  // 67
void DMA2_Stream5_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 68
void DMA2_Stream6_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 69
void DMA2_Stream7_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  // 70
void USART6_IRQHandler()                        __attribute__((weak, alias("Default_Handler")));  // 71
void I2C3_EV_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 72
void I2C3_ER_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 73
void FPU_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 81
void SPI4_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 84
void SPI5_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 85

}  // extern "C"

using Handler = void (*)();

__attribute__((section(".vectors"), used))
constexpr Handler vector_table[] = {
    __stack_top,                               // 0: initial SP
    Reset_Handler,                             // 1: reset
    NMI_Handler,                               // 2
    HardFault_Handler,                         // 3
    MemManage_Handler,                         // 4
    BusFault_Handler,                          // 5
    UsageFault_Handler,                        // 6
    nullptr, nullptr, nullptr, nullptr,        // 7..10 reserved
    SVC_Handler,                               // 11
    DebugMon_Handler,                          // 12
    nullptr,                                   // 13 reserved
    PendSV_Handler,                            // 14
    SysTick_Handler,                           // 15
    // Peripheral lines 0..85:
    WWDG_IRQHandler, PVD_IRQHandler, TAMP_STAMP_IRQHandler, RTC_WKUP_IRQHandler,  // 0..3
    FLASH_IRQHandler, RCC_IRQHandler, EXTI0_IRQHandler, EXTI1_IRQHandler,  // 4..7
    EXTI2_IRQHandler, EXTI3_IRQHandler, EXTI4_IRQHandler, DMA1_Stream0_IRQHandler,  // 8..11
    DMA1_Stream1_IRQHandler, DMA1_Stream2_IRQHandler, DMA1_Stream3_IRQHandler, DMA1_Stream4_IRQHandler,  // 12..15
    DMA1_Stream5_IRQHandler, DMA1_Stream6_IRQHandler, ADC_IRQHandler, nullptr,  // 16..19
    nullptr, nullptr, nullptr, EXTI9_5_IRQHandler,  // 20..23
    TIM1_BRK_TIM9_IRQHandler, TIM1_UP_TIM10_IRQHandler, TIM1_TRG_COM_TIM11_IRQHandler, TIM1_CC_IRQHandler,  // 24..27
    TIM2_IRQHandler, TIM3_IRQHandler, TIM4_IRQHandler, I2C1_EV_IRQHandler,  // 28..31
    I2C1_ER_IRQHandler, I2C2_EV_IRQHandler, I2C2_ER_IRQHandler, SPI1_IRQHandler,  // 32..35
    SPI2_IRQHandler, USART1_IRQHandler, USART2_IRQHandler, nullptr,  // 36..39
    EXTI15_10_IRQHandler, RTC_Alarm_IRQHandler, OTG_FS_WKUP_IRQHandler, nullptr,  // 40..43
    nullptr, nullptr, nullptr, DMA1_Stream7_IRQHandler,  // 44..47
    nullptr, SDIO_IRQHandler, TIM5_IRQHandler, SPI3_IRQHandler,  // 48..51
    nullptr, nullptr, nullptr, nullptr,  // 52..55
    DMA2_Stream0_IRQHandler, DMA2_Stream1_IRQHandler, DMA2_Stream2_IRQHandler, DMA2_Stream3_IRQHandler,  // 56..59
    DMA2_Stream4_IRQHandler, nullptr, nullptr, nullptr,  // 60..63
    nullptr, nullptr, nullptr, OTG_FS_IRQHandler,  // 64..67
    DMA2_Stream5_IRQHandler, DMA2_Stream6_IRQHandler, DMA2_Stream7_IRQHandler, USART6_IRQHandler,  // 68..71
    I2C3_EV_IRQHandler, I2C3_ER_IRQHandler, nullptr, nullptr,  // 72..75
    nullptr, nullptr, nullptr, nullptr,  // 76..79
    nullptr, FPU_IRQHandler, nullptr, nullptr,  // 80..83
    SPI4_IRQHandler, SPI5_IRQHandler,  // 84..85
};
