// startup_stm32g0b1.cpp - vector table + reset path for the STM32G0B1
// (and, by the same header, the G0C1's peripheral subset), compiled into
// EVERY image of this project (stm32_add_app() lists it alongside the
// app's own source - the glue slot the samc project's startup_samc21.cpp
// also fills). No ST startup template, no SystemInit, no
// SystemCoreClock: this file and ld/stm32g0b1re.ld are the whole crt.
//
// The app-binds-the-vector rule, ARM edition: every handler below is a
// WEAK alias for Default_Handler (a spin), and an app binds a vector by
// defining the strong symbol itself -
//
//   extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
//   extern "C" void USART2_LPUART2_IRQHandler() { ... Serial::isr() ... }
//
// exactly as a SAM app writes SERCOM5_Handler and an AVR app ISR(...).
//
// THE NAMES. The device header declares the IRQn ENUMERATORS
// (stm32g0b1xx.h) but no handler names at all; the spelling every STM32
// tool, template and user knows comes from ST's own startup file
// (cmsis-device-g0 Source/Templates/gcc/startup_stm32g0b1xx.s, the
// authority cited here, deliberately not vendored): the core exceptions
// in CMSIS-classic form (NMI_Handler, HardFault_Handler, SVC_Handler,
// PendSV_Handler, SysTick_Handler) and the peripheral lines as
// <acronym>_IRQHandler, with the ACRONYM of RM0444 table 61 - which is
// where this family's vector sharing shows: one line for USART2 AND
// LPUART2, one for USART3/4/5/6 AND LPUART1, one for TIM3 AND TIM4, and
// so on. A handler for a shared line asks each of its peripherals in
// turn. The samc lesson stands: a name this table does not reference
// compiles, links, and lands in Default_Handler's silent spin.
//
// Reset_Handler does the minimum an image needs: copy .data from flash,
// zero .bss, walk .init_array itself, call main(). It does NOT touch
// the clock (SysClock::init() in main() owns the tree) and does NOT
// write VTOR (the table is fetched through the boot alias at 0 - see
// the linker script). .noinit is neither loaded nor zeroed, so the
// PanicRecord breadcrumb survives a warm reset.
//
// HardFault gets its own weak spin rather than folding into
// Default_Handler: on ARMv6-M a BKPT with no debugger escalates here,
// and a distinct symbol makes the wreck legible in a backtrace.

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
// in - see samc/src/glue/startup_samc21.cpp for the full argument.
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

// Core exceptions (CMSIS-classic spelling, the ST startup's).
void NMI_Handler()        __attribute__((weak, alias("Default_Handler")));
void SVC_Handler()        __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler()     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler()    __attribute__((weak, alias("Default_Handler")));

// Peripheral lines, in IRQn order (stm32g0b1xx.h / RM0444 table 61).
void WWDG_IRQHandler()                              __attribute__((weak, alias("Default_Handler")));  // 0
void PVD_VDDIO2_IRQHandler()                        __attribute__((weak, alias("Default_Handler")));  // 1
void RTC_TAMP_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 2
void FLASH_IRQHandler()                             __attribute__((weak, alias("Default_Handler")));  // 3
void RCC_CRS_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 4
void EXTI0_1_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 5
void EXTI2_3_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 6
void EXTI4_15_IRQHandler()                          __attribute__((weak, alias("Default_Handler")));  // 7
void USB_UCPD1_2_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 8
void DMA1_Channel1_IRQHandler()                     __attribute__((weak, alias("Default_Handler")));  // 9
void DMA1_Channel2_3_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  // 10
void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler() __attribute__((weak, alias("Default_Handler")));  // 11
void ADC1_COMP_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 12
void TIM1_BRK_UP_TRG_COM_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  // 13
void TIM1_CC_IRQHandler()                           __attribute__((weak, alias("Default_Handler")));  // 14
void TIM2_IRQHandler()                              __attribute__((weak, alias("Default_Handler")));  // 15
void TIM3_TIM4_IRQHandler()                         __attribute__((weak, alias("Default_Handler")));  // 16
void TIM6_DAC_LPTIM1_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  // 17
void TIM7_LPTIM2_IRQHandler()                       __attribute__((weak, alias("Default_Handler")));  // 18
void TIM14_IRQHandler()                             __attribute__((weak, alias("Default_Handler")));  // 19
void TIM15_IRQHandler()                             __attribute__((weak, alias("Default_Handler")));  // 20
void TIM16_FDCAN_IT0_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  // 21
void TIM17_FDCAN_IT1_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  // 22
void I2C1_IRQHandler()                              __attribute__((weak, alias("Default_Handler")));  // 23
void I2C2_3_IRQHandler()                            __attribute__((weak, alias("Default_Handler")));  // 24
void SPI1_IRQHandler()                              __attribute__((weak, alias("Default_Handler")));  // 25
void SPI2_3_IRQHandler()                            __attribute__((weak, alias("Default_Handler")));  // 26
void USART1_IRQHandler()                            __attribute__((weak, alias("Default_Handler")));  // 27
void USART2_LPUART2_IRQHandler()                    __attribute__((weak, alias("Default_Handler")));  // 28
void USART3_4_5_6_LPUART1_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  // 29
void CEC_IRQHandler()                               __attribute__((weak, alias("Default_Handler")));  // 30
// Position 31 (AES/RNG) is the G0C1's; the G0B1 has no peripheral there.

}  // extern "C"

using Handler = void (*)();

__attribute__((section(".vectors"), used))
constexpr Handler vector_table[] = {
    __stack_top,                               // 0: initial SP
    Reset_Handler,                             // 1: reset
    NMI_Handler,                               // 2
    HardFault_Handler,                         // 3
    nullptr, nullptr, nullptr, nullptr,        // 4..7 reserved on v6-M
    nullptr, nullptr, nullptr,                 // 8..10 reserved
    SVC_Handler,                               // 11
    nullptr, nullptr,                          // 12..13 reserved
    PendSV_Handler,                            // 14
    SysTick_Handler,                           // 15
    // Peripheral lines 0..31:
    WWDG_IRQHandler, PVD_VDDIO2_IRQHandler, RTC_TAMP_IRQHandler, FLASH_IRQHandler,
    RCC_CRS_IRQHandler, EXTI0_1_IRQHandler, EXTI2_3_IRQHandler, EXTI4_15_IRQHandler,
    USB_UCPD1_2_IRQHandler, DMA1_Channel1_IRQHandler, DMA1_Channel2_3_IRQHandler,
    DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler,
    ADC1_COMP_IRQHandler, TIM1_BRK_UP_TRG_COM_IRQHandler, TIM1_CC_IRQHandler, TIM2_IRQHandler,
    TIM3_TIM4_IRQHandler, TIM6_DAC_LPTIM1_IRQHandler, TIM7_LPTIM2_IRQHandler, TIM14_IRQHandler,
    TIM15_IRQHandler, TIM16_FDCAN_IT0_IRQHandler, TIM17_FDCAN_IT1_IRQHandler, I2C1_IRQHandler,
    I2C2_3_IRQHandler, SPI1_IRQHandler, SPI2_3_IRQHandler, USART1_IRQHandler,
    USART2_LPUART2_IRQHandler, USART3_4_5_6_LPUART1_IRQHandler, CEC_IRQHandler, nullptr,
};
