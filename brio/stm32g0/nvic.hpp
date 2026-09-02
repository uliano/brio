/*
 * nvic.hpp
 *
 * Interrupt control on the STM32G0: the device header, then the core
 * stratum. InterruptGuard, the enable/disable/readback verbs, the Nvic
 * resource and irq_priority_levels are ARMv6-M and not ST, and live in
 * armv6m/nvic.hpp - this family's arrival is what factored that file
 * out of the SAM's; this header is the STM32G0's include of it, after
 * "stm32g0xx.h" has declared the IRQn enumerators and the priority
 * width the core file is written against.
 *
 * It sits at the BOTTOM of the stm32g0/ stratum: ticker.hpp and
 * platform_stm32.hpp both need the guard, and the platform includes the
 * ticker.
 *
 * The STM32G0's own fact about interrupts: SHARED LINES ARE THE RULE.
 * RM0444 table 61 gives 32 positions to more peripherals than that, so
 * USART2 shares a line with LPUART2, USART3..6 with LPUART1, TIM3 with
 * TIM4, and so on. Enabling a line enables it for every peripheral on
 * it; the app's handler for that vector calls each owner's ISR body in
 * turn (stm32g0/device_tables.hpp is where an instance's line is looked
 * up). NOT here (declared, not built): NVIC_SystemReset and the
 * vector-table relocation (VTOR exists on this core, RM0444 12.1) -
 * they belong with the reset/panic pass.
 */

#pragma once

#include "stm32g0xx.h"

#include "armv6m/nvic.hpp"
