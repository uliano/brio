/*
 * nvic.hpp
 *
 * Interrupt control on the STM32G0, what a program and a driver use to
 * mask interrupts and to switch a peripheral's interrupt line:
 * `InterruptGuard`, a scope that masks every interrupt on entry and
 * restores what it found on exit (the kernel's critical section; it
 * nests, and may be opened from any context, an interrupt handler
 * included); `enable_interrupts()`, the one unmask at the end of main()'s
 * setup, with `disable_interrupts()` and `interrupts_enabled()` beside
 * it; `Nvic::enable(irq)` / `disable(irq)` / `priority(irq, level)` and
 * the pending verbs, setup verbs over the device header's IRQn values;
 * and `irq_priority_levels`, the four levels this core has. All of them
 * are the Cortex-M's and not ST, so they live in cortexm/nvic.hpp; this
 * header is the STM32G0's include of it, after "stm32g0xx.h" has
 * declared the IRQn enumerators and the priority width the core file is
 * written against. Masking is all-or-nothing on this core (PRIMASK;
 * ARMv6-M has no BASEPRI), so the NVIC priorities order preemption
 * between handlers, never the reach of a critical section.
 *
 * It sits at the bottom of the stm32g0/ stratum: ticker.hpp and
 * platform.hpp both need the guard, and the platform includes the
 * ticker.
 *
 * The STM32G0's own fact about interrupts: shared lines are the rule.
 * RM0444 table 61 gives 32 positions to more peripherals than that, so
 * USART2 shares a line with LPUART2, USART3..6 with LPUART1, TIM3 with
 * TIM4, and so on. Enabling a line enables it for every peripheral on
 * it; the app's handler for that vector calls each owner's ISR body in
 * turn (stm32g0/device_tables.hpp is where an instance's line is looked
 * up). Not here: the vector-table relocation VTOR would allow (VTOR
 * exists on this core, RM0444 12.1). The software reset is not here
 * either - it is stm32g0/reset.hpp's `Reset::software()`, beside the
 * flags that say a reset happened.
 */

#pragma once

#include "stm32g0xx.h"

#include "cortexm/nvic.hpp"
