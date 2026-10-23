/*
 * nvic.hpp
 *
 * Interrupt control on the STM32F4: the critical section a program or a
 * driver opens around a decision (`brio::InterruptGuard guard;` masks
 * every interrupt until the scope ends, nests, and may be opened from any
 * context), enable_interrupts() once in main() before the kernel runs,
 * and the Nvic resource that enables, pends and reads a peripheral's line
 * by its IRQn. The device header comes first, then the core stratum.
 * InterruptGuard, the enable/disable/readback verbs, the Nvic resource
 * and irq_priority_levels are the Cortex-M's and not ST's, so they live
 * in cortexm/nvic.hpp - the file the three Cortex-M0+ families share and
 * this Cortex-M4 family includes unchanged: SysTick, the NVIC's
 * enable/pend/priority registers and PRIMASK have the same programmer's
 * model on ARMv7-M (PM0214 4.3, 4.4). This header is the STM32F4's
 * include of it, after "stm32f4xx.h" has declared the IRQn enumerators
 * and the priority width (four bits here: sixteen levels) the core file
 * is written against.
 *
 * What ARMv7-M adds and this stratum does not use: BASEPRI, the mask
 * "everything at or below priority N", and with it the possibility of
 * a critical section that lets urgent interrupts through. brio's kernel
 * promise (docs/design/kernel.md, section 11, "Platform: what the machine
 * provides") is that no interrupt nests over another, and this family
 * keeps it the way the SAM C21 does: every NVIC line at the same priority
 * (the reset value, 0), PRIMASK the one mask, `Nvic::priority()` the one
 * door and no driver opens it. The preemptive kernel, Rubato, does not
 * build on BASEPRI either: its scheduler rides PendSV at the lowest
 * priority (cortexm/schedule.hpp, stm32f4/platform.hpp's
 * Stm32f4RubatoPlatform), below every peripheral line, and the critical
 * section stays PRIMASK; a zero-latency tier above the kernel on BASEPRI
 * is a later decision.
 *
 * It sits at the bottom of the stm32f4/ stratum: ticker.hpp and
 * platform.hpp both need the guard, and the platform includes the
 * ticker. Not here: VTOR (the crt's) and the fault configuration
 * (stm32f4/reset.hpp's `Faults`), the software reset (reset.hpp's, beside
 * the flags that say a reset happened).
 */
#pragma once

#include "stm32f4xx.h"

#include "cortexm/nvic.hpp"
