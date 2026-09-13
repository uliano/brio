/*
 * nvic.hpp
 *
 * Interrupt control on the STM32F4: the device header, then the core
 * stratum. InterruptGuard, the enable/disable/readback verbs, the Nvic
 * resource and irq_priority_levels are the Cortex-M's and not ST's, so
 * they live in armv6m/nvic.hpp - the file the three Cortex-M0+ families
 * share, and which this Cortex-M4 family includes unchanged: SysTick,
 * the NVIC's enable/pend/priority registers and PRIMASK have the same
 * programmer's model on ARMv7-M (PM0214 4.3, 4.4). This header is the
 * STM32F4's include of it, after "stm32f4xx.h" has declared the IRQn
 * enumerators and the priority width (four bits here: sixteen levels)
 * the core file is written against.
 *
 * WHAT ARMv7-M ADDS AND THIS STRATUM DOES NOT USE: BASEPRI, the mask
 * "everything at or below priority N", and with it the possibility of
 * a critical section that lets urgent interrupts through. brio's kernel
 * promise (docs/design/kernel.md, section 11) is that NO INTERRUPT
 * NESTS OVER ANOTHER, and this family keeps it the way the SAM C21
 * does: every NVIC line at the same priority (the reset value, 0),
 * PRIMASK the one mask, `Nvic::priority()` the one door and no driver
 * opens it. BASEPRI is what a preemptive kernel would build on; that
 * kernel is another type and another day.
 *
 * It sits at the BOTTOM of the stm32f4/ stratum: ticker.hpp and
 * platform.hpp both need the guard, and the platform includes the
 * ticker. Not here: VTOR and the fault configuration (the crt's and the
 * reset/panic pass's), the software reset (reset.hpp's, beside the
 * flags that say a reset happened).
 */
#pragma once

#include "stm32f4xx.h"

#include "armv6m/nvic.hpp"
