/*
 * nvic.hpp
 *
 * Interrupt control on the SAM C21: what a program uses to turn
 * interrupts on (`brio::enable_interrupts()`, once in main() after the
 * drivers' init() calls and before the kernel's run()), to mask them
 * for a scope (`InterruptGuard`, the platform's CriticalSection, legal
 * from any context and nesting), and to enable, pend or set the
 * priority of one NVIC line (`Nvic`, which refuses a level this core
 * does not have). The header is the device header, then the core
 * stratum. InterruptGuard, the enable/disable/readback verbs, the Nvic
 * resource and irq_priority_levels are the Cortex-M's and not SAM, so they live
 * in cortexm/nvic.hpp; this header is the SAM's include of it, after
 * "sam.h" has declared the IRQn enumerators and the priority width the
 * core file is written against.
 *
 * It still sits at the bottom of the samc21/ stratum: ticker.hpp and
 * platform.hpp both need the guard, and the platform includes the
 * ticker - so the guard can live in neither of them.
 *
 * The SAM's own facts about interrupts: line 0 is shared by MCLK,
 * OSCCTRL, OSC32KCTRL, PAC and SUPC (SYSTEM_Handler reads the pending
 * peripheral itself); every other peripheral has a line of its own.
 */

#pragma once

#include "sam.h"

#include "cortexm/nvic.hpp"
