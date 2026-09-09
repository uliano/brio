/*
 * nvic.hpp
 *
 * Interrupt control on the SAM C21: the device header, then the core
 * stratum. InterruptGuard, the enable/disable/readback verbs, the Nvic
 * resource and irq_priority_levels are ARMv6-M and not SAM, so they live
 * in armv6m/nvic.hpp; this header is the SAM's include of it, after
 * "sam.h" has declared the IRQn enumerators and the priority width the
 * core file is written against.
 *
 * It still sits at the BOTTOM of the samc21/ stratum: ticker.hpp and
 * platform.hpp both need the guard, and the platform includes the
 * ticker - so the guard can live in neither of them.
 *
 * The SAM's own facts about interrupts: line 0 is SHARED by MCLK,
 * OSCCTRL, OSC32KCTRL, PAC and SUPC (SYSTEM_Handler reads the pending
 * peripheral itself); every other peripheral has a line of its own.
 */

#pragma once

#include "sam.h"

#include "armv6m/nvic.hpp"
