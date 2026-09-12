/*
 * nvic.hpp
 *
 * Interrupt control on the RP2040: the device header, then the core
 * stratum. InterruptGuard, the enable/disable/readback verbs, the Nvic
 * resource and irq_priority_levels are ARMv6-M and not Raspberry Pi's,
 * so they live in armv6m/nvic.hpp; this header is the RP2040's include
 * of it, after RP2040.h has declared the IRQn enumerators and the
 * priority width (two bits, four levels) the core file is written
 * against. It sits at the BOTTOM of the rp2040/ stratum: ticker.hpp
 * and platform.hpp both need the guard, and the platform includes the
 * ticker.
 *
 * THIS CHIP'S OWN FACT ABOUT INTERRUPTS: THERE ARE TWO NVICs. Each core
 * has its own, each holds its own enables and priorities, and every one
 * of the 32 lines of table 2.3.2 reaches both (a line enabled on both
 * cores interrupts both, and the second one in finds nothing to serve).
 * Every verb below acts on the NVIC of the core that runs it, and the
 * stratum's rule is that a line is enabled by exactly one core: the one
 * whose kernel hosts the driver that owns the peripheral. PRIMASK is
 * per core too, so a CriticalSection excludes THIS core's handlers and
 * nothing of the other core - the fact the second kernel's bridge is
 * built around. The no-nesting promise (design/kernel.md section 11) is
 * kept as on the other Cortex-M0+ families: every line at the same
 * priority, the reset value, and no driver opens Nvic::priority().
 *
 * NOT here: the software reset (the watchdog's or the resets block's
 * business, a later reset.hpp) and the launch of core 1 (the bootrom's
 * protocol over the SIO FIFO, the second kernel's business).
 */

#pragma once

#include "rp2040/device.hpp"

#include "armv6m/nvic.hpp"
