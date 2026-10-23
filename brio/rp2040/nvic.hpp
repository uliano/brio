/*
 * nvic.hpp
 *
 * Interrupt control on the RP2040: enabling and disabling a line, the
 * global mask, and InterruptGuard, the scoped critical section the
 * platform and the drivers use. The init() of the UART, the SPI host
 * and client, the I2C host and client and the USB device enables the
 * block's line on the calling core; the DMA's line is enabled by
 * DmaLine::enable() or an engine's arm(), the doorbell's by the inbox's
 * enable(). For every other block - the timer, the ADC and the RTC
 * (whose init() disables the line), the PIO, the PWM - the program
 * calls Nvic::enable itself, on the core that will serve it
 * (`Nvic::enable(Timer::irq(0))` in docs/rp2040/timer.md's use). A
 * program calls `brio::enable_interrupts()` once in main(), after the
 * setup; InterruptGuard may be opened from any context and nests.
 *
 * The code is the device header, then the core stratum. InterruptGuard,
 * the enable/disable/readback verbs, the Nvic resource and
 * irq_priority_levels are the Cortex-M's and not Raspberry Pi's, so they
 * live in cortexm/nvic.hpp; this header is the RP2040's include of it,
 * after RP2040.h has declared the IRQn enumerators and the priority
 * width (two bits, four levels) the core file is written against. It
 * sits at the bottom of the rp2040/ stratum: ticker.hpp and platform.hpp
 * both need the guard, and the platform includes the ticker.
 *
 * This chip's own fact about interrupts: there are two NVICs. Each core
 * has its own, each holds its own enables and priorities, and every one
 * of the 32 lines of table 2.3.2 reaches both (a line enabled on both
 * cores interrupts both, and the second one in finds nothing to serve).
 * Every verb below acts on the NVIC of the core that runs it, and the
 * stratum's rule is that a line is enabled by exactly one core: the one
 * whose kernel hosts the driver that owns the peripheral. PRIMASK is
 * per core too, so a CriticalSection excludes this core's handlers and
 * nothing of the other core - the fact the second kernel's bridge is
 * built around. The no-nesting promise (design/kernel.md section 11,
 * "Platform: what the machine provides": no interrupt handler runs over
 * another) is kept as on the other Cortex-M0+ families: every line at
 * the same priority, the reset value, and no driver opens
 * Nvic::priority().
 *
 * Not here: the software reset (rp2040/reset.hpp, the watchdog's
 * trigger) and the launch of core 1 (the bootrom's protocol over the
 * SIO FIFO, rp2040/multicore.hpp).
 */

#pragma once

#include "rp2040/device.hpp"

#include "cortexm/nvic.hpp"
