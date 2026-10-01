/*
 * dwt.hpp
 *
 * The cycle counter of this family's Cortex-M4: the device header, then
 * the core stratum's CycleCounter (cortexm/dwt.hpp), whose contract and
 * reasoning live there. What is the STM32F4's here is what holds on it:
 *
 *  - THE UNIT IS THERE. RM0390 33.13 (and its RM0090 / RM0383 / RM0386
 *    twins) gives this family's DWT its four comparators and the clock
 *    cycle counter, so init() answering false would be a finding, not a
 *    variant.
 *  - IT COUNTS HCLK, 5.6 ns a count at 180 MHz and 10 ns at the F411's
 *    100; a difference of two reads stays exact for about 24 s at the
 *    family's ceiling.
 *  - IT COUNTS THROUGH A SLEEP - MEASURED. A WFI with SLEEPDEEP clear
 *    stops the CPU clock and keeps FCLK, the core's free-running clock
 *    (RM0390 6.2's clock tree: the CPU clock gated in "sleep or
 *    deepsleep", FCLK in "deepsleep" alone), and no document of the
 *    stratum's vendor list says which of the two this implementation's
 *    counter runs on. DBGMCU_CR.DBG_SLEEP feeds HCLK through that sleep
 *    (33.16.1), and OpenOCD's stm32f4x.cfg writes it at every connection,
 *    the register surviving everything but a power-on reset
 *    (stm32f4/pwr.hpp, fact 6) - so the bit was CLEARED at run time on
 *    the STM32F446RE and a hundred SysTick periods spent in WFI read
 *    17999978 cycles against the 18000000 the tick says: the counter runs
 *    through a Sleep whatever DBG_SLEEP holds (design/benchmark.md, the
 *    idle-window control line). A span across a Sleep is a wall span.
 *  - STOP STOPS IT: every clock of the 1.2 V domain is off (5.3.5), so a
 *    span across a Stop reads the awake cycles alone.
 */

#pragma once

#include "stm32f4xx.h"

#include "cortexm/dwt.hpp"
