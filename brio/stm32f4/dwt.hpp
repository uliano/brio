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
 *  - SLEEP IS THE OPEN QUESTION, AND A PROBE ANSWERS IT FOR YOU. A WFI
 *    with SLEEPDEEP clear stops the CPU clock and keeps FCLK, the core's
 *    free-running clock (RM0390 6.2's clock tree: the CPU clock gated in
 *    "sleep or deepsleep", FCLK in "deepsleep" alone), and which of the
 *    two this implementation's counter runs on is stated in no document
 *    of the stratum's vendor list. DBGMCU_CR.DBG_SLEEP feeds HCLK through
 *    that sleep (33.16.1), and OpenOCD's stm32f4x.cfg writes it at every
 *    connection, the register surviving everything but a power-on reset
 *    (stm32f4/pwr.hpp, fact 6). Whether the count runs through a Sleep
 *    with DBG_SLEEP clear is NOT MEASURED on this family; a program that
 *    leans on it states the bit as it found it (Pwr::debug_in_sleep())
 *    and checks the count against a timebase that does run through Sleep
 *    (SysTick, the kernel tick: it is what ends the WFI).
 *  - STOP STOPS IT: every clock of the 1.2 V domain is off (5.3.5), so a
 *    span across a Stop reads the awake cycles alone.
 */

#pragma once

#include "stm32f4xx.h"

#include "cortexm/dwt.hpp"
