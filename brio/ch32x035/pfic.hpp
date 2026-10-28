/*
 * pfic.hpp
 *
 * Interrupt control on the CH32X035, for a program and for the drivers:
 * the global interrupt enable, the critical section the kernel masks
 * with, the per-line enables of the interrupt controller (the PFIC,
 * WCH's own), and the macro every interrupt handler of this target is
 * bound with. They are the QingKe core stratum's (qingke/pfic.hpp, whose
 * account they are), over this family's device header; a program and a
 * driver include this file, never that one. A program meets all four:
 *
 *     BRIO_CH32_VECTOR(usart2_handler) { (void)Serial::isr(); }
 *     brio::Pfic::enable(brio::Irq::usart2);     // a driver's init() does this itself
 *     brio::enable_interrupts();                  // once, before the kernel runs
 *     { brio::InterruptGuard guard; ... }         // a critical section, any context
 *
 * This family's facts: the hardware prologue is the project's
 * CH32X035_HPE option (ch32x035/CMakeLists.txt, on by default), with
 * which the crt (ch32x035/src/glue/startup_ch32x035.S) sets
 * INTSYSCR.HWSTKEN and the QingKe V4C saves the sixteen caller-saved
 * integer registers itself, two levels deep; the core has no
 * floating-point unit, so BRIO_CH32_LEAF_VECTOR and
 * BRIO_CH32_VECTOR_FLOAT are BRIO_CH32_VECTOR exactly; the code runs
 * from a flash whose fetch makes a function's cost depend on its address
 * modulo four, which is where the stratum's alignment of every vector
 * was first measured (docs/ch32x035/platform.md, "A handler in SRAM").
 */

#pragma once

#include "ch32x035/device.hpp"
#include "qingke/pfic.hpp"
