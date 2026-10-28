/*
 * pfic.hpp
 *
 * Interrupt control on the CH32V00x: the global mask, the critical
 * section the kernel masks with, the per-line enables of the interrupt
 * controller and the macro every interrupt handler of this target is
 * bound with. They are the QingKe core stratum's (qingke/pfic.hpp, whose
 * account they are), over this family's device header; a program and a
 * driver include this file, never that one. The smallest use is a vector
 * binding and its line's enable:
 *
 *     BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     brio::Pfic::enable(brio::Irq::exti7_0);       // a line a driver does not open itself
 *     brio::enable_interrupts();                     // in main, before Tenuto's run()
 *
 * This family's facts: the hardware prologue is the project's
 * CH32V00X_HPE option (ch32v00x/CMakeLists.txt, on by default), with
 * which the crt (ch32v00x/src/glue/startup_ch32v00x.S) sets
 * INTSYSCR.HWSTKEN and the QingKe V2 pushes the ten caller-saved
 * registers to the user stack (QingKe V2 manual 3.4); the core has no
 * floating-point unit, so BRIO_CH32_LEAF_VECTOR and
 * BRIO_CH32_VECTOR_FLOAT are BRIO_CH32_VECTOR exactly; the vector
 * placed in SRAM is this family's option on both parts
 * (docs/ch32v00x/platform.md, "A handler in SRAM").
 */

#pragma once

#include "ch32v00x/device.hpp"
#include "qingke/pfic.hpp"
