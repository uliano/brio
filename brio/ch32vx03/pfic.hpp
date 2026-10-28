/*
 * pfic.hpp
 *
 * Interrupt control on the CH32V203 and the CH32V303, for a program and
 * for the drivers: the global interrupt enable, the critical section the
 * kernel masks with, the per-line enables of the interrupt controller
 * (the PFIC, WCH's own), and the three macros that bind a vector - one
 * spelling a program writes whatever the build, the macro choosing the
 * handler's shape from the build. They are the QingKe core stratum's
 * (qingke/pfic.hpp, whose account they are: the trampoline the CH32V303's
 * images make of a vector, the float and leaf forms, the vector guard,
 * `no_icf`, the alignment), over this family's device header; a program
 * and a driver include this file, never that one. A program meets the
 * macros and the global enable as docs/ch32vx03/platform.md's "How to use
 * it" writes them:
 *
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *     BRIO_CH32_VECTOR(fault_handler) { brio::fault_reset<P>(); }
 *     ...
 *     brio::enable_interrupts();
 *
 * `P` is the program's platform type (`using P =
 * brio::Ch32vx03Platform<>;`, platform.hpp). This family's facts: the
 * hardware prologue is the project's CH32VX03_HPE option
 * (ch32vx03/CMakeLists.txt, on by default), with which the crt
 * (ch32vx03/src/glue/startup_ch32vx03.S) sets INTSYSCR.HWSTKEN; the
 * CH32V303's images are built with F (the project's part table,
 * ch32vx03/cmake/ch32vx03-parts.cmake), so there every vector bound with
 * BRIO_CH32_VECTOR is a naked trampoline and `brio check ch32vx03` holds
 * every such body to touching no f-register; on every CH32V203 image the
 * three macros are one.
 */

#pragma once

#include "ch32vx03/device.hpp"
#include "qingke/pfic.hpp"
