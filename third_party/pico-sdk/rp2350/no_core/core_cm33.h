/*
 * core_cm33.h - THE STUB THAT LETS THE RISC-V BUILD READ AN ARM DEVICE
 * HEADER.
 *
 * RP2350.h is the SVD's own description of the chip, and the chip is
 * the same silicon whichever processor is in the socket: the register
 * blocks as structs, the instance pointers, and the IRQn enumerators -
 * which ARE the Hazard3 external interrupt numbers, since the system
 * IRQ numbering is shared between the two architectures (datasheet
 * 3.8.4.2). So the RISC-V half of this target reads the same device
 * header, and the one thing it must not read is the Cortex-M33 core
 * description RP2350.h includes unconditionally.
 *
 * This file is that include, resolved to nothing. The RISC-V preset
 * (and the RISC-V half of the family check) puts this directory FIRST
 * on the include path, so `#include "core_cm33.h"` lands here; no other
 * build ever sees it, and the Arm build gets ARM's own file from
 * third_party/cmsis-core/.
 *
 * WHAT IT MUST STILL SUPPLY. Two things, both of which the real
 * core_cm33.h happens to bring: the fixed-width integer types every
 * register field is declared with, and the access qualifiers __I / __O
 * / __IO that RP2350.h's own __IM / __OM / __IOM fall back to. They are
 * one include and two keywords each. Nothing else of CMSIS-Core is
 * referenced by the device header's peripheral section: no SCB, no
 * NVIC, no intrinsic. What the RISC-V build needs in their place is
 * brio/rp2350/core_hazard3.hpp.
 */

#ifndef BRIO_RP2350_NO_CORE_CM33_H
#define BRIO_RP2350_NO_CORE_CM33_H

#include <stdint.h>

#ifdef __cplusplus
  #define   __I     volatile
#else
  #define   __I     volatile const
#endif
#define     __O     volatile
#define     __IO    volatile

#endif /* BRIO_RP2350_NO_CORE_CM33_H */
