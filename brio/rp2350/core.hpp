/*
 * core.hpp
 *
 * THE ONE FILE OF THIS STRATUM THAT ASKS WHICH PROCESSOR SITS IN THE
 * SOCKET.
 *
 * The RP2350 carries four cores over one set of peripherals: two Arm
 * Cortex-M33 and two RISC-V Hazard3 (datasheet 3.9). Which pair runs is
 * decided by the IMAGE_DEF in the image the bootrom finds, so it is a
 * property of the BUILD - the architecture is a preset axis of the
 * rp2350/ project - and the compiler's own `__riscv` is what tells the
 * two builds apart. This file asks that question once and includes one
 * of the two halves; both export THE SAME NAMES, so every other header
 * of the stratum, every driver and every app includes core.hpp and
 * never knows which answered:
 *
 *   InterruptGuard          the kernel's CriticalSection (PRIMASK / mstatus.MIE)
 *   enable_interrupts()     the global unmask an app calls once
 *   disable_interrupts()    its counterpart
 *   interrupts_enabled()    the readback
 *   Irq                     the per-line interrupt controller, taking the
 *                           device header's IRQn_Type: enable, disable,
 *                           enabled, set_pending, clear_pending, pending
 *   irq_priority_levels     sixteen on both, and used by neither
 *   wait_for_interrupt()    the idle path's sleep
 *   debug_break()           a breakpoint instruction
 *   core_id()               which core is running this
 *
 * The interrupt NUMBERS are shared between the architectures (3.8.4.2),
 * which is what makes one `Irq` type over `IRQn_Type` honest, and what
 * lets an app bind one vector name and be bound on both (the two crts in
 * rp2350/src/glue/).
 *
 * THE MACRO THIS FILE EXPORTS. One thing cannot be chosen by a constant:
 * an #include. The Arm half's timebase is SysTick (an ARM peripheral),
 * the RISC-V half's is the platform timer in the SIO, and the two live
 * in different files - so this file also defines BRIO_RP2350_CORE_M33 or
 * BRIO_RP2350_CORE_HAZARD3, and rp2350/ticker.hpp is the one place above
 * here that asks the preprocessor anything. It asks about THIS macro, a
 * fact of this stratum, and never about the compiler's `__riscv`.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#if defined(__riscv)
#define BRIO_RP2350_CORE_HAZARD3 1
#include "rp2350/core_hazard3.hpp"
#else
#define BRIO_RP2350_CORE_M33 1
#include "rp2350/core_m33.hpp"
#endif

namespace brio {

/// Which of the two processor designs this image is built for. A VALUE,
/// so that a driver with something genuinely different to do can branch
/// on it with `if constexpr` instead of asking the preprocessor.
enum class CoreKind : uint8_t { cortex_m33, hazard3 };

#if defined(BRIO_RP2350_CORE_HAZARD3)
inline constexpr CoreKind core_kind = CoreKind::hazard3;
#else
inline constexpr CoreKind core_kind = CoreKind::cortex_m33;
#endif

/// Which core is running this, 0 or 1: SIO's CPUID, which is the same
/// register and the same answer whichever architecture reads it (3.1.2).
/// The RISC-V hart id would answer as well; one spelling is better than
/// two.
inline uint8_t core_id() { return static_cast<uint8_t>(SIO->CPUID); }

} // namespace brio
