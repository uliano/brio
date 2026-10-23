/*
 * schedule.hpp
 *
 * The scheduling request of the preemptive kernel on this target, and
 * the place its scheduler runs - on the Cortex-M33 half and on the
 * Hazard3 half, under ONE SURFACE.
 *
 * What a Rubato platform gives the kernel (kernel/rubato.hpp) beyond what
 * Tenuto takes from it: a way to ask for the scheduler, and the place the
 * scheduler then runs. kernel/platform.hpp states the contract - the
 * request, pend_schedule(), runs the scheduler at once when it is made
 * from a dispatch and at the handler's exit when it is made from an
 * interrupt handler. This file is how the RP2350 keeps it, on whichever
 * processor pair the image was built for:
 *
 *  - on the Cortex-M33 it is the core stratum's PendSV and SVC, exactly
 *    as on the STM32F4 and the STM32G0 (cortexm/schedule.hpp, whose
 *    header holds the frames, the FPU's part and EXC_RETURN's bits);
 *  - on Hazard3 it is the machine software interrupt through the crt's
 *    own trampoline, and the scheduler's thread-mode entry returning by a
 *    `mret` of its own (rp2350/schedule_hazard3.hpp says how and why).
 *
 * The NAMES are the same on both halves - `request_scheduler()`,
 * `scheduler_init()`, `scheduler_thread<K>()` - and so is the binding a
 * program writes, one macro that defines the vectors its half needs:
 *
 *     using P = brio::Rp2350RubatoPlatform<>;
 *     using K = brio::Rubato<P, Sensor, Logger>;
 *     BRIO_RP2350_SCHEDULER_VECTORS(K)
 *     ...
 *     brio::scheduler_init();        // in main(), before interrupts are enabled
 *     K::run();
 *
 * which is `isr_pendsv` and `isr_svcall` on the Cortex-M33 (the crt's
 * names for PendSV and SVCall, startup_rp2350_arm.cpp) and
 * `isr_riscv_softirq` on Hazard3 (the crt's machine software interrupt,
 * startup_rp2350_riscv.S), each naked and each a body of this file or of
 * the core stratum's. A program that binds the vectors itself writes the
 * same three lines the macro expands to.
 *
 * ONE KERNEL, CORE 0. Both crts have ONE table for both cores of their
 * pair, so a second Rubato on core 1 would enter the same vectors with a
 * second kernel to serve: the two-kernel body (which kernel a vector
 * serves, asked of the core at the vector) is the RP2040's question too,
 * and Rp2350RubatoPlatform refuses core 1 until it is answered
 * (rp2350/platform.hpp).
 *
 * A PREPROCESSOR QUESTION ABOVE core.hpp IS ALWAYS ABOUT
 * BRIO_RP2350_CORE_M33, the macro core.hpp defines, and never about the
 * compiler's `__riscv` (rp2350/ticker.hpp says the same of itself). This
 * file asks it because an #include cannot be selected by a constant: the
 * Cortex-M33's realization is a file of the core stratum, the Hazard3's
 * a file of this one.
 */

#pragma once

#include "rp2350/core.hpp"

#if defined(BRIO_RP2350_CORE_M33)

#include "cortexm/schedule.hpp"

/// The binding of the scheduler's vectors on the Cortex-M33 half: PendSV
/// and SVCall, both naked (cortexm/schedule.hpp).
#define BRIO_RP2350_SCHEDULER_VECTORS(Kernel)                                         \
    extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception<Kernel>(); } \
    extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

#else

#include "rp2350/schedule_hazard3.hpp"

/// The binding of the scheduler's vector on the Hazard3 half: the machine
/// software interrupt, naked, called by the crt's trampoline
/// (rp2350/schedule_hazard3.hpp).
#define BRIO_RP2350_SCHEDULER_VECTORS(Kernel) \
    extern "C" [[gnu::naked]] void isr_riscv_softirq() { brio::scheduler_trap<Kernel>(); }

#endif
