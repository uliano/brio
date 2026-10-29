/*
 * schedule.hpp
 *
 * The scheduling request of the preemptive kernel on this target, and
 * the place its scheduler runs - on the Cortex-M33 half and on the
 * Hazard3 half, under ONE SURFACE, for a Rubato on core 0 alone or for a
 * Rubato on each core.
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
 * `request_scheduler_on<core>()`, `scheduler_init()`,
 * `scheduler_thread<K>()` - and so is the binding a program writes, one
 * macro that defines the vectors its half needs. One kernel, on core 0:
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
 * same lines the macro expands to. The macro refuses, at compile time, a
 * kernel that is not core 0's: the single body serves one kernel, and
 * core 0 is the core whose main() starts it.
 *
 * TWO KERNELS, ONE TABLE. Both crts give the two cores of a pair one
 * table: core 1 is launched with core 0's vector base (VTOR on one half,
 * mtvec on the other, rp2350/multicore.hpp), so the scheduler's vectors
 * are one entry each for both cores, and the entry taken on core 1 must
 * serve core 1's kernel. A program with a Rubato on each core binds the
 * PAIR body, which asks the core at the entry which kernel to serve:
 *
 *     using P0 = brio::Rp2350RubatoPlatform<0>;
 *     using P1 = brio::Rp2350RubatoPlatform<1>;
 *     using K0 = brio::Rubato<P0, Sensor, Logger>;   // core 0's pack
 *     using K1 = brio::Rubato<P1, Motor, Watch>;     // core 1's pack
 *     BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K0, K1)
 *     ...
 *     brio::scheduler_init();   // on each core, in its own start-up,
 *                               // before it enables its interrupts
 *
 * The pair bodies are the single ones with the question in place of the
 * single entry's address, and nothing else changed: on the Cortex-M33
 * SIO's CPUID (datasheet 3.1.2) indexing a table of two entries after
 * the body, on Hazard3 the hart's mhartid (the same answer, 3.1.2's
 * note) indexing the same kind of table; no branch, so both cores take
 * one path of one length. Each body says what it costs over the single
 * one. Everything else is per core already: PendSV's priority is a
 * register of each M33 (so scheduler_init() runs on each core), mie,
 * mscratch and the software interrupt flag are each hart's own, each
 * core's scheduler runs on its own stack (core 0's in scratch_y, core
 * 1's in scratch_x, rp2350/ld/), and each kernel's ceiling is a static of
 * its own platform type. The SVC body asks nothing: it steps over the
 * frame on the stack it finds, which is the core's own. A vector table
 * per core - core 1 launched with a copy whose scheduler entries name K1
 * - would spare the question at every request for a second table that
 * every bound vector must be kept in step with; it is declined.
 *
 * THE REQUEST IS MADE ONLY FROM THE PLATFORM'S OWN CORE.
 * Rp2350RubatoPlatform<core>::pend_schedule() is request_scheduler_on<
 * core>(), which asks only when the calling core is `core`. post() asks
 * after the queue's push whenever the receiver's index is below the
 * receiver's platform's ceiling (kernel/post.hpp), and a post from the
 * other core is a mispost that push() has already refused and counted
 * (kernel/event_queue.hpp). Asking then would raise the POSTING core's
 * request - PendSV in its own ICSR, its own hart's flag - and with a
 * Rubato on core 0 alone that core is core 1 under Tenuto: on the
 * Cortex-M33 the single body taken there would run core 0's kernel on
 * core 1, and on Hazard3 core 1, whose software interrupt is not enabled,
 * would spin for ever in the request's wait. Asked only on its own core,
 * a mispost stays what the queue made it: a count. With the pair bound
 * the unchecked request would merely run the posting core's own
 * scheduler once for nothing; the check is the same in both programs.
 * request_scheduler() itself is the unchecked request of the calling
 * core, what a bench letter that times the bare request calls.
 *
 * A PREPROCESSOR QUESTION ABOVE core.hpp IS ALWAYS ABOUT
 * BRIO_RP2350_CORE_M33, the macro core.hpp defines, and never about the
 * compiler's `__riscv` (rp2350/ticker.hpp says the same of itself). This
 * file asks it because an #include cannot be selected by a constant: the
 * Cortex-M33's realization is a file of the core stratum, the Hazard3's
 * a file of this one.
 */

#pragma once

#include <stdint.h>

#include "rp2350/core.hpp"

#include "kernel/rubato.hpp"

namespace brio {

namespace rp2350_detail {

/// The core a Rubato kernel runs on: its platform's `core`. Undefined for
/// anything but a Rubato, so a binding refuses a stand-in.
template <class K>
struct rubato_core;

template <Platform P, ActiveObject... Aos>
struct rubato_core<Rubato<P, Aos...>> {
    static constexpr uint8_t value = P::core;
};

/// The pair's two kernels in their order, checked once for both halves.
template <class K0, class K1>
consteval bool kernel_pair_in_order() {
    static_assert(rubato_core<K0>::value == 0u,
                  "the scheduler's pair binding <K0, K1>: K0 is the kernel of core 0 - a Rubato "
                  "on Rp2350RubatoPlatform<0>");
    static_assert(rubato_core<K1>::value == 1u,
                  "the scheduler's pair binding <K0, K1>: K1 is the kernel of core 1 - a Rubato "
                  "on Rp2350RubatoPlatform<1>");
    return true;
}

} // namespace rp2350_detail

} // namespace brio

#if defined(BRIO_RP2350_CORE_M33)

#include "cortexm/schedule.hpp"

namespace brio {

/// The request for core `core`'s scheduler, made only when the calling
/// core is `core` (the file header): SIO's CPUID compared, then
/// PENDSVSET with the DSB and the ISB, in one call the compiler cannot
/// see into - cortexm/schedule.hpp's request_scheduler() with the test in
/// front, inside the same opaque call, so that the asking path pays one
/// call and not a second one around the test. Any context; never waits.
template <uint8_t core>
[[gnu::noinline, gnu::noipa]] void request_scheduler_on() {
    if (SIO->CPUID == core) {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;   // every other ICSR bit written 0: no effect
        __DSB();
        __ISB();
    }
}

/// The body the program binds to `isr_pendsv` when each core runs a
/// Rubato, a naked handler: cortexm/schedule.hpp's scheduler_exception<K>()
/// - the two words, the fake frame, the return through it into thread
/// mode, the FP part of EXC_RETURN as that body builds it - with the
/// entry's address taken from a table of two after the body, indexed by
/// SIO's CPUID: scheduler_thread<K0>() on core 0, scheduler_thread<K1>()
/// on core 1.
///
/// Written for THIS core. The M33 is ARMv8-M main, so the question is
/// Thumb-2: the SIO's address is one MOV.W of a modified immediate (no
/// literal, no data fetch), the CPUID one load over the core's AHB port
/// to the SIO, zero wait states (2.2.6; this core has no single-cycle IO
/// port, where the RP2040's Cortex-M0+ had one), the table's address one
/// ADR and the entry one LDR with the index scaled in the instruction.
/// Four instructions where the single body has the entry's literal load:
/// sixteen instructions to its thirteen, 44 bytes with the table to its
/// 32 with the literal; by the Cortex-M4's table the suite counts with
/// (test_rp2350_rubato's header: a load 2, an ALU instruction 1) six
/// cycles where the literal took two, four more on each core, the same
/// path on both. The rest is the single body's instructions one for one,
/// the three that build EXC_RETURN included, and the body carries no DSB:
/// the barrier is the Cortex-M4's alone (that file's header).
///
/// The other ways to ask, declined: this core's own CPUID register in
/// the System Control Space reads the same part number on both cores
/// (3.1.2's warning); the stack pointer's bank tells the cores apart only
/// while each stack stays where the linker script puts it, and
/// Core1::launch() takes any stack top.
template <class K0, class K1>
[[gnu::always_inline]] inline void scheduler_exception_pair() {
    static_assert(rp2350_detail::kernel_pair_in_order<K0, K1>());
    // The SIO's address is the MOV.W's immediate, and CPUID its word 0:
    // these values, the device header's.
    static_assert(SIO_BASE == 0xD000'0000UL, "the SIO block at 0xd0000000 (datasheet 2.2)");
    static_assert(SIO_CPUID_OFFSET == 0u, "CPUID is the SIO's first register (3.1.2)");
    asm volatile(
        ".syntax unified\n\t"
        "cpsid i\n\t"
        "push {r0, lr}\n\t"           // a filler, then R's EXC_RETURN
        "sub sp, sp, #32\n\t"         // the fake frame
        "mov.w r1, #0xd0000000\n\t"   // SIO_BASE
        "ldr r1, [r1]\n\t"            // CPUID: 0 or 1
        "adr r0, 1f\n\t"
        "ldr.w r0, [r0, r1, lsl #2]\n\t"   // this core's entry, its Thumb bit set
        "subs r0, r0, #1\n\t"         // the stacked return address is halfword-aligned
        "str r0, [sp, #24]\n\t"       // F.PC
        "movs r0, #1\n\t"
        "lsls r0, r0, #24\n\t"
        "str r0, [sp, #28]\n\t"       // F.xPSR: the Thumb bit alone
        "mov r0, lr\n\t"
        "movs r1, #16\n\t"
        "orrs r0, r0, r1\n\t"         // R's EXC_RETURN, a basic frame
        "bx r0\n\t"
        ".balign 4\n"
        "1:\n\t"
        ".word %c0\n\t"               // CPUID 0
        ".word %c1\n\t"               // CPUID 1
        :: "i"(&scheduler_thread<K0>), "i"(&scheduler_thread<K1>));
}

} // namespace brio

/// The binding of the scheduler's vectors on the Cortex-M33 half, one
/// kernel on core 0: PendSV and SVCall, both naked (cortexm/schedule.hpp).
#define BRIO_RP2350_SCHEDULER_VECTORS(Kernel)                                              \
    static_assert(brio::rp2350_detail::rubato_core<Kernel>::value == 0u,                   \
                  "BRIO_RP2350_SCHEDULER_VECTORS(K): K is core 0's kernel - a Rubato on "  \
                  "core 1 needs BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K0, K1)");              \
    extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception<Kernel>(); }  \
    extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

/// The same for a Rubato on each core: PendSV to the pair body, SVCall
/// as for one kernel.
#define BRIO_RP2350_SCHEDULER_VECTORS_PAIR(Kernel0, Kernel1)                                     \
    extern "C" [[gnu::naked]] void isr_pendsv() {                                               \
        brio::scheduler_exception_pair<Kernel0, Kernel1>();                                     \
    }                                                                                           \
    extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

#else

#include "rp2350/schedule_hazard3.hpp"

/// The binding of the scheduler's vector on the Hazard3 half, one kernel
/// on core 0: the machine software interrupt, naked, called by the crt's
/// trampoline (rp2350/schedule_hazard3.hpp).
#define BRIO_RP2350_SCHEDULER_VECTORS(Kernel)                                              \
    static_assert(brio::rp2350_detail::rubato_core<Kernel>::value == 0u,                   \
                  "BRIO_RP2350_SCHEDULER_VECTORS(K): K is core 0's kernel - a Rubato on "  \
                  "core 1 needs BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K0, K1)");              \
    extern "C" [[gnu::naked]] void isr_riscv_softirq() { brio::scheduler_trap<Kernel>(); }

/// The same for a Rubato on each core: the software interrupt to the pair
/// body.
#define BRIO_RP2350_SCHEDULER_VECTORS_PAIR(Kernel0, Kernel1) \
    extern "C" [[gnu::naked]] void isr_riscv_softirq() {   \
        brio::scheduler_trap_pair<Kernel0, Kernel1>();      \
    }

#endif
