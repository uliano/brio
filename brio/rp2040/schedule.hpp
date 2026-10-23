/*
 * schedule.hpp - the RP2040's own half of the preemptive kernel's
 * scheduling request: ONE PendSV ENTRY FOR TWO KERNELS.
 *
 * What it is for. Rubato (kernel/rubato.hpp) asks a Cortex-M for its
 * scheduler through PendSV, and cortexm/schedule.hpp is how every
 * Cortex-M keeps that contract: PendSV pended in the ICSR of the core
 * that asks, taken by that core, returning into scheduler_thread<K>() in
 * thread mode on that core's stack, the scheduler leaving through SVC.
 * All of it is per core on this chip - each Cortex-M0+ has its own ICSR,
 * its own SHPR3 and its own stack - and nothing of it needs changing for
 * two cores. What this chip adds is one vector table for both: core 1 is
 * launched with core 0's VTOR (rp2040/multicore.hpp), so the one
 * `isr_pendsv` entry is taken by whichever core pended it, and
 * cortexm/schedule.hpp's scheduler_exception<K>() is a body written for
 * one kernel. A program with a Rubato on core 0 alone binds that body
 * (rp2040/platform.hpp's Rp2040RubatoPlatform says how: core 1 under
 * Tenuto never pends PendSV). A program with a Rubato on each core binds
 * the body below instead:
 *
 *     using P0 = brio::Rp2040RubatoPlatform<0>;
 *     using P1 = brio::Rp2040RubatoPlatform<1>;
 *     using K0 = brio::Rubato<P0, Sensor, Logger>;      // core 0's pack
 *     using K1 = brio::Rubato<P1, Motor, Watch>;        // core 1's pack
 *     extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception_pair<K0, K1>(); }
 *     extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }
 *     ...
 *     brio::scheduler_init();   // on each core, in its own start-up, before
 *                               // it enables its interrupts
 *
 * THE PAIR BODY. The same naked body as cortexm/schedule.hpp's - the two
 * words, the fake frame, the return through it into thread mode, read
 * that file for the frames - with one question asked first: which core
 * took the exception. SIO's CPUID (datasheet 2.3.1.1: 0 on core 0, 1 on
 * core 1, the first register of the IOPORT space at 0xd0000000) is read
 * in one cycle over the core's single-cycle IO port (2.3.1, table 81's
 * note a), scaled to a word and used to index a table of two return
 * addresses placed after the body: scheduler_thread<K0> or
 * scheduler_thread<K1>. No branch: the two cores take the same path, the
 * same number of cycles. Against the single body, by the release listing
 * (docs/rp2040/platform.md, "Types and verbs"), the selection replaces the
 * one literal load of the entry by six instructions - the SIO address
 * built in two (MOVS, LSLS: no literal, no data fetch from the flash),
 * the CPUID load, its scaling, the table's address (ADR) and the entry's
 * load - seven cycles where the literal took two at zero wait states
 * (table 81). And the body has no DSB, as the single body has none on
 * this core: the barrier is the Cortex-M4's alone, for its erratum
 * 838869 (that file's header), this chip's two cores are Cortex-M0+
 * r0p1, and the stores it would order are the core's own, unstacked by
 * the same core.
 * Eighteen instructions to the single body's thirteen, 44 bytes with the
 * table; two cycles more on each core.
 *
 * Each core keeps everything else as cortexm/schedule.hpp says: PendSV at
 * the lowest level in its own SHPR3 (so scheduler_init() runs on each core),
 * the scheduler on its own stack (core 0's in the bank at 0x2004_1000, core
 * 1's in the bank at 0x2004_0000, rp2040/ld/), its own ceiling
 * (Rp2040RubatoPlatform<core>::ceiling, a static of the core's own
 * platform type). scheduler_return() needs no question: it steps over the
 * SVC frame on the stack it finds, which is the core's own.
 *
 * THE OTHER ROAD, NOT BUILT: a vector table per core. Core 1 launched
 * with a table of its own (the launch protocol's VTOR word is the table
 * core 1 runs with, 2.8.2), identical to core 0's but for entry 14, which
 * would hold a single body bound to K1 - so each core's PendSV is
 * cortexm/schedule.hpp's thirteen instructions with nothing asked. What
 * it costs: a second table of 48 words in the image (192 bytes in the
 * flash, or a copy in SRAM, aligned as VTOR requires), every vector the
 * program binds present in both; the crt to define it
 * (rp2040/src/glue/startup_rp2040.cpp holds the one table today) and the
 * launch to take it (Core1::launch passes this core's SCB->VTOR); and a
 * second spelling of PendSV's binding that a program must keep apart
 * from the first. What it saves: the two cycles above, and the question
 * asked at every request. Which road a two-kernel program takes is an
 * open decision (docs/rp2040/platform.md, "Not covered yet").
 *
 * What this file assumes, beyond cortexm/schedule.hpp's own list: K0 is
 * a Rubato on core 0's platform type and K1 one on core 1's (checked at
 * compile time below), and the program binds `isr_pendsv` to this body
 * and to nothing else.
 *
 * Include-order contract: rp2040/platform.hpp first is enough (it brings
 * the device header, cortexm/schedule.hpp and the platform types).
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "kernel/rubato.hpp"
#include "rp2040/platform.hpp"
#include "cortexm/schedule.hpp"

namespace brio {

namespace rp2040_detail {

/// The core a Rubato kernel runs on: its platform's `core`. Undefined for
/// anything but a Rubato, so the pair refuses a stand-in.
template <class K>
struct rubato_core;

template <Platform P, ActiveObject... Aos>
struct rubato_core<Rubato<P, Aos...>> {
    static constexpr uint8_t value = P::core;
};

} // namespace rp2040_detail

/// The body the program binds to `isr_pendsv` when each core runs a
/// Rubato, a naked handler: the two words and the fake frame, then the
/// return into scheduler_thread<K0>() on core 0 and scheduler_thread<K1>()
/// on core 1 (the file header). Eighteen instructions, the same on both
/// cores.
template <class K0, class K1>
[[gnu::always_inline]] inline void scheduler_exception_pair() {
    static_assert(rp2040_detail::rubato_core<K0>::value == 0u,
                  "scheduler_exception_pair<K0, K1>: K0 is the kernel of core 0 - a Rubato on "
                  "Rp2040RubatoPlatform<0>");
    static_assert(rp2040_detail::rubato_core<K1>::value == 1u,
                  "scheduler_exception_pair<K0, K1>: K1 is the kernel of core 1 - a Rubato on "
                  "Rp2040RubatoPlatform<1>");
    // The SIO's address is built by MOVS and LSLS below: that is this
    // value, the device header's.
    static_assert(SIO_BASE == 0xD000'0000UL, "the SIO block at 0xd0000000 (datasheet 2.3.1)");
    asm volatile(
        ".syntax unified\n\t"
        "cpsid i\n\t"
        "push {r0, lr}\n\t"         // a filler, then R's EXC_RETURN
        "sub sp, sp, #32\n\t"       // the fake frame
        "movs r1, #0xd0\n\t"
        "lsls r1, r1, #24\n\t"      // SIO_BASE
        "ldr r1, [r1]\n\t"          // CPUID: 0 or 1, one cycle on the IOPORT
        "lsls r1, r1, #2\n\t"       // a word's offset in the table
        "adr r0, 1f\n\t"
        "ldr r0, [r0, r1]\n\t"      // this core's entry, its Thumb bit set
        "subs r0, r0, #1\n\t"       // the stacked return address is halfword-aligned
        "str r0, [sp, #24]\n\t"     // F.PC
        "movs r0, #1\n\t"
        "lsls r0, r0, #24\n\t"
        "str r0, [sp, #28]\n\t"     // F.xPSR: the Thumb bit alone
        "mov r0, lr\n\t"
        "movs r1, #16\n\t"
        "orrs r0, r0, r1\n\t"       // R's EXC_RETURN, a basic frame
        "bx r0\n\t"
        ".balign 4\n"
        "1:\n\t"
        ".word %c0\n\t"             // CPUID 0
        ".word %c1\n\t"             // CPUID 1
        :: "i"(&scheduler_thread<K0>), "i"(&scheduler_thread<K1>));
}

} // namespace brio
