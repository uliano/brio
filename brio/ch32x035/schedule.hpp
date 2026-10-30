/*
 * schedule.hpp
 *
 * The scheduling request of the preemptive kernel on the QingKe V4C of
 * the CH32X035, and the thread-mode scheduler it starts:
 * `request_scheduler()`, `scheduler_init()`, the two vector bodies
 * `scheduler_trap<K>()` and `scheduler_return<K>()` and their binding
 * BRIO_CH32_SCHEDULER_VECTOR are the QingKe core stratum's
 * (qingke/schedule.hpp, whose account they are: the software interrupt
 * one preemption level below every peripheral line, the return into the
 * scheduler in thread mode, the way out through the machine-mode
 * environment call). A program binds the two vectors as
 * Ch32x035RubatoPlatform's comment shows (ch32x035/platform.hpp):
 *
 *     BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
 *     BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }
 *
 * WHERE THE PATH RUNS is the program's choice, one line of its header,
 * `// build: scheduler = ram` (the default, the line absent) or
 * `// build: scheduler = flash`. The linker places the path - the two
 * vectors, the thread entry, the request, the scheduler and its queues'
 * verbs - in SRAM or leaves it in the flash (ch32x035/ld/scheduler_ram/
 * names the sections), and the build tells this file which with
 * BRIO_CH32_SCHEDULER_IN_RAM, 1 or 0 (ch32x035/CMakeLists.txt; a unit
 * compiled outside the project, with the macro undefined, is taken as 0).
 * Two facts follow it here.
 *
 * The family's fact the core file asks, `scheduling_request_posted`, is
 * where the request runs: from the flash the request's store is followed
 * by the trap at once - measured on the CH32X035F8U6 (the platform suite's
 * letter i), mepc read in the handler the store's next address, and a
 * `fence.i` after the store changing nothing - so there the request is
 * the store alone. From SRAM the same store is posted: three compressed
 * instructions retire behind it before the trap (two with corecfgr
 * cleared), and two behind the `csrsi` that unmasks a pending request,
 * measured on the CH32X035F8U6 with the sequence placed in SRAM (the
 * platform suite's letter j, which reads mepc in the handler on every
 * run); a `fence.i` after the store closes them, the trap then taken at the
 * instruction after the fence, for 2 cycles (12 from the store to the
 * handler where 10 were). So under `ram` the request ends with the
 * `fence.i`, as the CH32V203's does, and a dispatch the request starts
 * never finds an instruction of its caller run after the post; the
 * unmask's two instructions are the V4B's situation, which the platform's
 * critical section accepts (qingke/schedule.hpp).
 *
 * And the family's step of scheduler_init(): under `ram` the vector table
 * goes to SRAM too. From SRAM a trap through the crt's table in the flash
 * costs 18 cycles from the store that raises it to the handler, and 10
 * through a table in SRAM - the core fetches the entry over the flash's
 * wait states (measured on the CH32X035F8U6, the platform suite's letter
 * j) - so scheduler_vector_table_
 * init() copies the crt's 55 words into a table in SRAM and points mtvec
 * at the copy with both mode bits, the same entries naming the same
 * handlers: a program binds nothing new. The crt's table stays where the
 * core boots from and serves every trap until then. The copy is a
 * function of this file and not of the crt because the crt is assembled
 * for every image with the image's value of the macro, a Tenuto image's
 * included, and the default must cost a Tenuto image nothing: the table
 * and the copy exist only in an image that calls scheduler_init(), which
 * a preemptive program does once, in main(), before it enables
 * interrupts.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"
#include "ch32x035/pfic.hpp"

namespace brio {

/// Does this image run the scheduler path from SRAM? The build's
/// `// build: scheduler` line, through BRIO_CH32_SCHEDULER_IN_RAM (the
/// file header).
#if defined(BRIO_CH32_SCHEDULER_IN_RAM) && BRIO_CH32_SCHEDULER_IN_RAM
inline constexpr bool scheduler_in_ram = true;
#else
inline constexpr bool scheduler_in_ram = false;
#endif

/// The request's store is posted where the request runs from SRAM, and
/// not where it runs from the flash: the request ends with a fence.i in
/// the first image and is the store alone in the second (the file
/// header).
inline constexpr bool scheduling_request_posted = scheduler_in_ram;

/// The copy of the crt's vector table mtvec points at in an image that
/// runs the scheduler path from SRAM: 55 words, aligned to four as mtvec's
/// base field needs (RM 7.5.3.2), zeroed by the crt with .bss and filled
/// by scheduler_vector_table_init(). Only an image that calls that
/// function under `ram` holds it.
alignas(4) inline uint32_t scheduler_vector_table[vector_count];

/// The family's step of scheduler_init() (qingke/schedule.hpp): under
/// `ram`, the crt's table copied word for word into
/// scheduler_vector_table and mtvec pointed at the copy, vectored with
/// absolute addresses; under `flash`, nothing. Called with interrupts
/// masked, before the first is enabled; a trap taken before it - an
/// exception - goes through the crt's table to the same handler.
inline void scheduler_vector_table_init() {
    if constexpr (scheduler_in_ram) {
        // The crt's table, `_start` (ch32x035/src/glue/startup_ch32x035.S:
        // entry 0 the reset jump, entries 1..54 the handlers' addresses),
        // its address taken in assembly so that no declaration of the
        // symbol here meets a program's own.
        const uint32_t* crt_table;
        __asm__("la %0, _start" : "=r"(crt_table));
        for (uint8_t i = 0; i < vector_count; ++i) {
            scheduler_vector_table[i] = crt_table[i];
        }
        const uint32_t base = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(scheduler_vector_table));
        __asm__ volatile("csrw mtvec, %0" ::"r"(base | 3UL) : "memory");
    }
}

} // namespace brio

#include "qingke/schedule.hpp"
