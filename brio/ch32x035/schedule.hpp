/*
 * schedule.hpp
 *
 * The scheduling request of the preemptive kernel on the QingKe V4C of
 * the CH32X035, and the thread-mode scheduler it starts:
 * `request_scheduler()`, `scheduler_init()`, the two vector bodies
 * `scheduler_trap<K>()` and `scheduler_return()` and their binding
 * BRIO_CH32_SCHEDULER_VECTOR are the QingKe core stratum's
 * (qingke/schedule.hpp, whose account they are: the software interrupt
 * one preemption level below every peripheral line, the return into the
 * scheduler in thread mode, the way out through the machine-mode
 * environment call). A program binds the two vectors as
 * Ch32x035RubatoPlatform's comment shows (ch32x035/platform.hpp):
 *
 *     BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
 *     BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }
 *
 * This family's fact, the one the core file asks: run from the flash,
 * where every image of this project places it, the request's store is
 * followed by the trap at once - measured on the CH32X035F8U6 (the
 * platform suite's letter i), mepc read in the handler the store's next
 * address, and a `fence.i` after the store changing nothing - so the
 * request is the store alone. That is a fact of the flash's fetch, not of
 * the core: the same store run from SRAM retires three compressed
 * instructions before the trap (measured on the CH32X035F8U6 with the
 * sequence placed in SRAM), so a request placed in SRAM would carry the
 * `fence.i` as the CH32V203's does; qingke/schedule.hpp says why the
 * fact is the family's to state.
 */

#pragma once

#include "ch32x035/device.hpp"
#include "ch32x035/pfic.hpp"

namespace brio {

/// The request's store is not posted on the V4C: the request is the
/// store alone (the file header).
inline constexpr bool scheduling_request_posted = false;

} // namespace brio

#include "qingke/schedule.hpp"
