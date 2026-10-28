/*
 * schedule.hpp
 *
 * The scheduling request of the preemptive kernel on the CH32V203's
 * QingKe V4B and the CH32V303's V4F, and the thread-mode scheduler it
 * starts: `request_scheduler()`, `scheduler_init()`, the two vector
 * bodies `scheduler_trap<K>()` and `scheduler_return()` and their
 * binding BRIO_CH32_SCHEDULER_VECTOR are the QingKe core stratum's
 * (qingke/schedule.hpp, whose account they are: the software interrupt
 * one preemption level below every peripheral line, the return into the
 * scheduler in thread mode, the f-registers the V4F's entry saves). A
 * program binds the two vectors as Ch32vx03RubatoPlatform's comment
 * shows (ch32vx03/platform.hpp):
 *
 *     BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
 *     BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }
 *
 * This family's fact, the one the core file asks: on these two cores the
 * request's store into the PFIC is posted - measured on the CH32V203C8T6
 * and the CH32V303VCT6, three instructions retire behind it before the
 * trap - so the request ends with a `fence.i`, behind which none does.
 */

#pragma once

#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"

namespace brio {

/// The request's store is posted on the V4B and the V4F: the request ends
/// with a fence.i (the file header).
inline constexpr bool scheduling_request_posted = true;

} // namespace brio

#include "qingke/schedule.hpp"
