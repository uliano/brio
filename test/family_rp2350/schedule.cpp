// Scheduling-request family smoke TU: the core stratum's PendSV request
// and thread-mode scheduler for a preemptive kernel
// (cortexm/schedule.hpp) on the Arm half of this stratum, the Cortex-M33
// with its FPU - the request, the priority setup and the two naked
// handler bodies instantiated over a stand-in kernel. The Hazard3 half
// has no PendSV, and its build of this TU compiles the core selection
// alone.
#include "rp2350/core.hpp"

#if defined(BRIO_RP2350_CORE_M33)
#include "cortexm/schedule.hpp"

namespace {

struct Kernel {
    struct Platform {
        static inline uint8_t ceiling = 255;
    };
    static brio::Activation activate(uint8_t) { return {}; }
};

} // namespace

void schedule_verbs() {
    brio::scheduler_init();
    brio::request_scheduler();
}

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<Kernel>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }
#endif
