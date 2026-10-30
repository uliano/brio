// Scheduling-request family smoke TU: the core stratum's PendSV request
// and thread-mode scheduler for a preemptive kernel
// (cortexm/schedule.hpp), included after this family's device selection,
// on this family's Cortex-M0+ - the request, the priority setup and the
// two naked handler bodies instantiated over a stand-in kernel, so that
// the ARMv6-M build assembles every instruction of them.
#include "stm32g0/nvic.hpp"
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
