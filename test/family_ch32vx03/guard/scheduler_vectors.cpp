// guard: pass
//
// The preemptive kernel's two vectors (ch32vx03/schedule.hpp) in an image
// built with F: the software interrupt's body and the environment call's
// are naked and marked leaves, and the guard must hold them to that and
// let them through - and it must not walk into the scheduler they return
// into, which runs in thread mode under the entry's own f-register save,
// so a dispatch that does float work is no fault of a vector's.
#include <stdint.h>

#include "ch32vx03/platform.hpp"
#include "ch32vx03/schedule.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"

using P = brio::Ch32vx03RubatoPlatform<>;

volatile float acc = 1.0f;

struct Worker {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event& e) { acc = acc * 1.5f + static_cast<float>(e.n); }
};

using K = brio::Rubato<P, Worker>;

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }

int main() {
    brio::scheduler_init();
    brio::post<Worker>(Worker::Event{2});
    (void)K::activate();
    for (;;) {
    }
}
