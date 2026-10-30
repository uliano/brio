// Rubato runs on a preemptive platform only: Rp2040Platform has no
// ceiling and no scheduling request, so the kernel refuses it - on
// either core - rather than compiling a kernel that never preempts.
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "rp2040/platform.hpp"

using P = brio::Rp2040Platform<1>;

struct Ao {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

void f() { (void)brio::Rubato<P, Ao>::activate(255); }
