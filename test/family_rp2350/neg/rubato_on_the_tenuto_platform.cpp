// Rubato runs on a preemptive platform only: Rp2350Platform has no
// ceiling and no scheduling request, so the kernel refuses it - on both
// halves.
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "rp2350/platform.hpp"

using P = brio::Rp2350Platform<>;

struct Ao {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

void f() { (void)brio::Rubato<P, Ao>::activate(255); }
