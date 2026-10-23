// mcu: ch32v203c8 ch32v303vc
// Rubato runs on a preemptive platform alone: the Tenuto platform has no
// ceiling and no scheduling request, so a Rubato over it must be REFUSED
// (kernel/platform.hpp's Preemptive), not compiled into a kernel that
// never preempts.
#include "ch32vx03/platform.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"

using P = brio::Ch32vx03Platform<>;

struct Ao {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

bool f() { return brio::Rubato<P, Ao>::activate(); }
