// mcu: samc21e18a samc21g18a samc21j18a
// Rubato runs on a preemptive platform only: SamPlatform has no ceiling
// and no scheduling request, so the kernel refuses it.
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "samc21/platform.hpp"

using P = brio::SamPlatform;

struct Ao {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

void f() { (void)brio::Rubato<P, Ao>::activate(); }
