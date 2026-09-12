// Must FAIL: a TimeEvent of core 0's platform cannot post to an AO of
// core 1 (kernel/time_event.hpp's static_assert).
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/platform.hpp"

struct Tick {};
struct Remote : brio::Fsm<Remote, Tick> {
    static inline brio::EventQueue<Event, 4, brio::Rp2040Platform<1>> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

brio::TimeEvent<brio::Rp2040Platform<0>, Remote, Tick> timer{Tick{}};
void arm() { timer.arm(10); }
