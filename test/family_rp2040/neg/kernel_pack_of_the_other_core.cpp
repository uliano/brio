// Must FAIL: an AO whose queue is guarded by core 1's platform cannot sit
// in the pack of core 0's kernel (kernel/active_object.hpp's queue_on).
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "rp2040/platform.hpp"

struct Tick {};
struct Remote : brio::Fsm<Remote, Tick> {
    static inline brio::EventQueue<Event, 4, brio::Rp2040Platform<1>> queue;
    static void init() { start(&only); }
    static Status only(const Event&) { return handled(); }
};

void wrong_pack() { brio::Kernel<brio::Rp2040Platform<0>, Remote>::init_all(); }
