// The single binding serves one kernel, core 0's: given core 1's kernel,
// the macro refuses it - a Rubato on core 1 is bound through the pair body
// (BRIO_RP2350_SCHEDULER_VECTORS_PAIR) - on both halves.
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/schedule.hpp"

using P1 = brio::Rp2350RubatoPlatform<1>;

struct Ao1 {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P1> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

using K1 = brio::Rubato<P1, Ao1>;

BRIO_RP2350_SCHEDULER_VECTORS(K1)
