// The pair binding takes core 0's kernel first and core 1's second: given
// in the other order, the body would send each core into the other's
// scheduler, and its static_asserts refuse it - on both halves.
#include "kernel/event_queue.hpp"
#include "kernel/rubato.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/schedule.hpp"

using P0 = brio::Rp2350RubatoPlatform<0>;
using P1 = brio::Rp2350RubatoPlatform<1>;

struct Ao0 {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P0> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

struct Ao1 {
    struct Event { uint8_t n; };
    static inline brio::EventQueue<Event, 2, P1> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

using K0 = brio::Rubato<P0, Ao0>;
using K1 = brio::Rubato<P1, Ao1>;

BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K1, K0)
