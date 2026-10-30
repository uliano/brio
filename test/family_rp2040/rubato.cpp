// Rubato family smoke TU: the RP2040's preemptive platform, one per core
// (Rp2040RubatoPlatform<core, TB>, rp2040/platform.hpp), under the Rubato
// kernel, and the two ways of binding the scheduler's exception - the
// core stratum's single body (cortexm/schedule.hpp) for a Rubato on core
// 0 alone, and this chip's pair body (rp2040/schedule.hpp) for a Rubato
// on each core. The platforms are the Tenuto ones plus three members, so
// the concept checks come first, then a pack per core whose post,
// scheduler, time events and handler bodies are all instantiated, and a
// crossing from one core's pack to the other's.
#include "rp2040/platform.hpp"
#include "rp2040/schedule.hpp"
#include "cortexm/schedule.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/ticker.hpp"
#include "util/inbox.hpp"

using namespace brio;

using P0 = Rp2040RubatoPlatform<0>;
using P1 = Rp2040RubatoPlatform<1>;

static_assert(Platform<P0>);
static_assert(Platform<P1>);
static_assert(Preemptive<P0>);
static_assert(Preemptive<P1>);
static_assert(!Preemptive<Rp2040Platform<0>>);
static_assert(!Preemptive<Rp2040Platform<1>>);
static_assert(P0::core == 0u && P1::core == 1u);
static_assert(P0::ticks_per_second == 1000);

struct Hi0 {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P0> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

struct Lo0 {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P0> queue;
    static inline TimeEvent<P0, Lo0, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) { post<Hi0>(Hi0::Event{2}); }
};

struct Hi1 {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P1> queue;
    static void init() {}
    static void dispatch(const Event&) { send<Hi0>(Hi0::Event{3}); }
};

struct Lo1 {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P1> queue;
    static inline TimeEvent<P1, Lo1, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) { post<Hi1>(Hi1::Event{2}); }
};

using K0 = Rubato<P0, Hi0, Lo0>;
using K1 = Rubato<P1, Hi1, Lo1>;

void rubato_verbs_core0() {
    scheduler_init();
    K0::init_all();
    post<Lo0>(Lo0::Event{1});
    (void)K0::activate(255);
    K0::idle_if_empty();
    Lo0::alarm.arm(10);
    TimeEvents<P0>::process();
    Lo0::alarm.disarm();
    P0::pend_schedule();
    (void)P0::ceiling;
}

void rubato_verbs_core1() {
    scheduler_init();
    K1::init_all();
    post<Lo1>(Lo1::Event{1});
    (void)K1::activate(255);
    K1::idle_if_empty();
    Lo1::alarm.arm(10);
    TimeEvents<P1>::process();
    Lo1::alarm.disarm();
    P1::pend_schedule();
    (void)P1::ceiling;
}

// The two bindings, each under a name of its own here (a program binds
// one of them to isr_pendsv).
extern "C" [[gnu::naked]] void pendsv_core0_alone() { scheduler_exception<K0>(); }
extern "C" [[gnu::naked]] void isr_pendsv() { scheduler_exception_pair<K0, K1>(); }
extern "C" [[gnu::naked]] void isr_svcall() { scheduler_return(); }
// The doorbells: a crossing lands as a local post, road 1 on its core.
extern "C" void isr_sio_proc0() { Inboxes<Hi0>::isr(); }
extern "C" void isr_sio_proc1() { Inboxes<Hi1, Lo1>::isr(); }
