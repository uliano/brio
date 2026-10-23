// Rubato family smoke TU: the STM32G0's preemptive platform
// (Stm32g0RubatoPlatform<TB>, stm32g0/platform.hpp) under the Rubato
// kernel, and the core's scheduling request below it
// (cortexm/schedule.hpp) - on every header of the pack, the value line
// included. The platform is the Tenuto one plus three members, so the
// concept checks come first, then a pack whose post, scheduler, time
// events and handler bodies are all instantiated.
#include "stm32g0/platform.hpp"
#include "cortexm/schedule.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/ticker.hpp"

using namespace brio;

using P = Stm32g0RubatoPlatform<>;

static_assert(Platform<P>);
static_assert(Preemptive<P>);
static_assert(!Preemptive<Stm32g0Platform<>>);
static_assert(P::ticks_per_second == 1000);

struct Hi {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) {}
};

struct Lo {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Lo, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) { post<Hi>(Hi::Event{2}); }
};

using K = Rubato<P, Hi, Lo>;

void rubato_verbs() {
    scheduler_init();
    K::init_all();
    post<Lo>(Lo::Event{1});
    (void)K::activate();
    K::idle_if_empty();
    Lo::alarm.arm(10);
    TimeEvents<P>::process();
    Lo::alarm.disarm();
    P::pend_schedule();
    (void)P::ceiling;
}

extern "C" [[gnu::naked]] void PendSV_Handler() { scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { scheduler_return(); }
