// Rubato family smoke TU: the SAM C21's preemptive platform
// (SamRubatoPlatform, samc21/platform.hpp) under the Rubato kernel, and
// the core's scheduling request below it (cortexm/schedule.hpp) - on the
// E, G and J headers. The platform is the Tenuto one plus three members,
// so the concept checks come first, then a pack whose post, scheduler,
// time events and handler bodies are all instantiated, the SVC body
// bound by this crt's name for entry 11, SVCall_Handler.
#include "samc21/platform.hpp"
#include "cortexm/schedule.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "samc21/ticker.hpp"

using namespace brio;

using P = SamRubatoPlatform;

static_assert(Platform<P>);
static_assert(Preemptive<P>);
static_assert(!Preemptive<SamPlatform>);
static_assert(P::ticks_per_second == 1000);
static_assert(P::atomic_width == 4);

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
    (void)K::activate(255);
    K::idle_if_empty();
    Lo::alarm.arm(10);
    TimeEvents<P>::process();
    Lo::alarm.disarm();
    P::pend_schedule();
    (void)P::ceiling;
}

extern "C" [[gnu::naked]] void PendSV_Handler() { scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVCall_Handler() { scheduler_return(); }
