// Rubato family smoke TU: the preemptive kernel (kernel/rubato.hpp) over
// the CH32V203's and the CH32V303's Rubato platform
// (Ch32vx03RubatoPlatform<TB>) and the QingKe V4's scheduling request
// (ch32vx03/schedule.hpp), on every part and both ways the project builds
// an image (with the hardware prologue and without it).
#include "ch32vx03/platform.hpp"
#include "ch32vx03/schedule.hpp"
#include "ch32vx03/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"

using namespace brio;

using P = Ch32vx03RubatoPlatform<>;

static_assert(Platform<P>);
static_assert(HandlerAware<P>);   // in_handler() inherited from the Tenuto platform
static_assert(Preemptive<P>);
static_assert(!Preemptive<Ch32vx03Platform<>>);
static_assert(std::same_as<P::CriticalSection, Ch32vx03Platform<>::CriticalSection>);
static_assert(P::ticks_per_second == Ch32vx03Platform<>::ticks_per_second);

struct High {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, High, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

struct Low {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static void init() {}
    static void dispatch(const Event&) { post<High>(High::Event{2}); }
};

using K = Rubato<P, High, Low>;

void kernel_paths() {
    K::init_all();
    High::alarm.arm(10);
    post<Low>(Low::Event{1});
    (void)K::activate(255);
    K::idle_if_empty();
    P::pend_schedule();
    scheduler_init();
}

[[noreturn]] void kernel_run() { K::run(); }

// The scheduler's two vectors, one spelling whichever way the image is
// built.
BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }
