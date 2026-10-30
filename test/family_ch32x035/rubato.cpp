// Rubato family smoke TU: the preemptive kernel (kernel/rubato.hpp) over
// the CH32X035's Rubato platform (Ch32x035RubatoPlatform<TB>) and the
// QingKe V4C's scheduling request (ch32x035/schedule.hpp), on every part,
// both ways the project builds an image (with the hardware prologue and
// without it) and both places the build line puts the scheduler path
// (BRIO_CH32_SCHEDULER_IN_RAM 1 and 0: cli/checks/check_ch32x035.sh).
#include "ch32x035/platform.hpp"
#include "ch32x035/schedule.hpp"
#include "ch32x035/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"

using namespace brio;

using P = Ch32x035RubatoPlatform<>;

static_assert(Platform<P>);
static_assert(HandlerAware<P>);   // in_handler() inherited from the Tenuto platform
static_assert(Preemptive<P>);
static_assert(!Preemptive<Ch32x035Platform<>>);
static_assert(std::same_as<P::CriticalSection, Ch32x035Platform<>::CriticalSection>);
static_assert(P::ticks_per_second == Ch32x035Platform<>::ticks_per_second);
// The request carries the fence.i exactly where it runs from SRAM.
static_assert(scheduling_request_posted == scheduler_in_ram);

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
