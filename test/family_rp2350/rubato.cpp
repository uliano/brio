// Rubato family smoke TU: the preemptive kernel (kernel/rubato.hpp) over
// the RP2350's Rubato platform (Rp2350RubatoPlatform<core, TB>,
// rp2350/platform.hpp) and this target's scheduling request
// (rp2350/schedule.hpp) - on both halves: the core stratum's PendSV and
// SVC on the Cortex-M33, the machine software interrupt through the crt's
// trampoline and the entry's own mret on Hazard3. The platform is the
// Tenuto one plus three members, so the concept checks come first, then a
// pack whose post, scheduler, time events and vector bodies are all
// instantiated, bound by the one macro both halves spell the same.
#include "rp2350/platform.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"

using namespace brio;

using P = Rp2350RubatoPlatform<>;

static_assert(Platform<P>);
static_assert(Preemptive<P>);
static_assert(!Preemptive<Rp2350Platform<>>);
static_assert(std::same_as<P::CriticalSection, Rp2350Platform<>::CriticalSection>);
static_assert(P::ticks_per_second == Rp2350Platform<>::ticks_per_second);
static_assert(P::core == 0u);

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
    scheduler_init();
    K::init_all();
    High::alarm.arm(10);
    post<Low>(Low::Event{1});
    (void)K::activate(255);
    K::idle_if_empty();
    TimeEvents<P>::process();
    High::alarm.disarm();
    P::pend_schedule();
    request_scheduler();
    (void)P::ceiling;
}

[[noreturn]] void kernel_run() { K::run(); }

// The scheduler's vectors: isr_pendsv and isr_svcall on one half,
// isr_riscv_softirq on the other.
BRIO_RP2350_SCHEDULER_VECTORS(K)
