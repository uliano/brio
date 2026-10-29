// Rubato pair family smoke TU: a Rubato on each core of the RP2350
// (Rp2350RubatoPlatform<0> and <1>, rp2350/platform.hpp) through the one
// table both crts give a pair - the pair body that asks the core which
// kernel to serve (rp2350/schedule.hpp), bound by the one macro both
// halves spell the same: the core stratum's PendSV with SIO's CPUID on
// the Cortex-M33, the software interrupt's trampoline body with mhartid
// on Hazard3. Each core's pack has its post, scheduler, time events and
// request instantiated, a crossing from one core's pack to the other's
// and the doorbell's drain that lands it as a local post.
#include "rp2350/platform.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/ticker.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "util/inbox.hpp"

using namespace brio;

using P0 = Rp2350RubatoPlatform<0>;
using P1 = Rp2350RubatoPlatform<1>;

static_assert(Platform<P0> && Platform<P1>);
static_assert(Preemptive<P0> && Preemptive<P1>);
static_assert(P0::core == 0u && P1::core == 1u);
static_assert(std::same_as<P1::Doorbell, SioDoorbell<1>>);

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
    static constexpr uint8_t inbox_depth = 4;
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
    send<Hi1>(Hi1::Event{4});
    (void)K0::activate();
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
    (void)K1::activate();
    K1::idle_if_empty();
    Lo1::alarm.arm(10);
    TimeEvents<P1>::process();
    Lo1::alarm.disarm();
    P1::pend_schedule();
    (void)P1::ceiling;
}

[[noreturn]] void kernel_run_core1() { K1::run(); }

// The scheduler's vectors for two kernels: isr_pendsv and isr_svcall on
// one half, isr_riscv_softirq on the other.
BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K0, K1)

// The doorbell, one name for both cores: a crossing lands as a local post,
// road 1 on its core.
extern "C" void isr_sio_bell() {
    if (core_id() == 0u) {
        Inboxes<Hi0>::isr();
    } else {
        Inboxes<Hi1>::isr();
    }
}
