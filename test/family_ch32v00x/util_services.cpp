// Family smoke TU for the util/ services that have no target half of
// their own: the meter latch and its sampler, the trace ring, the input
// scanner and the bus arbiter's completion policy - the composition the
// AVR fixture proves on every DA/DB, here over the CH32V00x platform,
// with the latches fed by a hand-written ISR (this stratum has no
// timer meter yet) and the scanner over real Pins.
#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "kernel/kernel.hpp"
#include "util/bus_master.hpp"
#include "util/input_scanner.hpp"
#include "util/meter_sampler.hpp"
#include "util/trace.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

using PeriodLatch = MeterLatch<uint16_t, P, 0>;
using WidthLatch = MeterLatch<uint16_t, P, 1>;
using WideLatch = MeterLatch<uint32_t, P, 2>;

static_assert(MeterSource<PeriodLatch>);
static_assert(MeterSource<WideLatch>);

struct FreeRunning {
    static std::optional<uint32_t> take() { return std::nullopt; }
};
static_assert(MeterSource<FreeRunning>);

struct Watcher : Fsm<Watcher, MeterSample, InputEdge> {
    static inline EventQueue<Event, 4, P> queue;
    static inline uint32_t last_value = 0;
    static inline uint8_t last_index = 0;
    static inline bool last_active = false;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](MeterSample s) {
                last_index = s.index;
                last_value = s.value;
                return handled();
            },
            [](InputEdge s) {
                last_index = s.index;
                last_active = s.active;
                return handled();
            });
    }
};

using Meters = MeterSampler<P, Subscribers<Watcher>,
                            PeriodLatch, WidthLatch, WideLatch, FreeRunning>;

struct ButtonD4 {
    static bool read() { return !Pin<'D', 4>::read(); }
};
struct ButtonD3 {
    static bool read() { return !Pin<'D', 3>::read(); }
};
static_assert(ScannedInput<ButtonD4>);

using Buttons = InputScanner<P, Subscribers<Watcher>, ScanConfig{}, ButtonD4, ButtonD3>;
using FastButtons = InputScanner<P, Subscribers<Watcher>,
                                 ScanConfig{.stable_samples = 1}, ButtonD4>;
static_assert(Buttons::stable_samples == 3);
static_assert(FastButtons::stable_samples == 1);

using Marks = Trace<32, P>;
using NoMarks = Trace<32, P, false>;
static_assert(sizeof(NoMarks) == 1, "the disabled trace must cost nothing");
static_assert(sizeof(Marks) > sizeof(NoMarks));

Marks marks;
NoMarks no_marks;

struct FakeBus {
    struct Request {
        uint8_t what;
        ReplyTo<BusDone> reply;
    };
    static bool start(const Request&) { return false; }
    static uint8_t status() { return bus_ok; }
};

struct RetryThrice {
    static BusAction on_done(uint8_t status, uint8_t attempt) {
        return (status != bus_ok && attempt < 3) ? BusAction::retry : BusAction::pass;
    }
};

using PlainBus = BusMaster<FakeBus, P>;
using RetryBus = BusMaster<FakeBus, P, 4, RetryThrice>;
static_assert(!bus_policy_may_retry<BusPassThrough>());
static_assert(bus_policy_may_retry<RetryThrice>());

static_assert(ActiveObject<Meters>);
static_assert(ActiveObject<Buttons>);
static_assert(ActiveObject<PlainBus>);
static_assert(ActiveObject<RetryBus>);

using System = Kernel<P, Watcher, Meters, Buttons, PlainBus, RetryBus>;

// The ISR glue an app would write, with a reading of its own making.
void capture_isr() {
    PeriodLatch::store(1234);
    marks.stamp(1, PeriodLatch::missed());
    no_marks.stamp(1, 0);
}

void util_verbs() {
    SysClock::init();
    System::init_all();

    Meters::init(64);
    Buttons::init(4);
    FastButtons::init(1);

    capture_isr();
    WidthLatch::store(77);
    WideLatch::store(0x0001'0000UL);
    (void)System::step();

    (void)Meters::published();
    (void)Meters::missed(0);
    (void)Meters::running_every();
    Meters::start_every(32);
    Meters::stop();

    (void)Buttons::state(0);
    (void)Buttons::settled(1);
    (void)Buttons::all_settled();
    (void)Buttons::edges();
    Buttons::stop();
    Buttons::start_every(8);

    (void)marks.count();
    (void)marks.held();
    (void)marks.at(0).tag;
    marks.clear();
    (void)no_marks.count();
    no_marks.clear();

    post<PlainBus>(FakeBus::Request{1, {}});
    post<RetryBus>(FakeBus::Request{2, {}});
    post<RetryBus>(TransferDone{bus_engine_status});
    (void)PlainBus::rejected_count();
    (void)RetryBus::attempt();
    (void)System::step();
}
