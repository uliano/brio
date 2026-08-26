// Family smoke TU for the util/ services that have no target half of
// their own: the meter latch and its sampler, the trace ring, the input
// scanner and the bus arbiter's completion policy.
//
// util/ is target-independent, so nothing here is package-dependent by
// itself; what this TU proves is that the whole composition instantiates
// on every DA/DB - the latches fed by the REAL TCB meter ISR bodies, a
// scanner over real Pins, a trace over the AVR platform's clock, and a
// bus arbiter with a retry policy - and that the pieces still agree once
// they are held together by a kernel pack.
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/tcb.hpp"
#include "kernel/kernel.hpp"
#include "util/bus_master.hpp"
#include "util/input_scanner.hpp"
#include "util/meter_sampler.hpp"
#include "util/trace.hpp"

using namespace brio;

using P = AvrPlatform;
using SysClock = Clock<ClockSource::internal, 24'000'000>;

// ---- the meter latches and their sampler ---------------------------------------
// Two latches of the SAME width: what makes them two objects is the id.
using PeriodLatch = MeterLatch<uint16_t, P, 0>;
using WidthLatch = MeterLatch<uint16_t, P, 1>;
using WideLatch = MeterLatch<uint32_t, P, 2>;

static_assert(MeterSource<PeriodLatch>);
static_assert(MeterSource<WidthLatch>);
static_assert(MeterSource<WideLatch>);

/// A source that is not a latch: the concept asks for take() and nothing
/// else, which is how a peripheral with a readable register joins in.
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

// ---- the input scanner ---------------------------------------------------------
// PORTA exists on every package of the family; polarity lives in the
// input, so the button to ground inverts here and nowhere else.
struct ButtonA2 {
    static bool read() { return !Pin<'A', 2>::read(); }
};
struct ButtonA3 {
    static bool read() { return !Pin<'A', 3>::read(); }
};
static_assert(ScannedInput<ButtonA2>);

using Buttons = InputScanner<P, Subscribers<Watcher>, ScanConfig{}, ButtonA2, ButtonA3>;
using FastButtons = InputScanner<P, Subscribers<Watcher>,
                                 ScanConfig{.stable_samples = 1}, ButtonA2>;
static_assert(Buttons::stable_samples == 3);
static_assert(FastButtons::stable_samples == 1);

// ---- the trace -----------------------------------------------------------------
using Marks = Trace<32, P>;
using NoMarks = Trace<32, P, false>;
static_assert(sizeof(NoMarks) == 1, "the disabled trace must cost nothing");
static_assert(sizeof(Marks) > sizeof(NoMarks));

Marks marks;
NoMarks no_marks;

// ---- the bus arbiter and its policy --------------------------------------------
struct FakeBus {
    struct Request {
        uint8_t what;
        ReplyTo<BusDone> reply;
    };
    static bool start(const Request&) { return false; }
};

/// The retry half of the hook: three attempts on any engine failure.
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

// ---- the ISR glue an app would write --------------------------------------------
// The drivers stay untouched: the meter's ISR body returns the reading
// and the vector binding stores it into a latch.
using Meter = Tcb<0>;

void capture_isr() {
    PeriodLatch::store(FrequencyMeter<Meter>::period_ticks());
    marks.stamp(1, PeriodLatch::missed());
    no_marks.stamp(1, 0);
}

void width_isr() {
    WidthLatch::store(PulseWidthMeter<Tcb<1>>::width_ticks());
}

void util_verbs() {
    SysClock::init();
    System::init_all();

    FrequencyMeter<Meter>::init(SysClock{}, EventChannel<0>{});
    PulseWidthMeter<Tcb<1>>::init(SysClock{}, EventChannel<1>{});

    Meters::init(64);
    Buttons::init(4);
    FastButtons::init(1);

    capture_isr();
    width_isr();
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
