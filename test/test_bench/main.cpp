// Host tests for util/bench.hpp: the benchmark grammar - the ruler's
// wrap, the busy formula over idle windows and handler stamps (inside a
// window, preempting the thread), the adapter's platform face, and the
// line as characters on the wire, computed from numbers chosen so that
// every field is checked by hand.
// Run with: ctest --preset host (or ctest --preset host -R test_bench)
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>
#include <cstdint>
#include <optional>
#include <string>
#include "kernel/platform.hpp"
#include "util/bench.hpp"

namespace {

// ---- a capture sink ---------------------------------------------------------
struct Capture {
    static inline std::string text;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
    static void clear() { text.clear(); }
};
static_assert(brio::ByteSink<Capture>);

// ---- a ruler the test advances by hand ----------------------------------------
struct FakeRuler {
    static inline uint32_t t = 0;
    static uint32_t now() { return t; }
    static uint32_t hz() { return 48'000'000u; }
};
static_assert(brio::CycleRuler<FakeRuler>);

// ---- a platform whose idle() sleeps 100 cycles and may run a handler ----------
struct FakePlatform {
    struct CriticalSection {};
    static inline void (*on_idle)() = nullptr;
    static inline uint32_t idle_calls = 0;
    static inline brio::PanicRecord record{};
    static void idle() {
        ++idle_calls;
        FakeRuler::t += 100;
        if (on_idle != nullptr) {
            on_idle();
        }
    }
    static void break_here() {}
    static uint32_t now() { return 0; }
    static brio::PanicRecord& panic_record() { return record; }
    static constexpr uint32_t ticks_per_second = 1000;
    static constexpr unsigned atomic_width = 4;
};
static_assert(brio::Platform<FakePlatform>);

struct FakeTickless : FakePlatform {
    static inline std::optional<uint32_t> last_deadline;
    static void idle_until(std::optional<uint32_t> deadline) {
        last_deadline = deadline;
        FakeRuler::t += 200;
    }
};

// ---- one core of several: the optional members the adapter must carry --------
struct FakeDoorbell {};
struct FakeMulticore : FakePlatform {
    using Doorbell = FakeDoorbell;
    static inline bool own = true;
    static bool on_own_core() { return own; }
    static inline void (*sleep_hook)() = nullptr;
};

using Idle = brio::BenchIdle<FakePlatform, FakeRuler>;
using IdleOfCore = brio::BenchIdle<FakeMulticore, FakeRuler>;
using Meter = brio::IsrMeter<FakeRuler, Idle>;
static_assert(brio::Platform<Idle>);
static_assert(brio::IdleWindow<Idle>);
static_assert(brio::IdleWindow<brio::NoIdleWindow>);

template <typename P>
concept HasIdleUntil = requires(std::optional<uint32_t> d) {
    { P::idle_until(d) } -> std::same_as<void>;
};
static_assert(!HasIdleUntil<Idle>);
static_assert(HasIdleUntil<brio::BenchIdle<FakeTickless, FakeRuler>>);

template <typename P>
concept HasCores = requires {
    typename P::Doorbell;
    { P::on_own_core() } -> std::same_as<bool>;
};
template <typename P>
concept HasSleepHook = requires { P::sleep_hook = nullptr; };
static_assert(!HasCores<Idle>);
static_assert(!HasSleepHook<Idle>);
static_assert(HasCores<IdleOfCore>);
static_assert(HasSleepHook<IdleOfCore>);
static_assert(std::same_as<IdleOfCore::Doorbell, FakeDoorbell>);

Meter meter;

void handler_of(uint32_t cycles) {
    meter.enter();
    FakeRuler::t += cycles;
    meter.leave();
}

} // namespace

TEST_CASE("adapter: a core's optional members answer as the platform's own") {
    FakeMulticore::own = false;
    CHECK(!IdleOfCore::on_own_core());
    FakeMulticore::own = true;
    CHECK(IdleOfCore::on_own_core());
    static void (*const hook)() = [] {};
    IdleOfCore::sleep_hook = hook;          // written through the adapter...
    CHECK(FakeMulticore::sleep_hook == hook);  // ...it is the platform's variable
    FakeMulticore::sleep_hook = nullptr;
    CHECK(IdleOfCore::sleep_hook == nullptr);
}

TEST_CASE("stopwatch: exact across the ruler's wrap") {
    FakeRuler::t = 0xFFFFFF00u;
    brio::Stopwatch<FakeRuler> sw;
    sw.start();
    FakeRuler::t += 0x200u;
    CHECK(sw.elapsed() == 0x200u);
}

TEST_CASE("idle adapter: a window holds the sleep and the handler that ended it") {
    FakeRuler::t = 1000;
    FakePlatform::idle_calls = 0;
    const uint32_t cycles0 = Idle::idle_cycles();
    const uint32_t turns0 = Idle::idle_turns();
    FakePlatform::on_idle = [] { handler_of(50); };
    CHECK(!Idle::in_idle());
    Idle::idle();
    CHECK(!Idle::in_idle());
    CHECK(FakePlatform::idle_calls == 1);
    CHECK(Idle::idle_cycles() - cycles0 == 150u);
    CHECK(Idle::idle_turns() - turns0 == 1u);
    FakePlatform::on_idle = nullptr;
}

TEST_CASE("meter: inside a window and preempting the thread are kept apart") {
    FakeRuler::t = 5000;
    FakePlatform::on_idle = [] { handler_of(50); };
    const auto before = brio::bench_counters<Idle>(meter);
    Idle::idle();           // 100 asleep + a 50-cycle handler inside the window
    handler_of(30);         // a handler preempting the thread
    const auto after = brio::bench_counters<Idle>(meter);
    FakePlatform::on_idle = nullptr;
    CHECK(after.irq - before.irq == 2u);
    CHECK(after.isr_cycles - before.isr_cycles == 80u);
    CHECK(after.isr_in_idle - before.isr_in_idle == 50u);
    CHECK(after.idle_cycles - before.idle_cycles == 150u);
    CHECK(meter.cycles() >= 80u);
}

TEST_CASE("busy: wall minus the windows plus the handlers inside them") {
    const brio::BenchCounters before{1000, 400, 200, 7};
    const brio::BenchCounters after{1000 + 600, 400 + 80, 200 + 50, 7 + 3};
    const auto s = brio::bench_sample(1000, before, after);
    CHECK(s.wall == 1000u);
    CHECK(s.busy == 1000u - 600u + 50u);
    CHECK(s.irq == 3u);
    CHECK(s.isr == 80u);
}

TEST_CASE("busy: a handler that preempted the thread is not added twice") {
    brio::IsrMeter<FakeRuler, brio::NoIdleWindow> m;
    FakeRuler::t = 0;
    m.enter();
    FakeRuler::t += 40;
    m.leave();
    CHECK(m.count() == 1u);
    CHECK(m.cycles() == 40u);
    CHECK(m.cycles_in_idle() == 0u);
}

TEST_CASE("the line: rate and x computed from the wire") {
    // 4096 bytes in 4 000 000 cycles at 48 MHz = 49 152 B/s; the wire at
    // 115200 baud, ten bits a byte, is 11 520 B/s = 17 066 666 cycles for
    // the run: x = 0.23. The same bytes in 40 000 000 cycles: x = 2.34.
    Capture::clear();
    brio::bench_line(Capture{}, "print", 4096u, {4'000'000u, 1'000'000u, 4096u, 600'000u}, 48'000'000u,
                     11520u);
    CHECK(Capture::text ==
          "bench print n=4096 wall=4000000 busy=1000000 irq=4096 isr=600000 rate=49152 wire=11520 x=0.23\r\n");
    Capture::clear();
    brio::bench_line(Capture{}, "print", 4096u, {40'000'000u, 1'000'000u, 4096u, 600'000u}, 48'000'000u,
                     11520u);
    CHECK(Capture::text ==
          "bench print n=4096 wall=40000000 busy=1000000 irq=4096 isr=600000 rate=4915 wire=11520 x=2.34\r\n");
}

TEST_CASE("the line: a wire exactly met prints x=1.00, hundredths are two digits") {
    Capture::clear();
    // 256 bytes at the bus: 4 bytes a cycle at 180 MHz = 720 000 000 B/s; 64 cycles is the wire.
    brio::bench_line(Capture{}, "memcpy", 256u, {64u, 64u, 0u, 0u}, 180'000'000u, 720'000'000u);
    CHECK(Capture::text == "bench memcpy n=256 wall=64 busy=64 irq=0 isr=0 rate=720000000 wire=720000000 x=1.00\r\n");
    Capture::clear();
    brio::bench_line(Capture{}, "memcpy", 256u, {67u, 67u, 0u, 0u}, 180'000'000u, 720'000'000u);
    CHECK(Capture::text.find(" x=1.04\r\n") != std::string::npos);
}

TEST_CASE("the line: a zero wall or a zero wire prints a dash, never divides") {
    Capture::clear();
    brio::bench_line(Capture{}, "nop", 0u, {0u, 0u, 0u, 0u}, 48'000'000u, 11520u);
    CHECK(Capture::text == "bench nop n=0 wall=0 busy=0 irq=0 isr=0 rate=- wire=11520 x=-\r\n");
    Capture::clear();
    brio::bench_line(Capture{}, "tick", 0u, {1000u, 60u, 1u, 40u}, 48'000'000u, 0u);
    CHECK(Capture::text == "bench tick n=0 wall=1000 busy=60 irq=1 isr=40 rate=0 wire=0 x=-\r\n");
}
