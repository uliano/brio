// Host tests for util/meter_sampler.hpp: the latch that bridges a
// capture interrupt to the loop (freshness, overwrite counting), and the
// sampler that paces PUBLICATION rather than capture - fresh values
// published in pack order, stale ones silent, widths widened.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/time.hpp"
#include "util/meter_sampler.hpp"

namespace {

using brio::HostPlatform;
using brio::MeterSample;

// Three sources of two kinds. Two are real latches - one 32-bit, one
// 16-bit, which the sampler must widen - and the third is a SCRIPTED
// source: a plain type with a take(), which is all the concept asks for
// and which is how a peripheral with a readable register would join in.
using Wide = brio::MeterLatch<uint32_t, HostPlatform, 0>;
using Narrow = brio::MeterLatch<uint16_t, HostPlatform, 1>;

struct Scripted {
    static inline std::vector<uint32_t> script;   // consumed front to back
    static inline std::size_t at = 0;
    static std::optional<uint32_t> take() {
        if (at >= script.size()) {
            return std::nullopt;
        }
        return script[at++];
    }
};

static_assert(brio::MeterSource<Wide>);
static_assert(brio::MeterSource<Narrow>);
static_assert(brio::MeterSource<Scripted>);

// A latch of the same width but a different id is a different object:
// the type IS the latch.
using Twin = brio::MeterLatch<uint16_t, HostPlatform, 2>;

struct Sink : brio::Fsm<Sink, MeterSample> {
    using Base = brio::Fsm<Sink, MeterSample>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 16, HostPlatform> queue;
    static inline std::vector<MeterSample> got;
    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::Exit) { return Base::handled(); },
            [](MeterSample s) { got.push_back(s); return Base::handled(); });
    }
};

using Sampler = brio::MeterSampler<HostPlatform, brio::Subscribers<Sink>,
                                   Wide, Narrow, Scripted>;

void run_scheduler() {
    for (;;) {
        if (auto e = Sampler::queue.pop()) {
            Sampler::dispatch(*e);
        } else if (auto s = Sink::queue.pop()) {
            Sink::dispatch(*s);
        } else {
            break;
        }
    }
}

/// One kernel tick: mature the time events, then drain.
void tick() {
    ++HostPlatform::ticks;
    brio::TimeEvents<HostPlatform>::process();
    run_scheduler();
}

void reset(uint32_t period = 4) {
    HostPlatform::reset();
    brio::TimeEvents<HostPlatform>::clear_all();
    Wide::clear();
    Narrow::clear();
    Twin::clear();
    Scripted::script.clear();
    Scripted::at = 0;
    Sink::got.clear();
    while (Sink::queue.pop().has_value()) {}
    while (Sampler::queue.pop().has_value()) {}
    Sink::init();
    Sampler::init(period);
}

} // namespace

TEST_CASE("the latch holds the last value and reports it once") {
    reset();
    CHECK(!Wide::fresh());
    CHECK(!Wide::take().has_value());

    Wide::store(1234);
    CHECK(Wide::fresh());
    auto v = Wide::take();
    REQUIRE(v.has_value());
    CHECK(*v == 1234u);
    CHECK(!Wide::fresh());
    CHECK(!Wide::take().has_value());          // read AND clear
    CHECK(Wide::missed() == 0);
}

TEST_CASE("an overwrite of an untaken value is counted, and the last one wins") {
    reset();
    Wide::store(1);
    Wide::store(2);
    Wide::store(3);
    CHECK(Wide::missed() == 2);
    auto v = Wide::take();
    REQUIRE(v.has_value());
    CHECK(*v == 3u);                           // the newest, not the oldest
    Wide::store(4);
    CHECK(Wide::missed() == 2);                // this one landed on nothing
    CHECK(*Wide::take() == 4u);
}

TEST_CASE("latches of the same width but different ids are different objects") {
    reset();
    Narrow::store(7);
    CHECK(!Twin::fresh());
    CHECK(Narrow::fresh());
    Twin::store(9);
    CHECK(*Narrow::take() == 7u);
    CHECK(*Twin::take() == 9u);
}

TEST_CASE("the sampler paces publication, not capture") {
    reset(4);                                   // a tick every 4
    // Twenty captures between two ticks: the wire's rate.
    for (uint32_t i = 1; i <= 20; ++i) {
        Wide::store(100 + i);
    }
    CHECK(Sink::got.empty());                   // nothing yet: no tick has run
    for (int i = 0; i < 3; ++i) {
        tick();
    }
    CHECK(Sink::got.empty());                   // still inside the period
    tick();                                     // the 4th tick matures it
    REQUIRE(Sink::got.size() == 1);
    CHECK(Sink::got[0].index == 0);
    CHECK(Sink::got[0].value == 120u);          // the LAST capture, not the first
    CHECK(Sampler::published() == 1);
    CHECK(Sampler::missed(0) == 19);            // and the pace's cost, counted
}

TEST_CASE("a stale source publishes nothing at all") {
    reset(1);
    Wide::store(50);
    tick();
    REQUIRE(Sink::got.size() == 1);
    CHECK(Sink::got[0].value == 50u);

    for (int i = 0; i < 10; ++i) {              // ten ticks, no captures
        tick();
    }
    CHECK(Sink::got.size() == 1);               // silence, not repeats
    CHECK(Sampler::published() == 1);
    CHECK(Sampler::missed(0) == 0);             // nothing was overwritten

    Wide::store(51);
    tick();
    REQUIRE(Sink::got.size() == 2);
    CHECK(Sink::got[1].value == 51u);
}

TEST_CASE("several sources: labelled by pack order, mixed fresh and stale") {
    reset(1);
    Wide::store(11);
    Scripted::script = {33};
    tick();                                     // 0 fresh, 1 stale, 2 fresh
    REQUIRE(Sink::got.size() == 2);
    CHECK(Sink::got[0].index == 0);
    CHECK(Sink::got[0].value == 11u);
    CHECK(Sink::got[1].index == 2);
    CHECK(Sink::got[1].value == 33u);

    Sink::got.clear();
    Narrow::store(22);
    tick();                                     // only the middle one now
    REQUIRE(Sink::got.size() == 1);
    CHECK(Sink::got[0].index == 1);
    CHECK(Sink::got[0].value == 22u);

    Sink::got.clear();
    Wide::store(1);
    Narrow::store(2);
    Scripted::script.push_back(3);
    tick();                                     // all three, in pack order
    REQUIRE(Sink::got.size() == 3);
    CHECK(Sink::got[0].index == 0);
    CHECK(Sink::got[1].index == 1);
    CHECK(Sink::got[2].index == 2);
}

TEST_CASE("a 16-bit latch widens into the 32-bit event") {
    reset(1);
    Narrow::store(0xFFFF);
    tick();
    REQUIRE(Sink::got.size() == 1);
    CHECK(Sink::got[0].value == 0xFFFFu);
    static_assert(sizeof(MeterSample::value) == 4);
}

TEST_CASE("missed() passes through per source, and is 0 where none is kept") {
    reset(8);
    Narrow::store(1);
    Narrow::store(2);
    Narrow::store(3);
    CHECK(Sampler::missed(0) == 0);             // the wide latch saw nothing
    CHECK(Sampler::missed(1) == 2);
    CHECK(Sampler::missed(2) == 0);             // Scripted keeps no counter
    CHECK(Sampler::missed(9) == 0);             // out of range
}

TEST_CASE("stop() silences the pace and start_every() re-paces it") {
    reset(2);
    CHECK(Sampler::running_every());
    Sampler::stop();
    CHECK(!Sampler::running_every());
    Wide::store(5);
    for (int i = 0; i < 10; ++i) {
        tick();
    }
    CHECK(Sink::got.empty());                   // no tick, no publication
    CHECK(Wide::fresh());                       // the value is still waiting

    Sampler::start_every(1);
    CHECK(Sampler::running_every());
    tick();
    REQUIRE(Sink::got.size() == 1);
    CHECK(Sink::got[0].value == 5u);
}

TEST_CASE("the missed counter saturates instead of wrapping") {
    reset();
    Wide::store(0);
    for (uint32_t i = 0; i < 70000; ++i) {
        Wide::store(i);
    }
    CHECK(Wide::missed() == UINT16_MAX);
}
