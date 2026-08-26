// Host tests for util/input_scanner.hpp: the debounce rule (a bounce
// shorter than N absorbed, a clean press = exactly two edges), the
// silent startup, N = 1 as raw sampling, and several inputs kept apart.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <vector>

#include "host/platform_host.hpp"
#include "util/input_scanner.hpp"

namespace {

using brio::HostPlatform;
using brio::InputEdge;
using brio::ScanConfig;

/// A level the test drives by hand. Polarity lives in the input, so
/// this one is the already-active-true kind the concept asks for.
template <int Tag>
struct Level {
    static inline bool value = false;
    static inline unsigned reads = 0;
    static bool read() {
        ++reads;
        return value;
    }
};

using L0 = Level<0>;
using L1 = Level<1>;

static_assert(brio::ScannedInput<L0>);

/// The active-low wrapper the header documents: a button to ground.
struct PulledUpPin {
    static inline bool pad_high = true;
    static bool read() { return !pad_high; }
};
static_assert(brio::ScannedInput<PulledUpPin>);

template <int Tag>
struct Watch : brio::Fsm<Watch<Tag>, InputEdge> {
    using Base = brio::Fsm<Watch<Tag>, InputEdge>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 16, HostPlatform> queue;
    static inline std::vector<InputEdge> got;
    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::Exit) { return Base::handled(); },
            [](InputEdge s) { got.push_back(s); return Base::handled(); });
    }
};

using Sink3 = Watch<3>;
using Sink1 = Watch<1>;
using SinkP = Watch<9>;

// Three scanners: the default three-sample debounce over two inputs, a
// one-sample one (raw), and a single active-low pin.
using Three = brio::InputScanner<HostPlatform, brio::Subscribers<Sink3>,
                                 ScanConfig{}, L0, L1>;
using Raw = brio::InputScanner<HostPlatform, brio::Subscribers<Sink1>,
                               ScanConfig{.stable_samples = 1}, L0>;
using Pulled = brio::InputScanner<HostPlatform, brio::Subscribers<SinkP>,
                                  ScanConfig{}, PulledUpPin>;

static_assert(Three::stable_samples == 3);
static_assert(Raw::stable_samples == 1);
static_assert(Three::input_count == 2);

template <typename Scanner, typename Sink>
void pump() {
    for (;;) {
        if (auto e = Scanner::queue.pop()) {
            Scanner::dispatch(*e);
        } else if (auto s = Sink::queue.pop()) {
            Sink::dispatch(*s);
        } else {
            break;
        }
    }
}

/// One kernel tick for one scanner.
template <typename Scanner, typename Sink>
void tick() {
    ++HostPlatform::ticks;
    brio::TimeEvents<HostPlatform>::process();
    pump<Scanner, Sink>();
}

/// Drive a level for n ticks.
template <typename Scanner, typename Sink, typename In>
void hold(bool level, int n) {
    In::value = level;
    for (int i = 0; i < n; ++i) {
        tick<Scanner, Sink>();
    }
}

void reset_all() {
    HostPlatform::reset();
    brio::TimeEvents<HostPlatform>::clear_all();
    L0::value = false;
    L1::value = false;
    L0::reads = 0;
    L1::reads = 0;
    PulledUpPin::pad_high = true;
    Sink3::got.clear();
    Sink1::got.clear();
    SinkP::got.clear();
    while (Sink3::queue.pop().has_value()) {}
    while (Sink1::queue.pop().has_value()) {}
    while (SinkP::queue.pop().has_value()) {}
    while (Three::queue.pop().has_value()) {}
    while (Raw::queue.pop().has_value()) {}
    while (Pulled::queue.pop().has_value()) {}
    Sink3::init();
    Sink1::init();
    SinkP::init();
}

} // namespace

TEST_CASE("no edge at startup, even with an input already active") {
    reset_all();
    L0::value = true;                       // held down when the program boots
    L1::value = false;
    Three::init(1);
    CHECK(!Three::settled(0));
    for (int i = 0; i < 5; ++i) {
        tick<Three, Sink3>();
    }
    CHECK(Three::all_settled());
    CHECK(Three::state(0));                 // the level IS known...
    CHECK(!Three::state(1));
    CHECK(Sink3::got.empty());              // ...and nothing was published
    CHECK(Three::edges() == 0);
}

TEST_CASE("a clean press and release publish exactly two edges") {
    reset_all();
    Three::init(1);
    hold<Three, Sink3, L0>(false, 4);       // settle low
    REQUIRE(Sink3::got.empty());

    hold<Three, Sink3, L0>(true, 5);
    REQUIRE(Sink3::got.size() == 1);
    CHECK(Sink3::got[0].index == 0);
    CHECK(Sink3::got[0].active);
    CHECK(Three::state(0));

    hold<Three, Sink3, L0>(false, 5);
    REQUIRE(Sink3::got.size() == 2);
    CHECK(Sink3::got[1].index == 0);
    CHECK(!Sink3::got[1].active);
    CHECK(!Three::state(0));
    CHECK(Three::edges() == 2);
}

TEST_CASE("a bounce shorter than N is absorbed entirely") {
    reset_all();
    Three::init(1);
    hold<Three, Sink3, L0>(false, 4);

    // Two ticks high, back low, one tick high: never three in a row.
    L0::value = true;  tick<Three, Sink3>(); tick<Three, Sink3>();
    L0::value = false; tick<Three, Sink3>();
    L0::value = true;  tick<Three, Sink3>();
    L0::value = false; tick<Three, Sink3>(); tick<Three, Sink3>();
    CHECK(Sink3::got.empty());
    CHECK(!Three::state(0));

    // The same bounce followed by a level that HOLDS: one edge, late by
    // the debounce and not by the bounce.
    L0::value = true;  tick<Three, Sink3>();
    L0::value = false; tick<Three, Sink3>();
    L0::value = true;
    tick<Three, Sink3>();
    CHECK(Sink3::got.empty());              // one sample in
    tick<Three, Sink3>();
    CHECK(Sink3::got.empty());              // two
    tick<Three, Sink3>();
    REQUIRE(Sink3::got.size() == 1);        // three: believed
    CHECK(Sink3::got[0].active);
}

TEST_CASE("N = 1 degenerates to raw sampling") {
    reset_all();
    Raw::init(1);
    tick<Raw, Sink1>();                     // settles on the first read
    CHECK(Raw::all_settled());
    CHECK(Sink1::got.empty());

    L0::value = true;  tick<Raw, Sink1>();
    L0::value = false; tick<Raw, Sink1>();
    L0::value = true;  tick<Raw, Sink1>();
    CHECK(Sink1::got.size() == 3);          // every bounce is an edge now
    CHECK(Sink1::got[0].active);
    CHECK(!Sink1::got[1].active);
    CHECK(Sink1::got[2].active);
}

TEST_CASE("several inputs are debounced independently") {
    reset_all();
    Three::init(1);
    hold<Three, Sink3, L0>(false, 4);
    L0::reads = 0;
    L1::reads = 0;

    // L1 goes active while L0 bounces uselessly.
    L1::value = true;
    L0::value = true;  tick<Three, Sink3>();
    L0::value = false; tick<Three, Sink3>();
    L0::value = true;  tick<Three, Sink3>();
    REQUIRE(Sink3::got.size() == 1);
    CHECK(Sink3::got[0].index == 1);        // only the steady one spoke
    CHECK(Sink3::got[0].active);
    CHECK(Three::state(1));
    CHECK(!Three::state(0));
    CHECK(L0::reads == 3);                  // one read per input per tick
    CHECK(L1::reads == 3);
}

TEST_CASE("polarity is the input's: an active-low pin reads active when low") {
    reset_all();
    Pulled::init(1);
    for (int i = 0; i < 4; ++i) {
        tick<Pulled, SinkP>();
    }
    CHECK(!Pulled::state(0));               // pad high = not pressed
    CHECK(SinkP::got.empty());

    PulledUpPin::pad_high = false;          // pressed: pulled to ground
    for (int i = 0; i < 3; ++i) {
        tick<Pulled, SinkP>();
    }
    REQUIRE(SinkP::got.size() == 1);
    CHECK(SinkP::got[0].active);
    CHECK(Pulled::state(0));
}

TEST_CASE("stop() and start_every() pace the scan") {
    reset_all();
    Three::init(2);                          // every other tick
    CHECK(Three::running_every());
    L0::reads = 0;
    for (int i = 0; i < 8; ++i) {
        tick<Three, Sink3>();
    }
    CHECK(L0::reads == 4);
    Three::stop();
    CHECK(!Three::running_every());
    for (int i = 0; i < 8; ++i) {
        tick<Three, Sink3>();
    }
    CHECK(L0::reads == 4);                   // nothing polled while stopped
    Three::start_every(1);
    tick<Three, Sink3>();
    CHECK(L0::reads == 5);
}

TEST_CASE("out-of-range indices answer without reading anything") {
    reset_all();
    Three::init(1);
    CHECK(!Three::state(7));
    CHECK(!Three::settled(7));
}
