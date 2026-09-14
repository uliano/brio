// Host tests for util/quadrature.hpp: the transition table over its
// WHOLE state space, the detent divisor, the silent startup, bounce,
// and - the one that matters - what the decoder reports when the shaft
// outruns the poll.
// Run with: ctest --preset host (or ctest --preset host -R test_quadrature)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <vector>

#include "host/platform.hpp"
#include "host/sim_input.hpp"
#include "util/quadrature.hpp"

namespace {

using brio::HostPlatform;
using brio::QuadratureConfig;
using brio::Turned;

using Knob = brio::SimEncoder<0>;

template <int Tag>
struct Watch : brio::Fsm<Watch<Tag>, Turned> {
    using Base = brio::Fsm<Watch<Tag>, Turned>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 32, HostPlatform> queue;
    static inline std::vector<int8_t> got;
    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::Exit) { return Base::handled(); },
            [](Turned t) { got.push_back(t.detents); return Base::handled(); });
    }
};

using SinkFour = Watch<4>;
using SinkTwo = Watch<2>;
using SinkOpt = Watch<8>;

// The common part: four counts a detent, keeping still when a state is
// missed. Then a two-count part, and the optimistic policy.
using Four = brio::Quadrature<HostPlatform, brio::Subscribers<SinkFour>,
                              Knob::A, Knob::B, QuadratureConfig{}>;
using Two = brio::Quadrature<HostPlatform, brio::Subscribers<SinkTwo>,
                             Knob::A, Knob::B,
                             QuadratureConfig{.counts_per_detent = 2}>;
using Opt = brio::Quadrature<HostPlatform, brio::Subscribers<SinkOpt>,
                             Knob::A, Knob::B,
                             QuadratureConfig{.count_lost_steps = true}>;

static_assert(Four::counts_per_detent == 4);
static_assert(Two::counts_per_detent == 2);

template <typename Dec, typename Sink>
void pump() {
    for (;;) {
        if (auto e = Dec::queue.pop()) {
            Dec::dispatch(*e);
        } else if (auto s = Sink::queue.pop()) {
            Sink::dispatch(*s);
        } else {
            break;
        }
    }
}

/// One kernel tick: the decoder samples once.
template <typename Dec, typename Sink>
void tick() {
    ++HostPlatform::ticks;
    brio::TimeEvents<HostPlatform>::process();
    pump<Dec, Sink>();
}

/// A decoder armed on every tick, with the knob and the sink cleared.
template <typename Dec, typename Sink>
void begin() {
    HostPlatform::reset();
    Knob::reset();
    Sink::got.clear();
    Sink::init();
    Dec::init(1);
    tick<Dec, Sink>(); // the first reading only establishes where it sits
}

/// Turn the shaft by `counts`, one per tick - a hand the poll keeps up
/// with.
template <typename Dec, typename Sink>
void turn(int8_t counts) {
    const int8_t dir = counts >= 0 ? 1 : -1;
    for (int8_t i = 0; i < (counts >= 0 ? counts : -counts); ++i) {
        Knob::step(dir);
        tick<Dec, Sink>();
    }
}

int total(const std::vector<int8_t>& v) {
    int s = 0;
    for (int8_t d : v) {
        s += d;
    }
    return s;
}

} // namespace

// ---------------------------------------------------------------------
// The transition table, over the whole of its state space.
// ---------------------------------------------------------------------

TEST_CASE("every transition between two states is classified") {
    // Four states, four destinations: sixteen cases and not a sample of
    // them. The Gray order a turn walks is 00, 01, 11, 10.
    constexpr uint8_t cycle[4] = {0b00, 0b01, 0b11, 0b10};

    for (int i = 0; i < 4; ++i) {
        const uint8_t here = cycle[i];
        const uint8_t ahead = cycle[(i + 1) % 4];
        const uint8_t behind = cycle[(i + 3) % 4];
        const uint8_t across = cycle[(i + 2) % 4];

        INFO("from state " << int(here));
        const auto still = brio::quadrature_step(here, here);
        REQUIRE(still.has_value());
        CHECK(*still == 0);

        const auto fwd = brio::quadrature_step(here, ahead);
        REQUIRE(fwd.has_value());
        CHECK(*fwd == 1);

        const auto back = brio::quadrature_step(here, behind);
        REQUIRE(back.has_value());
        CHECK(*back == -1);

        // Two states away is the only transition where BOTH contacts
        // changed, and it is equally far in either direction.
        CHECK(!brio::quadrature_step(here, across).has_value());
    }
}

TEST_CASE("a full turn of the table returns to where it started") {
    uint8_t s = 0;
    int sum = 0;
    constexpr uint8_t cycle[5] = {0b00, 0b01, 0b11, 0b10, 0b00};
    for (int i = 0; i < 4; ++i) {
        const auto step = brio::quadrature_step(s, cycle[i + 1]);
        REQUIRE(step.has_value());
        sum += *step;
        s = cycle[i + 1];
    }
    CHECK(sum == 4);   // four counts make one revolution of the cycle
    CHECK(s == 0);
}

// ---------------------------------------------------------------------
// The decoder.
// ---------------------------------------------------------------------

TEST_CASE("where the knob sits at startup is not a turn") {
    HostPlatform::reset();
    Knob::reset();
    Knob::force(0b11);          // left somewhere by the last user
    SinkFour::got.clear();
    SinkFour::init();
    Four::init(1);
    CHECK(!Four::settled());
    tick<Four, SinkFour>();
    CHECK(Four::settled());
    CHECK(SinkFour::got.empty());
    // and it decodes from there, not from zero
    turn<Four, SinkFour>(4);
    CHECK(total(SinkFour::got) == 1);
}

TEST_CASE("four counts make one detent, and the remainder is kept") {
    begin<Four, SinkFour>();

    turn<Four, SinkFour>(3);
    CHECK(SinkFour::got.empty());       // three is not a detent yet
    turn<Four, SinkFour>(1);
    REQUIRE(SinkFour::got.size() == 1);
    CHECK(SinkFour::got[0] == 1);

    turn<Four, SinkFour>(4);
    REQUIRE(SinkFour::got.size() == 2);
    CHECK(SinkFour::got[1] == 1);
    CHECK(Four::detents() == 2);
}

TEST_CASE("turning back publishes the other sign") {
    begin<Four, SinkFour>();
    turn<Four, SinkFour>(8);
    CHECK(total(SinkFour::got) == 2);
    turn<Four, SinkFour>(-8);
    CHECK(total(SinkFour::got) == 0);
    CHECK(SinkFour::got.size() == 4);
}

TEST_CASE("a part with two counts a detent is told so") {
    begin<Two, SinkTwo>();
    turn<Two, SinkTwo>(2);
    REQUIRE(SinkTwo::got.size() == 1);
    CHECK(SinkTwo::got[0] == 1);
    turn<Two, SinkTwo>(-4);
    CHECK(total(SinkTwo::got) == -1);
}

TEST_CASE("twenty detents of a common part are twenty detents") {
    // 20 positions a revolution, four counts each: a whole turn of the
    // shaft is 80 counts and must be exactly 20 detents.
    begin<Four, SinkFour>();
    turn<Four, SinkFour>(80);
    CHECK(total(SinkFour::got) == 20);
    CHECK(Four::lost() == 0);
}

TEST_CASE("a contact bouncing between two states nets to nothing") {
    begin<Four, SinkFour>();
    // The shaft is between detents and one contact chatters: the states
    // walk back and forth over a legal edge, which is what a real
    // contact does and what no amount of counting can call a turn.
    for (int i = 0; i < 20; ++i) {
        Knob::force(i % 2 == 0 ? 0b01 : 0b00);
        tick<Four, SinkFour>();
    }
    CHECK(total(SinkFour::got) == 0);
    CHECK(Four::lost() == 0);      // every step was legal, just undone
}

TEST_CASE("a state missed is counted and, by default, not guessed") {
    begin<Four, SinkFour>();
    // 00 -> 11 skips 01: two states away, the same distance either way.
    Knob::force(0b11);
    tick<Four, SinkFour>();
    CHECK(Four::lost() == 1);
    CHECK(SinkFour::got.empty());

    // and the decoder picks up cleanly from where it now is
    turn<Four, SinkFour>(4);
    CHECK(total(SinkFour::got) == 1);
}

TEST_CASE("the optimistic policy guesses, in the last known direction") {
    HostPlatform::reset();
    Knob::reset();
    SinkOpt::got.clear();
    SinkOpt::init();
    Opt::init(1);
    tick<Opt, SinkOpt>();

    turn<Opt, SinkOpt>(2);            // establishes forward, 2 counts held
    CHECK(SinkOpt::got.empty());
    Knob::spin(2);                    // two counts between samples
    tick<Opt, SinkOpt>();
    CHECK(Opt::lost() == 1);
    // 2 held + 2 guessed = one whole detent, forward
    REQUIRE(SinkOpt::got.size() == 1);
    CHECK(SinkOpt::got[0] == 1);
}

// ---------------------------------------------------------------------
// What a shaft faster than the poll actually costs. This characterises
// the degradation rather than asserting it away.
// ---------------------------------------------------------------------

TEST_CASE("a shaft outrunning the poll degrades in a known way") {
    struct Outcome {
        int missed;      // counts advanced between two samples
        int reported;    // counts the decoder credited
        bool detected;
    };
    std::vector<Outcome> seen;

    for (int advance = 1; advance <= 4; ++advance) {
        begin<Four, SinkFour>();
        const uint16_t lost_before = Four::lost();
        // Four whole detents' worth, arriving `advance` counts at a time.
        for (int i = 0; i < 16 / advance; ++i) {
            Knob::spin(static_cast<int8_t>(advance));
            tick<Four, SinkFour>();
        }
        seen.push_back({advance, total(SinkFour::got) * 4,
                        Four::lost() > lost_before});
        MESSAGE("  " << advance << " count(s) between samples: credited "
                     << total(SinkFour::got) * 4 << " of 16, losses seen: "
                     << int(Four::lost()));
    }

    // Keeping up: exact.
    CHECK(seen[0].missed == 1);
    CHECK(seen[0].reported == 16);
    CHECK(!seen[0].detected);

    // One state missed each time: not a legal step, so it is SEEN, and
    // the conservative policy credits nothing at all.
    CHECK(seen[1].detected);
    CHECK(seen[1].reported == 0);

    // Two missed: three states forward reads as one state BACKWARD, and
    // nothing can tell. This is the encoding's own limit, not the
    // decoder's, and it is why sampling fast enough is the whole
    // defence - at four counts a detent and a millisecond tick, a
    // 20-detent knob would have to be spun some twelve turns a second
    // to get here.
    CHECK(!seen[2].detected);
    CHECK(seen[2].reported < 0);

    // Four missed is a whole cycle: the contacts are back where they
    // were and nothing happened at all.
    CHECK(!seen[3].detected);
    CHECK(seen[3].reported == 0);
}

TEST_CASE("stop and start_every pace the decoder") {
    begin<Four, SinkFour>();
    CHECK(Four::running_every());
    Four::stop();
    CHECK(!Four::running_every());
    turn<Four, SinkFour>(8);            // ticks pass, nothing is sampled
    CHECK(SinkFour::got.empty());
    Four::start_every(1);
    CHECK(Four::running_every());
    turn<Four, SinkFour>(4);
    CHECK(total(SinkFour::got) == 1);
}
