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
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <string>

#include "host/sim_input.hpp"
#include "host/sim_panel.hpp"
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

// ---------------------------------------------------------------------
// The panel a viewer writes and the program reads.
// ---------------------------------------------------------------------

TEST_CASE("a shaft set by absolute count walks the same states as a turn") {
    // The viewer owns the count; the pads must land where stepping would
    // have put them, or a snapshot and a turn would disagree.
    Knob::reset();
    for (int32_t n = 0; n < 16; ++n) {
        Knob::reset();
        for (int32_t i = 0; i < n; ++i) {
            Knob::step(1);
        }
        const uint8_t stepped = Knob::state();
        Knob::set_shaft(n);
        INFO("count " << n);
        CHECK(Knob::state() == stepped);
    }
    // and backwards, where the count goes negative
    Knob::reset();
    Knob::step(-1);
    const uint8_t back = Knob::state();
    Knob::set_shaft(-1);
    CHECK(Knob::state() == back);
}

TEST_CASE("a panel reads as nothing pressed until a viewer writes") {
    brio::SimPanel panel("qtest");
    CHECK(panel.seq() == 0);
    for (uint8_t i = 0; i < brio::sim_panel_buttons; ++i) {
        CHECK(!panel.pressed(i));
    }
    for (uint8_t i = 0; i < brio::sim_panel_shafts; ++i) {
        CHECK(panel.shaft(i) == 0);
    }
    // Out of range is released and nought, not a read past the end.
    CHECK(!panel.pressed(brio::sim_panel_buttons));
    CHECK(panel.shaft(brio::sim_panel_shafts) == 0);
    CHECK(panel.boot_id() != 0);
}

TEST_CASE("what a viewer writes is what the program reads") {
    brio::SimPanel panel("qtest2");

    // The viewer's end: attach by NAME, read-write, and set the world.
    const int fd = ::shm_open(panel.name().c_str(), O_RDWR, 0);
    REQUIRE(fd >= 0);
    void* p = ::mmap(nullptr, brio::sim_panel_bytes, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, 0);
    REQUIRE(p != MAP_FAILED);
    ::close(fd);
    auto* w = static_cast<brio::SimPanelState*>(p);
    CHECK(std::string(w->magic, 4) == "BRIP");
    CHECK(w->buttons == brio::sim_panel_buttons);

    w->pressed[2] = 1;
    w->shaft[1] = -7;
    ++w->seq;

    CHECK(panel.pressed(2));
    CHECK(!panel.pressed(1));
    CHECK(panel.shaft(1) == -7);
    CHECK(panel.seq() == 1);

    // And the program mirrors it into its devices, the way a board file
    // would in the idle path.
    brio::SimButton<3>::set(panel.pressed(2));
    Knob::set_shaft(panel.shaft(1));
    CHECK(brio::SimButton<3>::read());
    ::munmap(p, brio::sim_panel_bytes);
}

TEST_CASE("a panel name too long is refused rather than truncated") {
    CHECK_THROWS_AS(brio::SimPanel(std::string(40, 'x')), std::runtime_error);
}

TEST_CASE("a control carries the name its board file gave it") {
    brio::SimPanel panel("qtest3");
    // Unnamed reads as empty rather than as rubbish.
    CHECK(panel.button_name(0).empty());
    CHECK(panel.shaft_name(0).empty());

    panel.name_button(0, "Menu");
    panel.name_shaft(1, "Volume");
    CHECK(panel.button_name(0) == "Menu");
    CHECK(panel.shaft_name(1) == "Volume");
    CHECK(panel.button_name(1).empty());

    // A label is a label: too long is cut, not refused.
    panel.name_button(2, "a name far longer than the field allows");
    CHECK(panel.button_name(2).size() == brio::sim_panel_name_max);
    CHECK(panel.button_name(2) == "a name far longe");

    // Renaming clears what was there, rather than leaving a tail.
    panel.name_shaft(1, "Gain");
    CHECK(panel.shaft_name(1) == "Gain");

    // Out of range answers without reading past the arrays.
    CHECK(panel.button_name(brio::sim_panel_buttons).empty());
    CHECK(panel.shaft_name(brio::sim_panel_shafts).empty());
    panel.name_button(brio::sim_panel_buttons, "nowhere");
    panel.name_shaft(brio::sim_panel_shafts, "nowhere");
}

TEST_CASE("a knob declares its switch and its detent, rather than being guessed") {
    brio::SimPanel panel("qtest4");
    // Unsaid, a knob has no switch under it - a viewer must not invent one.
    CHECK(panel.shaft_switch(0) == brio::sim_panel_no_switch);
    CHECK(panel.shaft_detent(0) == 4); // the common part, until told otherwise

    panel.name_shaft(0, "Adjust", 1, 2);
    CHECK(panel.shaft_name(0) == "Adjust");
    CHECK(panel.shaft_switch(0) == 1);
    CHECK(panel.shaft_detent(0) == 2);

    // Naming without saying takes it back to having neither, which is
    // honest: a control is described by one call, not by accumulation.
    panel.name_shaft(0, "Coarse");
    CHECK(panel.shaft_switch(0) == brio::sim_panel_no_switch);
    CHECK(panel.shaft_detent(0) == 4);

    CHECK(panel.shaft_switch(brio::sim_panel_shafts) == brio::sim_panel_no_switch);
}
