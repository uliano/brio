// Host tests for util/power.hpp: the sleep-depth ladder, the vote round,
// standing restrictions, the deadline guard and the first-event-after-
// wake contract - driven through a real Kernel pack, so the AO contract
// is exercised and not simulated.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"

namespace {

using brio::HostPlatform;
using brio::PowerConfig;
using brio::PowerLock;
using brio::PrepareSleep;
using brio::SleepDepth;
using brio::SleepRequested;
using brio::SleepVote;
using brio::WakeReport;

using TE = brio::TimeEvents<HostPlatform>;

/**
 * A sleep site that only remembers. `deepest` models a target that does
 * NOT realize the whole ladder: anything below it is mapped to the
 * nearest shallower rung, which is the rule the concept states and the
 * AVR site (which has all four) cannot exercise.
 */
template <int Tag>
struct FakeSite {
    static inline uint16_t arms = 0;
    static inline uint16_t disarms = 0;
    static inline SleepDepth asked = SleepDepth::none;
    static inline SleepDepth state = SleepDepth::none;
    static inline SleepDepth deepest = SleepDepth::deep;
    static inline bool refuse = false;

    static bool arm(SleepDepth d) {
        ++arms;
        asked = d;
        if (refuse) {
            return false;
        }
        state = brio::shallower(d, deepest);
        return true;
    }

    static void disarm() {
        ++disarms;
        state = SleepDepth::none;
    }

    static SleepDepth armed() { return state; }

    static void reset() {
        arms = disarms = 0;
        asked = state = SleepDepth::none;
        deepest = SleepDepth::deep;
        refuse = false;
    }
};

static_assert(brio::SleepSite<FakeSite<0>>);

using Site = FakeSite<0>;

/// A stakeholder that votes as it is told and remembers being told.
template <int Tag>
struct Voter : brio::Fsm<Voter<Tag>, PrepareSleep, WakeReport> {
    using Base = brio::Fsm<Voter<Tag>, PrepareSleep, WakeReport>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static inline brio::EventQueue<Event, 4, HostPlatform> queue;

    static inline bool accept = true;
    static inline uint16_t asked = 0;
    static inline uint16_t woke = 0;
    static inline SleepDepth asked_depth = SleepDepth::none;
    static inline SleepDepth woke_from = SleepDepth::none;

    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    static void reset() {
        accept = true;
        asked = woke = 0;
        asked_depth = woke_from = SleepDepth::none;
        while (queue.pop().has_value()) {
        }
    }

    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::Exit) { return Base::handled(); },
            [](const PrepareSleep& p) {
                ++asked;
                asked_depth = p.depth;
                p.reply.send(SleepVote{accept});
                return Base::handled();
            },
            [](WakeReport w) {
                ++woke;
                woke_from = w.was;
                return Base::handled();
            });
    }
};

using VoterA = Voter<1>;
using VoterB = Voter<2>;

/// The one that asks for a sleep and hears the answer. It also owns the
/// time event the deadline guard looks at.
struct Asker : brio::Fsm<Asker, SleepVote> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<bool> votes;
    static inline brio::TimeEvent<HostPlatform, Asker, SleepVote> deadline{SleepVote{true}};

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    static void reset() {
        votes.clear();
        deadline.disarm();
        while (queue.pop().has_value()) {
        }
    }

    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](brio::Exit) { return handled(); },
            [](SleepVote v) { votes.push_back(v.ok); return handled(); });
    }

private:
    using Base = brio::Fsm<Asker, SleepVote>;
};

using Pm = brio::PowerManager<HostPlatform, Site, PowerConfig{}, VoterA, VoterB>;
using K = brio::Kernel<HostPlatform, Asker, VoterA, VoterB, Pm>;

/// A manager with no voters at all: the degenerate pack must still work.
using SoloSite = FakeSite<3>;
using Solo = brio::PowerManager<HostPlatform, SoloSite>;
using SoloK = brio::Kernel<HostPlatform, Solo>;

/// Serve events until every queue is empty (the loop, minus the sleep).
void pump() {
    for (int i = 0; i < 100 && K::step(); ++i) {
    }
}

void ask(SleepDepth d) {
    brio::post<Pm>(SleepRequested{d, brio::reply_to<Asker, SleepVote>()});
}

/// A fresh world: platform, time events, fakes, then the kernel's own
/// init - and only then are the site counters zeroed, so a test counts
/// what it caused and not what init did.
void setup() {
    HostPlatform::reset();
    TE::clear_all();
    Site::reset();
    VoterA::reset();
    VoterB::reset();
    Asker::reset();
    while (Pm::queue.pop().has_value()) {
    }
    K::init_all();
    Site::reset();
}

} // namespace

TEST_CASE("unanimity arms the requested depth") {
    setup();

    ask(SleepDepth::standby);
    pump();

    CHECK(VoterA::asked == 1);
    CHECK(VoterB::asked == 1);
    CHECK(VoterA::asked_depth == SleepDepth::standby);
    CHECK(VoterB::asked_depth == SleepDepth::standby);
    CHECK(Site::arms == 1);
    CHECK(Site::armed() == SleepDepth::standby);
    CHECK(Pm::armed_depth() == SleepDepth::standby);
    CHECK(Asker::votes == std::vector<bool>{true});
}

TEST_CASE("one refusal aborts the round and leaves nothing armed") {
    setup();
    VoterB::accept = false;

    ask(SleepDepth::standby);
    pump();

    CHECK(VoterA::asked == 1);          // everyone is still asked
    CHECK(VoterB::asked == 1);
    CHECK(Site::arms == 0);             // and the site is never armed
    CHECK(Site::disarms == 1);
    CHECK(Site::armed() == SleepDepth::none);
    CHECK(Pm::armed_depth() == SleepDepth::none);
    CHECK(Asker::votes == std::vector<bool>{false});
}

TEST_CASE("a site that refuses to arm ends the round the same way") {
    setup();
    Site::refuse = true;

    ask(SleepDepth::standby);
    pump();

    CHECK(Site::arms == 1);
    CHECK(Site::armed() == SleepDepth::none);
    CHECK(Asker::votes == std::vector<bool>{false});
}

TEST_CASE("a target without the deepest rung is armed at the nearest shallower one") {
    setup();
    Site::deepest = SleepDepth::standby;

    ask(SleepDepth::deep);
    pump();

    CHECK(Site::asked == SleepDepth::deep);            // asked for what was wanted
    CHECK(Site::armed() == SleepDepth::standby);       // took what it has
    CHECK(Pm::armed_depth() == SleepDepth::standby);   // and the manager records THAT
    CHECK(Asker::votes == std::vector<bool>{true});
}

TEST_CASE("a lock clamps the depth, releasing it restores the ladder") {
    setup();

    {
        PowerLock lock = Pm::restrict(SleepDepth::light);
        CHECK(static_cast<bool>(lock));
        CHECK(Pm::ceiling() == SleepDepth::light);

        ask(SleepDepth::deep);
        pump();
        CHECK(VoterA::asked_depth == SleepDepth::light);   // the voters see the clamp
        CHECK(Site::armed() == SleepDepth::light);
        CHECK(Asker::votes == std::vector<bool>{true});
    }
    CHECK(Pm::ceiling() == SleepDepth::deep);

    // The manager is armed: this request is the wake, and the round it
    // asks for is judged with the lock gone.
    ask(SleepDepth::deep);
    pump();
    CHECK(VoterA::woke == 1);
    CHECK(VoterA::woke_from == SleepDepth::light);
    CHECK(Site::armed() == SleepDepth::deep);
    CHECK(Asker::votes == std::vector<bool>{true, true});
}

TEST_CASE("the shallowest live lock wins, and a moved lock still releases") {
    setup();

    PowerLock light = Pm::restrict(SleepDepth::light);
    PowerLock standby = Pm::restrict(SleepDepth::standby);
    CHECK(Pm::ceiling() == SleepDepth::light);

    light.release();
    CHECK(Pm::ceiling() == SleepDepth::standby);
    light.release();                                  // idempotent
    CHECK(Pm::ceiling() == SleepDepth::standby);

    {
        PowerLock moved = std::move(standby);
        CHECK_FALSE(static_cast<bool>(standby));
        CHECK(Pm::ceiling() == SleepDepth::standby);  // the right moved, it did not die
    }
    CHECK(Pm::ceiling() == SleepDepth::deep);
}

TEST_CASE("a deadline nearer than min_deep_ticks refuses a deep request") {
    setup();

    Asker::deadline.arm(1);                     // sooner than the two-tick default
    ask(SleepDepth::deep);
    pump();

    CHECK(VoterA::asked == 0);                  // nobody is even asked
    CHECK(Site::arms == 0);
    CHECK(Asker::votes == std::vector<bool>{false});

    // Light is not deep: the guard does not apply to it.
    ask(SleepDepth::light);
    pump();
    CHECK(Site::armed() == SleepDepth::light);
    CHECK(Asker::votes == std::vector<bool>{false, true});
}

TEST_CASE("a distant deadline allows the deep request") {
    setup();

    Asker::deadline.arm(1000);
    ask(SleepDepth::deep);
    pump();

    CHECK(VoterA::asked == 1);
    CHECK(Site::armed() == SleepDepth::deep);
    CHECK(Asker::votes == std::vector<bool>{true});
    CHECK(TE::ticks_to_next() == std::optional<uint32_t>{1000});   // untouched
}

TEST_CASE("the first event after a wake disarms and reports") {
    setup();

    ask(SleepDepth::standby);
    pump();
    REQUIRE(Site::armed() == SleepDepth::standby);
    const uint16_t disarms_before = Site::disarms;

    // The wake path has nothing to ask for: "awake, no new request".
    ask(SleepDepth::none);
    pump();

    CHECK(VoterA::woke == 1);
    CHECK(VoterB::woke == 1);
    CHECK(VoterA::woke_from == SleepDepth::standby);
    CHECK(Site::disarms > disarms_before);
    CHECK(Site::armed() == SleepDepth::none);
    CHECK(Pm::armed_depth() == SleepDepth::none);
    CHECK(Asker::votes == std::vector<bool>{true, true});   // the no-op still replies ok
    CHECK(VoterA::asked == 1);                              // and asks nobody
}

TEST_CASE("a none request is a no-op that still replies ok") {
    setup();

    ask(SleepDepth::none);
    pump();

    CHECK(VoterA::asked == 0);
    CHECK(Site::arms == 0);
    CHECK(Site::armed() == SleepDepth::none);
    CHECK(Asker::votes == std::vector<bool>{true});
}

TEST_CASE("a second request while the votes are out is refused, not queued") {
    setup();

    ask(SleepDepth::standby);
    CHECK(K::step());                 // the manager posts the PrepareSleeps
    ask(SleepDepth::light);           // ...and now a rival request arrives
    pump();

    CHECK(Site::armed() == SleepDepth::standby);            // the first round won
    CHECK(Asker::votes == std::vector<bool>{false, true});  // rival refused, then ok
    CHECK(VoterA::asked == 1);
}

TEST_CASE("a manager with no voters arms on its own") {
    HostPlatform::reset();
    TE::clear_all();
    SoloSite::reset();
    while (Solo::queue.pop().has_value()) {
    }
    SoloK::init_all();
    SoloSite::reset();

    brio::post<Solo>(SleepRequested{SleepDepth::deep, {}});   // fire and forget
    for (int i = 0; i < 10 && SoloK::step(); ++i) {
    }
    CHECK(SoloSite::armed() == SleepDepth::deep);
    CHECK(Solo::armed_depth() == SleepDepth::deep);

    brio::post<Solo>(SleepRequested{SleepDepth::none, {}});
    for (int i = 0; i < 10 && SoloK::step(); ++i) {
    }
    CHECK(SoloSite::armed() == SleepDepth::none);
}
