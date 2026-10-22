// Host tests for kernel/rubato.hpp, the preemptive kernel, over
// host/platform.hpp's RubatoHostPlatform: the ceiling and the two roads
// to a preempting dispatch (an interrupt's exit, a post from a dispatch),
// an object never nested in itself, the loans and the hold under
// preemption, the time events walked while dispatches arm and disarm,
// idle at the base only, overflow, the ring's producer guard, and
// Tenuto's own scenarios giving Tenuto's traces.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "host/platform.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "util/ring.hpp"

namespace {

using brio::HostPlatform;
using RP = brio::RubatoHostPlatform;
using Trace = std::vector<std::string>;

Trace trace;

struct Hit { uint8_t n; };
struct Note { uint8_t n; };   // the published notification

// The harness: a simulated interrupt handler. It runs `f` (which posts)
// with in_isr raised, lowers it, and when a post asked for the scheduler
// runs it at the exit: road 1.
template <typename F>
void isr(F f) {
    RP::in_isr = true;
    f();
    RP::in_isr = false;
    if (RP::pending && RP::scheduler != nullptr) {
        RP::pending = false;
        (void)RP::scheduler();
    }
}

// Three objects of one shape, over either platform, so the same scenario
// can run under Tenuto and under Rubato. A Hit is traced at its start
// with the ceiling the dispatch sees ("lo1:c2") and at its end
// ("lo1:end"); between the two the test's action runs. `active` counts
// the object's dispatches in progress: a nested one is a re-entry.
template <typename Plat, int id>
struct Node : brio::Fsm<Node<Plat, id>, Hit, Note> {
    using Base = brio::Fsm<Node<Plat, id>, Hit, Note>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;

    static inline brio::EventQueue<Event, 4, Plat> queue;
    static inline void (*action)(uint8_t) = nullptr;
    static inline uint8_t active = 0;
    static inline uint8_t most_active = 0;
    static inline uint32_t hits = 0;
    static inline std::vector<uint8_t> depths;   // the critical section's depth at each Hit

    static std::string name() {
        static const char* const names[] = {"hi", "mid", "lo"};
        return names[id];
    }

    static void init() {
        trace.push_back(name() + ":init");
        Base::start(&only);
    }
    static void dispatch(const Event& e) { Base::dispatch(e); }

    static void clear() {
        while (queue.pop().has_value()) {}
        action = nullptr;
        active = 0;
        most_active = 0;
        hits = 0;
        depths.clear();
    }

    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back(name() + ":entry"); return Base::handled(); },
            [](Hit h) {
                ++active;
                if (active > most_active) {
                    most_active = active;
                }
                ++hits;
                depths.push_back(Plat::CriticalSection::depth);
                const std::string tag = name() + std::to_string(h.n);
                if constexpr (brio::Preemptive<Plat>) {
                    trace.push_back(tag + ":c" + std::to_string(Plat::ceiling));
                } else {
                    trace.push_back(tag + ":c-");
                }
                if (action != nullptr) {
                    action(h.n);
                }
                trace.push_back(tag + ":end");
                --active;
                return Base::handled();
            },
            [](Note n) { trace.push_back(name() + ":note" + std::to_string(n.n)); return Base::handled(); },
            [](auto) { return Base::unhandled(); });
    }
};

using High = Node<RP, 0>;
using Mid = Node<RP, 1>;
using Low = Node<RP, 2>;
using K = brio::Rubato<RP, High, Mid, Low>;

// The scheduler the platform calls, counted.
uint32_t activations = 0;
bool counted_activate() {
    ++activations;
    return K::activate();
}

void reset() {
    trace.clear();
    RP::reset();
    brio::TimeEvents<RP>::clear_all();
    High::clear();
    Mid::clear();
    Low::clear();
    activations = 0;
    K::init_all();
    RP::scheduler = &counted_activate;
    trace.clear();
}

std::string ceiling_now() { return "c" + std::to_string(RP::ceiling); }

} // namespace

// ---------------------------------------------------------------------------
// a. The ceiling, road 1: an interrupt's post served at its exit
// ---------------------------------------------------------------------------

TEST_CASE("an interrupt inside Low posting to High: High runs to its end before Low resumes") {
    reset();
    Low::action = [](uint8_t) {
        RP::preempt_hook = [] { isr([] { brio::post<High>(Hit{7}); }); };
        RP::preempt_point();
        RP::preempt_hook = nullptr;
        trace.push_back("lo:resumed:" + ceiling_now());
    };
    isr([] { brio::post<Low>(Hit{1}); });
    CHECK(trace == Trace{"lo1:c2", "hi7:c0", "hi7:end", "lo:resumed:c2", "lo1:end"});
    CHECK(RP::ceiling == 255);
    CHECK_FALSE(RP::pending);
    CHECK(activations == 2);   // the outer interrupt's exit, the inner one's
}

TEST_CASE("an interrupt inside Mid posting to Low: Low is served after Mid returns") {
    reset();
    Mid::action = [](uint8_t) {
        RP::preempt_hook = [] { isr([] { brio::post<Low>(Hit{5}); }); };
        RP::preempt_point();
        RP::preempt_hook = nullptr;
        trace.push_back("mid:resumed:" + ceiling_now());
    };
    isr([] { brio::post<Mid>(Hit{1}); });
    CHECK(trace == Trace{"mid1:c1", "mid:resumed:c1", "mid1:end", "lo5:c2", "lo5:end"});
    CHECK(RP::ceiling == 255);
    CHECK(activations == 1);   // the post to Low asked for nothing: below the ceiling
}

// ---------------------------------------------------------------------------
// b. Never nested: an object is not dispatched inside its own dispatch
// ---------------------------------------------------------------------------

TEST_CASE("an interrupt posting to Low during Low's own dispatch: served after, never inside") {
    reset();
    Low::action = [](uint8_t n) {
        if (n == 1) {
            const uint32_t before = RP::requests;
            RP::preempt_hook = [] { isr([] { brio::post<Low>(Hit{9}); }); };
            RP::preempt_point();
            RP::preempt_hook = nullptr;
            CHECK(RP::requests == before);   // not above the ceiling: no request
        }
    };
    isr([] { brio::post<Low>(Hit{1}); });
    CHECK(trace == Trace{"lo1:c2", "lo1:end", "lo9:c2", "lo9:end"});
    CHECK(Low::most_active == 1);
    CHECK(RP::ceiling == 255);
}

TEST_CASE("a dispatch posting to itself waits for its own return") {
    reset();
    High::action = [](uint8_t n) {
        if (n > 0) {
            brio::post<High>(Hit{static_cast<uint8_t>(n - 1)});
            trace.push_back("hi:posted");
        }
    };
    isr([] { brio::post<High>(Hit{2}); });
    CHECK(trace == Trace{"hi2:c0", "hi:posted", "hi2:end", "hi1:c0", "hi:posted", "hi1:end",
                         "hi0:c0", "hi0:end"});
    CHECK(High::most_active == 1);
}

// ---------------------------------------------------------------------------
// c. The chain, road 2: a post from a dispatch runs the receiver inside it
// ---------------------------------------------------------------------------

TEST_CASE("Low posts to Mid, Mid posts to High: High inside Mid inside Low") {
    reset();
    Low::action = [](uint8_t) {
        brio::post<Mid>(Hit{2});
        trace.push_back("lo:after:" + ceiling_now());
    };
    Mid::action = [](uint8_t) {
        brio::post<High>(Hit{3});
        trace.push_back("mid:after:" + ceiling_now());
    };
    brio::post<Low>(Hit{1});   // from the base: Low's dispatch runs inside this post
    CHECK(trace == Trace{"lo1:c2", "mid2:c1", "hi3:c0", "hi3:end", "mid:after:c1", "mid2:end",
                         "lo:after:c2", "lo1:end"});
    CHECK(RP::ceiling == 255);
    CHECK(High::most_active == 1);
    CHECK(Mid::most_active == 1);
    CHECK(Low::most_active == 1);
    CHECK(activations == 3);   // one per post, each served inside it
}

TEST_CASE("a post from a dispatch never fills a higher queue: each one is served at once") {
    reset();
    Low::action = [](uint8_t) {
        for (uint8_t i = 0; i < 20; ++i) {   // five times High's slots
            brio::post<High>(Hit{i});
        }
    };
    brio::post<Low>(Hit{0});
    CHECK(High::hits == 20);
    CHECK(High::queue.overflows() == 0);
}

// ---------------------------------------------------------------------------
// d. Road 2 to a lower object
// ---------------------------------------------------------------------------

TEST_CASE("High posts to Low: served after High returns, by the scheduler's next pass") {
    reset();
    High::action = [](uint8_t) {
        brio::post<Low>(Hit{2});
        trace.push_back("hi:after:" + ceiling_now());
    };
    isr([] { brio::post<High>(Hit{1}); });
    CHECK(trace == Trace{"hi1:c0", "hi:after:c0", "hi1:end", "lo2:c2", "lo2:end"});
    CHECK(activations == 1);
    CHECK(RP::ceiling == 255);
}

// ---------------------------------------------------------------------------
// e. The loan under preemption
// ---------------------------------------------------------------------------

namespace {

struct Lent { brio::Borrowed<char, brio::Lease::dispatch> text; };
struct Go { uint8_t mode; };

// The borrower: records the lent text, then writes into it (a loan for
// the dispatch is a writable view: tokenizing in place is its point).
struct Borrower : brio::Fsm<Borrower, Lent> {
    static inline brio::EventQueue<Event, 2, RP> queue;
    static inline std::string seen;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Lent l) {
                seen = l.text.get();
                l.text.get()[0] = 'X';
                trace.push_back("bo:read " + seen);
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

// The lender: fills a static buffer, lends it, and then - mode 1 -
// reuses it at once, or - mode 2, the trap - reads it again.
struct Lender : brio::Fsm<Lender, Go> {
    static inline brio::EventQueue<Event, 2, RP> queue;
    using LendsTo = brio::Subscribers<Borrower>;
    static inline char buffer[8];
    static inline std::string read_after;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Go g) {
                std::strcpy(buffer, "abc");
                brio::post<Borrower>(Lent{brio::lend<brio::Lease::dispatch>(buffer)});
                trace.push_back("le:post returned");
                if (g.mode == 1) {
                    std::strcpy(buffer, "zzz");   // the lease is over: reuse
                } else {
                    read_after = buffer;          // the trap: what the borrower left
                }
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

using KL = brio::Rubato<RP, Borrower, Lender>;

void reset_loan() {
    trace.clear();
    RP::reset();
    while (Borrower::queue.pop().has_value()) {}
    while (Lender::queue.pop().has_value()) {}
    Borrower::seen.clear();
    Lender::read_after.clear();
    KL::init_all();
    RP::scheduler = &KL::activate;
}

} // namespace

TEST_CASE("a Lease::dispatch loan: the borrower runs inside the post, the lender reuses after it") {
    reset_loan();
    isr([] { brio::post<Lender>(Go{1}); });
    CHECK(trace == Trace{"bo:read abc", "le:post returned"});
    CHECK(Borrower::seen == "abc");                   // the lender's data
    CHECK(std::string(Lender::buffer) == "zzz");      // reused at once
    CHECK(Borrower::seen == "abc");                   // and the record undisturbed
}

TEST_CASE("the trap: a lender reading the lent buffer after the post sees the borrower's write") {
    reset_loan();
    isr([] { brio::post<Lender>(Go{2}); });
    CHECK(Borrower::seen == "abc");
    CHECK(Lender::read_after == "Xbc");   // under Tenuto it would read "abc": the borrower not yet run
}

// ---------------------------------------------------------------------------
// f. The hold under preemption
// ---------------------------------------------------------------------------

namespace {

struct Keep { uint8_t v; };
struct Drop {};

template <typename Plat>
struct Keeper : brio::Fsm<Keeper<Plat>, Keep, Drop, Hit> {
    using Base = brio::Fsm<Keeper<Plat>, Keep, Drop, Hit>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 2, Plat> queue;   // three slots
    static inline brio::Held<Event> kept{};
    static inline void (*after_keep)() = nullptr;
    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](Keep) {
                kept = queue.hold();
                trace.push_back("kp:keep");
                if (after_keep != nullptr) {
                    after_keep();
                }
                return Base::handled();
            },
            [](Drop) {
                trace.push_back("kp:drop" + std::to_string(std::get<Keep>(queue.at(kept)).v));
                queue.release(kept);
                kept = {};
                return Base::handled();
            },
            [](Hit h) {
                trace.push_back("kp:hit" + std::to_string(h.n) + ":held" +
                                std::to_string(std::get<Keep>(queue.at(kept)).v));
                if (after_keep != nullptr) {
                    after_keep();
                }
                return Base::handled();
            },
            [](auto) { return Base::unhandled(); });
    }
};

using Kp = Keeper<RP>;
using KH = brio::Rubato<RP, High, Kp>;

} // namespace

TEST_CASE("a held slot survives the preemptions of its holder and is released by it") {
    trace.clear();
    RP::reset();
    High::clear();
    while (Kp::queue.pop().has_value()) {}
    Kp::kept = {};
    KH::init_all();
    RP::scheduler = &KH::activate;
    trace.clear();

    // In each of the holder's dispatches an interrupt lands, posting to
    // High, which preempts the holder and posts it another Hit.
    Kp::after_keep = [] {
        RP::preempt_hook = [] { isr([] { brio::post<High>(Hit{1}); }); };
        RP::preempt_point();
        RP::preempt_hook = nullptr;
    };
    High::action = [](uint8_t) {
        static uint8_t round = 0;
        if (round++ < 2) {
            brio::post<Kp>(Hit{round});
        }
    };
    isr([] { brio::post<Kp>(Keep{7}); });
    CHECK(trace == Trace{"kp:keep", "hi1:c0", "hi1:end",
                         "kp:hit1:held7", "hi1:c0", "hi1:end",
                         "kp:hit2:held7", "hi1:c0", "hi1:end"});
    CHECK(Kp::kept);
    Kp::after_keep = nullptr;
    isr([] { brio::post<Kp>(Drop{}); });
    CHECK(trace.back() == "kp:drop7");
    CHECK(Kp::queue.overflows() == 0);
    // Every slot back: three pushes accepted inside an interrupt, the fourth refused.
    isr([] {
        for (uint8_t i = 0; i < 3; ++i) {
            brio::post<Kp>(Drop{});
        }
        CHECK(Kp::queue.overflows() == 0);
        brio::post<Kp>(Drop{});
        CHECK(Kp::queue.overflows() == 1);
        while (Kp::queue.pop()) {}   // drained by hand: nothing is held to drop
    });
}

// ---------------------------------------------------------------------------
// g. Time events: the list walked while dispatches arm and disarm
// ---------------------------------------------------------------------------

namespace {

using TE = brio::TimeEvents<RP>;
brio::TimeEvent<RP, High, Hit> to_high{Hit{10}};        // one-shot: its fire re-arranges the list
brio::TimeEvent<RP, Low, Hit> periodic_low{Hit{20}};     // periodic, re-armed by High's fire
brio::TimeEvent<RP, Mid, Hit> to_mid{Hit{30}};           // one-shot, expected to fire
brio::TimeEvent<RP, Low, Hit> doomed{Hit{40}};           // one-shot, disarmed before the walk reaches it
brio::TimeEvent<RP, Low, Hit> fresh{Hit{50}};            // armed during the walk, delay 0
brio::TimeEvent<RP, Mid, Hit> by_interrupt{Hit{60}};     // disarmed by an interrupt's dispatch inside the walk

uint32_t fired(uint8_t n) {
    uint32_t count = 0;
    for (const auto& s : trace) {
        if (s.find(std::to_string(n) + ":c") != std::string::npos) {
            ++count;
        }
    }
    return count;
}

} // namespace

TEST_CASE("arm() takes exactly one critical section, disarm() one") {
    reset();
    const uint32_t before = RP::CriticalSection::entries;
    to_mid.arm(5);
    CHECK(RP::CriticalSection::entries == before + 1);
    to_mid.arm(5);   // a re-arm too: the unlink inside the same guard
    CHECK(RP::CriticalSection::entries == before + 2);
    to_mid.disarm();
    CHECK(RP::CriticalSection::entries == before + 3);
    CHECK(RP::CriticalSection::depth == 0);
}

TEST_CASE("dispatches arming and disarming inside process(): no corruption, every due fire, once") {
    reset();
    // Armed last is walked first: the list is to_high, periodic_low,
    // by_interrupt, to_mid, doomed.
    doomed.arm(5);
    to_mid.arm(5);
    by_interrupt.arm(5);
    periodic_low.arm_every(5);
    to_high.arm(5);

    High::action = [](uint8_t n) {
        if (n == 10) {
            doomed.disarm();             // ahead of the walk
            periodic_low.arm_every(5);   // ahead of the walk, moved behind it: restarted
            fresh.arm(0);                // behind the walk: looked at next turn
        }
    };
    // After the first fire, an interrupt lands in the walk: its dispatch
    // (Mid, preempting the base) disarms a timer the walk has not reached.
    static uint32_t points = 0;
    points = 0;
    RP::preempt_hook = [] {
        if (points++ == 0) {
            isr([] { brio::post<Mid>(Hit{99}); });
        }
    };
    Mid::action = [](uint8_t n) {
        if (n == 99) {
            by_interrupt.disarm();
        }
    };

    RP::ticks = 5;
    TE::process();
    CHECK(fired(10) == 1);
    CHECK(fired(99) == 1);
    CHECK(fired(30) == 1);   // to_mid, reached after the re-arrangements
    CHECK(fired(20) == 0);   // restarted at tick 5: due at 10
    CHECK(fired(40) == 0);   // disarmed
    CHECK(fired(50) == 0);   // armed during the walk
    CHECK(fired(60) == 0);   // disarmed by the interrupt's dispatch
    CHECK_FALSE(doomed.armed());
    CHECK_FALSE(by_interrupt.armed());
    CHECK_FALSE(to_mid.armed());
    CHECK(fresh.armed());
    // Every fire ran with the guard dropped.
    for (const uint8_t d : High::depths) { CHECK(d == 0); }
    for (const uint8_t d : Mid::depths) { CHECK(d == 0); }
    CHECK(RP::CriticalSection::depth == 0);

    TE::process();           // the next turn, same tick: the one armed during the walk
    CHECK(fired(50) == 1);
    CHECK(fired(20) == 0);
    RP::ticks = 10;
    TE::process();
    CHECK(fired(20) == 1);
    CHECK(fired(10) == 1);
    CHECK(fired(30) == 1);
    for (const uint8_t d : Low::depths) { CHECK(d == 0); }
    // The list is whole: exactly periodic_low is armed, due at 15.
    CHECK(TE::next_deadline() == std::optional<uint32_t>{15});
    CHECK(TE::ticks_to_next() == std::optional<uint32_t>{5});
    periodic_low.disarm();
    fresh.disarm();
    CHECK_FALSE(TE::next_deadline().has_value());
}

TEST_CASE("a fire's own dispatch disarming or re-arming its timer leaves the walk whole") {
    reset();
    to_mid.arm(5);
    periodic_low.arm_every(5);   // walked first
    Low::action = [](uint8_t n) {
        if (n == 20) {
            periodic_low.disarm();   // the node the walk stands behind
        }
    };
    RP::ticks = 5;
    TE::process();
    CHECK(fired(20) == 1);
    CHECK(fired(30) == 1);       // the walk went on to the next node
    CHECK_FALSE(periodic_low.armed());

    reset();
    to_mid.arm(5);
    periodic_low.arm_every(5);
    Low::action = [](uint8_t n) {
        if (n == 20) {
            periodic_low.arm_every(3);   // re-armed from its own fire: moved to the head
        }
    };
    RP::ticks = 5;
    TE::process();
    CHECK(fired(20) == 1);       // once, not again at the head
    CHECK(fired(30) == 1);
    CHECK(TE::next_deadline() == std::optional<uint32_t>{8});
    periodic_low.disarm();
}

TEST_CASE("a one-shot re-armed at delay 0 by its own fire posts once per turn") {
    reset();
    Mid::action = [](uint8_t n) {
        if (n == 30) {
            to_mid.arm(0);
        }
    };
    to_mid.arm(0);
    TE::process();
    CHECK(fired(30) == 1);
    TE::process();
    CHECK(fired(30) == 2);
    Mid::action = nullptr;
    to_mid.disarm();
}

TEST_CASE("a periodic timer far behind posts once per process(), its deadlines kept") {
    reset();
    periodic_low.arm_every(5);
    RP::ticks = 17;              // three periods due: 5, 10, 15
    const uint32_t before = RP::CriticalSection::entries;
    TE::process();
    CHECK(fired(20) == 1);
    // One guard for the walk, one after the fire; the post's own guard beside them.
    CHECK(RP::CriticalSection::entries == before + 3);
    TE::process();
    TE::process();
    CHECK(fired(20) == 3);
    TE::process();
    CHECK(fired(20) == 3);       // 20 is not due at 17
    periodic_low.disarm();
}

// ---------------------------------------------------------------------------
// h. Idle at the base only
// ---------------------------------------------------------------------------

namespace {

void turn() {
    brio::TimeEvents<RP>::process();
    if (!K::activate()) {
        K::idle_if_empty();
    }
}

} // namespace

TEST_CASE("idle is called only from the base, and only when every queue is empty") {
    reset();
    RP::scheduler = nullptr;    // the request recorded, not yet served
    brio::post<Low>(Hit{1});
    brio::post<Mid>(Hit{2});
    K::idle_if_empty();         // something waits: no sleep
    CHECK(RP::idle_calls == 0);
    CHECK(RP::pending);

    RP::pending = false;
    RP::scheduler = &counted_activate;
    Low::action = [](uint8_t) {
        brio::post<High>(Hit{3});
        CHECK(RP::idle_calls == 0);
    };
    High::action = [](uint8_t) { CHECK(RP::idle_calls == 0); };
    turn();                     // serves all, does not sleep
    CHECK(RP::idle_calls == 0);
    CHECK(trace == Trace{"mid2:c1", "mid2:end", "lo1:c2", "hi3:c0", "hi3:end", "lo1:end"});
    turn();                     // nothing left: the base sleeps
    CHECK(RP::idle_calls == 1);
    CHECK(RP::CriticalSection::depth == 0);
}

// ---------------------------------------------------------------------------
// i. Overflow
// ---------------------------------------------------------------------------

TEST_CASE("a full queue drops and counts; a dropped post serves nothing") {
    reset();
    isr([] {
        for (uint8_t i = 0; i < 6; ++i) {   // an idle queue of depth 4 takes 5
            brio::post<High>(Hit{i});
        }
        CHECK(High::queue.overflows() == 1);
    });
    CHECK(High::hits == 5);
    CHECK(activations == 1);                // one scheduler run, at the exit
    CHECK(RP::requests == 6);

    // From its own dispatch: four waiting places, the fifth post dropped,
    // and none of them asks for the scheduler.
    High::action = [](uint8_t n) {
        if (n == 100) {
            const uint32_t before = RP::requests;
            for (uint8_t i = 0; i < 5; ++i) {
                brio::post<High>(Hit{i});
            }
            CHECK(RP::requests == before);
            CHECK(High::queue.overflows() == 2);
        }
    };
    High::hits = 0;
    isr([] { brio::post<High>(Hit{100}); });
    CHECK(High::hits == 5);                 // the 100 and the four kept
    CHECK(High::most_active == 1);
}

// ---------------------------------------------------------------------------
// j. The ring's producer guard
// ---------------------------------------------------------------------------

TEST_CASE("a Ring's producer verbs take the critical section on a preemptive platform only") {
    RP::reset();
    {
        brio::Ring<uint8_t, 16, RP> ring;
        static_assert(brio::Ring<uint8_t, 16, RP>::lock_free);
        static_assert(!brio::Ring<uint8_t, 16, RP>::producer_lock_free);
        uint32_t before = RP::CriticalSection::entries;
        CHECK(ring.push(1));
        CHECK(RP::CriticalSection::entries == before + 1);
        before = RP::CriticalSection::entries;
        const auto run = ring.write_span();
        CHECK(RP::CriticalSection::entries == before + 1);
        run[0] = 2;
        ring.publish(1);
        CHECK(RP::CriticalSection::entries == before + 2);
        before = RP::CriticalSection::entries;
        CHECK(ring.pop() == std::optional<uint8_t>{1});   // the consumer stays bare
        CHECK(ring.read_span().size() == 1);
        ring.consume(1);
        CHECK(ring.count() == 0);
        CHECK(RP::CriticalSection::entries == before);
    }
    HostPlatform::reset();
    {
        brio::Ring<uint8_t, 16, HostPlatform> ring;
        static_assert(brio::Ring<uint8_t, 16, HostPlatform>::producer_lock_free);
        const uint32_t before = HostPlatform::CriticalSection::entries;
        CHECK(ring.push(1));
        const auto run = ring.write_span();
        run[0] = 2;
        ring.publish(1);
        CHECK(ring.pop() == std::optional<uint8_t>{1});
        CHECK(HostPlatform::CriticalSection::entries == before);
    }
}

// ---------------------------------------------------------------------------
// k. Tenuto's scenarios under Rubato, no interrupt fired: Tenuto's traces
// ---------------------------------------------------------------------------
//
// The same three objects over HostPlatform under Tenuto and over
// RubatoHostPlatform under Rubato. A post from a test body at the base is
// not the same act under the two kernels - under Rubato it serves the
// receiver at once - so each scenario feeds its events with Rubato's
// scheduler not connected (the request recorded and left), then cranks the
// kernel: Tenuto's step() until it finds nothing, Rubato's activate().
// What the dispatches post themselves goes through the connected scheduler.

namespace {

using TH = Node<HostPlatform, 0>;
using TM = Node<HostPlatform, 1>;
using TL = Node<HostPlatform, 2>;
using KT = brio::Tenuto<HostPlatform, TH, TM, TL>;
using Tk = Keeper<HostPlatform>;
using KTk = brio::Tenuto<HostPlatform, TH, Tk>;

struct UnderTenuto {
    template <int id> using N = Node<HostPlatform, id>;
    using Keep_ = Tk;
    static void init() {
        trace.clear();
        HostPlatform::reset();
        TH::clear();
        TM::clear();
        TL::clear();
        while (Tk::queue.pop().has_value()) {}
        Tk::kept = {};
        Tk::after_keep = nullptr;
        KT::init_all();
    }
    static void init_keeper() { KTk::init_all(); }
    static void feed_begin() {}
    static void feed_end() {}
    static void crank() { while (KT::step()) {} }
    static void crank_keeper() { while (KTk::step()) {} }
};

struct UnderRubato {
    template <int id> using N = Node<RP, id>;
    using Keep_ = Kp;
    static void init() {
        trace.clear();
        RP::reset();
        High::clear();
        Mid::clear();
        Low::clear();
        while (Kp::queue.pop().has_value()) {}
        Kp::kept = {};
        Kp::after_keep = nullptr;
        K::init_all();
    }
    static void init_keeper() { KH::init_all(); }
    static void feed_begin() { RP::scheduler = nullptr; }
    static void feed_end() {
        RP::scheduler = &K::activate;
        RP::pending = false;
    }
    static void crank() { (void)K::activate(); }
    static void crank_keeper() {
        RP::scheduler = &KH::activate;
        (void)KH::activate();
    }
};

/// A trace with every recorded ceiling (":c" and digits, at the end of an
/// entry) written as Tenuto's ":c-".
Trace without_ceilings(Trace t) {
    for (auto& s : t) {
        const auto at = s.rfind(":c");
        if (at != std::string::npos && at + 2 < s.size() &&
            s.find_first_not_of("0123456789", at + 2) == std::string::npos) {
            s = s.substr(0, at) + ":c-";
        }
    }
    return t;
}

template <typename U>
Trace scenario_init() {
    U::init();
    return trace;
}

template <typename U>
Trace scenario_priority() {
    U::init();
    trace.clear();
    U::feed_begin();
    brio::post<typename U::template N<2>>(Hit{1});
    brio::post<typename U::template N<0>>(Hit{2});
    brio::post<typename U::template N<1>>(Hit{3});
    brio::post<typename U::template N<0>>(Hit{4});
    U::feed_end();
    U::crank();
    return trace;
}

template <typename U>
Trace scenario_self_post() {
    U::init();
    trace.clear();
    using L = typename U::template N<2>;
    L::action = [](uint8_t n) {
        if (n > 0) {
            brio::post<L>(Hit{static_cast<uint8_t>(n - 1)});
        }
    };
    U::feed_begin();
    brio::post<L>(Hit{3});
    U::feed_end();
    U::crank();
    L::action = nullptr;
    return trace;
}

template <typename U>
Trace scenario_publish() {
    U::init();
    trace.clear();
    U::feed_begin();
    brio::publish(brio::Subscribers<typename U::template N<0>, typename U::template N<2>>{},
                  Note{5});
    U::feed_end();
    U::crank();
    return trace;
}

template <typename U>
Trace scenario_hold() {
    U::init();
    U::init_keeper();
    trace.clear();
    using Kq = typename U::Keep_;
    U::feed_begin();
    brio::post<Kq>(Keep{7});
    brio::post<Kq>(Hit{1});
    brio::post<Kq>(Hit{2});
    U::feed_end();
    U::crank_keeper();
    U::feed_begin();
    brio::post<Kq>(Drop{});
    U::feed_end();
    U::crank_keeper();
    return trace;
}

} // namespace

TEST_CASE("Tenuto's scenarios run under Rubato with no interrupt give Tenuto's traces") {
    const Trace t_init = scenario_init<UnderTenuto>();
    CHECK(t_init == Trace{"hi:init", "hi:entry", "mid:init", "mid:entry", "lo:init", "lo:entry"});
    CHECK(scenario_init<UnderRubato>() == t_init);

    const Trace t_priority = scenario_priority<UnderTenuto>();
    CHECK(t_priority == Trace{"hi2:c-", "hi2:end", "hi4:c-", "hi4:end", "mid3:c-", "mid3:end",
                              "lo1:c-", "lo1:end"});
    // The one difference allowed is the ceiling a Rubato dispatch records.
    CHECK(without_ceilings(scenario_priority<UnderRubato>()) == t_priority);

    const Trace t_self = scenario_self_post<UnderTenuto>();
    CHECK(t_self == Trace{"lo3:c-", "lo3:end", "lo2:c-", "lo2:end", "lo1:c-", "lo1:end",
                          "lo0:c-", "lo0:end"});
    CHECK(without_ceilings(scenario_self_post<UnderRubato>()) == t_self);

    const Trace t_publish = scenario_publish<UnderTenuto>();
    CHECK(t_publish == Trace{"hi:note5", "lo:note5"});
    CHECK(scenario_publish<UnderRubato>() == t_publish);

    const Trace t_hold = scenario_hold<UnderTenuto>();
    CHECK(t_hold == Trace{"kp:keep", "kp:hit1:held7", "kp:hit2:held7", "kp:drop7"});
    CHECK(scenario_hold<UnderRubato>() == t_hold);
    CHECK(RP::ceiling == 255);
}
