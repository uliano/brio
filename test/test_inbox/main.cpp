// Host tests for util/inbox.hpp: two kernels on two host cores, the
// events that cross between them through Inbox/send, the doorbell's
// bell-first drain, the misposts a wrong-core post earns, and a reply
// that crosses back. Deterministic: the test plays both cores, one at a
// time, by setting HostPlatform::current_core.
// Run with: ctest --preset host -R inbox
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <vector>

#include "host/platform.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time_event.hpp"
#include "util/inbox.hpp"

namespace {

using P0 = brio::HostCore<0>;
using P1 = brio::HostCore<1>;
using Bell0 = brio::HostDoorbell<0>;
using Bell1 = brio::HostDoorbell<1>;

struct Ping { uint8_t n; };
struct Pong { uint8_t n; };
struct Done { uint8_t status; };
struct Request {
    uint8_t work;
    brio::ReplyTo<Done> reply;
};

// ---- core 1: the echo service, receives Ping, sends Pong back --------------
struct Echo : brio::Fsm<Echo, Ping, Request> {
    static inline brio::EventQueue<Event, 4, P1> queue;
    static constexpr uint8_t inbox_depth = 3;
    static inline std::vector<uint8_t> seen;
    static void init() { start(&only); }
    static Status only(const Event& e);
};

// ---- core 0: the requester, receives Pong and Done -------------------------
struct Origin : brio::Fsm<Origin, Pong, Done> {
    static inline brio::EventQueue<Event, 4, P0> queue;
    static inline std::vector<uint8_t> pongs;
    static inline std::vector<uint8_t> dones;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Pong p) { pongs.push_back(p.n); return handled(); },
            [](Done d) { dones.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); });
    }
};

Echo::Status Echo::only(const Event& e) {
    return brio::match(e,
        [](brio::Entry) { return handled(); },
        [](Ping p) { seen.push_back(p.n); brio::send<Origin>(Pong{p.n}); return handled(); },
        [](Request r) { r.reply.send(Done{static_cast<uint8_t>(r.work * 2)}); return handled(); },
        [](auto) { return unhandled(); });
}

using K0 = brio::Kernel<P0, Origin>;
using K1 = brio::Kernel<P1, Echo>;
using Drain0 = brio::Inboxes<Origin>;
using Drain1 = brio::Inboxes<Echo>;

static_assert(brio::Inbox<Echo>::capacity() == 3);
static_assert(brio::Inbox<Origin>::capacity() == 8);

void fresh() {
    brio::HostPlatform::reset();
    Bell0::reset();
    Bell1::reset();
    brio::Inbox<Echo>::clear();
    brio::Inbox<Origin>::clear();
    Echo::seen.clear();
    Origin::pongs.clear();
    Origin::dones.clear();
    while (Echo::queue.pop()) {}
    while (Origin::queue.pop()) {}
    brio::HostPlatform::current_core = 0;
    K0::init_all();
    brio::HostPlatform::current_core = 1;
    K1::init_all();
    brio::HostPlatform::current_core = 0;
}

/// Core 1 takes its bells and serves everything it has.
void run_core1() {
    brio::HostPlatform::current_core = 1;
    Drain1::isr();
    while (K1::step()) {}
    brio::HostPlatform::current_core = 0;
}
void run_core0() {
    brio::HostPlatform::current_core = 0;
    Drain0::isr();
    while (K0::step()) {}
}

}  // namespace

TEST_CASE("a send from core 0 lands in the inbox and rings core 1's bell") {
    fresh();
    brio::send<Echo>(Ping{7});
    CHECK(brio::Inbox<Echo>::pending());
    CHECK(Bell1::rings == 1);
    CHECK(Bell1::pending == 1);
    CHECK(Echo::queue.empty());          // not posted yet: the drain does that
    run_core1();
    CHECK(Bell1::pending == 0);
    CHECK(!brio::Inbox<Echo>::pending());
    REQUIRE(Echo::seen.size() == 1);
    CHECK(Echo::seen[0] == 7);
}

TEST_CASE("order is kept, and a full inbox drops and counts") {
    fresh();
    for (uint8_t i = 1; i <= 5; ++i) {
        brio::send<Echo>(Ping{i});
    }
    CHECK(brio::Inbox<Echo>::overflows() == 2);   // capacity 3
    CHECK(Bell1::rings == 3);                     // a dropped event rings nothing
    run_core1();
    REQUIRE(Echo::seen.size() == 3);
    CHECK(Echo::seen == std::vector<uint8_t>{1, 2, 3});
    // Room again: the next send is taken.
    brio::send<Echo>(Ping{9});
    CHECK(brio::Inbox<Echo>::overflows() == 2);
    run_core1();
    CHECK(Echo::seen.back() == 9);
}

TEST_CASE("the reply crosses back: ping-pong through both inboxes") {
    fresh();
    brio::send<Echo>(Ping{3});
    brio::send<Echo>(Ping{4});
    run_core1();                        // Echo sends two Pongs to Origin
    CHECK(Bell0::rings == 2);
    CHECK(Origin::pongs.empty());
    run_core0();
    CHECK(Origin::pongs == std::vector<uint8_t>{3, 4});
    CHECK(Bell0::pops == 1);
}

TEST_CASE("a request that crosses carries a reply that crosses") {
    fresh();
    brio::send<Echo>(Request{21, brio::send_reply_to<Origin, Done>()});
    run_core1();
    CHECK(brio::Inbox<Origin>::pending());
    run_core0();
    CHECK(Origin::dones == std::vector<uint8_t>{42});
}

TEST_CASE("a post to a queue of the other core is refused and counted") {
    fresh();
    brio::HostPlatform::current_core = 0;
    brio::post<Echo>(Ping{1});          // Echo's queue is core 1's
    CHECK(Echo::queue.empty());
    CHECK(Echo::queue.misposts() == 1);
    brio::HostPlatform::current_core = 1;
    brio::post<Echo>(Ping{2});          // from its own core: taken
    CHECK(!Echo::queue.empty());
    CHECK(Echo::queue.misposts() == 1);
    while (Echo::queue.pop()) {}
}

TEST_CASE("bells first: a send during the drain is not lost") {
    fresh();
    brio::send<Echo>(Ping{1});
    brio::HostPlatform::current_core = 1;
    Bell1::pop_all();                   // the isr's first act
    // ... a send lands here, between the pop and the drain:
    brio::send<Echo>(Ping{2});
    CHECK(Bell1::pending == 1);         // its bell stands: a second pass will run
    (void)brio::Inbox<Echo>::drain();   // this pass takes both anyway
    while (K1::step()) {}
    CHECK(Echo::seen == std::vector<uint8_t>{1, 2});
    brio::HostPlatform::current_core = 0;
}

TEST_CASE("the drain reports how many, and idle when empty") {
    fresh();
    brio::send<Echo>(Ping{5});
    brio::send<Echo>(Ping{6});
    brio::HostPlatform::current_core = 1;
    CHECK(!Drain1::idle());
    CHECK(brio::Inbox<Echo>::drain() == 2);
    CHECK(Drain1::idle());
    CHECK(brio::Inbox<Echo>::drain() == 0);
    while (K1::step()) {}
    brio::HostPlatform::current_core = 0;
}
