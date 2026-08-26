// Host tests for util/bus_master.hpp: the arbiter itself (FIFO order,
// reject-when-full, the ReplyTo completion, both engine completion
// styles) and the completion-policy hook (pass-through by default, a
// retry ladder, the per-request attempt counter, and a retrying master
// voting NOT-OK on a PrepareSleep).
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/kernel.hpp"
#include "util/bus_master.hpp"
#include "util/power.hpp"

namespace {

using brio::BusAction;
using brio::BusDone;
using brio::bus_ok;
using brio::bus_rejected;
using brio::HostPlatform;
using brio::PrepareSleep;
using brio::SleepDepth;
using brio::SleepVote;
using brio::TransferDone;

constexpr uint8_t engine_fault = brio::bus_engine_status;      // 2
constexpr uint8_t engine_other = brio::bus_engine_status + 1;  // 3

/**
 * A scripted engine. `synchronous` makes start() complete inside itself
 * (the polled/degenerate style the contract allows); otherwise the
 * transfer stays in flight until the test posts a TransferDone.
 */
template <int Tag>
struct FakeBus {
    struct Request {
        uint8_t id;
        brio::ReplyTo<BusDone> reply;
    };
    static inline std::vector<uint8_t> started;
    static inline bool synchronous = false;

    static bool start(const Request& r) {
        started.push_back(r.id);
        return synchronous;
    }
    static void reset() {
        started.clear();
        synchronous = false;
    }
};

/// Retry every engine failure, up to `limit` times.
template <uint8_t limit>
struct RetryUpTo {
    static inline uint16_t asked = 0;
    static inline uint8_t last_attempt = 0xFF;
    static BusAction on_done(uint8_t status, uint8_t attempt) {
        ++asked;
        last_attempt = attempt;
        return (status != bus_ok && attempt < limit) ? BusAction::retry : BusAction::pass;
    }
    static void reset() {
        asked = 0;
        last_attempt = 0xFF;
    }
};

/// The requester: records every BusDone and every vote it collects.
template <int Tag>
struct Client : brio::Fsm<Client<Tag>, BusDone, SleepVote> {
    using Base = brio::Fsm<Client<Tag>, BusDone, SleepVote>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 16, HostPlatform> queue;
    static inline std::vector<uint8_t> done;
    static inline std::vector<bool> votes;
    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::Exit) { return Base::handled(); },
            [](BusDone d) { done.push_back(d.status); return Base::handled(); },
            [](SleepVote v) { votes.push_back(v.ok); return Base::handled(); });
    }
};

// ---- the three systems under test ------------------------------------------

using PlainEngine = FakeBus<0>;
using PlainClient = Client<0>;
using Plain = brio::BusMaster<PlainEngine, HostPlatform, 2>;   // depth 2 FIFO
using PlainSystem = brio::Kernel<HostPlatform, PlainClient, Plain>;

using RetryEngine = FakeBus<1>;
using RetryClient = Client<1>;
using Policy2 = RetryUpTo<2>;
using Retrying = brio::BusMaster<RetryEngine, HostPlatform, 4, Policy2>;
using RetrySystem = brio::Kernel<HostPlatform, RetryClient, Retrying>;

void pump(bool retrying) {
    for (uint16_t i = 0; i < 200; ++i) {
        if (retrying ? !RetrySystem::step() : !PlainSystem::step()) {
            return;
        }
    }
}

PlainEngine::Request plain_req(uint8_t id) {
    return PlainEngine::Request{id, brio::reply_to<PlainClient, BusDone>()};
}
RetryEngine::Request retry_req(uint8_t id) {
    return RetryEngine::Request{id, brio::reply_to<RetryClient, BusDone>()};
}

/**
 * Back to a quiet idle from whatever the previous case left behind.
 *
 * init_all() resets the arbiter fully (FIFO, tallies, active reply -
 * BusMaster::init() clears them like every other AO's init does), so
 * each case starts from power-on state.
 */
void reset_plain() {
    HostPlatform::reset();
    PlainSystem::init_all();
    PlainEngine::synchronous = true;
    brio::post<Plain>(plain_req(0));
    pump(false);
    PlainEngine::reset();
    PlainClient::done.clear();
    PlainClient::votes.clear();
    while (PlainClient::queue.pop().has_value()) {}
    while (Plain::queue.pop().has_value()) {}
    PlainSystem::init_all();
}

void reset_retry() {
    HostPlatform::reset();
    RetrySystem::init_all();
    RetryEngine::synchronous = true;
    brio::post<Retrying>(retry_req(0));
    pump(true);
    RetryEngine::reset();
    Policy2::reset();
    RetryClient::done.clear();
    RetryClient::votes.clear();
    while (RetryClient::queue.pop().has_value()) {}
    while (Retrying::queue.pop().has_value()) {}
    RetrySystem::init_all();
}

} // namespace

// ---- the arbiter ------------------------------------------------------------

TEST_CASE("one request goes out and its status comes back to the requester") {
    reset_plain();
    brio::post<Plain>(plain_req(1));
    pump(false);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1});
    CHECK(PlainClient::done.empty());                 // still in flight
    brio::post<Plain>(TransferDone{engine_fault});
    pump(false);
    CHECK(PlainClient::done == std::vector<uint8_t>{engine_fault});
}

TEST_CASE("requests are serialized in FIFO order, one on the wire at a time") {
    reset_plain();
    brio::post<Plain>(plain_req(1));
    brio::post<Plain>(plain_req(2));
    brio::post<Plain>(plain_req(3));
    pump(false);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1});   // 2 and 3 wait

    brio::post<Plain>(TransferDone{bus_ok});
    pump(false);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1, 2});
    brio::post<Plain>(TransferDone{bus_ok});
    pump(false);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1, 2, 3});
    brio::post<Plain>(TransferDone{engine_other});
    pump(false);
    CHECK(PlainClient::done == std::vector<uint8_t>{bus_ok, bus_ok, engine_other});
}

TEST_CASE("a full pending FIFO rejects immediately - never silently, never blocking") {
    reset_plain();
    const uint8_t rejected_before = Plain::rejected_count();
    brio::post<Plain>(plain_req(1));      // in flight
    brio::post<Plain>(plain_req(2));      // pending 1
    brio::post<Plain>(plain_req(3));      // pending 2 (depth is 2)
    brio::post<Plain>(plain_req(4));      // rejected
    pump(false);
    CHECK(PlainClient::done == std::vector<uint8_t>{bus_rejected});
    CHECK(Plain::rejected_count() == rejected_before + 1);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1});
}

TEST_CASE("a synchronous engine completes inside start() and drains the FIFO") {
    reset_plain();
    PlainEngine::synchronous = true;
    brio::post<Plain>(plain_req(1));
    brio::post<Plain>(plain_req(2));
    pump(false);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1, 2});
    CHECK(PlainClient::done == std::vector<uint8_t>{bus_ok, bus_ok});
    // Back to idle: a sleep round finds nothing in the way.
    brio::post<Plain>(PrepareSleep{SleepDepth::standby,
                                   brio::reply_to<PlainClient, SleepVote>()});
    pump(false);
    CHECK(PlainClient::votes == std::vector<bool>{true});
}

TEST_CASE("an idle bus votes yes, a busy one votes no") {
    reset_plain();
    brio::post<Plain>(PrepareSleep{SleepDepth::standby,
                                   brio::reply_to<PlainClient, SleepVote>()});
    pump(false);
    CHECK(PlainClient::votes == std::vector<bool>{true});

    brio::post<Plain>(plain_req(1));
    brio::post<Plain>(PrepareSleep{SleepDepth::standby,
                                   brio::reply_to<PlainClient, SleepVote>()});
    pump(false);
    CHECK(PlainClient::votes == std::vector<bool>{true, false});

    brio::post<Plain>(TransferDone{bus_ok});
    brio::post<Plain>(PrepareSleep{SleepDepth::standby,
                                   brio::reply_to<PlainClient, SleepVote>()});
    pump(false);
    CHECK(PlainClient::votes == std::vector<bool>{true, false, true});
}

// ---- the policy hook --------------------------------------------------------

TEST_CASE("the default policy is pass-through and costs no retry state") {
    reset_plain();
    static_assert(brio::BusPassThrough::never_retries);
    static_assert(brio::BusPassThrough::on_done(engine_fault, 0) == BusAction::pass);
    static_assert(!brio::bus_policy_may_retry<brio::BusPassThrough>());
    static_assert(brio::bus_policy_may_retry<Policy2>());

    brio::post<Plain>(plain_req(1));
    brio::post<Plain>(TransferDone{engine_fault});
    pump(false);
    CHECK(PlainEngine::started == std::vector<uint8_t>{1});   // started ONCE
    CHECK(PlainClient::done == std::vector<uint8_t>{engine_fault});
}

TEST_CASE("a retry policy re-starts the SAME request and passes the final status") {
    reset_retry();
    brio::post<Retrying>(retry_req(42));
    pump(true);
    CHECK(RetryEngine::started == std::vector<uint8_t>{42});
    CHECK(Retrying::attempt() == 0);

    brio::post<Retrying>(TransferDone{engine_fault});
    pump(true);
    CHECK(RetryEngine::started == std::vector<uint8_t>{42, 42});   // same id
    CHECK(Retrying::attempt() == 1);
    CHECK(RetryClient::done.empty());                              // said nothing yet

    brio::post<Retrying>(TransferDone{engine_fault});
    pump(true);
    CHECK(RetryEngine::started == std::vector<uint8_t>{42, 42, 42});
    CHECK(Retrying::attempt() == 2);
    CHECK(RetryClient::done.empty());

    // The limit is 2 retries: this completion passes, whatever it says.
    brio::post<Retrying>(TransferDone{engine_other});
    pump(true);
    CHECK(RetryEngine::started.size() == 3);
    CHECK(RetryClient::done == std::vector<uint8_t>{engine_other});
    CHECK(Retrying::attempt() == 0);
    CHECK(Policy2::asked == 3);
    CHECK(Policy2::last_attempt == 2);
}

TEST_CASE("a success on a retry is passed straight through") {
    reset_retry();
    brio::post<Retrying>(retry_req(7));
    brio::post<Retrying>(TransferDone{engine_fault});
    pump(true);
    CHECK(RetryEngine::started == std::vector<uint8_t>{7, 7});
    brio::post<Retrying>(TransferDone{bus_ok});
    pump(true);
    CHECK(RetryEngine::started.size() == 2);                 // no third try
    CHECK(RetryClient::done == std::vector<uint8_t>{bus_ok});
}

TEST_CASE("the attempt counter resets with each new request") {
    reset_retry();
    brio::post<Retrying>(retry_req(1));
    brio::post<Retrying>(TransferDone{engine_fault});     // retry 1
    brio::post<Retrying>(TransferDone{engine_fault});     // retry 2
    brio::post<Retrying>(TransferDone{engine_fault});     // limit: pass
    pump(true);
    CHECK(Retrying::attempt() == 0);
    CHECK(RetryClient::done == std::vector<uint8_t>{engine_fault});

    Policy2::reset();
    brio::post<Retrying>(retry_req(2));
    pump(true);
    CHECK(Retrying::attempt() == 0);
    brio::post<Retrying>(TransferDone{engine_fault});
    pump(true);
    CHECK(Policy2::last_attempt == 0);                   // counted from zero again
    CHECK(Retrying::attempt() == 1);
    CHECK(RetryEngine::started == std::vector<uint8_t>{1, 1, 1, 2, 2});
}

TEST_CASE("a retry that completes inside start() answers and drains the queue") {
    reset_retry();
    brio::post<Retrying>(retry_req(5));
    brio::post<Retrying>(retry_req(6));        // waits in the FIFO
    pump(true);
    RetryEngine::synchronous = true;           // the retry will finish at once
    brio::post<Retrying>(TransferDone{engine_fault});
    pump(true);
    CHECK(RetryEngine::started == std::vector<uint8_t>{5, 5, 6});
    CHECK(RetryClient::done == std::vector<uint8_t>{bus_ok, bus_ok});
    CHECK(Retrying::attempt() == 0);
}

TEST_CASE("a retrying master is BUSY and refuses a sleep round") {
    reset_retry();
    brio::post<Retrying>(retry_req(1));
    brio::post<Retrying>(TransferDone{engine_fault});    // -> retry in flight
    brio::post<Retrying>(PrepareSleep{SleepDepth::standby,
                                      brio::reply_to<RetryClient, SleepVote>()});
    pump(true);
    CHECK(RetryEngine::started == std::vector<uint8_t>{1, 1});
    CHECK(RetryClient::votes == std::vector<bool>{false});

    // Only when the ladder ends and the requester is answered does the
    // bus become quiet enough to vote yes.
    brio::post<Retrying>(TransferDone{engine_fault});
    brio::post<Retrying>(TransferDone{engine_fault});    // limit reached: passes
    brio::post<Retrying>(PrepareSleep{SleepDepth::standby,
                                      brio::reply_to<RetryClient, SleepVote>()});
    pump(true);
    CHECK(RetryClient::done.size() == 1);
    CHECK(RetryClient::votes == std::vector<bool>{false, true});
}
