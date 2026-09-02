// Host tests for util/bus_master.hpp: the arbiter itself (FIFO order,
// reject-when-full, the ReplyTo completion, both engine completion
// styles), the completion-policy hook (pass-through by default, a
// retry ladder, the per-request attempt counter, and a retrying master
// voting NOT-OK on a PrepareSleep), and the per-bus timeout (a transfer
// that never answers is recovered and answered bus_timeout; both halves
// of the race with the real completion staged deterministically on the
// virtual clock - the stale BusTimeout by queue order, the straggler
// TransferDone by posting it from inside recover(), which is exactly
// the window the real ISR has).
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

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

/**
 * The recoverable engine a TIMED master requires: FakeBus plus
 * recover(), with a hook run from INSIDE it - which is the one window
 * where a real engine's ISR could still post (recover() silences it),
 * so the straggler races are staged exactly where they live.
 */
template <int Tag>
struct RecoverableBus {
    struct Request {
        uint8_t id;
        brio::ReplyTo<BusDone> reply;
    };
    static inline std::vector<uint8_t> started;
    static inline bool synchronous = false;
    static inline int recovered = 0;
    static inline void (*on_recover)() = nullptr;

    static bool start(const Request& r) {
        started.push_back(r.id);
        return synchronous;
    }
    static void recover() {
        ++recovered;
        if (on_recover != nullptr) {
            on_recover();
        }
    }
    static void reset() {
        started.clear();
        synchronous = false;
        recovered = 0;
        on_recover = nullptr;
    }
};

// ---- the systems under test -------------------------------------------------

using PlainEngine = FakeBus<0>;
using PlainClient = Client<0>;
using Plain = brio::BusMaster<PlainEngine, HostPlatform, 2>;   // depth 2 FIFO
using PlainSystem = brio::Kernel<HostPlatform, PlainClient, Plain>;

using RetryEngine = FakeBus<1>;
using RetryClient = Client<1>;
using Policy2 = RetryUpTo<2>;
using Retrying = brio::BusMaster<RetryEngine, HostPlatform, 4, Policy2>;
using RetrySystem = brio::Kernel<HostPlatform, RetryClient, Retrying>;

// 50 ticks at the host's 1000 Hz: small enough to walk past in a test,
// and every deadline arithmetic below is relative to it.
constexpr uint32_t timeout_ticks = 50;

using TimedEngine = RecoverableBus<2>;
using TimedClient = Client<2>;
using Timed = brio::BusMaster<TimedEngine, HostPlatform, 2,
                              brio::BusPassThrough, timeout_ticks>;
using TimedSystem = brio::Kernel<HostPlatform, TimedClient, Timed>;

using TrEngine = RecoverableBus<3>;
using TrClient = Client<3>;
using TimedRetrying = brio::BusMaster<TrEngine, HostPlatform, 4, Policy2, timeout_ticks>;
using TrSystem = brio::Kernel<HostPlatform, TrClient, TimedRetrying>;

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

// ---- the timed systems' plumbing --------------------------------------------

/// The kernel loop's turn, faithfully: matured time events fire between
/// dispatches exactly as Kernel::run() has them.
template <typename System>
void pump_timed() {
    for (uint16_t i = 0; i < 200; ++i) {
        brio::TimeEvents<HostPlatform>::process();
        if (!System::step()) {
            return;
        }
    }
}

TimedEngine::Request timed_req(uint8_t id) {
    return TimedEngine::Request{id, brio::reply_to<TimedClient, BusDone>()};
}
TrEngine::Request tr_req(uint8_t id) {
    return TrEngine::Request{id, brio::reply_to<TrClient, BusDone>()};
}

void reset_timed() {
    HostPlatform::reset();
    brio::TimeEvents<HostPlatform>::clear_all();
    TimedEngine::reset();
    TimedClient::done.clear();
    TimedClient::votes.clear();
    while (TimedClient::queue.pop().has_value()) {}
    while (Timed::queue.pop().has_value()) {}
    TimedSystem::init_all();
}

void reset_timed_retry() {
    HostPlatform::reset();
    brio::TimeEvents<HostPlatform>::clear_all();
    TrEngine::reset();
    Policy2::reset();
    TrClient::done.clear();
    TrClient::votes.clear();
    while (TrClient::queue.pop().has_value()) {}
    while (TimedRetrying::queue.pop().has_value()) {}
    TrSystem::init_all();
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

// ---- the per-bus timeout ----------------------------------------------------

TEST_CASE("a transfer that never answers times out: recover, bus_timeout, bus alive") {
    reset_timed();
    brio::post<Timed>(timed_req(1));
    pump_timed<TimedSystem>();
    CHECK(TimedEngine::started == std::vector<uint8_t>{1});

    HostPlatform::ticks = timeout_ticks - 1;      // one tick short: nothing
    pump_timed<TimedSystem>();
    CHECK(TimedEngine::recovered == 0);
    CHECK(TimedClient::done.empty());

    HostPlatform::ticks = timeout_ticks + 1;      // matured
    pump_timed<TimedSystem>();
    CHECK(TimedEngine::recovered == 1);
    CHECK(TimedClient::done == std::vector<uint8_t>{brio::bus_timeout});
    CHECK(Timed::stale_events() == 0);

    // The bus survived its own diagnosis: the next request runs whole.
    brio::post<Timed>(timed_req(2));
    pump_timed<TimedSystem>();
    brio::post<Timed>(TransferDone{bus_ok});
    pump_timed<TimedSystem>();
    CHECK(TimedEngine::started == std::vector<uint8_t>{1, 2});
    CHECK(TimedClient::done == std::vector<uint8_t>{brio::bus_timeout, bus_ok});
}

TEST_CASE("a timeout overtaken by its own completion is dropped stale (idle)") {
    reset_timed();
    brio::post<Timed>(timed_req(1));
    pump_timed<TimedSystem>();

    // The completion enters the queue FIRST; the matured timer fires
    // behind it on the next turn. The completion wins, the bus goes
    // idle, and the stale BusTimeout must change nothing.
    HostPlatform::ticks = timeout_ticks + 1;
    brio::post<Timed>(TransferDone{engine_other});
    pump_timed<TimedSystem>();
    CHECK(TimedClient::done == std::vector<uint8_t>{engine_other});
    CHECK(TimedEngine::recovered == 0);
    CHECK(Timed::stale_events() == 1);
}

TEST_CASE("a stale timeout cannot kill the NEXT transfer: the sequence number") {
    reset_timed();
    brio::post<Timed>(timed_req(1));
    brio::post<Timed>(timed_req(2));              // waits in the FIFO
    pump_timed<TimedSystem>();
    CHECK(TimedEngine::started == std::vector<uint8_t>{1});

    // Request 1's completion and its matured timeout sit in the queue
    // together, completion first. Serving it starts request 2 (a NEW
    // wire transfer, a new sequence number); the stale BusTimeout then
    // lands in busy and must not touch it.
    HostPlatform::ticks = timeout_ticks + 1;
    brio::post<Timed>(TransferDone{bus_ok});
    pump_timed<TimedSystem>();
    CHECK(TimedEngine::started == std::vector<uint8_t>{1, 2});
    CHECK(TimedEngine::recovered == 0);
    CHECK(Timed::stale_events() == 1);
    CHECK(TimedClient::done == std::vector<uint8_t>{bus_ok});

    brio::post<Timed>(TransferDone{bus_ok});
    pump_timed<TimedSystem>();
    CHECK(TimedClient::done == std::vector<uint8_t>{bus_ok, bus_ok});
}

TEST_CASE("a straggler completion after a recovery is drained, never believed") {
    reset_timed();
    // The engine "completes" inside the very window recover() closes:
    // its TransferDone enters the queue BEFORE the BusFlushed marker,
    // which is the whole guarantee.
    TimedEngine::on_recover = [] { brio::post<Timed>(TransferDone{engine_fault}); };
    brio::post<Timed>(timed_req(1));
    brio::post<Timed>(timed_req(2));              // waits in the FIFO
    pump_timed<TimedSystem>();

    HostPlatform::ticks = timeout_ticks + 1;
    pump_timed<TimedSystem>();
    // Request 1 was answered bus_timeout; the straggler was drained in
    // the draining state (stale, counted) and request 2 started fresh -
    // NOT completed by the dead transfer's leftovers.
    CHECK(TimedEngine::recovered == 1);
    CHECK(Timed::stale_events() == 1);
    CHECK(TimedEngine::started == std::vector<uint8_t>{1, 2});
    CHECK(TimedClient::done == std::vector<uint8_t>{brio::bus_timeout});

    brio::post<Timed>(TransferDone{bus_ok});
    pump_timed<TimedSystem>();
    CHECK(TimedClient::done == std::vector<uint8_t>{brio::bus_timeout, bus_ok});
}

TEST_CASE("the draining dispatch holds requests and refuses a sleep round") {
    reset_timed();
    // Everything an ISR could throw into the recovery window: the
    // straggler, a new request, a sleep round - all queued BEFORE the
    // BusFlushed marker.
    TimedEngine::on_recover = [] {
        brio::post<Timed>(TransferDone{engine_fault});
        brio::post<Timed>(timed_req(3));
        brio::post<Timed>(PrepareSleep{SleepDepth::standby,
                                       brio::reply_to<TimedClient, SleepVote>()});
    };
    brio::post<Timed>(timed_req(1));
    pump_timed<TimedSystem>();

    HostPlatform::ticks = timeout_ticks + 1;
    pump_timed<TimedSystem>();
    // The straggler was dropped, request 3 waited its turn and went out
    // AFTER the flush, and the vote said no while the recovery stood.
    CHECK(Timed::stale_events() == 1);
    CHECK(TimedClient::votes == std::vector<bool>{false});
    CHECK(TimedEngine::started == std::vector<uint8_t>{1, 3});
    CHECK(TimedClient::done == std::vector<uint8_t>{brio::bus_timeout});

    brio::post<Timed>(TransferDone{bus_ok});
    pump_timed<TimedSystem>();
    CHECK(TimedClient::done == std::vector<uint8_t>{brio::bus_timeout, bus_ok});
}

TEST_CASE("a timeout is not a completion: the retry policy is never consulted") {
    reset_timed_retry();
    brio::post<TimedRetrying>(tr_req(1));
    pump_timed<TrSystem>();

    HostPlatform::ticks = timeout_ticks + 1;
    pump_timed<TrSystem>();
    CHECK(TrEngine::recovered == 1);
    CHECK(Policy2::asked == 0);                   // the engine never spoke
    CHECK(TrClient::done == std::vector<uint8_t>{brio::bus_timeout});
    CHECK(TimedRetrying::attempt() == 0);
}

TEST_CASE("a retry is a new wire transfer: its own timeout clock, its own seq") {
    reset_timed_retry();
    brio::post<TimedRetrying>(tr_req(9));
    pump_timed<TrSystem>();
    brio::post<TimedRetrying>(TransferDone{engine_fault});   // -> retry in flight
    pump_timed<TrSystem>();
    CHECK(TrEngine::started == std::vector<uint8_t>{9, 9});
    CHECK(TimedRetrying::attempt() == 1);

    // The RETRY wedges. Its timeout was re-armed at the retry's start
    // (tick 0 here), so it matures on the same arithmetic.
    HostPlatform::ticks = timeout_ticks + 1;
    pump_timed<TrSystem>();
    CHECK(TrEngine::recovered == 1);
    CHECK(TrClient::done == std::vector<uint8_t>{brio::bus_timeout});
    CHECK(Policy2::asked == 1);                   // the first failure only
    CHECK(TimedRetrying::attempt() == 0);
}
