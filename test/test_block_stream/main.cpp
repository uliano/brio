// Host tests for util/block_stream.hpp: the BlockSource/BlockPlayer
// concepts and BlockRelay - filled blocks handed to subscribers as
// Lease::dispatch loans, returned to their source on the relay's next
// dispatch, one block per source per dispatch, self-post whenever a
// loan is outstanding, and the accounting passed through rather than
// duplicated.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "host/platform_host.hpp"
#include "util/block_stream.hpp"

namespace {

using brio::BlockDone;
using brio::BlockReady;
using brio::HostPlatform;

// A scripted source honest to the ping-pong contract the SAM C21 engine
// set: two buffers, at most one being filled and at most two pending,
// an overrun SKIPPING the fill rather than tearing the held buffer, and
// release() restarting a stalled stream. The `id` makes two sources two
// (the MeterLatch convention).
template <uint8_t id>
struct SimSource {
    using element = uint16_t;
    static constexpr uint16_t block_len = 4;

    static inline std::array<std::array<uint16_t, block_len>, 2> buf{};
    static inline uint8_t fill = 0;
    static inline uint8_t drain = 0;
    static inline uint8_t pending = 0;
    static inline bool stalled_now = false;
    static inline uint32_t laps_n = 0;
    static inline uint32_t overruns_n = 0;
    static inline int releases = 0;
    static inline int restarts = 0;

    static volatile uint16_t* ready() {
        return pending != 0 ? buf[drain].data() : nullptr;
    }
    static uint16_t ready_length() { return pending != 0 ? block_len : 0; }
    static bool release() {
        if (pending == 0) {
            return false;
        }
        --pending;
        drain ^= 1;
        ++releases;
        if (stalled_now) {
            stalled_now = false;
            ++restarts;
        }
        return true;
    }
    static uint32_t laps() { return laps_n; }
    static uint32_t overruns() { return overruns_n; }
    static bool stalled() { return stalled_now; }

    // ---- test side ----------------------------------------------------
    /// The stimulus: one block just filled, values seeded from `base`.
    /// Mirrors the engine: a completion with both buffers already held
    /// is an overrun and a stall - the buffer is NOT overwritten.
    static void complete_block(uint16_t base) {
        if (pending >= 2) {
            ++overruns_n;
            stalled_now = true;
            return;
        }
        for (uint16_t k = 0; k < block_len; ++k) {
            buf[fill][k] = static_cast<uint16_t>(base + k);
        }
        fill ^= 1;
        ++pending;
        ++laps_n;
    }
    static void reset() {
        buf = {};
        fill = drain = pending = 0;
        stalled_now = false;
        laps_n = overruns_n = 0;
        releases = restarts = 0;
    }
};

using SrcA = SimSource<0>;
using SrcB = SimSource<1>;

static_assert(brio::BlockSource<SrcA>);
static_assert(brio::BlockSource<SrcB>);

// The playback half of the vocabulary, concept-checked against a
// minimal fake (the SAM engines are concept-checked in the family
// fixture, where they are in scope).
struct FakePlayer {
    using element = uint16_t;
    static uint32_t laps() { return 0; }
    static uint32_t faults() { return 0; }
    static bool running() { return false; }
    static void stop() {}
};
static_assert(brio::BlockPlayer<FakePlayer>);

// The consumer: copies every block OUT during its dispatch (the loan's
// whole window) and remembers the raw pointer so a test can prove the
// storage was reused only after release.
struct Consumer : brio::Fsm<Consumer, BlockReady<uint16_t>> {
    using Base = brio::Fsm<Consumer, BlockReady<uint16_t>>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline brio::EventQueue<Event, 8, HostPlatform> queue;

    struct Got {
        uint8_t source;
        std::vector<uint16_t> data;
        const volatile uint16_t* raw;
    };
    static inline std::vector<Got> got;

    static void init() { Base::start(&only); }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return Base::handled(); },
            [](brio::Exit) { return Base::handled(); },
            [](BlockReady<uint16_t> b) {
                REQUIRE(static_cast<bool>(b.data));
                Got g{b.source, {}, b.data.get()};
                for (uint16_t k = 0; k < b.length; ++k) {
                    // A volatile load, then a plain value: reading a
                    // block IS copying it out of memory the compiler
                    // cannot see written.
                    const uint16_t v = b.data.get()[k];
                    g.data.push_back(v);
                }
                got.push_back(std::move(g));
                return Base::handled();
            });
    }
};

using Relay = brio::BlockRelay<HostPlatform, brio::Subscribers<Consumer>,
                               SrcA, SrcB>;

/// The kernel loop's essence, pack order Consumer THEN Relay: the
/// borrower is served before the lender runs again, which is the
/// contract the release timing rests on.
void run_scheduler() {
    for (;;) {
        if (auto e = Consumer::queue.pop()) {
            Consumer::dispatch(*e);
        } else if (auto r = Relay::queue.pop()) {
            Relay::dispatch(*r);
        } else {
            break;
        }
    }
}

void fresh_start() {
    SrcA::reset();
    SrcB::reset();
    Consumer::got.clear();
    while (Relay::queue.pop()) {
    }
    while (Consumer::queue.pop()) {
    }
    Consumer::init();
    Relay::init();
}

TEST_CASE("one block: lent whole, copied intact, returned after the "
          "consumer's dispatch") {
    fresh_start();
    SrcA::complete_block(100);
    brio::post<Relay>(BlockDone{});
    run_scheduler();

    REQUIRE(Consumer::got.size() == 1);
    CHECK(Consumer::got[0].source == 0);
    CHECK(Consumer::got[0].data ==
          std::vector<uint16_t>{100, 101, 102, 103});
    CHECK(Relay::published() == 1);
    // The loan came back: the buffer was released (by the relay's
    // self-posted second dispatch), and only AFTER the consumer ran.
    CHECK(SrcA::releases == 1);
    CHECK(SrcA::pending == 0);
}

TEST_CASE("the storage is reused only after release: the copy taken "
          "during the dispatch survives the source moving on") {
    fresh_start();
    SrcA::complete_block(100);
    brio::post<Relay>(BlockDone{});
    run_scheduler();
    // The source refills the SAME buffer (two completions advance the
    // fill index past both): the consumer's copy must not change.
    SrcA::complete_block(500);
    SrcA::complete_block(900);
    CHECK(Consumer::got[0].data ==
          std::vector<uint16_t>{100, 101, 102, 103});
}

TEST_CASE("two sources in one wakeup: both lent, list order is the "
          "source index") {
    fresh_start();
    SrcA::complete_block(10);
    SrcB::complete_block(20);
    brio::post<Relay>(BlockDone{});
    run_scheduler();

    REQUIRE(Consumer::got.size() == 2);
    CHECK(Consumer::got[0].source == 0);
    CHECK(Consumer::got[1].source == 1);
    CHECK(Consumer::got[0].data[0] == 10);
    CHECK(Consumer::got[1].data[0] == 20);
    CHECK(SrcA::releases == 1);
    CHECK(SrcB::releases == 1);
}

TEST_CASE("a stall drains in order and release restarts the stream") {
    fresh_start();
    SrcA::complete_block(100);
    SrcA::complete_block(200);   // both buffers pending
    SrcA::complete_block(300);   // nowhere to go: overrun + stall
    brio::post<Relay>(BlockDone{});
    run_scheduler();

    // Both held blocks arrive, once each, oldest first; the overrun is
    // the SOURCE's count and the skipped block's samples exist nowhere.
    REQUIRE(Consumer::got.size() == 2);
    CHECK(Consumer::got[0].data[0] == 100);
    CHECK(Consumer::got[1].data[0] == 200);
    CHECK(SrcA::overruns_n == 1);
    CHECK(SrcA::restarts == 1);   // the first release lifted the stall
    CHECK(SrcA::pending == 0);
    CHECK(Relay::overruns(0) == 1);
}

TEST_CASE("coalesced or extra wakeups neither lose nor duplicate a "
          "block") {
    fresh_start();
    SrcA::complete_block(1);
    // Three wakeups for one block (the ISR glue racing the self-post is
    // the real-world shape): the scan-everything dispatch makes the
    // extras no-ops.
    brio::post<Relay>(BlockDone{});
    brio::post<Relay>(BlockDone{});
    brio::post<Relay>(BlockDone{});
    run_scheduler();
    CHECK(Consumer::got.size() == 1);
    CHECK(SrcA::releases == 1);

    // And a steady stream through repeated wakeups: every block exactly
    // once, in order.
    for (uint16_t n = 0; n < 5; ++n) {
        SrcA::complete_block(static_cast<uint16_t>(1000 + 100 * n));
        brio::post<Relay>(BlockDone{});
        run_scheduler();
    }
    REQUIRE(Consumer::got.size() == 6);
    for (uint16_t n = 0; n < 5; ++n) {
        CHECK(Consumer::got[n + 1].data[0] == 1000 + 100 * n);
    }
    CHECK(Relay::published() == 6);
}

TEST_CASE("accounting is the source's, passed through by position") {
    fresh_start();
    SrcA::complete_block(1);
    SrcB::complete_block(2);
    SrcB::complete_block(3);
    brio::post<Relay>(BlockDone{});
    run_scheduler();
    CHECK(Relay::laps(0) == SrcA::laps());
    CHECK(Relay::laps(1) == SrcB::laps());
    CHECK(Relay::laps(0) == 1);
    CHECK(Relay::laps(1) == 2);
    CHECK(Relay::overruns(0) == 0);
    CHECK(Relay::source_count == 2);
}

} // namespace
