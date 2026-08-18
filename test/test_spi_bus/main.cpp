// Host tests for util/bus_master.hpp through its SPI vocabulary
// (util/spi_bus.hpp): bus arbitration, FIFO order across clients,
// rejection policy, engine handshake, status pass-through. I2cBus is the
// same class under another alias, so this IS its test too.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/kernel.hpp"
#include "util/spi_bus.hpp"

namespace {

using brio::HostPlatform;
using brio::ReplyTo;
using brio::SpiDone;
using brio::TransferDone;

// Fake engine: records which transaction ids were started. `polled`
// mirrors the real engine's contract: start() returning true means the
// transaction completed synchronously and no TransferDone will follow.
struct FakeBus {
    struct Request {
        uint8_t id;
        ReplyTo<SpiDone> reply;
        bool polled = false;
    };
    static inline std::vector<uint8_t> started;
    static bool start(const Request& r) {
        started.push_back(r.id);
        return r.polled;
    }
};

using Spi = brio::SpiBus<FakeBus, HostPlatform, 2>;   // tiny FIFO on purpose

// Requester recording its replies.
struct Client : brio::Fsm<Client, SpiDone> {
    static inline brio::EventQueue<Event, 8, HostPlatform> queue;
    static inline std::vector<uint8_t> replies;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](SpiDone d) { replies.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); }
        );
    }
};

using K = brio::Kernel<HostPlatform, Client, Spi>;

void reset() {
    HostPlatform::reset();
    FakeBus::started.clear();
    Client::replies.clear();
    while (Spi::queue.pop().has_value()) {}
    while (Client::queue.pop().has_value()) {}
    K::init_all();
}

FakeBus::Request req(uint8_t id) {
    return {id, brio::reply_to<Client, SpiDone>()};
}

FakeBus::Request polled_req(uint8_t id) {
    return {id, brio::reply_to<Client, SpiDone>(), true};
}

} // namespace

TEST_CASE("an idle bus starts the request immediately") {
    reset();
    brio::post<Spi>(req(1));
    while (K::step()) {}

    CHECK(FakeBus::started == std::vector<uint8_t>{1});
    CHECK(Client::replies.empty());          // engine still 'running'
}

TEST_CASE("completion replies to the requester and serves the next in FIFO order") {
    reset();
    brio::post<Spi>(req(1));
    brio::post<Spi>(req(2));
    brio::post<Spi>(req(3));
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1});   // 2,3 pending

    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2});
    CHECK(Client::replies == std::vector<uint8_t>{brio::spi_ok});

    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3});
    CHECK(Client::replies == std::vector<uint8_t>(3, brio::spi_ok));
}

TEST_CASE("a full pending FIFO rejects immediately, loudly, and recovers") {
    reset();
    brio::post<Spi>(req(1));                   // will be started
    brio::post<Spi>(req(2));                   // pending slot 1
    brio::post<Spi>(req(3));                   // pending slot 2
    brio::post<Spi>(req(4));                   // FIFO full -> rejected
    while (K::step()) {}

    CHECK(FakeBus::started == std::vector<uint8_t>{1});
    CHECK(Client::replies == std::vector<uint8_t>{brio::spi_rejected});
    CHECK(Spi::rejected_count() == 1);

    // the bus still drains correctly afterwards
    for (int i = 0; i < 3; ++i) {
        brio::post<Spi>(TransferDone{brio::spi_ok});
        while (K::step()) {}
    }
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3});
    CHECK(Client::replies ==
          std::vector<uint8_t>{brio::spi_rejected, brio::spi_ok, brio::spi_ok,
                               brio::spi_ok});
}

TEST_CASE("the engine's status travels through to the requester") {
    reset();
    brio::post<Spi>(req(9));
    while (K::step()) {}
    brio::post<Spi>(TransferDone{42});         // engine-specific error code
    while (K::step()) {}

    CHECK(Client::replies == std::vector<uint8_t>{42});
}

TEST_CASE("back to idle: a later request starts at once") {
    reset();
    brio::post<Spi>(req(1));
    while (K::step()) {}
    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}

    brio::post<Spi>(req(2));
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2});
}

TEST_CASE("a synchronous completion replies within the dispatch, no TransferDone") {
    reset();
    brio::post<Spi>(polled_req(1));
    while (K::step()) {}

    CHECK(FakeBus::started == std::vector<uint8_t>{1});
    CHECK(Client::replies == std::vector<uint8_t>{brio::spi_ok});

    // the bus is back in idle: the next request starts immediately
    brio::post<Spi>(polled_req(2));
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2});
    CHECK(Client::replies == std::vector<uint8_t>(2, brio::spi_ok));
}

TEST_CASE("the pending FIFO drains through a chain of synchronous completions") {
    reset();
    brio::post<Spi>(req(1));                   // async: holds the bus
    brio::post<Spi>(polled_req(2));            // waits in the FIFO
    brio::post<Spi>(polled_req(3));            // waits in the FIFO
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1});
    CHECK(Client::replies.empty());

    // one TransferDone settles everything: 1 replies, then 2 and 3
    // start-and-complete back to back inside the same dispatch
    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3});
    CHECK(Client::replies == std::vector<uint8_t>(3, brio::spi_ok));

    // and the bus ended up idle, not stuck in busy
    brio::post<Spi>(req(4));
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3, 4});
}

TEST_CASE("a sync chain interrupted by an async transfer stays busy") {
    reset();
    brio::post<Spi>(req(1));                   // async: holds the bus
    brio::post<Spi>(polled_req(2));            // FIFO
    brio::post<Spi>(req(3));                   // FIFO, async
    while (K::step()) {}

    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    // 1 replied, 2 completed synchronously, 3 started and is in flight
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3});
    CHECK(Client::replies == std::vector<uint8_t>(2, brio::spi_ok));

    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    CHECK(Client::replies == std::vector<uint8_t>(3, brio::spi_ok));
}
