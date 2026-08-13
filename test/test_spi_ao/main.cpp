// Host tests for util/spi_ao.hpp: bus arbitration, FIFO order across
// clients, rejection policy, engine handshake. Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/kernel.hpp"
#include "util/spi_ao.hpp"

namespace {

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
using brio::HostPlatform;
using brio::ReplyTo;
using brio::SpiDone;
using brio::TransferDone;

// Fake engine: records which transaction ids were started.
struct FakeBus {
    struct Request {
        uint8_t id;
        ReplyTo<SpiDone> reply;
    };
    static inline std::vector<uint8_t> started;
    static void start(const Request& r) { started.push_back(r.id); }
};

using Spi = brio::SpiAo<FakeBus, HostPlatform, 2>;   // tiny FIFO on purpose

// Requester recording its replies.
struct ClientAo : brio::Fsm<ClientAo, SpiDone> {
    static inline brio::EventQueue<Event, 8, HostPlatform> queue;
    static inline std::vector<uint8_t> replies;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](SpiDone d) { replies.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }
};

using K = brio::Kernel<HostPlatform, ClientAo, Spi>;

void reset() {
    HostPlatform::reset();
    FakeBus::started.clear();
    ClientAo::replies.clear();
    while (Spi::queue.pop().has_value()) {}
    while (ClientAo::queue.pop().has_value()) {}
    K::init_all();
}

FakeBus::Request req(uint8_t id) {
    return {id, brio::reply_to<ClientAo, SpiDone>()};
}

} // namespace

TEST_CASE("an idle bus starts the request immediately") {
    reset();
    brio::post<Spi>(req(1));
    while (K::step()) {}

    CHECK(FakeBus::started == std::vector<uint8_t>{1});
    CHECK(ClientAo::replies.empty());          // engine still 'running'
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
    CHECK(ClientAo::replies == std::vector<uint8_t>{brio::spi_ok});

    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    brio::post<Spi>(TransferDone{brio::spi_ok});
    while (K::step()) {}
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3});
    CHECK(ClientAo::replies == std::vector<uint8_t>(3, brio::spi_ok));
}

TEST_CASE("a full pending FIFO rejects immediately, loudly, and recovers") {
    reset();
    brio::post<Spi>(req(1));                   // will be started
    brio::post<Spi>(req(2));                   // pending slot 1
    brio::post<Spi>(req(3));                   // pending slot 2
    brio::post<Spi>(req(4));                   // FIFO full -> rejected
    while (K::step()) {}

    CHECK(FakeBus::started == std::vector<uint8_t>{1});
    CHECK(ClientAo::replies == std::vector<uint8_t>{brio::spi_rejected});
    CHECK(Spi::rejected_count() == 1);

    // the bus still drains correctly afterwards
    for (int i = 0; i < 3; ++i) {
        brio::post<Spi>(TransferDone{brio::spi_ok});
        while (K::step()) {}
    }
    CHECK(FakeBus::started == std::vector<uint8_t>{1, 2, 3});
    CHECK(ClientAo::replies ==
          std::vector<uint8_t>{brio::spi_rejected, brio::spi_ok, brio::spi_ok,
                               brio::spi_ok});
}

TEST_CASE("the engine's status travels through to the requester") {
    reset();
    brio::post<Spi>(req(9));
    while (K::step()) {}
    brio::post<Spi>(TransferDone{42});         // engine-specific error code
    while (K::step()) {}

    CHECK(ClientAo::replies == std::vector<uint8_t>{42});
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
