// Host tests for the simulated SPI host (brio/host/sim_spi_host.hpp)
// against the contract's two ways to run a request
// (docs/design/spi-bus.md, "Two ways to run a request"): POLLED inside
// start(), or ASYNCHRONOUS, which is DMA - refused at once by a host with
// no DMA (has_dma false: spi_rejected, nothing on the wire), run polled
// inside start() when shorter than the host's threshold, and answered
// later otherwise. Each rule is judged at the engine and again through the
// arbiter of util/bus_master.hpp, which is what a client sees: the reply.
//
// The simulation has no rate and no cost, so the short request's threshold
// is a count the test sets (short_frames()), standing for the one a
// family's host derives from its rate and its measured costs.
//
// Run with: ctest --preset host (or ctest --preset host -R test_sim_spi_host)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <vector>

#include "host/platform.hpp"
#include "host/sim_spi_host.hpp"
#include "kernel/tenuto.hpp"
#include "util/spi_bus.hpp"

using namespace brio;

namespace {

/// A host with DMA (the default) and one without, two buses.
using DmaBus = SimSpiHost<0>;
using PlainBus = SimSpiHost<1, 64, false>;

// What a program asserts before it posts an asynchronous request.
static_assert(DmaBus::has_dma);
static_assert(!PlainBus::has_dma);

constexpr SimSpiPinRef dev_cs{2};
constexpr SimSpiPinRef dc_pin{3};

/// A device that counts what it was clocked and answers a fixed byte.
struct Counter {
    uint32_t selected = 0;
    uint32_t bytes = 0;
    void select(bool low) {
        if (low) {
            ++selected;
        }
    }
    void dc(bool) {}
    uint8_t byte(uint8_t) {
        ++bytes;
        return 0x5A;
    }
};
static_assert(SimSpiDeviceModel<Counter>);

Counter dma_dev;
Counter plain_dev;

uint8_t out[16];
uint8_t in[16];

template <typename Host>
typename Host::Request request(uint16_t len, bool polled, uint8_t cmd_len = 0) {
    typename Host::Request r{};
    r.cs = dev_cs;
    r.dc = dc_pin;
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(out));
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.len = len;
    r.polled = polled;
    return r;
}

void fresh() {
    DmaBus::release();
    PlainBus::release();
    DmaBus::completion(SimSpiCompletion::deferred);
    PlainBus::completion(SimSpiCompletion::deferred);
    DmaBus::short_frames(0);
    PlainBus::short_frames(0);
    SimSpiBench::reset();
    dma_dev = Counter{};
    plain_dev = Counter{};
    REQUIRE(DmaBus::attach(dev_cs, dma_dev));
    REQUIRE(PlainBus::attach(dev_cs, plain_dev));
    for (uint8_t& b : in) {
        b = 0;
    }
}

}   // namespace

// =============================================================================
// the engine
// =============================================================================

TEST_CASE("a host without DMA refuses an asynchronous request at once, nothing on the wire") {
    fresh();
    const auto r = request<PlainBus>(8, false);
    CHECK(PlainBus::start(r));                       // answered inside the call...
    CHECK(PlainBus::status() == spi_rejected);       // ...refused
    CHECK_FALSE(PlainBus::pending());
    CHECK(PlainBus::transactions() == 0);
    CHECK(PlainBus::bytes_clocked() == 0);
    CHECK(plain_dev.selected == 0);
    CHECK(SimSpiBench::sets[dev_cs.pin] == 0);       // the select never moved
    CHECK(SimSpiBench::clears[dev_cs.pin] == 0);
    CHECK(SimSpiBench::sets[dc_pin.pin] == 0);       // nor the D/C
    CHECK(SimSpiBench::clears[dc_pin.pin] == 0);
    CHECK(PlainBus::async_requests() == 1);
    CHECK(PlainBus::refused_requests() == 1);
}

TEST_CASE("a host without DMA refuses even an empty asynchronous request, and in the immediate style") {
    fresh();
    PlainBus::completion(SimSpiCompletion::immediate);
    CHECK(PlainBus::start(request<PlainBus>(0, false)));
    CHECK(PlainBus::status() == spi_rejected);
    CHECK(PlainBus::start(request<PlainBus>(4, false, 2)));
    CHECK(PlainBus::status() == spi_rejected);
    CHECK(PlainBus::refused_requests() == 2);
    CHECK(PlainBus::transactions() == 0);
}

TEST_CASE("a host without DMA runs the same request polled") {
    fresh();
    CHECK(PlainBus::start(request<PlainBus>(8, true)));
    CHECK(PlainBus::status() == spi_ok);
    CHECK(PlainBus::transactions() == 1);
    CHECK(plain_dev.bytes == 8);
    CHECK(in[7] == 0x5A);
    CHECK(PlainBus::refused_requests() == 0);
    // ...and the refusal after it is told apart from it.
    CHECK(PlainBus::start(request<PlainBus>(8, false)));
    CHECK(PlainBus::status() == spi_rejected);
    CHECK(PlainBus::transactions() == 1);
}

TEST_CASE("a short asynchronous request completes inside start(), a long one is held") {
    fresh();
    DmaBus::short_frames(6);
    // Five frames (a command and four data frames): under six, so short.
    CHECK(DmaBus::start(request<DmaBus>(4, false, 1)));
    CHECK(DmaBus::status() == spi_ok);
    CHECK_FALSE(DmaBus::pending());
    CHECK(dma_dev.bytes == 5);
    CHECK(DmaBus::short_requests() == 1);
    // Six frames: at the threshold, held for the DMA handler.
    CHECK_FALSE(DmaBus::start(request<DmaBus>(6, false)));
    CHECK(DmaBus::pending());
    CHECK(dma_dev.bytes == 5);
    CHECK(DmaBus::finish() == spi_ok);
    CHECK(dma_dev.bytes == 11);
    CHECK(DmaBus::short_requests() == 1);
    CHECK(DmaBus::async_requests() == 2);
    CHECK(DmaBus::refused_requests() == 0);
}

TEST_CASE("with no threshold set every asynchronous request is held, and a polled one never is") {
    fresh();
    CHECK_FALSE(DmaBus::start(request<DmaBus>(1, false)));
    CHECK(DmaBus::finish() == spi_ok);
    CHECK(DmaBus::start(request<DmaBus>(1, true)));
    DmaBus::short_frames(100);
    CHECK(DmaBus::start(request<DmaBus>(50, true)));   // polled: the threshold is not its
    CHECK(DmaBus::short_requests() == 0);
    CHECK(DmaBus::polled_requests() == 2);
}

// =============================================================================
// the arbiter over the two hosts
// =============================================================================

namespace {

using DmaArb = SpiBus<DmaBus, HostPlatform, 4>;
using PlainArb = SpiBus<PlainBus, HostPlatform, 4>;

struct Client : Fsm<Client, SpiDone> {
    static inline EventQueue<Event, 8, HostPlatform> queue;
    static inline std::vector<uint8_t> replies;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(
            e, [](Entry) { return handled(); },
            [](SpiDone d) {
                replies.push_back(d.status);
                return handled();
            },
            [](auto) { return unhandled(); });
    }
};

using K = Tenuto<HostPlatform, Client, DmaArb, PlainArb>;

void arbiter_fresh() {
    fresh();
    HostPlatform::reset();
    Client::replies.clear();
    while (DmaArb::queue.pop().has_value()) {
    }
    while (PlainArb::queue.pop().has_value()) {
    }
    while (Client::queue.pop().has_value()) {
    }
    K::init_all();
}

void settle() {
    while (K::step()) {
    }
}

}   // namespace

TEST_CASE("through the arbiter: the refusal is the requester's reply, and the bus stays free") {
    arbiter_fresh();
    auto r = request<PlainBus>(8, false);
    r.reply = reply_to<Client, SpiDone>();
    post<PlainArb>(r);
    settle();
    CHECK(Client::replies == std::vector<uint8_t>{spi_rejected});
    CHECK(PlainBus::transactions() == 0);
    // The bus is idle: the next request, polled, runs at once.
    auto p = request<PlainBus>(8, true);
    p.reply = reply_to<Client, SpiDone>();
    post<PlainArb>(p);
    settle();
    CHECK(Client::replies == std::vector<uint8_t>{spi_rejected, spi_ok});
    CHECK(plain_dev.bytes == 8);
}

TEST_CASE("through the arbiter: a short asynchronous request is answered in the dispatch that received it") {
    arbiter_fresh();
    DmaBus::short_frames(4);
    auto s = request<DmaBus>(3, false);
    s.reply = reply_to<Client, SpiDone>();
    post<DmaArb>(s);
    settle();
    CHECK(Client::replies == std::vector<uint8_t>{spi_ok});
    CHECK_FALSE(DmaBus::pending());
    CHECK(dma_dev.bytes == 3);
}

TEST_CASE("through the arbiter: a long one waits for its handler, the short ones behind it are answered in turn") {
    arbiter_fresh();
    DmaBus::short_frames(4);
    auto big = request<DmaBus>(8, false);
    big.reply = reply_to<Client, SpiDone>();
    auto small = request<DmaBus>(2, false);
    small.reply = reply_to<Client, SpiDone>();
    post<DmaArb>(big);
    post<DmaArb>(small);
    post<DmaArb>(small);
    settle();
    // The long one is on the "engines"; the short ones wait behind it.
    CHECK(Client::replies.empty());
    CHECK(DmaBus::pending());
    // THE TEST IS THE DMA HANDLER'S GLUE: the transfer ends, its status
    // posted; the arbiter answers it and drains the two short ones, each
    // completing inside its start().
    post<DmaArb>(TransferDone{DmaBus::finish()});
    settle();
    CHECK(Client::replies == std::vector<uint8_t>{spi_ok, spi_ok, spi_ok});
    CHECK(dma_dev.bytes == 12);
    CHECK(DmaBus::short_requests() == 2);
}
