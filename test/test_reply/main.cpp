// Host tests for ReplyTo: the request/reply return channel that SPI and
// I2C bus AOs will use. Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"

namespace {

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
using brio::HostPlatform;

// ---- a toy service protocol -------------------------------------------------
struct Done { uint8_t status; };

struct Request {
    uint8_t work;
    brio::ReplyTo<Done> reply;
};
static_assert(std::is_trivially_copyable_v<Request>);
static_assert(sizeof(brio::ReplyTo<Done>) == sizeof(void (*)()),
              "the capsule is one function pointer wide");

// ---- the service: doubles `work`, replies to whoever asked ------------------
struct ServiceAo : brio::Fsm<ServiceAo, Request> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](Request r) {
                r.reply.send(Done{static_cast<uint8_t>(r.work * 2)});
                return handled();
            },
            [](auto) { return unhandled(); },
        }, e);
    }
};

// ---- two independent requesters --------------------------------------------
struct AlphaAo : brio::Fsm<AlphaAo, Done> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<uint8_t> got;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](Done d) { got.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }
};

struct BetaAo : brio::Fsm<BetaAo, Done> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<uint8_t> got;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](Done d) { got.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }
};

using K = brio::Kernel<HostPlatform, AlphaAo, BetaAo, ServiceAo>;

void reset() {
    HostPlatform::reset();
    AlphaAo::got.clear();
    BetaAo::got.clear();
    while (ServiceAo::queue.pop().has_value()) {}
    while (AlphaAo::queue.pop().has_value()) {}
    while (BetaAo::queue.pop().has_value()) {}
    K::init_all();
}

} // namespace

TEST_CASE("the reply reaches the requester with the service's payload") {
    reset();
    brio::post<ServiceAo>(Request{21, brio::reply_to<AlphaAo, Done>()});
    while (K::step()) {}

    CHECK(AlphaAo::got == std::vector<uint8_t>{42});
    CHECK(BetaAo::got.empty());
}

TEST_CASE("the service serves interleaved clients without knowing them") {
    reset();
    brio::post<ServiceAo>(Request{1, brio::reply_to<AlphaAo, Done>()});
    brio::post<ServiceAo>(Request{2, brio::reply_to<BetaAo, Done>()});
    brio::post<ServiceAo>(Request{3, brio::reply_to<AlphaAo, Done>()});
    while (K::step()) {}

    CHECK(AlphaAo::got == std::vector<uint8_t>{2, 6});   // queue order kept
    CHECK(BetaAo::got == std::vector<uint8_t>{4});
}

TEST_CASE("a null capsule makes the request fire-and-forget") {
    reset();
    brio::post<ServiceAo>(Request{99, {}});              // nobody to answer
    while (K::step()) {}

    CHECK(AlphaAo::got.empty());
    CHECK(BetaAo::got.empty());                          // and no crash
}

TEST_CASE("the capsule reports whether it is armed") {
    constexpr auto armed = brio::reply_to<AlphaAo, Done>();
    constexpr brio::ReplyTo<Done> null{};
    static_assert(armed);
    static_assert(!null);
    CHECK(static_cast<bool>(armed));
    CHECK_FALSE(static_cast<bool>(null));
}
