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
struct Service : brio::Fsm<Service, Request> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Request r) {
                r.reply.send(Done{static_cast<uint8_t>(r.work * 2)});
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }
};

// ---- two independent requesters --------------------------------------------
struct Alpha : brio::Fsm<Alpha, Done> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<uint8_t> got;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Done d) { got.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); }
        );
    }
};

struct Beta : brio::Fsm<Beta, Done> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<uint8_t> got;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Done d) { got.push_back(d.status); return handled(); },
            [](auto) { return unhandled(); }
        );
    }
};

using K = brio::Kernel<HostPlatform, Alpha, Beta, Service>;

void reset() {
    HostPlatform::reset();
    Alpha::got.clear();
    Beta::got.clear();
    while (Service::queue.pop().has_value()) {}
    while (Alpha::queue.pop().has_value()) {}
    while (Beta::queue.pop().has_value()) {}
    K::init_all();
}

} // namespace

TEST_CASE("the reply reaches the requester with the service's payload") {
    reset();
    brio::post<Service>(Request{21, brio::reply_to<Alpha, Done>()});
    while (K::step()) {}

    CHECK(Alpha::got == std::vector<uint8_t>{42});
    CHECK(Beta::got.empty());
}

TEST_CASE("the service serves interleaved clients without knowing them") {
    reset();
    brio::post<Service>(Request{1, brio::reply_to<Alpha, Done>()});
    brio::post<Service>(Request{2, brio::reply_to<Beta, Done>()});
    brio::post<Service>(Request{3, brio::reply_to<Alpha, Done>()});
    while (K::step()) {}

    CHECK(Alpha::got == std::vector<uint8_t>{2, 6});   // queue order kept
    CHECK(Beta::got == std::vector<uint8_t>{4});
}

TEST_CASE("a null capsule makes the request fire-and-forget") {
    reset();
    brio::post<Service>(Request{99, {}});              // nobody to answer
    while (K::step()) {}

    CHECK(Alpha::got.empty());
    CHECK(Beta::got.empty());                          // and no crash
}

TEST_CASE("the capsule reports whether it is armed") {
    constexpr auto armed = brio::reply_to<Alpha, Done>();
    constexpr brio::ReplyTo<Done> null{};
    static_assert(armed);
    static_assert(!null);
    CHECK(static_cast<bool>(armed));
    CHECK_FALSE(static_cast<bool>(null));
}
