// Host tests for util/serial_port.hpp: line assembly, ping-pong ownership,
// backpressure with self-post, consumer-above-producer scheduling.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <deque>
#include <string>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "util/serial_port.hpp"

namespace {

using brio::HostPlatform;
using brio::LineReceived;
using brio::RxActivity;

// Fake byte transport: the test loads bytes, SerialPort drains them.
struct FakeTransport {
    static inline std::deque<uint8_t> rx;
    static bool read_byte(uint8_t& b) {
        if (rx.empty()) {
            return false;
        }
        b = rx.front();
        rx.pop_front();
        return true;
    }
    static void feed(const char* s) {
        while (*s) {
            rx.push_back(static_cast<uint8_t>(*s++));
        }
    }
};

// Sink AO: copies each received line during its dispatch (the only
// window in which the reference is valid).
struct Sink : brio::Fsm<Sink, LineReceived> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<std::string> lines;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](LineReceived l) { lines.emplace_back(l.line); return handled(); },
            [](auto) { return unhandled(); }
        );
    }
};

using Serial = brio::SerialPort<FakeTransport, HostPlatform, Sink, 16>;

// Mimic the kernel: consumer (Sink) above producer (Serial).
void run_scheduler() {
    for (;;) {
        if (auto e = Sink::queue.pop()) {
            Sink::dispatch(*e);
        } else if (auto s = Serial::queue.pop()) {
            Serial::dispatch(*s);
        } else {
            break;
        }
    }
}

void reset() {
    HostPlatform::reset();
    FakeTransport::rx.clear();
    Sink::lines.clear();
    while (Sink::queue.pop().has_value()) {}
    while (Serial::queue.pop().has_value()) {}
    Sink::init();
    Serial::init();
}

using Lines = std::vector<std::string>;

} // namespace

TEST_CASE("one line: assembled, delivered, CR ignored") {
    reset();
    FakeTransport::feed("HELLO\r\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler();

    CHECK(Sink::lines == Lines{"HELLO"});
    CHECK(FakeTransport::rx.empty());
}

TEST_CASE("a burst of many lines survives the two-buffer backpressure") {
    reset();
    FakeTransport::feed("A\nBB\nCCC\nDDDD\nEEEEE\n");   // 5 lines, 2 buffers
    brio::post<Serial>(RxActivity{});                   // ONE edge, like the ISR
    run_scheduler();

    CHECK(Sink::lines == Lines{"A", "BB", "CCC", "DDDD", "EEEEE"});
    CHECK(FakeTransport::rx.empty());
}

TEST_CASE("ping-pong: the first line stays intact while the second assembles") {
    reset();
    FakeTransport::feed("AAAA\nBBBB\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler();

    CHECK(Sink::lines == Lines{"AAAA", "BBBB"});      // no overwrite
}

TEST_CASE("split arrival: a line completed across two edges") {
    reset();
    FakeTransport::feed("HAL");
    brio::post<Serial>(RxActivity{});
    run_scheduler();
    CHECK(Sink::lines.empty());                       // partial: no event

    FakeTransport::feed("F\n");
    brio::post<Serial>(RxActivity{});                   // ring emptied: new edge
    run_scheduler();
    CHECK(Sink::lines == Lines{"HALF"});
}

TEST_CASE("empty lines are delivered (the sink decides their meaning)") {
    reset();
    FakeTransport::feed("\n\nX\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler();

    CHECK(Sink::lines == Lines{"", "", "X"});
}

TEST_CASE("an overlong line is dropped and counted, the stream recovers") {
    reset();
    FakeTransport::feed("0123456789ABCDEFGHIJ\nOK\n");  // first > 16 chars
    brio::post<Serial>(RxActivity{});
    run_scheduler();

    CHECK(Sink::lines == Lines{"OK"});
    CHECK(Serial::line_overflows() == 1);
}
