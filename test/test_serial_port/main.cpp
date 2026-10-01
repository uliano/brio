// Host tests for util/serial_port.hpp: line assembly, ping-pong ownership,
// backpressure with self-post, consumer-above-producer scheduling - each
// over the two drains: a byte at a time through read_byte(), and a run at
// a time over a transport that lends its receive ring in place.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <deque>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "host/platform.hpp"
#include "util/ring.hpp"
#include "util/serial_port.hpp"

namespace {

using brio::HostPlatform;
using brio::LineReceived;
using brio::RxActivity;

// Fake byte transport: the test loads bytes, SerialPort drains them one
// read_byte() at a time.
struct ByteFake {
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
    static uint32_t queued() { return static_cast<uint32_t>(rx.size()); }
    static void reset() { rx.clear(); }
};

// Fake run transport: a real ring, its consumer half lent in place. The
// counters say how the drain took the bytes - runs, not bytes.
struct RunFake {
    using Rx = brio::Ring<uint8_t, 32, HostPlatform>;
    static inline Rx ring;
    static inline uint32_t spans = 0;      // read_span() calls that lent bytes
    static inline uint32_t consumes = 0;   // consume() calls
    static inline uint32_t bytes_read = 0; // read_byte() calls that gave one

    static bool read_byte(uint8_t& b) {
        const auto v = ring.pop();
        if (!v) {
            return false;
        }
        ++bytes_read;
        b = *v;
        return true;
    }
    static std::span<const uint8_t> read_span() {
        const auto run = ring.read_span();
        if (!run.empty()) {
            ++spans;
        }
        return run;
    }
    static void consume(uint32_t n) {
        ++consumes;
        ring.consume(static_cast<Rx::index_t>(n));
    }
    static void feed(const char* s) {
        while (*s) {
            REQUIRE(ring.push(static_cast<uint8_t>(*s++)));
        }
    }
    static uint32_t queued() { return ring.count(); }
    static void reset() {
        ring.clear();
        spans = 0;
        consumes = 0;
        bytes_read = 0;
    }
};

static_assert(!brio::SpanSource<ByteFake>);
static_assert(brio::SpanSource<RunFake>);

// Sink AO: copies each received line during its dispatch (the only
// window in which the reference is valid).
struct Sink : brio::Fsm<Sink, LineReceived> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline std::vector<std::string> lines;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](LineReceived l) { lines.emplace_back(l.line.get()); return handled(); },
            [](auto) { return unhandled(); }
        );
    }
};

template <typename Transport>
using SerialOver = brio::SerialPort<Transport, HostPlatform, Sink, 16>;

// Mimic the kernel: consumer (Sink) above producer (Serial).
template <typename Serial>
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

template <typename Transport>
void reset() {
    using Serial = SerialOver<Transport>;
    HostPlatform::reset();
    Transport::reset();
    Sink::lines.clear();
    while (Sink::queue.pop().has_value()) {}
    while (Serial::queue.pop().has_value()) {}
    Sink::init();
    Serial::init();
}

using Lines = std::vector<std::string>;

} // namespace

TEST_CASE_TEMPLATE("one line: assembled, delivered, CR ignored", T, ByteFake, RunFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("HELLO\r\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"HELLO"});
    CHECK(T::queued() == 0);
}

TEST_CASE_TEMPLATE("a burst of many lines survives the two-buffer backpressure", T,
                   ByteFake, RunFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("A\nBB\nCCC\nDDDD\nEEEEE\n");   // 5 lines, 2 buffers
    brio::post<Serial>(RxActivity{});       // ONE edge, like the ISR
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"A", "BB", "CCC", "DDDD", "EEEEE"});
    CHECK(T::queued() == 0);
}

TEST_CASE_TEMPLATE("ping-pong: the first line stays intact while the second assembles", T,
                   ByteFake, RunFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("AAAA\nBBBB\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"AAAA", "BBBB"});      // no overwrite
}

TEST_CASE_TEMPLATE("split arrival: a line completed across two edges", T, ByteFake, RunFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("HAL");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines.empty());                       // partial: no event
    CHECK(T::queued() == 0);                          // ... but every byte taken

    T::feed("F\n");
    brio::post<Serial>(RxActivity{});                 // ring emptied: new edge
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"HALF"});
}

TEST_CASE_TEMPLATE("empty lines are delivered (the sink decides their meaning)", T,
                   ByteFake, RunFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("\n\nX\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"", "", "X"});
}

TEST_CASE_TEMPLATE("an overlong line is dropped and counted, the stream recovers", T,
                   ByteFake, RunFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    const uint8_t before = Serial::line_overflows();
    T::feed("0123456789ABCDEFGHIJ\nOK\n");  // first > 16 chars
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"OK"});
    CHECK(Serial::line_overflows() == before + 1);
}

TEST_CASE_TEMPLATE("the drain stops AT the second line and leaves the rest queued", T,
                   ByteFake, RunFake) {
    // Both buffers lent after "A\nB\n": the bytes after the second
    // newline must still be in the transport, not taken and lost - the
    // run drain releases exactly what the assembler took.
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("A\nB\nCC\nD\n");
    brio::post<Serial>(RxActivity{});

    // ONE dispatch of the port alone: the sink has not run yet.
    auto e = Serial::queue.pop();
    REQUIRE(e.has_value());
    Serial::dispatch(*e);
    CHECK(T::queued() == 5);                          // "CC\nD\n" still queued
    CHECK(Sink::queue.pop().has_value());             // "A" posted ...
    CHECK(Sink::queue.pop().has_value());             // ... and "B"
    CHECK_FALSE(Sink::queue.pop().has_value());
    CHECK(Serial::queue.pop().has_value());           // and the self-post

    // The sink's two loans are over; the port resumes where it stopped.
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"CC", "D"});
    CHECK(T::queued() == 0);
}

TEST_CASE("a run transport is drained a run at a time, never through read_byte") {
    using Serial = SerialOver<RunFake>;
    reset<RunFake>();
    RunFake::feed("HELLO\nWORLD\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"HELLO", "WORLD"});
    CHECK(RunFake::bytes_read == 0);
    CHECK(RunFake::spans == 1);                       // twelve bytes, one run
    CHECK(RunFake::consumes == 1);                    // ... one release
}

TEST_CASE("a run straddling the ring's end is drained as two runs, in order") {
    using Serial = SerialOver<RunFake>;
    reset<RunFake>();
    // Walk both indices to 26 of the ring's 32 slots, so a twelve-byte
    // burst sits six before the end and six after the wrap.
    for (int i = 0; i < 26; ++i) {
        REQUIRE(RunFake::ring.push(0));
        REQUIRE(RunFake::ring.pop().has_value());
    }
    RunFake::feed("HELLO\nWORLD\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"HELLO", "WORLD"});
    CHECK(RunFake::spans == 2);
    CHECK(RunFake::consumes == 2);
    CHECK(RunFake::queued() == 0);
}

TEST_CASE("a run cut at the second line resumes inside the same run") {
    // Three lines in one contiguous run: the first dispatch takes two and
    // releases only their bytes; the self-post's dispatch is lent the
    // remainder of the SAME region as a run of its own.
    using Serial = SerialOver<RunFake>;
    reset<RunFake>();
    RunFake::feed("A\nB\nC\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"A", "B", "C"});
    CHECK(RunFake::spans == 2);
    CHECK(RunFake::consumes == 2);
    CHECK(RunFake::queued() == 0);
}
