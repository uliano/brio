// Host tests for util/serial_port.hpp: line assembly, ping-pong ownership,
// backpressure with self-post, consumer-above-producer scheduling - each
// over the two drains: a byte at a time through read_byte(), and a run at
// a time over a transport that lends its receive ring in place - and,
// over a transport lending a HardwareRing whose scripted channel writes
// over a run while it is held, the rule that a line is posted only from a
// run released clean; and, over the same channel lapping the ring between
// two drains, the rule that a skip the drain did not make is seen; and,
// over a transport whose receive interrupt drops bytes into a SkipRing - a
// full ring, a discarded frame - the rule that no line is ever delivered
// spliced across a drop: the line begun is torn and counted, what the
// ring held is skipped, and the stream resumes after its next end of
// line or a silence.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <cstdio>
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

// The DMA side of a circular receive, scripted: a 16-byte storage the
// producer writes lap after lap, its count and its lap number what a
// circular channel shows (the count reloads to the length at a lap's end,
// the completion that counts a lap served at once).
struct Channel {
    static constexpr uint32_t length = 16;
    static inline uint8_t storage[length]{};
    static inline uint32_t written = 0;   // bytes the producer has written
    static uint32_t remaining() { return length - written % length; }
    static uint32_t laps() { return written / length; }
    static void write(const char* s) {
        while (*s) {
            storage[written % length] = static_cast<uint8_t>(*s++);
            ++written;
        }
    }
    static void reset() {
        written = 0;
        for (uint8_t& b : storage) {
            b = 0;
        }
    }
};

// Fake engined transport: a real HardwareRing over that channel, its
// consumer half lent in place - consume() answering whether the run was
// intact, rx_skips() the ring's skip epoch. Two hooks script the channel: `written_while_held` is written
// while the `lend`-th run lent from now is HELD, between the read_span()
// that lent it and the consume() that releases it; `written_after_refusal`
// right after a release is refused, as a channel that keeps receiving.
struct HwFake {
    using Rx = brio::HardwareRing<Channel::storage, Channel>;
    static inline const char* written_while_held = nullptr;
    static inline uint32_t lend = 1;
    static inline const char* written_after_refusal = nullptr;
    static inline uint32_t refusals = 0;   // consume() calls that answered false

    static std::span<const uint8_t> read_span() {
        const auto run = Rx::read_span();
        if (!run.empty() && written_while_held != nullptr && --lend == 0u) {
            Channel::write(written_while_held);
            written_while_held = nullptr;
        }
        return run;
    }
    static bool consume(uint32_t n) {
        const bool intact = Rx::consume(n);
        if (!intact) {
            ++refusals;
            if (written_after_refusal != nullptr) {
                Channel::write(written_after_refusal);
                written_after_refusal = nullptr;
            }
        }
        return intact;
    }
    static uint32_t rx_skips() { return Rx::skips(); }
    static void feed(const char* s) { Channel::write(s); }
    static uint32_t queued() { return Rx::count(); }
    static void reset() {
        Channel::reset();
        Rx::clear();
        Rx::clear_overruns();
        written_while_held = nullptr;
        lend = 1;
        written_after_refusal = nullptr;
        refusals = 0;
    }
};

// Fake interrupt receiver: a SkipRing filled as a receive interrupt fills
// it - a byte that finds the ring full is dropped and reported lost(), a
// byte the receiver flags is discarded and reported the same - its
// consumer half lent in place, rx_skips() the ring's skips.
// `received_while_held` arrives while the `lend`-th run lent from now is
// HELD, between its read_span() and its consume().
struct SkipFake {
    using Rx = brio::SkipRing<uint8_t, 32, HostPlatform>;
    static inline Rx ring;
    static inline const char* received_while_held = nullptr;
    static inline uint32_t lend = 1;
    static inline uint32_t dropped = 0;   // bytes the full ring refused

    static bool read_byte(uint8_t& b) {
        const auto v = ring.pop();
        if (!v) {
            return false;
        }
        b = *v;
        return true;
    }
    static std::span<const uint8_t> read_span() {
        const auto run = ring.read_span();
        if (!run.empty() && received_while_held != nullptr && --lend == 0u) {
            feed(received_while_held);
            received_while_held = nullptr;
        }
        return run;
    }
    static void consume(uint32_t n) { ring.consume(static_cast<Rx::index_t>(n)); }
    static uint32_t rx_skips() { return ring.skips(); }
    // The wire: each byte into the ring, or dropped and reported lost(); a
    // '~' is a frame the receiver flags, discarded and reported the same.
    static void feed(const char* s) {
        while (*s) {
            const char c = *s++;
            if (c == '~') {
                ring.lost();
            } else if (!ring.push(static_cast<uint8_t>(c))) {
                ring.lost();
                ++dropped;
            }
        }
    }
    static uint32_t queued() { return ring.count(); }
    static void reset() {
        ring.clear();
        received_while_held = nullptr;
        lend = 1;
        dropped = 0;
    }
};

static_assert(!brio::SpanSource<ByteFake>);
static_assert(brio::SpanSource<RunFake>);
static_assert(brio::SpanSource<HwFake>);
static_assert(!brio::SkippingSource<RunFake>);   // no rx_skips(): nothing compiled
static_assert(brio::SkippingSource<HwFake>);
static_assert(brio::SkippingSource<SkipFake>);   // a SkipRing's skips, reported

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

TEST_CASE_TEMPLATE("one line: assembled, delivered, CR ignored", T, ByteFake, RunFake, SkipFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("HELLO\r\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"HELLO"});
    CHECK(T::queued() == 0);
}

TEST_CASE_TEMPLATE("a burst of many lines survives the two-buffer backpressure", T,
                   ByteFake, RunFake, SkipFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("A\nBB\nCCC\nDDDD\nEEEEE\n");   // 5 lines, 2 buffers
    brio::post<Serial>(RxActivity{});       // ONE edge, like the ISR
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"A", "BB", "CCC", "DDDD", "EEEEE"});
    CHECK(T::queued() == 0);
}

TEST_CASE_TEMPLATE("ping-pong: the first line stays intact while the second assembles", T,
                   ByteFake, RunFake, SkipFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("AAAA\nBBBB\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"AAAA", "BBBB"});      // no overwrite
}

TEST_CASE_TEMPLATE("split arrival: a line completed across two edges", T, ByteFake, RunFake,
                   SkipFake) {
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
                   ByteFake, RunFake, SkipFake) {
    using Serial = SerialOver<T>;
    reset<T>();
    T::feed("\n\nX\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();

    CHECK(Sink::lines == Lines{"", "", "X"});
}

TEST_CASE_TEMPLATE("an overlong line is dropped and counted, the stream recovers", T,
                   ByteFake, RunFake, SkipFake) {
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
                   ByteFake, RunFake, SkipFake) {
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

// ---- a run written over while it is held -------------------------------------

TEST_CASE("over a HardwareRing a line goes out from a clean run, as over a Ring") {
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("HELLO\nWORLD\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"HELLO", "WORLD"});
    // Across the storage's end and through a lap: every line, in order.
    HwFake::feed("ABCDEFGHIJ\nK\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"HELLO", "WORLD", "ABCDEFGHIJ", "K"});
    CHECK(HwFake::queued() == 0u);
    CHECK(HwFake::refusals == 0u);
    CHECK(Serial::torn_lines() == torn_before);
}

TEST_CASE("a run the channel writes over while it is held posts nothing, and the stream resumes at a line") {
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    // "ONE\nTWO\nTH" (positions 0..10) is lent as one run; while it is
    // held the channel writes 21 more bytes (11..31), a lap and more, so
    // the slots the run spans hold positions 16..26 by the time they are
    // read: "XXX\nYYYYYYY". The assemblers complete "XXX" and begin
    // "YYYYYYY", and the release refuses the run: nothing is posted - not
    // "ONE" nor "TWO", whose bytes were never read, nor "XXX" - and the
    // line completed and the one begun are counted.
    HwFake::feed("ONE\nTWO\nTH");
    HwFake::written_while_held = "REE\nQXXX\nYYYYYYYZZZZZ";
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(HwFake::refusals == 1u);
    CHECK(Sink::lines.empty());
    CHECK(HwFake::Rx::overruns() == 1u);
    CHECK(Serial::torn_lines() == torn_before + 2u);
    // The ring skipped to its producer, in the middle of "ZZZZZ..": the
    // stream resumes after the next end of line, so the line whose
    // beginning the skip took never reaches the sink, and the whole ones
    // after it do, each assembled from empty.
    HwFake::feed("ZZ\nAFTER\nNEXT\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"AFTER", "NEXT"});
    CHECK(HwFake::refusals == 1u);
    CHECK(Serial::torn_lines() == torn_before + 2u);
}

TEST_CASE("a refused run after a line already out leaves that line's buffer alone") {
    // Two runs in one dispatch: the first, up to the storage's end,
    // completes "CD" and is released clean, so "CD" goes out and its
    // buffer is LENT until the sink's dispatch. The second, after the
    // wrap, is written over while held and completes "PQ" from what the
    // channel left there, which hands the assembly back to the lent
    // buffer; its release is refused, and the channel goes on with
    // "tt\nUV\n" in the same dispatch. The tear must hand the assembly to
    // the buffer that lends nothing and leave the lent one untouched, or
    // the sink reads "UV" - or nothing - where "CD" was posted.
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("0123456789AB\n");   // positions 0..12
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    REQUIRE(Sink::lines == Lines{"0123456789AB"});
    Sink::lines.clear();
    HwFake::feed("CD\n");             // 13..15: the first run, up to the end
    HwFake::feed("EF\nGH");           // 16..20: the second, after the wrap
    // 21..36 while the second is held: slots 0..4 then read "PQ\nRS".
    HwFake::written_while_held = "aaaaaaaaaaaPQ\nRS";
    HwFake::lend = 2;
    HwFake::written_after_refusal = "tt\nUV\n";
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(HwFake::refusals == 1u);
    CHECK(Sink::lines == Lines{"CD", "UV"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}

TEST_CASE("a line begun in a clean run and cut by a refused one is dropped and counted") {
    // "HEL" arrives alone and is released clean: a line begun. The next
    // run carries no end of line and is written over while held, so the
    // line it would have continued is cut - dropped and counted once - and
    // the stream resumes after the next end of line.
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("HEL");                // 0..2
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines.empty());
    HwFake::feed("LO");                 // 3..4
    HwFake::written_while_held = "aaaaaaaaaaaaaaaaa";   // 5..21: a lap over it
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(HwFake::refusals == 1u);
    CHECK(Sink::lines.empty());
    CHECK(Serial::torn_lines() == torn_before + 1u);
    HwFake::feed("aa\nNEXT\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"NEXT"});
}

// ---- a skip between two drains -------------------------------------------------

TEST_CASE("a lap missed between two drains ends the line begun as torn, in the drain's own look") {
    // "HEL" is released clean: a line begun. The channel then writes 17
    // bytes with no drain between, a lap over the tail: the next drain's
    // read_span() finds it, counts the overrun, skips to the head and
    // lends nothing. The bytes after the skip are not the rest of "HEL":
    // the begun line is dropped and counted, the stream resumes after the
    // next end of line - never "HELS" or "HELSNEXT".
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("HEL");                          // 0..2
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    REQUIRE(Sink::lines.empty());
    HwFake::feed("LO\nLOST LINE\nTAIL");          // 3..19: head 20, tail 3
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();                      // the skip, in read_span()
    CHECK(Sink::lines.empty());
    CHECK(HwFake::Rx::overruns() == 1u);
    CHECK(HwFake::refusals == 0u);
    HwFake::feed("S\nNEXT\n");                    // 20..26
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"NEXT"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
    CHECK(HwFake::queued() == 0u);
}

TEST_CASE("a skip made by a look outside the drain is seen at the next run") {
    // The same lap, found by a count() between two drains (a transport's
    // rx_pending(), say): the drain never sees an empty run where the
    // jump happened, only the epoch that moved.
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("HEL");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    HwFake::feed("LO\nLOST LINE\nTAIL");
    CHECK(HwFake::queued() == 0u);                // the look that skips
    CHECK(HwFake::Rx::overruns() == 1u);
    HwFake::feed("S\nNEXT\nMORE\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"NEXT", "MORE"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}

TEST_CASE("a skip with no line begun drops the fragment after it and counts nothing torn") {
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("A\n");                           // 0..1, a whole line
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    REQUIRE(Sink::lines == Lines{"A"});
    HwFake::feed("0123456789abcdefg");            // 2..18: a lap over the tail
    CHECK(HwFake::queued() == 0u);
    HwFake::feed("hij\nB\n");                     // the end of a line the skip cut
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"A", "B"});
    CHECK(Serial::torn_lines() == torn_before);
}

TEST_CASE("a skip in a stream of whole lines delivers no splice") {
    // Lines of four bytes ("Lnn\n") arrive in bursts; the drain sleeps
    // through one burst of more than a lap. Every line the sink gets is
    // one the channel wrote whole, in the order it was written - none is
    // the begun line completed with bytes from after the gap - and the
    // stream resumes and stays whole. (Where the skip lands on a line's
    // start, the drain cannot tell it from a cut and drops that line too:
    // here the skip lands after line 8, so line 9 goes as well.)
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    std::vector<std::string> sent;
    auto line = [&](uint32_t i) {
        std::string l = "L";
        l += static_cast<char>('0' + (i / 10u) % 10u);
        l += static_cast<char>('0' + i % 10u);
        sent.push_back(l);
        return l + "\n";
    };
    uint32_t i = 0;
    // Line 0 and half of line 1, then a drain: a line begun.
    std::string burst = line(i++);
    burst += line(i++).substr(0, 2);
    HwFake::feed(burst.c_str());                  // 0..5
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    REQUIRE(Sink::lines == Lines{sent[0]});
    // Slept through: the rest of line 1 and lines 2..7, 26 bytes.
    std::string slept = "1\n";
    for (int k = 0; k < 6; ++k) {
        slept += line(i++);
    }
    HwFake::feed(slept.c_str());                  // 6..31
    // Then a line a drain: line 8 lands before the first look and goes
    // with the skip, line 9 starts where the skip landed.
    for (int k = 0; k < 6; ++k) {
        HwFake::feed(line(i++).c_str());
        brio::post<Serial>(RxActivity{});
        run_scheduler<Serial>();
    }
    CHECK(HwFake::Rx::overruns() == 1u);
    CHECK(Serial::torn_lines() == torn_before + 1u);   // line 1, begun as "L0"
    CHECK(Sink::lines == Lines{sent[0], sent[10], sent[11], sent[12], sent[13]});
}

TEST_CASE("a refused release and the skip it makes tear the line once") {
    // The refusal case again, now that the refusal also moves the epoch:
    // the run after it sees the epoch moved, finds nothing begun, and
    // counts nothing more.
    using Serial = SerialOver<HwFake>;
    reset<HwFake>();
    const uint32_t torn_before = Serial::torn_lines();
    HwFake::feed("HEL");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    HwFake::feed("LO");
    HwFake::written_while_held = "aaaaaaaaaaaaaaaaa";
    HwFake::written_after_refusal = "aa\nONE\n";
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(HwFake::refusals == 1u);
    CHECK(HwFake::Rx::overruns() == 1u);
    CHECK(Sink::lines == Lines{"ONE"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}

// ---- a drop made by a receive interrupt, over a SkipRing --------------------------

TEST_CASE("a frame discarded between two runs tears the line it cut") {
    // "HEL" is drained: a line begun. The receiver discards a frame, then
    // "LO" and two whole lines arrive: the drain's next look skips all of
    // it - never "HELLO", never "HEL" with anything after it - and the
    // begun line is dropped and counted. What comes next soon after is the
    // tail of a line whose beginning was skipped, up to its end of line.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t torn_before = Serial::torn_lines();
    const uint32_t skips_before = SkipFake::ring.skips();
    SkipFake::feed("HEL");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    REQUIRE(Sink::lines.empty());
    SkipFake::feed("~LO\nNEXT\nMORE\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines.empty());
    CHECK(Serial::torn_lines() == torn_before + 1u);
    CHECK(SkipFake::queued() == 0u);
    CHECK(SkipFake::ring.skips() == skips_before + 1u);
    SkipFake::feed("TAIL\nAFTER\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"AFTER"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}

TEST_CASE("a full ring: what it held is skipped, and the line the drop cut is never spliced") {
    // The drain sleeps while a burst longer than the ring arrives: the 31
    // bytes the ring holds are whole lines and the head of a fourth, the
    // rest of that fourth line dropped. The look skips them all - the
    // drop is behind the head, and where is not kept - so "FOUR" never
    // meets "TAIL", and the line after the tail is whole.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t skips_before = SkipFake::ring.skips();
    SkipFake::feed("LINE-ONE\nLINE-TWO\nLINE-333\nFOUR");  // 31 bytes: full
    SkipFake::feed("-CUT\n");                               // dropped: full
    CHECK(SkipFake::dropped == 5u);
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines.empty());
    CHECK(SkipFake::ring.skips() == skips_before + 1u);
    SkipFake::feed("TAIL\nAFTER\n");     // "FOUR" + "TAIL" would be the splice
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"AFTER"});
}

TEST_CASE("a drop made while a run is held: the run is the stream, the next look skips") {
    // "AB\nC" is lent as one run; while it is held the line goes on and
    // the ring fills behind it, so bytes are dropped with the run still
    // unreleased. The run was read before the drop - it is released clean
    // and "AB" goes out - and the next look skips everything behind it:
    // the line begun as "C" is torn, and the stream resumes after the
    // next end of line.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t torn_before = Serial::torn_lines();
    SkipFake::feed("AB\nC");
    // 27 more fit behind the held run: five lines, twelve bytes, 3 lost.
    SkipFake::received_while_held = "DEFG\nH\nI\nJ\nK\nL\nMNOPQRSTUVWXxyz";
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(SkipFake::dropped == 3u);
    CHECK(Sink::lines == Lines{"AB"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
    SkipFake::feed("3\nOK\n");           // "MNO..X" + "3" would be the splice
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"AB", "OK"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}

TEST_CASE("a drop while both lines are in flight skips what the ring held") {
    // The drain takes two lines and stops (both buffers lent); the ring
    // fills and drops before the drain is back. The rest of the queue
    // goes with the drop - no line had begun, so none is torn - and the
    // stream resumes after the next end of line.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t torn_before = Serial::torn_lines();
    SkipFake::feed("A\nB\nCCCCCCCCC\nDD");
    brio::post<Serial>(RxActivity{});
    auto e = Serial::queue.pop();
    REQUIRE(e.has_value());
    Serial::dispatch(*e);                 // "A" and "B" out, the drain stops
    SkipFake::feed("DDDDDDDD\nEEEEEEEEEEEEE");   // 3 dropped: full
    CHECK(SkipFake::dropped == 3u);
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"A", "B"});
    SkipFake::feed("ee\nII\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"A", "B", "II"});
    CHECK(Serial::torn_lines() == torn_before);
}

TEST_CASE("over a SkipRing with no loss the epoch never moves and nothing is torn") {
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t torn_before = Serial::torn_lines();
    const uint32_t skips_before = SkipFake::ring.skips();
    for (int k = 0; k < 20; ++k) {
        SkipFake::feed("HELLO\nWORLD\n");
        brio::post<Serial>(RxActivity{});
        run_scheduler<Serial>();
    }
    CHECK(Sink::lines.size() == 40u);
    CHECK(SkipFake::ring.skips() == skips_before);
    CHECK(Serial::torn_lines() == torn_before);
}

TEST_CASE("lossy traffic over a SkipRing: every line delivered is a line sent, whole") {
    // Numbered lines, each carrying a check of its number, fed in bursts
    // of random length while the drain runs at random and the receiver
    // drops frames at random: a line delivered is exactly a line that was
    // sent - never two joined across a drop, never one with a hole - in
    // order, and some are delivered after every kind of loss.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    uint32_t seed = 4242u;
    auto rnd = [&](uint32_t n) {
        seed = seed * 1103515245u + 12345u;
        return (seed >> 16) % n;
    };
    auto line_of = [](uint32_t k) {
        char buf[16];
        const uint32_t check = (k * 7919u + 13u) % 10000u;
        std::snprintf(buf, sizeof buf, "%04u:%04u", static_cast<unsigned>(k % 10000u),
                      static_cast<unsigned>(check));
        return std::string(buf);
    };
    std::string wire;
    uint32_t sent = 0;
    for (int step = 0; step < 4000; ++step) {
        if (wire.empty() || (wire.size() < 64u && rnd(2) == 0)) {
            wire += line_of(sent++) + "\n";
        }
        const uint32_t n = rnd(12);
        std::string burst = wire.substr(0, n);
        wire.erase(0, n);
        for (char& c : burst) {
            if (rnd(40) == 0) {
                c = '~';                  // a frame the receiver discards
            }
        }
        SkipFake::feed(burst.c_str());
        if (rnd(3) == 0) {
            brio::post<Serial>(RxActivity{});
            run_scheduler<Serial>();
        }
        if (wire.empty() && rnd(20) == 0) {
            // Now and then a silence, between two lines: the sender's
            // pause inside a line after a drop took its end is the case
            // serial.md calls undecidable.
            brio::post<Serial>(RxActivity{});
            run_scheduler<Serial>();
            HostPlatform::ticks += 1000u;
        }
    }
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    REQUIRE(Sink::lines.size() > 100u);
    uint32_t last = 0;
    bool first = true;
    for (const std::string& l : Sink::lines) {
        REQUIRE(l.size() == 9u);
        const uint32_t k = static_cast<uint32_t>(std::stoul(l.substr(0, 4)));
        REQUIRE(l == line_of(k));
        if (!first) {
            REQUIRE(k != last);
        }
        first = false;
        last = k;
    }
    CHECK(Sink::lines.size() < sent);
    CHECK(Serial::torn_lines() > 0u);
    CHECK(SkipFake::dropped > 0u);
}

// ---- the resync after a skip ends at a silence ---------------------------------

TEST_CASE("a drop that took the line's own end of line: a silence ends the resync") {
    // "HELLO" is drained, a line begun; its end of line is lost. The look
    // that skips the drop finds the ring empty: the line is torn there.
    // Three seconds later "ERR" arrives whole - after a silence the resync
    // to the next end of line ends, and ERR is not taken for the cut
    // line's tail.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t torn_before = Serial::torn_lines();
    SkipFake::feed("HELLO");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    SkipFake::feed("~");                   // the end of line, lost
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Serial::torn_lines() == torn_before + 1u);
    HostPlatform::ticks += 3000u;
    SkipFake::feed("ERR\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"ERR"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}

TEST_CASE("a burst that overflows the ring, a silence, then one line: it is answered") {
    // The SAM C21's console case: lines back to back overflow the ring
    // while the drain sleeps, and the look skips all the ring held; seconds
    // later one command comes - and is delivered.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    SkipFake::feed("HELP\nHELP\nHELP\nHELP\nHELP\nHELP\nHE");   // 31: full
    SkipFake::feed("LP\nHELP\nHE");                                // all lost
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines.empty());
    HostPlatform::ticks += 3000u;
    SkipFake::feed("ERR\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"ERR"});
}

TEST_CASE("bytes soon after the drop are still the cut line's tail") {
    // No silence: what follows the drop within quiet_ticks of the empty
    // look is skipped to its end of line - the undecidable case where the
    // drop took a line's end and the next line came at once costs that
    // next line.
    using Serial = SerialOver<SkipFake>;
    reset<SkipFake>();
    const uint32_t torn_before = Serial::torn_lines();
    SkipFake::feed("HEL");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    SkipFake::feed("~");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    HostPlatform::ticks += 5u;
    SkipFake::feed("LO\nNEXT\n");
    brio::post<Serial>(RxActivity{});
    run_scheduler<Serial>();
    CHECK(Sink::lines == Lines{"NEXT"});
    CHECK(Serial::torn_lines() == torn_before + 1u);
}
