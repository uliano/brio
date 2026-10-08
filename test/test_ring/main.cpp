// Host tests for util/ring.hpp: brio::Ring (the SPSC FIFO), brio::SkipRing
// (a Ring whose producer can drop elements and whose consumer skips past
// the drop) and
// brio::HardwareRing (the consumer half of a ring whose producer is the
// hardware, driven here by a scripted circular channel).
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>
#include <vector>

#include "util/ring.hpp"
#include "util/stream.hpp"
#include "host/platform.hpp"

using brio::SkipRing;
using brio::HardwareRing;
using brio::HostPlatform;
using brio::Ring;

namespace {

// A platform whose only atomic access is one byte (the AVR truth): rings
// with 16-bit indices must then take the critical section. Its guard
// counts entries so the test can see the path that was chosen.
struct NarrowPlatform {
    class CriticalSection {
    public:
        CriticalSection() { ++entries; ++depth; }
        ~CriticalSection() { --depth; }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;
        static inline uint32_t entries = 0;
        static inline uint8_t depth = 0;
    };
    static void idle() {}
    static void break_here() {}
    static uint32_t now() { return 0; }
    static constexpr uint32_t ticks_per_second = 1000;
    static constexpr unsigned atomic_width = 1;
    static brio::PanicRecord& panic_record() {
        static brio::PanicRecord rec{};
        return rec;
    }
};
static_assert(brio::Platform<NarrowPlatform>);

} // namespace

TEST_CASE("index type and path follow the size and the platform width") {
    static_assert(sizeof(Ring<uint8_t, 2, HostPlatform>::index_t) == 1);
    static_assert(sizeof(Ring<uint8_t, 256, HostPlatform>::index_t) == 1);
    static_assert(sizeof(Ring<uint8_t, 512, HostPlatform>::index_t) == 2);
    static_assert(sizeof(Ring<uint8_t, 65536, HostPlatform>::index_t) == 2);
    static_assert(sizeof(Ring<uint8_t, 131072, HostPlatform>::index_t) == 4);

    // 32-bit-class host: everything lock-free
    static_assert(Ring<uint8_t, 64, HostPlatform>::lock_free);
    static_assert(Ring<uint8_t, 4096, HostPlatform>::lock_free);
    // byte-atomic target: lock-free up to 256 slots, guarded above
    static_assert(Ring<uint8_t, 256, NarrowPlatform>::lock_free);
    static_assert(!Ring<uint8_t, 512, NarrowPlatform>::lock_free);
    // 32-bit index on a 32-bit host: still lock-free; on a 16-bit-word
    // target it would take the guard - same source, no #ifdef
    static_assert(Ring<uint8_t, 131072, HostPlatform>::lock_free);
}

TEST_CASE("a fresh ring is empty and capacity is size - 1") {
    Ring<uint8_t, 8, HostPlatform> r;
    CHECK(r.empty());
    CHECK_FALSE(r.full());
    CHECK(r.count() == 0);
    CHECK(r.capacity() == 7);
    CHECK_FALSE(r.pop().has_value());
}

TEST_CASE("push/pop is FIFO, full rejects, then drains to empty") {
    Ring<uint16_t, 4, HostPlatform> r;
    CHECK(r.push(10));
    CHECK(r.push(20));
    CHECK(r.push(30));
    CHECK(r.full());
    CHECK(r.count() == 3);
    CHECK_FALSE(r.push(40));          // full: nothing written
    CHECK(r.count() == 3);

    CHECK(r.pop().value() == 10);
    CHECK(r.pop().value() == 20);
    CHECK_FALSE(r.full());
    CHECK(r.push(40));                // room again
    CHECK(r.pop().value() == 30);
    CHECK(r.pop().value() == 40);
    CHECK(r.empty());
    CHECK_FALSE(r.pop().has_value());
}

TEST_CASE("indices wrap correctly across many laps") {
    Ring<uint8_t, 4, HostPlatform> r;
    uint8_t next_in = 0, next_out = 0;
    for (int lap = 0; lap < 1000; ++lap) {
        // interleave: 3 in, 3 out - the ring is briefly full every lap
        // and the 2-bit indices wrap on almost every lap
        CHECK(r.push(next_in++));
        CHECK(r.push(next_in++));
        CHECK(r.push(next_in++));
        CHECK(r.full());
        CHECK(r.pop().value() == next_out++);
        CHECK(r.pop().value() == next_out++);
        CHECK(r.pop().value() == next_out++);
        CHECK(r.empty());
    }
    while (auto v = r.pop()) {
        CHECK(*v == next_out++);
    }
    CHECK(next_in == next_out);
}

TEST_CASE("clear resets to empty") {
    Ring<uint8_t, 8, HostPlatform> r;
    r.push(1);
    r.push(2);
    r.clear();
    CHECK(r.empty());
    CHECK(r.count() == 0);
    CHECK(r.push(3));
    CHECK(r.pop().value() == 3);
}

TEST_CASE("the full 65536-slot ring uses every one of its 65535 slots") {
    static Ring<uint8_t, 65536, HostPlatform> r;   // 64 KB: keep it off the stack
    r.clear();
    for (uint32_t i = 0; i < 65535; ++i) {
        REQUIRE(r.push(static_cast<uint8_t>(i)));
    }
    CHECK(r.full());
    CHECK_FALSE(r.push(0));
    for (uint32_t i = 0; i < 65535; ++i) {
        REQUIRE(r.pop().value() == static_cast<uint8_t>(i));
    }
    CHECK(r.empty());
}

TEST_CASE("lock-free path never touches the critical section") {
    NarrowPlatform::CriticalSection::entries = 0;
    Ring<uint8_t, 16, NarrowPlatform> r;
    r.push(1);
    (void)r.pop();
    (void)r.count();
    (void)r.empty();
    (void)r.full();
    CHECK(NarrowPlatform::CriticalSection::entries == 0);
}

TEST_CASE("guarded path wraps every operation and leaves the guard released") {
    NarrowPlatform::CriticalSection::entries = 0;
    Ring<uint8_t, 512, NarrowPlatform> r;
    CHECK(r.push(1));
    CHECK(NarrowPlatform::CriticalSection::entries == 1);
    CHECK(r.pop().value() == 1);
    CHECK(NarrowPlatform::CriticalSection::entries == 2);
    (void)r.count();
    CHECK(NarrowPlatform::CriticalSection::entries == 3);
    (void)r.empty();     // via count()
    (void)r.full();
    CHECK(NarrowPlatform::CriticalSection::entries == 5);
    CHECK(NarrowPlatform::CriticalSection::depth == 0);

    // and it still behaves as a FIFO
    for (int i = 0; i < 511; ++i) REQUIRE(r.push(static_cast<uint8_t>(i)));
    CHECK(r.full());
    CHECK_FALSE(r.push(0));
    for (int i = 0; i < 511; ++i) REQUIRE(r.pop().value() == static_cast<uint8_t>(i));
    CHECK(r.empty());
    CHECK(NarrowPlatform::CriticalSection::depth == 0);
}

TEST_CASE("simulated producer/consumer interleaving keeps order and count") {
    // A producer that pushes in bursts and a consumer that drains in
    // different bursts, over a ring much smaller than the traffic: models
    // an ISR filling and a loop draining. Every byte must come out once,
    // in order, and the ring must never report more than its capacity.
    Ring<uint8_t, 32, HostPlatform> r;
    uint32_t produced = 0, consumed = 0, rejected = 0;
    for (int round = 0; round < 5000; ++round) {
        const int burst_in = (round * 7) % 13;
        for (int i = 0; i < burst_in; ++i) {
            if (r.push(static_cast<uint8_t>(produced))) ++produced;
            else ++rejected;
        }
        CHECK(r.count() <= r.capacity());
        const int burst_out = (round * 5) % 11;
        for (int i = 0; i < burst_out; ++i) {
            if (auto v = r.pop()) {
                REQUIRE(*v == static_cast<uint8_t>(consumed));
                ++consumed;
            }
        }
    }
    while (auto v = r.pop()) {
        REQUIRE(*v == static_cast<uint8_t>(consumed));
        ++consumed;
    }
    CHECK(produced == consumed);
    CHECK(rejected > 0);   // the ring really was pushed past full
}

// =============================================================================
// The bulk (span) half of the API
//
// read_span()/consume() and write_span()/publish() hand each side the
// CONTIGUOUS run it already owns under the SPSC invariant. The cases
// below pin the two properties that are easy to get wrong: a span never
// wraps (so a wrapped ring is served in two calls), and the spare slot
// that tells full from empty is never handed out.
// =============================================================================

TEST_CASE("spans on an empty ring: nothing to read, the whole buffer to write") {
    Ring<uint8_t, 8, HostPlatform> r;
    CHECK(r.read_span().empty());
    // The producer starts at index 0 with the tail also at 0, so the run
    // stops one short of the end: that slot is the full/empty marker.
    const auto w = r.write_span();
    CHECK(w.size() == 7);
    CHECK(w.data() != nullptr);
    CHECK(r.capacity() == 7);
}

TEST_CASE("write_span/publish and read_span/consume move a whole run") {
    Ring<uint8_t, 8, HostPlatform> r;
    auto w = r.write_span();
    REQUIRE(w.size() == 7);
    for (uint8_t i = 0; i < 4; ++i) {
        w[i] = static_cast<uint8_t>(0xA0 + i);
    }
    CHECK(r.empty());          // nothing is visible until it is published
    r.publish(4);
    CHECK(r.count() == 4);

    auto rd = r.read_span();
    REQUIRE(rd.size() == 4);
    for (uint8_t i = 0; i < 4; ++i) {
        CHECK(rd[i] == static_cast<uint8_t>(0xA0 + i));
    }
    r.consume(4);
    CHECK(r.empty());
    CHECK(r.read_span().empty());
}

TEST_CASE("a partially consumed run is offered again from where it stopped") {
    Ring<uint8_t, 8, HostPlatform> r;
    auto w = r.write_span();
    for (uint8_t i = 0; i < 6; ++i) {
        w[i] = i;
    }
    r.publish(6);
    auto rd = r.read_span();
    REQUIRE(rd.size() == 6);
    r.consume(2);
    rd = r.read_span();
    REQUIRE(rd.size() == 4);
    CHECK(rd[0] == 2);
    CHECK(rd[3] == 5);
    CHECK(r.count() == 4);
}

TEST_CASE("a span NEVER wraps: a straddling ring is served in two calls") {
    Ring<uint8_t, 8, HostPlatform> r;
    // Push the head near the end of the buffer, then free the front.
    for (uint8_t i = 0; i < 6; ++i) {
        REQUIRE(r.push(i));
    }
    for (uint8_t i = 0; i < 5; ++i) {
        REQUIRE(r.pop().value() == i);
    }
    // head = 6, tail = 5: one element queued, and the free run must stop
    // at the end of the buffer rather than wrapping round to index 0.
    CHECK(r.count() == 1);
    auto w = r.write_span();
    REQUIRE(w.size() == 2);           // indices 6 and 7
    w[0] = 0x10;
    w[1] = 0x11;
    r.publish(2);
    CHECK(r.count() == 3);

    // Now the second half of the free room, from index 0.
    auto w2 = r.write_span();
    REQUIRE(w2.size() == 4);          // 0..3, stopping one short of tail = 5
    w2[0] = 0x20;
    r.publish(1);
    CHECK(r.count() == 4);

    // The readable side straddles the wrap the same way: 5,6,7 first...
    auto rd = r.read_span();
    REQUIRE(rd.size() == 3);
    CHECK(rd[0] == 5);
    CHECK(rd[1] == 0x10);
    CHECK(rd[2] == 0x11);
    r.consume(3);
    // ...then the part that had wrapped.
    auto rd2 = r.read_span();
    REQUIRE(rd2.size() == 1);
    CHECK(rd2[0] == 0x20);
    r.consume(1);
    CHECK(r.empty());
}

TEST_CASE("write_span never offers the spare slot, and is empty when full") {
    Ring<uint8_t, 8, HostPlatform> r;
    auto w = r.write_span();
    r.publish(static_cast<uint8_t>(w.size()));   // fill the whole first run
    CHECK(r.count() == 7);
    CHECK(r.full());
    CHECK(r.write_span().empty());
    CHECK_FALSE(r.push(0));            // and push agrees
    // One slot freed: exactly one slot offered back.
    (void)r.pop();
    CHECK(r.write_span().size() == 1);
}

TEST_CASE("publish and consume are clamped to what is really there") {
    Ring<uint8_t, 8, HostPlatform> r;
    // Publishing more than the free room must not walk the head into the
    // tail and make a full ring read as empty.
    r.publish(200);
    CHECK(r.count() == 7);
    CHECK(r.full());
    CHECK_FALSE(r.empty());
    // Consuming more than is queued must not walk the tail past the head.
    r.consume(200);
    CHECK(r.empty());
    CHECK(r.count() == 0);
    CHECK(r.read_span().empty());
}

TEST_CASE("byte and span operations interleave without losing order") {
    Ring<uint8_t, 16, HostPlatform> r;
    uint32_t produced = 0, consumed = 0;
    for (int round = 0; round < 2000; ++round) {
        // Produce: alternate a span burst with single pushes.
        if ((round & 1) == 0) {
            auto w = r.write_span();
            const size_t take = w.size() < 5u ? w.size() : 5u;
            for (size_t i = 0; i < take; ++i) {
                w[i] = static_cast<uint8_t>(produced + i);
            }
            r.publish(static_cast<uint8_t>(take));
            produced += static_cast<uint32_t>(take);
        } else {
            for (int i = 0; i < 3; ++i) {
                if (r.push(static_cast<uint8_t>(produced))) {
                    ++produced;
                }
            }
        }
        REQUIRE(r.count() <= r.capacity());

        // Consume: the mirror image.
        if ((round % 3) == 0) {
            auto rd = r.read_span();
            const size_t take = rd.size() < 4u ? rd.size() : 4u;
            for (size_t i = 0; i < take; ++i) {
                REQUIRE(rd[i] == static_cast<uint8_t>(consumed + i));
            }
            r.consume(static_cast<uint8_t>(take));
            consumed += static_cast<uint32_t>(take);
        } else {
            for (int i = 0; i < 2; ++i) {
                if (auto v = r.pop()) {
                    REQUIRE(*v == static_cast<uint8_t>(consumed));
                    ++consumed;
                }
            }
        }
    }
    while (auto v = r.pop()) {
        REQUIRE(*v == static_cast<uint8_t>(consumed));
        ++consumed;
    }
    CHECK(produced == consumed);
    CHECK(produced > 5000);   // the ring really was worked
}

TEST_CASE("the guarded path guards the span operations too") {
    NarrowPlatform::CriticalSection::entries = 0;
    Ring<uint8_t, 512, NarrowPlatform> r;
    (void)r.write_span();
    CHECK(NarrowPlatform::CriticalSection::entries == 1);
    r.publish(3);
    CHECK(NarrowPlatform::CriticalSection::entries == 2);
    (void)r.read_span();
    CHECK(NarrowPlatform::CriticalSection::entries == 3);
    r.consume(3);
    CHECK(NarrowPlatform::CriticalSection::entries == 4);
    CHECK(NarrowPlatform::CriticalSection::depth == 0);
    CHECK(r.empty());
}

TEST_CASE("spans are correct at the 65536-slot boundary, where size > index_t") {
    // index_t is uint16_t here and `size` is 65536: an end-of-buffer run
    // computed in index_t would come out as ZERO. This is the case that
    // says the arithmetic names its own width.
    static Ring<uint8_t, 65536, HostPlatform> r;
    r.clear();
    const auto w = r.write_span();
    CHECK(w.size() == 65535);
    r.publish(65535);
    CHECK(r.full());
    const auto rd = r.read_span();
    CHECK(rd.size() == 65535);
    r.consume(65535);
    CHECK(r.empty());
}

// =============================================================================
// HardwareRing: the ring whose producer is the hardware
//
// The producer below is a SCRIPTED CIRCULAR CHANNEL, honest to the
// RingCounter contract and to the silicon's habits: it writes one element
// at a time, the value of each being its own position since the start
// (truncated to the element), so a consumer can tell exactly what it was
// handed; its count reloads to N at a lap's end, as a circular channel's
// does; and its lap count is kept by a "completion handler" fed by ONE
// FLAG - served at once by default, held off on demand, in which case the
// count lags the counter, and a second wrap while the flag stands is lost,
// as a latched flag loses it. A burst can also land INSIDE a look, between
// the view's read of the lap count and its read of the counter.
// =============================================================================

namespace {

template <int id, uint32_t N, typename E = uint32_t>
struct Channel {
    static inline E storage[N]{};

    static inline uint32_t written = 0;      // positions written: the truth
    static inline uint32_t counted = 0;      // what laps() reports
    static inline bool flag = false;         // a wrap the handler has not served
    static inline bool held = false;         // the handler held off
    static inline bool zero_at_end = false;  // the count reads 0, not N, at a lap's end
    // A burst written on entry to the inject_at-th call of either function,
    // counted together: a producer that moves between the two reads of a
    // look, whichever read comes first.
    static inline uint32_t calls = 0;
    static inline uint32_t inject_at = 0;
    static inline uint32_t inject_burst = 0;

    static void maybe_inject() {
        ++calls;
        if (inject_burst != 0u && calls == inject_at) {
            const uint32_t k = inject_burst;
            inject_burst = 0;
            write(k);
        }
    }

    static uint32_t laps() {
        maybe_inject();
        return counted;
    }

    static uint32_t remaining() {
        maybe_inject();
        const uint32_t into = written % N;
        if (into == 0u && written != 0u && zero_at_end) {
            return 0;
        }
        return N - into;
    }

    static void serve() {
        if (flag) {
            flag = false;
            counted = counted + 1u;
        }
    }

    static void hold(bool on) {
        held = on;
        if (!on) {
            serve();
        }
    }

    static void write(uint32_t k) {
        for (uint32_t i = 0; i < k; ++i) {
            storage[written % N] = static_cast<E>(written);
            written = written + 1u;
            if (written % N == 0u) {
                flag = true;
                if (!held) {
                    serve();
                }
            }
        }
    }

    // `k` elements at once, for the long distances (the handler serving
    // every wrap): the lap count moves by the boundaries crossed and the
    // storage holds the last lap's values.
    static void jump(uint32_t k) {
        const uint32_t crossed = ((written % N) + k) / N;
        written = written + k;
        counted = counted + crossed;
        const uint32_t fill = k < N ? k : N;
        for (uint32_t i = 0; i < fill; ++i) {
            const uint32_t p = written - fill + i;
            storage[p % N] = static_cast<E>(p);
        }
    }

    static void call_inject(uint32_t calls_from_now, uint32_t burst) {
        inject_at = calls + calls_from_now;
        inject_burst = burst;
    }

    static void reset() {
        written = 0;
        counted = 0;
        flag = false;
        held = false;
        zero_at_end = false;
        calls = 0;
        inject_at = 0;
        inject_burst = 0;
        for (uint32_t i = 0; i < N; ++i) {
            storage[i] = static_cast<E>(0xEEEEEEEEu);
        }
    }
};

/// Read everything the view offers, run by run, checking each released
/// run was intact; returns the values in the order they came.
template <typename View>
std::vector<uint32_t> drain_all() {
    std::vector<uint32_t> got;
    for (;;) {
        const auto run = View::read_span();
        if (run.empty()) {
            break;
        }
        std::vector<uint32_t> part(run.begin(), run.end());
        if (View::consume(static_cast<uint32_t>(run.size()))) {
            got.insert(got.end(), part.begin(), part.end());
        }
    }
    return got;
}

std::vector<uint32_t> positions(uint32_t first, uint32_t count) {
    std::vector<uint32_t> v;
    for (uint32_t i = 0; i < count; ++i) {
        v.push_back(first + i);
    }
    return v;
}

using Ch8 = Channel<1, 8>;
using View8 = HardwareRing<Ch8::storage, Ch8>;

using Byte16 = Channel<2, 16, uint8_t>;
using ByteView = HardwareRing<Byte16::storage, Byte16>;

using Half8 = Channel<3, 8, uint16_t>;
using HalfView = HardwareRing<Half8::storage, Half8>;

void fresh8() {
    Ch8::reset();
    View8::clear();
    View8::clear_overruns();
}

} // namespace

static_assert(brio::RingCounter<Ch8>);
// With a byte element the view IS a SpanSource; with a wider one it is not
// (a SpanSource lends bytes).
static_assert(brio::SpanSource<ByteView>);
static_assert(!brio::SpanSource<View8>);
static_assert(!brio::SpanSource<HalfView>);
static_assert(std::is_same_v<View8::element, uint32_t>);
static_assert(std::is_same_v<HalfView::element, uint16_t>);
static_assert(View8::size == 8 && View8::capacity() == 7);

TEST_CASE("hardware ring: a fresh view is empty") {
    fresh8();
    CHECK(View8::empty());
    CHECK(View8::count() == 0);
    CHECK(View8::read_span().empty());
    CHECK_FALSE(View8::pop().has_value());
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: the head is the counter - a run up to it, then the next") {
    fresh8();
    Ch8::write(3);
    CHECK(View8::count() == 3);
    auto run = View8::read_span();
    REQUIRE(run.size() == 3);
    CHECK(std::vector<uint32_t>(run.begin(), run.end()) == positions(0, 3));
    CHECK(View8::consume(2));
    CHECK(View8::count() == 1);
    Ch8::write(2);
    run = View8::read_span();
    REQUIRE(run.size() == 3);
    CHECK(run[0] == 2);
    CHECK(run[2] == 4);
    CHECK(View8::consume(3));
    CHECK(View8::empty());
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: a run is cut by the end of the storage, the rest comes next") {
    fresh8();
    Ch8::write(6);
    CHECK(View8::consume(6) == true);   // nothing read: a plain release
    CHECK(View8::empty());
    Ch8::write(5);                      // positions 6..10: slots 6, 7, 0, 1, 2
    CHECK(View8::count() == 5);
    auto run = View8::read_span();
    REQUIRE(run.size() == 2);           // slots 6 and 7, stopping at the end
    CHECK(run[0] == 6);
    CHECK(run[1] == 7);
    CHECK(View8::consume(2));
    run = View8::read_span();
    REQUIRE(run.size() == 3);           // the wrapped part, from slot 0
    CHECK(run.data() == &Ch8::storage[0]);
    CHECK(std::vector<uint32_t>(run.begin(), run.end()) == positions(8, 3));
    CHECK(View8::consume(3));
    CHECK(View8::empty());
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: a consume across the wrap releases the wrapped part too") {
    fresh8();
    Ch8::write(5);
    CHECK(View8::consume(5));
    Ch8::write(6);                      // positions 5..10
    auto run = View8::read_span();
    REQUIRE(run.size() == 3);           // slots 5, 6, 7
    // Release past the run: clamped to what is queued, not to the run, so
    // two elements of the wrapped part go with it (Ring's rule).
    CHECK(View8::consume(5));
    run = View8::read_span();
    REQUIRE(run.size() == 1);
    CHECK(run[0] == 10);
    CHECK(run.data() == &Ch8::storage[2]);
    // An over-long release stops at the head.
    CHECK(View8::consume(100));
    CHECK(View8::empty());
    Ch8::write(1);
    CHECK(View8::pop().value() == 11);
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: every wrap, many laps, every element once and in order") {
    // A consumer that looks more often than once a lap, and a completion
    // handler held off at random - never across a second wrap, which one
    // latched flag cannot count.
    fresh8();
    uint32_t expect = 0;
    uint32_t r = 0x12345678u;
    uint32_t held_writes = 0;
    uint32_t holds = 0;
    for (int round = 0; round < 20000; ++round) {
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        // A burst that leaves the consumer short of a lap behind.
        const uint32_t room = View8::capacity() - View8::count();
        const uint32_t burst = room == 0u ? 0u : r % (room + 1u);
        if (Ch8::held && (held_writes + burst >= 8u || (r >> 20) % 4u == 0u)) {
            Ch8::hold(false);
        } else if (!Ch8::held && (r >> 22) % 3u == 0u) {
            Ch8::hold(true);
            held_writes = 0;
            ++holds;
        }
        if (Ch8::held) {
            held_writes += burst;
        }
        Ch8::write(burst);
        // Drain part or all of it, in runs the end of the storage cuts.
        const uint32_t take_runs = (r >> 8) % 3u;
        for (uint32_t k = 0; k < take_runs; ++k) {
            const auto run = View8::read_span();
            if (run.empty()) {
                break;
            }
            const uint32_t n = 1u + (r >> 12) % static_cast<uint32_t>(run.size());
            for (uint32_t i = 0; i < n; ++i) {
                REQUIRE(run[i] == expect + i);
            }
            REQUIRE(View8::consume(n));
            expect += n;
        }
    }
    Ch8::hold(false);
    for (uint32_t v : drain_all<View8>()) {
        REQUIRE(v == expect);
        ++expect;
    }
    CHECK(expect == Ch8::written);
    CHECK(Ch8::written > 8u * 1000u);   // many laps, every wrap crossed
    CHECK(holds > 1000u);
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: a completion not yet run is inferred from the head going back") {
    fresh8();
    Ch8::write(5);
    CHECK(View8::count() == 5);         // the view has looked: head 5
    CHECK(View8::consume(4));           // tail 4
    // The channel wraps (positions 5..10) but its handler is held off: the
    // lap count still says 0 and the counter says 8 - 3, a head of 3,
    // behind the 5 the view saw - impossible, so the lap is added back.
    Ch8::hold(true);
    Ch8::write(6);
    CHECK(Ch8::laps() == 0);
    CHECK(View8::count() == 7);
    CHECK(drain_all<View8>() == positions(4, 7));
    CHECK(View8::overruns() == 0);
    // The handler runs: the count catches up and nothing moves.
    Ch8::hold(false);
    CHECK(Ch8::laps() == 1);
    CHECK(View8::empty());
    Ch8::write(3);
    CHECK(drain_all<View8>() == positions(11, 3));
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: a wrap between the two reads of a look errs low and is added back") {
    fresh8();
    Ch8::write(6);
    CHECK(View8::count() == 6);         // head 6
    CHECK(View8::consume(6));
    // Four elements land INSIDE the next look, between its two reads: they
    // cross the wrap and the handler counts it at once. Read in the view's
    // order the look holds laps = 0 and a count past the wrap - a lap low,
    // added back; read the other way round it would hold a count from
    // before the wrap and laps = 1 - a lap HIGH, a whole lap of elements
    // that do not exist, and nothing could tell it from a real overrun.
    Ch8::call_inject(2, 4);
    CHECK(View8::count() == 4);
    CHECK(Ch8::laps() == 1);
    CHECK(drain_all<View8>() == positions(6, 4));
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: a count that reads 0 at a lap's end means the same head") {
    fresh8();
    Ch8::zero_at_end = true;
    Ch8::write(4);
    CHECK(drain_all<View8>() == positions(0, 4));
    Ch8::write(4);                      // to the lap's end: the count reads 0
    CHECK(Ch8::remaining() == 0);
    CHECK(View8::count() == 4);         // laps 1, count 0: a head of 8
    CHECK(drain_all<View8>() == positions(4, 4));
    // The same with the handler held: laps 1, count 0 - a head of 8, behind
    // the 13 the view last saw, and the lap added back.
    Ch8::hold(true);
    Ch8::write(5);
    CHECK(drain_all<View8>() == positions(8, 5));
    Ch8::write(3);                      // 16: the count reads 0, the lap pending
    CHECK(Ch8::laps() == 1);
    CHECK(View8::count() == 3);
    CHECK(drain_all<View8>() == positions(13, 3));
    Ch8::hold(false);
    Ch8::write(2);
    CHECK(drain_all<View8>() == positions(16, 2));
    // And a full lap unread there is an overrun like anywhere else.
    Ch8::write(8);
    CHECK(View8::count() == 0);
    CHECK(View8::overruns() == 1);
}

TEST_CASE("hardware ring: a lap missed is counted, skipped, and the stream resumes") {
    fresh8();
    Ch8::write(3);
    CHECK(drain_all<View8>() == positions(0, 3));
    // Seven unread is the capacity: still the stream.
    Ch8::write(7);
    CHECK(View8::count() == 7);
    CHECK(View8::overruns() == 0);
    CHECK(drain_all<View8>() == positions(3, 7));
    // A full lap unread is the producer's next write: an overrun.
    Ch8::write(8);
    CHECK(View8::read_span().empty());
    CHECK(View8::overruns() == 1);
    // The view skipped to the head: what comes next is the stream again.
    Ch8::write(2);
    CHECK(drain_all<View8>() == positions(18, 2));
    // Three laps and a bit behind: one look, one overrun.
    Ch8::write(27);
    CHECK_FALSE(View8::pop().has_value());
    CHECK(View8::overruns() == 2);
    Ch8::write(1);
    CHECK(View8::pop().value() == 47);
    CHECK(View8::overruns() == 2);
    View8::clear_overruns();
    CHECK(View8::overruns() == 0);
}

TEST_CASE("hardware ring: a run written over while held is refused at its release") {
    fresh8();
    Ch8::write(2);
    CHECK(drain_all<View8>() == positions(0, 2));
    Ch8::write(6);                      // positions 2..7, unread 6
    auto run = View8::read_span();
    REQUIRE(run.size() == 6);
    CHECK(run[0] == 2);
    // The consumer is slow: three more land, and the third is written into
    // the slot at the tail - the run it holds now says 10 where it said 2.
    Ch8::write(3);
    CHECK(run[0] == 10);
    CHECK_FALSE(View8::consume(6));
    CHECK(View8::overruns() == 1);
    CHECK(View8::empty());              // skipped to the head
    Ch8::write(2);
    CHECK(drain_all<View8>() == positions(11, 2));

    // Two land, filling the lap to its last free slot: the slot at the
    // tail is the producer's NEXT write, not yet made, so the run is intact.
    Ch8::write(5);                      // positions 13..17
    run = View8::read_span();
    REQUIRE(run.size() == 3);           // slots 5, 6, 7
    Ch8::write(3);                      // 18..20: unread 8, the tail untouched
    CHECK(run[0] == 13);
    CHECK(View8::consume(3));
    CHECK(View8::overruns() == 1);
    CHECK(drain_all<View8>() == positions(16, 5));
}

TEST_CASE("hardware ring: pop() takes one element, and refuses one written over under it") {
    fresh8();
    Ch8::write(3);
    CHECK(View8::pop().value() == 0);
    CHECK(View8::pop().value() == 1);
    CHECK(View8::count() == 1);
    // Six land between pop()'s look and its release: the slot it read
    // (position 2, slot 2) is written over by position 10.
    Ch8::write(4);                      // positions 3..6, unread 5
    Ch8::call_inject(3, 6);             // as pop's second look begins
    CHECK_FALSE(View8::pop().has_value());
    CHECK(View8::overruns() == 1);
    Ch8::write(1);
    CHECK(View8::pop().value() == 13);
    CHECK_FALSE(View8::pop().has_value());
}

TEST_CASE("hardware ring: a byte ring lends bytes as a SpanSource") {
    Byte16::reset();
    ByteView::clear();
    ByteView::clear_overruns();
    Byte16::write(20);                  // more than a lap before the first look
    CHECK(ByteView::count() == 0);      // 20 unread of 16 slots: an overrun
    CHECK(ByteView::overruns() == 1);
    Byte16::write(14);                  // positions 20..33: slots 4..15, 0, 1
    std::span<const uint8_t> run = ByteView::read_span();
    REQUIRE(run.size() == 12);
    CHECK(run[0] == 20);
    CHECK(run[11] == 31);
    CHECK(ByteView::consume(12));
    run = ByteView::read_span();
    REQUIRE(run.size() == 2);
    CHECK(run[0] == 32);
    CHECK(run[1] == 33);
    CHECK(ByteView::consume(2));
    CHECK(ByteView::empty());
}

TEST_CASE("hardware ring: a 16-bit ring counts beats, not bytes") {
    Half8::reset();
    HalfView::clear();
    HalfView::clear_overruns();
    Half8::write(5);
    CHECK(HalfView::consume(3));
    Half8::write(5);                    // 10: slots 0, 1 hold 8, 9
    // The counter says 8 - 2 beats to go: the head is 10 elements, 20 bytes.
    CHECK(HalfView::count() == 7);
    std::span<const uint16_t> run = HalfView::read_span();
    REQUIRE(run.size() == 5);           // slots 3..7
    CHECK(run[0] == 3);
    CHECK(run[4] == 7);
    CHECK(HalfView::consume(5));
    run = HalfView::read_span();
    REQUIRE(run.size() == 2);
    CHECK(run[0] == 8);
    CHECK(run[1] == 9);
    CHECK(HalfView::consume(2));
    CHECK(HalfView::overruns() == 0);
}

TEST_CASE("hardware ring: clear() restarts the view with a restarted producer") {
    fresh8();
    Ch8::write(13);
    (void)View8::count();
    Ch8::reset();                       // the channel re-armed from slot 0
    View8::clear();
    CHECK(View8::empty());
    Ch8::write(4);
    CHECK(drain_all<View8>() == positions(0, 4));
    CHECK(View8::overruns() == 1);      // the 13 were a lap missed
}

TEST_CASE("hardware ring: the positions run through 2^32 like any other number") {
    fresh8();
    // Long distances, each one a lap missed (counted), to bring the
    // producer's position just short of 2^32...
    Ch8::jump(0x7FFF'FFF0u);
    CHECK(View8::empty());
    Ch8::jump(0x7FFF'FFF0u);
    CHECK(View8::empty());
    CHECK(View8::overruns() == 2);
    CHECK(Ch8::written == 0xFFFF'FFE0u);
    // ...then ordinary traffic across it: the position wraps, the lap
    // count wraps its top bits away, and every element still comes once.
    uint32_t expect = Ch8::written;
    for (int round = 0; round < 40; ++round) {
        Ch8::write(3);
        for (uint32_t v : drain_all<View8>()) {
            REQUIRE(v == expect);
            ++expect;
        }
    }
    CHECK(Ch8::written == 0x0000'0058u);
    CHECK(expect == Ch8::written);
    CHECK(View8::overruns() == 2);
}

TEST_CASE("hardware ring: lossy traffic - a gap is always counted, a run released is the stream") {
    // Bursts that sometimes run more than a lap past the consumer, and a
    // consumer that is sometimes overtaken between reading a run and
    // releasing it. What must hold: every run released with true is a
    // contiguous slice of the stream continuing the last one, or opening
    // after a gap; and a gap never comes without an overrun counted since
    // the last element delivered.
    fresh8();
    uint32_t r = 0xC0FFEE11u;
    bool have_last = false;
    uint32_t last = 0;
    uint32_t seen_overruns = 0;
    uint32_t gaps = 0;
    uint32_t delivered = 0;
    for (int round = 0; round < 50000; ++round) {
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        Ch8::write(r % 11u);            // up to 10: more than a lap at times
        const auto run = View8::read_span();
        if (run.empty()) {
            continue;
        }
        std::vector<uint32_t> part(run.begin(), run.end());
        Ch8::write((r >> 8) % 4u == 0u ? (r >> 12) % 9u : 0u);   // overtaken, at times
        if (!View8::consume(static_cast<uint32_t>(part.size()))) {
            continue;                   // torn: dropped by the consumer
        }
        for (size_t i = 1; i < part.size(); ++i) {
            REQUIRE(part[i] == part[i - 1] + 1u);
        }
        if (have_last && part[0] != last + 1u) {
            REQUIRE(part[0] > last + 1u);
            REQUIRE(View8::overruns() > seen_overruns);
            ++gaps;
        }
        seen_overruns = View8::overruns();
        last = part.back();
        have_last = true;
        delivered += static_cast<uint32_t>(part.size());
    }
    CHECK(gaps > 100);                  // the loss really happened...
    CHECK(delivered > 50000);           // ...and so did the stream
    CHECK(View8::overruns() >= gaps);
}

TEST_CASE("hardware ring: a lap missed while its completion is pending is counted late, never undone") {
    // The documented corner: a consumer that has not looked for a whole
    // lap, looking while the completion of the latest wrap is still
    // pending, cannot tell that lap from none. What must hold even there:
    // what it delivers is the stream's own elements, strictly increasing
    // and contiguous within a run - a gap, never a repeat or a reordering -
    // and a gap the overrun count has not shown yet is shown at the first
    // look after the handler has run.
    fresh8();
    uint32_t r = 0x5EED1234u;
    bool have_last = false;
    uint32_t last = 0;
    uint32_t seen_overruns = 0;
    uint32_t held_writes = 0;
    bool owed = false;
    uint32_t owed_at = 0;
    uint32_t late = 0;
    uint32_t delivered = 0;
    for (int round = 0; round < 50000; ++round) {
        r ^= r << 13; r ^= r >> 17; r ^= r << 5;
        uint32_t burst = r % 7u;
        if (Ch8::held && (held_writes + burst >= 8u || (r >> 20) % 3u == 0u)) {
            Ch8::hold(false);
        } else if (!Ch8::held && (r >> 22) % 2u == 0u) {
            Ch8::hold(true);
            held_writes = 0;
        }
        if (Ch8::held) {
            held_writes += burst;
        }
        Ch8::write(burst);
        if ((r >> 25) % 3u != 0u) {
            continue;                   // the consumer does not look this round
        }
        const bool check_owed = owed && !Ch8::held;
        const auto run = View8::read_span();
        if (check_owed) {
            REQUIRE(View8::overruns() > owed_at);
            owed = false;
        }
        if (run.empty()) {
            continue;
        }
        std::vector<uint32_t> part(run.begin(), run.end());
        if (!View8::consume(static_cast<uint32_t>(part.size()))) {
            continue;
        }
        for (size_t i = 1; i < part.size(); ++i) {
            REQUIRE(part[i] == part[i - 1] + 1u);
        }
        if (have_last) {
            REQUIRE(part[0] > last);
            if (part[0] != last + 1u && View8::overruns() == seen_overruns) {
                REQUIRE(Ch8::held);     // late only while the handler is held
                if (!owed) {
                    owed = true;
                    owed_at = seen_overruns;
                }
                ++late;
            }
        }
        seen_overruns = View8::overruns();
        last = part.back();
        have_last = true;
        delivered += static_cast<uint32_t>(part.size());
    }
    CHECK(late > 0u);                   // the corner was really reached
    CHECK(delivered > 10000u);
}

TEST_CASE("hardware ring: skips() counts every skip, whichever look made it, and is never cleared") {
    fresh8();
    const uint32_t epoch = View8::skips();
    Ch8::write(8);                      // a lap unread
    CHECK(View8::count() == 0);         // a count() skips like a read_span()
    CHECK(View8::skips() == epoch + 1u);
    CHECK(View8::overruns() == 1u);
    Ch8::write(6);
    auto run = View8::read_span();
    REQUIRE(run.size() == 6);
    Ch8::write(3);                      // the slot at the tail written over
    CHECK_FALSE(View8::consume(6));     // a release refused is a skip too
    CHECK(View8::skips() == epoch + 2u);
    CHECK(View8::overruns() == 2u);
    // overruns() restarts from zero; the epoch does not.
    View8::clear_overruns();
    CHECK(View8::overruns() == 0u);
    CHECK(View8::skips() == epoch + 2u);
    Ch8::write(9);
    CHECK(View8::read_span().empty());
    CHECK(View8::overruns() == 1u);
    CHECK(View8::skips() == epoch + 3u);
    // Nor does clear(): a restarted producer is not a skip, and the epoch
    // a reader compares does not go back.
    Ch8::reset();
    View8::clear();
    CHECK(View8::skips() == epoch + 3u);
    CHECK(View8::overruns() == 1u);
}

TEST_CASE("hardware ring: waiting() looks and writes nothing") {
    fresh8();
    Ch8::write(3);
    CHECK(View8::waiting() == 3u);
    auto run = View8::read_span();
    REQUIRE(run.size() == 3);
    // Asked while the run is held, as an interrupt body asks it: it sees
    // the new elements and moves neither position.
    Ch8::write(2);
    CHECK(View8::waiting() == 5u);
    CHECK(View8::consume(3));
    CHECK(View8::waiting() == 2u);
    CHECK(drain_all<View8>() == positions(3, 2));
    CHECK(View8::waiting() == 0u);

    // A lap missed reads as `size` or more, and is NOT counted or skipped:
    // that is the consumer's next look's.
    const uint32_t epoch = View8::skips();
    Ch8::write(10);
    CHECK(View8::waiting() == 10u);
    CHECK(View8::waiting() == 10u);
    CHECK(View8::skips() == epoch);
    CHECK(View8::overruns() == 0u);
    CHECK(View8::count() == 0u);        // the consumer's look counts it
    CHECK(View8::skips() == epoch + 1u);
    CHECK(View8::waiting() == 0u);
}

TEST_CASE("hardware ring: waiting() between a held run and a lap under it leaves the release its verdict") {
    // The hazard waiting() exists for: a count() from an interrupt body,
    // made while the consumer holds a run and the producer has lapped the
    // ring, would skip the tail under the run - and the release that
    // follows would then find a small queue and answer TRUE for a run the
    // producer wrote over. waiting() leaves the tail where the consumer
    // put it, so the release judges the run.
    fresh8();
    Ch8::write(2);
    CHECK(drain_all<View8>() == positions(0, 2));
    Ch8::write(6);                      // positions 2..7
    auto run = View8::read_span();
    REQUIRE(run.size() == 6);
    Ch8::write(4);                      // 8..11: the slot at the tail written over
    CHECK(View8::waiting() >= 8u);      // the edge test: elements, and a lap
    CHECK_FALSE(View8::consume(6));
    CHECK(View8::overruns() == 1u);
}

TEST_CASE("hardware ring: waiting() infers a pending completion as a look does") {
    fresh8();
    Ch8::write(5);
    CHECK(View8::count() == 5);         // the view has looked: head 5
    CHECK(View8::consume(5));
    Ch8::hold(true);
    Ch8::write(6);                      // 5..10, the wrap not yet counted
    CHECK(Ch8::laps() == 0);
    CHECK(View8::waiting() == 6u);      // a head of 3 behind 5: a lap added back
    Ch8::hold(false);
    CHECK(View8::waiting() == 6u);
    CHECK(drain_all<View8>() == positions(5, 6));
}

// ---- SkipRing: a drop, and the skip past it ----------------------------------

namespace {

// A producer of a numbered stream into a SkipRing: every element is its
// own position in the stream, so a skip is visible in the values, and an
// element the ring refuses is lost().
template <typename R>
struct Numbered {
    R& ring;
    uint32_t next = 0;
    uint32_t refused = 0;
    void send(uint32_t n) {
        for (uint32_t i = 0; i < n; ++i) {
            if (!ring.push(static_cast<uint32_t>(next))) {
                ring.lost();
                ++refused;
            }
            ++next;
        }
    }
    void lose(uint32_t n) {   // n elements the receiver discards
        for (uint32_t i = 0; i < n; ++i) {
            ring.lost();
            ++next;
        }
    }
};

// Read every run, recording the elements and, for each run, whether the
// epoch moved since the run before it.
template <typename R>
std::vector<uint32_t> drain_runs(R& r, uint32_t& epoch, std::vector<bool>* moved = nullptr) {
    std::vector<uint32_t> got;
    for (;;) {
        const auto run = r.read_span();
        if (run.empty()) {
            if (r.skips() == epoch) {
                break;
            }
            epoch = r.skips();   // a skip made by this look: look again
            if (moved != nullptr) {
                moved->push_back(true);
            }
            continue;
        }
        const uint32_t now = r.skips();
        if (moved != nullptr) {
            moved->push_back(now != epoch);
        }
        epoch = now;
        for (auto v : run) {
            got.push_back(v);
        }
        r.consume(static_cast<typename R::index_t>(run.size()));
    }
    return got;
}

} // namespace

TEST_CASE("skip ring: with no loss it is a Ring, and the epoch stays") {
    SkipRing<uint8_t, 8, HostPlatform> r;
    CHECK(r.empty());
    CHECK(r.capacity() == 7);
    CHECK(r.push(1));
    CHECK(r.push(2));
    CHECK(r.count() == 2);
    CHECK(r.pop().value() == 1);
    const auto run = r.read_span();
    REQUIRE(run.size() == 1);
    CHECK(run[0] == 2);
    r.consume(1);
    CHECK(r.empty());
    CHECK_FALSE(r.pop().has_value());
    CHECK(r.read_span().empty());
    CHECK(r.skips() == 0u);
}

TEST_CASE("skip ring: two bytes beside the Ring") {
    static_assert(sizeof(SkipRing<uint8_t, 64, HostPlatform>) ==
                  sizeof(Ring<uint8_t, 64, HostPlatform>) + 2u);
    static_assert(sizeof(SkipRing<uint8_t, 256, HostPlatform>) ==
                  sizeof(Ring<uint8_t, 256, HostPlatform>) + 2u);
    static_assert(sizeof(SkipRing<uint8_t, 512, HostPlatform>) ==
                  sizeof(Ring<uint8_t, 512, HostPlatform>) + 2u);
}

TEST_CASE("skip ring: a byte lost to a FULL ring skips everything queued") {
    // The ring holds 0..6 when 7 and 8 are refused: the next look hands
    // out nothing - 0..6 go with the drop - and the stream resumes with
    // what came after it.
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(9);                          // 0..6 kept, 7 and 8 lost
    CHECK(p.refused == 2u);
    CHECK(r.count() == 7u);
    CHECK(r.skips() == 0u);             // nothing looked at yet
    CHECK(r.read_span().empty());       // the skip
    CHECK(r.skips() == 1u);
    CHECK(r.empty());
    p.send(3);                          // 9..11, across the storage's end
    uint32_t epoch = r.skips();
    std::vector<bool> moved;
    CHECK(drain_runs(r, epoch, &moved) == std::vector<uint32_t>{9, 10, 11});
    CHECK(moved == std::vector<bool>{false, false});
    CHECK(r.skips() == 1u);
}

TEST_CASE("skip ring: a run read before the drop is released whole, the next look skips") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(4);                          // 0..3
    const auto run = r.read_span();
    REQUIRE(run.size() == 4u);
    p.lose(1);                          // 4 discarded while the run is held
    p.send(2);                          // 5, 6
    CHECK(run[3] == 3u);                // the run is the stream before the drop
    r.consume(4);
    CHECK(r.skips() == 0u);
    CHECK(r.read_span().empty());       // 5, 6 go with the drop
    CHECK(r.skips() == 1u);
    p.send(1);
    CHECK(r.pop().value() == 7u);
}

TEST_CASE("skip ring: a discard into an empty ring is skipped at the next look") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(2);
    uint32_t epoch = 0;
    CHECK(drain_runs(r, epoch) == std::vector<uint32_t>{0, 1});
    p.lose(1);                          // 2 discarded with the ring empty
    CHECK(r.read_span().empty());       // skipped, nothing behind it
    CHECK(r.skips() == 1u);
    p.send(2);
    std::vector<bool> moved;
    epoch = r.skips();
    CHECK(drain_runs(r, epoch, &moved) == std::vector<uint32_t>{3, 4});
    CHECK(moved == std::vector<bool>{false});
}

TEST_CASE("skip ring: a skip at the storage's wrap") {
    // The data straddles the end of the storage when the drop falls: the
    // tail jumps across the wrap to the head, and the next run is the
    // stream after the drop, from the head's slot on.
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(6);                          // 0..5 in slots 0..5
    r.consume(5);                       // the tail at slot 5
    p.send(4);                          // 6..9 in slots 6, 7, 0, 1
    p.lose(1);                          // 10
    CHECK(r.count() == 5u);
    CHECK(r.read_span().empty());
    CHECK(r.skips() == 1u);
    CHECK(r.empty());
    p.send(3);                          // 11..13 in slots 2..4
    uint32_t epoch = r.skips();
    CHECK(drain_runs(r, epoch) == std::vector<uint32_t>{11, 12, 13});
    p.send(5);                          // 14..18 across the wrap
    CHECK(drain_runs(r, epoch) == std::vector<uint32_t>{14, 15, 16, 17, 18});
    CHECK(r.skips() == 1u);
}

TEST_CASE("skip ring: many drops between two looks are one skip") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    for (int k = 0; k < 50; ++k) {
        p.send(3);
        p.lose(7);
    }
    CHECK(r.read_span().empty());
    CHECK(r.skips() == 1u);
    CHECK(r.read_span().empty());       // the drops are behind: no second skip
    CHECK(r.skips() == 1u);
}

TEST_CASE("skip ring: exactly 256 drops between two looks are still seen") {
    // An epoch the producer incremented would read its old value again
    // after 256 drops, and the consumer would join the two sides of
    // them; one written as one past the consumer's copy cannot.
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    for (uint32_t n : {256u, 512u, 255u, 257u}) {
        const uint32_t before = r.skips();
        p.send(2);
        p.lose(n);
        p.send(2);
        CHECK(r.read_span().empty());
        CHECK(r.skips() == ((before + 1u) & 0xFFu));
        CHECK(r.empty());
    }
}

TEST_CASE("skip ring: skips() counts modulo 2^8, and every skip moves it") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    uint32_t last = r.skips();
    for (int k = 0; k < 600; ++k) {
        p.send(1);
        p.lose(1);
        CHECK(r.read_span().empty());
        const uint32_t now = r.skips();
        REQUIRE(now != last);
        REQUIRE(now == ((last + 1u) & 0xFFu));
        last = now;
    }
}

TEST_CASE("skip ring: pop() skips as read_span() does") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(2);                          // 0, 1
    CHECK(r.pop().value() == 0u);
    p.lose(1);                          // 2
    p.send(2);                          // 3, 4
    CHECK_FALSE(r.pop().has_value());   // the skip: 1, 3 and 4 with it
    CHECK(r.skips() == 1u);
    CHECK(r.empty());
    p.send(1);
    CHECK(r.pop().value() == 5u);
    p.lose(1);
    CHECK_FALSE(r.pop().has_value());   // an empty ring skips too
    CHECK(r.skips() == 2u);
}

TEST_CASE("skip ring: the producer may ask count() while a drop stands") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(3);
    p.lose(1);
    CHECK(r.count() == 3u);             // no skip from a count
    CHECK_FALSE(r.empty());
    CHECK(r.skips() == 0u);
    r.consume(1);                       // consume() judges nothing
    CHECK(r.count() == 2u);
    CHECK(r.read_span().empty());
    CHECK(r.skips() == 1u);
}

TEST_CASE("skip ring: clear() forgets a drop and keeps the epoch") {
    SkipRing<uint32_t, 8, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    p.send(1);
    p.lose(1);
    CHECK_FALSE(r.pop().has_value());
    CHECK(r.skips() == 1u);
    p.lose(1);
    p.send(1);
    r.clear();
    CHECK(r.empty());
    CHECK(r.push(7));
    CHECK(r.pop().value() == 7u);
    CHECK(r.skips() == 1u);
}

TEST_CASE("skip ring: on a byte-atomic core the guarded path") {
    using R = SkipRing<uint8_t, 512, NarrowPlatform>;
    static_assert(!R::lock_free);
    static R r;
    for (uint32_t i = 0; i < 300u; ++i) {
        REQUIRE(r.push(static_cast<uint8_t>(i)));
    }
    const uint32_t entries = NarrowPlatform::CriticalSection::entries;
    const auto run = r.read_span();
    CHECK(run.size() == 300u);
    r.lost();
    CHECK(r.read_span().empty());       // the skip, its head read under the guard
    CHECK(r.skips() == 1u);
    CHECK(r.empty());
    REQUIRE(r.push(0xAA));
    CHECK(r.pop().value() == 0xAAu);
    CHECK(NarrowPlatform::CriticalSection::entries > entries);
    CHECK(NarrowPlatform::CriticalSection::depth == 0u);
}

TEST_CASE("skip ring: lossy traffic - no run holds a drop, and every jump is a skip seen") {
    // A numbered stream through a small ring, the producer outrunning the
    // consumer at random and discarding at random: every run handed out
    // is consecutive, a run that does not follow on from the one before
    // comes after a move of skips() - and only then -, skips() never moves
    // over a look that hands out a run, and the stream never goes back.
    SkipRing<uint32_t, 16, HostPlatform> r;
    Numbered<decltype(r)> p{r};
    uint32_t seed = 12345u;
    auto rnd = [&](uint32_t n) {
        seed = seed * 1103515245u + 12345u;
        return (seed >> 16) % n;
    };
    uint32_t epoch = 0;
    uint32_t expected = 0;              // the element that follows the last one read
    uint32_t delivered = 0;
    uint32_t skipped = 0;
    for (int step = 0; step < 20000; ++step) {
        if (rnd(10) == 0) {
            p.lose(1 + rnd(2));
        } else {
            p.send(rnd(9));
        }
        uint32_t reads = rnd(3);
        while (reads-- > 0u) {
            const uint32_t at_look = r.skips();
            const auto run = r.read_span();
            if (run.empty()) {
                if (r.skips() != at_look) {
                    ++skipped;
                }
                break;
            }
            REQUIRE(r.skips() == at_look);   // a run lent moves nothing
            // Consecutive: a dropped element never enters the ring, so a
            // run holding a drop would jump inside.
            for (uint32_t k = 1; k < run.size(); ++k) {
                REQUIRE(run[k] == run[k - 1] + 1u);
            }
            if (r.skips() != epoch) {
                REQUIRE(run[0] > expected);
            } else {
                REQUIRE(run[0] == expected);
            }
            epoch = r.skips();
            const uint32_t take = 1u + rnd(static_cast<uint32_t>(run.size()));
            expected = run[take - 1u] + 1u;
            delivered += take;
            r.consume(static_cast<uint8_t>(take));
        }
    }
    delivered += static_cast<uint32_t>(drain_runs(r, epoch).size());
    CHECK(r.read_span().empty());
    CHECK(p.refused > 0u);
    CHECK(skipped > 0u);
    CHECK(delivered < p.next);
}
