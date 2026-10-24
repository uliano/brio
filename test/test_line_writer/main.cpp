// Host tests for util/line_writer.hpp, the one writer of a transport,
// over two simulated transports: the PL011 of host/sim_pl011.hpp (the IP
// stratum's real transport over a chip made of RAM) and a scripted ring
// local to this suite whose room, transmitter and transmit edge the test
// drives. The bytes a print() to the writer puts on the wire against the
// bytes print() writes to a capture; the bound against what was written;
// the message's room; lines in pieces from one source and from two under
// the preempting kernel; a handler's line and a handler's piece; the
// tear; the wait at the bound and the wake on the edge; the gap mark; the
// inline service; the panic drain; and the debug checks (the one-writer
// claim, a text on the stack). The two refusals that must not compile
// are this suite's neg/ TUs, compiled by ctest (test/CMakeLists.txt).
// Run with: ctest --preset host (or ctest --preset host -R test_line_writer)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>

#include "host/platform.hpp"
#include "host/sim_pl011.hpp"
#include "kernel/active_object.hpp"
#include "kernel/fsm.hpp"
#include "kernel/panic.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "pl011/uart.hpp"
#include "util/line_writer.hpp"
#include "util/print.hpp"

using namespace brio;

namespace {

// ---- the two transports ----------------------------------------------------

/// A ring of `cap` bytes whose transmitter is the test: shift() moves the
/// oldest bytes to the wire and raises the transmit edge when it empties
/// the ring. With drain_on_poll, every tx_room() moves one byte first -
/// the transmitter running while the inline service polls.
template <int id, uint32_t cap = 255>
struct Script {
    static inline std::string ring;
    static inline std::string wire;
    static inline bool edge = false;
    static inline bool drain_on_poll = false;
    static inline uint32_t polls = 0;

    static constexpr uint32_t tx_capacity() { return cap; }
    static uint32_t tx_room() {
        ++polls;
        if (drain_on_poll && !ring.empty()) {
            shift(1);
        }
        return cap - static_cast<uint32_t>(ring.size());
    }
    static bool write_byte(uint8_t b) {
        check_one_writer<Script>();
        if (ring.size() == cap) {
            return false;
        }
        ring.push_back(static_cast<char>(b));
        return true;
    }
    static uint32_t write_bulk(std::span<const uint8_t> run) {
        check_one_writer<Script>();
        const size_t room = cap - ring.size();
        const size_t take = run.size() < room ? run.size() : room;
        ring.append(reinterpret_cast<const char*>(run.data()), take);
        return static_cast<uint32_t>(take);
    }
    static bool tx_drained() {
        const bool e = edge;
        edge = false;
        return e;
    }

    static void shift(size_t n) {
        if (n == 0u || ring.empty()) {
            return;
        }
        n = n < ring.size() ? n : ring.size();
        wire += ring.substr(0, n);
        ring.erase(0, n);
        if (ring.empty()) {
            edge = true;
        }
    }
    static void drain() { shift(ring.size()); }
    /// Everything the transport took, in order.
    static std::string taken() { return wire + ring; }
    static void reset() {
        ring.clear();
        wire.clear();
        edge = false;
        drain_on_poll = false;
        polls = 0;
        WriterClaim<Script>::claimed = false;
        WriterClaim<Script>::writing = false;
    }
};
static_assert(EdgeSink<Script<0>> && BulkSink<Script<0>>);

/// The PL011 transport over the chip made of RAM.
using Clock = SimPl011Clock<125'000'000>;
constexpr SimPl011Pins pins{.tx = 0, .rx = 1};
using Port = Pl011Transport<SimPl011, 0, pins, 64, 256, SimNoEngine, SimNoEngine>;
static_assert(EdgeSink<Port>);

/// The byte capture print() writes to, the reference.
struct Capture {
    static inline std::string text;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
};

template <typename... Args>
std::string printed(const Args&... args) {
    Capture::text.clear();
    print(Capture{}, args...);
    return Capture::text;
}

/// Serve a writer the way a kernel does, until its queue is empty.
template <typename W>
void serve() {
    while (serve_one<W>()) {
    }
}

/// The transport's interrupt glue for a writer: the edge posts TxActivity.
template <typename W, typename T>
void glue() {
    if (T::tx_drained()) {
        post<W>(TxActivity{});
    }
}

/// Run the PL011's wire to the end: the handler while its line is
/// raised (the glue after it), the wire taking the FIFO's oldest entry.
template <typename W>
std::string pl011_wire() {
    std::string out;
    for (uint32_t guard = 0; guard < 100'000u; ++guard) {
        while (SimPl011::line_raised<0>()) {
            (void)Port::isr();
            glue<W, Port>();
        }
        const auto b = SimPl011::shift<0>();
        if (b) {
            out.push_back(static_cast<char>(*b));
        } else if (Port::tx_idle()) {
            break;
        }
    }
    return out;
}

void fresh_host() {
    HostPlatform::reset();
    dispatching<HostPlatform> = no_dispatch;
}

// Twenty prints over every type print.hpp prints, each a whole line.
#define LINES(X)                                                                               \
    X('c', '!', crlf)                                                                          \
    X("a text, ", std::string_view("a view"), crlf)                                            \
    X(int8_t{-128}, ' ', int8_t{127}, ' ', uint8_t{255}, crlf)                                 \
    X(int16_t{-32768}, ' ', int16_t{32767}, ' ', uint16_t{65535}, crlf)                        \
    X(std::numeric_limits<int32_t>::min(), ' ', std::numeric_limits<uint32_t>::max(), crlf)    \
    X(std::numeric_limits<int64_t>::min(), crlf)                                               \
    X(std::numeric_limits<uint64_t>::max(), crlf)                                              \
    X(true, false, " bools", crlf)                                                             \
    X(hex(0xBEEFu), ' ', hex(0u), ' ', hex(0xFFFF'FFFFu), crlf)                                \
    X(hex(uint64_t{1} << 40), ' ', hex(~uint64_t{0}), crlf)                                    \
    X(fixed(3.14159f, 8, 3), '|', fixed(-2.5f, -8, 1), '|', crlf)                              \
    X(fixed(1.0e30, 0, 18), crlf)                                                              \
    X(fixed(-3.4e38f, 60, 2), crlf)                                                            \
    X(sci(0.00123f, 3), ' ', sci(-6.02e23, 7), crlf)                                           \
    X(1.5f, ' ', -2.25, crlf)                                                                  \
    X(std::numeric_limits<double>::quiet_NaN(), ' ', -std::numeric_limits<float>::infinity(),  \
      crlf)                                                                                    \
    X(TimeStamp{12, 45}, ' ', TimeStamp{4'294'967'295u, 999}, crlf)                            \
    X("two lines", crlf, "in one call", crlf)                                                  \
    X("a ", "line ", "of ", "short ", "texts", crlf)                                           \
    X("", 'x', "", crlf)

}  // namespace

// =============================================================================
// 1. the bytes
// =============================================================================

using W1 = LineWriter<Port, HostPlatform, 8, 64>;

TEST_CASE("the bytes on the wire are the bytes print() writes to a transport") {
    fresh_host();
    SimPl011::reset_all();
    constexpr Clock clock{};
    REQUIRE(Port::init(clock, 115200));
    W1::init();
    uint32_t lines = 0;
#define ONE(...)                                                                               \
    {                                                                                          \
        const std::string want = printed(__VA_ARGS__);                                         \
        CHECK(print(W1{}, __VA_ARGS__));                                                       \
        CHECK(pl011_wire<W1>() == want);                                                       \
        ++lines;                                                                               \
    }
    LINES(ONE)
#undef ONE
    CHECK(lines == 20u);
    CHECK(W1::lost() == 0u);
    serve<W1>();   // the last edges' TxActivity
    CHECK(W1::idle());
    WriterClaim<Port>::claimed = false;
}

// =============================================================================
// 2. the bound
// =============================================================================

using W2 = LineWriter<Script<2>, HostPlatform, 8, 64, LineWriterOptions{.measured = true}>;

TEST_CASE("each message's bound holds the bytes it wrote; the ratio over the twenty") {
    fresh_host();
    Script<2>::reset();
    W2::init();
    bool every = true;
    std::string worst;
    double worst_ratio = 0.0;
#define ONE(...)                                                                               \
    {                                                                                          \
        const uint32_t b0 = W2::bytes_written();                                               \
        const uint32_t n0 = W2::bound_written();                                               \
        CHECK(print(W2{}, __VA_ARGS__));                                                       \
        Script<2>::drain();                                                                    \
        const uint32_t bytes = W2::bytes_written() - b0;                                       \
        const uint32_t bound = W2::bound_written() - n0;                                       \
        every = every && bytes <= bound;                                                       \
        const double r = bytes == 0u ? 0.0 : static_cast<double>(bound) / bytes;               \
        if (r > worst_ratio) {                                                                 \
            worst_ratio = r;                                                                   \
            worst = #__VA_ARGS__;                                                              \
        }                                                                                      \
    }
    LINES(ONE)
#undef ONE
    CHECK(every);
    CHECK(W2::lines_written() == 20u);
    const double ratio = static_cast<double>(W2::bound_written()) / W2::bytes_written();
    MESSAGE("bound/written over the twenty lines: " << W2::bound_written() << " / "
                                                    << W2::bytes_written() << " = " << ratio
                                                    << "; the worst line " << worst_ratio
                                                    << " (" << worst << ")");
    CHECK(ratio >= 1.0);
}

// =============================================================================
// 3. the message's room
// =============================================================================

using W3 = LineWriter<Script<3>, HostPlatform>;

TEST_CASE("a text too long for the bound is refused at the call and counted") {
    fresh_host();
    Script<3>::reset();
    W3::init();
    const std::string long_text(300, 'x');
    CHECK_FALSE(print(W3{}, long_text.c_str(), crlf));
    CHECK(W3::lost() == 1u);
    CHECK(Script<3>::taken() == "lost 1\r\n");   // the inline service drained the mark
    // A text that fits the 255 of the bound but not the ring with its
    // closing: 254 + 2 > 255.
    const std::string near(254, 'y');
    CHECK_FALSE(print(W3{}, near.c_str(), crlf));
    CHECK(W3::lost() == 2u);
    const std::string fits(253, 'z');
    Script<3>::drain();
    Script<3>::wire.clear();
    CHECK(print(W3{}, fits.c_str(), crlf));
    CHECK(Script<3>::taken().substr(0, 255) == fits + "\r\n");
}

// =============================================================================
// 4. a line in pieces
// =============================================================================

using W4 = LineWriter<Script<4>, HostPlatform>;

TEST_CASE("a line in pieces from one source leaves whole at its end, nothing before") {
    fresh_host();
    Script<4>::reset();
    W4::init();
    dispatching<HostPlatform> = 0;   // inside object 0's dispatch: posted, not served inline
    CHECK(print(W4{}, "count:"));
    serve<W4>();
    for (uint32_t i = 0; i < 3u; ++i) {
        CHECK(print(W4{}, ' ', i));
        serve<W4>();
        CHECK(Script<4>::taken().empty());
    }
    CHECK(print(W4{}, " end", crlf));
    CHECK(Script<4>::taken().empty());   // queued: the kernel serves the writer later
    serve<W4>();
    CHECK(Script<4>::taken() == "count: 0 1 2 end\r\n");
    CHECK(W4::idle());
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 5. two sources under the preempting kernel
// =============================================================================

using RP = RubatoHostPlatform;
using W5 = LineWriter<Script<5>, RP, 8, 48>;

void rubato_isr(void (*f)()) {
    RP::in_isr = true;
    f();
    RP::in_isr = false;
    if (RP::pending && RP::scheduler != nullptr) {
        RP::pending = false;
        (void)RP::scheduler();
    }
}

struct Go {};

struct High {
    using Event = std::variant<Go>;
    static inline EventQueue<Event, 4, RP> queue;
    static void init() {}
    static void dispatch(const Event&) { CHECK(print(W5{}, "HIGH ", 42, " whole", crlf)); }
};

struct Low {
    using Event = std::variant<Go>;
    static inline EventQueue<Event, 4, RP> queue;
    static void init() {}
    static void dispatch(const Event&) {
        CHECK(print(W5{}, "LOW "));
        CHECK(print(W5{}, "begun, "));
        rubato_isr([] { post<High>(Go{}); });   // an interrupt wakes High: road 1
        CHECK(Script<5>::taken().empty());       // High's line waits for the writer, below Low
        CHECK(print(W5{}, "ended", crlf));
    }
};

using K5 = Rubato<RP, High, Low, W5>;

TEST_CASE("two sources interleaved under Rubato: the preempting line whole, then the other") {
    RP::reset();
    Script<5>::reset();
    RP::scheduler = &K5::activate;
    K5::init_all();
    post<Low>(Go{});   // from the loop's base: Low, then the writer, run at once
    CHECK(Script<5>::taken() == "HIGH 42 whole\r\nLOW begun, ended\r\n");
    CHECK(W5::lost() == 0u);
    CHECK(W5::idle());
    RP::reset();
}

// =============================================================================
// 6. a handler's line, a handler's piece
// =============================================================================

using W6 = LineWriter<Script<6>, HostPlatform>;

TEST_CASE("a handler's whole line between two pieces goes out first; a handler's piece is refused") {
    fresh_host();
    Script<6>::reset();
    W6::init();
    dispatching<HostPlatform> = 0;
    CHECK(print(W6{}, "object: "));
    serve<W6>();
    HostPlatform::in_isr = true;
    CHECK(print(W6{}, "irq ", hex(0x55u), crlf));
    HostPlatform::in_isr = false;
    serve<W6>();
    CHECK(Script<6>::taken() == "irq 0x55\r\n");
    CHECK(print(W6{}, "done", crlf));
    serve<W6>();
    CHECK(Script<6>::taken() == "irq 0x55\r\nobject: done\r\n");

    HostPlatform::in_isr = true;
    CHECK_FALSE(print(W6{}, "irq piece "));
    HostPlatform::in_isr = false;
    CHECK(W6::lost() == 1u);
    CHECK(W6::torn_lines() == 0u);
    CHECK(print(W6{}, "after", crlf));
    serve<W6>();
    CHECK(Script<6>::taken() == "irq 0x55\r\nobject: done\r\nafter\r\nlost 1\r\n");
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 7. the tear
// =============================================================================

using W7 = LineWriter<Script<7>, HostPlatform, 4>;

TEST_CASE("a line of more pieces than free slots tears: refused, released unwritten, counted") {
    fresh_host();
    Script<7>::reset();
    W7::init();
    dispatching<HostPlatform> = 0;
    // Depth 4: five slots, one kept for the edge - four pieces held.
    uint32_t taken = 0;
    for (uint32_t i = 0; i < 6u; ++i) {
        if (print(W7{}, "piece ", i, ' ')) {
            ++taken;
        }
        serve<W7>();
    }
    CHECK(taken == 4u);
    CHECK(W7::lost() == 1u);                 // the line, once
    CHECK_FALSE(print(W7{}, "the end", crlf));   // refused unposted: it ends the tear
    CHECK(W7::lost() == 1u);
    CHECK(print(W7{}, "the next line", crlf));
    serve<W7>();
    CHECK(W7::torn_lines() == 1u);
    CHECK(Script<7>::taken() == "lost 1\r\nthe next line\r\n");   // the mark at the tear's wake
    CHECK(W7::idle());
    // Every slot came back: a line of four messages fits again.
    for (uint32_t i = 0; i < 3u; ++i) {
        CHECK(print(W7{}, i));
        serve<W7>();
    }
    CHECK(print(W7{}, crlf));
    serve<W7>();
    CHECK(Script<7>::taken() == "lost 1\r\nthe next line\r\n012\r\n");
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 8. the wait at the bound, the wake on the edge
// =============================================================================

using W8 = LineWriter<Script<8, 32>, HostPlatform>;

TEST_CASE("a ring with room for one line: one out, the others wait, each edge lets one more out") {
    fresh_host();
    using T = Script<8, 32>;
    T::reset();
    W8::init();
    dispatching<HostPlatform> = 0;
    CHECK(print(W8{}, "first line of twenty", crlf));    // 20 + 2
    CHECK(print(W8{}, "second line of twent", crlf));
    CHECK(print(W8{}, "third line of twenty", crlf));
    serve<W8>();
    CHECK(T::taken() == "first line of twenty\r\n");
    CHECK(W8::waits() == 1u);
    T::drain();
    glue<W8, T>();
    serve<W8>();
    CHECK(T::taken() == "first line of twenty\r\nsecond line of twent\r\n");
    CHECK(W8::waits() == 2u);
    T::drain();
    glue<W8, T>();
    serve<W8>();
    CHECK(T::taken() ==
          "first line of twenty\r\nsecond line of twent\r\nthird line of twenty\r\n");
    CHECK(W8::waits() == 2u);
    CHECK(W8::idle());
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 9. the gap mark
// =============================================================================

using W9 = LineWriter<Script<9>, HostPlatform, 2>;

TEST_CASE("after a loss the mark comes out once, after the line before it") {
    fresh_host();
    Script<9>::reset();
    W9::init();
    dispatching<HostPlatform> = 0;
    CHECK(print(W9{}, "before", crlf));
    serve<W9>();
    // Depth 2: three slots, one kept - two lines posted, the third refused.
    CHECK(print(W9{}, "one", crlf));
    CHECK(print(W9{}, "two", crlf));
    CHECK_FALSE(print(W9{}, "three", crlf));
    CHECK(W9::lost() == 1u);
    CHECK(W9::torn_lines() == 0u);
    serve<W9>();
    CHECK(Script<9>::taken() == "before\r\none\r\ntwo\r\nlost 1\r\n");
    CHECK(print(W9{}, "after", crlf));
    serve<W9>();
    CHECK(Script<9>::taken() == "before\r\none\r\ntwo\r\nlost 1\r\nafter\r\n");
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 10. the inline service
// =============================================================================

using W10 = LineWriter<Script<10, 40>, HostPlatform>;

TEST_CASE("from main context the call returns with the line written; from a dispatch, queued") {
    fresh_host();
    using T = Script<10, 40>;
    T::reset();
    W10::init();
    CHECK(print(W10{}, "main context ", 1, crlf));
    CHECK(T::taken() == "main context 1\r\n");
    CHECK(W10::queue.empty());
    CHECK(W10::idle());

    // A ring of 40 and lines of 32: the second waits, the transmitter
    // running while the service polls (one byte a poll).
    T::drain_on_poll = true;
    CHECK(print(W10{}, "a line of thirty-two bytes .....", crlf));
    CHECK(print(W10{}, "a line of thirty-two bytes ,,,,,", crlf));
    CHECK(W10::idle());
    CHECK(T::taken() ==
          "main context 1\r\na line of thirty-two bytes .....\r\na line of thirty-two bytes ,,,,,\r\n");
    CHECK(W10::waits() >= 1u);
    T::drain_on_poll = false;

    dispatching<HostPlatform> = 3;   // inside object 3's dispatch: posted alone
    const std::string before = T::taken();
    CHECK(print(W10{}, "from a dispatch", crlf));
    CHECK(T::taken() == before);
    CHECK_FALSE(W10::queue.empty());
    T::drain();   // the transmitter empties the ring: the edge
    glue<W10, T>();
    serve<W10>();
    CHECK(T::taken() == before + "from a dispatch\r\n");
    dispatching<HostPlatform> = no_dispatch;
}

using W10b = LineWriter<Script<11>, HostPlatform, 8, 32, LineWriterOptions{.inline_from = 2}>;

TEST_CASE("the serving context can be named: an object's dispatch") {
    fresh_host();
    Script<11>::reset();
    W10b::init();
    CHECK(print(W10b{}, "from main", crlf));   // not the serving context: queued
    CHECK(Script<11>::taken().empty());
    dispatching<HostPlatform> = 2;
    CHECK(print(W10b{}, "from the console", crlf));   // served, the queued line first
    CHECK(Script<11>::taken() == "from main\r\nfrom the console\r\n");
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 11. the panic drain
// =============================================================================

using W11 = LineWriter<Script<12, 48>, HostPlatform>;

TEST_CASE("drain_to_port writes the complete lines, then the open chain") {
    fresh_host();
    using T = Script<12, 48>;
    T::reset();
    W11::init();
    dispatching<HostPlatform> = 0;
    CHECK(print(W11{}, "a line of forty bytes to fill the ring..", crlf));
    CHECK(print(W11{}, "second", crlf));
    CHECK(print(W11{}, "third", crlf));
    serve<W11>();
    CHECK(T::taken() == "a line of forty bytes to fill the ring..\r\n");   // the others wait
    dispatching<HostPlatform> = 1;
    CHECK(print(W11{}, "open ", 7));
    serve<W11>();
    W11::drain_to_port([] { T::drain(); });
    CHECK(T::taken() == "a line of forty bytes to fill the ring..\r\nsecond\r\nthird\r\nopen 7\r\n");
    dispatching<HostPlatform> = no_dispatch;
}

struct Escape {};
struct ThrowingReporter {
    static void report(PanicCode, uint8_t) { throw Escape{}; }
};

using W11b = LineWriter<Script<17, 48>, HostPlatform>;
void pump17() { Script<17, 48>::drain(); }

TEST_CASE("the panic reporter drains the writer before it hands over") {
    fresh_host();
    using T = Script<17, 48>;
    T::reset();
    W11b::init();
    dispatching<HostPlatform> = 0;
    CHECK(print(W11b{}, "a line of forty bytes to fill the ring..", crlf));
    CHECK(print(W11b{}, "queued before the panic", crlf));
    serve<W11b>();
    bool escaped = false;
    try {
        panic<HostPlatform, LineWriterReporter<W11b, &pump17, ThrowingReporter>>(PanicCode::assert_failed, 9);
    } catch (Escape&) {
        escaped = true;
    }
    CHECK(escaped);
    CHECK(T::taken() == "a line of forty bytes to fill the ring..\r\nqueued before the panic\r\n");
    CHECK(HostPlatform::panic_record().context == 9u);
    dispatching<HostPlatform> = no_dispatch;
}

// =============================================================================
// 12. the debug checks
// =============================================================================

using W12 = LineWriter<Script<13>, HostPlatform>;
uint8_t misuse_seen = 0;

TEST_CASE("debug: a direct write to a claimed transport is a misuse, and the default is a panic") {
    static_assert(debug_checks, "the host tests compile the debug checks");
    fresh_host();
    Script<13>::reset();
    CHECK(Script<13>::write_byte('a'));   // not claimed: anyone writes
    W12::init();
    misuse_seen = 0;
    W12::on_misuse = [](uint8_t why) { misuse_seen = why; };
    CHECK(print(W12{}, "the writer", crlf));
    CHECK(misuse_seen == 0u);
    (void)Script<13>::write_byte('x');
    CHECK(misuse_seen == W12::misuse_direct_write);

    // The panic itself, escaped through a reporter that throws: the
    // breadcrumb names the misuse.
    W12::on_misuse = [](uint8_t why) { panic<HostPlatform, ThrowingReporter>(PanicCode::assert_failed, why); };
    bool escaped = false;
    try {
        (void)Script<13>::write_byte('y');
    } catch (Escape&) {
        escaped = true;
    }
    CHECK(escaped);
    CHECK(HostPlatform::panic_record().magic == panic_magic);
    CHECK(HostPlatform::panic_record().code == static_cast<uint8_t>(PanicCode::assert_failed));
    CHECK(HostPlatform::panic_record().context == W12::misuse_direct_write);
    W12::on_misuse = &W12::panic_on_misuse;
    WriterClaim<Script<13>>::claimed = false;
}

TEST_CASE("debug: the PL011 transport's own write verbs check the claim") {
    fresh_host();
    SimPl011::reset_all();
    constexpr Clock clock{};
    REQUIRE(Port::init(clock, 115200));
    W1::init();
    misuse_seen = 0;
    W1::on_misuse = [](uint8_t why) { misuse_seen = why; };
    CHECK(print(W1{}, "through the writer", crlf));
    CHECK(misuse_seen == 0u);
    (void)Port::write_byte('x');
    CHECK(misuse_seen == W1::misuse_direct_write);
    misuse_seen = 0;
    const uint8_t run[] = {'a', 'b'};
    (void)Port::write_bulk(run);
    CHECK(misuse_seen == W1::misuse_direct_write);
    W1::on_misuse = &W1::panic_on_misuse;
    WriterClaim<Port>::claimed = false;
    (void)pl011_wire<W1>();
    serve<W1>();
}

/// A host platform that tells a stack: the region a test names.
struct StackPlatform : HostPlatform {
    static inline std::span<const uint8_t> region{};
    static std::span<const uint8_t> stack_span() { return region; }
};

using W13 = LineWriter<Script<14>, StackPlatform>;

TEST_CASE("debug: a text on the stack is a misuse where the platform tells its stack") {
    HostPlatform::reset();
    dispatching<StackPlatform> = no_dispatch;
    Script<14>::reset();
    W13::init();
    misuse_seen = 0;
    W13::on_misuse = [](uint8_t why) { misuse_seen = why; };
    char buffer[16] = "on the stack";
    StackPlatform::region = std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(buffer), sizeof buffer);
    CHECK(print(W13{}, "a literal", crlf));
    CHECK(misuse_seen == 0u);
    CHECK(print(W13{}, static_cast<const char*>(buffer), crlf));
    CHECK(misuse_seen == W13::misuse_stack_text);
    W13::on_misuse = &W13::panic_on_misuse;
    StackPlatform::region = {};
    WriterClaim<Script<14>>::claimed = false;
}

// =============================================================================
// the bench grammar through a writer
// =============================================================================

#include "util/testbench.hpp"

namespace {

using WB = LineWriter<Script<15>, HostPlatform, 8, 64>;
TestBench<Capture> over_transport;
TestBench<WB> through_writer;

template <auto& bench, typename S>
void letter_one() {
    print(S{}, "  a measurement ", 17u, " cycles", crlf);
    bench.verdict("the first thing holds", true);
    bench.verdict("the second ", "with a suffix", false);
}

template <auto& bench, typename S>
void letter_two() {
    print(S{}, "  composed:");
    for (uint32_t i = 0; i < 3u; ++i) {
        print(S{}, ' ', i);
    }
    print(S{}, crlf);
    bench.verdict("the second letter", true);
}

}  // namespace

TEST_CASE("the bench grammar through a writer is, byte for byte, the bench over the transport") {
    fresh_host();
    Script<15>::reset();
    Script<15>::drain_on_poll = true;   // the transmitter runs while the bench waits
    WB::init();
    Capture::text.clear();
    over_transport.letter('a', "the first letter", &letter_one<over_transport, Capture>);
    over_transport.letter('b', "the second letter", &letter_two<over_transport, Capture>);
    over_transport.letter('v', "by name only", &letter_two<over_transport, Capture>, false);
    through_writer.letter('a', "the first letter", &letter_one<through_writer, WB>);
    through_writer.letter('b', "the second letter", &letter_two<through_writer, WB>);
    through_writer.letter('v', "by name only", &letter_two<through_writer, WB>, false);
    for (TestBench<Capture>* b : {&over_transport}) {
        b->menu();
        b->prompt();
        (void)b->handle('z');
        b->prompt();
        (void)b->handle('a');
        b->prompt();
    }
    through_writer.menu();
    through_writer.prompt();
    (void)through_writer.handle('z');
    through_writer.prompt();
    (void)through_writer.handle('a');
    through_writer.prompt();
    CHECK(Script<15>::taken() == Capture::text);
    CHECK(WB::lost() == 0u);
    CHECK(Capture::text.find("ALL: 2 pass, 1 fail\r\n> ") != std::string::npos);
}

namespace {
using WL = LineWriter<Script<16>, HostPlatform, 8, 64>;
TestBench<WL> lossy;
void loses_one() {
    const std::string long_text(300, 'x');
    (void)print(WL{}, long_text.c_str(), crlf);
    lossy.verdict("a line was refused", WL::lost() == 1u);
}
}  // namespace

TEST_CASE("through a writer, the tallies carry the lines the writer lost") {
    fresh_host();
    Script<16>::reset();
    Script<16>::drain_on_poll = true;
    WL::init();
    lossy.letter('a', "a letter that loses a line", &loses_one);
    (void)lossy.handle('z');
    const std::string out = Script<16>::taken();
    CHECK(out.find("lost 1\r\n") != std::string::npos);
    CHECK(out.find("  -> 1 pass, 0 fail, 1 lost, 0 torn\r\n\r\n") != std::string::npos);
    CHECK(out.find("ALL: 1 pass, 0 fail, 1 lost, 0 torn\r\n") != std::string::npos);
}
