// Host tests for util/line_writer.hpp, the one writer of a transport,
// over two simulated transports: the PL011 of host/sim_pl011.hpp (the IP
// stratum's real transport over a chip made of RAM) and a scripted ring
// local to this suite whose room, transmitter and copy the test drives -
// its write_bulk() can run a preemption inside the copy and reads the
// platform's mask depth there. The preempting cases run under Rubato on
// the host's preemptive platform (kernel/rubato.hpp, host/platform.hpp);
// the others name the object being dispatched by hand. The bytes a
// print() to the writer puts on the wire against the bytes print() writes
// to a capture; lines in pieces with a higher object's line between them;
// a preemption inside the holder's copy and two lines relayed in their
// completion order; no copy under the mask; the refusals (a line over its
// stage, no room, the pool exhausted) and the "lost N" mark; the waiting
// source; the panic drain's order; the peak; a handler's line; the bench
// grammar through the writer and its tally; and the debug claim. The two
// refusals that must not compile are this suite's neg/ TUs, compiled by
// ctest (test/CMakeLists.txt).
// Run with: ctest --preset host (or ctest --preset host -R test_line_writer)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <variant>

#include "host/platform.hpp"
#include "host/sim_pl011.hpp"
#include "kernel/active_object.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/panic.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "pl011/uart.hpp"
#include "util/line_writer.hpp"
#include "util/print.hpp"

using namespace brio;

namespace {

// ---- the two transports ----------------------------------------------------

/// Every scripted copy so far, and whether one ran with the mask held.
uint32_t copies_seen = 0;
bool masked_copy_seen = false;

/// A ring of `cap` bytes whose transmitter is the test: shift() moves the
/// oldest bytes to the wire. With drain_on_poll, every tx_room() moves one
/// byte first - the transmitter running while a waiting source polls.
/// `on_copy` runs once, inside the next write_bulk(), before it copies:
/// a preemption landing in the publisher's copy.
template <int id, uint32_t cap = 255>
struct Script {
    static inline std::string ring;
    static inline std::string wire;
    static inline bool drain_on_poll = false;
    static inline uint32_t polls = 0;
    static inline void (*on_copy)() = nullptr;

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
        ++copies_seen;
        if (HostPlatform::CriticalSection::depth != 0u) {
            masked_copy_seen = true;
        }
        if (on_copy != nullptr) {
            void (*const f)() = on_copy;
            on_copy = nullptr;
            f();
        }
        const size_t room = cap - ring.size();
        const size_t take = run.size() < room ? run.size() : room;
        ring.append(reinterpret_cast<const char*>(run.data()), take);
        return static_cast<uint32_t>(take);
    }

    static void shift(size_t n) {
        n = n < ring.size() ? n : ring.size();
        wire += ring.substr(0, n);
        ring.erase(0, n);
    }
    static void drain() { shift(ring.size()); }
    /// Everything the transport took, in order.
    static std::string taken() { return wire + ring; }
    static void reset() {
        ring.clear();
        wire.clear();
        drain_on_poll = false;
        polls = 0;
        on_copy = nullptr;
        WriterClaim<Script>::claimed = false;
        WriterClaim<Script>::writing = false;
    }
};
static_assert(BulkSink<Script<0>> && RoomSink<Script<0>>);

/// The PL011 transport over the chip made of RAM.
using Clock = SimPl011Clock<125'000'000>;
constexpr SimPl011Pins pins{.tx = 0, .rx = 1};
using Port = Pl011Transport<SimPl011, 0, pins, 64, 256, SimNoEngine, SimNoEngine>;

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

/// Run the PL011's wire to the end: the handler while its line is
/// raised, the wire taking the FIFO's oldest entry.
std::string pl011_wire() {
    std::string out;
    for (uint32_t guard = 0; guard < 100'000u; ++guard) {
        while (SimPl011::line_raised<0>()) {
            (void)Port::isr();
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

using RP = RubatoHostPlatform;

/// Run `f` as if inside the dispatch of the object at `index`
/// (no_dispatch: main context), the outer index put back after.
template <typename Plat = RP, typename F>
void as(uint8_t index, F f) {
    const uint8_t outer = dispatching<Plat>;
    dispatching<Plat> = index;
    f();
    dispatching<Plat> = outer;
}

/// Run `f` as an interrupt handler: the platform says so while it runs.
template <typename F>
void as_handler(F f) {
    HostPlatform::in_isr = true;
    f();
    HostPlatform::in_isr = false;
}

/// An interrupt over whatever runs, under Rubato: `f` with the platform
/// in a handler, then the scheduler at the handler's exit when it asked.
void rubato_isr(void (*f)()) {
    RP::in_isr = true;
    f();
    RP::in_isr = false;
    if (RP::pending && RP::scheduler != nullptr) {
        RP::pending = false;
        (void)RP::scheduler();
    }
}

// ---- a Rubato pack: four objects whose dispatch runs what the case sets ----

struct Go {};

template <uint8_t id>
struct Obj {
    using Event = std::variant<Go>;
    static inline EventQueue<Event, 4, RP> queue;
    static inline void (*action)() = nullptr;
    static void init() {}
    static void dispatch(const Event&) {
        if (action != nullptr) {
            action();
        }
    }
};

using High = Obj<0>;
using Mid = Obj<1>;
using Low = Obj<2>;
using Con = Obj<3>;   // the console: the object whose lines may wait
constexpr uint8_t con_index = 3;

using K = Rubato<RP, High, Mid, Low, Con>;

constexpr LineWriterOptions suite_opts{.max_sources = 8, .wait_from = con_index};

void fresh_kernel() {
    RP::reset();
    dispatching<RP> = no_dispatch;
    High::action = Mid::action = Low::action = Con::action = nullptr;
    RP::scheduler = &K::activate;
    K::init_all();
}

void fresh_host() {
    RP::reset();
    dispatching<RP> = no_dispatch;
    dispatching<HostPlatform> = no_dispatch;
}

// Twenty prints over every type print.hpp prints, each ending its line.
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
// the bytes
// =============================================================================

using W0 = LineWriter<Port, HostPlatform>;

TEST_CASE("the bytes on the wire are the bytes print() writes to a transport") {
    fresh_host();
    SimPl011::reset_all();
    constexpr Clock clock{};
    REQUIRE(Port::init(clock, 115200));
    W0::init();
    uint32_t lines = 0;
#define ONE(...)                                                                               \
    {                                                                                          \
        const std::string want = printed(__VA_ARGS__);                                         \
        print(W0{}, __VA_ARGS__);                                                              \
        CHECK(pl011_wire() == want);                                                           \
        ++lines;                                                                               \
    }
    LINES(ONE)
#undef ONE
    CHECK(lines == 20u);
    CHECK(W0::lost() == 0u);
    WriterClaim<Port>::claimed = false;
}

// =============================================================================
// 1. a line in pieces, a higher object's whole line between two pieces
// =============================================================================

using T1 = Script<1>;
using W1 = LineWriter<T1, RP, suite_opts>;

TEST_CASE("pieces from the console, a higher object's line between them: the higher first, both whole") {
    fresh_kernel();
    T1::reset();
    W1::init();
    High::action = [] { print(W1{}, "HIGH ", 42u, " whole", crlf); };
    Con::action = [] {
        print(W1{}, "CONSOLE ", 0u, ": pieces");
        rubato_isr([] { post<High>(Go{}); });   // an interrupt wakes High: road 1
        CHECK(T1::taken() == "HIGH 42 whole\r\n");   // High's line, whole, at once
        print(W1{}, ", piece ", 1u);
        print(W1{}, " :end", crlf);
    };
    post<Con>(Go{});
    CHECK(T1::taken() == "HIGH 42 whole\r\nCONSOLE 0: pieces, piece 1 :end\r\n");
    CHECK(W1::lost() == 0u);
}

// =============================================================================
// 2. a preemption inside the holder's copy
// =============================================================================

using T2 = Script<2>;
using W2 = LineWriter<T2, RP, suite_opts>;

TEST_CASE("a line completed during the holder's copy pends and follows the holder's") {
    fresh_kernel();
    T2::reset();
    W2::init();
    High::action = [] {
        print(W2{}, "H inside the copy", crlf);
        CHECK(T2::taken().empty());   // pending: the console holds the lock
    };
    Con::action = [] {
        T2::on_copy = [] { rubato_isr([] { post<High>(Go{}); }); };
        print(W2{}, "console line", crlf);
    };
    post<Con>(Go{});
    CHECK(T2::taken() == "console line\r\nH inside the copy\r\n");
    CHECK(W2::lost() == 0u);
}

// =============================================================================
// 3. two lines completed during one copy, relayed in completion order
// =============================================================================

using T3 = Script<3>;
// Three small stages: the holder's, and one for each line that pends.
using W3 = LineWriter<T3, RP, LineWriterOptions{.max_sources = 8, .wait_from = con_index, .small_stages = 3}>;

TEST_CASE("two lines completed during one copy - a higher object's, then a handler's - in that order") {
    fresh_kernel();
    T3::reset();
    W3::init();
    High::action = [] { print(W3{}, "first, High", crlf); };
    Low::action = [] {
        T3::on_copy = [] {
            rubato_isr([] { post<High>(Go{}); });
            rubato_isr([] { print(W3{}, "second, a handler", crlf); });
        };
        print(W3{}, "holder, Low", crlf);
    };
    post<Low>(Go{});
    CHECK(T3::taken() == "holder, Low\r\nfirst, High\r\nsecond, a handler\r\n");
    CHECK(W3::lost() == 0u);
}

// =============================================================================
// 4. no copy under the mask
// =============================================================================

using T4 = Script<4>;
using W4 = LineWriter<T4, RP, suite_opts>;

TEST_CASE("no copy into the transport ever runs inside a critical section") {
    fresh_kernel();
    T4::reset();
    W4::init();
    const uint32_t copies0 = copies_seen;
    High::action = [] { print(W4{}, "relayed", crlf); };
    Con::action = [] {
        as(7, [] { print(W4{}, "past the table", crlf); });   // a loss: the mark is owed
        T4::on_copy = [] { rubato_isr([] { post<High>(Go{}); }); };
        print(W4{}, "the console's line", crlf);
    };
    post<Con>(Go{});
    CHECK(T4::taken() == "lost 1\r\nthe console's line\r\nrelayed\r\n");
    CHECK(copies_seen - copies0 == 3u);   // the mark, the line, the relayed line
    CHECK_FALSE(masked_copy_seen);        // over this case and every one before it
    CHECK(HostPlatform::CriticalSection::depth == 0u);
}

// =============================================================================
// 5. a line over its stage
// =============================================================================

using T5 = Script<5>;
using W5 = LineWriter<T5, RP, suite_opts>;

TEST_CASE("a line that outgrows its stage is lost whole, counted once; the next line is fine") {
    fresh_host();
    T5::reset();
    W5::init();
    as(2, [] {
        for (int i = 0; i < 20; ++i) {
            print(W5{}, "0123456789");   // 200 bytes into a stage of 96
        }
        print(W5{}, crlf);
        CHECK(T5::taken().empty());
        CHECK(W5::lost() == 1u);
        print(W5{}, "after", crlf);
    });
    CHECK(T5::taken() == "lost 1\r\nafter\r\n");
    CHECK(W5::lost() == 1u);
    // A C string that crosses the stage's end is cut where it ends, and
    // the line refused at its end all the same.
    as(2, [] {
        const std::string long_text(97, 'x');
        print(W5{}, long_text.c_str(), crlf);
    });
    CHECK(W5::lost() == 2u);
    CHECK(T5::taken() == "lost 1\r\nafter\r\n");
}

// =============================================================================
// 6. no room for a source that may not wait
// =============================================================================

using T6 = Script<6, 32>;
using W6 = LineWriter<T6, RP, LineWriterOptions{.max_sources = 8, .wait_from = con_index, .long_size = 32,
                                                .small_size = 32}>;

TEST_CASE("no room for a source that may not wait: the line refused whole, nothing written, counted") {
    fresh_host();
    T6::reset();
    W6::init();
    T6::ring.assign(28, '.');   // four bytes of room
    as(2, [] { print(W6{}, "ten bytes", crlf); });   // 11 bytes
    CHECK(T6::taken() == std::string(28, '.'));
    CHECK(W6::lost() == 1u);
    T6::drain();
    T6::wire.clear();
    as(2, [] { print(W6{}, "next", crlf); });
    CHECK(T6::taken() == "lost 1\r\nnext\r\n");
}

// =============================================================================
// 7. the pool exhausted
// =============================================================================

using T7 = Script<7>;
using W7 = LineWriter<T7, RP, suite_opts>;

TEST_CASE("the pool exhausted: the third nested line refused whole, the two others whole") {
    fresh_host();
    T7::reset();
    W7::init();
    as(2, [] {
        print(W7{}, "L open");
        as(1, [] {
            print(W7{}, "M open");
            as(0, [] {
                print(W7{}, "H refused");   // both small stages taken; the long one is not H's
                print(W7{}, " and its rest", crlf);
                print(W7{}, "H next", crlf);   // a new line: still none free
            });
            print(W7{}, crlf);
        });
        print(W7{}, crlf);
    });
    CHECK(T7::taken() == "lost 2\r\nM open\r\nL open\r\n");
    CHECK(W7::lost() == 2u);
}

// =============================================================================
// 8. a waiting source on a full ring
// =============================================================================

using T8 = Script<8, 40>;
using W8 = LineWriter<T8, RP, LineWriterOptions{.max_sources = 8, .wait_from = con_index, .long_size = 40,
                                                .small_size = 40}>;

TEST_CASE("a waiting source with the ring full waits, and its line is written whole") {
    fresh_host();
    T8::reset();
    W8::init();
    T8::drain_on_poll = true;   // the transmitter runs while the source polls
    const uint32_t polls0 = T8::polls;
    as(con_index, [] {
        print(W8{}, "a line of thirty-two bytes .....", crlf);
        print(W8{}, "a line of thirty-two bytes ,,,,,", crlf);
    });
    as(no_dispatch, [] { print(W8{}, "main context waits too ..........", crlf); });
    CHECK(T8::taken() ==
          "a line of thirty-two bytes .....\r\na line of thirty-two bytes ,,,,,\r\nmain context waits too ..........\r\n");
    CHECK(T8::polls - polls0 > 3u);   // it did wait
    CHECK(W8::lost() == 0u);
    // A source that may not wait, the same ring full: refused.
    T8::drain_on_poll = false;
    T8::ring.assign(30, '.');
    as(2, [] { print(W8{}, "a line of thirty-two bytes '''''", crlf); });
    CHECK(W8::lost() == 1u);
}

// =============================================================================
// 9. the "lost N" mark
// =============================================================================

using T9 = Script<9, 24>;
using W9 = LineWriter<T9, RP, LineWriterOptions{.max_sources = 8, .wait_from = con_index, .long_size = 24,
                                                .small_size = 24}>;

TEST_CASE("the mark comes out once, before the next published line, with the losses since the last") {
    fresh_host();
    T9::reset();
    W9::init();
    for (int i = 0; i < 3; ++i) {
        as(7, [] { print(W9{}, "refused: past the table", crlf); });
    }
    CHECK(W9::lost() == 3u);
    CHECK(T9::taken().empty());
    // A source that may not wait, room for its line and not for both: the
    // line goes, the mark stays owed.
    T9::ring.assign(16, '.');
    as(2, [] { print(W9{}, "after", crlf); });
    CHECK(T9::taken() == std::string(16, '.') + "after\r\n");
    T9::drain();
    T9::wire.clear();
    as(2, [] { print(W9{}, "again", crlf); });
    CHECK(T9::taken() == "lost 3\r\nagain\r\n");
    T9::drain();
    as(2, [] { print(W9{}, "and once only", crlf); });
    CHECK(T9::taken() == "lost 3\r\nagain\r\nand once only\r\n");
    // The waiting source waits for the mark and the line.
    as(7, [] { print(W9{}, "x", crlf); });
    T9::drain();
    T9::wire.clear();
    T9::ring.assign(20, '.');
    T9::drain_on_poll = true;
    as(con_index, [] { print(W9{}, "console", crlf); });
    T9::drain_on_poll = false;
    CHECK(T9::taken() == std::string(20, '.') + "lost 1\r\nconsole\r\n");
}

// =============================================================================
// 10. the panic drain
// =============================================================================

using T10 = Script<10>;
using W10 = LineWriter<T10, RP, LineWriterOptions{.max_sources = 8, .wait_from = con_index, .small_stages = 3}>;
struct Escape {};

TEST_CASE("the panic drain writes the publishing stage, then the pending ones, then the open ones") {
    fresh_host();
    T10::reset();
    W10::init();
    as(2, [] { print(W10{}, "an open piece"); });
    bool escaped = false;
    try {
        as(con_index, [] {
            T10::on_copy = [] {
                as(1, [] { print(W10{}, "a pending line", crlf); });
                as(0, [] { print(W10{}, "a later one", crlf); });
                W10::drain_to_port([] {});   // the panic, inside the console's copy
                throw Escape{};
            };
            print(W10{}, "the line being published", crlf);
        });
    } catch (Escape&) {
        escaped = true;
    }
    dispatching<RP> = no_dispatch;
    CHECK(escaped);
    CHECK(T10::taken() == "the line being published\r\na pending line\r\na later one\r\nan open piece");
    // The state is consumed: every source starts afresh.
    T10::drain();
    T10::wire.clear();
    as(2, [] { print(W10{}, "fresh", crlf); });
    CHECK(T10::taken() == "fresh\r\n");
}

struct ThrowingReporter {
    static void report(PanicCode, uint8_t) { throw Escape{}; }
};

using T10b = Script<11, 48>;
using W10b = LineWriter<T10b, RP, LineWriterOptions{.max_sources = 8, .long_size = 48, .small_size = 48}>;
void pump11() { T10b::drain(); }

TEST_CASE("the panic reporter drains the writer, pumping the ring, before it hands over") {
    fresh_host();
    T10b::reset();
    W10b::init();
    T10b::ring.assign(40, '.');
    as(2, [] { print(W10b{}, "open before the panic: twenty-nine"); });
    bool escaped = false;
    try {
        panic<RP, LineWriterReporter<W10b, &pump11, ThrowingReporter>>(PanicCode::assert_failed, 9);
    } catch (Escape&) {
        escaped = true;
    }
    CHECK(escaped);
    CHECK(T10b::taken() == std::string(40, '.') + "open before the panic: twenty-nine");
    CHECK(RP::panic_record().context == 9u);
    // The drain released the claim: a reporter that prints to the
    // transport after it is no misuse.
    CHECK_FALSE(WriterClaim<T10b>::claimed);
}

// =============================================================================
// 11. the peak
// =============================================================================

using T11 = Script<12>;
using W11 = LineWriter<T11, RP, LineWriterOptions{.max_sources = 8, .wait_from = con_index, .peak = true}>;

TEST_CASE("peak_stages() reads the most stages held at once; the lines and bytes published") {
    fresh_host();
    T11::reset();
    W11::init();
    CHECK(W11::peak_stages() == 0u);
    as(2, [] { print(W11{}, "one", crlf); });
    CHECK(W11::peak_stages() == 1u);
    as(no_dispatch, [] {
        print(W11{}, "main ");
        as(2, [] {
            print(W11{}, "low ");
            as(1, [] { print(W11{}, "mid", crlf); });
            print(W11{}, crlf);
        });
        print(W11{}, crlf);
    });
    CHECK(T11::taken() == "one\r\nmid\r\nlow \r\nmain \r\n");
    CHECK(W11::peak_stages() == 3u);
    CHECK(W11::lines_written() == 4u);
    CHECK(W11::bytes_written() == 5u + 5u + 6u + 7u);
    CHECK(W11::lost() == 0u);
}

// =============================================================================
// 12. a handler's line
// =============================================================================

using T12 = Script<13>;
using W12 = LineWriter<T12, RP, suite_opts>;

TEST_CASE("a handler's whole line between two pieces of an object's goes out first, both whole") {
    fresh_kernel();
    T12::reset();
    W12::init();
    Low::action = [] {
        print(W12{}, "object: ");
        rubato_isr([] { print(W12{}, "irq ", hex(0x55u), crlf); });
        CHECK(T12::taken() == "irq 0x55\r\n");
        print(W12{}, "done", crlf);
        rubato_isr([] { print(W12{}, "irq again", crlf); });
    };
    post<Low>(Go{});
    CHECK(T12::taken() == "irq 0x55\r\nobject: done\r\nirq again\r\n");
    CHECK(W12::lost() == 0u);
}

// =============================================================================
// the debug claim
// =============================================================================

using T13 = Script<14>;
using W13 = LineWriter<T13, HostPlatform>;
uint8_t misuse_seen = 0;

TEST_CASE("debug: a direct write to a claimed transport is a misuse, and the default is a panic") {
    static_assert(debug_checks, "the host tests compile the debug checks");
    fresh_host();
    T13::reset();
    CHECK(T13::write_byte('a'));   // not claimed: anyone writes
    T13::ring.clear();
    W13::init();
    misuse_seen = 0;
    W13::on_misuse = [](uint8_t why) { misuse_seen = why; };
    print(W13{}, "the writer", crlf);
    CHECK(misuse_seen == 0u);
    CHECK(T13::taken() == "the writer\r\n");
    (void)T13::write_byte('x');
    CHECK(misuse_seen == W13::misuse_direct_write);

    W13::on_misuse = [](uint8_t why) { panic<HostPlatform, ThrowingReporter>(PanicCode::assert_failed, why); };
    bool escaped = false;
    try {
        (void)T13::write_byte('y');
    } catch (Escape&) {
        escaped = true;
    }
    CHECK(escaped);
    CHECK(HostPlatform::panic_record().magic == panic_magic);
    CHECK(HostPlatform::panic_record().context == W13::misuse_direct_write);
    W13::on_misuse = &W13::panic_on_misuse;
    WriterClaim<T13>::claimed = false;
}

TEST_CASE("debug: the PL011 transport's own write verbs check the claim") {
    fresh_host();
    SimPl011::reset_all();
    constexpr Clock clock{};
    REQUIRE(Port::init(clock, 115200));
    W0::init();
    misuse_seen = 0;
    W0::on_misuse = [](uint8_t why) { misuse_seen = why; };
    print(W0{}, "through the writer", crlf);
    CHECK(misuse_seen == 0u);
    (void)Port::write_byte('x');
    CHECK(misuse_seen == W0::misuse_direct_write);
    misuse_seen = 0;
    const uint8_t run[] = {'a', 'b'};
    (void)Port::write_bulk(run);
    CHECK(misuse_seen == W0::misuse_direct_write);
    W0::on_misuse = &W0::panic_on_misuse;
    WriterClaim<Port>::claimed = false;
    (void)pl011_wire();
}

// =============================================================================
// the bench grammar through a writer
// =============================================================================

#include "util/testbench.hpp"

namespace {

using TB = Script<15>;
using WB = LineWriter<TB, HostPlatform>;
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
    TB::reset();
    TB::drain_on_poll = true;   // the transmitter runs while main context waits
    WB::init();
    Capture::text.clear();
    over_transport.letter('a', "the first letter", &letter_one<over_transport, Capture>);
    over_transport.letter('b', "the second letter", &letter_two<over_transport, Capture>);
    over_transport.letter('v', "by name only", &letter_two<over_transport, Capture>, false);
    through_writer.letter('a', "the first letter", &letter_one<through_writer, WB>);
    through_writer.letter('b', "the second letter", &letter_two<through_writer, WB>);
    through_writer.letter('v', "by name only", &letter_two<through_writer, WB>, false);
    over_transport.menu();
    over_transport.prompt();
    (void)over_transport.handle('z');
    over_transport.prompt();
    (void)over_transport.handle('a');
    over_transport.prompt();
    through_writer.menu();
    through_writer.prompt();
    (void)through_writer.handle('z');
    through_writer.prompt();
    (void)through_writer.handle('a');
    through_writer.prompt();
    CHECK(TB::taken() == Capture::text);
    CHECK(WB::lost() == 0u);
    CHECK(Capture::text.find("ALL: 2 pass, 1 fail\r\n> ") != std::string::npos);
    WriterClaim<TB>::claimed = false;
}

namespace {
using TL = Script<16>;
using WL = LineWriter<TL, HostPlatform>;
TestBench<WL> lossy;
void loses_one() {
    const std::string long_text(300, 'x');
    print(WL{}, long_text.c_str(), crlf);
    lossy.verdict("a line was refused", WL::lost() == 1u);
}
}  // namespace

TEST_CASE("through a writer, the tallies carry the lines the writer lost; the wire marks them") {
    fresh_host();
    TL::reset();
    TL::drain_on_poll = true;
    WL::init();
    lossy.letter('a', "a letter that loses a line", &loses_one);
    (void)lossy.handle('z');
    const std::string out = TL::taken();
    CHECK(out.find("lost 1\r\n  PASS  a line was refused\r\n") != std::string::npos);
    CHECK(out.find("  -> 1 pass, 0 fail, 1 lost\r\n\r\n") != std::string::npos);
    CHECK(out.find("ALL: 1 pass, 0 fail, 1 lost\r\n") != std::string::npos);
    CHECK(out.find("torn") == std::string::npos);
    WriterClaim<TL>::claimed = false;
}
