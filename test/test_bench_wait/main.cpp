// Host tests for util/test_wait.hpp, the waits a bench test suite shares,
// and for util/testbench.hpp's skip verb and the tally lines it changes.
// A fake ruler advances by a fixed step at every read, a fake platform's
// tick by one at every read, and a fake console turns idle after a set
// number of asks, so every verb's true path, false path, the condition
// asked once more after the budget, the counter's wrap and the saturation
// are driven exactly. Run with: ctest --preset host (or ctest --preset
// host -R test_bench_wait)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <regex>
#include <string>

#include "kernel/platform.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

namespace {

// ---- the fakes ----------------------------------------------------------------

/// A ruler at 48 MHz whose count moves by `step` cycles at every read:
/// time passes only when the code under test looks at it.
struct FakeRuler {
    static inline uint32_t count = 0;
    static inline uint32_t step = 1;
    static inline uint32_t reads = 0;
    static uint32_t now() {
        ++reads;
        count += step;
        return count;
    }
    static constexpr uint32_t hz() { return 48'000'000; }
    static void reset(uint32_t start, uint32_t s) {
        count = start;
        step = s;
        reads = 0;
    }
};
static_assert(brio::CycleRuler<FakeRuler>);

/// A ruler at 1 MHz: one cycle per microsecond, the smallest whole rate.
struct SlowRuler {
    static uint32_t now() { return 0; }
    static constexpr uint32_t hz() { return 1'000'000; }
};

/// A platform whose tick count moves by one at every read of now(), or
/// stands still when `frozen` is set (the witness's quiet case).
struct FakePlatform {
    struct CriticalSection {};
    static inline uint32_t ticks = 0;
    static inline bool frozen = false;
    static void idle() {}
    static void break_here() {}
    static uint32_t now() {
        if (!frozen) {
            ++ticks;
        }
        return ticks;
    }
    static brio::PanicRecord& panic_record() {
        static brio::PanicRecord record{};
        return record;
    }
    static constexpr uint32_t ticks_per_second = 1000;
    static constexpr unsigned atomic_width = 4;
    static void reset(uint32_t start, bool freeze) {
        ticks = start;
        frozen = freeze;
    }
};
static_assert(brio::Platform<FakePlatform>);

/// A console whose transmitter turns idle at the `idle_after`-th ask
/// (0: never).
struct FakeConsole {
    static inline uint32_t asks = 0;
    static inline uint32_t idle_after = 0;
    static bool tx_idle() {
        ++asks;
        return idle_after != 0u && asks >= idle_after;
    }
    static void reset(uint32_t after) {
        asks = 0;
        idle_after = after;
    }
};
static_assert(brio::TxIdleSink<FakeConsole>);

/// A condition true from its `true_from`-th call on (0: never), counting
/// its calls.
struct Condition {
    uint32_t calls = 0;
    uint32_t true_from = 0;
    bool operator()() {
        ++calls;
        return true_from != 0u && calls >= true_from;
    }
};

/// A budget the compiler cannot see as a constant: the verbs' compile-time
/// refusal of a long constant budget must not fire on the saturation tests.
uint32_t runtime(uint32_t v) {
    volatile uint32_t x = v;
    return x;
}

} // namespace

// ---- the conversion -------------------------------------------------------------

TEST_CASE("microseconds convert to cycles by the rate's whole cycles per microsecond") {
    static_assert(brio::us_cycles<FakeRuler>(10) == 480);
    static_assert(brio::us_cycles<FakeRuler>(0) == 0);
    static_assert(brio::us_cycles<SlowRuler>(1234) == 1234);
    CHECK(brio::us_cycles<FakeRuler>(runtime(1000)) == 48'000u);
}

TEST_CASE("a run-time budget past half the wrap saturates within it") {
    // 2^31 / 48 = 44739242 us, 2147483616 cycles: the most under 2^31.
    constexpr uint32_t max_us = brio::half_wrap / 48u;
    static_assert(brio::us_cycles<FakeRuler>(max_us) == max_us * 48u);
    static_assert(brio::us_cycles<FakeRuler>(max_us) <= brio::half_wrap);
    CHECK(brio::us_cycles<FakeRuler>(runtime(max_us + 1u)) == max_us * 48u);
    CHECK(brio::us_cycles<FakeRuler>(runtime(0xFFFF'FFFFu)) == max_us * 48u);
    CHECK(brio::us_cycles<SlowRuler>(runtime(0xFFFF'FFFFu)) == brio::half_wrap);
}

// ---- wait_until -----------------------------------------------------------------

TEST_CASE("wait_until: true as soon as the condition holds, one read per turn") {
    FakeRuler::reset(0, 1);
    Condition c{.true_from = 5};
    CHECK(brio::wait_until<FakeRuler>(c, 1000));
    CHECK(c.calls == 5);
    // The start read, then one read per turn that found the condition false.
    CHECK(FakeRuler::reads == 1u + 4u);
}

TEST_CASE("wait_until: false when the budget runs out with the condition false") {
    FakeRuler::reset(0, 100);
    Condition c{};                                     // never true
    CHECK_FALSE(brio::wait_until<FakeRuler>(c, 10));   // 480 cycles
    // Turns at 100, 200, ... 500 cycles: five in the loop, the fifth past
    // the budget, then the condition asked once more.
    CHECK(c.calls == 5u + 1u);
    CHECK(FakeRuler::count >= 480u);
}

TEST_CASE("wait_until: a condition that holds on the ask after the budget is no time-out") {
    FakeRuler::reset(0, 100);
    Condition c{.true_from = 6};                       // the ask once more
    CHECK(brio::wait_until<FakeRuler>(c, 10));
    Condition late{.true_from = 7};                    // one ask too late
    FakeRuler::reset(0, 100);
    CHECK_FALSE(brio::wait_until<FakeRuler>(late, 10));
}

TEST_CASE("wait_until: exact across the ruler's wrap") {
    FakeRuler::reset(0xFFFF'FF00u, 100);
    Condition c{};
    CHECK_FALSE(brio::wait_until<FakeRuler>(c, 10));
    CHECK(c.calls == 6u);                              // the same five turns, then the ask
    FakeRuler::reset(0xFFFF'FF00u, 100);
    Condition soon{.true_from = 3};
    CHECK(brio::wait_until<FakeRuler>(soon, 10));
}

TEST_CASE("wait_until: a zero budget still asks the condition") {
    FakeRuler::reset(0, 1);
    Condition yes{.true_from = 1};
    CHECK(brio::wait_until<FakeRuler>(yes, 0));
    Condition no{};
    CHECK_FALSE(brio::wait_until<FakeRuler>(no, 0));
    CHECK(no.calls == 2u);                             // the turn, and the ask after it
}

TEST_CASE("wait_until: a run-time budget past half the wrap waits the saturated budget") {
    FakeRuler::reset(0, 0x1000'0000u);                 // 2^28 cycles a read
    Condition c{};
    CHECK_FALSE(brio::wait_until<FakeRuler>(c, runtime(0xFFFF'FFFFu)));
    // 2147483616 cycles: eight turns of 2^28 reach it, then the ask.
    CHECK(c.calls == 8u + 1u);
}

// ---- cycles_until ---------------------------------------------------------------

TEST_CASE("cycles_until: the wait's length when the condition holds") {
    FakeRuler::reset(1000, 7);
    Condition c{.true_from = 4};
    const auto n = brio::cycles_until<FakeRuler>(c, 100);
    REQUIRE(n.has_value());
    // The start read at 1007, then one read per turn, the fourth at 1035.
    CHECK(*n == 28u);
    CHECK(c.calls == 4u);
}

TEST_CASE("cycles_until: nothing on a time-out, a length on the ask after it") {
    FakeRuler::reset(0, 100);
    Condition never{};
    CHECK_FALSE(brio::cycles_until<FakeRuler>(never, 10).has_value());
    CHECK(never.calls == 5u + 1u);

    FakeRuler::reset(0, 100);
    Condition last{.true_from = 6};
    const auto n = brio::cycles_until<FakeRuler>(last, 10);
    REQUIRE(n.has_value());
    CHECK(*n == 600u);                                 // the read after the ask
}

TEST_CASE("cycles_until: exact across the ruler's wrap") {
    FakeRuler::reset(0xFFFF'FFF0u, 8);
    Condition c{.true_from = 5};
    const auto n = brio::cycles_until<FakeRuler>(c, 100);
    REQUIRE(n.has_value());
    CHECK(*n == 40u);
}

// ---- spin_us --------------------------------------------------------------------

TEST_CASE("spin_us: at least the span, ended by the first read past it") {
    FakeRuler::reset(0, 10);
    brio::spin_us<FakeRuler>(2);                       // 96 cycles
    // Start at 10; reads at 20 .. 110: the span 100 is the first >= 96.
    CHECK(FakeRuler::reads == 1u + 10u);
    FakeRuler::reset(0xFFFF'FFF0u, 10);
    brio::spin_us<FakeRuler>(2);
    CHECK(FakeRuler::reads == 11u);
    FakeRuler::reset(0, 10);
    brio::spin_us<FakeRuler>(0);                       // one read of the span
    CHECK(FakeRuler::reads == 2u);
}

// ---- wait_ticks -----------------------------------------------------------------

TEST_CASE("wait_ticks: true when the condition holds before the ticks pass") {
    FakePlatform::reset(0, false);
    Condition c{.true_from = 3};
    CHECK(brio::wait_ticks<FakePlatform>(c, 10));
    CHECK(c.calls == 3u);
}

TEST_CASE("wait_ticks: false after more than the ticks, the condition asked once more") {
    FakePlatform::reset(0, false);
    Condition c{};
    CHECK_FALSE(brio::wait_ticks<FakePlatform>(c, 4));
    // Start at tick 1; turns read 2, 3, 4, 5, 6: the first more than 4
    // ticks on (at least four whole periods), then the ask.
    CHECK(c.calls == 5u + 1u);
    FakePlatform::reset(0, false);
    Condition last{.true_from = 6};
    CHECK(brio::wait_ticks<FakePlatform>(last, 4));
}

TEST_CASE("wait_ticks: exact across the tick counter's wrap, and saturated") {
    FakePlatform::reset(0xFFFF'FFFDu, false);
    Condition c{};
    CHECK_FALSE(brio::wait_ticks<FakePlatform>(c, 4));
    CHECK(c.calls == 6u);
    // A run-time budget past half the wrap is held to 2^31 - 1 ticks; a
    // condition that comes true is still seen at once.
    FakePlatform::reset(0, false);
    Condition soon{.true_from = 2};
    CHECK(brio::wait_ticks<FakePlatform>(soon, runtime(0xFFFF'FFFFu)));
}

// ---- console_drain --------------------------------------------------------------

TEST_CASE("console_drain: true once the console says idle, false on the budget") {
    FakeRuler::reset(0, 1);
    FakeConsole::reset(3);
    CHECK(brio::console_drain<FakeRuler, FakeConsole>());
    CHECK(FakeConsole::asks == 3u);

    FakeRuler::reset(0, 48'000);                       // a millisecond a read
    FakeConsole::reset(0);                             // a wedged console
    CHECK_FALSE(brio::console_drain<FakeRuler, FakeConsole>(5'000));
    CHECK(FakeConsole::asks == 5u + 1u);

    FakeRuler::reset(0, 48'000);                       // the default 200 ms
    FakeConsole::reset(0);
    CHECK_FALSE(brio::console_drain<FakeRuler, FakeConsole>());
    CHECK(FakeConsole::asks == 200u + 1u);
}

// ---- the witness ----------------------------------------------------------------

TEST_CASE("TickWitness: quiet with no tick, the count with some, restart") {
    FakePlatform::reset(100, true);
    brio::TickWitness<FakePlatform> w;
    CHECK(w.quiet());
    CHECK(w.ticks() == 0u);

    FakePlatform::ticks = 103;                         // three ticks landed
    CHECK_FALSE(w.quiet());
    CHECK(w.ticks() == 3u);
    w.restart();
    CHECK(w.quiet());

    FakePlatform::reset(0xFFFF'FFFFu, true);           // across the wrap
    brio::TickWitness<FakePlatform> across;
    FakePlatform::ticks = 1;
    CHECK(across.ticks() == 2u);
}

// ---- spans ----------------------------------------------------------------------

TEST_CASE("since_clamped: a stale-low reading is zero, never four billion") {
    static_assert(brio::since_clamped(10, 5) == 5);
    static_assert(brio::since_clamped(5, 10) == 0);
    static_assert(brio::since_clamped(3, 0xFFFF'FFFEu) == 5);       // across the wrap
    static_assert(brio::since_clamped(0x7FFF'FFFFu, 0) == 0x7FFF'FFFFu);
    static_assert(brio::since_clamped(0x8000'0000u, 0) == 0);      // past 2^31: zero
    CHECK(brio::since_clamped(runtime(7), runtime(9)) == 0u);
}

TEST_CASE("span_fits: a narrow counter times a budget within half its wrap") {
    static_assert(brio::span_fits<16>(32768));
    static_assert(!brio::span_fits<16>(32769));
    static_assert(brio::span_fits<24>(0x80'0000u));
    static_assert(!brio::span_fits<24>(0x80'0001u));
    static_assert(brio::span_fits<32>(brio::half_wrap));
    static_assert(!brio::span_fits<32>(brio::half_wrap + 1u));
    static_assert(brio::span_fits<2>(2));
    static_assert(!brio::span_fits<2>(3));
    CHECK(brio::span_fits<16>(runtime(1000)));
}

// ---- the skip verb --------------------------------------------------------------

namespace {

/// The capture sink and the judge's own expression, as in test_testbench.
struct Capture {
    static inline std::string text;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
};
static_assert(brio::ByteSink<Capture>);

/// bin/brio's summary expression (cli/bench/common.py SUMMARY_RE) and its
/// skip field (SKIP_RE): what the tallies printed here must still match.
const std::regex summary_re(R"((\d+)\s+pass,\s*(\d+)\s+fail)");
const std::regex skip_re(R"((\d+)\s+pass,\s*(\d+)\s+fail(?:,\s*\d+\s+lost)?,\s*(\d+)\s+skipped)");

using Bench = brio::TestBench<Capture>;
Bench* active = nullptr;

void t_mixed() {
    active->verdict("ran and held", true);
    active->skip("needs the peer board");
    active->verdict("ran and failed", false);
    active->skip("needs a ", "second part");
}
void t_plain() { active->verdict("only verdicts", true); }
void t_skip_only() { active->skip("nothing this board can run"); }

bool contains(const std::string& text, const std::string& what) {
    return text.find(what) != std::string::npos;
}

} // namespace

TEST_CASE("skip: a SKIP line, counted apart, the tally carrying the count last") {
    Capture::text.clear();
    Bench b;
    active = &b;
    REQUIRE(b.letter('a', "mixed", t_mixed));
    const auto t = b.handle('a');
    REQUIRE(t.has_value());
    CHECK(t->pass == 1);
    CHECK(t->fail == 1);
    CHECK(b.skipped() == 2);
    CHECK(Capture::text ==
          "-- mixed\r\n"
          "  PASS  ran and held\r\n"
          "  SKIP  needs the peer board\r\n"
          "  FAIL  ran and failed\r\n"
          "  SKIP  needs a second part\r\n"
          "  -> 1 pass, 1 fail, 2 skipped\r\n"
          "\r\n");
    std::smatch m;
    REQUIRE(std::regex_search(Capture::text, m, summary_re));
    CHECK(m[1] == "1");
    CHECK(m[2] == "1");
    REQUIRE(std::regex_search(Capture::text, m, skip_re));
    CHECK(m[3] == "2");
}

TEST_CASE("skip: a letter with none prints the tally it always printed") {
    Capture::text.clear();
    Bench b;
    active = &b;
    REQUIRE(b.letter('a', "plain", t_plain));
    const auto t = b.handle('a');
    REQUIRE(t.has_value());
    CHECK(b.skipped() == 0);
    CHECK(Capture::text == "-- plain\r\n  PASS  only verdicts\r\n  -> 1 pass, 0 fail\r\n\r\n");
}

TEST_CASE("skip: the ALL line sums the letters' skips, and a run with none keeps its line") {
    Capture::text.clear();
    Bench b;
    active = &b;
    REQUIRE(b.letter('a', "mixed", t_mixed));
    REQUIRE(b.letter('b', "plain", t_plain));
    REQUIRE(b.letter('c', "skip only", t_skip_only));
    const auto t = b.handle('z');
    REQUIRE(t.has_value());
    CHECK(t->pass == 2);
    CHECK(t->fail == 1);
    CHECK(b.skipped() == 1);                           // the last letter's
    CHECK(contains(Capture::text, "  -> 1 pass, 1 fail, 2 skipped\r\n"));
    CHECK(contains(Capture::text, "  -> 1 pass, 0 fail\r\n"));
    CHECK(contains(Capture::text, "  -> 0 pass, 0 fail, 1 skipped\r\n"));
    CHECK(contains(Capture::text, "ALL: 2 pass, 1 fail, 3 skipped\r\n"));

    Capture::text.clear();
    Bench quiet;
    active = &quiet;
    REQUIRE(quiet.letter('b', "plain", t_plain));
    (void)quiet.handle('z');
    CHECK(contains(Capture::text, "ALL: 1 pass, 0 fail\r\n"));
}

TEST_CASE("skip: the count is cleared with the tally and restored with it") {
    Capture::text.clear();
    Bench b;
    active = &b;
    REQUIRE(b.letter('a', "mixed", t_mixed));
    REQUIRE(b.letter('b', "plain", t_plain));
    (void)b.handle('a');
    CHECK(b.skipped() == 2);
    (void)b.handle('b');                               // a new letter starts at zero
    CHECK(b.skipped() == 0);
    b.resume_tally(3, 1, 4);                           // banked across a reset
    CHECK(b.skipped() == 4);
    Capture::text.clear();
    b.end_letter();
    CHECK(Capture::text == "  -> 3 pass, 1 fail, 4 skipped\r\n\r\n");
    b.reset_tally();
    CHECK(b.skipped() == 0);
    b.resume_tally(2, 0);                              // the two-argument form
    CHECK(b.skipped() == 0);
}

TEST_CASE("skip: the letter's count saturates at 255") {
    Capture::text.clear();
    Bench b;
    active = &b;
    for (int i = 0; i < 300; ++i) {
        b.skip("again");
    }
    CHECK(b.skipped() == 255);
}

namespace {

/// A sink that loses lines (util/line_writer.hpp's surface the bench asks
/// for): the tally carries both counts, the lost lines first.
struct LossySink {
    static inline std::string text;
    static inline uint16_t lost_count = 0;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
    static uint16_t lost() { return lost_count; }
};
static_assert(brio::LosesLines<LossySink>);

using LossyBench = brio::TestBench<LossySink>;
LossyBench* lossy = nullptr;

void t_lossy() {
    lossy->verdict("held", true);
    lossy->skip("no peer");
    LossySink::lost_count = static_cast<uint16_t>(LossySink::lost_count + 2u);
}

} // namespace

TEST_CASE("skip: through a writer that lost lines, the skips follow the lost count") {
    LossySink::text.clear();
    LossySink::lost_count = 5;
    LossyBench b;
    lossy = &b;
    REQUIRE(b.letter('a', "lossy", t_lossy));
    (void)b.handle('z');
    CHECK(contains(LossySink::text, "  -> 1 pass, 0 fail, 2 lost, 1 skipped\r\n"));
    CHECK(contains(LossySink::text, "ALL: 1 pass, 0 fail, 2 lost, 1 skipped\r\n"));
    // The judge's loss expression (cli/bench/common.py LOSS_RE) still
    // reads the lost count where it always stood.
    const std::regex loss_re(R"((\d+)\s+pass,\s*(\d+)\s+fail,\s*(\d+)\s+lost)");
    std::smatch m;
    REQUIRE(std::regex_search(LossySink::text, m, loss_re));
    CHECK(m[3] == "2");
    REQUIRE(std::regex_search(LossySink::text, m, skip_re));
    CHECK(m[3] == "1");
}
