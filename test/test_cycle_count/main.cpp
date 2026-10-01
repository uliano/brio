// Host tests for util/cycle_count.hpp: ticker_consistent() and
// ticker_compose() against a timeline of a periodic counter and its
// handler, cycle by cycle - the counter's position, its pending flag, and
// a tick count the handler advances some cycles AFTER the restart - for
// both kinds of flag (raised at the period's last step, as a SysTick
// counting down to zero; raised at the restart) and for every handler
// latency the functions promise to cover.
// Run with: ctest --preset host (or ctest --preset host -R test_cycle_count)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <optional>

#include "util/cycle_count.hpp"

namespace {

using brio::ticker_compose;
using brio::ticker_consistent;
using brio::TickerSample;

// The ticker's loop body for one set of reads: the count when the sample
// is consistent, nothing when the ticker would read again.
std::optional<uint32_t> read_once(TickerSample s, uint32_t period) {
    if (ticker_consistent(s)) {
        return ticker_compose(s, period);
    }
    return std::nullopt;
}

enum class FlagAt { last_step, restart };

// What a ticker reads at true cycle c: the handler of the restart that
// ends period k runs `latency` cycles after that restart, counts the tick
// and clears the flag.
struct Instant {
    uint32_t ticks;
    bool pending;
    uint32_t elapsed;
};

Instant at(uint32_t c, uint32_t period, uint32_t latency, FlagAt kind) {
    Instant s{0, false, c % period};
    for (uint32_t k = 0; (k + 1u) * period <= c; ++k) {
        const uint32_t restart = (k + 1u) * period;
        const uint32_t handled = restart + latency;
        const uint32_t raised = kind == FlagAt::last_step ? restart - 1u : restart;
        if (c >= handled) {
            ++s.ticks;
        } else if (c >= raised) {
            s.pending = true;
        }
    }
    // The flag of the period that has not restarted yet, raised at its
    // last step.
    if (kind == FlagAt::last_step && c % period == period - 1u) {
        s.pending = true;
    }
    return s;
}

std::optional<uint32_t> read_at(uint32_t c, uint32_t period, uint32_t latency, FlagAt kind) {
    const Instant s = at(c, period, latency, kind);
    return read_once({s.ticks, s.pending, s.elapsed, s.pending, s.ticks}, period);
}

}  // namespace

TEST_CASE("every cycle of six periods reads as itself, for every latency under a period") {
    constexpr uint32_t period = 50;
    for (const FlagAt kind : {FlagAt::last_step, FlagAt::restart}) {
        for (uint32_t latency = 0; latency + 1u < period; ++latency) {
            for (uint32_t c = 0; c < 6u * period; ++c) {
                const auto r = read_at(c, period, latency, kind);
                REQUIRE(r.has_value());
                REQUIRE(*r == c);
            }
        }
    }
}

TEST_CASE("the naive composition goes back a period in the window - what the flag fixes") {
    // ticks * period + elapsed, the flag ignored: between the restart and
    // the handler it reads a whole period low. A check that cannot fail
    // proves nothing, so the timeline must show it.
    constexpr uint32_t period = 50;
    constexpr uint32_t latency = 7;
    uint32_t wrong = 0;
    for (uint32_t c = 0; c < 6u * period; ++c) {
        const Instant s = at(c, period, latency, FlagAt::restart);
        if (s.ticks * period + s.elapsed != c) {
            ++wrong;
        }
    }
    CHECK(wrong == 5u * latency);   // five restarts, `latency` cycles each
}

TEST_CASE("the edge: a flag at the last step is the old period's end, not the new one's start") {
    constexpr uint32_t period = 1000;
    // Position 999 with the flag raised, handler not yet run: still period 3.
    CHECK(read_once({3, true, 999, true, 3}, period) == 3u * period + 999u);
    // One step later the position is 0 with the flag standing: period 4.
    CHECK(read_once({3, true, 0, true, 3}, period) == 4u * period);
    // After the handler: the tick counted, the flag gone.
    CHECK(read_once({4, false, 5, false, 4}, period) == 4u * period + 5u);
}

TEST_CASE("reads that straddle the handler or a restart are refused") {
    CHECK_FALSE(ticker_consistent({3, false, 10, false, 4}));   // the handler ran
    CHECK_FALSE(ticker_consistent({3, false, 0, true, 3}));     // the restart landed
    CHECK_FALSE(ticker_consistent({3, true, 0, false, 3}));     // and was counted
    // Each piece alone decides: the counts agree but the flags do not, and
    // the other way round.
    CHECK_FALSE(ticker_consistent({7, true, 500, false, 7}));
    CHECK_FALSE(ticker_consistent({7, false, 500, false, 8}));
    CHECK(ticker_consistent({7, true, 500, true, 7}));
}

TEST_CASE("the count wraps at 2^32 and a difference across the wrap is exact") {
    constexpr uint32_t period = 48000;                       // 48 MHz at 1 kHz
    const uint32_t ticks = UINT32_MAX / period;              // the last whole period
    const auto a = read_once({ticks, false, 100, false, ticks}, period);
    const auto b = read_once({ticks + 1u, false, 200, false, ticks + 1u}, period);
    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    CHECK(static_cast<uint32_t>(*b - *a) == period + 100u);
}

TEST_CASE("the test and the composition answer at compile time") {
    static_assert(ticker_consistent({2, true, 3, true, 2}));
    static_assert(ticker_compose({2, true, 3, true, 2}, 10) == 33u);
    static_assert(!ticker_consistent({2, true, 3, false, 2}));
    CHECK(true);
}
