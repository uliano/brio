// Host tests for util/can.hpp: the classic frame's validity, the wire
// length of a frame, the bit-timing arithmetic and THE EXACT SEARCH under
// two controllers' limits - a bxCAN's and an M_CAN's nominal phase -
// against timings computed by hand, and the error state's folding.
// Run with: ctest --preset host (or ctest --preset host -R test_can)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>

#include "util/can.hpp"

using namespace brio;

namespace {

/// A bxCAN's registers: BRP ten bits, TS1 four, TS2 three, SJW two -
/// a bit of three to twenty-five quanta.
constexpr CanTimingLimits bxcan{1024, 16, 8, 4, 3, 25};

/// An M_CAN's nominal phase: NBRP nine bits, NTSEG1 eight, NTSEG2 and
/// NSJW seven - and the chapter's window of four to eighty-one quanta.
constexpr CanTimingLimits mcan_nominal{512, 255, 128, 128, 4, 81};

// Everything is constexpr: the core facts are locked at compile time.
static_assert(can_frame_valid(CanFrame{0x7FF, false, false, 8, {}, 0, 0}));
static_assert(!can_frame_valid(CanFrame{0x800, false, false, 0, {}, 0, 0}), "twelve bits standard");
static_assert(can_frame_valid(CanFrame{0x1FFFFFFF, true, false, 0, {}, 0, 0}));
static_assert(!can_frame_valid(CanFrame{0x20000000, true, false, 0, {}, 0, 0}));
static_assert(!can_frame_valid(CanFrame{0, false, false, 9, {}, 0, 0}), "eight bytes at most");

static_assert(can_frame_bits(0, false) == 44u && can_frame_bits(8, false) == 108u);
static_assert(can_frame_bits(8, true) == 128u, "64 + 8N with an extended identifier");

static_assert(can_timing_quanta(CanTiming{1, 12, 2, 2}) == 15u, "1 + TS1 + TS2");
static_assert(can_sample_point_of(CanTiming{1, 12, 2, 2}) == 866u, "13 of 15 quanta");
static_assert(can_sample_point_of(CanTiming{1, 13, 2, 2}) == 875u, "14 of 16 - exactly 87.5%");

static_assert(can_timing_limits_valid(bxcan) && can_timing_limits_valid(mcan_nominal));
static_assert(!can_timing_limits_valid(CanTimingLimits{1024, 16, 8, 4, 2, 25}), "a bit is three quanta at least");
static_assert(!can_timing_limits_valid(CanTimingLimits{1024, 16, 8, 4, 26, 25}), "the ceiling below the floor");

static_assert(can_timing_fits(CanTiming{1024, 16, 8, 4}, bxcan), "the widest bxCAN timing");
static_assert(!can_timing_fits(CanTiming{1025, 16, 8, 4}, bxcan), "BRP is ten bits there");
static_assert(!can_timing_fits(CanTiming{1, 17, 8, 4}, bxcan), "TS1 is four bits there");
static_assert(!can_timing_fits(CanTiming{1, 16, 9, 4}, bxcan), "TS2 is three bits there");
static_assert(!can_timing_fits(CanTiming{1, 16, 8, 5}, bxcan), "SJW is two bits there");
static_assert(!can_timing_fits(CanTiming{1, 16, 2, 3}, bxcan), "SJW may not exceed TS2");
static_assert(!can_timing_fits(CanTiming{}, bxcan), "an empty timing is not a timing");
static_assert(can_timing_fits(CanTiming{1, 40, 40, 40}, mcan_nominal), "an M_CAN bit of 81 quanta");
static_assert(!can_timing_fits(CanTiming{1, 41, 40, 40}, mcan_nominal), "82 quanta is past the window");

} // namespace

TEST_CASE("the exact search under bxCAN limits, against timings computed by hand") {
    // 72 MHz into 500 kbit/s is 144 clocks a bit: 16 quanta of 9 puts the
    // sample point at 14/16 = 87.5 % exactly, which no wider bit reaches.
    const CanTiming t72 = can_timing_search(72'000'000u, 500'000u, bxcan);
    CHECK(t72.brp == 9);
    CHECK(t72.ts1 == 13);
    CHECK(t72.ts2 == 2);
    CHECK(t72.sjw == 2);
    CHECK(can_bitrate_of(72'000'000u, t72) == 500'000u);
    CHECK(can_sample_point_of(t72) == 875u);

    // 45 MHz into 500 kbit/s is 90 clocks: 18 quanta reach 88.8 %, 15
    // quanta 86.6 % - closer - and nothing narrower beats that.
    const CanTiming t45 = can_timing_search(45'000'000u, 500'000u, bxcan);
    CHECK(t45.brp == 6);
    CHECK(t45.ts1 == 12);
    CHECK(t45.ts2 == 2);
    CHECK(t45.sjw == 2);
    CHECK(can_sample_point_of(t45) == 866u);

    // 45 MHz into 1 Mbit/s: 15 quanta of 3.
    const CanTiming t1m = can_timing_search(45'000'000u, 1'000'000u, bxcan);
    CHECK(t1m.brp == 3);
    CHECK(t1m.ts1 == 12);
    CHECK(t1m.ts2 == 2);
    CHECK(can_bitrate_of(45'000'000u, t1m) == 1'000'000u);

    // The sample point asked for moves the answer: 75 % from 45 MHz.
    const CanTiming t75 = can_timing_search(45'000'000u, 500'000u, bxcan, 750);
    CHECK(can_timing_fits(t75, bxcan));
    CHECK(can_sample_point_of(t75) <= 800u);
    CHECK(can_sample_point_of(t75) >= 700u);
}

TEST_CASE("an inexact rate, a zero and bad limits yield an empty timing") {
    CHECK(!can_timing_fits(can_timing_search(45'000'000u, 800'000u, bxcan), bxcan));   // 56.25 clocks a bit
    CHECK(can_timing_search(0u, 500'000u, bxcan).brp == 0);
    CHECK(can_timing_search(45'000'000u, 0u, bxcan).brp == 0);
    CHECK(can_timing_search(45'000'000u, 500'000u, CanTimingLimits{}).brp == 0);
}

TEST_CASE("the same search under an M_CAN's nominal limits") {
    // 48 MHz into 500 kbit/s is 96 clocks: the widest legal bit that
    // divides it is 48 quanta of 2, and 42 of 48 is 87.5 % exactly.
    const CanTiming t = can_timing_search(48'000'000u, 500'000u, mcan_nominal);
    CHECK(t.brp == 2);
    CHECK(t.ts1 == 41);
    CHECK(t.ts2 == 6);
    CHECK(t.sjw == 6);
    CHECK(can_timing_fits(t, mcan_nominal));
    CHECK(!can_timing_fits(t, bxcan));   // a bxCAN's TS1 stops at 16
    CHECK(can_bitrate_of(48'000'000u, t) == 500'000u);
}

TEST_CASE("the error state folds the deepest standing flag") {
    CHECK(can_error_state(false, false, false) == CanErrorState::active);
    CHECK(can_error_state(true, false, false) == CanErrorState::warning);
    CHECK(can_error_state(true, true, false) == CanErrorState::passive);
    CHECK(can_error_state(false, true, false) == CanErrorState::passive);
    CHECK(can_error_state(true, true, true) == CanErrorState::bus_off);
    CHECK(static_cast<uint8_t>(CanError::no_change) == 7);
    CHECK(static_cast<uint8_t>(CanError::bit_dominant) == 5);
}
