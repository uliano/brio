// Ticker family smoke TU: the kernel timebase on the core's own STK
// counter - every verb, the two rates a program may ask for, and the
// handler that clears the flag.
#include "ch32v203/clock.hpp"
#include "ch32v203/ticker.hpp"
#include "util/timestamp.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 48'000'000>;
using Slow = BasicTicker<100>;

static_assert(Ticker::ticks_per_second == 1000u);
static_assert(Slow::ticks_per_second == 100u);

// The counter and its compare are the core's, at the same address on
// every part of the family (QingKe V4 manual): sixty-four bits in two
// register pairs, where the CH32V00x's V2C has one of each.
static_assert(stk_ste == 1u && stk_stie == 2u && stk_stclk == 4u && stk_stre == 8u);

void ticker_verbs() {
    constexpr SysClock clock;
    (void)Ticker::init(clock);
    (void)Ticker::ticks();
    (void)Ticker::millis();
    (void)Ticker::secs();
    TimeStamp stamp{};
    Ticker::now(stamp);
    Ticker::advance(3);
    Ticker::pause();
    Ticker::resume();
    Ticker::rebase(24'000'000UL);
    Ticker::tick();
}

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { Ticker::tick(); }
