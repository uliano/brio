// Clock family smoke TU: the HPRE divider table, the two roots the task
// implements, the wait-state rule, and the init() of a rate from each.
#include "ch32v00x/clock.hpp"

using namespace brio;

// RM 3.4.2: codes 0..7 divide by 1..8, codes 8..15 by 2..256.
static_assert(hpre_divider(0) == 1 && hpre_divider(2) == 3 && hpre_divider(7) == 8);
static_assert(hpre_divider(8) == 2 && hpre_divider(11) == 16 && hpre_divider(15) == 256);
// The low code wins where the halves overlap, so a divider has one spelling.
static_assert(hpre_for(24'000'000, 12'000'000) == 1);
static_assert(hpre_for(24'000'000, 8'000'000) == 2);     // the reset state
static_assert(hpre_for(24'000'000, 24'000'000) == 0);
static_assert(hpre_for(24'000'000, 1'500'000) == 11);    // /16: only the high half has it
static_assert(hpre_for(48'000'000, 48'000'000) == 0);
static_assert(hpre_for(48'000'000, 3'000'000) == 11);
static_assert(hpre_for(24'000'000, 10'000'000) == 0xFF); // no divider reaches it
static_assert(hpre_for(24'000'000, 0) == 0xFF);

// RM 18.3.1's three columns.
static_assert(flash_latency_for(8'000'000) == 0);
static_assert(flash_latency_for(15'000'000) == 0);
static_assert(flash_latency_for(24'000'000) == 1);
static_assert(flash_latency_for(48'000'000) == 2);

using Fast = Clock<ClockSource::pll, 48'000'000>;
using Full = Clock<ClockSource::internal, 24'000'000>;
using Reset = Clock<ClockSource::internal, 8'000'000>;
using Slow = Clock<ClockSource::pll, 3'000'000>;

static_assert(Fast::is_static && Fast::hz == 48'000'000 && Fast::pclk_hz == Fast::hz);
static_assert(Fast::sysclk_hz == 48'000'000 && Fast::hpre_code == 0);
static_assert(Full::sysclk_hz == 24'000'000 && Full::hpre_code == 0);
static_assert(Reset::hpre_code == 2);
static_assert(Slow::hpre_code == 11);
static_assert(clock_hz(Fast{}) == 48'000'000);

void clock_verbs() {
    (void)Fast::init();
    (void)Full::init();
    (void)Reset::init();
    (void)Slow::init();
}
