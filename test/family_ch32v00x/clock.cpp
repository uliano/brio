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

// The wait-state table, the part's: RM 18.3.1's three columns on the
// CH32V006, two on the CH32V003.
static_assert(flash_latency_for(8'000'000) == 0);
#if defined(CH32V006)
static_assert(flash_latency_for(15'000'000) == 0);
static_assert(flash_latency_for(24'000'000) == 1);
static_assert(flash_latency_for(48'000'000) == 2);
#else
static_assert(flash_latency_for(24'000'000) == 0);
static_assert(flash_latency_for(48'000'000) == 1);
#endif

using Fast = Clock<ClockSource::pll, 48'000'000>;
// The HSE: a crystal as SYSCLK, an external clock in bypass, the PLL
// doubling a crystal - the rate named as the third parameter.
using Xtal = Clock<ClockSource::crystal, 24'000'000, 24'000'000>;
using XtalHalf = Clock<ClockSource::crystal, 12'000'000, 24'000'000>;
using Ext = Clock<ClockSource::external, 25'000'000, 25'000'000>;
using PllXtal = Clock<ClockSource::pll, 48'000'000, 24'000'000>;
static_assert(Xtal::uses_hse && Ext::uses_hse && PllXtal::uses_hse && !Fast::uses_hse);
static_assert(Xtal::sysclk_hz == 24'000'000 && PllXtal::sysclk_hz == 48'000'000 && XtalHalf::hpre_code == 1);
static_assert(PllXtal::root_hz == 24'000'000 && Fast::root_hz == 24'000'000);
using Full = Clock<ClockSource::internal, 24'000'000>;
using Reset = Clock<ClockSource::internal, 8'000'000>;
using Slow = Clock<ClockSource::pll, 3'000'000>;

static_assert(Fast::is_static && Fast::hz == 48'000'000 && Fast::pclk_hz == Fast::hz);
static_assert(Fast::sysclk_hz == 48'000'000 && Fast::hpre_code == 0);
static_assert(Full::sysclk_hz == 24'000'000 && Full::hpre_code == 0);
static_assert(Reset::hpre_code == 2);
static_assert(Slow::hpre_code == 11);
static_assert(clock_hz(Fast{}) == 48'000'000);

void hse_verbs() {
    (void)Xtal::init();
    (void)XtalHalf::init();
    (void)Ext::init();
    (void)PllXtal::init();
    (void)PllXtal::restore();
    Rcc::hse(true); (void)Rcc::hse_on(); (void)Rcc::hse_ready();
    Rcc::hse_bypass(false); (void)Rcc::hse_bypass();
    Rcc::css(true); (void)Rcc::css(); (void)Rcc::css_failed(); Rcc::clear_css_failed();
    (void)Rcc::pll_from_hse();
    Rcc::monitor(false); (void)Rcc::monitor(); (void)Rcc::clock_failed(); Rcc::clear_clock_failed();
}

void clock_verbs() {
    (void)Fast::init();
    (void)Full::init();
    (void)Reset::init();
    (void)Slow::init();
}

// ---- the resource ----------------------------------------------------------
void rcc_verbs() {
    (void)Rcc::hsi_on();
    (void)Rcc::hsi_ready();
    Rcc::hsi(true);
    (void)Rcc::hsi_calibration();
    Rcc::hsi_trim(Rcc::hsi_trim());
    Rcc::lsi(true);
    (void)Rcc::lsi_on();
    (void)Rcc::lsi_ready();
    (void)Rcc::pll_on();
    (void)Rcc::pll_ready();
    (void)Rcc::sysclk_source();
    Rcc::hpre(Rcc::hpre_code());
    Rcc::mco(rcc_mco_sysclk);
    (void)Rcc::mco();
    Rcc::monitor(true);
    (void)Rcc::monitor();
    (void)Rcc::clock_failed();
    Rcc::clear_clock_failed();
    Rcc::failure_interrupt(false);
    Rcc::clock(Bus::pb2, rcc_pb2_tim1, true);
    (void)Rcc::clock(Bus::pb1, 1UL << 0);
    Rcc::reset(Bus::pb2, rcc_pb2_tim1);
}

// ---- the runtime regime ----------------------------------------------------
struct Follower {
    static inline uint32_t hz = 0;
    static void rebase(uint32_t next) { hz = next; }
};
static_assert(ClockUser<Follower>);

using Dyn = DynamicClock<Fast, Follower>;
static_assert(!Dyn::is_static);
static_assert(Dyn::source_hz == 48'000'000);
static_assert(Dyn::rate_count == 16);
static_assert(Dyn::rate_hz(0) == 48'000'000 && Dyn::rate_hz(2) == 16'000'000);
static_assert(Dyn::rate_hz(11) == 3'000'000 && Dyn::rate_hz(15) == 187'500);
static_assert(Dyn::can_run_at(6'000'000) && !Dyn::can_run_at(10'000'000));
static_assert(Dyn::rebases<Follower>);
static_assert(!Dyn::rebases<int>);
static_assert(clock_follows<Dyn, Follower>());
static_assert(!clock_follows<Dyn, int>());

void dynamic_verbs() {
    (void)Dyn::init();
    Dyn::set<6'000'000>();
    (void)Dyn::set(3'000'000);
    (void)Dyn::hz();
    (void)Dyn::rate_index();
    (void)clock_hz(Dyn{});
}
