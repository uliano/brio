// Random number generator family smoke TU (RM0090 ch. 24, the chapter
// that "applies to the whole STM32F4xx family" - as far as the family has
// the block at all). THE POINT OF THIS TU IS THE ABSENCE: three of the
// twenty-three headers declare no RNG_BASE, and on those the resource must
// not exist while the vocabulary above it still compiles. Everything
// register-facing is therefore inside the same guard the driver uses.
#include "stm32f4/rng.hpp"

using namespace brio;

// ---- the clock rule, on every part ------------------------------------------------

// 24.4.2's constraint is a RATIO and not a rate: CECS is raised below
// f(HCLK)/16. These are arithmetic and hold whether or not the part has a
// generator.
static_assert(rng_clock_ratio_ok(48'000'000u, 180'000'000u), "the nominal domain at the top rate");
static_assert(rng_clock_ratio_ok(45'000'000u, 180'000'000u),
              "the PLL Q output at 180 MHz is 45 MHz, and the ratio still admits it");
static_assert(rng_clock_ratio_ok(11'250'000u, 180'000'000u), "exactly HCLK/16 is admitted");
static_assert(!rng_clock_ratio_ok(11'000'000u, 180'000'000u), "just under it is not");
static_assert(!rng_clock_ratio_ok(0u, 16'000'000u), "a rate with no PLL has no domain at all");
static_assert(rng_clock_ratio_ok(48'000'000u, 100'000'000u), "the F411's rate, had it a generator");
static_assert(rng_nominal_clock_hz == 48'000'000u);
static_assert(rng_clocks_per_word == 40u, "24.2: forty RNG_CLK periods between two words");

// ---- the reserve's presence answers ------------------------------------------------

// A part with no generator has neither gate. Where it has one, the enable
// and the reset are the same bit of their two registers - and WHICH bit
// depends on the bus: 6 on AHB2, and 31 on the F410's AHB1, where the
// generator was fitted into the top of a register that was already there.
static_assert(rng_present() == (rng_clock_mask() != 0u));
static_assert(rng_present() == (rng_reset_mask() != 0u));
static_assert(rng_clock_mask() == rng_reset_mask(),
              "the enable and the reset sit at the same bit of their own registers");
static_assert(!rng_present() || rng_clock_mask() == (rng_clock_on_ahb1() ? (1u << 31) : (1u << 6)));

#if defined(RNG_BASE)

static_assert(rng_present());
static_assert(Rng::clock_on_ahb1 == rng_clock_on_ahb1());
static_assert(Rng::irq() != NonMaskableInt_IRQn, "the block has a vector wherever it exists");

// ---- every verb, once ---------------------------------------------------------------

// A clock with a PLL, because `init()` refuses a rate whose 48 MHz domain
// has none - and the RESET rate through the PLL, because it is the one
// rate every header of the family accepts (above 16 MHz the parts whose
// frequency ladder the reserve does not know are refused, which is the
// clock chapter's rule and not this one's).
using SysClock = Clock<ClockSource::pll_hsi, 16'000'000>;

void rng_verbs() {
    (void)Rng::init(SysClock{});
    Rng::clock(true);
    (void)Rng::clock();
    (void)Rng::regs().CR;
    Rng::enable(true);
    (void)Rng::enabled();
    Rng::interrupt(true);
    (void)Rng::interrupt();
    (void)Rng::ready();
    (void)Rng::seed_error();
    (void)Rng::clock_error();
    (void)Rng::seed_error_flag();
    (void)Rng::clock_error_flag();
    Rng::clear_seed_error();
    Rng::clear_clock_error();
    (void)Rng::value();
    (void)Rng::read();
    (void)Rng::last_error();
    (void)Rng::read_blocking(4u);
    (void)Rng::discard_first(4u);
    (void)Rng::recover();
    const auto e = Rng::isr();
    (void)e.ready;
    (void)e.seed_error;
    (void)e.clock_error;
    Rng::reset_block();
    Rng::release();
}

#else

// On the F401, F411 and F446 the type does not exist at all. Nothing to
// instantiate here - `neg/rng_absent_block.cpp` is where that is proven,
// and this TU's job on such a part is to compile with the header included.
static_assert(!rng_present());
static_assert(rng_clock_mask() == 0u && rng_reset_mask() == 0u);

#endif
