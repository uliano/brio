// Family smoke TU for samc/freqm.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// FREQM is one instance on every member of the family and its registers
// do not vary by package, so what this fixture pins is the ARITHMETIC -
// the overflow budget and the ratio-to-hertz conversion, both decisions
// this driver makes rather than reads from a register.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/freqm.hpp"

using namespace brio;

// The two channels are header constants and NOT adjacent, which is
// exactly why they are read from the header and not computed.
static_assert(Freqm::gclk_measured == 3);
static_assert(Freqm::gclk_reference == 4);
static_assert(Freqm::gclk_measured != Freqm::gclk_reference);

// VALUE is 24 bits, and refnum_for() is the overflow budget: the largest
// REFNUM whose count still fits.
static_assert(Freqm::value_max == 0xFFFFFFUL);
static_assert(Freqm::refnum_for(1) == 255, "a slow measurand can count for ages");
static_assert(Freqm::refnum_for(48000) == 255, "48 MHz against 1 kHz still fits");
static_assert(Freqm::refnum_for(0xFFFFFF) == 1, "one reference period is all there is");
static_assert(Freqm::refnum_for(0x1000000) == 1, "and past that it cannot fit at all");
static_assert(Freqm::refnum_for(0) == 255, "a stopped measurand cannot overflow");
static_assert(Freqm::refnum_for(70000) == 239);
static_assert(70000ULL * Freqm::refnum_for(70000) <= Freqm::value_max,
              "whatever refnum_for returns must not overflow VALUE");

// to_hz is VALUE / REFNUM x f_ref, and it must not lose the top of a
// product that leaves 32 bits on the way.
static_assert(Freqm::to_hz(48000, 1024, 1) == 49152000UL);
static_assert(Freqm::to_hz(48000, 1024, 2) == 24576000UL);
static_assert(Freqm::to_hz(0, 1024, 1) == 0);
static_assert(Freqm::to_hz(100, 1000, 0) == 0, "a zero REFNUM answers zero, not UB");
// DIVREF folds into the reference, not into REFNUM.
static_assert(Freqm::to_hz(48000, 8192, 1, true) == Freqm::to_hz(48000, 1024, 1));
// THE CASE THE 64-BIT INTERMEDIATE EXISTS FOR: a full 24-bit count
// times a kilohertz reference is 17179868160, four times past what a
// 32-bit product holds, while the QUOTIENT fits comfortably. Done in 32
// bits this would come out as garbage.
static_assert(Freqm::to_hz(0xFFFFFF, 1024, 255) == 67372032UL);
static_assert(0xFFFFFFULL * 1024ULL > 0xFFFFFFFFULL, "and it really does not fit");

static_assert(!Freqm::config_valid(FreqmConfig{.refnum = 0}),
              "REFNUM must be non-zero (44.8.3)");
static_assert(!Freqm::config_valid(
                  FreqmConfig{.measured_generator = 2, .reference_generator = 2}),
              "one generator cannot be both clocks");

void verbs() {
    constexpr FreqmConfig cfg{
        .measured_generator = 0,
        .reference_generator = 5,
        .refnum = 4,
        .divide_reference = true,
    };
    (void)Freqm::init(cfg);
    (void)Freqm::config_valid(cfg);

    Freqm::bus_clock(true);
    (void)Freqm::busy_sync();
    (void)Freqm::wait_sync();
    (void)Freqm::reset();
    (void)Freqm::enabled();
    (void)Freqm::enable(true);

    (void)Freqm::status();
    (void)Freqm::running();
    (void)Freqm::overflowed();
    Freqm::clear_overflow();
    (void)Freqm::flags();
    (void)Freqm::armed();
    Freqm::clear_flags();
    Freqm::arm(FreqmFlag::done);
    Freqm::disarm(FreqmFlag::done);
    (void)Freqm::done_flag();
    (void)Freqm::isr();
    (void)Freqm::irq();

    Freqm::start();
    (void)Freqm::value();
    (void)Freqm::measure();
    Freqm::release();
}
