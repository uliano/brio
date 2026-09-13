/*
 * rng.hpp
 *
 * The STM32F4's true random number generator (RM0090 ch. 24 - the chapter
 * says of itself that it "applies to the whole STM32F4xx family"): `Rng`,
 * a MONOSTATE resource, because a part either carries one generator or
 * none. The F401, the F411 and the F446 carry none, and their device
 * header says so by declaring no RNG_BASE; on those parts the
 * register-facing half of this file is not compiled and an `Rng` spelled
 * there is a compile error naming the reason.
 *
 * WHAT THIS BLOCK IS. Several ring oscillators whose outputs are XORed
 * into a seed, the seed shifted into a linear feedback register, and the
 * register handed over 32 bits at a time - a TRUE generator with an analog
 * heart, not a pseudo-random sequence a program could reproduce. Beside
 * the datapath sit two monitors, one on the seed and one on the clock,
 * whose whole purpose is to say when the entropy is not to be trusted.
 *
 * SIX FACTS THAT SHAPE THE FILE.
 *
 * 1. IT HAS A CLOCK OF ITS OWN, AND THE 48 MHz DOMAIN IS WHERE IT COMES
 *    FROM. The LFSR is clocked by RNG_CLK "at a constant frequency, so
 *    that the quality of the random number is independent of the HCLK
 *    frequency" (24.3), and on this family that clock is the main PLL's Q
 *    output - the domain the USB OTG FS and the SDIO share, which
 *    `Clock::usb_hz` reports. THE MANUAL'S ONLY STATED CONSTRAINT ON IT IS
 *    A RATIO, not a rate: the clock monitor raises CECS when
 *    f(RNG_CLK) < f(HCLK)/16 (24.4.2), and that is what `init()`'s
 *    static_assert checks. 48 MHz is what the clock tree is meant to give
 *    the domain and what every ST example uses; a SYSCLK of 180 MHz gives
 *    45 MHz there, which the ratio still admits, and this driver does not
 *    refuse it - a rate that trips CECS is what the driver refuses, and
 *    CECS itself is what reports the rest.
 *
 * 2. THE CLOCK GATE IS ON A DIFFERENT BUS ON ONE PART. RNGEN is bit 6 of
 *    RCC_AHB2ENR on every part with the generator except the F410, which
 *    has no AHB2 at all and puts the same bit in RCC_AHB1ENR
 *    (`rng_clock_on_ahb1()` in the reserve). The bit number is the same;
 *    the register is not.
 *
 * 3. THE FIRST VALUE IS NOT A RANDOM NUMBER. FIPS PUB 140-2 asks that "the
 *    first random number generated after setting the RNGEN bit should not
 *    be used, but saved for comparison with the next" and that each
 *    subsequent number be compared with the one before it, the test
 *    failing on a repeat (24.3.1). `init()` performs the discard, and
 *    `read()` performs the comparison - a repeated word is reported as a
 *    failure and not handed out, which is the continuous test the standard
 *    asks for and costs one word of state.
 *
 * 4. THE TWO ERRORS ARE NOT THE SAME KIND. A CLOCK error (CECS) means the
 *    generator has stopped producing, but "has no impact on the previously
 *    generated random numbers, and the RNG_DR register contents can be
 *    used" - fix the clock tree, clear CEIS, carry on. A SEED error (SECS)
 *    means the analog part produced a sequence that failed its own test -
 *    more than 64 consecutive bits at one value, or more than 32
 *    alternations - and then "if a number is available in the RNG_DR
 *    register, it must not be used because it may not have enough
 *    entropy": clear SEIS, then clear AND SET RNGEN to reinitialize
 *    (24.3.2). `recover()` is that sequence, and `read()` refuses to hand
 *    out a word while either current-status bit stands.
 *
 * 5. THE TWO STATUS FLAGS ARE rc_w0, NOT rc_w1. SEIS and CEIS are
 *    "cleared by writing it to 0" (24.4.2) - the opposite convention from
 *    every other flag this stratum clears, and a plain read-modify-write
 *    with a one would leave them standing. So the clearing verbs store a
 *    word with the bit ZERO and every other bit of the register as it
 *    reads back, and never `|=`.
 *
 * 6. ONE VECTOR, AND IT MAY BE SHARED. The interrupt is raised by DRDY,
 *    SEIS or CEIS alike (24.4.1's IE bit), and there is no per-source
 *    enable - so `isr()` reports all three and the handler decides. The
 *    vector's NAME differs across the family (HASH_RNG_IRQn on the
 *    large-line parts, whose slot is shared with the hash processor,
 *    RNG_IRQn on the small ones); `irq()` is the reserve's answer to that.
 *
 * ERRATA. No item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is filed
 * against the RNG - and only the first of those three sheets belongs to a
 * part that has one.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"

namespace brio {

// =============================================================================
// The clock constraint
// =============================================================================

/**
 * THE ONE RATE RULE THE CHAPTER STATES (24.4.2, CECS and CEIS): the clock
 * monitor flags an error when f(RNG_CLK) is below f(HCLK)/16. True when
 * `rng_clk_hz` clears that bar - written as a multiplication so that a
 * division never rounds the answer in the caller's favour, and in 64 bits
 * because 16 x 48 MHz would be a plausible overflow to leave lying around.
 */
constexpr bool rng_clock_ratio_ok(uint32_t rng_clk_hz, uint32_t hclk_hz) {
    return rng_clk_hz != 0u &&
           static_cast<uint64_t>(rng_clk_hz) * 16u >= static_cast<uint64_t>(hclk_hz);
}

/// What ST's clock tree is built to put on the domain (RM0090 6.2.5's
/// "48 MHz clock": the USB OTG FS, the SDIO and this block). Stated
/// because it is the design point, NOT enforced: the chapter's constraint
/// is the ratio above.
inline constexpr uint32_t rng_nominal_clock_hz = 48'000'000;

/// What the generator costs per word (24.2): forty RNG_CLK periods between
/// two consecutive numbers - 833 ns at 48 MHz, 889 ns at 45.
inline constexpr uint32_t rng_clocks_per_word = 40;

#if defined(RNG_BASE)

// =============================================================================
// The block
// =============================================================================

/// What one attempt to read the generator produced, when it produced no
/// number - the reason, so that a caller can tell "not ready yet" from
/// "this silicon is not generating".
enum class RngError : uint8_t {
    not_ready,     ///< DRDY is clear: fewer than forty RNG_CLK periods have passed
    seed_error,    ///< SECS stands: the analog part failed its own sequence test
    clock_error,   ///< CECS stands: RNG_CLK is below HCLK/16, or absent
    repeated,      ///< the word equalled the one before it (the FIPS continuous test)
};

/**
 * The random number generator as a monostate resource.
 *
 *   brio::Rng::init(clock);                       // gate, enable, discard the first
 *   if (auto v = brio::Rng::read()) { use(*v); }  // or ask why not
 *
 * READ() IS THE WHOLE PROTOCOL: it checks the two error statuses, checks
 * DRDY, reads the word and compares it with the previous one, in the order
 * 24.3.1 asks for. `value()` beside it is the raw register for a program
 * that has already done its own checking - in an interrupt handler that
 * `isr()` has just told what happened, typically.
 */
struct Rng {
    Rng() = delete;

    static RNG_TypeDef& regs() { return *RNG; }

    /// The vector this block raises; its name differs across the family
    /// (see the file header, fact 6).
    static constexpr IRQn_Type irq() { return rng_irq(); }

    /// Whether the clock gate this part puts the generator behind is on
    /// AHB1 (the F410) or AHB2 (everyone else) - a fact worth reading back
    /// rather than assuming.
    static constexpr bool clock_on_ahb1 = rng_clock_on_ahb1();

    // ---- the clock gate --------------------------------------------------------
    //
    // The register differs by part and the bit does not, so the store is
    // selected by the header's own symbol: per-part CODE, which is what a
    // driver may keep a preprocessor branch for.

    static void clock(bool on) {
#if defined(RCC_AHB1ENR_RNGEN)
        Rcc::ahb1_clock(RCC_AHB1ENR_RNGEN, on);
#else
        Rcc::ahb2_clock(RCC_AHB2ENR_RNGEN, on);
#endif
    }
    static bool clock() {
#if defined(RCC_AHB1ENR_RNGEN)
        return Rcc::ahb1_clock(RCC_AHB1ENR_RNGEN);
#else
        return Rcc::ahb2_clock(RCC_AHB2ENR_RNGEN);
#endif
    }
    /// The peripheral's reset line: RNG_CR and RNG_SR to their reset
    /// values, the analog part restarted.
    static void reset_block() {
#if defined(RCC_AHB1RSTR_RNGRST)
        Rcc::ahb1_reset(RCC_AHB1RSTR_RNGRST);
#else
        Rcc::ahb2_reset(RCC_AHB2RSTR_RNGRST);
#endif
    }

    // ---- the control register (24.4.1) ------------------------------------------

    /// RNG_CR.RNGEN: the analog part, the LFSR and the error detector.
    static void enable(bool on) {
        regs().CR = on ? (regs().CR | RNG_CR_RNGEN) : (regs().CR & ~RNG_CR_RNGEN);
    }
    static bool enabled() { return (regs().CR & RNG_CR_RNGEN) != 0u; }

    /// RNG_CR.IE: one interrupt for all three events (DRDY, SEIS, CEIS).
    static void interrupt(bool on) {
        regs().CR = on ? (regs().CR | RNG_CR_IE) : (regs().CR & ~RNG_CR_IE);
    }
    static bool interrupt() { return (regs().CR & RNG_CR_IE) != 0u; }

    // ---- the status register (24.4.2) -------------------------------------------

    /// DRDY: a valid word is waiting in RNG_DR. Reading the data register
    /// puts it back to zero until the next one is computed.
    static bool ready() { return (regs().SR & RNG_SR_DRDY) != 0u; }

    /// SECS / CECS, the CURRENT statuses: what is wrong now, as opposed to
    /// what went wrong at some point (SEIS/CEIS below). Generation is
    /// stopped for as long as SECS stands.
    static bool seed_error() { return (regs().SR & RNG_SR_SECS) != 0u; }
    static bool clock_error() { return (regs().SR & RNG_SR_CECS) != 0u; }

    /// SEIS / CEIS, the latched interrupt statuses: set with their current
    /// status and cleared only by software, so they are the record that
    /// something happened even after the situation recovered.
    static bool seed_error_flag() { return (regs().SR & RNG_SR_SEIS) != 0u; }
    static bool clock_error_flag() { return (regs().SR & RNG_SR_CEIS) != 0u; }

    /// Clear one latched flag. rc_w0: the bit is cleared by writing a ZERO
    /// to it (see the file header, fact 5), so the store is the register as
    /// it reads with that one bit knocked out.
    static void clear_seed_error() { regs().SR = regs().SR & ~static_cast<uint32_t>(RNG_SR_SEIS); }
    static void clear_clock_error() { regs().SR = regs().SR & ~static_cast<uint32_t>(RNG_SR_CEIS); }

    // ---- the data register (24.4.3) ---------------------------------------------

    /// RNG_DR, raw: the caller has checked DRDY and the errors itself.
    static uint32_t value() { return regs().DR; }

    /**
     * One random word, or the reason there is none - the protocol of
     * 24.3.1 and 24.3.2 in one call: a seed error refuses (the word in
     * RNG_DR "must not be used"), a clock error refuses (the generator has
     * stopped; the word standing there is still good, and `value()` is how
     * a caller takes it deliberately), DRDY decides readiness, and the word
     * is compared with the previous one - the continuous test FIPS PUB
     * 140-2 asks for - before it is handed out.
     */
    static std::optional<uint32_t> read() {
        const uint32_t sr = regs().SR;
        if ((sr & RNG_SR_SECS) != 0u) {
            last_error_ = RngError::seed_error;
            return std::nullopt;
        }
        if ((sr & RNG_SR_CECS) != 0u) {
            last_error_ = RngError::clock_error;
            return std::nullopt;
        }
        if ((sr & RNG_SR_DRDY) == 0u) {
            last_error_ = RngError::not_ready;
            return std::nullopt;
        }
        const uint32_t v = regs().DR;
        if (v == previous_) {
            last_error_ = RngError::repeated;
            return std::nullopt;
        }
        previous_ = v;
        return v;
    }

    /// Why the last `read()` gave nothing. Meaningless after one that gave
    /// a word.
    static RngError last_error() { return last_error_; }

    /// A bounded wait for one word: `spins` reads of the status register at
    /// most. Forty RNG_CLK periods is under a microsecond, so a few
    /// thousand spins is a fault and not a slow start.
    static std::optional<uint32_t> read_blocking(uint32_t spins = 100'000u) {
        for (uint32_t i = 0; i < spins; ++i) {
            const auto v = read();
            if (v) {
                return v;
            }
            if (last_error_ != RngError::not_ready && last_error_ != RngError::repeated) {
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    // ---- bring-up and recovery ---------------------------------------------------

    /**
     * The gate, the enable and the FIPS discard, with the clock the
     * program runs checked against 24.4.2's ratio at COMPILE time for a
     * static clock. False when the first word never arrived - which on
     * this block means the clock domain is not running at all.
     *
     * `Clock::usb_hz` is the PLL's Q output, which is this generator's
     * clock as well as the USB's; a rate with no PLL has none and is
     * refused here rather than left to CECS.
     */
    template <typename Clock>
    static bool init(Clock) {
        if constexpr (Clock::is_static) {
            static_assert(Clock::usb_hz != 0u,
                          "brio Rng: this rate gives the 48 MHz domain no clock at all (there is "
                          "no PLL in it), and the generator has nothing to run on");
            static_assert(rng_clock_ratio_ok(Clock::usb_hz, Clock::hz),
                          "brio Rng: RM0090 24.4.2 - the clock monitor raises CECS below "
                          "f(HCLK)/16, and this rate's PLL Q output is under that bar");
        }
        clock(true);
        enable(true);
        return discard_first();
    }

    /**
     * The seed-error recovery of 24.3.2, in the order it is written: clear
     * SEIS, then clear and set RNGEN so the analog part and the LFSR start
     * again. The first word after it is discarded like the first word after
     * any enable. False when nothing came out afterwards.
     */
    static bool recover() {
        clear_seed_error();
        enable(false);
        enable(true);
        return discard_first();
    }

    /// Throw away one word and remember it as the comparison value - what
    /// the standard asks for after every enable (24.3.1). False when no
    /// word arrived within the bounded wait.
    static bool discard_first(uint32_t spins = 100'000u) {
        previous_ = 0;
        for (uint32_t i = 0; i < spins; ++i) {
            const uint32_t sr = regs().SR;
            if ((sr & (RNG_SR_SECS | RNG_SR_CECS)) != 0u) {
                return false;
            }
            if ((sr & RNG_SR_DRDY) != 0u) {
                previous_ = regs().DR;
                return true;
            }
        }
        return false;
    }

    // ---- the ISR body --------------------------------------------------------------

    /// What one interrupt found: any of the three sources may have raised
    /// it, and the two error flags are LATCHED, so the body reports them
    /// and clears them - the data, if any, is left in RNG_DR for the
    /// handler to take with `read()`.
    struct RngEvent {
        bool ready = false;
        bool seed_error = false;
        bool clock_error = false;
    };

    [[gnu::always_inline]] static RngEvent isr() {
        const uint32_t sr = regs().SR;
        RngEvent e{};
        e.ready = (sr & RNG_SR_DRDY) != 0u;
        e.seed_error = (sr & RNG_SR_SEIS) != 0u;
        e.clock_error = (sr & RNG_SR_CEIS) != 0u;
        if (e.seed_error || e.clock_error) {
            // rc_w0 on both: one store puts zeros where the flags stood and
            // leaves everything else as it reads.
            regs().SR = sr & ~static_cast<uint32_t>((e.seed_error ? RNG_SR_SEIS : 0u) |
                                                    (e.clock_error ? RNG_SR_CEIS : 0u));
        }
        return e;
    }

    /// Everything off: the generator disabled, the interrupt masked, the
    /// flags cleared and the gate shut.
    static void release() {
        interrupt(false);
        enable(false);
        clear_seed_error();
        clear_clock_error();
        clock(false);
    }

private:
    static inline uint32_t previous_ = 0;
    static inline RngError last_error_ = RngError::not_ready;
};

#endif // RNG_BASE

} // namespace brio
