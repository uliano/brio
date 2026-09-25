/*
 * rng.hpp
 *
 * The CH32V303's random number generator (RM ch. 29 - which says of
 * itself that it "applies to the whole family", as far as the family has
 * the block at all): `Rng`, a MONOSTATE resource over three registers.
 * The CH32V303RC and VC carry one and every other part of this stratum
 * carries none - the CH32V303 datasheet's table 2-1-1 gives the RNG row
 * a dash under the CB and the RB, and the CH32V203's table 2-1 has no
 * such row - so `device::has_rng` is the part fact, and an `Rng` named
 * on a part without it is a compile error naming the reason.
 *
 * WHAT THIS BLOCK IS. "Based on continuous analog noise" (29.1): an
 * analog circuit produces a seed, the seed is shifted into a linear
 * feedback register, and "when a large number of seeds are introduced"
 * the register's content is handed over as a 32-bit word (29.2). Beside
 * the datapath sit two monitors, one on the seed and one on the clock,
 * whose whole purpose is to say when a word is not to be trusted. Three
 * registers at 0x4002 3C00 (table 29-1): RNG_CR with two bits, RNG_SR
 * with five, RNG_DR with the word. No DMA request - no table of chapter
 * 11 names this block - and no event.
 *
 * SEVEN FACTS THAT SHAPE THE FILE.
 *
 * 1. SYSCLK RUNS IT, AND NO RATE IS REFUSED. Chapter 29 says the LFSR is
 *    clocked "by a dedicated clock (PLL48CLK)" and names that clock again
 *    in the two clock-error bits; the class's own clock tree (figure 3-3,
 *    "applied for ... CH32V30x_D8") and the CH32V303 datasheet's (figure
 *    2-4) draw SYSCLK to the TRNG instead, and RCC_CFGR2's RNG_SRC, which
 *    would choose, is the D8C classes' register (3.4.12). MEASURED on the
 *    CH32V303VCT6: the time from one word to the next is the same number
 *    of core cycles at 144, 96 and 48 MHz off the PLL, it does not move
 *    when USBPRE divides the PLL by 2 or 1 instead of 3, and with no PLL
 *    at all - SYSCLK on the bare HSI - the words keep coming with the
 *    clock monitor quiet. So the figures are right, the chapter's
 *    PLL48CLK is not this class's, and `init()` asks nothing of the
 *    clock tree.
 *
 * 2. THE GATE AND NO RESET LINE. RCC_HBPCENR.RNGEN (3.4.6, bit 9)
 *    opens the bus clock; RCC_AHBRSTR has no bit for this block (its
 *    [11:0] are reserved, 3.4.11), so there is no peripheral reset and
 *    no verb offers one - the way back is RNGEN cleared and set.
 *
 * 3. THE FIRST VALUE IS ZERO. FIPS PUB 140-2 asks that the first word
 *    generated after an enable be saved for comparison and not used, and
 *    that each word after it be compared with the one before, the test
 *    failing on a repeat. The chapter does not ask it; this silicon
 *    does: MEASURED, the first word after RNGEN is set is 0x00000000 on
 *    every enable, at every rate, some sixty core cycles after it - and
 *    RNGEN cleared freezes the block, the word standing across the
 *    disable staying readable and no other landing behind it. So a
 *    RESTART (`init()` from off, `recover()`) takes the standing word
 *    between the disable and the enable and discards TWO words after it,
 *    the zero and the word that follows: a restart that discarded one
 *    handed the zero out once in some thirteen thousand on that silicon,
 *    with the cause never caught again, and one word is the price of it
 *    never happening. The last word discarded is what the next is
 *    compared with, and `read()` refuses a repeated word instead of
 *    handing it out.
 *
 * 4. THE TWO ERRORS ARE NOT THE SAME KIND, and 29.2.2 says so. A CLOCK
 *    error (CECS) stops the generator but "has no effect on the last
 *    random number and can be used normally": fix the clock tree, clear
 *    CEIS, carry on. A SEED error (SECS: "more than 64 identical
 *    consecutive bits; more than 32 consecutive alternating 0's and
 *    1's") makes the word standing in RNG_DR unusable, and the way back
 *    is a SEQUENCE - "the SEIS bit needs to be cleared first, then the
 *    RNGEN bit is cleared and set to 1". `recover()` is that sequence,
 *    and `read()` hands out no word while either current status stands.
 *    Neither monitor has been seen to fire on this silicon: SYSCLK never
 *    stops, not even across a switch of the clock task, which parks on
 *    the HSI, and the seed test watches runs, which fact 7's words do
 *    not have.
 *
 * 5. THE TWO LATCHED FLAGS ARE CLEARED WITH A ZERO, AND A ONE DOES
 *    NOTHING. SEIS and CEIS are "RW" in 29.3.2, which names no clearing
 *    rule; WCH's own library clears one by storing the register with that
 *    bit ZERO and every other bit ONE. MEASURED: a one written into
 *    either flag sets neither, and the two reserved "RW" bits [4:3] keep
 *    nothing written into them. So the vendor's store clears exactly the
 *    one flag, and the other - should it latch between a read and the
 *    store - is written a one and kept: the clearing verbs and `isr()`
 *    use it, never a read-modify-write. The same measurement means no
 *    program can raise either flag, and the error paths are reached by
 *    the silicon alone.
 *
 * 6. ONE VECTOR FOR THREE EVENTS. RNG_CR.IE raises line 63 of this
 *    class's table (`Irq::rng`) on a word ready or on either error, with
 *    no per-source enable, so `isr()` reports all three and clears the
 *    two latched ones; the word itself is the handler's to take. A ready
 *    word keeps DRDY up until RNG_DR is read, so a handler that does not
 *    read it is entered again at once.
 *
 * 7. THE WORDS ARE NOT TO BE TAKEN AS RANDOM BITS. MEASURED on the
 *    CH32V303VCT6 (docs/ch32vx03/rng.md has the numbers): DRDY rises
 *    again over an UNCHANGED word, the more often the faster the words
 *    are taken - up to one in seven read raw at DRDY's pace - which is
 *    what `read()`'s comparison refuses; and the words come from a
 *    small set - from nine hundred to three thousand-odd distinct values
 *    in four thousand words, some two hundred and twenty in 256 read ten
 *    milliseconds apart - so a chi-square over their bytes fails at
 *    every rate and every pace tried, and so do FIPS PUB 140-2's poker
 *    and runs tests. The two monitors say nothing about it. THIS DRIVER
 *    DOES NOT CONDITION THE WORDS: what it hands out is what the block
 *    produced, a word that repeats its predecessor excepted, and a
 *    program that needs randomness from it decides how many words make
 *    one it can use.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"

namespace brio {

// =============================================================================
// The registers (RM 29.3)
// =============================================================================

/// Table 29-1: three registers on the HB bus at 0x4002 3C00
/// (device.hpp's `rng_base`).
struct RngRegs {
    volatile uint32_t CR;   ///< 0x00 IE and RNGEN
    volatile uint32_t SR;   ///< 0x04 DRDY, CECS, SECS, CEIS, SEIS
    volatile uint32_t DR;   ///< 0x08 the 32-bit word, read-only
};

inline RngRegs* rng_regs() { return reinterpret_cast<RngRegs*>(rng_base); }

/// RNG_CR (29.3.1).
inline constexpr uint32_t rng_rngen = 1UL << 2;   ///< the analog part, the LFSR and the detector
inline constexpr uint32_t rng_ie    = 1UL << 3;   ///< one interrupt for a word and both errors

/// RNG_SR (29.3.2). DRDY, CECS and SECS are read-only; CEIS and SEIS
/// latch with their current statuses, are cleared by a ZERO and ignore a
/// ONE (fact 5). Bits [4:3], "reserved RW", keep nothing written.
inline constexpr uint32_t rng_drdy = 1UL << 0;   ///< a word waits in RNG_DR; cleared by reading it
inline constexpr uint32_t rng_cecs = 1UL << 1;   ///< the clock is not detected correctly, now
inline constexpr uint32_t rng_secs = 1UL << 2;   ///< a faulty seed sequence, now
inline constexpr uint32_t rng_ceis = 1UL << 5;   ///< a clock error happened
inline constexpr uint32_t rng_seis = 1UL << 6;   ///< a seed error happened
inline constexpr uint32_t rng_latched = rng_ceis | rng_seis;

// =============================================================================
// The block
// =============================================================================

/// What one attempt produced when it produced no word - the reason, so
/// a caller can tell "not ready yet" from "this silicon is not
/// generating".
enum class RngError : uint8_t {
    not_ready,      ///< DRDY is clear: the next word is not computed yet
    seed_error,     ///< SECS stands: the analog part failed its own sequence test
    clock_error,    ///< CECS stands: the generator's clock is not detected correctly
    repeated,       ///< the word equalled the one before it (the continuous test, fact 3)
};

/// What one interrupt found: any of the three sources may have raised
/// it, and the two errors are LATCHED, so the body reports them and
/// clears them - the word, if any, is left in RNG_DR for the handler.
struct RngEvent {
    bool ready = false;
    bool seed_error = false;
    bool clock_error = false;
};

/**
 * The random number generator as a monostate resource. `Rng` is the one
 * instance's name and the only one a program writes:
 *
 *   if (brio::Rng::init()) {                   // gate, enable, discard the first
 *       if (auto v = brio::Rng::read()) { use(*v); }
 *   }
 *
 * A TEMPLATE WITH ONE INSTANCE, so that its refusal waits for a use:
 * every header of this stratum compiles on every part of the family,
 * and a class-scope static_assert in a plain struct would fire where the
 * header is included rather than where the block is named.
 *
 * READ() IS THE WHOLE PROTOCOL: the two current statuses, then DRDY,
 * then the word compared with the previous one, in the order 29.2.1
 * and fact 3 ask for. `value()` beside it is the raw register, for a
 * handler that `isr()` has just told what happened.
 */
template <uint8_t n>
struct RngUnit {
    static_assert(n == 1u, "brio Rng: a part carries one random number generator or none");
    static_assert(device::has_rng,
                  "brio Rng: this part has no random number generator - the CH32V303 "
                  "datasheet's table 2-1-1 gives one to the CH32V303RC and VC alone, and the "
                  "CH32V203 series has none (device::has_rng, parts/<part>.hpp)");

    RngUnit() = delete;

    static RngRegs& regs() { return *rng_regs(); }

    /// The vector this block raises: line 63 of the CH32V30x_D8's table.
    static constexpr Irq irq = Irq::rng;

    /// How many status reads the bounded waits spend. A BOUND and not
    /// an expectation: a word follows the last in tens of core cycles
    /// (measured), and a generator that never answers is a fact to
    /// report rather than a hang.
    static constexpr uint32_t spins = 100'000;

    // ---- the gate (3.4.6) --------------------------------------------------
    static void clock(bool on) {
        if (on) {
            Rcc::enable(Bus::hb, rcc_hb_rng);
        } else {
            Rcc::disable(Bus::hb, rcc_hb_rng);
        }
    }
    static bool clock() { return Rcc::enabled(Bus::hb, rcc_hb_rng); }

    // ---- the control register (29.3.1) --------------------------------------
    /// RNGEN: the analog part, the LFSR and the error detector (29.2.1's
    /// step 2). The raw verb: `init()` and `recover()` are the ones that
    /// restart the generator with fact 3's discards.
    static void enable(bool on) {
        regs().CR = on ? (regs().CR | rng_rngen) : (regs().CR & ~rng_rngen);
    }
    static bool enabled() { return (regs().CR & rng_rngen) != 0u; }

    /// IE: one interrupt for a word ready and for either error.
    static void interrupt(bool on) {
        regs().CR = on ? (regs().CR | rng_ie) : (regs().CR & ~rng_ie);
    }
    static bool interrupt() { return (regs().CR & rng_ie) != 0u; }

    // ---- the status register (29.3.2) ----------------------------------------
    static uint32_t status() { return regs().SR; }
    /// DRDY: a word waits in RNG_DR. Reading the data register puts it
    /// down until the next word is computed.
    static bool ready() { return (regs().SR & rng_drdy) != 0u; }
    /// SECS / CECS, the CURRENT statuses: what is wrong now.
    static bool seed_error() { return (regs().SR & rng_secs) != 0u; }
    static bool clock_error() { return (regs().SR & rng_cecs) != 0u; }
    /// SEIS / CEIS, the latched ones: set with their current status and
    /// cleared only by software - the record that something happened.
    static bool seed_error_flag() { return (regs().SR & rng_seis) != 0u; }
    static bool clock_error_flag() { return (regs().SR & rng_ceis) != 0u; }

    /// Clear one latched flag: a zero into it and a ONE everywhere else,
    /// which the other flag, the read-only bits and the reserved ones
    /// ignore (fact 5) - so nothing that latches meanwhile is lost.
    static void clear_seed_error() { regs().SR = ~rng_seis; }
    static void clear_clock_error() { regs().SR = ~rng_ceis; }

    // ---- the data register (29.3.3) --------------------------------------------
    /// RNG_DR, raw: the caller has checked DRDY and the errors itself.
    static uint32_t value() { return regs().DR; }

    /**
     * One word, or nothing and the reason in `last_error()`: a seed
     * error refuses (the word in RNG_DR is not to be used), a clock error
     * refuses (the generator has stopped; the word standing there is
     * still good, and `value()` is how a caller takes it on purpose),
     * DRDY decides readiness, and the word is compared with the previous
     * one before it is handed out - which on this silicon refuses real
     * repeats (fact 7) and not only a formality's.
     */
    static std::optional<uint32_t> read() {
        const uint32_t sr = regs().SR;
        if ((sr & rng_secs) != 0u) {
            last_error_ = RngError::seed_error;
            return std::nullopt;
        }
        if ((sr & rng_cecs) != 0u) {
            last_error_ = RngError::clock_error;
            return std::nullopt;
        }
        if ((sr & rng_drdy) == 0u) {
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

    /// Why the last `read()`, `init()` or `recover()` gave nothing.
    /// Meaningless after one that succeeded.
    static RngError last_error() { return last_error_; }

    /// A bounded wait for one word: `limit` attempts at most, giving up
    /// at once on an error that is not a matter of waiting.
    static std::optional<uint32_t> read_blocking(uint32_t limit = spins) {
        for (uint32_t i = 0; i < limit; ++i) {
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
     * The gate, the enable and the discard. A generator already running
     * is left running and its next word discarded; one that is off is
     * RESTARTED - the word a disable left standing taken, and fact 3's
     * two discards after the enable, so that neither that word nor the
     * enable's zero is handed out. False, with `last_error()` naming
     * why, when no word came. No clock is asked: SYSCLK runs the block
     * at every rate (fact 1).
     */
    static bool init() {
        clock(true);
        if (!enabled()) {
            return restart();
        }
        return discard_first();
    }

    /**
     * The seed-error recovery of 29.2.2, in the order it is written:
     * SEIS cleared, then RNGEN cleared and set, so the analog part and
     * the LFSR start again - with fact 3's two discards after it, as
     * `init()` restarts a generator that is off.
     */
    static bool recover() {
        clear_seed_error();
        return restart();
    }

    /// Throw one word away and keep it as the comparison value - fact 3,
    /// after every enable. False, and the reason in `last_error()`, when
    /// an error status stood or no word came within the bound.
    static bool discard_first(uint32_t limit = spins) {
        previous_ = 0;
        for (uint32_t i = 0; i < limit; ++i) {
            const uint32_t sr = regs().SR;
            if ((sr & rng_secs) != 0u) {
                last_error_ = RngError::seed_error;
                return false;
            }
            if ((sr & rng_cecs) != 0u) {
                last_error_ = RngError::clock_error;
                return false;
            }
            if ((sr & rng_drdy) != 0u) {
                previous_ = regs().DR;
                return true;
            }
        }
        last_error_ = RngError::not_ready;
        return false;
    }

    // ---- the ISR body ------------------------------------------------------------

    /// The vector's body: the three sources reported, the latched errors
    /// seen cleared in ONE store - a zero where each stood and a one
    /// everywhere else, which keeps one that latched after the read
    /// (fact 5).
    [[gnu::always_inline]] static RngEvent isr() {
        const uint32_t sr = regs().SR;
        RngEvent e{};
        e.ready = (sr & rng_drdy) != 0u;
        e.seed_error = (sr & rng_seis) != 0u;
        e.clock_error = (sr & rng_ceis) != 0u;
        if (e.seed_error || e.clock_error) {
            regs().SR = ~(sr & rng_latched);
        }
        return e;
    }

    /// Everything off: the interrupt masked, the generator disabled, the
    /// flags cleared and the gate shut.
    static void release() {
        interrupt(false);
        enable(false);
        regs().SR = ~rng_latched;
        clock(false);
    }

private:
    /// RNGEN cleared and set, the word standing across the disable taken
    /// between the two, and two words discarded after the enable (fact
    /// 3): its zero and the word that follows, the last kept for the
    /// comparison.
    static bool restart() {
        enable(false);
        (void)regs().DR;
        enable(true);
        return discard_first() && discard_first();
    }

    static inline uint32_t previous_ = 0;
    static inline RngError last_error_ = RngError::not_ready;
};

/// The generator, as a program names it.
using Rng = RngUnit<1>;

} // namespace brio
