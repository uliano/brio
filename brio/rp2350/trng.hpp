/*
 * trng.hpp
 *
 * The true random number generator (datasheet 12.12): `Trng`, a
 * MONOSTATE - one instance, and a block the RP2040 never had. Arm IP,
 * not Raspberry Pi's own, which is why its register names read unlike
 * anything else in this stratum and why its offsets start at 0x100.
 *
 * WHAT IT IS. A free-running ring oscillator with no connection to the
 * system clock tree, sampled every SAMPLE_CNT1 system clock cycles. The
 * samples go through three entropy checks and a von Neumann
 * decorrelator, and 192 accepted bits at a time appear in the six
 * EHR_DATA registers. About 7.5 kb/s at a 150 MHz system clock (12.12.1)
 * - a source to SEED with, not a stream to read from.
 *
 * THE THREE CHECKS, ALL ON AT RESET (12.12.2), AND WHAT EACH COSTS.
 *
 *   AUTOCORR_ERR  the autocorrelation test failed FOUR TIMES IN A ROW.
 *                 "RNG ceases functioning until next reset", and the
 *                 clear register says of this one bit alone "cannot be
 *                 cleared by SW". It is the one failure a program cannot
 *                 wait out: `recover()`, an internal soft reset, is the
 *                 only way back.
 *   CRNGT_ERR     two consecutive blocks of sixteen collected bits were
 *                 equal. The run is discarded and the next one starts.
 *   VN_ERR        thirty-two consecutive collected bits were identical.
 *                 Same: the run is discarded, nothing is presented.
 *
 * So a failed check is not an error in the ordinary sense - it is the
 * block refusing to hand out entropy it does not trust - and a driver
 * that treats it as one would give up where it should go round again.
 * `read()` reports each kind separately and `read_blocking()` goes round
 * again on the two recoverable ones, counting them, so that "how often
 * does this silicon fail its own checks" is a NUMBER a suite prints.
 *
 * GENERATION TIME IS NOT DETERMINISTIC (12.12.4). The mean and the mode
 * are close, but a run can take a hundred times the average. The
 * datasheet's advice is to keep a small pool and refill it in the
 * background rather than to block; every wait here is therefore bounded
 * and says so when it runs out, and no verb of this file is safe to call
 * from an interrupt handler with a deadline behind it.
 *
 * THE SETTINGS, AND WHY THE DEFAULTS ARE WHAT THEY ARE. 12.12.2: "For
 * acceptable results with an average generation time of about 2
 * milliseconds, use ROSC chain length settings of 0 or 1 and sample
 * count settings of 20-25." Longer sample counts trade time for quality.
 * A LOW sample count is not merely slower to succeed, it is wrong: with
 * the von Neumann decorrelator bypassed the count "must not be less than
 * seventeen" (SAMPLE_CNT1's own description), and `init()` refuses that
 * combination rather than letting it produce plausible rubbish.
 *
 * WHAT THE BOOTROM DOES INSTEAD, and why this driver does not. The ROM
 * streams RAW ROSC samples past every check straight into the SHA-256
 * accelerator (12.12.4.1), because it must boot in bounded time and a
 * hash is a better conditioner than the decorrelator. That is a
 * legitimate use of the same silicon and a different contract; this file
 * is the block as 12.12.3 specifies it, checks and all. The per-boot
 * random number the ROM produced that way is available through
 * rp2350/bootrom.hpp's `get_sys_info`, which is the cheap seed a program
 * usually wants.
 *
 * THE INTERNAL SOFT RESET NEEDS A DELAY. 12.12.4.1's listing writes
 * TRNG_SW_RESET and then reads the register twice before touching
 * anything else, with the comment "fixed delay is required after TRNG
 * soft reset". `sw_reset()` does exactly that and nothing more.
 *
 * ONE VECTOR, `isr_trng`, masked at reset (every RNG_IMR bit resets to
 * 1). The ISR body reports the four sources and clears the three that
 * can be cleared.
 */

#pragma once

#include <stdint.h>

#include <array>
#include <optional>

#include "rp2350/core.hpp"
#include "rp2350/device.hpp"
#include "rp2350/resets.hpp"

namespace brio {

/// The four bits RNG_ISR, RNG_ICR and RNG_IMR share (12.12.5).
struct TrngFlag {
    static constexpr uint32_t ehr_valid = TRNG_RNG_ISR_EHR_VALID_BITS;
    static constexpr uint32_t autocorr_error = TRNG_RNG_ISR_AUTOCORR_ERR_BITS;
    static constexpr uint32_t crngt_error = TRNG_RNG_ISR_CRNGT_ERR_BITS;
    static constexpr uint32_t von_neumann_error = TRNG_RNG_ISR_VN_ERR_BITS;
    /// Every source, for masking or clearing in one store.
    static constexpr uint32_t all = ehr_valid | autocorr_error | crngt_error | von_neumann_error;
    /// The two a program can clear and go round again on.
    static constexpr uint32_t recoverable = crngt_error | von_neumann_error;
};

/// How the entropy source is set up (12.12.2).
struct TrngConfig {
    /// TRNG_CONFIG.RND_SRC_SEL, 0..3: which of the four inverter-chain
    /// lengths the ring oscillator runs. Higher is longer and slower.
    uint8_t chain = 1;
    /// SAMPLE_CNT1: system clock cycles between two samples of the ring
    /// oscillator. The chapter's own working range is 20..25.
    uint32_t sample_cycles = 25;
    /// The three checks. All three run at reset, and a program that
    /// turns one off is asking for raw samples on purpose.
    bool autocorrelation = true;
    bool crngt = true;
    bool von_neumann = true;
};

/// SAMPLE_CNT1's own floor when the von Neumann decorrelator is
/// bypassed (12.12.5): "the minimum value for sample counter must not be
/// less than seventeen".
inline constexpr uint32_t trng_min_sample_cycles_no_von_neumann = 17;

/**
 * Whether a set of settings is one the chapter admits: a chain length
 * that exists, a sample count that is not zero, and the decorrelator's
 * floor respected when it is bypassed. Written once, so that the
 * compile-time `init<cfg>()` and the run-time `init(cfg)` cannot judge
 * the same settings differently.
 */
constexpr bool trng_config_legal(TrngConfig cfg) {
    return cfg.chain <= 3u && cfg.sample_cycles != 0u &&
           (cfg.von_neumann || cfg.sample_cycles >= trng_min_sample_cycles_no_von_neumann);
}

/// Why an attempt to read entropy produced none.
enum class TrngError : uint8_t {
    not_ready,        ///< the collection is still running (or the source is stopped)
    autocorrelation,  ///< AUTOCORR_ERR: the block has stopped until it is reset
    crngt,            ///< CRNGT_ERR: the run was discarded, the next one has started
    von_neumann,      ///< VN_ERR: likewise
    misconfigured,    ///< the settings asked for are outside the chapter's own rules
};

/// 192 bits, as the six registers present them.
struct TrngEntropy {
    std::array<uint32_t, 6> words{};
};

/// What one interrupt found. The two recoverable errors are cleared by
/// the body; AUTOCORR_ERR is left standing, because only a reset moves
/// it and the handler must be able to see it.
struct TrngEvent {
    bool ready = false;
    bool autocorr_error = false;
    bool crngt_error = false;
    bool von_neumann_error = false;
};

/**
 * The generator as a monostate resource.
 *
 *   brio::Trng::init();                       // the block, the settings
 *   brio::Trng::start();                      // the ring oscillator
 *   if (auto e = brio::Trng::read_blocking()) { seed(e->words); }
 *   brio::Trng::stop();                       // it is a waste of power idle
 */
struct Trng {
    Trng() = delete;

    static TRNG_Type& regs() { return *TRNG; }

    /// The reset controller's bit for this block (7.5).
    static constexpr uint32_t reset_block = ResetBlock::trng;

    /// The one vector, which an app binds as `isr_trng`.
    static constexpr IRQn_Type irq = TRNG_IRQ_IRQn;

    /// SAMPLE_CNT1's own floor when the decorrelator is bypassed.
    static constexpr uint32_t min_sample_cycles_no_von_neumann =
        trng_min_sample_cycles_no_von_neumann;

    /// The block's own width: 192 bits in six registers (RNG_VERSION's
    /// EHR_WIDTH_192 reports which the IP was built with).
    static constexpr uint32_t entropy_bits = 192;

    /// A whole generation takes about 2 ms with the default settings and
    /// can take a hundred times that, so a blocking read's budget is
    /// counted in status reads and is deliberately large; what it
    /// guarantees is that the wait ENDS, not that it is short.
    static constexpr uint32_t default_spins = 4'000'000u;

    // ---- bring-up ------------------------------------------------------------

    /**
     * The block from its reset state and the settings of `cfg`: the
     * subsystem reset cycled, the IP's own soft reset with its delay,
     * the chain length, the sample count, the three checks, every
     * interrupt masked. The source is left STOPPED - `start()` is a
     * separate verb because the oscillator costs power and 12.12.3 says
     * to clear the enable when it is not in use.
     *
     * False when the block did not come out of reset, or when `cfg` asks
     * for something the chapter forbids: a chain above 3, a sample count
     * of zero, or a count below seventeen with the von Neumann
     * decorrelator bypassed.
     */
    static bool init(TrngConfig cfg = {}) {
        if (!trng_config_legal(cfg)) {
            last_error_ = TrngError::misconfigured;
            return false;
        }
        if (!Resets::cycle(reset_block)) {
            last_error_ = TrngError::not_ready;
            return false;
        }
        sw_reset();
        regs().RND_SOURCE_ENABLE = 0;
        regs().TRNG_CONFIG = cfg.chain;
        regs().SAMPLE_CNT1 = cfg.sample_cycles;
        regs().TRNG_DEBUG_CONTROL =
            (cfg.autocorrelation ? 0u : TRNG_TRNG_DEBUG_CONTROL_AUTO_CORRELATE_BYPASS_BITS) |
            (cfg.crngt ? 0u : TRNG_TRNG_DEBUG_CONTROL_TRNG_CRNGT_BYPASS_BITS) |
            (cfg.von_neumann ? 0u : TRNG_TRNG_DEBUG_CONTROL_VNC_BYPASS_BITS);
        regs().RNG_ICR = TrngFlag::all;
        interrupt_mask(TrngFlag::all);
        recoverable_failures_ = 0;
        last_error_ = TrngError::not_ready;
        return true;
    }

    /// The same, with the settings a CONSTANT: what the chapter would
    /// have refused at run time is refused here by the compiler, naming
    /// the rule it broke.
    template <TrngConfig cfg>
    static bool init() {
        static_assert(cfg.chain <= 3u,
                      "brio Trng: TRNG_CONFIG.RND_SRC_SEL picks one of FOUR inverter chain "
                      "lengths (datasheet 12.12.5), so the chain is 0..3");
        static_assert(cfg.sample_cycles != 0u,
                      "brio Trng: SAMPLE_CNT1 is the number of system clock cycles between two "
                      "samples of the ring oscillator, and zero is not a sampling rate");
        static_assert(cfg.von_neumann ||
                          cfg.sample_cycles >= trng_min_sample_cycles_no_von_neumann,
                      "brio Trng: with the von Neumann decorrelator bypassed, SAMPLE_CNT1 must "
                      "not be less than seventeen (datasheet 12.12.5)");
        return init(cfg);
    }

    /**
     * The IP's internal soft reset (TRNG_SW_RESET), followed by the two
     * register reads 12.12.4.1 calls a required fixed delay. It is the
     * ONLY cure for AUTOCORR_ERR, and it returns every setting to its
     * reset value - so `recover()` below, which puts the settings back,
     * is what a program calls.
     */
    static void sw_reset() {
        regs().TRNG_SW_RESET = 1u;
        (void)regs().TRNG_SW_RESET;
        (void)regs().TRNG_SW_RESET;
    }

    /// The soft reset and the settings again, then the source started if
    /// it was running. What a program does when `read()` answers
    /// `TrngError::autocorrelation`.
    static bool recover(TrngConfig cfg = {}) {
        const bool was_running = running();
        if (!init(cfg)) {
            return false;
        }
        if (was_running) {
            start();
        }
        return true;
    }

    /// The block back into reset: the source stopped, its state gone.
    static void release() {
        regs().RND_SOURCE_ENABLE = 0;
        Resets::hold(reset_block);
    }

    // ---- the source ----------------------------------------------------------

    /// RND_SOURCE_ENABLE.RND_SRC_EN: the ring oscillator, and with it the
    /// collection. Clearing it is what 12.12.3 asks for when the block is
    /// not in use.
    static void start() { regs().RND_SOURCE_ENABLE = TRNG_RND_SOURCE_ENABLE_RND_SRC_EN_BITS; }
    static void stop() { regs().RND_SOURCE_ENABLE = 0; }
    static bool running() {
        return (regs().RND_SOURCE_ENABLE & TRNG_RND_SOURCE_ENABLE_RND_SRC_EN_BITS) != 0u;
    }

    /// TRNG_BUSY: a collection is in progress.
    static bool busy() { return (regs().TRNG_BUSY & TRNG_TRNG_BUSY_TRNG_BUSY_BITS) != 0u; }

    /// TRNG_VALID.EHR_VALID: 192 bits are collected and waiting. The same
    /// fact RNG_ISR's bit 0 reports, in a register of its own.
    static bool valid() { return (regs().TRNG_VALID & TRNG_TRNG_VALID_EHR_VALID_BITS) != 0u; }

    /// Reset the collected-bits counter (RST_BITS_COUNTER). 12.12.5: the
    /// source enable must be CLEAR for the reset to take, so this verb
    /// answers false rather than writing into the wind.
    static bool reset_bit_counter() {
        if (running()) {
            return false;
        }
        regs().RST_BITS_COUNTER = 1u;
        return true;
    }

    // ---- status and interrupts ------------------------------------------------

    /// RNG_ISR, raw: the four bits of `TrngFlag`.
    static uint32_t status() { return regs().RNG_ISR & TrngFlag::all; }

    /// RNG_ICR: write-one-to-clear, and AUTOCORR_ERR ignores it.
    static void clear(uint32_t flags) { regs().RNG_ICR = flags & TrngFlag::all; }

    /// RNG_IMR: a ONE masks. Every bit resets to one, so an interrupt is
    /// armed by clearing its bit and this verb takes the whole word.
    static void interrupt_mask(uint32_t masked) { regs().RNG_IMR = masked & TrngFlag::all; }
    static uint32_t interrupt_mask() { return regs().RNG_IMR & TrngFlag::all; }

    /// The ISR body an app's `isr_trng` calls. The two recoverable
    /// errors are cleared here; EHR_VALID is NOT, because it is cleared
    /// by reading the last result register and clearing it here would
    /// lose the handshake.
    [[gnu::always_inline]] static TrngEvent isr() {
        const uint32_t s = status();
        TrngEvent e{};
        e.ready = (s & TrngFlag::ehr_valid) != 0u;
        e.autocorr_error = (s & TrngFlag::autocorr_error) != 0u;
        e.crngt_error = (s & TrngFlag::crngt_error) != 0u;
        e.von_neumann_error = (s & TrngFlag::von_neumann_error) != 0u;
        if ((s & TrngFlag::recoverable) != 0u) {
            clear(s & TrngFlag::recoverable);
        }
        return e;
    }

    // ---- reading -------------------------------------------------------------

    /**
     * One attempt at 192 bits, with the protocol of 12.12.3 in the order
     * it is written: the fatal check first, then the two recoverable
     * ones (cleared here, so the next run is not judged by this one's
     * flag), then EHR_VALID, then the six words.
     *
     * The six are read 0..5 IN ORDER because reading EHR_DATA5 is what
     * clears the result registers - 12.12.3's own sentence - and a read
     * out of order would take a zero.
     */
    static std::optional<TrngEntropy> read() {
        const uint32_t s = status();
        if ((s & TrngFlag::autocorr_error) != 0u) {
            last_error_ = TrngError::autocorrelation;
            return std::nullopt;
        }
        if ((s & TrngFlag::recoverable) != 0u) {
            clear(s & TrngFlag::recoverable);
            ++recoverable_failures_;
            last_error_ = (s & TrngFlag::crngt_error) != 0u ? TrngError::crngt
                                                            : TrngError::von_neumann;
            return std::nullopt;
        }
        if ((s & TrngFlag::ehr_valid) == 0u) {
            last_error_ = TrngError::not_ready;
            return std::nullopt;
        }
        TrngEntropy e{};
        e.words[0] = regs().EHR_DATA0;
        e.words[1] = regs().EHR_DATA1;
        e.words[2] = regs().EHR_DATA2;
        e.words[3] = regs().EHR_DATA3;
        e.words[4] = regs().EHR_DATA4;
        e.words[5] = regs().EHR_DATA5;
        clear(TrngFlag::ehr_valid);
        return e;
    }

    /**
     * 192 bits or nothing, going round again on the two recoverable
     * checks: `spins` status reads at most, which is a BOUND and not a
     * promise of speed (see the file header). Nothing on
     * AUTOCORR_ERR - the block is stopped until `recover()` - and
     * nothing when the budget runs out with the collection still
     * running.
     */
    static std::optional<TrngEntropy> read_blocking(uint32_t spins = default_spins) {
        for (uint32_t i = 0; i < spins; ++i) {
            const auto e = read();
            if (e) {
                return e;
            }
            if (last_error_ == TrngError::autocorrelation) {
                return std::nullopt;
            }
        }
        last_error_ = TrngError::not_ready;
        return std::nullopt;
    }

    /// Why the last read gave nothing.
    static TrngError last_error() { return last_error_; }

    /// How many runs this block has discarded on a CRNGT or von Neumann
    /// check since `init()`. The block's own health, as a count.
    static uint32_t recoverable_failures() { return recoverable_failures_; }

    // ---- what the block says about itself -------------------------------------

    /// AUTOCORR_STATISTIC, as the two counters it packs: how many
    /// autocorrelation tests were started and how many failed. ANY write
    /// to the register resets both, and both stop counting at their
    /// limits (12.12.5).
    struct AutocorrStats {
        uint16_t tries = 0;   ///< 14 bits
        uint8_t failures = 0; ///< 8 bits
    };
    static AutocorrStats autocorr_stats() {
        const uint32_t v = regs().AUTOCORR_STATISTIC;
        return {
            .tries = static_cast<uint16_t>((v & TRNG_AUTOCORR_STATISTIC_AUTOCORR_TRYS_BITS) >>
                                           TRNG_AUTOCORR_STATISTIC_AUTOCORR_TRYS_LSB),
            .failures = static_cast<uint8_t>((v & TRNG_AUTOCORR_STATISTIC_AUTOCORR_FAILS_BITS) >>
                                             TRNG_AUTOCORR_STATISTIC_AUTOCORR_FAILS_LSB),
        };
    }
    static void clear_autocorr_stats() { regs().AUTOCORR_STATISTIC = 0; }

    /// The three built-in self-test counters, each a 22-bit ring
    /// oscillator count.
    static std::array<uint32_t, 3> bist() {
        return {
            regs().RNG_BIST_CNTR_0 & TRNG_RNG_BIST_CNTR_0_ROSC_CNTR_VAL_BITS,
            regs().RNG_BIST_CNTR_1 & TRNG_RNG_BIST_CNTR_1_ROSC_CNTR_VAL_BITS,
            regs().RNG_BIST_CNTR_2 & TRNG_RNG_BIST_CNTR_2_ROSC_CNTR_VAL_BITS,
        };
    }

    /// RNG_VERSION: which optional parts the IP was built with. Every bit
    /// is read-only, and the one that matters to a caller is the width.
    static uint32_t version() { return regs().RNG_VERSION; }
    static bool ehr_is_192_bits() {
        return (regs().RNG_VERSION & TRNG_RNG_VERSION_EHR_WIDTH_192_BITS) != 0u;
    }
    static bool has_autocorrelation() {
        return (regs().RNG_VERSION & TRNG_RNG_VERSION_AUTOCORR_EXISTS_BITS) != 0u;
    }
    static bool has_crngt() {
        return (regs().RNG_VERSION & TRNG_RNG_VERSION_CRNGT_EXISTS_BITS) != 0u;
    }

    /// The settings as they read back, for a suite that wants to say
    /// what it measured with.
    static uint8_t chain() { return static_cast<uint8_t>(regs().TRNG_CONFIG & 0x3u); }
    static uint32_t sample_cycles() { return regs().SAMPLE_CNT1; }
    static uint32_t debug_control() { return regs().TRNG_DEBUG_CONTROL; }

    /// RNG_DEBUG_EN_INPUT. The IP's debug mode, off at reset and left off
    /// by this driver: the verb exists so that a program can read the bit
    /// and say it is clear.
    static bool debug_mode() {
        return (regs().RNG_DEBUG_EN_INPUT & TRNG_RNG_DEBUG_EN_INPUT_RNG_DEBUG_EN_BITS) != 0u;
    }

private:
    static inline TrngError last_error_ = TrngError::not_ready;
    static inline uint32_t recoverable_failures_ = 0;
};

} // namespace brio
