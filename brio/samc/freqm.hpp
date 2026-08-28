/*
 * freqm.hpp
 *
 * The SAM C21 Frequency Meter (DS60001479M ch. 44): a hardware ratio
 * counter that measures one generic clock against another. There is no
 * AVR analog - on that family a clock is measured by counting its edges
 * in a timer, with the CPU in the loop - and it earns its place here
 * because everything the clock work ahead has to characterize (the
 * 32 kHz oscillators, the FDPLL, a crystal against the internal RC) is
 * exactly a ratio between two generators.
 *
 * WHAT THE SILICON DOES, in one formula and three constraints.
 *
 *   f_msr = VALUE / REFNUM x f_ref
 *
 * The block counts periods of the MEASURED clock for REFNUM periods of
 * the REFERENCE clock and leaves the count in VALUE (44.6.1). Both
 * clocks are generic clock channels of their own - GCLK_FREQM_MSR is
 * channel 3, GCLK_FREQM_REF channel 4 - so choosing what to measure and
 * what to measure it against is a GCLK routing question, not a FREQM
 * one, and this header takes generator numbers rather than sources.
 *
 * 1. THE REFERENCE MUST BE SLOWER THAN THE MEASURED CLOCK. The chapter
 *    says so in a call-out box (44.6.2.1) and the arithmetic says why:
 *    VALUE counts measured periods per reference period, so a reference
 *    faster than the measurand counts zero. Nothing in the hardware
 *    refuses it; `measure()` returns nothing rather than a wrong number,
 *    because a VALUE of zero and a genuinely stopped clock are the same
 *    reading.
 *
 * 2. VALUE IS 24 BITS, AND THE OVERFLOW IS THE CALLER'S TO AVOID.
 *    REFNUM x (f_msr/f_ref) must stay under 2^24, and STATUS.OVF says
 *    when it did not. Longer measurements are more precise and closer to
 *    overflowing, which is the whole trade: `refnum_for()` picks the
 *    largest REFNUM that stays safe for an expected ratio, and CFGA's
 *    DIVREF - a divide-by-8 on the reference alone - buys another
 *    eightfold when the ratio is small.
 *
 * 3. ERRATUM 1.24.1, LIVE ON EVERY SILICON REVISION INCLUDING THIS ONE,
 *    with no workaround offered: READING CTRLB RAISES A PAC PROTECTION
 *    ERROR. So CTRLB is never read here - not in a read-modify-write,
 *    not to check whether a measurement was started, not at all. START
 *    is a plain store of a single bit, which is all it ever needed to
 *    be, and the device header agrees: it declares CTRLB `__O`. The
 *    consequence worth stating is that there is NO WAY TO ASK the block
 *    whether a start was accepted; STATUS.BUSY and INTFLAG.DONE are the
 *    only honest evidence a measurement is running or finished.
 *
 * PRECISION, since a frequency meter that does not say how well it
 * measures is decoration. The count is a whole number of measured
 * periods over a whole number of reference periods, so the answer is
 * granular to about f_msr/VALUE - one part in VALUE. Measuring 48 MHz
 * against a 1 kHz reference with REFNUM = 1 gives VALUE around 48000 and
 * therefore about 20 ppm of granularity; REFNUM = 255 would give one
 * part in 12 million and overflow long before. The reference's OWN
 * accuracy is a separate matter and is not improved by counting longer:
 * this block measures a RATIO, and the absolute number it is turned into
 * is only ever as good as f_ref.
 *
 * NOT BUILT (docs/samc/freqm.md carries the list): the DONE interrupt is
 * exposed as flags and an ISR body but nothing here waits on it - the
 * measurements this serves are short and polled; and the GCLK_IO pins as
 * measurement sources (44.5.1), which need a pin claim this header does
 * not make.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "sam.h"

#include "samc/clock.hpp"

namespace brio {

/// INTFLAG / INTENSET / INTENCLR: the block has exactly one interrupt.
struct FreqmFlag {
    static constexpr uint8_t done = FREQM_INTFLAG_DONE_Msk;
};

/**
 * The frequency meter's configuration.
 *
 * `measured_generator` and `reference_generator` are GCLK GENERATOR
 * numbers, not sources: whatever generator 5 has been pointed at is what
 * gets measured. That indirection is the peripheral's own shape and it
 * is what makes the block general - anything a generator can be sourced
 * from is something this can measure.
 */
struct FreqmConfig {
    uint8_t measured_generator = 0;
    uint8_t reference_generator = 1;

    /// CFGA.REFNUM: how many reference periods one measurement lasts,
    /// 1..255. Zero is refused - the chapter requires a non-zero value
    /// and the arithmetic would divide by it.
    uint8_t refnum = 1;

    /// CFGA.DIVREF as 44.8.3 draws it - a divide-by-8 on the reference -
    /// AND IT IS REFUSED, because this silicon does not have it. Kept as
    /// vocabulary so the refusal has something to name; see Freqm::
    /// cfga_divref for what was measured.
    bool divide_reference = false;
};

/**
 * FREQM as a monostate resource.
 *
 * NOTE the division of labour with samc/clock.hpp: this header enables
 * the block's APB clock and connects its two GCLK CHANNELS to the
 * generators it is told about, but it never configures a generator. What
 * a generator is sourced from, and whether it is running, is the clock
 * driver's business and the caller's decision.
 */
struct Freqm {
    Freqm() = delete;

    /// The two generic clock channels, from the device header rather
    /// than from a formula (they are not adjacent to anything).
    static constexpr uint8_t gclk_measured = FREQM_GCLK_ID_MSR;   // 3
    static constexpr uint8_t gclk_reference = FREQM_GCLK_ID_REF;  // 4

    static constexpr IRQn_Type irq() { return FREQM_IRQn; }

    /// VALUE is 24 bits; a product past this is the overflow STATUS.OVF
    /// reports.
    static constexpr uint32_t value_max = 0xFFFFFFUL;

    /**
     * CFGA.DIVREF as chapter 44 draws it - AND THIS SILICON DOES NOT
     * HAVE IT. The bit is defined here so that the refusal below has
     * something to name, and so that the finding is recorded where
     * anyone reading 44.8.3 will look.
     *
     * The two documents disagreed: `FREQM_CFGA_Msk` is 0x00FF in the
     * device header, as though CFGA were eight bits wide, while 44.8.3
     * draws sixteen and puts DIVREF at bit 15 with a description
     * ("Divides the reference clock by 8"). The bench settled it twice
     * over (test_samc_freqm letter c): CFGA written with bit 15 and
     * REFNUM 1 READS BACK 0x0001 - the bit does not even stay written -
     * and setting it changes no measurement. The header is right, the
     * chapter's drawing is not, and `config_valid()` refuses a
     * configuration that asks for a divider that is not there rather
     * than accepting one that would silently do nothing.
     */
    static constexpr uint16_t cfga_divref = static_cast<uint16_t>(1u << 15);

    static constexpr bool config_valid(const FreqmConfig& c) {
        return c.refnum != 0u && c.measured_generator < GCLK_GEN_NUM &&
               c.reference_generator < GCLK_GEN_NUM &&
               c.measured_generator != c.reference_generator &&
               !c.divide_reference;   // measured absent - see cfga_divref
    }

    /// The largest REFNUM whose count still fits in VALUE for a ratio of
    /// `expected_ratio` measured periods per reference period. Returns 1
    /// when even a single reference period would overflow - the caller
    /// then needs a faster reference, which is the chapter's own advice
    /// (44.6.2.3).
    static constexpr uint8_t refnum_for(uint32_t expected_ratio) {
        if (expected_ratio == 0u) {
            return 255u;
        }
        const uint32_t n = value_max / expected_ratio;
        if (n == 0u) {
            return 1u;
        }
        return n > 255u ? 255u : static_cast<uint8_t>(n);
    }

    // ---- clocks and reset --------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_FREQM_Msk, on); }

    static constexpr uint32_t sync_mask =
        FREQM_SYNCBUSY_SWRST_Msk | FREQM_SYNCBUSY_ENABLE_Msk;

    static bool busy_sync(uint32_t mask = sync_mask) {
        return (FREQM_REGS->FREQM_SYNCBUSY & mask) != 0u;
    }
    static bool wait_sync(uint32_t mask = sync_mask, uint32_t spins = 0xFFFFu) {
        while (busy_sync(mask) && spins-- != 0u) {
        }
        return !busy_sync(mask);
    }

    static bool reset(uint32_t spins = 0xFFFFu) {
        FREQM_REGS->FREQM_CTRLA = FREQM_CTRLA_SWRST_Msk;
        return wait_sync(sync_mask, spins);
    }

    static bool enabled() {
        return (FREQM_REGS->FREQM_CTRLA & FREQM_CTRLA_ENABLE_Msk) != 0u;
    }
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        FREQM_REGS->FREQM_CTRLA =
            on ? FREQM_CTRLA_ENABLE_Msk : static_cast<uint8_t>(0);
        return wait_sync(FREQM_SYNCBUSY_ENABLE_Msk, spins);
    }

    // ---- configuration -----------------------------------------------------

    /**
     * Claim the block: APB clock on, reset, both GCLK channels pointed
     * at the generators the config names, CFGA written and the block
     * enabled. False when the configuration is impossible or a
     * synchronization never completed; nothing is left half-claimed that
     * `release()` cannot undo.
     *
     * CFGA IS ENABLE-PROTECTED (44.6.2.1), so it is written between the
     * reset and the enable, in that order and not another.
     */
    static bool init(const FreqmConfig& cfg, uint32_t spins = 0xFFFFu) {
        if (!config_valid(cfg)) {
            return false;
        }
        bus_clock(true);
        // THE CHANNELS BEFORE THE RESET, and the order is not cosmetic:
        // SWRST is write-synchronized into the peripheral's own clock
        // domain, and that domain is fed by these two channels. Resetting
        // a block whose generic clocks are not yet routed leaves
        // SYNCBUSY.SWRST standing forever - which is exactly how the
        // first version of this driver failed on the bench, with every
        // measurement returning nothing.
        if (!GclkChannel::connect(gclk_measured, cfg.measured_generator, spins) ||
            !GclkChannel::connect(gclk_reference, cfg.reference_generator, spins)) {
            return false;
        }
        if (!reset(spins)) {
            return false;
        }
        FREQM_REGS->FREQM_CFGA =
            static_cast<uint16_t>(FREQM_CFGA_REFNUM(cfg.refnum));
        return enable(true, spins);
    }

    /// Hand everything back: the block disabled and reset, both channels
    /// disconnected, the APB clock off.
    static void release(uint32_t spins = 0xFFFFu) {
        (void)enable(false, spins);
        (void)reset(spins);
        GclkChannel::disconnect(gclk_measured);
        GclkChannel::disconnect(gclk_reference);
        bus_clock(false);
    }

    // ---- status and flags --------------------------------------------------

    static uint8_t status() { return FREQM_REGS->FREQM_STATUS; }
    static bool running() { return (status() & FREQM_STATUS_BUSY_Msk) != 0u; }
    static bool overflowed() { return (status() & FREQM_STATUS_OVF_Msk) != 0u; }
    /// STATUS.OVF is write-one-to-clear and STATUS.BUSY is not writable.
    static void clear_overflow() { FREQM_REGS->FREQM_STATUS = FREQM_STATUS_OVF_Msk; }

    static uint8_t flags() { return FREQM_REGS->FREQM_INTFLAG; }
    static uint8_t armed() { return FREQM_REGS->FREQM_INTENSET; }
    static void clear_flags(uint8_t mask = FreqmFlag::done) {
        FREQM_REGS->FREQM_INTFLAG = mask;
    }
    static void arm(uint8_t mask) { FREQM_REGS->FREQM_INTENSET = mask; }
    static void disarm(uint8_t mask) { FREQM_REGS->FREQM_INTENCLR = mask; }
    static bool done_flag() { return (flags() & FreqmFlag::done) != 0u; }

    /// The ISR body; the app binds FREQM_Handler. Returns what it
    /// acknowledged.
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t p = static_cast<uint8_t>(flags() & armed());
        if (p != 0u) {
            clear_flags(p);
        }
        return p;
    }

    // ---- measuring ---------------------------------------------------------

    /// The raw count of the last measurement: measured periods per
    /// REFNUM reference periods.
    static uint32_t value() {
        return FREQM_REGS->FREQM_VALUE & FREQM_VALUE_VALUE_Msk;
    }

    /**
     * Start a measurement.
     *
     * CTRLB IS WRITTEN AND NEVER READ - erratum 1.24.1 makes a read a
     * PAC protection error on every silicon revision - so this is a
     * plain store and there is nothing to check afterwards but BUSY and
     * DONE.
     */
    static void start() {
        clear_flags();
        FREQM_REGS->FREQM_CTRLB = FREQM_CTRLB_START_Msk;
    }

    /**
     * Start a measurement, wait for it, and return the raw count -
     * nothing on an overflow or a wait that never ended.
     *
     * The bound is in SPINS and not in time on purpose: how long a
     * measurement takes is REFNUM reference periods, which is the
     * caller's own choice and can be anything from microseconds to
     * seconds. A caller measuring against a slow reference passes a
     * bigger bound; the default suits a reference in the kilohertz.
     */
    static std::optional<uint32_t> measure(uint32_t spins = 20'000'000UL) {
        if (!enabled()) {
            return std::nullopt;
        }
        clear_overflow();
        start();
        while (!done_flag() && spins-- != 0u) {
        }
        if (!done_flag()) {
            return std::nullopt;
        }
        clear_flags();
        if (overflowed()) {
            return std::nullopt;
        }
        return value();
    }

    /**
     * The measurement turned into hertz: VALUE / REFNUM x f_ref, with
     * DIVREF folded in when it is set.
     *
     * `reference_hz` is the caller's knowledge and not the block's -
     * FREQM measures a RATIO, and the absolute answer is only ever as
     * good as the reference the caller names.
     *
     * The arithmetic keeps its width: VALUE is 24 bits and a reference
     * in the tens of megahertz would overflow a 32-bit product, so the
     * multiply is done in 64 bits and narrowed once.
     */
    static constexpr uint32_t to_hz(uint32_t value_count, uint32_t reference_hz,
                                    uint8_t refnum, bool divide_reference = false) {
        if (refnum == 0u) {
            return 0;
        }
        const uint64_t ref = divide_reference ? (reference_hz / 8u) : reference_hz;
        return static_cast<uint32_t>(
            (static_cast<uint64_t>(value_count) * ref) / refnum);
    }
};

} // namespace brio
