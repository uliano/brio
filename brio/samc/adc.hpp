/*
 * adc.hpp
 *
 * The SAM C21's two 12-bit SAR converters (DS60001479M ch. 38) as one
 * resource template, `Adc<n>` - ADC0 and ADC1 are the SAME peripheral at
 * two addresses, so there is one type and the instance number carries
 * every difference the silicon has (which pads, which GCLK channel,
 * which EVSYS codes, which half of the host/client pair).
 *
 *   using Meter = brio::Adc<0>;
 *   Meter::init(gclk_generator, brio::AdcConfig{
 *       .reference = brio::Ref::vddana,
 *       .prescaler = brio::AdcPresc::div32,          // 48 MHz / 32 = 1.5 MHz
 *       .sample_length = 5});
 *   Meter::select(brio::AnalogIn<brio::Pin<'A', 8>>{});   // ADC0/AIN8
 *   const uint16_t raw = Meter::read();
 *   const uint16_t mv  = brio::adc_mv(raw, Meter::result_steps(), 5100);
 *
 * ONE TYPE WITH KNOBS, for the reason avrdx/adc.hpp gives and this
 * chapter repeats: every use - a single reading, a paced stream, an
 * oversampled reading, a window watch, a supply monitor - is the same
 * sequence (select, trigger, wait RESRDY, read RESULT) with different
 * knobs and different consumers of the same two flags.
 *
 * ---------------------------------------------------------------------
 * THE FOUR REGISTER DISCIPLINES, spelled per register because this
 * chapter mixes all four and the mixture is where the traps are.
 *
 * 1. ENABLE-PROTECTED (38.6.2.1) - writable only with CTRLA.ENABLE = 0,
 *    and NOT synchronized: CTRLB (the prescaler), REFCTRL, EVCTRL,
 *    CALIB. Every verb here that touches one of them REFUSES while the
 *    converter is enabled rather than storing into a register the
 *    silicon ignores.
 * 2. WRITE-SYNCHRONIZED (38.6.8) - CTRLA.SWRST, CTRLA.ENABLE,
 *    INPUTCTRL, CTRLC, AVGCTRL, SAMPCTRL, WINLT, WINUT, GAINCORR,
 *    OFFSETCORR, SWTRIG. Every wait in this file is bounded and
 *    reported, like every wait in this stratum.
 * 3. DOUBLE-BUFFERED (38.6.3.3) - and this is the one that surprises:
 *    INPUTCTRL, CTRLC, AVGCTRL, SAMPCTRL, WINLT, WINUT, GAINCORR and
 *    OFFSETCORR are ALSO buffered, so a write made during a conversion
 *    is held until the next RESULT and its SYNCBUSY bit STAYS SET for
 *    the whole conversion. A caller that spins on SYNCBUSY after
 *    changing the input mid-stream is waiting for the conversion, not
 *    for a bus. That is why `select()` is a plain void store with no
 *    wait (it is exactly what a buffered register wants) and
 *    `select_sync()` is the separate verb for a caller who needs the
 *    change to be in force before it returns.
 * 4. NEITHER - DBGCTRL, which is also NOT reset by CTRLA.SWRST
 *    (38.8.18), and SEQCTRL.
 *
 * ---------------------------------------------------------------------
 * THE RESSEL / AVGCTRL INTERPLAY, the chapter's classic trap.
 *
 * Accumulating or averaging more than one sample REQUIRES
 * CTRLC.RESSEL = 16BIT (38.6.2.9 and 38.6.2.10 both say so in a Note,
 * and 38.8.11 repeats it). A configuration that asks for
 * AVGCTRL.SAMPLENUM > 1 sample at 12-, 10- or 8-bit resolution is
 * refused here, at compile time in the `init<cfg>()` form.
 *
 * What the two AVGCTRL fields do together:
 *  - SAMPLENUM picks N = 1..1024 samples summed into one RESULT;
 *  - above 16 samples the hardware ALREADY right-shifts the sum to keep
 *    it inside the 16-bit RESULT register (table 38-1);
 *  - ADJRES is an ADDITIONAL right shift the software asks for, and it
 *    is what turns an accumulation into an average (table 38-2) or into
 *    an oversampled higher-resolution reading (table 38-3).
 * So the full scale of a RESULT is (base << log2(N)) >> (auto + ADJRES),
 * which `result_steps()` computes and `adc_result_steps()` computes at
 * compile time - and which is the number util/analog.hpp's `adc_mv()`
 * wants. The two helpers `adc_adjres_for_average()` and
 * `adc_adjres_for_oversampling()` fill ADJRES from the chapter's own
 * two tables so a caller never copies them by hand.
 *
 * ---------------------------------------------------------------------
 * CALIBRATION IS NOT OPTIONAL. 38.5.10: "The BIAS and LINEARITY
 * calibration values from the production test MUST be loaded from the
 * NVM Software Calibration Area into the ADC Calibration register
 * (CALIB) by software to achieve specified accuracy." The device header
 * says the same thing per instance (`ADCn_LOAD_CALIB`), and
 * samc/nvm.hpp has typed those two fields per converter since its own
 * campaign. So `init()` copies them, always, and `calibration_loaded()`
 * reports what stands in CALIB. Nothing here computes a calibration
 * value; the row is the only authority and the chapter says to copy it
 * and not to change it.
 *
 * ---------------------------------------------------------------------
 * THE CLOCK. CLK_ADCn = GCLK_ADCn / 2^(1 + CTRLB.PRESCALER), minimum
 * divider 2 (38.6.2.4). Table 45-22 bounds f_adc at 160 kHz .. 16 MHz,
 * so at a 48 MHz generator DIV2 is ILLEGAL (24 MHz) and DIV4 is the
 * fastest legal setting - `adc_clock_in_range()` says so and `init()`
 * refuses a prescaler that leaves the range. There is no `rebase()` and
 * no ClockUser here: on this family the converter has its OWN generic
 * clock channel, so a main-clock change does not move CLK_ADC at all -
 * the AVR's fan-out has nothing to fan out to (docs/samc/clock.md, the
 * DynamicClock ruling).
 *
 * INTREF NEEDS A LONG SAMPLE. 38.8.9's MUXPOS description: "If the
 * internal INTREF voltage input channel is selected, then the Sampling
 * Time Length bit group in the Sampling Control register must be written
 * with a corresponding value", and table 45-22 puts that value at
 * 10 us MINIMUM. `adc_samplen_for_ns()` turns a wanted sampling time
 * into a SAMPLEN at a given CLK_ADC, and `adc_intref_sampling_ns` is
 * the 10 us the table demands. Nothing enforces it - the driver does not
 * know CLK_ADC at the moment `select()` is called - so it is stated
 * here and measured in the suite.
 *
 * ---------------------------------------------------------------------
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F. Ten ADC
 * items; FIVE are this silicon.
 *
 *  - 1.4.4 Synchronized Event (all revisions): a SYNCHRONIZED event
 *    arriving during a conversion is never acknowledged and STALLS THE
 *    EVENT CHANNEL. The workaround is "only the asynchronous path from
 *    the Event System must be used" - which table 29-3 independently
 *    requires for both ADC users anyway. This is CODE here:
 *    `start_on()` and `flush_on()` refuse a channel configuration whose
 *    path is not asynchronous.
 *  - 1.4.5 Software Trigger Sync Busy (all revisions):
 *    SYNCBUSY.SWTRIG becomes STUCK AT ONE after a wake from standby, and
 *    the errata's own instruction is to ignore it - "The ADC result can
 *    be read after INTFLAG.RESRDY is set. To start the next conversion,
 *    write a '1' to SWTRIG.START." So `start()` and `flush()` are plain
 *    stores that WAIT FOR NOTHING, and `ready()`/`result()` are the only
 *    progress the rest of this file believes.
 *  - 1.4.6 Reference Buffer Offset Compensation (all revisions): with
 *    REFCTRL.REFCOMP = 1 and any reference other than VDDANA, the TUE of
 *    the first conversions is out of specification and "the first five
 *    conversions after enabling ADC must be ignored". `init()` spends
 *    them - `discard(5)` - exactly when that combination is configured,
 *    and `warm_up_conversions()` reports how many it decided to spend.
 *  - 1.4.9 Sequence State (all revisions): SEQSTATUS is not updated for
 *    the first conversion of a sequence that EXITS STANDBY. Stated on
 *    `sequence_state()`; there is nothing to write.
 *  - 1.4.10 Syncbusy Enable (all revisions): enabling ADC1 while ADC0 is
 *    disabled can leave ADC0.SYNCBUSY.ENABLE stuck at one. The
 *    workaround is "enable ADC0 before ADC1, or disregard the bit", and
 *    it is stated on `enable()`: each instance waits on ITS OWN
 *    SYNCBUSY, which the erratum does not touch, so nothing here hangs -
 *    but a caller reading ADC0's must know.
 *  - NOT this silicon: 1.4.1 (SWTRIG.START never clears), 1.4.2 (the LSB
 *    stuck at zero at 8 and 10 bits) and 1.4.3 (the window monitor
 *    keeping GCLK alive) are revision B only; 1.4.7 (differential and
 *    single-ended electrical characteristics) and 1.4.8 (power
 *    consumption) are revisions B..E. The device-level 1.8.2 - the AC
 *    having to borrow GCLK_ADC1 because GCLK_AC is dead - is revision B
 *    only, which is why samc/ac.hpp uses AC_GCLK_ID. 1.8.9 (the DAC
 *    output as MUXPOS makes both the DAC and the reading noisy, all
 *    revisions) is live but has no consumer here: there is no DAC driver
 *    in this stratum yet.
 *
 * ---------------------------------------------------------------------
 * THE HOST/CLIENT PAIR (38.6.3.1) is real and typed: ADC1 is the only
 * instance whose CTRLA.SLAVEEN exists (`ADC1_MASTER_SLAVE_MODE` = 2 in
 * the device header), ADC0 the only one whose CTRLC.DUALSEL means
 * anything (`ADC0_MASTER_SLAVE_MODE` = 1). Asking for either on the
 * wrong instance is refused, at compile time in the `init<cfg>()` form.
 *
 * NOT BUILT (docs/samc/adc.md carries the list): sleep behaviour beyond
 * the two CTRLA bits (the 38.6.7 table has an owner in util/power.hpp
 * but no bench leg here), the DAC as a reference or as an input (no DAC
 * driver), VREFA (needs a pin this board does not drive), and the
 * temperature sensor, which is the separate TSENS peripheral (ch. 43) on
 * this family and not an ADC channel.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/supc.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// The reference vocabulary
// =============================================================================

/**
 * REFCTRL.REFSEL (38.8.3) - THIS TARGET'S `brio::Ref`.
 *
 * util/analog.hpp asks every target for an enum of this name plus a
 * `ref_mv()`, because the arithmetic is target-independent and the
 * levels never are. On the AVR DA/DB the set is a shared VREF block's
 * four internal levels; here it is a per-converter multiplexer over the
 * bandgap, two divisions of the analog supply, the supply itself, an
 * external pin and the DAC.
 */
enum class Ref : uint8_t {
    /// INTREF: the SUPC bandgap, whose LEVEL is chosen in SUPC.VREF.SEL
    /// (samc/supc.hpp's `VrefLevel`) and not here. NOTE what is measured
    /// and what is not: the bandgap as an INPUT (MUXPOS INTREF) reads a
    /// flat ZERO until SUPC.VREF.VREFOE is set - measured, and neither
    /// chapter states the link - while the bandgap as a REFERENCE (this
    /// code) has NOT been exercised on the bench at all (docs/samc/adc.md,
    /// "Not covered yet"). Until it is, set VREFOE before trusting it.
    intref = ADC_REFCTRL_REFSEL_INTREF_Val,
    /// 1/1.6 of VDDANA.
    vddana_div1p6 = ADC_REFCTRL_REFSEL_INTVCC0_Val,
    /// 1/2 of VDDANA - the register description restricts it to
    /// VDDANA > 4.0 V.
    vddana_div2 = ADC_REFCTRL_REFSEL_INTVCC1_Val,
    /// The VREFA pin.
    vrefa = ADC_REFCTRL_REFSEL_AREFA_Val,
    /// The DAC's output. No DAC driver exists in this stratum yet.
    dac = ADC_REFCTRL_REFSEL_DAC_Val,
    /// VDDANA itself (the register calls it INTVCC2).
    vddana = ADC_REFCTRL_REFSEL_INTVCC2_Val,
};

/// Whether a REFSEL code is one the silicon implements (0x6..0xF are
/// Reserved).
constexpr bool ref_valid(Ref r) {
    const uint8_t v = static_cast<uint8_t>(r);
    return v <= static_cast<uint8_t>(Ref::vddana);
}

/**
 * The millivolts of a reference, for util/analog.hpp's arithmetic.
 *
 * `known_mv` is the millivolts of the reference's SOURCE where this
 * header cannot know it: VDDANA for the three supply-derived codes, the
 * pin's voltage for `vrefa`, the DAC's output for `dac`. Zero means "not
 * known", and a conversion against it yields zero rather than a
 * plausible lie. `intref_level` is what stands in SUPC.VREF.SEL.
 *
 * NOTE the 1/1.6 division is exact as 5/8, so it is spelled that way and
 * not as a float.
 */
constexpr uint16_t ref_mv(Ref r, uint16_t known_mv = 0,
                          VrefLevel intref_level = VrefLevel::v1_024) {
    switch (r) {
    case Ref::intref: return vref_mv(intref_level);
    case Ref::vddana_div1p6: return static_cast<uint16_t>((known_mv * 5u + 4u) / 8u);
    case Ref::vddana_div2: return static_cast<uint16_t>((known_mv + 1u) / 2u);
    case Ref::vrefa:
    case Ref::dac:
    case Ref::vddana: return known_mv;
    }
    return 0;
}

// =============================================================================
// The knobs
// =============================================================================

/// CTRLB.PRESCALER (38.8.2): CLK_ADC = GCLK_ADC / 2^(1 + code).
enum class AdcPresc : uint8_t {
    div2 = 0, div4, div8, div16, div32, div64, div128, div256
};

constexpr uint16_t adc_presc_divisor(AdcPresc p) {
    return static_cast<uint16_t>(2u << static_cast<uint8_t>(p));
}

/**
 * CTRLC.RESSEL (38.8.10).
 *
 * `bits16` is the register's own 16BIT code and it is NOT a sixteen-bit
 * conversion: the converter still makes 12-bit samples, and this code
 * says "the RESULT register carries an accumulated or oversampled value
 * up to 16 bits wide". It is MANDATORY for any AVGCTRL.SAMPLENUM above
 * one sample and pointless below it.
 */
enum class AdcRes : uint8_t {
    bits12 = ADC_CTRLC_RESSEL_12BIT_Val,
    bits16 = ADC_CTRLC_RESSEL_16BIT_Val,
    bits10 = ADC_CTRLC_RESSEL_10BIT_Val,
    bits8 = ADC_CTRLC_RESSEL_8BIT_Val,
};

/// The full scale of ONE sample at a resolution. `bits16` is 12-bit
/// samples accumulated, so its base is the 12-bit one.
constexpr uint32_t adc_sample_steps(AdcRes r) {
    switch (r) {
    case AdcRes::bits12:
    case AdcRes::bits16: return 4096;
    case AdcRes::bits10: return 1024;
    case AdcRes::bits8: return 256;
    }
    return 4096;
}

/// The number of BITS one sample carries (the nDATA of 38.6.2.8's
/// conversion-time formula). `bits16` converts at 12 bits.
constexpr uint8_t adc_sample_bits(AdcRes r) {
    switch (r) {
    case AdcRes::bits12:
    case AdcRes::bits16: return 12;
    case AdcRes::bits10: return 10;
    case AdcRes::bits8: return 8;
    }
    return 12;
}

/// AVGCTRL.SAMPLENUM (38.8.11): how many samples are summed into one
/// RESULT. 0xB..0xF are Reserved and have no enumerator.
enum class AdcAverage : uint8_t {
    samples1 = 0, samples2, samples4, samples8, samples16, samples32,
    samples64, samples128, samples256, samples512, samples1024
};

constexpr uint16_t adc_average_count(AdcAverage a) {
    return static_cast<uint16_t>(1u << static_cast<uint8_t>(a));
}

/// The AUTOMATIC right shifts the hardware applies above 16 samples to
/// keep the sum inside the 16-bit RESULT register (table 38-1).
constexpr uint8_t adc_auto_shift(AdcAverage a) {
    const uint8_t n = static_cast<uint8_t>(a);
    return n > 4u ? static_cast<uint8_t>(n - 4u) : 0u;
}

/// AVGCTRL.ADJRES for a true AVERAGE - table 38-2's own column, which is
/// min(SAMPLENUM, 4): the result comes back at 12-bit precision whatever
/// N is.
constexpr uint8_t adc_adjres_for_average(AdcAverage a) {
    const uint8_t n = static_cast<uint8_t>(a);
    return n > 4u ? 4u : n;
}

/**
 * AVGCTRL.ADJRES for OVERSAMPLING AND DECIMATION - table 38-3, which is
 * a different column from the averaging one and the reason both helpers
 * exist. `extra_bits` is 1..4 (13..16-bit results); the matching sample
 * count is `adc_oversampling_average()`.
 */
constexpr uint8_t adc_adjres_for_oversampling(uint8_t extra_bits) {
    switch (extra_bits) {
    case 1: return 1;   // 13 bits, N = 4,   auto 0
    case 2: return 2;   // 14 bits, N = 16,  auto 0
    case 3: return 1;   // 15 bits, N = 64,  auto 2
    case 4: return 0;   // 16 bits, N = 256, auto 4
    default: return 0;
    }
}

/// The sample count table 38-3 pairs with `extra_bits`: 4^n.
constexpr AdcAverage adc_oversampling_average(uint8_t extra_bits) {
    return static_cast<AdcAverage>(2u * extra_bits);
}

/// CTRLC.WINMODE (38.8.10). The names are the conditions, not the
/// register's MODE1..MODE4.
enum class AdcWindow : uint8_t {
    none = ADC_CTRLC_WINMODE_DISABLE_Val,
    above_lower = ADC_CTRLC_WINMODE_MODE1_Val,    ///< RESULT > WINLT
    below_upper = ADC_CTRLC_WINMODE_MODE2_Val,    ///< RESULT < WINUT
    inside = ADC_CTRLC_WINMODE_MODE3_Val,         ///< WINLT < RESULT < WINUT
    /**
     * MODE4, and THE TWO DOCUMENTS DISAGREE ABOUT WHAT IT MEANS: 38.8.10
     * prints "WINUT < RESULT < WINLT" (a band with the thresholds
     * swapped, which is a reject band only when WINUT < WINLT), while
     * the device header's own comment on the same value reads
     * "!(WINLT < RESULT < WINUT)" - the plain complement of MODE3. They
     * differ for every ordering of the two thresholds, so this is a
     * silicon question and docs/samc/adc.md carries the measured answer.
     */
    outside = ADC_CTRLC_WINMODE_MODE4_Val,
};

/// CTRLC.DUALSEL (38.8.10) - the HOST's knob, meaningless unless the
/// client has SLAVEEN set.
/// The device header declares the FIELD but names neither value (0x2 and
/// 0x3 are Reserved), so these two come from 38.8.10's own table.
enum class AdcDual : uint8_t {
    both = 0,
    interleave = 1,
};

/// INPUTCTRL.MUXPOS, the codes that are not an AIN pad (38.8.9).
enum class AdcInput : uint8_t {
    /// The SUPC bandgap. Table 45-22 asks for at least 10 us of sampling
    /// on this channel - see `adc_samplen_for_ns()`.
    intref = ADC_INPUTCTRL_MUXPOS_BANDGAP_Val,
    /// 1/4 of VDDCORE, the regulated core supply.
    scaled_core = ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val,
    /// 1/4 of VDDANA, the analog/IO supply. With `Ref::vddana` this
    /// channel must read a quarter of full scale exactly, which makes it
    /// the cheapest self-check the converter has.
    scaled_supply = ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val,
    /**
     * The DAC output. THE DOCUMENTS DISAGREE and the device header
     * wins by house rule: 38.8.9's MUXPOS table marks 0x1C..0x1F
     * Reserved, while the header declares
     * `ADC_INPUTCTRL_MUXPOS_DAC_Val` = 0x1C and erratum 1.8.9 - filed
     * against the E/G/J row at every revision - describes exactly this
     * selection misbehaving, which no Reserved code could. There is no
     * DAC driver in this stratum, so nothing here can validate it.
     */
    dac = ADC_INPUTCTRL_MUXPOS_DAC_Val,
};

/// INPUTCTRL.MUXNEG (38.8.9). ONLY AIN0..AIN5 and ground - the negative
/// multiplexer is SIX pads wide where the positive one is twelve, and
/// 0x06..0x17 and 0x19..0x1F are Reserved.
enum class AdcNegative : uint8_t {
    ain0 = 0, ain1, ain2, ain3, ain4, ain5,
    ground = ADC_INPUTCTRL_MUXNEG_GND_Val,
};

constexpr bool adc_negative_valid(AdcNegative n) {
    return static_cast<uint8_t>(n) <= 5u || n == AdcNegative::ground;
}

/// Whether a MUXPOS code is one the silicon implements.
constexpr bool adc_muxpos_valid(uint8_t code) {
    return code <= 0x0Bu || code == static_cast<uint8_t>(AdcInput::intref) ||
           code == static_cast<uint8_t>(AdcInput::scaled_core) ||
           code == static_cast<uint8_t>(AdcInput::scaled_supply) ||
           code == static_cast<uint8_t>(AdcInput::dac);
}

// ---- the clock arithmetic ---------------------------------------------------

/// CLK_ADC = GCLK_ADC / 2^(1 + PRESCALER) (38.6.2.8).
constexpr uint32_t adc_clock_hz(uint32_t gclk_hz, AdcPresc p) {
    return gclk_hz / adc_presc_divisor(p);
}

/// Table 45-22's f_adc bounds.
inline constexpr uint32_t adc_clock_min_hz = 160'000;
inline constexpr uint32_t adc_clock_max_hz = 16'000'000;

constexpr bool adc_clock_in_range(uint32_t gclk_hz, AdcPresc p) {
    const uint32_t f = adc_clock_hz(gclk_hz, p);
    return f >= adc_clock_min_hz && f <= adc_clock_max_hz;
}

/// The prescaler putting CLK_ADC closest to `target_hz` while staying in
/// range; the slowest in-range one if nothing is close.
constexpr AdcPresc adc_presc_for(uint32_t gclk_hz, uint32_t target_hz) {
    AdcPresc best = AdcPresc::div256;
    uint32_t best_err = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8u; ++i) {
        const AdcPresc p = static_cast<AdcPresc>(i);
        if (!adc_clock_in_range(gclk_hz, p)) {
            continue;
        }
        const uint32_t f = adc_clock_hz(gclk_hz, p);
        const uint32_t err = f > target_hz ? f - target_hz : target_hz - f;
        if (err < best_err) {
            best_err = err;
            best = p;
        }
    }
    return best;
}

/// SAMPCTRL.SAMPLEN for a wanted sampling time: 38.8.12's own
/// "Sampling time = (SAMPLEN + 1) / f_CLK_ADC", rounded UP and clamped
/// to the field's six bits.
constexpr uint8_t adc_samplen_for_ns(uint32_t clk_adc_hz, uint32_t ns) {
    if (clk_adc_hz == 0u) {
        return 63;
    }
    const uint64_t cycles =
        (static_cast<uint64_t>(ns) * clk_adc_hz + 999'999'999ULL) / 1'000'000'000ULL;
    if (cycles <= 1u) {
        return 0;
    }
    const uint64_t len = cycles - 1u;
    return len > 63u ? 63u : static_cast<uint8_t>(len);
}

/// The minimum sampling time table 45-22 states for the INTREF channel.
inline constexpr uint32_t adc_intref_sampling_ns = 10'000;

// =============================================================================
// The configuration
// =============================================================================

/// EVCTRL (38.8.4), enable-protected, both directions in one struct.
struct AdcEventControl {
    /// EVCTRL.STARTEI: any incoming event starts a conversion.
    bool start_in = false;
    /// EVCTRL.FLUSHEI: any incoming event flushes the pipeline and
    /// restarts. 38.6.6: if both arrive at once, FLUSH wins.
    bool flush_in = false;
    bool invert_start = false;   ///< STARTINV
    bool invert_flush = false;   ///< FLUSHINV
    bool result_out = false;     ///< RESRDYEO
    bool window_out = false;     ///< WINMONEO
};

/// Inverting an input nobody listens to is a configuration with no
/// meaning, and the same refusal samc/ac.hpp makes.
constexpr bool adc_event_control_valid(const AdcEventControl& e) {
    return (!e.invert_start || e.start_in) && (!e.invert_flush || e.flush_in);
}

/**
 * The whole configuration of one converter.
 *
 * The window thresholds and the two correction values live here because
 * they belong to the same register block and the same synchronization
 * regime; every one of them also has a runtime verb, because a window is
 * a thing an application moves while running.
 */
struct AdcConfig {
    // -- REFCTRL (enable-protected) --
    Ref reference = Ref::vddana;
    /// REFCTRL.REFCOMP: the reference buffer's offset is sensed during
    /// sampling and cancelled during conversion, which reduces gain
    /// error at no latency (38.6.2.15). ERRATUM 1.4.6 makes it expensive
    /// on any reference but VDDANA: the first five conversions are out
    /// of specification, and `init()` spends them.
    bool reference_compensation = false;

    // -- CTRLB (enable-protected) --
    AdcPresc prescaler = AdcPresc::div32;

    // -- CTRLC (write-synchronized, double-buffered) --
    AdcRes resolution = AdcRes::bits12;
    bool differential = false;
    bool left_adjust = false;
    bool free_running = false;
    /// CTRLC.CORREN: subtract OFFSETCORR and multiply by GAINCORR before
    /// RESULT. Costs 13 CLK_ADC cycles per conversion in single mode,
    /// and only on the first one in free-running mode (38.6.2.14).
    bool correction = false;
    /// CTRLC.R2R: rail-to-rail common mode. 38.6.3.2 requires a
    /// four-cycle sampling period, "achieved by enabling offset
    /// compensation", so this asks for `offset_compensation` too and is
    /// refused without it.
    bool rail_to_rail = false;
    AdcWindow window = AdcWindow::none;
    /// CTRLC.DUALSEL - the HOST's knob (ADC0 here). Refused on the
    /// client.
    AdcDual dual = AdcDual::both;

    // -- AVGCTRL (write-synchronized, double-buffered) --
    AdcAverage average = AdcAverage::samples1;
    /// AVGCTRL.ADJRES, the extra right shift. `adc_adjres_for_average()`
    /// and `adc_adjres_for_oversampling()` fill it from the chapter's
    /// two tables.
    uint8_t adjust = 0;

    // -- SAMPCTRL (write-synchronized, double-buffered) --
    /// SAMPCTRL.SAMPLEN: sampling lasts SAMPLEN + 1 CLK_ADC cycles.
    uint8_t sample_length = 0;
    /// SAMPCTRL.OFFCOMP: comparator offset compensation, which FIXES the
    /// sampling period at four cycles - so 38.8.12 forbids it together
    /// with a non-zero SAMPLEN, and so does `adc_config_valid()`.
    bool offset_compensation = false;

    // -- the window thresholds and the digital corrections --
    uint16_t window_low = 0;      ///< WINLT
    uint16_t window_high = 0;     ///< WINUT
    /// GAINCORR, 12 bits: a 1-bit integer and an 11-bit fraction, so
    /// 0x800 is exactly 1.0 and the legal range is [0.5, 2).
    uint16_t gain_correction = 0x800;
    /// OFFSETCORR, 12 bits, two's complement.
    int16_t offset_correction = 0;

    // -- CTRLA (not synchronized except ENABLE/SWRST) --
    bool run_standby = false;
    /// CTRLA.ONDEMAND: the analog block powers down between conversions
    /// and pays its start-up time again on the next request.
    bool on_demand = false;
    /// CTRLA.SLAVEEN - the CLIENT's knob (ADC1 here). Refused on the
    /// host.
    bool client_enable = false;

    // -- DBGCTRL (survives a software reset) --
    bool debug_run = false;

    // -- EVCTRL (enable-protected) --
    AdcEventControl events{};

    /// SEQCTRL: a bit per positive input to include in an automatic
    /// sequence (38.6.2.12). Zero disables the sequencer, and then the
    /// conversion uses MUXPOS.
    uint32_t sequence = 0;
};

/**
 * What the compile-time form static_asserts and the runtime form returns
 * false for. Everything here is a rule of the chapter, and `instance`
 * matters because the host/client pair is not symmetric.
 */
constexpr bool adc_config_valid(uint8_t instance, const AdcConfig& c) {
    if (!ref_valid(c.reference)) {
        return false;
    }
    if (c.adjust > 7u || c.sample_length > 63u) {
        return false;
    }
    if (c.gain_correction > 0x0FFFu) {
        return false;
    }
    if (c.offset_correction > 2047 || c.offset_correction < -2048) {
        return false;
    }
    // 38.8.12: OFFCOMP fixes the sampling period, so SAMPLEN must be 0.
    if (c.offset_compensation && c.sample_length != 0u) {
        return false;
    }
    // 38.6.3.2: rail-to-rail needs that four-cycle period.
    if (c.rail_to_rail && !c.offset_compensation) {
        return false;
    }
    // THE RESSEL/AVGCTRL TRAP (38.6.2.9, 38.6.2.10, 38.8.11): more than
    // one sample per result requires the 16BIT code.
    if (c.average != AdcAverage::samples1 && c.resolution != AdcRes::bits16) {
        return false;
    }
    // ...and the 16BIT code with a single sample is a result nothing
    // widened: harmless in the silicon, but it means the caller believes
    // something this driver's own arithmetic would then contradict.
    if (c.average == AdcAverage::samples1 && c.resolution == AdcRes::bits16 &&
        c.adjust != 0u) {
        return false;
    }
    // The pair, from the device header's own roles.
    if (c.client_enable && adc_pair_role(instance) != 2u) {
        return false;
    }
    if (c.dual != AdcDual::both && adc_pair_role(instance) != 1u) {
        return false;
    }
    return adc_event_control_valid(c.events);
}

/// The full scale of a RESULT: the sample's, widened by the accumulation
/// and narrowed by every right shift (tables 38-1 and 38-2). This is the
/// `steps` util/analog.hpp's `adc_mv()` wants.
constexpr uint32_t adc_result_steps(const AdcConfig& c) {
    const uint32_t base = adc_sample_steps(c.resolution);
    const uint32_t sum = base * adc_average_count(c.average);
    const uint8_t shift =
        static_cast<uint8_t>(adc_auto_shift(c.average) + c.adjust);
    return shift >= 32u ? 1u : (sum >> shift) == 0u ? 1u : (sum >> shift);
}

/**
 * CLK_ADC cycles for ONE SAMPLE, from table 45-22's own four rows.
 *
 * With OFFCOMP the sampling period is fixed and the totals are constants
 * (16/15/13 single-ended, 16/14/12 differential at 12/10/8 bits);
 * without it they are SAMPLEN plus a per-mode constant. `bits16` is a
 * 12-bit conversion, which `adc_sample_bits()` already says.
 */
constexpr uint32_t adc_sample_cycles(const AdcConfig& c) {
    const uint8_t bits = adc_sample_bits(c.resolution);
    if (c.offset_compensation) {
        if (c.differential) {
            return bits == 12u ? 16u : bits == 10u ? 14u : 12u;
        }
        return bits == 12u ? 16u : bits == 10u ? 15u : 13u;
    }
    const uint32_t base = c.differential
                              ? (bits == 12u ? 13u : bits == 10u ? 11u : 9u)
                              : (bits == 12u ? 13u : bits == 10u ? 12u : 10u);
    return base + c.sample_length;
}

/**
 * CLK_ADC cycles for one RESULT - every accumulated sample, plus the
 * digital correction's 13 cycles WHERE THEY ARE ACTUALLY CHARGED.
 *
 * 38.6.2.14 is precise and easy to read past: "The correction will
 * introduce a latency of 13 CLK_ADC clock cycles. IN FREE RUNNING MODE
 * THIS LATENCY IS INTRODUCED ON THE FIRST CONVERSION ONLY, since its
 * duration is always less than the propagation delay. In single
 * conversion mode this latency is introduced for each conversion." So a
 * free-running stream pays it once and not per result - measured on the
 * bench, where a corrected free-running conversion came out at exactly
 * the uncorrected 13 cycles and a corrected single one at 13 more
 * (docs/samc/adc.md).
 */
constexpr uint32_t adc_conversion_cycles(const AdcConfig& c) {
    const uint32_t per = adc_sample_cycles(c);
    const uint32_t n = adc_average_count(c.average);
    return per * n + ((c.correction && !c.free_running) ? 13u : 0u);
}

/// Results per second at a given GCLK_ADC rate.
constexpr uint32_t adc_result_hz(uint32_t gclk_hz, const AdcConfig& c) {
    const uint32_t cycles = adc_conversion_cycles(c);
    return cycles == 0u ? 0u : adc_clock_hz(gclk_hz, c.prescaler) / cycles;
}

// =============================================================================
// Inputs as types
// =============================================================================

/**
 * A pad as an analog input. The AIN CODE IS NOT IN THE TYPE, because on
 * this family it is not a property of the pad: PA08 is ADC0/AIN8 and
 * ADC1/AIN10 at the same time. So the tag names the pad and the
 * CONVERTER resolves it - `Adc<n>::input_code(AnalogIn<P>{})` - which is
 * also exactly the shape util/analog_sampler.hpp's `SamplerInput`
 * concept asks for.
 */
template <typename P>
struct AnalogIn {
    static constexpr char port = P::port_letter;
    static constexpr uint8_t pin = P::pin_number;
    using PinType = P;
};

// =============================================================================
// The converter
// =============================================================================

template <uint8_t n>
class Adc {
    static_assert(n < adc_count(), "this device has no such ADC instance");

public:
    Adc() = delete;

    static constexpr uint8_t instance = n;
    /// `ADCn_EXTCHANNEL_MSB` + 1: the CODE SPACE of the positive mux's
    /// pad inputs. Which of them this package bonds is `ain_exists()`.
    static constexpr uint8_t external_channels = adc_external_channels(n);
    static constexpr uint8_t gclk_id = adc_gclk_id(n);
    /// 1 = the host of the pair (owns DUALSEL), 2 = the client (owns
    /// SLAVEEN).
    static constexpr uint8_t pair_role = adc_pair_role(n);
    static constexpr bool is_host = pair_role == 1u;
    static constexpr bool is_client = pair_role == 2u;
    static constexpr bool needs_calibration = adc_loads_calibration(n);

    static constexpr IRQn_Type irq() { return n == 0 ? ADC0_IRQn : ADC1_IRQn; }

    // ---- the vocabularies this peripheral publishes -------------------------
    //
    // evsys.hpp owns the FABRIC and dmac.hpp owns the CHANNELS; the codes
    // of their tables that belong to the ADC live here, probed from the
    // device header in samc/device_tables.hpp.

    /// Generator: a conversion result is available.
    static constexpr uint8_t resrdy_generator = adc_resrdy_generator(n);
    /// Generator: the window monitor's condition matched.
    static constexpr uint8_t winmon_generator = adc_winmon_generator(n);
    /// User: start a conversion. TABLE 29-3 MARKS IT ASYNCHRONOUS PATH
    /// ONLY, and erratum 1.4.4 makes that a hard requirement rather than
    /// a preference - `start_on()` enforces it.
    static constexpr uint8_t start_event_user = adc_start_user(n);
    /// User: flush the pipeline. The device header spells it SYNC after
    /// table 29-3's row name; EVCTRL and 38.6.6 call it FLUSH.
    static constexpr uint8_t flush_event_user = adc_flush_user(n);
    /// DMAC trigger: the one DMA request this peripheral has (38.6.4).
    static constexpr uint8_t dma_trigger_resrdy = adc_dma_resrdy_id(n);

    /// INTFLAG / INTENSET bits, named.
    static constexpr uint8_t flag_resrdy = ADC_INTFLAG_RESRDY_Msk;
    static constexpr uint8_t flag_overrun = ADC_INTFLAG_OVERRUN_Msk;
    static constexpr uint8_t flag_winmon = ADC_INTFLAG_WINMON_Msk;

    static adc_registers_t& regs() { return n == 0 ? *ADC0_REGS : *ADC1_REGS; }

    // ---- per-package pad legality ------------------------------------------

    /// Which AIN code a pad is FOR THIS CONVERTER on THIS package, or -1.
    static constexpr int ain_of(char port, uint8_t pin) {
        return adc_ain_code(n, port, pin);
    }
    /// Whether an AIN code reaches any pad on this package. The E bonds
    /// no PORT B pad to either converter, which leaves ADC1 there with
    /// AIN10 and AIN11 and nothing else.
    static constexpr bool ain_exists(uint8_t ain) {
        return adc_ain_exists(n, ain);
    }

    static constexpr bool config_valid(const AdcConfig& c) {
        return adc_config_valid(n, c);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) {
        Mclk::apb_c(n == 0 ? MCLK_APBCMASK_ADC0_Msk : MCLK_APBCMASK_ADC1_Msk, on);
    }

    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    /// Bounded spin on SYNCBUSY. It is a 16-bit register here, so this
    /// cannot reuse clock.hpp's 32-bit `clock_wait`.
    static bool sync_wait(uint16_t mask, uint32_t spins = 0xFFFFu) {
        while (spins-- != 0u) {
            if ((regs().ADC_SYNCBUSY & mask) == 0u) {
                return true;
            }
        }
        return false;
    }
    static uint16_t sync_busy() { return regs().ADC_SYNCBUSY; }

    /// CTRLA.SWRST: every register except DBGCTRL back to reset, and the
    /// converter disabled.
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().ADC_CTRLA = ADC_CTRLA_SWRST_Msk;
        return sync_wait(ADC_SYNCBUSY_SWRST_Msk, spins);
    }

    /**
     * CTRLA.ENABLE, preserving the other CTRLA bits.
     *
     * ERRATUM 1.4.10 (all revisions): enabling ADC1 while ADC0 is
     * DISABLED can leave ADC0.SYNCBUSY.ENABLE stuck at one. This waits
     * on its OWN instance's bit, which the erratum does not touch, so
     * nothing here hangs - but a caller that reads ADC0's SYNCBUSY after
     * bringing ADC1 up alone must disregard it, or enable ADC0 first.
     */
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint8_t v =
            static_cast<uint8_t>(regs().ADC_CTRLA & ~ADC_CTRLA_ENABLE_Msk);
        regs().ADC_CTRLA =
            on ? static_cast<uint8_t>(v | ADC_CTRLA_ENABLE_Msk) : v;
        return sync_wait(ADC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() { return (regs().ADC_CTRLA & ADC_CTRLA_ENABLE_Msk) != 0u; }

    // ---- configuration ------------------------------------------------------

    /**
     * The whole chapter in one call: bus clock, generic clock, reset,
     * the factory calibration, every register in the order the four
     * disciplines require, enable, and erratum 1.4.6's five discarded
     * conversions where they are owed.
     *
     * `gclk_hz` is what the generator runs at, and it is here for ONE
     * reason: to refuse a prescaler that puts CLK_ADC outside table
     * 45-22's 160 kHz .. 16 MHz. Pass 0 to skip that check when the rate
     * is genuinely unknown.
     */
    static bool init(uint8_t generator, const AdcConfig& cfg,
                     uint32_t gclk_hz = 0, uint32_t spins = 0xFFFFu) {
        if (!config_valid(cfg)) {
            return false;
        }
        if (gclk_hz != 0u && !adc_clock_in_range(gclk_hz, cfg.prescaler)) {
            return false;
        }
        Nvic::disable(irq());
        bus_clock(true);
        if (!clock(generator, spins)) {
            return false;
        }
        if (!reset(spins)) {
            return false;
        }
        // Enable-protected, and the converter is disabled out of reset.
        load_calibration();
        regs().ADC_CTRLB = static_cast<uint8_t>(
            ADC_CTRLB_PRESCALER(static_cast<uint8_t>(cfg.prescaler)));
        regs().ADC_REFCTRL = reference_word(cfg);
        regs().ADC_EVCTRL = event_word(cfg.events);
        regs().ADC_DBGCTRL = cfg.debug_run ? ADC_DBGCTRL_DBGRUN_Msk : 0u;
        regs().ADC_SEQCTRL = cfg.sequence;

        // Write-synchronized and double-buffered. Nothing is converting
        // yet, so each wait is the bus's and not a conversion's.
        regs().ADC_CTRLC = control_c_word(cfg);
        if (!sync_wait(ADC_SYNCBUSY_CTRLC_Msk, spins)) {
            return false;
        }
        regs().ADC_AVGCTRL = average_word(cfg);
        if (!sync_wait(ADC_SYNCBUSY_AVGCTRL_Msk, spins)) {
            return false;
        }
        regs().ADC_SAMPCTRL = sample_word(cfg);
        if (!sync_wait(ADC_SYNCBUSY_SAMPCTRL_Msk, spins)) {
            return false;
        }
        regs().ADC_WINLT = cfg.window_low;
        regs().ADC_WINUT = cfg.window_high;
        if (!sync_wait(ADC_SYNCBUSY_WINLT_Msk | ADC_SYNCBUSY_WINUT_Msk, spins)) {
            return false;
        }
        regs().ADC_GAINCORR = static_cast<uint16_t>(cfg.gain_correction & 0x0FFFu);
        regs().ADC_OFFSETCORR =
            static_cast<uint16_t>(static_cast<uint16_t>(cfg.offset_correction) & 0x0FFFu);
        if (!sync_wait(ADC_SYNCBUSY_GAINCORR_Msk | ADC_SYNCBUSY_OFFSETCORR_Msk,
                       spins)) {
            return false;
        }
        // A clean slate: mux at ground, no flag standing, no interrupt.
        regs().ADC_INPUTCTRL =
            static_cast<uint16_t>(ADC_INPUTCTRL_MUXPOS(0u) |
                                  ADC_INPUTCTRL_MUXNEG(ADC_INPUTCTRL_MUXNEG_GND_Val));
        if (!sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk, spins)) {
            return false;
        }
        regs().ADC_INTENCLR = flag_resrdy | flag_overrun | flag_winmon;
        regs().ADC_INTFLAG = flag_resrdy | flag_overrun | flag_winmon;

        // CTRLA last: SLAVEEN, RUNSTDBY and ONDEMAND are not
        // synchronized, ENABLE is.
        const uint8_t ctrla = static_cast<uint8_t>(
            (cfg.on_demand ? ADC_CTRLA_ONDEMAND_Msk : 0u) |
            (cfg.run_standby ? ADC_CTRLA_RUNSTDBY_Msk : 0u) |
            (cfg.client_enable ? ADC_CTRLA_SLAVEEN_Msk : 0u));
        regs().ADC_CTRLA = ctrla;
        if (!enable(true, spins)) {
            return false;
        }
        cfg_ = cfg;
        warm_up_ = warm_up_needed(cfg);
        discard(warm_up_);
        return true;
    }

    /// The compile-time twin: the whole configuration as a constant, so
    /// every rule of `adc_config_valid()` is a compile error instead of
    /// a false return.
    template <AdcConfig cfg>
    static bool init(uint8_t generator, uint32_t gclk_hz = 0,
                     uint32_t spins = 0xFFFFu) {
        static_assert(adc_config_valid(n, cfg),
                      "brio AdcConfig: see adc_config_valid() - the RESSEL/"
                      "AVGCTRL pairing, OFFCOMP against SAMPLEN, R2R without "
                      "offset compensation, a Reserved reference, or a "
                      "host/client knob on the wrong instance");
        return init(generator, cfg, gclk_hz, spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        bus_clock(false);
    }

    /// The configuration in force, as `init()` was given it.
    static const AdcConfig& config() { return cfg_; }

    // ---- the factory calibration -------------------------------------------

    /**
     * 38.5.10 and CALIB's own register description: copy BIASREFBUF and
     * BIASCOMP from the NVM software calibration row, and DO NOT CHANGE
     * THEM. samc/nvm.hpp typed the four fields (two per converter) when
     * it was written, with a comment promising this call.
     *
     * CALIB is ENABLE-PROTECTED, so this refuses while the converter is
     * enabled instead of storing into a dead register.
     */
    static bool load_calibration() {
        if (enabled()) {
            return false;
        }
        const NvmCalibration cal = NvmCalibration::read();
        const uint8_t refbuf = n == 0 ? cal.adc0_biasrefbuf() : cal.adc1_biasrefbuf();
        const uint8_t comp = n == 0 ? cal.adc0_biascomp() : cal.adc1_biascomp();
        regs().ADC_CALIB = static_cast<uint16_t>(ADC_CALIB_BIASREFBUF(refbuf) |
                                                 ADC_CALIB_BIASCOMP(comp));
        return true;
    }

    static uint16_t calibration() { return regs().ADC_CALIB; }
    static uint8_t bias_reference_buffer() {
        return static_cast<uint8_t>((calibration() & ADC_CALIB_BIASREFBUF_Msk) >>
                                    ADC_CALIB_BIASREFBUF_Pos);
    }
    static uint8_t bias_comparator() {
        return static_cast<uint8_t>((calibration() & ADC_CALIB_BIASCOMP_Msk) >>
                                    ADC_CALIB_BIASCOMP_Pos);
    }

    // ---- the result's scale -------------------------------------------------

    /// Full scale of ONE sample (4096 / 1024 / 256).
    static uint32_t sample_steps() { return adc_sample_steps(cfg_.resolution); }
    /// Full scale of a RESULT as read - accumulation and every right
    /// shift accounted. This is util/analog.hpp's `steps`.
    static uint32_t result_steps() { return adc_result_steps(cfg_); }
    static uint8_t result_shift() {
        return static_cast<uint8_t>(adc_auto_shift(cfg_.average) + cfg_.adjust);
    }
    static Ref reference() { return cfg_.reference; }

    /// A reading in MILLIVOLTS, through util/analog.hpp's arithmetic and
    /// this converter's own scale. `known_mv` is what `ref_mv()` needs
    /// where the driver cannot know it (VDDANA for the supply-derived
    /// references), `intref_level` what stands in SUPC.VREF.SEL.
    static uint16_t to_mv(uint32_t counts, uint16_t known_mv = 0,
                          VrefLevel intref_level = VrefLevel::v1_024) {
        return adc_mv(counts, result_steps(),
                      ref_mv(cfg_.reference, known_mv, intref_level));
    }

    // ---- input selection ----------------------------------------------------

    /// The MUXPOS code this converter reports for an input, at compile
    /// time - what `selected()` reads back while it is in effect, and
    /// what util/analog_sampler.hpp labels a result with.
    template <typename P>
    static constexpr uint8_t input_code(AnalogIn<P>) {
        static_assert(adc_ain_code(n, AnalogIn<P>::port, AnalogIn<P>::pin) >= 0,
                      "this package does not bond that pad to THIS converter "
                      "(the two ADCs have different maps over overlapping "
                      "pads, and the E bonds no PORT B pad to either)");
        return static_cast<uint8_t>(
            adc_ain_code(n, AnalogIn<P>::port, AnalogIn<P>::pin));
    }
    static constexpr uint8_t input_code(AdcInput in) {
        return static_cast<uint8_t>(in);
    }

    /**
     * Select a positive input. VOID AND NO WAIT, deliberately: INPUTCTRL
     * is double-buffered, so a change made mid-stream takes effect at
     * the next conversion and its SYNCBUSY bit stands until then -
     * waiting here would mean waiting out a conversion. `select_sync()`
     * is the verb for a caller who needs it in force before returning.
     *
     * THE PAD IS NOT TOUCHED. An analog input is a direct connection to
     * the pad, so a pad left under PORT - even driven as an output - is
     * read as it stands, which is what makes a wireless bench test
     * possible (samc/ac.hpp established the same thing for the
     * comparators). 38.5.1's recommended configuration for a real analog
     * source is `claim_pad<P>()`.
     */
    template <typename P>
    static void select(AnalogIn<P> in) { select_code(input_code(in)); }
    static void select(AdcInput in) { select_code(static_cast<uint8_t>(in)); }

    /// The negative input of a differential pair - AIN0..AIN5 or ground,
    /// and nothing else exists (38.8.9).
    static bool select_negative(AdcNegative neg) {
        if (!adc_negative_valid(neg)) {
            return false;
        }
        const uint16_t v = static_cast<uint16_t>(
            (regs().ADC_INPUTCTRL & ADC_INPUTCTRL_MUXPOS_Msk) |
            ADC_INPUTCTRL_MUXNEG(static_cast<uint16_t>(neg)));
        regs().ADC_INPUTCTRL = v;
        return true;
    }

    template <typename P>
    static bool select_sync(AnalogIn<P> in, uint32_t spins = 0xFFFFu) {
        select(in);
        return sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk, spins);
    }
    static bool select_sync(AdcInput in, uint32_t spins = 0xFFFFu) {
        select(in);
        return sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk, spins);
    }

    /// MUXPOS as it stands. Read it with the value in the RESRDY ISR
    /// body's glue to label a result (util/analog_sampler.hpp).
    static uint8_t selected() {
        return static_cast<uint8_t>((regs().ADC_INPUTCTRL & ADC_INPUTCTRL_MUXPOS_Msk) >>
                                    ADC_INPUTCTRL_MUXPOS_Pos);
    }
    static uint8_t selected_negative() {
        return static_cast<uint8_t>((regs().ADC_INPUTCTRL & ADC_INPUTCTRL_MUXNEG_Msk) >>
                                    ADC_INPUTCTRL_MUXNEG_Pos);
    }

    /// 38.5.1's recommended pad configuration for a real analog source:
    /// peripheral function B with the digital input buffer off. NOT
    /// needed to read the pad (the connection is direct); needed to stop
    /// the digital receiver toggling on a mid-rail voltage.
    template <typename P>
    static void claim_pad() {
        static_assert(adc_ain_code(n, P::port_letter, P::pin_number) >= 0,
                      "this package does not bond that pad to THIS converter");
        P::function(PinFunction::b, PinConfig{});
    }
    template <typename P>
    static void release_pad() { P::release(); }

    // ---- the automatic sequence (38.6.2.12) --------------------------------

    /// One bit per positive input; the sequence walks them from the
    /// lowest. Zero disables it and the conversion uses MUXPOS. NOT
    /// enable-protected and NOT synchronized.
    static void sequence(uint32_t mask) { regs().ADC_SEQCTRL = mask; }
    static uint32_t sequence() { return regs().ADC_SEQCTRL; }

    /// SEQSTATUS.SEQBUSY: a sequence is in progress.
    static bool sequence_busy() {
        return (regs().ADC_SEQSTATUS & ADC_SEQSTATUS_SEQBUSY_Msk) != 0u;
    }
    /**
     * SEQSTATUS.SEQSTATE: the input the LAST completed conversion of the
     * sequence came from.
     *
     * ERRATUM 1.4.9 (all revisions): it is NOT updated for the first
     * conversion of a sequence that exited standby - the RESULT is
     * right, the label is not, and there is no workaround.
     */
    static uint8_t sequence_state() {
        return static_cast<uint8_t>(regs().ADC_SEQSTATUS & ADC_SEQSTATUS_SEQSTATE_Msk);
    }

    // ---- conversions --------------------------------------------------------

    /**
     * SWTRIG.START. NO WAIT, and that is erratum 1.4.5 as code:
     * SYNCBUSY.SWTRIG becomes stuck at one after a wake from standby,
     * and the errata's own instruction is to ignore the bit and to start
     * the next conversion by writing START again. RESRDY is the only
     * progress this file believes.
     */
    static void start() { regs().ADC_SWTRIG = ADC_SWTRIG_START_Msk; }

    /// SWTRIG.FLUSH: abandon everything in the pipeline and restart the
    /// ADC clock; a pending conversion begins again (38.8.17). Also
    /// without a wait, and for the same reason.
    static void flush() { regs().ADC_SWTRIG = ADC_SWTRIG_FLUSH_Msk; }

    static bool ready() { return (regs().ADC_INTFLAG & flag_resrdy) != 0u; }
    static bool overrun() { return (regs().ADC_INTFLAG & flag_overrun) != 0u; }

    /**
     * RESULT.
     *
     * READING IT CLEARS BOTH RESRDY AND WINMON (38.8.7), so the window
     * verdict is captured first and `window_hit()` reports it for the
     * last value read - the same shape avrdx/adc.hpp has, for the same
     * reason.
     */
    static uint16_t result() {
        last_hit_ = (regs().ADC_INTFLAG & flag_winmon) != 0u;
        return regs().ADC_RESULT;
    }
    static int16_t result_signed() { return static_cast<int16_t>(result()); }

    /// Start, wait for RESRDY, read. Bounded: a converter that never
    /// answers returns the previous reading and says so.
    static bool read(uint16_t& out, uint32_t spins = 0xFFFFFu) {
        clear_flags(flag_resrdy | flag_overrun);
        start();
        while (spins-- != 0u) {
            if (ready()) {
                out = result();
                return true;
            }
        }
        return false;
    }
    /// The convenience form, for a converter known to be running.
    static uint16_t read(uint32_t spins = 0xFFFFFu) {
        uint16_t v = 0;
        (void)read(v, spins);
        return v;
    }

    /// Spend `count` conversions and throw them away - erratum 1.4.6's
    /// five, or a caller's own warm-up.
    static void discard(uint8_t count, uint32_t spins = 0xFFFFFu) {
        for (uint8_t i = 0; i < count; ++i) {
            uint16_t v = 0;
            (void)read(v, spins);
        }
    }
    /// How many conversions `init()` decided to discard.
    static uint8_t warm_up_conversions() { return warm_up_; }

    // ---- the window monitor -------------------------------------------------

    /**
     * CTRLC.WINMODE with its two thresholds, under a running converter.
     * All three registers are write-synchronized AND double-buffered, so
     * a change made mid-stream takes effect with the next result; the
     * wait here is bounded and reported.
     *
     * 38.6.2.13: in DIFFERENTIAL mode the thresholds are read as SIGNED,
     * and only the bits the resolution carries are significant - in
     * 8-bit mode the eighth bit is the sign even though the ninth is
     * zero. `window_signed()` is the differential spelling.
     */
    static bool window(AdcWindow mode, uint16_t low, uint16_t high,
                       uint32_t spins = 0xFFFFu) {
        regs().ADC_WINLT = low;
        regs().ADC_WINUT = high;
        if (!sync_wait(ADC_SYNCBUSY_WINLT_Msk | ADC_SYNCBUSY_WINUT_Msk, spins)) {
            return false;
        }
        const uint16_t v = static_cast<uint16_t>(
            (regs().ADC_CTRLC & ~static_cast<uint16_t>(ADC_CTRLC_WINMODE_Msk)) |
            ADC_CTRLC_WINMODE(static_cast<uint16_t>(mode)));
        regs().ADC_CTRLC = v;
        return sync_wait(ADC_SYNCBUSY_CTRLC_Msk, spins);
    }
    static bool window_signed(AdcWindow mode, int16_t low, int16_t high,
                              uint32_t spins = 0xFFFFu) {
        return window(mode, static_cast<uint16_t>(low), static_cast<uint16_t>(high),
                      spins);
    }
    static bool window_off(uint32_t spins = 0xFFFFu) {
        return window(AdcWindow::none, regs().ADC_WINLT, regs().ADC_WINUT, spins);
    }
    static AdcWindow window_mode() {
        return static_cast<AdcWindow>((regs().ADC_CTRLC & ADC_CTRLC_WINMODE_Msk) >>
                                      ADC_CTRLC_WINMODE_Pos);
    }
    /// Did the LAST value read by `result()`/`read()` match the window?
    /// (The hardware flag is cleared by the RESULT read itself.)
    static bool window_hit() { return last_hit_; }
    /// The live flag, before any RESULT read.
    static bool window_flag() { return (regs().ADC_INTFLAG & flag_winmon) != 0u; }

    // ---- the digital corrections (38.6.2.14) --------------------------------

    /// GAINCORR: a 1-bit integer plus an 11-bit fraction, so 0x800 is
    /// 1.0 and the register's own range is [0.5, 2). Write-synchronized
    /// and double-buffered.
    static bool gain_correction(uint16_t value, uint32_t spins = 0xFFFFu) {
        if (value > 0x0FFFu) {
            return false;
        }
        regs().ADC_GAINCORR = value;
        return sync_wait(ADC_SYNCBUSY_GAINCORR_Msk, spins);
    }
    static uint16_t gain_correction() {
        return static_cast<uint16_t>(regs().ADC_GAINCORR & 0x0FFFu);
    }
    /// OFFSETCORR: 12 bits, two's complement, SUBTRACTED from the
    /// conversion before the gain multiply.
    static bool offset_correction(int16_t value, uint32_t spins = 0xFFFFu) {
        if (value > 2047 || value < -2048) {
            return false;
        }
        regs().ADC_OFFSETCORR =
            static_cast<uint16_t>(static_cast<uint16_t>(value) & 0x0FFFu);
        return sync_wait(ADC_SYNCBUSY_OFFSETCORR_Msk, spins);
    }
    static int16_t offset_correction() {
        const uint16_t raw = static_cast<uint16_t>(regs().ADC_OFFSETCORR & 0x0FFFu);
        return static_cast<int16_t>(raw & 0x800u ? raw | 0xF000u : raw);
    }
    /// CTRLC.CORREN under a running converter.
    static bool correction_enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint16_t v = regs().ADC_CTRLC & ~static_cast<uint16_t>(ADC_CTRLC_CORREN_Msk);
        regs().ADC_CTRLC =
            on ? static_cast<uint16_t>(v | ADC_CTRLC_CORREN_Msk) : v;
        return sync_wait(ADC_SYNCBUSY_CTRLC_Msk, spins);
    }

    // ---- free running -------------------------------------------------------

    static bool free_running(bool on, uint32_t spins = 0xFFFFu) {
        const uint16_t v = regs().ADC_CTRLC & ~static_cast<uint16_t>(ADC_CTRLC_FREERUN_Msk);
        regs().ADC_CTRLC =
            on ? static_cast<uint16_t>(v | ADC_CTRLC_FREERUN_Msk) : v;
        return sync_wait(ADC_SYNCBUSY_CTRLC_Msk, spins);
    }
    static bool free_running() {
        return (regs().ADC_CTRLC & ADC_CTRLC_FREERUN_Msk) != 0u;
    }

    // ---- interrupts ---------------------------------------------------------

    static void arm(uint8_t mask) { regs().ADC_INTENSET = mask; }
    static void disarm(uint8_t mask) { regs().ADC_INTENCLR = mask; }
    static uint8_t armed() { return regs().ADC_INTENSET; }
    static uint8_t flags() { return regs().ADC_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().ADC_INTFLAG = mask; }

    /**
     * The ISR body - ONE VECTOR for all three sources, so the app binds
     * ADCn_Handler once and dispatches on the returned mask.
     *
     * OVERRUN is cleared here; RESRDY and WINMON are NOT, because
     * reading RESULT is what clears them and the value is the point. A
     * handler that returns without reading RESULT will be called again.
     */
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t pending =
            static_cast<uint8_t>(regs().ADC_INTFLAG & regs().ADC_INTENSET);
        if ((pending & flag_overrun) != 0u) {
            regs().ADC_INTFLAG = flag_overrun;
        }
        return pending;
    }

    /// The RESRDY half of the ISR body, for the glue that only wants the
    /// value: the result, with the window verdict captured first.
    [[gnu::always_inline]] static uint16_t resrdy() { return result(); }

    // ---- events -------------------------------------------------------------

    /// EVCTRL is ENABLE-PROTECTED (38.8.4), so this refuses while the
    /// converter is enabled rather than storing into a dead register.
    static bool event_config(const AdcEventControl& e) {
        if (enabled() || !adc_event_control_valid(e)) {
            return false;
        }
        regs().ADC_EVCTRL = event_word(e);
        return true;
    }
    static AdcEventControl event_config() {
        const uint8_t v = regs().ADC_EVCTRL;
        return AdcEventControl{
            .start_in = (v & ADC_EVCTRL_STARTEI_Msk) != 0u,
            .flush_in = (v & ADC_EVCTRL_FLUSHEI_Msk) != 0u,
            .invert_start = (v & ADC_EVCTRL_STARTINV_Msk) != 0u,
            .invert_flush = (v & ADC_EVCTRL_FLUSHINV_Msk) != 0u,
            .result_out = (v & ADC_EVCTRL_RESRDYEO_Msk) != 0u,
            .window_out = (v & ADC_EVCTRL_WINMONEO_Msk) != 0u,
        };
    }

    /**
     * Route an EVSYS channel to this converter's START user and turn
     * EVCTRL.STARTEI on - the two halves that must both happen, in one
     * verb so neither can be forgotten.
     *
     * IT REFUSES A CHANNEL THAT IS NOT ASYNCHRONOUS, which is erratum
     * 1.4.4 as code (a synchronized event arriving during a conversion
     * is never acknowledged and STALLS THE CHANNEL) and table 29-3's own
     * restriction besides. EVCTRL being enable-protected, the converter
     * must be DISABLED when this is called.
     */
    static bool start_on(uint8_t channel, const EventChannelConfig& cfg,
                         bool invert = false) {
        if (cfg.path != EventPath::asynchronous || enabled()) {
            return false;
        }
        AdcEventControl e = event_config();
        e.start_in = true;
        e.invert_start = invert;
        if (!event_config(e)) {
            return false;
        }
        return Evsys::connect(start_event_user, channel, cfg);
    }

    /// The same for the FLUSH user, under the same restriction.
    static bool flush_on(uint8_t channel, const EventChannelConfig& cfg,
                         bool invert = false) {
        if (cfg.path != EventPath::asynchronous || enabled()) {
            return false;
        }
        AdcEventControl e = event_config();
        e.flush_in = true;
        e.invert_flush = invert;
        if (!event_config(e)) {
            return false;
        }
        return Evsys::connect(flush_event_user, channel, cfg);
    }

    /// Stop listening: both users disconnected and both input enables
    /// cleared. Needs the converter disabled, EVCTRL being
    /// enable-protected.
    static bool stop_events() {
        if (enabled()) {
            return false;
        }
        Evsys::disconnect(start_event_user);
        Evsys::disconnect(flush_event_user);
        AdcEventControl e = event_config();
        e.start_in = false;
        e.flush_in = false;
        e.invert_start = false;
        e.invert_flush = false;
        return event_config(e);
    }

private:
    static void select_code(uint8_t code) {
        const uint16_t v = static_cast<uint16_t>(
            (regs().ADC_INPUTCTRL & ADC_INPUTCTRL_MUXNEG_Msk) |
            ADC_INPUTCTRL_MUXPOS(code));
        regs().ADC_INPUTCTRL = v;
    }

    static constexpr uint8_t reference_word(const AdcConfig& c) {
        return static_cast<uint8_t>(
            ADC_REFCTRL_REFSEL(static_cast<uint8_t>(c.reference)) |
            (c.reference_compensation ? ADC_REFCTRL_REFCOMP_Msk : 0u));
    }

    static constexpr uint8_t event_word(const AdcEventControl& e) {
        return static_cast<uint8_t>((e.flush_in ? ADC_EVCTRL_FLUSHEI_Msk : 0u) |
                                    (e.start_in ? ADC_EVCTRL_STARTEI_Msk : 0u) |
                                    (e.invert_flush ? ADC_EVCTRL_FLUSHINV_Msk : 0u) |
                                    (e.invert_start ? ADC_EVCTRL_STARTINV_Msk : 0u) |
                                    (e.result_out ? ADC_EVCTRL_RESRDYEO_Msk : 0u) |
                                    (e.window_out ? ADC_EVCTRL_WINMONEO_Msk : 0u));
    }

    static constexpr uint16_t control_c_word(const AdcConfig& c) {
        return static_cast<uint16_t>(
            (c.differential ? ADC_CTRLC_DIFFMODE_Msk : 0u) |
            (c.left_adjust ? ADC_CTRLC_LEFTADJ_Msk : 0u) |
            (c.free_running ? ADC_CTRLC_FREERUN_Msk : 0u) |
            (c.correction ? ADC_CTRLC_CORREN_Msk : 0u) |
            ADC_CTRLC_RESSEL(static_cast<uint16_t>(c.resolution)) |
            (c.rail_to_rail ? ADC_CTRLC_R2R_Msk : 0u) |
            ADC_CTRLC_WINMODE(static_cast<uint16_t>(c.window)) |
            ADC_CTRLC_DUALSEL(static_cast<uint16_t>(c.dual)));
    }

    static constexpr uint8_t average_word(const AdcConfig& c) {
        return static_cast<uint8_t>(
            ADC_AVGCTRL_SAMPLENUM(static_cast<uint8_t>(c.average)) |
            ADC_AVGCTRL_ADJRES(c.adjust));
    }

    static constexpr uint8_t sample_word(const AdcConfig& c) {
        return static_cast<uint8_t>(
            ADC_SAMPCTRL_SAMPLEN(c.sample_length) |
            (c.offset_compensation ? ADC_SAMPCTRL_OFFCOMP_Msk : 0u));
    }

    /// ERRATUM 1.4.6: REFCOMP with any reference but VDDANA puts the
    /// first five conversions out of specification.
    static constexpr uint8_t warm_up_needed(const AdcConfig& c) {
        return (c.reference_compensation && c.reference != Ref::vddana) ? 5u : 0u;
    }

    static inline AdcConfig cfg_{};
    static inline bool last_hit_ = false;
    static inline uint8_t warm_up_ = 0;
};

} // namespace brio
