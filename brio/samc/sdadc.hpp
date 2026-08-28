/*
 * sdadc.hpp
 *
 * The SAM C21's 16-bit SIGMA-DELTA converter (DS60001479M ch. 39), this
 * family's third and strangest converter, and the one the Multislope
 * work will lean on. A MONOSTATE `Sdadc`, not `Sdadc<n>`: there is
 * exactly one instance on every C21 variant, so an index would be a
 * parameter with a single legal value (the `Rtc` and `Dac` precedent in
 * this stratum, against `Adc<n>`'s two real instances).
 *
 *   brio::Sdadc::init(generator, brio::SdadcConfig{
 *       .reference = brio::SdadcRef::vddana,
 *       .prescaler = 3,                       // 48 MHz / 8 = 6 MHz
 *       .osr = brio::SdadcOsr::osr256});
 *   (void)brio::Sdadc::select(0);             // the AINN0/AINP0 pair
 *   int16_t v = 0;
 *   if (brio::Sdadc::read(v)) { ... }         // SIGNED, -VREF .. +VREF
 *
 * ---------------------------------------------------------------------
 * WHAT THE SILICON DOES, and how little of it is like the SAR.
 *
 * A sigma-delta modulator running at CLK_SDADC_FS feeds a THIRD-ORDER
 * SINC DECIMATION FILTER (39.6.3), and the filter - not a successive
 * approximation - is what makes the result. (The chapter states the
 * FILTER's order and never the modulator's, so neither does this file.)
 * So:
 *
 *  - THE INPUT IS ALWAYS DIFFERENTIAL. MUXSEL names a PAIR of pads
 *    (AINN<k>, AINP<k>), never a single one, and there are three pairs.
 *    "Single-ended mode" in the electrical tables is the same thing with
 *    the negative pad grounded EXTERNALLY (table 45-26 note 3), which
 *    costs a bit of resolution and is not a register setting.
 *  - THE RESULT IS SIGNED, spanning -VREF .. +VREF. 39.1 and 39.8.19
 *    say signed; 39.6.1, 39.6.3.1 and 39.6.3.4 all say UNSIGNED, and
 *    39.6.3.4 even saturates to [0 : 2^16-1]. THE CHAPTER CONTRADICTS
 *    ITSELF FOUR TIMES; the bench settles it - a rail-to-rail
 *    differential reads 0x7FFFFF one way round and 0x800000 the other.
 *  - AND THE DATAPATH IS TWENTY-FOUR BITS WIDE, WHICH THE CHAPTER NEVER
 *    SAYS. 39.8.19 calls RESULT "a signed integer value with 24-bit
 *    size" and adds that the conversion result is "left-adjusted" in it,
 *    which reads like eight bits of padding. It is not padding: the
 *    register SATURATES at +/-2^23 (0x7FFFFF and 0x800000, not 0x7FFF00
 *    and 0x800000), the corrections of 39.6.3.4 act in RAW 24-BIT UNITS -
 *    an OFFSETCORR of 25600 moves the reading by EXACTLY 100 of the
 *    16-bit datum's counts, i.e. by 25600/256 - and the low bits carry
 *    real filter output. So this file has THREE result verbs: `result()` is the
 *    16-bit datum the chapter specifies (the top 16 bits, which is what
 *    `half_steps` scales), `result24()` is the whole signed 24-bit value,
 *    and `result_raw()` is the register. Anything about the CORRECTIONS
 *    is in 24-bit units.
 *  - THERE IS NO SAMPLE LENGTH AND NO RESOLUTION KNOB. The output rate
 *    is CLK_SDADC_FS / OSR, the specified resolution is always 16 bits,
 *    and OSR is the only thing trading rate against noise. 39.8.3: "The
 *    OSR must never be changed while the SDADC is running."
 *  - SKPCNT is a WARM-UP, not an average. A sigma-delta's filter has to
 *    fill before its output means anything, so a SINGLE conversion
 *    actually runs SKPCNT+1 decimation windows and returns the last -
 *    measured, each skipped window costing exactly one window's time.
 *    The reset value is 2, which is 39.6.2.3's "the first valid sample
 *    starts from the third sample onward" expressed as a register, and
 *    it is also erratum 1.18.3's own workaround ("Write CTRLB.SKPCNT to
 *    2 before running single conversions"). `sdadc_config_valid()`
 *    REFUSES a single-conversion configuration with fewer than two
 *    skipped windows, because the chapter, the reset value and the
 *    errata all say the same thing: that data is invalid.
 *
 * ---------------------------------------------------------------------
 * THE CLOCK, which has THREE stages and is easy to get wrong by a factor
 * of four:
 *
 *      CLK_SDADC     = GCLK_SDADC / (2 x (PRESCALER + 1))   (39.5.3)
 *      CLK_SDADC_FS  = CLK_SDADC / 4                        (39.5.3)
 *      output rate   = CLK_SDADC_FS / OSR                   (free running)
 *                    = CLK_SDADC_FS / (OSR x (SKPCNT + 1))  (single)
 *
 * PRESCALER IS EIGHT BITS AND LINEAR, and this is where the device
 * header lies: it declares eight enumerators DIV2, DIV4, DIV8 ... DIV256
 * for codes 0..7, which is the SAM D21's power-of-two prescaler. This
 * silicon's is a 7-bit-plus-one linear divider - 39.5.3 and 39.8.3 both
 * say "between GCLK_SDADC/2 (PRESCAL 0) and GCLK_SDADC/512 (PRESCAL
 * 255)", and figure 39-2 draws DIV2, DIV4, DIV6, DIV8, DIV10 ... DIV510,
 * DIV512. A power-of-two reading cannot reach 512 from an 8-bit field at
 * all. MEASURED, crystal-ruled: PRESCALER 3/4/7/23 give conversion
 * periods in the ratios 1000/1250/2001/6005 where 2 x (P + 1) predicts
 * 1000/1250/2000/6000 and the header's enumerators predict
 * 1000/2000/16000. `sdadc_prescaler_divisor()` follows the datasheet and
 * no enumerator of the header's is used anywhere in this file.
 *
 * Table 45-26 bounds CLK_SDADC at 1 MHz .. 6 MHz, so at a 48 MHz
 * generator PRESCALER must be 3..23. `init()` refuses a prescaler that
 * leaves the range when it is told the generator's rate.
 *
 * No `rebase()` and no `ClockUser`: this converter has its own generic
 * clock channel and a main-clock change does not move CLK_SDADC
 * (docs/samc/clock.md, the DynamicClock deferral).
 *
 * ---------------------------------------------------------------------
 * THE FOUR REGISTER DISCIPLINES, spelled per register - and this chapter
 * disagrees with ITSELF about three of them, so each disagreement is
 * named where it lives and measured in the suite.
 *
 * 1. ENABLE-PROTECTED. 39.6.2.1's list is CTRLA.ONDEMAND/RUNSTDBY,
 *    CTRLB, CTRLC, EVCTRL, ANACTRL. The individual property lines say
 *    Enable-Protected for REFCTRL, CTRLB and EVCTRL only - so REFCTRL is
 *    protected by its own description and missing from the list, while
 *    CTRLC and ANACTRL are in the list and say only Write-Synchronized.
 *    MEASURED, by writing each raw under a running converter: REFCTRL,
 *    CTRLB and EVCTRL DISCARD the write and CTRLC and ANACTRL TAKE it -
 *    so the individual property lines are right and 39.6.2.1's list is
 *    two entries too long. Everything here still writes all five only
 *    while the converter is DISABLED, which is correct under either
 *    reading and is what makes CTRLC's and ANACTRL's synchronization
 *    honest; the runtime verbs refuse while it is enabled.
 * 2. WRITE-SYNCHRONIZED, and THE BUS ERROR THAT MAKES THIS CHAPTER
 *    DIFFERENT. 39.6.8: "If an operation that require synchronization is
 *    executed while its busy bit is on, the operation is discarded AND A
 *    BUS ERROR IS GENERATED." The ADC's and the DAC's chapters promise a
 *    silent discard; here the threatened penalty is a HardFault. So
 *    EVERY synchronized write in this file WAITS BEFORE WRITING, not
 *    after, and returns false rather than storing into a busy register -
 *    which is why `select()` is a bool and not the void store
 *    samc/adc.hpp's is. The bits that exist: SWRST, ENABLE, CTRLC,
 *    INPUTCTRL (SYNCBUSY calls it MUXCTRL), WINCTRL, WINLT, WINUT,
 *    OFFSETCORR, GAINCORR, SHIFTCORR, SWTRIG, ANACTRL.
 *    NOTE REFCTRL: 39.6.8's prose lists it among the registers needing
 *    write synchronization, and SYNCBUSY HAS NO BIT FOR IT. One of the
 *    two statements has to be wrong; this driver treats REFCTRL as
 *    enable-protected only, as its own property line does.
 * 3. WRITE-ONLY. SWTRIG's two bits are `W` in 39.8.17 and self-clearing;
 *    there is nothing to read back, and `start()`/`flush()` say so.
 * 4. NEITHER. EVCTRL, INTENSET/CLR, INTFLAG, SEQSTATUS, SEQCTRL, and
 *    DBGCTRL - which 39.8.22 also says is not reset by a software reset.
 *
 * NOT PAC-PROTECTED (39.5.8): INTFLAG, and nothing else.
 *
 * ---------------------------------------------------------------------
 * THE POST-PROCESSING, which is the reason a Multislope wants this part.
 *
 *      Data = (Data0 + OFFSETCORR) x GAINCORR / 2^SHIFTCORR
 *
 * with OFFSETCORR a signed 24-bit value, GAINCORR an unsigned 14-bit one
 * and SHIFTCORR four bits (39.6.3.4 and each register's own description,
 * which print the formula three more times). IT RUNS IN 24-BIT UNITS,
 * measured: `Data0` is the 24-bit filter output, not the 16-bit datum
 * the chapter's "unsigned integer defined on 16 bits" claims, so an
 * OFFSETCORR of 1 moves the reading by ONE 256th of a 16-bit count.
 * `sdadc_corrected()` is that formula, in those units. Two more things
 * follow that no caller should have to discover:
 *
 *  - GAINCORR'S RESET VALUE IS ONE, NOT ZERO, and a GAINCORR of zero
 *    multiplies every result to nothing. `sdadc_config_valid()` refuses
 *    it. (Erratum 1.18.3 is exactly this on revision B silicon, where
 *    the reset value WAS zero; the reset values printed in this revision
 *    of the chapter - GAINCORR 0x0001 and CTRLB.SKPCNT 2 - ARE that
 *    item's workaround, baked into the register descriptions.)
 *  - THE GAIN IS AN INTEGER OVER A POWER OF TWO, not a fixed-point
 *    fraction like the SAR's GAINCORR: a gain of 1.5 is GAINCORR 3 with
 *    SHIFTCORR 1. `sdadc_gain_of()` and `sdadc_shift_for_gain()` do that
 *    arithmetic once.
 *
 * CHOPPING (ANACTRL.ONCHOP) is the analog half of the same idea: 39.6.3.4
 * says offset error "can be compensated by setting the Chopper mode ON",
 * and table 45-27's DC figures are all taken with it on.
 *
 * ---------------------------------------------------------------------
 * WHAT THE HEADER HAS AND THE CHAPTER DOES NOT: `REFCTRL.REFRANGE`, two
 * bits at 5:4, present in the header's register mask (0xB3) and in NO
 * part of chapter 39 - not the bit table, not the register summary, not
 * the prose. It is exposed here because the house rule is that the
 * header wins on what EXISTS, and it is documented as unknown because
 * nothing in the documents of record says what it does.
 *
 * ---------------------------------------------------------------------
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F. Four SDADC
 * items and one device-level item; ONE is this silicon.
 *
 *  - 1.8.10 DAC Output Reference Selection (device level, ALL
 *    REVISIONS): with REFCTRL.REFSEL = DAC, starting a conversion makes
 *    THE DAC'S OUTPUT VOLTAGE noisy. The workaround is
 *    REFCTRL.ONREFBUF = 1, which `sdadc_config_valid()` REQUIRES for the
 *    DAC reference - and for INTREF too, because 39.8.2's own Note asks
 *    for the buffer on both internal references. Measured in the suite.
 *  - 1.8.7 DMA Write Access in standby (ALL REVISIONS) names
 *    `SDADC: SWTRIG` among the registers a SleepWalking DMA write may
 *    fail to reach. The workaround is the application's - use Idle and
 *    not Standby when a DMA channel writes SWTRIG - and it is stated on
 *    `start()`; a driver cannot know which sleep mode is coming.
 *  - NOT this silicon, and each is a read-the-row trap: 1.18.1 (the APB
 *    clock having to be at least twice GCLK_SDADC or the first
 *    conversion of a sequence is invalid) is REVISION B ONLY; 1.18.3
 *    (GAINCORR and SKPCNT resetting to zero) is REVISION B ONLY, and
 *    this revision's reset values are its workaround; 1.18.2 (poor INL
 *    near VREF, workaround "limit the range to +/- 0.7 x VREF") and
 *    1.18.4 (power consumption) are revisions B..E. NOTE that table
 *    45-26 has ABSORBED half of 1.18.2 as a specification: for
 *    VREF >= VDDANA - 0.3 V the input conversion range IS +/-0.7 x VREF
 *    on every revision, so a full-scale differential against VDDANA is
 *    outside specification here whatever the errata say.
 *
 * ---------------------------------------------------------------------
 * NOT BUILT (docs/samc/sdadc.md carries the list): sleep behaviour
 * beyond the two CTRLA bits, and the C20 half of the family, which has
 * no SDADC at all.
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/supc.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// The reference vocabulary - this converter's own
// =============================================================================

/**
 * REFCTRL.REFSEL (39.8.2). FOUR codes and none of them Reserved, which
 * makes this the only one of the family's three converters whose
 * reference field has no illegal value.
 *
 * `SdadcRef` is deliberately not `brio::Ref` (samc/adc.hpp's, six codes)
 * nor `DacRef` (samc/dac.hpp's, three): on this family there is no shared
 * reference block at all, so one enum would be a type no register accepts
 * - the ADC campaign's judgment call 1, taken there and followed twice
 * since.
 */
enum class SdadcRef : uint8_t {
    /// INTREF: the SUPC bandgap, whose LEVEL is SUPC.VREF.SEL
    /// (samc/supc.hpp's `VrefLevel`) and not a knob of this peripheral.
    /// 39.8.2's Note asks for the reference BUFFER with it, which
    /// `sdadc_config_valid()` requires.
    intref = SDADC_REFCTRL_REFSEL_INTREF_Val,
    /// The VREFB pin - PA04 on this family, which the device header
    /// spells VREFP. Table 45-26 bounds it at 1 V .. VDDANA.
    vrefb = SDADC_REFCTRL_REFSEL_AREFB_Val,
    /// The DAC's output (samc/dac.hpp, CTRLB.IOEN). ERRATUM 1.8.10 is
    /// about exactly this selection and its workaround is the buffer,
    /// which `sdadc_config_valid()` requires.
    dac = SDADC_REFCTRL_REFSEL_DAC_Val,
    /// The analog supply. The header calls it INTVCC.
    vddana = SDADC_REFCTRL_REFSEL_INTVCC_Val,
};

/// Every REFSEL code is implemented; this exists so the family fixture
/// can say so out loud and so a cast from a wider integer is caught.
constexpr bool sdadc_ref_valid(SdadcRef r) {
    return static_cast<uint8_t>(r) <= static_cast<uint8_t>(SdadcRef::vddana);
}

/**
 * The millivolts of a reference, for util/analog.hpp's arithmetic.
 *
 * `known_mv` is the millivolts of the reference's SOURCE where this
 * header cannot know it - VDDANA for `vddana`, the pin's voltage for
 * `vrefb`, the DAC's output for `dac` - and zero means "not known",
 * which yields zero rather than a plausible lie. `intref_level` is what
 * stands in SUPC.VREF.SEL.
 */
constexpr uint16_t sdadc_ref_mv(SdadcRef r, uint16_t known_mv = 0,
                                VrefLevel intref_level = VrefLevel::v1_024) {
    switch (r) {
    case SdadcRef::intref: return vref_mv(intref_level);
    case SdadcRef::vrefb:
    case SdadcRef::dac:
    case SdadcRef::vddana: return known_mv;
    }
    return 0;
}

// =============================================================================
// The knobs
// =============================================================================

/// CTRLB.OSR (39.8.3): the decimation filter's over-sampling ratio.
/// Codes 0x5..0x7 are Reserved (the register description prints the
/// range as "0x4 - 0xF", which is both a typo and impossible in a
/// three-bit field).
enum class SdadcOsr : uint8_t {
    osr64 = SDADC_CTRLB_OSR_OSR64_Val,
    osr128 = SDADC_CTRLB_OSR_OSR128_Val,
    osr256 = SDADC_CTRLB_OSR_OSR256_Val,
    osr512 = SDADC_CTRLB_OSR_OSR512_Val,
    osr1024 = SDADC_CTRLB_OSR_OSR1024_Val,
};

constexpr bool sdadc_osr_valid(SdadcOsr o) {
    return static_cast<uint8_t>(o) <= static_cast<uint8_t>(SdadcOsr::osr1024);
}

/// The ratio itself: 64 << code, table 45-26's 64 .. 1024.
constexpr uint16_t sdadc_osr_value(SdadcOsr o) {
    return static_cast<uint16_t>(64u << static_cast<uint8_t>(o));
}

/// CTRLC.WINMODE (39.8.11). The names are the conditions and not the
/// register's ABOVE/BELOW/INSIDE/OUTSIDE, which mean different things in
/// different chapters of this datasheet. Codes 0x5..0x7 are Reserved.
enum class SdadcWindow : uint8_t {
    none = 0,
    above_lower = 1,   ///< RESULT > WINLT
    below_upper = 2,   ///< RESULT < WINUT
    inside = 3,        ///< WINLT < RESULT < WINUT
    outside = 4,       ///< WINUT < RESULT or RESULT < WINLT
};

constexpr bool sdadc_window_valid(SdadcWindow w) {
    return static_cast<uint8_t>(w) <= static_cast<uint8_t>(SdadcWindow::outside);
}

// ---- the clock arithmetic ---------------------------------------------------

/**
 * CTRLB.PRESCALER: the divider is 2 x (PRESCALER + 1), so the field's
 * 256 codes give the EVEN divisors 2, 4, 6 ... 512 (39.5.3, 39.8.3 and
 * figure 39-2). The device header's DIV2/DIV4/DIV8/.../DIV256
 * enumerators are the SAM D21's power-of-two prescaler and are not used
 * anywhere in this file - see the header comment.
 */
constexpr uint16_t sdadc_prescaler_divisor(uint8_t prescaler) {
    return static_cast<uint16_t>(2u * (static_cast<uint16_t>(prescaler) + 1u));
}

/// CLK_SDADC, the "sampling clock speed" table 45-26 bounds.
constexpr uint32_t sdadc_clock_hz(uint32_t gclk_hz, uint8_t prescaler) {
    return gclk_hz / sdadc_prescaler_divisor(prescaler);
}

/// CLK_SDADC_FS = CLK_SDADC / 4 (39.5.3, "the reduction comes from the
/// phase generator between the prescaler and the SDADC").
constexpr uint32_t sdadc_sampling_hz(uint32_t gclk_hz, uint8_t prescaler) {
    return sdadc_clock_hz(gclk_hz, prescaler) / 4u;
}

/// Table 45-26's CLK_SDADC bounds.
inline constexpr uint32_t sdadc_clock_min_hz = 1'000'000;
inline constexpr uint32_t sdadc_clock_max_hz = 6'000'000;

constexpr bool sdadc_clock_in_range(uint32_t gclk_hz, uint8_t prescaler) {
    const uint32_t f = sdadc_clock_hz(gclk_hz, prescaler);
    return f >= sdadc_clock_min_hz && f <= sdadc_clock_max_hz;
}

/// The prescaler putting CLK_SDADC closest to `target_hz` while staying
/// inside table 45-26; 0xFF (which is in range at no sane generator) if
/// nothing is.
constexpr uint8_t sdadc_prescaler_for(uint32_t gclk_hz, uint32_t target_hz) {
    uint8_t best = 0xFFu;
    uint32_t best_err = 0xFFFFFFFFu;
    for (uint16_t i = 0; i < 256u; ++i) {
        const uint8_t p = static_cast<uint8_t>(i);
        if (!sdadc_clock_in_range(gclk_hz, p)) {
            continue;
        }
        const uint32_t f = sdadc_clock_hz(gclk_hz, p);
        const uint32_t err = f > target_hz ? f - target_hz : target_hz - f;
        if (err < best_err) {
            best_err = err;
            best = p;
        }
    }
    return best;
}

// ---- the post-processing arithmetic ----------------------------------------

/// The gain the pair (GAINCORR, SHIFTCORR) asks for, in PARTS PER
/// THOUSAND - the correction is an integer over a power of two, so 1.5 is
/// GAINCORR 3 with SHIFTCORR 1 and this returns 1500.
constexpr uint32_t sdadc_gain_permille(uint16_t gain, uint8_t shift) {
    return (static_cast<uint32_t>(gain) * 1000u) >> (shift > 31u ? 31u : shift);
}

/**
 * The whole post-processing formula of 39.6.3.4, in software - what a
 * caller can predict a corrected result will be from an uncorrected one.
 *
 * IT WORKS IN THE RAW 24-BIT UNITS THE SILICON USES, which is the
 * campaign's central measurement: `data0` and the return value are
 * `result24()`s, not `result()`s, and the saturation is the register's
 * own +/-2^23. A caller thinking in 16-bit counts multiplies by
 * `sdadc_raw_per_count` on the way in and divides on the way out.
 */
constexpr int32_t sdadc_corrected(int32_t data0, int32_t offset, uint16_t gain,
                                  uint8_t shift) {
    const int64_t v = (static_cast<int64_t>(data0) + offset) *
                      static_cast<int64_t>(gain);
    const int64_t out = v >> (shift > 31u ? 31u : shift);
    if (out > 8388607) {
        return 8388607;
    }
    if (out < -8388608) {
        return -8388608;
    }
    return static_cast<int32_t>(out);
}

/// The full scale of the SPECIFIED converter: sixteen bits, SIGNED, so a
/// reading of +32767 is +VREF and -32768 is -VREF. This is the
/// `half_steps` util/analog.hpp's `adc_mv_signed()` wants.
inline constexpr uint32_t sdadc_half_steps = 32768;
/// The full scale of the raw 24-bit datapath under it.
inline constexpr int32_t sdadc_raw_half_steps = 8388608;
/// How many raw 24-bit units there are in one 16-bit count - the factor
/// between `result()` and `result24()`, and the units OFFSETCORR speaks.
inline constexpr int32_t sdadc_raw_per_count = 256;

// =============================================================================
// The configuration
// =============================================================================

/// EVCTRL (39.8.4), enable-protected, both directions in one struct.
struct SdadcEventControl {
    /// EVCTRL.FLUSHEI: an incoming event flushes the pipeline and
    /// restarts. 39.6.6: if FLUSH and START arrive together, FLUSH wins.
    bool flush_in = false;
    /// EVCTRL.STARTEI: an incoming event starts a conversion.
    bool start_in = false;
    bool invert_flush = false;   ///< FLUSHINV
    bool invert_start = false;   ///< STARTINV
    bool result_out = false;     ///< RESRDYEO
    bool window_out = false;     ///< WINMONEO
};

/// Inverting an input nobody listens to is a configuration with no
/// meaning - the same refusal samc/adc.hpp, samc/dac.hpp and samc/ac.hpp
/// all make.
constexpr bool sdadc_event_control_valid(const SdadcEventControl& e) {
    return (!e.invert_start || e.start_in) && (!e.invert_flush || e.flush_in);
}

/// The whole configuration of the converter.
struct SdadcConfig {
    // -- REFCTRL (enable-protected) --
    SdadcRef reference = SdadcRef::vddana;
    /// REFCTRL.ONREFBUF: the reference buffer, which drops the load on
    /// the reference from 5 uA to 0.1 uA. 39.8.2's Note asks for it with
    /// INTREF or the DAC, and erratum 1.8.10 makes it the workaround for
    /// the DAC selection; `sdadc_config_valid()` requires it for both.
    bool reference_buffer = false;
    /// REFCTRL.REFRANGE - TWO BITS THE DEVICE HEADER HAS AND CHAPTER 39
    /// DOES NOT DESCRIBE ANYWHERE. Exposed because the header is the
    /// authority on what exists; left at its reset value because nothing
    /// in the documents of record says what it selects.
    uint8_t reference_range = 0;

    // -- CTRLB (enable-protected) --
    /// CLK_SDADC = GCLK_SDADC / (2 x (PRESCALER + 1)) - LINEAR, see the
    /// header comment. Table 45-26 wants the result in 1..6 MHz.
    uint8_t prescaler = 3;
    /// The decimation ratio, and the only rate-against-noise knob there
    /// is. 39.8.3: it must never be changed while the converter runs.
    SdadcOsr osr = SdadcOsr::osr256;
    /// CTRLB.SKPCNT: decimation windows spent warming the filter before
    /// the one that is returned. The reset value is 2, which is
    /// 39.6.2.3's "the first valid sample starts from the third sample
    /// onward"; it multiplies a SINGLE conversion's time by SKPCNT + 1,
    /// measured window for window. FEWER THAN TWO IS REFUSED IN
    /// SINGLE-CONVERSION MODE - the chapter, the reset value and erratum
    /// 1.18.3's own workaround all say those windows are invalid data.
    uint8_t skip_count = 2;

    // -- CTRLC (in 39.6.2.1's enable-protected list, write-synchronized
    //    by its own property line) --
    bool free_running = false;

    // -- WINCTRL / WINLT / WINUT (write-synchronized) --
    SdadcWindow window = SdadcWindow::none;
    /// The thresholds are compared against the SIGNED result; the
    /// registers are 24 bits wide and take the value in the same
    /// left-adjusted placement RESULT reports it in
    /// (`sdadc_threshold_word()`).
    int16_t window_low = 0;      ///< WINLT
    int16_t window_high = 0;     ///< WINUT

    // -- the post-processing (write-synchronized) --
    /// OFFSETCORR, a SIGNED 24-bit value ADDED before the gain multiply,
    /// IN THE RAW 24-BIT UNITS the datapath works in: 256 of these is
    /// one count of `result()`.
    int32_t offset_correction = 0;
    /// GAINCORR, an UNSIGNED 14-bit INTEGER. Unity is ONE, not a
    /// fixed-point 0x800, and ZERO multiplies every result away - which
    /// is refused.
    uint16_t gain_correction = 1;
    /// SHIFTCORR, a right shift applied after the multiply: the pair
    /// (gain, shift) is how a fractional gain is spelled.
    uint8_t shift_correction = 0;

    // -- SEQCTRL (neither protected nor synchronized) --
    /// One bit per differential pair to include in an automatic sequence
    /// (39.6.2.7). Zero disables the sequencer and the conversion uses
    /// INPUTCTRL.MUXSEL.
    uint8_t sequence = 0;

    // -- ANACTRL (in 39.6.2.1's enable-protected list, write-synchronized
    //    by its own property line) --
    /// ANACTRL.ONCHOP: the input chopper, which is how 39.6.3.4 says
    /// offset error is compensated in the ANALOG domain. Table 45-27's
    /// DC figures are all taken with it on.
    bool chopper = false;
    /// ANACTRL.CTLSDADC, the bias current control, which 39.8.21 calls
    /// "used for Debug/Characterization". THE FIELD WIDTH DISAGREES:
    /// 39.8.21 draws five bits (4:0) and the device header declares six
    /// (CTRSDADC, mask 0x3F). The header wins in code, as the house rule
    /// says, and the suite measures which bits stay written.
    uint8_t bias_control = 0;
    /// ANACTRL.BUFTEST, a bit the register description names and gives
    /// NO description of at all. Exposed for completeness; nothing here
    /// can say what it does.
    bool buffer_test = false;

    // -- CTRLA (enable-protected per 39.6.2.1; ENABLE and SWRST
    //    write-synchronized) --
    /// CTRLA.RUNSTDBY (table 39-1).
    bool run_standby = false;
    /// CTRLA.ONDEMAND: the analog block is powered off when a conversion
    /// completes and pays its start-up again on the next request.
    bool on_demand = false;

    // -- DBGCTRL (survives a software reset) --
    bool debug_run = false;

    // -- EVCTRL (enable-protected) --
    SdadcEventControl events{};
};

/**
 * What the compile-time form static_asserts and the runtime form returns
 * false for. Every rule is the chapter's or the errata's.
 */
constexpr bool sdadc_config_valid(const SdadcConfig& c) {
    if (!sdadc_ref_valid(c.reference) || !sdadc_osr_valid(c.osr) ||
        !sdadc_window_valid(c.window)) {
        return false;
    }
    // 39.8.2's Note ("the reference buffer should be enabled when using
    // the internal INTREF or DAC output as reference") and erratum
    // 1.8.10, whose whole workaround is that bit.
    if ((c.reference == SdadcRef::intref || c.reference == SdadcRef::dac) &&
        !c.reference_buffer) {
        return false;
    }
    if (c.reference_range > 3u) {
        return false;
    }
    if (c.skip_count > 15u || c.shift_correction > 15u) {
        return false;
    }
    // 39.6.2.3 ("the first valid sample starts from the third sample
    // onward"), the reset value of 2, and erratum 1.18.3's workaround
    // ("Write CTRLB.SKPCNT to 2 before running single conversions") all
    // say the same thing: a single conversion that skips fewer than two
    // decimation windows returns the filter still filling. Free running
    // pays the warm-up once, at the start, so it is exempt.
    if (!c.free_running && c.skip_count < 2u) {
        return false;
    }
    // GAINCORR is 14 bits, and a gain of ZERO multiplies every result
    // away - which is what erratum 1.18.3 describes happening by
    // accident on revision B, where it was the reset value.
    if (c.gain_correction == 0u || c.gain_correction > 0x3FFFu) {
        return false;
    }
    // SEQCTRL is three bits, and a bit for a pair this package does not
    // bond asks for a conversion on pads that do not exist.
    if (c.sequence > 0x7u) {
        return false;
    }
    for (uint8_t p = 0; p < 3u; ++p) {
        if ((c.sequence & (1u << p)) != 0u && !sdadc_pair_exists(p)) {
            return false;
        }
    }
    if (c.bias_control > 0x3Fu) {
        return false;
    }
    return sdadc_event_control_valid(c.events);
}

/// CLK_SDADC cycles for one RESULT: OSR decimation windows of four
/// cycles each, times SKPCNT + 1 in SINGLE conversion mode (39.6.3.3 and
/// table 45-26's own two rows). Free running pays the warm-up once, at
/// the start, and not per result.
constexpr uint32_t sdadc_conversion_cycles(const SdadcConfig& c) {
    const uint32_t window = 4u * static_cast<uint32_t>(sdadc_osr_value(c.osr));
    return c.free_running ? window
                          : window * (static_cast<uint32_t>(c.skip_count) + 1u);
}

/// Results per second at a given GCLK_SDADC rate.
constexpr uint32_t sdadc_result_hz(uint32_t gclk_hz, const SdadcConfig& c) {
    const uint32_t cycles = sdadc_conversion_cycles(c);
    return cycles == 0u ? 0u : sdadc_clock_hz(gclk_hz, c.prescaler) / cycles;
}

/// One conversion's time in microseconds, the number a caller pacing the
/// converter by hand actually wants.
constexpr uint32_t sdadc_conversion_us(uint32_t gclk_hz, const SdadcConfig& c) {
    const uint32_t f = sdadc_clock_hz(gclk_hz, c.prescaler);
    return f == 0u ? 0u
                   : static_cast<uint32_t>(
                         (static_cast<uint64_t>(sdadc_conversion_cycles(c)) *
                          1'000'000ULL) / f);
}

// =============================================================================
// The result's placement
// =============================================================================

/**
 * 39.8.19: "The RESULT is a signed integer value with 24-bit size. The
 * SDADC conversion result is LEFT-ADJUSTED in the RESULT register."
 *
 * The first half of that is exact and the second is misleading: the
 * SPECIFIED sixteen bits are at [23:8], but the eight below them are not
 * padding - the register saturates at +/-2^23 rather than at a
 * left-shifted +/-2^15, and the corrections act in those units. So
 * `sdadc_result_of()` extracts the datum the chapter specifies and
 * `sdadc_raw_signed()` gives the whole thing.
 *
 * The window thresholds go through the same placement, because the
 * comparison happens against the register.
 */
constexpr int16_t sdadc_result_of(uint32_t raw) {
    return static_cast<int16_t>((raw >> 8) & 0xFFFFu);
}
/// The whole 24-bit register as a signed value.
constexpr int32_t sdadc_raw_signed(uint32_t raw) {
    const uint32_t v = raw & 0x00FFFFFFu;
    return static_cast<int32_t>(v & 0x00800000u ? v | 0xFF000000u : v);
}
constexpr uint32_t sdadc_threshold_word(int16_t value) {
    return (static_cast<uint32_t>(static_cast<uint16_t>(value)) << 8) & 0x00FFFFFFu;
}

// =============================================================================
// The converter
// =============================================================================

/**
 * The one sigma-delta converter. A monostate resource: this family has a
 * single instance on every C21 variant, so there is no index to carry.
 */
class Sdadc {
    static_assert(sdadc_count() == 1u,
                  "this device has no SDADC (chapter 39 is SAM C21 only)");

public:
    Sdadc() = delete;

    /// How many differential pairs MUXSEL can NAME. Which of them this
    /// package bonds is `pair_exists()`.
    static constexpr uint8_t channels = sdadc_channels();
    static constexpr uint8_t gclk_id = sdadc_gclk_id();
    /// The signed full scale: +/- this is +/- VREF.
    static constexpr uint32_t half_steps = sdadc_half_steps;

    static constexpr IRQn_Type irq() { return SDADC_IRQn; }

    // ---- the vocabularies this peripheral publishes -------------------------
    //
    // evsys.hpp owns the FABRIC and dmac.hpp owns the CHANNELS; the codes
    // of their tables that belong to the SDADC live here, probed from the
    // device header in samc/device_tables.hpp.

    /// Generator: a conversion result is available.
    static constexpr uint8_t resrdy_generator = sdadc_resrdy_generator();
    /// Generator: the window monitor's condition matched.
    static constexpr uint8_t winmon_generator = sdadc_winmon_generator();
    /// User: start a conversion. 39.6.6 - ASYNCHRONOUS PATH ONLY, which
    /// `start_on()` enforces.
    static constexpr uint8_t start_event_user = sdadc_start_user();
    /// User: flush the pipeline and restart. Same restriction.
    static constexpr uint8_t flush_event_user = sdadc_flush_user();
    /// DMAC trigger: the one DMA request this peripheral has (39.6.4).
    static constexpr uint8_t dma_trigger_resrdy = sdadc_dma_resrdy_id();

    /// INTFLAG / INTENSET bits, named.
    static constexpr uint8_t flag_resrdy = SDADC_INTFLAG_RESRDY_Msk;
    static constexpr uint8_t flag_overrun = SDADC_INTFLAG_OVERRUN_Msk;
    static constexpr uint8_t flag_winmon = SDADC_INTFLAG_WINMON_Msk;

    static sdadc_registers_t& regs() { return *SDADC_REGS; }

    // ---- per-package pad legality ------------------------------------------

    /// Whether BOTH pads of a differential pair are bonded on this
    /// package. The E has pair 0 alone, the G adds pair 1, only the J
    /// carries pair 2.
    static constexpr bool pair_exists(uint8_t pair) {
        return sdadc_pair_exists(pair);
    }
    /// The negative pad of a pair, as a port letter and a pin number, or
    /// ('\0', 0xFF) where the package does not bond it. Published as data
    /// so a caller does not have to know the pinout.
    static constexpr char negative_port(uint8_t pair) {
        return sdadc_pad_port(sdadc_negative_pad(pair));
    }
    static constexpr uint8_t negative_pin(uint8_t pair) {
        return sdadc_pad_pin(sdadc_negative_pad(pair));
    }
    static constexpr char positive_port(uint8_t pair) {
        return sdadc_pad_port(sdadc_positive_pad(pair));
    }
    static constexpr uint8_t positive_pin(uint8_t pair) {
        return sdadc_pad_pin(sdadc_positive_pad(pair));
    }

    static constexpr bool config_valid(const SdadcConfig& c) {
        return sdadc_config_valid(c);
    }

    // ---- claim and teardown ------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_c(MCLK_APBCMASK_SDADC_Msk, on); }

    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    static uint32_t sync_busy() { return regs().SDADC_SYNCBUSY; }

    /**
     * Bounded spin until every bit of `mask` is CLEAR.
     *
     * This is used BEFORE a synchronized write and not after, which is
     * the discipline 39.6.8 forces on this chapter alone: a write made
     * while its own busy bit stands is "discarded and A BUS ERROR IS
     * GENERATED", and a bus error on this core is a HardFault. The ADC's
     * and the DAC's chapters promise a silent discard; this one does not.
     */
    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().SDADC_SYNCBUSY, mask, false, spins);
    }

    /// CTRLA.SWRST: every register except DBGCTRL back to reset, and the
    /// converter disabled.
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!sync_wait(SDADC_SYNCBUSY_SWRST_Msk | SDADC_SYNCBUSY_ENABLE_Msk, spins)) {
            return false;
        }
        regs().SDADC_CTRLA = SDADC_CTRLA_SWRST_Msk;
        return sync_wait(SDADC_SYNCBUSY_SWRST_Msk, spins);
    }

    /// CTRLA.ENABLE, preserving ONDEMAND and RUNSTDBY.
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        if (!sync_wait(SDADC_SYNCBUSY_ENABLE_Msk | SDADC_SYNCBUSY_SWRST_Msk, spins)) {
            return false;
        }
        const uint8_t v =
            static_cast<uint8_t>(regs().SDADC_CTRLA & ~SDADC_CTRLA_ENABLE_Msk);
        regs().SDADC_CTRLA =
            on ? static_cast<uint8_t>(v | SDADC_CTRLA_ENABLE_Msk) : v;
        return sync_wait(SDADC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() {
        return (regs().SDADC_CTRLA & SDADC_CTRLA_ENABLE_Msk) != 0u;
    }

    // ---- configuration ------------------------------------------------------

    /**
     * The whole chapter in one call: bus clock, generic clock, reset, the
     * enable-protected registers, the synchronized ones, enable.
     *
     * `gclk_hz` is what the generator runs at, and it is here for ONE
     * reason: to refuse a prescaler that puts CLK_SDADC outside table
     * 45-26's 1 MHz .. 6 MHz. Pass 0 to skip that check when the rate is
     * genuinely unknown.
     *
     * The input multiplexer is NOT set here (INPUTCTRL is neither
     * enable-protected nor part of the configuration this struct
     * describes): `select()` is the verb, and the reset value is pair 0.
     */
    static bool init(uint8_t generator, const SdadcConfig& cfg,
                     uint32_t gclk_hz = 0, uint32_t spins = 0xFFFFu) {
        if (!sdadc_config_valid(cfg)) {
            return false;
        }
        if (gclk_hz != 0u && !sdadc_clock_in_range(gclk_hz, cfg.prescaler)) {
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
        // REFCTRL has NO SYNCBUSY bit whatever 39.6.8's prose lists.
        regs().SDADC_REFCTRL = reference_word(cfg);
        regs().SDADC_CTRLB = control_b_word(cfg);
        regs().SDADC_EVCTRL = event_word(cfg.events);
        regs().SDADC_SEQCTRL = static_cast<uint8_t>(SDADC_SEQCTRL_SEQEN(cfg.sequence));
        regs().SDADC_DBGCTRL = cfg.debug_run ? SDADC_DBGCTRL_DBGRUN_Msk : 0u;

        // Write-synchronized. Nothing is converting yet, so each wait is
        // the bus's and not a conversion's - but every one of them still
        // happens BEFORE the store (39.6.8's bus error).
        if (!write_sync(regs().SDADC_CTRLC, SDADC_SYNCBUSY_CTRLC_Msk,
                        cfg.free_running ? SDADC_CTRLC_FREERUN_Msk : 0u, spins) ||
            !write_sync(regs().SDADC_ANACTRL, SDADC_SYNCBUSY_ANACTRL_Msk,
                        analog_word(cfg), spins) ||
            !write_sync(regs().SDADC_OFFSETCORR, SDADC_SYNCBUSY_OFFSETCORR_Msk,
                        static_cast<uint32_t>(cfg.offset_correction) & 0x00FFFFFFu,
                        spins) ||
            !write_sync(regs().SDADC_GAINCORR, SDADC_SYNCBUSY_GAINCORR_Msk,
                        static_cast<uint16_t>(cfg.gain_correction & 0x3FFFu), spins) ||
            !write_sync(regs().SDADC_SHIFTCORR, SDADC_SYNCBUSY_SHIFTCORR_Msk,
                        static_cast<uint8_t>(cfg.shift_correction & 0xFu), spins) ||
            !write_sync(regs().SDADC_WINLT, SDADC_SYNCBUSY_WINLT_Msk,
                        sdadc_threshold_word(cfg.window_low), spins) ||
            !write_sync(regs().SDADC_WINUT, SDADC_SYNCBUSY_WINUT_Msk,
                        sdadc_threshold_word(cfg.window_high), spins) ||
            !write_sync(regs().SDADC_WINCTRL, SDADC_SYNCBUSY_WINCTRL_Msk,
                        static_cast<uint8_t>(SDADC_WINCTRL_WINMODE(
                            static_cast<uint8_t>(cfg.window))),
                        spins) ||
            !write_sync(regs().SDADC_INPUTCTRL, SDADC_SYNCBUSY_INPUTCTRL_Msk,
                        static_cast<uint8_t>(SDADC_INPUTCTRL_MUXSEL(0u)), spins)) {
            return false;
        }

        regs().SDADC_INTENCLR = flag_resrdy | flag_overrun | flag_winmon;
        regs().SDADC_INTFLAG = flag_resrdy | flag_overrun | flag_winmon;

        // CTRLA last: ONDEMAND and RUNSTDBY are enable-protected and not
        // synchronized, ENABLE is synchronized.
        regs().SDADC_CTRLA = static_cast<uint8_t>(
            (cfg.on_demand ? SDADC_CTRLA_ONDEMAND_Msk : 0u) |
            (cfg.run_standby ? SDADC_CTRLA_RUNSTDBY_Msk : 0u));
        if (!enable(true, spins)) {
            return false;
        }
        cfg_ = cfg;
        return true;
    }

    /// The compile-time twin: every rule of `sdadc_config_valid()`
    /// becomes a compile error instead of a false return.
    template <SdadcConfig cfg>
    static bool init(uint8_t generator, uint32_t gclk_hz = 0,
                     uint32_t spins = 0xFFFFu) {
        static_assert(sdadc_config_valid(cfg),
                      "brio SdadcConfig: see sdadc_config_valid() - a Reserved "
                      "OSR or window code, an internal reference without "
                      "REFCTRL.ONREFBUF (39.8.2's Note and erratum 1.8.10), a "
                      "GAINCORR of zero (which multiplies every result away), "
                      "a single-conversion SKPCNT under two (39.6.2.3's "
                      "invalid first samples), a sequence bit for a pair this "
                      "package does not bond, or an inverted event input "
                      "nothing listens to");
        return init(generator, cfg, gclk_hz, spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        bus_clock(false);
    }

    /// The configuration in force, as `init()` was given it.
    static const SdadcConfig& config() { return cfg_; }
    static SdadcRef reference() { return cfg_.reference; }

    /// The reference in millivolts, for `to_mv()` and for a caller doing
    /// its own arithmetic.
    static uint16_t reference_mv(uint16_t known_mv = 0,
                                 VrefLevel intref_level = VrefLevel::v1_024) {
        return sdadc_ref_mv(cfg_.reference, known_mv, intref_level);
    }

    /// A SIGNED reading in millivolts, through util/analog.hpp's
    /// arithmetic and this converter's own scale.
    static int16_t to_mv(int32_t counts, uint16_t known_mv = 0,
                         VrefLevel intref_level = VrefLevel::v1_024) {
        return adc_mv_signed(counts, half_steps,
                             sdadc_ref_mv(cfg_.reference, known_mv, intref_level));
    }

    // ---- input selection ----------------------------------------------------

    /**
     * INPUTCTRL.MUXSEL: which differential PAIR converts.
     *
     * Codes 0x3..0xF are Reserved (39.8.9), and a pair whose pads this
     * package does not bond is refused as well - both are a false return
     * here and a compile error in the templated form.
     *
     * A BOOL AND NOT A VOID STORE, unlike samc/adc.hpp's `select()`:
     * INPUTCTRL is write-synchronized, and on this chapter a write made
     * while the busy bit stands is a BUS ERROR (39.6.8), so the wait has
     * to happen before the store and its failure has to be reportable.
     */
    static bool select(uint8_t pair, uint32_t spins = 0xFFFFu) {
        if (pair >= channels || !pair_exists(pair)) {
            return false;
        }
        return write_sync(regs().SDADC_INPUTCTRL, SDADC_SYNCBUSY_INPUTCTRL_Msk,
                          static_cast<uint8_t>(SDADC_INPUTCTRL_MUXSEL(pair)), spins);
    }

    /// The compile-time twin, so a pair this package does not bond is a
    /// compile error and not a false return.
    template <uint8_t pair>
    static bool select(uint32_t spins = 0xFFFFu) {
        static_assert(pair < sdadc_channels(),
                      "MUXSEL names three differential pairs; 0x3..0xF are "
                      "Reserved (39.8.9)");
        static_assert(sdadc_pair_bonded<pair>,
                      "this package does not bond both pads of that SDADC "
                      "differential pair (the E has pair 0 alone, the G adds "
                      "pair 1, only the J carries pair 2)");
        return select(pair, spins);
    }

    /// MUXSEL as it stands.
    static uint8_t selected() {
        return static_cast<uint8_t>(regs().SDADC_INPUTCTRL & SDADC_INPUTCTRL_MUXSEL_Msk);
    }

    // ---- the pads -----------------------------------------------------------

    /// Which peripheral function a pad needs to be an SDADC NEGATIVE
    /// input, or -1 if it is not one on this package.
    static constexpr int negative_function(char port, uint8_t pin) {
        return sdadc_inn_code(port, pin);
    }
    static constexpr int positive_function(char port, uint8_t pin) {
        return sdadc_inp_code(port, pin);
    }
    /// The same for the external reference pin (VREFB; the device header
    /// spells the pad symbol VREFP).
    static constexpr int vrefb_function(char port, uint8_t pin) {
        return sdadc_vrefb_code(port, pin);
    }

    /**
     * 39.5.1's recommended pad configuration for a real analog source:
     * the peripheral function, digital input buffer off.
     *
     * NOT NEEDED TO READ THE PAD - an analog input is a direct connection,
     * so a pad left under PORT (even driven as an output) is converted as
     * it stands, which is what makes a wireless bench test of this
     * chapter possible at all. It IS needed to stop the digital receiver
     * toggling on a mid-rail voltage, and table 45-26 wants an external
     * anti-alias filter in front of every input besides.
     */
    template <typename P>
    static void claim_negative() {
        static_assert(sdadc_inn_pad_exists<P::port_letter, P::pin_number>,
                      "this package does not bond that pad as an SDADC "
                      "negative input");
        P::function(static_cast<PinFunction>(
                        sdadc_inn_code(P::port_letter, P::pin_number)),
                    PinConfig{});
    }
    template <typename P>
    static void claim_positive() {
        static_assert(sdadc_inp_pad_exists<P::port_letter, P::pin_number>,
                      "this package does not bond that pad as an SDADC "
                      "positive input");
        P::function(static_cast<PinFunction>(
                        sdadc_inp_code(P::port_letter, P::pin_number)),
                    PinConfig{});
    }
    template <typename P>
    static void claim_vrefb() {
        static_assert(sdadc_vrefb_pad_exists<P::port_letter, P::pin_number>,
                      "this package does not bond that pad as the SDADC's "
                      "external reference (VREFB is PA04 on this family)");
        P::function(static_cast<PinFunction>(
                        sdadc_vrefb_code(P::port_letter, P::pin_number)),
                    PinConfig{});
    }
    template <typename P>
    static void release_pad() { P::release(); }

    // ---- conversions --------------------------------------------------------

    /**
     * SWTRIG.START. Write-only and self-clearing (39.8.17), so there is
     * nothing to read back and `false` means the register was busy, not
     * that the conversion failed.
     *
     * ERRATUM 1.8.7 (all revisions) names `SDADC: SWTRIG` among the
     * registers a DMA write made during standby SleepWalking may not
     * reach; the workaround is to use Idle instead, and it is the
     * application's, since this header cannot know which sleep is coming.
     */
    static bool start(uint32_t spins = 0xFFFFu) {
        return write_sync(regs().SDADC_SWTRIG, SDADC_SYNCBUSY_SWTRIG_Msk,
                          static_cast<uint8_t>(SDADC_SWTRIG_START_Msk), spins);
    }

    /// SWTRIG.FLUSH: abandon everything in the pipeline and restart
    /// (39.8.17). "After the flush, the ADC will resume where it left
    /// off; i.e., if a conversion was pending, the ADC will start a new
    /// conversion."
    static bool flush(uint32_t spins = 0xFFFFu) {
        return write_sync(regs().SDADC_SWTRIG, SDADC_SYNCBUSY_SWTRIG_Msk,
                          static_cast<uint8_t>(SDADC_SWTRIG_FLUSH_Msk), spins);
    }

    static bool ready() { return (regs().SDADC_INTFLAG & flag_resrdy) != 0u; }
    static bool overrun() { return (regs().SDADC_INTFLAG & flag_overrun) != 0u; }

    /// RESULT as the silicon holds it: the whole 24-bit register, for a
    /// caller checking the placement or moving it by DMA.
    static uint32_t result_raw() {
        last_hit_ = (regs().SDADC_INTFLAG & flag_winmon) != 0u;
        return regs().SDADC_RESULT & SDADC_RESULT_RESULT_Msk;
    }

    /**
     * RESULT as the SPECIFIED sixteen-bit signed conversion: the top
     * sixteen bits of the register, which is what `half_steps` scales
     * and what table 45-26's "Res 16 bits" means.
     *
     * READING RESULT CLEARS BOTH RESRDY AND WINMON (39.8.7), so the
     * window verdict is captured first and `window_hit()` reports it for
     * the last value read - the same shape samc/adc.hpp has, for the same
     * reason.
     */
    static int16_t result() { return sdadc_result_of(result_raw()); }

    /// RESULT as the WHOLE signed 24-bit value, which is what the
    /// datapath and the corrections actually work in and what a caller
    /// wanting more than the specified sixteen bits should read. Clears
    /// the same two flags, being the same register read.
    static int32_t result24() { return sdadc_raw_signed(result_raw()); }

    /// Start, wait for RESRDY, read. Bounded: a converter that never
    /// answers returns false and leaves `out` alone.
    static bool read(int16_t& out, uint32_t spins = 0xFFFFFFu) {
        clear_flags(flag_resrdy | flag_overrun);
        if (!start()) {
            return false;
        }
        while (spins-- != 0u) {
            if (ready()) {
                out = result();
                return true;
            }
        }
        return false;
    }
    /// The convenience form, for a converter known to be running.
    static int16_t read(uint32_t spins = 0xFFFFFFu) {
        int16_t v = 0;
        (void)read(v, spins);
        return v;
    }

    /// Wait for the next result of a FREE-RUNNING converter and read it.
    static bool next(int16_t& out, uint32_t spins = 0xFFFFFFu) {
        clear_flags(flag_resrdy);
        while (spins-- != 0u) {
            if (ready()) {
                out = result();
                return true;
            }
        }
        return false;
    }

    /// Spend `count` conversions and throw them away - a caller's own
    /// warm-up, on top of the SKPCNT the silicon spends.
    static void discard(uint8_t count, uint32_t spins = 0xFFFFFFu) {
        for (uint8_t i = 0; i < count; ++i) {
            int16_t v = 0;
            (void)read(v, spins);
        }
    }

    // ---- free running -------------------------------------------------------

    /// CTRLC.FREERUN under a running converter. Write-synchronized; in
    /// 39.6.2.1's enable-protected LIST but not in 39.8.10's property
    /// line, so this refuses while enabled - correct under either
    /// reading, and the suite measures which one the silicon means.
    static bool free_running(bool on, uint32_t spins = 0xFFFFu) {
        if (enabled()) {
            return false;
        }
        if (!write_sync(regs().SDADC_CTRLC, SDADC_SYNCBUSY_CTRLC_Msk,
                        static_cast<uint8_t>(on ? SDADC_CTRLC_FREERUN_Msk : 0u),
                        spins)) {
            return false;
        }
        cfg_.free_running = on;
        return true;
    }
    static bool free_running() {
        return (regs().SDADC_CTRLC & SDADC_CTRLC_FREERUN_Msk) != 0u;
    }

    // ---- the window monitor (39.6.2.8) -------------------------------------

    /**
     * WINCTRL with its two thresholds, under a running converter. All
     * three registers are write-synchronized; the thresholds are placed
     * the way RESULT is (`sdadc_threshold_word()`) so a caller compares
     * against the same numbers it reads.
     */
    static bool window(SdadcWindow mode, int16_t low, int16_t high,
                       uint32_t spins = 0xFFFFu) {
        if (!sdadc_window_valid(mode)) {
            return false;
        }
        if (!write_sync(regs().SDADC_WINLT, SDADC_SYNCBUSY_WINLT_Msk,
                        sdadc_threshold_word(low), spins) ||
            !write_sync(regs().SDADC_WINUT, SDADC_SYNCBUSY_WINUT_Msk,
                        sdadc_threshold_word(high), spins)) {
            return false;
        }
        if (!write_sync(regs().SDADC_WINCTRL, SDADC_SYNCBUSY_WINCTRL_Msk,
                        static_cast<uint8_t>(
                            SDADC_WINCTRL_WINMODE(static_cast<uint8_t>(mode))),
                        spins)) {
            return false;
        }
        cfg_.window = mode;
        cfg_.window_low = low;
        cfg_.window_high = high;
        return true;
    }
    /// The same with the thresholds given in RAW 24-bit units, which is
    /// the width the registers really are - for a caller working in
    /// `result24()`s rather than in the specified sixteen bits.
    static bool window_raw(SdadcWindow mode, int32_t low, int32_t high,
                           uint32_t spins = 0xFFFFu) {
        if (!sdadc_window_valid(mode) || low > 8388607 || low < -8388608 ||
            high > 8388607 || high < -8388608) {
            return false;
        }
        if (!write_sync(regs().SDADC_WINLT, SDADC_SYNCBUSY_WINLT_Msk,
                        static_cast<uint32_t>(low) & 0x00FFFFFFu, spins) ||
            !write_sync(regs().SDADC_WINUT, SDADC_SYNCBUSY_WINUT_Msk,
                        static_cast<uint32_t>(high) & 0x00FFFFFFu, spins)) {
            return false;
        }
        if (!write_sync(regs().SDADC_WINCTRL, SDADC_SYNCBUSY_WINCTRL_Msk,
                        static_cast<uint8_t>(
                            SDADC_WINCTRL_WINMODE(static_cast<uint8_t>(mode))),
                        spins)) {
            return false;
        }
        cfg_.window = mode;
        return true;
    }
    static bool window_off(uint32_t spins = 0xFFFFu) {
        return window(SdadcWindow::none, cfg_.window_low, cfg_.window_high, spins);
    }
    static SdadcWindow window_mode() {
        return static_cast<SdadcWindow>(regs().SDADC_WINCTRL &
                                        SDADC_WINCTRL_WINMODE_Msk);
    }
    /// Did the LAST value read by `result()`/`read()` match the window?
    /// (The hardware flag is cleared by the RESULT read itself.)
    static bool window_hit() { return last_hit_; }
    /// The live flag, before any RESULT read.
    static bool window_flag() { return (regs().SDADC_INTFLAG & flag_winmon) != 0u; }

    // ---- the post-processing (39.6.3.4) -------------------------------------

    /// OFFSETCORR: a SIGNED 24-bit value ADDED to the filter's output
    /// before the gain multiply, IN RAW 24-BIT UNITS - so 256 of these
    /// is one count of `result()` (`sdadc_raw_per_count`), measured.
    static bool offset_correction(int32_t value, uint32_t spins = 0xFFFFu) {
        if (value > 8388607 || value < -8388608) {
            return false;
        }
        if (!write_sync(regs().SDADC_OFFSETCORR, SDADC_SYNCBUSY_OFFSETCORR_Msk,
                        static_cast<uint32_t>(value) & 0x00FFFFFFu, spins)) {
            return false;
        }
        cfg_.offset_correction = value;
        return true;
    }
    static int32_t offset_correction() {
        const uint32_t raw = regs().SDADC_OFFSETCORR & 0x00FFFFFFu;
        return static_cast<int32_t>(raw & 0x800000u ? raw | 0xFF000000u : raw);
    }

    /// GAINCORR: an UNSIGNED 14-bit INTEGER, unity ONE. Zero would
    /// multiply every result away and is refused.
    static bool gain_correction(uint16_t value, uint32_t spins = 0xFFFFu) {
        if (value == 0u || value > 0x3FFFu) {
            return false;
        }
        if (!write_sync(regs().SDADC_GAINCORR, SDADC_SYNCBUSY_GAINCORR_Msk,
                        value, spins)) {
            return false;
        }
        cfg_.gain_correction = value;
        return true;
    }
    static uint16_t gain_correction() {
        return static_cast<uint16_t>(regs().SDADC_GAINCORR & 0x3FFFu);
    }

    /// SHIFTCORR: the right shift applied after the multiply.
    static bool shift_correction(uint8_t value, uint32_t spins = 0xFFFFu) {
        if (value > 15u) {
            return false;
        }
        if (!write_sync(regs().SDADC_SHIFTCORR, SDADC_SYNCBUSY_SHIFTCORR_Msk,
                        value, spins)) {
            return false;
        }
        cfg_.shift_correction = value;
        return true;
    }
    static uint8_t shift_correction() {
        return static_cast<uint8_t>(regs().SDADC_SHIFTCORR & 0xFu);
    }

    // ---- the automatic sequence (39.6.2.7) ---------------------------------

    /// One bit per differential pair; the sequence walks them from the
    /// lowest. Zero disables it and the conversion uses MUXSEL. Neither
    /// enable-protected nor synchronized.
    static bool sequence(uint8_t mask) {
        if (mask > 0x7u) {
            return false;
        }
        for (uint8_t p = 0; p < 3u; ++p) {
            if ((mask & (1u << p)) != 0u && !pair_exists(p)) {
                return false;
            }
        }
        regs().SDADC_SEQCTRL = static_cast<uint8_t>(SDADC_SEQCTRL_SEQEN(mask));
        cfg_.sequence = mask;
        return true;
    }
    static uint8_t sequence() {
        return static_cast<uint8_t>(regs().SDADC_SEQCTRL & SDADC_SEQCTRL_SEQEN_Msk);
    }

    /// SEQSTATUS.SEQBUSY: a sequence is in progress.
    static bool sequence_busy() {
        return (regs().SDADC_SEQSTATUS & SDADC_SEQSTATUS_SEQBUSY_Msk) != 0u;
    }
    /// SEQSTATUS.SEQSTATE: the input the LAST completed conversion of the
    /// sequence came from.
    static uint8_t sequence_state() {
        return static_cast<uint8_t>(regs().SDADC_SEQSTATUS &
                                    SDADC_SEQSTATUS_SEQSTATE_Msk);
    }

    // ---- the analog control (39.8.21) --------------------------------------

    /// ANACTRL under a disabled converter - the chopper, the bias field
    /// whose width the two documents disagree about, and the bit the
    /// register description leaves blank.
    static bool analog_control(const SdadcConfig& c, uint32_t spins = 0xFFFFu) {
        if (enabled() || c.bias_control > 0x3Fu) {
            return false;
        }
        if (!write_sync(regs().SDADC_ANACTRL, SDADC_SYNCBUSY_ANACTRL_Msk,
                        analog_word(c), spins)) {
            return false;
        }
        cfg_.chopper = c.chopper;
        cfg_.bias_control = c.bias_control;
        cfg_.buffer_test = c.buffer_test;
        return true;
    }
    static uint8_t analog_control() { return regs().SDADC_ANACTRL; }
    static bool chopper() { return (regs().SDADC_ANACTRL & SDADC_ANACTRL_ONCHOP_Msk) != 0u; }

    // ---- the reference ------------------------------------------------------

    /// REFCTRL under a disabled converter (39.8.2 marks it
    /// enable-protected, and it has no SYNCBUSY bit whatever 39.6.8's
    /// prose says).
    static bool reference(const SdadcConfig& c) {
        if (enabled() || !sdadc_config_valid(c)) {
            return false;
        }
        regs().SDADC_REFCTRL = reference_word(c);
        cfg_.reference = c.reference;
        cfg_.reference_buffer = c.reference_buffer;
        cfg_.reference_range = c.reference_range;
        return true;
    }
    static uint8_t reference_register() { return regs().SDADC_REFCTRL; }
    static bool reference_buffer() {
        return (regs().SDADC_REFCTRL & SDADC_REFCTRL_ONREFBUF_Msk) != 0u;
    }

    // ---- interrupts ----------------------------------------------------------

    static void arm(uint8_t mask) { regs().SDADC_INTENSET = mask; }
    static void disarm(uint8_t mask) { regs().SDADC_INTENCLR = mask; }
    static uint8_t armed() { return regs().SDADC_INTENSET; }
    static uint8_t flags() { return regs().SDADC_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().SDADC_INTFLAG = mask; }

    /**
     * The ISR body - ONE VECTOR for all three sources, so the app binds
     * SDADC_Handler once and dispatches on the returned mask.
     *
     * OVERRUN is cleared here; RESRDY and WINMON are NOT, because reading
     * RESULT is what clears them and the value is the point. A handler
     * that returns without reading RESULT will be called again.
     */
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t pending =
            static_cast<uint8_t>(regs().SDADC_INTFLAG & regs().SDADC_INTENSET);
        if ((pending & flag_overrun) != 0u) {
            regs().SDADC_INTFLAG = flag_overrun;
        }
        return pending;
    }

    /// The RESRDY half of the ISR body, for glue that only wants the
    /// value: the result, with the window verdict captured first.
    [[gnu::always_inline]] static int16_t resrdy() { return result(); }

    // ---- events ---------------------------------------------------------------

    /// EVCTRL is ENABLE-PROTECTED (39.8.4), so this refuses while the
    /// converter is enabled rather than storing into a dead register.
    static bool event_config(const SdadcEventControl& e) {
        if (enabled() || !sdadc_event_control_valid(e)) {
            return false;
        }
        regs().SDADC_EVCTRL = event_word(e);
        cfg_.events = e;
        return true;
    }
    static SdadcEventControl event_config() {
        const uint8_t v = regs().SDADC_EVCTRL;
        return SdadcEventControl{
            .flush_in = (v & SDADC_EVCTRL_FLUSHEI_Msk) != 0u,
            .start_in = (v & SDADC_EVCTRL_STARTEI_Msk) != 0u,
            .invert_flush = (v & SDADC_EVCTRL_FLUSHINV_Msk) != 0u,
            .invert_start = (v & SDADC_EVCTRL_STARTINV_Msk) != 0u,
            .result_out = (v & SDADC_EVCTRL_RESRDYEO_Msk) != 0u,
            .window_out = (v & SDADC_EVCTRL_WINMONEO_Msk) != 0u,
        };
    }

    /**
     * Route an EVSYS channel to the START user and turn EVCTRL.STARTEI on
     * - the two halves that must both happen, in one verb so neither can
     * be forgotten.
     *
     * IT REFUSES A CHANNEL THAT IS NOT ASYNCHRONOUS: 39.6.6 says in so
     * many words that "the SDADC uses only asynchronous events.
     * Therefore, an asynchronous Event System channel path must be
     * configured", and table 29-3 marks both users the same way.
     */
    static bool start_on(uint8_t channel, const EventChannelConfig& cfg,
                         bool invert = false) {
        if (cfg.path != EventPath::asynchronous || enabled()) {
            return false;
        }
        SdadcEventControl e = event_config();
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
        SdadcEventControl e = event_config();
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
        SdadcEventControl e = event_config();
        e.start_in = false;
        e.flush_in = false;
        e.invert_start = false;
        e.invert_flush = false;
        return event_config(e);
    }

private:
    /// The one place a synchronized write happens: WAIT, then STORE.
    /// 39.6.8's bus error is why the wait is on this side of the store.
    template <typename R, typename V>
    static bool write_sync(volatile R& reg, uint32_t busy, V value,
                           uint32_t spins) {
        if (!sync_wait(busy, spins)) {
            return false;
        }
        reg = static_cast<R>(value);
        return true;
    }

    static constexpr uint8_t reference_word(const SdadcConfig& c) {
        return static_cast<uint8_t>(
            SDADC_REFCTRL_REFSEL(static_cast<uint8_t>(c.reference)) |
            SDADC_REFCTRL_REFRANGE(c.reference_range) |
            (c.reference_buffer ? SDADC_REFCTRL_ONREFBUF_Msk : 0u));
    }

    static constexpr uint16_t control_b_word(const SdadcConfig& c) {
        return static_cast<uint16_t>(
            SDADC_CTRLB_PRESCALER(c.prescaler) |
            SDADC_CTRLB_OSR(static_cast<uint16_t>(c.osr)) |
            SDADC_CTRLB_SKPCNT(c.skip_count));
    }

    static constexpr uint8_t event_word(const SdadcEventControl& e) {
        return static_cast<uint8_t>(
            (e.flush_in ? SDADC_EVCTRL_FLUSHEI_Msk : 0u) |
            (e.start_in ? SDADC_EVCTRL_STARTEI_Msk : 0u) |
            (e.invert_flush ? SDADC_EVCTRL_FLUSHINV_Msk : 0u) |
            (e.invert_start ? SDADC_EVCTRL_STARTINV_Msk : 0u) |
            (e.result_out ? SDADC_EVCTRL_RESRDYEO_Msk : 0u) |
            (e.window_out ? SDADC_EVCTRL_WINMONEO_Msk : 0u));
    }

    static constexpr uint8_t analog_word(const SdadcConfig& c) {
        return static_cast<uint8_t>(
            SDADC_ANACTRL_CTRSDADC(c.bias_control) |
            (c.chopper ? SDADC_ANACTRL_ONCHOP_Msk : 0u) |
            (c.buffer_test ? SDADC_ANACTRL_BUFTEST_Msk : 0u));
    }

    static inline SdadcConfig cfg_{};
    static inline bool last_hit_ = false;
};

} // namespace brio
