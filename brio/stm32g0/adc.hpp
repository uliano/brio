/*
 * adc.hpp
 *
 * The STM32G0's analog-to-digital converter (RM0444 ch. 15) in this
 * stratum's two faces:
 *
 *   Adc              the RESOURCE - one 12-bit SAR converter, and there
 *                    is exactly one on every part of the family, so it
 *                    is a MONOSTATE (the samc Dac/Sdadc/Tsens precedent)
 *                    and not an Adc<n>.
 *   AnalogIn<Pin,ch> a pad handed to a channel. The channel NUMBER is
 *                    the datasheet's (DS13560 table 12's "additional
 *                    functions" column) and no device header of this
 *                    pack carries it, so the CALLER states it - exactly
 *                    as stm32g0/tim.hpp's TimPad takes the AF number.
 *
 * FIVE FACTS THAT SHAPE THIS FILE.
 *
 * 1. IT HAS A REGULATOR AND A CALIBRATION, AND BOTH ARE PROCEDURES.
 *    ADVREGEN must be raised and tADCVREG_STUP (DS13560 table 62: 20 us)
 *    spent before anything else (15.3.2); ADCAL then runs a self
 *    calibration that removes an offset which varies part to part
 *    (15.3.3) and costs 82 ADC cycles. So `init()` takes the CLOCK - it
 *    has real microseconds to spend - and calibration is not optional
 *    here: the errata's own 2.6.5 is about what is left AFTER it.
 *    Neither is a factory value copied into a register, which is where
 *    this converter differs from both earlier targets.
 *
 * 2. THE FACTORY VALUES ARE ELSEWHERE, AND THEY ARE MEASUREMENTS, NOT
 *    TRIMS. VREFINT_CAL at 0x1FFF75AA and TS_CAL1/TS_CAL2 at 0x1FFF75A8
 *    / 0x1FFF75CA (DS13560 tables 5 and 6) are ADC RESULTS taken at
 *    VDDA = VREF+ = 3.0 V - the first at 30 C, the last two at 30 C and
 *    130 C. Nothing is written back anywhere: they are the arithmetic's
 *    input, which is why `vdda_mv()` and `temperature_c()` live here and
 *    `AdcFactory` is a read-only view.
 *
 * 3. THE SEQUENCER HAS TWO FACES AND A HANDSHAKE. CHSELR is a bitmap of
 *    nineteen channels scanned in numeric order (CHSELRMOD = 0), or
 *    eight 4-bit slots scanned in the order written, channels 0..14 only
 *    (CHSELRMOD = 1) - 15.3.8. EITHER WAY the write is not in force
 *    until ISR.CCRDY rises, and 15.12.5's own note says an ADSTART
 *    written before that is IGNORED. So every channel-selection verb in
 *    this file clears CCRDY, writes, and waits - `select()` bounded and
 *    silent (util/analog_sampler.hpp's contract wants a void), and
 *    `select_sync()` with an answer.
 *
 * 4. THE CLOCK IS TWO CHOICES, NOT ONE. CKMODE picks between the APB
 *    clock divided by 1, 2 or 4 - deterministic trigger latency,
 *    15.3.5's table 74 - and an ASYNCHRONOUS root selected in
 *    RCC_CCIPR.ADCSEL and then divided by ADC_CCR.PRESC, which reaches
 *    the top rate whatever the bus is doing but adds jitter to a
 *    hardware trigger. The 45..55 % duty rule of 15.3.5 makes PCLK/1
 *    legal only with the AHB and APB prescalers in bypass, which is what
 *    `clock_ok()` asks the silicon. fADC has a ceiling of 35 MHz in
 *    voltage range 1 (DS13560 table 62) and `adc_config_valid()` cannot
 *    know it, so `init()` - which does know the clock - is where the
 *    rate is judged.
 *
 * 5. THE OVERSAMPLER CHANGES THE FULL SCALE, AND THE FULL SCALE IS
 *    util/analog.hpp's `steps`. Up to 256 accumulated conversions shift
 *    right by up to 8 and are truncated to 16 bits (15.8's table 79), so
 *    `result_steps()` is what adc_mv() must be given, and it is computed
 *    from the config rather than assumed. Alignment is IGNORED while
 *    oversampling (15.8.1's note) and the watchdogs then compare the top
 *    12 bits of the 16 (15.8.2) - both stated on the verbs that care.
 *
 * THE REFERENCE is stm32g0/vref.hpp's `Ref`, not this file's: on this
 * family VREF+ is one rail shared by the ADC, the DAC and the
 * comparators, so the enum lives with the rail (that file's header says
 * why, against the samc's opposite ruling).
 *
 * ERRATA, ES0548 Rev 3 on the bench chip's revision Z column:
 *  - 2.6.1 OVR may stay low when an EOC clear coincides with a
 *    conversion end. Its workaround is a TIMING obligation on the
 *    reader, which no driver can enforce; stated on `overrun()`.
 *  - 2.6.2 a write to ADC_CFGR1 with ADEN set resets RES to 12 bits.
 *    ANSWERED STRUCTURALLY: `configure()` is a disabled-state verb (the
 *    chapter's own 15.3.7 rule) and there is no other writer of CFGR1
 *    here, so the combination cannot be spelled.
 *  - 2.6.3 AWD1 in single-channel mode misses a channel that is not the
 *    first of the sequence. `watchdog1()` states the obligation and
 *    cannot enforce it (it does not own the sequence).
 *  - 2.6.4 a 1.5 or 3.5 cycle sampling time takes one cycle longer on a
 *    single conversion or the first of a sequence. No workaround; the
 *    prediction in `conversion_cycles()` is DELIBERATELY the chapter's,
 *    with the extra cycle named where it is measured.
 *  - 2.6.5 is revision A only and does not apply to this silicon.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/vref.hpp"
#include "util/analog.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/// ADC_CFGR2.CKMODE (15.3.5). `async` takes RCC_CCIPR.ADCSEL's root
/// through PRESC; the three others divide PCLK and give the
/// deterministic trigger latency of table 74.
enum class AdcClockMode : uint8_t {
    async = 0,
    pclk_div2 = 1,
    pclk_div4 = 2,
    pclk_div1 = 3,
};

/// RCC_CCIPR.ADCSEL - which root the asynchronous clock comes from
/// (5.4.13). PLLPCLK needs the PLL's P output enabled, which this
/// stratum's `Clock<>` does not configure; `init()` refuses it rather
/// than clocking the converter from a dead branch.
enum class AdcAsyncSource : uint8_t { sysclk = 0, pllp = 1, hsi16 = 2 };

/// ADC_CCR.PRESC - the asynchronous clock's divider (15.12.10). Twelve
/// implemented codes; everything above is Reserved.
enum class AdcPresc : uint8_t {
    div1 = 0, div2 = 1, div4 = 2, div6 = 3, div8 = 4, div10 = 5,
    div12 = 6, div16 = 7, div32 = 8, div64 = 9, div128 = 10, div256 = 11,
};

constexpr uint16_t adc_presc_divisor(AdcPresc p) {
    switch (p) {
        case AdcPresc::div1: return 1;
        case AdcPresc::div2: return 2;
        case AdcPresc::div4: return 4;
        case AdcPresc::div6: return 6;
        case AdcPresc::div8: return 8;
        case AdcPresc::div10: return 10;
        case AdcPresc::div12: return 12;
        case AdcPresc::div16: return 16;
        case AdcPresc::div32: return 32;
        case AdcPresc::div64: return 64;
        case AdcPresc::div128: return 128;
        case AdcPresc::div256: return 256;
        default: return 0;   // Reserved
    }
}

/// ADC_CFGR1.RES (15.4.2). The DATA is always 12 bits wide with the
/// unused low bits read as zero; what changes is tSAR and the number of
/// distinct results.
enum class AdcRes : uint8_t { bits12 = 0, bits10 = 1, bits8 = 2, bits6 = 3 };

/// The full scale of one conversion at this resolution - util/analog.hpp's
/// `steps` before any oversampling.
constexpr uint32_t adc_sample_steps(AdcRes r) {
    switch (r) {
        case AdcRes::bits12: return 4096;
        case AdcRes::bits10: return 1024;
        case AdcRes::bits8: return 256;
        case AdcRes::bits6: return 64;
        default: return 0;
    }
}

/// tSAR in HALF ADC cycles (table 76: 12.5, 10.5, 8.5, 6.5), kept in
/// halves so the arithmetic below never rounds a half away.
constexpr uint16_t adc_sar_half_cycles(AdcRes r) {
    switch (r) {
        case AdcRes::bits12: return 25;
        case AdcRes::bits10: return 21;
        case AdcRes::bits8: return 17;
        case AdcRes::bits6: return 13;
        default: return 0;
    }
}

/// ADC_SMPR's two sampling-time fields (15.3.9). Every channel picks one
/// of the two through SMPSEL, which is what makes a slow internal source
/// and a fast pad share one sequence.
enum class AdcSampleTime : uint8_t {
    cycles1_5 = 0, cycles3_5 = 1, cycles7_5 = 2, cycles12_5 = 3,
    cycles19_5 = 4, cycles39_5 = 5, cycles79_5 = 6, cycles160_5 = 7,
};

/// tSMPL in HALF ADC cycles.
constexpr uint16_t adc_sample_half_cycles(AdcSampleTime s) {
    switch (s) {
        case AdcSampleTime::cycles1_5: return 3;
        case AdcSampleTime::cycles3_5: return 7;
        case AdcSampleTime::cycles7_5: return 15;
        case AdcSampleTime::cycles12_5: return 25;
        case AdcSampleTime::cycles19_5: return 39;
        case AdcSampleTime::cycles39_5: return 79;
        case AdcSampleTime::cycles79_5: return 159;
        case AdcSampleTime::cycles160_5: return 321;
        default: return 0;
    }
}

/// ADC_CFGR1.EXTSEL - table 73's eight hardware triggers. Every one of
/// them but the last is a TIMER's TRGO or capture/compare output, which
/// is what makes a no-CPU conversion chain on this family a timer
/// question and not an event-system one.
enum class AdcTrigger : uint8_t {
    tim1_trgo2 = 0, tim1_cc4 = 1, tim2_trgo = 2, tim3_trgo = 3,
    tim15_trgo = 4, tim6_trgo = 5, tim4_trgo = 6, exti11 = 7,
};

/// ADC_CFGR1.EXTEN - table 75.
enum class AdcEdge : uint8_t { none = 0, rising = 1, falling = 2, both = 3 };

/// ADC_CFGR1.SCANDIR - only meaningful with CHSELRMOD = 0 (15.3.8).
enum class AdcScanDir : uint8_t { forward = 0, backward = 1 };

/// ADC_CFGR2.OVSR - the number of conversions accumulated (15.8).
enum class AdcOversampling : uint8_t {
    x2 = 0, x4 = 1, x8 = 2, x16 = 3, x32 = 4, x64 = 5, x128 = 6, x256 = 7,
};

constexpr uint16_t adc_oversampling_ratio(AdcOversampling r) {
    return static_cast<uint16_t>(2u << static_cast<uint8_t>(r));
}

/// The three channels that are not pads (15.3.8), with the numbers
/// stm32g0/device_tables.hpp reads off the ADC_CCR enable bits.
enum class AdcInput : uint8_t {
    temperature = 12,
    vrefint = 13,
    vbat = 14,
};

constexpr bool adc_input_valid(AdcInput in) {
    switch (in) {
        case AdcInput::temperature: return adc_temperature_channel() != 0xFF;
        case AdcInput::vrefint: return adc_vrefint_channel() != 0xFF;
        case AdcInput::vbat: return adc_vbat_channel() != 0xFF;
        default: return false;
    }
}

/// ADC_ISR/ADC_IER share one bit layout, so a flag mask is an interrupt
/// mask (15.12.1, 15.12.2) and one set of constants serves both.
struct AdcFlag {
    static constexpr uint32_t ready = ADC_ISR_ADRDY;
    static constexpr uint32_t sampled = ADC_ISR_EOSMP;
    static constexpr uint32_t converted = ADC_ISR_EOC;
    static constexpr uint32_t sequence_done = ADC_ISR_EOS;
    static constexpr uint32_t overrun = ADC_ISR_OVR;
    static constexpr uint32_t watchdog1 = ADC_ISR_AWD1;
    static constexpr uint32_t watchdog2 = ADC_ISR_AWD2;
    static constexpr uint32_t watchdog3 = ADC_ISR_AWD3;
    static constexpr uint32_t calibrated = ADC_ISR_EOCAL;
    static constexpr uint32_t channels_ready = ADC_ISR_CCRDY;
};

/// What `Adc::configure()` is given. The defaults are the arrangement a
/// caller who wants "one 12-bit conversion when I ask for it" would
/// write out by hand.
struct AdcConfig {
    AdcClockMode clock_mode = AdcClockMode::pclk_div2;
    AdcAsyncSource async_source = AdcAsyncSource::hsi16;
    AdcPresc prescaler = AdcPresc::div1;

    AdcRes resolution = AdcRes::bits12;
    /// ADC_CFGR1.ALIGN. IGNORED by the silicon while oversampling
    /// (15.8.1's note), and `adc_config_valid()` refuses the pair rather
    /// than letting a caller believe in it.
    bool left_aligned = false;

    /// The two sampling times, and which channels take the second one:
    /// bit k of `sample2_channels` is ADC_SMPR.SMPSELk (15.3.9).
    AdcSampleTime sample1 = AdcSampleTime::cycles1_5;
    AdcSampleTime sample2 = AdcSampleTime::cycles160_5;
    uint32_t sample2_channels = 0;

    /// CHSELRMOD: false = the bitmap scanned in numeric order,
    /// true = the eight ordered slots (channels 0..14 only).
    bool ordered_sequence = false;
    AdcScanDir scan = AdcScanDir::forward;

    bool continuous = false;     ///< CONT
    bool discontinuous = false;  ///< DISCEN - 15.4.1 forbids it with CONT
    bool wait = false;           ///< WAIT: pace the ADC by the reader (15.6.1)
    bool auto_off = false;       ///< AUTOFF: power down between sequences (15.6.2)
    /// OVRMOD: true = the newest result wins, false = an unread one is
    /// kept and the new conversion discarded (15.5.2).
    bool overrun_overwrite = false;

    bool dma = false;            ///< DMAEN
    bool dma_circular = false;   ///< DMACFG: 15.5.5's circular mode

    AdcTrigger trigger = AdcTrigger::tim1_trgo2;
    AdcEdge trigger_edge = AdcEdge::none;
    /// CFGR2.LFTRIG - for trigger rates below the figure DS13560 gives
    /// (15.4.6); costs nothing when it is not needed.
    bool low_frequency_trigger = false;

    bool oversampling = false;
    AdcOversampling oversampling_ratio = AdcOversampling::x2;
    uint8_t oversampling_shift = 0;    ///< OVSS, 0..8
    /// TOVS: one trigger per accumulated conversion instead of one per
    /// oversampled result (15.8.3). It forces DISCEN, which is why
    /// `adc_config_valid()` does not also demand it.
    bool triggered_oversampling = false;
};

/// Everything the chapter refuses, and nothing the chapter does not.
constexpr bool adc_config_valid(const AdcConfig& c) {
    if (adc_presc_divisor(c.prescaler) == 0u) {
        return false;   // a Reserved PRESC code
    }
    if (adc_sample_steps(c.resolution) == 0u) {
        return false;
    }
    if (c.continuous && c.discontinuous) {
        return false;   // 15.4.1's own note
    }
    if (c.oversampling && c.oversampling_shift > 8u) {
        return false;   // OVSS is four bits, 15.8 uses nine of the codes
    }
    if (c.oversampling && c.left_aligned) {
        return false;   // 15.8.1: ALIGN is ignored - so it is not offered
    }
    if (c.triggered_oversampling && !c.oversampling) {
        return false;
    }
    if (c.dma_circular && !c.dma) {
        return false;
    }
    if ((c.sample2_channels >> adc_channels()) != 0u) {
        return false;   // an SMPSEL bit past the channel count
    }
    return true;
}

/// The full scale of what lands in ADC_DR - util/analog.hpp's `steps`.
/// Without the oversampler it is the resolution's; with it, table 79's
/// accumulate-then-shift, truncated to the sixteen bits the register
/// holds.
constexpr uint32_t adc_result_steps(const AdcConfig& c) {
    const uint32_t base = adc_sample_steps(c.resolution);
    if (!c.oversampling) {
        return base;
    }
    const uint32_t raw = base * adc_oversampling_ratio(c.oversampling_ratio);
    const uint32_t shifted = raw >> c.oversampling_shift;
    return shifted > 65536UL ? 65536UL : shifted;
}

/// The sampling time in effect for channel `ch` under this config.
constexpr AdcSampleTime adc_sample_time_of(const AdcConfig& c, uint8_t ch) {
    return ((c.sample2_channels >> ch) & 1u) != 0u ? c.sample2 : c.sample1;
}

/// tCONV in HALF ADC cycles for one conversion of channel `ch`
/// (15.3.9: tSMPL + tSAR). ES0548 2.6.4 adds ONE WHOLE CYCLE to a single
/// conversion, or to the first of a sequence, when the sampling time is
/// 1.5 or 3.5 cycles - deliberately NOT folded in, because the chapter's
/// number is what a caller is predicting against and the erratum is
/// measured beside it (test_stm32_analog letter k).
constexpr uint32_t adc_conversion_half_cycles(const AdcConfig& c, uint8_t ch) {
    return static_cast<uint32_t>(adc_sample_half_cycles(adc_sample_time_of(c, ch))) +
           adc_sar_half_cycles(c.resolution);
}

/// The ADC clock this config would run at, given the bus rate. Zero for
/// a Reserved prescaler.
constexpr uint32_t adc_clock_hz(uint32_t pclk_hz, uint32_t async_hz, const AdcConfig& c) {
    switch (c.clock_mode) {
        case AdcClockMode::pclk_div1: return pclk_hz;
        case AdcClockMode::pclk_div2: return pclk_hz / 2u;
        case AdcClockMode::pclk_div4: return pclk_hz / 4u;
        case AdcClockMode::async: {
            const uint16_t d = adc_presc_divisor(c.prescaler);
            return d == 0u ? 0u : async_hz / d;
        }
        default: return 0;
    }
}

/// DS13560 table 62's ceiling, voltage range 1 (which is the range
/// stm32g0/clock.hpp's Clock<> requires anyway). Range 2 halves it to
/// 16 MHz and this stratum never enters it.
constexpr uint32_t adc_max_hz = 35'000'000UL;

// =============================================================================
// The factory calibration (DS13560 tables 5 and 6)
// =============================================================================

/**
 * The three numbers ST measured on this die and left in the system
 * memory. They are ADC RESULTS at VDDA = VREF+ = 3.0 V, not register
 * trims: nothing is written back anywhere, and `Adc::vdda_mv()` /
 * `Adc::temperature_c()` are the whole of their purpose.
 *
 * The addresses are the DATASHEET'S and no header of this pack declares
 * them, which is why they are spelled here - the peripheral that uses a
 * number owns it, the same rule that keeps the DMAMUX request ids in
 * their peripherals.
 */
struct AdcFactory {
    /// The conditions ST used, so the arithmetic never hides them.
    static constexpr uint16_t characterization_mv = 3000;
    static constexpr int16_t ts_cal1_celsius = 30;
    static constexpr int16_t ts_cal2_celsius = 130;

    static uint16_t vrefint_cal() { return read(0x1FFF75AAUL); }
    static uint16_t ts_cal1() { return read(0x1FFF75A8UL); }
    static uint16_t ts_cal2() { return read(0x1FFF75CAUL); }

    /// A blank or erased engineering byte pair reads 0xFFFF (and a zero
    /// would divide by nothing), so every consumer asks first.
    static bool plausible() {
        const uint16_t v = vrefint_cal();
        const uint16_t a = ts_cal1();
        const uint16_t b = ts_cal2();
        return v != 0u && v != 0xFFFFu && a != 0u && a != 0xFFFFu && b != 0u &&
               b != 0xFFFFu && a != b;
    }

private:
    static uint16_t read(uint32_t address) {
        return *reinterpret_cast<const volatile uint16_t*>(address);
    }
};

// =============================================================================
// A pad on a channel
// =============================================================================

/**
 * AnalogIn<Pin, channel>: the claim that this pad is that ADC channel.
 *
 * WHICH channel a pad is comes from DS13560 table 12 and NOTHING CHECKS
 * IT - the device header carries no analog pin table (stm32g0/pin.hpp
 * states that once for the stratum, and stm32g0/tim.hpp's TimPad has the
 * same shape for AF numbers). So the caller writes the number, the bench
 * is the check, and a wrong one reads a different pad rather than
 * failing to compile.
 *
 * `claim()` is analog mode, which is also the pad's reset state and the
 * one mode where 7.3.1 turns the input buffer OFF - so a claimed analog
 * pad reads zero on IDR, and that is not a fault.
 */
template <class P, uint8_t channel_>
struct AnalogIn {
    static_assert(channel_ < adc_channels(),
                  "brio AnalogIn: no such ADC channel on this device");

    using pin = P;
    static constexpr uint8_t channel = channel_;

    static void claim() { P::analog(); }
    static void release() { P::analog(); }
};

// =============================================================================
// The resource
// =============================================================================

class Adc {
public:
    static_assert(adc_present(), "brio Adc: this device declares no ADC1_BASE");

    Adc() = delete;

    static constexpr uint8_t channels = adc_channels();
    static constexpr uint8_t temperature_channel = adc_temperature_channel();
    static constexpr uint8_t vrefint_channel = adc_vrefint_channel();
    static constexpr uint8_t vbat_channel = adc_vbat_channel();

    /// The NVIC line, SHARED WITH THE COMPARATORS where they exist
    /// (table 61) - so a handler bound here is a dispatcher.
    static constexpr IRQn_Type irq() { return adc_irq(); }

    static ADC_TypeDef& regs() { return *reinterpret_cast<ADC_TypeDef*>(adc_base()); }

    /// ADC_CCR - the asynchronous prescaler and the three internal-source
    /// enables. The device header puts it in its OWN structure at its own
    /// base (ADC_Common_TypeDef), because the block is shared with
    /// converters this family does not have.
    static ADC_Common_TypeDef& common() {
        return *reinterpret_cast<ADC_Common_TypeDef*>(adc_common_base());
    }

    /**
     * The DMAMUX request id of the converter (table 55 row 5), for a
     * stm32g0/dma.hpp engine's arm(). It lives here and not in the
     * reserve for the reason this stratum settled with the timers: no
     * device header of this pack declares one of these numbers, and a
     * fabric driver owns the fabric while a peripheral owns its own
     * vocabulary.
     *
     * CFGR1.DMAEN is what makes the converter ASSERT it; this is what
     * makes a channel listen. 15.5.5: DMAEN must be set AFTER the
     * calibration, which is why `configure()` is a post-`calibrate()`
     * verb in `init()`'s own order.
     */
    static constexpr uint8_t dma_request = 5;

    /// Where a DMA channel reads a result from.
    static volatile void* data_address() { return &regs().DR; }

    // ---- the bus clock and the kernel clock ----------------------------------

    static void bus_clock(bool on) { Rcc::apb2_clock(adc_clock_mask(), on); }
    static bool bus_clock() { return Rcc::apb2_clock(adc_clock_mask()); }

    static void reset() {
        Rcc::apb2_reset(adc_reset_mask());
    }

    /// RCC_CCIPR.ADCSEL, the asynchronous root. Meaningless with
    /// CKMODE != async, and written by `init()` only when it is not.
    static void async_source(AdcAsyncSource s) {
        Rcc::kernel_clock(RCC_CCIPR_ADCSEL_Pos, static_cast<uint8_t>(s));
    }
    static AdcAsyncSource async_source() {
        return static_cast<AdcAsyncSource>(Rcc::kernel_clock(RCC_CCIPR_ADCSEL_Pos));
    }

    /// 15.3.5's caution: PCLK/1 needs the AHB and APB prescalers in
    /// bypass for the analog clock's duty cycle to be legal. This asks
    /// the silicon rather than trusting the Clock<> type, so a future
    /// task that divides the buses cannot make this file lie.
    static bool clock_ok() { return Rcc::bus_prescalers_are_unity(); }

    // ---- the regulator, the calibration and the on/off pair ------------------

    static bool regulator() { return (regs().CR & ADC_CR_ADVREGEN) != 0u; }

    /**
     * Raise ADVREGEN and spend tADCVREG_STUP (DS13560 table 62 gives
     * 20 us max; 25 is spent). 15.3.2: nothing - not the calibration,
     * not ADEN - is legal before this.
     */
    template <typename Clock>
    static bool regulator_on(Clock clock) {
        regs().CR = (regs().CR & keepable_cr) | ADC_CR_ADVREGEN;
        return delay_us(clock, 25);
    }

    /// 15.3.2: drop it before a low-power mode. The calibration survives
    /// (the chapter's own note) but the analog block does not.
    static void regulator_off() {
        regs().CR = regs().CR & ~ADC_CR_ADVREGEN;
    }

    /**
     * Run ADCAL (15.3.3), the offset self-calibration. Legal only with
     * ADEN = 0, AUTOFF = 0, ADVREGEN = 1 and DMAEN = 0 - all four are
     * checked, because 15.3.7 warns that a forbidden write leaves the
     * converter in an undefined state.
     *
     * @return the calibration factor, or nothing if the preconditions
     * were not met or ADCAL never cleared.
     */
    static bool calibrate(uint32_t spins = 0x100000UL) {
        if (enabled() || (regs().CFGR1 & (ADC_CFGR1_AUTOFF | ADC_CFGR1_DMAEN)) != 0u ||
            !regulator()) {
            return false;
        }
        regs().CR = (regs().CR & keepable_cr) | ADC_CR_ADCAL;
        for (uint32_t i = 0; i < spins; ++i) {
            if ((regs().CR & ADC_CR_ADCAL) == 0u) {
                return true;
            }
        }
        return false;
    }

    /// ADC_CALFACT[6:0] - readable after a calibration, and writable
    /// while the converter is enabled and idle (15.3.3's "calibration
    /// factor forcing"), which is how a saved factor is restored without
    /// paying for a new sweep.
    static uint8_t calibration() {
        return static_cast<uint8_t>(regs().CALFACT & ADC_CALFACT_CALFACT);
    }
    static bool set_calibration(uint8_t factor) {
        if (!enabled() || converting()) {
            return false;
        }
        regs().CALFACT = factor & ADC_CALFACT_CALFACT;
        return true;
    }

    static bool enabled() { return (regs().CR & ADC_CR_ADEN) != 0u; }

    /**
     * 15.3.4's enable procedure, verbatim: clear ADRDY, set ADEN, wait
     * for ADRDY. In AUTOFF mode ADRDY is never set (the chapter's note),
     * so the wait is skipped there and the answer is true.
     */
    static bool enable(uint32_t spins = 0x100000UL) {
        if (enabled()) {
            return true;
        }
        regs().ISR = AdcFlag::ready;
        regs().CR = (regs().CR & keepable_cr) | ADC_CR_ADEN;
        if ((regs().CFGR1 & ADC_CFGR1_AUTOFF) != 0u) {
            return true;
        }
        for (uint32_t i = 0; i < spins; ++i) {
            if ((regs().ISR & AdcFlag::ready) != 0u) {
                return true;
            }
        }
        return false;
    }

    /// 15.3.4's disable procedure: stop anything running, then ADDIS,
    /// then wait for ADEN to clear by itself.
    static bool disable(uint32_t spins = 0x100000UL) {
        if (!enabled()) {
            return true;
        }
        if (converting() && !stop(spins)) {
            return false;
        }
        regs().CR = (regs().CR & keepable_cr) | ADC_CR_ADDIS;
        for (uint32_t i = 0; i < spins; ++i) {
            if (!enabled()) {
                regs().ISR = AdcFlag::ready;
                return true;
            }
        }
        return false;
    }

    // ---- configuration (a DISABLED-state verb, 15.3.7) -----------------------

    static constexpr bool config_valid(const AdcConfig& c) { return adc_config_valid(c); }

    /**
     * Write CFGR1, CFGR2, SMPR and CCR's prescaler from `c`.
     *
     * REFUSES while the converter is enabled - which is 15.3.7's rule and
     * ALSO the structural answer to ES0548 2.6.2 (a CFGR1 write with ADEN
     * set silently resets RES to 12 bits): with no path to that write,
     * the erratum has nothing to act on. `configure_while_enabled()`
     * exists so a suite can stage the erratum deliberately; nothing else
     * should call it.
     */
    static bool configure(const AdcConfig& c) {
        if (!adc_config_valid(c) || enabled() || (regs().CR & ADC_CR_ADCAL) != 0u) {
            return false;
        }
        cfg_ = c;
        write_ccr_prescaler(c);
        regs().CFGR2 = cfgr2_word(c);
        regs().SMPR = smpr_word(c);
        regs().CFGR1 = cfgr1_word(c);
        return true;
    }

    /// ES0548 2.6.2's staging ground and nothing else: the same store
    /// with the enable check removed. Named for what it is so it cannot
    /// be reached for by accident.
    static void configure_while_enabled(const AdcConfig& c) {
        cfg_ = c;
        regs().CFGR1 = cfgr1_word(c);
    }

    static const AdcConfig& config() { return cfg_; }

    /**
     * The whole bring-up in one verb, in the order the chapter requires:
     * bus clock, kernel clock, regulator + its start-up time,
     * calibration, configuration, enable.
     *
     * `async_hz` is the rate of the root ADCSEL selects and is the
     * caller's to state - the same "a ratio meter cannot know what its
     * own reference is worth" shape samc/freqm.hpp and samc/tsens.hpp
     * use. It is only read when the config asks for the asynchronous
     * clock; with a PCLK mode the clock is the one `clock` names.
     *
     * REFUSES when the resulting fADC would exceed DS13560 table 62's
     * ceiling, when PCLK/1 is asked for with the bus prescalers divided
     * (15.3.5's duty-cycle caution), and when the asynchronous root is
     * PLLPCLK with the PLL's P output not enabled - this stratum's
     * Clock<> drives R only, and a converter clocked from a dead branch
     * would simply never convert.
     */
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& c, uint32_t async_hz = 16'000'000UL) {
        if (!adc_config_valid(c)) {
            return false;
        }
        if (c.clock_mode == AdcClockMode::pclk_div1 && !clock_ok()) {
            return false;
        }
        const uint32_t f = adc_clock_hz(clock_hz(clock), async_hz, c);
        if (f == 0u || f > adc_max_hz) {
            return false;
        }
        bus_clock(true);
        reset();
        bus_clock(true);
        if (c.clock_mode == AdcClockMode::async) {
            if (c.async_source == AdcAsyncSource::pllp &&
                (RCC->PLLCFGR & RCC_PLLCFGR_PLLPEN) == 0u) {
                return false;
            }
            async_source(c.async_source);
        }
        if (!regulator_on(clock)) {
            return false;
        }
        if (!calibrate()) {
            return false;
        }
        if (!configure(c)) {
            return false;
        }
        return enable();
    }

    /// Everything this file turned on, off again: conversions stopped,
    /// interrupts disarmed, the converter disabled, the internal sources
    /// powered down, the regulator dropped, the block reset and its bus
    /// clock closed.
    static void release() {
        (void)stop();
        regs().IER = 0;
        (void)disable();
        common().CCR = common().CCR & ~(internal_enable_mask());
        regulator_off();
        reset();
        bus_clock(false);
    }

    // ---- the internal sources (ADC_CCR, 15.9) --------------------------------

    /// Wake VREFINT (15.9's VREFEN). 15.9's own note: drop it before a
    /// Stop mode.
    static void vrefint(bool on) { ccr_bit(ADC_CCR_VREFEN, on); }
    static bool vrefint() { return (common().CCR & ADC_CCR_VREFEN) != 0u; }

    /// Wake the temperature sensor (TSEN). It has a start-up time
    /// (tSTART) the chapter says to spend before the first conversion,
    /// and 15.9's advice is to raise this with ADEN so the two overlap.
    static void temperature(bool on) { ccr_bit(ADC_CCR_TSEN, on); }
    static bool temperature() { return (common().CCR & ADC_CCR_TSEN) != 0u; }

    /// Wake the divided VBAT pin (VBATEN).
    static void vbat(bool on) { ccr_bit(ADC_CCR_VBATEN, on); }
    static bool vbat() { return (common().CCR & ADC_CCR_VBATEN) != 0u; }

    // ---- channel selection (15.3.8, and its handshake) -----------------------

    /// util/analog_sampler.hpp's `input_code`: the channel number a
    /// result was taken on, as `selected()` reports it.
    template <class P, uint8_t ch>
    static constexpr uint8_t input_code(AnalogIn<P, ch>) { return ch; }
    static constexpr uint8_t input_code(AdcInput in) { return static_cast<uint8_t>(in); }

    /**
     * Select ONE channel - the bitmap face of the sequencer with a
     * single bit set.
     *
     * VOID AND BOUNDED, because util/analog_sampler.hpp's AnalogConverter
     * concept asks for a void select() and this silicon needs a
     * handshake: CCRDY is cleared, CHSELR written, CCRDY waited for.
     * `select_sync()` is the same thing with an answer, and is what a
     * caller that can act on a failure should use.
     */
    template <class P, uint8_t ch>
    static void select(AnalogIn<P, ch>) { (void)select_channel(ch); }
    static void select(AdcInput in) { (void)select_channel(static_cast<uint8_t>(in)); }

    template <class P, uint8_t ch>
    static bool select_sync(AnalogIn<P, ch>, uint32_t spins = 0x10000UL) {
        return select_channel(ch, spins);
    }
    static bool select_sync(AdcInput in, uint32_t spins = 0x10000UL) {
        return select_channel(static_cast<uint8_t>(in), spins);
    }

    static bool select_channel(uint8_t ch, uint32_t spins = 0x10000UL) {
        if (ch >= channels) {
            return false;
        }
        return write_chselr(static_cast<uint32_t>(1u) << ch, spins);
    }

    /**
     * The whole bitmap at once (CHSELRMOD = 0): the channels are scanned
     * in numeric order, forward or backward as CFGR1.SCANDIR says.
     */
    static bool sequence(uint32_t mask, uint32_t spins = 0x10000UL) {
        if (cfg_.ordered_sequence || (mask >> channels) != 0u || mask == 0u) {
            return false;
        }
        return write_chselr(mask, spins);
    }

    /**
     * The ORDERED sequence (CHSELRMOD = 1): up to eight slots scanned in
     * the order given, channels 0..14 only, terminated by 0xF (15.3.8).
     * A shorter list is terminated here; a full eight needs none.
     */
    static bool sequence_ordered(const uint8_t* order, uint8_t count,
                                 uint32_t spins = 0x10000UL) {
        if (!cfg_.ordered_sequence || order == nullptr || count == 0u || count > 8u) {
            return false;
        }
        uint32_t word = 0xFFFFFFFFUL;
        for (uint8_t i = 0; i < count; ++i) {
            if (order[i] > 14u) {
                return false;   // the ordered face reaches channel 14 and no further
            }
            word = (word & ~(0xFUL << (4u * i))) | (static_cast<uint32_t>(order[i]) << (4u * i));
        }
        return write_chselr(word, spins);
    }

    /// What CHSELR holds - the bitmap, or the eight packed slots.
    static uint32_t selection() { return regs().CHSELR; }

    /**
     * The channel the LAST result was taken on, which is what
     * util/analog_sampler.hpp labels a sample with.
     *
     * THE SILICON DOES NOT REPORT IT. There is no "current channel"
     * register on this converter (unlike the SAM's INPUTCTRL readback):
     * the sequencer walks a list the driver wrote, so the driver is what
     * knows where it is. With a single-channel selection - the sampler's
     * whole usage - that is exact; inside a multi-channel sequence it is
     * the sequence's FIRST channel, and a caller walking a sequence
     * labels its own results by position.
     */
    static uint8_t selected() { return selected_; }

    // ---- conversions ---------------------------------------------------------

    static bool converting() { return (regs().CR & ADC_CR_ADSTART) != 0u; }

    /// ADSTART. With EXTEN != none this ARMS the hardware trigger rather
    /// than converting at once (15.4); with EXTEN none it is the software
    /// trigger. Void, because util/analog_sampler.hpp asks for a void
    /// start().
    static void start() {
        regs().CR = (regs().CR & keepable_cr) | ADC_CR_ADSTART;
    }

    /// ADSTP (15.3.14): abort, discard the partial result, reset the
    /// sequence. Waits for both bits to clear.
    static bool stop(uint32_t spins = 0x100000UL) {
        if (!converting()) {
            return true;
        }
        regs().CR = (regs().CR & keepable_cr) | ADC_CR_ADSTP;
        for (uint32_t i = 0; i < spins; ++i) {
            if (!converting()) {
                return true;
            }
        }
        return false;
    }

    static bool ready() { return (regs().ISR & AdcFlag::converted) != 0u; }
    static bool sequence_done() { return (regs().ISR & AdcFlag::sequence_done) != 0u; }

    /**
     * ADC_DR. READING IT CLEARS EOC (15.4.3), which is the
     * acknowledgement the converter counts, so a caller that wants the
     * flag left standing must read `flags()` and not this.
     *
     * ES0548 2.6.1: if this read lands in the same APB cycle as a new
     * result, the overrun happens but OVR may stay low. The workaround is
     * a timing obligation on the reader - clear EOC well inside one
     * conversion period - and no driver can enforce it.
     */
    static uint16_t result() { return static_cast<uint16_t>(regs().DR); }

    /// One conversion, polled: start, wait, read. Returns false rather
    /// than hanging if the result never arrives.
    static bool read(uint16_t& out, uint32_t spins = 0x100000UL) {
        start();
        for (uint32_t i = 0; i < spins; ++i) {
            if (ready()) {
                out = result();
                return true;
            }
        }
        return false;
    }

    static uint16_t read(uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        (void)read(v, spins);
        return v;
    }

    /// Convert `count` times and keep the last - what a caller wants
    /// after waking an internal source or changing a channel.
    static uint16_t read_settled(uint8_t count, uint32_t spins = 0x100000UL) {
        uint16_t v = 0;
        for (uint8_t i = 0; i < count; ++i) {
            (void)read(v, spins);
        }
        return v;
    }

    // ---- the arithmetic (util/analog.hpp's `steps` for this converter) -------

    static uint32_t result_steps() { return adc_result_steps(cfg_); }
    static uint32_t sample_steps() { return adc_sample_steps(cfg_.resolution); }

    /// tCONV of channel `ch` in half ADC cycles, under the config in
    /// force.
    static uint32_t conversion_half_cycles(uint8_t ch) {
        return adc_conversion_half_cycles(cfg_, ch);
    }

    /**
     * VREF+ in millivolts, MEASURED: 15.9's own formula,
     * VREF+ = 3.0 V x VREFINT_CAL / VREFINT_DATA. On a board that ties
     * VREF+ to VDDA - which is every board this stratum has met - that
     * is the analog supply, which is why the verb is spelled for what it
     * is used for.
     *
     * The caller supplies the VREFINT reading, because taking one here
     * would mean this verb owning the channel selection and the sequence
     * of whoever called it.
     */
    static uint16_t vdda_mv(uint16_t vrefint_data) {
        if (vrefint_data == 0u || !AdcFactory::plausible()) {
            return 0;
        }
        const uint32_t num = static_cast<uint32_t>(AdcFactory::characterization_mv) *
                             AdcFactory::vrefint_cal();
        return static_cast<uint16_t>((num + vrefint_data / 2u) / vrefint_data);
    }

    /**
     * The junction temperature in HUNDREDTHS of a degree Celsius, from
     * 15.9's two-point formula. Centi-degrees because this stratum has
     * no floating point in a driver and one degree is a coarse answer
     * for a sensor whose own linearity is +/- 2 C.
     *
     * `ts_data` must be a 12-bit right-aligned reading and `vdda_mv_` the
     * supply it was taken at: ST's calibration points are at 3.0 V and a
     * raw count taken at another supply is on another scale, so it is
     * rescaled first. That rescaling is the step 15.9's formula leaves
     * out and every application gets wrong once.
     */
    static int32_t temperature_centi_c(uint16_t ts_data, uint16_t vdda_mv_) {
        if (!AdcFactory::plausible() || vdda_mv_ == 0u) {
            return 0;
        }
        const int32_t cal1 = static_cast<int32_t>(AdcFactory::ts_cal1());
        const int32_t cal2 = static_cast<int32_t>(AdcFactory::ts_cal2());
        const int32_t scaled = static_cast<int32_t>(
            (static_cast<uint32_t>(ts_data) * vdda_mv_ +
             AdcFactory::characterization_mv / 2u) /
            AdcFactory::characterization_mv);
        const int32_t span = (AdcFactory::ts_cal2_celsius - AdcFactory::ts_cal1_celsius) * 100;
        return ((scaled - cal1) * span) / (cal2 - cal1) +
               AdcFactory::ts_cal1_celsius * 100;
    }

    // ---- the three analog watchdogs (15.7) -----------------------------------

    /**
     * AWD1: one channel, or all of them (table 78).
     *
     * A DISABLED-STATE VERB, and the bench is why. The channel selection
     * lives in CFGR1, which 15.3.7 makes writable only with ADEN clear;
     * a forbidden write there DOES land on this silicon (the first
     * version of test_stm32_analog letter h configured this watchdog
     * with the converter running and it worked), but the SAME forbidden
     * write is also what ES0548 2.6.2 turns into a silent reset of the
     * resolution - so the rule is kept and the verb refuses. Its two
     * siblings below refuse for a stronger reason still: their register
     * simply ignores the write.
     *
     * The thresholds are compared against the RAW LEFT-ALIGNED 12-bit
     * data whatever the resolution is (table 77), so a caller at 10 bits
     * keeps the two low bits of both thresholds clear - and while
     * oversampling the comparison is on the top 12 bits of the 16
     * (15.8.2).
     *
     * ES0548 2.6.3: in single-channel mode this watchdog misses a
     * channel that is not the FIRST of the sequence. The obligation is
     * the caller's - this verb does not own the sequence - and it is
     * stated rather than pretended away.
     */
    static bool watchdog1(uint16_t low, uint16_t high, bool single, uint8_t channel = 0) {
        if (enabled() || low > 0xFFFu || high > 0xFFFu ||
            (single && channel >= channels)) {
            return false;
        }
        regs().AWD1TR = (static_cast<uint32_t>(high) << ADC_AWD1TR_HT1_Pos) | low;
        uint32_t c = regs().CFGR1 & ~(ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1CH);
        c |= ADC_CFGR1_AWD1EN;
        if (single) {
            c |= ADC_CFGR1_AWD1SGL |
                 (static_cast<uint32_t>(channel) << ADC_CFGR1_AWD1CH_Pos);
        }
        regs().CFGR1 = c;
        return true;
    }

    static bool watchdog1_off() {
        if (enabled()) {
            return false;
        }
        regs().CFGR1 = regs().CFGR1 & ~(ADC_CFGR1_AWD1EN | ADC_CFGR1_AWD1SGL);
        return true;
    }

    /**
     * AWD2 and AWD3: a MASK of channels each, and the mask is also the
     * enable - "the corresponding watchdog is enabled when any AWDxCHy
     * bit is set" (15.7.2), so `watchdog2(0, ...)` is how one is turned
     * off and there is no separate bit to forget.
     *
     * ALSO A DISABLED-STATE VERB, and here the silicon is unforgiving:
     * 15.12.13's own note says "the software is allowed to write this bit
     * only when ADEN = 0", and MEASURED, a write with the converter
     * enabled takes no effect AT ALL - in silence, and unlike CFGR1's
     * AWD1 bits, which land. Same sentence in the manual, two behaviours
     * in the silicon (test_stm32_analog letter h stages both).
     */
    static bool watchdog2(uint32_t channel_mask, uint16_t low, uint16_t high) {
        return watchdog_23(true, channel_mask, low, high);
    }
    static bool watchdog3(uint32_t channel_mask, uint16_t low, uint16_t high) {
        return watchdog_23(false, channel_mask, low, high);
    }

    static uint32_t watchdog2_channels() { return regs().AWD2CR; }
    static uint32_t watchdog3_channels() { return regs().AWD3CR; }

    /**
     * The THRESHOLDS alone, which are the one part of a watchdog that
     * really is live: 15.7.4 says LTx and HTx "can be changed during an
     * analog-to-digital conversion", with the comparison masked for the
     * conversion that was in flight and the new window in force from the
     * next one. So a control loop moves its limits without stopping the
     * converter, and only the CHANNEL SELECTION costs an enable cycle.
     */
    static bool watchdog_thresholds(uint8_t watchdog, uint16_t low, uint16_t high) {
        if (low > 0xFFFu || high > 0xFFFu) {
            return false;
        }
        const uint32_t tr = (static_cast<uint32_t>(high) << 16) | low;
        switch (watchdog) {
            case 1: regs().AWD1TR = tr; return true;
            case 2: regs().AWD2TR = tr; return true;
            case 3: regs().AWD3TR = tr; return true;
            default: return false;
        }
    }

    // ---- flags and interrupts -------------------------------------------------

    static uint32_t flags() { return regs().ISR; }
    static bool flag(uint32_t mask) { return (regs().ISR & mask) != 0u; }
    /// ADC_ISR is write-1-to-clear (15.12.1) - unlike the timers' rc_w0
    /// status register, which is this stratum's one inverted reflex.
    static void clear_flags(uint32_t mask) { regs().ISR = mask; }

    static bool overrun() { return (regs().ISR & AdcFlag::overrun) != 0u; }

    static void interrupts(uint32_t mask, bool on) {
        regs().IER = on ? (regs().IER | mask) : (regs().IER & ~mask);
    }
    static uint32_t interrupts() { return regs().IER; }

    /**
     * The ISR BODY: read the flags this converter has ARMED, clear
     * exactly those, hand them back. The vector is shared with the
     * comparators where they exist, so a handler answers for its own
     * sources and this returns 0 when none of them spoke.
     *
     * IT DOES NOT READ ADC_DR - the result is the handler's to take, and
     * reading it here would clear EOC behind the handler's back.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t hit = regs().ISR & regs().IER;
        if (hit != 0u) {
            regs().ISR = hit;
        }
        return hit;
    }

private:
    /// The bits of ADC_CR a read-modify-write must NOT carry back: every
    /// command bit is set-only and writing a stale one re-issues it
    /// (15.12.3's own notes), so a command store keeps the regulator and
    /// nothing else.
    static constexpr uint32_t keepable_cr = ADC_CR_ADVREGEN;

    static constexpr uint32_t internal_enable_mask() {
        return ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_VBATEN;
    }

    static void ccr_bit(uint32_t mask, bool on) {
        common().CCR = on ? (common().CCR | mask) : (common().CCR & ~mask);
    }

    static void write_ccr_prescaler(const AdcConfig& c) {
        common().CCR = (common().CCR & ~ADC_CCR_PRESC) |
                     (static_cast<uint32_t>(c.prescaler) << ADC_CCR_PRESC_Pos);
    }

    /// The CCRDY handshake of 15.3.8, in one place: clear the flag, write
    /// the register, wait for it. Also remembers the channel a single
    /// selection names, which is what `selected()` reports.
    static bool write_chselr(uint32_t word, uint32_t spins) {
        if (converting()) {
            return false;
        }
        regs().ISR = AdcFlag::channels_ready;
        regs().CHSELR = word;
        for (uint32_t i = 0; i < spins; ++i) {
            if ((regs().ISR & AdcFlag::channels_ready) != 0u) {
                selected_ = first_channel(word);
                return true;
            }
        }
        selected_ = first_channel(word);
        return false;
    }

    /// Which channel a CHSELR word starts on - the low nibble in the
    /// ordered face, the lowest set bit (or the highest, scanning
    /// backward) in the bitmap one.
    static uint8_t first_channel(uint32_t word) {
        if (cfg_.ordered_sequence) {
            return static_cast<uint8_t>(word & 0xFu);
        }
        if (cfg_.scan == AdcScanDir::backward) {
            for (uint8_t i = channels; i-- > 0;) {
                if ((word >> i) & 1u) {
                    return i;
                }
            }
            return 0;
        }
        for (uint8_t i = 0; i < channels; ++i) {
            if ((word >> i) & 1u) {
                return i;
            }
        }
        return 0;
    }

    static bool watchdog_23(bool second, uint32_t channel_mask, uint16_t low, uint16_t high) {
        if (enabled() || low > 0xFFFu || high > 0xFFFu ||
            (channel_mask >> channels) != 0u) {
            return false;
        }
        const uint32_t tr = (static_cast<uint32_t>(high) << 16) | low;
        if (second) {
            regs().AWD2TR = tr;
            regs().AWD2CR = channel_mask;
        } else {
            regs().AWD3TR = tr;
            regs().AWD3CR = channel_mask;
        }
        return true;
    }

    static constexpr uint32_t cfgr1_word(const AdcConfig& c) {
        uint32_t w = 0;
        if (c.dma) w |= ADC_CFGR1_DMAEN;
        if (c.dma_circular) w |= ADC_CFGR1_DMACFG;
        if (c.scan == AdcScanDir::backward) w |= ADC_CFGR1_SCANDIR;
        w |= static_cast<uint32_t>(c.resolution) << ADC_CFGR1_RES_Pos;
        if (c.left_aligned) w |= ADC_CFGR1_ALIGN;
        w |= static_cast<uint32_t>(c.trigger) << ADC_CFGR1_EXTSEL_Pos;
        w |= static_cast<uint32_t>(c.trigger_edge) << ADC_CFGR1_EXTEN_Pos;
        if (c.overrun_overwrite) w |= ADC_CFGR1_OVRMOD;
        if (c.continuous) w |= ADC_CFGR1_CONT;
        if (c.wait) w |= ADC_CFGR1_WAIT;
        if (c.auto_off) w |= ADC_CFGR1_AUTOFF;
        if (c.discontinuous) w |= ADC_CFGR1_DISCEN;
        if (c.ordered_sequence) w |= ADC_CFGR1_CHSELRMOD;
        return w;
    }

    static constexpr uint32_t cfgr2_word(const AdcConfig& c) {
        uint32_t w = static_cast<uint32_t>(c.clock_mode) << ADC_CFGR2_CKMODE_Pos;
        if (c.low_frequency_trigger) w |= ADC_CFGR2_LFTRIG;
        if (c.oversampling) {
            w |= ADC_CFGR2_OVSE;
            w |= static_cast<uint32_t>(c.oversampling_ratio) << ADC_CFGR2_OVSR_Pos;
            w |= static_cast<uint32_t>(c.oversampling_shift) << ADC_CFGR2_OVSS_Pos;
            if (c.triggered_oversampling) w |= ADC_CFGR2_TOVS;
        }
        return w;
    }

    static constexpr uint32_t smpr_word(const AdcConfig& c) {
        return (static_cast<uint32_t>(c.sample1) << ADC_SMPR_SMP1_Pos) |
               (static_cast<uint32_t>(c.sample2) << ADC_SMPR_SMP2_Pos) |
               ((c.sample2_channels << ADC_SMPR_SMPSEL_Pos) & ADC_SMPR_SMPSEL_Msk);
    }

    inline static AdcConfig cfg_{};
    inline static uint8_t selected_ = 0;
};

} // namespace brio
