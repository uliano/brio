/*
 * tsens.hpp
 *
 * The SAM C21's on-die temperature sensor (DS60001479M ch. 43), a whole
 * small chapter in one monostate resource - `Tsens`, not `Tsens<n>`:
 * this family has exactly ONE instance on every variant, so an index
 * would be a parameter with a single legal value (the `Rtc` / `Dac` /
 * `Sdadc` precedent in this stratum).
 *
 *   brio::Tsens::init(generator, brio::TsensConfig{
 *       .calibration = brio::TsensCalibration::factory()});
 *   const auto centi = brio::Tsens::measure_average(10);   // 43.6.2.3
 *
 * ---------------------------------------------------------------------
 * THIS IS NOT AN ADC CHANNEL, and everything else follows from that.
 *
 * On the AVR the die temperature is one more analog input: a voltage
 * into the SAR, a SIGROW correction, a reading in kelvin. HERE IT IS A
 * CLOCK RATIO. A temperature-dependent oscillator (TOSC) is run twice -
 * once in its "min" configuration and once in its "max" - and the
 * DIFFERENCE of the two periods, amplified over GAIN periods of
 * GCLK_TSENS, is counted by a counter clocked by GCLK_TSENS itself: up
 * during the first phase, down during the second (43.6.1). What lands in
 * VALUE is
 *
 *     VALUE = OFFSET + GAIN x (f_TOSCMIN - f_TOSCMAX) / f_GCLK
 *
 * so THE GENERIC CLOCK IS THE MEASUREMENT'S RULER, not merely its pace.
 * Three consequences the chapter states only in a note, and this driver
 * takes seriously:
 *
 * 1. THE FACTORY GAIN AND OFFSET ARE CALIBRATED FOR ONE CLOCK - "the
 *    undivided internal 48MHz oscillator (OSC48M)". Under that clock,
 *    and only under it, VALUE is a temperature in HUNDREDTHS OF A DEGREE
 *    CELSIUS: 43.8.10's own example gives 2500 for 25 C and -2500 for
 *    -25 C. That is the unit this whole header speaks, and it is why
 *    nothing here uses a float: a signed 24-bit register already carries
 *    +-83886 C at a centi-degree a step.
 *
 * 2. ANY OTHER GCLK_TSENS RATE RESCALES THE ANSWER, by exactly
 *    f_actual / 48 MHz on the GAIN term. Two ways out, and both are
 *    offered as constexpr arithmetic that takes the rate AS A CALLER
 *    ARGUMENT (the samc/freqm.hpp `reference_hz` pattern - a ratio
 *    meter cannot know what its own reference is worth):
 *      - `tsens_gain_for(factory_gain, f)` scales the GAIN register, so
 *        VALUE keeps arriving in centi-degrees;
 *      - `tsens_rescale(value, offset, f)` scales the READING, for a
 *        caller who would rather leave the factory number alone.
 *
 * 3. AND THE REFERENCE'S OWN ERROR IS IN THE ANSWER. OSC48M on the bench
 *    die measures 47.755 MHz against the board's crystal - 5100 ppm slow,
 *    well inside table 45-57 and not a fault - so a reading taken on the
 *    factory clock is scaled by 48/47.755 in its GAIN term. It is a
 *    small error on a temperature and a large one on the SPAN from
 *    OFFSET, which is where it is actually applied; docs/samc/tsens.md
 *    carries the two-reference measurement.
 *
 * ---------------------------------------------------------------------
 * CALIBRATION IS THE CHAPTER'S HEART (43.5.9). Four factory values live
 * in the NVM Temperature Calibration Area at 0x00806030 (table 9-6) and
 * MUST be copied in by software: GAIN and OFFSET into their own
 * registers, TCAL and FCAL into CAL. `samc/nvm.hpp`'s
 * `NvmTemperatureCalibration` has typed all four since the NVMCTRL pass;
 * `TsensCalibration::factory()` is the promise that file's comment made,
 * kept - the same shape `Adc::load_calibration()` has for the SAR.
 *
 * WITHOUT THEM THE PERIPHERAL IS NOT MERELY INACCURATE, AND ITS RESET
 * VALUE IS A TRAP. GAIN reads zero out of reset, and a zero GAIN is not
 * "no gain": 43.8.13 makes GAIN "the number of GCLK_TSENS periods that
 * will be used for a measurement cycle" and the field is 24 bits wide, so
 * the value that reads as 0 behaves as 2^24. MEASURED, twice over
 * (docs/samc/tsens.md): the conversion takes 699 ms instead of 3.7 ms -
 * 2 x 2^24 periods at 48 MHz to the millisecond - and the result is the
 * gain term multiplied by 2^24 / GAIN, about two hundredfold, which
 * arrives looking like a temperature of -16000 C. `tsens_config_valid()`
 * therefore refuses a zero GAIN outright rather than letting a caller
 * wait most of a second for that.
 *
 * ---------------------------------------------------------------------
 * THE REGISTER DISCIPLINES, spelled per register the way this stratum's
 * other converters spell theirs.
 *
 * 1. ENABLE-PROTECTED (43.6.2.1): CTRLC, EVCTRL, WINLT, WINUT, GAIN,
 *    OFFSET, CAL - and CTRLA.RUNSTDBY, which is a BIT and not a
 *    register. Everything on that list is written by `init()` between
 *    the reset and the enable; the individual verbs that touch them
 *    (`window()`, `event_config()`, `calibration()`) refuse while the
 *    block is enabled rather than storing into a register that would
 *    drop the write.
 * 2. WRITE-SYNCHRONIZED (43.6.7): CTRLA.SWRST and CTRLA.ENABLE, and
 *    NOTHING ELSE - SYNCBUSY has exactly two bits and the device header
 *    masks it 0x3. THE WAIT IS ON THE NEAR SIDE OF EVERY STORE, and
 *    every such verb returns bool, because 43.6.7 says an operation
 *    requiring synchronization issued while its busy bit stands "is
 *    discarded and a bus error is generated" - and a bus error on a
 *    Cortex-M0+ is a HardFault. That is the samc/sdadc.hpp position,
 *    taken for the same sentence in 39.6.8.
 * 3. NOT PAC-PROTECTED (43.5.8): CTRLB and INTFLAG - the pair a DMA
 *    engine and an ISR need. SEE THE ERRATUM BELOW: this is exactly the
 *    sentence the silicon does not honour.
 * 4. NOT RESET BY A SOFTWARE RESET (43.8.1): GAIN, OFFSET, CAL and
 *    DBGCTRL. So a `reset()` keeps the calibration - which is what makes
 *    reconfiguring a running sensor cheap, and is asserted at the bench.
 * 5. WRITE-ONLY: CTRLB is `__O` in the device header and drawn with W
 *    access in 43.8.2. `start()` is a plain store; there is no way to
 *    ask whether a start was taken, and BUSY does not exist in this
 *    block - INTFLAG.RESRDY is the only evidence a measurement finished.
 *
 * ---------------------------------------------------------------------
 * HOW WIDE IS VALUE? The chapter says both. 43.6.4 introduces
 * INTFLAG.OVF as "the result required more than 16 bits and overflowed
 * the VALUE register"; 43.8.7's own bit description says TWENTY-FOUR,
 * and the register summary, the bit table and `TSENS_VALUE_Msk` all draw
 * VALUE[23:0]. This header follows the register description - the datum
 * is a signed 24-bit two's-complement number, `tsens_signed()` sign-
 * extends it and `tsens_value_fits()` bounds it - and docs/samc/tsens.md
 * carries the measurement that convicts the other sentence.
 *
 * ---------------------------------------------------------------------
 * THE WINDOW MONITOR (43.6.2.4) compares VALUE against WINLT and WINUT,
 * both signed 24-bit. Six modes plus a Reserved seventh. ONE OF THEM IS
 * DESCRIBED TWICE AND DIFFERENTLY: 43.8.3's table prints OUTSIDE as
 * "WINUT < VALUE < WINLT" while the device header's own enumerator
 * comment says "VALUE less than WINLT or VALUE greater than WINUT" -
 * which are not the same condition, and do not even want the thresholds
 * in the same order. `tsens_config_valid()` therefore refuses a crossed
 * pair for INSIDE and for the two hysteresis modes (where both documents
 * agree) and refuses NOTHING for OUTSIDE; the bench says which reading
 * is the silicon's.
 *
 * ---------------------------------------------------------------------
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F.
 *
 *  - 1.19.1 PAC Write-protection, LIVE ON EVERY REVISION: "when PAC
 *    Write-Protection is enabled for TSENS, writes to TSENS.CTRLB are
 *    not functional", i.e. a START written under protection does
 *    nothing. It contradicts 43.5.8, which lists CTRLB among the
 *    registers PAC protection does not cover. THE OBLIGATION IS THE
 *    CALLER'S and this header states it rather than coding around it,
 *    because brio has no PAC driver at all: PAC write protection is OFF
 *    out of reset (11.5.2.2) and nothing in this framework turns it on,
 *    so the item is inapplicable by construction TODAY and becomes a
 *    real constraint the day a PAC pass arrives. `Tsens::pac_id` is the
 *    peripheral identifier that pass will need. The errata's other
 *    workaround - "or use the TSENS in free-running mode" - is exactly
 *    `TsensConfig::free_running`, which needs no CTRLB write after the
 *    enable.
 *  - There is no other TSENS item in the document, on any row.
 *
 * ---------------------------------------------------------------------
 * NOT BUILT (docs/samc/tsens.md carries the list): sleep behaviour
 * beyond the one CTRLA bit (table 43-1 is written and never entered),
 * and the E/G variants are compile-only.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/device_tables.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"

namespace brio {

// =============================================================================
// The datum: a signed 24-bit number of CENTI-DEGREES CELSIUS
// =============================================================================

/// The GCLK_TSENS rate the factory GAIN and OFFSET are calibrated for
/// (43.6.1's note): OSC48M undivided, i.e. 48 MHz NOMINAL. The real
/// oscillator's error is the caller's to know and is what
/// `tsens_rescale()` exists for.
inline constexpr uint32_t tsens_calibration_gclk_hz = 48'000'000UL;

/// VALUE, WINLT, WINUT, GAIN and OFFSET are all 24-bit fields.
inline constexpr uint32_t tsens_field_mask = 0x00FFFFFFUL;
inline constexpr int32_t tsens_value_min = -8388608;
inline constexpr int32_t tsens_value_max = 8388607;

/// Sign-extend a 24-bit two's-complement register field (43.8.10).
constexpr int32_t tsens_signed(uint32_t raw) {
    const uint32_t v = raw & tsens_field_mask;
    return static_cast<int32_t>((v & 0x800000UL) != 0u ? (v | 0xFF000000UL) : v);
}

/// The inverse: a signed value packed back into a 24-bit field, for
/// WINLT / WINUT / OFFSET.
constexpr uint32_t tsens_field(int32_t value) {
    return static_cast<uint32_t>(value) & tsens_field_mask;
}

/// Whether a signed value is one a 24-bit field can carry.
constexpr bool tsens_value_fits(int32_t value) {
    return value >= tsens_value_min && value <= tsens_value_max;
}

/**
 * A reading taken with GCLK_TSENS at `gclk_hz`, rescaled to what the
 * factory calibration would have produced at its own 48 MHz.
 *
 * Only the GAIN term carries the clock (43.6.1), so the OFFSET is the
 * pivot and not part of the scaling: the span from OFFSET is what moves.
 * `offset` is the OFFSET register's own signed value - `Tsens::offset()`
 * for the one in force.
 *
 * The arithmetic is 64-bit on the way through: a span of tens of
 * thousands times a rate of tens of millions leaves 32 bits at once.
 * The division truncates toward zero, which is at most one centi-degree
 * and is not rounded away, because rounding a number this far inside its
 * own +-11 C accuracy band (table 45-37) would be decoration.
 */
constexpr int32_t tsens_rescale(int32_t value, int32_t offset, uint32_t gclk_hz,
                                uint32_t calibration_hz = tsens_calibration_gclk_hz) {
    if (gclk_hz == 0u || calibration_hz == 0u) {
        return value;
    }
    const int64_t span = static_cast<int64_t>(value) - static_cast<int64_t>(offset);
    return static_cast<int32_t>(
        static_cast<int64_t>(offset) +
        (span * static_cast<int64_t>(gclk_hz)) / static_cast<int64_t>(calibration_hz));
}

/**
 * The other way round: the GAIN a caller should WRITE so that VALUE
 * keeps arriving in centi-degrees with GCLK_TSENS at `gclk_hz`.
 *
 * VALUE - OFFSET is proportional to GAIN / f_GCLK, so a clock n times
 * faster wants a GAIN n times bigger. Rounded to nearest and clamped to
 * the field; a rate that would need more than 24 bits of GAIN cannot be
 * compensated this way and the clamp says so by producing a saturated
 * value the caller can compare against.
 */
constexpr uint32_t tsens_gain_for(uint32_t factory_gain, uint32_t gclk_hz,
                                  uint32_t calibration_hz = tsens_calibration_gclk_hz) {
    if (calibration_hz == 0u) {
        return factory_gain;
    }
    const uint64_t scaled =
        (static_cast<uint64_t>(factory_gain) * gclk_hz + calibration_hz / 2u) /
        calibration_hz;
    return scaled > tsens_field_mask ? tsens_field_mask
                                     : static_cast<uint32_t>(scaled);
}

/// Centi-degrees to milli-degrees, for a caller whose other numbers are
/// in thousandths. No float anywhere in this header.
constexpr int32_t tsens_milli_celsius(int32_t centi) { return centi * 10; }

// =============================================================================
// The factory calibration (43.5.9, table 9-6)
// =============================================================================

/**
 * The four production values, in one struct so that "calibrated" is a
 * thing a configuration can CARRY rather than a ritual a caller has to
 * remember.
 *
 * `factory()` reads them out of the NVM Temperature Calibration Area
 * through samc/nvm.hpp - which typed all four in the NVMCTRL pass with a
 * comment promising this driver would consume them.
 */
struct TsensCalibration {
    /// GAIN[23:0]: the number of GCLK_TSENS periods a measurement cycle
    /// uses, and therefore the slope. ZERO IS NOT "NONE" - the field is
    /// 24 bits and reads-as-zero behaves as 2^24 (see this file's header),
    /// which is why it is refused rather than accepted as a default.
    uint32_t gain = 0;
    /// OFFSET[23:0], signed: added to the gain term (43.6.1). Held as a
    /// signed value here and packed on the way to the register.
    int32_t offset = 0;
    /// CAL.TCAL[5:0] and CAL.FCAL[5:0]: trims of the temperature-
    /// dependent oscillator itself. 43.8.15 is explicit that these "must
    /// be copied only, and must not be changed".
    uint8_t tcal = 0;
    uint8_t fcal = 0;

    /// The production values for this die.
    static TsensCalibration factory() {
        const NvmTemperatureCalibration c = NvmTemperatureCalibration::read();
        return TsensCalibration{
            .gain = c.tsens_gain(),
            .offset = tsens_signed(c.tsens_offset()),
            .tcal = c.tsens_tcal(),
            .fcal = c.tsens_fcal(),
        };
    }

    /// A weak sanity check on what came out of the calibration area: a
    /// blank (erased) row reads all ones, and a GAIN of zero measures
    /// nothing. NOT an accuracy claim - only that something was
    /// programmed there.
    constexpr bool programmed() const {
        return gain != 0u && gain != tsens_field_mask;
    }

    constexpr uint32_t cal_word() const {
        return TSENS_CAL_TCAL(tcal & 0x3Fu) | TSENS_CAL_FCAL(fcal & 0x3Fu);
    }
};

// =============================================================================
// The configuration
// =============================================================================

/// CTRLC.WINMODE (43.8.3). 0x7 is Reserved.
enum class TsensWindow : uint8_t {
    disabled = TSENS_CTRLC_WINMODE_DISABLE_Val,
    above = TSENS_CTRLC_WINMODE_ABOVE_Val,             ///< VALUE > WINLT
    below = TSENS_CTRLC_WINMODE_BELOW_Val,             ///< VALUE < WINUT
    inside = TSENS_CTRLC_WINMODE_INSIDE_Val,           ///< WINLT < VALUE < WINUT
    /// THE ONE MODE THE TWO DOCUMENTS DESCRIBE DIFFERENTLY - 43.8.3
    /// prints "WINUT < VALUE < WINLT", the device header's enumerator
    /// comment says "VALUE < WINLT or VALUE > WINUT". Nothing here
    /// refuses either threshold order for this mode; docs/samc/tsens.md
    /// carries what the silicon does.
    outside = TSENS_CTRLC_WINMODE_OUTSIDE_Val,
    /// VALUE > WINUT, releasing back below WINLT.
    hysteresis_above = TSENS_CTRLC_WINMODE_HYST_ABOVE_Val,
    /// VALUE < WINLT, releasing back above WINUT.
    hysteresis_below = TSENS_CTRLC_WINMODE_HYST_BELOW_Val,
};

constexpr bool tsens_window_valid(TsensWindow w) {
    return static_cast<uint8_t>(w) <= static_cast<uint8_t>(TsensWindow::hysteresis_below);
}

/// Whether a mode uses both thresholds AND wants them in the documented
/// order (lower below upper). `outside` is deliberately not in this set -
/// see the enum's comment.
constexpr bool tsens_window_needs_ordered_pair(TsensWindow w) {
    return w == TsensWindow::inside || w == TsensWindow::hysteresis_above ||
           w == TsensWindow::hysteresis_below;
}

/// EVCTRL (43.8.4), both directions in one struct.
struct TsensEventControl {
    /// EVCTRL.STARTEI: an incoming event starts a measurement. TABLE
    /// 29-3 GIVES THIS USER ALL THREE PATHS - asynchronous, synchronous
    /// AND resynchronized - which makes it the exception among this
    /// stratum's converters, whose START users are asynchronous-only.
    bool start_in = false;
    /// EVCTRL.STARTINV: act on the falling edge instead.
    bool invert_start = false;
    /// EVCTRL.WINEO: the window monitor's match becomes an output event.
    bool window_out = false;
};

/// Inverting an input nobody listens to is a configuration with no
/// meaning - the refusal samc/dac.hpp, samc/adc.hpp and samc/sdadc.hpp
/// all make.
constexpr bool tsens_event_control_valid(const TsensEventControl& e) {
    return !e.invert_start || e.start_in;
}

/// The whole configuration of the sensor.
struct TsensConfig {
    /// GAIN, OFFSET and CAL - enable-protected, and NOT reset by a
    /// software reset. Default-constructed it is all zeros, which
    /// `tsens_config_valid()` refuses: a zero GAIN is 2^24, not none.
    TsensCalibration calibration{};

    // -- CTRLC (enable-protected) --
    /// CTRLC.FREERUN: a new measurement starts when the previous one
    /// ends, with no trigger at all. Also erratum 1.19.1's second
    /// workaround, since it needs no CTRLB write.
    bool free_running = false;
    TsensWindow window = TsensWindow::disabled;

    // -- WINLT / WINUT (enable-protected), signed centi-degrees --
    int32_t window_lower = 0;
    int32_t window_upper = 0;

    // -- CTRLA.RUNSTDBY (enable-protected bit, not synchronized) --
    /// Table 43-1: with FREERUN clear the block runs in standby only
    /// when something requests it; with FREERUN set it always runs.
    bool run_standby = false;

    // -- DBGCTRL (survives a software reset) --
    bool debug_run = false;

    // -- EVCTRL (enable-protected) --
    TsensEventControl events{};
};

/**
 * What the compile-time form static_asserts and the runtime form returns
 * false for. Every rule is the chapter's or the arithmetic's.
 */
constexpr bool tsens_config_valid(const TsensConfig& c) {
    // 43.8.3: WINMODE 0x7 is Reserved.
    if (!tsens_window_valid(c.window)) {
        return false;
    }
    // A zero GAIN is the RESET VALUE and it is not "no gain": the field
    // is 24 bits and 0 behaves as 2^24, so the conversion takes 699 ms
    // and the result is amplified about two hundredfold. See this file's
    // header for the two measurements that say so.
    if (c.calibration.gain == 0u) {
        return false;
    }
    if (!tsens_value_fits(c.calibration.offset)) {
        return false;
    }
    if (c.calibration.gain > tsens_field_mask) {
        return false;
    }
    // The thresholds are 24-bit signed fields like the value they are
    // compared against.
    if (c.window != TsensWindow::disabled &&
        (!tsens_value_fits(c.window_lower) || !tsens_value_fits(c.window_upper))) {
        return false;
    }
    // Where both documents agree that a mode uses an ORDERED pair, a
    // crossed pair is a window that can never open.
    if (tsens_window_needs_ordered_pair(c.window) &&
        c.window_lower >= c.window_upper) {
        return false;
    }
    return tsens_event_control_valid(c.events);
}

// =============================================================================
// The sensor
// =============================================================================

/**
 * The one temperature sensor. A monostate resource: this family has a
 * single instance on every C21 variant, so there is no index to carry.
 */
class Tsens {
    static_assert(tsens_count() == 1u,
                  "this device has no temperature sensor (chapter 43 is "
                  "SAM C21 only, and TSENS is absent from the AEC-Q100 "
                  "qualified part numbers - table 1-1's note)");

public:
    Tsens() = delete;

    static constexpr uint8_t gclk_id = tsens_gclk_id();
    static constexpr IRQn_Type irq() { return TSENS_IRQn; }

    /// The PAC peripheral identifier. Published because erratum 1.19.1 is
    /// about this number and because there is no PAC driver yet to own
    /// it (see this file's header).
    static constexpr uint16_t pac_id = tsens_pac_id();

    // ---- the vocabularies this peripheral publishes -------------------------
    //
    // evsys.hpp owns the FABRIC and dmac.hpp owns the CHANNELS; the codes
    // of their tables that belong to the TSENS live here, probed from the
    // device header in samc/device_tables.hpp.

    /// Generator: the window monitor matched (43.6.5).
    static constexpr uint8_t window_generator = tsens_winmon_generator();
    /// User: start a measurement. TABLE 29-3 GIVES IT ALL THREE PATHS -
    /// it is user 0, the first row of that table, and the only converter
    /// user on this family that is not asynchronous-only.
    static constexpr uint8_t start_event_user = tsens_start_user();
    /// DMAC trigger: the one DMA request this peripheral has (43.6.3),
    /// set when a result is available and cleared when VALUE is read.
    static constexpr uint8_t dma_trigger_resrdy = tsens_dma_resrdy_id();

    /// INTFLAG / INTENSET / INTENCLR bits, named.
    static constexpr uint8_t flag_result_ready = TSENS_INTFLAG_RESRDY_Msk;
    static constexpr uint8_t flag_overrun = TSENS_INTFLAG_OVERRUN_Msk;
    static constexpr uint8_t flag_window = TSENS_INTFLAG_WINMON_Msk;
    static constexpr uint8_t flag_overflow = TSENS_INTFLAG_OVF_Msk;
    static constexpr uint8_t flag_all = TSENS_INTFLAG_Msk;

    static tsens_registers_t& regs() { return *TSENS_REGS; }

    // ---- claim and teardown -------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_a(MCLK_APBAMASK_TSENS_Msk, on); }

    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    static uint32_t sync_busy() { return regs().TSENS_SYNCBUSY; }
    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().TSENS_SYNCBUSY, mask, false, spins);
    }

    /**
     * CTRLA.SWRST. GAIN, OFFSET, CAL and DBGCTRL SURVIVE IT (43.8.1) -
     * so a reset keeps the die's calibration and only the operating
     * configuration goes back to zero.
     *
     * THE WAIT IS BEFORE THE STORE, and this is the one discipline this
     * driver never relaxes: 43.6.7 promises a BUS ERROR - a HardFault on
     * this core - for a synchronized write issued while its own busy bit
     * stands. False means the block was still busy and NOTHING WAS
     * WRITTEN.
     */
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!sync_wait(TSENS_SYNCBUSY_Msk, spins)) {
            return false;
        }
        regs().TSENS_CTRLA = TSENS_CTRLA_SWRST_Msk;
        return sync_wait(TSENS_SYNCBUSY_SWRST_Msk, spins);
    }

    /// CTRLA.ENABLE, preserving RUNSTDBY. Same wait-then-store rule.
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        if (!sync_wait(TSENS_SYNCBUSY_Msk, spins)) {
            return false;
        }
        const uint8_t keep =
            static_cast<uint8_t>(regs().TSENS_CTRLA & TSENS_CTRLA_RUNSTDBY_Msk);
        regs().TSENS_CTRLA =
            on ? static_cast<uint8_t>(keep | TSENS_CTRLA_ENABLE_Msk) : keep;
        return sync_wait(TSENS_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() {
        return (regs().TSENS_CTRLA & TSENS_CTRLA_ENABLE_Msk) != 0u;
    }

    /**
     * The whole chapter in one call: bus clock, generic clock, reset, the
     * enable-protected registers, the ones that are not, and the enable.
     *
     * `generator` is a GCLK GENERATOR NUMBER, and WHICH ONE MATTERS
     * ARITHMETICALLY here in a way it does not for any other peripheral
     * in this stratum - the generator's rate is the measurement's ruler.
     * The factory calibration assumes 48 MHz; anything else wants
     * `tsens_gain_for()` on the way in or `tsens_rescale()` on the way
     * out.
     */
    static bool init(uint8_t generator, const TsensConfig& cfg,
                     uint32_t spins = 0xFFFFu) {
        if (!tsens_config_valid(cfg)) {
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
        // Enable-protected, and the block is disabled out of reset.
        write_calibration(cfg.calibration);
        regs().TSENS_CTRLC = control_c_word(cfg);
        regs().TSENS_WINLT = tsens_field(cfg.window_lower);
        regs().TSENS_WINUT = tsens_field(cfg.window_upper);
        regs().TSENS_EVCTRL = event_word(cfg.events);
        // Not enable-protected.
        regs().TSENS_DBGCTRL = cfg.debug_run ? TSENS_DBGCTRL_DBGRUN_Msk : 0u;
        regs().TSENS_INTENCLR = flag_all;
        regs().TSENS_INTFLAG = flag_all;

        if (cfg.run_standby) {
            // RUNSTDBY is enable-protected and NOT synchronized (43.8.1),
            // so it goes in before the enable and on its own.
            regs().TSENS_CTRLA = TSENS_CTRLA_RUNSTDBY_Msk;
        }
        cfg_ = cfg;
        return enable(true, spins);
    }

    /// The compile-time twin: every rule of `tsens_config_valid()`
    /// becomes a compile error instead of a false return.
    template <TsensConfig cfg>
    static bool init(uint8_t generator, uint32_t spins = 0xFFFFu) {
        static_assert(tsens_config_valid(cfg),
                      "brio TsensConfig: see tsens_config_valid() - the "
                      "Reserved WINMODE code, a zero GAIN (which is 2^24 "
                      "and not none), a threshold outside the 24-bit "
                      "signed field, a crossed threshold pair in a mode that "
                      "needs an ordered one, or an inverted event input "
                      "nothing listens to");
        return init(generator, cfg, spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)enable(false, spins);
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        bus_clock(false);
    }

    /// The configuration in force, as `init()` was given it.
    static const TsensConfig& config() { return cfg_; }

    // ---- calibration ---------------------------------------------------------

    /**
     * GAIN, OFFSET and CAL. Enable-protected, so this REFUSES while the
     * block is enabled rather than storing into registers that would
     * drop the write.
     */
    static bool calibration(const TsensCalibration& c) {
        if (enabled() || c.gain == 0u || c.gain > tsens_field_mask ||
            !tsens_value_fits(c.offset)) {
            return false;
        }
        write_calibration(c);
        cfg_.calibration = c;
        return true;
    }

    /// What the three registers actually hold, read back rather than
    /// remembered.
    static TsensCalibration calibration() {
        const uint32_t cal = regs().TSENS_CAL;
        return TsensCalibration{
            .gain = regs().TSENS_GAIN & tsens_field_mask,
            .offset = tsens_signed(regs().TSENS_OFFSET),
            .tcal = static_cast<uint8_t>((cal & TSENS_CAL_TCAL_Msk) >>
                                         TSENS_CAL_TCAL_Pos),
            .fcal = static_cast<uint8_t>((cal & TSENS_CAL_FCAL_Msk) >>
                                         TSENS_CAL_FCAL_Pos),
        };
    }

    /// The OFFSET in force, signed - the pivot `tsens_rescale()` needs.
    static int32_t offset() { return tsens_signed(regs().TSENS_OFFSET); }
    /// The GAIN in force.
    static uint32_t gain() { return regs().TSENS_GAIN & tsens_field_mask; }

    // ---- measuring -----------------------------------------------------------

    /**
     * CTRLB.START: begin one measurement.
     *
     * A plain store, and there is nothing to read back: CTRLB is `__O` in
     * the device header and W in 43.8.2, and this block has no BUSY bit.
     * INTFLAG.RESRDY is the only evidence a measurement finished.
     *
     * ERRATUM 1.19.1 IS ABOUT THIS STORE: under PAC write protection it
     * silently does nothing. Nothing in brio enables that protection -
     * see this file's header.
     */
    static void start() { regs().TSENS_CTRLB = TSENS_CTRLB_START_Msk; }

    static uint8_t flags() { return regs().TSENS_INTFLAG; }
    static void clear_flags(uint8_t mask = flag_all) {
        regs().TSENS_INTFLAG = mask;
    }
    static void arm(uint8_t mask) { regs().TSENS_INTENSET = mask; }
    static void disarm(uint8_t mask) { regs().TSENS_INTENCLR = mask; }
    static uint8_t armed() { return regs().TSENS_INTENSET; }

    /// INTFLAG.RESRDY: a result is waiting. Cleared by writing a one OR
    /// by reading VALUE (43.8.7). It DOES NOT SET when the conversion
    /// overflowed - which is why `measure()` watches both flags.
    static bool result_ready() { return (flags() & flag_result_ready) != 0u; }
    /// INTFLAG.OVERRUN: a valid result was overwritten before the CPU
    /// read the previous one. Cleared only by writing a one.
    static bool overrun() { return (flags() & flag_overrun) != 0u; }
    /// INTFLAG.WINMON: the window condition matched. Cleared by writing a
    /// one OR by reading VALUE.
    static bool window_hit() { return (flags() & flag_window) != 0u; }
    /// INTFLAG.OVF: the result did not fit VALUE. Cleared only by
    /// writing a one.
    static bool overflow_flag() { return (flags() & flag_overflow) != 0u; }

    /// STATUS.OVF (43.8.8), the LEVEL beside INTFLAG's latch: the result
    /// currently in VALUE is not valid. Read-only, and writing it has no
    /// effect - unlike almost every other STATUS in this stratum.
    static bool overflowed() {
        return (regs().TSENS_STATUS & TSENS_STATUS_OVF_Msk) != 0u;
    }

    /// The VALUE register, raw.
    static uint32_t value_raw() { return regs().TSENS_VALUE & tsens_field_mask; }
    /// VALUE as the signed 24-bit number 43.8.10 says it is. Reading it
    /// clears RESRDY and WINMON.
    static int32_t value() { return tsens_signed(regs().TSENS_VALUE); }

    /**
     * Wait for a result and return it - the signed VALUE, which with the
     * factory calibration at 48 MHz IS a temperature in hundredths of a
     * degree Celsius.
     *
     * Nothing on a timeout and nothing on an overflow: 43.8.8 says an
     * overflowed VALUE "is not valid", and returning it would be
     * returning a number that means nothing. The overflow FLAG is left
     * standing for the caller to see; `clear_flags()` clears it.
     */
    static std::optional<int32_t> read(uint32_t spins = 2'000'000UL) {
        while (spins-- != 0u) {
            const uint8_t f = flags();
            if ((f & flag_overflow) != 0u) {
                return std::nullopt;
            }
            if ((f & flag_result_ready) != 0u) {
                if (overflowed()) {
                    return std::nullopt;
                }
                return value();
            }
        }
        return std::nullopt;
    }

    /// One measurement, start to finish. Not for a free-running block -
    /// there `read()` alone is the verb.
    static std::optional<int32_t> measure(uint32_t spins = 2'000'000UL) {
        if (!enabled()) {
            return std::nullopt;
        }
        clear_flags(flag_all);
        start();
        return read(spins);
    }

    /**
     * The chapter's own recommendation as a verb: "to prevent any
     * discrepancies in the temperature measurement, an average on 10
     * measurements is recommended" (43.6.2.3), which is also the
     * condition table 45-37's accuracy figures are taken under.
     *
     * The sum is 64-bit and the rounding is half-away-from-zero, so a
     * negative average does not drift toward zero. Nothing if any one
     * measurement failed - a partial average would be a quieter lie than
     * a missing one.
     */
    static std::optional<int32_t> measure_average(uint8_t n = 10,
                                                  uint32_t spins = 2'000'000UL) {
        if (n == 0u) {
            return std::nullopt;
        }
        int64_t sum = 0;
        for (uint8_t i = 0; i < n; ++i) {
            const auto v = measure(spins);
            if (!v) {
                return std::nullopt;
            }
            sum += *v;
        }
        const int64_t half = n / 2;
        const int64_t rounded = sum >= 0 ? (sum + half) / n : (sum - half) / n;
        return static_cast<int32_t>(rounded);
    }

    /// A reading rescaled from the GCLK_TSENS rate it was taken at to the
    /// 48 MHz the factory calibration assumes, using the OFFSET in force
    /// as the pivot. `gclk_hz` is the caller's knowledge - this block
    /// measures against its clock and cannot know what that clock is.
    static int32_t rescaled(int32_t value_centi, uint32_t gclk_hz) {
        return tsens_rescale(value_centi, offset(), gclk_hz);
    }

    /**
     * The ISR body - ONE VECTOR for all four sources, so the app binds
     * TSENS_Handler once and dispatches on the returned mask.
     *
     * RESRDY and WINMON are NOT cleared here: reading VALUE is what
     * clears them, and reading VALUE is the whole point of the
     * interrupt. OVERRUN and OVF are cleared, because nothing else can
     * clear them and nothing else carries their information.
     */
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t pending =
            static_cast<uint8_t>(regs().TSENS_INTFLAG & regs().TSENS_INTENSET);
        const uint8_t sticky =
            static_cast<uint8_t>(pending & (flag_overrun | flag_overflow));
        if (sticky != 0u) {
            regs().TSENS_INTFLAG = sticky;
        }
        return pending;
    }

    // ---- the window monitor ---------------------------------------------------

    /**
     * CTRLC.WINMODE with its two thresholds. All three registers are
     * enable-protected (43.6.2.1), so this REFUSES while the block is
     * enabled - disable first, or use `init()`.
     */
    static bool window(TsensWindow mode, int32_t lower, int32_t upper) {
        TsensConfig probe = cfg_;
        probe.window = mode;
        probe.window_lower = lower;
        probe.window_upper = upper;
        if (enabled() || !tsens_config_valid(probe)) {
            return false;
        }
        regs().TSENS_WINLT = tsens_field(lower);
        regs().TSENS_WINUT = tsens_field(upper);
        regs().TSENS_CTRLC = control_c_word(probe);
        cfg_ = probe;
        return true;
    }
    static TsensWindow window() {
        return static_cast<TsensWindow>((regs().TSENS_CTRLC & TSENS_CTRLC_WINMODE_Msk) >>
                                        TSENS_CTRLC_WINMODE_Pos);
    }
    static int32_t window_lower() { return tsens_signed(regs().TSENS_WINLT); }
    static int32_t window_upper() { return tsens_signed(regs().TSENS_WINUT); }

    /// CTRLC.FREERUN alone - enable-protected like the rest of CTRLC.
    static bool free_running(bool on) {
        if (enabled()) {
            return false;
        }
        cfg_.free_running = on;
        regs().TSENS_CTRLC = control_c_word(cfg_);
        return true;
    }
    static bool free_running() {
        return (regs().TSENS_CTRLC & TSENS_CTRLC_FREERUN_Msk) != 0u;
    }

    // ---- events -----------------------------------------------------------------

    /// EVCTRL, enable-protected (43.6.2.1) and refused while running.
    static bool event_config(const TsensEventControl& e) {
        if (enabled() || !tsens_event_control_valid(e)) {
            return false;
        }
        regs().TSENS_EVCTRL = event_word(e);
        cfg_.events = e;
        return true;
    }
    static TsensEventControl event_config() {
        const uint8_t v = regs().TSENS_EVCTRL;
        return TsensEventControl{
            .start_in = (v & TSENS_EVCTRL_STARTEI_Msk) != 0u,
            .invert_start = (v & TSENS_EVCTRL_STARTINV_Msk) != 0u,
            .window_out = (v & TSENS_EVCTRL_WINEO_Msk) != 0u,
        };
    }

    /**
     * Route an EVSYS channel to the START user and turn EVCTRL.STARTEI
     * on - the two halves that must both happen, in one verb so neither
     * can be forgotten.
     *
     * NO PATH IS REFUSED, and that is the difference from samc/dac.hpp's
     * and samc/sdadc.hpp's identical-looking verbs: table 29-3 gives
     * user 0 "asynchronous, synchronous, and resynchronized paths",
     * where those two are asynchronous-only. The channel's own legality
     * (an edge on an asynchronous path, and the rest) is still EVSYS's
     * to judge and `Evsys::connect` judges it.
     */
    static bool start_on(uint8_t channel, const EventChannelConfig& cfg,
                         bool invert = false) {
        if (enabled()) {
            return false;
        }
        TsensEventControl e = event_config();
        e.start_in = true;
        e.invert_start = invert;
        if (!event_config(e)) {
            return false;
        }
        return Evsys::connect(start_event_user, channel, cfg);
    }

    /// Stop listening: the user disconnected and the input enable
    /// cleared. Needs the block disabled, for the same reason.
    static bool stop_events() {
        if (enabled()) {
            return false;
        }
        Evsys::disconnect(start_event_user);
        TsensEventControl e = event_config();
        e.start_in = false;
        e.invert_start = false;
        return event_config(e);
    }

private:
    static void write_calibration(const TsensCalibration& c) {
        regs().TSENS_GAIN = c.gain & tsens_field_mask;
        regs().TSENS_OFFSET = tsens_field(c.offset);
        regs().TSENS_CAL = c.cal_word();
    }

    static constexpr uint8_t control_c_word(const TsensConfig& c) {
        return static_cast<uint8_t>(
            (c.free_running ? TSENS_CTRLC_FREERUN_Msk : 0u) |
            TSENS_CTRLC_WINMODE(static_cast<uint8_t>(c.window)));
    }

    static constexpr uint8_t event_word(const TsensEventControl& e) {
        return static_cast<uint8_t>(
            (e.start_in ? TSENS_EVCTRL_STARTEI_Msk : 0u) |
            (e.invert_start ? TSENS_EVCTRL_STARTINV_Msk : 0u) |
            (e.window_out ? TSENS_EVCTRL_WINEO_Msk : 0u));
    }

    static inline TsensConfig cfg_{};
};

} // namespace brio
