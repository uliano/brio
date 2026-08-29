/*
 * dac.hpp
 *
 * The SAM C21's 10-bit digital-to-analog converter (DS60001479M ch. 41),
 * a whole small chapter in one monostate resource - `Dac`, not `Dac<n>`:
 * this family has exactly ONE instance on every variant, so an index
 * would be a parameter with a single legal value (the `Rtc` precedent in
 * this stratum, against `Adc<n>`'s two real instances).
 *
 *   brio::Dac::init(generator, brio::DacConfig{
 *       .reference = brio::DacRef::vddana,
 *       .external_output = true});          // the buffer drives PA02
 *   brio::Dac::set(512);                    // about VDDANA / 2
 *   brio::Dac::set_mv(2500, vdd_mv);        // or in millivolts
 *
 * ---------------------------------------------------------------------
 * WHAT THE SILICON DOES. One channel, 10 bits, up to 350 ksps
 * (table 45-31), converting DATA into a voltage between GND and the
 * selected reference - VOUT = DATA / 0x3FF x VREF (41.6.2.4). Two ways
 * to start a conversion (41.6.1): write DATA, or let a START event copy
 * DATABUF into DATA. Two outputs, independently enabled and usable at
 * once (41.6.8.1): the high-drive buffer on the VOUT pad
 * (CTRLB.EOEN) and the internal path to the AC, the ADC and the SDADC
 * (CTRLB.IOEN). There is NO conversion-done flag - "as there is no
 * automatic indication that a conversion is done, the sampling period
 * must be greater than or equal to the specified conversion time"
 * (41.6.2.4) - so STATUS.READY is about the STARTUP time (3 us,
 * table 45-31) and nothing else.
 *
 * THE FOUR REGISTER DISCIPLINES, spelled per register the way
 * samc/adc.hpp spells its chapter's:
 *
 * 1. ENABLE-PROTECTED - CTRLB, and 41.6.2.1 also names EVCTRL while
 *    41.8.3's own property line does NOT ("PAC Write-Protection" alone).
 *    THE TWO STATEMENTS DISAGREE; `event_config()` refuses while the
 *    converter is enabled, which is correct under either reading, and
 *    docs/samc/dac.md carries what the silicon actually does.
 * 2. WRITE-SYNCHRONIZED - CTRLA.SWRST, CTRLA.ENABLE, DATA, DATABUF
 *    (41.6.7). Nothing needs synchronization when READ. But DATA and
 *    DATABUF are not ordinary bus crossings and the chapter does not
 *    say so: SYNCBUSY.DATABUF stands until a START EVENT CONSUMES the
 *    value, and SYNCBUSY.DATA stands with it, so a DAC with no start
 *    event configured is left with both bits set by a single DATABUF
 *    write - and every later write to either register is discarded.
 *    That is why `buffer()` is a plain void store and `buffer_sync()`
 *    the separate verb, exactly as samc/adc.hpp's `select()` is (the
 *    two chapters have the same double-buffer shape and neither
 *    describes it).
 * 3. NEITHER - EVCTRL, INTENSET/CLR, INTFLAG, STATUS, and DBGCTRL,
 *    which 41.8.11 also says is not reset by a software reset.
 * 4. WRITE-ONLY - DATA and DATABUF are `__O` in the device header and
 *    drawn with W access in 41.8.8/41.8.9. THERE IS NO CODE READBACK,
 *    so `code()` reports what this driver last WROTE and says so.
 *
 * NOT PAC-PROTECTED, and it is the pair a DMA engine needs (41.5.8):
 * INTFLAG and DATABUF.
 *
 * ---------------------------------------------------------------------
 * THE REFERENCE, and a NAME THAT LIES IN THE DEVICE HEADER.
 *
 * CTRLB.REFSEL has three codes. 41.8.2 calls them INTREF ("supplied by
 * the bandgap, refer to SUPC.VREF.SEL for voltage level information"),
 * VDDANA and VREFA. The device header names the same three values
 * INT1V, AVCC and VREFP - INT1V being the SAM D21's fixed 1.0 V
 * internal reference, which THIS family does not have: here the
 * internal reference is the SUPC bandgap and its level is chosen in
 * another peripheral's register. `DacRef` follows the DATASHEET for
 * meaning and the header for symbols, and the enumerators carry the
 * disagreement; docs/samc/dac.md carries the measurement that settles
 * it.
 *
 * `DacRef` is this converter's OWN vocabulary and deliberately not
 * `brio::Ref`, which samc/adc.hpp owns for REFCTRL.REFSEL: on this
 * family there is no shared reference block at all - the ADC's
 * multiplexer has six codes, this one has three, the SDADC's are
 * different again - so a single enum would be a type no register
 * accepts (the ADC campaign's judgment call 1, taken there and followed
 * here).
 *
 * ---------------------------------------------------------------------
 * WHERE THE DATA SITS IN THE REGISTER is decided by two bits together
 * (table 41-1), which is why `dac_data_word()` exists and no caller
 * shifts by hand:
 *
 *   DITHER  LEFTADJ   the 10-bit code goes to   the dither bits
 *      0       0      DATA[9:0]                 -
 *      0       1      DATA[15:6]                -
 *      1       0      DATA[13:4]                DATA[3:0]
 *      1       1      DATA[15:6]                DATA[5:2]
 *
 * DITHERING (41.6.8.4) makes 16 sub-conversions of DATA[13:4] or
 * DATA[13:4] + 1 so the average carries 14 bits, and it is NOT a mode a
 * CPU-driven converter can use: the chapter requires a periodic START
 * event (EVCTRL.STARTEI = 1) generating sixteen events per value, with
 * DATABUF reloaded every sixteen. `dac_config_valid()` refuses dither
 * without the start event input for exactly that reason.
 *
 * ---------------------------------------------------------------------
 * MILLIVOLTS. `set_mv()` is a thin wrapper over util/analog.hpp's
 * `dac_code(mv, steps, ref_mv)` with `steps` = 1024, exactly as
 * avrdx/dac.hpp does - the arithmetic is target-independent and stays
 * there. NOTE the half-truth in that: 41.6.2.4's own formula divides by
 * 0x3FF (1023) and not by 1024, so the two conventions differ by
 * mv/ref of one LSB - nothing at the bottom of the range and one whole
 * LSB at full scale. Below full scale the difference is smaller than
 * this converter's own gain error (table 45-32: +/-8 mV typical against
 * a 5 mV LSB at a 5 V reference), which is why the bench declines to
 * decide between them and this comment says so instead.
 *
 * THE LINEAR RANGE IS NOT THE WHOLE RANGE. Table 45-30 gives it as
 * 0.05 V .. VDDANA - 0.05 V, so at a 5.1 V reference roughly the bottom
 * ten and the top ten codes are outside specification, and any straight
 * line fitted through them is fitted through a clipped end.
 *
 * ---------------------------------------------------------------------
 * ERRATA, DS80000740S, read on the E/G/J ROW at revision F.
 *
 *  - 1.8.9 DAC Output (device level, ALL REVISIONS): selecting the DAC
 *    output as the ADC's positive multiplexer input makes BOTH the DAC
 *    output and the reading noisy, and the workaround is to "wire the
 *    DAC VOUT pin externally to an ADC AINx pin input" and select that
 *    pad instead. ON THIS SILICON THAT WIRE HAS ZERO LENGTH: PA02 is
 *    DAC/VOUT and ADC0/AIN0 at once, so enabling CTRLB.EOEN and
 *    selecting `AnalogIn<Pin<'A',2>>` on ADC0 IS the workaround.
 *    `Dac::adc_input_pad_hint` names that pad and the bench measures
 *    what the internal channel costs.
 *  - 1.9.2 Standby Sleep Mode (ALL REVISIONS): with CTRLA.RUNSTDBY = 0
 *    and DATABUF written but not yet consumed, entering standby sets
 *    INTFLAG.EMPTY on the way out. The workaround is the caller's -
 *    ignore and clear EMPTY after a standby wake - and `clear_flags()`
 *    is how; a driver cannot know that a wake happened.
 *  - 1.8.10 (the DAC as the SDADC's reference) is live on this row too,
 *    and INAPPLICABLE HERE BY ABSENCE: there is no SDADC driver in this
 *    stratum. Its workaround, if one is ever written, is
 *    REFCTRL.ONREFBUF = 1 on the SDADC's side.
 *  - NOT this silicon: 1.9.1 (dithering with right-adjusted data giving
 *    16 LSB of INL) is REVISION B ONLY - the E/G/J row carries one X
 *    and it is under B - so `left_adjust` is not forced here, and the
 *    row is the trap the errata document sets over and over.
 *
 * ---------------------------------------------------------------------
 * NOT BUILT (docs/samc/dac.md carries the list): the SDADC side of
 * everything, and the sleep behaviour beyond the one CTRLA bit.
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
 * CTRLB.REFSEL (41.8.2). Three codes; 0x3 is Reserved.
 *
 * The names follow the DATASHEET's meanings. The device header's own
 * enumerator names are the SAM D21's and are stale here - see this
 * file's header comment.
 */
enum class DacRef : uint8_t {
    /// INTREF: the SUPC bandgap, whose LEVEL is SUPC.VREF.SEL
    /// (samc/supc.hpp's `VrefLevel`) and not a knob of this peripheral.
    /// THE DEVICE HEADER CALLS THIS VALUE `INT1V`, a fixed 1.0 V
    /// reference this family does not have; 41.8.2 is the authority on
    /// what it means and docs/samc/dac.md on what it measures.
    intref = DAC_CTRLB_REFSEL_INT1V_Val,
    /// The analog supply. The header calls it AVCC.
    vddana = DAC_CTRLB_REFSEL_AVCC_Val,
    /// The VREFA pin (PA03 on this family). The header calls it VREFP.
    /// Table 45-30 bounds it at 1 V .. VDDANA - 0.6 V.
    vrefa = DAC_CTRLB_REFSEL_VREFP_Val,
};

/// Whether a REFSEL code is one the silicon implements (0x3 is
/// Reserved).
constexpr bool dac_ref_valid(DacRef r) {
    return static_cast<uint8_t>(r) <= static_cast<uint8_t>(DacRef::vrefa);
}

/**
 * The millivolts of a reference, for util/analog.hpp's arithmetic.
 *
 * `known_mv` is the millivolts of the reference's SOURCE where this
 * header cannot know it - VDDANA for `vddana`, the pin's voltage for
 * `vrefa` - and zero means "not known", which yields zero rather than a
 * plausible lie. `intref_level` is what stands in SUPC.VREF.SEL.
 */
constexpr uint16_t dac_ref_mv(DacRef r, uint16_t known_mv = 0,
                              VrefLevel intref_level = VrefLevel::v1_024) {
    switch (r) {
    case DacRef::intref: return vref_mv(intref_level);
    case DacRef::vddana:
    case DacRef::vrefa: return known_mv;
    }
    return 0;
}

// =============================================================================
// The configuration
// =============================================================================

/// EVCTRL (41.8.3), both directions in one struct.
struct DacEventControl {
    /// EVCTRL.STARTEI: an incoming event copies DATABUF into DATA and
    /// starts a conversion. TABLE 29-3 MARKS THE START USER
    /// ASYNCHRONOUS PATH ONLY, which `start_on()` enforces.
    bool start_in = false;
    /// EVCTRL.EMPTYEO: generate an event when DATABUF becomes empty.
    bool empty_out = false;
    /// EVCTRL.INVEI: detect the falling edge of the start event
    /// instead of the rising one.
    bool invert_start = false;
};

/// Inverting an input nobody listens to is a configuration with no
/// meaning - the same refusal samc/adc.hpp and samc/ac.hpp make.
constexpr bool dac_event_control_valid(const DacEventControl& e) {
    return !e.invert_start || e.start_in;
}

/// The whole configuration of the converter.
struct DacConfig {
    // -- CTRLB (enable-protected) --
    DacRef reference = DacRef::vddana;
    /// CTRLB.EOEN: the high-drive buffer drives the VOUT pad. 41.6.8.1
    /// ALSO claims it is required for the ADC to see the DAC at all
    /// ("the output buffer must be enabled") - MEASURED FALSE on this
    /// die: with IOEN alone the ADC's internal DAC channel reads the
    /// same 3057 counts it reads with both outputs on
    /// (docs/samc/dac.md). EOEN is for the PAD.
    bool external_output = false;
    /// CTRLB.IOEN: the internal output, which is what the AC's
    /// `AcNegative::dac` and the SDADC take.
    bool internal_output = false;
    /// CTRLB.LEFTADJ: where the ten bits sit in DATA (table 41-1).
    bool left_adjust = false;
    /// CTRLB.DITHER: 14-bit dithering, which needs a periodic START
    /// event and is refused without one (41.6.8.4).
    bool dither = false;
    /// CTRLB.VPD: the voltage pump is switched automatically by the
    /// operating voltage and this bit turns it off, which saves power
    /// ABOVE 2.5 V and breaks the converter below it (41.6.8.3). When
    /// the pump runs, GCLK_DAC must be at least four times the sampling
    /// rate - an obligation of the caller's, since this header knows
    /// neither rate.
    bool voltage_pump_disabled = false;

    // -- CTRLA (SWRST/ENABLE synchronized, RUNSTDBY not) --
    /// CTRLA.RUNSTDBY: the output buffer KEEPS ITS VALUE in standby
    /// instead of being disabled (41.6.6). Erratum 1.9.2 is about the
    /// other setting.
    bool run_standby = false;

    // -- DBGCTRL (survives a software reset) --
    bool debug_run = false;

    // -- EVCTRL --
    DacEventControl events{};
};

/**
 * What the compile-time form static_asserts and the runtime form
 * returns false for. Every rule is the chapter's.
 */
constexpr bool dac_config_valid(const DacConfig& c) {
    if (!dac_ref_valid(c.reference)) {
        return false;
    }
    // 41.6.8.4: dithering IS the event-driven mode - sixteen START
    // events per value, DATABUF reloaded every sixteen. Without
    // STARTEI there is no way to make the sixteen sub-conversions
    // happen, so the configuration asks for something the silicon
    // cannot do.
    if (c.dither && !c.events.start_in) {
        return false;
    }
    return dac_event_control_valid(c.events);
}

/// The full scale of the converter: ten bits.
inline constexpr uint32_t dac_steps = 1024;
/// The full scale of a DITHERED value: fourteen bits (41.6.8.4).
inline constexpr uint32_t dac_dither_steps = 16384;

/**
 * Where a value sits in DATA / DATABUF, from table 41-1 - the ONE place
 * the two adjustment bits are turned into a shift.
 *
 * `value` is a 10-bit code with dithering off and a 14-bit one with it
 * on; anything above the scale is clamped, so a caller cannot silently
 * write bits into a neighbouring field.
 */
constexpr uint16_t dac_data_word(uint16_t value, bool left_adjust, bool dither) {
    const uint32_t scale = dither ? dac_dither_steps : dac_steps;
    uint32_t v = value;
    if (v >= scale) {
        v = scale - 1u;
    }
    if (!dither) {
        return static_cast<uint16_t>(left_adjust ? (v << 6u) : v);
    }
    // 14 bits: [13:4] + [3:0] right-adjusted, [15:2] left-adjusted.
    return static_cast<uint16_t>(left_adjust ? (v << 2u) : v);
}

/// One conversion's worth of time at the chapter's own ceiling
/// (table 45-31: 350 ksps in normal mode), in nanoseconds. There is no
/// done flag, so this is what a caller must leave between values.
inline constexpr uint32_t dac_conversion_ns = 2857;
/// Table 45-31's startup time, which STATUS.READY reports the end of.
inline constexpr uint32_t dac_startup_ns = 3000;

// =============================================================================
// The converter
// =============================================================================

/**
 * The one DAC. A monostate resource: this family has a single instance
 * on every variant, so there is no index to carry.
 */
class Dac {
    static_assert(dac_count() == 1u,
                  "this device has no DAC (chapter 41 is SAM C21 only)");

public:
    Dac() = delete;

    static constexpr uint32_t steps = dac_steps;             ///< 10 bits
    static constexpr uint32_t dither_steps = dac_dither_steps;
    static constexpr uint8_t gclk_id = dac_gclk_id();

    static constexpr IRQn_Type irq() { return DAC_IRQn; }

    // ---- the vocabularies this peripheral publishes -------------------------
    //
    // evsys.hpp owns the FABRIC and dmac.hpp owns the CHANNELS; the
    // codes of their tables that belong to the DAC live here, probed
    // from the device header in samc/device_tables.hpp.

    /// Generator: DATABUF became empty (41.6.5).
    static constexpr uint8_t empty_generator = dac_empty_generator();
    /// User: copy DATABUF into DATA and convert. TABLE 29-3 MARKS IT
    /// ASYNCHRONOUS PATH ONLY - `start_on()` enforces it.
    static constexpr uint8_t start_event_user = dac_start_user();
    /// DMAC trigger: the one DMA request this peripheral has (41.6.3).
    static constexpr uint8_t dma_trigger_empty = dac_dma_empty_id();

    /// INTFLAG / INTENSET bits, named.
    static constexpr uint8_t flag_underrun = DAC_INTFLAG_UNDERRUN_Msk;
    static constexpr uint8_t flag_empty = DAC_INTFLAG_EMPTY_Msk;

    static dac_registers_t& regs() { return *DAC_REGS; }

    // ---- the pads -----------------------------------------------------------

    /// Which peripheral function a pad needs to be the analog output, or
    /// -1 if it is not the VOUT pad on this package.
    static constexpr int vout_function(char port, uint8_t pin) {
        return dac_vout_code(port, pin);
    }
    /// The same for the external reference pin (VREFA / VREFP).
    static constexpr int vrefa_function(char port, uint8_t pin) {
        return dac_vrefa_code(port, pin);
    }

    /**
     * THE ZERO-LENGTH WIRE erratum 1.8.9's workaround asks for: on this
     * family VOUT and ADC0/AIN0 are the SAME PAD (PA02), so an ADC that
     * wants the DAC's voltage should enable CTRLB.EOEN and select this
     * pad rather than MUXPOS = DAC. Published as data so a caller does
     * not have to know the pinout.
     */
    static constexpr char adc_input_pad_port = 'A';
    static constexpr uint8_t adc_input_pad_pin = 2;

    /// Hand the VOUT pad to the DAC. 41.5.1 asks for the pad to be
    /// configured in PORT; the peripheral function comes from the
    /// reserve's own probe, so nothing here names a mux letter.
    template <typename P>
    static void claim_vout() {
        static_assert(dac_vout_code(P::port_letter, P::pin_number) >= 0,
                      "this package does not bond that pad as the DAC's "
                      "analog output (VOUT is PA02 on this family)");
        P::function(static_cast<PinFunction>(
                        dac_vout_code(P::port_letter, P::pin_number)),
                    PinConfig{});
    }
    template <typename P>
    static void claim_vrefa() {
        static_assert(dac_vrefa_code(P::port_letter, P::pin_number) >= 0,
                      "this package does not bond that pad as the DAC's "
                      "external reference (VREFA is PA03 on this family)");
        P::function(static_cast<PinFunction>(
                        dac_vrefa_code(P::port_letter, P::pin_number)),
                    PinConfig{});
    }
    template <typename P>
    static void release_pad() { P::release(); }

    // ---- claim and teardown -------------------------------------------------

    static void bus_clock(bool on) { Mclk::apb_c(MCLK_APBCMASK_DAC_Msk, on); }

    static bool clock(uint8_t generator, uint32_t spins = 0xFFFFu) {
        return GclkChannel::connect(gclk_id, generator, spins);
    }

    /// Bounded spin on SYNCBUSY, like every wait in this stratum.
    static bool sync_wait(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().DAC_SYNCBUSY, mask, false, spins);
    }
    static uint32_t sync_busy() { return regs().DAC_SYNCBUSY; }

    /// CTRLA.SWRST: every register except DBGCTRL back to reset, and
    /// the converter disabled.
    static bool reset(uint32_t spins = 0xFFFFu) {
        regs().DAC_CTRLA = DAC_CTRLA_SWRST_Msk;
        return sync_wait(DAC_SYNCBUSY_SWRST_Msk, spins);
    }

    /// CTRLA.ENABLE, preserving RUNSTDBY.
    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint8_t v =
            static_cast<uint8_t>(regs().DAC_CTRLA & ~DAC_CTRLA_ENABLE_Msk);
        regs().DAC_CTRLA =
            on ? static_cast<uint8_t>(v | DAC_CTRLA_ENABLE_Msk) : v;
        return sync_wait(DAC_SYNCBUSY_ENABLE_Msk, spins);
    }
    static bool enabled() {
        return (regs().DAC_CTRLA & DAC_CTRLA_ENABLE_Msk) != 0u;
    }

    // ---- configuration ------------------------------------------------------

    /**
     * The whole chapter in one call: bus clock, generic clock, reset,
     * the enable-protected registers, the ones that are not, enable,
     * and the startup wait STATUS.READY reports the end of.
     *
     * The converter comes up holding code 0 - DATA's reset value - and
     * the first `set()` is what puts a voltage on the pad.
     */
    static bool init(uint8_t generator, const DacConfig& cfg,
                     uint32_t spins = 0xFFFFu) {
        if (!dac_config_valid(cfg)) {
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
        regs().DAC_CTRLB = control_b_word(cfg);
        regs().DAC_EVCTRL = event_word(cfg.events);
        regs().DAC_DBGCTRL = cfg.debug_run ? DAC_DBGCTRL_DBGRUN_Msk : 0u;
        regs().DAC_INTENCLR = flag_empty | flag_underrun;
        regs().DAC_INTFLAG = flag_empty | flag_underrun;

        regs().DAC_CTRLA =
            static_cast<uint8_t>(cfg.run_standby ? DAC_CTRLA_RUNSTDBY_Msk : 0u);
        if (!enable(true, spins)) {
            return false;
        }
        cfg_ = cfg;
        code_ = 0;
        return wait_ready(spins);
    }

    /// The compile-time twin: every rule of `dac_config_valid()` becomes
    /// a compile error instead of a false return.
    template <DacConfig cfg>
    static bool init(uint8_t generator, uint32_t spins = 0xFFFFu) {
        static_assert(dac_config_valid(cfg),
                      "brio DacConfig: see dac_config_valid() - the Reserved "
                      "REFSEL code, dithering without the START event input "
                      "(41.6.8.4), or an inverted event input nothing listens "
                      "to");
        return init(generator, cfg, spins);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        Nvic::disable(irq());
        (void)reset(spins);
        GclkChannel::disconnect(gclk_id);
        bus_clock(false);
    }

    /// The configuration in force, as `init()` was given it.
    static const DacConfig& config() { return cfg_; }
    static DacRef reference() { return cfg_.reference; }

    /**
     * CTRLB under a running converter - which is REFUSED, because
     * 41.8.2 marks the register enable-protected and a store into it
     * would be silently dropped. Disable first, or use `init()`.
     */
    static bool control_b(const DacConfig& cfg) {
        if (enabled() || !dac_config_valid(cfg)) {
            return false;
        }
        regs().DAC_CTRLB = control_b_word(cfg);
        cfg_ = cfg;
        return true;
    }
    static uint8_t control_b() { return regs().DAC_CTRLB; }

    /// CTRLB.EOEN alone - the one CTRLB bit an application plausibly
    /// wants to move on its own (41.6.2.3: "the output buffer should be
    /// enabled only when external output is needed"). Enable-protected
    /// like the rest of the register, so it refuses while running.
    static bool external_output(bool on) {
        if (enabled()) {
            return false;
        }
        const uint8_t v =
            static_cast<uint8_t>(regs().DAC_CTRLB & ~DAC_CTRLB_EOEN_Msk);
        regs().DAC_CTRLB =
            on ? static_cast<uint8_t>(v | DAC_CTRLB_EOEN_Msk) : v;
        cfg_.external_output = on;
        return true;
    }
    static bool external_output() {
        return (regs().DAC_CTRLB & DAC_CTRLB_EOEN_Msk) != 0u;
    }
    static bool internal_output() {
        return (regs().DAC_CTRLB & DAC_CTRLB_IOEN_Msk) != 0u;
    }

    // ---- the startup ---------------------------------------------------------

    /// STATUS.READY: the startup time (3 us, table 45-31) has passed and
    /// the converter can convert. It is NOT a conversion-done flag -
    /// 41.6.2.4 says there is no such indication.
    static bool ready() { return (regs().DAC_STATUS & DAC_STATUS_READY_Msk) != 0u; }
    static bool wait_ready(uint32_t spins = 0xFFFFu) {
        while (spins-- != 0u) {
            if (ready()) {
                return true;
            }
        }
        return ready();
    }

    // ---- the data path -------------------------------------------------------

    /**
     * Write DATA: the conversion starts as soon as the value lands
     * (41.6.2.4). Write-synchronized, and the wait is bounded and
     * reported.
     *
     * `value` is a 10-bit code, or a 14-bit one when dithering is
     * configured; `dac_data_word()` places and clamps it.
     *
     * FALSE MEANS THE WRITE WAS DISCARDED, and there is one way to
     * arrange that: a value standing in DATABUF that no start event has
     * taken (see `buffer()`) holds SYNCBUSY.DATA up as well, and
     * 41.6.7 discards a synchronized write made while its busy bit is
     * one. `buffer_pending()` is how to ask before writing.
     */
    static bool set(uint16_t value, uint32_t spins = 0xFFFFu) {
        regs().DAC_DATA = dac_data_word(value, cfg_.left_adjust, cfg_.dither);
        code_ = value;
        return sync_wait(DAC_SYNCBUSY_DATA_Msk, spins);
    }

    /**
     * Write DATABUF: the value waits there until a START event copies it
     * into DATA (41.6.8.2). Writing it also clears INTFLAG.EMPTY. NOT
     * PAC-protected, which is what lets a DMA channel feed it.
     *
     * VOID AND NO WAIT, DELIBERATELY, and this is measured rather than
     * deduced: SYNCBUSY.DATABUF is not a bus crossing that finishes on
     * its own. It stands until the value is CONSUMED by a start event,
     * and SYNCBUSY.DATA stands with it - so in a DAC with no start
     * event configured, one DATABUF write leaves both bits set forever
     * and every later write to either register is DISCARDED (41.6.7).
     * A caller that spun here would be waiting for an event, not for a
     * bus. This is the same shape samc/adc.hpp's `select()` has, for
     * the same reason; `buffer_sync()` is the verb for a caller that
     * knows a consumer exists.
     */
    static void buffer(uint16_t value) {
        regs().DAC_DATABUF = dac_data_word(value, cfg_.left_adjust, cfg_.dither);
    }

    /// DATABUF with the wait, for a caller whose start events are
    /// running: false means the value has not been consumed yet.
    static bool buffer_sync(uint16_t value, uint32_t spins = 0xFFFFu) {
        buffer(value);
        return sync_wait(DAC_SYNCBUSY_DATABUF_Msk, spins);
    }

    /// SYNCBUSY.DATABUF: a value is standing in the buffer that no start
    /// event has taken yet. While it stands, writes to DATA and DATABUF
    /// are discarded - only a start event or a reset clears it.
    static bool buffer_pending() {
        return (regs().DAC_SYNCBUSY & DAC_SYNCBUSY_DATABUF_Msk) != 0u;
    }

    /**
     * The code this driver last WROTE, and not a readback: DATA and
     * DATABUF are write-only in both the register description and the
     * device header, so the silicon offers no way to ask.
     */
    static uint16_t code() { return code_; }

    /// A voltage instead of a code, through util/analog.hpp's
    /// arithmetic - see this file's header on the 1023-vs-1024 question
    /// the bench cannot settle. `reference_mv` is what the reference
    /// really is (`dac_ref_mv()` knows it for INTREF; VDDANA and VREFA
    /// are the caller's to measure).
    static bool set_mv(uint16_t mv, uint16_t reference_mv,
                       uint32_t spins = 0xFFFFu) {
        return set(dac_code(mv, steps, reference_mv), spins);
    }

    /// The voltage a code aims at, the inverse of `set_mv()`.
    static uint16_t code_mv(uint16_t value, uint16_t reference_mv) {
        return dac_mv(value, steps, reference_mv);
    }

    // ---- interrupts ----------------------------------------------------------

    static void arm(uint8_t mask) { regs().DAC_INTENSET = mask; }
    static void disarm(uint8_t mask) { regs().DAC_INTENCLR = mask; }
    static uint8_t armed() { return regs().DAC_INTENSET; }
    static uint8_t flags() { return regs().DAC_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().DAC_INTFLAG = mask; }

    /// INTFLAG.EMPTY: DATABUF has been consumed and can take a new
    /// value. Cleared by writing a one OR by writing DATABUF (41.8.6).
    /// ERRATUM 1.9.2 sets it spuriously across a standby entry made
    /// with RUNSTDBY = 0 and DATABUF full - after such a wake, clear it
    /// and do not believe it.
    static bool empty() { return (flags() & flag_empty) != 0u; }
    /// INTFLAG.UNDERRUN: a START event arrived with DATABUF empty.
    /// Cleared only by writing a one.
    static bool underrun() { return (flags() & flag_underrun) != 0u; }

    /**
     * The ISR body - ONE VECTOR for both sources, so the app binds
     * DAC_Handler once and dispatches on the returned mask.
     *
     * UNDERRUN is cleared here (nothing else can clear it and nothing
     * else carries its information); EMPTY is NOT, because writing
     * DATABUF is what clears it and putting a value there is the whole
     * point of the interrupt. A handler that returns without feeding
     * the buffer will be called again.
     */
    [[gnu::always_inline]] static uint8_t isr() {
        const uint8_t pending =
            static_cast<uint8_t>(regs().DAC_INTFLAG & regs().DAC_INTENSET);
        if ((pending & flag_underrun) != 0u) {
            regs().DAC_INTFLAG = flag_underrun;
        }
        return pending;
    }

    // ---- events ---------------------------------------------------------------

    /**
     * EVCTRL, and the one place this chapter contradicts itself:
     * 41.6.2.1 lists EVCTRL among the enable-protected registers while
     * 41.8.3's own property line says only "PAC Write-Protection". This
     * REFUSES while the converter is enabled, which is correct under
     * either reading - and docs/samc/dac.md carries what the silicon
     * does when asked directly.
     */
    static bool event_config(const DacEventControl& e) {
        if (enabled() || !dac_event_control_valid(e)) {
            return false;
        }
        regs().DAC_EVCTRL = event_word(e);
        cfg_.events = e;
        return true;
    }
    static DacEventControl event_config() {
        const uint8_t v = regs().DAC_EVCTRL;
        return DacEventControl{
            .start_in = (v & DAC_EVCTRL_STARTEI_Msk) != 0u,
            .empty_out = (v & DAC_EVCTRL_EMPTYEO_Msk) != 0u,
            .invert_start = (v & DAC_EVCTRL_INVEI_Msk) != 0u,
        };
    }

    /**
     * Route an EVSYS channel to the START user and turn EVCTRL.STARTEI
     * on - the two halves that must both happen, in one verb so neither
     * can be forgotten.
     *
     * IT REFUSES A CHANNEL THAT IS NOT ASYNCHRONOUS: table 29-3 marks
     * the DAC START user asynchronous-path-only, and 41.6.5 explains
     * why - the event is resynchronized inside the DAC Controller, so
     * the fabric must not try to do it first.
     */
    static bool start_on(uint8_t channel, const EventChannelConfig& cfg,
                         bool invert = false) {
        if (cfg.path != EventPath::asynchronous || enabled()) {
            return false;
        }
        DacEventControl e = event_config();
        e.start_in = true;
        e.invert_start = invert;
        if (!event_config(e)) {
            return false;
        }
        return Evsys::connect(start_event_user, channel, cfg);
    }

    /// Stop listening: the user disconnected and the input enable
    /// cleared. Needs the converter disabled, for the same reason.
    static bool stop_events() {
        if (enabled()) {
            return false;
        }
        Evsys::disconnect(start_event_user);
        DacEventControl e = event_config();
        e.start_in = false;
        e.invert_start = false;
        return event_config(e);
    }

private:
    static constexpr uint8_t control_b_word(const DacConfig& c) {
        return static_cast<uint8_t>(
            (c.external_output ? DAC_CTRLB_EOEN_Msk : 0u) |
            (c.internal_output ? DAC_CTRLB_IOEN_Msk : 0u) |
            (c.left_adjust ? DAC_CTRLB_LEFTADJ_Msk : 0u) |
            (c.voltage_pump_disabled ? DAC_CTRLB_VPD_Msk : 0u) |
            (c.dither ? DAC_CTRLB_DITHER_Msk : 0u) |
            DAC_CTRLB_REFSEL(static_cast<uint8_t>(c.reference)));
    }

    static constexpr uint8_t event_word(const DacEventControl& e) {
        return static_cast<uint8_t>((e.start_in ? DAC_EVCTRL_STARTEI_Msk : 0u) |
                                    (e.empty_out ? DAC_EVCTRL_EMPTYEO_Msk : 0u) |
                                    (e.invert_start ? DAC_EVCTRL_INVEI_Msk : 0u));
    }

    static inline DacConfig cfg_{};
    static inline uint16_t code_ = 0;
};

} // namespace brio
