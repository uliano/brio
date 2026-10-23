/*
 * adc.hpp
 *
 * Adc<n>: voltages measured - a sensor on a pin, a differential pair,
 * the supply, the die's temperature, a stream of samples paced by a
 * timer - with the analog-to-digital converter of an AVR DA, DB or EA.
 * A program configures the one converter once with an AdcConfig, names
 * an input, and either reads it (blocking) or lets the RESRDY interrupt
 * hand each result to an active object (a component of a brio program
 * with its own event queue, served one event at a time by the kernel).
 * The smallest use, a blocking reading of PD1 on the DA/DB:
 *
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *     using Meter = brio::Adc<0>;
 *     using Wire = brio::AnalogIn<brio::Pin<'D', 1>>;            // AIN1
 *
 *     SysClock::init();                                          // first in main()
 *     Meter::init<brio::AdcConfig{.reference = brio::Ref::v2048}>(clock);   // checked at compile time
 *     Meter::select(Wire{});
 *     const uint16_t raw = Meter::read();                        // start, wait, read
 *
 * On the AVR EA (its board's 20 MHz crystal, CLK_ADC 2 MHz by default)
 * the same reading states the sampling an internal reference needs,
 * which init<cfg>() refuses to go without, and claims the pad once:
 *
 *     Meter::init<brio::AdcConfig{.reference = brio::Ref::v2048,
 *                                 .sample_duration = 8}>(clock);   // 4 us x 2 MHz
 *     Meter::claim(Wire{});
 *     Meter::select(Wire{});
 *     const uint16_t raw = Meter::read();
 *
 * Results as events: the program binds the vector and posts what the
 * interrupt body returns (the ISR condenses, the active object decides;
 * Monitor is the program's active object, AdcResult its event), the
 * interrupt enabled by enable_resrdy_interrupt(true) before sei():
 *
 *     ISR(ADC0_RESRDY_vect) { brio::post<Monitor>(AdcResult{Meter::resrdy()}); }
 *
 * Which context calls what. init() (both forms), reconfigure(),
 * disable(), the window and interrupt-enable verbs, start_on() and
 * start_on_events() are setup verbs of the converter's one owner:
 * main() after the clock task, or the handler of the active object that
 * owns the converter. The interrupt-enable verbs are a read-modify-write
 * of INTCTRL with no guard, so they are not called from an interrupt
 * body while the owner configures. resrdy(), wcmp() (the DA/DB),
 * samprdy() and error() (the EA) are interrupt bodies, called from the
 * program's handler for their vector; error() is also the polled form.
 * Nothing in this file masks interrupts. What waits: the DA/DB's init()
 * waits 10 us of warm-up (delay_us on the clock's rate), the EA's waits
 * for nothing; read(), read_sum() (the EA) and flush() (the DA/DB) spin
 * until RESRDY with no time-out, a conversion's length on an enabled
 * converter (on the EA plus the initialization table 31-5 times, up to
 * 60 us with LOWLAT off), and a disabled converter never raises RESRDY;
 * reconfigure() and the DA/DB's rebase() spin on busy() until the
 * converter is idle; the EA's select under an event pace spins the
 * output latency, 36 CLK_PER at most, with interrupts on. One converter, one owner: the window match a
 * result read captures, and on the EA the TEMP registers its 16- and
 * 32-bit registers share, are state for one context at a time. The
 * driver holds no buffer.
 *
 * Adc<n> is the analog-to-digital converter of the stratum as one brio
 * task with knobs (a task: a driver layer with state of its own over a
 * resource, served from its peripheral's interrupt or called by an
 * active object) - see docs/avrxt/adc.md for why not several: every use
 * (single reading, paced stream, oversampled reading, window watch,
 * temperature, supply monitor) is the same sequence - select input,
 * trigger, wait for the result, read it - with different knobs and
 * different consumers of the same events. So: one type, a config struct
 * that owns the whole configuration, inputs as types, the ISR bodies for
 * the app to bind and post, and event start through the event system.
 * Ownership of the one converter belongs to one active object.
 *
 * Two converters, two engines, one file. The AVR DA and DB carry a
 * 12-bit SAR ADC with a 16-bit result register and a buffered input mux
 * (DS40002247B ch. 33); the AVR EA carries the 12-bit differential ADC
 * with a programmable gain amplifier (DS40002443A ch. 31): one COMMAND
 * store starts a conversion and carries its mode, a 16-bit SAMPLE and a
 * 32-bit RESULT, burst and series accumulation to 1024 samples, start-up
 * timed in hardware against CLKCTRL.MCLKTIMEBASE, its own reference
 * selection, and an unbuffered input mux. The two share the surface
 * where the operation is the same (init<cfg>/init(cfg), select, start,
 * read, the window, the ISR bodies, event start, rebase) and nothing of
 * the engine; each half below is written from its own chapter, and the
 * part's header decides which half compiles (ADC_PGAEN_bm is the EA's
 * own bit). docs/avrxt/adc.md holds both halves' findings.
 *
 * The config comes in two forms with one implementation: init<cfg>()
 * (the struct as a template argument: static_asserts on the knobs,
 * every register write folded) and init(cfg) (a run-time value: same
 * writes computed on the fly - reconfigure from a console command;
 * false, touching nothing, for what the template form refuses). Both
 * take the clock tag: the prescaler's range (and the DA/DB's warm-up
 * wait) is computed from the rate the clock type claims, so init() comes
 * after the clock task's, and a DynamicClock that does not list this
 * Adc among its users fails to compile at init().
 *
 * Event start: Adc<0>::start_on(EventChannel<n>{}) makes the converter
 * a user of that channel (async, works in standby).
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrxt/delay.hpp"
#include "avrxt/device_tables.hpp"
#include "avrxt/evsys.hpp"
#include "util/clock.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/vref.hpp"
#include "util/analog.hpp"

#if !defined(ADC_PGAEN_bm)

// =============================================================================
// The AVR DA/DB: the 12-bit SAR ADC (DS40002247B ch. 33)
// =============================================================================
//
// Knobs, named as the datasheet names them (33.5): reference (VREF.
// ADC0REF), resolution 12/10 bit, differential, prescaler CLK_ADC =
// CLK_PER / {2..256} (keep CLK_ADC within 125 kHz .. 2 MHz; accuracy
// is specified at 500 kHz), sample_length (0-255 extra CLK_ADC cycles
// of sampling: high-impedance sources, the temperature sensor's 28 us),
// sample_delay (0-15, to move sampling off a noise harmonic),
// init_delay (0/16/../256 CLK_ADC cycles before the first sample after
// enable or standby: reference and warm-up settling, >= 6 us),
// accumulate 1..128 samples per result (hardware oversampling; RES is
// the sum, truncated to 16 bits above 16), left_adjust, free_running,
// run_standby. Conversion time = 2/fPER + n*(13.5 + 2 + sample_delay +
// sample_length)/fADC + 2/fPER (12-bit; 11.5 at 10 bits).
// reconfigure(cfg) is init(cfg) that first waits for an idle converter
// (the datasheet forbids changing RESSEL/CONVMODE/PRESC/SAMPNUM during
// a conversion, and free-running must be stopped first).
//
// Errata DS80000915F: 2.3.2 (all silicon) - with init_delay != 0, a
// MUXPOS/MUXNEG/accumulate change made after enabling the ADC or after
// a reference change takes effect only after one conversion. init()
// writes the mux before enabling, so the first configured input is
// right; a later select() with init_delay != 0 must be followed by a
// throw-away conversion - flush() does exactly that, blocking. 2.3.1
// (A4): -3 mV typical single-ended offset.
//
// The clock: CLK_ADC = CLK_PER / prescaler must stay within 125 kHz ..
// 2 MHz - init() refuses (compile time for a static clock) a prescaler
// that does not. Adc is a ClockUser (a driver a DynamicClock tells of
// every rate change through its rebase(hz)): on a DynamicClock change
// rebase() re-picks the prescaler (stopped, idle, rewritten,
// free-running resumed); the owner pauses event-started conversions
// around the switch. The intent is the CLK_ADC chosen at init; the code
// aims at the one in effect before the switch (clk_adc_hz_ is rewritten
// by every rebase), so a round trip of the clock can leave it slower -
// a gap docs/avrxt/adc.md lists.
//
// Bench facts (the AVR DB's silicon A5): the converter needs its warm-up (t_ADC_INIT
// 6 us typ.) after ENABLE before the first conversion is trustworthy -
// init(clock, cfg) waits for it; INITDLY is paid only for the first
// conversion after enable (not per start); the unbuffered
// DAC0 input (MUXPOS DAC0) is high-impedance and reads 3-4 % low with
// the default 2-cycle sampling - give it sample_length (16-32 at
// 1.5 MHz) like any source above 10 kOhm; the WCMP flag is cleared by
// reading RES (33.5.12) - result() captures it first, window_hit()
// tells about the last result read.

namespace brio {

// ---- the knobs ------------------------------------------------------------------

enum class AdcRes : uint8_t { bits12, bits10 };

enum class AdcPresc : uint8_t {
    div2, div4, div8, div12, div16, div20, div24, div28,
    div32, div48, div64, div96, div128, div256
};

constexpr uint16_t adc_presc_divisor(AdcPresc p) {
    constexpr uint16_t d[] = {2, 4, 8, 12, 16, 20, 24, 28, 32, 48, 64, 96, 128, 256};
    return d[static_cast<uint8_t>(p)];
}

enum class AdcInitDelay : uint8_t { none, cycles16, cycles32, cycles64, cycles128, cycles256 };

constexpr uint16_t adc_init_delay_cycles(AdcInitDelay d) {
    constexpr uint16_t c[] = {0, 16, 32, 64, 128, 256};
    return c[static_cast<uint8_t>(d)];
}

struct AdcConfig {
    Ref reference = Ref::vdd;
    bool reference_always_on = false;
    AdcRes resolution = AdcRes::bits12;
    bool differential = false;
    AdcPresc prescaler = AdcPresc::div16;   ///< 24 MHz / 16 = 1.5 MHz
    uint8_t sample_length = 0;             ///< extra CLK_ADC cycles, 0..255
    uint8_t sample_delay = 0;              ///< 0..15 CLK_ADC cycles
    AdcInitDelay init_delay = AdcInitDelay::none;
    uint8_t accumulate = 1;                ///< 1, 2, 4, ... 128 samples per result
    bool left_adjust = false;
    bool free_running = false;
    bool run_standby = false;
    bool debug_run = false;        ///< DBGRUN: keep converting while the CPU is halted
};

/// What the compile-time form static_asserts and the run-time form
/// returns false for.
constexpr bool adc_config_valid(const AdcConfig& c) {
    const uint8_t a = c.accumulate;
    const bool acc_ok = a != 0 && (a & (a - 1)) == 0 && a <= 128;
    return acc_ok && c.sample_delay <= 15;
}

/// CLK_ADC in Hz for a peripheral clock and a prescaler.
constexpr uint32_t adc_clock_hz(uint32_t clk_per_hz, AdcPresc p) {
    return clk_per_hz / adc_presc_divisor(p);
}

/// The CLK_ADC range the silicon supports (TCLK_ADC 0.5 .. 8 us).
inline constexpr uint32_t adc_clock_min_hz = 125'000;
inline constexpr uint32_t adc_clock_max_hz = 2'000'000;

constexpr bool adc_clock_in_range(uint32_t clk_per_hz, AdcPresc p) {
    const uint32_t f = adc_clock_hz(clk_per_hz, p);
    return f >= adc_clock_min_hz && f <= adc_clock_max_hz;
}

/// The prescaler giving the CLK_ADC closest to `target_hz` at
/// `clk_per_hz`, within range (the nearest in-range one otherwise).
constexpr AdcPresc adc_presc_for(uint32_t clk_per_hz, uint32_t target_hz) {
    AdcPresc best = AdcPresc::div256;
    uint32_t best_err = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 14; ++i) {
        const AdcPresc p = static_cast<AdcPresc>(i);
        if (!adc_clock_in_range(clk_per_hz, p)) continue;
        const uint32_t f = adc_clock_hz(clk_per_hz, p);
        const uint32_t err = f > target_hz ? f - target_hz : target_hz - f;
        if (err < best_err) { best_err = err; best = p; }
    }
    return best;
}

/// Total conversion time in CLK_PER cycles for a configuration (33.3.3.4).
constexpr uint32_t adc_conversion_cycles(const AdcConfig& c) {
    const uint32_t div = adc_presc_divisor(c.prescaler);
    const uint32_t per_sample_x2 = (c.resolution == AdcRes::bits12 ? 27u : 23u) +   // 13.5 / 11.5 doubled
                                   2u * (2u + c.sample_delay + c.sample_length);
    return 2u + (c.accumulate * per_sample_x2 * div + 1u) / 2u + 2u;
}

// ---- inputs as types -----------------------------------------------------------

/// Internal inputs (MUXPOS codes; MUXNEG accepts gnd and dac0 only).
enum class AdcInput : uint8_t {
    gnd = 0x40, temp = 0x42,
#ifdef MVIO
    // DB only: the DA reserves these mux codes (no MVIO there).
    vdd_div10 = 0x44, vddio2_div10 = 0x45,
#endif
    dac0 = 0x48, dacref0 = 0x49, dacref1 = 0x4A, dacref2 = 0x4B
};

/// The negatives the differential mux accepts among the internal
/// codes: GND and DAC0 (the AIN negatives go through AnalogIn pins).
constexpr bool adc_neg_valid(AdcInput in) {
    return in == AdcInput::gnd || in == AdcInput::dac0;
}

/// A pin as an analog input: AIN0-7 = PD0-7, AIN8-15 = PE0-7, AIN16-21
/// = PF0-5 (the device's mapping - a seed of the per-family table).
/// AIN16-21 cannot be a negative input.
template <typename P>
struct AnalogIn {
    static constexpr char port = P::port_letter;
    static constexpr uint8_t pin = P::pin_number;
    static_assert(port == 'D' || port == 'E' || (port == 'F' && pin <= 5),
                  "ADC inputs: PD0-7 (AIN0-7), PE0-7 (AIN8-15), PF0-5 (AIN16-21)");
    static constexpr uint8_t ain = static_cast<uint8_t>(
        port == 'D' ? pin : port == 'E' ? 8 + pin : 16 + pin);
    static constexpr uint8_t code = ain;
    static constexpr bool negative_ok = ain <= 15;
    using PinType = P;
};

// ---- the converter -------------------------------------------------------------

template <uint8_t adc_num>
class Adc {
    static_assert(adc_num == 0, "AVR DA/DB have one ADC: ADC0");

public:
    Adc() = delete;

    /// Full scale of one sample: 4096 or 1024 (times accumulate for a result).
    static uint32_t steps() { return steps_; }
    static uint8_t accumulate() { return acc_; }
    static Ref reference() { return ref_; }
    /// RES holds the sum of the accumulated samples truncated to 16 bits:
    /// above 16 samples the hardware drops LSBs - 32 -> 1 bit, 64 -> 2,
    /// 128 -> 3. sum = result() << result_shift().
    static uint8_t result_shift() { return acc_ <= 16 ? 0 : acc_ == 32 ? 1 : acc_ == 64 ? 2 : 3; }
    /// Full scale of a RESULT as read (accumulated, truncated).
    static uint32_t result_steps() { return (steps_ * acc_) >> result_shift(); }

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: the whole configuration as a constant, checked
    /// - including the CLK_ADC range for a static clock.
    template <AdcConfig cfg, typename Clock>
    static bool init(Clock clock) {
        static_assert(adc_config_valid(cfg),
                      "AdcConfig: accumulate must be 1,2,4,..,128 and sample_delay <= 15");
        if constexpr (Clock::is_static) {
            static_assert(adc_clock_in_range(Clock::hz, cfg.prescaler),
                          "AdcConfig: CLK_PER / prescaler must stay within 125 kHz .. 2 MHz");
        }
        return init(clock, cfg);      // a DynamicClock's current rate can still refuse
    }

    /// Run-time form. Writes the mux (GND) and every knob before enabling
    /// (errata 2.3.2), enables, then waits the converter's warm-up
    /// (t_ADC_INIT, 6 us typ.: the first conversion before that is
    /// garbage - measured). Returns false, touching nothing, for an
    /// invalid configuration.
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& cfg) {
        static_assert(clock_follows<Clock, Adc>(),
                      "this Adc is initialized with a DynamicClock that does not list it "
                      "among its Users: CLK_ADC would drift out of range on a clock change");
        if (!adc_config_valid(cfg) || !adc_clock_in_range(clock_hz(clock), cfg.prescaler)) {
            return false;
        }
        auto& r = regs();
        r.CTRLA = 0;                                       // off while configuring
        Vref::adc0(cfg.reference, cfg.reference_always_on);
        ref_ = cfg.reference;
        steps_ = cfg.resolution == AdcRes::bits12 ? 4096u : 1024u;
        acc_ = cfg.accumulate;
        clk_adc_hz_ = adc_clock_hz(clock_hz(clock), cfg.prescaler);   // what rebase() preserves
        free_running_ = cfg.free_running;
        r.MUXPOS = ADC_MUXPOS_GND_gc;
        r.MUXNEG = ADC_MUXNEG_GND_gc;
        r.CTRLB = sampnum_bits(cfg.accumulate);
        r.CTRLC = static_cast<uint8_t>(cfg.prescaler);    // PRESC codes 0x0..0xD in enum order
        r.CTRLD = static_cast<uint8_t>((static_cast<uint8_t>(cfg.init_delay) << 5) |
                                       (cfg.sample_delay & ADC_SAMPDLY_gm));
        r.SAMPCTRL = cfg.sample_length;
        r.CTRLE = ADC_WINCM_NONE_gc;
        r.EVCTRL = 0;
        r.INTCTRL = 0;
        r.INTFLAGS = ADC_RESRDY_bm | ADC_WCMP_bm;
        r.CTRLA = static_cast<uint8_t>(
            ADC_ENABLE_bm |
            (cfg.free_running ? ADC_FREERUN_bm : 0) |
            (cfg.resolution == AdcRes::bits10 ? ADC_RESSEL_10BIT_gc : ADC_RESSEL_12BIT_gc) |
            (cfg.left_adjust ? ADC_LEFTADJ_bm : 0) |
            (cfg.differential ? ADC_CONVMODE_bm : 0) |
            (cfg.run_standby ? ADC_RUNSTBY_bm : 0));
        r.DBGCTRL = cfg.debug_run ? ADC_DBGRUN_bm : 0;
        delay_us(clock, 10);                                // t_ADC_INIT (6 us typ.)
        return true;
    }

    /// Change the configuration under a running program: stops free-
    /// running, waits for the converter to be idle, then init(cfg).
    /// The mux selection is reset to GND: select() again.
    template <typename Clock>
    static bool reconfigure(Clock clock, const AdcConfig& cfg) {
        if (!adc_config_valid(cfg)) {
            return false;
        }
        stop();
        while (busy()) {
        }
        return init(clock, cfg);
    }

    /// The peripheral clock changed (DynamicClock fan-out): keep CLK_ADC
    /// as close as possible to the one in effect before the switch,
    /// within range - init's on the first rebase, the last rebase's
    /// after it (clk_adc_hz_ below is rewritten here; aiming at init's
    /// is the intent, a gap docs/avrxt/adc.md lists).
    /// PRESC must not change during a conversion: this stops, waits for
    /// idle, rewrites, and restarts free-running if it was on. The owner
    /// must have paused event-started conversions (start_on_events(false))
    /// before the clock switches - a conversion can begin at any time
    /// otherwise. Sample and init delays are in CLK_ADC cycles: they
    /// scale with the new CLK_ADC like everything else.
    /// clock_ok() tells whether the last rebase found an in-range
    /// prescaler: rebase() itself must stay void (the ClockUser
    /// contract) and best-effort, but a CLK_PER too slow for
    /// 125 kHz .. 2 MHz must not fail in silence.
    static bool clock_ok() { return clock_ok_; }
    static void rebase(uint32_t hz) {
        const AdcPresc p = adc_presc_for(hz, clk_adc_hz_);
        clock_ok_ = adc_clock_in_range(hz, p);
        stop();
        while (busy()) {
        }
        regs().CTRLC = static_cast<uint8_t>(p);
        clk_adc_hz_ = adc_clock_hz(hz, p);
        if (free_running_) {
            start();
        }
    }

    /// CLK_ADC in effect (after init/rebase).
    static uint32_t clock_hz_adc() { return clk_adc_hz_; }

    // ---- input selection --------------------------------------------------

    /// Positive input from a pin (its digital input buffer is disabled,
    /// as the datasheet recommends). Buffered by the hardware: takes
    /// effect at the next conversion. With init_delay != 0 see flush().
    template <typename P>
    static void select(AnalogIn<P>) {
        P::input_enable(false);
        regs().MUXPOS = AnalogIn<P>::code;
    }
    static void select(AdcInput in) { regs().MUXPOS = static_cast<uint8_t>(in); }

    /// Differential pair: positive pin/internal, negative pin (AIN0-15),
    /// GND or DAC0.
    template <typename Pp, typename Pn>
    static void select(AnalogIn<Pp> p, AnalogIn<Pn>) {
        static_assert(AnalogIn<Pn>::negative_ok, "AIN16-21 (PF0-5) cannot be a negative input");
        Pn::input_enable(false);
        select(p);
        regs().MUXNEG = AnalogIn<Pn>::code;
    }
    template <typename Pp>
    static bool select(AnalogIn<Pp> p, AdcInput neg) {
        if (!adc_neg_valid(neg)) return false;
        select(p);
        regs().MUXNEG = neg_code(neg);
        return true;
    }
    static bool select(AdcInput pos, AdcInput neg) {
        if (!adc_neg_valid(neg)) return false;
        select(pos);
        regs().MUXNEG = neg_code(neg);
        return true;
    }

    /// The positive input code in effect (MUXPOS): what AnalogIn<P>::code
    /// or AdcInput says for the input selected. Read it in the RESRDY ISR
    /// body's glue to label a result (util/analog_sampler.hpp).
    static uint8_t selected() { return regs().MUXPOS; }

    /// The code selected() reports for an input, at compile time.
    template <typename P>
    static constexpr uint8_t input_code(AnalogIn<P>) { return AnalogIn<P>::code; }
    static constexpr uint8_t input_code(AdcInput in) { return static_cast<uint8_t>(in); }

    /// Errata 2.3.2: one throw-away conversion after a select() made
    /// with init_delay != 0 (or after a reference change). Blocking.
    static void flush() { (void)read(); }

    // ---- conversions ------------------------------------------------------

    static void start() { regs().COMMAND = ADC_STCONV_bm; }
    static void stop() { regs().COMMAND = ADC_SPCONV_bm; }
    static bool busy() { return (regs().COMMAND & ADC_STCONV_bm) != 0; }

    /// True once a result is ready (RESRDY flag); result() clears it.
    static bool ready() { return (regs().INTFLAGS & ADC_RESRDY_bm) != 0; }

    /// The result (RES: a sample, or the sum of `accumulate` samples,
    /// right- or left-adjusted as configured). Reading RES clears both
    /// RESRDY and WCMP: the window verdict is captured first, see
    /// window_hit().
    static uint16_t result() {
        last_hit_ = (regs().INTFLAGS & ADC_WCMP_bm) != 0;
        return regs().RES;
    }
    static int16_t result_signed() { return static_cast<int16_t>(result()); }

    /// Blocking one-shot: start, wait for the RESULT (RESRDY - STCONV
    /// clears 2 CLK_PER before the result and its flags are formatted;
    /// waiting on busy() alone reads the previous result), read. The
    /// wait has no time-out: it ends when RESRDY is set, one conversion
    /// on an enabled converter; a disabled converter never sets it.
    static uint16_t read() {
        start();
        while (!ready()) {
        }
        return result();
    }

    // ---- window comparator --------------------------------------------------

    enum class Window : uint8_t { none = 0, below = 1, above = 2, inside = 3, outside = 4 };

    /// Compare each RESULT (the accumulated value) against the thresholds;
    /// WCMP flag/interrupt when the condition holds.
    static void window(Window mode, uint16_t low, uint16_t high) {
        regs().WINLT = low;
        regs().WINHT = high;
        regs().CTRLE = static_cast<uint8_t>(mode);
    }
    /// The thresholds in the differential conversion's format (33.5.16:
    /// two's complement) - the unsigned form above is for single-ended.
    static void window_signed(Window mode, int16_t low, int16_t high) {
        window(mode, static_cast<uint16_t>(low), static_cast<uint16_t>(high));
    }
    static void window_off() { regs().CTRLE = ADC_WINCM_NONE_gc; }
    /// Did the last result read by result()/read() match the window?
    /// (The hardware flag is cleared by the RES read itself, 33.5.12.)
    static bool window_hit() { return last_hit_; }
    /// The live flag, before any RES read (e.g. in a polling loop that
    /// only wants hits): true when the last conversion matched.
    static bool window_flag() { return (regs().INTFLAGS & ADC_WCMP_bm) != 0; }
    static void clear_window_flag() { regs().INTFLAGS = ADC_WCMP_bm; }

    // ---- interrupts and events ------------------------------------------------

    static void enable_resrdy_interrupt(bool on) { irq(ADC_RESRDY_bm, on); }
    static void enable_wcmp_interrupt(bool on) { irq(ADC_WCMP_bm, on); }

    /// Die temperature in kelvin from a 12-bit single-ended reading of
    /// AdcInput::temp with the 2.048 V reference (DS40002247B 33.3.3.8:
    /// T = (offset - result) * slope / 4096, the two factors from the
    /// signature row). Pure arithmetic over the reading: call it with
    /// result() / read() of a temp conversion.
    static uint16_t temp_kelvin(uint16_t result12) {
        uint32_t t = static_cast<uint32_t>(SIGROW.TEMPSENSE1) - result12;
        t *= SIGROW.TEMPSENSE0;
        t += 4096u / 2;
        t /= 4096u;
        return static_cast<uint16_t>(t);
    }

    /// ISR body for ADCn_RESRDY_vect: the result (flags cleared by the read).
    [[gnu::always_inline]] static uint16_t resrdy() { return result(); }
    /// ISR body for ADCn_WCMP_vect: the matching result (flags cleared by the read).
    [[gnu::always_inline]] static uint16_t wcmp() { return result(); }

    /// Start a conversion on every rising edge of the channel's event
    /// (async: works in standby). Pace with EvPitDiv<n> on the channel.
    template <uint8_t n>
    static void start_on(EventChannel<n> ch) {
        EvAdc0Start::listen(ch);
        regs().EVCTRL = ADC_STARTEI_bm;
    }
    static void start_on_events(bool on) { regs().EVCTRL = on ? ADC_STARTEI_bm : 0; }

    static void disable() { regs().CTRLA &= static_cast<uint8_t>(~ADC_ENABLE_bm); }

private:
    static constexpr ADC_t& regs() { return ADC0; }

    static void irq(uint8_t bit, bool on) {
        if (on) regs().INTCTRL |= bit; else regs().INTCTRL &= static_cast<uint8_t>(~bit);
    }

    static constexpr uint8_t sampnum_bits(uint8_t acc) {
        uint8_t v = 0;
        while ((1u << v) < acc) ++v;          // 1->0, 2->1, 4->2, ... 128->7
        return v;
    }
    static constexpr uint8_t neg_code(AdcInput in) {
        return in == AdcInput::dac0 ? ADC_MUXNEG_DAC0_gc : ADC_MUXNEG_GND_gc;
    }

    static inline Ref ref_ = Ref::vdd;
    static inline uint32_t steps_ = 4096;
    static inline uint8_t acc_ = 1;
    /// Written by result(), which the RESRDY vector's body calls; read
    /// by window_hit() in the thread: volatile.
    static inline volatile bool last_hit_ = false;
    static inline uint32_t clk_adc_hz_ = 0;
    static inline bool free_running_ = false;
    static inline bool clock_ok_ = true;
};

static_assert(ClockUser<Adc<0>>);

} // namespace brio

#else // ADC_PGAEN_bm

// =============================================================================
// The AVR EA: the 12-bit differential ADC with PGA (DS40002443A ch. 31)
// =============================================================================
//
// The engine, from the chapter:
//  - One COMMAND store is a conversion (31.5.11): DIFF | MODE | START.
//    The driver keeps the DIFF | MODE image (cmd_) and a start is that
//    image with IMMEDIATE ORed in - a load and a store to COMMAND, beside
//    the store that notes the start mode (start_). MODE is the
//    configuration's: SINGLE_12BIT / SINGLE_8BIT, SERIES, BURST and their
//    _SCALING forms (table 31-1); DIFF is the input's (an AdcDiff pair
//    sets it, a single-ended input clears it), so a walk mixing single-
//    ended and differential inputs costs nothing extra.
//  - Two outputs (31.3.3.3): SAMPLE, 16 bits, every conversion; RESULT,
//    32 bits, the sum of an accumulation (4095 x 1024 = 4193280, 22
//    bits - adc_result_bits() below; in DIFF -2048 x 1024 at its lowest,
//    a signed 22) or the single conversion. result() reads RESULT[15:0]
//    (two loads, the low byte first: 8.10.2), sum() all four bytes,
//    sample() SAMPLE. SCALING right-shifts the sum by SAMPNUM - 4 (left
//    below 16 samples) so a scaled result's full scale is always 65536.
//  - The start modes (31.3.3.2): IMMEDIATE (start()), EVENT_TRIGGER
//    (start_on(): the EVSYS user, edge, async), MUXPOS_WRITE
//    (start_on_select(): the select is the start, one store a
//    conversion). MUXNEG_WRITE is declined: a differential select
//    writes MUXNEG first and MUXPOS last, so MUXPOS_WRITE serves both.
//  - The input mux is not buffered (31.3.3.5: "Changing any of these
//    while a conversion is ongoing will corrupt the output"). Under the
//    software pace the converter is idle when the owner selects; under
//    the event pace a select can land on an event-started conversion of
//    the old input, whose result would then be read under the new
//    input's label. select() there is COMMAND = STOP (an event-started
//    conversion in flight is aborted, 31.3.3.2), a wait of the output
//    latency (a conversion already past its end delivers its result up
//    to 0.5 CLK_ADC + a CLK_MAIN after the STOP, measured, and its label
//    must still be the old input's: 4 CLK_PER at CLK_PER / 4, 34 at / 64),
//    the mux, COMMAND = EVENT_TRIGGER again - the worst case is one
//    aborted conversion or one trigger that lands on STOP, never a
//    result under the wrong label (test_avr_analog letter s measures
//    it). Measured too: a mux write mid-conversion aborts the conversion
//    (no result), whichever input it selects - the chapter names only the
//    DAC, TEMPSENSE and VDD inputs as aborting. Free-running is the
//    owner's to stop before a select (31.3.3.5).
//  - Initialization is timed in hardware, per conversion (table 31-5):
//    the ADC 10 us, the PGA 20 us, an internal reference 60 us, the
//    temperature sensor and the DAC input 35 us, counted in
//    CLKCTRL.MCLKTIMEBASE (the clock owns it: clock.hpp's
//    ClockTimebase). LOWLAT (CTRLA) keeps the modules on and removes it
//    from every conversion but the first; it is off by default - the
//    energy of the modules kept on against 10..60 us a trigger -
//    and `.low_latency` turns it on. STATUS.ADCBUSY covers conversion
//    and settling: busy(). init() waits for nothing: the first
//    conversion waits for its settling in hardware.
//  - The reference is ADC.CTRLC's own (31.5.3): VDD 0, VREFA 2, 1.024 V
//    4, 2.048 V 5, 4.096 V 6, 2.5 V 7 - another encoding than VREF's
//    (vref.hpp's Ref names the same six levels; adc_refsel() maps them).
//    An internal reference wants VDD >= reference + 0.5 V and CLK_ADC
//    <= 2 MHz; VDD and VREFA allow 6 MHz (table 35-24); never under
//    300 kHz.
//  - The PGA (31.3.3.5.1, 31.5.12): gain 1..16, bias 100/75/50/25 %
//    usable to CLK_ADC 6/4.5/3/1.5 MHz; the ADC samples the PGA for a
//    fixed 3/5/6/12 us by bias (4 us at 100 % with an internal
//    reference, table 31-6); the VIA bits are shared by both muxes, so
//    `.pga` routes every input of the configuration through it.
//  - Sampling (31.5.9): SAMPDUR + 0.5 CLK_ADC cycles (+1 with the PGA);
//    with an internal reference and no PGA SAMPDUR >= 4 us x fCLK_ADC
//    (the reference buffer's autozero, 31.3.3.6); a burst through the
//    PGA SAMPDUR >= 11 (31.3.3.6.6; the PGA paragraph of 31.3.3.6 says
//    12 - measured, 11 reads as 12: test_avr_analog letter p). init()
//    refuses less.
//  - The window (31.3.3.8, 31.5.4): WINCM and WINSRC (RESULT[15:0] or
//    SAMPLE) in CTRLD; INSIDE is strict (WINLT < x < WINHT); WCMP rides
//    the RESRDY or the SAMPRDY vector by WINSRC (there is no WCMP
//    vector) and is cleared only by writing 1 (31.5.6): result() and
//    sample() capture it and clear it, window_hit() tells about the last
//    read. A window on RESULT is refused where RESULT is wider than 16
//    bits (an unscaled sum of 32 or more samples).
//  - The flags (31.5.6, table 31-11): RESRDY and SAMPRDY clear when
//    RESULT and SAMPLE are read; TRIGOVR (a trigger during a
//    conversion: the pace too fast), SAMPOVR and RESOVR (an unread
//    sample or result overwritten: the reader too slow) on the ERROR
//    vector - the hardware's own count of what a paced reader drops.
//  - The temporary registers are shared by RESULT, SAMPLE, WINLT and
//    WINHT (31.5.17): one owner, one context reading or writing them.
//
// Cost per conversion (the release listing, CLK_PER cycles): start() 9 -
// the start mode noted in RAM (a store), the image's load and one store
// to COMMAND; the RESRDY body 14 - one load of
// INTFLAGS, the window match cleared and kept (two stores), RESULT's two
// bytes; selected() 9 - two loads; a select ~20 - one MUXPOS store, two
// for a pair - and under the event pace two COMMAND stores and the
// output latency's 3 x latency_loops more. A burst of N costs one start,
// one interrupt and one read.

namespace brio {

// ---- the knobs (31.5) ---------------------------------------------------------

/// The single conversion's resolution: MODE SINGLE_12BIT or SINGLE_8BIT
/// (the 8-bit conversion state is 9 CLK_ADC cycles against 13,
/// 31.3.3.6.1). The accumulating modes convert at 12 bits.
enum class AdcRes : uint8_t { bits12, bits8 };

/// CLK_ADC = CLK_PER / n (CTRLB.PRESC, 31.5.2): the enumerator is the code.
enum class AdcPresc : uint8_t {
    div2, div4, div6, div8, div10, div12, div14, div16,
    div20, div24, div28, div32, div40, div48, div56, div64
};
static_assert(static_cast<uint8_t>(AdcPresc::div10) == ADC_PRESC_DIV10_gc &&
              static_cast<uint8_t>(AdcPresc::div64) == ADC_PRESC_DIV64_gc);

/// The divisor of a code: 2, 4 .. 16 by twos, 20 .. 32 by fours, 40 .. 64
/// by eights (31.5.2) - arithmetic, not a table a run-time call would copy.
constexpr uint8_t adc_presc_divisor(AdcPresc p) {
    const uint8_t i = static_cast<uint8_t>(p);
    if (i < 8) return static_cast<uint8_t>(2u * (i + 1u));
    if (i < 12) return static_cast<uint8_t>(16u + 4u * (i - 7u));
    return static_cast<uint8_t>(32u + 8u * (i - 11u));
}
static_assert(adc_presc_divisor(AdcPresc::div16) == 16 && adc_presc_divisor(AdcPresc::div20) == 20 &&
              adc_presc_divisor(AdcPresc::div32) == 32 && adc_presc_divisor(AdcPresc::div40) == 40 &&
              adc_presc_divisor(AdcPresc::div64) == 64);

/// The PGA's gain (PGACTRL.GAIN, 31.5.12).
enum class AdcGain : uint8_t { x1, x2, x4, x8, x16 };
static_assert(static_cast<uint8_t>(static_cast<uint8_t>(AdcGain::x16) << ADC_GAIN_gp) ==
              ADC_GAIN_16X_gc);

/// The PGA's bias current (PGACTRL.PGABIASSEL): the less current, the
/// slower the CLK_ADC it takes (6 / 4.5 / 3 / 1.5 MHz) and the longer the
/// ADC samples it (3 / 5 / 6 / 12 us, table 31-6).
enum class AdcPgaBias : uint8_t { pct100, pct75, pct50, pct25 };
static_assert(static_cast<uint8_t>(static_cast<uint8_t>(AdcPgaBias::pct25) << ADC_PGABIASSEL_gp) ==
              ADC_PGABIASSEL_25PCT_gc);

/// The window comparator's mode (CTRLD.WINCM, 31.5.4): the enumerator is
/// the code. INSIDE is strict on this converter: WINLT < x < WINHT.
enum class AdcWindow : uint8_t { none, below, above, inside, outside };
/// What the window compares (CTRLD.WINSRC): RESULT[15:0] or SAMPLE.
enum class AdcWindowSource : uint8_t { result, sample };

struct AdcConfig {
    Ref reference = Ref::vdd;
    AdcRes resolution = AdcRes::bits12;       ///< single conversions only
    AdcPresc prescaler = AdcPresc::div10;     ///< 20 MHz / 10 = 2 MHz, the accuracy table's rate
    uint8_t sample_duration = 2;              ///< SAMPDUR: + 0.5 CLK_ADC cycles of sampling (+1 with the PGA)
    uint16_t accumulate = 1;                  ///< 1, 2, 4 .. 1024 samples per result
    bool series = false;                      ///< one trigger per sample (SERIES) instead of one per run (BURST)
    bool scaled = false;                      ///< the sum scaled to 16 bits (full scale 65536)
    bool pga = false;                         ///< every input through the PGA (VIA)
    AdcGain gain = AdcGain::x1;
    AdcPgaBias pga_bias = AdcPgaBias::pct100;
    bool chopping = false;                    ///< sign chopping: accumulating modes only
    bool low_latency = false;                 ///< LOWLAT: the modules kept on, no initialization per trigger
    bool left_adjust = false;                 ///< 12-bit single or scaled only
    bool free_running = false;                ///< single and burst; not series
    bool run_standby = false;
    bool debug_run = false;                   ///< DBGRUN: keep converting while the CPU is halted
    AdcWindow window = AdcWindow::none;
    AdcWindowSource window_source = AdcWindowSource::result;
};

/// Bits of a RESULT as configured: 8 or 12 for a single conversion, 16
/// left-adjusted or scaled, 12 + log2(N) for an unscaled sum of N.
constexpr uint8_t adc_result_bits(const AdcConfig& c) {
    if (c.accumulate <= 1) {
        if (c.resolution == AdcRes::bits8) return 8;
        return c.left_adjust ? 16 : 12;
    }
    if (c.scaled) return 16;
    uint8_t k = 0;
    while ((1u << k) < c.accumulate) ++k;
    return static_cast<uint8_t>(12 + k);
}

// The configuration's rules, one predicate each (a static_assert per rule
// in init<cfg>, the conjunction in init(cfg)).
constexpr bool adc_accumulate_ok(const AdcConfig& c) {
    const uint16_t a = c.accumulate;
    return a != 0 && (a & (a - 1)) == 0 && a <= 1024;
}
/// 31.3.3.1: the accumulating modes convert at 12 bits; SERIES and SCALING
/// name an accumulation, so they want two samples or more; CHOPPING works
/// in the accumulating modes only (31.3.3.5.2).
constexpr bool adc_mode_ok(const AdcConfig& c) {
    if (c.accumulate > 1 && c.resolution == AdcRes::bits8) return false;
    if (c.accumulate == 1 && (c.series || c.scaled || c.chopping)) return false;
    return true;
}
/// 31.5.10: FREERUN does not apply to series.
constexpr bool adc_free_running_ok(const AdcConfig& c) { return !(c.free_running && c.series); }
/// Tables 31-2/31-3: left adjustment exists in 12-bit single and the
/// scaled modes, not at 8 bits nor for an unscaled sum.
constexpr bool adc_left_adjust_ok(const AdcConfig& c) {
    if (!c.left_adjust) return true;
    return c.accumulate == 1 ? c.resolution == AdcRes::bits12 : c.scaled;
}
/// The comparator sees RESULT[15:0]: on a sum wider than 16 bits it would
/// compare a wrapped number.
constexpr bool adc_window_ok(const AdcConfig& c) {
    return c.window == AdcWindow::none || c.window_source == AdcWindowSource::sample ||
           adc_result_bits(c) <= 16;
}
/// 31.3.3.6.6: a burst through the PGA wants SAMPDUR >= 11.
constexpr bool adc_burst_pga_ok(const AdcConfig& c) {
    return !(c.pga && c.accumulate > 1 && !c.series && c.sample_duration < 11);
}
constexpr bool adc_config_valid(const AdcConfig& c) {
    return adc_accumulate_ok(c) && adc_mode_ok(c) && adc_free_running_ok(c) &&
           adc_left_adjust_ok(c) && adc_window_ok(c) && adc_burst_pga_ok(c);
}

/// The REFSEL code of a reference in ADC.CTRLC (31.5.3) - not VREF's.
constexpr uint8_t adc_refsel(Ref r) {
    switch (r) {
    case Ref::vdd: return ADC_REFSEL_VDD_gc;
    case Ref::vrefa: return ADC_REFSEL_VREFA_gc;
    case Ref::v1024: return ADC_REFSEL_1V024_gc;
    case Ref::v2048: return ADC_REFSEL_2V048_gc;
    case Ref::v4096: return ADC_REFSEL_4V096_gc;
    case Ref::v2500: return ADC_REFSEL_2V500_gc;
    }
    return ADC_REFSEL_VDD_gc;
}
constexpr bool adc_ref_internal(Ref r) { return r != Ref::vdd && r != Ref::vrefa; }

/// CLK_ADC in Hz for a peripheral clock and a prescaler.
constexpr uint32_t adc_clock_hz(uint32_t clk_per_hz, AdcPresc p) {
    return clk_per_hz / adc_presc_divisor(p);
}
/// CLK_ADC's range (table 35-24): 300 kHz up to 2 MHz with an internal
/// reference, 6 MHz with VDD or VREFA.
inline constexpr uint32_t adc_clock_min_hz = 300'000;
constexpr uint32_t adc_clock_max_hz(Ref r) { return adc_ref_internal(r) ? 2'000'000u : 6'000'000u; }
/// The fastest CLK_ADC each PGA bias takes (31.5.12).
constexpr uint32_t adc_pga_clock_max_hz(AdcPgaBias b) {
    return 1'500'000u * (4u - static_cast<uint8_t>(b));        // 6, 4.5, 3, 1.5 MHz
}
constexpr bool adc_clock_in_range(uint32_t clk_per_hz, const AdcConfig& c) {
    const uint32_t f = adc_clock_hz(clk_per_hz, c.prescaler);
    if (f < adc_clock_min_hz || f > adc_clock_max_hz(c.reference)) return false;
    return !c.pga || f <= adc_pga_clock_max_hz(c.pga_bias);
}
/// SAMPDUR's floor at a clock: with an internal reference and no PGA
/// the reference buffer's autozero needs SAMPDUR >= 4 us x fCLK_ADC
/// (31.3.3.6, 31.5.9), rounded up; a burst through the PGA 11.
constexpr uint8_t adc_min_sample_duration(uint32_t clk_per_hz, const AdcConfig& c) {
    uint32_t m = 0;
    if (adc_ref_internal(c.reference) && !c.pga) {
        m = (4u * adc_clock_hz(clk_per_hz, c.prescaler) + 999'999u) / 1'000'000u;
    }
    if (c.pga && c.accumulate > 1 && !c.series && m < 11) m = 11;
    return static_cast<uint8_t>(m > 255 ? 255 : m);
}
constexpr bool adc_clock_valid(uint32_t clk_per_hz, const AdcConfig& c) {
    return adc_clock_in_range(clk_per_hz, c) &&
           c.sample_duration >= adc_min_sample_duration(clk_per_hz, c);
}

/// The prescaler giving the CLK_ADC closest to `target_hz` at
/// `clk_per_hz` among those the configuration's reference and PGA bias
/// allow (the nearest valid one; div64 when none is).
constexpr AdcPresc adc_presc_for(uint32_t clk_per_hz, uint32_t target_hz, const AdcConfig& c) {
    AdcPresc best = AdcPresc::div64;
    uint32_t best_err = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 16; ++i) {
        AdcConfig k = c;
        k.prescaler = static_cast<AdcPresc>(i);
        if (!adc_clock_in_range(clk_per_hz, k)) continue;
        const uint32_t f = adc_clock_hz(clk_per_hz, k.prescaler);
        const uint32_t err = f > target_hz ? f - target_hz : target_hz - f;
        if (err < best_err) { best_err = err; best = k.prescaler; }
    }
    return best;
}

/// How long the ADC samples the PGA, in microseconds (table 31-6): 3, 5,
/// 6, 12 by bias, 4 at 100 % with an internal reference.
constexpr uint8_t adc_pga_sample_us(const AdcConfig& c) {
    switch (c.pga_bias) {
    case AdcPgaBias::pct100: return adc_ref_internal(c.reference) ? 4 : 3;
    case AdcPgaBias::pct75: return 5;
    case AdcPgaBias::pct50: return 6;
    case AdcPgaBias::pct25: return 12;
    }
    return 12;
}

/// The free-running period in CLK_PER cycles of one result (single
/// modes), or of one sample within a burst or a series, initialization
/// excluded - the chapter's formulas (31.3.3.6.1-6) as the silicon runs
/// them (test_avr_analog letter 7, every row of tables 31-7 and 31-8
/// within its counting window):
///  - a burst sample, no PGA: (SAMPDUR + 14) CLK_ADC, as written;
///  - a single conversion: (SAMPDUR + 15) at 12 bits, plus the output
///    latency note 3 of figure 31-3 states (0.5 CLK_ADC and a CLK_MAIN)
///    plus one more CLK_PER - the restart of a free-running conversion
///    waits for its result to land;
///  - through the PGA the ADC samples it for whole CLK_ADC cycles:
///    floor(tPGA x fCLK_ADC) + 1 of them, where the chapter adds the bare
///    3 / 5 / 6 / 12 us (table 31-6) - (SAMPDUR + 15.5 + that) a single
///    conversion, (SAMPDUR + 2 + that) a burst sample;
///  - at 8 bits the chapter's SAMPDUR + 11 (+ the same latency) is what
///    this returns, and the measured rate is 1..6 % lower at short
///    SAMPDUR: not explained (docs/avrxt/adc.md).
constexpr uint32_t adc_period_cycles(uint32_t clk_per_hz, const AdcConfig& c) {
    const bool burst = c.accumulate > 1 && !c.series;
    const bool bits8 = c.accumulate <= 1 && c.resolution == AdcRes::bits8;
    const uint32_t div = adc_presc_divisor(c.prescaler);
    const uint32_t sd = c.sample_duration;
    const uint32_t pga = c.pga ? static_cast<uint32_t>(adc_pga_sample_us(c)) * (clk_per_hz / div) / 1'000'000u + 1u
                               : 0u;                     // whole CLK_ADC cycles of PGA sampling
    if (burst) {
        return (c.pga ? sd + 2u + pga : sd + 14u) * div;
    }
    const uint32_t half = 2u * sd + (bits8 ? 23u : 31u);  // + 15.5 or 11.5 CLK_ADC: note 3's half cycle
    return half * div / 2u + pga * div + 2u;              // + a CLK_MAIN (note 3) + one CLK_PER (measured)
}
static_assert(adc_period_cycles(20'000'000, AdcConfig{.prescaler = AdcPresc::div4, .sample_duration = 20,
                                                      .accumulate = 2}) == 136);   // 147059 sps, table 31-7

// ---- inputs -------------------------------------------------------------------

/// The internal inputs, by their MUXPOS codes (31.5.13). GND and DAC0 are
/// negative inputs as well, at the same codes (31.5.14); VDDDIV10 and
/// TEMPSENSE are positive only. The DAC0 input is the DAC's unbuffered
/// output (32.3.2.3.1): it needs the sampling time of a high-impedance
/// source.
enum class AdcInput : uint8_t {
    gnd = ADC_MUXPOS_GND_gc,
    vdd_div10 = ADC_MUXPOS_VDD10_gc,
    temp = ADC_MUXPOS_TEMPSENSE_gc,
    dac0 = ADC_MUXPOS_DAC0_gc,
};
static_assert(static_cast<uint8_t>(AdcInput::gnd) == ADC_MUXNEG_GND_gc &&
              static_cast<uint8_t>(AdcInput::dac0) == ADC_MUXNEG_DAC0_gc);

/// The comparators' DAC references, negative inputs only (31.5.14): the
/// positive mux has no code for them on this converter.
enum class AdcDacref : uint8_t {
    dacref0 = ADC_MUXNEG_DACREF0_gc,
    dacref1 = ADC_MUXNEG_DACREF1_gc,
};

/// The AIN number of a pin on this die (DS40002443A 3.1, 31.5.13), or
/// 0xFF: AIN0-7 = PD0-7, AIN8-11 = PE0-3, AIN16-21 = PF0-5, AIN22-27 =
/// PA2-7, AIN28-31 = PC0-3.
constexpr uint8_t adc_ain_of(char port, uint8_t pin) {
    switch (port) {
    case 'D': return pin;
    case 'E': return pin <= 3 ? static_cast<uint8_t>(8 + pin) : 0xFF;
    case 'F': return pin <= 5 ? static_cast<uint8_t>(16 + pin) : 0xFF;
    case 'A': return pin >= 2 ? static_cast<uint8_t>(20 + pin) : 0xFF;
    case 'C': return pin <= 3 ? static_cast<uint8_t>(28 + pin) : 0xFF;
    default: return 0xFF;
    }
}

/// A pin as an analog input. Every AIN is a positive and a negative input
/// on this converter (31.5.13, 31.5.14).
template <typename P>
struct AnalogIn {
    static constexpr char port = P::port_letter;
    static constexpr uint8_t pin = P::pin_number;
    static constexpr uint8_t ain = adc_ain_of(port, pin);
    static_assert(ain != 0xFF, "ADC inputs on the AVR EA: PD0-7 (AIN0-7), PE0-3 (AIN8-11), "
                               "PF0-5 (AIN16-21), PA2-7 (AIN22-27), PC0-3 (AIN28-31)");
    static constexpr uint8_t code = ain;
    static constexpr bool negative_ok = true;
    using PinType = P;
};

template <typename P> constexpr uint8_t adc_code(AnalogIn<P>) { return AnalogIn<P>::code; }
constexpr uint8_t adc_code(AdcInput in) { return static_cast<uint8_t>(in); }
constexpr uint8_t adc_code(AdcDacref in) { return static_cast<uint8_t>(in); }

template <typename P> constexpr bool adc_positive_ok(AnalogIn<P>) { return true; }
constexpr bool adc_positive_ok(AdcInput) { return true; }
constexpr bool adc_positive_ok(AdcDacref) { return false; }

template <typename P> constexpr bool adc_negative_ok(AnalogIn<P>) { return true; }
constexpr bool adc_negative_ok(AdcInput in) { return in == AdcInput::gnd || in == AdcInput::dac0; }
constexpr bool adc_negative_ok(AdcDacref) { return true; }

/// A differential pair as an input value: `AdcDiff<AnalogIn<Pin<'D', 1>>{},
/// AdcInput::gnd>{}`. Selecting it sets COMMAND.DIFF and both muxes; the
/// result is signed, (VINP - VINN) x gain / VREF x 2048 (31.3.3.3). The
/// positions are checked here, at compile time.
template <auto pos, auto neg>
struct AdcDiff {
    static_assert(adc_positive_ok(pos),
                  "AdcDiff: a comparator's DACREF is a negative input only on the AVR EA");
    static_assert(adc_negative_ok(neg),
                  "AdcDiff: the negative input is a pin (any AIN), GND, DAC0 or a comparator's "
                  "DACREF - VDDDIV10 and TEMPSENSE are positive inputs only (31.5.14)");
    static constexpr uint8_t pos_code = adc_code(pos);
    static constexpr uint8_t neg_code = adc_code(neg);
};

/// What selected() reports for a differential pair: the positive's code
/// with bit 7 set (no MUXPOS code reaches bit 6).
inline constexpr uint8_t adc_diff_flag = ADC_DIFF_bm;

/// The error flags error() returns (INTFLAGS, 31.5.6).
inline constexpr uint8_t adc_trigger_overrun = ADC_TRIGOVR_bm;
inline constexpr uint8_t adc_sample_overwrite = ADC_SAMPOVR_bm;
inline constexpr uint8_t adc_result_overwrite = ADC_RESOVR_bm;

// ---- the converter -------------------------------------------------------------

template <uint8_t adc_num>
class Adc {
    static_assert(adc_num == 0, "the AVR EA has one ADC: ADC0");

public:
    Adc() = delete;

    using Window = AdcWindow;

    /// Full scale of one sample: 4096, or 256 at 8 bits.
    static uint32_t steps() { return steps_; }
    static uint16_t accumulate() { return cfg_.accumulate; }
    static Ref reference() { return cfg_.reference; }
    /// The configuration in effect (after init, rebase).
    static const AdcConfig& config() { return cfg_; }
    /// Nothing is truncated on this converter (the 32-bit RESULT holds the
    /// whole sum): the shift is 0, kept for the surface the DA/DB share.
    static uint8_t result_shift() { return 0; }
    /// Full scale of a RESULT as read: 65536 scaled, steps() x N unscaled
    /// (the signed half of it in DIFF).
    static uint32_t result_steps() {
        return cfg_.scaled ? 65536u : steps_ * cfg_.accumulate;
    }

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: the whole configuration as a constant, checked
    /// rule by rule - including the CLK_ADC range and SAMPDUR's floor for
    /// a static clock.
    template <AdcConfig cfg, typename Clock>
    static bool init(Clock clock) {
        static_assert(adc_accumulate_ok(cfg), "AdcConfig: accumulate is 1, 2, 4 .. 1024");
        static_assert(adc_mode_ok(cfg),
                      "AdcConfig: the accumulating modes convert at 12 bits, and series, scaled and "
                      "chopping need accumulate >= 2 (31.3.3.1, 31.3.3.5.2)");
        static_assert(adc_free_running_ok(cfg), "AdcConfig: series accumulation is not free-running (31.5.10)");
        static_assert(adc_left_adjust_ok(cfg),
                      "AdcConfig: left adjustment exists for a 12-bit single conversion or a scaled sum "
                      "(tables 31-2, 31-3)");
        static_assert(adc_window_ok(cfg),
                      "AdcConfig: the window compares RESULT[15:0] - an unscaled sum of 32 or more "
                      "samples is wider: scale it, or compare each SAMPLE");
        static_assert(adc_burst_pga_ok(cfg), "AdcConfig: a burst through the PGA needs sample_duration >= 11 (31.3.3.6.6)");
        if constexpr (Clock::is_static) {
            static_assert(adc_clock_in_range(Clock::hz, cfg),
                          "AdcConfig: CLK_ADC = CLK_PER / prescaler must stay within 300 kHz .. 2 MHz with "
                          "an internal reference, .. 6 MHz with VDD or VREFA (table 35-24), and within the "
                          "PGA bias's limit (31.5.12)");
            static_assert(cfg.sample_duration >= adc_min_sample_duration(Clock::hz, cfg),
                          "AdcConfig: with an internal reference and no PGA, sample_duration >= 4 us x "
                          "fCLK_ADC (31.3.3.6)");
        }
        return init(clock, cfg);      // a DynamicClock's current rate can still refuse
    }

    /// Run-time form. Enables first, then configures (31.3.2), mux on GND,
    /// nothing started. No wait: initialization is timed in hardware at
    /// the first conversion. False, touching nothing, for an invalid
    /// configuration.
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& cfg) {
        static_assert(clock_follows<Clock, Adc>(),
                      "this Adc is initialized with a DynamicClock that does not list it "
                      "among its Users: CLK_ADC would drift out of range on a clock change");
        const uint32_t hz = clock_hz(clock);
        if (!adc_config_valid(cfg) || !adc_clock_valid(hz, cfg)) {
            return false;
        }
        auto& r = regs();
        r.COMMAND = ADC_START_STOP_gc;                    // anything in flight aborted
        r.INTCTRL = 0;
        r.CTRLA = static_cast<uint8_t>(ADC_ENABLE_bm |
                                       (cfg.low_latency ? ADC_LOWLAT_bm : 0) |
                                       (cfg.run_standby ? ADC_RUNSTDBY_bm : 0));
        r.CTRLB = static_cast<uint8_t>(cfg.prescaler);
        r.CTRLC = adc_refsel(cfg.reference);
        r.CTRLD = static_cast<uint8_t>(static_cast<uint8_t>(cfg.window) |
                                       (cfg.window_source == AdcWindowSource::sample ? ADC_WINSRC_bm : 0));
        r.CTRLE = cfg.sample_duration;
        r.CTRLF = static_cast<uint8_t>(sampnum_bits(cfg.accumulate) |
                                       (cfg.free_running ? ADC_FREERUN_bm : 0) |
                                       (cfg.left_adjust ? ADC_LEFTADJ_bm : 0) |
                                       (cfg.chopping ? ADC_CHOPPING_bm : 0));
        r.PGACTRL = cfg.pga ? static_cast<uint8_t>((static_cast<uint8_t>(cfg.gain) << ADC_GAIN_gp) |
                                                   (static_cast<uint8_t>(cfg.pga_bias) << ADC_PGABIASSEL_gp) |
                                                   ADC_PGAEN_bm)
                            : 0;
        via_ = cfg.pga ? static_cast<uint8_t>(ADC_VIA_PGA_gc) : 0;
        r.MUXNEG = static_cast<uint8_t>(ADC_MUXNEG_GND_gc | via_);
        r.MUXPOS = static_cast<uint8_t>(ADC_MUXPOS_GND_gc | via_);
        r.INTFLAGS = ADC_RESRDY_bm | ADC_SAMPRDY_bm | ADC_WCMP_bm | ADC_RESOVR_bm |
                     ADC_SAMPOVR_bm | ADC_TRIGOVR_bm;
        r.DBGCTRL = cfg.debug_run ? ADC_DBGRUN_bm : 0;
        cfg_ = cfg;
        steps_ = cfg.accumulate <= 1 && cfg.resolution == AdcRes::bits8 ? 256u : 4096u;
        clk_adc_hz_ = adc_clock_hz(hz, cfg.prescaler);
        target_adc_hz_ = clk_adc_hz_;                     // what rebase() returns to
        latency_loops_ = latency_loops(cfg.prescaler);
        clock_ok_ = true;
        start_ = ADC_START_STOP_gc;
        cmd_ = mode_bits(cfg);
        r.COMMAND = cmd_;                                 // the mode, START = STOP: nothing starts
        return true;
    }

    /// Change the configuration under a running program: aborts what is in
    /// flight, waits for the converter to settle, then init(cfg). The mux
    /// is back on GND: select() again.
    template <typename Clock>
    static bool reconfigure(Clock clock, const AdcConfig& cfg) {
        if (!adc_config_valid(cfg)) {
            return false;
        }
        stop();
        while (busy()) {
        }
        return init(clock, cfg);
    }

    /// The peripheral clock changed (DynamicClock fan-out): keep CLK_ADC as
    /// close as possible to the one chosen at init - not to the last
    /// rebase's, so a round trip of the clock brings it back (20 -> 10 ->
    /// 20 MHz: 2 -> 1.67 -> 2 MHz) - within the range the reference and
    /// the PGA bias allow. Aborts a conversion in flight,
    /// rewrites PRESC, re-arms the start mode in effect (free-running, the
    /// event pace, the select start). The owner pauses an event pace
    /// (start_on_events(false)) around the switch (docs/design/clock.md).
    /// MCLKTIMEBASE is the clock's: it moves with the switch. clock_ok()
    /// tells whether the last rebase found a valid CLK_ADC and SAMPDUR's
    /// floor still holds.
    static bool clock_ok() { return clock_ok_; }
    static void rebase(uint32_t hz) {
        const AdcPresc p = adc_presc_for(hz, target_adc_hz_, cfg_);
        cfg_.prescaler = p;
        clock_ok_ = adc_clock_valid(hz, cfg_);
        auto& r = regs();
        r.COMMAND = cmd_;                                 // STOP
        r.CTRLB = static_cast<uint8_t>(p);
        clk_adc_hz_ = adc_clock_hz(hz, p);
        latency_loops_ = latency_loops(p);
        if (cfg_.free_running && start_ == ADC_START_IMMEDIATE_gc) {
            r.COMMAND = static_cast<uint8_t>(cmd_ | ADC_START_IMMEDIATE_gc);
        } else if (start_ != ADC_START_STOP_gc && start_ != ADC_START_IMMEDIATE_gc) {
            r.COMMAND = static_cast<uint8_t>(cmd_ | start_);
        }
    }

    /// CLK_ADC in effect (after init/rebase).
    static uint32_t clock_hz_adc() { return clk_adc_hz_; }

    // ---- input selection --------------------------------------------------

    /// The digital input buffer of an input's pins turned off, once (the
    /// PINCTRL write leaves the select path: a select is a mux store).
    /// Internal inputs have no pin: nothing to do. AnalogSampler calls it
    /// for every input of its list at init.
    template <typename P>
    static void claim(AnalogIn<P>) { P::input_enable(false); }
    static void claim(AdcInput) {}
    static void claim(AdcDacref) {}
    template <auto pos, auto neg>
    static void claim(AdcDiff<pos, neg>) { claim(pos); claim(neg); }

    /// A single-ended input (DIFF cleared).
    template <typename P>
    static void select(AnalogIn<P>) { route(AnalogIn<P>::code, 0, false); }
    static void select(AdcInput in) { route(static_cast<uint8_t>(in), 0, false); }
    /// A differential pair (DIFF set, MUXNEG then MUXPOS).
    template <auto pos, auto neg>
    static void select(AdcDiff<pos, neg>) {
        route(AdcDiff<pos, neg>::pos_code, AdcDiff<pos, neg>::neg_code, true);
    }

    /// The input in effect: the MUXPOS code, with bit 7 (adc_diff_flag)
    /// set for a differential pair. Read it in the result ISR's glue to
    /// label a result (util/analog_sampler.hpp).
    static uint8_t selected() {
        return static_cast<uint8_t>((regs().MUXPOS & ADC_MUXPOS_gm) | (regs().COMMAND & ADC_DIFF_bm));
    }

    /// The code selected() reports for an input, at compile time.
    template <typename P>
    static constexpr uint8_t input_code(AnalogIn<P>) { return AnalogIn<P>::code; }
    static constexpr uint8_t input_code(AdcInput in) { return static_cast<uint8_t>(in); }
    template <auto pos, auto neg>
    static constexpr uint8_t input_code(AdcDiff<pos, neg>) {
        return static_cast<uint8_t>(AdcDiff<pos, neg>::pos_code | adc_diff_flag);
    }

    // ---- conversions ------------------------------------------------------

    /// One trigger (START = IMMEDIATE): a conversion, a burst, or one
    /// sample of a series. Ends an event or select pace.
    static void start() {
        start_ = ADC_START_IMMEDIATE_gc;
        regs().COMMAND = static_cast<uint8_t>(cmd_ | ADC_START_IMMEDIATE_gc);
    }
    /// START = STOP: aborts a conversion in flight (its result undefined,
    /// 31.3.3.2) and ends any pace.
    static void stop() {
        start_ = ADC_START_STOP_gc;
        regs().COMMAND = cmd_;
    }
    /// Converting, or waiting for a settling (STATUS.ADCBUSY, 31.5.7).
    static bool busy() { return (regs().STATUS & ADC_ADCBUSY_bm) != 0; }

    /// A result is ready (RESRDY); result() / sum() clear it.
    static bool ready() { return (regs().INTFLAGS & ADC_RESRDY_bm) != 0; }
    /// A sample is ready (SAMPRDY); sample() clears it.
    static bool sample_ready() { return (regs().INTFLAGS & ADC_SAMPRDY_bm) != 0; }

    /// RESULT[15:0]: a single conversion, a scaled sum, or an unscaled sum
    /// of up to 16 samples whole (wider sums: sum()). Captures and clears
    /// the window match (written 1, 31.5.6) - window_hit().
    static uint16_t result() { return take_result(); }
    static int16_t result_signed() { return static_cast<int16_t>(result()); }
    /// The whole 32-bit RESULT: an unscaled sum of any length (sign-
    /// extended in DIFF: sum_signed()).
    static uint32_t sum() {
        take_hit();
        auto& r = regs();
        const uint8_t b0 = r.RESULT0;
        const uint8_t b1 = r.RESULT1;
        const uint8_t b2 = r.RESULT2;
        const uint8_t b3 = r.RESULT3;
        return static_cast<uint32_t>(b0) | (static_cast<uint32_t>(b1) << 8) |
               (static_cast<uint32_t>(b2) << 16) | (static_cast<uint32_t>(b3) << 24);
    }
    static int32_t sum_signed() { return static_cast<int32_t>(sum()); }
    /// SAMPLE: the latest conversion, every conversion (clears SAMPRDY;
    /// captures a window match on SAMPLE).
    static uint16_t sample() { return take_sample(); }
    static int16_t sample_signed() { return static_cast<int16_t>(sample()); }

    /// Blocking: trigger and wait for the RESULT - one start for a single
    /// conversion or a burst, one per sample for a series. Then result().
    /// The wait has no time-out: the conversions and the initialization
    /// table 31-5 times on an enabled converter; a disabled converter
    /// never raises SAMPRDY and the wait never ends.
    static uint16_t read() {
        convert();
        return result();
    }
    /// The same, the whole sum.
    static uint32_t read_sum() {
        convert();
        return sum();
    }

    // ---- window comparator --------------------------------------------------

    /// Mode, thresholds and source at run time; false, touching nothing,
    /// for a window on RESULT where RESULT is wider than 16 bits. The
    /// thresholds are in the format of what is compared (31.5.18):
    /// unsigned single-ended, two's complement in DIFF (window_signed).
    static bool window(Window mode, uint16_t low, uint16_t high,
                       AdcWindowSource source = AdcWindowSource::result) {
        AdcConfig c = cfg_;
        c.window = mode;
        c.window_source = source;
        if (!adc_window_ok(c)) return false;
        cfg_ = c;
        auto& r = regs();
        r.WINLT = low;
        r.WINHT = high;
        r.CTRLD = static_cast<uint8_t>(static_cast<uint8_t>(mode) |
                                       (source == AdcWindowSource::sample ? ADC_WINSRC_bm : 0));
        return true;
    }
    static bool window_signed(Window mode, int16_t low, int16_t high,
                              AdcWindowSource source = AdcWindowSource::result) {
        return window(mode, static_cast<uint16_t>(low), static_cast<uint16_t>(high), source);
    }
    /// The thresholds alone (the mode the configuration named stays).
    static void window_thresholds(uint16_t low, uint16_t high) {
        regs().WINLT = low;
        regs().WINHT = high;
    }
    static void window_off() {
        cfg_.window = AdcWindow::none;
        regs().CTRLD = 0;
    }
    /// Did the last result()/sum()/sample() match the window?
    static bool window_hit() { return last_hit_ != 0; }
    /// The live flag (it stays until written 1).
    static bool window_flag() { return (regs().INTFLAGS & ADC_WCMP_bm) != 0; }
    static void clear_window_flag() { regs().INTFLAGS = ADC_WCMP_bm; }

    // ---- interrupts and events ------------------------------------------------

    static void enable_resrdy_interrupt(bool on) { irq(ADC_RESRDY_bm, on); }
    static void enable_samprdy_interrupt(bool on) { irq(ADC_SAMPRDY_bm, on); }
    /// The window match rides the RESRDY or the SAMPRDY vector by its
    /// source (31.3.3.8): bind that one.
    static void enable_wcmp_interrupt(bool on) { irq(ADC_WCMP_bm, on); }
    /// TRIGOVR, SAMPOVR, RESOVR on the ERROR vector.
    static void enable_error_interrupt(bool on) {
        irq(ADC_TRIGOVR_bm | ADC_SAMPOVR_bm | ADC_RESOVR_bm, on);
    }

    /// ISR body for ADC0_RESRDY_vect: the result (RESRDY cleared by the
    /// read, a window match captured and cleared).
    [[gnu::always_inline]] static uint16_t resrdy() { return take_result(); }
    /// ISR body for ADC0_SAMPRDY_vect: the sample.
    [[gnu::always_inline]] static uint16_t samprdy() { return take_sample(); }
    /// ISR body for ADC0_ERROR_vect, and the polled form: the error flags
    /// raised (adc_trigger_overrun / adc_sample_overwrite /
    /// adc_result_overwrite), cleared.
    [[gnu::always_inline]] static uint8_t error() {
        const uint8_t f = static_cast<uint8_t>(regs().INTFLAGS &
                                               (ADC_TRIGOVR_bm | ADC_SAMPOVR_bm | ADC_RESOVR_bm));
        regs().INTFLAGS = f;
        return f;
    }

    /// Die temperature in kelvin from a 12-bit single-ended reading of
    /// AdcInput::temp with the 1.024 V reference and SAMPDUR >= 32 us x
    /// fCLK_ADC (31.3.3.7): T = (result + offset) x slope / 4096, both
    /// signature-row factors signed (TEMPSENSE1 the offset, TEMPSENSE0 the
    /// slope). Pure arithmetic over the reading.
    static uint16_t temp_kelvin(uint16_t result12) {
        const int16_t offset = static_cast<int16_t>(SIGROW.TEMPSENSE1);
        const int16_t slope = static_cast<int16_t>(SIGROW.TEMPSENSE0);
        int32_t t = (static_cast<int32_t>(result12) + offset) * static_cast<int32_t>(slope);
        t += 4096 / 2;
        t /= 4096;
        return static_cast<uint16_t>(t);
    }

    /// Start a conversion (a burst, a series sample) on every rising edge
    /// of the channel's event (async: works in Idle, and in Standby with
    /// run_standby). The pace ends with start_on_events(false), stop() or
    /// start().
    template <uint8_t n>
    static void start_on(EventChannel<n> ch) {
        EvAdc0Start::listen(ch);
        start_on_events(true);
    }
    static void start_on_events(bool on) {
        start_ = on ? static_cast<uint8_t>(ADC_START_EVENT_TRIGGER_gc) : static_cast<uint8_t>(ADC_START_STOP_gc);
        regs().COMMAND = static_cast<uint8_t>(cmd_ | start_);
    }
    /// START = MUXPOS_WRITE: from now on every select() starts the
    /// conversion of the input it selects - a walk at one store an input.
    static void start_on_select(bool on) {
        start_ = on ? static_cast<uint8_t>(ADC_START_MUXPOS_WRITE_gc) : static_cast<uint8_t>(ADC_START_STOP_gc);
        regs().COMMAND = static_cast<uint8_t>(cmd_ | start_);
    }

    static void disable() {
        stop();
        regs().CTRLA &= static_cast<uint8_t>(~ADC_ENABLE_bm);
    }

    static constexpr ADC_t& regs() { return ADC0; }

private:
    static void irq(uint8_t bits, bool on) {
        if (on) regs().INTCTRL |= bits; else regs().INTCTRL &= static_cast<uint8_t>(~bits);
    }

    /// RESULT[15:0] and SAMPLE, the low byte first (it latches the high
    /// byte into TEMP, 8.10.1), the window match taken first.
    [[gnu::always_inline]] static uint16_t take_result() {
        take_hit();
        auto& r = regs();
        const uint8_t lo = r.RESULT0;
        const uint8_t hi = r.RESULT1;
        return static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
    }
    [[gnu::always_inline]] static uint16_t take_sample() {
        take_hit();
        auto& r = regs();
        const uint8_t lo = r.SAMPLEL;
        const uint8_t hi = r.SAMPLEH;
        return static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
    }

    /// The window match of the conversion being read: captured, and
    /// cleared by writing 1 (the only way, 31.5.6) - a write of 0 when
    /// there is none changes nothing.
    [[gnu::always_inline]] static void take_hit() {
        const uint8_t hit = static_cast<uint8_t>(regs().INTFLAGS & ADC_WCMP_bm);
        regs().INTFLAGS = hit;
        last_hit_ = hit;                                  // the WCMP bit itself: one store
    }

    /// One store to the mux (two for a pair) under the software pace.
    /// Under the event pace: STOP with the old input's DIFF, a wait of the
    /// output latency, the mux, the pace re-armed with the new DIFF - a
    /// conversion past its end when STOP lands still delivers its result
    /// up to 0.5 CLK_ADC + one CLK_MAIN later (figure 31-3's note 3; a
    /// STOP 1 us before the end at 312.5 kHz left the result, 3 us before
    /// it left none - test_avr_analog letter e), and its RESRDY body must
    /// read the old input's label: the wait lets it land before the mux
    /// moves (interrupts on). Under the select start, COMMAND is armed with
    /// the pair's DIFF before the MUXPOS store that starts it.
    [[gnu::always_inline]] static void route(uint8_t pos, uint8_t neg, bool diff) {
        auto& r = regs();
        const uint8_t old = cmd_;
        const uint8_t cmd = diff ? static_cast<uint8_t>(old | ADC_DIFF_bm)
                                 : static_cast<uint8_t>(old & ~ADC_DIFF_bm);
        cmd_ = cmd;
        const uint8_t v = via_;
        const uint8_t st = start_;
        if (st == ADC_START_EVENT_TRIGGER_gc) {
            r.COMMAND = old;                              // STOP: the old input's conversion aborted
            uint8_t n = latency_loops_;                   // a result past its end lands, old label
            __asm__ __volatile__("1: dec %0\n\tbrne 1b" : "+r"(n));
            if (diff) r.MUXNEG = static_cast<uint8_t>(neg | v);
            r.MUXPOS = static_cast<uint8_t>(pos | v);
            r.COMMAND = static_cast<uint8_t>(cmd | st);
        } else if (st == ADC_START_MUXPOS_WRITE_gc) {
            r.COMMAND = static_cast<uint8_t>(cmd | st);
            if (diff) r.MUXNEG = static_cast<uint8_t>(neg | v);
            r.MUXPOS = static_cast<uint8_t>(pos | v);    // the start
        } else {
            if (diff) r.MUXNEG = static_cast<uint8_t>(neg | v);
            r.MUXPOS = static_cast<uint8_t>(pos | v);
        }
    }

    /// The output latency in iterations of route()'s wait (a DEC and a
    /// taken BRNE, 3 CLK_PER each, at least one): 0.5 CLK_ADC = PRESC / 2
    /// CLK_PER, plus a CLK_MAIN and one more CLK_PER, rounded up - 2 at
    /// CLK_PER / 4, 12 at / 64 (measured at / 64: a result rose at most 23
    /// CLK_PER after the STOP, letter e).
    static constexpr uint8_t latency_loops(AdcPresc p) {
        return static_cast<uint8_t>((adc_presc_divisor(p) / 2u + 2u + 2u) / 3u);
    }
    static_assert(latency_loops(AdcPresc::div2) >= 1 && latency_loops(AdcPresc::div64) == 12);

    /// Trigger until the RESULT is there: one start for a single
    /// conversion or a burst, one per sample for a series.
    static void convert() {
        auto& r = regs();
        do {
            r.INTFLAGS = ADC_SAMPRDY_bm;
            start();
            while ((r.INTFLAGS & ADC_SAMPRDY_bm) == 0) {
            }
        } while ((r.INTFLAGS & ADC_RESRDY_bm) == 0);
    }

    static constexpr uint8_t sampnum_bits(uint16_t acc) {
        uint8_t v = 0;
        while ((1u << v) < acc) ++v;          // 1->0, 2->1, 4->2, ... 1024->10
        return v;
    }
    static constexpr uint8_t mode_bits(const AdcConfig& c) {
        if (c.accumulate <= 1) {
            return c.resolution == AdcRes::bits8 ? static_cast<uint8_t>(ADC_MODE_SINGLE_8BIT_gc)
                                                 : static_cast<uint8_t>(ADC_MODE_SINGLE_12BIT_gc);
        }
        if (c.series) {
            return c.scaled ? static_cast<uint8_t>(ADC_MODE_SERIES_SCALING_gc)
                            : static_cast<uint8_t>(ADC_MODE_SERIES_gc);
        }
        return c.scaled ? static_cast<uint8_t>(ADC_MODE_BURST_SCALING_gc)
                        : static_cast<uint8_t>(ADC_MODE_BURST_gc);
    }

    static inline AdcConfig cfg_{};
    static inline uint32_t steps_ = 4096;
    static inline uint32_t clk_adc_hz_ = 0;
    /// CLK_ADC as init() chose it: the target of every rebase.
    static inline uint32_t target_adc_hz_ = 0;
    /// The COMMAND image: DIFF | MODE, START = STOP.
    static inline uint8_t cmd_ = 0;
    /// The start mode in effect: STOP, IMMEDIATE, EVENT_TRIGGER, MUXPOS_WRITE.
    static inline uint8_t start_ = 0;
    /// VIA: the PGA route ORed into every mux store.
    static inline uint8_t via_ = 0;
    /// route()'s wait under the event pace (latency_loops of PRESC).
    static inline uint8_t latency_loops_ = 0;
    /// Written by result()/sample(), which the vectors' bodies call; read
    /// by window_hit() in the thread: volatile.
    static inline volatile uint8_t last_hit_ = 0;
    static inline bool clock_ok_ = true;
};

static_assert(ClockUser<Adc<0>>);

/// A converter whose configuration is a constant of its type: init(clock)
/// is init<cfg>(clock), and the width of what it delivers is known at
/// compile time - `sample_bits`, which AnalogSampler checks against its
/// 16-bit Sampled.value (an unscaled sum of 32 or more samples is refused
/// there). Every other verb is Adc<n>'s.
template <uint8_t n, AdcConfig cfg>
struct AdcConfigured : Adc<n> {
    static_assert(adc_config_valid(cfg), "AdcConfigured: an invalid AdcConfig (init<cfg> names the rule)");
    static constexpr AdcConfig config_value = cfg;
    static constexpr uint8_t sample_bits = adc_result_bits(cfg);
    template <typename Clock>
    static bool init(Clock clock) { return Adc<n>::template init<cfg>(clock); }
};

} // namespace brio

#endif // ADC_PGAEN_bm
