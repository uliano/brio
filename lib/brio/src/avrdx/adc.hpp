/*
 * adc.hpp
 *
 * Adc<n>: the AVR DA/DB 12-bit SAR ADC (DS40002247B ch. 33) as ONE
 * brio task with knobs - see docs/design/analog.md for why not several:
 * every use (single reading, paced stream, oversampled reading, window
 * watch, temperature, supply monitor) is the same sequence - select
 * input, trigger, wait RESRDY, read RES - with different knobs and
 * different consumers of the same two events. So: one type, a config
 * struct that owns the whole configuration, inputs as types, the two
 * ISR bodies for the app to bind and post, and event start through the
 * event system. Ownership of the one converter belongs to one AO.
 *
 *   using Meter = brio::Adc<0>;
 *   Meter::init<brio::AdcConfig{.reference = brio::Ref::v2048,
 *                               .prescaler = brio::AdcPresc::div12,
 *                               .accumulate = 16}>(clock); // checked at compile time
 *   Meter::select(brio::AnalogIn<brio::Pin<'D', 1>>{});   // AIN1
 *   Meter::start();  while (Meter::busy()) {}  auto raw = Meter::result();
 *
 * The config comes in two forms with one implementation: init<cfg>()
 * (the struct as a template argument: static_asserts on the knobs,
 * every register write folded) and init(cfg) (a run-time value: same
 * writes computed on the fly - reconfigure from a console command).
 * reconfigure(cfg) is init(cfg) that first waits for an idle converter
 * (the datasheet forbids changing RESSEL/CONVMODE/PRESC/SAMPNUM during
 * a conversion, and free-running must be stopped first).
 *
 * Knobs, named as the datasheet names them (33.5): reference (VREF.
 * ADC0REF), resolution 12/10 bit, differential, prescaler CLK_ADC =
 * CLK_PER / {2..256} (keep CLK_ADC within 125 kHz .. 2 MHz; accuracy
 * is specified at 500 kHz), sample_length (0-255 extra CLK_ADC cycles
 * of sampling: high-impedance sources, the temperature sensor's 28 us),
 * sample_delay (0-15, to move sampling off a noise harmonic),
 * init_delay (0/16/../256 CLK_ADC cycles before the first sample after
 * enable or standby: reference and warm-up settling, >= 6 us),
 * accumulate 1..128 samples per result (hardware oversampling; RES is
 * the SUM, truncated to 16 bits above 16), left_adjust, free_running,
 * run_standby. Conversion time = 2/fPER + n*(13.5 + 2 + sample_delay +
 * sample_length)/fADC + 2/fPER (12-bit; 11.5 at 10 bits).
 *
 * Errata DS80000915F: 2.3.2 (all silicon) - with init_delay != 0, a
 * MUXPOS/MUXNEG/accumulate change made after enabling the ADC or after
 * a reference change takes effect only after ONE conversion. init()
 * writes the mux before enabling, so the first configured input is
 * right; a later select() with init_delay != 0 must be followed by a
 * throw-away conversion - flush() does exactly that, blocking. 2.3.1
 * (A4): -3 mV typical single-ended offset.
 *
 * Bench facts (analog0, silicon A5): the converter needs its warm-up
 * (t_ADC_INIT 6 us typ.) after ENABLE before the first conversion is
 * trustworthy - init(clock, cfg) waits for it; INITDLY is paid only for
 * the first conversion after enable (not per start); the UNBUFFERED
 * DAC0 input (MUXPOS DAC0) is high-impedance and reads 3-4 % low with
 * the default 2-cycle sampling - give it sample_length (16-32 at
 * 1.5 MHz) like any source above 10 kOhm; the WCMP flag is cleared by
 * reading RES (33.5.12) - result() captures it first, window_hit()
 * tells about the LAST result read.
 *
 * ISR bodies (the app binds ADC0_RESRDY_vect / ADC0_WCMP_vect and
 * posts what they return - the ISR condenses, the AO decides):
 *   ISR(ADC0_RESRDY_vect) { post<Meter>(AdcResult{Adc<0>::resrdy()}); }
 * Event start: Adc<0>::start_on(EventChannel<n>{}) makes the converter
 * a user of that channel (async, works in standby): the natural pacer
 * is EvPitDiv<n> on the channel.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/vref.hpp"
#include "util/analog.hpp"

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
    gnd = 0x40, temp = 0x42, vdd_div10 = 0x44, vddio2_div10 = 0x45,
    dac0 = 0x48, dacref0 = 0x49, dacref1 = 0x4A, dacref2 = 0x4B
};

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
    /// RES holds the SUM of the accumulated samples truncated to 16 bits:
    /// above 16 samples the hardware drops LSBs - 32 -> 1 bit, 64 -> 2,
    /// 128 -> 3. sum = result() << result_shift().
    static uint8_t result_shift() { return acc_ <= 16 ? 0 : acc_ == 32 ? 1 : acc_ == 64 ? 2 : 3; }
    /// Full scale of a RESULT as read (accumulated, truncated).
    static uint32_t result_steps() { return (steps_ * acc_) >> result_shift(); }

    // ---- configuration ----------------------------------------------------

    /// Compile-time form: the whole configuration as a constant, checked.
    template <AdcConfig cfg, typename Clock>
    static void init(Clock clock) {
        static_assert(adc_config_valid(cfg),
                      "AdcConfig: accumulate must be 1,2,4,..,128 and sample_delay <= 15");
        init(clock, cfg);
    }

    /// Run-time form. Writes the mux (GND) and every knob BEFORE enabling
    /// (errata 2.3.2), enables, then waits the converter's warm-up
    /// (t_ADC_INIT, 6 us typ.: the first conversion before that is
    /// garbage - measured). Returns false, touching nothing, for an
    /// invalid configuration.
    template <typename Clock>
    static bool init(Clock clock, const AdcConfig& cfg) {
        if (!adc_config_valid(cfg)) {
            return false;
        }
        auto& r = regs();
        r.CTRLA = 0;                                       // off while configuring
        Vref::adc0(cfg.reference, cfg.reference_always_on);
        ref_ = cfg.reference;
        steps_ = cfg.resolution == AdcRes::bits12 ? 4096u : 1024u;
        acc_ = cfg.accumulate;
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

    // ---- input selection --------------------------------------------------

    /// Positive input from a pin (its digital input buffer is disabled,
    /// as the datasheet recommends). Buffered by the hardware: takes
    /// effect at the next conversion. With init_delay != 0 see flush().
    template <typename P>
    static void select(AnalogIn<P>) {
        P::disable_digital_input();
        regs().MUXPOS = AnalogIn<P>::code;
    }
    static void select(AdcInput in) { regs().MUXPOS = static_cast<uint8_t>(in); }

    /// Differential pair: positive pin/internal, negative pin (AIN0-15),
    /// GND or DAC0.
    template <typename Pp, typename Pn>
    static void select(AnalogIn<Pp> p, AnalogIn<Pn>) {
        static_assert(AnalogIn<Pn>::negative_ok, "AIN16-21 (PF0-5) cannot be a negative input");
        Pn::disable_digital_input();
        select(p);
        regs().MUXNEG = AnalogIn<Pn>::code;
    }
    template <typename Pp>
    static void select(AnalogIn<Pp> p, AdcInput neg) {
        select(p);
        regs().MUXNEG = neg_code(neg);
    }
    static void select(AdcInput pos, AdcInput neg) {
        select(pos);
        regs().MUXNEG = neg_code(neg);
    }

    /// Errata 2.3.2: one throw-away conversion after a select() made
    /// with init_delay != 0 (or after a reference change). Blocking.
    static void flush() { (void)read(); }

    // ---- conversions ------------------------------------------------------

    static void start() { regs().COMMAND = ADC_STCONV_bm; }
    static void stop() { regs().COMMAND = ADC_SPCONV_bm; }
    static bool busy() { return (regs().COMMAND & ADC_STCONV_bm) != 0; }

    /// True once a result is ready (RESRDY flag); result() clears it.
    static bool ready() { return (regs().INTFLAGS & ADC_RESRDY_bm) != 0; }

    /// The result (RES: a sample, or the SUM of `accumulate` samples,
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
    /// waiting on busy() alone reads the previous result), read.
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
    static void window_off() { regs().CTRLE = ADC_WINCM_NONE_gc; }
    /// Did the LAST result read by result()/read() match the window?
    /// (The hardware flag is cleared by the RES read itself, 33.5.12.)
    static bool window_hit() { return last_hit_; }
    /// The live flag, before any RES read (e.g. in a polling loop that
    /// only wants hits): true when the last conversion matched.
    static bool window_flag() { return (regs().INTFLAGS & ADC_WCMP_bm) != 0; }
    static void clear_window_flag() { regs().INTFLAGS = ADC_WCMP_bm; }

    // ---- interrupts and events ------------------------------------------------

    static void enable_resrdy_interrupt(bool on) { irq(ADC_RESRDY_bm, on); }
    static void enable_wcmp_interrupt(bool on) { irq(ADC_WCMP_bm, on); }

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
    static inline bool last_hit_ = false;
};

} // namespace brio
