// test_avr_analog - the analog block self-test SUITE for the AVR DA/DB
// target: VREF, DAC, ADC exercised knob by knob against expectations,
// PASS/FAIL on the console. Bench-verified 54/54 on an AVR128DB48 rev
// A5 at 3.3 V (2026-08-19). It is the reference test of vref.hpp,
// dac.hpp and adc.hpp: keep it passing through every restructuring
// (docs/avrdx/adc.md holds the findings it produced).
//
// Naming: test_<target>_<subject> marks a bench test suite as opposed
// to a demo app; suites live in src/apps/ while the multi-app tooling
// is what it is, and move to a per-target directory when there are
// enough of them.
//
// Bench diagnostic, NOT a kernel app: sequential, blocking (delay_us,
// polling), declared outside the AO rules like mcp_diag - it needs
// waits and ordered steps, and its findings feed the drivers. The
// kernel integration of the ADC (results as events, event-paced
// sampling) is the sampler app over util/analog_sampler.hpp.
//
// Wiring: PD6 (DAC0 OUT) -> PD1 (AIN1) and PD6 -> PD7 (VREFA). Nothing
// else. Console 460800; type a test number, 'a' for all, '?' for the
// list. Every test prints expected / measured and a verdict; the
// tolerances are the datasheet's (references +-4 %, ADC gain +-5 LSB,
// DAC +-10 LSB absolute) with the wire's own errors added.
//
// Supply-independent: VDD is MEASURED at start (VDDDIV10 against the
// 2.048 V reference) and everything that depends on it follows -
// which internal references have headroom (2.048 needs 2.5 V, 2.5
// needs 2.95 V, 4.096 needs 4.55 V: at 5 V all four are exercised),
// the VDD/10 expectations. Run it at 3.3 V and at 5 V.
//
// What is checked, and how it checks itself with one wire:
//   1  references: DAC at ref A, code 768; ADC at ref B reads it: the
//      ratio must be refA/refB (all pairs VDD allows at 3.3 V: 1.024,
//      2.048, 2.5) - internal path (MUXPOS DAC0) and the wire agree;
//   2  DAC ramp on the wire, 12-bit ADC: monotonic, gain, offset;
//   3  DAC OUTEN off: internal path still exact (with sample_length: the
//      unbuffered DAC output is high-impedance - measured), wire floats;
//   4  DAC settling: read 1 us vs 20 us after a full-scale RISING step
//      (falling steps are slow on a bare pin: ~1 uA sink - measured);
//   5  ADC 12 vs 10 bit, LEFTADJ bit patterns, same input;
//   6  differential: PD1 vs DAC0 internal ~ 0; PD1 vs GND = single-ended;
//   7  prescalers: free-running rate for each PRESC vs the formula
//      (Ticker-timed, OSC32K tolerance);
//   8  accumulation 1..128: mean stays, noise falls;
//   9  sample_length / sample_delay / init_delay lengthen the
//      conversion as the formula says;
//  10  event start: PIT/64 -> ADC start, 512 results/s (EVSYS user);
//  11  window comparator, four modes, on a DAC ramp: WCMP at the exact
//      codes;
//  12  errata 2.3.2: with init_delay != 0, select() after enable gives
//      the OLD input for one conversion - shown, then flush() fixes it;
//  13  internal inputs: GND ~ 0, VDDDIV10 ~ VDD/10, VDDIO2DIV10,
//      TEMPSENSE plausible (15..45 C);
//  14  VREFA = the DAC itself: with the ADC referenced to VREFA and
//      reading DAC0 internally, the result is full scale for any DAC
//      code >= 1.024 V.
// Not tested here (no instruments): absolute accuracy of the internal
// references (needs a trusted meter), the DAC's drive current (taken
// from the datasheet), RUNSTDBY/ALWAYSON (sleep timing), DACREF0-2
// (needs the ACs), the RESRDY event as a generator (needs a TCB).

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/ac.hpp"
#include "avrdx/adc.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/dac.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/analog.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using D = Dac<0>;
using A = Adc<0>;
using Wire = AnalogIn<Pin<'D', 1>>;            // PD6 -> PD1

uint16_t vdd_mv = 0;                            // measured at start (VDDDIV10 vs 2.048 V)

// The internal references this supply has headroom for (VREF specs:
// 2.048 needs VDD >= 2.5 V, 2.5 needs 2.95 V, 4.096 needs 4.55 V).
uint8_t usable_refs(Ref* out) {
    uint8_t n = 0;
    out[n++] = Ref::v1024;
    if (vdd_mv >= 2500) out[n++] = Ref::v2048;
    if (vdd_mv >= 2950) out[n++] = Ref::v2500;
    if (vdd_mv >= 4550) out[n++] = Ref::v4096;
    return n;
}

// ---- tiny test harness ------------------------------------------------------
uint8_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}

// |a - b| <= tol
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

// N single readings, averaged (blocking).
uint16_t read_avg(uint8_t n) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; ++i) sum += A::read();
    return static_cast<uint16_t>(sum / n);
}

// Differential results are signed: average them as such.
int16_t read_avg_signed(uint8_t n) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < n; ++i) sum += static_cast<int16_t>(A::read());
    return static_cast<int16_t>(sum / n);
}

void adc_default(Ref r) {
    A::init(clock, AdcConfig{.reference = r, .prescaler = AdcPresc::div16});   // 1.5 MHz
    A::select(Wire{});
}

// The unbuffered DAC0 input is high-impedance: 32 extra CLK_ADC cycles
// of sampling (21 us at 1.5 MHz) - measured: 2 cycles read 3-4 % low.
constexpr uint8_t internal_sample_length = 32;

void adc_internal(Ref r) {
    A::init(clock, AdcConfig{.reference = r, .prescaler = AdcPresc::div16,
                             .sample_length = internal_sample_length});
}

void dac_default(Ref r) {
    D::init({.reference = r});
    D::set(0);
    delay_us(clock, 300);       // reference start-up (200 us worst) + settling
}

// ---- 1: references cross-check ---------------------------------------------------
void t1_references() {
    print(serial, "1 references: DAC code 768 at ref A, ADC at ref B (internal path and wire)", crlf);
    Ref refs[4];
    const uint8_t nrefs = usable_refs(refs);
    print(serial, "  VDD ", vdd_mv, " mV: ", nrefs, " internal references usable", crlf);
    for (uint8_t ia = 0; ia < nrefs; ++ia) {
        const Ref ra = refs[ia];
        dac_default(ra);
        D::set(768);
        delay_us(clock, 50);
        const uint16_t dac_mv_ = dac_mv(768, D::steps, ref_mv(ra));
        for (uint8_t ib = 0; ib < nrefs; ++ib) {
            const Ref rb = refs[ib];
            const uint16_t rb_mv = ref_mv(rb);
            // expected counts, saturating at full scale
            const uint32_t exp = dac_mv_ >= rb_mv ? 4095u : (static_cast<uint32_t>(dac_mv_) * 4096u) / rb_mv;
            adc_internal(rb);
            A::select(AdcInput::dac0);
            const uint16_t in = read_avg(8);
            adc_default(rb);
            const uint16_t wi = read_avg(8);
            const int32_t tol = static_cast<int32_t>(exp * 5 / 100) + 12;   // 4 % refs + gain/offset
            print(serial, "  DAC ", ref_mv(ra), " mV ref -> ADC ", rb_mv, " mV ref: exp ", exp,
                  " internal ", in, " wire ", wi, crlf);
            verdict("internal path", near(in, static_cast<int32_t>(exp), tol));
            verdict("wire", near(wi, static_cast<int32_t>(exp), tol));
        }
    }
}

// ---- 2: DAC ramp on the wire -------------------------------------------------------
void t2_ramp() {
    print(serial, "2 DAC ramp 0..1023 step 64 -> ADC on the wire (both 2.048 V)", crlf);
    dac_default(Ref::v2048);
    adc_default(Ref::v2048);
    bool mono = true;
    int32_t worst = 0;
    uint16_t prev = 0;
    D::set(0);
    delay_us(clock, 5000);                   // the FALL to 0 V is slow: 1 uA sink into the pin
    for (uint16_t code = 0; code < 1024; code += 64) {
        D::set(code);
        delay_us(clock, 300);                // rising step: ~50 us ring, then settled
        const uint16_t r = read_avg(4);
        const int32_t exp = static_cast<int32_t>(code) * 4;   // 10 -> 12 bits, same reference
        const int32_t err = static_cast<int32_t>(r) - exp;
        if (err > worst) worst = err;
        if (-err > worst) worst = -err;
        if (code != 0 && r < prev) mono = false;
        prev = r;
        print(serial, "  code ", code, " exp ", exp, " read ", r, " err ", err, crlf);
    }
    verdict("monotonic", mono);
    verdict("|error| <= 60 LSB12 (DAC 10 + ADC 5 LSB, x4, + wire)", worst <= 60);
    print(serial, "  (falling steps are slow on a bare pin: the buffer sinks ~1 uA - datasheet note 2)", crlf);
}

// ---- 3: OUTEN off ------------------------------------------------------------------
void t3_outen() {
    print(serial, "3 DAC output buffer off: internal path exact, wire floats", crlf);
    D::init({.reference = Ref::v2048, .output_pin = false});
    D::set(512);
    delay_us(clock, 300);
    // the internal path with default sampling vs with sample_length:
    // the unbuffered DAC output is high-impedance (bench finding)
    adc_default(Ref::v2048);
    A::select(AdcInput::dac0);
    const uint16_t in_short = read_avg(8);
    adc_internal(Ref::v2048);
    A::select(AdcInput::dac0);
    const uint16_t in_long = read_avg(8);
    A::select(Wire{});
    const uint16_t wi = read_avg(8);
    print(serial, "  internal, 2-cycle sampling: ", in_short, "  with sample_length ",
          internal_sample_length, ": ", in_long, " (exp ~2048)  wire ", wi,
          " (floating: informative - the pin holds charge for a while)", crlf);
    verdict("internal path with OUTEN off, sampled long enough", near(in_long, 2048, 30));
    verdict("longer sampling reads closer (unbuffered source impedance)",
            (in_long > in_short) && near(in_long, 2048, 30));
    D::init({.reference = Ref::v2048});   // buffer back on (errata: keep it on)
}

// ---- 4: settling ---------------------------------------------------------------------
void t4_settling() {
    print(serial, "4 DAC settling: 0 -> 1023 read after 1 us and after 20 us", crlf);
    dac_default(Ref::v2048);
    adc_default(Ref::v2048);
    D::set(0);
    delay_us(clock, 50);
    D::set(1023);
    delay_us(clock, 1);
    const uint16_t early = A::read();
    delay_us(clock, 20);
    const uint16_t late = A::read();
    print(serial, "  1 us: ", early, "  20 us: ", late, " (exp ~4092)", crlf);
    verdict("settled after 20 us", near(late, 4092, 40));
    print(serial, "  (early value is informative: the ADC samples ~1.5 us after start)", crlf);
}

// ---- 5: resolution and adjust -------------------------------------------------------
void t5_resolution() {
    print(serial, "5 12 vs 10 bit, left adjust", crlf);
    dac_default(Ref::v2048);
    D::set(600);
    delay_us(clock, 50);
    adc_default(Ref::v2048);
    const uint16_t r12 = read_avg(8);
    A::init(clock, AdcConfig{.reference = Ref::v2048, .resolution = AdcRes::bits10});
    A::select(Wire{});
    const uint16_t r10 = read_avg(8);
    A::init(clock, AdcConfig{.reference = Ref::v2048, .left_adjust = true});
    A::select(Wire{});
    const uint16_t rl = A::read();                 // one read: averaging would fill the low nibble
    print(serial, "  12-bit ", r12, "  10-bit ", r10, " (exp 12/4)  left-adjusted ", hex(rl),
          " (exp 12-bit << 4)", crlf);
    verdict("10-bit ~ 12-bit / 4 (a different conversion, +-8 LSB10)", near(r10, r12 / 4, 8));
    verdict("left adjust = value << 4", near(rl >> 4, r12, 32) && (rl & 0x000F) == 0);
}

// ---- 6: differential ---------------------------------------------------------------
void t6_differential() {
    print(serial, "6 differential: PD1 - DAC0(internal) ~ 0; PD1 - GND = single-ended", crlf);
    dac_default(Ref::v2048);
    D::set(700);
    delay_us(clock, 50);
    A::init(clock, AdcConfig{.reference = Ref::v2048, .differential = true,
                             .sample_length = internal_sample_length});
    A::select(Wire{}, AdcInput::dac0);
    const int16_t d0 = read_avg_signed(8);
    A::select(Wire{}, AdcInput::gnd);
    const int16_t d1 = read_avg_signed(8);
    print(serial, "  wire - dac0 = ", d0, " (exp ~0)   wire - gnd = ", d1, " (exp ~1400)", crlf);
    verdict("wire - dac0 = the buffer's offset, within +-20 mV (40 counts)", near(d0, 0, 40));
    verdict("wire - gnd = 700*2 (differential full scale is 2048)", near(d1, 1400, 40));
}

// ---- 7: prescalers ------------------------------------------------------------------
void t7_prescalers() {
    print(serial, "7 free-running rate per prescaler vs formula (100 ms windows)", crlf);
    dac_default(Ref::v2048);
    D::set(512);
    constexpr AdcPresc ps[] = {AdcPresc::div12, AdcPresc::div16, AdcPresc::div32,
                               AdcPresc::div64, AdcPresc::div128};
    for (AdcPresc p : ps) {
        AdcConfig c{.reference = Ref::v2048, .prescaler = p, .free_running = true};
        A::init(clock, c);
        A::select(Wire{});
        A::start();
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() == t0) {}                 // align to a tick
        const uint32_t start = Ticker::millis();
        uint32_t n = 0;
        while (Ticker::millis() - start < 100) {
            if (A::ready()) { (void)A::result(); ++n; }
        }
        A::stop();
        const uint32_t cycles = adc_conversion_cycles(c);
        const uint32_t exp = (SysClock::hz / cycles) / 10;   // per 100 ms
        print(serial, "  div", adc_presc_divisor(p), ": ", n, " results/100ms, formula ",
              exp, crlf);
        verdict("rate within 8 %", near(static_cast<int32_t>(n), static_cast<int32_t>(exp),
                                        static_cast<int32_t>(exp * 8 / 100) + 2));
    }
    A::init(clock, AdcConfig{.reference = Ref::v2048});
}

// ---- 8: accumulation ----------------------------------------------------------------
void t8_accumulation() {
    print(serial, "8 accumulation: mean holds, noise falls", crlf);
    dac_default(Ref::v2048);
    D::set(512);
    delay_us(clock, 300);                    // rising step settles (~50 us ring)
    constexpr uint8_t accs[] = {1, 4, 16, 64, 128};
    for (uint8_t a : accs) {
        A::init(clock, AdcConfig{.reference = Ref::v2048, .accumulate = a});
        A::select(Wire{});
        uint32_t sum = 0, mn = 0xFFFF, mx = 0;
        for (uint8_t i = 0; i < 16; ++i) {
            const uint16_t r = A::read();
            sum += r; if (r < mn) mn = r; if (r > mx) mx = r;
        }
        // RES is the truncated sum: << result_shift() restores the scale
        const uint32_t mean_per_sample = ((sum / 16) << A::result_shift()) / a;
        print(serial, "  acc ", a, ": mean/sample ", mean_per_sample, " (exp ~2048) spread ",
              (mx - mn) << A::result_shift(), " sum-counts over 16 results (",
              ((mx - mn) << A::result_shift()) * 100 / a, "/100 per sample)", crlf);
        verdict("mean per sample ~ 2048", near(static_cast<int32_t>(mean_per_sample), 2048, 30));
    }
}

// ---- 9: sampling knobs ----------------------------------------------------------------
void t9_sampling() {
    print(serial, "9 sample_length / sample_delay / init_delay lengthen the conversion", crlf);
    dac_default(Ref::v2048);
    D::set(512);
    struct Case { const char* name; AdcConfig c; };
    const Case cases[] = {
        {"base div64", AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div64, .free_running = true}},
        {"sample_length 64", AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div64,
                                       .sample_length = 64, .free_running = true}},
        {"sample_delay 15", AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div64,
                                      .sample_delay = 15, .free_running = true}},
    };
    for (const Case& k : cases) {
        A::init(clock, k.c);
        A::select(Wire{});
        A::start();
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() == t0) {}
        const uint32_t start = Ticker::millis();
        uint32_t n = 0;
        while (Ticker::millis() - start < 100) {
            if (A::ready()) { (void)A::result(); ++n; }
        }
        A::stop();
        const uint32_t exp = (SysClock::hz / adc_conversion_cycles(k.c)) / 10;
        print(serial, "  ", k.name, ": ", n, "/100ms, formula ", exp, crlf);
        verdict(k.name, near(static_cast<int32_t>(n), static_cast<int32_t>(exp),
                             static_cast<int32_t>(exp * 8 / 100) + 2));
    }
    // init_delay: one-shot conversions per 100 ms with a big init delay
    // (each start pays the delay: 256 CLK_ADC at 375 kHz = 683 us)
    A::init(clock, AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div64,
                      .init_delay = AdcInitDelay::cycles256});
    A::select(Wire{});
    A::flush();
    const uint32_t start = Ticker::millis();
    uint32_t n = 0;
    while (Ticker::millis() - start < 100) { (void)A::read(); ++n; }
    // Bench finding (A5): INITDLY is paid only for the first conversion
    // after enable, not per start - one-shots run at the plain rate.
    print(serial, "  init_delay 256 @ 375 kHz: ", n, " one-shots/100ms (bench: ~2240 - the delay "
                  "is paid once after enable, not per start)", crlf);
    verdict("init_delay applies once after enable (not per start)", n > 1500);
    A::init(clock, AdcConfig{.reference = Ref::v2048});
}

// ---- 10: event start -----------------------------------------------------------------
void t10_event_start() {
    print(serial, "10 event start: PIT/64 (512 Hz) on channel 1 -> ADC start", crlf);
    dac_default(Ref::v2048);
    D::set(512);
    A::init(clock, AdcConfig{.reference = Ref::v2048});
    A::select(Wire{});
    EventChannel<1>::source(EvPitDiv<64>{});
    A::start_on(EventChannel<1>{});
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() == t0) {}
    const uint32_t start = Ticker::millis();
    uint32_t n = 0;
    uint16_t last = 0;
    while (Ticker::millis() - start < 1000) {
        if (A::ready()) { last = A::result(); ++n; }
    }
    A::start_on_events(false);
    EventChannel<1>::off();
    print(serial, "  ", n, " results in 1 s (exp 512, OSC32K tolerance), last ", last, crlf);
    verdict("512 event-started conversions per second", near(static_cast<int32_t>(n), 512, 30));
    verdict("value sane", near(last, 2048, 40));
}

// ---- 11: window comparator ------------------------------------------------------------
void t11_window() {
    print(serial, "11 window comparator on a DAC ramp (thresholds 1000..3000)", crlf);
    dac_default(Ref::v2048);
    A::init(clock, AdcConfig{.reference = Ref::v2048});
    A::select(Wire{});
    struct Mode { const char* name; A::Window m; bool at_500, at_2000, at_3500; };
    const Mode modes[] = {
        {"below 1000", A::Window::below, true, false, false},
        {"above 3000", A::Window::above, false, false, true},
        {"inside", A::Window::inside, false, true, false},
        {"outside", A::Window::outside, true, false, true},
    };
    const uint16_t codes[] = {125, 500, 875};   // ~500, ~2000, ~3500 counts
    for (const Mode& m : modes) {
        A::window(m.m, 1000, 3000);
        bool ok = true;
        for (uint8_t i = 0; i < 3; ++i) {
            D::set(codes[i]);
            delay_us(clock, 2000);           // the fall from 875 back to 125 is slow (1 uA sink)
            (void)A::read();                       // result() captures WCMP before RES clears it
            const bool hit = A::window_hit();
            const bool exp = i == 0 ? m.at_500 : i == 1 ? m.at_2000 : m.at_3500;
            if (hit != exp) ok = false;
            print(serial, "  ", m.name, " @", codes[i] * 4, ": ", hit ? "hit" : "-", " exp ",
                  exp ? "hit" : "-", crlf);
        }
        verdict(m.name, ok);
    }
    A::window_off();
}

// ---- 12: errata 2.3.2 -----------------------------------------------------------------
void t12_errata() {
    print(serial, "12 errata 2.3.2: select() after enable with init_delay != 0 lags one conversion", crlf);
    dac_default(Ref::v2048);
    D::set(800);                                   // ~3200 counts on the wire
    delay_us(clock, 50);
    A::init(clock, AdcConfig{.reference = Ref::v2048, .init_delay = AdcInitDelay::cycles64});
    // init() left the mux on GND (before enable). Now select the wire AFTER enable:
    A::select(Wire{});
    const uint16_t first = A::read();
    const uint16_t second = A::read();
    print(serial, "  first read after select: ", first, "  second: ", second, " (wire ~3200)", crlf);
    verdict("first conversion still on the OLD input (GND) - the erratum", first < 100 && second > 3000);
    A::select(AdcInput::gnd);
    A::flush();                                    // the driver's remedy
    const uint16_t after_flush = A::read();
    verdict("flush() makes the next read current", after_flush < 100);
}

// ---- 13: internal inputs -----------------------------------------------------------------
void t13_internal() {
    print(serial, "13 internal inputs", crlf);
    A::init(clock, AdcConfig{.reference = Ref::v2048});
    A::select(AdcInput::gnd);
    const uint16_t g = read_avg(8);
    A::select(AdcInput::vdd_div10);
    const uint16_t v = read_avg(8);
    A::select(AdcInput::vddio2_div10);
    const uint16_t v2 = read_avg(8);
    const uint16_t vdd_meas = adc_mv(v, 4096, 2048) * 10;
    print(serial, "  gnd ", g, "  vdd/10 -> VDD ~ ", vdd_meas, " mV (start-up: ", vdd_mv,
          ")  vddio2/10 -> ", adc_mv(v2, 4096, 2048) * 10, " mV", crlf);
    verdict("gnd ~ 0", g < 8);
    verdict("VDD/10 repeatable vs the start-up measurement (2 %)", near(vdd_meas, vdd_mv, vdd_mv / 50));
    // temperature: 2.048 V ref, init delay >= 25 us, sample length >= 28 us
    // at CLK_ADC = 24 MHz / 64 = 375 kHz: 1 cycle = 2.67 us -> 32 cycles > 25 us... use
    // 64 for the init delay and sample_length 12 (32 us).
    A::init(clock, AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div64, .sample_length = 12,
                      .init_delay = AdcInitDelay::cycles64});
    A::select(AdcInput::temp);
    A::flush();
    delay_us(clock, 40);
    const uint16_t t = read_avg(8);
    const uint16_t k = A::temp_kelvin(t);
    print(serial, "  temp raw ", t,
          " -> ", k, " K = ", static_cast<int16_t>(k - 273), " C", crlf);
    verdict("die temperature plausible (10..50 C)", k > 283 && k < 323);
}

// ---- 14: VREFA from the DAC ------------------------------------------------------------
void t14_vrefa() {
    print(serial, "14 VREFA (PD7) driven by the DAC: ADC on DAC0 internal must read full scale", crlf);
    dac_default(Ref::v2048);
    A::init(clock, AdcConfig{.reference = Ref::vrefa, .sample_length = internal_sample_length});
    A::select(AdcInput::dac0);
    bool ok = true;
    for (uint16_t code = 520; code <= 1020; code += 250) {   // >= 1.04 V (VREFA min 1.024)
        D::set(code);
        delay_us(clock, 300);
        A::flush();
        const uint16_t r = read_avg(8);
        print(serial, "  DAC ", code, " (", dac_mv(code, D::steps, 2048), " mV) as VREFA: ADC(dac0) = ", r,
              " (exp ~4095)", crlf);
        if (!near(r, 4095, 30)) ok = false;
    }
    verdict("full scale for every VREFA level", ok);
    // and VDD/10 against a known VREFA: 3300/10 = 330 mV / (DAC 1023 = 2046 mV) * 4096
    D::set(1023);
    delay_us(clock, 300);
    A::select(AdcInput::vdd_div10);
    A::flush();
    const uint16_t v = read_avg(8);
    const uint32_t exp = (static_cast<uint32_t>(vdd_mv) / 10) * 4096u / 2046u;
    print(serial, "  VDD/10 with VREFA = 2046 mV: ", v, " (exp ", exp, ")", crlf);
    verdict("VDD/10 vs VREFA", near(v, static_cast<int32_t>(exp), static_cast<int32_t>(exp / 8)));
}

// ---- 15: extras (dacref inputs, signed window, DAC readback, live rebase) --------
using DynClock = DynamicClock<SysClock, Serial, A>;

void t15_extras() {
    print(serial, "x extras: dacref0..2 inputs (30 us), signed window, DAC readback, live rebase", crlf);

    dac_default(Ref::v2048);
    D::set(512);
    verdict("DAC DATA readback", D::code() == 512);

    // the three AC DACREFs as ADC inputs: tADC_DACREF = 30 us wants a
    // long sample (64 CLK_ADC at 1.5 MHz = 43 us)
    (void)Ac<0>::init({.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                       .reference = Ref::v2048, .dacref = ac_dacref_code(500, 2048)});
    (void)Ac<1>::init({.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                       .reference = Ref::v2048, .dacref = ac_dacref_code(1000, 2048)});
    (void)Ac<2>::init({.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                       .reference = Ref::v2048, .dacref = ac_dacref_code(1500, 2048)});
    A::init(clock, AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div16,
                             .sample_length = 64});
    const uint16_t exp[3] = {1000, 2000, 3000};    // counts at 2.048 V
    const AdcInput in[3] = {AdcInput::dacref0, AdcInput::dacref1, AdcInput::dacref2};
    for (uint8_t k = 0; k < 3; ++k) {
        A::select(in[k]);
        A::flush();
        const uint16_t v = read_avg(8);
        print(serial, "  dacref", k, " reads ", v, " counts (expect ", exp[k], ")", crlf);
        verdict("dacref within 40 counts (~20 mV)", near(v, exp[k], 40));
    }
    Ac<0>::disable(); Ac<1>::disable(); Ac<2>::disable();

    // the signed window in differential mode: GND - DAC0(700) ~ -1400
    D::set(700);
    delay_us(clock, 50);
    A::init(clock, AdcConfig{.reference = Ref::v2048, .differential = true,
                             .sample_length = internal_sample_length});
    verdict("enum-negative select accepted", A::select(AdcInput::gnd, AdcInput::dac0));
    verdict("invalid negative refused", !A::select(AdcInput::gnd, AdcInput::temp));
    (void)A::select(AdcInput::gnd, AdcInput::dac0);
    A::window_signed(A::Window::below, -1000, 0);
    A::flush();
    (void)A::read();
    verdict("signed below -1000 hits at ~-1400", A::window_hit());
    A::window_signed(A::Window::below, -2000, 0);
    (void)A::read();
    verdict("signed below -2000 does not hit", !A::window_hit());
    A::window_off();

    // live rebase: the conversion result must not move with CLK_PER
    verdict("DynamicClock init", DynClock::init());
    (void)A::init(DynClock{}, AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div16,
                                        .sample_length = 64});
    (void)Ac<1>::init({.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                       .reference = Ref::v2048, .dacref = ac_dacref_code(1000, 2048)});
    A::select(AdcInput::dacref1);
    A::flush();
    const uint16_t at24 = read_avg(8);
    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    A::flush();
    const uint16_t at12 = read_avg(8);
    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    print(serial, "  dacref1: ", at24, " counts at 24 MHz, ", at12, " at 12 MHz", crlf);
    verdict("the reading does not move across the switch (+-8)", near(at12, at24, 8));
    verdict("CLK_ADC stayed in range (clock_ok)", A::clock_ok());
    Ac<1>::disable(); D::set(0);
}

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_references}, {'2', t2_ramp}, {'3', t3_outen}, {'4', t4_settling},
    {'5', t5_resolution}, {'6', t6_differential}, {'7', t7_prescalers},
    {'8', t8_accumulation}, {'9', t9_sampling}, {'0', t10_event_start},
    {'w', t11_window}, {'e', t12_errata}, {'i', t13_internal}, {'v', t14_vrefa},
    {'x', t15_extras},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_analog: 1 refs | 2 ramp | 3 outen | 4 settling | 5 res | 6 diff | 7 presc | "
                  "8 acc | 9 sampling | 0 event start | w window | e errata | i internal | v vrefa | x extras | a all",
          crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    sei();
    // Measure the supply first: VDD/10 against the 2.048 V reference,
    // sampled long (internal divider), averaged. Everything else follows.
    A::init(clock, AdcConfig{.reference = Ref::v2048, .sample_length = internal_sample_length});
    A::select(AdcInput::vdd_div10);
    A::flush();
    vdd_mv = static_cast<uint16_t>(adc_mv(read_avg(16), 4096, 2048) * 10);
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_analog - VREF/DAC/ADC self-test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF",
          ", silicon rev ", hex(SYSCFG.REVID), ")", crlf,
          "wire PD6->PD1 and PD6->PD7; VDD measured ", vdd_mv, " mV", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'a' || c == 'A') {
            uint8_t tp = 0, tf = 0;
            for (const Test& t : tests) { run(t.fn); tp += passed; tf += failed; }
            print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
        } else {
            bool found = false;
            for (const Test& t : tests) {
                if (t.key == c) { run(t.fn); found = true; }
            }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
