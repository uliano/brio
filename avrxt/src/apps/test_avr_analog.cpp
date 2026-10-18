// test_avr_analog - the analog block self-test SUITE of the avrxt
// target: VREF, DAC, ADC exercised knob by knob against expectations,
// PASS/FAIL on the console. It is the reference test of vref.hpp,
// dac.hpp and adc.hpp: keep it passing through every restructuring
// (docs/avrxt/adc.md holds the findings it produced). TWO HALVES, one per
// ADC engine (adc.hpp): the AVR EA's differential ADC with PGA first,
// its letters listed in its own header below; the AVR DA/DB's after,
// described here.
//
// Naming: test_<target>_<subject> marks a bench test suite as opposed
// to a demo app.
//
// Bench diagnostic, NOT a kernel app: sequential, blocking (delay_us,
// polling), outside the AO rules - it needs waits and ordered steps,
// and its findings feed the drivers. The
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
// 2.048 V reference on a DB; on a DA, which has no VDDDIV10 input - no
// MVIO - DAC0 at 2000 mV from the 2.048 V reference read against VDD
// itself) and everything that depends on it follows -
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
//  13  internal inputs: GND ~ 0, VDDDIV10 ~ VDD/10, VDDIO2DIV10 (DB
//      only; on a DA the DAC-against-VDD reading repeats instead),
//      TEMPSENSE plausible (15..45 C);
//  14  VREFA = the DAC itself: with the ADC referenced to VREFA and
//      reading DAC0 internally, the result is full scale for any DAC
//      code >= 1.024 V.
// Not tested here (no instruments): absolute accuracy of the internal
// references (needs a trusted meter), the DAC's drive current (taken
// from the datasheet), RUNSTDBY/ALWAYSON (sleep timing), DACREF0-2
// (needs the ACs), the RESRDY event as a generator (needs a TCB).

// build: boards = db48,da48,ea48
// build: monitor_speed = 460800

#include <avr/io.h>

#if defined(ADC_PGAEN_bm)

// =============================================================================
// THE AVR EA HALF: the differential ADC with PGA (DS40002443A ch. 31), the
// DAC (ch. 32) and the VREF (ch. 20) on the board's analog net
// =============================================================================
//
// The net: PD6 (the DAC's buffered output) wired to PD1 (AIN1) and PD7
// (VREFA, AIN7), 10 kOhm from the net to ground (avr_board.hpp's
// board_analog_net, board_dac_load_ohms). Letter n measures what else is
// on it (its capacitance) and every wait after a DAC step below is
// derived from that measurement's bound (settle_ms).
//
// Bounds. The data sheet's figures are typical, so a bound is three
// typicals or the stated maximum, with the arithmetic beside it:
//   references: +-2 % (1.024, 2.048), +-3 % (2.5, 4.096) to 85 C (table
//     35-17) - the DAC's reference and the ADC's are separate buffers, so
//     a ratio of the two carries the sum;
//   ADC: TUE 5 LSB12 typ., offset 1.5, INL 0.8 (table 35-25);
//   DAC: accuracy 1 LSB10 typ., offset -2, INL 0.3 (table 35-22) - x4 in
//     LSB12 at one reference;
//   PGA: gain error -0.2 .. -0.8 % typ. (table 35-26).
// Rates are counted by the event system (the RESRDY or SAMPRDY event into
// a TCB counting events) over a window of the crystal's time (delay_us):
// no CPU in the count.

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrxt/ac.hpp"
#include "avrxt/adc.hpp"
#include "avrxt/clock.hpp"
#include "avrxt/dac.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/platform.hpp"
#include "avrxt/tca.hpp"
#include "avrxt/tcb.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "avrxt/userrow.hpp"
#include "util/analog.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#include "avr_board.hpp"

using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
constexpr SysClock clock;
constexpr uint32_t hz = SysClock::hz;
constexpr uint32_t tpus = hz / 1'000'000u;          // CLK_PER cycles a microsecond

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

TestBench<Serial> bench('a');

using D = Dac<0>;
using A = Adc<0>;
using Wire = AnalogIn<Pin<'D', 1>>;                 // AIN1, on the net
using Vrefa = AnalogIn<Pin<'D', 7>>;                // AIN7, on the net
using T1 = Tcb<1>;                                  // the event counter
using ChCount = EventChannel<2>;                    // what is counted
using ChPace = EventChannel<3>;                     // the pace

uint16_t vdd_mv = 0;                                // measured at start
uint8_t pgactrl_at_reset = 0;                       // PGACTRL before any write (A6)
uint32_t net_tau_us = 0;                            // letter n's 10 k decay
// The wait after a DAC step on the net: 5 time constants of the 10 k
// against the measured capacitance (e^-5 = 0.7 %: 14 mV of a 2 V step,
// under the bounds below), the 100 nF a REV 1.2 PCB places on VREFA
// assumed (5 x 10 k x 100 nF = 5 ms) until letter n has measured it.
uint32_t settle_ms = 5;

void verdict(const char* name, bool ok) { bench.verdict(name, ok); }
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
void hold_ms(uint32_t ms) {
    while (ms--) delay_us(clock, 1000);
}

// N blocking reads, averaged.
uint16_t read_avg(uint8_t n) {
    uint32_t s = 0;
    for (uint8_t i = 0; i < n; ++i) s += A::read();
    return static_cast<uint16_t>((s + n / 2) / n);
}
int16_t read_avg_signed(uint8_t n) {
    int32_t s = 0;
    for (uint8_t i = 0; i < n; ++i) s += static_cast<int16_t>(A::read());
    return static_cast<int16_t>(s / n);
}

// The internal references at this supply: the ADC's need VDD >=
// reference + 0.5 V (31.3.3.5), the DAC's (VREF) 1.8 / 2.55 / 3.0 /
// 4.6 V (table 35-17): the stricter of the two.
uint8_t usable_refs(Ref* out) {
    uint8_t n = 0;
    out[n++] = Ref::v1024;
    if (vdd_mv >= 2550) out[n++] = Ref::v2048;
    if (vdd_mv >= 3000) out[n++] = Ref::v2500;
    if (vdd_mv >= 4600) out[n++] = Ref::v4096;
    return n;
}
/// A reading of the DAC (on a 2 % reference) by the ADC (on another):
/// the two references' 2 % each, the DAC's 3 LSB10 (12 LSB12), the
/// ADC's TUE x3 (15).
constexpr int32_t two_refs(int32_t exp) { return exp * 40 / 1000 + 27; }

/// A reference's tolerance in tenths of a percent (table 35-17, 85 C).
uint8_t ref_tol_permille(Ref r) {
    return (r == Ref::v1024 || r == Ref::v2048) ? 20 : r == Ref::vdd ? 0 : 30;
}

/// The ADC at 2 MHz (20 MHz / 10, the accuracy table's rate) with the
/// sampling an internal reference needs: SAMPDUR >= 4 us x 2 MHz = 8
/// (31.3.3.6); `sd` longer for a high-impedance source.
AdcConfig at(Ref r, uint8_t sd = 8) {
    return AdcConfig{.reference = r, .prescaler = AdcPresc::div10, .sample_duration = sd};
}
/// The unbuffered DAC0 input: a high-impedance source - 64 CLK_ADC of
/// sampling (32 us at 2 MHz), the data sheet's 35 us DAC-input settling
/// timed in hardware before it (table 31-5).
constexpr uint8_t sd_internal = 64;

void dac_to(Ref r, uint16_t code) {
    D::init({.reference = r});
    D::set(code);
    hold_ms(settle_ms);
}
void dac_set(uint16_t code) {
    D::set(code);
    hold_ms(settle_ms);
}

// TCA0 as a free-running time base: CLK_PER / 16 = 1.25 MHz, 0.8 us a
// count, 52 ms a wrap.
void tbase_on() {
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div16, .period = 0xFFFF});
}
uint16_t tbase() { return Tca<0>::count(); }
uint32_t tbase_us(uint16_t counts) { return (static_cast<uint32_t>(counts) * 16u) / tpus; }

// Count generator G's pulses for `ms` of crystal time: the event system
// carries them into TCB1 counting events - no CPU in the count.
template <typename G>
uint16_t count_events(G, uint16_t ms) {
    ChCount::source(G{});
    PulseCounter<T1>::init(ChCount{});
    PulseCounter<T1>::reset();
    hold_ms(ms);
    const uint16_t n = PulseCounter<T1>::count();
    T1::disable();
    ChCount::off();
    return n;
}

// ---- n: the analog net ------------------------------------------------------------
// The DAC charges the net to ~2 V; then (a) the buffer off (disable()
// releases the pin on the AVR EA): the 10 k alone discharges the net -
// tau = R C, the time to V0/e, and the time to V0/e^2 is 2 tau if the
// decay is the resistor's (an exponential), shorter if a current sink
// adds to it; (b) the buffer on, code 0: the same fall with the buffer
// sinking - the mean sink current = C dV/dt - V/R; (c) a rising step:
// the mean source current. The ADC at 5 MHz on VDD, LOWLAT, reads PD1
// every ~5 us; TCA0 times it.
struct Fall { uint32_t t1_us; uint32_t t2_us; uint16_t v0; };

Fall timed_fall(bool buffer_on) {
    Fall f{0, 0, 0};
    dac_to(Ref::v2048, 1000);                      // 2.0 V
    f.v0 = read_avg(8);
    const uint16_t e1 = static_cast<uint16_t>(static_cast<uint32_t>(f.v0) * 3679u / 10000u);  // V0/e
    const uint16_t e2 = static_cast<uint16_t>(static_cast<uint32_t>(f.v0) * 1353u / 10000u);  // V0/e^2
    tbase_on();
    const uint16_t t0 = tbase();
    if (buffer_on) D::set(0); else D::disable();
    bool got1 = false;
    for (;;) {
        const uint16_t v = A::read();
        const uint16_t dt = static_cast<uint16_t>(tbase() - t0);
        if (!got1 && v <= e1) { f.t1_us = tbase_us(dt); got1 = true; }
        if (v <= e2) { f.t2_us = tbase_us(dt); break; }
        if (dt > 60000u) break;                    // 48 ms: neither reached
    }
    Tca<0>::reset();
    return f;
}

void tn_net() {
    print(serial, "n the analog net PD1-PD6-PD7: the 10 k's decay gives its capacitance; "
                  "the DAC's sink and source on it", crlf);
    A::init(clock, AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div4, .low_latency = true});
    A::claim(Wire{});
    A::select(Wire{});
    const Fall off = timed_fall(false);
    const uint32_t r = board_dac_load_ohms;
    const uint32_t c_nf = r ? off.t1_us * 1000u / r : 0;   // tau = R C: C [nF] = tau [us] x 1000 / R
    print(serial, "  buffer off: from ", off.v0, " counts, V0/e after ", off.t1_us, " us, V0/e^2 after ",
          off.t2_us, " us -> C = ", c_nf, " nF against ", r, " Ohm", crlf);
    // An exponential: t(e^-2) = 2 t(e^-1). The ADC reads every ~5 us
    // (3.4 us conversion + the loop) and TCA0 counts 0.8 us: +-10 us on
    // each time, against tau of a millisecond or so.
    verdict("the net decays as R C: t(V0/e^2) = 2 t(V0/e) within 5 % + 20 us",
            off.t1_us > 0 && near(static_cast<int32_t>(off.t2_us), 2 * static_cast<int32_t>(off.t1_us),
                                  static_cast<int32_t>(off.t1_us / 10u + 20u)));
    net_tau_us = off.t1_us;
    // 5 tau, rounded up to a millisecond, at least 1 ms.
    settle_ms = (5u * net_tau_us + 999u) / 1000u;
    if (settle_ms == 0) settle_ms = 1;
    print(serial, "  settle_ms for the letters after: ", settle_ms, crlf);

    D::init({.reference = Ref::v2048});
    const Fall on = timed_fall(true);
    // The buffer's mean sink over the fall from V0 to V0/e^2: the charge
    // C (V0 - V2) in t2, less what the 10 k took (its mean V / R, the
    // mean of an exponential between V0 and V0/e^2 being 0.432 V0).
    const uint32_t v0_mv = static_cast<uint32_t>(on.v0) * vdd_mv / 4096u;
    uint32_t sink_ua = 0;
    if (on.t2_us && r && c_nf) {
        const uint32_t total_ua = c_nf * (v0_mv * 8647u / 10000u) / on.t2_us;       // nF x mV / us = uA
        const uint32_t res_ua = (v0_mv * 432u / 1000u) * 1000u / r;                 // mV / kOhm = uA
        sink_ua = total_ua > res_ua ? total_ua - res_ua : 0;
    }
    print(serial, "  buffer on, code 0: from ", v0_mv, " mV, V0/e after ", on.t1_us, " us, V0/e^2 after ", on.t2_us,
          " us -> the buffer sinks ~", sink_ua, " uA beyond the 10 k", crlf);
    verdict("the buffer on falls no slower than the 10 k alone", on.t2_us > 0 && on.t2_us <= off.t2_us + 20u);

    // The rise: 0.2 V -> 2.0 V, the time to 90 %.
    dac_to(Ref::v2048, 100);
    const uint16_t lo = read_avg(4);
    tbase_on();
    const uint16_t t0 = tbase();
    D::set(1000);
    const uint16_t hi_target = static_cast<uint16_t>(lo + (static_cast<uint32_t>(1638u - lo) * 9u) / 10u);
    uint32_t rise_us = 0;
    for (;;) {
        const uint16_t v = A::read();
        const uint16_t dt = static_cast<uint16_t>(tbase() - t0);
        if (v >= hi_target) { rise_us = tbase_us(dt); break; }
        if (dt > 60000u) break;
    }
    Tca<0>::reset();
    const uint32_t src_ua = (rise_us && c_nf) ? c_nf * 1620u / rise_us + 110u : 0;   // 0.9 x 1.8 V; + the 10 k's mean
    print(serial, "  rise 0.2 -> 2.0 V: 90 % after ", rise_us, " us -> the buffer sources ~", src_ua,
          " uA into the net", crlf);
    verdict("the rise reaches 90 % within 48 ms", rise_us > 0);

    // (d) a digital pad on the net: the DAC off (its pin released), PD1
    // driven high then low, PD7 read as a digital input - the time to its
    // VIH (0.8 VDD: 1.6 time constants of the pad against C) and to its VIL
    // (0.2 VDD: 1.6 too): the pad's output resistance, which a timer
    // suite's parked pad on this net waits out.
    D::disable();
    hold_ms(settle_ms);
    using P1 = Pin<'D', 1>;
    using P7 = Pin<'D', 7>;
    P7::input_enable(true);
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1, .period = 0xFFFF});
    P1::clear();
    P1::output();
    hold_ms(settle_ms);
    uint16_t t1 = Tca<0>::count();
    P1::set();
    while (!P7::read() && static_cast<uint16_t>(Tca<0>::count() - t1) < 60000u) {}
    const uint16_t up = static_cast<uint16_t>(Tca<0>::count() - t1);
    hold_ms(1);
    t1 = Tca<0>::count();
    P1::clear();
    while (P7::read() && static_cast<uint16_t>(Tca<0>::count() - t1) < 60000u) {}
    const uint16_t down = static_cast<uint16_t>(Tca<0>::count() - t1);
    P1::input();
    Tca<0>::reset();
    P7::input_enable(false);
    A::claim(Wire{});
    const uint32_t up_ns = static_cast<uint32_t>(up) * 1000u / tpus;
    const uint32_t down_ns = static_cast<uint32_t>(down) * 1000u / tpus;
    // R = t / (1.6 C): ns / nF = Ohm
    print(serial, "  a pad drives the net: PD7 reads high after ", up_ns, " ns, low after ", down_ns,
          " ns -> the pad's ", c_nf ? up_ns * 10u / 16u / c_nf : 0, " / ", c_nf ? down_ns * 10u / 16u / c_nf : 0,
          " Ohm (high / low)", crlf);
    verdict("a pad charges the net to VIH within 100 us", up_ns > 0 && up_ns < 100'000u);
    D::init({.reference = Ref::v2048});
    D::set(0);
}

// ---- 1: references cross-check -------------------------------------------------------
void t1_references() {
    print(serial, "1 references: DAC code 768 at ref A, ADC at ref B, internal (DAC0) and wire (PD1)", crlf);
    Ref refs[4];
    const uint8_t nrefs = usable_refs(refs);
    print(serial, "  VDD ", vdd_mv, " mV: ", nrefs, " internal references usable", crlf);
    A::claim(Wire{});
    for (uint8_t ia = 0; ia < nrefs; ++ia) {
        const Ref ra = refs[ia];
        dac_to(ra, 768);
        const uint16_t dmv = dac_mv(768, D::steps, ref_mv(ra));
        for (uint8_t ib = 0; ib < nrefs; ++ib) {
            const Ref rb = refs[ib];
            const uint16_t rb_mv = ref_mv(rb);
            const uint32_t exp = dmv >= rb_mv ? 4095u : static_cast<uint32_t>(dmv) * 4096u / rb_mv;
            (void)A::init(clock, at(rb, sd_internal));
            A::select(AdcInput::dac0);
            const uint16_t in = read_avg(8);
            (void)A::init(clock, at(rb));
            A::select(Wire{});
            const uint16_t wi = read_avg(8);
            // the two references' tolerances (each +-2 or +-3 %) + the DAC's
            // 3 LSB10 (12 LSB12) + the ADC's TUE x3 (15)
            const int32_t tol = static_cast<int32_t>(exp * (ref_tol_permille(ra) + ref_tol_permille(rb)) / 1000u) + 27;
            print(serial, "  DAC ", ref_mv(ra), " mV ref -> ADC ", rb_mv, " mV ref: exp ", exp, " +-", tol,
                  " internal ", in, " wire ", wi, crlf);
            verdict("internal path", exp == 4095u ? in >= 4095 - tol : near(in, static_cast<int32_t>(exp), tol));
            verdict("wire", exp == 4095u ? wi >= 4095 - tol : near(wi, static_cast<int32_t>(exp), tol));
        }
    }
}

// ---- 2: DAC ramp on the wire ------------------------------------------------------------
void t2_ramp() {
    print(serial, "2 DAC ramp 64..960 step 64 -> ADC on the wire (both 2.048 V)", crlf);
    dac_to(Ref::v2048, 64);
    (void)A::init(clock, at(Ref::v2048));
    A::select(Wire{});
    uint16_t r[15];
    bool mono = true;
    for (uint8_t i = 0; i < 15; ++i) {
        dac_set(static_cast<uint16_t>(64u * (i + 1u)));
        r[i] = read_avg(8);
        if (i && r[i] <= r[i - 1]) mono = false;
    }
    // INL against the line through the end points: a residual bound of
    // the DAC's INL 0.3 LSB10 (1.2 LSB12) and the ADC's 0.8, x3, + the
    // 8-read mean's noise (2): 8 LSB12.
    int32_t worst = 0;
    for (uint8_t i = 0; i < 15; ++i) {
        const int32_t line = r[0] + (static_cast<int32_t>(r[14]) - r[0]) * i / 14;
        const int32_t d = static_cast<int32_t>(r[i]) - line;
        if (d > worst) worst = d;
        if (-d > worst) worst = -d;
        print(serial, "  code ", 64u * (i + 1u), " exp ", 256u * (i + 1u), " read ", r[i], " off the line ", d, crlf);
    }
    verdict("monotonic", mono);
    verdict("|off the end-point line| <= 8 LSB12 (DAC INL + ADC INL, x3, + noise)", worst <= 8);
    // the slope: the two references' 4 % + 0.5 % for the end points' offsets
    const int32_t slope = static_cast<int32_t>(r[14]) - r[0];            // exp 14 x 256 = 3584
    verdict("slope 3584 +- 4.5 % (two references' +-2 %, offsets)", near(slope, 3584, 161));
}

// ---- 3: OUTEN off ---------------------------------------------------------------------
void t3_outen() {
    print(serial, "3 DAC output buffer off: the internal path exact, the wire pulled to ground by the 10 k", crlf);
    D::init({.reference = Ref::v2048, .output_pin = false});
    D::set(512);
    hold_ms(settle_ms);
    (void)A::init(clock, at(Ref::v2048, 8));
    A::select(AdcInput::dac0);
    const uint16_t in_short = read_avg(8);
    (void)A::init(clock, at(Ref::v2048, sd_internal));
    A::select(AdcInput::dac0);
    const uint16_t in_long = read_avg(8);
    A::select(Wire{});
    const uint16_t wi = read_avg(8);
    print(serial, "  internal, SAMPDUR 8: ", in_short, "  SAMPDUR ", sd_internal, ": ", in_long,
          " (exp 2048 +-4 %)  wire ", wi, crlf);
    verdict("the internal path with OUTEN off, sampled long: 2048 +- 4 % + 27",
            near(in_long, 2048, two_refs(2048)));
    // the 10 k to ground with the pin released: leakage 5 nA x 10 k = 50 uV;
    // the ADC's offset 1.5 typ x3 + noise: 8 counts
    if constexpr (board_dac_load_ohms != 0) {
        verdict("the released wire reads ground (<= 8 counts: the 10 k)", wi <= 8);
    }
    D::init({.reference = Ref::v2048});
}

// ---- 4: OUTRANGE --------------------------------------------------------------------
// DS40002443A 32.5.1 says FULL 0, LOW 1, HIGH 2; the device header AUTO 0,
// LOW 2, HIGH 3 (A5). Each of the four codes written, CTRLA read back,
// the wire read near the bottom (code 64, 128 mV) and the top (code 1000,
// 2.0 V) under the 10 k: a range that cannot reach an end reads short.
void t4_outrange() {
    print(serial, "4 DAC OUTRANGE: the four codes, read back; the wire at 128 mV and 2.0 V under the 10 k", crlf);
    (void)A::init(clock, at(Ref::v2048));
    A::select(Wire{});
    uint16_t lo[4], hi[4];
    uint8_t back[4];
    for (uint8_t code = 0; code < 4; ++code) {
        D::init({.reference = Ref::v2048});
        D::regs().CTRLA = static_cast<uint8_t>((D::regs().CTRLA & ~DAC_OUTRANGE_gm) | (code << DAC_OUTRANGE_gp));
        back[code] = static_cast<uint8_t>((D::regs().CTRLA & DAC_OUTRANGE_gm) >> DAC_OUTRANGE_gp);
        dac_set(64);
        lo[code] = read_avg(8);
        dac_set(1000);
        hi[code] = read_avg(8);
        print(serial, "  OUTRANGE ", code, " (reads back ", back[code], "): 128 mV -> ", lo[code],
              " (exp 256)  2000 mV -> ", hi[code], " (exp 4000)", crlf);
    }
    verdict("each code reads back as written", back[0] == 0 && back[1] == 1 && back[2] == 2 && back[3] == 3);
    // the full range at both ends: the two references' 4 % + 27
    verdict("code 0 (full) reaches both ends", near(lo[0], 256, 37) && near(hi[0], 4000, two_refs(4000)));
    D::init({.reference = Ref::v2048});
    D::set(0);
}

// ---- 5: resolution and adjust -------------------------------------------------------
void t5_resolution() {
    print(serial, "5 12 vs 8 bit, left adjust", crlf);
    dac_to(Ref::v2048, 600);
    (void)A::init(clock, at(Ref::v2048));
    A::select(Wire{});
    const uint16_t r12 = read_avg(8);
    AdcConfig c8 = at(Ref::v2048);
    c8.resolution = AdcRes::bits8;
    (void)A::init(clock, c8);
    A::select(Wire{});
    const uint16_t r8 = read_avg(8);
    AdcConfig cl = at(Ref::v2048);
    cl.left_adjust = true;
    (void)A::init(clock, cl);
    A::select(Wire{});
    const uint16_t rl = A::read();
    print(serial, "  12-bit ", r12, "  8-bit ", r8, " (exp 12/16)  left-adjusted ", hex(rl), crlf);
    verdict("8-bit ~ 12-bit / 16 (another conversion: +-2 LSB8)", near(r8, r12 / 16, 2));
    verdict("left adjust = value << 4", near(rl >> 4, r12, 8) && (rl & 0x000F) == 0);
}

// ---- 6: differential --------------------------------------------------------------
using WireGnd = AdcDiff<Wire{}, AdcInput::gnd>;
using GndWire = AdcDiff<AdcInput::gnd, Wire{}>;
using WireDac = AdcDiff<Wire{}, AdcInput::dac0>;
using WireVrefa = AdcDiff<Wire{}, Vrefa{}>;

void t6_differential() {
    print(serial, "6 differential pairs as input values, a walk mixing them with single-ended inputs", crlf);
    dac_to(Ref::v2048, 700);                       // 1.4 V
    (void)A::init(clock, at(Ref::v2048, sd_internal));
    A::claim(WireVrefa{});
    A::select(Wire{});
    const uint16_t s = read_avg(8);
    A::select(WireGnd{});
    const int16_t wg = read_avg_signed(8);
    A::select(GndWire{});
    const int16_t gw = read_avg_signed(8);
    A::select(WireDac{});
    const int16_t wd = read_avg_signed(8);
    A::select(WireVrefa{});
    const int16_t wv = read_avg_signed(8);
    A::select(Wire{});
    const uint16_t s2 = read_avg(8);
    print(serial, "  single ", s, " | wire-gnd ", wg, " gnd-wire ", gw, " wire-dac0 ", wd, " wire-PD7 ", wv,
          " | single again ", s2, crlf);
    // 31.3.3.3: differential = (VINP - VINN) / VREF x 2048 = single / 2;
    // offset 1.5 typ x3 = 5
    verdict("wire - gnd = single / 2 (+-5)", near(wg, s / 2, 5));
    verdict("gnd - wire = -(wire - gnd) (+-5)", near(gw, -wg, 5));
    // the buffer's offset against the unbuffered node: +-20 mV (the DB's
    // +-10 mV spec x2; the EA's sheet gives none) = +-20 counts
    verdict("wire - dac0 = the buffer's offset, within +-20 mV", near(wd, 0, 20));
    verdict("two pins of one net: wire - PD7 ~ 0 (+-5)", near(wv, 0, 5));
    verdict("the single-ended input after the pairs reads as before (+-4)", near(s2, s, 4));
    verdict("selected() folds DIFF into bit 7", A::selected() == A::input_code(Wire{}) &&
                                                 A::input_code(WireGnd{}) == (1 | adc_diff_flag));
}

// ---- p: the PGA ------------------------------------------------------------------
// Each gain g at a DAC level V = 3.2 V / g (DAC on 4.096 V), single-ended
// through the PGA against the same level read direct: g x direct, both
// at 4.096 V, each a scaled burst of 64 (1/16 LSB). Gain error -0.2 ..
// -0.8 % typ. (table 35-26): bound -2.4 .. +0.6 % (three typicals), + the
// direct reading's offset 1.5 LSB12 over its counts. Then noise per gain
// (256 SAMPLEs' spread), the burst-through-PGA SAMPDUR 11 against 12
// (A11), and chopping on a GND-GND accumulation (A3).
void tp_pga() {
    print(serial, "p PGA: gains 1..16 against the direct path (4.096 V), noise, SAMPDUR 11/12, chopping", crlf);
    constexpr AdcGain gains[] = {AdcGain::x1, AdcGain::x2, AdcGain::x4, AdcGain::x8, AdcGain::x16};
    AdcConfig direct{.reference = Ref::v4096, .prescaler = AdcPresc::div10, .sample_duration = 11,
                     .accumulate = 64, .scaled = true};
    for (uint8_t i = 0; i < 5; ++i) {
        const uint8_t g = static_cast<uint8_t>(1u << i);
        const uint16_t code = static_cast<uint16_t>(800u / g);        // 3.2 V / g at 4.096 V
        dac_to(Ref::v4096, code);
        (void)A::init(clock, direct);
        A::select(Wire{});
        const uint32_t d = read_avg(4);                             // 65536 = 4.096 V
        AdcConfig pg = direct;
        pg.pga = true;
        pg.gain = gains[i];
        (void)A::init(clock, pg);
        A::select(Wire{});
        const uint32_t p = read_avg(4);
        // in units of 0.01 %: p / (g d) - 1; the direct reading's offset,
        // 1.5 LSB12 = 24 scaled counts, over its counts
        const int32_t err = d ? static_cast<int32_t>(p * 10000u / (d * g)) - 10000 : 0;
        const int32_t off = d ? static_cast<int32_t>(240'000UL / d) : 0;
        print(serial, "  x", g, ": direct ", d, " PGA ", p, " gain error ", err, " /10000 (bound -",
              240 + off, " .. +", 60 + off, ")", crlf);
        verdict("gain within -2.4 .. +0.6 % (+ the direct reading's offset)",
                err >= -(240 + off) && err <= 60 + off);
        // noise: 256 SAMPLEs' standard deviation at this gain, single 12-bit
        AdcConfig one{.reference = Ref::v4096, .prescaler = AdcPresc::div10, .sample_duration = 11,
                      .pga = true, .gain = gains[i]};
        (void)A::init(clock, one);
        A::select(Wire{});
        const int16_t first = static_cast<int16_t>(A::read());
        int32_t s = 0, s2 = 0;
        for (uint16_t k = 0; k < 256; ++k) {
            const int32_t v = static_cast<int16_t>(A::read()) - first;
            s += v; s2 += v * v;
        }
        // variance x 256^2 = 256 s2 - s^2; rms in tenths of an LSB
        const int32_t var = (256 * s2 - s * s);
        uint32_t rms10 = 0;
        while ((rms10 + 1) * (rms10 + 1) * 65536u / 100u <= static_cast<uint32_t>(var < 0 ? 0 : var)) ++rms10;
        print(serial, "    noise x", g, ": ", rms10 / 10, ".", rms10 % 10, " LSB12 rms = ",
              rms10 * 1000u / 10u / g, " uV at the input (table 35-26: ", 32u * g, " uV output-referred if so)", crlf);
    }
    // A11: a burst through the PGA at SAMPDUR 11 and 12 reads alike; 10 refused
    dac_to(Ref::v4096, 400);
    AdcConfig b11{.reference = Ref::vdd, .prescaler = AdcPresc::div4, .sample_duration = 11,
                  .accumulate = 16, .scaled = true, .pga = true};
    (void)A::init(clock, b11);
    A::select(Wire{});
    const uint16_t r11 = read_avg(4);
    AdcConfig b12 = b11;
    b12.sample_duration = 12;
    (void)A::init(clock, b12);
    A::select(Wire{});
    const uint16_t r12 = read_avg(4);
    AdcConfig b10 = b11;
    b10.sample_duration = 10;
    print(serial, "  burst through the PGA at 5 MHz: SAMPDUR 11 ", r11, ", 12 ", r12, " (scaled)", crlf);
    // two readings of one level: noise 1 LSB12 = 16 scaled, x3
    verdict("SAMPDUR 11 reads as 12 (31.3.3.6.6 against 31.3.3.6), +-48 scaled", near(r11, r12, 48));
    verdict("SAMPDUR 10 refused for a burst through the PGA", !A::init(clock, b10));
    // A3: GND - GND, 1024 accumulated, chopping off and on, at x1 and x16
    using GndGnd = AdcDiff<AdcInput::gnd, AdcInput::gnd>;
    for (uint8_t gi = 0; gi < 2; ++gi) {
        int32_t off[2];
        for (uint8_t ch = 0; ch < 2; ++ch) {
            AdcConfig c{.reference = Ref::v2048, .prescaler = AdcPresc::div10, .sample_duration = 11,
                        .accumulate = 1024, .pga = true, .gain = gi ? AdcGain::x16 : AdcGain::x1,
                        .chopping = ch != 0};
            (void)A::init(clock, c);
            A::select(GndGnd{});
            off[ch] = static_cast<int32_t>(A::read_sum());
        }
        print(serial, "  GND - GND x", gi ? 16 : 1, ": 1024 samples sum ", off[0], " plain, ", off[1],
              " chopped (", off[0] / 1024, " / ", off[1] / 1024, " LSB a sample)", crlf);
        // table 35-25's offset, 1.5 LSB typ., x3: 4.5 LSB a sample, 4608 in
        // the sum of 1024 - with and without chopping. Whether chopping
        // lowers it is printed, not judged: here it does not (adc.md).
        verdict("the offset within 4.5 LSB a sample, plain and chopped",
                (off[0] < 0 ? -off[0] : off[0]) <= 4608 && (off[1] < 0 ? -off[1] : off[1]) <= 4608);
    }
    D::set(0);
}

// ---- 8: accumulation ------------------------------------------------------------
void t8_accumulation() {
    print(serial, "8 accumulation: burst, series, scaled - 1..1024; RESULT's width; SAMPRDY per sample", crlf);
    dac_to(Ref::v2048, 512);                       // 1.024 V: 2048 counts at 2.048 V
    (void)A::init(clock, at(Ref::v2048));
    A::select(Wire{});
    const uint16_t one = read_avg(16);
    constexpr uint16_t accs[] = {2, 16, 32, 256, 1024};
    for (uint16_t a : accs) {
        AdcConfig c = at(Ref::v2048);
        c.accumulate = a;
        (void)A::init(clock, c);
        A::select(Wire{});
        const uint32_t sum = A::read_sum();
        c.scaled = true;
        (void)A::init(clock, c);
        A::select(Wire{});
        const uint16_t sc = A::read();
        c.scaled = false;
        c.series = true;
        (void)A::init(clock, c);
        A::select(Wire{});
        const uint32_t ser = A::read_sum();
        // per sample: sum / a; scaled: 65536 full scale = one x 16
        print(serial, "  N ", a, ": burst sum ", sum, " (", sum / a, "/sample)  scaled ", sc, " (exp ",
              static_cast<uint32_t>(one) * 16u, ")  series sum ", ser, crlf);
        // the mean of N against the 16-read mean of single conversions:
        // noise and the input's wander, +-4 LSB12 a sample; scaled +-64
        verdict("burst sum / N = the single reading (+-4)", near(static_cast<int32_t>(sum / a), one, 4));
        verdict("scaled = 16 x the single reading (+-64)", near(sc, static_cast<int32_t>(one) * 16, 64));
        verdict("series sum / N = the single reading (+-4)", near(static_cast<int32_t>(ser / a), one, 4));
    }
    // RESULT's width: N = 1024 unscaled at full scale is 22 bits
    AdcConfig cf = at(Ref::v1024);
    cf.accumulate = 1024;
    (void)A::init(clock, cf);
    A::select(Wire{});                             // 1.024 V at 1.024 V: full scale
    const uint32_t full = A::read_sum();
    print(serial, "  N 1024 at full scale: sum ", full, " (exp ~4095 x 1024 = 4193280), ",
          adc_sum_mv(full, A::result_steps(), 1024), " mV by adc_sum_mv", crlf);
    verdict("a 1024-sample sum carries 22 bits", full > 0x200000u && full <= 4095UL * 1024u);
    // a burst raises SAMPRDY per sample and RESRDY once
    AdcConfig cb = at(Ref::v2048);
    cb.accumulate = 64;
    (void)A::init(clock, cb);
    A::select(Wire{});
    ChCount::source(EvAdc0Sample{});
    PulseCounter<T1>::init(ChCount{});
    PulseCounter<T1>::reset();
    (void)A::read_sum();
    const uint16_t samples = PulseCounter<T1>::count();
    T1::disable();
    ChCount::off();
    verdict("one burst of 64: 64 SAMPRDY events, one result", samples == 64);
    verdict("an unscaled N >= 32 refused a window on RESULT at run time",
            !A::window(A::Window::inside, 0, 100));
    verdict("... and accepted on SAMPLE", A::window(A::Window::inside, 0, 100, AdcWindowSource::sample));
    A::window_off();
}

// ---- 7: the conversion rates (A12) ------------------------------------------------
// Tables 31-7 (fCLK_ADC = 5 MHz = 20 MHz / 4, PGABIASSEL 100 %) and 31-8
// (312.5 kHz = 20 MHz / 64, 25 %): free-running, the results (the
// samples of a burst) counted by the event system for 100 ms, LOWLAT on
// as the tables assume; the tables' figures in brackets. The verdict is
// against adc_period_cycles(), the driver's formula: the chapter's, with
// the output latency and the PGA's whole cycles the silicon adds (adc.hpp
// says which) - the bound 0.2 % + one count a window (the window is a
// cycle-counted delay: its loop and the counter's start and read, a few
// cycles of 2 million; the crystal +-50 ppm). The 8-bit column is
// printed, not judged: it runs 1..6 % under the formula, unexplained.
// Then two rows with LOWLAT off: does free-running pay the
// initialization per conversion?
struct RateRow { uint8_t sd; bool pga; uint32_t s8, s12, burst; };
constexpr RateRow t31_7[] = {
    {20, false, 161290, 142857, 147059}, {64, false, 66667, 63291, 64103},
    {128, false, 35971, 34965, 35211},  {255, false, 18797, 18519, 18587},
    {15, true, 120482, 109890, 156250},  {20, true, 107527, 99010, 135135},
    {64, true, 55249, 52910, 61728},     {255, true, 17762, 17513, 18382},
};
constexpr RateRow t31_8[] = {
    {2, false, 24038, 18382, 19531}, {11, false, 14205, 12019, 12500},
    {64, false, 4167, 3956, 4006},   {255, false, 1175, 1157, 1162},
    {2, true, 18116, 14706, 0},      {11, true, 11905, 10331, 18657},
    {64, true, 3943, 3754, 4480},    {255, true, 1156, 1139, 1198},
};
constexpr uint16_t rate_window_ms = 100;

uint32_t measure_rate(const AdcConfig& c) {
    if (!A::init(clock, c)) return 0;
    A::select(Wire{});
    A::start();                                    // free-running from here
    const uint16_t n = c.accumulate > 1 ? count_events(EvAdc0Sample{}, rate_window_ms)
                                        : count_events(EvAdc0Ready{}, rate_window_ms);
    A::stop();
    return static_cast<uint32_t>(n) * (1000u / rate_window_ms);
}
bool rate_ok(uint32_t got, const AdcConfig& c) {
    const uint32_t exp = hz / adc_period_cycles(hz, c);
    return near(static_cast<int32_t>(got), static_cast<int32_t>(exp),
                static_cast<int32_t>(exp / 500u + 1000u / rate_window_ms));
}

void rate_table(const RateRow* rows, uint8_t n, AdcPresc p, AdcPgaBias bias, bool lowlat) {
    for (uint8_t i = 0; i < n; ++i) {
        const RateRow& r = rows[i];
        AdcConfig c{.reference = Ref::vdd, .prescaler = p, .sample_duration = r.sd, .pga = r.pga,
                    .pga_bias = bias, .low_latency = lowlat, .free_running = true};
        c.resolution = AdcRes::bits8;
        const uint32_t g8 = measure_rate(c);
        c.resolution = AdcRes::bits12;
        const uint32_t g12 = measure_rate(c);
        const uint32_t f12 = hz / adc_period_cycles(hz, c);
        uint32_t gb = 0, fb = 0;
        AdcConfig cb = c;
        cb.accumulate = 1024;
        if (r.burst) {
            gb = measure_rate(cb);
            fb = hz / adc_period_cycles(hz, cb);
        }
        print(serial, "  SAMPDUR ", r.sd, r.pga ? " PGA" : "    ", ": 8-bit ", g8, " [", r.s8, "]  12-bit ", g12,
              " [", r.s12, "] formula ", f12, "  burst ", gb, " [", r.burst, "] formula ", fb, crlf);
        // with LOWLAT off too: free-running starts the next conversion
        // with the modules still up - no initialization between them
        verdict(lowlat ? "12-bit single rate = adc_period_cycles (0.2 % + 1 a window)"
                       : "LOWLAT off, free-running: the same rate (no initialization between)", rate_ok(g12, c));
        if (r.burst) verdict("burst sample rate = adc_period_cycles", rate_ok(gb, cb));
    }
}

void t7_rates() {
    print(serial, "7 rates, free-running, events counted 100 ms", crlf);
    dac_to(Ref::v2048, 512);
    print(serial, "  table 31-7: 5 MHz, PGA bias 100 %, LOWLAT", crlf);
    rate_table(t31_7, 8, AdcPresc::div4, AdcPgaBias::pct100, true);
    print(serial, "  table 31-8: 312.5 kHz, PGA bias 25 %, LOWLAT", crlf);
    rate_table(t31_8, 8, AdcPresc::div64, AdcPgaBias::pct25, true);
    print(serial, "  LOWLAT off, 5 MHz", crlf);
    rate_table(t31_7, 1, AdcPresc::div4, AdcPgaBias::pct100, false);
    rate_table(t31_7 + 4, 1, AdcPresc::div4, AdcPgaBias::pct100, false);
}

// ---- 9: initialization per trigger (table 31-5) --------------------------------------
// Software-started single conversions back to back (start, wait, read)
// for 50 ms, LOWLAT off then on: the difference per conversion is the
// initialization the chapter tables - ADC 10 us, PGA 20, an internal
// reference 60, the temperature sensor and the DAC input 35 (the slowest
// in parallel). Bound: +-20 % + 4 us (the state machine's granularity is
// not stated; the loop's own time cancels in the difference).
struct InitCase { const char* name; AdcConfig c; uint8_t input; uint8_t exp_us; };

uint32_t oneshot_period_ns(const AdcConfig& c, uint8_t input) {
    (void)A::init(clock, c);
    if (input == 0) A::select(Wire{}); else A::select(static_cast<AdcInput>(input));
    (void)A::read();                               // the first pays the start-up either way
    tbase_on();
    const uint16_t t0 = tbase();
    uint16_t n = 0;
    while (static_cast<uint16_t>(tbase() - t0) < 62500u) {   // 50 ms
        (void)A::read();
        ++n;
    }
    Tca<0>::reset();
    return n ? 50'000'000u / n : 0;
}

void t9_init_time() {
    print(serial, "9 initialization per trigger, LOWLAT off against on (table 31-5)", crlf);
    dac_to(Ref::v2048, 512);
    const InitCase cases[] = {
        {"VDD reference", AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div4}, 0, 10},
        {"the PGA", AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div4, .pga = true}, 0, 20},
        {"1.024 V internal", AdcConfig{.reference = Ref::v1024, .prescaler = AdcPresc::div10,
                                       .sample_duration = 8}, 0, 60},
        {"TEMPSENSE on VDD", AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div4,
                                       .sample_duration = 64}, static_cast<uint8_t>(AdcInput::temp), 35},
        {"DAC0 on VDD", AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div4,
                                  .sample_duration = 64}, static_cast<uint8_t>(AdcInput::dac0), 35},
    };
    for (const InitCase& k : cases) {
        AdcConfig off = k.c, on = k.c;
        on.low_latency = true;
        const uint32_t p_off = oneshot_period_ns(off, k.input);
        const uint32_t p_on = oneshot_period_ns(on, k.input);
        const int32_t init_us = (static_cast<int32_t>(p_off) - static_cast<int32_t>(p_on)) / 1000;
        print(serial, "  ", k.name, ": ", p_off, " ns a conversion LOWLAT off, ", p_on, " on -> ", init_us,
              " us (table: ", k.exp_us, ")", crlf);
        verdict(k.name, near(init_us, k.exp_us, k.exp_us / 5 + 4));
    }
}

// ---- 0: event start, series, select start ----------------------------------------
void t0_event_start() {
    print(serial, "0 event start: TCA0 overflow at 1 kHz -> START; series of 4; the select as the start", crlf);
    dac_to(Ref::v2048, 512);
    (void)A::init(clock, at(Ref::v2048));
    A::select(Wire{});
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1, .period = static_cast<uint16_t>(hz / 1000u - 1u)});
    ChPace::source(EvTcaOvf<0>{});
    A::start_on(ChPace{});
    const uint16_t n = count_events(EvAdc0Ready{}, 200);
    A::start_on_events(false);
    print(serial, "  ", n, " results in 200 ms (exp 200)", crlf);
    // 200 overflows of the crystal-timed TCA in a 200 ms crystal window: +-1 at the edges
    verdict("200 event-started conversions in 200 ms (+-1)", near(n, 200, 1));
    verdict("the value", near(A::result(), 2048, two_refs(2048)));
    AdcConfig s = at(Ref::v2048);
    s.accumulate = 4;
    s.series = true;
    (void)A::init(clock, s);
    A::select(Wire{});
    A::start_on(ChPace{});
    const uint16_t rs = count_events(EvAdc0Ready{}, 200);
    A::start_on_events(false);
    print(serial, "  series of 4 on the same pace: ", rs, " results in 200 ms (exp 50)", crlf);
    verdict("a series takes one trigger per sample: 50 (+-1)", near(rs, 50, 1));
    Tca<0>::reset();
    ChPace::off();
    // the select as the start: a walk of GND and the wire, one store each
    (void)A::init(clock, at(Ref::v2048));
    A::start_on_select(true);
    A::select(AdcInput::gnd);
    while (!A::ready()) {}
    const uint16_t g = A::result();
    A::select(Wire{});
    while (!A::ready()) {}
    const uint16_t w = A::result();
    A::start_on_select(false);
    print(serial, "  start on select: GND ", g, ", wire ", w, crlf);
    verdict("each select starts its own conversion (GND <= 8, the wire ~2048)",
            g <= 8 && near(w, 2048, two_refs(2048)));
}

// ---- w: the window comparator -------------------------------------------------------
void tw_window() {
    print(serial, "w window: four modes on RESULT, the flag written 1, strict INSIDE, signed, on SAMPLE", crlf);
    (void)A::init(clock, at(Ref::v2048));
    A::select(Wire{});
    D::init({.reference = Ref::v2048});
    struct Mode { const char* name; AdcWindow m; bool a, b, c; };
    const Mode modes[] = {
        {"below 1000", AdcWindow::below, true, false, false},
        {"above 3000", AdcWindow::above, false, false, true},
        {"inside", AdcWindow::inside, false, true, false},
        {"outside", AdcWindow::outside, true, false, true},
    };
    const uint16_t codes[] = {125, 500, 875};      // ~500, ~2000, ~3500 counts
    for (const Mode& m : modes) {
        (void)A::window(m.m, 1000, 3000);
        bool ok = true;
        for (uint8_t i = 0; i < 3; ++i) {
            dac_set(codes[i]);
            (void)A::read();
            const bool hit = A::window_hit();
            if (hit != (i == 0 ? m.a : i == 1 ? m.b : m.c)) ok = false;
        }
        verdict(m.name, ok);
    }
    // WCMP stays until written 1 (31.5.6): a matching conversion, then a
    // non-matching one without reading - the flag still up; result()
    // clears it.
    // No driver read between (each clears the flag): the conversions are
    // waited out - 60 us of the internal reference's settling and 11.5 us
    // of conversion, 200 us given.
    (void)A::window(AdcWindow::above, 0, 3000);
    dac_set(125);
    A::clear_window_flag();
    A::start(); delay_us(clock, 200);
    const bool stays_clear = !A::window_flag();
    dac_set(875);
    A::start(); delay_us(clock, 200);              // a match
    dac_set(125);
    A::start(); delay_us(clock, 200);              // then none, unread
    const bool still_up = A::window_flag();
    (void)A::result();
    const bool cleared = !A::window_flag();
    verdict("no match, no flag", stays_clear);
    verdict("a match's flag survives a later conversion (cleared only by a 1)", still_up);
    verdict("result() clears it (written 1)", cleared);
    // strict INSIDE at the boundary (A10): GND in 8-bit reads 0
    AdcConfig c8 = at(Ref::v2048);
    c8.resolution = AdcRes::bits8;
    (void)A::init(clock, c8);
    A::select(AdcInput::gnd);
    const uint16_t g = A::read();
    (void)A::window(AdcWindow::inside, 0, 10);
    (void)A::read();
    const bool in0 = A::window_hit();
    (void)A::window(AdcWindow::below, 1, 0);
    (void)A::read();
    const bool below1 = A::window_hit();
    print(serial, "  GND (8-bit) reads ", g, ": inside (0, 10) ", in0 ? "hit" : "-", ", below 1 ",
          below1 ? "hit" : "-", crlf);
    verdict("INSIDE is strict: 0 is not inside (0, 10)", g == 0 && !in0 && below1);
    // signed, differential: GND - wire(1.4 V) ~ -1400
    dac_to(Ref::v2048, 700);
    (void)A::init(clock, at(Ref::v2048));
    A::select(GndWire{});
    (void)A::window_signed(AdcWindow::below, -1000, 0);
    (void)A::read();
    const bool s1 = A::window_hit();
    (void)A::window_signed(AdcWindow::below, -2000, 0);
    (void)A::read();
    verdict("signed: below -1000 hits at ~-1400, below -2000 does not", s1 && !A::window_hit());
    // on SAMPLE in a burst: the match is per sample
    AdcConfig cb = at(Ref::v2048);
    cb.accumulate = 64;
    (void)A::init(clock, cb);
    A::select(Wire{});
    (void)A::window(AdcWindow::above, 0, 1000, AdcWindowSource::sample);
    (void)A::read_sum();
    verdict("on SAMPLE: a burst of samples above 1000 matches", A::window_hit());
    A::window_off();
    A::clear_window_flag();
    D::set(0);
}

// ---- e: the flags, the aborts (A8, A9), PGACTRL's reset (A6) ------------------------------
// A long conversion: 312.5 kHz, SAMPDUR 255: (255 + 15) / 312.5 kHz =
// 864 us after the ADC's 10 us of initialization (LOWLAT off, VDD).
AdcConfig long_cfg() {
    return AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div64, .sample_duration = 255};
}
// TCA0 at CLK_PER: 50 ns a count, 3.3 ms a wrap.
void fine_on() {
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1, .period = 0xFFFF});
}
uint16_t fine() { return Tca<0>::count(); }
/// RESRDY within `us`, or false (no letter waits forever on a flag).
bool wait_ready(uint32_t us) {
    while (us--) {
        if (A::ready()) return true;
        delay_us(clock, 1);
    }
    return A::ready();
}

void te_flags() {
    print(serial, "e flags and aborts: what clears RESRDY, the overwrite and overrun flags, STOP against the "
                  "conversion's end, mux writes mid-conversion, PGACTRL at reset", crlf);
    dac_to(Ref::v2048, 1000);                      // 2.0 V: 1638 at VDD
    (void)A::init(clock, at(Ref::vdd));
    A::select(Wire{});
    A::start(); (void)wait_ready(1000);
    (void)A::result();
    verdict("a 16-bit read of RESULT clears RESRDY (A8)", !A::ready());
    A::start(); (void)wait_ready(1000);
    (void)A::sample();
    verdict("a SAMPLE read leaves RESRDY", A::ready());
    (void)A::sum();
    verdict("... and the 32-bit read clears it", !A::ready());
    // two conversions, nothing read: SAMPOVR and RESOVR (each waited out:
    // 10 us of initialization + 11.5 us, 100 us given)
    (void)A::error();
    A::start(); delay_us(clock, 100);
    A::start(); delay_us(clock, 100);
    const uint8_t ovr = A::error();
    (void)A::result(); (void)A::sample();
    verdict("an unread result and sample overwritten: RESOVR and SAMPOVR",
            (ovr & adc_result_overwrite) && (ovr & adc_sample_overwrite));
    // a trigger during a conversion: TRIGOVR, ignored
    (void)A::init(clock, long_cfg());
    A::select(Wire{});
    (void)A::error();
    A::start();
    delay_us(clock, 100);
    A::start();
    const bool first = wait_ready(2000);
    const uint8_t tov = A::error();
    (void)A::result();
    const bool second = wait_ready(1000);
    verdict("a trigger mid-conversion: TRIGOVR, and one result only", (tov & adc_trigger_overrun) && first && !second);
    // STOP against the conversion's end (A9). The end measured first: start
    // to RESRDY in CLK_PER counts (it wanders by a CLK_ADC, 64 CLK_PER: the
    // start waits for the prescaler's edge). Then STOP from 6 us before
    // that end to the end, by 0.5 us, four tries each, and RESRDY polled
    // after the STOP: a result that rises AFTER the STOP is a conversion
    // that had passed its end, delivered through the output latency (0.5
    // CLK_ADC + a CLK_MAIN, figure 31-3 note 3) - the select engine's wait
    // must cover the longest such delay. Bound: 32 + 2 CLK_PER of latency,
    // + 8 of the poll loop, + 4 of the STOP store and the counter's read:
    // 46 CLK_PER.
    fine_on();
    uint16_t t_end = 0xFFFF;
    for (uint8_t k = 0; k < 3; ++k) {
        const uint16_t t0 = fine();
        A::start();
        while (!A::ready()) {}
        const uint16_t t = static_cast<uint16_t>(fine() - t0);
        if (t < t_end) t_end = t;
        (void)A::result(); (void)A::sample();
    }
    print(serial, "  start to RESRDY: ", t_end, " CLK_PER (", t_end / tpus, " us) at the earliest of three", crlf);
    uint16_t after_max = 0, after_n = 0, before_n = 0, none_n = 0;
    for (uint8_t d = 12; d != 0xFF; --d) {               // half microseconds: 6 us .. 0
        for (uint8_t k = 0; k < 4; ++k) {
            const uint16_t when = static_cast<uint16_t>(t_end - static_cast<uint16_t>(d) * tpus / 2u);
            const uint16_t t0 = fine();
            A::start();
            while (static_cast<uint16_t>(fine() - t0) < when) {}
            const bool already = A::ready();
            A::stop();
            const uint16_t ts = fine();
            uint16_t tr = 0;
            bool rose = false;
            while (static_cast<uint16_t>(fine() - ts) < 30u * tpus) {
                if (A::ready()) { tr = static_cast<uint16_t>(fine() - ts); rose = true; break; }
            }
            if (already) ++before_n;
            else if (rose) { ++after_n; if (tr > after_max) after_max = tr; }
            else ++none_n;
            (void)A::result(); (void)A::sample();
        }
    }
    Tca<0>::reset();
    print(serial, "  52 STOPs around the end: ", none_n, " aborted, ", before_n, " after the result, ", after_n,
          " with the result rising after the STOP, the latest ", after_max, " CLK_PER after it", crlf);
    verdict("a result rises after a STOP only within the output latency (<= 46 CLK_PER at CLK_PER / 64)",
            after_max <= 46);
    verdict("STOP aborts a conversion short of its end (some STOPs left no result)", none_n > 0);
    // a COMMAND rewrite to EVENT_TRIGGER mid-conversion
    A::start();
    delay_us(clock, 100);
    A::start_on_events(true);
    const bool ev_rewrite = wait_ready(2000);
    A::start_on_events(false);
    (void)A::result();
    print(serial, "  COMMAND rewritten to EVENT_TRIGGER mid-conversion: the conversion ",
          ev_rewrite ? "completed" : "was aborted", crlf);
    // the mux written mid-sampling: to another pin of the net, to GND
    A::select(Wire{});
    A::start();
    delay_us(clock, 100);
    A::select(Vrefa{});
    const bool pin_done = wait_ready(2000);
    const uint16_t pin_val = A::result();
    A::select(Wire{});
    A::start();
    delay_us(clock, 100);
    A::select(AdcInput::gnd);
    const bool gnd_done = wait_ready(2000);
    const uint16_t gnd_val = A::result();
    A::select(Wire{});
    A::start();
    const bool next_done = wait_ready(2000);
    const uint16_t next = A::result();
    print(serial, "  the mux moved mid-sampling: to PD7 (the same net) ", pin_done ? "completed, " : "aborted, ", pin_val,
          "; to GND ", gnd_done ? "completed, " : "aborted, ", gnd_val, "; the next conversion of the wire ",
          next_done ? "" : "(none) ", next, " (the wire 1638, GND 0)", crlf);
    verdict("the conversion after a mid-conversion mux write is clean (the wire, +-4 %)",
            next_done && near(next, 1638, two_refs(1638)));
    print(serial, "  PGACTRL read before any write: ", hex(pgactrl_at_reset),
          " (its register box says 0x04, its fields 0)", crlf);
}

// ---- i: internal inputs ----------------------------------------------------------------
using WireDacref0 = AdcDiff<Wire{}, AdcDacref::dacref0>;
using WireDacref1 = AdcDiff<Wire{}, AdcDacref::dacref1>;

uint16_t vdd_by_dac() {
    dac_to(Ref::v2048, 1000);
    (void)A::init(clock, AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div10,
                                   .sample_duration = sd_internal});
    A::select(AdcInput::dac0);
    const uint16_t counts = read_avg(16);
    return counts ? static_cast<uint16_t>(static_cast<uint32_t>(dac_mv(1000, D::steps, 2048)) * 4096u / counts) : 0;
}

void ti_internal() {
    print(serial, "i internal inputs: GND, VDDDIV10 against VDD by the DAC, TEMPSENSE, DACREF as the negative", crlf);
    (void)A::init(clock, at(Ref::v2048));
    A::select(AdcInput::gnd);
    const uint16_t g = read_avg(8);
    (void)A::init(clock, at(Ref::v1024, sd_internal));
    A::select(AdcInput::vdd_div10);
    const uint16_t v = read_avg(16);
    const uint16_t div10_mv = static_cast<uint16_t>(adc_mv(v, 4096, 1024) * 10u);
    const uint16_t dac_vdd = vdd_by_dac();
    print(serial, "  GND ", g, "  VDDDIV10 -> ", div10_mv, " mV, VDD by the DAC ", dac_vdd, " mV", crlf);
    verdict("GND <= 8 (offset 1.5 typ x3, noise)", g <= 8);
    // VDDDIV10's accuracy is not specified (A15); the two paths each carry
    // their references' +-2 %: 4 %
    verdict("VDDDIV10 = VDD by the DAC within 4 %", near(div10_mv, dac_vdd, dac_vdd * 4 / 100));
    // the temperature at 1.024 V, SAMPDUR >= 32 us x 2 MHz = 64 (31.3.3.7)
    (void)A::init(clock, at(Ref::v1024, 64));
    A::select(AdcInput::temp);
    const uint16_t t = read_avg(8);
    const uint16_t k = A::temp_kelvin(t);
    const int16_t s0 = static_cast<int16_t>(SIGROW.TEMPSENSE0), s1 = static_cast<int16_t>(SIGROW.TEMPSENSE1);
    // the DA/DB formula on the same reading, for comparison only
    const int32_t db_k = ((static_cast<int32_t>(static_cast<uint16_t>(s1)) - t) * static_cast<uint16_t>(s0) + 2048) / 4096;
    print(serial, "  TEMPSENSE raw ", t, " (TEMPSENSE0 ", s0, ", TEMPSENSE1 ", s1, ") -> ", k, " K = ",
          static_cast<int16_t>(k - 273), " C; the DB's formula would say ", db_k, " K", crlf);
    verdict("die temperature plausible (10..50 C)", k > 283 && k < 323);
    // DACREF0/1 as the negative (A14): the wire 1.0 V against 0.5 V and
    // 1.5 V of the comparators' DACREFs, a SAMPDUR sweep for their settling
    dac_to(Ref::v2048, 500);
    (void)Ac<0>::init({.positive = AcPos::ainp0, .negative = AcNeg::dacref, .reference = Ref::v2048,
                       .dacref = ac_dacref_code(500, 2048)});
    (void)Ac<1>::init({.positive = AcPos::ainp0, .negative = AcNeg::dacref, .reference = Ref::v2048,
                       .dacref = ac_dacref_code(1500, 2048)});
    hold_ms(1);
    constexpr uint8_t sds[] = {8, 32, 128, 255};
    int16_t d0[4], d1[4];
    for (uint8_t i = 0; i < 4; ++i) {
        (void)A::init(clock, at(Ref::v2048, sds[i]));
        A::select(WireDacref0{});
        d0[i] = read_avg_signed(8);
        A::select(WireDacref1{});
        d1[i] = read_avg_signed(8);
        print(serial, "  SAMPDUR ", sds[i], " (", sds[i] / 2, " us): wire - DACREF0 ", d0[i], " (exp 500), wire - DACREF1 ",
              d1[i], " (exp -500)", crlf);
    }
    Ac<0>::disable(); Ac<1>::disable();
    // the DAC's 7 mV + DACREF's 1 LSB8 (8 mV) + the references' 2 % of 1 V
    // + offset: 40 counts of 1 mV (differential at 2.048 V)
    verdict("wire - DACREF0 = +500 mV (+-40), settled at SAMPDUR 128", near(d0[2], 500, 40));
    verdict("wire - DACREF1 = -500 mV (+-40)", near(d1[2], -500, 40));
    D::set(0);
}

// ---- v: VREFA from the DAC ------------------------------------------------------------
void tv_vrefa() {
    print(serial, "v VREFA (PD7) from the DAC: DAC0 against it reads full scale; VDDDIV10 against it", crlf);
    D::init({.reference = Ref::v2048});
    bool ok = true;
    for (uint16_t code = 520; code <= 1020; code += 250) {        // >= 1.04 V (VREF min 1.024, table 35-23)
        dac_set(code);
        (void)A::init(clock, AdcConfig{.reference = Ref::vrefa, .prescaler = AdcPresc::div10,
                                       .sample_duration = sd_internal});
        A::select(AdcInput::dac0);
        const uint16_t r = read_avg(8);
        const uint16_t mv = dac_mv(code, D::steps, 2048);
        // the pad is the internal node plus the buffer's offset (+-20 mV)
        const uint16_t tol = static_cast<uint16_t>(4095UL * 20u / mv + 8u);
        print(serial, "  DAC ", code, " (", mv, " mV) as VREFA: DAC0 reads ", r, " (exp 4095 -", tol, ")", crlf);
        if (!near(r, 4095, tol)) ok = false;
    }
    verdict("full scale at every VREFA level, within the buffer's 20 mV", ok);
    dac_set(1020);
    A::select(AdcInput::vdd_div10);
    const uint16_t v = read_avg(8);
    const uint32_t exp = (static_cast<uint32_t>(vdd_mv) / 10u) * 4096u / dac_mv(1020, D::steps, 2048);
    print(serial, "  VDDDIV10 against VREFA = ", dac_mv(1020, D::steps, 2048), " mV: ", v, " (exp ", exp, ")", crlf);
    // the DAC's 2 % reference + VDDDIV10's 4 % (letter i) + the buffer
    verdict("VDDDIV10 against VREFA (+-7 %)", near(v, static_cast<int32_t>(exp), static_cast<int32_t>(exp * 7 / 100)));
    D::set(0);
}

// ---- s: the select under a hardware pace ------------------------------------------------
// The AnalogSampler's walk on an unbuffered mux. A TCA0 pace of 80 us
// starts conversions of ~56 us (312.5 kHz, SAMPDUR 2, LOWLAT: the output
// latency of 0.5 CLK_ADC is 1.6 us here, wide enough to land in); the
// RESRDY ISR records value and label (selected()); the loop, like the
// sampler's dispatch, selects the OTHER input after each result, `lag`
// microseconds late - swept over the whole period, so the select lands in
// every phase of the next conversion, its end and its output latency
// included. GND reads ~0, the wire (2.0 V at VDD) ~1638: a result is
// MISLABELLED when GND's label carries > 819 or the wire's < 819. The
// driver's select (STOP, the latency waited, the mux, the pace re-armed)
// against a bare MUXPOS store (the control: a mux write mid-conversion
// aborts the conversion, letter e, so only a write inside the output
// latency can mislabel - the sweep finds it, the engine must not).
volatile uint16_t s_value = 0;
volatile uint8_t s_label = 0;
volatile uint8_t s_new = 0;
volatile bool s_on = false;
constexpr uint16_t s_period_us = 80;

struct WalkStats { uint16_t results; uint16_t mislabel; uint16_t trigovr; };

WalkStats walk(bool engine, uint8_t lag_us, uint16_t results) {
    WalkStats w{0, 0, 0};
    (void)A::init(clock, AdcConfig{.reference = Ref::vdd, .prescaler = AdcPresc::div64, .sample_duration = 2,
                                   .low_latency = true});
    A::select(AdcInput::gnd);
    (void)A::error();
    s_new = 0;
    s_on = true;
    A::enable_resrdy_interrupt(true);
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1,
                        .period = static_cast<uint16_t>(s_period_us * tpus - 1u)});
    ChPace::source(EvTcaOvf<0>{});
    A::start_on(ChPace{});
    const uint8_t gnd = A::input_code(AdcInput::gnd), wire = A::input_code(Wire{});
    uint16_t guard = 0;
    while (w.results < results && ++guard < 60000u) {
        if (!s_new) { delay_us(clock, 1); continue; }
        cli();
        const uint16_t v = s_value;
        const uint8_t l = s_label;
        s_new = 0;
        sei();
        ++w.results;
        if ((l == gnd && v > 819) || (l == wire && v < 819)) ++w.mislabel;
        if (lag_us) delay_us(clock, lag_us);
        const bool to_wire = l == gnd;
        if (engine) {
            if (to_wire) A::select(Wire{}); else A::select(AdcInput::gnd);
        } else {
            A::regs().MUXPOS = to_wire ? wire : gnd;      // the bare store: the control
        }
        w.trigovr = static_cast<uint16_t>(w.trigovr + ((A::error() & adc_trigger_overrun) ? 1 : 0));
    }
    A::start_on_events(false);
    A::enable_resrdy_interrupt(false);
    s_on = false;
    Tca<0>::reset();
    ChPace::off();
    return w;
}

void ts_select_engine() {
    print(serial, "s the select under a hardware pace (80 us, conversions of ~56 us at 312.5 kHz): "
                  "mislabels, the engine against a bare mux store, the lag swept 0..78 us", crlf);
    dac_to(Ref::v2048, 1000);                      // the wire at 2.0 V
    A::claim(Wire{});
    uint32_t eng_mis = 0, eng_res = 0, bare_mis = 0, bare_res = 0;
    for (uint8_t lag = 0; lag < s_period_us; lag = static_cast<uint8_t>(lag + 2)) {
        const WalkStats e = walk(true, lag, 100);
        const WalkStats b = walk(false, lag, 100);
        eng_mis += e.mislabel; eng_res += e.results;
        bare_mis += b.mislabel; bare_res += b.results;
        if (e.mislabel || b.mislabel || lag % 10 == 0) {
            print(serial, "  lag ", lag, " us: engine ", e.results, " results, ", e.mislabel, " mislabelled, ",
                  e.trigovr, " overruns | bare store ", b.results, " results, ", b.mislabel, " mislabelled", crlf);
        }
    }
    print(serial, "  total: the engine ", eng_mis, " mislabelled of ", eng_res, ", the bare store ", bare_mis,
          " of ", bare_res, crlf);
    verdict("the engine labels every result right", eng_mis == 0 && eng_res >= 39u * 100u);
    verdict("the bare store mislabels somewhere in the sweep (the hazard is real)", bare_mis > 0);
    D::set(0);
}

// ---- x: extras -------------------------------------------------------------------------
using DynClock = DynamicClock<SysClock, Serial, A>;

void tx_extras() {
    print(serial, "x extras: DAC readback, a live rebase 20 -> 10 MHz, MCLKTIMEBASE 0 against 20 (A7)", crlf);
    dac_to(Ref::v2048, 512);
    verdict("DAC DATA readback", D::code() == 512);
    verdict("DynamicClock init", DynClock::init());
    (void)A::init(DynClock{}, at(Ref::v2048, sd_internal));
    A::select(AdcInput::dac0);
    const uint16_t at20 = read_avg(8);
    verdict("switch to 10 MHz", DynClock::set(10'000'000u));
    const uint16_t at10 = read_avg(8);
    print(serial, "  DAC0: ", at20, " at 20 MHz (CLK_ADC 2 MHz), ", at10, " at 10 MHz (CLK_ADC ",
          A::clock_hz_adc(), " Hz)", crlf);
    verdict("CLK_ADC kept and in range (clock_ok)", A::clock_ok());
    verdict("the reading does not move across the switch (+-8)", near(at10, at20, 8));
    verdict("back to 20 MHz", DynClock::set(20'000'000u));
    verdict("CLK_ADC back to init's 2 MHz (rebase aims at init's, not the last)",
            A::clock_hz_adc() == 2'000'000u);
    // A7: MCLKTIMEBASE 0 (its reset value) against the clock's 20: the
    // first conversion with an internal reference, LOWLAT off, right after
    // init - the 60 us settling counted in a time base of 0 cycles
    uint16_t first[2];
    for (uint8_t i = 0; i < 2; ++i) {
        ClockTimebase::write(i == 0 ? 0 : clock_timebase_for(hz));
        (void)A::init(clock, at(Ref::v2048, sd_internal));
        A::select(AdcInput::dac0);
        first[i] = A::read();
    }
    ClockTimebase::write(clock_timebase_for(hz));
    print(serial, "  the first conversion after init: TIMEBASE 0 -> ", first[0], ", TIMEBASE 20 -> ", first[1],
          " (exp ~2048)", crlf);
    verdict("TIMEBASE 20: the first conversion right", near(first[1], 2048, two_refs(2048)));
    D::set(0);
}

void register_tests() {
    bench.letter('n', "the analog net", tn_net);
    bench.letter('1', "references", t1_references);
    bench.letter('2', "DAC ramp", t2_ramp);
    bench.letter('3', "OUTEN off", t3_outen);
    bench.letter('4', "OUTRANGE", t4_outrange);
    bench.letter('5', "resolution", t5_resolution);
    bench.letter('6', "differential", t6_differential);
    bench.letter('p', "PGA", tp_pga);
    bench.letter('8', "accumulation", t8_accumulation);
    bench.letter('7', "rates", t7_rates);
    bench.letter('9', "initialization", t9_init_time);
    bench.letter('0', "event start", t0_event_start);
    bench.letter('w', "window", tw_window);
    bench.letter('e', "flags and aborts", te_flags);
    bench.letter('i', "internal inputs", ti_internal);
    bench.letter('v', "VREFA", tv_vrefa);
    bench.letter('s', "select engine", ts_select_engine);
    bench.letter('x', "extras", tx_extras);
}

void help() {
    print(serial, "test_avr_analog (AVR EA):", crlf);
    bench.menu();
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(ADC0_RESRDY_vect) {
    const uint16_t v = A::resrdy();
    const uint8_t l = A::selected();
    if (s_on) { s_value = v; s_label = l; s_new = 1; }
}

int main() {
    pgactrl_at_reset = A::regs().PGACTRL;          // A6: before any write
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    Ticker::pause();                               // no PIT ISR stretching the crystal-time holds
    sei();
    vdd_mv = vdd_by_dac();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_analog - the AVR EA's ADC/DAC/VREF (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " ", hz / 1'000'000u, " MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf,
          "the net PD6-PD1-PD7 with ", board_dac_load_ohms, " Ohm to ground; VDD measured ", vdd_mv, " mV", crlf);
    register_tests();
    help();
    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (!bench.handle(static_cast<char>(c))) { print(serial, "? for help", crlf); }
        bench.prompt();
    }
}

#else // the AVR DA/DB half

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrxt/ac.hpp"
#include "avrxt/adc.hpp"
#include "avrxt/clock.hpp"
#include "avrxt/dac.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/platform.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "avrxt/userrow.hpp"
#include "util/analog.hpp"
#include "util/print.hpp"

#include "avr_board.hpp"

using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using D = Dac<0>;
using A = Adc<0>;
using Wire = AnalogIn<Pin<'D', 1>>;            // PD6 -> PD1

uint16_t vdd_mv = 0;                            // measured at start (see main())

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

#if !defined(MVIO)
/// VDD on a part with no VDDDIV10 input (the DA: no MVIO): DAC0 at 2000 mV
/// from the 2.048 V reference, read internally against VDD as the ADC's
/// reference - VDD = 2000 mV x 4096 / counts. The DAC's +-10 LSB and the
/// reference's tolerance are the error, the same class as the DB's path.
uint16_t vdd_by_dac() {
    dac_default(Ref::v2048);
    D::set(1000);
    delay_us(clock, 300);
    A::init(clock, AdcConfig{.reference = Ref::vdd, .sample_length = internal_sample_length});
    A::select(AdcInput::dac0);
    A::flush();
    const uint16_t counts = read_avg(16);
    const uint32_t dac = dac_mv(1000, D::steps, 2048);
    return counts ? static_cast<uint16_t>(dac * 4096u / counts) : 0;
}
#endif

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
    // the unbuffered DAC output is high-impedance (measured)
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
    // MEASURED on rev A5, where the formula reads otherwise: INITDLY is
    // paid only for the first conversion after enable, not per start -
    // one-shots run at the plain rate.
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
#if defined(MVIO)
    A::select(AdcInput::vdd_div10);
    const uint16_t v = read_avg(8);
    A::select(AdcInput::vddio2_div10);
    const uint16_t v2 = read_avg(8);
    const uint16_t vdd_meas = adc_mv(v, 4096, 2048) * 10;
    print(serial, "  gnd ", g, "  vdd/10 -> VDD ~ ", vdd_meas, " mV (start-up: ", vdd_mv,
          ")  vddio2/10 -> ", adc_mv(v2, 4096, 2048) * 10, " mV", crlf);
    verdict("gnd ~ 0", g < 8);
    verdict("VDD/10 repeatable vs the start-up measurement (2 %)", near(vdd_meas, vdd_mv, vdd_mv / 50));
#else
    const uint16_t vdd_meas = vdd_by_dac();
    print(serial, "  gnd ", g, "  DAC0 against VDD -> VDD ~ ", vdd_meas, " mV (start-up: ",
          vdd_mv, "); no VDDDIV10 / VDDIO2DIV10 input on this part (no MVIO)", crlf);
    verdict("gnd ~ 0", g < 8);
    verdict("VDD by DAC0 repeatable vs the start-up measurement (2 %)",
            near(vdd_meas, vdd_mv, vdd_mv / 50));
#endif
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
    // THE PAD IS THE INTERNAL NODE PLUS THE BUFFER'S OFFSET: VREFA comes
    // off PD6 through the output buffer, the ADC reads DAC0 before it,
    // so full scale is short by that offset over the level - the same
    // +-20 mV test 5 grants the buffer, here in counts of each level
    // (79 at 1.04 V, 40 at 2.04 V). A fixed 30 was a 5 V die's luck.
    bool ok = true;
    for (uint16_t code = 520; code <= 1020; code += 250) {   // >= 1.04 V (VREFA min 1.024)
        D::set(code);
        delay_us(clock, 300);
        A::flush();
        const uint16_t r = read_avg(8);
        const uint16_t mv = dac_mv(code, D::steps, 2048);
        const uint16_t tol = static_cast<uint16_t>(4095UL * 20u / mv);
        print(serial, "  DAC ", code, " (", mv, " mV) as VREFA: ADC(dac0) = ", r, " (exp 4095 -", tol, "..+0)",
              crlf);
        if (!near(r, 4095, tol)) ok = false;
    }
    verdict("full scale for every VREFA level, within the buffer's 20 mV", ok);
#if defined(MVIO)
    // and VDD/10 against a known VREFA: 3300/10 = 330 mV / (DAC 1023 = 2046 mV) * 4096
    D::set(1023);
    delay_us(clock, 300);
    A::select(AdcInput::vdd_div10);
    A::flush();
    const uint16_t v = read_avg(8);
    const uint32_t exp = (static_cast<uint32_t>(vdd_mv) / 10) * 4096u / 2046u;
    print(serial, "  VDD/10 with VREFA = 2046 mV: ", v, " (exp ", exp, ")", crlf);
    verdict("VDD/10 vs VREFA", near(v, static_cast<int32_t>(exp), static_cast<int32_t>(exp / 8)));
#else
    print(serial, "  VDD/10 against VREFA: SKIP - no VDDDIV10 input on this part (no MVIO)", crlf);
#endif
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
#if defined(MVIO)
    A::init(clock, AdcConfig{.reference = Ref::v2048, .sample_length = internal_sample_length});
    A::select(AdcInput::vdd_div10);
    A::flush();
    vdd_mv = static_cast<uint16_t>(adc_mv(read_avg(16), 4096, 2048) * 10);
#else
    vdd_mv = vdd_by_dac();
#endif
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_analog - VREF/DAC/ADC self-test suite (board ", board,
          ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF",
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

#endif // ADC_PGAEN_bm
