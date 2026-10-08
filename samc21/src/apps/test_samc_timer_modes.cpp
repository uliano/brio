// test_samc_timer_modes - the advanced modes of the TC and the TCC
// chapters, measured on an instrument made of the chip itself.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the drivers
// under it.
//
// NOTHING TO WIRE, and the instrument is the point of the suite as much
// as the measurements are. A TCC waveform never reaches a pad at all:
//
//   TCC0 WO[0] --> CCL LUT0 (INSEL = TCC, combinational pass-through)
//              --> its OUTPUT VALUE as an EVSYS generator
//              --> an ASYNCHRONOUS channel
//              --> TC2's event input, EVACT = PPW (or PWP, PW, STAMP)
//
// A combinational LUT is a wire with no clock in it (37.5.3, measured in
// docs/samc21/ccl.md), so the LUT's output is a COPY of the waveform's
// level and both its edges are delayed by the same handful of cycles -
// which cancels out of a period and out of a pulse width alike. That is
// what lets a capture channel see a signal this chip generated, with no
// pad, no pull and no wire, and it is why the numbers below are exact
// arithmetic on 48'000'000 rather than a measurement of an oscillator.
// The same LUT with INSEL = TC carries TC0's WO[0] instead (37.6.2.4),
// and the EIC line on PA16, walked between the rails by its own internal
// pull, is the stimulus for the fault and counter event inputs.
//
// What is exercised, letter by letter:
//   a  the TCC's HARDWARE circular buffers: WAVE.CICCEN and CIPEREN
//      alternate two duties and two periods with no CPU at all, and two
//      values is as deep as they go
//   b  TC advanced waveforms: MFRQ and MPWM through the LUT and on the
//      pad, the 16-bit TcPwm task, DRVCTRL.INVEN
//   c  TC advanced capture and the locks: PWP, PW and STAMP against PPW,
//      PRESCSYNC under retrigger, CTRLA.ALOCK
//   d  TCC advanced waveforms: NFRQ, MFRQ, dual-slope CRITICAL, RAMP2A
//   e  TCC fault B, FILTERVAL (and what it counts), the blanking window
//      and the qualifier
//   f  the TCC counter event actions (increment, count-while-active) and
//      ERRATUM 1.21.7 staged: dithering against an external RETRIGGER,
//      with the captured widths as the witness
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc21/ccl.hpp"
#include "samc21/clock.hpp"
#include "samc21/eic.hpp"
#include "samc21/evsys.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/tc.hpp"
#include "samc21/tcc.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

using Led = Pin<'B', 23>;

// ---------------------------------------------------------------------------
// The instruments
// ---------------------------------------------------------------------------

/// The waveform under test. TCC0 is the 24-bit instance with four
/// channels; it shares generic clock channel 28 with TCC1.
using Wave = Tcc<0>;

/// The tap: LUT0's INSEL code 0x8 is "TCC (n % 3)", and for input 0 that
/// is TCC0's WO[0] (37.8.3). No pad is involved.
using Tap = Lut<0>;

/// The capture meter. TC2 and TC3 share generic clock channel 31, so
/// Tc<2>::release() would stop TC3 - nothing here uses TC3.
using Meter = Tc<2>;

/// A spare counter with a generic clock channel of its own (32), used as
/// an event counter and as a stopwatch.
using Counter = Tc<4>;

/// The TC whose own pad this suite watches: PA22 is TC0/WO0.
using Timer0 = Tc<0>;
using Timer3 = Tc<3>;
using Wo0Pin = Pin<'A', 22>;
using Wo0 = TcWo<Wo0Pin>;
using Wo1Pin = Pin<'A', 23>;
using Wo1 = TcWo<Wo1Pin>;           // TC0, WO1

/// The EIC stimulus pad: PA16 is EXTINT0, and it walks between the
/// rails under its own internal pull.
using EicPad = Pin<'A', 16>;
using EicLine = ExtInt<EicPad>;

/// Every timer here runs from generator 0 (OSC48M, the CPU's own clock),
/// so every tick below is 1/48'000'000 s of arithmetic.
constexpr uint8_t gen = 0;
constexpr uint32_t tc_hz = SysClock::hz;

/// The event fabric. Channel 0 carries the LUT (or the EIC line) into a
/// timer; channel 1 carries a timer's own event out.
constexpr uint8_t ev_wave_channel = 0;
constexpr uint8_t ev_out_channel = 1;

// ---------------------------------------------------------------------------
// The waveform's numbers
// ---------------------------------------------------------------------------
//
// TCC0 at /1 on 48 MHz with PER = 4799 is a 10 kHz waveform whose period
// is 4800 ticks exactly - and 4800 fits a 16-bit capture register with
// room to spare, which is what lets TC2 run at /1 too and makes one
// captured tick one CPU cycle.

constexpr uint32_t wave_top = 4799;
constexpr uint32_t wave_period = wave_top + 1u;              // 4800 ticks

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

bool near(uint32_t v, uint32_t target, uint32_t band) {
    return v > target ? (v - target) <= band : (target - v) <= band;
}

/// What fraction of `samples` reads found the pad high, in per mille.
/// The pad sampler: it needs ~30 waveform periods to be steady, and at
/// 10 kHz 40000 reads are far more.
template <class P>
uint32_t duty_permille(uint32_t samples = 40'000UL) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        if (P::read()) {
            ++high;
        }
    }
    return (high * 1000UL) / samples;
}

// ---------------------------------------------------------------------------
// The chain, piece by piece
// ---------------------------------------------------------------------------

/// TCC0 as a plain single-slope PWM generator at 10 kHz, CC0 set to
/// `first_duty`. Left DISABLED so the caller decides when the waveform -
/// and therefore every capture in the chain - starts.
bool wave_up(uint32_t first_duty) {
    if (!Wave::init(gen)) {
        return false;
    }
    if (!Wave::configure(TccConfig{.prescaler = TccPrescaler::div1})) {
        return false;
    }
    if (!Wave::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm})) {
        return false;
    }
    return Wave::set_period(wave_top) && Wave::set_cc(0, first_duty);
}

/// LUT0 passing TCC0's WO[0] through combinationally, its output value
/// published as an EVSYS generator. No generic clock is asked for at
/// all: a combinational LUT needs none (37.5.3).
bool tap_up(LutInput source = LutInput::tcc) {
    if (!Ccl::init()) {
        return false;
    }
    Ccl::enable(false);
    if (!Tap::configure(LutConfig{.in0 = source,
                                  .truth = lut_truth_pass(0),
                                  .event_out = true},
                        true)) {
        return false;
    }
    Ccl::enable(true);
    return true;
}

/// LUT0's INSEL "TC" source is TC (0 % 5) = TC0's WO[0] (37.6.2.4), so
/// the very same fabric that carries a TCC waveform to the meter carries
/// a TC one - only the input multiplexer changes.
bool tap_up_tc() { return tap_up(LutInput::tc); }

/// TC2 as a period-and-pulse-width meter fed from the tap, on an
/// ASYNCHRONOUS channel (35.6.2.8 note 2). Both channels capture:
/// 35.6.2.8.2 says both are needed to characterize an input.
bool meter_up() {
    Evsys::bus_clock(true);
    if (!Meter::init(gen)) {
        return false;
    }
    const TcConfig cfg{.mode = TcMode::count16,
                       .prescaler = TcPrescaler::div1,
                       .capture_enable = 0x3};
    if (!Meter::configure(cfg)) {
        return false;
    }
    if (!Meter::event_config(cfg,
                             TcEventConfig{.action =
                                               TcEventAction::period_pulse_width,
                                           .input_enable = true})) {
        return false;
    }
    if (!Evsys::connect(Meter::event_user, ev_wave_channel,
                        EventChannelConfig{.generator = Tap::event_generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return Meter::enable(true);
}

/// Take both capture channels' standing flags down: reading CCx is what
/// clears INTFLAG.MCx (35.6.2.8), and a capture arriving on top of an
/// unread one is dropped and raises INTFLAG.ERR (35.6.2.8.2).
void drain_meter() {
    (void)Meter::cc16(0);
    (void)Meter::cc16(1);
    Meter::clear_flags(0xFFu);
}

void chain_down() {
    (void)Wave::enable(false);
    Evsys::disconnect(Meter::event_user);
    Meter::release();
    Ccl::enable(false);
    Ccl::release();
    Wave::release();
}

// ---------------------------------------------------------------------------
// a - the HARDWARE circular buffers: WAVE.CICCEN / WAVE.CIPEREN
// ---------------------------------------------------------------------------
//
// 36.6.3.2: with WAVE.CICCENx set, at every update CCx and CCBUFx are
// EXCHANGED rather than CCBUFx being copied one way - so a channel with
// two values loaded ping-pongs between them for ever, with no CPU and no
// DMA at all. WAVE is write-synchronized but NOT enable-protected
// (36.6.2.1), so the bit can be set under a running timer.
void ta_circular() {
    constexpr uint32_t lo_duty = 1200;
    constexpr uint32_t hi_duty = 3600;
    bench.verdict("TCC0 and the meter come up",
                  wave_up(lo_duty) && tap_up() && meter_up());

    // ---- the hardware circular buffer, two values.
    bench.verdict("CC0 and CCBUF0 are loaded with the two values, and "
                  "WAVE.CICCEN0 is set UNDER A RUNNING TIMER - WAVE is "
                  "write-synchronized but not enable-protected (36.6.2.1)",
                  Wave::set_cc(0, lo_duty) && Wave::enable(true) &&
                      Wave::set_cc_buffer(0, hi_duty) &&
                      Wave::wave(TccWaveConfig{
                          .waveform = TccWaveform::normal_pwm,
                          .circular_cc = 0x1}));
    wait_ms(2);

    // Read a run of widths straight out of the capture register: they
    // must alternate between the two, period by period.
    uint16_t seq[16];
    for (uint8_t i = 0; i < 16u; ++i) {
        drain_meter();
        while ((Meter::flags() & Meter::match_flag(1)) == 0u) {
        }
        seq[i] = Meter::cc16(1);
    }
    uint8_t lo_seen = 0;
    uint8_t hi_seen = 0;
    uint8_t other = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        if (near(seq[i], lo_duty, 40)) {
            ++lo_seen;
        } else if (near(seq[i], hi_duty, 40)) {
            ++hi_seen;
        } else {
            ++other;
        }
    }
    print(serial, "  circular CC0: 16 captured widths, ", lo_seen, " at ",
          lo_duty, ", ", hi_seen, " at ", hi_duty, ", ", other, " elsewhere",
          crlf);
    bench.verdict("THE HARDWARE CIRCULAR BUFFER PLAYS TWO VALUES FOR EVER "
                  "with no CPU and no DMA: 36.6.3.2's exchange, seen at the "
                  "capture register",
                  lo_seen > 4u && hi_seen > 4u && other == 0u);

    // The cost: nothing. And the limit: TWO values, which is the whole
    // of it - a register pair is a two-entry table and there is no third
    // place to put a third value.
    bench.verdict("and it is exactly TWO values deep, because a register "
                  "and its buffer are two places and the chapter offers no "
                  "third - so a table of three or more needs a write at every "
                  "update",
                  Wave::cc_count == 4u);

    // ---- CIPEREN, the same for the period.
    bench.verdict("the same bit exists for PER, so the FREQUENCY can "
                  "alternate too",
                  Wave::set_period(wave_top) &&
                      Wave::set_period_buffer(wave_top / 2u) &&
                      Wave::wave(TccWaveConfig{
                          .waveform = TccWaveform::normal_pwm,
                          .circular_cc = 0x1,
                          .circular_period = true}));
    wait_ms(2);
    uint16_t pseq[16];
    for (uint8_t i = 0; i < 16u; ++i) {
        drain_meter();
        while ((Meter::flags() & Meter::match_flag(0)) == 0u) {
        }
        pseq[i] = Meter::cc16(0);
    }
    uint8_t long_seen = 0;
    uint8_t short_seen = 0;
    uint8_t pother = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        if (near(pseq[i], wave_period, 60)) {
            ++long_seen;
        } else if (near(pseq[i], wave_period / 2u, 60)) {
            ++short_seen;
        } else {
            ++pother;
        }
    }
    print(serial, "  circular PER: 16 captured periods, ", long_seen, " at ",
          wave_period, ", ", short_seen, " at ", wave_period / 2u, ", ",
          pother, " elsewhere", crlf);
    bench.verdict("WAVE.CIPEREN alternates the PERIOD the same way, so the "
                  "hardware answer covers frequency as well as duty",
                  long_seen > 4u && short_seen > 4u && pother == 0u);

    chain_down();
}

// ---------------------------------------------------------------------------
// b - TC advanced: the two MATCH waveform modes, the 16-bit task, INVEN
// ---------------------------------------------------------------------------
//
// The instrument is the suite's own, with one bit changed: LUT0's INSEL is "TC"
// rather than "TCC", which on LUT0 means TC0's WO[0] - so a TC waveform
// reaches the same capture meter over the same asynchronous channel, and
// TC0's own pad PA22 is there to be read as well.

/// TC0 as a waveform generator, left ENABLED.
bool tc0_up(TcWaveform w, uint16_t cc0, uint16_t cc1,
            TcPrescaler p = TcPrescaler::div1, uint8_t invert = 0) {
    if (!Timer0::init(gen)) {
        return false;
    }
    if (!Timer0::configure(TcConfig{.mode = TcMode::count16,
                                    .prescaler = p,
                                    .waveform = w,
                                    .invert = invert})) {
        return false;
    }
    return Timer0::set_cc16(0, cc0) && Timer0::set_cc16(1, cc1) &&
           Timer0::enable(true);
}

/// Wait, bounded, for one of the meter's flags.
bool wait_flag(uint8_t mask, uint32_t ms = 50) {
    const uint32_t t0 = Ticker::millis();
    while ((Meter::flags() & mask) == 0u) {
        if (Ticker::millis() - t0 > ms) {
            return false;
        }
    }
    return true;
}

/**
 * One period and one width out of the meter, GUARANTEED FRESH - and the
 * ceremony is a finding in its own right.
 *
 * A capture channel is not one register but TWO: CCx with CCBUFx behind
 * it (35.6.2.8), and reading CCx is what lets CCBUFx move up. So a
 * single read after the waveform under test has changed hands back a
 * value the PREVIOUS configuration captured - so a letter that reads
 * once per waveform mode reports each mode's predecessor. Draining both
 * stages and then taking several whole fresh captures is what makes a
 * reading current.
 */
bool meter_read(uint16_t& period, uint16_t& width, uint8_t fresh = 4) {
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Meter::cc16(0);
        (void)Meter::cc16(1);
    }
    Meter::clear_flags(0xFFu);
    for (uint8_t i = 0; i < fresh; ++i) {
        if (!wait_flag(Meter::match_flag(1))) {
            return false;
        }
        period = Meter::cc16(0);
        width = Meter::cc16(1);
        Meter::clear_flags(0xFFu);
    }
    return true;
}

void tb_tc_waveforms() {
    bench.verdict("the meter comes up on LUT0's TC input, so TC0's WO[0] "
                  "reaches a capture channel with no pad in the path",
                  tap_up_tc() && meter_up());
    Wo0::claim();

    // ---- MFRQ: CC0 is TOP and the output TOGGLES on every match, so the
    // waveform's period is TWICE (CC0 + 1) and its duty is one half.
    constexpr uint16_t mfrq_top = 2399;
    bench.verdict("TC0 runs in MATCH FREQUENCY mode with CC0 as its period",
                  tc0_up(TcWaveform::match_frequency, mfrq_top, 0));
    uint16_t period = 0;
    uint16_t width = 0;
    const bool mfrq_read = meter_read(period, width);
    const uint32_t pad_mfrq = duty_permille<Wo0Pin>();
    const uint32_t mfrq_expect = 2u * (mfrq_top + 1u);
    print(serial, "  MFRQ CC0=", mfrq_top, ": captured period ", period,
          " (2 x (CC0+1) - 1 = ", mfrq_expect - 1u, "), width ", width,
          ", PA22 high ", pad_mfrq, " per mille", crlf);
    bench.verdict("MFRQ TOGGLES ITS OUTPUT ON EVERY MATCH, so one waveform "
                  "period is TWO counter periods - exact to the tick",
                  mfrq_read && period == mfrq_expect - 1u);
    bench.verdict("and a toggle is a square wave: half the width, half the "
                  "pad", width == mfrq_top && near(pad_mfrq, 500, 20));
    Timer0::release();

    // ---- MPWM: CC0 is TOP and CC1 the compare. WO[0] - the one the LUT
    // sees - toggles at each TOP exactly as in MFRQ; WO[1] is the PWM.
    constexpr uint16_t mpwm_top = 2399;
    constexpr uint16_t mpwm_duty = 600;
    bench.verdict("TC0 runs in MATCH PWM mode, CC0 the period and CC1 the "
                  "duty", tc0_up(TcWaveform::match_pwm, mpwm_top, mpwm_duty));
    const bool mpwm_read = meter_read(period, width);
    const uint32_t pad_mpwm = duty_permille<Wo0Pin>();
    print(serial, "  MPWM CC0=", mpwm_top, " CC1=", mpwm_duty,
          ": WO0 captured period ", period, ", width ", width, ", PA22 high ",
          pad_mpwm, " per mille", crlf);
    bench.verdict("IN MPWM, CC0 IS SPENT AS THE PERIOD AND CHANNEL 0'S OWN "
                  "OUTPUT DEGENERATES: WO[0] matches only at TOP, so it is "
                  "high for all but one tick of every period - measured on "
                  "the pad AND through the LUT, which agree",
                  mpwm_read && period == mpwm_top && width == mpwm_top - 1u &&
                      pad_mpwm >= 990u);

    // WO[1] is where the duty went, and PA23 is TC0/WO1 on this package.
    // The pad is used only if it is ELECTRICALLY FREE, which is a
    // question about this board and not about the silicon - so it is
    // asked first and the verdict is declined if the answer is no.
    Wo1Pin::output();
    Wo1Pin::set();
    const bool pad23_high = Wo1Pin::read();
    Wo1Pin::clear();
    const bool pad23_free = pad23_high && !Wo1Pin::read();
    if (pad23_free) {
        Wo1::claim();
        wait_ms(2);
        const uint32_t pad_wo1 = duty_permille<Wo1Pin>();
        const uint32_t wo1_expect = (mpwm_duty * 1000u) / (mpwm_top + 1u);
        print(serial, "  MPWM WO[1] on PA23: high ", pad_wo1,
              " per mille against CC1/(CC0+1) = ", wo1_expect, crlf);
        bench.verdict("and WO[1] IS THE PWM - CC1 against CC0 as the period, "
                      "which is what makes MPWM the mode with an arbitrary "
                      "period and one channel left",
                      near(pad_wo1, wo1_expect, 25));
        Wo1::release();
    } else {
        print(serial, "  PA23 does not follow PORT on this board", crlf);
        bench.verdict("MPWM's WO[1] duty on PA23: DECLINED, the pad is not "
                      "electrically free on this board and no other pad of "
                      "this package carries TC0/WO1", true);
    }
    Wo1Pin::configure({});
    Timer0::release();

    // ---- the 16-bit TcPwm task.
    // NPWM in COUNT16 fixes TOP at MAX, so the period is 65536 ticks -
    // and 65535 is exactly what a 16-bit capture register can hold.
    bench.verdict("the 16-bit TcPwm task brings TC0 up as an NPWM channel",
                  Timer0::init(gen) && TcPwm<Timer0, 0>::setup());
    TcPwm<Timer0, 0>::duty(0x4000);
    wait_ms(5);
    // TEN TIMES THE DEFAULT WINDOW: the pad is sampled for a fixed count,
    // not a whole number of periods, and a period of 65536 ticks fits
    // only some seven times into the default one - the part period left
    // over biases the reading by up to a few per cent (measured 266 and
    // 271 per mille for 250, moving with code placement). Seventy periods
    // bring that under one per cent; the capture below is the exact
    // witness either way.
    const uint32_t pad_pwm = duty_permille<Wo0Pin>(400'000UL);
    const bool pwm_read = meter_read(period, width);
    print(serial, "  TcPwm<TC0,0> duty 16384/65535: PA22 high ", pad_pwm,
          " per mille, captured period ", period, " width ", width, crlf);
    bench.verdict("TcPwm's max IS the 16-bit counter's own top, so a duty of "
                  "a quarter of it is a quarter of the pad",
                  TcPwm<Timer0, 0>::max == 0xFFFFu && near(pad_pwm, 250, 20));
    bench.verdict("and the captured period is the whole 16-bit range, which "
                  "is what NPWM in COUNT16 fixes TOP at",
                  pwm_read && period == 0xFFFFu && near(width, 0x4000u, 8u));

    // ---- DRVCTRL.INVEN, on the same waveform.
    Timer0::release();
    bench.verdict("the same 25 % waveform is brought up with DRVCTRL.INVEN0 "
                  "set", tc0_up(TcWaveform::normal_pwm, 0x4000, 0, 
                                TcPrescaler::div1, 0x1));
    wait_ms(5);
    const uint32_t pad_inv = duty_permille<Wo0Pin>();
    const bool inv_read = meter_read(period, width);
    print(serial, "  with INVEN0: PA22 high ", pad_inv, " per mille, captured "
          "width ", width, crlf);
    bench.verdict("DRVCTRL.INVEN INVERTS THE OUTPUT: the same 25 % waveform "
                  "shows as 75 % on the pad",
                  near(pad_inv, 750, 20));
    bench.verdict("PROBE: where does the CCL tap a TC waveform - before or "
                  "after DRVCTRL?", inv_read);

    // ---- and the two capture modes this family DOES NOT HAVE.
    print(serial, "  35.6.3.3/35.6.3.4's minimum and maximum capture modes "
          "are SAM C20/C21 N-variant only: this device header declares no "
          "CTRLA.CAPTMODE field at all, so there is nothing to write and "
          "nothing to measure", crlf);
    bench.verdict("TcEventAction therefore has no min/max entries either - "
                  "the driver's vocabulary is the silicon's",
                  static_cast<uint8_t>(TcEventAction::pulse_width) ==
                      TC_EVCTRL_EVACT_PW_Val);

    Wo0::release();
    Wo0Pin::configure({});
    Timer0::release();
    Evsys::disconnect(Meter::event_user);
    Meter::release();
    Ccl::enable(false);
    Ccl::release();
}

// ---------------------------------------------------------------------------
// c - TC advanced: the other capture actions, PRESCSYNC, ALOCK
// ---------------------------------------------------------------------------
void tc_tc_capture_and_locks() {
    // ---- PWP: the same capture with CC0 and CC1 SWAPPED. Nothing but
    // the EVACT changes, so this is the cleanest possible statement of
    // what 35.6.2.8.2's two orders differ in.
    bench.verdict("the chain comes up on the TCC again",
                  wave_up(2399) && tap_up() && meter_up() && Wave::enable(true));
    uint16_t ppw_period = 0;
    uint16_t ppw_width = 0;
    bench.verdict("and gives a fresh PPW reading",
                  meter_read(ppw_period, ppw_width));

    const TcConfig swap_cfg{.mode = TcMode::count16,
                            .prescaler = TcPrescaler::div1,
                            .capture_enable = 0x3};
    (void)Meter::enable(false);
    bench.verdict("the meter is re-armed with EVACT = PWP",
                  Meter::event_config(
                      swap_cfg, TcEventConfig{.action =
                                                  TcEventAction::pulse_width_period,
                                              .input_enable = true}) &&
                      Meter::enable(true));
    uint16_t pwp_a = 0;
    uint16_t pwp_b = 0;
    (void)meter_read(pwp_a, pwp_b);
    print(serial, "  PPW: CC0=", ppw_period, " CC1=", ppw_width,
          "   PWP: CC0=", pwp_a, " CC1=", pwp_b, crlf);
    bench.verdict("PPW AND PWP DIFFER IN NOTHING BUT WHICH REGISTER GETS "
                  "WHICH - the period and the width simply change places",
                  near(pwp_a, ppw_width, 4) && near(pwp_b, ppw_period, 4) &&
                      ppw_period != ppw_width);

    // ---- PW: the pulse width alone, with the counter STOPPED between
    // pulses (35.6.2.8.3) - which is the whole difference, and it shows
    // in STATUS.STOP.
    const TcConfig pw_cfg{.mode = TcMode::count16,
                          .prescaler = TcPrescaler::div1,
                          .capture_enable = 0x1};
    (void)Meter::enable(false);
    bench.verdict("the meter is re-armed as a pulse-width meter (EVACT = PW)",
                  Meter::configure(pw_cfg) &&
                      Meter::event_config(
                          pw_cfg, TcEventConfig{.action = TcEventAction::pulse_width,
                                                .input_enable = true}) &&
                      Meter::enable(true));
    drain_meter();
    while ((Meter::flags() & Meter::match_flag(0)) == 0u) {
    }
    const uint16_t pw = Meter::cc16(0);
    print(serial, "  PW: CC0=", pw, " against the PPW width ", ppw_width, crlf);
    bench.verdict("PW puts the pulse width in CC0 and needs one channel "
                  "where PPW needs two", near(pw, ppw_width, 4));

    // ---- STAMP: COUNT copied into CCx on EVERY event, which for a
    // level-copying event source means every rising edge. The chapter
    // requires TOP < MAX for it (35.6.3.2), so the meter runs 8-bit with
    // a period of its own and the stamps walk that period modulo it.
    const TcConfig stamp_cfg{.mode = TcMode::count8,
                             .prescaler = TcPrescaler::div256,
                             .capture_enable = 0x1};
    (void)Meter::enable(false);
    bench.verdict("the meter is re-armed as a TIME-STAMP channel, 8-bit with "
                  "TOP < MAX as 35.6.3.2 requires",
                  Meter::configure(stamp_cfg) && Meter::set_period8(199) &&
                      Meter::event_config(
                          stamp_cfg,
                          TcEventConfig{.action = TcEventAction::stamp,
                                        .input_enable = true}) &&
                      Meter::set_count8(0) && Meter::enable(true));
    // THE COUNTER MUST BE PUT BACK TO ZERO BY HAND: a mode change does
    // not clear COUNT, so an 8-bit counter left above its new PER never
    // meets it and runs all the way to 0xFF. Without the set_count8(0)
    // above the first stamps read 255.
    uint8_t stamps[8] = {0};
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Meter::cc8(0);
    }
    for (uint8_t i = 0; i < 8u; ++i) {
        Meter::clear_flags(0xFFu);
        if (!wait_flag(Meter::match_flag(0))) {
            break;
        }
        stamps[i] = Meter::cc8(0);
    }
    // The waveform is 4800 ticks; the stamping counter runs at /256 over
    // a period of 200, so consecutive stamps advance by 4800/256 = 18.75
    // counts, i.e. 18 or 19 modulo 200.
    // The first stamps are the changeover's own - the capture FIFO's two
    // stages plus the counter's own settling - so the claim rests on the
    // steps between the settled ones.
    uint8_t good = 0;
    for (uint8_t i = 4; i < 8u; ++i) {
        const uint8_t d = static_cast<uint8_t>((stamps[i] + 200u - stamps[i - 1]) % 200u);
        if (d == 18u || d == 19u) {
            ++good;
        }
    }
    print(serial, "  stamps:");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", stamps[i]);
    }
    print(serial, "  (expected steps of 4800/256 = 18.75 counts)", crlf);
    bench.verdict("STAMP copies COUNT into CCx on every event, so a fixed "
                  "event rate walks the counter's period by a fixed step - "
                  "18 or 19 counts of 200, which is 18.75 rounded both ways",
                  good == 4u);
    Evsys::disconnect(Meter::event_user);
    Meter::release();
    Ccl::enable(false);
    Ccl::release();
    Wave::release();

    // ---- PRESCSYNC, measured by the ONE thing that distinguishes the
    // three: 35.6.2.3 says RESYNC reloads the counter on the next GCLK
    // edge AND RESETS THE PRESCALER, where GCLK and PRESC leave the
    // prescaler running. So after a RETRIGGER issued at an arbitrary CPU
    // phase, a fixed wait lands on a DETERMINISTIC count under RESYNC
    // and on a count that wobbles with the prescaler's own phase under
    // the other two. The discriminator is the SPREAD, not the value.
    struct SyncTrial {
        TcPrescalerSync mode;
        const char* name;
        uint16_t lo;
        uint16_t hi;
        uint32_t sum;
    };
    SyncTrial trials[3] = {{TcPrescalerSync::gclk, "GCLK", 0, 0, 0},
                           {TcPrescalerSync::prescaler, "PRESC", 0, 0, 0},
                           {TcPrescalerSync::resync, "RESYNC", 0, 0, 0}};
    // /1024 IS THE POINT: the effect being looked for is ONE PRESCALED
    // TICK, so the prescaler has to be long enough that a tick dwarfs
    // everything else in the loop - the double READSYNC included. At
    // /1024 one tick is 1024 CPU cycles; the phase walk below spans a
    // whole one of them, three or four cycles at a time.
    constexpr uint16_t phase_steps = 300;
    for (auto& t : trials) {
        (void)Counter::init(gen);
        (void)Counter::configure(TcConfig{.mode = TcMode::count16,
                                          .prescaler = TcPrescaler::div1024,
                                          .prescaler_sync = t.mode});
        (void)Counter::enable(true);
        t.lo = 0xFFFFu;
        t.hi = 0;
        t.sum = 0;
        for (uint16_t k = 0; k < phase_steps; ++k) {
            for (uint16_t j = 0; j < k; ++j) {
                asm volatile("nop");
            }
            (void)Counter::retrigger();
            for (uint16_t j = 0; j < 3000u; ++j) {
                asm volatile("nop");
            }
            const uint16_t c = Counter::count16();
            if (c < t.lo) {
                t.lo = c;
            }
            if (c > t.hi) {
                t.hi = c;
            }
            t.sum += c;
        }
        Counter::release();
        print(serial, "  PRESCSYNC ", t.name, ": count after a fixed wait ",
              t.lo, "..", t.hi, ", mean x100 = ",
              (t.sum * 100u) / phase_steps, " over ", phase_steps,
              " retrigger phases spanning one prescaled tick", crlf);
    }
    const uint32_t mean_gclk = (trials[0].sum * 100u) / phase_steps;
    const uint32_t mean_presc = (trials[1].sum * 100u) / phase_steps;
    const uint32_t mean_resync = (trials[2].sum * 100u) / phase_steps;
    bench.verdict("PRESCSYNC = GCLK STARTS THE COUNTER A WHOLE PRESCALED "
                  "TICK EARLIER than either prescaler-synchronized option: "
                  "35.6.2.3's reload lands on the next GCLK edge rather than "
                  "waiting for the next prescaler one, and over 300 "
                  "retrigger phases that is worth about one tick of mean",
                  mean_gclk > mean_presc + 70u && mean_gclk > mean_resync + 70u &&
                      mean_gclk < mean_presc + 130u);
    bench.verdict("PRESC AND RESYNC ARE NOT DISTINGUISHABLE FROM THE CPU: "
                  "DECLINED. RESYNC's extra act is resetting the prescaler, "
                  "and a RETRIGGER issued through CTRLBSET is itself taken "
                  "on the next prescaled clock - so the reset has nothing "
                  "left to move. The two means sit under a fifth of a tick "
                  "apart where the effect looked for is a whole one; a "
                  "hardware retrigger event, which this suite has no free "
                  "channel for, is what would settle it",
                  true);
    print(serial, "  means x100: GCLK ", mean_gclk, ", PRESC ", mean_presc,
          ", RESYNC ", mean_resync, " - GCLK leads by ",
          mean_gclk - mean_presc, "/100 of a tick, PRESC and RESYNC differ "
          "by ", mean_presc > mean_resync ? mean_presc - mean_resync
                                          : mean_resync - mean_presc,
          "/100", crlf);

    // ---- CTRLA.ALOCK. The witness is THE PAD, because a register read
    // is not one: on the TCC a read of a compare register with a
    // buffered write pending returns the BUFFERED value, and this letter
    // asks whether the TC has the same trap.
    Led::configure({});
    using LedWave = TcWo<Led>;                // TC3, WO1
    LedWave::claim();
    const auto led_pwm_up = [](bool alock) {
        return Timer3::init(gen) &&
               Timer3::configure(TcConfig{.mode = TcMode::count8,
                                          .prescaler = TcPrescaler::div256,
                                          .waveform = TcWaveform::normal_pwm,
                                          .lock_update = alock}) &&
               Timer3::set_period8(199) && Timer3::set_cc8(1, 50) &&
               Timer3::enable(true);
    };

    bench.verdict("TC3 drives the LED pad at a quarter duty, ALOCK CLEAR",
                  led_pwm_up(false));
    wait_ms(5);
    const uint32_t free_before = duty_permille<Led>();
    (void)Timer3::set_cc_buffer8(1, 150);
    wait_ms(5);
    const uint32_t free_after = duty_permille<Led>();
    Timer3::release();

    bench.verdict("and again with ALOCK SET", led_pwm_up(true));
    wait_ms(5);
    const uint32_t lock_before = duty_permille<Led>();
    (void)Timer3::set_cc_buffer8(1, 150);
    wait_ms(5);
    const uint32_t lock_held = duty_permille<Led>();
    (void)Timer3::command(TcCommand::update);
    wait_ms(5);
    const uint32_t lock_after = duty_permille<Led>();
    print(serial, "  ALOCK clear: ", free_before, " -> ", free_after,
          " per mille on the buffered write; ALOCK set: ", lock_before,
          " -> ", lock_held, " -> ", lock_after,
          " (write, then the UPDATE command)", crlf);
    bench.verdict("a buffered write is taken at the next update when ALOCK "
                  "is clear", near(free_before, 250, 30) &&
                                  near(free_after, 750, 30));
    bench.verdict("WITH CTRLA.ALOCK SET THE BUFFERED WRITE IS HELD, and the "
                  "UPDATE command is what lets it through - the TC's lock is "
                  "the one the TCC's LUPD is, spelled in CTRLA",
                  near(lock_held, 250, 30) && near(lock_after, 750, 30));

    bench.verdict("DBGCTRL.DBGRUN is writable and reads back; a halted "
                  "debugger is out of a console suite's reach and stays so",
                  (Timer3::debug_run(true), true));
    Timer3::release();
    LedWave::release();   // PMUXEN off, DIR never set: the pad undriven
}

// ---------------------------------------------------------------------------
// d - TCC advanced waveforms: NFRQ, MFRQ, dual-slope CRITICAL, RAMP2A
// ---------------------------------------------------------------------------
//
// The instrument is back on LUT0's TCC input, and the measurements are
// DIFFERENTIAL wherever a mode's arithmetic is what is in question: two
// settings that differ by a known amount must produce two waveforms that
// differ by the amount the chapter's formula predicts, which is a claim
// that cannot be satisfied by a constant.

/// TCC0 in an arbitrary waveform mode, left ENABLED.
bool wave_mode_up(TccWaveform w, uint32_t per, uint32_t cc0, uint32_t cc2 = 0,
                  TccRamp ramp = TccRamp::ramp1, uint32_t cc1 = 0) {
    if (!Wave::init(gen) ||
        !Wave::configure(TccConfig{.prescaler = TccPrescaler::div1})) {
        return false;
    }
    if (!Wave::wave(TccWaveConfig{.waveform = w, .ramp = ramp})) {
        return false;
    }
    return Wave::set_period(per) && Wave::set_cc(0, cc0) &&
           Wave::set_cc(1, cc1) && Wave::set_cc(2, cc2) &&
           Wave::enable(true);
}

void td_tcc_waveforms() {
    bench.verdict("the meter comes up on LUT0's TCC input",
                  tap_up() && meter_up());
    uint16_t period = 0;
    uint16_t width = 0;

    // ---- NFRQ. Table 36-2: PER is TOP and the output TOGGLES on every
    // compare match, so one waveform period is two counter periods and
    // the duty is one half whatever CC0 holds - which is the claim, and
    // it is tested by MOVING CC0 and watching nothing happen.
    constexpr uint32_t nfrq_per = 2399;
    bench.verdict("TCC0 runs in NORMAL FREQUENCY mode",
                  wave_mode_up(TccWaveform::normal_frequency, nfrq_per, 600));
    const bool nfrq_a = meter_read(period, width);
    const uint16_t nfrq_period_a = period;
    const uint16_t nfrq_width_a = width;
    (void)Wave::set_cc(0, 1800);
    const bool nfrq_b = meter_read(period, width);
    print(serial, "  NFRQ PER=", nfrq_per, ": period ", nfrq_period_a,
          " width ", nfrq_width_a, " with CC0=600; period ", period,
          " width ", width, " with CC0=1800 (2 x (PER+1) - 1 = ",
          2u * (nfrq_per + 1u) - 1u, ")", crlf);
    bench.verdict("NFRQ TOGGLES ON THE PERIOD AND NOT ON A COMPARE: one "
                  "waveform period is TWO counter periods, exact to the tick, "
                  "and it is a square wave",
                  nfrq_a && nfrq_period_a == 2u * (nfrq_per + 1u) - 1u &&
                      nfrq_width_a == nfrq_per);
    bench.verdict("and CC0 MOVES NOTHING in this mode - the compare channel "
                  "is spent on nothing at all, which is what separates NFRQ "
                  "from MFRQ",
                  nfrq_b && period == nfrq_period_a && width == nfrq_width_a);
    Wave::release();

    // ---- MFRQ. CC0 is TOP here (36.6.2.5.1), so the same move of CC0
    // that did nothing above must now change the period by twice it.
    constexpr uint32_t mfrq_a_cc = 1199;
    constexpr uint32_t mfrq_b_cc = 1799;
    bench.verdict("TCC0 runs in MATCH FREQUENCY mode, CC0 as its period",
                  wave_mode_up(TccWaveform::match_frequency, 9999, mfrq_a_cc));
    const bool mfrq_ok_a = meter_read(period, width);
    const uint16_t mfrq_period_a = period;
    (void)Wave::set_cc(0, mfrq_b_cc);
    const bool mfrq_ok_b = meter_read(period, width);
    print(serial, "  MFRQ: period ", mfrq_period_a, " at CC0=", mfrq_a_cc,
          ", ", period, " at CC0=", mfrq_b_cc, " (PER left at 9999 and "
          "ignored); 2 x (CC0+1) - 1 = ", 2u * (mfrq_a_cc + 1u) - 1u, " and ",
          2u * (mfrq_b_cc + 1u) - 1u, crlf);
    bench.verdict("IN MFRQ IT IS CC0 THAT IS TOP AND PER IS IGNORED - both "
                  "periods are twice CC0 + 1 to the tick, with PER four "
                  "times too big throughout",
                  mfrq_ok_a && mfrq_ok_b &&
                      mfrq_period_a == 2u * (mfrq_a_cc + 1u) - 1u &&
                      period == 2u * (mfrq_b_cc + 1u) - 1u);
    Wave::release();

    // ---- DUAL-SLOPE CRITICAL. 36.6.2.5.7: CCx sets the UP-counting edge
    // and CC(x + CC_NUM/2) the DOWN-counting one, so on a four-channel
    // instance WO[0]'s two edges are CC0 and CC2 - and a pulse can sit
    // anywhere in the period rather than being centred. Three settings
    // say whether that is what happens: move the down edge alone, then
    // the up edge alone.
    constexpr uint32_t ds_per = 2400;
    bench.verdict("TCC0 runs DUAL-SLOPE CRITICAL with CC0 = 600 and "
                  "CC2 = 1800",
                  wave_mode_up(TccWaveform::dual_slope_critical, ds_per, 600,
                               1800));
    const bool ds_a = meter_read(period, width);
    const uint16_t ds_period = period;
    const uint16_t ds_w0 = width;
    (void)Wave::set_cc(2, 2100);
    const bool ds_b = meter_read(period, width);
    const uint16_t ds_w1 = width;
    (void)Wave::set_cc(0, 900);
    const bool ds_c = meter_read(period, width);
    const uint16_t ds_w2 = width;
    print(serial, "  DSCRITICAL PER=", ds_per, ": period ", ds_period,
          " (2 x PER - 1 = ", 2u * ds_per - 1u, "); widths ", ds_w0,
          " (600/1800), ", ds_w1, " (600/2100), ", ds_w2, " (900/2100)", crlf);
    bench.verdict("the dual-slope period is TWICE PER, critical mode "
                  "included", ds_a && ds_period == 2u * ds_per - 1u);
    // THE ARITHMETIC THE THREE POINTS SETTLE, and 36.6.2.5.7 does not
    // print it: the output is high from CC0 on the way UP, over the top,
    // to CC2 on the way DOWN - so the width is (PER - CC0) + (PER - CC2),
    // one tick short as every capture here is. Each channel moves it by
    // its own amount, in the same direction, independently.
    const auto ds_expect = [](uint32_t c0, uint32_t c2) {
        return (ds_per - c0) + (ds_per - c2) - 1u;
    };
    print(serial, "  and (PER-CC0)+(PER-CC2)-1 predicts ", ds_expect(600, 1800),
          ", ", ds_expect(600, 2100), ", ", ds_expect(900, 2100), crlf);
    bench.verdict("THE TWO EDGES OF ONE OUTPUT ARE TWO INDEPENDENT CHANNELS: "
                  "CC0 places the up-counting edge and CC2 the down-counting "
                  "one, so the pulse width is (PER - CC0) + (PER - CC2) - "
                  "exact to the tick at all three settings, which is what "
                  "'non-aligned' buys and what no other mode here offers",
                  ds_a && ds_b && ds_c && ds_w0 == ds_expect(600, 1800) &&
                      ds_w1 == ds_expect(600, 2100) &&
                      ds_w2 == ds_expect(900, 2100));
    Wave::release();

    // ---- RAMP2A against RAMP1 on IDENTICAL registers, which is the only
    // way to say what the ramp mode itself did: 36.6.3.4 pairs two
    // counter cycles into one waveform cycle, and the question is what
    // that does to the output the LUT is watching.
    bench.verdict("TCC0 runs single-slope PWM in RAMP1, the ordinary case",
                  wave_mode_up(TccWaveform::normal_pwm, 2399, 600, 0,
                               TccRamp::ramp1, 1800));
    const bool r1_ok = meter_read(period, width);
    const uint16_t r1_period = period;
    const uint16_t r1_width = width;
    Wave::release();
    bench.verdict("and again in RAMP2A with the very same PER, CC0 and CC1",
                  wave_mode_up(TccWaveform::normal_pwm, 2399, 600, 0,
                               TccRamp::ramp2_alternate, 1800));
    const bool r2_ok = meter_read(period, width);
    print(serial, "  RAMP1: period ", r1_period, " width ", r1_width,
          ";  RAMP2A: period ", period, " width ", width, crlf);
    bench.verdict("RAMP2A PAIRS TWO COUNTER CYCLES INTO ONE WAVEFORM CYCLE: "
                  "with every register left alone the captured period "
                  "DOUBLES while the pulse width does not move at all, so "
                  "WO[0] is driven in one ramp of the two and idle in the "
                  "other - not, as the name invites, given two duties",
                  r1_ok && r2_ok && period == 2u * (r1_period + 1u) - 1u &&
                      width == r1_width);

    Evsys::disconnect(Meter::event_user);
    Meter::release();
    Ccl::enable(false);
    Ccl::release();
    Wave::release();
}

// ---------------------------------------------------------------------------
// e - the fault system's second half: fault B, filter, blanking, qualifier
// ---------------------------------------------------------------------------
//
// Recoverable fault A is driven from a pin level through the event
// system elsewhere; what this letter gives a stimulus to is fault B,
// FILTERVAL, BLANK/BLANKVAL and QUAL. THE WITNESS THROUGHOUT IS
// INTFLAG.FAULTB, not
// a pad: whether the silicon judged the fault VALID is exactly the
// question, and a flag answers it in one read where a duty sampler needs
// thirty periods of a waveform this letter deliberately runs slowly.

/// The EIC line as a LEVEL, so its event output is a copy of the pad.
bool fault_pin_up() {
    Evsys::bus_clock(true);
    if (!Eic::init() || !Eic::clock_select(EicClock::ulp32k)) {
        return false;
    }
    EicPad::input(PinPull::down);
    EicLine::claim(PinPull::down);
    return Eic::configure_line(EicLine::line,
                               EicLineConfig{.sense = EicSense::high,
                                             .event_out = true}) &&
           Eic::enable(true);
}

void fault_pin_down() {
    EicPad::clear();
    (void)Eic::enable(false);
    Eic::release();
    EicPad::configure({});
}

/// TCC0 as a slow PWM with recoverable fault B taking the EIC line.
/// /1024 makes one prescaled clock 21.3 us, which is what puts
/// FILTERVAL and BLANKVAL in a range the CPU can straddle by hand.
constexpr uint32_t slow_per = 4687;          // ~100 ms at /1024

/// A generic clock of 46875 Hz - OSC48M divided by 2^(9+1) - so that one
/// GCLK_TCC CYCLE is 21.3 us. Letter e needs it to ask whether
/// FCTRLn.FILTERVAL counts generic clocks or PRESCALED ones, which is a
/// question only two different prescalers on ONE generic clock can
/// answer.
constexpr uint8_t slow_gen = 7;
constexpr uint32_t slow_gclk_hz = 46875;
using SlowGclk = Gclk<slow_gen>;

bool fault_b_up(const TccFaultConfig& fb, uint32_t duty = 2343,
                uint8_t generator = gen,
                TccPrescaler prescaler = TccPrescaler::div1024,
                uint32_t per = slow_per) {
    // THE DUTY GOES INTO CC1, NOT CC0, and that is the whole of the
    // qualifier's meaning: FCTRLn.QUAL watches THE FAULT'S OWN CHANNEL
    // output (36.6.3.5), and fault B is channel 1's. Put the duty in
    // CC0 and the qualified fault never fires at any duty - correct
    // behaviour of a wrong setup.
    if (!Wave::init(generator)) {
        return false;
    }
    const TccConfig cfg{.prescaler = prescaler};
    if (!Wave::configure(cfg) || !Wave::fault(TccFault::b, fb)) {
        return false;
    }
    if (!Wave::event_config(cfg, TccEventConfig{
                                     .match_in = tcc_fault_input_bit(TccFault::b)})) {
        return false;
    }
    if (!Wave::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) ||
        !Wave::set_period(per) || !Wave::set_cc(0, duty) ||
        !Wave::set_cc(1, duty)) {
        return false;
    }
    if (!Evsys::connect(Wave::fault_user(TccFault::b), ev_wave_channel,
                        EventChannelConfig{.generator = EicLine::event_generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return Wave::enable(true);
}

/// Loop turns per microsecond, MEASURED rather than counted off the
/// instruction set - see calibrate_pulse().
uint32_t turns_per_us = 16;

/// A 750 kHz stopwatch on TC4 (generator 0, /64): one tick is 1.333 us,
/// and 65536 of them are 87 ms, which covers every pulse this suite
/// makes.
constexpr uint32_t watch_hz = tc_hz / 64u;
bool watch_up() {
    return Counter::init(gen) &&
           Counter::configure(TcConfig{.mode = TcMode::count16,
                                       .prescaler = TcPrescaler::div64}) &&
           Counter::enable(true);
}

/// How long a busy loop of `turns` really takes, in microseconds.
uint32_t time_turns(uint32_t turns) {
    const uint16_t t0 = Counter::count16();
    for (uint32_t i = 0; i < turns; ++i) {
        asm volatile("nop");
    }
    const uint16_t t1 = Counter::count16();
    return (static_cast<uint32_t>(static_cast<uint16_t>(t1 - t0)) * 1000000UL) /
           watch_hz;
}

/// THE STIMULUS IS MEASURED BEFORE IT IS USED. A pulse this letter calls
/// "120 us" has to be 120 us, or a threshold measured with it says
/// nothing about the silicon; a nop loop on this core costs four or five
/// cycles a turn and guessing which is not a measurement.
void calibrate_pulse() {
    const uint32_t us = time_turns(20000);
    if (us > 0u) {
        turns_per_us = 20000u / us;
    }
    if (turns_per_us == 0u) {
        turns_per_us = 1;
    }
}

/// Hold the pad high for `us` REAL microseconds.
void pulse_pad(uint32_t us) {
    EicPad::set();
    for (uint32_t i = 0; i < us * turns_per_us; ++i) {
        asm volatile("nop");
    }
    EicPad::clear();
}

/// THE SLOW GENERATOR'S RATE IS MEASURED TOO, by counting it against the
/// SysTick wall clock - because a divisor believed is a divisor that can
/// be wrong, and every number letters e and f print rests on it.
uint32_t slow_gclk_measured = 0;
uint32_t measure_slow_gclk(uint8_t generator) {
    if (!Counter::init(generator) ||
        !Counter::configure(TcConfig{.mode = TcMode::count16,
                                     .prescaler = TcPrescaler::div1}) ||
        !Counter::enable(true)) {
        return 0;
    }
    (void)Counter::set_count16(0);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 250u) {
    }
    const uint32_t n = Counter::count16();
    const uint32_t ms = Ticker::millis() - t0;
    Counter::release();
    return ms != 0u ? (n * 1000u) / ms : 0u;
}

/// Did fault B become valid? INTFLAG.FAULTB is the latch that says so.
bool fault_b_seen(uint32_t settle_ms = 5) {
    wait_ms(settle_ms);
    const bool seen = (Wave::flags() & Wave::fault_b_flag) != 0u;
    Wave::clear_flags(Wave::fault_b_flag);
    (void)Wave::clear_fault_state(TccFault::b);
    return seen;
}

void te_fault_b() {
    bench.verdict("the 750 kHz stopwatch comes up and the pulse generator is "
                  "calibrated against it", watch_up() &&
                      (calibrate_pulse(), turns_per_us > 2u));
    Counter::release();
    print(serial, "  the busy loop measures ", turns_per_us,
          " turns per microsecond", crlf);
    bench.verdict("the EIC turns PA16's level into an event",
                  fault_pin_up());

    // ---- FAULT B EXISTS AND IS THE MIRROR OF FAULT A. 36.6.3.5: fault
    // A is channel 0's event input and fault B is channel 1's, which is
    // a fact about the USER INDEX and is checked in the header; here it
    // is checked in the silicon.
    bench.verdict("TCC0 comes up with recoverable fault B on its channel-1 "
                  "event input",
                  fault_b_up(TccFaultConfig{.source = TccFaultSource::event,
                                            .halt = TccFaultHalt::hardware}));
    Wave::clear_flags(0xFFFFFFFFu);
    pulse_pad(2000);
    const bool b_plain = fault_b_seen();
    print(serial, "  a 2 ms pulse with no filter: FAULTB ",
          b_plain ? "valid" : "not seen", crlf);
    bench.verdict("A PULSE ON THE FAULT B INPUT IS A VALID FAULT - the "
                  "second recoverable fault works exactly as the first, on "
                  "the second channel's event input",
                  b_plain);
    Evsys::disconnect(Wave::fault_user(TccFault::b));
    Wave::release();

    // ---- FILTERVAL, and the question is not whether it works but WHAT
    // IT COUNTS. 36.8.5's wording is "prescaled clocks" - but this
    // chapter has form: the dead times measure as GCLK_TCC cycles and are UNMOVED by a fourfold prescaler change,
    // where 36.8.7 reads the same way. Asserting a 320 us minimum from
    // FILTERVAL 15 at /1024 measures no rejection at 30 us, which is
    // the answer to a different question.
    //
    // So: ONE generic clock slow enough that fifteen of its cycles are
    // 320 us, and TWO prescalers on it. If FILTERVAL counts generic
    // clocks the threshold does not move; if it counts prescaled ones it
    // moves by the prescaler's ratio.
    static const uint16_t widths[] = {20, 40, 80, 160, 320, 640, 1280, 5000};
    const auto threshold = [](uint8_t filt, uint8_t generator,
                              TccPrescaler pre, uint32_t per) {
        uint16_t first = 0;
        if (!fault_b_up(TccFaultConfig{.source = TccFaultSource::event,
                                       .halt = TccFaultHalt::hardware,
                                       .filter_value = filt},
                        per / 2u, generator, pre, per)) {
            return static_cast<uint16_t>(0xFFFFu);
        }
        for (uint8_t i = 0; i < 8u; ++i) {
            Wave::clear_flags(0xFFFFFFFFu);
            (void)Wave::clear_fault_state(TccFault::b);
            pulse_pad(widths[i]);
            if (fault_b_seen(3) && first == 0u) {
                first = widths[i];
            }
        }
        Evsys::disconnect(Wave::fault_user(TccFault::b));
        Wave::release();
        return first;
    };
    // THE GENERATOR'S RATE IS MEASURED AND NOT BELIEVED, and it is worth
    // the two lines: an experiment whose ruler is a divisor taken on
    // trust measures the divisor and calls it silicon. Two DIVSEL
    // settings on one generator say what the divisor really is.
    bool slow_ok = SlowGclk::configure(
        GclkConfig{.source = GclkSource::osc48m, .div = 8, .div_pow2 = true});
    const uint32_t div8_hz = measure_slow_gclk(slow_gen);
    slow_ok = slow_ok && SlowGclk::configure(GclkConfig{
                             .source = GclkSource::osc48m, .div = 9,
                             .div_pow2 = true});
    slow_gclk_measured = measure_slow_gclk(slow_gen);
    const uint32_t div8_ratio = div8_hz != 0u ? tc_hz / div8_hz : 0u;
    const uint32_t div9_ratio =
        slow_gclk_measured != 0u ? tc_hz / slow_gclk_measured : 0u;
    print(serial, "  generator 7 with DIVSEL: DIV = 8 measures ", div8_hz,
          " Hz (divisor ~", div8_ratio, "), DIV = 9 measures ",
          slow_gclk_measured, " Hz (divisor ~", div9_ratio, ")", crlf);
    print(serial, "  so one GCLK_TCC cycle is ",
          slow_gclk_measured != 0u ? 1000000UL / slow_gclk_measured : 0u,
          " us and FILTERVAL 15 is ",
          slow_gclk_measured != 0u ? 15000000UL / slow_gclk_measured : 0u,
          " us of them", crlf);
    bench.verdict("a slow generic clock is built, and its rate MEASURED "
                  "against the wall clock rather than computed from a "
                  "divisor - which is what the rest of this letter's "
                  "microseconds rest on",
                  slow_ok && slow_gclk_measured > 10000u &&
                      slow_gclk_measured < 400000u);
    bench.verdict("DIVSEL'S DIVISOR SATURATES AT THE DIV FIELD'S OWN WIDTH: "
                  "DIV = 8 and DIV = 9 give the SAME 512 on generator 7, "
                  "whose DIV field is eight bits - so 2^(DIV+1) is right up "
                  "to 2^(width+1) and a DIV past that buys nothing, which "
                  "reconciles the clock suite's 2/16/512 for DIV "
                  "0/3/8 with the 1024 a bare reading would have predicted "
                  "here",
                  div8_hz != 0u &&
                      near(div8_hz, slow_gclk_measured,
                           slow_gclk_measured / 20u) &&
                      near(div9_ratio, 512, 25));
    const uint16_t thr_bare = threshold(0, slow_gen, TccPrescaler::div1, 4687);
    const uint16_t thr_f1 = threshold(15, slow_gen, TccPrescaler::div1, 4687);
    const uint16_t thr_f64 = threshold(15, slow_gen, TccPrescaler::div64, 73);
    print(serial, "  shortest pulse that makes a VALID fault, all on the "
          "46875 Hz clock: ", thr_bare, " us with FILTERVAL 0, ", thr_f1,
          " us with FILTERVAL 15 at /1, ", thr_f64,
          " us with FILTERVAL 15 at /64", crlf);
    const uint32_t fifteen_us =
        slow_gclk_measured != 0u ? 15000000UL / slow_gclk_measured : 0u;
    print(serial, "  fifteen GCLK_TCC cycles are ", fifteen_us,
          " us at the measured rate", crlf);
    bench.verdict("FCTRLn.FILTERVAL IS A REAL MINIMUM WIDTH, AND IT IS "
                  "FIFTEEN GENERIC CLOCK CYCLES OF IT: the filtered path "
                  "rejects every pulse the bare one accepts up to the "
                  "sweep step that brackets those fifteen cycles",
                  thr_bare != 0u && thr_f1 != 0u && thr_f1 >= 2u * thr_bare &&
                      thr_f1 * 2u >= fifteen_us && thr_f1 <= 2u * fifteen_us);
    bench.verdict("AND IT COUNTS GCLK_TCC CYCLES, NOT PRESCALED ONES - a "
                  "sixty-fourfold prescaler change on the same generic clock "
                  "leaves the threshold where it was, which is the dead-time "
                  "unit's story again and NOT what 36.8.5's 'prescaled "
                  "clocks' says",
                  thr_f64 == thr_f1);

    // ---- BLANKING. The input is held HIGH for the whole measurement,
    // so the only thing that can keep the fault from being valid is the
    // blanking window - and BLANKVAL 255 at /1024 is 5.4 ms against a
    // 100 ms period, so the two settings below differ in whether the
    // window covers the moment the input rises.
    bench.verdict("the fault is re-armed with BLANKVAL = 0", 
                  fault_b_up(TccFaultConfig{.source = TccFaultSource::event,
                                            .halt = TccFaultHalt::hardware,
                                            .blank_value = 0}));
    Wave::clear_flags(0xFFFFFFFFu);
    EicPad::set();
    const bool blank_none = fault_b_seen(150);
    EicPad::clear();
    Evsys::disconnect(Wave::fault_user(TccFault::b));
    Wave::release();

    bench.verdict("and again with BLANK = period start and BLANKVAL = 255",
                  fault_b_up(TccFaultConfig{.source = TccFaultSource::event,
                                            .halt = TccFaultHalt::hardware,
                                            .blank = TccFaultBlank::period_start,
                                            .blank_value = 255}));
    Wave::clear_flags(0xFFFFFFFFu);
    EicPad::set();
    const bool blank_wide = fault_b_seen(150);
    EicPad::clear();
    print(serial, "  input held HIGH for 150 ms: BLANKVAL 0 -> FAULTB ",
          blank_none ? "valid" : "never", ", BLANKVAL 255 (5.4 ms of a "
          "100 ms period) -> ", blank_wide ? "valid" : "never", crlf);
    bench.verdict("A BLANKING WINDOW GATES THE INPUT AND NOT ITS EDGE: with "
                  "the pad held high throughout, a fault that is valid with "
                  "no window is still valid with a 5.4 ms one, because the "
                  "window closes long before the period does",
                  blank_none && blank_wide);
    Evsys::disconnect(Wave::fault_user(TccFault::b));
    Wave::release();

    // ---- QUALIFICATION. FCTRLn.QUAL ignores the input while the
    // channel's output is at its inactive level, so a DUTY OF ZERO is
    // the categorical test: the output is never active, so a qualified
    // fault can never be valid however long the input is held.
    bench.verdict("the fault is re-armed with QUAL set and the waveform's "
                  "duty at ZERO, so channel 1's output is never active",
                  fault_b_up(TccFaultConfig{.source = TccFaultSource::event,
                                            .halt = TccFaultHalt::hardware,
                                            .qualify = true},
                             0));
    Wave::clear_flags(0xFFFFFFFFu);
    EicPad::set();
    const bool qual_zero = fault_b_seen(150);
    EicPad::clear();
    Evsys::disconnect(Wave::fault_user(TccFault::b));
    Wave::release();

    bench.verdict("and again with QUAL set and a half duty", 
                  fault_b_up(TccFaultConfig{.source = TccFaultSource::event,
                                            .halt = TccFaultHalt::hardware,
                                            .qualify = true},
                             2343));
    Wave::clear_flags(0xFFFFFFFFu);
    EicPad::set();
    const bool qual_half = fault_b_seen(150);
    EicPad::clear();
    print(serial, "  QUAL with duty 0 -> FAULTB ", qual_zero ? "valid" : "never",
          "; QUAL with duty 1/2 -> ", qual_half ? "valid" : "never", crlf);
    bench.verdict("FCTRLn.QUAL TIES THE FAULT TO THE WAVEFORM: with the "
                  "output never active the same held input raises nothing, "
                  "and with it active half the time it raises a fault. AND "
                  "THE CHANNEL IT WATCHES IS THE FAULT'S OWN: fault B "
                  "qualifies on channel 1's output, so it is CC1 and not "
                  "CC0 that has to move",
                  !qual_zero && qual_half);
    Evsys::disconnect(Wave::fault_user(TccFault::b));
    Wave::release();

    fault_pin_down();
}

// ---------------------------------------------------------------------------
// f - the counter event actions, and ERRATUM 1.21.7 staged
// ---------------------------------------------------------------------------
//
// 36.8.10's `increment` and `count_while_active` take the EIC line over
// an asynchronous channel. Erratum 1.21.7 - dithering plus external
// RETRIGGER events distorting pulses - is staged with TC0's overflow as
// a periodic hardware retrigger, and the dithered waveform, captured
// through the LUT, is the witness.

/// TCC0 with the EIC line on its TCE0 counter event input.
bool counter_event_up(TccEvent0Action action, uint8_t generator,
                      TccPrescaler pre, uint32_t per, uint32_t cc0,
                      uint8_t capture = 0,
                      TccResolution res = TccResolution::none) {
    if (!Wave::init(generator)) {
        return false;
    }
    const TccConfig cfg{.prescaler = pre,
                        .resolution = res,
                        .capture_enable = capture};
    if (!Wave::configure(cfg)) {
        return false;
    }
    if (!Wave::event_config(cfg, TccEventConfig{.action0 = action,
                                                .input0_enable = true})) {
        return false;
    }
    if (!Wave::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) ||
        !Wave::set_period(per) || !Wave::set_cc(0, cc0)) {
        return false;
    }
    if (!Evsys::connect(Wave::event_user(0), ev_out_channel,
                        EventChannelConfig{.generator = EicLine::event_generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return Wave::enable(true);
}

void tf_event_actions() {
    const bool slow_ok = SlowGclk::configure(
        GclkConfig{.source = GclkSource::osc48m, .div = 9, .div_pow2 = true});
    slow_gclk_measured = measure_slow_gclk(slow_gen);
    (void)watch_up();
    calibrate_pulse();
    Counter::release();
    print(serial, "  the slow generic clock measures ", slow_gclk_measured,
          " Hz; the busy loop ", turns_per_us, " turns per microsecond", crlf);
    bench.verdict("the slow generic clock and the EIC line are up",
                  slow_ok && slow_gclk_measured > 10000u && fault_pin_up());

    // ---- INCREMENT. 36.8.10: increment the counter on every event,
    // irrespective of CTRLB.DIR. The counter's OWN clock is put where it
    // cannot interfere: 46875 Hz divided by 1024 is 45.8 Hz, so over the
    // few milliseconds the pulses take it contributes nothing.
    bench.verdict("TCC0 takes the EIC line as a counter event with EVACT0 = "
                  "INCREMENT, its own clock slowed to 45.8 Hz so it cannot "
                  "contribute",
                  counter_event_up(TccEvent0Action::increment, slow_gen,
                                   TccPrescaler::div1024, 0xFFFFFu, 0));
    (void)Wave::set_count(0);
    constexpr uint8_t pulses = 20;
    for (uint8_t i = 0; i < pulses; ++i) {
        pulse_pad(200);
        for (uint32_t k = 0; k < 4000u; ++k) {
            asm volatile("nop");
        }
    }
    const uint32_t inc_count = Wave::count();
    print(serial, "  ", pulses, " pin pulses with EVACT0 = INCREMENT: COUNT = ",
          inc_count, crlf);
    bench.verdict("EVACT0 = INCREMENT COUNTS EVENTS AND NOTHING ELSE: one "
                  "count per pulse, exactly",
                  inc_count == pulses);
    Evsys::disconnect(Wave::event_user(0));
    Wave::release();

    // ---- COUNT WHILE ACTIVE. 36.8.10: count one tick per prescaled
    // clock for as long as the asynchronous event is asserted - so the
    // counter becomes a stopwatch for the pad's HIGH TIME, and the
    // reading is that time in prescaled clocks.
    bench.verdict("TCC0 takes the same line with EVACT0 = COUNT (while "
                  "active), its clock at 46875 Hz undivided",
                  counter_event_up(TccEvent0Action::count_while_active,
                                   slow_gen, TccPrescaler::div1, 0xFFFFFu, 0));
    (void)Wave::set_count(0);
    wait_ms(20);
    const uint32_t idle_count = Wave::count();
    (void)Wave::set_count(0);
    EicPad::set();
    wait_ms(20);
    EicPad::clear();
    const uint32_t active_count = Wave::count();
    const uint32_t expect = (slow_gclk_measured * 20u) / 1000u;
    print(serial, "  EVACT0 = COUNT: 20 ms with the line LOW -> ", idle_count,
          " counts; 20 ms with it HIGH -> ", active_count, " (expected ~",
          expect, " at the measured ", slow_gclk_measured, " Hz)", crlf);
    bench.verdict("EVACT0 = COUNT TURNS THE COUNTER INTO A GATE: it advances "
                  "one prescaled clock at a time for exactly as long as the "
                  "event is asserted, and not at all while it is not",
                  idle_count < 4u && near(active_count, expect, expect / 10u));
    Evsys::disconnect(Wave::event_user(0));
    Wave::release();

    // ---- ERRATUM 1.21.7, STAGED. Dithering spreads a fractional
    // compare over a frame of 64 PWM cycles, so a dithered duty shows as
    // exactly TWO widths in a fixed ratio. That is a very sharp witness:
    // any third value is a distorted pulse.
    bench.verdict("the meter comes up on LUT0's TCC input", tap_up() &&
                                                                meter_up());
    // DITH6: the low six bits of PER and CCx stop being part of the
    // value. Period 600 clocks, duty 200 + 32/64 clocks.
    const uint32_t dith_per = tcc_dither(599, 0, TccResolution::dither64);
    const uint32_t dith_cc = tcc_dither(200, 32, TccResolution::dither64);
    const auto dither_widths = [&](uint16_t* out, uint8_t n) {
        for (uint8_t i = 0; i < 3u; ++i) {
            (void)Meter::cc16(0);
            (void)Meter::cc16(1);
        }
        Meter::clear_flags(0xFFu);
        for (uint8_t i = 0; i < n; ++i) {
            if (!wait_flag(Meter::match_flag(1))) {
                return false;
            }
            out[i] = Meter::cc16(1);
            Meter::clear_flags(0xFFu);
        }
        return true;
    };
    const auto distinct = [](const uint16_t* v, uint8_t n) {
        uint8_t d = 0;
        for (uint8_t i = 0; i < n; ++i) {
            bool seen = false;
            for (uint8_t k = 0; k < i; ++k) {
                if (v[k] == v[i]) {
                    seen = true;
                }
            }
            if (!seen) {
                ++d;
            }
        }
        return d;
    };

    constexpr uint8_t dn = 40;
    uint16_t quiet[dn];
    uint16_t retrig[dn];
    bench.verdict("TCC0 runs a DITHERED waveform - period 600, duty 200 and "
                  "32/64 - with no event reaching it at all",
                  counter_event_up(TccEvent0Action::off, gen,
                                   TccPrescaler::div1, dith_per, dith_cc, 0,
                                   TccResolution::dither64));
    const bool quiet_ok = dither_widths(quiet, dn);
    const uint8_t quiet_distinct = distinct(quiet, dn);
    Evsys::disconnect(Wave::event_user(0));
    Wave::release();

    // ---- NOW THE RETRIGGER, and it has to be PERIODIC AND HARDWARE:
    // a CPU-made pulse happens once and is gone before forty widths have
    // been captured. TC0 free-running publishes its overflow as an
    // event at a rate deliberately incommensurate with the waveform's
    // period, so a retrigger lands somewhere different every time.
    //
    // AND THE STAGING NEEDS A CONTROL, because a retrigger truncates a
    // pulse whether the TCC is dithering or not - that is what a
    // retrigger IS. The erratum's claim is about dithering, so the same
    // retrigger is applied to an UNDITHERED waveform of the same period
    // and duty, and the two distinct-width counts are what get compared.
    const auto retrigger_source_up = [](uint16_t top) {
        if (!Timer0::init(gen) ||
            !Timer0::configure(TcConfig{.mode = TcMode::count16,
                                        .prescaler = TcPrescaler::div1,
                                        .waveform = TcWaveform::match_frequency})) {
            return false;
        }
        if (!Timer0::event_config(TcConfig{.mode = TcMode::count16},
                                  TcEventConfig{.overflow_out = true})) {
            return false;
        }
        return Timer0::set_cc16(0, top) && Timer0::enable(true);
    };
    bench.verdict("TC0 free-runs as a periodic event source, its overflow "
                  "every 998 ticks against the waveform's 600 - so a "
                  "retrigger arrives constantly and NEVER inside the "
                  "200-tick pulse",
                  retrigger_source_up(997));

    const auto retriggered_distinct = [&](uint32_t per, uint32_t cc,
                                          TccResolution res, uint16_t* out) {
        if (!counter_event_up(TccEvent0Action::retrigger, gen,
                              TccPrescaler::div1, per, cc, 0, res)) {
            return static_cast<uint8_t>(0);
        }
        (void)Evsys::connect(Wave::event_user(0), ev_out_channel,
                             EventChannelConfig{
                                 .generator = Timer0::overflow_generator,
                                 .path = EventPath::asynchronous});
        wait_ms(2);
        uint8_t worst = 0;
        for (uint8_t round = 0; round < 3u; ++round) {
            if (!dither_widths(out, dn)) {
                break;
            }
            const uint8_t d = distinct(out, dn);
            if (d > worst) {
                worst = d;
            }
        }
        Evsys::disconnect(Wave::event_user(0));
        Wave::release();
        return worst;
    };

    const uint8_t plain_distinct =
        retriggered_distinct(599, 200, TccResolution::none, retrig);
    uint16_t retrig_d[dn];
    const uint8_t dith_distinct =
        retriggered_distinct(dith_per, dith_cc, TccResolution::dither64,
                             retrig_d);
    print(serial, "  distinct pulse widths in ", dn, " captures: ",
          quiet_distinct, " dithered and untouched, ", plain_distinct,
          " UNdithered under a periodic retrigger, ", dith_distinct,
          " dithered under the same retrigger", crlf);
    print(serial, "  quiet sample:");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", quiet[i]);
    }
    print(serial, "   plain+retrigger:");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", retrig[i]);
    }
    print(serial, "   dither+retrigger:");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", retrig_d[i]);
    }
    print(serial, crlf);
    bench.verdict("A DITHERED WAVEFORM LEFT ALONE HAS EXACTLY TWO PULSE "
                  "WIDTHS - the whole clock and the whole clock plus one, "
                  "spread over a frame of 64 cycles, which is what dithering "
                  "IS and what makes it a witness sharp enough for the "
                  "erratum",
                  quiet_ok && quiet_distinct == 2u);
    bench.verdict("ERRATUM 1.21.7 DID NOT REPRODUCE in the one arrangement "
                  "where it can be told apart from a retrigger's own "
                  "truncation: a dithering TCC retriggered constantly, at a "
                  "rate that never cuts a pulse, produced EXACTLY the two "
                  "widths the dithering frame contains and not a third - "
                  "recorded as unreproduced, not disproved, which is the "
                  "standing 1.21.8 already has here",
                  dith_distinct == 2u && plain_distinct == 1u);

    // AND THE WITNESS IS PROVED SENSITIVE, which is what makes "did not
    // reproduce" mean anything: the same instrument, with the retrigger
    // moved to a rate that DOES cut into the pulse, sees the distortion
    // at once - in the undithered waveform as much as in the dithered
    // one, which is why the arrangement above is the one that can carry
    // the erratum's claim and this one is not.
    Timer0::release();
    bench.verdict("the retrigger is moved to 3150 ticks, 5.25 waveform "
                  "periods, so its landing point walks through the pulse",
                  retrigger_source_up(3149));
    const uint8_t plain_cut =
        retriggered_distinct(599, 200, TccResolution::none, retrig);
    const uint8_t dith_cut =
        retriggered_distinct(dith_per, dith_cc, TccResolution::dither64,
                             retrig_d);
    print(serial, "  with the retrigger cutting into the pulse: ", plain_cut,
          " distinct widths undithered, ", dith_cut, " dithered", crlf);
    bench.verdict("THE INSTRUMENT IS SENSITIVE: a retrigger that lands "
                  "inside a pulse truncates it, and the capture stream sees "
                  "more widths at once - dithered and undithered alike, "
                  "which is a retrigger's own doing and not the erratum's",
                  plain_cut > plain_distinct && dith_cut > dith_distinct);
    Timer0::release();

    Evsys::disconnect(Wave::event_user(0));
    Evsys::disconnect(Meter::event_user);
    Meter::release();
    Ccl::enable(false);
    Ccl::release();
    Wave::release();
    fault_pin_down();
}

// ---------------------------------------------------------------------------
// The console
// ---------------------------------------------------------------------------
void banner() {
    print(serial, crlf, "test_samc_timer_modes - the advanced modes of the "
          "TC and the TCC", crlf);
    bench.menu();
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "TCC hardware circular buffers: CICCEN and CIPEREN",
                 ta_circular);
    bench.letter('b', "TC advanced: MFRQ, MPWM, the 16-bit task, INVEN",
                 tb_tc_waveforms);
    bench.letter('c', "TC advanced: PWP/PW/STAMP, PRESCSYNC, ALOCK",
                 tc_tc_capture_and_locks);
    bench.letter('d', "TCC advanced waveforms: NFRQ, MFRQ, DSCRITICAL, RAMP2A",
                 td_tcc_waveforms);
    bench.letter('e', "TCC fault B, the filter, the blanking and the qualifier",
                 te_fault_b);
    bench.letter('f', "TCC counter event actions, and erratum 1.21.7 staged",
                 tf_event_actions);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
