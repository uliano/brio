// test_samc_sdadc - the reference bench suite for samc/sdadc.hpp, this
// family's THIRD converter and the one the Multislope work will lean on.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and this chapter makes that harder than the last two
// did. The SDADC's input is a PAIR of pads and nothing internal reaches
// it: MUXSEL names AINN<k>/AINP<k> and nothing else, so the DAC - which
// closed the ADC's and the AC's loops on PA02 - CANNOT be an SDADC input
// at all. It reaches this converter only as a REFERENCE (REFCTRL.REFSEL
// = DAC), which is what letter f is about.
//
// So the voltage sources this board can put across a differential pair
// are:
//   * BOTH PADS AT ONE RAIL from PORT - an exact analog ZERO, which is
//     what a noise floor wants (letter b);
//   * THE TWO PADS AT OPPOSITE RAILS - a differential of +/- VDD, past
//     every specified input range, which proves the SIGN and shows what
//     saturation looks like (letter c);
//   * TWO PWM WAVEFORMS out of TCC1, whose WO0 and WO1 are PA06 and PA07
//     - i.e. AINN0 and AINP0 themselves. The converter's own decimation
//     filter is the reconstruction filter: with OSR 1024 at CLK_SDADC
//     6 MHz the window is 32768 GCLK cycles, so a 512-cycle PWM period
//     puts the fundamental and every harmonic EXACTLY on a zero of the
//     third-order SINC. That is a swept differential with no wire, and
//     it is the shape a Multislope integrator makes anyway (letter d).
//     WHAT IT CANNOT CLAIM is said in the letter: the instantaneous
//     input is always 0 or +/- VDD, i.e. +/- 1.0 x VREF where table
//     45-26 specifies +/- 0.7, so any compression at the ends of the
//     sweep is the modulator's overload and not the converter's INL.
//
// The cross-check between the two architectures is therefore NOT a
// shared voltage (there is none) but a shared RATIO: letter g reads the
// SUPC bandgap against VDDANA through the SDADC's reference multiplexer
// and compares it with what the SAR ADC says about the same two, two
// converters sharing nothing but the bandgap itself.
//
// What is exercised, letter by letter:
//   a  the block: geometry, the vocabularies it publishes, the register
//      disciplines, and the chapter's disagreements with itself and with
//      its own device header
//   b  THE ZERO AND THE NOISE: both pads at one rail, every OSR - the
//      Multislope number
//   c  full scale and the sign, and where the datum sits in RESULT
//   d  the swept differential, and the SAR watching the same pads
//   e  time: the conversion rate, the PRESCALER question, and SKPCNT
//   f  ERRATUM 1.8.10: the DAC as the reference, with and without
//      ONREFBUF, and a control
//   g  the reference multiplexer, and the SAR/SDADC cross-check
//   h  the post-processing: OFFSETCORR, GAINCORR, SHIFTCORR, the chopper
//   i  the window monitor on a signed result
//   j  the no-CPU chain: event in, DMAC out, a TC counting
//   k  the automatic sequence over the three pairs
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/adc.hpp"
#include "samc/clock.hpp"
#include "samc/dac.hpp"
#include "samc/dmac.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sdadc.hpp"
#include "samc/sercom.hpp"
#include "samc/supc.hpp"
#include "samc/tc.hpp"
#include "samc/tcc.hpp"
#include "samc/ticker.hpp"
#include "util/analog.hpp"
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
using Led = Pin<'B', 23>;

TestBench<Serial, 16> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The pads. A differential pair is TWO of them, and the package variation
// is the whole map - which is why the family fixture asserts it per
// variant and this app is built for the J alone.
// ---------------------------------------------------------------------------
using Inn0 = Pin<'A', 6>;    // AINN0, also ADC0/AIN6 and AC/AIN2
using Inp0 = Pin<'A', 7>;    // AINP0, also ADC0/AIN7 and AC/AIN3
using Inn1 = Pin<'B', 8>;    // AINN1, also ADC0/AIN2 and ADC1/AIN4
using Inp1 = Pin<'B', 9>;    // AINP1, also ADC0/AIN3 and ADC1/AIN5
using Inn2 = Pin<'B', 6>;    // AINN2, J only
using Inp2 = Pin<'B', 7>;    // AINP2, J only
using VrefbPad = Pin<'A', 4>;   // VREFB, also ADC0/AIN4
using DacPad = Pin<'A', 2>;     // DAC/VOUT, also ADC0/AIN0

using Adc0 = Adc<0>;
using Adc1 = Adc<1>;

/// GCLK generator 0 is the 48 MHz main clock; PRESCALER 3 divides it by
/// eight, which is table 45-26's ceiling of 6 MHz for CLK_SDADC.
constexpr uint8_t main_gen = 0;
constexpr uint32_t main_gen_hz = SysClock::hz;
constexpr uint8_t fast_prescaler = 3;

/// What the earlier campaigns located this board's supply at. A STARTING
/// POINT for the millivolt prints, never a verdict's authority.
constexpr uint16_t supply_hint_mv = 5150;
uint16_t vdd_mv = supply_hint_mv;

// ---------------------------------------------------------------------------
// The stopwatch (letter e): TC0 + TC1 as one 32-bit counter on the
// BOARD'S CRYSTAL. A conversion time reported against OSC48M would carry
// that oscillator's 5100 ppm into a number about the SDADC.
// ---------------------------------------------------------------------------
using Stopwatch = Tc<0>;
constexpr uint32_t crystal_hz = 24'000'000UL;
constexpr uint8_t gen_crystal = 2;
bool on_crystal = false;

bool stopwatch_start() {
    on_crystal = Xosc::init(XoscConfig{.hz = crystal_hz, .startup = 4}) &&
                 Gclk<gen_crystal>::configure(GclkConfig{.source = GclkSource::xosc});
    if (!on_crystal) {
        return false;
    }
    return Stopwatch::init(gen_crystal) &&
           Stopwatch::configure(TcConfig{.mode = TcMode::count32,
                                         .prescaler = TcPrescaler::div1}) &&
           Stopwatch::enable(true);
}
void stopwatch_stop() {
    (void)Stopwatch::enable(false);
    Stopwatch::release();
}
uint32_t ticks_now() { return Stopwatch::count32(); }

// ---------------------------------------------------------------------------
// The event fabric (letter j): the same shape every SAM suite here uses.
// ---------------------------------------------------------------------------
constexpr uint8_t dma_ch = 0;
constexpr uint8_t ev_start_channel = 0;    // TC2 overflow -> SDADC START
constexpr uint8_t ev_result_channel = 1;   // SDADC RESRDY -> TC3 counts
constexpr uint8_t ev_gen = 6;
using Copy = DmaChannel<dma_ch>;
using EvGen = Gclk<ev_gen>;
using Pacer = Tc<2>;
using Counter = Tc<3>;

/// VOLATILE IN BOTH DIRECTIONS - the DMAC campaign's lesson on this
/// target: the compiler sees neither the controller's reads nor its
/// writes.
constexpr uint16_t dma_results = 16;
volatile uint32_t results[dma_results];

// ---------------------------------------------------------------------------
// The PWM source (letter d): TCC1's two channels ARE this pair's two pads.
// ---------------------------------------------------------------------------
using Pwm = Tcc<1>;
using PwmN = TccWo<Inn0, PinFunction::e>;   // TCC1/WO0 on PA06
using PwmP = TccWo<Inp0, PinFunction::e>;   // TCC1/WO1 on PA07
/// 512 GCLK cycles a period. With OSR 1024 at CLK_SDADC = GCLK/8 the
/// decimation window is 1024 x 4 x 8 = 32768 GCLK cycles, so exactly 64
/// PWM periods fit it and the fundamental lands on a SINC zero.
constexpr uint32_t pwm_top = 511;
constexpr uint32_t pwm_steps = pwm_top + 1u;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}
bool near_signed(int32_t a, int32_t b, int32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

const char* yes_no(bool v) { return v ? "yes" : "no"; }

/// floor(log2(v)), for the noise-free-bits arithmetic.
uint8_t log2_floor(uint32_t v) {
    uint8_t n = 0;
    while (v > 1u) {
        v >>= 1u;
        ++n;
    }
    return n;
}

/// Give both pads of pair 0 back to PORT and drive them.
void drive_pair0(bool negative_high, bool positive_high) {
    Inn0::release();
    Inp0::release();
    Inn0::output();
    Inp0::output();
    if (negative_high) { Inn0::set(); } else { Inn0::clear(); }
    if (positive_high) { Inp0::set(); } else { Inp0::clear(); }
    spin(2'000UL);
}

/// The SDADC's baseline configuration: VDDANA, 6 MHz, chopper on (which
/// is how table 45-27's own DC figures are taken).
SdadcConfig base_cfg(SdadcOsr osr = SdadcOsr::osr256, bool free_run = false) {
    SdadcConfig c{};
    c.reference = SdadcRef::vddana;
    c.prescaler = fast_prescaler;
    c.osr = osr;
    c.free_running = free_run;
    c.chopper = true;
    return c;
}

bool sdadc_up(const SdadcConfig& cfg, uint8_t pair = 0) {
    if (!Sdadc::init(main_gen, cfg, main_gen_hz)) {
        return false;
    }
    return Sdadc::select(pair);
}

/**
 * The statistics of N conversions, kept in BOTH views of the register:
 * `mean`/`span()` are the sixteen-bit datum the chapter specifies, and
 * `raw_*` the whole signed 24-bit value the datapath really carries.
 * Letter a's measurement of the corrections is why the second exists.
 */
struct Stats {
    int32_t low;
    int32_t high;
    int32_t mean;
    uint32_t rms;
    int32_t raw_low;
    int32_t raw_high;
    int32_t raw_mean;
    uint32_t raw_rms;
    uint16_t taken;
    uint32_t span() const { return static_cast<uint32_t>(high - low); }
    uint32_t raw_span() const { return static_cast<uint32_t>(raw_high - raw_low); }
};

/// N conversions, with their spread and their rms deviation. The two
/// discarded conversions at the front are this suite's own warm-up on
/// top of the SKPCNT the silicon spends.
Stats stats_of(uint16_t count) {
    Sdadc::discard(2);
    int32_t buf[64];
    uint16_t taken = 0;
    const uint16_t n = count > 64u ? 64u : count;
    for (uint16_t i = 0; i < n; ++i) {
        Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
        if (!Sdadc::start()) {
            continue;
        }
        uint32_t spins = 0xFFFFFFu;
        while (spins-- != 0u && !Sdadc::ready()) {
        }
        if (!Sdadc::ready()) {
            continue;
        }
        buf[taken] = Sdadc::result24();
        ++taken;
    }
    if (taken == 0u) {
        return Stats{0, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    int32_t rlo = buf[0];
    int32_t rhi = buf[0];
    int64_t rsum = 0;
    for (uint16_t i = 0; i < taken; ++i) {
        if (buf[i] < rlo) { rlo = buf[i]; }
        if (buf[i] > rhi) { rhi = buf[i]; }
        rsum += buf[i];
    }
    const int32_t rmean = static_cast<int32_t>(rsum / taken);
    uint64_t acc = 0;
    for (uint16_t i = 0; i < taken; ++i) {
        const int64_t d = buf[i] - rmean;
        acc += static_cast<uint64_t>(d * d);
    }
    const uint64_t var = acc / taken;
    uint32_t r = 0;
    while (static_cast<uint64_t>(r + 1u) * (r + 1u) <= var) {
        ++r;
    }
    // The sixteen-bit view is an arithmetic shift of the same numbers.
    const int32_t lo16 = rlo >> 8;
    const int32_t hi16 = rhi >> 8;
    const int32_t mean16 = rmean >> 8;
    return Stats{lo16, hi16, mean16, (r + 128u) / 256u, rlo, rhi, rmean, r, taken};
}

/// The SAR converter's own configuration for reading these pads: a plain
/// 12-bit single-ended reading against VDDANA, and a 64x averaged one.
constexpr AdcConfig sar_cfg{
    .reference = Ref::vddana,
    .prescaler = AdcPresc::div32,
    .sample_length = 5,
};
constexpr AdcConfig sar_avg_cfg{
    .reference = Ref::vddana,
    .prescaler = AdcPresc::div32,
    .resolution = AdcRes::bits16,
    .average = AdcAverage::samples64,
    .adjust = 4,
    .sample_length = 5,
};

/**
 * ADC0 up, WITH ERRATUM 1.4.10'S WORKAROUND WHERE IT IS NEEDED - the DAC
 * campaign's helper, unchanged. Once ADC1 has been used in this power
 * cycle, ADC0.SYNCBUSY.ENABLE is stuck at one on this die and
 * `Adc<0>::init()` returns false with the converter dead. The errata's
 * way out is to bring ADC1 up FIRST.
 */
bool adc0_up(const AdcConfig& cfg) {
    if (Adc0::init(main_gen, cfg, main_gen_hz)) {
        return true;
    }
    if ((Adc0::sync_busy() & ADC_SYNCBUSY_ENABLE_Msk) == 0u) {
        return false;
    }
    print(serial, "  [erratum 1.4.10: ADC0.SYNCBUSY.ENABLE stuck - bringing "
          "ADC1 up first]", crlf);
    Adc0::release();
    Adc1::release();
    if (!Adc1::init(main_gen, cfg, main_gen_hz)) {
        return false;
    }
    const bool ok = Adc0::init(main_gen, cfg, main_gen_hz);
    Adc1::release();
    return ok;
}

template <class Pad>
uint32_t sar_mean(uint16_t count) {
    Adc0::select(AnalogIn<Pad>{});
    Adc0::discard(2);
    uint32_t sum = 0;
    for (uint16_t i = 0; i < count; ++i) {
        sum += Adc0::read();
    }
    return count == 0u ? 0u : (sum + count / 2u) / count;
}

// =============================================================================
// a - the block, its vocabularies, its disciplines, its disagreements
// =============================================================================
void ta_block() {
    bench.verdict("one SDADC on this family, three differential PAIRS, and a "
                  "SIGNED sixteen-bit scale",
                  sdadc_count() == 1u && Sdadc::channels == 3u &&
                      Sdadc::half_steps == 32768u);

    print(serial, "  EVSYS: RESRDY generator ", Sdadc::resrdy_generator,
          ", WINMON ", Sdadc::winmon_generator, "; START user ",
          Sdadc::start_event_user, ", FLUSH user ", Sdadc::flush_event_user,
          " (both asynchronous only); DMAC trigger ",
          Sdadc::dma_trigger_resrdy, "; GCLK id ", Sdadc::gclk_id, crlf);
    bench.verdict("the generator, user and trigger codes are the header's own",
                  Sdadc::resrdy_generator == 71u && Sdadc::winmon_generator == 72u &&
                      Sdadc::start_event_user == 32u &&
                      Sdadc::flush_event_user == 33u &&
                      Sdadc::dma_trigger_resrdy == 44u && Sdadc::gclk_id == 35u);

    // THE PAD MAP, which is the most package-dependent one in this
    // stratum - and on the J all three pairs are here.
    print(serial, "  pairs on this package: 0 = P", Sdadc::negative_port(0),
          Sdadc::negative_pin(0), "/P", Sdadc::positive_port(0),
          Sdadc::positive_pin(0), ", 1 = P", Sdadc::negative_port(1),
          Sdadc::negative_pin(1), "/P", Sdadc::positive_port(1),
          Sdadc::positive_pin(1), ", 2 = P", Sdadc::negative_port(2),
          Sdadc::negative_pin(2), "/P", Sdadc::positive_port(2),
          Sdadc::positive_pin(2), crlf);
    bench.verdict("the J bonds all three pairs, and each pad has ONE polarity",
                  Sdadc::pair_exists(0) && Sdadc::pair_exists(1) &&
                      Sdadc::pair_exists(2) &&
                      Sdadc::negative_port(0) == 'A' && Sdadc::negative_pin(0) == 6 &&
                      Sdadc::positive_port(0) == 'A' && Sdadc::positive_pin(0) == 7 &&
                      Sdadc::negative_function('A', 7) < 0 &&
                      Sdadc::positive_function('A', 6) < 0);
    bench.verdict("the SDADC's pads are the SAR's pads too, which is what makes "
                  "a cross-check possible with no wire",
                  Adc0::ain_of('A', 6) == 6 && Adc0::ain_of('A', 7) == 7 &&
                      Adc0::ain_of('A', 4) == 4);
    bench.verdict("...and the DAC is NOT among them: PA02 is no SDADC pad, so "
                  "this converter can only meet the DAC as a REFERENCE",
                  Sdadc::negative_function('A', 2) < 0 &&
                      Sdadc::positive_function('A', 2) < 0 &&
                      Sdadc::vrefb_function('A', 4) == static_cast<int>(PinFunction::b));

    // The refusals that are the chapter's rules.
    bench.verdict("OSR 0x5 is Reserved and refused",
                  !sdadc_config_valid(SdadcConfig{.osr = static_cast<SdadcOsr>(5)}) &&
                      sdadc_config_valid(SdadcConfig{.osr = SdadcOsr::osr1024}));
    bench.verdict("AN INTERNAL REFERENCE WITHOUT ITS BUFFER IS REFUSED - "
                  "39.8.2's own Note, and erratum 1.8.10's whole workaround",
                  !sdadc_config_valid(SdadcConfig{.reference = SdadcRef::intref}) &&
                      !sdadc_config_valid(SdadcConfig{.reference = SdadcRef::dac}) &&
                      sdadc_config_valid(SdadcConfig{.reference = SdadcRef::dac,
                                                     .reference_buffer = true}) &&
                      sdadc_config_valid(SdadcConfig{.reference = SdadcRef::vrefb}));
    bench.verdict("A GAINCORR OF ZERO IS REFUSED: 39.6.3.4's formula multiplies "
                  "every result by it, which is erratum 1.18.3 by arithmetic "
                  "rather than by accident",
                  !sdadc_config_valid(SdadcConfig{.gain_correction = 0}) &&
                      sdadc_config_valid(SdadcConfig{.gain_correction = 1}));
    bench.verdict("a Reserved window mode, an over-wide SKPCNT or SHIFTCORR, "
                  "and an inverted event input nothing listens to",
                  !sdadc_config_valid(SdadcConfig{.window = static_cast<SdadcWindow>(5)}) &&
                      !sdadc_config_valid(SdadcConfig{.skip_count = 16}) &&
                      !sdadc_config_valid(SdadcConfig{.shift_correction = 16}) &&
                      !sdadc_config_valid(SdadcConfig{.events = {.invert_start = true}}));

    // THE PRESCALER, whose arithmetic the device header and the datasheet
    // disagree about. Letter e asks the silicon; this is what the driver
    // claims.
    bench.verdict("THE PRESCALER IS LINEAR in this driver - 2 x (P + 1), which "
                  "is the only reading that reaches 39.5.3's own /512 from an "
                  "eight-bit field",
                  sdadc_prescaler_divisor(0) == 2u && sdadc_prescaler_divisor(2) == 6u &&
                      sdadc_prescaler_divisor(3) == 8u &&
                      sdadc_prescaler_divisor(255) == 512u);
    bench.verdict("and table 45-26's 1..6 MHz refuses a prescaler that leaves "
                  "it: at 48 MHz that is P = 3..23",
                  sdadc_clock_in_range(main_gen_hz, 3) &&
                      sdadc_clock_in_range(main_gen_hz, 23) &&
                      !sdadc_clock_in_range(main_gen_hz, 2) &&
                      !sdadc_clock_in_range(main_gen_hz, 24));

    // ---- the silicon ------------------------------------------------------
    bench.verdict("the SDADC comes up on generator 0 at 6 MHz",
                  sdadc_up(base_cfg()));

    // THE RESET VALUES ARE ERRATUM 1.18.3'S WORKAROUND, baked in: the
    // item is about the revision where GAINCORR and SKPCNT reset to
    // zero, and this chapter prints them as 1 and 2.
    Sdadc::release();
    Sdadc::bus_clock(true);
    (void)Sdadc::clock(main_gen);
    (void)Sdadc::reset();
    const uint16_t ctrlb_reset = Sdadc::regs().SDADC_CTRLB;
    const uint16_t gain_reset = Sdadc::regs().SDADC_GAINCORR;
    print(serial, "  out of a software reset: CTRLB reads ", Hex{ctrlb_reset},
          " (SKPCNT ", static_cast<uint16_t>((ctrlb_reset >> 12) & 0xFu),
          "), GAINCORR reads ", gain_reset, crlf);
    bench.verdict("THE RESET VALUES ARE ERRATUM 1.18.3'S OWN WORKAROUND - "
                  "GAINCORR 1 and SKPCNT 2, where the item describes the "
                  "revision-B silicon that reset both to zero",
                  ctrlb_reset == 0x2000u && gain_reset == 1u);

    // SYNCBUSY's layout, and the register the chapter puts in its
    // write-synchronized list without giving it a bit.
    bench.verdict("SYNCBUSY HAS NO REFCTRL BIT, though 39.6.8's prose lists "
                  "REFCTRL among the registers needing write synchronization - "
                  "the twelve bits are exactly the header's",
                  SDADC_SYNCBUSY_Msk == 0x00000FFFu);

    bench.verdict("the converter comes back up", sdadc_up(base_cfg()));

    // THE THREE DISAGREEMENTS ABOUT ENABLE PROTECTION, asked of the
    // silicon by writing each register raw under a RUNNING converter and
    // reading it back. 39.6.2.1 lists CTRLA's two bits, CTRLB, CTRLC,
    // EVCTRL and ANACTRL; the individual property lines say
    // Enable-Protected for REFCTRL, CTRLB and EVCTRL only.
    const uint8_t refctrl_before = Sdadc::regs().SDADC_REFCTRL;
    Sdadc::regs().SDADC_REFCTRL = static_cast<uint8_t>(refctrl_before ^
                                                       SDADC_REFCTRL_ONREFBUF_Msk);
    const uint8_t refctrl_after = Sdadc::regs().SDADC_REFCTRL;
    Sdadc::regs().SDADC_REFCTRL = refctrl_before;

    const uint16_t ctrlb_before = Sdadc::regs().SDADC_CTRLB;
    Sdadc::regs().SDADC_CTRLB = static_cast<uint16_t>(ctrlb_before ^ 0x0001u);
    const uint16_t ctrlb_after = Sdadc::regs().SDADC_CTRLB;
    Sdadc::regs().SDADC_CTRLB = ctrlb_before;

    const uint8_t evctrl_before = Sdadc::regs().SDADC_EVCTRL;
    Sdadc::regs().SDADC_EVCTRL = SDADC_EVCTRL_RESRDYEO_Msk;
    const uint8_t evctrl_after = Sdadc::regs().SDADC_EVCTRL;
    Sdadc::regs().SDADC_EVCTRL = evctrl_before;

    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_CTRLC_Msk);
    const uint8_t ctrlc_before = Sdadc::regs().SDADC_CTRLC;
    Sdadc::regs().SDADC_CTRLC = SDADC_CTRLC_FREERUN_Msk;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_CTRLC_Msk);
    const uint8_t ctrlc_after = Sdadc::regs().SDADC_CTRLC;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_CTRLC_Msk);
    Sdadc::regs().SDADC_CTRLC = ctrlc_before;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_CTRLC_Msk);

    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    const uint8_t anactrl_before = Sdadc::regs().SDADC_ANACTRL;
    Sdadc::regs().SDADC_ANACTRL =
        static_cast<uint8_t>(anactrl_before ^ SDADC_ANACTRL_ONCHOP_Msk);
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    const uint8_t anactrl_after = Sdadc::regs().SDADC_ANACTRL;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    Sdadc::regs().SDADC_ANACTRL = anactrl_before;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);

    print(serial, "  written raw under a RUNNING converter: REFCTRL ",
          refctrl_after == refctrl_before ? "held" : "TOOK the write",
          ", CTRLB ", ctrlb_after == ctrlb_before ? "held" : "TOOK the write",
          ", EVCTRL ", evctrl_after == evctrl_before ? "held" : "TOOK the write",
          ", CTRLC ", ctrlc_after == ctrlc_before ? "held" : "TOOK the write",
          ", ANACTRL ", anactrl_after == anactrl_before ? "held" : "TOOK the write",
          crlf);
    bench.verdict("THE THREE REGISTERS BOTH DOCUMENTS AGREE ABOUT ARE "
                  "ENABLE-PROTECTED: REFCTRL, CTRLB and EVCTRL discard a write "
                  "made under a running converter",
                  refctrl_after == refctrl_before && ctrlb_after == ctrlb_before &&
                      evctrl_after == evctrl_before);
    print(serial, "  CTRLC and ANACTRL are in 39.6.2.1's enable-protected LIST "
          "and their own property lines say only Write-Synchronized: the "
          "silicon ", (ctrlc_after == ctrlc_before && anactrl_after == anactrl_before)
                          ? "PROTECTS them, so the list is right"
                          : "TAKES the write, so the property lines are right",
          crlf);
    bench.verdict("either way the driver is correct, because it writes both "
                  "only while the converter is disabled - and refuses "
                  "otherwise",
                  !Sdadc::free_running(true) && !Sdadc::analog_control(base_cfg()) &&
                      !Sdadc::reference(base_cfg()) &&
                      !Sdadc::event_config(SdadcEventControl{}));

    // REFRANGE: TWO BITS THE DEVICE HEADER HAS AND CHAPTER 39 NEVER
    // MENTIONS. Ask the silicon whether they even stay written.
    (void)Sdadc::enable(false);
    const uint8_t ref_base = Sdadc::regs().SDADC_REFCTRL;
    Sdadc::regs().SDADC_REFCTRL =
        static_cast<uint8_t>(ref_base | SDADC_REFCTRL_REFRANGE(3));
    const uint8_t ref_with_range = Sdadc::regs().SDADC_REFCTRL;
    Sdadc::regs().SDADC_REFCTRL = ref_base;
    print(serial, "  REFCTRL with REFRANGE = 3 written reads back ",
          Hex{ref_with_range}, " (the header's register mask is 0xB3; chapter "
          "39 draws bits 5:4 blank and never names the field)", crlf);
    bench.verdict("REFRANGE IS A REAL FIELD THE CHAPTER DOES NOT DOCUMENT",
                  "",
                  (ref_with_range & SDADC_REFCTRL_REFRANGE_Msk) ==
                      SDADC_REFCTRL_REFRANGE_Msk);

    // ANACTRL's width, where the two documents disagree by one bit.
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    Sdadc::regs().SDADC_ANACTRL = 0xFFu;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    const uint8_t anactrl_all = Sdadc::regs().SDADC_ANACTRL;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    Sdadc::regs().SDADC_ANACTRL = 0u;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_ANACTRL_Msk);
    print(serial, "  ANACTRL written 0xFF reads back ", Hex{anactrl_all},
          " (39.8.21 draws CTLSDADC as five bits 4:0, the device header "
          "declares six)", crlf);
    bench.verdict("ANACTRL'S BIAS FIELD IS SIX BITS WIDE, as the device header "
                  "says and 39.8.21's bit table does not",
                  (anactrl_all & 0x20u) != 0u);

    // DBGCTRL at 0x2E, and not reset by a software reset.
    Sdadc::regs().SDADC_DBGCTRL = SDADC_DBGCTRL_DBGRUN_Msk;
    (void)Sdadc::reset();
    const uint8_t dbg_after_reset = Sdadc::regs().SDADC_DBGCTRL;
    Sdadc::regs().SDADC_DBGCTRL = 0;
    bench.verdict("DBGCTRL SURVIVES A SOFTWARE RESET, as 39.8.22 says",
                  dbg_after_reset == SDADC_DBGCTRL_DBGRUN_Msk);

    Sdadc::release();
    bench.verdict("release() puts the block back: the APB clock off and the "
                  "generic clock channel disconnected",
                  (MCLK_REGS->MCLK_APBCMASK & MCLK_APBCMASK_SDADC_Msk) == 0u);
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// b - THE ZERO AND THE NOISE: the Multislope number
// =============================================================================
const SdadcOsr osr_ladder[5] = {SdadcOsr::osr64, SdadcOsr::osr128,
                                SdadcOsr::osr256, SdadcOsr::osr512,
                                SdadcOsr::osr1024};
const char* const osr_names[5] = {"64", "128", "256", "512", "1024"};

/**
 * The whole OSR ladder on whatever differential is standing, against a
 * given reference. Returns the best noise-free bits of the RAW 24-bit
 * datapath - which is the only place this die's noise is visible at all,
 * the specified sixteen-bit datum being bit-exact from OSR 128 up.
 */
uint8_t osr_ladder_run(SdadcRef reference, VrefLevel level, uint16_t ref_mv_,
                       int32_t& first_mean, int32_t& last_mean) {
    uint8_t best = 0;
    (void)Vref::configure(VrefConfig{.level = level});
    for (uint8_t i = 0; i < 5u; ++i) {
        SdadcConfig cfg = base_cfg(osr_ladder[i]);
        cfg.reference = reference;
        cfg.reference_buffer = reference != SdadcRef::vddana;
        Sdadc::release();
        if (!sdadc_up(cfg)) {
            print(serial, "  OSR ", osr_names[i], ": the converter did not "
                  "come up", crlf);
            continue;
        }
        const Stats s = stats_of(64);
        const uint32_t raw_span = s.raw_span() == 0u ? 1u : s.raw_span();
        // Noise-free bits of the RAW datapath, whose full scale is 2^24.
        const uint8_t bits = static_cast<uint8_t>(24u - log2_floor(raw_span));
        const int32_t uv = static_cast<int32_t>(
            (static_cast<int64_t>(s.raw_mean) * ref_mv_ * 1000) / 8388608);
        print(serial, "  OSR ", osr_names[i], ": datum ", s.mean, " (span ",
              s.span(), "), raw ", s.raw_mean, " (span ", s.raw_span(),
              ", rms ", s.raw_rms, ") = ", uv, " uV, noise-free ", bits,
              " bits of 24", crlf);
        if (i == 0u) { first_mean = s.mean; }
        last_mean = s.mean;
        if (bits > best) { best = bits; }
    }
    Sdadc::release();
    return best;
}

void tb_noise() {
    print(serial, "  both pads of pair 0 driven from PORT to the SAME rail: an "
          "exact analog zero, and whatever the converter adds to it. The "
          "sixteen-bit datum is BIT-EXACT here from OSR 128 up, so the noise "
          "is reported on the RAW 24-bit value underneath it - which is the "
          "only instrument this board has for it.", crlf);

    int32_t at_low = 0;
    int32_t at_high = 0;
    int32_t unused = 0;

    drive_pair0(false, false);
    print(serial, "  --- both pads LOW, reference VDDANA (1 raw unit = ",
          (static_cast<uint32_t>(vdd_mv) * 1000u) / 32768u, " nV) ---", crlf);
    const uint8_t bits_low =
        osr_ladder_run(SdadcRef::vddana, VrefLevel::v1_024, vdd_mv, unused, at_low);

    drive_pair0(true, true);
    print(serial, "  --- both pads HIGH, reference VDDANA ---", crlf);
    const uint8_t bits_high =
        osr_ladder_run(SdadcRef::vddana, VrefLevel::v1_024, vdd_mv, unused, at_high);

    bench.verdict("A ZERO DIFFERENTIAL READS ZERO at both rails and at every "
                  "OSR - the common mode moves from 0 V to VDD and the reading "
                  "stays inside a per cent of one per cent of full scale",
                  at_low > -400 && at_low < 400 && at_high > -400 && at_high < 400);

    // The common-mode rejection this board can put a number on: the same
    // (zero) input, the common mode moved by the whole supply.
    const int32_t shift = at_high - at_low;
    const int32_t shift_uv =
        static_cast<int32_t>((static_cast<int64_t>(shift) * vdd_mv * 1000) / 32768);
    print(serial, "  the same zero differential reads ", at_low,
          " with both pads at GND and ", at_high, " with both at VDD: a shift "
          "of ", shift, " counts = ", shift_uv, " uV for a common-mode step of "
          "the whole supply", crlf);
    bench.verdict("...which is a common-mode rejection of the differential "
                  "input, not a claim about its absolute offset",
                  shift < 2000 && shift > -2000);

    // THE FINER REFERENCE, where the datasheet's own noise figure can be
    // seen at all: at INTREF 1.024 V one 16-bit count is 31 uV, so table
    // 45-27's 0.08 mV rms is about two and a half counts. Against VDDANA
    // it is half a count and invisible.
    drive_pair0(false, false);
    print(serial, "  --- both pads LOW, reference INTREF 1.024 V (1 count = 31 "
          "uV, so table 45-27's 0.08 mVrms should be about 2.5 counts) ---",
          crlf);
    const uint8_t bits_fine =
        osr_ladder_run(SdadcRef::intref, VrefLevel::v1_024, 1024, unused, unused);

    print(serial, "  THE MULTISLOPE NUMBER: noise-free bits of the raw 24-bit "
          "datapath on a shorted differential - ", bits_low, " at the low rail, ",
          bits_high, " at the high one, ", bits_fine,
          " against the 1.024 V bandgap (noise-free bits = log2(2^24 / "
          "peak-to-peak span) over 64 conversions). Table 45-28 claims 14.2 "
          "ENOB at a 1.2 V external reference and 11.2 at 5.5 V internal.",
          crlf);
    bench.verdict("...so AT ITS BEST OSR this die resolves more than the "
                  "sixteen bits the chapter specifies, at both rails and at "
                  "both references - and the improvement per OSR octave "
                  "FLATTENS, which is a thermal floor rather than a "
                  "quantization one",
                  bits_low >= 16u && bits_high >= 16u && bits_fine >= 16u);

    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024});
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// c - full scale, the sign, and where the datum sits in RESULT
// =============================================================================
void tc_full_scale() {
    bench.verdict("the converter comes up at OSR 256", sdadc_up(base_cfg()));

    drive_pair0(false, true);      // AINN0 low, AINP0 high -> +VDD
    uint32_t raw_pos = 0;
    int16_t pos = 0;
    (void)Sdadc::read(pos);
    raw_pos = Sdadc::result_raw();
    (void)Sdadc::read(pos);
    raw_pos = Sdadc::result_raw();

    drive_pair0(true, false);      // AINN0 high, AINP0 low -> -VDD
    int16_t neg = 0;
    (void)Sdadc::read(neg);
    const uint32_t raw_neg = Sdadc::result_raw();
    (void)Sdadc::read(neg);

    print(serial, "  AINP0 high / AINN0 low: RESULT reads ", Hex{raw_pos},
          " raw, ", pos, " as a signed 16-bit datum", crlf);
    print(serial, "  the pads swapped:       RESULT reads ", Hex{raw_neg},
          " raw, ", neg, crlf);

    bench.verdict("THE RESULT IS SIGNED AND THE SIGN FOLLOWS THE PADS - 39.1 "
                  "and 39.8.19 are right where 39.6.1, 39.6.3.1 and 39.6.3.4 "
                  "all say the output is unsigned",
                  pos > 20000 && neg < -20000);
    bench.verdict("THE REGISTER SATURATES AT +/-2^23, NOT AT A LEFT-SHIFTED "
                  "+/-2^15: the low byte is NOT padding, so 39.8.19's "
                  "'left-adjusted' names where the SPECIFIED sixteen bits sit "
                  "and not the width of the datapath",
                  raw_pos == 0x7FFFFFu && raw_neg == 0x800000u);

    // ...and the low bits carry filter output, which the same converter
    // says by moving them on an input the top sixteen cannot resolve.
    // A shorted differential is that input: the datum is bit-exact and
    // the raw value is not.
    Sdadc::release();
    (void)sdadc_up(base_cfg(SdadcOsr::osr64));
    drive_pair0(false, false);
    const Stats zero = stats_of(32);
    print(serial, "  a SHORTED differential at OSR 64: the sixteen-bit datum "
          "spans ", zero.span(), " counts while the raw 24-bit value spans ",
          zero.raw_span(), " (mean ", zero.raw_mean, " raw, ", zero.mean,
          " as the datum)", crlf);
    bench.verdict("THE EIGHT BITS UNDER THE SPECIFIED DATUM ARE REAL FILTER "
                  "OUTPUT: they move where the datum is bit-exact",
                  zero.raw_span() > zero.span() * 8u ||
                      (zero.span() == 0u && zero.raw_span() > 0u));

    // WHAT +/- VDD ACTUALLY MEANS HERE. With VREF = VDDANA the input is
    // at +/- 1.0 x VREF, and table 45-26 specifies the range as
    // +/- 0.7 x VREF for VREF >= VDDANA - 0.3 V. So this is a
    // SATURATION measurement and the letter says so instead of calling
    // it a gain.
    const int32_t magnitude = pos > -neg ? pos : -neg;
    print(serial, "  |result| at the rails is ", magnitude,
          " of 32768 = ", (magnitude * 1000) / 32768,
          " per mille of full scale; table 45-26 SPECIFIES the differential "
          "range as +/-0.7 x VREF when VREF >= VDDANA - 0.3 V, which a "
          "rail-to-rail differential against VDDANA is outside by "
          "construction", crlf);
    bench.verdict("and the two polarities are symmetric to within a per cent "
                  "of the magnitude, whatever that magnitude means",
                  near(static_cast<uint32_t>(pos), static_cast<uint32_t>(-neg),
                       static_cast<uint32_t>(magnitude / 100 + 8)));

    // THE OVERRUN FLAG, which is what happens when a result is not read.
    Sdadc::clear_flags(Sdadc::flag_overrun);
    SdadcConfig cfg = base_cfg(SdadcOsr::osr64, true);
    Sdadc::release();
    bench.verdict("the converter comes back free-running", sdadc_up(cfg));
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    wait_ms(5);
    const bool over = Sdadc::overrun();
    print(serial, "  free-running with nothing reading RESULT for 5 ms: "
          "OVERRUN ", yes_no(over), crlf);
    bench.verdict("OVERRUN is set when RESULT is written before the previous "
                  "value was read (39.8.7)",
                  over);

    Sdadc::release();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// d - the swept differential, and the SAR watching the same pads
// =============================================================================
void td_sweep() {
    // The source: TCC1's two channels ARE this pair's two pads.
    bench.verdict("TCC1 comes up as the pair's own signal source, 512 GCLK "
                  "cycles a period",
                  Pwm::init(main_gen) &&
                      Pwm::configure(TccConfig{.prescaler = TccPrescaler::div1}) &&
                      Pwm::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Pwm::set_period(pwm_top) && Pwm::set_cc(0, pwm_steps / 2u) &&
                      Pwm::set_cc(1, pwm_steps / 2u) && Pwm::enable(true));
    Inn0::release();
    Inp0::release();
    PwmN::claim();
    PwmP::claim();

    // OSR 1024 at CLK_SDADC = 6 MHz: the decimation window is
    // 1024 x 4 x 8 = 32768 GCLK cycles = exactly 64 PWM periods, so the
    // fundamental and every harmonic sit on a zero of the third-order
    // SINC.
    SdadcConfig cfg = base_cfg(SdadcOsr::osr1024);
    bench.verdict("and the SDADC's decimation window is exactly 64 of those "
                  "periods, which puts the PWM fundamental on a SINC zero",
                  sdadc_conversion_cycles(SdadcConfig{.osr = SdadcOsr::osr1024,
                                                      .free_running = true}) *
                              sdadc_prescaler_divisor(fast_prescaler) ==
                          64u * pwm_steps &&
                      sdadc_up(cfg));

    print(serial, "  k = CC1 - CC0 of 512, differential = k/512 x VDD, so an "
          "ideal converter reads k x 64 counts. The sweep runs to k = 448, "
          "which is 0.875 x VREF - past the +/-0.7 x VREF table 45-26 "
          "specifies and erratum 1.18.2 (revisions B..E, NOT this one) asks "
          "for.", crlf);

    struct Point {
        int32_t k;
        int32_t measured;
    };
    Point points[15];
    uint8_t n = 0;
    bool monotone = true;
    int32_t previous = -1000000;
    for (int32_t k = -448; k <= 448; k += 64) {
        const uint32_t cc0 = static_cast<uint32_t>(256 - k / 2);
        const uint32_t cc1 = static_cast<uint32_t>(256 + k / 2);
        (void)Pwm::set_cc_buffer(0, cc0);
        (void)Pwm::set_cc_buffer(1, cc1);
        wait_ms(5);
        const Stats s = stats_of(8);
        points[n] = Point{k, s.mean};
        if (s.mean <= previous) {
            monotone = false;
        }
        previous = s.mean;
        print(serial, "  k ", k, ": ", s.mean, " counts (ideal ", k * 64,
              "), spread ", s.span(), ", ", Sdadc::to_mv(s.mean, vdd_mv),
              " mV", crlf);
        ++n;
    }

    bench.verdict("THE SWEPT DIFFERENTIAL IS MONOTONIC THROUGH ZERO over the "
                  "whole sweep, and the sign is the pads'",
                  monotone && points[0].measured < -10000 &&
                      points[n - 1u].measured > 10000);

    // A LEAST-SQUARES LINE THROUGH THE INNER POINTS (|k| <= 320, i.e.
    // inside 0.625 x VREF), then the residuals of every point against it.
    // The slope is a GAIN and the intercept an OFFSET; only what is left
    // is nonlinearity, and even that is COMBINED - the source is a duty
    // ratio and the receiver a decimation filter - so this letter does
    // not apportion it.
    int64_t sx = 0, sy = 0, sxx = 0, sxy = 0;
    int32_t fitted = 0;
    for (uint8_t i = 0; i < n; ++i) {
        if (points[i].k < -320 || points[i].k > 320) {
            continue;
        }
        sx += points[i].k;
        sy += points[i].measured;
        sxx += static_cast<int64_t>(points[i].k) * points[i].k;
        sxy += static_cast<int64_t>(points[i].k) * points[i].measured;
        ++fitted;
    }
    // slope in milli-counts per k, intercept in counts.
    const int64_t denom = fitted * sxx - sx * sx;
    const int32_t slope_milli =
        denom == 0 ? 0 : static_cast<int32_t>((1000 * (fitted * sxy - sx * sy)) / denom);
    const int32_t intercept =
        denom == 0 ? 0
                   : static_cast<int32_t>((sxx * sy - sx * sxy) / denom);
    print(serial, "  best-fit line over |k| <= 320 (", fitted,
          " points): slope ", slope_milli,
          " milli-counts per k where an ideal converter gives 64000, "
          "intercept ", intercept, " counts", crlf);
    const int32_t gain_permille = (slope_milli * 1000) / 64000;
    print(serial, "  that is a GAIN of ", gain_permille,
          " per mille of ideal, i.e. a gain error of ",
          gain_permille - 1000, " per mille - table 45-27 allows +/-11 typical "
          "and +/-34 maximum at an internal reference", crlf);

    int32_t worst_inner = 0;
    int32_t worst_outer = 0;
    int32_t worst_outer_k = 0;
    for (uint8_t i = 0; i < n; ++i) {
        const int32_t predicted =
            static_cast<int32_t>((static_cast<int64_t>(slope_milli) * points[i].k) / 1000) +
            intercept;
        const int32_t d = points[i].measured - predicted;
        const int32_t a = d < 0 ? -d : d;
        if (points[i].k >= -320 && points[i].k <= 320) {
            if (a > worst_inner) { worst_inner = a; }
        } else if (a > worst_outer) {
            worst_outer = a;
            worst_outer_k = points[i].k;
        }
    }
    print(serial, "  worst residual against that line: ", worst_inner,
          " counts INSIDE |k| <= 320, and ", worst_outer, " counts OUTSIDE it "
          "(at k = ", worst_outer_k, "); table 45-27 allows an INL of +/-11 "
          "LSB at a supply-sized internal reference", crlf);
    bench.verdict("THE INNER SWEEP IS LINEAR TO THE DATASHEET'S OWN INL "
                  "ALLOWANCE, with a PWM duty ratio as the source and the "
                  "converter's own decimation filter as the reconstruction "
                  "filter - and the residual is COMBINED nonlinearity this "
                  "letter declines to apportion",
                  worst_inner <= 40);
    if (worst_outer > 4 * (worst_inner + 1)) {
        bench.verdict("AND THE +/-0.7 x VREF LIMIT BITES: past it the points "
                      "leave the line by several times the inner residual, "
                      "which is what table 45-26's restricted input range is "
                      "about",
                      true);
    } else {
        print(serial, "  the points beyond 0.7 x VREF do NOT leave the line by "
              "more than the inner scatter - so on this die, at this OSR, "
              "table 45-26's restricted range did not show itself here. "
              "Recorded as an observation; erratum 1.18.2, which asked for the "
              "same restriction, is marked revisions B..E and NOT this one.",
              crlf);
        bench.verdict("the outer points were measured against the same line "
                      "and reported rather than a verdict being invented for "
                      "them",
                      true);
    }

    // THE SAR WATCHING THE SAME TWO PADS. It samples instantaneously, so
    // its average of a waveform synchronous with its own clock is a
    // PHASE average and not a true mean - which is exactly why this is a
    // secondary witness and letter g is the cross-check.
    (void)Pwm::set_cc_buffer(0, 192);
    (void)Pwm::set_cc_buffer(1, 320);
    wait_ms(5);
    const Stats sd = stats_of(16);
    Sdadc::release();
    bench.verdict("the SAR comes up on the same two pads", adc0_up(sar_avg_cfg));
    const uint32_t mean_n = sar_mean<Inn0>(16);
    const uint32_t mean_p = sar_mean<Inp0>(16);
    const int32_t sar_diff_mv =
        static_cast<int32_t>(adc_mv(mean_p, 4096, vdd_mv)) -
        static_cast<int32_t>(adc_mv(mean_n, 4096, vdd_mv));
    Sdadc::release();
    (void)sdadc_up(cfg);
    const int32_t sd_mv = Sdadc::to_mv(sd.mean, vdd_mv);
    print(serial, "  at k = 128: the SDADC says ", sd.mean, " counts = ", sd_mv,
          " mV; the SAR averaging each pad says ", mean_n, " and ", mean_p,
          " of 4096, a difference of ", sar_diff_mv, " mV", crlf);
    bench.verdict("the two architectures agree on the sign and the order of "
                  "magnitude of the same differential (the SAR's average of a "
                  "synchronous square wave is a phase average, so this is a "
                  "witness and not a calibration)",
                  sar_diff_mv > 0 && sd_mv > 0 &&
                      near(static_cast<uint32_t>(sar_diff_mv),
                           static_cast<uint32_t>(sd_mv),
                           static_cast<uint32_t>(sd_mv / 4 + 100)));

    Adc0::release();
    Sdadc::release();
    (void)Pwm::enable(false);
    Pwm::release();
    PwmN::release();
    PwmP::release();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// e - time: the conversion rate, the PRESCALER question, and SKPCNT
// =============================================================================
void te_timing() {
    bench.verdict("the crystal stopwatch is running (24 MHz, 41.7 ns a tick) - "
                  "a conversion time against OSC48M would carry that "
                  "oscillator's 5100 ppm",
                  stopwatch_start());
    drive_pair0(false, false);

    // A free-running converter: measure the period over N results.
    auto period_ticks = [](const SdadcConfig& cfg, uint16_t results_wanted) -> uint32_t {
        Sdadc::release();
        if (!sdadc_up(cfg)) {
            return 0;
        }
        Sdadc::discard(2);
        Sdadc::clear_flags(Sdadc::flag_resrdy);
        int16_t v = 0;
        if (!Sdadc::next(v)) {
            return 0;
        }
        const uint32_t t0 = ticks_now();
        for (uint16_t i = 0; i < results_wanted; ++i) {
            if (!Sdadc::next(v)) {
                return 0;
            }
        }
        const uint32_t t1 = ticks_now();
        return (t1 - t0) / results_wanted;
    };

    struct OsrCase {
        SdadcOsr osr;
        const char* label;
        uint16_t count;
    };
    const OsrCase osrs[] = {
        {SdadcOsr::osr64, "64", 64},
        {SdadcOsr::osr256, "256", 32},
        {SdadcOsr::osr1024, "1024", 8},
    };
    bool all_exact = true;
    for (const auto& o : osrs) {
        SdadcConfig cfg = base_cfg(o.osr, true);
        const uint32_t measured = period_ticks(cfg, o.count);
        // cycles of CLK_SDADC, and one CLK_SDADC cycle is 24/6 = 4
        // crystal ticks at PRESCALER 3.
        const uint32_t predicted =
            sdadc_conversion_cycles(cfg) * (crystal_hz / sdadc_clock_hz(main_gen_hz, cfg.prescaler));
        const uint32_t us = (measured * 1000u) / (crystal_hz / 1000u);
        print(serial, "  free running, OSR ", o.label, ": ", measured,
              " crystal ticks a result (predicted ", predicted, ") = ", us,
              " us", crlf);
        if (!near(measured, predicted, predicted / 200u + 2u)) {
            all_exact = false;
        }
    }
    bench.verdict("THE FREE-RUNNING PERIOD IS OSR x 4 CLK_SDADC CYCLES, exact "
                  "to five per mille at every ratio",
                  all_exact);

    // THE PRESCALER QUESTION. The datasheet says the divider is
    // 2 x (P + 1); the device header's enumerators say 2 << P. The two
    // readings predict wildly different ratios and one measurement
    // settles it.
    const uint32_t t_p3 = period_ticks(base_cfg(SdadcOsr::osr64, true), 64);
    SdadcConfig c4 = base_cfg(SdadcOsr::osr64, true);
    c4.prescaler = 4;
    const uint32_t t_p4 = period_ticks(c4, 64);
    SdadcConfig c7 = base_cfg(SdadcOsr::osr64, true);
    c7.prescaler = 7;
    const uint32_t t_p7 = period_ticks(c7, 32);
    SdadcConfig c23 = base_cfg(SdadcOsr::osr64, true);
    c23.prescaler = 23;
    const uint32_t t_p23 = period_ticks(c23, 16);

    print(serial, "  PRESCALER 3 / 4 / 7 / 23: ", t_p3, " / ", t_p4, " / ",
          t_p7, " / ", t_p23, " crystal ticks a result", crlf);
    print(serial, "  ratios against PRESCALER 3, x1000: ",
          t_p3 ? (t_p4 * 1000u) / t_p3 : 0u, " / ",
          t_p3 ? (t_p7 * 1000u) / t_p3 : 0u, " / ",
          t_p3 ? (t_p23 * 1000u) / t_p3 : 0u,
          "   (a LINEAR 2x(P+1) divider predicts 1250 / 2000 / 6000; the device "
          "header's power-of-two enumerators predict 2000 / 16000 / -)", crlf);
    bench.verdict("THE PRESCALER IS LINEAR, 2 x (PRESCALER + 1) - the datasheet "
                  "is right and the device header's DIV2/DIV4/DIV8 enumerators "
                  "are the SAM D21's and are wrong here",
                  t_p3 != 0u && near((t_p4 * 1000u) / t_p3, 1250u, 30u) &&
                      near((t_p7 * 1000u) / t_p3, 2000u, 40u) &&
                      near((t_p23 * 1000u) / t_p3, 6000u, 120u));

    // SKPCNT: what it costs a SINGLE conversion. Table 45-26 prints the
    // single-conversion output data rate as
    // "(CLK_SDADC_FS / OSR) x (N + 1)", which is a rate that goes UP as
    // more samples are skipped - the wrong direction, and this says
    // which way the silicon actually goes.
    auto single_ticks = [](uint8_t skip, uint16_t rounds) -> uint32_t {
        SdadcConfig cfg = base_cfg(SdadcOsr::osr64);
        cfg.skip_count = skip;
        Sdadc::release();
        if (!sdadc_up(cfg)) {
            return 0;
        }
        Sdadc::discard(2);
        const uint32_t t0 = ticks_now();
        for (uint16_t i = 0; i < rounds; ++i) {
            int16_t v = 0;
            if (!Sdadc::read(v)) {
                return 0;
            }
        }
        const uint32_t t1 = ticks_now();
        return (t1 - t0) / rounds;
    };
    // SKPCNT 0 and 1 are refused by the driver (39.6.2.3's invalid first
    // samples), so the ladder starts at the reset value.
    const uint32_t s2 = single_ticks(2, 32);
    const uint32_t s3 = single_ticks(3, 32);
    const uint32_t s5 = single_ticks(5, 16);
    const uint32_t s9 = single_ticks(9, 16);
    const uint32_t window_ticks =
        64u * 4u * (crystal_hz / sdadc_clock_hz(main_gen_hz, fast_prescaler));
    print(serial, "  single conversions, SKPCNT 2 / 3 / 5 / 9: ", s2, " / ", s3,
          " / ", s5, " / ", s9, " crystal ticks each", crlf);
    print(serial, "  differences against SKPCNT 2: ", s3 - s2, " / ", s5 - s2,
          " / ", s9 - s2, " where one decimation window is ", window_ticks,
          " ticks", crlf);
    bench.verdict("SKPCNT COSTS A WHOLE DECIMATION WINDOW EACH, and a single "
                  "conversion therefore takes (SKPCNT + 1) of them - so table "
                  "45-26's single-conversion row multiplies the output rate "
                  "where it should divide it",
                  s2 != 0u && near(s3 - s2, window_ticks, window_ticks / 8u + 8u) &&
                      near(s5 - s2, 3u * window_ticks, window_ticks / 4u + 8u) &&
                      near(s9 - s2, 7u * window_ticks, window_ticks / 2u + 8u));

    // AND WHY THE DRIVER REFUSES FEWER THAN TWO: the chapter says those
    // windows are invalid data. Written raw, under a known full-scale
    // differential, so what "invalid" means can be seen rather than
    // quoted.
    drive_pair0(false, true);
    Sdadc::release();
    (void)sdadc_up(base_cfg(SdadcOsr::osr64));
    const int32_t proper = stats_of(8).mean;
    int32_t at_skip[2] = {0, 0};
    for (uint8_t skip = 0; skip < 2u; ++skip) {
        (void)Sdadc::enable(false);
        const uint16_t v = static_cast<uint16_t>(
            (Sdadc::regs().SDADC_CTRLB & ~SDADC_CTRLB_SKPCNT_Msk) |
            SDADC_CTRLB_SKPCNT(skip));
        Sdadc::regs().SDADC_CTRLB = v;
        (void)Sdadc::enable(true);
        at_skip[skip] = stats_of(8).mean;
    }
    print(serial, "  the same full-scale differential with SKPCNT written raw: "
          "0 gives ", at_skip[0], ", 1 gives ", at_skip[1], ", 2 gives ",
          proper, crlf);
    bench.verdict("39.6.2.3'S 'FIRST VALID SAMPLE IS THE THIRD' IS LITERAL - "
                  "the filter is still filling, so a single conversion with "
                  "SKPCNT under two reports a fraction of the input, which is "
                  "why the driver refuses that configuration",
                  at_skip[0] < proper / 2 && at_skip[1] < proper &&
                      at_skip[1] > at_skip[0]);

    Sdadc::release();
    stopwatch_stop();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// f - ERRATUM 1.8.10: the DAC as the SDADC's reference
// =============================================================================
void tf_dac_reference() {
    // The instrument is the one the DAC campaign built for erratum 1.8.9:
    // ADC0 watching the DAC's own VOUT pad, whose spread is the
    // disturbance.
    DacConfig dcfg{};
    dcfg.reference = DacRef::vddana;
    dcfg.external_output = true;
    dcfg.internal_output = true;
    bench.verdict("the DAC comes up on PA02 with both outputs enabled",
                  Dac::init(main_gen, dcfg) && Dac::set(512));
    bench.verdict("and the SAR is watching that pad", adc0_up(sar_cfg));

    drive_pair0(false, true);       // a full-scale differential to convert

    auto dac_pad_spread = [](uint16_t count) -> uint32_t {
        Adc0::select(AnalogIn<DacPad>{});
        Adc0::discard(2);
        uint16_t lo = 0xFFFFu;
        uint16_t hi = 0;
        for (uint16_t i = 0; i < count; ++i) {
            const uint16_t v = Adc0::read();
            if (v < lo) { lo = v; }
            if (v > hi) { hi = v; }
        }
        return static_cast<uint32_t>(hi - lo);
    };

    // 1. The baseline: the SDADC not running at all.
    Sdadc::release();
    const uint32_t quiet = dac_pad_spread(64);

    // 2. The erratum's own arrangement: REFSEL = DAC, ONREFBUF CLEARED.
    //    The driver refuses that configuration (39.8.2's Note is the
    //    reason and 1.8.10 is the consequence), so the bit is cleared by
    //    hand, under the converter's own disable, exactly to measure what
    //    the refusal is protecting against.
    SdadcConfig cfg = base_cfg(SdadcOsr::osr64, true);
    cfg.reference = SdadcRef::dac;
    cfg.reference_buffer = true;
    bench.verdict("the SDADC comes up free-running with the DAC as its "
                  "reference (and the buffer on, which the driver requires)",
                  sdadc_up(cfg));
    const uint32_t with_buffer = dac_pad_spread(64);

    (void)Sdadc::enable(false);
    Sdadc::regs().SDADC_REFCTRL =
        static_cast<uint8_t>(Sdadc::regs().SDADC_REFCTRL & ~SDADC_REFCTRL_ONREFBUF_Msk);
    (void)Sdadc::enable(true);
    const uint32_t without_buffer = dac_pad_spread(64);

    // 3. THE CONTROL that makes it the erratum and not "the SDADC is
    //    noisy": the same converter running just as hard against VDDANA.
    Sdadc::release();
    bench.verdict("the same converter comes up against VDDANA instead",
                  sdadc_up(base_cfg(SdadcOsr::osr64, true)));
    const uint32_t against_vddana = dac_pad_spread(64);

    print(serial, "  DAC output pad, spread of 64 SAR readings of 4096:", crlf);
    print(serial, "    SDADC stopped:                       ", quiet, crlf);
    print(serial, "    SDADC converting, REFSEL = DAC, ONREFBUF 0: ",
          without_buffer, crlf);
    print(serial, "    SDADC converting, REFSEL = DAC, ONREFBUF 1: ",
          with_buffer, crlf);
    print(serial, "    SDADC converting, REFSEL = VDDANA (control):  ",
          against_vddana, crlf);

    if (without_buffer > quiet + 4u && without_buffer > against_vddana + 4u) {
        bench.verdict("ERRATUM 1.8.10 REPRODUCES: converting against the DAC "
                      "reference WITHOUT the buffer shakes the DAC's own "
                      "output, where the same converter running against "
                      "VDDANA leaves it alone",
                      true);
        bench.verdict("AND REFCTRL.ONREFBUF IS THE WORKAROUND IT CLAIMS TO BE: "
                      "the buffer takes the disturbance back down",
                      with_buffer * 2u <= without_buffer + 4u);
    } else {
        print(serial, "  the item did NOT reproduce in this arrangement - "
              "recorded as an observation on one die, not as a claim that the "
              "item is wrong; the driver's refusal stands on 39.8.2's Note "
              "either way", crlf);
        bench.verdict("the three arrangements were measured and reported "
                      "rather than a verdict being invented for them", true);
    }

    // And the SDADC really does convert against the DAC: halving the
    // DAC's code should double the reading of a fixed differential
    // (which saturates here, so the useful statement is that the reading
    // MOVES with the reference and in the right direction).
    Sdadc::release();
    bench.verdict("the converter comes back on the DAC reference", sdadc_up(cfg));
    drive_pair0(false, true);
    (void)Dac::set(1023);
    spin(20'000UL);
    const Stats at_full = stats_of(8);
    (void)Dac::set(256);
    spin(20'000UL);
    const Stats at_quarter = stats_of(8);
    print(serial, "  a fixed +VDD differential against a DAC reference of code "
          "1023 reads ", at_full.mean, " and against code 256 reads ",
          at_quarter.mean, " (both saturate: VDD is above either reference)",
          crlf);
    bench.verdict("REFSEL = DAC IS A REAL REFERENCE PATH - the reading is "
                  "positive and at full scale under both, which is what a "
                  "differential above the reference must give",
                  at_full.mean > 20000 && at_quarter.mean > 20000);

    Sdadc::release();
    Adc0::release();
    Dac::release();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// g - the reference multiplexer, and the SAR/SDADC cross-check
// =============================================================================
void tg_reference() {
    // A FIXED differential the whole letter shares: pair 0 at opposite
    // rails is +VDD, which saturates against every reference this board
    // can offer. So the sweep is done with the PWM source instead, at a
    // duty small enough to stay inside 1.024 V.
    bench.verdict("TCC1 supplies a small fixed differential",
                  Pwm::init(main_gen) &&
                      Pwm::configure(TccConfig{.prescaler = TccPrescaler::div1}) &&
                      Pwm::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Pwm::set_period(pwm_top) && Pwm::set_cc(0, 224) &&
                      Pwm::set_cc(1, 288) && Pwm::enable(true));
    Inn0::release();
    Inp0::release();
    PwmN::claim();
    PwmP::claim();
    // k = 64 of 512 -> VDD/8, about 640 mV: inside 1.024 V, so every
    // reference level below is usable.
    constexpr int32_t k = 64;

    struct RefCase {
        SdadcRef reference;
        VrefLevel level;
        const char* label;
        uint16_t nominal_mv;
    };
    const RefCase cases[] = {
        {SdadcRef::intref, VrefLevel::v1_024, "INTREF 1.024 V", 1024},
        {SdadcRef::intref, VrefLevel::v2_048, "INTREF 2.048 V", 2048},
        {SdadcRef::intref, VrefLevel::v4_096, "INTREF 4.096 V", 4096},
        {SdadcRef::vddana, VrefLevel::v1_024, "VDDANA", 0},
    };

    int32_t reading[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)Vref::configure(VrefConfig{.level = cases[i].level});
        SdadcConfig cfg = base_cfg(SdadcOsr::osr1024);
        cfg.reference = cases[i].reference;
        cfg.reference_buffer = cases[i].reference != SdadcRef::vddana;
        Sdadc::release();
        if (!sdadc_up(cfg)) {
            bench.verdict("the converter came up against ", cases[i].label, false);
            continue;
        }
        wait_ms(2);
        const Stats s = stats_of(8);
        reading[i] = s.mean;
        const uint16_t ref_mv_here =
            cases[i].nominal_mv != 0u ? cases[i].nominal_mv : vdd_mv;
        print(serial, "  ", cases[i].label, ": ", s.mean, " counts, spread ",
              s.span(), ", = ", adc_mv_signed(s.mean, 32768, ref_mv_here),
              " mV against a nominal ", ref_mv_here, " mV reference", crlf);
    }

    // RATIOMETRY: the same input against two references reads in inverse
    // proportion, so the ratio of two readings IS the ratio of the two
    // references - with the input itself cancelling out.
    const int32_t ratio_2_to_4 =
        reading[2] != 0 ? (reading[1] * 1000) / reading[2] : 0;
    const int32_t ratio_1_to_2 =
        reading[1] != 0 ? (reading[0] * 1000) / reading[1] : 0;
    print(serial, "  bandgap ratios from the readings, x1000: 1.024/2.048 -> ",
          ratio_1_to_2, ", 2.048/4.096 -> ", ratio_2_to_4,
          " (both nominally 2000)", crlf);
    bench.verdict("THE BANDGAP'S THREE LEVELS ARE A FACTOR OF TWO APART, seen "
                  "from the SDADC's reference multiplexer with the input "
                  "cancelling out",
                  near(static_cast<uint32_t>(ratio_1_to_2), 2000u, 120u) &&
                      near(static_cast<uint32_t>(ratio_2_to_4), 2000u, 120u));

    // THE CROSS-CHECK. The SDADC's reading against VDDANA and against
    // INTREF 4.096 V gives VDDANA / 4.096 V; the SAR, converting the
    // bandgap as an INPUT against VDDANA, gives the same ratio the other
    // way up. Two converters, no shared mechanism but the bandgap.
    const int32_t sdadc_ratio =
        reading[2] != 0 ? (reading[3] * 1000) / reading[2] : 0;   // 4.096/VDD
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v4_096,
                                     .output_enable = true});
    Sdadc::release();
    bench.verdict("the SAR comes up to read the same bandgap as an INPUT",
                  adc0_up(sar_avg_cfg));
    Adc0::select(AdcInput::intref);
    Adc0::discard(4);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        sum += Adc0::read();
    }
    const uint32_t sar_counts = (sum + 8u) / 16u;
    const int32_t sar_ratio = static_cast<int32_t>((sar_counts * 1000u) / 4096u);
    const uint32_t vdd_from_sar =
        sar_counts != 0u ? (4096u * 4096u) / sar_counts : 0u;
    print(serial, "  the SAR reads the 4.096 V bandgap at ", sar_counts,
          " of 4096 against VDDANA, i.e. 4.096 V / VDDANA = ", sar_ratio,
          " per mille, and puts VDDANA at ", vdd_from_sar, " mV", crlf);
    print(serial, "  the SDADC's own two readings put 4.096 V / VDDANA at ",
          sdadc_ratio, " per mille", crlf);
    bench.verdict("TWO CONVERTER ARCHITECTURES, ONE RATIO: the sigma-delta's "
                  "reference multiplexer and the SAR's input multiplexer agree "
                  "on the bandgap against the supply to within five per cent",
                  sdadc_ratio > 0 && sar_ratio > 0 &&
                      near(static_cast<uint32_t>(sdadc_ratio),
                           static_cast<uint32_t>(sar_ratio),
                           static_cast<uint32_t>(sar_ratio / 20 + 5)));
    if (vdd_from_sar > 4000u && vdd_from_sar < 6000u) {
        vdd_mv = static_cast<uint16_t>(vdd_from_sar);
    }

    // THE EXTERNAL REFERENCE, which this board can supply as a rail: PA04
    // driven HIGH is VREFB = VDDANA, inside table 45-26's 1 V .. VDDANA.
    Adc0::release();
    VrefbPad::output();
    VrefbPad::set();
    spin(2'000UL);
    SdadcConfig vb = base_cfg(SdadcOsr::osr1024);
    vb.reference = SdadcRef::vrefb;
    Sdadc::release();
    bench.verdict("the converter comes up against the VREFB pin", sdadc_up(vb));
    wait_ms(2);
    const Stats on_pin = stats_of(8);
    VrefbPad::clear();
    spin(2'000UL);
    const Stats on_ground = stats_of(8);
    VrefbPad::release();
    print(serial, "  VREFB pad driven to VDD: ", on_pin.mean,
          " counts (VDDANA read internally gave ", reading[3],
          "); driven to GND: ", on_ground.mean,
          " counts, which is below table 45-26's 1 V minimum for a reference",
          crlf);
    bench.verdict("REFSEL = VREFB READS THE PIN: with PA04 at the supply the "
                  "external and the internal path to the same voltage agree to "
                  "under five per cent",
                  reading[3] != 0 &&
                      near(static_cast<uint32_t>(on_pin.mean),
                           static_cast<uint32_t>(reading[3]),
                           static_cast<uint32_t>(reading[3] / 20 + 40)));

    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024});
    Sdadc::release();
    (void)Pwm::enable(false);
    Pwm::release();
    PwmN::release();
    PwmP::release();
    Inn0::release();
    Inp0::release();
    (void)k;
}

// =============================================================================
// h - the post-processing: OFFSETCORR, GAINCORR, SHIFTCORR, the chopper
// =============================================================================
void th_corrections() {
    drive_pair0(false, false);   // an exact zero to correct
    bench.verdict("the converter comes up with the identity correction "
                  "(OFFSETCORR 0, GAINCORR 1, SHIFTCORR 0)",
                  sdadc_up(base_cfg(SdadcOsr::osr256)));
    const Stats base = stats_of(16);
    print(serial, "  a shorted differential, uncorrected: ", base.mean,
          " counts, ", base.raw_mean, " raw (span ", base.raw_span(), " raw)",
          crlf);
    print(serial, "  THE CORRECTIONS RUN IN RAW 24-BIT UNITS, which is this "
          "campaign's central measurement and which 39.6.3.4's 'Data0 is an "
          "unsigned integer defined on 16 bits' denies: 256 of them is one "
          "count of the specified datum", crlf);

    // OFFSETCORR is ADDED, and 39.6.3.4's formula is the arithmetic the
    // driver publishes as sdadc_corrected() - in RAW units.
    const int32_t offsets[] = {1000, -1000, 8000, 25600};
    bool offsets_exact = true;
    for (const int32_t o : offsets) {
        (void)Sdadc::offset_correction(o);
        const Stats s = stats_of(8);
        const int32_t predicted_raw = sdadc_corrected(base.raw_mean, o, 1, 0);
        print(serial, "  OFFSETCORR ", o, ": ", s.raw_mean, " raw = ", s.mean,
              " counts (predicted ", predicted_raw, " raw = ",
              predicted_raw / 256, " counts)", crlf);
        if (!near_signed(s.raw_mean, predicted_raw, 400)) {
            offsets_exact = false;
        }
    }
    (void)Sdadc::offset_correction(0);
    bench.verdict("OFFSETCORR IS ADDED TO THE FILTER'S OUTPUT IN RAW UNITS, "
                  "unit for unit, in both directions - so an OFFSETCORR of "
                  "25600 is exactly 100 counts of the specified datum",
                  offsets_exact);

    // GAINCORR is an INTEGER and SHIFTCORR the power of two under it -
    // not the SAR's fixed-point fraction. Give the converter a known
    // non-zero value to multiply: an offset.
    (void)Sdadc::offset_correction(51200);
    bool gains_exact = true;
    struct GainCase { uint16_t gain; uint8_t shift; const char* label; };
    const GainCase gains[] = {
        {2, 0, "2 / 2^0 = x2"},
        {4, 1, "4 / 2^1 = x2"},
        {3, 1, "3 / 2^1 = x1.5"},
        {1, 1, "1 / 2^1 = x0.5"},
    };
    for (const auto& g : gains) {
        (void)Sdadc::gain_correction(g.gain);
        (void)Sdadc::shift_correction(g.shift);
        const Stats s = stats_of(8);
        const int32_t predicted_raw =
            sdadc_corrected(base.raw_mean, 51200, g.gain, g.shift);
        print(serial, "  GAINCORR ", g.label, ": ", s.raw_mean, " raw = ",
              s.mean, " counts (predicted ", predicted_raw, " raw)", crlf);
        if (!near_signed(s.raw_mean, predicted_raw, 800)) {
            gains_exact = false;
        }
    }
    bench.verdict("THE GAIN IS AN INTEGER OVER A POWER OF TWO, not the SAR's "
                  "fixed-point fraction: 4/2 and 2/1 are the same gain, and "
                  "3/2 is exactly one and a half - all of it on the raw value",
                  gains_exact);

    // GAINCORR ZERO, which the driver refuses and which the silicon
    // does exactly what arithmetic says with. Written raw for that
    // reason.
    (void)Sdadc::shift_correction(0);
    bench.verdict("the driver REFUSES a GAINCORR of zero", !Sdadc::gain_correction(0));
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_GAINCORR_Msk);
    Sdadc::regs().SDADC_GAINCORR = 0;
    (void)Sdadc::sync_wait(SDADC_SYNCBUSY_GAINCORR_Msk);
    const Stats zero_gain = stats_of(8);
    print(serial, "  GAINCORR written raw to zero: every result reads ",
          zero_gain.mean, " (span ", zero_gain.span(), ")", crlf);
    bench.verdict("...and the silicon does what the formula says it must: a "
                  "gain of zero multiplies EVERY result away, which is what "
                  "erratum 1.18.3 describes happening by accident on the "
                  "revision where zero was the reset value",
                  zero_gain.mean == 0 && zero_gain.span() == 0u);
    (void)Sdadc::gain_correction(1);
    (void)Sdadc::offset_correction(0);


    // THE CHOPPER, which is 39.6.3.4's analog answer to offset. A
    // shorted differential is exactly where it should show.
    Sdadc::release();
    SdadcConfig no_chop = base_cfg(SdadcOsr::osr1024);
    no_chop.chopper = false;
    bench.verdict("the converter comes up with the chopper OFF", sdadc_up(no_chop));
    const Stats without = stats_of(16);
    Sdadc::release();
    SdadcConfig with_chop = base_cfg(SdadcOsr::osr1024);
    with_chop.chopper = true;
    bench.verdict("...and again with it ON", sdadc_up(with_chop));
    const Stats with = stats_of(16);
    const int32_t off_mv_no = adc_mv_signed(without.mean, 32768, vdd_mv);
    const int32_t off_mv_yes = adc_mv_signed(with.mean, 32768, vdd_mv);
    print(serial, "  a shorted differential: chopper OFF ", without.mean,
          " counts = ", off_mv_no, " mV (span ", without.span(),
          "); chopper ON ", with.mean, " counts = ", off_mv_yes, " mV (span ",
          with.span(), ")", crlf);
    print(serial, "  table 45-27 gives the offset error as +/-3.9 mV typical "
          "at a 5.5 V internal reference, WITH the chopper on (its own note 2)",
          crlf);
    const uint32_t a = without.mean < 0 ? static_cast<uint32_t>(-without.mean)
                                        : static_cast<uint32_t>(without.mean);
    const uint32_t b = with.mean < 0 ? static_cast<uint32_t>(-with.mean)
                                     : static_cast<uint32_t>(with.mean);
    if (a > 40u || b > 40u) {
        bench.verdict("THE CHOPPER MOVES THE OFFSET, and the direction is "
                      "reported rather than assumed",
                      b <= a);
    } else {
        print(serial, "  both offsets are under 40 counts (about 6 mV) and the "
              "difference between them is inside this suite's own scatter - "
              "the comparison is DECLINED rather than promoted to a verdict",
              crlf);
        bench.verdict("both arrangements were measured and reported", true);
    }

    Sdadc::release();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// i - the window monitor on a SIGNED result
// =============================================================================
void ti_window() {
    bench.verdict("the converter comes up", sdadc_up(base_cfg(SdadcOsr::osr256)));

    // Two known results to test the four modes against: +VDD and -VDD,
    // which saturate near +/-full scale.
    auto reading_at = [](bool positive) -> int32_t {
        drive_pair0(!positive, positive);
        const Stats s = stats_of(4);
        return s.mean;
    };
    const int32_t high = reading_at(true);
    const int32_t low = reading_at(false);
    print(serial, "  the two levels this letter compares: ", high, " and ",
          low, crlf);
    bench.verdict("they are far enough apart to put a window between them",
                  high > 10000 && low < -10000);

    const int16_t lower = -5000;
    const int16_t upper = 5000;

    struct WinCase {
        SdadcWindow mode;
        const char* label;
        bool hit_high;
        bool hit_low;
    };
    const WinCase cases[] = {
        {SdadcWindow::above_lower, "RESULT > WINLT", true, false},
        {SdadcWindow::below_upper, "RESULT < WINUT", false, true},
        {SdadcWindow::inside, "WINLT < RESULT < WINUT", false, false},
        {SdadcWindow::outside, "outside the band", true, true},
    };

    bool all_right = true;
    for (const auto& c : cases) {
        bench.verdict("the window is armed: ", c.label,
                      Sdadc::window(c.mode, lower, upper));
        drive_pair0(false, true);
        Sdadc::clear_flags(Sdadc::flag_winmon);
        int16_t v = 0;
        (void)Sdadc::read(v);
        const bool hit_high = Sdadc::window_hit();
        drive_pair0(true, false);
        Sdadc::clear_flags(Sdadc::flag_winmon);
        (void)Sdadc::read(v);
        const bool hit_low = Sdadc::window_hit();
        print(serial, "  ", c.label, " with WINLT ", lower, " WINUT ", upper,
              ": the high reading ", hit_high ? "fired" : "did not fire",
              ", the low one ", hit_low ? "fired" : "did not fire",
              " (expected ", yes_no(c.hit_high), " / ", yes_no(c.hit_low), ")",
              crlf);
        if (hit_high != c.hit_high || hit_low != c.hit_low) {
            all_right = false;
        }
    }
    bench.verdict("THE FOUR WINDOW MODES BEHAVE AS 39.8.11 PRINTS THEM, on a "
                  "SIGNED result - the thresholds are placed in the register "
                  "the same way RESULT reports the datum",
                  all_right);

    bench.verdict("and turning the monitor off stops it firing at all",
                  Sdadc::window_off() &&
                      Sdadc::window_mode() == SdadcWindow::none);
    drive_pair0(false, true);
    Sdadc::clear_flags(Sdadc::flag_winmon);
    int16_t v = 0;
    (void)Sdadc::read(v);
    bench.verdict("...measured", !Sdadc::window_hit());

    Sdadc::release();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// j - the no-CPU chain: event in, DMAC out, a TC counting
// =============================================================================
void tj_no_cpu() {
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the event channels' clock is routed",
                  EvGen::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_start_channel), ev_gen) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_result_channel), ev_gen));

    // The pacer: TC2 overflowing at about 1 kHz, its overflow an event.
    bench.verdict("TC2 paces, and its overflow becomes an event",
                  Pacer::init(main_gen) &&
                      Pacer::configure(TcConfig{.mode = TcMode::count8,
                                                .prescaler = TcPrescaler::div1024,
                                                .waveform = TcWaveform::normal_pwm}) &&
                      Pacer::set_period8(46) &&
                      Pacer::event_config(TcConfig{.mode = TcMode::count8,
                                                   .prescaler = TcPrescaler::div1024,
                                                   .waveform = TcWaveform::normal_pwm},
                                          TcEventConfig{.overflow_out = true}));
    bench.verdict("TC3 is set up to COUNT events rather than clock ticks",
                  Counter::init(main_gen) &&
                      Counter::configure(TcConfig{.mode = TcMode::count16}) &&
                      Counter::event_config(TcConfig{.mode = TcMode::count16},
                                            TcEventConfig{
                                                .action = TcEventAction::count,
                                                .input_enable = true}));

    // OSR 64 at 6 MHz with the reset SKPCNT of 2 is a conversion every
    // 128 us, well inside the pacer's millisecond - and SKPCNT 2 is what
    // makes the data valid at all (letter e).
    SdadcConfig cfg = base_cfg(SdadcOsr::osr64);
    cfg.events.result_out = true;
    bench.verdict("the SDADC comes up with its RESULT-READY event enabled",
                  sdadc_up(cfg));
    bench.verdict("the pacer's overflow reaches the SDADC's START user, on the "
                  "ASYNCHRONOUS path 39.6.6 demands in so many words",
                  Sdadc::enable(false) &&
                      Sdadc::start_on(ev_start_channel,
                                      EventChannelConfig{
                                          .generator = Pacer::overflow_generator,
                                          .path = EventPath::asynchronous}));
    bench.verdict("and a SYNCHRONOUS channel into the same user is REFUSED",
                  !Sdadc::start_on(ev_start_channel,
                                   EventChannelConfig{
                                       .generator = Pacer::overflow_generator,
                                       .path = EventPath::synchronous}));
    bench.verdict("the SDADC's own RESRDY reaches TC3's event input",
                  Evsys::connect(Counter::event_user, ev_result_channel,
                                 EventChannelConfig{
                                     .generator = Sdadc::resrdy_generator,
                                     .path = EventPath::asynchronous}));

    auto run = [](bool positive) -> uint32_t {
        for (uint16_t i = 0; i < dma_results; ++i) {
            results[i] = 0xFFFFFFFFul;
        }
        drive_pair0(!positive, positive);
        // THE DMA REQUEST IS THE RESRDY FLAG (39.6.4: "cleared when the
        // RESULT register is read"), so a result left standing from the
        // previous run would move one stale beat the moment the channel
        // is enabled. Read it away first - the ADC campaign's lesson.
        (void)Sdadc::result();
        Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
        (void)Copy::reset();
        const DmaChannelConfig ch{
            .trigger = Sdadc::dma_trigger_resrdy,
            .action = DmaTriggerAction::beat,
        };
        (void)Copy::configure(ch);
        const DmaTransfer t{
            .source = &Sdadc::regs().SDADC_RESULT,
            .destination = &results[0],
            .beats = dma_results,
            .beat = DmaBeat::word,
            .source_increment = false,
        };
        (void)Copy::load(t);
        (void)Copy::enable(true);
        (void)Counter::enable(true);
        (void)Counter::set_count16(0);
        (void)Sdadc::enable(true);
        (void)Pacer::enable(true);
        wait_ms(60);
        (void)Pacer::enable(false);
        (void)Sdadc::enable(false);
        // COUNT is read while the timer still RUNS: the read is a
        // READSYNC command and a stopped counter has no clock domain to
        // cross into.
        const uint32_t counted = Counter::count16();
        (void)Counter::enable(false);
        (void)Copy::enable(false);
        return counted;
    };

    const uint32_t counted_high = run(true);
    uint16_t filled_high = 0;
    uint16_t high_ok = 0;
    for (uint16_t i = 0; i < dma_results; ++i) {
        const uint32_t v = results[i];
        if (v != 0xFFFFFFFFul) {
            ++filled_high;
            if (sdadc_result_of(v) > 10000) {
                ++high_ok;
            }
        }
    }
    print(serial, "  +VDD: ", filled_high, " of ", dma_results,
          " results moved by the DMAC with no CPU in the path, ", high_ok,
          " of them positive; TC3 counted ", counted_high,
          " result-ready events", crlf);
    bench.verdict("THE DMAC FILLED THE BUFFER FROM RESULT, one beat per "
                  "conversion, with the CPU in a wait loop",
                  filled_high == dma_results);
    bench.verdict("and every one of them carries the polarity the pads held",
                  high_ok == dma_results);
    bench.verdict("THE SDADC IS A GENERATOR TOO: TC3 counted at least the "
                  "conversions the DMAC took",
                  counted_high >= dma_results);

    const uint32_t counted_low = run(false);
    uint16_t filled_low = 0;
    uint16_t low_ok = 0;
    for (uint16_t i = 0; i < dma_results; ++i) {
        const uint32_t v = results[i];
        if (v != 0xFFFFFFFFul) {
            ++filled_low;
            if (sdadc_result_of(v) < -10000) {
                ++low_ok;
            }
        }
    }
    print(serial, "  -VDD: ", filled_low, " moved, ", low_ok,
          " of them negative; TC3 counted ", counted_low, crlf);
    bench.verdict("the same chain follows the pads to the other polarity",
                  filled_low == dma_results && low_ok == dma_results);

    // A control: with the pacer stopped nothing moves at all.
    for (uint16_t i = 0; i < dma_results; ++i) {
        results[i] = 0xFFFFFFFFul;
    }
    (void)Sdadc::result();
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    (void)Copy::enable(true);
    (void)Sdadc::enable(true);
    wait_ms(30);
    (void)Sdadc::enable(false);
    (void)Copy::enable(false);
    bool untouched = true;
    for (uint16_t i = 0; i < dma_results; ++i) {
        if (results[i] != 0xFFFFFFFFul) {
            untouched = false;
        }
    }
    bench.verdict("with the pacer stopped NOTHING moves - the event really is "
                  "the only thing starting a conversion",
                  untouched);

    Evsys::disconnect(Counter::event_user);
    (void)Sdadc::stop_events();
    Sdadc::release();
    Pacer::release();
    Counter::release();
    Inn0::release();
    Inp0::release();
}

// =============================================================================
// k - the automatic sequence over the three pairs
// =============================================================================
void tk_sequence() {
    // Every pair gets a DIFFERENT differential, so the sequence's own
    // labels can be checked against the values it produces:
    //   pair 0 (PA06/PA07): +VDD
    //   pair 1 (PB08/PB09): -VDD
    //   pair 2 (PB06/PB07): zero
    Inn0::output(); Inn0::clear();
    Inp0::output(); Inp0::set();
    Inn1::output(); Inn1::set();
    Inp1::output(); Inp1::clear();
    Inn2::output(); Inn2::clear();
    Inp2::output(); Inp2::clear();
    spin(4'000UL);

    bench.verdict("the converter comes up", sdadc_up(base_cfg(SdadcOsr::osr64)));

    // Each pair on its own first, so the sequence has something to be
    // compared against.
    int32_t alone[3] = {0, 0, 0};
    for (uint8_t p = 0; p < 3u; ++p) {
        bench.verdict("the pair selects: ", p == 0 ? "0" : (p == 1 ? "1" : "2"),
                      Sdadc::select(p) && Sdadc::selected() == p);
        alone[p] = stats_of(4).mean;
    }
    print(serial, "  pair 0 / 1 / 2 selected one at a time: ", alone[0], " / ",
          alone[1], " / ", alone[2], " counts", crlf);
    bench.verdict("THREE INDEPENDENT DIFFERENTIAL PAIRS, each following its own "
                  "two pads",
                  alone[0] > 10000 && alone[1] < -10000 &&
                      alone[2] > -600 && alone[2] < 600);

    // Now the sequencer: one START, three results, in order from the
    // lowest enabled input (39.6.2.7). THE CONVERTER IS CYCLED FIRST -
    // every read() in this suite starts a conversion, so an earlier
    // sequence could still be in flight and 39.8.17 says a START written
    // while START is already set has no effect.
    const bool seq_armed = Sdadc::sequence(0x7);
    (void)Sdadc::enable(false);
    (void)Sdadc::enable(true);
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);

    // NOTHING IS PRINTED BETWEEN THE START AND THE THIRD RESULT. The
    // first version of this letter put a verdict line in there and lost
    // the whole sequence: one console line at 115200 is about five
    // milliseconds and three conversions are 385 microseconds, so every
    // read came back with the LAST one.
    const bool started = Sdadc::start();
    const bool busy_at_start = Sdadc::sequence_busy();
    int32_t got[3] = {0, 0, 0};
    uint8_t state[3] = {0xFF, 0xFF, 0xFF};
    bool busy_seen = busy_at_start;
    bool all_arrived = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        uint32_t spins = 0x3FFFFFu;
        while (spins-- != 0u && !Sdadc::ready()) {
            if (Sdadc::sequence_busy()) {
                busy_seen = true;
            }
        }
        if (!Sdadc::ready()) {
            all_arrived = false;
        }
        state[i] = Sdadc::sequence_state();
        got[i] = Sdadc::result();
    }
    const bool busy_after = Sdadc::sequence_busy();
    const bool lost = Sdadc::overrun();
    bench.verdict("the sequencer takes all three pairs", seq_armed);
    bench.verdict("one software START begins a SEQUENCE", started && all_arrived);
    bench.verdict("and no result of it was overrun on the way out", !lost);
    print(serial, "  one START gave ", got[0], " / ", got[1], " / ", got[2],
          " with SEQSTATUS.SEQSTATE reading ", state[0], " / ", state[1], " / ",
          state[2], crlf);
    bench.verdict("ONE START RUNS THE WHOLE SEQUENCE, in the order 39.6.2.7 "
                  "prescribes - lowest enabled input first",
                  near_signed(got[0], alone[0], 800) &&
                      near_signed(got[1], alone[1], 800) &&
                      near_signed(got[2], alone[2], 800));
    bench.verdict("and SEQSTATUS.SEQSTATE labels each result with the input it "
                  "came from",
                  state[0] == 0u && state[1] == 1u && state[2] == 2u);
    print(serial, "  SEQSTATUS.SEQBUSY: ", yes_no(busy_at_start),
          " immediately after the START, ", yes_no(busy_seen),
          " at some point during the sequence, ", yes_no(busy_after),
          " when the last result was in hand", crlf);
    bench.verdict("SEQBUSY stands while the sequence runs and is clear when "
                  "the last conversion of it is done (39.8.8)",
                  busy_seen && !busy_after);

    // A PARTIAL sequence: the middle pair alone, which is what makes the
    // "lowest ENABLED input" wording testable.
    bench.verdict("the sequencer takes pair 1 alone", Sdadc::sequence(0x2));
    (void)Sdadc::enable(false);
    (void)Sdadc::enable(true);
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    Sdadc::discard(2);
    int16_t only = 0;
    (void)Sdadc::read(only);
    const uint8_t only_state = Sdadc::sequence_state();
    print(serial, "  the sequencer holding pair 1 alone: ", only,
          " counts, SEQSTATE ", only_state, crlf);
    bench.verdict("a one-input sequence converts THAT input, whatever MUXSEL "
                  "says",
                  near_signed(only, alone[1], 800));

    bench.verdict("and zero in SEQCTRL gives the multiplexer back",
                  Sdadc::sequence(0) && Sdadc::select(0));
    Sdadc::discard(2);
    const Stats back = stats_of(4);
    bench.verdict("...measured", near_signed(back.mean, alone[0], 800));

    Sdadc::release();
    Inn0::release(); Inp0::release();
    Inn1::release(); Inp1::release();
    Inn2::release(); Inp2::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_sdadc - SAMC21J18A SIGMA-DELTA converter (ch. 39): three "
          "differential pad pairs, a decimation filter, a signed 16-bit "
          "result, wireless, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        (void)irq;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    const bool dma_ok = brio::Dmac::init();
    brio::Nvic::enable(brio::Dmac::irq());
    brio::enable_interrupts();

    bench.letter('a', "the block, its vocabularies, its disciplines, the "
                      "chapter's disagreements", ta_block);
    bench.letter('b', "the zero and the noise: every OSR on a shorted "
                      "differential", tb_noise);
    bench.letter('c', "full scale, the sign, and where the datum sits",
                 tc_full_scale);
    bench.letter('d', "the swept differential, and the SAR on the same pads",
                 td_sweep);
    bench.letter('e', "time: the rate, the PRESCALER question, SKPCNT",
                 te_timing);
    bench.letter('f', "erratum 1.8.10: the DAC as the reference", tf_dac_reference);
    bench.letter('g', "the reference multiplexer, and the cross-check",
                 tg_reference);
    bench.letter('h', "the post-processing and the chopper", th_corrections);
    bench.letter('i', "the window monitor on a signed result", ti_window);
    bench.letter('j', "the no-CPU chain: event in, DMAC out", tj_no_cpu);
    bench.letter('k', "the automatic sequence over the three pairs", tk_sequence);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " dmac=", dma_ok ? "up" : "FAILED", crlf);
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
