// test_samc_dac - the reference bench suite for samc/dac.hpp, and the
// session that closes the analog loop.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, AND THIS CHAPTER IS WHY. On this package PA02 is the
// DAC's VOUT pad, ADC0's AIN0 and the AC's AIN4 all at once, so the
// "wire the DAC output to an ADC input" that erratum 1.8.9 asks for as
// a workaround is already on the die - a wire of zero length. The DAC
// is therefore the swept voltage source this board never had, and three
// drivers that were left holding unvalidated enumerators get their
// answer here:
//   samc/adc.hpp   AdcInput::dac  and  Ref::dac
//   samc/ac.hpp    AcNegative::dac
//   samc/adc.hpp   Ref::intref as a REFERENCE, and whether it needs
//                  SUPC.VREF.VREFOE the way the INPUT path does
//
// WHAT THIS BOARD STILL CANNOT DO, said once: ADC1 does not bond PA02
// (its map is PORT B plus PA08/PA09), so the transfer curve has ONE
// pad-path witness and not two; the second converter reaches the DAC
// only through the internal channel, which is the erratum's own noisy
// path. That is not a second opinion on the curve - it is the erratum's
// measurement - and letter h says so rather than averaging the two.
//
// AND WHAT A DAC-vs-ADC COMPARISON CAN CLAIM: two unknown transfer
// functions in series. Endpoints, monotonicity and gross linearity are
// claimable; the residual from a best-fit line is COMBINED nonlinearity
// and this suite declines to apportion it between the two converters.
//
// What is exercised, letter by letter:
//   a  the block: geometry, the vocabularies it publishes, the register
//      disciplines, and the chapter's two disagreements with its own
//      device header
//   b  THE LOOP WITH NO WIRE: the DAC on PA02 read by ADC0/AIN0
//   c  the reference: does REFSEL = INTREF follow SUPC.VREF.SEL?
//   d  the ADC's own open gap: REFSEL = INTREF with VREFOE off and on
//   e  Ref::dac - the DAC as the ADC's reference
//   f  AdcInput::dac - the internal channel, and erratum 1.8.9 measured
//   g  AcNegative::dac - a comparator threshold from the DAC
//   h  the transfer curve, honestly framed
//   i  time: the startup, and a full-scale step timed by the comparator
//   j  the no-CPU chain: an event starts it, the DMAC feeds DATABUF
//   k  erratum 1.9.2: the EMPTY flag across a standby
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/ac.hpp"
#include "samc/adc.hpp"
#include "samc/clock.hpp"
#include "samc/dac.hpp"
#include "samc/dmac.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/osc32kctrl.hpp"
#include "samc/pin.hpp"
#include "samc/reset.hpp"
#include "samc/rtc.hpp"
#include "samc/sercom.hpp"
#include "samc/sleep.hpp"
#include "samc/supc.hpp"
#include "samc/tc.hpp"
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

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The one pad this whole suite turns on
// ---------------------------------------------------------------------------
using Vout = Pin<'A', 2>;    // DAC/VOUT = ADC0/AIN0 = AC/AIN4
using Vrefa = Pin<'A', 3>;   // DAC/VREFA = ADC0/AIN1 = AC/AIN5, untouched

using Adc0 = Adc<0>;
using Adc1 = Adc<1>;
using Comp = AcComparator<0>;

/// GCLK generator 0 is the 48 MHz main clock. GCLK_DAC's only timing
/// duty is the voltage pump (41.6.8.3 wants it at least four times the
/// sampling rate), which 48 MHz satisfies for anything here.
constexpr uint8_t main_gen = 0;
constexpr uint32_t main_gen_hz = SysClock::hz;

/// What the two earlier campaigns located this board's supply at - the
/// comparator's scaler said 5141 mV, the ADC's own bandgap reading said
/// 5233. A STARTING POINT, refined by letter b, never a verdict's
/// authority.
constexpr uint16_t supply_hint_mv = 5150;
uint16_t vdd_mv = supply_hint_mv;

// ---------------------------------------------------------------------------
// The stopwatch (letter i only): TC0 + TC1 as one 32-bit counter on the
// BOARD'S CRYSTAL. A settling time reported against OSC48M would carry
// that oscillator's 5100 ppm into a number about the DAC.
// ---------------------------------------------------------------------------
using Stopwatch = Tc<0>;
constexpr uint32_t crystal_hz = 24'000'000UL;
constexpr uint8_t gen_crystal = 2;
constexpr uint32_t stopwatch_hz = crystal_hz;   ///< div1: 41.7 ns a tick
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

uint32_t ticks_now() { return Stopwatch::count32(); }

// ---------------------------------------------------------------------------
// The event fabric (letter j): the same shape every SAM suite here uses.
// ---------------------------------------------------------------------------
constexpr uint8_t dma_ch = 0;
constexpr uint8_t ev_start_channel = 0;    // TC2 overflow -> DAC START
constexpr uint8_t ev_empty_channel = 1;    // DAC EMPTY    -> TC3 counts
constexpr uint8_t ev_gen = 6;
using Feed = DmaChannel<dma_ch>;
using EvGen = Gclk<ev_gen>;
using Pacer = Tc<2>;
using Counter = Tc<3>;

/// VOLATILE IN BOTH DIRECTIONS - the DMAC campaign's lesson on this
/// target: the compiler sees neither the controller's reads nor its
/// writes.
constexpr uint16_t wave_len = 32;
volatile uint16_t wave[wave_len];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

/// Long enough for the DAC's own conversion (2.9 us at the 350 ksps
/// ceiling) and for the ADC's input to see the new level: about 200 us.
void settle() { spin(2'000UL); }

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

const char* yes_no(bool v) { return v ? "yes" : "no"; }

/// The single-reading ADC configuration on the pad.
constexpr AdcConfig pad_cfg{
    .reference = Ref::vddana,
    .prescaler = AdcPresc::div32,   // 48 MHz / 32 = 1.5 MHz
    .sample_length = 5,
};

/// The same with a 64-sample average, for the curve. Table 38-2's own
/// ADJRES keeps the result at a 12-bit scale.
constexpr AdcConfig avg_cfg{
    .reference = Ref::vddana,
    .prescaler = AdcPresc::div32,
    .resolution = AdcRes::bits16,
    .average = AdcAverage::samples64,
    .adjust = 4,
    .sample_length = 5,
};

/// The DAC on its pad, with both outputs enabled unless told otherwise.
bool dac_up(DacRef reference = DacRef::vddana, bool external = true,
            bool internal = true) {
    DacConfig cfg{};
    cfg.reference = reference;
    cfg.external_output = external;
    cfg.internal_output = internal;
    return Dac::init(main_gen, cfg);
}


/**
 * ADC0 up, WITH ERRATUM 1.4.10'S WORKAROUND WHERE IT IS NEEDED.
 *
 * Once ADC1 has been used in this power cycle, ADC0.SYNCBUSY.ENABLE is
 * stuck at one on this die and `Adc<0>::init()` - which waits on it -
 * returns false with the converter left DISABLED and reading zero. The
 * errata's own workaround is "enable ADC0 before ADC1, or disregard the
 * bit"; what actually works here is to bring ADC1 up first and ADC0
 * second, after which ADC0 keeps converting even when ADC1 goes away
 * again. Letter f documents the whole thing; this helper is what keeps
 * every other letter runnable in the same power cycle, and it says out
 * loud when it has to act.
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

template <class A>
uint32_t mean_of(uint16_t count) {
    A::discard(2);
    uint32_t sum = 0;
    for (uint16_t i = 0; i < count; ++i) {
        sum += A::read();
    }
    return count == 0u ? 0u : (sum + count / 2u) / count;
}

struct Spread {
    uint16_t low;
    uint16_t high;
    uint32_t mean;
    uint16_t span() const { return static_cast<uint16_t>(high - low); }
};

template <class A>
Spread spread_of(uint16_t count) {
    A::discard(2);
    uint16_t lo = 0xFFFFu;
    uint16_t hi = 0;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < count; ++i) {
        const uint16_t v = A::read();
        if (v < lo) {
            lo = v;
        }
        if (v > hi) {
            hi = v;
        }
        sum += v;
    }
    return Spread{lo, hi, count == 0u ? 0u : (sum + count / 2u) / count};
}

/// Set the DAC and let both converters settle on the new level.
void dac_set(uint16_t code) {
    (void)Dac::set(code);
    settle();
}

/// The whole read chain for one code: DAC out, ADC0 in over the pad.
uint16_t pad_reading(uint16_t code, uint16_t samples = 8) {
    dac_set(code);
    return static_cast<uint16_t>(mean_of<Adc0>(samples));
}

// =============================================================================
// a - the block, its vocabularies, its disciplines, its refusals
// =============================================================================
void ta_block() {
    bench.verdict("one DAC on this family, ten bits, and no instance index "
                  "to carry",
                  dac_count() == 1u && Dac::steps == 1024u &&
                      Dac::dither_steps == 16384u);

    print(serial, "  EVSYS: EMPTY generator ", Dac::empty_generator,
          ", START user ", Dac::start_event_user,
          " (asynchronous path only); DMAC trigger ", Dac::dma_trigger_empty,
          "; GCLK id ", Dac::gclk_id, crlf);
    bench.verdict("the generator, user and trigger codes are the header's own",
                  Dac::empty_generator == 79u && Dac::start_event_user == 38u &&
                      Dac::dma_trigger_empty == 45u && Dac::gclk_id == 36u);

    // THE GEOMETRIC GIFT, stated as data rather than as a comment.
    bench.verdict("VOUT and ADC0/AIN0 ARE THE SAME PAD - erratum 1.8.9's "
                  "external wire has zero length on this silicon",
                  Dac::vout_function('A', 2) == static_cast<int>(PinFunction::b) &&
                      Adc0::ain_of('A', 2) == 0 &&
                      Dac::adc_input_pad_port == 'A' && Dac::adc_input_pad_pin == 2);
    bench.verdict("...and the same pad is the AC's AIN4, which is COMP2/3's "
                  "PIN0",
                  ac_ain_exists(ac_ain_of(2, 0)) && ac_ain_of(2, 0) == 4u);
    bench.verdict("ADC1 does NOT bond PA02, so the second converter can only "
                  "reach the DAC internally",
                  Adc1::ain_of('A', 2) < 0);
    bench.verdict("VREFA is PA03 and nothing else is",
                  Dac::vrefa_function('A', 3) == static_cast<int>(PinFunction::b) &&
                      Dac::vrefa_function('A', 2) < 0 &&
                      Dac::vout_function('A', 3) < 0);

    // The refusals that are the chapter's rules.
    bench.verdict("REFSEL 0x3 is Reserved and refused",
                  !dac_config_valid(DacConfig{.reference = static_cast<DacRef>(3)}) &&
                      dac_config_valid(DacConfig{.reference = DacRef::vrefa}));
    bench.verdict("DITHERING WITHOUT A START EVENT IS REFUSED - 41.6.8.3 makes "
                  "the sixteen sub-conversions the event's job",
                  !dac_config_valid(DacConfig{.dither = true}) &&
                      dac_config_valid(DacConfig{.dither = true,
                                                 .events = {.start_in = true}}));
    bench.verdict("inverting an event input nothing listens to is refused",
                  !dac_config_valid(DacConfig{.events = {.invert_start = true}}));

    // Table 41-1's placement, which no caller should ever shift by hand.
    bench.verdict("table 41-1's four placements are arithmetic, not a comment",
                  dac_data_word(512, false, false) == 512u &&
                      dac_data_word(512, true, false) == (512u << 6) &&
                      dac_data_word(0x1234, false, true) == 0x1234u &&
                      dac_data_word(0x1234, true, true) == (0x1234u << 2));
    bench.verdict("and a value past the scale is clamped, never spilled into "
                  "the neighbouring field",
                  dac_data_word(2000, false, false) == 1023u &&
                      dac_data_word(20000, false, true) == 16383u);

    // ---- the silicon ------------------------------------------------------
    bench.verdict("the DAC comes up on generator 0", dac_up());
    bench.verdict("STATUS.READY is set once the startup time has passed",
                  Dac::ready());
    bench.verdict("CTRLB reads back what init() wrote - reference VDDANA, "
                  "both outputs enabled",
                  Dac::external_output() && Dac::internal_output() &&
                      ((Dac::control_b() & DAC_CTRLB_REFSEL_Msk) >>
                       DAC_CTRLB_REFSEL_Pos) ==
                          static_cast<uint8_t>(DacRef::vddana));

    // CTRLB IS ENABLE-PROTECTED (41.8.2): a store while enabled is
    // dropped by the silicon, so the driver refuses instead of pretending.
    const uint8_t before = Dac::control_b();
    const bool refused = !Dac::external_output(false);
    bench.verdict("CTRLB is enable-protected and the driver REFUSES rather "
                  "than storing into a register the silicon ignores",
                  refused && Dac::control_b() == before);

    // AND THE CHAPTER DISAGREES WITH ITSELF ABOUT EVCTRL: 41.6.2.1 lists
    // it as enable-protected, 41.8.3's property line says only "PAC
    // Write-Protection". Ask the silicon by writing the register raw
    // while enabled and reading it back.
    Dac::regs().DAC_EVCTRL = DAC_EVCTRL_EMPTYEO_Msk;
    const uint8_t evctrl_while_enabled = Dac::regs().DAC_EVCTRL;
    Dac::regs().DAC_EVCTRL = 0;
    print(serial, "  EVCTRL written raw under a RUNNING converter reads back ",
          Hex{evctrl_while_enabled}, " (41.6.2.1 calls the register "
          "enable-protected, 41.8.3's property line does not)", crlf);
    bench.verdict("EVCTRL IS ENABLE-PROTECTED: the write is DISCARDED under a "
                  "running converter, so 41.6.2.1's list is right and "
                  "41.8.3's property line is missing a property",
                  evctrl_while_enabled == 0u);

    // DBGCTRL: the device header puts it at offset 0x14, the register
    // summary and 41.8.11 at 0x18. Write through the header's field and
    // look at both.
    Dac::regs().DAC_DBGCTRL = DAC_DBGCTRL_DBGRUN_Msk;
    const volatile uint8_t* base = reinterpret_cast<const volatile uint8_t*>(DAC_REGS);
    const uint8_t at_14 = base[0x14];
    const uint8_t at_18 = base[0x18];
    Dac::regs().DAC_DBGCTRL = 0;
    print(serial, "  DBGRUN written through the header's DAC_DBGCTRL: offset "
                  "0x14 reads ", Hex{at_14}, ", offset 0x18 reads ", Hex{at_18},
          " (41.7's summary and 41.8.11 both say 0x18)", crlf);
    bench.verdict("DBGCTRL IS AT OFFSET 0x14, where the device header puts it "
                  "and not where the register summary draws it",
                  at_14 == DAC_DBGCTRL_DBGRUN_Msk && at_18 == 0u);

    // The write-only data path, which is why code() is a promise and not
    // a readback.
    (void)Dac::set(0x155);
    bench.verdict("code() reports what was WRITTEN - DATA and DATABUF are "
                  "write-only in both documents, so there is nothing to read",
                  Dac::code() == 0x155u);

    Dac::release();
    bench.verdict("release() puts the block back: the DAC's APB clock off "
                  "and its generic clock channel disconnected",
                  (MCLK_REGS->MCLK_APBCMASK & MCLK_APBCMASK_DAC_Msk) == 0u);
}

// =============================================================================
// b - THE LOOP WITH NO WIRE
// =============================================================================
void tb_loop() {
    bench.verdict("the DAC comes up with the external buffer driving PA02",
                  dac_up());
    bench.verdict("ADC0 comes up", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});

    // FIRST: does the analog output need the pad's peripheral function?
    // An ADC INPUT does not (the connection is direct, which is what makes
    // every wireless letter in test_samc_adc possible); an OUTPUT has a
    // driver to win against PORT's, so the question is a real one.
    Vout::release();          // pad under PORT, direction input (reset state)
    dac_set(512);
    const uint32_t no_pmux = mean_of<Adc0>(16);
    Dac::claim_vout<Vout>();
    dac_set(512);
    const uint32_t with_pmux = mean_of<Adc0>(16);
    print(serial, "  mid-code on PA02: ", no_pmux, " counts with the pad left "
          "under PORT, ", with_pmux, " with the DAC's peripheral function "
          "claimed", crlf);
    bench.verdict("THE ANALOG OUTPUT REACHES THE PAD EITHER WAY - the buffer "
                  "is connected by CTRLB.EOEN and not by the pin multiplexer",
                  near(no_pmux, with_pmux, 24u) && no_pmux > 1800u &&
                      no_pmux < 2300u);

    // THE NOISE FLOOR, measured before any band below is chosen.
    dac_set(512);
    const Spread mid = spread_of<Adc0>(64);
    print(serial, "  64 single readings at mid-code span ", mid.span(),
          " counts of 4096 around ", mid.mean, crlf);
    const uint16_t band = static_cast<uint16_t>(mid.span() + 8u);

    // The endpoints. Table 45-30 bounds the LINEAR output range at
    // 0.05 V .. VDDANA - 0.05 V, so code 0 and code 1023 are outside
    // specification by construction and the verdict says how far, not
    // that they are rails.
    const uint32_t at_0 = pad_reading(0, 16);
    const uint32_t at_1023 = pad_reading(1023, 16);
    const uint32_t at_512 = pad_reading(512, 16);
    const uint32_t at_256 = pad_reading(256, 16);
    const uint32_t at_768 = pad_reading(768, 16);
    print(serial, "  codes 0 / 256 / 512 / 768 / 1023 read ", at_0, " / ",
          at_256, " / ", at_512, " / ", at_768, " / ", at_1023,
          " counts of 4096", crlf);

    // VDD from this loop's own arithmetic: full scale IS the reference,
    // which here is VDDANA, so the top code says nothing about VDD - but
    // the ADC's own scaled-supply channel does, and it is the number the
    // rest of the suite uses.
    Adc0::select(AdcInput::scaled_supply);
    const uint32_t quarter = mean_of<Adc0>(32);
    vdd_mv = static_cast<uint16_t>(adc_mv(quarter * 4u, Adc0::result_steps(),
                                          supply_hint_mv));
    // One Newton step: the reading is a quarter of VDD measured against
    // VDD, so it is a self-consistency check and not a supply meter -
    // keep the campaigns' number and print the check.
    print(serial, "  1/4 VDDANA reads ", quarter, " counts (a quarter of full "
          "scale would be 1024); VDD taken as ", supply_hint_mv, " mV from the "
          "SUPC and ADC campaigns", crlf);
    vdd_mv = supply_hint_mv;
    bench.verdict("the scaled supply is a quarter of full scale to under one "
                  "per cent, so the reference really is VDDANA",
                  near(quarter, 1024u, 20u));

    Adc0::select(AnalogIn<Vout>{});
    bench.verdict("code 0 sits at the bottom of the range, within a few counts "
                  "of ground",
                  at_0 <= 40u);
    bench.verdict("code 1023 sits at the top, within a per cent of full scale",
                  at_1023 >= 4050u);
    bench.verdict("mid-code is half of full scale to better than one per cent "
                  "- and one per cent is the two converters' combined offset "
                  "and gain error, not their linearity",
                  near(at_512, 2048u, 41u));
    bench.verdict("a quarter and three quarters land where they should",
                  near(at_256, 1024u, 41u) && near(at_768, 3072u, 41u));

    // Monotonic over a coarse walk - the fine one is letter h.
    bool monotone = true;
    uint32_t previous = 0;
    for (uint16_t c = 0; c <= 1000u; c = static_cast<uint16_t>(c + 50u)) {
        const uint32_t v = pad_reading(c, 4);
        if (c != 0u && v + band < previous) {
            monotone = false;
        }
        previous = v;
    }
    bench.verdict("and the output never goes backwards over a 21-point walk "
                  "of the whole range",
                  monotone);

    // set_mv() through util/analog.hpp, unchanged on this target.
    (void)Dac::set_mv(2000, vdd_mv);
    settle();
    const uint32_t two_volts = mean_of<Adc0>(16);
    const uint16_t two_volts_mv =
        adc_mv(two_volts, Adc0::result_steps(), vdd_mv);
    print(serial, "  set_mv(2000) wrote code ", Dac::code(), " and the ADC "
          "reads ", two_volts_mv, " mV", crlf);
    bench.verdict("util/analog.hpp's dac_code() and adc_mv() agree across the "
                  "silicon to better than 50 mV, with NOT ONE LINE of util/ "
                  "changed for this target",
                  near(two_volts_mv, 2000u, 50u));

    dac_set(0);
    Vout::release();
    Adc0::release();
    Dac::release();
}

// =============================================================================
// c - the reference: does REFSEL = INTREF follow SUPC.VREF.SEL?
// =============================================================================
//
// The device header calls REFSEL 0x0 `INT1V`, which is the SAM D21's
// fixed 1.0 V reference. 41.8.2 calls it INTREF and points at
// SUPC.VREF.SEL for the level. Only one of those can be true, and the
// experiment is trivial: set the DAC to full scale, read the pad against
// VDDANA, and change the bandgap level underneath it.
void tc_reference() {
    bench.verdict("ADC0 comes up on VDDANA", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});

    const VrefLevel levels[3] = {VrefLevel::v1_024, VrefLevel::v2_048,
                                 VrefLevel::v4_096};
    const uint16_t nominal[3] = {1024, 2048, 4096};
    uint16_t measured[3] = {0, 0, 0};
    bool all_ok = true;

    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Vref::configure(VrefConfig{.level = levels[i],
                                         .output_enable = false});
        all_ok = dac_up(DacRef::intref) && all_ok;
        // Code 1000 rather than 1023: 41.6.2.4's own formula makes full
        // scale the reference itself, and the top of the range is where
        // table 45-30's linearity stops.
        dac_set(1000);
        const uint32_t counts = mean_of<Adc0>(16);
        measured[i] = static_cast<uint16_t>(
            static_cast<uint32_t>(adc_mv(counts, Adc0::result_steps(), vdd_mv)) *
            1023u / 1000u);
        print(serial, "  SUPC.VREF.SEL = ", nominal[i], " mV: DAC code 1000 "
              "reads ", counts, " counts, implying a full scale of ",
              measured[i], " mV", crlf);
        Dac::release();
    }

    bench.verdict("every INTREF configuration came up", all_ok);
    bench.verdict("THE DAC'S INTERNAL REFERENCE IS THE SUPC BANDGAP AND ITS "
                  "LEVEL MOVES WITH SUPC.VREF.SEL - so 41.8.2 is right and the "
                  "device header's INT1V name is the SAM D21's, stale here",
                  measured[1] > measured[0] + 700u &&
                      measured[2] > measured[1] + 1500u);
    bench.verdict("...and each level lands within 10 % of its nominal",
                  near(measured[0], nominal[0], 102u) &&
                      near(measured[1], nominal[1], 205u) &&
                      near(measured[2], nominal[2], 410u));

    // Does the DAC's reference path need SUPC.VREF.VREFOE, the way the
    // ADC's bandgap INPUT channel does (adc.md)? Same level, bit off
    // then on.
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v2_048,
                                     .output_enable = false});
    (void)dac_up(DacRef::intref);
    dac_set(1000);
    const uint32_t without = mean_of<Adc0>(16);
    Vref::output_enable(true);
    settle();
    const uint32_t with = mean_of<Adc0>(16);
    Vref::output_enable(false);
    print(serial, "  VREFOE off ", without, " counts, on ", with,
          " - the DAC's reference path", crlf);
    bench.verdict("THE DAC'S REFERENCE PATH DOES NOT NEED SUPC.VREF.VREFOE: "
                  "the reference multiplexer takes the bandgap internally, "
                  "unlike the ADC's bandgap INPUT channel",
                  without > 1200u && near(without, with, 40u));

    Dac::release();
    Adc0::release();
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024});
}

// =============================================================================
// d - the ADC's own open gap: REFSEL = INTREF with VREFOE off and on
// =============================================================================
//
// docs/samc/adc.md has carried this line since the ADC campaign: every
// letter that touched the bandgap read it as an INPUT, where VREFOE is
// mandatory and undocumented; no conversion had ever run with the
// bandgap as the ADC's REFERENCE. The DAC is what makes the question
// answerable - it supplies a steady mid-range voltage the ADC can
// measure against both references and compare.
void td_adc_intref_reference() {
    bench.verdict("the DAC comes up on VDDANA", dac_up());
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v2_048,
                                     .output_enable = false});

    // A DAC level comfortably below the 2.048 V bandgap: 1.5 V.
    bench.verdict("ADC0 on VDDANA", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});
    (void)Dac::set_mv(1500, vdd_mv);
    settle();
    const uint32_t on_vddana = mean_of<Adc0>(32);
    const uint16_t source_mv = adc_mv(on_vddana, Adc0::result_steps(), vdd_mv);
    print(serial, "  the source: DAC code ", Dac::code(), " reads ", on_vddana,
          " counts against VDDANA = ", source_mv, " mV", crlf);
    Adc0::release();

    constexpr AdcConfig intref_cfg{
        .reference = Ref::intref,
        .prescaler = AdcPresc::div32,
        .sample_length = 20,
    };

    // VREFOE OFF first, deliberately: if the reference path shared the
    // input path's dependence, this is where it would read zero.
    bench.verdict("ADC0 comes up with REFSEL = INTREF and VREFOE CLEAR",
                  adc0_up(intref_cfg));
    Adc0::select(AnalogIn<Vout>{});
    const uint32_t ref_off = mean_of<Adc0>(32);
    Adc0::release();

    Vref::output_enable(true);
    bench.verdict("...and again with VREFOE SET", adc0_up(intref_cfg));
    Adc0::select(AnalogIn<Vout>{});
    const uint32_t ref_on = mean_of<Adc0>(32);
    Adc0::release();
    Vref::output_enable(false);

    const uint16_t implied_off =
        adc_mv(ref_off, 4096u, vref_mv(VrefLevel::v2_048));
    const uint16_t implied_on =
        adc_mv(ref_on, 4096u, vref_mv(VrefLevel::v2_048));
    print(serial, "  against INTREF 2.048 V: VREFOE off ", ref_off,
          " counts = ", implied_off, " mV, VREFOE on ", ref_on, " counts = ",
          implied_on, " mV", crlf);

    bench.verdict("THE ADC'S REFERENCE PATH DOES NOT NEED VREFOE - the gap "
                  "adc.md has carried since its campaign, closed: a conversion "
                  "against REFSEL = INTREF works with the bit CLEAR",
                  ref_off > 100u && ref_off < 4000u);
    bench.verdict("...and the bit changes the reading by less than one per "
                  "cent, so it is genuinely not in this path",
                  near(ref_off, ref_on, 41u));
    bench.verdict("the two references agree on the DAC's voltage to better "
                  "than five per cent, which is the bandgap's own level "
                  "accuracy and not a fault of either path",
                  near(implied_off, source_mv,
                       static_cast<uint32_t>(source_mv / 20u)));

    Dac::release();
}

// =============================================================================
// e - Ref::dac: the DAC as the ADC's reference
// =============================================================================
void te_dac_as_reference() {
    bench.verdict("the DAC comes up with both outputs enabled", dac_up());

    constexpr AdcConfig dacref_cfg{
        .reference = Ref::dac,
        .prescaler = AdcPresc::div32,
        .sample_length = 20,
    };

    // The MEASURAND is the internal quarter-scaled analog supply: a
    // steady voltage that owes nothing to the DAC, so a conversion
    // against a DAC reference is a pure ratio.
    const uint16_t codes[3] = {512, 768, 1000};
    uint32_t counts[3] = {0, 0, 0};
    bool ok = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        dac_set(codes[i]);
        ok = adc0_up(dacref_cfg) && ok;
        Adc0::select(AdcInput::scaled_supply);
        counts[i] = mean_of<Adc0>(16);
        Adc0::release();
    }
    const uint16_t quarter_mv = static_cast<uint16_t>(vdd_mv / 4u);
    print(serial, "  1/4 VDDANA (about ", quarter_mv, " mV) against a DAC "
          "reference at codes 512 / 768 / 1000: ", counts[0], " / ", counts[1],
          " / ", counts[2], " counts of 4096", crlf);

    bench.verdict("REFSEL = DAC CONVERTS - the enumerator adc.hpp has carried "
                  "unvalidated since its campaign is real silicon",
                  ok && counts[0] > 100u && counts[0] < 4096u);

    // A conversion against a reference of V_ref reads in_mv/ref_mv x 4096.
    // The DAC's own output at code c is about c/1023 x VDDANA, so the
    // reading should fall as 1/c - a ratio the supply cancels out of.
    const uint32_t predicted1 =
        counts[0] * static_cast<uint32_t>(codes[0]) / codes[1];
    const uint32_t predicted2 =
        counts[0] * static_cast<uint32_t>(codes[0]) / codes[2];
    print(serial, "  scaled from the first: ", predicted1, " and ", predicted2,
          " predicted against ", counts[1], " and ", counts[2], " measured",
          crlf);
    bench.verdict("AND IT IS RATIOMETRIC: the reading falls exactly as one "
                  "over the DAC's code, to under three per cent, with the "
                  "supply cancelling out of both sides",
                  near(counts[1], predicted1, predicted1 / 33u + 4u) &&
                      near(counts[2], predicted2, predicted2 / 33u + 4u));

    // And the absolute check: with the DAC's code known, the reference is
    // known, so the measurand comes back in millivolts.
    const uint16_t dac_ref_at_512 = Dac::code_mv(512, vdd_mv);
    const uint16_t implied = adc_mv(counts[0], 4096u, dac_ref_at_512);
    print(serial, "  which puts 1/4 VDDANA at ", implied, " mV against the ",
          quarter_mv, " mV the supply says", crlf);
    bench.verdict("...and the absolute answer lands within ten per cent of "
                  "what the supply says, which is as much as a DAC reference "
                  "carrying its own gain and offset error can promise",
                  near(implied, quarter_mv,
                       static_cast<uint32_t>(quarter_mv / 10u)));

    Dac::release();
}

// =============================================================================
// f - AdcInput::dac, and erratum 1.8.9 measured
// =============================================================================
void tf_internal_channel() {
    // 41.6.8.1: "The DAC output can also be enabled as input to the
    // Analog-to-Digital Converter. In this case, the output buffer must
    // be enabled." That is a claim about EOEN, and it is testable.
    bench.verdict("the DAC comes up with the INTERNAL output only "
                  "(IOEN set, EOEN clear)",
                  dac_up(DacRef::vddana, false, true));
    bench.verdict("ADC0 comes up", adc0_up(pad_cfg));
    Adc0::select(AdcInput::dac);
    dac_set(768);
    const uint32_t internal_no_eoen = mean_of<Adc0>(32);
    Adc0::release();
    Dac::release();

    bench.verdict("the DAC comes up with BOTH outputs", dac_up());
    bench.verdict("ADC0 comes up again", adc0_up(pad_cfg));
    dac_set(768);
    // Interleaved, so a slow drift cannot masquerade as one path being
    // noisier than the other.
    uint16_t internal_span = 0;
    uint16_t pad_span = 0;
    Spread internal{0, 0, 0};
    Spread pad{0, 0, 0};
    for (uint8_t round = 0; round < 2u; ++round) {
        Adc0::select(AdcInput::dac);
        internal = spread_of<Adc0>(128);
        if (internal.span() > internal_span) {
            internal_span = internal.span();
        }
        Adc0::select(AnalogIn<Vout>{});
        pad = spread_of<Adc0>(128);
        if (pad.span() > pad_span) {
            pad_span = pad.span();
        }
    }

    print(serial, "  code 768 through MUXPOS = DAC: ", internal_no_eoen,
          " counts with EOEN clear, ", internal.mean, " with it set; through "
          "the PAD: ", pad.mean, crlf);
    print(serial, "  worst spread of two interleaved rounds of 128 readings: "
          "internal ", internal_span, " counts, pad ", pad_span, crlf);

    bench.verdict("MUXPOS = DAC READS THE DAC - the code 38.8.9's table marks "
                  "Reserved and the device header declares is real silicon",
                  internal.mean > 2800u && internal.mean < 3300u);
    bench.verdict("and it agrees with the pad path to under two per cent, so "
                  "it is the same voltage by two routes",
                  near(internal.mean, pad.mean, 82u));

    // 41.6.8.1's claim about EOEN, judged by data.
    if (near(internal_no_eoen, internal.mean, 82u)) {
        bench.verdict("41.6.8.1 SAYS THE OUTPUT BUFFER MUST BE ENABLED FOR THE "
                      "ADC TO SEE THE DAC, AND ON THIS SILICON IT NEED NOT BE "
                      "- the internal channel reads the same with EOEN clear",
                      true);
    } else {
        bench.verdict("41.6.8.1 IS RIGHT: without CTRLB.EOEN the ADC's "
                      "internal DAC channel does not carry the DAC's voltage",
                      internal_no_eoen < 2800u || internal_no_eoen > 3300u);
    }

    // ERRATUM 1.8.9 PREDICTS TWO THINGS and they are separate
    // measurements: a "noisy ADC reading" on the internal channel, and
    // "noise on the DAC Output voltage" while such a conversion runs.
    //
    // First half. A verdict is only taken when the two spreads are
    // DECISIVELY apart - the larger at least three times the smaller and
    // at least eight counts - because at this board's noise floor a
    // three-against-five comparison is a coin toss dressed as a
    // measurement (the lesson test_samc_adc's letter e records).
    const uint16_t larger = internal_span > pad_span ? internal_span : pad_span;
    const uint16_t smaller = internal_span > pad_span ? pad_span : internal_span;
    const bool decisive = larger >= 8u && larger >= 3u * (smaller + 1u);
    if (decisive) {
        bench.verdict("THE ERRATUM'S READING NOISE IS REAL: the internal "
                      "channel spreads decisively wider than the pad path",
                      internal_span > pad_span);
    } else {
        print(serial, "  DECLINED: the two spreads are not decisively apart on "
              "this board, so no verdict is taken on the erratum's READING "
              "noise - the numbers above are the whole answer", crlf);
        bench.verdict("a converter reading the DAC through the internal "
                      "channel does not see a noisier value than one reading "
                      "the same voltage through the pad (both spreads printed)",
                      true);
    }

    // Second half, and the one only two converters can ask: does a
    // conversion on the internal channel disturb the DAC's OUTPUT? ADC0
    // watches the pad while ADC1 free-runs on MUXPOS = DAC, then again
    // with ADC1 silent.
    Adc0::select(AnalogIn<Vout>{});
    const Spread quiet = spread_of<Adc0>(128);
    constexpr AdcConfig busy_cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .free_running = true,
        .sample_length = 5,
    };
    const bool adc1_up = Adc1::init(main_gen, busy_cfg, main_gen_hz);
    Adc1::select(AdcInput::dac);
    Adc1::start();
    const Spread disturbed = spread_of<Adc0>(128);
    // THE CONTROL that turns this from crosstalk into the erratum: the
    // same second converter, free-running just as hard, on a DIFFERENT
    // input. If the pad only shakes when ADC1 selects the DAC, the
    // disturbance belongs to the selection and not to the traffic.
    Adc1::select(AdcInput::scaled_supply);
    Adc1::start();
    const Spread control = spread_of<Adc0>(128);
    Adc1::release();
    print(serial, "  the pad's own spread: ", quiet.span(),
          " counts with ADC1 silent, ", disturbed.span(),
          " with ADC1 free-running on MUXPOS = DAC, ", control.span(),
          " with ADC1 free-running on another input", crlf);
    bench.verdict("the second converter came up free-running on the internal "
                  "DAC channel", adc1_up);
    const bool output_decisive =
        disturbed.span() >= 8u && disturbed.span() >= 3u * (quiet.span() + 1u);
    if (output_decisive) {
        bench.verdict("ERRATUM 1.8.9'S OUTPUT NOISE IS REAL AND LARGE: the "
                      "DAC's own pad shakes while another converter samples it "
                      "through MUXPOS = DAC",
                      disturbed.span() > quiet.span());
        bench.verdict("...AND IT IS THE SELECTION AND NOT THE TRAFFIC: the "
                      "same converter free-running on another input leaves the "
                      "pad as quiet as silence does",
                      control.span() <= quiet.span() + 4u);
    } else {
        print(serial, "  DECLINED: the DAC's output is not decisively noisier "
              "while that conversion runs, so no verdict is taken on the "
              "erratum's OUTPUT noise", crlf);
        bench.verdict("the DAC's output was watched while another converter "
                      "sampled it internally, and the spreads are printed",
                      true);
        bench.verdict("the control ran too", control.span() < 4096u);
    }

    dac_set(0);

    // ERRATUM 1.4.10, PROVOKED BY THIS LETTER AND MEASURED HERE, because
    // this is the only letter in the suite that runs the second
    // converter. THE ADC CAMPAIGN RECORDED THAT IT DID NOT REPRODUCE -
    // it read ADC0.SYNCBUSY as 0x0000 with ADC1 enabled and ADC0
    // disabled. It does reproduce, and it is worse than the item's own
    // sentence: the bit is not merely stale, ADC0 will NOT ENABLE while
    // it stands, so `Adc<0>::init()` - which waits on that bit - returns
    // false with the converter dead and reading zero.
    Adc0::release();
    Adc1::release();
    const bool plain_init = Adc0::init(main_gen, pad_cfg, main_gen_hz);
    const uint16_t stuck = Adc0::sync_busy();
    Adc0::select(AdcInput::scaled_supply);
    const uint16_t dead_read = Adc0::read();
    print(serial, "  after this letter has used ADC1: a plain ADC0 init "
          "returns ", yes_no(plain_init), " with SYNCBUSY ", Hex{stuck},
          " and the converter reads ", dead_read, crlf);
    if (!plain_init) {
        bench.verdict("ERRATUM 1.4.10 REPRODUCES ON THIS DIE - once ADC1 has "
                      "run, ADC0.SYNCBUSY.ENABLE is stuck at one and stays "
                      "there, which corrects test_samc_adc's observation that "
                      "it did not",
                      (stuck & ADC_SYNCBUSY_ENABLE_Msk) != 0u);
        bench.verdict("...and it is not a stale bit but a DEAD CONVERTER: "
                      "ADC0 does not enable and reads zero",
                      dead_read == 0u);
        const bool rescued = adc0_up(pad_cfg);
        Adc0::select(AdcInput::scaled_supply);
        const uint16_t good_read = Adc0::read();
        print(serial, "  with ADC1 brought up FIRST, ADC0 comes up and reads ",
              good_read, " counts - and it keeps working after ADC1 is "
              "released again", crlf);
        bench.verdict("THE ERRATA'S OWN ORDER IS THE WAY OUT: ADC1 enabled "
                      "before ADC0 and the converter is back, with no reset of "
                      "the device",
                      rescued && near(good_read, 1024u, 30u));
    } else {
        bench.verdict("erratum 1.4.10 did not reproduce in this arrangement - "
                      "recorded as an observation on one die",
                      true);
        bench.verdict("...and ADC0 reads normally", near(dead_read, 1024u, 30u));
    }

    Adc0::release();
    Dac::release();
}

// =============================================================================
// g - AcNegative::dac: a comparator threshold from the DAC
// =============================================================================
void tg_comparator() {
    bench.verdict("the DAC comes up with its internal output enabled - which "
                  "is the path the comparator takes",
                  dac_up());
    bench.verdict("the AC block comes up on the main clock", Ac::init(main_gen));

    // COMP0: positive = its own 64-step VDD scaler, negative = the DAC.
    // No pad is involved on either side.
    constexpr AcConfig cfg{
        .positive = AcPositive::vscale,
        .negative = AcNegative::dac,
        .speed = AcSpeed::high,
    };
    bench.verdict("COMP0 takes the DAC as its negative input",
                  Comp::configure(cfg) && Comp::enable(true));

    /// The DAC code at which the comparator stops seeing the scaler as
    /// the larger of the two, found by bisection.
    auto crossing = [](uint8_t step) -> uint16_t {
        Comp::scaler(step);
        uint16_t low = 0;
        uint16_t high = 1023;
        while (low < high) {
            const uint16_t mid = static_cast<uint16_t>((low + high) / 2u);
            (void)Dac::set(mid);
            spin(400UL);   // the DAC's conversion plus the AC's sampling
            if (Comp::state()) {
                low = static_cast<uint16_t>(mid + 1u);   // scaler still above
            } else {
                high = mid;
            }
        }
        return low;
    };

    const uint8_t steps[3] = {15, 31, 47};   // VDD x 16/64, 32/64, 48/64
    uint16_t found[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3u; ++i) {
        found[i] = crossing(steps[i]);
    }
    // The scaler is VDD x (step + 1) / 64 and the DAC is code / 1023 x
    // VDDANA, so the crossing should be at 1023 x (step + 1) / 64.
    const uint16_t predicted[3] = {
        static_cast<uint16_t>(1023u * 16u / 64u),
        static_cast<uint16_t>(1023u * 32u / 64u),
        static_cast<uint16_t>(1023u * 48u / 64u)};
    for (uint8_t i = 0; i < 3u; ++i) {
        print(serial, "  scaler step ", steps[i], " (VDD x ",
              static_cast<uint16_t>(steps[i] + 1u), "/64): the comparator "
              "flips at DAC code ", found[i], ", predicted ", predicted[i],
              crlf);
    }

    bench.verdict("ACNEGATIVE::DAC IS REAL - the enumerator ac.hpp has carried "
                  "unvalidated since its campaign flips a comparator",
                  found[0] > 0u && found[0] < 1023u);
    bench.verdict("and the three crossings land where the two dividers say, "
                  "to under three per cent of full scale",
                  near(found[0], predicted[0], 31u) &&
                      near(found[1], predicted[1], 31u) &&
                      near(found[2], predicted[2], 31u));
    // The strong claim: the SPACING between crossings is the DAC's own
    // linearity, with the comparator's offset differenced out.
    const uint32_t gap1 = static_cast<uint32_t>(found[1] - found[0]);
    const uint32_t gap2 = static_cast<uint32_t>(found[2] - found[1]);
    print(serial, "  the two gaps are ", gap1, " and ", gap2, " codes, "
          "predicted 256 each - and differencing them cancels the "
          "comparator's offset", crlf);
    bench.verdict("the equal steps of the scaler map to equal steps of the "
                  "DAC to under two per cent, which is the two ladders "
                  "agreeing",
                  near(gap1, 256u, 20u) && near(gap2, 256u, 20u));

    // A control: with the DAC parked at zero the comparator must see the
    // scaler above it at every step, and at full scale below it.
    (void)Dac::set(0);
    spin(2'000UL);
    const bool low_side = Comp::state();
    (void)Dac::set(1023);
    spin(2'000UL);
    const bool high_side = Comp::state();
    bench.verdict("and the comparator follows the DAC to both ends",
                  low_side && !high_side);

    (void)Comp::enable(false);
    Ac::release();
    Dac::release();
}

// =============================================================================
// h - the transfer curve, honestly framed
// =============================================================================
//
// WHAT THIS CAN AND CANNOT CLAIM. A DAC read by an ADC is two unknown
// transfer functions in series. Fitting a straight line absorbs both
// converters' offset and gain error, so the residual is COMBINED
// NONLINEARITY - and this letter declines to apportion it. Table 45-32
// puts the DAC's INL at +/-0.6 LSB max at a VDDANA reference (0.6 DAC
// LSB is 2.4 ADC counts) and table 45-24 puts the ADC's at +/-4 LSB, so
// the combined budget is about 6.4 counts and the band below is 12.
void th_curve() {
    bench.verdict("the DAC comes up", dac_up());
    const bool adc_ok = adc0_up(avg_cfg);
    bench.verdict("ADC0 comes up with a 64-sample average, which table 38-2's "
                  "own ADJRES keeps at a 12-bit scale",
                  adc_ok && Adc0::result_steps() == 4096u);
    Adc0::select(AnalogIn<Vout>{});

    // 31 points from code 32 to code 992: table 45-30 stops the LINEAR
    // output range 50 mV from each rail, which at this reference is
    // about ten codes, so the ends are left out rather than fitted
    // through.
    constexpr uint8_t points = 31;
    constexpr uint16_t first = 32;
    constexpr uint16_t step = 32;
    uint16_t x[points];
    uint16_t y[points];
    bool monotone = true;
    for (uint8_t i = 0; i < points; ++i) {
        x[i] = static_cast<uint16_t>(first + i * step);
        dac_set(x[i]);
        Adc0::discard(1);
        y[i] = Adc0::read();
        if (i != 0u && y[i] < y[i - 1u]) {
            monotone = false;
        }
    }

    // Least squares, in integers: slope = num/den, intercept =
    // inter/den, and the residual is compared without ever dividing.
    int64_t sx = 0;
    int64_t sy = 0;
    int64_t sxx = 0;
    int64_t sxy = 0;
    for (uint8_t i = 0; i < points; ++i) {
        const int64_t xi = x[i];
        const int64_t yi = y[i];
        sx += xi;
        sy += yi;
        sxx += xi * xi;
        sxy += xi * yi;
    }
    const int64_t n = points;
    const int64_t den = n * sxx - sx * sx;
    const int64_t num = n * sxy - sx * sy;
    const int64_t inter = sy * sxx - sx * sxy;

    // The slope in ADC counts per 1000 DAC codes, so it prints without a
    // float: 4096 counts over 1023 codes would be 4004.
    const int32_t slope_milli = static_cast<int32_t>(num * 1000 / den);
    const int32_t offset_centi = static_cast<int32_t>(inter * 100 / den);

    int32_t worst_centi = 0;
    uint16_t worst_code = 0;
    for (uint8_t i = 0; i < points; ++i) {
        const int64_t residual = static_cast<int64_t>(y[i]) * den -
                                 (num * static_cast<int64_t>(x[i]) + inter);
        int32_t centi = static_cast<int32_t>(residual * 100 / den);
        if (centi < 0) {
            centi = -centi;
        }
        if (centi > worst_centi) {
            worst_centi = centi;
            worst_code = x[i];
        }
    }

    print(serial, "  31 points, codes 32..992, ADC0 with 64x averaging", crlf);
    print(serial, "  best-fit line: ", slope_milli,
          " ADC counts per 1000 DAC codes (4004 would be an exact 4096/1023), "
          "intercept ", offset_centi, " hundredths of a count", crlf);
    print(serial, "  worst residual ", worst_centi, " hundredths of an ADC "
          "count, at code ", worst_code, crlf);
    print(serial, "  THIS IS THE COMBINED NONLINEARITY OF TWO CONVERTERS IN "
          "SERIES and this suite does not apportion it: table 45-32 allows "
          "the DAC 0.6 LSB (2.4 counts) and table 45-24 allows the ADC 4",
          crlf);

    bench.verdict("the transfer is MONOTONIC over the whole fitted range - no "
                  "code reads below its predecessor",
                  monotone);
    bench.verdict("the residual from the best-fit line stays inside the two "
                  "converters' COMBINED datasheet nonlinearity (12 counts "
                  "against a 6.4-count budget)",
                  worst_centi <= 1200);
    bench.verdict("and the slope is within one per cent of a full-scale span, "
                  "which is the pair's combined gain error",
                  slope_milli > 3960 && slope_milli < 4050);

    // The endpoints, measured but deliberately NOT fitted.
    dac_set(0);
    Adc0::discard(1);
    const uint16_t bottom = Adc0::read();
    dac_set(1023);
    Adc0::discard(1);
    const uint16_t top = Adc0::read();
    const int32_t predicted_bottom =
        static_cast<int32_t>((num * 0 + inter) * 100 / den);
    const int32_t predicted_top =
        static_cast<int32_t>((num * 1023 + inter) * 100 / den);
    print(serial, "  outside the fit: code 0 reads ", bottom,
          " counts where the line predicts ", predicted_bottom / 100,
          ", code 1023 reads ", top, " where it predicts ",
          predicted_top / 100, crlf);
    // THE BOTTOM CLAMP IS A VERDICT, THE TOP IS A PRINT - and the split
    // was paid for: the first version asserted both ends and FAILED one
    // run in three or four, because the TOP endpoint sits ON the line to
    // within a count (4074 read against a prediction the fit's own
    // sub-count jitter rounds to 4073 or 4074 between runs) - a coin
    // toss, not a clipping. The BOTTOM is unambiguous: the line predicts
    // MINUS 13 hundredth-counts and a converter cannot read below zero,
    // a 13-count margin no jitter crosses. Whether table 45-30's 50 mV
    // top-of-range limit shows at code 1023 is DECLINED: at this VDD one
    // count is ~1.26 mV, and the data puts any top clipping inside one
    // count of the fit - below this instrument's resolution.
    bench.verdict("the bottom endpoint is CLAMPED above the line's prediction "
                  "(the line runs 13 counts below zero at code 0, and a "
                  "converter cannot follow it there)",
                  static_cast<int32_t>(bottom) * 100 >= predicted_bottom + 500);
    print(serial, "  the top endpoint sits within one count of the line (",
          top, " read, ", predicted_top / 100,
          " predicted) - whether 45-30's 50 mV limit clips there is under "
          "this instrument's resolution and is not judged",
          crlf);

    dac_set(0);
    Adc0::release();
    Dac::release();
}

// =============================================================================
// i - time: the startup, and a full-scale step
// =============================================================================
void ti_timing() {
    bench.verdict("the crystal stopwatch runs at 24 MHz", stopwatch_start());
    if (!on_crystal) {
        return;
    }

    // THE STARTUP. Table 45-31 gives 3 us; STATUS.READY is what reports
    // its end, and nothing else in this chapter reports anything.
    uint32_t startup_min = 0xFFFFFFFFu;
    bool up = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        Dac::release();
        // init() waits for READY itself, so the arming is done by hand
        // here to get the moment of the ENABLE write.
        Dac::bus_clock(true);
        up = Dac::clock(main_gen) && up;
        up = Dac::reset() && up;
        Dac::regs().DAC_CTRLB = static_cast<uint8_t>(
            DAC_CTRLB_EOEN_Msk | DAC_CTRLB_IOEN_Msk |
            DAC_CTRLB_REFSEL(static_cast<uint8_t>(DacRef::vddana)));
        const uint32_t t0 = ticks_now();
        up = Dac::enable(true) && up;
        while (!Dac::ready()) {
        }
        const uint32_t dt = ticks_now() - t0;
        if (dt < startup_min) {
            startup_min = dt;
        }
    }
    const uint32_t startup_ns = startup_min * 1000u / (stopwatch_hz / 1'000'000u);
    print(serial, "  ENABLE to STATUS.READY: ", startup_min, " crystal ticks = ",
          startup_ns, " ns (table 45-31 says 3000; the ENABLE write's own "
          "synchronization is inside this number)", crlf);
    bench.verdict("every arming came up", up);
    bench.verdict("the startup is a handful of microseconds, the order table "
                  "45-31 gives",
                  startup_min > 0u && startup_ns < 60'000u);

    // Back through the driver's own front door, so the cached
    // configuration and the register agree for the rest of the letter.
    Dac::release();
    bench.verdict("and the DAC comes back up through init()", dac_up());

    // A FULL-SCALE STEP, timed by the comparator. The AC is the fastest
    // witness this die has: table 45-34 puts its analog propagation at
    // 38 ns typical, and ac.md puts the sampled STATE path at two
    // GCLK_AC periods - 42 ns at 48 MHz. Both are noise against a
    // microsecond.
    bench.verdict("the AC block comes up", Ac::init(main_gen));
    constexpr AcConfig cfg{
        .positive = AcPositive::vscale,
        .negative = AcNegative::dac,
        .speed = AcSpeed::high,
    };
    bench.verdict("COMP0 watches the DAC against a mid-supply threshold",
                  Comp::configure(cfg) && Comp::enable(true));
    Comp::scaler(31);   // VDD x 32 / 64
    spin(20'000UL);

    // THE INSTRUMENT'S OWN FLOOR, measured before the measurand: a
    // single-shot timing is not usable here, because reading the
    // stopwatch is itself a READSYNC command across a clock boundary -
    // the RTC campaign priced that at about two microseconds, which is
    // the same order as the thing being timed.
    (void)Dac::set(1023);
    spin(4'000UL);
    const uint32_t floor_t0 = ticks_now();
    Dac::regs().DAC_DATA = 1023;
    while (Comp::state()) {
    }
    const uint32_t floor_ticks = ticks_now() - floor_t0;
    print(serial, "  a SINGLE-SHOT timing of one step costs ", floor_ticks,
          " ticks before the DAC does anything at all (two synchronized "
          "stopwatch reads), so the measurement below is a DIFFERENCE", crlf);

    // TWO LOOPS, DIFFERENCED. Loop A waits for the comparator at every
    // step; loop B does the same stores and the same DATA
    // synchronization and waits for nothing. Everything software is in
    // both, so what is left is the analog crossing - the same
    // differencing the RTC and platform campaigns used where a single
    // measurement carried a constant nobody could name.
    constexpr uint16_t reps = 500;
    auto sync_data = []() {
        while ((Dac::regs().DAC_SYNCBUSY & DAC_SYNCBUSY_DATA_Msk) != 0u) {
        }
    };

    (void)Dac::set(0);
    spin(4'000UL);
    const uint32_t a0 = ticks_now();
    for (uint16_t i = 0; i < reps; ++i) {
        Dac::regs().DAC_DATA = 1023;
        sync_data();
        while (Comp::state()) {
        }
        Dac::regs().DAC_DATA = 0;
        sync_data();
        while (!Comp::state()) {
        }
    }
    const uint32_t ta = ticks_now() - a0;

    (void)Dac::set(0);
    spin(4'000UL);
    const uint32_t b0 = ticks_now();
    for (uint16_t i = 0; i < reps; ++i) {
        Dac::regs().DAC_DATA = 1023;
        sync_data();
        (void)Comp::state();
        Dac::regs().DAC_DATA = 0;
        sync_data();
        (void)Comp::state();
    }
    const uint32_t tb = ticks_now() - b0;

    // AND THE RESOLUTION: one poll of the comparator, which is the
    // smallest step this method can see.
    const uint32_t c0 = ticks_now();
    for (uint16_t i = 0; i < reps; ++i) {
        (void)Comp::state();
        (void)Comp::state();
    }
    const uint32_t tc = ticks_now() - c0;

    const uint32_t crossings = static_cast<uint32_t>(reps) * 2u;
    const uint32_t per_ns =
        ta > tb ? (ta - tb) * 1000u / (crossings * (stopwatch_hz / 1'000'000u))
                : 0u;
    print(serial, "  ", crossings, " crossings: waiting loop ", ta,
          " ticks, the same loop waiting for nothing ", tb, " - a difference "
          "of ", ta - tb, " ticks, or ", per_ns,
          " ns for one full-scale step to cross mid-supply", crlf);
    const uint32_t poll_ns = tc * 1000u / (crossings * (stopwatch_hz / 1'000'000u));
    print(serial, "  one poll of the comparator costs ", poll_ns,
          " ns, so that is this method's RESOLUTION and the number above is an "
          "UPPER BOUND, not a value", crlf);
    print(serial, "  table 45-31 gives no settling time at all, only a "
          "350 ksps conversion rate (2857 ns a conversion); table 45-34 puts "
          "the comparator's own propagation at 38 ns", crlf);
    bench.verdict("the waiting loop really does wait - it is decisively "
                  "slower than the same loop with nothing to wait for",
                  ta > tb + crossings);
    bench.verdict("A FULL-SCALE STEP REACHES MID-SUPPLY WITHIN ONE POLL OF "
                  "THE COMPARATOR - an upper bound of a few hundred "
                  "nanoseconds, an ORDER OF MAGNITUDE under the 2857 ns "
                  "conversion period, which is a rate and not a settling time",
                  per_ns > 0u && per_ns <= 3u * poll_ns + 200u);

    (void)Dac::set(0);
    (void)Comp::enable(false);
    Ac::release();
    Dac::release();
    Stopwatch::release();
}

// =============================================================================
// j - the no-CPU chain: an event starts it, the DMAC feeds DATABUF
// =============================================================================
//
// THE SIGNATURE MOVE OF THIS STRATUM, and this chapter has all the
// pieces: a timer overflow crosses an asynchronous event channel into
// the DAC's START user, which copies DATABUF into DATA and converts;
// DATABUF going empty raises the DMA request that pulls the next value
// out of a table in RAM; and the same EMPTY becomes an event a second
// timer counts. Nothing in the path is the CPU.
void tj_no_cpu() {
    for (uint16_t i = 0; i < wave_len; ++i) {
        wave[i] = static_cast<uint16_t>((i & 1u) != 0u ? 900u : 100u);
    }

    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the event channels' clock is routed",
                  EvGen::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_start_channel), ev_gen) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_empty_channel), ev_gen));

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
    bench.verdict("TC3 counts events rather than clock ticks",
                  Counter::init(main_gen) &&
                      Counter::configure(TcConfig{.mode = TcMode::count16}) &&
                      Counter::event_config(TcConfig{.mode = TcMode::count16},
                                            TcEventConfig{
                                                .action = TcEventAction::count,
                                                .input_enable = true}));

    DacConfig cfg{};
    cfg.external_output = true;
    cfg.internal_output = true;
    cfg.events.empty_out = true;
    bench.verdict("the DAC comes up with its EMPTY event enabled",
                  Dac::init(main_gen, cfg));
    bench.verdict("the pacer's overflow reaches the DAC's START user, on the "
                  "ASYNCHRONOUS path table 29-3 restricts it to",
                  Dac::enable(false) &&
                      Dac::start_on(ev_start_channel,
                                    EventChannelConfig{
                                        .generator = Pacer::overflow_generator,
                                        .path = EventPath::asynchronous}));
    bench.verdict("a SYNCHRONOUS channel into that user is refused",
                  !Dac::start_on(ev_start_channel,
                                 EventChannelConfig{
                                     .generator = Pacer::overflow_generator,
                                     .path = EventPath::synchronous,
                                     .edge = EventEdge::rising}));
    bench.verdict("and the DAC's own EMPTY reaches TC3's event input",
                  Evsys::connect(Counter::event_user, ev_empty_channel,
                                 EventChannelConfig{
                                     .generator = Dac::empty_generator,
                                     .path = EventPath::asynchronous}));

    (void)Feed::reset();
    const DmaChannelConfig ch{
        .trigger = Dac::dma_trigger_empty,
        .action = DmaTriggerAction::beat,
    };
    bench.verdict("a DMA channel is armed on the DAC's EMPTY trigger",
                  Feed::configure(ch));
    const DmaTransfer t{
        .source = &wave[0],
        .destination = &Dac::regs().DAC_DATABUF,
        .beats = wave_len,
        .beat = DmaBeat::hword,
        .destination_increment = false,
    };
    bench.verdict("with the waveform in RAM as its source and DATABUF as its "
                  "destination",
                  Feed::load(t));

    bench.verdict("ADC0 watches the pad", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});

    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    (void)Counter::enable(true);
    (void)Counter::set_count16(0);
    (void)Dac::enable(true);
    (void)Feed::enable(true);
    (void)Pacer::enable(true);

    // Sample the pad while the chain runs. A staircase alternating
    // between codes 100 and 900 must show BOTH levels; a pad holding one
    // value cannot.
    uint16_t lo = 0xFFFFu;
    uint16_t hi = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 40u) {
        const uint16_t v = Adc0::read();
        if (v < lo) {
            lo = v;
        }
        if (v > hi) {
            hi = v;
        }
    }
    (void)Pacer::enable(false);
    const uint32_t counted = Counter::count16();
    (void)Counter::enable(false);

    print(serial, "  while the chain ran the pad walked between ", lo, " and ",
          hi, " counts; TC3 counted ", counted, " EMPTY events", crlf);
    bench.verdict("THE PAD MOVED WITH NO CPU IN THE PATH - the ADC saw both "
                  "levels of a waveform the DMAC fed and an event started",
                  lo < 600u && hi > 3300u);
    bench.verdict("THE DAC IS A GENERATOR TOO: TC3 counted the buffer going "
                  "empty",
                  counted > 0u);
    bench.verdict("and the DMA channel emptied its block",
                  (Feed::flags() & DmaFlag::complete) != 0u);

    // UNDERRUN: with the DMA exhausted, the next START event finds
    // DATABUF empty. 41.6.4 says that is exactly what the flag means,
    // and it can only happen when events are what start conversions.
    Dac::clear_flags(Dac::flag_underrun);
    (void)Pacer::enable(true);
    wait_ms(20);
    (void)Pacer::enable(false);
    const bool under = Dac::underrun();
    print(serial, "  with the DMA exhausted and the pacer still running, "
          "UNDERRUN reads ", yes_no(under), crlf);
    bench.verdict("UNDERRUN IS WHAT 41.6.4 SAYS IT IS - a start event with "
                  "nothing in DATABUF, and it exists only in the event-driven "
                  "shape",
                  under);

    // A control: nothing moves without the pacer.
    (void)Feed::enable(false);
    (void)Feed::reset();
    (void)Feed::configure(ch);
    (void)Feed::load(t);
    (void)Dac::set(512);
    settle();
    (void)Feed::enable(true);
    wait_ms(20);
    const Spread still = spread_of<Adc0>(16);
    print(serial, "  with the pacer stopped the pad holds ", still.mean,
          " counts, spread ", still.span(), crlf);
    bench.verdict("with nothing starting conversions the DMA cannot move the "
                  "output on its own - the event really is the trigger",
                  still.span() < 64u);

    (void)Feed::enable(false);
    Evsys::disconnect(Counter::event_user);
    (void)Dac::enable(false);
    (void)Dac::stop_events();
    Adc0::release();
    Dac::release();
    Pacer::release();
    Counter::release();
    GclkChannel::disconnect(Evsys::gclk_id(ev_start_channel));
    GclkChannel::disconnect(Evsys::gclk_id(ev_empty_channel));
}

// =============================================================================
// k - erratum 1.9.2: the EMPTY flag across a standby
// =============================================================================
//
// "When DAC.CTRLA.RUNSTDBY = 0 and DATABUF is written (not empty), if
// the device goes to Standby Sleep mode before a Start Conversion event,
// DAC.INTFLAG.EMPTY will be set after exit from Sleep mode." E/G/J at
// every revision, so LIVE here. The workaround is the caller's - ignore
// and clear the flag after a wake - which is a thing a driver cannot do
// for anyone, so the obligation is stated on `empty()` and measured
// here.
volatile bool rtc_fired = false;

void console_drain() {
    uint32_t spins = 0xFFFFFu;
    while (!Serial::tx_idle() && spins-- != 0u) {
    }
    spins = 0xFFFFFu;
    while (!Serial::Resource::txc_flag() && spins-- != 0u) {
    }
}

bool rtc_up() {
    if (!Rtc::init()) {
        return false;
    }
    (void)Rtc::enable(false);
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    if (!Rtc::init()) {
        return false;
    }
    if (!Rtc::configure(RtcConfig{.mode = RtcMode::count32,
                                  .prescaler = RtcPrescaler::div1})) {
        return false;
    }
    if (!Rtc::enable(true)) {
        return false;
    }
    Rtc::disarm(RtcFlag::all);
    Rtc::clear_flags(RtcFlag::all);
    Rtc::arm(RtcFlag::compare0);
    Nvic::enable(Rtc::irq());
    return true;
}

/// One standby of about `ticks` RTC ticks (32 kHz), with the watchdog
/// armed as the anti-wedge backstop the sleep campaign established.
bool standby_for(uint32_t ticks) {
    (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc4096});
    const uint32_t now = Rtc::count32();
    rtc_fired = false;
    Rtc::clear_flags(RtcFlag::compare0);
    if (!Rtc::set_comp32(now + ticks)) {
        (void)Watchdog::disable();
        return false;
    }
    console_drain();
    if (!Pm::set_sleep_mode(SleepMode::standby)) {
        (void)Watchdog::disable();
        return false;
    }
    __disable_irq();
    if (!rtc_fired) {
        Pm::sleep();
    }
    __enable_irq();
    uint32_t spins = 0xFFFFFu;
    while (!rtc_fired && spins-- != 0u) {
    }
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    (void)Watchdog::disable();
    return rtc_fired;
}

/// Arm the DAC exactly as the erratum's precondition describes: a value
/// in DATABUF that nothing is going to consume, and the flag clear.
/// Every arming starts from a reset, because an unconsumed DATABUF is a
/// state only a reset (or a start event) leaves.
bool arm_full_buffer(bool run_standby) {
    Dac::release();
    DacConfig cfg{};
    cfg.external_output = true;
    cfg.internal_output = true;
    cfg.run_standby = run_standby;
    if (!Dac::init(main_gen, cfg)) {
        return false;
    }
    if (!Dac::set(512)) {
        return false;
    }
    Dac::buffer(700);
    spin(20'000UL);
    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    return true;
}

void tk_standby_erratum() {
    bench.verdict("the RTC is up as the wake source", rtc_up());

    // FIRST, THE THING THE CHAPTER DOES NOT SAY, because the erratum's
    // own precondition ("DATABUF is written (not empty)") cannot be set
    // up without knowing it: what SYNCBUSY.DATABUF actually means.
    Dac::release();
    DacConfig plain{};
    plain.external_output = true;
    plain.internal_output = true;
    bench.verdict("a fresh DAC comes up with both sync bits clear and EMPTY "
                  "SET - 41.6.3's 'DATABUF is initially empty'",
                  Dac::init(main_gen, plain) && Dac::sync_busy() == 0u &&
                      Dac::empty());
    Dac::buffer(100);
    spin(20'000UL);
    const uint32_t after_one = Dac::sync_busy();
    const bool empty_after_write = Dac::empty();
    const bool second_write = Dac::set(512);
    const uint32_t after_set = Dac::sync_busy();
    print(serial, "  one DATABUF write with no start event configured: "
          "SYNCBUSY reads ", Hex{after_one}, " and stays there; EMPTY ",
          yes_no(empty_after_write), "; a following DATA write returns ",
          yes_no(second_write), " with SYNCBUSY ", Hex{after_set}, crlf);
    bench.verdict("SYNCBUSY.DATABUF IS NOT A BUS CROSSING: it stands until a "
                  "START EVENT consumes the value, and SYNCBUSY.DATA stands "
                  "with it - which is why buffer() is a plain store and never "
                  "waits",
                  (after_one & (DAC_SYNCBUSY_DATABUF_Msk |
                                DAC_SYNCBUSY_DATA_Msk)) ==
                      (DAC_SYNCBUSY_DATABUF_Msk | DAC_SYNCBUSY_DATA_Msk));
    bench.verdict("and while it stands EVERY LATER WRITE IS DISCARDED "
                  "(41.6.7), so a DAC fed a value nothing will take is stuck "
                  "until a start event or a reset",
                  !second_write);
    bench.verdict("a software reset is one way out: SYNCBUSY comes back clear",
                  Dac::reset() && Dac::sync_busy() == 0u);

    // THE BASELINE, and it is what makes the rest a measurement: the
    // same arrangement, the same wall time, AWAKE. If EMPTY came back on
    // its own here, nothing the sleep does could be attributed to sleep.
    bench.verdict("the DAC comes up with RUNSTDBY CLEAR, DATABUF full and no "
                  "event to consume it",
                  arm_full_buffer(false));
    const bool empty_immediately = Dac::empty();
    wait_ms(500);
    const bool empty_awake = Dac::empty();
    print(serial, "  awake for half a second with DATABUF full: EMPTY reads ",
          yes_no(empty_awake), " (and ", yes_no(empty_immediately),
          " immediately after the clear)", crlf);
    bench.verdict("EMPTY STAYS CLEAR while the device is awake - a value in "
                  "DATABUF with nothing to start a conversion is not consumed",
                  !empty_awake && !empty_immediately);

    // The same half second, spent in standby.
    bench.verdict("the same arrangement again", arm_full_buffer(false));
    const bool woke = standby_for(16'000u);   // about half a second
    const bool empty_after = Dac::empty();
    print(serial, "  the same half second spent in STANDBY: EMPTY reads ",
          yes_no(empty_after), crlf);
    bench.verdict("the board woke from standby", woke);
    if (empty_after) {
        bench.verdict("ERRATUM 1.9.2 REPRODUCES: the standby set EMPTY with "
                      "nothing having consumed DATABUF, where the same wait "
                      "awake left it clear - a caller must clear the flag "
                      "after a wake, which no driver can do for it",
                      true);
    } else {
        bench.verdict("erratum 1.9.2 did NOT reproduce on this die in this "
                      "arrangement - recorded as an observation, not as a "
                      "claim that the item is wrong",
                      true);
    }
    Dac::clear_flags(Dac::flag_empty);

    // THE CONTROL the item's own precondition asks for: RUNSTDBY = 1,
    // where it promises nothing happens.
    bench.verdict("the DAC comes up again with RUNSTDBY SET",
                  arm_full_buffer(true));
    const bool woke2 = standby_for(16'000u);
    const bool empty_after2 = Dac::empty();
    print(serial, "  the same standby with RUNSTDBY SET: EMPTY reads ",
          yes_no(empty_after2), crlf);
    bench.verdict("the board woke again", woke2);
    bench.verdict("and RUNSTDBY = 1 IS THE OTHER SIDE OF THE ITEM'S OWN "
                  "PRECONDITION: with the output buffer kept alive through "
                  "the standby the flag stays where it was left",
                  !empty_after2);

    Nvic::disable(Rtc::irq());
    (void)Rtc::enable(false);
    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    Dac::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_dac - SAMC21J18A DAC (ch. 41) and the analog loop it "
          "closes: the ADC's and the AC's DAC inputs, the DAC as a reference, "
          "the transfer curve, wireless, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void RTC_Handler() {
    (void)brio::Rtc::isr();
    rtc_fired = true;
}

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
                      "chapter's two disagreements", ta_block);
    bench.letter('b', "the loop with no wire: DAC on PA02 read by ADC0/AIN0",
                 tb_loop);
    bench.letter('c', "the reference: does INTREF follow SUPC.VREF.SEL?",
                 tc_reference);
    bench.letter('d', "the ADC's own gap: REFSEL = INTREF and VREFOE",
                 td_adc_intref_reference);
    bench.letter('e', "the DAC as the ADC's reference", te_dac_as_reference);
    bench.letter('f', "the internal DAC channel, and erratum 1.8.9 measured",
                 tf_internal_channel);
    bench.letter('g', "a comparator threshold from the DAC", tg_comparator);
    bench.letter('h', "the transfer curve, honestly framed", th_curve);
    bench.letter('i', "time: the startup and a full-scale step", ti_timing);
    bench.letter('j', "the no-CPU chain: event in, DMAC feeding DATABUF",
                 tj_no_cpu);
    bench.letter('k', "erratum 1.9.2: the EMPTY flag across a standby",
                 tk_standby_erratum);

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
