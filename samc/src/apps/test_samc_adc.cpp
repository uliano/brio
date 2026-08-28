// test_samc_adc - the reference bench suite for samc/adc.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. An ADC input is a DIRECT connection to the pad, so a
// pad left under PORT and driven as an ordinary output is a rail the
// converter measures - the technique samc/ac.hpp's campaign established
// for the comparators and this one inherits. The other voltages in the
// room are all internal: the SUPC bandgap through MUXPOS INTREF, the
// quarter-scaled analog supply, the quarter-scaled core supply, and the
// three reference divisions of VDDANA.
//
// WHAT THIS BOARD CANNOT DO, said once: there is no DAC driver in this
// stratum, so no ramp and no arbitrary mid-rail voltage exists. Every
// measurement here is a rail, an internal divider or a bandgap; INL,
// DNL, a transfer curve and anything needing a swept source are OUT OF
// REACH and are named as gaps in docs/samc/adc.md rather than faked.
//
// THE PADS, and why these: PA08 and PA09 are the only two pads on this
// package that reach BOTH converters (PA08 = ADC0/AIN8 and ADC1/AIN10),
// which is what makes letter d - one input, two converters - possible at
// all. PA04 and PA05 are ADC0/AIN4 and AIN5 and both are free here.
//
// What is exercised, letter by letter:
//   a  the block: geometry, the vocabularies it publishes, the FACTORY
//      CALIBRATION copied out of the NVM row, and every refusal
//   b  the rails, and the internal supply channels against them
//   c  INTREF, VREFOE and where VDD really is - from the ADC's side
//   d  ONE PAD, TWO CONVERTERS: how far apart ADC0 and ADC1 read
//   e  averaging and oversampling: the noise they buy, measured
//   f  conversion time against 45-22's formula, on the crystal
//   g  the window monitor, and MODE4's documented ambiguity settled
//   h  the NO-CPU CHAIN: a timer event starts it, the DMAC takes the
//      result, and a second timer counts the result-ready events
//   i  util/analog_sampler.hpp INSIDE A REAL KERNEL, unchanged
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>
#include <variant>

#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "samc/adc.hpp"
#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sercom.hpp"
#include "samc/supc.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "util/analog.hpp"
#include "util/analog_sampler.hpp"
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
// The pads and the converters
// ---------------------------------------------------------------------------
using PadShared = Pin<'A', 8>;    // ADC0/AIN8 and ADC1/AIN10
using PadShared2 = Pin<'A', 9>;   // ADC0/AIN9 and ADC1/AIN11
using PadA4 = Pin<'A', 4>;        // ADC0/AIN4
using PadA5 = Pin<'A', 5>;        // ADC0/AIN5

using Adc0 = Adc<0>;
using Adc1 = Adc<1>;

/// GCLK generator 0 is the 48 MHz main clock; at DIV32 that is a
/// nominal 1.5 MHz CLK_ADC, comfortably inside table 45-22's
/// 160 kHz .. 16 MHz. Letter f swaps in the crystal, where TIME is the
/// measurand and OSC48M's 5100 ppm error would be the reported number's.
constexpr uint8_t adc_gen = 0;
constexpr uint32_t adc_gen_hz = SysClock::hz;

/// What the SUPC campaign located this board's supply at, through the
/// comparator's own scaler against the bandgap (docs/samc/supc.md). It
/// is the number letter c re-derives from the ADC's side, so it is a
/// STARTING POINT here and never a verdict's authority.
constexpr uint16_t supc_vdd_mv = 5141;
uint16_t vdd_mv = supc_vdd_mv;   ///< refined by letter c, used by the rest

// ---------------------------------------------------------------------------
// The stopwatch, for letter f only: TC0 + TC1 as one 32-bit counter, and
// THE RULER IS THE BOARD'S CRYSTAL. A conversion time reported against
// OSC48M would carry that oscillator's 5100 ppm and its wander into a
// verdict about the ADC.
// ---------------------------------------------------------------------------
using Stopwatch = Tc<0>;
constexpr uint32_t crystal_hz = 24'000'000UL;
constexpr uint8_t gen_crystal = 2;
constexpr uint32_t stopwatch_hz = crystal_hz / 8u;
bool on_crystal = false;

bool stopwatch_start() {
    on_crystal = Xosc::init(XoscConfig{.hz = crystal_hz, .startup = 4}) &&
                 Gclk<gen_crystal>::configure(GclkConfig{.source = GclkSource::xosc});
    if (!on_crystal) {
        return false;
    }
    return Stopwatch::init(gen_crystal) &&
           Stopwatch::configure(TcConfig{.mode = TcMode::count32,
                                         .prescaler = TcPrescaler::div8}) &&
           Stopwatch::enable(true);
}

uint32_t ticks_now() { return Stopwatch::count32(); }

// ---------------------------------------------------------------------------
// The event fabric, as in every SAM suite here: DMAC channel 0 is EVSYS
// user 5, and the event channels take their clock from generator 6.
// ---------------------------------------------------------------------------
constexpr uint8_t dma_ch = 0;
constexpr uint8_t ev_start_channel = 0;    // TC2 overflow -> ADC0 START
constexpr uint8_t ev_result_channel = 1;   // ADC0 RESRDY  -> TC3 counts
constexpr uint8_t ev_gen = 6;
using Copy = DmaChannel<dma_ch>;
using EvGen = Gclk<ev_gen>;
using Pacer = Tc<2>;      // the trigger source
using Counter = Tc<3>;    // counts result-ready events

/// VOLATILE IN BOTH DIRECTIONS - the lesson the DMAC campaign paid for
/// on this target: the compiler sees neither the controller's writes nor
/// its reads.
constexpr uint16_t dma_results = 16;
volatile uint16_t results[dma_results];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

void settle() { spin(20'000UL); }

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

/// Drive a pad from PORT. The pad stays under PORT - no PMUX - because
/// the analog input reaches it directly and the mux would take the
/// output driver away.
template <class P>
void drive(bool high) {
    P::output();
    if (high) {
        P::set();
    } else {
        P::clear();
    }
}

/// The precondition every measurement here rests on: does this pad go
/// where PORT drives it? Read back through PORT.IN before any converter
/// looks at it.
template <class P>
bool pad_follows_port() {
    P::output();
    P::set();
    settle();
    const bool high = P::read();
    P::clear();
    settle();
    const bool low = P::read();
    return high && !low;
}

/// One converter up on the main clock with a given configuration.
template <class A>
bool adc_up(const AdcConfig& cfg, uint8_t generator = adc_gen) {
    return A::init(generator, cfg, generator == adc_gen ? adc_gen_hz : crystal_hz);
}

/// The mean of `count` readings, in counts. The converter must be
/// configured and the input selected.
template <class A>
uint32_t mean_of(uint16_t count) {
    A::discard(2);   // the mux change and the first conversion after it
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

/// counts -> millivolts, against the configured reference.
template <class A>
uint16_t counts_mv(uint32_t counts, uint16_t known_mv,
                   VrefLevel level = VrefLevel::v1_024) {
    return adc_mv(counts, A::result_steps(), ref_mv(A::reference(), known_mv, level));
}

const char* yes_no(bool v) { return v ? "yes" : "no"; }

// =============================================================================
// a - the block, the vocabularies it publishes, the calibration, the refusals
// =============================================================================
void ta_block() {
    bench.verdict("two converters, twelve external channels each, and the pair "
                  "is not symmetric",
                  adc_count() == 2u && Adc0::external_channels == 12u &&
                      Adc1::external_channels == 12u && Adc0::is_host &&
                      Adc1::is_client);

    print(serial, "  EVSYS: ADC0 RESRDY gen ", Adc0::resrdy_generator,
          ", WINMON gen ", Adc0::winmon_generator, ", START user ",
          Adc0::start_event_user, ", FLUSH user ", Adc0::flush_event_user,
          " (asynchronous path only)", crlf);
    print(serial, "  DMAC: ADC0 RESRDY trigger ", Adc0::dma_trigger_resrdy,
          ", ADC1 ", Adc1::dma_trigger_resrdy, "; GCLK ids ", Adc0::gclk_id,
          " and ", Adc1::gclk_id, crlf);
    bench.verdict("the generator and user codes are table 29-3's own",
                  Adc0::resrdy_generator == 0x43u && Adc0::winmon_generator == 0x44u &&
                      Adc1::resrdy_generator == 0x45u &&
                      Adc0::start_event_user == 28u && Adc1::start_event_user == 30u);
    bench.verdict("and the DMA trigger ids are the header's",
                  Adc0::dma_trigger_resrdy == 42u && Adc1::dma_trigger_resrdy == 43u);

    // THE PAD MAP IS PER INSTANCE, which is the fact that made the
    // reserve's ADC entry need two maps rather than one.
    bench.verdict("ONE PAD, TWO NUMBERS: PA08 is ADC0/AIN8 and ADC1/AIN10",
                  Adc0::ain_of('A', 8) == 8 && Adc1::ain_of('A', 8) == 10);
    bench.verdict("and PA04 belongs to ADC0 alone",
                  Adc0::ain_of('A', 4) == 4 && Adc1::ain_of('A', 4) < 0);
    bench.verdict("this package (the J) bonds every AIN of both converters",
                  Adc0::ain_exists(2) && Adc1::ain_exists(0) && Adc1::ain_exists(6));

    // The refusals that are the chapter's rules.
    bench.verdict("accumulating more than one sample at 12-bit resolution is "
                  "REFUSED - 38.6.2.9's Note is the driver's rule",
                  !Adc0::config_valid(AdcConfig{.average = AdcAverage::samples16,
                                                .adjust = 4}) &&
                      Adc0::config_valid(AdcConfig{.resolution = AdcRes::bits16,
                                                   .average = AdcAverage::samples16,
                                                   .adjust = 4}));
    bench.verdict("offset compensation beside a non-zero SAMPLEN is refused "
                  "(38.8.12)",
                  !Adc0::config_valid(AdcConfig{.sample_length = 3,
                                                .offset_compensation = true}));
    bench.verdict("rail-to-rail without it is refused (38.6.3.2)",
                  !Adc0::config_valid(AdcConfig{.rail_to_rail = true}) &&
                      Adc0::config_valid(AdcConfig{.rail_to_rail = true,
                                                   .offset_compensation = true}));
    bench.verdict("a Reserved reference code is refused",
                  !Adc0::config_valid(AdcConfig{.reference = static_cast<Ref>(6)}));
    bench.verdict("SLAVEEN on the host and DUALSEL on the client are refused",
                  !Adc0::config_valid(AdcConfig{.client_enable = true}) &&
                      !Adc1::config_valid(AdcConfig{.dual = AdcDual::interleave}));
    bench.verdict("the negative multiplexer is SIX pads wide, not twelve",
                  adc_negative_valid(AdcNegative::ain5) &&
                      !adc_negative_valid(static_cast<AdcNegative>(6)));
    bench.verdict("at a 48 MHz generator DIV2 leaves the 16 MHz ceiling and "
                  "init() refuses it, while DIV4 is accepted",
                  !adc_clock_in_range(adc_gen_hz, AdcPresc::div2) &&
                      !Adc0::init(adc_gen, AdcConfig{.prescaler = AdcPresc::div2},
                                  adc_gen_hz) &&
                      adc_clock_in_range(adc_gen_hz, AdcPresc::div4));

    // THE PROMISE nvm.hpp's comment has carried since phase B1.
    const NvmCalibration cal = NvmCalibration::read();
    bench.verdict("ADC0 comes up", adc_up<Adc0>(AdcConfig{}));
    bench.verdict("ADC1 comes up", adc_up<Adc1>(AdcConfig{}));
    print(serial, "  NVM software calibration row: ADC0 biasrefbuf ",
          cal.adc0_biasrefbuf(), " biascomp ", cal.adc0_biascomp(),
          ", ADC1 biasrefbuf ", cal.adc1_biasrefbuf(), " biascomp ",
          cal.adc1_biascomp(), crlf);
    print(serial, "  CALIB after init: ADC0 ", Hex{Adc0::calibration()},
          ", ADC1 ", Hex{Adc1::calibration()}, crlf);
    bench.verdict("INIT COPIED THE FACTORY CALIBRATION INTO ADC0.CALIB - "
                  "38.5.10 calls it mandatory and samc/nvm.hpp typed the "
                  "fields for exactly this",
                  Adc0::bias_reference_buffer() == cal.adc0_biasrefbuf() &&
                      Adc0::bias_comparator() == cal.adc0_biascomp());
    bench.verdict("and ADC1's, from ITS OWN pair of fields",
                  Adc1::bias_reference_buffer() == cal.adc1_biasrefbuf() &&
                      Adc1::bias_comparator() == cal.adc1_biascomp());
    bench.verdict("CALIB is enable-protected, so a reload under a running "
                  "converter is refused",
                  Adc0::enabled() && !Adc0::load_calibration());

    // EVCTRL is enable-protected too, and ERRATUM 1.4.4 IS CODE.
    bench.verdict("the event control is refused while the converter runs",
                  !Adc0::event_config(AdcEventControl{.result_out = true}));
    bench.verdict("and accepted with it disabled",
                  Adc0::enable(false) &&
                      Adc0::event_config(AdcEventControl{.start_in = true,
                                                         .result_out = true}));
    const AdcEventControl back = Adc0::event_config();
    bench.verdict("reading back field for field", back.start_in && back.result_out &&
                                                      !back.flush_in);
    bench.verdict("ERRATUM 1.4.4 AS CODE: start_on() refuses a SYNCHRONOUS "
                  "channel - a synchronized event during a conversion is never "
                  "acknowledged and stalls the whole channel",
                  !Adc0::start_on(ev_start_channel,
                                  EventChannelConfig{.generator = 1,
                                                     .path = EventPath::synchronous,
                                                     .edge = EventEdge::rising}) &&
                      !Adc0::start_on(ev_start_channel,
                                      EventChannelConfig{
                                          .generator = 1,
                                          .path = EventPath::resynchronized,
                                          .edge = EventEdge::rising}));
    bench.verdict("inverting an event input nobody listens to is refused",
                  !Adc0::event_config(AdcEventControl{.invert_flush = true}));
    bench.verdict("stop_events() clears both users and both enables",
                  Adc0::stop_events() && !Adc0::event_config().start_in);

    Adc0::release();
    Adc1::release();
}

// =============================================================================
// b - the rails, and the internal supply channels against them
// =============================================================================
void tb_rails() {
    bench.verdict("PA08 reaches both rails under PORT - the precondition every\n"
                  "                analog measurement in this suite rests on",
                  pad_follows_port<PadShared>());
    bench.verdict("so do PA09, PA04 and PA05",
                  pad_follows_port<PadShared2>() && pad_follows_port<PadA4>() &&
                      pad_follows_port<PadA5>());

    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
    };
    bench.verdict("ADC0 comes up on VDDANA at 1.5 MHz", adc_up<Adc0>(cfg));
    print(serial, "  CLK_ADC = ", adc_clock_hz(adc_gen_hz, cfg.prescaler),
          " Hz nominal, one result every ", adc_conversion_cycles(cfg),
          " CLK_ADC cycles, full scale ", Adc0::result_steps(), crlf);

    Adc0::select(AnalogIn<PadShared>{});
    drive<PadShared>(true);
    settle();
    const uint32_t high = mean_of<Adc0>(32);
    drive<PadShared>(false);
    settle();
    const uint32_t low = mean_of<Adc0>(32);
    print(serial, "  PA08 driven: high ", high, " counts (", counts_mv<Adc0>(high, vdd_mv),
          " mV), low ", low, " counts (", counts_mv<Adc0>(low, vdd_mv), " mV)", crlf);
    bench.verdict("a pad driven HIGH reads full scale (within 1 %)",
                  high >= 4096u - 41u);
    bench.verdict("and driven LOW reads zero (within 1 % of full scale)",
                  low <= 41u);

    // THE CHEAPEST SELF-CHECK THE CONVERTER HAS: the scaled analog
    // supply against the analog supply itself is a quarter, by
    // construction, with no external voltage involved at all.
    Adc0::select(AdcInput::scaled_supply);
    settle();
    const uint32_t quarter = mean_of<Adc0>(64);
    print(serial, "  SCALEDVDDANA against VDDANA: ", quarter,
          " counts of ", Adc0::result_steps(), " (a quarter would be 1024)", crlf);
    bench.verdict("1/4 VDDANA against VDDANA is a quarter of full scale, "
                  "within 2 % - a ratio with no external voltage in it",
                  near(quarter, 1024u, 21u));

    Adc0::select(AdcInput::scaled_core);
    settle();
    const uint32_t core = mean_of<Adc0>(64);
    const uint16_t core_mv = static_cast<uint16_t>(4u * counts_mv<Adc0>(core, vdd_mv));
    print(serial, "  SCALEDVDDCORE: ", core, " counts -> VDDCORE about ", core_mv,
          " mV (at VDD ", vdd_mv, " mV)", crlf);
    bench.verdict("the scaled CORE supply sits where a 1.2 V regulated core "
                  "belongs - 900..1500 mV",
                  core_mv >= 900u && core_mv <= 1500u);

    // The three reference divisions, seen from the other side: with the
    // input HELD AT THE RAIL, a smaller reference saturates.
    Adc0::select(AnalogIn<PadShared>{});
    drive<PadShared>(true);
    bench.verdict("re-initialized on 1/2 VDDANA",
                  Adc0::enable(false) &&
                      adc_up<Adc0>(AdcConfig{.reference = Ref::vddana_div2,
                                             .prescaler = AdcPresc::div32,
                                             .sample_length = 5}));
    Adc0::select(AnalogIn<PadShared>{});
    settle();
    const uint32_t half_ref = mean_of<Adc0>(16);
    bench.verdict("re-initialized on 1/1.6 VDDANA",
                  Adc0::enable(false) &&
                      adc_up<Adc0>(AdcConfig{.reference = Ref::vddana_div1p6,
                                             .prescaler = AdcPresc::div32,
                                             .sample_length = 5}));
    Adc0::select(AnalogIn<PadShared>{});
    settle();
    const uint32_t small_ref = mean_of<Adc0>(16);
    print(serial, "  PA08 high against 1/2 VDDANA: ", half_ref,
          " counts; against 1/1.6 VDDANA: ", small_ref, crlf);
    bench.verdict("a rail against a reference BELOW it saturates at full "
                  "scale, both divisions",
                  half_ref >= 4090u && small_ref >= 4090u);

    drive<PadShared>(false);
    Adc0::release();
}

// =============================================================================
// c - INTREF, VREFOE, and where VDD really is
// =============================================================================
//
// The bandgap read against the supply is a RATIO whose numerator is
// known: 1.024, 2.048 or 4.096 V. So VDD = level * full_scale / counts,
// and this is the same supply the SUPC campaign located from the
// COMPARATOR's side (5251 / 5141 / 5090 mV at the three levels) with no
// mechanism in common - the AC compares, this one converts.
void tc_bandgap() {
    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        // 38.8.9 asks for a "corresponding value" on the INTREF channel
        // and table 45-22 puts it at 10 us; at 1.5 MHz that is SAMPLEN 14.
        .sample_length = 14,
    };
    bench.verdict("SAMPLEN for 10 us at 1.5 MHz is 14 (38.8.12's own formula)",
                  adc_samplen_for_ns(1'500'000, adc_intref_sampling_ns) == 14u);
    bench.verdict("ADC0 comes up with the long sample", adc_up<Adc0>(cfg));

    // FIRST THE QUESTION supc.hpp's own comment raises: 22.8.7 words
    // VREFOE as routing the bandgap "to an ADC input channel", which
    // would make this channel a floating promise without it.
    bench.verdict("the bandgap is configured at 1.024 V with VREFOE OFF",
                  Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                             .output_enable = false}));
    Adc0::select(AdcInput::intref);
    settle();
    const uint32_t without_oe = mean_of<Adc0>(32);
    bench.verdict("and then with VREFOE ON",
                  Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                             .output_enable = true}));
    settle();
    const uint32_t with_oe = mean_of<Adc0>(32);
    print(serial, "  MUXPOS INTREF at 1.024 V: VREFOE off ", without_oe,
          " counts, on ", with_oe, " counts", crlf);
    bench.verdict("THE BANDGAP CHANNEL IS DEAD WITHOUT SUPC.VREF.VREFOE - "
                  "22.8.7 words that bit as routing the reference 'to an ADC "
                  "input channel' and ch. 38 never mentions it, but MUXPOS "
                  "INTREF reads a flat zero until it is set",
                  without_oe == 0u);
    bench.verdict("and a real voltage once it is",
                  with_oe > 400u && with_oe < 1200u);

    // Now the supply, three ways, from the three bandgap levels.
    struct Level {
        VrefLevel level;
        uint16_t nominal;
        const char* name;
        uint16_t supc;
    };
    const Level levels[] = {
        {VrefLevel::v1_024, 1024, "1.024", 5251},
        {VrefLevel::v2_048, 2048, "2.048", 5141},
        {VrefLevel::v4_096, 4096, "4.096", 5090},
    };
    uint32_t sum_mv = 0;
    uint8_t counted = 0;
    bool all_plausible = true;
    for (const Level& l : levels) {
        (void)Vref::configure(VrefConfig{.level = l.level, .output_enable = true});
        settle();
        Adc0::select(AdcInput::intref);
        const uint32_t counts = mean_of<Adc0>(64);
        const uint32_t supply =
            counts == 0u ? 0u
                         : (static_cast<uint32_t>(l.nominal) * Adc0::result_steps() +
                            counts / 2u) / counts;
        print(serial, "  INTREF ", l.name, " V reads ", counts,
              " counts -> VDDANA ", supply, " mV (the AC said ", l.supc, ")", crlf);
        if (supply < 4500u || supply > 5800u) {
            all_plausible = false;
        } else {
            sum_mv += supply;
            ++counted;
        }
    }
    bench.verdict("ALL THREE BANDGAP LEVELS PUT VDD IN THE SAME PLACE, and it "
                  "is a 5 V board: 4500..5800 mV each",
                  all_plausible && counted == 3u);
    const uint32_t mean_vdd = counted == 0u ? supc_vdd_mv : (sum_mv + counted / 2u) / counted;
    print(serial, "  ADC-side mean VDDANA: ", mean_vdd,
          " mV against the AC's 5141 mV at the same level", crlf);
    bench.verdict("and it agrees with the comparator's own answer within 5 % -\n"
                  "                two peripherals, no shared mechanism, one supply",
                  near(mean_vdd, supc_vdd_mv, supc_vdd_mv / 20u));
    vdd_mv = static_cast<uint16_t>(mean_vdd);

    // THE SAMPLING RULE, measured rather than trusted: the same channel
    // with the sampling time the table forbids.
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                     .output_enable = true});
    settle();
    Adc0::select(AdcInput::intref);
    const uint32_t long_sample = mean_of<Adc0>(64);
    bench.verdict("re-initialized with SAMPLEN 0 - below the 10 us table 45-22 "
                  "demands for this channel",
                  Adc0::enable(false) &&
                      adc_up<Adc0>(AdcConfig{.reference = Ref::vddana,
                                             .prescaler = AdcPresc::div32,
                                             .sample_length = 0}));
    Adc0::select(AdcInput::intref);
    settle();
    const uint32_t short_sample = mean_of<Adc0>(64);
    const uint32_t err = long_sample > short_sample ? long_sample - short_sample
                                                    : short_sample - long_sample;
    print(serial, "  INTREF at SAMPLEN 14 (10.0 us): ", long_sample,
          " counts; at SAMPLEN 0 (0.67 us): ", short_sample, " counts, ",
          (err * 1000u) / (long_sample == 0u ? 1u : long_sample),
          " per mille apart", crlf);
    // NO VERDICT ON THE SIGN OR THE SIZE of that difference: the rule is
    // the datasheet's and what a too-short sample costs is a silicon
    // measurement, printed and carried into docs/samc/adc.md. What IS
    // asserted is that the long sample is the one that lands where the
    // other two mechanisms say the bandgap is.
    bench.verdict("the 10 us sample is the one that agrees with the other two "
                  "witnesses (within 2 %)",
                  near((1024u * Adc0::result_steps()) /
                           (long_sample == 0u ? 1u : long_sample),
                       vdd_mv, vdd_mv / 50u));

    (void)Vref::configure(VrefConfig{});
    Adc0::release();
}

// =============================================================================
// d - one pad, two converters
// =============================================================================
void td_two_converters() {
    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
    };
    bench.verdict("both converters come up on the same configuration",
                  adc_up<Adc0>(cfg) && adc_up<Adc1>(cfg));
    bench.verdict("and both report the same full scale",
                  Adc0::result_steps() == Adc1::result_steps());

    struct Pair {
        const char* name;
        uint32_t a0;
        uint32_t a1;
    };
    Pair pairs[3]{};
    uint8_t np = 0;
    uint32_t worst = 0;

    drive<PadShared>(true);
    settle();
    Adc0::select(AnalogIn<PadShared>{});
    Adc1::select(AnalogIn<PadShared>{});
    pairs[np++] = Pair{"PA08 high", mean_of<Adc0>(64), mean_of<Adc1>(64)};

    drive<PadShared>(false);
    settle();
    pairs[np++] = Pair{"PA08 low ", mean_of<Adc0>(64), mean_of<Adc1>(64)};

    Adc0::select(AdcInput::scaled_supply);
    Adc1::select(AdcInput::scaled_supply);
    settle();
    pairs[np++] = Pair{"1/4 VDDANA", mean_of<Adc0>(64), mean_of<Adc1>(64)};

    for (uint8_t i = 0; i < np; ++i) {
        const uint32_t d =
            pairs[i].a0 > pairs[i].a1 ? pairs[i].a0 - pairs[i].a1 : pairs[i].a1 - pairs[i].a0;
        if (d > worst) {
            worst = d;
        }
        print(serial, "  ", pairs[i].name, ": ADC0 ", pairs[i].a0, ", ADC1 ",
              pairs[i].a1, ", apart ", d, " counts (",
              counts_mv<Adc0>(d, vdd_mv), " mV)", crlf);
    }
    bench.verdict("TWO CONVERTERS, ONE TRUTH: the worst disagreement over a\n"
                  "                rail, a ground and an internal divider is "
                  "under 1 % of full scale",
                  worst <= 41u);
    print(serial, "  worst disagreement ", worst, " counts of 4096 = ",
          (worst * 1000u) / 4096u, " per mille of full scale", crlf);

    // ERRATUM 1.4.10, tested rather than assumed: enabling ADC1 while
    // ADC0 is disabled may leave ADC0.SYNCBUSY.ENABLE stuck at one.
    Adc0::release();
    Adc1::release();
    bench.verdict("ADC1 alone comes up with ADC0 down", adc_up<Adc1>(cfg));
    const uint16_t adc0_sync = Adc0::sync_busy();
    print(serial, "  erratum 1.4.10: with ADC1 enabled and ADC0 not, "
          "ADC0.SYNCBUSY reads ", Hex{adc0_sync}, " (the item's ENABLE bit ",
          yes_no((adc0_sync & ADC_SYNCBUSY_ENABLE_Msk) != 0u), ")", crlf);
    bench.verdict("ERRATUM 1.4.10 DID NOT REPRODUCE on this silicon: ADC0's "
                  "ENABLE synchronization bit is CLEAR after ADC1 came up "
                  "alone, where the item says it may stick at one",
                  (adc0_sync & ADC_SYNCBUSY_ENABLE_Msk) == 0u);
    bench.verdict("and ADC1's own is clear too - which is the bit this "
                  "driver's enable() waits on, so the item can never hang it "
                  "whatever the other instance does",
                  (Adc1::sync_busy() & ADC_SYNCBUSY_ENABLE_Msk) == 0u);

    Adc1::release();
    drive<PadShared>(false);
}

// =============================================================================
// e - averaging and oversampling, and what they buy
// =============================================================================
//
// THE NOISE IS MEASURED BEFORE ANY CLAIM ABOUT IT IS MADE. Averaging can
// only be shown to reduce a spread that exists, so this letter first
// surveys what the three sources it can reach actually wander by, picks
// the noisiest as its measurand, and DECLINES the reduction verdict in
// print if even that one is quieter than the resolution of the question.
void te_averaging() {
    constexpr AdcConfig single{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
    };
    bench.verdict("ADC0 up, single 12-bit samples", adc_up<Adc0>(single));
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                     .output_enable = true});
    settle();

    struct Source {
        const char* name;
        AdcInput input;
    };
    const Source sources[] = {
        {"1/4 VDDANA ", AdcInput::scaled_supply},
        {"1/4 VDDCORE", AdcInput::scaled_core},
        {"INTREF     ", AdcInput::intref},
    };
    Spread survey[3]{};
    uint8_t noisiest = 0;
    for (uint8_t i = 0; i < 3u; ++i) {
        Adc0::select(sources[i].input);
        settle();
        survey[i] = spread_of<Adc0>(64);
        print(serial, "  ", sources[i].name, ": 64 single readings, mean ",
              survey[i].mean, ", ", survey[i].low, "..", survey[i].high,
              ", span ", survey[i].span(), " counts", crlf);
        if (survey[i].span() > survey[noisiest].span()) {
            noisiest = i;
        }
    }
    const Spread raw = survey[noisiest];
    print(serial, "  the noisiest of the three is ", sources[noisiest].name,
          " at ", raw.span(), " counts of 4096 - that is the measurand", crlf);
    bench.verdict("every internal source reads somewhere sane and none of them "
                  "is stuck",
                  survey[0].mean > 900u && survey[0].mean < 1150u &&
                      survey[1].mean > 150u && survey[1].mean < 400u &&
                      survey[2].mean > 400u && survey[2].mean < 1200u);

    // A TRUE AVERAGE: 64 samples, ADJRES from table 38-2, and the result
    // comes back at 12-bit precision so the two numbers are comparable
    // without any scaling at all.
    constexpr AdcAverage n = AdcAverage::samples64;
    constexpr AdcConfig averaged{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .resolution = AdcRes::bits16,
        .average = n,
        .adjust = adc_adjres_for_average(n),
        .sample_length = 5,
    };
    static_assert(adc_result_steps(averaged) == 4096,
                  "table 38-2: a true average comes back at 12 bits");
    bench.verdict("re-initialized to average 64 samples per result",
                  Adc0::enable(false) && adc_up<Adc0>(averaged));
    bench.verdict("and the driver's own arithmetic says the full scale is "
                  "unchanged at 4096 - which is what makes the two spreads "
                  "comparable",
                  Adc0::result_steps() == 4096u);
    Adc0::select(sources[noisiest].input);
    settle();
    const Spread avg = spread_of<Adc0>(16);
    print(serial, "  the same source, 16 averaged (64x) readings: mean ",
          avg.mean, ", ", avg.low, "..", avg.high, ", span ", avg.span(),
          " counts", crlf);
    bench.verdict("the averaged readings sit on the SAME value as the single "
                  "ones (within 4 counts)",
                  near(avg.mean, raw.mean, 4u));
    if (raw.span() >= 4u) {
        bench.verdict("AVERAGING BOUGHT WHAT IT PROMISES: the spread of 64x "
                      "averaged readings is smaller than that of single ones",
                      avg.span() < raw.span());
        print(serial, "  spread ", raw.span(), " -> ", avg.span(), " counts",
              crlf);
    } else {
        // THE HONEST OUTCOME on a board this quiet, and a verdict is
        // deliberately NOT taken: with a single-sample spread of one or
        // two counts, "the average is tighter" is a coin toss dressed as
        // a measurement. What CAN be asserted is that averaging did not
        // move the value, which the verdict above already says.
        print(serial, "  NO VERDICT ON THE NOISE REDUCTION: the noisiest source "
              "this board can offer spans only ", raw.span(),
              " counts of 4096 unaveraged (the averaged run spans ",
              avg.span(),
              "), which is at or below the quantization - too small a "
              "signal for a comparison to mean anything. A source that "
              "actually wanders needs a DAC or a wire, and this campaign "
              "has neither.", crlf);
    }

    // OVERSAMPLING AND DECIMATION is the OTHER column of the chapter: the
    // same 256 samples, but ADJRES chosen so the result KEEPS the extra
    // bits instead of dividing them away.
    constexpr AdcConfig oversampled{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .resolution = AdcRes::bits16,
        .average = adc_oversampling_average(4),
        .adjust = adc_adjres_for_oversampling(4),
        .sample_length = 5,
    };
    static_assert(adc_result_steps(oversampled) == 65536,
                  "table 38-3: 256 samples with ADJRES 0 is a 16-bit result");
    bench.verdict("re-initialized to oversample to 16 bits (256 samples, "
                  "ADJRES 0 - table 38-3's own row)",
                  Adc0::enable(false) && adc_up<Adc0>(oversampled));
    bench.verdict("the full scale is now 65536, and the driver says so before "
                  "any reading is taken",
                  Adc0::result_steps() == 65536u);
    Adc0::select(sources[noisiest].input);
    settle();
    const Spread over = spread_of<Adc0>(8);
    print(serial, "  8 oversampled (16-bit) readings: mean ", over.mean, ", ",
          over.low, "..", over.high, "; /16 = ", over.mean / 16u, crlf);
    bench.verdict("THE TWO SCALES AGREE: a 16-bit oversampled reading divided "
                  "by 16 lands on the 12-bit one (within 4 counts)",
                  near(over.mean / 16u, raw.mean, 4u));
    bench.verdict("and both come out as the same VOLTAGE through "
                  "util/analog.hpp, which is the whole point of result_steps()",
                  near(adc_mv(over.mean, 65536u, vdd_mv),
                       adc_mv(raw.mean, 4096u, vdd_mv), 4u));

    Adc0::release();
}

// =============================================================================
// f - conversion time against the formula, on the crystal
// =============================================================================
void tf_timing() {
    bench.verdict("the 24 MHz crystal starts and generator 2 carries it",
                  stopwatch_start());
    if (!on_crystal) {
        print(serial, "  no crystal: the rest of this letter would report OSC48M's "
              "own 5100 ppm error as the ADC's", crlf);
        Stopwatch::release();
        return;
    }
    print(serial, "  ruler: TC0+TC1 at ", stopwatch_hz,
          " Hz off the crystal; GCLK_ADC0 from the same crystal at ", crystal_hz,
          " Hz", crlf);

    // DIV64 puts CLK_ADC at 375 kHz, so one conversion is tens of
    // microseconds and the polling loop's own cost is under a per cent
    // of it. One CLK_ADC cycle is exactly eight stopwatch ticks.
    constexpr AdcPresc presc = AdcPresc::div64;
    constexpr uint32_t clk_adc = crystal_hz / 64u;               // 375 kHz
    constexpr uint32_t ticks_per_adc_cycle = stopwatch_hz / clk_adc;   // 8
    static_assert(ticks_per_adc_cycle * clk_adc == stopwatch_hz,
                  "the two rulers must divide exactly or the prediction is not "
                  "a prediction");

    struct Case {
        const char* name;
        AdcConfig cfg;
        uint16_t rounds;
    };
    // EVERY CASE IS FREE-RUNNING, which is what makes the window measure
    // the CONVERSION and not the software that asks for one - and it is
    // also the mode in which 38.6.2.14 charges the digital correction
    // only ONCE, so the config carries the flag and the prediction knows.
    const Case cases[] = {
        {"12-bit, SAMPLEN 0 ",
         AdcConfig{.reference = Ref::vddana, .prescaler = presc,
                   .free_running = true}, 200},
        {"12-bit, SAMPLEN 20",
         AdcConfig{.reference = Ref::vddana, .prescaler = presc,
                   .free_running = true, .sample_length = 20}, 200},
        {"8-bit,  SAMPLEN 0 ",
         AdcConfig{.reference = Ref::vddana, .prescaler = presc,
                   .resolution = AdcRes::bits8, .free_running = true}, 200},
        {"12-bit, OFFCOMP   ",
         AdcConfig{.reference = Ref::vddana, .prescaler = presc,
                   .free_running = true, .offset_compensation = true}, 200},
        {"12-bit, CORREN    ",
         AdcConfig{.reference = Ref::vddana, .prescaler = presc,
                   .free_running = true, .correction = true}, 200},
        {"16x average       ",
         AdcConfig{.reference = Ref::vddana, .prescaler = presc,
                   .resolution = AdcRes::bits16, .free_running = true,
                   .average = AdcAverage::samples16,
                   .adjust = adc_adjres_for_average(AdcAverage::samples16)}, 32},
    };

    bool all_ok = true;
    uint32_t worst_ppm = 0;
    for (const Case& c : cases) {
        if (!Adc0::init(gen_crystal, c.cfg, crystal_hz)) {
            bench.verdict("a timing configuration failed to come up", false);
            all_ok = false;
            continue;
        }
        Adc0::select(AdcInput::scaled_supply);
        Adc0::start();
        Adc0::discard(4);
        Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
        while (!Adc0::ready()) {
        }
        (void)Adc0::result();
        const uint32_t t0 = ticks_now();
        for (uint16_t i = 0; i < c.rounds; ++i) {
            while (!Adc0::ready()) {
            }
            (void)Adc0::result();
        }
        const uint32_t elapsed = ticks_now() - t0;
        const uint32_t measured = (elapsed + c.rounds / 2u) / c.rounds;
        const uint32_t predicted = adc_conversion_cycles(c.cfg) * ticks_per_adc_cycle;
        const uint32_t err = measured > predicted ? measured - predicted
                                                  : predicted - measured;
        const uint32_t ppm_off = predicted == 0u ? 0u : (err * 1000u) / predicted;
        if (ppm_off > worst_ppm) {
            worst_ppm = ppm_off;
        }
        print(serial, "  ", c.name, ": ", adc_conversion_cycles(c.cfg),
              " CLK_ADC cycles predicted -> ", predicted, " ticks, measured ",
              measured, " (", ppm_off, " per mille off, ",
              (measured * 1000u) / (stopwatch_hz / 1'000'000u), " ns)", crlf);
        if (err > predicted / 20u + 2u) {
            all_ok = false;
        }
        Adc0::release();
    }
    bench.verdict("EVERY FREE-RUNNING CONFIGURATION LANDS WITHIN 5 % OF TABLE "
                  "45-22'S OWN CYCLE COUNTS - the sampling length, the "
                  "resolution, the offset compensation's fixed four cycles and "
                  "the accumulation's multiplication, all of them",
                  all_ok);
    print(serial, "  worst departure ", worst_ppm, " per mille", crlf);

    // THE DIGITAL CORRECTION, both what it DOES and what it COSTS.
    //
    // 38.6.2.14 charges its 13 cycles ONCE in free-running mode - which
    // the table above has just confirmed, a corrected stream running at
    // the uncorrected 13 cycles - and PER CONVERSION in single mode. The
    // same sentence then qualifies that in a way nothing else in the
    // chapter explains: "Conversion time will be increased by 13 cycles
    // ACCORDING TO THE VALUE in the Offset Correction Value bit group",
    // which reads as if a zero offset might cost nothing. So the
    // measurement is made three ways, and the correction is first proved
    // to be DOING something before its cost is discussed at all.
    struct Corrected {
        uint32_t ticks;
        uint32_t counts;
    };
    auto single_shot = [](bool corrected, int16_t offset) -> Corrected {
        const AdcConfig cfg{.reference = Ref::vddana, .prescaler = presc,
                            .correction = corrected,
                            .offset_correction = offset};
        if (!Adc0::init(gen_crystal, cfg, crystal_hz)) {
            return Corrected{0, 0};
        }
        Adc0::select(AdcInput::scaled_supply);
        Adc0::discard(4);
        const uint16_t rounds = 200;
        uint32_t sum = 0;
        const uint32_t t0 = ticks_now();
        for (uint16_t i = 0; i < rounds; ++i) {
            Adc0::clear_flags(Adc0::flag_resrdy);
            Adc0::start();
            while (!Adc0::ready()) {
            }
            sum += Adc0::result();
        }
        const uint32_t elapsed = ticks_now() - t0;
        Adc0::release();
        return Corrected{(elapsed + rounds / 2u) / rounds,
                         (sum + rounds / 2u) / rounds};
    };
    const Corrected plain = single_shot(false, 0);
    const Corrected zero_offset = single_shot(true, 0);
    const Corrected with_offset = single_shot(true, 100);
    print(serial, "  single conversions, CORREN off:            ", plain.ticks,
          " ticks, ", plain.counts, " counts", crlf);
    print(serial, "  single conversions, CORREN + OFFSETCORR 0: ",
          zero_offset.ticks, " ticks, ", zero_offset.counts, " counts", crlf);
    print(serial, "  single conversions, CORREN + OFFSETCORR 100: ",
          with_offset.ticks, " ticks, ", with_offset.counts, " counts", crlf);

    // First: is the correction actually in the path? OFFSETCORR is
    // SUBTRACTED from the conversion before RESULT (38.6.2.14's own
    // formula), so a hundred counts of offset must cost a hundred counts
    // of reading. That verdict is what makes the timing one meaningful.
    const uint32_t dropped = zero_offset.counts > with_offset.counts
                                 ? zero_offset.counts - with_offset.counts
                                 : 0u;
    bench.verdict("THE CORRECTION IS REALLY IN THE PATH: OFFSETCORR 100 takes "
                  "a hundred counts off the reading, which is 38.6.2.14's own "
                  "subtraction",
                  near(dropped, 100u, 4u));
    bench.verdict("and a zero offset with CORREN on changes nothing",
                  near(zero_offset.counts, plain.counts, 4u));

    // Now the cost. The single-shot numbers carry the polling loop's own
    // overhead, so only the DIFFERENCES between them mean anything.
    const uint32_t cost_zero = zero_offset.ticks > plain.ticks
                                   ? zero_offset.ticks - plain.ticks : 0u;
    const uint32_t cost_offset = with_offset.ticks > plain.ticks
                                     ? with_offset.ticks - plain.ticks : 0u;
    print(serial, "  the correction's cost in single mode: ", cost_zero,
          " ticks at OFFSETCORR 0 and ", cost_offset,
          " at OFFSETCORR 100, where 38.6.2.14's 13 CLK_ADC cycles would be ",
          13u * ticks_per_adc_cycle, " ticks", crlf);
    bench.verdict("38.6.2.14'S PER-CONVERSION 13 CYCLES ARE NOT OBSERVED - a "
                  "corrected single conversion costs under two CLK_ADC cycles "
                  "more than an uncorrected one, offset or no offset, where "
                  "the sentence predicts thirteen",
                  cost_zero < 2u * ticks_per_adc_cycle &&
                      cost_offset < 2u * ticks_per_adc_cycle);
    // NO VERDICT on which reading of that sentence is right: the chapter
    // says free-running pays once (confirmed) and single mode pays every
    // time (not seen at either offset), and there is nothing further this
    // board can ask. adc_conversion_cycles() keeps charging the 13 in
    // single mode, because a pacing prediction that is too GENEROUS is
    // safe and one that is too tight is not - docs/samc/adc.md says so.

    Stopwatch::release();
    GclkChannel::disconnect(Evsys::gclk_id(0));
}

// =============================================================================
// g - the window monitor, and MODE4's documented ambiguity
// =============================================================================
void tg_window() {
    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
    };
    bench.verdict("ADC0 up with the window disabled", adc_up<Adc0>(cfg));
    Adc0::select(AnalogIn<PadShared>{});

    // Reading RESULT clears WINMON as well as RESRDY (38.8.7), so
    // window_hit() reports the verdict for the value just read - the
    // shape avrdx/adc.hpp has for the same reason.
    auto hit_at = [](bool high) {
        drive<PadShared>(high);
        settle();
        Adc0::discard(2);
        (void)Adc0::read();
        return Adc0::window_hit();
    };

    bench.verdict("MODE1 (RESULT > WINLT) with the threshold at mid-scale",
                  Adc0::window(AdcWindow::above_lower, 2048, 4095));
    const bool m1_high = hit_at(true);
    const bool m1_low = hit_at(false);
    bench.verdict("fires on the high rail and stays silent on the low one",
                  m1_high && !m1_low);

    bench.verdict("MODE2 (RESULT < WINUT), same threshold",
                  Adc0::window(AdcWindow::below_upper, 0, 2048));
    const bool m2_high = hit_at(true);
    const bool m2_low = hit_at(false);
    bench.verdict("fires on the low rail and stays silent on the high one",
                  !m2_high && m2_low);

    bench.verdict("MODE3 (WINLT < RESULT < WINUT), a band this board cannot "
                  "reach with a pad",
                  Adc0::window(AdcWindow::inside, 1000, 3000));
    const bool m3_high = hit_at(true);
    const bool m3_low = hit_at(false);
    bench.verdict("stays silent at BOTH rails - the band is empty of them",
                  !m3_high && !m3_low);

    // THE DISAGREEMENT. 38.8.10 prints MODE4 as "WINUT < RESULT < WINLT",
    // which with WINLT = 1000 < WINUT = 3000 is an EMPTY band and can
    // never fire. The device header's own comment on the same value
    // reads "!(WINLT < RESULT < WINUT)", which is the complement of
    // MODE3 and must fire at BOTH rails. One reading, one experiment.
    bench.verdict("MODE4 with the SAME thresholds MODE3 just used",
                  Adc0::window(AdcWindow::outside, 1000, 3000));
    const bool m4_high = hit_at(true);
    const bool m4_low = hit_at(false);
    print(serial, "  MODE4 with WINLT 1000 < WINUT 3000: high rail ",
          yes_no(m4_high), ", low rail ", yes_no(m4_low),
          " -- the datasheet's 'WINUT < RESULT < WINLT' predicts no/no, the "
          "device header's '!(WINLT < RESULT < WINUT)' predicts yes/yes", crlf);
    bench.verdict("and the silicon answers unambiguously (both rails alike, "
                  "which is what makes this a decidable question)",
                  m4_high == m4_low);
    bench.verdict("MODE4 IS THE COMPLEMENT OF MODE3, as the device header says "
                  "and 38.8.10's table does not",
                  m4_high && m4_low);

    bench.verdict("the window turns off and the flag stops following",
                  Adc0::window_off() && Adc0::window_mode() == AdcWindow::none);
    (void)hit_at(true);
    bench.verdict("no hit with the monitor disabled", !Adc0::window_hit());

    // 38.6.2.13's other clause: only the bits the resolution carries are
    // significant, so the same thresholds mean different things at 8 bits.
    bench.verdict("re-initialized at 8-bit resolution with MODE1 at 128",
                  Adc0::enable(false) &&
                      adc_up<Adc0>(AdcConfig{.reference = Ref::vddana,
                                             .prescaler = AdcPresc::div32,
                                             .resolution = AdcRes::bits8,
                                             .window = AdcWindow::above_lower,
                                             .sample_length = 5,
                                             .window_low = 128,
                                             .window_high = 255}));
    Adc0::select(AnalogIn<PadShared>{});
    const bool b8_high = hit_at(true);
    const bool b8_low = hit_at(false);
    bench.verdict("the thresholds follow the resolution: full scale is 256 "
                  "here and the mid-point is 128",
                  Adc0::result_steps() == 256u && b8_high && !b8_low);

    drive<PadShared>(false);
    Adc0::release();
}

// =============================================================================
// h - the no-CPU chain
// =============================================================================
//
// THIS STRATUM'S SIGNATURE MOVE, and here it runs in both directions at
// once: a TC overflow crosses an ASYNCHRONOUS event channel into the
// ADC's START user, the conversion's RESRDY pulls a DMA beat out of
// RESULT, and the SAME RESRDY crosses a second channel into a second
// timer that counts them. The CPU touches nothing between arming it and
// reading the answer.
void th_no_cpu() {
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the event channels' clock is routed",
                  EvGen::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_start_channel), ev_gen) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_result_channel), ev_gen));

    // The pacer: TC2 overflowing at about 1 kHz, its overflow an event.
    // OSC48M / 1024 / 46 is close enough - the rate is not the measurand.
    bench.verdict("TC2 paces, and its overflow becomes an event",
                  Pacer::init(adc_gen) &&
                      Pacer::configure(TcConfig{.mode = TcMode::count8,
                                                .prescaler = TcPrescaler::div1024,
                                                .waveform = TcWaveform::normal_pwm}) &&
                      Pacer::set_period8(46) &&
                      Pacer::event_config(TcConfig{.mode = TcMode::count8,
                                                   .prescaler = TcPrescaler::div1024,
                                                   .waveform = TcWaveform::normal_pwm},
                                          TcEventConfig{.overflow_out = true}));

    // The counter: TC3 counting whatever arrives on its event input.
    bench.verdict("TC3 is set up to COUNT events rather than clock ticks",
                  Counter::init(adc_gen) &&
                      Counter::configure(TcConfig{.mode = TcMode::count16}) &&
                      Counter::event_config(TcConfig{.mode = TcMode::count16},
                                            TcEventConfig{
                                                .action = TcEventAction::count,
                                                .input_enable = true}));

    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
        .events = {.result_out = true},
    };
    bench.verdict("ADC0 comes up with its RESULT-READY event enabled",
                  adc_up<Adc0>(cfg));
    Adc0::select(AnalogIn<PadShared>{});

    // start_on() needs the converter disabled - EVCTRL is
    // enable-protected - and refuses anything but the asynchronous path.
    bench.verdict("the pacer's overflow reaches the ADC's START user, on the "
                  "ASYNCHRONOUS path erratum 1.4.4 and table 29-3 both demand",
                  Adc0::enable(false) &&
                      Adc0::start_on(ev_start_channel,
                                     EventChannelConfig{
                                         .generator = Pacer::overflow_generator,
                                         .path = EventPath::asynchronous}));
    bench.verdict("and the ADC's own RESRDY reaches TC3's event input",
                  Evsys::connect(Counter::event_user, ev_result_channel,
                                 EventChannelConfig{
                                     .generator = Adc0::resrdy_generator,
                                     .path = EventPath::asynchronous}));

    auto run = [](bool high) -> uint32_t {
        for (uint16_t i = 0; i < dma_results; ++i) {
            results[i] = 0xFFFFu;
        }
        drive<PadShared>(high);
        settle();
        // THE DMA REQUEST IS THE RESRDY FLAG, and 38.6.4 says it is
        // "cleared when the RESULT register is read" - so a result left
        // standing from the previous run would move one stale beat the
        // moment the channel is enabled. Read it away first; clearing
        // the flag alone is not the same thing.
        (void)Adc0::result();
        Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
        (void)Copy::reset();
        const DmaChannelConfig ch{
            .trigger = Adc0::dma_trigger_resrdy,
            .action = DmaTriggerAction::beat,
        };
        (void)Copy::configure(ch);
        const DmaTransfer t{
            .source = &Adc0::regs().ADC_RESULT,
            .destination = &results[0],
            .beats = dma_results,
            .beat = DmaBeat::hword,
            .source_increment = false,
        };
        (void)Copy::load(t);
        (void)Copy::enable(true);
        (void)Counter::enable(true);
        (void)Counter::set_count16(0);
        (void)Adc0::enable(true);
        (void)Pacer::enable(true);
        wait_ms(60);
        (void)Pacer::enable(false);
        (void)Adc0::enable(false);
        // COUNT IS READ WHILE THE TIMER STILL RUNS: the read needs a
        // READSYNC command, and a command written to a stopped counter
        // has no clock domain to cross into.
        const uint32_t counted = Counter::count16();
        (void)Counter::enable(false);
        (void)Copy::enable(false);
        return counted;
    };

    const uint32_t counted_high = run(true);
    uint16_t filled_high = 0;
    uint16_t high_ok = 0;
    for (uint16_t i = 0; i < dma_results; ++i) {
        const uint16_t v = results[i];
        if (v != 0xFFFFu) {
            ++filled_high;
            if (v >= 4055u) {
                ++high_ok;
            }
        }
    }
    print(serial, "  pad high: ", filled_high, " of ", dma_results,
          " results moved by the DMAC with no CPU in the path, ", high_ok,
          " of them at full scale; TC3 counted ", counted_high,
          " result-ready events", crlf);
    bench.verdict("THE DMAC FILLED THE BUFFER FROM RESULT, one beat per "
                  "conversion, with the CPU asleep in a wait loop",
                  filled_high == dma_results);
    bench.verdict("and every one of them is the rail the pad was holding",
                  high_ok == dma_results);
    bench.verdict("THE ADC IS A GENERATOR TOO: TC3 counted at least the "
                  "conversions the DMAC took",
                  counted_high >= dma_results);

    const uint32_t counted_low = run(false);
    uint16_t filled_low = 0;
    uint16_t low_ok = 0;
    for (uint16_t i = 0; i < dma_results; ++i) {
        const uint16_t v = results[i];
        if (v != 0xFFFFu) {
            ++filled_low;
            if (v <= 41u) {
                ++low_ok;
            }
        }
    }
    print(serial, "  pad low:  ", filled_low, " of ", dma_results,
          " moved, ", low_ok, " of them at zero; TC3 counted ", counted_low,
          crlf);
    bench.verdict("the same chain follows the pad to the other rail",
                  filled_low == dma_results && low_ok == dma_results);

    // A control: with the pacer stopped, nothing moves at all.
    for (uint16_t i = 0; i < dma_results; ++i) {
        results[i] = 0xFFFFu;
    }
    (void)Adc0::result();
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    (void)Copy::enable(true);
    (void)Adc0::enable(true);
    wait_ms(30);
    (void)Adc0::enable(false);
    (void)Copy::enable(false);
    bool untouched = true;
    for (uint16_t i = 0; i < dma_results; ++i) {
        if (results[i] != 0xFFFFu) {
            untouched = false;
        }
    }
    bench.verdict("with the pacer stopped NOTHING moves - the event really is "
                  "the only thing starting a conversion",
                  untouched);

    Evsys::disconnect(Counter::event_user);
    (void)Adc0::stop_events();
    Adc0::release();
    Pacer::release();
    Counter::release();
    GclkChannel::disconnect(Evsys::gclk_id(ev_start_channel));
    GclkChannel::disconnect(Evsys::gclk_id(ev_result_channel));
    drive<PadShared>(false);
}

// =============================================================================
// i - util/analog_sampler.hpp inside a real kernel
// =============================================================================
//
// THE CAMPAIGN'S POINT, not a bonus letter. util/analog_sampler.hpp was
// designed on the AVR around a converter that reports ONE result per
// interrupt and reads back which input it was taken on. Its own file
// comment doubted the shape would survive on a target with a hardware
// sequencer and DMA. It survives: the concept is satisfied by Adc<0> as
// written, the walk works, and NOT ONE LINE OF util/ CHANGED.

struct Collector;
using Subs = Subscribers<Collector>;
using Sampler = AnalogSampler<Adc0, SamPlatform, Subs, AdcInput::scaled_supply,
                              AnalogIn<PadShared>{}>;

struct Collector {
    using Event = std::variant<AnalogSample>;
    static inline EventQueue<Event, 8, SamPlatform> queue;

    static inline uint16_t samples = 0;
    static inline uint16_t per_index[2] = {0, 0};
    static inline uint16_t last[2] = {0, 0};

    static void init() {
        samples = 0;
        per_index[0] = 0;
        per_index[1] = 0;
        last[0] = 0;
        last[1] = 0;
    }
    static void dispatch(const Event& e) {
        match(e, [](AnalogSample s) {
            if (s.index < 2u) {
                ++per_index[s.index];
                last[s.index] = s.value;
            }
            if (samples != UINT16_MAX) {
                ++samples;
            }
        });
    }
};

using SamplerKernel = Kernel<SamPlatform, Collector, Sampler>;

volatile uint16_t adc_interrupts = 0;

void ti_sampler_ao() {
    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .sample_length = 5,
    };
    bench.verdict("the converter is configured by the OWNER, before the "
                  "sampler starts - the sampler never reconfigures",
                  adc_up<Adc0>(cfg));
    bench.verdict("Adc<0> satisfies util/analog_sampler.hpp's converter "
                  "concept as written, and both inputs satisfy the input one",
                  AnalogConverter<Adc0> && SamplerInput<Adc0, AdcInput::scaled_supply> &&
                      SamplerInput<Adc0, AnalogIn<PadShared>{}>);

    drive<PadShared>(true);
    settle();
    adc_interrupts = 0;
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    Adc0::arm(Adc0::flag_resrdy);
    Nvic::enable(Adc0::irq());

    SamplerKernel::init_all();
    Sampler::start_every(Ticker::ticks_per_second / 50u);   // 20 ms

    const uint32_t started = Ticker::millis();
    while (Ticker::millis() - started < 1000UL) {
        TimeEvents<SamPlatform>::process();
        while (SamplerKernel::step()) {
        }
    }
    Sampler::stop();
    Nvic::disable(Adc0::irq());
    Adc0::disarm(Adc0::flag_resrdy);

    print(serial, "  one second at 50 Hz: ", adc_interrupts,
          " conversion interrupts, ", Collector::samples,
          " AnalogSample events received (", Collector::per_index[0],
          " on the scaled supply, ", Collector::per_index[1], " on PA08); "
          "unknown inputs ", Sampler::unknown_inputs(), crlf);
    print(serial, "  last values: scaled supply ", Collector::last[0],
          " counts (", counts_mv<Adc0>(Collector::last[0], vdd_mv),
          " mV), PA08 ", Collector::last[1], " counts (",
          counts_mv<Adc0>(Collector::last[1], vdd_mv), " mV)", crlf);

    bench.verdict("the conversion interrupt ran at the sampler's pace",
                  adc_interrupts >= 40u && adc_interrupts <= 60u);
    bench.verdict("EVERY RESULT REACHED THE SUBSCRIBER THROUGH THE KERNEL",
                  Collector::samples == adc_interrupts);
    bench.verdict("the sampler WALKED its list - both inputs, alternating",
                  Collector::per_index[0] >= 18u && Collector::per_index[1] >= 18u &&
                      near(Collector::per_index[0], Collector::per_index[1], 1u));
    bench.verdict("and never mislabelled one: no result arrived with an input "
                  "code outside the list",
                  Sampler::unknown_inputs() == 0u);
    bench.verdict("the values are attributed to the right inputs - the pad at "
                  "its rail, the scaled supply at a quarter",
                  Collector::last[1] >= 4055u && near(Collector::last[0], 1024u, 41u));

    drive<PadShared>(false);
    Adc0::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_adc - SAMC21J18A ADC (ch. 38): both converters, the "
          "references, averaging, the window, both event directions and the "
          "sampler AO, wireless, clk=",
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

/// The ISR glue util/analog_sampler.hpp's file comment describes, in the
/// application where a vector name is allowed to appear: the selected
/// input is read WITH the value, so attribution never depends on the
/// sampler's dispatch being on time.
extern "C" void ADC0_Handler() {
    const uint8_t pending = brio::Adc<0>::isr();
    if ((pending & brio::Adc<0>::flag_resrdy) != 0u) {
        const uint8_t input = brio::Adc<0>::selected();
        const uint16_t value = brio::Adc<0>::resrdy();
        const uint16_t seen = adc_interrupts;
        if (seen != UINT16_MAX) {
            adc_interrupts = static_cast<uint16_t>(seen + 1u);
        }
        brio::post<Sampler>(brio::Sampled{value, input});
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

    bench.letter('a', "the block, its vocabularies, the calibration, the refusals",
                 ta_block);
    bench.letter('b', "the rails and the internal supply channels", tb_rails);
    bench.letter('c', "INTREF, VREFOE and where VDD really is", tc_bandgap);
    bench.letter('d', "one pad, two converters", td_two_converters);
    bench.letter('e', "averaging and oversampling, and what they buy",
                 te_averaging);
    bench.letter('f', "conversion time against the formula, on the crystal",
                 tf_timing);
    bench.letter('g', "the window monitor, and MODE4's ambiguity settled",
                 tg_window);
    bench.letter('h', "the no-CPU chain: event in, DMA out, events out",
                 th_no_cpu);
    bench.letter('i', "AnalogSampler inside a real kernel", ti_sampler_ao);

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
