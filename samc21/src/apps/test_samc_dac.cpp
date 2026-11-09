// test_samc_dac - the reference bench suite for samc21/dac.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, AND THIS CHAPTER IS WHY. On this package PA02 is the
// DAC's VOUT pad, ADC0's AIN0 and the AC's AIN4 all at once, so the
// "wire the DAC output to an ADC input" that erratum 1.8.9 asks for as
// a workaround is already on the die - a wire of zero length. The DAC
// is therefore this board's swept voltage source, and it is what makes
// three enumerators in other drivers answerable at all:
//   samc21/adc.hpp   AdcInput::dac  and  Ref::dac
//   samc21/ac.hpp    AcNegative::dac
//   samc21/adc.hpp   Ref::intref as a REFERENCE, and whether it needs
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
//   j  the event-paced DAC: an event starts each conversion, the DAC's
//      own EMPTY interrupt feeds DATABUF from a table
//   k  erratum 1.9.2: the EMPTY flag across a standby
//   l  EMPTY as a wake from STANDBY, a timer's event starting the
//      conversion; RUNSTDBY clear the control
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc21/ac.hpp"
#include "samc21/adc.hpp"
#include "samc21/clock.hpp"
#include "samc21/dac.hpp"
#include "samc21/evsys.hpp"
#include "samc21/nvic.hpp"
#include "samc21/osc32kctrl.hpp"
#include "samc21/pin.hpp"
#include "samc21/reset.hpp"
#include "samc21/rtc.hpp"
#include "samc21/sercom.hpp"
#include "samc21/sleep.hpp"
#include "samc21/supc.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/analog.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
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

/// The SysTick ticker read as CPU cycles: the ruler of this suite's
/// waits (util/test_wait.hpp).
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};

/// Every queued console byte off the wire, bounded (util/test_wait.hpp):
/// false on a wedged console.
bool console_drain() { return brio::console_drain<Ruler, Serial>(); }

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

/// Where this board's supply sits, in millivolts: 0 until locate_vdd()
/// has read it, once a boot, before the first verdict that rests on it.
/// The board takes 3.3 V or 5 V on a jumper (docs/boards/samc21j.md), so
/// no figure is assumed.
uint16_t vdd_mv = 0;

/// How far the located supply can sit from the true one, per mille. The
/// supply is the 1.024 V level read by ADC0 against VDDANA, so the true
/// supply is vdd_mv x (the level + ADC0's error) / 1024: the level
/// 1003..1045 mV (table 45-36, at 5.0 V) moved by the levels' supply
/// coefficient over 2.7..5.5 V (-0.2..+0.3 %/V, the table's typical, the
/// only figure it gives): 996..1050 mV, and ADC0's error reading it
/// (table 45-24's maxima: offset 80 mV, gain 42 mV x 0.39 - the level's
/// fraction of a 2.7 V supply - and INL 4 counts, 5.4 mV at 5.5 V: 102
/// mV) - a factor of 894/1024 .. 1152/1024.
constexpr uint32_t supply_lo_permille = 873;

/// The most counts of 4096 a voltage of `mv` (up to 1000) can be against
/// the true supply, given the located one (rounded up); full scale when
/// no supply has been located (nothing judged tight).
uint32_t located_counts(uint32_t mv) {
    if (vdd_mv == 0u) return 4096u;
    return (mv * 4096u * 1000u + vdd_mv * supply_lo_permille - 1u) /
           (vdd_mv * supply_lo_permille);
}

/// The DAC read by ADC0 against VDDANA: the pair's worst combined error at
/// a fraction f_num / f_den of full scale, in millivolts, from the
/// tables' maxima - the DAC (DS60001479M table 45-32) offset 20 mV and
/// gain 20 mV x f, ADC0 single-ended (table 45-24, the larger of its
/// 3.0 V and 5.0 V rows) offset 80 mV and gain 42 mV x f - and then in
/// counts of 4096 through located_counts(), with the INLs (the DAC's 0.9
/// LSB of 1024, 4 counts; the ADC's 4) and a count of quantization.
uint32_t pair_err_mv(uint32_t f_num, uint32_t f_den) {
    return 100u + (62u * f_num) / f_den;
}
uint32_t pair_err_counts(uint32_t f_num, uint32_t f_den) {
    if (vdd_mv == 0u) return 4096u;   // no supply located: nothing is judged tight
    return located_counts(pair_err_mv(f_num, f_den)) + 9u;
}

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
// The event fabric and the interrupt feed (letter j).
// ---------------------------------------------------------------------------
constexpr uint8_t ev_start_channel = 0;    // TC2 overflow -> DAC START
constexpr uint8_t ev_empty_channel = 1;    // DAC EMPTY    -> TC3 counts
constexpr uint8_t ev_gen = 6;
using EvGen = Gclk<ev_gen>;
using Pacer = Tc<2>;
using Counter = Tc<3>;

/// The table DAC_Handler plays into DATABUF, one value an EMPTY, and the
/// handler's place in it. VOLATILE because the handler and the letter
/// share them: the letter fills and arms, the handler advances.
constexpr uint16_t wave_len = 32;
volatile uint16_t wave[wave_len];
volatile uint16_t feed_next = 0;     ///< the index the next EMPTY writes
volatile uint16_t feed_left = 0;     ///< values still to write
volatile uint32_t feed_empties = 0;  ///< EMPTY interrupts taken

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Long enough for the DAC's own conversion (2.9 us at the 350 ksps
/// ceiling) and for the ADC's input to see the new level: 400 us.
void settle() { brio::spin_us<Ruler>(400); }

/// The longer pause after a block is set up or a reference changes: 4 ms.
void settle_long() { brio::spin_us<Ruler>(4'000); }

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

/**
 * VDDANA located, once a boot: the 1.024 V bandgap level (MUXPOS INTREF,
 * VREFOE set - the ADC's bandgap input channel is dead without it,
 * docs/samc21/adc.md) against VDDANA as the reference, at SAMPLEN 14 (the
 * 10 us table 45-22 asks of the channel). The level sits under every
 * supply the device runs on (2.7..5.5 V, DS60001479M table 45-2), so the
 * reading never saturates; its accuracy is the level's and ADC0's, as
 * supply_lo_permille spells out. Leaves ADC0 released and SUPC.VREF at
 * reset.
 */
bool locate_vdd() {
    if (vdd_mv != 0u) {
        return true;
    }
    if (!adc0_up(AdcConfig{.reference = Ref::vddana,
                           .prescaler = AdcPresc::div32,
                           .sample_length = 14})) {
        return false;
    }
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024, .output_enable = true});
    Adc0::select(AdcInput::intref);
    settle_long();
    const uint32_t counts = mean_of<Adc0>(32);
    (void)Vref::configure(VrefConfig{});
    Adc0::release();
    // What the level can read under any supply the device runs on: the
    // level's 1003..1045 mV (table 45-36) with ADC0's offset and gain (80
    // mV and 42 mV x the fraction, table 45-24), INL and quantization
    // against 2.7..5.5 V (table 45-2) is 676..1737 counts. Outside that no
    // level was read, and a smaller count would overflow the millivolts.
    if (counts < 676u || counts > 1737u) {
        return false;
    }
    vdd_mv = static_cast<uint16_t>((1024UL * 4096UL + counts / 2u) / counts);
    return true;
}

/// locate_vdd() for a letter whose verdicts rest on the supply: a failure
/// is the letter's verdict.
bool need_vdd() {
    const bool ok = locate_vdd();
    if (!ok) {
        bench.verdict("VDDANA located through the 1.024 V bandgap level", false);
    }
    return ok;
}

/// Whether a bandgap level of `level_mv` can be produced and read under a
/// supply located at `supply_mv`: the level at its top (+2.05 %, table
/// 45-36, and 0.69 % of typical supply coefficient: x 1.028) with ADC0's
/// offset and gain at full scale over it (122 mV, table 45-24, which
/// covers the DAC's offsets too), under the least the true supply can be
/// (supply_lo_permille).
constexpr bool level_below_supply(uint32_t level_mv, uint32_t supply_mv) {
    return level_mv * 1028u + 122'000u < supply_mv * supply_lo_permille;
}

/// Letter i's wait on a fast analog edge, at the cost of the condition
/// alone for its first 64 reads - every edge the letter times arrives
/// inside them, a few hundred nanoseconds - and only then on the ruler,
/// whose read (about 1.5 us, util/test_wait.hpp) would otherwise sit in
/// the waiting loop and not in its control: false after 10 ms.
template <typename F>
bool fast_until(F done) {
    for (uint8_t k = 0; k < 64u; ++k) {
        if (done()) {
            return true;
        }
    }
    return brio::wait_until<Ruler>(done, 10'000);
}

/// Before a timed window: the console drained, or a failed verdict and
/// false - a line still going out puts its interrupt inside the window.
bool window_ready() {
    const bool drained = console_drain();
    if (!drained) {
        bench.verdict("the console drained before the timed window", false);
    }
    return drained;
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
          " (asynchronous path only); GCLK id ", Dac::gclk_id, crlf);
    bench.verdict("the generator, user and clock codes are the header's own",
                  Dac::empty_generator == 79u && Dac::start_event_user == 38u &&
                      Dac::gclk_id == 36u);

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

    // The write-only data path: DATA and DATABUF are write-only in both
    // documents, so code() is the driver's own record of what was written
    // and there is nothing on the silicon to judge it against.

    Dac::release();
    bench.verdict("release() puts the block back: the DAC's APB clock off "
                  "and its generic clock channel disconnected",
                  (MCLK_REGS->MCLK_APBCMASK & MCLK_APBCMASK_DAC_Msk) == 0u);
}

// =============================================================================
// b - THE LOOP WITH NO WIRE
// =============================================================================
void tb_loop() {
    if (!need_vdd()) {
        return;
    }
    bench.verdict("the DAC comes up with the external buffer driving PA02",
                  dac_up());
    bench.verdict("ADC0 comes up", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});

    // FIRST: does the analog output need the pad's peripheral function?
    // An ADC INPUT does not (the connection is direct, which is what
    // makes a wireless analog test possible at all); an OUTPUT has a
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
    // THE NOISE FLOOR, measured before any band below is chosen.
    dac_set(512);
    const Spread mid = spread_of<Adc0>(64);
    print(serial, "  64 single readings at mid-code span ", mid.span(),
          " counts of 4096 around ", mid.mean, crlf);
    const uint16_t band = static_cast<uint16_t>(mid.span() + 8u);
    // The two readings are one code through one converter: they differ by
    // the noise alone (the 64 readings' span, measured above) and a count;
    // either is the pair at mid-scale, within the tables' error.
    const uint32_t mid_err = pair_err_counts(1u, 2u);
    bench.verdict("THE ANALOG OUTPUT REACHES THE PAD EITHER WAY - the buffer "
                  "is connected by CTRLB.EOEN and not by the pin multiplexer",
                  near(no_pmux, with_pmux, mid.span() + 1u) &&
                      near(no_pmux, 2048u, mid_err));

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

    // The reference here is VDDANA, so the top code says nothing about the
    // supply, and neither does the scaled-supply channel - a quarter of
    // VDD measured against VDD is a self-consistency check, not a supply
    // meter. The supply is the bandgap's figure, located before the letter.
    Adc0::select(AdcInput::scaled_supply);
    const uint32_t quarter = mean_of<Adc0>(32);
    print(serial, "  1/4 VDDANA reads ", quarter, " counts (a quarter of full "
          "scale would be 1024); VDDANA located at ", vdd_mv, " mV through the "
          "1.024 V bandgap level", crlf);
    // ADC0's own error at a quarter of full scale is 80 + 10.5 mV (table
    // 45-24), some 75 counts at 5 V; the divider's is not tabled. +-10 %
    // holds both, and the nearest other reference (INTREF at 4.096 V) would
    // read 27 % high, the lower levels saturate.
    bench.verdict("the scaled supply is a quarter of full scale within 10 %, so the "
                  "reference really is VDDANA",
                  near(quarter, 1024u, 102u));

    Adc0::select(AnalogIn<Vout>{});
    // The ends lie outside table 45-30's linear range by its 50 mV, plus
    // the pair's error there; the inner codes are the pair's error alone.
    const uint32_t lin_counts = located_counts(50u);
    bench.verdict("code 0 sits at the bottom of the range, within the linear range's "
                  "50 mV and the pair's error of ground",
                  at_0 <= lin_counts + pair_err_counts(0u, 1u));
    bench.verdict("code 1023 sits at the top, within the linear range's 50 mV and the "
                  "pair's error of full scale",
                  at_1023 + lin_counts + pair_err_counts(1u, 1u) >= 4096u);
    bench.verdict("mid-code is half of full scale within the two converters' "
                  "combined offset and gain error (tables 45-24, 45-32)",
                  near(at_512, 2048u, mid_err));
    bench.verdict("a quarter and three quarters land where they should",
                  near(at_256, 1024u, pair_err_counts(1u, 4u)) &&
                      near(at_768, 3072u, pair_err_counts(3u, 4u)));

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

    // set_mv(), whose arithmetic is util/analog.hpp's.
    (void)Dac::set_mv(2000, vdd_mv);
    settle();
    const uint32_t two_volts = mean_of<Adc0>(16);
    const uint16_t two_volts_mv =
        adc_mv(two_volts, Adc0::result_steps(), vdd_mv);
    print(serial, "  set_mv(2000) wrote code ", Dac::code(), " and the ADC "
          "reads ", two_volts_mv, " mV", crlf);
    // The pair's error at 2 V of the supply, in millivolts, and the two
    // conversions' quantization (a DAC step, an ADC count). The located
    // supply cancels from the code written and the millivolts read back
    // but for the pair's error, which comes back scaled by the located
    // supply over the true one: up to 1000/873.
    const uint32_t two_err = pair_err_mv(2000u, vdd_mv) * 1000u / supply_lo_permille + 1u +
                             (vdd_mv + 1023u) / 1024u + (9u * vdd_mv + 4095u) / 4096u;
    bench.verdict("util/analog.hpp's dac_code() and adc_mv() agree across the "
                  "silicon within the pair's tabled error, with NOT ONE LINE of "
                  "util/ changed for this target",
                  near(two_volts_mv, 2000u, two_err));

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
    if (!need_vdd()) {
        return;
    }
    bench.verdict("ADC0 comes up on VDDANA", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});

    // A level is produced only where the supply holds it: the DAC's output
    // cannot rise above VDDANA, so at 3.3 V the 4.096 V level is skipped.
    const VrefLevel levels[3] = {VrefLevel::v1_024, VrefLevel::v2_048,
                                 VrefLevel::v4_096};
    const uint16_t nominal[3] = {1024, 2048, 4096};
    uint16_t measured[3] = {0, 0, 0};
    bool held[3] = {false, false, false};
    bool all_ok = true;

    for (uint8_t i = 0; i < 3u; ++i) {
        held[i] = level_below_supply(nominal[i], vdd_mv);
        if (!held[i]) {
            print(serial, "  SUPC.VREF.SEL = ", nominal[i], " mV not produced: "
                  "the supply, at ", vdd_mv, " mV, does not hold it", crlf);
            bench.skip("a bandgap level the supply does not hold");
            continue;
        }
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
                      (!held[2] || measured[2] > measured[1] + 1500u));
    // Each level within its tables. The figure is 1.023 x (the DAC's
    // output at code 1000 + ADC0's error) x 1024 / (the 1.024 V level +
    // ADC0's error reading it), so the supply cancels and the two ADC0
    // errors meet: their gains cancel (each in proportion to its voltage)
    // and their common offset is left weighed by 1.023 - L/1024 (80 mV,
    // table 45-24). With it: the level and the 1.024 V level scaled by
    // L/1024, 2.05 % each (table 45-36) and 0.69 % each of typical supply
    // coefficient, 5.5 % of L; the DAC's offset and gain at code 1000 (20
    // + 20 mV, table 45-32's one row, x 1.023: 41 mV) and its INL at an
    // INTREF reference (3.5 LSB of L/1024, table 45-32); 4 counts of INL
    // on each reading (6 mV x (1 + L/1024)); 3 mV of quantization.
    // Between two levels the gains cancel and the offset weighs once more
    // per 1024 mV: the spreads, the offset and the INLs leave over 800 of
    // the 1024 mV and over 1600 of the 2048, against the 700 and 1500
    // judged above.
    const auto level_tol = [](uint16_t nominal) {
        const uint32_t l = nominal;
        const uint32_t weigh = 1023u * 1024u > 1000u * l ? 1023u * 1024u - 1000u * l
                                                       : 1000u * l - 1023u * 1024u;
        return l * 55u / 1000u + 41u + (80u * weigh + 1024u * 1000u - 1u) / (1024u * 1000u) +
               6u * (1024u + l) / 1024u + 35u * l / 10240u + 4u;
    };
    bench.verdict("...and each level lands within its tables' band of its nominal",
                  near(measured[0], nominal[0], level_tol(nominal[0])) &&
                      near(measured[1], nominal[1], level_tol(nominal[1])) &&
                      (!held[2] || near(measured[2], nominal[2], level_tol(nominal[2]))));

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
                  // the same one-per-cent definition as letter d's
                  without > 1200u && near(without, with, 40u));

    Dac::release();
    Adc0::release();
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024});
}

// =============================================================================
// d - the ADC's own open gap: REFSEL = INTREF with VREFOE off and on
// =============================================================================
//
// TWO DIFFERENT PATHS WEAR THE SAME NAME. Reading the bandgap as an
// INPUT needs SUPC.VREF.VREFOE, which is mandatory and undocumented;
// whether the bandgap as the ADC's REFERENCE needs it too is a
// separate question, and the DAC is what makes it answerable - it
// supplies a steady mid-range voltage the ADC can measure against both
// references and compare.
void td_adc_intref_reference() {
    if (!need_vdd()) {
        return;
    }
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

    bench.verdict("THE ADC'S REFERENCE PATH DOES NOT NEED VREFOE: a conversion "
                  "against REFSEL = INTREF works with the bit CLEAR",
                  ref_off > 100u && ref_off < 4000u);
    // "Not in this path" is defined here as an effect under one per cent:
    // the same chain read twice differs by its noise alone (table 45-24's
    // 2 mV rms, a tenth of a count after 32 conversions), and a reference
    // that leaned on VREFOE would read near zero with the bit clear.
    bench.verdict("...and the bit changes the reading by less than one per "
                  "cent, so it is genuinely not in this path",
                  near(ref_off, ref_on, 41u));
    // The band: the 2.048 V level +-2.05 % and the supply located through
    // the 1.024 V level +-2.05 % (table 45-36), ADC0's offset and gain at
    // each reference (table 45-24: 80 + 42 mV against VDDANA, 21 + 28 mV
    // against a 2 V reference) and 10 mV of INL and quantization.
    bench.verdict("the two references agree on the DAC's voltage within the "
                  "bandgap's level accuracy and the converter's tabled errors",
                  near(implied_off, source_mv,
                       static_cast<uint32_t>(source_mv) * 41u / 1000u + 181u));

    Dac::release();
}

// =============================================================================
// e - Ref::dac: the DAC as the ADC's reference
// =============================================================================
void te_dac_as_reference() {
    if (!need_vdd()) {
        return;
    }
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

    bench.verdict("REFSEL = DAC CONVERTS - adc.hpp's enumerator is real silicon",
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
    // The ratio cancels the supply and ADC0's gain; what is left is the
    // DAC's 20 mV offset against each reference voltage (table 45-32: 0.8 %
    // at code 512 on 5 V, 1.2 % at 3.3 V), twice, and 4 counts of INL:
    // under three per cent and 4 counts.
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
    // The absolute figure is printed above, not judged: the quarter-supply
    // divider's own accuracy is in no table, and with a DAC as the
    // reference no ADC row applies either.

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

    // Code 768 is 3075 counts against VDDANA, within the pair's tabled
    // error at three quarters of full scale; the two routes share the
    // converter and the reference, so they differ by their noise (the
    // spans above) and a count of each mean.
    bench.verdict("MUXPOS = DAC READS THE DAC - the code 38.8.9's table marks "
                  "Reserved and the device header declares is real silicon",
                  near(internal.mean, 3075u, pair_err_counts(3u, 4u)));
    bench.verdict("and it agrees with the pad path within the two routes' noise, so "
                  "it is the same voltage by two routes",
                  near(internal.mean, pad.mean, internal_span + pad_span + 2u));

    // 41.6.8.1's claim about EOEN, printed: either answer is a fact about
    // the silicon, and brio depends on neither (dac_up() sets both).
    print(serial, "  41.6.8.1 says the output buffer must be enabled for the "
          "ADC to see the DAC; with EOEN clear the internal channel reads ",
          internal_no_eoen, " counts against ", internal.mean, " with it set: ",
          near(internal_no_eoen, internal.mean, 82u) ? "the same voltage"
                                                     : "not the DAC's voltage",
          crlf);

    // ERRATUM 1.8.9 PREDICTS TWO THINGS and they are separate
    // measurements: a "noisy ADC reading" on the internal channel, and
    // "noise on the DAC Output voltage" while such a conversion runs.
    //
    // First half. A verdict is only taken when the two spreads are
    // DECISIVELY apart - the larger at least three times the smaller and
    // at least eight counts - because at this board's noise floor a
    // three-against-five comparison is a coin toss dressed as a
    // measurement.
    const uint16_t larger = internal_span > pad_span ? internal_span : pad_span;
    const uint16_t smaller = internal_span > pad_span ? pad_span : internal_span;
    const bool decisive = larger >= 8u && larger >= 3u * (smaller + 1u);
    // Printed, not judged: an erratum need not reproduce on a die.
    print(serial, "  erratum 1.8.9's reading noise: ",
          decisive && internal_span > pad_span ? "shows (the internal channel decisively wider)"
                                               : "not decisively shown", crlf);

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
    // Printed, not judged, for the same reason.
    print(serial, "  erratum 1.8.9's output noise: ",
          output_decisive && disturbed.span() > quiet.span()
              ? (control.span() <= quiet.span() + 4u
                     ? "shows, and only with MUXPOS = DAC selected"
                     : "shows, with traffic on another input too")
              : "not decisively shown", crlf);

    dac_set(0);

    // ERRATUM 1.4.10, PROVOKED BY THIS LETTER AND MEASURED HERE, because
    // this is the only letter in the suite that runs the second
    // converter. A NARROW PROBE MISSES IT: reading ADC0.SYNCBUSY as
    // 0x0000 with ADC1 enabled and ADC0 disabled shows nothing. It does
    // reproduce, and it is worse than the item's own
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
    // Whether erratum 1.4.10 shows is printed, not judged; the errata's
    // way out (ADC1 first) is the verdict, and a quarter of the supply
    // reads within 10 % (ADC0's tabled error at a quarter of full scale,
    // some 75 counts at 5 V, and the divider's untabled own).
    if (!plain_init) {
        print(serial, "  erratum 1.4.10 shows: SYNCBUSY.ENABLE ",
              (stuck & ADC_SYNCBUSY_ENABLE_Msk) != 0u ? "stuck" : "clear",
              ", the converter reads ", dead_read, crlf);
        const bool rescued = adc0_up(pad_cfg);
        Adc0::select(AdcInput::scaled_supply);
        const uint16_t good_read = Adc0::read();
        print(serial, "  with ADC1 brought up FIRST, ADC0 comes up and reads ",
              good_read, " counts - and it keeps working after ADC1 is "
              "released again", crlf);
        bench.verdict("THE ERRATA'S OWN ORDER IS THE WAY OUT: ADC1 enabled "
                      "before ADC0 and the converter is back, with no reset of "
                      "the device",
                      rescued && near(good_read, 1024u, 102u));
    } else {
        print(serial, "  erratum 1.4.10 did not reproduce in this arrangement - "
              "an observation on one die", crlf);
        bench.verdict("...and ADC0 reads normally", near(dead_read, 1024u, 102u));
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
            brio::spin_us<Ruler>(80);   // the DAC's conversion plus the AC's sampling
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

    bench.verdict("ACNEGATIVE::DAC IS REAL - ac.hpp's enumerator flips a comparator",
                  found[0] > 0u && found[0] < 1023u);
    // Where the crossings land, and the gaps between them below, are
    // printed, not judged: the VDD scaler's INL, offset and gain errors are
    // typical figures only (table 45-34), so nothing written bounds them.
    // The strong claim: the SPACING between crossings is the DAC's own
    // linearity, with the comparator's offset differenced out.
    const uint32_t gap1 = static_cast<uint32_t>(found[1] - found[0]);
    const uint32_t gap2 = static_cast<uint32_t>(found[2] - found[1]);
    print(serial, "  the two gaps are ", gap1, " and ", gap2, " codes, "
          "predicted 256 each - and differencing them cancels the "
          "comparator's offset", crlf);
    bench.verdict("the crossings come in the scaler's order",
                  found[0] < found[1] && found[1] < found[2]);

    // A control: with the DAC parked at zero the comparator must see the
    // scaler above it at every step, and at full scale below it.
    (void)Dac::set(0);
    settle();
    const bool low_side = Comp::state();
    (void)Dac::set(1023);
    settle();
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
// LSB is 2.4 ADC counts, 0.9 at 2.7 V) and table 45-24 puts the ADC's at
// +/-4 LSB; with the fit's tilt the band below is 21 counts.
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
    // The residual's bound: each point carries the pair's INL at a VDDANA
    // reference (the DAC's 0.9 LSB of 1024 at 2.7 V, 3.6 counts, table
    // 45-32; the ADC's 4, table 45-24): 7.6 counts; the 31-point line can
    // be tilted by the same INL times its hat weights' largest row sum,
    // 1.625: 7.6 x 2.625 = 20 counts, and one more for the 64-sample means'
    // noise and quantization: 21.
    bench.verdict("the residual from the best-fit line stays inside the two "
                  "converters' COMBINED datasheet nonlinearity (21 counts, the "
                  "INLs and the fit's tilt)",
                  worst_centi <= 2100);
    // The pair's gain errors: the DAC's 20 mV and the ADC's 42 mV (24 at
    // 3.0 V) of full scale, 1.3 % at the worst row: 4004 x 0.987..1.013.
    bench.verdict("and the slope is within the pair's combined gain error of a "
                  "full-scale span (1.3 %)",
                  slope_milli > 3950 && slope_milli < 4058);

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
    // BOTH ENDPOINTS ARE PRINTED, and neither is a verdict. Where the
    // line runs below zero at code 0, "the reading sits above it" holds
    // for any reading a converter can give; and the TOP endpoint sits on
    // the line within a count, the fit's own sub-count jitter - whether
    // table 45-30's 50 mV top-of-range limit clips there is below this
    // instrument's resolution.
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
    if (!window_ready()) {
        Stopwatch::release();
        return;
    }
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
        up = fast_until([] { return Dac::ready(); }) && up;
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
    // The figure is printed above, not judged: table 45-31's 3 us is a
    // simulation's, and the ENABLE write's synchronization inside the
    // number has no figure of its own.
    bench.verdict("STATUS.READY rose after every ENABLE", startup_min > 0u);

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
    settle_long();

    // THE INSTRUMENT'S OWN FLOOR, measured before the measurand: a
    // single-shot timing is not usable here, because reading the
    // stopwatch is itself a READSYNC command across a clock boundary,
    // measured at about two microseconds, which is the same order as
    // the thing being timed. Every window from here to the last loop
    // opens on a drained console and prints only after it closes.
    (void)Dac::set(1023);
    brio::spin_us<Ruler>(800);
    if (!window_ready()) {
        (void)Comp::enable(false);
        Ac::release();
        Dac::release();
        Stopwatch::release();
        return;
    }
    const uint32_t floor_t0 = ticks_now();
    Dac::regs().DAC_DATA = 1023;
    const bool floor_crossed = fast_until([] { return !Comp::state(); });
    const uint32_t floor_ticks = ticks_now() - floor_t0;

    // TWO LOOPS, DIFFERENCED. Loop A waits for the comparator at every
    // step; loop B does the same stores and the same DATA
    // synchronization and waits for nothing. Everything software is in
    // both, so what is left is the analog crossing. Differencing is the
    // only honest shape where a single measurement carries a constant
    // nobody can name. Each wait is fast_until(): within its first 64
    // reads it costs the reads alone, so the two loops differ by the
    // crossing and nothing else.
    constexpr uint16_t reps = 500;
    auto sync_data = []() {
        return fast_until(
            [] { return (Dac::regs().DAC_SYNCBUSY & DAC_SYNCBUSY_DATA_Msk) == 0u; });
    };

    bool crossed = floor_crossed;
    (void)Dac::set(0);
    brio::spin_us<Ruler>(800);
    const uint32_t a0 = ticks_now();
    for (uint16_t i = 0; crossed && i < reps; ++i) {
        Dac::regs().DAC_DATA = 1023;
        crossed = sync_data() && fast_until([] { return !Comp::state(); });
        Dac::regs().DAC_DATA = 0;
        crossed = crossed && sync_data() && fast_until([] { return Comp::state(); });
    }
    const uint32_t ta = ticks_now() - a0;

    (void)Dac::set(0);
    brio::spin_us<Ruler>(800);
    const uint32_t b0 = ticks_now();
    for (uint16_t i = 0; crossed && i < reps; ++i) {
        Dac::regs().DAC_DATA = 1023;
        crossed = sync_data();
        (void)Comp::state();
        Dac::regs().DAC_DATA = 0;
        crossed = crossed && sync_data();
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

    print(serial, "  a SINGLE-SHOT timing of one step costs ", floor_ticks,
          " ticks before the DAC does anything at all (two synchronized "
          "stopwatch reads), so the measurement below is a DIFFERENCE", crlf);
    bench.verdict("the comparator followed every step and every DATA write "
                  "synchronized, within 10 ms each",
                  crossed);

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
    // "Decisively": beyond what the two windows can differ by with no
    // wait at all - each window's own pair of stopwatch reads, whose
    // whole cost (the single-shot figure above) bounds their jitter,
    // which also covers the one SysTick entry by which two windows of
    // about 1.5 ms can differ.
    bench.verdict("the waiting loop really does wait - it is decisively "
                  "slower than the same loop with nothing to wait for",
                  ta > tb + 2u * floor_ticks);
    // How fast the step crosses is printed above, not judged: table 45-31
    // gives no settling time at all, only a conversion rate.

    (void)Dac::set(0);
    (void)Comp::enable(false);
    Ac::release();
    Dac::release();
    Stopwatch::release();
}

// =============================================================================
// j - the event-paced DAC, fed from its EMPTY interrupt
// =============================================================================
//
// A timer overflow crosses an asynchronous event channel into the DAC's
// START user, which copies DATABUF into DATA and converts; DATABUF
// going empty raises INTFLAG.EMPTY, and DAC_Handler answers it with the
// next value of a table in RAM; and the same EMPTY becomes an event a
// second timer counts. THE CPU IS IN THE PATH for one DATABUF write a
// conversion - the pacing and the conversion stay the event's, the
// supply is the handler's (41.6.3 and 41.6.4: EMPTY is a DMA request
// and an interrupt source alike).
//
// THE FIRST VALUE IS THE STARTER'S, because EMPTY is an EVENT and not a
// state: 41.8.6 sets it when DATABUF is consumed, so a converter just
// enabled whose buffer was never written reads it clear, and a feed
// that waited for the flag would never begin. The letter writes wave[0]
// itself and the handler writes the other wave_len - 1.
void tj_event_feed() {
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

    bench.verdict("ADC0 watches the pad", adc0_up(pad_cfg));
    Adc0::select(AnalogIn<Vout>{});

    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    (void)Counter::enable(true);
    (void)Counter::set_count16(0);
    (void)Dac::enable(true);
    // The starter's value, then the handler's turn. UNDERRUN is left
    // UNARMED: isr() would clear it, and the letter reads it below.
    feed_empties = 0;
    Dac::buffer(wave[0]);
    feed_next = 1;
    feed_left = static_cast<uint16_t>(wave_len - 1u);
    Dac::arm(Dac::flag_empty);
    Nvic::enable(Dac::irq());
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
          hi, " counts; TC3 counted ", counted, " EMPTY events, the handler "
          "took ", feed_empties, " EMPTY interrupts and wrote ",
          feed_next, " of ", wave_len, " values (wave[0] the starter's)", crlf);
    // The table's codes 100 and 900 are 400 and 3604 counts against
    // VDDANA; the extremes the ADC saw reach each within the pair's
    // tabled error there.
    bench.verdict("THE PAD FOLLOWED THE TABLE - the ADC saw both levels of a "
                  "waveform the EMPTY interrupt fed and an event started",
                  lo <= 400u + pair_err_counts(100u, 1023u) &&
                      hi + pair_err_counts(900u, 1023u) >= 3604u);
    bench.verdict("THE DAC IS A GENERATOR TOO: TC3 counted the buffer going "
                  "empty",
                  counted > 0u);
    bench.verdict("and the handler played the whole table into DATABUF, one "
                  "value an EMPTY",
                  feed_left == 0u && feed_next == wave_len);

    // UNDERRUN: with the table played, the next START event finds
    // DATABUF empty. 41.6.4 says that is exactly what the flag means,
    // and it can only happen when events are what start conversions.
    Dac::clear_flags(Dac::flag_underrun);
    (void)Pacer::enable(true);
    brio::spin_us<Ruler>(20'000);
    (void)Pacer::enable(false);
    const bool under = Dac::underrun();
    print(serial, "  with the table played and the pacer still running, "
          "UNDERRUN reads ", yes_no(under), crlf);
    bench.verdict("UNDERRUN IS WHAT 41.6.4 SAYS IT IS - a start event with "
                  "nothing in DATABUF, and it exists only in the event-driven "
                  "shape",
                  under);

    // A control: nothing moves without the pacer. DATA first - a value
    // standing in DATABUF would make the DATA write a discarded one
    // (41.6.7, see Dac::set()) - then the feed re-armed on the table.
    Dac::disarm(Dac::flag_empty);
    (void)Dac::set(512);
    settle();
    Dac::buffer(wave[0]);
    feed_next = 1;
    feed_left = static_cast<uint16_t>(wave_len - 1u);
    Dac::arm(Dac::flag_empty);
    brio::spin_us<Ruler>(20'000);
    const Spread still = spread_of<Adc0>(16);
    print(serial, "  with the pacer stopped the pad holds ", still.mean,
          " counts, spread ", still.span(), "; the handler wrote ",
          feed_next - 1u, " values", crlf);
    bench.verdict("with nothing starting conversions the feed cannot move the "
                  "output on its own - the event really is the trigger",
                  still.span() < 64u && feed_next == 1u);

    Nvic::disable(Dac::irq());
    Dac::disarm(Dac::flag_empty);
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
/// armed as the anti-wedge backstop: a wake that never comes costs a
/// reboot and a banner instead of a mute board. A wake that came early
/// waits awake for the compare, on the ruler (the SysTick runs again once
/// the core is out of standby), at most twice the sleep asked for.
bool standby_for(uint32_t ticks) {
    (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc4096});
    const uint32_t now = Rtc::count32();
    rtc_fired = false;
    Rtc::clear_flags(RtcFlag::compare0);
    if (!Rtc::set_comp32(now + ticks)) {
        (void)Watchdog::disable();
        return false;
    }
    if (!console_drain() || !Pm::set_sleep_mode(SleepMode::standby)) {
        (void)Watchdog::disable();
        return false;
    }
    __disable_irq();
    if (!rtc_fired) {
        Pm::sleep();
    }
    __enable_irq();
    // ticks of 32768 Hz as microseconds: x 1e6 / 32768 = x 15625 / 512,
    // inside 32 bits for any sleep under 8 s.
    const uint32_t sleep_us = (ticks * 15'625u) >> 9;
    const bool fired = brio::wait_until<Ruler>([] { return rtc_fired; }, 2u * sleep_us);
    (void)Pm::set_sleep_mode(SleepMode::idle0);
    (void)Watchdog::disable();
    return fired;
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
    settle_long();
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
    settle_long();
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
    brio::spin_us<Ruler>(500'000);
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
    // Printed, not judged: whether the item reproduces is a fact about the
    // die, and the caller's duty (clear EMPTY after a wake) holds either way.
    print(serial, empty_after
                      ? "  erratum 1.9.2 reproduces: the standby set EMPTY with "
                        "nothing having consumed DATABUF, where the same wait "
                        "awake left it clear"
                      : "  erratum 1.9.2 did not reproduce on this die in this "
                        "arrangement",
          crlf);
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

// =============================================================================
// l - EMPTY as a wake from STANDBY
// =============================================================================
//
// DATABUF written, then the core in STANDBY: TC2 (RUNSTDBY) overflows
// 5.5 ms later (CLK_PER / 1024, PER 255) and its event, on an
// asynchronous channel kept running in standby, starts the conversion
// that empties DATABUF - EMPTY, the interrupt that ends the sleep. The
// suite's own RTC compare 300 ms out is the backstop. RUNSTDBY set on the
// DAC: one wake, DAC's line pending, the backstop silent. RUNSTDBY clear:
// the converter is off in standby, nothing empties, the backstop ends the
// sleep (erratum 1.9.2 may then raise EMPTY after the wake - letter k's
// subject - so the control is judged on WHAT ENDED the sleep).
void tl_wake() {
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the RTC is up as the backstop", rtc_up());
    auto leg = [](bool run_standby, uint32_t& pend, bool& backstop, uint32_t& empties) {
        pend = 0;
        backstop = false;
        empties = 0;
        Dac::release();
        DacConfig cfg{};
        cfg.external_output = true;
        cfg.internal_output = true;
        cfg.run_standby = run_standby;
        constexpr TcConfig pace{.mode = TcMode::count8, .prescaler = TcPrescaler::div1024,
                                .waveform = TcWaveform::normal_pwm, .run_standby = true};
        if (!Dac::init(main_gen, cfg) || !Dac::enable(false) ||
            !Dac::start_on(ev_start_channel,
                           EventChannelConfig{.generator = Pacer::overflow_generator,
                                              .path = EventPath::asynchronous,
                                              .run_standby = true}) ||
            !Dac::enable(true) || !Pacer::init(main_gen) || !Pacer::configure(pace) ||
            !Pacer::set_period8(255) ||
            !Pacer::event_config(pace, TcEventConfig{.overflow_out = true})) {
            return;
        }
        Dac::buffer(700);
        settle_long();
        Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
        feed_left = 0;                    // the handler counts, then disarms
        const uint32_t e0 = feed_empties;
        Dac::arm(Dac::flag_empty);
        Nvic::enable(Dac::irq());
        const uint32_t now = Rtc::count32();
        rtc_fired = false;
        Rtc::clear_flags(RtcFlag::compare0);
        if (!Rtc::set_comp32(now + 9830u)) {   // 300 ms
            return;
        }
        // A console that does not drain ends the leg with no sleep: its
        // verdicts fail on the pending word they never got.
        if (!console_drain() || !Pm::set_sleep_mode(SleepMode::standby)) {
            return;
        }
        (void)Pacer::set_count8(0);
        (void)Pacer::enable(true);
        __disable_irq();
        if (feed_empties == e0 && !rtc_fired) {
            Pm::sleep();
            pend = NVIC->ISPR[0];
        }
        __enable_irq();
        (void)Pm::set_sleep_mode(SleepMode::idle0);
        backstop = rtc_fired;
        (void)Pacer::enable(false);
        Dac::disarm(Dac::flag_empty);
        Nvic::disable(Dac::irq());
        empties = feed_empties - e0;
    };
    uint32_t p_on = 0, p_off = 0, e_on = 0, e_off = 0;
    bool b_on = false, b_off = false;
    leg(true, p_on, b_on, e_on);
    leg(false, p_off, b_off, e_off);
    const uint32_t line = 1u << static_cast<uint32_t>(Dac::irq());
    print(serial, "  STANDBY, RUNSTDBY set: pending ", Hex{p_on}, ", EMPTY interrupts ",
          e_on, ", backstop ", b_on ? "FIRED" : "silent", "; RUNSTDBY clear: pending ",
          Hex{p_off}, ", backstop ", b_off ? "fired" : "SILENT", crlf);
    bench.verdict("EMPTY WAKES THE CORE FROM STANDBY with RUNSTDBY: DAC's line, the "
                  "backstop silent",
                  (p_on & line) != 0u && e_on >= 1u && !b_on);
    // The control's wake is the RTC's: the backstop RAN (300 ms, so the
    // pacer's start 5.5 ms in woke nothing). DAC's line is pending beside
    // it at that wake (measured, every run): EMPTY raised on the exit from
    // sleep with DATABUF still written - erratum 1.9.2, letter k's subject.
    const uint32_t rtc_line = 1u << static_cast<uint32_t>(Rtc::irq());
    bench.verdict("without RUNSTDBY the backstop is what ends the sleep",
                  b_off && (p_off & rtc_line) != 0u);
    Dac::release();
    Pacer::release();
    Evsys::disconnect(Dac::start_event_user);
    Evsys::release_channel(ev_start_channel);
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

/// The DAC's one vector, letter j's feed: each EMPTY writes the next
/// value of the table into DATABUF, which is also what clears the flag
/// (41.8.6). With the table played the handler disarms EMPTY instead -
/// isr() leaves that flag standing by contract, and nothing else would
/// clear it.
extern "C" void DAC_Handler() {
    const uint8_t pending = brio::Dac::isr();
    if ((pending & brio::Dac::flag_empty) != 0u) {
        feed_empties = feed_empties + 1u;
        const uint16_t left = feed_left;
        if (left != 0u) {
            const uint16_t i = feed_next;
            brio::Dac::buffer(wave[i]);
            feed_next = static_cast<uint16_t>(i + 1u);
            feed_left = static_cast<uint16_t>(left - 1u);
        } else {
            brio::Dac::disarm(brio::Dac::flag_empty);
        }
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

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
    bench.letter('j', "the event-paced DAC fed from its EMPTY interrupt",
                 tj_event_feed);
    bench.letter('k', "erratum 1.9.2: the EMPTY flag across a standby",
                 tk_standby_erratum);
    bench.letter('l', "EMPTY as a wake from STANDBY", tl_wake);

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
