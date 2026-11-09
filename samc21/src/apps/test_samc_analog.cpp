// test_samc_analog - the CROSS-CHAPTER analog suite: what the five
// analog chapters (38 ADC, 39 SDADC, 40 AC, 41 DAC, 43 TSENS) can only
// be asked with more than one of them running at once.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the drivers
// under it.
//
// WHAT MAKES IT POSSIBLE, and each of the three is a whole letter's
// stimulus:
//   - THE DAC IS A REAL MID-SCALE SOURCE. On this package PA02 is
//     DAC/VOUT, ADC0/AIN0 and the AC's AIN4 at once, so a swept voltage
//     between the rails reaches the SAR converter's positive input, its
//     negative multiplexer and two comparators' positive input with no
//     wire at all. Differential mode, the comparator hysteresis and the
//     40.6.10 offset procedure are all sweeps of that one source.
//   - supc.hpp's BANDGAP, and its VREFOE bit, which is what puts a
//     known mid-scale level on a comparator's negative input.
//   - THE EVENT PACERS. A TC overflow into the DAC's START user is what
//     dithering IS (41.6.8.4 makes the sixteen sub-conversions the
//     event's job), and a comparator output is a LEVEL, which is what an
//     inverted event input needs to be measurable at all.
//
// WHAT IS DELIBERATELY NOT HERE, said once: anything needing a wire or a
// second board (VREFA on any converter, the SDADC's external reference
// at a non-rail level, the DAC's voltage pump - this board sits at
// ~5.15 V where it is off); sleep, which test_samc_sleepwalk owns;
// util/ adapters (analog_sampler-vs-SDADC and a TSENS MeterSource stay
// design questions, restated in the docs and not built); and
// SDADC.ANACTRL's CTLSDADC/BUFTEST, which 39.8.21 calls
// "Debug/Characterization" with no values and describes not at all -
// poking them blind would produce a number with no meaning, so they stay
// declined with that reason.
//
// What is exercised, letter by letter:
//   a  THE ADC HOST/CLIENT PAIR (38.6.3.1): both dual modes on silicon,
//      the three restart options, erratum 1.4.10 spent visibly
//   b  the ADC's AUTOMATIC SEQUENCE (38.6.2.12) over six inputs with six
//      distinct known values, and what it costs
//   c  ADC DIFFERENTIAL MODE against the DAC: sign, magnitude, and the
//      signed window thresholds
//   d  CTRLC.R2R and SAMPCTRL.OFFCOMP, measured on the same input at
//      three common modes
//   e  DAC DITHERING (41.6.8.4): sub-LSB means between two adjacent
//      codes, with the undithered control, DATABUF fed from the DAC's
//      own EMPTY interrupt - and CTRLB.LEFTADJ on silicon
//   f  the DAC's EMPTY and UNDERRUN interrupts through the NVIC
//   g  the AC completed: COMP2, COMP3 and WINDOW 1 on silicon
//   h  the BANDGAP as a comparator's negative input, VREFOE both ways,
//      erratum 1.5.6 staged with a control, and Ac::take_flags() from a
//      real bound handler
//   i  AC HYSTERESIS measured in DAC steps, both transitions, both
//      speeds - and the 40.6.10 SWAP offset procedure
//   j  the rising/falling INTSEL flavours and INVEIx driven by a real
//      event
//   k  SDADC leftovers: WINMONEO, the flush, and the three interrupts
//      through the NVIC
//   l  TSENS EVCTRL.STARTINV against a real event edge
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "samc21/ac.hpp"
#include "samc21/adc.hpp"
#include "samc21/clock.hpp"
#include "samc21/dac.hpp"
#include "samc21/evsys.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/sdadc.hpp"
#include "samc21/sercom.hpp"
#include "samc21/supc.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "samc21/tsens.hpp"
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
/// false on a wedged console - a timed window opens only after it.
bool console_drain() { return brio::console_drain<Ruler, Serial>(); }

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The pads, and what each one is on this die
// ---------------------------------------------------------------------------
using Vout = Pin<'A', 2>;      // DAC/VOUT = ADC0/AIN0 = AC/AIN4 (COMP2/3 PIN0)
using PadA3 = Pin<'A', 3>;     // DAC/VREFA = ADC0/AIN1 = AC/AIN5 (COMP2/3 PIN1)
using PadA4 = Pin<'A', 4>;     // ADC0/AIN4 = AC/AIN0 (COMP0/1 PIN0)
using PadShared = Pin<'A', 8>; // ADC0/AIN8 and ADC1/AIN10 at once
using PadSdN = Pin<'A', 6>;    // SDADC AINN0
using PadSdP = Pin<'A', 7>;    // SDADC AINP0

using Adc0 = Adc<0>;
using Adc1 = Adc<1>;
using Comp0 = AcComparator<0>;
using Comp2 = AcComparator<2>;
using Comp3 = AcComparator<3>;
using Window1 = AcWindow<1>;

constexpr uint8_t main_gen = 0;
constexpr uint32_t main_gen_hz = SysClock::hz;

/// Where this board's supply sits, in millivolts: 0 until locate_supply()
/// has read it (at boot, and again by a letter that needs it). The board
/// takes 3.3 V or 5 V on a jumper (docs/boards/samc21j.md), so no figure
/// is assumed.
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
constexpr uint32_t supply_hi_permille = 1125;

/// The most counts of 4096 a voltage of `mv` (up to 1000) can be against
/// the true supply, given the located one (rounded up), or full scale
/// when no supply has been located (nothing judged tight).
uint32_t mv_counts(uint32_t mv) {
    return vdd_mv != 0u
               ? (mv * 4096u * 1000u + vdd_mv * supply_lo_permille - 1u) /
                     (vdd_mv * supply_lo_permille)
               : 4096u;
}
/// The same as DAC codes of 1024 (rounded up).
uint32_t mv_codes(uint32_t mv) {
    return vdd_mv != 0u
               ? (mv * 1024u * 1000u + vdd_mv * supply_lo_permille - 1u) /
                     (vdd_mv * supply_lo_permille)
               : 1024u;
}
/// The fewest DAC codes a voltage of `mv` can be (rounded down), 0 when
/// no supply has been located.
uint32_t mv_codes_min(uint32_t mv) {
    return vdd_mv != 0u ? (mv * 1024u * 1000u) / (vdd_mv * supply_hi_permille) : 0u;
}

// ---------------------------------------------------------------------------
// The stopwatch: TC0 + TC1 as one 32-bit counter on the BOARD'S CRYSTAL.
// A conversion time reported against OSC48M would carry that
// oscillator's 5100 ppm into a number about a converter.
// ---------------------------------------------------------------------------
using Stopwatch = Tc<0>;
constexpr uint32_t crystal_hz = 24'000'000UL;
constexpr uint8_t gen_crystal = 2;
bool on_crystal = false;

bool stopwatch_start() {
    if (on_crystal) {
        return true;
    }
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
/// Crystal ticks to microseconds: 24 MHz, so 24 ticks is one.
uint32_t ticks_us(uint32_t ticks) { return ticks / 24u; }

// ---------------------------------------------------------------------------
// The event fabric: the shape every SAM suite here uses.
// ---------------------------------------------------------------------------
constexpr uint8_t ev_a = 0;    ///< the pacer's channel
constexpr uint8_t ev_b = 1;    ///< an output event's channel
constexpr uint8_t ev_gen = 6;  ///< the generator the event channels run on
using EvGen = Gclk<ev_gen>;
using Pacer = Tc<2>;
using Counter = Tc<3>;   ///< NB TC2 and TC3 SHARE generic clock channel 31

bool event_clock_up() {
    Evsys::bus_clock(true);
    return EvGen::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
           GclkChannel::connect(Evsys::gclk_id(ev_a), ev_gen) &&
           GclkChannel::connect(Evsys::gclk_id(ev_b), ev_gen);
}

void event_clock_down() {
    GclkChannel::disconnect(Evsys::gclk_id(ev_a));
    GclkChannel::disconnect(Evsys::gclk_id(ev_b));
}

// ---------------------------------------------------------------------------
// Interrupt bookkeeping, filled by the bound handlers
// ---------------------------------------------------------------------------
volatile uint32_t dac_empty_irqs = 0;
volatile uint32_t dac_underrun_irqs = 0;
volatile uint32_t dac_irq_entries = 0;
volatile uint32_t ac_irqs = 0;
volatile uint8_t ac_last_mask = 0;
volatile uint32_t sdadc_irqs = 0;
volatile uint8_t sdadc_last_mask = 0;
volatile int32_t sdadc_last_result = 0;
volatile bool dac_feed_from_isr = false;
volatile uint16_t dac_feed_value = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Long enough for the DAC's own conversion (2.9 us at the 350 ksps
/// ceiling) and for a converter's input to see the new level: 400 us.
void settle() { brio::spin_us<Ruler>(400); }
/// The sweep version: still an order of magnitude over the conversion
/// time, and short enough that a 200-code sweep is milliseconds: 60 us.
void settle_fast() { brio::spin_us<Ruler>(60); }
/// After a block comes up or a reference changes: 4 ms.
void settle_long() { brio::spin_us<Ruler>(4'000); }

/// The bound on one SAR conversion or pair of them here: 10 ms, hundreds
/// of times the 12 us pad_cfg takes at 1.5 MHz.
constexpr uint32_t adc_budget_us = 10'000;

/// ADC0's RESRDY within that bound.
bool adc0_lands() {
    return brio::wait_until<Ruler>([] { return Adc0::ready(); }, adc_budget_us);
}

/// Hand a pad back as reset leaves it: an input, its buffer off, no pull
/// (release() alone leaves PORT's output driver on).
template <class P>
void hand_back() {
    P::input();
    P::configure({});
}

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}
bool near_signed(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

const char* yes_no(bool v) { return v ? "yes" : "no"; }

/// The single-reading ADC configuration on a pad.
constexpr AdcConfig pad_cfg{
    .reference = Ref::vddana,
    .prescaler = AdcPresc::div32,   // 48 MHz / 32 = 1.5 MHz CLK_ADC
    .sample_length = 5,
};

/**
 * ADC0 up, WITH ERRATUM 1.4.10'S WORKAROUND WHERE IT IS NEEDED.
 *
 * Once ADC1 has been enabled in this power cycle, ADC0.SYNCBUSY.ENABLE
 * is stuck at one on this die and `Adc<0>::init()` - which waits on it -
 * returns false with the converter left DISABLED and reading zero. The
 * errata's own workaround is "enable ADC0 before ADC1, or disregard the
 * bit"; what works here is to bring ADC1 up first and ADC0 second, after
 * which ADC0 keeps converting even when ADC1 goes away again. It says
 * out loud when it has to act, which is what keeps `z` re-runnable in
 * one power-on.
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
    uint32_t low;
    uint32_t high;
    uint32_t mean;
    uint32_t span() const { return high - low; }
};

template <class A>
Spread spread_of(uint16_t count) {
    A::discard(2);
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < count; ++i) {
        const uint32_t v = A::read();
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

/// The DAC on its pad. Both outputs unless told otherwise: the pad for
/// the ADC and the comparators' PIN0, the internal path for
/// AcNegative::dac.
bool dac_up(bool external = true, bool internal = true,
            bool left_adjust = false) {
    DacConfig cfg{};
    cfg.reference = DacRef::vddana;
    cfg.external_output = external;
    cfg.internal_output = internal;
    cfg.left_adjust = left_adjust;
    return Dac::init(main_gen, cfg);
}

void dac_set(uint16_t code) {
    (void)Dac::set(code);
    settle();
}

/**
 * Locate the supply: read the BANDGAP as an input against VDDANA as
 * the reference, so the reading is 1.024 V of VDDANA and the supply
 * falls out of it. The other direction - the
 * quarter-supply channel against the bandgap - saturates on this board,
 * since VDDANA/4 is about 1.29 V and the smallest bandgap level is
 * 1.024 V.
 *
 * THE BANDGAP INPUT CHANNEL NEEDS SUPC.VREF.VREFOE (measured: the
 * channel reads a flat zero without it) and table 45-22's 10 us of
 * sampling; SAMPLEN 20 at 1.5 MHz is 14 us. The level sits under every
 * supply the device runs on (2.7..5.5 V, DS60001479M table 45-2), so the
 * reading never saturates; its accuracy is the level's and ADC0's, as
 * supply_lo_permille spells out. Once a boot: true when the supply is
 * located, now or before.
 */
bool locate_supply() {
    if (vdd_mv != 0u) {
        return true;
    }
    if (!adc0_up(AdcConfig{.reference = Ref::vddana,
                           .prescaler = AdcPresc::div32,
                           .sample_length = 20})) {
        return false;
    }
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                     .output_enable = true});
    settle_long();
    Adc0::select(AdcInput::intref);
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    settle();
    const uint32_t counts = mean_of<Adc0>(16);
    // What the level can read under any supply the device runs on: the
    // level's 1003..1045 mV (table 45-36) with ADC0's offset and gain (80
    // mV and 42 mV x the fraction, table 45-24), INL and quantization
    // against 2.7..5.5 V (table 45-2) is 676..1737 counts; outside that
    // no level was read.
    if (counts >= 676u && counts <= 1737u) {
        vdd_mv = static_cast<uint16_t>((1024UL * 4096UL + counts / 2u) / counts);
    }
    Adc0::release();
    return vdd_mv != 0u;
}

/// locate_supply() for a letter whose verdicts rest on the supply: a
/// failure is the letter's verdict.
bool need_vdd() {
    const bool ok = locate_supply();
    if (!ok) {
        bench.verdict("VDDANA located through the 1.024 V bandgap level", false);
    }
    return ok;
}

/// Whether a bandgap level of `level_mv` can be reached under a supply
/// located at `supply_mv`: the level at its top (+2.05 %, table 45-36,
/// and 0.69 % of typical supply coefficient: x 1.028) with 122 mV over
/// it (ADC0's offset and gain at full scale, table 45-24, which covers
/// the DAC's and the comparator's offsets too), under the least the true
/// supply can be (supply_lo_permille).
constexpr bool level_below_supply(uint32_t level_mv, uint32_t supply_mv) {
    return level_mv * 1028u + 122'000u < supply_mv * supply_lo_permille;
}

// =============================================================================
// a - THE ADC HOST/CLIENT PAIR (38.6.3.1)
// =============================================================================
//
// The most single-board item on the die: ADC0 is the host (it owns
// CTRLC.DUALSEL), ADC1 the client (it owns CTRLA.SLAVEEN), and the
// driver already refuses either knob on the wrong instance at compile
// time. What has never run is the pair itself.
//
// PA08 is ADC0/AIN8 AND ADC1/AIN10 - one pad, two converters - so
// "simultaneous" has a witness that costs no wire: the two results must
// agree, as they do when each converter reads that pad on its own.
//
// THE RATE CLAIM IS FRAMED AS A FALSIFIABLE ONE. Both modes deliver two
// results per trigger; what INTERLEAVE buys is that ONE signal can be
// sampled faster than one converter can convert, because each converter
// gets every OTHER trigger. So the measurement is made at a trigger
// period SHORTER than one conversion: a single converter must overrun
// there and the interleaved pair must not.
void ta_pair() {
    bench.verdict("the device header gives the two instances their roles, and "
                  "they are not symmetric",
                  Adc0::pair_role == 1u && Adc1::pair_role == 2u &&
                      Adc0::is_host && Adc1::is_client &&
                      !Adc0::is_client && !Adc1::is_host);

    // The compile-time refusals, restated as runtime ones so the letter
    // stands on its own.
    AdcConfig wrong_client = pad_cfg;
    wrong_client.client_enable = true;
    AdcConfig wrong_dual = pad_cfg;
    wrong_dual.dual = AdcDual::interleave;
    bench.verdict("SLAVEEN on the host and DUALSEL on the client are both "
                  "refused by the configuration itself",
                  !Adc0::config_valid(wrong_client) &&
                      !Adc1::config_valid(wrong_dual) &&
                      Adc1::config_valid(wrong_client) &&
                      Adc0::config_valid(wrong_dual));

    bench.verdict("PA08 is ADC0/AIN8 and ADC1/AIN10 - one pad, both "
                  "converters",
                  Adc0::ain_of('A', 8) == 8 && Adc1::ain_of('A', 8) == 10);

    // ---- the pair, in the errata's own order --------------------------------
    //
    // ERRATUM 1.4.10 says enabling ADC1 while ADC0 is DISABLED can leave
    // ADC0.SYNCBUSY.ENABLE stuck at one, and the workaround is "enable
    // ADC0 before ADC1". That is exactly the order a host/client pair
    // wants anyway, and this is the letter where it is SPENT visibly.
    Adc0::release();
    Adc1::release();
    AdcConfig host = pad_cfg;
    host.dual = AdcDual::both;
    AdcConfig client = pad_cfg;
    client.client_enable = true;

    const bool host_up = adc0_up(host);
    const bool client_up = Adc1::init(main_gen, client, main_gen_hz);
    bench.verdict("the HOST comes up first and the CLIENT second - erratum "
                  "1.4.10's own instruction, and the order the pair wants",
                  host_up && client_up);
    print(serial, "  after both inits CTRLA reads ", Hex{Adc0::regs().ADC_CTRLA},
          " on the host and ", Hex{Adc1::regs().ADC_CTRLA},
          " on the client; ENABLE reads ", yes_no(Adc0::enabled()), " / ",
          yes_no(Adc1::enabled()), crlf);
    bench.verdict("the host is still enabled after the client came up on top "
                  "of it - which is the whole point of the errata's order",
                  Adc0::enabled());
    bench.verdict("THE CLIENT'S OWN CTRLA.ENABLE DOES NOT STAND - the bit is "
                  "written and reads back zero, with only SLAVEEN left, which "
                  "is 38.6.3.1's 'the Client ADC is enabled by accessing the "
                  "CTRLA register of Host ADC' meant LITERALLY; the client "
                  "converts anyway, as the pairs below show, so a caller "
                  "watching that bit would think the converter was off",
                  !Adc1::enabled() &&
                      (Adc1::regs().ADC_CTRLA & ADC_CTRLA_SLAVEEN_Msk) != 0u);
    // Printed, not judged: SLAVEEN is the verdict above's, and DUALSEL =
    // BOTH is 0, the field's reset value, so reading it back proves nothing.
    print(serial, "  the host's CTRLC.DUALSEL reads ",
          static_cast<uint16_t>((Adc0::regs().ADC_CTRLC & ADC_CTRLC_DUALSEL_Msk) >>
                                ADC_CTRLC_DUALSEL_Pos),
          " (BOTH)", crlf);

    // Both converters on the SAME pad.
    Adc0::select(AnalogIn<PadShared>{});
    Adc1::select(AnalogIn<PadShared>{});
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    (void)Adc1::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);

    // ---- DUALSEL = BOTH: one trigger, two results ---------------------------
    PadShared::output();
    PadShared::set();
    settle();

    uint32_t both_pairs = 0;
    uint32_t worst = 0;
    uint32_t last0 = 0;
    uint32_t last1 = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
        Adc1::clear_flags(Adc1::flag_resrdy | Adc1::flag_overrun);
        Adc0::start();                     // the HOST's trigger
        if (!brio::wait_until<Ruler>([] { return Adc0::ready() && Adc1::ready(); },
                                     adc_budget_us)) {
            continue;   // a trigger with no pair is not counted
        }
        last0 = Adc0::result();
        last1 = Adc1::result();
        ++both_pairs;
        const uint32_t d = last0 > last1 ? last0 - last1 : last1 - last0;
        if (d > worst) {
            worst = d;
        }
    }
    print(serial, "  DUALSEL = BOTH, PA08 at VDD: 16 host triggers gave ",
          both_pairs, " pairs, last ", last0, " / ", last1,
          ", worst disagreement ", worst, " counts", crlf);
    bench.verdict("ONE TRIGGER ON THE HOST STARTS BOTH CONVERTERS - sixteen "
                  "triggers, sixteen pairs, and nothing had to be asked of the "
                  "client at all",
                  both_pairs == 16u);
    // Each converter within its tabled offset and gain at the rail (table
    // 45-24: 80 + 42 mV, the larger of its 3.0 V and 5.0 V rows), so the
    // two within twice that, and their INLs and quantization (10 counts).
    bench.verdict("...and on a shared pad the two results agree, which is what "
                  "'simultaneously' has to mean",
                  worst <= mv_counts(244u) + 10u);

    // The other rail, so the agreement is not an artefact of one level.
    PadShared::clear();
    settle();
    Adc0::clear_flags(Adc0::flag_resrdy);
    Adc1::clear_flags(Adc1::flag_resrdy);
    Adc0::start();
    const bool low_pair = brio::wait_until<Ruler>(
        [] { return Adc0::ready() && Adc1::ready(); }, adc_budget_us);
    const uint32_t low0 = Adc0::result();
    const uint32_t low1 = Adc1::result();
    print(serial, "  the same pair with PA08 at GND: ", low0, " / ", low1, crlf);
    // At ground each reads its offset at most (80 mV) and 5 counts.
    bench.verdict("the pair agrees at the other rail too",
                  low_pair && low0 <= mv_counts(80u) + 5u && low1 <= mv_counts(80u) + 5u);

    // ---- the rate question, at a trigger period shorter than one conversion --
    //
    // pad_cfg is div32 (CLK_ADC 1.5 MHz) and SAMPLEN 5, so one
    // conversion is 13 + 5 = 18 CLK_ADC cycles = 12.0 us. The pacer runs
    // at 8.0 us, i.e. FASTER than one converter can convert and slower
    // than two taking turns.
    PadShared::set();
    settle();
    bench.verdict("the event fabric and the pacer are up", event_clock_up());
    constexpr uint8_t pacer_period = 191;    // (191+1) x div2 = 384 cycles = 8.0 us
    const TcConfig pacer_cfg{.mode = TcMode::count8,
                             .prescaler = TcPrescaler::div2,
                             .waveform = TcWaveform::normal_pwm};
    bench.verdict("TC2 paces at 8.0 us, shorter than the 12.0 us one "
                  "conversion takes",
                  Pacer::init(main_gen) && Pacer::configure(pacer_cfg) &&
                      Pacer::set_period8(pacer_period) &&
                      Pacer::event_config(pacer_cfg,
                                          TcEventConfig{.overflow_out = true}));

    // The host's EVCTRL is the pair's EVCTRL: 38.6.3.1 says the host's
    // event inputs are routed to the client, so this is configured once.
    (void)Adc1::enable(false);
    (void)Adc0::enable(false);
    bench.verdict("the pacer reaches the HOST's start user on the "
                  "asynchronous path erratum 1.4.4 makes mandatory",
                  Adc0::start_on(ev_a,
                                 EventChannelConfig{
                                     .generator = Pacer::overflow_generator,
                                     .path = EventPath::asynchronous}));

    // A counting run: read whatever is ready for a fixed window, and let
    // the OVERRUN flag say whether anything was lost.
    struct Run {
        uint32_t results;
        bool over0;
        bool over1;
    };
    auto run_window = [](bool client_on, AdcDual dual) -> Run {
        (void)Adc0::enable(false);
        (void)Adc1::enable(false);
        // CTRLC is write-synchronized and double-buffered rather than
        // enable-protected, so DUALSEL is rewritten with the converter
        // down and the wait is the bus's alone. SLAVEEN is a plain
        // CTRLA bit: clearing it is what makes the "one converter"
        // arrangement really one, whatever DUALSEL says.
        Adc0::regs().ADC_CTRLC = static_cast<uint16_t>(
            (Adc0::regs().ADC_CTRLC & ~static_cast<uint16_t>(ADC_CTRLC_DUALSEL_Msk)) |
            ADC_CTRLC_DUALSEL(static_cast<uint16_t>(dual)));
        (void)Adc0::sync_wait(ADC_SYNCBUSY_CTRLC_Msk);
        Adc1::regs().ADC_CTRLA = static_cast<uint8_t>(
            client_on ? ADC_CTRLA_SLAVEEN_Msk : 0u);
        (void)Adc0::enable(true);
        if (client_on) {
            (void)Adc1::enable(true);
        }
        Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
        Adc1::clear_flags(Adc1::flag_resrdy | Adc1::flag_overrun);
        (void)Pacer::set_count8(0);
        (void)Pacer::enable(true);
        uint32_t got = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 20u) {
            if (Adc0::ready()) {
                (void)Adc0::result();
                ++got;
            }
            if (client_on && Adc1::ready()) {
                (void)Adc1::result();
                ++got;
            }
        }
        (void)Pacer::enable(false);
        const Run r{got, Adc0::overrun(), Adc1::overrun()};
        (void)Adc0::enable(false);
        (void)Adc1::enable(false);
        return r;
    };

    const Run solo = run_window(false, AdcDual::both);
    const Run pair_both = run_window(true, AdcDual::both);
    const Run pair_ilv = run_window(true, AdcDual::interleave);

    // THE BANDS. The window is 19..20 ms (Ticker::millis() steps once a
    // millisecond, and t0 is read anywhere inside a step), the pacer and
    // the ticker both run on OSC48M, so no oscillator error lies between
    // them. A conversion is 18 CLK_ADC cycles (table 45-22: SAMPLEN + 13),
    // 12.0 us, after its start is synchronized to the converter's clock
    // (38.6.2.8: under one CLK_ADC cycle, 0.7 us): 12.7 us, longer than
    // one period (8 us) and shorter than two (16 us). So one converter
    // answers every other trigger, 19000/16 - 1 = 1186 to 20000/16 + 1 =
    // 1251 results; INTERLEAVE answers every trigger, 19000/8 - 2 = 2373
    // to 20000/8 + 1 = 2501; BOTH answers every other trigger twice,
    // 2372..2502.
    constexpr uint32_t every_other_lo = 19'000u / 16u - 1u;
    constexpr uint32_t every_other_hi = 20'000u / 16u + 1u;
    constexpr uint32_t every_lo = 19'000u / 8u - 2u;
    constexpr uint32_t every_hi = 20'000u / 8u + 1u;
    print(serial, "  20 ms at one trigger every 8.0 us (2500 triggers): "
          "ADC0 alone ", solo.results, " results, overrun ",
          yes_no(solo.over0), crlf);
    print(serial, "  DUALSEL=BOTH  ", pair_both.results, " results, overrun ",
          yes_no(pair_both.over0), " / ", yes_no(pair_both.over1), crlf);
    print(serial, "  DUALSEL=INTERLEAVE ", pair_ilv.results,
          " results, overrun ", yes_no(pair_ilv.over0), " / ",
          yes_no(pair_ilv.over1), crlf);
    bench.verdict("A SINGLE CONVERTER ANSWERS EVERY OTHER TRIGGER at a "
                  "trigger period shorter than its own conversion - AND ITS "
                  "OVERRUN FLAG STAYS CLEAR, because a trigger arriving during "
                  "a conversion is simply IGNORED: 38.6.5's OVERRUN is about a "
                  "RESULT nobody read, not about a trigger nobody took",
                  solo.results >= every_other_lo && solo.results <= every_other_hi &&
                      !solo.over0);
    bench.verdict("INTERLEAVE DELIVERS ONE RESULT PER TRIGGER, i.e. EXACTLY "
                  "TWICE what one converter could - each converter takes every "
                  "other trigger and so has two periods to convert in, which is "
                  "the whole content of 38.6.3.1's second trigger mode",
                  pair_ilv.results >= every_lo && pair_ilv.results <= every_hi);
    bench.verdict("...and BOTH mode is NOT the same thing: it doubles the "
                  "results by converting two INPUTS at one instant, so its "
                  "aggregate is exactly twice what the SAME converter managed "
                  "alone - each of the two is still as late as one alone, and "
                  "no signal is sampled any faster",
                  pair_both.results >= 2u * every_other_lo &&
                      pair_both.results <= 2u * every_other_hi);

    // ---- the three restart options 38.6.3.1 lists ---------------------------
    //
    // "To restart an interleaved sequence, the user can apply different
    // options: flush the host, disable/re-enable the host, reset and
    // reconfigure the host."
    auto interleave_up = []() {
        (void)Adc0::enable(false);
        (void)Adc1::enable(false);
        Adc0::regs().ADC_CTRLC = static_cast<uint16_t>(
            (Adc0::regs().ADC_CTRLC & ~static_cast<uint16_t>(ADC_CTRLC_DUALSEL_Msk)) |
            ADC_CTRLC_DUALSEL(static_cast<uint16_t>(AdcDual::interleave)));
        (void)Adc0::sync_wait(ADC_SYNCBUSY_CTRLC_Msk);
        Adc1::regs().ADC_CTRLA = ADC_CTRLA_SLAVEEN_Msk;
        (void)Adc0::enable(true);
        (void)Adc1::enable(true);
    };
    /// Which converter answers the NEXT software trigger. After a
    /// restart the sequence must begin at the host again.
    auto next_answering = []() -> int {
        Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
        Adc1::clear_flags(Adc1::flag_resrdy | Adc1::flag_overrun);
        Adc0::start();
        int who = -1;
        (void)brio::wait_until<Ruler>(   // the answer is `who`, -1 for none
            [&who] {
                if (Adc0::ready()) {
                    (void)Adc0::result();
                    (void)Adc1::result();
                    who = 0;
                    return true;
                }
                if (Adc1::ready()) {
                    (void)Adc1::result();
                    (void)Adc0::result();
                    who = 1;
                    return true;
                }
                return false;
            },
            adc_budget_us);
        return who;
    };

    (void)Adc0::stop_events();

    /// Eight consecutive software triggers as a pattern of bits: bit k
    /// is which converter answered trigger k; nothing when a trigger went
    /// unanswered.
    auto pattern_of = [&]() -> std::optional<uint8_t> {
        uint8_t bits = 0;
        bool all = true;
        for (uint8_t i = 0; i < 8u; ++i) {
            const int who = next_answering();
            if (who == 1) {
                bits = static_cast<uint8_t>(bits | (1u << i));
            } else if (who < 0) {
                all = false;
            }
        }
        return all ? std::optional<uint8_t>{bits} : std::nullopt;
    };
    auto print_pattern = [](const char* what, std::optional<uint8_t> p) {
        print(serial, "  ", what);
        if (!p) {
            print(serial, " (a trigger went unanswered)", crlf);
            return;
        }
        for (uint8_t i = 0; i < 8u; ++i) {
            print(serial, ' ', static_cast<char>(
                                   ((*p >> i) & 1u) != 0u ? '1' : '0'));
        }
        print(serial, crlf);
    };
    /// An interleaved pattern: every trigger answered, the converters
    /// strictly alternating.
    auto alternates = [](std::optional<uint8_t> p) {
        return p && (*p == 0x55u || *p == 0xAAu);
    };

    // WHICH converter a restart puts first is not the question. The
    // question is whether the restart puts the sequence in a DEFINED
    // state at all - which is what "restart an interleaved sequence"
    // means - so each option is applied from two DIFFERENT parities and
    // the two patterns that follow are compared. A restart that works
    // makes them identical; one that does not lets the parity through.
    // Both patterns must be whole and alternating for the comparison to
    // mean anything: two converters that never answered would match too.
    auto probe_restart = [&](uint8_t option, const char* name) -> bool {
        auto apply = [&]() {
            if (option == 0) {
                Adc0::flush();
                settle();
            } else if (option == 1) {
                interleave_up();
            } else {
                (void)Adc0::reset();
                (void)Adc0::init(main_gen, pad_cfg, main_gen_hz);
                interleave_up();
            }
        };
        apply();
        const std::optional<uint8_t> a = pattern_of();
        (void)next_answering();          // leave the parity the other way
        apply();
        const std::optional<uint8_t> b = pattern_of();
        print(serial, "  ", name, ":", crlf);
        print_pattern("    from one parity ", a);
        print_pattern("    from the other  ", b);
        return alternates(a) && alternates(b) && *a == *b;
    };

    interleave_up();
    (void)pattern_of();          // drain whatever the paced run left in flight
    const std::optional<uint8_t> plain = pattern_of();
    print_pattern("eight interleaved software triggers, answered by ADC", plain);
    bench.verdict("SOFTWARE TRIGGERS ALTERNATE STRICTLY - 38.8.10's 'start "
                  "event or software trigger' covers the software one, and "
                  "eight of them are answered alternately by the two "
                  "converters with no trigger going to the same one twice",
                  alternates(plain));

    const bool flush_ok = probe_restart(0, "SWTRIG.FLUSH on the host");
    const bool cycle_ok = probe_restart(1, "a disable/enable cycle of the host");
    const bool reset_ok = probe_restart(2, "a software reset of the host");
    bench.verdict("A SOFTWARE RESET OF THE HOST RESTARTS THE INTERLEAVED "
                  "SEQUENCE - the third of 38.6.3.1's options: after it the "
                  "same eight triggers answer the same way whichever parity "
                  "the sequence was left in",
                  reset_ok);
    // The other two options are printed, not judged: whether the flush
    // and the enable cycle restart the sequence on a die is that die's
    // fact (38.6.3.1 lists them; measured, the parity has carried through
    // both), and brio restarts an interleaved pair with the reset.
    print(serial, "  the other two options 38.6.3.1 lists: SWTRIG.FLUSH ",
          flush_ok ? "restarts" : "does NOT restart", " the sequence, a "
          "disable/enable cycle of the host ",
          cycle_ok ? "restarts" : "does NOT restart", " it", crlf);

    (void)Pacer::enable(false);
    (void)Adc0::enable(false);
    (void)Adc0::stop_events();
    Adc0::release();
    Adc1::release();
    Pacer::release();
    Counter::release();
    event_clock_down();
    hand_back<PadShared>();
}

// =============================================================================
// b - THE AUTOMATIC SEQUENCE (38.6.2.12)
// =============================================================================
//
// SEQCTRL is one bit per MUXPOS code and nothing has ever run one. The
// test is only worth making if every slot carries a DIFFERENT KNOWN
// value, so a swapped pair is caught rather than averaged away - which
// is why the six inputs here span two driven pads, the DAC, the bandgap
// and the two internal supply dividers.
void tb_sequence() {
    constexpr uint8_t seq_ain0 = 0;                                     // PA02 = the DAC
    constexpr uint8_t seq_ain1 = 1;                                     // PA03, driven LOW
    constexpr uint8_t seq_ain4 = 4;                                     // PA04, driven HIGH
    constexpr uint8_t seq_bandgap = static_cast<uint8_t>(AdcInput::intref);
    constexpr uint8_t seq_core = static_cast<uint8_t>(AdcInput::scaled_core);
    constexpr uint8_t seq_supply = static_cast<uint8_t>(AdcInput::scaled_supply);
    constexpr uint8_t order[6] = {seq_ain0, seq_ain1, seq_ain4,
                                  seq_bandgap, seq_core, seq_supply};
    constexpr uint32_t mask = (1UL << seq_ain0) | (1UL << seq_ain1) |
                              (1UL << seq_ain4) | (1UL << seq_bandgap) |
                              (1UL << seq_core) | (1UL << seq_supply);

    PadA3::output();
    PadA3::clear();
    PadA4::output();
    PadA4::set();

    // THE BANDGAP CHANNEL NEEDS TWO THINGS: SUPC.VREF.VREFOE (measured;
    // neither chapter states it) and table 45-22's 10 us of sampling.
    // SAMPLEN 20 at 1.5 MHz is 14 us.
    bench.verdict("the bandgap output is on and its level is 1.024 V",
                  Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                             .output_enable = true}) &&
                      Vref::output_enabled());
    AdcConfig cfg = pad_cfg;
    cfg.sample_length = 20;
    bench.verdict("ADC0 up with a sampling time long enough for the INTREF "
                  "channel", adc0_up(cfg));
    bench.verdict("the DAC is up on PA02", dac_up());
    dac_set(700);

    // The reference readings, one input at a time, with the sequencer
    // OFF: this is what the sequence has to reproduce.
    uint32_t single[6] = {};
    for (uint8_t i = 0; i < 6u; ++i) {
        Adc0::select(static_cast<AdcInput>(order[i]));
        (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
        settle();
        single[i] = mean_of<Adc0>(8);
    }
    print(serial, "  one at a time: AIN0(DAC 700) ", single[0], ", AIN1(GND) ",
          single[1], ", AIN4(VDD) ", single[2], ", bandgap ", single[3],
          ", VDDCORE/4 ", single[4], ", VDDANA/4 ", single[5], crlf);

    // Are the six really distinct? The verdict below is only a test of
    // the sequencer if they are, so the letter says so first.
    uint32_t closest = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 6u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 6u; ++j) {
            const uint32_t d = single[i] > single[j] ? single[i] - single[j]
                                                     : single[j] - single[i];
            if (d < closest) {
                closest = d;
            }
        }
    }
    print(serial, "  the closest two of the six are ", closest,
          " counts apart", crlf);
    // Two 8-reading means of one input differ by their noise: table
    // 45-24's 2 mV rms is 1 mV rms on such a difference, 4 mV at four
    // sigma, and a count of rounding in each. A match judged at half the
    // closest separation catches a swap only when that half clears it.
    const uint32_t same_input = mv_counts(4u) + 2u;
    bench.verdict("THE SIX SLOTS CARRY SIX DISTINCT VALUES, so a swapped pair "
                  "would be caught and not averaged away",
                  closest > 2u * same_input);

    // ---- the sequence itself ------------------------------------------------
    Adc0::sequence(mask);
    bench.verdict("SEQCTRL holds one bit per positive input, and the "
                  "internal channels are in it like any other",
                  Adc0::sequence() == mask);

    uint32_t got[6] = {};
    uint8_t state[6] = {};
    bool busy_seen = false;
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    Adc0::start();
    bool complete = true;
    for (uint8_t i = 0; i < 6u; ++i) {
        if (!adc0_lands()) {
            complete = false;
            break;
        }
        if (i < 5u && Adc0::sequence_busy()) {
            busy_seen = true;
        }
        state[i] = Adc0::sequence_state();
        got[i] = Adc0::result();
    }
    const bool busy_after = Adc0::sequence_busy();

    print(serial, "  the sequence returned ", got[0], " ", got[1], " ", got[2],
          " ", got[3], " ", got[4], " ", got[5], crlf);
    print(serial, "  SEQSTATE reported ", state[0], " ", state[1], " ",
          state[2], " ", state[3], " ", state[4], " ", state[5],
          " (the codes 0,1,4,25,26,27)", crlf);
    bench.verdict("ONE TRIGGER PRODUCED SIX RESULTS", complete);

    bool labels_ok = true;
    bool values_ok = true;
    for (uint8_t i = 0; i < 6u; ++i) {
        if (state[i] != order[i]) {
            labels_ok = false;
        }
        // Half the closest separation: a slot that was another input sits
        // at least `closest` away, the same input within its noise.
        if (!near(got[i], single[i], closest / 2u)) {
            values_ok = false;
        }
    }
    bench.verdict("...IN THE ORDER 38.6.2.12 STATES - lowest MUXPOS first, "
                  "and SEQSTATUS.SEQSTATE labels each result with the input it "
                  "came from",
                  labels_ok);
    bench.verdict("...and every one of the six matches what the same input "
                  "gave one at a time",
                  values_ok);
    bench.verdict("SEQBUSY stands while the sequence runs and is clear when it "
                  "ends", busy_seen && !busy_after);

    // 38.6.2.12's last sentence, as a control.
    Adc0::sequence(0);
    Adc0::select(AnalogIn<PadA4>{});
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    settle();
    const uint32_t plain = mean_of<Adc0>(8);
    print(serial, "  with SEQCTRL cleared the conversion follows MUXPOS again: ",
          plain, crlf);
    // The same input through the same converter, a sequence slot or
    // MUXPOS: a slot that was another input sits at least `closest`
    // away, so half of that tells them apart.
    bench.verdict("'if no bits are set the sequence is disabled' - the "
                  "conversion is MUXPOS's again",
                  near(plain, single[2], closest / 2u) && !Adc0::sequence_busy());

    // ---- what it costs ------------------------------------------------------
    bench.verdict("the crystal stopwatch is running", stopwatch_start());
    constexpr uint16_t rounds = 64;
    Adc0::sequence(0);
    Adc0::select(static_cast<AdcInput>(seq_supply));
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    Adc0::discard(2);
    // Both spans run back to back on a quiet console and print after: a
    // line going out puts its interrupts inside one of them.
    const bool quiet = console_drain();
    uint32_t t0 = ticks_now();
    for (uint16_t r = 0; r < rounds; ++r) {
        for (uint8_t i = 0; i < 6u; ++i) {
            (void)Adc0::read();
        }
    }
    const uint32_t singles_ticks = ticks_now() - t0;

    Adc0::sequence(mask);
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    // The sequencer paces its own six conversions, so the poll's ruler
    // read (about 1.5 us a result, util/test_wait.hpp) is inside the
    // 12 us each takes and does not stretch the sequence.
    bool seq_landed = true;
    t0 = ticks_now();
    for (uint16_t r = 0; r < rounds && seq_landed; ++r) {
        Adc0::start();
        for (uint8_t i = 0; i < 6u; ++i) {
            seq_landed = adc0_lands() && seq_landed;
            (void)Adc0::result();
        }
    }
    const uint32_t seq_ticks = ticks_now() - t0;

    const uint32_t per_single = ticks_us(singles_ticks * 1000u / (rounds * 6u));
    const uint32_t per_seq = ticks_us(seq_ticks * 1000u / (rounds * 6u));
    print(serial, "  ", rounds * 6u, " conversions cost ",
          ticks_us(singles_ticks), " us one software trigger at a time and ",
          ticks_us(seq_ticks), " us inside sequences of six - ", per_single,
          " vs ", per_seq, " NANOseconds a conversion", crlf);
    bench.verdict("THE SEQUENCE IS CHEAPER PER CONVERSION than the same "
                  "conversions triggered one at a time - the saving is the "
                  "trigger and the input change the sequencer does itself",
                  quiet && seq_landed && seq_ticks < singles_ticks);

    Adc0::sequence(0);
    Adc0::release();
    Dac::release();
    hand_back<PadA3>();
    hand_back<PadA4>();
}

// =============================================================================
// c - DIFFERENTIAL MODE, finally measurable
// =============================================================================
//
// adc.md's own words for why this was never done: "two pads at two
// different driven levels would only ever give a full-scale or a zero
// difference, which is not a test of anything". The DAC on PA02 is the
// answer - AIN0 is a swept mid-scale voltage now - so a difference can
// be small, large, and of either sign.
//
// 38.6.2.5's arithmetic, which the verdicts below check rather than
// assume: a differential RESULT is signed and full scale is +/-VREF, so
// a difference reads HALF of what the same two nodes read one at a time
// single-ended.
void tc_differential() {
    PadA3::output();
    PadA3::clear();

    bench.verdict("the DAC is up on PA02 and the ADC on the same pad",
                  dac_up() && adc0_up(pad_cfg));

    // The single-ended references first: this is the ruler the
    // differential readings are checked against.
    Adc0::select(AnalogIn<Vout>{});
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    dac_set(700);
    const uint32_t se_dac700 = mean_of<Adc0>(16);
    Adc0::select(AnalogIn<PadA3>{});
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    settle();
    const uint32_t se_a3_low = mean_of<Adc0>(16);
    PadA3::set();
    settle();
    const uint32_t se_a3_high = mean_of<Adc0>(16);
    Adc0::select(AdcInput::scaled_supply);
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    settle();
    const uint32_t se_quarter = mean_of<Adc0>(16);
    print(serial, "  single-ended: DAC(700) ", se_dac700, ", PA03 low ",
          se_a3_low, ", PA03 high ", se_a3_high, ", VDDANA/4 ", se_quarter,
          crlf);

    // ---- differential, both signs ------------------------------------------
    AdcConfig diff = pad_cfg;
    diff.differential = true;
    Adc0::release();
    bench.verdict("ADC0 comes back in DIFFERENTIAL mode", adc0_up(diff));
    Adc0::select(AnalogIn<Vout>{});                 // MUXPOS = AIN0 = the DAC
    bench.verdict("the negative multiplexer takes AIN1 and refuses a code "
                  "past AIN5",
                  Adc0::select_negative(AdcNegative::ain1) &&
                      !Adc0::select_negative(static_cast<AdcNegative>(6)));
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);

    PadA3::clear();
    dac_set(700);
    Adc0::discard(2);
    bool landed = true;   // every conversion below, folded into the verdicts
    int32_t sum = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        sum += Adc0::result_signed();
        Adc0::start();
        landed = adc0_lands() && landed;
    }
    const int32_t d_positive = (sum + 8) / 16;

    PadA3::set();
    settle();
    Adc0::discard(2);
    sum = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        sum += Adc0::result_signed();
        Adc0::start();
        landed = adc0_lands() && landed;
    }
    const int32_t d_negative = (sum + 8) / 16;

    const int32_t predict_pos =
        static_cast<int32_t>(se_dac700 - se_a3_low) / 2;
    const int32_t predict_neg =
        (static_cast<int32_t>(se_dac700) - static_cast<int32_t>(se_a3_high)) / 2;
    print(serial, "  differential AIN0 - AIN1: with PA03 at GND ", d_positive,
          " (predicted ", predict_pos, "), with PA03 at VDD ", d_negative,
          " (predicted ", predict_neg, ")", crlf);
    bench.verdict("THE SIGN IS THE DIFFERENCE'S SIGN - the same pair reads "
                  "positive with the negative input at ground and negative "
                  "with it at the supply",
                  landed && d_positive > 0 && d_negative < 0);
    // No table bounds these two readings: with CTRLC.R2R clear table
    // 45-22 wants the common mode within VREF/2 +-0.2 V, and these pairs
    // sit at about a third and five sixths of the supply. What the verdict
    // asks is the SCALE, and the scale's alternative is a factor of two
    // (a full scale of +/-VREF/2 would read the whole single-ended
    // difference): half the prediction tells the two apart. The
    // agreement itself is printed above.
    bench.verdict("AND THE MAGNITUDE IS HALF THE SINGLE-ENDED DIFFERENCE, "
                  "which is what a signed full scale of +/-VREF means",
                  near_signed(d_positive, predict_pos, predict_pos / 2) &&
                      near_signed(d_negative, predict_neg, -predict_neg / 2));

    // ---- a real zero crossing, swept ---------------------------------------
    //
    // MUXPOS is an INTERNAL channel here - VDDANA/4, a fixed mid-scale
    // node - and MUXNEG the DAC, so a sweep crosses zero inside one
    // arrangement instead of between two.
    Adc0::select(AdcInput::scaled_supply);
    (void)Adc0::select_negative(AdcNegative::ain0);
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
    PadA3::clear();

    int32_t swept[7] = {};
    constexpr uint16_t codes[7] = {0, 100, 200, 256, 300, 400, 600};
    for (uint8_t i = 0; i < 7u; ++i) {
        dac_set(codes[i]);
        Adc0::discard(3);
        int32_t s2 = 0;
        for (uint8_t k = 0; k < 8u; ++k) {
            s2 += Adc0::result_signed();
            Adc0::start();
            landed = adc0_lands() && landed;
        }
        swept[i] = (s2 + 4) / 8;
    }
    print(serial, "  VDDANA/4 minus the DAC, codes 0..600: ", swept[0], " ",
          swept[1], " ", swept[2], " ", swept[3], " ", swept[4], " ", swept[5],
          " ", swept[6], crlf);
    bool monotone = true;
    for (uint8_t i = 1; i < 7u; ++i) {
        if (swept[i] >= swept[i - 1]) {
            monotone = false;
        }
    }
    // Where each reading lands rests on the quarter-supply divider, whose
    // ratio no table gives, and on a common mode table 45-22 does not
    // cover with R2R clear (above). What the DAC alone decides is the
    // SPAN between the ends: 600 codes, 1200 counts of this signed scale
    // whatever the divider does, judged within half against the factor
    // of two a wrong scale would make. The ends are not rails (+2047,
    // -2048), and the sign changes between them.
    const int32_t span = swept[0] - swept[6];
    bench.verdict("A SWEPT DIFFERENCE FALLS MONOTONICALLY THROUGH ZERO as the "
                  "DAC passes the fixed node - neither end is a rail and the "
                  "span between them is the DAC's 600 codes",
                  landed && monotone && swept[0] > 0 && swept[6] < 0 &&
                      swept[0] < 2047 && swept[6] > -2048 &&
                      near_signed(span, 1200, 600));
    // VDDANA/4 is code 256 of 1024 by construction, so that is where the
    // difference would vanish with an exact divider: printed, not judged,
    // since the divider's ratio is in no table.
    print(serial, "  at the DAC code that is a quarter of the supply (256) the "
          "difference reads ", swept[3], " counts - the quarter-supply "
          "divider's own error, untabled", crlf);

    // ---- the signed window in differential mode ----------------------------
    //
    // 38.6.2.13: "If differential input is selected, the WINLT and WINUT
    // are evaluated as signed values." A threshold BELOW zero is
    // therefore only meaningful here.
    bench.verdict("a signed lower threshold of -200 is accepted",
                  Adc0::window_signed(AdcWindow::above_lower, -200, 2047));
    dac_set(100);                      // well above the threshold
    Adc0::discard(3);
    (void)Adc0::read();
    const bool hit_above = Adc0::window_hit();
    dac_set(600);                      // well below it
    Adc0::discard(3);
    (void)Adc0::read();
    const bool hit_below = Adc0::window_hit();
    print(serial, "  WINMODE 'RESULT > WINLT' with WINLT = -200: at a "
          "difference of about +", swept[1], " it fires ", yes_no(hit_above),
          ", at about ", swept[6], " it fires ", yes_no(hit_below), crlf);
    bench.verdict("THE THRESHOLD IS READ AS A SIGNED NUMBER - a negative WINLT "
                  "separates two differences that are both legal results and "
                  "would be indistinguishable read unsigned",
                  hit_above && !hit_below);
    (void)Adc0::window_off();

    // Erratum 1.4.7 named, since this is the letter that would meet it.
    print(serial, "  [erratum 1.4.7 - differential and single-ended electrical "
          "characteristics out of specification - is marked revisions B..E on "
          "the E/G/J row and is NOT this silicon]", crlf);

    Adc0::release();
    Dac::release();
    hand_back<PadA3>();
}

// =============================================================================
// d - RAIL-TO-RAIL AND OFFSET COMPENSATION
// =============================================================================
//
// 38.6.3.2: "The accuracy of the ADC is highest when the input common
// mode voltage is close to VREF/2. To enable a full range of common mode
// voltages, CTRLC.R2R should be written to one. Rail-to-rail operation
// requires a sampling period of four cycles. This is achieved by
// enabling offset compensation."
//
// So the thing to measure is a COMMON MODE, not a level: the same
// difference, once near mid-supply where the plain converter is at its
// best and twice near the rails where it is not. The DAC and one driven
// pad make all three.
void td_rail_to_rail() {
    bench.verdict("the crystal stopwatch is running", stopwatch_start());
    PadA3::output();
    bench.verdict("the DAC is up on PA02", dac_up());

    // The three configurations, each a legal one and each refused
    // without its companion.
    AdcConfig plain = pad_cfg;
    plain.differential = true;
    AdcConfig offcomp = pad_cfg;
    offcomp.differential = true;
    offcomp.sample_length = 0;
    offcomp.offset_compensation = true;
    AdcConfig r2r = offcomp;
    r2r.rail_to_rail = true;
    AdcConfig bad = pad_cfg;
    bad.rail_to_rail = true;              // no offset compensation
    AdcConfig bad2 = offcomp;
    bad2.sample_length = 5;               // OFFCOMP with a non-zero SAMPLEN
    bench.verdict("R2R without offset compensation and OFFCOMP with a non-zero "
                  "SAMPLEN are both refused - 38.6.3.2 and 38.8.12",
                  !Adc0::config_valid(bad) && !Adc0::config_valid(bad2) &&
                      Adc0::config_valid(offcomp) && Adc0::config_valid(r2r));

    /// One arrangement measured under one configuration: the DAC at
    /// `code` against PA03 at `high`, differential, as a mean AND a
    /// spread - because the whole letter turns on whether a difference
    /// between configurations is bigger than the reading's own scatter.
    struct Reading {
        int32_t mean;
        int32_t span;
    };
    auto measure = [](const AdcConfig& cfg, uint16_t code, bool high) -> Reading {
        Adc0::release();
        if (!adc0_up(cfg)) {
            return Reading{0x7FFFFFFF, 0};
        }
        Adc0::select(AnalogIn<Vout>{});
        (void)Adc0::select_negative(AdcNegative::ain1);
        (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
        if (high) {
            PadA3::set();
        } else {
            PadA3::clear();
        }
        dac_set(code);
        Adc0::discard(4);
        int32_t s2 = 0;
        int32_t lo = 32767;
        int32_t hi = -32768;
        for (uint8_t k = 0; k < 32u; ++k) {
            const int32_t v = Adc0::result_signed();
            s2 += v;
            if (v < lo) {
                lo = v;
            }
            if (v > hi) {
                hi = v;
            }
            Adc0::start();
            if (!adc0_lands()) {
                return Reading{0x7FFFFFFF, 0};   // as a converter that never came up
            }
        }
        return Reading{(s2 + 16) / 32, hi - lo};
    };

    // Three common modes. What changes between the rows is where the
    // pair sits: on a 5.2 V supply about 0.5 V, about 1.8 V (a third of
    // the supply, the row nearest VREF/2, where 38.6.3.2 says the plain
    // converter is at its best) and about 4.7 V.
    struct Row {
        const char* name;
        uint16_t code;
        bool high;
    };
    constexpr Row rows[3] = {
        {"near GND   (DAC 200 vs PA03 low) ", 200, false},
        {"mid supply (DAC 700 vs PA03 low) ", 700, false},
        {"near VDD   (DAC 824 vs PA03 high)", 824, true},
    };
    int32_t worst_shift = 0;
    int32_t worst_span = 0;
    for (uint8_t i = 0; i < 3u; ++i) {
        const Reading a = measure(plain, rows[i].code, rows[i].high);
        const Reading b = measure(offcomp, rows[i].code, rows[i].high);
        const Reading c = measure(r2r, rows[i].code, rows[i].high);
        print(serial, "  ", rows[i].name, "  plain ", a.mean, " (spread ",
              a.span, ")   +OFFCOMP ", b.mean, " (", b.span,
              ")   +OFFCOMP+R2R ", c.mean, " (", c.span, ")", crlf);
        const int32_t s1 = b.mean > a.mean ? b.mean - a.mean : a.mean - b.mean;
        const int32_t s2 = c.mean > a.mean ? c.mean - a.mean : a.mean - c.mean;
        if (s1 > worst_shift) {
            worst_shift = s1;
        }
        if (s2 > worst_shift) {
            worst_shift = s2;
        }
        for (const int32_t sp : {a.span, b.span, c.span}) {
            if (sp > worst_span) {
                worst_span = sp;
            }
        }
    }
    print(serial, "  across all three common modes the largest shift either "
          "knob produced is ", worst_shift,
          " counts of 4096, against a per-reading spread of up to ",
          worst_span, crlf);
    // Printed above, not judged: what either knob does to a reading on a
    // die is that die's fact - 38.6.3.2 promises better accuracy near the
    // rails, not a size - and the shifts sit at the reading's own scatter
    // (one count of 4096 with VREF = VDDANA), where a bound would judge
    // noise. A converter that does not convert fails the timing verdicts
    // below, which need conversions to time.

    // ---- what each one costs -----------------------------------------------
    //
    // With OFFCOMP the sampling period is FIXED at four cycles, so
    // adc_sample_cycles() charges 16 where this differential
    // configuration's 13 + SAMPLEN is 18 - i.e. the compensation is
    // SHORTER here, not longer, because it replaces a five-cycle sample
    // with a four-cycle one. The stopwatch is asked for the DIFFERENCE
    // and not for the absolute, since a software read loop's own
    // overhead sits in both.
    auto time_of = [](const AdcConfig& cfg) -> uint32_t {
        Adc0::release();
        if (!adc0_up(cfg)) {
            return 0;
        }
        Adc0::select(AnalogIn<Vout>{});
        (void)Adc0::select_negative(AdcNegative::ain1);
        (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);
        Adc0::discard(4);
        // A quiet console, or no figure: a line going out would add its
        // interrupts to one configuration's span and not another's.
        if (!console_drain()) {
            return 0;
        }
        const uint32_t t0 = ticks_now();
        for (uint16_t k = 0; k < 256u; ++k) {
            (void)Adc0::read();
        }
        return ticks_now() - t0;
    };
    const uint32_t t_plain = time_of(plain);
    const uint32_t t_off = time_of(offcomp);
    const uint32_t t_r2r = time_of(r2r);
    // One CLK_ADC cycle at 1.5 MHz is 16 crystal ticks.
    print(serial, "  256 conversions: plain ", t_plain, " ticks (",
          t_plain / 256u, " a conversion, ", t_plain / 256u / 16u,
          " CLK_ADC cycles), +OFFCOMP ", t_off, " (", t_off / 256u / 16u,
          "), +R2R ", t_r2r, " (", t_r2r / 256u / 16u, ")", crlf);
    const int32_t saved_ticks =
        (static_cast<int32_t>(t_plain) - static_cast<int32_t>(t_off) + 128) / 256;
    print(serial, "  the difference is ", saved_ticks,
          " crystal ticks a conversion = ", saved_ticks * 10 / 16,
          " tenths of a CLK_ADC cycle, where the driver's own arithmetic "
          "predicts ", adc_conversion_cycles(plain), " - ",
          adc_conversion_cycles(offcomp), " = ",
          adc_conversion_cycles(plain) - adc_conversion_cycles(offcomp),
          " (both absolutes carry the same software read loop, which is why "
          "only their difference is judged)", crlf);
    bench.verdict("OFFSET COMPENSATION IS A FIXED FOUR-CYCLE SAMPLING PERIOD "
                  "AND THAT MAKES IT SHORTER HERE, NOT LONGER - it REPLACES "
                  "SAMPLEN rather than adding to it, and the saving measured "
                  "against the crystal is the two CLK_ADC cycles table 45-22's "
                  "OFFCOMP row implies against a five-cycle sample",
                  // two CLK_ADC cycles are 32 crystal ticks at a nominal
                  // OSC48M (CLK_ADC is OSC48M / 32), within its 5 % (table
                  // 45-57: 1.6 ticks) and a tick of the rounding: 29..35
                  saved_ticks >= 29 && saved_ticks <= 35);
    bench.verdict("...and R2R on top of it costs nothing at all: it changes "
                  "the input stage, not the timing",
                  near(t_r2r, t_off, t_off / 64u + 1u));

    Adc0::release();
    Dac::release();
    hand_back<PadA3>();
}

// =============================================================================
// e - DAC DITHERING (41.6.8.4), and CTRLB.LEFTADJ on silicon
// =============================================================================
//
// The chapter makes the sixteen sub-conversions THE EVENT'S JOB: "the
// STARTEI event must be configured to generate 16 events for each
// DATA[13:0] conversion, and DATABUF must be loaded every 16 DAC
// conversions". So dithering cannot be driven from the CPU at all, which
// is why dac_config_valid() refuses it without the start event and why
// this is the first letter that can run it.
//
// THE WITNESS HAS TO AVERAGE OVER WHOLE DITHER PERIODS, and the numbers
// were chosen so that it does EXACTLY:
//   pacer   TC2, count8, div4, PER = 255  -> one event every 1024 CPU cycles
//   dither  16 events                     -> one period every 16384 cycles
//   ADC     div32, SAMPLEN 5              -> 18 CLK_ADC = 576 CPU cycles
//   average 1024 samples                  -> 589824 cycles = 36 periods
// 576 / 1024 = 9/16 and 9 is coprime with 256, so the 1024 samples land
// on each of the sixteen sub-conversion slots exactly 64 times. The mean is
// the dithered mean and not a phase artefact.
//
// The scale that makes it visible: 1024 accumulated 12-bit samples with
// ADJRES 0 is a 16-bit result, so ONE DAC LSB IS 64 COUNTS and one
// sixteenth of an LSB is 4. That is the whole point - the effect is
// under the ADC's own LSB and only the accumulation can see it.
//
// THE FEED IS THE DAC'S OWN EMPTY INTERRUPT, so the CPU is in the path
// for one DATABUF write a value. 41.6.8.4: "EMPTYx event and DMA request
// are therefore generated every 16 DATABUF to DATA transfers", and it
// names an interrupt on EMPTY as the way to feed it without the DMA -
// but THE DEADLINE IS ONE EVENT, NOT SIXTEEN: measured, an UNDERRUN
// follows the EMPTY by one start event (278 cycles at a 256-cycle pacer)
// whenever the handler has not written DATABUF by then. So the pacer is
// slowed to one event every 1024 cycles, where the feeding path - about
// 105 cycles in the release listing (two calls, into Dac::buffer() and
// the placement arithmetic of dac_data_word(), one APB read pair and
// one APB write) plus the core's exception entry and return, plus a
// console or tick handler it may wait behind - fits with room. The
// letter judges the deadline met by UNDERRUN staying at zero across
// every accumulation, and prints the EMPTY count against the elapsed
// events.
constexpr AdcConfig dither_witness{
    .reference = Ref::vddana,
    .prescaler = AdcPresc::div32,
    .resolution = AdcRes::bits16,
    .average = AdcAverage::samples1024,
    .adjust = 0,
    .sample_length = 5,
};

/// One accumulated reading: 1024 samples, 12.3 ms.
uint32_t accumulate() {
    Adc0::discard(1);
    return Adc0::read(4'000'000UL);
}

/// Stop the interrupt feed: nothing refills DATABUF after this.
void dither_feed_stop() {
    Nvic::disable(Dac::irq());
    Dac::disarm(Dac::flag_empty | Dac::flag_underrun);
    dac_feed_from_isr = false;
}

bool dither_chain_up(bool left_adjust) {
    dither_feed_stop();
    Dac::release();
    DacConfig cfg{};
    cfg.reference = DacRef::vddana;
    cfg.external_output = true;
    cfg.internal_output = true;
    cfg.left_adjust = left_adjust;
    cfg.dither = true;
    cfg.events.start_in = true;
    if (!Dac::init(main_gen, cfg)) {
        return false;
    }
    if (!Dac::enable(false)) {
        return false;
    }
    if (!Dac::start_on(ev_a, EventChannelConfig{
                                 .generator = Pacer::overflow_generator,
                                 .path = EventPath::asynchronous})) {
        return false;
    }
    return Dac::enable(true);
}

/**
 * Start the interrupt feed on one 14-bit value: DAC_Handler writes it
 * into DATABUF at every EMPTY, and the counters it keeps start at zero.
 * THE FIRST VALUE IS WRITTEN HERE, by the starter, because EMPTY is an
 * EVENT and not a state - 41.8.6 sets it when DATABUF is consumed, so a
 * converter just enabled whose buffer was never written reads it clear,
 * and a feed that waited for the flag would never begin. Called with
 * the pacer stopped and the converter fresh from dither_chain_up().
 */
void dither_feed_start(uint16_t value14) {
    dither_feed_stop();
    dac_feed_value = value14;
    dac_empty_irqs = 0;
    dac_underrun_irqs = 0;
    dac_irq_entries = 0;
    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    Dac::buffer(value14);
    dac_feed_from_isr = true;
    Dac::arm(Dac::flag_empty | Dac::flag_underrun);
    Nvic::enable(Dac::irq());
}

/// A new value for the running feed: the handler writes it at the next
/// EMPTY, so DATA carries it within one dither period, and the
/// accumulation that follows discards a whole reading first.
void dither_value(uint16_t value14) { dac_feed_value = value14; }

void te_dither() {
    bench.verdict("dithering without a start event is refused, because "
                  "41.6.8.4 makes the sixteen sub-conversions the event's job",
                  !dac_config_valid(DacConfig{.dither = true}) &&
                      dac_config_valid(DacConfig{
                          .dither = true, .events = {.start_in = true}}));
    bench.verdict("table 41-1's placement puts a 14-bit value where the two "
                  "adjustment bits say",
                  dac_data_word(0x1234, false, true) == 0x1234u &&
                      dac_data_word(0x1234, true, true) == 0x48D0u &&
                      dac_data_word(700, false, false) == 700u &&
                      dac_data_word(700, true, false) == (700u << 6));

    bench.verdict("the event fabric is up", event_clock_up());
    const TcConfig pacer_cfg{.mode = TcMode::count8,
                             .prescaler = TcPrescaler::div4,
                             .waveform = TcWaveform::normal_pwm};
    bench.verdict("TC2 paces one event every 1024 CPU cycles - 46.875 kHz, so a "
                  "14-bit value lasts 16384 cycles, the DAC well inside its "
                  "350 ksps ceiling and its EMPTY handler inside one event",
                  Pacer::init(main_gen) && Pacer::configure(pacer_cfg) &&
                      Pacer::set_period8(255) &&
                      Pacer::event_config(pacer_cfg,
                                          TcEventConfig{.overflow_out = true}));
    bench.verdict("the DAC comes up DITHERING, with the pacer on its START "
                  "user", dither_chain_up(false));
    bench.verdict("the ADC watches PA02 with a 1024-sample accumulation, whose "
                  "window is exactly 36 dither periods",
                  adc0_up(dither_witness) && Adc0::result_steps() == 65536u);
    Adc0::select(AnalogIn<Vout>{});
    (void)Adc0::sync_wait(ADC_SYNCBUSY_INPUTCTRL_Msk);

    // ---- the noise, BEFORE any band is chosen -------------------------------
    constexpr uint16_t base_code = 512;
    dither_feed_start(base_code * 16u);
    (void)Pacer::set_count8(0);
    (void)Pacer::enable(true);
    brio::spin_us<Ruler>(2'000);
    bench.verdict("THE DAC'S OWN EMPTY INTERRUPT FEEDS DATABUF the same 14-bit "
                  "value every time the buffer empties - the starter wrote the "
                  "first one, the vector every one after it, and no start "
                  "event found the buffer empty",
                  dac_empty_irqs > 0u && dac_underrun_irqs == 0u);
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    const uint32_t fed0 = dac_empty_irqs;
    const uint32_t ms0 = Ticker::millis();
    for (uint8_t i = 0; i < 6u; ++i) {
        const uint32_t v = accumulate();
        if (v < lo) {
            lo = v;
        }
        if (v > hi) {
            hi = v;
        }
    }
    const uint32_t fed = dac_empty_irqs - fed0;
    const uint32_t fed_ms = Ticker::millis() - ms0;
    const uint32_t fed_events = fed_ms * 46'875UL / 1000UL;
    const uint32_t noise = hi - lo;
    print(serial, "  six accumulated readings of one dithered value span ",
          noise, " counts of a 16-bit scale, where ONE DAC LSB IS 64 and one "
          "sixteenth of one is 4", crlf);
    print(serial, "  meanwhile the vector fed DATABUF ", fed, " times in ",
          fed_ms, " ms of about ", fed_events, " start events: one EMPTY per ",
          fed != 0u ? fed_events / fed : 0u, " events, where 41.6.8.4's "
          "sixteen sub-conversions a value say 16", crlf);
    // The premise of the staircase below: six readings' span under six
    // sub-LSB steps (24 counts), or the means cannot be told apart.
    bench.verdict("THE NOISE IS MEASURED FIRST and it is smaller than the "
                  "sub-LSB step this letter is about",
                  noise < 24u);

    // ---- the sub-LSB staircase ----------------------------------------------
    uint32_t means[5] = {};
    constexpr uint8_t dvals[5] = {0, 4, 8, 12, 15};
    for (uint8_t i = 0; i < 5u; ++i) {
        dither_value(static_cast<uint16_t>(base_code * 16u + dvals[i]));
        means[i] = accumulate();
    }
    const uint32_t right_underruns = dac_underrun_irqs;
    print(serial, "  dithered means for DATA[3:0] = 0 / 4 / 8 / 12 / 15: ",
          means[0], " ", means[1], " ", means[2], " ", means[3], " ", means[4],
          crlf);
    const int32_t swing =
        static_cast<int32_t>(means[4]) - static_cast<int32_t>(means[0]);
    print(serial, "  the whole dither range moved the mean by ", swing,
          " counts where fifteen sixteenths of an LSB is 60", crlf);
    bool rising = true;
    for (uint8_t i = 1; i < 5u; ++i) {
        if (means[i] <= means[i - 1]) {
            rising = false;
        }
    }
    bench.verdict("THE DITHERED MEAN MOVES IN SUB-LSB STEPS - five settings of "
                  "the four dither bits give five means, each above the last, "
                  "all inside ONE code of the plain converter",
                  rising);
    // Fifteen sixteenths of a DAC LSB is 60 counts here; the DAC's DNL at a
    // VDDANA reference (0.3 LSB, table 45-32: 19 counts) and the
    // readings' own noise (measured above) and two counts of rounding.
    const int32_t dither_tol = 19 + static_cast<int32_t>(noise) + 2;
    bench.verdict("...and the whole range is fifteen sixteenths of one LSB, "
                  "which is what 41.6.8.4's sixteen sub-conversions can add up "
                  "to and nothing else is",
                  swing > 60 - dither_tol && swing < 60 + dither_tol);
    print(serial, "  over the whole right-adjusted measurement: ",
          dac_empty_irqs, " EMPTY interrupts, ", right_underruns,
          " UNDERRUN", crlf);
    bench.verdict("...AND THE FEED NEVER FELL BEHIND: no start event found "
                  "DATABUF empty while the means were taken, so every "
                  "accumulation saw the value it was given",
                  right_underruns == 0u);

    // ---- the control: undithered, the same two codes ------------------------
    (void)Pacer::enable(false);
    dither_feed_stop();
    Dac::release();
    bench.verdict("the DAC comes back with dithering OFF", dac_up());
    dac_set(base_code);
    const uint32_t plain_c = accumulate();
    dac_set(static_cast<uint16_t>(base_code + 1u));
    const uint32_t plain_c1 = accumulate();
    print(serial, "  undithered codes ", base_code, " and ", base_code + 1u,
          " read ", plain_c, " and ", plain_c1, " - one LSB apart, with "
          "nothing between them", crlf);
    const int32_t step =
        static_cast<int32_t>(plain_c1) - static_cast<int32_t>(plain_c);
    // One DAC LSB is 64 counts, within the same DNL and noise.
    bench.verdict("THE CONTROL: without dithering the same converter steps a "
                  "WHOLE LSB and can put nothing in between",
                  step > 64 - dither_tol && step < 64 + dither_tol);
    bench.verdict("...and the dithered swing is a fraction of that step, "
                  "measured on the same instrument in the same letter",
                  swing < step);

    // ---- CTRLB.LEFTADJ on silicon, dithered and plain ----------------------
    //
    // dac_data_word()'s placements are pinned by the family fixture;
    // this is where the pad judges them. A left-adjusted code must put
    // the SAME voltage on the pad as the right-adjusted one.
    Dac::release();
    bench.verdict("the DAC comes up LEFT-ADJUSTED", dac_up(true, true, true));
    dac_set(base_code);
    const uint32_t left_plain = accumulate();
    print(serial, "  code ", base_code, " right-adjusted reads ", plain_c,
          " and left-adjusted ", left_plain, crlf);
    bench.verdict("CTRLB.LEFTADJ IS A PLACEMENT AND NOT A SCALE - the same "
                  "code puts the same voltage on the pad through both "
                  "arrangements of table 41-1",
                  // one voltage read twice: the readings' noise (measured
                  // above) and two counts; a scale would be a factor of 16
                  near(left_plain, plain_c, noise + 2u));

    // ERRATUM 1.9.1 says dithering with RIGHT-adjusted data gives an INL
    // of 16 LSB, and its E/G/J row carries one X and it is under B. The
    // staircase above was right-adjusted; this is the same staircase
    // left-adjusted, so the two can be compared.
    (void)Pacer::enable(false);
    bench.verdict("the dithering chain comes back, left-adjusted this time",
                  dither_chain_up(true));
    dither_feed_start(static_cast<uint16_t>(base_code * 16u + dvals[0]));
    (void)Pacer::set_count8(0);
    (void)Pacer::enable(true);
    uint32_t lmeans[5] = {};
    for (uint8_t i = 0; i < 5u; ++i) {
        dither_value(static_cast<uint16_t>(base_code * 16u + dvals[i]));
        lmeans[i] = accumulate();
    }
    const uint32_t left_underruns = dac_underrun_irqs;
    const int32_t lswing =
        static_cast<int32_t>(lmeans[4]) - static_cast<int32_t>(lmeans[0]);
    print(serial, "  the same staircase LEFT-adjusted: ", lmeans[0], " ",
          lmeans[1], " ", lmeans[2], " ", lmeans[3], " ", lmeans[4],
          " - swing ", lswing, " against the right-adjusted ", swing, crlf);
    // Two swings are four accumulated readings, each within the noise
    // measured above of its own level: twice that noise and two counts
    // of rounding.
    bench.verdict("ERRATUM 1.9.1 IS NOT THIS SILICON, AND THE ROW SAYS SO "
                  "BEFORE THE BENCH DOES (E/G/J revision B alone): the two "
                  "adjustments give the same dithered staircase, where a 16 "
                  "LSB nonlinearity in one of them could not hide in a swing "
                  "of one",
                  near_signed(lswing, swing, 2 * static_cast<int32_t>(noise) + 2));
    bench.verdict("and the left-adjusted feed never fell behind either: zero "
                  "UNDERRUN",
                  left_underruns == 0u);

    (void)Pacer::enable(false);
    dither_feed_stop();
    (void)Dac::enable(false);
    (void)Dac::stop_events();
    Dac::release();
    Adc0::release();
    Pacer::release();
    Counter::release();
    event_clock_down();
}

// =============================================================================
// f - THE DAC'S INTERRUPTS THROUGH THE NVIC
// =============================================================================
//
// dac.md: "EMPTY and UNDERRUN are read, cleared and used as verdicts;
// neither has ever driven the NVIC, and isr() is compile-verified only."
// The vector name is the DEVICE HEADER'S: the header, not CMSIS habit,
// is the authority, and a name this table does not carry links happily
// and lands in Default_Handler's silent spin.
void tf_dac_interrupts() {
    bench.verdict("the event fabric is up", event_clock_up());
    const TcConfig pacer_cfg{.mode = TcMode::count8,
                             .prescaler = TcPrescaler::div1024,
                             .waveform = TcWaveform::normal_pwm};
    bench.verdict("TC2 paces slowly enough that every interrupt is countable "
                  "by hand - about 1 kHz",
                  Pacer::init(main_gen) && Pacer::configure(pacer_cfg) &&
                      Pacer::set_period8(45) &&
                      Pacer::event_config(pacer_cfg,
                                          TcEventConfig{.overflow_out = true}));

    Dac::release();
    DacConfig cfg{};
    cfg.reference = DacRef::vddana;
    cfg.external_output = true;
    cfg.internal_output = true;
    cfg.events.start_in = true;
    bench.verdict("the DAC comes up with the pacer on its START user",
                  Dac::init(main_gen, cfg) && Dac::enable(false) &&
                      Dac::start_on(ev_a,
                                    EventChannelConfig{
                                        .generator = Pacer::overflow_generator,
                                        .path = EventPath::asynchronous}) &&
                      Dac::enable(true));

    dac_empty_irqs = 0;
    dac_underrun_irqs = 0;
    dac_irq_entries = 0;
    dac_feed_from_isr = true;
    dac_feed_value = 400;
    Dac::clear_flags(Dac::flag_empty | Dac::flag_underrun);
    Dac::buffer(400);
    Dac::arm(Dac::flag_empty | Dac::flag_underrun);
    Nvic::enable(Dac::irq());
    (void)Pacer::set_count8(0);
    (void)Pacer::enable(true);
    brio::spin_us<Ruler>(40'000);
    const uint32_t empties = dac_empty_irqs;
    const uint32_t entries = dac_irq_entries;
    print(serial, "  40 ms of start events with the handler refilling DATABUF: ",
          entries, " vector entries, ", empties, " EMPTY, ", dac_underrun_irqs,
          " UNDERRUN", crlf);
    // One EMPTY a start event: the pacer's period is 46 x 1024 = 47104
    // CPU cycles and the spin 40 ms of the same OSC48M, so 40.76 periods:
    // 40 events, 41 if the spin overran by a quarter period.
    constexpr uint32_t pacer_cycles = (45u + 1u) * 1024u;
    constexpr uint32_t fed_events =
        40'000u * (SysClock::hz / 1'000'000u) / pacer_cycles;
    bench.verdict("THE VECTOR IS BOUND AND THE FLAG REACHES IT - the DAC's "
                  "EMPTY drove the NVIC once a start event, and the handler "
                  "kept the buffer fed",
                  empties >= fed_events && empties <= fed_events + 1u &&
                      dac_underrun_irqs == 0u);

    // Now stop feeding: the next start event finds DATABUF empty.
    dac_feed_from_isr = false;
    Dac::clear_flags(Dac::flag_underrun);
    dac_underrun_irqs = 0;
    // EMPTY cannot be cleared by the handler any more, so disarm it -
    // 41.8.6: only writing DATABUF or writing a one clears it, and
    // isr() deliberately does NEITHER for EMPTY.
    Dac::disarm(Dac::flag_empty);
    brio::spin_us<Ruler>(20'000);
    const uint32_t unders = dac_underrun_irqs;
    print(serial, "  with the handler no longer feeding it: ", unders,
          " UNDERRUN interrupts", crlf);
    // 20 ms is 20.38 periods, 20 or 21 events, the first of which may
    // still find the last value the handler wrote: 19..21 UNDERRUN.
    constexpr uint32_t starved_events =
        20'000u * (SysClock::hz / 1'000'000u) / pacer_cycles;
    bench.verdict("UNDERRUN DRIVES THE VECTOR TOO, and it exists only in the "
                  "event-driven shape - a start event arriving with DATABUF "
                  "empty",
                  unders + 1u >= starved_events && unders <= starved_events + 1u);
    bench.verdict("...and the ISR body's own contract holds: it CLEARS "
                  "UNDERRUN, because nothing else carries that information, "
                  "and leaves EMPTY to whoever feeds the buffer",
                  !Dac::underrun());

    (void)Pacer::enable(false);
    Nvic::disable(Dac::irq());
    Dac::disarm(Dac::flag_empty | Dac::flag_underrun);
    (void)Dac::enable(false);
    (void)Dac::stop_events();
    Dac::release();
    Pacer::release();
    Counter::release();
    event_clock_down();
}

// =============================================================================
// g - THE AC COMPLETED: COMP2, COMP3 AND WINDOW 1 ON SILICON
// =============================================================================
//
// ac.md: "COMP0 and COMP1 carry every measured fact... no silicon has
// run window 1." The pair owns the pads - COMP2/3 take AIN[4..7] - and
// AIN4 IS PA02, the DAC's own output pad. So window 1 gets something
// window 0 never had: a SIGNAL THAT REALLY SITS BETWEEN THE LIMITS,
// swept, so the window's three states are reached by moving the signal
// rather than by swapping the roles of signal and limits.
void tg_comp23_window1() {
    bench.verdict("the pair owns the pads: COMP2/3's PIN0 is AIN4 and this "
                  "package bonds AIN4 through AIN7",
                  ac_ain_of(2, 0) == 4u && ac_ain_of(3, 0) == 4u &&
                      ac_ain_exists(4) && ac_ain_exists(5) &&
                      ac_ain_exists(6) && ac_ain_exists(7));
    bench.verdict("...and AIN4 is PA02, which is also the DAC's output pad",
                  Dac::vout_function('A', 2) == static_cast<int>(PinFunction::b));

    bench.verdict("the DAC is up on PA02 and the AC block on GCLK 0",
                  dac_up() && Ac::init(main_gen));

    // COMP2 and COMP3 both watch PA02; their own VDD scalers are the two
    // limits. 40.6.4 asks for the same measurement mode and the same
    // positive input on both, which pair_consistent() asks the silicon.
    constexpr uint8_t low_step = 15;    // VDD x 16/64
    constexpr uint8_t high_step = 39;   // VDD x 40/64
    const AcConfig pair{.positive = AcPositive::pin0,
                        .negative = AcNegative::vscale,
                        .speed = AcSpeed::high};
    bench.verdict("COMP2 and COMP3 take the configuration, each with its own "
                  "scaler",
                  Comp2::configure(pair) && Comp3::configure(pair));
    Comp2::scaler(low_step);
    Comp3::scaler(high_step);
    bench.verdict("both comparators come up", Comp2::enable(true) &&
                                                  Comp3::enable(true));
    settle();
    bench.verdict("both report READY", Comp2::ready() && Comp3::ready());
    bench.verdict("40.6.4'S TWO UNENFORCED REQUIREMENTS HOLD, asked of the "
                  "silicon rather than of a struct",
                  Window1::pair_consistent());

    // The individual comparators first: COMP2 and COMP3 have never run.
    dac_set(100);
    const bool c2_low = Comp2::state();
    const bool c3_low = Comp3::state();
    dac_set(900);
    const bool c2_high = Comp2::state();
    const bool c3_high = Comp3::state();
    dac_set(500);
    const bool c2_mid = Comp2::state();
    const bool c3_mid = Comp3::state();
    print(serial, "  COMP2 (scaler ", low_step + 1, "/64) and COMP3 (",
          high_step + 1, "/64) with the DAC at 100 / 500 / 900: ",
          yes_no(c2_low), yes_no(c3_low), " / ", yes_no(c2_mid),
          yes_no(c3_mid), " / ", yes_no(c2_high), yes_no(c3_high), crlf);
    bench.verdict("COMP2 AND COMP3 RUN, and each follows its own threshold: "
                  "the DAC below both, between them, and above both gives "
                  "three different pairs of states",
                  !c2_low && !c3_low && c2_mid && !c3_mid && c2_high && c3_high);

    // ---- window 1 -----------------------------------------------------------
    bench.verdict("window 1 turns on under a running block - WINCTRL is "
                  "write-synchronized and NOT enable-protected",
                  Window1::configure(true, AcWindowInterrupt::inside) &&
                      Window1::enabled());
    bench.verdict("the window is ready once both its comparators are",
                  Window1::ready());

    dac_set(100);
    const AcWindowState below = Window1::state();
    dac_set(500);
    const AcWindowState inside = Window1::state();
    dac_set(900);
    const AcWindowState above = Window1::state();
    print(serial, "  WSTATE1 at DAC 100 / 500 / 900: ",
          static_cast<uint8_t>(below), " / ", static_cast<uint8_t>(inside),
          " / ", static_cast<uint8_t>(above),
          " (0 = above, 1 = inside, 2 = below)", crlf);
    bench.verdict("WINDOW 1 REACHES ALL THREE STATES ON A REAL SWEPT SIGNAL - "
                  "the pair's shared positive input is the DAC and the two "
                  "limits are the comparators' own scalers, which is the "
                  "arrangement 40.6.4 actually describes",
                  below == AcWindowState::below &&
                      inside == AcWindowState::inside &&
                      above == AcWindowState::above);

    // The interrupt condition, and its silence outside it.
    Window1::clear_flag();
    dac_set(100);
    Window1::clear_flag();
    dac_set(500);
    const bool fired_inside = Window1::flag_set();
    Window1::clear_flag();
    dac_set(900);
    const bool fired_leaving = Window1::flag_set();
    print(serial, "  WINTSEL = INSIDE: entering the band fires ",
          yes_no(fired_inside), ", leaving it upward fires ",
          yes_no(fired_leaving), crlf);
    bench.verdict("WINTSEL picks ONE of the four conditions - entering the "
                  "band raises the flag and leaving it does not",
                  fired_inside && !fired_leaving);

    // The comparators keep their own flags and states throughout, which
    // 40.6.4 promises and nothing had checked on this pair.
    bench.verdict("the individual comparators are unaffected by window mode - "
                  "their own STATE still follows their own thresholds",
                  Comp2::state() && Comp3::state());

    (void)Window1::configure(false, AcWindowInterrupt::inside);
    (void)Comp2::enable(false);
    (void)Comp3::enable(false);
    Ac::release();
    Dac::release();
}

// =============================================================================
// h - THE BANDGAP AS A NEGATIVE INPUT, VREFOE, AND ERRATUM 1.5.6
// =============================================================================
//
// ac.md's gap, verbatim: "the bandgap as a negative input, whose INTREF
// output has to be turned on in SUPC.VREF first (22.6.2.2) and whose
// level is selected there too - and which is the input erratum 1.5.6 is
// about, so that erratum is stated and not exercised here."
//
// Three things, then: does the input work; does it need VREFOE; and does
// enabling on it raise the spurious flag the erratum promises - with a
// CONTROL, because a flag seen once is not an erratum reproduced.
//
// This is also the letter that binds AC_Handler and reads the flags
// through Ac::take_flags() from a real vector, which nothing had done.
void th_bandgap() {
    bench.verdict("the DAC is up on PA02 and the AC block with it",
                  dac_up() && Ac::init(main_gen));

    /// Sweep the DAC upward on COMP2's PIN0 against a fixed negative
    /// input, and report the code at which the output falls. Returns
    /// 0xFFFF if it never does.
    auto crossing = [](AcNegative neg, uint16_t from, uint16_t to) -> uint16_t {
        const AcConfig cfg{.positive = AcPositive::pin0,
                           .negative = neg,
                           .speed = AcSpeed::high};
        if (!Comp2::configure(cfg) || !Comp2::enable(true)) {
            return 0xFFFFu;
        }
        brio::spin_us<Ruler>(800);
        Comp2::clear_flag();
        uint16_t found = 0xFFFFu;
        for (uint16_t c = from; c <= to; ++c) {
            (void)Dac::set(c);
            settle_fast();
            if (Comp2::state()) {
                found = c;
                break;
            }
        }
        (void)Comp2::enable(false);
        return found;
    };

    // ---- does it need VREFOE? -----------------------------------------------
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                     .output_enable = false});
    settle_long();
    const bool oe_off = Vref::output_enabled();
    const uint16_t off_cross = crossing(AcNegative::bandgap, 0, 1023);
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                     .output_enable = true});
    settle_long();
    const uint16_t on_cross = crossing(AcNegative::bandgap, 0, 1023);
    print(serial, "  MUXNEG = bandgap at 1.024 V: with VREFOE CLEAR (read back ",
          yes_no(oe_off), ") the comparator flips at DAC code ", off_cross,
          ", with it SET at ", on_cross, crlf);
    // The level, 1003..1045 mV (table 45-36), against the DAC's offset
    // and gain error (20 mV, and 20 mV at full scale, table 45-32: 4..8
    // mV at these codes) and the comparator's offset (high speed,
    // -12..+20 mV, table 45-34, taken as +-20): 959..1093 mV, codes
    // 1024 x mV / VDDANA over the device's 2.7..5.5 V (table 45-2) with
    // the DAC's INL (0.9 LSB) and the first code past the threshold:
    // 177..417.
    constexpr uint16_t bandgap_code_lo = 177;
    constexpr uint16_t bandgap_code_hi = 417;
    auto at_bandgap = [](uint16_t code) {
        return code >= bandgap_code_lo && code <= bandgap_code_hi;
    };
    bench.verdict("THE BANDGAP REACHES A COMPARATOR'S NEGATIVE INPUT AT ALL - "
                  "the output really does flip at the code where the DAC "
                  "makes about a volt, and not somewhere a floating input "
                  "would put it",
                  at_bandgap(on_cross));
    bench.verdict("...AND IT DOES NOT NEED SUPC.VREF.VREFOE, WHICH CORRECTS "
                  "ac.hpp's OWN COMMENT: the crossing is the same code with "
                  "the bit clear and set, so the comparator's reference "
                  "multiplexer takes the bandgap internally exactly as the "
                  "DAC's and the ADC's REFERENCE paths do - only the ADC's "
                  "bandgap INPUT CHANNEL is dead without the bit, which is the "
                  "distinction dac.md drew and nobody had carried across to "
                  "40.6.3",
                  // with the bit clear the crossing lies in the same band
                  // the level puts it in; how far apart the two crossings
                  // are is printed above
                  !oe_off && at_bandgap(off_cross));

    // ---- and what is it worth? ---------------------------------------------
    //
    // A THIRD independent route to the same bandgap: the AC's own scaler
    // measured it, the ADC measured it, and this is the DAC measuring it
    // through a comparator.
    // The DAC cannot rise above VDDANA, so a level the supply does not
    // hold is never crossed: at 3.3 V the 4.096 V level is skipped.
    const bool located = need_vdd();
    const bool top_held = located && level_below_supply(4096u, vdd_mv);
    uint16_t levels[3] = {};
    const VrefLevel sel[3] = {VrefLevel::v1_024, VrefLevel::v2_048,
                              VrefLevel::v4_096};
    for (uint8_t i = 0; i < 3u; ++i) {
        if (i == 2u && !top_held) {
            print(serial, "  the 4.096 V level is not crossed: the supply, at ",
                  vdd_mv, " mV, does not hold it", crlf);
            bench.skip("a bandgap level the supply does not hold");
            continue;
        }
        (void)Vref::configure(VrefConfig{.level = sel[i], .output_enable = true});
        settle_long();
        levels[i] = crossing(AcNegative::bandgap, 0, 1023);
    }
    const uint32_t mv0 = static_cast<uint32_t>(levels[0]) * vdd_mv / 1024u;
    const uint32_t mv1 = static_cast<uint32_t>(levels[1]) * vdd_mv / 1024u;
    const uint32_t mv2 = static_cast<uint32_t>(levels[2]) * vdd_mv / 1024u;
    print(serial, "  the three bandgap levels cross at DAC codes ", levels[0],
          " / ", levels[1], " / ", levels[2], " = ", mv0, " / ", mv1, " / ",
          mv2, " mV against a supply this suite located at ", vdd_mv, " mV",
          crlf);
    // Twice one crossing against the next, in DAC codes so the supply
    // cancels but for the conversion of the tolerance. The levels' own
    // spread (table 45-36: 2 x 1045 - 2007 = 83 mV, 2 x 2089 - 4014 = 164
    // mV), the DAC's offset once (20 mV; its gain error doubles with the
    // code and cancels), the comparator's offset at two common modes
    // (2 x 20 + 12 = 52 mV) and the levels' supply coefficient away from
    // the table's 5.0 V (-0.2..+0.3 %/V over 2.3 V on both: 24 and 47 mV):
    // 179 and 283 mV, and six codes of the three crossings' INL and steps.
    const auto doubles = [](uint16_t lower, uint16_t upper, uint32_t tol_mv) {
        return lower < upper &&
               near(2u * lower, upper, mv_codes(tol_mv) + 6u);
    };
    bench.verdict("THE THREE LEVELS ARE THERE AND THEY DOUBLE - a comparator "
                  "and a DAC weigh the same bandgap the ADC and the AC's own "
                  "scaler weighed, and the 1:2:4 of 22.8.7 comes out of the "
                  "crossings",
                  located && doubles(levels[0], levels[1], 179u) &&
                      (!top_held || doubles(levels[1], levels[2], 283u)));

    // ---- erratum 1.5.6, staged with a control -------------------------------
    //
    // "A spurious COMP interrupt can occur upon AC enabling when INTREF
    // is selected as the negative input." The workaround is the VDD
    // scaler - so the CONTROL is the same enable at the same level
    // through the scaler, and the arrangement is one where the output
    // must NOT change: the DAC held well below the threshold, so STATE
    // is zero before and after.
    (void)Vref::configure(VrefConfig{.level = VrefLevel::v1_024,
                                     .output_enable = true});
    settle_long();
    dac_set(20);                   // far below both thresholds

    // "Upon AC enabling" is taken at both levels it can mean, since the
    // item does not say which: the comparator's own COMPCTRLn.ENABLE and
    // the block's CTRLA.ENABLE are both cycled on every trial.
    auto count_spurious = [](AcNegative neg, uint8_t scaler_step) -> uint8_t {
        uint8_t seen = 0;
        for (uint8_t i = 0; i < 64u; ++i) {
            (void)Comp2::enable(false);
            (void)Ac::enable(false);
            Ac::clear_flags(0xFFu);
            const AcConfig cfg{.positive = AcPositive::pin0,
                               .negative = neg,
                               .speed = AcSpeed::high};
            (void)Comp2::configure(cfg);
            if (neg == AcNegative::vscale) {
                Comp2::scaler(scaler_step);
            }
            (void)Ac::enable(true);
            (void)Comp2::enable(true);
            brio::spin_us<Ruler>(800);
            if (Comp2::flag_set() && !Comp2::state()) {
                ++seen;
            }
        }
        (void)Comp2::enable(false);
        return seen;
    };
    // The scaler step at the bandgap's level: 1.024 V is (step + 1)/64 of
    // the supply located above (12/64 of 5.15 V, 20/64 of 3.3 V).
    const uint8_t at_level =
        vdd_mv != 0u ? static_cast<uint8_t>((1024UL * 64UL + vdd_mv / 2u) / vdd_mv - 1u) : 11u;
    const uint8_t with_bandgap = count_spurious(AcNegative::bandgap, 0);
    const uint8_t with_scaler = count_spurious(AcNegative::vscale, at_level);
    print(serial, "  sixty-four enables with the output held low throughout: "
          "MUXNEG = bandgap raised ", with_bandgap,
          " flags with STATE still clear, MUXNEG = the scaler at the same "
          "level raised ", with_scaler, crlf);
    // Printed, not judged: whether the item reproduces is the die's fact,
    // and the driver's stated obligation (clear the flag after enabling,
    // before arming) stands either way.
    print(serial, with_bandgap > with_scaler
                      ? "  erratum 1.5.6 reproduces with its own workaround as "
                        "the control: enabling on the bandgap raises a COMP "
                        "flag, enabling on the scaler at the same level does not"
                      : "  erratum 1.5.6 did not reproduce in this arrangement on "
                        "this die",
          crlf);

    // ---- Ac::take_flags() from a real bound handler -------------------------
    //
    // ac.md: "both bench programs poll, and the AC vector is never
    // bound". The device header spells it AC_Handler.
    ac_irqs = 0;
    ac_last_mask = 0;
    const AcConfig watch{.positive = AcPositive::pin0,
                         .negative = AcNegative::bandgap,
                         .interrupt_on = AcInterrupt::toggle,
                         .speed = AcSpeed::high};
    bench.verdict("COMP2 watches the bandgap again, interrupting on every "
                  "toggle", Comp2::configure(watch) && Comp2::enable(true));
    dac_set(500);                  // start ABOVE, so all eight moves are edges
    brio::spin_us<Ruler>(800);
    Comp2::clear_flag();
    Comp2::arm(true);
    Nvic::enable(Ac::irq());
    for (uint8_t i = 0; i < 4u; ++i) {
        dac_set(20);
        dac_set(500);
    }
    Nvic::disable(Ac::irq());
    Comp2::arm(false);
    print(serial, "  eight crossings of the bandgap threshold produced ",
          ac_irqs, " vector entries, last mask ", Hex{ac_last_mask}, crlf);
    bench.verdict("Ac::take_flags() RUNS FROM A REAL HANDLER - the AC vector "
                  "is bound under the name the device header declares, and the "
                  "read-and-clear body returns the comparator's own bit",
                  ac_irqs >= 8u && (ac_last_mask & Comp2::flag) != 0u);

    (void)Comp2::enable(false);
    Ac::release();
    Dac::release();
}

// =============================================================================
// i - HYSTERESIS, MEASURED - AND THE 40.6.10 SWAP PROCEDURE
// =============================================================================
//
// Two measurements with one rig, because both are flip-code sweeps of
// the DAC on a comparator's NEGATIVE input against a fixed positive one:
//   - hysteresis is the GAP between the up-sweep and the down-sweep flip
//     codes, and one DAC LSB is VDD/1024 = about 5 mV, so table 45-34's
//     typical 100 mV is about twenty codes - comfortably resolvable;
//   - the offset procedure is the DIFFERENCE between the unswapped and
//     the swapped flip codes, whose half is the comparator's own input
//     offset and whose MIDPOINT is the crossing with the offset removed.
void ti_hysteresis() {
    if (!need_vdd()) {   // the millivolt bands below are the supply's
        return;
    }
    bench.verdict("hysteresis in single-shot mode is refused - 40.6.6 makes it "
                  "continuous-mode only",
                  !Comp2::config_valid(AcConfig{.single_shot = true,
                                                .hysteresis = true}) &&
                      Comp2::config_valid(AcConfig{.hysteresis = true}));

    bench.verdict("the DAC is up with its INTERNAL output, which is what "
                  "AcNegative::dac takes", dac_up(true, true));
    bench.verdict("the AC block is up", Ac::init(main_gen));

    constexpr uint8_t scaler_step = 31;      // VDD x 32/64, mid supply
    constexpr uint16_t centre = 512;
    constexpr uint16_t half_span = 96;

    /// The comparator's positive input is its own VDD scaler and its
    /// negative input is the DAC, so the output is one while the DAC is
    /// below the scaler. Sweeping the DAC UP finds the 1 -> 0 flip;
    /// sweeping it DOWN finds the 0 -> 1 flip.
    auto sweep = [](bool hysteresis, AcSpeed speed, bool swap,
                    bool upward) -> uint16_t {
        const AcConfig cfg{.positive = AcPositive::vscale,
                           .negative = AcNegative::dac,
                           .speed = speed,
                           .hysteresis = hysteresis,
                           .swap = swap};
        if (!Comp2::configure(cfg)) {
            return 0xFFFFu;
        }
        Comp2::scaler(scaler_step);
        if (!Comp2::enable(true)) {
            return 0xFFFFu;
        }
        // Start well outside the band so the comparator is settled in
        // the state the sweep has to leave.
        (void)Dac::set(upward ? centre - half_span : centre + half_span);
        settle_long();
        // SWAP swaps the terminals AND inverts the output (40.6.10), so
        // the two cancel and the sense of the output is UNCHANGED -
        // which is the whole reason the procedure works: only the
        // OFFSET changes sign. So the state the sweep looks for does
        // not depend on the swap.
        const bool target = !upward;
        uint16_t found = 0xFFFFu;
        for (uint16_t k = 0; k <= 2u * half_span; ++k) {
            const uint16_t c = upward ? static_cast<uint16_t>(centre - half_span + k)
                                      : static_cast<uint16_t>(centre + half_span - k);
            (void)Dac::set(c);
            settle_fast();
            if (Comp2::state() == target) {
                found = c;
                break;
            }
        }
        (void)Comp2::enable(false);
        return found;
    };

    // ---- the noise floor, before any band is chosen ------------------------
    uint16_t lo = 0xFFFFu;
    uint16_t hi = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        const uint16_t c = sweep(false, AcSpeed::high, false, true);
        if (c < lo) {
            lo = c;
        }
        if (c > hi) {
            hi = c;
        }
    }
    print(serial, "  four repeats of the same up-sweep with hysteresis OFF "
          "land between codes ", lo, " and ", hi, crlf);
    // A sweep that never flips returns 0xFFFF, outside every band below:
    // each verdict asks that its own sweeps flipped.
    const auto flipped = [](uint16_t code) { return code <= centre + half_span; };
    const uint16_t spread = static_cast<uint16_t>(hi - lo);
    // The premise: the spread under the smallest hysteresis table 45-34
    // allows (29 mV high speed), in the fewest codes that can be.
    bench.verdict("THE REPEATABILITY IS MEASURED FIRST, and it is a few DAC "
                  "codes - under the smallest hysteresis the table allows, so "
                  "the gap below is a signal and not a spread",
                  flipped(hi) && spread < mv_codes_min(29u));

    const uint16_t up_off = sweep(false, AcSpeed::high, false, true);
    const uint16_t down_off = sweep(false, AcSpeed::high, false, false);
    const uint16_t up_on = sweep(true, AcSpeed::high, false, true);
    const uint16_t down_on = sweep(true, AcSpeed::high, false, false);
    const uint16_t up_lp = sweep(true, AcSpeed::low_power, false, true);
    const uint16_t down_lp = sweep(true, AcSpeed::low_power, false, false);

    const int32_t gap_off =
        static_cast<int32_t>(up_off) - static_cast<int32_t>(down_off);
    const int32_t gap_on =
        static_cast<int32_t>(up_on) - static_cast<int32_t>(down_on);
    const int32_t gap_lp =
        static_cast<int32_t>(up_lp) - static_cast<int32_t>(down_lp);
    const uint32_t lsb_uv = static_cast<uint32_t>(vdd_mv) * 1000u / 1024u;
    print(serial, "  high speed, hysteresis OFF: up ", up_off, ", down ",
          down_off, ", gap ", gap_off, " codes = ",
          static_cast<int32_t>(gap_off) * static_cast<int32_t>(lsb_uv) / 1000,
          " mV", crlf);
    print(serial, "  high speed, hysteresis ON:  up ", up_on, ", down ",
          down_on, ", gap ", gap_on, " codes = ",
          static_cast<int32_t>(gap_on) * static_cast<int32_t>(lsb_uv) / 1000,
          " mV (table 45-34: 29..190, typ 100)", crlf);
    print(serial, "  LOW POWER, hysteresis ON:   up ", up_lp, ", down ",
          down_lp, ", gap ", gap_lp, " codes = ",
          static_cast<int32_t>(gap_lp) * static_cast<int32_t>(lsb_uv) / 1000,
          " mV (table 45-34: 25..248, typ 100)", crlf);

    // With no hysteresis the up-sweep stops on the first code past the
    // threshold and the down-sweep on the first code below it: a gap of
    // one, each sweep within the spread measured above and a code of its
    // own flip either way.
    const int32_t scatter = static_cast<int32_t>(spread);
    bench.verdict("WITH HYSTERESIS OFF THE TWO SWEEPS MEET - the up-flip and "
                  "the down-flip are the same code to within the repeatability "
                  "measured above",
                  flipped(up_off) && flipped(down_off) &&
                      near_signed(gap_off, 1, 2 * scatter + 1));
    // A hysteresis of H codes gives a gap of H to H + 2 (each sweep stops
    // on a whole code) give or take the DAC's DNL (0.3 LSB at each edge,
    // table 45-32) and the spread s: H lies within gap - 3 - 2s .. gap + 1
    // + 2s codes, and a code is the located supply's LSB within the
    // location's own 873..1125 per mille. The band judged is the
    // table's, met by some value in that interval.
    const auto in_band = [scatter, lsb_uv](int32_t gap, int32_t lo_mv, int32_t hi_mv) {
        const int32_t low = (gap - 3 - 2 * scatter) * static_cast<int32_t>(lsb_uv) *
                            static_cast<int32_t>(supply_lo_permille) / 1'000'000;
        const int32_t high = (gap + 1 + 2 * scatter) * static_cast<int32_t>(lsb_uv) *
                             static_cast<int32_t>(supply_hi_permille) / 1'000'000;
        return high >= lo_mv && low <= hi_mv;
    };
    bench.verdict("AND WITH IT ON THEY DO NOT: the gap between them IS the "
                  "hysteresis, and it lands inside table 45-34's own high-speed "
                  "band, within this instrument's step and the supply's location",
                  flipped(up_on) && flipped(down_on) && in_band(gap_on, 29, 190));

    // ERRATUM 1.5.1 says hysteresis is present ONLY for a falling
    // transition of the output, and its E/G/J row carries one X, under
    // B. The hysteresis-off crossing is the true threshold; if only one
    // edge moved, it would sit ON one of the two hysteresis edges.
    const int32_t mid_off = (static_cast<int32_t>(up_off) +
                             static_cast<int32_t>(down_off) + 1) / 2;
    const int32_t up_shift = static_cast<int32_t>(up_on) - mid_off;
    const int32_t down_shift = mid_off - static_cast<int32_t>(down_on);
    print(serial, "  against the hysteresis-free crossing at ", mid_off,
          " the two edges moved out by ", up_shift, " and ", down_shift,
          " codes", crlf);
    // An edge that did not move sits within the crossing's own scatter
    // of the hysteresis-free middle: the spread twice, a code of its flip
    // and the hysteresis-free gap's one code. Both edges must clear it.
    bench.verdict("ERRATUM 1.5.1 IS NOT THIS SILICON (E/G/J revision B alone) "
                  "AND THE BENCH AGREES WITH THE ROW: BOTH edges move away "
                  "from the hysteresis-free crossing, where the item describes "
                  "only one of them moving",
                  flipped(up_on) && flipped(down_on) &&
                      up_shift > 2 * scatter + 2 && down_shift > 2 * scatter + 2);

    // ERRATUM 1.5.2 - low-power mode WITH hysteresis - is marked B..E on
    // the E/G/J row and is NOT this revision, so the pairing is legal
    // here and this is what it does.
    bench.verdict("ERRATUM 1.5.2 MAKES THE LOW-POWER/HYSTERESIS PAIRING LEGAL "
                  "AT THIS REVISION (B..E on the E/G/J row) AND IT BEHAVES: "
                  "the slower comparator shows a hysteresis of the same order, "
                  "inside its own row of table 45-34",
                  flipped(up_lp) && flipped(down_lp) && in_band(gap_lp, 25, 248));
    print(serial, "  [erratum 1.5.4 - the hysteresis specification itself "
          "being wrong - is revisions B..E on this row, so table 45-34 as "
          "printed is the right band to judge against]", crlf);

    // ---- 40.6.10: the swap offset procedure --------------------------------
    //
    // "COMPCTRLx.SWAP controls switching of the input signals to a
    // comparator's positive and negative terminals. When the comparator
    // terminals are swapped, the output signal is also inverted. This
    // allows the user to measure or compensate for the comparator input
    // offset voltage."
    //
    // The offset sits on one terminal, so it moves the crossing one way
    // unswapped and the other way swapped: the two straddle the true
    // crossing and their MEAN is it. Hysteresis is off throughout -
    // otherwise the gap being measured would be the wrong one.
    static_assert(Comp2::config_valid(AcConfig{.swap = true}),
                  "SWAP is a configuration bit like any other, so it can only be "
                  "changed with the comparator disabled - which is what "
                  "configure() does");

    int32_t plain_sum = 0;
    int32_t swap_sum = 0;
    int32_t plain_lo = 4096;
    int32_t plain_hi = -1;
    int32_t swap_lo = 4096;
    int32_t swap_hi = -1;
    constexpr uint8_t reps = 4;
    for (uint8_t i = 0; i < reps; ++i) {
        const int32_t p = sweep(false, AcSpeed::high, false, true);
        const int32_t s = sweep(false, AcSpeed::high, true, true);
        plain_sum += p;
        swap_sum += s;
        if (p < plain_lo) {
            plain_lo = p;
        }
        if (p > plain_hi) {
            plain_hi = p;
        }
        if (s < swap_lo) {
            swap_lo = s;
        }
        if (s > swap_hi) {
            swap_hi = s;
        }
    }
    const int32_t plain_mean = (plain_sum + reps / 2) / reps;
    const int32_t swap_mean = (swap_sum + reps / 2) / reps;
    const int32_t midpoint = (plain_sum + swap_sum + reps) / (2 * reps);
    const int32_t offset_codes = (plain_mean - swap_mean) / 2;
    const int32_t offset_uv = offset_codes * static_cast<int32_t>(lsb_uv) / 2;
    // The scaler's own nominal voltage, in DAC codes: (step+1)/64 of the
    // supply is (step+1) x 16 of 1024.
    constexpr int32_t nominal = (scaler_step + 1) * 16;
    print(serial, "  unswapped crossing ", plain_mean, " (spread ",
          plain_hi - plain_lo, "), swapped ", swap_mean, " (spread ",
          swap_hi - swap_lo, "), midpoint ", midpoint, ", nominal ", nominal,
          crlf);
    print(serial, "  half their difference is ", offset_codes,
          " DAC codes, so the comparator's input offset is about ", offset_uv,
          " uV - with ONE DAC code (", lsb_uv,
          " uV) as this instrument's own floor under that number", crlf);
    // The two crossings straddle the true one by the comparator's offset
    // each way: table 45-34's high-speed -12..+20 mV, so apart by at most
    // twice 20 mV, and a code of each mean's step and the two spreads.
    const bool swaps_flipped = flipped(static_cast<uint16_t>(plain_hi)) &&
                               flipped(static_cast<uint16_t>(swap_hi));
    const int32_t swap_spread =
        (plain_hi - plain_lo) + (swap_hi - swap_lo);
    bench.verdict("40.6.10'S TWO-MEASUREMENT RECIPE RUNS AT ALL - the swapped "
                  "comparator keeps the SENSE of its output (the terminals and "
                  "the output are inverted together, which is what makes the "
                  "recipe an offset measurement and not a polarity change) and "
                  "still crosses within twice the comparator's tabled offset "
                  "of the unswapped crossing",
                  swaps_flipped &&
                      near_signed(plain_mean, swap_mean,
                                  static_cast<int32_t>(mv_codes(40u)) + 2 + swap_spread));
    // Half the difference is the offset, inside the same row of the table
    // with a code of rounding and half the spreads. Where the midpoint
    // lands against the scaler's nominal is printed above, not judged: the
    // scaler's offset and gain errors are typical figures only (table
    // 45-34).
    bench.verdict("...AND WHAT IT MEASURES IS THE COMPARATOR'S INPUT OFFSET: "
                  "half the difference between the two crossings lies inside "
                  "table 45-34's high-speed -12..+20 mV - and the figure "
                  "printed above sits against the 5 mV step of the only "
                  "source that can sweep it, an instrument coarser than the "
                  "table's typical -0.1/+1 mV",
                  swaps_flipped &&
                      (offset_codes < 0 ? -offset_codes : offset_codes) <=
                          static_cast<int32_t>(mv_codes(20u)) + 1 + swap_spread / 2);

    (void)Comp2::enable(false);
    Ac::release();
    Dac::release();
}

// =============================================================================
// j - THE INTSEL FLAVOURS AND INVEIx ON A REAL EVENT
// =============================================================================
//
// ac.md's remaining two lines: "the rising/falling INTSEL flavours -
// toggle and end-of-comparison are exercised" and "INVEIx: written and
// read back, never used to invert a real event".
//
// The event that inverts has to be a LEVEL, or both edges are in it and
// no inversion can be told apart. A comparator's own output IS a level
// (40.6.13: "a copy of the comparator status"), so COMP0 watching a pad
// driven by PORT is the stimulus, and COMP2 in SINGLE-SHOT mode is the
// user: whether its comparison starts on the rise or on the fall of that
// level is the whole question.
void tj_intsel_invei() {
    bench.verdict("the DAC is up and the AC block with it",
                  dac_up() && Ac::init(main_gen));

    // ---- INTSEL rising and falling -----------------------------------------
    constexpr uint8_t step = 31;
    auto arm_comp2 = [](AcInterrupt on) -> bool {
        const AcConfig cfg{.positive = AcPositive::vscale,
                           .negative = AcNegative::dac,
                           .interrupt_on = on,
                           .speed = AcSpeed::high};
        if (!Comp2::configure(cfg)) {
            return false;
        }
        Comp2::scaler(step);
        return Comp2::enable(true);
    };

    struct Edges {
        bool on_rise;
        bool on_fall;
    };
    auto edges_of = [](AcInterrupt on) -> Edges {
        // The output is one while the DAC is BELOW the scaler, so a low
        // code is a rising output and a high code a falling one.
        (void)Dac::set(900);
        settle_long();
        Comp2::clear_flag();
        (void)Dac::set(100);           // output 0 -> 1
        settle_long();
        const bool rise = Comp2::flag_set();
        Comp2::clear_flag();
        (void)Dac::set(900);           // output 1 -> 0
        settle_long();
        const bool fall = Comp2::flag_set();
        Comp2::clear_flag();
        (void)on;
        return Edges{rise, fall};
    };

    bench.verdict("COMP2 arms on RISING", arm_comp2(AcInterrupt::rising));
    const Edges rising = edges_of(AcInterrupt::rising);
    (void)Comp2::enable(false);
    bench.verdict("COMP2 arms on FALLING", arm_comp2(AcInterrupt::falling));
    const Edges falling = edges_of(AcInterrupt::falling);
    (void)Comp2::enable(false);
    bench.verdict("COMP2 arms on TOGGLE", arm_comp2(AcInterrupt::toggle));
    const Edges toggle = edges_of(AcInterrupt::toggle);
    (void)Comp2::enable(false);

    print(serial, "  flag after an output rise / fall: INTSEL rising ",
          yes_no(rising.on_rise), yes_no(rising.on_fall), ", falling ",
          yes_no(falling.on_rise), yes_no(falling.on_fall), ", toggle ",
          yes_no(toggle.on_rise), yes_no(toggle.on_fall), crlf);
    bench.verdict("THE TWO DIRECTED INTSEL FLAVOURS ARE DIRECTED - rising "
                  "fires on the rise and stays silent on the fall, falling "
                  "does the opposite, and toggle takes both",
                  rising.on_rise && !rising.on_fall && !falling.on_rise &&
                      falling.on_fall && toggle.on_rise && toggle.on_fall);

    // ---- INVEIx on a real event --------------------------------------------
    //
    // COMP0's output is the level; COMP2 is single-shot and started by
    // it through SOC2 on the asynchronous path table 29-3 restricts the
    // four start users to.
    PadA4::output();
    PadA4::clear();
    bench.verdict("the event fabric is up", event_clock_up());

    const AcConfig source{.positive = AcPositive::pin0,     // AIN0 = PA04
                          .negative = AcNegative::vscale,
                          .speed = AcSpeed::high};
    bench.verdict("COMP0 turns PA04's level into an event generator",
                  Comp0::configure(source));
    Comp0::scaler(31);
    bench.verdict("...and comes up", Comp0::enable(true));

    const AcConfig single{.positive = AcPositive::vscale,
                          .negative = AcNegative::dac,
                          .single_shot = true,
                          .interrupt_on = AcInterrupt::end_of_comparison,
                          .speed = AcSpeed::high};
    bench.verdict("COMP2 is SINGLE-SHOT, interrupting at the end of each "
                  "comparison - the flavour 40.8.12 restricts to this mode",
                  Comp2::configure(single));
    Comp2::scaler(31);

    /// One trial: route COMP0's output to COMP2's start user with or
    /// without inversion, then move PA04 one way and see whether a
    /// comparison happened.
    auto started_on = [](bool invert, bool rise) -> bool {
        (void)Comp2::enable(false);
        (void)Ac::enable(false);
        AcEventControl e{};
        e.comparator_out = 0x1u;                        // COMPEO0
        e.start_in = static_cast<uint8_t>(1u << 2);     // COMPEI2
        e.invert_in = invert ? static_cast<uint8_t>(1u << 2) : 0u;
        const bool cfg_ok = Ac::event_config(e);
        (void)Ac::enable(true);
        const bool routed =
            Evsys::connect(Ac::start_user(2), ev_a,
                           EventChannelConfig{
                               .generator = Ac::comparator_generator(0),
                               .path = EventPath::asynchronous});
        if (!cfg_ok || !routed) {
            return false;
        }
        // Put the level where the wanted transition starts from, then
        // arm and make the transition.
        if (rise) {
            PadA4::clear();
        } else {
            PadA4::set();
        }
        settle_long();
        (void)Comp2::enable(true);
        settle_long();
        Comp2::clear_flag();
        if (rise) {
            PadA4::set();
        } else {
            PadA4::clear();
        }
        settle_long();
        return Comp2::flag_set();
    };

    // RE-POINTING A CHANNEL AT A GENERATOR CAN LEAVE ONE EVENT
    // STANDING, so every arrangement is run twice and the SECOND is the
    // verdict.
    (void)started_on(false, true);
    const bool plain_rise = started_on(false, true);
    (void)started_on(false, false);
    const bool plain_fall = started_on(false, false);
    (void)started_on(true, true);
    const bool inv_rise = started_on(true, true);
    (void)started_on(true, false);
    const bool inv_fall = started_on(true, false);

    print(serial, "  a single-shot comparison started by COMP0's level: "
          "INVEI clear -> rise ", yes_no(plain_rise), ", fall ",
          yes_no(plain_fall), "; INVEI set -> rise ", yes_no(inv_rise),
          ", fall ", yes_no(inv_fall), crlf);
    bench.verdict("INVEIx INVERTS A REAL EVENT: with the bit clear the "
                  "comparison starts on the level's RISE and not on its fall, "
                  "and with the bit set the two swap - which is only "
                  "measurable at all because a comparator output is a level "
                  "and not a pulse",
                  plain_rise && !plain_fall && inv_fall && !inv_rise);

    Evsys::disconnect(Ac::start_user(2));
    (void)Comp0::enable(false);
    (void)Comp2::enable(false);
    Ac::release();
    Dac::release();
    hand_back<PadA4>();
    event_clock_down();
}

// =============================================================================
// k - SDADC LEFTOVERS: WINMONEO, THE FLUSH, AND THE INTERRUPTS
// =============================================================================
//
// sdadc.md's three testable gaps: "EVCTRL.WINMONEO is written and never
// routed anywhere"; "EVCTRL.FLUSHEI and SWTRIG.FLUSH: no bench letter
// has flushed a conversion in flight"; "the interrupts through the NVIC:
// none has ever driven a vector".
//
// ANACTRL.CTLSDADC and ANACTRL.BUFTEST stay DECLINED and this letter
// says why in print rather than poking them: 39.8.21 calls the first
// "used for Debug/Characterization" and gives no values at all, and
// gives the second no description whatsoever. A number obtained by
// writing them would mean nothing.
constexpr SdadcConfig sd_free{
    .reference = SdadcRef::vddana,
    .prescaler = 3,
    .osr = SdadcOsr::osr64,
    .skip_count = 2,
    .free_running = true,
};

/// The least a differential at the pair's own rails can read. With
/// VREF = VDDANA table 45-26 bounds the conversion range at 0.7 x VREF,
/// 22937 counts; an input past it reads at least that less the gain
/// error (3.4 %, table 45-27: 22157), the offset (6 mV, 73 counts at a
/// 2.7 V reference) and the INL (11 counts): 22073.
constexpr int32_t sd_past_range = 22073;

/// One decimation window a conversion: GCLK 48 MHz / (2 x (3 + 1)) = 6
/// MHz CLK_SDADC, 4 x OSR = 256 cycles, 42.7 us.
constexpr uint32_t sd_window_ns = 2u * (3u + 1u) * 4u * 64u * 1000u / 48u;

void tk_sdadc() {
    print(serial, "  [ANACTRL.CTLSDADC and ANACTRL.BUFTEST stay DECLINED: "
          "39.8.21 calls the first Debug/Characterization and lists no values, "
          "and describes the second not at all - a reading taken by poking "
          "them would carry no meaning]", crlf);

    // A known, large differential: the pair's two pads at opposite rails.
    PadSdN::output();
    PadSdN::clear();
    PadSdP::output();
    PadSdP::set();

    bench.verdict("the SDADC comes up free-running on pair 0",
                  Sdadc::pair_exists(0) &&
                      Sdadc::init(main_gen, sd_free, main_gen_hz) &&
                      Sdadc::select(0));
    brio::spin_us<Ruler>(8'000);
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun |
                       Sdadc::flag_winmon);
    // A free-running result every 43 us; 10 ms is hundreds of them.
    const bool steady_landed =
        brio::wait_until<Ruler>([] { return Sdadc::ready(); }, 10'000);
    const int32_t steady = Sdadc::result();
    print(serial, "  with AINN0 at GND and AINP0 at VDD the converter reads ",
          steady, crlf);
    bench.verdict("a full-scale differential is what the pads make",
                  steady_landed && steady >= sd_past_range);

    // ---- the flush: what it costs, timed, and what it discards -------------
    //
    // The VALUE cannot be the witness here: this pad pair makes a
    // full-scale differential, so a filter still filling and a filter
    // full both read the rail. THE TIME can be, and it is the honest
    // question anyway - 39.6.2.3's "the first valid sample starts from
    // the third" is a statement about WHOLE DECIMATION WINDOWS, so a
    // flush that throws the filter away must cost whole windows.
    bench.verdict("the crystal stopwatch is running", stopwatch_start());

    /// Wait for the next result and read it; returns the crystal tick at
    /// which it was seen, and clears `k_landed` when none came within
    /// 10 ms (a result is due every 43 us).
    bool k_landed = true;
    auto next_result_at = [&k_landed]() -> uint32_t {
        k_landed = brio::wait_until<Ruler>([] { return Sdadc::ready(); }, 10'000) &&
                   k_landed;
        const uint32_t t = ticks_now();
        (void)Sdadc::result();
        return t;
    };

    // The period, timed across THIRTY-TWO results in one stopwatch
    // window so no single reading's phase can bias it.
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    const uint32_t p0 = next_result_at();
    for (uint8_t i = 0; i < 31u; ++i) {
        (void)next_result_at();
    }
    const uint32_t period = (next_result_at() - p0) / 32u;
    // GCLK 48 MHz / (2 x (3+1)) = 6 MHz CLK_SDADC; one decimation window
    // is 4 x OSR = 256 cycles = 42.7 us = 1024 crystal ticks.
    const uint32_t window_ticks = 4u * 64u * 24u * 8u / 48u;

    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    bench.verdict("SWTRIG.FLUSH is accepted under a running converter",
                  Sdadc::flush());
    uint32_t marks[4] = {};
    const uint32_t t0 = ticks_now();
    for (uint8_t i = 0; i < 4u; ++i) {
        marks[i] = next_result_at();
    }
    print(serial, "  the free-running result period is ", period,
          " crystal ticks against the ", window_ticks,
          " one decimation window predicts (4 x OSR CLK_SDADC cycles at "
          "GCLK/8)", crlf);
    print(serial, "  the four results after a SWTRIG.FLUSH arrived ",
          marks[0] - t0, " ", marks[1] - marks[0], " ", marks[2] - marks[1],
          " ", marks[3] - marks[2], " ticks apart", crlf);
    // CLK_SDADC is OSC48M's, the stopwatch the crystal's: OSC48M's 5 %
    // (table 45-57), and the poll turn at each end of the 32 periods
    // (about 1.5 us, util/test_wait.hpp; 36 ticks over 32) and the
    // division's tick: 51 + 3 ticks.
    bench.verdict("A FREE-RUNNING RESULT ARRIVES EVERY DECIMATION WINDOW, "
                  "which is 4 x OSR CLK_SDADC cycles and nothing else - the "
                  "period everything below is judged against",
                  k_landed && near(period, window_ticks, window_ticks / 20u + 3u));
    // What one flush costs is printed, not judged: no part of brio depends
    // on what a flush discards, and 39.8.17's "flush the pipeline and
    // restart" names no time (the flush events below are counted, not
    // judged, for the same reason).
    const uint32_t first_gap = marks[0] - t0;
    print(serial, "  a single SWTRIG.FLUSH ",
          first_gap > period + period / 2u
              ? "costs whole decimation windows: the first result after it is "
                "late by more than a period"
              : "does not cost a whole window: the next result arrives sooner "
                "than one period after it",
          crlf);
    print(serial, "  [what a flush DISCARDS cannot be read off the VALUE on "
          "this bench: the pair's two pads make a differential at the "
          "converter's own rail, where a filter still filling and a filter "
          "full both report 0x7FFF - so the cost is measured in time and the "
          "discarded samples are declined]", crlf);

    // ---- EVCTRL.FLUSHEI, on a real event -----------------------------------
    //
    // Flush events every 20 us, inside the 43 us a decimation window takes.
    // If each flush threw the window in progress away, no result would
    // arrive; the single flush above lets a result through a fifth of a
    // window after it (docs/samc21/sdadc.md), so what a flush event leaves
    // standing is MEASURED here, not judged: the results counted over ten
    // milliseconds of flush events, printed beside the same converter's
    // count over ten milliseconds without them. The pacer runs two
    // decimation windows before the count opens and the flags are cleared
    // only then, so every result counted completed while flush events were
    // already arriving. The control is a verdict: with the pacer stopped
    // the converter delivers one result a period again - 240000 ticks over
    // the period measured above, within 2 (a window of fixed length holds
    // the floor or the ceiling of that quotient, and the period is an
    // integer quotient under a tick short, which moves it by under a
    // quarter).
    bench.verdict("the event fabric is up", event_clock_up());
    const TcConfig flusher{.mode = TcMode::count8,
                           .prescaler = TcPrescaler::div8,
                           .waveform = TcWaveform::normal_pwm};
    bench.verdict("TC2 paces a flush every 20 us, well inside the 43 us a "
                  "decimation window takes",
                  Pacer::init(main_gen) && Pacer::configure(flusher) &&
                      Pacer::set_period8(119) &&
                      Pacer::event_config(flusher,
                                          TcEventConfig{.overflow_out = true}));
    (void)Sdadc::enable(false);
    bench.verdict("a SYNCHRONOUS channel into the flush user is refused, and "
                  "an asynchronous one is taken",
                  !Sdadc::flush_on(ev_a,
                                   EventChannelConfig{
                                       .generator = Pacer::overflow_generator,
                                       .path = EventPath::synchronous,
                                       .edge = EventEdge::rising}) &&
                      Sdadc::flush_on(ev_a,
                                      EventChannelConfig{
                                          .generator = Pacer::overflow_generator,
                                          .path = EventPath::asynchronous}));
    (void)Sdadc::enable(true);
    brio::spin_us<Ruler>(8'000);
    constexpr uint32_t count_span = 240'000UL;     // ten milliseconds of crystal ticks
    /// Results read over `count_span` ticks; `first` the tick, from the
    /// window's opening, of the first one (0 when none came).
    auto count_results = [](uint32_t& first) -> uint16_t {
        Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
        uint16_t n = 0;
        first = 0;
        // Bounded on the ruler too, at twice the span, so a stopwatch that
        // stops ends the count instead of hanging it.
        const uint32_t r0 = Ruler::now();
        const uint32_t c0 = ticks_now();
        uint32_t now = c0;
        while (now - c0 < count_span &&
               Ruler::now() - r0 < brio::us_cycles<Ruler>(20'000)) {
            if (Sdadc::ready()) {
                (void)Sdadc::result();
                if (n == 0u) first = now - c0;
                ++n;
            }
            now = ticks_now();
        }
        return n;
    };
    (void)Pacer::set_count8(0);
    (void)Pacer::enable(true);
    brio::spin_us<Ruler>(2u * window_ticks / 24u + 1u);   // two windows, crystal ticks at 24 MHz
    uint32_t first_flushing = 0;
    const uint16_t while_flushing = count_results(first_flushing);
    (void)Pacer::enable(false);
    brio::spin_us<Ruler>(8'000);
    uint32_t first_free = 0;
    const uint16_t free_count = count_results(first_free);
    const uint32_t due = period != 0u ? count_span / period : 0u;
    print(serial, "  results in ten milliseconds with a flush event every 20 us: ",
          while_flushing, " (the first ", first_flushing, " ticks in); with the pacer "
          "stopped: ", free_count, " (", due, " due at the period above)", crlf);
    bench.verdict("the control: with the flush events stopped the same converter "
                  "delivers one result a period again (+-2 in ten milliseconds)",
                  near(free_count, due, 2u));
    (void)Sdadc::enable(false);
    (void)Sdadc::stop_events();
    (void)Sdadc::enable(true);
    Pacer::release();

    // ---- the window monitor's OUTPUT event ---------------------------------
    //
    // The input is fixed, so the THRESHOLD is what moves. A TC counting
    // events is the witness this stratum uses everywhere.
    bench.verdict("the event fabric is up", event_clock_up());
    bench.verdict("TC3 counts events rather than clock ticks",
                  Counter::init(main_gen) &&
                      Counter::configure(TcConfig{.mode = TcMode::count16}) &&
                      Counter::event_config(
                          TcConfig{.mode = TcMode::count16},
                          TcEventConfig{.action = TcEventAction::count,
                                        .input_enable = true}));

    SdadcConfig win = sd_free;
    win.events.window_out = true;
    win.window = SdadcWindow::above_lower;
    win.window_low = static_cast<int16_t>(steady / 2);
    Sdadc::release();
    bench.verdict("the SDADC comes back with WINMONEO set and a threshold the "
                  "reading is above",
                  Sdadc::init(main_gen, win, main_gen_hz) && Sdadc::select(0));
    bench.verdict("the window generator reaches TC3",
                  Evsys::connect(Counter::event_user, ev_b,
                                 EventChannelConfig{
                                     .generator = Sdadc::winmon_generator,
                                     .path = EventPath::asynchronous}));
    (void)Counter::enable(true);
    (void)Counter::set_count16(0);
    brio::spin_us<Ruler>(20'000);
    const uint16_t hits = Counter::count16();

    // THE CONTROL HAS TO BE A CONDITION THIS READING CANNOT MEET, and no
    // upper threshold is one: the pads put the result at the converter's
    // positive rail, so nothing is above it. "RESULT < a large negative
    // number" is, and it is the same window monitor with the same event
    // enable - only the condition changes.
    (void)Sdadc::window(SdadcWindow::below_upper, static_cast<int16_t>(-32768),
                        static_cast<int16_t>(-32000));
    brio::spin_us<Ruler>(8'000);
    (void)Counter::set_count16(0);
    brio::spin_us<Ruler>(20'000);
    const uint16_t misses = Counter::count16();
    print(serial, "  WINMONEO: 20 ms with the condition matching gave ", hits,
          " events, and the same 20 ms with a condition this reading cannot "
          "meet gave ", misses, crlf);
    // One event a conversion: 20 ms of the ruler and the converter, both
    // on OSC48M, is 20000000 / 42667 ns = 468.75 windows - 468 or 469,
    // and one more at either edge. The control's condition cannot match:
    // no event at all.
    constexpr uint32_t windows_20ms = 20'000'000u / sd_window_ns;
    bench.verdict("EVCTRL.WINMONEO MOVES SOMETHING REAL, with a control that "
                  "moves nothing: the window monitor's match is an event "
                  "generator and a counter downstream sees one event per "
                  "matching conversion",
                  hits + 1u >= windows_20ms && hits <= windows_20ms + 2u &&
                      misses == 0u);

    // ---- the three interrupts through the NVIC -----------------------------
    (void)Sdadc::window_off();
    brio::spin_us<Ruler>(8'000);
    sdadc_irqs = 0;
    sdadc_last_mask = 0;
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun |
                       Sdadc::flag_winmon);
    Sdadc::arm(Sdadc::flag_resrdy);
    Nvic::enable(Sdadc::irq());
    brio::spin_us<Ruler>(10'000);
    Nvic::disable(Sdadc::irq());
    Sdadc::disarm(Sdadc::flag_resrdy | Sdadc::flag_overrun |
                  Sdadc::flag_winmon);
    print(serial, "  10 ms of free-running conversions with RESRDY armed: ",
          sdadc_irqs, " vector entries, last mask ", Hex{sdadc_last_mask},
          ", last result ", sdadc_last_result, crlf);
    // One entry a conversion: 10 ms is 234.4 windows, 234 or 235, one
    // more for a result that landed between the flags' clearing and the
    // vector's enabling, one fewer at the far edge.
    constexpr uint32_t windows_10ms = 10'000'000u / sd_window_ns;
    bench.verdict("THE SDADC VECTOR IS BOUND UNDER THE DEVICE HEADER'S OWN "
                  "NAME and its RESRDY drives the NVIC once a conversion - the "
                  "handler reads RESULT, which is what clears the flag",
                  sdadc_irqs + 1u >= windows_10ms && sdadc_irqs <= windows_10ms + 2u &&
                      (sdadc_last_mask & Sdadc::flag_resrdy) != 0u &&
                      sdadc_last_result >= sd_past_range);

    // OVERRUN through the same vector: arm it and stop reading.
    sdadc_irqs = 0;
    sdadc_last_mask = 0;
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    Sdadc::arm(Sdadc::flag_overrun);
    Nvic::enable(Sdadc::irq());
    brio::spin_us<Ruler>(5'000);
    Nvic::disable(Sdadc::irq());
    const uint32_t over_irqs = sdadc_irqs;
    const uint8_t over_mask = sdadc_last_mask;
    print(serial, "  with RESRDY disarmed and nothing reading RESULT: ",
          over_irqs, " entries, last mask ", Hex{over_mask}, crlf);
    bench.verdict("OVERRUN DRIVES THE SAME VECTOR - one interrupt line for "
                  "three sources, dispatched on the mask the ISR body returns",
                  over_irqs > 0u &&
                      (over_mask & Sdadc::flag_overrun) != 0u);

    Sdadc::disarm(Sdadc::flag_resrdy | Sdadc::flag_overrun |
                  Sdadc::flag_winmon);
    Evsys::disconnect(Counter::event_user);
    Sdadc::release();
    Counter::release();
    Pacer::release();
    event_clock_down();
    hand_back<PadSdN>();
    hand_back<PadSdP>();
}

// =============================================================================
// l - TSENS EVCTRL.STARTINV
// =============================================================================
//
// tsens.md: "EVCTRL.STARTINV, the inverted start event: implemented, and
// no chain here needed a falling edge."
//
// The same stimulus letter j needed and for the same reason: an
// inversion can only be told apart on a LEVEL, and a comparator output
// is one. COMP0 watches PA04 under PORT; the level's rise or fall starts
// a temperature measurement, and which of the two does is the question.
//
// THE WINDOW HYSTERESIS MODES ARE DECLINED, in print, with the reason:
// they need the die to cross a threshold and come BACK, and the only
// thing that moves this die's temperature here is its own self-heating -
// a slow one-way drift of a few tenths of a degree. A threshold placed
// in that drift is crossed once and never returns, so the hysteresis
// itself would never be exercised and a verdict would be about the
// crossing and not about the mode.
void tl_tsens_startinv() {
    print(serial, "  [the window-mode HYSTERESIS flavours stay DECLINED: they "
          "need the die to cross a threshold and come back, and the only "
          "stimulus here is this chip's own self-heating - a one-way drift, so "
          "the hysteresis would never be the thing measured]", crlf);
    print(serial, "  [and a MeterSource / analog_sampler adapter for this "
          "block stays a design question and not a gap: the util concept wants "
          "an unsigned reading and a channel to select, and this block has "
          "neither]", crlf);

    PadA4::output();
    PadA4::clear();
    bench.verdict("the DAC is up and the AC block with it",
                  dac_up() && Ac::init(main_gen));
    bench.verdict("the event fabric is up", event_clock_up());

    // EVCTRL is enable-protected at the BLOCK level, so COMPEO0 goes in
    // with the block down. Without it the comparator's status never
    // leaves the peripheral and every arrangement below would measure
    // the same nothing.
    (void)Ac::enable(false);
    bench.verdict("the AC publishes COMP0's status as an event",
                  Ac::event_config(AcEventControl{.comparator_out = 0x1u}) &&
                      Ac::enable(true));

    const AcConfig source{.positive = AcPositive::pin0,     // AIN0 = PA04
                          .negative = AcNegative::vscale,
                          .speed = AcSpeed::high};
    bench.verdict("COMP0 turns PA04's level into an event generator",
                  Comp0::configure(source) && (Comp0::scaler(31), true) &&
                      Comp0::enable(true));

    TsensConfig cfg{};
    cfg.calibration = TsensCalibration::factory();
    cfg.events.start_in = true;
    bench.verdict("the factory calibration is there and a zero GAIN is refused "
                  "because it is 2^24 and not none",
                  cfg.calibration.gain != 0u &&
                      !tsens_config_valid(TsensConfig{}) &&
                      tsens_config_valid(cfg));

    /// Arm TSENS on the level with or without STARTINV, then move PA04
    /// one way and see whether a measurement happened.
    auto measured_on = [](const TsensConfig& base, bool invert,
                          bool rise) -> bool {
        Tsens::release();
        if (!Tsens::init(main_gen, base)) {
            return false;
        }
        if (!Tsens::enable(false)) {
            return false;
        }
        if (!Tsens::start_on(ev_a,
                             EventChannelConfig{
                                 .generator = Ac::comparator_generator(0),
                                 .path = EventPath::asynchronous},
                             invert)) {
            return false;
        }
        if (!Tsens::enable(true)) {
            return false;
        }
        if (rise) {
            PadA4::clear();
        } else {
            PadA4::set();
        }
        settle_long();
        Tsens::clear_flags();
        if (rise) {
            PadA4::set();
        } else {
            PadA4::clear();
        }
        // A measurement is 2 x GAIN + about 2020 GCLK_TSENS periods -
        // some milliseconds at 48 MHz - so this waits generously.
        return brio::wait_until<Ruler>([] { return Tsens::result_ready(); }, 20'000);
    };

    // TSENS's own EVCTRL is enable-protected too, which start_on()
    // enforces by refusing while the block is up - so the routing has to
    // be done with it down, which is what measured_on() does.
    (void)Tsens::release();
    bench.verdict("TSENS refuses to have its event control written while it is "
                  "enabled - 43.6.2.1's enable protection, as a refusal rather "
                  "than a dropped store",
                  Tsens::init(main_gen, cfg) &&
                      !Tsens::event_config(TsensEventControl{.start_in = true}) &&
                      !Tsens::start_on(ev_a,
                                       EventChannelConfig{
                                           .generator = Ac::comparator_generator(0),
                                           .path = EventPath::asynchronous}));

    (void)measured_on(cfg, false, true);
    const bool plain_rise = measured_on(cfg, false, true);
    const bool plain_fall = measured_on(cfg, false, false);
    const bool inv_rise = measured_on(cfg, true, true);
    const bool inv_fall = measured_on(cfg, true, false);
    print(serial, "  a measurement started by COMP0's level: STARTINV clear -> "
          "rise ", yes_no(plain_rise), ", fall ", yes_no(plain_fall),
          "; STARTINV set -> rise ", yes_no(inv_rise), ", fall ",
          yes_no(inv_fall), crlf);
    bench.verdict("EVCTRL.STARTINV INVERTS A REAL START EVENT: with the bit "
                  "clear a measurement begins on the level's RISE and not on "
                  "its fall, and with it set the two swap",
                  plain_rise && !plain_fall && inv_fall && !inv_rise);

    // And what the INVERTED path started is still a temperature, so it
    // is not merely triggering something empty. The arrangement is the
    // inverted one, so the edge that has to be made is a FALL.
    const bool armed_inverted = measured_on(cfg, true, false);
    const auto reading = Tsens::read();
    if (reading) {
        print(serial, "  the measurement that edge started reads ", *reading,
              " hundredths of a degree Celsius", crlf);
    }
    bench.verdict("...and what the inverted edge started is a real "
                  "measurement: a plausible die temperature and not an empty "
                  "trigger",
                  armed_inverted && reading.has_value() && *reading > -2000 &&
                      *reading < 12000);

    Tsens::release();
    (void)Comp0::enable(false);
    Ac::release();
    Dac::release();
    hand_back<PadA4>();
    event_clock_down();
}

void banner() {
    print(serial, crlf,
          "test_samc_analog - SAMC21J18A: the analog completion - the ADC "
          "pair and its sequence, differential mode, DAC dithering, the AC "
          "finished (COMP2/3, window 1, the bandgap, hysteresis, the swap "
          "procedure), SDADC and TSENS leftovers, wireless, clk=",
          SysClock::hz, " Hz, VDD located at ", vdd_mv, " mV", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// The DAC's one vector for both sources. The ISR body clears UNDERRUN
/// (nothing else carries that information) and leaves EMPTY to whoever
/// feeds the buffer - so this handler is what feeds it.
extern "C" void DAC_Handler() {
    dac_irq_entries = dac_irq_entries + 1u;
    const uint8_t pending = brio::Dac::isr();
    if ((pending & brio::Dac::flag_underrun) != 0u) {
        dac_underrun_irqs = dac_underrun_irqs + 1u;
    }
    if ((pending & brio::Dac::flag_empty) != 0u) {
        dac_empty_irqs = dac_empty_irqs + 1u;
        if (dac_feed_from_isr) {
            brio::Dac::buffer(dac_feed_value);
        } else {
            brio::Dac::clear_flags(brio::Dac::flag_empty);
        }
    }
}

/// The AC's one vector for four comparators and two windows.
extern "C" void AC_Handler() {
    const uint8_t mask = brio::Ac::take_flags();
    if (mask != 0u) {
        ac_irqs = ac_irqs + 1u;
        ac_last_mask = mask;
    }
}

/// The SDADC's one vector for its three sources. RESRDY is cleared by
/// READING the result, which is what the body's resrdy() half does. An
/// entry with nothing pending does nothing: letter k disables the vector
/// at the end of its RESRDY window, and a RESRDY that rose just before
/// leaves the NVIC's pending bit set, so the vector is entered once at the
/// next enable with RESRDY disarmed - an entry that, if it disarmed
/// OVERRUN, would hide the overrun the next leg waits for.
extern "C" void SDADC_Handler() {
    const uint8_t pending = brio::Sdadc::isr();
    if (pending != 0u) {
        sdadc_irqs = sdadc_irqs + 1u;
        sdadc_last_mask = pending;
    }
    if ((pending & brio::Sdadc::flag_resrdy) != 0u) {
        sdadc_last_result = brio::Sdadc::result();
    } else if ((pending & brio::Sdadc::flag_overrun) != 0u) {
        brio::Sdadc::disarm(brio::Sdadc::flag_overrun);
        brio::Sdadc::clear_flags(brio::Sdadc::flag_overrun);
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    brio::enable_interrupts();

    (void)locate_supply();   // the banner prints it; a letter that needs it asks again

    bench.letter('a', "the ADC host/client pair: both dual modes and the three "
                      "restart options", ta_pair);
    bench.letter('b', "the ADC's automatic sequence over six known inputs",
                 tb_sequence);
    bench.letter('c', "differential mode against the DAC, and the signed "
                      "window", tc_differential);
    bench.letter('d', "rail-to-rail and offset compensation at three common "
                      "modes", td_rail_to_rail);
    bench.letter('e', "DAC dithering: sub-LSB means, and LEFTADJ on silicon",
                 te_dither);
    bench.letter('f', "the DAC's EMPTY and UNDERRUN through the NVIC",
                 tf_dac_interrupts);
    bench.letter('g', "the AC completed: COMP2, COMP3 and window 1",
                 tg_comp23_window1);
    bench.letter('h', "the bandgap as a negative input, VREFOE, erratum 1.5.6",
                 th_bandgap);
    bench.letter('i', "hysteresis measured, and the 40.6.10 swap procedure",
                 ti_hysteresis);
    bench.letter('j', "the INTSEL flavours and INVEIx on a real event",
                 tj_intsel_invei);
    bench.letter('k', "SDADC: the window event, the flush, the interrupts",
                 tk_sdadc);
    bench.letter('l', "TSENS: EVCTRL.STARTINV against a real event edge",
                 tl_tsens_startinv);

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
