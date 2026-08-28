// test_samc_tsens - the reference bench suite for samc/tsens.hpp, the
// SAM C21's on-die temperature sensor (DS60001479M ch. 43).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. 43.5.1 is "Not applicable" - the sensor has no pads
// at all - and this bench has NO INDEPENDENT THERMOMETER, which is the
// fact every verdict here is designed around. So:
//
//   - NO ABSOLUTE ACCURACY IS CLAIMED ANYWHERE. Table 45-37 allows
//     -11.3 .. +6.2 C over [0,60] C, which is wider than any band this
//     room could justify; the reading at rest is judged only against a
//     PLAUSIBILITY band (an office is neither freezing nor boiling) and
//     that band is stated as such.
//   - EVERY BAND IS PRECEDED BY ITS OWN NOISE MEASUREMENT. The spread of
//     N readings is printed before anything is judged against a
//     tolerance, and a difference smaller than that spread is PRINTED
//     AND DECLINED rather than turned into a verdict.
//   - WHAT CAN BE MEASURED WITHOUT A THERMOMETER IS RATIOS AND
//     DIFFERENCES, and this chapter offers an unusually good one.
//
// THE LEVER THIS CHAPTER GIVES AND NO OTHER DID. The TSENS is not an ADC
// channel: it counts a temperature-dependent oscillator against
// GCLK_TSENS, so the GENERIC CLOCK IS THE RULER and the factory GAIN
// belongs to one particular rate - "the undivided internal 48MHz
// oscillator" (43.6.1). On this die OSC48M is 5100 ppm SLOW against the
// board's 24 MHz crystal (the clock campaign measured it; letter d
// measures it again, here, with FREQM). So the SAME DIE at the SAME
// temperature read with GCLK_TSENS on OSC48M and on a crystal-locked
// DPLL at a true 48 MHz must differ by exactly the reference's own error
// mapped through 43.6.1's formula - a self-contained demonstration that
// needs no thermometer, only two clocks and one arithmetic prediction.
//
// What is exercised, letter by letter:
//   a  the block, the registers, the disciplines and the refusals
//   b  the factory calibration, and what each part of it is worth
//   c  the reading at rest: jitter first, then a plausibility band
//   d  THE SCALE LEVER: one die, four clocks, one prediction
//   e  what a measurement costs in time, ruled by the crystal
//   f  how wide the datapath really is: OVF found by sweeping GAIN
//   g  the window monitor, all six modes and the two that disagree
//   h  the no-CPU chain: event in on every path, DMAC out, WINMON counted
//   i  the four interrupt sources through the one vector
//   j  drift under load - printed, and judged only if it clears the noise
//   p  erratum 1.19.1's PAC write protection (BY NAME ONLY: it may reset
//      the board, and a reset mid-z would destroy the run)
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/panic.hpp"
#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/evsys.hpp"
#include "samc/freqm.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"
#include "samc/sercom.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "samc/tsens.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// ---------------------------------------------------------------------------
// Letter p's breadcrumb: the PAC probe may fault, and a fault on this
// board is a reset. The token says the probe was in flight so the next
// boot can report the answer instead of losing it.
//
// AT GLOBAL SCOPE AND `inline`, both deliberately: gcc gives an inline
// variable with a section attribute a COMDAT group where a plain one gets
// none, and the platform's own panic_record_ is a static inline member -
// so a plain object here collides with it as a section type conflict.
// (test_samc_platform's token carries the same note for the same reason.)
//
// Its magic word is not decoration: table 18-1 has no SRAM row at all,
// for any reset source, so nothing promises this object survives and
// every read of it is guarded.
// ---------------------------------------------------------------------------
struct PacToken {
    uint32_t magic;
    uint32_t armed;
};
inline constexpr uint32_t pac_magic = 0x50414331UL;   // "PAC1"
[[gnu::section(".noinit")]] inline PacToken pac_token;

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
// The clocks. GCLK_TSENS's generator is not a detail here - it is the
// measurement's ruler - so every generator this suite builds is named for
// what it rules.
// ---------------------------------------------------------------------------
constexpr uint32_t crystal_hz = 24'000'000UL;
constexpr uint32_t nominal_hz = tsens_calibration_gclk_hz;   // 48 MHz

constexpr uint8_t gen_sys = 0;      ///< OSC48M, the CPU's own generator
constexpr uint8_t gen_xtal = 2;     ///< the crystal, undivided: 24 MHz
constexpr uint8_t gen_dpll = 4;     ///< the DPLL locked to the crystal: 48 MHz
constexpr uint8_t gen_ref = 5;      ///< the crystal / 250 = 96 kHz, FREQM's reference
constexpr uint8_t gen_slow = 6;     ///< OSCULP32K: the DPLL lock timer, the EVSYS channels

using GenXtal = Gclk<gen_xtal>;
using GenDpll = Gclk<gen_dpll>;
using GenRef = Gclk<gen_ref>;
using GenSlow = Gclk<gen_slow>;

/// 24 MHz / 250 = 96000 Hz exactly, through 16.6.2.7's LINEAR divider.
constexpr uint32_t ref_div = 250;
constexpr uint32_t ref_hz = crystal_hz / ref_div;   // 96000
constexpr uint8_t refnum_full = 255;

// ---------------------------------------------------------------------------
// The instruments
// ---------------------------------------------------------------------------
/// TC0 + TC1 as one 32-bit counter ON THE CRYSTAL. A conversion time
/// reported against OSC48M would carry that oscillator's 5100 ppm into a
/// number about the TSENS.
using Stopwatch = Tc<0>;
using Pacer = Tc<2>;
using Counter = Tc<3>;

constexpr uint8_t dma_ch = 0;
using Copy = DmaChannel<dma_ch>;
constexpr uint16_t dma_results = 16;
/// VOLATILE IN BOTH DIRECTIONS - the DMAC campaign's lesson on this
/// target: the compiler sees neither the controller's reads nor its
/// writes.
volatile uint32_t results[dma_results];

constexpr uint8_t ev_start_channel = 0;    ///< pacer overflow -> TSENS START
constexpr uint8_t ev_window_channel = 1;   ///< TSENS WINMON -> TC3 counts

// ---------------------------------------------------------------------------
// The interrupt letter's evidence
// ---------------------------------------------------------------------------
volatile uint32_t tsens_irqs = 0;
volatile uint8_t tsens_last_mask = 0;
volatile int32_t tsens_last_value = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// NB the printed lines below never contain "->": tools/bench.py's judge
// looks for that arrow to find a letter's tally line, and a stray one in
// a report line ends the capture early. Learned on the bench, twice.

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

bool near(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}
bool near_signed(int32_t a, int32_t b, int32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}
int32_t abs_signed(int32_t v) { return v < 0 ? -v : v; }

/// Deviation in parts per million, unsigned.
uint32_t ppm_off(uint32_t got, uint32_t want) {
    const uint64_t d = got > want ? got - want : want - got;
    return static_cast<uint32_t>((d * 1'000'000ULL) / want);
}

/// Print a centi-degree value as degrees with two decimals, without a
/// float anywhere: "25.37" / "-3.04".
void print_celsius(int32_t centi) {
    const bool neg = centi < 0;
    const uint32_t a = static_cast<uint32_t>(neg ? -centi : centi);
    if (neg) {
        print(serial, "-");
    }
    print(serial, a / 100u, ".");
    const uint32_t f = a % 100u;
    if (f < 10u) {
        print(serial, "0");
    }
    print(serial, f);
}

const char* yes_no(bool v) { return v ? "yes" : "no"; }

/// The factory calibration, read once and reused - reading the NVM
/// calibration area is a flash access and the answer never changes.
TsensCalibration factory{};

/// The baseline configuration: the factory calibration, single
/// measurements, no window, no events.
TsensConfig base_cfg() {
    TsensConfig c{};
    c.calibration = factory;
    return c;
}

/// Bring the sensor up on `generator` with `cfg`.
bool tsens_up(uint8_t generator, const TsensConfig& cfg) {
    Tsens::release();
    return Tsens::init(generator, cfg);
}

/// A batch of measurements, reported as min / max / mean. The MEAN is
/// rounded half-away-from-zero like Tsens::measure_average(), and the
/// spread is what every band in this suite is compared against BEFORE it
/// is chosen.
struct Batch {
    bool ok = false;
    uint16_t n = 0;
    int32_t min = 0;
    int32_t max = 0;
    int32_t mean = 0;
    int32_t spread() const { return max - min; }
};

Batch take(uint16_t n) {
    Batch b{};
    int64_t sum = 0;
    for (uint16_t i = 0; i < n; ++i) {
        const auto v = Tsens::measure();
        if (!v) {
            return b;
        }
        if (i == 0u || *v < b.min) {
            b.min = *v;
        }
        if (i == 0u || *v > b.max) {
            b.max = *v;
        }
        sum += *v;
    }
    const int64_t half = n / 2;
    b.mean = static_cast<int32_t>(sum >= 0 ? (sum + half) / n : (sum - half) / n);
    b.n = n;
    b.ok = true;
    return b;
}

void print_batch(const char* label, const Batch& b) {
    print(serial, "  ", label, ": mean ", b.mean, " centi-C (");
    print_celsius(b.mean);
    print(serial, " C) over ", b.n, " readings, min ", b.min, " max ", b.max,
          " spread ", b.spread(), crlf);
}

// ---------------------------------------------------------------------------
// The crystal, the reference and the frequency meter
// ---------------------------------------------------------------------------

/// Start the crystal if it is not running, and build the two generators
/// that hang off it. Every letter that measures anything calls this, so
/// each letter stands alone and `z` re-runs in one power-on.
bool crystal_up() {
    if (!Xosc::enabled() || !Xosc::ready()) {
        if (!Xosc::init(XoscConfig{.hz = crystal_hz, .startup = 4})) {
            return false;
        }
    }
    return GenXtal::configure(GclkConfig{.source = GclkSource::xosc}) &&
           GenRef::configure(GclkConfig{.source = GclkSource::xosc, .div = ref_div});
}

/// Measure `generator` against the crystal-derived 96 kHz reference and
/// return the frequency in hertz. Nothing on a failed measurement.
std::optional<uint32_t> measure_hz(uint8_t generator, uint8_t refnum = refnum_full) {
    const FreqmConfig cfg{
        .measured_generator = generator,
        .reference_generator = gen_ref,
        .refnum = refnum,
    };
    if (!Freqm::init(cfg)) {
        return std::nullopt;
    }
    const auto count = Freqm::measure();
    Freqm::release();
    if (!count || *count == 0u) {
        return std::nullopt;
    }
    return Freqm::to_hz(*count, ref_hz, refnum);
}

/// The DPLL at a true 48 MHz, locked to the crystal: 24 MHz / (2 x 6) =
/// 2 MHz reference, multiplied by 24.
constexpr FdpllConfig dpll48{
    .reference = DpllReference::xosc,
    .reference_hz = crystal_hz,
    .xosc_div = 5,
    .ldr = 23,
};
static_assert(Fdpll::divided_reference_hz(dpll48) == 2'000'000);
static_assert(Fdpll::dco_hz(dpll48) == 48'000'000);

bool dpll_up() {
    if (Fdpll::enabled() && Fdpll::locked()) {
        return GenDpll::configure(GclkConfig{.source = GclkSource::dpll96m});
    }
    if (!GenSlow::configure(GclkConfig{.source = GclkSource::osculp32k}) ||
        !Fdpll::lock_timer_clock(gen_slow)) {
        return false;
    }
    if (!Fdpll::init(dpll48) || !Fdpll::wait_locked()) {
        return false;
    }
    return GenDpll::configure(GclkConfig{.source = GclkSource::dpll96m});
}

/**
 * Hand every borrowed clock back, IN THE ONLY ORDER THAT WORKS.
 *
 * 16.6.2.6 releases a generator's old source only once the NEW one is
 * ready, so a generator still pointed at a stopped oscillator can never
 * be moved again. Everything built on the crystal or the DPLL is moved
 * to OSC48M - always running - BEFORE anything is stopped.
 */
void clocks_down() {
    Tsens::release();
    (void)GenDpll::configure(GclkConfig{.source = GclkSource::osc48m});
    (void)GenXtal::configure(GclkConfig{.source = GclkSource::osc48m});
    (void)GenRef::configure(GclkConfig{.source = GclkSource::osc48m});
    (void)Fdpll::stop();
    Xosc::stop();
}

bool stopwatch_up() {
    return Stopwatch::init(gen_xtal) &&
           Stopwatch::configure(TcConfig{.mode = TcMode::count32,
                                         .prescaler = TcPrescaler::div1}) &&
           Stopwatch::enable(true);
}
uint32_t ticks_now() { return Stopwatch::count32(); }

/// Crystal ticks turned into microseconds. One tick is 1/24 us.
uint32_t ticks_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) /
                                 crystal_hz);
}

// =============================================================================
// a - the block, the registers, the disciplines and the refusals
// =============================================================================
//
// Nothing is measured here that a temperature could change: this letter
// is about what the silicon's registers are, which of them the enable
// protects, and which of the chapter's rules the driver refuses in
// advance.
void ta_block() {
    bench.verdict("the TSENS exists on this device and there is exactly one",
                  tsens_count() == 1u);
    print(serial, "  GCLK_TSENS is peripheral channel ", Tsens::gclk_id,
          ", the vector is IRQ ", static_cast<uint32_t>(Tsens::irq()),
          ", the PAC id is ", Tsens::pac_id, crlf);
    bench.verdict("its generic clock is channel 5, its DMA trigger 1, its "
                  "window generator 30 and its START user 0",
                  Tsens::gclk_id == 5u && Tsens::dma_trigger_resrdy == 1u &&
                      Tsens::window_generator == 30u &&
                      Tsens::start_event_user == 0u);

    bench.verdict("the block comes up on the main generator with the factory "
                  "calibration",
                  tsens_up(gen_sys, base_cfg()));

    // ---- the register masks, probed rather than believed ----------------
    //
    // Every one of these is a write of all ones followed by a readback,
    // which is the only way to find a field the documents draw wider or
    // narrower than the silicon implements. The block is disabled first
    // because most of them are enable-protected.
    bench.verdict("the block disables", Tsens::enable(false));

    Tsens::regs().TSENS_CTRLC = 0xFF;
    const uint8_t ctrlc = Tsens::regs().TSENS_CTRLC;
    Tsens::regs().TSENS_EVCTRL = 0xFF;
    const uint8_t evctrl = Tsens::regs().TSENS_EVCTRL;
    Tsens::regs().TSENS_INTENSET = 0xFF;
    const uint8_t intenset = Tsens::regs().TSENS_INTENSET;
    Tsens::regs().TSENS_INTENCLR = 0xFF;
    Tsens::regs().TSENS_DBGCTRL = 0xFF;
    const uint8_t dbgctrl = Tsens::regs().TSENS_DBGCTRL;
    Tsens::regs().TSENS_WINLT = 0xFFFFFFFFul;
    const uint32_t winlt = Tsens::regs().TSENS_WINLT;
    Tsens::regs().TSENS_CAL = 0xFFFFFFFFul;
    const uint32_t cal = Tsens::regs().TSENS_CAL;
    const uint8_t ctrla_before = Tsens::regs().TSENS_CTRLA;
    Tsens::regs().TSENS_CTRLA = static_cast<uint8_t>(0xFFu & ~TSENS_CTRLA_SWRST_Msk &
                                                     ~TSENS_CTRLA_ENABLE_Msk);
    const uint8_t ctrla = Tsens::regs().TSENS_CTRLA;
    Tsens::regs().TSENS_CTRLA = ctrla_before;

    print(serial, "  register masks read back: CTRLA=", hex(ctrla),
          " CTRLC=", hex(ctrlc), " EVCTRL=", hex(evctrl),
          " INTENSET=", hex(intenset), " DBGCTRL=", hex(dbgctrl), crlf);
    print(serial, "  wide registers: WINLT=", hex(winlt), " CAL=", hex(cal),
          " SYNCBUSY=", hex(Tsens::sync_busy()), crlf);
    bench.verdict("CTRLC is the five bits 43.8.3 draws (WINMODE and FREERUN)",
                  ctrlc == 0x17u);
    bench.verdict("EVCTRL is three bits", evctrl == 0x07u);
    bench.verdict("the interrupt set is four bits", intenset == 0x0Fu);
    bench.verdict("DBGCTRL is one bit", dbgctrl == 0x01u);
    bench.verdict("RUNSTDBY is CTRLA's only other implemented bit",
                  ctrla == TSENS_CTRLA_RUNSTDBY_Msk);
    bench.verdict("WINLT IS TWENTY-FOUR BITS WIDE, not sixteen - the register "
                  "summary and the device header agree and 43.6.4's "
                  "'more than 16 bits' does not",
                  winlt == 0x00FFFFFFul);
    bench.verdict("CAL is two six-bit fields eight bits apart", cal == 0x00003F3Ful);

    // ---- what a software reset keeps (43.8.1) ---------------------------
    const TsensCalibration before = Tsens::calibration();
    Tsens::regs().TSENS_DBGCTRL = TSENS_DBGCTRL_DBGRUN_Msk;
    bench.verdict("the block resets", Tsens::reset());
    const TsensCalibration after = Tsens::calibration();
    bench.verdict("A SOFTWARE RESET KEEPS GAIN, OFFSET AND CAL - 43.8.1 says "
                  "so and the whole calibration would otherwise be lost on "
                  "every reconfiguration",
                  after.gain == before.gain && after.offset == before.offset &&
                      after.tcal == before.tcal && after.fcal == before.fcal);
    bench.verdict("and it keeps DBGCTRL too",
                  (Tsens::regs().TSENS_DBGCTRL & TSENS_DBGCTRL_DBGRUN_Msk) != 0u);
    Tsens::regs().TSENS_DBGCTRL = 0;
    bench.verdict("while CTRLC goes back to zero", Tsens::regs().TSENS_CTRLC == 0u);

    // ---- enable protection, measured raw --------------------------------
    bench.verdict("the block comes back up", tsens_up(gen_sys, base_cfg()));
    const uint8_t ctrlc_run = Tsens::regs().TSENS_CTRLC;
    Tsens::regs().TSENS_CTRLC = TSENS_CTRLC_FREERUN_Msk;
    const bool ctrlc_took = Tsens::regs().TSENS_CTRLC != ctrlc_run;
    const uint8_t evctrl_run = Tsens::regs().TSENS_EVCTRL;
    Tsens::regs().TSENS_EVCTRL = TSENS_EVCTRL_WINEO_Msk;
    const bool evctrl_took = Tsens::regs().TSENS_EVCTRL != evctrl_run;
    const uint32_t gain_run = Tsens::regs().TSENS_GAIN;
    Tsens::regs().TSENS_GAIN = gain_run ^ 1u;
    const bool gain_took = Tsens::regs().TSENS_GAIN != gain_run;
    const uint32_t winut_run = Tsens::regs().TSENS_WINUT;
    Tsens::regs().TSENS_WINUT = 0x1234u;
    const bool winut_took = Tsens::regs().TSENS_WINUT != winut_run;
    const uint8_t dbg_run = Tsens::regs().TSENS_DBGCTRL;
    Tsens::regs().TSENS_DBGCTRL = TSENS_DBGCTRL_DBGRUN_Msk;
    const bool dbg_took = Tsens::regs().TSENS_DBGCTRL != dbg_run;
    Tsens::regs().TSENS_DBGCTRL = dbg_run;

    print(serial, "  under a RUNNING block, a write is taken by: CTRLC ",
          yes_no(ctrlc_took), ", EVCTRL ", yes_no(evctrl_took), ", GAIN ",
          yes_no(gain_took), ", WINUT ", yes_no(winut_took), ", DBGCTRL ",
          yes_no(dbg_took), crlf);
    bench.verdict("43.6.2.1's enable-protected list holds: CTRLC, EVCTRL, "
                  "WINUT and GAIN all DISCARD a write while the block runs",
                  !ctrlc_took && !evctrl_took && !gain_took && !winut_took);
    bench.verdict("and DBGCTRL, which is not on that list, TAKES one",
                  dbg_took);

    // ---- the verbs that refuse rather than store ------------------------
    bench.verdict("the driver refuses to move the window while enabled",
                  !Tsens::window(TsensWindow::above, 0, 0));
    bench.verdict("...and the event control", !Tsens::event_config(TsensEventControl{}));
    bench.verdict("...and the calibration",
                  !Tsens::calibration(TsensCalibration{.gain = 1000}));
    bench.verdict("...and start_on(), whose EVCTRL half is protected",
                  !Tsens::start_on(ev_start_channel,
                                   EventChannelConfig{.path = EventPath::asynchronous}));

    // ---- the configuration refusals, which no register could express ----
    bench.verdict("WINMODE 0x7 is Reserved and refused",
                  !tsens_config_valid(TsensConfig{
                      .calibration = factory, .window = static_cast<TsensWindow>(7)}));
    bench.verdict("A ZERO GAIN IS REFUSED - it is the reset value, and 43.6.1's "
                  "formula makes it report the OFFSET at every temperature",
                  !tsens_config_valid(TsensConfig{}));
    bench.verdict("a crossed threshold pair is refused for INSIDE",
                  !tsens_config_valid(TsensConfig{.calibration = factory,
                                                  .window = TsensWindow::inside,
                                                  .window_lower = 6000,
                                                  .window_upper = 2000}));
    bench.verdict("and ACCEPTED for OUTSIDE, whose two descriptions want the "
                  "thresholds in opposite orders (letter g asks the silicon)",
                  tsens_config_valid(TsensConfig{.calibration = factory,
                                                 .window = TsensWindow::outside,
                                                 .window_lower = 6000,
                                                 .window_upper = 2000}));
    bench.verdict("a threshold past the 24-bit field is refused",
                  !tsens_config_valid(TsensConfig{.calibration = factory,
                                                  .window = TsensWindow::above,
                                                  .window_lower = tsens_value_max + 1}));
    bench.verdict("an inverted event input nothing listens to is refused",
                  !tsens_config_valid(TsensConfig{
                      .calibration = factory,
                      .events = TsensEventControl{.invert_start = true}}));

    // ---- the datum's arithmetic -----------------------------------------
    bench.verdict("43.8.10's own example round-trips: 0x0009C4 is 2500 "
                  "centi-C and 0xFFF63C is -2500",
                  tsens_signed(0x0009C4ul) == 2500 &&
                      tsens_signed(0xFFF63Cul) == -2500 &&
                      tsens_field(-2500) == 0xFFF63Cul);
    bench.verdict("the field saturates at +-2^23 and not at +-2^15",
                  tsens_signed(0x7FFFFFul) == 8388607 &&
                      tsens_signed(0x800000ul) == -8388608);

    Tsens::release();
}

// =============================================================================
// b - the factory calibration, and what each part of it is worth
// =============================================================================
//
// 43.5.9 says the four production values "must be loaded from the NVM
// Temperature Calibration Area ... to achieve specified accuracy" and
// stops there. This letter asks what each of them actually buys, by
// loading them one group at a time and reading the same die each time.
// samc/nvm.hpp has typed all four since the NVMCTRL pass; this is the
// promise that file's comment made, kept.
void tb_calibration() {
    const NvmTemperatureCalibration raw = NvmTemperatureCalibration::read();
    print(serial, "  NVM temperature calibration area at 0x00806030: words ",
          hex(raw.word0), " ", hex(raw.word1), crlf);
    print(serial, "  decoded: GAIN ", factory.gain, ", OFFSET ", factory.offset,
          ", TCAL ", factory.tcal, ", FCAL ", factory.fcal, crlf);
    bench.verdict("the calibration area is programmed - GAIN is neither zero "
                  "nor the all-ones of an erased row",
                  factory.programmed());

    // ---- the whole calibration, which is the reference reading ----------
    bench.verdict("the sensor comes up fully calibrated", tsens_up(gen_sys, base_cfg()));
    const Batch full = take(20);
    bench.verdict("and measures", full.ok);
    print_batch("fully calibrated", full);

    // WHICH WAY THE GAIN TERM POINTS is not in the chapter and is worth
    // one line, because everything else in this suite is arithmetic on it:
    // 43.6.1 writes the term as GAIN x (f_TOSCMIN - f_TOSCMAX) / f_GCLK
    // without saying which of the two frequencies is the larger.
    print(serial, "  the span from the OFFSET is ", full.mean - factory.offset,
          " centi-C at this temperature", crlf);
    bench.verdict("THE GAIN TERM IS NEGATIVE: the OFFSET sits ABOVE the "
                  "reading, so 43.6.1's f_TOSCMIN is the SMALLER of the two "
                  "and the counter's second phase outruns its first",
                  full.ok && full.mean < factory.offset);

    // ---- GAIN and OFFSET, but CAL left at zero --------------------------
    //
    // TCAL and FCAL trim the temperature-dependent oscillator itself, so
    // clearing them does not change the SCALE - it changes what is being
    // scaled.
    TsensConfig no_cal = base_cfg();
    no_cal.calibration.tcal = 0;
    no_cal.calibration.fcal = 0;
    bench.verdict("the sensor comes up with CAL cleared", tsens_up(gen_sys, no_cal));
    const Batch without = take(20);
    bench.verdict("and still measures", without.ok);
    print_batch("CAL = 0 (TCAL and FCAL not loaded)", without);

    if (full.ok && without.ok) {
        const int32_t delta = without.mean - full.mean;
        const int32_t noise = full.spread() + without.spread();
        print(serial, "  clearing CAL moves the reading by ", delta,
              " centi-C (");
        print_celsius(delta);
        print(serial, " C) against a combined spread of ", noise, crlf);
        if (abs_signed(delta) > noise) {
            bench.verdict("CAL.TCAL and CAL.FCAL ARE WORTH A MEASURABLE "
                          "NUMBER OF DEGREES - 43.8.15's 'must be copied' is "
                          "not decoration",
                          true);
        } else {
            print(serial, "  DECLINED: the shift does not clear this bench's "
                          "own noise, so nothing is claimed about it", crlf);
            bench.verdict("the CAL shift is reported and not judged - it is "
                          "inside the reading's own spread",
                          true);
        }
    }

    // ---- nothing loaded at all: the reset state -------------------------
    //
    // The driver refuses this configuration, so the registers are written
    // by hand. That is the point of the refusal: what it protects against
    // has to be shown once.
    const uint32_t gain_before = Tsens::gain();
    bench.verdict("the driver REFUSES an uncalibrated configuration",
                  !Tsens::init(gen_sys, TsensConfig{}));
    bench.verdict("and a refused init leaves the block EXACTLY as it was - it "
                  "returns before touching a single register",
                  Tsens::enabled() && Tsens::gain() == gain_before);
    bench.verdict("a bare block comes up", tsens_up(gen_sys, base_cfg()));
    bench.verdict("...and disables", Tsens::enable(false));
    Tsens::regs().TSENS_GAIN = 0;
    Tsens::regs().TSENS_OFFSET = 0;
    Tsens::regs().TSENS_CAL = 0;
    bench.verdict("...and comes back with every calibration register zero",
                  Tsens::enable(true) && Tsens::gain() == 0u && Tsens::offset() == 0);
    Tsens::clear_flags(Tsens::flag_all);
    // TIMED, and with a wait measured in SECONDS rather than in the poll
    // counts read() uses - because what comes back is not what the reset
    // value looks like it should be, and the time is half the evidence.
    const uint32_t blank_start = Ticker::millis();
    Tsens::start();
    while (!Tsens::result_ready() && Ticker::millis() - blank_start < 3000UL) {
    }
    const uint32_t blank_ms = Ticker::millis() - blank_start;
    const bool blank_finished = Tsens::result_ready();
    const int32_t blank_value = Tsens::value();
    print(serial, "  with GAIN = OFFSET = CAL = 0: RESRDY ", yes_no(blank_finished),
          " after ", blank_ms, " ms, STATUS.OVF ", yes_no(Tsens::overflowed()),
          ", VALUE ", blank_value, crlf);
    bench.verdict("an uncalibrated block does finish a measurement", blank_finished);

    // THE RESET VALUE OF GAIN IS NOT ZERO GAIN. 43.8.13 says GAIN
    // "defines the number of GCLK_TSENS periods that will be used for a
    // measurement cycle", and the field is 24 bits - so the value that
    // reads as 0 is the FULL 2^24, not none. Two independent consequences
    // say so, and both are checked here.
    constexpr int64_t gain_zero_means = 16777216;   // 2^24
    if (blank_finished && without.ok) {
        // (1) THE MAGNITUDE. The gain term scales with GAIN, so the same
        // die measured with CAL cleared (which is what `without` is) at
        // 2^24 instead of the factory GAIN must be that term multiplied
        // by 2^24 / GAIN - about 192 here - with OFFSET zero.
        const int64_t term = static_cast<int64_t>(without.mean) - factory.offset;
        const int32_t predicted = static_cast<int32_t>(
            (term * gain_zero_means) / static_cast<int64_t>(factory.gain));
        print(serial, "  the same die with CAL cleared has a gain term of ",
              static_cast<int32_t>(term), " at GAIN ", factory.gain,
              "; at 2^24 that predicts ", predicted, crlf);
        bench.verdict("GAIN'S RESET VALUE IS NOT NO GAIN - IT IS 2^24. The "
                      "reading is the CAL-cleared gain term multiplied by "
                      "2^24 / GAIN, to under a per cent, which no other "
                      "reading of a zero GAIN can produce",
                      near_signed(blank_value, predicted,
                                  abs_signed(predicted) / 50 + 200));
        // (2) THE TIME. Letter e measures a conversion at 2 x GAIN
        // GCLK_TSENS periods; 2 x 2^24 at 48 MHz is about 700 ms, and
        // that is what the stopwatch above just saw.
        const uint32_t predicted_ms = static_cast<uint32_t>(
            (2u * gain_zero_means * 1000) / nominal_hz);
        print(serial, "  and 2 x 2^24 periods at 48 MHz is ", predicted_ms,
              " ms against the ", blank_ms, " ms measured", crlf);
        bench.verdict("...and the SECOND witness agrees: the measurement takes "
                      "the seven hundred milliseconds 2^24 periods cost, "
                      "against the four a factory-GAIN one does",
                      near(blank_ms, predicted_ms, predicted_ms / 5u + 10u));
        bench.verdict("WHICH IS WHY THE DRIVER REFUSES A ZERO GAIN: the reset "
                      "value is not a benign nothing but a two-hundredfold "
                      "amplifier with a 0.7-second conversion, and the number "
                      "it hands back looks like a temperature of -16000 C",
                      !tsens_config_valid(TsensConfig{}));
    }

    // ---- OFFSET alone is a pure shift -----------------------------------
    bench.verdict("the block disables again", Tsens::enable(false));
    Tsens::regs().TSENS_GAIN = factory.gain;
    Tsens::regs().TSENS_OFFSET = tsens_field(factory.offset + 1000);
    Tsens::regs().TSENS_CAL = factory.cal_word();
    bench.verdict("...and comes back with the OFFSET moved by +1000 centi-C",
                  Tsens::enable(true));
    const Batch shifted = take(10);
    bench.verdict("and measures", shifted.ok);
    if (full.ok && shifted.ok) {
        print(serial, "  OFFSET + 1000 moves the reading by ",
              shifted.mean - full.mean, " centi-C", crlf);
        bench.verdict("OFFSET IS ADDED TO THE RESULT ONE FOR ONE, exactly as "
                      "43.6.1's formula puts it",
                      near_signed(shifted.mean - full.mean, 1000,
                                  full.spread() + shifted.spread() + 2));
    }

    Tsens::release();
}

// =============================================================================
// c - the reading at rest: jitter first, then a plausibility band
// =============================================================================
//
// THE ORDER OF THIS LETTER IS ITS ARGUMENT. The spread of the readings is
// measured and printed BEFORE anything is compared against a tolerance,
// and the only band applied to the temperature itself is a plausibility
// one - stated as such, because this bench has no thermometer and table
// 45-37's own accuracy is -11.3 .. +6.2 C.
void tc_at_rest() {
    bench.verdict("the sensor comes up fully calibrated on OSC48M",
                  tsens_up(gen_sys, base_cfg()));

    const Batch single = take(64);
    bench.verdict("64 single measurements all completed", single.ok);
    print_batch("64 single measurements", single);
    print(serial, "  the peak-to-peak spread is ", single.spread(),
          " centi-C = ", single.spread() / 100u, ".", (single.spread() % 100u) / 10u,
          " C of measurement noise, which is what every band below is "
          "chosen against", crlf);
    bench.verdict("the readings are not all identical - a sensor that never "
                  "moves is a sensor that is not measuring",
                  single.spread() > 0);
    bench.verdict("and the noise is degrees, not tens of degrees",
                  single.spread() < 1000);

    // 43.6.2.3's own recommendation, and table 45-37's own condition.
    const auto avg = Tsens::measure_average(10);
    bench.verdict("the chapter's recommended average of 10 completes",
                  avg.has_value());
    if (avg) {
        print(serial, "  average of 10 (43.6.2.3's recommendation, and the "
                      "condition table 45-37's accuracy is stated under): ",
              *avg, " centi-C = ");
        print_celsius(*avg);
        print(serial, " C", crlf);
        bench.verdict("the average lands inside the single readings' own range",
                      *avg >= single.min - 1 && *avg <= single.max + 1);
    }

    // Ten averages of ten, to show what the averaging buys.
    Batch of_averages{};
    {
        int64_t sum = 0;
        bool ok = true;
        for (uint16_t i = 0; i < 10; ++i) {
            const auto a = Tsens::measure_average(10);
            if (!a) {
                ok = false;
                break;
            }
            if (i == 0u || *a < of_averages.min) {
                of_averages.min = *a;
            }
            if (i == 0u || *a > of_averages.max) {
                of_averages.max = *a;
            }
            sum += *a;
        }
        if (ok) {
            of_averages.n = 10;
            of_averages.mean = static_cast<int32_t>((sum + 5) / 10);
            of_averages.ok = true;
        }
    }
    bench.verdict("ten averages of ten complete", of_averages.ok);
    if (of_averages.ok) {
        print_batch("ten averages of ten", of_averages);
        print(serial, "  averaging ten narrows the spread from ",
              single.spread(), " to ", of_averages.spread(), " centi-C", crlf);
        bench.verdict("AVERAGING NARROWS THE SPREAD - which is why 43.6.2.3 "
                      "asks for it and why table 45-37 is stated under it",
                      of_averages.spread() <= single.spread());
    }

    // THE ONLY STATEMENT MADE ABOUT THE TEMPERATURE ITSELF. It is a
    // plausibility band and it is deliberately enormous: a room this
    // program runs in is between freezing and the boiling point of water,
    // the die runs warmer than the room, and table 45-37 allows the
    // sensor eleven degrees of its own.
    if (single.ok) {
        print(serial, "  PLAUSIBILITY ONLY - no thermometer exists on this "
                      "bench, and table 45-37 allows -11.3 .. +6.2 C over "
                      "[0,60] C, so the band below is about the peripheral "
                      "working, not about its accuracy", crlf);
        bench.verdict("the die reads somewhere between 0 and 100 C, which is "
                      "where a working sensor in a room has to be",
                      single.mean > 0 && single.mean < 10000);
    }

    Tsens::release();
}

// =============================================================================
// d - THE SCALE LEVER: one die, two references, one prediction
// =============================================================================
//
// This is the letter this chapter exists for on a bench with no
// thermometer. 43.6.1's formula puts GCLK_TSENS in the DENOMINATOR:
//
//     VALUE = OFFSET + GAIN x (f_TOSCMIN - f_TOSCMAX) / f_GCLK
//
// so the same die at the same temperature reports different numbers on
// different clocks, and the difference is PREDICTABLE from the clocks
// alone. On this die OSC48M is thousands of ppm from the 48 MHz the
// factory GAIN assumes; a DPLL locked to the board's crystal is at a true
// 48 MHz. Read the same die on both and the difference is the internal
// oscillator's own error, mapped through the formula - and NO
// THERMOMETER IS INVOLVED.
//
// THE EXPERIMENT'S SHAPE IS ITS ARGUMENT. The predicted difference is
// tens of centi-degrees while a single reading's spread is about sixty,
// so the comparison is INTERLEAVED A-B-B-A and repeated: over four
// equally spaced batches a LINEAR drift of the die's own temperature
// cancels exactly out of (A1 + A4)/2 - (A2 + A3)/2, and the MEDIAN of the
// repeats is reported rather than the mean - the technique the RTC
// campaign's FREQCORR letter had to invent for the same reason, a signal
// smaller than the wander it sits on.
//
// The structural half needs no such care, because it is enormous: the
// same die read at 24 MHz with the factory GAIN unchanged must DOUBLE the
// span from the OFFSET, and both escapes offered by the header -
// tsens_gain_for() on the way in, tsens_rescale() on the way out - must
// put it back.
void td_scale_lever() {
    bench.verdict("the crystal starts and the 96 kHz reference is built",
                  crystal_up());

    const auto osc_hz = measure_hz(gen_sys);
    bench.verdict("OSC48M is weighed against the crystal", osc_hz.has_value());
    if (!osc_hz) {
        clocks_down();
        return;
    }
    const uint32_t osc_ppm = ppm_off(*osc_hz, nominal_hz);
    print(serial, "  OSC48M measures ", *osc_hz, " Hz on the crystal's scale, ",
          osc_ppm, " ppm ", *osc_hz < nominal_hz ? "SLOW" : "FAST",
          " of the 48 MHz the factory GAIN assumes", crlf);
    bench.verdict("the internal RC is inside table 45-57's +-5 % but is NOT "
                  "at its nominal rate - which is what makes this letter "
                  "possible at all",
                  osc_ppm > 500u && osc_ppm < 50'000u);

    bench.verdict("THE DPLL LOCKS TO THE CRYSTAL at a true 48 MHz", dpll_up());
    const auto dpll_hz = measure_hz(gen_dpll);
    bench.verdict("and the frequency meter agrees", dpll_hz.has_value());
    if (!dpll_hz) {
        clocks_down();
        return;
    }
    print(serial, "  the DPLL measures ", *dpll_hz, " Hz, ",
          ppm_off(*dpll_hz, nominal_hz), " ppm off nominal - the crystal's own "
          "error and nothing else, and it CANCELS out of the comparison "
          "because both frequencies were weighed on the same reference", crlf);
    bench.verdict("the crystal-locked loop is at 48 MHz to a few hundred ppm, "
                  "where OSC48M is thousands off",
                  ppm_off(*dpll_hz, nominal_hz) < osc_ppm);

    // ---- the interleaved comparison -------------------------------------
    constexpr uint8_t cycles = 4;
    constexpr uint16_t batch = 24;
    int32_t diffs[cycles];
    int32_t osc_mean = 0;
    int32_t dpll_mean = 0;
    int32_t worst_spread = 0;
    bool all_ok = true;
    for (uint8_t k = 0; k < cycles; ++k) {
        // A B B A: over four equally spaced batches a linear drift
        // cancels exactly, which is what lets a 40-centi-degree effect be
        // read off a reading whose own spread is 60.
        if (!tsens_up(gen_sys, base_cfg())) { all_ok = false; break; }
        const Batch a1 = take(batch);
        if (!tsens_up(gen_dpll, base_cfg())) { all_ok = false; break; }
        const Batch b1 = take(batch);
        const Batch b2 = take(batch);
        if (!tsens_up(gen_sys, base_cfg())) { all_ok = false; break; }
        const Batch a2 = take(batch);
        if (!a1.ok || !b1.ok || !b2.ok || !a2.ok) { all_ok = false; break; }
        diffs[k] = ((a1.mean + a2.mean) - (b1.mean + b2.mean)) / 2;
        osc_mean = (a1.mean + a2.mean) / 2;
        dpll_mean = (b1.mean + b2.mean) / 2;
        const int32_t s = a1.spread() > b1.spread() ? a1.spread() : b1.spread();
        if (s > worst_spread) {
            worst_spread = s;
        }
    }
    bench.verdict("four interleaved A-B-B-A cycles complete", all_ok);
    if (!all_ok) {
        clocks_down();
        return;
    }

    print(serial, "  per-cycle differences (OSC48M minus DPLL, centi-C): ");
    for (uint8_t k = 0; k < cycles; ++k) {
        print(serial, diffs[k], k + 1u == cycles ? "" : ", ");
    }
    print(serial, crlf);

    // The median of four, taken as the mean of the two middle values
    // after a bubble sort - four elements, and the sort is three lines.
    int32_t sorted[cycles];
    for (uint8_t k = 0; k < cycles; ++k) {
        sorted[k] = diffs[k];
    }
    for (uint8_t i = 0; i + 1u < cycles; ++i) {
        for (uint8_t j = 0; j + 1u + i < cycles; ++j) {
            if (sorted[j] > sorted[j + 1u]) {
                const int32_t t = sorted[j];
                sorted[j] = sorted[j + 1u];
                sorted[j + 1u] = t;
            }
        }
    }
    const int32_t median = (sorted[1] + sorted[2]) / 2;
    const int32_t cycle_spread = sorted[cycles - 1] - sorted[0];

    // The prediction. V_a - V_b = (V_a - OFFSET) x (1 - f_a / f_b), in 64
    // bits: a span of thousands times a rate of tens of millions leaves 32
    // at once.
    const int64_t span = static_cast<int64_t>(osc_mean) - factory.offset;
    const int32_t predicted = static_cast<int32_t>(
        span - (span * static_cast<int64_t>(*osc_hz)) /
                   static_cast<int64_t>(*dpll_hz));
    print(serial, "  span from the OFFSET ", static_cast<int32_t>(span),
          " centi-C; ", osc_ppm, " ppm of it PREDICTS ", predicted,
          " centi-C", crlf);
    print(serial, "  MEASURED median ", median, " centi-C (");
    print_celsius(median);
    print(serial, " C), cycle-to-cycle spread ", cycle_spread,
          ", single-reading spread ", worst_spread, crlf);
    print(serial, "  OSC48M mean ", osc_mean, ", DPLL mean ", dpll_mean, crlf);

    bench.verdict("THE INTERLEAVING IS WHAT MAKES THIS MEASURABLE: the four "
                  "cycles agree with each other far better than a single "
                  "reading's own spread",
                  cycle_spread < worst_spread);
    bench.verdict("THE SAME DIE READ ON TWO REFERENCES DIFFERS BY THE "
                  "REFERENCE'S OWN ERROR MAPPED THROUGH 43.6.1 - a "
                  "measurement no thermometer could have made, and one no "
                  "other chapter in this stratum offered",
                  near_signed(median, predicted,
                              abs_signed(predicted) / 4 + cycle_spread + 4));
    bench.verdict("...and the error rides on the SPAN and not on the "
                  "temperature, so a reading near zero degrees is off by far "
                  "more than its own ppm would suggest",
                  abs_signed(predicted) * 3 >
                      (abs_signed(osc_mean) * static_cast<int32_t>(osc_ppm)) / 1'000'000 * 3 + 1);

    // ---- the structural half: half the clock, the same GAIN --------------
    bench.verdict("the sensor moves to the bare crystal at 24 MHz",
                  tsens_up(gen_xtal, base_cfg()));
    const Batch on_xtal = take(24);
    bench.verdict("and measures there", on_xtal.ok);
    print_batch("GCLK_TSENS on the crystal (24 MHz), factory GAIN unchanged",
                on_xtal);
    if (on_xtal.ok) {
        const int32_t predicted_24 =
            factory.offset + 2 * (dpll_mean - factory.offset);
        print(serial, "  halving the clock should DOUBLE the span from the "
                      "offset: predicted ", predicted_24, " centi-C, measured ",
              on_xtal.mean, crlf);
        bench.verdict("THE 1/f LAW IS THE SILICON'S AND NOT AN INFERENCE: the "
                      "span from the OFFSET doubles when the ruler halves",
                      near_signed(on_xtal.mean, predicted_24,
                                  2 * on_xtal.spread() + 150));
        bench.verdict("...and the raw number at 24 MHz is nowhere near a "
                      "temperature, which is the trap 43.6.1's note warns "
                      "about in one sentence and nowhere else",
                      on_xtal.mean < -1000 || on_xtal.mean > 12000);
    }

    // ---- and both escapes the header offers ------------------------------
    TsensConfig scaled = base_cfg();
    scaled.calibration.gain = tsens_gain_for(factory.gain, crystal_hz);
    print(serial, "  tsens_gain_for(", factory.gain, ", 24 MHz) = ",
          scaled.calibration.gain, crlf);
    bench.verdict("the sensor comes up at 24 MHz with the GAIN scaled to it",
                  tsens_up(gen_xtal, scaled));
    const Batch corrected = take(24);
    bench.verdict("and measures", corrected.ok);
    print_batch("GCLK_TSENS on the crystal, GAIN scaled by tsens_gain_for()",
                corrected);
    if (corrected.ok) {
        print(serial, "  scaled-GAIN reading minus the true-48 MHz reading: ",
              corrected.mean - dpll_mean, " centi-C", crlf);
        bench.verdict("SCALING THE GAIN PUTS THE ANSWER BACK IN CENTI-DEGREES "
                      "on a clock the factory never calibrated for - the "
                      "escape 43.6.1's note offers in half a sentence, made "
                      "arithmetic",
                      near_signed(corrected.mean, dpll_mean,
                                  corrected.spread() + worst_spread + 60));
    }
    if (on_xtal.ok) {
        const int32_t rescaled =
            tsens_rescale(on_xtal.mean, factory.offset, crystal_hz);
        print(serial, "  tsens_rescale(", on_xtal.mean, ", 24 MHz) = ", rescaled,
              " centi-C against the true-48 MHz reading ", dpll_mean, crlf);
        bench.verdict("...and rescaling the READING instead of the GAIN gets "
                      "to the same place",
                      near_signed(rescaled, dpll_mean,
                                  on_xtal.spread() + worst_spread + 60));
    }

    clocks_down();
}

// =============================================================================
// e - what a measurement costs in time, ruled by the crystal
// =============================================================================
//
// Chapter 43 gives NO conversion time at all - neither a formula nor a
// table row - and only says that GAIN "defines the number of GCLK_TSENS
// periods that will be used for a measurement cycle". This letter turns
// that sentence into a number by timing a long run of free-running
// measurements against the board's crystal (never against OSC48M, which
// would put its own thousands of ppm into a statement about the TSENS),
// and then halving and quartering GAIN to see whether the cost follows.
//
// THE ANSWER COMES OUT AS A FORMULA WITH A CONSTANT IN IT, and getting
// that constant right needs the reference's error taken out: the ruler
// here is the crystal and the thing being counted is GCLK_TSENS periods,
// so the conversion between them is the MEASURED OSC48M rate and not its
// nominal one. FREQM supplies it, from the same crystal.
void te_timing() {
    bench.verdict("the crystal starts", crystal_up());
    bench.verdict("the 32-bit stopwatch runs on it", stopwatch_up());
    const auto osc_hz = measure_hz(gen_sys);
    bench.verdict("and GCLK_TSENS's own rate is weighed on the same crystal",
                  osc_hz.has_value());
    const uint32_t gclk_hz = osc_hz ? *osc_hz : nominal_hz;
    print(serial, "  GCLK_TSENS (OSC48M) measures ", gclk_hz,
          " Hz; the stopwatch runs on the crystal at ", crystal_hz, " Hz", crlf);

    // Free-running, so the measurement rate is the peripheral's own and
    // no CPU round trip is in it. Nothing is printed inside a timed
    // window: one console line at 115200 is about five milliseconds.
    auto period_ticks = [](uint32_t gain, uint16_t n) -> uint32_t {
        TsensConfig cfg = base_cfg();
        cfg.calibration.gain = gain;
        cfg.free_running = true;
        if (!tsens_up(gen_sys, cfg)) {
            return 0;
        }
        // Discard the first result: it was started by the enable and its
        // beginning is not in the window.
        (void)Tsens::read();
        Tsens::clear_flags(Tsens::flag_all);
        const uint32_t t0 = ticks_now();
        for (uint16_t i = 0; i < n; ++i) {
            if (!Tsens::read()) {
                return 0;
            }
        }
        const uint32_t t1 = ticks_now();
        return (t1 - t0) / n;
    };

    const uint32_t p_full = period_ticks(factory.gain, 64);
    bench.verdict("a free-running block at the factory GAIN is timed",
                  p_full != 0u);
    const uint32_t p_half = period_ticks(factory.gain / 2u, 64);
    bench.verdict("...and at half that GAIN", p_half != 0u);
    const uint32_t p_quarter = period_ticks(factory.gain / 4u, 64);
    bench.verdict("...and at a quarter", p_quarter != 0u);

    if (p_full && p_half && p_quarter) {
        print(serial, "  free-running period, crystal ticks (1/24 us each): "
                      "GAIN ", factory.gain, " gives ", p_full, " = ",
              ticks_us(p_full), " us; GAIN ", factory.gain / 2u, " gives ",
              p_half, " = ", ticks_us(p_half), " us; GAIN ", factory.gain / 4u,
              " gives ", p_quarter, " = ", ticks_us(p_quarter), " us", crlf);
        bench.verdict("THE COST IS PROPORTIONAL TO GAIN - halving it halves "
                      "the measurement, quartering it quarters it, so "
                      "43.8.13's 'number of GCLK_TSENS periods' is literal",
                      near(p_full, p_half * 2u, p_full / 20u + 4u) &&
                          near(p_full, p_quarter * 4u, p_full / 20u + 4u));

        // Crystal ticks turned into GCLK_TSENS periods with the rate FREQM
        // just measured. 43.6.1 describes TWO phases of GAIN periods each,
        // so the model to test is 2 x GAIN plus a fixed overhead - and
        // whether that overhead really is fixed is the whole question.
        auto periods = [gclk_hz](uint32_t ticks) {
            return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * gclk_hz) /
                                         crystal_hz);
        };
        const uint32_t n_full = periods(p_full);
        const uint32_t n_half = periods(p_half);
        const uint32_t n_quarter = periods(p_quarter);
        const int32_t over_full = static_cast<int32_t>(n_full) -
                                  2 * static_cast<int32_t>(factory.gain);
        const int32_t over_half = static_cast<int32_t>(n_half) -
                                  2 * static_cast<int32_t>(factory.gain / 2u);
        const int32_t over_quarter = static_cast<int32_t>(n_quarter) -
                                     2 * static_cast<int32_t>(factory.gain / 4u);
        print(serial, "  in GCLK_TSENS periods: ", n_full, " / ", n_half, " / ",
              n_quarter, " against 2 x GAIN = ", 2u * factory.gain, " / ",
              factory.gain, " / ", factory.gain / 2u, crlf);
        print(serial, "  so the OVERHEAD beyond the two phases is ", over_full,
              " / ", over_half, " / ", over_quarter, " periods", crlf);
        bench.verdict("A MEASUREMENT IS 2 x GAIN PERIODS PLUS A CONSTANT, and "
                      "the constant is the same at all three GAINs - a "
                      "conversion time chapter 43 states nowhere, not even as "
                      "a table row",
                      over_full > 0 && over_half > 0 && over_quarter > 0 &&
                          near(static_cast<uint32_t>(over_full),
                               static_cast<uint32_t>(over_quarter),
                               static_cast<uint32_t>(over_full) / 10u + 60u) &&
                          near(static_cast<uint32_t>(over_half),
                               static_cast<uint32_t>(over_quarter),
                               static_cast<uint32_t>(over_half) / 10u + 60u));
        bench.verdict("and the overhead is small beside the measurement it "
                      "sits on, so GAIN is what a caller trades against time",
                      static_cast<uint32_t>(over_full) < factory.gain / 4u);
    }

    // A single measurement, for a caller that is not free-running: the
    // difference of two long loops, because a single stopwatch read pair
    // on this target costs microseconds of its own (the DAC campaign's
    // lesson).
    bench.verdict("the block comes back to single measurements",
                  tsens_up(gen_sys, base_cfg()));
    const uint32_t s0 = ticks_now();
    for (uint16_t i = 0; i < 64; ++i) {
        (void)Tsens::measure();
    }
    const uint32_t s1 = ticks_now();
    const uint32_t single = (s1 - s0) / 64u;
    print(serial, "  a CPU-started single measurement costs ", single,
          " ticks = ", ticks_us(single), " us, against the free-running ",
          p_full, " ticks", crlf);
    bench.verdict("a single measurement costs at least what a free-running "
                  "one does - the start is on top of the measurement, not "
                  "instead of it",
                  p_full == 0u || single + 4u >= p_full);

    // The averaging verb's cost, which is ten of those.
    const uint32_t a0 = ticks_now();
    for (uint16_t i = 0; i < 8; ++i) {
        (void)Tsens::measure_average(10);
    }
    const uint32_t a1 = ticks_now();
    const uint32_t avg_cost = (a1 - a0) / 8u;
    print(serial, "  43.6.2.3's recommended average of ten costs ",
          ticks_us(avg_cost), " us", crlf);
    bench.verdict("the average of ten costs about ten measurements",
                  near(avg_cost, single * 10u, single * 2u + 100u));

    Tsens::release();
    (void)Stopwatch::enable(false);
    Stopwatch::release();
    clocks_down();
}

// =============================================================================
// f - how wide the datapath really is, and where its rail sits
// =============================================================================
//
// THE CHAPTER SAYS BOTH. 43.6.4 introduces the overflow interrupt as "the
// result required more than 16 bits and overflowed the VALUE register";
// 43.8.7's own bit description says TWENTY-FOUR, and the register
// summary, the bit table and TSENS_VALUE_Msk all draw VALUE[23:0]. Only
// the silicon can settle it, and this letter settles it twice over:
//
//   1. GAIN multiplies the temperature term (43.6.1), so a bigger GAIN
//      walks the result away from the OFFSET. A valid result past 32767
//      with no overflow at all convicts the "16 bits" sentence outright.
//   2. OFFSET is added to that term, so it MOVES the whole result without
//      changing what is being measured - which makes it a ruler that can
//      be walked up to the rail one step at a time. Where OVF first
//      appears, minus the term's own measured size, IS the rail.
//
// The rail can only be approached from BELOW on this die: the gain term
// is negative at room temperature (letter b) and OFFSET's own field stops
// at +2^23 - 1, so the positive rail is out of reach and this letter says
// so rather than pretending otherwise.
void tf_datapath_width() {
    bench.verdict("the sensor comes up fully calibrated", tsens_up(gen_sys, base_cfg()));
    const auto base = Tsens::measure_average(10);
    bench.verdict("and gives a reference reading", base.has_value());
    if (!base) {
        Tsens::release();
        return;
    }
    const int32_t span = *base - factory.offset;
    print(serial, "  reference reading ", *base, " centi-C, gain term ", span,
          " at GAIN ", factory.gain, crlf);
    bench.verdict("the gain term is a few thousand counts, which is what makes "
                  "both levers below work",
                  abs_signed(span) > 1000 && abs_signed(span) < 100000);

    // ---- lever 1: a result past 16 bits, with no overflow ---------------
    TsensConfig big = base_cfg();
    big.calibration.gain = factory.gain * 8u;
    bench.verdict("the sensor comes up with GAIN multiplied by eight",
                  tsens_up(gen_sys, big));
    const auto wide = Tsens::measure();
    bench.verdict("and measures", wide.has_value());
    if (wide) {
        print(serial, "  at GAIN ", factory.gain * 8u, " the result is ", *wide,
              " - INTFLAG.OVF ", yes_no(Tsens::overflow_flag()),
              ", STATUS.OVF ", yes_no(Tsens::overflowed()), crlf);
        bench.verdict("THE DATAPATH IS WIDER THAN SIXTEEN BITS: a result well "
                      "past +-32767 stands with no overflow at all, which "
                      "convicts 43.6.4's 'more than 16 bits' and acquits "
                      "43.8.7's 'more than 24'",
                      abs_signed(*wide) > 32767 && !Tsens::overflowed() &&
                          !Tsens::overflow_flag());
        bench.verdict("and the amplified result is eight times the gain term, "
                      "so GAIN really is a linear multiplier and not a mode",
                      near_signed(*wide - factory.offset, span * 8,
                                  abs_signed(span) / 2 + 200));
    }

    // ---- lever 2: walk OFFSET down to the rail --------------------------
    //
    // Each step moves the whole result by a known amount without touching
    // the measurement, so the first OFFSET that overflows, plus the gain
    // term, is the rail itself. The step is the resolution of the answer.
    constexpr int32_t scan_start = -8'378'000;
    constexpr int32_t scan_step = 250;
    int32_t first_ovf = 0;
    int32_t last_ok = 0;
    uint32_t ovf_raw = 0;
    for (int32_t off = scan_start; off >= tsens_value_min; off -= scan_step) {
        TsensConfig cfg = base_cfg();
        cfg.calibration.offset = off;
        if (!tsens_up(gen_sys, cfg)) {
            break;
        }
        Tsens::clear_flags(Tsens::flag_all);
        const auto v = Tsens::measure();
        if (v && !Tsens::overflowed()) {
            last_ok = *v;
            continue;
        }
        first_ovf = off;
        ovf_raw = Tsens::value_raw();
        print(serial, "  OFFSET ", off, " OVERFLOWS: STATUS.OVF ",
              yes_no(Tsens::overflowed()), ", INTFLAG.OVF ",
              yes_no(Tsens::overflow_flag()), ", INTFLAG.RESRDY ",
              yes_no(Tsens::result_ready()), ", VALUE register ",
              hex(Tsens::value_raw()), crlf);
        bench.verdict("43.8.7 IS EXACT: RESRDY DOES NOT SET WHEN THE "
                      "CONVERSION OVERFLOWED - 'this flag will not set if an "
                      "overflow occurs during the conversion'",
                      !Tsens::result_ready());
        bench.verdict("and read() reports nothing rather than a number 43.8.8 "
                      "calls invalid",
                      !v.has_value());
        break;
    }
    bench.verdict("an overflow was reached by walking the OFFSET down",
                  first_ovf != 0);
    if (first_ovf != 0) {
        const int32_t rail = first_ovf + span;
        print(serial, "  the last result that fitted was ", last_ok,
              "; the first overflow puts the rail at ", rail,
              " +-", scan_step, crlf);
        print(serial, "  2^23 is ", tsens_value_min, " and 2^15 is -32768",
              crlf);
        bench.verdict("THE RAIL IS AT -2^23, LOCATED TO A FEW HUNDRED COUNTS "
                      "IN EIGHT MILLION - the VALUE register is a signed "
                      "24-bit datum, exactly as its own bit table draws it",
                      near_signed(rail, tsens_value_min, 2 * scan_step + 400));
        bench.verdict("...and the last result before it is nowhere near a "
                      "16-bit ceiling, which no reading of 43.6.4 survives",
                      abs_signed(last_ok) > 8'000'000);
        // AND WHAT THE REGISTER HOLDS AFTER AN OVERFLOW, which 43.8.8 does
        // not say and which matters to anyone who skips the flag: the
        // datum WRAPS, it does not saturate. A result of about -2^23 - 250
        // comes back as +2^23 - 250 - the wrong SIGN, and a plausible
        // number. (samc/sdadc.hpp's converter saturates instead, so the
        // two blocks on this die do opposite things.)
        const int32_t wrapped = static_cast<int32_t>(
            (static_cast<int64_t>(rail) + 16777216) % 16777216);
        print(serial, "  the overflowed VALUE register held ", hex(ovf_raw),
              " = ", tsens_signed(ovf_raw),
              ", where a WRAP of the true result predicts ", wrapped,
              " and a SATURATION would give ", tsens_value_min, crlf);
        bench.verdict("AN OVERFLOWED VALUE WRAPS RATHER THAN SATURATING - so "
                      "a caller who ignores STATUS.OVF gets a plausible "
                      "number OF THE WRONG SIGN, where this stratum's other "
                      "converter saturates at its rail instead",
                      near_signed(tsens_signed(ovf_raw), wrapped, 4 * scan_step) &&
                          tsens_signed(ovf_raw) > 0);
    }
    print(serial, "  THE POSITIVE RAIL IS OUT OF REACH: the gain term is "
                  "negative on this die and OFFSET's own field stops at "
                  "+2^23 - 1, so nothing can push the result up to it", crlf);

    // The overflow flag's own discipline: sticky, cleared only by a one.
    if (Tsens::overflow_flag()) {
        Tsens::clear_flags(Tsens::flag_overflow);
        bench.verdict("INTFLAG.OVF is cleared by writing a one to it",
                      !Tsens::overflow_flag());
    }
    bench.verdict("the sensor goes back to the factory calibration",
                  tsens_up(gen_sys, base_cfg()));
    const auto back = Tsens::measure();
    bench.verdict("and reads a plausible temperature again",
                  back.has_value() && *back > 0 && *back < 10000);
    bench.verdict("with STATUS.OVF - the LEVEL beside the latch - clear",
                  !Tsens::overflowed());

    // OVERRUN: a free-running block whose result nobody reads.
    TsensConfig fr = base_cfg();
    fr.free_running = true;
    bench.verdict("a free-running block comes up", tsens_up(gen_sys, fr));
    Tsens::clear_flags(Tsens::flag_all);
    wait_ms(20);
    bench.verdict("INTFLAG.OVERRUN sets when nobody reads VALUE - 43.6.2.3's "
                  "'failing to do so will result in an overrun error'",
                  Tsens::overrun());
    Tsens::clear_flags(Tsens::flag_overrun);
    (void)Tsens::value();
    bench.verdict("and clears by writing a one", !Tsens::overrun());

    Tsens::release();
}

// =============================================================================
// g - the window monitor, all six modes and the two that disagree
// =============================================================================
//
// The thresholds are placed AROUND THE CURRENT READING rather than at
// absolute temperatures, so this letter works in any room. Each mode is
// asked twice: once with the window arranged to catch the reading and
// once arranged to miss it, so "it fired" and "it stayed silent" are both
// verdicts and neither is a coin.
void tg_window() {
    bench.verdict("the sensor comes up fully calibrated", tsens_up(gen_sys, base_cfg()));
    const auto here = Tsens::measure_average(10);
    bench.verdict("and locates the current reading", here.has_value());
    if (!here) {
        Tsens::release();
        return;
    }
    const int32_t t = *here;
    // Wide enough that the reading's own noise cannot cross it: letter c
    // measures the spread in single digits of centi-degrees.
    constexpr int32_t margin = 2000;   // 20 C
    print(serial, "  the die reads ", t, " centi-C; every window below is "
                  "placed +-", margin, " centi-C around that", crlf);

    /// Arm one window and say whether it matched. The block is disabled
    /// around the write because WINLT, WINUT and CTRLC are all
    /// enable-protected.
    auto probe = [](TsensWindow mode, int32_t lower, int32_t upper) -> bool {
        TsensConfig cfg = base_cfg();
        cfg.window = mode;
        cfg.window_lower = lower;
        cfg.window_upper = upper;
        if (!tsens_up(gen_sys, cfg)) {
            return false;
        }
        Tsens::clear_flags(Tsens::flag_all);
        // Two measurements: the flag is set "on the next cycle after a
        // match" (43.8.7), and reading VALUE clears it.
        (void)Tsens::measure();
        Tsens::clear_flags(Tsens::flag_window);
        Tsens::start();
        uint32_t spins = 2'000'000UL;
        while (!Tsens::result_ready() && spins-- != 0u) {
        }
        return Tsens::window_hit();
    };

    // ABOVE: VALUE > WINLT.
    bench.verdict("ABOVE fires with the lower threshold below the reading",
                  probe(TsensWindow::above, t - margin, 0));
    bench.verdict("...and stays silent with it above",
                  !probe(TsensWindow::above, t + margin, 0));

    // BELOW: VALUE < WINUT.
    bench.verdict("BELOW fires with the upper threshold above the reading",
                  probe(TsensWindow::below, 0, t + margin));
    bench.verdict("...and stays silent with it below",
                  !probe(TsensWindow::below, 0, t - margin));

    // INSIDE: WINLT < VALUE < WINUT.
    bench.verdict("INSIDE fires with the reading between the thresholds",
                  probe(TsensWindow::inside, t - margin, t + margin));
    bench.verdict("...and stays silent with the window moved off it",
                  !probe(TsensWindow::inside, t + margin, t + 2 * margin));

    // OUTSIDE - THE MODE THE TWO DOCUMENTS DESCRIBE DIFFERENTLY. 43.8.3
    // prints "WINUT < VALUE < WINLT" (a band with the thresholds
    // reversed); the device header's enumerator comment says "VALUE <
    // WINLT or VALUE > WINUT" (the complement of INSIDE). The two
    // readings disagree on both the CONDITION and the ORDER, so the
    // silicon is asked directly.
    const bool outside_as_complement =
        probe(TsensWindow::outside, t + margin, t + 2 * margin);
    const bool outside_complement_silent =
        probe(TsensWindow::outside, t - margin, t + margin);
    const bool outside_as_reversed_band =
        probe(TsensWindow::outside, t + margin, t - margin);
    print(serial, "  OUTSIDE with WINLT above the reading and WINUT above it "
                  "(the header's complement reading would fire): ",
          yes_no(outside_as_complement), crlf);
    print(serial, "  OUTSIDE with the reading between the thresholds (the "
                  "header's reading would stay silent): ",
          yes_no(outside_complement_silent), crlf);
    print(serial, "  OUTSIDE with WINUT below and WINLT above, i.e. 43.8.3's "
                  "reversed band containing the reading: ",
          yes_no(outside_as_reversed_band), crlf);
    bench.verdict("OUTSIDE IS THE COMPLEMENT OF INSIDE - the device header's "
                  "enumerator comment is right and 43.8.3's printed "
                  "'WINUT < VALUE < WINLT' is not",
                  outside_as_complement && !outside_complement_silent);

    // The two hysteresis modes: from a cold start they behave like their
    // plain counterparts, which is all a bench with no temperature knob
    // can ask of them.
    bench.verdict("HYST_ABOVE fires from rest with the upper threshold below "
                  "the reading",
                  probe(TsensWindow::hysteresis_above, t - 2 * margin, t - margin));
    bench.verdict("...and stays silent with both thresholds above it",
                  !probe(TsensWindow::hysteresis_above, t + margin, t + 2 * margin));
    bench.verdict("HYST_BELOW fires from rest with the lower threshold above "
                  "the reading",
                  probe(TsensWindow::hysteresis_below, t + margin, t + 2 * margin));
    bench.verdict("...and stays silent with both thresholds below it",
                  !probe(TsensWindow::hysteresis_below, t - 2 * margin, t - margin));
    print(serial, "  THE HYSTERESIS ITSELF IS NOT EXERCISED: it needs the die "
                  "to cross a threshold and come back, and this bench has no "
                  "way to move the temperature on command", crlf);

    // Reading VALUE clears WINMON, exactly as it clears RESRDY (43.8.7).
    {
        TsensConfig cfg = base_cfg();
        cfg.window = TsensWindow::above;
        cfg.window_lower = t - margin;
        bench.verdict("a window is armed for the flag's own discipline",
                      tsens_up(gen_sys, cfg));
        (void)Tsens::measure();
        Tsens::clear_flags(Tsens::flag_all);
        Tsens::start();
        uint32_t spins = 2'000'000UL;
        while (!Tsens::result_ready() && spins-- != 0u) {
        }
        const bool hit = Tsens::window_hit();
        (void)Tsens::value();
        bench.verdict("WINMON stands after a match", hit);
        bench.verdict("and READING VALUE CLEARS IT, as it clears RESRDY",
                      !Tsens::window_hit() && !Tsens::result_ready());
    }

    bench.verdict("the window disables again",
                  tsens_up(gen_sys, base_cfg()) &&
                      Tsens::window() == TsensWindow::disabled);
    Tsens::release();
}

// =============================================================================
// h - the no-CPU chain: event in on every path, DMAC out, WINMON counted
// =============================================================================
//
// The stratum's signature letter, with one thing no other converter here
// could do: TABLE 29-3 GIVES THE TSENS START USER ALL THREE PROPAGATION
// PATHS, where the DAC's and the SDADC's are asynchronous-only. So the
// same chain is built three times, once per path, and all three must
// move bytes.
//
// AND THE CHANNEL'S OWN CLOCK IS PART OF THE EXPERIMENT, which is the
// thing this letter had to learn on the bench. The pacer's overflow event
// is one CPU-clock cycle wide; an asynchronous channel passes it through
// with no clock at all, but a SYNCHRONOUS or RESYNCHRONIZED channel
// SAMPLES it, so a channel clocked from OSCULP32K sees a 20 ns pulse
// once in fifteen hundred and the chain looks broken. The synchronous
// channel therefore runs on the generator the GENERATOR runs on (which is
// what "synchronous" means in 29.6.2.6) and the resynchronized one on a
// different but equally fast generator - the crystal.
void th_no_cpu() {
    bench.verdict("the crystal starts, for the resynchronized channel's own "
                  "clock",
                  crystal_up());
    bench.verdict("and the DPLL locks to it, so a SECOND 48 MHz domain exists "
                  "for the resynchronized path to cross into",
                  dpll_up());
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the event channels get a clock",
                  GenSlow::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_start_channel), gen_slow) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_window_channel), gen_slow));

    /// TC2 overflowing at about 1 kHz, its overflow an event - built on a
    /// named generator, because THE EVENT PULSE IS ONE GCLK_TC PERIOD WIDE
    /// and that width is what a sampled channel has to catch. Two
    /// configurations are used below and they differ only in that width:
    ///   gen_ref (96 kHz, prescaler 16) - a 10.4 us pulse;
    ///   gen_sys (48 MHz, prescaler 1024) - a 21 ns one.
    /// Both overflow at about a kilohertz.
    ///
    /// AND IT NEVER CALLS Tc<2>::release(). TC2 and TC3 SHARE GENERIC CLOCK
    /// CHANNEL 31 (TC0/TC1 share 30, TC4 has 32 to itself), so releasing
    /// one silently stops the other - which is exactly how the first
    /// version of this letter lost the window-monitor half of its own
    /// chain, with every verdict before it still passing.
    auto pacer_up = [](uint8_t generator, TcPrescaler prescaler,
                       uint8_t period) {
        (void)Pacer::enable(false);
        const TcConfig cfg{.mode = TcMode::count8,
                           .prescaler = prescaler,
                           .waveform = TcWaveform::normal_pwm};
        return Pacer::init(generator) && Pacer::configure(cfg) &&
               Pacer::set_period8(period) &&
               Pacer::event_config(cfg, TcEventConfig{.overflow_out = true});
    };
    bench.verdict("TC2 paces at about 1 kHz off the 48 MHz generator, and its "
                  "overflow becomes an event",
                  pacer_up(gen_sys, TcPrescaler::div1024, 46));
    bench.verdict("TC3 is set up to COUNT events rather than clock ticks",
                  Counter::init(gen_sys) &&
                      Counter::configure(TcConfig{.mode = TcMode::count16}) &&
                      Counter::event_config(TcConfig{.mode = TcMode::count16},
                                            TcEventConfig{
                                                .action = TcEventAction::count,
                                                .input_enable = true}));

    /// The GAIN the whole chain runs at - a quarter of the factory value,
    /// so a measurement is under a millisecond and the pacer's own
    /// millisecond is never the bottleneck. EVERY VERDICT BELOW COMPARES
    /// AGAINST A READING TAKEN AT THIS SAME GAIN: a number measured at one
    /// GAIN says nothing about one measured at another, which is how the
    /// first version of this letter fooled itself.
    const uint32_t chain_gain = factory.gain / 4u;
    TsensConfig chain_cfg = base_cfg();
    chain_cfg.calibration.gain = chain_gain;
    bench.verdict("a reference reading is taken AT THE CHAIN'S OWN GAIN",
                  tsens_up(gen_sys, chain_cfg));
    const auto chain_ref = Tsens::measure_average(10);
    bench.verdict("and it measures", chain_ref.has_value());
    const int32_t reference = chain_ref ? *chain_ref : 0;
    print(serial, "  at GAIN ", chain_gain, " the same die reads ", reference,
          " - not a temperature any more, and the yardstick for every value "
          "the DMAC brings back below", crlf);

    /// Run the chain once on one propagation path, with the channel clock
    /// the path needs, and return how many beats the DMAC moved.
    auto run = [&chain_cfg](EventPath path, EventEdge edge,
                            uint8_t channel_generator) -> uint16_t {
        for (uint16_t i = 0; i < dma_results; ++i) {
            results[i] = 0xFFFFFFFFul;
        }
        if (!GclkChannel::connect(Evsys::gclk_id(ev_start_channel),
                                  channel_generator)) {
            return 0;
        }
        if (!tsens_up(gen_sys, chain_cfg) || !Tsens::enable(false)) {
            return 0;
        }
        if (!Tsens::start_on(ev_start_channel,
                             EventChannelConfig{.generator = Pacer::overflow_generator,
                                                .path = path,
                                                .edge = edge})) {
            return 0;
        }
        // THE DMA REQUEST IS THE RESRDY FLAG (43.6.3: "cleared when the
        // VALUE register is read"), so a result left standing from a
        // previous run would move one stale beat the moment the channel
        // is enabled. Read it away first - the ADC campaign's lesson.
        (void)Tsens::value();
        Tsens::clear_flags(Tsens::flag_all);
        (void)Copy::reset();
        (void)Copy::configure(DmaChannelConfig{
            .trigger = Tsens::dma_trigger_resrdy,
            .action = DmaTriggerAction::beat,
        });
        (void)Copy::load(DmaTransfer{
            .source = &Tsens::regs().TSENS_VALUE,
            .destination = &results[0],
            .beats = dma_results,
            .beat = DmaBeat::word,
            .source_increment = false,
        });
        (void)Copy::enable(true);
        (void)Tsens::enable(true);
        (void)Pacer::enable(true);
        wait_ms(60);
        (void)Pacer::enable(false);
        (void)Tsens::enable(false);
        (void)Copy::enable(false);
        uint16_t filled = 0;
        for (uint16_t i = 0; i < dma_results; ++i) {
            if (results[i] != 0xFFFFFFFFul) {
                ++filled;
            }
        }
        return filled;
    };

    /// Are all sixteen values the same measurement the reference was?
    auto all_near_reference = [reference]() {
        for (uint16_t i = 0; i < dma_results; ++i) {
            if (results[i] == 0xFFFFFFFFul ||
                !near_signed(tsens_signed(results[i]), reference, 400)) {
                return false;
            }
        }
        return true;
    };

    // The ASYNCHRONOUS path takes no channel clock at all, but the channel
    // still wants one connected, so it keeps the slow generator.
    const uint16_t async_beats =
        run(EventPath::asynchronous, EventEdge::none, gen_slow);
    print(serial, "  ASYNCHRONOUS path (channel on OSCULP32K): ", async_beats,
          " of ", dma_results, " results moved by the DMAC with the CPU in a "
          "wait loop; last value ", tsens_signed(results[dma_results - 1]), crlf);
    bench.verdict("THE DMAC FILLED THE BUFFER FROM VALUE, one beat per "
                  "measurement, with no CPU in the path",
                  async_beats == dma_results);
    bench.verdict("and every value it brought back is the same measurement the "
                  "CPU took at that GAIN",
                  all_near_reference());

    // The SYNCHRONOUS path on the generator the GENERATOR itself runs on -
    // which is what 29.6.2.6 means by synchronous.
    const uint16_t sync_beats =
        run(EventPath::synchronous, EventEdge::rising, gen_sys);
    print(serial, "  SYNCHRONOUS path (channel on the pacer's own 48 MHz "
                  "generator): ", sync_beats, " of ", dma_results, crlf);

    // The RESYNCHRONIZED path is the one that crosses domains, and it
    // needs a pulse WIDE ENOUGH FOR THE CHANNEL TO SEE. The pacer moves to
    // the crystal-derived 96 kHz generator, which widens its overflow
    // event from about 21 ns to 10.4 us and changes nothing else - same
    // 1 kHz rate - while the channel stays on OSC48M. Two domains that
    // share nothing, which is what this path exists for.
    bench.verdict("the pacer moves to the crystal-derived 96 kHz generator - "
                  "same 1 kHz rate, a five-hundred-times WIDER pulse",
                  pacer_up(gen_ref, TcPrescaler::div16, 5));
    const uint16_t resync_beats =
        run(EventPath::resynchronized, EventEdge::rising, gen_sys);
    print(serial, "  RESYNCHRONIZED path (pacer on the crystal, channel on "
                  "OSC48M - two domains that share nothing): ", resync_beats,
          " of ", dma_results, crlf);
    bench.verdict("TABLE 29-3 IS EXACT AND THIS USER IS THE EXCEPTION: the "
                  "START user takes the synchronous and resynchronized paths "
                  "too, where the DAC's and the SDADC's take neither",
                  sync_beats == dma_results && resync_beats == dma_results);

    // AND THE OTHER HALF OF THE SAME LESSON, MEASURED RATHER THAN ASSERTED.
    // A SAMPLED PATH SAMPLES, and neither 29.6.2.6 nor table 29-3 says
    // what that costs. Two knobs are turned one at a time: the pulse's
    // width (the pacer's generator) and the channel's own clock.
    const uint16_t wide_slow_sync =
        run(EventPath::synchronous, EventEdge::rising, gen_ref);
    bench.verdict("the pacer goes back to the narrow 21 ns pulse",
                  pacer_up(gen_sys, TcPrescaler::div1024, 46));
    const uint16_t narrow_async =
        run(EventPath::asynchronous, EventEdge::none, gen_slow);
    const uint16_t narrow_slow =
        run(EventPath::synchronous, EventEdge::rising, gen_slow);
    const uint16_t narrow_half =
        run(EventPath::resynchronized, EventEdge::rising, gen_xtal);
    print(serial, "  narrow 21 ns pulse: ASYNCHRONOUS on OSCULP32K ",
          narrow_async, " of ", dma_results,
          "; SYNCHRONOUS on the pacer's own 48 MHz ", sync_beats,
          "; SYNCHRONOUS on OSCULP32K ", narrow_slow,
          "; RESYNCHRONIZED on the 24 MHz crystal ", narrow_half, crlf);
    print(serial, "  wide 10.4 us pulse: SYNCHRONOUS on the pacer's own "
                  "96 kHz generator ", wide_slow_sync, " of ", dma_results,
          "; RESYNCHRONIZED on OSC48M ", resync_beats, crlf);
    bench.verdict("THE ASYNCHRONOUS PATH DOES NOT CARE ABOUT ANY OF THIS - it "
                  "has no clock to sample with, so a 21 ns pulse reaches a "
                  "channel clocked at 32 kHz and every event lands",
                  narrow_async == dma_results);
    bench.verdict("A SAMPLED PATH DOES: the same narrow event on a channel "
                  "clocked SLOWER than the generator is mostly not there when "
                  "the channel looks, and 24 MHz against 48 is no better than "
                  "32 kHz",
                  narrow_slow < dma_results / 2u && narrow_half < dma_results / 2u);
    bench.verdict("AND WIDENING THE PULSE IS NOT THE WHOLE ANSWER EITHER: a "
                  "synchronous channel on a 96 kHz generator loses a pulse as "
                  "wide as its OWN PERIOD, so 'synchronous' is not a licence "
                  "to clock the channel slowly - the channel clock is "
                  "ON-DEMAND, which erratum 1.12.1 leaves no alternative to, "
                  "and starting it costs about a period",
                  wide_slow_sync < dma_results / 2u);
    bench.verdict("the channel goes back to a clock the chain can use",
                  GclkChannel::connect(Evsys::gclk_id(ev_start_channel),
                                       gen_slow));

    // The control: with the pacer stopped nothing moves.
    for (uint16_t i = 0; i < dma_results; ++i) {
        results[i] = 0xFFFFFFFFul;
    }
    (void)Tsens::value();
    Tsens::clear_flags(Tsens::flag_all);
    (void)Copy::reset();
    (void)Copy::configure(DmaChannelConfig{.trigger = Tsens::dma_trigger_resrdy,
                                           .action = DmaTriggerAction::beat});
    (void)Copy::load(DmaTransfer{.source = &Tsens::regs().TSENS_VALUE,
                                 .destination = &results[0],
                                 .beats = dma_results,
                                 .beat = DmaBeat::word,
                                 .source_increment = false});
    (void)Copy::enable(true);
    (void)Tsens::enable(true);
    wait_ms(60);
    (void)Tsens::enable(false);
    (void)Copy::enable(false);
    uint16_t idle_beats = 0;
    for (uint16_t i = 0; i < dma_results; ++i) {
        if (results[i] != 0xFFFFFFFFul) {
            ++idle_beats;
        }
    }
    print(serial, "  with the pacer stopped: ", idle_beats, " beats", crlf);
    bench.verdict("WITH NOTHING TRIGGERING, NOTHING MOVES - the chain is the "
                  "event's and not the enable's",
                  idle_beats == 0u);

    // THE OTHER DIRECTION: the window monitor as an event GENERATOR,
    // counted by TC3. The window is placed to match every measurement, so
    // one event per measurement is what should arrive.
    {
        // AGAIN AT THE CHAIN'S OWN GAIN: `reference` above is what this
        // block reads at this GAIN, and placing a threshold from a
        // factory-GAIN reading instead is exactly the mistake that made
        // the first version of this control fire when it should not have.
        const bool here = chain_ref.has_value();
        bench.verdict("the current reading at the chain's GAIN is in hand",
                      here);
        if (here) {
            TsensConfig cfg = chain_cfg;
            cfg.free_running = true;
            cfg.window = TsensWindow::above;
            cfg.window_lower = reference - 3000;
            cfg.events.window_out = true;
            bench.verdict("a free-running block with the window monitor's "
                          "output event enabled comes up",
                          tsens_up(gen_sys, cfg));
            bench.verdict("TSENS WINMON reaches TC3's event input",
                          Evsys::connect(Counter::event_user, ev_window_channel,
                                         EventChannelConfig{
                                             .generator = Tsens::window_generator,
                                             .path = EventPath::asynchronous}));
            (void)Counter::enable(true);
            (void)Counter::set_count16(0);
            wait_ms(50);
            const uint32_t counted = Counter::count16();
            (void)Counter::enable(false);
            print(serial, "  50 ms of free-running measurement produced ",
                  counted, " window events on TC3", crlf);
            bench.verdict("THE WINDOW MONITOR IS A GENERATOR TOO - its match "
                          "left the peripheral as an event and was counted by "
                          "a timer that never saw a temperature",
                          counted > 0u);

            // And the control: a window that cannot match generates
            // nothing at all.
            TsensConfig miss = cfg;
            miss.window_lower = reference + 3000;
            bench.verdict("the same chain with an unreachable threshold comes "
                          "up",
                          tsens_up(gen_sys, miss) &&
                              Evsys::connect(Counter::event_user, ev_window_channel,
                                             EventChannelConfig{
                                                 .generator = Tsens::window_generator,
                                                 .path = EventPath::asynchronous}));
            (void)Counter::enable(true);
            (void)Counter::set_count16(0);
            wait_ms(50);
            const uint32_t missed = Counter::count16();
            (void)Counter::enable(false);
            print(serial, "  the same 50 ms with the window out of reach: ",
                  missed, " events", crlf);
            bench.verdict("and a window that cannot match generates nothing",
                          missed == 0u);
        }
    }

    Tsens::release();
    Evsys::disconnect(Counter::event_user);
    (void)Copy::reset();
    (void)Pacer::enable(false);
    Pacer::release();
    Counter::release();
    // The event channels back onto a generator that will still be running
    // when the borrowed clocks go home, BEFORE they go home: 16.6.2.6
    // makes a generator on a stopped source unroutable until reset.
    (void)GclkChannel::connect(Evsys::gclk_id(ev_start_channel), gen_slow);
    clocks_down();
}

// =============================================================================
// i - the four interrupt sources through the one vector
// =============================================================================
//
// All four flags are ORed into one NVIC line (43.6.4), so the handler has
// to read INTFLAG to know what happened - and the driver's isr() body is
// what an app binds. The DAC's and the SDADC's suites both read their
// flags and never bound their vectors, so this closes the same gap their
// pages carry; the ADC's did bind one, through util/analog_sampler.hpp.
void ti_interrupts() {
    TsensConfig cfg = base_cfg();
    cfg.calibration.gain = factory.gain / 4u;
    bench.verdict("the sensor comes up", tsens_up(gen_sys, cfg));

    tsens_irqs = 0;
    tsens_last_mask = 0;
    Tsens::clear_flags(Tsens::flag_all);
    Tsens::arm(Tsens::flag_result_ready);
    bench.verdict("RESRDY arms", (Tsens::armed() & Tsens::flag_result_ready) != 0u);
    Nvic::enable(Tsens::irq());
    Tsens::start();
    wait_ms(10);
    Nvic::disable(Tsens::irq());
    Tsens::disarm(Tsens::flag_all);
    print(serial, "  one started measurement produced ", tsens_irqs,
          " interrupts, last mask ", hex(tsens_last_mask), ", value ",
          tsens_last_value, " centi-C", crlf);
    bench.verdict("THE VECTOR FIRES, and the ONE vector carries all four "
                  "sources - the handler has to read INTFLAG to know which",
                  tsens_irqs >= 1u);
    bench.verdict("and the handler saw RESRDY in the mask isr() returned",
                  (tsens_last_mask & Tsens::flag_result_ready) != 0u);
    bench.verdict("the value the handler read is a plausible temperature",
                  tsens_last_value > 0 && tsens_last_value < 10000);
    bench.verdict("reading VALUE in the handler cleared RESRDY, so the "
                  "interrupt did not re-enter",
                  tsens_irqs == 1u);

    // OVERRUN through the same vector, with RESRDY disarmed so the only
    // thing that can wake the handler is the overrun itself.
    TsensConfig fr = cfg;
    fr.free_running = true;
    bench.verdict("a free-running block comes up", tsens_up(gen_sys, fr));
    tsens_irqs = 0;
    tsens_last_mask = 0;
    Tsens::clear_flags(Tsens::flag_all);
    Tsens::arm(Tsens::flag_overrun);
    Nvic::enable(Tsens::irq());
    wait_ms(10);
    Nvic::disable(Tsens::irq());
    Tsens::disarm(Tsens::flag_all);
    print(serial, "  ten milliseconds of unread free-running results: ",
          tsens_irqs, " overrun interrupts, last mask ", hex(tsens_last_mask),
          crlf);
    bench.verdict("OVERRUN reaches the same vector", tsens_irqs >= 1u);
    bench.verdict("and isr() acknowledged it - it is sticky, and nothing else "
                  "would ever clear it",
                  (tsens_last_mask & Tsens::flag_overrun) != 0u);

    // A disarmed flag sets but does not interrupt.
    bench.verdict("the block comes back to single measurements",
                  tsens_up(gen_sys, cfg));
    tsens_irqs = 0;
    Tsens::clear_flags(Tsens::flag_all);
    Nvic::enable(Tsens::irq());
    (void)Tsens::measure();
    wait_ms(5);
    Nvic::disable(Tsens::irq());
    bench.verdict("a flag with nothing armed sets without interrupting",
                  tsens_irqs == 0u);

    Tsens::release();
}

// =============================================================================
// j - drift under load, printed and judged only if it clears the noise
// =============================================================================
//
// The one thing a temperature sensor with no thermometer CAN be asked
// about the temperature: does the number move when the die is made to
// work? The baseline is taken first, then the CPU is held in a tight loop
// with the DMAC churning memory for forty seconds, then the same batch is
// taken again. NOTHING IS CLAIMED unless the shift clears the measured
// noise - and a bench in a room with air moving in it is entitled to
// drift either way.
void tj_drift() {
    bench.verdict("the sensor comes up fully calibrated", tsens_up(gen_sys, base_cfg()));

    const Batch before = take(64);
    bench.verdict("the baseline is taken", before.ok);
    print_batch("before the load", before);

    // Forty seconds of work: the CPU spinning and the DMAC copying, which
    // is as much of this die as a wireless suite can switch on.
    static volatile uint32_t churn_src[64];
    static volatile uint32_t churn_dst[64];
    for (uint16_t i = 0; i < 64; ++i) {
        churn_src[i] = i * 0x01010101ul;
    }
    (void)Copy::reset();
    (void)Copy::configure(DmaChannelConfig{.trigger = 0,
                                           .action = DmaTriggerAction::block});

    print(serial, "  loading the die for 60 seconds (CPU spinning, DMAC "
                  "copying, TSENS measuring without pause), sampling the "
                  "temperature every ten; nothing is printed until it ends",
          crlf);
    constexpr uint8_t trend_points = 6;
    int32_t trend[trend_points];
    uint8_t trend_n = 0;
    const uint32_t t0 = Ticker::millis();
    uint32_t blocks = 0;
    uint32_t next_sample = 10'000UL;
    while (Ticker::millis() - t0 < 60'000UL) {
        (void)Copy::load(DmaTransfer{.source = &churn_src[0],
                                     .destination = &churn_dst[0],
                                     .beats = 64,
                                     .beat = DmaBeat::word});
        (void)Copy::enable(true);
        Copy::trigger();
        spin(2000);
        (void)Copy::enable(false);
        ++blocks;
        if (Ticker::millis() - t0 >= next_sample && trend_n < trend_points) {
            const auto v = Tsens::measure_average(10);
            trend[trend_n] = v ? *v : 0;
            ++trend_n;
            next_sample += 10'000UL;
        }
    }
    (void)Copy::reset();

    const Batch after = take(64);
    bench.verdict("the second batch is taken", after.ok);
    print_batch("after the load", after);

    print(serial, "  the trend across the minute, ten seconds apart: ");
    for (uint8_t i = 0; i < trend_n; ++i) {
        print(serial, trend[i], i + 1u == trend_n ? "" : ", ");
    }
    print(serial, " centi-C", crlf);
    bench.verdict("the whole minute was sampled", trend_n == trend_points);
    if (trend_n == trend_points) {
        int32_t lo = trend[0];
        int32_t hi = trend[0];
        for (uint8_t i = 1; i < trend_n; ++i) {
            if (trend[i] < lo) { lo = trend[i]; }
            if (trend[i] > hi) { hi = trend[i]; }
        }
        const int32_t slope = trend[trend_n - 1] - trend[0];
        print(serial, "  first to last ", slope, " centi-C, whole-minute range ",
              hi - lo, ", against the baseline batch's own spread ",
              before.spread(), crlf);
        if (abs_signed(slope) > before.spread()) {
            print(serial, "  the trend CLEARS the reading's own spread: the "
                          "die is measurably ",
                  slope > 0 ? "warming" : "cooling", " under the load", crlf);
        } else {
            print(serial, "  DECLINED: the trend is inside the reading's own "
                          "spread, so nothing is claimed about self-heating",
                  crlf);
        }
        bench.verdict("A MINUTE OF CONTINUOUS CONVERSION UNDER LOAD DOES NOT "
                      "WALK THE READING AWAY - whatever the trend is, it stays "
                      "inside a couple of degrees, which is the only claim "
                      "this bench is entitled to make about it",
                      hi - lo < 200);
    }

    if (before.ok && after.ok) {
        const int32_t delta = after.mean - before.mean;
        const int32_t noise = before.spread() + after.spread();
        print(serial, "  ", blocks, " DMA blocks in the minute; the reading "
                      "moved ", delta, " centi-C (");
        print_celsius(delta);
        print(serial, " C) against a combined spread of ", noise, crlf);
        if (abs_signed(delta) > noise) {
            print(serial, "  the shift CLEARS the noise: the die is measurably ",
                  delta > 0 ? "warmer" : "cooler", " after the load", crlf);
            bench.verdict("the sensor FOLLOWS the die's own working "
                          "temperature - a shift larger than the reading's own "
                          "spread",
                          true);
        } else {
            print(serial, "  DECLINED: the shift is inside the reading's own "
                          "spread, so this bench cannot tell self-heating from "
                          "noise and claims nothing", crlf);
            bench.verdict("the drift is reported and NOT judged - which is the "
                          "honest answer when the effect is smaller than the "
                          "instrument",
                          true);
        }
        // What IS claimable either way: the sensor did not wander off.
        bench.verdict("the reading stayed a plausible temperature throughout "
                      "the load",
                      after.mean > 0 && after.mean < 10000);
        bench.verdict("and the noise did not grow by an order of magnitude "
                      "under a busy bus",
                      after.spread() <= before.spread() * 10 + 50);
    }

    Tsens::release();
}

// =============================================================================
// p - erratum 1.19.1, the PAC write protection (BY NAME ONLY)
// =============================================================================
//
// 43.5.8 says CTRLB and INTFLAG are the two registers PAC write
// protection does NOT cover. ERRATUM 1.19.1, live on every silicon
// revision including this one, says the opposite for CTRLB: "when PAC
// Write-Protection is enabled for TSENS, writes to TSENS.CTRLB are not
// functional". One of the two documents is wrong about this silicon.
//
// THIS LETTER IS OUTSIDE z, AND DELIBERATELY. 11.5.2.4 says a protected
// write returns an access error, and an access error on a Cortex-M0+ may
// be a HardFault - which on this board is a reset (the suite binds
// hard_fault_reset). A reset in the middle of z would destroy the run, so
// the probe is asked for by name, leaves a breadcrumb in .noinit before
// it starts, and the banner reports the answer at the next boot if the
// board did in fact come back the hard way.
//
// There is no PAC driver in this stratum, so the two PAC registers are
// written raw. That is a failure injection and not a configuration - the
// precedent is test_samc_clock's letter c, which clears XTALEN by hand.
void tp_pac_protection() {
    bench.verdict("the sensor comes up fully calibrated", tsens_up(gen_sys, base_cfg()));
    const auto before = Tsens::measure();
    bench.verdict("and measures with PAC protection off, as it is out of "
                  "reset (11.5.2.2)",
                  before.has_value());

    print(serial, "  arming the breadcrumb: if the next line does not appear, "
                  "the protected write FAULTED and the board reset", crlf);
    pac_token.magic = pac_magic;
    pac_token.armed = 1;

    // PAC.WRCTRL: peripheral id 12, KEY = SET. Every PAC flag is cleared
    // first, so anything standing afterwards was raised by THIS write.
    PAC_REGS->PAC_INTFLAGA = PAC_REGS->PAC_INTFLAGA;
    PAC_REGS->PAC_INTFLAGB = PAC_REGS->PAC_INTFLAGB;
    PAC_REGS->PAC_WRCTRL = PAC_WRCTRL_PERID(Tsens::pac_id) | PAC_WRCTRL_KEY_SET;
    const uint32_t statusa = PAC_REGS->PAC_STATUSA;
    const uint32_t statusb = PAC_REGS->PAC_STATUSB;
    const bool protection_on = (statusa & (1ul << 12)) != 0u;
    Tsens::clear_flags(Tsens::flag_all);
    Tsens::start();
    uint32_t spins = 500'000UL;
    while (!Tsens::result_ready() && spins-- != 0u) {
    }
    const bool started = Tsens::result_ready();
    const uint32_t intflaga = PAC_REGS->PAC_INTFLAGA;
    const uint32_t intflagb = PAC_REGS->PAC_INTFLAGB;

    PAC_REGS->PAC_WRCTRL = PAC_WRCTRL_PERID(Tsens::pac_id) | PAC_WRCTRL_KEY_CLR;
    PAC_REGS->PAC_INTFLAGA = intflaga;
    PAC_REGS->PAC_INTFLAGB = intflagb;
    pac_token.armed = 0;

    print(serial, "  THE BOARD SURVIVED the protected write. PAC STATUSA=",
          hex(statusa), " STATUSB=", hex(statusb), " (TSENS is bridge A bit 12)",
          crlf);
    print(serial, "  protection stood: ", yes_no(protection_on),
          "; the START took effect: ", yes_no(started),
          "; PAC INTFLAGA=", hex(intflaga), " INTFLAGB=", hex(intflagb), crlf);
    bench.verdict("PAC write protection for the TSENS can be set at all, and "
                  "STATUSA reports it on bridge A bit 12",
                  protection_on);
    bench.verdict("A PAC-PROTECTED WRITE DOES NOT FAULT THIS CORE. 11.5.2.4's "
                  "'peripheral returns an access error' is not a bus error "
                  "here: the CPU walks straight past it, which is why this "
                  "letter's breadcrumb was never needed",
                  true);
    bench.verdict("ERRATUM 1.19.1 IS REPRODUCED: with PAC write protection set "
                  "for the TSENS the START does nothing, so 43.5.8's list of "
                  "the two registers protection 'does not apply to' is wrong "
                  "about CTRLB on this silicon",
                  !started);
    bench.verdict("AND IT IS WORSE THAN THE ERRATUM SAYS: no PAC interrupt "
                  "flag is raised either, so 11.5.2.4's 'the corresponding "
                  "interrupt flag bit will be set' does not happen and the "
                  "write is dropped in COMPLETE SILENCE - nothing a program "
                  "can read tells it the measurement never started",
                  (intflaga & (1ul << 12)) == 0u);

    bench.verdict("with the protection cleared the block measures again",
                  Tsens::measure().has_value());
    bench.verdict("and free-running mode - the errata's other workaround - "
                  "needs no CTRLB write at all",
                  Tsens::enable(false) && Tsens::free_running(true) &&
                      Tsens::enable(true) && Tsens::read().has_value());

    Tsens::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_tsens - SAMC21J18A temperature sensor (ch. 43): a CLOCK "
          "RATIO, not an ADC channel; clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }
extern "C" void HardFault_Handler() { brio::hard_fault_reset<brio::SamPlatform>(); }

extern "C" void TSENS_Handler() {
    const uint8_t mask = brio::Tsens::isr();
    tsens_last_mask = mask;
    if ((mask & brio::Tsens::flag_result_ready) != 0u) {
        // Reading VALUE is what clears RESRDY (43.8.7), so the handler
        // that wants the number is also the handler that acknowledges.
        tsens_last_value = brio::Tsens::value();
    }
    tsens_irqs = tsens_irqs + 1u;
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
    const bool dma_ok = brio::Dmac::init();
    brio::Nvic::enable(brio::Dmac::irq());
    Led::output();
    brio::enable_interrupts();

    factory = TsensCalibration::factory();

    bench.letter('a', "the block, the registers and the refusals", ta_block);
    bench.letter('b', "the factory calibration, and what it is worth",
                 tb_calibration);
    bench.letter('c', "the reading at rest: jitter first, then a band",
                 tc_at_rest);
    bench.letter('d', "THE SCALE LEVER: one die, two references, one prediction",
                 td_scale_lever);
    bench.letter('e', "what a measurement costs, ruled by the crystal",
                 te_timing);
    bench.letter('f', "how wide the datapath really is", tf_datapath_width);
    bench.letter('g', "the window monitor, all six modes", tg_window);
    bench.letter('h', "the no-CPU chain: event in on every path, DMAC out",
                 th_no_cpu);
    bench.letter('i', "the four interrupt sources through one vector",
                 ti_interrupts);
    bench.letter('j', "drift under load, printed and judged honestly",
                 tj_drift);
    bench.letter('p', "erratum 1.19.1's PAC write protection (may reset)",
                 tp_pac_protection, false);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " dmac=", dma_ok ? "ok" : "FAILED", crlf);
        const auto record = brio::take_panic_record<brio::SamPlatform>();
        if (pac_token.magic == pac_magic && pac_token.armed != 0u) {
            pac_token.armed = 0;
            print(serial, "  LETTER p's BREADCRUMB STANDS: the PAC-protected "
                          "write to TSENS.CTRLB FAULTED and reset the board. "
                          "11.5.2.4's 'access error' is a bus error on this "
                          "core, and erratum 1.19.1 understates it.", crlf);
        } else if (record) {
            print(serial, "  a panic record from the previous boot: code ",
                  static_cast<uint32_t>(record->code), " context ",
                  record->context, crlf);
        }
        print(serial, "  factory calibration: GAIN ", factory.gain, " OFFSET ",
              factory.offset, " TCAL ", factory.tcal, " FCAL ", factory.fcal,
              crlf);
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
