// test_samc_analog_dma - the reference bench suite for the two STREAMING
// engines of samc/dmac.hpp (DmaLoopEngine, DmaPingPongEngine) and for
// the element-type generalization of all four.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. On this package PA02 is the DAC's VOUT pad and ADC0's
// AIN0 at once, so a table played out of RAM by one DMA channel and
// captured back into RAM by another crosses a wire of zero length. The
// SDADC's letter uses its own pair 0 pads (PA06/PA07) driven by PORT,
// and TSENS has no pads at all.
//
// WHAT THIS SUITE IS ACTUALLY ABOUT, said before any letter: the two new
// engines are not "DMA for the analog peripherals", they are TWO SHAPES
// a stream can have, proved on the only chapters in this stratum that
// can generate and consume one without a signal generator on the desk.
//
//   DmaLoopEngine       one table, played for ever, re-armed from the
//                       TCMPL interrupt because THIS CONTROLLER HAS NO
//                       HARDWARE CIRCULAR MODE (25.6.3.1 offers only a
//                       self-linked descriptor, and linked descriptors
//                       are deliberately not built here - erratum 1.10.4
//                       corrupts the write-back that 25.6.2.6 makes the
//                       LIVE descriptor, so a self-linked chain has no
//                       second copy to judge the first against).
//   DmaPingPongEngine   two buffers, the engine filling one while the
//                       caller drains the other, and THE ACCOUNTING IS
//                       THE API: laps, overruns, stalls. A stream whose
//                       drainer falls behind cannot be made correct, so
//                       the only thing worth building is one that says
//                       so exactly.
//
// THE CHAIN letters c..f run, with no CPU in the sample path at all:
//
//   TC0 overflow --(EVSYS async)--> DAC START  -> DATABUF into DATA
//   DAC EMPTY    --(DMA trigger)--> ch0 loop   -> next table entry
//   TC0 MC0      --(EVSYS async)--> ADC0 START -> one conversion
//   ADC0 RESRDY  --(DMA trigger)--> ch1 stream -> RESULT into a buffer
//
// One TC period is one sample of both converters, and the ADC's start
// sits at three quarters of the period so the DAC has long settled. That
// makes the captured stream the DAC's own table, rotated by a CONSTANT,
// and the constant is what letter c measures: an offset that follows its
// own arithmetic block after block is a proof that NOT ONE SAMPLE is
// lost at any lap or block boundary, which is the thing a re-armed
// software loop has to earn.
//
// What is exercised, letter by letter:
//   a  the shapes: element types, beats, the codes the four analog
//      peripherals publish, and what the engines refuse
//   b  THE NOISE FLOOR FIRST: the static DAC-to-ADC calibration table
//      and its spread, which is where the bands of letter c come from
//   c  THE ROUND TRIP: the table played and captured, every sample
//      inside the measured band, and the phase arithmetic
//   d  the rate against the wall clock, and the two engines' counts
//      against each other
//   e  the accounting under a deliberately slow drainer
//   f  erratum 1.10.4: the same chain under concurrent m2m churn
//   g  A TRIGGER IS AN EDGE: a bare channel armed on a standing request
//      moves nothing, and kick() is what rescues it
//   h  a dead block abandoned, counted, and the stream resumed
//   i  the SDADC through the ping-pong engine, on WORD beats
//   j  TSENS through the same engine, on WORD beats
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
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "samc/platform_sam.hpp"
#include "samc/tsens.hpp"
#include "kernel/kernel.hpp"
#include "util/block_stream.hpp"
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
// The pads and the peripherals
// ---------------------------------------------------------------------------
using Vout = Pin<'A', 2>;    // DAC/VOUT = ADC0/AIN0 - the zero-length wire
using Adc0 = Adc<0>;
using Pacer = Tc<0>;

/// SDADC differential pair 0: PA06 is INN0 and PA07 is INP0 on this
/// package, and letter i drives both with PORT.
using SdN = Pin<'A', 6>;
using SdP = Pin<'A', 7>;

constexpr uint8_t main_gen = 0;                    ///< GCLK0 = OSC48M
constexpr uint32_t main_gen_hz = SysClock::hz;
constexpr uint8_t ev_gen = 6;                      ///< the EVSYS channels' clock

constexpr uint8_t ev_dac_start = 0;   // TC0 overflow -> DAC START
constexpr uint8_t ev_adc_start = 1;   // TC0 MC0      -> ADC0 START

// ---------------------------------------------------------------------------
// The DMA channels, one engine each
// ---------------------------------------------------------------------------
constexpr uint8_t ch_dac = 0;
constexpr uint8_t ch_adc = 1;
constexpr uint8_t ch_wide = 2;    // the SDADC's and TSENS's word stream
constexpr uint8_t ch_bare = 3;    // letter g's control: a channel with no engine
constexpr uint8_t ch_churn0 = 4;  // letter f's background traffic
constexpr uint8_t ch_churn1 = 5;

using DacLoop = DmaLoopEngine<ch_dac, uint16_t>;
using AdcStream = DmaPingPongEngine<ch_adc, uint16_t>;
using WideStream = DmaPingPongEngine<ch_wide, uint32_t>;
using Bare = DmaChannel<ch_bare>;
using Churn0 = DmaChannel<ch_churn0>;
using Churn1 = DmaChannel<ch_churn1>;

// ---------------------------------------------------------------------------
// The buffers. VOLATILE IN BOTH DIRECTIONS - the DMAC campaign's lesson
// on this target, where gcc sank a zeroing store past a transfer. The
// engines take volatile pointers precisely so this needs no cast.
// ---------------------------------------------------------------------------
constexpr uint16_t table_len = 32;    ///< the DAC's lap
constexpr uint16_t block_len = 24;    ///< the ADC's block

/// 24 and 32 are NOT in a divisor relationship on purpose: a lap
/// boundary and a block boundary therefore walk past each other, and the
/// table phase of a block's first sample advances by 24 (mod 32) every
/// block. A stream that lost one sample anywhere would break that
/// arithmetic instead of hiding inside a coincidence.
static_assert(block_len % table_len != 0u && table_len % block_len != 0u,
              "the block and the lap must not divide each other, or a lost "
              "sample at a boundary would be invisible");

volatile uint16_t wave[table_len];
volatile uint16_t capture_a[block_len];
volatile uint16_t capture_b[block_len];

/// Letter b's calibration: what the ADC reads for each table entry with
/// the DAC held still, and the widest spread any of them showed.
uint16_t expected[table_len];
uint16_t calib_spread = 0;
bool calibrated = false;

/// What INTFLAG.EMPTY read at the moment the loop engine was armed -
/// recorded rather than acted on, because the answer is a finding (see
/// chain_up()) and letter c prints it.
bool dac_empty_at_arm = false;

/// The SDADC's and TSENS's stream. Four words is one and a half of the
/// SDADC's own conversion times at OSR 256 and well under a second of
/// TSENS at the factory gain.
constexpr uint16_t wide_len = 4;
volatile uint32_t wide_a[wide_len];
volatile uint32_t wide_b[wide_len];

/// Letter f's churn buffers.
volatile uint8_t churn_src[128];
volatile uint8_t churn_dst[128];

/// The churn channels' configuration: no peripheral trigger, one
/// software trigger per whole block.
constexpr DmaChannelConfig churn_config{
    .trigger = dma_trigger_none,
    .action = DmaTriggerAction::block,
};

/**
 * One turn of letter f's background traffic, WITH ITS OWN RECOVERY.
 *
 * A memory-to-memory block of 64 bytes takes a couple of microseconds,
 * so a channel still enabled after a generous wait has not finished
 * late - it has STOPPED, which is erratum 1.10.4's destructive form
 * (25.6.2.6: the write-back IS the live descriptor of an ongoing block,
 * so a corrupted one leaves the channel running someone else's transfer
 * for ever, enabled, with no flag anywhere). The bounded wait plus the
 * reset is the same owner-decides-the-block-is-dead doctrine the engines
 * carry, spelled out here for a bare channel.
 *
 * Returns 1 when a block completed; `deaths` counts the ones that did
 * not and had to be reset.
 */
template <typename Ch>
uint32_t churn_turn(uint8_t offset, uint32_t& deaths) {
    if (!Ch::enabled()) {
        (void)Ch::load(DmaTransfer{.source = &churn_src[offset],
                                   .destination = &churn_dst[offset],
                                   .beats = 64,
                                   .beat = DmaBeat::byte});
        Ch::enable(true);
    }
    Ch::trigger();
    uint32_t spins = 20'000u;
    while (spins-- != 0u && Ch::enabled()) {
    }
    if (Ch::enabled()) {
        ++deaths;
        (void)Ch::reset();
        (void)Ch::configure(churn_config);
        return 0;
    }
    return 1;
}

/**
 * The same channel, sprayed rather than paced: re-arm when the last
 * block ended, trigger, and DO NOT WAIT.
 *
 * THIS IS THE ONE THAT REACHES THE ERRATUM, and the difference is the
 * whole point: 1.10.4's precondition is "several channels triggered
 * CONCURRENTLY", and a churn that waits for each of its own blocks is
 * barely concurrent with anything. Measured, the two shapes are worlds
 * apart - the waiting one ran 43000 blocks against a live analog chain
 * with not one refused reading, while spraying reached 682 refusals in
 * 21100 harvests.
 *
 * `stuck` counts consecutive turns that found the channel still
 * enabled. A 64-byte memory-to-memory block is a couple of
 * microseconds, so a channel that has been enabled for thousands of
 * turns has stopped, not slowed - the same predicate churn_turn() uses,
 * measured in turns instead of spins because nothing here waits.
 */
template <typename Ch>
void churn_spray(uint8_t offset, uint32_t& blocks, uint32_t& deaths,
                 uint32_t& stuck) {
    if (!Ch::enabled()) {
        (void)Ch::load(DmaTransfer{.source = &churn_src[offset],
                                   .destination = &churn_dst[offset],
                                   .beats = 64,
                                   .beat = DmaBeat::byte});
        Ch::enable(true);
        ++blocks;
        stuck = 0;
    } else if (++stuck > 5'000u) {
        ++deaths;
        (void)Ch::reset();
        (void)Ch::configure(churn_config);
        stuck = 0;
    }
    Ch::trigger();
}

// ---------------------------------------------------------------------------
// The pacer. TC0 in 8-bit mode, where PER is a real register and both
// compare channels stay free: the overflow starts the DAC and the CC0
// match starts the ADC three quarters of a period later.
//
// div64 of the 48 MHz main clock is 750 kHz, so PER = 149 makes a period
// of 150 ticks = 200 us and the pair runs at 5 kHz. THE RULER IS OSC48M,
// which the clock campaign measured 5100 ppm slow against the board's
// crystal, so every absolute rate this suite prints is on that scale and
// letter d says so where it matters.
// ---------------------------------------------------------------------------
constexpr uint8_t pacer_period = 149;      ///< PER: 150 ticks
constexpr uint8_t pacer_match = 112;       ///< CC0: three quarters of it
constexpr uint32_t pacer_hz = main_gen_hz / 64u / (pacer_period + 1u);   // 5000

constexpr TcConfig pacer_cfg{
    .mode = TcMode::count8,
    .prescaler = TcPrescaler::div64,
    .waveform = TcWaveform::normal_pwm,
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
const char* yes_no(bool v) { return v ? "yes" : "no"; }

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

/// Long enough for the DAC's conversion (2.9 us at its 350 ksps ceiling)
/// and for the ADC's input to follow: about 200 us.
void settle() { spin(2'000UL); }

uint16_t abs_diff(uint16_t a, uint16_t b) {
    return a > b ? static_cast<uint16_t>(a - b) : static_cast<uint16_t>(b - a);
}

/// The table's own code for entry i - a 32-step ramp that stays inside
/// table 45-30's linear range (0.05 V .. VDDANA - 0.05 V is roughly code
/// 10 .. 1013 at this board's supply).
constexpr uint16_t table_code(uint16_t i) {
    return static_cast<uint16_t>(40u + i * 30u);   // 40 .. 970
}

void fill_table() {
    for (uint16_t i = 0; i < table_len; ++i) {
        // dac_data_word() rather than a bare code: where the ten bits sit
        // in DATABUF is table 41-1's business and not this suite's.
        wave[i] = dac_data_word(table_code(i), false, false);
    }
}

// ---------------------------------------------------------------------------
// Bringing the peripherals up
// ---------------------------------------------------------------------------
bool dac_up(bool with_start_event) {
    DacConfig cfg{};
    cfg.reference = DacRef::vddana;
    cfg.external_output = true;    ///< the buffer drives PA02
    if (!Dac::init(main_gen, cfg)) {
        return false;
    }
    if (!with_start_event) {
        return true;
    }
    // EVCTRL is enable-protected and table 29-3 marks the START user
    // asynchronous-path-only, both of which start_on() enforces.
    return Dac::enable(false) &&
           Dac::start_on(ev_dac_start,
                         EventChannelConfig{.generator = Pacer::overflow_generator,
                                            .path = EventPath::asynchronous}) &&
           Dac::enable(true) && Dac::wait_ready();
}

bool adc_up(bool with_start_event) {
    constexpr AdcConfig cfg{
        .reference = Ref::vddana,
        .prescaler = AdcPresc::div32,
        .resolution = AdcRes::bits12,
    };
    if (!Adc0::init<cfg>(main_gen, main_gen_hz)) {
        return false;
    }
    Adc0::select(AnalogIn<Vout>{});
    if (!with_start_event) {
        return true;
    }
    // Erratum 1.4.4 as code: an ADC event user takes the asynchronous
    // path or nothing, and EVCTRL is enable-protected.
    return Adc0::enable(false) &&
           Adc0::start_on(ev_adc_start,
                          EventChannelConfig{.generator = Pacer::match_generator(0),
                                             .path = EventPath::asynchronous}) &&
           Adc0::enable(true);
}

bool pacer_up() {
    if (!Pacer::init(main_gen) || !Pacer::configure(pacer_cfg)) {
        return false;
    }
    if (!Pacer::set_period8(pacer_period) || !Pacer::set_cc8(0, pacer_match)) {
        return false;
    }
    // Both event outputs: OVFEO for the DAC, MCEO0 for the ADC.
    return Pacer::event_config(pacer_cfg,
                               TcEventConfig{.overflow_out = true, .match_out = 0x1});
}

bool fabric_up() {
    Evsys::bus_clock(true);
    Evsys::reset();
    return Gclk<ev_gen>::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
           GclkChannel::connect(Evsys::gclk_id(ev_dac_start), ev_gen) &&
           GclkChannel::connect(Evsys::gclk_id(ev_adc_start), ev_gen);
}

/// Stop everything letters c..f start, in the order that leaves nothing
/// triggering into a channel that is being taken apart.
void chain_down() {
    (void)Pacer::enable(false);
    DacLoop::stop();
    AdcStream::stop();
    Pacer::release();
    Adc0::release();
    Dac::release();
    Evsys::release_channel(ev_dac_start);
    Evsys::release_channel(ev_adc_start);
}

/**
 * Bring the whole chain up and set both streams going.
 *
 * THE TWO SIDES NEED OPPOSITE TREATMENT AT THE FIRST ARM, and the bench
 * taught it rather than the chapter: kick() is right where the standing
 * request is one the stream WANTS SERVED, and wrong where it is one the
 * stream should never have seen.
 *
 *  - THE DAC IS KICKED. Its request is "DATABUF is empty", which is
 *    exactly the state a fresh converter is in, and serving it is what
 *    puts the table's first entry where the first START event will find
 *    it. Without it the first event underruns.
 *  - THE ADC IS DRAINED INSTEAD. Its request is "a result is waiting",
 *    and the result waiting at this moment is one of the FIVE erratum
 *    1.4.6 warm-up conversions `init()` spends - a real conversion, but
 *    not one of this stream's. Kicking would put it in slot zero and
 *    shift the whole capture by a sample; MEASURED, exactly that, as a
 *    first block whose phase was one out while every block after it
 *    stepped by the right 24. So the owner READS RESULT to take the
 *    request down, and the first rise the channel then sees is the
 *    first paced conversion.
 *
 * The general rule the pair states: kick() only where the standing
 * request belongs to the stream. Where the owner can instead ensure NO
 * request is standing before it arms - which it can whenever the pacing
 * has not started yet - that is the better move, because it needs no
 * judgment about what the pending data is.
 *
 * AND ONE MORE THING THE BENCH TAUGHT, which is about the PACER and not
 * about DMA at all: the two events of a period are not simultaneous and
 * THE ADC'S COMES FIRST. CC0 matches at count 112 and the overflow
 * happens at 149, so in the very first period the converter samples a
 * pad the DAC has not been given anything for yet - one sample of the
 * DAC's pre-table value, at the head of an otherwise perfect capture
 * (measured: a first block whose phase was one out, every block after
 * it stepping by the right 24, and a residual of exactly what code 0
 * reads). The owner's answer is to PRIME DATA with the table's LAST
 * entry before the pacer starts, so that even the first sample belongs
 * to the same endless sequence. It has to happen before the loop engine
 * fills DATABUF: a value standing in DATABUF holds SYNCBUSY.DATA up and
 * 41.6.7 discards a DATA write made under it.
 */
bool chain_up() {
    fill_table();
    for (uint16_t i = 0; i < block_len; ++i) {
        capture_a[i] = 0;
        capture_b[i] = 0;
    }

    if (!fabric_up() || !pacer_up() || !dac_up(true) || !adc_up(true)) {
        return false;
    }

    DacLoop::arm(&Dac::regs().DAC_DATABUF, Dac::dma_trigger_empty);
    AdcStream::arm(&Adc0::regs().ADC_RESULT, Adc0::dma_trigger_resrdy);
    DacLoop::clear_faults();
    AdcStream::clear_faults();
    DmaChannel<ch_dac>::clear_counters();
    DmaChannel<ch_adc>::clear_counters();

    // Prime DATA with the entry that PRECEDES the table's first, so the
    // sample the ADC takes before the DAC's first START event is part
    // of the same endless sequence. Before the loop engine fills
    // DATABUF, for the synchronization reason above.
    if (!Dac::set(table_code(table_len - 1))) {
        return false;
    }
    settle();

    // Take the ADC's standing request down BEFORE the channel is armed:
    // reading RESULT is what clears RESRDY (38.6.4), and the pacer is
    // not running yet, so the next rise is the stream's own first
    // sample.
    if (Adc0::ready()) {
        (void)Adc0::result();
    }
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    Dac::clear_flags(Dac::flag_underrun);

    if (!DacLoop::start(wave, table_len)) {
        return false;
    }
    if (!AdcStream::start(capture_a, capture_b, block_len)) {
        return false;
    }

    // THE KICK IS UNCONDITIONAL, and the flag is only recorded. See the
    // comment above this function: INTFLAG.EMPTY reads ZERO on a DAC
    // that has just been enabled even though DATABUF is empty, so a
    // stream that waited for the flag would never start and would pay
    // an UNDERRUN and a lost period for finding out. What makes the
    // kick right is not the flag but the OWNER'S OWN KNOWLEDGE that it
    // has reset the converter and written DATABUF never.
    dac_empty_at_arm = Dac::empty();
    DacLoop::kick();
    return Pacer::enable(true);
}

// ---------------------------------------------------------------------------
// The calibration pass, shared by letters b and c
// ---------------------------------------------------------------------------
/// Hold each table code still and read the ADC eight times. The MEAN is
/// what letter c compares against and the widest SPREAD is where its
/// band comes from - measured first, chosen second, which is the house
/// rule for every band in this repo.
bool calibrate() {
    if (!dac_up(false) || !adc_up(false)) {
        return false;
    }
    calib_spread = 0;
    for (uint16_t i = 0; i < table_len; ++i) {
        (void)Dac::set(table_code(i));
        settle();
        uint32_t sum = 0;
        uint16_t lo = 0xFFFFu;
        uint16_t hi = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            const uint16_t v = Adc0::read();
            sum += v;
            if (v < lo) {
                lo = v;
            }
            if (v > hi) {
                hi = v;
            }
        }
        expected[i] = static_cast<uint16_t>(sum / 8u);
        const uint16_t spread = static_cast<uint16_t>(hi - lo);
        if (spread > calib_spread) {
            calib_spread = spread;
        }
    }
    calibrated = true;
    Adc0::release();
    Dac::release();
    return true;
}

/// The band letter c judges against: the measured noise, doubled, plus a
/// fixed two counts so a perfectly quiet run still has one. Printed with
/// its derivation, and checked against the table's own step so a verdict
/// that could not fail is refused as a verdict.
uint16_t band() {
    const uint16_t b = static_cast<uint16_t>(2u * calib_spread + 2u);
    return b < 6u ? 6u : b;
}

/// The largest |sample - expected| over one captured block, assuming the
/// block's first sample is table entry `offset`.
uint32_t block_error(const volatile uint16_t* buf, uint16_t offset) {
    uint32_t worst = 0;
    for (uint16_t k = 0; k < block_len; ++k) {
        const uint16_t want = expected[(offset + k) % table_len];
        const uint32_t e = abs_diff(buf[k], want);
        if (e > worst) {
            worst = e;
        }
    }
    return worst;
}

/// Which table entry a captured block starts on: the offset whose worst
/// error is smallest. Also reports the worst offset's error, because an
/// alignment that fits everywhere fits nothing.
uint16_t best_offset(const volatile uint16_t* buf, uint32_t& best_err,
                     uint32_t& worst_err) {
    uint16_t best = 0;
    best_err = 0xFFFFFFFFu;
    worst_err = 0;
    for (uint16_t o = 0; o < table_len; ++o) {
        const uint32_t e = block_error(buf, o);
        if (e < best_err) {
            best_err = e;
            best = o;
        }
        if (e > worst_err) {
            worst_err = e;
        }
    }
    return best;
}

// =============================================================================
// a - the shapes, and what they refuse
// =============================================================================
void ta_shapes() {
    // The element type IS the beat, in one `sizeof` that feeds both
    // BEATSIZE and the end-address arithmetic.
    static_assert(dma_beat_of<uint8_t>() == DmaBeat::byte);
    static_assert(dma_beat_of<uint16_t>() == DmaBeat::hword);
    static_assert(dma_beat_of<uint32_t>() == DmaBeat::word);
    static_assert(DmaTxEngine<0>::beat == DmaBeat::byte);
    static_assert(DacLoop::beat == DmaBeat::hword);
    static_assert(AdcStream::beat == DmaBeat::hword);
    static_assert(WideStream::beat == DmaBeat::word);

    bench.verdict("the element type is the beat, and the default is still a "
                  "byte - every existing engine spelling means what it did",
                  dma_beat_bytes(DmaTxEngine<0>::beat) == 1u &&
                      dma_beat_bytes(DacLoop::beat) == 2u &&
                      dma_beat_bytes(WideStream::beat) == 4u);

    print(serial, "  trigger codes the analog chapters publish: DAC EMPTY ",
          Dac::dma_trigger_empty, ", ADC0 RESRDY ", Adc0::dma_trigger_resrdy,
          ", ADC1 RESRDY ", Adc<1>::dma_trigger_resrdy, ", SDADC RESRDY ",
          Sdadc::dma_trigger_resrdy, ", TSENS RESRDY ", Tsens::dma_trigger_resrdy,
          crlf);
    bench.verdict("each peripheral states its own DMAC trigger id - dmac.hpp "
                  "owns the channels and never table 25-2",
                  Dac::dma_trigger_empty != dma_trigger_none &&
                      Adc0::dma_trigger_resrdy != dma_trigger_none &&
                      Sdadc::dma_trigger_resrdy != dma_trigger_none &&
                      Tsens::dma_trigger_resrdy != dma_trigger_none &&
                      Adc<1>::dma_trigger_resrdy != Adc0::dma_trigger_resrdy);

    // The end-address arithmetic at two widths, which is the one place a
    // wrong beat would run over the wrong memory silently.
    constexpr DmaDescriptor hw =
        dma_descriptor_at(0x2000'0100u, 0x4200'0018u,
                          DmaTransfer{.beats = table_len,
                                      .beat = DmaBeat::hword,
                                      .source_increment = true,
                                      .destination_increment = false});
    static_assert(hw.srcaddr == 0x2000'0100u + 2u * table_len);
    static_assert(hw.dstaddr == 0x4200'0018u);
    constexpr DmaDescriptor wd =
        dma_descriptor_at(0x4200'1000u, 0x2000'0200u,
                          DmaTransfer{.beats = wide_len,
                                      .beat = DmaBeat::word,
                                      .source_increment = false,
                                      .destination_increment = true});
    static_assert(wd.dstaddr == 0x2000'0200u + 4u * wide_len);
    bench.verdict("the end address counts BYTES, so a widened block ends "
                  "where its element size says (25.6.2.7)",
                  hw.srcaddr - 0x2000'0100u == 2u * table_len &&
                      wd.dstaddr - 0x2000'0200u == 4u * wide_len);

    // The engines refuse what has no meaning, before any silicon is
    // touched. A zero length, a null buffer, and - the ping-pong's own
    // rule - two names for one buffer, which would make the swap a lie.
    volatile uint16_t sink = 0;
    DacLoop::arm(&sink, dma_trigger_none);
    bench.verdict("an empty table is refused", !DacLoop::start(wave, 0));
    bench.verdict("a null table is refused", !DacLoop::start(nullptr, table_len));
    DacLoop::stop();

    WideStream::arm(&sink, dma_trigger_none);
    bench.verdict("a ping-pong stream with one buffer named twice is refused - "
                  "the swap would hand the caller the buffer being filled",
                  !WideStream::start(wide_a, wide_a, wide_len));
    bench.verdict("and so is a zero-length block",
                  !WideStream::start(wide_a, wide_b, 0));
    WideStream::stop();

    // The state a fresh engine reports, which is what every accounting
    // verdict below is differenced against.
    bench.verdict("a stopped stream holds nothing and claims nothing",
                  AdcStream::ready() == nullptr && AdcStream::pending() == 0u &&
                      !AdcStream::stalled() && !AdcStream::running());

    print(serial, "  the lap is ", table_len, " halfwords and the block ",
          block_len, "; the pacer is ", pacer_hz,
          " Hz on OSC48M's scale (5100 ppm slow against the crystal)", crlf);
    bench.verdict("the lap and the block do not divide each other, so a lost "
                  "sample cannot hide in a coincidence",
                  block_len % table_len != 0u && table_len % block_len != 0u);
}

// =============================================================================
// b - the noise floor first, then the band
// =============================================================================
//
// EVERY BAND IN LETTER C COMES FROM HERE. The DAC is held still at each
// of the table's 32 codes and the ADC reads it eight times; the mean is
// what a streamed sample is compared against and the widest spread is
// what the comparison is allowed. Measuring the noise BEFORE choosing
// the band is the house rule, and the letter also checks that the band
// it derives is much smaller than one table step - a band as wide as the
// staircase would pass anything.
void tb_noise() {
    bench.verdict("the calibration pass runs: the DAC held at each of the "
                  "table's 32 codes, read back through PA02",
                  calibrate());
    if (!calibrated) {
        return;
    }

    print(serial, "  code -> counts:");
    for (uint16_t i = 0; i < table_len; i += 4) {
        print(serial, " ", table_code(i), "/", expected[i]);
    }
    print(serial, crlf);

    const uint16_t step = abs_diff(expected[1], expected[0]);
    print(serial, "  widest spread over 8 readings: ", calib_spread,
          " counts; one table step measures ", step, " counts; the band is ",
          band(), crlf);

    bench.verdict("the ramp is monotonic in the ADC's counts", [] {
        for (uint16_t i = 1; i < table_len; ++i) {
            if (expected[i] <= expected[i - 1]) {
                return false;
            }
        }
        return true;
    }());
    bench.verdict("THE BAND IS A REAL BAND: it is derived from the measured "
                  "noise and is far narrower than one step of the staircase",
                  step > 4u * band());
    bench.verdict("the whole ramp stays inside the converter's range",
                  expected[0] > 20u && expected[table_len - 1] < 4000u);
    Dac::release();
    Adc0::release();
}

// =============================================================================
// c - THE ROUND TRIP
// =============================================================================
//
// The table is played by one engine and captured by the other with the
// CPU touching neither sample. Two things are then true if and only if
// nothing is lost:
//
//  1. every captured sample sits within the measured band of the table
//     entry it should be;
//  2. the table entry a block STARTS on advances by exactly block_len
//     (mod table_len) from block to block - which is the arithmetic of a
//     stream that drops nothing at a lap boundary, at a block boundary,
//     or anywhere else.
//
// The second is the one a re-armed software loop has to earn, and it is
// why the lap and the block were chosen not to divide each other.
void tc_round_trip() {
    if (!calibrated && !calibrate()) {
        bench.verdict("the calibration pass runs", false);
        return;
    }
    bench.verdict("the chain comes up: pacer, fabric, both converters, both "
                  "engines", chain_up());

    constexpr uint8_t blocks = 12;
    uint16_t offsets[blocks] = {};
    uint32_t errors[blocks] = {};
    uint32_t worst_alt = 0;
    uint8_t taken = 0;

    const uint32_t t0 = Ticker::millis();
    while (taken < blocks && Ticker::millis() - t0 < 500u) {
        const volatile uint16_t* buf = AdcStream::ready();
        if (buf == nullptr) {
            continue;
        }
        uint32_t best_err = 0;
        uint32_t worst_err = 0;
        offsets[taken] = best_offset(buf, best_err, worst_err);
        errors[taken] = best_err;
        if (worst_err > worst_alt) {
            worst_alt = worst_err;
        }
        ++taken;
        (void)AdcStream::release();
    }
    (void)Pacer::enable(false);

    uint32_t worst_fit = 0;
    for (uint8_t i = 0; i < taken; ++i) {
        if (errors[i] > worst_fit) {
            worst_fit = errors[i];
        }
    }

    print(serial, "  captured ", taken, " blocks of ", block_len,
          " samples; worst residual against the calibration ", worst_fit,
          " counts, band ", band(), "; the worst rival alignment is off by ",
          worst_alt, crlf);
    print(serial, "  block start offsets:");
    for (uint8_t i = 0; i < taken; ++i) {
        print(serial, " ", offsets[i]);
    }
    print(serial, crlf);
    print(serial, "  INTFLAG.EMPTY read ", yes_no(dac_empty_at_arm),
          " on the freshly enabled DAC whose DATABUF was empty - the flag is "
          "an EVENT, not a state, and the owner's own knowledge is what makes "
          "the first kick right", crlf);

    bench.verdict("the stream produced blocks at all", taken == blocks);
    bench.verdict("EVERY SAMPLE OF EVERY BLOCK IS THE TABLE ENTRY IT SHOULD "
                  "BE, inside the noise letter b measured",
                  taken == blocks && worst_fit <= band());
    bench.verdict("and the alignment is information rather than a tautology: "
                  "the worst rival offset misses by more than a whole step",
                  worst_alt > 8u * band());

    // THE ARITHMETIC. Block k starts at table entry (o0 + k*block_len)
    // mod table_len, and nothing else is consistent with a stream that
    // loses no sample at any boundary.
    bool arithmetic = taken == blocks;
    for (uint8_t i = 1; i < taken; ++i) {
        const uint16_t want =
            static_cast<uint16_t>((offsets[i - 1] + block_len) % table_len);
        if (offsets[i] != want) {
            arithmetic = false;
        }
    }
    bench.verdict("THE PHASE FOLLOWS ITS OWN ARITHMETIC BLOCK AFTER BLOCK - "
                  "not one sample is lost at a lap boundary, at a block "
                  "boundary or between them",
                  arithmetic);

    print(serial, "  laps: DAC ", DacLoop::laps(), ", ADC ", AdcStream::laps(),
          "; DAC underrun ", yes_no(Dac::underrun()), ", ADC overrun ",
          yes_no((Adc0::flags() & Adc0::flag_overrun) != 0u), "; overruns ",
          AdcStream::overruns(), ", faults ", DacLoop::faults(), "/",
          AdcStream::faults(), crlf);
    bench.verdict("both engines counted laps, and neither converter reported "
                  "a starved or dropped sample",
                  DacLoop::laps() > 0u && AdcStream::laps() >= blocks &&
                      !Dac::underrun() &&
                      (Adc0::flags() & Adc0::flag_overrun) == 0u);
    bench.verdict("no write-back reading was refused on either streaming "
                  "channel",
                  DmaChannel<ch_dac>::violations() == 0u &&
                      DmaChannel<ch_adc>::violations() == 0u);
    chain_down();
}

// =============================================================================
// d - the rate, and the two engines against each other
// =============================================================================
//
// THE RULER IS NAMED: the pacer counts GCLK0 and SysTick counts the CPU
// clock, and on this target both ARE OSC48M - so this letter checks the
// divider arithmetic and the two engines' bookkeeping, NOT the
// oscillator. OSC48M itself is 5100 ppm slow against the board's
// crystal (the clock campaign measured it), and every absolute figure
// printed here carries that.
void td_rate() {
    if (!calibrated && !calibrate()) {
        bench.verdict("the calibration pass runs", false);
        return;
    }
    bench.verdict("the chain comes up", chain_up());

    // Drain diligently for a fixed wall-clock window and count what came
    // through. Draining is the only CPU work in the window.
    const uint32_t window_ms = 400;
    const uint32_t t0 = Ticker::millis();
    uint32_t drained = 0;
    while (Ticker::millis() - t0 < window_ms) {
        if (AdcStream::ready() != nullptr) {
            (void)AdcStream::release();
            ++drained;
        }
    }
    const uint32_t elapsed = Ticker::millis() - t0;
    (void)Pacer::enable(false);

    const uint32_t adc_laps = AdcStream::laps();
    const uint32_t dac_laps = DacLoop::laps();
    const uint32_t adc_samples = adc_laps * block_len;
    const uint32_t dac_samples = dac_laps * table_len;
    const uint32_t rate = elapsed == 0u ? 0u : adc_samples * 1000UL / elapsed;

    print(serial, "  in ", elapsed, " ms: ", adc_laps, " ADC blocks (",
          adc_samples, " samples, ", drained, " drained) and ", dac_laps,
          " DAC laps (", dac_samples, " values); measured ", rate,
          " samples/s against ", pacer_hz, " nominal", crlf);

    // A per-mille band, and it is generous ON PURPOSE: the window is
    // quantized by a 1 kHz tick at both ends, which is 2.5 parts per
    // thousand of a 400 ms window all by itself.
    const uint32_t low = pacer_hz - pacer_hz / 100u;
    const uint32_t high = pacer_hz + pacer_hz / 100u;
    bench.verdict("the sample rate is the pacer's, inside one per cent of a "
                  "window a 1 kHz tick quantizes at both ends",
                  rate > low && rate < high);
    bench.verdict("the two converters are paced by the same timer, so the two "
                  "engines carried the same number of samples to within one "
                  "lap and one block",
                  adc_samples + block_len + table_len > dac_samples &&
                      dac_samples + block_len + table_len > adc_samples);
    bench.verdict("the drainer kept up: every block was handed over and "
                  "released, and nothing stalled",
                  drained + 2u >= adc_laps && AdcStream::overruns() == 0u &&
                      !AdcStream::stalled());
    chain_down();
}

// =============================================================================
// e - the accounting under a deliberately slow drainer
// =============================================================================
//
// The engine's whole contract in one letter. The drainer is made to
// sleep, both buffers fill, and the engine does the one thing that is
// not a lie: it SKIPS THE LAP rather than write into the buffer the
// caller is holding, counts the stall, and waits. What that costs is
// samples, and the samples are NOT this engine's to count - a stalled
// channel moves nothing, so the loss is the ADC's own OVERRUN flag and
// the letter reads it there.
void te_accounting() {
    if (!calibrated && !calibrate()) {
        bench.verdict("the calibration pass runs", false);
        return;
    }
    bench.verdict("the chain comes up", chain_up());

    // Let the stream fill both buffers and then some.
    wait_ms(60);
    const uint32_t laps_at_stall = AdcStream::laps();
    const uint8_t pending = AdcStream::pending();
    const bool stalled = AdcStream::stalled();
    const uint32_t overruns = AdcStream::overruns();
    const bool adc_overrun = (Adc0::flags() & Adc0::flag_overrun) != 0u;

    print(serial, "  after 60 ms with no drainer: laps ", laps_at_stall,
          ", pending ", pending, ", stalled ", yes_no(stalled), ", overruns ",
          overruns, "; the ADC's own OVERRUN reads ", yes_no(adc_overrun), crlf);

    bench.verdict("both buffers filled and the engine stopped rather than "
                  "write into the one the caller holds",
                  pending == 2u && stalled && overruns >= 1u);
    bench.verdict("THE ENGINE DOES NOT COUNT WHAT IT NEVER MOVED: the samples "
                  "lost during the stall are the converter's to report, and "
                  "the converter reports them",
                  adc_overrun);

    // WHAT WAS HANDED OVER IS STILL WHOLE. That is the trade the skip
    // buys, so it is the thing to check: both held buffers still fit the
    // table at some offset, inside letter b's band.
    //
    // MEASURED WITHOUT A PRINT ANYWHERE INSIDE, and that is not tidiness:
    // one verdict line is about four milliseconds of console at 115200
    // and a restarted stream fills a 24-sample block in 4.8 ms, so a
    // `stalled()` read placed after a print is a read of the NEXT stall
    // and not of the release under test. The first version of this
    // letter failed exactly that way.
    uint32_t e0 = 0;
    uint32_t w0 = 0;
    const volatile uint16_t* first = AdcStream::ready();
    const bool first_ok = first != nullptr;
    if (first_ok) {
        (void)best_offset(first, e0, w0);
    }
    const bool released_first = AdcStream::release();
    const bool restarted = !AdcStream::stalled();

    uint32_t e1 = 0;
    uint32_t w1 = 0;
    const volatile uint16_t* second = AdcStream::ready();
    const bool second_ok = second != nullptr;
    if (second_ok) {
        (void)best_offset(second, e1, w1);
    }
    (void)AdcStream::release();

    bench.verdict("the first held buffer is an untorn block: it still fits "
                  "the table inside the measured band",
                  first_ok && e0 <= band());
    bench.verdict("and so is the second", second_ok && e1 <= band());
    print(serial, "  the two held blocks fit within ", e0, " and ", e1,
          " counts", crlf);

    // The first release is what restarts a stalled stream - the one
    // place that verb does more than bookkeeping.
    bench.verdict("releasing restarted the stream", released_first && restarted);

    Adc0::clear_flags(Adc0::flag_overrun);
    wait_ms(40);
    const uint32_t laps_after = AdcStream::laps();
    print(serial, "  after releasing and waiting 40 ms the lap count went ",
          laps_at_stall, " -> ", laps_after, crlf);
    bench.verdict("and the stream really resumed - the lap count moved again",
                  laps_after > laps_at_stall);

    // A drained stream, drained properly, raises nothing at all.
    const uint32_t overruns_before = AdcStream::overruns();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 100u) {
        if (AdcStream::ready() != nullptr) {
            (void)AdcStream::release();
        }
    }
    bench.verdict("with the drainer keeping up, no further overrun is counted",
                  AdcStream::overruns() == overruns_before);
    chain_down();
}

// =============================================================================
// f - erratum 1.10.4: the same chain under concurrent churn
// =============================================================================
//
// 1.10.4 is live on this silicon (E/G/J revisions E, F and H): when
// several channels are triggered concurrently the WRITE-BACK descriptors
// may be corrupted, and 25.6.2.6 makes the write-back the controller's
// LIVE copy - so a corrupted one does not merely misreport a transfer,
// it destroys it.
//
// THE POSITION THIS DRIVER TAKES IS TO REFUSE THE READING, NEVER SUFFER
// IT: harvest() compares every invariant field of a write-back against
// the descriptor that was loaded and discards a reading that fails,
// counting it. This letter runs the analog chain with two more channels
// hammering memory-to-memory blocks alongside it, and asks the only
// question that matters: were the SAMPLES right, whatever the readings
// said.
void tf_churn() {
    if (!calibrated && !calibrate()) {
        bench.verdict("the calibration pass runs", false);
        return;
    }
    bench.verdict("the chain comes up", chain_up());

    for (uint16_t i = 0; i < 128; ++i) {
        churn_src[i] = static_cast<uint8_t>(i * 7u + 3u);
        churn_dst[i] = 0;
    }
    Churn0::clear_counters();
    Churn1::clear_counters();
    (void)Churn0::reset();
    (void)Churn1::reset();
    bench.verdict("two more channels come up as memory-to-memory churn",
                  Churn0::configure(churn_config) &&
                      Churn1::configure(churn_config));

    uint32_t churn_blocks = 0;
    uint32_t churn_deaths = 0;
    uint32_t harvests = 0;
    uint32_t refused = 0;
    uint32_t drained = 0;
    uint32_t judged = 0;
    uint32_t adc_deaths = 0;
    uint32_t dac_deaths = 0;
    uint32_t worst_fit = 0;
    uint8_t skip = 0;
    bool blocks_ok = true;

    uint32_t last_block = Ticker::millis();
    uint32_t last_lap_ms = last_block;
    uint32_t last_laps = DacLoop::laps();
    uint32_t last_harvest_ms = last_block;

    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 800u) {
        churn_blocks += churn_turn<Churn0>(0, churn_deaths);
        churn_blocks += churn_turn<Churn1>(64, churn_deaths);

        const uint32_t now = Ticker::millis();

        // Ask the streaming channel where it has got to - the one path
        // that reads a write-back at all, and therefore the one 1.10.4
        // can reach. PACED at about one per millisecond and not spun
        // on: harvest() suspends the channel inside a critical section,
        // so a tight loop over it is a stressor in its own right and
        // would confuse what is being measured with what is doing the
        // measuring.
        if (now != last_harvest_ms) {
            last_harvest_ms = now;
            ++harvests;
            if (!AdcStream::progress()) {
                ++refused;
            }
        }

        const volatile uint16_t* buf = AdcStream::ready();
        if (buf != nullptr) {
            uint32_t best_err = 0;
            uint32_t worst_err = 0;
            (void)best_offset(buf, best_err, worst_err);
            // A block that SPANS a recovery cannot fit the table and
            // must not be judged as if it could: the loop restarts at
            // the table's head, so the samples either side of an
            // abandonment belong to two different phases. Two blocks
            // are skipped after any death and the count of what WAS
            // judged is printed, so nothing is quietly excused.
            if (skip != 0u) {
                --skip;
            } else {
                ++judged;
                if (best_err > worst_fit) {
                    worst_fit = best_err;
                }
                if (best_err > band()) {
                    blocks_ok = false;
                }
            }
            ++drained;
            (void)AdcStream::release();
            last_block = now;
        }

        if (DacLoop::laps() != last_laps) {
            last_laps = DacLoop::laps();
            last_lap_ms = now;
        }

        // THE OWNER'S DEAD-BLOCK PREDICATE, and here it is a wall clock
        // because the owner knows something the engine cannot: THE
        // PACER IS RUNNING. A stream that has handed over nothing for
        // fifty milliseconds - ten blocks' worth at 5 kHz - is not slow,
        // it is dead, and the erratum's destructive form is what kills
        // it. abandon() is the answer, and the ADC's standing result is
        // DRAINED rather than kicked for the reason chain_up() gives.
        if (now - last_block > 50u) {
            ++adc_deaths;
            skip = 2;
            (void)AdcStream::abandon();
            if (Adc0::ready()) {
                (void)Adc0::result();
            }
            last_block = now;
        }
        if (now - last_lap_ms > 50u) {
            ++dac_deaths;
            skip = 2;
            (void)DacLoop::abandon();
            DacLoop::kick();
            last_lap_ms = now;
        }
    }
    const uint32_t violations = DmaChannel<ch_adc>::violations() +
                                DmaChannel<ch_dac>::violations() +
                                Churn0::violations() + Churn1::violations();
    const uint32_t timeouts = DmaChannel<ch_adc>::suspend_timeouts();

    // ---- THE PROVOCATION -----------------------------------------------
    //
    // THE PACER KEEPS RUNNING THROUGH THIS, and the first version of the
    // letter got that wrong: with the chain stopped, the "provocation"
    // was churn and harvests against an idle channel, it delivered no
    // block at all, and the stall detector duly fired once per fifty
    // milliseconds - five times, every run, deterministically. Five
    // identical numbers from a thing described as weather is the shape
    // of a measurement that is measuring the test.
    //
    // The loop above is what an application would do. THIS is not: a
    // TIGHT harvest loop, with no pacing at all. harvest() suspends the
    // channel inside a critical section, so spinning on it is itself a
    // concurrency stressor on top of the churn - and it is what reaches
    // erratum 1.10.4 on this silicon, where the paced loop above
    // typically does not (measured: 682 refusals in 21100 tight
    // harvests, against none in 800 paced ones over a longer window).
    //
    // WHETHER THE ERRATUM FIRES IS WEATHER - it depends on the arbiter's
    // phase - so nothing here verdicts that it did. What is claimed is
    // the two things that must hold either way: every reading was
    // consistent or REFUSED, and the chain came back.
    const uint32_t v_before = DmaChannel<ch_adc>::violations() +
                              DmaChannel<ch_dac>::violations() +
                              Churn0::violations() + Churn1::violations();
    const uint32_t to_before = DmaChannel<ch_adc>::suspend_timeouts();
    uint32_t burst_harvests = 0;
    uint32_t burst_refused = 0;
    uint32_t burst_deaths = 0;
    uint32_t burst_blocks = 0;
    uint32_t stuck0 = 0;
    uint32_t stuck1 = 0;
    last_block = Ticker::millis();
    const uint32_t t_burst = Ticker::millis();
    while (Ticker::millis() - t_burst < 300u) {
        churn_spray<Churn0>(0, burst_blocks, churn_deaths, stuck0);
        churn_spray<Churn1>(64, burst_blocks, churn_deaths, stuck1);
        ++burst_harvests;
        if (!AdcStream::progress()) {
            ++burst_refused;
        }
        if (AdcStream::ready() != nullptr) {
            (void)AdcStream::release();
            last_block = Ticker::millis();
        }
        const uint32_t now = Ticker::millis();
        if (now - last_block > 50u) {
            ++burst_deaths;
            (void)AdcStream::abandon();
            if (Adc0::ready()) {
                (void)Adc0::result();
            }
            last_block = now;
        }
    }
    const uint32_t burst_violations =
        DmaChannel<ch_adc>::violations() + DmaChannel<ch_dac>::violations() +
        Churn0::violations() + Churn1::violations() - v_before;
    const uint32_t burst_timeouts =
        DmaChannel<ch_adc>::suspend_timeouts() - to_before;
    (void)Pacer::enable(false);

    // AND THEN: IS IT STILL ALIVE? A letter that merely survived its own
    // stress proves nothing about recovery, so the churn is stopped and
    // the stream is given a clean window of its own.
    (void)Churn0::enable(false);
    (void)Churn1::enable(false);
    if (Adc0::ready()) {
        (void)Adc0::result();
    }
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    (void)Pacer::enable(true);
    uint32_t after = 0;
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 150u) {
        if (AdcStream::ready() != nullptr) {
            (void)AdcStream::release();
            ++after;
        }
    }
    (void)Pacer::enable(false);

    // THE RECOVERY LADDER, AND ITS SECOND RUNG IS A MEASUREMENT.
    //
    // abandon() reclaims a channel by resetting and reconfiguring it -
    // and CHCTRLA.SWRST is IGNORED while ENABLE is still set, silently
    // (25.8.18), while ENABLE itself does not clear until the internal
    // buffer has drained. So when the erratum leaves a channel that
    // will not go down, there is nothing at the CHANNEL level left to
    // try, and the only way out is a reset of the BLOCK.
    //
    // Measured here rather than assumed: on the runs where the
    // provocation reached the erratum hard, abandon() did NOT bring the
    // stream back and this rung did. Which rung was needed is printed,
    // because "it recovered" and "it recovered by resetting the whole
    // controller" are not the same claim.
    uint32_t after_reset = 0;
    const bool needed_block_reset = after <= 20u;
    if (needed_block_reset) {
        chain_down();
        (void)Dmac::init();
        Nvic::enable(Dmac::irq());
        if (chain_up()) {
            const uint32_t t2 = Ticker::millis();
            while (Ticker::millis() - t2 < 150u) {
                if (AdcStream::ready() != nullptr) {
                    (void)AdcStream::release();
                    ++after_reset;
                }
            }
        }
        (void)Pacer::enable(false);
    }

    print(serial, crlf, "  ---- erratum 1.10.4 under churn ----", crlf);
    print(serial, "  churn blocks ", churn_blocks, " (", churn_deaths,
          " destroyed and reset), harvests ", harvests, ", refused ", refused,
          ", write-back violations ", violations, ", suspend timeouts ",
          timeouts, crlf);
    print(serial, "  analog blocks drained ", drained, " (", judged,
          " judged), worst residual ", worst_fit, " counts against a band of ",
          band(), "; stalls ADC ", adc_deaths, " DAC ", dac_deaths,
          ", engine faults ", DacLoop::faults(), "/", AdcStream::faults(), crlf);
    print(serial, "  THE PROVOCATION (sprayed churn + tight harvest, 300 ms): ",
          burst_blocks, " churn blocks, ", burst_harvests, " harvests, ",
          burst_refused, " refused, ", burst_violations,
          " write-back violations, ", burst_timeouts, " suspend timeouts, ",
          burst_deaths, " stalls detected and recovered", crlf);
    print(serial, "  with the churn stopped the stream delivered ", after,
          " more blocks in 150 ms", crlf);
    if (needed_block_reset) {
        print(serial, "  -> ABANDON() WAS NOT ENOUGH: a channel whose ENABLE "
                      "will not clear cannot be reset (25.8.18 ignores SWRST "
                      "silently while it stands), so the BLOCK was reset - "
                      "after which the chain delivered ",
              after_reset, " blocks in 150 ms", crlf);
    }
    // TWO DIFFERENT THINGS, AND THIS LETTER DOES NOT CONFLATE THEM.
    // A REFUSED READING is erratum 1.10.4 caught in the act: a
    // write-back whose invariant fields no longer match the descriptor
    // that was loaded, which nothing but corruption can produce. A
    // STALL is a stream that stopped delivering - which the erratum's
    // destructive form would cause, but so would other things this
    // bench cannot separate. So the stalls are reported as stalls,
    // recovered, and NOT attributed.
    if (violations + burst_violations != 0u) {
        print(serial, "  ERRATUM 1.10.4 CAUGHT IN THE ACT on this run: ",
              violations + burst_violations,
              " write-back readings failed the invariant check and every one "
              "of them was REFUSED, never believed", crlf);
    } else {
        print(serial, "  no write-back reading failed its invariant check on "
                      "this run - whether the erratum fires is the arbiter's "
                      "weather, so nothing here verdicts that it did", crlf);
    }
    if (adc_deaths + dac_deaths + burst_deaths + churn_deaths != 0u) {
        print(serial, "  and ", adc_deaths + dac_deaths + burst_deaths +
                                    churn_deaths,
              " transfers STOPPED DELIVERING and were abandoned and restarted "
              "- reported as stalls and NOT attributed to the erratum, since "
              "no refused reading accompanied them", crlf);
    }

    bench.verdict("the churn really ran alongside the analog chain",
                  churn_blocks > 100u && drained > 10u && judged > 5u);
    bench.verdict("THE SAMPLES WERE RIGHT THROUGHOUT: every block judged under "
                  "the churn still fits the table inside the band",
                  blocks_ok);
    bench.verdict("every write-back reading was either consistent with the "
                  "descriptor that was loaded or REFUSED - none was believed",
                  refused == violations + timeouts);
    bench.verdict("and the refusals are accounted, not silent: the counters "
                  "add up to what harvest() answered",
                  harvests >= refused);
    bench.verdict("UNDER THE PROVOCATION TOO, every reading was consistent or "
                  "refused - the tight harvest loop is what reaches the "
                  "erratum, and it never got a wrong answer believed",
                  burst_refused == burst_violations + burst_timeouts &&
                      burst_harvests > refused);
    bench.verdict("THE CHAIN IS ALIVE AT THE END, whether or not the erratum "
                  "struck - by abandon() where the channel can still be "
                  "reclaimed, and by a controller reset where it cannot; the "
                  "line above says which rung this run needed",
                  after > 20u || after_reset > 20u);

    chain_down();
    // HAND THE CONTROLLER BACK CLEAN. The erratum's destructive form
    // leaves a channel enabled for ever with no flag on it, and a letter
    // that left one behind would poison every letter after it - which is
    // exactly what the first version of this suite did, turning one
    // strike in `f` into twelve failures across `g`..`j`.
    (void)Dmac::init();
    Nvic::enable(Dmac::irq());
}

// =============================================================================
// g - A TRIGGER IS AN EDGE, AND WHERE THE EDGE COMES FROM
// =============================================================================
//
// A peripheral asserts its DMA request as a LEVEL and the controller
// latches a pending trigger when that level RISES (25.8.8) - which is
// the whole mechanism behind the wedge the UART campaign diagnosed, and
// behind kick() existing at all. This letter asks the sharper question
// that campaign left open: WHICH rise?
//
// The answer, measured here in two legs with the same standing request,
// is that the trigger multiplexer's OUTPUT is what has to rise, and
// there are two ways to make it:
//
//   LEG A  SELECTING TRIGSRC IS ITSELF AN EDGE. Write CHCTRLB.TRIGSRC
//          while the peripheral's request already stands and the mux
//          output goes from the DISABLE code's constant zero to one - so
//          a channel CONFIGURED onto a standing request does NOT wedge,
//          it takes the trigger. That is why every arm() in this driver
//          - which resets and reconfigures the channel - starts cleanly
//          on a standing request, and it is a fact no part of ch. 25
//          states.
//   LEG B  A RISE THAT ARRIVES WHILE THE CHANNEL IS DISABLED is the
//          case that has nowhere to go: TRIGSRC is already selected, so
//          there is no mux transition, and the channel is not there to
//          take the peripheral's own. This is the shape a re-armed
//          stream is in between blocks, and it is what kick() is for.
void tg_edge_not_level() {
    bench.verdict("the ADC comes up, CPU-driven", adc_up(false));
    Adc0::select(AnalogIn<Vout>{});
    bench.verdict("the DAC holds a level on the shared pad",
                  dac_up(false) && Dac::set(700));
    settle();

    static volatile uint16_t landing[2] = {0xAAAA, 0xAAAA};
    const DmaChannelConfig trig{
        .trigger = Adc0::dma_trigger_resrdy,
        .action = DmaTriggerAction::beat,
    };
    const DmaTransfer one_beat{
        .source = &Adc0::regs().ADC_RESULT,
        .destination = &landing[0],
        .beats = 1,
        .beat = DmaBeat::hword,
        .source_increment = false,
    };

    // ---- LEG A: the request stands BEFORE the channel is configured ----
    //
    // One conversion, RESULT DELIBERATELY NOT READ: RESRDY - which is
    // the DMA request (38.6.4) - is standing before the channel exists.
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    Adc0::start();
    uint32_t spins = 0xFFFFu;
    while (spins-- != 0u && !Adc0::ready()) {
    }
    bench.verdict("a conversion is left unread, so the ADC's DMA request "
                  "stands as a level before any channel exists",
                  Adc0::ready());

    landing[0] = 0xAAAA;
    (void)Bare::reset();
    Bare::clear_counters();
    const bool armed_a = Bare::configure(trig) && Bare::load(one_beat);
    Bare::enable(true);
    wait_ms(5);
    const uint16_t landed_a = landing[0];
    const bool ready_a = Adc0::ready();
    (void)Bare::enable(false);

    print(serial, "  LEG A - configured onto the standing request: the "
                  "landing halfword is ",
          landed_a, " and RESRDY now reads ", yes_no(ready_a), crlf);
    bench.verdict("a bare channel is armed on the standing request", armed_a);
    bench.verdict("SELECTING TRIGSRC IS ITSELF AN EDGE: the beat moves with "
                  "no software trigger at all, because the multiplexer's "
                  "output rose when the code was written",
                  landed_a != 0xAAAAu && !ready_a);
    bench.verdict("and the beat carried the conversion, not a stale word",
                  landed_a > 20u && landed_a < 4090u);

    // ---- LEG B: the rise arrives while the channel is DISABLED ----
    //
    // The channel keeps the TRIGSRC it already has - nothing is
    // reconfigured, so there is no mux transition to borrow - and the
    // peripheral's own rise happens with the channel not there to take
    // it. That is the state a stream is in between two blocks.
    Adc0::clear_flags(Adc0::flag_resrdy);
    landing[0] = 0xAAAA;
    Adc0::start();
    spins = 0xFFFFu;
    while (spins-- != 0u && !Adc0::ready()) {
    }
    const bool rose_while_disabled = Adc0::ready() && !Bare::enabled();

    // Now give it a block, WITHOUT reconfiguring: load and enable only.
    (void)Bare::load(one_beat);
    Bare::enable(true);
    wait_ms(5);
    const uint16_t landed_b = landing[0];
    const bool needed_kick = landed_b == 0xAAAAu;
    const uint8_t status_b = Bare::status();

    // The missing edge, supplied by the owner - the one thing that can,
    // since the owner is the only thing that reads its peripheral's flag.
    Bare::trigger();
    wait_ms(5);
    const uint16_t landed_after = landing[0];
    (void)Bare::enable(false);

    print(serial, "  LEG B - the rise arrived while the channel was "
                  "disabled, then load+enable with TRIGSRC untouched: the "
                  "landing halfword read ",
          landed_b, " (CHSTATUS ", status_b, ") and after one software "
          "trigger ", landed_after, crlf);
    bench.verdict("the setup is the one the mechanism needs: the request rose "
                  "with the channel disabled and its trigger already selected",
                  rose_while_disabled);
    if (needed_kick) {
        print(serial, "  -> THE WEDGE: the rise had nowhere to go and the "
                      "block sat still until the owner kicked it", crlf);
    } else {
        print(serial, "  -> the controller had latched the rise even with the "
                      "channel disabled, so the block ran on its own", crlf);
    }
    bench.verdict("either way the block runs, and one software trigger is "
                  "what closes the gap when it does not - which is why kick() "
                  "is safe to issue at every first arm (25.8.8 makes a kick "
                  "that races a real trigger LOST, never doubled)",
                  landed_after != 0xAAAAu && landed_after > 20u &&
                      landed_after < 4090u);

    Adc0::release();
    Dac::release();
}

// =============================================================================
// h - a dead block abandoned, counted, and the stream resumed
// =============================================================================
//
// The other half of the 1.10.4 answer. Validating a reading is NECESSARY
// AND NOT SUFFICIENT: when the write-back is corrupted the transfer
// itself is destroyed, so somebody has to throw the block away. WHO
// decides it is dead is the peripheral's owner and never the engine, and
// what the abandonment loses is stated rather than pretended away.
//
// The corruption is injected by hand here - the write-back is this
// driver's own SRAM, so a suite can scribble on it and watch the
// validation say no - which is the same technique test_samc_dma uses on
// the serial engines.
void th_abandon() {
    if (!calibrated && !calibrate()) {
        bench.verdict("the calibration pass runs", false);
        return;
    }
    bench.verdict("the chain comes up", chain_up());

    // EVERY MEASUREMENT BELOW IS TAKEN WITH THE STREAM DRAINED FIRST and
    // all the printing is left to the end - the letter-e lesson: a
    // stream fills a block in 4.8 ms and one verdict line is about four
    // milliseconds of console, so a letter that prints between two
    // measurements is measuring its own console.
    const uint32_t t_warm = Ticker::millis();
    while (Ticker::millis() - t_warm < 20u) {
        if (AdcStream::ready() != nullptr) {
            (void)AdcStream::release();
        }
    }

    DmaChannel<ch_adc>::clear_counters();
    while (AdcStream::ready() != nullptr) {
        (void)AdcStream::release();
    }
    const bool healthy = AdcStream::progress().has_value() &&
                         DmaChannel<ch_adc>::violations() == 0u;

    // SCRIBBLE. One invariant field of the live write-back is enough:
    // the controller copies SRCADDR from the descriptor it fetched and
    // never touches it again, so a changed one can only mean corruption.
    volatile dmac_descriptor_registers_t& wb = Dmac::write_back(ch_adc);
    const uint32_t good = wb.DMAC_SRCADDR;
    wb.DMAC_SRCADDR = good + 4u;
    const bool refused = !AdcStream::progress().has_value();
    const uint32_t violations = DmaChannel<ch_adc>::violations();
    wb.DMAC_SRCADDR = good;

    // THE OWNER DECIDES. Here the fact that makes a block dead is
    // arranged rather than waited for: the pacer is stopped, so the ADC
    // converts nothing, RESRDY cannot rise and the block in flight can
    // never finish. That is a state the owner can read - its converter
    // is idle and its stream claims a block in flight - and it is what
    // abandon() is for.
    (void)Pacer::enable(false);
    // LET THE LAST PERIOD FINISH BEFORE ASKING ANYTHING. Stopping the
    // pacer mid-period leaves one conversion still to complete, and its
    // RESRDY can rise after the drain and before the question - which
    // made the first version of this letter fail about one run in four
    // on a predicate that is otherwise exact. Five milliseconds is
    // twenty-five periods.
    wait_ms(5);
    while (AdcStream::ready() != nullptr) {
        (void)AdcStream::release();
    }
    if (Adc0::ready()) {
        (void)Adc0::result();
    }
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    const uint32_t laps_before = AdcStream::laps();
    const uint32_t faults_before = AdcStream::faults();
    const bool dead = !Adc0::ready() && AdcStream::running() &&
                      !AdcStream::stalled();
    // A STALLED STREAM IS NOT A DEAD ONE, and the engine refuses to
    // count a fault for it: there is no block in flight to throw away.
    const bool refuses_stalled = AdcStream::stalled() ? !AdcStream::abandon()
                                                      : true;
    const bool abandoned = AdcStream::abandon();
    const uint32_t faults_after = AdcStream::faults();
    const bool ready_again = AdcStream::running() && !AdcStream::stalled();

    print(serial, "  harvest on a healthy stream: ", yes_no(healthy),
          "; with one invariant field of the write-back changed it answers ",
          refused ? "nothing" : "a reading", " (violations ", violations,
          ")", crlf);
    bench.verdict("a healthy stream harvests cleanly", healthy);
    bench.verdict("THE READING IS REFUSED AND COUNTED, never believed",
                  refused && violations == 1u);
    bench.verdict("the owner can see the block is dead: its converter is idle "
                  "and its stream still claims one in flight",
                  dead);
    bench.verdict("abandon() throws it away and counts exactly one fault",
                  abandoned && faults_after == faults_before + 1u &&
                      refuses_stalled);
    bench.verdict("and hands back a channel ready for the next block",
                  ready_again);

    // Recovery, which is the whole point: with the pacer running again
    // the stream must produce blocks. The abandoned block's samples are
    // GONE and nothing here pretends otherwise - only the lap count is
    // asked about.
    // THE SAME OWNER RULE AS chain_up(), and the bench charged for it
    // here too: the conversion standing when the pacer stopped is a real
    // result but NOT one of the resumed stream's, so it is DRAINED and
    // not kicked. Kicked, it landed in slot zero of the first block
    // after the recovery and that block came back exactly one table step
    // out - a residual of 120 counts where the noise is four.
    if (Adc0::ready()) {
        (void)Adc0::result();
    }
    Adc0::clear_flags(Adc0::flag_resrdy | Adc0::flag_overrun);
    (void)Pacer::enable(true);
    const uint32_t t0 = Ticker::millis();
    uint32_t drained = 0;
    uint32_t worst_fit = 0;
    bool blocks_ok = true;
    while (Ticker::millis() - t0 < 200u) {
        const volatile uint16_t* buf = AdcStream::ready();
        if (buf == nullptr) {
            continue;
        }
        uint32_t best_err = 0;
        uint32_t worst_err = 0;
        (void)best_offset(buf, best_err, worst_err);
        if (best_err > worst_fit) {
            worst_fit = best_err;
        }
        if (best_err > band()) {
            blocks_ok = false;
        }
        ++drained;
        (void)AdcStream::release();
    }
    (void)Pacer::enable(false);
    print(serial, "  after the abandonment: ", drained,
          " blocks in 200 ms, worst residual ", worst_fit, " counts (band ",
          band(), "); laps ", laps_before, " -> ", AdcStream::laps(), crlf);
    bench.verdict("THE STREAM RESUMED and every block after the abandonment "
                  "is whole",
                  drained > 5u && blocks_ok && AdcStream::laps() > laps_before);
    chain_down();
}

// =============================================================================
// i - the SDADC through the ping-pong engine, on WORD beats
// =============================================================================
//
// WHY THE ELEMENT TYPE IS `uint32_t` HERE AND `uint16_t` FOR THE SAR.
// SDADC.RESULT is a 32-bit register with a 24-BIT datum in it, and the
// SPECIFIED sixteen-bit conversion is its TOP sixteen bits - so a
// halfword beat would carry RESULT[15:0], which is the eight bits BELOW
// the specified datum plus half of it, i.e. not the reading at all. The
// honest element is the word, and this letter proves it with a value
// that could not survive a halfword: a rail-to-rail differential
// saturates the datapath at its 24-bit rail, far past anything sixteen
// bits can hold.
void ti_sdadc() {
    // The pair's two pads, driven by PORT to opposite rails. An analog
    // input is a direct connection to the pad, so a pad left under PORT
    // is read as it stands - the technique the AC and the ADC campaigns
    // established and this suite inherits.
    SdN::output();
    SdN::clear();
    SdP::output();
    SdP::set();

    constexpr SdadcConfig cfg{
        .reference = SdadcRef::vddana,
        .prescaler = 3,
        .osr = SdadcOsr::osr256,
        .free_running = true,
    };
    bench.verdict("the SDADC comes up free-running on pair 0",
                  Sdadc::init(main_gen, cfg, main_gen_hz) && Sdadc::select(0));
    bench.verdict("this package bonds pair 0 (PA06/PA07 - the E has this one "
                  "alone)",
                  Sdadc::pair_exists(0));

    // The CPU's own reading first: what the DMA'd words are compared
    // against, and the spread that is the comparison's band.
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    uint32_t cpu_lo = 0xFFFFFFFFu;
    uint32_t cpu_hi = 0;
    uint32_t cpu_last = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        uint32_t spins = 0xFFFFFFu;
        while (spins-- != 0u && !Sdadc::ready()) {
        }
        cpu_last = Sdadc::result_raw();
        if (cpu_last < cpu_lo) {
            cpu_lo = cpu_last;
        }
        if (cpu_last > cpu_hi) {
            cpu_hi = cpu_last;
        }
    }
    const uint32_t cpu_spread = cpu_hi - cpu_lo;
    print(serial, "  the CPU reads RESULT = ", cpu_last, " raw over a spread "
          "of ", cpu_spread, " (the specified 16-bit datum is ",
          static_cast<int32_t>(sdadc_result_of(cpu_last)), ")", crlf);
    bench.verdict("A RAIL DIFFERENTIAL PUTS THE DATUM PAST SIXTEEN BITS: the "
                  "raw reading does not fit in a halfword, so a halfword beat "
                  "would carry something else entirely",
                  cpu_last > 0xFFFFu);

    for (uint16_t i = 0; i < wide_len; ++i) {
        wide_a[i] = 0;
        wide_b[i] = 0;
    }
    WideStream::arm(&Sdadc::regs().SDADC_RESULT, Sdadc::dma_trigger_resrdy);
    WideStream::clear_faults();
    DmaChannel<ch_wide>::clear_counters();
    bench.verdict("a WORD-beat ping-pong stream is armed on the SDADC's "
                  "RESRDY trigger",
                  WideStream::start(wide_a, wide_b, wide_len));
    Sdadc::clear_flags(Sdadc::flag_resrdy);
    if (Sdadc::ready()) {
        WideStream::kick();
    }

    uint32_t blocks = 0;
    uint32_t worst = 0;
    bool sane = true;
    const uint32_t t0 = Ticker::millis();
    while (blocks < 4u && Ticker::millis() - t0 < 500u) {
        const volatile uint32_t* buf = WideStream::ready();
        if (buf == nullptr) {
            continue;
        }
        for (uint16_t k = 0; k < wide_len; ++k) {
            const uint32_t w = buf[k];
            if (w <= 0xFFFFu) {
                sane = false;   // a word that a halfword could have held
            }
            const uint32_t d = w > cpu_last ? w - cpu_last : cpu_last - w;
            if (d > worst) {
                worst = d;
            }
        }
        ++blocks;
        (void)WideStream::release();
    }
    WideStream::stop();

    print(serial, "  ", blocks, " word blocks; first ", wide_a[0],
          "; worst difference from the CPU's reading ", worst,
          " raw units against a CPU spread of ", cpu_spread, crlf);
    bench.verdict("the stream produced its blocks", blocks == 4u);
    bench.verdict("EVERY DMA'd WORD IS THE WHOLE 24-BIT DATUM, matching what "
                  "the CPU reads within the converter's own measured noise",
                  blocks == 4u && sane &&
                      worst <= cpu_spread + 4u * sdadc_raw_per_count);
    bench.verdict("and no write-back reading was refused on the word channel",
                  DmaChannel<ch_wide>::violations() == 0u &&
                      WideStream::faults() == 0u);

    // ---- THE SECOND LEG, and it is the one that carries the claim ----
    //
    // A rail differential SATURATES the datapath, and a saturated
    // reading is a CONSTANT: "the DMA'd word equals the CPU's" is
    // trivially true of a constant, and so is a spread of zero. So the
    // pair is now SHORTED at ground, where the reading is the
    // converter's own offset and its low bits are live filter output -
    // and the same claim is made against a datum that MOVES.
    SdP::clear();
    settle();
    Sdadc::clear_flags(Sdadc::flag_resrdy | Sdadc::flag_overrun);
    // THE SINC FILTER HAS TO REFILL after a step of this size - 39.6.2.3
    // is why SKPCNT exists at all - so the first conversions after the
    // pad moved are the step response and not the level. Discarding them
    // is what keeps the band below a measurement of the transient: with
    // them in, the CPU's eight readings spanned 19189 raw units against
    // the stream's 208, and the comparison would have been decided by
    // the settling rather than by the agreement.
    for (uint8_t k = 0; k < 6; ++k) {
        uint32_t s = 0xFFFFFFu;
        while (s-- != 0u && !Sdadc::ready()) {
        }
        (void)Sdadc::result_raw();
    }
    int32_t zero_lo = 0x7FFFFFFF;
    int32_t zero_hi = -0x7FFFFFFF;
    int32_t zero_sum = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        uint32_t s = 0xFFFFFFu;
        while (s-- != 0u && !Sdadc::ready()) {
        }
        const int32_t v = Sdadc::result24();
        zero_sum += v;
        if (v < zero_lo) {
            zero_lo = v;
        }
        if (v > zero_hi) {
            zero_hi = v;
        }
    }
    const int32_t zero_mean = zero_sum / 8;
    const int32_t zero_spread = zero_hi - zero_lo;

    for (uint16_t i = 0; i < wide_len; ++i) {
        wide_a[i] = 0;
        wide_b[i] = 0;
    }
    WideStream::arm(&Sdadc::regs().SDADC_RESULT, Sdadc::dma_trigger_resrdy);
    (void)WideStream::start(wide_a, wide_b, wide_len);
    Sdadc::clear_flags(Sdadc::flag_resrdy);

    uint32_t zblocks = 0;
    int32_t s_lo = 0x7FFFFFFF;
    int32_t s_hi = -0x7FFFFFFF;
    const uint32_t tz = Ticker::millis();
    while (zblocks < 4u && Ticker::millis() - tz < 500u) {
        const volatile uint32_t* buf = WideStream::ready();
        if (buf == nullptr) {
            continue;
        }
        for (uint16_t k = 0; k < wide_len; ++k) {
            const int32_t v = sdadc_raw_signed(buf[k]);
            if (v < s_lo) {
                s_lo = v;
            }
            if (v > s_hi) {
                s_hi = v;
            }
        }
        ++zblocks;
        (void)WideStream::release();
    }
    WideStream::stop();

    print(serial, "  shorted at ground: the CPU's eight readings span ",
          zero_lo, " .. ", zero_hi, " raw (mean ", zero_mean,
          "), the streamed sixteen span ", s_lo, " .. ", s_hi, crlf);
    bench.verdict("the shorted stream produced its blocks", zblocks == 4u);
    bench.verdict("THE DATUM IS LIVE AND NOT A SATURATED CONSTANT: the "
                  "reading has left the rail and its low bits move",
                  zblocks == 4u && s_hi < 0x400000 && s_lo > -0x400000);
    // THE BAND IS THE MEASUREMENT'S OWN, and it has to be: two
    // sixteen-reading samples of the same noisy quantity are compared,
    // so the allowance is twice the spread the CPU saw plus one
    // sixteen-bit count, and the letter prints both spans so the reader
    // can see the comparison rather than take it.
    const int32_t allow = 2 * zero_spread + sdadc_raw_per_count;
    bench.verdict("and the streamed readings are the same quantity the CPU "
                  "read, inside twice the spread the CPU itself showed",
                  zblocks == 4u && s_lo > zero_mean - allow &&
                      s_hi < zero_mean + allow);

    Sdadc::release();
    SdN::release();
    SdP::release();
}

// =============================================================================
// j - TSENS through the same engine
// =============================================================================
//
// The cheapest possible second user of the word-beat stream, and a
// converter that is not a converter: TSENS counts a temperature-
// dependent oscillator against GCLK_TSENS, so its VALUE is a SIGNED
// 24-bit datum in a 32-bit register - the same shape the SDADC has and
// the same reason the element is a word. The factory calibration assumes
// 48 MHz, which is the generator this suite runs everything on.
void tj_tsens() {
    constexpr TsensConfig base{.free_running = true};
    TsensConfig cfg = base;
    cfg.calibration = TsensCalibration::factory();
    bench.verdict("the factory GAIN and OFFSET are readable, and a zero GAIN "
                  "would be refused (it is 2^24, not none)",
                  cfg.calibration.gain != 0u && tsens_config_valid(cfg));
    bench.verdict("TSENS comes up free-running on the 48 MHz its calibration "
                  "assumes",
                  Tsens::init(main_gen, cfg));

    Tsens::clear_flags(Tsens::flag_result_ready);
    uint32_t spins = 0xFFFFFFu;
    while (spins-- != 0u && !Tsens::result_ready()) {
    }
    const int32_t cpu_centi = Tsens::value();
    print(serial, "  the CPU reads ", cpu_centi, " centi-degrees Celsius", crlf);
    bench.verdict("and the reading is a plausible die temperature",
                  cpu_centi > -2000 && cpu_centi < 12000);

    for (uint16_t i = 0; i < wide_len; ++i) {
        wide_a[i] = 0;
        wide_b[i] = 0;
    }
    WideStream::arm(&Tsens::regs().TSENS_VALUE, Tsens::dma_trigger_resrdy);
    WideStream::clear_faults();
    DmaChannel<ch_wide>::clear_counters();
    bench.verdict("the same WORD-beat engine is armed on TSENS's RESRDY "
                  "trigger - the engine knows nothing about either "
                  "peripheral",
                  WideStream::start(wide_a, wide_b, wide_len));
    Tsens::clear_flags(Tsens::flag_result_ready);
    if (Tsens::result_ready()) {
        WideStream::kick();
    }

    uint32_t blocks = 0;
    int32_t lo = 0x7FFFFFFF;
    int32_t hi = -0x7FFFFFFF;
    const uint32_t t0 = Ticker::millis();
    while (blocks < 2u && Ticker::millis() - t0 < 1000u) {
        const volatile uint32_t* buf = WideStream::ready();
        if (buf == nullptr) {
            continue;
        }
        for (uint16_t k = 0; k < wide_len; ++k) {
            const int32_t v = tsens_signed(buf[k]);
            if (v < lo) {
                lo = v;
            }
            if (v > hi) {
                hi = v;
            }
        }
        ++blocks;
        (void)WideStream::release();
    }
    WideStream::stop();

    print(serial, "  ", blocks, " word blocks; the streamed readings span ",
          lo, " .. ", hi, " centi-C against the CPU's ", cpu_centi, crlf);
    bench.verdict("the stream produced its blocks", blocks == 2u);
    bench.verdict("EVERY STREAMED VALUE IS THE SAME DIE THE CPU READ, within "
                  "the spread of the readings themselves",
                  blocks == 2u && lo > cpu_centi - 400 && hi < cpu_centi + 400);
    bench.verdict("no write-back reading was refused",
                  DmaChannel<ch_wide>::violations() == 0u &&
                      WideStream::faults() == 0u);
    Tsens::release();
}

// =============================================================================
// k - the util contract, live: BlockRelay inside a running kernel
// =============================================================================
//
// The block-stream vocabulary (util/block_stream.hpp,
// design/block-stream.md) run on the silicon that shaped it: the same
// DAC-to-ADC chain as letter c, but the blocks now travel as
// Lease::dispatch loans through a REAL kernel - StreamSink (the
// borrower) before BlockRelay (the lender) in the pack, the DMAC
// completion posting the wakeup, and every buffer returned to the
// engine by the relay's next dispatch. The sink does letter c's whole
// verification INSIDE the loan window, which is the point: a block is
// consumed during one dispatch, and nothing is copied anywhere.

/// The DMAC handler posts to the relay only while this letter runs -
/// the other letters drain the engine by hand and a queue nobody pumps
/// would merely count overflows, but a suite should not manufacture
/// noise to ignore.
volatile bool relay_live = false;

struct StreamSink : Fsm<StreamSink, BlockReady<uint16_t>> {
    using Base = Fsm<StreamSink, BlockReady<uint16_t>>;
    using Event = typename Base::Event;
    using Status = typename Base::Status;
    static inline EventQueue<Event, 4, SamPlatform> queue;

    static constexpr uint8_t max_blocks = 16;
    static inline uint8_t seen = 0;
    static inline uint16_t offsets[max_blocks];
    static inline uint32_t worst_fit = 0;
    static inline uint16_t lengths_ok = 0;

    static void init() {
        seen = 0;
        worst_fit = 0;
        lengths_ok = 0;
        Base::start(&only);
    }
    static void dispatch(const Event& e) { Base::dispatch(e); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return Base::handled(); },
            [](Exit) { return Base::handled(); },
            [](BlockReady<uint16_t> b) {
                // The whole verification happens HERE, inside the loan's
                // window - after this dispatch returns, the relay hands
                // the buffer back and the engine refills it.
                if (b.data && b.length == block_len &&
                    lengths_ok != UINT16_MAX) {
                    ++lengths_ok;
                }
                if (b.data && seen < max_blocks) {
                    uint32_t best_err = 0;
                    uint32_t worst_err = 0;
                    offsets[seen] =
                        best_offset(b.data.get(), best_err, worst_err);
                    if (best_err > worst_fit) {
                        worst_fit = best_err;
                    }
                    ++seen;
                }
                return Base::handled();
            });
    }
};

using Relay = BlockRelay<SamPlatform, Subscribers<StreamSink>, AdcStream>;
using StreamKernel = Kernel<SamPlatform, StreamSink, Relay>;

void tk_relay() {
    if (!calibrated && !calibrate()) {
        bench.verdict("the calibration pass runs", false);
        return;
    }
    // EVERY MEASUREMENT FIRST, EVERY PRINT AFTER - the suite's own
    // lesson, and here it is load-bearing: chain_up() starts the 5 kHz
    // stream, a print is milliseconds (and BLOCKS while the console
    // ring, still full of the previous letters' output in a z run,
    // drains at 115200), and the engine's whole slack is two blocks =
    // 9.6 ms. The first version printed the chain verdict between the
    // start and the pump and overran once, deterministically, in every
    // z run - and never when the letter ran alone with an empty ring.
    const bool up = chain_up();

    constexpr uint8_t blocks = 12;
    StreamKernel::init_all();
    relay_live = true;

    const uint32_t t0 = Ticker::millis();
    while (StreamSink::seen < blocks && Ticker::millis() - t0 < 500u) {
        (void)StreamKernel::step();
    }
    (void)Pacer::enable(false);
    // Drain the tail: blocks already in flight when the pacer stopped
    // still arrive, and the last loans must come home.
    while (StreamKernel::step()) {
    }
    relay_live = false;

    const uint8_t seen = StreamSink::seen;
    bench.verdict("the chain comes up: pacer, fabric, both converters, both "
                  "engines", up);
    print(serial, "  ", seen, " blocks through the kernel (", blocks,
          " asked; the tail is blocks already in flight at the stop); worst "
          "residual ", StreamSink::worst_fit, " counts, band ", band(), crlf);
    print(serial, "  relay published ", Relay::published(), ", engine laps ",
          AdcStream::laps(), ", overruns ", AdcStream::overruns(),
          ", pending at the end ", AdcStream::pending(), crlf);

    bench.verdict("the relay delivered the stream through a real kernel",
                  seen >= blocks && seen <= blocks + 3u &&
                      StreamSink::lengths_ok == seen);
    bench.verdict("EVERY BLOCK IS EXACT INSIDE ITS OWN LOAN WINDOW - letter "
                  "c's verification, done during the dispatch",
                  seen >= blocks && StreamSink::worst_fit <= band());
    bool arithmetic = seen >= blocks;
    for (uint8_t i = 1; i < seen; ++i) {
        const uint16_t want = static_cast<uint16_t>(
            (StreamSink::offsets[i - 1] + block_len) % table_len);
        if (StreamSink::offsets[i] != want) {
            arithmetic = false;
        }
    }
    bench.verdict("the phase arithmetic holds block after block: the kernel "
                  "path loses nothing either",
                  arithmetic);
    bench.verdict("every loan came home: no buffer is still pending and the "
                  "engine never skipped a lap - the kernel consumer kept up",
                  AdcStream::pending() == 0u && AdcStream::overruns() == 0u);
    bench.verdict("the relay's accounting is the source's, passed through",
                  Relay::published() == seen &&
                      Relay::laps(0) == AdcStream::laps() &&
                      Relay::overruns(0) == AdcStream::overruns());
    chain_down();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_samc_analog_dma - streaming DMA for the analog "
                        "peripherals (board C, no wires)", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

// The DMAC has ONE vector for twelve channels, and INTPEND names the
// lowest one pending together with its flags - so the whole dispatch is
// a read and a store, with no CHID contention against main context.
//
// THE TRAP THIS BINDING AVOIDS, learned by the UART campaign: an engine
// is a static-only class, so telling the WRONG engine that a block
// finished reprograms a running channel. Here each channel belongs to
// exactly one engine and the switch says so; a transfer error is not a
// completion and must not be reported as one.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        if (!irq->complete()) {
            continue;
        }
        switch (irq->channel) {
        case ch_dac: (void)DacLoop::complete(); break;
        case ch_adc:
            (void)AdcStream::complete();
            // Letter k routes completions into the kernel: the relay's
            // wakeup is the engine's own completion, posted from here.
            if (relay_live) {
                brio::post<Relay>(brio::BlockDone{});
            }
            break;
        case ch_wide: (void)WideStream::complete(); break;
        default: break;
        }
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

    bench.letter('a', "the shapes: element types, beats, published codes",
                 ta_shapes);
    bench.letter('b', "the noise floor first, then the band", tb_noise);
    bench.letter('c', "THE ROUND TRIP: table played, captured, phase-locked",
                 tc_round_trip);
    bench.letter('d', "the rate against the wall clock", td_rate);
    bench.letter('e', "the accounting under a slow drainer", te_accounting);
    bench.letter('f', "erratum 1.10.4: the chain under concurrent churn",
                 tf_churn);
    bench.letter('g', "a trigger is an EDGE: the wedge, and kick()",
                 tg_edge_not_level);
    bench.letter('h', "a dead block abandoned, counted, resumed", th_abandon);
    bench.letter('i', "the SDADC on WORD beats", ti_sdadc);
    bench.letter('j', "TSENS on the same word engine", tj_tsens);
    bench.letter('k', "the util contract live: BlockRelay in a real kernel",
                 tk_relay);

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
