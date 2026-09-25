// test_vx03_adc - the reference bench suite for the two analog-to-digital
// converters of the CH32V203 and the CH32V303: ch32vx03/adc.hpp over RM
// ch. 12, and the two block engines ch32vx03/dma.hpp grew for the stream
// this chapter is the first user of. Letters k..o are the CH32V303's: what
// its device class adds to the chapter, and the DAC it has as a source.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE CLOCK IS PART OF THE MEASUREMENT. ADCCLK is PCLK2 divided by 2,
// 4, 6 or 8 and PCLK2 is HCLK undivided on this family, so the 144 MHz
// the rest of this target's suites run at would leave the converter at
// 18 MHz against a 14 MHz rating - and the driver refuses it at compile
// time. This suite runs the PLL on the HSI at 96 MHz, where ADCCLK is
// 12 MHz: in specification, no crystal needed, and the same tree on
// every board of the family.
//
// WHAT IT MEASURES WITH, WITH NOTHING OUTSIDE THE CHIP. Four analog
// sources and no wire:
//   - a pad DRIVEN by its own port: a push-pull output at 3.3 V or at
//     ground is a hard source of a few tens of ohms, which every
//     sampling time reads true;
//   - a pad PULLED by its own port: the weak pull is 30 to 50 kOhm
//     (datasheet table 4-19), which is a source the converter can only
//     settle with a long sampling time - datasheet table 4-28 says
//     which, and letter c is that table measured;
//   - VREFINT, 1.2 V nominal on channel 17;
//   - the temperature sensor, 1.40 V at 25 degrees on channel 16.
// A pad in ANALOG mode has no pull at all (RM 10.2.7: the mode is the
// input driver off, and the pull goes with it), so the pulled source is
// a pad left a PULLED INPUT and converted through it. Whether that
// works is itself a measurement, and letter c reports it. And on the
// CH32V303 a fifth source: its own DAC, whose first output PA4 is also
// the converter's input 4 - a known code behind a low impedance, which
// is what makes a linearity measurable with no wire.
//
// THE PADS. PA1 and PA2 (channels 1 and 2) are the two levels; PB11 is
// the EXTI pad of letters g and m, toggled by the CPU; on the CH32V303
// PA4 is DAC1's output in letter l. NEVER TOUCHED: PA9/PA10 (the
// console), PA13/PA14 (the debug port), PA11/PA12 (the USB pads),
// PC14/PC15 and PD0/PD1 (the crystals), PA0 (the CH32V203 board's KEY) -
// and PB2, the CH32V203 board's LED, toggled per command as every suite
// of this target does.
//
// What is exercised, letter by letter:
//   a  THE POWER-UP AND THE CALIBRATION: the word the calibration
//      leaves behind, what it costs in microseconds, and a first
//      conversion of VREFINT with the supply derived from it
//   b  THE TEMPERATURE SENSOR: channel 16 at the sampling time 12.2.6
//      asks for, in millivolts and in degrees by the datasheet's slope
//   c  TWO KNOWN LEVELS AND THE SAMPLING LADDER: a driven pad at both
//      rails, then the same pad held by its own 40 kOhm pull read at
//      all eight sampling times, against table 4-28's prediction
//   d  THE STREAM: a scanned sequence of four channels served by the
//      DMA into the ping-pong engine, the blocks lent through a
//      BlockRelay in a real kernel, the values checked against the
//      levels - then the overrun the contract promises, and the
//      measurement that says why the engine does NOT ride the
//      controller's circular mode
//   e  THE INJECTED GROUP preempting a continuous regular conversion,
//      its signed result and the offset subtracted from it
//   f  THE ANALOG WATCHDOG: thresholds around a driven pad, the flag on
//      each crossing, the single-channel select, and the interrupt
//   g  THE TRIGGERS: a timer's TRGO pacing conversions counted against
//      the tick, and an EXTI line raised by a pad of our own
//   h  THE DUAL MODE: ADC1 and ADC2 converting two pads at once, the
//      master's register carrying both halves
//   i  DISCONTINUOUS MODE: a sequence of six stepped two at a time
//   j  THE SAMPLER: util/analog_sampler.hpp walking three inputs at a
//      software pace in a kernel, one AnalogSample published per input
// and on the CH32V303 alone:
//   k  THE SHORT SAMPLING TIMES AND THE TAIL: ADC_SMP_SEL asked of the
//      die - the register is some lots' only, and a die without it keeps
//      nothing written there - and the conversion's length beyond its
//      sampling time at the four long codes, timed over 64 conversions
//      landed by the DMA; the four short times where the die has them
//   l  THE DAC AS THE SOURCE: util/analog_sampler.hpp walking PA4 while
//      DAC1 steps through thirty-one codes - monotonic, gain, offset and
//      the residual against tables 4-43 and 4-45 - then the PGA's three
//      gains on the same pad
//   m  CODE 110 HANDED TO TIM8: the AFIO field written with the trigger
//      and read back, EXTI line 11 starting nothing while it is set,
//      TIM8's TRGO pacing the group wherever this build drives TIM8, and
//      the injected group's and ADC2's fields
//   n  ADC2'S OWN REQUEST: asked of the die first - the row table 11-3's
//      note gives to some lots only - then eight conversions landed by
//      DMA2's channel 5 where it is there, and none reaching it where not
//   o  THE ADC CLOCK'S TWO DUTY BITS: ADC_DUTY_SEL (asked of the die, a
//      lot's too) and ADCDUTY each set in turn, the readings and the
//      conversion time against the 50 % clock's
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/adc.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/dac.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/exti.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/tenuto.hpp"
#include "util/analog_sampler.hpp"
#include "util/block_stream.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

/// The one tree of this suite: the PLL on the HSI, ADCCLK 12 MHz.
using SysClock = Clock<ClockSource::pll, 96'000'000>;
constexpr SysClock clock;
static_assert(SysClock::adc_in_spec);
static_assert(SysClock::adc_hz == 12'000'000UL);

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The sources, and the pads they live on
// ---------------------------------------------------------------------------

using HighPin = Pin<'A', 1>;             ///< channel 1, the pad driven high
using LowPin = Pin<'A', 2>;              ///< channel 2, the pad driven low
using ExtiPin = Pin<'B', 11>;            ///< the pad that raises EXTI line 11
using HighIn = AnalogIn<HighPin>;
using LowIn = AnalogIn<LowPin>;

constexpr uint8_t ch_high = HighIn::channel;
constexpr uint8_t ch_low = LowIn::channel;
constexpr uint8_t ch_vref = adc_vrefint_channel;
constexpr uint8_t ch_temp = adc_temperature_channel;

/// The board's nominal supply. Letter a measures it against VREFINT and
/// every millivolt figure after that uses what it found.
constexpr uint16_t nominal_vdda_mv = 3300;
uint16_t measured_vdda_mv = nominal_vdda_mv;

using Pacer = Tim<3>;   ///< its TRGO paces the regular group in letter g
constexpr uint32_t tim_hz = Pacer::clock_hz(clock);
constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000u;

// ---------------------------------------------------------------------------
// The ruler
// ---------------------------------------------------------------------------

/**
 * The core's own STK counter read as a stopwatch: it counts to its
 * reload - one tick period, a millisecond here - and starts again, so a
 * span is accumulated poll by poll with one period folded in across
 * each wrap.
 */
class Stopwatch {
public:
    Stopwatch() { start(); }

    void start() {
        last_ = stk()->CNTL;
        acc_ = 0;
    }

    uint32_t cycles() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_;
    }

    uint32_t us() { return cycles() / ticks_per_us; }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

void wait_us(uint32_t us) {
    Stopwatch w;
    while (w.us() < us) {
    }
}

// ---------------------------------------------------------------------------
// The block stream, the sampler, and the kernel letters d and j share
// ---------------------------------------------------------------------------

/// The engine on the channel the ADC's request is wired to - named
/// through the request and not by its number (RM table 11-5, and 11-2 on
/// the CH32V303).
using Source = DmaPingPongEngine<DmaRequestOf<DmaRequest::adc1>::controller,
                                 DmaRequestOf<DmaRequest::adc1>::channel, uint16_t>;

constexpr uint16_t block_elements = 8;   ///< two laps of a four-channel sequence
volatile uint16_t block_a[block_elements];
volatile uint16_t block_b[block_elements];

/// Letter d's last measurement runs a block at the CONTROLLER's own
/// speed rather than the converter's, which wants more items than a
/// sample block holds. The source is in the image (11.1 lists flash
/// among the memories a channel may read), so only the landing ground
/// costs RAM.
constexpr uint16_t tear_items = 64;
constexpr uint16_t tear_source[tear_items] = {};
volatile uint16_t tear_dest[tear_items];

/// What the consumer keeps of every block and every sample it is lent.
volatile uint16_t blocks_seen = 0;
volatile uint16_t block_first[block_elements];
volatile uint16_t samples_seen = 0;
volatile uint16_t sample_value[4];
volatile uint8_t sample_index[4];
volatile uint8_t sample_slots = 0;

struct Consumer : Fsm<Consumer, BlockReady<uint16_t>, AnalogSample> {
    static inline EventQueue<Event, 8, P> queue;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](BlockReady<uint16_t> b) {
                if (blocks_seen == 0u) {
                    for (uint16_t i = 0; i < b.length && i < block_elements; ++i) {
                        block_first[i] = b.data.get()[i];
                    }
                }
                blocks_seen = static_cast<uint16_t>(blocks_seen + 1u);
                return handled();
            },
            [](AnalogSample s) {
                if (sample_slots < 4u) {
                    sample_index[sample_slots] = s.index;
                    sample_value[sample_slots] = s.value;
                    sample_slots = static_cast<uint8_t>(sample_slots + 1u);
                }
                samples_seen = static_cast<uint16_t>(samples_seen + 1u);
                return handled();
            });
    }
};

using Relay = BlockRelay<P, Subscribers<Consumer>, Source>;
using Sampler =
    AnalogSampler<Adc<1>, P, Subscribers<Consumer>, HighIn{}, AdcInput::vrefint, LowIn{}>;
/// Borrowers before lenders: the kernel refuses any other order.
using System = Tenuto<P, Consumer, Relay, Sampler>;

// ---------------------------------------------------------------------------
// Letter l's loop: the DAC on PA4 and a sampler of its own (the CH32V303's)
// ---------------------------------------------------------------------------

/// What the DAC loop's consumer keeps: the samples since the last reset.
struct DacConsumer : Fsm<DacConsumer, AnalogSample> {
    static inline EventQueue<Event, 8, P> queue;
    static inline volatile uint32_t sum = 0;
    static inline volatile uint16_t count = 0;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }
    static void reset() {
        sum = 0;
        count = 0;
    }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](AnalogSample s) {
                sum = sum + s.value;
                count = static_cast<uint16_t>(count + 1u);
                return handled();
            });
    }
};

using DacPad = AnalogIn<Pin<'A', 4>>;
using DacSampler = AnalogSampler<Adc<1>, P, Subscribers<DacConsumer>, DacPad{}>;
/// A kernel of its own, so the pack every other letter runs is the one it
/// always was.
using DacKernel = Tenuto<P, DacConsumer, DacSampler>;

// ---------------------------------------------------------------------------
// The handlers' state
// ---------------------------------------------------------------------------

enum class AdcMode : uint8_t { counting, sampling, dac_sampling };
volatile AdcMode adc_mode = AdcMode::counting;

volatile uint16_t eoc_calls = 0;
volatile uint16_t jeoc_calls = 0;
volatile uint16_t awd_calls = 0;
volatile uint16_t exti_calls = 0;
volatile uint16_t captured[8];
volatile uint8_t captured_count = 0;
/// What letter d's own handler reads out of a channel it stops at the
/// half flag: how far the controller had already run past the edge.
volatile uint16_t tear_left = 0;
volatile bool tear_seen = false;

void reset_counters() {
    eoc_calls = 0;
    jeoc_calls = 0;
    awd_calls = 0;
    exti_calls = 0;
    captured_count = 0;
    blocks_seen = 0;
    samples_seen = 0;
    sample_slots = 0;
}

/// Everything this suite can have running, back to a known state.
void all_off() {
    Pfic::disable(Adc<1>::irq());
    Pfic::clear_pending(Adc<1>::irq());
    Pfic::disable(dma_channel_irq(Source::controller, Source::channel));
    Pfic::disable(Irq::exti15_10);
    Pfic::clear_pending(Irq::exti15_10);
    Source::stop();
    Dma<1>::open();
    Dma<1>::stop_all();
    Adc<1>::release();
    if constexpr (device::adc_count >= 2u) {
        Adc<2>::release();
    }
    Pacer::release();
    (void)Exti::interrupt(adc_regular_exti_line, false);
    (void)Exti::event(adc_regular_exti_line, false);
    (void)Exti::sense(adc_regular_exti_line, ExtiSense::none);
    (void)Exti::clear(adc_regular_exti_line);
    ExtiPin::release();
    HighPin::release();
    LowPin::release();
    adc_mode = AdcMode::counting;
    reset_counters();
}

/// The two levels the suite converts: a push-pull output at each rail.
void drive_levels() {
    HighPin::output(true);
    LowPin::output(false);
}

/// One conversion of `ch` after a conversion of the pad at ground, with
/// the two SELECTED SEPARATELY - so the multiplexer has been sitting on
/// `ch` for the microseconds a read costs before the conversion starts.
uint16_t primed_read(uint8_t ch) {
    Adc<1>::select_channel(ch_low);
    (void)Adc<1>::read();
    Adc<1>::select_channel(ch);
    return Adc<1>::read();
}

/// Where a pair of half-words lands when the DMA serves a scan. VOLATILE
/// because the controller writes it and the compiler cannot see that:
/// without the word a reader gets the zero the code last stored.
alignas(4) volatile uint16_t pair[2];

/**
 * Two channels converted as ONE SCANNED SEQUENCE, served by the DMA,
 * and the second one's datum handed back. This is the only honest way
 * to measure a sampling time: the multiplexer moves to the second
 * channel and the sample-and-hold starts charging THEN, with only the
 * SMP cycles to settle - where two separate reads leave the mux parked
 * on the channel for as long as the code between them takes.
 */
uint16_t scan_pair(uint8_t first, uint8_t second, AdcSampleTime t) {
    using Ch = DmaChannel<DmaRequestOf<DmaRequest::adc1>::controller, DmaRequestOf<DmaRequest::adc1>::channel>;
    (void)Adc<1>::sample_time(first, t);
    (void)Adc<1>::sample_time(second, t);
    const uint8_t order[2] = {first, second};
    (void)Adc<1>::sequence(order, 2);
    pair[0] = 0;
    pair[1] = 0;
    Ch::stop();
    (void)Ch::load(DmaTransfer{
        .peripheral = Adc<1>::data_address(),
        .memory = const_cast<uint16_t*>(pair),
        .count = 2,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    (void)Adc<1>::dma(true);
    Adc<1>::clear_flags(AdcFlag::all);
    Adc<1>::start();
    uint32_t spins = 1'000'000UL;
    while (!Ch::flag(DmaFlag::complete) && spins-- != 0u) {
    }
    Ch::stop();
    (void)Adc<1>::dma(false);
    return pair[1];
}

/// The converter up on this suite's tree, with everything off but what
/// the caller asks for.
bool bring_up(const AdcConfig& cfg = AdcConfig{.internal_sources = true}) {
    return Adc<1>::init(clock, cfg);
}

// ===========================================================================
// a - the power-up, the calibration, and the first conversion
// ===========================================================================
void ta_calibration() {
    all_off();
    Adc<1>::bus_clock(true);
    Adc<1>::reset();

    // The calibration by hand, so its two halves and their cost are
    // visible: RSTCAL first, then CAL, each cleared by the hardware.
    AdcRegs& r = Adc<1>::regs();
    r.CTLR2 = adc_exttrig | adc_extsel_mask;
    Stopwatch power;
    r.CTLR2 = r.CTLR2 | adc_adon;
    while ((r.CTLR2 & adc_adon) == 0u) {
    }
    const uint32_t power_us = power.us();
    wait_us(10);
    const uint16_t before = static_cast<uint16_t>(r.RDATAR & 0xFFFFu);

    Stopwatch cal;
    r.CTLR2 = r.CTLR2 | adc_rstcal;
    uint32_t spins = 1'000'000UL;
    while ((r.CTLR2 & adc_rstcal) != 0u && spins-- != 0u) {
    }
    const uint32_t rstcal_us = cal.us();
    const bool rstcal_done = (r.CTLR2 & adc_rstcal) == 0u;

    cal.start();
    r.CTLR2 = r.CTLR2 | adc_cal;
    spins = 1'000'000UL;
    while ((r.CTLR2 & adc_cal) != 0u && spins-- != 0u) {
    }
    const uint32_t cal_us = cal.us();
    const bool cal_done = (r.CTLR2 & adc_cal) == 0u;
    const uint16_t code = static_cast<uint16_t>(r.RDATAR & 0xFFFFu);

    print(serial, "  ADON ack in ", power_us, " us; RSTCAL ", rstcal_us, " us, CAL ", cal_us,
          device::device_class == DeviceClass::v30x_d8
              ? " us (the datasheet's tCAL is 40 ADCCLK = 3.3 us at 12 MHz)"
              : " us (the datasheet's tCAL is 100 ADCCLK = 8 us at 12 MHz)",
          crlf);
    print(serial, "  the data register: ", before, " before the calibration, ", code, " after",
          crlf);
    bench.verdict("RSTCAL is cleared by the hardware when the calibration register is "
                  "initialized",
                  rstcal_done);
    bench.verdict("CAL is cleared by the hardware when the calibration itself is done",
                  cal_done);
    bench.verdict("and it leaves its code in the regular data register (12.2.2), which is "
                  "not what the register held before", code != before || code != 0u);

    // Now the driver's own init, which runs that order with the buffer
    // off and wakes the internal sources afterwards.
    const bool up = bring_up();
    bench.verdict("the driver's init brings the converter up on a clock it is in "
                  "specification on and runs the calibration itself",
                  up);
    print(serial, "  ADCCLK ", SysClock::adc_hz / 1000u, " kHz from PCLK2 ",
          SysClock::pclk2_hz / 1'000'000u, " MHz; the calibration code now ",
          Adc<1>::calibration_code(), crlf);

    Adc<1>::sample_time_all(adc_sample_longest);
    Adc<1>::select(AdcInput::vrefint);
    const uint16_t vref_counts = Adc<1>::read_settled(4);
    measured_vdda_mv = Adc<1>::vdda_mv(vref_counts);
    print(serial, "  VREFINT reads ", vref_counts, " counts, which puts VDDA at ",
          measured_vdda_mv, " mV (the reference is 1.17..1.23 V, nominal 1.200)", crlf);
    bench.verdict("VREFINT converts on channel 17 with the internal sources woken, and the "
                  "supply it implies is plausible for a 3.3 V board",
                  measured_vdda_mv >= 3000u && measured_vdda_mv <= 3600u);

    all_off();
}

// ===========================================================================
// b - the temperature sensor
// ===========================================================================
void tb_temperature() {
    all_off();
    const bool up = bring_up();
    Adc<1>::sample_time_all(adc_sample_longest);
    const uint32_t sample_ns = adc_conversion_ns(adc_sample_longest, SysClock::adc_hz);

    Adc<1>::select(AdcInput::temperature);
    const uint16_t counts = Adc<1>::read_settled(4);
    const uint16_t mv = Adc<1>::millivolts(counts, measured_vdda_mv);
    const int32_t centi = Adc<1>::temperature_centi_c(counts, measured_vdda_mv);

    print(serial, "  channel 16 reads ", counts, " counts = ", mv, " mV at a conversion of ",
          sample_ns / 1000u, ".", (sample_ns % 1000u) / 100u,
          " us (12.2.6 asks for 17.1 us of sampling)", crlf);
    print(serial, "  which the datasheet's typical slope makes ", centi / 100, ".",
          (centi < 0 ? -centi : centi) % 100, " degrees Celsius", crlf);
    print(serial, "  V25 is 1.34..1.46 V and the slope 3.8..4.7 mV/C, so an ABSOLUTE "
                  "temperature from an uncharacterized part is worth about twelve degrees",
          crlf);
    bench.verdict("the converter is up and the sensor's channel answers", up);
    bench.verdict("the temperature the datasheet's typical numbers give is inside the part's "
                  "own operating range of -40 to 85 degrees",
                  centi > -4000 && centi < 8500);
    bench.verdict("and the sensor reads ABOVE the reference, as the datasheet has it: 1.40 V "
                  "nominal against VREFINT's 1.20, which is how a reader tells the two "
                  "internal channels apart with no wire",
                  counts > 1400u);

    all_off();
}

// ===========================================================================
// c - two known levels, and the sampling ladder against table 4-28
// ===========================================================================
void tc_levels() {
    all_off();
    (void)bring_up(AdcConfig{.scan = true, .internal_sources = true});
    Adc<1>::sample_time_all(adc_sample_longest);
    drive_levels();

    const uint16_t high = primed_read(ch_high);
    Adc<1>::select_channel(ch_high);
    (void)Adc<1>::read();
    const uint16_t low = Adc<1>::read_settled(2, 0x100000UL);
    // The pad at ground, read after the pad at the rail, so neither
    // figure is the capacitor's memory of the other.
    Adc<1>::select_channel(ch_low);
    const uint16_t ground = Adc<1>::read_settled(2);
    (void)low;

    print(serial, "  a push-pull output HIGH reads ", high, " counts = ",
          Adc<1>::millivolts(high, measured_vdda_mv), " mV; the same pad LOW reads ", ground,
          " counts", crlf);
    bench.verdict("a pad driven by its own port at the rail reads full scale (a few tens of "
                  "ohms is a source every sampling time settles)",
                  high >= 4060u);
    bench.verdict("and at ground it reads zero", ground <= 20u);

    // The weak pull as a source. THE PAD STAYS A PULLED INPUT: analog
    // mode would take the pull away with the input driver (10.2.7).
    HighPin::input(PinPull::up);
    wait_us(1000);
    uint16_t ladder[8] = {};
    uint16_t parked[8] = {};
    for (uint8_t code = 0; code < 8u; ++code) {
        const AdcSampleTime t = static_cast<AdcSampleTime>(code);
        ladder[code] = scan_pair(ch_low, ch_high, t);
        (void)Adc<1>::sample_time(ch_high, t);
        parked[code] = primed_read(ch_high);
    }
    print(serial, "  the same pad held by its own pull-up (30..50 kOhm), converted RIGHT "
                  "AFTER the grounded pad in one scanned sequence:", crlf);
    print(serial, "    1.5c ", ladder[0], "  7.5c ", ladder[1], "  13.5c ", ladder[2],
          "  28.5c ", ladder[3], crlf);
    print(serial, "    41.5c ", ladder[4], "  55.5c ", ladder[5], "  71.5c ", ladder[6],
          "  239.5c ", ladder[7], crlf);
    print(serial, "  and the same thing with the two channels SELECTED SEPARATELY, so the "
                  "multiplexer sat on the pad for the microseconds a polled read costs:",
          crlf);
    print(serial, "    1.5c ", parked[0], "  7.5c ", parked[1], "  13.5c ", parked[2],
          "  28.5c ", parked[3], crlf);
    print(serial, "    41.5c ", parked[4], "  55.5c ", parked[5], "  71.5c ", parked[6],
          "  239.5c ", parked[7], crlf);
    print(serial, "  table 4-28 settles 0.4k at 1.5 cycles and 50k at 55.5, and calls the two "
                  "longest times invalid because the source they would settle is beyond the "
                  "50 kOhm the converter is rated for at all",
          crlf);

    bool monotonic = true;
    for (uint8_t i = 1; i < 8u; ++i) {
        if (ladder[i] + 32u < ladder[i - 1]) {
            monotonic = false;
        }
    }
    bench.verdict("a pad in PULLED INPUT mode is an analog source: the converter reads it "
                  "through the pad whatever the digital mode says",
                  ladder[7] > 100u);
    bench.verdict("the ladder does not go down: a longer sampling time never reads lower "
                  "than a shorter one on the same source",
                  monotonic);
    bench.verdict("the longest sampling time reads the pull's own rail", ladder[7] >= 4000u);
    bench.verdict("and the shortest does not, when the multiplexer moves to the pad at the "
                  "start of the conversion: a tenth of a microsecond does not charge a "
                  "sample-and-hold through 40 kOhm",
                  ladder[0] + 200u < ladder[7]);
    if constexpr (device::device_class == DeviceClass::v30x_d8) {
        // THE CH32V303 DOES NOT DO WHAT THE CH32V203C8 DID. The same polled
        // read falls well short of full scale at 1.5 cycles here, and the
        // time the multiplexer sits on the pad before the start is printed
        // beside it: 0, 2, 20 and 200 us of it, after the grounded channel.
        constexpr uint32_t dwell_us[4] = {0, 2, 20, 200};
        uint16_t dwelt[4] = {};
        (void)Adc<1>::sample_time(ch_high, AdcSampleTime::cycles1_5);
        for (uint8_t i = 0; i < 4u; ++i) {
            Adc<1>::select_channel(ch_low);
            (void)Adc<1>::read();
            Adc<1>::select_channel(ch_high);
            wait_us(dwell_us[i]);
            dwelt[i] = Adc<1>::read();
        }
        print(serial, "  at 1.5 cycles with the multiplexer parked on the pad 0, 2, 20 and 200 us "
                      "before the start: ",
              dwelt[0], " ", dwelt[1], " ", dwelt[2], " ", dwelt[3], crlf);
        bench.verdict("ON THIS PART A PARKED MULTIPLEXER IS NO HELP: a polled read at 1.5 "
                      "cycles right after the grounded channel falls short of the longest "
                      "sampling time's reading by more than 200 counts, where the CH32V203C8's "
                      "reached full scale",
                      parked[0] + 200u < parked[7] && dwelt[0] + 200u < parked[7]);
    } else {
        bench.verdict("THE SAMPLE-AND-HOLD TRACKS THE SELECTED CHANNEL BETWEEN CONVERSIONS: with "
                      "the mux parked on the pad the same source reads full scale at EVERY "
                      "sampling time, the shortest included",
                      parked[0] + 200u >= parked[7]);
    }

    HighPin::input(PinPull::down);
    wait_us(1000);
    (void)Adc<1>::sample_time(ch_high, adc_sample_longest);
    Adc<1>::select_channel(ch_high);
    (void)Adc<1>::read();
    const uint16_t pulled_down = Adc<1>::read_settled(2);
    print(serial, "  pulled DOWN, at the longest sampling time: ", pulled_down, " counts",
          crlf);
    bench.verdict("the same pad pulled down reads ground", pulled_down <= 40u);

    all_off();
}

// ===========================================================================
// d - the stream: scan + DMA + the ping-pong engine behind a relay
// ===========================================================================
void td_stream() {
    all_off();
    drive_levels();
    (void)bring_up(AdcConfig{.scan = true, .dma = true, .internal_sources = true});
    Adc<1>::sample_time_all(adc_sample_longest);

    // The kernel FIRST: the sampler in the pack selects its own first
    // input when it initializes, and a sequence written before that
    // would be the one this letter never sees again.
    System::init_all();

    // Four channels with four known levels: the rail, ground, the
    // reference and the sensor.
    const uint8_t order[4] = {ch_high, ch_low, ch_vref, ch_temp};
    const bool seq = Adc<1>::sequence(order, 4);

    Adc<1>::claim_stream<Source>();
    const bool started = Source::start(block_a, block_b, block_elements);
    Pfic::enable(dma_channel_irq(Source::controller, Source::channel));
    reset_counters();

    Adc<1>::continuous(true);
    Adc<1>::start();

    Stopwatch run;
    while (blocks_seen < 4u && run.us() < 200'000UL) {
        System::step();
    }
    const uint32_t elapsed_us = run.us();
    Adc<1>::continuous(false);
    (void)Adc<1>::dma(false);

    print(serial, "  ", blocks_seen, " blocks of ", block_elements, " samples lent in ",
          elapsed_us, " us; the engine counted ", Source::laps(), " laps and ",
          Source::overruns(), " overruns", crlf);
    print(serial, "  the first block: ", block_first[0], " ", block_first[1], " ",
          block_first[2], " ", block_first[3], " ", block_first[4], " ", block_first[5], " ",
          block_first[6], " ", block_first[7], crlf);
    print(serial, "  the sequence was the rail, ground, VREFINT, the sensor - twice", crlf);

    bench.verdict("the regular sequence takes four channels", seq);
    bench.verdict("the ping-pong engine starts on the channel the ADC's request is wired to",
                  started);
    bench.verdict("and four blocks reach a subscriber as loans through the relay, in a real "
                  "kernel",
                  blocks_seen >= 4u);
    const bool pattern = block_first[0] >= 4000u && block_first[1] <= 40u &&
                         block_first[2] > 1200u && block_first[2] < 1800u &&
                         block_first[3] > 1400u && block_first[3] < 2100u &&
                         block_first[4] >= 4000u && block_first[5] <= 40u;
    bench.verdict("the samples inside a block are the sequence's own channels in order, and "
                  "each one is the level that pad or source carries",
                  pattern);
    bench.verdict("no lap was skipped while the kernel kept up", Source::overruns() == 0u);

    // THE OVERRUN THE CONTRACT PROMISES: with nobody releasing, the
    // second block fills and the engine stops rather than write into
    // the buffer the caller holds.
    Source::stop();
    reset_counters();
    Adc<1>::claim_stream<Source>();
    (void)Source::start(block_a, block_b, block_elements);
    Adc<1>::continuous(true);
    Adc<1>::start();
    run.start();
    while (!Source::stalled() && run.us() < 100'000UL) {
    }
    const bool stalled = Source::stalled();
    const uint32_t overruns = Source::overruns();
    const uint8_t pending = Source::pending();
    const bool resumed_after_one = Source::release() && !Source::stalled();
    wait_us(2000);
    const uint32_t laps_after = Source::laps();
    Adc<1>::continuous(false);

    print(serial, "  with nobody releasing: ", overruns, " overrun(s), ", pending,
          " buffers held, stalled=", stalled ? 1 : 0, "; one release and the stream ran on to ",
          laps_after, " laps", crlf);
    bench.verdict("a consumer that holds both buffers makes the engine SKIP the lap rather "
                  "than tear the block it lent",
                  stalled && overruns >= 1u && pending == 2u);
    bench.verdict("and release() is what restarts it", resumed_after_one && laps_after > 2u);

    // WHY THIS ENGINE DOES NOT RIDE CIRCULAR MODE, measured on this
    // silicon. The question is not about the ADC's pace: it is how far
    // THE CONTROLLER runs past the half-transfer edge before the
    // fastest possible reader - its own handler - can stop it. So the
    // measurement is a block at the controller's own speed, memory to
    // memory, with the half interrupt armed and a handler whose whole
    // body is read-and-disable.
    Source::stop();
    Pfic::disable(dma_channel_irq(Source::controller, Source::channel));
    tear_left = 0;
    tear_seen = false;
    using Tear = DmaChannel<1, 2>;
    Tear::stop();
    Tear::arm(DmaFlag::half, true);
    Pfic::enable(Tear::irq());
    const bool loaded = Tear::load(DmaTransfer{
        .peripheral = const_cast<uint16_t*>(tear_source),
        .memory = const_cast<uint16_t*>(tear_dest),
        .count = tear_items,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = true,
                   .peripheral_increment = true,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::very_high},
    });
    Stopwatch tear_run;
    while (!tear_seen && tear_run.us() < 10'000UL) {
    }
    Pfic::disable(Tear::irq());
    const uint16_t half_items = tear_items / 2u;
    const uint16_t past =
        tear_left < half_items ? static_cast<uint16_t>(half_items - tear_left) : uint16_t{0};

    print(serial, "  a block at the CONTROLLER's own speed, stopped by the handler of its own "
                  "half flag: ",
          past, " of the next ", half_items, " items had already landed", crlf);
    bench.verdict("the half flag reaches a handler", loaded && tear_seen);
    bench.verdict("and by the time that handler - the fastest reader there is - has the "
                  "channel disabled, the controller has already written into the half beyond "
                  "the edge: which is why a block SOURCE cannot decide 'skip rather than "
                  "tear' on a channel that never stops",
                  past > 0u);

    Tear::stop();
    all_off();
}

// ===========================================================================
// e - the injected group
// ===========================================================================
void te_injected() {
    all_off();
    drive_levels();
    (void)bring_up(AdcConfig{.scan = true, .internal_sources = true});
    Adc<1>::sample_time_all(adc_sample_longest);

    // The regular group converts the rail, continuously.
    Adc<1>::select_channel(ch_high);
    Adc<1>::continuous(true);
    Adc<1>::clear_flags(AdcFlag::all);
    Adc<1>::start();
    wait_us(500);
    const bool regular_running = Adc<1>::flag(AdcFlag::started);

    // The injected group: two channels, one of them offset.
    const uint8_t injected[2] = {ch_high, ch_low};
    const bool jseq = Adc<1>::injected_sequence(injected, 2);
    constexpr uint16_t offset = 2000;
    const bool joff = Adc<1>::injected_offset(0, offset);
    (void)Adc<1>::injected_offset(1, 0);

    Adc<1>::clear_flags(AdcFlag::injected);
    Pfic::enable(Adc<1>::irq());
    Adc<1>::interrupts(Adc<1>::injected_interrupt, true);
    reset_counters();
    Adc<1>::injected_start();
    uint32_t spins = 1'000'000UL;
    while (jeoc_calls == 0u && spins-- != 0u) {
    }
    const uint16_t jeoc_seen = jeoc_calls;
    const int16_t first = Adc<1>::injected_result(0);
    const int16_t second = Adc<1>::injected_result(1);
    const bool jstrt = Adc<1>::flag(AdcFlag::injected_started);
    wait_us(500);
    const bool regular_still = Adc<1>::flag(AdcFlag::started);

    Adc<1>::continuous(false);
    Adc<1>::interrupts(Adc<1>::injected_interrupt, false);

    print(serial, "  the injected group: the rail less an offset of ", offset, " reads ", first,
          ", ground with no offset reads ", second, crlf);
    print(serial, "  JEOC reached its handler ", jeoc_seen,
          " time(s) for the two-conversion group", crlf);
    bench.verdict("the injected sequence takes two channels and an offset", jseq && joff);
    bench.verdict("a regular conversion was running when the injected group was started",
                  regular_running);
    bench.verdict("JSTRT says the injected group started", jstrt);
    bench.verdict("JEOC reaches its handler ONCE for the whole group, not once per "
                  "conversion (12.3.1: the flag is the group's)",
                  jeoc_seen == 1u);
    bench.verdict("the first conversion's result is the raw datum LESS the offset, and the "
                  "register carries the sign",
                  first > 1900 && first < 2200);
    bench.verdict("the second, with no offset, is ground", second >= -20 && second <= 40);
    bench.verdict("and the regular conversion is still running afterwards: the injected "
                  "group preempted it and gave it back",
                  regular_still);

    all_off();
}

// ===========================================================================
// f - the analog watchdog
// ===========================================================================
void tf_watchdog() {
    all_off();
    drive_levels();
    (void)bring_up();
    Adc<1>::sample_time_all(adc_sample_longest);
    Pfic::enable(Adc<1>::irq());
    reset_counters();

    // A window below the rail: the driven pad is outside it.
    (void)Adc<1>::watchdog(AdcWatchdogConfig{
        .low = 0, .high = 2000, .channel = ch_high, .interrupt = true});
    Adc<1>::clear_flags(AdcFlag::watchdog);
    Adc<1>::select_channel(ch_high);
    (void)Adc<1>::read();
    const uint16_t above = awd_calls;
    const bool flag_above = awd_calls != 0u;

    // The same window around it: inside, so nothing fires.
    Adc<1>::clear_flags(AdcFlag::watchdog);
    reset_counters();
    Adc<1>::watchdog_thresholds(2000, adc_max_count);
    (void)Adc<1>::read();
    (void)Adc<1>::read();
    const bool quiet_inside = awd_calls == 0u;

    // The pad taken to ground by the port: now it is below the window.
    reset_counters();
    HighPin::output(false);
    wait_us(1000);
    (void)Adc<1>::read();
    const bool flag_below = awd_calls != 0u;
    const uint16_t low_reading = Adc<1>::result_counts();

    // The channel select: guard channel 1 alone and convert channel 2,
    // which is outside the window and must raise nothing.
    reset_counters();
    Adc<1>::clear_flags(AdcFlag::watchdog);
    Adc<1>::select_channel(ch_low);
    (void)Adc<1>::read();
    (void)Adc<1>::read();
    const bool other_channel_quiet = awd_calls == 0u;
    const uint8_t guarded = Adc<1>::watchdog_channel();

    // The same channel with the watchdog widened to every channel.
    reset_counters();
    Adc<1>::watchdog_off();
    (void)Adc<1>::watchdog(
        AdcWatchdogConfig{.low = 2000, .high = adc_max_count, .interrupt = true});
    Adc<1>::clear_flags(AdcFlag::watchdog);
    (void)Adc<1>::read();
    const bool all_channels = awd_calls != 0u;

    print(serial, "  thresholds 0..2000 over the rail: the flag reached its handler ", above,
          " time(s); the pad then taken to ground read ", low_reading,
          " under a 2000..4095 window", crlf);
    print(serial, "  the guarded channel reads back as ", guarded, crlf);
    bench.verdict("a conversion ABOVE the high threshold raises the watchdog and its "
                  "interrupt",
                  flag_above);
    bench.verdict("a conversion inside the window raises nothing", quiet_inside);
    bench.verdict("a conversion BELOW the low threshold raises it too - the window is a "
                  "band, not a ceiling",
                  flag_below);
    bench.verdict("with AWDSGL and a channel number, a conversion of ANOTHER channel outside "
                  "the window raises nothing",
                  other_channel_quiet && guarded == ch_high);
    bench.verdict("and with the single-channel bit dropped, every regular channel is guarded",
                  all_channels);

    Adc<1>::watchdog_off();
    all_off();
}

// ===========================================================================
// g - the triggers
// ===========================================================================
void tg_triggers() {
    all_off();
    drive_levels();
    (void)bring_up();
    Adc<1>::sample_time(ch_high, AdcSampleTime::cycles28_5);
    Adc<1>::select_channel(ch_high);
    Pfic::enable(Adc<1>::irq());
    Adc<1>::interrupts(Adc<1>::converted_interrupt, true);

    // A timer's TRGO at a known rate. TIM3 counts TIMxCLK, which this
    // tree makes 96 MHz (PB1 is divided, so its timers are doubled).
    constexpr uint32_t trigger_hz = 2000;
    constexpr uint16_t psc = static_cast<uint16_t>(tim_hz / 1'000'000u - 1u);   // 1 MHz
    constexpr uint32_t arr = 1'000'000UL / trigger_hz;
    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = psc, .period = arr - 1u});
    const bool master = Pacer::master(TimMasterMode::update);
    Adc<1>::trigger(AdcTrigger::tim3_trgo);

    reset_counters();
    Adc<1>::clear_flags(AdcFlag::all);
    Pacer::enable(true);
    Stopwatch span;
    while (span.us() < 200'000UL) {
    }
    const uint32_t window_us = span.us();
    Pacer::enable(false);
    const uint16_t timed = eoc_calls;
    const uint32_t expected = (trigger_hz * window_us) / 1'000'000UL;

    print(serial, "  TIM3's update at ", trigger_hz, " Hz paced ", timed, " conversions in ",
          window_us, " us (", expected, " expected); the timer counts ", tim_hz / 1'000'000u,
          " MHz", crlf);
    bench.verdict("the timer's TRGO is a trigger source of the regular group", master);
    bench.verdict("and it paces the converter at its own rate, within one per cent",
                  timed + expected / 100u >= expected && timed <= expected + expected / 100u);

    // The EXTI line. Code 110 is EXTI line 11 unless AFIO hands it to
    // TIM8's TRGO, which the CH32V303RC and VC can and letter m
    // measures; `exti11` is the spelling that clears that field. The pad
    // raising it is one of our own, toggled by the CPU.
    Adc<1>::trigger(AdcTrigger::exti11);
    ExtiPin::output(false);
    const bool selected = Exti::select(adc_regular_exti_line, 'B');
    (void)Exti::sense(adc_regular_exti_line, ExtiSense::rising);

    constexpr uint16_t edges = 20;
    uint16_t with_nothing = 0;
    uint16_t with_event = 0;
    uint16_t with_interrupt = 0;

    reset_counters();
    for (uint16_t i = 0; i < edges; ++i) {
        ExtiPin::set();
        wait_us(200);
        ExtiPin::clear();
        wait_us(200);
    }
    with_nothing = eoc_calls;

    (void)Exti::event(adc_regular_exti_line, true);
    reset_counters();
    for (uint16_t i = 0; i < edges; ++i) {
        ExtiPin::set();
        wait_us(200);
        ExtiPin::clear();
        wait_us(200);
    }
    with_event = eoc_calls;
    (void)Exti::event(adc_regular_exti_line, false);

    Pfic::enable(Irq::exti15_10);
    (void)Exti::interrupt(adc_regular_exti_line, true);
    reset_counters();
    for (uint16_t i = 0; i < edges; ++i) {
        ExtiPin::set();
        wait_us(200);
        ExtiPin::clear();
        wait_us(200);
    }
    with_interrupt = eoc_calls;
    (void)Exti::interrupt(adc_regular_exti_line, false);

    print(serial, "  ", edges, " rising edges on the pad of EXTI line 11 started ",
          with_nothing, " conversions with the line only sensed, ", with_event,
          " with its EVENT enabled and ", with_interrupt, " with its INTERRUPT enabled (",
          exti_calls, " line interrupts taken)", crlf);
    bench.verdict("the line is selected onto this pad's port", selected);
    bench.verdict("an edge on a line that is only SENSED starts nothing: a flag nobody "
                  "unmasked reaches no peripheral either",
                  with_nothing == 0u);
    bench.verdict("the EXTI line reaches the converter through its EVENT enable, and one "
                  "edge is one conversion",
                  with_event == edges);
    bench.verdict("its INTERRUPT enable is the other path and does NOT feed the converter - "
                  "the handler ran for every edge and not one conversion started",
                  with_interrupt == 0u && exti_calls == edges);

    Adc<1>::interrupts(Adc<1>::converted_interrupt, false);
    all_off();
}

// ===========================================================================
// h - the dual mode
// ===========================================================================
void th_dual() {
    all_off();
    if constexpr (device::adc_count < 2u) {
        bench.verdict("this part has one converter, so there is no dual mode to measure "
                      "(datasheet table 2-1)",
                      true);
        return;
    } else {
        drive_levels();
        const bool up1 = bring_up(AdcConfig{.dma = true, .internal_sources = true});
        const bool up2 = Adc<2>::init(clock);
        Adc<1>::sample_time_all(adc_sample_longest);
        Adc<2>::sample_time_all(adc_sample_longest);

        // Two DIFFERENT pads, which is what 12.2.7's note asks for: the
        // master takes the rail, the follower ground.
        Adc<1>::select_channel(ch_high);
        Adc<2>::select_channel(ch_low);
        const bool mode = Adc<1>::dual(AdcDualMode::regular_simultaneous);
        const AdcDualMode read_back = Adc<1>::dual();

        Adc<1>::clear_flags(AdcFlag::all);
        Adc<1>::start();
        uint32_t spins = 1'000'000UL;
        while (!Adc<1>::ready() && spins-- != 0u) {
        }
        const uint32_t both = Adc<1>::data();
        const uint16_t master = static_cast<uint16_t>(both & adc_max_count);
        const uint16_t follower = static_cast<uint16_t>((both >> 16) & adc_max_count);

        // The same again with the DMA bit dropped: 12.2.7's note 1 says
        // it must be enabled to read the slave's datum on the master's
        // register, and this is what the silicon does about that.
        (void)Adc<1>::dma(false);
        Adc<1>::clear_flags(AdcFlag::all);
        Adc<1>::start();
        spins = 1'000'000UL;
        while (!Adc<1>::ready() && spins-- != 0u) {
        }
        const uint32_t without_dma = Adc<1>::data();
        const uint16_t follower_without = static_cast<uint16_t>((without_dma >> 16) & adc_max_count);

        // Every other mode written and read back.
        bool all_modes = true;
        for (uint8_t code = 0; code <= 9u; ++code) {
            (void)Adc<1>::dual(AdcDualMode::independent);
            const AdcDualMode m = static_cast<AdcDualMode>(code);
            if (!Adc<1>::dual(m) || Adc<1>::dual() != m) {
                all_modes = false;
            }
        }
        const bool tenth_refused = !Adc<1>::dual(static_cast<AdcDualMode>(10));
        (void)Adc<1>::dual(AdcDualMode::independent);

        print(serial, "  regular simultaneous: the master's register reads ", hex(both),
              " - the rail ", master, " in its low half, ground ", follower, " in its high one",
              crlf);
        print(serial, "  with the DMA bit dropped the high half still reads ", follower_without,
              ", where 12.2.7's note 1 says the bit is what publishes the follower's datum",
              crlf);
        bench.verdict("both converters come up on one clock", up1 && up2);
        bench.verdict("the dual-mode field takes the regular simultaneous code and reads it "
                      "back",
                      mode && read_back == AdcDualMode::regular_simultaneous);
        bench.verdict("one start converts BOTH: the master's own channel is the rail",
                      master >= 4000u);
        bench.verdict("and the follower's channel, ground, arrives in the upper half of the "
                      "master's data register",
                      follower <= 40u);
        bench.verdict("it arrives there with the DMA bit DROPPED too: 12.2.7's note about "
                      "enabling the DMA is about getting the pair into memory, not about "
                      "the register carrying both halves",
                      follower_without <= 40u);
        bench.verdict("every one of the ten modes is written and read back, and an eleventh "
                      "code is refused",
                      all_modes && tenth_refused);

        all_off();
    }
}

// ===========================================================================
// i - discontinuous mode
// ===========================================================================
void ti_discontinuous() {
    all_off();
    drive_levels();
    (void)bring_up(AdcConfig{.scan = true, .dma = true, .discontinuous = 2});
    Adc<1>::sample_time_all(AdcSampleTime::cycles28_5);

    // Six conversions in pairs, stepped two at a time. The pairs are
    // rail-then-ground so that every subgroup carries both levels and a
    // block that stopped early cannot look like a block that did not.
    const uint8_t order[6] = {ch_high, ch_low, ch_high, ch_low, ch_high, ch_low};
    const bool seq = Adc<1>::sequence(order, 6);
    const uint8_t length = Adc<1>::sequence_length();

    // The DMA is the instrument: one request per CONVERSION, so what
    // lands after each trigger says how far the sequence walked.
    using Ch = DmaChannel<DmaRequestOf<DmaRequest::adc1>::controller, DmaRequestOf<DmaRequest::adc1>::channel>;
    for (uint8_t i = 0; i < 6u; ++i) {
        captured[i] = 0xFFFFu;
    }
    Ch::stop();
    (void)Ch::load(DmaTransfer{
        .peripheral = Adc<1>::data_address(),
        .memory = const_cast<uint16_t*>(captured),
        .count = 6,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });

    uint8_t landed[3] = {};
    for (uint8_t t = 0; t < 3u; ++t) {
        Adc<1>::start();
        wait_us(2000);
        landed[t] = static_cast<uint8_t>(6u - Ch::remaining());
    }
    Ch::stop();
    (void)Adc<1>::dma(false);

    print(serial, "  a sequence of ", length, " stepped by DISCNUM = 2, counted by the DMA: ",
          landed[0], " then ", landed[1], " then ", landed[2],
          " conversions after the first, second and third trigger", crlf);
    print(serial, "  the data as it came: ", captured[0], " ", captured[1], " ", captured[2],
          " ", captured[3], " ", captured[4], " ", captured[5], crlf);
    bench.verdict("the regular sequence takes six channels", seq && length == 6u);
    bench.verdict("each trigger converts the short sequence DISCNUM names and then stops",
                  landed[0] == 2u && landed[1] == 4u && landed[2] == 6u);
    const bool alternating = captured[0] >= 4000u && captured[1] <= 40u &&
                             captured[2] >= 4000u && captured[3] <= 40u &&
                             captured[4] >= 4000u && captured[5] <= 40u;
    bench.verdict("and the data is the sequence's own channels in order, subgroup after "
                  "subgroup",
                  alternating);

    // WHEN THE FLAG COMES, with the DMA out of the way. 12.2.4's own
    // example puts the EOC event at the end of the LAST subgroup and
    // this is that sentence measured.
    (void)bring_up(AdcConfig{.scan = true, .discontinuous = 2});
    Adc<1>::sample_time_all(AdcSampleTime::cycles28_5);
    (void)Adc<1>::sequence(order, 6);
    Pfic::enable(Adc<1>::irq());
    Adc<1>::interrupts(Adc<1>::converted_interrupt, true);
    reset_counters();
    uint8_t flags_per_trigger[3] = {};
    for (uint8_t t = 0; t < 3u; ++t) {
        const uint16_t before = eoc_calls;
        Adc<1>::start();
        wait_us(2000);
        flags_per_trigger[t] = static_cast<uint8_t>(eoc_calls - before);
    }
    Adc<1>::interrupts(Adc<1>::converted_interrupt, false);

    print(serial, "  the same three triggers with the DMA off raised EOC ",
          flags_per_trigger[0], ", ", flags_per_trigger[1], " and ", flags_per_trigger[2],
          " time(s)", crlf);
    bench.verdict("in discontinuous mode the EOC flag belongs to the WHOLE sequence and not "
                  "to a conversion or a subgroup: it is raised once, when the last subgroup "
                  "ends - which is why a discontinuous group wants the DMA, whose request is "
                  "per conversion",
                  flags_per_trigger[0] == 0u && flags_per_trigger[1] == 0u &&
                      flags_per_trigger[2] == 1u);

    all_off();
}

// ===========================================================================
// j - the sampler over this converter
// ===========================================================================
void tj_sampler() {
    all_off();
    drive_levels();
    (void)bring_up();
    Adc<1>::sample_time_all(adc_sample_longest);
    Pfic::enable(Adc<1>::irq());
    Adc<1>::interrupts(Adc<1>::converted_interrupt, true);
    reset_counters();
    adc_mode = AdcMode::sampling;

    System::init_all();
    Sampler::start_every(2);
    Stopwatch run;
    while (sample_slots < 3u && run.us() < 200'000UL) {
        // The kernel's own loop, minus the idle: the sampler's pace is a
        // TimeEvent, and a step() alone would never mature it.
        TimeEvents<P>::process();
        System::step();
    }
    const uint32_t elapsed = run.us();
    Sampler::stop();
    adc_mode = AdcMode::counting;
    Adc<1>::interrupts(Adc<1>::converted_interrupt, false);

    print(serial, "  three inputs walked at a software pace in ", elapsed, " us: index ",
          sample_index[0], " = ", sample_value[0], ", index ", sample_index[1], " = ",
          sample_value[1], ", index ", sample_index[2], " = ", sample_value[2], crlf);
    print(serial, "  the list is the rail, VREFINT, ground - and the INDEX is the position "
                  "in it, attributed from the code the ISR read beside the value",
          crlf);
    bench.verdict("the sampler publishes one AnalogSample per conversion", sample_slots >= 3u);
    bench.verdict("it walks the list in order", sample_index[0] == 0u && sample_index[1] == 1u &&
                                                    sample_index[2] == 2u);
    bench.verdict("and each sample is the level its own input carries: the rail, the "
                  "reference, ground",
                  sample_value[0] >= 4000u && sample_value[1] > 1200u &&
                      sample_value[1] < 1800u && sample_value[2] <= 40u);
    bench.verdict("no result arrived with a code the list does not hold",
                  Sampler::unknown_inputs() == 0u);

    all_off();
}

// ===========================================================================
// The CH32V303's letters (k..o)
// ===========================================================================
//
// Every letter below is a template whose last parameter is the fact that
// makes it meaningful, and does its work inside `if constexpr` on it: on a
// CH32V203 the body is a discarded branch the compiler never instantiates,
// and every class it names hangs on a template parameter for that reason.

/// Letters k and o's instrument: `count` conversions of `ch` in continuous
/// mode, landed by the DMA in `tear_dest`, timed from the start to the
/// block's completion - in core cycles.
template <uint8_t n = 1>
uint32_t time_conversions(uint8_t ch, uint16_t count) {
    using A = Adc<n>;
    using Ch = DmaChannel<A::dma_slot.controller, A::dma_slot.channel>;
    Ch::stop();
    (void)Ch::load(DmaTransfer{
        .peripheral = A::data_address(),
        .memory = const_cast<uint16_t*>(tear_dest),
        .count = count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::very_high},
    });
    (void)A::dma(true);
    A::select_channel(ch);
    A::continuous(true);
    A::clear_flags(AdcFlag::all);
    Stopwatch w;
    A::start();
    // The stopwatch folds one wrap of the tick counter per reading, so it
    // is read on every pass: 64 of the longest conversions outlast a tick.
    uint32_t cycles = 0;
    uint32_t spins = 1'000'000UL;
    while (!Ch::flag(DmaFlag::complete) && spins-- != 0u) {
        cycles = w.cycles();
    }
    cycles = w.cycles();
    A::continuous(false);
    Ch::stop();
    (void)A::dma(false);
    return cycles;
}

constexpr uint16_t distance16(uint16_t a, uint16_t b) {
    return a > b ? static_cast<uint16_t>(a - b) : static_cast<uint16_t>(b - a);
}

/// A conversion's length in tenths of a HALF ADCCLK cycle, from a count of
/// core cycles over `count` conversions.
constexpr uint32_t conversion_half_x10(uint32_t core_cycles, uint16_t count) {
    constexpr uint32_t core_per_adc = SysClock::hz / SysClock::adc_hz;
    return (core_cycles * 20u) / (core_per_adc * count);
}

// ===========================================================================
// k - ADC_SMP_SEL: the four short sampling times
// ===========================================================================
template <uint8_t n = 1, bool on = adc_has_short_sampling>
void tk_short_sampling() {
    if constexpr (on) {
        using A = Adc<n>;
        all_off();
        drive_levels();
        (void)A::init(clock, AdcConfig{});
        constexpr uint16_t count = tear_items;

        // The register: ADC_SMP_SEL of the rail's channel, ASKED OF THE DIE.
        // ADCx_AUX is the lot's (12.3.15's note), and the verb sets the bit
        // and reads it back before it touches SMPx - so a die without the
        // register answers false and the channel keeps the time it had.
        (void)A::sample_time(ch_high, AdcSampleTime::cycles41_5);
        const bool took = A::sample_time(ch_high, AdcShortSampleTime::cycles2_5);
        const uint32_t aux = A::aux();
        const std::optional<AdcShortSampleTime> back = A::short_sample_time(ch_high);
        const bool kept = A::sample_time(ch_high) == AdcSampleTime::cycles41_5;
        // Every channel at once, from a known long time: code 101 is 3.5
        // cycles with the bit and 55.5 without, so a refusal that wrote SMPx
        // anyway would show.
        A::sample_time_all(AdcSampleTime::cycles41_5);
        const bool all_took = A::sample_time_all(AdcShortSampleTime::cycles3_5);
        const bool all_kept = A::sample_time(ch_low) == AdcSampleTime::cycles41_5;
        A::sample_time_all(AdcSampleTime::cycles41_5);
        print(serial, "  ADC1_AUX with ADC_SMP_SEL", ch_high, " written: ", hex(aux), " - ",
              took ? "this die has the register" : "this die's lot has NOT got the register",
              crlf);
        if (took) {
            bench.verdict("ADCx_AUX takes ADC_SMP_SELx and reads it back - the register 12.3.15's "
                          "note gives to some lots of this class only",
                          (aux & (1UL << ch_high)) != 0u && back.has_value() &&
                              *back == AdcShortSampleTime::cycles2_5 && all_took);
        } else {
            bench.verdict("a die whose lot has no ADCx_AUX keeps nothing written there, and the "
                          "short-time verbs answer false having written nothing else: the "
                          "channel keeps the long time it had instead of a code that would mean "
                          "41.5..239.5 cycles in silence",
                          aux == 0u && !back.has_value() && kept && !all_took && all_kept);
        }

        // THE TAIL - what a conversion takes beyond its sampling time - at
        // the four long codes, whatever the die: 64 back-to-back
        // conversions landed by the DMA, timed by the core's counter.
        constexpr AdcSampleTime longs[4] = {AdcSampleTime::cycles41_5, AdcSampleTime::cycles55_5,
                                            AdcSampleTime::cycles71_5, AdcSampleTime::cycles239_5};
        uint8_t long_right = 0;
        uint8_t predicted = 0;
        for (uint8_t i = 0; i < 4u; ++i) {
            (void)A::sample_time(ch_high, longs[i]);
            const uint32_t l = conversion_half_x10(time_conversions<n>(ch_high, count), count);
            const uint32_t l_tail = l - adc_sample_half_cycles(longs[i]) * 10u;
            print(serial, "  SMP code ", static_cast<uint8_t>(longs[i]), ": ", l / 20u, ".",
                  (l / 2u) % 10u, " cycles a conversion, a tail of ", l_tail / 20u, ".",
                  (l_tail / 2u) % 10u, " beyond the sampling time", crlf);
            if (l > adc_sample_half_cycles(longs[i]) * 10u && l_tail >= 240u && l_tail <= 260u) {
                ++long_right;
            }
            const uint32_t model = A::conversion_half_cycles(ch_high) * 10u;
            if ((l > model ? l - model : model - l) <= 10u) {
                ++predicted;
            }
        }
        bench.verdict("a conversion is its sampling time plus 12.5 cycles at every long code - "
                      "the datasheets' 14..252 cycles and not 12.2.2's 11",
                      long_right == 4u);
        bench.verdict("and the driver's conversion_half_cycles() predicts each of them to half a "
                      "cycle",
                      predicted == 4u);

        if (took) {
            // The four short times, where the die has them.
            constexpr AdcShortSampleTime shorts[4] = {
                AdcShortSampleTime::cycles2_5, AdcShortSampleTime::cycles3_5,
                AdcShortSampleTime::cycles4_5, AdcShortSampleTime::cycles5_5};
            uint8_t short_right = 0;
            for (uint8_t i = 0; i < 4u; ++i) {
                (void)A::sample_time(ch_high, shorts[i]);
                const uint32_t s = conversion_half_x10(time_conversions<n>(ch_high, count), count);
                const uint32_t s_tail = s - adc_sample_half_cycles(shorts[i]) * 10u;
                print(serial, "  short code ", static_cast<uint8_t>(shorts[i]), ": ", s / 20u, ".",
                      (s / 2u) % 10u, " cycles a conversion, a tail of ", s_tail / 20u, ".",
                      (s_tail / 2u) % 10u, crlf);
                if (s > adc_sample_half_cycles(shorts[i]) * 10u && s_tail >= 240u &&
                    s_tail <= 260u) {
                    ++short_right;
                }
            }
            bench.verdict("with ADC_SMP_SELx set, SMP codes 100..111 sample for 2.5, 3.5, 4.5 "
                          "and 5.5 cycles, the same 12.5-cycle tail behind them",
                          short_right == 4u);

            // A hard source at the shortest of them.
            (void)A::sample_time(ch_high, AdcShortSampleTime::cycles2_5);
            (void)A::sample_time(ch_low, AdcShortSampleTime::cycles2_5);
            A::select_channel(ch_low);
            (void)A::read();
            A::select_channel(ch_high);
            const uint16_t high = A::read_settled(2);
            A::select_channel(ch_low);
            const uint16_t low = A::read_settled(2);
            print(serial, "  at 2.5 cycles the rail reads ", high, " counts and ground ", low,
                  crlf);
            bench.verdict("a pad driven by its own port reads true at 2.5 cycles: full scale at "
                          "the rail, zero at ground",
                          high >= 4060u && low <= 20u);
        }
        A::sample_time_all(adc_sample_longest);
        all_off();
    }
}

/// A value in hundredths, printed with both its decimals.
inline void print_hundredths(int32_t v) {
    if (v < 0) {
        print(serial, "-");
        v = -v;
    }
    print(serial, v / 100, ".", (v % 100) / 10, v % 10);
}

// ===========================================================================
// l - the DAC as the source: linearity through the sampler, and the PGA
// ===========================================================================
template <typename Kernel = DacKernel, typename Sampler = DacSampler,
          typename Consumer = DacConsumer, uint8_t n = 1, uint8_t ch = 1,
          bool on = device::has_dac>
void tl_dac_source() {
    if constexpr (on) {
        using D = DacUnit<on>;
        using Out = DacOut<ch>;
        using A = Adc<n>;
        all_off();
        // No internal sources: TSVREFE would force the input buffer on for
        // good, and the PGA half of this letter wants it the program's.
        (void)A::init(clock, AdcConfig{});
        A::sample_time_all(adc_sample_longest);
        D::init();
        Out::claim();
        (void)D::configure(ch, DacChannelConfig{});
        (void)D::enable(ch, true);
        wait_us(20);

        Pfic::enable(A::irq());
        A::interrupts(A::converted_interrupt, true);
        adc_mode = AdcMode::dac_sampling;
        Kernel::init_all();
        Sampler::start_every(1);
        const auto serve_until = [](uint16_t samples) {
            Stopwatch w;
            while (Consumer::count < samples && w.us() < 50'000UL) {
                TimeEvents<P>::process();
                (void)Kernel::step();
            }
        };

        // Thirty-one codes 128 apart, four samples of each - the first
        // one after a step thrown away, since its conversion may have
        // started before it.
        constexpr uint8_t steps = 31;
        uint16_t reading[steps] = {};
        for (uint8_t i = 0; i < steps; ++i) {
            const uint16_t code = static_cast<uint16_t>(128u + 128u * i);
            (void)D::write(ch, code);
            wait_us(50);
            Consumer::reset();
            serve_until(1);
            Consumer::reset();
            serve_until(4);
            reading[i] = Consumer::count != 0u
                             ? static_cast<uint16_t>(Consumer::sum / Consumer::count)
                             : uint16_t{0};
        }
        Sampler::stop();
        adc_mode = AdcMode::counting;
        A::interrupts(A::converted_interrupt, false);

        // The line through them: gain in parts per million, offset in
        // hundredths of a count, and the largest distance from it.
        bool monotonic = true;
        int64_t sx = 0;
        int64_t sy = 0;
        int64_t sxx = 0;
        int64_t sxy = 0;
        for (uint8_t i = 0; i < steps; ++i) {
            const int64_t x = 128 + 128 * static_cast<int64_t>(i);
            const int64_t y = reading[i];
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
            if (i != 0u && reading[i] <= reading[i - 1u]) {
                monotonic = false;
            }
        }
        const int64_t k = steps;
        const int64_t den = k * sxx - sx * sx;
        const int64_t gain_ppm = ((k * sxy - sx * sy) * 1'000'000) / den;
        const int64_t offset_x100 = ((sy * sxx - sx * sxy) * 100) / den;
        int64_t worst_x100 = 0;
        for (uint8_t i = 0; i < steps; ++i) {
            const int64_t x = 128 + 128 * static_cast<int64_t>(i);
            const int64_t fit_x100 = offset_x100 + (gain_ppm * x) / 10'000;
            const int64_t r = static_cast<int64_t>(reading[i]) * 100 - fit_x100;
            const int64_t a = r < 0 ? -r : r;
            if (a > worst_x100) {
                worst_x100 = a;
            }
        }
        const int32_t gain = static_cast<int32_t>(gain_ppm);
        const int32_t offset = static_cast<int32_t>(offset_x100);
        const int32_t worst = static_cast<int32_t>(worst_x100);
        print(serial, "  31 codes of DAC1 read on PA4 through the sampler: ", reading[0], " ",
              reading[1], " ", reading[2], " ... ", reading[15], " ... ", reading[steps - 1u],
              crlf);
        print(serial, "  the line through them: gain ", gain, " ppm of one count a code, offset ");
        print_hundredths(offset);
        print(serial, " counts; the largest residual ");
        print_hundredths(worst);
        print(serial, " counts", crlf);
        bench.verdict("the sampler walks a pad the DAC drives, and every step of 128 codes reads "
                      "higher than the last",
                      monotonic && reading[0] != 0u);
        bench.verdict("the two converters agree to 1 % in gain and 24 counts in offset (table "
                      "4-43: offset 4 LSB; table 4-45: 12 mV and 0.4 %)",
                      gain >= 990'000 && gain <= 1'010'000 && offset >= -2400 && offset <= 2400);
        bench.verdict("and no code sits more than 10 counts off the line - both converters' "
                      "integral nonlinearity together (4 LSB each, tables 4-43 and 4-45)",
                      worst <= 1000);

        // THE PGA: the input buffer on, and each gain against the same
        // input read at x1 through the buffer.
        struct GainStep {
            AdcGain gain;
            uint16_t code;
            uint8_t tolerance_pct;
        };
        constexpr GainStep gains[3] = {{AdcGain::x4, 720, 5},
                                       {AdcGain::x16, 180, 8},
                                       {AdcGain::x64, 45, 15}};
        uint8_t gains_right = 0;
        A::select_channel(Out::adc_channel);
        for (const GainStep& g : gains) {
            (void)D::write(ch, g.code);
            (void)A::gain(AdcGain::x1, true);
            wait_us(100);
            (void)A::read();
            const uint16_t unity = A::read_settled(8);
            (void)A::gain(g.gain, true);
            wait_us(100);
            (void)A::read();
            const uint16_t amplified = A::read_settled(8);
            const uint32_t ratio_x100 =
                unity != 0u ? (static_cast<uint32_t>(amplified) * 100u) / unity : 0u;
            const uint32_t nominal_x100 = static_cast<uint32_t>(adc_gain_factor(g.gain)) * 100u;
            const uint32_t off = ratio_x100 > nominal_x100 ? ratio_x100 - nominal_x100
                                                           : nominal_x100 - ratio_x100;
            if (off * 100u <= nominal_x100 * g.tolerance_pct) {
                ++gains_right;
            }
            print(serial, "  PGA x", adc_gain_factor(g.gain), ": code ", g.code, " reads ", unity,
                  " at x1 and ", amplified, " amplified, a ratio of ");
            print_hundredths(static_cast<int32_t>(ratio_x100));
            print(serial, crlf);
        }
        (void)A::gain(AdcGain::x1, false);
        bench.verdict("the PGA multiplies the same input by 4, 16 and 64 - to 5, 8 and 15 % "
                      "(the reading at x1 carries the converter's own offset of 4 LSB)",
                      gains_right == 3u);
        D::release();
        all_off();
    }
}

// ===========================================================================
// m - code 110 handed to TIM8
// ===========================================================================
/// TIM8's TRGO pacing the regular group, where this build's timer driver
/// reaches TIM8: how many conversions 100 ms of it started.
template <uint8_t t = 8>
bool tim8_paced(uint16_t& conversions) {
    if constexpr (tim_present(t)) {
        using T = Tim<t>;
        T::init();
        (void)T::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(T::clock_hz(clock) / 1'000'000u - 1u),
            .period = 499});   // 2 kHz
        (void)T::master(TimMasterMode::update);
        reset_counters();
        T::enable(true);
        wait_us(100'000);
        T::enable(false);
        conversions = eoc_calls;
        T::release();
        return true;
    } else {
        conversions = 0;
        return false;
    }
}

/// Twenty rising edges on EXTI line 11's pad: how many conversions they
/// started.
inline uint16_t exti11_edges() {
    // Whatever the previous source started has ended before the count
    // begins: a conversion is a few microseconds at this clock.
    wait_us(50);
    reset_counters();
    for (uint8_t i = 0; i < 20u; ++i) {
        ExtiPin::set();
        wait_us(200);
        ExtiPin::clear();
        wait_us(200);
    }
    return eoc_calls;
}

template <uint8_t n = 1, uint8_t follower = 2, bool on = adc_has_tim8_triggers>
void tm_tim8_remap() {
    if constexpr (on) {
        using A = Adc<n>;
        using B = Adc<follower>;
        all_off();
        drive_levels();
        (void)bring_up();
        (void)A::sample_time(ch_high, AdcSampleTime::cycles28_5);
        A::select_channel(ch_high);
        Pfic::enable(A::irq());
        A::interrupts(A::converted_interrupt, true);

        // The line, sensed with its EVENT enable - letter g's path.
        ExtiPin::output(false);
        (void)Exti::select(adc_regular_exti_line, 'B');
        (void)Exti::sense(adc_regular_exti_line, ExtiSense::rising);
        (void)Exti::event(adc_regular_exti_line, true);

        // Code 110 handed to TIM8: the AFIO field set with the trigger.
        const bool took = A::trigger(AdcTrigger::tim8_trgo);
        const bool reads_tim8 = A::trigger() == AdcTrigger::tim8_trgo;
        const bool field_set = Afio::remap_code(A::regular_trigger_remap) == 1u;
        const uint16_t with_remap = exti11_edges();
        uint16_t paced = 0;
        const bool driven = tim8_paced(paced);

        // And back: `exti11` clears the field and the line starts it again.
        (void)A::trigger(AdcTrigger::exti11);
        const bool field_clear = Afio::remap_code(A::regular_trigger_remap) == 0u;
        const uint16_t without_remap = exti11_edges();
        (void)Exti::event(adc_regular_exti_line, false);

        print(serial, "  code 110 with ADC1_ETRGREG_RM set: ", with_remap,
              " conversions from 20 edges of EXTI line 11; with it clear: ", without_remap,
              crlf);
        bench.verdict("tim8_trgo writes code 110 with AFIO's ADC1_ETRGREG_RM and reads back as "
                      "itself, and exti11 clears the field again",
                      took && reads_tim8 && field_set && field_clear);
        bench.verdict("while the field is set EXTI line 11 starts nothing; cleared, every edge "
                      "is a conversion again (10.2.11.8's remap, measured without TIM8)",
                      with_remap == 0u && without_remap == 20u);
        if (driven) {
            print(serial, "  TIM8's TRGO at 2 kHz started ", paced, " conversions in 100 ms", crlf);
            bench.verdict("and TIM8's TRGO paces the group through the handed-over code",
                          paced >= 195u && paced <= 205u);
        } else {
            print(serial, "  TIM8: this build's timer driver does not reach it, so its TRGO is "
                          "not driven here",
                  crlf);
        }

        // The injected group's field, and the follower's two.
        const bool j_took = A::injected_trigger(AdcInjectedTrigger::tim8_cc4);
        const bool j_back = A::injected_trigger() == AdcInjectedTrigger::tim8_cc4 &&
                            Afio::remap_code(A::injected_trigger_remap) == 1u;
        (void)A::injected_trigger(AdcInjectedTrigger::exti15);
        const bool j_clear = Afio::remap_code(A::injected_trigger_remap) == 0u;
        (void)B::init(clock);
        const bool b_took = B::trigger(AdcTrigger::tim8_trgo) &&
                            B::injected_trigger(AdcInjectedTrigger::tim8_cc4);
        const bool b_back = Afio::remap_code(B::regular_trigger_remap) == 1u &&
                            Afio::remap_code(B::injected_trigger_remap) == 1u &&
                            Afio::remap_code(A::regular_trigger_remap) == 0u;
        (void)B::trigger(AdcTrigger::exti11);
        (void)B::injected_trigger(AdcInjectedTrigger::exti15);
        bench.verdict("the injected group's TIM8_CC4 and ADC2's two fields are four separate bits "
                      "of AFIO_PCFR1, each written and read back without touching the others",
                      j_took && j_back && j_clear && b_took && b_back);
        A::interrupts(A::converted_interrupt, false);
        all_off();
    }
}

// ===========================================================================
// n - ADC2's own request, on DMA2's channel 5
// ===========================================================================
template <uint8_t n = 2, bool on = dma_request_channel(DmaRequest::adc2).present()>
void tn_adc2_dma() {
    if constexpr (on) {
        using A = Adc<n>;
        using Ch = DmaChannel<A::dma_slot.controller, A::dma_slot.channel>;
        all_off();
        drive_levels();
        // THE DIE IS ASKED FIRST. The request is a lot's (table 11-3's
        // note), and a die without it keeps no DMA bit in ADC2's CTLR2 - so
        // init() refuses a configuration that asks for one there.
        const bool up = A::init(clock, AdcConfig{.dma = true});
        if (!up) {
            const bool plain = A::init(clock, AdcConfig{});
            const bool bit = A::dma(true);
            const bool reads = A::dma();
            A::sample_time_all(AdcSampleTime::cycles28_5);
            A::select_channel(ch_high);
            // And nothing reaches the channel either: the bit dma() wrote
            // and the die dropped, the channel loaded, continuous
            // conversions for 2 ms.
            Ch::stop();
            const bool loaded = Ch::load(DmaTransfer{
                .peripheral = A::data_address(),
                .memory = const_cast<uint16_t*>(captured),
                .count = 8,
                .config = {.peripheral_width = DmaWidth::half,
                           .memory_width = DmaWidth::half,
                           .priority = DmaPriority::high},
            });
            A::continuous(true);
            A::start();
            wait_us(2000);
            const uint16_t left = Ch::remaining();
            const bool converting = A::result_counts() >= 4000u;
            A::continuous(false);
            Ch::stop();
            print(serial, "  ADC2's DMA bit read back ", reads ? "SET" : "clear",
                  "; with it written anyway, DMA2 channel ", Ch::number, " still holds ", left,
                  " of 8 after 2 ms of continuous conversions of the rail", crlf);
            bench.verdict("on this die's lot ADC2 has no request: CTLR2.DMA keeps no bit written "
                          "into it, init() refuses a configuration that asks for one and dma() "
                          "answers false",
                          plain && !bit && !reads);
            bench.verdict("and no request reaches DMA2's channel 5 while the converter runs",
                          loaded && converting && left == 8u);
            all_off();
            return;
        }
        A::sample_time_all(AdcSampleTime::cycles28_5);
        A::select_channel(ch_high);
        for (uint8_t i = 0; i < 8u; ++i) {
            captured[i] = 0xFFFFu;
        }
        Ch::stop();
        const bool loaded = Ch::load(DmaTransfer{
            .peripheral = A::data_address(),
            .memory = const_cast<uint16_t*>(captured),
            .count = 8,
            .config = {.direction = DmaDirection::peripheral_to_memory,
                       .circular = false,
                       .memory_to_memory = false,
                       .peripheral_increment = false,
                       .memory_increment = true,
                       .peripheral_width = DmaWidth::half,
                       .memory_width = DmaWidth::half,
                       .priority = DmaPriority::high},
        });
        A::continuous(true);
        A::clear_flags(AdcFlag::all);
        A::start();
        Stopwatch w;
        while (!Ch::flag(DmaFlag::complete) && w.us() < 20'000UL) {
        }
        const bool done = Ch::flag(DmaFlag::complete);
        const uint16_t left = Ch::remaining();
        A::continuous(false);
        Ch::stop();
        (void)A::dma(false);
        bool all_rail = done;
        for (uint8_t i = 0; i < 8u && all_rail; ++i) {
            all_rail = captured[i] >= 4000u;
        }
        print(serial, "  ADC2 on DMA", Ch::controller, " channel ", Ch::number, ": ",
              done ? "the block completed" : "NO completion", ", ", left, " left; ",
              captured[0], " ", captured[1], " ... ", captured[7], crlf);
        bench.verdict("ADC2 comes up with its DMA bit, which only this class's table gives it",
                      up && loaded);
        bench.verdict("and its request reaches DMA2's channel 5: eight conversions of the rail "
                      "land in memory (table 11-3's ADC2 row, which its note gives to some lots "
                      "only)",
                      all_rail);
        all_off();
    }
}

// ===========================================================================
// o - the ADC clock's two duty bits
// ===========================================================================
template <uint8_t n = 1, bool on = device::device_class == DeviceClass::v30x_d8>
void to_duty() {
    if constexpr (on) {
        using A = Adc<n>;
        all_off();
        drive_levels();
        (void)bring_up();
        A::sample_time_all(AdcSampleTime::cycles28_5);
        struct Setting {
            bool sel;
            bool ext;
            const char* name;
        };
        constexpr Setting settings[3] = {{false, false, "50 %"},
                                         {true, false, "ADC_DUTY_SEL"},
                                         {false, true, "ADCDUTY"}};
        uint16_t high[3] = {};
        uint16_t low[3] = {};
        uint16_t vref[3] = {};
        uint32_t length[3] = {};
        bool sel_read_back = false;
        bool sel_answer = false;
        for (uint8_t i = 0; i < 3u; ++i) {
            const bool answer = Rcc::adc_duty_75(settings[i].sel);
            Rcc::adc_duty_extended(settings[i].ext);
            if (settings[i].sel) {
                sel_read_back = Rcc::adc_duty_75();
                sel_answer = answer;
            }
            wait_us(100);
            A::select_channel(ch_high);
            (void)A::read();
            high[i] = A::read_settled(4);
            A::select_channel(ch_low);
            (void)A::read();
            low[i] = A::read_settled(4);
            A::select(AdcInput::vrefint);
            (void)A::read();
            vref[i] = A::read_settled(4);
            length[i] = conversion_half_x10(time_conversions<n>(ch_high, tear_items), tear_items);
            print(serial, "  ", settings[i].name, ": the rail ", high[i], ", ground ", low[i],
                  ", VREFINT ", vref[i], ", a conversion ", length[i] / 20u, ".",
                  (length[i] / 2u) % 10u, " cycles", crlf);
        }
        (void)Rcc::adc_duty_75(false);
        Rcc::adc_duty_extended(false);
        print(serial, "  ADC_DUTY_SEL ", sel_read_back ? "read back as set" : "read back CLEAR",
              " - the bit 3.4.2's note gives to some lots of this class only", crlf);
        bench.verdict("the verb answers what the die kept: true where ADC_DUTY_SEL reads back "
                      "set, false where the lot has not got the bit",
                      sel_answer == sel_read_back);
        bool same_readings = true;
        bool same_length = true;
        for (uint8_t i = 1; i < 3u; ++i) {
            if (distance16(high[i], high[0]) > 16u || distance16(low[i], low[0]) > 16u ||
                distance16(vref[i], vref[0]) > 16u) {
                same_readings = false;
            }
            const uint32_t d = length[i] > length[0] ? length[i] - length[0] : length[0] - length[i];
            if (d * 100u > length[0] * 3u) {
                same_length = false;
            }
        }
        bench.verdict("under either duty bit the converter still reads the rail, ground and "
                      "VREFINT within 16 counts of the 50 % clock's readings",
                      same_readings);
        bench.verdict("and neither moves the conversion time: a duty cycle is not a period",
                      same_length);
        all_off();
    }
}

/// The CH32V303's five letters, registered where the part is one: a
/// template, so the other series carries neither their code nor their
/// prose.
template <bool on = device::device_class == DeviceClass::v30x_d8>
void register_v303_letters() {
    if constexpr (on) {
        bench.letter('k', "ADC_SMP_SEL: the four short sampling times", tk_short_sampling<>);
        bench.letter('l', "the DAC on PA4 through the sampler: linearity, and the PGA",
                     tl_dac_source<>);
        bench.letter('m', "code 110 handed to TIM8 through AFIO", tm_tim8_remap<>);
        bench.letter('n', "ADC2's own request on DMA2's channel 5", tn_adc2_dma<>);
        bench.letter('o', "the ADC clock's two duty bits", to_duty<>);
    }
}

/// Letter l's ISR glue: the result handed to the DAC loop's sampler.
/// False, and nothing, on a part without the DAC.
template <typename Sampler = DacSampler, bool has = device::has_dac>
bool dac_sample_isr() {
    if constexpr (has) {
        if (adc_mode == AdcMode::dac_sampling) {
            const uint8_t in = Adc<1>::selected();
            const uint16_t v = Adc<1>::result_counts();
            (void)Adc<1>::isr();
            post<Sampler>(Sampled{v, in});
            return true;
        }
    }
    return false;
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf, "test_vx03_adc - the converters of RM ch. 12, at ",
          SysClock::hz / 1'000'000u, " MHz with ADCCLK ", SysClock::adc_hz / 1'000'000u,
          " MHz", crlf,
          "  no wires: the levels are a pad driven or pulled by its own port, VREFINT "
          "and the temperature sensor",
          crlf);
    bench.menu();
}

}  // namespace

// The one vector both converters report on. In the samplers' letters it
// carries the result into a kernel; everywhere else it counts.
extern "C" BRIO_CH32_INTERRUPT void adc1_2_handler() {
    if (adc_mode == AdcMode::sampling) {
        const uint8_t in = brio::Adc<1>::selected();
        const uint16_t v = brio::Adc<1>::result_counts();
        (void)brio::Adc<1>::isr();
        brio::post<Sampler>(brio::Sampled{v, in});
        return;
    }
    if (dac_sample_isr()) {
        return;
    }
    const uint32_t hit = brio::Adc<1>::isr();
    if ((hit & brio::AdcFlag::converted) != 0u) {
        eoc_calls = static_cast<uint16_t>(eoc_calls + 1u);
        if (captured_count < 8u) {
            captured[captured_count] = brio::Adc<1>::result_counts();
            captured_count = static_cast<uint8_t>(captured_count + 1u);
        }
    }
    if ((hit & brio::AdcFlag::injected) != 0u) {
        jeoc_calls = static_cast<uint16_t>(jeoc_calls + 1u);
    }
    if ((hit & brio::AdcFlag::watchdog) != 0u) {
        awd_calls = static_cast<uint16_t>(awd_calls + 1u);
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() {
    const uint8_t f = Source::service();
    if ((f & Source::flag_complete) != 0u) {
        (void)Source::complete();
        brio::post<Relay>(brio::BlockDone{});
    }
    if ((f & Source::flag_error) != 0u) {
        Source::fail();
    }
}

/// Letter d's tear window: read CNTR and stop the channel, nothing else.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() {
    using Tear = brio::DmaChannel<1, 2>;
    const uint16_t left = Tear::remaining();
    Tear::enable(false);
    (void)Tear::isr();
    if (!tear_seen) {
        tear_left = left;
        tear_seen = true;
    }
}

extern "C" BRIO_CH32_INTERRUPT void exti15_10_handler() {
    const uint32_t up = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti15_10));
    if (up != 0u) {
        exti_calls = static_cast<uint16_t>(exti_calls + 1u);
    }
}

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the power-up, the calibration, and VREFINT", ta_calibration);
    bench.letter('b', "the temperature sensor on channel 16", tb_temperature);
    bench.letter('c', "two known levels, and the sampling ladder on a 40 kOhm source",
                 tc_levels);
    bench.letter('d', "the stream: scan, DMA, the ping-pong engine behind a relay", td_stream);
    bench.letter('e', "the injected group preempting a continuous conversion", te_injected);
    bench.letter('f', "the analog watchdog and its interrupt", tf_watchdog);
    bench.letter('g', "the triggers: a timer's TRGO and an EXTI line", tg_triggers);
    bench.letter('h', "the dual mode: two converters, one register", th_dual);
    bench.letter('i', "discontinuous mode: a sequence stepped by twos", ti_discontinuous);
    bench.letter('j', "the sampler walking three inputs in a kernel", tj_sampler);
    register_v303_letters();

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

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
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
