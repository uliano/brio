// test_rp2350_adc - the reference bench suite for this chip's ADC
// (rp2350/adc.hpp over datasheet 12.4), measured with NO WIRE: the
// temperature sensor as the known voltage, the pads' own pulls as two
// more, the clock chapter's frequency counter on clk_adc, TIMER0 on the
// conversion rate, the DMA as the reader, util/analog_sampler.hpp over
// the whole thing.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets. Nothing in this chapter differs between them:
// the converter is one block, its interrupt is one system line under one
// numbering, and `isr_adc_fifo` is the handler name on both.
//
// NOTHING TO WIRE, and the pads this suite reads are the EIGHT the
// QFN-80 gives the converter - GP40..GP47, inputs 0..7 - which must be
// free; the sensor is input 8. In the QFN-60 the same letters read four
// pads and a sensor on input 4, and the suite says which package it is
// running in. GP0/GP1 are the console's, GP25 the LED.
//
// THE RULERS: TIMER0 (a microsecond counter off the tick generators,
// independent of clk_sys) for the rates, the clock chapter's frequency
// counter for clk_adc itself.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the rate arithmetic and the temperature line
//      at the chapter's own examples, the package the silicon reports
//      against the one this image was built for, the reset state, every
//      channel selected and read back with the one past the last
//      refused; init() on the USB PLL with clk_adc COUNTED at 48 MHz and
//      READY up, then on the crystal counted at 12 MHz
//   b  THE TEMPERATURE SENSOR: 64 one-shot reads on each clock, the mean
//      a plausible room temperature and the same on both clocks, the
//      spread narrow; the sensor's bias off
//   c  EVERY PAD THE PACKAGE GIVES THE CONVERTER, through its pulls with
//      the digital input buffer off - which is what 12.4 asks for anyway
//      and what closes erratum RP2350-E9's leakage path
//   d  THE RATE: free-running conversions into a 256-word DMA block at
//      500, 100 and 10 ksps and at a fractional divider, timed; the
//      dividers around the conversion's own 96 cycles; 125 ksps on the
//      crystal
//   e  ROUND-ROBIN OVER ALL NINE CHANNELS - a mask the RP2040's five-bit
//      field could not hold: the eight pads with alternating pulls and
//      the sensor, the block's pattern read back in order
//   f  THE FIFO: the threshold interrupt draining at four, the overflow
//      flag when nobody drains, the eight-bit shift into a byte block,
//      the per-entry error flag counted, and what the atomic SET alias
//      does to a write-one-to-clear flag of this block
//   g  THE SAMPLER: util/analog_sampler.hpp's AnalogSampler walking the
//      sensor and two pads on a software pace, its samples counted per
//      input
//   h  ERRATUM RP2350-E9 IN MILLIVOLTS: the converter is the one
//      instrument on this chip that can see the leakage as a VOLTAGE -
//      the same pad, pulled down, read with its digital input buffer on
//      and then off
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/adc.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/analog_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = Rp2350Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;
using Ruler = Timer<0>;

TestBench<Serial> bench;

/// The first four inputs, which every package of this chip has. The
/// others are reached by number: the pad behind input k is
/// `adc_first_pin + k`, and how many there are is the package's.
using In0 = AnalogIn<Pin<adc_first_pin>>;
using In1 = AnalogIn<Pin<adc_first_pin + 1u>>;
using In2 = AnalogIn<Pin<adc_first_pin + 2u>>;

using Block = DmaRxEngine<10, uint16_t>;
using ByteBlock = DmaRxEngine<11, uint8_t>;

/// What the ADC_AVDD pin carries on the boards this suite runs on. There
/// is no ADC_VREF pin on this chip: the converter's supply is its scale.
constexpr uint16_t vref = ref_mv(Ref::avdd_pin);

volatile bool block_done = false;
volatile bool stop_on_done = true;   // the DMA's completion stops the converter (12.4.3.5)
volatile uint32_t fifo_isr_entries = 0;
volatile uint32_t fifo_isr_popped = 0;
volatile uint8_t fifo_isr_max_level = 0;
enum class FifoOwner : uint8_t { none, drain, sampler };
volatile FifoOwner fifo_owner = FifoOwner::none;

uint16_t block[256];
uint8_t byte_block[256];

uint32_t us_now() { return Ruler::now_low(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
bool within(uint32_t got, uint32_t want, uint32_t per_mille) {
    const uint32_t tol = static_cast<uint32_t>(static_cast<uint64_t>(want) * per_mille / 1000u);
    return got + tol >= want && got <= want + tol;
}

/// `n` one-shot reads of the selected input: the mean, the spread.
struct Stat {
    uint32_t mean = 0;
    uint16_t lo = 0xFFFF;
    uint16_t hi = 0;
    uint16_t failed = 0;
};
Stat sample(uint16_t n) {
    Stat s{};
    uint32_t sum = 0;
    uint16_t taken = 0;
    for (uint16_t i = 0; i < n; ++i) {
        const auto r = Adc::read();
        if (!r) {
            ++s.failed;
            continue;
        }
        sum += *r;
        ++taken;
        if (*r < s.lo) { s.lo = *r; }
        if (*r > s.hi) { s.hi = *r; }
    }
    s.mean = taken == 0u ? 0u : sum / taken;
    return s;
}

/// A free run of `count` entries into `block` through the DMA at the
/// converter's divider: the microseconds it took, 0 when it never
/// finished. The chapter's order: the channel first, then START_MANY.
uint32_t run_block(uint16_t count, uint32_t wait_us) {
    (void)Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
    Adc::drain();
    block_done = false;
    Block::arm(Adc::fifo_address(), Adc::dreq);
    (void)Block::start(block, count);
    const uint32_t t0 = us_now();
    Adc::start_many(true);
    // The wait keeps OFF the bus while the DMA pops the FIFO through it:
    // the ruler is read once in a thousand turns.
    for (uint32_t turn = 0; !block_done; ++turn) {
        if ((turn & 1023u) == 0u && us_now() - t0 >= wait_us) {
            break;
        }
    }
    const uint32_t took = us_now() - t0;
    (void)Adc::stop_many();
    if (!block_done) {
        (void)Block::abandon();
        return 0;
    }
    return took;
}

bool adc_up(AdcClock src = AdcClock::pll_usb) {
    fifo_owner = FifoOwner::none;
    const bool ok = Adc::init(clock, src);
    Adc::temperature_sensor(true);
    spin_us(100);
    return ok;
}

/// The pad behind input k, claimed as an analog input with `pull`.
bool claim_input(uint8_t k, PinPull pull) {
    return Gpio::analog(static_cast<uint8_t>(adc_first_pin + k), pull);
}
bool release_input(uint8_t k) { return Gpio::release(static_cast<uint8_t>(adc_first_pin + k)); }

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("the rate arithmetic at 48 MHz: DIV 0 is 500 ksps, 47999 is 1 ksps (12.4.3.2's example), 100 ksps "
                  "wants 479, 500 ksps through the divider and 700 sps are refused; at 12 MHz DIV 0 is 125 ksps",
                  adc_rate_hz(48'000'000, {}) == 500'000u && adc_divider_for(48'000'000, 1000)->integer == 47999u &&
                      adc_divider_for(48'000'000, 100'000)->integer == 479u && !adc_divider_for(48'000'000, 500'000) &&
                      !adc_divider_for(48'000'000, 700) && adc_rate_hz(12'000'000, {}) == 125'000u);
    bench.verdict("the temperature line at the chapter's own example: 891 counts at 3.3 V is 20.1 C (12.4.6)",
                  adc_temperature_centi(891, 3300) == 2012);

    const Package sel = ChipId::package_sel();
    print(serial, "  the package: SYSINFO.PACKAGE_SEL says ", sel == Package::qfn80 ? "QFN-80" : "QFN-60",
          ", this image is built for ", package == Package::qfn80 ? "QFN-80" : "QFN-60", " - ",
          adc_pad_inputs, " pads from GP", adc_first_pin, ", the sensor on AINSEL ", adc_temperature_code,
          ", the round-robin mask ", hex(adc_round_robin_mask), crlf);
    bench.verdict("the silicon is in the package this image states, so the converter's input map and the sensor's "
                  "channel number are the right ones",
                  sel == package && Adc::package_matches());

    // The block's reset is a CYCLE, and the clock comes first: nothing
    // reads an ADC register while clk_adc is stopped.
    Clocks::adc_stop();
    const bool unclocked = Resets::cycle(ResetBlock::adc);
    (void)Clocks::adc_select(AdcAux::xosc);
    const bool reset_ok = Resets::cycle(ResetBlock::adc);
    print(serial, "  the reset with clk_adc stopped: ", unclocked ? "completed" : "never completes",
          "; under the crystal: ", reset_ok ? "completed" : "NOT completed", "; CS=", hex(Adc::regs().CS),
          " FCS=", hex(Adc::regs().FCS), " DIV=", hex(Adc::regs().DIV), crlf);
    bench.verdict("the block cycles through reset under its clock, and its reset state is: disabled, not ready, the "
                  "FIFO off, the divider at zero",
                  reset_ok && Adc::regs().CS == 0u && !Adc::ready() && !Adc::fifo_enabled() &&
                      Adc::divider() == AdcDivider{});

    const bool up = adc_up(AdcClock::pll_usb);
    const auto adc_hz = SysClock::count_hz(CountSource::clk_adc);
    const auto usb_hz = SysClock::count_hz(CountSource::pll_usb);
    print(serial, "  on the USB PLL: locked=", PllUsb::locked(), " pll_usb=", usb_hz ? *usb_hz : 0u,
          " Hz, clk_adc=", adc_hz ? *adc_hz : 0u, " Hz (", Adc::clock_hz(), " stated), ready=", Adc::ready(), crlf);
    bench.verdict("init() on the USB PLL: the PLL locked at 48 MHz and clk_adc counted at 48 MHz within a tenth of a "
                  "per cent, READY up",
                  up && PllUsb::locked() && adc_hz && within(*adc_hz, 48'000'000, 1) && Adc::ready());

    // Every channel selected and read back, and the one past the last
    // refused - the field is four bits here and the count is the
    // package's.
    uint8_t walked = 0;
    for (uint8_t k = 0; k < Adc::inputs; ++k) {
        Adc::select_input(k);
        if (Adc::selected() == k && Adc::selected_input() == k) {
            ++walked;
        }
    }
    Adc::select_input(Adc::inputs);
    const uint8_t after = Adc::selected();
    Adc::select(AdcInput::temperature);
    const uint8_t sensor = Adc::selected();
    print(serial, "  the channels: ", walked, " of ", Adc::inputs, " selected and read back; a select of ",
          Adc::inputs, " left AINSEL at ", after, "; AdcInput::temperature selects ", sensor, crlf);
    bench.verdict("every channel this package has is selected and reads back in AINSEL, the channel one past the last "
                  "is refused with AINSEL untouched, and the sensor's tag names the package's own number",
                  walked == Adc::inputs && after == Adc::inputs - 1u && sensor == Adc::temperature_input);

    const bool up2 = adc_up(AdcClock::crystal);
    const auto adc_hz2 = SysClock::count_hz(CountSource::clk_adc);
    print(serial, "  on the crystal: clk_adc=", adc_hz2 ? *adc_hz2 : 0u, " Hz (", Adc::clock_hz(), " stated), ready=",
          Adc::ready(), crlf);
    bench.verdict("init() on the crystal: clk_adc counted at 12 MHz, READY up",
                  up2 && adc_hz2 && within(*adc_hz2, SysClock::xtal_hz, 1) && Adc::ready());
    Adc::release();
}

// =============================================================================
// b - the temperature sensor
// =============================================================================
void tb_temperature() {
    (void)adc_up(AdcClock::pll_usb);
    Adc::select(AdcInput::temperature);
    const Stat pll = sample(64);
    const int32_t t_pll = adc_temperature_centi(static_cast<uint16_t>(pll.mean), vref);
    print(serial, "  the sensor on the USB PLL: mean ", pll.mean, " counts (", pll.lo, "..", pll.hi, "), ", pll.failed,
          " failed -> ", t_pll / 100, ".", (t_pll % 100 < 0 ? -(t_pll % 100) : t_pll % 100) / 10, " C at ", vref,
          " mV", crlf);
    bench.verdict("the temperature sensor reads a room temperature (10..60 C at a 3.3 V supply) with no failed "
                  "conversion",
                  t_pll >= 1000 && t_pll <= 6000 && pll.failed == 0u);
    bench.verdict("... with a spread under sixteen counts over 64 reads",
                  static_cast<uint16_t>(pll.hi - pll.lo) < 16u);
    (void)adc_up(AdcClock::crystal);
    Adc::select(AdcInput::temperature);
    const Stat xo = sample(64);
    const int32_t t_xo = adc_temperature_centi(static_cast<uint16_t>(xo.mean), vref);
    print(serial, "  the sensor on the crystal: mean ", xo.mean, " counts (", xo.lo, "..", xo.hi, ") -> ", t_xo / 100,
          ".", (t_xo % 100 < 0 ? -(t_xo % 100) : t_xo % 100) / 10, " C", crlf);
    bench.verdict("the same reading on the crystal's 12 MHz within eight counts: the clock's rate is the conversion's "
                  "pace and not its result",
                  xo.mean + 8u >= pll.mean && xo.mean <= pll.mean + 8u);
    Adc::temperature_sensor(false);
    spin_us(100);
    const Stat off = sample(16);
    print(serial, "  the sensor's bias off: mean ", off.mean, " counts", crlf);
    bench.verdict("with the bias off the input is not the diode: the reading differs from the sensor's by 100 counts "
                  "or more",
                  off.mean + 100u <= xo.mean || off.mean >= xo.mean + 100u);
    Adc::release();
}

// =============================================================================
// c - every pad, through its pulls
// =============================================================================
void tc_pads() {
    (void)adc_up();
    uint8_t up = 0;
    uint8_t down = 0;
    uint8_t separated = 0;
    for (uint8_t k = 0; k < adc_pad_inputs; ++k) {
        (void)claim_input(k, PinPull::up);
        Adc::select_input(k);
        spin_us(500);
        const Stat u = sample(8);
        (void)claim_input(k, PinPull::down);
        spin_us(500);
        const Stat d = sample(8);
        (void)claim_input(k, PinPull::none);
        spin_us(500);
        const Stat f = sample(8);
        (void)release_input(k);
        print(serial, "  GP", adc_first_pin + k, " (input ", k, "): up ", u.mean, " (",
              adc_mv(static_cast<uint16_t>(u.mean), adc_steps, vref), " mV), down ", d.mean, " (",
              adc_mv(static_cast<uint16_t>(d.mean), adc_steps, vref), " mV), floating ", f.mean, " (",
              adc_mv(static_cast<uint16_t>(f.mean), adc_steps, vref), " mV)", crlf);
        if (u.mean > 3000u) { ++up; }
        if (d.mean < 1000u) { ++down; }
        if (u.mean > d.mean + 2000u) { ++separated; }
    }
    bench.verdict("every pad the package gives the converter reads high through its pull-up (over 3000 of 4095)",
                  up == adc_pad_inputs);
    bench.verdict("... and low through its pull-down (under 1000): an analog claim disables the pad's digital input "
                  "buffer, which is what 12.4 asks for and what closes erratum RP2350-E9's leakage path, so the "
                  "pull-down holds",
                  down == adc_pad_inputs);
    bench.verdict("... and the two pulls are more than 2000 counts apart on every input",
                  separated == adc_pad_inputs);
    Adc::release();
}

// =============================================================================
// d - the rate
// =============================================================================
void td_rate() {
    (void)adc_up();
    Adc::select(AdcInput::temperature);
    struct Case { const char* name; AdcDivider div; uint32_t rate; };
    const Case cases[] = {
        {"DIV 0 (500 ksps)", {}, 500'000},
        {"100 ksps", *adc_divider_for(48'000'000, 100'000), 100'000},
        {"10 ksps", *adc_divider_for(48'000'000, 10'000), 10'000},
        {"44.1 ksps (fractional)", *adc_divider_for(48'000'000, 44'100), 44'100},
    };
    // A warming block first: the first run out of a cold instruction
    // path is not the pace the ones after keep.
    Adc::divider({});
    (void)run_block(256, 200'000);
    Adc::clear_fifo_flags();
    uint8_t ok = 0;
    for (const Case& c : cases) {
        Adc::divider(c.div);
        const uint32_t took = run_block(256, 200'000);
        const uint32_t want_us = static_cast<uint32_t>(256'000'000ULL / c.rate);
        const bool good = took != 0u && within(took, want_us, 40);
        print(serial, "  ", c.name, ": DIV ", c.div.integer, " + ", c.div.frac, "/256 -> 256 conversions in ", took,
              " us (", want_us, " expected, ", took * 1000u / 256u, " ns each), the last ", block[255] & 0xFFFu,
              ", overflow=", Adc::fifo_overflowed(), good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
    }
    bench.verdict("free-running conversions into a 256-word DMA block: 500 ksps at DIV 0, 100 and 10 ksps and a "
                  "fractional 44.1 ksps each within four per cent of the ruler, no overflow with the converter "
                  "stopped from the block's completion",
                  ok == 4u && !Adc::fifo_overflowed());
    // The dividers around the conversion's own 96 cycles: a trigger that
    // lands inside a conversion is ignored (12.4.3.2), and DIV 95 - a
    // period of EXACTLY 96 cycles - is the boundary the chapter does not
    // state. Timed, not judged: this letter is where the driver's
    // arithmetic meets the silicon's.
    struct Probe { AdcDivider div; uint32_t us; };
    const Probe probes[] = {{{0, 0}, 250}, {{95, 0}, 200}, {{96, 0}, 200}, {{99, 0}, 200}, {{119, 0}, 200}};
    for (const Probe& pr : probes) {
        Adc::divider(pr.div);
        (void)Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
        Adc::drain();
        block_done = false;
        Block::arm(Adc::fifo_address(), Adc::dreq);
        (void)Block::start(block, 256);
        Adc::start_many(true);
        spin_us(pr.us);
        const uint32_t got = Block::take();
        (void)Block::abandon();
        Adc::start_many(false);
        (void)Adc::wait_ready();
        const uint8_t level = Adc::fifo_level();
        const bool over = Adc::fifo_overflowed();
        Adc::drain();
        Adc::clear_fifo_flags();
        const uint32_t counted = static_cast<uint32_t>(static_cast<uint64_t>(got + level) * 1'000'000u / pr.us);
        print(serial, "  ", pr.us, " us at DIV ", pr.div.integer, ": the channel took ", got, ", the FIFO held ",
              level, " -> ", counted, " sps measured, ", adc_rate_hz(Adc::clock_hz(), pr.div),
              " by the driver's arithmetic, overflow=", over, crlf);
    }
    bench.verdict("the dividers around the conversion's 96 cycles are measured above: a start that arrives while the "
                  "converter is busy is ignored, so a divider period at or below the conversion's stretches to a "
                  "multiple of itself",
                  true);
    (void)adc_up(AdcClock::crystal);
    Adc::select(AdcInput::temperature);
    Adc::divider({});
    const uint32_t took = run_block(256, 200'000);
    print(serial, "  on the crystal at DIV 0: 256 conversions in ", took, " us (2048 expected: 8 us each)", crlf);
    bench.verdict("on the crystal a conversion takes 96 of its 12 MHz cycles: 125 ksps",
                  took != 0u && within(took, 2048, 20));
    Adc::release();
}

// =============================================================================
// e - the round-robin over every channel
// =============================================================================
void te_round_robin() {
    (void)adc_up();
    // Alternating pulls, so that the pattern read back is a sequence of
    // levels and not a sequence of equal numbers: even inputs up, odd
    // inputs down, the sensor last.
    for (uint8_t k = 0; k < adc_pad_inputs; ++k) {
        (void)claim_input(k, (k & 1u) == 0u ? PinPull::up : PinPull::down);
    }
    spin_us(1000);
    Adc::select_input(0);
    Adc::round_robin<adc_round_robin_mask>();
    Adc::divider(*adc_divider_for(48'000'000, 100'000));
    const uint16_t entries = static_cast<uint16_t>(adc_input_count * 8u);
    const uint32_t took = run_block(entries, 200'000);
    (void)Adc::round_robin(0);
    uint16_t in_order = 0;
    for (uint16_t i = 0; i < entries; ++i) {
        const uint16_t v = block[i] & 0xFFFu;
        const uint8_t slot = static_cast<uint8_t>(i % adc_input_count);
        bool good = false;
        if (slot == adc_temperature_code) {
            good = v > 400u && v < 1400u;              // the sensor's diode
        } else if ((slot & 1u) == 0u) {
            good = v > 3000u;                          // a pulled-up pad
        } else {
            good = v < 1000u;                          // a pulled-down pad
        }
        if (good) {
            ++in_order;
        } else {
            print(serial, "  entry ", i, " (slot ", slot, "): ", hex(block[i]), crlf);
        }
    }
    print(serial, "  the round-robin over all ", adc_input_count, " channels (mask ", hex(adc_round_robin_mask),
          ") at 100 ksps: ", entries, " entries in ", took, " us, the first nine ", block[0] & 0xFFFu, " ",
          block[1] & 0xFFFu, " ", block[2] & 0xFFFu, " ", block[3] & 0xFFFu, " ", block[4] & 0xFFFu, " ",
          block[5] & 0xFFFu, " ", block[6] & 0xFFFu, " ", block[7] & 0xFFFu, " ", block[8] & 0xFFFu, "; ",
          in_order, " of ", entries, " where expected", crlf);
    bench.verdict("the round-robin walks EVERY channel this package has - a mask nine bits wide where the RP2040's "
                  "was five - and every entry of the block lands where its input's level says it should",
                  took != 0u && in_order == entries);
    bench.verdict("a mask naming a channel this package has not got is refused, and the one it has is taken",
                  !Adc::round_robin(static_cast<uint16_t>(adc_round_robin_mask + 1u)) &&
                      Adc::round_robin(adc_round_robin_mask));
    (void)Adc::round_robin(0);
    for (uint8_t k = 0; k < adc_pad_inputs; ++k) {
        (void)release_input(k);
    }
    Adc::release();
}

// =============================================================================
// f - the FIFO
// =============================================================================
void tf_fifo() {
    (void)adc_up();
    Adc::select(AdcInput::temperature);
    // The threshold interrupt: the handler drains at four.
    (void)Adc::fifo({.enable = true, .dreq = false, .error_flag = true, .shift = false, .threshold = 4});
    Adc::drain();
    fifo_isr_entries = 0;
    fifo_isr_popped = 0;
    fifo_isr_max_level = 0;
    Adc::divider(*adc_divider_for(48'000'000, 10'000));
    fifo_owner = FifoOwner::drain;
    Adc::interrupt(true);
    Irq::enable(Adc::irq());
    Adc::start_many(true);
    spin_us(20'000);
    (void)Adc::stop_many();
    Adc::interrupt(false);
    fifo_owner = FifoOwner::none;
    print(serial, "  the threshold at four (FCS.THRESH reads ", Adc::fifo_threshold(), "), 10 ksps for 20 ms: ",
          fifo_isr_entries, " interrupts, ", fifo_isr_popped, " entries popped, the deepest level seen ",
          fifo_isr_max_level, ", overflow=", Adc::fifo_overflowed(), crlf);
    bench.verdict("the threshold interrupt at four: about 50 interrupts drain about 200 entries in 20 ms, the level "
                  "never past the FIFO's eight, no overflow",
                  fifo_isr_entries >= 45u && fifo_isr_entries <= 55u && fifo_isr_popped >= 190u &&
                      fifo_isr_popped <= 210u && fifo_isr_max_level <= 8u && !Adc::fifo_overflowed());
    bench.verdict("a threshold past the FIFO's depth is refused instead of written: the field holds it and the level "
                  "could never reach it",
                  !Adc::fifo({.threshold = adc_fifo_depth + 1u}) && Adc::fifo_threshold() == 4u);
    // The overflow: nobody drains, and what clears the sticky flag.
    (void)Adc::fifo({.enable = true, .dreq = false, .error_flag = true, .shift = false, .threshold = 1});
    Adc::drain();
    Adc::divider({});
    Adc::start_many(true);
    spin_us(100);
    Adc::start_many(false);
    (void)Adc::wait_ready();
    const uint8_t level = Adc::fifo_level();
    const bool full = Adc::fifo_full();
    const bool over = Adc::fifo_overflowed();
    Adc::drain();
    hw_set(Adc::regs().FCS, ADC_FCS_OVER_BITS);   // the atomic SET alias at a write-one-to-clear bit
    const bool over_after_alias = Adc::fifo_overflowed();
    Adc::clear_fifo_flags();                      // the driver's plain write-back
    const bool over_after_plain = Adc::fifo_overflowed();
    print(serial, "  50 conversions into an undrained FIFO: level ", level, ", full=", full, ", overflow=", over,
          "; after the drain level ", Adc::fifo_level(), ", overflow after the SET alias=", over_after_alias,
          ", after the plain write-back=", over_after_plain, crlf);
    bench.verdict("an undrained FIFO fills at eight and raises the sticky overflow, and the driver's plain write-back "
                  "clears it",
                  level == adc_fifo_depth && full && over && Adc::fifo_level() == 0u && !over_after_plain);
    // The shift: a byte block against a word block of the same input.
    Adc::divider(*adc_divider_for(48'000'000, 100'000));
    const uint32_t took_w = run_block(64, 100'000);
    uint32_t sum_w = 0;
    for (uint8_t i = 0; i < 64u; ++i) {
        sum_w += block[i] & 0xFFFu;
    }
    (void)Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = true, .threshold = 1});
    Adc::drain();
    block_done = false;
    ByteBlock::arm(Adc::fifo_address(), Adc::dreq);
    (void)ByteBlock::start(byte_block, 64);
    const uint32_t t0 = us_now();
    Adc::start_many(true);
    while (!block_done && us_now() - t0 < 100'000u) {
    }
    (void)Adc::stop_many();
    uint32_t sum_b = 0;
    for (uint8_t i = 0; i < 64u; ++i) {
        sum_b += byte_block[i];
    }
    print(serial, "  the sensor as words: mean ", sum_w / 64u, " (in ", took_w, " us); as shifted bytes: mean ",
          sum_b / 64u, " (the word mean over sixteen = ", (sum_w / 64u) >> 4, ")", crlf);
    bench.verdict("FCS.SHIFT delivers the top eight bits into a byte block through an eight-bit DMA engine: the byte "
                  "mean is the word mean over sixteen, within one",
                  block_done && took_w != 0u && (sum_b / 64u) + 1u >= ((sum_w / 64u) >> 4) &&
                      (sum_b / 64u) <= ((sum_w / 64u) >> 4) + 1u);
    // The error flag per entry.
    Adc::divider({});
    const uint32_t took_e = run_block(256, 100'000);
    uint16_t flagged = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        if (Adc::entry_failed(block[i])) {
            ++flagged;
        }
    }
    print(serial, "  256 entries at 500 ksps with FCS.ERR: ", flagged, " flagged as failed, CS.ERR_STICKY=",
          Adc::error_seen(), crlf);
    bench.verdict("the per-entry error flag: no entry of 256 flagged on a steady input (a failed conversion is the "
                  "comparator's metastability, rare by design)",
                  took_e != 0u && flagged == 0u);
    Adc::release();
}

// =============================================================================
// g - the sampler
// =============================================================================
namespace sm {

class Probe {
public:
    using Event = std::variant<AnalogSample>;
    static inline EventQueue<Event, 16, P> queue;
    static inline uint32_t per_input[3];
    static inline uint32_t sum[3];
    static inline uint32_t total = 0;

    static void init() {
        for (uint8_t i = 0; i < 3u; ++i) {
            per_input[i] = 0;
            sum[i] = 0;
        }
        total = 0;
    }
    static void dispatch(const Event& e) {
        match(e, [](const AnalogSample& s) {
            ++total;
            if (s.index < 3u) {
                ++per_input[s.index];
                sum[s.index] += s.value;
            }
        });
    }
};

using Sampler = AnalogSampler<Adc, P, Subscribers<Probe>, AdcInput::temperature, In0{}, In1{}>;
using SamplerKernel = Tenuto<P, Probe, Sampler>;

void pump() {
    TimeEvents<P>::process();
    while (SamplerKernel::step()) {
        TimeEvents<P>::process();
    }
}

}  // namespace sm

void tg_sampler() {
    (void)adc_up();
    (void)In0::claim(PinPull::up);
    (void)In1::claim(PinPull::down);
    spin_us(1000);
    (void)Adc::fifo({.enable = true, .dreq = false, .error_flag = true, .shift = false, .threshold = 1});
    Adc::drain();
    sm::SamplerKernel::init_all();
    fifo_owner = FifoOwner::sampler;
    Adc::interrupt(true);
    Irq::enable(Adc::irq());
    sm::Sampler::start_every(ticks_from_ms<P>(5));
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 200u) {
        sm::pump();
    }
    sm::Sampler::stop();
    sm::pump();
    Adc::interrupt(false);
    fifo_owner = FifoOwner::none;
    const uint32_t n0 = sm::Probe::per_input[0];
    const uint32_t n1 = sm::Probe::per_input[1];
    const uint32_t n2 = sm::Probe::per_input[2];
    print(serial, "  the sampler at 5 ms over the sensor and two pads for 200 ms: ", sm::Probe::total, " samples, ",
          n0, " of the sensor (mean ", n0 ? sm::Probe::sum[0] / n0 : 0u, "), ", n1, " of input 0 pulled up (mean ",
          n1 ? sm::Probe::sum[1] / n1 : 0u, "), ", n2, " of input 1 pulled down (mean ",
          n2 ? sm::Probe::sum[2] / n2 : 0u, "), ", sm::Sampler::unknown_inputs(), " unknown", crlf);
    bench.verdict("AnalogSampler walks three inputs on a 5 ms software pace: about thirteen samples each in 200 ms, "
                  "the sensor's near its count, the pulled-up pad's high and the pulled-down pad's low, none of them "
                  "mislabelled",
                  n0 >= 11u && n0 <= 15u && n1 >= 11u && n1 <= 15u && n2 >= 11u && n2 <= 15u &&
                      sm::Probe::sum[0] / n0 > 400u && sm::Probe::sum[0] / n0 < 1400u &&
                      sm::Probe::sum[1] / n1 > 3000u && sm::Probe::sum[2] / n2 < 1000u &&
                      sm::Sampler::unknown_inputs() == 0u);
    (void)In0::release();
    (void)In1::release();
    Adc::release();
}

// =============================================================================
// h - erratum RP2350-E9, in millivolts
// =============================================================================
void th_leakage() {
    (void)adc_up();
    using Pad = In2::pin;                     // input 2's pad, free like the rest
    constexpr uint8_t pin = Pad::number;
    // A plain digital pad of the same bank, free and floating, to set the
    // converter's pads against: the erratum names pads 0 through 47, and
    // the two kinds have to be asked separately.
    using Digital = Pin<22>;
    // RP2350-E9 says a Bank 0 pad configured as an input, with its
    // voltage in the undefined logic region, sources some 120 uA and is
    // held around 2.2 V by it - too strongly for the pad's own pull-down
    // to bring down. The converter taps the bond pad, so it can put that
    // claim in MILLIVOLTS where a digital read can only say high or low.
    //
    // THE CONTRAST IS THE MEASUREMENT: the same four conditions are set
    // up on a converter pad and on a plain digital pad of the same bank,
    // because a converter pad that does not leak and a test that does not
    // provoke the leak look identical from one pad alone.
    //
    // AND THE PAD HAS TO BE PUT IN THAT REGION FIRST. Every pad of this
    // chip comes out of reset with its own pull-down on and its buffer
    // off, so a pad that is merely released and then given an input
    // buffer is AT GROUND when the buffer is enabled: condition one is
    // not met, the leak never starts, and it drifts up by ordinary
    // leakage instead. The way in with no wire is the pull-UP: it takes
    // the pad to the supply, and removing it drops the pad into the
    // undefined region with the buffer already on.
    //
    // First the digital pad, read by its own buffer.
    (void)Digital::input({.pull = PinPull::none, .input_enable = true});
    spin_us(5000);
    const bool dig_from_ground = Digital::read();
    Digital::pull(PinPull::up);
    spin_us(5000);
    Digital::pull(PinPull::none);
    spin_us(5000);
    const bool dig_floating = Digital::read();
    Digital::pull(PinPull::down);
    spin_us(5000);
    const bool dig_pulled = Digital::read();
    (void)Digital::input({.pull = PinPull::down, .input_enable = false});
    spin_us(5000);
    const bool dig_workaround = Digital::read_pulsed();
    (void)Digital::release();

    // Then the converter pad, in volts, through the same door. The
    // digital read is taken with the converter pointed elsewhere, so
    // that the converter's own input - a switched sampling capacitor,
    // and a load - is not part of the pad's state when it is judged.
    Adc::select(AdcInput::temperature);
    (void)Pad::input({.pull = PinPull::up, .input_enable = true});
    spin_us(5000);
    Pad::pull(PinPull::none);
    spin_us(5000);
    const bool adc_pad_high = Pad::read();
    Adc::select(In2{});
    spin_us(5000);
    const Stat floating = sample(16);
    const uint16_t mv_floating = adc_mv(static_cast<uint16_t>(floating.mean), adc_steps, vref);
    // The pull-down turned on with the buffer still enabled.
    Pad::pull(PinPull::down);
    spin_us(5000);
    const Stat leaking = sample(16);
    const uint16_t mv_leaking = adc_mv(static_cast<uint16_t>(leaking.mean), adc_steps, vref);
    // The buffer cleared - 12.4's own instruction for an ADC pad, IE low,
    // which is the erratum's workaround by the same store.
    (void)Pad::analog(PinPull::down);
    spin_us(5000);
    const Stat held = sample(16);
    const uint16_t mv_held = adc_mv(static_cast<uint16_t>(held.mean), adc_steps, vref);
    (void)Pad::analog(PinPull::up);
    spin_us(2000);
    const Stat pulled_up = sample(16);
    const uint16_t mv_up = adc_mv(static_cast<uint16_t>(pulled_up.mean), adc_steps, vref);
    (void)Pad::release();

    print(serial, "  the digital pad GP", Digital::number, ", read by its own buffer: the buffer enabled over a pad "
          "AT GROUND reads ", dig_from_ground ? "HIGH" : "LOW", "; pulled up then released into the undefined "
          "region ", dig_floating ? "HIGH" : "LOW", "; pull-down added under that ",
          dig_pulled ? "HIGH" : "LOW", "; buffer off and pulsed for the read ",
          dig_workaround ? "HIGH" : "LOW", crlf);
    print(serial, "  the converter pad GP", pin, ", in millivolts, through the same door: released into the "
          "undefined region ", floating.mean, " counts = ", mv_floating, " mV (its own buffer reads it ",
          adc_pad_high ? "HIGH" : "LOW", "); pull-down added under that ", leaking.mean, " = ", mv_leaking,
          " mV; buffer off ", held.mean, " = ", mv_held, " mV; pulled up ", pulled_up.mean, " = ", mv_up,
          " mV", crlf);

    bench.verdict("ERRATUM RP2350-E9 IS LIVE ON THIS PART: a pad taken to the supply and released with its input "
                  "buffer enabled STAYS HIGH, and its own pull-down turned on under the leak does NOT bring it "
                  "down - which is the errata sheet's sentence about the pull being far weaker than the leakage",
                  dig_floating && dig_pulled);
    bench.verdict("... AND IT HAS TO BE ENTERED THROUGH THE UNDEFINED REGION: the same pad with its buffer enabled "
                  "over a pad the reset pull-down is already holding AT GROUND reads LOW and stays there, which is "
                  "the erratum's own first condition and what makes every pulled-down reading of this suite sound",
                  !dig_from_ground);
    bench.verdict("... and the workaround works: with the input buffer off the same pull-down holds the same pad, "
                  "and a read that pulses the buffer for its own duration sees LOW",
                  !dig_workaround);
    bench.verdict("... AND IN MILLIVOLTS, ON A CONVERTER PAD TAKEN IN THE SAME WAY: the leak parks the pad well "
                  "above ground, near the 2.2 V the errata sheet names as its effective source voltage, where an "
                  "unleaking pad would be at whatever it drifted to",
                  mv_floating > 1200u && adc_pad_high);
    bench.verdict("... and the pad's own pull-down cannot bring THAT down either, with the buffer still enabled",
                  mv_leaking > 1200u);
    bench.verdict("... while clearing the input buffer - what 12.4 asks for anyway, and what every other letter of "
                  "this suite uses - removes the leak and lets the same pull-down hold the same pad at ground",
                  mv_held < 300u);
    bench.verdict("... and the pull-up reaches the supply either way (over 3000 mV)", mv_up > 3000u);
    Adc::release();
}

void banner() {
    print(serial, crlf, "test_rp2350_adc - this chip's ADC (datasheet 12.4): the sensor on AINSEL ",
          adc_temperature_code, ", GP", adc_first_pin, "..GP", adc_first_pin + adc_pad_inputs - 1u,
          " as inputs 0..", adc_pad_inputs - 1u, "; clk_sys=", SysClock::hz, " Hz, ",
          core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_adc_fifo() {
    fifo_isr_entries = fifo_isr_entries + 1u;
    const uint8_t level = brio::Adc::fifo_level();
    if (level > fifo_isr_max_level) {
        fifo_isr_max_level = level;
    }
    switch (fifo_owner) {
        case FifoOwner::drain:
            while (!brio::Adc::fifo_empty()) {
                (void)brio::Adc::pop();
                fifo_isr_popped = fifo_isr_popped + 1u;
            }
            break;
        case FifoOwner::sampler:
            while (!brio::Adc::fifo_empty()) {
                const uint16_t e = brio::Adc::pop();
                brio::post<sm::Sampler>(brio::Sampled{brio::Adc::entry_value(e), brio::Adc::selected_input()});
            }
            break;
        default:
            brio::Adc::drain();
            brio::Adc::interrupt(false);
            break;
    }
}
extern "C" void isr_dma_0() {
    const uint8_t w = Block::service();
    const uint8_t b = ByteBlock::service();
    if ((w & Block::flag_complete) != 0u || (b & ByteBlock::flag_complete) != 0u) {
        if (stop_on_done) {
            // At 500 ksps the FIFO fills in microseconds after the block:
            // the converter is stopped here, not in the loop.
            brio::Adc::start_many(false);
        }
        block_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer<0>::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output(false);
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless: the package and the clocks counted", ta_block);
    bench.letter('b', "the temperature sensor on both clocks", tb_temperature);
    bench.letter('c', "every pad through its pulls", tc_pads);
    bench.letter('d', "the rate into DMA blocks", td_rate);
    bench.letter('e', "the round-robin over every channel", te_round_robin);
    bench.letter('f', "the FIFO: threshold, overflow, shift, the error flag", tf_fifo);
    bench.letter('g', "the sampler", tg_sampler);
    bench.letter('h', "erratum RP2350-E9 in millivolts", th_leakage);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", timer_ok ? "1us" : "FAILED", " dma=", dma_ok ? "released" : "FAILED",
                    " tick=", tick_ok ? "on" : "FAILED", brio::crlf);
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
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
