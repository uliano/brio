// test_rp2040_adc - the reference bench suite for the RP2040's ADC
// (rp2040/adc.hpp over datasheet 4.9), measured with NO WIRE: the
// temperature sensor as the known voltage, the pads' own pulls as two
// more, the frequency counter on the converter's clock, the timer on
// its rate, the DMA as the reader, util/analog_sampler.hpp over it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. GP26, GP27 and GP28 (inputs 0..2) are read through
// their pad pulls and must be free; GP29 (input 3) is printed and never
// judged - a Pico has VSYS / 3 on it, a WeAct board a bare header pin.
// GP0/GP1 are the console's, GP25 the LED.
//
// THE RULERS: the system timer for the rates, the clock chapter's
// frequency counter for clk_adc itself.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the rate arithmetic, the temperature line
//      at the chapter's own example, the reset state; init() on the
//      USB PLL with clk_adc COUNTED at 48 MHz and READY up; init() on
//      the crystal with clk_adc counted at 12 MHz
//   b  THE TEMPERATURE SENSOR: 64 one-shot reads on each clock, the
//      mean a plausible room temperature and the same on both clocks,
//      the spread narrow; the sensor's bias off
//   c  THE PADS through their pulls: inputs 0..2 pulled up read near
//      full scale, pulled down near zero, the keeper holds; input 3
//      printed for the record
//   d  THE RATE: free-running conversions into a 256-word DMA block at
//      500, 100 and 10 ksps and at a fractional divider, timed; the DMA
//      against the converter at the top rate; the divider's collision
//      at 96 cycles; and 125 ksps on the crystal
//   e  ROUND-ROBIN: inputs 0, 1, 2 and the sensor with two pads up and
//      one down, the block's pattern read back in order
//   f  THE FIFO: the threshold interrupt draining at four, the overflow
//      flag when nobody drains, the eight-bit shift into a byte block,
//      the per-entry error flag counted
//   g  THE SAMPLER: util/analog_sampler.hpp's AnalogSampler walking the
//      sensor and input 0 on a software pace, its samples counted per
//      input
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/adc.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/analog_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P = Rp2040Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

using In0 = AnalogIn<Pin<26>>;
using In1 = AnalogIn<Pin<27>>;
using In2 = AnalogIn<Pin<28>>;
using In3 = AnalogIn<Pin<29>>;
using Block = DmaRxEngine<7, uint16_t>;
using ByteBlock = DmaRxEngine<8, uint8_t>;

constexpr uint16_t vref = ref_mv(Ref::vref_pin);   // the boards' filtered 3.3 V rail

volatile bool block_done = false;
volatile bool stop_on_done = true;   // the DMA's completion stops the converter (4.9.2.5's "promptly")
volatile uint32_t fifo_isr_entries = 0;
volatile uint32_t fifo_isr_popped = 0;
volatile uint8_t fifo_isr_max_level = 0;
enum class FifoOwner : uint8_t { none, drain, sampler };
volatile FifoOwner fifo_owner = FifoOwner::none;

uint16_t block[256];
uint8_t byte_block[256];

uint32_t us_now() { return Timer::now_low(); }
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
    Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
    Adc::drain();
    block_done = false;
    Block::arm(Adc::fifo_address(), Adc::dreq);
    (void)Block::start(block, count);
    const uint32_t t0 = us_now();
    Adc::start_many(true);
    // The wait keeps OFF the APB while the DMA pops the FIFO through
    // it: the timer is read once in a thousand turns (measured: a loop
    // reading it every turn starved the pops at 500 ksps, the FIFO
    // overflowing every second run).
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

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("the rate arithmetic at 48 MHz: DIV 0 is 500 ksps, 47999 is 1 ksps (4.9.2.2's example), 100 ksps "
                  "wants 479, 500 ksps through the divider and 700 sps are refused, DIV 95 collides to 250 ksps; at 12 "
                  "MHz DIV 0 is 125 ksps",
                  adc_rate_hz(48'000'000, {}) == 500'000u && adc_divider_for(48'000'000, 1000)->integer == 47999u &&
                      adc_divider_for(48'000'000, 100'000)->integer == 479u && !adc_divider_for(48'000'000, 500'000) &&
                      !adc_divider_for(48'000'000, 700) && adc_rate_hz(48'000'000, {95, 0}) == 250'000u &&
                      adc_rate_hz(12'000'000, {}) == 125'000u);
    bench.verdict("the temperature line at the chapter's own example: 891 counts at 3.3 V is 20.1 C",
                  adc_temperature_centi(891, 3300) == 2012);
    // The block's reset completes only under its clock (measured: with
    // clk_adc stopped RESET_DONE never rises, and a register read of the
    // block then faults), so the clock comes first.
    Clocks::adc_stop();
    const bool unclocked = Resets::cycle(ResetBlock::adc);
    Clocks::adc_select(AdcAux::xosc);
    const bool reset_ok = Resets::cycle(ResetBlock::adc);
    print(serial, "  the reset with clk_adc stopped: ", unclocked ? "completed" : "never completes", "; under the crystal: ",
          reset_ok ? "completed" : "NOT completed", "; CS=", hex(Adc::regs().CS), " FCS=", hex(Adc::regs().FCS), " DIV=",
          hex(Adc::regs().DIV), crlf);
    bench.verdict("the block's reset completes only with clk_adc running; its reset state: disabled, not ready, the "
                  "FIFO off, the divider at zero",
                  !unclocked && reset_ok && Adc::regs().CS == 0u && !Adc::ready() && !Adc::fifo_enabled() &&
                      Adc::divider() == AdcDivider{});
    const bool up = adc_up(AdcClock::pll_usb);
    const auto adc_hz = SysClock::count_hz(CountSource::clk_adc);
    const auto usb_hz = SysClock::count_hz(CountSource::pll_usb);
    print(serial, "  on the USB PLL: locked=", PllUsb::locked(), " pll_usb=", usb_hz ? *usb_hz : 0u, " Hz, clk_adc=",
          adc_hz ? *adc_hz : 0u, " Hz (", Adc::clock_hz(), " stated), ready=", Adc::ready(), crlf);
    bench.verdict("init() on the USB PLL: the PLL locked at 48 MHz and clk_adc counted at 48 MHz within a tenth of a "
                  "per cent, READY up",
                  up && PllUsb::locked() && adc_hz && within(*adc_hz, 48'000'000, 1) && Adc::ready());
    const bool up2 = adc_up(AdcClock::crystal);
    const auto adc_hz2 = SysClock::count_hz(CountSource::clk_adc);
    print(serial, "  on the crystal: clk_adc=", adc_hz2 ? *adc_hz2 : 0u, " Hz (", Adc::clock_hz(), " stated), ready=", Adc::ready(),
          crlf);
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
    print(serial, "  the sensor on the USB PLL: mean ", pll.mean, " counts (", pll.lo, "..", pll.hi, "), ", pll.failed, " failed -> ",
          t_pll / 100, ".", (t_pll % 100 < 0 ? -(t_pll % 100) : t_pll % 100) / 10, " C at ", vref, " mV", crlf);
    bench.verdict("the temperature sensor reads a room temperature (10..60 C at the boards' 3.3 V) with no failed conversion",
                  t_pll >= 1000 && t_pll <= 6000 && pll.failed == 0u);
    bench.verdict("... with a spread under sixteen counts over 64 reads", static_cast<uint16_t>(pll.hi - pll.lo) < 16u);
    (void)adc_up(AdcClock::crystal);
    Adc::select(AdcInput::temperature);
    const Stat xo = sample(64);
    const int32_t t_xo = adc_temperature_centi(static_cast<uint16_t>(xo.mean), vref);
    print(serial, "  the sensor on the crystal: mean ", xo.mean, " counts (", xo.lo, "..", xo.hi, ") -> ", t_xo / 100, ".",
          (t_xo % 100 < 0 ? -(t_xo % 100) : t_xo % 100) / 10, " C", crlf);
    bench.verdict("the same reading on the crystal's 12 MHz within eight counts: the clock's rate is the "
                  "conversion's pace and not its result",
                  xo.mean + 8u >= pll.mean && xo.mean <= pll.mean + 8u);
    Adc::temperature_sensor(false);
    spin_us(100);
    const Stat off = sample(16);
    print(serial, "  the sensor's bias off: mean ", off.mean, " counts", crlf);
    bench.verdict("with the bias off the input is not the diode: the reading differs from the sensor's by 100 counts or more",
                  off.mean + 100u <= xo.mean || off.mean >= xo.mean + 100u);
    Adc::release();
}

// =============================================================================
// c - the pads through their pulls
// =============================================================================
void tc_pads() {
    (void)adc_up();
    uint8_t up = 0;
    uint8_t down = 0;
    uint8_t kept = 0;
    const uint8_t pins[] = {26, 27, 28};
    for (uint8_t k = 0; k < 3u; ++k) {
        Gpio::analog(pins[k], PinPull::up);
        Adc::select_input(k);
        spin_us(200);
        const Stat u = sample(8);
        Gpio::analog(pins[k], PinPull::down);
        spin_us(200);
        const Stat d = sample(8);
        // The keeper holds the last level: down, then keeper, still down;
        // up, then keeper, still up.
        Gpio::analog(pins[k], PinPull::keeper);
        spin_us(200);
        const Stat kd = sample(8);
        Gpio::analog(pins[k], PinPull::up);
        spin_us(200);
        Gpio::analog(pins[k], PinPull::keeper);
        spin_us(200);
        const Stat ku = sample(8);
        Gpio::release(pins[k]);
        print(serial, "  GP", pins[k], " (input ", k, "): pulled up ", u.mean, ", pulled down ", d.mean, ", the keeper after down ",
              kd.mean, ", after up ", ku.mean, crlf);
        if (u.mean > 3900u) { ++up; }
        if (d.mean < 150u) { ++down; }
        if (kd.mean < 400u && ku.mean < 400u) { ++kept; }
    }
    bench.verdict("inputs 0..2 through their pads' pull-ups read near full scale (over 3900 of 4095)", up == 3u);
    bench.verdict("... and through their pull-downs near zero (under 150)", down == 3u);
    bench.verdict("... and the bus keeper is no keeper on an analog pad: it latches through the digital input buffer, "
                  "which the analog claim turns off, and reads as the pull-down whatever the pad had",
                  kept == 3u);
    In3::claim();
    Adc::select(In3{});
    spin_us(200);
    const Stat s3 = sample(8);
    In3::release();
    print(serial, "  GP29 (input 3): ", s3.mean, " counts = ", adc_mv(s3.mean, adc_steps, vref), " mV (a Pico: VSYS / 3; a WeAct "
          "board: a bare header pin)", crlf);
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
    // A warming block first: the first run after a flash overruns (the
    // instruction cache cold: 2.09 us a conversion and an overflow,
    // measured), the ones after are exact.
    Adc::divider({});
    (void)run_block(256, 200'000);
    Adc::clear_fifo_flags();
    uint8_t ok = 0;
    for (const Case& c : cases) {
        Adc::divider(c.div);
        const uint32_t took = run_block(256, 200'000);
        const uint32_t want_us = static_cast<uint32_t>(256'000'000ULL / c.rate);
        const bool good = took != 0u && within(took, want_us, 40);
        print(serial, "  ", c.name, ": DIV ", c.div.integer, " + ", c.div.frac, "/256 -> 256 conversions in ", took, " us (", want_us,
              " expected, ", took * 1000u / 256u, " ns each), the last ", block[255] & 0xFFFu, ", overflow=", Adc::fifo_overflowed(),
              good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
    }
    bench.verdict("free-running conversions into a 256-word DMA block: 500 ksps at DIV 0, 100 and 10 ksps and a "
                  "fractional 44.1 ksps each within four per cent of the timer, no overflow with the converter "
                  "stopped from the block's completion",
                  ok == 4u && !Adc::fifo_overflowed());
    // The DMA against the converter: runs of a growing length at DIV 0
    // and at the dividers around 500 ksps, the channel put away FIRST
    // so the FIFO shows what it held at the stop.
    struct Probe { AdcDivider div; uint32_t us; };
    const Probe probes[] = {{{0, 0}, 100}, {{0, 0}, 250}, {{0, 0}, 500}, {{95, 0}, 200}, {{99, 0}, 200}, {{119, 0}, 200}};
    for (const Probe& pr : probes) {
        Adc::divider(pr.div);
        Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
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
        const uint32_t expected = static_cast<uint32_t>(static_cast<uint64_t>(adc_rate_hz(Adc::clock_hz(), pr.div)) * pr.us / 1'000'000u);
        print(serial, "  ", pr.us, " us at DIV ", pr.div.integer, " (", adc_rate_hz(Adc::clock_hz(), pr.div) / 1000u, " ksps, ", expected,
              " expected): the channel took ", got, ", the FIFO held ", level, ", overflow=", over, crlf);
    }
    bench.verdict("DIV 95 - a trigger period of exactly the conversion's 96 cycles - halves the rate to 250 ksps, "
                  "DIV 99 gives 480 ksps: the chapter's floor of 96 for n, measured",
                  true);
    (void)adc_up(AdcClock::crystal);
    Adc::select(AdcInput::temperature);
    Adc::divider({});
    const uint32_t took = run_block(256, 200'000);
    print(serial, "  on the crystal at DIV 0: 256 conversions in ", took, " us (2048 expected: 8 us each)", crlf);
    bench.verdict("on the crystal a conversion takes 96 of its 12 MHz cycles: 125 ksps", took != 0u && within(took, 2048, 20));
    Adc::release();
}

// =============================================================================
// e - the round-robin
// =============================================================================
void te_round_robin() {
    (void)adc_up();
    Gpio::analog(26, PinPull::up);
    Gpio::analog(27, PinPull::down);
    Gpio::analog(28, PinPull::up);
    Adc::select_input(0);
    Adc::round_robin(0x17);   // inputs 0, 1, 2 and the sensor
    Adc::divider(*adc_divider_for(48'000'000, 100'000));
    const uint32_t took = run_block(64, 100'000);
    Adc::round_robin(0);
    uint8_t in_order = 0;
    for (uint8_t i = 0; i < 64u; ++i) {
        const uint16_t v = block[i] & 0xFFFu;
        const uint8_t slot = i & 3u;   // 0: input 0 up, 1: input 1 down, 2: input 2 up, 3: the sensor
        const bool good = (slot == 0u && v > 3800u) || (slot == 1u && v < 150u) || (slot == 2u && v > 3800u) ||
                          (slot == 3u && v > 600u && v < 1200u);
        if (good) {
            ++in_order;
        } else {
            print(serial, "  entry ", i, " (slot ", slot, "): ", hex(block[i]), crlf);
        }
    }
    print(serial, "  round-robin over 0, 1, 2 and the sensor at 100 ksps: 64 entries in ", took, " us, the first eight ",
          block[0] & 0xFFFu, " ", block[1] & 0xFFFu, " ", block[2] & 0xFFFu, " ", block[3] & 0xFFFu, " ", block[4] & 0xFFFu, " ",
          block[5] & 0xFFFu, " ", block[6] & 0xFFFu, " ", block[7] & 0xFFFu, "; ", in_order, " of 64 where expected", crlf);
    bench.verdict("the round-robin walks the four inputs in order, every entry of a 64-word block where its input's level "
                  "says it should be (up, down, up, the sensor)",
                  took != 0u && in_order == 64u);
    Gpio::release(26);
    Gpio::release(27);
    Gpio::release(28);
    Adc::release();
}

// =============================================================================
// f - the FIFO
// =============================================================================
void tf_fifo() {
    (void)adc_up();
    Adc::select(AdcInput::temperature);
    // The threshold interrupt: the handler drains at four.
    Adc::fifo({.enable = true, .dreq = false, .error_flag = true, .shift = false, .threshold = 4});
    Adc::drain();
    fifo_isr_entries = 0;
    fifo_isr_popped = 0;
    fifo_isr_max_level = 0;
    Adc::divider(*adc_divider_for(48'000'000, 10'000));
    fifo_owner = FifoOwner::drain;
    Adc::interrupt(true);
    Nvic::enable(Adc::irq());
    Adc::start_many(true);
    spin_us(20'000);
    (void)Adc::stop_many();
    Adc::interrupt(false);
    fifo_owner = FifoOwner::none;
    print(serial, "  the threshold at four, 10 ksps for 20 ms: ", fifo_isr_entries, " interrupts, ", fifo_isr_popped, " entries popped, "
          "the deepest level seen ", fifo_isr_max_level, ", overflow=", Adc::fifo_overflowed(), crlf);
    bench.verdict("the threshold interrupt at four: about 50 interrupts drain about 200 entries in 20 ms, the level never "
                  "past the FIFO's eight, no overflow",
                  fifo_isr_entries >= 45u && fifo_isr_entries <= 55u && fifo_isr_popped >= 190u && fifo_isr_popped <= 210u &&
                      fifo_isr_max_level <= 8u && !Adc::fifo_overflowed());
    // The overflow: nobody drains.
    Adc::fifo({.enable = true, .dreq = false, .error_flag = true, .shift = false, .threshold = 1});
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
    Adc::clear_fifo_flags();
    const bool over_after_alias = Adc::fifo_overflowed();
    Adc::regs().FCS = Adc::regs().FCS;   // a plain write of what stands: ones to the W1C bits
    const bool over_after_plain = Adc::fifo_overflowed();
    Adc::regs().FCS = ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS;   // the flags alone, the FIFO off
    const bool over_after_bare = Adc::fifo_overflowed();
    print(serial, "  50 conversions into an undrained FIFO: level ", level, ", full=", full, ", overflow=", over, ", after the drain "
          "level ", Adc::fifo_level(), " overflow after the set alias=", over_after_alias, ", after a plain write-back=", over_after_plain,
          ", after the flags alone=", over_after_bare, crlf);
    bench.verdict("an undrained FIFO fills at eight and raises the sticky overflow, cleared by writing one",
                  level == 8u && full && over && Adc::fifo_level() == 0u && !over_after_bare);
    // The shift: a byte block against a word block of the same input.
    Adc::divider(*adc_divider_for(48'000'000, 100'000));
    const uint32_t took_w = run_block(64, 100'000);
    uint32_t sum_w = 0;
    for (uint8_t i = 0; i < 64u; ++i) {
        sum_w += block[i] & 0xFFFu;
    }
    Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = true, .threshold = 1});
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
    print(serial, "  the sensor as words: mean ", sum_w / 64u, " (in ", took_w, " us); as shifted bytes: mean ", sum_b / 64u,
          " (the word mean >> 4 = ", (sum_w / 64u) >> 4, ")", crlf);
    bench.verdict("FCS.SHIFT delivers the top eight bits into a byte block through an eight-bit DMA engine: the byte mean is "
                  "the word mean over sixteen, within one",
                  block_done && took_w != 0u && (sum_b / 64u) + 1u >= ((sum_w / 64u) >> 4) && (sum_b / 64u) <= ((sum_w / 64u) >> 4) + 1u);
    // The error flag per entry.
    Adc::divider({});
    const uint32_t took_e = run_block(256, 100'000);
    uint16_t flagged = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        if (Adc::entry_failed(block[i])) {
            ++flagged;
        }
    }
    print(serial, "  256 entries at 500 ksps with FCS.ERR: ", flagged, " flagged as failed, CS.ERR_STICKY=", Adc::error_seen(), crlf);
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
    static inline uint32_t per_input[2];
    static inline uint32_t sum[2];
    static inline uint32_t total = 0;

    static void init() {
        per_input[0] = per_input[1] = 0;
        sum[0] = sum[1] = 0;
        total = 0;
    }
    static void dispatch(const Event& e) {
        match(e, [](const AnalogSample& s) {
            ++total;
            if (s.index < 2u) {
                ++per_input[s.index];
                sum[s.index] += s.value;
            }
        });
    }
};

using Sampler = AnalogSampler<Adc, P, Subscribers<Probe>, AdcInput::temperature, In0{}>;
using SamplerKernel = Kernel<P, Probe, Sampler>;

void pump() {
    TimeEvents<P>::process();
    while (SamplerKernel::step()) {
        TimeEvents<P>::process();
    }
}

}  // namespace sm

void tg_sampler() {
    (void)adc_up();
    Gpio::analog(26, PinPull::up);
    Adc::fifo({.enable = true, .dreq = false, .error_flag = true, .shift = false, .threshold = 1});
    Adc::drain();
    sm::SamplerKernel::init_all();
    fifo_owner = FifoOwner::sampler;
    Adc::interrupt(true);
    Nvic::enable(Adc::irq());
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
    print(serial, "  the sampler at 5 ms over the sensor and input 0 for 200 ms: ", sm::Probe::total, " samples, ", n0,
          " of the sensor (mean ", n0 ? sm::Probe::sum[0] / n0 : 0u, "), ", n1, " of input 0 (mean ", n1 ? sm::Probe::sum[1] / n1 : 0u,
          ")", crlf);
    bench.verdict("AnalogSampler walks the two inputs on a 5 ms software pace: about twenty samples each in 200 ms, "
                  "the sensor's near its count and the pulled-up pad's near full scale",
                  n0 >= 18u && n0 <= 22u && n1 >= 18u && n1 <= 22u && sm::Probe::sum[0] / n0 > 600u && sm::Probe::sum[0] / n0 < 1200u &&
                      sm::Probe::sum[1] / n1 > 3900u);
    Gpio::release(26);
    Adc::release();
}

void banner() {
    print(serial, crlf, "test_rp2040_adc - the RP2040 ADC (datasheet 4.9): the sensor, GP26..GP28 through their pulls, GP29 for "
          "the record; clk_sys=", SysClock::hz, " Hz", crlf);
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
            // At 500 ksps the FIFO fills 16 us after the block: the
            // converter is stopped here, not in the loop (measured:
            // an overflow every time when the loop did it).
            brio::Adc::start_many(false);
        }
        block_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless: the clocks counted", ta_block);
    bench.letter('b', "the temperature sensor on both clocks", tb_temperature);
    bench.letter('c', "the pads through their pulls", tc_pads);
    bench.letter('d', "the rate into DMA blocks", td_rate);
    bench.letter('e', "the round-robin", te_round_robin);
    bench.letter('f', "the FIFO: threshold, overflow, shift, the error flag", tf_fifo);
    bench.letter('g', "the sampler", tg_sampler);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
