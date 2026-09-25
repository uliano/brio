// test_vx03_dac - the reference bench suite for the CH32V303's
// digital-to-analog converter: ch32vx03/dac.hpp over RM ch. 17, read
// back by the converter of ch32vx03/adc.hpp on the pads the two blocks
// share, and fed by the second DMA controller of ch32vx03/dma.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. DAC1's output is PA4 and DAC2's is PA5, and those two
// pads are also the converter's inputs 4 and 5 on every package: the ADC
// reads each channel through the bond pad they share. Every trigger this
// suite uses comes from inside the chip - the software trigger, a timer's
// TRGO - or from a pad the CPU toggles itself (PB9, EXTI line 9).
//
// THE CLOCK. The converter that judges the DAC is rated at 14 MHz and
// ADCCLK is PCLK2 divided by at most eight, so this suite runs the PLL on
// the HSI at 96 MHz exactly as test_vx03_adc does: ADCCLK 12 MHz.
//
// THE JUDGE IS RATIOMETRIC. The DAC's output is VDDA x DOR / 4096 (17.2.3)
// and the ADC's count is 4096 x VIN / VDDA, both against the same VREF+,
// so a code and the count it reads back are compared directly: the supply
// cancels, and the millivolts printed beside them - from the VDDA that
// VREFINT implies - are for the reader.
//
// THE PADS. PA4 and PA5 (the two outputs, in analog mode throughout) and
// PB9 (letter c's EXTI line 9). NEVER TOUCHED: PA9/PA10 (the console),
// PA13/PA14 (the debug port), PA11/PA12 (the USB pads), PC14/PC15 and
// PD0/PD1 (the crystals) - and PB2, toggled per command as every suite of
// this target does.
//
// What is exercised, letter by letter:
//   a  THE HOLDING REGISTERS: the gate, a configuration read back, the
//      configurations the chapter forbids refused, and the three
//      placements and three dual ones written, read back and reaching DOR
//      with no trigger selected
//   b  THE PADS, READ BY THE CONVERTER: seven codes on each channel,
//      buffered and not, against table 4-45's offset, gain and rails
//   c  THE TRIGGERS: the software one, the TRGO of every timer table 17-1
//      names that this build drives, and EXTI line 9 through each of its
//      two enables - each time the datum held until the trigger comes
//   d  THE NOISE GENERATOR: the preload on the first trigger and the whole
//      sequence against figure 17-5's register, masked at three widths
//   e  THE TRIANGLE: the counter walking up and down one step a trigger,
//      between the holding register and the amplitude MAMP selects
//   f  THE STREAM: DmaLoopEngine on DMA2's channel 3 playing a table into
//      DAC1 at the pace of TIM6's TRGO, the laps counted by the channel's
//      handler and every level found on the pad by the converter
//   g  THE DUAL STREAM: one engine on DAC1's request feeding both channels
//      through the dual holding register, one word a TIM6 update
//   h  BOTH CHANNELS AT ONCE: each pad following its own channel's
//      configuration - RM V2.3's 17.4.1 closed with a note that both would
//      follow channel 1's, and RM V2.5 deleted it
//
// build: boards = v303vc
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

TestBench<Serial> bench;

using Out1 = DacOut<1>;           ///< PA4, the converter's input 4
using Out2 = DacOut<2>;           ///< PA5, the converter's input 5
using ExtiPin = Pin<'B', 9>;      ///< the pad that raises EXTI line 9
constexpr uint8_t exti_line = dac_exti_line;
static_assert(Out1::adc_channel == 4u && Out2::adc_channel == 5u);

/// The engines of letters f and g, on DAC1's request - named through the
/// request and not by its number (table 11-3: DMA2's channel 3).
using Row1 = DmaRequestOf<DmaRequest::dac1>;
using Player = DmaLoopEngine<Row1::controller, Row1::channel, uint16_t>;
using DualPlayer = DmaLoopEngine<Row1::controller, Row1::channel, uint32_t>;
static_assert(Row1::controller == 2u && Row1::channel == 3u);

/// The timer that paces letters f and g: TIM6, code 0 of table 17-1 - the
/// basic timer whose one job is this TRGO, on the CH32V303RC and VC.
using Pacer = Tim<6>;
constexpr DacTrigger pacer_trigger = DacTrigger::tim6_trgo;
static_assert(dac_trigger_timer(pacer_trigger) == Pacer::instance);

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
// The judge, and the state the handlers share
// ---------------------------------------------------------------------------

/// What DMA2's channel 3 serves in the letter that is running.
enum class Role : uint8_t { none, player, dual };
volatile Role role = Role::none;
volatile uint16_t exti_calls = 0;

/// The board's supply, measured from VREFINT whenever the converter comes
/// up - for the millivolts printed, not for the verdicts.
uint16_t vdda_mv = 3300;

void converter_up() {
    (void)Adc<1>::init(clock, AdcConfig{.internal_sources = true});
    Adc<1>::sample_time_all(adc_sample_longest);
    Adc<1>::select(AdcInput::vrefint);
    vdda_mv = Adc<1>::vdda_mv(Adc<1>::read_settled(4));
}

/// One settled conversion of a DAC channel's pad, in counts.
uint16_t pad_counts(uint8_t ch) {
    Adc<1>::select_channel(dac_adc_channel(ch));
    (void)Adc<1>::read();
    return Adc<1>::read_settled(4);
}

/// The quicker read of letters f and g: one conversion, no settling.
uint16_t pad_once(uint8_t ch) {
    Adc<1>::select_channel(dac_adc_channel(ch));
    return Adc<1>::read();
}

uint16_t distance(uint16_t a, uint16_t b) {
    return a > b ? static_cast<uint16_t>(a - b) : static_cast<uint16_t>(b - a);
}

/// Everything this suite can have running, back to a known state.
void all_off() {
    Pfic::disable(dma_channel_irq(Row1::controller, Row1::channel));
    Pfic::clear_pending(dma_channel_irq(Row1::controller, Row1::channel));
    Pfic::disable(Irq::exti9_5);
    Pfic::clear_pending(Irq::exti9_5);
    Player::stop();
    DualPlayer::stop();
    Dma<2>::open();
    Dma<2>::stop_all();
    role = Role::none;
    Dac::release();
    Pacer::release();
    (void)Exti::interrupt(exti_line, false);
    (void)Exti::event(exti_line, false);
    (void)Exti::sense(exti_line, ExtiSense::none);
    (void)Exti::clear(exti_line);
    ExtiPin::release();
    Adc<1>::release();
    exti_calls = 0;
}

/// The block up with both pads in analog mode - 17.2.2.1 asks for that
/// BEFORE a channel is enabled.
void dac_up() {
    Dac::init();
    Out1::claim();
    Out2::claim();
}

// ===========================================================================
// a - the holding registers
// ===========================================================================
void ta_registers() {
    all_off();
    Dac::init();
    const bool gate = Dac::bus_clock();
    Out1::claim();
    Out2::claim();

    // One configuration of every field, read back, and the other channel's
    // sixteen bits left alone.
    const DacChannelConfig asked{.buffered = false,
                                 .triggered = true,
                                 .trigger = DacTrigger::tim4_trgo,
                                 .wave = DacWave::triangle,
                                 .amplitude = 7,
                                 .dma = true};
    const bool taken = Dac::configure(1, asked);
    const DacChannelConfig back = Dac::configuration(1);
    const bool read_back = back.buffered == asked.buffered && back.triggered &&
                           back.trigger == asked.trigger && back.wave == asked.wave &&
                           back.amplitude == asked.amplitude && back.dma;
    const bool other_alone = (Dac::regs().CTLR >> 16) == 0u;
    print(serial, "  DAC_CTLR with channel 1 configured: ", hex(Dac::regs().CTLR), crlf);
    bench.verdict("the block's gate opens with init()", gate);
    bench.verdict("a configuration of every field is written and read back, the buffer's "
                  "inverted bit and all, and channel 2's half of the register is untouched",
                  taken && read_back && other_alone);

    // What the chapter forbids is refused and nothing is written.
    const uint32_t before = Dac::regs().CTLR;
    const bool no_wave_untriggered = !Dac::configure(1, DacChannelConfig{.wave = DacWave::noise});
    const bool no_dma_on_software = !Dac::configure(
        1, DacChannelConfig{.triggered = true, .trigger = DacTrigger::software, .dma = true});
    const bool no_wide_amplitude = !Dac::configure(1, DacChannelConfig{.amplitude = 16});
    const bool untouched = Dac::regs().CTLR == before;
    bench.verdict("a generator with no trigger, a DMA on the software trigger and a five-bit "
                  "amplitude are refused, and the register is untouched",
                  no_wave_untriggered && no_dma_on_software && no_wide_amplitude && untouched);

    // The placements, with no trigger selected: each datum reaches DOR
    // one PB1 cycle after it is held (17.2.2.5), which is before the core
    // can read it back.
    (void)Dac::configure(1, DacChannelConfig{});
    (void)Dac::configure(2, DacChannelConfig{});
    (void)Dac::enable(1, true);
    (void)Dac::enable(2, true);
    wait_us(20);
    uint8_t right = 0;

    (void)Dac::write(1, 0x123);
    (void)Dac::write(2, 0x456);
    if (Dac::code(1) == 0x123u && Dac::code(2) == 0x456u && Dac::output(1) == 0x123u &&
        Dac::output(2) == 0x456u) {
        ++right;
    }
    (void)Dac::write_left(1, 0xABCD);
    (void)Dac::write_left(2, 0x789F);
    const uint16_t left1 = Dac::code(1);
    const uint16_t left2 = Dac::code(2);
    if (left1 == 0xABCu && left2 == 0x789u && Dac::output(1) == 0xABCu &&
        Dac::output(2) == 0x789u) {
        ++right;
    }
    (void)Dac::write8(1, 0x5A);
    (void)Dac::write8(2, 0xC3);
    const uint16_t byte1 = Dac::code(1);
    const uint16_t byte2 = Dac::code(2);
    if (byte1 == 0x5A0u && byte2 == 0xC30u && Dac::output(1) == 0x5A0u &&
        Dac::output(2) == 0xC30u) {
        ++right;
    }
    Dac::write_dual(0x111, 0x222);
    if (Dac::code(1) == 0x111u && Dac::code(2) == 0x222u && Dac::output(1) == 0x111u &&
        Dac::output(2) == 0x222u) {
        ++right;
    }
    Dac::write_dual_left(0x3330, 0x4440);
    const uint16_t dual_left1 = Dac::code(1);
    const uint16_t dual_left2 = Dac::code(2);
    if (dual_left1 == 0x333u && dual_left2 == 0x444u && Dac::output(1) == 0x333u &&
        Dac::output(2) == 0x444u) {
        ++right;
    }
    Dac::write_dual8(0x55, 0x66);
    const uint16_t dual_byte1 = Dac::code(1);
    const uint16_t dual_byte2 = Dac::code(2);
    if (dual_byte1 == 0x550u && dual_byte2 == 0x660u && Dac::output(1) == 0x550u &&
        Dac::output(2) == 0x660u) {
        ++right;
    }
    print(serial, "  left-aligned 0xABCD/0x789F read back as ", hex(left1), "/", hex(left2),
          "; 8-bit 0x5A/0xC3 as ", hex(byte1), "/", hex(byte2), crlf);
    print(serial, "  dual left 0x3330/0x4440 as ", hex(dual_left1), "/", hex(dual_left2),
          "; dual 8-bit 0x55/0x66 as ", hex(dual_byte1), "/", hex(dual_byte2), crlf);
    bench.verdict("all six placements land in the one twelve-bit holding register of each "
                  "channel - the left-aligned ones from bits 15:4 and 31:20, the 8-bit ones "
                  "in 11:4 - and reach DOR with no trigger selected",
                  right == 6u);
    all_off();
}

// ===========================================================================
// b - the pads, read by the converter
// ===========================================================================
void tb_pads() {
    all_off();
    converter_up();
    dac_up();
    constexpr uint8_t count = 7;
    constexpr uint16_t codes[count] = {0, 512, 1024, 2048, 3072, 3584, 4095};

    // [0] buffered, [1] not: the worst mid-scale distance on either pad,
    // and the two rails.
    uint16_t worst[2] = {};
    uint16_t bottom[2] = {};
    uint16_t top[2] = {};
    for (uint8_t b = 0; b < 2u; ++b) {
        const bool buffered = b == 0u;
        (void)Dac::configure(1, DacChannelConfig{.buffered = buffered});
        (void)Dac::configure(2, DacChannelConfig{.buffered = buffered});
        (void)Dac::enable(1, true);
        (void)Dac::enable(2, true);
        wait_us(20);   // tWAKEUP, 10 us at most (table 4-45)
        print(serial, buffered ? "  buffered:" : "  unbuffered:", crlf);
        for (uint8_t i = 0; i < count; ++i) {
            (void)Dac::write(1, codes[i]);
            (void)Dac::write(2, codes[i]);
            wait_us(50);   // tSETTLING, 4 us at most
            const uint16_t c1 = pad_counts(1);
            const uint16_t c2 = pad_counts(2);
            print(serial, "    code ", codes[i], ": PA4 ", c1, " counts = ",
                  Adc<1>::millivolts(c1, vdda_mv), " mV, PA5 ", c2, " counts = ",
                  Adc<1>::millivolts(c2, vdda_mv), " mV", crlf);
            if (i == 0u) {
                bottom[b] = c1 > c2 ? c1 : c2;
            } else if (i == count - 1u) {
                top[b] = c1 < c2 ? c1 : c2;
            } else {
                const uint16_t d = distance(c1, codes[i]) > distance(c2, codes[i])
                                       ? distance(c1, codes[i])
                                       : distance(c2, codes[i]);
                if (d > worst[b]) {
                    worst[b] = d;
                }
            }
        }
    }
    print(serial, "  the worst mid-scale distance, code to count: ", worst[0], " buffered, ",
          worst[1], " not; the supply ", vdda_mv, " mV by VREFINT", crlf);
    bench.verdict("buffered, every mid-scale code reads back within 48 counts of itself on "
                  "both pads (table 4-45: offset 12 mV, gain 0.4 %, INL 4 LSB)",
                  worst[0] <= 48u);
    bench.verdict("buffered, code 0 reads within 40 counts of ground and code 4095 within 60 "
                  "of full scale (table 4-45: 0 to 8 mV, and 3.29 V of 3.3)",
                  bottom[0] <= 40u && top[0] >= 4035u);
    bench.verdict("unbuffered, every mid-scale code reads back within 48 counts of itself - "
                  "the converter's own sampling is the only load",
                  worst[1] <= 48u);
    bench.verdict("unbuffered, both rails are reached closer still (table 4-45: 0 to 3 mV, "
                  "and 3.295 V of 3.3)",
                  bottom[1] <= 24u && top[1] >= 4050u);
    all_off();
}

// ===========================================================================
// c - the triggers
// ===========================================================================
/// A timer's TRGO as channel 1's trigger: the datum written after a first
/// trigger is HELD until the next update, then taken. A template on the
/// timer, so a timer this build's timer driver does not reach is a
/// discarded branch and a line that says so.
template <uint8_t n, DacTrigger t>
void timer_trigger(uint8_t& passed, uint8_t& tried) {
    static_assert(dac_trigger_timer(t) == n);
    if constexpr (tim_present(n) && dac_trigger_valid(t)) {
        using T = Tim<n>;
        ++tried;
        T::init();
        (void)T::configure(TimConfig{
            .prescaler = static_cast<uint16_t>(T::clock_hz(clock) / 1'000'000u - 1u),
            .period = 999});   // an update a millisecond
        const bool master = T::master(TimMasterMode::update);
        (void)Dac::configure(1, DacChannelConfig{.triggered = true, .trigger = t});
        (void)Dac::enable(1, true);
        (void)Dac::write(1, 0x100);
        T::enable(true);
        wait_us(2500);   // two updates: 0x100 is in DOR
        const uint16_t base = Dac::output(1);
        (void)Dac::write(1, 0xE00);
        const uint16_t held = Dac::output(1);
        Stopwatch w;
        while (Dac::output(1) != 0xE00u && w.us() < 3000UL) {
        }
        const uint32_t took = w.us();
        const bool moved = Dac::output(1) == 0xE00u;
        T::enable(false);
        T::release();
        print(serial, "  TIM", n, "'s TRGO (code ", static_cast<uint8_t>(t), "): DOR ", hex(base),
              ", still ", hex(held), " after the write, taken ", took, " us later", crlf);
        if (master && base == 0x100u && held == 0x100u && moved && took <= 1100UL) {
            ++passed;
        }
    } else {
        print(serial, "  TIM", n, " (code ", static_cast<uint8_t>(t),
              "): this build's timer driver does not reach it, so its trigger is not "
              "driven here",
              crlf);
    }
}

/// One rising edge on the EXTI pad with a fresh datum waiting: was it
/// taken?
bool edge_takes(uint16_t value) {
    (void)Dac::write(1, value);
    const bool held = Dac::output(1) != value;
    ExtiPin::set();
    wait_us(50);
    ExtiPin::clear();
    wait_us(50);
    return held && Dac::output(1) == value;
}

void tc_triggers() {
    all_off();
    dac_up();

    // THE SOFTWARE TRIGGER: TEN set and TSEL 111, the datum waiting in the
    // holding register until SWTRIG.
    (void)Dac::configure(1, DacChannelConfig{.triggered = true, .trigger = DacTrigger::software});
    (void)Dac::enable(1, true);
    (void)Dac::write(1, 0x0F0);
    (void)Dac::software_trigger(1);
    wait_us(5);
    const uint16_t base = Dac::output(1);
    (void)Dac::write(1, 0x0E1);
    wait_us(5);
    const uint16_t held = Dac::output(1);
    (void)Dac::software_trigger(1);
    const uint16_t taken = Dac::output(1);
    print(serial, "  software: DOR ", hex(base), ", still ", hex(held),
          " five microseconds after the write, ", hex(taken), " right after SWTRIG1", crlf);
    bench.verdict("with the software trigger selected a datum waits in the holding register "
                  "until SWTRIG, and reaches DOR one PB1 cycle after it - before the core can "
                  "look",
                  base == 0x0F0u && held == 0x0F0u && taken == 0x0E1u);

    // EVERY TIMER table 17-1 names that this build drives.
    uint8_t passed = 0;
    uint8_t tried = 0;
    timer_trigger<2, DacTrigger::tim2_trgo>(passed, tried);
    timer_trigger<4, DacTrigger::tim4_trgo>(passed, tried);
    timer_trigger<5, DacTrigger::tim5_trgo>(passed, tried);
    timer_trigger<6, DacTrigger::tim6_trgo>(passed, tried);
    timer_trigger<7, DacTrigger::tim7_trgo>(passed, tried);
    timer_trigger<8, DacTrigger::tim8_trgo>(passed, tried);
    bench.verdict("every timer TRGO driven here moves the datum on the timer's update and not "
                  "before - the held value read back after the write, the new one within a "
                  "period",
                  tried >= 2u && passed == tried);

    // EXTI LINE 9, the one hardware trigger that is not a timer, raised by
    // a pad of our own - through each of the line's two enables in turn.
    (void)Dac::configure(1, DacChannelConfig{.triggered = true, .trigger = DacTrigger::exti9});
    (void)Dac::enable(1, true);
    ExtiPin::output(false);
    const bool selected = Exti::select(exti_line, 'B');
    (void)Exti::sense(exti_line, ExtiSense::rising);
    const bool sensed_only = edge_takes(0x210);
    (void)Exti::event(exti_line, true);
    const bool by_event = edge_takes(0x321);
    (void)Exti::event(exti_line, false);
    Pfic::enable(Irq::exti9_5);
    (void)Exti::interrupt(exti_line, true);
    exti_calls = 0;
    const bool by_interrupt = edge_takes(0x432);
    (void)Exti::interrupt(exti_line, false);
    print(serial, "  EXTI line 9 from PB9: an edge on the line only sensed ",
          sensed_only ? "WAS" : "was not", " taken; with the EVENT enable it ",
          by_event ? "was" : "was not", "; with the INTERRUPT enable it ",
          by_interrupt ? "was" : "was not", " (", exti_calls, " line interrupts taken)", crlf);
    bench.verdict("the line is selected onto PB9's port", selected);
    bench.verdict("an edge on a line that is only SENSED moves nothing", !sensed_only);
    bench.verdict("and the line reaches the converter through its EVENT enable and not its "
                  "interrupt enable - the path the ADC's EXTI trigger takes too - while the "
                  "line's own handler runs for the edge the converter ignored",
                  by_event && !by_interrupt && exti_calls != 0u);
    all_off();
}

// ===========================================================================
// d - the noise generator
// ===========================================================================
/// Figure 17-5 as arithmetic: the register shifts RIGHT, bit 11 takes the
/// XOR of bits 6, 4, 1 and 0 with the NOR of all twelve - the anti-lock
/// that injects a one into a register at zero.
constexpr uint16_t lfsr_step(uint16_t v) {
    const uint16_t b = static_cast<uint16_t>(v & 0x0FFFu);
    const uint16_t lock = b == 0u ? uint16_t{1} : uint16_t{0};
    const uint16_t fb =
        static_cast<uint16_t>(((b >> 6) ^ (b >> 4) ^ (b >> 1) ^ b ^ lock) & 1u);
    return static_cast<uint16_t>((b >> 1) | (fb << 11));
}
static_assert(lfsr_step(0) == 0x800u);        // the anti-lock
static_assert(lfsr_step(0x0AAA) == 0x0D55u);   // bits 6, 4, 1, 0 of 0xAAA: 0, 0, 1, 0

/// `count` software triggers on a noise channel over `base`, each DOR
/// compared with the model. Returns how many matched; the first value and
/// the first eight are handed back for the report.
uint8_t noise_run(uint8_t mamp, uint16_t base, uint8_t count, uint16_t& first,
                  uint16_t (&head)[8]) {
    (void)Dac::configure(1, DacChannelConfig{.triggered = true,
                                             .trigger = DacTrigger::software,
                                             .wave = DacWave::noise,
                                             .amplitude = mamp});
    (void)Dac::write(1, base);
    (void)Dac::enable(1, true);
    wait_us(20);
    const uint16_t mask = dac_wave_amplitude(mamp);
    uint16_t model = dac_lfsr_preload;
    uint8_t matched = 0;
    for (uint8_t i = 0; i < count; ++i) {
        (void)Dac::software_trigger(1);
        wait_us(2);
        const uint16_t got = Dac::output(1);
        const uint16_t want = static_cast<uint16_t>((base + (model & mask)) & 0x0FFFu);
        if (i == 0u) {
            first = got;
        }
        if (i < 8u) {
            head[i] = got;
        }
        if (got == want) {
            ++matched;
        }
        model = lfsr_step(model);
    }
    // WAVE back to 00 resets the generator (17.2.5), and the enable
    // drops with the next configure().
    (void)Dac::wave(1, DacWave::none);
    return matched;
}

void td_noise() {
    all_off();
    dac_up();
    constexpr uint8_t runs = 64;
    uint16_t head[8] = {};
    uint16_t first4 = 0;
    uint16_t first8 = 0;
    uint16_t first12 = 0;
    const uint8_t m4 = noise_run(3, 0, runs, first4, head);
    const uint8_t m8 = noise_run(7, 0x800, runs, first8, head);
    const uint8_t m12 = noise_run(11, 0, runs, first12, head);
    print(serial, "  first values: ", hex(first4), " (mask 0x00F), ", hex(first8),
          " (0x800 + mask 0x0FF), ", hex(first12), " (mask 0xFFF); the preload is 0xAAA", crlf);
    print(serial, "  the twelve-bit run begins ", hex(head[0]), " ", hex(head[1]), " ",
          hex(head[2]), " ", hex(head[3]), " ", hex(head[4]), " ", hex(head[5]), " ",
          hex(head[6]), " ", hex(head[7]), crlf);
    print(serial, "  of ", runs, " triggers the model of figure 17-5 predicted ", m4, ", ", m8,
          " and ", m12, crlf);
    bench.verdict("the first trigger adds the LFSR's preload 0xAAA, masked by MAMP, to the "
                  "holding register (17.2.5)",
                  first4 == 0x00Au && first8 == 0x8AAu && first12 == 0xAAAu);
    bench.verdict("and every value after it is figure 17-5's register - a right shift, bit 11 "
                  "fed by bits 6, 4, 1 and 0 - masked at all three widths",
                  m4 == runs && m8 == runs && m12 == runs);
    all_off();
}

// ===========================================================================
// e - the triangle
// ===========================================================================
void te_triangle() {
    all_off();
    dac_up();
    constexpr uint8_t mamp = 2;
    constexpr uint16_t amplitude = dac_wave_amplitude(mamp);   // 7
    constexpr uint16_t base = 0x400;
    constexpr uint8_t count = 40;
    (void)Dac::configure(1, DacChannelConfig{.triggered = true,
                                             .trigger = DacTrigger::software,
                                             .wave = DacWave::triangle,
                                             .amplitude = mamp});
    (void)Dac::write(1, base);
    (void)Dac::enable(1, true);
    wait_us(20);
    uint16_t seen[count] = {};
    for (uint8_t i = 0; i < count; ++i) {
        (void)Dac::software_trigger(1);
        wait_us(2);
        seen[i] = Dac::output(1);
    }
    bool in_band = true;
    bool single_steps = true;
    uint8_t tops = 0;
    uint8_t bottoms = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (seen[i] < base || seen[i] > base + amplitude) {
            in_band = false;
        }
        if (i != 0u && distance(seen[i], seen[i - 1u]) > 1u) {
            single_steps = false;
        }
        if (seen[i] == base + amplitude) {
            ++tops;
        }
        if (i != 0u && seen[i] == base) {
            ++bottoms;
        }
    }
    print(serial, "  MAMP ", mamp, " over ", hex(base), ": ", seen[0] - base, " ", seen[1] - base,
          " ", seen[2] - base, " ", seen[3] - base, " ", seen[4] - base, " ", seen[5] - base,
          " ", seen[6] - base, " ", seen[7] - base, " ", seen[8] - base, " ", seen[9] - base, " ",
          seen[10] - base, " ", seen[11] - base, " ", seen[12] - base, " ", seen[13] - base, " ",
          seen[14] - base, " ", seen[15] - base, " ...", crlf);
    (void)Dac::wave(1, DacWave::none);
    bench.verdict("the first trigger adds the counter's starting zero (17.3.3: the counter is "
                  "added, THEN updated)",
                  seen[0] == base);
    bench.verdict("every trigger moves DOR by at most one step, and never outside the holding "
                  "register plus the amplitude MAMP selects (2^(MAMP+1) - 1)",
                  in_band && single_steps);
    bench.verdict("and the walk turns at both ends: forty triggers reach the top and come "
                  "back to the holding register more than once",
                  tops >= 2u && bottoms >= 2u);
    all_off();
}

// ===========================================================================
// f - the stream: DMA2's channel 3 playing a table into DAC1
// ===========================================================================
constexpr uint8_t levels = 8;
constexpr uint16_t table[levels] = {256, 768, 1280, 1792, 2304, 2816, 3328, 3840};

/// The table entry a count is closest to, or `levels` when it is near none.
uint8_t level_of(uint16_t counts) {
    for (uint8_t i = 0; i < levels; ++i) {
        if (distance(counts, table[i]) <= 64u) {
            return i;
        }
    }
    return levels;
}

void pace_at_1khz() {
    Pacer::init();
    (void)Pacer::configure(TimConfig{
        .prescaler = static_cast<uint16_t>(Pacer::clock_hz(clock) / 1'000'000u - 1u),
        .period = 999});
    (void)Pacer::master(TimMasterMode::update);
}

void tf_stream() {
    all_off();
    converter_up();
    Adc<1>::sample_time_all(AdcSampleTime::cycles28_5);
    dac_up();
    pace_at_1khz();
    Dac::configure<1, DacChannelConfig{.triggered = true,
                                       .trigger = pacer_trigger,
                                       .dma = true}>();
    role = Role::player;
    Dac::claim_stream<1, Player>();
    const bool started = Player::start(table, levels);
    (void)Dac::enable(1, true);
    Pacer::enable(true);

    // Twenty milliseconds of the pad, read as fast as the converter goes:
    // two and a half laps of an eight-step table at a step a millisecond.
    uint16_t samples = 0;
    uint16_t on_level = 0;
    uint16_t levels_seen = 0;
    Stopwatch w;
    while (w.us() < 20'000UL) {
        const uint8_t l = level_of(pad_once(1));
        ++samples;
        if (l < levels) {
            ++on_level;
            levels_seen = static_cast<uint16_t>(levels_seen | (1u << l));
        }
    }
    Pacer::enable(false);
    const uint32_t laps = Player::laps();
    const uint32_t faults = Player::faults();
    Player::stop();
    (void)Dac::dma(1, false);
    print(serial, "  ", samples, " conversions of PA4 in 20 ms: ", on_level,
          " on a table level, levels seen ", hex(levels_seen), "; the engine counted ", laps,
          " laps and ", faults, " faults", crlf);
    bench.verdict("the loop engine starts on the channel DAC1's request is wired to (DMA2's "
                  "channel 3, table 11-3)",
                  started);
    bench.verdict("each TRGO raises the request that pours the next table entry into the "
                  "holding register: the channel's own handler counted the laps a timer at "
                  "1 kHz makes of eight entries in 20 ms",
                  laps >= 2u && laps <= 3u && faults == 0u);
    bench.verdict("and the pad carries every level of the table, the converter finding "
                  "nine samples in ten on one of them",
                  levels_seen == 0xFFu && on_level * 10u >= samples * 9u);
    all_off();
}

// ===========================================================================
// g - the dual stream: one request, both channels
// ===========================================================================
constexpr uint32_t pack(uint16_t one, uint16_t two) {
    return (static_cast<uint32_t>(two) << 16) | one;
}
constexpr uint8_t pairs = 4;
/// Every pair sums to 4096: an invariant a converter can check from two
/// back-to-back reads without knowing which pair it caught.
constexpr uint32_t dual_table[pairs] = {pack(512, 3584), pack(1536, 2560), pack(2560, 1536),
                                        pack(3584, 512)};

void tg_dual() {
    all_off();
    converter_up();
    Adc<1>::sample_time_all(AdcSampleTime::cycles28_5);
    dac_up();
    pace_at_1khz();
    (void)Dac::configure(1, DacChannelConfig{.triggered = true, .trigger = pacer_trigger});
    (void)Dac::configure(2, DacChannelConfig{.triggered = true, .trigger = pacer_trigger});
    role = Role::dual;
    Dac::claim_dual_stream<DualPlayer, DacFormat::right12>();
    const bool only_dac1 = Dac::dma(1) && !Dac::dma(2);
    const bool started = DualPlayer::start(dual_table, pairs);
    (void)Dac::enable(1, true);
    (void)Dac::enable(2, true);
    Pacer::enable(true);
    wait_us(3000);

    // The registers: DOR1 and DOR2 read back to back, a pair of the table
    // unless an update fell between the two reads.
    uint16_t reads = 0;
    uint16_t consistent = 0;
    uint16_t sums_right = 0;
    uint16_t conversions = 0;
    Stopwatch w;
    while (w.us() < 10'000UL) {
        const uint16_t d1 = Dac::output(1);
        const uint16_t d2 = Dac::output(2);
        ++reads;
        for (uint8_t i = 0; i < pairs; ++i) {
            if (pack(d1, d2) == dual_table[i]) {
                ++consistent;
                break;
            }
        }
        const uint16_t a1 = pad_once(1);
        const uint16_t a2 = pad_once(2);
        ++conversions;
        if (distance(static_cast<uint16_t>(a1 + a2), 4096u) <= 96u) {
            ++sums_right;
        }
    }
    Pacer::enable(false);
    const uint32_t laps = DualPlayer::laps();
    DualPlayer::stop();
    print(serial, "  ", reads, " register pairs read, ", consistent,
          " of them a pair of the table; ", conversions, " pad pairs, ", sums_right,
          " summing to full scale; ", laps, " laps", crlf);
    bench.verdict("one stream on DAC1's request feeds both channels: DMAEN1 set and DMAEN2 "
                  "clear, the engine started on DMA2's channel 3",
                  only_dac1 && started);
    bench.verdict("each word of the table lands in both DORs on one trigger - nine register "
                  "pairs in ten are a pair of the table",
                  consistent * 10u >= reads * 9u && laps >= 2u);
    bench.verdict("and both pads carry it: nine back-to-back conversions in ten sum to full "
                  "scale, as every pair of the table does",
                  sums_right * 10u >= conversions * 9u);
    all_off();
}

// ===========================================================================
// h - both channels at once
// ===========================================================================
void th_both() {
    all_off();
    converter_up();
    dac_up();

    // Channel 1: a triangle of amplitude 1023 over 512, stepped by the
    // software trigger. Channel 2: a fixed 3072, no trigger, no wave.
    (void)Dac::configure(1, DacChannelConfig{.triggered = true,
                                             .trigger = DacTrigger::software,
                                             .wave = DacWave::triangle,
                                             .amplitude = 9});
    (void)Dac::write(1, 512);
    (void)Dac::configure(2, DacChannelConfig{});
    (void)Dac::write(2, 3072);
    (void)Dac::enable(1, true);
    (void)Dac::enable(2, true);
    wait_us(20);
    for (uint16_t i = 0; i < 700u; ++i) {
        (void)Dac::software_trigger(1);
    }
    wait_us(50);
    const uint16_t dor1 = Dac::output(1);
    const uint16_t dor2 = Dac::output(2);
    const uint16_t pa4 = pad_counts(1);
    const uint16_t pa5 = pad_counts(2);

    // Each channel alone.
    (void)Dac::enable(1, false);
    wait_us(50);
    const uint16_t pa5_alone = pad_counts(2);
    (void)Dac::enable(2, false);
    (void)Dac::enable(1, true);
    wait_us(50);
    const uint16_t pa4_alone = pad_counts(1);
    (void)Dac::wave(1, DacWave::none);

    print(serial, "  both enabled: DOR1 ", dor1, " and PA4 reads ", pa4, "; DOR2 ", dor2,
          " and PA5 reads ", pa5, crlf);
    print(serial, "  alone: PA5 ", pa5_alone, " with channel 1 off, PA4 ", pa4_alone,
          " with channel 2 off", crlf);
    bench.verdict("with both channels enabled each pad follows its OWN channel: PA4 the "
                  "triangle's DOR1, PA5 channel 2's fixed code - not channel 1's configuration "
                  "on both, as RM V2.3's note had it",
                  distance(pa4, dor1) <= 48u && distance(pa5, 3072u) <= 48u && dor2 == 3072u);
    bench.verdict("and each channel alone drives its own pad the same way",
                  distance(pa5_alone, 3072u) <= 48u && distance(pa4_alone, dor1) <= 48u);
    all_off();
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf, "test_vx03_dac - the DAC of RM ch. 17, read by the ADC on PA4/PA5",
          crlf, "  no wires: the two outputs are the converter's inputs 4 and 5", crlf);
    bench.menu();
}

}  // namespace

// DMA2's channel 3 carries DAC1's request: the player of letter f or the
// dual one of letter g.
extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() {
    if (role == Role::player) {
        const uint8_t f = Player::service();
        if ((f & Player::flag_error) != 0u) {
            Player::fail();
        } else if ((f & Player::flag_complete) != 0u) {
            Player::lap();
        }
    } else if (role == Role::dual) {
        const uint8_t f = DualPlayer::service();
        if ((f & DualPlayer::flag_error) != 0u) {
            DualPlayer::fail();
        } else if ((f & DualPlayer::flag_complete) != 0u) {
            DualPlayer::lap();
        }
    } else {
        (void)brio::DmaChannel<2, 3>::isr();
    }
}

extern "C" BRIO_CH32_INTERRUPT void exti9_5_handler() {
    const uint32_t up = brio::Exti::isr(brio::Exti::vector_lines(brio::Irq::exti9_5));
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

    bench.letter('a', "the holding registers: six placements, the configuration read back",
                 ta_registers);
    bench.letter('b', "the pads, read by the converter, buffered and not", tb_pads);
    bench.letter('c', "the triggers: software, the timers' TRGO, EXTI line 9", tc_triggers);
    bench.letter('d', "the noise generator against figure 17-5", td_noise);
    bench.letter('e', "the triangle generator", te_triangle);
    bench.letter('f', "the stream: DMA2's channel 3 playing a table into DAC1", tf_stream);
    bench.letter('g', "the dual stream: one request, both channels", tg_dual);
    bench.letter('h', "both channels at once, each on its own configuration", th_both);

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
