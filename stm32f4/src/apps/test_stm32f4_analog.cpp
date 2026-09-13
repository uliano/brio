// test_stm32f4_analog - the reference bench suite for the STM32F4's
// ANALOG CONVERTERS: the ADC (RM0090 ch. 13, RM0390 ch. 13, RM0383
// ch. 11), the DAC (RM0090 ch. 14, RM0390 ch. 14) and, through them,
// util/analog.hpp and util/analog_sampler.hpp on this silicon.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// ONE SUITE FOR TWO CHAPTERS, and the reason is the silicon: this
// family's DAC has no internal route at all - 14.3's note connects the
// pad to the converter the moment a channel is enabled and there is no
// code that keeps it off - so the ONLY path from the DAC to the ADC is
// the BOND PAD the two share. PA4 is DAC_OUT1 and ADC12_IN4, PA5 is
// DAC_OUT2 and ADC12_IN5, on every part of the family that has both. A
// converter written and read back through one pad is this board's whole
// analog instrument, and splitting the two chapters would mean writing
// that bring-up twice.
//
// NOTHING TO WIRE. Four techniques carry it:
//   1. THE ZERO-LENGTH WIRE. The DAC drives an ADC input through one bond
//      pad and nothing else - linearity, settling, the wave generators
//      and the analog watchdog all measured against a source this program
//      sets itself.
//   2. THE FACTORY VALUES AS THE SCALE. VREFINT and its calibration at
//      0x1FFF7A2A give VDDA in millivolts with no meter; TS_CAL1 and
//      TS_CAL2 at 0x1FFF7A2C and 0x1FFF7A2E give the junction
//      temperature. All three are ADC results taken at 3.3 V, so the
//      arithmetic is a ratio and the board's own supply comes out.
//   3. THE CORE'S OWN CLOCK AS THE RULER. SysTick's VAL counts HCLK, and
//      a conversion time is measured DIFFERENTIALLY - the same loop at
//      two sampling times - so the polling overhead cancels and what is
//      left is the chapter's own cycle count.
//   4. THE EXTI's SOFTWARE TRIGGER AS A HARDWARE ONE. The last code of
//      both trigger tables is a pad's EXTI line (11 for the regular
//      group, 15 for the injected one), and EXTI_SWIER raises a line with
//      no pad involved - so the external-trigger path is exercised
//      without a timer and without a wire.
//
// THE PADS. PA4 and PA5 are the DAC's own on every part of this family,
// and what else a board hangs on them is the board's business: on the
// Nucleo-F446RE PA5 also carries LD2, which this suite USES - a LED and
// its resistor are a load, and letter d is the difference between a
// buffered output and an unbuffered one driving it. Every letter that
// claims PA5 puts it back an output driven low before it returns.
//
// A PART WITH NO PARTNER. The F401, F411 and F412 have no DAC at all and
// their device header says so, so brio's `Dac` does not exist there: nine
// letters are compiled out on such a part and each says what it needs
// when it is asked for. What stays is everything the converter can do
// against its own internal channels - the scale, the conversion times,
// the overrun modes, the errata, the sampler - and the letters that lose
// only their STIMULUS (the watchdog, the injected group) fall back to
// VREFINT, which is a fixed source this program does not choose.
//
// NOTHING FORCED. No flash is written, no option byte, no reset: every
// letter leaves the converters powered down, the internal sources off,
// the multi-ADC mode independent, both DAC pads parked, and every DMA
// stream it used stopped with its vector disabled again.
//
// What is exercised, letter by letter:
//   a  the block: the reserve's facts, the prescaler and fADC, the
//      factory values, the pads, and every refusal the config makes
//   b  the DAC as an actuator: the holding register, the output register
//      and the cycles between them, the three data formats, both triggers
//   c  the zero-length wire: the DAC's transfer curve read back through
//      ADC12_IN4, monotonic, with the buffer's own swing limits
//   d  buffered against unbuffered, on a free pad and on the LED's
//   e  the settling time, by sampling at increasing delays after a step
//   f  the conversion time in ADCCLK cycles, measured differentially per
//      resolution and per sampling time
//   g  the scale: VDDA from VREFINT, the junction temperature from both
//      formulas, VBAT/4 - and which of the two switches wins the channel
//      they share
//   h  the analog watchdog on a DAC step, single and all-channel, and its
//      thresholds proved independent of the alignment
//   i  the injected group: the tail rule, the offset and its sign,
//      auto-injection, and an injected conversion interrupting a
//      continuous regular one
//   j  dual regular simultaneous mode on ADC1 and ADC2 - two converters
//      on two DAC pads, and then both on ONE
//   k  the wave generators: the LFSR's first two words exactly, the
//      triangle's amplitude, and both seen on the pad by the ADC
//   l  overrun: raised where the chapter enables detection, absent where
//      it does not, and the recovery
//   m  the sequencer-during-conversion erratum staged, and the driver's
//      refusal that answers it
//   n  the ART prefetch against the ADC's spread (the errata's own
//      workaround, measured)
//   o  the external trigger path, through the EXTI's software trigger
//   p  AnalogSampler inside a REAL KERNEL walking three inputs
//   q  the ADC's DMA: four ranks into memory on one trigger, and the
//      same four with nobody taking them
//   r  the DAC's DMA: a table played by a hardware trigger, and the
//      underrun a spent stream leaves behind
//
// build: boards = f429zi,f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/adc.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dac.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/analog.hpp"
#include "util/analog_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
constexpr bool led_is_dac_pad = false;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
constexpr bool led_is_dac_pad = false;
#else
using Led = Pin<'A', 5>;
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
/// The Nucleo's LD2 IS DAC_OUT2 and ADC12_IN5: this suite uses it as an
/// analog pad and as a load, and puts it back an output driven low.
constexpr bool led_is_dac_pad = true;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial, 20> bench;

using Adc1 = Adc<1>;
using Pa4 = Pin<'A', 4>;
using Pa5 = Pin<'A', 5>;
using In4 = AnalogIn<Pa4>;   ///< DAC_OUT1's pad, ADC12_IN4
using In5 = AnalogIn<Pa5>;   ///< DAC_OUT2's pad, ADC12_IN5

/// The two DAC channels by index, so that no letter counts from one.
constexpr uint8_t dac_ch_pa4 = 0;
constexpr uint8_t dac_ch_pa5 = 1;

// Which of the two DAC pads is FREE and which carries a LOAD is a fact of
// the board: on the Nucleo-64 PA5 is the LED's pad (a resistor and a
// diode to ground) and PA4 is free; on the STM32F429I-DISC1 PA4 belongs
// to the display's VSYNC and something pulls it high, PA5 is the free
// one. The transfer curve wants the free pad; letter d wants both.
#if defined(STM32F429xx)
constexpr uint8_t free_ch = dac_ch_pa5;
constexpr uint8_t loaded_ch = dac_ch_pa4;
using FreeIn = In5;
using LoadedIn = In4;
constexpr const char* free_pad_name = "PA5";
constexpr const char* loaded_pad_name = "PA4 (the display's VSYNC)";
#else
constexpr uint8_t free_ch = dac_ch_pa4;
constexpr uint8_t loaded_ch = dac_ch_pa5;
using FreeIn = In4;
using LoadedIn = In5;
constexpr const char* free_pad_name = "PA4";
constexpr const char* loaded_pad_name = "PA5 (the LED's)";
#endif

constexpr bool have_dac = dac_present();
constexpr bool have_adc2 = adc_present(2);

/// APB2 is what the converter's prescaler divides - 90 MHz on the two
/// 180 MHz boards, 50 MHz on the black pill.
constexpr uint32_t pclk2 = SysClock::pclk2_hz;

/// Let the console's own transmitter fall silent before a timed
/// measurement: its interrupt would land in the middle of one.
void console_drain() {
    const uint32_t t0 = Ticker::ticks();
    while (!Serial::tx_idle() && Ticker::ticks() - t0 < 200u) {
    }
}

// =============================================================================
// The ruler: SysTick's VAL, accumulated across its wraps
// =============================================================================

struct Cycles {
    uint32_t last = 0;
    uint32_t total = 0;

    static uint32_t period() { return SysTick->LOAD + 1u; }
    void start() {
        total = 0;
        last = SysTick->VAL;
    }
    /// Must be called at least once per SysTick period, which every loop
    /// below does by construction (a conversion is far shorter than 1 ms).
    void mark() {
        const uint32_t now = SysTick->VAL;
        total += (last >= now) ? (last - now) : (last + period() - now);
        last = now;
    }
};

/// How many HCLK cycles one ADCCLK cycle is worth, as a Q8 fixed-point
/// ratio so that a 4:1 or 8:1 division is exact and anything else is
/// close: HCLK / fADC.
uint32_t core_cycles_per_adc_cycle_q8() {
    const uint32_t f = AdcCommon::adc_hz(pclk2);
    // 180 MHz times 256 is past what 32 bits hold, so the accumulator
    // names its width.
    return f == 0u ? 0u
                   : static_cast<uint32_t>((static_cast<uint64_t>(SysClock::hz) * 256ULL) / f);
}

// =============================================================================
// Bring-up shared by every letter
// =============================================================================

/// The converter as most letters want it: one 12-bit conversion of one
/// channel when asked, a long sampling time (the pads here are driven by
/// a DAC buffer or an internal source, and 480 cycles costs 22 us and
/// removes every source-impedance question from the numbers below).
AdcConfig base_config() {
    AdcConfig c{};
    c.resolution = AdcRes::bits12;
    c.eoc_per_conversion = true;
    return c;
}

bool adc_up(const AdcConfig& c, AdcSampleTime smp = AdcSampleTime::cycles480) {
    AdcCommon::reset();
    (void)AdcCommon::multi(AdcMulti::independent);
    if (!Adc1::init(clock, c)) {
        return false;
    }
    Adc1::sample_time_all(smp);
    Adc1::clear_flags(AdcFlag::all);
    return true;
}

// A NOTE ON THE GUARDS BELOW. Where a part has no DAC the TYPE does not
// exist (stm32f4/dac.hpp compiles its resource only under the header's
// own DAC_BASE), and a name that is not dependent is looked up even in
// the discarded branch of an `if constexpr` - so the DAC's half of this
// suite is behind the preprocessor and not behind a constant. `have_dac`
// stays for the places where only a VALUE is needed.

/// The DAC as most letters want it: the block up, one channel buffered
/// and untriggered on its own pad, its output at `code`.
#if defined(DAC_BASE)
/// One channel, with the BLOCK already up: `Dac::init()` resets every
/// register of the peripheral, so a letter that wants both channels
/// resets once and then brings each channel up with this.
bool dac_channel_up(uint8_t ch, uint16_t code, bool buffered = true) {
    if (!Dac::channel_valid(ch)) {
        return false;
    }
    if (ch == dac_ch_pa4) {
        Dac::claim_pad<Pa4>();
    } else {
        Dac::claim_pad<Pa5>();
    }
    DacChannelConfig cfg{};
    cfg.buffered = buffered;
    if (!Dac::configure(ch, cfg)) {
        return false;
    }
    (void)Dac::write(ch, code);
    (void)Dac::enable(ch, true);
    (void)delay_us(clock, 20);   // tWAKEUP, 14.3.1
    return true;
}

/// The block and one channel, for a letter that wants only one.
bool dac_up(uint8_t ch, uint16_t code, bool buffered = true) {
    Dac::init();
    return dac_channel_up(ch, code, buffered);
}
#endif

/// Every letter's exit: the converters down, the pads parked, the LED an
/// output driven low again where it is one of the pads.
void analog_down() {
    Adc1::release();
#if defined(ADC2_BASE)
    Adc<2>::release();
    Adc<3>::release();
#endif
    AdcCommon::release();
#if defined(DAC_BASE)
    Dac::release();
#endif
    Pa4::release();
    if constexpr (led_is_dac_pad) {
        Led::output(false);
    } else {
        Pa5::release();
    }
}

/// A DAC code written, given time to settle and read back on its own pad
/// by ADC1 - the zero-length wire in one call.
#if defined(DAC_BASE)
uint16_t loop_read(uint8_t ch, uint16_t code, uint8_t settle_reads = 2) {
    (void)Dac::write(ch, code);
    (void)delay_us(clock, 30);
    return Adc1::read_settled(settle_reads);
}
#endif
constexpr uint16_t max_of(uint16_t a, uint16_t b) { return a > b ? a : b; }

uint16_t abs_diff(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }

// =============================================================================
// The external trigger, with no timer and no wire
// =============================================================================

/// One SOFTWARE edge on `line`, and whether the converter answered it.
/// The pending bit is taken down FIRST: SWIER does not clear itself, and
/// a second write over a standing edge is no edge at all.
bool trigger_converts(uint8_t line, bool injected) {
    (void)Exti::clear(line);
    Adc1::clear_flags(AdcFlag::all);
    (void)Exti::trigger(line);
    for (uint32_t i = 0; i < 20000u; ++i) {
        if (injected ? Adc1::injected_ready() : Adc1::ready()) {
            return true;
        }
    }
    return false;
}

// A REAL EDGE, AND STILL NO WIRE: an output pad feeds its OWN EXTI line,
// which is the technique the EXTI chapter's own suite uses. The pads have
// to be pin 11 and pin 15 of some port, because a line's number IS its
// pin number - and which port bonds and frees those two is a BOARD fact,
// so the legs that need them run where this suite knows the answer and
// say so where it does not.
#if defined(STM32F446xx)
// PC11 and PB15, both free on the Nucleo's Morpho header. Port B has no
// pin 11 on this package at all, which is what a first attempt at PB11
// found out by getting no edge from it.
using TrigPad11 = Pin<'C', 11>;
using TrigPad15 = Pin<'B', 15>;
constexpr bool have_trigger_pads = true;

/// Point `line` at this pad, rising, with both masks as asked. STEAL and
/// not select: a letter that armed the line for the software trigger
/// first holds it, and a line already in use is what select() refuses.
template <class Pad>
void arm_pad_line(uint8_t line, bool with_event = true, bool with_interrupt = false) {
    Pad::output(false);
    (void)Exti::steal(line, Pad::port_letter);
    (void)Exti::sense(line, ExtiSense::rising);
    (void)Exti::event(line, with_event);
    // The NVIC line is never enabled in this suite, so an open IMR costs
    // a pending bit and no handler.
    (void)Exti::interrupt(line, with_interrupt);
    (void)Exti::clear(line);
}

/// One rising edge on the pad, held long enough to be seen: an edge
/// shorter than one APB2 period is not (the EXTI chapter's measurement).
template <class Pad>
void pad_edge(uint8_t line) {
    (void)Exti::clear(line);
    Pad::set();
    (void)delay_us(clock, 2);
    Pad::clear();
    (void)delay_us(clock, 2);
}

template <class Pad>
bool pad_edge_converts(uint8_t line, bool injected, bool with_event, bool with_interrupt) {
    arm_pad_line<Pad>(line, with_event, with_interrupt);
    Adc1::clear_flags(AdcFlag::all);
    Pad::set();
    (void)delay_us(clock, 2);
    bool ok = false;
    for (uint32_t i = 0; i < 20000u && !ok; ++i) {
        ok = injected ? Adc1::injected_ready() : Adc1::ready();
    }
    Pad::clear();
    (void)Exti::clear(line);
    Pad::release();
    return ok;
}
#else
constexpr bool have_trigger_pads = false;
#endif

#if !defined(DAC_BASE)
void needs_dac(const char* what) {
    print(serial, "  this part has no DAC: ", what, " needs one and claims nothing", crlf);
}
#endif

// =============================================================================
// a - the block, the clock and the facts
// =============================================================================
void ta_block() {
    print(serial, "  converters ", adc_instances(), ", common block at ", hex(adc_common_base()),
          ", one reset line for all of them", crlf);
    print(serial, "  PCLK2 ", pclk2 / 1000u, " kHz", crlf);

    bench.verdict("the reserve counts the converters the header declares",
                  adc_instances() == (have_adc2 ? 3u : 1u));
    bench.verdict("ADC1 is the master and every converter shares one vector",
                  Adc1::is_master && Adc1::irq() == ADC_IRQn);

    const bool up = adc_up(base_config());
    const AdcPrescaler p = AdcCommon::prescaler();
    const uint32_t f = AdcCommon::adc_hz(pclk2);
    print(serial, "  init: ADCPRE /", adc_prescaler_divisor(p), " -> fADC ", f / 1000u, " kHz (",
          adc_max_hz / 1000u, " kHz is the datasheets' ceiling at 3.3 V)", crlf);
    bench.verdict("the converter came up", up);
    bench.verdict("init() picked the SMALLEST division that keeps fADC legal",
                  p == adc_prescaler_for(pclk2) && f <= adc_max_hz && f >= adc_min_hz);
    bench.verdict("and half the bus was refused where half the bus is too fast",
                  (pclk2 / 2u <= adc_max_hz) == (p == AdcPrescaler::div2));
    bench.verdict("ADON is set and the converter is powered", Adc1::powered());

    // THE ACKNOWLEDGEMENT, which is what makes every sequence verb below
    // usable more than once: STRT is set by the silicon and taken down by
    // nothing but a write, so a driver that did not clear it when the
    // datum is read would answer "a conversion is in flight" for ever
    // after the first one - and refuse every selection in silence.
    AdcCommon::internal_sources(true);
    (void)Adc1::select_sync(AdcInput::vrefint);
    (void)delay_us(clock, 20);
    (void)Adc1::read();
    bench.verdict("a completed conversion is ACKNOWLEDGED by the read of its datum, so the "
                  "next selection is accepted and not refused for ever",
                  !Adc1::converting() && Adc1::select_sync(In4{}));
    AdcCommon::internal_sources(false);

    // The internal channels, as the reserve reads the manual.
    print(serial, "  internal channels: VREFINT ", adc_input_channel(AdcInput::vrefint),
          ", sensor ", adc_input_channel(AdcInput::temperature), ", VBAT ",
          adc_input_channel(AdcInput::vbat), " (/", AdcCommon::vbat_divider(), ")",
          AdcCommon::sensor_shares_vbat() ? ", sensor and battery SHARE the channel" : "", crlf);
    bench.verdict("the internal channels are the manual's numbers for this class",
                  adc_input_valid(AdcInput::vrefint) && adc_input_valid(AdcInput::temperature) &&
                      adc_input_valid(AdcInput::vbat));

    print(serial, "  factory: VREFINT_CAL ", AdcFactory::vrefint_cal(), ", TS_CAL1 ",
          AdcFactory::ts_cal1(), " (", AdcFactory::ts_cal1_celsius, " C), TS_CAL2 ",
          AdcFactory::ts_cal2(), " (", AdcFactory::ts_cal2_celsius, " C), all at ",
          AdcFactory::characterization_mv, " mV", crlf);
    bench.verdict("the three factory measurements are present and usable",
                  AdcFactory::plausible());
    bench.verdict("and they are ordered as two temperature points must be",
                  AdcFactory::ts_cal2() > AdcFactory::ts_cal1());

    // The pad map, and the refusals of the config.
    bench.verdict("the pads carry the channels every datasheet of the family gives",
                  In4::channel == 4u && In5::channel == 5u);
    AdcConfig bad = base_config();
    bad.discontinuous = true;
    bad.injected_discontinuous = true;
    bench.verdict("a config with both discontinuous modes is refused",
                  !Adc1::config_valid(bad) && !Adc1::configure(bad));
    bad = base_config();
    bad.auto_injected = true;
    bad.injected_trigger_edge = AdcEdge::rising;
    bench.verdict("an auto-injected group with an external trigger is refused",
                  !Adc1::configure(bad));
    bench.verdict("a sequence longer than sixteen is refused, and one of zero too", [] {
        static const uint8_t seq[1] = {4};
        return !Adc1::regular_sequence(seq, 17) && !Adc1::regular_sequence(seq, 0);
    }());
    bench.verdict("an injected sequence longer than four is refused", [] {
        static const uint8_t seq[1] = {4};
        return !Adc1::injected_sequence(seq, 5);
    }());
    bench.verdict("a watchdog threshold past twelve bits is refused",
                  !Adc1::watchdog_thresholds(0, 0x1000) && !Adc1::watchdog(0x1000, 0, true, false, false));
    bench.verdict("an interleave delay outside 5..20 cycles is refused",
                  !AdcCommon::interleave_delay(static_cast<uint8_t>(4)) &&
                      !AdcCommon::interleave_delay(static_cast<uint8_t>(21)));
    bench.verdict("a multi-ADC mode this part has not got the converters for is refused",
                  AdcCommon::multi(AdcMulti::triple_regular) == (adc_instances() >= 3u) &&
                      !AdcCommon::multi(static_cast<AdcMulti>(0x03)));
    (void)AdcCommon::multi(AdcMulti::independent);

#if defined(DAC_BASE)
    {
        Dac::init();
        print(serial, "  DAC: ", Dac::channels, " channel(s)",
              Dac::channels_known ? " (the manual's count)" : " (the manual was not read)",
              ", full scale ", Dac::steps, ", pads PA", dac_pad_pin(0), " and PA", dac_pad_pin(1),
              crlf);
        bench.verdict("the DAC has the two channels the manual gives this part",
                      Dac::channels == 2u && Dac::channels_known);
        DacChannelConfig w{};
        w.wave = DacWave::triangle;
        bench.verdict("a wave generator without a trigger is refused", !Dac::configure(0, w));
        DacChannelConfig d{};
        d.triggered = true;
        d.trigger = DacTrigger::software;
        d.dma = true;
        bench.verdict("a DMA request off the SOFTWARE trigger is refused (14.3.7)",
                      !Dac::configure(0, d));
        bench.verdict("a channel this part has not got is refused",
                      !Dac::write(2, 0) && !Dac::enable(2, true));
    }
#else
    needs_dac("the DAC's half of this letter");
#endif
    analog_down();
}

// =============================================================================
// b - the DAC as an actuator
// =============================================================================
void tb_dac() {
#if !defined(DAC_BASE)
    needs_dac("letter b");
#else
    {
        Dac::init();
        Dac::claim_pad<Pa4>();
        DacChannelConfig cfg{};
        bench.verdict("an untriggered buffered channel configures", Dac::configure(dac_ch_pa4, cfg));
        bench.verdict("and configuring leaves it DISABLED (14.3.6 and 14.3.9 want the enable "
                      "down while TSEL and MAMP are written)",
                      !Dac::enabled(dac_ch_pa4));
        (void)Dac::enable(dac_ch_pa4, true);
        (void)delay_us(clock, 20);

        // The holding register reaches DOR by itself, one APB1 cycle
        // later - measured in HCLK cycles, which is what this core counts.
        Cycles m{};
        (void)Dac::write(dac_ch_pa4, 0);
        (void)delay_us(clock, 5);
        m.start();
        (void)Dac::write(dac_ch_pa4, 0x0ABC);
        uint32_t polls = 0;
        while (Dac::output(dac_ch_pa4) != 0x0ABCu && polls < 1000u) {
            ++polls;
        }
        m.mark();
        print(serial, "  DHR -> DOR with no trigger: ", m.total, " HCLK cycles, ", polls,
              " read(s) of DOR (one APB1 cycle is ", SysClock::hz / SysClock::pclk1_hz,
              " of them)", crlf);
        bench.verdict("an untriggered write reaches the output register by itself",
                      Dac::output(dac_ch_pa4) == 0x0ABCu);
        bench.verdict("and it is there by the FIRST read of DOR - one APB1 cycle is shorter "
                      "than the load-and-compare that looks for it",
                      polls == 0u);

        // The three data formats all reach the same internal holding
        // register, shifted into place (14.3.3).
        (void)Dac::write8(dac_ch_pa4, 0x80);
        bench.verdict("the 8-bit format is a PLACEMENT: 0x80 lands at DHR[11:4] = 0x800",
                      Dac::code(dac_ch_pa4) == 0x800u);
        (void)Dac::write_left(dac_ch_pa4, 0xABC0);
        bench.verdict("the left-aligned format puts 0xABC0 at DHR[11:0] = 0xABC",
                      Dac::code(dac_ch_pa4) == 0xABCu);
        (void)Dac::write(dac_ch_pa4, 0x123);
        bench.verdict("and the right-aligned one is the datum itself",
                      Dac::code(dac_ch_pa4) == 0x123u);

        // With a trigger the holding register waits.
        cfg.triggered = true;
        cfg.trigger = DacTrigger::software;
        (void)Dac::enable(dac_ch_pa4, false);
        bench.verdict("a software-triggered channel configures", Dac::configure(dac_ch_pa4, cfg));
        (void)Dac::write(dac_ch_pa4, 0x555);
        (void)Dac::enable(dac_ch_pa4, true);
        (void)delay_us(clock, 20);
        const uint16_t before = Dac::output(dac_ch_pa4);
        (void)Dac::write(dac_ch_pa4, 0x777);
        (void)delay_us(clock, 5);
        const uint16_t held = Dac::output(dac_ch_pa4);
        (void)Dac::trigger(dac_ch_pa4);
        (void)delay_us(clock, 5);
        const uint16_t after = Dac::output(dac_ch_pa4);
        print(serial, "  triggered: DOR was ", before, ", stayed ", held,
              " over a write of 0x777, became ", after, " on SWTRIG", crlf);
        bench.verdict("with TEN set the holding register WAITS for the trigger",
                      held == before && after == 0x777u);
        bench.verdict("SWTRIGx clears itself once the datum has been taken (14.5.2)",
                      (Dac::regs().SWTRIGR & DAC_SWTRIGR_SWTRIG1) == 0u);

        // Both channels in one store, and both triggered at once.
        if (Dac::channels > 1u) {
            Dac::claim_pad<Pa5>();
            DacChannelConfig c2 = cfg;
            (void)Dac::enable(dac_ch_pa5, false);
            (void)Dac::configure(dac_ch_pa5, c2);
            (void)Dac::enable(dac_ch_pa5, true);
            (void)delay_us(clock, 20);
            (void)Dac::write_dual(0x111, 0x222);
            Dac::trigger_both();
            (void)delay_us(clock, 5);
            print(serial, "  dual store then one trigger: DOR1 ", Dac::output(dac_ch_pa4),
                  ", DOR2 ", Dac::output(dac_ch_pa5), crlf);
            bench.verdict("one 32-bit store carries both channels (14.4)",
                          Dac::output(dac_ch_pa4) == 0x111u && Dac::output(dac_ch_pa5) == 0x222u);
        }
        bench.verdict("no underrun happened without a DMA in sight",
                      !Dac::underrun(0) && !Dac::underrun(1));
        analog_down();
    }
#endif
}

// =============================================================================
// c - the zero-length wire: the transfer curve
// =============================================================================
void tc_transfer() {
#if !defined(DAC_BASE)
    needs_dac("letter c");
#else
    {
        if (!adc_up(base_config()) || !dac_up(free_ch, 0)) {
            bench.verdict("the converters came up", false);
            return;
        }
        Adc1::select(FreeIn{});

        uint16_t last = 0;
        bool monotonic = true;
        uint16_t worst = 0;
        uint16_t worst_code = 0;
        const uint16_t at_zero = loop_read(free_ch, 0, 4);
        for (uint16_t code = 0; code <= 4095u; code = static_cast<uint16_t>(code + 128u)) {
            const uint16_t got = loop_read(free_ch, code);
            if (code != 0u && got + 8u < last) {
                monotonic = false;
            }
            last = got;
            // The middle of the range is where a buffered output is
            // linear; the ends are its own swing limit and are judged
            // separately.
            if (code >= 256u && code <= 3840u) {
                const uint16_t err = abs_diff(got, code);
                if (err > worst) {
                    worst = err;
                    worst_code = code;
                }
            }
            if ((code & 0x1FFu) == 0u) {
                print(serial, "    code ", code, " -> ", got, crlf);
            }
        }
        const uint16_t at_full = loop_read(free_ch, 4095, 4);
        print(serial, "  buffered swing: code 0 reads ", at_zero, ", code 4095 reads ", at_full,
              "; worst error over codes 256..3840 is ", worst, " LSB at code ", worst_code, crlf);

        bench.verdict("the curve is MONOTONIC over the whole range", monotonic);
        // The bound is 1.5 % of full scale and the number printed above is
        // the measurement: what it contains is the DAC's own error, the
        // ADC's, and the two converters' shared reference - a round trip
        // through both, and neither of them alone.
        bench.verdict("and the round trip through both converters stays within 1.5 % of full "
                      "scale away from the rails", worst < 62u);
        bench.verdict("the BUFFER cannot reach either rail - the ends of the curve are its own "
                      "output swing and not the converter's",
                      at_zero > 20u && at_full < 4075u);
        analog_down();
    }
#endif
}

// =============================================================================
// d - buffered against unbuffered, free pad and loaded pad
// =============================================================================
void td_buffer() {
#if !defined(DAC_BASE)
    needs_dac("letter d");
#else
    {
        if (!adc_up(base_config())) {
            bench.verdict("the converter came up", false);
            return;
        }
        static const uint16_t codes[3] = {0, 2048, 4095};
        uint16_t free_buf[3] = {0, 0, 0};
        uint16_t free_raw[3] = {0, 0, 0};
        uint16_t load_buf[3] = {0, 0, 0};
        uint16_t load_raw[3] = {0, 0, 0};

        for (uint8_t pass = 0; pass < 2u; ++pass) {
            const bool buffered = pass == 0u;
            (void)dac_up(free_ch, 0, buffered);
            Adc1::select(FreeIn{});
            for (uint8_t i = 0; i < 3u; ++i) {
                const uint16_t v = loop_read(free_ch, codes[i], 4);
                if (buffered) {
                    free_buf[i] = v;
                } else {
                    free_raw[i] = v;
                }
            }
            if (Dac::channels > 1u) {
                (void)dac_up(loaded_ch, 0, buffered);
                Adc1::select(LoadedIn{});
                for (uint8_t i = 0; i < 3u; ++i) {
                    const uint16_t v = loop_read(loaded_ch, codes[i], 4);
                    if (buffered) {
                        load_buf[i] = v;
                    } else {
                        load_raw[i] = v;
                    }
                }
            }
            Dac::release();
        }

        print(serial, "  free pad ", free_pad_name, ": buffered ", free_buf[0], "/", free_buf[1], "/",
              free_buf[2], ", unbuffered ", free_raw[0], "/", free_raw[1], "/", free_raw[2],
              " (codes 0/2048/4095)", crlf);
        print(serial, "  loaded pad ", loaded_pad_name, ": buffered ", load_buf[0], "/", load_buf[1], "/",
              load_buf[2], ", unbuffered ", load_raw[0], "/", load_raw[1], "/", load_raw[2], crlf);

        bench.verdict("UNBUFFERED REACHES THE RAILS and buffered does not: on a pad with "
                      "nothing on it the unbuffered output is nearer both ends",
                      free_raw[0] < free_buf[0] && free_raw[2] > free_buf[2]);
        bench.verdict("and it is still linear in the middle of the range with only the ADC's "
                      "own sampling network on it",
                      abs_diff(free_raw[1], 2048u) < 60u);
        if (Dac::channels > 1u) {
            // The load's direction is the board's (the LED sinks at full
            // scale, a pull-up lifts at zero), so the judgement is by how
            // much farther the UNBUFFERED output strays from the free pad's
            // than the buffered one does, at whichever rail the load fights.
            uint16_t raw_stray = 0, buf_stray = 0;
            for (uint8_t r = 0; r < 3u; r += 2u) {
                raw_stray = max_of(raw_stray, abs_diff(load_raw[r], free_raw[r]));
                buf_stray = max_of(buf_stray, abs_diff(load_buf[r], free_buf[r]));
            }
            print(serial, "  the load moves the unbuffered output ", raw_stray,
                  " LSB from the free pad's at the rail it fights, the buffered one ", buf_stray, crlf);
            bench.verdict("A LOAD IS WHAT THE BUFFER IS FOR: the load pulls the UNBUFFERED "
                          "output far from the code it was given at one rail, while the "
                          "buffered one holds",
                          raw_stray > buf_stray + 200u);
        }
        analog_down();
    }
#endif
}

// =============================================================================
// e - the settling time after a step
// =============================================================================
void te_settling() {
#if !defined(DAC_BASE)
    needs_dac("letter e");
#else
    {
        AdcConfig c = base_config();
        // A SHORT sampling time is what makes this a probe rather than an
        // average: 15 ADCCLK cycles is 0.67 us at 22.5 MHz, and the DAC's
        // buffer is low enough an impedance for it.
        if (!adc_up(c, AdcSampleTime::cycles15) || !dac_up(dac_ch_pa4, 0)) {
            bench.verdict("the converters came up", false);
            return;
        }
        Adc1::select(In4{});
        static const uint16_t delays_us[6] = {0, 1, 2, 5, 10, 20};
        uint16_t got[6] = {0, 0, 0, 0, 0, 0};
        for (uint8_t i = 0; i < 6u; ++i) {
            (void)Dac::write(dac_ch_pa4, 0);
            (void)delay_us(clock, 200);
            (void)Adc1::read();
            Adc1::clear_flags(AdcFlag::all);
            (void)Dac::write(dac_ch_pa4, 4000);
            if (delays_us[i] != 0u) {
                (void)delay_us(clock, delays_us[i]);
            }
            got[i] = Adc1::read();
        }
        const uint16_t settled = loop_read(dac_ch_pa4, 4000, 4);
        print(serial, "  after a 0 -> 4000 step, one 15-cycle conversion started at");
        for (uint8_t i = 0; i < 6u; ++i) {
            print(serial, " +", delays_us[i], "us:", got[i]);
        }
        print(serial, "; settled ", settled, crlf);

        uint16_t first_within = 0xFFFFu;
        for (uint8_t i = 0; i < 6u; ++i) {
            if (abs_diff(got[i], settled) <= 40u) {
                first_within = delays_us[i];
                break;
            }
        }
        print(serial, "  within 1 % of the settled value from +", first_within, " us", crlf);
        bench.verdict("the step is NOT there at once - the reading taken with no delay is "
                      "below the settled one", got[0] + 40u < settled);
        bench.verdict("and it is within 1 % well inside the 20 us this letter waits",
                      first_within <= 20u);
        analog_down();
    }
#endif
}

// =============================================================================
// f - the conversion time in ADCCLK cycles, measured differentially
// =============================================================================
void tf_timing() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
    Adc1::select(AdcInput::vrefint);
    (void)delay_us(clock, 20);
    console_drain();

    const uint32_t q8 = core_cycles_per_adc_cycle_q8();
    print(serial, "  fADC ", AdcCommon::adc_hz(pclk2) / 1000u, " kHz, HCLK ", SysClock::hz / 1000u,
          " kHz: one ADCCLK cycle is ", q8 / 256u, " and ", (q8 % 256u) * 100u / 256u,
          "/100 HCLK cycles", crlf);

    static const AdcRes res[4] = {AdcRes::bits12, AdcRes::bits10, AdcRes::bits8, AdcRes::bits6};
    static const char* res_name[4] = {"12", "10", " 8", " 6"};
    static const AdcSampleTime smp[3] = {AdcSampleTime::cycles3, AdcSampleTime::cycles84,
                                         AdcSampleTime::cycles480};
    const uint16_t rounds = 128;
    bool all_close = true;
    uint32_t overhead_cycles = 0;

    for (uint8_t r = 0; r < 4u; ++r) {
        AdcConfig c = base_config();
        c.resolution = res[r];
        (void)Adc1::configure(c);
        uint32_t per[3] = {0, 0, 0};
        for (uint8_t s = 0; s < 3u; ++s) {
            Adc1::sample_time_all(smp[s]);
            (void)Adc1::read();   // one throwaway: the first after a change
            Cycles m{};
            m.start();
            for (uint16_t k = 0; k < rounds; ++k) {
                (void)Adc1::read();
                m.mark();
            }
            per[s] = m.total / rounds;
        }
        // THE DIFFERENCE is the measurement: the same loop at two
        // sampling times differs by exactly the sampling times' own
        // difference, and every cycle of polling overhead cancels.
        const uint32_t measured = per[2] - per[0];
        const uint32_t predicted =
            ((adc_sample_cycles(smp[2]) - adc_sample_cycles(smp[0])) * q8) / 256u;
        const uint32_t err = measured > predicted ? measured - predicted : predicted - measured;
        print(serial, "    ", res_name[r], " bits: ", per[0], " / ", per[1], " / ", per[2],
              " HCLK per conversion at 3 / 84 / 480 cycles; 477 ADCCLK cycles measured ",
              measured, " HCLK, predicted ", predicted, crlf);
        if (err * 100u > predicted) {
            all_close = false;
        }
        if (r == 0u) {
            // What is left when the chapter's own cycles are taken out of
            // the shortest conversion is the polling loop.
            const uint32_t conv =
                (adc_conversion_cycles(res[0], smp[0]) * q8) / 256u;
            overhead_cycles = per[0] > conv ? per[0] - conv : 0u;
        }
    }
    print(serial, "  the polling loop around one conversion costs ", overhead_cycles,
          " HCLK cycles", crlf);
    bench.verdict("the sampling time's contribution is the chapter's own, to under 1 %, at "
                  "every resolution", all_close);

    // And the SAR's own contribution, the other axis of the same table.
    Adc1::sample_time_all(AdcSampleTime::cycles3);
    uint32_t at12 = 0, at6 = 0;
    for (uint8_t pass = 0; pass < 2u; ++pass) {
        AdcConfig c = base_config();
        c.resolution = pass == 0u ? AdcRes::bits12 : AdcRes::bits6;
        (void)Adc1::configure(c);
        Adc1::sample_time_all(AdcSampleTime::cycles3);
        (void)Adc1::read();
        Cycles m{};
        m.start();
        for (uint16_t k = 0; k < rounds; ++k) {
            (void)Adc1::read();
            m.mark();
        }
        (pass == 0u ? at12 : at6) = m.total / rounds;
    }
    const uint32_t sar_measured = at12 - at6;
    const uint32_t sar_predicted =
        ((adc_sar_cycles(AdcRes::bits12) - adc_sar_cycles(AdcRes::bits6)) * q8) / 256u;
    print(serial, "  12 bits costs ", sar_measured, " HCLK more than 6 at the same sampling "
          "time; six SAR cycles predict ", sar_predicted, crlf);
    // The tolerance is 15 % plus ONE ADCCLK cycle: the polling loop's own
    // granularity is a few HCLK, which is a whole ADCCLK cycle where the
    // prescaler is four.
    bench.verdict("and the RESOLUTION's contribution is one ADCCLK cycle per bit (13.7)",
                  abs_diff(static_cast<uint16_t>(sar_measured),
                           static_cast<uint16_t>(sar_predicted)) * 100u <=
                      sar_predicted * 15u + (q8 * 100u) / 256u);
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// g - the scale: VDDA, the junction temperature, the battery
// =============================================================================
void tg_scale() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
    (void)delay_us(clock, 20);   // tSTART, the sensor's and the reference's

    Adc1::select(AdcInput::vrefint);
    const uint16_t vrefint = Adc1::read_settled(4);
    const uint16_t vdda = Adc1::vdda_mv(vrefint);
    print(serial, "  VREFINT reads ", vrefint, " against a calibration of ",
          AdcFactory::vrefint_cal(), " -> VDDA ", vdda, " mV", crlf);
    bench.verdict("VREFINT measures the supply, and it is a 3 V-class board (2.7..3.6 V)",
                  vdda >= 2700u && vdda <= 3600u);
    bench.verdict("the raw count is near the calibration's own, as it must be on a board "
                  "whose rail is near the 3.3 V ST used",
                  abs_diff(vrefint, AdcFactory::vrefint_cal()) < 200u);

    Adc1::select(AdcInput::temperature);
    const uint16_t ts = Adc1::read_settled(4);
    const int32_t tc = Adc1::temperature_centi_c(ts, vdda);
    const uint16_t ts_mv = adc_mv(ts, Adc1::result_steps(), vdda);
    const int32_t tc_typ = Adc1::temperature_centi_c_typical(ts_mv);
    print(serial, "  sensor reads ", ts, " (", ts_mv, " mV): ", tc / 100, ".",
          (tc % 100) / 10, " C from the two calibration points, ", tc_typ / 100, ".",
          (tc_typ % 100) / 10, " C from the datasheet's typical slope", crlf);
    bench.verdict("the junction temperature is a room-temperature one", tc > 0 && tc < 6000);
    bench.verdict("and the two formulas disagree by less than the sensor's own part-to-part "
                  "offset, which the manual puts at up to 45 C",
                  (tc > tc_typ ? tc - tc_typ : tc_typ - tc) < 4500);

    // The battery, and the switch that wins the channel they share.
    AdcCommon::vbat(true);
    Adc1::select(AdcInput::vbat);
    const uint16_t vbat_raw = Adc1::read_settled(4);
    const uint16_t vbat_mv = static_cast<uint16_t>(
        adc_mv(vbat_raw, Adc1::result_steps(), vdda) * AdcCommon::vbat_divider());
    print(serial, "  VBAT/", AdcCommon::vbat_divider(), " reads ", vbat_raw, " -> VBAT ",
          vbat_mv, " mV", crlf);
    bench.verdict("the battery pin measures the supply it is tied to on this board, through "
                  "the bridge the chapter names",
                  abs_diff(vbat_mv, vdda) < 200u);

    if (AdcCommon::sensor_shares_vbat()) {
        // BOTH switches on: 13.11 says the battery wins, and the reading
        // is what says so - the sensor's count and the battery's are far
        // apart.
        const uint16_t both = Adc1::read_settled(4);
        AdcCommon::vbat(false);
        Adc1::select(AdcInput::temperature);
        const uint16_t sensor_only = Adc1::read_settled(4);
        print(serial, "  with TSVREFE and VBATE both set the shared channel reads ", both,
              " (battery ", vbat_raw, ", sensor ", sensor_only, " - only ",
              abs_diff(vbat_raw, sensor_only), " LSB apart on a 3.3 V board, which is what "
              "makes this a close-run reading and not an obvious one)", crlf);
        // What the shared channel reads with both switches set is a fact of
        // the part: the battery alone on one, neither source alone on
        // another (measured) - reported, and judged only in that the reading
        // is not the sensor's alone.
        print(serial, "  13.11's precedence: the reading is ",
              abs_diff(both, vbat_raw) <= 20u ? "the BATTERY's"
              : abs_diff(both, sensor_only) <= 20u ? "the SENSOR's" : "NEITHER source alone", crlf);
        bench.verdict("with both switches set the shared channel is not the sensor alone",
                      abs_diff(both, sensor_only) > 15u || abs_diff(both, vbat_raw) <= 20u);
    }
    AdcCommon::vbat(false);
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// h - the analog watchdog
// =============================================================================
void th_watchdog() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    // The stimulus is the DAC where there is one and VREFINT where there
    // is not: a fixed source still proves the window, only not the
    // crossing.
    uint8_t channel = adc_input_channel(AdcInput::vrefint);
#if defined(DAC_BASE)
    if (dac_up(dac_ch_pa4, 2048)) {
        channel = In4::channel;
    }
#else
    AdcCommon::internal_sources(true);
    (void)delay_us(clock, 20);
#endif
    (void)Adc1::select_channel(channel);
    const uint16_t middle = Adc1::read_settled(4);
    print(serial, "  guarding channel ", channel, ", which reads ", middle, crlf);

    const uint16_t low = static_cast<uint16_t>(middle > 600u ? middle - 600u : 0u);
    const uint16_t high = static_cast<uint16_t>(middle + 600u > 4095u ? 4095u : middle + 600u);
    bench.verdict("a single-channel window arms", Adc1::watchdog(low, high, true, false, true, channel));
    bench.verdict("and the register holds what it was given",
                  Adc1::watchdog_low() == low && Adc1::watchdog_high() == high &&
                      Adc1::watchdog_channel() == channel);
    Adc1::clear_flags(AdcFlag::all);
    (void)Adc1::read();
    const bool inside_quiet = !Adc1::flag(AdcFlag::watchdog);
    bench.verdict("a conversion INSIDE the window raises nothing", inside_quiet);

#if defined(DAC_BASE)
    {
        (void)loop_read(dac_ch_pa4, 200);
        Adc1::clear_flags(AdcFlag::all);
        (void)Adc1::read();
        const bool below = Adc1::flag(AdcFlag::watchdog);
        (void)loop_read(dac_ch_pa4, 3900);
        Adc1::clear_flags(AdcFlag::all);
        (void)Adc1::read();
        const bool above = Adc1::flag(AdcFlag::watchdog);
        print(serial, "  window [", low, ", ", high, "]: below it AWD ", below ? "up" : "down",
              ", above it AWD ", above ? "up" : "down", crlf);
        bench.verdict("a DAC step out of the window raises AWD on both sides", below && above);

        // 13.3.8: the comparison happens BEFORE alignment, so the same
        // window holds with ALIGN set - which is exactly the trap a
        // caller who reads only the data register would fall into.
        AdcConfig c = base_config();
        c.left_aligned = true;
        (void)Adc1::configure(c);
        (void)loop_read(dac_ch_pa4, 2048);
        Adc1::clear_flags(AdcFlag::all);
        const uint16_t left = Adc1::read();
        const bool quiet_left = !Adc1::flag(AdcFlag::watchdog);
        (void)loop_read(dac_ch_pa4, 200);
        Adc1::clear_flags(AdcFlag::all);
        (void)Adc1::read();
        const bool fired_left = Adc1::flag(AdcFlag::watchdog);
        print(serial, "  left-aligned, the same window: 2048 reads ", left, " and is quiet, 200 ",
              fired_left ? "fires" : "does not fire", crlf);
        bench.verdict("13.3.8 MEASURED: the thresholds are compared before the alignment, so "
                      "a left-aligned result four bits up still lands in the same window",
                      quiet_left && fired_left && left > 2048u);
        (void)Adc1::configure(base_config());

        // All channels, the other row of table 85.
        bench.verdict("an all-channel window arms", Adc1::watchdog(low, high, true, false, false));
        (void)loop_read(dac_ch_pa4, 2048);
        Adc1::clear_flags(AdcFlag::all);
        (void)Adc1::read();
        bench.verdict("and the guarded value is still inside it", !Adc1::flag(AdcFlag::watchdog));
    }
#else
    print(serial, "  no DAC: the crossing cannot be staged from here", crlf);
#endif
    Adc1::watchdog_off();
    bench.verdict("the watchdog disarms and the flag stays down through a conversion", [] {
        Adc1::clear_flags(AdcFlag::all);
        (void)Adc1::watchdog_thresholds(0, 0);
        (void)Adc1::read();
        return !Adc1::flag(AdcFlag::watchdog);
    }());
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// i - the injected group
// =============================================================================
void ti_injected() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    uint8_t regular_ch = adc_input_channel(AdcInput::vrefint);
    uint8_t injected_ch = adc_input_channel(AdcInput::vrefint);
#if defined(DAC_BASE)
    Dac::init();
    (void)dac_channel_up(dac_ch_pa4, 1000);
    (void)dac_channel_up(dac_ch_pa5, 3000);
    regular_ch = In4::channel;
    injected_ch = In5::channel;
#else
    (void)regular_ch;
    AdcCommon::internal_sources(true);
    (void)delay_us(clock, 20);
#endif

    // THE TAIL RULE. One channel means JSQ4 and nothing else (13.13.12).
    static uint8_t one[1] = {0};
    one[0] = injected_ch;
    bench.verdict("a one-channel injected sequence writes", Adc1::injected_sequence(one, 1));
    print(serial, "  JSQR slots 1..4 hold ", Adc1::injected_slot_channel(1), " ",
          Adc1::injected_slot_channel(2), " ", Adc1::injected_slot_channel(3), " ",
          Adc1::injected_slot_channel(4), " for a list of one channel ", injected_ch, crlf);
    bench.verdict("13.13.12's TAIL RULE is hidden by the verb: a list of one lands in JSQ4, "
                  "which is the slot a JL of 0 converts",
                  Adc1::injected_slot_channel(4) == injected_ch && Adc1::injected_length() == 1u);

    int16_t inj = 0;
    bench.verdict("an injected conversion completes", Adc1::read_injected(inj));
    print(serial, "  JDR1 reads ", inj, crlf);

    // The offset, and the sign it can produce.
    (void)Adc1::injected_offset(1, 500);
    int16_t with_offset = 0;
    (void)Adc1::read_injected(with_offset);
    print(serial, "  with JOFR1 = 500 the same conversion reads ", with_offset, crlf);
    bench.verdict("the injected offset is SUBTRACTED from the raw result",
                  abs_diff(static_cast<uint16_t>(inj - with_offset), 500u) < 40u);
    (void)Adc1::injected_offset(1, 4000);
    int16_t negative = 0;
    (void)Adc1::read_injected(negative);
    print(serial, "  with JOFR1 = 4000 it reads ", negative,
          " - the only signed datapath this converter has", crlf);
    bench.verdict("and it can push the result below zero, sign-extended", negative < 0);
    (void)Adc1::injected_offset(1, 0);

#if defined(DAC_BASE)
    {
        // THE PREEMPTION. A continuous regular run on one pad, an
        // injected conversion on the other: the injected result is the
        // OTHER pad's, and the regular run carries on.
        // EOCS = 0, ON PURPOSE: with EOC at the end of the sequence and
        // no DMA there is no overrun detection at all (13.8.3), so a
        // continuous run nobody reads keeps running instead of stopping
        // at the first unread result - which is what lets this letter
        // leave it alone while the injected group takes the converter.
        // Letter l is where the other choice is measured.
        AdcConfig c = base_config();
        c.continuous = true;
        c.eoc_per_conversion = false;
        (void)Adc1::configure(c);
        (void)Adc1::select_channel(regular_ch);
        Adc1::clear_flags(AdcFlag::all);
        Adc1::start();
        (void)delay_us(clock, 200);
        const uint16_t regular_before = Adc1::result();
        int16_t injected = 0;
        const bool got = Adc1::read_injected(injected);
        (void)delay_us(clock, 200);
        const bool still_running = Adc1::ready();
        const uint16_t regular_after = Adc1::result();
        Adc1::stop();
        print(serial, "  continuous regular on channel ", regular_ch, " reads ", regular_before,
              "; an injected conversion of channel ", injected_ch, " reads ", injected,
              "; the regular run then reads ", regular_after, crlf);
        bench.verdict("an injected trigger INTERRUPTS the regular run and converts its own "
                      "channel - two pads, two values, one converter",
                      got && abs_diff(static_cast<uint16_t>(injected), regular_before) > 800u);
        bench.verdict("and the regular sequence RESUMES afterwards, still on its own channel",
                      still_running && abs_diff(regular_after, regular_before) < 200u);

        // AUTO-INJECTION: no trigger at all, the injected group follows
        // the regular one by itself.
        c = base_config();
        c.auto_injected = true;
        bench.verdict("an auto-injected config takes", Adc1::configure(c));
        (void)Adc1::select_channel(regular_ch);
        Adc1::clear_flags(AdcFlag::all);
        (void)Adc1::read();
        uint32_t spins = 0;
        while (!Adc1::injected_ready() && spins < 100000u) {
            ++spins;
        }
        const int16_t auto_value = Adc1::injected_result(1);
        print(serial, "  JAUTO: one regular conversion, and JEOC arrived after ", spins,
              " polls with JDR1 = ", auto_value, crlf);
        bench.verdict("with JAUTO the injected group runs after the regular one with NO "
                      "trigger of its own (13.3.10)",
                      Adc1::injected_ready() &&
                          abs_diff(static_cast<uint16_t>(auto_value), regular_before) > 800u);
        (void)Adc1::configure(base_config());
    }
#endif
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// j - dual regular simultaneous mode
// =============================================================================
void tj_dual() {
#if !defined(ADC2_BASE)
    print(serial, "  this part has ADC1 alone: the multi-ADC modes need two converters and "
                  "this letter claims nothing", crlf);
#else
#if !defined(DAC_BASE)
    needs_dac("letter j's stimulus");
#else
    {
        using Adc2 = Adc<2>;
        if (!adc_up(base_config())) {
            bench.verdict("the converters came up", false);
            return;
        }
        Adc2::bus_clock(true);
        if (!Adc2::configure(base_config()) || !Adc2::power_on(clock)) {
            bench.verdict("ADC2 came up", false);
            return;
        }
        Adc2::sample_time_all(AdcSampleTime::cycles480);
        Dac::init();
        (void)dac_channel_up(dac_ch_pa4, 1200);
        (void)dac_channel_up(dac_ch_pa5, 3000);

        // First the single-converter answers, as the reference.
        Adc1::select(In4{});
        const uint16_t single4 = Adc1::read_settled(4);
        Adc1::select(In5{});
        const uint16_t single5 = Adc1::read_settled(4);

        // TWO PADS, TWO CONVERTERS, ONE TRIGGER - the arrangement 13.9.2
        // is written for.
        Adc1::select(In4{});
        (void)Adc2::select_channel(In5::channel);
        bench.verdict("regular simultaneous mode selects", AdcCommon::multi(AdcMulti::dual_regular));
        Adc1::clear_flags(AdcFlag::all);
        Adc2::clear_flags(AdcFlag::all);
        Adc1::start();
        uint32_t spins = 0;
        while (!Adc1::ready() && spins < 100000u) {
            ++spins;
        }
        const uint32_t pair = AdcCommon::data();
        const uint32_t csr = AdcCommon::status();
        const uint16_t master = Adc1::result();
        const uint16_t slave = Adc2::result();
        print(serial, "  two pads, one trigger: ADC1 ", master, " (single ", single4,
              "), ADC2 ", slave, " (single ", single5, ") after ", spins, " polls; CSR ",
              hex(csr), ", CDR ", hex(pair), crlf);
        bench.verdict("ONE trigger on the MASTER converts BOTH: neither converter was started "
                      "by itself and both have a result",
                      abs_diff(master, single4) < 60u && abs_diff(slave, single5) < 60u);
        bench.verdict("and the common status register shows both ends of the pair, which is "
                      "the only place a reader of one converter can see the other",
                      (csr & AdcFlag::converted) != 0u &&
                          ((csr >> 8) & AdcFlag::converted) != 0u);
        // AND THE COMMON DATA REGISTER IS NOT LOADED YET. 13.13.17 says
        // ADC_CDR holds the pair in dual mode and does not say what turns
        // it on; measured, with DMA[1:0] at 00 it stays at zero however
        // many pairs have converted, and selecting a multi-ADC DMA mode -
        // with no stream armed and nothing to serve the requests - is
        // what starts filling it.
        AdcCommon::multi_dma(AdcMultiDma::mode2, false);
        Adc1::clear_flags(AdcFlag::all);
        Adc2::clear_flags(AdcFlag::all);
        Adc1::start();
        spins = 0;
        while (!Adc1::ready() && spins < 100000u) {
            ++spins;
        }
        const uint32_t with_dma = AdcCommon::data();
        const uint32_t slave_sr = Adc2::flags();
        AdcCommon::multi_dma(AdcMultiDma::off, false);
        print(serial, "  with DMA[1:0] = 10 selected and no stream armed, CDR ", hex(with_dma),
              " -> ADC1 ", static_cast<uint16_t>(with_dma & 0xFFFFu), ", ADC2 ",
              static_cast<uint16_t>(with_dma >> 16), "; the slave's own SR reads ",
              hex(slave_sr), crlf);
        bench.verdict("and the SLAVE's own EOC does NOT stand after such a round - the pair "
                      "went to the common register and the datum the slave's DR holds is not "
                      "flagged, which is why a reconfiguration has to clear its STRT by hand",
                      (slave_sr & AdcFlag::started) != 0u &&
                          (slave_sr & AdcFlag::converted) == 0u);
        bench.verdict("THE COMMON DATA REGISTER IS THE DMA's: at DMA[1:0] = 00 it stays zero "
                      "however many pairs convert, and a multi-ADC DMA mode fills it with the "
                      "pair - the master in the low half - with no stream in sight",
                      pair == 0u && abs_diff(static_cast<uint16_t>(with_dma & 0xFFFFu), master) < 60u &&
                          abs_diff(static_cast<uint16_t>(with_dma >> 16), slave) < 60u);

        // ONE PAD, TWO CONVERTERS - which 13.9.2's own note tells an
        // application not to do ("no overlapping sampling times for the
        // two ADCs when converting the same channel"). What it costs is
        // the point of measuring it.
        (void)AdcCommon::multi(AdcMulti::independent);
        Adc1::clear_flags(AdcFlag::all);
        Adc2::clear_flags(AdcFlag::all);
        const bool moved = Adc2::select_channel(In4::channel);
        print(serial, "  ADC2 moved to channel ", Adc2::sequence_channel(1), " (the write was ",
              moved ? "taken" : "REFUSED", ")", crlf);
        (void)AdcCommon::multi(AdcMulti::dual_regular);
        Adc1::clear_flags(AdcFlag::all);
        Adc2::clear_flags(AdcFlag::all);
        Adc1::start();
        spins = 0;
        while (!Adc1::ready() && spins < 100000u) {
            ++spins;
        }
        const uint16_t same_m = Adc1::result();
        const uint16_t same_s = Adc2::result();
        print(serial, "  ONE pad on both: ADC1 ", same_m, ", ADC2 ", same_s, ", difference ",
              abs_diff(same_m, same_s), " LSB (single-converter answer ", single4, ")", crlf);
        bench.verdict("the two converters sampling the SAME pad at the same time still agree "
                      "with each other and with the single-converter answer - the chapter's "
                      "caution costs nothing on a source this stiff",
                      abs_diff(same_m, same_s) < 60u && abs_diff(same_m, single4) < 80u);

        (void)AdcCommon::multi(AdcMulti::independent);
        bench.verdict("and coming back to independent mode is one write",
                      AdcCommon::multi() == AdcMulti::independent);
        analog_down();
    }
#endif
#endif
}

// =============================================================================
// k - the wave generators
// =============================================================================
void tk_waves() {
#if !defined(DAC_BASE)
    needs_dac("letter k");
#else
    {
        if (!adc_up(base_config())) {
            bench.verdict("the converter came up", false);
            return;
        }
        Dac::init();
        Dac::claim_pad<Pa4>();
        DacChannelConfig cfg{};
        cfg.triggered = true;
        cfg.trigger = DacTrigger::software;
        cfg.wave = DacWave::noise;
        cfg.amplitude = 11;   // the whole twelve bits unmasked
        bench.verdict("a noise channel configures", Dac::configure(dac_ch_pa4, cfg));
        (void)Dac::write(dac_ch_pa4, 0);
        (void)Dac::enable(dac_ch_pa4, true);
        (void)delay_us(clock, 20);

        // THE ONE DETERMINISTIC THING ABOUT AN LFSR: its preload. 14.3.8
        // gives 0xAAA, and figure 98 gives the value after one more step.
        (void)Dac::trigger(dac_ch_pa4);
        (void)delay_us(clock, 5);
        const uint16_t first = Dac::output(dac_ch_pa4);
        (void)Dac::trigger(dac_ch_pa4);
        (void)delay_us(clock, 5);
        const uint16_t second = Dac::output(dac_ch_pa4);
        print(serial, "  LFSR with DHR = 0: first trigger gives ", hex(first),
              ", second gives ", hex(second), " (14.3.8 preloads ", hex(dac_lfsr_preload),
              ", figure 98 steps to 0xD55)", crlf);
        bench.verdict("the LFSR's preload is on the output after ONE trigger",
                      first == dac_lfsr_preload);
        bench.verdict("and the second step is figure 98's own value", second == 0x0D55u);

        // Resetting WAVE reloads it (14.3.8's last sentence).
        (void)Dac::wave(dac_ch_pa4, DacWave::none);
        (void)Dac::wave(dac_ch_pa4, DacWave::noise);
        (void)Dac::trigger(dac_ch_pa4);
        (void)delay_us(clock, 5);
        bench.verdict("clearing WAVEx RESETS the generator - the next trigger gives the "
                      "preload again", Dac::output(dac_ch_pa4) == dac_lfsr_preload);

        // The noise as the ADC sees it on the pad.
        Adc1::select(In4{});
        uint16_t nmin = 0xFFFFu, nmax = 0;
        for (uint16_t k = 0; k < 400u; ++k) {
            (void)Dac::trigger(dac_ch_pa4);
            (void)delay_us(clock, 10);
            const uint16_t v = Adc1::read();
            if (v < nmin) nmin = v;
            if (v > nmax) nmax = v;
        }
        print(serial, "  400 noise steps read back through the pad: ", nmin, " .. ", nmax, crlf);
        bench.verdict("the noise reaches most of the range on the PAD, not just in DOR",
                      static_cast<uint16_t>(nmax - nmin) > 2000u);

        // The triangle, whose amplitude is the one thing MAMP promises.
        static const uint8_t mamps[2] = {7, 9};
        bool amplitudes_right = true;
        for (uint8_t i = 0; i < 2u; ++i) {
            DacChannelConfig t = cfg;
            t.wave = DacWave::triangle;
            t.amplitude = mamps[i];
            (void)Dac::enable(dac_ch_pa4, false);
            (void)Dac::configure(dac_ch_pa4, t);
            (void)Dac::write(dac_ch_pa4, 0);
            (void)Dac::enable(dac_ch_pa4, true);
            (void)delay_us(clock, 20);
            uint16_t lo = 0xFFFFu, hi = 0;
            const uint16_t steps = static_cast<uint16_t>(4u * (dac_wave_amplitude(mamps[i]) + 1u));
            for (uint16_t k = 0; k < steps; ++k) {
                (void)Dac::trigger(dac_ch_pa4);
                const uint16_t v = Dac::output(dac_ch_pa4);
                if (v < lo) lo = v;
                if (v > hi) hi = v;
            }
            const uint16_t expect = dac_wave_amplitude(mamps[i]);
            print(serial, "    MAMP ", mamps[i], ": DOR swept ", lo, " .. ", hi, ", amplitude ",
                  expect, " expected", crlf);
            if (lo != 0u || hi != expect) {
                amplitudes_right = false;
            }
        }
        bench.verdict("the triangle's peak is exactly 2^(MAMP+1) - 1 above the holding "
                      "register, and it comes back to it", amplitudes_right);
        analog_down();
    }
#endif
}

// =============================================================================
// l - overrun, and the two data-management modes
// =============================================================================
void tl_overrun() {
    AdcConfig c = base_config();
    c.continuous = true;
    c.eoc_per_conversion = true;   // 13.13.3: this is what enables detection
    if (!adc_up(c, AdcSampleTime::cycles3)) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
    Adc1::select(AdcInput::vrefint);
    (void)delay_us(clock, 20);

    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    (void)delay_us(clock, 200);   // many conversions, none of them read
    const bool over = Adc1::overrun();
    print(serial, "  EOCS = 1, continuous, nothing read for 200 us: OVR ", over ? "up" : "down",
          crlf);
    bench.verdict("with EOC at every conversion, an unread result raises OVR (13.8.2)", over);

    // The recovery the chapter gives: clear the flag and trigger again.
    Adc1::stop();
    Adc1::clear_flags(AdcFlag::all);
    uint16_t after = 0;
    const bool recovered = Adc1::read(after);
    print(serial, "  after clearing OVR and starting again: ", after, crlf);
    bench.verdict("clearing OVR and re-triggering is the whole recovery, and the flag stays "
                  "down", recovered && !Adc1::overrun());

    // And the mode where the chapter says there is no detection at all.
    c.eoc_per_conversion = false;
    c.dma = false;
    (void)Adc1::configure(c);
    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    (void)delay_us(clock, 200);
    const bool quiet = !Adc1::overrun();
    Adc1::stop();
    print(serial, "  EOCS = 0 and no DMA, the same 200 us: OVR ", quiet ? "down" : "up", crlf);
    bench.verdict("13.8.3 MEASURED: with EOC at the end of the sequence and no DMA there is "
                  "no overrun detection at all - converting without reading is a CHOICE the "
                  "chapter offers", quiet);

    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// m - the sequencer-during-conversion erratum, and the refusal that answers it
// =============================================================================
void tm_erratum() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
    Adc1::select(AdcInput::vrefint);
    (void)delay_us(clock, 20);
    const uint8_t other = adc_input_channel(AdcInput::vbat);

    // A 480-cycle sampling time makes one conversion about 22 us at
    // 22.5 MHz - long enough to rewrite the sequence inside it from a
    // 180 MHz core.
    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    // STRT is what the driver's own "in flight" answer rests on.
    uint32_t spins = 0;
    while (!Adc1::started() && spins < 1000u) {
        ++spins;
    }
    const bool saw_started = Adc1::started();
    const bool refused = !Adc1::select_channel(other);

    // AND THE ERRATUM ITSELF, staged with the unchecked verb - INSIDE the
    // same conversion, which is why nothing is printed until it is over:
    // a console line at 115200 is longer than the 22 us this conversion
    // lasts.
    static uint8_t one[1] = {0};
    one[0] = other;
    (void)Adc1::regular_sequence_unchecked(one, 1);
    uint32_t waited = 0;
    while (!Adc1::ready() && waited < 200000u) {
        ++waited;
    }
    const bool never_came = !Adc1::ready();

    print(serial, "  STRT rose after ", spins, " polls; the driver ",
          refused ? "REFUSED" : "accepted", " a sequence write inside the conversion", crlf);
    bench.verdict("STRT rises as soon as a conversion starts, which is what makes the "
                  "in-flight answer possible at all", saw_started);
    bench.verdict("and the driver REFUSES a sequence write there - the structural answer to "
                  "the errata item", refused);
    print(serial, "  the sequence rewritten INSIDE a software-started conversion: EOC ",
          never_came ? "never arrived" : "arrived anyway", " after ", waited, " polls", crlf);
    bench.verdict("THE ERRATUM MEASURED: with a software start, a sequence written during a "
                  "conversion resets it and nothing restarts it", never_came);

    // The workaround: start again.
    Adc1::start();
    uint16_t v = 0;
    uint32_t again = 0;
    while (!Adc1::ready() && again < 200000u) {
        ++again;
    }
    if (Adc1::ready()) {
        v = Adc1::result();
    }
    print(serial, "  one more SWSTART and the new sequence converts: ", v, crlf);
    bench.verdict("and the errata's own workaround - trigger again - brings it back",
                  again < 200000u);

    // THE HARDWARE TRIGGER IS SPARED, which is the other half of the
    // item's own text - and the only way to say it on this board is a pad
    // driven onto its own EXTI line (letter o is where that technique is
    // established).
    if constexpr (have_trigger_pads) {
#if defined(STM32F446xx)
        AdcConfig c = base_config();
        c.trigger = AdcTrigger::exti11;
        c.trigger_edge = AdcEdge::rising;
        (void)Adc1::configure(c);
        (void)Adc1::select_channel(adc_input_channel(AdcInput::vrefint));
        arm_pad_line<TrigPad11>(adc_regular_exti_line);
        Adc1::clear_flags(AdcFlag::all);

        // One edge starts a conversion; the sequence is rewritten inside
        // it, so that conversion is lost - and the NEXT edge starts the
        // new sequence with no software help at all.
        pad_edge<TrigPad11>(adc_regular_exti_line);
        (void)Adc1::regular_sequence_unchecked(one, 1);
        uint32_t lost = 0;
        while (!Adc1::ready() && lost < 20000u) {
            ++lost;
        }
        const bool first_lost = !Adc1::ready();
        pad_edge<TrigPad11>(adc_regular_exti_line);
        uint32_t hw = 0;
        while (!Adc1::ready() && hw < 20000u) {
            ++hw;
        }
        const bool recovered = Adc1::ready();
        const uint16_t hw_value = Adc1::result();
        TrigPad11::release();
        (void)Exti::release(adc_regular_exti_line);
        print(serial, "  with a HARDWARE trigger: the conversion the rewrite landed in was ",
              first_lost ? "lost" : "delivered anyway", " and the NEXT edge converted ",
              recovered ? "" : "nothing, ", hw_value, crlf);
        bench.verdict("and the erratum SPARES the hardware trigger, as its own text says: no "
                      "software touched the converter between the rewrite and the result",
                      first_lost && recovered && hw_value > 100u);
#endif
    } else {
        print(serial, "  no pad on line 11 this suite knows to be free on this board: the "
                      "hardware trigger's half of the item is not staged here", crlf);
    }

    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// n - the flash accelerator against the converter's spread
// =============================================================================
void tn_noise() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
    Adc1::select(AdcInput::vrefint);
    (void)delay_us(clock, 20);
    console_drain();

    uint16_t spread[2] = {0, 0};
    uint32_t mean[2] = {0, 0};
    for (uint8_t pass = 0; pass < 2u; ++pass) {
        FlashAccel::prefetch(pass == 0u);
        uint16_t lo = 0xFFFFu, hi = 0;
        uint32_t sum = 0;
        for (uint16_t k = 0; k < 256u; ++k) {
            const uint16_t v = Adc1::read();
            if (v < lo) lo = v;
            if (v > hi) hi = v;
            sum += v;
        }
        spread[pass] = static_cast<uint16_t>(hi - lo);
        mean[pass] = sum / 256u;
    }
    FlashAccel::prefetch(true);
    print(serial, "  256 conversions of VREFINT: prefetch ON spread ", spread[0], " LSB (mean ",
          mean[0], "), prefetch OFF spread ", spread[1], " LSB (mean ", mean[1], ")", crlf);
    bench.verdict("the converter is quiet either way at 12 bits - a handful of LSB, not a "
                  "handful of per cent", spread[0] < 40u && spread[1] < 40u);
    bench.verdict("and the two means agree, so the errata's workaround costs the ACCURACY "
                  "nothing and buys the NOISE what this letter reports",
                  abs_diff(static_cast<uint16_t>(mean[0]), static_cast<uint16_t>(mean[1])) < 20u);
    bench.verdict("the accelerator is back on where the clock task left it",
                  FlashAccel::prefetch() && FlashAccel::icache() && FlashAccel::dcache());
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// o - the external trigger path, through the EXTI's software trigger
// =============================================================================

void to_trigger() {
    AdcConfig c = base_config();
    c.trigger = AdcTrigger::exti11;
    c.trigger_edge = AdcEdge::rising;
    c.injected_trigger = AdcInjectedTrigger::exti15;
    c.injected_trigger_edge = AdcEdge::rising;
    if (!adc_up(c)) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
    Adc1::select(AdcInput::vrefint);
    static uint8_t one[1] = {0};
    one[0] = adc_input_channel(AdcInput::vrefint);
    (void)Adc1::injected_sequence(one, 1);
    (void)delay_us(clock, 20);

    // The lines are armed with a rising sense and as EVENTS first:
    // nothing is bound to a vector here and the NVIC lines stay disabled
    // throughout, so a pending bit is a witness and never a handler.
    (void)Exti::sense(adc_regular_exti_line, ExtiSense::rising);
    (void)Exti::event(adc_regular_exti_line, true);
    (void)Exti::sense(adc_injected_exti_line, ExtiSense::rising);
    (void)Exti::event(adc_injected_exti_line, true);

    Adc1::clear_flags(AdcFlag::all);
    (void)delay_us(clock, 200);
    const bool quiet = !Adc1::ready() && !Adc1::injected_ready();
    bench.verdict("with a hardware trigger selected and no edge, nothing converts", quiet);

    // WHICH MASK THE SOFTWARE TRIGGER OBEYS is the question this letter
    // answers, and the two chapters between them do not: 12.3.6 gives
    // SWIER's own condition in terms of the INTERRUPT mask alone, and the
    // ADC's table 87 says only "EXTI line 11".
    const bool with_event = trigger_converts(adc_regular_exti_line, false);
    (void)Exti::interrupt(adc_regular_exti_line, true);
    const bool with_interrupt = trigger_converts(adc_regular_exti_line, false);
    const bool line_fired = Exti::pending(adc_regular_exti_line);
    print(serial, "  line ", adc_regular_exti_line, " raised by SWIER: the line ",
          line_fired ? "FIRED" : "did not fire", ", the converter answered with EMR alone ",
          with_event ? "yes" : "no", " and with IMR open too ", with_interrupt ? "yes" : "no",
          crlf);
    bench.verdict("THE SOFTWARE TRIGGER DOES NOT REACH THE CONVERTER: SWIER raises the line's "
                  "own pending bit and no conversion follows it, under either mask - so the "
                  "ADC's trigger takes the EDGE DETECTOR's output and not the register's",
                  line_fired && !with_event && !with_interrupt);
    (void)Exti::clear(adc_regular_exti_line);
    (void)Exti::interrupt(adc_regular_exti_line, false);

    // SWSTART is not EXTEN's business. 13.13.3 gives the two bits
    // separately and this is what that means: a converter armed for a
    // hardware trigger still converts when the software asks.
    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    uint32_t spins = 0;
    while (!Adc1::ready() && spins < 20000u) {
        ++spins;
    }
    const uint16_t by_software = Adc1::result();
    bench.verdict("and SWSTART is INDEPENDENT of it: a converter armed for a hardware trigger "
                  "still converts on the software one", spins < 20000u && by_software > 100u);

    if constexpr (have_trigger_pads) {
#if defined(STM32F446xx)
        // A REAL EDGE now, from a pad this program drives itself - once
        // with the event mask alone and once with the interrupt mask too.
        const bool pad_unmasked =
            pad_edge_converts<TrigPad11>(adc_regular_exti_line, false, false, false);
        const bool pad_event_only =
            pad_edge_converts<TrigPad11>(adc_regular_exti_line, false, true, false);
        const bool pad_irq_only =
            pad_edge_converts<TrigPad11>(adc_regular_exti_line, false, false, true);
        const uint16_t pad_value = Adc1::result();
        print(serial, "  a rising edge driven onto the line's own pad converts: with NEITHER "
              "mask ", pad_unmasked ? "yes" : "no", ", with EMR alone ",
              pad_event_only ? "yes" : "no", ", with IMR alone ", pad_irq_only ? "yes" : "no",
              "; DR ", pad_value, crlf);
        bench.verdict("THE REGULAR TRIGGER PATH, MEASURED WITH NO WIRE: a pad driven high feeds "
                      "its own EXTI line, and line 11 starts a regular conversion",
                      pad_event_only && pad_value > 100u);
        bench.verdict("and NEITHER MASK GATES IT: the converter takes the EDGE DETECTOR's "
                      "output, which is why an unmasked line still triggers and why the "
                      "software trigger - injected past that detector - never does",
                      pad_unmasked && pad_irq_only);
        const bool inj_by_pad =
            pad_edge_converts<TrigPad15>(adc_injected_exti_line, true, true, false);
        print(serial, "  and on line 15: JEOC ", inj_by_pad ? "yes" : "no", ", JDR1 ",
              Adc1::injected_result(1), crlf);
        bench.verdict("and the injected group has a line of its own, line 15",
                      inj_by_pad && Adc1::injected_result(1) > 100);
#endif
    } else {
        print(serial, "  no pad on lines 11 and 15 this suite knows to be free on this board: "
                      "the edge that would start a conversion is not staged here", crlf);
    }

    (void)Exti::clear(adc_regular_exti_line);
    (void)Exti::clear(adc_injected_exti_line);
    (void)Exti::release(adc_regular_exti_line);
    (void)Exti::release(adc_injected_exti_line);
    bench.verdict("and both lines clear and release on demand",
                  !Exti::pending(adc_regular_exti_line) &&
                      !Exti::pending(adc_injected_exti_line));
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// p - AnalogSampler inside a real kernel, walking three inputs
// =============================================================================

struct Collector;
using Subs = Subscribers<Collector>;

// THE LIST IS THE SAME ON EVERY PART, and the third input is a PAD -
// values of two different types in one pack, an AdcInput enumerator and
// an AnalogIn. The divided battery would have been the obvious stand-in
// where no DAC can drive that pad, and it CANNOT BE ONE: on every class
// but the F405's the sensor and the battery share a channel, so a list
// naming both would be two inputs with one code and the sampler's own
// static_assert says so. A pad with nothing on it is still a distinct
// input, and this letter judges what it does with the LABEL, not with
// the value.
using Sampler = AnalogSampler<Adc1, P, Subs, AdcInput::vrefint, AdcInput::temperature, In4{}>;
constexpr const char* third_input_name = have_dac ? "the DAC's pad" : "an undriven pad";

struct Collector {
    using Event = std::variant<AnalogSample>;
    static inline EventQueue<Event, 8, P> queue;

    static inline uint16_t samples = 0;
    static inline uint16_t per_index[3] = {0, 0, 0};
    static inline uint16_t last[3] = {0, 0, 0};

    static void init() {
        samples = 0;
        for (uint8_t i = 0; i < 3u; ++i) {
            per_index[i] = 0;
            last[i] = 0;
        }
    }

    static void dispatch(const Event& e) {
        match(e, [](const AnalogSample& s) {
            if (s.index < 3u) {
                ++per_index[s.index];
                last[s.index] = s.value;
            }
            ++samples;
        });
    }
};

using AnalogKernel = Tenuto<P, Collector, Sampler>;

bool kernel_mode = false;

void tp_sampler() {
    if (!adc_up(base_config())) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
#if defined(DAC_BASE)
    (void)dac_up(dac_ch_pa4, 3000);
#endif
    (void)delay_us(clock, 20);
    console_drain();

    Adc1::interrupts(AdcFlag::converted, true);
    Nvic::clear_pending(Adc1::irq());
    Nvic::enable(Adc1::irq());
    AnalogKernel::init_all();
    kernel_mode = true;
    Sampler::start_every(2);   // one conversion every 2 ms

    // Tenuto::step() serves ONE queued event and nothing else - only
    // Tenuto::run() matures time events, and this loop is not run(). The
    // sampler's software pace IS a time event, so the pump does both
    // halves by hand.
    const uint32_t deadline = Ticker::ticks() + 400u;
    while (Collector::samples < 60u && Ticker::ticks() < deadline) {
        TimeEvents<P>::process();
        while (AnalogKernel::step()) {
        }
    }
    Sampler::stop();
    for (uint8_t i = 0; i < 8u; ++i) {
        (void)AnalogKernel::step();
    }
    kernel_mode = false;
    Nvic::disable(Adc1::irq());
    Adc1::interrupts(AdcFlag::converted, false);

    const uint16_t vdda = Adc1::vdda_mv(Collector::last[0]);
    print(serial, "  ", Collector::samples, " samples: VREFINT ", Collector::per_index[0],
          " (last ", Collector::last[0], " = VDDA ", vdda, " mV), sensor ",
          Collector::per_index[1], " (last ", Collector::last[1], "), ", third_input_name, " ",
          Collector::per_index[2], " (last ", Collector::last[2], "); unknown inputs ",
          Sampler::unknown_inputs(), ", queue overflows ", Collector::queue.overflows(), crlf);

    bench.verdict("AnalogSampler RUNS UNCHANGED ON THIS ARCHITECTURE: an active object "
                  "walking three inputs of a converter whose sequencer and DMA it does not use",
                  Collector::samples >= 60u);
    bench.verdict("the walk is EVEN - three inputs, three roughly equal counts, because the "
                  "sampler selects the next one on every result",
                  Collector::per_index[0] >= 18u && Collector::per_index[1] >= 18u &&
                      Collector::per_index[2] >= 18u);
    bench.verdict("and NOT ONE sample was mislabelled: every result carried a code the list "
                  "knows", Sampler::unknown_inputs() == 0u);
    bench.verdict("the values are the right ones for their labels", vdda >= 2700u && vdda <= 3600u);
#if defined(DAC_BASE)
    bench.verdict("and the DAC's pad reads near the code this letter set it to",
                  abs_diff(Collector::last[2], 3000u) < 200u);
#endif
    AdcCommon::vbat(false);
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// q - the ADC's DMA: a scan of four channels into memory
// =============================================================================

/// ADC1's request is DMA2's stream 0 on channel 0 (or stream 4 on the
/// same channel) - the reserve's own table, and the driver checks the
/// cell rather than trusting this line.
using AdcStream = DmaRxEngine<2, 0, 0, uint16_t>;
static_assert(Adc1::engine_placed<AdcStream>(),
              "the stream this suite uses must be one of ADC1's own cells");

volatile bool adc_dma_complete = false;
volatile uint8_t adc_dma_errors = 0;

void tq_adc_dma() {
    AdcConfig c = base_config();
    c.scan = true;                   // walk the sequence, one conversion each
    c.eoc_per_conversion = false;    // EOC at the END of the sequence
    c.dma = true;
    if (!adc_up(c)) {
        bench.verdict("the converter came up", false);
        return;
    }
    AdcCommon::internal_sources(true);
#if defined(DAC_BASE)
    Dac::init();
    (void)dac_channel_up(dac_ch_pa4, 1000);
    (void)dac_channel_up(dac_ch_pa5, 3000);
#endif
    (void)delay_us(clock, 30);

    static const uint8_t seq[4] = {In4::channel, In5::channel,
                                   adc_input_channel(AdcInput::vrefint),
                                   adc_input_channel(AdcInput::temperature)};
    bench.verdict("a four-rank regular sequence writes",
                  Adc1::regular_sequence(seq, 4) && Adc1::sequence_length() == 4u);

    static uint16_t block[4] = {0, 0, 0, 0};
    Dma<2>::init();
    adc_dma_complete = false;
    adc_dma_errors = 0;
    AdcStream::arm(Adc1::data_address());
    bench.verdict("the stream takes a run of four halfwords",
                  AdcStream::start(block, 4));

    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    uint32_t spins = 0;
    while (!adc_dma_complete && spins < 200000u) {
        ++spins;
    }
    const bool moved = adc_dma_complete;
    const bool overran = Adc1::overrun();
    print(serial, "  four ranks into memory after ", spins, " polls: ", block[0], " ", block[1],
          " ", block[2], " ", block[3], " (errors ", adc_dma_errors, ", OVR ",
          overran ? "up" : "down", ")", crlf);
    bench.verdict("ONE trigger, FOUR conversions, four halfwords in memory - the stream's own "
                  "completion says when", moved && adc_dma_errors == 0u);
    bench.verdict("and they are in the SEQUENCE's order, each rank on its own channel",
                  block[2] > 1200u && block[2] < 1800u && block[3] > 700u && block[3] < 1200u);
#if defined(DAC_BASE)
    bench.verdict("the first two ranks are the two DAC pads, two thousand codes apart",
                  abs_diff(block[0], 1000u) < 200u && abs_diff(block[1], 3000u) < 200u &&
                      block[1] > block[0]);
#endif
    bench.verdict("nothing overran: the stream took every result before the next arrived",
                  !overran);

    // DDS = 0 MEANS THE CONVERTER STOPS ASKING, and 13.8.1 says what it
    // takes to make it ask again: "the DMA bit is not cleared by hardware.
    // It must be written to 0, then to 1 to start a new transfer." So a
    // fresh stream over an untouched CR2 gets nothing at all.
    for (uint8_t i = 0; i < 4u; ++i) {
        block[i] = 0;
    }
    adc_dma_complete = false;
    (void)AdcStream::start(block, 4);
    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    (void)delay_us(clock, 400);
    const uint16_t stale_taken = AdcStream::take();
    const uint32_t stale_sr = Adc1::flags();
    print(serial, "  a second run over an untouched CR2 moved ", stale_taken,
          " halfwords and left SR ", hex(stale_sr), crlf);
    bench.verdict("13.8.1 MEASURED: with DDS clear the converter stops asking after the last "
                  "transfer, and a fresh stream over an untouched converter gets NOTHING - the "
                  "conversions run, the results pile in one register and no request is made",
                  stale_taken == 0u && (stale_sr & AdcFlag::converted) != 0u);

    // WHAT MAKES IT ASK AGAIN is not this letter's answer to give: neither
    // a cycled DMA bit nor a full re-initialization of both ends brought a
    // second block back here, and the arrangement the chapter really
    // points at for a stream that keeps running is DDS with a CIRCULAR
    // one, which these engines do not offer. It is in the gap list with
    // that reason.

    // AND THE SAME SEQUENCE WITH NO STREAM AT ALL. The counter-experiment
    // is the whole reason this register needs a DMA: four conversions into
    // one data register, three of them lost, and the flag that says so
    // (13.8.2's own arrangement - EOC at every conversion, no DMA).
    AdcStream::stop();
    AdcConfig polled = c;
    polled.dma = false;
    polled.eoc_per_conversion = true;
    (void)Adc1::configure(polled);
    Adc1::clear_flags(AdcFlag::all);
    Adc1::start();
    (void)delay_us(clock, 400);
    const bool lost = Adc1::overrun();
    const uint16_t only = Adc1::result();
    print(serial, "  the same four ranks with no stream and EOC at every conversion: OVR ",
          lost ? "up" : "down", ", one datum left in DR (", only, ")", crlf);
    bench.verdict("WITHOUT the stream the same sequence overruns - four conversions into one "
                  "register is what the DMA is for", lost);

    AdcStream::stop();
    Nvic::disable(Dma<2>::irq(0));
    AdcCommon::internal_sources(false);
    analog_down();
}

// =============================================================================
// r - the DAC's DMA: a table played by a hardware trigger, and the underrun
// =============================================================================
#if defined(DAC_BASE)
/// DAC channel 1's request is DMA1's stream 5 on channel 7, and its only
/// cell. THE ELEMENT IS A WORD: 14.5 says the DAC's registers "have to be
/// accessed by words (32 bits)", so the beat is 32 bits wide even though
/// the datum is twelve.
using DacStream = DmaTxEngine<1, 5, 7, uint32_t>;
static_assert(Dac::engine_placed<DacStream>(dac_ch_pa4),
              "the stream this suite uses must be DAC channel 1's own cell");

volatile bool dac_dma_complete = false;
volatile uint8_t dac_dma_errors = 0;
#endif

void tr_dac_dma() {
#if !defined(DAC_BASE)
    needs_dac("letter r");
#elif !defined(STM32F446xx)
    print(serial, "  no pad on EXTI line 9 this suite knows to be free on this board: the "
                  "DAC's DMA request comes from a HARDWARE trigger only (14.3.7) and there "
                  "is none to give it here", crlf);
#else
    {
        // The trigger is EXTI line 9 - the DAC's own table 94 row - driven
        // from a pad this program owns, because 14.3.7 raises no request
        // at all on the software trigger.
        using TrigPad9 = Pin<'C', 9>;   // free on the Nucleo's Morpho header
        constexpr uint8_t dac_line = dac_exti_line;

        if (!adc_up(base_config())) {
            bench.verdict("the converter came up", false);
            return;
        }
        Adc1::select(In4{});

        static const uint32_t table[8] = {200, 700, 1200, 1700, 2200, 2700, 3200, 3700};
        Dac::init();
        Dac::claim_pad<Pa4>();
        DacChannelConfig cfg{};
        cfg.triggered = true;
        cfg.trigger = DacTrigger::exti9;
        cfg.dma = true;
        bench.verdict("a DMA-fed, hardware-triggered channel configures",
                      Dac::configure(dac_ch_pa4, cfg));
        // 14.3.7: the FIRST datum is the holding register's, and the DMA
        // refills it behind each trigger - so the run starts at the
        // table's second entry.
        (void)Dac::write(dac_ch_pa4, table[0]);

        Dma<1>::init();
        dac_dma_complete = false;
        dac_dma_errors = 0;
        DacStream::arm(Dac::data_address_12r(dac_ch_pa4));
        bench.verdict("the stream takes the table's tail", DacStream::start(&table[1], 7));

        (void)Dac::enable(dac_ch_pa4, true);
        (void)delay_us(clock, 20);
        Dac::clear_flags(0xFFFFFFFFu);

        // ES0206 2.6.1 / ES0298 2.7.1 IS WHY THIS COUNT EXISTS. A request
        // left pending when a DMA-to-DAC round is stopped survives both
        // the DMAEN clear and the clock being closed, and is served the
        // moment the converter comes back - so the stream can have taken
        // a beat before any edge. It was seen once, after this letter
        // followed the ADC's DMA letter, and it does not reproduce on
        // demand; so the offset is COUNTED and the table compared from
        // where the stream really is, rather than assumed to be zero.
        (void)delay_us(clock, 10);
        const uint16_t spurious = DacStream::progress().done;

        TrigPad9::output(false);
        (void)Exti::steal(dac_line, 'C');
        (void)Exti::sense(dac_line, ExtiSense::rising);
        (void)Exti::event(dac_line, true);
        (void)Exti::clear(dac_line);

        const uint8_t edges = static_cast<uint8_t>(8u - spurious);
        uint16_t played[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        bool followed = true;
        for (uint8_t i = 0; i < edges; ++i) {
            (void)Exti::clear(dac_line);
            TrigPad9::set();
            (void)delay_us(clock, 2);
            TrigPad9::clear();
            (void)delay_us(clock, 40);   // the DAC's settling, then a conversion
            played[i] = Adc1::read();
            if (abs_diff(played[i], static_cast<uint16_t>(table[i + spurious])) > 120u) {
                followed = false;
            }
        }
        print(serial, "  ", edges, " edges played");
        for (uint8_t i = 0; i < edges; ++i) {
            print(serial, " ", played[i]);
        }
        print(serial, " for a table of 200..3700 by 500 from entry ", spurious, crlf);
        bench.verdict("A TABLE PLAYED BY HARDWARE: one pad edge, one trigger, one DMA beat "
                      "into the holding register, and the pad follows the table",
                      followed && edges >= 7u);
        bench.verdict("and the stream reported its own completion after the last beat",
                      dac_dma_complete && dac_dma_errors == 0u);
        print(serial, "  beats taken before the first edge: ", spurious, crlf);
        bench.verdict("and AT MOST ONE beat was taken before the first trigger - the count is "
                      "the SPURIOUS REQUEST of ES0206 2.6.1 / ES0298 2.7.1, which this letter "
                      "measures instead of assuming away because a request left pending by an "
                      "earlier round survives the DMAEN clear and the clock being closed",
                      spurious <= 1u);
        bench.verdict("no underrun while the stream was serving - and none on the ONE trigger "
                      "past its last beat either, because 14.3.7 wants a SECOND unserved one",
                      !Dac::underrun(dac_ch_pa4));

        // THE UNDERRUN. The stream is spent, so the next triggers raise
        // requests nobody acknowledges - 14.3.7's own condition - and the
        // channel keeps converting the old datum.
        for (uint8_t i = 0; i < 3u; ++i) {
            (void)Exti::clear(dac_line);
            TrigPad9::set();
            (void)delay_us(clock, 2);
            TrigPad9::clear();
            (void)delay_us(clock, 20);
        }
        const bool under = Dac::underrun(dac_ch_pa4);
        const uint16_t held = Adc1::read();
        print(serial, "  three more edges with the stream spent: DMAUDR ", under ? "up" : "down",
              ", the pad still at ", held, crlf);
        bench.verdict("A SPENT STREAM IS AN UNDERRUN: the trigger keeps arriving, nobody "
                      "acknowledges the request, and DMAUDRx says so", under);
        bench.verdict("and the channel keeps converting the OLD datum rather than going quiet",
                      abs_diff(held, static_cast<uint16_t>(table[7])) < 150u);

        // The errata's stop sequence, as far as this driver reaches.
        bench.verdict("stop_dma() clears the flag and drops both DMAEN and the enable",
                      Dac::stop_dma(dac_ch_pa4) && !Dac::underrun(dac_ch_pa4) &&
                          !Dac::enabled(dac_ch_pa4));
        DacStream::stop();
        Nvic::disable(Dma<1>::irq(5));
        TrigPad9::release();
        (void)Exti::release(dac_line);
        analog_down();
    }
#endif
}

void banner() {
    print(serial, crlf, "test_stm32f4_analog - the ADC and the DAC, measured through the pad "
          "they share", crlf);
    bench.menu();
}

} // namespace

// ---- target glue --------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// The ADC's one vector, shared by every converter. Letter p is the only
/// user; every other letter polls.
/// ONE VECTOR PER STREAM on this family, so each of these answers for
/// itself alone. The engines' `service()` reads and clears; what a
/// completion MEANS is the owner's, which here is a flag a letter waits
/// on.
extern "C" void DMA2_Stream0_IRQHandler() {
    const uint8_t hit = AdcStream::service();
    if ((hit & AdcStream::flag_complete) != 0u) {
        adc_dma_complete = true;
    }
    if ((hit & (AdcStream::flag_error | AdcStream::flag_fifo_error)) != 0u) {
        adc_dma_errors = static_cast<uint8_t>(adc_dma_errors + 1u);
    }
}

#if defined(DAC_BASE)
extern "C" void DMA1_Stream5_IRQHandler() {
    const uint8_t hit = DacStream::service();
    if ((hit & DacStream::flag_complete) != 0u) {
        (void)DacStream::complete();
        dac_dma_complete = true;
    }
    if ((hit & (DacStream::flag_error | DacStream::flag_fifo_error)) != 0u) {
        dac_dma_errors = static_cast<uint8_t>(dac_dma_errors + 1u);
    }
}
#endif

extern "C" void ADC_IRQHandler() {
    const uint32_t hit = Adc1::isr();
    if (kernel_mode && (hit & brio::AdcFlag::converted) != 0u) {
        brio::post<Sampler>(brio::Sampled{Adc1::result(), Adc1::selected()});
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(false);
    brio::enable_interrupts();

    bench.letter('a', "the block, the prescaler and every refusal", ta_block);
    bench.letter('b', "the DAC as an actuator: holding, output, formats, triggers", tb_dac);
    bench.letter('c', "the zero-length wire: the transfer curve", tc_transfer);
    bench.letter('d', "buffered against unbuffered, free pad and loaded pad", td_buffer);
    bench.letter('e', "the settling time after a step", te_settling);
    bench.letter('f', "the conversion time in ADCCLK cycles", tf_timing);
    bench.letter('g', "the scale: VDDA, the junction temperature, the battery", tg_scale);
    bench.letter('h', "the analog watchdog on a DAC step", th_watchdog);
    bench.letter('i', "the injected group and its preemption", ti_injected);
    bench.letter('j', "dual regular simultaneous mode", tj_dual);
    bench.letter('k', "the wave generators", tk_waves);
    bench.letter('l', "overrun and the two data-management modes", tl_overrun);
    bench.letter('m', "the sequencer-during-conversion erratum", tm_erratum);
    bench.letter('n', "the flash accelerator against the converter's spread", tn_noise);
    bench.letter('o', "the external trigger path through the EXTI", to_trigger);
    bench.letter('p', "AnalogSampler inside a real kernel", tp_sampler);
    bench.letter('q', "the ADC's DMA: a scan of four channels into memory", tq_adc_dma);
    bench.letter('r', "the DAC's DMA: a table played by a hardware trigger", tr_dac_dma);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " ADCs=", brio::adc_instances(),
                    " DAC=", have_dac ? "yes" : "no", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t ch = 0;
        if (!Serial::read_byte(ch)) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(ch), brio::crlf);
        if (ch == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(ch))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
