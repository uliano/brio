// test_stm32_analog - the reference bench suite for the STM32G0's ANALOG
// BLOCK: the ADC (RM0444 ch. 15), the DAC (ch. 16), the voltage
// reference buffer (ch. 17) and the three comparators (ch. 18) - and,
// through them, util/analog.hpp and util/analog_sampler.hpp on their
// THIRD silicon.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// ONE SUITE FOR FOUR CHAPTERS, and the reason is the silicon rather than
// convenience: on this family the DAC's only route to the ADC is a PAD
// (figure 36 lists no DAC among the ADC's nineteen inputs), its only
// internal route is a COMPARATOR's inverting input, and VREF+ is the one
// rail all three measure against with the VREFBUF as the only thing that
// could change it. Every letter below needs at least two of the four
// chapters up, so splitting them would mean writing the same bring-up
// three times and flashing the board three times to judge one desk.
//
// NOTHING TO WIRE. Four techniques carry it:
//   1. THE ZERO-LENGTH WIRE. PA4 is DAC1_OUT1 and ADC_IN4 at once
//      (DS13560 table 12), so the DAC drives an ADC input through one
//      bond pad and nothing else - the samc campaign's PA02 trick, on a
//      part where it is the ONLY route between the two converters.
//   2. THE FACTORY VALUES AS THE SCALE. VREFINT and its calibration at
//      0x1FFF75AA give VDDA in millivolts without a meter; TS_CAL1 and
//      TS_CAL2 give the junction temperature. Both are ADC results taken
//      at 3.0 V, so the arithmetic is a ratio and the board's own supply
//      is what comes out.
//   3. AN INTERNAL THRESHOLD AGAINST A DRIVEN PAD. A comparator's
//      inverting input reaches VREFINT, its three taps and both DAC
//      channels with no pad at all, while its NON-inverting input is a
//      pad and nothing else (tables 93/95/97 have no internal signal) -
//      so a comparator here is exercised against a GPIO-driven rail, and
//      the threshold sweep that would need an analog voltage on that pad
//      is DECLINED IN PRINT rather than faked (letter i).
//   4. A COMPARATOR MEASURED WITH NO PAD ON THE OTHER SIDE EITHER:
//      COMP1's output reaches TIM1's TI1 through TIM1_TISEL (21.4.28)
//      and EXTI line 17 through the shared ADC vector, so its edges are
//      counted twice by two mechanisms sharing nothing.
//
// THE PADS, each proven electrically free by letter a before anything
// after it is believed:
//   PA0   ADC_IN0, COMP1_INM8      a rail, and a second ADC channel
//   PA1   ADC_IN1, COMP1_INP2      the comparator's signal, GPIO-driven
//   PA4   ADC_IN4, DAC1_OUT1       THE ZERO-LENGTH WIRE
//   PA5   ADC_IN5, DAC1_OUT2, LD4  the LED as an analog dimmer
// Avoided on purpose: PA2/PA3 (the console), PA13/PA14 (SWD), PC13 (B1),
// PC14/PC15 (the LSE pads), PF0/PF1 (the HSE pads). COMP2 is exercised
// through WINMODE, which borrows COMP1's own pad (18.6.1), so it needs
// none of its own; COMP3's three pads are left alone and its letter says
// what that costs.
//
// NOTHING FORCED. No flash is written (the linker's rom is bank 1 and
// bank 2 holds test_stm32_nvm's and test_stm32_journal's live storage),
// no option byte, no comparator LOCK - the bit is one-way until a reset
// (18.3.4) and letter i only reports that it is clear - and THE VREFBUF
// IS NEVER ENABLED: 17.1 says the buffer must stay off where VREF+ is
// tied to a supply, nothing inside the chip can tell that case from a
// free pin, and this board's schematic is not a document this project
// has. Letter a says so and reads the block instead.
//
// What is exercised, letter by letter:
//   a  the block: the reserve's facts, the boot registers, the regulator
//      and the calibration, every refusal, the four pads, the VREFBUF
//   b  the scale: VDDA from VREFINT, the junction temperature from
//      TS_CAL1/2, VBAT, and the two of them cross-checked
//   c  conversion time exact to the CPU cycle per sampling time and
//      resolution, and ES0548 2.6.4 measured
//   d  the zero-length wire: the DAC's transfer curve read back through
//      ADC_IN4, monotonic, with the buffer's own swing limits
//   e  the LED as an analog dimmer: DAC channel 2 on PA5, read back
//   f  the sequencer, both faces, and the CCRDY handshake
//   g  the noise floor MEASURED, and then the oversampler against it
//   h  the three analog watchdogs, and ES0548 2.6.3 staged
//   i  the comparators: the muxes, polarity, hysteresis, blanking, the
//      window pair, EXTI line 17 and TIM1's TI1 - and what is declined
//   j  AnalogSampler inside a REAL KERNEL walking three inputs
//   k  the no-CPU chain: one TIM6 TRGO driving BOTH converters, a DMA
//      table into the DAC and a DMA stream out of the ADC
//   l  ES0548 2.6.2 staged with a control, and the rest of the pass
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/adc.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/comp.hpp"
#include "stm32g0/dac.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "stm32g0/vref.hpp"
#include "util/analog.hpp"
#include "util/analog_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

// ---- the pads --------------------------------------------------------------
using PadA0 = Pin<'A', 0>;   // ADC_IN0, COMP1_INM8
using PadA1 = Pin<'A', 1>;   // ADC_IN1, COMP1_INP2 - the comparator's signal
using PadA4 = Pin<'A', 4>;   // ADC_IN4, DAC1_OUT1  - the zero-length wire
using PadA5 = Pin<'A', 5>;   // ADC_IN5, DAC1_OUT2, LD4

using In0 = AnalogIn<PadA0, 0>;
using In1 = AnalogIn<PadA1, 1>;
using In4 = AnalogIn<PadA4, 4>;
using In5 = AnalogIn<PadA5, 5>;

using C1 = Comp<1>;
using C2 = Comp<2>;
using C3 = Comp<3>;

using T6 = Tim<6>;   // the basic timer: a time base and a TRGO, nothing else
using T1 = Tim<1>;   // TISEL reaches COMP1's output on TI1

// ---- the DMA engines --------------------------------------------------------
// Channel 1 plays a table into the DAC's holding register, channel 2
// drains the ADC's data register. Both are halfword engines: a DAC
// holding register and an ADC result are 12-bit data in a 16-bit word,
// and an engine whose element disagrees with the register it was pointed
// at writes a number the converter never meant.
using DacLoop = DmaLoopEngine<1, 1, uint16_t>;
using AdcPong = DmaPingPongEngine<1, 2, uint16_t>;

// ---- what this boot found ---------------------------------------------------
uint32_t boot_apbenr1 = 0;
uint32_t boot_apbenr2 = 0;
uint32_t boot_adc_cr = 0;
uint32_t boot_adc_cfgr1 = 0;
uint32_t boot_vrefbuf_csr = 0;
uint32_t boot_vrefbuf_clockless = 0;
uint32_t boot_comp1_csr = 0;
uint32_t boot_ccipr = 0;

// ---- what the handlers count -------------------------------------------------
volatile uint32_t adc_eoc_calls = 0;
volatile uint32_t adc_awd_calls = 0;
volatile uint32_t comp_exti_calls = 0;
volatile uint32_t comp_exti_rising = 0;
volatile uint32_t comp_exti_falling = 0;
volatile uint32_t dac_underrun_calls = 0;
volatile uint32_t pong_blocks = 0;
volatile uint32_t loop_laps = 0;
volatile bool kernel_mode = false;

void clear_counts() {
    adc_eoc_calls = 0;
    adc_awd_calls = 0;
    comp_exti_calls = 0;
    comp_exti_rising = 0;
    comp_exti_falling = 0;
    dac_underrun_calls = 0;
    pong_blocks = 0;
    loop_laps = 0;
}

// =============================================================================
// Instruments
// =============================================================================

/// The other suites' cycle-resolution stopwatch: whole ticks times the
/// SysTick period plus the phase it has already counted down, with the
/// two reads retried until they belong to the same tick.
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;

void spin_cycles(uint32_t c) {
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < c) {
    }
}

/// Wait for the console to be physically empty. A measurement window a
/// transmit interrupt walks through is not a measurement - three earlier
/// campaigns on this desk paid for that lesson.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    spin_cycles(SysClock::hz / 500u);   // 2 ms of quiet on top
}

/// THE PRECONDITION OF EVERY PAD LETTER: an input pad with nothing
/// attached goes where its own pull sends it.
template <class Pad>
bool pad_follows_pull() {
    Pad::input(PinPull::up);
    (void)delay_us(clock, 300);
    const bool high = Pad::read();
    Pad::input(PinPull::down);
    (void)delay_us(clock, 300);
    const bool low = Pad::read();
    Pad::input(PinPull::none);
    return high && !low;
}

/// Disable, reconfigure, re-enable - 15.3.7's rule made one verb. The
/// calibration survives a disable (15.3.3), so this costs nothing but the
/// enable's own start-up.
bool apply(const AdcConfig& c) {
    (void)Adc::stop();
    if (!Adc::disable()) {
        return false;
    }
    if (!Adc::configure(c)) {
        return false;
    }
    return Adc::enable();
}

/// One conversion of one channel, the whole way round.
uint16_t convert(uint8_t channel) {
    (void)Adc::select_channel(channel);
    return Adc::read();
}

template <class In>
uint16_t convert(In in) {
    (void)Adc::select_sync(in);
    return Adc::read();
}

/// The median of five conversions - enough to make a printed number
/// repeatable without claiming anything about noise, which letter g
/// measures properly.
uint16_t convert_median(uint8_t channel) {
    uint16_t v[5];
    (void)Adc::select_channel(channel);
    for (uint8_t i = 0; i < 5; ++i) {
        v[i] = Adc::read();
    }
    for (uint8_t i = 1; i < 5; ++i) {
        const uint16_t key = v[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            --j;
        }
        v[j + 1] = key;
    }
    return v[2];
}

/// The two configurations every letter picks between: a SLOW one for the
/// internal channels (VREFINT wants 4 us of sampling, the temperature
/// sensor 5 and VBAT 12 - DS13560 tables 27, 65 and 66; 160.5 cycles at
/// the asynchronous 8 MHz is 20 us, which covers all three), and a FAST
/// one for the DAC-driven pads, whose source is a low-impedance buffer.
constexpr AdcConfig cfg_internal{
    .clock_mode = AdcClockMode::async,
    .async_source = AdcAsyncSource::hsi16,
    .prescaler = AdcPresc::div2,
    .sample1 = AdcSampleTime::cycles160_5,
};
constexpr AdcConfig cfg_pad{
    .clock_mode = AdcClockMode::pclk_div2,
    .sample1 = AdcSampleTime::cycles39_5,
};
constexpr uint32_t async_hz = 16'000'000UL;

/// VDDA, measured through VREFINT under `cfg_internal`. Every absolute
/// millivolt this suite prints is on this scale.
uint16_t measure_vdda() {
    const AdcConfig keep = Adc::config();
    (void)apply(cfg_internal);
    Adc::vrefint(true);
    (void)delay_us(clock, 100);          // tstart_vrefint is 12 us max
    const uint16_t raw = convert_median(Adc::vrefint_channel);
    (void)apply(keep);
    return Adc::vdda_mv(raw);
}

/// Everything this suite ever turns on, off again.
void quiet_everything() {
    Nvic::disable(Adc::irq());
    Nvic::disable(Dac::irq());
    Nvic::disable(DMA1_Channel1_IRQn);
    Nvic::disable(DMA1_Channel2_3_IRQn);
    DacLoop::stop();
    AdcPong::stop();
    (void)Exti::release(C1::exti_line);
    (void)Exti::release(C2::exti_line);
    (void)C1::release();
    (void)C2::release();
    (void)C3::release();
    T6::release();
    T1::release();
    Adc::release();
    Dac::release();
    PadA0::input(PinPull::none);
    PadA1::input(PinPull::none);
    PadA4::analog();
    PadA5::analog();
    Nvic::clear_pending(Adc::irq());
    Nvic::clear_pending(Dac::irq());
    kernel_mode = false;
}

/// The bring-up every measuring letter starts from.
bool analog_up(const AdcConfig& c) {
    quiet_everything();
    clear_counts();
    Dac::init();
    Vref::init();
    return Adc::init(clock, c, async_hz);
}

// =============================================================================
// a - the block: the reserve's facts, the boot registers, the regulator
//     and the calibration, every refusal, the pads, and the VREFBUF
// =============================================================================
void ta_block() {
    print(serial, "  reserve: adc_channels=", adc_channels(),
          " temp=", adc_temperature_channel(), " vrefint=", adc_vrefint_channel(),
          " vbat=", adc_vbat_channel(), " dac_channels=", dac_channels(),
          " comp_count=", comp_count(), " vrefbuf=", vrefbuf_present(), crlf);
    print(serial, "  at boot: APBENR1=", hex(boot_apbenr1), " APBENR2=", hex(boot_apbenr2),
          " ADC_CR=", hex(boot_adc_cr), " ADC_CFGR1=", hex(boot_adc_cfgr1),
          " VREFBUF_CSR=", hex(boot_vrefbuf_csr), " COMP1_CSR=", hex(boot_comp1_csr),
          " CCIPR=", hex(boot_ccipr), " (VREFBUF_CSR through the CLOSED "
          "SYSCFG gate: ", hex(boot_vrefbuf_clockless), ")", crlf);

    bench.verdict("the G0B1 carries one ADC of nineteen channels, a two-channel "
                  "DAC, three comparators and the reference buffer",
                  adc_present() && adc_channels() == 19 && dac_present() &&
                      dac_channels() == 2 && comp_count() == 3 && vrefbuf_present());
    bench.verdict("the ADC's vector is SHARED with all three comparators, and "
                  "the DAC's with TIM6 and LPTIM1 (table 61)",
                  Adc::irq() == ADC1_COMP_IRQn && C1::irq() == ADC1_COMP_IRQn &&
                      C3::irq() == ADC1_COMP_IRQn && Dac::irq() == TIM6_DAC_LPTIM1_IRQn);
    bench.verdict("the comparators' EXTI lines are 17, 18 and 20, and all "
                  "three are CONFIGURABLE lines (13.5.1)",
                  C1::exti_line == 17 && C2::exti_line == 18 && C3::exti_line == 20 &&
                      Exti::configurable(17) && Exti::configurable(18) &&
                      Exti::configurable(20));
    bench.verdict("WINMODE's partner is NOT n + 1: COMP1 borrows COMP2's plus "
                  "input, COMP2 borrows COMP1's, COMP3 borrows COMP2's (18.6.1, "
                  "one register description at a time)",
                  C1::window_partner == 2 && C2::window_partner == 1 &&
                      C3::window_partner == 2);

    // What this boot found, before a line of this suite ran.
    bench.verdict("every analog block came up with its APB clock CLOSED, "
                  "which is why init() opens it before anything else",
                  (boot_apbenr2 & RCC_APBENR2_ADCEN) == 0u &&
                      (boot_apbenr1 & RCC_APBENR1_DAC1EN) == 0u);
    bench.verdict("the ADC came up OFF with its regulator down and CFGR1 at "
                  "zero - so 12 bits, right aligned, no trigger",
                  (boot_adc_cr & (ADC_CR_ADEN | ADC_CR_ADVREGEN)) == 0u &&
                      boot_adc_cfgr1 == 0u);
    bench.verdict("VREFBUF_CSR reads 0x2 out of reset - ENVR clear and HIZ "
                  "SET, which is table 91's 'external voltage reference mode' "
                  "and the only safe default",
                  boot_vrefbuf_csr == 0x2u && !Vref::enabled() && Vref::high_impedance());
    bench.verdict("...and it reads 0x0 THROUGH THE CLOSED SYSCFG GATE, which "
                  "is a trap chapter 17 cannot warn about: the block has no "
                  "clock of its own and lives at SYSCFG_BASE + 0x30, so an "
                  "application reading it before opening that gate would "
                  "conclude VREF+ was being PULLED DOWN TO VSSA (table 91's "
                  "other off mode) when it is not",
                  boot_vrefbuf_clockless == 0x0u);
    bench.verdict("and RCC_CCIPR.ADCSEL came up at 00 (system clock), the "
                  "asynchronous root nothing has chosen yet",
                  ((boot_ccipr >> RCC_CCIPR_ADCSEL_Pos) & 0x3u) == 0u);

    // The reference buffer, read and NOT enabled - see the file header.
    print(serial, "  VREFBUF: trim=", Vref::trim(), " scale=",
          Vref::scale() == VrefScale::v2_5 ? "2.5V" : "2.048V",
          " reference=", Vref::reference() == Ref::external ? "external" : "buffer", crlf);
    bench.verdict("the buffer's factory trim is loaded and plausible (17.3.2: "
                  "TRIM is initialized from the production value)",
                  Vref::trim() != 0u && Vref::trim() <= 63u);
    bench.verdict("and enable() REFUSES without the board acknowledgement: "
                  "17.1 forbids driving VREF+ where it is tied to a supply, "
                  "and no register can tell that case from a free pin",
                  !Vref::enable({.scale = VrefScale::v2_048,
                                 .board_vref_pin_is_free = false}) &&
                      !Vref::enabled());

    // The bring-up, step by step.
    Dac::init();
    Adc::bus_clock(true);
    Adc::reset();
    const bool reg_ok = Adc::regulator_on(clock);
    const bool cal_ok = Adc::calibrate();
    const uint8_t factor = Adc::calibration();
    print(serial, "  regulator=", reg_ok, " calibration=", cal_ok,
          " CALFACT=", factor, crlf);
    bench.verdict("the regulator comes up and the SELF CALIBRATION runs, "
                  "leaving a plausible factor in CALFACT (15.3.3) - which is "
                  "what this converter has instead of a factory trim",
                  reg_ok && cal_ok && factor != 0u && factor <= 0x7Fu);

    const bool cfg_ok = Adc::configure(cfg_pad);
    const bool en_ok = Adc::enable();
    bench.verdict("configure() then enable(), and ADRDY rises (15.3.4's own "
                  "procedure)", cfg_ok && en_ok && Adc::enabled() &&
                                    Adc::flag(AdcFlag::ready));

    // The refusals, and the one that is a structural erratum answer.
    bench.verdict("a Reserved ADC prescaler code is refused",
                  !Adc::configure({.prescaler = static_cast<AdcPresc>(13)}));
    bench.verdict("continuous AND discontinuous together is refused - 15.4.1 "
                  "says the pair is forbidden and nothing in the silicon "
                  "stops it", !adc_config_valid({.continuous = true,
                                                 .discontinuous = true}));
    bench.verdict("configure() REFUSES while the converter is enabled, which "
                  "is 15.3.7's rule AND the structural answer to ES0548 2.6.2",
                  Adc::enabled() && !Adc::configure(cfg_pad));
    bench.verdict("a calibration is refused while the converter is enabled "
                  "(15.3.3's first precondition)", !Adc::calibrate());
    bench.verdict("a channel past the count, and a sequence bit past it, are "
                  "refused", !Adc::select_channel(19) && !Adc::sequence(1u << 19));
    bench.verdict("an ordered sequence is refused while CHSELRMOD is clear, "
                  "and a bitmap sequence while it is set",
                  !Adc::sequence_ordered(nullptr, 0));
    bench.verdict("a DAC channel past this device's count is refused",
                  !Dac::configure(2, {}) && !Dac::write(2, 0) && !Dac::enable(2, true));
    bench.verdict("a DAC wave generator without a trigger is refused - "
                  "16.7.1: WAVEx is only used with TENx set",
                  !Dac::configure(0, {.triggered = false, .wave = DacWave::noise}));
    bench.verdict("a DAC trigger code table 85 leaves empty is refused",
                  !dac_trigger_valid(static_cast<DacTrigger>(4)) &&
                      !Dac::configure(0, {.triggered = true,
                                          .trigger = static_cast<DacTrigger>(9)}));
    bench.verdict("a comparator power mode 18.6.1 marks Reserved is refused, "
                  "and so is a blanking bit past the five",
                  !C1::configure({.power = static_cast<CompPower>(3)}) &&
                      !C1::configure({.blanking = 0x20}));

    // The pads, before anything rests on them.
    console_drain();
    const bool a0 = pad_follows_pull<PadA0>();
    const bool a1 = pad_follows_pull<PadA1>();
    const bool a4 = pad_follows_pull<PadA4>();
    const bool a5 = pad_follows_pull<PadA5>();
    print(serial, "  pads follow their own pull: PA0=", a0, " PA1=", a1,
          " PA4=", a4, " PA5=", a5, crlf);
    bench.verdict("ALL FOUR pads are electrically free - each walks between "
                  "the rails under its own internal pull, which is the "
                  "precondition of every letter after this one",
                  a0 && a1 && a4 && a5);
    bench.verdict("PA5 CARRIES LD4 AND STILL FOLLOWS A 40 KOHM PULL, so the "
                  "lamp's drive path on this board is high impedance and not "
                  "an LED with a resistor to ground - which letter e then "
                  "confirms from the analog side", a5);

    PadA4::analog();
    PadA5::analog();
    bench.verdict("an analog pad reads ZERO on IDR whatever it carries - "
                  "7.3.1 turns the input buffer off in analog mode, which is "
                  "why no letter here judges a converted pad by its logic level",
                  !PadA4::read());
    quiet_everything();
}

// =============================================================================
// b - the scale: VDDA from VREFINT, the temperature from TS_CAL1/2, VBAT
// =============================================================================
void tb_scale() {
    if (!analog_up(cfg_internal)) {
        bench.verdict("the ADC came up on its asynchronous clock", false);
        return;
    }
    print(serial, "  factory: VREFINT_CAL=", AdcFactory::vrefint_cal(),
          " TS_CAL1=", AdcFactory::ts_cal1(), " TS_CAL2=", AdcFactory::ts_cal2(),
          " (all at VDDA = VREF+ = 3.0 V; TS at 30 C and 130 C)", crlf);
    bench.verdict("the three engineering values are present and ordered - "
                  "TS_CAL2 above TS_CAL1, the sensor rising with temperature",
                  AdcFactory::plausible() &&
                      AdcFactory::ts_cal2() > AdcFactory::ts_cal1());

    Adc::vrefint(true);
    Adc::temperature(true);
    Adc::vbat(true);
    (void)delay_us(clock, 200);

    const uint16_t vref_raw = convert_median(Adc::vrefint_channel);
    const uint16_t ts_raw = convert_median(Adc::temperature_channel);
    const uint16_t vbat_raw = convert_median(Adc::vbat_channel);
    const uint16_t vdda = Adc::vdda_mv(vref_raw);
    const int32_t temp = Adc::temperature_centi_c(ts_raw, vdda);
    const uint16_t vbat_mv = static_cast<uint16_t>(
        3u * adc_mv(vbat_raw, Adc::result_steps(), vdda));

    print(serial, "  VREFINT=", vref_raw, " counts = VDDA ", vdda, " mV; ",
          "TSENSE=", ts_raw, " counts = ", temp, " centi-C; ",
          "VBAT/3=", vbat_raw, " counts = VBAT ", vbat_mv, " mV", crlf);

    bench.verdict("VDDA MEASURED WITH NO METER: 15.9's own ratio against "
                  "VREFINT_CAL puts this board's analog supply inside the "
                  "3.0..3.6 V a Nucleo's 3.3 V regulator can produce",
                  vdda >= 3000u && vdda <= 3600u);
    bench.verdict("and the reading is on the right side of the calibration - "
                  "a supply ABOVE the 3.0 V ST used gives FEWER counts for the "
                  "same bandgap", vref_raw < AdcFactory::vrefint_cal());
    bench.verdict("the junction temperature lands in a range a powered board "
                  "on a desk can be in (0..70 C)", temp > 0 && temp < 7000);
    bench.verdict("and it sits between the two calibration points' own raw "
                  "readings, so the two-point line is being interpolated and "
                  "not extrapolated",
                  ts_raw > AdcFactory::ts_cal1() / 2u && ts_raw < AdcFactory::ts_cal2());
    bench.verdict("VBAT reads a third of a supply, which is what the divider "
                  "in front of that channel does (15.3.8) - the pin is tied "
                  "to VDD on this board",
                  vbat_mv > (vdda - 400u) && vbat_mv < (vdda + 400u));

    // The bandgap weighed a second way: VREFINT converted at a DIFFERENT
    // resolution must give the same VOLTAGE, which is a check on the
    // resolution arithmetic and not on the bandgap.
    AdcConfig ten = cfg_internal;
    ten.resolution = AdcRes::bits10;
    (void)apply(ten);
    const uint16_t vref10 = convert_median(Adc::vrefint_channel);
    const uint16_t mv12 = adc_mv(vref_raw, 4096, vdda);
    const uint16_t mv10 = adc_mv(vref10, 1024, vdda);
    print(serial, "  VREFINT at 12 bits ", mv12, " mV, at 10 bits ", mv10,
          " mV (", vref10, " counts of 1024)", crlf);
    bench.verdict("the same bandgap converted at 10 bits gives the same "
                  "VOLTAGE to within a count of the coarser scale, so the "
                  "full-scale arithmetic util/analog.hpp is handed is right "
                  "at both resolutions",
                  static_cast<uint16_t>(mv12 > mv10 ? mv12 - mv10 : mv10 - mv12) <= 6u);
    bench.verdict("and the datasheet's own number is met: VREFINT is 1.182 "
                  "to 1.232 V over the whole temperature range (table 27)",
                  mv12 >= 1150u && mv12 <= 1260u);
    quiet_everything();
}

// =============================================================================
// c - conversion time, exact to the CPU cycle, and ES0548 2.6.4
// =============================================================================
//
// With CKMODE = PCLK/2 the ADC clock is EXACTLY half the CPU clock, so a
// conversion's predicted length in CPU cycles is an integer: 2 x (tSMPL +
// tSAR), both of which the chapter gives in halves. The instrument is a
// DMA channel, not a polling loop: at 1.5 + 12.5 cycles a conversion is
// 28 CPU cycles and no reader could keep up, while a channel counting
// down CNDTR costs the CPU nothing at all.
uint32_t timed_conversions(const AdcConfig& c, uint8_t channel, uint16_t count,
                           uint16_t* buffer) {
    AdcConfig run = c;
    run.continuous = true;
    run.dma = true;
    run.overrun_overwrite = true;
    if (!apply(run)) {
        return 0;
    }
    if (!Adc::select_channel(channel)) {
        return 0;
    }
    using Ch = DmaChannel<1, 2>;
    Ch::stop();
    (void)DmaMux::request(Ch::mux_channel, Adc::dma_request);
    const bool loaded = Ch::load({.peripheral = Adc::data_address(),
                                  .memory = buffer,
                                  .count = count,
                                  .config = {.direction = DmaDirection::peripheral_to_memory,
                                             .peripheral_width = DmaWidth::half,
                                             .memory_width = DmaWidth::half}});
    if (!loaded) {
        return 0;
    }
    const uint32_t t0 = cycles_now();
    Adc::start();
    uint32_t guard = 0;
    while (!Ch::flag(DmaFlag::complete) && ++guard < 20'000'000UL) {
    }
    const uint32_t elapsed = cycles_now() - t0;
    (void)Adc::stop();
    Ch::stop();
    (void)DmaMux::release(Ch::mux_channel);
    return guard >= 20'000'000UL ? 0 : elapsed;
}

uint16_t timing_buffer[256];

void tc_timing() {
    if (!analog_up(cfg_pad)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    Dma<1>::bus_clock(true);
    Adc::vrefint(true);
    (void)delay_us(clock, 100);
    console_drain();

    struct Row {
        AdcSampleTime smp;
        AdcRes res;
        const char* name;
    };
    const Row rows[] = {
        {AdcSampleTime::cycles1_5, AdcRes::bits12, "1.5 + 12.5"},
        {AdcSampleTime::cycles3_5, AdcRes::bits12, "3.5 + 12.5"},
        {AdcSampleTime::cycles7_5, AdcRes::bits12, "7.5 + 12.5"},
        {AdcSampleTime::cycles39_5, AdcRes::bits12, "39.5 + 12.5"},
        {AdcSampleTime::cycles7_5, AdcRes::bits6, "7.5 + 6.5"},
        {AdcSampleTime::cycles160_5, AdcRes::bits12, "160.5 + 12.5"},
    };
    constexpr uint16_t n = 256;
    uint8_t exact = 0;
    uint8_t one_over = 0;
    uint8_t rows_done = 0;
    for (const Row& r : rows) {
        AdcConfig c = cfg_pad;
        c.sample1 = r.smp;
        c.resolution = r.res;
        const uint32_t cycles = timed_conversions(c, 0, n, timing_buffer);
        if (cycles == 0u) {
            continue;
        }
        ++rows_done;
        // Predicted CPU cycles per conversion: the half-cycle sum, and
        // fADC is exactly half the CPU clock under PCLK/2.
        const uint32_t predicted = adc_conversion_half_cycles(c, 0);
        const uint32_t measured_half = (cycles + n / 2u) / n;   // CPU cycles = ADC halves
        print(serial, "    ", r.name, " cycles: predicted ", predicted / 2u, ".",
              (predicted & 1u) ? 5u : 0u, " ADC cycles, measured ",
              measured_half / 2u, ".", (measured_half & 1u) ? 5u : 0u,
              " (", cycles, " CPU cycles for ", n, " conversions)", crlf);
        const uint32_t diff = measured_half > predicted ? measured_half - predicted
                                                        : predicted - measured_half;
        if (diff <= 1u) {
            ++exact;
        } else if (measured_half > predicted && diff <= 3u) {
            ++one_over;
        }
    }
    bench.verdict("every sampling time and every resolution converts in "
                  "EXACTLY 15.3.9's tSMPL + tSAR - the chapter's arithmetic "
                  "measured against a clock that is exactly twice fADC",
                  rows_done == 6u && exact + one_over == 6u && exact >= 4u);

    // ES0548 2.6.4 wants a SINGLE conversion, or the first of a sequence:
    // in continuous mode above, the extra cycle (if it is there at all)
    // is amortized over 256 and invisible. Here one conversion is timed
    // on its own, against the same conversion at a long sampling time
    // where the erratum does not apply, so the software overhead cancels.
    AdcConfig one = cfg_pad;
    one.sample1 = AdcSampleTime::cycles1_5;
    (void)apply(one);
    (void)Adc::select_channel(0);
    uint32_t best_short = 0xFFFFFFFFUL;
    for (uint8_t i = 0; i < 32; ++i) {
        const uint32_t t0 = cycles_now();
        (void)Adc::read();
        const uint32_t d = cycles_now() - t0;
        if (d < best_short) best_short = d;
    }
    one.sample1 = AdcSampleTime::cycles7_5;
    (void)apply(one);
    (void)Adc::select_channel(0);
    uint32_t best_long = 0xFFFFFFFFUL;
    for (uint8_t i = 0; i < 32; ++i) {
        const uint32_t t0 = cycles_now();
        (void)Adc::read();
        const uint32_t d = cycles_now() - t0;
        if (d < best_long) best_long = d;
    }
    // 7.5 - 1.5 = 6 ADC cycles = 12 CPU cycles, IF neither pays an extra.
    const uint32_t step = best_long - best_short;
    print(serial, "  single conversion: 1.5 cycles ", best_short,
          " CPU cycles, 7.5 cycles ", best_long, ", difference ", step,
          " against 12 predicted (ES0548 2.6.4 would make it 14)", crlf);
    bench.verdict("ES0548 2.6.4 MEASURED: a single conversion at 1.5 sampling "
                  "cycles is timed against one at 7.5, where the erratum does "
                  "not apply - the difference says whether the short one paid "
                  "the extra cycle", step >= 10u && step <= 16u);
    quiet_everything();
}

// =============================================================================
// d - the zero-length wire: the DAC read back through the ADC on ONE pad
// =============================================================================
//
// PA4 is DAC1_OUT1 and ADC_IN4 (DS13560 table 12), so the two converters
// meet on a bond pad with nothing between them. THIS IS THE ONLY ROUTE
// BETWEEN THEM ON THIS FAMILY: figure 36 lists nineteen ADC inputs and
// the DAC is not among them, so the DAC's "connected to on-chip
// peripherals" mode (16.7.16) reaches the comparators and not this
// converter. What is measured here is therefore a DAC, a pad and an ADC
// in series, and the nonlinearity that comes out is the PAIR's - it is
// reported, not apportioned.

/// Set a code, let the buffer settle, and read the pad.
uint16_t dac_then_adc(uint8_t ch, uint16_t code, uint8_t adc_channel) {
    (void)Dac::write(ch, code);
    (void)delay_us(clock, 60);   // DS13560's tSETTLING is 1.7 us typ; 60 is generous
    return convert_median(adc_channel);
}

void td_zero_length_wire() {
    if (!analog_up(cfg_pad)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    const uint16_t vdda = measure_vdda();
    (void)apply(cfg_pad);
    Dac::claim_pad<PadA4>();
    const bool dac_ok = Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered}) &&
                        Dac::enable(0, true);
    (void)delay_us(clock, 100);

    bench.verdict("the DAC's channel 1 is configured, enabled and driving its "
                  "own pad", dac_ok && Dac::enabled(0));

    // DHR is not DOR (16.4.5): the write lands in the holding register
    // and reaches the output one dac_pclk cycle later with no trigger.
    (void)Dac::write(0, 1234);
    (void)delay_us(clock, 10);
    print(serial, "  after write(1234): DHR reads ", Dac::code(0), ", DOR reads ",
          Dac::output(0), crlf);
    bench.verdict("DAC_DHR and DAC_DOR are two registers and the transfer is "
                  "real: with no trigger the holding register reaches the "
                  "output by itself (16.4.5)",
                  Dac::code(0) == 1234u && Dac::output(0) == 1234u);

    // The sweep. Sixteen points across the range, read back through the
    // pad, with the monotonicity and the straightness judged separately.
    constexpr uint8_t points = 17;
    uint16_t code[points];
    uint16_t got[points];
    bool monotonic = true;
    for (uint8_t i = 0; i < points; ++i) {
        code[i] = static_cast<uint16_t>(i == points - 1u ? 4095u : i * 256u);
        got[i] = dac_then_adc(0, code[i], In4::channel);
        if (i > 0 && got[i] < got[i - 1]) {
            monotonic = false;
        }
    }
    console_drain();
    print(serial, "  DAC / pad / ADC:");
    for (uint8_t i = 0; i < points; i += 2) {
        print(serial, " ", code[i], "/", got[i]);
    }
    print(serial, crlf);

    // The best-fit line through the two ends, and the worst residual.
    const int32_t x0 = code[1], y0 = got[1];      // 256, off the buffer's floor
    const int32_t x1 = code[points - 2], y1 = got[points - 2];   // 3840, off its ceiling
    uint16_t worst = 0;
    for (uint8_t i = 1; i < points - 1u; ++i) {
        const int32_t fit = y0 + ((static_cast<int32_t>(code[i]) - x0) * (y1 - y0)) /
                                     (x1 - x0);
        const int32_t r = static_cast<int32_t>(got[i]) - fit;
        const uint16_t a = static_cast<uint16_t>(r < 0 ? -r : r);
        if (a > worst) {
            worst = a;
        }
    }
    print(serial, "  worst residual from the end-to-end line: ", worst,
          " counts of 4096 (the two converters' COMBINED nonlinearity, "
          "deliberately not apportioned)", crlf);
    bench.verdict("the transfer curve is MONOTONIC across the whole range",
                  monotonic);
    bench.verdict("and straight to better than 1 % of full scale - which is "
                  "a DAC, a bond pad and an ADC in series, reported as one "
                  "number because nothing here can split them", worst < 41u);

    // The buffer's own swing, measured: a buffered output cannot reach
    // either rail, and turning the buffer OFF is what says by how much.
    const uint16_t floor_buf = dac_then_adc(0, 0, In4::channel);
    const uint16_t ceil_buf = dac_then_adc(0, 4095, In4::channel);
    (void)Dac::enable(0, false);
    (void)Dac::configure(0, {.mode = DacMode::pin_unbuffered});
    (void)Dac::enable(0, true);
    (void)delay_us(clock, 200);
    const uint16_t floor_raw = dac_then_adc(0, 0, In4::channel);
    const uint16_t ceil_raw = dac_then_adc(0, 4095, In4::channel);
    print(serial, "  code 0 / code 4095: buffered ",
          adc_mv(floor_buf, 4096, vdda), " / ", adc_mv(ceil_buf, 4096, vdda),
          " mV, unbuffered ", adc_mv(floor_raw, 4096, vdda), " / ",
          adc_mv(ceil_raw, 4096, vdda), " mV (VDDA ", vdda, " mV)", crlf);
    bench.verdict("THE OUTPUT BUFFER CANNOT REACH THE RAILS and the "
                  "unbuffered mode can: turning it off moves code 0 DOWN and "
                  "code 4095 UP, which is the swing limit measured rather "
                  "than quoted", floor_raw < floor_buf && ceil_raw > ceil_buf);
    bench.verdict("and an unbuffered output really does span nearly the whole "
                  "reference, so the pad and the ADC are not what is limiting "
                  "it", floor_raw < 60u && ceil_raw > 4030u);

    // The three data formats, all reaching the same output.
    (void)Dac::enable(0, false);
    (void)Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::enable(0, true);
    (void)Dac::write(0, 2048);
    (void)delay_us(clock, 20);
    const uint16_t by12 = Dac::output(0);
    (void)Dac::write_left(0, 0x8000);
    (void)delay_us(clock, 20);
    const uint16_t by12l = Dac::output(0);
    (void)Dac::write8(0, 128);
    (void)delay_us(clock, 20);
    const uint16_t by8 = Dac::output(0);
    print(serial, "  half scale by 12R=", by12, " 12L=", by12l, " 8R=", by8, crlf);
    bench.verdict("the three data formats are PLACEMENTS of the same 12-bit "
                  "datum (16.4.4): 12-bit right, 12-bit left and 8-bit right "
                  "all put half scale in DOR", by12 == 2048u && by12l == 2048u &&
                                                   by8 == 2048u);
    quiet_everything();
}

// =============================================================================
// e - the LED as an analog dimmer, and what its load does to the buffer
// =============================================================================
//
// PA5 is DAC1_OUT2 AND LD4 (16.3, and the board). A DAC-driven LED is a
// brightness with no PWM in it at all - and because PA5 is also ADC_IN5,
// the pad says what the LED is doing to the buffer that drives it. The
// comparison against PA4, which carries no load, is what turns that into
// a measurement.
void te_led_dimmer() {
    if (!analog_up(cfg_pad)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    const uint16_t vdda = measure_vdda();
    (void)apply(cfg_pad);
    Dac::claim_pad<PadA4>();
    Dac::claim_pad<PadA5>();
    const bool up = Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered}) &&
                    Dac::configure(1, {.mode = DacMode::pin_and_internal_buffered}) &&
                    Dac::enable(0, true) && Dac::enable(1, true);
    (void)delay_us(clock, 200);
    bench.verdict("both DAC channels run at once - one on a free pad, one on "
                  "the board's LED", up && Dac::enabled(0) && Dac::enabled(1));

    constexpr uint8_t points = 9;
    uint16_t free_mv[points];
    uint16_t led_mv[points];
    uint16_t worst_droop = 0;
    uint16_t droop_at = 0;
    for (uint8_t i = 0; i < points; ++i) {
        const uint16_t c = static_cast<uint16_t>(i * 512u > 4095u ? 4095u : i * 512u);
        free_mv[i] = adc_mv(dac_then_adc(0, c, In4::channel), 4096, vdda);
        led_mv[i] = adc_mv(dac_then_adc(1, c, In5::channel), 4096, vdda);
        const uint16_t d = free_mv[i] > led_mv[i] ? free_mv[i] - led_mv[i] : 0u;
        if (d > worst_droop) {
            worst_droop = d;
            droop_at = c;
        }
    }
    console_drain();
    print(serial, "  code: free pad mV / LED pad mV");
    for (uint8_t i = 0; i < points; ++i) {
        print(serial, " ", i * 512u, ":", free_mv[i], "/", led_mv[i]);
    }
    print(serial, crlf, "  worst droop ", worst_droop, " mV at code ", droop_at, crlf);

    bench.verdict("the LED pad tracks the free pad at the BOTTOM of the "
                  "range, where the LED is below its forward voltage and "
                  "draws nothing",
                  static_cast<uint16_t>(free_mv[1] > led_mv[1]
                                            ? free_mv[1] - led_mv[1]
                                            : led_mv[1] - free_mv[1]) < 60u);
    bench.verdict("AND THE TWO CHANNELS AGREE ALL THE WAY UP: the pad that "
                  "carries LD4 tracks the free one to within a handful of "
                  "millivolts at every code, so whatever LD4's drive path is "
                  "on this board it is NOT a load the DAC's buffer can feel - "
                  "measured, where the obvious guess was a diode drooping the "
                  "output", worst_droop < 20u);
    bench.verdict("the dimmer works over the whole range: the LED pad is "
                  "monotonic in the code, which is what a brightness knob "
                  "needs", led_mv[8] > led_mv[4] && led_mv[4] > led_mv[1]);

    // Leave the LED at a visible level for a moment, then dark: the one
    // thing in this suite a human can see.
    for (uint16_t c = 0; c <= 4000u; c = static_cast<uint16_t>(c + 200u)) {
        (void)Dac::write(1, c);
        (void)delay_us(clock, 20000);
    }
    (void)Dac::write(1, 0);
    quiet_everything();
}

// =============================================================================
// f - the sequencer, both faces, and the CCRDY handshake
// =============================================================================
//
// THREE KNOWN-DISTINCT INPUTS AND NOT ONE OF THEM IS A DRIVEN PAD, which
// is a fact about this family: 7.3.13 says the weak pull-up and pull-down
// are DISABLED BY HARDWARE in analog mode, so the pull-walked pad that
// carried the SAM's analog letters does not exist here and a pad the CPU
// drives is not connected to the converter at all. The DAC is the only
// analog source inside this chip, so the sequence walks the DAC's two
// channels and VREFINT.
void tf_sequencer() {
    if (!analog_up(cfg_internal)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    Adc::vrefint(true);
    Dac::claim_pad<PadA4>();
    Dac::claim_pad<PadA5>();
    (void)Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::configure(1, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::enable(0, true);
    (void)Dac::enable(1, true);
    (void)Dac::write(0, 1024);   // a quarter of the reference on PA4 (ch 4)
    (void)Dac::write(1, 3072);   // three quarters on PA5 (ch 5)
    (void)delay_us(clock, 500);

    // First, the pull that is not there.
    PadA0::analog();
    PadA0::pull(PinPull::up);
    (void)delay_us(clock, 500);
    const uint16_t floating_up = convert_median(In0::channel);
    PadA0::pull(PinPull::down);
    (void)delay_us(clock, 500);
    const uint16_t floating_down = convert_median(In0::channel);
    PadA0::pull(PinPull::none);
    print(serial, "  an ANALOG pad with its pull-up asked for reads ",
          floating_up, " counts, with the pull-down ", floating_down, crlf);
    bench.verdict("7.3.13 IS LITERAL: the weak pulls are disabled by hardware "
                  "in analog mode, so a pad asked to pull up does NOT read "
                  "full scale - the SAM's pull-walked analog stimulus has no "
                  "twin here", floating_up < 3800u);

    // The bitmap face, forward and backward.
    const uint32_t mask = (1u << In4::channel) | (1u << In5::channel) |
                          (1u << Adc::vrefint_channel);
    AdcConfig fwd = cfg_internal;
    fwd.sample1 = AdcSampleTime::cycles160_5;
    (void)apply(fwd);
    const bool seq_ok = Adc::sequence(mask);
    uint16_t f[3] = {0, 0, 0};
    Adc::start();
    for (uint8_t i = 0; i < 3; ++i) {
        uint32_t guard = 0;
        while (!Adc::ready() && ++guard < 1'000'000UL) {
        }
        f[i] = Adc::result();
    }
    const bool eos_fwd = Adc::sequence_done();
    Adc::clear_flags(AdcFlag::sequence_done);

    AdcConfig bwd = fwd;
    bwd.scan = AdcScanDir::backward;
    (void)apply(bwd);
    (void)Adc::sequence(mask);
    uint16_t b[3] = {0, 0, 0};
    Adc::start();
    for (uint8_t i = 0; i < 3; ++i) {
        uint32_t guard = 0;
        while (!Adc::ready() && ++guard < 1'000'000UL) {
        }
        b[i] = Adc::result();
    }
    print(serial, "  forward ", f[0], " ", f[1], " ", f[2], " | backward ",
          b[0], " ", b[1], " ", b[2], " (ch4 quarter, ch5 three quarters, "
          "ch13 VREFINT)", crlf);
    bench.verdict("the bitmap sequencer scans in NUMERIC ORDER and SCANDIR "
                  "reverses it exactly - the same three values come back "
                  "front to back",
                  seq_ok && f[0] > 800u && f[0] < 1300u && f[1] > 2700u &&
                      f[1] < 3400u && f[2] < 2000u && b[0] < 2000u &&
                      b[1] > 2700u && b[1] < 3400u && b[2] > 800u && b[2] < 1300u);
    bench.verdict("and EOS rises at the end of the sequence and not before",
                  eos_fwd);

    // The ordered face: the same three channels in an order the numbers
    // do not give.
    AdcConfig ord = fwd;
    ord.ordered_sequence = true;
    (void)apply(ord);
    const uint8_t order[3] = {Adc::vrefint_channel, In5::channel, In4::channel};
    const bool ord_ok = Adc::sequence_ordered(order, 3);
    uint16_t o[3] = {0, 0, 0};
    Adc::start();
    for (uint8_t i = 0; i < 3; ++i) {
        uint32_t guard = 0;
        while (!Adc::ready() && ++guard < 1'000'000UL) {
        }
        o[i] = Adc::result();
    }
    print(serial, "  ordered (13, 5, 4): ", o[0], " ", o[1], " ", o[2],
          " CHSELR=", hex(Adc::selection()), crlf);
    bench.verdict("the FULLY CONFIGURABLE face walks the channels in the "
                  "order written and not in their numeric order (15.3.8), "
                  "with 0xF terminating a short list",
                  ord_ok && o[0] < 2000u && o[1] > 2700u && o[2] > 800u &&
                      o[2] < 1300u && (Adc::selection() & 0xFFFF000u) != 0u);

    // The handshake. 15.3.8: a CHSELR write is not in force until CCRDY
    // rises, and 15.12.5 says an ADSTART written before it is IGNORED.
    Adc::clear_flags(AdcFlag::channels_ready);
    const bool before = Adc::flag(AdcFlag::channels_ready);
    const bool sel = Adc::select_sync(AdcInput::vrefint);
    const bool after = Adc::flag(AdcFlag::channels_ready);
    bench.verdict("CCRDY is cleared, CHSELR is written, CCRDY comes back - "
                  "the handshake every channel verb in this driver spends "
                  "(15.3.8's 'it is mandatory to wait')",
                  !before && sel && after);
    AdcConfig busy = fwd;
    busy.continuous = true;
    busy.overrun_overwrite = true;
    (void)apply(busy);
    (void)Adc::select_channel(Adc::vrefint_channel);
    Adc::start();
    const bool running = Adc::converting();
    const bool refused = !Adc::select_channel(In4::channel);
    (void)Adc::stop();
    bench.verdict("and a channel selection is refused while a conversion is "
                  "running (15.3.8's last sentence), which is the rule that "
                  "makes the handshake mean anything", running && refused);
    quiet_everything();
}

// =============================================================================
// g - the noise floor MEASURED, and only then the oversampler
// =============================================================================
struct Spread {
    uint16_t lo;
    uint16_t hi;
    uint32_t mean;
};

uint32_t base_mean_delta(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }
uint16_t mv_delta(uint16_t a, uint16_t b) {
    return static_cast<uint16_t>(a > b ? a - b : b - a);
}

Spread spread_of(uint8_t channel, uint8_t n) {
    (void)Adc::select_channel(channel);
    uint16_t lo = 0xFFFFu, hi = 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; ++i) {
        const uint16_t v = Adc::read();
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    return {lo, hi, sum / n};
}

void tg_oversampler() {
    if (!analog_up(cfg_internal)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    Adc::vrefint(true);
    Dac::claim_pad<PadA4>();
    (void)Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::enable(0, true);
    (void)Dac::write(0, 2048);
    (void)delay_us(clock, 500);
    console_drain();

    const Spread ref = spread_of(Adc::vrefint_channel, 64);
    const Spread pad = spread_of(In4::channel, 64);
    print(serial, "  noise floor over 64 conversions: VREFINT spans ",
          ref.hi - ref.lo, " counts around ", ref.mean, ", the DAC-driven pad ",
          pad.hi - pad.lo, " around ", pad.mean, crlf);
    bench.verdict("THE NOISE IS MEASURED BEFORE ANYTHING IS CLAIMED ABOUT IT: "
                  "both sources are quiet enough to be a converter's own "
                  "noise and not a broken connection",
                  ref.hi >= ref.lo && pad.hi >= pad.lo &&
                      static_cast<uint16_t>(ref.hi - ref.lo) < 60u &&
                      static_cast<uint16_t>(pad.hi - pad.lo) < 60u);

    // The noisier of the two is the measurand; the oversampler is judged
    // against it and only if there is something to reduce.
    const uint8_t noisy = (pad.hi - pad.lo) >= (ref.hi - ref.lo) ? In4::channel
                                                                 : Adc::vrefint_channel;
    const uint16_t base_span = (pad.hi - pad.lo) >= (ref.hi - ref.lo)
                                   ? static_cast<uint16_t>(pad.hi - pad.lo)
                                   : static_cast<uint16_t>(ref.hi - ref.lo);

    AdcConfig ovs = cfg_internal;
    ovs.oversampling = true;
    ovs.oversampling_ratio = AdcOversampling::x16;
    ovs.oversampling_shift = 4;   // back on the 12-bit scale (table 79)
    (void)apply(ovs);
    const Spread avg = spread_of(noisy, 64);
    print(serial, "  x16 oversampling, shift 4: spans ", avg.hi - avg.lo,
          " counts around ", avg.mean, " (full scale ", Adc::result_steps(), ")", crlf);
    bench.verdict("16x oversampling with a 4-bit shift is back on the 12-bit "
                  "full scale, and the MEAN does not move - table 79's row, "
                  "measured",
                  Adc::result_steps() == 4096u &&
                      base_mean_delta(avg.mean, noisy == In4::channel ? pad.mean
                                                                       : ref.mean) < 20u);
    if (base_span >= 4u) {
        bench.verdict("and the spread SHRINKS - which is claimed only because "
                      "the noise floor above was big enough to have something "
                      "to reduce",
                      static_cast<uint16_t>(avg.hi - avg.lo) < base_span);
    } else {
        print(serial, "  the base spread is ", base_span, " counts, too small "
              "to judge a reduction - DECLINED in print rather than dressed up",
              crlf);
        bench.verdict("the reduction verdict is DECLINED, and the declining is "
                      "itself the check: the measured floor really is under "
                      "four counts", base_span < 4u);
    }

    // The other half of table 79: no shift at all widens the datum.
    ovs.oversampling_shift = 0;
    (void)apply(ovs);
    const Spread raw = spread_of(noisy, 16);
    print(serial, "  x16 oversampling, no shift: mean ", raw.mean,
          " (full scale ", Adc::result_steps(), ")", crlf);
    bench.verdict("with no shift the accumulator itself lands in ADC_DR: the "
                  "full scale is sixteen times as wide and the reading is "
                  "sixteen times as big, which is exactly what util/analog.hpp "
                  "must be told through result_steps()",
                  Adc::result_steps() == 65536u && raw.mean > avg.mean * 15u &&
                      raw.mean < avg.mean * 17u);
    bench.verdict("and util/analog.hpp converts BOTH scales to the same "
                  "voltage, because the arithmetic takes the full scale as an "
                  "argument rather than assuming twelve bits",
                  mv_delta(adc_mv(raw.mean, 65536, 3300),
                           adc_mv(avg.mean, 4096, 3300)) <= 2u);
    quiet_everything();
}

// =============================================================================
// h - the three analog watchdogs
// =============================================================================
/// A watchdog's CHANNEL selection costs an enable cycle on this
/// converter (15.3.7 and 15.12.13), so the cycle is spent in one place.
void arm_watchdog1(uint16_t low, uint16_t high, bool single, uint8_t channel) {
    (void)Adc::disable();
    (void)Adc::watchdog1(low, high, single, channel);
    (void)Adc::enable();
}
void arm_watchdog2(uint32_t mask, uint16_t low, uint16_t high) {
    (void)Adc::disable();
    (void)Adc::watchdog2(mask, low, high);
    (void)Adc::enable();
}

void th_watchdogs() {
    if (!analog_up(cfg_internal)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    Adc::vrefint(true);
    Dac::claim_pad<PadA4>();
    (void)Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::enable(0, true);
    (void)Dac::write(0, 2048);
    (void)delay_us(clock, 500);

    const uint16_t v = convert_median(Adc::vrefint_channel);
    const uint16_t d = convert_median(In4::channel);
    print(serial, "  VREFINT ", v, " counts, the DAC's pad ", d, crlf);

    // EVERY WATCHDOG VERB IS A DISABLED-STATE VERB on this converter -
    // 15.3.7 for CFGR1's AWD1 bits, 15.12.13's own note for AWD2CR and
    // AWD3CR - so the enable cycle is spent around each of them.
    const bool refuse_enabled = Adc::enabled() && !Adc::watchdog1(0, 100, false) &&
                                !Adc::watchdog2(1u << 4, 0, 100) &&
                                !Adc::watchdog3(1u << 4, 0, 100);
    bench.verdict("a watchdog cannot be configured while the converter is "
                  "enabled, and the driver REFUSES rather than storing into a "
                  "register the silicon ignores", refuse_enabled);

    // AWD1, all channels, with the reading INSIDE the window.
    Adc::clear_flags(AdcFlag::watchdog1 | AdcFlag::watchdog2 | AdcFlag::watchdog3);
    arm_watchdog1(static_cast<uint16_t>(v > 200u ? v - 200u : 0u),
                  static_cast<uint16_t>(v + 200u), false, 0);
    (void)convert(Adc::vrefint_channel);
    const bool inside = Adc::flag(AdcFlag::watchdog1);
    // ...and then with it OUTSIDE, from above and from below.
    Adc::clear_flags(AdcFlag::watchdog1);
    arm_watchdog1(0, static_cast<uint16_t>(v > 200u ? v - 200u : 0u), false, 0);
    (void)convert(Adc::vrefint_channel);
    const bool above = Adc::flag(AdcFlag::watchdog1);
    Adc::clear_flags(AdcFlag::watchdog1);
    arm_watchdog1(static_cast<uint16_t>(v + 200u), 4095, false, 0);
    (void)convert(Adc::vrefint_channel);
    const bool below = Adc::flag(AdcFlag::watchdog1);
    Adc::clear_flags(AdcFlag::watchdog1);
    bench.verdict("AWD1 guards a WINDOW and both sides of it: a reading "
                  "inside raises nothing, one above the high threshold and "
                  "one below the low threshold both raise the flag (15.7.1)",
                  !inside && above && below);

    // AWD1 on a SINGLE channel: the guarded one flags, the other does not.
    arm_watchdog1(0, static_cast<uint16_t>(v > 200u ? v - 200u : 0u), true,
                  Adc::vrefint_channel);
    (void)convert(Adc::vrefint_channel);
    const bool single_hit = Adc::flag(AdcFlag::watchdog1);
    Adc::clear_flags(AdcFlag::watchdog1);
    (void)convert(In4::channel);
    const bool single_miss = Adc::flag(AdcFlag::watchdog1);
    Adc::clear_flags(AdcFlag::watchdog1);
    bench.verdict("and AWD1SGL narrows it to one channel: the guarded "
                  "conversion flags and an unguarded one on the same "
                  "watchdog does not (table 78)", single_hit && !single_miss);
    (void)Adc::disable();
    (void)Adc::watchdog1_off();
    (void)Adc::enable();

    // AWD2 and AWD3: the MASK is the enable (15.7.2).
    Adc::clear_flags(AdcFlag::watchdog2 | AdcFlag::watchdog3);
    arm_watchdog2(1u << In4::channel, 0,
                  static_cast<uint16_t>(d > 200u ? d - 200u : 0u));
    (void)convert(In4::channel);
    const bool awd2_hit = Adc::flag(AdcFlag::watchdog2);
    Adc::clear_flags(AdcFlag::watchdog2);
    (void)convert(Adc::vrefint_channel);
    const bool awd2_other = Adc::flag(AdcFlag::watchdog2);
    Adc::clear_flags(AdcFlag::watchdog2);
    arm_watchdog2(0, 0, 0);
    (void)convert(In4::channel);
    const bool awd2_off = Adc::flag(AdcFlag::watchdog2);
    bench.verdict("AWD2's CHANNEL MASK IS ITS ENABLE - a bit set guards that "
                  "channel, a mask of zero turns the watchdog off, and there "
                  "is no separate bit to forget (15.7.2)",
                  awd2_hit && !awd2_other && !awd2_off);

    (void)Adc::disable();
    (void)Adc::watchdog3(1u << Adc::vrefint_channel,
                         static_cast<uint16_t>(v + 200u), 4095);
    (void)Adc::enable();
    Adc::clear_flags(AdcFlag::watchdog3);
    (void)convert(Adc::vrefint_channel);
    const bool awd3_hit = Adc::flag(AdcFlag::watchdog3);
    (void)Adc::disable();
    (void)Adc::watchdog3(0, 0, 0);
    (void)Adc::enable();
    Adc::clear_flags(AdcFlag::watchdog3);
    bench.verdict("and the third watchdog is the second one again, at its own "
                  "thresholds and over its own mask", awd3_hit);

    // THE SAME SENTENCE, TWO BEHAVIOURS - staged through the register
    // itself, because the driver refuses to make the mistake. 15.3.7 and
    // 15.12.13 both say ADEN must be clear, and the silicon disagrees
    // with itself: a CFGR1 write with the converter enabled LANDS (this
    // is also what ES0548 2.6.2 is about), while an AWD2CR write with the
    // converter enabled is dropped in silence.
    arm_watchdog2(0, 0, 0);
    const uint32_t cfgr1_before = Adc::regs().CFGR1;
    Adc::regs().CFGR1 = cfgr1_before | ADC_CFGR1_AWD1EN;
    const bool cfgr1_landed = (Adc::regs().CFGR1 & ADC_CFGR1_AWD1EN) != 0u;
    Adc::regs().CFGR1 = cfgr1_before;
    Adc::regs().AWD2CR = 1u << In4::channel;
    const bool awd2cr_landed = Adc::regs().AWD2CR != 0u;
    Adc::regs().AWD2CR = 0;
    print(serial, "  written with ADEN set: CFGR1's AWD1EN landed ",
          cfgr1_landed, ", AWD2CR landed ", awd2cr_landed, crlf);
    bench.verdict("A FORBIDDEN WRITE IS NOT ONE THING ON THIS CONVERTER: "
                  "CFGR1 takes an AWD1 bit with ADEN set (which is exactly "
                  "the door ES0548 2.6.2 comes through) while AWD2CR ignores "
                  "the same forbidden write in complete silence - two "
                  "behaviours behind one sentence, which is why every "
                  "watchdog verb here refuses instead", cfgr1_landed && !awd2cr_landed);

    // The thresholds ALONE are live: 15.7.4 says so, and it is the half
    // of a watchdog a control loop needs to move without stopping.
    (void)Adc::disable();
    (void)Adc::watchdog1(static_cast<uint16_t>(v + 200u), 4095, false);
    (void)Adc::enable();
    Adc::clear_flags(AdcFlag::watchdog1);
    (void)convert(Adc::vrefint_channel);
    const bool tight = Adc::flag(AdcFlag::watchdog1);
    const bool live = Adc::watchdog_thresholds(1, static_cast<uint16_t>(v > 200u ? v - 200u : 0u),
                                               static_cast<uint16_t>(v + 200u));
    Adc::clear_flags(AdcFlag::watchdog1);
    (void)convert(Adc::vrefint_channel);
    (void)convert(Adc::vrefint_channel);
    const bool loose = Adc::flag(AdcFlag::watchdog1);
    bench.verdict("but the THRESHOLDS are live (15.7.4): moving the window "
                  "around the reading while the converter runs stops the "
                  "flag, with no enable cycle anywhere",
                  tight && live && !loose);
    (void)Adc::disable();
    (void)Adc::watchdog1_off();
    (void)Adc::enable();

    // The interrupt, through the vector the comparators share.
    clear_counts();
    Adc::interrupts(AdcFlag::watchdog1, true);
    Nvic::enable(Adc::irq());
    arm_watchdog1(static_cast<uint16_t>(v + 200u), 4095, false, 0);
    for (uint8_t i = 0; i < 4; ++i) {
        (void)convert(Adc::vrefint_channel);
    }
    Nvic::disable(Adc::irq());
    Adc::interrupts(AdcFlag::watchdog1, false);
    (void)Adc::disable();
    (void)Adc::watchdog1_off();
    (void)Adc::enable();
    print(serial, "  watchdog interrupts: ", adc_awd_calls, crlf);
    bench.verdict("the watchdog reaches the NVIC through the vector it shares "
                  "with the three comparators, once per guarded conversion",
                  adc_awd_calls == 4u);

    // ES0548 2.6.3 staged: AWD1 in SINGLE mode on a channel that is not
    // the first of a sequence.
    AdcConfig seq = cfg_internal;
    (void)apply(seq);
    (void)Adc::sequence((1u << In4::channel) | (1u << Adc::vrefint_channel));
    arm_watchdog1(0, static_cast<uint16_t>(v > 200u ? v - 200u : 0u), true,
                  Adc::vrefint_channel);   // channel 13: the SECOND of the two
    Adc::clear_flags(AdcFlag::watchdog1);
    Adc::start();
    uint32_t guard = 0;
    while (!Adc::sequence_done() && ++guard < 2'000'000UL) {
    }
    const bool second_flagged = Adc::flag(AdcFlag::watchdog1);
    Adc::clear_flags(AdcFlag::watchdog1 | AdcFlag::sequence_done);
    // The control: the SAME watchdog on the FIRST channel of the same
    // sequence, whose value is equally far outside its own window.
    arm_watchdog1(0, static_cast<uint16_t>(d > 200u ? d - 200u : 0u), true,
                  In4::channel);
    Adc::start();
    guard = 0;
    while (!Adc::sequence_done() && ++guard < 2'000'000UL) {
    }
    const bool first_flagged = Adc::flag(AdcFlag::watchdog1);
    Adc::clear_flags(AdcFlag::watchdog1 | AdcFlag::sequence_done);
    (void)Adc::disable();
    (void)Adc::watchdog1_off();
    (void)Adc::enable();
    print(serial, "  ES0548 2.6.3 staged: guarded SECOND channel flagged ",
          second_flagged, ", guarded FIRST channel (the control) flagged ",
          first_flagged, crlf);
    bench.verdict("ES0548 2.6.3 STAGED WITH A CONTROL: the erratum says an "
                  "AWD1 single-channel watchdog misses a channel that is not "
                  "the first of a sequence, and the control proves the "
                  "instrument sensitive", first_flagged);
    quiet_everything();
}

// =============================================================================
// i - the comparators: what CAN be measured with no wire, and what cannot
// =============================================================================
//
// THE STIMULUS PROBLEM, stated before it is worked around. A comparator's
// NON-inverting input is a pad and only a pad (tables 93/95/97 have no
// internal signal in them), and 7.3.13 disconnects the pad's own pull the
// moment it goes analog - so there is no way to hold that input at a
// chosen voltage without a wire. What CAN be done is to PRECHARGE it: the
// pad is driven to a rail by GPIO, then handed to the comparator, and the
// node's own capacitance keeps it there for as long as its leakage
// allows. That is enough for every logical question this chapter asks -
// the multiplexers, the polarity, the window, the blanking, the EXTI line
// and the timer input - and it is NOT enough for a threshold sweep, which
// is declined at the end of this letter with the wire that would settle it
// named.
void precharge(bool high) {
    PadA1::output(high);
    (void)delay_us(clock, 300);
    PadA1::analog();
}

void ti_comparators() {
    quiet_everything();
    clear_counts();
    C1::init();

    print(serial, "  COMP1 INP: 0=P", static_cast<char>(C1::positive_pin(CompPositive::input0).port),
          C1::positive_pin(CompPositive::input0).pin, " 1=P",
          static_cast<char>(C1::positive_pin(CompPositive::input1).port),
          C1::positive_pin(CompPositive::input1).pin, " 2=P",
          static_cast<char>(C1::positive_pin(CompPositive::input2).port),
          C1::positive_pin(CompPositive::input2).pin,
          "; COMP1 INM8=P", static_cast<char>(C1::negative_pin(CompNegative::input8).port),
          C1::negative_pin(CompNegative::input8).pin, crlf);
    bench.verdict("the input tables are per instance and are NOT a pattern - "
                  "COMP1's plus inputs are PC5/PB2/PA1 and COMP2's PB4/PB6/PA3 "
                  "(tables 93 and 95)",
                  C1::positive_pin(CompPositive::input2).port == 'A' &&
                      C1::positive_pin(CompPositive::input2).pin == 1 &&
                      C2::positive_pin(CompPositive::input2).port == 'A' &&
                      C2::positive_pin(CompPositive::input2).pin == 3);

    constexpr CompConfig base{.positive = CompPositive::input2,      // PA1
                              .negative = CompNegative::vrefint_half};
    const bool up = C1::claim_inputs(base) && C1::configure(base) && C1::enable(true);
    (void)delay_us(clock, 100);   // tSTART_SCALER is 200 us max, and the
    (void)delay_us(clock, 300);   // comparator's own 5 us on top
    bench.verdict("COMP1 configures and enables against half of VREFINT, an "
                  "internal threshold that needs no pad at all", up && C1::enabled());

    precharge(true);
    const bool high_value = C1::value();
    precharge(false);
    const bool low_value = C1::value();
    print(serial, "  precharged high gives VALUE ", high_value,
          ", precharged low gives VALUE ", low_value, crlf);
    bench.verdict("THE PRECHARGED PAD IS A REAL STIMULUS: with the plus input "
                  "left at a rail by the GPIO that just released it, VALUE "
                  "says which rail", high_value && !low_value);

    // How long the charge lasts - the number that says whether the
    // technique is a measurement or a race.
    precharge(true);
    const uint32_t t0 = cycles_now();
    uint32_t held = 0;
    const uint32_t cap = SysClock::hz / 10u;   // 100 ms
    while (C1::value() && (held = cycles_now() - t0) < cap) {
    }
    const uint32_t held_us = held / cycles_per_us;
    print(serial, "  the precharged node held above half VREFINT for ",
          held_us, " us", crlf);
    bench.verdict("and it holds for long enough to be read many times over - "
                  "the pad's own capacitance against its leakage, measured "
                  "rather than assumed", held_us > 500u);

    // POLARITY inverts VALUE, which 18.6.1 says sits AFTER the selector.
    precharge(true);
    CompConfig inv = base;
    inv.inverted = true;
    (void)C1::enable(false);
    (void)C1::configure(inv);
    (void)C1::enable(true);
    precharge(true);
    const bool inverted_high = C1::value();
    (void)C1::enable(false);
    (void)C1::configure(base);
    (void)C1::enable(true);
    bench.verdict("POLARITY inverts what VALUE reports, because VALUE is read "
                  "AFTER the polarity selector and the blanking (18.6.1's own "
                  "wording)", !inverted_high);

    // The four VREFINT taps: at a rail they must all agree, and the
    // register must hold what was written.
    bool taps_ok = true;
    const CompNegative taps[4] = {CompNegative::vrefint_quarter, CompNegative::vrefint_half,
                                  CompNegative::vrefint_three_quarters, CompNegative::vrefint};
    for (CompNegative t : taps) {
        CompConfig c = base;
        c.negative = t;
        (void)C1::enable(false);
        (void)C1::configure(c);
        (void)C1::enable(true);
        (void)delay_us(clock, 300);
        precharge(true);
        const bool hi = C1::value();
        precharge(false);
        const bool lo = C1::value();
        if (!hi || lo || C1::negative() != t) {
            taps_ok = false;
        }
    }
    bench.verdict("all four VREFINT taps are selectable and all four sit "
                  "between the rails - each one reads high on a precharged "
                  "3.3 V and low on a precharged ground", taps_ok);

    // Hysteresis and speed: the FIELDS are written and read back; their
    // analog effect is declined, see the end of this letter.
    CompConfig hy = base;
    hy.hysteresis = CompHysteresis::high;
    hy.power = CompPower::medium_speed;
    (void)C1::enable(false);
    const bool hy_ok = C1::configure(hy);
    (void)C1::enable(true);
    (void)delay_us(clock, 300);
    bench.verdict("hysteresis and the power/speed selector are written and "
                  "read back (DS13560 table 68 puts the three hysteresis "
                  "levels at 10, 20 and 30 mV)",
                  hy_ok && C1::hysteresis() == CompHysteresis::high &&
                      C1::power() == CompPower::medium_speed);
    (void)C1::enable(false);
    (void)C1::configure(base);
    (void)C1::enable(true);
    (void)delay_us(clock, 300);

    // BLANKING, measured: TIM1's OC4 held ACTIVE by force, with the
    // comparator's plus input precharged high, must gate VALUE low.
    T1::init();
    (void)T1::configure({.prescaler = 63, .period = 999});
    (void)T1::output_channel(3, {.mode = TimOutputMode::force_inactive, .enable = true});
    (void)T1::main_output(true);
    T1::enable(true);
    CompConfig blank = base;
    blank.blanking = CompBlank::tim1_oc4;
    (void)C1::enable(false);
    (void)C1::configure(blank);
    (void)C1::enable(true);
    (void)delay_us(clock, 300);
    precharge(true);
    // The order matters: the comparator is up and reading HIGH first, and
    // only then does the blanking source rise - so what is watched is an
    // EDGE arriving at a live comparator, not a level that was already
    // there when it was enabled.
    const bool unblanked = C1::value();
    (void)T1::output_channel(3, {.mode = TimOutputMode::force_active, .enable = true});
    (void)delay_us(clock, 200);
    const bool blanked = C1::value();
    (void)T1::output_channel(3, {.mode = TimOutputMode::force_inactive, .enable = true});
    (void)delay_us(clock, 200);
    const bool released = C1::value();
    print(serial, "  blanking from TIM1 OC4: before ", unblanked,
          ", forced active gives VALUE ", blanked, ", forced inactive again gives ",
          released, crlf);
    bench.verdict("THE BLANKING WINDOW IS A REAL GATE, and it is measured "
                  "with no pad and no wire: TIM1's OC4 held active by force "
                  "drives VALUE low while the input says high, and releasing "
                  "it gives the answer back (18.3.7)",
                  unblanked && !blanked && released);
    (void)C1::enable(false);
    (void)C1::configure(base);
    (void)C1::enable(true);
    (void)delay_us(clock, 300);

    // The WINDOW pair, with no second pad: COMP2's WINMODE borrows
    // COMP1's own plus input (18.6.1), so one precharged node feeds two
    // comparators at two thresholds.
    C2::init();
    CompConfig lower{.positive = CompPositive::open,
                     .negative = CompNegative::vrefint_quarter,
                     .window_input = true};
    CompConfig upper = base;
    upper.negative = CompNegative::vrefint_three_quarters;
    (void)C1::enable(false);
    (void)C1::configure(upper);
    (void)C1::enable(true);
    const bool win_ok = C2::configure(lower) && C2::enable(true);
    (void)delay_us(clock, 400);
    precharge(true);
    const bool above_both = C1::value() && C2::value();
    precharge(false);
    const bool below_both = !C1::value() && !C2::value();
    print(serial, "  window pair (COMP1 at 3/4 VREFINT, COMP2 at 1/4 through "
          "WINMODE): above both ", above_both, ", below both ", below_both, crlf);
    bench.verdict("A WINDOW COMPARATOR WITH ONE PAD: COMP2's WINMODE takes "
                  "COMP1's plus input, so one precharged node is compared "
                  "against two thresholds at once (18.3.5)",
                  win_ok && above_both && below_both);
    print(serial, "  the INSIDE state is DECLINED: it needs the node held "
          "between the two thresholds, which no source inside this chip can "
          "do (tables 93/95/97 offer the plus input no internal signal). One "
          "wire from PA4 - DAC1_OUT1 - to PA1 would settle it.", crlf);

    // WINOUT: the XOR of the pair, and figure 69 puts it on the
    // comparator that OWNS the pad - the one with WINMODE CLEAR - while
    // the one borrowing the input keeps WINOUT clear. The first version
    // of this letter set both bits on COMP2 and measured a value that
    // never moved, which is the figure read the wrong way round.
    precharge(true);
    const bool plain_above = C1::value();
    precharge(false);
    const bool plain_below = C1::value();
    CompConfig xo = upper;
    xo.window_output = true;
    (void)C1::enable(false);
    (void)C1::configure(xo);
    (void)C1::enable(true);
    (void)delay_us(clock, 300);
    const uint32_t csr_with = C1::regs().CSR;
    precharge(true);
    const bool xor_above = C1::value();
    const bool c2_above = C2::value();
    precharge(false);
    const bool xor_below = C1::value();
    const bool c2_below = C2::value();
    print(serial, "  COMP1 without WINOUT: high ", plain_above, ", low ",
          plain_below, "; with WINOUT (CSR ", hex(csr_with), "): high ",
          xor_above, ", low ", xor_below, "; COMP2 meanwhile ", c2_above, "/",
          c2_below, crlf);
    // THE DISCRIMINATING HALF. At a rail both comparators agree, so an
    // xor and a plain value differ only if the xor is live - and the
    // reading above says it is not. So the partner is INVERTED, which
    // makes them disagree at BOTH rails: a live xor is then CONSTANT
    // where the bare value follows the node, and the two hypotheses are
    // told apart without an intermediate voltage.
    CompConfig lower_inv = lower;
    lower_inv.inverted = true;
    (void)C2::enable(false);
    (void)C2::configure(lower_inv);
    (void)C2::enable(true);
    (void)delay_us(clock, 400);
    precharge(true);
    const bool dis_above = C1::value();
    const bool c2i_above = C2::value();
    precharge(false);
    const bool dis_below = C1::value();
    const bool c2i_below = C2::value();
    print(serial, "  partner inverted so the pair DISAGREES: COMP1 with "
          "WINOUT reads ", dis_above, "/", dis_below, " where COMP2 reads ",
          c2i_above, "/", c2i_below, crlf);
    bench.verdict("CSR.VALUE IS READ BEFORE THE WINDOW SELECTOR, and this is "
                  "where figure 68 has to be read carefully: with WINOUT set "
                  "COMP1's VALUE bit still follows the node exactly, whether "
                  "the partner agrees with it or - inverted - disagrees at "
                  "both rails. The bit is written and standing; it simply "
                  "does not reach this status bit",
                  plain_above && !plain_below && dis_above == plain_above &&
                      dis_below == plain_below && c2i_above != c2i_below &&
                      (csr_with & COMP_CSR_WINOUT_Msk) != 0u);

    // ...so WHERE does it reach? Figure 68 sends COMPx WINOUT to the
    // pad, the timers and the EXTI line, and VALUE branches off before
    // it. That is testable without any new stimulus: with the partner
    // inverted the xor is CONSTANT, so the EXTI line must stop seeing
    // edges the bare output still shows.
    (void)Exti::sense(C1::exti_line, ExtiSense::both);
    (void)Exti::interrupt(C1::exti_line, true);
    (void)Exti::clear(C1::exti_line);
    Nvic::clear_pending(Adc::irq());
    Nvic::enable(Adc::irq());
    clear_counts();
    for (uint8_t i = 0; i < 4; ++i) {
        precharge((i & 1u) == 0u);
        (void)delay_us(clock, 300);
    }
    const uint32_t edges_xor = comp_exti_calls;
    // The control: the same four rail changes with WINOUT cleared.
    (void)C1::enable(false);
    (void)C1::configure(upper);
    (void)C1::enable(true);
    (void)delay_us(clock, 400);
    (void)Exti::clear(C1::exti_line);
    clear_counts();
    for (uint8_t i = 0; i < 4; ++i) {
        precharge((i & 1u) == 0u);
        (void)delay_us(clock, 300);
    }
    const uint32_t edges_plain = comp_exti_calls;
    Nvic::disable(Adc::irq());
    (void)Exti::release(C1::exti_line);
    print(serial, "  four rail changes with the pair disagreeing: EXTI line "
          "17 fired ", edges_xor, " times with WINOUT set and ", edges_plain,
          " with it clear", crlf);
    bench.verdict("the instrument is SENSITIVE either way: the same four "
                  "rail changes fire the comparator's EXTI line with WINOUT "
                  "set and with it clear, so a silence would have been "
                  "visible", edges_xor >= 3u && edges_plain >= 3u);
    print(serial, "  SO WINOUT CHANGED NOTHING THIS BENCH CAN SEE - not "
          "CSR.VALUE and not the EXTI line - although the bit is written and "
          "stands, the arrangement is figure 69's, both comparators are "
          "enabled on the same node and the partner's polarity was inverted "
          "to make the pair disagree at both rails. RECORDED, NOT EXPLAINED. "
          "What would settle it is a node held BETWEEN the two thresholds, "
          "where the pair disagrees on the SIGNAL and not on a polarity bit - "
          "one wire from PA4 to PA1, the same wire the INSIDE state wants.",
          crlf);
    (void)C2::release();

    // The output where nobody has to look at a pad: EXTI line 17 through
    // the ADC's own vector, and TIM1's TI1 through TISEL.
    (void)C1::enable(false);
    (void)C1::configure(base);
    (void)C1::enable(true);
    (void)delay_us(clock, 300);
    (void)Exti::sense(C1::exti_line, ExtiSense::both);
    (void)Exti::interrupt(C1::exti_line, true);
    (void)Exti::clear(C1::exti_line);
    clear_counts();
    Nvic::clear_pending(Adc::irq());
    Nvic::enable(Adc::irq());

    const bool tisel_ok = T1::input_select(0, 1);   // TI1SEL = COMP1 output
    (void)T1::capture_channel(0, {.polarity = TimCapturePolarity::both});
    T1::clear_flags(T1::compare_flag(0) | T1::overcapture_flag(0));
    T1::set_count(0);

    uint8_t captures = 0;
    for (uint8_t i = 0; i < 6; ++i) {
        precharge((i & 1u) == 0u);
        (void)delay_us(clock, 200);
        if (T1::flag(T1::compare_flag(0))) {
            ++captures;
            (void)T1::compare(0);
        }
    }
    Nvic::disable(Adc::irq());
    (void)Exti::release(C1::exti_line);
    print(serial, "  six rail changes: EXTI line 17 fired ", comp_exti_calls,
          " times (", comp_exti_rising, " rising, ", comp_exti_falling,
          " falling), TIM1's TI1 captured ", captures, crlf);
    bench.verdict("the comparator's output reaches EXTI LINE 17 and the "
                  "ADC's own vector serves it, with the rising and falling "
                  "edges in the two SEPARATE pending registers (18.5)",
                  comp_exti_calls >= 5u && comp_exti_rising >= 2u &&
                      comp_exti_falling >= 2u);
    bench.verdict("and the SAME edges reach TIM1's capture unit through "
                  "TIM1_TISEL (21.4.28) - two mechanisms sharing nothing but "
                  "the comparator agreeing on what happened",
                  tisel_ok && captures >= 5u);

    // The lock: reported, never set.
    print(serial, "  COMP1_CSR=", hex(C1::regs().CSR), " locked=", C1::locked(), crlf);
    bench.verdict("the LOCK bit is clear and stays clear: 18.3.4 makes it "
                  "one-way until the next MCU reset, so this suite reads it "
                  "and never writes it", !C1::locked() && !C2::locked() &&
                                             !C3::locked());
    print(serial, "  COMP3 is present and its registers answer, but its three "
          "plus pads (PB0/PC1/PE7) are left alone on this desk - no verdict "
          "is offered on a comparator this suite never gives an input.", crlf);
    bench.verdict("COMP3 exists on this part and its CSR is reachable through "
                  "the SYSCFG gate, which is what its register block lives "
                  "behind (18.3.3)",
                  comp_present(3) && C3::configure({.negative = CompNegative::vrefint}) &&
                      C3::negative() == CompNegative::vrefint);
    (void)C3::release();
    quiet_everything();
}

// =============================================================================
// j - AnalogSampler inside a REAL KERNEL, walking three inputs
// =============================================================================
//
// util/analog_sampler.hpp's third silicon, and its own comment doubted
// this one: "on a converter with a hardware sequencer and DMA the natural
// delivery is a block per interrupt and the selected input is a count the
// driver keeps". Both halves of that are true here - and the sampler uses
// NEITHER, so the shape survives again with not one line of util/ changed.
// What it costs is stated where it belongs: this converter has no
// current-channel register at all, so Adc::selected() is the driver's own
// memory of the last selection, and it is exact precisely because the
// sampler selects one channel at a time.

struct Collector;
using Subs = Subscribers<Collector>;
using Sampler = AnalogSampler<Adc, Stm32Platform, Subs, AdcInput::vrefint,
                              AdcInput::temperature, In4{}>;

struct Collector {
    using Event = std::variant<AnalogSample>;
    static inline EventQueue<Event, 8, Stm32Platform> queue;

    static inline uint16_t samples = 0;
    static inline uint16_t per_index[3] = {0, 0, 0};
    static inline uint16_t last[3] = {0, 0, 0};

    static void init() {
        samples = 0;
        for (uint8_t i = 0; i < 3; ++i) {
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

using AnalogKernel = Kernel<Stm32Platform, Collector, Sampler>;

void tj_sampler_ao() {
    if (!analog_up(cfg_internal)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    Adc::vrefint(true);
    Adc::temperature(true);
    Dac::claim_pad<PadA4>();
    (void)Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered});
    (void)Dac::enable(0, true);
    (void)Dac::write(0, 3000);
    (void)delay_us(clock, 500);
    console_drain();

    Adc::interrupts(AdcFlag::converted, true);
    Nvic::clear_pending(Adc::irq());
    Nvic::enable(Adc::irq());
    AnalogKernel::init_all();
    kernel_mode = true;
    Sampler::start_every(2);   // one conversion every 2 ms

    // Kernel::step() serves ONE queued event and nothing else - only
    // Kernel::run() matures time events, and this loop is not run(). The
    // sampler's software pace IS a time event, so the pump has to do
    // both halves by hand (the samc suite's own shape).
    const uint32_t deadline = Ticker::ticks() + 400u;
    while (Collector::samples < 60u && Ticker::ticks() < deadline) {
        TimeEvents<Stm32Platform>::process();
        while (AnalogKernel::step()) {
        }
    }
    Sampler::stop();
    for (uint8_t i = 0; i < 8; ++i) {
        AnalogKernel::step();
    }
    kernel_mode = false;
    Nvic::disable(Adc::irq());
    Adc::interrupts(AdcFlag::converted, false);

    const uint16_t vdda = Adc::vdda_mv(Collector::last[0]);
    print(serial, "  ", Collector::samples, " samples: VREFINT ",
          Collector::per_index[0], " (last ", Collector::last[0], " = VDDA ",
          vdda, " mV), TSENSE ", Collector::per_index[1], " (last ",
          Collector::last[1], "), DAC pad ", Collector::per_index[2], " (last ",
          Collector::last[2], "); unknown inputs ", Sampler::unknown_inputs(),
          ", queue overflows ", Collector::queue.overflows(), crlf);

    bench.verdict("AnalogSampler RUNS UNCHANGED ON THE THIRD ARCHITECTURE: "
                  "an active object walking three inputs of a converter with "
                  "a hardware sequencer it does not use", Collector::samples >= 60u);
    bench.verdict("the walk is EVEN - three inputs, three roughly equal "
                  "counts, because the sampler selects the next one on every "
                  "result",
                  Collector::per_index[0] >= 18u && Collector::per_index[1] >= 18u &&
                      Collector::per_index[2] >= 18u);
    bench.verdict("and NOT ONE sample was mislabelled: every result carried "
                  "an input code the list knows, so `unknown_inputs` stayed "
                  "at zero", Sampler::unknown_inputs() == 0u);
    bench.verdict("the values are the right ones for their labels - VREFINT "
                  "gives a plausible VDDA and the DAC's pad reads near the "
                  "three quarters it was set to",
                  vdda >= 3000u && vdda <= 3600u && Collector::last[2] > 2700u &&
                      Collector::last[2] < 3200u);
    quiet_everything();
}

// =============================================================================
// k - the no-CPU chain: ONE trigger driving BOTH converters
// =============================================================================
//
// TIM6's TRGO is dac_ch1_trg5 (table 85) AND the ADC's EXTSEL 101 (table
// 73), so one basic timer paces both converters from the same edge. A
// DmaLoopEngine plays a table into DAC_DHR12R1 and a DmaPingPongEngine
// drains ADC_DR, and the CPU is in neither path. THE ADC READS THE
// PREVIOUS TABLE ENTRY, on purpose: both converters start on the same
// edge and the DAC's output needs three dac_pclk cycles plus its settling
// time, so what is checked is that the captured sequence FOLLOWS the
// played table with a constant phase - which is the only thing a seam
// check can mean when the two ends are one edge apart.

constexpr uint16_t table_len = 16;
constexpr uint16_t block_len = 24;   // deliberately not a divisor of 16
volatile uint16_t dac_table[table_len];
volatile uint16_t pong_a[block_len];
volatile uint16_t pong_b[block_len];

/// Which table entry a reading is nearest to, or 0xFF if it is nearer to
/// nothing - the table's step is 4096/16 = 256 counts and the converter's
/// own noise is a handful, so the nearest entry is never in doubt.
uint8_t nearest_entry(uint16_t value) {
    uint16_t best = 0xFFFFu;
    uint8_t at = 0xFF;
    for (uint8_t i = 0; i < table_len; ++i) {
        const uint16_t t = dac_table[i];
        const uint16_t d = value > t ? value - t : t - value;
        if (d < best) {
            best = d;
            at = i;
        }
    }
    return best <= 90u ? at : 0xFF;
}

void tk_chain() {
    if (!analog_up(cfg_pad)) {
        bench.verdict("the ADC came up", false);
        return;
    }
    Dma<1>::bus_clock(true);
    Dac::claim_pad<PadA4>();
    for (uint16_t i = 0; i < table_len; ++i) {
        dac_table[i] = static_cast<uint16_t>(120u + i * 240u);
    }
    for (uint16_t i = 0; i < block_len; ++i) {
        pong_a[i] = 0;
        pong_b[i] = 0;
    }

    // The pacer: 5 kHz, a rate at which the DAC's buffer has 200 us to
    // settle and the ADC's 1.2 us conversion is over long before the next
    // edge.
    constexpr uint32_t rate = 5'000;
    T6::init();
    const uint32_t div = tim_clock_hz(clock) / rate;
    const bool pacer = T6::configure({.prescaler = 63,
                                      .period = (div / 64u) - 1u}) &&
                       T6::master(TimMasterMode::update);

    // The DAC: triggered by TIM6, fed by the DMA, its first datum in the
    // holding register BEFORE the first trigger (16.4.8).
    const bool dac_ok =
        Dac::configure(0, {.mode = DacMode::pin_and_internal_buffered,
                           .triggered = true,
                           .trigger = DacTrigger::tim6_trgo,
                           .dma = true}) &&
        Dac::enable(0, true);
    (void)Dac::write(0, dac_table[0]);

    // The ADC: the SAME trigger, in circular DMA mode so it keeps
    // requesting past the end of any one block (15.5.5).
    // THE SAMPLING TIME IS THE DELAY. Both converters start on the same
    // edge, and the ADC's sample-and-hold closes at the END of tSMPL - so
    // a sampling window LONGER than the DAC's settling time (DS13560's
    // tSETTLING, 1.7 us typical) holds the value the DAC has just
    // reached rather than one caught mid-slew. At 160.5 cycles of a
    // 32 MHz fADC that window is 5 us. The first version of this letter
    // sampled for 1.2 us and read nine transitional values in six blocks.
    AdcConfig chain = cfg_pad;
    chain.sample1 = AdcSampleTime::cycles160_5;
    chain.trigger = AdcTrigger::tim6_trgo;
    chain.trigger_edge = AdcEdge::rising;
    chain.dma = true;
    chain.dma_circular = true;
    chain.overrun_overwrite = true;
    const bool adc_ok = apply(chain) && Adc::select_sync(In4{});

    clear_counts();
    DacLoop::arm(Dac::data_address_12r(0), Dac::dma_request(0));
    AdcPong::arm(Adc::data_address(), Adc::dma_request);
    Nvic::enable(DMA1_Channel1_IRQn);
    Nvic::enable(DMA1_Channel2_3_IRQn);
    const bool loop_started = DacLoop::start(dac_table, table_len);
    const bool pong_started = AdcPong::start(pong_a, pong_b, block_len);
    Adc::start();          // arms the hardware trigger (15.4)
    T6::enable(true);

    // Six blocks of 24 at 5 kHz is 29 ms. Nothing is printed inside the
    // window: a verdict line is four milliseconds of console and a block
    // is under five, which is the lesson three campaigns on this desk
    // have already paid for.
    // THE FIRST BLOCK IS DISCARDED, and the reason is in the chapter:
    // 16.4.8 makes the caller write the first datum into the holding
    // register BEFORE the first trigger, so the stream's opening entry is
    // the CPU's and not the table's, and the block that carries it is one
    // entry out of phase with every block after it. Measured exactly that
    // way the first time this letter ran (entries 0, 7, 15, 7, 15, 7 -
    // one step of seven among five of eight), which is why the launch
    // block is now spent rather than judged.
    uint8_t first_entry[8];
    uint8_t steps_ok = 0;
    uint8_t blocks = 0;
    uint16_t inside_bad = 0;
    bool launch_spent = false;
    const uint32_t deadline = Ticker::ticks() + 400u;
    while (blocks < 6u && Ticker::ticks() < deadline) {
        volatile uint16_t* ready = AdcPong::ready();
        if (ready == nullptr) {
            continue;
        }
        if (!launch_spent) {
            launch_spent = true;
            (void)AdcPong::release();
            continue;
        }
        uint8_t at = nearest_entry(ready[0]);
        first_entry[blocks] = at;
        if (at != 0xFF) {
            for (uint16_t i = 1; i < block_len; ++i) {
                const uint8_t want = static_cast<uint8_t>((at + i) % table_len);
                if (nearest_entry(ready[i]) != want) {
                    ++inside_bad;
                }
            }
        } else {
            inside_bad = static_cast<uint16_t>(inside_bad + block_len);
        }
        ++blocks;
        (void)AdcPong::release();
    }
    for (uint8_t i = 1; i < blocks; ++i) {
        if (first_entry[i] != 0xFF && first_entry[i - 1] != 0xFF &&
            first_entry[i] == static_cast<uint8_t>((first_entry[i - 1] + block_len) %
                                                   table_len)) {
            ++steps_ok;
        }
    }
    const uint32_t laps = DacLoop::laps();
    const uint32_t overruns = AdcPong::overruns();
    const bool underrun = Dac::underrun(0);
    T6::enable(false);
    (void)Adc::stop();
    DacLoop::stop();
    AdcPong::stop();
    Nvic::disable(DMA1_Channel1_IRQn);
    Nvic::disable(DMA1_Channel2_3_IRQn);

    print(serial, "  ", blocks, " blocks of ", block_len, " at ", rate,
          " Hz: entries ");
    for (uint8_t i = 0; i < blocks; ++i) {
        print(serial, first_entry[i], " ");
    }
    print(serial, "; ", inside_bad, " samples off the table, ", steps_ok,
          " of ", blocks - 1u, " seams stepped by ", block_len % table_len,
          "; DAC laps ", laps, ", engine overruns ", overruns,
          ", DAC underrun ", underrun, crlf);

    bench.verdict("ONE TIMER PACES BOTH CONVERTERS AND THE CPU IS IN NEITHER "
                  "PATH: TIM6's TRGO starts the DAC and the ADC on the same "
                  "edge, a DMA table feeds one and a DMA stream drains the "
                  "other", pacer && dac_ok && adc_ok && loop_started &&
                               pong_started && blocks >= 6u);
    bench.verdict("and every sample inside every block is the NEXT table "
                  "entry - the loop wrapped under the stream and nothing "
                  "slipped", inside_bad == 0u);
    bench.verdict("the SEAMS hold too: a block of 24 over a table of 16 must "
                  "start 8 entries further on every time, and it does",
                  steps_ok == blocks - 1u);
    bench.verdict("no engine overran and the DAC never underran, so the two "
                  "streams kept up with the pace they were given",
                  overruns == 0u && !underrun && laps >= 6u);
    quiet_everything();
}

// =============================================================================
// l - the errata pass
// =============================================================================
void tl_errata() {
    if (!analog_up(cfg_pad)) {
        bench.verdict("the ADC came up", false);
        return;
    }

    // ES0548 2.6.2, staged with its own control. The workaround IS the
    // driver's shape - configure() refuses while enabled - so the staging
    // goes through configure_while_enabled(), which exists for this and
    // is named so nobody reaches it by accident.
    AdcConfig ten = cfg_pad;
    ten.resolution = AdcRes::bits10;
    const bool set_ok = apply(ten);
    const uint32_t res_before =
        (Adc::regs().CFGR1 & ADC_CFGR1_RES_Msk) >> ADC_CFGR1_RES_Pos;
    Adc::configure_while_enabled(ten);          // a CFGR1 write with ADEN set
    const uint32_t res_after =
        (Adc::regs().CFGR1 & ADC_CFGR1_RES_Msk) >> ADC_CFGR1_RES_Pos;
    // The control: the same word written with the converter DISABLED.
    (void)Adc::disable();
    (void)Adc::configure(ten);
    const uint32_t res_control =
        (Adc::regs().CFGR1 & ADC_CFGR1_RES_Msk) >> ADC_CFGR1_RES_Pos;
    (void)Adc::enable();
    print(serial, "  ES0548 2.6.2: RES was ", res_before,
          ", after a CFGR1 write with ADEN set it reads ", res_after,
          ", and with ADEN clear (the control) ", res_control,
          " (1 = 10 bits, 0 = 12)", crlf);
    bench.verdict("the CONTROL is sound: a CFGR1 write with the converter "
                  "DISABLED keeps the resolution it was given",
                  set_ok && res_before == 1u && res_control == 1u);
    bench.verdict("ES0548 2.6.2 STAGED: a CFGR1 write with ADEN set is what "
                  "the erratum is about, and the driver's own answer is "
                  "STRUCTURAL - configure() refuses while enabled, so the "
                  "combination cannot be spelled through the API at all",
                  !Adc::configure(cfg_pad) && Adc::enabled() &&
                      (res_after == 0u || res_after == 1u));
    (void)apply(cfg_pad);

    // ES0548 2.6.1: a timing obligation on the READER, which no driver
    // can enforce - so it is stated and its shape is shown.
    print(serial, "  ES0548 2.6.1 is a TIMING obligation and not a register "
          "state - OVR may stay low when an EOC clear coincides with a "
          "conversion end, and the workaround is to clear EOC well inside one "
          "conversion period. No driver can enforce that, so overrun() states "
          "it and no verdict is offered.", crlf);

    // The overrun path itself, which the erratum only qualifies.
    AdcConfig cont = cfg_pad;
    cont.continuous = true;
    cont.overrun_overwrite = false;
    (void)apply(cont);
    (void)Adc::select_channel(0);
    Adc::clear_flags(AdcFlag::overrun);
    Adc::start();
    spin_cycles(SysClock::hz / 1000u);
    const bool overran = Adc::overrun();
    (void)Adc::stop();
    Adc::clear_flags(AdcFlag::overrun);
    bench.verdict("the overrun itself is real and reachable: a continuous "
                  "conversion nobody reads raises OVR within a millisecond "
                  "(15.5.2)", overran);

    print(serial, "  ES0548 2.6.3 is staged in letter h; 2.6.4 is measured in "
          "letter c; 2.6.5 (ADC offset out of specification) is REVISION A "
          "ONLY and this die is revision Z, so it does not apply.", crlf);
    bench.verdict("and the pass leaves the block as it found it: nothing "
                  "here wrote a lock bit, an option byte or a flash cell",
                  !C1::locked() && !C2::locked() && !C3::locked() &&
                      !Vref::enabled());
    quiet_everything();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_stm32_analog - ADC + DAC + VREFBUF + COMP "
          "(RM0444 ch. 15..18) on the STM32G0B1RE", crlf,
          "  nothing to wire; PA4 is DAC1_OUT1 AND ADC_IN4, which is the "
          "only route between the two converters on this family", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf, "  ?  this menu", crlf);
}

}  // namespace

// =============================================================================
// The vectors
// =============================================================================
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// ONE VECTOR, FOUR SOURCES: the ADC and all three comparators' EXTI
/// lines (table 61). Every one of them is answered here, each for its own
/// flags, which is what makes this a dispatcher rather than a handler.
extern "C" void ADC1_COMP_IRQHandler() {
    const uint32_t f = Adc::isr();
    if ((f & brio::AdcFlag::converted) != 0u) {
        adc_eoc_calls = adc_eoc_calls + 1u;
        // The result and the input code are read TOGETHER, in the
        // interrupt: util/analog_sampler.hpp's whole attribution rule is
        // that a late dispatch may lose a sample and can never mislabel
        // one.
        const uint16_t value = Adc::result();
        const uint8_t input = Adc::selected();
        if (kernel_mode) {
            brio::post<Sampler>(brio::Sampled{value, input});
        }
    }
    if ((f & (brio::AdcFlag::watchdog1 | brio::AdcFlag::watchdog2 |
              brio::AdcFlag::watchdog3)) != 0u) {
        adc_awd_calls = adc_awd_calls + 1u;
    }

    constexpr uint32_t comp_lines = (1u << C1::exti_line) | (1u << C2::exti_line) |
                                    (1u << C3::exti_line);
    const brio::ExtiPending p = brio::Exti::isr(comp_lines);
    if (p.any()) {
        comp_exti_calls = comp_exti_calls + 1u;
        if (p.rising != 0u) {
            comp_exti_rising = comp_exti_rising + 1u;
        }
        if (p.falling != 0u) {
            comp_exti_falling = comp_exti_falling + 1u;
        }
    }
}

/// The DAC's own line, shared with TIM6 and LPTIM1.
extern "C" void TIM6_DAC_LPTIM1_IRQHandler() {
    if (Dac::isr() != 0u) {
        dac_underrun_calls = dac_underrun_calls + 1u;
    }
}

/// DMA1 channel 1: the table the DAC is played from.
extern "C" void DMA1_Channel1_IRQHandler() {
    const uint8_t f = DacLoop::service();
    if ((f & DacLoop::flag_error) != 0u) {
        DacLoop::fail();
    } else if ((f & DacLoop::flag_complete) != 0u) {
        (void)DacLoop::complete();
        loop_laps = loop_laps + 1u;
    }
}

/// DMA1 channels 2 and 3: the ADC's stream is on channel 2.
extern "C" void DMA1_Channel2_3_IRQHandler() {
    const uint8_t f = AdcPong::service();
    if ((f & AdcPong::flag_error) != 0u) {
        AdcPong::fail();
    } else if ((f & AdcPong::flag_complete) != 0u) {
        (void)AdcPong::complete();
        pong_blocks = pong_blocks + 1u;
    }
}

int main() {
    // Sampled BEFORE anything of ours runs: letter a judges what this boot
    // found, and every verb of this suite writes some of it.
    boot_apbenr1 = RCC->APBENR1;
    boot_apbenr2 = RCC->APBENR2;
    boot_ccipr = RCC->CCIPR;
    boot_adc_cr = ADC1->CR;
    boot_adc_cfgr1 = ADC1->CFGR1;
    // COMP1_CSR *and VREFBUF_CSR* live inside the SYSCFG block, which is
    // clocked at reset by nothing - so the gate is opened first and both
    // reads are honest. Read through the closed gate (the first version
    // of this suite did) VREFBUF_CSR answers 0x0, which is not its reset
    // value but table 91's OTHER off mode.
    boot_vrefbuf_clockless = VREFBUF->CSR;
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
    (void)RCC->APBENR2;
    boot_vrefbuf_csr = VREFBUF->CSR;
    boot_comp1_csr = COMP1->CSR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    brio::enable_interrupts();

    bench.letter('a', "the block: geometry, boot registers, calibration, refusals, pads",
                 ta_block);
    bench.letter('b', "the scale: VDDA from VREFINT, the junction temperature, VBAT",
                 tb_scale);
    bench.letter('c', "conversion time exact to the CPU cycle, and ES0548 2.6.4",
                 tc_timing);
    bench.letter('d', "the zero-length wire: the DAC read back through ADC_IN4",
                 td_zero_length_wire);
    bench.letter('e', "the LED as an analog dimmer, and what its load costs",
                 te_led_dimmer);
    bench.letter('f', "the sequencer, both faces, and the CCRDY handshake",
                 tf_sequencer);
    bench.letter('g', "the noise floor measured, and then the oversampler",
                 tg_oversampler);
    bench.letter('h', "the three analog watchdogs, and ES0548 2.6.3 staged",
                 th_watchdogs);
    bench.letter('i', "the comparators: the muxes, the window, EXTI 17, TIM1's TI1",
                 ti_comparators);
    bench.letter('j', "AnalogSampler inside a real kernel, three inputs",
                 tj_sampler_ao);
    bench.letter('k', "the no-CPU chain: one TRGO, both converters, two DMA engines",
                 tk_chain);
    bench.letter('l', "the errata pass: ES0548 2.6.2 staged with a control",
                 tl_errata);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
