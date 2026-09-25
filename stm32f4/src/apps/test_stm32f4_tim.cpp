// test_stm32f4_tim - the reference bench suite for the STM32F4's timers:
// the advanced-control pair, the four-channel general-purpose timers, the
// small ones and the basic ones - their time base and its shadow
// registers, the counting modes, the capture/compare channels in both
// faces, the slave controller and the master TRGO, the option registers,
// the break/dead-time unit and the four split vectors - stm32f4/tim.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. Four stimuli, all inside the chip:
//   - THE INTERNAL TRIGGERS: a master's TRGO into a slave's ITRx, which
//     makes one timer count another exactly, with no pad and no CPU;
//   - THE OPTION REGISTERS: TIM5_OR puts the LSE or the LSI on TIM5's
//     channel 4 and TIM11_OR puts HSE_RTC on TIM11's channel 1, so an
//     oscillator is weighed against the core clock with no pad at all;
//   - A PAD THE PROGRAM DRIVES: a GPIO in output mode still feeds its
//     input buffer, so a pad whose AF nibble names a timer channel is
//     both the program's output and the timer's input;
//   - SysTick, the ruler every time below is measured against.
//
// What is exercised, letter by letter:
//   a  the reset state, the geometry, and what a small timer refuses
//   b  the time base: TIMxCLK measured against the core clock, TIMPRE,
//      and the two shadow registers
//   c  the internal trigger: one timer counting another, exactly
//   d  the gated counter: a duty cycle measured with no pad
//   e  a PWM output read back on its own pad
//   f  the two 32-bit counters, and the refusals at the width
//   g  the LSE and the LSI weighed through TIM5's option register
//   h  HSE_RTC weighed through TIM11's option register
//   i  one-pulse mode: the width on the pad, and the counter that stops
//   j  the repetition counter's update rate
//   k  the complementary pair and the dead time in nanoseconds
//   l  the break: MOE cleared, the outputs parked, and the way back
//   m  the encoder, driven by two pads the program toggles
//   n  the meter tasks against the ruler
//   o  the vectors: four lines on one timer, one line for two timers
//
// build: boards = f411ce,f446re,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/rtc.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F469xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000,
                             brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the board ----------------------------------------------------------------
//
// FIVE PADS, none of them the board's own: a waveform pad on a
// general-purpose channel, the advanced-control timer's channel 1 and its
// complement, and a quadrature pair. Every one is free on the three boards
// this suite builds for, and the AF numbers are the datasheets' (DS10314
// table 9, DS10693 table 11, DS11189 table 12 - the same rows on all
// three). The quadrature pair alone moves: TIM4's channels 1 and 2 are
// PB6/PB7 on the black pill and the Nucleo, and on the 32F469IDISCOVERY
// those two pads are the QSPI flash's select and the USB power switch's
// over-current input, so the pair takes the timer's other pads, PD12 and
// PD13 (MB1189: PD12 unconnected, PD13 to an unfitted resistor).
#if defined(STM32F411xE)
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
constexpr uint32_t hse_hz = 25'000'000u;
#elif defined(STM32F469xx)
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
constexpr uint32_t hse_hz = 8'000'000u;
#else
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
constexpr uint32_t hse_hz = 8'000'000u;
#endif

constexpr PinSel wave_sel{'A', 6, PinFunction::af2};     // TIM3_CH1
constexpr PinSel adv_sel{'A', 8, PinFunction::af1};      // TIM1_CH1
constexpr PinSel advn_sel{'B', 13, PinFunction::af1};    // TIM1_CH1N
#if defined(STM32F469xx)
constexpr PinSel enc_a_sel{'D', 12, PinFunction::af2};   // TIM4_CH1
constexpr PinSel enc_b_sel{'D', 13, PinFunction::af2};   // TIM4_CH2
#else
constexpr PinSel enc_a_sel{'B', 6, PinFunction::af2};    // TIM4_CH1
constexpr PinSel enc_b_sel{'B', 7, PinFunction::af2};    // TIM4_CH2
#endif

using WavePad = TimPad<wave_sel>;
using AdvPad = TimPad<adv_sel>;
using AdvnPad = TimPad<advn_sel>;
using EncA = TimPad<enc_a_sel>;
using EncB = TimPad<enc_b_sel>;

// ---- the timers this suite uses -------------------------------------------------
using Adv = Tim<1>;        // advanced-control: four vectors, RCR, BDTR
using Wide = Tim<2>;       // 32-bit, APB1
using Wave = Tim<3>;       // the waveform generator on WavePad
using Quad = Tim<4>;       // the encoder
using Meter = Tim<5>;      // 32-bit, and the option register that reaches the LSE
using Small2 = Tim<9>;     // two channels, a slave controller, no encoder
using Small1 = Tim<11>;    // one channel, no slave controller, HSE_RTC on its input

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- what the registers held before this program touched them -------------------
struct BootState {
    bool apb1_gate = false, apb2_gate = false;
    uint32_t cr1 = 0, cr2 = 0, smcr = 0, dier = 0, sr = 0, ccer = 0, arr = 0, opt = 0;
    uint8_t rtcpre = 0;
    bool timpre = false;
};
BootState boot;

// ---- handler bookkeeping ---------------------------------------------------------
volatile uint16_t adv_update = 0, adv_cc = 0, adv_brk = 0, adv_trg = 0;
volatile uint16_t small2_entries = 0, small1_entries = 0;
volatile uint32_t adv_brk_flags = 0;

void reset_counters() {
    P::CriticalSection cs;
    adv_update = 0;
    adv_cc = 0;
    adv_brk = 0;
    adv_trg = 0;
    adv_brk_flags = 0;
    small2_entries = 0;
    small1_entries = 0;
}

// ---- a ruler --------------------------------------------------------------------
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// A cycle count that spans ticks. It wraps every forty seconds or so, so
/// only DIFFERENCES of it are used below; every bound on a wait is a
/// millisecond deadline on the ticker instead.
uint32_t cycles_now() {
    const uint32_t period = systick_period();
    uint32_t t0 = 0, v = 0, t1 = 0;
    do {
        t0 = Ticker::ticks();
        v = SysTick->VAL;
        t1 = Ticker::ticks();
    } while (t0 != t1);
    return t0 * period + (period - 1u - v);
}

/// A wait that cannot hang: true while `ms` milliseconds have not passed
/// since `start`.
bool within(uint32_t start, uint32_t ms) { return Ticker::ticks() - start < ms; }

/// Two counts within `slack` of each other.
bool near(uint32_t a, uint32_t b, uint32_t slack) {
    return (a > b ? a - b : b - a) <= slack;
}

/// Everything this suite ever starts, stopped and back to its reset
/// state, so no letter inherits another's.
void quiet_everything() {
    Adv::release();
    Wide::release();
    Wave::release();
    Quad::release();
    Meter::release();
    Small2::release();
    Small1::release();
    Nvic::disable(Adv::irq());
    Nvic::disable(Adv::cc_irq());
    Nvic::disable(Adv::break_irq());
    Nvic::disable(Adv::trigger_irq());
    Nvic::disable(Small2::irq());
    Nvic::disable(Small1::irq());
    Nvic::clear_pending(Adv::irq());
    Nvic::clear_pending(Adv::cc_irq());
    Nvic::clear_pending(Adv::break_irq());
    Nvic::clear_pending(Adv::trigger_irq());
    WavePad::release();
    AdvPad::release();
    AdvnPad::release();
    EncA::release();
    EncB::release();
    reset_counters();
}

/// The clock every timer of this suite counts, as the header derives it.
constexpr uint32_t apb1_tim_hz = tim_clock_hz(clock, false);
constexpr uint32_t apb2_tim_hz = tim_clock_hz(clock, true);

// =============================================================================
// a - the reset state, the geometry, and what a small timer refuses
// =============================================================================
void ta_reset_state() {
    print(serial, "  at boot: APB1 gate ", boot.apb1_gate ? "OPEN" : "closed", ", APB2 gate ",
          boot.apb2_gate ? "OPEN" : "closed", ", TIM3 CR1=", hex(boot.cr1), " SMCR=",
          hex(boot.smcr), " DIER=", hex(boot.dier), " SR=", hex(boot.sr), " CCER=",
          hex(boot.ccer), " ARR=", hex(boot.arr), " OR=", hex(boot.opt), crlf);
    bench.verdict("a timer's bus clock is CLOSED out of reset - nothing but this driver "
                  "opens it",
                  !boot.apb1_gate && !boot.apb2_gate);
    bench.verdict("and every register of it reads zero behind a closed gate, ARR "
                  "included - which is the register map's reset value for all but ARR, "
                  "and the proof that a peripheral without its clock does not answer",
                  (boot.cr1 | boot.cr2 | boot.smcr | boot.dier | boot.sr | boot.ccer |
                   boot.arr | boot.opt) == 0u);

    // init() opens the gate and pulses the reset: the register map's own
    // reset values, whatever a bootloader left.
    Wave::init();
    bench.verdict("init() opens the gate", Wave::bus_clock());
    const bool armed = Wave::configure({.prescaler = 7, .period = 99}) &&
                       Wave::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 50});
    Wave::enable(true);
    const bool running = Wave::enabled() && Wave::period() == 99u;
    Wave::reset();
    const bool cleared = !Wave::enabled() && Wave::period() == 0xFFFFu &&
                         Wave::regs().CCER == 0u && Wave::regs().CCMR1 == 0u;
    print(serial, "  after the reset pulse: ARR=", hex(Wave::period()), " CCMR1=",
          hex(Wave::regs().CCMR1), " CCER=", hex(Wave::regs().CCER), crlf);
    bench.verdict("configure() and a channel take, and the counter runs", armed && running);
    bench.verdict("the reset pulse puts every register back to the map's value (ARR is "
                  "the one that is not zero: it resets to all ones)",
                  cleared);
    Wave::release();
    bench.verdict("release() closes the gate again", !Wave::bus_clock());

    // The geometry the reserve states, printed and asserted.
    print(serial, "  TIM1: ", Adv::counter_bits, " bits, ", Adv::channels, " channels, ",
          Adv::complementary_channels, " complementary; TIM2: ", Wide::counter_bits,
          " bits; TIM9: ", Small2::channels, " channels; TIM11: ", Small1::channels,
          " channel", crlf);
    bench.verdict("the two 32-bit counters are TIM2 and TIM5, and no other",
                  Wide::counter_bits == 32 && Meter::counter_bits == 32 &&
                      Wave::counter_bits == 16 && Adv::counter_bits == 16);
    bench.verdict("the break unit, the repetition counter and the complementary outputs "
                  "are one fact under three names, and TIM1's alone here",
                  Adv::has_break && Adv::has_repetition && Adv::complementary_channels == 3 &&
                      !Wave::has_break && !Meter::has_break);

    // A small timer refuses what it has not got, rather than storing into
    // a hole in the address map.
    Small1::init();
    const bool no_slave = !Small1::slave({.mode = TimSlaveMode::gated});
    const bool no_master = !Small1::master(TimMasterMode::update);
    const bool no_etr = !Small1::external_trigger({.clock_mode2 = true});
    const bool no_bdtr = !Small1::break_dead_time({}) && !Small1::main_output(true) &&
                         !Small1::break_event();
    const bool no_rcr = !Small1::set_repetition(3);
    const bool no_burst = !Small1::dma_burst(TimBurstBase::arr, 2) &&
                          Small1::dmar_address() == nullptr;
    const bool no_second_channel = !Small1::set_compare(1, 10) &&
                                   Small1::ccr_address(1) == nullptr;
    const uint32_t after = Small1::regs().SMCR | Small1::regs().CR2 | Small1::regs().BDTR |
                           Small1::regs().RCR | Small1::regs().DCR;
    print(serial, "  after every refused verb TIM11's SMCR|CR2|BDTR|RCR|DCR reads ",
          hex(after), crlf);
    bench.verdict("a timer with no slave controller, no CR2, no BDTR, no RCR and no "
                  "burst engine REFUSES each of them",
                  no_slave && no_master && no_etr && no_bdtr && no_rcr && no_burst &&
                      no_second_channel);
    bench.verdict("and wrote nothing into the holes those registers are", after == 0u);

    // TIM9 is the middle case: a slave controller, and no encoder in it.
    Small2::init();
    bench.verdict("TIM9 slaves but does not count a quadrature pair - SMS 001..011 are "
                  "Reserved there",
                  Small2::slave({.mode = TimSlaveMode::gated, .trigger = TimTrigger::itr0}) &&
                      !Small2::slave({.mode = TimSlaveMode::encoder3}) &&
                      !Small2::master(TimMasterMode::update));
    quiet_everything();
}

// =============================================================================
// b - the time base: TIMxCLK measured, TIMPRE, and the shadow registers
// =============================================================================

/// Count `t`'s ticks over a window of about `ms` milliseconds and return
/// the rate its counter really runs at, in Hz. The timer must be 32-bit
/// (or prescaled enough not to wrap).
template <class T>
uint32_t measure_rate(uint32_t ms, uint16_t prescaler) {
    T::init();
    (void)T::configure({.prescaler = prescaler, .period = T::max_period});
    T::enable(true);
    const uint32_t c0 = cycles_now();
    const uint32_t n0 = T::count();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
    }
    const uint32_t n1 = T::count();
    const uint32_t c1 = cycles_now();
    T::enable(false);
    const uint64_t ticks = static_cast<uint64_t>((n1 - n0) & T::max_period) *
                           (static_cast<uint64_t>(prescaler) + 1u);
    return static_cast<uint32_t>(ticks * SysClock::hz / (c1 - c0));
}

void tb_time_base() {
    quiet_everything();

    // TIM9 is 16-bit, so it is prescaled far enough not to wrap inside
    // the window; TIM2 is 32-bit and counts its clock undivided.
    const uint32_t apb1_measured = measure_rate<Wide>(200, 0);
    const uint32_t apb2_measured = measure_rate<Small2>(200, 4999);
    print(serial, "  HCLK ", SysClock::hz / 1'000'000u, " MHz, PCLK1 ",
          SysClock::pclk1_hz / 1'000'000u, " MHz (prescaler ", SysClock::apb1_div,
          "), PCLK2 ", SysClock::pclk2_hz / 1'000'000u, " MHz (prescaler ",
          SysClock::apb2_div, ")", crlf);
    print(serial, "  an APB1 timer counts ", apb1_measured, " Hz (the header says ",
          apb1_tim_hz, "), an APB2 timer ", apb2_measured, " Hz (", apb2_tim_hz, ")", crlf);
    const uint32_t tol1 = apb1_tim_hz / 500u;
    const uint32_t tol2 = apb2_tim_hz / 500u;
    bench.verdict("AN APB1 TIMER'S CLOCK IS NOT ITS BUS CLOCK: measured against the core "
                  "clock it is what tim_clock_hz() derives from the prescalers, within "
                  "two parts in a thousand",
                  apb1_measured + tol1 > apb1_tim_hz && apb1_measured < apb1_tim_hz + tol1);
    bench.verdict("and so is an APB2 timer's",
                  apb2_measured + tol2 > apb2_tim_hz && apb2_measured < apb2_tim_hz + tol2);
    bench.verdict("the silicon's own prescalers agree with the clock task's",
                  Wide::clock_hz_now(SysClock::hz) == apb1_tim_hz &&
                      Small2::clock_hz_now(SysClock::hz) == apb2_tim_hz);

    // TIMPRE: on a bus divided by more than two it doubles the timers'
    // clock; where every prescaler is 1 or 2 it can change nothing.
    Rcc::timpre(true);
    const uint32_t apb1_high = measure_rate<Wide>(200, 0);
    const uint32_t expect_high = tim_clock_hz(clock, false, true);
    Rcc::timpre(boot.timpre);
    print(serial, "  with TIMPRE set an APB1 timer counts ", apb1_high, " Hz (the header "
          "says ", expect_high, ")", crlf);
    const uint32_t tolh = expect_high / 500u;
    bench.verdict("TIMPRE moves the timers' clock exactly as the rule says - HCLK at a "
                  "prescaler of 1 or 2, four times PCLK beyond",
                  Rcc::timpre() == boot.timpre && apb1_high + tolh > expect_high &&
                      apb1_high < expect_high + tolh);

    // The two shadow registers (17.4.11, 17.4.12). The counter is left
    // free-running over the whole 32 bits so a delta is a delta.
    Wide::init();
    (void)Wide::configure({.prescaler = 0, .period = 0xFFFFFFFFUL});
    Wide::enable(true);
    Wide::set_prescaler(9999);
    const uint32_t before = Wide::count();
    (void)delay_us(clock, 200);
    const uint32_t moved_fast = Wide::count() - before;
    Wide::update();
    Wide::clear_flags(Wide::update_flag);
    const uint32_t after0 = Wide::count();
    (void)delay_us(clock, 200);
    const uint32_t moved_slow = Wide::count() - after0;
    print(serial, "  PSC written to 9999 while running: ", moved_fast,
          " ticks in 200 us before the update event, ", moved_slow, " after", crlf);
    bench.verdict("THE PRESCALER IS SHADOWED: a write to PSC does nothing until an "
                  "update event loads it",
                  moved_fast > 10u * moved_slow && moved_slow > 0u);

    Wide::enable(false);
    (void)Wide::configure({.prescaler = 0, .period = 999, .auto_reload_preload = true});
    Wide::enable(true);
    (void)Wide::set_period(499);
    const bool arr_held = Wide::period() == 499u;   // the register reads what was written
    Wide::update();
    Wide::clear_flags(Wide::update_flag);
    Wide::enable(false);
    bench.verdict("ARR reads back what was written whether it is preloaded or not - the "
                  "shadow is not the register",
                  arr_held);

    bench.verdict("a null auto-reload is refused: it would block the counter while the "
                  "timer looked alive",
                  !Wide::set_period(0) && !Wide::configure({.period = 0}));
    quiet_everything();
}

// =============================================================================
// c - the internal trigger: one timer counting another, exactly
// =============================================================================
void tc_internal_trigger() {
    quiet_everything();

    constexpr uint8_t link = tim_trigger_index_for(3, 2);
    print(serial, "  TIM3 reaches TIM2 on ITR", link, "; TIM2 reaches TIM1 on ITR",
          tim_trigger_index_for(2, 1), ", TIM5 reaches TIM3 on ITR",
          tim_trigger_index_for(5, 3), crlf);
    bench.verdict("the manual's trigger table answers in the direction a caller thinks "
                  "in - the MASTER, not a number",
                  link == 1u && tim_internal_trigger(3, link) == 2u);

    // THE EXACT TEST: the master's TRGO is its RESET event, which EGR.UG
    // raises by software - so N software updates must be N counts in the
    // slave and not one more.
    Wide::init();
    Wave::init();
    (void)Wide::configure({.prescaler = 0, .period = 0xFFFFu});
    (void)Wide::master(TimMasterMode::reset);
    // The slave's clock is enabled and its mode written FIRST: the
    // errata's own obligation, and this is the order it wants.
    const bool slaved = TimEventCounter<Wave>::setup(static_cast<TimTrigger>(link));
    TimEventCounter<Wave>::restart();
    for (uint16_t k = 0; k < 100u; ++k) {
        Wide::update();
    }
    // The trigger is resynchronized to the slave's clock, so the last
    // edge needs a few cycles to reach the counter before it is read.
    (void)delay_us(clock, 10);
    const uint32_t counted = TimEventCounter<Wave>::count();
    print(serial, "  100 software update events on the master: the slave counted ",
          counted, crlf);
    bench.verdict("A SLAVE IN EXTERNAL CLOCK MODE 1 COUNTS ITS MASTER'S TRGO EXACTLY - "
                  "a hundred events, a hundred counts, with no pad and no CPU in the path",
                  slaved && counted == 100u);

    // A slave pointed at an ITRx nothing drives counts nothing.
    TimEventCounter<Wave>::restart();
    (void)Wave::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::itr3});
    for (uint16_t k = 0; k < 100u; ++k) {
        Wide::update();
    }
    (void)delay_us(clock, 10);
    const uint32_t wrong = TimEventCounter<Wave>::count();
    bench.verdict("and a slave pointed at another ITRx counts nothing at all", wrong == 0u);

    // The free-running version: the master's UPDATE event as a frequency
    // divider, measured against the ruler.
    (void)Wave::slave({.mode = TimSlaveMode::external_clock1, .trigger = static_cast<TimTrigger>(link)});
    Wave::set_count(0);
    (void)Wide::configure({.prescaler = 0, .period = 999});
    (void)Wide::master(TimMasterMode::update);
    Wide::enable(true);
    const uint32_t c0 = cycles_now();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 100u) {
    }
    const uint32_t c1 = cycles_now();
    const uint32_t events = Wave::count();
    Wide::enable(false);
    const uint32_t expected = static_cast<uint32_t>(
        static_cast<uint64_t>(c1 - c0) * apb1_tim_hz / SysClock::hz / 1000u);
    print(serial, "  a 1000-tick master over ", (c1 - c0) / cycles_per_us, " us: ", events,
          " update events, ", expected, " expected", crlf);
    const uint32_t slack = expected / 200u + 2u;
    bench.verdict("the same link free-running divides the master's clock by its period, "
                  "to within half a per cent of the ruler",
                  events + slack > expected && events < expected + slack);
    quiet_everything();
}

// =============================================================================
// d - the gated counter: a duty cycle measured with no pad
// =============================================================================
void td_gated_duty() {
    quiet_everything();

    constexpr uint8_t link = tim_trigger_index_for(5, 3);
    Wave::init();
    Meter::init();
    // The master publishes OC1REF - the waveform ITSELF - on TRGO, so the
    // slave's gate is the PWM's high time and nothing has to reach a pad.
    (void)Wave::configure({.prescaler = 0, .period = 999, .auto_reload_preload = true});
    (void)Wave::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 250});
    (void)Wave::master(TimMasterMode::oc1ref);
    const bool gated = TimGatedCounter<Meter>::setup(static_cast<TimTrigger>(link));
    TimGatedCounter<Meter>::restart();
    Wave::enable(true);

    const uint32_t c0 = cycles_now();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 100u) {
    }
    const uint32_t c1 = cycles_now();
    const uint32_t high = TimGatedCounter<Meter>::count();
    Wave::enable(false);

    const uint32_t total = static_cast<uint32_t>(
        static_cast<uint64_t>(c1 - c0) * apb1_tim_hz / SysClock::hz);
    const uint32_t per_mille = static_cast<uint32_t>(static_cast<uint64_t>(high) * 1000u / total);
    print(serial, "  duty 250/1000 at ", apb1_tim_hz / 1000u, " kHz counter ticks: the "
          "gate was open ", high, " of ", total, " ticks = ", per_mille, " per mille", crlf);
    bench.verdict("A GATED SLAVE MEASURES A DUTY CYCLE INSIDE THE CHIP: the master's own "
                  "OC1REF is the gate, and the count is the high time to five per mille",
                  gated && per_mille + 5u > 250u && per_mille < 255u);

    // The two ends of the scale, where the waveform has no edge at all.
    for (uint32_t duty : {0u, 1000u}) {
        (void)Wave::set_compare(0, duty);
        Wave::update();
        Wave::clear_flags(Wave::update_flag);
        TimGatedCounter<Meter>::restart();
        Wave::enable(true);
        const uint32_t d0 = cycles_now();
        const uint32_t s0 = Ticker::ticks();
        while (Ticker::ticks() - s0 < 20u) {
        }
        const uint32_t d1 = cycles_now();
        const uint32_t got = TimGatedCounter<Meter>::count();
        Wave::enable(false);
        const uint32_t window = static_cast<uint32_t>(
            static_cast<uint64_t>(d1 - d0) * apb1_tim_hz / SysClock::hz);
        if (duty == 0u) {
            bench.verdict("at a compare of zero the gate never opens", got == 0u);
        } else {
            print(serial, "  at a compare of 1000 the gate stood open ", got, " of ",
                  window, " ticks", crlf);
            bench.verdict("at a compare equal to the period it never closes",
                          got + window / 100u > window);
        }
    }
    quiet_everything();
}

// =============================================================================
// e - a PWM output read back on its own pad
// =============================================================================

/// Sample a pad `samples` times as fast as the loop can and return how
/// many reads were high.
uint32_t sample_high(uint32_t samples) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        if (WavePad::read()) {
            ++high;
        }
    }
    return high;
}

uint32_t sample_adv(uint32_t samples) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        if (AdvPad::read()) {
            ++high;
        }
    }
    return high;
}

constexpr uint32_t pad_samples = 100000;

void te_pwm_pad() {
    quiet_everything();
    WavePad::claim();
    using Lamp = TimPwm<Wave, 0, 999>;
    static_assert(PwmChannel<Lamp>, "a PWM channel is what an actuator above asks for");
    Wave::init();
    // About 10 kHz, so a sampling loop of a few milliseconds crosses many
    // tens of periods and its own phase averages out.
    const uint16_t psc = static_cast<uint16_t>(apb1_tim_hz / 1000u / 10000u - 1u);
    const bool up = Lamp::setup(psc);
    print(serial, "  TIM3_CH1 on the pad at ", apb1_tim_hz / (psc + 1u) / 1000u,
          " Hz, max ", Lamp::max, crlf);

    Lamp::duty(0);
    (void)delay_us(clock, 5000);
    const uint32_t at_zero = sample_high(pad_samples);
    Lamp::duty(Lamp::max);
    (void)delay_us(clock, 5000);
    const uint32_t at_full = sample_high(pad_samples);
    Lamp::duty(500);
    (void)delay_us(clock, 5000);
    const uint32_t at_half = sample_high(pad_samples);
    print(serial, "  100000 reads of the pad's own IDR: ", at_zero, " high at duty 0, ",
          at_half, " at 500, ", at_full, " at 999", crlf);
    bench.verdict("AN ALTERNATE-FUNCTION PAD FEEDS ITS OWN INPUT BUFFER: the waveform "
                  "the timer drives is readable on IDR with nothing attached - a "
                  "compare of zero never rises and a compare at the period barely falls",
                  up && at_zero == 0u && at_full > pad_samples - pad_samples / 100u);
    bench.verdict("and half the period is half the samples, within three per cent",
                  near(at_half, pad_samples / 2u, pad_samples / 33u));

    Lamp::duty(0xFFFFu);
    bench.verdict("the PwmChannel contract clamps a duty past the full scale instead of "
                  "wrapping it",
                  Lamp::duty() == Lamp::max);

    // PWM mode 2 is the complement of mode 1, on the same compare.
    (void)Lamp::setup(psc, TimOutputMode::pwm2);
    Lamp::duty(500);
    (void)delay_us(clock, 5000);
    const uint32_t inverted = sample_high(pad_samples);
    (void)Lamp::setup(psc, TimOutputMode::pwm1, true);
    Lamp::duty(250);
    (void)delay_us(clock, 5000);
    const uint32_t active_low = sample_high(pad_samples);
    print(serial, "  the same compare in PWM mode 2: ", inverted,
          " high; mode 1 with an active-low pad at duty 250: ", active_low, crlf);
    bench.verdict("PWM mode 2 is mode 1's complement and CCxP inverts the pad - two "
                  "independent inversions",
                  near(inverted, pad_samples / 2u, pad_samples / 33u) &&
                      near(active_low, pad_samples * 3u / 4u, pad_samples / 33u));
    quiet_everything();
}

// =============================================================================
// f - the two 32-bit counters, and the refusals at the width
// =============================================================================
void tf_widths() {
    quiet_everything();
    Wide::init();
    Wave::init();
    (void)Wide::configure({.prescaler = 0, .period = 0xFFFFFFFFUL});
    (void)Wave::configure({.prescaler = 0, .period = 0xFFFFu});

    Wide::set_count(0xFFFFFFF0UL);
    Wave::set_count(0xFFFFFFF0UL);
    const uint32_t wide_holds = Wide::count();
    const uint32_t narrow_holds = Wave::count();
    print(serial, "  0xFFFFFFF0 written into TIM2's counter reads ", hex(wide_holds),
          ", into TIM3's ", hex(narrow_holds), crlf);
    bench.verdict("TIM2's counter is thirty-two bits wide and holds what was written",
                  wide_holds == 0xFFFFFFF0UL && Wide::max_period == 0xFFFFFFFFUL);
    bench.verdict("a 16-bit counter takes the low half and the driver masks the write "
                  "rather than letting it wrap",
                  narrow_holds == 0xFFF0u);
    bench.verdict("and a period past the counter's width is refused, not truncated",
                  !Wave::set_period(0x10000UL) && Wide::set_period(0x10000UL) &&
                      !Wave::set_compare(0, 0x10000UL));

    // The wrap is at ARR, not at the width.
    (void)Wide::configure({.prescaler = 0, .period = 99});
    Wide::set_count(95);
    Wide::enable(true);
    (void)delay_us(clock, 100);
    Wide::enable(false);
    const uint32_t wrapped = Wide::count();
    print(serial, "  a 32-bit counter with ARR = 99 wrapped to ", wrapped, crlf);
    bench.verdict("a 32-bit counter still wraps at ITS AUTO-RELOAD and not at the width",
                  wrapped < 100u);
    quiet_everything();
}

// =============================================================================
// g - the LSE and the LSI weighed through TIM5's option register
// =============================================================================

/// Wait for one capture on channel `ch` of `T` and return it; 0xFFFFFFFF
/// when nothing arrived inside `ms`.
template <class T>
uint32_t wait_capture(uint8_t ch, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (!T::flag(T::compare_flag(ch))) {
        if (Ticker::ticks() - t0 > ms) {
            return 0xFFFFFFFFUL;
        }
    }
    return T::compare(ch);   // the read is the acknowledgement
}

/// Weigh whatever TIM5's channel 4 is pointed at, in milli-hertz. The
/// SOURCE is written after init(), because init() pulses the block's
/// reset and the option register goes back to the pad with it.
uint32_t weigh_input4(Tim5Input4 source, uint32_t captures, TimCapturePrescaler divider,
                      uint32_t per_capture) {
    Meter::init();
    (void)Meter::input4_source(source);
    (void)Meter::configure({.prescaler = 0, .period = Meter::max_period});
    (void)Meter::capture_channel(3, {.select = TimChannelSelect::direct,
                                     .polarity = TimCapturePolarity::rising,
                                     .prescaler = divider});
    Meter::enable(true);
    const uint32_t first = wait_capture<Meter>(3, 500);
    if (first == 0xFFFFFFFFUL) {
        return 0;
    }
    uint32_t last = first;
    for (uint32_t i = 0; i < captures; ++i) {
        const uint32_t c = wait_capture<Meter>(3, 500);
        if (c == 0xFFFFFFFFUL) {
            return 0;
        }
        last = c;
    }
    const uint32_t span = last - first;
    if (span == 0u) {
        return 0;
    }
    return static_cast<uint32_t>(static_cast<uint64_t>(apb1_tim_hz) * captures *
                                 per_capture * 1000u / span);
}

void print_hz(const char* what, uint32_t mhz) {
    print(serial, "  ", what, " measured ", mhz / 1000u, ".", mhz % 1000u, " Hz", crlf);
}

void tg_option_lse() {
    quiet_everything();

    // The crystal has to be running first: the backup domain's own gate,
    // its write protection, then LSEON.
    RtcDomain::pwr_bus_clock(true);
    (void)RtcDomain::unlock(true);
    RtcDomain::lse_enable(true);
    const bool lse = RtcDomain::lse_wait_ready();
    Rcc::lsi_enable(true);
    const bool lsi = Rcc::lsi_wait_ready();
    print(serial, "  LSE ", lse ? "running" : "NOT RUNNING", ", LSI ",
          lsi ? "running" : "NOT RUNNING", crlf);

    Meter::init();
    bench.verdict("TIM5 is one of the three timers with an option register, and TIM1 is "
                  "not",
                  Meter::has_option_register && !Adv::has_option_register);
    bench.verdict("its field takes the source and reads it back",
                  Meter::input4_source(Tim5Input4::lse) &&
                      Meter::input4_source() == Tim5Input4::lse);

    const uint32_t lse_mhz = weigh_input4(Tim5Input4::lse, 2048, TimCapturePrescaler::every8, 8);
    print_hz("THE 32768 Hz CRYSTAL on TIM5's channel 4", lse_mhz);
    const int32_t ppm = static_cast<int32_t>((static_cast<int64_t>(lse_mhz) - 32'768'000) *
                                             1000000 / 32'768'000);
    print(serial, "  which is ", ppm, " ppm from nominal", crlf);
    bench.verdict("THE LSE REACHES A CAPTURE CHANNEL WITH NO PAD, and weighs within a "
                  "hundred parts per million of 32768 Hz",
                  lse_mhz > 32'764'000u && lse_mhz < 32'772'000u);

    const uint32_t lsi_mhz = weigh_input4(Tim5Input4::lsi, 256, TimCapturePrescaler::every8, 8);
    print_hz("the LSI on the same channel", lsi_mhz);
    bench.verdict("and so does the LSI, inside the manual's own 17..47 kHz window",
                  lsi_mhz > 17'000'000u && lsi_mhz < 47'000'000u);

    bench.verdict("the field is two bits and every one of its four codes is a source, so "
                  "the reader gives back exactly what was written",
                  Meter::input4_source(Tim5Input4::rtc_wakeup) &&
                      Meter::input4_source() == Tim5Input4::rtc_wakeup &&
                      Meter::input4_source(Tim5Input4::pad) &&
                      Meter::input4_source() == Tim5Input4::pad);
    quiet_everything();
}

// =============================================================================
// h - HSE_RTC weighed through TIM11's option register
// =============================================================================
void th_option_hse() {
    quiet_everything();

    // RTCPRE lives in RCC_CFGR but the chapter makes it one-way: it "must
    // be configured before selecting the RTC clock source", and RTCSEL is
    // itself one-way, so a backup domain that already names a source has
    // to be RESET before the divider will move. That reset costs the
    // calendar, the alarms and the twenty backup registers - which is why
    // this letter is not part of z and is asked for by name.
    const uint8_t wanted = static_cast<uint8_t>(hse_hz / 1'000'000u);
    RtcDomain::pwr_bus_clock(true);
    (void)RtcDomain::unlock(true);
    bool wiped = false;
    if (!RtcDomain::hse_divider(wanted)) {
        RtcDomain::reset();
        wiped = true;
        (void)RtcDomain::hse_divider(wanted);
    }
    const uint8_t div = RtcDomain::hse_divider();
    print(serial, "  RTCPRE was ", boot.rtcpre, " at boot and is ", div, " now (",
          wiped ? "the backup domain was reset to free it" : "written with the domain untouched",
          "); HSE is ", hse_hz / 1'000'000u, " MHz", crlf);
    bench.verdict("the HSE divider is writable once the backup domain names no clock "
                  "source",
                  div == wanted);

    Small1::init();
    bench.verdict("TIM11's option register takes HSE_RTC and reads it back",
                  Small1::input1_source(Tim11Input1::hse_rtc) &&
                      Small1::input1_source() == Tim11Input1::hse_rtc);

    (void)Small1::configure({.prescaler = 0, .period = 0xFFFFu});
    (void)Small1::capture_channel(0, {.select = TimChannelSelect::direct,
                                      .prescaler = TimCapturePrescaler::every8});
    Small1::enable(true);
    uint32_t last = wait_capture<Small1>(0, 100);
    uint32_t span = 0;
    const uint32_t rounds = 1000;
    bool ok = last != 0xFFFFFFFFUL;
    for (uint32_t i = 0; ok && i < rounds; ++i) {
        const uint32_t c = wait_capture<Small1>(0, 100);
        if (c == 0xFFFFFFFFUL) {
            ok = false;
            break;
        }
        span += (c - last) & 0xFFFFu;   // a 16-bit counter, one wrap at most per capture
        last = c;
    }
    const uint32_t hse_rtc_hz = ok && span != 0u
        ? static_cast<uint32_t>(static_cast<uint64_t>(apb2_tim_hz) * rounds * 8u / span)
        : 0u;
    const uint32_t hse_measured = hse_rtc_hz * div;
    print(serial, "  HSE_RTC measured ", hse_rtc_hz, " Hz on a 16-bit counter at ",
          apb2_tim_hz / 1'000'000u, " MHz, so HSE is ", hse_measured / 1000u, " kHz", crlf);
    const uint32_t slack = hse_hz / 2000u;
    bench.verdict("THE HSE DIVIDED BY RTCPRE REACHES TIM11'S CHANNEL 1 WITH NO PAD, and "
                  "times the divider it is the crystal the PLL was told about - which is "
                  "the whole clock tree checked against itself",
                  hse_measured + slack > hse_hz && hse_measured < hse_hz + slack);

    (void)Small1::input1_source(Tim11Input1::pad);
    bench.verdict("codes 0, 1 and 3 all mean the pad, so the reader answers 'pad' for "
                  "each of them",
                  Small1::input1_source() == Tim11Input1::pad &&
                      Small1::option(1) && Small1::input1_source() == Tim11Input1::pad &&
                      Small1::option(3) && Small1::input1_source() == Tim11Input1::pad);
    bench.verdict("and the field is two bits: a code past it is refused", !Small1::option(4));
    quiet_everything();
}

// =============================================================================
// i - one-pulse mode: the width on the pad, and the counter that stops
// =============================================================================
void ti_one_pulse() {
    quiet_everything();
    WavePad::claim();
    Wave::init();
    using Pulse = TimOnePulse<Wave, 0>;

    // A pulse of about 500 us after a delay of about 200 us, counted in
    // microseconds so the pad can be watched by a polling loop.
    const uint16_t psc = static_cast<uint16_t>(apb1_tim_hz / 1'000'000u - 1u);
    const bool armed = Pulse::setup(psc, 200, 500);
    bench.verdict("a pulse of zero width is refused - there is nothing to make",
                  armed && !Pulse::setup(psc, 200, 0));

    const bool low_before = !WavePad::read();
    const uint32_t deadline = Ticker::ticks();
    const uint32_t t_fire = cycles_now();
    Pulse::fire();
    while (!WavePad::read() && within(deadline, 20)) {
    }
    const uint32_t t_rise = cycles_now();
    while (WavePad::read() && within(deadline, 40)) {
    }
    const uint32_t t_fall = cycles_now();
    const uint32_t delay_us_measured = (t_rise - t_fire) / cycles_per_us;
    const uint32_t width_us = (t_fall - t_rise) / cycles_per_us;
    const bool stopped = !Pulse::busy();
    print(serial, "  fire to rise ", delay_us_measured, " us (200 asked), high for ",
          width_us, " us (500 asked); the counter is ", stopped ? "stopped" : "STILL RUNNING",
          crlf);
    bench.verdict("ONE-PULSE MODE MAKES ONE PULSE: the pad rises at the compare and "
                  "falls at the update, within two per cent of the asked width",
                  low_before && width_us > 490u && width_us < 510u);
    bench.verdict("the delay before it is the compare value, to five per cent",
                  delay_us_measured > 190u && delay_us_measured < 215u);
    bench.verdict("AND THE COUNTER STOPS ITSELF: CR1.OPM clears CEN at the update, so "
                  "the timer costs nothing until the next trigger",
                  stopped);

    // Nothing more happens until it is fired again.
    (void)delay_us(clock, 2000);
    const bool quiet = !WavePad::read() && !Pulse::busy();
    Pulse::fire();
    (void)delay_us(clock, 100);
    const bool again = WavePad::read() || Pulse::busy();
    (void)delay_us(clock, 1000);
    bench.verdict("it stays quiet until it is fired again, and then makes another",
                  quiet && again);
    quiet_everything();
}

// =============================================================================
// j - the repetition counter's update rate
// =============================================================================
void tj_repetition() {
    quiet_everything();

    constexpr uint8_t link = tim_trigger_index_for(2, 1);
    Adv::init();
    Wide::init();
    (void)Adv::configure({.prescaler = 0, .period = 999});
    (void)Adv::master(TimMasterMode::update);
    (void)TimEventCounter<Wide>::setup(static_cast<TimTrigger>(link));

    uint32_t counts[4] = {};
    const uint8_t reps[4] = {0, 1, 3, 7};
    for (uint8_t i = 0; i < 4u; ++i) {
        Adv::enable(false);
        (void)Adv::configure({.prescaler = 0, .period = 999, .repetition = reps[i]});
        TimEventCounter<Wide>::restart();
        // The window is aligned to a tick BOTH ends: "ticks - t0 < 50"
        // started at an arbitrary point inside a tick is a 49..50 ms
        // window, and two per cent of slop is more than the thing being
        // measured.
        const uint32_t align = Ticker::ticks();
        while (Ticker::ticks() == align) {
        }
        Adv::enable(true);
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < 50u) {
        }
        Adv::enable(false);
        counts[i] = TimEventCounter<Wide>::count();
    }
    print(serial, "  50 ms of a 1000-tick TIM1 at RCR 0/1/3/7: ", counts[0], " / ",
          counts[1], " / ", counts[2], " / ", counts[3], " update events", crlf);
    bench.verdict("THE REPETITION COUNTER DIVIDES THE UPDATE EVENT: RCR + 1 counter "
                  "periods make one update, and the four rates halve in turn to within "
                  "half a per cent",
                  counts[0] > 100u && near(counts[1] * 2u, counts[0], counts[0] / 200u + 2u) &&
                      near(counts[2] * 4u, counts[0], counts[0] / 200u + 4u) &&
                      near(counts[3] * 8u, counts[0], counts[0] / 200u + 8u));
    bench.verdict("and a timer with no RCR refuses one rather than writing a hole",
                  Adv::set_repetition(2) && !Wave::set_repetition(2) &&
                      !Small2::set_repetition(2));
    quiet_everything();
}

// =============================================================================
// k - the complementary pair and the dead time in nanoseconds
// =============================================================================

/// The cycles between CH1 falling and CH1N rising - the dead time plus
/// this loop's own latency, which is why it is measured twice.
uint32_t complementary_gap() {
    uint32_t best = 0xFFFFFFFFUL;
    for (uint8_t k = 0; k < 8u; ++k) {
        const uint32_t deadline = Ticker::ticks();
        while (!AdvPad::read() && within(deadline, 50)) {
        }
        while (AdvPad::read() && within(deadline, 50)) {
        }
        const uint32_t t0 = cycles_now();
        while (!AdvnPad::read() && within(deadline, 50)) {
        }
        const uint32_t took = cycles_now() - t0;
        if (took < best) {
            best = took;
        }
    }
    return best;
}

void tk_dead_time() {
    quiet_everything();
    AdvPad::claim();
    AdvnPad::claim();
    Adv::init();
    using Pair = TimPairPwm<Adv, 0, 999>;
    static_assert(PwmChannel<Pair>, "the pair is one actuator and one duty");

    // Slow enough that a polling loop can watch both pads: about 200 Hz.
    const uint16_t psc = static_cast<uint16_t>(apb2_tim_hz / 1000u / 200u - 1u);
    (void)Pair::setup(psc, 0, TimClockDivision::div4);
    Pair::duty(500);
    (void)delay_us(clock, 20000);
    const uint32_t baseline = complementary_gap();

    const uint8_t dtg = 0xFF;
    (void)Pair::setup(psc, dtg, TimClockDivision::div4);
    Pair::duty(500);
    (void)delay_us(clock, 20000);
    const uint32_t with_dead = complementary_gap();

    // tDTS is TIMxCLK divided by CKD, and the dead time is in tDTS ticks -
    // it does NOT pass through the counter's prescaler.
    const uint32_t ticks = tim_dead_time_ticks(dtg);
    const uint32_t expected_ns = static_cast<uint32_t>(
        static_cast<uint64_t>(ticks) * 4u * 1000000000ULL / apb2_tim_hz);
    const uint32_t measured_ns = (with_dead - baseline) * 1000u / cycles_per_us;
    print(serial, "  DTG ", hex(dtg), " = ", ticks, " tDTS at CKD/4 on a ",
          apb2_tim_hz / 1'000'000u, " MHz timer = ", expected_ns,
          " ns; the pads say ", measured_ns, " ns (the polling loop's own ",
          baseline * 1000u / cycles_per_us, " ns subtracted)", crlf);
    bench.verdict("THE DEAD TIME IS THE DATASHEET'S FORMULA IN NANOSECONDS: the gap "
                  "between the pad falling and its complement rising is DTG's decoded "
                  "tick count times tDTS, within five per cent",
                  measured_ns + expected_ns / 20u > expected_ns &&
                      measured_ns < expected_ns + expected_ns / 20u);
    bench.verdict("the driver's own decode of that code agrees",
                  Pair::dead_time_ticks() == ticks && tim_dead_time_ticks(0xFF) == 1008u);
    bench.verdict("and the search for a code always rounds UP, never short",
                  tim_dead_time_ticks(tim_dead_time_code(300)) >= 300u &&
                      tim_dead_time_code(2000) == 0xFFu);

    // The pair is a pair: never both high.
    uint32_t both = 0, neither = 0;
    for (uint32_t i = 0; i < 200000u; ++i) {
        const bool a = AdvPad::read();
        const bool b = AdvnPad::read();
        if (a && b) {
            ++both;
        }
        if (!a && !b) {
            ++neither;
        }
    }
    print(serial, "  200000 reads of the pair: both high ", both, " times, both low ",
          neither, crlf);
    bench.verdict("THE TWO OUTPUTS ARE NEVER HIGH TOGETHER - that is what the dead time "
                  "is for - and the samples that catch them both low are the dead time "
                  "itself",
                  both == 0u && neither > 0u);
    quiet_everything();
}

// =============================================================================
// l - the break: MOE cleared, the outputs parked, and the way back
// =============================================================================
void tl_break() {
    quiet_everything();
    AdvPad::claim();
    Adv::init();
    const uint16_t psc = static_cast<uint16_t>(apb2_tim_hz / 1000u / 10000u - 1u);
    (void)Adv::configure({.prescaler = psc, .period = 999, .auto_reload_preload = true});
    (void)Adv::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500});
    // OSSI drives the outputs to their IDLE level while MOE is clear, so
    // the break is visible on the pad instead of leaving it floating.
    (void)Adv::break_dead_time({.main_output_enable = true, .off_state_idle = true});
    Adv::enable(true);
    (void)delay_us(clock, 20000);

    const uint32_t before = sample_adv(pad_samples);
    Adv::clear_flags(Adv::break_flag);
    (void)Adv::break_event();
    (void)delay_us(clock, 20000);
    const bool moe_gone = !Adv::main_output();
    const bool flagged = Adv::flag(Adv::break_flag);
    const uint32_t after = sample_adv(pad_samples);
    print(serial, "  100000 reads: ", before, " high before the break, ", after,
          " after; MOE ", moe_gone ? "cleared" : "STILL SET", ", BIF ",
          flagged ? "raised" : "clear", crlf);
    bench.verdict("A SOFTWARE BREAK CLEARS MOE AND PARKS THE OUTPUT: with OSSI set the "
                  "pad holds its idle level and the waveform is gone",
                  near(before, pad_samples / 2u, pad_samples / 33u) && after == 0u &&
                      moe_gone && flagged);

    // The way back is one bit, and the waveform comes straight back.
    Adv::clear_flags(Adv::break_flag);
    (void)Adv::main_output(true);
    (void)delay_us(clock, 20000);
    const uint32_t back = sample_adv(pad_samples);
    bench.verdict("and raising MOE again is the whole of the way back",
                  Adv::main_output() && near(back, pad_samples / 2u, pad_samples / 33u));

    // With AOE the outputs re-arm themselves at the next update event.
    Adv::clear_flags(Adv::break_flag);
    (void)Adv::break_dead_time({.main_output_enable = true, .automatic_output_enable = true,
                                .off_state_idle = true});
    (void)delay_us(clock, 1000);
    (void)Adv::break_event();
    const bool moe_after_break = Adv::main_output();
    uint32_t waited = 0;
    const uint32_t deadline = Ticker::ticks();
    while (!Adv::main_output() && within(deadline, 20)) {
        ++waited;
    }
    const bool re_armed = Adv::main_output();
    print(serial, "  with AOE set: MOE ", moe_after_break ? "still up" : "down",
          " right after the break, and back ", re_armed ? "up" : "STILL DOWN", " after ",
          waited, " reads; BIF ", Adv::flag(Adv::break_flag) ? "standing" : "clear", crlf);
    bench.verdict("WITH AOE SET THE OUTPUTS RE-ARM THEMSELVES at the next update event - "
                  "the cycle-by-cycle regulation the chapter describes, and the shape "
                  "the errata says a SYSTEM break must not be allowed to take, which is "
                  "why TimBreakDeadTime leaves AOE clear",
                  !moe_after_break && re_armed);
    bench.verdict("a timer with no break unit refuses the whole of it",
                  !Wave::break_event() && !Wave::main_output(true) &&
                      !Wave::break_dead_time({}) && Wave::dead_time_ticks() == 0u);
    quiet_everything();
}

// =============================================================================
// m - the encoder, driven by two pads the program toggles
// =============================================================================

/// One quadrature step. THE PADS ARE THE TIMER'S OWN OUTPUTS, held in
/// alternate function and driven by the chapter's FORCED OUTPUT MODE
/// (17.3.7): OCxREF follows CCMRx and not the counter, so the program
/// puts a level on the pin through the timer's own output stage - the
/// only stimulus a capture input has on a board with no wire.
void quad_write(bool a, bool b) {
    (void)Quad::output_mode(0, a ? TimOutputMode::force_active : TimOutputMode::force_inactive);
    (void)Quad::output_mode(1, b ? TimOutputMode::force_active : TimOutputMode::force_inactive);
    (void)delay_us(clock, 2);
}

void quad_turn(bool forward, uint8_t cycles) {
    for (uint8_t i = 0; i < cycles; ++i) {
        if (forward) {
            quad_write(true, false);
            quad_write(true, true);
            quad_write(false, true);
            quad_write(false, false);
        } else {
            quad_write(false, true);
            quad_write(true, true);
            quad_write(true, false);
            quad_write(false, false);
        }
    }
}

/// THE QUESTION THIS LETTER ANSWERS FIRST: does a pad in GPIO OUTPUT mode
/// reach a timer's alternate-function INPUT, the way it reaches the EXTI?
/// Channel 1 of TIM4 is made a plain capture, the pad is put in output
/// mode with the timer's AF nibble already selected, and the program
/// toggles it - the count of captures is the answer.
uint32_t captures_from_a_gpio_output() {
    Quad::init();
    EncA::drive(false);
    (void)Quad::configure({.prescaler = 0, .period = 0xFFFFu});
    (void)Quad::capture_channel(0, {.select = TimChannelSelect::direct,
                                    .polarity = TimCapturePolarity::both});
    Quad::enable(true);
    Quad::clear_flags(Quad::all_flags);
    uint32_t seen = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        EncA::set();
        (void)delay_us(clock, 5);
        if (Quad::flag(Quad::compare_flag(0))) {
            ++seen;
            (void)Quad::compare(0);
        }
        EncA::clear();
        (void)delay_us(clock, 5);
        if (Quad::flag(Quad::compare_flag(0))) {
            ++seen;
            (void)Quad::compare(0);
        }
    }
    Quad::release();
    EncA::release();
    return seen;
}

/// TimEncoder's setup, then the two channels put BACK into their output
/// face. CCxS chooses what the CAPTURE UNIT reads; the quadrature
/// interface reads TI1FP1 and TI2FP2, which come off the pads before that
/// multiplexer - so a channel can drive its pad and be counted on it at
/// the same time, and that is the whole trick of this letter.
bool arm_encoder(TimSlaveMode mode, uint32_t period) {
    const bool up = TimEncoder<Quad>::setup({.mode = mode}, period);
    (void)Quad::output_channel(0, {.mode = TimOutputMode::force_inactive, .preload = false});
    (void)Quad::output_channel(1, {.mode = TimOutputMode::force_inactive, .preload = false});
    quad_write(false, false);
    return up;
}

void tm_encoder() {
    quiet_everything();

    const uint32_t from_gpio = captures_from_a_gpio_output();
    print(serial, "  sixteen edges written on a pad in GPIO OUTPUT mode, its AF nibble "
          "already naming the timer: ", from_gpio, " captures", crlf);
    bench.verdict("A PAD IN GPIO OUTPUT MODE DOES NOT REACH A TIMER'S INPUT, though it "
                  "does reach the EXTI: the alternate-function input multiplexer is "
                  "opened by MODER, not by the AF nibble alone - so a capture's only "
                  "wireless stimulus is the timer's OWN output stage",
                  from_gpio == 0u);

    // The stimulus that does work: both channels as OUTPUTS in alternate
    // function, driven by forced output mode, and the encoder reading the
    // pads back.
    Quad::init();
    (void)Quad::configure({.prescaler = 0, .period = 0xFFFFu});
    (void)Quad::output_channel(0, {.mode = TimOutputMode::force_inactive, .preload = false});
    (void)Quad::output_channel(1, {.mode = TimOutputMode::force_inactive, .preload = false});
    EncA::claim();
    EncB::claim();
    (void)delay_us(clock, 10);
    const bool pads_low = !EncA::read() && !EncB::read();
    quad_write(true, false);
    const bool pad_a_follows = EncA::read() && !EncB::read();
    quad_write(false, false);
    bench.verdict("FORCED OUTPUT MODE IS THE PROGRAM'S HAND ON THE PAD: OCxREF follows "
                  "CCMRx and not the counter, and the pad follows OCxREF",
                  pads_low && pad_a_follows);

    using Knob = TimEncoder<Quad>;
    const bool up = arm_encoder(TimSlaveMode::encoder3, 0xFFFFu);
    Knob::set_count(0x8000u);
    quad_turn(true, 10);
    const uint32_t forward = Knob::count();
    const bool forward_dir = !Knob::reversing();
    quad_turn(false, 10);
    const uint32_t back = Knob::count();
    const bool back_dir = Knob::reversing();
    print(serial, "  x4 counting: ten quadrature cycles forward took the counter from "
          "0x8000 to ", forward, ", ten back to ", back, crlf);
    bench.verdict("THE QUADRATURE INTERFACE COUNTS THE PADS THE TIMER ITSELF DRIVES: ten "
                  "cycles the program wrote are forty counts in encoder mode 3, and the "
                  "same ten backwards undo them exactly",
                  up && forward == 0x8000u + 40u && back == 0x8000u);
    bench.verdict("and CR1.DIR is the SILICON's answer to which way the shaft turned",
                  forward_dir && back_dir);

    // Mode 1 counts one track's edges only: half as many.
    (void)arm_encoder(TimSlaveMode::encoder1, 0xFFFFu);
    Knob::set_count(0x8000u);
    quad_turn(true, 10);
    const uint32_t half = Knob::count() - 0x8000u;
    (void)arm_encoder(TimSlaveMode::encoder2, 0xFFFFu);
    Knob::set_count(0x8000u);
    quad_turn(true, 10);
    const uint32_t other_half = Knob::count() - 0x8000u;
    print(serial, "  the same ten cycles: ", half, " counts in encoder mode 1, ",
          other_half, " in mode 2", crlf);
    bench.verdict("encoder modes 1 and 2 count ONE track's edges - twenty counts where "
                  "mode 3 makes forty",
                  half == 20u && other_half == 20u);

    // The counter runs modulo ARR in both directions.
    (void)arm_encoder(TimSlaveMode::encoder3, 39);
    Knob::set_count(0);
    quad_turn(true, 10);
    const uint32_t wrapped = Knob::count();
    Knob::set_count(0);
    quad_turn(false, 1);
    const uint32_t under = Knob::count();
    print(serial, "  with ARR = 39 a full forty counts came back to ", wrapped,
          ", and four counts below zero to ", under, crlf);
    bench.verdict("the counter runs modulo the auto-reload in BOTH directions - an "
                  "underflow lands at ARR",
                  wrapped == 0u && under == 36u);
    quiet_everything();
}

// =============================================================================
// n - the meter tasks against the ruler
// =============================================================================
void tn_meters() {
    quiet_everything();

    // The interval meter on the crystal: the same source letter g weighed
    // with the raw verbs, through the task an application would use.
    RtcDomain::pwr_bus_clock(true);
    (void)RtcDomain::unlock(true);
    RtcDomain::lse_enable(true);
    const bool lse = RtcDomain::lse_wait_ready();
    Meter::init();
    (void)Meter::input4_source(Tim5Input4::lse);
    using Crystal = TimIntervalMeter<Meter, 3>;
    const bool up = Crystal::setup(0, 0, TimCapturePolarity::rising,
                                   TimCapturePrescaler::every8);
    Crystal::restart();
    uint64_t total = 0;
    uint32_t taken = 0;
    const uint32_t t0 = Ticker::ticks();
    while (taken < 1024u && Ticker::ticks() - t0 < 1000u) {
        if (Meter::flag(Crystal::capture_flag)) {
            if (auto d = Crystal::interval()) {
                total += *d;
                ++taken;
            }
        }
    }
    const uint32_t mhz = taken == 0u ? 0u
        : static_cast<uint32_t>(static_cast<uint64_t>(apb1_tim_hz) * 8u * 1000u * taken / total);
    print(serial, "  TimIntervalMeter over ", taken, " captures of eight crystal periods: ",
          mhz / 1000u, ".", mhz % 1000u, " Hz", crlf);
    bench.verdict("THE INTERVAL METER IS THE TASK AN APPLICATION FEEDS A MeterLatch "
                  "FROM: its captures weigh the crystal to the same hundred parts per "
                  "million the raw verbs did",
                  lse && up && mhz > 32'764'000u && mhz < 32'772'000u);
    bench.verdict("and it says nothing at all on the first edge, there being no interval "
                  "yet",
                  (Crystal::restart(), !Crystal::interval().has_value()));

    // A TIMER CAPTURING ITS OWN OUTPUT. Channel 1 drives the pad with a
    // PWM at one microsecond a tick; channel 2 watches the SAME input
    // through the INDIRECT mapping (IC2 = TI1FP2, the pairing PWM input
    // mode is built on) and captures the FALLING edge - so what comes back
    // is the high time, and the whole capture path is exercised with
    // nothing attached. The rising edge would be no measurement at all:
    // the waveform is locked to the counter that times it, so its period
    // IS the counter's own wrap and every rising capture reads the same.
    quiet_everything();
    Wave::init();
    const uint16_t psc = static_cast<uint16_t>(apb1_tim_hz / 1'000'000u - 1u);   // 1 us a tick
    (void)Wave::configure({.prescaler = psc, .period = 999, .auto_reload_preload = true});
    (void)Wave::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 300});
    WavePad::claim();
    (void)Wave::capture_channel(1, {.select = TimChannelSelect::indirect,
                                    .polarity = TimCapturePolarity::falling});
    Wave::enable(true);
    uint32_t read_back[3] = {};
    const uint32_t asked[3] = {300, 700, 120};
    bool captured = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Wave::set_compare(0, asked[i]);
        (void)delay_us(clock, 3000);
        Wave::clear_flags(Wave::all_flags);
        const uint32_t c = wait_capture<Wave>(1, 20);
        if (c == 0xFFFFFFFFUL) {
            captured = false;
            break;
        }
        read_back[i] = c;
    }
    print(serial, "  a 1000 us PWM captured by its OWN timer's second channel through "
          "the indirect mapping: high times of ", asked[0], "/", asked[1], "/", asked[2],
          " us read back as ", read_back[0], "/", read_back[1], "/", read_back[2], crlf);
    bench.verdict("A TIMER SEES ITS OWN OUTPUT PAD ON ITS INPUT PATH: channel 2 in the "
                  "INDIRECT mapping - PWM input mode's own pairing - captures the "
                  "waveform channel 1 is driving, and follows its duty to the "
                  "microsecond",
                  captured && near(read_back[0], asked[0], 2u) &&
                      near(read_back[1], asked[1], 2u) && near(read_back[2], asked[2], 2u));

    // The slave controller's half of PWM input mode: the counter is
    // reinitialized on TI1's rising edge, so the same capture is measured
    // from the edge and not from the counter's own origin.
    (void)Wave::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::ti1});
    (void)Wave::set_compare(0, 300);
    (void)delay_us(clock, 3000);
    Wave::clear_flags(Wave::all_flags);
    const uint32_t width = wait_capture<Wave>(1, 20);
    print(serial, "  with the slave controller reset on the rising edge, the falling "
          "capture reads ", width, " us of high time (300 asked)", crlf);
    bench.verdict("AND THE SLAVE CONTROLLER'S RESET IS THE OTHER HALF: TI1FP1 "
                  "reinitializes the counter, which is what puts both of PWM input "
                  "mode's readings on one origin",
                  width != 0xFFFFFFFFUL && near(width, 300u, 2u));
    quiet_everything();
}

// =============================================================================
// o - the vectors: four lines on one timer, one line for two timers
// =============================================================================
void to_vectors() {
    quiet_everything();
    print(serial, "  TIM1's four vectors are ", static_cast<uint8_t>(Adv::irq()), " (up), ",
          static_cast<uint8_t>(Adv::cc_irq()), " (cc), ",
          static_cast<uint8_t>(Adv::break_irq()), " (break), ",
          static_cast<uint8_t>(Adv::trigger_irq()), " (trigger/commutation); TIM9 rides ",
          static_cast<uint8_t>(Small2::irq()), " and TIM11 rides ",
          static_cast<uint8_t>(Small1::irq()), crlf);
    bench.verdict("the small timers SHARE the advanced-control timer's break, update and "
                  "trigger lines - the reserve derives each name from what is present",
                  Small2::irq() == Adv::break_irq() && Small1::irq() == Adv::trigger_irq() &&
                      Adv::irq() != Adv::cc_irq());

    Adv::init();
    (void)Adv::configure({.prescaler = 0, .period = 99});
    (void)Adv::output_channel(0, {.mode = TimOutputMode::frozen, .compare = 50,
                                  .enable = false});
    (void)Adv::break_dead_time({.main_output_enable = true});
    Adv::interrupts(Adv::update_interrupt | Adv::compare_interrupt(0) |
                        Adv::break_interrupt | Adv::trigger_interrupt |
                        Adv::commutation_interrupt,
                    true);
    Nvic::enable(Adv::irq());
    Nvic::enable(Adv::cc_irq());
    Nvic::enable(Adv::break_irq());
    Nvic::enable(Adv::trigger_irq());
    reset_counters();
    Adv::enable(true);
    (void)delay_us(clock, 200);
    Adv::enable(false);
    const uint16_t ups = adv_update, ccs = adv_cc;
    (void)Adv::break_event();
    (void)delay_us(clock, 20);
    (void)Adv::trigger_event();
    (void)delay_us(clock, 20);
    (void)Adv::commutation_event();
    (void)delay_us(clock, 20);
    print(serial, "  200 us of a 100-tick TIM1: ", ups, " update and ", ccs,
          " compare calls on two different vectors; the break vector ran ", adv_brk,
          " time(s) with flags ", hex(adv_brk_flags), ", the trigger/commutation vector ",
          adv_trg, crlf);
    bench.verdict("AN ADVANCED-CONTROL TIMER'S FOUR EVENT GROUPS REACH FOUR VECTORS, and "
                  "each body serves only the flags its own vector answers for - the "
                  "trigger and the commutation share the fourth",
                  ups > 0u && ccs > 0u && adv_brk == 1u &&
                      adv_brk_flags == Adv::break_flag && adv_trg == 2u);

    // The other half of the sharing: two owners on one line, each body
    // answering for its own.
    Small2::init();
    (void)Small2::configure({.prescaler = 0, .period = 999});
    Small2::interrupts(Small2::update_interrupt, true);
    Nvic::enable(Small2::irq());
    reset_counters();
    Small2::enable(true);
    (void)delay_us(clock, 200);
    Small2::enable(false);
    (void)Adv::break_event();
    (void)delay_us(clock, 50);
    print(serial, "  on the SHARED line: ", small2_entries, " TIM9 update calls and ",
          adv_brk, " TIM1 break call", crlf);
    bench.verdict("one vector, two owners, two bodies: each answered for its own timer "
                  "and left the other's flags alone",
                  small2_entries > 0u && adv_brk == 1u);

    // A flag whose interrupt is not enabled stands for a poller, which is
    // what makes every measurement above possible with no handler at all.
    Adv::interrupts(Adv::update_interrupt, false);
    Adv::clear_flags(Adv::all_flags);
    reset_counters();
    Adv::enable(true);
    (void)delay_us(clock, 200);
    Adv::enable(false);
    const bool standing = Adv::flag(Adv::update_flag);
    bench.verdict("a flag with no interrupt enabled is left STANDING for a poller - the "
                  "ISR body clears only what it served",
                  standing && adv_update == 0u);

    // rc_w0, the one flag register in this stratum that is not cleared by
    // writing ones.
    Adv::regs().SR = 0xFFFFFFFFUL;
    const bool survived_ones = Adv::flag(Adv::update_flag);
    Adv::clear_flags(Adv::update_flag);
    const bool cleared_by_zero = !Adv::flag(Adv::update_flag);
    bench.verdict("AND THE STATUS REGISTER IS rc_w0: a word of ones written over a "
                  "standing flag leaves it exactly where it was, and a zero at its bit "
                  "is what clears it",
                  survived_ones && cleared_by_zero);
    quiet_everything();
}

void banner() {
    print(serial, crlf, "test_stm32f4_tim - the time base, the channels, the internal "
          "triggers, the option registers, the break unit and the four vectors", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#elif defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

// TIM1's four lines, three of them SHARED with a small timer: each
// handler calls one body per owner and each answers for its own flags.
extern "C" void TIM1_UP_TIM10_IRQHandler() {
    if (Adv::isr(Adv::vector_flags(Adv::irq())) != 0u) {
        adv_update = adv_update + 1u;
    }
}
extern "C" void TIM1_CC_IRQHandler() {
    if (Adv::isr(Adv::vector_flags(Adv::cc_irq())) != 0u) {
        adv_cc = adv_cc + 1u;
    }
}
extern "C" void TIM1_BRK_TIM9_IRQHandler() {
    const uint32_t mine = Adv::isr(Adv::vector_flags(Adv::break_irq()));
    if (mine != 0u) {
        adv_brk = adv_brk + 1u;
        adv_brk_flags = mine;
    }
    if (Small2::isr() != 0u) {
        small2_entries = small2_entries + 1u;
    }
}
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler() {
    if (Adv::isr(Adv::vector_flags(Adv::trigger_irq())) != 0u) {
        adv_trg = adv_trg + 1u;
    }
    if (Small1::isr() != 0u) {
        small1_entries = small1_entries + 1u;
    }
}

int main() {
    // What the silicon held before a line of this program ran: the gates
    // are read out of RCC, which does not open them, and a timer behind a
    // closed gate answers zero to every read.
    boot.apb1_gate = brio::Rcc::apb1_clock(RCC_APB1ENR_TIM3EN);
    boot.apb2_gate = brio::Rcc::apb2_clock(RCC_APB2ENR_TIM1EN);
    boot.cr1 = Wave::regs().CR1;
    boot.cr2 = Wave::regs().CR2;
    boot.smcr = Wave::regs().SMCR;
    boot.dier = Wave::regs().DIER;
    boot.sr = Wave::regs().SR;
    boot.ccer = Wave::regs().CCER;
    boot.arr = Wave::regs().ARR;
    boot.opt = Wave::regs().OR;
    boot.rtcpre = brio::RtcDomain::hse_divider();
    boot.timpre = brio::Rcc::timpre();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the reset state, the geometry, and what a small timer refuses",
                 ta_reset_state);
    bench.letter('b', "the time base: TIMxCLK measured, TIMPRE, the shadow registers",
                 tb_time_base);
    bench.letter('c', "the internal trigger: one timer counting another, exactly",
                 tc_internal_trigger);
    bench.letter('d', "the gated counter: a duty cycle measured with no pad", td_gated_duty);
    bench.letter('e', "a PWM output read back on its own pad", te_pwm_pad);
    bench.letter('f', "the two 32-bit counters, and the refusals at the width", tf_widths);
    bench.letter('g', "the LSE and the LSI weighed through TIM5's option register",
                 tg_option_lse);
    bench.letter('h', "HSE_RTC through TIM11's option register (RESETS THE BACKUP DOMAIN)",
                 th_option_hse, false);
    bench.letter('i', "one-pulse mode: the width on the pad", ti_one_pulse);
    bench.letter('j', "the repetition counter's update rate", tj_repetition);
    bench.letter('k', "the complementary pair and the dead time in nanoseconds",
                 tk_dead_time);
    bench.letter('l', "the break: MOE cleared, the outputs parked, and the way back",
                 tl_break);
    bench.letter('m', "the encoder, on two pads the timer drives itself", tm_encoder);
    bench.letter('n', "the meter tasks against the ruler", tn_meters);
    bench.letter('o', "the vectors: four lines on one timer, one line for two timers",
                 to_vectors);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
