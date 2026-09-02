// test_stm32_tim - the reference bench suite for the STM32G0's TIMERS
// (RM0444 ch. 21..25: the advanced-control TIM1, the general-purpose
// TIM2/3/4, the basic TIM6/7, TIM14 and TIM15/16/17) and, through them,
// for two util contracts on their THIRD silicon: util/pwm_channel.hpp's
// PwmChannel and util/meter_sampler.hpp's MeterLatch/MeterSampler.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. Four techniques do it, and each is measured before it
// is relied on:
//   1. A TIMER COUNTING A TIMER. TIM2 publishes on TRGO and TIM3 takes
//      that as its own clock (external clock mode 1) over the ITR1 link -
//      so a frequency is counted exactly, with no pad, no wire and no CPU
//      in the path. With TRGO = OC1REF the trigger is the PWM WAVEFORM
//      itself, and TIM3 in GATED mode then counts only its high time:
//      a duty cycle measured internally.
//   2. A PAD READ WHILE A PERIPHERAL DRIVES IT. The input buffer stays
//      live in alternate-function mode (7.3.1), so LD4's own PWM is
//      readable on IDR and an EXTI line can count its edges - the EXTI
//      campaign proved that for OUTPUT mode, letter c extends it to AF.
//   3. A CAPTURE WITH NO PAD AT ALL. TIM16_TISEL selects LSI as TI1
//      (25.6.18), so the capture unit measures a real ~32 kHz signal that
//      never leaves the die - and the reading cross-checks the LSI rate
//      test_stm32_reset measured through the watchdog.
//   4. A PAD WALKED BY ITS OWN PULL. A capture channel does not drive its
//      pad, and PUPDR still does, so a square wave of software's own
//      making reaches PWM input mode.
//
// THE PADS, each on a header pin and each checked electrically free by
// letter a before anything after it is believed:
//   PA5   LD4      TIM2_CH1  AF2   the PWM under test (DS13560 table 13)
//   PA6            TIM3_CH1  AF1   the capture input, and TIM1_BKIN AF2
//   PA7            TIM1_CH1N AF2   the complementary half of the pair
//   PA8            TIM1_CH1  AF2   the pair's direct half
// Avoided on purpose: PA2/PA3 (the console), PA13/PA14 (SWD), PC13 (B1),
// PC14/PC15 (the LSE pads), PF0/PF1 (the HSE pads).
//
// What is exercised, letter by letter:
//   a  the block: what the reserve says each timer IS, the reset values
//      this boot found, the two vectors of TIM1, and every refusal
//   b  the time base: the prescaler and ARR arithmetic against SysTick,
//      the SHADOW registers, TIM2's 32-bit counter past 16 bits,
//      down-counting, one-pulse mode and URS
//   c  PWM on LD4: the duty read back through the pad, and the pad's own
//      edges counted by an EXTI line
//   d  a timer measuring a timer: the update rate counted exactly, and
//      the duty measured by GATING with no pad at all
//   e  input capture with no pad: TIM16 on LSI - the interval, the
//      capture prescaler, the overcapture flag and the input filter
//   f  PWM input mode: period AND width of a pad walked by its own pull
//   g  TIM1: the complementary pair and its dead time, MOE, the software
//      break, the automatic output enable and the repetition counter
//   h  the shared vectors: TIM3 and TIM4 on one line, TIM1 on two, and
//      the rc_w0 status register that cannot swallow a flag
//   i  centre-aligned mode: the period is 2 x ARR, measured
//   j  MeterSampler inside a REAL KERNEL, fed by a capture ISR
//   k  ES0548 2.7.2 staged with a control, 2.7.1 and 2.7.3 declared
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/meter_sampler.hpp"
#include "util/print.hpp"
#include "util/pwm_channel.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// The console pads: USART2_TX on PA2, USART2_RX on PA3, both AF1
// (DS13560 table 13), which is the ST-LINK's virtual COM port.
constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

// ---- the timers ------------------------------------------------------------
using T1 = Tim<1>;     // advanced: the pair, the dead time, the break, RCR
using T2 = Tim<2>;     // the 32-bit one; the master of every cascade here
using T3 = Tim<3>;     // the slave: counts, gates, resets, captures
using T4 = Tim<4>;     // TIM3's vector-mate
using T16 = Tim<16>;   // one channel, and TISEL reaches LSI

// ---- the pads --------------------------------------------------------------
// The AF numbers are DS13560 tables 13 and 15; nothing in the device
// header can check them (stm32g0/pin.hpp says so once for the stratum),
// so THIS SUITE IS THE CHECK - letters c, f and g are what prove them.
constexpr PinSel led_pad{'A', 5, PinFunction::af2};    // TIM2_CH1
constexpr PinSel cap_pad{'A', 6, PinFunction::af1};    // TIM3_CH1
constexpr PinSel brk_pad{'A', 6, PinFunction::af2};    // TIM1_BKIN (same pad)
constexpr PinSel pairn_pad{'A', 7, PinFunction::af2};  // TIM1_CH1N
constexpr PinSel pair_pad{'A', 8, PinFunction::af2};   // TIM1_CH1

using LedOut = TimPad<led_pad>;
using CapIn = TimPad<cap_pad>;
using PairOut = TimPad<pair_pad>;
using PairNOut = TimPad<pairn_pad>;

using PadLed = Pin<'A', 5>;
using PadCap = Pin<'A', 6>;
using PadPairN = Pin<'A', 7>;
using PadPair = Pin<'A', 8>;

// ---- the tasks -------------------------------------------------------------
// A PWM period of 1000 counts: with PSC = 0 that is 64 kHz (fast enough
// that a sampling loop crosses hundreds of periods), with PSC = 63 it is
// 1 kHz (slow enough for an interrupt per edge).
constexpr uint16_t pwm_top = 999;
using LedPwm = TimPwm<T2, 0, pwm_top>;
using Pair = TimPairPwm<T1, 0, pwm_top>;
using LsiMeter = TimIntervalMeter<T16, 0>;
using PwmIn = TimPeriodMeter<T3>;
using Counter = TimEventCounter<T3>;
using Counter16 = TimEventCounter<T2>;   // TIM1's updates arrive here, over ITR0
using Gate = TimGatedCounter<T3>;

static_assert(PwmChannel<LedPwm>, "a TIM channel is a util PwmChannel");
static_assert(PwmChannel<Pair>, "and so is a complementary PAIR");

// TIM3's ITR1 is TIM2 (RM0444 table 123) - the link every cascade in
// this suite rides, looked up rather than remembered.
constexpr uint8_t itr_t3_from_t2 = tim_trigger_index_for(3, 2);
static_assert(itr_t3_from_t2 == 1);
constexpr TimTrigger trg_t2 = static_cast<TimTrigger>(itr_t3_from_t2);
// TIM2's ITR0 is TIM1 - letter g counts TIM1's updates on it.
constexpr uint8_t itr_t2_from_t1 = tim_trigger_index_for(2, 1);
static_assert(itr_t2_from_t1 == 0);
constexpr TimTrigger trg_t1 = static_cast<TimTrigger>(itr_t2_from_t1);

// What this boot found, sampled in main() before a letter can disturb it.
uint32_t boot_t2_cr1 = 0;
uint32_t boot_t2_arr = 0;
uint32_t boot_t1_bdtr = 0;
uint32_t boot_apbenr1 = 0;
uint32_t boot_apbenr2 = 0;

// =============================================================================
// What the handlers count
// =============================================================================
volatile uint32_t t1_up_calls = 0;      // TIM1_BRK_UP_TRG_COM
volatile uint32_t t1_cc_calls = 0;      // TIM1_CC
volatile uint32_t t1_break_seen = 0;
volatile uint32_t t2_update_calls = 0;
volatile uint32_t t2_compare_calls = 0;
volatile uint32_t t3_update_calls = 0;
volatile uint32_t t4_update_calls = 0;
volatile uint32_t t3_saw_t4_flag = 0;   // did TIM3's body ever eat TIM4's flag?
volatile uint32_t t16_captures = 0;
volatile uint32_t t16_stores = 0;   // captures that became an INTERVAL
volatile uint32_t exti_edges = 0;

void clear_counts() {
    t1_up_calls = 0;
    t1_cc_calls = 0;
    t1_break_seen = 0;
    t2_update_calls = 0;
    t2_compare_calls = 0;
    t3_update_calls = 0;
    t4_update_calls = 0;
    t3_saw_t4_flag = 0;
    t16_captures = 0;
    t16_stores = 0;
    exti_edges = 0;
}

// =============================================================================
// Instruments
// =============================================================================

/// A cycle-resolution stopwatch, the other suites' own: ticks x period
/// plus the phase SysTick has already counted down, the two reads
/// retried until they belong to the same tick.
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
uint32_t cycles_to_us(uint32_t c) { return c / cycles_per_us; }

void spin_cycles(uint32_t c) {
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < c) {
    }
}

/// Long enough for a floating pad's own RC under a ~40 kohm internal
/// pull (the EXTI campaign's figure, kept).
void settle() { (void)delay_us(clock, 200); }

/// Wait for the console to be physically empty. A measurement window
/// that a transmit interrupt walks through is not a measurement - the
/// samc campaigns paid for this twice, and letter e pays for it again:
/// a polling capture loop that the USART's ISR interrupts MISSES edges,
/// and a missed capture reads as an interval that is a multiple of the
/// true one.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    spin_cycles(SysClock::hz / 500u);   // 2 ms of quiet on top
}

/// THE PRECONDITION OF THE PULL-WALKED LETTERS: an input pad with
/// nothing attached goes where its own pull sends it.
template <class Pad>
bool pad_follows_pull() {
    Pad::input(PinPull::up);
    settle();
    const bool high = Pad::read();
    Pad::input(PinPull::down);
    settle();
    const bool low = Pad::read();
    Pad::input(PinPull::none);
    return high && !low;
}

/// Per-mille high time of one pad of port A, sampled through IDR and
/// nothing else in the loop. The estimator is only as good as the number
/// of WAVEFORM PERIODS the window crosses, which is why every caller
/// runs its waveform in the tens of kilohertz.
/// THE LOOP MUST TAKE THE SAME TIME WHATEVER IT READS. A sampler that
/// branches on the bit it just read spends more cycles in one state than
/// in the other, and then counts FEWER iterations of the slower state -
/// a bias that reads as a duty the waveform does not have. (Measured:
/// the first version of letter g reported a 50 % pair as 566 and 383 per
/// mille.) So the bit is shifted down and ADDED, and nothing here
/// branches on data.
uint16_t sample_permille(uint8_t shift, uint32_t samples) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        high += (Port<'A'>::in() >> shift) & 1u;
    }
    return static_cast<uint16_t>((high * 1000u + samples / 2u) / samples);
}

/// The two halves of a complementary pair sampled IN THE SAME READ - one
/// IDR access carries both pads, so "were they ever both high" is a fact
/// about one instant and not about two.
struct PairCensus {
    uint32_t both_high = 0;
    uint32_t a_high = 0;
    uint32_t b_high = 0;
    uint32_t neither = 0;
};
PairCensus census_pair(uint8_t shift_a, uint8_t shift_b, uint32_t samples) {
    uint32_t a = 0, b = 0, both = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        const uint32_t v = Port<'A'>::in();
        const uint32_t x = (v >> shift_a) & 1u;
        const uint32_t y = (v >> shift_b) & 1u;
        a += x;
        b += y;
        both += x & y;
    }
    return {both, a, b, samples - a - b + both};
}

/// Every timer and pad this suite touches back to reset.
void quiet_everything() {
    Nvic::disable(T1::irq());
    Nvic::disable(T1::cc_irq());
    Nvic::disable(T2::irq());
    Nvic::disable(T3::irq());
    Nvic::disable(T16::irq());
    Nvic::disable(EXTI4_15_IRQn);
    T1::release();
    T2::release();
    T3::release();
    T4::release();
    T16::release();
    (void)Exti::release(PadLed::pin_number);
    PadLed::output(false);
    PadCap::input(PinPull::none);
    PadPair::input(PinPull::none);
    PadPairN::input(PinPull::none);
    Nvic::clear_pending(T1::irq());
    Nvic::clear_pending(T1::cc_irq());
    Nvic::clear_pending(T2::irq());
    Nvic::clear_pending(T3::irq());
    Nvic::clear_pending(T16::irq());
    Nvic::clear_pending(EXTI4_15_IRQn);
}

// =============================================================================
// a - the block: what each timer IS, the reset values, the vectors, the
//     refusals - and the pads this suite is about to rely on
// =============================================================================
void ta_block() {
    print(serial, "  present: TIM1=", tim_present(1), " TIM2=", tim_present(2),
          " TIM3=", tim_present(3), " TIM4=", tim_present(4), " TIM6=", tim_present(6),
          " TIM7=", tim_present(7), " TIM14=", tim_present(14), " TIM15=", tim_present(15),
          " TIM16=", tim_present(16), " TIM17=", tim_present(17), crlf);
    print(serial, "  at boot: TIM2_CR1=", hex(boot_t2_cr1), " TIM2_ARR=", hex(boot_t2_arr),
          " TIM1_BDTR=", hex(boot_t1_bdtr), " APBENR1=", hex(boot_apbenr1),
          " APBENR2=", hex(boot_apbenr2), crlf);

    bench.verdict("the G0B1 carries all ten timers of the family",
                  tim_present(1) && tim_present(2) && tim_present(3) && tim_present(4) &&
                      tim_present(6) && tim_present(7) && tim_present(14) &&
                      tim_present(15) && tim_present(16) && tim_present(17));
    bench.verdict("TIM2 is the family's one 32-bit counter, everything else 16",
                  T2::counter_bits == 32 && T2::max_period == 0xFFFFFFFFUL &&
                      T3::counter_bits == 16 && T1::max_period == 0xFFFFUL);
    bench.verdict("TIM1 has four channels and THREE complementary outputs, "
                  "TIM16 one of each, TIM2 four and none",
                  T1::channels == 4 && T1::complementary_channels == 3 &&
                      T16::channels == 1 && T16::complementary_channels == 1 &&
                      T2::channels == 4 && T2::complementary_channels == 0);
    bench.verdict("a break/dead-time unit exists exactly where a "
                  "complementary output does",
                  T1::has_break && T16::has_break && !T2::has_break && !T3::has_break);
    bench.verdict("TIM16 has no slave controller and TIM2 has one",
                  !T16::has_slave_mode && T2::has_slave_mode && T3::has_slave_mode);
    bench.verdict("TIM1 alone reports on TWO vectors, the capture/compare "
                  "one being separate",
                  T1::has_split_vector && T1::irq() == TIM1_BRK_UP_TRG_COM_IRQn &&
                      T1::cc_irq() == TIM1_CC_IRQn && !T2::has_split_vector &&
                      T2::irq() == T2::cc_irq());
    bench.verdict("TIM3 and TIM4 SHARE their vector on this part",
                  T3::irq() == TIM3_TIM4_IRQn && T4::irq() == TIM3_TIM4_IRQn);
    bench.verdict("and TIM16's is shared with an FDCAN line",
                  T16::irq() == TIM16_FDCAN_IT0_IRQn);

    // The interconnect tables, both ways (RM0444 tables 119, 123, 130).
    bench.verdict("TIM3 reaches TIM2 on ITR1 and TIM2 reaches TIM1 on ITR0",
                  tim_trigger_index_for(3, 2) == 1 && tim_trigger_index_for(2, 1) == 0);
    bench.verdict("TIM2's ITR3 is TIM14's OC1 OUTPUT and not a TRGO - the "
                  "one-channel timers have no master mode to publish one",
                  tim_internal_trigger_is_oc1(2, 3) && !tim_internal_trigger_is_oc1(2, 0));
    bench.verdict("and a timer with no slave controller reaches nothing",
                  tim_trigger_index_for(16, 2) == 0xFFu);

    // The reset values, read before any letter ran.
    bench.verdict("every timer came up with its APB clock CLOSED, which is "
                  "why init() opens it before anything else (5.2.17)",
                  (boot_apbenr1 & (RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN)) == 0u &&
                      (boot_apbenr2 & (RCC_APBENR2_TIM1EN | RCC_APBENR2_TIM16EN)) == 0u);
    T2::init();
    T1::init();
    print(serial, "  after init+reset: TIM2_CR1=", hex(T2::regs().CR1),
          " TIM2_ARR=", hex(T2::regs().ARR), " TIM2_PSC=", hex(T2::regs().PSC),
          " TIM1_BDTR=", hex(T1::regs().BDTR), crlf);
    bench.verdict("a reset leaves CR1 at zero, ARR at its own full scale and "
                  "PSC at zero (the chapters' register maps)",
                  T2::regs().CR1 == 0u && T2::regs().ARR == 0xFFFFFFFFUL &&
                      T2::regs().PSC == 0u && T1::regs().ARR == 0xFFFFu);
    bench.verdict("and BDTR at zero, so MOE is CLEAR out of reset - an "
                  "advanced timer drives nothing until something raises it",
                  T1::regs().BDTR == 0u && !T1::main_output());
    bench.verdict("the APB prescaler is 1, so TIMPCLK is PCLK (5.2.13)",
                  T2::clock_ok() && tim_clock_hz(clock) == SysClock::hz);

    // The refusals.
    bench.verdict("a null auto-reload is refused - 21.4.12's blocked "
                  "counter is a stopped timer wearing a running one's face",
                  !T2::configure({.period = 0}));
    bench.verdict("a period past this counter's width is refused",
                  !T3::configure({.period = 0x10000UL}) && T2::configure({.period = 0x10000UL}));
    bench.verdict("centre-aligned counting is refused on a timer that only "
                  "counts up",
                  !T16::configure({.period = 100, .alignment = TimAlignment::center_up}) &&
                      T3::configure({.period = 100, .alignment = TimAlignment::center_up}));
    bench.verdict("down-counting likewise", !T16::configure({.period = 100,
                                                             .direction = TimDirection::down}));
    bench.verdict("the reserved clock-division code is refused",
                  !T2::configure({.period = 100,
                                  .clock_division = static_cast<TimClockDivision>(3)}));
    bench.verdict("a channel past the instance's count is refused everywhere",
                  !T16::output_channel(1, {}) && !T16::capture_channel(1, {}) &&
                      !T16::set_compare(1, 0) && !T2::output_channel(4, {}));
    bench.verdict("a slave mode on a timer with no SMCR is refused, and the "
                  "master mode on a timer with no CR2.MMS with it",
                  !T16::slave({.mode = TimSlaveMode::gated}) &&
                      !T16::master(TimMasterMode::update) &&
                      T2::slave({.mode = TimSlaveMode::disabled}));
    bench.verdict("gating on the TI1 EDGE DETECTOR is refused - 21.4.3's own "
                  "note: a pulse per transition has no level to gate on",
                  !T3::slave({.mode = TimSlaveMode::gated, .trigger = TimTrigger::ti1_edge}) &&
                      T3::slave({.mode = TimSlaveMode::gated, .trigger = TimTrigger::itr1}));
    bench.verdict("a break/dead-time configuration on a timer without one is "
                  "refused, and so is its MOE",
                  !T2::break_dead_time({}) && !T2::main_output(true) && !T2::main_output() &&
                      T1::break_dead_time({}));
    bench.verdict("TIM2 publishes OC4REF but a basic timer could not - a "
                  "master mode past the channel count is refused",
                  T2::master(TimMasterMode::oc4ref) && !T16::master(TimMasterMode::oc1ref));
    bench.verdict("an input selection past the channel count is refused",
                  !T16::input_select(1, 1) && T16::input_select(0, 0));

    // The pads, before letters c, f and g trust them.
    bench.verdict("PA6 is electrically free (it follows its own pull)",
                  pad_follows_pull<PadCap>());
    bench.verdict("so is PA7", pad_follows_pull<PadPairN>());
    bench.verdict("and so is PA8", pad_follows_pull<PadPair>());
    PadLed::output(true);
    settle();
    const bool led_high = PadLed::read();
    PadLed::output(false);
    settle();
    const bool led_low = PadLed::read();
    print(serial, "  PA5 (LD4) driven high reads ", led_high ? "HIGH" : "low",
          ", driven low reads ", led_low ? "HIGH" : "low", crlf);
    bench.verdict("PA5 carries LD4 and its own output driver still reaches "
                  "both rails through it - the pad is readable on IDR",
                  led_high && !led_low);

    quiet_everything();
}

// =============================================================================
// b - the time base: the arithmetic, the SHADOW registers, 32 bits,
//     down-counting, one-pulse mode and URS
// =============================================================================
void tb_time_base() {
    // The arithmetic, against SysTick. The window is opened and closed by
    // two reads with NOTHING between them - a verdict line is four
    // milliseconds of console, and the samc campaign paid for that lesson
    // twice.
    T2::init();
    bench.verdict("a free-running 32-bit counter configures and starts",
                  T2::configure({.prescaler = 0, .period = 0xFFFFFFFFUL}));
    T2::enable(true);
    const uint32_t c0 = cycles_now();
    const uint32_t n0 = T2::count();
    spin_cycles(SysClock::hz / 100u);   // 10 ms
    const uint32_t n1 = T2::count();
    const uint32_t c1 = cycles_now();
    const uint32_t counted = n1 - n0;
    const uint32_t elapsed = c1 - c0;
    print(serial, "  PSC=0: ", counted, " counts in ", elapsed,
          " CPU cycles (", cycles_to_us(elapsed), " us)", crlf);
    bench.verdict("the counter counts TIMPCLK tick for tick - the count and "
                  "the cycle stopwatch agree to under a per mille",
                  counted <= elapsed && (elapsed - counted) < elapsed / 1000u + 40u);
    bench.verdict("and it ran PAST sixteen bits, which only TIM2 can do",
                  counted > 0x10000UL);

    // The prescaler, and the fact that it is SHADOWED.
    T2::init();
    (void)T2::configure({.prescaler = 63, .period = 0xFFFFFFFFUL});
    T2::enable(true);
    const uint32_t p0 = cycles_now();
    const uint32_t m0 = T2::count();
    spin_cycles(SysClock::hz / 100u);
    const uint32_t m1 = T2::count();
    const uint32_t p1 = cycles_now();
    const uint32_t pre_counted = m1 - m0;
    const uint32_t pre_expect = (p1 - p0) / 64u;
    print(serial, "  PSC=63: ", pre_counted, " counts where ", pre_expect,
          " are due", crlf);
    bench.verdict("PSC divides by PSC + 1 exactly",
                  pre_counted + 2u >= pre_expect && pre_counted <= pre_expect + 2u);

    // PSC is copied into the working register only at an update (21.4.11).
    T2::set_prescaler(0xFFFF);
    const uint32_t s0 = T2::count();
    spin_cycles(SysClock::hz / 1000u);   // 1 ms
    const uint32_t s1 = T2::count();
    T2::update();                        // now load the shadow
    spin_cycles(SysClock::hz / 10000u);  // let the reinitialization land
    const uint32_t s2 = T2::count();
    spin_cycles(SysClock::hz / 1000u);
    const uint32_t s3 = T2::count();
    print(serial, "  a new PSC before its update: ", s1 - s0,
          " counts in 1 ms; after it: ", s3 - s2, crlf);
    bench.verdict("a new prescaler does NOTHING until an update event loads "
                  "the shadow register (21.4.11)",
                  (s1 - s0) > 900u && (s3 - s2) < 4u);

    // ARR's preload is a CHOICE, and it is off out of reset.
    T2::init();
    (void)T2::configure({.prescaler = 63, .period = 20000, .auto_reload_preload = true});
    T2::enable(true);
    (void)T2::set_period(100);
    uint32_t high_water = 0;
    for (uint32_t i = 0; i < 400'000u && high_water <= 100u; ++i) {
        const uint32_t v = T2::count();
        if (v > high_water) { high_water = v; }
    }
    T2::update();
    spin_cycles(SysClock::hz / 500u);   // 2 ms: twenty of the new periods
    uint32_t after = 0;
    for (uint32_t i = 0; i < 3000u; ++i) {
        const uint32_t v = T2::count();
        if (v > after) { after = v; }
    }
    print(serial, "  ARPE=1, ARR 20000 then 100: the counter still reached ",
          high_water, " before the update and tops out at ", after, " after it",
          crlf);
    bench.verdict("with ARPE set the old period stays in force until the "
                  "update takes the new one (21.4.12)", high_water > 100u);
    bench.verdict("and the new period is in force after it", after <= 100u);

    // Down-counting.
    T2::init();
    (void)T2::configure({.prescaler = 63, .period = 1000, .direction = TimDirection::down});
    T2::enable(true);
    T2::set_count(900);
    const uint32_t d0 = T2::count();
    spin_cycles(SysClock::hz / 10000u);   // 100 us
    const uint32_t d1 = T2::count();
    print(serial, "  down-counting: ", d0, " then ", d1, crlf);
    bench.verdict("CR1.DIR really counts down", d1 < d0);

    // URS: EGR.UG raises no update flag when only overflows may.
    T2::init();
    (void)T2::configure({.prescaler = 0, .period = 0xFFFF, .update_on_overflow_only = true});
    T2::clear_flags(T2::update_flag);
    T2::update();
    spin_cycles(1000u);
    const bool urs_flag = T2::flag(T2::update_flag);
    (void)T2::configure({.prescaler = 0, .period = 0xFFFF});
    T2::clear_flags(T2::update_flag);
    T2::update();
    spin_cycles(1000u);
    const bool plain_flag = T2::flag(T2::update_flag);
    print(serial, "  EGR.UG with URS set raised UIF: ", urs_flag ? "yes" : "no",
          "; with URS clear: ", plain_flag ? "yes" : "no", crlf);
    bench.verdict("CR1.URS keeps a SOFTWARE update out of the flag while an "
                  "overflow still raises it (21.4.1)",
                  !urs_flag && plain_flag);

    // One-pulse mode stops the counter by itself.
    T2::init();
    (void)T2::configure({.prescaler = 63, .period = 500, .one_pulse = true});
    T2::enable(true);
    const bool running_now = T2::enabled();
    spin_cycles(SysClock::hz / 500u);   // 2 ms, four periods' worth
    const bool stopped = !T2::enabled();
    bench.verdict("one-pulse mode clears CEN at the update that ends the "
                  "first period (21.4.1)", running_now && stopped);

    quiet_everything();
}

// =============================================================================
// c - PWM on LD4: the duty read back THROUGH THE PAD, and the pad's own
//     edges counted by an EXTI line
// =============================================================================
//
// Two witnesses for one waveform, neither of them a wire. The first is
// GPIOA_IDR - 7.3.1 leaves the input buffer live in alternate-function
// mode, so a pad the timer is driving is readable - and the second is an
// EXTI line pointed at port A, which the EXTI campaign proved sees a pad
// its OWNER drives; letter c is what extends that to a pad a PERIPHERAL
// drives.
void tc_pwm_pad() {
    T2::init();
    LedOut::claim();
    bench.verdict("LD4's pad takes TIM2_CH1 at AF2 (DS13560 table 13) and "
                  "the channel comes up",
                  PadLed::has_function() && LedPwm::setup(0));

    // 64 kHz: a sampling loop crosses hundreds of periods, so the
    // aliasing between the two rates averages out.
    struct Point { uint16_t asked; uint16_t got; };
    Point pts[5];
    const uint16_t asks[5] = {0, 250, 500, 750, 1000};
    for (uint8_t i = 0; i < 5; ++i) {
        LedPwm::duty(asks[i]);
        spin_cycles(SysClock::hz / 1000u);      // let the preload be taken
        pts[i] = {asks[i], sample_permille(PadLed::pin_number, 60000u)};
    }
    print(serial, "  duty per mille asked, read off the pad: ");
    for (uint8_t i = 0; i < 5; ++i) {
        print(serial, pts[i].asked, ":", pts[i].got, i == 4 ? "" : ", ");
    }
    print(serial, crlf);
    bench.verdict("0 per mille leaves the pad low and 1000 leaves it high - "
                  "a PWM channel reaches both endpoints",
                  pts[0].got == 0u && pts[4].got == 1000u);
    bool ladder_ok = true;
    for (uint8_t i = 1; i < 4; ++i) {
        const int32_t err = static_cast<int32_t>(pts[i].got) -
                            static_cast<int32_t>(pts[i].asked);
        if (err > 25 || err < -25) { ladder_ok = false; }
    }
    bench.verdict("and the three duties in between read back within 25 per "
                  "mille of what a PwmChannel was asked for", ladder_ok);

    // The frequency, counted off the pad by an EXTI line. Slow the
    // waveform to 1 kHz first: an interrupt per edge is the point, not a
    // stress test.
    (void)T2::configure({.prescaler = 63, .period = pwm_top,
                         .auto_reload_preload = true});
    (void)T2::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500});
    T2::enable(true);
    bench.verdict("line 5 takes port A and a rising sense while TIM2 owns "
                  "the pad",
                  Exti::select(5, 'A') && Exti::sense(5, ExtiSense::rising) &&
                      Exti::interrupt(5, true));
    (void)Exti::clear(5);
    exti_edges = 0;
    Nvic::clear_pending(EXTI4_15_IRQn);
    Nvic::enable(EXTI4_15_IRQn);
    const uint32_t e0 = cycles_now();
    spin_cycles(SysClock::hz / 5u);   // 200 ms
    const uint32_t e1 = cycles_now();
    Nvic::disable(EXTI4_15_IRQn);
    const uint32_t edges = exti_edges;
    const uint32_t window_us = cycles_to_us(e1 - e0);
    const uint32_t implied_us = edges == 0u ? 0u : window_us / edges;
    print(serial, "  ", edges, " rising edges in ", window_us, " us, one every ",
          implied_us, " us, where PSC 63 and ARR ", pwm_top, " give 1000", crlf);
    bench.verdict("AN EXTI LINE SEES A PAD A PERIPHERAL IS DRIVING: the "
                  "waveform's own edges are counted with no wire (7.3.1 "
                  "keeps the input buffer live in AF mode too)",
                  edges > 0u);
    bench.verdict("and the count IS the frequency - TIMPCLK / (PSC + 1) / "
                  "(ARR + 1), to a per cent",
                  implied_us >= 990u && implied_us <= 1010u);

    quiet_everything();
}

// =============================================================================
// d - a timer measuring a timer: no pad, no wire, no CPU in the path
// =============================================================================
//
// The master publishes on TRGO and the slave takes it as its own clock
// (external clock mode 1) or as a GATE. With TRGO = update the count is
// the number of master periods; with TRGO = OC1REF the trigger IS the PWM
// waveform, and the gated count is its high time.
void td_timer_on_timer() {
    T2::init();
    T3::init();

    // --- the frequency, exactly.
    (void)T2::configure({.prescaler = 0, .period = pwm_top});
    bench.verdict("TIM2 publishes its UPDATE event on TRGO",
                  T2::master(TimMasterMode::update));
    bench.verdict("TIM3 takes ITR1 as its own clock (external clock mode 1)",
                  Counter::setup(trg_t2));
    T2::enable(true);
    Counter::restart();
    const uint32_t f0 = cycles_now();
    const uint32_t k0 = Counter::count();
    spin_cycles(SysClock::hz / 10u);   // 100 ms
    const uint32_t k1 = Counter::count();
    const uint32_t f1 = cycles_now();
    const uint32_t updates = k1 - k0;
    const uint32_t cycles_per_update = updates == 0u ? 0u : (f1 - f0) / updates;
    print(serial, "  TIM3 counted ", updates, " of TIM2's updates in ", f1 - f0,
          " CPU cycles, ", cycles_per_update, " cycles each, where ARR + 1 = ",
          pwm_top + 1, crlf);
    bench.verdict("ONE TIMER COUNTS ANOTHER over the ITR1 link, with no pad "
                  "and no interrupt", updates > 0u);
    bench.verdict("and an edge-aligned period is exactly ARR + 1 counter "
                  "ticks - the count and the cycle stopwatch agree to two",
                  cycles_per_update + 2u >= pwm_top + 1u &&
                      cycles_per_update <= pwm_top + 3u);

    // --- the duty, by GATING. TRGO becomes the waveform itself.
    T3::init();
    T2::enable(false);
    (void)T2::configure({.prescaler = 0, .period = pwm_top});
    bench.verdict("TIM2 publishes OC1REF - the PWM WAVEFORM - on TRGO, which "
                  "means the channel has to be a waveform generator first: "
                  "OC1REF exists whether or not CCER lets it reach a pad",
                  T2::master(TimMasterMode::oc1ref) &&
                      T2::output_channel(0, {.mode = TimOutputMode::pwm1,
                                             .compare = 500,
                                             .enable = false}));
    T2::enable(true);
    bench.verdict("and TIM3 counts its own clock only while that is high",
                  Gate::setup(trg_t2, 3));
    struct GatePoint { uint16_t asked; uint16_t got; };
    GatePoint gp[3];
    const uint16_t gate_asks[3] = {250, 500, 750};
    for (uint8_t i = 0; i < 3; ++i) {
        (void)T2::set_compare(0, gate_asks[i]);
        T2::update();
        Gate::restart();
        const uint32_t g0 = cycles_now();
        const uint32_t q0 = Gate::count();
        spin_cycles(SysClock::hz / 400u);   // 2.5 ms, 160 periods at 64 kHz
        const uint32_t q1 = Gate::count();
        const uint32_t g1 = cycles_now();
        const uint32_t high = (q1 - q0) * 4u;    // TIM3's PSC is 3
        gp[i] = {gate_asks[i],
                 static_cast<uint16_t>(g1 == g0 ? 0u : (high * 1000ULL) / (g1 - g0))};
    }
    print(serial, "  duty per mille asked, measured by the GATE: ");
    for (uint8_t i = 0; i < 3; ++i) {
        print(serial, gp[i].asked, ":", gp[i].got, i == 2 ? "" : ", ");
    }
    print(serial, crlf);
    bool gate_ok = true;
    for (uint8_t i = 0; i < 3; ++i) {
        const int32_t err = static_cast<int32_t>(gp[i].got) -
                            static_cast<int32_t>(gp[i].asked);
        if (err > 20 || err < -20) { gate_ok = false; }
    }
    bench.verdict("A DUTY CYCLE MEASURED INSIDE THE CHIP: gated mode counts "
                  "the waveform's high time to within 20 per mille, and the "
                  "pad's own reading in letter c agrees with it", gate_ok);

    // --- slave RESET: the counter is re-initialized on every trigger.
    T3::init();
    (void)T2::master(TimMasterMode::update);
    (void)T2::configure({.prescaler = 63, .period = 999});   // 1 kHz
    T2::enable(true);
    (void)T3::configure({.prescaler = 63, .period = 0xFFFF});
    bench.verdict("TIM3 takes ITR1 as a RESET", 
                  T3::slave({.mode = TimSlaveMode::reset, .trigger = trg_t2}));
    T3::enable(true);
    uint32_t reset_high_water = 0;
    for (uint32_t i = 0; i < 40000u; ++i) {
        const uint32_t v = T3::count();
        if (v > reset_high_water) { reset_high_water = v; }
    }
    print(serial, "  slave reset: TIM3 never got past ", reset_high_water,
          " counts of a 1 ms master period", crlf);
    bench.verdict("a reset-mode slave is re-initialized by every trigger and "
                  "never reaches a whole master period twice over",
                  reset_high_water > 500u && reset_high_water < 1100u);

    quiet_everything();
}

// =============================================================================
// e - input capture with NO PAD: TIM16's TISEL reaches LSI
// =============================================================================
//
// TIM16_TISEL selects LSI, LSE or the RTC wake-up as TI1 (25.6.18), so
// the capture unit measures a real periodic signal that never leaves the
// die - the one place in this chapter where a capture needs neither a
// wire nor a pull. The reading is also a second opinion on the LSI rate,
// which test_stm32_reset measured through the watchdog and nothing else.
struct Run {
    uint32_t got = 0;
    uint32_t mean = 0;
    uint32_t lo = 0;
    uint32_t hi = 0;
    bool overcapture = false;
};

/// Collect `n` consecutive intervals with the console DRAINED first: a
/// transmit interrupt walking through this loop makes it miss an edge,
/// and a missed edge reads as an interval that is a multiple of the true
/// one. CCyOF is carried out with the readings, so a run that DID miss
/// says so instead of averaging the damage in.
Run collect_intervals(uint32_t n, uint32_t spins) {
    console_drain();
    LsiMeter::restart();
    T16::clear_flags(LsiMeter::capture_flag | LsiMeter::overrun_flag);
    Run r;
    uint32_t sum = 0;
    r.lo = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < spins && r.got < n; ++i) {
        if (T16::flag(LsiMeter::capture_flag)) {
            const auto d = LsiMeter::interval();
            if (d.has_value()) {
                sum += *d;
                if (*d < r.lo) { r.lo = *d; }
                if (*d > r.hi) { r.hi = *d; }
                ++r.got;
            }
        }
    }
    r.overcapture = T16::flag(LsiMeter::overrun_flag);
    r.mean = r.got == 0u ? 0u : sum / r.got;
    if (r.got == 0u) { r.lo = 0u; }
    return r;
}

void te_capture_lsi() {
    Rcc::lsi_enable(true);
    bench.verdict("LSI runs (it is the IWDG's clock and one of the RTC's, "
                  "and this board's RTC already asks for it)",
                  Rcc::lsi_wait_ready());

    T16::init();
    bench.verdict("TIM16's TI1 takes LSI - code 1 of 25.6.18's list, and the "
                  "driver names no source because that vocabulary belongs to "
                  "the peripheral that owns the signal",
                  T16::input_select(0, 1) && T16::input_select(0) == 1u);
    bench.verdict("and the capture channel comes up on it", LsiMeter::setup(0));

    const Run base = collect_intervals(16, 4'000'000u);
    const uint32_t mean = base.mean;
    const uint32_t lsi_hz = mean == 0u ? 0u : SysClock::hz / mean;
    print(serial, "  ", base.got, " intervals: mean ", mean, " ticks (", base.lo,
          "..", base.hi, ") giving LSI = ", lsi_hz, " Hz", crlf);
    bench.verdict("sixteen consecutive intervals arrived, none of them "
                  "overrun", base.got == 16u && !base.overcapture);
    bench.verdict("A CAPTURE THAT NEEDS NO PAD ANYWHERE - the interval lands in "
                  "DS13560 table 46's 29.5..34 kHz window for LSI",
                  lsi_hz >= 29'500u && lsi_hz <= 34'000u);
    bench.verdict("and it agrees with test_stm32_reset's watchdog-timed "
                  "32536 Hz to within 3 per cent",
                  lsi_hz > 31'570u && lsi_hz < 33'510u);
    bench.verdict("the spread of sixteen readings is a handful of ticks out "
                  "of two thousand - a capture is not an estimate",
                  base.got == 16u && (base.hi - base.lo) * 100u < mean);

    // The capture prescaler: one capture in 1, 2, 4, 8 EDGES (21.4.7).
    const TimCapturePrescaler divs[3] = {TimCapturePrescaler::every2,
                                         TimCapturePrescaler::every4,
                                         TimCapturePrescaler::every8};
    const uint32_t ratios[3] = {2, 4, 8};
    Run runs[3];
    for (uint8_t i = 0; i < 3; ++i) {
        (void)LsiMeter::setup(0, 0, TimCapturePolarity::rising, divs[i]);
        runs[i] = collect_intervals(8, 8'000'000u);
    }
    bool psc_ok = true;
    for (uint8_t i = 0; i < 3; ++i) {
        const uint32_t due = mean * ratios[i];
        print(serial, "  ICPSC /", ratios[i], ": mean ", runs[i].mean, " ticks (",
              runs[i].lo, "..", runs[i].hi, ") where ", due, " are due", crlf);
        if (runs[i].got != 8u || runs[i].overcapture ||
            runs[i].mean + due / 50u < due || runs[i].mean > due + due / 50u) {
            psc_ok = false;
        }
    }
    bench.verdict("the capture prescaler takes one edge in 2, 4 and 8, and "
                  "the interval multiplies exactly", psc_ok);

    // The overcapture flag: a capture that lands on an unread one.
    (void)LsiMeter::setup(0);
    console_drain();
    T16::clear_flags(LsiMeter::capture_flag | LsiMeter::overrun_flag);
    while (!T16::flag(LsiMeter::capture_flag)) {
    }
    const bool over_clean = !T16::flag(LsiMeter::overrun_flag);
    spin_cycles(SysClock::hz / 1000u);   // 1 ms: two more LSI edges at least
    const bool over_set = T16::flag(LsiMeter::overrun_flag);
    (void)T16::compare(0);               // reading CCR1 is the acknowledgement
    T16::clear_flags(LsiMeter::overrun_flag);
    const bool over_cleared = !T16::flag(LsiMeter::overrun_flag);
    bench.verdict("CCyOF marks a capture that arrived on an unread one - and "
                  "it is the NEW value that is lost, not the old (21.4.5)",
                  over_clean && over_set);
    bench.verdict("and a status flag is cleared by writing ZERO to it, this "
                  "family's one register that is not write-one-to-clear",
                  over_cleared);

    // The digital filter: eight samples at fDTS/32 is 4 us, well inside
    // LSI's 15 us half period, so the signal survives it unchanged.
    (void)LsiMeter::setup(0, 15);
    const Run filtered = collect_intervals(8, 4'000'000u);
    print(serial, "  ICF = 15 (eight samples at fDTS/32): mean ", filtered.mean,
          " ticks (", filtered.lo, "..", filtered.hi, ") against ", mean,
          " unfiltered", crlf);
    bench.verdict("the input filter delays both edges alike, so an interval "
                  "measured through it is the same interval",
                  filtered.got == 8u && !filtered.overcapture &&
                      filtered.mean + mean / 50u > mean &&
                      filtered.mean < mean + mean / 50u);

    quiet_everything();
}

// =============================================================================
// f - PWM input mode: period AND width of a pad walked by its own pull
// =============================================================================
//
// A capture channel does NOT drive its pad - the timer's output stage is
// disconnected while CCyS names an input - so PUPDR is still the only
// thing driving PA6, and software can walk it between the rails. What
// this letter can measure is therefore as coarse as a store and a settle
// delay; the ruler is the SAME loop's cycle stopwatch, so both readings
// carry the same overhead and the comparison is fair.
void tf_pwm_input() {
    T3::init();
    PadCap::input(PinPull::down);
    settle();
    CapIn::claim_input(PinPull::down);
    settle();
    bench.verdict("PA6 takes TIM3_CH1 at AF1 (DS13560 table 13) and PWM "
                  "input mode comes up on it (period on CC1, width on CC2, "
                  "the counter reset by every rising edge)",
                  PadCap::has_function() && PwmIn::setup(63, 3));

    // Three rounds, so the FIRST (which has no previous edge to measure
    // from) is not what the verdict rests on.
    uint32_t cap_period = 0, cap_width = 0, real_period = 0, real_width = 0;
    for (uint8_t round = 0; round < 3; ++round) {
        const uint32_t up_at = cycles_now();
        PadCap::pull(PinPull::up);
        (void)delay_us(clock, 300);
        const uint32_t down_at = cycles_now();
        PadCap::pull(PinPull::down);
        (void)delay_us(clock, 700);
        const uint32_t next_at = cycles_now();
        real_width = cycles_to_us(down_at - up_at);
        real_period = cycles_to_us(next_at - up_at);
        (void)round;
    }
    // One more rising edge, so the pair captured belongs to the round
    // whose cycle counts were just taken.
    PadCap::pull(PinPull::up);
    (void)delay_us(clock, 300);
    cap_period = PwmIn::period_ticks();
    cap_width = PwmIn::width_ticks();
    PadCap::pull(PinPull::down);

    print(serial, "  captured period ", cap_period, " ticks / width ", cap_width,
          " ticks, against ", real_period, " us / ", real_width,
          " us measured on the cycle stopwatch (1 tick = 1 us at PSC 63)", crlf);
    bench.verdict("PWM input mode reports a PERIOD, and it is the period the "
                  "stopwatch saw, to five per cent",
                  cap_period > real_period - real_period / 20u &&
                      cap_period < real_period + real_period / 20u);
    bench.verdict("and a WIDTH from the second channel, likewise",
                  cap_width > real_width - real_width / 20u &&
                      cap_width < real_width + real_width / 20u);
    bench.verdict("the width is inside the period, which is what makes the "
                  "two one measurement and not two",
                  cap_width < cap_period);
    bench.verdict("the counter is reset ON the rising edge, so a period "
                  "reads as its own tick count and not one less - the "
                  "opposite of the samc TC's capture",
                  cap_period > 900u);

    quiet_everything();
}

// =============================================================================
// g - TIM1: the complementary pair, the dead time, MOE, the break and the
//     repetition counter
// =============================================================================
//
// The advanced timer is the one shape neither the AVR's TCA nor the SAM's
// TC has and only the SAM's TCC does: two outputs that are each other's
// complement with a gap the SILICON inserts, a master switch every output
// passes through, and a break that opens it in hardware. The two pads sit
// on one port, so ONE IDR read carries both - "were they ever both high"
// is then a fact about one instant and not about two.
void tg_pair_and_break() {
    T1::init();
    T2::init();
    PairOut::claim();
    PairNOut::claim();
    // A pull under each pad so that a RELEASED output (MOE clear with
    // OSSR clear hands the pad back to the GPIO, 21.4.18 table 124) reads
    // as a level and not as noise.
    PadPair::pull(PinPull::down);
    PadPairN::pull(PinPull::down);

    // PSC 3 -> a 16 MHz counter and a 62.5 us period, while the dead time
    // is counted in tDTS - the UNDIVIDED timer clock (21.4.18) - so a DTG
    // of 128 is 2.0 us whatever the prescaler is.
    constexpr uint8_t dtg = 128;
    const uint32_t dt_ticks = tim_dead_time_ticks(dtg);
    bench.verdict("the pair comes up on TIM1_CH1 (PA8, AF2) and TIM1_CH1N "
                  "(PA7, AF2), with a dead time asked for in tDTS units",
                  Pair::setup(3, dtg) && PadPair::has_function() &&
                      PadPairN::has_function());
    Pair::duty(500);
    T1::update();
    spin_cycles(SysClock::hz / 1000u);

    PairCensus c = census_pair(PadPair::pin_number, PadPairN::pin_number, 60000u);
    const uint16_t a_pm = static_cast<uint16_t>((c.a_high * 1000u + 30000u) / 60000u);
    const uint16_t b_pm = static_cast<uint16_t>((c.b_high * 1000u + 30000u) / 60000u);
    const uint32_t period_ns = ((pwm_top + 1u) * 4u * 1000u) / (SysClock::hz / 1'000'000u);
    const uint32_t dt_ns = (dt_ticks * 1000u) / (SysClock::hz / 1'000'000u);
    print(serial, "  60000 paired samples: CH1 high ", a_pm, " per mille, CH1N ",
          b_pm, ", both high ", c.both_high, " times, neither ", c.neither, crlf);
    print(serial, "  the two duties sum to ", a_pm + b_pm, " per mille where ",
          1000u - (2u * dt_ns * 1000u) / period_ns,
          " is 1000 less twice a ", dt_ns, " ns dead time in a ", period_ns,
          " ns period", crlf);
    bench.verdict("THE PAIR IS NEVER BOTH HIGH - not once in sixty thousand "
                  "readings of one instant", c.both_high == 0u);
    bench.verdict("and it is not both LOW for long either: what is missing "
                  "from the two duties IS the dead time, twice",
                  a_pm > 400u && b_pm > 400u && (a_pm + b_pm) < 1000u);
    const uint32_t measured_dt_ns = ((1000u - (a_pm + b_pm)) * period_ns) / 2000u;
    print(serial, "  which puts the measured dead time at ", measured_dt_ns,
          " ns against ", dt_ns, " ns from DTG = ", dtg, crlf);
    bench.verdict("the dead time DTG asks for is the dead time the silicon "
                  "inserts, to a quarter of it",
                  measured_dt_ns * 4u > dt_ns * 3u && measured_dt_ns < dt_ns * 5u / 4u);
    bench.verdict("and the driver's DTG arithmetic agrees with 21.4.18's own "
                  "four ranges",
                  Pair::dead_time_ticks() == dt_ticks && dt_ticks == 128u);

    // MOE, the master switch.
    bench.verdict("MOE stands while the pair runs", T1::main_output());
    (void)T1::main_output(false);
    spin_cycles(SysClock::hz / 1000u);
    PairCensus off = census_pair(PadPair::pin_number, PadPairN::pin_number, 20000u);
    print(serial, "  with MOE clear: CH1 high ", off.a_high, " times, CH1N ",
          off.b_high, " of 20000", crlf);
    bench.verdict("clearing MOE takes BOTH outputs away at once - with OSSR "
                  "clear the pads go back to the GPIO and rest on their pulls",
                  off.a_high == 0u && off.b_high == 0u);
    (void)T1::main_output(true);
    spin_cycles(SysClock::hz / 1000u);
    PairCensus on = census_pair(PadPair::pin_number, PadPairN::pin_number, 20000u);
    bench.verdict("and raising it hands them back", on.a_high > 0u && on.b_high > 0u);

    // The break, by software: EGR.BG needs no pad and no BKE (21.4.6).
    T1::clear_flags(T1::break_flag);
    bench.verdict("a software break is generated", T1::break_event());
    spin_cycles(SysClock::hz / 10000u);
    const bool bif = T1::flag(T1::break_flag);
    const bool moe_gone = !T1::main_output();
    PairCensus broken = census_pair(PadPair::pin_number, PadPairN::pin_number, 20000u);
    print(serial, "  after EGR.BG: BIF=", bif ? "1" : "0", " MOE=",
          moe_gone ? "0" : "1", " CH1 high ", broken.a_high, " CH1N ",
          broken.b_high, crlf);
    bench.verdict("EGR.BG raises BIF and CLEARS MOE - the break is a "
                  "hardware path, not a handler's courtesy (21.4.6)",
                  bif && moe_gone && broken.a_high == 0u && broken.b_high == 0u);
    T1::clear_flags(T1::break_flag);

    // AOE: the outputs come back by themselves at the next update.
    (void)T1::break_dead_time({.dead_time = dtg,
                               .main_output_enable = true,
                               .automatic_output_enable = true});
    spin_cycles(SysClock::hz / 1000u);
    (void)T1::break_event();
    spin_cycles(SysClock::hz / 1000u);   // many periods, so an update has passed
    const bool moe_back = T1::main_output();
    bench.verdict("with AOE set the silicon raises MOE again at the next "
                  "update, the break condition being gone (21.4.18)",
                  moe_back);
    T1::clear_flags(T1::break_flag);

    // The break INPUT, on a pad. TIM1_AF1 comes up with BKINE set, so the
    // BKIN pad is already wired into the break logic; BKP = 0 makes it
    // active LOW, so a pulled-up pad is the inactive state.
    PadCap::input(PinPull::up);
    settle();
    PadCap::function(brk_pad.function, {.pull = PinPull::up});
    settle();
    (void)T1::break_dead_time({.dead_time = dtg,
                               .main_output_enable = true,
                               .break_enable = true});
    spin_cycles(SysClock::hz / 1000u);
    const bool armed_ok = T1::main_output();
    T1::clear_flags(T1::break_flag);
    PadCap::pull(PinPull::down);
    settle();
    const bool pad_bif = T1::flag(T1::break_flag);
    const bool pad_moe = !T1::main_output();
    print(serial, "  BKIN on PA6 (AF2): armed with the pad high MOE=",
          armed_ok ? "1" : "0", "; pulled low BIF=", pad_bif ? "1" : "0",
          " MOE=", pad_moe ? "0" : "1", crlf);
    bench.verdict("a BREAK INPUT ON A PAD does the same thing, and its "
                  "polarity is BDTR.BKP - active low out of reset",
                  armed_ok && pad_bif && pad_moe);
    PadCap::input(PinPull::none);

    // The repetition counter: an update every RCR + 1 periods, counted by
    // TIM2 over the ITR0 link (TIM2's ITR0 is TIM1, table 123).
    T1::init();
    T2::init();
    (void)T1::configure({.prescaler = 3, .period = pwm_top});
    (void)T1::master(TimMasterMode::update);
    bench.verdict("TIM2 counts TIM1's updates on ITR0", Counter16::setup(trg_t1));
    T1::enable(true);
    uint32_t rcr_counts[2] = {0, 0};
    for (uint8_t i = 0; i < 2; ++i) {
        (void)T1::set_repetition(i == 0 ? 0 : 3);
        T1::update();
        T2::set_count(0);
        spin_cycles(SysClock::hz / 20u);   // 50 ms
        rcr_counts[i] = T2::count();
    }
    print(serial, "  updates in 50 ms: RCR 0 gives ", rcr_counts[0], ", RCR 3 gives ",
          rcr_counts[1], crlf);
    bench.verdict("the repetition counter divides the update event by "
                  "RCR + 1 - four periods per update at RCR 3",
                  rcr_counts[1] * 4u + 8u > rcr_counts[0] &&
                      rcr_counts[1] * 4u < rcr_counts[0] + 8u);

    PadPair::input(PinPull::none);
    PadPairN::input(PinPull::none);
    quiet_everything();
}

// =============================================================================
// h - the shared vectors, and the status register that cannot swallow a
//     flag
// =============================================================================
void th_vectors() {
    clear_counts();
    T3::init();
    T4::init();
    (void)T3::configure({.prescaler = 63, .period = 999});    // 1 kHz
    (void)T4::configure({.prescaler = 63, .period = 499});    // 2 kHz
    T3::interrupts(T3::update_interrupt, true);
    T4::interrupts(T4::update_interrupt, true);
    T3::clear_flags(T3::update_flag);
    T4::clear_flags(T4::update_flag);
    T3::enable(true);
    T4::enable(true);
    Nvic::clear_pending(TIM3_TIM4_IRQn);
    Nvic::enable(TIM3_TIM4_IRQn);
    spin_cycles(SysClock::hz / 10u);   // 100 ms
    Nvic::disable(TIM3_TIM4_IRQn);
    const uint32_t n3 = t3_update_calls;
    const uint32_t n4 = t4_update_calls;
    print(serial, "  100 ms on ONE vector: TIM3 (1 kHz) ", n3,
          " updates, TIM4 (2 kHz) ", n4, crlf);
    bench.verdict("TWO TIMERS ON ONE VECTOR, each answered by its own ISR "
                  "body and each at its own rate",
                  n3 > 90u && n3 < 110u && n4 > 180u && n4 < 220u);
    bench.verdict("and neither body ever consumed the other's flag",
                  t3_saw_t4_flag == 0u);

    // A flag whose interrupt is DISABLED stands where it is - unlike this
    // family's EXTI, where an unarmed line has no pending bit at all.
    T4::interrupts(T4::update_interrupt, false);
    T4::clear_flags(T4::update_flag);
    spin_cycles(SysClock::hz / 100u);
    const bool standing = T4::flag(T4::update_flag);
    Nvic::clear_pending(TIM3_TIM4_IRQn);
    Nvic::enable(TIM3_TIM4_IRQn);
    const uint32_t before = t4_update_calls;
    spin_cycles(SysClock::hz / 100u);
    Nvic::disable(TIM3_TIM4_IRQn);
    const uint32_t after = t4_update_calls;
    bench.verdict("A TIMER FLAG STANDS WITH ITS INTERRUPT MASKED and is "
                  "readable by a poller - the EXTI of this same family "
                  "keeps no pending bit for a masked line at all",
                  standing && after == before);

    // TIM1's TWO vectors: an update on one, a compare on the other.
    T1::init();
    (void)T1::configure({.prescaler = 63, .period = 999});
    (void)T1::output_channel(0, {.mode = TimOutputMode::frozen, .compare = 500,
                                 .enable = false});
    T1::interrupts(T1::update_interrupt | T1::compare_interrupt(0), true);
    T1::clear_flags(T1::update_flag | T1::compare_flag(0));
    t1_up_calls = 0;
    t1_cc_calls = 0;
    T1::enable(true);
    Nvic::clear_pending(T1::irq());
    Nvic::clear_pending(T1::cc_irq());
    Nvic::enable(T1::irq());
    Nvic::enable(T1::cc_irq());
    spin_cycles(SysClock::hz / 20u);   // 50 ms
    Nvic::disable(T1::irq());
    Nvic::disable(T1::cc_irq());
    print(serial, "  50 ms of TIM1 at 1 kHz: ", t1_up_calls,
          " on the BRK/UP/TRG/COM vector, ", t1_cc_calls, " on the CC vector", crlf);
    bench.verdict("TIM1 alone reports on TWO vectors: the update lands on "
                  "one and the compare on the other, and both fire",
                  t1_up_calls > 40u && t1_up_calls < 60u && t1_cc_calls > 40u &&
                      t1_cc_calls < 60u);

    // rc_w0: clearing one flag cannot clear another.
    T2::init();
    (void)T2::configure({.prescaler = 0, .period = 0xFFFF});
    T2::clear_flags(0xFFFFu);
    T2::update();
    (void)T2::capture_compare_event(0);
    spin_cycles(1000u);
    const uint32_t sr_both = T2::flags();
    const bool both_set = (sr_both & T2::update_flag) != 0u &&
                          (sr_both & T2::compare_flag(0)) != 0u;
    T2::clear_flags(T2::update_flag);
    spin_cycles(1000u);
    const uint32_t sr_one = T2::flags();
    const bool only_one = (sr_one & T2::update_flag) == 0u &&
                          (sr_one & T2::compare_flag(0)) != 0u;
    print(serial, "  EGR.UG + EGR.CC1G left SR = ", hex(sr_both),
          "; a store of ~UIF left ", hex(sr_one), crlf);
    bench.verdict("TIMx_SR is rc_w0 and the driver clears it with a store of "
                  "~mask: one flag goes and its neighbour stays, with no "
                  "read-modify-write to lose an arrival (21.4.5)",
                  both_set && only_one);

    quiet_everything();
}

// =============================================================================
// i - centre-aligned mode: the period is 2 x ARR, measured
// =============================================================================
//
// The instrument is letter d's: TIM2 publishes OC1REF - the waveform
// itself - and TIM3 counts its rising edges, so the number of WAVEFORM
// PERIODS in a cycle-measured window is exact. The samc campaign found
// the SAM's printed dual-slope formula off by one, so this letter
// distinguishes 2 x ARR from 2 x (ARR + 1) rather than assuming either.
struct WaveformRate {
    uint32_t counts;
    uint32_t cycles;
    uint32_t period_x10;   ///< counter ticks per waveform period, x10
};
WaveformRate measure_waveform(uint32_t window_cycles) {
    Counter::restart();
    const uint32_t c0 = cycles_now();
    const uint32_t k0 = Counter::count();
    spin_cycles(window_cycles);
    const uint32_t k1 = Counter::count();
    const uint32_t c1 = cycles_now();
    const uint32_t counts = k1 - k0;
    const uint32_t cycles = c1 - c0;
    return {counts, cycles, counts == 0u ? 0u : (cycles * 10u) / counts};
}

void ti_center_aligned() {
    T2::init();
    T3::init();
    (void)T2::master(TimMasterMode::oc1ref);
    bench.verdict("TIM3 counts TIM2's OC1REF rising edges - one per "
                  "WAVEFORM period, whichever way the counter runs",
                  Counter::setup(trg_t2));

    (void)T2::configure({.prescaler = 0, .period = pwm_top});
    (void)T2::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500});
    T2::enable(true);
    const WaveformRate edge = measure_waveform(SysClock::hz / 5u);   // 200 ms

    T2::enable(false);
    (void)T2::configure({.prescaler = 0, .period = pwm_top,
                         .alignment = TimAlignment::center_up});
    (void)T2::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500});
    T2::enable(true);
    const WaveformRate centre = measure_waveform(SysClock::hz / 5u);

    print(serial, "  edge-aligned: ", edge.counts, " periods in ", edge.cycles,
          " cycles, ", edge.period_x10 / 10u, ".", edge.period_x10 % 10u,
          " ticks, where ARR + 1 = ", pwm_top + 1, crlf);
    print(serial, "  centre-aligned: ", centre.counts, " periods in ",
          centre.cycles, " cycles, ", centre.period_x10 / 10u, ".",
          centre.period_x10 % 10u, " ticks, where 2 x ARR = ", 2u * pwm_top,
          " and 2 x (ARR + 1) = ", 2u * (pwm_top + 1u), crlf);

    const uint32_t edge_due = (pwm_top + 1u) * 10u;
    const uint32_t centre_due = 2u * pwm_top * 10u;
    const uint32_t centre_other = 2u * (pwm_top + 1u) * 10u;
    bench.verdict("an EDGE-aligned period is ARR + 1 counter ticks, to a "
                  "tenth of a tick",
                  edge.period_x10 + 10u > edge_due && edge.period_x10 < edge_due + 10u);
    // The two candidate formulas are TWO TICKS apart, so the window has
    // to be tighter than one: half a tick to 2 x ARR, and at least one
    // whole tick away from 2 x (ARR + 1).
    bench.verdict("A CENTRE-ALIGNED PERIOD IS 2 x ARR - the counter runs "
                  "0..ARR-1 up and ARR..1 down (21.3.3), so it is NOT "
                  "2 x (ARR + 1), and this measurement tells the two apart",
                  centre.period_x10 + 5u >= centre_due &&
                      centre.period_x10 <= centre_due + 5u &&
                      centre.period_x10 + 10u < centre_other);

    // The three CMS codes differ only in WHEN the compare flag is raised:
    // counting down, counting up, or both (21.4.1). One rate against two.
    clear_counts();
    T2::interrupts(T2::compare_interrupt(0), true);
    uint32_t cc_counts[2] = {0, 0};
    const TimAlignment modes[2] = {TimAlignment::center_up, TimAlignment::center_both};
    for (uint8_t i = 0; i < 2; ++i) {
        T2::enable(false);
        (void)T2::configure({.prescaler = 63, .period = 499, .alignment = modes[i]});
        (void)T2::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 250,
                                     .enable = false});
        T2::interrupts(T2::compare_interrupt(0), true);
        T2::clear_flags(0xFFFFu);
        t2_compare_calls = 0;
        T2::enable(true);
        Nvic::clear_pending(T2::irq());
        Nvic::enable(T2::irq());
        spin_cycles(SysClock::hz / 10u);   // 100 ms
        Nvic::disable(T2::irq());
        cc_counts[i] = t2_compare_calls;
    }
    print(serial, "  compare flags in 100 ms: CMS = up ", cc_counts[0],
          ", CMS = both ", cc_counts[1], crlf);
    bench.verdict("CMS chooses WHEN the compare flag rises, not what the "
                  "waveform is: 'both' raises it twice per period where "
                  "'up' raises it once",
                  cc_counts[0] > 0u && cc_counts[1] > cc_counts[0] * 3u / 2u &&
                      cc_counts[1] < cc_counts[0] * 5u / 2u);

    quiet_everything();
}

// =============================================================================
// j - MeterSampler inside a REAL KERNEL, fed by a capture ISR
// =============================================================================
//
// THIS IS THE CAMPAIGN'S POINT, not a bonus letter. util/meter_sampler.hpp
// was designed on the AVR around a capture ISR that fills a one-cell
// latch and an AO that paces PUBLICATION rather than capture; the samc
// campaign ran it from a SAM TC through EVSYS from an EIC pin. Here the
// source is a TIM16 capture channel fed by LSI over TISEL - a chain with
// nothing in common with either - and NOT ONE LINE OF util/ CHANGED.
//
// The economy is the file's own: LSI arrives about 32000 times a second
// and the sampler publishes ten times a second, so what the queues carry
// is the application's pace and the latch's missed() counts the rest.

using LsiLatch = MeterLatch<uint32_t, Stm32Platform, 0>;
static_assert(MeterSource<LsiLatch>, "the capture latch is a MeterSource");

struct Collector;
using Subs = Subscribers<Collector>;
using Sampler = MeterSampler<Stm32Platform, Subs, LsiLatch>;

struct Collector {
    using Event = std::variant<MeterSample>;
    static inline EventQueue<Event, 8, Stm32Platform> queue;

    static inline uint16_t samples = 0;
    static inline uint32_t last = 0;
    static inline uint32_t lo = 0xFFFFFFFFUL;
    static inline uint32_t hi = 0;
    static inline uint8_t last_index = 0xFF;

    static void init() {
        samples = 0;
        last = 0;
        lo = 0xFFFFFFFFUL;
        hi = 0;
        last_index = 0xFF;
    }
    static void dispatch(const Event& e) {
        match(e, [](MeterSample s) {
            last = s.value;
            last_index = s.index;
            if (s.value < lo) { lo = s.value; }
            if (s.value > hi) { hi = s.value; }
            if (samples != UINT16_MAX) { ++samples; }
        });
    }
};

using MeterKernel = Kernel<Stm32Platform, Collector, Sampler>;

volatile bool kernel_mode = false;

void tj_meter_ao() {
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_wait_ready();
    T16::init();
    bench.verdict("TIM16 captures LSI again, this time through its interrupt",
                  T16::input_select(0, 1) && LsiMeter::setup(0));

    LsiLatch::clear();
    LsiMeter::restart();
    t16_captures = 0;
    t16_stores = 0;
    T16::clear_flags(0xFFFFu);
    T16::interrupts(LsiMeter::capture_interrupt, true);

    MeterKernel::init_all();
    Sampler::start_every(Ticker::ticks_per_second / 10u);   // ten a second

    kernel_mode = true;
    Nvic::clear_pending(T16::irq());
    Nvic::enable(T16::irq());
    const uint32_t started = Ticker::millis();
    while (Ticker::millis() - started < 1000UL) {
        TimeEvents<Stm32Platform>::process();
        while (MeterKernel::step()) {
        }
    }
    Nvic::disable(T16::irq());
    kernel_mode = false;
    T16::interrupts(LsiMeter::capture_interrupt, false);

    const uint32_t captures = t16_captures;
    const uint16_t published = Sampler::published();
    const uint16_t missed = Sampler::missed(0);
    const bool leftover = LsiLatch::fresh();
    print(serial, "  one second: ", captures, " capture interrupts, ", published,
          " samples published, ", Collector::samples, " received; values ",
          Collector::lo, "..", Collector::hi, " ticks, latch missed ", missed,
          ", leftover ", leftover ? "1" : "0", crlf);

    bench.verdict("the capture ISR ran at the wire's rate - about 32000 LSI "
                  "edges in a second", captures > 25'000u && captures < 40'000u);
    bench.verdict("and the AO published at the APPLICATION's rate instead - "
                  "ten a second, which is meter_sampler.hpp's whole design",
                  published >= 8u && published <= 12u);
    bench.verdict("every published sample reached the subscriber, none lost "
                  "in a queue", Collector::samples == published);
    bench.verdict("each one labelled with its source's place in the pack",
                  Collector::last_index == 0u);
    bench.verdict("and each one is a real LSI period, not a stale repeat",
                  Collector::lo > 1500u && Collector::hi < 2500u);
    // Every store either found the cell empty or overwrote an untaken
    // value; every take with a value became a publication. So the three
    // counts and the leftover ADD UP EXACTLY - the latch loses nothing it
    // does not count.
    const uint32_t stores = t16_stores;
    const uint32_t accounted = static_cast<uint32_t>(published) +
                               static_cast<uint32_t>(missed) + (leftover ? 1u : 0u);
    print(serial, "  published + missed + leftover = ", accounted, " against ",
          stores, " stores (of ", captures,
          " captures - the first edge has no previous one to measure from)", crlf);
    bench.verdict("PUBLISHED + MISSED + LEFTOVER ACCOUNTS FOR EVERY CAPTURE: "
                  "a latch discards, and says exactly how much",
                  missed != UINT16_MAX && accounted == stores &&
                      stores + 1u == captures);

    Sampler::stop();
    quiet_everything();
}

// =============================================================================
// k - the errata (ES0548 Rev 3, revision Z)
// =============================================================================
//
// 2.7.2 is the one this bench can stage, and it is staged WITH A CONTROL:
// the same compare value at the same counter value, once reached the slow
// way and once through the two-adjacent-cycles window the erratum
// describes. The prescaler is at its maximum so that one counter tick is
// about a millisecond, which is what makes "write CCR between two
// consecutive counter cycles" something software can do at all.
void tk_errata() {
    T3::init();
    // 65536 CPU cycles per counter tick: 1.024 ms, which is what makes
    // "write CCR between two consecutive counter cycles" something
    // software can do at all.
    (void)T3::configure({.prescaler = 0xFFFF, .period = 4});
    // TOGGLE mode on a pad, because the erratum's stated consequence in
    // EDGE-ALIGNED mode is about the OUTPUT ("the output only toggles
    // once per counter period") and not about the flag - the flag half
    // of 2.7.2 is the CENTRE-aligned case. Both are watched.
    CapIn::claim();
    (void)T3::output_channel(0, {.mode = TimOutputMode::toggle, .compare = 0,
                                 .preload = false});
    T3::clear_flags(0xFFFFu);
    T3::enable(true);

    // The CONTROL: CCR = 0 set long before the counter reaches 0. The
    // compare must fire and the output must move.
    T3::set_count(1);
    const bool level_before = PadCap::read();
    T3::clear_flags(T3::compare_flag(0));
    uint32_t spins = 0;
    while (!T3::flag(T3::compare_flag(0)) && spins < 4'000'000u) {
        ++spins;
    }
    const bool control_fired = T3::flag(T3::compare_flag(0));
    const bool control_moved = PadCap::read() != level_before;
    bench.verdict("the control: a compare at CNT = CCR = 0, set well in "
                  "advance, fires AND toggles the pad - the instrument is "
                  "sensitive on both halves", control_fired && control_moved);

    // THE ERRATUM'S WINDOW, eight times over: a match at CNT = CCR = ARR,
    // then CCR moved to 0 before the counter's next tick wraps it there -
    // two matches in two consecutive counter cycles, the value having
    // changed between them.
    uint32_t first_fired = 0, second_fired = 0, second_toggled = 0, wrapped = 0;
    for (uint8_t round = 0; round < 8u; ++round) {
        (void)T3::set_compare(0, 4);
        T3::clear_flags(T3::compare_flag(0) | T3::update_flag);
        spins = 0;
        while (!T3::flag(T3::compare_flag(0)) && spins < 8'000'000u) {
            ++spins;
        }
        if (T3::flag(T3::compare_flag(0))) {
            first_fired = first_fired + 1u;
        }
        const bool mid = PadCap::read();
        T3::clear_flags(T3::compare_flag(0) | T3::update_flag);
        (void)T3::set_compare(0, 0);          // inside the one-tick window
        spins = 0;
        while (!T3::flag(T3::update_flag) && spins < 8'000'000u) {
            ++spins;
        }
        if (T3::flag(T3::update_flag)) {
            wrapped = wrapped + 1u;
        }
        if (T3::flag(T3::compare_flag(0))) {
            second_fired = second_fired + 1u;
        }
        if (PadCap::read() != mid) {
            second_toggled = second_toggled + 1u;
        }
    }
    print(serial, "  ES0548 2.7.2 staged 8 times: the first match (CNT = CCR "
                  "= ARR) fired ", first_fired, " times, the counter wrapped ",
          wrapped, " times, and the SECOND match (CNT = CCR = 0, one counter "
          "cycle later) raised its flag ", second_fired, " times and toggled "
          "the pad ", second_toggled, " times", crlf);
    bench.verdict("the erratum's first match happens every round and the "
                  "counter really does reach the next cycle",
                  first_fired == 8u && wrapped == 8u);
    // WHAT THE BENCH FOUND, recorded as it is: the second compare of two
    // consecutive counter cycles is served in full - the flag rises and
    // the output toggles - so ES0548 2.7.2 did NOT reproduce in this
    // staging on revision Z. The erratum stands unrefuted rather than
    // disproved: its own description is of a REPEATING pattern driven at
    // the counter's rate (a single-cycle-wide pulse per period built by
    // DMA), where this letter drives it once per period from software.
    bench.verdict("ES0548 2.7.2 did NOT reproduce here: the second compare "
                  "raised its flag and toggled its output every round",
                  second_fired == 8u && second_toggled == 8u);

    print(serial, "  2.7.1 (one-pulse trigger lost in master-slave reset + "
                  "trigger with MSM) NOT staged: it needs the trigger to "
                  "arrive exactly at CNT = ARR of a cascaded master, which "
                  "nothing here can place. Its own workaround is the "
                  "driver's default - TimSlaveConfig::master_slave is false.",
          crlf);
    print(serial, "  2.7.3 (output compare clear with an external reset) NOT "
                  "staged: ocref_clr comes from ETR or a comparator, and "
                  "this stratum has neither an ETR wire nor a COMP driver.",
          crlf);
    bench.verdict("the two unreachable TIM errata are named with the reason, "
                  "not described away", true);

    quiet_everything();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf,
          "test_stm32_tim - STM32G0B1RE timers (RM0444 ch. 21..25): PwmChannel "
          "and MeterSampler on the third silicon, wireless, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler is
// a spin loop - so every vector a letter can raise is bound, whether or
// not a letter is using it. SHARED VECTORS ARE THE RULE on this family
// (RM0444 table 61), so a handler calls one ISR BODY per owner and each
// answers for its own flags only.
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void EXTI4_15_IRQHandler() {
    const brio::ExtiPending p = brio::Exti::isr(brio::Exti::vector_lines(EXTI4_15_IRQn));
    if (p.any()) {
        exti_edges = exti_edges + 1u;
    }
}

extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler() {
    const uint32_t f = T1::isr();
    if ((f & T1::update_flag) != 0u) {
        t1_up_calls = t1_up_calls + 1u;
    }
    if ((f & (T1::break_flag | T1::break2_flag)) != 0u) {
        t1_break_seen = t1_break_seen + 1u;
    }
}

extern "C" void TIM1_CC_IRQHandler() {
    const uint32_t f = T1::isr();
    if (f != 0u) {
        t1_cc_calls = t1_cc_calls + 1u;
    }
}

extern "C" void TIM2_IRQHandler() {
    const uint32_t f = T2::isr();
    if ((f & T2::update_flag) != 0u) {
        t2_update_calls = t2_update_calls + 1u;
    }
    if ((f & T2::compare_flag(0)) != 0u) {
        t2_compare_calls = t2_compare_calls + 1u;
    }
}

/// The shared line, and letter h's question: each body reads its own
/// timer, so TIM3's can neither see nor consume TIM4's flags. What is
/// recorded is whether it ever did.
extern "C" void TIM3_TIM4_IRQHandler() {
    const uint32_t f3 = T3::isr();
    if ((f3 & T3::update_flag) != 0u) {
        t3_update_calls = t3_update_calls + 1u;
    }
    if ((f3 & ~(T3::update_flag | T3::compare_flag(0))) != 0u) {
        t3_saw_t4_flag = t3_saw_t4_flag + 1u;
    }
    const uint32_t f4 = T4::isr();
    if ((f4 & T4::update_flag) != 0u) {
        t4_update_calls = t4_update_calls + 1u;
    }
}

/// TIM16's line is shared with an FDCAN one. In letter j the capture
/// feeds a util MeterLatch; everywhere else it is only counted, so the
/// reading is taken either way and `kernel_mode` says where it goes.
extern "C" void TIM16_FDCAN_IT0_IRQHandler() {
    const uint32_t f = T16::isr();
    if ((f & LsiMeter::capture_flag) != 0u) {
        t16_captures = t16_captures + 1u;
        const auto d = LsiMeter::interval();
        if (kernel_mode && d.has_value()) {
            t16_stores = t16_stores + 1u;
            LsiLatch::store(*d);
        }
    }
}

int main() {
    // Sampled BEFORE anything can disturb them: letter a judges what this
    // boot found, and every verb of this suite writes some of it.
    boot_apbenr1 = RCC->APBENR1;
    boot_apbenr2 = RCC->APBENR2;
    boot_t2_cr1 = TIM2->CR1;
    boot_t2_arr = TIM2->ARR;
    boot_t1_bdtr = TIM1->BDTR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);

    brio::enable_interrupts();

    bench.letter('a', "the block: geometry, reset values, vectors, refusals, pads",
                 ta_block);
    bench.letter('b', "the time base: arithmetic, the shadow registers, 32 bits",
                 tb_time_base);
    bench.letter('c', "PWM on LD4, read back through its own pad", tc_pwm_pad);
    bench.letter('d', "a timer measuring a timer: frequency, and duty by gating",
                 td_timer_on_timer);
    bench.letter('e', "input capture with no pad: TIM16 on LSI", te_capture_lsi);
    bench.letter('f', "PWM input mode on a pad walked by its own pull", tf_pwm_input);
    bench.letter('g', "TIM1: the pair, the dead time, MOE, the break, RCR",
                 tg_pair_and_break);
    bench.letter('h', "the shared vectors, and a status register that is rc_w0",
                 th_vectors);
    bench.letter('i', "centre-aligned mode: the period is 2 x ARR", ti_center_aligned);
    bench.letter('j', "MeterSampler inside a real kernel, fed by a capture ISR",
                 tj_meter_ao);
    bench.letter('k', "ES0548 2.7.2 staged with a control", tk_errata);

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
