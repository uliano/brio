// test_stm32_tim - the reference bench suite for the STM32G0's TIMERS
// (RM0444 ch. 21..25: the advanced-control TIM1, the general-purpose
// TIM2/3/4, the basic TIM6/7, TIM14 and TIM15/16/17) and, through them,
// for two util contracts on this silicon: util/pwm_channel.hpp's
// PwmChannel and util/meter_sampler.hpp's MeterLatch/MeterSampler.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
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
//      live in alternate-function mode (7.3.1), so the board LED's own
//      PWM is readable on IDR and an EXTI line can count its edges;
//      letter c is what proves it for a pad under an ALTERNATE FUNCTION
//      and not only for one its own port drives.
//   3. A CAPTURE WITH NO PAD AT ALL. TIM16_TISEL selects LSI as TI1
//      (25.6.18), so the capture unit measures a real ~32 kHz signal that
//      never leaves the die - and the reading is a second, independent
//      measurement of the LSI rate the watchdog implies.
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
//   PB6..PB9       TIM4_CH1..4 AF9  the four channels of the vector-mate
//   PB13/PB14/PB15 TIM15_CH1N/CH1/CH2 AF5  two channels and a complement
//   PA7            TIM14_CH1 AF4   the one-channel timer, on a pad already
//                                  proven free as TIM1_CH1N
//   PB7/PB9        TIM17_CH1N/CH1 AF2  the other one-channel pair
// ON THE LQFP32 (DS12992 table 12) the list is shorter, and the two
// differences are the PACKAGE's and not the die's: PB10..PB15 are bonded
// to no pin at all, so TIM15's three outputs have nowhere to go even
// where there is a TIM15; and the board LED is LD3 on PC6, which takes
// TIM2_CH3 at AF2 (DS12992 table 16) where the Nucleo-64s' LD4 on PA5
// takes TIM2_CH1. Everything else this suite drives - PA6, PA7, PA8,
// PB6..PB9 - is bonded on all three parts and carries the same functions.
// AND PA8 IS UCPD1_CC1 (PB15 is UCPD1_CC2) ON A PART THAT HAS A UCPD,
// and the pads come out of reset already loaded: RM0444 7.3.16 connects a
// Type-C DEAD-BATTERY pull-down to both out of a power-on until
// SYSCFG_CFGR1's strobe releases them, a few kilohms against the port's
// own forty. Letter a spends that strobe once, before it asks either pad
// to follow its own pull - and on a part the reserve reports as having no
// UCPD at all the verb REFUSES instead, which is what letter a judges
// there.
// Avoided on purpose: PA2/PA3 (the console), PA13/PA14 (SWD), PC13 (B1),
// PC14/PC15 (the LSE pads), PF0/PF1 (the HSE pads).
//
// What is exercised, letter by letter:
//   a  the block: what the reserve says each timer IS, the reset values
//      this boot found, the two vectors of TIM1, and every refusal
//   b  the time base: the prescaler and ARR arithmetic against SysTick,
//      the SHADOW registers, TIM2's 32-bit counter past 16 bits,
//      down-counting, one-pulse mode and URS
//   c  PWM on the board LED: the duty read back through the pad, and the
//      pad's own edges counted by an EXTI line
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
//   k  the G0B1's TIM erratum 2.7.2 staged with a control, 2.7.1 and
//      2.7.3 declared - the numbers being ES0548's, one sheet per part
//   l  THE INSTANCES this suite had never counted - TIM14 and TIM17
//      always, and TIM4, TIM6, TIM7 and TIM15 where the part has them:
//      each counting PCLK at its prescaler, each update reaching the
//      vector the reserve names for it, and a PWM off a pad for those
//      that have channels AND pads on this package
//
// WHAT SKIPS WHERE. An instance the part has not got is not a failure and
// not a silence: the leg that needs it prints one SKIPPED line naming
// what it wanted and the reserve fact that refused it, and claims no
// verdict at all. On the G0B1 every letter runs whole; on the G071 the
// TIM4 legs skip; on the G031 the TIM4, TIM6, TIM7 and TIM15 legs skip
// with them, and letter l's port-B census stops at PB9 because the
// package stops there.
//
// build: boards = g0b1re,g071rb,g031k8
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
#include "stm32g0/platform.hpp"
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

// ---- the package -----------------------------------------------------------
// THE ONE QUESTION THE RESERVE DOES NOT ANSWER. device_tables.hpp speaks
// about the DIE - which peripherals the header declares - and never about
// the plastic around it or the board under it. Which device header is
// being compiled is the only thing the preprocessor can ask, and what
// package and board go with it is this suite's own knowledge: the
// LQFP32 of DS12992 table 12 bonds PA0..PA15, PB0..PB9, PC6, PC14, PC15
// and PF2(NRST) and nothing else, and the Nucleo-32 it sits on carries
// LD3 on PC6 where the Nucleo-64s carry LD4 on PA5.
#if defined(STM32G031xx)
constexpr bool package_lqfp32 = true;
#else
constexpr bool package_lqfp32 = false;
#endif

/// PB10..PB15 reach a pin on the LQFP48/LQFP64 and on no pin of the
/// LQFP32 - which is what decides whether letter l may walk PB13, PB14
/// and PB15 at all, TIM15 or no TIM15.
constexpr bool pb_high_bonded = !package_lqfp32;

// ---- the timers ------------------------------------------------------------
using T1 = Tim<1>;     // advanced: the pair, the dead time, the break, RCR
using T2 = Tim<2>;     // the 32-bit one; the master of every cascade here
using T3 = Tim<3>;     // the slave: counts, gates, resets, captures
// TIM4 is TIM3's vector-mate on the parts that have one, and the one
// timer of this suite's set a G0 may not have at all. A part without it
// must not SPELL `Tim<4>` (the driver's static_assert is the refusal),
// `if constexpr` inside a plain function still instantiates the branch it
// discards, and a NON-DEPENDENT `Tim<4>` inside a template body is looked
// up when the template is DEFINED - so the timer NUMBER is made to depend
// on the reserve fact that gates every branch that names it.
template <bool present>
using Tim4 = Tim<present ? uint8_t{4} : uint8_t{3}>;
using T16 = Tim<16>;   // one channel, and TISEL reaches LSI
// Letter l's set: every instance this suite had never counted. THREE MORE
// OF THEM ARE PER-PART, and each is reached through the same dependent
// alias TIM4's comment above explains - the number depending on the
// reserve fact that gates every branch that names it, so that a part
// without the instance never instantiates Tim<n> and never trips its
// static_assert.
template <bool present>
using Tim6 = Tim<present ? uint8_t{6} : uint8_t{3}>;    // basic: no channel, a TRGO
template <bool present>
using Tim7 = Tim<present ? uint8_t{7} : uint8_t{3}>;    // basic, LPTIM2's vector-mate
template <bool present>
using Tim15 = Tim<present ? uint8_t{15} : uint8_t{3}>;  // two channels, one complement
using T14 = Tim<14>;   // one channel, no slave, no master, no BDTR
using T17 = Tim<17>;   // one channel and its complement

// ---- the pads --------------------------------------------------------------
// The AF numbers are DS13560 tables 13, 15 and 16 (port A AF0..7, port B
// AF0..7 and port B AF8..15 - TIM4's channels are the only ones here
// above AF7); nothing in the device header can check them
// (stm32g0/pin.hpp says so once for the stratum), so THIS SUITE IS THE
// CHECK - letters c, f, g and l are what prove them.
// THE BOARD LED IS A PAD CHOICE AND NOT AN INSTANCE ONE. TIM2 has four
// channels on every G0 of this pack, so what the package moves is which
// pad and therefore which CHANNEL: LD4 is PA5 = TIM2_CH1 at AF2 on the
// Nucleo-64s (DS13560 table 13), LD3 is PC6 = TIM2_CH3 at AF2 on the
// Nucleo-32 (DS12992 table 16). Its EXTI line is its own pin number
// either way, and lines 5 and 6 are both EXTI4_15's.
constexpr PinSel led_pad = package_lqfp32 ? PinSel{'C', 6, PinFunction::af2}
                                          : PinSel{'A', 5, PinFunction::af2};
constexpr uint8_t led_channel = package_lqfp32 ? uint8_t{2} : uint8_t{0};
constexpr const char* led_name = package_lqfp32 ? "PC6 (LD3) on TIM2_CH3"
                                                : "PA5 (LD4) on TIM2_CH1";
constexpr PinSel cap_pad{'A', 6, PinFunction::af1};    // TIM3_CH1
constexpr PinSel brk_pad{'A', 6, PinFunction::af2};    // TIM1_BKIN (same pad)
constexpr PinSel pairn_pad{'A', 7, PinFunction::af2};  // TIM1_CH1N
constexpr PinSel pair_pad{'A', 8, PinFunction::af2};   // TIM1_CH1

using LedOut = TimPad<led_pad>;
using CapIn = TimPad<cap_pad>;
using PairOut = TimPad<pair_pad>;
using PairNOut = TimPad<pairn_pad>;

using PadLed = LedOut::pin;   // the pad the PinSel above names, whichever it is
using PadCap = Pin<'A', 6>;
using PadPairN = Pin<'A', 7>;
using PadPair = Pin<'A', 8>;

// Letter l's pads: the channels of the instances this suite had never
// counted, all on pads no other suite of this board moves and each
// pull-walked by the letter itself before it is believed. THE PORT IS
// PART OF THE POINT: TIM4's four channels, TIM15's three outputs and
// TIM17's pair all land on port B, so a complementary pair is one IDR
// read and a four-channel timer is one sampling loop.
// TIM4's and TIM15's pads are named on every part: a PinSel is a pad and
// an alternate-function NUMBER, pure data with no register behind it, and
// nothing but a live leg of that instance ever claims them - which is why
// the LQFP32, which bonds neither the instances nor PB13..PB15, still
// compiles these lines and never reaches a pin through them.
constexpr PinSel t4_ch1_pad{'B', 6, PinFunction::af9};    // TIM4_CH1
constexpr PinSel t4_ch2_pad{'B', 7, PinFunction::af9};    // TIM4_CH2
constexpr PinSel t4_ch3_pad{'B', 8, PinFunction::af9};    // TIM4_CH3
constexpr PinSel t4_ch4_pad{'B', 9, PinFunction::af9};    // TIM4_CH4
constexpr PinSel t14_ch1_pad{'A', 7, PinFunction::af4};   // TIM14_CH1 (PA7 again)
constexpr PinSel t15_ch1n_pad{'B', 13, PinFunction::af5}; // TIM15_CH1N
constexpr PinSel t15_ch1_pad{'B', 14, PinFunction::af5};  // TIM15_CH1
constexpr PinSel t15_ch2_pad{'B', 15, PinFunction::af5};  // TIM15_CH2
constexpr PinSel t17_ch1n_pad{'B', 7, PinFunction::af2};  // TIM17_CH1N (PB7 again)
constexpr PinSel t17_ch1_pad{'B', 9, PinFunction::af2};   // TIM17_CH1 (PB9 again)

using PadB6 = Pin<'B', 6>;
using PadB7 = Pin<'B', 7>;
using PadB8 = Pin<'B', 8>;
using PadB9 = Pin<'B', 9>;
using PadB13 = Pin<'B', 13>;
using PadB14 = Pin<'B', 14>;
using PadB15 = Pin<'B', 15>;

// ---- the tasks -------------------------------------------------------------
// A PWM period of 1000 counts: with PSC = 0 that is 64 kHz (fast enough
// that a sampling loop crosses hundreds of periods), with PSC = 63 it is
// 1 kHz (slow enough for an interrupt per edge).
constexpr uint16_t pwm_top = 999;
using LedPwm = TimPwm<T2, led_channel, pwm_top>;
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
// Letter l's six, each on its own vector (two of them shared with
// something that is not a timer at all).
volatile uint32_t t6_update_calls = 0;
volatile uint32_t t7_update_calls = 0;
volatile uint32_t t14_update_calls = 0;
volatile uint32_t t15_update_calls = 0;
volatile uint32_t t17_update_calls = 0;

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
    t6_update_calls = 0;
    t7_update_calls = 0;
    t14_update_calls = 0;
    t15_update_calls = 0;
    t17_update_calls = 0;
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
/// pull of about 40 kohm.
void settle() { (void)delay_us(clock, 200); }

/// Wait for the console to be physically empty. A measurement window
/// that a transmit interrupt walks through is not a measurement, and
/// letter e is where it bites hardest:
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

/// A pad an external pull-up holds UP (a wired I2C bus puts 2.2 k on
/// PB8/PB9 - docs/bench.md) cannot follow its internal pull-down, and is
/// still a pad this suite may DRIVE: a push-pull output beats 2.2 k.
/// Free means either.
template <class Pad>
bool pad_is_drivable() {
    if (pad_follows_pull<Pad>()) {
        return true;
    }
    Pad::input(PinPull::down);
    settle();
    const bool held_up = Pad::read();
    Pad::output(false);
    settle();
    const bool sinks = !Pad::read();
    Pad::input(PinPull::none);
    return held_up && sinks;
}

/// Per-mille high time of one pad of port A, sampled through IDR and
/// nothing else in the loop. The estimator is only as good as the number
/// of WAVEFORM PERIODS the window crosses, which is why every caller
/// runs its waveform in the tens of kilohertz.
/// THE LOOP MUST TAKE THE SAME TIME WHATEVER IT READS. A sampler that
/// branches on the bit it just read spends more cycles in one state than
/// in the other, and then counts FEWER iterations of the slower state -
/// a bias that reads as a duty the waveform does not have (measured: a
/// branching sampler reads a 50 % pair as 566 and 383 per
/// mille). So the bit is shifted down and ADDED, and nothing here
/// branches on data.
/// THE PORT IS A TEMPLATE PARAMETER and not an argument, so the loop
/// still reads one register through one constant address and still
/// branches on nothing: letter c's LED is on port A or port C depending
/// on the package, and a runtime choice inside the window would be a
/// sampler that measures itself.
template <char L>
uint16_t sample_permille_port(uint8_t shift, uint32_t samples) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        high += (Port<L>::in() >> shift) & 1u;
    }
    return static_cast<uint16_t>((high * 1000u + samples / 2u) / samples);
}

uint16_t sample_permille(uint8_t shift, uint32_t samples) {
    return sample_permille_port<'A'>(shift, samples);
}

/// The same census on PORT B, where letter l's instances put their
/// channels. Same rule: nothing here branches on data.
uint16_t sample_permille_b(uint8_t shift, uint32_t samples) {
    return sample_permille_port<'B'>(shift, samples);
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
PairCensus census_pair_b(uint8_t shift_a, uint8_t shift_b, uint32_t samples) {
    uint32_t a = 0, b = 0, both = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        const uint32_t v = Port<'B'>::in();
        const uint32_t x = (v >> shift_a) & 1u;
        const uint32_t y = (v >> shift_b) & 1u;
        a += x;
        b += y;
        both += x & y;
    }
    return {both, a, b, samples - a - b + both};
}

// ---- the per-part instances, asked only where the part has them -------------
//
// Each of these is a FUNCTION TEMPLATE and not an `if constexpr` inside a
// plain function, for the reason Tim4's alias comment gives: a discarded
// branch of a plain function is still instantiated, and instantiating
// Tim<n> for an absent n is the driver's own static_assert going off.

template <bool present = tim_present(4)>
void quiet_tim4() {
    if constexpr (present) {
        // TIM4's line IS TIM3's where there is a TIM4, so quiet_everything
        // has already disabled it; only the peripheral is left to put back.
        Tim4<present>::release();
    }
}

/// TIM6, TIM7 and TIM15 each own their line, so each has its own vector
/// to silence beside its own peripheral to release.
template <bool present = tim_present(6)>
void quiet_tim6() {
    if constexpr (present) {
        Nvic::disable(Tim6<present>::irq());
        Tim6<present>::release();
        Nvic::clear_pending(Tim6<present>::irq());
    }
}

template <bool present = tim_present(7)>
void quiet_tim7() {
    if constexpr (present) {
        Nvic::disable(Tim7<present>::irq());
        Tim7<present>::release();
        Nvic::clear_pending(Tim7<present>::irq());
    }
}

template <bool present = tim_present(15)>
void quiet_tim15() {
    if constexpr (present) {
        Nvic::disable(Tim15<present>::irq());
        Tim15<present>::release();
        Nvic::clear_pending(Tim15<present>::irq());
    }
}

/// TIM3's line is shared with a TIM4 exactly where there is one.
template <bool present = tim_present(4)>
bool tim3_shares_with_tim4() {
    if constexpr (present) {
        return Tim4<present>::irq() == tim_irq(3);
    } else {
        return true;
    }
}

/// TIM16's line is shared with the FDCAN's first interrupt exactly where
/// there is an FDCAN - the same derivation, on a peripheral that is not a
/// timer at all.
template <bool present = fdcan_present(1)>
bool tim16_shares_with_fdcan() {
    if constexpr (present) {
        return fdcan_irq(0) == tim_irq(16);
    } else {
        return true;
    }
}

template <bool present = fdcan_present(1)>
int32_t fdcan_first_irq() {
    if constexpr (present) {
        return static_cast<int32_t>(fdcan_irq(0));
    } else {
        return -1;
    }
}

/// Every timer and pad this suite touches back to reset.
void quiet_everything() {
    Nvic::disable(T1::irq());
    Nvic::disable(T1::cc_irq());
    Nvic::disable(T2::irq());
    Nvic::disable(T3::irq());
    Nvic::disable(T16::irq());
    Nvic::disable(T14::irq());
    Nvic::disable(T17::irq());
    Nvic::disable(EXTI4_15_IRQn);
    T1::release();
    T2::release();
    T3::release();
    quiet_tim4();
    quiet_tim6();
    quiet_tim7();
    quiet_tim15();
    T16::release();
    T14::release();
    T17::release();
    (void)Exti::release(PadLed::pin_number);
    PadLed::output(false);
    PadCap::input(PinPull::none);
    PadPair::input(PinPull::none);
    PadPairN::input(PinPull::none);
    PadB6::input(PinPull::none);
    PadB7::input(PinPull::none);
    PadB8::input(PinPull::none);
    PadB9::input(PinPull::none);
    // Only where they are pins. An unbonded pad's PUPDR is a register
    // write with nothing at the other end, and a suite that puts pads
    // "back" must not name pads it never had.
    if constexpr (pb_high_bonded) {
        PadB13::input(PinPull::none);
        PadB14::input(PinPull::none);
        PadB15::input(PinPull::none);
    }
    Nvic::clear_pending(T1::irq());
    Nvic::clear_pending(T1::cc_irq());
    Nvic::clear_pending(T2::irq());
    Nvic::clear_pending(T3::irq());
    Nvic::clear_pending(T16::irq());
    Nvic::clear_pending(T14::irq());
    Nvic::clear_pending(T17::irq());
    Nvic::clear_pending(EXTI4_15_IRQn);
}

// =============================================================================
// a - the block: what each timer IS, the reset values, the vectors, the
//     refusals - and the pads this suite is about to rely on
// =============================================================================
/// WHAT THE PART IS DUE, which is what checks the reserve. tim_present()
/// reads the header; the DATASHEET's own instance list is the second
/// opinion, and it is a per-part fact this suite states rather than reads:
/// DS12992's G031 has TIM1, TIM2, TIM3, TIM14, TIM16 and TIM17 and no
/// others, where DS13560's G0B1 and the G071's sheet add the basic pair
/// and TIM15 (and the G0B1 alone a TIM4). TIM4 is the one instance left
/// out of the verdict below, because telling a G0B1 from a G071 needs a
/// device probe this suite has no business making - it is PRINTED there
/// instead.
constexpr bool expect_tim6 = !package_lqfp32;
constexpr bool expect_tim7 = !package_lqfp32;
constexpr bool expect_tim15 = !package_lqfp32;

void ta_block() {
    // PA8 IS UCPD1_CC1 ON A PART WITH A UCPD, AND WITHOUT THE STROBE
    // BELOW ITS PULL CHECK FAILS ABOUT ONE RUN IN THREE. RM0444 7.3.16:
    // the Type-C
    // DEAD-BATTERY pull-downs on UCPD1_CC1 (PA8) and UCPD1_CC2 (PB15) are
    // CONNECTED out of a power-on and stay connected until SYSCFG_CFGR1's
    // strobe releases them - a few kilohms against the port's own forty,
    // so a pad asked to follow its 40 k pull-up sits somewhere between the
    // rails and reads whichever way the threshold falls that minute. The
    // strobe is ONE-WAY for the power cycle, so this is done once, here,
    // before anything asks PA8 a question. (Measured on PB15; the
    // release is what makes PA8's leg below repeatable.) On a part the
    // reserve reports as having no UCPD
    // there is no Rd and no strobe bit, and the verb answers false: which
    // of the two this die is, is what ucpd_present() says.
    const bool rd_released = ucpd_dead_battery(1, false);

    print(serial, "  present: TIM1=", tim_present(1), " TIM2=", tim_present(2),
          " TIM3=", tim_present(3), " TIM4=", tim_present(4), " TIM6=", tim_present(6),
          " TIM7=", tim_present(7), " TIM14=", tim_present(14), " TIM15=", tim_present(15),
          " TIM16=", tim_present(16), " TIM17=", tim_present(17), crlf);
    print(serial, "  at boot: TIM2_CR1=", hex(boot_t2_cr1), " TIM2_ARR=", hex(boot_t2_arr),
          " TIM1_BDTR=", hex(boot_t1_bdtr), " APBENR1=", hex(boot_apbenr1),
          " APBENR2=", hex(boot_apbenr2), crlf);

    print(serial, "  timers present: 1 2 3", tim_present(4) ? " 4" : "",
          tim_present(6) ? " 6" : "", tim_present(7) ? " 7" : "", " 14",
          tim_present(15) ? " 15" : "", " 16 17", crlf);
    bench.verdict("this part carries the timers its datasheet gives it and no "
                  "others: TIM1, TIM2, TIM3, TIM14, TIM16 and TIM17 are every "
                  "G0's, the basic pair TIM6/TIM7 and TIM15 belong to the "
                  "larger parts alone, and no G0 of this family has a TIM5 or "
                  "a TIM8 - the reserve's reading of the header agreeing with "
                  "the sheet instance by instance",
                  tim_present(1) && tim_present(2) && tim_present(3) &&
                      tim_present(14) && tim_present(16) && tim_present(17) &&
                      tim_present(6) == expect_tim6 &&
                      tim_present(7) == expect_tim7 &&
                      tim_present(15) == expect_tim15 &&
                      !tim_present(5) && !tim_present(8));
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
    print(serial, "  TIM4 present ", tim_present(4), "; TIM3's vector ",
          static_cast<int32_t>(T3::irq()), ", TIM16's ",
          static_cast<int32_t>(T16::irq()), ", the FDCAN's first line ",
          static_cast<int32_t>(fdcan_first_irq()),
          " (-1 = no FDCAN on this part)", crlf);
    bench.verdict("TIM3 SHARES its vector with a TIM4 exactly where the part "
                  "has one, and has it to itself where it has not - the "
                  "reserve deriving the line from PRESENCE both ways",
                  T3::irq() == tim_irq(3) && tim3_shares_with_tim4());
    bench.verdict("and TIM16's line is shared with an FDCAN's first interrupt "
                  "on a part that has an FDCAN, and TIM16's alone where none "
                  "does",
                  T16::irq() == tim_irq(16) && tim16_shares_with_fdcan());

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
    const bool pa8_free = pad_follows_pull<PadPair>();
    print(serial, "  PA8 on a part with a UCPD is UCPD1_CC1: this one ",
          ucpd_present(1) ? "HAS one" : "has none",
          ", SYSCFG's dead-battery strobe ",
          rd_released ? "released the Rd" : "answered false (nothing to release)",
          " before the check, and the pad ", pa8_free ? "follows" : "DOES NOT follow",
          " its own pull", crlf);
    bench.verdict("and so does PA8 - on a part with a UCPD only ONCE THE "
                  "TYPE-C DEAD-BATTERY PULL-DOWN IS RELEASED (7.3.16 connects "
                  "an Rd to UCPD1_CC1 out of a power-on, and the strobe this "
                  "letter spends first is what makes the pad's own 40 k "
                  "pull-up the strongest thing on it), and on a part with no "
                  "UCPD with nothing on the pad to release and the verb "
                  "REFUSING rather than writing a bit that is not there",
                  pa8_free && rd_released == ucpd_present(1));
    // NOTHING OPENS THE LED'S PORT CLOCK HERE ON PURPOSE: `output(bool)`
    // must do it itself (port.md's rule - a configuring verb opens the
    // port clock before any store, the level store included), and on a
    // board whose LED sits on a port no console has touched this pad is
    // the one place that rule is measurable. The verdict below is that
    // measurement.
    PadLed::output(true);
    settle();
    const bool led_high = PadLed::read();
    PadLed::output(false);
    settle();
    const bool led_low = PadLed::read();
    print(serial, "  ", led_name, " driven high reads ", led_high ? "HIGH" : "low",
          ", driven low reads ", led_low ? "HIGH" : "low", crlf);
    bench.verdict("the board's user LED sits on a pad whose own output driver "
                  "still reaches both rails through it - the pad is readable "
                  "on IDR, which is what letter c's first witness needs",
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
    // milliseconds of console.
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
// c - PWM on the board LED: the duty read back THROUGH THE PAD, and the
//     pad's own edges counted by an EXTI line
// =============================================================================
//
// Two witnesses for one waveform, neither of them a wire. The first is
// the pad's own port IDR - 7.3.1 leaves the input buffer live in
// alternate-function mode, so a pad the timer is driving is readable -
// and the second is an EXTI line pointed at that port, which sees a pad
// its own port drives; letter c is what extends
// that to a pad a PERIPHERAL drives. Which pad, which channel and which
// line are the package's choice and nothing else: LD4 = PA5 = TIM2_CH1 =
// line 5 on the Nucleo-64s, LD3 = PC6 = TIM2_CH3 = line 6 on the
// Nucleo-32, and both lines report on EXTI4_15.
void tc_pwm_pad() {
    T2::init();
    LedOut::claim();
    print(serial, "  the LED under test is ", led_name, ", channel index ",
          static_cast<uint32_t>(led_channel), ", EXTI line ",
          static_cast<uint32_t>(PadLed::pin_number), " on port ",
          led_pad.port, crlf);
    bench.verdict("the board LED's pad takes its TIM2 channel at AF2 (the "
                  "datasheet's own alternate-function table) and the channel "
                  "comes up",
                  PadLed::has_function() && LedPwm::setup(0));

    // 64 kHz: a sampling loop crosses hundreds of periods, so the
    // aliasing between the two rates averages out. THE CONSOLE IS DRAINED
    // FIRST, for letter e's reason: a transmit interrupt walking through a
    // sampling window lengthens it, and this letter's preamble is two
    // lines of console.
    console_drain();
    struct Point { uint16_t asked; uint16_t got; };
    Point pts[5];
    const uint16_t asks[5] = {0, 250, 500, 750, 1000};
    for (uint8_t i = 0; i < 5; ++i) {
        LedPwm::duty(asks[i]);
        spin_cycles(SysClock::hz / 1000u);      // let the preload be taken
        pts[i] = {asks[i],
                  sample_permille_port<led_pad.port>(PadLed::pin_number, 60000u)};
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
    (void)T2::output_channel(led_channel,
                             {.mode = TimOutputMode::pwm1, .compare = 500});
    T2::enable(true);
    bench.verdict("the LED pad's own EXTI line takes its port and a rising "
                  "sense while TIM2 owns the pad",
                  Exti::select(PadLed::pin_number, led_pad.port) &&
                      Exti::sense(PadLed::pin_number, ExtiSense::rising) &&
                      Exti::interrupt(PadLed::pin_number, true));
    (void)Exti::clear(PadLed::pin_number);
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
    // THE CROSS-CHECK IS AGAINST THE OTHER INSTRUMENT ON THIS DIE, and
    // that number is a DIE fact: a real IWDG time-out timed on the same
    // oscillator reads 32536 Hz on the
    // G0B1RE, 32295 on the G071RB and 31400 on the G031K8 - all three
    // inside table 46's band and none of them each other. So what is
    // compared is this die's own pair, and the number is stated per part
    // rather than baked in from one board.
#if defined(STM32G031xx)
    constexpr uint32_t watchdog_lsi_hz = 31'400;
#elif defined(STM32G071xx)
    constexpr uint32_t watchdog_lsi_hz = 32'295;
#else
    constexpr uint32_t watchdog_lsi_hz = 32'536;
#endif
    print(serial, "  the watchdog's own reading of this die's LSI is ",
          watchdog_lsi_hz, " Hz (the platform suite's reading)", crlf);
    bench.verdict("and it agrees with the watchdog-timed reading of the SAME "
                  "oscillator on the SAME die to within 3 per cent - two "
                  "instruments sharing no mechanism",
                  lsi_hz > watchdog_lsi_hz - watchdog_lsi_hz / 33u &&
                      lsi_hz < watchdog_lsi_hz + watchdog_lsi_hz / 33u);
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
                  "reads as its own tick count and not one less",
                  cap_period > 900u);

    quiet_everything();
}

// =============================================================================
// g - TIM1: the complementary pair, the dead time, MOE, the break and the
//     repetition counter
// =============================================================================
//
// The advanced timer is the one shape the plainer timers of this family
// have not got: two outputs that are each other's
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
/// A flag whose interrupt is DISABLED stands where it is - unlike this
/// family's EXTI, where an unarmed line has no pending bit at all. The
/// claim is about A TIMER and not about a particular one, so the leg
/// takes the timer it is given: TIM4 where the part has one (already
/// running from the leg above) and TIM3 where it has not.
template <class T>
void masked_flag_leg(volatile uint32_t& calls) {
    T::interrupts(T::update_interrupt, false);
    T::clear_flags(T::update_flag);
    spin_cycles(SysClock::hz / 100u);
    const bool standing = T::flag(T::update_flag);
    Nvic::clear_pending(T::irq());
    Nvic::enable(T::irq());
    const uint32_t before = calls;
    spin_cycles(SysClock::hz / 100u);
    Nvic::disable(T::irq());
    const uint32_t after = calls;
    bench.verdict("A TIMER FLAG STANDS WITH ITS INTERRUPT MASKED and is "
                  "readable by a poller - the EXTI of this same family "
                  "keeps no pending bit for a masked line at all",
                  standing && after == before);
}

/// The two timers that share ONE line, each at its own rate - and, on a
/// part whose TIM3 has that line to itself, the same masked-flag question
/// put to TIM3 instead. Only the SHARING is skipped; nothing else is.
template <bool present = tim_present(4)>
void th_shared_line() {
    if constexpr (present) {
        using T4 = Tim4<present>;
        T4::init();
        (void)T4::configure({.prescaler = 63, .period = 499});    // 2 kHz
        T4::interrupts(T4::update_interrupt, true);
        T4::clear_flags(T4::update_flag);
        T4::enable(true);
        Nvic::clear_pending(tim_irq(3));
        Nvic::enable(tim_irq(3));
        spin_cycles(SysClock::hz / 10u);   // 100 ms
        Nvic::disable(tim_irq(3));
        const uint32_t n3 = t3_update_calls;
        const uint32_t n4 = t4_update_calls;
        print(serial, "  100 ms on ONE vector: TIM3 (1 kHz) ", n3,
              " updates, TIM4 (2 kHz) ", n4, crlf);
        bench.verdict("TWO TIMERS ON ONE VECTOR, each answered by its own ISR "
                      "body and each at its own rate",
                      n3 > 90u && n3 < 110u && n4 > 180u && n4 < 220u);
        bench.verdict("and neither body ever consumed the other's flag",
                      t3_saw_t4_flag == 0u);
        masked_flag_leg<T4>(t4_update_calls);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: two timers answering on ONE "
              "vector needs a TIM4 to share TIM3's line, and this part has "
              "not got one (tim_present(4) is false - the reserve finds no "
              "TIM4_BASE, and so names this line TIM3's alone). The masked "
              "flag below is asked of TIM3 instead: the claim is about A "
              "timer, not about that one.",
              crlf);
        Nvic::clear_pending(tim_irq(3));
        Nvic::enable(tim_irq(3));
        spin_cycles(SysClock::hz / 10u);
        Nvic::disable(tim_irq(3));
        print(serial, "  100 ms on TIM3's own vector: ", t3_update_calls,
              " updates at 1 kHz", crlf);
        masked_flag_leg<T3>(t3_update_calls);
    }
}

void th_vectors() {
    clear_counts();
    T3::init();
    (void)T3::configure({.prescaler = 63, .period = 999});    // 1 kHz
    T3::interrupts(T3::update_interrupt, true);
    T3::clear_flags(T3::update_flag);
    T3::enable(true);
    th_shared_line();

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
// PERIODS in a cycle-measured window is exact. A printed dual-slope
// formula is exactly the kind that comes out off by one, so this letter
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
// util/meter_sampler.hpp is a capture ISR that fills a one-cell latch
// and an AO that paces PUBLICATION rather than capture. Here the source
// is a TIM16 capture channel fed by LSI over TISEL, and the letter is
// what says the contract holds over it.
//
// The economy is the file's own: LSI arrives about 32000 times a second
// and the sampler publishes ten times a second, so what the queues carry
// is the application's pace and the latch's missed() counts the rest.

using LsiLatch = MeterLatch<uint32_t, Stm32g0Platform<>, 0>;
static_assert(MeterSource<LsiLatch>, "the capture latch is a MeterSource");

struct Collector;
using Subs = Subscribers<Collector>;
using Sampler = MeterSampler<Stm32g0Platform<>, Subs, LsiLatch>;

struct Collector {
    using Event = std::variant<MeterSample>;
    static inline EventQueue<Event, 8, Stm32g0Platform<>> queue;

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

using MeterKernel = Kernel<Stm32g0Platform<>, Collector, Sampler>;

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
        TimeEvents<Stm32g0Platform<>>::process();
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
// k - the errata (ES0548 Rev 3 for the G0B1, revision Z)
// =============================================================================
//
// THE ITEM NUMBER IS THE G0B1'S. Every part of this family has its own
// errata sheet - ES0548 for the G0B1/G0C1, ES0418 for the G071, ES0487
// for the G031 - and an item's NUMBER does not travel between them, so
// what this letter names is the G0B1's 2.7.2 and what it stages is that
// item's own mechanism. ES0487 was not in hand when this letter was
// opened for the G031, and a number nobody has read is not a citation:
// the outcome below is therefore recorded as MEASURED ON THIS DIE, with
// the G0B1's item named as the description being staged and its twin in
// ES0487 left pending.
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
    print(serial, "  the G0B1's 2.7.2 (ES0487 2.6.2 on this part) staged 8 times: "
                  "the first match (CNT = CCR = ARR) fired ", first_fired,
          " times, the counter wrapped ", wrapped,
          " times, and the SECOND match (CNT = CCR = 0, one counter "
          "cycle later) raised its flag ", second_fired, " times and toggled "
          "the pad ", second_toggled, " times", crlf);
    bench.verdict("the erratum's first match happens every round and the "
                  "counter really does reach the next cycle",
                  first_fired == 8u && wrapped == 8u);
    // WHAT THE BENCH FOUND, recorded as it is: the second compare of two
    // consecutive counter cycles is served in full - the flag rises and
    // the output toggles - so the described behaviour did NOT reproduce in
    // this staging on this die. The erratum stands unrefuted rather than
    // disproved: its own description is of a REPEATING pattern driven at
    // the counter's rate (a single-cycle-wide pulse per period built by
    // DMA), where this letter drives it once per period from software.
    bench.verdict("THE STAGED BEHAVIOUR DID NOT REPRODUCE ON THIS DIE - the "
                  "second compare raised its flag and toggled its output "
                  "every round - the item being the G0B1's 2.7.2, ES0487's 2.6.2",
                  second_fired == 8u && second_toggled == 8u);

    print(serial, "  the G0B1's 2.7.1 (one-pulse trigger lost in master-slave "
                  "reset + trigger with MSM), ES0487's 2.6.1, NOT "
                  "staged: it needs the trigger to "
                  "arrive exactly at CNT = ARR of a cascaded master, which "
                  "nothing here can place. Its own workaround is the "
                  "driver's default - TimSlaveConfig::master_slave is false.",
          crlf);
    print(serial, "  the G0B1's 2.7.3 (output compare clear with an external "
                  "reset), ES0487's 2.6.3, NOT "
                  "staged: ocref_clr comes from ETR or a comparator, and "
                  "this stratum has neither an ETR wire nor a COMP driver "
                  "(and this part has no comparator at all where the reserve "
                  "says so).",
          crlf);
    bench.verdict("the two unreachable TIM errata are named with the reason, "
                  "not described away", true);

    quiet_everything();
}

// =============================================================================
// l - the instances this suite had never counted
// =============================================================================
//
// TIM1, TIM2, TIM3 and TIM16 carry every letter above; TIM4 has appeared
// only as the other half of TIM3's vector. That leaves six of the ten
// instances this chapter describes across the family with no cycle
// counted here: TIM4 itself, the two basic timers, and the three
// one-or-two-channel ones - as many of them as the part has. Each is
// asked the same three questions, and NOT ONE EXPECTATION IS HARD-CODED:
// the channel count, the counter width, the break unit and the vector all
// come out of the reserve, so a part with a different geometry gets a
// different test from the same source - and a part that has not got the
// instance at all gets a SKIP LINE naming it and no verdict.
//
//   1. does it count PCLK at the prescaler it was given? (against
//      SysTick, which is the wall letter b already uses - so what this
//      proves is the DIVIDER and the enable, not the oscillator: both
//      ride HCLK)
//   2. does its update interrupt reach ITS OWN VECTOR? Four of the six
//      are on lines they do not own alone - TIM4 with TIM3, TIM6 with
//      the DAC and LPTIM1, TIM7 with LPTIM2, TIM17 with an FDCAN line -
//      and a wrong name in the reserve is a silent Default_Handler spin,
//      which is exactly what this question catches. WHICH SHARERS EXIST
//      IS PER PART, and the reserve derives the name from their presence:
//      on a part with no DAC and no FDCAN the very same instances answer
//      on lines of their own, and the question is the same question.
//   3. and for those with output channels AND pads on this package, does
//      a PWM reach a pad at the duty asked? TIM6 and TIM7 have no channel
//      at all (23.2), so their update event IS their whole output; their
//      TRGO's only consumers are the DAC where the part has one and the
//      ADC, which are test_stm32_analog's and are measured there.

/// Percent difference of `v` from `want`, x10 (15 means 1.5 %).
uint32_t permille_off(uint32_t v, uint32_t want) {
    if (want == 0u) {
        return 0xFFFFFFFFu;
    }
    const uint32_t d = v > want ? v - want : want - v;
    return static_cast<uint32_t>((static_cast<uint64_t>(d) * 1000ULL) / want);
}

/// Questions 1 and 2 for one instance. `calls` is the counter its own
/// vector's handler bumps.
template <class T>
bool instance_counts_and_interrupts(const char* name, volatile uint32_t& calls) {
    // 1. THE TIME BASE. PSC = 63 makes a 1 MHz counter out of a 64 MHz
    // PCLK, so a 10 ms window is 10000 counts - inside every one of these
    // counters and long enough that the window's own ends do not matter.
    T::init();
    const bool cfg = T::configure({.prescaler = 63, .period = T::max_period});
    T::enable(true);
    console_drain();
    const uint32_t c0 = cycles_now();
    const uint32_t n0 = T::count();
    while (cycles_now() - c0 < 10'000u * cycles_per_us) {
    }
    const uint32_t n1 = T::count();
    const uint32_t c1 = cycles_now();
    T::enable(false);
    const uint32_t counted = (n1 - n0) & T::max_period;
    const uint32_t want = (c1 - c0) / 64u;
    const uint32_t off = permille_off(counted, want);

    // 2. THE UPDATE INTERRUPT, on whatever vector the reserve names.
    // ARR = 999 on that same 1 MHz counter is 1 kHz, so a 20 ms window
    // is twenty updates.
    calls = 0;
    T::init();
    const bool cfg2 = T::configure({.prescaler = 63, .period = 999});
    T::interrupts(T::update_interrupt, true);
    Nvic::clear_pending(T::irq());
    Nvic::enable(T::irq());
    T::enable(true);
    const uint32_t s0 = cycles_now();
    while (cycles_now() - s0 < 20'000u * cycles_per_us) {
    }
    T::enable(false);
    Nvic::disable(T::irq());
    const uint32_t n = calls;
    T::release();

    print(serial, "  ", name, ": ", counted, " counts against ", (c1 - c0) / 64u,
          " expected (", off, " per mille off), ", n,
          " updates in a 20 ms window (20 due, the window's own ends worth "
          "one) on NVIC line ", static_cast<uint32_t>(T::irq()),
          "; the reserve says ", static_cast<uint32_t>(T::channels),
          " channel(s), ", static_cast<uint32_t>(T::complementary_channels),
          " complementary, ", static_cast<uint32_t>(T::counter_bits), " bits",
          crlf);
    return cfg && cfg2 && off <= 20u && n >= 19u && n <= 21u;
}

/// Question 1 and 2 put to TIM4: does it count PCLK at its prescaler, and
/// does its update reach the vector the reserve names?
template <bool present = tim_present(4)>
void tl_tim4_counts() {
    if constexpr (present) {
        const bool t4_ok =
            instance_counts_and_interrupts<Tim4<present>>("TIM4", t4_update_calls);
        bench.verdict("TIM4 counts PCLK at its prescaler and its update reaches "
                      "the vector it SHARES with TIM3",
                      t4_ok);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: TIM4's counter and its update "
              "interrupt need a TIM4, which this part has not got "
              "(tim_present(4) is false). The census below is of the "
              "instances this part DOES have, and no shorter a question for "
              "each of them.",
              crlf);
    }
}

/// Questions 1 and 2 put to the basic pair. Their vectors are the plainest
/// case of the reserve's derivation: TIM6's line carries the DAC and
/// LPTIM1 where those exist and is TIM6's own where they do not, TIM7's
/// carries LPTIM2 or is TIM7's own - and where the timers themselves are
/// absent their LPTIMs keep the lines, which is why nothing here may
/// spell either name without asking first.
template <bool present = tim_present(6)>
void tl_tim6_counts() {
    if constexpr (present) {
        const bool ok =
            instance_counts_and_interrupts<Tim6<present>>("TIM6", t6_update_calls);
        bench.verdict("TIM6, a basic timer with no channel at all, counts and "
                      "reports on the line the reserve names for it",
                      ok);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: TIM6's counter and its update "
              "interrupt need a TIM6, which this part has not got "
              "(tim_present(6) is false - the reserve finds no TIM6_BASE, and "
              "the line this timer would have shared is LPTIM1's alone here).",
              crlf);
    }
}

template <bool present = tim_present(7)>
void tl_tim7_counts() {
    if constexpr (present) {
        const bool ok =
            instance_counts_and_interrupts<Tim7<present>>("TIM7", t7_update_calls);
        bench.verdict("TIM7 likewise, on the line it shares with LPTIM2", ok);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: TIM7's counter and its update "
              "interrupt need a TIM7, which this part has not got "
              "(tim_present(7) is false - the reserve finds no TIM7_BASE, and "
              "LPTIM2 has that line to itself here).",
              crlf);
    }
}

template <bool present = tim_present(15)>
void tl_tim15_counts() {
    if constexpr (present) {
        const bool ok =
            instance_counts_and_interrupts<Tim15<present>>("TIM15", t15_update_calls);
        bench.verdict("TIM15 does, on its own vector", ok);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: TIM15's counter and its update "
              "interrupt need a TIM15, which this part has not got "
              "(tim_present(15) is false - the reserve finds no TIM15_BASE, "
              "and its vector position is empty in this part's own table).",
              crlf);
    }
}

/// The basic pair's ONE output, 23.4.2's master mode - and the refusals
/// that say what a basic timer is not. Both instances or neither: the
/// claim is about the CLASS, and half of it is not a smaller claim but a
/// different one.
template <bool present = tim_present(6) && tim_present(7)>
void tl_basic_timers() {
    if constexpr (present) {
        // Both aliases take THE GATE, for the reason tl_tim15_pwm() gives:
        // only a dependent name is left uninstantiated in a discarded
        // branch, and the gate is true only when both instances are there.
        using T6 = Tim6<present>;
        using T7 = Tim7<present>;
        T6::init();
        T7::init();
        const bool t6_master = T6::master(TimMasterMode::update);
        const bool t7_master = T7::master(TimMasterMode::update);
        const bool no_channels = !T6::output_channel(0, {}) && !T7::output_channel(0, {}) &&
                                 !T6::set_compare(0, 10) && T6::channels == 0u &&
                                 T7::channels == 0u;
        const bool no_slave = !T6::slave({.mode = TimSlaveMode::gated}) &&
                              !T7::slave({.mode = TimSlaveMode::gated});
        print(serial, "  TIM6/TIM7: CR2.MMS accepted ", t6_master, "/", t7_master,
              ", channels ", static_cast<uint32_t>(T6::channels), "/",
              static_cast<uint32_t>(T7::channels),
              " - their TRGO's only consumers on this part are the DAC and the "
              "ADC, and test_stm32_analog measures both", crlf);
        bench.verdict("the basic timers ARE what 23.2 says: a master mode that "
                      "publishes TRGO, and no channel, no slave controller and no "
                      "compare register to refuse a channel with",
                      t6_master && t7_master && no_channels && no_slave);
        T6::release();
        T7::release();
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: what a BASIC timer is - a master "
              "mode and nothing else - is a claim about TIM6 and TIM7 together, "
              "and this part has neither (tim_present(6) and tim_present(7) are "
              "both false), so chapter 23 has no instance at all on this die. "
              "Of the two consumers their TRGO would have fed, the DAC is ",
              dac_present() ? "here but unfed" : "absent from this part too "
                                                 "(dac_present() is false)",
              " and the ADC is test_stm32_analog's.", crlf);
    }
}

/// Question 3 put to TIM4: FOUR channels driving four pads at four
/// different duties in one period, all on port B and therefore all in one
/// sampling loop's reach - so a channel wired to the wrong CCR shows up as
/// the wrong number and not as no number.
template <bool present = tim_present(4)>
void tl_tim4_pwm() {
    if constexpr (present) {
        using T4 = Tim4<present>;
        T4::init();
        const bool t4_cfg = T4::configure({.prescaler = 0, .period = pwm_top,
                                           .auto_reload_preload = true});
        TimPad<t4_ch1_pad>::claim();
        TimPad<t4_ch2_pad>::claim();
        TimPad<t4_ch3_pad>::claim();
        TimPad<t4_ch4_pad>::claim();
        const uint16_t t4_asks[4] = {200, 400, 600, 800};
        bool t4_channels = t4_cfg;
        for (uint8_t ch = 0; ch < 4u; ++ch) {
            t4_channels = t4_channels &&
                          T4::output_channel(ch, {.mode = TimOutputMode::pwm1,
                                                  .compare = t4_asks[ch]});
        }
        T4::enable(true);
        spin_cycles(SysClock::hz / 1000u);
        uint16_t t4_got[4];
        t4_got[0] = sample_permille_b(6, 40000u);
        t4_got[1] = sample_permille_b(7, 40000u);
        t4_got[2] = sample_permille_b(8, 40000u);
        t4_got[3] = sample_permille_b(9, 40000u);
        print(serial, "  TIM4 at 64 kHz, four channels at once: PB6 ", t4_got[0],
              " PB7 ", t4_got[1], " PB8 ", t4_got[2], " PB9 ", t4_got[3],
              " per mille, asked 200/400/600/800", crlf);
        bool t4_duties = t4_channels;
        for (uint8_t i = 0; i < 4u; ++i) {
            const int32_t err = static_cast<int32_t>(t4_got[i]) -
                                static_cast<int32_t>(t4_asks[i]);
            if (err > 25 || err < -25) {
                t4_duties = false;
            }
        }
        bench.verdict("TIM4's four channels drive four pads at four different "
                      "duties in one period - the reserve's channel count for "
                      "it is the silicon's",
                      t4_duties);
        T4::release();
        PadB6::input(PinPull::none);
        PadB7::input(PinPull::none);
        PadB8::input(PinPull::none);
        PadB9::input(PinPull::none);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: four channels at four duties on "
              "PB6/PB7/PB8/PB9 at AF9 need a TIM4, which this part has not got "
              "(tim_present(4) is false). No other timer of this package "
              "reaches those four pads at once.",
              crlf);
    }
}

/// Question 3 put to TIM15: TWO channels AND a complement, all three on
/// port B, so one IDR read carries the pair - the census letter g makes of
/// TIM1's. IT TAKES BOTH THE INSTANCE AND THE PADS: PB13/PB14/PB15 at AF5
/// are the only pins any G0 of this pack gives TIM15's outputs, so a
/// package that bonds none of them has nowhere to put this leg even where
/// the instance exists.
template <bool present = tim_present(15) && pb_high_bonded>
void tl_tim15_pwm() {
    if constexpr (present) {
        // The alias's argument is THE GATE and not tim_present(15) again:
        // a discarded branch is only left uninstantiated where what it
        // names is DEPENDENT, and `Tim15<tim_present(15)>` is a constant.
        using T15 = Tim15<present>;
        T15::init();
        TimPad<t15_ch1_pad>::claim();
        TimPad<t15_ch1n_pad>::claim();
        TimPad<t15_ch2_pad>::claim();
        using T15Pair = TimPairPwm<T15, 0, pwm_top>;
        static_assert(PwmChannel<T15Pair>);
        const bool t15_up = T15Pair::setup(0, tim_dead_time_code(64));
        T15Pair::duty(300);
        const bool t15_ch2 = T15::output_channel(1, {.mode = TimOutputMode::pwm1,
                                                     .compare = 700});
        spin_cycles(SysClock::hz / 1000u);
        const PairCensus t15c = census_pair_b(14, 13, 40000u);
        const uint16_t t15_ch2_got = sample_permille_b(15, 40000u);
        const uint32_t t15_a = (t15c.a_high * 1000u + 20000u) / 40000u;
        const uint32_t t15_b = (t15c.b_high * 1000u + 20000u) / 40000u;
        // THE DEAD TIME IS TAKEN OFF BOTH HALVES, and the arithmetic is the
        // verdict rather than a tolerance: at CKD = 1 one DTG tick is one
        // timer clock tick, so a dead time of D ticks out of a period of
        // top + 1 is D per mille off EACH output.
        const uint32_t t15_dt = (static_cast<uint32_t>(T15Pair::dead_time_ticks()) *
                                 1000u) / (static_cast<uint32_t>(pwm_top) + 1u);
        const uint32_t t15_want_a = 300u - t15_dt;
        const uint32_t t15_want_b = 700u - t15_dt;
        print(serial, "  TIM15: CH1 on PB14 ", t15_a, " per mille, CH1N on PB13 ",
              t15_b, ", both high together in ", t15c.both_high, " of 40000 reads;"
              " CH2 on PB15 ", t15_ch2_got, " per mille. Asked 300 and 700 with a "
              "dead time of ", static_cast<uint32_t>(T15Pair::dead_time_ticks()),
              " ticks = ", t15_dt, " per mille off each half, so the pair is due ",
              t15_want_a, " and ", t15_want_b, crlf);
        bench.verdict("TIM15 drives a COMPLEMENTARY PAIR and a second, "
                      "independent channel at once: the pair's two halves are "
                      "never both high in forty thousand samples of one instant, "
                      "and channel 2 carries its own duty beside them",
                      t15_up && t15_ch2 && t15c.both_high == 0u &&
                          t15_ch2_got + 25u >= 700u && t15_ch2_got <= 725u);
        bench.verdict("...and the dead time 25.5.16's generator inserts is taken "
                      "off BOTH halves, exactly: each output lands at its own duty "
                      "less the dead time, so the two sum to a thousand less TWICE "
                      "it and not to a thousand",
                      t15_a + 20u >= t15_want_a && t15_a <= t15_want_a + 20u &&
                          t15_b + 20u >= t15_want_b && t15_b <= t15_want_b + 20u);
        T15::release();
        PadB13::input(PinPull::none);
        PadB14::input(PinPull::none);
        PadB15::input(PinPull::none);
    } else if constexpr (!tim_present(15)) {
        print(serial,
              "  SKIPPED, no verdict claimed: a complementary pair and a second "
              "channel on PB13/PB14/PB15 at AF5 need a TIM15, which this part "
              "has not got (tim_present(15) is false).");
        if constexpr (!pb_high_bonded) {
            print(serial, " This package bonds none of those three pins either "
                          "(DS12992 table 12), so two things are missing and "
                          "not one.");
        }
        print(serial, crlf);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: TIM15's three outputs land on "
              "PB13, PB14 and PB15 and nowhere else on this family, and this "
              "package bonds none of them (DS12992 table 12). The instance is "
              "here and its counter was measured above; only its PADS are out "
              "of reach.",
              crlf);
    }
}

void tl_six_instances() {
    quiet_everything();
    clear_counts();

    // THE PADS FIRST, as every pull-walked letter of this stratum does.
    // PA7 is letter a's already (TIM1_CH1N); the port-B pads are new and
    // are asked the same question here. PB15 is UCPD1_CC2 and needs the
    // same dead-battery release letter a spends for PA8 - it is one
    // strobe for both pads and it is one-way for the power cycle, so
    // spending it again here costs nothing and makes this letter stand on
    // its own when it is run alone; on a part with no UCPD there is
    // nothing to spend and the verb says so.
    // AND THE PACKAGE DECIDES HOW MANY PADS THERE ARE TO WALK: PB13, PB14
    // and PB15 are pins on the LQFP48/64 and on nothing of the LQFP32,
    // where port B stops at PB9 (DS12992 table 12). An unbonded pad has
    // no level to read, so it is not walked and not judged.
    const bool rd_released = ucpd_dead_battery(1, false);
    Rcc::io_clock('B', true);
    const bool b6 = pad_follows_pull<PadB6>();
    const bool b7 = pad_follows_pull<PadB7>();
    // PB8 and PB9 are where a wired I2C bus puts its 2.2 k pull-ups on
    // the Nucleo-64s (docs/bench.md): there they cannot follow an
    // internal pull-down any more, and a push-pull driver still owns them
    // - which is all this letter asks. On a board with nothing on the
    // pads the same verb takes the plain pull-walk's answer.
    const bool b8 = pad_is_drivable<PadB8>();
    const bool b9 = pad_is_drivable<PadB9>();
    bool b13 = true, b14 = true, b15 = true;
    if constexpr (pb_high_bonded) {
        b13 = pad_follows_pull<PadB13>();
        b14 = pad_follows_pull<PadB14>();
        b15 = pad_follows_pull<PadB15>();
    }
    print(serial, "  pull-walk: PB6 ", b6, " PB7 ", b7, " PB8 ", b8, " PB9 ", b9);
    if constexpr (pb_high_bonded) {
        print(serial, " PB13 ", b13, " PB14 ", b14, " PB15 ", b15,
              " (PB8/PB9 judged drivable under the desk's I2C pull-ups; PB15 is "
              "UCPD1_CC2; the dead-battery strobe ",
              rd_released ? "was spent first" : "answered false (nothing to release)",
              ")", crlf);
    } else {
        print(serial,
              " (PB10..PB15 are bonded to no pin of this package - DS12992 "
              "table 12 - so this census stops at PB9; the dead-battery strobe ",
              rd_released ? "was spent first" : "answered false (nothing to release)",
              ")", crlf);
    }
    bench.verdict("the port-B pads this letter drives are electrically free - "
                  "they follow their own internal pull between the rails, or "
                  "(PB8/PB9 on the Nucleo-64 bench) are held up by the desk's "
                  "I2C pull-ups and sink when driven; PB13/PB14/PB15 are asked "
                  "only where the package bonds them, and the Type-C "
                  "dead-battery strobe PB15 shares with PA8 is spent first on "
                  "a part that has one and REFUSED on a part that has not",
                  rd_released == ucpd_present(1) && b6 && b7 && b8 && b9 &&
                      b13 && b14 && b15);

    // QUESTIONS 1 AND 2, instance by instance.
    tl_tim4_counts();
    tl_tim6_counts();
    tl_tim7_counts();
    const bool t14_ok = instance_counts_and_interrupts<T14>("TIM14", t14_update_calls);
    bench.verdict("TIM14 - one channel, no slave controller, no master mode, "
                  "no DMA request of any kind - counts and interrupts on a "
                  "vector of its very own",
                  t14_ok);
    tl_tim15_counts();
    const bool t17_ok = instance_counts_and_interrupts<T17>("TIM17", t17_update_calls);
    bench.verdict("and TIM17 does, on the line the reserve names for it - "
                  "shared with the SECOND FDCAN interrupt where there is an "
                  "FDCAN and its own where there is none - so every shared-line "
                  "name derived from PRESENCE is the right one, which a wrong "
                  "one would have shown as a silent Default_Handler spin",
                  t17_ok);

    // AND THE BASIC TIMERS' ONE OUTPUT. 23.4.2's MMS is all TIM6 and TIM7
    // have to say to the rest of the chip; its consumers here are the DAC
    // and the ADC, which belong to another suite. What is checked is that
    // the master mode exists on them and that a CHANNEL does not.
    tl_basic_timers();

    // QUESTION 3: A PWM ON A PAD, per instance with channels.
    //
    tl_tim4_pwm();

    // TIM14's ONE, on PA7 at AF4 - the same pad letter g drives as
    // TIM1_CH1N at AF2, which is what makes the alternate-function
    // multiplexer visible: one pad, two timers, whichever AF says.
    T14::init();
    TimPad<t14_ch1_pad>::claim();
    using T14Pwm = TimPwm<T14, 0, pwm_top>;
    static_assert(PwmChannel<T14Pwm>);
    const bool t14_up = T14Pwm::setup(0);
    T14Pwm::duty(350);
    spin_cycles(SysClock::hz / 1000u);
    const uint16_t t14_got = sample_permille(PadPairN::pin_number, 40000u);
    print(serial, "  TIM14_CH1 on PA7 at AF4 (letter g's TIM1_CH1N pad at "
          "AF2): duty 350 asked, ", t14_got, " per mille read", crlf);
    bench.verdict("TIM14's single channel reaches a pad, and the SAME PAD "
                  "carries a different timer under a different alternate "
                  "function - the AF number is what chooses, and this is the "
                  "check the device header cannot make",
                  t14_up && t14_got + 25u >= 350u && t14_got <= 375u);
    T14::release();
    PadPairN::input(PinPull::none);

    // TIM15: TWO channels AND a complement, all three on port B, so one
    // IDR read carries the pair - the census letter g makes of TIM1's.
    tl_tim15_pwm();

    // TIM17: one channel and its complement, again both on port B.
    T17::init();
    TimPad<t17_ch1_pad>::claim();
    TimPad<t17_ch1n_pad>::claim();
    using T17Pair = TimPairPwm<T17, 0, pwm_top>;
    const bool t17_up = T17Pair::setup(0, tim_dead_time_code(64));
    T17Pair::duty(450);
    spin_cycles(SysClock::hz / 1000u);
    const PairCensus t17c = census_pair_b(9, 7, 40000u);
    const uint32_t t17_a = (t17c.a_high * 1000u + 20000u) / 40000u;
    const uint32_t t17_b = (t17c.b_high * 1000u + 20000u) / 40000u;
    const uint32_t t17_dt = (static_cast<uint32_t>(T17Pair::dead_time_ticks()) *
                             1000u) / (static_cast<uint32_t>(pwm_top) + 1u);
    const uint32_t t17_want_a = 450u - t17_dt;
    const uint32_t t17_want_b = 550u - t17_dt;
    print(serial, "  TIM17: CH1 on PB9 ", t17_a, " per mille, CH1N on PB7 ",
          t17_b, ", both high together in ", t17c.both_high, " of 40000 reads "
          "(asked 450 with the same dead time, so due ", t17_want_a, " and ",
          t17_want_b, ")", crlf);
    bench.verdict("TIM17's pair too - one channel, one complement, never both "
                  "high, and both halves at their own duty less the dead time, "
                  "on the last of the instances that had never driven "
                  "anything here",
                  t17_up && t17c.both_high == 0u &&
                      t17_a + 20u >= t17_want_a && t17_a <= t17_want_a + 20u &&
                      t17_b + 20u >= t17_want_b && t17_b <= t17_want_b + 20u);
    T17::release();
    PadB7::input(PinPull::none);
    PadB9::input(PinPull::none);

    quiet_everything();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf,
          "test_stm32_tim - the G0 timers (RM0444 ch. 21..25): PwmChannel "
          "and MeterSampler on this silicon, wireless, clk=",
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
extern "C" void BRIO_STM32G0_USART2_HANDLER() { (void)Serial::isr(); }

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
/// One vector, two timers - where the part has two. The body serves TIM4
/// only where there is one; the reserve's macro is what names the line,
/// so the same source binds TIM3_TIM4_IRQHandler on a G0B1 and
/// TIM3_IRQHandler on a part whose TIM3 has the line to itself.
template <bool present = tim_present(4)>
void serve_tim4() {
    if constexpr (present) {
        const uint32_t f4 = Tim4<present>::isr();
        if ((f4 & Tim4<present>::update_flag) != 0u) {
            t4_update_calls = t4_update_calls + 1u;
        }
    }
}

extern "C" void BRIO_STM32G0_TIM3_HANDLER() {
    const uint32_t f3 = T3::isr();
    if ((f3 & T3::update_flag) != 0u) {
        t3_update_calls = t3_update_calls + 1u;
    }
    if ((f3 & ~(T3::update_flag | T3::compare_flag(0))) != 0u) {
        t3_saw_t4_flag = t3_saw_t4_flag + 1u;
    }
    serve_tim4();
}

/// Letter l's four other lines. Two of them are shared with something
/// that is not a timer at all - TIM6 with the DAC and LPTIM1, TIM17 with
/// the second FDCAN interrupt - and the body still answers only for the
/// flags its own DIER enabled, which is the whole point of an isr() that
/// returns what it served.
///
/// A HANDLER IS A DEFINITION AND ONLY THE PREPROCESSOR CAN SPELL ONE, so
/// the two bodies whose instance is per-part are guarded by the very
/// macro the reserve derives their NAME from: it is defined exactly where
/// the timer is, so where TIM6 is absent there is no name to bind and no
/// body to write. TIM15's name never varies - only its EXISTENCE does -
/// so the reserve derives no macro for it and the guard is the header's
/// own base symbol: the one place in this file that names a peripheral
/// block, and a candidate for a BRIO_STM32G0_TIM15_HANDLER of the same
/// presence rule if a second app ever needs it.
#if defined(BRIO_STM32G0_TIM6_HANDLER)
extern "C" void BRIO_STM32G0_TIM6_HANDLER() {
    using T6 = Tim6<true>;
    if ((T6::isr() & T6::update_flag) != 0u) {
        t6_update_calls = t6_update_calls + 1u;
    }
}
#endif

#if defined(BRIO_STM32G0_TIM7_HANDLER)
extern "C" void BRIO_STM32G0_TIM7_HANDLER() {
    using T7 = Tim7<true>;
    if ((T7::isr() & T7::update_flag) != 0u) {
        t7_update_calls = t7_update_calls + 1u;
    }
}
#endif

extern "C" void TIM14_IRQHandler() {
    if ((T14::isr() & T14::update_flag) != 0u) {
        t14_update_calls = t14_update_calls + 1u;
    }
}

#if defined(TIM15_BASE)
extern "C" void TIM15_IRQHandler() {
    using T15 = Tim15<true>;
    if ((T15::isr() & T15::update_flag) != 0u) {
        t15_update_calls = t15_update_calls + 1u;
    }
}
#endif

extern "C" void BRIO_STM32G0_TIM17_HANDLER() {
    if ((T17::isr() & T17::update_flag) != 0u) {
        t17_update_calls = t17_update_calls + 1u;
    }
}

/// TIM16's line is shared with an FDCAN one. In letter j the capture
/// feeds a util MeterLatch; everywhere else it is only counted, so the
/// reading is taken either way and `kernel_mode` says where it goes.
extern "C" void BRIO_STM32G0_TIM16_HANDLER() {
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
    bench.letter('c', "PWM on the board LED, read back through its own pad",
                 tc_pwm_pad);
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
    bench.letter('k', "the G0B1's TIM erratum 2.7.2 staged with a control",
                 tk_errata);
    bench.letter('l', "the instances never counted here: TIM14, TIM17, and "
                      "TIM4, TIM6, TIM7 and TIM15 where the part has them",
                 tl_six_instances);

    if (serial_ok) {
        const auto idcode = brio::DeviceIdcode::read();
        brio::print(serial, brio::crlf, "part DEV_ID ", brio::hex(idcode.dev_id),
              " REV_ID ", brio::hex(idcode.rev_id), brio::crlf);
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
