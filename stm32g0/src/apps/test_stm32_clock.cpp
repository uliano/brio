// test_stm32_clock - THE DYNAMIC CLOCK: SYSCLK moved under the running
// program between three tuples of (root, VCORE range, regulator) - 64 MHz
// on the PLL in Range 1, 16 MHz on HSISYS in Range 2, 2 MHz on HSISYS/8
// in low-power run - with every driver that follows the rate rebased
// before each switch and the kernel timebase off the core clock. No
// wires on any board.
//
// This whole image runs on Stm32g0Platform<LptimTicker<>> (the tickless
// timebase: kernel time on a 32 kHz root, unmoved by SYSCLK) with the
// CONSOLE ON HSI16 (its kernel clock, so its rebase folds to nothing and
// the report never depends on the switch under test) and the rebased
// user under test on PCLK: USART1 as a single wire on PA9 (HDSEL, the
// serial suite's loop-back), which reads back what it sent only if its
// divisor followed.
//
// TWO THINGS ARE PER-BOARD HERE AND BOTH ARE STATED WHERE THEY ARE
// CHOSEN. THE CONSOLE'S INSTANCE: a console whose divisor must not
// follow the switch needs a kernel-clock multiplexer, and table 183
// gives USART2 one only on the parts where it is FULL - so where it is
// BASIC the console is LPUART1 instead, on the same two pads at AF6.
// THE WALL: the finest clock that does not move with SYSCLK is HSI16/64
// through MCO into TIM2's ETR (4 us a count) where 22.4.25's ETRSEL list
// has that code, and NOTHING AT ALL where the part has no such code and
// the board no crystal - there the RTC's own sub-second counter is the
// wall (about 30 us a count, and the rate is measured at boot because an
// LSI is not a nominal), TIM2 counts PCLK as letter g's awake meter, and
// the letters that need four microseconds say so and skip.
// What is judged:
//
//   a  THE BOOT RATE AND THE PACK: 64 MHz on the PLL in Range 1 with two
//      wait states, the readbacks against the type's claims, the MCO
//      wall weighed against the crystal, delay_us against the crystal,
//      and one of the four steps the design named as unbenched - the
//      PLL's refusal to be reconfigured while it runs
//   b  THE FALL TO 16 MHz IN RANGE 2: the switch's duration, SWS on
//      HSISYS with the PLL off, VOS 2, ONE wait state (the Range 2
//      column, where Range 1 wants none at 16 MHz - a wait-state
//      DECREASE from two, the second unbenched step, and the range's own
//      column, the fourth), the loop-back exact at the new rate, the
//      CPU really at 16 MHz (delay_us against the crystal)
//   c  THE FALL TO 2 MHz IN LOW-POWER RUN: HSIDIV written under a running
//      HSISYS core (the third unbenched step), LPR set and REGLPF
//      standing, no wait state, the loop-back exact, the CPU at 2 MHz
//   d  THE RISE BACK TO 64: LPR left (REGLPF clear), Range 1 before the
//      PLL, two wait states before the frequency, everything as in a;
//      then THE LADDER walked twenty-four times round with the loop-back
//      exchanging bytes at every rung - zero errors, the worst durations
//   e  THE ADC ACROSS THE LADDER: a converter on PCLK/1 configured at
//      16 MHz reads VREFINT at 16, 64 (its division moved to /2 by the
//      rebase), 2 MHz in low-power run (/1 again) and 16 again; the four
//      VDDA readings agree, and the asynchronous mode (HSI16, no rebase)
//      agrees with them at 2 MHz
//   f  A KERNEL THROUGH THE SWITCHES: a 50-tick periodic and an AO that
//      walks the ladder on a 300-tick periodic, three seconds - every
//      firing on its cadence, never early (the timebase is on the
//      crystal), the switches counted
//   g  STOP 1 AT 64 MHz AND AT 2 MHz LOW-POWER RUN through the power
//      manager and the plain site: after the wake the site's
//      resume_clock() is the clock's restore() - the rate IN FORCE comes
//      back, its index unchanged, the deadline met on the RTC wall; at
//      2 MHz the part wakes in low-power run with HSIDIV kept (4.3.6) and
//      restore() finds nothing to do
//   h  delay_us AT EVERY RUNG on the MCO wall: the per-rate table
//      selected by the rate index, at least and never early at 64, 16
//      and 2 MHz
//   i  THE SYSTICK TICKER AS A CLOCK USER: SysTick handed from the
//      interrupt-less counter to BasicTicker for one letter, the tick
//      rate held at 1000 Hz against the crystal across the ladder
//      (rebase() restarts the period: under a tick late per switch,
//      never fast), then handed back
//   j  A PLL RATE IN RANGE 2: 16 MHz as PLLRCLK with the VCO at table
//      47's 128 MHz ceiling, reached from 64 MHz on the PLL (a
//      reconfiguration through HSISYS), from 2 MHz in low-power run and
//      back, the loop exact, the CPU at 16 MHz
//   x  OUTSIDE z: the durations of every transition, broken down.
//
// Letter g also stages what the chapter leaves open: a STOP 0 from
// low-power run, judged by the MCO wall (TIM2 counts only while HSI16
// runs, i.e. only awake - a Sleep that was not a Stop counts 75000 in
// 300 ms, a Stop counts a few hundred).
//
// THE WALL THAT DOES NOT MOVE WITH SYSCLK: HSI16 through the MCO
// multiplexer divided by 64 (250 kHz), fed to TIM2's ETR through
// TIM2_AF1.ETRSEL = MCO with no pad - external clock mode 2 - so TIM2
// counts a 4 us wall whatever the core runs at (250 kHz is below a
// quarter of the slowest TIMPCLK, 2 MHz, as 22.4.3 wants). Weighed
// against the LSE timebase in letter a. The RTC's sub-second counter is
// the wall a Stop cannot stop, the IWDG the backstop of every sleep.
//
// build: boards = g0b1re,g071rb,g031k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/adc.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/lptim.hpp"
#include "stm32g0/lpuart.hpp"
#include "stm32g0/lptim_ticker.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/sleep.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

// ---------------------------------------------------------------------------
// The rates, the users, the clock
// ---------------------------------------------------------------------------

using Fast = Clock<ClockSource::pll, 64'000'000>;
using Mid = Clock<ClockSource::internal, 16'000'000, PowerRegime::range2>;
using Slow = Clock<ClockSource::internal, 2'000'000, PowerRegime::low_power_run>;
// The fourth rate, letter j's: the PLL in Range 2 - M 1 / N 8 / R 8,
// VCO 128 MHz exactly at DS13560 table 47's Range 2 ceiling.
using Pll16 = Clock<ClockSource::pll, 16'000'000, PowerRegime::range2>;
static_assert(Pll16::pll.m == 1 && Pll16::pll.n == 8 && Pll16::pll.r == 8);

// The console: USART2 on HSI16, so that its divisor never moves and the
// report is never a function of the thing under test. It is still
// LISTED (clock_follows demands it) and its rebase() folds to nothing.
constexpr UartOptions console_opts{.kernel_clock = UsartClock::hsi16};

// AND ON A PART WHOSE USART2 HAS NO KERNEL-CLOCK MULTIPLEXER, THAT COSTS
// A DIFFERENT PERIPHERAL. Table 183's FULL/BASIC split moves with the
// part: where USART2 is BASIC it runs on PCLK, full stop - its divisor
// would follow every switch under test and the report would be a
// function of its own subject. The instance that always has a
// multiplexer is the LPUART (34.4.6: every LPUART of the family has
// one), and LPUART1_TX/RX reach THE SAME TWO PADS at AF6 - the shape
// test_stm32_serial's letter v proves on the G0B1. So the console moves
// to LPUART1 exactly where USART2's multiplexer is missing.
//
// The HANDLER has to be chosen by the preprocessor, which cannot call
// a constexpr function - so the same header symbol the reserve probes
// for usart_has_clock_select(2) is probed here, and the static_assert
// below is what keeps the two answers one answer.
#if defined(RCC_CCIPR_USART2SEL_Pos)
#define BRIO_SUITE_CONSOLE_HANDLER BRIO_STM32G0_USART2_HANDLER
constexpr bool console_on_lpuart = false;
constexpr PinFunction console_af = PinFunction::af1;
#else
#define BRIO_SUITE_CONSOLE_HANDLER BRIO_STM32G0_LPUART1_HANDLER
constexpr bool console_on_lpuart = true;
constexpr PinFunction console_af = PinFunction::af6;
#endif
static_assert(console_on_lpuart == !usart_has_clock_select(2),
              "the console's instance and the reserve's own column must be "
              "one answer");

constexpr UartPins console_pins{
    .tx = {'A', 2, console_af},
    .rx = {'A', 3, console_af},
};
using Serial = std::conditional_t<
    console_on_lpuart,
    LpUart<1, console_pins, 128, 1024, NoDmaEngine, NoDmaEngine, console_opts>,
    Uart<2, console_pins, 128, 1024, NoDmaEngine, NoDmaEngine, console_opts>>;
static_assert(Serial::options.kernel_clock == UsartClock::hsi16);

// The rebased user under test: USART1 on PCLK as a single wire on PA9.
constexpr UartOptions loop_opts = uart_half_duplex();
constexpr UartPins loop_pins{
    .tx = {'A', 9, PinFunction::af1},
    .rx = {'A', 10, PinFunction::af1},
};
using Loop = Uart<1, loop_pins, 64, 64, NoDmaEngine, NoDmaEngine, loop_opts>;
static_assert(Loop::options.kernel_clock == UsartClock::pclk && Loop::options.half_duplex);

// BOTH SysTick writers are listed, which no program would do: letter i
// hands SysTick from the interrupt-less counter to the BasicTicker and
// back, and each must follow the rate while it holds the register. The
// two rebase() write the same reload; the counter's, last in the list,
// is what stands after every switch outside that letter.
using SysClock = DynamicClock<Rates<Fast, Mid, Slow, Pll16>, Ticker, SysTickCounter, Serial, Loop, Adc>;
constexpr SysClock clock;
static_assert(SysClock::rate_count == 4);
static_assert(delay_rates<SysClock>[0].cycles_per_us == 64);
static_assert(delay_rates<SysClock>[2].cycles_per_us == 2);
static_assert(delay_rates<SysClock>[3].cycles_per_us == 16);

constexpr uint8_t r_fast = 0;
constexpr uint8_t r_mid = 1;
constexpr uint8_t r_slow = 2;
constexpr uint8_t r_pll16 = 3;

// THE TIMEBASE, and the platform on it. The kernel tick must not move
// with SYSCLK, which is what an LPTIM on a 32 kHz root gives - the
// crystal where the board has one, and LSI where it has not. An LSI
// ticker STATES its rate and the number to state is one NOT BELOW the
// true one (the ticker's own directional rule: a tick declared faster
// than it is makes every deadline late and none early), so the default
// is DS12992 table 46's own ceiling and this suite takes it. Which board
// this is, is the one question only the preprocessor can ask.
#if defined(STM32G031xx)
constexpr LptimTickerConfig tb_cfg{.source = LptimTickerSource::lsi};
#else
constexpr LptimTickerConfig tb_cfg{};
#endif
using Tb = LptimTicker<tb_cfg>;
using P = Stm32g0Platform<Tb>;
static_assert(Tickless<Tb>);

Serial serial;
TestBench<Serial> bench;

using T2 = Tim<2>;
using Site = Stm32g0SleepSite<SysClock, Tb>;
static_assert(!Site::pauses_tick);

// ---------------------------------------------------------------------------
// The walls
// ---------------------------------------------------------------------------

/// TIM2 counting a clock that does NOT move with SYSCLK, through its own
/// ETR and with no pad anywhere - the wall every rung of the ladder is
/// weighed against.
///
/// WHICH clock is a per-part table cell and not a choice: RM0444
/// 22.4.25's ETRSEL list gives 0100 (MCO), 0101 (MCO2) and 0110 (COMP3)
/// to the G0B1/G0C1 sales types alone, while 0011 (LSE) is every part's,
/// and the reserve is what says which (`tim_etrsel_has_mco`). So the wall
/// is HSI16/64 through MCO at 250 kHz where the part offers that code -
/// 4 us a count - and the LSE crystal at 32.768 kHz where it does not,
/// 30.5 us a count. The Stop letters need only a clock that stops with
/// the part, and either serves; letter h needs 4 us of resolution and
/// says so where it cannot have it.
///
/// AND ON A BOARD WITH NEITHER, THERE IS NO ETR WALL AT ALL. Where the
/// part offers no MCO code the only every-part code is the LSE, and a
/// board whose crystal does not start leaves TIM2's ETR with nothing on
/// it: the counter stands still, in silence, and a loop that waits on it
/// waits for ever - which is how the second silicon of this desk first
/// met 22.4.25's footnote. So the wall is PROBED at boot, counted over a
/// real interval of the kernel timebase, and `etr_wall_ok` is what every
/// user of it asks first; the fallback for real time is the RTC's own
/// sub-second counter below, coarser by a factor of eight and running on
/// whatever the domain has.
constexpr uint8_t tim2_etrsel_mco = 4;    ///< TIM2_AF1.ETRSEL 0100 = MCO (RM0444 22.4.25)
constexpr uint8_t tim2_etrsel_lse = 3;    ///< 0011 = LSE, on every part
constexpr bool wall_on_mco = tim_etrsel_has_mco(2);
constexpr uint32_t etr_wall_hz = wall_on_mco ? 16'000'000u / 64u : 32768u;
constexpr const char* etr_wall_name =
    wall_on_mco ? "MCO = HSI16/64" : "the LSE crystal";
bool etr_wall_ok = false;
uint32_t t2() { return T2::count(); }
uint32_t t2_us(uint32_t counts) {
    return static_cast<uint32_t>((static_cast<uint64_t>(counts) * 1'000'000u) /
                                 etr_wall_hz);
}
bool wall_mco_up() {
    if (wall_on_mco && !Rcc::mco(Rcc::mco_hsi16_code, 6)) {
        return false;
    }
    T2::bus_clock(true);
    T2::reset();
    if (!T2::configure({.prescaler = 0, .period = 0xFFFFFFFFu})) {
        return false;
    }
    if (!T2::external_trigger_select(wall_on_mco ? tim2_etrsel_mco
                                                 : tim2_etrsel_lse)) {
        return false;
    }
    if (!T2::external_trigger({.clock_mode2 = true})) {
        return false;
    }
    T2::enable(true);
    return true;
}

/// The RTC's sub-second counter at PREDIV_A 0 / PREDIV_S 32767: a 30.5 us
/// stopwatch on the domain's own root that keeps counting through a Stop (the
/// lptim suite's instrument, verbatim).
constexpr RtcPrescalers wall_prescalers{.async = 0, .sync = 32767};
bool wall_ready = false;
/// What RTCCLK really is: the crystal's 32768 where the domain runs on
/// the LSE, and the LSI's rate where it does not - stated from DS12992
/// table 46's ceiling until the boot measurement replaces it, so a
/// conversion made before the measurement can only over-state elapsed
/// time and never under-state it.
uint32_t rtcclk_hz = 32768;
bool wall_on_lse = false;

uint32_t wall_ticks_per_second() {
    return static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
}
uint32_t wall_modulus() { return 60u * wall_ticks_per_second(); }
uint32_t wall_hz() {
    return rtcclk_hz / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
}
uint32_t wall() {
    RtcReading r{};
    if (!Rtc::read(r)) {
        return 0xFFFFFFFFu;
    }
    const uint32_t per_second = wall_ticks_per_second();
    return static_cast<uint32_t>(r.time.second) * per_second +
           (per_second - 1u - r.subsecond);
}
uint32_t wall_delta(uint32_t from, uint32_t to) {
    return (to >= from) ? (to - from) : (wall_modulus() - from + to);
}
uint32_t wall_ms(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000ULL) / wall_hz());
}
bool wall_up() {
    if (Rtc::prescalers().sync == wall_prescalers.sync &&
        Rtc::prescalers().async == wall_prescalers.async) {
        return true;
    }
    return Rtc::init(wall_prescalers,
                     RtcDateTime{.hour = 0, .minute = 0, .second = 0,
                                 .day = 1, .month = 1, .year = 24, .weekday = 1});
}
uint32_t wall_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) /
                                 wall_hz());
}
/// Microseconds between two wall readings. THE RTC IS THE SCALE EVERY
/// RATE CLAIM IS JUDGED ON, and deliberately not the ETR wall: that one
/// counts HSI16/64 where it exists, and HSI16 is trimmed to a per cent,
/// which is wider than the band a "the CPU is really at this rate"
/// verdict lives in. The RTC's root is a crystal by construction or an
/// LSI at the rate this boot measured - either way a scale, not a
/// nominal.
uint32_t rtc_us(uint32_t from, uint32_t to) {
    return wall_us(wall_delta(from, to));
}

// ---- ONE REAL-TIME WALL, whichever this board has --------------------------
//
// Every span this suite times in microseconds - a switch, a loop-back's
// turnaround, a Stop - is read here, and what answers is the ETR wall
// where it runs and the RTC's sub-second counter where it does not. The
// two differ in RESOLUTION (4 us against about 30) and in nothing else
// that matters: both keep counting whatever SYSCLK does. A letter whose
// claim needs the finer of the two says so and skips.
uint32_t mono() { return etr_wall_ok ? t2() : wall(); }
uint32_t mono_delta(uint32_t from, uint32_t to) {
    return etr_wall_ok ? (to - from) : wall_delta(from, to);
}
uint32_t mono_us(uint32_t counts) {
    return etr_wall_ok ? t2_us(counts) : wall_us(counts);
}
uint32_t mono_resolution_us() {
    return etr_wall_ok ? (1'000'000u / etr_wall_hz) : (1'000'000u / wall_hz());
}

// ---- THE LSI's OWN RATE, where the wall runs on it -------------------------
//
// A crystal is 32768 Hz by construction; an LSI is an RC oscillator the
// datasheet only bounds (table 46: 29.5..34 kHz), so a wall built on one
// has to be WEIGHED before anything is timed against it. TIM16's capture
// channel with TISEL on LSI (25.6.18's code 1) against a counter clocked
// from PCLK is that measurement, with the MEDIAN of a batch as the
// estimator - test_stm32_rtc's letter c owns the technique and the
// reason: an unfiltered capture of an internal clock line errs in BOTH
// directions, so neither the minimum nor the mean is an estimator of the
// period.
using LsiMeter = TimIntervalMeter<Tim<16>, 0>;
constexpr uint16_t lsi_meter_prescaler = 15;   ///< 250 ns a tick at 64 MHz

uint32_t measure_lsi_hz() {
    Rcc::lsi_enable(true);
    if (!Rcc::lsi_wait_ready()) {
        return 0;
    }
    Tim<16>::bus_clock(true);
    Tim<16>::enable(false);
    if (!LsiMeter::setup(lsi_meter_prescaler, 8) ||
        !Tim<16>::input_select(0, Rcc::lsi_tim16_ti1_code)) {
        return 0;
    }
    (void)Tim<16>::isr();
    LsiMeter::restart();
    static uint32_t samples[33];
    constexpr uint16_t want = 33;
    uint16_t got = 0;
    for (uint32_t spin = 0; spin < 40'000'000UL && got < want; ++spin) {
        if ((Tim<16>::flags() & LsiMeter::capture_flag) == 0u) {
            continue;
        }
        Tim<16>::clear_flags(LsiMeter::capture_flag);
        const std::optional<uint32_t> d = LsiMeter::interval();
        if (d.has_value()) {
            samples[got++] = *d;
        }
    }
    Tim<16>::release();
    if (got != want) {
        return 0;
    }
    for (uint16_t i = 1; i < want; ++i) {
        const uint32_t key = samples[i];
        uint16_t j = i;
        while (j != 0u && samples[j - 1u] > key) {
            samples[j] = samples[j - 1u];
            --j;
        }
        samples[j] = key;
    }
    const uint32_t median = samples[want / 2u];
    return median != 0u ? (Fast::hz / (lsi_meter_prescaler + 1u)) / median : 0u;
}

/// TIM2 AS AN AWAKE METER, which is a different question from the wall's.
/// A Stop stops every clock in the VCORE domain, TIM2's own included, so
/// a counter that ran across the round says how much of it the core spent
/// awake - and that is what tells a Stop from a WFI that fell through.
/// Where the ETR wall runs, TIM2 counts it and the rate is fixed; where
/// it does not, TIM2 counts PCLK, which MOVES with the rate, so the
/// conversion asks the clock what is in force. Both stop with the part.
uint32_t awake_us(uint32_t counts) {
    if (etr_wall_ok) {
        return t2_us(counts);
    }
    const uint32_t per_us = SysClock::hz() / 1'000'000u;
    return per_us != 0u ? counts / per_us : 0u;
}

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

void feed() { Iwdg::refresh(); }

/// Wait on the timebase (the crystal), whatever the core runs at.
void tb_wait_ms(uint32_t ms) {
    const uint32_t t0 = Tb::millis();
    while (Tb::millis() - t0 < ms) {
        feed();
    }
}

/// A measurement window a transmit interrupt walks through is not a
/// measurement; and at 2 MHz the console drains slowly.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    tb_wait_ms(3);
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// The silicon's own account of the clock: what every rung is judged by.
struct ClockState {
    SysclkSource sws;
    uint8_t hsidiv;
    bool pll_on;
    uint8_t vos;
    bool lpr;
    bool reglpf;
    uint8_t latency;
    uint8_t index;
    uint32_t hz;
};
ClockState state() {
    return ClockState{
        .sws = Rcc::sysclk_status(),
        .hsidiv = Rcc::hsi_div(),
        .pll_on = (RCC->CR & RCC_CR_PLLON) != 0u,
        .vos = Pwr::range(),
        .lpr = Pwr::low_power_run(),
        .reglpf = Pwr::on_low_power_regulator(),
        .latency = FlashWaitStates::get(),
        .index = SysClock::rate_index(),
        .hz = SysClock::hz(),
    };
}
void print_state(const char* label, const ClockState& s) {
    print(serial, "  ", label, ": SWS=", s.sws == SysclkSource::pllrclk ? "PLL" : "HSISYS",
          " HSIDIV=", s.hsidiv, " PLLON=", s.pll_on, " VOS=", s.vos, " LPR=", s.lpr,
          " REGLPF=", s.reglpf, " LATENCY=", s.latency, " rate_index=", s.index,
          " hz=", s.hz, crlf);
}
bool is_fast(const ClockState& s) {
    return s.sws == SysclkSource::pllrclk && s.hsidiv == 0u && s.pll_on && s.vos == 1u &&
           !s.lpr && !s.reglpf && s.latency == 2u && s.index == r_fast && s.hz == 64'000'000u;
}
bool is_mid(const ClockState& s) {
    return s.sws == SysclkSource::hsisys && s.hsidiv == 0u && !s.pll_on && s.vos == 2u &&
           !s.lpr && !s.reglpf && s.latency == 1u && s.index == r_mid && s.hz == 16'000'000u;
}
bool is_slow(const ClockState& s) {
    return s.sws == SysclkSource::hsisys && s.hsidiv == 3u && !s.pll_on && s.vos == 2u &&
           s.lpr && s.reglpf && s.latency == 0u && s.index == r_slow && s.hz == 2'000'000u;
}
bool is_pll16(const ClockState& s) {
    return s.sws == SysclkSource::pllrclk && s.hsidiv == 0u && s.pll_on && s.vos == 2u &&
           !s.lpr && !s.reglpf && s.latency == 1u && s.index == r_pll16 && s.hz == 16'000'000u;
}
bool (*checker_of(uint8_t index))(const ClockState&) {
    switch (index) {
        case r_fast: return is_fast;
        case r_mid: return is_mid;
        case r_slow: return is_slow;
        default: return is_pll16;
    }
}

/// A switch, timed on the MCO wall (the wall's own 4 us quantum on
/// top), with the console drained first.
struct Switch {
    bool ok = false;
    uint32_t us = 0;
};
Switch go(uint8_t index) {
    console_drain();
    Switch s{};
    const uint32_t t0 = mono();
    s.ok = SysClock::set_index(index);
    s.us = mono_us(mono_delta(t0, mono()));
    return s;
}

/// Move `count` bytes round the single wire and say how many came back
/// exact - the Uart TASK, whose divisor the switch rebased.
uint32_t loop_run(uint32_t count) {
    uint8_t v = 0x31;
    uint32_t good = 0;
    for (uint32_t i = 0; i < count; ++i) {
        uint8_t sink = 0;
        (void)Loop::read_byte(sink);      // anything stale
        if (!Loop::write_byte(v)) {
            continue;
        }
        // One character at 115200 is 87 us; wait on the wall, since
        // a spin is 32 times longer at 2 MHz than at 64.
        const uint32_t t0 = mono();
        bool got = false;
        uint8_t b = 0;
        while (mono_us(mono_delta(t0, mono())) < 2000u) {
            if (Loop::read_byte(b)) {
                got = true;
                break;
            }
        }
        if (got && b == v) {
            ++good;
        }
        v = static_cast<uint8_t>(v * 5u + 1u);
    }
    return good;
}

/// N delay_us(999) rounds against the crystal: the CPU is at the rate the
/// type claims iff this takes ~N ms (SysTick counts real cycles, the
/// reload and the table are the type's) - plus the calls' own cycles,
/// which are a fixed count and so cost 32 times more at 2 MHz than at
/// 64: `delay_rounds_ok()` prices them at 80 cycles a call.
uint32_t delay_rounds_ms(uint32_t rounds) {
    console_drain();
    // ON THE WALL, NOT ON THE KERNEL TICK. The tick is an LptimTicker,
    // which converts with a STATED rate: exact on a crystal, and
    // deliberately a few per cent slow on an LSI (the never-early rule
    // makes it state a rate at or above the true one). The RTC wall runs
    // on the same root but at the rate this boot MEASURED, so it is the
    // instrument that means microseconds on either board - and what this
    // function judges is the CPU's rate, which needs a true scale.
    const uint32_t t0 = wall();
    for (uint32_t i = 0; i < rounds; ++i) {
        (void)delay_us(clock, 999u);
    }
    return rtc_us(t0, wall()) / 1000u;
}
bool delay_rounds_ok(uint32_t rounds, uint32_t ms) {
    const uint32_t overhead_ms = (rounds * 80u) / (SysClock::hz() / 1000u);
    return within(ms, rounds - 2u, rounds + overhead_ms + 2u);
}

// ---------------------------------------------------------------------------
// Shared ISR state
// ---------------------------------------------------------------------------

volatile uint32_t lptim_irqs = 0;
volatile uint32_t isr_restores = 0;    ///< LPTIM interrupts that found SYSCLK off the rate
volatile uint32_t systick_irqs = 0;    ///< SysTick_Handler runs (letter i's only)
volatile uint32_t rtc_wakes = 0;
volatile bool kernel_live = false;

// ---------------------------------------------------------------------------
// The kernel half (letters f and g)
// ---------------------------------------------------------------------------

struct Beat {};
struct Step {};
struct Blip {};
struct Woke {};

/// A 50-tick periodic, every firing judged in the timebase's own ticks.
struct Metronome : Fsm<Metronome, Beat> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Metronome, Beat> beat{Beat{}};
    static inline uint32_t fired = 0;
    static inline uint32_t last_tick = 0;
    static inline uint32_t min_ticks = 0xFFFFFFFFu;
    static inline uint32_t max_ticks = 0;
    static inline uint32_t early = 0;
    static inline uint8_t seen_rates = 0;   ///< bit per rate index a firing ran at
    static void clear() {
        fired = 0;
        last_tick = 0;
        min_ticks = 0xFFFFFFFFu;
        max_ticks = 0;
        early = 0;
        seen_rates = 0;
    }
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](Beat) {
                const uint32_t now = Tb::ticks();
                if (fired != 0u) {
                    const uint32_t dt = now - last_tick;
                    if (dt < 50u) ++early;
                    if (dt < min_ticks) min_ticks = dt;
                    if (dt > max_ticks) max_ticks = dt;
                }
                last_tick = now;
                ++fired;
                seen_rates = static_cast<uint8_t>(seen_rates | (1u << SysClock::rate_index()));
                return handled();
            });
    }
};

/// Walks the ladder on its own periodic: fast -> mid -> slow -> fast ...
struct Walker : Fsm<Walker, Step> {
    static inline EventQueue<Event, 4, P> queue;
    static inline TimeEvent<P, Walker, Step> step{Step{}};
    static inline uint32_t switches = 0;
    static inline uint32_t failures = 0;
    static void clear() { switches = failures = 0; }
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](Step) {
                const uint8_t next = static_cast<uint8_t>((SysClock::rate_index() + 1u) % SysClock::rate_count);
                if (!SysClock::set_index(next)) ++failures;
                ++switches;
                return handled();
            });
    }
};
using WalkKernel = Kernel<P, Metronome, Walker>;

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip, Woke> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};
    static inline uint16_t blips = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline uint32_t blip_wall = 0;
    static inline uint32_t blip_ticks = 0;
    static inline ClockState blip_state{};
    static void clear() {
        blips = wakes = votes = 0;
        blip_wall = 0;
        blip_ticks = 0;
    }
    static void init() { start(&only); }
    static Status only(const Event& e);
};

using Manager = PowerManager<P, Site, PowerConfig{}, Probe>;
using K = Kernel<P, Probe, Manager>;

Probe::Status Probe::only(const Event& e) {
    return match(e,
        [](Entry) { return handled(); },
        [](Exit) { return handled(); },
        [](SleepVote) { ++votes; return handled(); },
        [](const PrepareSleep& p) { p.reply.send(SleepVote{true}); return handled(); },
        [](WakeReport) { ++wakes; return handled(); },
        [](Woke) {
            post<Manager>(SleepRequested{SleepDepth::none, reply_to<Probe, SleepVote>()});
            return handled();
        },
        [](Blip) {
            ++blips;
            blip_wall = wall();
            blip_ticks = Tb::ticks();
            blip_state = state();
            post<Manager>(SleepRequested{SleepDepth::none, reply_to<Probe, SleepVote>()});
            return handled();
        });
}

void pump_until_blip(uint32_t guard_ms) {
    const uint32_t t0 = wall();
    while (Probe::blips == 0u && wall_ms(wall_delta(t0, wall())) < guard_ms) {
        feed();
        TimeEvents<P>::process();
        if (!K::step()) {
            K::idle_if_empty();
        }
    }
    while (K::step()) {
    }
}

// =============================================================================
// a - the boot rate and the pack
// =============================================================================
void ta_boot() {
    feed();
    const ClockState s = state();
    print_state("boot", s);
    bench.verdict("THE BOOT RATE IS THE PACK'S FIRST: 64 MHz on the PLL, Range 1, two wait "
                  "states, HSIDIV 0, the low-power regulator off - the silicon agrees with "
                  "the type on every field",
                  is_fast(s));
    bench.verdict("the pack as the type states it: 64 M / 16 M / 2 M, Range 1 / Range 2 / "
                  "low-power run, PLL / HSISYS / HSISYS",
                  SysClock::rate_hz(0) == 64'000'000u && SysClock::rate_hz(1) == 16'000'000u &&
                      SysClock::rate_hz(2) == 2'000'000u &&
                      SysClock::rate_regime(0) == PowerRegime::range1 &&
                      SysClock::rate_regime(1) == PowerRegime::range2 &&
                      SysClock::rate_regime(2) == PowerRegime::low_power_run &&
                      SysClock::rate_source(0) == SysclkSource::pllrclk &&
                      SysClock::rate_source(2) == SysclkSource::hsisys);

    // THE WALL, weighed: two seconds of the timebase on TIM2's count.
    console_drain();
    const uint32_t c0 = t2();
    tb_wait_ms(2000);
    const uint32_t counted = t2() - c0;
    constexpr uint32_t etr_wall_nominal = etr_wall_hz * 2u;
    if (etr_wall_ok) {
        print(serial, "  2 s of the kernel timebase: TIM2 on ", etr_wall_name,
              " counted ", counted, " (", etr_wall_nominal, " nominal, ",
              (counted / (etr_wall_nominal / 1000u)), " per mille)", crlf);
        bench.verdict("THE WALL RUNS: a clock that does not move with SYSCLK "
                      "reaches TIM2's ETR with no pad (external clock mode 2, "
                      "ETRSEL naming MCO where the part has that code and the "
                      "LSE where it has not), within 1 % of its stated rate",
                      within(counted, (etr_wall_nominal / 100u) * 99u,
                             (etr_wall_nominal / 100u) * 101u));
    } else {
        // NOTHING REACHES THE ETR HERE, and both halves of that are
        // printed because either alone would be a guess: the part offers
        // no MCO code (a reserve fact) and the board's crystal does not
        // start (a board fact, and the LSE is the only other code every
        // part has). What TIM2 counts instead is PCLK - an awake meter,
        // used by letter g and by nothing that needs real time.
        print(serial, "  TIM2 on ETR counted ", counted,
              " in 2 s of kernel timebase: NOTHING REACHES IT. "
              "tim_etrsel_has_mco(2) is ", wall_on_mco,
              " on this part and the LSE (code 0011, the only other every-part "
              "source) is not running on this board, so external clock mode 2 "
              "has no clock. TIM2 counts PCLK instead - an AWAKE meter, which "
              "is what letter g needs of it - and every microsecond this suite "
              "prints comes from the RTC's sub-second counter at ",
              mono_resolution_us(), " us a tick.", crlf);
        print(serial, "  SKIPPED, no verdict claimed: the ETR wall's own rate.",
              crlf);
    }

    // The CPU really at 64 MHz: delay_us on SysTick's cycles against the crystal.
    const uint32_t ms = delay_rounds_ms(500);
    print(serial, "  500 x delay_us(999) at 64 MHz: ", ms, " ms on the RTC wall", crlf);
    bench.verdict("500 near-millisecond waits take 500 ms - SysTick's reload, delay_us's "
                  "table and the CPU's rate agree, on the RTC wall's scale",
                  delay_rounds_ok(500, ms));

    // The PLL's refusal while running: one of the four steps the design
    // named as unbenched.
    const uint32_t before = RCC->PLLCFGR;
    const bool refused = !Rcc::pll_configure({.m = 1, .n = 12, .r = 3});
    const bool untouched = RCC->PLLCFGR == before;
    bench.verdict("THE PLL IS NOT RECONFIGURED WHILE IT RUNS: pll_configure() refuses with "
                  "PLLON set and PLLCFGR is untouched (5.4.4) - which is why every switch "
                  "onto the PLL parks SYSCLK on HSISYS first",
                  refused && untouched);
}

// =============================================================================
// b - the fall to 16 MHz in Range 2
// =============================================================================
void tb_fall_mid() {
    feed();
    if (SysClock::rate_index() != r_fast) {
        (void)go(r_fast);
    }
    const Switch sw = go(r_mid);
    const ClockState s = state();
    print(serial, "  set_index(mid) -> ", sw.ok, " in ", sw.us, " us", crlf);
    print_state("mid", s);
    bench.verdict("THE FALL LANDS: SYSCLK on HSISYS undivided with the PLL OFF, VOS 2, "
                  "the low-power regulator off, rate_index 1, hz() 16 MHz",
                  sw.ok && is_mid(s));
    bench.verdict("A WAIT-STATE DECREASE, AND THE RANGE 2 COLUMN: LATENCY read back 1 - "
                  "down from 2 after the fall (3.3.4's order), and one where Range 1 "
                  "would want none at 16 MHz (table 13's second column)",
                  s.latency == 1u && s.vos == 2u);
    const uint32_t good = loop_run(64);
    print(serial, "  the single-wire loop at 16 MHz: ", good, " of 64 bytes exact (BRR ",
          hex(Usart<1>::brr()), ")", crlf);
    bench.verdict("THE REBASED USART READS BACK WHAT IT SENDS at the new rate: its divisor "
                  "followed (rebase() drained, then reloaded BRR)",
                  good == 64u);
    const uint32_t ms = delay_rounds_ms(500);
    print(serial, "  500 x delay_us(999) at 16 MHz: ", ms, " ms on the RTC wall", crlf);
    bench.verdict("the CPU is at 16 MHz: the SysTick counter rebased, delay_us's table "
                  "indexed by the rate, 500 waits in 500 ms",
                  delay_rounds_ok(500, ms));
    bench.verdict("the switch took under a millisecond", sw.us < 1000u);
}

// =============================================================================
// c - the fall to 2 MHz in low-power run
// =============================================================================
void tc_fall_slow() {
    feed();
    if (SysClock::rate_index() != r_mid) {
        (void)go(r_mid);
    }
    const Switch sw = go(r_slow);
    const ClockState s = state();
    print(serial, "  set_index(slow) -> ", sw.ok, " in ", sw.us, " us", crlf);
    print_state("slow", s);
    bench.verdict("HSIDIV WRITTEN UNDER A RUNNING HSISYS CORE: the divider went 0 -> 3 with "
                  "SYSCLK on it, the program went on, and the silicon reads 2 MHz",
                  sw.ok && s.sws == SysclkSource::hsisys && s.hsidiv == 3u);
    bench.verdict("LOW-POWER RUN ENTERED LAST: LPR set and REGLPF standing (the regulator "
                  "really moved), VOS 2, no wait state, rate_index 2",
                  is_slow(s));
    const uint32_t good = loop_run(64);
    print(serial, "  the single-wire loop at 2 MHz: ", good, " of 64 bytes exact (BRR ",
          hex(Usart<1>::brr()), ")", crlf);
    bench.verdict("the rebased USART reads back what it sends at 2 MHz (BRR 17: 117647 "
                  "baud, self-consistent on a loop)",
                  good == 64u);
    const uint32_t ms = delay_rounds_ms(500);
    print(serial, "  500 x delay_us(999) at 2 MHz: ", ms, " ms on the RTC wall", crlf);
    bench.verdict("the CPU is at 2 MHz: 500 waits in 500 ms on the RTC wall, plus the "
                  "calls' own cycles - which are 32 times dearer here",
                  delay_rounds_ok(500, ms));
}

// =============================================================================
// d - the rise back to 64, then the ladder
// =============================================================================
void td_rise_and_ladder() {
    feed();
    if (SysClock::rate_index() != r_slow) {
        (void)go(r_slow);
    }
    const Switch sw = go(r_fast);
    const ClockState s = state();
    print(serial, "  set_index(fast) -> ", sw.ok, " in ", sw.us, " us", crlf);
    print_state("fast", s);
    bench.verdict("THE RISE LANDS: low-power run left (LPR clear, REGLPF clear), Range 1, "
                  "two wait states, SYSCLK on the PLL at 64 MHz - the whole rising order "
                  "of 4.3.2 and 4.1.4 in one verb",
                  sw.ok && is_fast(s));
    const uint32_t good = loop_run(64);
    print(serial, "  the single-wire loop back at 64 MHz: ", good, " of 64 bytes exact", crlf);
    bench.verdict("the USART's divisor came back with the rate", good == 64u);
    const uint32_t ms = delay_rounds_ms(500);
    print(serial, "  500 x delay_us(999) at 64 MHz: ", ms, " ms on the RTC wall", crlf);
    bench.verdict("the CPU is back at 64 MHz on the RTC wall's scale",
                  delay_rounds_ok(500, ms));

    // THE LADDER, twenty-four times round, the loop exchanging at every rung.
    constexpr uint8_t rounds = 24;
    uint32_t failed_switches = 0;
    uint32_t bad_bytes = 0;
    uint32_t bad_states = 0;
    uint32_t worst_us[3] = {0, 0, 0};    // into fast / mid / slow
    for (uint8_t r = 0; r < rounds; ++r) {
        feed();
        static const uint8_t order[3] = {r_mid, r_slow, r_fast};
        for (uint8_t k = 0; k < 3; ++k) {
            const uint8_t target = order[k];
            const uint32_t t0 = mono();
            if (!SysClock::set_index(target)) ++failed_switches;
            const uint32_t us = mono_us(mono_delta(t0, mono()));
            if (us > worst_us[target]) worst_us[target] = us;
            const ClockState st = state();
            if (!checker_of(target)(st)) ++bad_states;
            bad_bytes += 16u - loop_run(16);
        }
    }
    print(serial, "  the ladder ", rounds, " times round (", rounds * 3u, " switches): ",
          failed_switches, " refused, ", bad_states, " wrong states, ", bad_bytes,
          " bad bytes of ", rounds * 48u, "; worst durations into fast ", worst_us[r_fast],
          " us, into mid ", worst_us[r_mid], " us, into slow ", worst_us[r_slow], " us", crlf);
    bench.verdict("SEVENTY-TWO SWITCHES ROUND THE LADDER: none refused, the silicon's state "
                  "right after every one, and the rebased USART exact at every rung",
                  failed_switches == 0u && bad_states == 0u && bad_bytes == 0u);
    bench.verdict("every transition under two milliseconds",
                  worst_us[0] < 2000u && worst_us[1] < 2000u && worst_us[2] < 2000u);
}

// =============================================================================
// e - the ADC across the ladder
// =============================================================================
constexpr AdcConfig cfg_pclk{
    .clock_mode = AdcClockMode::pclk_div1,
    .sample1 = AdcSampleTime::cycles160_5,
};
constexpr AdcConfig cfg_async{
    .clock_mode = AdcClockMode::async,
    .async_source = AdcAsyncSource::hsi16,
    .prescaler = AdcPresc::div2,
    .sample1 = AdcSampleTime::cycles160_5,
};

/// VREFINT, the median of nine.
uint16_t vrefint_raw() {
    uint16_t v[9];
    for (uint8_t i = 0; i < 9u; ++i) {
        v[i] = Adc::read();
    }
    for (uint8_t i = 1; i < 9u; ++i) {
        for (uint8_t j = i; j > 0u && v[j - 1u] > v[j]; --j) {
            const uint16_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
        }
    }
    return v[4];
}

void te_adc() {
    feed();
    (void)go(r_mid);
    // A converter on PCLK/1 is LEGAL at 16 MHz and not at 64: the rebase
    // is what carries it up the ladder.
    const bool up = Adc::init(clock, cfg_pclk);
    Adc::vrefint(true);
    (void)delay_us(clock, 100);
    const bool sel = Adc::select_sync(AdcInput::vrefint);
    bench.verdict("the ADC comes up on PCLK/1 at 16 MHz with VREFINT selected", up && sel);

    struct Rung { const char* name; uint8_t index; AdcClockMode expect; };
    static const Rung rungs[4] = {
        {"16 MHz mid", r_mid, AdcClockMode::pclk_div1},
        {"64 MHz fast", r_fast, AdcClockMode::pclk_div2},
        {"2 MHz slow", r_slow, AdcClockMode::pclk_div1},
        {"16 MHz mid", r_mid, AdcClockMode::pclk_div1},
    };
    uint16_t vdda[4] = {0, 0, 0, 0};
    bool modes_ok = true;
    bool enabled_ok = true;
    for (uint8_t i = 0; i < 4; ++i) {
        feed();
        (void)go(rungs[i].index);
        const AdcClockMode m = Adc::clock_mode();
        if (m != rungs[i].expect) modes_ok = false;
        if (!Adc::enabled()) enabled_ok = false;
        const uint16_t raw = vrefint_raw();
        vdda[i] = Adc::vdda_mv(raw);
        print(serial, "  ", rungs[i].name, ": CKMODE ", static_cast<uint8_t>(m), " (fADC ",
              Adc::adc_hz(SysClock::hz()) / 1000u, " kHz), VREFINT ", raw, " -> VDDA ",
              vdda[i], " mV", crlf);
    }
    bench.verdict("THE DIVISION FOLLOWS THE RATE: /1 at 16 MHz, /2 at 64 (fADC would be 64 "
                  "MHz - the rebase moved it, converter disabled and re-enabled), /1 "
                  "again at 2 MHz and at 16 - a pure function of the config and the rate",
                  modes_ok && enabled_ok);
    uint16_t lo = 0xFFFF, hi = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        if (vdda[i] < lo) lo = vdda[i];
        if (vdda[i] > hi) hi = vdda[i];
    }
    bench.verdict("VDDA through VREFINT agrees within 30 mV across the four rungs - the "
                  "calibration factor holds across the clock changes, low-power run "
                  "included",
                  lo > 3000u && static_cast<uint16_t>(hi - lo) <= 30u);

    // The control: the asynchronous mode at 2 MHz in low-power run - HSI16/2
    // whatever the bus does, and a rebase that does nothing.
    (void)go(r_slow);
    const bool re = Adc::disable() && Adc::configure(cfg_async) && Adc::enable();
    Adc::vrefint(true);
    (void)delay_us(clock, 100);
    (void)Adc::select_sync(AdcInput::vrefint);
    const uint16_t raw_async = vrefint_raw();
    const uint16_t vdda_async = Adc::vdda_mv(raw_async);
    const AdcClockMode m_async = Adc::clock_mode();
    (void)go(r_fast);
    const AdcClockMode m_async_after = Adc::clock_mode();
    print(serial, "  async HSI16/2 at 2 MHz LPR: VREFINT ", raw_async, " -> VDDA ",
          vdda_async, " mV; CKMODE ", static_cast<uint8_t>(m_async), " before and ",
          static_cast<uint8_t>(m_async_after), " after a rise to 64 MHz", crlf);
    bench.verdict("THE ASYNCHRONOUS CONVERTER AGREES (within 30 mV) at 2 MHz in low-power "
                  "run and its rebase is the no-op it should be: CKMODE async before and "
                  "after a switch",
                  re && vdda_async > 3000u &&
                      static_cast<uint16_t>(vdda_async > lo ? vdda_async - lo : lo - vdda_async) <= 30u &&
                      m_async == AdcClockMode::async && m_async_after == AdcClockMode::async);
    Adc::release();
}

// =============================================================================
// f - a kernel through the switches
// =============================================================================
void tf_kernel() {
    feed();
    console_drain();
    (void)go(r_fast);
    WalkKernel::init_all();
    Metronome::clear();
    Walker::clear();
    Metronome::beat.arm_every(50u);
    Walker::step.arm_every(300u);
    const uint32_t t0 = Tb::ticks();
    while (Tb::ticks() - t0 < 3072u) {
        feed();
        TimeEvents<P>::process();
        if (!WalkKernel::step()) {
            WalkKernel::idle_if_empty();
        }
    }
    Metronome::beat.disarm();
    Walker::step.disarm();
    while (WalkKernel::step()) {
    }
    (void)go(r_fast);
    print(serial, "  3 s of kernel: the 50-tick periodic fired ", Metronome::fired,
          " times, intervals ", Metronome::min_ticks, "..", Metronome::max_ticks,
          " ticks, early ", Metronome::early, "; the walker switched ", Walker::switches,
          " times (", Walker::failures, " refused), rates seen by the periodic 0x",
          hex(Metronome::seen_rates), crlf);
    bench.verdict("A PERIODIC KEEPS ITS CADENCE THROUGH TEN SWITCHES: sixty firings at "
                  "50..51 ticks, none early - kernel time runs on its own 32 kHz "
                  "root and a "
                  "SYSCLK change is invisible to it",
                  within(Metronome::fired, 59u, 62u) && Metronome::min_ticks == 50u &&
                      Metronome::max_ticks <= 51u && Metronome::early == 0u);
    bench.verdict("the walker's ten switches all landed and the periodic ran at all four "
                  "rates",
                  Walker::switches == 10u && Walker::failures == 0u &&
                      Metronome::seen_rates == 0xFu);
}

// =============================================================================
// g - Stop 1 at 64 MHz and at 2 MHz low-power run
// =============================================================================
/// One round through the manager: `depth` deep is Stop 1, standby is
/// Stop 0. THE WITNESS THAT THE STOP HAPPENED is the MCO wall: TIM2
/// counts HSI16/64 only while HSI16 runs and its bus is clocked, i.e.
/// only awake - a WFI that fell through as a Sleep (table 31's own
/// escape: a Stop whose entry is refused "is ignored and program
/// execution continues") counts 75000 in 300 ms, a Stop a few hundred.
struct StopRound {
    uint32_t to_blip_ms = 0;
    uint32_t kernel_ticks = 0;
    uint32_t awake_counts = 0;     ///< TIM2 counts across the round
    uint32_t irqs = 0;
    uint32_t restores = 0;
    ClockState before{}, at_blip{}, after{};
    bool ok = false;               ///< one blip, the manager's round closed
};
StopRound stop_round(const char* name, uint8_t index, SleepDepth depth) {
    StopRound r{};
    feed();
    console_drain();
    (void)go(index);
    r.before = state();
    K::init_all();
    Probe::clear();
    lptim_irqs = 0;
    isr_restores = 0;
    kernel_live = true;
    const uint32_t w0 = wall();
    const uint32_t k0 = Tb::ticks();
    const uint32_t c0 = t2();   // the AWAKE meter, not the wall
    Probe::deadline.arm(300u);
    post<Manager>(SleepRequested{depth, reply_to<Probe, SleepVote>()});
    pump_until_blip(3000u);
    kernel_live = false;
    r.awake_counts = t2() - c0;
    r.to_blip_ms = wall_ms(wall_delta(w0, Probe::blip_wall));
    r.kernel_ticks = Probe::blip_ticks - k0;
    r.irqs = lptim_irqs;
    r.restores = isr_restores;
    r.at_blip = Probe::blip_state;
    r.after = state();
    r.ok = Probe::blips == 1u && Site::armed() == SleepDepth::none;
    print(serial, "  ", depth == SleepDepth::deep ? "Stop 1" : "Stop 0", " at ", name,
          ": a 300-tick deadline matured after ", r.to_blip_ms, " ms of RTC wall, kernel ticks ",
          r.kernel_ticks, ", ", r.irqs, " LPTIM interrupt(s) of which ", r.restores,
          " restored the clock, votes ", Probe::votes, " wakes ", Probe::wakes,
          "; TIM2 on ", etr_wall_ok ? etr_wall_name : "PCLK", " counted ",
          r.awake_counts, " (", awake_us(r.awake_counts),
          " us awake of ", r.to_blip_ms, " ms)", crlf);
    print_state("  at the deadline", r.at_blip);
    print_state("  after the round", r.after);
    return r;
}
/// Under 50 ms of the round's 300 awake, on whichever wall this part has.
bool stopped(const StopRound& r) { return awake_us(r.awake_counts) < 50'000u; }

void tg_stop() {
    const StopRound fast = stop_round("64 MHz on the PLL", r_fast, SleepDepth::deep);
    bench.verdict("the deadline met through the Stop, never early",
                  fast.ok && within(fast.to_blip_ms, 292u, 320u) && within(fast.kernel_ticks, 300u, 302u));
    bench.verdict("THE STOP WAS A STOP: TIM2 on the wall counted only the awake "
                  "fraction of the round - under 50 ms of 300, where a Sleep "
                  "would count the whole of it",
                  stopped(fast));
    bench.verdict("THE RATE IN FORCE CAME BACK: restore() put the silicon where the "
                  "index says - from the wake's own ISR, before the deadline's AO ran - "
                  "and it still stands after the round",
                  is_fast(fast.before) && is_fast(fast.at_blip) && is_fast(fast.after));
    // EVERY LPTIM wake of the round, not "exactly one": a deadline that
    // lands near the timebase's own lap boundary is served by two
    // interrupts (the lap's ARRM and the deadline's CMPM), which is the
    // tickless suite's subject and not this letter's - so what is judged
    // here is that the ISR restored the clock on every one of them.
    print(serial, "  the round took ", fast.irqs, " LPTIM interrupt(s) and ",
          fast.restores, " of them re-locked the PLL", crlf);
    bench.verdict("at 64 MHz EVERY wake ISR of the round found SYSCLK on "
                  "HSISYS and re-locked the PLL there",
                  fast.irqs >= 1u && fast.restores == fast.irqs);

    const StopRound slow = stop_round("2 MHz in low-power run", r_slow, SleepDepth::deep);
    bench.verdict("the deadline met through the Stop 1 from low-power run, never early, "
                  "and the Stop was a Stop",
                  slow.ok && within(slow.to_blip_ms, 292u, 320u) && stopped(slow));
    bench.verdict("AT 2 MHz THE PART WOKE IN LOW-POWER RUN WITH HSIDIV KEPT (4.3.6): LPR "
                  "and REGLPF standing after the wake, SYSCLK on HSISYS/8 - restore() "
                  "found the rate already in force, the wake ISR restored nothing",
                  is_slow(slow.at_blip) && is_slow(slow.after) && slow.restores == 0u);

    // STOP 0 FROM LOW-POWER RUN: 4.3.6 describes the main regulator on
    // in Stop 0 and says of LPR only that HSIDIV must leave a 2 MHz
    // wake; 4.3.7 admits Stop 1 from low-power run in so many words and
    // Stop 0 is not spelled either way. Staged, and judged by the wall.
    const StopRound lpr0 = stop_round("2 MHz in low-power run", r_slow, SleepDepth::standby);
    bench.verdict("FINDING: A STOP 0 IS ENTERED FROM LOW-POWER RUN - the deadline met on "
                  "the wall with TIM2 counting only the awake fraction, and the part back "
                  "in low-power run with HSIDIV kept",
                  lpr0.ok && within(lpr0.to_blip_ms, 292u, 320u) && stopped(lpr0) &&
                      is_slow(lpr0.at_blip) && is_slow(lpr0.after));
    (void)go(r_fast);
}

// =============================================================================
// h - delay_us at every rung
// =============================================================================
void th_delay() {
    // THE INSTRUMENT IS THE WALL'S QUANTUM. A 20 us call cannot be judged
    // on a 30.5 us tick, so where the part's ETRSEL has no MCO code the
    // letter says so and claims nothing rather than measuring the
    // quantum instead of the delay.
    if constexpr (!wall_on_mco) {
        print(serial,
              "  SKIPPED, no verdict claimed: judging a 20 us delay needs the "
              "4 us wall, which is TIM2's ETR taking MCO - and RM0444 "
              "22.4.25's ETRSEL list gives code 0100 to the G0B1/G0C1 sales "
              "types alone (tim_etrsel_has_mco(2) is false here). The LSE "
              "code 0011 every part has makes a 30.5 us tick, coarser than "
              "three of this letter's four spans, so what it would measure is "
              "its own quantum.",
              crlf);
        return;
    }
    static const uint32_t spans[] = {20, 100, 500, 900};
    static const uint8_t rungs[3] = {r_fast, r_mid, r_slow};
    static const char* const names[3] = {"64 MHz", "16 MHz", "2 MHz"};
    bool never_early = true;
    bool bounded = true;
    for (uint8_t r = 0; r < 3; ++r) {
        feed();
        (void)go(rungs[r]);
        print(serial, "  at ", names[r], ":");
        for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
            const uint32_t t0 = t2();
            const bool ok = delay_us(clock, spans[i]);
            const uint32_t took = t2_us(t2() - t0);
            print(serial, " ", spans[i], "->", took);
            // The wall's quantum is 4 us; the loop's own overhead grows
            // 32-fold at 2 MHz (a poll is ~20 cycles).
            if (!ok || took + 4u < spans[i]) never_early = false;
            if (took > spans[i] + (rungs[r] == r_slow ? 60u : 16u)) bounded = false;
        }
        print(serial, " us", crlf);
        const bool refused = !delay_us(clock, 1000u);
        if (!refused) bounded = false;
    }
    (void)go(r_fast);
    bench.verdict("delay_us SERVES AT LEAST ITS ARGUMENT AT EVERY RUNG - the per-rate "
                  "table selected by rate_index(), no division at wait time",
                  never_early);
    bench.verdict("and never much more (the wall's 4 us, the poll's own cost at 2 MHz), "
                  "with the millisecond cap refused at every rate",
                  bounded);
}

// =============================================================================
// i - the SysTick ticker as a clock user
// =============================================================================
void ti_systick_ticker() {
    feed();
    console_drain();
    (void)go(r_fast);
    // Hand SysTick over: the counter off, the ticker on with its
    // interrupt (SysTick_Handler is bound below and counts its runs).
    SysTickCounter::stop();
    systick_irqs = 0;
    const bool up = Ticker::init(clock);
    bench.verdict("BasicTicker comes up on a DynamicClock (it is listed among the users)", up);

    static const uint8_t rungs[4] = {r_mid, r_slow, r_pll16, r_fast};
    static const char* const names[4] = {"16 MHz R2", "2 MHz LPR", "16 MHz PLL R2", "64 MHz"};
    bool rate_held = true;
    bool never_fast = true;
    for (uint8_t i = 0; i < 4; ++i) {
        feed();
        console_drain();
        const Switch sw = go(rungs[i]);
        // 500 ms of the crystal: the ticker must count 500 (+-1 %), and
        // never MORE than the wall says - a wrong reload would run it
        // 4x or 32x fast.
        const uint32_t w0 = wall();
        const uint32_t s0 = Ticker::ticks();
        const uint32_t i0 = systick_irqs;
        while (rtc_us(w0, wall()) < 500'000u) {
        }
        const uint32_t counted = Ticker::ticks() - s0;
        const uint32_t irqs = systick_irqs - i0;
        print(serial, "  ", names[i], " (switch ", sw.ok, ", ", sw.us, " us): ", counted,
              " SysTick ticks in 500 ms of the RTC wall, ", irqs,
              " handler runs", crlf);
        // THE BAND IS THE WALL'S. A crystal wall resolves a per cent;
        // an RC one does not - and on this die the 2 MHz rung reads
        // about 1.5 % fast against an LSI wall where the other three
        // read half a per cent, which is either the oscillator moving
        // with the voltage regime or the core doing so, and this
        // instrument cannot say which. What the letter is FOR survives
        // either way: a reload that had not followed the rate would be
        // four or thirty-two times out, not one and a half per cent.
        const uint32_t slack = wall_on_lse ? 6u : 16u;
        if (!within(counted, 500u - slack, 500u + slack) || irqs != counted) {
            rate_held = false;
        }
        if (counted > 500u + slack) never_fast = false;
    }
    bench.verdict("THE SYSTICK TICKER HOLDS 1000 Hz ACROSS THE LADDER: rebase() reloads it "
                  "at every switch, 500 ticks per 500 ms of the RTC wall at 16, "
                  "2, 16 (PLL) and 64 MHz - inside the wall's own resolution, "
                  "which is a per cent on a crystal and three on an RC root",
                  rate_held);
    bench.verdict("and never runs FAST - the restarted period costs under a tick, late",
                  never_fast);
    // Hand it back: the interrupt-less counter, and the handler silent.
    Ticker::pause();
    const bool back = SysTickCounter::start(clock);
    const uint32_t i1 = systick_irqs;
    tb_wait_ms(20);
    bench.verdict("SysTick handed back to the interrupt-less counter: delay_us works and "
                  "the handler no longer runs",
                  back && delay_us(clock, 10) && systick_irqs == i1);
}

// =============================================================================
// j - a PLL rate in Range 2
// =============================================================================
void tj_pll_range2() {
    feed();
    (void)go(r_fast);
    const Switch sw = go(r_pll16);
    const ClockState s = state();
    const uint32_t cfg = RCC->PLLCFGR;
    const uint8_t n = static_cast<uint8_t>((cfg & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos);
    const uint8_t r = static_cast<uint8_t>(((cfg & RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos) + 1u);
    const uint8_t m = static_cast<uint8_t>(((cfg & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos) + 1u);
    print(serial, "  set_index(pll16) from 64 MHz -> ", sw.ok, " in ", sw.us, " us; PLLCFGR M ",
          m, " N ", n, " R ", r, " (VCO ", (16u / m) * n, " MHz)", crlf);
    print_state("pll16", s);
    bench.verdict("A PLL RATE IN RANGE 2: PLLRCLK at 16 MHz from a VCO at 128 MHz (M 1, N 8, "
                  "R 8 - table 47's Range 2 ceiling), VOS 2, one wait state, reached from "
                  "the 64 MHz PLL by a reconfiguration through HSISYS",
                  sw.ok && is_pll16(s) && m == 1u && n == 8u && r == 8u);
    const uint32_t good = loop_run(64);
    const uint32_t ms = delay_rounds_ms(500);
    print(serial, "  the single-wire loop: ", good, " of 64 exact; 500 x delay_us(999): ", ms,
          " ms on the RTC wall", crlf);
    bench.verdict("the rebased USART exact and the CPU at 16 MHz on the RTC wall's scale",
                  good == 64u && delay_rounds_ok(500, ms));

    // From and to the other end of the ladder, eight times each way.
    uint32_t bad = 0;
    uint32_t worst_from_slow = 0, worst_to_slow = 0;
    for (uint8_t k = 0; k < 8; ++k) {
        feed();
        Switch a = go(r_slow);
        if (!a.ok || !is_slow(state())) ++bad;
        if (a.us > worst_to_slow) worst_to_slow = a.us;
        Switch b = go(r_pll16);
        if (!b.ok || !is_pll16(state())) ++bad;
        if (b.us > worst_from_slow) worst_from_slow = b.us;
        bad += 8u - loop_run(8);
    }
    print(serial, "  pll16 <-> 2 MHz LPR eight times: ", bad, " faults; worst pll16 -> slow ",
          worst_to_slow, " us, slow -> pll16 ", worst_from_slow, " us", crlf);
    bench.verdict("the PLL in Range 2 is left for low-power run and re-locked from it, "
                  "the range never leaving 2, the loop exact each time",
                  bad == 0u);
    const Switch back = go(r_fast);
    bench.verdict("and back to 64 MHz in Range 1 (the range up, then the PLL reconfigured)",
                  back.ok && is_fast(state()));
}

// =============================================================================
// x - the durations, broken down (outside z)
// =============================================================================
void tx_durations() {
    static const uint8_t from[10] = {r_fast, r_mid, r_slow, r_fast, r_slow, r_mid, r_fast, r_pll16, r_slow, r_pll16};
    static const uint8_t to[10] = {r_mid, r_slow, r_fast, r_slow, r_mid, r_fast, r_pll16, r_fast, r_pll16, r_slow};
    static const char* const names[4] = {"fast", "mid", "slow", "pll16"};
    for (uint8_t i = 0; i < 10; ++i) {
        (void)go(from[i]);
        uint32_t lo = 0xFFFFFFFFu, hi = 0;
        for (uint8_t k = 0; k < 8; ++k) {
            (void)go(from[i]);
            const Switch s = go(to[i]);
            if (s.us < lo) lo = s.us;
            if (s.us > hi) hi = s.us;
        }
        print(serial, "  ", names[from[i]], " -> ", names[to[i]], ": ", lo, "..", hi, " us over 8", crlf);
    }
    (void)go(r_fast);
    bench.verdict("durations printed", true);
}

void banner() {
    print(serial, "test_stm32_clock - THE DYNAMIC CLOCK on the tickless platform: ",
          "64 M PLL R1 / 16 M HSISYS R2 / 2 M HSISYS LPR; console on HSI16, USART1 loop on PCLK",
          crlf);
    bench.menu();
}

} // namespace

// ---------------------------------------------------------------------------
// Vectors
// ---------------------------------------------------------------------------

extern "C" void BRIO_SUITE_CONSOLE_HANDLER() { (void)Serial::isr(); }

/// Letter i's only: the BasicTicker's tick while it holds SysTick.
extern "C" void SysTick_Handler() {
    Ticker::tick();
    systick_irqs = systick_irqs + 1u;
}
extern "C" void USART1_IRQHandler() { (void)Loop::isr(); }

/// The timebase's vector - and, this program's choice, where the rate
/// comes back after a Stop: a Stop lands on HSISYS and the plain site
/// restores the clock at disarm(), which the manager calls at the wake
/// convention - AFTER the AO the deadline was for has run, at 16 MHz
/// with the PLL off (measured: the first version of letter g read
/// SWS=HSISYS at the deadline). A program that wants its rate back
/// before any AO runs calls restore() from the wake's own ISR, as here;
/// with no Stop in the way (every other letter) it is one register read.
extern "C" void BRIO_STM32G0_LPTIM1_HANDLER() {
    (void)Tb::isr();
    lptim_irqs = lptim_irqs + 1u;
    if (brio::Rcc::sysclk_status() != SysClock::rate_source(SysClock::rate_index()) &&
        !SysClock::switching()) {
        isr_restores = isr_restores + 1u;
        (void)SysClock::restore();
    }
}

extern "C" void RTC_TAMP_IRQHandler() {
    rtc_wakes = rtc_wakes + 1u;
    (void)brio::Rtc::isr();
    if (kernel_live) {
        brio::post<Probe>(Woke{});
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    brio::Pwr::bus_clock(true);
    brio::Pwr::rtc_domain_unlock(true);
    brio::RtcDomain::apb_clock(true);

    // The RTC domain as the tickless suite takes it: on LSE, or empty and
    // then taken for the crystal.
    // THE RTC WALL. RTCSEL is one-way, so what the domain already holds
    // decides: a domain on either 32 kHz root is kept, and only an empty
    // one is claimed - for the crystal if it starts, for LSI if it does
    // not. The LSI's rate is not a constant, so it is MEASURED (letter
    // a prints it) rather than assumed.
    const brio::RtcClockSource sel = brio::RtcDomain::selected();
    if (sel == brio::RtcClockSource::none) {
        brio::RtcDomain::lse_enable(true);
        wall_on_lse = brio::RtcDomain::lse_wait_ready(4'000'000UL);
        if (!wall_on_lse) {
            brio::RtcDomain::lse_enable(false);
            brio::Rcc::lsi_enable(true);
            (void)brio::Rcc::lsi_wait_ready();
        }
        (void)brio::RtcDomain::open(wall_on_lse ? brio::RtcClockSource::lse
                                                : brio::RtcClockSource::lsi);
    } else {
        wall_on_lse = sel == brio::RtcClockSource::lse;
    }
    if (!wall_on_lse) {
        const uint32_t measured = measure_lsi_hz();
        if (measured != 0u) {
            rtcclk_hz = measured;
        }
    }
    brio::Rtc::bypass_shadow(true);
    wall_ready = wall_up();
    (void)brio::Rtc::wake_line_open();
    brio::Nvic::enable(brio::Rtc::irq());

    const bool serial_ok = Serial::init(clock, 115200);
    const bool loop_ok = Loop::init(clock, 115200);
    const bool mco_ok = wall_mco_up();
    const bool tick_ok = Tb::init(clock);
    // THE WALL IS PROBED AND NOT ASSUMED: a counter with nothing on its
    // ETR stands still in silence, and every bounded wait in this suite
    // would then be unbounded. Where it is dead, TIM2 goes on PCLK as
    // letter g's awake meter instead.
    if (tick_ok) {
        const uint32_t c0 = t2();
        const uint32_t t0 = Tb::millis();
        while (Tb::millis() - t0 < 20u) {
        }
        etr_wall_ok = (t2() - c0) > 8u;
        if (!etr_wall_ok) {
            T2::enable(false);
            (void)T2::external_trigger({.clock_mode2 = false});
            (void)T2::configure({.prescaler = 0, .period = 0xFFFFFFFFu});
            T2::enable(true);
        }
    }
    const bool wd = brio::Iwdg::arm(brio::IwdgConfig{
        .prescaler = brio::IwdgPrescaler::div256,
        .reload = 0x0FFF,
        .window = 0x0FFF});
    brio::enable_interrupts();

    bench.letter('a', "the boot rate and the pack; the MCO wall; the PLL's refusal", ta_boot);
    bench.letter('b', "the fall to 16 MHz in Range 2", tb_fall_mid);
    bench.letter('c', "the fall to 2 MHz in low-power run", tc_fall_slow);
    bench.letter('d', "the rise back to 64, then the ladder 24 times round", td_rise_and_ladder);
    bench.letter('e', "the ADC across the ladder", te_adc);
    bench.letter('f', "a kernel through the switches", tf_kernel);
    bench.letter('g', "Stop 1 at 64 MHz and at 2 MHz low-power run", tg_stop);
    bench.letter('h', "delay_us at every rung", th_delay);
    bench.letter('i', "the SysTick ticker as a clock user", ti_systick_ticker);
    bench.letter('j', "a PLL rate in Range 2", tj_pll_range2);
    bench.letter('x', "diagnostic: the durations, broken down (outside z)", tx_durations, false);

    if (serial_ok) {
        const auto idcode = brio::DeviceIdcode::read();
        print(serial, crlf, "part DEV_ID ", hex(idcode.dev_id),
              " REV_ID ", hex(idcode.rev_id), crlf);
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz (rate 0 of 3)" : "FAILED",
              " console=", console_on_lpuart ? "LPUART1 on HSI16"
                                            : "USART2 on HSI16",
              " loop=", loop_ok ? "USART1 PA9 on PCLK" : "FAILED",
              " tick=", tick_ok ? (tb_cfg.source == LptimTickerSource::lse
                                       ? "LPTIM1 on LSE, tickless"
                                       : "LPTIM1 on LSI, tickless")
                                : "FAILED",
              " wall=", wall_ready ? (wall_on_lse ? "RTC on LSE" : "RTC on LSI")
                                   : "NO RTC",
              " RTCCLK=", rtcclk_hz,
              " mco=", mco_ok ? (etr_wall_ok ? "TIM2 ETR runs"
                                             : "TIM2 ETR DEAD, on PCLK instead")
                              : "FAILED",
              " backstop=", wd ? "IWDG 32 s" : "FAILED", crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        feed();
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
