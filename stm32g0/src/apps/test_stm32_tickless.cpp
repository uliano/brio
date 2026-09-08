// test_stm32_tickless - THE TICKLESS KERNEL TIMEBASE: kernel time on an
// LPTIM clocked from the LSE crystal, the kernel's optional idle_until()
// placing every deadline in the compare register, and NO periodic
// interrupt anywhere in the program (SysTick runs interrupt-less as
// delay_us's cycle counter). No wires.
//
// This whole image runs on Stm32g0Platform<LptimTicker<>>: the timebase
// is a template argument, so a tickless program is a whole program and
// the SysTick suites stay what they are. What is judged here:
//
//   a  THE TIMEBASE - SysTick counting with its interrupt off (and the
//      bound handler proving it never runs), the LPTIM's rate against
//      TIM2 on the PLL, millis()/secs()/now() exact to their arithmetic,
//      monotonic under two hundred thousand reads
//   b  THE LAP CARRY on a second LptimTicker, LPTIM2 at the crystal's
//      own rate (a lap of two seconds): monotonic across wraps in a
//      tight loop, and with PRIMASK held across a wrap - the pending-ARRM
//      correction ticks() carries for exactly that case; plus the one
//      compare question the driver's rule 4 leaves to the bench (does
//      CMPM fire at CMP == ARR?). LPTIM2's counter runs on silicon here
//      for the first time.
//   c  idle_until() OUTSIDE THE KERNEL: N ticks asked, N ticks slept
//      (against TIM2), exactly one LPTIM interrupt and it served CMPM,
//      never early in the timebase's own units; the +2 floor's one-tick
//      lateness for N = 1; a due deadline that does not sleep; no
//      deadline at all, ended by a foreign wake (the RTC's wake-up timer)
//   d  THE HANDSHAKE the errata force: two arms back to back, the second
//      deferred exactly once (rule 1), the register reading the second
//      value, CMPM at the second deadline; and rule 3's question staged
//      as a FINDING - a compare stored immediately before a Stop 1, with
//      and without the wait, judged on the RTC's wall
//   e  A REAL KERNEL: three periodic events at 7, 30 and 250 ticks
//      pumped through process()/step()/idle_if_empty() for three seconds,
//      every firing stamped on TIM2 in its own AO - never early, one
//      LPTIM wake per distinct deadline, and the SysTick handler still
//      never run
//   f  STOP 1 THROUGH THE POWER MANAGER with the PLAIN site (the timed
//      ones refuse this platform at compile time): a 500 ms deadline
//      matures on the RTC wall with kernel time having RUN through the
//      Stop - the restriction the plain site carries on the SysTick
//      timebase is simply gone - SYSCLK back on the PLL after the round
//   g  delay_us on the interrupt-less SysTick, the platform suite's
//      arithmetic re-run here
//   h  AN LSI-CLOCKED LptimTicker as a witness on LPTIM2: the internal
//      RC at the rate the program STATES (32586 Hz, this die's measured
//      one), its ticks and millis() against the crystal, a wake placed
//      on it and served, and the directional rule's price at the
//      default statement (table 46's ceiling)
//   i  THE LAP WAKES OF A LONG STOP: ten seconds in Stop 1 with one
//      deadline at the end - five ARRM wakes to carry the high word,
//      each on HSISYS with the PLL never re-locked, their awake time
//      counted by TIM3 on MCO = HSI16/8 (a clock that runs only while
//      the part is awake) - what a rare-event program pays for a
//      16-bit counter that must not be divided
//   u  OUTSIDE z: a console keystroke as the foreign wake of a twenty-
//      second idle_until (an operator's hand).
//
// Instruments: TIM2 free-running at 64 MHz on the PLL as the awake
// wall (15.6 ns), the RTC's sub-second counter on the same crystal as
// the wall a Stop cannot stop, the IWDG as the backstop of every sleep.
//
// build: boards = g0b1re,g071rb,g031k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/lptim.hpp"
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

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// THE TIMEBASE, and the platform on it. An LPTIM on the 32 kHz crystal,
// which every board of this desk runs. A board without one would put
// the ticker on LSI here and STATE its rate, the number to state being
// one NOT BELOW the true one (the driver's own directional rule: a tick
// declared faster than it is makes every deadline late and none early)
// and a power of two, since the tick is the count SHIFTED and a
// power-of-two statement keeps the whole timebase's arithmetic a shift
// (1024 ticks a second, millis() 1000 per 1024, secs() a right shift by
// ten) - the crystal's own 32768 serves both, at the price of a tick some
// per cent longer than it says on any die of table 46. Letter h below is
// where that LSI arrangement is measured, on a witness of its own.
constexpr LptimTickerConfig tb_cfg{};
using Tb = LptimTicker<tb_cfg>;
using P = Stm32g0Platform<Tb>;
static_assert(Tickless<Tb>);
static_assert(P::ticks_per_second == lptim_ticker_hz(tb_cfg));

// The lap witness of letter b: LPTIM2 at the root's own rate, undivided.
constexpr LptimTickerConfig witness_cfg{.instance = 2, .shift = 0,
                                        .source = tb_cfg.source,
                                        .lsi_hz = tb_cfg.lsi_hz};
using Witness = LptimTicker<witness_cfg>;
static_assert(Witness::ticks_per_second == lptim_ticker_count_hz(tb_cfg));
/// The counter's own lap and the rate the witness counts at: a second of
/// it is `witness_hz` counts whatever the root.
constexpr uint32_t witness_hz = lptim_ticker_count_hz(witness_cfg);

// Letter h's witness: LPTIM2 again, on LSI at the rate the die under it
// measured (test_stm32_rtc's letter c, a TIM16 capture), shift 5. That
// is a DIE fact, and the dies of this desk differ by more than the band
// the letter judges in, so the preprocessor picks the number.
#if defined(STM32G031xx)
constexpr uint32_t lsi_measured_hz = 31'496;
#else
constexpr uint32_t lsi_measured_hz = 32'586;
#endif
using LsiWitness = LptimTicker<LptimTickerConfig{
    .instance = 2, .shift = 5, .source = LptimTickerSource::lsi, .lsi_hz = lsi_measured_hz}>;
static_assert(!LsiWitness::on_crystal && LsiWitness::ticks_per_second == (lsi_measured_hz >> 5));
using LsiDefault = LptimTicker<LptimTickerConfig{
    .instance = 2, .shift = 5, .source = LptimTickerSource::lsi}>;
static_assert(LsiDefault::ticks_per_second == (34'000u >> 5));
/// Which body LPTIM2's vector serves: 0 = the crystal witness, 1 = the LSI one.
volatile uint8_t witness_mode = 0;

using L1 = Lptim<1>;
using L2 = Lptim<2>;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

using T2 = Tim<2>;
using Site = Stm32g0SleepSite<SysClock, Tb>;
static_assert(!Site::pauses_tick);

// ---------------------------------------------------------------------------
// The two walls
// ---------------------------------------------------------------------------

/// TIM2 free-running at 64 MHz: the awake wall, 15.6 ns a count, a lap
/// of 67 s. Stops with HCLK in a Stop, which is what the RTC is for.
constexpr uint32_t t2_hz = SysClock::hz;
uint32_t t2() { return T2::count(); }
uint32_t t2_us(uint32_t counts) { return counts / (t2_hz / 1'000'000u); }
bool t2_up() {
    T2::bus_clock(true);
    if (!T2::configure({.prescaler = 0, .period = 0xFFFFFFFFu})) {
        return false;
    }
    T2::enable(true);
    return true;
}

// ---- THE LSI's OWN RATE, where the walls run on it -------------------------
//
// A crystal is 32768 Hz by construction; an LSI is an RC oscillator the
// datasheet only bounds (table 46: 29.5..34 kHz), so a wall built on one
// is WEIGHED before anything is timed against it. TIM16's capture channel
// with TISEL on LSI (25.6.18's code 1) against a counter clocked from
// PCLK is that measurement, and the estimator is the MEDIAN of a batch -
// test_stm32_rtc's letter c owns the technique and the reason: an
// unfiltered capture of an internal clock line errs in BOTH directions.
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
    return median != 0u ? (SysClock::hz / (lsi_meter_prescaler + 1u)) / median : 0u;
}

/// The RTC's sub-second counter at PREDIV_A 0 / PREDIV_S 32767: a 30.5 us
/// stopwatch on the crystal that keeps counting through a Stop (the
/// lptim suite's instrument, verbatim).
constexpr RtcPrescalers wall_prescalers{.async = 0, .sync = 32767};
bool wall_ready = false;
/// What RTCCLK really is: the crystal's 32768 where the domain runs on
/// it, and the LSI's MEASURED rate where it does not - a nominal here
/// would put a per-cent error on every microsecond this suite prints.
uint32_t rtcclk_hz = 32768;
bool wall_on_lse = false;

uint32_t wall_ticks_per_second() {
    return static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
}
uint32_t wall_modulus() { return 60u * wall_ticks_per_second(); }
uint32_t wall_hz() {
    return rtcclk_hz / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
}

/// A span the KERNEL calls `ms` is this many milliseconds OF THE WALL.
/// The two clocks are the same 32 kHz root, so the ratio is exactly the
/// ticker's STATEMENT over the root's true rate: one on a crystal, and
/// four per cent on a die whose LSI measures 31496 against the 32768 the
/// ticker states. A band written in kernel milliseconds is converted
/// through here before it is compared with a wall reading, which is what
/// keeps "never early" a claim about the kernel and not about the RC.
uint32_t wall_ms_for_kernel_ms(uint32_t ms) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ms) * lptim_ticker_count_hz(tb_cfg)) / rtcclk_hz);
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

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

void feed() { Iwdg::refresh(); }

void spin_us(uint32_t us) {
    const uint32_t t0 = t2();
    const uint32_t c = us * (t2_hz / 1'000'000u);
    while (t2() - t0 < c) {
    }
}

/// A measurement window a transmit interrupt walks through is not a
/// measurement.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    spin_us(2000);
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// Wait for the very next count edge of the timebase - the metrology
/// verb the lptim suite explains: a deadline armed at an unknown phase
/// inside a count is N - 1 to N counts of real time away, and a wall
/// reading of N - 1 would be honest. Synchronizing first is what lets
/// "never early" be judged against the nominal.
void sync_to_count() {
    const uint32_t t = Tb::ticks();
    uint32_t guard = 4'000'000u;
    while (Tb::ticks() == t && guard-- != 0u) {
    }
}

/// Spin into the first half of a lap, so that the next second holds no
/// wrap: the legs that judge "one interrupt" or "the LPTIM never spoke"
/// over a window under a second assume the ARRM handler does NOT run in
/// it (it would sweep CMPOK on the deferral leg, and end a single WFI
/// early) - a phase the first versions left to luck, 12..25 % against
/// per leg, which is what a lap every two seconds costs a test and
/// costs a program nothing (the loop turns once more).
void lap_room() {
    while ((Tb::count() & 0xFFFFu) >= 0x8000u) {
        feed();
    }
}

// ---------------------------------------------------------------------------
// Shared ISR state
// ---------------------------------------------------------------------------

volatile uint32_t systick_irqs = 0;
volatile uint32_t lptim_irqs = 0;
volatile uint32_t lptim_last_served = 0;
volatile uint32_t lptim_cmpm = 0;
volatile uint32_t lptim_arrm = 0;
volatile uint32_t lptim_sweeps = 0;    ///< handler runs that served nothing
volatile uint32_t lptim_t2_at = 0;     ///< TIM2 at the last LPTIM1 interrupt
volatile uint32_t lptim_cnt_at = 0;    ///< LPTIM1 CNT at the last interrupt's entry
volatile uint32_t lptim_isr_at = 0;    ///< LPTIM1 ISR register at the last interrupt's entry
volatile uint32_t lptim2_irqs = 0;
volatile uint32_t rtc_wakes = 0;
volatile uint32_t usart_irqs = 0;
volatile bool kernel_live = false;

void clear_counters() {
    lptim_irqs = 0;
    lptim_last_served = 0;
    lptim_cmpm = 0;
    lptim_arrm = 0;
    lptim_sweeps = 0;
    lptim_t2_at = 0;
    lptim_cnt_at = 0;
    lptim_isr_at = 0;
    lptim2_irqs = 0;
    rtc_wakes = 0;
    usart_irqs = 0;
}

// ---------------------------------------------------------------------------
// The kernel half (letters e and f)
// ---------------------------------------------------------------------------

struct Fast {};
struct Mid {};
struct Slow {};
struct Blip {};
struct Woke {};

/// One periodic event's bookkeeping: every firing stamped on TIM2, the
/// worst deviation from its own cadence, and the count of early ones.
struct Track {
    uint32_t period_ticks = 0;
    uint32_t fired = 0;
    uint32_t last_t2 = 0;
    uint32_t first_t2 = 0;
    uint32_t min_us = 0xFFFFFFFFu;
    uint32_t max_us = 0;
    uint32_t last_tick = 0;
    uint32_t min_ticks = 0xFFFFFFFFu;
    uint32_t max_ticks = 0;
    uint32_t early = 0;
    void hit(uint32_t now) {
        if (fired == 0u) {
            first_t2 = now;
        } else {
            const uint32_t us = t2_us(now - last_t2);
            if (us < min_us) min_us = us;
            if (us > max_us) max_us = us;
            // "Early" is judged in the timebase's OWN units: two firings
            // closer than the period in ticks.
            const uint32_t dt = Tb::ticks() - last_tick;
            if (dt < period_ticks) ++early;
            if (dt < min_ticks) min_ticks = dt;
            if (dt > max_ticks) max_ticks = dt;
        }
        last_t2 = now;
        last_tick = Tb::ticks();
        ++fired;
    }
};

/// One AO with three periodic events, each stamping TIM2 at every
/// firing and keeping its worst deviation from its own cadence.
struct Metronome : Fsm<Metronome, Fast, Mid, Slow> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Metronome, Fast> fast{Fast{}};
    static inline TimeEvent<P, Metronome, Mid> mid{Mid{}};
    static inline TimeEvent<P, Metronome, Slow> slow{Slow{}};

    static inline Track tf;
    static inline Track tm;
    static inline Track ts;

    static void clear() {
        tf = Track{};
        tm = Track{};
        ts = Track{};
        tf.period_ticks = 7;
        tm.period_ticks = 30;
        ts.period_ticks = 250;
    }
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](Fast) { tf.hit(t2()); return handled(); },
            [](Mid) { tm.hit(t2()); return handled(); },
            [](Slow) { ts.hit(t2()); return handled(); });
    }
};
using MetroKernel = Kernel<P, Metronome>;

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip, Woke> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline uint16_t blips = 0;
    static inline uint16_t woke = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline bool last_ok = false;
    static inline uint32_t blip_wall = 0;
    static inline uint32_t blip_ticks = 0;
    static inline SleepDepth last_report = SleepDepth::none;

    static void clear() {
        blips = woke = wakes = votes = 0;
        last_ok = false;
        blip_wall = 0;
        blip_ticks = 0;
        last_report = SleepDepth::none;
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
        [](SleepVote v) { ++votes; last_ok = v.ok; return handled(); },
        [](const PrepareSleep& p) { p.reply.send(SleepVote{true}); return handled(); },
        [](WakeReport w) { ++wakes; last_report = w.was; return handled(); },
        [](Woke) {
            // util/power.hpp's convention: a wake path with nothing to
            // say says SleepRequested{none}. Load-bearing here too - it
            // is what puts the clock back and disarms the Stop.
            ++woke;
            post<Manager>(SleepRequested{SleepDepth::none, reply_to<Probe, SleepVote>()});
            return handled();
        },
        [](Blip) {
            ++blips;
            blip_wall = wall();
            blip_ticks = Tb::ticks();
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

/// One idle_until() by hand, masked as the kernel calls it, timed on
/// TIM2 (Sleep mode: TIM2 keeps running).
struct Nap {
    uint32_t asked = 0;          ///< the deadline, absolute
    uint32_t t2_us = 0;
    uint32_t ticks_before = 0;
    uint32_t ticks_after = 0;
    uint32_t irqs = 0;
    uint32_t served = 0;
    uint32_t isr_at = 0;         ///< LPTIM_ISR at the handler's entry
    uint32_t cnt_at = 0;         ///< CNT at the handler's entry
    uint32_t irq_us = 0;         ///< TIM2 at the handler's entry, from the nap's start
    uint16_t cmp_after = 0;      ///< the register after the arm
    uint32_t deferrals = 0;
    uint32_t stores = 0;
    uint32_t rounds = 0;         ///< idle_until calls until the deadline was due
    uint32_t rtc = 0;            ///< RTC interrupts during the nap
    uint32_t usart = 0;          ///< console interrupts during the nap
    bool interrupts_after = false;
};

/// One idle_until() by hand, masked as the kernel calls it, timed on
/// TIM2 (Sleep mode: TIM2 keeps running). The console is drained FIRST:
/// a transmit interrupt is a wake, and a print between two naps would
/// end the second one early (this stratum's oldest lesson).
Nap nap(std::optional<int32_t> delta) {
    Nap n{};
    console_drain();
    clear_counters();
    const uint32_t d0 = Tb::deferrals();
    const uint32_t s0 = Tb::stores();
    sync_to_count();
    n.ticks_before = Tb::ticks();
    const std::optional<uint32_t> deadline =
        delta.has_value() ? std::optional<uint32_t>{n.ticks_before + static_cast<uint32_t>(*delta)}
                          : std::nullopt;
    n.asked = deadline.value_or(0u);
    const uint32_t t0 = t2();
    // The loop's own shape: idle again until the deadline is due - a lap
    // wake (ARRM) or a declined arm is a turn, not a failure. With no
    // deadline, one call.
    n.interrupts_after = true;
    for (;;) {
        {
            InterruptGuard g;
            P::idle_until(deadline);
            n.interrupts_after = n.interrupts_after && interrupts_enabled();
        }
        ++n.rounds;
        if (!deadline.has_value() || static_cast<int32_t>(Tb::ticks() - *deadline) >= 0 ||
            n.rounds > 100'000u) {
            break;
        }
    }
    n.t2_us = t2_us(t2() - t0);
    n.rtc = rtc_wakes;
    n.usart = usart_irqs;
    n.ticks_after = Tb::ticks();
    n.irqs = lptim_irqs;
    n.served = lptim_last_served;
    n.isr_at = lptim_isr_at;
    n.cnt_at = lptim_cnt_at;
    n.irq_us = lptim_irqs != 0u ? t2_us(lptim_t2_at - t0) : 0u;
    n.cmp_after = L1::cmp();
    n.deferrals = Tb::deferrals() - d0;
    n.stores = Tb::stores() - s0;
    return n;
}

void print_nap(const char* what, const Nap& n) {
    print(serial, "  ", what, ": slept ", n.t2_us, " us in ", n.rounds, " round(s), ticks ",
          n.ticks_before, " -> ", n.ticks_after, " (asked ", n.asked, "), CMP 0x",
          hex(n.cmp_after), ", stores ", n.stores, " deferrals ", n.deferrals, ", rtc ", n.rtc,
          " usart ", n.usart, ", ", n.irqs, " lptim irq");
    if (n.irqs != 0u) {
        print(serial, " at ", n.irq_us, " us with CNT ", n.cnt_at, " ISR 0x", hex(n.isr_at),
              " served 0x", hex(n.served));
    }
    print(serial, crlf);
}

// =============================================================================
// a - the timebase
// =============================================================================
void ta_timebase() {
    feed();
    console_drain();

    // SysTick: counting, no interrupt, the reload BasicTicker would use.
    const uint32_t ctrl = SysTick->CTRL;
    const uint32_t load = SysTick->LOAD;
    print(serial, "  SysTick CTRL 0x", hex(ctrl), " LOAD ", load, " (", SysClock::hz / 1000u - 1u,
          " expected); SysTick_Handler ran ", systick_irqs, " times since boot", crlf);
    bench.verdict("SysTick counts with its interrupt OFF: CLKSOURCE and ENABLE set, "
                  "TICKINT clear, the millisecond reload delay_us derives its cap from",
                  (ctrl & SysTick_CTRL_ENABLE_Msk) != 0u &&
                      (ctrl & SysTick_CTRL_CLKSOURCE_Msk) != 0u &&
                      (ctrl & SysTick_CTRL_TICKINT_Msk) == 0u &&
                      load == SysClock::hz / 1000u - 1u);
    bench.verdict("and the bound SysTick_Handler has never run - this program has "
                  "no periodic interrupt",
                  systick_irqs == 0u);

    // The rate against TIM2: 2 s of the PLL's wall -> 2048 ticks. HSI16
    // against the crystal, so this is coherence (within a percent), not
    // metrology.
    sync_to_count();
    const uint32_t k0 = Tb::ticks();
    const uint32_t m0 = Tb::millis();
    const uint32_t w0 = t2();
    while (t2() - w0 < 2u * t2_hz) {
        feed();
    }
    const uint32_t ticks = Tb::ticks() - k0;
    const uint32_t ms = Tb::millis() - m0;
    constexpr uint32_t ticks_due = 2u * P::ticks_per_second;
    print(serial, "  2 s of TIM2: ", ticks, " ticks (", ticks_due,
          " due by the statement), millis() moved ", ms, " (2000 due)", crlf);
    // THE BAND IS THE ROOT'S. A crystal is 32768 Hz by construction and
    // the only spread left is HSI16's own per cent, so the two agree to
    // a count either way. An LSI is the rate the program STATED - at or
    // above the true one by the directional rule - so against a real
    // second the tick can only be SLOW, never fast, and by no more than
    // table 46's own spread. That is the claim on such a board: never
    // faster than it says, and inside the datasheet's band.
    const uint32_t ticks_lo = Tb::on_crystal ? ticks_due - 20u
                                             : ticks_due - ticks_due / 8u;
    const uint32_t ticks_hi = Tb::on_crystal ? ticks_due + 20u : ticks_due;
    const uint32_t ms_lo = Tb::on_crystal ? 1980u : 1750u;
    const uint32_t ms_hi = Tb::on_crystal ? 2020u : 2000u;
    bench.verdict("the timebase runs at the rate its configuration states, "
                  "against a wall that is not it - to a count on a crystal, "
                  "and on an RC root never FASTER than stated and inside "
                  "table 46's own spread",
                  within(ticks, ticks_lo, ticks_hi));
    bench.verdict("and millis() is 1000 per 1024 ticks - shifts, no division "
                  "- whatever root the counter runs on",
                  within(ms, ms_lo, ms_hi));

    // The arithmetic, exact: secs()/now() from one reading.
    const uint32_t t = Tb::ticks();
    TimeStamp stamp{};
    Tb::now(stamp);
    const uint32_t t_after = Tb::ticks();
    print(serial, "  ticks ", t, ": secs() ", Tb::secs(), ", now() ", stamp.seconds, ".",
          stamp.millis, crlf);
    bench.verdict("secs() is ticks >> 10 and now()'s fraction is the low ten bits in "
                  "milliseconds",
                  stamp.seconds == (t_after >> 10) &&
                      stamp.millis == static_cast<uint16_t>(((t_after & 1023u) * 1000u) >> 10) &&
                      Tb::secs() >= (t >> 10));

    // Monotonic under a tight read loop (the bracket read, the double
    // CNT read, the volatile laps): not one step backwards.
    uint32_t back = 0;
    uint32_t prev = Tb::ticks();
    for (uint32_t i = 0; i < 200'000u; ++i) {
        const uint32_t v = Tb::ticks();
        if (static_cast<int32_t>(v - prev) < 0) {
            ++back;
        }
        prev = v;
    }
    bench.verdict("two hundred thousand reads never step backwards", back == 0u);
    bench.verdict("the write handshake has never timed out", Tb::write_timeouts() == 0u);
}

// =============================================================================
// b - the lap carry, on LPTIM2 at the crystal's rate
// =============================================================================
void tb_lap_carry() {
    feed();
    console_drain();
    clear_counters();

    const bool up = Witness::init(clock);
    print(serial, "  the witness on LPTIM", 2, " at ", Witness::ticks_per_second,
          " ticks a second (shift 0): ", up ? "up" : "REFUSED", "; a lap is ",
          Witness::lap_counts, " counts = 2 s", crlf);
    bench.verdict("A SECOND LptimTicker COMES UP ON LPTIM2 - the instance whose "
                  "counter had never run on silicon",
                  up);
    if (!up) {
        return;
    }

    // Tight loop across at least two wraps: monotonic, and the laps
    // counted by the handler match the wraps the count implies.
    const uint32_t start = Witness::ticks();
    const uint32_t laps0 = Witness::laps();
    uint32_t back = 0;
    uint32_t prev = start;
    uint32_t reads = 0;
    while (static_cast<int32_t>(Witness::ticks() - start) < static_cast<int32_t>(5u * witness_hz)) {
        const uint32_t v = Witness::ticks();
        if (static_cast<int32_t>(v - prev) < 0) {
            ++back;
        }
        prev = v;
        ++reads;
        feed();
    }
    const uint32_t laps = Witness::laps() - laps0;
    print(serial, "  5 s on the witness: ", reads, " reads, ", back, " backwards, ", laps,
          " laps carried by the handler (", lptim2_irqs, " LPTIM2 interrupts)", crlf);
    bench.verdict("five seconds of tight reads across two wraps: monotonic, the "
                  "high word carried by the lap interrupt",
                  back == 0u && laps >= 2u && lptim2_irqs >= 2u);

    // PRIMASK HELD ACROSS A WRAP: the handler cannot run, ISR.ARRM stands,
    // and ticks() must add the lap itself (the pending-ARRM correction).
    // Wait until the wrap is 100 ms away, mask, spin 200 ms on TIM2
    // (feeding the watchdog), read under the mask, unmask, read again.
    while ((Witness::count() & 0xFFFFu) < 0x10000u - 3277u) {
        feed();
    }
    uint32_t masked_read = 0;
    uint32_t masked_lo = 0;
    bool arrm_stood = false;
    uint32_t laps_seen = 0;
    {
        InterruptGuard g;
        const uint32_t t0 = t2();
        while (t2() - t0 < (t2_hz / 5u)) {
            feed();
        }
        laps_seen = Witness::laps();
        arrm_stood = (L2::status() & LptimFlag::arrm) != 0u;
        masked_read = Witness::ticks();
        masked_lo = masked_read & 0xFFFFu;
    }
    const uint32_t after = Witness::ticks();
    const uint32_t laps_after = Witness::laps();
    const int32_t gap = static_cast<int32_t>(after - masked_read);
    print(serial, "  under a 200 ms mask across the wrap: ARRM stood=", arrm_stood,
          ", laps still ", laps_seen, " (", laps_after, " after unmask), the masked read's "
          "low half ", masked_lo, "; the unmasked read is ", gap, " counts later", crlf);
    bench.verdict("THE WRAP HAPPENED UNDER THE MASK and the handler had not run: ARRM "
                  "standing, laps not yet carried",
                  arrm_stood && laps_after == laps_seen + 1u);
    bench.verdict("and ticks() read under the mask ALREADY carried the lap - the "
                  "unmasked read follows it by microseconds, not by 65536",
                  gap >= 0 && gap < 200);

    // RULE 4's OPEN QUESTION: does CMPM fire at CMP == ARR (0xFFFF)? Put
    // the witness's compare there and watch a whole lap.
    clear_counters();
    const bool stored = L2::set_cmp(0xFFFFu) && L2::wait_cmp_ok();
    const uint32_t lap_start = Witness::ticks();
    while (static_cast<int32_t>(Witness::ticks() - lap_start) < static_cast<int32_t>(2u * witness_hz + 4000u)) {
        feed();
    }
    const uint32_t cmpm_at_arr = lptim_cmpm;
    print(serial, "  CMP = 0xFFFF = ARR for a lap and a bit: ", cmpm_at_arr,
          " CMPM interrupt(s) (", lptim2_irqs, " LPTIM2 interrupts in all)", crlf);
    if (cmpm_at_arr == 0u) {
        bench.verdict("A COMPARE EQUAL TO ARR NEVER MATCHES (0xFFFF would need refusing)",
                      stored);
    } else {
        bench.verdict("A COMPARE EQUAL TO ARR MATCHES like any other value - 0xFFFF needs "
                      "no special case (rule 4 as written)",
                      stored);
    }
    Nvic::disable(L2::irq());
    L2::init();
    L2::release();
}

// =============================================================================
// c - idle_until outside the kernel
// =============================================================================
void tc_idle_until() {
    feed();
    console_drain();

    static const uint32_t spans[] = {2, 3, 10, 100, 500};
    constexpr uint8_t n_spans = sizeof(spans) / sizeof(spans[0]);
    Nap naps[n_spans];
    for (uint8_t i = 0; i < n_spans; ++i) {
        feed();
        naps[i] = nap(std::optional<int32_t>{static_cast<int32_t>(spans[i])});
    }
    // TIM2 rides the PLL, i.e. HSI16, which is 0.2..0.3 % off the crystal
    // and drifts with the board's temperature: a nap is judged on the
    // wall's OWN scale - one tick weighed on TIM2 first (letter e's
    // lesson, applied here after a run that failed by 80 us of drift).
    console_drain();
    const uint32_t cal_k0 = Tb::ticks();
    while (Tb::ticks() == cal_k0) {
    }
    const uint32_t cal_t0 = t2();
    while (Tb::ticks() - cal_k0 < 257u) {
    }
    const uint32_t tick_ns = (((t2() - cal_t0) / 256u) * 1000u) / (t2_hz / 1'000'000u);   // ns per tick
    print(serial, "  one tick is ", tick_ns, " ns on TIM2's scale (976563 nominal)", crlf);
    bool all_ok = true;
    bool one_irq = true;
    bool never_early = true;
    uint32_t worst_over_us = 0;
    for (uint8_t i = 0; i < n_spans; ++i) {
        const Nap& r = naps[i];
        const uint32_t n = spans[i];
        const uint32_t nominal_us = (n * tick_ns) / 1000u;
        const uint32_t over = r.t2_us > nominal_us ? r.t2_us - nominal_us : 0u;
        if (over > worst_over_us) worst_over_us = over;
        print_nap("idle_until(now + N)", r);
        // THE LOWER BOUND SCALES WITH THE SPAN, because the nominal does
        // not come from a specification: `tick_ns` is the LSE measured
        // against TIM2 ON THE PLL, i.e. against HSI16's own 1 % trim, and
        // the residue of that ratio grows with the sleep. One per mille
        // plus the 40 us of quantisation is what the calibration can
        // claim; NEVER EARLY is judged separately below, in the
        // timebase's own units, where no scale enters at all.
        if (!within(r.t2_us, nominal_us - nominal_us / 1000u - 40u,
                    nominal_us + 1040u)) {
            all_ok = false;
        }
        // One CMPM ends the sleep; a lap in the way adds one ARRM and one round.
        if (r.irqs != r.rounds || (r.served & LptimFlag::cmpm) == 0u || r.rounds > 2u) one_irq = false;
        if (static_cast<int32_t>(r.ticks_after - r.asked) < 0) never_early = false;
        if (!r.interrupts_after) all_ok = false;
    }
    bench.verdict("N ticks asked, N ticks slept (within one count of the phase), "
                  "interrupts back on at return",
                  all_ok);
    bench.verdict("exactly ONE LPTIM interrupt per sleep and it served CMPM - no "
                  "spurious wake, no completion interrupt (a lap in the way costs one "
                  "ARRM and one more round, the loop's own shape)",
                  one_irq);
    bench.verdict("NEVER EARLY in the timebase's own units: ticks() >= the deadline at "
                  "every return",
                  never_early);
    bench.verdict("the wake lands within a count of the nominal (no phase conversion: "
                  "the compare is the same LSE edge the count is)",
                  worst_over_us < 1040u);

    // A DEADLINE ONE TICK AWAY, right after a tick edge: 32 counts of
    // room, well past the six-count floor - it sleeps and lands at the
    // tick. The floor itself (a deadline under six counts away) is a
    // matter of phase and is counted, not staged: floor_declines().
    const Nap one = nap(std::optional<int32_t>{1});
    print_nap("idle_until(now + 1)", one);
    bench.verdict("A DEADLINE ONE TICK AWAY sleeps to that tick: the compare is placed to "
                  "the count, 30 us, not to the tick",
                  one.ticks_after - one.ticks_before == 1u && within(one.t2_us, 700u, 1100u) &&
                      one.irqs == 1u);
    print(serial, "  arms declined for the six-count floor so far: ", Tb::floor_declines(),
          "; for a standing completion: ", Tb::deferrals(), crlf);

    // A due deadline: no sleep, no interrupt, microseconds.
    const Nap due = nap(std::optional<int32_t>{-5});
    print_nap("idle_until(now - 5)", due);
    bench.verdict("a deadline already due does not sleep at all", due.t2_us < 20u && due.irqs == 0u);

    // NO DEADLINE: the sleep ends on a foreign wake - the RTC's wake-up
    // timer 250 ms out - and the LPTIM stays silent (a lap inside the
    // window would speak, once, for the carry: the window is placed in
    // the first half of a lap).
    Rtc::clear_wakeup();
    lap_room();
    const bool wut = Rtc::set_wakeup(RtcWakeupClock::div16, 512u);   // (512 + 1) / 2048 s
    const Nap none = nap(std::nullopt);
    Rtc::clear_wakeup();
    print(serial, "  (the wake-up timer was ", wut ? "armed" : "REFUSED", ")", crlf);
    print_nap("idle_until(nothing armed), RTC 250 ms out", none);
    // The wake-up timer's first period is shortened by its asynchronous
    // start (the sleep suite's own band for this setting is 230..290 ms).
    bench.verdict("with nothing armed the sleep lasts until a FOREIGN wake - the RTC "
                  "wake-up timer ended it, the LPTIM never spoke",
                  within(none.t2_us, 220'000u, 290'000u) && none.rtc == 1u && none.irqs == 0u);
}

// =============================================================================
// d - the handshake, and the Stop question
// =============================================================================
void td_handshake() {
    feed();
    console_drain();
    lap_room();

    // TWO ARMS BACK TO BACK with different deadlines. The first stores;
    // the second finds CMPOK standing (the first landed and no LPTIM
    // interrupt has run) and is DEFERRED: the vector pended, no sleep.
    // The handler sweeps the flag; the third arm stores the second value.
    // (The first arm must find the flag clear: a sweep is forced first.)
    Nvic::set_pending(Tb::irq());
    spin_us(50);
    clear_counters();
    const uint32_t d0 = Tb::deferrals();
    const uint32_t s0 = Tb::stores();
    const uint32_t now0 = Tb::ticks();
    bool first = false, second = false, third = false;
    uint32_t sweeps_between = 0;
    {
        InterruptGuard g;
        first = Tb::arm_wake(now0, now0 + 300u);
        spin_us(150);                       // let the first store land
        second = Tb::arm_wake(now0, now0 + 400u);
    }
    // Interrupts back: the pended handler runs and sweeps CMPOK - and
    // the clear takes its time to cross into the kernel clock domain.
    const uint32_t x0 = t2();
    uint32_t guard = 4'000'000u;
    while (L1::cmp_ok() && guard-- != 0u) {
    }
    const uint32_t crossing_us = t2_us(t2() - x0);
    sweeps_between = lptim_sweeps;
    {
        InterruptGuard g;
        third = Tb::arm_wake(now0, now0 + 400u);
    }
    const uint16_t reg = L1::cmp();
    const uint32_t deferrals = Tb::deferrals() - d0;
    const uint32_t stores = Tb::stores() - s0;
    const uint16_t want = static_cast<uint16_t>(((Tb::count() & ~31u) + 0u) & 0xFFFFu);
    (void)want;
    print(serial, "  arm(now+300) -> ", first, ", arm(now+400) 150 us later -> ", second,
          " (deferred: ", deferrals, ", handler sweeps: ", sweeps_between, ", the clear crossed "
          "in ", crossing_us, " us), arm(now+400) again -> ", third, "; CMP reads 0x", hex(reg),
          ", stores ", stores, crlf);
    bench.verdict("THE SECOND STORE WAITS FOR THE HANDLER: the arm that finds CMPOK "
                  "standing pends the vector and declines the sleep, exactly once",
                  first && !second && deferrals == 1u && sweeps_between >= 1u);
    bench.verdict("and the next arm stores the new value - two stores, one completion "
                  "between them, 26.4.11 kept",
                  third && stores == 2u && reg == Tb::cmp_reg());
    // Let that compare fire and clear the way.
    while (static_cast<int32_t>(Tb::ticks() - (now0 + 400u)) < 0) {
        feed();
    }

    // RULE 3 AS A FINDING. A compare stored IMMEDIATELY before a Stop 1:
    // with the wait (the ticker's own rule, SLEEPDEEP set means
    // wait_cmp_ok before the WFI) and WITHOUT it (the raw sequence, to
    // see whether an in-flight APB->kernel transfer completes with PCLK
    // stopped). Judged on the RTC wall. The second leg goes through
    // Pwr and Lptim directly - the ticker would wait.
    console_drain();
    lap_room();
    clear_counters();
    sync_to_count();
    const uint32_t k0 = Tb::ticks();
    const uint32_t w0 = wall();
    (void)Pwr::arm(PwrMode::stop1);
    {
        InterruptGuard g;
        P::idle_until(std::optional<uint32_t>{k0 + 300u});
    }
    const uint32_t w1 = wall();
    const uint32_t k1 = Tb::ticks();
    const bool back_hsisys = Rcc::sysclk_status() == SysclkSource::hsisys;
    (void)Pwr::arm(PwrMode::sleep);
    (void)Site::resume_clock();
    const uint32_t waited = Tb::stop_waits();
    print(serial, "  a 300-tick deadline through a Stop 1 with the wait: ",
          wall_ms(wall_delta(w0, w1)), " ms of RTC wall, ticks moved ", k1 - k0,
          ", stop waits so far ", waited, ", ", lptim_irqs, " LPTIM interrupt(s); SYSCLK "
          "came back as HSISYS=", back_hsisys, crlf);
    bench.verdict("THROUGH A STOP 1: the compare waited into place fires at the deadline "
                  "on the RTC's wall, kernel time RUNS through the Stop, one interrupt",
                  within(wall_ms(wall_delta(w0, w1)), 290u, 320u) && within(k1 - k0, 300u, 302u) &&
                      lptim_irqs == 1u && waited >= 1u);
    bench.verdict("and the Stop was a Stop: SYSCLK came back on HSISYS and the site's "
                  "resume_clock() restored the PLL",
                  back_hsisys && Rcc::sysclk_status() == SysclkSource::pllrclk);

    // THE RAW LEG: store, no wait, straight into the Stop. If the
    // transfer needs PCLK the compare never lands and the RTC backstop
    // (a 2 s wake-up) ends the sleep; if it lands, the match ends it at
    // 300 counts. Either answer is a finding, not a verdict on the code.
    console_drain();
    clear_counters();
    Rtc::clear_wakeup();
    (void)Rtc::set_wakeup(RtcWakeupClock::div16, 4095u);   // 2 s backstop
    // A fresh store needs CMPOK clear: force a sweep first.
    Nvic::set_pending(Tb::irq());
    spin_us(400);
    lap_room();
    sync_to_count();
    // The compare in the ticker's own arithmetic: the count where the
    // tick becomes now + 300, less one (CMPM fires at the edge after).
    const uint32_t cr = Tb::count();
    const uint16_t raw_cmp =
        static_cast<uint16_t>(((cr & ~(Tb::counts_per_tick - 1u)) + 300u * Tb::counts_per_tick) - 1u);
    const uint32_t wr0 = wall();
    (void)Pwr::arm(PwrMode::stop1);
    {
        InterruptGuard g;
        (void)L1::set_cmp(raw_cmp);
        __DSB();
        __WFI();
        __enable_irq();
    }
    const uint32_t wr1 = wall();
    const bool raw_hsisys = Rcc::sysclk_status() == SysclkSource::hsisys;
    (void)Pwr::arm(PwrMode::sleep);
    (void)Site::resume_clock();
    Rtc::clear_wakeup();
    const uint32_t raw_ms = wall_ms(wall_delta(wr0, wr1));
    const bool landed = lptim_cmpm >= 1u && within(raw_ms, 290u, 320u);
    print(serial, "  the same store with NO wait, straight into the Stop: ", raw_ms,
          " ms of wall, ", lptim_irqs, " LPTIM interrupt(s) (", lptim_cmpm, " CMPM, ", lptim_arrm,
          " ARRM), RTC backstop wakes ", rtc_wakes, ", Stop was real=", raw_hsisys, crlf);
    if (landed) {
        bench.verdict("FINDING: an in-flight compare write LANDS with PCLK stopped - the "
                      "APB-to-kernel transfer completes on the kernel clock alone; rule "
                      "3's wait is insurance the silicon does not need (kept: 93 us per "
                      "Stop round, and the chapter promises nothing)",
                      raw_hsisys);
    } else {
        bench.verdict("FINDING: an in-flight compare write DOES NOT land with PCLK "
                      "stopped - the deadline came and went unmatched and something else "
                      "(the lap, or the backstop) ended the sleep; rule 3's wait is what "
                      "makes a deadline through a Stop real",
                      raw_hsisys && lptim_cmpm == 0u);
    }
    // Whatever happened, the ticker's mirror no longer matches the
    // register: re-park through the ticker's own path.
    Nvic::set_pending(Tb::irq());
    spin_us(50);
}

// =============================================================================
// e - a real kernel
// =============================================================================
void te_kernel() {
    feed();
    console_drain();
    Metronome::clear();
    MetroKernel::init_all();
    clear_counters();
    const uint32_t s0 = systick_irqs;

    sync_to_count();
    Metronome::fast.arm_every(7);
    Metronome::mid.arm_every(30);
    Metronome::slow.arm_every(250);
    const uint32_t t0 = t2();
    const uint32_t k0 = Tb::ticks();
    uint32_t turns = 0;
    uint32_t sleeps = 0;
    while (t2() - t0 < 3u * t2_hz) {
        feed();
        TimeEvents<P>::process();
        if (!MetroKernel::step()) {
            const uint32_t before = lptim_irqs;
            MetroKernel::idle_if_empty();
            if (lptim_irqs != before) ++sleeps;
        }
        ++turns;
    }
    Metronome::fast.disarm();
    Metronome::mid.disarm();
    Metronome::slow.disarm();
    const uint32_t kticks = Tb::ticks() - k0;
    const uint32_t window_us = t2_us(t2() - t0);
    // TIM2 rides the PLL (HSI16 x 4) and the timebase the crystal: the
    // wall's scale is what the window measured, not the nominal.
    const uint32_t tick_us_x1000 = (window_us * 1000u) / kticks;   // ~976.5 x 1000, on TIM2
    const uint32_t irqs = lptim_irqs;
    const uint32_t cmpms = lptim_cmpm;
    const uint32_t deferrals = Tb::deferrals();
    const Track& f = Metronome::tf;
    const Track& m = Metronome::tm;
    const Track& s = Metronome::ts;
    print(serial, "  3 s of kernel (", kticks, " ticks, ", turns, " turns, ", irqs,
          " LPTIM interrupts of which ", cmpms, " CMPM, ", lptim_arrm, " ARRM; ", sleeps,
          " sleeps ended by the LPTIM; deferrals so far ", deferrals, ")", crlf);
    const uint32_t f_nom = (7u * tick_us_x1000) / 1000u;
    const uint32_t m_nom = (30u * tick_us_x1000) / 1000u;
    const uint32_t s_nom = (250u * tick_us_x1000) / 1000u;
    print(serial, "  one tick is ", tick_us_x1000, " ns on TIM2's scale (976563 nominal: the "
          "PLL against the crystal)", crlf);
    print(serial, "  fast  7 ticks: ", f.fired, " firings, period ", f.min_us, "..", f.max_us,
          " us (", f_nom, " on that scale), in ticks ", f.min_ticks, "..", f.max_ticks,
          ", early ", f.early, crlf);
    print(serial, "  mid  30 ticks: ", m.fired, " firings, period ", m.min_us, "..", m.max_us,
          " us (", m_nom, "), in ticks ", m.min_ticks, "..", m.max_ticks, ", early ", m.early,
          crlf);
    print(serial, "  slow 250 ticks: ", s.fired, " firings, period ", s.min_us, "..", s.max_us,
          " us (", s_nom, "), in ticks ", s.min_ticks, "..", s.max_ticks, ", early ", s.early,
          crlf);
    // HOW MANY FIRINGS ARE DUE IS THE WINDOW'S OWN ARITHMETIC and not a
    // constant: the window is three seconds OF THE WALL, and how many
    // kernel ticks that is depends on the root the timebase runs on -
    // 3072 on a crystal, about 2950 on an LSI whose tick is the four per
    // cent long its statement makes it. So the count due is the ticks
    // the window really carried, divided by each cadence.
    bench.verdict("THREE PERIODICS THROUGH A TICKLESS KERNEL: every firing on its cadence "
                  "within a count on the wall, drift-free from the first",
                  within(f.fired, kticks / 7u - 6u, kticks / 7u + 6u) &&
                      within(m.fired, kticks / 30u - 2u, kticks / 30u + 2u) &&
                      within(s.fired, kticks / 250u, kticks / 250u + 1u) &&
                      within(f.max_us, f_nom - 40u, f_nom + 1100u) &&
                      within(m.max_us, m_nom - 40u, m_nom + 1100u) &&
                      within(s.max_us, s_nom - 40u, s_nom + 1100u));
    bench.verdict("NOT ONE EARLY, in three thousand milliseconds of three cadences - judged "
                  "in the timebase's own ticks",
                  f.early == 0u && m.early == 0u && s.early == 0u && f.max_ticks <= 8u &&
                      m.max_ticks <= 31u && s.max_ticks <= 251u);
    // Distinct deadline instants over 3 s: fast ~439, mid ~102, slow ~12,
    // minus the coincidences (every 210 ticks fast and mid meet, every 1750
    // fast and slow) - the LPTIM wakes should number about that and no more.
    // A deadline on the counter's own wrap serves CMPM and ARRM in ONE
    // interrupt (letter b: a compare at ARR matches), so the flags may
    // overlap; what may not happen is an interrupt that served nothing.
    bench.verdict("about one LPTIM wake per distinct deadline instant and none for "
                  "anything else - the kernel slept TO its deadlines",
                  within(irqs, 480u, 560u) && cmpms <= irqs &&
                      cmpms + lptim_arrm + lptim_sweeps >= irqs);
    bench.verdict("and the SysTick handler still never ran", systick_irqs == s0);
}

// =============================================================================
// f - Stop 1 through the manager, with the plain site
// =============================================================================
void tf_stop_through_manager() {
    feed();
    console_drain();
    K::init_all();
    Probe::clear();
    clear_counters();
    kernel_live = true;

    const bool dbg_stop = Pwr::debug_in_stop();
    sync_to_count();
    const uint32_t w0 = wall();
    const uint32_t k0 = Tb::ticks();
    Probe::deadline.arm(500u);
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip(3000u);
    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ticks = Probe::blip_ticks - k0;
    const bool pll_back = Rcc::sysclk_status() == SysclkSource::pllrclk;
    kernel_live = false;
    print(serial, "  a 500-tick deadline through a Stop 1 under the manager: matured after ",
          to_blip, " ms of RTC wall, kernel ticks elapsed ", kernel_ticks, ", ", lptim_irqs,
          " LPTIM interrupt(s), stop waits ", Tb::stop_waits(), ", votes ", Probe::votes,
          " wakes ", Probe::wakes, " (DBGMCU_CR.DBG_STOP = ", dbg_stop, ")", crlf);
    bench.verdict("THE PLAIN SITE MEETS A DEADLINE THROUGH A STOP: no timed site, no "
                  "resync - kernel time ran through the Stop on the LPTIM",
                  Probe::blips == 1u && within(to_blip, 488u, 520u) && within(kernel_ticks, 500u, 502u));
    bench.verdict("never early", to_blip >= 488u && kernel_ticks >= 500u);
    // A lap inside the 500 ticks (one in four) adds one ARRM wake, which
    // the loop turns through: the compare's own is the one that ends it.
    bench.verdict("the compare's own interrupt ended the Stop (plus one per lap wrapped "
                  "inside the window, none otherwise)",
                  lptim_irqs == 1u + lptim_arrm && lptim_cmpm >= 1u);
    bench.verdict("the round closed by the convention and SYSCLK is back on the PLL",
                  Probe::wakes >= 1u && Site::armed() == SleepDepth::none && pll_back);

    // Six shorter rounds, never early.
    constexpr uint16_t repeats = 6;
    constexpr uint32_t nominal = 150;
    uint16_t on_time = 0;
    uint32_t worst_lo = 0xFFFFFFFFu;
    uint32_t worst_hi = 0;
    for (uint16_t i = 0; i < repeats; ++i) {
        feed();
        K::init_all();
        Probe::clear();
        kernel_live = true;
        console_drain();
        sync_to_count();
        const uint32_t a = wall();
        Probe::deadline.arm(nominal);
        post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
        pump_until_blip(1000u);
        kernel_live = false;
        if (Probe::blips != 1u) {
            continue;
        }
        const uint32_t ms = wall_ms(wall_delta(a, Probe::blip_wall));
        if (ms < worst_lo) worst_lo = ms;
        if (ms > worst_hi) worst_hi = ms;
        // 150 ticks = 146.5 ms.
        if (within(ms, 146u, 170u)) ++on_time;
    }
    print(serial, "  ", repeats, " rounds of ", nominal, " ticks (146.5 ms) through a Stop: ",
          worst_lo, "..", worst_hi, " ms of wall", crlf);
    bench.verdict("six shorter rounds all inside the band", on_time == repeats);
    bench.verdict("and not one early", worst_lo >= 146u);
    (void)Site::resume_clock();
}

// =============================================================================
// g - delay_us on the interrupt-less SysTick
// =============================================================================
void tg_delay() {
    feed();
    console_drain();
    const uint32_t s0 = systick_irqs;

    static const uint32_t spans[] = {5, 30, 100, 500, 900};
    bool all_exact = true;
    for (uint8_t i = 0; i < sizeof(spans) / sizeof(spans[0]); ++i) {
        const uint32_t t0 = t2();
        const bool ok = delay_us(clock, spans[i]);
        const uint32_t took = t2_us(t2() - t0);
        print(serial, "  delay_us(", spans[i], ") -> ", took, " us on TIM2", crlf);
        if (!ok || took < spans[i] || took > spans[i] + 12u) {
            all_exact = false;
        }
    }
    bench.verdict("delay_us serves 5..900 us AT LEAST on a SysTick with no interrupt",
                  all_exact);

    uint32_t min_took = 0xFFFFFFFFu;
    for (uint16_t k = 0; k < 200; ++k) {
        const uint32_t t0 = t2();
        (void)delay_us(clock, 50u);
        const uint32_t took = t2_us(t2() - t0);
        if (took < min_took) min_took = took;
    }
    const bool refused = !delay_us(clock, 1000u);
    const bool served = delay_us(clock, 999u);
    print(serial, "  200 x delay_us(50): min ", min_took, " us; delay_us(1000) refused=",
          refused, ", delay_us(999) served=", served, crlf);
    bench.verdict("two hundred 50 us waits, not one early", min_took >= 50u);
    bench.verdict("the cap is a SysTick period: 1000 us refused, 999 served - a "
                  "millisecond, which on this timebase is 1.024 ticks",
                  refused && served);
    bench.verdict("and SysTick_Handler still never ran", systick_irqs == s0);
}

// =============================================================================
// u - the keystroke (outside z)
// =============================================================================
void tu_keystroke() {
    feed();
    print(serial, "  idle_until() with nothing armed for up to 20 s: press a key to end "
                  "it (the IWDG is fed by the wake-up timer every second)", crlf);
    console_drain();
    Rtc::clear_wakeup();
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 0u);   // every second: a feed
    clear_counters();
    const uint32_t t0 = wall();
    uint32_t rounds = 0;
    uint8_t key = 0;
    bool got = false;
    while (rounds < 20u) {
        {
            InterruptGuard g;
            P::idle_until(std::nullopt);
        }
        feed();
        ++rounds;
        if (Serial::read_byte(key)) {
            got = true;
            break;
        }
    }
    Rtc::clear_wakeup();
    const uint32_t ms = wall_ms(wall_delta(t0, wall()));
    print(serial, "  ended after ", ms, " ms and ", rounds, " wake(s); key=",
          got ? static_cast<char>(key) : '-', ", LPTIM interrupts ", lptim_irqs, crlf);
    bench.verdict("a UART byte is a foreign wake of a deadline-less idle_until", got);
}

// =============================================================================
// h - an LSI-clocked ticker as a witness
// =============================================================================
void th_lsi_witness() {
    feed();
    console_drain();
    clear_counters();
    witness_mode = 1;
    const bool up = LsiWitness::init(clock);
    print(serial, "  LptimTicker on LPTIM2 from LSI, stated ", lsi_measured_hz, " Hz, shift 5: ",
          up ? "up" : "REFUSED", " - ", LsiWitness::ticks_per_second,
          " ticks a second by the statement", crlf);
    bench.verdict("AN LSI-CLOCKED LptimTicker COMES UP: RCC's own oscillator, no RTC "
                  "domain gate in the way",
                  up);
    if (!up) {
        witness_mode = 0;
        return;
    }
    // Two seconds of the crystal timebase: the LSI witness's ticks and
    // millis() against the statement (LSI wanders 1..2 % between runs;
    // the band says so).
    // TWO SECONDS OF TIM2 AND NOT OF THE KERNEL TICK: on a board whose
    // timebase is itself an LSI, a window measured in kernel ticks is a
    // window whose LENGTH carries the timebase's own error, and what
    // this leg weighs is the WITNESS. TIM2 rides the PLL, i.e. HSI16's
    // per cent, which is well inside the band below.
    const uint32_t t0 = t2();
    const uint32_t w0 = LsiWitness::ticks();
    const uint32_t m0 = LsiWitness::millis();
    while (t2() - t0 < 2u * t2_hz) {
        feed();
    }
    const uint32_t w_ticks = LsiWitness::ticks() - w0;
    const uint32_t w_ms = LsiWitness::millis() - m0;
    const uint32_t expected = 2u * LsiWitness::ticks_per_second;
    print(serial, "  2 s of TIM2: the LSI witness counted ", w_ticks, " ticks (",
          expected, " by the statement) and ", w_ms, " ms", crlf);
    bench.verdict("its ticks and millis() follow the STATED rate within 3 % of a "
                  "wall that is not its own - the arithmetic is exact, the "
                  "number is the oscillator's",
                  within(w_ticks, expected - expected / 33u, expected + expected / 33u) &&
                      within(w_ms, 1940u, 2060u));

    // A wake placed on it: arm_wake and a WFI by hand, judged in ITS
    // units (never early) and on the crystal (the statement's error).
    clear_counters();
    const uint32_t before = LsiWitness::ticks();
    const uint32_t asked = before + 100u;
    const uint32_t c0 = t2();
    bool armed = false;
    uint32_t rounds = 0;
    for (;;) {
        InterruptGuard g;
        const uint32_t now = LsiWitness::ticks();
        if (static_cast<int32_t>(asked - now) <= 0) {
            break;
        }
        armed = LsiWitness::arm_wake(now, asked);
        if (armed) {
            P::idle();
        } else {
            enable_interrupts();
        }
        ++rounds;
        if (rounds > 2000u) break;
    }
    const uint32_t took_us = t2_us(t2() - c0);
    const uint32_t landed = LsiWitness::ticks();
    print(serial, "  a 100-tick wake on the LSI witness: slept ", took_us, " us in ", rounds,
          " round(s), ticks ", before, " -> ", landed, " (asked ", asked, "), ", lptim2_irqs,
          " LPTIM2 interrupt(s)", crlf);
    bench.verdict("the compare placed on the LSI counter wakes at its tick, never early in "
                  "the witness's own units, one interrupt",
                  armed && static_cast<int32_t>(landed - asked) >= 0 &&
                      static_cast<int32_t>(landed - asked) <= 1 && lptim2_irqs >= 1u &&
                      within(took_us, 95'000u, 105'000u));

    // THE DIRECTIONAL RULE'S PRICE: the default statement is table 46's
    // ceiling, 34000 - never early on any part, and this many late on
    // this one.
    const uint32_t late_per_mille = (34'000u * 1000u) / lsi_measured_hz - 1000u;
    print(serial, "  the default statement (34000 Hz) on this die's ", lsi_measured_hz,
          " Hz: every millisecond asked lands ", late_per_mille,
          " per mille late, never early (", LsiDefault::ticks_per_second,
          " ticks a second would be counted at ", lsi_measured_hz >> 5, ")", crlf);
    // THE BAND IS TABLE 46'S OWN, because the number is a DIE fact and
    // the dies differ: the ceiling over the band's floor is 34000/29500,
    // 153 per mille, and any part of this family lands under it. What is
    // claimed is the DIRECTION and the bound, not one die's figure.
    bench.verdict("the default over-states the rate, so a program that does not measure "
                  "its LSI is late and never early - by at most what table 46's own "
                  "ceiling over its own floor allows",
                  late_per_mille > 0u && late_per_mille < 160u);

    Nvic::disable(L2::irq());
    L2::init();
    L2::release();
    witness_mode = 0;
    // SysTick was restarted by the witness's init (the same reload):
    // the timebase's own is put back for good measure.
    (void)SysTickCounter::start(clock);
}

// =============================================================================
// i - the lap wakes of a long Stop
// =============================================================================
// THE AWAKE-TIME METER IS TIM2 ITSELF, re-clocked for this letter from
// MCO = HSI16/8 through its ETR (TIM2 is the ONE timer whose ETRSEL
// names the MCOs and LSE; TIM3/TIM4's name the comparators only -
// RM0444 22.4.26..27): a 2 MHz count that runs only while HSI16 runs
// and the APB is clocked, i.e. only awake. The PLL wall is put back at
// the end.
constexpr uint32_t awake_hz = 16'000'000u / 8u;
/// WHETHER THE METER CAN EXIST AT ALL. RM0444 22.4.25's ETRSEL list
/// footnotes 0100 (MCO), 0101 (MCO2) and 0110 (COMP3) "available on
/// STM32G0B1xx and STM32G0C1xx sales types only"; on every smaller part
/// the code selects nothing and TIM2 does not count. THE PRICE OF
/// GETTING THAT WRONG IS THE WHOLE SUITE: `spin_us()` above rides the
/// same TIM2, so a meter that does not count is a wait that never ends
/// (measured - the board sat in `spin_us` until the watchdog reset it).
/// So the meter is built only where the reserve says the code exists,
/// and TIM2 is left on its PLL wall everywhere else.
constexpr bool meter_possible = tim_etrsel_has_mco(2);
volatile uint32_t arrm_t2[8];                       ///< the meter at each ARRM entry
volatile uint8_t arrm_seen = 0;
volatile uint8_t arrm_on_hsisys = 0;
volatile bool lap_watch = false;

bool awake_meter_up() {
    if (!Rcc::mco(Rcc::mco_hsi16_code, 3)) {
        return false;
    }
    T2::enable(false);
    T2::reset();
    if (!T2::configure({.prescaler = 0, .period = 0xFFFFFFFFu})) {
        return false;
    }
    if (!T2::external_trigger_select(4) || !T2::external_trigger({.clock_mode2 = true})) {
        return false;
    }
    T2::enable(true);
    return true;
}

void ti_lap_wakes() {
    feed();
    console_drain();
    bool meter = false;
    if constexpr (meter_possible) {
        meter = awake_meter_up();
        bench.verdict("TIM2 counts MCO = HSI16/8 through its ETR (the awake-time meter: "
                      "HSI16 stops in a Stop and so does the timer's bus)",
                      meter);
    } else {
        print(serial,
              "  SKIPPED, no verdict claimed: the awake-time meter is TIM2's "
              "ETR taking MCO, and RM0444 22.4.25's ETRSEL list gives code "
              "0100 to the G0B1/G0C1 sales types alone "
              "(tim_etrsel_has_mco(2) is false here). TIM2 stays on its PLL "
              "wall - which this suite's own spin_us() also rides - and the "
              "awake-time verdict below is the only one that goes with it; "
              "the lap count and the PLL claim do not need a meter.",
              crlf);
    }
    console_drain();   // a Stop under a line in flight is a garbled line (measured)

    K::init_all();
    Probe::clear();
    clear_counters();
    arrm_seen = 0;
    arrm_on_hsisys = 0;
    lap_watch = true;
    kernel_live = true;
    sync_to_count();
    const uint32_t w0 = wall();
    const uint32_t k0 = Tb::ticks();
    const uint32_t m0 = t2();
    Probe::deadline.arm(10240u);    // ten seconds: five laps
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip(12000u);
    kernel_live = false;
    lap_watch = false;
    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ticks = Probe::blip_ticks - k0;
    const uint32_t m1 = t2();
    const bool pll_back = Rcc::sysclk_status() == SysclkSource::pllrclk;

    // The awake time of one lap wake: the meter between consecutive
    // ARRM entries (nothing else runs in between: the ISR, the loop's
    // empty turn, the WFI, the Stop, the next wake's latency up to the
    // ISR).
    uint32_t per_wake_min = 0xFFFFFFFFu, per_wake_max = 0;
    for (uint8_t i = 1; i < arrm_seen && i < 8u; ++i) {
        const uint32_t d = arrm_t2[i] - arrm_t2[i - 1u];
        const uint32_t us = (d * 1000u) / (awake_hz / 1000u);
        if (us < per_wake_min) per_wake_min = us;
        if (us > per_wake_max) per_wake_max = us;
    }
    const uint32_t total_awake_us = ((m1 - m0) * 1000u) / (awake_hz / 1000u);
    print(serial, "  10 s in Stop 1 with one deadline: matured after ", to_blip,
          " ms of RTC wall, kernel ticks ", kernel_ticks, "; ", lptim_irqs,
          " LPTIM interrupts = ", lptim_arrm, " ARRM + ", lptim_cmpm, " CMPM; ", arrm_on_hsisys,
          " of ", arrm_seen, " ARRM entries found SYSCLK on HSISYS; awake per lap wake ",
          per_wake_min, "..", per_wake_max, " us (ISR to ISR on the meter), ", total_awake_us,
          " us awake in the whole round; PLL back at the end=", pll_back, crlf);
    bench.verdict("A TEN-SECOND STOP COSTS FIVE LAP WAKES AND ONE FOR THE DEADLINE: six "
                  "LPTIM interrupts in all - the parked compare's CMPM rides the ARRM's "
                  "own interrupt (the first version parked it mid-lap: ten), the deadline "
                  "met",
                  Probe::blips == 1u && lptim_arrm == 5u && lptim_irqs == 6u &&
                      within(to_blip, wall_ms_for_kernel_ms(9990u),
                             wall_ms_for_kernel_ms(10'100u)) &&
                      within(kernel_ticks, 10240u, 10242u));
    bench.verdict("THE PLL IS NEVER RE-LOCKED FOR A LAP WAKE: every ARRM ran on HSISYS and "
                  "the loop went straight back to sleep (no AO, no manager round)",
                  arrm_seen == 5u && arrm_on_hsisys == 5u);
    if (meter) {
        bench.verdict("a lap wake keeps the part awake for under 300 us (ISR to ISR on a "
                      "clock that only runs awake: the Stop exit, the handler, one empty "
                      "loop turn and the WFI, all at 16 MHz with the PLL's two wait states "
                      "still in FLASH_ACR) - a 10^-4 duty for a rare-event program",
                      per_wake_max < 300u);
    } else {
        print(serial, "  SKIPPED, no verdict claimed: what a lap wake COSTS "
              "needs a clock that runs only while the part is awake, which "
              "on this part the meter above could not build.", crlf);
    }
    bench.verdict("and the round closed with the PLL back", pll_back);
    // The PLL wall back for whatever runs next.
    if (meter) {
        T2::enable(false);
        T2::reset();
        Rcc::mco_off();
        (void)t2_up();
    }
}

// =============================================================================
// x - the compare's own latencies at the ticker's prescaler (diagnostic)
// =============================================================================
void tx_latency() {
    feed();
    console_drain();
    Nvic::set_pending(Tb::irq());
    spin_us(400);

    // 1. CMP write to CMPOK on the undivided counter, five times.
    print(serial, "  CMPOK latency on the undivided counter (32768 Hz):", crlf);
    for (uint8_t i = 0; i < 5u; ++i) {
        const uint32_t before = L1::status();
        const uint16_t far = static_cast<uint16_t>(Tb::count() + 30000u);
        const uint32_t c0 = t2();
        (void)L1::set_cmp(far);
        uint32_t guard = 20'000'000u;
        while (!L1::cmp_ok() && guard-- != 0u) {
        }
        const uint32_t us = t2_us(t2() - c0);
        print(serial, "    ISR before 0x", hex(before), ": store -> CMPOK in ", us, " us (",
              (us * 32768u + 500'000u) / 1'000'000u, " counts)", crlf);
        Nvic::set_pending(Tb::irq());
        spin_us(400);
    }

    // 2. The match: CMP = count + k stored right after a count edge.
    print(serial, "  the match, CMP = count + k stored right after a count edge:", crlf);
    static const uint32_t ks[] = {2, 3, 4, 5, 6, 10, 40};
    for (uint8_t i = 0; i < sizeof(ks) / sizeof(ks[0]); ++i) {
        feed();
        console_drain();
        Nvic::set_pending(Tb::irq());
        spin_us(400);
        clear_counters();
        const uint32_t c_sync = Tb::count();
        while (Tb::count() == c_sync) {
        }
        const uint32_t now = Tb::count();
        const uint16_t cmp = static_cast<uint16_t>(now + ks[i]);
        const uint32_t c0 = t2();
        (void)L1::set_cmp(cmp);
        uint32_t guard = 200'000'000u;
        while (lptim_irqs == 0u && guard-- != 0u) {
            feed();
        }
        const uint32_t us = t2_us(lptim_t2_at - c0);
        print(serial, "    k=", ks[i], ": CMP ", cmp, " stored at CNT ", now & 0xFFFFu,
              " -> irq at CNT ", lptim_cnt_at, " after ", us, " us (",
              (us * 32768u + 500'000u) / 1'000'000u, " counts), ISR at entry 0x",
              hex(lptim_isr_at), " served 0x", hex(lptim_last_served), crlf);
    }
    Nvic::set_pending(Tb::irq());
    spin_us(400);
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_stm32_tickless - the LPTIM kernel timebase and idle_until "
          "(no wires)", crlf);
    bench.menu();
    print(serial, "  z  run them all (a..g)", crlf);
}

}   // namespace

/// Bound ON PURPOSE, to prove it never runs.
extern "C" void SysTick_Handler() { systick_irqs = systick_irqs + 1u; }

extern "C" void BRIO_STM32G0_USART2_HANDLER() {
    usart_irqs = usart_irqs + 1u;
    (void)Serial::isr();
}

/// LPTIM1's vector - shared with TIM6 and the DAC on the parts that have
/// them, LPTIM1's own where they are absent, and the reserve's macro is
/// what names it either way: the timebase's body, plus the counters the
/// letters read. A raw G0B1 name here would leave the tick's interrupt
/// bound to nothing on a smaller part, which is the Default_Handler spin
/// the samc21 stratum learned to fear - and this suite MEASURED it: the
/// G031's first run answered nothing at all, not even a banner.
extern "C" void BRIO_STM32G0_LPTIM1_HANDLER() {
    lptim_t2_at = t2();
    lptim_cnt_at = brio::Lptim<1>::count_raw();
    lptim_isr_at = brio::Lptim<1>::status();
    const uint32_t served = Tb::isr();
    lptim_irqs = lptim_irqs + 1u;
    lptim_last_served = served;
    if ((served & brio::LptimFlag::cmpm) != 0u) lptim_cmpm = lptim_cmpm + 1u;
    if ((served & brio::LptimFlag::arrm) != 0u) {
        lptim_arrm = lptim_arrm + 1u;
        if (lap_watch && arrm_seen < 8u) {
            arrm_t2[arrm_seen] = T2::count();
            if (brio::Rcc::sysclk_status() == brio::SysclkSource::hsisys) {
                arrm_on_hsisys = arrm_on_hsisys + 1u;
            }
            arrm_seen = arrm_seen + 1u;
        }
    }
    if (served == 0u) lptim_sweeps = lptim_sweeps + 1u;
}

/// LPTIM2's vector - shared with TIM7 on the parts that have one, its
/// own where they have not, and the reserve's macro is what names it:
/// the witness's body, the lap one of letter b or the LSI one of h.
extern "C" void BRIO_STM32G0_LPTIM2_HANDLER() {
    const uint32_t served = witness_mode == 1u ? LsiWitness::isr() : Witness::isr();
    lptim2_irqs = lptim2_irqs + 1u;
    if ((served & brio::LptimFlag::cmpm) != 0u) lptim_cmpm = lptim_cmpm + 1u;
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

    // The RTC domain is opened as it stands and never reset (RTCSEL is
    // one-way, and a BDRST would cost the backup registers): a domain
    // already on either 32 kHz root is kept, and a domain nobody has
    // selected yet - no backup battery on a Nucleo, so a USB unplug
    // empties it - is claimed for the crystal if it starts and for LSI
    // if it does not. An LSI wall is not a nominal, so its rate is
    // MEASURED before anything is timed on it.
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
    const bool wall2_ok = t2_up();
    // THE TIMEBASE: the LPTIM on the board's own 32 kHz root, and
    // SysTick interrupt-less.
    const bool tick_ok = Tb::init(clock);
    const bool wd = brio::Iwdg::arm(brio::IwdgConfig{
        .prescaler = brio::IwdgPrescaler::div256,
        .reload = 0x0FFF,
        .window = 0x0FFF});
    brio::enable_interrupts();

    bench.letter('a', "THE TIMEBASE: SysTick silent, the LPTIM's rate, the arithmetic",
                 ta_timebase);
    bench.letter('b', "the lap carry on LPTIM2, masked across a wrap; CMP == ARR",
                 tb_lap_carry);
    bench.letter('c', "idle_until outside the kernel: N ticks, +2 floor, due, foreign",
                 tc_idle_until);
    bench.letter('d', "the handshake the errata force, and the store before a Stop",
                 td_handshake);
    bench.letter('e', "A REAL KERNEL: three periodics, never early, one wake each",
                 te_kernel);
    bench.letter('f', "STOP 1 through the manager with the PLAIN site", tf_stop_through_manager);
    bench.letter('g', "delay_us on the interrupt-less SysTick", tg_delay);
    bench.letter('h', "an LSI-clocked ticker as a witness: the stated rate, a wake, the rule's price",
                 th_lsi_witness);
    bench.letter('i', "the lap wakes of a ten-second Stop: count, no PLL, awake time", ti_lap_wakes);
    bench.letter('u', "a keystroke as the foreign wake (outside z)", tu_keystroke, false);
    bench.letter('x', "diagnostic: the compare's latencies on the counter (outside z)", tx_latency, false);

    if (serial_ok) {
        const auto idcode = brio::DeviceIdcode::read();
        print(serial, crlf, "part DEV_ID ", hex(idcode.dev_id),
              " REV_ID ", hex(idcode.rev_id), crlf);
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? (Tb::on_crystal ? "LPTIM1 on LSE, tickless"
                                                  : "LPTIM1 on LSI, tickless")
                                : "FAILED",
              " at ", P::ticks_per_second, " Hz",
              " wall=", wall_ready ? (wall_on_lse ? "RTC on LSE" : "RTC on LSI")
                                   : "NO RTC",
              " RTCCLK=", rtcclk_hz,
              " (BDCR 0x", hex(brio::RtcDomain::bdcr()), ")",
              " tim2=", wall2_ok ? "64 MHz" : "FAILED",
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
