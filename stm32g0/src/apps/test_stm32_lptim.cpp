// test_stm32_lptim - the reference bench suite for the STM32G0's
// LOW-POWER TIMERS: stm32g0/lptim.hpp (RM0444 ch. 26, the whole of it)
// and, over it, the THIRD sleep site of stm32g0/sleep.hpp - the one that
// gives util/power.hpp's timed lift without owning the RTC.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. Four pads of port B carry LPTIM1's four signals on
// AF5 (DS13560 table 15) and every one of them is walked by ITS OWN
// INTERNAL PULL - which letter a proves before anything rests on it. The
// other stimuli are internal: the RTC's alarm A, COMP1's output, and the
// DMAMUX request generator, which takes LPTIM1_OUT as trigger input 20
// (table 56) and so counts this timer's own output edges with no pad and
// no CPU in the path.
//
// THE WALL CLOCK IS THE RTC, and it has to be: every TIM of this family
// stops in Stop and so does SysTick. The calendar runs on the LSE
// crystal with PREDIV_A 0 / PREDIV_S 32767 - the sleep suite's own
// split, a 30.5 us stopwatch that keeps counting with every clock in the
// chip stopped. THE DOMAIN IS NEVER RESET: RTCSEL is left where it is
// and the backup registers other suites wrote are not touched.
//
// THE BACKSTOP IS THE IWDG, armed once in main() at about 32 seconds and
// refreshed at the top of every letter and inside every long loop. It
// cannot be turned off again (28.3.1), which is the point: a sleep with
// no wake behind it costs one reboot and a banner instead of a board
// that has to be re-flashed.
//
// What is exercised, letter by letter:
//   a  the block: the reset walk, the pads proven free, the FOUR kernel
//      clocks weighed against the crystal, the enable and start
//      latencies, the CMP/ARR write handshake in CPU cycles, what a
//      forbidden write really does, and how often two CNT reads disagree
//   b  counting: the eight prescaler ratios, continuous against
//      one-shot and the switch both ways on the fly, COUNTRST's
//      synchronization cost counted, RSTARE, and PRELOAD measured as a
//      period that changes now or at the end
//   c  the waveform on LPTIM1_OUT: the arithmetic 26.4.10 never prints,
//      the three shapes, WAVPOL, the LptimPwm task's duty ladder off the
//      pad, and the output rate claim through the DMAMUX generator
//   d  counter mode: a pull-walked input sampled by the internal clock,
//      the input AS the clock with 26.4.12's lost first edges COUNTED,
//      the glitch filter as a real threshold, and a comparator's output
//      counted with no pad on the timer's side at all
//   e  triggers: an RTC alarm and a comparator starting the counter, the
//      ETR pad's trigger latency in kernel clocks, a trigger ignored
//      while running, and the timeout function
//   f  encoder mode on LPTIM1: quadrature walked by two pulls, the three
//      sub-modes of table 144, the direction flags, and LPTIM2 refused
//   g  THROUGH STOP: the counter on each of the four kernel clocks
//      across a real Stop 1, and the compare wake from Stop 0 and Stop 1
//   h  THE THIRD SLEEP SITE in a real kernel: a deadline met on the wall
//      and never early - and the RTC left at the chapter's own low-power
//      split while it happens
//   i  the two errata, as code and as measurement
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/comp.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/lptim.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/sleep.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/pwm_channel.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32Platform;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

using L1 = Lptim<1>;
using L2 = Lptim<2>;

// LPTIM1's four signals, all on port B at AF5 (DS13560 table 15). The
// numbers are the DATASHEET'S and nothing in the device header can check
// them - letter a is the only check there is.
constexpr PinSel out_sel{'B', 0, PinFunction::af5};   // LPTIM1_OUT
constexpr PinSel in1_sel{'B', 5, PinFunction::af5};   // LPTIM1_IN1
constexpr PinSel etr_sel{'B', 6, PinFunction::af5};   // LPTIM1_ETR
constexpr PinSel in2_sel{'B', 7, PinFunction::af5};   // LPTIM1_IN2
using OutPad = LptimPad<out_sel>;
using In1Pad = LptimPad<in1_sel>;
using EtrPad = LptimPad<etr_sel>;
using In2Pad = LptimPad<in2_sel>;
using OutPin = Pin<'B', 0>;
using In1Pin = Pin<'B', 5>;
using EtrPin = Pin<'B', 6>;
using In2Pin = Pin<'B', 7>;

/// COMP1's plus input is PA1 (its code `input2`), which is the pad the
/// analog campaign precharges. Nothing of this suite drives it except
/// through that helper.
using PadA1 = Pin<'A', 1>;
using C1 = Comp<1>;

using Dma1 = Dma<1>;
using EdgeCh = DmaChannel<1, 1>;
using Gen0 = DmaMuxGenerator<0>;

// ---------------------------------------------------------------------------
// The wall clock: the RTC's sub-second counter
// ---------------------------------------------------------------------------
//
// PREDIV_A 0 / PREDIV_S 32767 from the 32768 Hz crystal: ck_apre is the
// crystal itself, ck_spre is still exactly 1 Hz, so the calendar is a
// calendar AND the sub-second counter is a 30.5 us stopwatch that keeps
// counting through a Stop. Letter h moves the split to the chapter's own
// low-power one for a while, so every reader of this instrument asks the
// SILICON what the split is instead of assuming.

constexpr uint32_t lse_hz = 32768;
constexpr RtcPrescalers wall_prescalers{.async = 0, .sync = 32767};
constexpr RtcPrescalers lowpower_prescalers{.async = 127, .sync = 255};

bool wall_ready = false;

uint32_t wall_ticks_per_second() {
    return static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
}
uint32_t wall_modulus() { return 60u * wall_ticks_per_second(); }

/// How many sub-second ticks make one REAL second: ck_apre, the crystal
/// divided by PREDIV_A + 1.
uint32_t wall_hz() {
    return lse_hz / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
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

uint32_t wall_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) /
                                 wall_hz());
}
uint32_t wall_ms(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000ULL) /
                                 wall_hz());
}

bool wall_split(const RtcPrescalers& p) {
    if (Rtc::prescalers().sync == p.sync && Rtc::prescalers().async == p.async) {
        return true;
    }
    return Rtc::init(p, RtcDateTime{.hour = 0, .minute = 0, .second = 0,
                                    .day = 1, .month = 1, .year = 24,
                                    .weekday = 1});
}
bool wall_up() { return wall_split(wall_prescalers); }

// ---------------------------------------------------------------------------
// Instruments
// ---------------------------------------------------------------------------

/// The cycle-resolution stopwatch this stratum's suites share.
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
void spin_us(uint32_t us) { spin_cycles(us * cycles_per_us); }

void feed() { Iwdg::refresh(); }

/// A measurement window a transmit interrupt walks through is not a
/// measurement - three campaigns of this stratum have paid for that.
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    spin_cycles(SysClock::hz / 500u);
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// Percent difference of `v` from `want`, x10 (so 15 means 1.5 %).
uint32_t permille_off(uint32_t v, uint32_t want) {
    if (want == 0u) {
        return 0xFFFFFFFFu;
    }
    const uint32_t d = v > want ? v - want : want - v;
    return static_cast<uint32_t>((static_cast<uint64_t>(d) * 1000ULL) / want);
}

/// THE PRECONDITION OF EVERY PULL-WALKED LETTER: an input pad with
/// nothing attached goes where its own pull sends it - and the question
/// this chapter adds is whether it still does so WITH THE PAD HANDED TO
/// THE PERIPHERAL, which on the SAM depended on whether the function
/// drives the pad or only reads it.
template <class Pin>
bool pull_walks(bool in_af, PinFunction fn) {
    if (in_af) {
        Pin::function(fn, {.pull = PinPull::up});
    } else {
        Pin::input(PinPull::up);
    }
    spin_us(300);
    const bool high = Pin::read();
    Pin::pull(PinPull::down);
    spin_us(300);
    const bool low = Pin::read();
    Pin::pull(PinPull::up);
    spin_us(300);
    return high && !low;
}

/// Move a pull-walked pad and leave it there.
template <class Pin>
void walk(bool high, uint32_t settle_us = 60) {
    Pin::pull(high ? PinPull::up : PinPull::down);
    spin_us(settle_us);
}

/// Bring LPTIM1 up on `src` with `p`, running continuously at ARR =
/// 0xFFFF with no interrupt at all - the plain free-running counter
/// every rate measurement here uses.
bool free_run(LptimClock src, LptimPrescaler p) {
    L1::init();
    L1::kernel_clock(src);
    if (!L1::configure({.prescaler = p})) {
        return false;
    }
    L1::enable();
    if (!L1::set_arr(0xFFFFu) || !L1::wait_arr_ok()) {
        return false;
    }
    (void)L1::clear_flags(LptimFlag::all);
    return L1::start_continuous();
}

/// Counts made over `ms` milliseconds of WALL time, modulo the lap. The
/// double read is used where it can settle and a single read where it
/// cannot - which on a counter running at one count per CPU cycle is
/// never (letter a measures exactly that).
uint16_t counter_now() {
    const std::optional<uint16_t> v = L1::count();
    return v.value_or(L1::count_raw());
}
std::optional<uint32_t> counts_over(uint32_t ms) {
    const uint16_t a = counter_now();
    const uint32_t w0 = wall();
    while (wall_ms(wall_delta(w0, wall())) < ms) {
    }
    const uint16_t b = counter_now();
    return static_cast<uint32_t>((static_cast<uint32_t>(b) - a) & 0xFFFFu);
}

/// Everything this suite ever claims, put back: pads to analog, both
/// timers reset, the comparator off, the DMA quiet.
void quiet_everything() {
    // THE NVIC LINE FIRST, and it is not tidiness: letter h leaves the
    // sleep site's handler armed, and a later letter that configured an
    // interrupt would find its flags SWEPT by that handler before it
    // could look at them. (It did - letter i measured a flag as already
    // clear and called the driver's refusal a failure.)
    Nvic::disable(L1::irq());
    Nvic::disable(L2::irq());
    Nvic::clear_pending(L1::irq());
    Nvic::clear_pending(L2::irq());
    (void)L1::wake_line(false);
    (void)L2::wake_line(false);
    OutPad::release();
    In1Pad::release();
    EtrPad::release();
    In2Pad::release();
    L1::init();
    L1::release();
    L2::init();
    L2::release();
    (void)C1::enable(false);
    PadA1::analog();
    Gen0::release();
    EdgeCh::stop();
}

// ---------------------------------------------------------------------------
// Shared ISR state
// ---------------------------------------------------------------------------

/// Which body the shared TIM6/DAC/LPTIM1 vector runs. Both are compiled
/// and only one is right at a time, so every letter that drives LPTIM1
/// by hand says so. (The sleep suite paid for this lesson: a letter that
/// inherits the previous one's handler measures a wake that never
/// happened the way it thought.)
volatile bool site_round = false;
volatile bool kernel_live = false;
volatile uint32_t lptim_irqs = 0;
volatile uint32_t lptim_served0 = 0;
volatile uint32_t lptim_served1 = 0;
/// LPTIM_ISR read in the handler IMMEDIATELY AFTER the ICR store - the
/// witness of whether a flag clear crosses into the kernel clock domain
/// before the handler can return.
volatile uint32_t lptim_isr_after0 = 0;
volatile uint32_t lptim_wall = 0;

// ---------------------------------------------------------------------------
// The kernel half (letter h)
// ---------------------------------------------------------------------------

struct Blip {};
struct Woke {};

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip, Woke> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline uint16_t blips = 0;
    static inline uint16_t woke = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline bool last_ok = false;
    static inline uint32_t blip_wall = 0;
    static inline SleepDepth last_report = SleepDepth::none;

    static void clear() {
        blips = woke = wakes = votes = 0;
        last_ok = false;
        blip_wall = 0;
        last_report = SleepDepth::none;
    }

    static void init() { start(&only); }
    static Status only(const Event& e);
};

constexpr LptimTimedSleepConfig site_cfg{.instance = 1,
                                         .source = LptimClock::lse,
                                         .rate_hz = 32'768,
                                         .prescaler = LptimPrescaler::div32};
using Site = Stm32LptimTimedSleepSite<P, SysClock, site_cfg>;
using PlainSite = Stm32SleepSite<SysClock>;
using Manager = PowerManager<P, Site, PowerConfig{}, Probe>;
using K = Kernel<P, Probe, Manager>;

Probe::Status Probe::only(const Event& e) {
    return match(e,
        [](Entry) { return handled(); },
        [](Exit) { return handled(); },
        [](SleepVote v) {
            ++votes;
            last_ok = v.ok;
            return handled();
        },
        [](const PrepareSleep& p) {
            p.reply.send(SleepVote{true});
            return handled();
        },
        [](WakeReport w) {
            ++wakes;
            last_report = w.was;
            return handled();
        },
        [](Woke) {
            // util/power.hpp's convention, and with a Stop it is
            // LOAD-BEARING: a wake path with nothing to say says it with
            // SleepRequested{none}. Without it the site is never
            // disarmed, the clock stays at 16 MHz and the kernel's tick
            // stays paused - so nothing matures, ever.
            ++woke;
            post<Manager>(SleepRequested{SleepDepth::none,
                                         reply_to<Probe, SleepVote>()});
            return handled();
        },
        [](Blip) {
            ++blips;
            blip_wall = wall();
            post<Manager>(SleepRequested{SleepDepth::none,
                                         reply_to<Probe, SleepVote>()});
            return handled();
        });
}

/// Wait for the very next SysTick edge, and THIS IS A METROLOGY VERB,
/// not tidiness. A TimeEvent armed for N ticks matures at the (now + N)
/// tick EDGE, and the arming happens at an unknown phase inside the
/// current tick - so without this the deadline is anywhere from N - 1 to
/// N milliseconds of real time away and a wall reading of N - 1 would be
/// perfectly honest. Synchronizing first puts the phase at a few
/// microseconds, which is what lets "never early" be judged against the
/// nominal instead of against the nominal minus a tick.
void sync_to_tick() {
    const uint32_t t = Ticker::ticks();
    uint32_t guard = 2'000'000u;
    while (Ticker::ticks() == t && guard-- != 0u) {
    }
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
// a - the block
// =============================================================================
void ta_block() {
    feed();
    quiet_everything();
    (void)wall_up();

    // THE GATE. 5.2.17: a peripheral whose bus clock is off does not
    // answer register reads. What it answers instead is a fact about
    // this bus, printed rather than assumed.
    L1::bus_clock(false);
    const uint32_t closed = L1::regs().ARR;
    L1::init();
    const uint32_t isr = L1::regs().ISR;
    const uint32_t ier = L1::regs().IER;
    const uint32_t cfgr = L1::regs().CFGR;
    const uint32_t cr = L1::regs().CR;
    const uint32_t cmp = L1::regs().CMP;
    const uint32_t arr = L1::regs().ARR;
    const uint32_t cnt = L1::regs().CNT;
    const uint32_t cfgr2 = L1::regs().CFGR2;
    print(serial, "  ARR through the CLOSED bus gate reads ", hex(closed),
          "; open and RCC-reset: ISR=", hex(isr), " IER=", hex(ier), " CFGR=",
          hex(cfgr), " CR=", hex(cr), " CMP=", hex(cmp), " ARR=", hex(arr),
          " CNT=", hex(cnt), " CFGR2=", hex(cfgr2), crlf);
    bench.verdict("out of its RCC reset every register reads table 147's own "
                  "value - and ARR's is ONE, the only nonzero reset value in "
                  "the block",
                  isr == 0u && ier == 0u && cfgr == 0u && cr == 0u &&
                      cmp == 0u && arr == 1u && cnt == 0u && cfgr2 == 0u);

    // CFGR2's two fields: the header draws them FOUR bits wide where
    // 26.7.9 draws them two. Which is right is a question only the
    // silicon answers.
    L1::regs().CFGR2 = 0xFFFFFFFFu;
    const uint32_t cfgr2_all = L1::regs().CFGR2;
    L1::regs().CFGR2 = 0;
    print(serial, "  CFGR2 written all ones reads back ", hex(cfgr2_all),
          " (26.7.9 draws IN1SEL[1:0] and IN2SEL[1:0]; the device header "
          "declares FOUR bits each)", crlf);
    bench.verdict("A DOCUMENTARY DISPUTE SETTLED BY EXPERIMENT: CFGR2 keeps "
                  "EIGHT bits, so IN1SEL and IN2SEL are four bits wide as the "
                  "device header declares them and NOT the two bits 26.7.9 "
                  "draws - the two upper codes of each are simply unnamed",
                  cfgr2_all == 0xFFu);

    // THE PADS, before anything rests on them. The question this chapter
    // adds to the EXTI campaign's: does a pad handed to a peripheral
    // INPUT still follow its own pull? (On the SAM a DRIVING function
    // took the pull away and an input function did not.)
    Rcc::io_clock('B', true);
    const bool out_plain = pull_walks<OutPin>(false, PinFunction::af5);
    const bool in1_plain = pull_walks<In1Pin>(false, PinFunction::af5);
    const bool etr_plain = pull_walks<EtrPin>(false, PinFunction::af5);
    const bool in2_plain = pull_walks<In2Pin>(false, PinFunction::af5);
    const bool in1_af = pull_walks<In1Pin>(true, PinFunction::af5);
    const bool etr_af = pull_walks<EtrPin>(true, PinFunction::af5);
    const bool in2_af = pull_walks<In2Pin>(true, PinFunction::af5);
    In1Pin::analog();
    EtrPin::analog();
    In2Pin::analog();
    OutPin::analog();
    print(serial, "  pull-walk as plain inputs: PB0 ", out_plain, " PB5 ",
          in1_plain, " PB6 ", etr_plain, " PB7 ", in2_plain,
          "; under AF5 (the LPTIM's own function): PB5 ", in1_af, " PB6 ",
          etr_af, " PB7 ", in2_af, crlf);
    bench.verdict("all four LPTIM1 pads are free on this board: each follows "
                  "its own internal pull between the rails",
                  out_plain && in1_plain && etr_plain && in2_plain);
    bench.verdict("AND THE PULL SURVIVES THE HANDOVER: a pad given to an LPTIM "
                  "INPUT function still walks between the rails, which is what "
                  "makes every stimulus of this suite wireless",
                  in1_af && etr_af && in2_af);

    // THE FOUR KERNEL CLOCKS, each weighed against the crystal.
    struct Src {
        const char* name;
        LptimClock code;
        LptimPrescaler presc;
        uint32_t nominal;   // the counter's expected rate
        uint32_t lo;
        uint32_t hi;
    };
    const Src sources[] = {
        // LSE against a wall that IS the LSE: this measures the
        // arithmetic and the prescaler, not the crystal.
        {"LSE", LptimClock::lse, LptimPrescaler::div1, 32'768, 32'600, 32'940},
        // LSI: DS13560 bounds it at 29.5..34 kHz, and the other suites
        // of this stratum have measured 32536..32586 Hz.
        {"LSI", LptimClock::lsi, LptimPrescaler::div1, 32'586, 29'500, 34'000},
        // HSI16 divided by 128: 125 kHz, with HSI16's own 1 % trim.
        {"HSI16/128", LptimClock::hsi16, LptimPrescaler::div128, 125'000,
         123'000, 127'000},
        // PCLK is SYSCLK here, 64 MHz, divided by 128 = 500 kHz.
        {"PCLK/128", LptimClock::pclk, LptimPrescaler::div128, 500'000,
         495'000, 505'000},
    };
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_wait_ready();
    bool rates_ok = true;
    for (const Src& s : sources) {
        feed();
        if (!free_run(s.code, s.presc)) {
            rates_ok = false;
            continue;
        }
        const std::optional<uint32_t> n = counts_over(100);
        const uint32_t hz = n.has_value() ? *n * 10u : 0u;
        print(serial, "  kernel clock ", s.name, ": ", hz,
              " counts a second (nominal ", s.nominal, ", off by ",
              permille_off(hz, s.nominal), " per mille)", crlf);
        rates_ok = rates_ok && within(hz, s.lo, s.hi);
    }
    bench.verdict("all four kernel clocks of RCC_CCIPR.LPTIM1SEL drive the "
                  "counter, and each lands inside its own document's band",
                  rates_ok);

    // THE ENABLE AND START LATENCIES, at LSE where they are tens of
    // microseconds and at PCLK where they are nothing. 26.4.13 says two
    // counter clocks after ENABLE; 26.4.7 says three kernel clocks
    // between CNTSTRT and the counter really moving.
    uint32_t arr_us_at[2] = {0, 0};
    uint32_t move_us_at[2] = {0, 0};
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        feed();
        const LptimClock src = leg == 0u ? LptimClock::lse : LptimClock::pclk;
        const char* name = leg == 0u ? "LSE" : "PCLK";
        const uint32_t kernel_hz = leg == 0u ? 32'768u : SysClock::hz;
        L1::init();
        L1::kernel_clock(src);
        (void)L1::configure({.prescaler = LptimPrescaler::div1});
        console_drain();
        const uint32_t t0 = cycles_now();
        L1::enable();
        (void)L1::set_arr(0xFFFFu);
        (void)L1::wait_arr_ok();
        const uint32_t t_arr = cycles_now();
        (void)L1::start_continuous();
        uint32_t guard = 40'000'000u;
        while (L1::count_raw() == 0u && guard-- != 0u) {
        }
        const uint32_t t_move = cycles_now();
        arr_us_at[leg] = (t_arr - t0) / cycles_per_us;
        move_us_at[leg] = (t_move - t_arr) / cycles_per_us;
        const uint32_t kernel_ns = 1'000'000'000UL / kernel_hz;
        print(serial, "  ", name, ": enable + the first ARR write to ARROK ",
              arr_us_at[leg], " us; CNTSTRT to the counter moving ",
              move_us_at[leg], " us (one kernel clock is ", kernel_ns, " ns)",
              crlf);
    }
    // THE VERDICT JUDGES THE COMPARISON ITS OWN SENTENCE MAKES. "Tens of
    // microseconds" is read as at least ten, and "instant on PCLK" as
    // under ten - one LSE period being 30.5 us and one PCLK period 15 ns,
    // the two legs cannot land in the same decade unless the latency is
    // the APB's, which is the thing being denied.
    bench.verdict("the enable and the software start both cost REAL KERNEL "
                  "CLOCKS, not APB ones: on LSE the same two steps that are "
                  "instant on PCLK take tens of microseconds",
                  arr_us_at[0] >= 10u && move_us_at[0] >= 10u &&
                      arr_us_at[1] < 10u && move_us_at[1] < 10u);

    // THE WRITE HANDSHAKE, in CPU cycles. The chapter gives no number.
    uint32_t cmp_cycles_at[2] = {0, 0};
    uint32_t arr_cycles_at[2] = {0, 0};
    bool handshakes_landed = true;
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        feed();
        const LptimClock src = leg == 0u ? LptimClock::lse : LptimClock::pclk;
        const char* name = leg == 0u ? "LSE" : "PCLK";
        (void)free_run(src, LptimPrescaler::div1);
        console_drain();
        (void)L1::clear_flags(LptimFlag::cmpok | LptimFlag::arrok);
        const uint32_t c0 = cycles_now();
        (void)L1::set_cmp(1234);
        const bool ok_cmp = L1::wait_cmp_ok();
        const uint32_t c1 = cycles_now();
        (void)L1::clear_flags(LptimFlag::cmpok);
        const uint32_t a0 = cycles_now();
        (void)L1::set_arr(0xFFF0u);
        const bool ok_arr = L1::wait_arr_ok();
        const uint32_t a1 = cycles_now();
        (void)L1::clear_flags(LptimFlag::arrok);
        cmp_cycles_at[leg] = c1 - c0;
        arr_cycles_at[leg] = a1 - a0;
        handshakes_landed = handshakes_landed && ok_cmp && ok_arr;
        print(serial, "  ", name, ": CMP write to CMPOK ", cmp_cycles_at[leg],
              " cycles, ARR write to ARROK ", arr_cycles_at[leg], " cycles",
              ok_cmp && ok_arr ? "" : " (A FLAG NEVER ROSE)", crlf);
    }
    // Again the sentence is the predicate: "thousands" on the 32 kHz
    // clock, "a handful" on PCLK. Both flags must also really have
    // risen, or the numbers are the bound of a poll that gave up.
    bench.verdict("26.4.11's latency is real and is the KERNEL clock's: the "
                  "same write costs thousands of CPU cycles on a 32 kHz clock "
                  "and a handful on PCLK",
                  handshakes_landed && cmp_cycles_at[0] >= 1000u &&
                      arr_cycles_at[0] >= 1000u && cmp_cycles_at[1] < 1000u &&
                      arr_cycles_at[1] < 1000u);

    // A FORBIDDEN WRITE IS NOT ONE THING ON THIS FAMILY (the analog
    // campaign's finding). 26.7.4 says CFGR is disabled-only; whether
    // the silicon drops such a store or takes it is measured.
    (void)free_run(LptimClock::pclk, LptimPrescaler::div1);
    const uint32_t cfgr_before = L1::regs().CFGR;
    L1::regs().CFGR = cfgr_before | (7u << LPTIM_CFGR_PRESC_Pos);
    const uint32_t cfgr_after = L1::regs().CFGR;
    const bool refused = !L1::configure({.prescaler = LptimPrescaler::div8});
    print(serial, "  CFGR while ENABLED: ", hex(cfgr_before), " -> wrote ",
          hex(cfgr_before | (7u << LPTIM_CFGR_PRESC_Pos)), " -> reads ",
          hex(cfgr_after), crlf);
    bench.verdict("A FORBIDDEN WRITE IS NOT ONE THING ON THIS FAMILY (the "
                  "analog campaign's finding, met again): 26.7.4 says CFGR "
                  "must only be modified while the LPTIM is disabled, and the "
                  "store LANDS anyway - the register takes it, and what the "
                  "chapter forbids is what the COUNTER then does with it",
                  cfgr_after != cfgr_before);
    bench.verdict("so the refusal has to be the driver's: configure() returns "
                  "false and writes nothing while the block is enabled",
                  refused);

    // 26.4.7's other note: a start written while DISABLED is discarded.
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    (void)L1::configure({});
    const bool start_refused = !L1::start_continuous();
    L1::regs().CR = LPTIM_CR_CNTSTRT;     // by hand, past the refusal
    const uint32_t cr_after = L1::regs().CR;
    L1::enable();
    spin_us(200);
    const uint16_t after_enable = L1::count_raw();
    print(serial, "  CNTSTRT written while disabled: CR reads ", hex(cr_after),
          " and after ENABLE the counter is at ", after_enable, crlf);
    bench.verdict("a start written while the timer is disabled is DISCARDED BY "
                  "HARDWARE (26.4.7), so enabling afterwards does not start a "
                  "count - and the driver refuses the write in the first place",
                  start_refused && cr_after == 0u && after_enable == 0u);

    // THE DOUBLE-READ RULE, counted rather than claimed - and the
    // finding is that 26.7.8's rule has TWO reasons behind it, only one
    // of which the chapter names.
    struct ReadLeg {
        const char* name;
        LptimClock src;
        LptimPrescaler presc;
    };
    const ReadLeg read_legs[] = {
        {"LSE /1 (asynchronous, 30.5 us a count)", LptimClock::lse,
         LptimPrescaler::div1},
        {"PCLK /128 (synchronous, 2 us a count)", LptimClock::pclk,
         LptimPrescaler::div128},
        {"PCLK /1 (synchronous, ONE CYCLE a count)", LptimClock::pclk,
         LptimPrescaler::div1},
    };
    uint32_t disagreements[3] = {0, 0, 0};
    bool has_value[3] = {false, false, false};
    uint8_t idx = 0;
    for (const ReadLeg& leg : read_legs) {
        feed();
        (void)free_run(leg.src, leg.presc);
        console_drain();
        constexpr uint32_t samples = 2000;
        for (uint32_t i = 0; i < samples; ++i) {
            const uint16_t a = L1::count_raw();
            const uint16_t b = L1::count_raw();
            if (a != b) {
                ++disagreements[idx];
            }
        }
        has_value[idx] = L1::count().has_value();
        print(serial, "  ", leg.name, ": two consecutive CNT reads disagreed ",
              disagreements[idx], " times in ", samples, "; count() ",
              has_value[idx] ? "answered" : "GAVE NOTHING", crlf);
        ++idx;
    }
    bench.verdict("26.7.8's double read is not a formality on an asynchronous "
                  "kernel clock: at LSE a few readings in two thousand really "
                  "do disagree, and count() is what refuses to hand one back",
                  disagreements[0] > 0u && disagreements[0] < 200u &&
                      has_value[0]);
    bench.verdict("BUT THE RULE HAS A SECOND REASON THE CHAPTER DOES NOT NAME: "
                  "with the counter clocked at ONE COUNT PER CPU CYCLE the two "
                  "reads NEVER agree - not because the value is incoherent but "
                  "because it has moved - so count() gives nothing there and "
                  "count_raw() is the only reader that means anything on a "
                  "fast synchronous clock",
                  disagreements[2] == 2000u && !has_value[2] &&
                      disagreements[1] < 2000u && has_value[1]);

    quiet_everything();
}

// =============================================================================
// b - counting
// =============================================================================
void tb_counting() {
    feed();
    quiet_everything();

    // THE EIGHT PRESCALER RATIOS. The kernel clock is PCLK and the ruler
    // is the CPU's own cycle counter, so each window is sized to give
    // about four thousand counts whatever the divider is.
    bool ratios_ok = true;
    for (uint8_t p = 0; p < 8u; ++p) {
        feed();
        const LptimPrescaler presc = static_cast<LptimPrescaler>(p);
        if (!free_run(LptimClock::pclk, presc)) {
            ratios_ok = false;
            continue;
        }
        const uint32_t window = 4096u * lptim_prescaler_divider(presc);
        console_drain();
        const uint16_t a = L1::count_raw();
        const uint32_t t0 = cycles_now();
        while (cycles_now() - t0 < window) {
        }
        const uint16_t b = L1::count_raw();
        const uint32_t counts = static_cast<uint32_t>((b - a) & 0xFFFFu);
        const uint32_t off = permille_off(counts, 4096);
        print(serial, "  /", lptim_prescaler_divider(presc), ": ", counts,
              " counts in ", window, " cycles (want 4096, off ", off,
              " per mille)", crlf);
        // The polling loop's own granularity is tens of cycles, which at
        // /1 is a per cent of the window and at /128 is nothing.
        ratios_ok = ratios_ok && off < (p == 0u ? 40u : 20u);
    }
    bench.verdict("table 143's eight dividing factors are exact: the same "
                  "window gives the same count at every one of them",
                  ratios_ok);

    // CONTINUOUS AGAINST ONE-SHOT, and the two on-the-fly switches
    // 26.4.8 promises.
    feed();
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    (void)L1::configure({.prescaler = LptimPrescaler::div128});
    L1::enable();
    (void)L1::set_arr(999);
    (void)L1::wait_arr_ok();
    (void)L1::clear_flags(LptimFlag::all);
    (void)L1::start_single();
    spin_us(6000);   // 1000 counts at 500 kHz is 2 ms; three of them
    const uint16_t single_1 = L1::count_raw();
    spin_us(4000);
    const uint16_t single_2 = L1::count_raw();
    const bool arrm_once = (L1::status() & LptimFlag::arrm) != 0u;
    print(serial, "  one-shot at ARR = 999: the counter reads ", single_1,
          " and, four milliseconds later, ", single_2, crlf);
    bench.verdict("SNGSTRT stops the counter at the ARR match and it stays "
                  "stopped - one pass and no more (26.4.8)",
                  single_1 == single_2 && arrm_once);

    (void)L1::start_continuous();
    spin_us(6000);
    const uint16_t cont_1 = L1::count_raw();
    spin_us(2000);
    const uint16_t cont_2 = L1::count_raw();
    bench.verdict("...and a CNTSTRT written into that stopped one-shot "
                  "restarts it in Continuous mode, on the fly",
                  cont_1 != cont_2 || cont_2 != single_2);

    (void)L1::start_single();
    spin_us(8000);
    const uint16_t back_1 = L1::count_raw();
    spin_us(4000);
    const uint16_t back_2 = L1::count_raw();
    print(serial, "  the switch back: continuous read ", cont_1, "/", cont_2,
          ", then SNGSTRT stopped it at ", back_1, "/", back_2, crlf);
    bench.verdict("and a SNGSTRT written into a RUNNING continuous count "
                  "stops it at the next ARR match - both of 26.4.8's switch "
                  "sentences, measured",
                  back_1 == back_2);

    // COUNTRST's synchronization cost, counted at LSE where a kernel
    // clock is 30.5 us and the reads themselves cost nothing.
    feed();
    (void)free_run(LptimClock::lse, LptimPrescaler::div1);
    spin_us(2000);
    console_drain();
    const uint16_t before = L1::count_raw();
    const bool reset_ok = L1::reset_count();
    uint16_t peak = before;
    uint32_t guard = 4'000'000u;
    for (;;) {
        const uint16_t c = L1::count_raw();
        if (c < before) {
            break;
        }
        if (c > peak) {
            peak = c;
        }
        if (guard-- == 0u) {
            break;
        }
    }
    const uint32_t extra = static_cast<uint32_t>(peak - before);
    const bool second_refused = !(L1::count_reset_pending() && L1::reset_count());
    print(serial, "  COUNTRST at ", before, ": the counter reached ", peak,
          " before the reset landed - ", extra,
          " extra pulses (26.4.14 says three kernel clocks)", crlf);
    bench.verdict("COUNTRST really is synchronous: the counter takes a few "
                  "more pulses before the reset lands, which is 26.4.14's own "
                  "sentence with a number on it",
                  reset_ok && extra <= 8u);
    bench.verdict("and a second COUNTRST while the first still stands is "
                  "refused (26.7.5's Caution, which no hardware enforces)",
                  second_refused);

    // RSTARE: the read IS the reset.
    feed();
    (void)free_run(LptimClock::lse, LptimPrescaler::div1);
    spin_us(2000);
    const bool rstare_ok = L1::reset_on_read(true);
    const uint16_t r1 = L1::count_raw();
    spin_us(200);
    const uint16_t r2 = L1::count_raw();
    (void)L1::reset_on_read(false);
    print(serial, "  RSTARE on: two reads 200 us apart gave ", r1, " and ", r2,
          crlf);
    bench.verdict("RSTARE makes every read of CNT reset it, which is why "
                  "count()'s double read is IMPOSSIBLE in that mode and "
                  "count_raw() is the only legal reader (26.4.14)",
                  rstare_ok && r1 > 8u && r2 < 16u);

    // PRELOAD, measured as WHEN a new period takes effect. The witness
    // is the ARRM flag's own timing on the wall.
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        feed();
        const bool preload = leg == 1u;
        L1::init();
        L1::kernel_clock(LptimClock::lse);
        (void)L1::configure({.prescaler = LptimPrescaler::div1,
                             .preload = preload});
        L1::enable();
        (void)L1::set_arr(3275);      // 3276 ticks of LSE = 100 ms
        (void)L1::wait_arr_ok();
        (void)L1::clear_flags(LptimFlag::all);
        (void)L1::start_continuous();
        // Land on a period boundary first.
        while ((L1::status() & LptimFlag::arrm) == 0u) {
        }
        (void)L1::clear_flags(LptimFlag::arrm | LptimFlag::arrok);
        (void)L1::set_arr(1637);      // half the period
        const uint32_t w0 = wall();
        while ((L1::status() & LptimFlag::arrm) == 0u) {
        }
        const uint32_t first = wall_ms(wall_delta(w0, wall()));
        (void)L1::clear_flags(LptimFlag::arrm);
        const uint32_t w1 = wall();
        while ((L1::status() & LptimFlag::arrm) == 0u) {
        }
        const uint32_t second = wall_ms(wall_delta(w1, wall()));
        print(serial, "  PRELOAD ", preload ? "1" : "0",
              ": after halving ARR the next period was ", first,
              " ms and the one after ", second, " ms", crlf);
        if (!preload) {
            bench.verdict("with PRELOAD clear an ARR write is taken AT ONCE - "
                          "the very next period is already the new one",
                          within(first, 45u, 55u) && within(second, 45u, 55u));
        } else {
            bench.verdict("with PRELOAD set it is taken AT THE END OF THE "
                          "CURRENT PERIOD (26.4.11) - the next period is still "
                          "the old one and only the one after is new",
                          within(first, 95u, 105u) && within(second, 45u, 55u));
        }
    }

    quiet_everything();
}

// =============================================================================
// c - the waveform
// =============================================================================

/// Sample LPTIM1_OUT through GPIOB's IDR - the pad's input buffer stays
/// live in alternate mode (7.3.1), so a driven waveform is readable.
/// THE LOOP MUST NOT BRANCH on what it reads: a branch makes its
/// duration depend on the level and biases the answer (the tim campaign
/// paid for this).
uint32_t pad_permille(uint32_t samples) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        high += (Port<'B'>::in() >> 0) & 1u;   // PB0 = LPTIM1_OUT
    }
    return (high * 1000u + samples / 2u) / samples;
}

/// Bring LPTIM1 up as a 64 kHz PWM on PB0 - the rate the pad sampler
/// wants. THE WAVEFORM HAS TO BE FAST: the sampling loop is a fixed-rate
/// sampler, so it must cross HUNDREDS of periods for the aliasing
/// between the two rates to average out (the tim campaign's own
/// finding, paid for again here - a 1 kHz waveform sampled over three
/// periods reported a 500 per mille duty as 552).
bool pwm_64k(uint16_t compare, bool inverted = false) {
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    if (!L1::configure({.prescaler = LptimPrescaler::div1,
                        .output_inverted = inverted})) {
        return false;
    }
    L1::enable();
    if (!L1::set_arr(999) || !L1::wait_arr_ok()) {
        return false;
    }
    (void)L1::clear_flags(LptimFlag::all);
    if (!L1::set_cmp(compare) || !L1::wait_cmp_ok()) {
        return false;
    }
    (void)L1::clear_flags(LptimFlag::cmpok);
    return L1::start_continuous();
}

void tc_waveform() {
    feed();
    quiet_everything();
    OutPad::claim();

    // The arithmetic 26.4.10 describes in words. PCLK/128 = 500 kHz,
    // ARR = 499, so a period is 1 kHz if a period is ARR + 1 ticks.
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    (void)L1::configure({.prescaler = LptimPrescaler::div128});
    L1::enable();
    (void)L1::set_arr(499);
    (void)L1::wait_arr_ok();
    (void)L1::clear_flags(LptimFlag::all);
    (void)L1::set_cmp(249);
    (void)L1::wait_cmp_ok();
    (void)L1::clear_flags(LptimFlag::cmpok);
    (void)L1::start_continuous();

    // The PERIOD, counted off the ARRM flag over a wall window.
    console_drain();
    (void)L1::clear_flags(LptimFlag::arrm);
    const uint32_t w0 = wall();
    uint32_t periods = 0;
    while (wall_ms(wall_delta(w0, wall())) < 200u) {
        if ((L1::status() & LptimFlag::arrm) != 0u) {
            (void)L1::clear_flags(LptimFlag::arrm);
            ++periods;
        }
    }
    const uint32_t hz = periods * 5u;
    print(serial, "  ARR = 499 on a 500 kHz counter: ", periods,
          " periods in 200 ms = ", hz, " Hz (ARR + 1 ticks would be 1000 Hz, "
          "ARR ticks 1002)", crlf);
    bench.verdict("A PERIOD IS ARR + 1 COUNTER TICKS - the counter runs 0..ARR "
                  "inclusive, which 26.4.10 never prints",
                  within(hz, 995u, 1005u));

    // THE HIGH TIME, from the pad, on a 64 kHz waveform the sampler can
    // really average. 26.4.10 describes the shape in words and prints no
    // formula, so the formula is MEASURED: the two candidates that read
    // out of its sentences are ARR - CMP ticks and ARR - CMP + 1, and
    // they are one tick apart in a thousand - which is exactly what a
    // 60000-sample census of a 64 kHz waveform tells apart.
    struct Duty {
        uint16_t cmp;
        uint32_t want;   // per mille, (ARR - CMP + 1) / (ARR + 1)
    };
    const Duty duties[] = {{0, 1000}, {249, 751}, {499, 501}, {749, 251},
                           {899, 101}};
    bool duty_ok = true;
    for (const Duty& d : duties) {
        feed();
        if (!pwm_64k(d.cmp)) {
            duty_ok = false;
            continue;
        }
        spin_us(2000);
        const uint32_t got = pad_permille(60000);
        print(serial, "  ARR 999, CMP ", d.cmp, ": pad high ", got,
              " per mille, (ARR - CMP + 1)/(ARR + 1) = ", d.want, crlf);
        duty_ok = duty_ok && got + 20u >= d.want && got <= d.want + 20u;
    }
    bench.verdict("and the HIGH TIME IS ARR - CMP + 1 TICKS out of the "
                  "period's ARR + 1 - one tick MORE than 26.4.10's two "
                  "sentences read on their own, so CMP = 0 is a FULL duty and "
                  "not one tick short of it",
                  duty_ok);

    // The two values 26.4.10 forbids ("ARR must be strictly greater than
    // CMP"), which is a rule with no hardware behind it.
    (void)pwm_64k(999);          // CMP == ARR
    spin_us(2000);
    const uint32_t at_arr = pad_permille(60000);
    (void)pwm_64k(1200);         // CMP > ARR
    spin_us(2000);
    const uint32_t past_arr = pad_permille(60000);
    print(serial, "  the forbidden pair: CMP = ARR gives ", at_arr,
          " per mille, CMP > ARR gives ", past_arr, crlf);
    bench.verdict("26.4.10's \"ARR must be strictly greater than CMP\" is a "
                  "rule with no hardware behind it, and what the silicon "
                  "really does with the two illegal settings is a FLAT LOW "
                  "output - not one tick of duty, not a fault, and NOT what "
                  "the arithmetic above would extrapolate to (CMP = ARR would "
                  "be one tick high)",
                  at_arr < 30u && past_arr < 30u);

    // WAVPOL. The compare is deliberately NOT the half-way one: a 50 %
    // waveform inverted is still 50 %, so a symmetric duty would let a
    // polarity bit that does nothing pass unnoticed.
    (void)pwm_64k(749);                       // 250 per mille
    spin_us(2000);
    const uint32_t straight = pad_permille(60000);
    (void)pwm_64k(749, true);
    spin_us(2000);
    const uint32_t inverted = pad_permille(60000);
    print(serial, "  CMP = 749: WAVPOL 0 gives ", straight,
          " per mille and WAVPOL 1 gives ", inverted, crlf);
    bench.verdict("WAVPOL inverts the waveform and nothing else",
                  within(straight, 230u, 270u) && within(inverted, 730u, 770u));

    // ONE-PULSE (WAVE = 0 with SNGSTRT) and SET-ONCE (WAVE = 1).
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        feed();
        const bool set_once = leg == 1u;
        L1::init();
        L1::kernel_clock(LptimClock::pclk);
        (void)L1::configure({.prescaler = LptimPrescaler::div128,
                             .waveform = set_once ? LptimWaveform::set_once
                                                  : LptimWaveform::pwm_or_pulse});
        L1::enable();
        (void)L1::set_arr(999);
        (void)L1::wait_arr_ok();
        (void)L1::set_cmp(499);
        (void)L1::wait_cmp_ok();
        (void)L1::clear_flags(LptimFlag::all);
        (void)L1::start_single();
        // The pulse is 500 ticks at 500 kHz = 1 ms, inside a 2 ms period.
        spin_us(1500);
        const bool during = OutPin::read();
        spin_us(4000);
        const bool after = OutPin::read();
        print(serial, "  ", set_once ? "set-once" : "one-pulse",
              ": the pad reads ", during, " inside the pulse and ", after,
              " long after it", crlf);
        if (!set_once) {
            bench.verdict("one-pulse: the output is reset permanently after "
                          "the single pulse (26.4.10)",
                          during && !after);
        } else {
            bench.verdict("set-once: the output KEEPS its last level instead, "
                          "which is the whole difference between the two",
                          during && after);
        }
    }

    // THE TASK: util/pwm_channel.hpp's PwmChannel on its FOURTH
    // implementation, driven through the duty ladder every PWM letter of
    // this project uses.
    feed();
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    using Lamp = LptimPwm<L1, 999>;
    static_assert(PwmChannel<Lamp>);
    const bool lamp_up = Lamp::setup(LptimPrescaler::div1);
    bool ladder_ok = lamp_up;
    const uint16_t ladder[] = {0, 250, 500, 750, 999};
    for (uint16_t v : ladder) {
        feed();
        Lamp::duty(v);
        spin_us(2000);
        const uint32_t got = pad_permille(60000);
        // The task's mapping is CMP = max - v, so the measured duty is
        // (v + 1) / (max + 1) per unit - except at v = 0, where CMP
        // lands on ARR and the output is the flat LOW measured above.
        // BOTH ENDPOINTS ARE THEREFORE EXACT, by two different routes.
        const uint32_t want = v == 0u ? 0u
                                      : (static_cast<uint32_t>(v) + 1u) * 1000u /
                                            (999u + 1u);
        print(serial, "  LptimPwm duty ", v, "/999: pad ", got,
              " per mille (the mapping says ", want, ")", crlf);
        ladder_ok = ladder_ok && (got + 20u >= want) && (got <= want + 20u);
    }
    bench.verdict("LptimPwm is a PwmChannel: the duty ladder lands where the "
                  "task's stated mapping puts it, both endpoints included",
                  ladder_ok);
    bench.verdict("and the task's own readback agrees with what was asked",
                  Lamp::duty() == 999u);

    // THE OUTPUT RATE CLAIM ("signals with frequencies up to the LPTIM
    // clock frequency divided by 2"), through the DMAMUX REQUEST
    // GENERATOR: LPTIM1_OUT is table 56's trigger input 20, so a DMA
    // channel with no peripheral counts this timer's edges with no pad
    // and no CPU in the path.
    feed();
    Dma1::bus_clock(true);
    Dma1::reset();
    L1::init();
    L1::kernel_clock(LptimClock::lse);
    (void)L1::configure({.prescaler = LptimPrescaler::div1});
    L1::enable();
    (void)L1::set_arr(1);        // period 2 ticks: the fastest waveform
    (void)L1::wait_arr_ok();
    (void)L1::set_cmp(0);
    (void)L1::wait_cmp_ok();
    (void)L1::start_continuous();

    static uint32_t sink = 0;
    static const uint32_t source_word = 0x5A5A5A5Au;
    (void)Gen0::configure(L1::dmamux_generator_input, DmaMuxEdge::rising, 1);
    (void)EdgeCh::prepare(DmaTransfer{
        .peripheral = const_cast<uint32_t*>(&source_word),
        .memory = &sink,
        .count = 20000,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .peripheral_increment = false,
                   .memory_increment = false,
                   .peripheral_width = DmaWidth::word,
                   .memory_width = DmaWidth::word}});
    DmaMux::request(EdgeCh::mux_channel, Gen0::request_id);
    (void)EdgeCh::enable(true);
    Gen0::enable(true);
    console_drain();
    const uint32_t gw0 = wall();
    const uint16_t g0 = EdgeCh::count();
    while (wall_ms(wall_delta(gw0, wall())) < 200u) {
    }
    const uint16_t g1 = EdgeCh::count();
    Gen0::enable(false);
    EdgeCh::stop();
    const uint32_t edges = static_cast<uint32_t>(g0 - g1);
    const uint32_t edge_hz = edges * 5u;
    print(serial, "  ARR = 1 on the 32768 Hz crystal: the DMAMUX generator "
                  "counted ", edges, " rising edges in 200 ms = ", edge_hz,
          " Hz, against the kernel clock's half, 16384", crlf);
    bench.verdict("26.4.10's \"up to the LPTIM clock frequency divided by 2\" "
                  "is exact, measured by a DMA channel with no peripheral, no "
                  "pad and no CPU in the loop",
                  within(edge_hz, 16'200u, 16'560u));
    print(serial, "  the same claim at the PCLK extreme (32 MHz of output) is "
                  "DECLINED: no counter this board can spare resolves it, and "
                  "the DMA itself cannot serve requests that fast", crlf);

    Dma1::reset();
    Dma1::bus_clock(false);
    quiet_everything();
}

// =============================================================================
// d - counter mode
// =============================================================================

/// N pull-walked edges on IN1, slow enough for any filter setting under
/// test to pass them.
void walk_in1(uint32_t edges, uint32_t half_us) {
    for (uint32_t i = 0; i < edges; ++i) {
        walk<In1Pin>(true, half_us);
        walk<In1Pin>(false, half_us);
    }
}

void td_counter_mode() {
    feed();
    quiet_everything();

    // COUNTMODE = 1: the INTERNAL clock samples the input, so nothing is
    // lost at the start and the prescaler must be /1 (26.4.12).
    In1Pad::claim_input(PinPull::down);
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    const bool sampled_up = L1::configure({.count_external = true,
                                           .input1 = LptimInput1::pad});
    L1::enable();
    (void)L1::set_arr(0xFFFFu);
    (void)L1::wait_arr_ok();
    (void)L1::start_continuous();
    spin_us(500);
    const uint16_t s0 = L1::count_raw();
    walk_in1(20, 60);
    const uint16_t s1 = L1::count_raw();
    const uint32_t sampled = static_cast<uint32_t>((s1 - s0) & 0xFFFFu);
    print(serial, "  COUNTMODE = 1, 20 pull-walked rising edges on PB5: the "
                  "counter moved by ", sampled, crlf);
    bench.verdict("the internal clock SAMPLING a pull-walked input counts "
                  "every edge, losing none - which is the difference between "
                  "COUNTMODE = 1 and CKSEL = 1",
                  sampled_up && sampled == 20u);

    // A CONTROL: with the pad left alone nothing counts.
    const uint16_t q0 = L1::count_raw();
    spin_us(20000);
    const uint16_t q1 = L1::count_raw();
    bench.verdict("and with the pad left alone the counter does not move at "
                  "all - the input really is the only thing driving it",
                  q0 == q1);

    // CKSEL = 1: THE INPUT IS THE CLOCK. 26.4.12's own sentence is that
    // "the first five active edges on the LPTIM external Input1 (after
    // LPTIM is enable) are lost" - which is a number, so it is counted.
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        feed();
        const bool falling = leg == 1u;
        L1::init();
        L1::kernel_clock(LptimClock::pclk);
        (void)L1::configure({.clock = LptimClockSource::external_input1,
                             .clock_polarity = falling
                                                   ? LptimClockPolarity::falling
                                                   : LptimClockPolarity::rising,
                             .input1 = LptimInput1::pad});
        L1::enable();
        (void)L1::set_arr(0xFFFFu);
        (void)L1::wait_arr_ok();
        (void)L1::start_continuous();
        walk<In1Pin>(false, 200);
        constexpr uint32_t applied = 20;
        walk_in1(applied, 60);
        const uint16_t got = L1::count_raw();
        print(serial, "  CKSEL = 1, ", falling ? "falling" : "rising",
              " edges: ", applied, " applied, ", got, " counted - ",
              applied - got, " lost at the start", crlf);
        bench.verdict(falling ? "the input IS the clock on the FALLING edge, "
                                "and the edges lost at the start are counted"
                              : "the input IS the clock on the RISING edge, "
                                "and the edges lost at the start are counted",
                      got + 8u >= applied && got < applied);
    }

    bench.verdict("both polarities work and BOTH EDGES AT ONCE is refused - "
                  "26.4.12 gives an externally clocked counter one edge or the "
                  "other, never both",
                  !L1::configure({.clock = LptimClockSource::external_input1,
                                  .clock_polarity = LptimClockPolarity::both}) &&
                      !lptim_config_valid(
                          1, LptimConfig{.clock = LptimClockSource::external_input1,
                                         .clock_polarity = LptimClockPolarity::both}));

    // THE GLITCH FILTER as a real threshold. The kernel clock is LSE, so
    // one sample is 30.5 us and CKFLT = 8 samples is 244 us: a 60 us
    // blip should be rejected and a 1 ms one counted. (26.4.5 makes an
    // internal clock mandatory for the filters, which is why this leg
    // runs on COUNTMODE = 1 and not on CKSEL = 1.)
    feed();
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        const bool filtered = leg == 1u;
        L1::init();
        L1::kernel_clock(LptimClock::lse);
        (void)L1::configure({.count_external = true,
                             .clock_filter = filtered ? LptimFilter::samples8
                                                      : LptimFilter::none,
                             .input1 = LptimInput1::pad});
        L1::enable();
        (void)L1::set_arr(0xFFFFu);
        (void)L1::wait_arr_ok();
        (void)L1::start_continuous();
        walk<In1Pin>(false, 2000);
        const uint16_t f0 = L1::count_raw();
        for (uint32_t i = 0; i < 10u; ++i) {
            walk<In1Pin>(true, 60);      // a 60 us blip
            walk<In1Pin>(false, 2000);
        }
        const uint16_t f1 = L1::count_raw();
        for (uint32_t i = 0; i < 10u; ++i) {
            walk<In1Pin>(true, 2000);    // 2 ms, well past 8 samples
            walk<In1Pin>(false, 2000);
        }
        const uint16_t f2 = L1::count_raw();
        const uint32_t blips = static_cast<uint32_t>((f1 - f0) & 0xFFFFu);
        const uint32_t longs = static_cast<uint32_t>((f2 - f1) & 0xFFFFu);
        print(serial, "  CKFLT ", filtered ? "8 samples" : "off",
              " on a 32768 Hz clock: ten 60 us blips counted ", blips,
              ", ten 2 ms pulses counted ", longs, crlf);
        if (!filtered) {
            bench.verdict("with the filter off a short blip is a transition "
                          "like any other",
                          blips >= 8u && longs >= 8u);
        } else {
            bench.verdict("and CKFLT = 8 samples is a REAL THRESHOLD: at one "
                          "sample per 30.5 us a 60 us blip is rejected and a "
                          "2 ms pulse passes",
                          blips <= 2u && longs >= 8u);
        }
    }

    // IN1SEL = COMP1_OUT: the timer's input with NO PAD ON THE TIMER'S
    // SIDE AT ALL. The comparator is flipped by precharging its own plus
    // input, the analog campaign's wireless technique.
    feed();
    In1Pad::release();
    C1::init();
    constexpr CompConfig comp_cfg{.positive = CompPositive::input2,   // PA1
                                  .negative = CompNegative::vrefint_half};
    const bool comp_up =
        C1::claim_inputs(comp_cfg) && C1::configure(comp_cfg) && C1::enable(true);
    spin_us(500);
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    const bool internal_route = L1::configure({.count_external = true,
                                               .input1 = LptimInput1::comp1_out});
    L1::enable();
    (void)L1::set_arr(0xFFFFu);
    (void)L1::wait_arr_ok();
    (void)L1::start_continuous();
    PadA1::output(false);
    spin_us(300);
    PadA1::analog();
    spin_us(300);
    const uint16_t c0 = L1::count_raw();
    constexpr uint32_t flips = 12;
    for (uint32_t i = 0; i < flips; ++i) {
        PadA1::output(true);
        spin_us(300);
        PadA1::analog();
        spin_us(300);
        PadA1::output(false);
        spin_us(300);
        PadA1::analog();
        spin_us(300);
    }
    const uint16_t c1 = L1::count_raw();
    const uint32_t counted = static_cast<uint32_t>((c1 - c0) & 0xFFFFu);
    print(serial, "  IN1SEL = COMP1_OUT: ", flips,
          " comparator flips counted as ", counted,
          " (the comparator's own input is a precharged PA1; the TIMER sees "
          "no pad at all)", crlf);
    bench.verdict("table 140's mux1 is real: the LPTIM counts a comparator's "
                  "output over an internal route, with no pad on the timer's "
                  "side",
                  comp_up && internal_route && counted == flips);

    quiet_everything();
}

// =============================================================================
// e - triggers
// =============================================================================
void te_triggers() {
    feed();
    quiet_everything();
    (void)wall_up();

    // ROW 0: the ETR PAD, pull-walked. The counter is armed and does not
    // move until the edge arrives - which also puts a number on
    // 26.4.7's "two-counter-clock period latency".
    EtrPad::claim_input(PinPull::down);
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    const bool etr_cfg = L1::configure({.prescaler = LptimPrescaler::div1,
                                        .trigger = LptimTrigger::etr_pad,
                                        .trigger_edge = LptimTriggerEdge::rising});
    L1::enable();
    (void)L1::set_arr(0xFFFFu);
    (void)L1::wait_arr_ok();
    (void)L1::clear_flags(LptimFlag::all);
    (void)L1::start_continuous();
    spin_us(500);
    const uint16_t armed_at = L1::count_raw();
    const bool no_trig_yet = (L1::status() & LptimFlag::exttrig) == 0u;
    console_drain();
    const uint32_t t0 = cycles_now();
    EtrPin::pull(PinPull::up);
    uint32_t guard = 8'000'000u;
    while (L1::count_raw() == armed_at && guard-- != 0u) {
    }
    const uint32_t latency = cycles_now() - t0;
    const bool trig_flag = (L1::status() & LptimFlag::exttrig) != 0u;
    print(serial, "  ETR on PB6: armed at ", armed_at,
          " and still there until the edge; edge to first count ", latency,
          " CPU cycles (a kernel clock here is 1 cycle; 26.4.7 promises two "
          "for the synchronization)", crlf);
    bench.verdict("TRIGEN != 00 really arms rather than starts: the counter "
                  "does not move until an edge arrives on the trigger the "
                  "config named, and EXTTRIG marks it",
                  etr_cfg && armed_at == 0u && no_trig_yet && trig_flag);

    // A SECOND TRIGGER WHILE RUNNING IS IGNORED, and 26.7.1 says the
    // flag is not even set for it.
    (void)L1::clear_flags(LptimFlag::exttrig);
    const uint16_t running = L1::count_raw();
    EtrPin::pull(PinPull::down);
    spin_us(200);
    EtrPin::pull(PinPull::up);
    spin_us(200);
    const bool ignored = (L1::status() & LptimFlag::exttrig) == 0u;
    const uint16_t still_running = L1::count_raw();
    print(serial, "  a second ETR edge while the counter runs: EXTTRIG ",
          ignored ? "stayed clear" : "ROSE", ", the counter went from ",
          running, " to ", still_running, crlf);
    bench.verdict("a trigger arriving while the timer already runs is ignored "
                  "AND ITS FLAG IS NOT SET - 26.7.1's easily-missed sentence, "
                  "measured",
                  ignored && still_running != running);
    EtrPad::release();

    // THE TIMEOUT FUNCTION (26.4.9): the same trigger now RESETS the
    // counter, so a compare match means "no trigger arrived in time".
    feed();
    EtrPad::claim_input(PinPull::down);
    using Timeout = LptimTimeout<L1>;
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    // 500 kHz counter, CMP = 5000 -> a 10 ms timeout.
    const bool to_up = Timeout::setup(LptimTrigger::etr_pad,
                                      LptimTriggerEdge::rising, 5000,
                                      LptimPrescaler::div128);
    (void)L1::clear_flags(LptimFlag::all);
    EtrPin::pull(PinPull::up);
    spin_us(200);
    // FED: an edge every 3 ms, six times - the compare is never reached.
    bool fed_clean = true;
    for (uint8_t i = 0; i < 6u; ++i) {
        EtrPin::pull(PinPull::down);
        spin_us(1500);
        EtrPin::pull(PinPull::up);
        spin_us(1500);
        if (Timeout::expired()) {
            fed_clean = false;
        }
    }
    // STARVED: nothing for 30 ms.
    spin_us(30000);
    const bool starved = Timeout::expired();
    print(serial, "  timeout at 10 ms: fed every 3 ms it ",
          fed_clean ? "never expired" : "EXPIRED", "; starved for 30 ms it ",
          starved ? "expired" : "DID NOT", crlf);
    bench.verdict("26.4.9's timeout function: a trigger RESTARTS the counter, "
                  "so a compare match is the statement that no trigger arrived "
                  "in time",
                  to_up && fed_clean && starved);
    EtrPad::release();

    // ROW 1: THE RTC'S ALARM A, with no interrupt of its own - the
    // alarm is the trigger and nothing else.
    feed();
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    const bool rtc_cfg = L1::configure({.prescaler = LptimPrescaler::div128,
                                        .trigger = LptimTrigger::rtc_alarm_a,
                                        .trigger_edge = LptimTriggerEdge::rising});
    L1::enable();
    (void)L1::set_arr(0xFFFFu);
    (void)L1::wait_arr_ok();
    (void)L1::clear_flags(LptimFlag::all);
    (void)L1::start_continuous();
    RtcReading now{};
    (void)Rtc::read(now);
    // Every second, on the seconds boundary: masked everything but the
    // seconds field is not needed - the default alarm fires once a
    // second, which is all this needs.
    const bool alarm_set = Rtc::set_alarm(RtcAlarmId::a, RtcAlarm{}, false);
    const uint16_t before_alarm = L1::count_raw();
    const uint32_t aw0 = wall();
    uint32_t aguard = 40'000'000u;
    while (L1::count_raw() == before_alarm && aguard-- != 0u) {
        if (wall_ms(wall_delta(aw0, wall())) > 2500u) {
            break;
        }
    }
    const uint16_t after_alarm = L1::count_raw();
    const bool alarm_trig = (L1::status() & LptimFlag::exttrig) != 0u;
    Rtc::clear_alarm(RtcAlarmId::a);
    print(serial, "  RTC alarm A as the trigger: the counter went from ",
          before_alarm, " to ", after_alarm, " and EXTTRIG ",
          alarm_trig ? "rose" : "STAYED CLEAR", crlf);
    bench.verdict("table 138's row 1 is real: the RTC's alarm A starts an "
                  "LPTIM with no pad, no wire and no interrupt anywhere",
                  rtc_cfg && alarm_set && after_alarm != before_alarm &&
                      alarm_trig);

    // ROW 6: COMP1_OUT. The same precharge stimulus letter d uses.
    feed();
    C1::init();
    constexpr CompConfig comp_cfg{.positive = CompPositive::input2,
                                  .negative = CompNegative::vrefint_half};
    const bool comp_up =
        C1::claim_inputs(comp_cfg) && C1::configure(comp_cfg) && C1::enable(true);
    PadA1::output(false);
    spin_us(300);
    PadA1::analog();
    spin_us(300);
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    const bool comp_cfg_ok =
        L1::configure({.prescaler = LptimPrescaler::div128,
                       .trigger = LptimTrigger::comp1_out,
                       .trigger_edge = LptimTriggerEdge::rising});
    L1::enable();
    (void)L1::set_arr(0xFFFFu);
    (void)L1::wait_arr_ok();
    (void)L1::clear_flags(LptimFlag::all);
    (void)L1::start_continuous();
    spin_us(1000);
    const uint16_t comp_before = L1::count_raw();
    PadA1::output(true);
    spin_us(300);
    PadA1::analog();
    spin_us(2000);
    const uint16_t comp_after = L1::count_raw();
    const bool comp_trig = (L1::status() & LptimFlag::exttrig) != 0u;
    print(serial, "  COMP1_OUT as the trigger: the counter went from ",
          comp_before, " to ", comp_after, ", EXTTRIG ",
          comp_trig ? "set" : "CLEAR", crlf);
    bench.verdict("and table 138's row 6 too: a comparator's output starts the "
                  "counter over an internal route",
                  comp_up && comp_cfg_ok && comp_before == 0u &&
                      comp_after != 0u && comp_trig);

    quiet_everything();
}

// =============================================================================
// f - encoder mode
// =============================================================================

/// The quadrature walker: two pulls stepping round the Gray code
/// 00 -> 10 -> 11 -> 01 -> 00, one TRANSITION at a time. Counting
/// transitions rather than "cycles" is what makes the expected counts
/// arithmetic instead of a guess: over any whole number of FOUR
/// transitions exactly half of them are rising edges of one input or the
/// other, whatever phase the walk starts in.
uint8_t quad_pos = 0;

void quad_apply(uint32_t hold_us = 200) {
    const bool a = (quad_pos == 1u || quad_pos == 2u);
    const bool b = (quad_pos == 2u || quad_pos == 3u);
    In1Pin::pull(a ? PinPull::up : PinPull::down);
    In2Pin::pull(b ? PinPull::up : PinPull::down);
    spin_us(hold_us);
}

void quad_home() {
    quad_pos = 0;
    quad_apply(1000);
}

void quad_turn(bool forward, uint32_t steps) {
    for (uint32_t i = 0; i < steps; ++i) {
        quad_pos = static_cast<uint8_t>((quad_pos + (forward ? 1u : 3u)) & 3u);
        quad_apply();
    }
}

void tf_encoder() {
    feed();
    quiet_everything();

    In1Pad::claim_input(PinPull::down);
    In2Pad::claim_input(PinPull::down);
    using Enc = LptimEncoder<L1>;

    struct Mode {
        const char* name;
        LptimClockPolarity sub;
        uint32_t per_four;   // counts table 144 gives per four transitions
    };
    const Mode modes[] = {
        // Table 144, read a column at a time. In sub-mode 1 the ACTIVE
        // edge is a rising one on EITHER input, and a quadrature cycle
        // has exactly two of those; in sub-mode 2 the two falling ones;
        // in sub-mode 3 all four transitions count.
        {"sub-mode 1 (rising edges)", LptimClockPolarity::rising, 2},
        {"sub-mode 2 (falling edges)", LptimClockPolarity::falling, 2},
        {"sub-mode 3 (both edges)", LptimClockPolarity::both, 4},
    };
    constexpr uint32_t steps = 32;   // eight whole quadrature cycles
    bool encoder_ok = true;
    for (const Mode& m : modes) {
        feed();
        L1::init();
        L1::kernel_clock(LptimClock::pclk);
        quad_home();
        const bool up = Enc::setup(0xFFFFu, m.sub, LptimInput1::pad,
                                   LptimInput2::pad, false);
        (void)L1::clear_flags(LptimFlag::all);
        spin_us(1000);
        const uint16_t p0 = Enc::position().value_or(0xFFFFu);
        quad_turn(true, steps);
        const uint16_t p1 = Enc::position().value_or(0xFFFFu);
        quad_turn(false, steps);
        const uint16_t p2 = Enc::position().value_or(0xFFFFu);
        const bool down_flag = Enc::went_down();
        (void)L1::clear_flags(LptimFlag::encoder_only);
        quad_turn(true, steps);
        const uint16_t p3 = Enc::position().value_or(0xFFFFu);
        const bool up_flag = Enc::went_up();
        const uint32_t forward = static_cast<uint32_t>((p1 - p0) & 0xFFFFu);
        const uint32_t back = static_cast<uint32_t>((p1 - p2) & 0xFFFFu);
        const uint32_t again = static_cast<uint32_t>((p3 - p2) & 0xFFFFu);
        const uint32_t want = m.per_four * (steps / 4u);
        print(serial, "  ", m.name, ": ", steps, " transitions forward moved ",
              forward, ", back moved ", back, ", forward again ", again,
              " (table 144 says ", want, " each way); DOWN ", down_flag,
              " then UP ", up_flag, crlf);
        encoder_ok = encoder_ok && up && forward == want && back == want &&
                     again == want && up_flag && down_flag;
    }
    bench.verdict("all three of table 144's encoder sub-modes count a "
                  "quadrature pair walked by two internal pulls, EXACTLY as "
                  "the table says - and the flags mark each CHANGE of "
                  "direction, which is not the same thing as a direction",
                  encoder_ok);

    // The instance rule, both ways.
    bench.verdict("LPTIM2 has no encoder mode (table 135), so a runtime "
                  "configuration asking for it is refused - and the "
                  "compile-time twin is the family negative "
                  "lptim_encoder_on_lptim2.cpp",
                  !L2::configure({.encoder = true}) &&
                      !lptim_config_valid(2, LptimConfig{.encoder = true}) &&
                      !L2::has_encoder && !L2::has_input2);
    bench.verdict("and the chapter's own Caution is enforced: encoder mode "
                  "wants an internal clock and a prescaler of one (26.4.15)",
                  !L1::configure({.prescaler = LptimPrescaler::div4,
                                  .encoder = true}) &&
                      !L1::configure({.clock = LptimClockSource::external_input1,
                                      .encoder = true}));

    quiet_everything();
}

// =============================================================================
// g - through a Stop
// =============================================================================
void tg_through_stop() {
    feed();
    quiet_everything();
    (void)wall_up();
    site_round = false;

    // Each kernel clock in turn: the counter runs free, the machine
    // takes a real 250 ms Stop 1 woken by the RTC, and the counts made
    // across it are weighed against the wall.
    struct Leg {
        const char* name;
        LptimClock code;
        LptimPrescaler presc;
        uint32_t nominal;   // counts per second
        bool expect_running;
    };
    const Leg legs[] = {
        {"LSE", LptimClock::lse, LptimPrescaler::div1, 32'768, true},
        {"LSI", LptimClock::lsi, LptimPrescaler::div1, 32'586, true},
        {"HSI16/128", LptimClock::hsi16, LptimPrescaler::div128, 125'000, false},
        {"PCLK/128", LptimClock::pclk, LptimPrescaler::div128, 500'000, false},
    };
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_wait_ready();
    bool stop_ok = true;
    bool hsi_ran = false;
    for (const Leg& leg : legs) {
        feed();
        if (!free_run(leg.code, leg.presc)) {
            stop_ok = false;
            continue;
        }
        Nvic::clear_pending(Rtc::irq());
        Nvic::enable(Rtc::irq());
        (void)Rtc::set_wakeup(RtcWakeupClock::div16, 512, true);   // 250 ms
        Nvic::clear_pending(Rtc::irq());
        console_drain();
        Ticker::pause();
        const uint16_t c0 = L1::count_raw();
        const uint32_t w0 = wall();
        (void)Pwr::arm(PwrMode::stop1);
        __DSB();
        __WFI();
        (void)PlainSite::resume_clock();
        const uint16_t c1 = L1::count_raw();
        const uint32_t span = wall_ms(wall_delta(w0, wall()));
        Ticker::resume();
        (void)Pwr::arm(PwrMode::sleep);
        Rtc::clear_wakeup();
        Nvic::disable(Rtc::irq());
        const uint32_t counts = static_cast<uint32_t>((c1 - c0) & 0xFFFFu);
        const uint32_t expected =
            span * leg.nominal / 1000u;
        const uint32_t got_hz = span == 0u ? 0u : counts * 1000u / span;
        print(serial, "  ", leg.name, ": ", counts, " counts across a ", span,
              " ms Stop 1 - ", got_hz, " counts a second against ",
              leg.nominal, " running (expected ", expected, ")", crlf);
        if (leg.expect_running) {
            stop_ok = stop_ok && permille_off(counts, expected) < 60u;
        } else if (leg.code == LptimClock::pclk) {
            stop_ok = stop_ok && counts < 16u;
        } else {
            hsi_ran = counts > expected / 10u;
        }
    }
    bench.verdict("26.5's table 145 is exact: on LSE and on LSI the counter is "
                  "UNAFFECTED by a Stop and counts the whole of it, while on "
                  "PCLK it stops dead with the rest of the VCORE domain",
                  stop_ok);
    print(serial, "  HSI16 through a Stop 1: the counter ",
          hsi_ran ? "KEPT RUNNING" : "STOPPED", crlf);
    bench.verdict("AND HSI16 IS NOT A STOP CLOCK FOR A FREE-RUNNING COUNTER: "
                  "5.3 lists the LPTIMs among the peripherals that can REQUEST "
                  "HSI16 in Stop, and a counter that merely COUNTS makes no "
                  "such request - it stops with PCLK. Only LSE and LSI, which "
                  "table 145 names, keep it running; and ES0548 2.2.4 breaks "
                  "clock requests on a divided HSI anyway",
                  !hsi_ran);

    // WHAT A COMPARE MATCH IS, measured before anything rests on it -
    // and it is not a one-shot. 26.7.1 says CMPM "is set by hardware to
    // inform application that LPTIM_CNT register value reached the
    // LPTIM_CMP register's value", which with a counter that reloads is
    // ONCE PER LAP, for ever. Both kernel clocks are run AWAKE, with no
    // Stop anywhere near them, and the count of handler entries is
    // weighed against the number of laps the window holds.
    struct MatchLeg {
        const char* name;
        LptimClock src;
        LptimPrescaler presc;
        uint32_t counter_hz;
        uint32_t window_ms;
    };
    const MatchLeg match_legs[] = {
        {"LSE", LptimClock::lse, LptimPrescaler::div1, 32'768, 300},
        {"PCLK/128", LptimClock::pclk, LptimPrescaler::div128, 500'000, 100},
    };
    bool per_lap_ok = true;
    bool clear_lands = true;
    for (const MatchLeg& leg : match_legs) {
        feed();
        L1::init();
        L1::kernel_clock(leg.src);
        (void)L1::configure({.prescaler = leg.presc,
                             .interrupts = LptimFlag::cmpm});
        L1::enable();
        (void)L1::set_arr(999);
        (void)L1::wait_arr_ok();
        (void)L1::set_cmp(200);
        uint32_t guard = 4'000'000u;
        while (!L1::cmp_ok() && guard-- != 0u) {
        }
        lptim_irqs = 0;
        lptim_served0 = 0;
        lptim_served1 = 0;
        lptim_isr_after0 = 0;
        Nvic::clear_pending(L1::irq());
        Nvic::enable(L1::irq());
        console_drain();
        (void)L1::start_continuous();
        spin_us(leg.window_ms * 1000u);
        Nvic::disable(L1::irq());
        const uint32_t laps =
            leg.window_ms * leg.counter_hz / 1000u / 1000u;
        print(serial, "  awake on ", leg.name, ": in ", leg.window_ms,
              " ms the counter made about ", laps,
              " laps of ARR = 999 and the handler ran ", lptim_irqs,
              " time(s), each serving ", hex(lptim_served0),
              "; LPTIM_ISR read ", hex(lptim_isr_after0),
              " IMMEDIATELY AFTER the ICR store", crlf);
        per_lap_ok = per_lap_ok && lptim_irqs + 1u >= laps &&
                     lptim_irqs <= laps + 1u &&
                     lptim_served0 == LptimFlag::cmpm;
        clear_lands = clear_lands && (lptim_isr_after0 & LptimFlag::cmpm) == 0u;
        L1::init();
    }
    bench.verdict("A COMPARE MATCH IS A PER-LAP EVENT AND NOT A ONE-SHOT: the "
                  "handler runs once for every time the counter passes CMP, on "
                  "both kernel clocks and to within one lap of the window's "
                  "own edges - which is why a compare left standing keeps "
                  "waking a device for ever",
                  per_lap_ok);
    bench.verdict("and the flag clear LANDS AT ONCE even on a 32 kHz kernel "
                  "clock: LPTIM_ISR already reads it gone in the instruction "
                  "after the ICR store, so a handler is entered once per event "
                  "and not once per event plus a spurious re-entry",
                  clear_lands);

    // THE COMPARE WAKE, through EXTI line 29, from both deep rungs.
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        feed();
        const PwrMode mode = leg == 0u ? PwrMode::stop0 : PwrMode::stop1;
        const char* name = leg == 0u ? "Stop 0" : "Stop 1";
        L1::init();
        L1::kernel_clock(LptimClock::lse);
        (void)L1::configure({.prescaler = LptimPrescaler::div32,
                             .interrupts = LptimFlag::cmpm});
        L1::enable();
        (void)L1::set_arr(0xFFFFu);
        (void)L1::wait_arr_ok();
        // THE COMPARE IS PLACED BEFORE THE COUNTER IS STARTED, and that
        // order is not tidiness: CMP comes out of reset at ZERO, so a
        // counter started first matches it on its very first tick and
        // spends the interrupt before the sleep. (It did, and the wake
        // measurement then reported a wall span of nearly a minute -
        // the stamp belonging to a match that happened before the
        // window opened.)
        constexpr uint16_t compare = 512;   // half a second at LSE / 32
        (void)L1::set_cmp(compare);
        // With CMPMIE armed only the handler may clear a flag (ES0548
        // 2.8.2), so the completion is waited for as a flag READ.
        uint32_t guard = 4'000'000u;
        while (!L1::cmp_ok() && guard-- != 0u) {
        }
        (void)L1::wake_line(true);
        lptim_irqs = 0;
        lptim_served0 = 0;
        lptim_served1 = 0;
        Nvic::clear_pending(L1::irq());
        Nvic::enable(L1::irq());
        console_drain();
        Ticker::pause();
        (void)L1::start_continuous();
        const uint16_t at_sleep = 0;
        const uint32_t w0 = wall();
        (void)Pwr::arm(mode);
        __DSB();
        __WFI();
        (void)PlainSite::resume_clock();
        const uint32_t woke_at = lptim_wall;
        const uint32_t span = wall_us(wall_delta(w0, woke_at));
        Ticker::resume();
        (void)Pwr::arm(PwrMode::sleep);
        Nvic::disable(L1::irq());
        const uint32_t distance =
            static_cast<uint32_t>((compare - at_sleep) & 0xFFFFu);
        // The counter is LSE/32 = 1024 counts a second.
        const uint32_t want_us = distance * 1'000'000UL / 1024u;
        print(serial, "  ", name, ": the compare was ", distance,
              " counts ahead when the core stopped, and the match woke it "
              "after ", span, " us of wall against ", want_us,
              " predicted, with ", lptim_irqs, " interrupt(s) - the first "
              "served ", hex(lptim_served0), " and the second ",
              hex(lptim_served1), crlf);
        bench.verdict(name, " is left by an LPTIM compare match through EXTI "
                            "line 29 - a DIRECT line whose pending state is "
                            "the peripheral's own flag - and it lands where "
                            "the counter said it would",
                      lptim_irqs >= 1u && lptim_served0 == LptimFlag::cmpm &&
                          span + 8'000u >= want_us && span <= want_us + 8'000u);
        bench.verdict(name, ": and ONE match is one handler run - the "
                            "compare was placed before the counter started, "
                            "so no unrequested match at count zero spent the "
                            "interrupt before the sleep",
                      lptim_irqs == 1u && lptim_served1 == 0u);
        L1::init();
    }

    quiet_everything();
}

// =============================================================================
// h - the third sleep site
// =============================================================================
void th_site() {
    feed();
    quiet_everything();
    (void)wall_up();
    site_round = true;

    const bool up = Site::init();
    print(serial, "  the LPTIM site ", up ? "initialized" : "REFUSED",
          " on LPTIM", static_cast<uint32_t>(site_cfg.instance),
          ": stated rate ", Site::rate_hz, " Hz, counter ", Site::counter_hz,
          " Hz, one lap is ", Site::span_ticks, " kernel ticks", crlf);
    bench.verdict("the third site comes up on the crystal, with the RTC "
                  "untouched", up);
    if (!up) {
        site_round = false;
        return;
    }

    // The alarm's arithmetic, checked before any sleep.
    K::init_all();
    Probe::clear();
    kernel_live = true;
    Probe::deadline.arm(500u);
    const bool armed = Site::arm(SleepDepth::deep);
    const uint32_t counts = Site::last_counts();
    Site::disarm();
    Probe::deadline.disarm();
    const uint32_t want =
        (500u * Site::counter_hz + P::ticks_per_second - 1u) / P::ticks_per_second +
        1u;
    print(serial, "  a 500 ms deadline placed the compare ", counts,
          " counts ahead; the arithmetic says ", want,
          " (ceil, plus the one count that pays for the phase of the "
          "reading it is measured from)", crlf);
    bench.verdict("the alarm lands where the stated arithmetic puts it",
                  armed && counts == want);

    // NOTHING MAY BE PRINTED BETWEEN arm() AND disarm(): arm() pauses
    // the kernel's tick for a deep rung, so the frozen span the resync
    // hands back is the WHOLE armed window, and a verdict line is four
    // milliseconds of console.
    const bool no_deadline = Site::arm(SleepDepth::deep);
    const bool no_alarm = !Site::alarm_armed();
    Site::disarm();
    const uint32_t napless = Site::last_advance();
    bench.verdict("a deadline-less round places no alarm at all", no_deadline && no_alarm);
    bench.verdict("and a round that never slept advances at most a tick",
                  napless <= 1u);

    // THE ROUND TRIP.
    K::init_all();
    Probe::clear();
    console_drain();
    sync_to_tick();
    const uint32_t w0 = wall();
    const uint32_t k0 = Ticker::millis();
    Probe::deadline.arm(500u);
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip(3000u);
    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ms = Ticker::millis() - k0;
    print(serial, "  a 500 ms event through a Stop matured after ", to_blip,
          " ms of wall (", kernel_ms, " ms of kernel tick); the resync put "
          "back ", Site::last_advance(), " ticks", crlf);
    bench.verdict("THE EVENT MATURED THROUGH A STOP, on the wall, with the "
                  "LPTIM as both the alarm and the witness",
                  Probe::blips == 1u && within(to_blip, 500u, 560u));
    bench.verdict("and NEVER EARLY - the lower bound is the kernel's own "
                  "promise", to_blip >= 500u);
    bench.verdict("the sleep was real: the resync handed back a frozen span of "
                  "hundreds of ticks",
                  Site::last_advance() > 350u && Site::last_advance() < 520u);
    bench.verdict("and the round closed by the convention",
                  Probe::wakes >= 1u && Site::armed() == SleepDepth::none);

    // NEVER EARLY, REPEATED.
    constexpr uint16_t repeats = 6;
    constexpr uint32_t nominal = 150;
    uint16_t on_time = 0;
    uint32_t worst_lo = 0xFFFFFFFFu;
    uint32_t worst_hi = 0;
    for (uint16_t i = 0; i < repeats; ++i) {
        feed();
        K::init_all();
        Probe::clear();
        console_drain();
        sync_to_tick();
        const uint32_t a = wall();
        Probe::deadline.arm(nominal);
        post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
        pump_until_blip(1000u);
        if (Probe::blips != 1u) {
            continue;
        }
        const uint32_t ms = wall_ms(wall_delta(a, Probe::blip_wall));
        if (ms < worst_lo) {
            worst_lo = ms;
        }
        if (ms > worst_hi) {
            worst_hi = ms;
        }
        if (within(ms, nominal, nominal + 20u)) {
            ++on_time;
        }
    }
    print(serial, "  ", repeats, " rounds of ", nominal,
          " ms through a Stop: wall spans ", worst_lo, "..", worst_hi, " ms",
          crlf);
    bench.verdict("every one of six shorter rounds matured inside the band",
                  on_time == repeats);
    bench.verdict("and not one of them was early", worst_lo >= nominal);

    // THE POINT OF THIS SITE: the RTC is the APPLICATION'S. The calendar
    // is put on the chapter's own low-power split (30.3.4's PREDIV_A
    // 127) and an alarm of the application's own is armed - and the site
    // still meets its deadline through a Stop.
    feed();
    const bool split_moved = wall_split(lowpower_prescalers);
    const bool alarm_mine = Rtc::set_alarm(RtcAlarmId::a, RtcAlarm{}, false);
    K::init_all();
    Probe::clear();
    console_drain();
    sync_to_tick();
    const uint32_t lw0 = wall();
    Probe::deadline.arm(300u);
    post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip(3000u);
    const uint32_t low_span = wall_ms(wall_delta(lw0, Probe::blip_wall));
    const uint32_t low_wall_hz = wall_hz();
    const uint32_t low_advance = Site::last_advance();
    const bool alarm_survived = Rtc::alarm_enabled(RtcAlarmId::a);
    const RtcPrescalers left = Rtc::prescalers();
    Rtc::clear_alarm(RtcAlarmId::a);
    kernel_live = false;
    // THE JUDGE IS COARSE HERE AND THE VERDICT SAYS SO: at PREDIV_A 127
    // one sub-second tick is 1/256 s, so this wall reads in steps of
    // about 4 ms and a maturity exactly on the deadline can print as
    // 296. The lower bound is therefore the nominal LESS ONE QUANTUM,
    // and it is the instrument that is being allowed for, not the site -
    // the fine-split rounds above are where "never early" is judged.
    const uint32_t quantum_ms = (1000u + low_wall_hz - 1u) / low_wall_hz;
    print(serial, "  with the RTC at the chapter's own split (PREDIV_A ",
          left.async, " / PREDIV_S ", left.sync,
          ") and ALARM A armed for the application, a 300 ms deadline matured "
          "after ", low_span, " ms of wall; the resync put back ", low_advance,
          " ticks - and this wall's own quantum is ", quantum_ms, " ms", crlf);
    bench.verdict("THE SITE DOES NOT NEED THE RTC: the same deadline is met "
                  "with the calendar at 30.3.4's low-power split - a "
                  "resolution the RTC-backed site refuses at compile time - "
                  "and with an application alarm standing untouched",
                  split_moved && alarm_mine && Probe::blips == 1u &&
                      low_span + quantum_ms >= 300u && low_span <= 340u &&
                      alarm_survived && left.async == 127u && left.sync == 255u);

    (void)wall_up();
    site_round = false;
    quiet_everything();
}

// =============================================================================
// i - the errata
// =============================================================================
void ti_errata() {
    feed();
    quiet_everything();

    // 2.8.2, THE STRUCTURAL HALF. With any interrupt enabled, a flag may
    // be cleared only inside the interrupt routine - so clear_flags()
    // refuses in thread mode and does not when IER is clear.
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    (void)L1::configure({.prescaler = LptimPrescaler::div128});
    L1::enable();
    (void)L1::set_arr(99);
    (void)L1::wait_arr_ok();
    (void)L1::start_continuous();
    spin_us(2000);
    const bool clear_allowed = L1::clear_flags(LptimFlag::arrm);
    const bool cleared = (L1::status() & LptimFlag::arrm) == 0u;

    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    (void)L1::configure({.prescaler = LptimPrescaler::div128,
                         .interrupts = LptimFlag::cmpm});
    L1::enable();
    (void)L1::set_arr(99);
    (void)L1::wait_arr_ok();
    (void)L1::start_continuous();
    spin_us(2000);
    const bool clear_refused = !L1::clear_flags(LptimFlag::arrm);
    const bool still_set = (L1::status() & LptimFlag::arrm) != 0u;
    print(serial, "  clear_flags() with IER = 0: ",
          clear_allowed ? "allowed" : "REFUSED", ", flag ",
          cleared ? "cleared" : "STANDING", "; with CMPMIE set: ",
          clear_refused ? "refused" : "ALLOWED", ", flag ",
          still_set ? "left standing" : "CLEARED", crlf);
    bench.verdict("ES0548 2.8.2 IS CODE: with an interrupt enabled a flag "
                  "clear from thread mode is refused and nothing is written, "
                  "while with IER clear - the erratum's own precondition "
                  "absent - it is an ordinary W1C register again",
                  clear_allowed && cleared && clear_refused && still_set);

    // And the ORDER, which is the other half of the workaround. The ISR
    // body clears the disabled-interrupt flags first and the enabled
    // ones second; what a caller can check is that it SERVED only the
    // enabled ones and swept both.
    const uint32_t before = L1::status();
    const uint32_t served = L1::isr();
    const uint32_t after = L1::status();
    print(serial, "  the ISR body over ISR = ", hex(before), " with IER = ",
          hex(L1::interrupts()), ": served ", hex(served), ", left ",
          hex(after), crlf);
    bench.verdict("...and the ISR body implements the erratum's ORDER: it "
                  "sweeps the flags whose interrupt is disabled FIRST and the "
                  "enabled ones second, and returns only the second group - "
                  "which is what a shared vector answers with",
                  (before & LptimFlag::all) != 0u &&
                      (served & ~L1::interrupts()) == 0u &&
                      (after & before & LptimFlag::all) == 0u);

    // 2.8.1, THE STRUCTURAL HALF: no verb in this driver clears ENABLE,
    // and disable() is the RCC reset the erratum's workaround demands.
    // What a bench CAN show is that the two are distinguishable: an
    // ENABLE clear would leave the configuration standing, and this does
    // not.
    L1::init();
    L1::kernel_clock(LptimClock::pclk);
    (void)L1::configure({.prescaler = LptimPrescaler::div64,
                         .output_inverted = true});
    L1::enable();
    (void)L1::set_arr(1234);
    (void)L1::wait_arr_ok();
    (void)L1::start_continuous();
    const uint32_t cfgr_live = L1::regs().CFGR;
    const uint32_t arr_live = L1::regs().ARR;
    L1::disable();
    const uint32_t cfgr_dead = L1::regs().CFGR;
    const uint32_t arr_dead = L1::regs().ARR;
    const bool still_enabled = L1::enabled();
    print(serial, "  before disable(): CFGR=", hex(cfgr_live), " ARR=",
          arr_live, "; after: CFGR=", hex(cfgr_dead), " ARR=", arr_dead,
          " ENABLE=", still_enabled, crlf);
    bench.verdict("ES0548 2.8.1's workaround IS the disable: disable() takes "
                  "the whole block through RCC_APBRSTR1, so the configuration "
                  "goes with it - which a CR.ENABLE clear, the thing the "
                  "erratum forbids, would not have done",
                  cfgr_live != 0u && arr_live == 1234u && cfgr_dead == 0u &&
                      arr_dead == 1u && !still_enabled);

    print(serial, "  2.8.1 itself is NOT STAGED and no verdict pretends it "
                  "was: reproducing it needs the very ENABLE clear this driver "
                  "has no verb for, its own description calls the occurrence "
                  "\"very low\", and the failure mode - a wake-up signal frozen "
                  "active - would leave the board unable to enter Stop at all",
          crlf);

    quiet_everything();
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_stm32_lptim - the low-power timers and the third sleep site "
          "(board E, no wires)", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

/// LPTIM1's vector, shared with TIM6 and the DAC on this part.
extern "C" void TIM6_DAC_LPTIM1_IRQHandler() {
    if (site_round) {
        Site::isr();
        return;
    }
    const uint32_t n = lptim_irqs;
    if (n == 0u) {
        lptim_wall = wall();
    }
    const uint32_t served = brio::Lptim<1>::isr();
    if (n == 0u) {
        lptim_served0 = served;
        lptim_isr_after0 = brio::Lptim<1>::status();
    } else if (n == 1u) {
        lptim_served1 = served;
    }
    lptim_irqs = n + 1u;
}

extern "C" void RTC_TAMP_IRQHandler() { (void)brio::Rtc::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    brio::Pwr::bus_clock(true);
    brio::Pwr::rtc_domain_unlock(true);
    brio::RtcDomain::apb_clock(true);

    // THE RTC DOMAIN IS NEVER RESET HERE. The wall clock wants the
    // crystal, and RTCSEL is one-way - but the RTC campaign left this
    // board's domain on LSE, and wiping it would cost the backup
    // registers other suites own. So the domain is opened as it stands
    // and the wall reports itself unavailable if it is not the crystal.
    const bool on_lse = brio::RtcDomain::selected() == brio::RtcClockSource::lse;
    brio::RtcDomain::lse_enable(true);
    const bool lse_ok = brio::RtcDomain::lse_wait_ready(4'000'000UL);
    if (on_lse && lse_ok) {
        (void)brio::RtcDomain::open(brio::RtcClockSource::lse);
        brio::Rtc::bypass_shadow(true);
        wall_ready = wall_up();
    }

    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool wd = brio::Iwdg::arm(brio::IwdgConfig{
        .prescaler = brio::IwdgPrescaler::div256,
        .reload = 0x0FFF,
        .window = 0x0FFF});
    brio::enable_interrupts();

    bench.letter('a', "the block, the pads and the four kernel clocks", ta_block);
    bench.letter('b', "counting: prescalers, modes, the two resets, PRELOAD",
                 tb_counting);
    bench.letter('c', "the waveform and its arithmetic, off the pad", tc_waveform);
    bench.letter('d', "counter mode: sampled, clocked, filtered, internal",
                 td_counter_mode);
    bench.letter('e', "triggers: a pad, an RTC alarm, a comparator, a timeout",
                 te_triggers);
    bench.letter('f', "encoder mode on LPTIM1, and LPTIM2 refused", tf_encoder);
    bench.letter('g', "THROUGH STOP: which clocks survive, and the wake",
                 tg_through_stop);
    bench.letter('h', "THE THIRD SLEEP SITE, with the RTC left alone", th_site);
    bench.letter('i', "the two errata, as code and as measurement", ti_errata);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " wall=", wall_ready ? "RTC on LSE" : "NO CRYSTAL",
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
