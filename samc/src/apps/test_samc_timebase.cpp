// build: boards = c21j
// build: monitor_speed = 115200
//
// test_samc_timebase - the standby-surviving timebase, live: the v2
// answer to "kernel time stands still in standby" (samc/sleep.hpp's
// SamTimedSleepSite + samc/ticker.hpp's advance()), proven inside a
// REAL kernel against the board's crystal.
//
// THE CLAIM UNDER TEST, in one sentence: a program with ARMED TIME
// EVENTS may sleep in STANDBY and still meet them - the RTC on
// OSCULP32K is the alarm (a COMP0 wake placed on the next deadline) and
// the witness (the frozen span handed to Ticker::advance()), the power
// MODEL is untouched, and the kernel's own promise holds throughout:
// at least, never early.
//
// THE RULER is the test_samc_sleep instrument: the TC2+TC3 pair as a
// 32-bit counter on generator 2 fed by the 24 MHz crystal, RUNSTDBY
// through the whole chain, counting while the CPU is stopped. Every
// wall-clock claim below is crystal-referenced; the kernel's own
// millis() is the SUBJECT here and never the judge.
//
// THE BANDS, derived and not guessed: the site's default rtc_hz is a
// deliberate OVER-estimate (33500 against an OSCULP32K this bench has
// measured at 32907..33074), and the rule is directional by design -
// the alarm is placed by rounding UP and the resync converts DOWN, so
// every error lands on the LATE side. Concretely: a 500 ms deadline
// becomes ceil(500 x 33.5) = 16750 counts, which the real oscillator
// takes ~509 ms to count; the resync under-advances by the same ratio.
// The verdicts therefore accept [nominal .. nominal + 3.5%] of wall
// and REJECT anything early - the lower bound is the contract.
//
// Letters (all in z):
//  a  the site's own surface: no deadline = no alarm, a deadline = a
//     COMP placed where the arithmetic says, a napless round advances
//     nothing
//  b  THE ROUND TRIP: a 500 ms time event, a standby vote round, the
//     RTC wake, the event matured on the wall - and millis() honest
//  c  a FOREIGN wake (the watchdog's early warning) in the middle: the
//     convention re-requests, the alarm is re-placed for the remainder,
//     the event still matures on time
//  d  NEVER EARLY, repeated: six 150 ms rounds, each judged on the
//     crystal
//
// Wiring: NONE. Board C, console SERCOM5 PB02/PB03 at 115200.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/osc32kctrl.hpp"
#include "samc/platform_sam.hpp"
#include "samc/reset.hpp"
#include "samc/rtc.hpp"
#include "samc/sercom.hpp"
#include "samc/sleep.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;

using P = SamPlatform;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The crystal ruler (the test_samc_sleep instrument, single-role here)
// ---------------------------------------------------------------------------

using Ruler = Tc<2>;                    ///< TC2+TC3 pair, 32-bit
constexpr uint8_t gen_xtal = 2;
constexpr uint32_t crystal_hz = 24'000'000UL;

bool ruler_up() {
    if (!Xosc::init(XoscConfig{.hz = crystal_hz,
                               .startup = 4,
                               .on_demand = false,
                               .run_standby = true})) {
        return false;
    }
    if (!Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::xosc})) {
        return false;
    }
    (void)Ruler::enable(false);
    if (!Ruler::init(gen_xtal)) {
        return false;
    }
    if (!Ruler::configure(TcConfig{.mode = TcMode::count32,
                                   .prescaler = TcPrescaler::div1,
                                   .run_standby = true})) {
        return false;
    }
    return Ruler::enable(true);
}

/// Crystal ticks now. The driver's double-READSYNC makes one call
/// current (samc/tc.hpp).
uint32_t wall() { return Ruler::count32(); }

uint32_t wall_ms(uint32_t ticks) { return ticks / (crystal_hz / 1000u); }

// ---------------------------------------------------------------------------
// The kernel: probe AO + manager over the TIMED site
// ---------------------------------------------------------------------------

using TimedSite = SamTimedSleepSite<P>;

/// The site's default over-estimate, named once for the bands.
constexpr uint32_t rtc_hz_assumed = TimedSleepConfig{}.rtc_hz;   // 33500

struct Blip {};      ///< the deadline's payload
struct Foreign {};   ///< posted by the watchdog early-warning handler

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip, Foreign> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline uint16_t blips = 0;
    static inline uint16_t foreigns = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline bool last_ok = false;
    static inline uint32_t blip_wall = 0;      ///< crystal stamp at Blip
    static inline bool renew_on_foreign = false;

    static void reset_counters() {
        blips = foreigns = wakes = votes = 0;
        last_ok = false;
        blip_wall = 0;
    }

    static void init() { start(&only); }

    static Status only(const Event& e);
};

using Pm_ = PowerManager<P, TimedSite, PowerConfig{}, Probe>;
using K = Kernel<P, Probe, Pm_>;

void request_standby() {
    post<Pm_>(SleepRequested{SleepDepth::standby, reply_to<Probe, SleepVote>()});
}

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
        [](WakeReport) {
            ++wakes;
            return handled();
        },
        [](Blip) {
            ++blips;
            blip_wall = wall();
            // THE CONVENTION (samc/sleep.hpp): after a wake, speak to
            // the manager - here, nothing more to do, so say none.
            post<Pm_>(SleepRequested{SleepDepth::none,
                                     reply_to<Probe, SleepVote>()});
            return handled();
        },
        [](Foreign) {
            ++foreigns;
            if (renew_on_foreign) {
                // The other half of the convention: still waiting on the
                // deadline, so request again - the manager wakes, tells,
                // and runs a fresh round whose arm() re-places the alarm
                // for the REMAINDER.
                request_standby();
            }
            return handled();
        });
}

/// One kernel turn, exactly as Kernel::run() spells it, with an exit.
void pump_until_blips(uint16_t want, uint32_t guard_ms) {
    const uint32_t t0 = wall();
    while (Probe::blips < want &&
           wall_ms(wall() - t0) < guard_ms) {
        TimeEvents<P>::process();
        if (!K::step()) {
            K::idle_if_empty();
        }
    }
    // The exit fires the instant the blip is SERVED; the convention's
    // tail - the {none} request, the manager's wake, the report, the
    // reply - is still queued. Serve it before anyone judges counters.
    while (K::step()) {
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Nothing may print between arming a sleep and the wake: the console's
/// SERCOM stops in standby mid-character.
void console_drain() {
    uint32_t spins = 4'000'000UL;
    while (!Serial::tx_idle() && spins-- != 0u) {
    }
    spins = 4'000'000UL;
    while (!Serial::Resource::txc_flag() && spins-- != 0u) {
    }
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// nominal .. nominal + the over-estimate ratio + slack for the wake
/// and the vote round (a handful of ms).
uint32_t band_hi(uint32_t nominal_ms) {
    return nominal_ms + (nominal_ms * 35u) / 1000u + 6u;
}

void watchdog_backstop(bool on) {
    if (on) {
        (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc4096});
    } else {
        (void)Watchdog::disable();
    }
}

// ---------------------------------------------------------------------------
// a - the site's own surface
// ---------------------------------------------------------------------------

void ta_site() {
    bench.verdict("the crystal ruler runs", ruler_up() && wall() != wall());
    // (wall() != wall(): two synchronized reads of a running counter
    // differ - the ruler's own liveness, in one expression.)

    bench.verdict("the timed site initializes: RTC routed, counting, IRQ up",
                  TimedSite::init() && TimedSite::ready());

    // No deadline: a standby arm places NO alarm, and the round still
    // baselines the resync.
    K::init_all();
    bench.verdict("a deadline-less arm places no alarm",
                  TimedSite::arm(SleepDepth::standby) &&
                      !TimedSite::alarm_armed() &&
                      TimedSite::armed() == SleepDepth::standby);
    TimedSite::disarm();
    bench.verdict("a round that never slept advances nothing",
                  TimedSite::last_advance() == 0u);

    // A deadline: the COMP lands where ceil(ticks x rtc_hz / 1000) says.
    Probe::deadline.arm(500u);
    const uint32_t before = Rtc::count32();
    const bool armed = TimedSite::arm(SleepDepth::standby);
    const bool alarm = TimedSite::alarm_armed();
    const uint32_t placed = Rtc::comp32() - before;   // counts ahead
    TimedSite::disarm();
    Probe::deadline.disarm();
    const uint32_t expect = (500u * rtc_hz_assumed + 999u) / 1000u;
    print(serial, "  alarm placed ", placed, " RTC counts ahead, expected ",
          expect, " (ceil of 500 ms x ", rtc_hz_assumed, " Hz)", crlf);
    bench.verdict("a deadline places the alarm by the stated arithmetic",
                  armed && alarm && within(placed, expect, expect + 40u));
    bench.verdict("and disarm() takes it back down",
                  !TimedSite::alarm_armed());
}

// ---------------------------------------------------------------------------
// b - the round trip
// ---------------------------------------------------------------------------

void tb_round_trip() {
    K::init_all();
    Probe::reset_counters();
    Probe::renew_on_foreign = false;

    const uint32_t wall0 = wall();
    const uint32_t mill0 = Ticker::millis();

    Probe::deadline.arm(500u);
    request_standby();
    console_drain();
    watchdog_backstop(true);
    pump_until_blips(1, 900u);
    watchdog_backstop(false);

    const uint32_t to_blip = wall_ms(Probe::blip_wall - wall0);
    const uint32_t wall_span = wall_ms(wall() - wall0);
    const uint32_t mill_span = Ticker::millis() - mill0;
    print(serial, "  blip after ", to_blip, " ms of wall (band 500..",
          band_hi(500u), "); the round advanced the tick by ",
          TimedSite::last_advance(), "; millis moved ", mill_span,
          " over ", wall_span, " ms of wall", crlf);

    bench.verdict("THE EVENT MATURED THROUGH A STANDBY, on the wall and "
                  "never early",
                  Probe::blips == 1u &&
                      within(to_blip, 500u, band_hi(500u)));
    bench.verdict("the sleep was real: the resync put back a frozen span "
                  "of hundreds of ticks",
                  TimedSite::last_advance() > 350u &&
                      TimedSite::last_advance() < 520u);
    bench.verdict("millis() is honest against the crystal: no more than "
                  "the wall, and behind it only by the stated bias",
                  mill_span <= wall_span + 1u &&
                      mill_span + (wall_span * 25u) / 1000u + 3u >= wall_span);
    bench.verdict("the round closed by the convention: a wake report, an "
                  "ok vote, the alarm down",
                  Probe::wakes >= 1u && Probe::last_ok &&
                      !TimedSite::alarm_armed());
}

// ---------------------------------------------------------------------------
// c - a foreign wake in the middle
// ---------------------------------------------------------------------------

void tc_foreign() {
    K::init_all();
    Probe::reset_counters();
    Probe::renew_on_foreign = true;

    // The intruder: the watchdog's early warning, ~125 ms in - a proven
    // standby wake source (test_samc_sleep letter h) that posts to its
    // own AO like any real interrupt.
    (void)Watchdog::disable();
    (void)Watchdog::arm(WdtConfig{.period = WdtCycles::cyc1024,
                                  .early_warning = true,
                                  .ew_offset = WdtCycles::cyc128});
    Nvic::enable(Watchdog::irq());

    const uint32_t wall0 = wall();
    Probe::deadline.arm(500u);
    request_standby();
    console_drain();
    pump_until_blips(1, 900u);

    Nvic::disable(Watchdog::irq());
    (void)Watchdog::disable();

    const uint32_t to_blip = wall_ms(Probe::blip_wall - wall0);
    print(serial, "  ", Probe::foreigns, " foreign wake(s); blip after ",
          to_blip, " ms of wall (band 500..", band_hi(500u) + 6u, "); ",
          Probe::wakes, " wake reports", crlf);

    bench.verdict("the intruder woke the machine mid-sleep",
                  Probe::foreigns >= 1u);
    bench.verdict("the convention re-requested and the alarm was re-placed "
                  "for the remainder: TWO rounds ended",
                  Probe::wakes >= 2u);
    bench.verdict("AND THE DEADLINE STILL MATURED ON THE WALL, never early",
                  Probe::blips == 1u &&
                      within(to_blip, 500u, band_hi(500u) + 6u));
}

// ---------------------------------------------------------------------------
// d - never early, repeated
// ---------------------------------------------------------------------------

void td_never_early() {
    K::init_all();
    Probe::renew_on_foreign = false;

    constexpr uint16_t rounds = 6;
    constexpr uint32_t nominal = 150;
    uint16_t on_time = 0;
    uint32_t worst_lo = UINT32_MAX;
    uint32_t worst_hi = 0;

    watchdog_backstop(true);
    for (uint16_t r = 0; r < rounds; ++r) {
        Probe::reset_counters();
        const uint32_t wall0 = wall();
        Probe::deadline.arm(nominal);
        request_standby();
        console_drain();
        pump_until_blips(1, 400u);
        if (Probe::blips != 1u) {
            continue;
        }
        const uint32_t ms = wall_ms(Probe::blip_wall - wall0);
        if (ms < worst_lo) {
            worst_lo = ms;
        }
        if (ms > worst_hi) {
            worst_hi = ms;
        }
        if (within(ms, nominal, band_hi(nominal))) {
            ++on_time;
        }
    }
    watchdog_backstop(false);

    print(serial, "  ", rounds, " rounds of ", nominal,
          " ms through standby: wall spans ", worst_lo, "..", worst_hi,
          " ms (band ", nominal, "..", band_hi(nominal), ")", crlf);
    bench.verdict("EVERY round matured inside the band",
                  on_time == rounds);
    bench.verdict("and not one was early - the lower bound is the kernel's "
                  "own promise",
                  worst_lo >= nominal);
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_samc_timebase - the standby-surviving timebase "
          "(board C, no wires)", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }
extern "C" void RTC_Handler() { TimedSite::isr(); }
extern "C" void WDT_Handler() {
    brio::Watchdog::clear_flags();
    brio::post<Probe>(Foreign{});
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)brio::Watchdog::disable();
    brio::enable_interrupts();

    bench.letter('a', "the site's surface: alarm arithmetic, napless rounds",
                 ta_site);
    bench.letter('b', "THE ROUND TRIP: a 500 ms event through a standby",
                 tb_round_trip);
    bench.letter('c', "a foreign wake mid-sleep, the alarm re-placed",
                 tc_foreign);
    bench.letter('d', "never early: six rounds on the crystal", td_never_early);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
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
