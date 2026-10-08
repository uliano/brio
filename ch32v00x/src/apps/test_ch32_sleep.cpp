// test_ch32_sleep - the reference bench suite for the CH32V00x's PWR
// chapter and the sleep sites over it: the auto-wakeup unit as an alarm
// (and the LSI it counts, measured against the STK), Sleep and Standby
// through util/power.hpp's site contract, the clock put back after a
// Standby, kernel time handed the frozen span.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE for `z`. What a Standby costs in microamps is the
// bench meter's, with the probe detached (a core in debug mode never
// sleeps): this suite proves the modes are entered and left and what
// they do to the clock and the kernel's time, not what they draw.
//
// What is exercised, letter by letter:
//   a  PWR as found: Sleep armed by default, the regulator in normal
//      mode, the PVD off; the AWU's arithmetic
//   b  the AWU awake: the LSI on, a window timed on the STK, the rate
//      that comes out (and its distance from the nominal 128 kHz),
//      then a second window at a prescaled grain checked against it
//   c  Sleep through the plain site: armed light, idle() sleeps and the
//      tick brings it back, the ladder's mapping (standby and deep both
//      land on Standby - the deepest this silicon has)
//   d  STANDBY THROUGH THE TIMED SITE: a time event armed 300 ms out,
//      the site places the AWU, idle() enters Standby, the AWU ends it,
//      disarm() restores the PLL and advances the tick by the span -
//      and the event is due, not early. The first line after the wake
//      is printed only once the clock is back.
//   e  a Standby that something ELSE ends: the AWU armed for a long
//      window while the console's own byte is the wake - the site
//      advances by nothing, honestly, and says so
//   w  THE AWU'S EDGE AGAINST THE SLEEP ENTRY: the AWU free-running at
//      about 64 us (256 us on the CH32V003, whose Sleep lasts at least
//      66 us), each try locked to one match by polling its flag and
//      the next one placed D cycles after the wait ends, D = 0..399 -
//      the edge so walks across the kernel's masked check, the sleep
//      entry and the sleep itself, the LSI's own jitter spreading each D
//      over some forty cycles. Four ways line 9 can reach the core:
//      in a Sleep (the tick held off by hand, TIM2 timing each try and
//      rescuing a lost one at 4 ms) the event with the PFIC-disabled
//      interrupt line (the timed site's old shape), that line's pending
//      edge alone (the control: it must lose), and the line enabled in
//      the PFIC (the site's shape); and in a Standby through the plain
//      site, the clock put back after each try, the enabled line ALONE,
//      no event to fall back on - no counter runs in a Standby, so a
//      lost wake there would be a try that never returns. Then fifty
//      real Standbys through the timed site, each a 3 ms deadline.
//      THE NET: the letter arms the IWDG (256 ms), fed before every try
//      and by the tick, so a sleep entry that wedges the core reboots the
//      board instead of needing a hand (the tally then never comes).
//
// build: boards = v006k8,v003f4
// build: groups = abcde,w
// (on the CH32V003 the suite is two images, test_ch32_sleep-1 and -2:
// whole it leaves the part's 15 KB under 2 per cent)
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/reset.hpp"
#include "ch32v00x/sleep.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;

using Plain = Ch32SleepSite<SysClock>;
using Timed = Ch32TimedSleepSite<P, SysClock>;

TestBench<Serial> bench;

// A time event to have something armed: the AO it posts to is never
// dispatched here, its queue is read by hand.
struct Sleeper {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Sleeper, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

uint32_t cycles_now() { return stk()->CNT; }

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 300);
}

// Time one AWU window on the STK, awake. Returns cycles, 0 on a timeout.
uint32_t time_window(uint8_t prescaler, uint8_t window) {
    Awu::arm(prescaler, window);
    const uint32_t c0 = cycles_now();
    const uint32_t t0 = Ticker::ticks();
    uint32_t polls = 0;
    while (!Awu::fired() && polls < 20'000'000u) {
        ++polls;
    }
    const uint32_t ticks = Ticker::ticks() - t0;
    const uint32_t c1 = cycles_now();
    Awu::disarm();
    if (polls >= 20'000'000u) {
        return 0;
    }
    // Whole ticks plus the phase: the STK reloads every ms, so the
    // elapsed cycles are the reloads counted by the tick and the signed
    // difference of the two phases.
    const int64_t total = static_cast<int64_t>(ticks) * (stk()->CMP + 1u) +
                          static_cast<int64_t>(c1) - static_cast<int64_t>(c0);
    return total > 0 ? static_cast<uint32_t>(total) : 0u;
}

// ---------------------------------------------------------------------------
// a - as found
// ---------------------------------------------------------------------------
void ta_found() {
    Pwr::open();
    print(serial, "  PWR CTLR=", hex(pwr()->CTLR), " CSR=", hex(pwr()->CSR),
          " SCTLR=", hex(pfic_sctlr()), crlf);
    bench.verdict("Sleep is what a deep instruction would give as found (PDDS clear)",
                  !Pwr::standby());
    bench.verdict("SLEEPDEEP is clear as found", (pfic_sctlr() & sctlr_sleepdeep) == 0u);
    if constexpr (device::pwr_has_ldo_modes) {
        bench.verdict("the regulator is in normal mode (1.2 V)", Pwr::ldo() == pwr_ldo_normal);
    } else {
        print(serial, "  (no LDO_MODE on this part: the regulator has one mode)", crlf);
    }
    bench.verdict("the PVD is off", !Pwr::pvd());
    // THE PVD LADDER: every level of the part from the lowest up, the
    // flag read at each - the supply is bracketed between the last
    // level that reads not low and the first that reads low (a 3.3 V
    // board never reads low on the CH32V006, whose ladder ends at
    // 2.66 V; on the CH32V003 it falls between 3.05 and 3.5 V).
    bool monotonic = true;
    bool low_seen = false;
    uint32_t last_not_low_mv = 0;
    uint32_t first_low_mv = 0;
    for (uint8_t code = 0; code <= static_cast<uint8_t>(pvd_level_highest); ++code) {
        const PvdLevel level = static_cast<PvdLevel>(code);
        Pwr::pvd(true, level);
        (void)delay_us(clock, 100);
        const bool low = Pwr::supply_low();
        if (low) {
            if (!low_seen) { first_low_mv = pvd_rising_mv(level); }
            low_seen = true;
        } else {
            if (low_seen) { monotonic = false; }
            last_not_low_mv = pvd_rising_mv(level);
        }
    }
    Pwr::pvd(false);
    print(serial, "  PVD ladder: not low up to ", last_not_low_mv, " mV rising");
    if (low_seen) {
        print(serial, ", low from ", first_low_mv, " mV rising", crlf);
    } else {
        print(serial, ", never low (the ladder ends under the supply)", crlf);
    }
    bench.verdict("the PVD ladder: the supply reads NOT low at the part's lowest threshold, and once low "
                  "stays low up the ladder",
                  monotonic && last_not_low_mv > 0u);
    bench.verdict("the AWU's longest period at the nominal LSI is 2.048 s (window 63, /4096)",
                  Awu::period_us(13, 63, 128'000) == 2'048'000u);
    bench.verdict("the AWU is off", !Awu::enabled());
}

// ---------------------------------------------------------------------------
// b - the AWU awake, and the LSI measured
// ---------------------------------------------------------------------------
void tb_awu() {
    bench.verdict("the AWU initializes (the LSI came ready)", Awu::init());

    // 64 undivided counts: about 0.5 ms at 128 kHz.
    const uint32_t cycles = time_window(0, 63);
    const uint32_t lsi_hz = cycles == 0u ? 0u
        : static_cast<uint32_t>((static_cast<uint64_t>(SysClock::hz) * 64u) / cycles);
    print(serial, "  64 LSI counts = ", cycles, " HCLK cycles -> LSI = ", lsi_hz, " Hz (",
          lsi_hz > 128'000u ? "+" : "-",
          (lsi_hz > 128'000u ? lsi_hz - 128'000u : 128'000u - lsi_hz) * 100u / 128'000u,
          "% of nominal)", crlf);
    bench.verdict("the window fires while awake, within the poll budget", cycles != 0u);
    bench.verdict("the LSI measures within +-40% of 128 kHz",
                  lsi_hz > 77'000u && lsi_hz < 180'000u);

    // The same rate through a prescaler: /128 x 8 counts should take
    // 1024/64 = 16 times the first window.
    const uint32_t cycles2 = time_window(8, 7);
    print(serial, "  8 counts at /128 = ", cycles2, " cycles, expected ", cycles * 16u, crlf);
    bench.verdict("a prescaled window scales as the divider says (within 5%)",
                  cycles2 != 0u && cycles2 > cycles * 16u * 95u / 100u &&
                      cycles2 < cycles * 16u * 105u / 100u);

    bench.verdict("the timed site's own init measures the same rate (within 2%)",
                  Timed::init() &&
                      (Timed::lsi_hz() > lsi_hz * 98u / 100u && Timed::lsi_hz() < lsi_hz * 102u / 100u));
    print(serial, "  site's LSI = ", Timed::lsi_hz(), " Hz", crlf);
}

// ---------------------------------------------------------------------------
// c - Sleep through the plain site
// ---------------------------------------------------------------------------
void tc_sleep() {
    bench.verdict("the ladder: light is Sleep", Plain::arm(SleepDepth::light) && Plain::armed() == SleepDepth::light);
    bench.verdict("standby is Standby", Plain::arm(SleepDepth::standby) && Plain::armed() == SleepDepth::standby);
    bench.verdict("deep is Standby too - the deepest this silicon has",
                  Plain::arm(SleepDepth::deep) && Plain::armed() == SleepDepth::standby);
    Plain::disarm();
    bench.verdict("disarmed: none, and the clock still the PLL",
                  Plain::armed() == SleepDepth::none && Rcc::sysclk_source() == rcc_sws_pll);

    (void)Plain::arm(SleepDepth::light);
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        {
            P::CriticalSection cs;
            P::idle();
        }
        ++calls;
    }
    Plain::disarm();
    print(serial, "  armed light: ", calls, " idle() call(s) to the next tick", crlf);
    bench.verdict("a light sleep is ended by the tick, inside one tick", calls >= 1u && calls < 20u);
    bench.verdict("and the tick went on counting through it", Ticker::ticks() > t1);
}

// ---------------------------------------------------------------------------
// d - Standby through the timed site, ended by the AWU
// ---------------------------------------------------------------------------
void td_standby() {
    if (!Timed::init()) {
        bench.verdict("the timed site initializes", false);
        return;
    }
    Sleeper::alarm.arm(ticks_from_ms<P>(300));
    const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
    bench.verdict("a time event 300 ms out is the nearest deadline", next && *next == 300u);

    // The deadline as the site will see it, a few milliseconds of
    // printing after the event was armed.
    const uint32_t remaining = TimeEvents<P>::ticks_to_next().value_or(0u);
    const uint32_t t_before = Ticker::ticks();
    bench.verdict("arm(deep) takes and places the AWU",
                  Timed::arm(SleepDepth::deep) && Timed::armed() == SleepDepth::standby && Awu::enabled());
    print(serial, "  placed: LSI ", Timed::lsi_hz(), " Hz, prescaler code ", Timed::prescaler_code(),
          ", window ", Timed::window(), " -> ", Timed::span_ticks(), " ticks; entering Standby...", crlf);
    console_drain();
    // The loop's shape: a stale latched event ends the first WFE at
    // once and the loop sleeps again; the AWU's wake is the one that
    // sets its flag.
    uint8_t turns = 0;
    while (!Awu::fired() && turns < 20u) {
        P::CriticalSection cs;
        P::idle();       // the WFE: Standby, until the AWU
        ++turns;
    }
    // Woken on the HSI. Nothing printed until the tree is back.
    const bool fired = Awu::fired();
    const uint32_t sws_at_wake = Rcc::sysclk_source();
    const uint32_t hpre_at_wake = Rcc::hpre_code();
    Timed::disarm();     // the clock restored, the tick advanced
    const uint32_t t_after = Ticker::ticks();
    const uint32_t advanced = Timed::last_advance();
    print(serial, "  woke after ", turns, " idle() turn(s): AWU fired=", fired,
          " SYSCLK at wake=", hex(sws_at_wake), " HPRE=", hpre_at_wake,
          " -> restored SWS=", hex(Rcc::sysclk_source()), "; tick ", t_before, " -> ",
          t_after, " (advanced by ", advanced, ")", crlf);
    bench.verdict("the AWU is what ended the Standby", fired);
    bench.verdict("the core woke on the HSI (the PLL was off)", sws_at_wake == rcc_sws_hsi);
    bench.verdict("disarm() put the PLL back", Rcc::sysclk_source() == rcc_sws_pll);
    bench.verdict("kernel time was advanced by the FROZEN span (the alarm's span "
                  "less the ticks counted awake), and arm to here covers the "
                  "deadline the site was given - late, never early",
                  advanced > 0u && t_after - t_before >= remaining &&
                      t_after - t_before < remaining + 100u);
    bench.verdict("and the time event is due (never early)",
                  !TimeEvents<P>::ticks_to_next().has_value() ||
                      *TimeEvents<P>::ticks_to_next() == 0u);
    TimeEvents<P>::process();
    bench.verdict("processing matures it", Sleeper::queue.pop().has_value());
    Sleeper::alarm.disarm();
}

// ---------------------------------------------------------------------------
// e - a Standby ended by something else
// ---------------------------------------------------------------------------
void te_other_wake() {
    if (!Timed::init()) {
        bench.verdict("the timed site initializes", false);
        return;
    }
    // Two seconds out: the AWU would end it at 2 s; the console's next
    // byte - the USART's RX interrupt - is what ends it instead. But
    // USART1 stops in Standby with every other peripheral clock, so the
    // wake has to come through an EXTI pad... which this bench has no
    // wire for. So this letter stops short: it arms, checks what was
    // placed, and disarms WITHOUT sleeping - proving the accounting's
    // other branch (nothing fired, nothing advanced) and no more.
    Sleeper::alarm.arm(ticks_from_secs<P>(2));
    (void)Timed::arm(SleepDepth::deep);
    const bool placed = Awu::enabled();
    Timed::disarm();
    print(serial, "  armed for 2 s, disarmed without sleeping: advanced by ",
          Timed::last_advance(), crlf);
    bench.verdict("the AWU was placed for the 2 s deadline", placed);
    bench.verdict("a disarm with no alarm fired advances by nothing", Timed::last_advance() == 0u);
    bench.verdict("and the clock is untouched (no Standby ran)", Rcc::sysclk_source() == rcc_sws_pll);
    Sleeper::alarm.disarm();
}

// ---------------------------------------------------------------------------
// w - the AWU's edge against the sleep entry, placed to the cycle
// ---------------------------------------------------------------------------
using Rescue = Tim<2>;
volatile uint32_t rescue_periods = 0;
volatile uint32_t awu_entries = 0;
volatile bool net_armed = false;   ///< letter w's IWDG: fed by every try and by the tick

/// How line 9 reaches the core: the timed site's old shape (an EXTI event
/// and a PFIC-disabled interrupt line, SEVONPEND's pending edge), that
/// pending edge alone (the instrument's control), the site's shape now
/// (the line enabled in the PFIC - a level - with the event kept), and
/// the level alone (no event to fall back on: in a Standby, where no
/// counter runs to time a late wake, a lost level would be a hang).
enum class AwuPath : uint8_t { event_and_pending, pending_only, level, level_alone };

struct SweepResult {
    uint32_t tries;
    uint32_t slept;      ///< tries that reached the sleep (idle() called)
    uint32_t lost;       ///< woken a whole AWU period late, or rescued
    uint32_t first_lost;
    uint32_t last_lost;
    uint32_t rescued;    ///< of the lost, those no later match woke (TIM2 did)
    uint32_t entries;    ///< AWU handler entries (the level paths)
};

/// Cycles from `from` to `to` on the STK, one reload folded in.
uint32_t fold(uint32_t from, uint32_t to, uint32_t period) {
    return to >= from ? to - from : to + period - from;
}

/// One pass of D = 0..positions-1, `repeats` tries each: lock to an AWU
/// match, wait so the next one lands D cycles after the wait ends, then
/// the kernel's shape - a masked check of the flag, idle() - with the
/// sleep a Sleep (the tick held off by hand) or, `standby`, a Standby
/// armed through the plain site and the clock put back after each try.
SweepResult awu_sweep(AwuPath path, bool standby, uint32_t awu_period, uint32_t awu_period_us,
                      uint32_t positions, uint32_t repeats) {
    const bool level = path == AwuPath::level || path == AwuPath::level_alone;
    Exti::event(exti_line_awu, path == AwuPath::event_and_pending || path == AwuPath::level);
    Exti::interrupt(exti_line_awu, true);
    Pfic::disable(Irq::awu);
    SweepResult r{0, 0, 0, 0, 0, 0, 0};
    const uint32_t entries0 = awu_entries;
    const uint32_t period = stk()->CMP + 1u;
    for (uint32_t d = 0; d < positions; ++d) {
        for (uint32_t k = 0; k < repeats; ++k) {
            Iwdg::refresh();
            if (standby) {
                (void)Plain::arm(SleepDepth::standby);
            }
            bool rescued = false;
            uint32_t took_us = 0;
            uint32_t calls = 0;
            {
                P::CriticalSection cs;
                Awu::clear();
                while (!Awu::fired()) {
                }
                const uint32_t t0 = stk()->CNT;
                Awu::clear();
                if (level) {
                    Pfic::enable(Irq::awu);   // the site's arm(): cleared, then enabled
                }
                const uint32_t wait = awu_period - d;
                while (fold(t0, stk()->CNT, period) < wait) {
                }
                const uint32_t r0 = rescue_periods;
                Rescue::set_count(0);
                for (;;) {
                    P::CriticalSection turn;   // the kernel's own shape
                    if (Awu::fired()) {
                        break;
                    }
                    P::idle();
                    ++calls;
                }
                disable_interrupts();
                took_us = Rescue::count();
                rescued = rescue_periods != r0;
                Pfic::disable(Irq::awu);
                Awu::clear();
            }
            if (standby) {
                Plain::disarm();   // the PLL back before the next lock
            }
            ++r.tries;
            if (calls != 0u) {
                ++r.slept;
            }
            // On time, the loop ends within a few us of the match; a lost
            // edge waits for a later one or for the rescue. In a Standby
            // the counters stop, so only a hang would show there.
            if (!standby && (rescued || took_us > awu_period_us / 2u + 10u)) {
                if (r.lost == 0u) {
                    r.first_lost = d;
                }
                r.last_lost = d;
                ++r.lost;
                if (rescued) {
                    ++r.rescued;
                }
            }
        }
    }
    r.entries = awu_entries - entries0;
    Exti::event(exti_line_awu, true);
    return r;
}

void print_sweep(const char* name, const SweepResult& r) {
    print(serial, "  ", name, ": ", r.tries, " tries, ", r.slept, " slept, lost ", r.lost);
    if (r.lost != 0u) {
        print(serial, " (D=", r.first_lost, "..", r.last_lost, ", ", r.rescued, " of them woken only by the rescue)");
    }
    print(serial, ", handler entries ", r.entries, crlf);
}

void tw_awu_edge() {
    if (!net_armed) {
        // THE NET: /32 and a reload of 999, 256 ms at the 125 kHz LSI -
        // a sleep entry that wedges the core (measured on the CH32V003's
        // V2A, docs/ch32v00x/platform.md) reboots the board instead of
        // needing a hand. Fed before every try and by the tick; never
        // stopped again.
        (void)Iwdg::arm(IwdgConfig{.prescaler = IwdgPrescaler::div32, .reload = 999});
        net_armed = true;
    }
    if (!Awu::init()) {
        bench.verdict("the AWU initializes", false);
        return;
    }
    // The AWU free-running at 8 undivided LSI counts (about 64 us): it
    // matches again every period while enabled, which is what the sweep
    // locks to. Its period over 32 matches is the lock's spread. On the
    // CH32V003 32 counts (about 256 us): a Sleep there lasts at least
    // 66 us (platform.hpp), so a wake on time and one a period late are
    // told apart only by a period longer than that.
    Awu::arm(0, device::sleep_entry_from_sram ? 31 : 7);
    const uint32_t period = stk()->CMP + 1u;
    uint32_t p_min = 0xFFFF'FFFFu, p_max = 0;
    {
        P::CriticalSection cs;
        Awu::clear();
        while (!Awu::fired()) {
        }
        uint32_t last = stk()->CNT;
        Awu::clear();
        for (int i = 0; i < 32; ++i) {
            while (!Awu::fired()) {
            }
            const uint32_t now = stk()->CNT;
            Awu::clear();
            const uint32_t p = fold(last, now, period);
            last = now;
            p_min = p < p_min ? p : p_min;
            p_max = p > p_max ? p : p_max;
        }
    }
    const uint32_t awu_period = (p_min + p_max) / 2u;
    const uint32_t awu_period_us = awu_period / (SysClock::hz / 1'000'000u);
    print(serial, "  the AWU re-matches every ", p_min, "..", p_max, " cycles (", awu_period_us, " us)", crlf);
    bench.verdict("the AWU matches again every period while enabled, within 2% of itself",
                  p_min > 0u && p_max < p_min + p_min / 50u);

    Rescue::init();
    (void)Rescue::configure(TimConfig{.prescaler = static_cast<uint16_t>(SysClock::hz / 1'000'000u - 1u),
                                      .period = 3'999u});
    Rescue::interrupts(Rescue::update_interrupt, true);
    Pfic::enable(Irq::tim2);
    Rescue::enable(true);
    console_drain();

    // Sleep: the counters run through it, so a late wake is timed. The
    // tick is held off by hand, as idle() holds it off for a Standby.
    // Each sweep's line printed as it ends, so that a sweep the net
    // cuts short is named by the last line before the reboot.
    Ticker::pause();
    const SweepResult a = awu_sweep(AwuPath::event_and_pending, false, awu_period, awu_period_us, 400, 16);
    Ticker::resume();
    print_sweep("Sleep, event + pending edge (the site's old shape)", a);
    console_drain();
    Ticker::pause();
    const SweepResult b = awu_sweep(AwuPath::pending_only, false, awu_period, awu_period_us, 400, 16);
    Ticker::resume();
    print_sweep("Sleep, pending edge alone", b);
    console_drain();
    Ticker::pause();
    const SweepResult c = awu_sweep(AwuPath::level, false, awu_period, awu_period_us, 400, 16);
    Ticker::resume();
    print_sweep("Sleep, the line enabled (the site's shape)", c);
    console_drain();
    // Standby: the entry is the same instructions with SLEEPDEEP armed;
    // the clock stops at the WFE, and the level is alone.
    const SweepResult s = awu_sweep(AwuPath::level_alone, true, awu_period, awu_period_us, 400, 32);
    Awu::disarm();

    Rescue::enable(false);
    Rescue::interrupts(Rescue::update_interrupt, false);
    Pfic::disable(Irq::tim2);
    Rescue::release();

    print_sweep("Standby, the line enabled and no event", s);
    bench.verdict("each sweep crosses the sleep entry (some tries end at the check, some asleep)",
                  b.slept > 0u && b.slept < b.tries && c.slept > 0u && c.slept < c.tries &&
                      s.slept > 0u && s.slept < s.tries);
    bench.verdict("the instrument reaches the hole: in a Sleep, SEVONPEND's pending edge alone "
                  "loses positions",
                  b.lost > 0u);
    bench.verdict("the AWU's wake as the timed site arms it (its line enabled, a level) loses none",
                  c.lost == 0u);
    bench.verdict("its handler runs once per wake, never again on its own, in a Sleep and in a "
                  "Standby - where no try was left asleep",
                  c.entries == c.slept && s.entries == s.slept);

    // Real Standbys through the timed site, one after the other, each a
    // time event 3 ms out and the AWU the alarm.
    if (!Timed::init()) {
        bench.verdict("the timed site initializes", false);
        return;
    }
    console_drain();
    constexpr uint32_t standbys = 50;
    uint32_t by_alarm = 0, most_turns = 0, line_wrong = 0;
    const uint32_t entries0 = awu_entries;
    for (uint32_t i = 0; i < standbys; ++i) {
        Iwdg::refresh();
        Sleeper::alarm.arm(3);
        (void)Timed::arm(SleepDepth::deep);
        if (!Pfic::enabled(Irq::awu)) {
            ++line_wrong;
        }
        uint8_t turns = 0;
        while (!Awu::fired() && turns < 20u) {
            P::CriticalSection cs;
            P::idle();
            ++turns;
        }
        if (Awu::fired()) {
            ++by_alarm;
        }
        most_turns = turns > most_turns ? turns : most_turns;
        Timed::disarm();
        if (Pfic::enabled(Irq::awu)) {
            ++line_wrong;
        }
        Sleeper::alarm.disarm();
        while (Sleeper::queue.pop().has_value()) {
        }
    }
    const uint32_t entries = awu_entries - entries0;
    print(serial, "  ", standbys, " Standbys through the timed site: ", by_alarm, " ended by the AWU, ",
          entries, " handler entries, at most ", most_turns, " idle() calls each", crlf);
    bench.verdict("every Standby is ended by the AWU, its handler run once each",
                  by_alarm == standbys && entries == standbys);
    bench.verdict("arm() opened the AWU's line and disarm() closed it, every time", line_wrong == 0u);
}

void banner() {
    print(serial, crlf, "test_ch32_sleep - ", device::part_name, " (Sleep, Standby, the AWU on the LSI)", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() {
    brio::Ticker::tick();
    if (net_armed) {
        brio::Iwdg::refresh();
    }
}
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

/// The timed site's AWU wake is a level: its line enabled, this vector
/// bound to the site's body (counted here for letter w).
extern "C" BRIO_CH32_INTERRUPT void awu_handler() {
    awu_entries = awu_entries + 1u;
    Timed::awu_isr();
}

/// Letter w's rescue: TIM2's update, counted.
extern "C" BRIO_CH32_INTERRUPT void tim2_handler() {
    Rescue::clear_flags(Rescue::update_flag);
    rescue_periods = rescue_periods + 1u;
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "PWR as found, and the AWU's arithmetic", ta_found);
    bench.letter('b', "the AWU awake, the LSI measured", tb_awu);
    bench.letter('c', "Sleep through the plain site", tc_sleep);
    bench.letter('d', "STANDBY through the timed site, ended by the AWU", td_standby);
    bench.letter('e', "a Standby armed and disarmed unslept", te_other_wake);
    bench.letter('w', "the AWU's edge against the sleep entry, placed to the cycle", tw_awu_edge);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
