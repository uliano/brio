// test_rp2040_sleep - the reference bench suite for the RP2040's power
// chapter (datasheet 2.11) and the sleep sites over it: the SLEEP state
// and its clock gates, DORMANT on either oscillator, the system timer
// as alarm and witness, the calendar as the wake of a dormant, the
// power manager over the timed site.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE for `z`. What a state costs in microamps is the bench
// meter's, with the probe detached: this suite proves the states are
// entered and left, what they stop and what they do to kernel time.
//
// THE RULERS: the system timer (its gates kept open through every
// standby), the PWM's counter as the instrument that says whether a
// clock gate closed, the calendar for a dormant that stops the timer.
//
// What is exercised, letter by letter:
//   a  as found: every gate open in both masks, SLEEPDEEP clear, no
//      dormant wake enabled, the idle path's hook empty; the ladder
//      through the plain site (light = WFI, standby = SLEEPDEEP, deep
//      REFUSED with no way back, taken once a GPIO wake is enabled);
//      the named gate sets' arithmetic
//   b  a light sleep through the plain site: idle() ended by the tick,
//      the tick counting through
//   c  THE INSTRUMENT: a PWM slice counting clk_sys with its gate
//      pruned from the SLEEP mask - does it stop across a WFI with
//      SLEEPDEEP clear, with SLEEPDEEP set, with core 1 asleep in
//      SLEEPDEEP too? And does SysTick count across the SLEEP state?
//      The answers are what docs/rp2040/sleep.md states
//   d  STANDBY THROUGH THE TIMED SITE: a time event 300 ms out, the
//      timer alarm placed, idle() takes the SLEEP state, the alarm
//      ends it, disarm() advances kernel time by the span the timer
//      witnessed less the ticks counted awake - the event due, never
//      early
//   e  a standby that something ELSE ends: the site's alarm 2 s out,
//      the suite's own alarm at 100 ms - the timer is a real witness,
//      so the advance is the true span, and the deadline still stands
//   f  DORMANT ON THE RING OSCILLATOR with the calendar's alarm as the
//      wake: the crystal keeps clk_rtc, a time event 2 s out places
//      the alarm, the idle path's hook moves the clocks, stops the PLL
//      and writes the keyword; the RTC ends it, the tree comes back,
//      the calendar witnesses two seconds the timer never saw
//   g  DORMANT ON THE CRYSTAL with a GPIO level wake: the console's RX
//      pin idles high, a level-high wake ends the dormant at once; the
//      crystal restarted, the PLL back, the console alive
//   h  THE MANAGER over the timed site: the ladder through real rounds,
//      the deadline guard, a standby round through the kernel's own
//      idle hook ended by an alarm that posts, the first event after
//      the wake disarming and reporting
//
//   x  (by name only, NOT in z) DORMANT ON THE CRYSTAL UNTIL A KEY: the
//      RX pin's falling edge (a start bit) is the wake - press any key
//      on the console once the board is dormant.
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/pwm.hpp"
#include "rp2040/rtc.hpp"
#include "rp2040/sleep.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Rp2040Platform<>;
using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;
constexpr uint8_t rx_pin = 1;

TestBench<Serial, 16> bench;

using Plain = Rp2040SleepSite<SysClock>;                                   // a dormant stops the crystal
using PlainRosc = Rp2040SleepSite<SysClock, DormantSource::rosc>;           // a dormant stops the ring oscillator
using Timed = Rp2040TimedSleepSite<P, SysClock, DormantSource::xosc, 3>;    // alarm 3 the site's
using TimedRosc = Rp2040TimedSleepSite<P, SysClock, DormantSource::rosc, 3>;
constexpr uint8_t suite_alarm = 2;                                          // alarm 2 the suite's own wake

/// The gates a standby of this suite keeps: the core set, the console,
/// the calendar, the timer. The PWM is left out on purpose: the
/// instrument.
constexpr SleepClocks standby_gates = sleep_clocks_core | sleep_clocks_uart0 | sleep_clocks_rtc | Timed::timer_gates;

// A time event to have something armed: the AO it posts to is never
// dispatched by a kernel here, its queue is read by hand.
struct Sleeper {
    struct Event { uint8_t n; };
    static inline EventQueue<Event, 2, P> queue;
    static inline TimeEvent<P, Sleeper, Event> alarm{Event{1}};
    static void init() {}
    static void dispatch(const Event&) {}
};

volatile bool suite_alarm_fired = false;
volatile bool site_alarm_fired = false;
volatile bool rtc_fired = false;
volatile bool post_on_suite_alarm = false;

uint32_t us_now() { return Timer::now_low(); }

void console_drain() {
    for (uint32_t spins = 2'000'000u; spins != 0u && !Serial::tx_idle(); --spins) {
    }
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 300u) {
    }
}

const char* depth_name(SleepDepth d) {
    switch (d) {
    case SleepDepth::none: return "none";
    case SleepDepth::light: return "light";
    case SleepDepth::standby: return "standby";
    case SleepDepth::deep: return "deep";
    }
    return "?";
}

bool sleepdeep() { return (SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0u; }
bool tree_on_pll() { return Clocks::sys_source() == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX && PllSys::locked() && Xosc::stable(); }

/// The suite's own wake: alarm 2 `us` out.
void arm_suite_alarm(uint32_t us) {
    suite_alarm_fired = false;
    Timer::clear(suite_alarm);
    Timer::alarm_in(suite_alarm, us);
    Timer::interrupt(suite_alarm, true);
}

/// idle() until `flag` or `max_turns`: the loop's shape, one WFI a
/// turn - and SysTick ends a turn every millisecond through the SLEEP
/// state (letter c), so a wait of hundreds of milliseconds is hundreds
/// of turns.
uint16_t idle_until(volatile bool& flag, uint16_t max_turns = 5000) {
    uint16_t turns = 0;
    while (!flag && turns < max_turns) {
        P::CriticalSection cs;
        P::idle();
        ++turns;
    }
    return turns;
}

/// The state every letter starts from.
void quiesce() {
    Timed::disarm();
    TimedRosc::disarm();
    Plain::disarm();
    PlainRosc::disarm();
    Timer::interrupt(suite_alarm, false);
    Timer::disarm(suite_alarm);
    Timer::clear(suite_alarm);
    DormantWake::disable_all();
    Clocks::sleep_enables(sleep_clocks_all);
    Sleeper::alarm.disarm();
    TimeEvents<P>::clear_all();
    while (Sleeper::queue.pop().has_value()) {
    }
    suite_alarm_fired = false;
    site_alarm_fired = false;
    rtc_fired = false;
    post_on_suite_alarm = false;
}

// ---- the instrument: a PWM slice counting clk_sys / 256 -------------------
using Instrument = PwmSlice<7>;

void instrument_start() {
    (void)Instrument::configure({.divider = *pwm_divider_of(4096), .top = 0xFFFF});   // 125 MHz / 256 = 488 kHz, wraps every 134 ms
    Instrument::counter(0);
    Instrument::enable(true);
}
uint16_t instrument_count() { return Instrument::counter(); }

/// Core 1 parked in a deep WFI: SLEEPDEEP set on its own SCB, then
/// asleep for good (a launch through the bootrom's protocol).
void core1_deep_sleep() {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    for (;;) {
        __WFI();
    }
}

// =============================================================================
// a - as found, and the ladder
// =============================================================================
void ta_found() {
    quiesce();
    const SleepClocks sleep = Clocks::sleep_enables();
    const SleepClocks wake = Clocks::wake_enables();
    const SleepClocks now = Clocks::enabled();
    print(serial, "  SLEEP_EN ", hex(sleep.en0), " ", hex(sleep.en1), "; WAKE_EN ", hex(wake.en0), " ", hex(wake.en1), "; ENABLED ",
          hex(now.en0), " ", hex(now.en1), "; SLEEPDEEP=", sleepdeep(), crlf);
    bench.verdict("every gate is open in both masks as found, and ENABLED reads the wake set less the one clock whose generator is "
                  "not running (clk_usb: no USB PLL)",
                  sleep == sleep_clocks_all && wake == sleep_clocks_all && (now & wake) == now &&
                      (wake.en1 & ~now.en1) == CLOCKS_ENABLED1_CLK_USB_USBCTRL_BITS && wake.en0 == now.en0);
    bench.verdict("SLEEPDEEP is clear, no dormant wake is enabled, the idle path's hook is empty",
                  !sleepdeep() && !DormantWake::any_enabled() && P::sleep_hook == nullptr);
    bench.verdict("the named sets: the core set keeps the fabric, the memories, the timer and the watchdog, not the PWM; the "
                  "console's set adds both UART0 gates",
                  (sleep_clocks_core.en0 & CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC_BITS) != 0u &&
                      (sleep_clocks_core.en0 & CLOCKS_SLEEP_EN0_CLK_SYS_SRAM0_BITS) != 0u &&
                      (sleep_clocks_core.en1 & CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS) != 0u &&
                      (sleep_clocks_core.en0 & CLOCKS_SLEEP_EN0_CLK_SYS_PWM_BITS) == 0u &&
                      (standby_gates.en1 & CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS) != 0u &&
                      (standby_gates & sleep_clocks_pwm) == sleep_clocks_none);

    bench.verdict("the ladder: light is a WFI with SLEEPDEEP clear",
                  Plain::arm(SleepDepth::light) && Plain::armed() == SleepDepth::light && !sleepdeep() && P::sleep_hook == nullptr);
    bench.verdict("standby is SLEEPDEEP", Plain::arm(SleepDepth::standby) && Plain::armed() == SleepDepth::standby && sleepdeep());
    bench.verdict("deep is REFUSED with no way back (no GPIO wake, the crystal the oscillator stopped), the standby left armed",
                  !Plain::arm(SleepDepth::deep) && Plain::armed() == SleepDepth::standby);
    bench.verdict("and refused on the ring-oscillator site too while the calendar's alarm is not armed",
                  !PlainRosc::dormant_wake_ready() && !PlainRosc::arm(SleepDepth::deep));
    (void)DormantWake::enable(rx_pin, dormant_wake_bits(DormantWakeEvent::level_high), true);
    bench.verdict("a GPIO wake enabled, deep arms: SLEEPDEEP clear and the hook installed",
                  DormantWake::enabled(rx_pin) == dormant_wake_bits(DormantWakeEvent::level_high) && Plain::dormant_wake_ready() &&
                      Plain::arm(SleepDepth::deep) && Plain::armed() == SleepDepth::deep && !sleepdeep() && P::sleep_hook != nullptr);
    Plain::disarm();
    bench.verdict("disarmed: none, SLEEPDEEP clear, the hook empty", Plain::armed() == SleepDepth::none && !sleepdeep() && P::sleep_hook == nullptr);
    bench.verdict("the dormant-wake verbs refuse a 31st pin and an event outside the four",
                  !DormantWake::enable(30, dormant_wake_bits(DormantWakeEvent::edge_low), true) && !DormantWake::enable(1, 0x10, true));
    quiesce();
}

// =============================================================================
// b - a light sleep
// =============================================================================
void tb_light() {
    quiesce();
    (void)Plain::arm(SleepDepth::light);
    console_drain();
    const uint32_t t1 = Ticker::ticks();
    uint8_t calls = 0;
    while (Ticker::ticks() == t1 && calls < 20u) {
        P::CriticalSection cs;
        P::idle();
        ++calls;
    }
    Plain::disarm();
    print(serial, "  armed light: ", calls, " idle() call(s) to the next tick", crlf);
    bench.verdict("a light sleep is ended by the tick, inside one tick", calls >= 1u && calls < 20u);
    bench.verdict("and the tick went on counting through it", Ticker::ticks() > t1);
}

// =============================================================================
// c - the instrument: what the SLEEP state stops
// =============================================================================
struct Leg {
    const char* name;
    uint16_t counted;   // the instrument's counts across the sleep
    uint32_t us;        // the timer's microseconds across it
    uint32_t ticks;     // SysTick's ticks across it
    uint16_t turns;     // idle() turns to the alarm
};

Leg sleep_leg(const char* name, bool deep, uint32_t us) {
    Clocks::sleep_enables(standby_gates);
    if (deep) { SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; } else { SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; }
    console_drain();
    Instrument::counter(0);
    arm_suite_alarm(us);
    const uint32_t t0 = us_now();
    const uint32_t k0 = Ticker::ticks();
    const uint16_t turns = idle_until(suite_alarm_fired);
    const uint32_t k1 = Ticker::ticks();
    const uint32_t t1 = us_now();
    const uint16_t counted = instrument_count();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    Clocks::sleep_enables(sleep_clocks_all);
    return Leg{name, counted, t1 - t0, k1 - k0, turns};
}

void tc_instrument() {
    quiesce();
    instrument_start();
    // Awake, as the baseline: what the instrument counts in 20 ms.
    Instrument::counter(0);
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 20'000u) {
    }
    const uint16_t awake = instrument_count();
    print(serial, "  awake 20 ms: the instrument counted ", awake, " (488 kHz -> 9766)", crlf);
    bench.verdict("the instrument counts clk_sys / 256 awake", awake > 9'000u && awake < 10'500u);

    Leg legs[3];
    legs[0] = sleep_leg("WFI, SLEEPDEEP clear", false, 20'000u);
    legs[1] = sleep_leg("WFI, SLEEPDEEP set, core 1 in the bootrom", true, 20'000u);
    const bool launched = Core1::launch(core1_deep_sleep);
    legs[2] = sleep_leg("WFI, SLEEPDEEP set, core 1 asleep in SLEEPDEEP", true, 20'000u);
    for (const Leg& l : legs) {
        print(serial, "  ", l.name, ": ", l.us, " us on the timer, ", l.ticks, " ticks on SysTick, the PWM (its gate pruned) counted ",
              l.counted, ", ", l.turns, " idle() turn(s)", crlf);
    }
    bench.verdict("core 1 launched into its deep WFI", launched);
    bench.verdict("the timer counted the 20 ms of every leg (its gates kept)", legs[0].us > 19'000u && legs[1].us > 19'000u && legs[2].us > 19'000u);
    const bool state_plain = legs[0].counted < 500u;
    const bool state_deep = legs[1].counted < 500u;
    const bool state_with_core1 = legs[2].counted < 500u;
    print(serial, "  -> the SLEEP state is reached on a plain WFI with core 1 in the bootrom: ", state_plain ? "YES" : "no",
          "; with SLEEPDEEP: ", state_deep ? "YES" : "no", "; with core 1 asleep in SLEEPDEEP too: ", state_with_core1 ? "YES" : "no",
          "; SysTick in it: ", legs[2].ticks >= 18u ? "COUNTS" : legs[2].ticks == 0u ? "frozen" : "partly", crlf);
    bench.verdict("THE SLEEP STATE IS REACHED ON A PLAIN WFI, SLEEPDEEP clear, core 1 in the bootrom's WFE: the pruned PWM's counter "
                  "stands still (2.11.2's rule taken literally - the note asking for deep sleep on both cores is not what the "
                  "silicon needs)",
                  state_plain);
    bench.verdict("and with SLEEPDEEP set, alone and with core 1 asleep in SLEEPDEEP too", state_deep && state_with_core1);
    bench.verdict("SYSTICK COUNTS THROUGH THE SLEEP STATE: the ticker kept its 20 ticks in every leg and each tick ended a turn, so "
                  "kernel time never freezes in a standby",
                  legs[0].ticks >= 18u && legs[1].ticks >= 18u && legs[2].ticks >= 18u && legs[2].turns >= 18u);
    bench.verdict("the alarm of the timer (its gates kept) ends the SLEEP state", suite_alarm_fired);
    Instrument::enable(false);
    Core1::reset();
    quiesce();
}

// =============================================================================
// d - standby through the timed site
// =============================================================================
void td_standby() {
    quiesce();
    if (!Timed::init()) {
        bench.verdict("the timed site initializes", false);
        return;
    }
    (void)Core1::launch(core1_deep_sleep);
    Clocks::sleep_enables(standby_gates);
    instrument_start();
    Sleeper::alarm.arm(ticks_from_ms<P>(300));
    const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
    bench.verdict("a time event 300 ms out is the nearest deadline", next && *next == 300u);
    const uint32_t remaining = TimeEvents<P>::ticks_to_next().value_or(0u);
    const uint32_t t_before = Ticker::ticks();
    bench.verdict("arm(standby) takes, sets SLEEPDEEP, keeps the timer's gates and places alarm 3",
                  Timed::arm(SleepDepth::standby) && Timed::armed() == SleepDepth::standby && sleepdeep() && Timed::alarm_armed() &&
                      Timer::armed(3) && (Clocks::sleep_enables() & Timed::timer_gates) == Timed::timer_gates);
    print(serial, "  placed: alarm 3 at +", Timer::alarm_reg(3) - us_now(), " us; entering the SLEEP state...", crlf);
    console_drain();
    Instrument::counter(0);
    const uint32_t us0 = us_now();
    const uint16_t turns = idle_until(site_alarm_fired);
    const uint32_t slept_us = us_now() - us0;
    const uint16_t counted = instrument_count();
    const bool fired = site_alarm_fired;
    const bool still_deep = sleepdeep();
    Timed::disarm();
    const uint32_t t_after = Ticker::ticks();
    const uint32_t advanced = Timed::last_advance();
    print(serial, "  woke after ", turns, " idle() turn(s), ", slept_us, " us on the timer, the pruned PWM counted ", counted,
          "; SLEEPDEEP at the wake=", still_deep, "; tick ", t_before, " -> ", t_after, " (advanced by ", advanced, ")", crlf);
    bench.verdict("the site's alarm is what ended the standby", fired);
    bench.verdict("the ISR body handed the machine back to a ticking sleep (SLEEPDEEP clear at the wake)", !still_deep);
    bench.verdict("the chip was in the SLEEP state (the pruned PWM stood still)", counted < 500u);
    bench.verdict("kernel time from arm to here covers the deadline the site was given - late, never early - and by less than a tick "
                  "plus the awake turns",
                  t_after - t_before >= remaining && t_after - t_before < remaining + 20u);
    bench.verdict("and the time event is due (never early)",
                  !TimeEvents<P>::ticks_to_next().has_value() || *TimeEvents<P>::ticks_to_next() == 0u);
    TimeEvents<P>::process();
    bench.verdict("processing matures it", Sleeper::queue.pop().has_value());
    Instrument::enable(false);
    Core1::reset();
    quiesce();
}

// =============================================================================
// e - a standby ended by something else
// =============================================================================
void te_other_wake() {
    quiesce();
    if (!Timed::init()) {
        bench.verdict("the timed site initializes", false);
        return;
    }
    (void)Core1::launch(core1_deep_sleep);
    Clocks::sleep_enables(standby_gates);
    Sleeper::alarm.arm(ticks_from_secs<P>(2));
    const bool armed = Timed::arm(SleepDepth::standby) && Timed::alarm_armed();
    console_drain();
    const uint32_t t_before = Ticker::ticks();
    arm_suite_alarm(100'000u);   // the suite's own wake at 100 ms
    const uint32_t us0 = us_now();
    const uint16_t turns = idle_until(suite_alarm_fired);
    const uint32_t slept_us = us_now() - us0;
    const bool site_fired = site_alarm_fired;
    Timed::disarm();
    const uint32_t advanced = Timed::last_advance();
    const uint32_t elapsed = Ticker::ticks() - t_before;
    print(serial, "  armed for 2 s, woken by alarm 2 after ", slept_us, " us and ", turns, " turn(s): advanced by ", advanced,
          ", kernel time moved ", elapsed, " ticks, the deadline now ", TimeEvents<P>::ticks_to_next().value_or(0u), " ticks out", crlf);
    bench.verdict("the site's alarm was placed and never fired", armed && !site_fired);
    bench.verdict("a foreign wake is accounted for honestly: kernel time moved by the 100 ms the timer witnessed, within a few ticks",
                  elapsed >= 100u && elapsed < 110u);
    bench.verdict("and the deadline still stands, about 1.9 s out",
                  TimeEvents<P>::ticks_to_next().has_value() && *TimeEvents<P>::ticks_to_next() > 1'850u &&
                      *TimeEvents<P>::ticks_to_next() <= 1'900u);
    Core1::reset();
    quiesce();
}

// =============================================================================
// f - dormant on the ring oscillator, the calendar's alarm the wake
// =============================================================================
void tf_dormant_rosc() {
    quiesce();
    const bool rtc_up = Rtc::init(clock) && Rtc::set({.year = 2026, .month = 9, .day = 13, .weekday = 0, .hour = 12, .minute = 0, .second = 0});
    bench.verdict("the calendar runs on the crystal over 256", rtc_up && Rtc::running());
    if (!rtc_up || !TimedRosc::init()) {
        bench.verdict("the timed site initializes", TimedRosc::ready());
        return;
    }
    // The manual way back, should the calendar not be one: a falling
    // edge on the console's RX pin (any key). The edges the console's
    // own traffic left are acknowledged just before the dormant.
    (void)DormantWake::enable(rx_pin, dormant_wake_bits(DormantWakeEvent::edge_low), true);
    Sleeper::alarm.arm(ticks_from_secs<P>(2));
    const uint32_t remaining = TimeEvents<P>::ticks_to_next().value_or(0u);
    const uint32_t t_before = Ticker::ticks();
    const bool armed = TimedRosc::arm(SleepDepth::deep);
    const std::optional<RtcAlarm> placed = Rtc::alarm();
    print(serial, "  arm(deep): ", armed ? "taken" : "REFUSED", ", the calendar's alarm at ", placed ? placed->at.hour : 0u, ":",
          placed ? placed->at.minute : 0u, ":", placed ? placed->at.second : 0u, " armed=", Rtc::alarm_armed(),
          " INTE=", Rtc::interrupt_enabled(), "; the hook ", P::sleep_hook != nullptr ? "installed" : "EMPTY", "; going dormant...", crlf);
    bench.verdict("arm(deep) takes on the ring-oscillator site: the calendar's alarm placed two seconds out, its interrupt on, the "
                  "hook installed",
                  armed && TimedRosc::armed() == SleepDepth::deep && placed && placed->at.second == 2u && Rtc::alarm_armed() &&
                      Rtc::interrupt_enabled() && P::sleep_hook != nullptr);
    if (!armed) {
        quiesce();
        return;
    }
    console_drain();
    DormantWake::acknowledge(rx_pin, 0x0Fu);
    const std::optional<RtcDateTime> before = Rtc::read();
    const uint32_t us0 = us_now();
    const uint16_t turns = idle_until(rtc_fired, 5);
    const uint32_t timer_saw = us_now() - us0;
    const bool back = tree_on_pll();
    const std::optional<RtcDateTime> after = Rtc::read();
    const bool by_rtc = rtc_fired;
    TimedRosc::disarm();
    const uint32_t t_after = Ticker::ticks();
    const uint32_t advanced = TimedRosc::last_advance();
    const uint32_t rtc_span = before && after ? (after->second + 60u - before->second) % 60u : 99u;
    print(serial, "  back after ", turns, " turn(s): by the RTC=", by_rtc, ", dormants=", PlainRosc::dormants(), ", the tree on the PLL=",
          back, "; the calendar ", before ? before->second : 0u, " -> ", after ? after->second : 0u, " s, the timer saw ", timer_saw,
          " us; tick ", t_before, " -> ", t_after, " (advanced by ", advanced, ")", crlf);
    bench.verdict("the calendar's interrupt ended the dormant, once, and the tree is back on the PLL", by_rtc && PlainRosc::dormants() == 1u && back);
    bench.verdict("the calendar counted two seconds the timer never saw (clk_ref stood with the ring oscillator)",
                  rtc_span == 2u && timer_saw < 100'000u);
    bench.verdict("kernel time was advanced by the frozen span: arm to here covers the deadline, late, never early",
                  advanced >= 1'900u && t_after - t_before >= remaining && t_after - t_before < remaining + 1'100u);
    bench.verdict("and the time event is due", !TimeEvents<P>::ticks_to_next().has_value() || *TimeEvents<P>::ticks_to_next() == 0u);
    TimeEvents<P>::process();
    bench.verdict("processing matures it", Sleeper::queue.pop().has_value());
    quiesce();
}

// =============================================================================
// g - dormant on the crystal, a GPIO level the wake
// =============================================================================
void tg_dormant_xosc() {
    quiesce();
    const bool armed = DormantWake::enable(rx_pin, dormant_wake_bits(DormantWakeEvent::level_high), true) && Plain::arm(SleepDepth::deep);
    bench.verdict("a level-high wake on the RX pin (idle high) enabled, deep arms on the crystal site", armed && Plain::armed() == SleepDepth::deep);
    if (!armed) {
        return;
    }
    print(serial, "  going dormant on the crystal, the RX pin's level the way back...", crlf);
    console_drain();
    const uint32_t before = Plain::dormants();
    volatile bool never = false;
    const uint16_t turns = idle_until(never, 1);
    const bool back = tree_on_pll();
    Plain::disarm();
    print(serial, "  back after ", turns, " turn: dormants ", before, " -> ", Plain::dormants(), ", XOSC stable=", Xosc::stable(),
          " enabled=", Xosc::enabled(), ", PLL locked=", PllSys::locked(), ", clk_sys on aux=", Clocks::sys_source(), crlf);
    bench.verdict("the dormant ran and returned - the level wake ends it as soon as it begins - the crystal restarted and the PLL "
                  "relocked by the hook, this line printed at the rate the console was told",
                  Plain::dormants() == before + 1u && back);
    bench.verdict("the raw flag of the level event stands (it follows the pad) and the site is disarmed",
                  (DormantWake::raised(rx_pin) & dormant_wake_bits(DormantWakeEvent::level_high)) != 0u && Plain::armed() == SleepDepth::none);
    quiesce();
}

// =============================================================================
// h - the manager over the timed site
// =============================================================================
struct Blip {};

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline bool accept = true;
    static inline uint8_t votes = 0;
    static inline bool last_ok = false;
    static inline uint8_t asked = 0;
    static inline SleepDepth asked_depth = SleepDepth::none;
    static inline uint8_t wakes = 0;
    static inline SleepDepth woke_from = SleepDepth::none;
    static inline uint16_t blips = 0;

    static void init() { start(&only); }

    static Status only(const Event& e) {
        return match(
            e, [](Entry) { return handled(); }, [](Exit) { return handled(); },
            [](SleepVote v) {
                ++votes;
                last_ok = v.ok;
                return handled();
            },
            [](const PrepareSleep& p) {
                ++asked;
                asked_depth = p.depth;
                p.reply.send(SleepVote{accept});
                return handled();
            },
            [](WakeReport w) {
                ++wakes;
                woke_from = w.was;
                return handled();
            },
            [](Blip) {
                ++blips;
                return handled();
            });
    }
};

using Pm_ = PowerManager<P, Timed, PowerConfig{}, Probe>;
using K = Kernel<P, Probe, Pm_>;

void pump() {
    for (uint16_t i = 0; i < 500u; ++i) {
        TimeEvents<P>::process();
        if (!K::step()) {
            return;
        }
    }
}

void ask(SleepDepth d) {
    post<Pm_>(SleepRequested{d, reply_to<Probe, SleepVote>()});
    pump();
}

/// "Awake, no new request": the one event a wake path posts when it has
/// nothing else to say. Any event would disarm the site.
void release_sleep() { ask(SleepDepth::none); }

void manager_quiesce() {
    quiesce();
    Probe::deadline.disarm();
    while (Probe::queue.pop().has_value()) {
    }
    while (Pm_::queue.pop().has_value()) {
    }
    Probe::accept = true;
    Probe::votes = Probe::asked = Probe::wakes = 0;
    Probe::last_ok = false;
    Probe::asked_depth = SleepDepth::none;
    Probe::woke_from = SleepDepth::none;
    Probe::blips = 0;
    K::init_all();   // Pm_::init() disarms the site
}

void th_manager() {
    manager_quiesce();
    (void)Timed::init();
    (void)Core1::launch(core1_deep_sleep);
    Clocks::sleep_enables(standby_gates);

    // The ladder through real rounds.
    struct RoundLeg {
        SleepDepth asked;
        bool deep;
        SleepDepth manager;
        bool ok;
    };
    RoundLeg legs[4];
    const SleepDepth ladder[4] = {SleepDepth::none, SleepDepth::light, SleepDepth::standby, SleepDepth::deep};
    for (uint8_t i = 0; i < 4u; ++i) {
        Probe::last_ok = false;
        ask(ladder[i]);
        legs[i] = RoundLeg{ladder[i], sleepdeep(), Pm_::armed_depth(), Probe::last_ok};
        release_sleep();
    }
    for (const RoundLeg& l : legs) {
        print(serial, "  ", depth_name(l.asked), ": SLEEPDEEP=", l.deep, " manager=", depth_name(l.manager), l.ok ? " (accepted)" : " (refused)", crlf);
    }
    bench.verdict("none arms nothing and is accepted; light arms light; standby arms standby with SLEEPDEEP",
                  legs[0].ok && legs[0].manager == SleepDepth::none && legs[1].ok && legs[1].manager == SleepDepth::light && !legs[1].deep &&
                      legs[2].ok && legs[2].manager == SleepDepth::standby && legs[2].deep);
    bench.verdict("deep is refused by the site (no way back configured) and the manager reports the refusal", !legs[3].ok && legs[3].manager == SleepDepth::none);
    bench.verdict("every round but none's asked the voter once (none is no round)", Probe::asked == 3u);

    // A voter's veto.
    Probe::accept = false;
    ask(SleepDepth::standby);
    bench.verdict("one not-ok ends the round: nothing armed", !Probe::last_ok && Pm_::armed_depth() == SleepDepth::none && !sleepdeep());
    Probe::accept = true;

    // The deadline guard.
    Probe::deadline.arm(1);
    ask(SleepDepth::standby);
    const bool guarded = !Probe::last_ok;
    Probe::deadline.disarm();
    pump();
    bench.verdict("a standby with a time event one tick away is refused by the deadline guard (min_deep_ticks 2)", guarded);

    // A real standby round through the kernel's own idle hook, ended by
    // the suite's alarm posting a Blip, then the first event after the
    // wake.
    Probe::deadline.arm(300);
    ask(SleepDepth::standby);
    bench.verdict("a round with a 300 ms deadline arms standby, the site's alarm placed by the timed site",
                  Probe::last_ok && Pm_::armed_depth() == SleepDepth::standby && sleepdeep() && Timed::alarm_armed());
    post_on_suite_alarm = true;
    console_drain();
    arm_suite_alarm(50'000u);   // a foreign wake at 50 ms that posts (armed after the drain: a line takes ten milliseconds)
    const uint32_t k0 = Ticker::ticks();
    const uint32_t us0 = us_now();
    // THE KERNEL'S OWN HOOK, turn after turn: idle_if_empty() masks,
    // finds every queue empty and calls P::idle(); SysTick ends each
    // turn (letter c) and the loop takes the next, until the alarm's
    // Blip fills a queue.
    uint16_t turns = 0;
    while (!suite_alarm_fired && turns < 5000u) {
        K::idle_if_empty();
        ++turns;
    }
    const uint32_t slept_us = us_now() - us0;
    const uint32_t k1 = Ticker::ticks();
    print(serial, "  the kernel slept ", slept_us, " us of the timer and ", k1 - k0, " ticks of kernel time in ", turns, " turns", crlf);
    bench.verdict("the kernel's idle hook took the standby the manager armed, a turn a tick, and the foreign alarm ended it near 50 ms",
                  suite_alarm_fired && slept_us > 45'000u && slept_us < 60'000u && turns >= 40u);
    bench.verdict("the site is still armed: a wake that says nothing to the manager changes nothing",
                  Pm_::armed_depth() == SleepDepth::standby && Timed::armed() == SleepDepth::standby);
    pump();
    bench.verdict("the wake ISR's event reached its own AO", Probe::blips >= 1u);
    const uint8_t wakes_before = Probe::wakes;
    release_sleep();
    print(serial, "  WakeReport: ", Probe::wakes - wakes_before, " received, was=", depth_name(Probe::woke_from), crlf);
    bench.verdict("the first event to reach the MANAGER disarms: SLEEPDEEP clear, the site's alarm gone, kernel time honest",
                  !sleepdeep() && Timed::armed() == SleepDepth::none && !Timed::alarm_armed() && Ticker::ticks() - k0 >= 50u);
    bench.verdict("and publishes the WakeReport of the depth that was armed", Probe::wakes == wakes_before + 1u && Probe::woke_from == SleepDepth::standby);
    bench.verdict("the manager forgot the round", Pm_::armed_depth() == SleepDepth::none);
    Probe::deadline.disarm();
    Core1::reset();
    manager_quiesce();
}

// =============================================================================
// x - dormant on the crystal until a key (outside z)
// =============================================================================
void tx_dormant_key() {
    quiesce();
    (void)DormantWake::enable(rx_pin, dormant_wake_bits(DormantWakeEvent::edge_low), true);
    const bool armed = Plain::arm(SleepDepth::deep);
    print(serial, "  dormant on the crystal now; PRESS ANY KEY to wake (the RX pin's falling edge)", crlf);
    console_drain();
    DormantWake::acknowledge(rx_pin, 0x0Fu);
    volatile bool never = false;
    (void)idle_until(never, 1);
    const bool back = tree_on_pll();
    Plain::disarm();
    print(serial, "  awake: dormants=", Plain::dormants(), ", the tree on the PLL=", back, crlf);
    bench.verdict("the key's edge ended the dormant and the tree is back", armed && back);
    quiesce();
}

void banner() {
    print(serial, crlf, "test_rp2040_sleep - the RP2040 power chapter (2.11): the SLEEP state and its gates, DORMANT on either oscillator, "
                        "the sites over util/power.hpp; clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_timer_2() {
    if (brio::Timer::pending(suite_alarm)) {
        brio::Timer::clear(suite_alarm);
        brio::Timer::interrupt(suite_alarm, false);
        suite_alarm_fired = true;
        if (post_on_suite_alarm) {
            brio::post<Probe>(Blip{});
        }
    }
}
extern "C" void isr_timer_3() {
    site_alarm_fired = true;
    Timed::isr();
    TimedRosc::isr();
}
extern "C" void isr_rtc() {
    if (TimedRosc::rtc_isr() || Timed::rtc_isr()) {
        rtc_fired = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::Nvic::enable(brio::Timer::irq(suite_alarm));
    brio::enable_interrupts();

    bench.letter('a', "as found, and the ladder through the plain site", ta_found);
    bench.letter('b', "a light sleep, ended by the tick", tb_light);
    bench.letter('c', "the instrument: what the SLEEP state stops", tc_instrument);
    bench.letter('d', "standby through the timed site, ended by its alarm", td_standby);
    bench.letter('e', "a standby ended by something else", te_other_wake);
    bench.letter('f', "dormant on the ring oscillator, the calendar the wake", tf_dormant_rosc);
    bench.letter('g', "dormant on the crystal, a GPIO level the wake", tg_dormant_xosc);
    bench.letter('h', "the manager over the timed site", th_manager);
    bench.letter('x', "dormant on the crystal until a key", tx_dormant_key, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
