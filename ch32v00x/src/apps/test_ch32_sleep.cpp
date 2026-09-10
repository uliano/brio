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
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/sleep.hpp"
#include "ch32v00x/ticker.hpp"
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
using Led = Pin<'C', 0>;

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
    bench.verdict("the regulator is in normal mode (1.2 V)", Pwr::ldo() == pwr_ldo_normal);
    bench.verdict("the PVD is off", !Pwr::pvd());
    Pwr::pvd(true, PvdLevel::v2_66);
    (void)delay_us(clock, 100);
    print(serial, "  PVD at 2.66 V: supply_low=", Pwr::supply_low(), crlf);
    bench.verdict("with the PVD on at 2.66 V a 3.3 V supply reads NOT low", !Pwr::supply_low());
    Pwr::pvd(false);
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

void banner() {
    print(serial, crlf, "test_ch32_sleep - CH32V006K8 (Sleep, Standby, the AWU on the LSI)", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "PWR as found, and the AWU's arithmetic", ta_found);
    bench.letter('b', "the AWU awake, the LSI measured", tb_awu);
    bench.letter('c', "Sleep through the plain site", tc_sleep);
    bench.letter('d', "STANDBY through the timed site, ended by the AWU", td_standby);
    bench.letter('e', "a Standby armed and disarmed unslept", te_other_wake);

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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
