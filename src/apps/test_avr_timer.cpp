// test_avr_timer - the TCA/TCB test SUITE for the AVR DA/DB target:
// the TCA tasks generate known waveforms, the TCB tasks measure them
// back through the event system (pin-level generators on the driven
// pins: no jumper wires), counters count known event rates against
// the RTC Ticker. Reference test of avrdx/tca.hpp and avrdx/tcb.hpp
// (docs/avrdx/tca.md, tcb.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking; the ISRs
// hand captures to the test through volatile cells). Console on USART2
// ALT1 (PF4/PF5) at 460800.
//
// Pins used (bench board): PD0 = TCA0 WO0 (generator), PC0 = TCB2 WO
// (Pwm8), PB5 = TCB3 WO (one-shot; LED2 blue on the traffic bench - it
// flickers), PC1 = a level the test drives for the direction input.
// Nothing to wire. Event channels: 0 PB5, 1 software, 2 PD0, 3 PC1 or
// PC0, 4/5 internal generators - re-sourced per test.
//
// Commands: ? | 1 frequency | 2 PWM16 duty | 3 heartbeat | 4 one-shot
// | 5 Pwm8 | 6 pulse counter | 7 32-bit cascade | 8 tick + timeout
// | 9 event counter | a all
//
// Tests:
//   1  FrequencyGenerator (TCA0, PD0) at 500 Hz / 1 kHz / 10 kHz /
//      100 kHz -> FrequencyMeter (TCB0 via EvPin PD0): period in ticks
//      = 24 MHz / f within 1 tick; set_hz under a running output; the
//      TCB clocked by CLK_TCA (div64) and SYNCUPD: ticks scale by 64;
//   2  TcaPwm16 (24000 steps = 1 kHz) duty 25/50/75 % -> DutyMeter
//      (FRQPW): period 24000, duty within 2 permille; PulseWidthMeter
//      agrees; endpoints 0 and max: static pin, no capture;
//   3  Heartbeat 100 Hz with a 500 us pulse on WO0: OVF interrupts
//      counted for 1 s (Ticker) = 100 +-1; the pulse width 500 us +-1 us;
//   4  OneShotPulse (TCB3, PB5) 100 us fired by software -> PulseWidth-
//      Meter (TCB1 via EvPin PB5): 2400 ticks +-2; async variant; busy()
//      during the pulse; a second fire() during the pulse is ignored
//      (one capture); 10 us and 1000 us widths;
//   5  Pwm8 (TCB2, PC0) period 255, duty 64/128/192 -> DutyMeter (TCB0
//      via EvPin PC0): period 256 ticks, duty = v/256 within 2 permille;
//      duty(max) = static high (pin reads 1, no capture);
//   6  PulseCounter (TCB1) counting TCA0 OVF events (Heartbeat 1 kHz)
//      for 200 ms: 200 +-2; snapshot by software event latches CNT;
//   7  CascadedCounter (TCB1 + TCB2, carry on ch 4, snapshot on ch 5)
//      clocked at CLK_PER for 1 s: 24000000 +-0.2 %; monotonic reads;
//      reset;
//   8  PeriodicTick (TCB0) at 1000 Hz: interrupts in 1 s = 1000 +-2;
//      Timeout (TCB1) of 1 ms watching PD0: with a 100 Hz 50 % square
//      (5 ms high) CAPT fires each period; with 10 kHz (50 us high) it
//      never fires;
//   9  EventCounter (TCA1) counting TCB0 CAPT events (PeriodicTick 1 kHz)
//      for 200 ms: 200 +-2; direction from PC1 level: counts down.
// Not testable here: the TCA1 PORTC three-channel route and TCB WO
// ALT1 pins (console), RUNSTDBY (no sleep in this suite), CCL/AC (their
// own docs: no driver yet), TCD.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/tca.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

// ---- the actors --------------------------------------------------------------
using GenPin = Pin<'D', 0>;          // TCA0 WO0
using Pwm8Pin = Pin<'C', 0>;         // TCB2 WO
using ShotPin = Pin<'B', 5>;         // TCB3 WO
using DirPin = Pin<'C', 1>;          // a level the test drives

using T0 = Tcb<0>;                   // the meters
using T1 = Tcb<1>;                   // counters, timeout, the one-shot's meter
using T2 = Tcb<2>;                   // Pwm8, cascade MSB
using T3 = Tcb<3>;                   // one-shot

using ChShot = EventChannel<0>;      // PB5 level (PORTA/B: channels 0-1)
using ChSoft = EventChannel<1>;      // software pulses
using ChGen = EventChannel<2>;       // PD0 level (PORTC/D: channels 2-3)
using ChC = EventChannel<3>;         // PC0 or PC1 level
using ChCarry = EventChannel<4>;     // internal generators
using ChSnap = EventChannel<5>;

// ---- what the ISRs hand over -------------------------------------------------
// One hook per TCB vector: each test installs what its mode needs.
using Hook = void (*)();
volatile Hook tcb0_hook = nullptr;
volatile Hook tcb1_hook = nullptr;
volatile uint16_t cap_a = 0, cap_b = 0;      // last capture(s)
volatile uint16_t captures = 0;              // how many
volatile uint16_t irq_count = 0;             // periodic interrupts

void tcb0_frequency() { cap_a = FrequencyMeter<T0>::period_ticks(); ++captures; }
void tcb0_duty() { const auto r = DutyMeter<T0>::reading(); cap_a = r.period_ticks; cap_b = r.width_ticks; ++captures; }
void tcb0_width() { cap_a = PulseWidthMeter<T0>::width_ticks(); ++captures; }
void tcb0_tick() { PeriodicTick<T0>::tick(); ++irq_count; }
void tcb1_width() { cap_a = PulseWidthMeter<T1>::width_ticks(); ++captures; }
void tcb1_timeout() { Timeout<T1>::expired(); ++captures; }

void clear_captures() {
    cli();
    cap_a = cap_b = 0;
    captures = 0;
    irq_count = 0;
    sei();
}

// ---- tiny test harness --------------------------------------------------------
uint8_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
void hold_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {}
}
// Wait for at least n captures, at most ms milliseconds.
bool wait_captures(uint16_t n, uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        if (captures >= n) return true;
    }
    return false;
}
// Stop everything a previous test may have left running.
void quiesce() {
    tcb0_hook = nullptr;
    tcb1_hook = nullptr;
    T0::disable(); T1::disable(); T2::disable(); T3::disable();
    T0::enable_capt_interrupt(false); T1::enable_capt_interrupt(false);
    Tca<0>::disable(); Tca<1>::disable();
    ChShot::off(); ChSoft::off(); ChGen::off(); ChC::off(); ChCarry::off(); ChSnap::off();
    clear_captures();
}

// ---- 1 frequency -------------------------------------------------------------
using Gen = FrequencyGenerator<0, 'D'>;

void t1_frequency() {
    print(serial, "1 FrequencyGenerator TCA0/PD0 -> FrequencyMeter TCB0 (EvPin PD0 on ch 2)", crlf);
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChGen{});
    constexpr uint32_t rates[] = {500, 1000, 10000, 100000};   // 366 Hz is the slowest a 16-bit capture at CLK_PER resolves
    for (uint32_t hz : rates) {
        clear_captures();
        verdict("generator accepts the rate", Gen::init(clock, hz));
        const bool got = wait_captures(3, 200);
        const uint16_t t = cap_a;
        const uint32_t expect = SysClock::hz / hz;
        print(serial, "  ", hz, " Hz: actual_hz=", Gen::actual_hz(), " period ticks=", t,
              " expect ", expect, " (", FrequencyMeter<T0>::hz(t), " Hz)", crlf);
        verdict("captures arrive", got);
        verdict("period within 1 tick", near(t, static_cast<int32_t>(expect), 1));
        verdict("hz() rounds back", near(static_cast<int32_t>(FrequencyMeter<T0>::hz(t)), static_cast<int32_t>(hz), static_cast<int32_t>(hz / 200 + 1)));
    }
    // set_hz under a running output (buffered CMP0)
    clear_captures();
    Gen::set_hz(2000);
    wait_captures(5, 100);
    verdict("set_hz under run: 2 kHz within 1 tick", near(cap_a, 12000, 1));
    // the TCB clocked by the TCA's prescaled clock, restarting with it
    Gen::stop();
    Tca<0>::init({.mode = TcaMode::frequency, .clock = TcaClock::div64, .compare0 = 374,
                  .outputs = 0x01, .route = 'D'});        // 24e6 / (2*64*375) = 500 Hz
    clear_captures();
    T0::init({.mode = TcbMode::frequency, .clock = TcbClock::tca0, .compare = 0,
              .event_input = true, .sync_update = true});
    T0::capture_on(ChGen{});
    T0::enable_capt_interrupt(true);
    wait_captures(3, 100);
    print(serial, "  CLK_TCA (div64) clocked TCB: period ticks=", cap_a, " expect 750", crlf);
    verdict("TCB on CLK_TCA: 500 Hz = 750 ticks of CLK_PER/64", near(cap_a, 750, 1));
    quiesce();
}

// ---- 2 PWM16 duty --------------------------------------------------------------
using Pwm16 = TcaPwm16<0, 'D', 24000>;      // 1 kHz at div1

void t2_pwm16() {
    print(serial, "2 TcaPwm16 TCA0/PD0 (24000 steps) -> DutyMeter / PulseWidthMeter TCB0", crlf);
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    Pwm16::init(TcaClock::div1, 0x01);
    constexpr uint16_t duties[] = {6000, 12000, 18000};
    for (uint16_t d : duties) {
        tcb0_hook = tcb0_duty;
        DutyMeter<T0>::init(clock, ChGen{});
        Pwm16::duty<0>(d);
        hold_ms(3);                                   // buffered: lands at BOTTOM
        clear_captures();
        wait_captures(2, 50);
        const uint16_t period = cap_a, width = cap_b;
        const uint16_t pm = DutyMeter<T0>::duty_permille({period, width});
        print(serial, "  duty ", d, "/24000: period=", period, " width=", width, " = ", pm, " permille", crlf);
        verdict("period 24000 +-1", near(period, 24000, 1));
        verdict("duty within 2 permille", near(pm, d / 24, 2));
        tcb0_hook = tcb0_width;
        PulseWidthMeter<T0>::init(clock, ChGen{});
        clear_captures();
        wait_captures(2, 50);
        verdict("PulseWidthMeter agrees +-1", near(cap_a, d, 1));
    }
    Pwm16::duty<0>(0);
    hold_ms(3);
    clear_captures();
    const bool none0 = !wait_captures(1, 20);
    verdict("duty 0: static low, no capture", none0 && !GenPin::read());
    Pwm16::duty<0>(24000);
    hold_ms(3);
    clear_captures();
    const bool none1 = !wait_captures(1, 20);
    verdict("duty max: static high, no capture", none1 && GenPin::read());
    quiesce();
}

// ---- 3 heartbeat ---------------------------------------------------------------
using Beat = Heartbeat<0, 'D'>;
volatile uint16_t beats = 0;

void t3_heartbeat() {
    print(serial, "3 Heartbeat TCA0 100 Hz, 500 us pulse on WO0/PD0", crlf);
    quiesce();
    verdict("init", Beat::init(clock, 100, 0x01));
    Beat::pulse_us<0>(500);
    beats = 0;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 1000) {}
    const uint16_t b = beats;
    print(serial, "  beats in 1 s: ", b, crlf);
    verdict("100 +-1 OVF interrupts per second", near(b, 100, 1));
    ChGen::source(EvPin<GenPin>{});
    tcb0_hook = tcb0_width;
    PulseWidthMeter<T0>::init(clock, ChGen{});
    clear_captures();
    wait_captures(3, 100);
    print(serial, "  pulse width: ", cap_a, " ticks = ", PulseWidthMeter<T0>::us(cap_a), " us", crlf);
    verdict("pulse 500 us +-1 us", near(PulseWidthMeter<T0>::us(cap_a), 500, 1));
    quiesce();
}

// ---- 4 one-shot ----------------------------------------------------------------
using Shot = OneShotPulse<T3>;

void t4_one_shot() {
    print(serial, "4 OneShotPulse TCB3/PB5 fired by software -> PulseWidthMeter TCB1 (EvPin PB5 on ch 0)", crlf);
    quiesce();
    ChShot::source(EvPin<ShotPin>{});
    tcb1_hook = tcb1_width;
    PulseWidthMeter<T1>::init(clock, ChShot{});
    constexpr uint32_t widths[] = {10, 100, 1000};
    for (uint32_t w : widths) {
        verdict("init accepts the width", Shot::init(clock, w, ChSoft{}));
        clear_captures();
        Shot::fire();
        delay_us(clock, 2);                      // the counter starts two CLK_PER after the event
        const bool busy = Shot::busy();
        wait_captures(1, 20);
        print(serial, "  ", w, " us: width=", cap_a, " ticks = ", PulseWidthMeter<T1>::us(cap_a), " us, captures=", captures, crlf);
        verdict("one pulse captured", captures == 1);
        verdict("width within 2 ticks", near(cap_a, static_cast<int32_t>(w * 24), 2));
        if (w >= 100) verdict("busy() right after fire()", busy);
    }
    // retrigger ignored during the pulse
    Shot::init(clock, 1000, ChSoft{});
    clear_captures();
    Shot::fire();
    delay_us(clock, 200);
    Shot::fire();
    hold_ms(5);
    verdict("second fire() during the pulse ignored (one capture)", captures == 1 && near(cap_a, 24000, 2));
    // async: WO rises on the event itself
    Shot::init(clock, 100, ChSoft{}, {.async = true});
    clear_captures();
    Shot::fire();
    wait_captures(1, 20);
    print(serial, "  async 100 us: width=", cap_a, " ticks", crlf);
    verdict("async pulse 100 us within 4 ticks", near(cap_a, 2400, 4));
    quiesce();
}

// ---- 5 Pwm8 --------------------------------------------------------------------
using Pwm = Pwm8<T2>;

void t5_pwm8() {
    print(serial, "5 Pwm8 TCB2/PC0 (period 255) -> DutyMeter TCB0 (EvPin PC0 on ch 3)", crlf);
    quiesce();
    ChC::source(EvPin<Pwm8Pin>{});
    tcb0_hook = tcb0_duty;
    DutyMeter<T0>::init(clock, ChC{});
    Pwm::init();
    constexpr uint16_t duties[] = {64, 128, 192};
    for (uint16_t d : duties) {
        Pwm::duty(d);
        clear_captures();
        wait_captures(3, 20);
        const uint16_t pm = DutyMeter<T0>::duty_permille({cap_a, cap_b});
        print(serial, "  duty ", d, ": period=", cap_a, " width=", cap_b, " = ", pm, " permille", crlf);
        verdict("period 256 ticks", cap_a == 256);
        verdict("width = duty", near(cap_b, d, 1));
    }
    Pwm::duty(Pwm::max);
    hold_ms(1);
    clear_captures();
    const bool none = !wait_captures(1, 10);
    verdict("duty max: static high, no capture", none && Pwm8Pin::read());
    Pwm::duty(0);
    hold_ms(1);
    clear_captures();
    const bool none0 = !wait_captures(1, 10);
    verdict("duty 0: static low", none0 && !Pwm8Pin::read());
    quiesce();
}

// ---- 6 pulse counter -----------------------------------------------------------
void t6_pulse_counter() {
    print(serial, "6 PulseCounter TCB1 counting TCA0 OVF events (Heartbeat 1 kHz) on ch 4", crlf);
    quiesce();
    Beat::init(clock, 1000, 0, false);
    ChCarry::source(Tca<0>::OvfEvent{});
    PulseCounter<T1>::init(ChCarry{});
    PulseCounter<T1>::snapshot_on(ChSnap{});
    PulseCounter<T1>::reset();
    hold_ms(200);
    ChSnap::pulse();
    const uint16_t live = PulseCounter<T1>::count();
    const uint16_t snap = PulseCounter<T1>::captured();
    print(serial, "  after 200 ms: count=", live, " snapshot=", snap, crlf);
    verdict("200 +-2 events", near(live, 200, 2));
    verdict("snapshot latched the count", near(snap, live, 1));
    verdict("no overflow", !PulseCounter<T1>::overflowed());
    quiesce();
}

// ---- 7 cascade -----------------------------------------------------------------
using Wide = CascadedCounter<T1, T2>;

void t7_cascade() {
    print(serial, "7 CascadedCounter TCB1+TCB2 at CLK_PER (carry ch 4, snapshot ch 5)", crlf);
    quiesce();
    Wide::init(TcbClock::div1, ChCarry{}, ChSnap{});
    Wide::reset();
    const uint32_t a = Wide::read();
    hold_ms(1000);
    const uint32_t b = Wide::read();
    const uint32_t c = Wide::read();
    print(serial, "  reads: ", a, " ", b, " ", c, " (1 s = 24000000 ticks)", crlf);
    verdict("1 s within 0.2 % (RTC millis vs crystal)", near(static_cast<int32_t>(b - a), 24'000'000, 48'000));
    verdict("monotonic", c >= b && b >= a);
    verdict("crossed 65536: the MSB counted", b > 65535);
    Wide::reset();
    verdict("reset", Wide::read() < 1000);
    quiesce();
}

// ---- 8 tick + timeout ------------------------------------------------------------
void t8_tick_timeout() {
    print(serial, "8 PeriodicTick TCB0 1 kHz; Timeout TCB1 1 ms watching PD0", crlf);
    quiesce();
    tcb0_hook = tcb0_tick;
    verdict("PeriodicTick init 1000 Hz", PeriodicTick<T0>::init(clock, 1000));
    clear_captures();
    hold_ms(1000);
    const uint16_t n = irq_count;
    print(serial, "  ticks in 1 s: ", n, crlf);
    verdict("1000 +-2 (millis granularity)", near(n, 1000, 2));
    PeriodicTick<T0>::stop();
    // Timeout: start on the rising edge of PD0, CAPT if 1 ms passes before the falling
    ChGen::source(EvPin<GenPin>{});
    tcb1_hook = tcb1_timeout;
    verdict("Timeout init 1 ms", Timeout<T1>::init(clock, 1000, ChGen{}));
    Gen::init(clock, 100);                       // 100 Hz square: 5 ms high
    clear_captures();
    hold_ms(105);
    const uint16_t slow = captures;
    Gen::init(clock, 10000);                     // 50 us high: never times out
    clear_captures();
    hold_ms(50);
    const uint16_t fast = captures;
    print(serial, "  timeouts: 100 Hz -> ", slow, " in 105 ms, 10 kHz -> ", fast, " in 50 ms", crlf);
    verdict("5 ms highs time out each period (10 +-1)", near(slow, 10, 1));
    verdict("50 us highs never time out", fast == 0);
    quiesce();
}

// ---- 9 event counter -------------------------------------------------------------
void t9_event_counter() {
    print(serial, "9 EventCounter TCA1 counting TCB0 CAPT events (PeriodicTick 1 kHz) on ch 4; direction from PC1 on ch 3", crlf);
    quiesce();
    tcb0_hook = tcb0_tick;
    PeriodicTick<T0>::init(clock, 1000);
    ChCarry::source(T0::CaptEvent{});
    DirPin::clear();
    DirPin::output();
    ChC::source(EvPin<DirPin>{});
    EventCounter<1>::init(ChCarry{});
    EventCounter<1>::reset();
    hold_ms(200);
    const uint16_t up = EventCounter<1>::count();
    print(serial, "  up for 200 ms: ", up, crlf);
    verdict("200 +-2 events", near(up, 200, 2));
    EventCounter<1>::direction_on(ChC{});
    DirPin::set();                               // down while high
    hold_ms(100);
    const uint16_t down = EventCounter<1>::count();
    print(serial, "  then down for 100 ms: ", down, crlf);
    verdict("counted down by 100 +-2", near(static_cast<int32_t>(up) - down, 100, 2));
    DirPin::clear();
    DirPin::input();
    quiesce();
}

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_frequency}, {'2', t2_pwm16}, {'3', t3_heartbeat}, {'4', t4_one_shot},
    {'5', t5_pwm8}, {'6', t6_pulse_counter}, {'7', t7_cascade}, {'8', t8_tick_timeout},
    {'9', t9_event_counter},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_timer: 1 frequency | 2 PWM16 duty | 3 heartbeat | 4 one-shot | 5 Pwm8 | "
                  "6 pulse counter | 7 32-bit cascade | 8 tick+timeout | 9 event counter | a all", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(TCB0_INT_vect) { if (tcb0_hook) tcb0_hook(); else (void)T0::take_flags(); }
ISR(TCB1_INT_vect) { if (tcb1_hook) tcb1_hook(); else (void)T1::take_flags(); }
ISR(TCA0_OVF_vect) { Beat::beat(); ++beats; }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    sei();
    print(serial, crlf, "test_avr_timer - TCA/TCB test suite (clk=", xtal ? "XTAL" : "OSCHF",
          " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'a' || c == 'A') {
            uint8_t tp = 0, tf = 0;
            for (const Test& t : tests) { run(t.fn); tp += passed; tf += failed; }
            print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
        } else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
