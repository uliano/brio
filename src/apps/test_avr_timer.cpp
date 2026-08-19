// test_avr_timer - the TCA/TCB/CCL/AC test SUITE for the AVR DA/DB
// target: the TCA tasks generate known waveforms, the TCB tasks measure
// them back through the event system (pin-level generators on the
// driven pins: no jumper wires), counters count known event rates
// against the RTC Ticker; the CCL is checked on pins and with a timer
// event through a flip-flop, the comparators against the DAC on PD6.
// Reference test of avrdx/tca.hpp, tcb.hpp, ccl.hpp, ac.hpp
// (docs/avrdx/tca.md, tcb.md, ccl.md, ac.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking; the ISRs
// hand captures to the test through volatile cells). Console on USART2
// ALT1 (PF4/PF5) at 460800. Holds are counted by delay_us (crystal):
// the Ticker's OSC32K is +-10 % and is itself measured (test 7).
//
// Pins used (bench board): PD0 = TCA0 WO0 (generator), PC0 = TCB2 WO
// (Pwm8), PB5 = TCB3 WO (one-shot; LED2 blue on the traffic bench - it
// flickers), PC1 = a level the test drives for the direction input,
// PB0/PB1 = LUT4 inputs driven by the test, PB3 = LUT4 output (LED1
// R/G and LED2 R flicker), PD6 = DAC0 -> AC AINP3 (internal to the
// pin). Nothing to wire. Event channels: 0 PB5, 1 software / LUT4,
// 2 PD0, 3 PC1 or PC0, 4/5 internal generators - re-sourced per test.
//
// Commands: ? | 1 frequency | 2 PWM16 duty | 3 heartbeat | 4 one-shot
// | 5 Pwm8 | 6 pulse counter | 7 32-bit cascade | 8 tick + timeout
// | 9 event counter | c CCL | m comparators | a all
//
// Tests:
//   1  FrequencyGenerator (TCA0, PD0) at 500 Hz / 1 kHz / 10 kHz /
//      100 kHz -> FrequencyMeter (TCB0 via EvPin PD0): period in ticks
//      = 24 MHz / f within 1 tick; set_hz under a running output; the
//      TCB clocked by CLK_TCA (div64): ticks scale by 64; with SYNCUPD
//      the capture aligned with the TCA's TOP reads the restart (0/1);
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
//      reset; the Ticker's second measured in crystal ticks (finding);
//   8  PeriodicTick (TCB0) at 1000 Hz: its CAPT event measured by TCB1 =
//      24000 ticks +-1, interrupts counted over 200 ms;
//      Timeout (TCB1) of 1 ms watching PD0: with 100 Hz pulses 2 ms
//      high CAPT fires once per period; with 500 us pulses never;
//   9  EventCounter (TCA1) counting TCB0 CAPT events (PeriodicTick 1 kHz)
//      for 200 ms: 200 +-2; direction from PC1 level: counts down.
//   c  CCL: LUT4 as AND of PB0/PB1 (driven by the test) read back on
//      PB3 - the four input patterns, then OR after a reconfiguration
//      (whole block disabled: errata 2.4.1); ToggleFlipFlop<0> (LUT0/1)
//      toggled by TCB1's 1 kHz CAPT event: LUT0's output event measured
//      by TCB0 = 500 Hz (48000 ticks); LUT4 as a sync + edge detector
//      on the same event: one-clock pulses counted by TCB2 = 1000/s;
//   m  AC: DAC0 on PD6 against Threshold<Ac<0>> at 1000 mV (ref 2.048):
//      below/above states; the up and down crossings found by sweeping
//      the DAC (offset within 20 mV, the medium hysteresis ~25 mV -
//      findings); interrupts per crossing; Window<Ac<0>, Ac<2>> 500..
//      1500 mV: below / inside / above; AC0's event on a channel -> EVOUT
//      is not checked (no analyzer in the loop).
// Not testable here: the TCA1 PORTC three-channel route and TCB WO
// ALT1 pins (console), RUNSTDBY (no sleep in this suite), the CCL
// filter delay in clocks (no capture of it), AC response time (the
// DAC's slew dominates), TCD.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/ac.hpp"
#include "avrdx/ccl.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/dac.hpp"
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
void tcb1_tick() { PeriodicTick<T1>::tick(); }
void tcb1_frequency() { cap_a = FrequencyMeter<T1>::period_ticks(); ++captures; }

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
// Holds are counted in CRYSTAL time (delay_us, a cycle loop), not by
// the Ticker: the RTC runs from the internal OSC32K (+-10 %; bench:
// +0.9 % fast on this board) and the suite measures the timers against
// the crystal they share. A cycle loop is stretched by ISR time, so the
// PIT is paused for the whole suite and the ISRs of the test under way
// are the only ones (a few per mille at most).
void hold_ms(uint32_t ms) {
    while (ms--) delay_us(clock, 1000);
}
// Wait for at least n captures, at most ms milliseconds.
bool wait_captures(uint16_t n, uint32_t ms) {
    while (ms--) {
        if (captures >= n) return true;
        delay_us(clock, 1000);
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
    print(serial, "  set_hz(2000) under run: period ticks=", cap_a, " expect 12000, actual_hz=", Gen::actual_hz(), crlf);
    verdict("set_hz under run: 2 kHz within 1 tick", near(cap_a, 12000, 1));
    // the TCB clocked by the TCA's prescaled clock
    Gen::stop();
    Tca<0>::init({.mode = TcaMode::frequency, .clock = TcaClock::div64, .compare0 = 374,
                  .outputs = 0x01, .route = 'D'});        // 24e6 / (2*64*375) = 500 Hz
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChGen{}, TcbClock::tca0);
    clear_captures();
    wait_captures(3, 100);
    print(serial, "  CLK_TCA (div64) clocked TCB: period ticks=", cap_a, " expect 750", crlf);
    verdict("TCB on CLK_TCA: 500 Hz = 750 ticks of CLK_PER/64", near(cap_a, 750, 1));
    // ... and restarting with the TCA (SYNCUPD): the TCB restarts at
    // the TCA's TOP, which IS the edge it captures - it reads 0 (bench)
    T0::init({.mode = TcbMode::frequency, .clock = TcbClock::tca0, .compare = 0,
              .event_input = true, .sync_update = true});
    T0::capture_on(ChGen{});
    T0::enable_capt_interrupt(true);
    clear_captures();
    wait_captures(3, 100);
    print(serial, "  with SYNCUPD (restart at the TCA's TOP = the captured edge): ", cap_a, " (finding: 1 = counter just restarted)", crlf);
    verdict("SYNCUPD aligned capture reads the restart", cap_a <= 1);
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
    tcb0_hook = tcb0_duty;
    DutyMeter<T0>::init(clock, ChGen{});
    for (uint16_t d : duties) {
        Pwm16::duty<0>(d);
        hold_ms(3);                                   // buffered: lands at BOTTOM
        clear_captures();
        const bool got = wait_captures(2, 50);
        const uint16_t period = cap_a, width = cap_b;
        const uint16_t pm = DutyMeter<T0>::duty_permille({period, width});
        print(serial, "  duty ", d, "/24000: period=", period, " width=", width, " = ", pm, " permille (", captures, " captures", got ? "" : " - TIMED OUT", ")", crlf);
        verdict("period 24000 +-1", near(period, 24000, 1));
        verdict("duty within 2 permille", near(pm, d / 24, 2));
    }
    tcb0_hook = tcb0_width;
    PulseWidthMeter<T0>::init(clock, ChGen{});
    for (uint16_t d : duties) {
        Pwm16::duty<0>(d);
        hold_ms(3);
        clear_captures();
        wait_captures(2, 50);
        print(serial, "  duty ", d, ": PulseWidthMeter width=", cap_a, crlf);
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
    hold_ms(1000);
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
    // the Ticker (RTC from the internal OSC32K) against the crystal: a finding
    Ticker::resume();
    hold_ms(5);
    Wide::reset();
    const uint32_t m0 = Ticker::millis();
    while (Ticker::millis() - m0 < 1000) {}
    const uint32_t ticker_s = Wide::read();
    Ticker::pause();
    const int32_t ppm = static_cast<int32_t>((static_cast<int64_t>(24'000'000) - ticker_s) * 1'000'000 / 24'000'000);
    print(serial, "  Ticker 1000 ms = ", ticker_s, " crystal ticks: Ticker runs ", ppm >= 0 ? "+" : "", ppm, " ppm (OSC32K; finding, informative)", crlf);
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
    ChCarry::source(T0::CaptEvent{});                // the tick as an event ...
    tcb1_hook = tcb1_frequency;
    FrequencyMeter<T1>::init(clock, ChCarry{});      // ... measured by TCB1 against the same crystal
    clear_captures();
    wait_captures(3, 100);
    const uint16_t period = cap_a;
    irq_count = 0;
    hold_ms(200);
    const uint16_t n = irq_count;
    print(serial, "  tick period: ", period, " ticks (expect 24000); ", n, " interrupts in 200 ms", crlf);
    verdict("tick period 24000 +-1", near(period, 24000, 1));
    verdict("interrupts arrive (200 +-3)", near(n, 200, 3));
    PeriodicTick<T0>::stop();
    T1::disable(); tcb1_hook = nullptr; ChCarry::off();
    // Timeout: start on the rising edge of PD0, CAPT if 1 ms passes before the falling
    ChGen::source(EvPin<GenPin>{});
    tcb1_hook = tcb1_timeout;
    verdict("Timeout init 1 ms", Timeout<T1>::init(clock, 1000, ChGen{}));
    Beat::init(clock, 100, 0x01, false);         // 100 Hz, a pulse on PD0 ...
    Beat::pulse_us<0>(2000);                     // ... 2 ms high: longer than the time-out
    hold_ms(15);
    clear_captures();
    hold_ms(105);
    const uint16_t slow = captures;
    Beat::pulse_us<0>(500);                      // 500 us high: never times out
    hold_ms(15);
    clear_captures();
    hold_ms(105);
    const uint16_t fast = captures;
    print(serial, "  timeouts: 2 ms highs -> ", slow, " in 105 ms, 500 us highs -> ", fast, crlf);
    verdict("2 ms highs time out once per period (10 +-1)", near(slow, 10, 1));
    verdict("500 us highs never time out", fast == 0);
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

// ---- c CCL ---------------------------------------------------------------------
using LutA = Pin<'B', 0>;
using LutB = Pin<'B', 1>;
using LutOut = Pin<'B', 3>;

void tc_ccl() {
    print(serial, "c CCL: LUT4 AND/OR of PB0/PB1 on PB3; ToggleFlipFlop LUT0/1 by TCB1 events; edge detector", crlf);
    quiesce();
    LutA::clear(); LutB::clear();
    LutA::output(); LutB::output();
    Ccl::disable();
    verdict("LUT4 init (AND)", Lut<4>::init({.in0 = LutInput::pin, .in1 = LutInput::pin,
                                             .truth = lut_truth([](bool a, bool b, bool) { return a && b; }),
                                             .output_pin = true}));
    Ccl::enable();
    bool ok = true;
    for (uint8_t k = 0; k < 4; ++k) {
        if (k & 1) LutA::set(); else LutA::clear();
        if (k & 2) LutB::set(); else LutB::clear();
        delay_us(clock, 2);
        const bool out = LutOut::read();
        ok = ok && (out == (k == 3));
        print(serial, "  AND ", k & 1, " ", (k >> 1) & 1, " -> ", out, crlf);
    }
    verdict("AND truth table on the pins", ok);
    // reconfigure to OR: the whole block off meanwhile (errata 2.4.1)
    Ccl::disable();
    Lut<4>::init({.in0 = LutInput::pin, .in1 = LutInput::pin,
                  .truth = lut_truth([](bool a, bool b, bool) { return a || b; }), .output_pin = true});
    Ccl::enable();
    ok = true;
    for (uint8_t k = 0; k < 4; ++k) {
        if (k & 1) LutA::set(); else LutA::clear();
        if (k & 2) LutB::set(); else LutB::clear();
        delay_us(clock, 2);
        ok = ok && (LutOut::read() == (k != 0));
    }
    verdict("OR after reconfiguration", ok);
    // flip-flop toggled by a 1 kHz event: 500 Hz on LUT0's output event
    Ccl::disable();
    tcb1_hook = tcb1_tick;
    PeriodicTick<T1>::init(clock, 1000);
    ChCarry::source(T1::CaptEvent{});
    ToggleFlipFlop<0>::init(ChCarry{});
    // edge detector: LUT4 = sync + edge of the same event, one-clock pulses
    Lut<4>::init({.in0 = LutInput::event_a, .truth = lut_truth([](bool a, bool, bool) { return a; }),
                  .filter = LutFilter::sync, .edge_detect = true});
    Lut<4>::event_a_on(ChCarry{});
    Ccl::enable();
    ChSnap::source(Lut<0>::OutEvent{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChSnap{});
    ChSoft::source(Lut<4>::OutEvent{});
    PulseCounter<T2>::init(ChSoft{});
    PulseCounter<T2>::reset();
    clear_captures();
    const bool got = wait_captures(3, 100);
    print(serial, "  flip-flop output period: ", cap_a, " ticks (expect 48000)", crlf);
    verdict("JK toggles once per event: 500 Hz", got && near(cap_a, 48000, 2));
    PulseCounter<T2>::reset();
    hold_ms(200);
    const uint16_t pulses = PulseCounter<T2>::count();
    print(serial, "  edge-detector pulses in 200 ms: ", pulses, crlf);
    verdict("one pulse per event (200 +-2)", near(pulses, 200, 2));
    Ccl::disable();
    LutA::input(); LutB::input();
    quiesce();
}

// ---- m comparators ----------------------------------------------------------------
using D = Dac<0>;
constexpr uint16_t ac_ref_mv = 2048;
volatile uint16_t ac_irqs = 0;

void tm_comparators() {
    print(serial, "m AC: DAC0/PD6 -> AC0 AINP3; Threshold 1000 mV, Window 500..1500 mV (ref 2.048)", crlf);
    quiesce();
    D::init({.reference = Ref::v2048});
    D::set_mv(500, ac_ref_mv);
    verdict("Threshold init", Threshold<Ac<0>>::init(AcPos::ainp3, 1000, Ref::v2048));
    hold_ms(2);
    ac_irqs = 0;
    verdict("500 mV: below", !Threshold<Ac<0>>::above());
    D::set_mv(1500, ac_ref_mv);
    hold_ms(2);
    verdict("1500 mV: above", Threshold<Ac<0>>::above());
    verdict("one interrupt on the rising crossing", ac_irqs == 1);
    // sweep up from 900 to find the crossing, then down
    D::set_mv(900, ac_ref_mv);
    hold_ms(2);
    uint16_t up = 0, down = 0;
    for (uint16_t mv = 900; mv <= 1100; ++mv) {
        D::set_mv(mv, ac_ref_mv);
        delay_us(clock, 200);
        if (Threshold<Ac<0>>::above()) { up = mv; break; }
    }
    for (uint16_t mv = 1100; mv >= 900; --mv) {
        D::set_mv(mv, ac_ref_mv);
        delay_us(clock, 500);                      // the DAC falls slowly on a bare pin
        if (!Threshold<Ac<0>>::above()) { down = mv; break; }
    }
    print(serial, "  crossings: up at ", up, " mV, down at ", down, " mV (hysteresis ", up - down, " mV) - findings", crlf);
    verdict("up crossing within 1000 +-30 mV (offset + hysteresis)", near(up, 1000, 30));
    verdict("hysteresis medium: 10..45 mV", up > down && up - down >= 10 && up - down <= 45);
    Ac<0>::enable_interrupt(false);
    // window
    verdict("Window init", Window<Ac<0>, Ac<2>>::init(AcPos::ainp3, 500, 1500, AcWindowSense::outside, Ref::v2048));
    D::set_mv(300, ac_ref_mv);  hold_ms(2);
    const auto s1 = Window<Ac<0>, Ac<2>>::state();
    D::set_mv(1000, ac_ref_mv); hold_ms(2);
    const auto s2 = Window<Ac<0>, Ac<2>>::state();
    D::set_mv(1800, ac_ref_mv); hold_ms(2);
    const auto s3 = Window<Ac<0>, Ac<2>>::state();
    print(serial, "  window states: ", static_cast<uint8_t>(s1), " ", static_cast<uint8_t>(s2), " ", static_cast<uint8_t>(s3), " (2 below, 1 inside, 0 above)", crlf);
    verdict("below / inside / above", s1 == AcWindowState::below && s2 == AcWindowState::inside && s3 == AcWindowState::above);
    Ac<0>::enable_interrupt(false);
    Ac<0>::disable(); Ac<2>::disable();
    D::set(0);
    quiesce();
}

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_frequency}, {'2', t2_pwm16}, {'3', t3_heartbeat}, {'4', t4_one_shot},
    {'5', t5_pwm8}, {'6', t6_pulse_counter}, {'7', t7_cascade}, {'8', t8_tick_timeout},
    {'9', t9_event_counter}, {'c', tc_ccl}, {'m', tm_comparators},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_timer: 1 frequency | 2 PWM16 duty | 3 heartbeat | 4 one-shot | 5 Pwm8 | "
                  "6 pulse counter | 7 32-bit cascade | 8 tick+timeout | 9 event counter | c CCL | m comparators | a all", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(TCB0_INT_vect) { if (tcb0_hook) tcb0_hook(); else (void)T0::take_flags(); }
ISR(TCB1_INT_vect) { if (tcb1_hook) tcb1_hook(); else (void)T1::take_flags(); }
ISR(TCA0_OVF_vect) { Beat::beat(); ++beats; }
ISR(AC0_AC_vect) { (void)Ac<0>::cmp(); ++ac_irqs; }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    Ticker::pause();                      // no PIT ISR inflating the crystal-time holds (test 7 resumes it briefly)
    sei();
    print(serial, crlf, "test_avr_timer - TCA/TCB/CCL/AC test suite (clk=", xtal ? "XTAL" : "OSCHF",
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
