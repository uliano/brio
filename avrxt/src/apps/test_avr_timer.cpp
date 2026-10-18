// test_avr_timer - the TCA/TCB/CCL/AC test SUITE for the AVR DA/DB
// target: the TCA tasks generate known waveforms, the TCB tasks measure
// them back through the event system (pin-level generators on the
// driven pins: no jumper wires), counters count known event rates
// against the RTC Ticker; the CCL is checked on pins and with a timer
// event through a flip-flop, the comparators against the DAC on PD6.
// Reference test of avrxt/tca.hpp, tcb.hpp, ccl.hpp, ac.hpp
// (docs/avrxt/tca.md, tcb.md, ccl.md, ac.md): keep it passing.
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
// | 9 event counter | c CCL | m comparators | n timeout duration
// | o meter variants | r live rebase | s TCA split escape
// | d centered PWM | l CCL logic | i CCL inputs | w AC extras
// | t TCA modes | e EVSYS generators and positions | k CCL extras | a all
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
//      findings); interrupts per crossing; Window<Ac<0>, AcUp> 500..
//      1500 mV: below / inside / above; AC0's event on a channel -> EVOUT
//      is not checked (no analyzer in the loop).
//   t  TCA's other modes: dual slope with OVF at TOP (PER + CMP after
//      the rising edge) and at BOTH (two a period); the split mode's
//      LUNF/HUNF/LCMP0 vectors on TCA1 against the LUNF event counted
//      by TCB1; event B's three restarts and event A's any-edge and
//      while-high counting on PC1's level; an FRQ WO2 phase-shifted by
//      CMP2 (PD2); the UPDATE command over LUPD; ALUPD; TCA1's PORTC
//      route read back on PC4..PC6.
//   x  the AVR EA's own (its board only): TCA1's PORTA and PORTD routes
//      parked and read back; RESTART against DIR; Pwm8's duty through a
//      one-byte CCMPH write; the AC code on LUT input 2 (AC1 there); SPI0
//      on LUT inputs 0/1 (MOSI or MISO); LINK into LUT3; one LUT
//      reconfigured under a running block; TCA1 into a LUT with no pin.
//
// On the AVR EA board (board type ea48) the letters keep their claims
// with what that chip and board have: four LUTs (letter c's pins are
// LUT2's PD0/PD1 -> PD3, the pinless LUT the letters use LUT3), AC0 and
// AC1 (the window's upper comparator and the PC6 output are AC1's), and
// every tick count derived from the board's 20 MHz.
// Not testable here: the TCB0/TCB1 ALT1 pins (they are the console;
// TCB2's ALT1 = PB4 is tested), RUNSTDBY (no sleep in this suite:
// test_avr_sleep o), AC response time (the DAC's slew dominates), TCD.

// THE AVR64EA48 builds the suite as two images, held to the ATmega4809's
// 48 KB (the groups line below; docs/avrxt/README.md, "The group axis"):
// the TCA and TCB letters in one, the CCL, AC and event-system letters
// with the EA's own x in the other; the 128 KB DA and DB build one.
//
// build: boards = db48,da48,ea48
// build: groups = 123456789norsdt,cmliwekx
// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>
#include <type_traits>

#include "avrxt/ac.hpp"
#include "avrxt/ccl.hpp"
#include "avrxt/clock.hpp"
#include "avrxt/dac.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/pin.hpp"
#include "avrxt/spi.hpp"
#include "avrxt/tca.hpp"
#include "avrxt/tcb.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "avrxt/userrow.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#include "avr_board.hpp"

using SysClock = BoardClock;   // the board's HF source (avr_board.hpp)
constexpr SysClock clock;

/// The letters were first written at 24 MHz and keep their literal text
/// there; at another board rate (the AVR EA board's 20 MHz) the same
/// verdicts are derived from the rate: CLK_PER ticks in a millisecond
/// (24000 / 20000) and in a microsecond (24 / 20).
constexpr bool at24 = SysClock::hz == 24'000'000u;
constexpr uint32_t tpms = SysClock::hz / 1000u;
constexpr uint32_t mhz = SysClock::hz / 1'000'000u;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

// ---- the actors --------------------------------------------------------------
using GenPin = Pin<'D', 0>;          // TCA0 WO0
using Pwm8Pin = Pin<'C', 0>;         // TCB2 WO
using ShotPin = Pin<'B', 5>;         // TCB3 WO
using DirPin = Pin<'C', 1>;          // a level the test drives

#if defined(BRIO_BOARD_ea48)
// The AVR EA has four LUTs and two comparators: the pinless LUT the
// letters use is LUT3 (LUT4 on the DA/DB48), the window's upper
// comparator and the PC6 output AC1 (AC2 there).
using LutX = Lut<3>;
using AcUp = Ac<1>;
#else
using LutX = Lut<4>;
using AcUp = Ac<2>;
#endif
/// A pin generator's slot for a SECOND pin of a port already routed: the
/// AVR EA's port has two (evsys.md), the DA/DB's every pin its own.
constexpr uint8_t slot_b = has_port_evgen ? 1 : 0;

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

void tcb0_frequency() { cap_a = FrequencyMeter<T0>::period_ticks(); captures = captures + 1; }
#if defined(BRIO_BOARD_ea48)
volatile uint32_t mean_sum = 0;              // letter k's mean of consecutive periods
void tcb0_frequency_mean() {
    const uint16_t t = FrequencyMeter<T0>::period_ticks();
    cap_a = t;
    mean_sum = mean_sum + t;
    captures = captures + 1;
}
#endif
void tcb0_duty() { const auto r = DutyMeter<T0>::reading(); cap_a = r.period_ticks; cap_b = r.width_ticks; captures = captures + 1; }
void tcb0_width() { cap_a = PulseWidthMeter<T0>::width_ticks(); captures = captures + 1; }
void tcb0_tick() { PeriodicTick<T0>::tick(); irq_count = irq_count + 1; }
void tcb1_width() { cap_a = PulseWidthMeter<T1>::width_ticks(); captures = captures + 1; }
void tcb1_timeout() { Timeout<T1>::expired(); captures = captures + 1; }
void tcb1_tick() { PeriodicTick<T1>::tick(); }
void tcb1_frequency() { cap_a = FrequencyMeter<T1>::period_ticks(); captures = captures + 1; }

void clear_captures() {
    cli();
    cap_b = 0;
    cap_a = cap_b;
    captures = 0;
    irq_count = 0;
    sei();
}

// ---- the bench ---------------------------------------------------------------
// 'a' runs every letter this image carries (the suite's all-key).
TestBench<Serial> bench('a');

void verdict(const char* name, bool ok) { bench.verdict(name, ok); }
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
// Holds are counted in CRYSTAL time (delay_us, a cycle loop), not by
// the Ticker: the RTC runs from the internal OSC32K (+-10 %; measured
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
    if constexpr (!at24) {
        // The captures of the 100 kHz before the change lands count too:
        // at 20 MHz the meter's interrupt every 10 us slows the main
        // context enough that five of them arrive before set_hz() returns
        // (measured: the old period read). Let the buffered CMP0 land,
        // then count. The 24 MHz images keep their text; the race is
        // theirs too, and the fix waits for a run on those boards.
        hold_ms(2);
        clear_captures();
    }
    wait_captures(5, 100);
    if constexpr (at24) {
        print(serial, "  set_hz(2000) under run: period ticks=", cap_a, " expect 12000, actual_hz=", Gen::actual_hz(), crlf);
    } else {
        print(serial, "  set_hz(2000) under run: period ticks=", cap_a, " expect ", SysClock::hz / 2000u,
              ", actual_hz=", Gen::actual_hz(), crlf);
    }
    verdict("set_hz under run: 2 kHz within 1 tick", near(cap_a, SysClock::hz / 2000u, 1));
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
    // the TCA's TOP, which is the edge it captures - it reads 0 (measured)
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
using Pwm16 = TcaPwm16<0, 'D', tpms>;       // 1 kHz at div1

void t2_pwm16() {
    if constexpr (at24) {
        print(serial, "2 TcaPwm16 TCA0/PD0 (24000 steps) -> DutyMeter / PulseWidthMeter TCB0", crlf);
    } else {
        print(serial, "2 TcaPwm16 TCA0/PD0 (", tpms, " steps) -> DutyMeter / PulseWidthMeter TCB0", crlf);
    }
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    Pwm16::init(TcaClock::div1, 0x01);
    constexpr uint16_t duties[] = {tpms / 4u, tpms / 2u, 3u * tpms / 4u};
    tcb0_hook = tcb0_duty;
    DutyMeter<T0>::init(clock, ChGen{});
    for (uint16_t d : duties) {
        Pwm16::duty<0>(d);
        hold_ms(3);                                   // buffered: lands at BOTTOM
        clear_captures();
        const bool got = wait_captures(2, 50);
        const uint16_t period = cap_a, width = cap_b;
        const uint16_t pm = DutyMeter<T0>::duty_permille({period, width});
        if constexpr (at24) {
            print(serial, "  duty ", d, "/24000: period=", period, " width=", width, " = ", pm, " permille (", captures, " captures", got ? "" : " - TIMED OUT", ")", crlf);
        } else {
            print(serial, "  duty ", d, "/", tpms, ": period=", period, " width=", width, " = ", pm, " permille (", captures, " captures", got ? "" : " - TIMED OUT", ")", crlf);
        }
        verdict(at24 ? "period 24000 +-1" : "period 1 ms of CLK_PER +-1", near(period, tpms, 1));
        verdict("duty within 2 permille", near(pm, d / static_cast<int>(mhz), 2));
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
    Pwm16::duty<0>(tpms);
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
        verdict("width within 2 ticks", near(cap_a, static_cast<int32_t>(w * mhz), 2));
        if (w >= 100) verdict("busy() right after fire()", busy);
    }
    // retrigger ignored during the pulse
    Shot::init(clock, 1000, ChSoft{});
    clear_captures();
    Shot::fire();
    delay_us(clock, 200);
    Shot::fire();
    hold_ms(5);
    verdict("second fire() during the pulse ignored (one capture)", captures == 1 && near(cap_a, tpms, 2));
    // async: WO rises on the event itself
    Shot::init(clock, 100, ChSoft{}, {.async = true});
    clear_captures();
    Shot::fire();
    wait_captures(1, 20);
    print(serial, "  async 100 us: width=", cap_a, " ticks", crlf);
    verdict("async pulse 100 us within 4 ticks", near(cap_a, tpms / 10u, 4));
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
    if constexpr (at24) {
        print(serial, "  reads: ", a, " ", b, " ", c, " (1 s = 24000000 ticks)", crlf);
    } else {
        print(serial, "  reads: ", a, " ", b, " ", c, " (1 s = ", SysClock::hz, " ticks)", crlf);
    }
    verdict("1 s within 0.2 % (RTC millis vs crystal)", near(static_cast<int32_t>(b - a), SysClock::hz, SysClock::hz / 500u));
    verdict("monotonic", c >= b && b >= a);
    // the Ticker (RTC from the internal OSC32K) against the crystal: a finding
    Ticker::resume();
    hold_ms(5);
    Wide::reset();
    const uint32_t m0 = Ticker::millis();
    while (Ticker::millis() - m0 < 1000) {}
    const uint32_t ticker_s = Wide::read();
    Ticker::pause();
    const int32_t ppm = static_cast<int32_t>((static_cast<int64_t>(SysClock::hz) - ticker_s) * 1'000'000 / static_cast<int32_t>(SysClock::hz));
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
    if constexpr (at24) {
        print(serial, "  tick period: ", period, " ticks (expect 24000); ", n, " interrupts in 200 ms", crlf);
    } else {
        print(serial, "  tick period: ", period, " ticks (expect ", tpms, "); ", n, " interrupts in 200 ms", crlf);
    }
    verdict(at24 ? "tick period 24000 +-1" : "tick period 1 ms of CLK_PER +-1", near(period, tpms, 1));
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
#if defined(BRIO_BOARD_ea48)
// The AVR EA board: LUT2, IN0 = PD0, OUT = PD3, and the second input
// LUT2's IN1 = PD1 - or, where PD1 is on the board's analog net
// (avr_board.hpp), its IN2 = PD2: the net carries 96 nF (VREFA's
// capacitor, test_avr_analog letter n), which a pad charges with a time
// constant of microseconds - past the 2 us the letter reads its output
// after a change (measured: the OR's 0 -> 1 on PD1 read 0 at 2 us, 1 at
// 200 us).
using LutP = Lut<2>;
using LutA = Pin<'D', 0>;
using LutB = std::conditional_t<board_analog_net, Pin<'D', 2>, Pin<'D', 1>>;
using LutOut = Pin<'D', 3>;
/// The two-input LUT of the letter on the second input's position.
template <typename F>
constexpr LutConfig lut_pair(F f) {
    if constexpr (board_analog_net) {
        return {.in0 = LutInput::pin, .in2 = LutInput::pin,
                .truth = lut_truth([f](bool a, bool, bool c) { return f(a, c); }), .output_pin = true};
    } else {
        return {.in0 = LutInput::pin, .in1 = LutInput::pin,
                .truth = lut_truth([f](bool a, bool b, bool) { return f(a, b); }), .output_pin = true};
    }
}
#else
using LutP = Lut<4>;
using LutA = Pin<'B', 0>;
using LutB = Pin<'B', 1>;
using LutOut = Pin<'B', 3>;
#endif

void tc_ccl() {
#if defined(BRIO_BOARD_ea48)
    print(serial, "c CCL: LUT2 AND/OR of PD0/", board_analog_net ? "PD2" : "PD1",
          " on PD3; ToggleFlipFlop LUT0/1 by TCB1 events; edge detector", crlf);
    quiesce();
    Tca<0>::reset();                 // PD0 is TCA0's WO0: an enabled compare output owns it even disabled
#else
    print(serial, "c CCL: LUT4 AND/OR of PB0/PB1 on PB3; ToggleFlipFlop LUT0/1 by TCB1 events; edge detector", crlf);
    quiesce();
#endif
    LutA::clear(); LutB::clear();
    LutA::output(); LutB::output();
    Ccl::disable();
#if defined(BRIO_BOARD_ea48)
    verdict("LUT2 init (AND)", LutP::init(lut_pair([](bool a, bool b) { return a && b; })));
#else
    verdict(lut_count == 6 ? "LUT4 init (AND)" : "LUT2 init (AND)", LutP::init({.in0 = LutInput::pin, .in1 = LutInput::pin,
                                             .truth = lut_truth([](bool a, bool b, bool) { return a && b; }),
                                             .output_pin = true}));
#endif
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
#if defined(BRIO_BOARD_ea48)
    LutP::init(lut_pair([](bool a, bool b) { return a || b; }));
#else
    LutP::init({.in0 = LutInput::pin, .in1 = LutInput::pin,
                  .truth = lut_truth([](bool a, bool b, bool) { return a || b; }), .output_pin = true});
#endif
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
    // edge detector: LUT4 (LUT3 on the AVR EA) = sync + edge of the same
    // event, one-clock pulses
    LutX::init({.in0 = LutInput::event_a, .truth = lut_truth([](bool a, bool, bool) { return a; }),
                  .filter = LutFilter::sync, .edge_detect = true});
    LutX::event_a_on(ChCarry{});
    Ccl::enable();
    ChSnap::source(Lut<0>::OutEvent{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChSnap{});
    ChSoft::source(LutX::OutEvent{});
    PulseCounter<T2>::init(ChSoft{});
    PulseCounter<T2>::reset();
    clear_captures();
    const bool got = wait_captures(3, 100);
    if constexpr (at24) {
        print(serial, "  flip-flop output period: ", cap_a, " ticks (expect 48000)", crlf);
    } else {
        print(serial, "  flip-flop output period: ", cap_a, " ticks (expect ", 2u * tpms, ")", crlf);
    }
    verdict("JK toggles once per event: 500 Hz", got && near(cap_a, 2u * tpms, 2));
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
    verdict("Window init", Window<Ac<0>, AcUp>::init(AcPos::ainp3, 500, 1500, AcWindowSense::outside, Ref::v2048));
    D::set_mv(300, ac_ref_mv);  hold_ms(2);
    const auto s1 = Window<Ac<0>, AcUp>::state();
    D::set_mv(1000, ac_ref_mv); hold_ms(2);
    const auto s2 = Window<Ac<0>, AcUp>::state();
    D::set_mv(1800, ac_ref_mv); hold_ms(2);
    const auto s3 = Window<Ac<0>, AcUp>::state();
    print(serial, "  window states: ", static_cast<uint8_t>(s1), " ", static_cast<uint8_t>(s2), " ", static_cast<uint8_t>(s3), " (2 below, 1 inside, 0 above)", crlf);
    verdict("below / inside / above", s1 == AcWindowState::below && s2 == AcWindowState::inside && s3 == AcWindowState::above);
    Ac<0>::enable_interrupt(false);
    Ac<0>::disable(); AcUp::disable();
    D::set(0);
    quiesce();
}

// ---- n Timeout duration ----------------------------------------------------------
// Does Timeout's CAPT fire exactly TOP ticks after the start edge ("at
// least us")? Measured all-internally: T0 free-runs on CLK_PER and
// stamps CNT twice through the same sync CAPT-in user - on the start
// edge (ChGen) and on the DUT's CAPT event (ChCarry); the difference
// is the duration. The event topology is identical for the one-shot,
// whose width is known exact (test 4): its measured offset calibrates
// the fixed event/sync skew, and the timeout must show the SAME offset
// over its own TOP.
volatile uint16_t stamp_start = 0, stamp_end = 0;
volatile uint8_t stamp_phase = 0;
void tcb0_stamp() {
    const uint16_t v = T0::capture();
    if (stamp_phase == 0) { stamp_start = v; T0::capture_on(ChCarry{}); stamp_phase = 1; }
    else { stamp_end = v; stamp_phase = 2; T0::capture_on_events(false); }
}

uint16_t measure_duration() {                    // 0xFFFF = stamps missed
    // Arm only while PD0 is LOW: a sync CAPT-in enabled on a channel
    // already high sees a spurious rising edge (its edge detector
    // starts from 0) and the stamps pair with the wrong events.
    for (uint16_t i = 0; i < 30000 && GenPin::read(); ++i) {}
    T1::clear_capt(); T3::clear_capt();
    stamp_phase = 0;
    // The CAPT user still listens where the LAST measurement left it (the
    // second stamp's channel): point it at the start channel BEFORE the
    // init enables the event input, or an event on the old channel can be
    // the first stamp (measured once in twelve runs: two OVFs stamped,
    // 12000 apart, in letter t's dual slope at BOTH).
    T0::capture_on(ChGen{});
    T0::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0,
              .event_input = true});
    T0::capture_on(ChGen{});
    T0::enable_capt_interrupt(true);
    for (uint8_t i = 0; i < 40 && stamp_phase != 2; ++i) hold_ms(1);
    T0::disable();
    return stamp_phase == 2 ? static_cast<uint16_t>(stamp_end - stamp_start) : 0xFFFF;
}

void tn_timeout_exact() {
    print(serial, "n Timeout duration: T0 stamps the PD0 edge and the CAPT event; the one-shot calibrates the path", crlf);
    quiesce();
    tcb0_hook = tcb0_stamp;
    ChGen::source(EvPin<GenPin>{});
    Beat::init(clock, 100, 0x01, false);         // rising edge on PD0 every 10 ms ...
    Beat::pulse_us<0>(2000);                     // ... high for 2 ms

    verdict("one-shot init 100 us", Shot::init(clock, 100, ChGen{}));
    ChCarry::source(T3::CaptEvent{});
    const uint16_t d1 = measure_duration(), d2 = measure_duration(), d3 = measure_duration();
    if constexpr (at24) {
        print(serial, "  one-shot 2400 ticks: measured ", d1, " ", d2, " ", d3, crlf);
    } else {
        print(serial, "  one-shot ", tpms / 10u, " ticks: measured ", d1, " ", d2, " ", d3, crlf);
    }
    verdict("deterministic (three equal)", d1 == d2 && d2 == d3 && d1 != 0xFFFF);
    const int32_t c = static_cast<int32_t>(d1) - static_cast<int>(tpms / 10u);
    print(serial, "  event-path offset c = ", c, " ticks", crlf);
    verdict("offset small (0..8 ticks)", c >= 0 && c <= 8);

    // the noise canceler measured: FILTER on the DUT's start path only
    // (the stamp path stays bare), so the whole measurement shifts by
    // the canceler's delay.
    verdict("one-shot init (filter)", Shot::init(clock, 100, ChGen{}, {.filter = true}));
    const uint16_t df1 = measure_duration(), df2 = measure_duration();
    print(serial, "  filtered: measured ", df1, " ", df2, " = bare + ", static_cast<int32_t>(df1) - static_cast<int32_t>(d1), crlf);
    // Four identical samples decide; the first coincides with the bare
    // sync path's sampling instant, so the DIFFERENTIAL delay is 3
    // (the datasheet's "four clock cycles" counts from the event).
    verdict("noise canceler adds 3 CLK_PER over the bare path",
            df1 == df2 && static_cast<int32_t>(df1) - static_cast<int32_t>(d1) == 3);
    T3::disable();

    for (const uint32_t us : {1000u, 500u}) {
        verdict("Timeout init", Timeout<T1>::init(clock, us, ChGen{}));
        ChCarry::source(T1::CaptEvent{});
        const uint16_t d = measure_duration();
        const int32_t top = static_cast<int32_t>(us) * static_cast<int>(mhz);
        print(serial, "  timeout ", us, " us (TOP ", top, "): measured ", d,
              " = TOP + ", static_cast<int32_t>(d) - top, crlf);
        verdict("fires TOP ticks after the start (offset = c)",
                static_cast<int32_t>(d) - top == c);
        T1::disable();
    }

    // edge = true: start on the falling edge, stop on the rising. With
    // the stop edge 9.5 ms away and TOP = 1 ms the counter FREE-RUNS
    // past TOP and CAPT re-fires at every wrap through TOP (measured:
    // 24000, then every 65536 ticks -> 4 per 9.5 ms low). In general
    // 1 + (9.5 - 1) ms / 65536 ticks: 1 + 3.11 -> 4 at 24 MHz, 1 + 2.59
    // -> 3 at 20 MHz; ten and a half periods in the window, +-3.
    constexpr int32_t wraps = 1 + static_cast<int32_t>((SysClock::hz / 1000u) * 17u / 2u / 65536u);
    tcb1_hook = tcb1_timeout;
    Beat::pulse_us<0>(500);
    verdict("Timeout init (edge = true)", Timeout<T1>::init(clock, 1000, ChGen{}, true));
    hold_ms(15); clear_captures(); hold_ms(105);
    if constexpr (at24) {
        print(serial, "  reversed edges: ", captures, " timeouts in 105 ms (4 per period: TOP, then each CNT wrap)", crlf);
    } else {
        print(serial, "  reversed edges: ", captures, " timeouts in 105 ms (", wraps,
              " per period: TOP, then each CNT wrap)", crlf);
    }
    verdict(at24 ? "lows time out; CAPT re-fires at each wrap through TOP (40 +-3)"
                 : "lows time out; CAPT re-fires at each wrap through TOP (10 periods +-3)",
            near(captures, 10 * wraps, 3));
    quiesce();
}

// ---- o meter variants, OVF safety net, capture gate ------------------------------
void to_meter_variants() {
    print(serial, "o meter variants (falling/low/inverted), OVF safety net, the CAPT gate", crlf);
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    Beat::init(clock, 1000, 0x01, false);        // 1 kHz on PD0 ...
    Beat::pulse_us<0>(300);                      // ... 300 us high, 700 us low

    tcb0_hook = tcb0_width;
    PulseWidthMeter<T0>::init(clock, ChGen{}, TcbClock::div1, /*low=*/true);
    clear_captures(); wait_captures(3, 20);
    if constexpr (at24) {
        print(serial, "  low time: ", cap_a, " ticks (expect 16800)", crlf);
    } else {
        print(serial, "  low time: ", cap_a, " ticks (expect ", 700u * mhz, ")", crlf);
    }
    verdict(at24 ? "PW low = 700 us (16800 +-2)" : "PW low = 700 us (+-2)", near(cap_a, 700u * mhz, 2));
    T0::disable();

    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChGen{}, TcbClock::div1, /*falling=*/true);
    clear_captures(); wait_captures(3, 20);
    print(serial, "  period between falling edges: ", cap_a, " ticks", crlf);
    verdict(at24 ? "FRQ falling = 24000 +-1" : "FRQ falling = 1 ms +-1", near(cap_a, tpms, 1));
    T0::disable();

    tcb0_hook = tcb0_duty;
    DutyMeter<T0>::init(clock, ChGen{}, TcbClock::div1, /*inverted=*/true);
    clear_captures(); wait_captures(3, 20);
    print(serial, "  inverted: period ", cap_a, ", width (low) ", cap_b, crlf);
    verdict(at24 ? "inverted period 24000 +-1" : "inverted period 1 ms +-1", near(cap_a, tpms, 1));
    verdict(at24 ? "inverted width = low time (16800 +-2)" : "inverted width = low time (+-2)",
            near(cap_b, 700u * mhz, 2));
    T0::disable();

    // div2 and CLK_TCA1 as counter clocks, same 1 kHz signal
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChGen{}, TcbClock::div2);
    clear_captures(); wait_captures(3, 20);
    if constexpr (at24) {
        print(serial, "  div2 period: ", cap_a, " ticks (expect 12000)", crlf);
    } else {
        print(serial, "  div2 period: ", cap_a, " ticks (expect ", tpms / 2u, ")", crlf);
    }
    verdict(at24 ? "div2 period 12000 +-1" : "div2 period 0.5 ms +-1", near(cap_a, tpms / 2u, 1));
    T0::disable();
    Tca<1>::init({.mode = TcaMode::normal, .clock = TcaClock::div8, .period = 0xFFFF});
    FrequencyMeter<T0>::init(clock, ChGen{}, TcbClock::tca1);
    clear_captures(); wait_captures(3, 20);
    if constexpr (at24) {
        print(serial, "  CLK_TCA1 (div8, 3 MHz) period: ", cap_a, " ticks (expect 3000)", crlf);
    } else {
        print(serial, "  CLK_TCA1 (div8) period: ", cap_a, " ticks (expect ", tpms / 8u, ")", crlf);
    }
    // 24000 CLK_PER / 8 = 3000 ticks of CLK_TCA1, but the /8 prescaler's
    // phase against the edge is this run's own (TCA1 started at an
    // arbitrary instant), and the capture's restart lands on either side
    // of a prescaled tick: 3000 +-1 (measured 2999 once in twelve runs).
    verdict(at24 ? "tca1-clocked period 3000 +-1" : "tca1-clocked period 1 ms / 8 +-1", near(cap_a, tpms / 8u, 1));
    verdict("tick_hz unknown on a TCA clock (0)", FrequencyMeter<T0>::tick_hz() == 0);
    T0::disable(); Tca<1>::disable();

    // CCMPINIT: the idle WO level in the modes without one of their own
    T2::init({.mode = TcbMode::capture, .clock = TcbClock::div1,
              .output = true, .output_init_high = true});
    delay_us(clock, 2);
    verdict("CCMPINIT high on PC0", Pwm8Pin::read());
    T2::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .output = true});
    delay_us(clock, 2);
    verdict("CCMPINIT low on PC0", !Pwm8Pin::read());
    T2::disable();

    // the ALT1 route, on an instance whose ALT1 is NOT the console:
    // TCB2 -> PB4 (a traffic LED, harmless), read back through EvPin
    ChShot::source(EvPin<Pin<'B', 4>>{});
    FrequencyMeter<T0>::init(clock, ChShot{});
    verdict("Pwm8 on the ALT1 pin accepted", Pwm8<T2>::init(TcbClock::div1, /*alt_pin=*/true));
    Pwm8<T2>::duty(128);
    clear_captures(); wait_captures(3, 20);
    print(serial, "  ALT1 (PB4) period: ", cap_a, " ticks (expect 256)", crlf);
    verdict("ALT1 route outputs (period 256 +-1)", near(cap_a, 256, 1));
    Pwm8<T2>::duty(0); T2::disable(); T0::disable(); ChShot::off(); tcb0_hook = nullptr;

    // OVF safety net: TOP lowered below a running CNT (polled flags)
    T1::init({.mode = TcbMode::periodic, .clock = TcbClock::div1, .compare = 60000});
    while (T1::count() < 40000) {}
    T1::compare(1000);                           // TOP below CNT: to MAX first, OVF, then restart
    hold_ms(3);
    verdict("OVF fired (counter ran to MAX)", T1::ovf_flag());
    T1::clear_ovf(); T1::clear_capt();
    hold_ms(1);
    verdict("periodic resumed at the new TOP", T1::capt_flag());
    T1::disable();

    // the CAPT gate: capture_on_events(false) silences a Timeout
    Beat::init(clock, 100, 0x01, false);
    Beat::pulse_us<0>(2000);
    tcb1_hook = tcb1_timeout;
    verdict("Timeout init 1 ms", Timeout<T1>::init(clock, 1000, ChGen{}));
    hold_ms(15); clear_captures(); hold_ms(50);
    const uint16_t armed = captures;
    T1::capture_on_events(false);
    clear_captures(); hold_ms(50);
    const uint16_t gated = captures;
    T1::capture_on_events(true);
    clear_captures(); hold_ms(50);
    const uint16_t rearmed = captures;
    print(serial, "  timeouts: armed ", armed, ", gated ", gated, ", re-armed ", rearmed, crlf);
    verdict("armed counts (5 +-1)", near(armed, 5, 1));
    verdict("gated counts zero", gated == 0);
    verdict("re-armed counts again (5 +-1)", near(rearmed, 5, 1));
    quiesce();
}

// ---- r live rebase ---------------------------------------------------------------
// The whole clock fan-out under fire: a DynamicClock rebases the
// console, the signal source (Heartbeat keeps its 1 kHz) and the
// meter (its conversions follow), then the prescaler switches. The
// ticks must track CLK_PER while the microseconds stand still.
// Holds run on the static `clock`, so real time doubles at 12 MHz -
// harmless here.
using DynClock = DynamicClock<SysClock, Serial, Beat, FrequencyMeter<T0>>;

void tr_rebase_live() {
    if constexpr (at24) {
        print(serial, "r live rebase: 24 -> 12 -> 24 MHz under a running Heartbeat + FrequencyMeter", crlf);
    } else {
        print(serial, "r live rebase: ", mhz, " -> ", mhz / 2u, " -> ", mhz,
              " MHz under a running Heartbeat + FrequencyMeter", crlf);
    }
    quiesce();
    ChGen::source(EvPin<GenPin>{});
    tcb0_hook = tcb0_frequency;
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    Beat::init(DynClock{}, 1000, 0x01, false);
    Beat::pulse_us<0>(300);
    FrequencyMeter<T0>::init(DynClock{}, ChGen{});
    clear_captures(); wait_captures(3, 20);
    const uint16_t t24 = cap_a;
    const uint32_t us24 = FrequencyMeter<T0>::us(cap_a);
    verdict(at24 ? "at 24 MHz: 24000 ticks" : "at the board rate: 1 ms of ticks", near(t24, tpms, 1));
    verdict(at24 ? "switch to 12 MHz" : "switch to half the rate", DynClock::set(SysClock::hz / 2u));
    clear_captures(); wait_captures(3, 40);
    const uint16_t t12 = cap_a;
    const uint32_t us12 = FrequencyMeter<T0>::us(cap_a);
    if constexpr (at24) {
        print(serial, "  24 MHz: ", t24, " ticks = ", us24, " us; 12 MHz: ", t12, " ticks = ", us12, " us", crlf);
    } else {
        print(serial, "  ", mhz, " MHz: ", t24, " ticks = ", us24, " us; ", mhz / 2u, " MHz: ", t12,
              " ticks = ", us12, " us", crlf);
    }
    verdict(at24 ? "ticks follow CLK_PER (12000 +-2)" : "ticks follow CLK_PER (half, +-2)", near(t12, tpms / 2u, 2));
    verdict("microseconds stand still (1000 +-1)", near(us12, 1000, 1));
    verdict(at24 ? "back to 24 MHz" : "back to the board rate", DynClock::set(SysClock::hz));
    clear_captures(); wait_captures(3, 20);
    verdict(at24 ? "restored (24000 +-1)" : "restored (1 ms +-1)", near(cap_a, tpms, 1));
    quiesce();
}

// ---- s TCA split escape, CTRLC, HUNF ---------------------------------------------
// The RESET command must carry CMDEN to work FROM split mode (23.7.6):
// without it a TcaPwm cannot hand the instance back to a normal-mode
// task. Then the split-only HUNF flag/event and the CTRLC parking.
void ts_tca_split() {
    print(serial, "s TCA: split -> normal escape (RESET with CMDEN), HUNF, CTRLC parking", crlf);
    quiesce();
    using PwmC = TcaPwm<0, 'C'>;
    PwmC::init();                                  // split, div16
    PwmC::duty<0>(128);                            // PC0 at 50 %
    ChC::source(EvPin<Pwm8Pin>{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChC{});
    clear_captures(); wait_captures(3, 20);
    print(serial, "  split PWM period: ", cap_a, " ticks (expect 4096)", crlf);
    verdict("split mode runs (256 x div16)", near(cap_a, 4096, 1));
    T0::disable();

    Tca<0>::clear_hunf();
    hold_ms(1);
    verdict("HUNF flag sets in split mode", Tca<0>::hunf_flag());
    ChCarry::source(Tca<0>::HunfEvent{});
    PulseCounter<T1>::init(ChCarry{});
    PulseCounter<T1>::reset();
    hold_ms(20);
    const uint16_t hunfs = PulseCounter<T1>::count();
    // CLK_PER / 16 / 256 for 20 ms: 117.2 at 24 MHz, 97.7 at 20 MHz.
    constexpr int32_t hunf_20ms = static_cast<int32_t>((SysClock::hz / 4096u + 25u) / 50u);
    if constexpr (at24) {
        print(serial, "  HUNF events in 20 ms: ", hunfs, " (expect 117)", crlf);
    } else {
        print(serial, "  HUNF events in 20 ms: ", hunfs, " (expect ", hunf_20ms, ")", crlf);
    }
    verdict(at24 ? "HUNF event rate (117 +-2)" : "HUNF event rate (CLK_PER / 4096 +-2)", near(hunfs, hunf_20ms, 2));
    T1::disable();

    // the escape: a normal-mode task right after split
    using Gen = FrequencyGenerator<0, 'D'>;
    verdict("FrequencyGenerator init after split", Gen::init(clock, 1000));
    ChGen::source(EvPin<GenPin>{});
    FrequencyMeter<T0>::init(clock, ChGen{});
    clear_captures(); wait_captures(3, 20);
    if constexpr (at24) {
        print(serial, "  after the escape: period ", cap_a, " ticks (expect 24000)", crlf);
    } else {
        print(serial, "  after the escape: period ", cap_a, " ticks (expect ", tpms, ")", crlf);
    }
    verdict(at24 ? "normal mode works after split (24000 +-1)" : "normal mode works after split (1 ms +-1)",
            near(cap_a, tpms, 1));
    T0::disable(); tcb0_hook = nullptr;

    // CTRLC: park WO0 while the timer is DISABLED (CMP0EN stays from
    // the generator's init, so the parked value owns the pin)
    Tca<0>::disable();
    Tca<0>::output_value<0>(true);
    delay_us(clock, 2);
    verdict("CTRLC parks WO0 high while disabled", GenPin::read());
    Tca<0>::output_value<0>(false);
    delay_us(clock, 2);
    verdict("CTRLC parks WO0 low", !GenPin::read());
    quiesce();
}

// ---- d centered PWM --------------------------------------------------------------
void td_pwm_centered() {
    print(serial, "d TcaPwmCentered (dual-slope) on PD0: width, buffered duty, OVF at the pulse centre", crlf);
    quiesce();
    using C = TcaPwmCentered<0, 'D', tpms / 2u>;  // 24 MHz / (2 x 12000) = 1 kHz
    ChGen::source(EvPin<GenPin>{});
    tcb0_hook = tcb0_duty;
    DutyMeter<T0>::init(clock, ChGen{});
    verdict("init (event at bottom)", C::init(TcaClock::div1, 0x01, C::EventAt::bottom));
    C::duty<0>(3000);
    clear_captures(); wait_captures(3, 20);
    if constexpr (at24) {
        print(serial, "  duty 3000/12000: period ", cap_a, ", width ", cap_b, crlf);
    } else {
        print(serial, "  duty 3000/", tpms / 2u, ": period ", cap_a, ", width ", cap_b, crlf);
    }
    verdict(at24 ? "period = 2 x steps (24000 +-1)" : "period = 2 x steps (+-1)", near(cap_a, tpms, 1));
    verdict("width = 2 x CMP (6000 +-2)", near(cap_b, 6000, 2));
    C::duty<0>(6000);
    hold_ms(3); clear_captures(); wait_captures(3, 20);
    verdict("buffered duty lands at BOTTOM (width 12000 +-2)", near(cap_b, 12000, 2));
    T0::disable();

    // the claim that names the task: OVF sits at the pulse CENTRE -
    // stamp rising edge -> OVF event; the distance must be CMP (+ the
    // constant event-path offset), at both duties.
    tcb0_hook = tcb0_stamp;
    ChCarry::source(Tca<0>::OvfEvent{});
    C::duty<0>(3000); hold_ms(3);
    const uint16_t e1 = measure_duration();
    C::duty<0>(6000); hold_ms(3);
    const uint16_t e2 = measure_duration();
    print(serial, "  rising -> OVF: ", e1, " (CMP 3000), ", e2, " (CMP 6000)", crlf);
    // The centring proof is the SCALING: the distance grows by exactly
    // delta-CMP; the constant skew is small and negative (measured -3 -
    // the rising edge is stamped through the PIN's input synchronizer,
    // the OVF event is internal).
    verdict("OVF at the pulse centre (distance scales as CMP, small constant skew)",
            static_cast<uint16_t>(e2 - e1) == 3000 && near(e1, 3000, 5));
    quiesce();
}

// ---- l / i CCL extras ------------------------------------------------------------
void ccl_quiesce() {
    Ccl::disable();
    Ccl::sequencer<0>(Sequencer::none);
    Ccl::sequencer<1>(Sequencer::none);
#if defined(BRIO_BOARD_ea48)
    Lut<0>::disable(); Lut<1>::disable(); Lut<2>::disable();
    Lut<3>::disable();                 // the AVR EA: four LUTs, two sequencers
#else
    Ccl::sequencer<2>(Sequencer::none);
    Lut<0>::disable(); Lut<1>::disable(); Lut<2>::disable();
    Lut<3>::disable(); Lut<4>::disable(); Lut<5>::disable();
#endif
}

void tl_ccl_logic() {
    print(serial, "l CCL: LINK chain, DFF and RS sequencers, LUT1 ALT1 on PC6, pin release", crlf);
    quiesce(); ccl_quiesce();

#if defined(BRIO_BOARD_ea48)
    // LINK: LUT2's link input is LUT3's output, read on LUT2's pin PD3
    // (the AVR EA's four LUTs: LUT3's own link, into LUT0, is letter x's)
    Ccl::disable();
    Lut<3>::init({.in0 = LutInput::mask, .truth = 0x01});      // constant 1
    Lut<2>::init({.in0 = LutInput::link, .truth = 0xAA, .output_pin = true});
    Ccl::enable();
    delay_us(clock, 5);
    verdict("LINK passes LUT3 = 1 to PD3", Pin<'D', 3>::read());
    Ccl::disable();
    Lut<3>::init({.in0 = LutInput::mask, .truth = 0x00});      // constant 0
    Lut<2>::init({.in0 = LutInput::link, .truth = 0xAA, .output_pin = true});
    Ccl::enable();
    delay_us(clock, 5);
    verdict("LINK passes LUT3 = 0", !Pin<'D', 3>::read());
#else
    // LINK: LUT4's link input is LUT5's output (no wires, no pins on
    // LUT5 needed - it lives on internal signals even on 48-pin parts)
    Ccl::disable();
    Lut<5>::init({.in0 = LutInput::mask, .truth = 0x01});      // constant 1
    Lut<4>::init({.in0 = LutInput::link, .truth = 0xAA, .output_pin = true});
    Ccl::enable();
    delay_us(clock, 5);
    verdict("LINK passes LUT5 = 1 to PB3", Pin<'B', 3>::read());
    Ccl::disable();
    Lut<5>::init({.in0 = LutInput::mask, .truth = 0x00});      // constant 0
    Lut<4>::init({.in0 = LutInput::link, .truth = 0xAA, .output_pin = true});
    Ccl::enable();
    delay_us(clock, 5);
    verdict("LINK passes LUT5 = 0", !Pin<'B', 3>::read());
#endif

    // DFF on pair 1: D = LUT2 (a 1 kHz square via event), EN = LUT3
    // (constant 1), clocked by CLK_PER - the output follows D
    using Gen = FrequencyGenerator<0, 'D'>;
    Gen::init(clock, 1000);
    ChGen::source(EvPin<GenPin>{});
    Ccl::disable();
    Ccl::sequencer<1>(Sequencer::d_flip_flop);
    Lut<2>::init({.in0 = LutInput::event_a, .truth = 0xAA});
    Lut<2>::event_a_on(ChGen{});
    Lut<3>::init({.in0 = LutInput::mask, .truth = 0x01});      // EN = 1
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});                       // the pair's output
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChCarry{});
    clear_captures(); wait_captures(3, 20);
    if constexpr (at24) {
        print(serial, "  DFF output period: ", cap_a, " ticks (expect 24000)", crlf);
    } else {
        print(serial, "  DFF output period: ", cap_a, " ticks (expect ", tpms, ")", crlf);
    }
    verdict(at24 ? "DFF follows D at 1 kHz (24000 +-2)" : "DFF follows D at 1 kHz (+-2)", near(cap_a, tpms, 2));
    T0::disable(); tcb0_hook = nullptr; Gen::stop();

    // RS latch on pair 1: S = LUT2 (software pulses), R = LUT3;
    // the pair's output shown on LUT2's pin (PD3)
    Ccl::disable();
    Ccl::sequencer<1>(Sequencer::rs_latch);
    Lut<2>::init({.in0 = LutInput::event_a, .truth = 0xAA, .output_pin = true});
    Lut<2>::event_a_on(ChSoft{});
    Lut<3>::init({.in0 = LutInput::event_a, .truth = 0xAA});
    Lut<3>::event_a_on(ChSnap{});
    Ccl::enable();
    delay_us(clock, 5);
    ChSoft::pulse(); delay_us(clock, 5);
    verdict("RS latch sets on S (PD3 = 1)", Pin<'D', 3>::read());
    ChSnap::pulse(); delay_us(clock, 5);
    verdict("RS latch resets on R", !Pin<'D', 3>::read());

    // ALT1 on an instance that has one: LUT1 -> PC6; then the release
    Ccl::disable();
    Ccl::sequencer<1>(Sequencer::none);
    Lut<2>::disable(); Lut<3>::disable();
    Lut<1>::init({.in0 = LutInput::mask, .truth = 0x01, .output_pin = true, .alt_pin = true});
    Ccl::enable();
    delay_us(clock, 5);
    verdict("LUT1 ALT1 drives PC6 high", Pin<'C', 6>::read());
    Ccl::disable();
    Lut<1>::init({.in0 = LutInput::mask, .truth = 0x01});      // no pin any more
    Ccl::enable();
    verdict("re-init releases PC6 (direction back to input)",
            (port_by_letter('C').DIR & (1u << 6)) == 0);
    ccl_quiesce(); quiesce();
}

void ti_ccl_inputs() {
    print(serial, "i CCL: TCA WO and AC as LUT inputs; the filter delays measured", crlf);
    quiesce(); ccl_quiesce();

    // TCA0 WO0 as a LUT input - the INTERNAL signal, any route
    using Gen = FrequencyGenerator<0, 'D'>;
    Gen::init(clock, 1000);
    Ccl::disable();
    Lut<2>::init({.in0 = LutInput::tca0, .truth = 0xAA});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChCarry{});
    clear_captures(); wait_captures(3, 20);
    print(serial, "  WO0 through the LUT: ", cap_a, " ticks", crlf);
    verdict(at24 ? "TCA0 WO0 as LUT input (24000 +-1)" : "TCA0 WO0 as LUT input (1 ms +-1)", near(cap_a, tpms, 1));
    T0::disable(); tcb0_hook = nullptr; Gen::stop();

    // AC0 as a LUT input: DAC crossings counted through the LUT
    D::init({.reference = Ref::v2048});
    D::set_mv(500, ac_ref_mv);
    Threshold<Ac<0>>::init(AcPos::ainp3, 1000, Ref::v2048);
    Ccl::disable();
    Lut<2>::init({.in0 = LutInput::ac, .truth = 0xAA});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    PulseCounter<T1>::init(ChCarry{});
    hold_ms(2);
    PulseCounter<T1>::reset();
    for (uint8_t k = 0; k < 3; ++k) {
        D::set_mv(1500, ac_ref_mv); hold_ms(2);
        D::set_mv(500, ac_ref_mv);  hold_ms(3);
    }
    const uint16_t crossings = PulseCounter<T1>::count();
    print(serial, "  AC0 through the LUT: ", crossings, " rising crossings (expect 3)", crlf);
    verdict("AC as LUT input (3 crossings)", crossings == 3);
    T1::disable(); Ac<0>::disable(); D::set(0);

    // the filter delays, measured with the stamp technique (test n):
    // start = PD0 edge, end = the LUT's output event; bare vs sync vs
    // filter is the LUT's own differential delay
    Beat::init(clock, 100, 0x01, false);
    Beat::pulse_us<0>(2000);
    ChGen::source(EvPin<GenPin>{});
    tcb0_hook = tcb0_stamp;
    uint16_t d[3];
    const LutFilter variants[3] = {LutFilter::none, LutFilter::sync, LutFilter::filter};
    for (uint8_t k = 0; k < 3; ++k) {
        Ccl::disable();
        LutX::init({.in0 = LutInput::event_a, .truth = 0xAA, .filter = variants[k]});
        LutX::event_a_on(ChGen{});
        Ccl::enable();
        ChCarry::source(LutX::OutEvent{});
        d[k] = measure_duration();
    }
    // The pass-through LUT's edge coincides with the start, so the
    // stamp catches the NEXT period (the absolute wraps); the LUT's
    // delay shifts that edge too, so the DIFFERENTIALS are exact.
    print(serial, "  LUT differential delay: sync +", static_cast<uint16_t>(d[1] - d[0]),
          ", filter +", static_cast<uint16_t>(d[2] - d[0]), " CLK_PER", crlf);
    verdict("sync adds 2 LUT clocks", static_cast<uint16_t>(d[1] - d[0]) == 2);
    verdict("filter adds 4 LUT clocks", static_cast<uint16_t>(d[2] - d[0]) == 4);

    // the same filter on OSC32K: a low-power debouncer (2..5 cycles
    // of ~732 CLK_PER each, the domain crossing quantizes)
    Osc32k::run_standby(true);
    Ccl::disable();
    LutX::init({.in0 = LutInput::event_a, .truth = 0xAA,
                  .filter = LutFilter::filter, .clock = LutClock::osc32k});
    LutX::event_a_on(ChGen{});
    Ccl::enable();
    ChCarry::source(LutX::OutEvent{});
    hold_ms(5);                                   // let the slow domain settle
    // ONE fresh measurement: a repeat through the stamp re-arm races
    // the slow domain (a back-to-back second read returned the
    // spurious re-arm stamp) - the protocol, not the silicon; a
    // slow-domain-proof stamp is an open bench refinement.
    const uint16_t d32 = measure_duration();
    Osc32k::run_standby(false);
    // One OSC32K cycle is 732 CLK_PER at 24 MHz, 610 at 20 MHz.
    constexpr uint16_t osc32k_cycle = static_cast<uint16_t>(SysClock::hz / 32768u);
    if constexpr (at24) {
        print(serial, "  filtered on OSC32K: ", d32, " ticks (~", static_cast<uint16_t>((d32 + 366) / 732),
              " cycles of ~732) - finding", crlf);
    } else {
        print(serial, "  filtered on OSC32K: ", d32, " ticks (~",
              static_cast<uint16_t>((d32 + osc32k_cycle / 2u) / osc32k_cycle), " cycles of ~",
              osc32k_cycle, ") - finding", crlf);
    }
    verdict("OSC32K filter delay in 2..5 cycles",
            d32 >= (at24 ? 1400 : 2 * osc32k_cycle - 64) && d32 <= (at24 ? 3800 : 5 * osc32k_cycle + 140));
    ccl_quiesce(); quiesce();
}

// ---- w AC extras -----------------------------------------------------------------
// Pin-vs-pin without wires: the positive is AINP3 = PD6, the DAC's own
// pin; the negative is a GPIO the test drives (AC0: PD3 = AINN0;
// AC1: PD5 = AINN0). Then INVERT, the OUT pins (PA7 and PC6), the OUT
// event consumed by a counter, and the window senses.
void tw_ac_extras() {
    print(serial, "w AC: pin-vs-pin (GPIO negative), AC1, INVERT, OUT pins, window senses", crlf);
    quiesce();
    D::init({.reference = Ref::v2048});
    D::set_mv(1500, ac_ref_mv);
    hold_ms(2);

    // AC0: PD6 (1500 mV) against PD3 driven low / high
    Pin<'D', 3>::output(); Pin<'D', 3>::clear();
    verdict("AC0 pin-vs-pin init", Ac<0>::init({.positive = AcPos::ainp3, .negative = AcNeg::ainn0}));
    hold_ms(1);
    verdict("N low: P > N (state 1)", Ac<0>::state());
    Pin<'D', 3>::set();                            // N = VDD > 1.5 V
    hold_ms(1);
    verdict("N high: P < N (state 0)", !Ac<0>::state());

    // INVERT flips the same comparison
    verdict("AC0 re-init inverted", Ac<0>::init({.positive = AcPos::ainp3, .negative = AcNeg::ainn0, .invert = true}));
    hold_ms(1);
    verdict("inverted: N high reads 1", Ac<0>::state());
    Pin<'D', 3>::clear();
    hold_ms(1);
    verdict("inverted: N low reads 0", !Ac<0>::state());
    Ac<0>::disable(); Pin<'D', 3>::input();

    // AC1 (never exercised before) + its OUT event counted by a TCB
    Pin<'D', 5>::output(); Pin<'D', 5>::set();     // N high: state 0
    verdict("AC1 pin-vs-pin init", Ac<1>::init({.positive = AcPos::ainp3, .negative = AcNeg::ainn0}));
    ChCarry::source(Ac<1>::OutEvent{});
    PulseCounter<T1>::init(ChCarry{});
    hold_ms(1);
    PulseCounter<T1>::reset();
    for (uint8_t k = 0; k < 3; ++k) {
        Pin<'D', 5>::clear(); hold_ms(1);          // P > N: rising OUT
        Pin<'D', 5>::set();   hold_ms(1);
    }
    const uint16_t edges = PulseCounter<T1>::count();
    print(serial, "  AC1 OUT events counted: ", edges, " (expect 3)", crlf);
    verdict("AC1 works and its OUT event is consumed (3 risings)", edges == 3);
    T1::disable(); Ac<1>::disable(); Pin<'D', 5>::input();

    // the OUT pins: AC0 -> PA7 (default; CLKOUT is off here), then
    // AC2 -> PC6 (ALT1; AC1 on the AVR EA, which has no AC2)
    verdict("AC0 out on PA7 init", Ac<0>::init({.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                                                .reference = Ref::v2048, .dacref = ac_dacref_code(1000, ac_ref_mv),
                                                .output_pin = true}));
    D::set_mv(500, ac_ref_mv); hold_ms(2);
    const bool pa7_low = !Pin<'A', 7>::read();
    D::set_mv(1500, ac_ref_mv); hold_ms(2);
    verdict("PA7 follows the comparison", pa7_low && Pin<'A', 7>::read());
    Ac<0>::disable();
    verdict(ac_count == 3 ? "AC2 out on PC6 (ALT1) init" : "AC1 out on PC6 (ALT1) init",
            AcUp::init({.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                                                       .reference = Ref::v2048, .dacref = ac_dacref_code(1000, ac_ref_mv),
                                                       .output_pin = true, .alt_pin = true}));
    D::set_mv(500, ac_ref_mv); hold_ms(2);
    const bool pc6_low = !Pin<'C', 6>::read();
    D::set_mv(1500, ac_ref_mv); hold_ms(2);
    verdict("PC6 (ALT1) follows the comparison", pc6_low && Pin<'C', 6>::read());
    AcUp::disable();

    // the window senses: an interrupt on entering each chosen state
    for (uint8_t k = 0; k < 3; ++k) {
        const AcWindowSense when = k == 0 ? AcWindowSense::below
                                 : k == 1 ? AcWindowSense::inside : AcWindowSense::above;
        const uint16_t target = k == 0 ? 300 : k == 1 ? 1000 : 1800;
        D::set_mv(k == 0 ? 1000 : 300, ac_ref_mv);         // start OUTSIDE the chosen state
        verdict("Window init", Window<Ac<0>, AcUp>::init(AcPos::ainp3, 500, 1500, when, Ref::v2048));
        hold_ms(2);
        ac_irqs = 0;
        D::set_mv(target, ac_ref_mv);
        hold_ms(2);
        print(serial, "  sense ", k == 0 ? "below" : k == 1 ? "inside" : "above",
              ": irqs on entering = ", ac_irqs, crlf);
        verdict("interrupt fires on entering the chosen state", ac_irqs >= 1);
    }
    Ac<0>::disable(); AcUp::disable(); D::set(0);
    // PD3 was AC0's negative: init() switched its digital input buffer
    // off (an analog input). Back to a plain input, or the next reader of
    // PD3 - letter l's RS latch on LUT2's pin - reads 0 whatever the pad
    // does (measured: l failed after w in every run but the first).
    Pin<'D', 3>::configure({});
    Pin<'D', 3>::input();
    quiesce();
}

// ---- k CCL: the inputs, the latch and the clocks no other letter uses -----------------
// TCB2's WO, USART1's TXD and SPI0's SCK as LUT inputs, each passed
// through a LUT whose output event a TCB measures; the D LATCH sequencer
// (transparent while G is high, holding while it is low) read on EVOUT
// PD2; FEEDBACK closing a DFF on itself so it divides its clock by two,
// that clock first the LUT's own INPUT 2 (a 1 kHz square from TCA0),
// then OSC32K and OSC1K - the two slow clocks judged by their ratio.
void tk_ccl_extras() {
    print(serial, "k CCL: TCB, USART and SPI inputs; the D latch; FEEDBACK with an "
                  "input-2, OSC32K and OSC1K clock", crlf);
    quiesce(); ccl_quiesce();

    // 1. TCB2's WO (Pwm8, period 256 ticks) on LUT2's input 2.
    Pwm::init();
    Pwm::duty(64);
    Lut<2>::init({.in2 = LutInput::tcb, .truth = 0xF0});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChCarry{});
    clear_captures(); wait_captures(3, 20);
    const uint16_t tcb_period = cap_a;
    T0::disable(); tcb0_hook = nullptr; T2::disable();
    print(serial, "  TCB2 WO through LUT2's input 2: period ", tcb_period, " ticks (expect 256)",
          crlf);
    verdict("TCB2's WO as a LUT input (256 +-1)", near(tcb_period, 256, 1));
    ccl_quiesce();

    // 2. USART1's TXD (asynchronous, ALT1 pins PC4/PC5) on LUT2's input 1:
    //    four frames of 0x55 (8N1, LSB first: start 0, then 1 0 1 0 1 0 1
    //    0, stop 1) rise five times each - 20 rising edges counted by TCB1.
    Lut<2>::init({.in1 = LutInput::usart_txd, .truth = 0xCC});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    PulseCounter<T1>::init(ChCarry{});
    const uint16_t u1_baud = usart_baud_reg(SysClock::hz, 115200);
    const bool u1 = u1_baud != 0 && Usart<1>::init({.route = UsartRoute::alt1, .baud = u1_baud});
    hold_ms(1);
    PulseCounter<T1>::reset();
    bool sent = u1;
    for (uint8_t i = 0; i < 4; ++i) sent = sent && Usart<1>::send(0x55);
    hold_ms(2);
    const uint16_t txd_rises = PulseCounter<T1>::count();
    Usart<1>::release();
    T1::disable();
    print(serial, "  USART1 TXD through LUT2's input 1: ", txd_rises,
          " rising edges over four 0x55 frames (expect 20)", crlf);
    verdict("USART1's TXD as a LUT input (five rises a 0x55 frame)", sent && txd_rises == 20);
    ccl_quiesce();

    // 3. SPI0's SCK (a pinless host, mode 0) on LUT2's input 2: four
    //    bytes, 32 rising edges.
    Lut<2>::init({.in2 = LutInput::spi0, .truth = 0xF0});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    PulseCounter<T1>::init(ChCarry{});
    const bool spi_up = Spi<0>::init({.route = SpiRoute::none});
    PulseCounter<T1>::reset();
    bool spi_ok = spi_up;
    for (uint8_t i = 0; i < 4; ++i) spi_ok = spi_ok && Spi<0>::transfer(0x00).has_value();
    const uint16_t sck_rises = PulseCounter<T1>::count();
    Spi<0>::release();
    T1::disable();
    print(serial, "  SPI0 SCK through LUT2's input 2: ", sck_rises,
          " rising edges over four bytes (expect 32)", crlf);
    verdict("SPI0's SCK as a LUT input (32)", spi_ok && sck_rises == 32);
    ccl_quiesce();

    // 4. The D latch on pair 0: D = LUT0 (PC1's level), G = LUT1 (PB0's
    //    level), Q on EVOUT PD2.
    using GPin = Pin<'B', 0>;
    using QPin = Pin<'D', 2>;
    DirPin::output(); DirPin::clear();
    GPin::output(); GPin::clear();
    ChC::source(EvPin<DirPin>{});
    ChSoft::source(EvPin<GPin>{});
    Ccl::sequencer<0>(Sequencer::d_latch);
    Lut<0>::init({.in0 = LutInput::event_a, .truth = 0xAA});
    Lut<0>::event_a_on(ChC{});
    Lut<1>::init({.in0 = LutInput::event_a, .truth = 0xAA});
    Lut<1>::event_a_on(ChSoft{});
    Ccl::enable();
    ChCarry::source(Lut<0>::OutEvent{});
    EvOut<QPin>::listen(ChCarry{});
    auto q = []() { delay_us(clock, 5); return QPin::read(); };
    GPin::set();
    DirPin::set();
    const bool t1 = q();
    DirPin::clear();
    const bool t0 = q();
    GPin::clear();                                 // latch Q = 0
    DirPin::set();
    const bool h0 = q();
    GPin::set();                                   // transparent again
    const bool t1b = q();
    GPin::clear();                                 // latch Q = 1
    DirPin::clear();
    const bool h1 = q();
    EvOut<QPin>::unlisten();
    GPin::clear();
    print(serial, "  D latch: G high D 1/0 -> Q ", t1, "/", t0, "; G low D 1 -> Q ", h0,
          "; G high -> Q ", t1b, "; G low D 0 -> Q ", h1, crlf);
    verdict("the D latch is transparent while G is high", t1 && !t0 && t1b);
    verdict("and holds while G is low, both ways", !h0 && h1);
    ccl_quiesce();

    // 5. FEEDBACK: a DFF on pair 0 whose D is NOT its own Q (LUT0's input
    //    0 = feedback, truth = the negation; LUT1, the enable, constant 1)
    //    divides its clock by two. Clocked from LUT0's INPUT 2 - the
    //    event B of TCA0's 1 kHz square on PD0 - it runs at 500 Hz: 48000
    //    ticks a period, +-1 for the capture.
    Gen::init(clock, 1000);
    ChGen::source(EvPin<GenPin>{});
    auto dff_leg = [](LutClock clk) -> uint16_t {
        ccl_quiesce();
        Ccl::sequencer<0>(Sequencer::d_flip_flop);
        // Input 2 is the clock only when the clock is input 2; otherwise
        // it is masked, or its level would enter the truth table.
        Lut<0>::init({.in0 = LutInput::feedback,
                      .in2 = clk == LutClock::input2 ? LutInput::event_b : LutInput::mask,
                      .truth = 0x01, .clock = clk});
        Lut<0>::event_b_on(ChGen{});
        Lut<1>::init({.truth = 0x01});
        Ccl::enable();
        ChCarry::source(Lut<0>::OutEvent{});
        tcb0_hook = tcb0_frequency;
        FrequencyMeter<T0>::init(clock, ChCarry{});
#if defined(BRIO_BOARD_ea48)
        // The AVR64EA48's OSC32K jitters by a few CLK_PER from cycle to
        // cycle (rtc.md): one two-cycle period read up to 132 ticks off
        // the 32:1 ratio once multiplied by 32. The mean of 32
        // consecutive periods (after the first, which may straddle the
        // start) telescopes the jitter to the two ends: +-1 tick of the
        // capture each, divided by 32.
        clear_captures();
        wait_captures(1, 200);
        // The hook and the tally change together: a capture between the
        // two, through the old hook, counts without adding (measured once:
        // a mean of 32 periods over 33).
        cli(); tcb0_hook = tcb0_frequency_mean; captures = 0; mean_sum = 0; sei();
        wait_captures(32, 200);
        cli();
        const uint16_t n = captures;
        const uint32_t sum = mean_sum;
        sei();
        const uint16_t p = n >= 32 ? static_cast<uint16_t>((sum + n / 2) / n) : 0;
#else
        clear_captures();
        wait_captures(3, 200);
        const uint16_t p = captures >= 3 ? static_cast<uint16_t>(cap_a) : 0;
#endif
        T0::disable(); tcb0_hook = nullptr;
        return p;
    };
    const uint16_t p_in2 = dff_leg(LutClock::input2);
    Gen::stop();
    Osc32k::run_standby(true);                     // keep the 32 kHz domain requested
    const uint16_t p_32k = dff_leg(LutClock::osc32k);
    const uint16_t p_1k = dff_leg(LutClock::osc1k);
    Osc32k::run_standby(false);
    if constexpr (at24) {
        print(serial, "  DFF fed back on itself: clocked by input 2 (1 kHz) period ", p_in2,
              " ticks (expect 48000); by OSC32K ", p_32k, ", by OSC1K ", p_1k, " (ratio x32)",
              crlf);
    } else {
        print(serial, "  DFF fed back on itself: clocked by input 2 (1 kHz) period ", p_in2,
              " ticks (expect ", 2u * tpms, "); by OSC32K ", p_32k, ", by OSC1K ", p_1k,
              " (ratio x32)", crlf);
    }
    verdict(at24 ? "FEEDBACK closes the DFF: input 2 as its clock halves 1 kHz (48000 +-1)"
                 : "FEEDBACK closes the DFF: input 2 as its clock halves 1 kHz (2 ms +-1)",
            near(p_in2, 2u * tpms, 1));
    // OSC1K is OSC32K / 32: the two periods measure the same oscillator,
    // so their ratio is 32 to the capture's tick (32 x +-1 on the fast
    // one, +-1 on the slow) plus the oscillator's wander between the two
    // legs (600 ppm at most on this desk: 28 ticks) - 64 ticks.
    verdict("OSC32K and OSC1K clock it too, at exactly a 32:1 ratio (+-64 ticks)",
            p_32k > 1000 && near(p_1k, static_cast<int32_t>(p_32k) * 32, 64));
    ccl_quiesce(); quiesce();
}

// ---- e EVSYS: the generators and positions no other letter routes ------------------
// SPI0's SCK and USART1's XCK as LEVEL generators - SCK counted in
// rising edges by TCB1 (a pinless SPI host), XCK timed by TCB0 as a
// frequency meter (USART1 a synchronous host on its ALT1 pins PC4..PC7,
// XCK on PC6); the software event on a channel a
// level holds HIGH (16.3.2.6: it inverts for one CLK_PER - a low pulse,
// whose end is a rising edge); EVOUT's ALT1 positions PA7, PC7 and PD7
// following PC1's level - PD7 only on a board that leaves it a free pad
// (avr_board.hpp's board_analog_net).
void te_evsys() {
    print(serial, "e EVSYS: SPI SCK and USART XCK generators, the software event on a "
                  "held level, EVOUT ALT1 on PA7/PC7/PD7", crlf);
    quiesce();
    D::disable();                     // PD7 below may share the analog suite's DAC wire

    // 1. SPI0 SCK: four bytes in mode 0 = 32 rising edges.
    ChCarry::source(EvSpiSck<0>{});
    PulseCounter<T1>::init(ChCarry{});
    const bool spi_up = Spi<0>::init({.route = SpiRoute::none});
    PulseCounter<T1>::reset();
    bool spi_ok = spi_up;
    for (uint8_t i = 0; i < 4; ++i) spi_ok = spi_ok && Spi<0>::transfer(0xA5).has_value();
    const uint16_t sck = PulseCounter<T1>::count();
    Spi<0>::release();
    T1::disable();
    print(serial, "  SPI0 SCK event: ", sck, " rising edges over four bytes", crlf);
    verdict("the SPI SCK generator: eight rising edges a byte (32)", spi_ok && sck == 32);

    // 2. USART1 XCK, a synchronous host at 100 kHz on ALT1: BAUD 7680,
    //    of which [15:6] count, 24 MHz / (2 x 120) = 100 kHz exactly. Its
    //    event measured by TCB0 as a frequency meter: 240 CLK_PER ticks a
    //    period, +-1 for the capture's synchronizer - first with no frame
    //    sent at all, then under four frames.
    using Xh = SyncHost<1, UsartRoute::alt1>;
    ChCarry::source(EvUsartXck<1>{});
    const bool xck_up = Xh::init(clock, 100'000u);
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChCarry{});
    clear_captures();
    const bool idle_got = wait_captures(3, 20);
    const uint16_t xck_idle = cap_a;
    bool sent = xck_up;
    clear_captures();
    for (uint8_t i = 0; i < 4; ++i) sent = sent && Usart<1>::send(0x5A);
    const bool busy_got = wait_captures(3, 20);
    const uint16_t xck_busy = cap_a;
    T0::disable();
    tcb0_hook = nullptr;
    Usart<1>::release();
    if constexpr (at24) {
        print(serial, "  USART1 XCK event: period ", xck_idle, " ticks with no frame sent, ",
              xck_busy, " under four 8N1 frames (expect 240)", crlf);
    } else {
        print(serial, "  USART1 XCK event: period ", xck_idle, " ticks with no frame sent, ",
              xck_busy, " under four 8N1 frames (expect ", SysClock::hz / 100'000u, ")", crlf);
    }
    verdict(at24 ? "the USART XCK generator: 100 kHz, 240 +-1 ticks"
                 : "the USART XCK generator: 100 kHz, 10 us of ticks +-1", sent && busy_got &&
            near(xck_busy, SysClock::hz / 100'000u, 1));
    verdict("and a synchronous host's XCK runs with no frame in flight", idle_got &&
            near(xck_idle, SysClock::hz / 100'000u, 1));

    // 3. The software event on a channel PC1 holds HIGH: each pulse is a
    //    one-CLK_PER LOW, ended by a rising edge TCB1 counts. The control:
    //    the same three pulses with PC1 low (high pulses).
    DirPin::output();
    DirPin::set();
    ChC::source(EvPin<DirPin>{});
    PulseCounter<T1>::init(ChC{});
    delay_us(clock, 10);
    PulseCounter<T1>::reset();
    for (uint8_t i = 0; i < 3; ++i) { ChC::pulse(); delay_us(clock, 5); }
    const uint16_t on_high = PulseCounter<T1>::count();
    DirPin::clear();
    delay_us(clock, 10);
    PulseCounter<T1>::reset();
    for (uint8_t i = 0; i < 3; ++i) { ChC::pulse(); delay_us(clock, 5); }
    const uint16_t on_low = PulseCounter<T1>::count();
    T1::disable();
    print(serial, "  three software events: ", on_high, " rising edges with the level "
          "HIGH, ", on_low, " with it low", crlf);
    verdict("on a held-high channel the software event INVERTS: a low pulse each (3)",
            on_high == 3);
    verdict("and on an idle channel a high pulse each (3)", on_low == 3);

    // 4. EVOUT ALT1: PA7, PC7 and PD7 follow PC1's level on channel 3 -
    //    PD7 only where it is a free pad: on a board whose PD7 is on the
    //    analog net (avr_board.hpp's board_analog_net: VREFA's 100 nF and
    //    the DAC's resistor) no digital output drives it, and EVOUTD ALT1
    //    has no other position, so PA7 and PC7 alone are read there.
    if constexpr (board_analog_net) {
        static_assert(board_pa7_free, "letter e reads EVOUTA ALT1 on PA7");
        EvOut<Pin<'A', 7>>::listen(ChC{});
        EvOut<Pin<'C', 7>>::listen(ChC{});
        DirPin::set(); delay_us(clock, 5);
        const bool all_high = Pin<'A', 7>::read() && Pin<'C', 7>::read();
        DirPin::clear(); delay_us(clock, 5);
        const bool all_low = !Pin<'A', 7>::read() && !Pin<'C', 7>::read();
        EvOut<Pin<'A', 7>>::unlisten();
        EvOut<Pin<'C', 7>>::unlisten();
        print(serial, "  EVOUT ALT1 on PA7/PC7 (PD7 on the analog net, not driven): level high "
                      "read ", all_high ? "high" : "NOT high", ", low read ",
              all_low ? "low" : "NOT low", crlf);
        verdict("EVOUTA/C at their ALT1 positions (Px7) follow the channel", all_high && all_low);
    } else {
        EvOut<Pin<'A', 7>>::listen(ChC{});
        EvOut<Pin<'C', 7>>::listen(ChC{});
        EvOut<Pin<'D', 7>>::listen(ChC{});
        DirPin::set(); delay_us(clock, 5);
        const bool all_high = Pin<'A', 7>::read() && Pin<'C', 7>::read() && Pin<'D', 7>::read();
        DirPin::clear(); delay_us(clock, 5);
        const bool all_low = !Pin<'A', 7>::read() && !Pin<'C', 7>::read() && !Pin<'D', 7>::read();
        EvOut<Pin<'A', 7>>::unlisten();
        EvOut<Pin<'C', 7>>::unlisten();
        EvOut<Pin<'D', 7>>::unlisten();
        print(serial, "  EVOUT ALT1 on PA7/PC7/PD7: level high read ", all_high ? "high" : "NOT high",
              ", low read ", all_low ? "low" : "NOT low", crlf);
        verdict("EVOUTA/C/D at their ALT1 positions (Px7) follow the channel", all_high && all_low);
    }
    quiesce();
}

// ---- t TCA: the modes the other tests leave ----------------------------------------
// The dual-slope OVF placements other than BOTTOM, the split mode's
// three vectors, event input B's three restart actions, event input A's
// any-edge and while-high counting, an FRQ output phase-shifted by its
// compare, the UPDATE command over a locked update, and ALUPD.
volatile uint16_t lunf_irqs = 0, hunf_irqs = 0, lcmp0_irqs = 0;
volatile uint16_t stamp2_start = 0, stamp2_end = 0;
volatile uint8_t stamp2_phase = 0;

/// TCB0's hook for the FRQ phase: the first stamp on PD0's rising edge
/// (ChGen), the second on PD2's (ChC).
void tcb0_stamp_wo2() {
    const uint16_t v = T0::capture();
    if (stamp2_phase == 0) { stamp2_start = v; T0::capture_on(ChC{}); stamp2_phase = 1; }
    else { stamp2_end = v; stamp2_phase = 2; T0::capture_on_events(false); }
}

/// PD0 rising -> the next PD2 rising, in CLK_PER ticks (0xFFFF = missed).
uint16_t measure_wo0_to_wo2() {
    for (uint16_t i = 0; i < 30000 && GenPin::read(); ++i) {}
    stamp2_phase = 0;
    tcb0_hook = tcb0_stamp_wo2;
    T0::capture_on(ChGen{});                       // as measure_duration(): the user first
    T0::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0,
              .event_input = true});
    T0::capture_on(ChGen{});
    T0::enable_capt_interrupt(true);
    for (uint8_t i = 0; i < 40 && stamp2_phase != 2; ++i) hold_ms(1);
    T0::disable();
    tcb0_hook = nullptr;
    return stamp2_phase == 2 ? static_cast<uint16_t>(stamp2_end - stamp2_start) : 0xFFFF;
}

/// TCA0's OVF events counted by TCB1 over 20 ms.
uint16_t ovf_events_20ms() {
    ChCarry::source(Tca<0>::OvfEvent{});
    PulseCounter<T1>::init(ChCarry{});
    PulseCounter<T1>::reset();
    hold_ms(20);
    const uint16_t n = PulseCounter<T1>::count();
    T1::disable();
    return n;
}

using LevelPin = DirPin;                       // PC1: the level the test drives
using WO2Pin = Pin<'D', 2>;

void tt_tca_modes() {
    print(serial, "t TCA: dual-slope TOP/BOTH, split vectors, event B restarts, event A "
                  "any-edge/while-high, FRQ phase, UPDATE over LUPD, ALUPD", crlf);
    quiesce();

    // 1. Dual slope, 1 kHz (12000 steps at div1). OVF at BOTTOM sits at
    //    the pulse centre (test d); at TOP it sits PER + CMP after the
    //    rising edge (the edge at CMP counting down, BOTTOM CMP later,
    //    the falling edge CMP after that, TOP PER - CMP on); BOTH gives
    //    two events a period. The counts over 20 ms are crystal time:
    //    20 periods, +-1 for the window's two ends.
    using C = TcaPwmCentered<0, 'D', tpms / 2u>;
    ChGen::source(EvPin<GenPin>{});
    verdict("dual slope, OVF at TOP: init", C::init(TcaClock::div1, 0x01, C::EventAt::top));
    tcb0_hook = tcb0_stamp;                        // test d's stamps: PD0 rising, then OVF
    ChCarry::source(Tca<0>::OvfEvent{});
    C::duty<0>(3000); hold_ms(3);
    const uint16_t top1 = measure_duration();
    C::duty<0>(6000); hold_ms(3);
    const uint16_t top2 = measure_duration();
    const uint16_t n_top = ovf_events_20ms();
    print(serial, "  OVF at TOP: rising -> OVF ", top1, " (CMP 3000), ", top2,
          " (CMP 6000); expect PER + CMP; ", n_top, " OVF events in 20 ms", crlf);
    verdict("OVF at TOP: PER + CMP after the rising edge (the skew of test d, 5)",
            near(top1, tpms / 2u + 3000u, 5) && static_cast<uint16_t>(top2 - top1) == 3000);
    verdict("one OVF a period (20 +-1 in 20 ms)", near(n_top, 20, 1));
    verdict("dual slope, OVF at BOTH: init", C::init(TcaClock::div1, 0x01, C::EventAt::both));
    tcb0_hook = tcb0_stamp;
    ChCarry::source(Tca<0>::OvfEvent{});
    C::duty<0>(3000); hold_ms(3);
    const uint16_t both1 = measure_duration();
    const uint16_t n_both = ovf_events_20ms();
    print(serial, "  OVF at BOTH: rising -> first OVF ", both1, " (CMP 3000); ", n_both,
          " OVF events in 20 ms", crlf);
    verdict("OVF at BOTH: the first after the rising edge is BOTTOM's, CMP after it",
            near(both1, 3000, 5));
    verdict("two OVF a period (40 +-1 in 20 ms)", near(n_both, 40, 1));
    quiesce();

    // 2. Split mode's three vectors on TCA1 (TcaPwm on PORTB, all six
    //    outputs at duty 0 = pins low; WO0 at 128): LUNF, HUNF and LCMP0
    //    each fire once per 8-bit period, 24 MHz / 16 / 256 = 5859 Hz.
    //    The reference is the hardware: TCB1 counts the LUNF EVENTS over
    //    the same window, so the three handlers' own time - which
    //    stretches the cycle-loop hold by some 4 % here - moves both
    //    counts alike; +-1 for the window's two ends, which the counter
    //    read and the interrupt disables do not share.
    using Bar = TcaPwm<1, 'B'>;
    Bar::init();
    Bar::duty<0>(128);
    cli(); lunf_irqs = 0; hunf_irqs = 0; lcmp0_irqs = 0; sei();
    Tca<1>::clear_lunf(); Tca<1>::clear_hunf(); Tca<1>::clear_lcmp<0>();
    ChCarry::source(Tca<1>::OvfEvent{});           // LUNF, in split mode
    PulseCounter<T1>::init(ChCarry{});
    cli();
    PulseCounter<T1>::reset();
    Tca<1>::enable_lunf_interrupt(true);
    Tca<1>::enable_hunf_interrupt(true);
    Tca<1>::enable_lcmp_interrupt<0>(true);
    sei();
    hold_ms(20);
    cli();
    Tca<1>::enable_lunf_interrupt(false);
    Tca<1>::enable_hunf_interrupt(false);
    Tca<1>::enable_lcmp_interrupt<0>(false);
    const uint16_t events = PulseCounter<T1>::count();
    sei();
    T1::disable();
    print(serial, "  split TCA1, 20 ms of hold: LUNF ", lunf_irqs, ", HUNF ", hunf_irqs,
          ", LCMP0 ", lcmp0_irqs, " interrupts; the LUNF event counted ", events,
          " times by TCB1", crlf);
    verdict("the split LUNF vector fires once per low-half period (the event count +-1)",
            near(lunf_irqs, events, 1));
    verdict("the split HUNF vector once per high-half period", near(hunf_irqs, events, 1));
    verdict("the split LCMP0 vector once per low-half period", near(lcmp0_irqs, events, 1));
    verdict("at 5859 Hz: over 100 in the window", events > 100);
    Tca<1>::reset();
    quiesce();

    // 3. Event input B's restarts, normal mode at CLK_PER/1024 (23437 Hz:
    //    2 ms = 46.9 counts). The event is PC1's level through EvPin.
    LevelPin::output();
    LevelPin::clear();
    ChC::source(EvPin<LevelPin>{});
    // Two ms of CLK_PER/1024 is 46.9 counts at 24 MHz, 39.1 at 20: a hold
    // is a cycle loop, never short, so at least the whole counts less one
    // for the prescaler's phase - 45 and 38; the 24 MHz text keeps the
    // round 40 it was written with.
    constexpr uint16_t floor_2ms = at24 ? 40u : static_cast<uint16_t>(SysClock::hz / 512000u - 1u);
    auto restart_leg = [](TcaEventB action) {
        (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1024,
                            .period = 0xFFFF, .event_b = action});
        Tca<0>::event_b_on(ChC{});
        hold_ms(2);
    };
    restart_leg(TcaEventB::restart_posedge);
    const uint16_t pe0 = Tca<0>::count();
    LevelPin::set(); delay_us(clock, 5);
    const uint16_t pe1 = Tca<0>::count();
    hold_ms(2);
    LevelPin::clear(); delay_us(clock, 5);
    const uint16_t pe2 = Tca<0>::count();
    print(serial, "  event B restart on rising: ", pe0, " -> rise -> ", pe1, " ... ", "fall -> ",
          pe2, crlf);
    verdict(at24 ? "restart_posedge: the rise restarts the count (46.9 -> below 2)"
                 : "restart_posedge: the rise restarts the count (2 ms of counts -> below 2)",
            pe0 >= floor_2ms && pe1 < 2);
    verdict("and the fall does not (2 ms more counted)", pe2 >= floor_2ms);
    restart_leg(TcaEventB::restart_anyedge);
    LevelPin::set(); delay_us(clock, 5);
    hold_ms(2);
    const uint16_t ae0 = Tca<0>::count();
    LevelPin::clear(); delay_us(clock, 5);
    const uint16_t ae1 = Tca<0>::count();
    print(serial, "  event B restart on any edge: ", ae0, " -> fall -> ", ae1, crlf);
    verdict("restart_anyedge: the fall restarts it too", ae0 >= floor_2ms && ae1 < 2);
    restart_leg(TcaEventB::restart_while_high);
    LevelPin::set(); delay_us(clock, 5);
    hold_ms(2);
    const uint16_t hl0 = Tca<0>::count();
    LevelPin::clear();
    hold_ms(2);
    const uint16_t hl1 = Tca<0>::count();
    print(serial, "  event B restart while high: ", hl0, " after 2 ms high, ", hl1,
          " after 2 ms low", crlf);
    verdict("restart_while_high: held at 0 while the level is high", hl0 < 2);
    verdict("and counting once it is low", hl1 >= floor_2ms);
    quiesce();

    // 4. Event input A counting: any edge (ten edges = 10, the rising
    //    count of the same edges = 5 as the control) and while high
    //    (CLK_PER/1024 counted only while PC1 is high: 2 ms = 46.9, +1
    //    for the prescaler's phase, -1 for the window's other end). The
    //    hold is a cycle loop: the console is drained first, or its
    //    transmit interrupts stretch it.
    LevelPin::output();
    LevelPin::clear();
    ChC::source(EvPin<LevelPin>{});
    auto count_leg = [](TcaEventA action) -> uint16_t {
        (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1024,
                            .period = 0xFFFF, .event_a = action});
        Tca<0>::event_a_on(ChC{});
        Tca<0>::count(0);
        for (uint8_t i = 0; i < 5; ++i) {
            LevelPin::set(); delay_us(clock, 20);
            LevelPin::clear(); delay_us(clock, 20);
        }
        return Tca<0>::count();
    };
    const uint16_t any = count_leg(TcaEventA::count_anyedge);
    const uint16_t pos = count_leg(TcaEventA::count_posedge);
    while (!Serial::tx_idle()) {
    }
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1024,
                        .period = 0xFFFF, .event_a = TcaEventA::count_while_high});
    Tca<0>::event_a_on(ChC{});
    Tca<0>::count(0);
    hold_ms(2);
    const uint16_t wh_low = Tca<0>::count();
    LevelPin::set();
    hold_ms(2);
    LevelPin::clear();
    const uint16_t wh_high = Tca<0>::count();
    print(serial, "  event A: five pulses counted ", any, " on any edge, ", pos,
          " on rising; while high: ", wh_low, " after 2 ms low, ", wh_high,
          " after 2 ms more high", crlf);
    verdict("count_anyedge counts both edges of five pulses: 10", any == 10);
    verdict("count_posedge, the control, the rising ones: 5", pos == 5);
    verdict("count_while_high: nothing while low", wh_low == 0);
    // 2 ms of CLK_PER/1024 rounded: 47 (46.9) at 24 MHz, 39 (39.1) at 20,
    // +1 for the prescaler's phase, -1 for the window's other end.
    constexpr int32_t round_2ms = static_cast<int32_t>((SysClock::hz + 256000u) / 512000u);
    verdict(at24 ? "and the prescaled clock while high (46.9 in 2 ms, 46..48)"
                 : "and the prescaled clock while high (2 ms of counts +-1)",
            near(wh_high, round_2ms, 1));
    quiesce();

    // 5. FRQ: WO2 toggles at its CMP2 inside WO0's counting period
    //    (CMP0 = 2399, each output 5 kHz). From the init's RESTART both
    //    outputs are low and both rise in the first period, WO2 at CMP2,
    //    WO0 at CMP0; WO2's next rise is two periods later, so WO0's rise
    //    is followed by WO2's after CMP0 + CMP2 + 2 ticks - a distance
    //    that moves exactly with CMP2 (both edges stamped through a pin's
    //    synchronizer, so the skew cancels). Each CMP2 from a fresh init:
    //    an unbuffered CMP2 written mid-run can toggle WO2 twice in one
    //    period and invert it against WO0 (measured).
    auto frq_leg = [](uint16_t cmp2) -> uint16_t {
        (void)Tca<0>::init({.mode = TcaMode::frequency, .clock = TcaClock::div1,
                            .compare0 = 2399, .compare2 = cmp2, .outputs = 0x05, .route = 'D'});
        ChGen::source(EvPin<GenPin>{});
        ChC::source(EvPin<WO2Pin, slot_b>{});      // PD0 holds PORTD's first slot on the AVR EA
        hold_ms(1);
        return measure_wo0_to_wo2();
    };
    const uint16_t ph1 = frq_leg(600);
    const uint16_t ph2 = frq_leg(1200);
    print(serial, "  FRQ, CMP0 2399: WO0 rising -> WO2 rising ", ph1, " (CMP2 600), ", ph2,
          " (CMP2 1200); expect CMP0 + CMP2 + 2", crlf);
    verdict("the WO2 phase is CMP0 + CMP2 + 2 ticks behind WO0's rise",
            ph1 == 3001 && ph2 == 3601);
    quiesce();

    // 6. The UPDATE command over a locked update, then ALUPD. Single
    //    slope at 1 kHz (div1, PER 23999): with LUPD set a buffered CMP0
    //    survives three TOPs untransferred; UPDATE moves it at once.
    (void)Tca<0>::init({.mode = TcaMode::single_slope, .clock = TcaClock::div1, .period = tpms - 1u,
                        .compare0 = 1000});
    Tca<0>::lock_update(true);
    Tca<0>::compare_buffered<0>(5000);
    hold_ms(3);
    const uint16_t locked = Tca<0>::compare<0>();
    Tca<0>::update();
    const uint16_t forced = Tca<0>::compare<0>();
    Tca<0>::lock_update(false);
    print(serial, "  LUPD: CMP0 after three TOPs ", locked, "; after UPDATE ", forced, crlf);
    verdict("LUPD holds a buffered compare across TOPs", locked == 1000);
    verdict("the UPDATE command transfers it at once, lock or not", forced == 5000);
    // ALUPD with CMP0 and CMP2 enabled (WO0 PD0, WO2 PD2): the buffers
    // land only once BOTH are written (23.5.2).
    (void)Tca<0>::init({.mode = TcaMode::single_slope, .clock = TcaClock::div1, .period = tpms - 1u,
                        .compare0 = 1000, .compare2 = 2000, .outputs = 0x05, .route = 'D',
                        .auto_lock_update = true});
    Tca<0>::compare_buffered<0>(3000);
    hold_ms(3);
    const uint16_t half0 = Tca<0>::compare<0>();
    Tca<0>::compare_buffered<2>(4000);
    hold_ms(3);
    const uint16_t both0 = Tca<0>::compare<0>(), both2 = Tca<0>::compare<2>();
    print(serial, "  ALUPD: CMP0 after its buffer alone ", half0, "; after CMP2's too: CMP0 ",
          both0, ", CMP2 ", both2, crlf);
    verdict("ALUPD: one enabled buffer written alone does not land", half0 == 1000);
    verdict("both written, both land at the next TOP", both0 == 3000 && both2 == 4000);
    quiesce();

    // 7. TCA1's three-channel PORTC route: WO0..2 on PC4..PC6, each
    //    parked high and then low through CTRLC with the timer disabled
    //    and read back on its own pad (test s's method).
    (void)Tca<1>::init({.mode = TcaMode::single_slope, .clock = TcaClock::div1,
                        .period = tpms - 1u, .outputs = 0x07, .route = 'C'});
    Tca<1>::disable();
    Tca<1>::output_value<0>(true); Tca<1>::output_value<1>(true); Tca<1>::output_value<2>(true);
    delay_us(clock, 2);
    const bool hi = Pin<'C', 4>::read() && Pin<'C', 5>::read() && Pin<'C', 6>::read();
    Tca<1>::output_value<0>(false); Tca<1>::output_value<1>(false); Tca<1>::output_value<2>(false);
    delay_us(clock, 2);
    const bool lo = !Pin<'C', 4>::read() && !Pin<'C', 5>::read() && !Pin<'C', 6>::read();
    print(serial, "  TCA1 -> PORTC: WO0..2 parked high read ", hi ? "high" : "NOT high",
          " on PC4..PC6, parked low read ", lo ? "low" : "NOT low", crlf);
    verdict("TCA1's PORTC route drives WO0..2 on PC4..PC6", hi && lo);
    Tca<1>::reset();
    Pin<'C', 4>::input(); Pin<'C', 5>::input(); Pin<'C', 6>::input();
    // TCA0 merely disabled keeps CMP2EN, and a disabled TCA with a compare
    // output enabled still overrides its pin with CTRLC's value: PD2 would
    // stay the TCA's (measured: letter k's latch, read on EVOUT PD2, read
    // 1 throughout in every 'a' run). RESET clears CTRLB.
    Tca<0>::reset();
    Pin<'D', 2>::input();
    quiesce();
}

#if defined(BRIO_BOARD_ea48)
// ---- x the AVR EA's own --------------------------------------------------------------
// What the AVR EA's chapters leave open against the DA/DB's (the routes
// its PORTMUX adds, the DA/DB errata with no EA twin, the input menu's
// two contradictions), each settled on the silicon. Pads: TCA1's WO0..2
// on PA4..PA6 and PD4..PD6 (parked, read back), PC0 (Pwm8), PD2 (EVOUT),
// PD6 (the DAC). A parked pad whose input buffer an analog letter turned
// off reads 0 whatever it carries: the pads read here get theirs back.
void tx_ea() {
    print(serial, "x the AVR EA: TCA1's PORTA/PORTD routes, RESTART vs DIR, the PWM8 byte "
                  "write, the CCL's AC1/SPI0/LINK/reconfiguration/TCA1 inputs", crlf);
    quiesce(); ccl_quiesce();
    Tca<0>::reset();
    D::disable();

    // 1. TCA1's three-channel PORTA and PORTD routes (DS40002443A 17.3.7):
    //    WO0..2 on pins 4..6, parked high then low through CTRLC with the
    //    timer disabled (letter t's method on PORTC).
    using A4 = Pin<'A', 4>; using A5 = Pin<'A', 5>; using A6 = Pin<'A', 6>;
    using D4 = Pin<'D', 4>; using D5 = Pin<'D', 5>; using D6 = Pin<'D', 6>;
    A4::configure({}); A5::configure({}); A6::configure({});
    D4::configure({}); D5::configure({}); D6::configure({});
    // PD6 is on the board's analog net (avr_board.hpp): its 96 nF and 10 k
    // take a pad 4.7 us to VIH and 3.3 us to VIL (test_avr_analog letter
    // n) - 30 us is six times that; a bare pad settles in nanoseconds.
    auto park = [](bool high) {
        Tca<1>::output_value<0>(high); Tca<1>::output_value<1>(high); Tca<1>::output_value<2>(high);
        delay_us(clock, board_analog_net ? 30 : 2);
    };
    (void)Tca<1>::init({.mode = TcaMode::single_slope, .clock = TcaClock::div1,
                        .period = tpms - 1u, .outputs = 0x07, .route = 'A'});
    Tca<1>::disable();
    park(true);
    const bool a_hi = A4::read() && A5::read() && A6::read();
    park(false);
    const bool a_lo = !A4::read() && !A5::read() && !A6::read();
    Tca<1>::reset();
    A4::input(); A5::input(); A6::input();
    (void)Tca<1>::init({.mode = TcaMode::single_slope, .clock = TcaClock::div1,
                        .period = tpms - 1u, .outputs = 0x07, .route = 'D'});
    Tca<1>::disable();
    park(true);
    const bool d_hi = D4::read() && D5::read() && D6::read();
    park(false);
    const bool d_lo = !D4::read() && !D5::read() && !D6::read();
    Tca<1>::reset();
    D4::input(); D5::input(); D6::input();
    print(serial, "  TCA1 -> PORTA: PA4..PA6 parked high ", a_hi ? "high" : "NOT high", ", low ",
          a_lo ? "low" : "NOT low", "; -> PORTD: PD4..PD6 high ", d_hi ? "high" : "NOT high",
          ", low ", d_lo ? "low" : "NOT low", crlf);
    verdict("TCA1's PORTA route drives WO0..2 on PA4..PA6", a_hi && a_lo);
    verdict("TCA1's PORTD route drives WO0..2 on PD4..PD6", d_hi && d_lo);

    // 2. RESTART against DIR: the DB's A4/A5 (DS80000915F 2.12.1) and every
    //    DA (DS80000882C 2.11.2) put DIR back to up; the EA's 22.3.3.5 names
    //    only the counter and the outputs, and DS80001048D has no such item.
    //    NORMAL at CLK_PER/1024, counting down, restarted: DIR read back,
    //    then 2 ms counted - up from 0 reads about 39, down wraps to 65535
    //    and reads about 65496.
    (void)Tca<0>::init({.mode = TcaMode::normal, .clock = TcaClock::div1024, .period = 0xFFFF,
                        .count_down = true});
    hold_ms(1);
    const bool down_before = Tca<0>::counting_down();
    Tca<0>::restart();
    const bool down_after = Tca<0>::counting_down();
    hold_ms(2);
    const uint16_t after = Tca<0>::count();
    Tca<0>::reset();
    print(serial, "  RESTART: DIR down before ", down_before ? 1 : 0, ", after ", down_after ? 1 : 0,
          "; 2 ms later CNT = ", after, crlf);
    verdict("RESTART keeps DIR on the AVR EA (no 2.12.1 twin): still down, CNT wrapped below MAX",
            down_before && down_after && after > 0xFF00u);

    // 3. Pwm8's duty as ONE byte, CCMPH alone: the DB's A4/A5 refuse it
    //    (DS80000915F 2.13.1: CCMP is 16-bit only in PWM8, hence the
    //    driver's word write); the EA has no such item. Period 256 ticks,
    //    duty 64 written whole, then 192 into CCMPH alone - after a read of
    //    CNT, which leaves CNT's high byte (0: the count is below 256) in
    //    the instance's TEMP register: a 16-bit write of CCMP would take
    //    CCMPL from TEMP and the period would collapse to one tick, an
    //    8-bit CCMPH write keeps it.
    ChC::source(EvPin<Pwm8Pin>{});
    tcb0_hook = tcb0_duty;
    DutyMeter<T0>::init(clock, ChC{});
    Pwm::init();
    Pwm::duty(64);
    clear_captures(); wait_captures(3, 20);
    const uint16_t w_word = cap_b;
    (void)T2::count();
    T2::regs().CCMPH = 192;
    hold_ms(1);
    clear_captures(); wait_captures(3, 20);
    const uint16_t w_byte = cap_b, p_byte = cap_a;
    T2::disable(); T0::disable(); tcb0_hook = nullptr; ChC::off();
    print(serial, "  PWM8: width ", w_word, " after the word write of 64, ", w_byte,
          " (period ", p_byte, ") after CCMPH = 192 alone", crlf);
    verdict("the one-byte CCMPH write moves the duty and keeps CCMPL (no 2.13.1 twin)",
            near(w_word, 64, 1) && near(w_byte, 192, 1) && p_byte == 256);

    // 4. The AC code on LUT input 2: AC1 by the header and 29.2.2.1,
    //    "reserved" by the register summary 29.5.8. AC1 on PD6 (the DAC)
    //    against 1000 mV, AC0 off; LUT2 passes input 2; three DAC crossings.
    D::init({.reference = Ref::v2048});
    D::set_mv(500, ac_ref_mv);
    Ac<0>::disable();
    // No interrupt: AC1's vector is not bound in this suite (an unbound
    // vector is a jump to 0).
    verdict("AC1 threshold init", Threshold<Ac<1>>::init(AcPos::ainp3, 1000, Ref::v2048, 0,
                                                         AcHysteresis::medium, AcSense::both, false));
    Lut<2>::init({.in2 = LutInput::ac, .truth = 0xF0});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    PulseCounter<T1>::init(ChCarry{});
    hold_ms(2);
    PulseCounter<T1>::reset();
    for (uint8_t k = 0; k < 3; ++k) {
        D::set_mv(1500, ac_ref_mv); hold_ms(2);
        D::set_mv(500, ac_ref_mv);  hold_ms(3);
    }
    const uint16_t ac1_crossings = PulseCounter<T1>::count();
    T1::disable(); Ac<1>::disable(); D::disable();
    Pin<'D', 6>::configure({});
    ccl_quiesce();
    print(serial, "  LUT input 2's AC code: ", ac1_crossings, " rising crossings of AC1 (expect 3)",
          crlf);
    verdict("LUT input 2's AC code is AC1 (lut_input_ac(2) == 1)",
            lut_input_ac(2) == 1 && ac1_crossings == 3);

    // 5. SPI0 on LUT inputs 0 and 1: MOSI by the data sheet (29.2.2.1), MISO
    //    by the header's comments. A pinless host sends four 0x55: MOSI
    //    rises four times a byte (16), SCK eight (32), a MISO no pin drives
    //    not at all.
    uint16_t spi_rises[2] = {0, 0};
    for (uint8_t in = 0; in < 2; ++in) {
        if (in == 0) Lut<2>::init({.in0 = LutInput::spi0, .truth = 0xAA});
        else Lut<2>::init({.in1 = LutInput::spi0, .truth = 0xCC});
        Ccl::enable();
        ChCarry::source(Lut<2>::OutEvent{});
        PulseCounter<T1>::init(ChCarry{});
        const bool up = Spi<0>::init({.route = SpiRoute::none});
        PulseCounter<T1>::reset();
        bool ok = up;
        for (uint8_t i = 0; i < 4; ++i) ok = ok && Spi<0>::transfer(0x55).has_value();
        spi_rises[in] = ok ? PulseCounter<T1>::count() : 0xFFFF;
        Spi<0>::release();
        T1::disable();
        ccl_quiesce();
    }
    print(serial, "  SPI0 on LUT input 0: ", spi_rises[0], " rises, input 1: ", spi_rises[1],
          " over four 0x55 (MOSI 16, SCK 32, MISO 0)", crlf);
    verdict("SPI0 on LUT inputs 0 and 1 is MOSI (16 rises over four 0x55)",
            spi_rises[0] == 16 && spi_rises[1] == 16);

    // 6. LINK into LUT3 (the DA/DB 28/32-pin errata DS80000915F 2.4.2 /
    //    DS80000882C 2.3.1 lose it; no EA item): LUT3 passes LUT0's output,
    //    a constant 1 then 0, read on EVOUT PD2.
    using Q = Pin<'D', 2>;
    ChCarry::source(Lut<3>::OutEvent{});
    EvOut<Q>::listen(ChCarry{});
    auto link_leg = [](uint8_t lut0_truth) {
        Ccl::disable();
        Lut<0>::init({.truth = lut0_truth});
        Lut<3>::init({.in0 = LutInput::link, .truth = 0xAA});
        Ccl::enable();
        delay_us(clock, 5);
        return Q::read();
    };
    const bool link1 = link_leg(0x01);
    const bool link0 = link_leg(0x00);
    EvOut<Q>::unlisten();
    ccl_quiesce();
    print(serial, "  LINK into LUT3: LUT0 = 1 reads ", link1 ? 1 : 0, ", LUT0 = 0 reads ",
          link0 ? 1 : 0, crlf);
    verdict("LINK carries LUT0's output into LUT3, both ways", link1 && !link0);

    // 7. One LUT reconfigured under a RUNNING block (the DB's A4/A5 want
    //    the whole CCL disabled, DS80000915F 2.4.1; no EA item). LUT0
    //    passes TCA0's WO0 (an FRQ square at 1 kHz: 2 toggles a period,
    //    each a CMP0 event); TCB1 counts LUT0's rising edges, TCB2 the
    //    CMP0 events, while LUT2 is re-initialised fifty times with the
    //    block left enabled, alternating 1 and 0, its output read on PD2.
    //    No gap: the CMP0 events are exactly twice LUT0's rises (+-1 for
    //    the two counters' reads), and LUT2 shows each new table.
    using Gen = FrequencyGenerator<0, 'D'>;
    Gen::init(clock, 1000);
    Ccl::disable();
    Lut<0>::init({.in0 = LutInput::tca0, .truth = 0xAA});
    Lut<2>::init({.truth = 0x01});
    Ccl::enable();
    ChCarry::source(Lut<0>::OutEvent{});
    PulseCounter<T1>::init(ChCarry{});
    ChSoft::source(Tca<0>::CmpEvent<0>{});
    PulseCounter<T2>::init(ChSoft{});
    ChSnap::source(Lut<2>::OutEvent{});
    EvOut<Q>::listen(ChSnap{});
    cli();
    PulseCounter<T1>::reset();
    PulseCounter<T2>::reset();
    sei();
    bool tables = true;
    for (uint8_t k = 0; k < 50; ++k) {
        const bool one = (k & 1) == 0;
        Lut<2>::init({.truth = static_cast<uint8_t>(one ? 0x01 : 0x00)});
        delay_us(clock, 5);
        tables = tables && Q::read() == one;
        hold_ms(1);
    }
    cli();
    const uint16_t lut0_rises = PulseCounter<T1>::count();
    const uint16_t cmp0_events = PulseCounter<T2>::count();
    sei();
    EvOut<Q>::unlisten();
    Gen::stop();
    T1::disable(); T2::disable();
    ccl_quiesce();
    print(serial, "  LUT2 re-initialised 50 times under a running block: its tables ",
          tables ? "followed" : "did NOT follow", "; LUT0 rose ", lut0_rises, " times against ",
          cmp0_events, " CMP0 events", crlf);
    verdict("a LUT reconfigures under a running block (each new table at once)", tables);
    verdict("and the other LUTs run undisturbed (CMP0 events = 2 x LUT0's rises +-1)",
            lut0_rises > 40 && near(cmp0_events, 2 * static_cast<int32_t>(lut0_rises), 1));

    // 8. TCA1 into a LUT with no pin: TCA1 in FRQ at 1 kHz, no output
    //    enabled and no route written; LUT2 passes its WO0 (the internal
    //    signal reaches the CCL whatever CMPnEN and the route say).
    (void)Tca<1>::init({.mode = TcaMode::frequency, .clock = TcaClock::div1,
                        .compare0 = static_cast<uint16_t>(tpms / 2u - 1u)});
    Lut<2>::init({.in0 = LutInput::tca1, .truth = 0xAA});
    Ccl::enable();
    ChCarry::source(Lut<2>::OutEvent{});
    tcb0_hook = tcb0_frequency;
    FrequencyMeter<T0>::init(clock, ChCarry{});
    clear_captures(); wait_captures(3, 20);
    const uint16_t tca1_period = cap_a;
    T0::disable(); tcb0_hook = nullptr;
    Tca<1>::reset();
    ccl_quiesce();
    print(serial, "  TCA1 WO0 (no pin) through LUT2: period ", tca1_period, " ticks (expect ", tpms,
          ")", crlf);
    verdict("TCA1's WO0 reaches a LUT with no pin route (1 ms +-1)", near(tca1_period, tpms, 1));
    quiesce();
}
#endif

void register_tests() {
    bench.letter('1', "frequency", t1_frequency);
    bench.letter('2', "PWM16 duty", t2_pwm16);
    bench.letter('3', "heartbeat", t3_heartbeat);
    bench.letter('4', "one-shot", t4_one_shot);
    bench.letter('5', "Pwm8", t5_pwm8);
    bench.letter('6', "pulse counter", t6_pulse_counter);
    bench.letter('7', "32-bit cascade", t7_cascade);
    bench.letter('8', "tick + timeout", t8_tick_timeout);
    bench.letter('9', "event counter", t9_event_counter);
    bench.letter('c', "CCL", tc_ccl);
    bench.letter('m', "comparators", tm_comparators);
    bench.letter('n', "timeout duration", tn_timeout_exact);
    bench.letter('o', "meter variants", to_meter_variants);
    bench.letter('r', "live rebase", tr_rebase_live);
    bench.letter('s', "TCA split escape", ts_tca_split);
    bench.letter('d', "centered PWM", td_pwm_centered);
    bench.letter('l', "CCL logic", tl_ccl_logic);
    bench.letter('i', "CCL inputs", ti_ccl_inputs);
    bench.letter('w', "AC extras", tw_ac_extras);
    bench.letter('t', "TCA modes", tt_tca_modes);
    bench.letter('e', "EVSYS generators and positions", te_evsys);
    bench.letter('k', "CCL extras", tk_ccl_extras);
#if defined(BRIO_BOARD_ea48)
    bench.letter('x', "the AVR EA's own", tx_ea);
#endif
}

void help() {
    print(serial, "test_avr_timer:", crlf);
    bench.menu();
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(TCB0_INT_vect) { if (tcb0_hook) tcb0_hook(); else (void)T0::take_flags(); }
ISR(TCB1_INT_vect) { if (tcb1_hook) tcb1_hook(); else (void)T1::take_flags(); }
ISR(TCA0_OVF_vect) { Beat::beat(); beats = beats + 1; }
ISR(AC0_AC_vect) { (void)Ac<0>::cmp(); ac_irqs = ac_irqs + 1; }
// Test t's split-mode vectors on TCA1 (LUNF shares TCA1_OVF's slot).
ISR(TCA1_LUNF_vect) { Tca<1>::lunf(); lunf_irqs = lunf_irqs + 1; }
ISR(TCA1_HUNF_vect) { Tca<1>::hunf(); hunf_irqs = hunf_irqs + 1; }
ISR(TCA1_LCMP0_vect) { Tca<1>::lcmp<0>(); lcmp0_irqs = lcmp0_irqs + 1; }

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    Ticker::pause();                      // no PIT ISR inflating the crystal-time holds (test 7 resumes it briefly)
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    if constexpr (at24) {
        print(serial, crlf, "test_avr_timer - TCA/TCB/CCL/AC test suite (board ", board,
              ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF",
              " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    } else {
        print(serial, crlf, "test_avr_timer - TCA/TCB/CCL/AC test suite (board ", board,
              ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF", " ", mhz,
              " MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    }
    register_tests();
    help();
    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (!bench.handle(static_cast<char>(c))) { print(serial, "? for help", crlf); }
        bench.prompt();
    }
}
