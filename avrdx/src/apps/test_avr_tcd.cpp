// test_avr_tcd - the TCD (12-bit timer/counter type D) test SUITE for
// the AVR DA/DB target, and the first bench that makes the PLL
// electrically observable: on this silicon the TCD is the PLL's only
// consumer, so a multiplier can only be proven by what the TCD does.
//
// NO WIRES. Every measurement closes inside the chip:
//   - the four waveform outputs sit on the DEFAULT route, PA4..PA7, and
//     each is read back as an EVENT (EvPin) into a TCB meter - the
//     standing wireless technique of this bench;
//   - the TCD's own event generators (CMPBCLR, CMPASET, CMPBSET,
//     PROGEV) carry the cycle out to a TCB when no pin may be claimed;
//   - PD3 and PD4, driven from PORT by software, are the level/edge
//     SOURCES for the TCD's two event inputs (a pin's level is an
//     event: no wire, and a level can be HELD, which a software event
//     cannot);
//   - the TCD is clocked from CLK_PER with both prescalers at DIV1 for
//     every formula test, so one counter tick IS one TCB tick and the
//     chapter's equations are checked in whole ticks.
//
// Reference test of avrdx/tcd.hpp (docs/avrdx/tcd.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800 - PF4/PF5 are never claimed here, and
// test a claims PF0..PF3 (the ALT2 route) only for as long as the
// alternate-route erratum measurement takes.
//
// Commands: ? for the menu, z runs a..k.

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/tcd.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using D = Tcd<0>;

// The DEFAULT route's four outputs.
using WoaPin = Pin<'A', 4>;
using WobPin = Pin<'A', 5>;
using WocPin = Pin<'A', 6>;
using WodPin = Pin<'A', 7>;

// The two software-driven event sources for the TCD's inputs.
using LevelA = Pin<'D', 3>;
using LevelB = Pin<'D', 4>;

// Channels. PORTA/PORTB pin events live on channels 0-1, PORTC/PORTD on
// 2-3, PORTE/PORTF on 4-5 (evsys.hpp); the TCD's own generators are
// legal everywhere.
using ChA = EventChannel<0>;       ///< a PORTA (or PORTB) output pin
using ChB = EventChannel<1>;       ///< a second PORTA output pin
using ChInA = EventChannel<2>;     ///< PD3 -> TCD input A
using ChInB = EventChannel<3>;     ///< PD4 -> TCD input B
using ChTcd = EventChannel<4>;     ///< a TCD event generator, or a PORTF pin

// The instruments.
using CycleTcb = Tcb<0>;
using Cycle = FrequencyMeter<CycleTcb>;
using WidthATcb = Tcb<1>;
using WidthA = PulseWidthMeter<WidthATcb>;
using WidthBTcb = Tcb<2>;
using WidthB = PulseWidthMeter<WidthBTcb>;
using CountTcb = Tcb<3>;
using Edges = PulseCounter<CountTcb>;

using DynClock = DynamicClock<SysClock, Serial, Cycle, WidthA, WidthB, TcdPwm<TcdRoute::def>>;

// ---- verdicts ----------------------------------------------------------------

uint16_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
void verdict(const char* a, const char* b, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", a, b, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

// ---- measurement -------------------------------------------------------------

struct Stat {
    uint16_t min = 0xFFFF;
    uint16_t max = 0;
    uint16_t n = 0;
    uint32_t sum = 0;
    void add(uint16_t v) {
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
        ++n;
    }
    uint16_t mean() const { return n ? static_cast<uint16_t>((sum + n / 2) / n) : 0; }
    bool tight() const { return n != 0 && static_cast<uint16_t>(max - min) <= 1; }
};

void show(const char* label, const Stat& s) {
    print(serial, "    ", label, ": n=", s.n, " min=", s.min, " max=", s.max,
          " mean=", s.mean(), crlf);
}

/// Collect `count` captures from one TCB meter, discarding the first
/// `discard` (the arming edge is not a measurement). Returns a Stat with
/// n < count when the signal stopped.
template <typename T, typename Read>
Stat gather(Read read, uint8_t count = 8, uint8_t discard = 2) {
    Stat s;
    T::clear_capt();
    const uint16_t want = static_cast<uint16_t>(count) + discard;
    for (uint16_t got = 0; got < want; ++got) {
        uint32_t spins = 400'000;
        while (!T::capt_flag()) {
            if (--spins == 0) return s;
        }
        const uint16_t v = read();
        if (got >= discard) s.add(v);
    }
    return s;
}

/// The tcb.hpp meters are built for interrupt use: their init() turns
/// the CAPT interrupt on. This suite POLLS, so every arming closes it
/// again IMMEDIATELY - a capture between two meter inits, on a probe pin
/// that is momentarily an undriven input, is a real event, and the four
/// TCB vectors are bound below only as the second line of defence.
void quiet_meters() {
    CycleTcb::enable_capt_interrupt(false);
    WidthATcb::enable_capt_interrupt(false);
    WidthBTcb::enable_capt_interrupt(false);
    CountTcb::enable_capt_interrupt(false);
}

/// Period meter on `a`, high-time meters on `a` (WOA) and `b` (WOB).
template <uint8_t ca, uint8_t cb>
void arm_meters(EventChannel<ca> a, EventChannel<cb> b, bool low = false) {
    Cycle::init(clock, a);
    CycleTcb::enable_capt_interrupt(false);
    WidthA::init(clock, a, TcbClock::div1, low);
    WidthATcb::enable_capt_interrupt(false);
    WidthB::init(clock, b, TcbClock::div1, low);
    quiet_meters();
}

struct Wave {
    Stat cycle;
    Stat high_a;
    Stat high_b;
};

Wave measure(uint8_t n = 8) {
    Wave w;
    w.cycle = gather<CycleTcb>([] { return Cycle::period_ticks(); }, n);
    w.high_a = gather<WidthATcb>([] { return WidthA::width_ticks(); }, n);
    w.high_b = gather<WidthBTcb>([] { return WidthB::width_ticks(); }, n);
    return w;
}

void show_wave(const Wave& w) {
    show("cycle", w.cycle);
    show("WOA high", w.high_a);
    show("WOB high", w.high_b);
}

/// The TCD's DEFAULT-route probes: WOA on channel 0, WOB on channel 1.
void probe_ab() {
    ChA::source(EvPin<WoaPin>{});
    ChB::source(EvPin<WobPin>{});
}
/// WOC and WOD instead (test j).
void probe_cd() {
    ChA::source(EvPin<WocPin>{});
    ChB::source(EvPin<WodPin>{});
}

/// The level sources: PD3/PD4 driven low, their level on ChInA/ChInB,
/// the TCD's two inputs listening.
void arm_level_sources() {
    LevelA::clear();
    LevelB::clear();
    LevelA::output();
    LevelB::output();
    ChInA::source(EvPin<LevelA>{});
    ChInB::source(EvPin<LevelB>{});
    D::input_a_on(ChInA{});
    D::input_b_on(ChInB{});
}

/// A base configuration: one ramp on CLK_PER with both prescalers at
/// DIV1, so one counter tick is one CLK_PER tick and the chapter's
/// formulas are checked in whole ticks.
TcdConfig base_one_ramp(uint16_t a_set, uint16_t a_clr, uint16_t b_set, uint16_t b_clr) {
    TcdConfig c{};
    c.route = TcdRoute::def;
    c.clock = TcdClock::clkper;
    c.waveform = TcdWaveform::one_ramp;
    c.compare_a_set = a_set;
    c.compare_a_clear = a_clr;
    c.compare_b_set = b_set;
    c.compare_b_clear = b_clr;
    c.enable_woa = true;
    c.enable_wob = true;
    return c;
}

void quiesce() {
    (void)D::release();
    LevelA::clear();
    LevelB::clear();
    D::input_a_off();
    D::input_b_off();
}

// ---- a: routes, the sync disciplines, the alternate-route erratum ------------

void ta_routes() {
    print(serial, "a routes, the three synchronization disciplines, and errata 2.14.2 "
                  "on an alternate route", crlf);
    quiesce();

    // --- the route table this package offers
    print(serial, "  package = ", tcd_package_pins, " pins; route table:", crlf);
    verdict("DEFAULT exists and bonds WOA..WOD (PA4..PA7)",
            tcd_route_exists(TcdRoute::def) && tcd_pin(TcdRoute::def, TcdOutput::woa).port == 'A' &&
            tcd_pin(TcdRoute::def, TcdOutput::woa).pin == 4 &&
            tcd_pin(TcdRoute::def, TcdOutput::wod).bonded);
    verdict("ALT1 exists here (PORTB) and bonds WOA/WOB only",
            tcd_route_exists(TcdRoute::alt1) &&
            tcd_pin(TcdRoute::alt1, TcdOutput::woa).port == 'B' &&
            tcd_pin(TcdRoute::alt1, TcdOutput::woa).pin == 4 &&
            tcd_pin(TcdRoute::alt1, TcdOutput::wob).bonded &&
            !tcd_pin(TcdRoute::alt1, TcdOutput::woc).bonded &&
            !tcd_pin(TcdRoute::alt1, TcdOutput::wod).bonded);
    verdict("ALT2 exists (PORTF) and bonds all four here",
            tcd_route_exists(TcdRoute::alt2) &&
            tcd_pin(TcdRoute::alt2, TcdOutput::woa).port == 'F' &&
            tcd_pin(TcdRoute::alt2, TcdOutput::woa).pin == 0 &&
            tcd_pin(TcdRoute::alt2, TcdOutput::wod).bonded);
    verdict("ALT3 (PORTG) does not exist on 48 pins", !tcd_route_exists(TcdRoute::alt3));
    verdict("a WOC on ALT1 is refused at run time",
            !D::init({.route = TcdRoute::alt1, .compare_b_clear = 999,
                      .enable_woa = true, .enable_woc = true}));

    // --- the PORTMUX code and the pin claim/teardown, per route
    {
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.enable_woc = true;
        c.enable_wod = true;
        verdict("DEFAULT init", D::init(c));
        verdict("PORTMUX.TCDROUTEA = DEFAULT (0)",
                (PORTMUX.TCDROUTEA & PORTMUX_TCD0_gm) ==
                    static_cast<uint8_t>(PORTMUX_TCD0_DEFAULT_gv));
        verdict("PA4..PA7 claimed as outputs",
                WoaPin::is_output() && WobPin::is_output() && WocPin::is_output() &&
                WodPin::is_output());
        verdict("FAULTCTRL carries the four enables",
                (D::fault_control() & (TCD_CMPAEN_bm | TCD_CMPBEN_bm | TCD_CMPCEN_bm |
                                       TCD_CMPDEN_bm)) ==
                    (TCD_CMPAEN_bm | TCD_CMPBEN_bm | TCD_CMPCEN_bm | TCD_CMPDEN_bm));
        verdict("release() brings the TCD down", D::release());
        verdict("PA4..PA7 back to inputs and PORTMUX back to DEFAULT",
                !WoaPin::is_output() && !WobPin::is_output() && !WocPin::is_output() &&
                !WodPin::is_output() && (PORTMUX.TCDROUTEA & PORTMUX_TCD0_gm) == 0);
    }
    {
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.route = TcdRoute::alt1;
        verdict("ALT1 init (WOA/WOB on PB4/PB5)", D::init(c));
        verdict("PORTMUX.TCDROUTEA = ALT1 (1)",
                (PORTMUX.TCDROUTEA & PORTMUX_TCD0_gm) ==
                    static_cast<uint8_t>(PORTMUX_TCD0_ALT1_gv));
        verdict("PB4/PB5 claimed", Pin<'B', 4>::is_output() && Pin<'B', 5>::is_output());
        (void)D::release();
        verdict("PB4/PB5 released", !Pin<'B', 4>::is_output() && !Pin<'B', 5>::is_output());
    }

    // --- discipline 1: ENABLE only under STATUS.ENRDY
    {
        verdict("init for the discipline round", D::init(base_one_ramp(100, 400, 700, 1199)));
        D::regs().CTRLA &= static_cast<uint8_t>(~TCD_ENABLE_bm);   // raw: no wait
        const bool closed = !D::enable_ready();
        uint16_t spins = 0;
        while (!D::enable_ready() && spins < 0xFFFF) ++spins;
        print(serial, "    ENRDY closed by the ENABLE write: ", closed, ", reopened after ",
              spins, " spins", crlf);
        verdict("writing ENABLE closes ENRDY", closed);
        verdict("ENRDY reopens by itself", spins < 0xFFFF);
        verdict("the TCD really went down", !D::enabled());
        verdict("enable() under the discipline", D::enable());
    }

    // --- discipline 2: the CTRLE strobes only under STATUS.CMDRDY
    {
        verdict("CMDRDY is open before a command", D::command_ready());
        D::regs().CTRLE = TCD_SYNC_bm;                              // raw: no wait
        const bool closed = !D::command_ready();
        uint16_t spins = 0;
        while (!D::command_ready() && spins < 0xFFFF) ++spins;
        print(serial, "    CMDRDY closed by the SYNC strobe: ", closed, ", reopened after ",
              spins, " spins", crlf);
        verdict("a strobe closes CMDRDY", closed);
        verdict("CMDRDY reopens by itself", spins < 0xFFFF);
        verdict("CTRLE is a strobe register (self-clearing)", D::regs().CTRLE == 0);
    }

    // --- discipline 3: the static registers refuse a write while enabled
    {
        const uint8_t ctrlb = D::regs().CTRLB;
        verdict("still enabled", D::enabled());
        verdict("waveform() refused while enabled", !D::waveform(TcdWaveform::four_ramp));
        verdict("CTRLB unchanged by the refusal", D::regs().CTRLB == ctrlb);
        verdict("clock() refused while enabled",
                !D::clock(TcdClock::oschf, TcdSyncPrescaler::div8, TcdCountPrescaler::div32));
        verdict("input_mode_a() refused while enabled", !D::input_mode_a(TcdInputMode::freq));
        verdict("output_control() refused while enabled",
                !D::output_control(true, false, false, TcdWaveformSelect::pwm_a,
                                   TcdWaveformSelect::pwm_a));
        verdict("output_values() refused while enabled", !D::output_values(0xF, 0xF));
        verdict("delay() refused while enabled",
                !D::delay(TcdDelaySelect::input_blanking, TcdDelayTrigger::cmpaset,
                          TcdDelayPrescaler::div1, 10));
        verdict("disable()", D::disable());
        verdict("waveform() accepted while disabled", D::waveform(TcdWaveform::four_ramp));
        verdict("CTRLB took the new mode",
                D::waveform() == TcdWaveform::four_ramp);
    }
    quiesce();

    // --- errata 2.14.2: on an alternate route CMPAEN gates every output
    //
    // Control first: on the DEFAULT route a WOB-only configuration DOES
    // drive its pin. Then the same thing on ALT2 (PF0..PF3), where the
    // erratum says nothing moves until CMPAEN is set too.
    {
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.enable_woa = false;
        c.enable_wob = true;
        verdict("DEFAULT, WOB only: init", D::init(c));
        ChA::source(EvPin<WobPin>{});
        Edges::init(ChA{});
        quiet_meters();
        Edges::reset();
        delay_us(clock, 20'000);
        const uint16_t def_edges = Edges::count();
        print(serial, "    DEFAULT route, CMPBEN only: ", def_edges,
              " WOB edges in 20 ms (cycle 20 kHz -> ~400 expected)", crlf);
        verdict("the DEFAULT route drives WOB without CMPAEN", def_edges > 100);
        quiesce();

        // ALT2: PF0 = WOA, PF1 = WOB. PORTF pin events live on channels
        // 4-5. PF1 gets a pull-up so an UNDRIVEN pad reads a steady high
        // (zero edges) instead of floating.
        Pin<'F', 1>::pullup(true);
        c.route = TcdRoute::alt2;
        verdict("ALT2, WOB only: init", D::init(c));
        ChTcd::source(EvPin<Pin<'F', 1>>{});
        Edges::init(ChTcd{});
        quiet_meters();
        Edges::reset();
        delay_us(clock, 20'000);
        const uint16_t alt_b_only = Edges::count();
        (void)D::release();

        c.enable_woa = true;                       // CMPAEN as well
        verdict("ALT2, CMPAEN + CMPBEN: init", D::init(c));
        Edges::reset();
        delay_us(clock, 20'000);
        const uint16_t alt_both = Edges::count();
        (void)D::release();
        Pin<'F', 1>::pullup(false);

        print(serial, "    ALT2 route: CMPBEN alone -> ", alt_b_only,
              " WOB edges; CMPAEN + CMPBEN -> ", alt_both, " edges (20 ms)", crlf);
        verdict("errata 2.14.2: on ALT2 a WOB-only configuration drives NOTHING",
                alt_b_only == 0);
        verdict("errata 2.14.2: adding CMPAEN makes WOB drive again", alt_both > 100);
    }
    quiesce();
}

// ---- b: one ramp -------------------------------------------------------------

void tb_one_ramp() {
    print(serial, "b one ramp: the cycle and the two on-times in whole CLK_PER ticks "
                  "(TCD on CLK_PER, both prescalers DIV1)", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    // The chapter's own ordering: CMPASET < CMPACLR < CMPBSET < CMPBCLR.
    {
        verdict("init", D::init(base_one_ramp(100, 400, 700, 1199)));
        const Wave w = measure();
        show_wave(w);
        print(serial, "    expected: cycle = CMPBCLR+1 = 1200, WOA high = CMPACLR-CMPASET = "
                      "300, WOB high = CMPBCLR-CMPBSET = 499", crlf);
        verdict("cycle = CMPBCLR + 1", near(w.cycle.mean(), 1200, 1));
        verdict("the cycle is stable to a tick", w.cycle.tight());
        verdict("WOA on-time = CMPACLR - CMPASET", near(w.high_a.mean(), 300, 1));
        verdict("WOB on-time = CMPBCLR - CMPBSET", near(w.high_b.mean(), 499, 1));
    }

    // Figure 25-4: CMPBSET < CMPASET makes the two on-times OVERLAP.
    {
        verdict("init (CMPBSET < CMPASET)", D::init(base_one_ramp(100, 400, 50, 1199)));
        const Wave w = measure();
        show_wave(w);
        verdict("cycle unchanged", near(w.cycle.mean(), 1200, 1));
        verdict("WOA on-time still CMPACLR - CMPASET", near(w.high_a.mean(), 300, 1));
        verdict("WOB on-time now spans CMPBSET..CMPBCLR = 1149",
                near(w.high_b.mean(), 1149, 1));
    }

    // "A match with CMPBCLR always clears every output; a compare bigger
    // than CMPBCLR never happens."
    {
        verdict("init (CMPACLR > CMPBCLR)", D::init(base_one_ramp(100, 2000, 700, 1199)));
        const Wave w = measure();
        show_wave(w);
        verdict("cycle unchanged", near(w.cycle.mean(), 1200, 1));
        verdict("WOA runs to the end of the cycle (CMPBCLR - CMPASET = 1099)",
                near(w.high_a.mean(), 1099, 1));
    }

    // "If CMPACLR is smaller than CMPASET the clear has no effect."
    {
        verdict("init (CMPACLR < CMPASET)", D::init(base_one_ramp(400, 100, 700, 1199)));
        const Wave w = measure();
        show_wave(w);
        verdict("cycle unchanged", near(w.cycle.mean(), 1200, 1));
        verdict("WOA runs to the end of the cycle (CMPBCLR - CMPASET = 799)",
                near(w.high_a.mean(), 799, 1));
    }
    quiesce();
}

// ---- c: two ramp and four ramp -----------------------------------------------

void tc_ramps() {
    print(serial, "c two ramp and four ramp: the cycle formulas, and the dead-times "
                  "isolated with CMPOVR + CTRLD", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    // Two ramp: cycle = (CMPACLR + 1) + (CMPBCLR + 1).
    {
        TcdConfig c = base_one_ramp(100, 499, 200, 899);
        c.waveform = TcdWaveform::two_ramp;
        verdict("two ramp init", D::init(c));
        const Wave w = measure();
        show_wave(w);
        print(serial, "    expected: cycle = 500 + 900 = 1400, WOA high = 399, "
                      "WOB high = 699", crlf);
        verdict("two ramp cycle = CMPACLR+1 + CMPBCLR+1", near(w.cycle.mean(), 1400, 1));
        verdict("two ramp WOA on-time", near(w.high_a.mean(), 399, 1));
        verdict("two ramp WOB on-time", near(w.high_b.mean(), 699, 1));

        // Table 25-12: with CMPOVR the four states drive the two outputs
        // directly - CMPAVAL = 0b0001 puts WOA high in DEAD-TIME A only,
        // CMPBVAL = 0b0100 puts WOB high in DEAD-TIME B only. That makes
        // the two dead-times DIRECTLY measurable on a pin.
        c.compare_override = true;
        c.compare_a_value = 0b0001;
        c.compare_b_value = 0b0100;
        verdict("two ramp + CMPOVR init", D::init(c));
        const Wave d = measure();
        show_wave(d);
        print(serial, "    expected dead-times: DTA = CMPASET+1 = 101, "
                      "DTB = CMPBSET+1 = 201", crlf);
        verdict("dead-time A isolated on WOA", near(d.high_a.mean(), 101, 2));
        verdict("dead-time B isolated on WOB", near(d.high_b.mean(), 201, 2));
    }

    // Four ramp: cycle = the four compares plus four.
    {
        TcdConfig c = base_one_ramp(99, 199, 299, 399);
        c.waveform = TcdWaveform::four_ramp;
        verdict("four ramp init", D::init(c));
        const Wave w = measure();
        show_wave(w);
        print(serial, "    expected: cycle = 100+200+300+400 = 1000, "
                      "WOA high = CMPACLR+1 = 200, WOB high = CMPBCLR+1 = 400", crlf);
        verdict("four ramp cycle = the four ramps", near(w.cycle.mean(), 1000, 2));
        verdict("four ramp WOA on-time", near(w.high_a.mean(), 200, 2));
        verdict("four ramp WOB on-time", near(w.high_b.mean(), 400, 2));

        c.compare_override = true;
        c.compare_a_value = 0b0001;
        c.compare_b_value = 0b0100;
        verdict("four ramp + CMPOVR init", D::init(c));
        const Wave d = measure();
        show_wave(d);
        print(serial, "    expected dead-times: DTA = CMPASET+1 = 100, "
                      "DTB = CMPBSET+1 = 300", crlf);
        verdict("dead-time A isolated on WOA", near(d.high_a.mean(), 100, 2));
        verdict("dead-time B isolated on WOB", near(d.high_b.mean(), 300, 2));
    }
    quiesce();

    // The TASK over the resource: TcdPwm, the complementary pair with a
    // dead time on both edges. The two on-times plus the two dead times
    // must add up to the cycle exactly at every duty - that IS the
    // dead-time guarantee, measured.
    {
        using Pwm = TcdPwm<TcdRoute::def>;
        verdict("TcdPwm init (1200 ticks of CLK_PER, 60 ticks of dead time)",
                Pwm::init(clock, {.clock = TcdClock::clkper, .period_ticks = 1200,
                                  .dead_time_ticks = 60}));
        print(serial, "    period ", Pwm::period_ticks(), " ticks, dead ",
              Pwm::dead_ticks(), ", max duty ", Pwm::max(), ", counter ",
              Pwm::counter_hz(), " Hz, cycle ", Pwm::cycle_hz(), " Hz", crlf);
        verdict("max = period - 1 - the two dead times", Pwm::max() == 1080);
        verdict("cycle_hz = CLK_PER / period", Pwm::cycle_hz() == 20'000u);
        for (uint16_t duty : {static_cast<uint16_t>(540), static_cast<uint16_t>(270),
                              static_cast<uint16_t>(900)}) {
            verdict("duty()", Pwm::duty(duty));
            verdict("sync()", Pwm::sync());
            delay_us(clock, 500);
            const Wave w = measure();
            const int32_t rest = 1200 - static_cast<int32_t>(duty) - 2 * 60;
            print(serial, "    duty = ", duty, " ticks: cycle ", w.cycle.mean(),
                  ", WOA ", w.high_a.mean(), ", WOB ", w.high_b.mean(),
                  " (expected WOB = period - duty - 2 x dead = ", rest, ")", crlf);
            verdict("the cycle is untouched by a duty change",
                    near(w.cycle.mean(), 1200, 1));
            verdict("WOA carries the duty", near(w.high_a.mean(), duty, 1));
            verdict("WOB carries the rest, less the two dead times",
                    near(w.high_b.mean(), rest, 1));
        }
        verdict("a duty above max is refused", !Pwm::duty(1081));
        verdict("TcdPwm on the PLL's usual input rate is expressible too",
                Pwm::init(clock, {.clock = TcdClock::oschf, .source_hz = 24'000'000u,
                                  .hz = 20'000u, .dead_time_ticks = 60}));
        print(serial, "    on OSCHF 24 MHz asked for 20 kHz: period ", Pwm::period_ticks(),
              " ticks -> ", Pwm::cycle_hz(), " Hz", crlf);
        verdict("the period was derived from the source rate", Pwm::period_ticks() == 1200);
        verdict("release()", Pwm::release());
    }
    quiesce();
}

// ---- d: dual slope -----------------------------------------------------------

void td_dual_slope() {
    print(serial, "d dual slope: the period formula measured in whole ticks, the two "
                  "on-times, CMPACLR ignored, FIFTY", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    // TWO geometries, because the chapter's period formula turns out to
    // be off by one and one data point cannot say so.
    struct Geometry { uint16_t a_set, b_set, b_clr; };
    static constexpr Geometry geoms[] = {{200, 400, 599}, {150, 300, 999}};
    Wave last{};
    for (const Geometry& g : geoms) {
        TcdConfig c{};
        c.route = TcdRoute::def;
        c.clock = TcdClock::clkper;
        c.waveform = TcdWaveform::dual_slope;
        c.compare_a_set = g.a_set;
        c.compare_a_clear = 0;
        c.compare_b_set = g.b_set;
        c.compare_b_clear = g.b_clr;
        c.enable_woa = true;
        c.enable_wob = true;
        verdict("dual slope init", D::init(c));
        const Wave w = measure();
        last = w;
        show_wave(w);
        const uint32_t period = tcd_cycle_ticks(TcdWaveform::dual_slope, g.a_set, 0, g.b_set,
                                                g.b_clr);
        const int32_t woa = 2 * static_cast<int32_t>(g.a_set);
        const int32_t wob = 2 * (static_cast<int32_t>(g.b_clr) - g.b_set + 1);
        print(serial, "    CMPASET = ", g.a_set, ", CMPBSET = ", g.b_set, ", CMPBCLR = ",
              g.b_clr, ": the chapter prints T = 2 x CMPBCLR + 1 = ",
              2u * g.b_clr + 1u, ", the silicon runs ", period,
              " = 2 x (CMPBCLR + 1); WOA = 2 x CMPASET = ", woa,
              ", WOB = 2 x (CMPBCLR - CMPBSET + 1) = ", wob, crlf);
        verdict("the dual-slope period is 2 x (CMPBCLR + 1), not the chapter's +1",
                near(w.cycle.mean(), static_cast<int32_t>(period), 1));
        verdict("the period is stable to a tick", w.cycle.tight());
        verdict("WOA spans the bottom: 2 x CMPASET", near(w.high_a.mean(), woa, 1));
        verdict("WOB spans the top: 2 x (CMPBCLR - CMPBSET + 1)",
                near(w.high_b.mean(), wob, 1));
    }
    const Wave w = last;

    // CMPACLR is not used in Dual Slope mode: writing it changes nothing.
    {
        verdict("write CMPACLR = 4095 under a running dual slope", D::compare_a_clear(4095));
        verdict("SYNC the double buffers", D::sync());
        delay_us(clock, 2000);
        const Wave v = measure();
        show_wave(v);
        verdict("the cycle did not move", near(v.cycle.mean(), w.cycle.mean(), 1));
        verdict("WOA did not move", near(v.high_a.mean(), w.high_a.mean(), 2));
        verdict("WOB did not move", near(v.high_b.mean(), w.high_b.mean(), 2));
        verdict("CMPACLR reads back what was written", D::compare_a_clear() == 4095);
    }
    quiesce();

    // FIFTY: a write to one SET lands in both, and so does a write to
    // one CLR (25.5.3).
    {
        TcdConfig f = base_one_ramp(100, 400, 700, 1199);
        f.fifty_percent = true;
        verdict("one ramp + FIFTY init", D::init(f));
        verdict("write CMPASET = 300", D::compare_a_set(300));
        verdict("CMPBSET mirrored", D::compare_b_set() == 300);
        verdict("write CMPBSET = 250", D::compare_b_set(250));
        verdict("CMPASET mirrored", D::compare_a_set() == 250);
        verdict("write CMPACLR = 900", D::compare_a_clear(900));
        verdict("CMPBCLR mirrored", D::compare_b_clear() == 900);
    }
    quiesce();
}

// ---- e: clocks and prescalers ------------------------------------------------

struct DivCase {
    TcdSyncPrescaler s;
    TcdCountPrescaler c;
    uint16_t divisor;
    const char* name;
};

void te_clocks() {
    print(serial, "e CLKSEL x SYNCPRES x CNTPRES, and what a CLK_PER rebase does (and "
                  "does not do) to a TCD", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    static constexpr DivCase cases[] = {
        {TcdSyncPrescaler::div1, TcdCountPrescaler::div1, 1, "sync 1 x cnt 1"},
        {TcdSyncPrescaler::div2, TcdCountPrescaler::div1, 2, "sync 2 x cnt 1"},
        {TcdSyncPrescaler::div4, TcdCountPrescaler::div1, 4, "sync 4 x cnt 1"},
        {TcdSyncPrescaler::div8, TcdCountPrescaler::div1, 8, "sync 8 x cnt 1"},
        {TcdSyncPrescaler::div1, TcdCountPrescaler::div4, 4, "sync 1 x cnt 4"},
        {TcdSyncPrescaler::div2, TcdCountPrescaler::div4, 8, "sync 2 x cnt 4"},
        {TcdSyncPrescaler::div1, TcdCountPrescaler::div32, 32, "sync 1 x cnt 32"},
        {TcdSyncPrescaler::div4, TcdCountPrescaler::div32, 128, "sync 4 x cnt 32"},
    };
    constexpr uint16_t period = 300;                 // CMPBCLR + 1

    for (const DivCase& d : cases) {
        TcdConfig c = base_one_ramp(50, 150, 200, period - 1);
        c.sync_prescaler = d.s;
        c.count_prescaler = d.c;
        if (!D::init(c)) {
            verdict("init ", d.name, false);
            continue;
        }
        const Stat s = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 6);
        const uint32_t want = static_cast<uint32_t>(period) * d.divisor;
        print(serial, "    ", d.name, ": cycle = ", s.mean(), " CLK_PER ticks, expected ",
              want, crlf);
        verdict("total division = SYNCPRES x CNTPRES, ", d.name,
                near(s.mean(), static_cast<int32_t>(want), 2));
    }
    quiesce();

    // A TCD on OSCHF is IMMUNE to a CLK_PER rebase; a TCD on CLK_PER
    // follows it. Both sides measured, in microseconds (the meters are
    // ClockUsers, so their arithmetic follows the main clock).
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    Cycle::init(DynClock{}, ChA{});
    WidthA::init(DynClock{}, ChA{});
    WidthB::init(DynClock{}, ChB{});
    quiet_meters();

    for (uint8_t leg = 0; leg < 2; ++leg) {
        const bool on_oschf = leg == 0;
        TcdConfig c = base_one_ramp(50, 150, 200, 1199);
        c.clock = on_oschf ? TcdClock::oschf : TcdClock::clkper;
        const char* who = on_oschf ? "TCD on OSCHF" : "TCD on CLK_PER";
        verdict("init ", who, D::init(c));

        const Stat at24 = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 6);
        const uint32_t us24 = Cycle::us(at24.mean());
        verdict("switch to 12 MHz", DynClock::set(12'000'000u));
        delay_us(DynClock{}, 2000);
        const Stat at12 = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 6);
        const uint32_t us12 = Cycle::us(at12.mean());
        verdict("back to 24 MHz", DynClock::set(24'000'000u));
        delay_us(DynClock{}, 2000);
        const Stat back = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 6);
        const uint32_t usb = Cycle::us(back.mean());

        print(serial, "    ", who, ": 24 MHz -> ", at24.mean(), " ticks (", us24,
              " us); 12 MHz -> ", at12.mean(), " ticks (", us12, " us); back -> ",
              back.mean(), " ticks (", usb, " us)", crlf);
        if (on_oschf) {
            verdict("a TCD on OSCHF keeps its period in TIME across the rebase",
                    near(static_cast<int32_t>(us24), static_cast<int32_t>(us12), 3));
            verdict("and its tick count HALVES, because the meter's ticks did",
                    near(at12.mean(), at24.mean() / 2, 3));
        } else {
            verdict("a TCD on CLK_PER keeps its period in TICKS across the rebase",
                    near(at12.mean(), at24.mean(), 2));
            verdict("and its period in TIME doubles with the main clock",
                    near(static_cast<int32_t>(us12), static_cast<int32_t>(us24) * 2, 4));
        }
        verdict("back at 24 MHz the reading returns ", who,
                near(back.mean(), at24.mean(), 2));
    }
    quiesce();
    Cycle::init(clock, ChA{});
    quiet_meters();
}

// ---- f: the PLL, on the wire at last -----------------------------------------

/// One PLL (or plain OSCHF) leg: configure, run the TCD on it, measure
/// the cycle in CLK_PER ticks. The main clock stays on the 24 MHz
/// crystal throughout, so a CLK_PER tick is an exact time unit and the
/// measured tick count is inversely proportional to CLK_TCD.
uint16_t pll_leg(const char* name, uint32_t oschf_hz, PllMultiplier mul, uint16_t period) {
    (void)D::disable();
    Pll::stop();
    (void)Oschf::set_hz(oschf_hz);
    if (mul != PllMultiplier::off) {
        if (!Pll::start(PllSource::oschf, mul)) {
            print(serial, "    ", name, ": PLL refused", crlf);
            return 0;
        }
    }
    TcdConfig c = base_one_ramp(50, 150, 200, static_cast<uint16_t>(period - 1));
    c.clock = mul == PllMultiplier::off ? TcdClock::oschf : TcdClock::pll;
    if (!D::init(c)) {
        print(serial, "    ", name, ": TCD init refused (no CLK_TCD?)", crlf);
        return 0;
    }
    delay_us(clock, 2000);
    const Stat s = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 8);
    const uint32_t hz = s.mean() ? (24'000'000u / s.mean()) : 0;
    print(serial, "    ", name, ": cycle = ", s.mean(), " CLK_PER ticks (", hz,
          " Hz), PLLS = ", Pll::locked(), crlf);
    return s.mean();
}

void tf_pll() {
    print(serial, "f the PLL: its only consumer is the TCD, so its multipliers are "
                  "proven by the TCD's own cycle rate", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});
    constexpr uint16_t period = 3200;              // CMPBCLR + 1, inside the 12 bits

    // Every leg runs from the SAME oscillator, so the ratios between the
    // legs are exact even though OSCHF itself is only +-2..5 %.
    const uint16_t t16 = pll_leg("OSCHF 16 MHz, no PLL", 16'000'000u, PllMultiplier::off, period);
    const uint16_t t16x2 = pll_leg("OSCHF 16 MHz x2 = 32 MHz", 16'000'000u, PllMultiplier::x2, period);
    verdict("PLLS locks with the TCD requesting the PLL", Pll::locked());
    const uint16_t t16x3 = pll_leg("OSCHF 16 MHz x3 = 48 MHz", 16'000'000u, PllMultiplier::x3, period);
    const uint16_t t24 = pll_leg("OSCHF 24 MHz, no PLL", 24'000'000u, PllMultiplier::off, period);
    const uint16_t t24x2 = pll_leg("OSCHF 24 MHz x2 = 48 MHz", 24'000'000u, PllMultiplier::x2, period);

    verdict("every leg produced a reading",
            t16 && t16x2 && t16x3 && t24 && t24x2);
    // ratio = t(no PLL) / t(PLL): the multiplier itself, in per-mille.
    if (t16x2 && t16x3 && t24x2) {
        const uint32_t r2 = (static_cast<uint32_t>(t16) * 1000u + t16x2 / 2) / t16x2;
        const uint32_t r3 = (static_cast<uint32_t>(t16) * 1000u + t16x3 / 2) / t16x3;
        const uint32_t r24 = (static_cast<uint32_t>(t24) * 1000u + t24x2 / 2) / t24x2;
        print(serial, "    measured multipliers (x1000): 16 MHz x2 -> ", r2,
              ", 16 MHz x3 -> ", r3, ", 24 MHz x2 -> ", r24, crlf);
        verdict("MULFAC 2x measures 2.000 against its own oscillator",
                near(static_cast<int32_t>(r2), 2000, 8));
        verdict("MULFAC 3x measures 3.000 against its own oscillator",
                near(static_cast<int32_t>(r3), 3000, 12));
        verdict("MULFAC 2x from 24 MHz measures 2.000",
                near(static_cast<int32_t>(r24), 2000, 8));
        verdict("16 MHz x3 and 24 MHz x2 are the same 48 MHz",
                near(t16x3, t24x2, 8));
    }

    // Errata 2.5.3 (DB A4/A5, DA every revision): PLLS never sets with
    // RUNSTDBY and no requester.
    {
        (void)D::release();                        // the requester goes away
        Pll::stop();
        delay_us(clock, 1000);
        verdict("PLL stopped: PLLS clear", !Pll::locked());
        verdict("start with RUNSTDBY and no requester",
                Pll::start(PllSource::oschf, PllMultiplier::x2, true));
        uint16_t spins = 0;
        while (!Pll::locked() && spins < 0xFFFF) ++spins;
        print(serial, "    RUNSTDBY = 1, TCD down: PLLS = ", Pll::locked(), " after ",
              spins, " spins", crlf);
        verdict("errata 2.5.3: PLLS stays 0 with RUNSTDBY and no requester",
                !Pll::locked());
        TcdConfig c = base_one_ramp(50, 150, 200, static_cast<uint16_t>(period - 1));
        c.clock = TcdClock::pll;
        verdict("the TCD becomes the requester", D::init(c));
        spins = 0;
        while (!Pll::locked() && spins < 0xFFFF) ++spins;
        print(serial, "    with the TCD requesting: PLLS = ", Pll::locked(), " after ",
              spins, " spins", crlf);
        verdict("PLLS sets as soon as a peripheral requests the PLL", Pll::locked());
    }

    // Errata 2.5.4 (DB A4/A5): the PLL will not run from an XOSCHF
    // CRYSTAL - and this board's 24 MHz source IS a crystal. The driver
    // refuses the combination outright; the refusal is what is observed.
    {
        const uint8_t before = CLKCTRL.PLLCTRLA;
        verdict("XOSCHF is in crystal mode on this board",
                (CLKCTRL.XOSCHFCTRLA & CLKCTRL_SELHF_bm) == 0);
        verdict("errata 2.5.4: source_ok(xoschf) is false with a crystal",
                !Pll::source_ok(PllSource::xoschf));
        verdict("errata 2.5.4: start(xoschf, ...) is refused",
                !Pll::start(PllSource::xoschf, PllMultiplier::x2));
        verdict("and PLLCTRLA was left untouched", CLKCTRL.PLLCTRLA == before);
        verdict("source_ok(oschf) is true", Pll::source_ok(PllSource::oschf));
    }

    quiesce();
    Pll::stop();
    (void)Oschf::set_hz(24'000'000u);              // back to the fallback rate
}

// ---- g: capture --------------------------------------------------------------

void tg_capture() {
    print(serial, "g capture: the software strobes, the event captures, and the "
                  "chapter's own PWM-capture example", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    // A slow counter, so a software capture can be spaced by a known
    // wait: CLK_PER / (8 x 32) = 93750 Hz, cycle 4096 ticks = 43.7 ms.
    {
        TcdConfig c = base_one_ramp(100, 1000, 2000, 4095);
        c.sync_prescaler = TcdSyncPrescaler::div8;
        c.count_prescaler = TcdCountPrescaler::div32;
        verdict("slow init (CLK_TCD_CNT = 93750 Hz)", D::init(c));
        delay_us(clock, 1000);
        // PRIMING READ. A capture register is "blocked for an update of
        // new capture data until the higher byte of this register is
        // read" (25.5.19), so the first read of a fresh sequence hands
        // back whatever the register was blocked on - a value from an
        // earlier test, not the counter now.
        const uint16_t stale = D::capture_a();
        // Capture first, print afterwards: a console line at 460800 is
        // itself worth a hundred counter ticks here.
        uint16_t v[8] = {};
        bool strobed = true;
        for (uint8_t i = 0; i < 8; ++i) {
            if (!D::software_capture_a()) { strobed = false; break; }
            v[i] = D::capture_a();
            delay_us(clock, 2000);
        }
        verdict("eight SCAPTUREA strobes", strobed);
        print(serial, "    the priming read (the value the register was blocked on) = ",
              stale, crlf);
        bool rising = true;
        bool in_range = true;
        bool spaced = true;
        print(serial, "    software captures 2 ms apart (CLK_TCD_CNT = 93750 Hz, so "
                      "+187 counts each):", crlf);
        for (uint8_t i = 0; i < 8; ++i) {
            const int32_t d = i ? static_cast<int32_t>(v[i]) - static_cast<int32_t>(v[i - 1]) : 0;
            print(serial, "      CAPTUREA = ", v[i]);
            if (i) print(serial, "  (delta ", d, ")");
            print(serial, crlf);
            if (v[i] > 4095) in_range = false;
            if (i && v[i] <= v[i - 1]) rising = false;
            // The first one or two intervals after an enable run long by
            // twenty-odd counts, repeatably; from the third on the
            // spacing is exactly what the wait is worth, to the count.
            if (i >= 4 && !near(d, 188, 1)) spaced = false;
        }
        verdict("every software capture is inside the 12-bit range", in_range);
        verdict("the captures advance with the counter", rising);
        verdict("2 ms of wait = 188 counter ticks at 93750 Hz, to the count "
                "(the first intervals after an enable settle first)", spaced);
        (void)D::capture_b();                      // prime B the same way
        verdict("software capture B works too", D::software_capture_b());
        print(serial, "      CAPTUREB = ", D::capture_b(), crlf);
    }
    quiesce();

    // An event capture: PD3's rising edge, input mode 0 (no fault action).
    {
        arm_level_sources();
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.input_a = {.enable = true, .action = TcdEventAction::capture, .rising = true,
                     .config = TcdEventConfig::neither, .mode = TcdInputMode::none};
        verdict("init with input A capturing", D::init(c));
        arm_level_sources();
        D::clear_flags();
        (void)D::capture_a();                      // priming read: see above
        uint8_t hits = 0;
        bool in_range = true;
        uint16_t first = 0, last = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            LevelA::set();
            delay_us(clock, 200);
            if (D::trig_a_flag()) {
                const uint16_t v = D::capture_a();
                if (i == 0) first = v;
                last = v;
                if (v > 1199) in_range = false;
                ++hits;
                D::clear_trig_a();
            }
            LevelA::clear();
            delay_us(clock, 700);
        }
        print(serial, "    event captures: ", hits, "/4 TRIGA, first = ", first,
              ", last = ", last, crlf);
        verdict("every rising edge of the input raised TRIGA", hits == 4);
        verdict("the captured counter values are inside the cycle", in_range);
        verdict("the captures are not all the same value", first != last);
    }
    quiesce();

    // Example 25-3: both event inputs on ONE channel, opposite edges -
    // the duty of the very signal the TCD generates, captured by the TCD.
    {
        TcdConfig c = base_one_ramp(200, 800, 1000, 1999);
        c.input_a = {.enable = true, .action = TcdEventAction::capture, .rising = true,
                     .config = TcdEventConfig::neither, .mode = TcdInputMode::none};
        c.input_b = {.enable = true, .action = TcdEventAction::capture, .rising = false,
                     .config = TcdEventConfig::neither, .mode = TcdInputMode::none};
        verdict("init for the PWM-capture example", D::init(c));
        // Both inputs listen to WOA's own pin event.
        D::input_a_on(ChA{});
        D::input_b_on(ChA{});
        const Wave w = measure();
        show_wave(w);
        // The capture register is blocked for a new value until its HIGH
        // byte is read, so each read/clear pair takes the FIRST capture
        // of the next window - four of them tell whether the value is an
        // edge or the counter as it stands.
        uint16_t rise[4] = {0, 0, 0, 0};
        uint16_t fall[4] = {0, 0, 0, 0};
        bool both = true;
        (void)D::capture_a();                      // priming reads: see above
        (void)D::capture_b();
        for (uint8_t i = 0; i < 4; ++i) {
            D::clear_flags();
            delay_us(clock, 400);                  // ~5 TCD cycles of 2000 ticks
            both = both && D::trig_a_flag() && D::trig_b_flag();
            rise[i] = D::capture_a();
            fall[i] = D::capture_b();
        }
        bool rise_ok = true, fall_ok = true, on_ok = true;
        for (uint8_t i = 0; i < 4; ++i) {
            const int32_t on = static_cast<int32_t>(fall[i]) - static_cast<int32_t>(rise[i]);
            print(serial, "    CAPTUREA (WOA rise, CMPASET = 200) = ", rise[i],
                  ", CAPTUREB (WOA fall, CMPACLR = 800) = ", fall[i], ", on-time = ", on,
                  " ticks", crlf);
            if (!near(rise[i], 203, 2)) rise_ok = false;
            if (!near(fall[i], 803, 2)) fall_ok = false;
            if (!near(on, w.high_a.mean(), 2)) on_ok = false;
        }
        print(serial, "    both captures carry the SAME +3 CLK_TCD_CNT offset (the pin "
                      "event's trip through EVSYS and the capture synchronizer), so the "
                      "difference is exact", crlf);
        verdict("both triggers fired on one channel with opposite edges", both);
        verdict("the rising capture lands on CMPASET", rise_ok);
        verdict("the falling capture lands on CMPACLR", fall_ok);
        verdict("the captured on-time agrees with the pulse-width meter", on_ok);
        D::input_a_off();
        D::input_b_off();
    }
    quiesce();
}

// ---- h: the input modes ------------------------------------------------------

/// The base for the input-mode round: one ramp on CLK_PER with the
/// synchronizer divided by 8 (CLK_TCD_SYNC = 3 MHz), a 1.2 ms cycle -
/// slow enough for software to act inside one on-time.
TcdConfig input_base(TcdInputMode mode, TcdEventConfig cfg, TcdEventAction action) {
    TcdConfig c = base_one_ramp(300, 1800, 2100, 3599);
    c.sync_prescaler = TcdSyncPrescaler::div8;
    c.input_a = {.enable = true, .action = action, .rising = true, .config = cfg,
                 .mode = mode};
    return c;
}

/// Count TCD cycles over `us` by watching the peripheral's own CMPBCLR
/// event: the one instrument that keeps working when every output has
/// been faulted off.
uint16_t cycles_in(uint32_t us) {
    Edges::reset();
    delay_us(clock, us);
    return Edges::count();
}

/// Is this output still moving? Sample the pin flat out for about ten
/// TCD cycles and report whether both levels were seen. The direct
/// answer, next to what STATUS.PWMACT claims.
template <typename P>
bool output_moving(uint32_t samples = 100'000) {
    const bool first = P::read();
    while (samples--) {
        if (P::read() != first) return true;
    }
    return false;
}

void th_input_modes() {
    print(serial, "h the input modes, the qualifiers (async / filter / blanking) and "
                  "errata 2.14.1 and 2.14.3", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});
    ChTcd::source(EvTcdCmpBClr{});
    Edges::init(ChTcd{});
    quiet_meters();
    arm_level_sources();

    // --- mode 0: no action on the outputs
    {
        verdict("mode 0 init", D::init(input_base(TcdInputMode::none, TcdEventConfig::neither,
                                                  TcdEventAction::capture)));
        arm_level_sources();
        const uint16_t free_run = cycles_in(20'000);
        LevelA::set();
        const uint16_t held = cycles_in(20'000);
        LevelA::clear();
        print(serial, "    mode 0: ", free_run, " cycles free, ", held,
              " with the input held high (20 ms each)", crlf);
        verdict("mode 0 leaves the counter alone", near(free_run, held, 2));
        verdict("mode 0 still captures", D::trig_a_flag());
    }
    quiesce();
    arm_level_sources();

    // --- mode 4: stop all outputs, maintain the frequency
    {
        verdict("mode 4 init", D::init(input_base(TcdInputMode::freq, TcdEventConfig::neither,
                                                  TcdEventAction::fault)));
        arm_level_sources();
        const uint16_t free_run = cycles_in(20'000);
        D::clear_pwm_activity();
        const bool move_a = output_moving<WoaPin>();
        const bool move_b = output_moving<WobPin>();
        const TcdActivity moving = D::take_pwm_activity();
        LevelA::set();
        delay_us(clock, 2000);
        D::clear_pwm_activity();
        const uint16_t held = cycles_in(20'000);
        const bool held_a = output_moving<WoaPin>();
        const bool held_b = output_moving<WobPin>();
        const TcdActivity stopped = D::take_pwm_activity();
        LevelA::clear();
        delay_us(clock, 2000);
        D::clear_pwm_activity();
        const bool back_a = output_moving<WoaPin>();
        const bool back_b = output_moving<WobPin>();
        const TcdActivity again = D::take_pwm_activity();
        print(serial, "    mode 4: ", free_run, " cycles free, ", held,
              " while faulted; the PINS move A/B = ", move_a, move_b, " free, ",
              held_a, held_b, " faulted, ", back_a, back_b, " released", crlf);
        print(serial, "    and STATUS.PWMACT A/B reads ", moving.a, moving.b, " / ",
              stopped.a, stopped.b, " / ", again.a, again.b,
              " over the same three windows", crlf);
        verdict("mode 4 keeps the frequency", near(free_run, held, 2));
        verdict("both outputs move when nothing is faulted", move_a && move_b);
        verdict("mode 4 stops BOTH outputs while the input is active",
                !held_a && !held_b);
        verdict("both outputs come back when the input is released", back_a && back_b);

        // STATUS.PWMACT read 1/1 in all three windows above, including
        // the one where the pins provably did not move. Two probes tell
        // whether the W1C clear fails or the detector watches something
        // other than the pad.
        D::clear_pwm_activity();
        const TcdActivity immediate = D::take_pwm_activity();
        (void)D::disable();
        D::clear_pwm_activity();
        delay_us(clock, 5000);
        const TcdActivity down = D::take_pwm_activity();
        (void)D::enable();
        print(serial, "    PWMACT right after a clear with the TCD running = ",
              immediate.a, immediate.b, "; with the TCD DISABLED for 5 ms = ", down.a,
              down.b, crlf);
        verdict("the W1C clear does hold once nothing can toggle (TCD disabled)",
                !down.a && !down.b);
        verdict("so PWMACT follows the WAVEFORM GENERATOR, not the pad: it keeps "
                "setting while a fault holds the outputs still",
                stopped.a && stopped.b && !held_a && !held_b);
    }
    quiesce();
    arm_level_sources();

    // --- mode 10: stop the corresponding output at level, keep the rest
    {
        verdict("mode 10 init",
                D::init(input_base(TcdInputMode::level_trig_freq, TcdEventConfig::neither,
                                   TcdEventAction::fault)));
        arm_level_sources();
        LevelA::set();
        delay_us(clock, 2000);
        D::clear_pwm_activity();
        const uint16_t held = cycles_in(20'000);
        const bool held_a = output_moving<WoaPin>();
        const bool held_b = output_moving<WobPin>();
        const TcdActivity a = D::take_pwm_activity();
        LevelA::clear();
        print(serial, "    mode 10 with the input held: the pins move A/B = ", held_a,
              held_b, " (PWMACT reads ", a.a, a.b, "), ", held, " cycles in 20 ms", crlf);
        verdict("mode 10 blocks WOA and leaves WOB running", !held_a && held_b);
        verdict("mode 10 keeps the frequency", held > 10);
    }
    quiesce();
    arm_level_sources();

    // --- modes 8 and 9: an edge inside the on-time
    for (uint8_t which = 0; which < 2; ++which) {
        const TcdInputMode m = which ? TcdInputMode::edge_trig_freq : TcdInputMode::edge_trig;
        const char* name = which ? "mode 9 (edge, keep the frequency)"
                                 : "mode 8 (edge, jump to the next compare cycle)";
        verdict("init ", name, D::init(input_base(m, TcdEventConfig::neither,
                                                  TcdEventAction::fault)));
        arm_level_sources();
        const Stat nominal = gather<WidthATcb>([] { return WidthA::width_ticks(); }, 4);
        const uint16_t free_run = cycles_in(20'000);
        // Wait for WOA to be high, then fire one edge inside its on-time.
        WidthATcb::clear_capt();
        uint32_t spins = 400'000;
        while (!WoaPin::read() && --spins) {
        }
        delay_us(clock, 20);
        LevelA::set();
        delay_us(clock, 5);
        LevelA::clear();
        spins = 400'000;
        while (!WidthATcb::capt_flag() && --spins) {
        }
        const uint16_t cut = WidthA::width_ticks();
        const uint16_t held = cycles_in(20'000);
        print(serial, "    ", name, ": nominal WOA on-time ", nominal.mean(),
              " ticks, the interrupted one ", cut, " ticks; cycles 20 ms: ", free_run,
              " free / ", held, " with one edge", crlf);
        verdict("the edge cut the on-time short, ", name,
                cut < nominal.mean() && cut > 0);
        if (which) {
            verdict("mode 9 leaves the counter alone", near(free_run, held, 2));
        }
    }
    quiesce();
    arm_level_sources();

    // --- mode 7 and errata 2.14.3
    {
        verdict("mode 7 init with CMPASET = 300",
                D::init(input_base(TcdInputMode::wait_sw, TcdEventConfig::neither,
                                   TcdEventAction::fault)));
        arm_level_sources();
        const uint16_t free_run = cycles_in(20'000);
        LevelA::set();
        delay_us(clock, 2000);
        const uint16_t halted = cycles_in(20'000);
        LevelA::clear();
        delay_us(clock, 2000);
        const uint16_t still = cycles_in(20'000);
        verdict("RESTART strobe", D::restart());
        delay_us(clock, 2000);
        const uint16_t resumed = cycles_in(20'000);
        print(serial, "    mode 7: ", free_run, " cycles free, ", halted, " while held, ",
              still, " after release, ", resumed, " after RESTART (20 ms each)", crlf);
        verdict("mode 7 halts the counter while the input is active", halted <= 1);
        verdict("mode 7 stays halted after the input goes away", still <= 1);
        verdict("a RESTART command restarts it", resumed > 10);

        // The erratum, positively observed. tcd_config_valid() refuses
        // CMPASET = 0 with mode 7, so the broken combination is built
        // through the resource's own verbs - which is what those verbs
        // are for. What "does not work" MEANS is measured here rather
        // than assumed: the halt AND the software restart, both halves.
        (void)D::disable();
        verdict("CMPASET = 0 through the raw verb", D::compare_a_set(0));
        verdict("SYNC it into the TCD domain", D::sync());
        verdict("re-enable", D::enable());
        delay_us(clock, 2000);
        const uint16_t z_free = cycles_in(20'000);
        LevelA::set();
        delay_us(clock, 2000);
        const uint16_t z_held = cycles_in(20'000);
        LevelA::clear();
        delay_us(clock, 2000);
        const uint16_t z_still = cycles_in(20'000);
        const bool z_restart = D::restart();
        delay_us(clock, 2000);
        const uint16_t z_resumed = cycles_in(20'000);
        const bool z_moving = output_moving<WoaPin>();
        print(serial, "    mode 7 with CMPASET = 0: ", z_free, " cycles free, ", z_held,
              " while held, ", z_still, " after release, ", z_resumed,
              " after RESTART (strobe accepted: ", z_restart, "); WOA moving after: ",
              z_moving, crlf);
        print(serial, "    errata 2.14.3 says this combination 'does not work'; the "
                      "numbers above are what it actually does on this die", crlf);
        TcdConfig broken = input_base(TcdInputMode::wait_sw, TcdEventConfig::neither,
                                      TcdEventAction::fault);
        broken.compare_a_set = 0;
        verdict("tcd_config_valid() refuses mode 7 with CMPASET = 0",
                !tcd_config_valid(broken));
        broken.compare_a_set = 300;
        verdict("and accepts it with a non-zero CMPASET", tcd_config_valid(broken));
    }
    quiesce();
    arm_level_sources();

    // --- async vs synchronous override latency
    {
        int32_t lat[2] = {0, 0};
        for (uint8_t leg = 0; leg < 2; ++leg) {
            const TcdEventConfig cfg = leg ? TcdEventConfig::async : TcdEventConfig::neither;
            TcdConfig c = input_base(TcdInputMode::level_trig_freq, cfg,
                                     TcdEventAction::fault);
            if (!D::init(c)) { verdict("latency leg init", false); continue; }
            arm_level_sources();
            // A free-running CLK_PER counter that latches WOA's FALLING
            // edge: the stamp technique.
            CountTcb::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0,
                            .event_input = true, .edge = true});
            CountTcb::capture_on(ChA{});
            quiet_meters();
            // The stamp is a SIGNED difference: with ASYNCON the override
            // can land before the counter read that follows the PORT
            // store, and an unsigned subtraction would print 65533.
            int16_t best = 32767;
            for (uint8_t i = 0; i < 8; ++i) {
                LevelA::clear();
                delay_us(clock, 2000);
                uint32_t spins = 400'000;
                while (!WoaPin::read() && --spins) {
                }
                delay_us(clock, 30);
                CountTcb::clear_capt();
                LevelA::set();
                const uint16_t t0 = CountTcb::count();
                spins = 400'000;
                while (!CountTcb::capt_flag() && --spins) {
                }
                const uint16_t t1 = CountTcb::capture();
                const int16_t d = static_cast<int16_t>(static_cast<uint16_t>(t1 - t0));
                if (d < best) best = d;
            }
            LevelA::clear();
            lat[leg] = best;
            print(serial, "    ", leg ? "ASYNCON" : "synchronous",
                  " override latency (best of 8) = ", best,
                  " CLK_PER ticks from the PORT store", crlf);
        }
        const int32_t diff = lat[0] - lat[1];
        print(serial, "    CLK_TCD_SYNC = CLK_PER / 8 here, so one synchronizer cycle is "
                      "8 CLK_PER ticks; the differential is ", diff, " ticks = ",
              diff / 8, " synchronizer cycles", crlf);
        verdict("the asynchronous path overrides the output sooner", lat[1] < lat[0]);
        verdict("the synchronous path costs about two to three CLK_TCD_SYNC cycles",
                diff >= 8 && diff <= 40);
        // Put the meters back the way the rest of the test wants them.
        arm_meters(ChA{}, ChB{});
        Edges::init(ChTcd{});
        quiet_meters();
    }
    quiesce();
    arm_level_sources();

    // --- the digital filter: four CLK_TCD_CNT cycles
    {
        TcdConfig c = input_base(TcdInputMode::none, TcdEventConfig::filter,
                                 TcdEventAction::capture);
        c.count_prescaler = TcdCountPrescaler::div32;      // CLK_TCD_CNT = 93750 Hz
        verdict("filter init (CLK_TCD_CNT = 93750 Hz: four cycles = 43 us)", D::init(c));
        arm_level_sources();
        D::clear_flags();
        LevelA::set();
        delay_us(clock, 5);                                 // well under four cycles
        LevelA::clear();
        delay_us(clock, 500);
        const bool short_seen = D::trig_a_flag();
        D::clear_flags();
        LevelA::set();
        delay_us(clock, 400);                               // well over four cycles
        const bool long_seen = D::trig_a_flag();
        LevelA::clear();
        print(serial, "    filter: a 5 us pulse ", short_seen ? "PASSED" : "was rejected",
              ", a 400 us pulse ", long_seen ? "passed" : "WAS REJECTED", crlf);
        verdict("the digital filter rejects a pulse shorter than four counter cycles",
                !short_seen);
        verdict("and passes a longer one", long_seen);
    }
    quiesce();
    arm_level_sources();

    // --- input blanking
    {
        TcdConfig c = input_base(TcdInputMode::none, TcdEventConfig::neither,
                                 TcdEventAction::capture);
        c.delay = TcdDelaySelect::input_blanking;
        c.delay_trigger = TcdDelayTrigger::cmpaset;         // the WOA rise starts the window
        c.delay_prescaler = TcdDelayPrescaler::div8;
        c.delay_value = 150;                                // 8 x 150 / 3 MHz = 400 us
        verdict("blanking init (window = 400 us from the WOA rise)", D::init(c));
        arm_level_sources();
        verdict("input blanking is what DLYCTRL selects", D::input_blanking_enabled());
        verdict("and the programmable output event therefore is not",
                !D::output_event_enabled());
        print(serial, "    blanking window = ", D::delay_cycles(),
              " CLK_TCD_SYNC cycles", crlf);

        uint8_t inside = 0, outside = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            uint32_t spins = 400'000;
            while (WoaPin::read() && --spins) {
            }
            spins = 400'000;
            while (!WoaPin::read() && --spins) {
            }
            D::clear_flags();
            delay_us(clock, 50);                            // inside the window
            LevelA::set();
            delay_us(clock, 20);
            LevelA::clear();
            delay_us(clock, 50);
            if (D::trig_a_flag()) ++inside;

            spins = 400'000;
            while (WoaPin::read() && --spins) {
            }
            spins = 400'000;
            while (!WoaPin::read() && --spins) {
            }
            D::clear_flags();
            delay_us(clock, 600);                           // past the window
            LevelA::set();
            delay_us(clock, 20);
            LevelA::clear();
            delay_us(clock, 50);
            if (D::trig_a_flag()) ++outside;
        }
        print(serial, "    blanking: ", inside, "/4 events honoured INSIDE the window, ",
              outside, "/4 honoured after it", crlf);
        verdict("an event inside the blanking window is ignored", inside == 0);
        verdict("an event after it is honoured", outside == 4);
    }
    quiesce();
    arm_level_sources();

    // --- errata 2.14.1: ASYNC events + a counter prescaler lose events.
    // The erratum names the CONDITION (CFG = ASYNC and CNTPRES != DIV1)
    // but not how short an event has to be to fall through, so the sweep
    // is over both axes: the three counter prescalers by three pulse
    // widths, with SYNCPRES pinned at DIV1 so only CNTPRES moves.
    {
        struct Leg { TcdCountPrescaler p; const char* name; };
        static constexpr Leg legs[] = {
            {TcdCountPrescaler::div1, "CNTPRES = 1 (the workaround)"},
            {TcdCountPrescaler::div4, "CNTPRES = 4"},
            {TcdCountPrescaler::div32, "CNTPRES = 32"},
        };
        static constexpr uint16_t widths[] = {0, 1, 4};      // us; 0 = back-to-back stores
        uint8_t seen[3][3] = {};
        for (uint8_t l = 0; l < 3; ++l) {
            TcdConfig c = input_base(TcdInputMode::none, TcdEventConfig::async,
                                     TcdEventAction::capture);
            c.sync_prescaler = TcdSyncPrescaler::div1;
            c.count_prescaler = legs[l].p;
            if (!D::init(c)) { verdict("errata 2.14.1 leg init", false); continue; }
            arm_level_sources();
            for (uint8_t wi = 0; wi < 3; ++wi) {
                uint8_t hits = 0;
                for (uint8_t i = 0; i < 16; ++i) {
                    D::clear_flags();
                    LevelA::set();
                    if (widths[wi]) delay_us(clock, widths[wi]);
                    LevelA::clear();
                    delay_us(clock, 200);
                    if (D::trig_a_flag()) ++hits;
                }
                seen[l][wi] = hits;
            }
            print(serial, "    ", legs[l].name, ", ASYNCON: pulses seen out of 16 - "
                  "back-to-back ", seen[l][0], ", 1 us ", seen[l][1], ", 4 us ",
                  seen[l][2], crlf);
        }
        const bool lost = seen[1][0] < 16 || seen[1][1] < 16 || seen[1][2] < 16 ||
                          seen[2][0] < 16 || seen[2][1] < 16 || seen[2][2] < 16;
        const bool clean = seen[0][0] == 16 && seen[0][1] == 16 && seen[0][2] == 16;
        print(serial, "    the CAPTURE path (ACTION = capture, INPUTMODE = 0) loses ",
              lost ? "some" : "NOTHING", " here", crlf);
        verdict("CNTPRES = DIV1 (the documented workaround) loses no capture", clean);
        verdict("on THIS die the capture path loses none with a counter prescaler "
                "either - errata 2.14.1 NOT reproduced there", !lost);

        // The erratum says "events can be missed", and the events that
        // matter to this peripheral are the ones that reach the OUTPUT
        // override, not only the ones that reach a capture register. The
        // fault path gets its own sweep: input mode 9 blocks the rest of
        // the on-time on a positive edge, so a missed event is an
        // on-time that came out full length. Both legs are arranged to
        // the SAME 800 us cycle and 400 us on-time, so only the
        // prescaler that produces them differs.
        struct FaultLeg {
            TcdSyncPrescaler s;
            TcdCountPrescaler c;
            uint16_t a_set, a_clr, b_set, b_clr;
            const char* name;
        };
        static constexpr FaultLeg fl[] = {
            {TcdSyncPrescaler::div1, TcdCountPrescaler::div32, 100, 400, 450, 599,
             "SYNCPRES = 1, CNTPRES = 32 (the erratum's condition)"},
            {TcdSyncPrescaler::div8, TcdCountPrescaler::div1, 400, 1600, 1800, 2399,
             "SYNCPRES = 8, CNTPRES = 1 (the documented workaround)"},
        };
        uint8_t blocked[2] = {0, 0};
        for (uint8_t l = 0; l < 2; ++l) {
            TcdConfig c = base_one_ramp(fl[l].a_set, fl[l].a_clr, fl[l].b_set, fl[l].b_clr);
            c.sync_prescaler = fl[l].s;
            c.count_prescaler = fl[l].c;
            c.input_a = {.enable = true, .action = TcdEventAction::fault, .rising = true,
                         .config = TcdEventConfig::async,
                         .mode = TcdInputMode::edge_trig_freq};
            if (!D::init(c)) { verdict("errata 2.14.1 fault leg init", false); continue; }
            arm_level_sources();
            const Stat nominal = gather<WidthATcb>([] { return WidthA::width_ticks(); }, 4);
            uint8_t hits = 0;
            for (uint8_t i = 0; i < 8; ++i) {
                uint32_t spins = 400'000;
                while (WoaPin::read() && --spins) {
                }
                spins = 400'000;
                while (!WoaPin::read() && --spins) {
                }
                WidthATcb::clear_capt();
                delay_us(clock, 100);                      // a quarter into the on-time
                LevelA::set();                             // back-to-back: the shortest event
                LevelA::clear();
                spins = 400'000;
                while (!WidthATcb::capt_flag() && --spins) {
                }
                if (WidthA::width_ticks() + 200 < nominal.mean()) ++hits;
            }
            blocked[l] = hits;
            print(serial, "    ", fl[l].name, ": nominal on-time ", nominal.mean(),
                  " ticks, ", hits, "/8 shortest-possible ASYNC events blocked it", crlf);
        }
        verdict("the SYNCPRES workaround misses no output override", blocked[1] == 8);
        verdict("and on THIS die neither does CNTPRES = 32 - errata 2.14.1 NOT "
                "reproduced on the output path either", blocked[0] == 8);
        print(serial, "    errata 2.14.1 says asynchronous input events can be missed "
                      "with CNTPRES != DIV1 on this revision. Neither path lost one here, "
                      "at any prescaler, down to a back-to-back set/clear (~125 ns). The "
                      "driver carries the erratum and names the workaround; the bench "
                      "records it as NOT REPRODUCED, not as absent.", crlf);
    }
    quiesce();
}

// ---- i: dithering ------------------------------------------------------------

void ti_dither() {
    print(serial, "i dithering: the fractional cycle, and the mode-dependent table", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    constexpr uint16_t period = 600;               // CMPBCLR + 1
    // On-time B dithering adds ONE tick to the cycle at every accumulator
    // overflow (table 25-7). DITHER = 8 overflows every second cycle, so
    // 32 cycles carry exactly 16 extra ticks.
    {
        TcdConfig c = base_one_ramp(50, 150, 200, period - 1);
        c.dither_select = TcdDitherSelect::on_time_b;
        c.dither = 8;
        verdict("one ramp + DITHERSEL = on-time B, DITHER = 8", D::init(c));
        const Stat s = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 32);
        show("cycle", s);
        const uint32_t want = 32u * period + 16u;
        print(serial, "    32 cycles summed = ", s.sum, " ticks, expected 32 x ", period,
              " + 16 = ", want, crlf);
        verdict("the accumulator adds exactly DITHER/16 of a tick per cycle",
                near(static_cast<int32_t>(s.sum), static_cast<int32_t>(want), 3));
        verdict("and it does it by lengthening single cycles by one tick",
                s.min == period && s.max == period + 1);
        verdict("tcd_dither_cycle_cost() says one ramp + on-time B costs 1",
                tcd_dither_cycle_cost(TcdWaveform::one_ramp, TcdDitherSelect::on_time_b) == 1);
    }

    // Dead-time B dithering adds NOTHING to the one-ramp cycle: the tick
    // is taken out of the following on-time instead.
    {
        TcdConfig c = base_one_ramp(50, 150, 200, period - 1);
        c.dither_select = TcdDitherSelect::dead_time_b;
        c.dither = 8;
        verdict("one ramp + DITHERSEL = dead-time B, DITHER = 8", D::init(c));
        const Stat s = gather<CycleTcb>([] { return Cycle::period_ticks(); }, 32);
        const Stat b = gather<WidthBTcb>([] { return WidthB::width_ticks(); }, 32);
        show("cycle", s);
        show("WOB high", b);
        print(serial, "    32 cycles summed = ", s.sum, " ticks, expected 32 x ", period,
              " = ", 32u * period, "; WOB nominal = CMPBCLR - CMPBSET = 399", crlf);
        verdict("the cycle length does not move",
                near(static_cast<int32_t>(s.sum), static_cast<int32_t>(32u * period), 3));
        verdict("the compensation comes out of on-time B instead",
                b.max == 399 && b.min == 398);
        verdict("tcd_dither_cycle_cost() says one ramp + dead-time B costs 0",
                tcd_dither_cycle_cost(TcdWaveform::one_ramp, TcdDitherSelect::dead_time_b) == 0);
    }

    // Dithering is not supported in Dual Slope mode - refused.
    {
        TcdConfig c{};
        c.route = TcdRoute::def;
        c.clock = TcdClock::clkper;
        c.waveform = TcdWaveform::dual_slope;
        c.compare_a_set = 100;
        c.compare_b_set = 300;
        c.compare_b_clear = 599;
        c.enable_woa = true;
        c.dither = 8;
        verdict("dithering in dual slope is refused at run time", !D::init(c));
        c.dither = 0;
        verdict("and the same configuration without it is accepted", D::init(c));
    }
    quiesce();
}

// ---- j: the output plumbing --------------------------------------------------

void tj_outputs() {
    print(serial, "j output plumbing: CMPOVR + CTRLD, WOC/WOD selection, the fault "
                  "levels, DISEOC and AUPDATE", crlf);
    quiesce();
    probe_ab();
    arm_meters(ChA{}, ChB{});

    // Table 25-13, One Ramp mode: PWMA takes CMPAVAL[1] in dead-time A
    // and CMPAVAL[0] in on-time A; PWMB takes CMPBVAL[3] in dead-time B
    // and CMPBVAL[2] in on-time B.
    {
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.compare_override = true;
        c.compare_a_value = 0b0001;                 // OTA high, DTA low: the plain waveform
        c.compare_b_value = 0b0100;                 // OTB high, DTB low
        verdict("one ramp + CMPOVR, values as the normal waveform", D::init(c));
        const Wave w = measure();
        show_wave(w);
        verdict("WOA reproduces its on-time", near(w.high_a.mean(), 300, 2));

        c.compare_a_value = 0b0010;                 // DTA high, OTA low: inverted
        verdict("one ramp + CMPOVR, WOA inverted", D::init(c));
        const Wave v = measure();
        show_wave(v);
        print(serial, "    inverted WOA high = cycle - on-time = 900 expected", crlf);
        verdict("the override inverts WOA", near(v.high_a.mean(), 900, 3));
        verdict("and leaves WOB alone", near(v.high_b.mean(), 499, 2));
    }

    // WOC and WOD follow whichever waveform CTRLC selects.
    {
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.enable_woc = true;
        c.enable_wod = true;
        c.wo_c = TcdWaveformSelect::pwm_a;
        c.wo_d = TcdWaveformSelect::pwm_b;
        verdict("all four outputs, WOC = A and WOD = B", D::init(c));
        probe_cd();
        arm_meters(ChA{}, ChB{});
        const Wave w = measure();
        show_wave(w);
        verdict("WOC carries WOA's on-time", near(w.high_a.mean(), 300, 2));
        verdict("WOD carries WOB's on-time", near(w.high_b.mean(), 499, 2));

        c.wo_c = TcdWaveformSelect::pwm_b;
        c.wo_d = TcdWaveformSelect::pwm_a;
        verdict("swap: WOC = B and WOD = A", D::init(c));
        const Wave v = measure();
        show_wave(v);
        verdict("WOC now carries WOB's on-time", near(v.high_a.mean(), 499, 2));
        verdict("WOD now carries WOA's on-time", near(v.high_b.mean(), 300, 2));
        probe_ab();
        arm_meters(ChA{}, ChB{});
    }
    quiesce();
    arm_level_sources();

    // The FAULTCTRL CMPx levels ARE what the pins take while a fault is
    // held (25.3.3.7). Mode 4 with the level source held is the fault.
    {
        for (uint8_t leg = 0; leg < 2; ++leg) {
            TcdConfig c = input_base(TcdInputMode::freq, TcdEventConfig::neither,
                                     TcdEventAction::fault);
            c.fault_woa = leg == 0;
            c.fault_wob = leg != 0;
            if (!D::init(c)) { verdict("fault-level leg init", false); continue; }
            arm_level_sources();
            LevelA::set();
            delay_us(clock, 3000);
            const bool a = WoaPin::read();
            const bool b = WobPin::read();
            LevelA::clear();
            print(serial, "    fault levels CMPA = ", c.fault_woa, " CMPB = ", c.fault_wob,
                  " -> WOA reads ", a, ", WOB reads ", b, crlf);
            verdict("the held fault drives the pins to their CMPx levels",
                    a == c.fault_woa && b == c.fault_wob);
        }
    }
    quiesce();

    // DISEOC: down at the end of the cycle, with ENRDY held low until it
    // has actually happened.
    {
        verdict("init for DISEOC", D::init(base_one_ramp(100, 400, 700, 1199)));
        ChA::source(EvPin<WoaPin>{});
        Edges::init(ChA{});
        quiet_meters();
        const uint16_t before = cycles_in(5000);
        verdict("DISEOC strobe", D::disable_at_end());
        uint16_t spins = 0;
        while (D::enabled() && spins < 0xFFFF) ++spins;
        const uint16_t after = cycles_in(5000);
        print(serial, "    DISEOC: ", before, " WOA edges in 5 ms before, ", after,
              " after; the TCD went down in ", spins, " spins", crlf);
        verdict("the TCD disabled itself at the end of the cycle", !D::enabled());
        verdict("and its output stopped", after == 0);
        verdict("ENRDY is open again once it is down", D::enable_ready());
        arm_meters(ChA{}, ChB{});
    }
    quiesce();

    // AUPDATE: the write of CMPBCLR's HIGH byte is the synchronization
    // request, so a change to CMPACLR alone waits for it.
    {
        TcdConfig c = base_one_ramp(100, 400, 700, 1199);
        c.auto_update = true;
        verdict("init with AUPDATE", D::init(c));
        const Wave w = measure();
        show("WOA high (nominal)", w.high_a);
        verdict("write CMPACLR = 900 (no CMPBCLR write yet)", D::compare_a_clear(900));
        delay_us(clock, 5000);
        const Stat held = gather<WidthATcb>([] { return WidthA::width_ticks(); }, 6);
        show("WOA high after the CMPACLR write alone", held);
        verdict("nothing moved: AUPDATE waits for the CMPBCLR write",
                near(held.mean(), 300, 2));
        verdict("write CMPBCLR (unchanged value: the write IS the request)",
                D::compare_b_clear(1199));
        delay_us(clock, 5000);
        const Stat now = gather<WidthATcb>([] { return WidthA::width_ticks(); }, 6);
        show("WOA high after the CMPBCLR write", now);
        verdict("the new on-time took effect", near(now.mean(), 800, 2));
    }
    quiesce();
}

// ---- k: the two interrupt vectors --------------------------------------------

volatile uint16_t ovf_hits = 0;
volatile uint16_t trig_a_hits = 0;
volatile uint16_t trig_b_hits = 0;

void tk_vectors() {
    print(serial, "k the two vectors: OVF once per cycle, TRIGA/TRIGB per event, the "
                  "flags W1C and no storms", crlf);
    quiesce();
    probe_ab();
    ChTcd::source(EvTcdCmpBClr{});
    Edges::init(ChTcd{});
    quiet_meters();
    arm_level_sources();

    // A ~1 kHz cycle: CLK_PER / 8 = 3 MHz, 3000 ticks.
    {
        TcdConfig c = base_one_ramp(300, 1500, 1800, 2999);
        c.sync_prescaler = TcdSyncPrescaler::div8;
        verdict("init (cycle = 1 kHz)", D::init(c));
        D::clear_flags();
        ovf_hits = 0;
        D::enable_ovf_interrupt(true);
        Edges::reset();
        delay_us(clock, 100'000);
        const uint16_t events = Edges::count();
        const uint16_t hits = ovf_hits;
        D::enable_ovf_interrupt(false);
        print(serial, "    100 ms: ", events, " CMPBCLR events, ", hits,
              " OVF interrupts", crlf);
        verdict("one OVF interrupt per TCD cycle", near(hits, events, 2));
        verdict("and about a hundred of them at 1 kHz", near(hits, 100, 5));

        // No storm once the source is quiet.
        ovf_hits = 0;
        (void)D::disable();
        D::clear_flags();
        D::enable_ovf_interrupt(true);
        delay_us(clock, 20'000);
        D::enable_ovf_interrupt(false);
        print(serial, "    with the TCD disabled: ", ovf_hits, " OVF interrupts in 20 ms",
              crlf);
        verdict("a cleared flag stays cleared: no storm", ovf_hits == 0);
    }
    quiesce();
    arm_level_sources();

    // TRIGA and TRIGB, one interrupt per injected event.
    {
        TcdConfig c = base_one_ramp(300, 1500, 1800, 2999);
        c.sync_prescaler = TcdSyncPrescaler::div8;
        c.input_a = {.enable = true, .action = TcdEventAction::capture, .rising = true,
                     .config = TcdEventConfig::neither, .mode = TcdInputMode::none};
        c.input_b = {.enable = true, .action = TcdEventAction::capture, .rising = true,
                     .config = TcdEventConfig::neither, .mode = TcdInputMode::none};
        verdict("init with both inputs capturing", D::init(c));
        arm_level_sources();
        D::clear_flags();
        trig_a_hits = 0;
        trig_b_hits = 0;
        D::enable_trig_a_interrupt(true);
        D::enable_trig_b_interrupt(true);
        for (uint8_t i = 0; i < 10; ++i) {
            LevelA::set();
            delay_us(clock, 300);
            LevelA::clear();
            delay_us(clock, 300);
        }
        for (uint8_t i = 0; i < 6; ++i) {
            LevelB::set();
            delay_us(clock, 300);
            LevelB::clear();
            delay_us(clock, 300);
        }
        D::enable_trig_a_interrupt(false);
        D::enable_trig_b_interrupt(false);
        print(serial, "    10 edges on input A -> ", trig_a_hits, " TRIGA interrupts; ",
              "6 on input B -> ", trig_b_hits, " TRIGB interrupts", crlf);
        verdict("one TRIGA per event on input A", trig_a_hits == 10);
        verdict("one TRIGB per event on input B", trig_b_hits == 6);
        verdict("the flags are down afterwards", !D::trig_a_flag() && !D::trig_b_flag());
    }
    print(serial, "    DBGRUN and FAULTDET need a halted CPU in an OCD session: not "
                  "bench-verifiable from a running suite (see the doc)", crlf);
    quiesce();
}

// ---- the menu ----------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_routes}, {'b', tb_one_ramp}, {'c', tc_ramps}, {'d', td_dual_slope},
    {'e', te_clocks}, {'f', tf_pll}, {'g', tg_capture}, {'h', th_input_modes},
    {'i', ti_dither}, {'j', tj_outputs}, {'k', tk_vectors},
};
constexpr char all_tests[] = "abcdefghijk";

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            run(t.fn);
            tp += passed;
            tf += failed;
        }
    }
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

void help() {
    print(serial, "test_avr_tcd: a routes and the sync disciplines | b one ramp | "
                  "c two and four ramp | d dual slope | e clocks and prescalers | "
                  "f the PLL | g capture | h input modes | i dithering | "
                  "j output plumbing | k the two vectors    -> z = all of a..k", crlf);
    print(serial, "  No wires: WOA..WOD on PA4..PA7 read back as pin events into TCB "
                  "meters, the TCD's own CMPBCLR event as the cycle counter, PD3/PD4 "
                  "driven from PORT as the two input-event sources. PF0..PF3 (ALT2) are "
                  "claimed only inside test a; PF4/PF5 are the console and are never "
                  "touched.", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

// The four meter TCBs are POLLED, never serviced by a vector - but
// tcb.hpp's meters arm their CAPT interrupt in init(), and an unbound
// vector on this toolchain is a jump to the reset address. These four
// bodies acknowledge and drop, so a capture that slips through between
// an arming and its quiet_meters() costs nothing.
ISR(TCB0_INT_vect) { (void)brio::Tcb<0>::take_flags(); }
ISR(TCB1_INT_vect) { (void)brio::Tcb<1>::take_flags(); }
ISR(TCB2_INT_vect) { (void)brio::Tcb<2>::take_flags(); }
ISR(TCB3_INT_vect) { (void)brio::Tcb<3>::take_flags(); }

ISR(TCD0_OVF_vect) {
    brio::Tcd<0>::ovf();
    ovf_hits = ovf_hits + 1;
}

ISR(TCD0_TRIG_vect) {
    const auto t = brio::Tcd<0>::take_triggers();
    if (t.a) trig_a_hits = trig_a_hits + 1;
    if (t.b) trig_b_hits = trig_b_hits + 1;
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    // The four DEFAULT-route outputs are read back as pin EVENTS, and a
    // released TCD hands them back to PORT as inputs: a pull-up keeps an
    // undriven probe pin at a steady level instead of letting it toggle
    // on noise and feed the meters nonsense.
    PinSet<WoaPin, WobPin, WocPin, WodPin>::configure({.pullup = true});
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_tcd - TCD test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ", hex(SYSCFG.REVID),
          ")", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'z' || c == 'Z') { run_set(all_tests); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
