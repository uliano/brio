// test_avr_pin - the PORT test SUITE for the AVR DA/DB target: pin
// interrupts (senses, flags, the one-vector-per-port pattern), the
// one-store PinConfig, pull-up, input disable, the multi-pin engine
// through PinSet across two ports, the Port<L> mask verbs and slew
// limit. Reference test of avrdx/pin.hpp (docs/avrdx/port.md): keep
// it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console
// on USART2 ALT1 (PF4/PF5) at 460800. No wires: an OUTPUT pin whose
// input buffer stays on senses its own edges, so the test drives the
// pins it watches (PD3, PC6, PC7 - all free on this bench).
//
// Commands: ? | 1 senses | 2 level | 3 invert | 4 flags | 5 pullup
// | 6 input off | 7 multi-pin | 8 port verbs | a all
// Not testable here: INLVL thresholds and the slew rate (analog
// levels / a scope), the fully-async wake (needs standby - queued
// with RUNSTDBY), the buttons PA2..PA5 (a human extra).

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using D3 = Pin<'D', 3>;
using C6 = Pin<'C', 6>;
using C7 = Pin<'C', 7>;

volatile uint16_t d3_irqs = 0, c6_irqs = 0, c7_irqs = 0;

void clear_irqs() {
    cli();
    d3_irqs = c6_irqs = c7_irqs = 0;
    sei();
}

uint8_t passed = 0, failed = 0;
void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

/// n edges >= 4 CLK_PER apart (the partially-async dead-time is 3).
void toggle_d3(uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        D3::toggle();
        delay_us(clock, 2);
    }
}

void quiesce() {
    D3::configure({}); C6::configure({}); C7::configure({});
    D3::input(); C6::input(); C7::input();
    Port<'D'>::clear_flags(0xFF); Port<'C'>::clear_flags(0xFF);
    clear_irqs();
}

// ---- 1: the edge senses -----------------------------------------------------------
void t1_senses() {
    print(serial, "1 senses on PD3 (an output senses its own edges)", crlf);
    quiesce();
    D3::output(); D3::clear();
    struct Case { const char* name; PinSense s; uint16_t expect; };
    const Case cases[] = {
        {"rising: 10 per 10 periods", PinSense::rising, 10},
        {"falling: 10", PinSense::falling, 10},
        {"both: 20", PinSense::both, 20},
        {"none: 0", PinSense::none, 0},
    };
    for (const Case& c : cases) {
        D3::clear();
        D3::sense(c.s);
        D3::clear_flag();                          // an ISC change may fake one (18.3.3)
        clear_irqs();
        toggle_d3(20);                             // 10 full periods
        print(serial, "  ", c.name, " -> ", d3_irqs, crlf);
        verdict(c.name, d3_irqs == c.expect);
    }
    quiesce();
}

// ---- 2: the low level re-fires ----------------------------------------------------
void t2_level() {
    print(serial, "2 level_low on PD3: re-fires while low, quiet while high", crlf);
    quiesce();
    D3::output(); D3::set();                       // high: level_low is quiet
    D3::configure({.sense = PinSense::level_low});
    D3::clear_flag();
    clear_irqs();
    delay_us(clock, 50);
    const uint16_t while_high = d3_irqs;
    D3::clear();                                   // low: the storm
    delay_cycles(480);                             // ~20 us nominal (the ISRs stretch it)
    D3::set();                                     // high again: it must stop
    const uint16_t stormed = d3_irqs;
    delay_us(clock, 50);
    const uint16_t after = d3_irqs;
    print(serial, "  high: ", while_high, ", low window: ", stormed, ", after: ", after, crlf);
    verdict("quiet while high", while_high == 0);
    verdict("re-fires while low (> 5)", stormed > 5);
    verdict("stops when high again", after == stormed);
    quiesce();
}

// ---- 3: INVEN and the senses ------------------------------------------------------
void t3_invert() {
    print(serial, "3 rising sense + INVEN = counts the physical falling edges", crlf);
    quiesce();
    D3::output(); D3::set();                       // physical high = logical low (inverted)
    D3::configure({.invert = true, .sense = PinSense::rising});
    D3::clear_flag();
    clear_irqs();
    toggle_d3(20);                                 // 10 physical falls = 10 logical rises
    print(serial, "  logical rising in 10 periods: ", d3_irqs, crlf);
    verdict("10 counts on the inverted edge", d3_irqs == 10);
    quiesce();
}

// ---- 4: the flags are write-1-to-clear --------------------------------------------
void t4_flags() {
    print(serial, "4 INTFLAGS: one pin's clear must not eat the other's flag", crlf);
    quiesce();
    C6::output(); C7::output(); C6::clear(); C7::clear();
    // senses on, INTERRUPTS observed via flags only: keep the vector
    // quiet by clearing inside the ISR? No - the ISR would take them.
    // So: fire with the flags read in the ISR path disabled - use the
    // counters instead, then re-fire with the ISR counting.
    C6::configure({.sense = PinSense::rising});
    C7::configure({.sense = PinSense::rising});
    cli();                                         // hold the ISR off: raw flag semantics
    C6::set(); C7::set();
    delay_us(clock, 2);
    const uint8_t both = Port<'C'>::flags();
    C6::clear_flag();                              // plain W1C store
    delay_us(clock, 1);                            // the clear lands a cycle later:
                                                   // a back-to-back read still sees
                                                   // the old flags (bench finding)
    const uint8_t after = Port<'C'>::flags();
    Port<'C'>::clear_flags(0xFF);
    sei();
    print(serial, "  flags: ", hex(both), " -> after PC6 clear: ", hex(after), crlf);
    verdict("both flags set", (both & 0xC0) == 0xC0);
    verdict("clearing PC6 leaves PC7 (no RMW eating W1C)", (after & 0xC0) == 0x80);
    quiesce();
}

// ---- 5: pull-up -------------------------------------------------------------------
void t5_pullup() {
    print(serial, "5 pull-up on an input reads high", crlf);
    quiesce();
    D3::input();
    D3::configure({.pullup = true});
    delay_us(clock, 5);
    verdict("PD3 with pull-up reads 1", D3::read());
    D3::configure({});
    quiesce();
}

// ---- 6: input disable -------------------------------------------------------------
void t6_input_off() {
    print(serial, "6 input_disable freezes IN (the pad keeps its analog life)", crlf);
    quiesce();
    D3::output(); D3::set();
    delay_us(clock, 2);
    verdict("IN follows OUT while the buffer is on", D3::read());
    D3::sense(PinSense::input_disable);
    D3::clear();                                   // the PAD goes low ...
    delay_us(clock, 5);
    const bool frozen = D3::read();
    D3::sense(PinSense::none);                     // buffer back on
    delay_us(clock, 2);
    const bool live = D3::read();
    print(serial, "  IN with buffer off: ", frozen ? 1 : 0, ", back on: ", live ? 1 : 0, crlf);
    verdict("IN frozen at the last value with the buffer off", frozen);
    verdict("IN live again with the buffer on", !live);
    quiesce();
}

// ---- 7: the multi-pin engine across two ports -------------------------------------
void t7_multipin() {
    print(serial, "7 PinSet{PD3, PC6, PC7}.configure: one setting, two ports, one shot", crlf);
    quiesce();
    using Set = PinSet<D3, C6, C7>;
    static_assert(Set::port_mask<'D'>() == 0x08 && Set::port_mask<'C'>() == 0xC0);
    const PinConfig cfg{.pullup = true, .sense = PinSense::both};
    Set::configure(cfg);
    const uint8_t want = pin_ctrl_byte(cfg);
    verdict("PD3 PINCTRL matches", D3::pinctrl() == want);
    verdict("PC6 PINCTRL matches", C6::pinctrl() == want);
    verdict("PC7 PINCTRL matches", C7::pinctrl() == want);
    // and they are alive: an edge on each port counts
    D3::output(); C6::output();
    Port<'D'>::clear_flags(0xFF); Port<'C'>::clear_flags(0xFF);
    clear_irqs();
    D3::toggle(); C6::toggle();
    delay_us(clock, 5);
    print(serial, "  after one toggle each: PD3 ", d3_irqs, ", PC6 ", c6_irqs, crlf);
    verdict("both ports interrupt", d3_irqs == 1 && c6_irqs == 1);
    quiesce();
}

// ---- 8: the Port verbs ------------------------------------------------------------
void t8_port() {
    print(serial, "8 Port<'C'> mask verbs and the slew limit", crlf);
    quiesce();
    using PC = Port<'C'>;
    PC::dir_set(0xC0);
    PC::out_clear(0xC0);
    PC::out_set(0x40);                             // PC6 high, PC7 low
    delay_us(clock, 2);
    verdict("in() reads the mask (PC6 set)", (PC::in() & 0xC0) == 0x40);
    PC::out_toggle(0xC0);
    delay_us(clock, 2);
    verdict("out_toggle flips both", (PC::in() & 0xC0) == 0x80);
    PC::slew_limit(true);
    verdict("slew limit set", PC::slew_limit());
    PC::slew_limit(false);
    verdict("slew limit cleared", !PC::slew_limit());
    PC::dir_clear(0xC0);
    quiesce();
}

uint8_t run_all_pass = 0;
using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_senses}, {'2', t2_level}, {'3', t3_invert}, {'4', t4_flags},
    {'5', t5_pullup}, {'6', t6_input_off}, {'7', t7_multipin}, {'8', t8_port},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_pin: 1 senses | 2 level | 3 invert | 4 flags | 5 pullup | "
                  "6 input off | 7 multi-pin | 8 port verbs | a all", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(PORTD_PORT_vect) {
    const uint8_t f = brio::Port<'D'>::take_flags();
    if (f & 0x08) ++d3_irqs;
}
ISR(PORTC_PORT_vect) {
    const uint8_t f = brio::Port<'C'>::take_flags();
    if (f & 0x40) ++c6_irqs;
    if (f & 0x80) ++c7_irqs;
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_pin - PORT test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF",
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
