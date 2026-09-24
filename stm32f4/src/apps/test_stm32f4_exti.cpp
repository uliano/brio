// test_stm32f4_exti - the reference bench suite for the STM32F4's EXTI
// and the SYSCFG block its pin multiplexer lives in: the twenty-three
// lines, their triggers, the one pending register, the two masks, the
// software trigger, the multiplexer and the vectors - stm32f4/exti.hpp and
// stm32f4/syscfg.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, and nothing to press. Two stimuli, both inside the
// chip: the SOFTWARE TRIGGER (EXTI_SWIER, which needs no pad at all and
// reaches the peripheral wake-up lines as well as the pin ones), and A PAD
// THE PROGRAM DRIVES ITSELF - the board's own LED pin, because a GPIO in
// output mode still feeds its input buffer, so the EXTI sees the edges the
// program makes. The board's user button is configured and reported and
// nothing is expected of it; letter p, which is not part of z, waits for a
// press for whoever is at the desk.
//
// What is exercised, letter by letter:
//   a  SYSCFG: the gate that is closed out of reset, the memory map, the
//      compensation cell, the PHY selector where there is one
//   b  the EXTI's reset state and the multiplexer: every register at zero,
//      every line pointed at port A, the port codes, the refusals
//   c  the software trigger: the flag it raises with no pad, the bit that
//      stands until the flag is cleared, and what it does with the
//      interrupt masked
//   d  the latency from the SWIER store to the handler, in cycles
//   e  a pad the program drives: rising, falling, both, and analog mode
//      as the one state that blinds a line
//   f  the two masks: the flag that stands with the interrupt masked, and
//      the NVIC line as the second gate
//   g  an EXTI EVENT returning the core from WFE with no handler
//   h  the grouped vectors: two lines of one vector served in turn, and a
//      handler that leaves another vector's flag alone
//   i  one pin number is one line: the second claim refused, and stolen
//      on purpose
//   j  the lines above 15: each to its own vector, and the ones this part
//      has not got refusing every verb
//   k  the pending bit: rc_w1, and why a handler clears it FIRST
//   l  the board's button: configured, reported, and quiet
//   p  (not in z) wait for a press of the button
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/syscfg.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx) || defined(STM32F469xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// The two pads this suite touches, per board. The LED is the STIMULUS -
// an output the program drives, whose edges its own line sees - and the
// button is READ ONLY: its line is armed and reported, never expected to
// fire.
#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
using Button = Pin<'A', 0>;
constexpr bool button_press_is_rising = true;    // B1 pulls the pad UP when pressed
constexpr PinPull button_pull = PinPull::down;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F469xx)
using Led = Pin<'G', 6>;                         // LD1, lit when low - an edge is an edge
using Button = Pin<'A', 0>;
constexpr bool button_press_is_rising = true;    // the blue B2 reads 1 when pressed (UM1932 4.15)
constexpr PinPull button_pull = PinPull::down;
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
using Button = Pin<'A', 0>;
constexpr bool button_press_is_rising = false;   // the key pulls the pad to ground
constexpr PinPull button_pull = PinPull::up;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
using Led = Pin<'A', 5>;
using Button = Pin<'C', 13>;
constexpr bool button_press_is_rising = false;   // B1 pulls the pad to ground
constexpr PinPull button_pull = PinPull::none;   // the board's own pull-up holds it high
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using LedInt = ExtInt<Led>;
using ButtonInt = ExtInt<Button>;
static_assert(exti_lines_distinct<LedInt, ButtonInt>(),
              "the stimulus pad and the button must not share a line");

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// The lines this suite drives by software, none of them a pad's: line 1
// has a vector of its own (the latency and the pending-bit letters), lines
// 5 and 6 share EXTI9_5 and line 10 belongs to EXTI15_10 (the grouped
// vectors), and line 0 is the conflict pair's (PA0 against PB0).
constexpr uint8_t sw_line = 1;
constexpr uint8_t group_a = 5, group_b = 6;
constexpr uint8_t group_c = 10;

// ---- what the registers held before this program touched them ---------------
struct BootState {
    bool syscfg_gate = false;
    uint32_t imr = 0, emr = 0, rtsr = 0, ftsr = 0, swier = 0, pr = 0;
    uint32_t exticr[4] = {};
};
BootState boot;

// ---- handler bookkeeping -----------------------------------------------------
volatile uint32_t served_lines = 0;      ///< OR of every mask a handler saw
volatile uint16_t entries[24] = {};      ///< per-line handler entries
volatile uint16_t vector_entries = 0;    ///< handler calls, whichever vector
volatile uint32_t handler_val = 0;       ///< SysTick VAL sampled on entry
volatile uint16_t leftover_seen = 0;     ///< a foreign line's flag seen by a handler
volatile uint8_t clear_late = 0;         ///< the pending-bit letter's wrong handler
volatile uint16_t late_entries = 0;

[[gnu::always_inline]] inline void serve(IRQn_Type v) {
    handler_val = SysTick->VAL;
    const uint32_t fired = Exti::isr(Exti::vector_lines(v));
    served_lines |= fired;
    vector_entries = vector_entries + 1u;
    for (uint8_t l = 0; l < 24u; ++l) {
        if (Exti::served(fired, l)) {
            entries[l] = entries[l] + 1u;
        }
    }
    // What the vector could see but must not touch: another vector's flag.
    if ((Exti::pending() & ~Exti::vector_lines(v)) != 0u) {
        leftover_seen = leftover_seen + 1u;
    }
}

void reset_counters() {
    P::CriticalSection cs;
    served_lines = 0;
    vector_entries = 0;
    leftover_seen = 0;
    for (uint8_t l = 0; l < 24u; ++l) {
        entries[l] = 0;
    }
}

/// Every line this suite ever arms, back to reset, and the NVIC lines with
/// them - so no letter inherits another's state.
void quiet_everything() {
    for (uint8_t l = 0; l < 24u; ++l) {
        if (Exti::implemented(l)) {
            Nvic::disable(Exti::irq(l));
            (void)Exti::release(l);
            // The EXTI's flag is not the whole request: one that reached
            // the NVIC while its line was disabled is latched there and
            // outlives it (letter f measures exactly that).
            Nvic::clear_pending(Exti::irq(l));
        }
    }
    reset_counters();
}

/// Let the console's own transmitter fall silent: its interrupt would
/// otherwise wake every WFE and WFI below.
void console_drain() {
    const uint32_t t0 = Ticker::ticks();
    while (!Serial::tx_idle() && Ticker::ticks() - t0 < 200u) {
    }
}

// ---- a ruler ------------------------------------------------------------------
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

// A pulse the edge detector is guaranteed to see. Two adjacent BSRR stores
// are one AHB cycle apart and the detector samples at the APB2 clock: on
// one part of the family that pulse goes unseen, on another it is caught
// (letter f measures it). Every step whose proof depends on an edge being
// SEEN uses this one microsecond instead.
void pulse() {
    Led::set();
    (void)delay_us(clock, 1);
    Led::clear();
}

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// A cycle count that spans ticks: it wraps every few seconds, which is
/// longer than anything measured here.
uint32_t cycles_now() {
    const uint32_t period = systick_period();
    uint32_t t0 = 0, v = 0, t1 = 0;
    do {
        t0 = Ticker::ticks();
        v = SysTick->VAL;
        t1 = Ticker::ticks();
    } while (t0 != t1);
    return t0 * period + (period - 1u - v);
}

uint32_t cycles_to_us(uint32_t c) { return c / cycles_per_us; }

/// The distance between two VAL samples of one SysTick period.
uint32_t val_delta(uint32_t v0, uint32_t v1) {
    const uint32_t period = systick_period();
    return (v0 >= v1) ? (v0 - v1) : (v0 + period - v1);
}

// =============================================================================
// a - SYSCFG: the gate and the block
// =============================================================================
void ta_syscfg() {
    print(serial, "  SYSCFGEN at boot ", boot.syscfg_gate ? "OPEN" : "closed", ", memory map ",
          static_cast<uint8_t>(Syscfg::memory_map()), ", PHY select ",
          Syscfg::has_phy_select() ? "present" : "absent", crlf);
    bench.verdict("RCC_APB2ENR.SYSCFGEN is CLOSED out of reset - nothing but this "
                  "driver opens it",
                  !boot.syscfg_gate);
    bench.verdict("the first verb of the block opened it", Syscfg::clock());

    // Every verb opens the gate, not only the configuring ones: close it
    // and read the memory map.
    Syscfg::clock(false);
    const bool shut = !Syscfg::clock();
    const MemoryMap map = Syscfg::memory_map();
    bench.verdict("closing the gate is one RCC bit and it reads back shut", shut);
    bench.verdict("and a READ of the block opens it again (a verb never speaks to a "
                  "dead peripheral)",
                  Syscfg::clock());
    bench.verdict("the memory map at address 0 is the main flash the BOOT pins chose",
                  map == MemoryMap::main_flash);

    // The write half of the same rule: with the gate shut, a multiplexer
    // write still lands, because the verb opens the gate before storing.
    (void)Exti::steal(group_a, 'A');
    Syscfg::clock(false);
    const bool wrote = Exti::steal(group_a, 'B');
    const char after = Exti::selected(group_a);
    (void)Exti::steal(group_a, 'A');
    bench.verdict("a multiplexer write from a CLOSED gate lands, because the verb "
                  "opens the gate first",
                  wrote && after == 'B');

    // The compensation cell (9.1): off by default, and ready once powered.
    const bool cell_off = !Syscfg::compensation_cell();
    Syscfg::compensation_cell(true);
    uint16_t spins = 0;
    while (!Syscfg::compensation_ready() && spins < 1000u) {
        ++spins;
    }
    const bool ready = Syscfg::compensation_ready();
    Syscfg::compensation_cell(false);
    print(serial, "  the compensation cell reported ready after ", spins, " reads", crlf);
    bench.verdict("the I/O compensation cell is powered down by default", cell_off);
    bench.verdict("enabling it raises CMPCR.READY", ready);
    bench.verdict("and powering it down again is one bit", !Syscfg::compensation_cell());

    // The Ethernet PHY selector: a part fact, and the verb answers it.
    bench.verdict("the PHY selector answers exactly where the header declares it",
                  Syscfg::phy_rmii(false) == Syscfg::has_phy_select());
}

// =============================================================================
// b - the EXTI's reset state and the multiplexer
// =============================================================================
void tb_reset_and_mux() {
    print(serial, "  at boot IMR=", hex(boot.imr), " EMR=", hex(boot.emr), " RTSR=",
          hex(boot.rtsr), " FTSR=", hex(boot.ftsr), " SWIER=", hex(boot.swier), " PR=",
          hex(boot.pr), crlf);
    print(serial, "  EXTICR ", hex(boot.exticr[0]), " ", hex(boot.exticr[1]), " ",
          hex(boot.exticr[2]), " ", hex(boot.exticr[3]), ", implemented lines ",
          hex(Exti::implemented_mask), crlf);
    bench.verdict("every EXTI register comes up at zero - no line is unmasked, "
                  "triggered or pending behind the program's back",
                  (boot.imr | boot.emr | boot.rtsr | boot.ftsr | boot.swier | boot.pr) == 0u);
    bench.verdict("and every EXTICR field comes up at zero, which is port A",
                  (boot.exticr[0] | boot.exticr[1] | boot.exticr[2] | boot.exticr[3]) == 0u);

    // The multiplexer: a code per port, the letter's distance from 'A'.
    bench.verdict("the port codes are the contiguous A..K encoding",
                  exti_port_code('A') == 0 && exti_port_code('B') == 1 &&
                      exti_port_code('C') == 2 && exti_port_code('H') == 7);
    bench.verdict("a port this device has not got has no code", exti_port_code('Z') == 0xFFu);

    quiet_everything();
    const bool to_a = Exti::select(group_a, 'A') && Exti::selected(group_a) == 'A';
    const bool to_b = Exti::select(group_a, 'B') && Exti::selected(group_a) == 'B';
    const bool to_c = Exti::select(group_a, 'C') && Exti::selected(group_a) == 'C';
    bench.verdict("a free line follows the port it is pointed at", to_a && to_b && to_c);
    bench.verdict("a port with no code is refused and the line does not move",
                  !Exti::select(group_a, 'Z') && Exti::selected(group_a) == 'C');
    bench.verdict("a line above 15 has no multiplexer at all",
                  !Exti::select(16, 'A') && Exti::selected(16) == 0);
    (void)Exti::steal(group_a, 'A');

    // The reserve's vector map, against the vectors this image binds.
    bench.verdict("lines 0..4 have a vector each and 5..9 share one",
                  Exti::irq(0) == EXTI0_IRQn && Exti::irq(4) == EXTI4_IRQn &&
                      Exti::irq(5) == EXTI9_5_IRQn && Exti::irq(9) == EXTI9_5_IRQn);
    bench.verdict("lines 10..15 share the other",
                  Exti::irq(10) == EXTI15_10_IRQn && Exti::irq(15) == EXTI15_10_IRQn);
    bench.verdict("and each vector's mask is exactly its own lines",
                  Exti::vector_lines(EXTI9_5_IRQn) == 0x03E0u &&
                      Exti::vector_lines(EXTI15_10_IRQn) == 0xFC00u);
}

// =============================================================================
// c - the software trigger
// =============================================================================
void tc_software_trigger() {
    quiet_everything();

    // A line with NO trigger edge selected at all, armed in IMR, NVIC shut:
    // the flag is the whole observation.
    (void)Exti::interrupt(sw_line, true);
    const bool no_sense = Exti::sense(sw_line) == ExtiSense::none;
    (void)Exti::trigger(sw_line);
    const bool flagged = Exti::pending(sw_line);
    const bool bit_stands = Exti::triggered(sw_line);
    bench.verdict("a software trigger raises the flag on a line with no edge "
                  "selected and no pad behind it",
                  no_sense && flagged);
    bench.verdict("and SWIER's bit STANDS - it is not self-clearing on this family",
                  bit_stands);

    // A second trigger before the first is acknowledged does nothing: the
    // bit is already 1 and there is no 0 -> 1 transition to make.
    (void)Exti::trigger(sw_line);
    (void)Exti::clear(sw_line);
    const bool swier_cleared = !Exti::triggered(sw_line);
    const bool pr_cleared = !Exti::pending(sw_line);
    bench.verdict("clearing the pending bit is what clears SWIER", swier_cleared && pr_cleared);
    bench.verdict("and the second trigger of the pair left nothing behind (one "
                  "acknowledgement cleared both)",
                  !Exti::pending(sw_line));

    // ... and once acknowledged the line triggers again.
    (void)Exti::trigger(sw_line);
    bench.verdict("an acknowledged line triggers again", Exti::pending(sw_line));
    (void)Exti::clear(sw_line);

    // With the interrupt MASKED. 12.3.5 makes the pending bit conditional
    // on IMR for a SOFTWARE trigger, which is not what it says for an edge.
    (void)Exti::interrupt(sw_line, false);
    (void)Exti::trigger(sw_line);
    const bool masked_flag = Exti::pending(sw_line);
    const bool masked_swier = Exti::triggered(sw_line);
    print(serial, "  with IMR clear a software trigger left PR ", masked_flag ? "SET" : "clear",
          " and SWIER ", masked_swier ? "set" : "clear", crlf);
    bench.verdict("a software trigger with the interrupt masked raises no flag "
                  "(12.3.5's own condition)",
                  !masked_flag);
    bench.verdict("and the SWIER bit it wrote stands, waiting for a mask that never "
                  "came - so the line must be cleared before it is armed",
                  masked_swier);
    (void)Exti::clear(sw_line);
    quiet_everything();

    // The whole path, through the NVIC this time.
    (void)Exti::interrupt(sw_line, true);
    Nvic::enable(Exti::irq(sw_line));
    reset_counters();
    for (uint8_t k = 0; k < 5u; ++k) {
        (void)Exti::trigger(sw_line);
        (void)delay_us(clock, 20);
    }
    print(serial, "  five software triggers reached the handler ", vector_entries, " time(s)",
          crlf);
    bench.verdict("every software trigger reached the real vector and the handler's "
                  "clear let the next one through",
                  vector_entries == 5u && entries[sw_line] == 5u);
    quiet_everything();
}

// =============================================================================
// d - the latency from the store to the handler
// =============================================================================
void td_latency() {
    quiet_everything();
    (void)Exti::interrupt(sw_line, true);
    Nvic::enable(Exti::irq(sw_line));

    uint32_t worst = 0, best = 0xFFFFFFFFu, total = 0;
    const uint8_t rounds = 16;
    for (uint8_t k = 0; k < rounds; ++k) {
        reset_counters();
        const uint32_t v0 = SysTick->VAL;
        (void)Exti::trigger(sw_line);
        while (vector_entries == 0u) {
        }
        const uint32_t took = val_delta(v0, handler_val);
        total += took;
        if (took > worst) {
            worst = took;
        }
        if (took < best) {
            best = took;
        }
        (void)delay_us(clock, 20);
    }
    const uint32_t mean = total / rounds;
    print(serial, "  SWIER store to handler: ", best, "..", worst, " cycles, mean ", mean,
          " (", mean * 1000u / cycles_per_us, " ns at ", SysClock::hz / 1'000'000u, " MHz)", crlf);
    bench.verdict("the software trigger reaches its handler in under 200 cycles, "
                  "every round",
                  worst < 200u && best > 0u);
    quiet_everything();
}

// =============================================================================
// e - a pad the program drives
// =============================================================================
void te_pad() {
    quiet_everything();
    Led::output(false);
    (void)LedInt::select();
    bench.verdict("the line follows the LED pad's own port", LedInt::selected());
    (void)LedInt::arm(true);   // the flag, not the vector: the NVIC stays shut

    // Eight low-to-high and eight high-to-low transitions, counted through
    // the pending flag on each of the four senses.
    struct Leg {
        ExtiSense sense;
        uint8_t up = 0, down = 0;
    };
    Leg legs[4] = {{ExtiSense::none, 0, 0},
                   {ExtiSense::rising, 0, 0},
                   {ExtiSense::falling, 0, 0},
                   {ExtiSense::both, 0, 0}};
    for (Leg& leg : legs) {
        Led::clear();
        (void)LedInt::configure(leg.sense);
        (void)LedInt::clear();
        for (uint8_t k = 0; k < 8u; ++k) {
            Led::set();
            if (LedInt::pending()) {
                ++leg.up;
                (void)LedInt::clear();
            }
            Led::clear();
            if (LedInt::pending()) {
                ++leg.down;
                (void)LedInt::clear();
            }
        }
    }
    for (const Leg& leg : legs) {
        print(serial, "  sense ", static_cast<uint8_t>(leg.sense), ": ", leg.up, " up, ",
              leg.down, " down", crlf);
    }
    bench.verdict("A PAD IN OUTPUT MODE FEEDS ITS OWN LINE: eight rising edges the "
                  "program made, eight flags",
                  legs[1].up == 8u && legs[1].down == 0u);
    bench.verdict("a falling sense sees the other eight and no more",
                  legs[2].up == 0u && legs[2].down == 8u);
    bench.verdict("both edges is both bits, sixteen flags",
                  legs[3].up == 8u && legs[3].down == 8u);
    bench.verdict("and a line with no trigger selected sees nothing",
                  legs[0].up == 0u && legs[0].down == 0u);

    // Analog mode: the input buffer is off, so the pad disappears from its
    // line even though the program is still storing to it.
    (void)LedInt::configure(ExtiSense::both);
    Led::analog();
    (void)LedInt::clear();
    uint8_t blind = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        pulse();
        if (LedInt::pending()) {
            ++blind;
            (void)LedInt::clear();
        }
    }
    bench.verdict("ANALOG MODE IS THE ONE STATE THAT BLINDS A LINE: sixteen stores, "
                  "no edge",
                  blind == 0u);

    Led::output(false);
    quiet_everything();
}

// =============================================================================
// f - the two masks and the NVIC
// =============================================================================
void tf_masks() {
    quiet_everything();
    Led::output(false);
    (void)LedInt::select();
    (void)LedInt::configure(ExtiSense::rising);

    // Does the flag appear at all with the interrupt masked? The chapter
    // never says; the block diagram (figure 41) puts the interrupt mask
    // between the edge detector and the pending register.
    (void)LedInt::arm(false);
    (void)LedInt::clear();
    for (uint8_t k = 0; k < 8u; ++k) {
        pulse();
    }
    const bool flag_while_masked = LedInt::pending();
    (void)LedInt::arm(true);
    const bool resurrected = LedInt::pending();
    print(serial, "  with IMR clear, eight edges left the flag ",
          flag_while_masked ? "SET" : "clear", "; unmasking after them left it ",
          resurrected ? "SET" : "clear", crlf);
    bench.verdict("THE PENDING BIT EXISTS ONLY FOR AN UNMASKED INTERRUPT: eight "
                  "edges under the mask leave no trace, and unmasking does not "
                  "resurrect them",
                  !flag_while_masked && !resurrected);

    // So a line is WATCHED by arming the EXTI's mask and leaving the NVIC
    // line disabled: the flag stands and no handler runs.
    Nvic::disable(LedInt::irq());
    Nvic::clear_pending(LedInt::irq());
    reset_counters();
    (void)LedInt::clear();
    pulse();
    const bool watched = LedInt::pending() && vector_entries == 0u;
    const bool nvic_latched = Nvic::pending(LedInt::irq());
    bench.verdict("a line is WATCHED by arming the EXTI mask and leaving the NVIC "
                  "line shut: the flag stands, no handler runs",
                  watched);

    // ... but the request reached the NVIC all the same, and LATCHED there.
    // Clearing the EXTI's flag does not clear the NVIC's own pending bit.
    (void)LedInt::clear();
    const bool still_latched = Nvic::pending(LedInt::irq());
    Nvic::enable(LedInt::irq());
    (void)delay_us(clock, 50);
    const uint16_t after_enable = vector_entries;
    print(serial, "  the NVIC latched the watched line's request (", nvic_latched ? "yes" : "no",
          "), and it survived the flag's clear (", still_latched ? "yes" : "no",
          "): ", after_enable, " call(s) on enabling the line", crlf);
    bench.verdict("THE NVIC LATCHES THE REQUEST TOO, and clearing the EXTI's flag "
                  "does not clear it: opening the vector delivers one call for an "
                  "edge already acknowledged - Nvic::clear_pending is the other "
                  "half of a teardown",
                  nvic_latched && still_latched && after_enable == 1u);
    Nvic::disable(LedInt::irq());
    Nvic::clear_pending(LedInt::irq());
    reset_counters();
    (void)LedInt::clear();
    pulse();
    (void)LedInt::clear();
    Nvic::clear_pending(LedInt::irq());
    Nvic::enable(LedInt::irq());
    (void)delay_us(clock, 50);
    bench.verdict("and with both halves cleared, opening the vector delivers "
                  "nothing", vector_entries == 0u);

    // How narrow a pulse does the detector see? Two adjacent stores are the
    // narrowest the program can make; whether they register is a fact of
    // the part and its bus timing, reported and not judged.
    Nvic::disable(LedInt::irq());
    Nvic::clear_pending(LedInt::irq());
    (void)LedInt::clear();
    uint8_t narrow = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        Led::set();
        Led::clear();
        if (LedInt::pending()) {
            ++narrow;
            (void)LedInt::clear();
        }
    }
    print(serial, "  a pulse of two adjacent stores (one AHB cycle at ", SysClock::hz / 1'000'000u,
          " MHz) is seen ", narrow, " times in 8; the suite pulses one microsecond", crlf);
    Nvic::clear_pending(LedInt::irq());
    Nvic::enable(LedInt::irq());

    // With everything open, every edge is a handler call.
    reset_counters();
    for (uint8_t k = 0; k < 8u; ++k) {
        Led::set();
        (void)delay_us(clock, 10);
        Led::clear();
        (void)delay_us(clock, 10);
    }
    print(serial, "  eight rising edges, ", vector_entries, " handler calls", crlf);
    bench.verdict("eight edges through the whole path are eight handler calls",
                  vector_entries == 8u && entries[LedInt::line] == 8u);

    quiet_everything();
    Led::output(false);
}

// =============================================================================
// g - an EXTI event out of WFE
// =============================================================================
void tg_event() {
    quiet_everything();
    Led::output(false);
    (void)LedInt::select();
    (void)LedInt::configure(ExtiSense::rising);

    // The CPU event: no NVIC line, no handler, no flag to clear (12.2.3).
    const bool event_only = LedInt::event(true) && LedInt::event() && !LedInt::armed();

    // THE MEASUREMENT IS A TIME, and two traps stand in its way. An
    // EXCEPTION ENTRY sets this core's event register just as SEV and the
    // EXTI do, so a loop that ends by returning from SysTick_Handler leaves
    // it SET and the next WFE returns whatever the EXTI did; and every byte
    // of a console line is an interrupt, which returns a WFE too. So: drain
    // the console, align to the tick, clear the event register with SEV +
    // WFE, make the edge, sleep - and print afterwards.
    console_drain();
    uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    __SEV();
    __WFE();                    // consumes the register's own set bit
    Led::set();                 // the edge
    const uint32_t c0 = cycles_now();
    __WFE();
    const uint32_t event_us = cycles_to_us(cycles_now() - c0);
    const bool event_flag = LedInt::pending();

    // The control: the same edge with the event masked, so only the tick
    // can end the wait.
    (void)LedInt::event(false);
    Led::clear();
    t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    __SEV();
    __WFE();
    Led::set();
    const uint32_t c1 = cycles_now();
    __WFE();
    const uint32_t control_us = cycles_to_us(cycles_now() - c1);

    print(serial, "  WFE after an edge: ", event_us, " us with the CPU event unmasked, ",
          control_us, " us with it masked (the tick)", crlf);
    bench.verdict("EMR is set and IMR is not - an event line has no NVIC side",
                  event_only);
    bench.verdict("AN EXTI EVENT RETURNS THE CORE FROM WFE with no handler and no "
                  "vector", event_us < 100u);
    bench.verdict("with the event masked the same edge does nothing and the WFE "
                  "waits for the next tick", control_us > 100u);
    bench.verdict("and a CPU event leaves NO pending bit to acknowledge", !event_flag);

    quiet_everything();
    Led::output(false);
}

// =============================================================================
// h - the grouped vectors
// =============================================================================
void th_grouped() {
    quiet_everything();

    // Two lines of one vector, each pointed at a different port - the
    // multiplexer's choice, even though the stimulus is the software
    // trigger and no pad is involved.
    (void)Exti::select(group_a, 'A');
    (void)Exti::select(group_b, 'B');
    (void)Exti::interrupt(group_a, true);
    (void)Exti::interrupt(group_b, true);
    Nvic::enable(EXTI9_5_IRQn);
    bench.verdict("two lines of the same vector, pointed at two different ports",
                  Exti::selected(group_a) == 'A' && Exti::selected(group_b) == 'B' &&
                      Exti::irq(group_a) == Exti::irq(group_b));

    reset_counters();
    for (uint8_t k = 0; k < 4u; ++k) {
        (void)Exti::trigger(group_a);
        (void)delay_us(clock, 20);
        (void)Exti::trigger(group_b);
        (void)delay_us(clock, 20);
    }
    print(serial, "  EXTI9_5: ", vector_entries, " calls, line ", group_a, " x",
          entries[group_a], ", line ", group_b, " x", entries[group_b], crlf);
    bench.verdict("A GROUPED VECTOR IS A DISPATCHER: eight triggers on two lines, "
                  "eight calls, four each",
                  vector_entries == 8u && entries[group_a] == 4u && entries[group_b] == 4u);

    // Both at once: one call, two lines in the mask.
    reset_counters();
    {
        P::CriticalSection cs;
        (void)Exti::trigger(group_a);
        (void)Exti::trigger(group_b);
    }
    (void)delay_us(clock, 50);
    print(serial, "  two lines raised together: ", vector_entries, " call(s), mask ",
          hex(served_lines), crlf);
    bench.verdict("two lines raised inside one critical section are ONE call with "
                  "both lines in the mask",
                  vector_entries == 1u && entries[group_a] == 1u && entries[group_b] == 1u);

    // ONE PENDING REGISTER, SEPARATE HANDLERS. Leave a line of the OTHER
    // group standing with its vector shut, and make this one run: its body
    // must see the foreign flag and leave it exactly where it found it.
    (void)Exti::interrupt(group_c, true);
    Nvic::disable(EXTI15_10_IRQn);
    (void)Exti::trigger(group_c);
    const bool foreign_set = Exti::pending(group_c);
    reset_counters();
    (void)Exti::trigger(group_a);
    (void)delay_us(clock, 50);
    print(serial, "  with line ", group_c, " standing, EXTI9_5 ran ", vector_entries,
          " time(s) and saw a foreign flag ", leftover_seen, " time(s)", crlf);
    bench.verdict("a line of another vector stands untouched while this one runs",
                  foreign_set && leftover_seen == 1u && Exti::pending(group_c));
    bench.verdict("and the handler served only its own line",
                  vector_entries == 1u && entries[group_a] == 1u && entries[group_c] == 0u);

    // And the other vector, opened, takes it.
    reset_counters();
    Nvic::enable(EXTI15_10_IRQn);
    (void)delay_us(clock, 50);
    bench.verdict("the other vector serves it as soon as its NVIC line opens",
                  vector_entries == 1u && entries[group_c] == 1u);

    quiet_everything();
}

// =============================================================================
// i - one pin number is one line
// =============================================================================
void ti_one_line_per_number() {
    quiet_everything();

    using PadA = ExtInt<Pin<'A', 0>>;
    using PadB = ExtInt<Pin<'B', 0>>;
    static_assert(PadA::line == PadB::line, "the whole point: one line, two pads");
    static_assert(!exti_lines_distinct<PadA, PadB>(),
                  "and the application-level check says so at compile time");

    // Nobody is using the line: the second claim is free.
    bench.verdict("a line nobody uses changes hands freely",
                  PadA::select() && PadA::selected() && PadB::select() && PadB::selected());

    // Now port A takes it and arms it - and the claim is refused.
    (void)PadA::select();
    (void)PadA::configure(ExtiSense::rising);
    (void)PadA::arm(true);
    const bool in_use = Exti::in_use(PadA::line);
    const bool refused = !PadB::select();
    const bool kept = PadA::selected();
    bench.verdict("a line IN USE (a sense, an interrupt or an event) is not handed "
                  "to another port behind its owner's back",
                  in_use && refused && kept);

    // The deliberate override says what it is doing in its name.
    const bool stolen = PadB::steal() && PadB::selected();
    bench.verdict("steal() takes it anyway - the verb an application writes when it "
                  "means to",
                  stolen);
    bench.verdict("and the line's sense and mask did not move with the port: the "
                  "multiplexer is the only thing that changed",
                  Exti::sense(PadA::line) == ExtiSense::rising && Exti::interrupt(PadA::line));

    // Released, it is free again.
    (void)Exti::release(PadA::line);
    bench.verdict("a released line is free for the next claim",
                  !Exti::in_use(PadA::line) && PadA::select());

    quiet_everything();
}

// =============================================================================
// j - the lines above 15
// =============================================================================
void tj_peripheral_lines() {
    quiet_everything();
    print(serial, "  implemented lines ", hex(Exti::implemented_mask), ": 16 PVD, 17 RTC "
          "alarm, 18 OTG FS wake, 19 Ethernet wake, 20 OTG HS wake, 21 RTC tamper, 22 "
          "RTC wake", crlf);

    // Every line this part has above 15 whose vector this image binds,
    // software-triggered to ITS OWN vector: the wake-up lines are ordinary
    // edge-detected lines here, with a trigger, a flag and a SWIER bit.
    static const uint8_t own_vector[] = {16, 17, 18, 21, 22};
    uint8_t reached = 0, expected = 0;
    for (uint8_t l : own_vector) {
        if (!Exti::implemented(l)) {
            continue;
        }
        ++expected;
        (void)Exti::interrupt(l, true);
        Nvic::enable(Exti::irq(l));
        reset_counters();
        (void)Exti::trigger(l);
        (void)delay_us(clock, 50);
        if (vector_entries == 1u && entries[l] == 1u) {
            ++reached;
        }
        Nvic::disable(Exti::irq(l));
        (void)Exti::release(l);
    }
    print(serial, "  ", reached, " of ", expected, " peripheral lines reached their own "
          "vector", crlf);
    bench.verdict("each line above 15 that this part has reaches a vector of its "
                  "own, and a software trigger is enough to prove it",
                  expected >= 4u && reached == expected);

    // Line 20 is the USB OTG HS wake-up: present on the parts with that
    // controller, and this image binds no handler for it - so it is judged
    // by its flag, with the NVIC shut.
    if (Exti::implemented(20)) {
        (void)Exti::interrupt(20, true);
        (void)Exti::trigger(20);
        bench.verdict("the USB OTG HS wake-up line flags like any other", Exti::pending(20));
        (void)Exti::release(20);
    } else {
        bench.verdict("this part has no USB OTG HS, so line 20 refuses every verb",
                      !Exti::interrupt(20, true) && !Exti::trigger(20) && !Exti::pending(20));
    }

    // And a line this part has not got answers false to everything rather
    // than writing a bit into a reserved field.
    const uint8_t absent = Exti::implemented(19) ? 24u : 19u;
    bench.verdict("a line this device does not implement refuses the sense, the "
                  "trigger, the mask and the release",
                  !Exti::sense(absent, ExtiSense::rising) && !Exti::trigger(absent) &&
                      !Exti::interrupt(absent, true) && !Exti::release(absent) &&
                      Exti::irq(absent) == NonMaskableInt_IRQn);

    quiet_everything();
}

// =============================================================================
// k - the pending bit, and why a handler clears it first
// =============================================================================
void tk_pending() {
    quiet_everything();
    (void)Exti::interrupt(sw_line, true);

    // rc_w1: a zero leaves the bit alone, a one clears it.
    (void)Exti::trigger(sw_line);
    Exti::clear_lines(0u);
    const bool zero_kept = Exti::pending(sw_line);
    Exti::clear_lines(~(1UL << sw_line) & Exti::implemented_mask);
    const bool others_kept = Exti::pending(sw_line);
    const uint32_t v0 = SysTick->VAL;
    (void)Exti::clear(sw_line);
    const bool gone = !Exti::pending(sw_line);
    const uint32_t took = val_delta(v0, SysTick->VAL);
    print(serial, "  the clear and its read-back took ", took, " cycles", crlf);
    bench.verdict("PR is rc_w1: a zero written over a standing flag leaves it, and "
                  "so does a one written at every OTHER line",
                  zero_kept && others_kept);
    bench.verdict("a one clears it, and the read that follows the store already "
                  "sees it gone",
                  gone);

    // WHY THE ISR BODY CLEARS FIRST. The errata item filed under the RTC
    // (ES0206 2.9.3 and its twins) says the effective clear is DELAYED with
    // respect to the store: a handler that clears LAST returns while the
    // request still stands, and the NVIC enters it again.
    Nvic::enable(Exti::irq(sw_line));
    reset_counters();
    late_entries = 0;
    clear_late = 1;
    (void)Exti::trigger(sw_line);
    (void)delay_us(clock, 200);
    const uint16_t late = late_entries;
    clear_late = 0;
    (void)Exti::clear(sw_line);

    reset_counters();
    (void)Exti::trigger(sw_line);
    (void)delay_us(clock, 200);
    const uint16_t early = vector_entries;

    print(serial, "  one trigger: ", late, " entries clearing LAST, ", early,
          " clearing first", crlf);
    bench.verdict("clearing first serves one trigger exactly once", early == 1u);
    bench.verdict("and clearing it as the handler's LAST store is served once too "
                  "here - the delay the errata warns of did not outlive the "
                  "exception return on this part, which is a measurement and not a "
                  "licence: the driver clears first",
                  late == 1u);

    quiet_everything();
}

// =============================================================================
// l - the board's button
// =============================================================================
void tl_button() {
    quiet_everything();
    (void)ButtonInt::claim(button_pull);
    (void)ButtonInt::configure(button_press_is_rising ? ExtiSense::rising : ExtiSense::falling);
    (void)ButtonInt::arm(true);
    Nvic::enable(ButtonInt::irq());
    reset_counters();

    print(serial, "  the button is on line ", ButtonInt::line, " of port ", ButtonInt::port,
          ", vector ", static_cast<uint8_t>(ButtonInt::irq()), "; a press is a ",
          button_press_is_rising ? "RISING" : "FALLING", " edge, the pad reads ",
          Button::read() ? "high" : "low", " at rest", crlf);
    bench.verdict("the pad rests at the level a press moves away from",
                  Button::read() == !button_press_is_rising);
    bench.verdict("the line is armed, on its own port, with the sense a press makes",
                  ButtonInt::selected() && ButtonInt::armed() &&
                      ButtonInt::sense() ==
                          (button_press_is_rising ? ExtiSense::rising : ExtiSense::falling));

    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 50u) {
    }
    bench.verdict("and it raises nothing in 50 ms with nobody pressing", vector_entries == 0u);
    quiet_everything();
}

// =============================================================================
// p - wait for a press (not part of z)
// =============================================================================
void tp_press() {
    quiet_everything();
    (void)ButtonInt::claim(button_pull);
    (void)ButtonInt::configure(ExtiSense::both);
    (void)ButtonInt::arm(true);
    Nvic::enable(ButtonInt::irq());
    reset_counters();

    print(serial, "  press the button (5 s) ...", crlf);
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 5000u && vector_entries == 0u) {
    }
    const uint16_t seen = vector_entries;
    print(serial, "  ", seen, " edge(s), the pad now reads ", Button::read() ? "high" : "low",
          crlf);
    bench.verdict("a press reached the handler", seen > 0u);
    quiet_everything();
}

void banner() {
    print(serial, crlf, "test_stm32f4_exti - the EXTI's lines, triggers, flags, masks "
          "and vectors, and SYSCFG's multiplexer", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#elif defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void EXTI0_IRQHandler() { serve(EXTI0_IRQn); }
extern "C" void EXTI1_IRQHandler() {
    if (clear_late == 0u) {
        serve(EXTI1_IRQn);
        return;
    }
    // The wrong order, on purpose: everything the handler does comes
    // FIRST, and the clear is its last store before the return.
    late_entries = late_entries + 1u;
    if (late_entries > 4u) {   // never spin, whatever the silicon does
        clear_late = 0;
    }
    brio::Exti::clear_lines(brio::Exti::vector_lines(EXTI1_IRQn));
}
extern "C" void EXTI4_IRQHandler() { serve(EXTI4_IRQn); }
extern "C" void EXTI9_5_IRQHandler() { serve(EXTI9_5_IRQn); }
extern "C" void EXTI15_10_IRQHandler() { serve(EXTI15_10_IRQn); }
extern "C" void PVD_IRQHandler() { serve(PVD_IRQn); }
extern "C" void RTC_Alarm_IRQHandler() { serve(RTC_Alarm_IRQn); }
extern "C" void TAMP_STAMP_IRQHandler() { serve(TAMP_STAMP_IRQn); }
extern "C" void RTC_WKUP_IRQHandler() { serve(RTC_WKUP_IRQn); }
extern "C" void OTG_FS_WKUP_IRQHandler() { serve(OTG_FS_WKUP_IRQn); }

int main() {
    // What the silicon held before a line of this program ran. SYSCFG's
    // gate is read out of RCC, so reading it does not open it; the EXTI has
    // no gate at all.
    boot.syscfg_gate = brio::Syscfg::clock();
    boot.imr = brio::Exti::regs().IMR;
    boot.emr = brio::Exti::regs().EMR;
    boot.rtsr = brio::Exti::regs().RTSR;
    boot.ftsr = brio::Exti::regs().FTSR;
    boot.swier = brio::Exti::regs().SWIER;
    boot.pr = brio::Exti::regs().PR;
    for (uint8_t i = 0; i < 4u; ++i) {
        boot.exticr[i] = brio::Syscfg::regs().EXTICR[i];
    }

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "SYSCFG: the gate, the map, the compensation cell", ta_syscfg);
    bench.letter('b', "the EXTI's reset state and the multiplexer", tb_reset_and_mux);
    bench.letter('c', "the software trigger", tc_software_trigger);
    bench.letter('d', "the latency from the SWIER store to the handler", td_latency);
    bench.letter('e', "a pad the program drives, and the four senses", te_pad);
    bench.letter('f', "the two masks and the NVIC line", tf_masks);
    bench.letter('g', "an EXTI event out of WFE", tg_event);
    bench.letter('h', "the grouped vectors", th_grouped);
    bench.letter('i', "one pin number is one line", ti_one_line_per_number);
    bench.letter('j', "the lines above 15", tj_peripheral_lines);
    bench.letter('k', "the pending bit, and why a handler clears it first", tk_pending);
    bench.letter('l', "the board's button, armed and quiet", tl_button);
    bench.letter('p', "wait for a press of the button", tp_press, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
        bench.prompt();
    }
}
