// test_stm32_exti - the reference bench suite for the STM32G0's
// EXTENDED INTERRUPT AND EVENT CONTROLLER (RM0444 ch. 13) and, with it,
// the pin senses stm32g0/pin.hpp deliberately does not have: this
// family keeps no interrupt in GPIO at all, so an edge on a pad is this
// peripheral's business from the multiplexer to the vector.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, and that is the technique rather than a convenience.
// A chapter about EXTERNAL interrupts is exercised here with no wire at
// all, because the chip moves its own pads: the EXTI watches a GPIO
// PORT'S INPUT, which stays live in input, output and alternate modes
// alike (7.3.1), so a pad can be walked between the rails by its own
// internal pull OR driven by its own output driver, and either way the
// line sees a real edge on a real pin. Letter b measures both before
// anything after it is believed - and it also measures the one mode
// that hides a pad from its line (analog, where the input buffer is
// off).
//
// THE PADS, each chosen electrically free on this board (letter b's
// first verdicts) and each on a header pin, so a scope could check any
// of it:
//   PA0, PB0   line 0   (EXTI0_1) - and the same line through two
//                       ports, which is what makes the multiplexer
//                       measurable
//   PB3        line 3   (EXTI2_3)
//   PB7        line 7   (EXTI4_15)
//   PA8        line 8   (EXTI4_15, a second line on the shared vector)
//   PC13       the user button B1, READ ONLY: a press cannot be staged
//                       from here, so letter u prints the level and
//                       judges only what the board itself decides
// Avoided on purpose: PA2/PA3 (the console), PA5 (LD4), PA13/PA14
// (SWD), PC14/PC15 (the LSE pads), PF0/PF1 (the HSE pads).
//
// What is exercised, letter by letter:
//   a  the block: the header's geometry, the reset values this boot
//      found, the three vectors, and every refusal (a direct line's
//      triggers, a line the part does not have, a port it does not
//      bond)
//   b  the stimulus, measured rather than assumed: does a pad follow
//      its own pull, does the EXTI see a pad its owner is DRIVING, and
//      does analog mode blind it
//   c  the four senses counted on self-driven edges, and the two
//      pending registers proven to be two
//   d  the software trigger, through the real vector
//   e  the multiplexer: one line, two ports, and the pad that goes
//      quiet
//   f  the three shared vectors, each dispatching its own lines only
//   g  the masks: what an EXTI-masked line leaves behind (13.3.1's
//      claim, measured), and an EXTI EVENT returning the core from WFE
//      with no interrupt at all
//   h  ExtInt inside a REAL KERNEL: an edge becomes an event an active
//      object receives
//   u  the user button as a plain input
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// The console pads: USART2_TX on PA2, USART2_RX on PA3, both AF1
// (DS13560 table 13), which is the ST-LINK's virtual COM port.
constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

using Led = Pin<'A', 5>;   // LD4

TestBench<Serial> bench;

// ---- the pads and their lines ----------------------------------------------
using PadA0 = Pin<'A', 0>;
using PadB0 = Pin<'B', 0>;
using PadB3 = Pin<'B', 3>;
using PadB7 = Pin<'B', 7>;
using PadA8 = Pin<'A', 8>;
using PadButton = Pin<'C', 13>;

using IntA0 = ExtInt<PadA0>;
using IntB0 = ExtInt<PadB0>;
using IntB3 = ExtInt<PadB3>;
using IntB7 = ExtInt<PadB7>;
using IntA8 = ExtInt<PadA8>;

// The application-level guard the driver cannot apply for us: PA0 and
// PB0 ARE the same line, and this suite means them to be - it uses them
// one at a time to measure the multiplexer. Every OTHER pair must be
// distinct, and that is what is asserted.
static_assert(exti_lines_distinct<IntA0, IntB3, IntB7, IntA8>(),
              "the suite's own lines must not collide");
static_assert(!exti_lines_distinct<IntA0, IntB0>(),
              "PA0 and PB0 are one line - the whole point of letter e");

// What this boot found in the EXTI, sampled in main() before a letter
// can disturb it.
uint32_t boot_imr1 = 0;
uint32_t boot_imr2 = 0;
uint32_t boot_emr1 = 0;
uint32_t boot_rtsr1 = 0;
uint32_t boot_exticr0 = 0;

// =============================================================================
// What the handlers count
// =============================================================================
//
// One counter per line per edge, plus WHICH VECTOR served it - which is
// the whole of letter f's question - plus, per vector, whatever pending
// bits were standing OUTSIDE that vector's own lines when it ran: the
// evidence that Exti::isr(mask) confines a handler to its own lines
// rather than eating a neighbour's.
volatile uint16_t line_rising[16];
volatile uint16_t line_falling[16];
volatile uint8_t line_vector[16];
volatile uint16_t vector_calls[4];
volatile uint32_t vector_saw_outside[4];

void clear_counts() {
    for (uint8_t i = 0; i < 16; ++i) {
        line_rising[i] = 0;
        line_falling[i] = 0;
        line_vector[i] = 0;
    }
    for (uint8_t i = 0; i < 4; ++i) {
        vector_calls[i] = 0;
        vector_saw_outside[i] = 0;
    }
}

/// The body every EXTI vector of this suite runs. `tag` is 1/2/3 for
/// EXTI0_1 / EXTI2_3 / EXTI4_15, and it is what a line's counter
/// records, so "a line 7 edge never enters EXTI0_1's handler" is a
/// reading and not an inference.
[[gnu::always_inline]] inline void serve(IRQn_Type v, uint8_t tag) {
    const uint32_t mine = Exti::vector_lines(v);
    vector_saw_outside[tag] |=
        (Exti::rising_pending() | Exti::falling_pending()) & ~mine;
    const ExtiPending p = Exti::isr(mine);
    vector_calls[tag] = static_cast<uint16_t>(vector_calls[tag] + 1);
    for (uint8_t l = 0; l < 16; ++l) {
        const uint32_t b = static_cast<uint32_t>(1u) << l;
        if ((p.rising & b) != 0u) {
            line_rising[l] = static_cast<uint16_t>(line_rising[l] + 1);
            line_vector[l] = tag;
        }
        if ((p.falling & b) != 0u) {
            line_falling[l] = static_cast<uint16_t>(line_falling[l] + 1);
            line_vector[l] = tag;
        }
    }
}

// =============================================================================
// Instruments
// =============================================================================

/// A cycle-resolution stopwatch, the other two strata's suites' own:
/// ticks x period + the phase SysTick has already counted down, the two
/// reads retried until they belong to the same tick.
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000UL;
uint32_t cycles_to_us(uint32_t cycles) { return cycles / cycles_per_us; }

/// Long enough for a floating pad's own RC to settle under a ~40 kohm
/// internal pull - the slowest stimulus this suite uses. An
/// output-driven edge is nanoseconds; this covers both.
void settle() { (void)delay_us(clock, 200); }

/// Wait for the console to be physically empty - a WFE that any pending
/// interrupt can return is only a measurement when nothing else is
/// asking (test_stm32_platform's idle() letter paid for this lesson).
void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

/// THE PRECONDITION OF EVERYTHING: an input pad with nothing attached
/// goes where its own pull sends it. A pad that fails this is wired to
/// something and is no stimulus.
template <class Pad>
bool pad_follows_pull() {
    Pad::input(PinPull::up);
    settle();
    const bool high = Pad::read();
    Pad::input(PinPull::down);
    settle();
    const bool low = Pad::read();
    Pad::input(PinPull::none);
    return high && !low;
}

/// The two stimulus techniques, as verbs.
template <class Pad>
void walk_to(bool high) {
    Pad::pull(high ? PinPull::up : PinPull::down);
    settle();
}
template <class Pad>
void drive_to(bool high) {
    if (high) {
        Pad::set();
    } else {
        Pad::clear();
    }
    settle();
}

/// A pad parked at a known level, in output mode, ready to be driven.
template <class Pad>
void park_driven(bool level) {
    Pad::output(level);
    settle();
}

/// Every line this suite touches back to reset, and the pads with them.
void quiet_everything() {
    Nvic::disable(EXTI0_1_IRQn);
    Nvic::disable(EXTI2_3_IRQn);
    Nvic::disable(EXTI4_15_IRQn);
    IntA0::release();
    IntB0::release();
    IntB3::release();
    IntB7::release();
    IntA8::release();
    Nvic::clear_pending(EXTI0_1_IRQn);
    Nvic::clear_pending(EXTI2_3_IRQn);
    Nvic::clear_pending(EXTI4_15_IRQn);
}

/// Arm a line for POLLING: the EXTI's own interrupt mask has to be
/// open, because on this family a pending bit only appears for an
/// unmasked interrupt (13.3.1) - but the NVIC line stays shut, so
/// nothing is dispatched and the flags are the caller's to read.
template <class Line, class Pad>
bool poll_line(ExtiSense s) {
    const bool claimed = Line::select();
    const bool sensed = Line::configure(s);
    const bool armed = Line::arm(true);
    (void)Line::clear();
    Nvic::disable(Line::irq());
    Nvic::clear_pending(Line::irq());
    (void)sizeof(Pad);
    return claimed && sensed && armed;
}

// =============================================================================
// a - the block: geometry, reset values, vectors, refusals
// =============================================================================
void ta_block() {
    print(serial, "  lines: implemented1=", hex(exti_implemented_mask1),
          " configurable1=", hex(exti_configurable_mask1),
          " implemented2=", hex(exti_implemented_mask2),
          " configurable2=", hex(exti_configurable_mask2), crlf);

    bench.verdict("sixteen GPIO lines, read off EXTICR's own geometry",
                  Exti::gpio_lines == 16u);
    bench.verdict("lines 0..15 are all implemented and all configurable",
                  (exti_implemented_mask1 & 0xFFFFu) == 0xFFFFu &&
                      (exti_configurable_mask1 & 0xFFFFu) == 0xFFFFu);
    bench.verdict("and only those sixteen are GPIO lines",
                  Exti::gpio(15) && !Exti::gpio(16));

    // Table 65 on THIS part: the RTC is line 19 and it is DIRECT, the
    // PVD is line 16 and is CONFIGURABLE. Named as landmarks, not as
    // vocabulary this driver owns (see the file header of exti.hpp).
    bench.verdict("line 19 (the RTC) is implemented and DIRECT - no "
                  "trigger, no pending bit of its own",
                  Exti::implemented(19) && !Exti::configurable(19));
    bench.verdict("line 16 (the PVD) is implemented and CONFIGURABLE",
                  Exti::implemented(16) && Exti::configurable(16));
    bench.verdict("this part has the second register group (lines 32..36) "
                  "and line 34 is configurable in it",
                  Exti::implemented(34) && Exti::configurable(34) &&
                      exti_rtsr2() != nullptr && exti_imr2() != nullptr);

    // THE RESET VALUES THIS BOOT FOUND, and the first place the manual
    // disagrees with itself. 13.5.12 states the RULE in words - "the
    // reset value is set such as to, by default, enable interrupt from
    // direct lines, and disable interrupt from configurable lines" -
    // and then prints 0xFFF8 0000, which has line 20 unmasked although
    // line 20 (COMP3) is a CONFIGURABLE line on this part. The rule and
    // the number cannot both be right, so this verdict tests THE RULE,
    // computed from the device header's own two masks, and prints what
    // the silicon actually held.
    const uint32_t direct1 = exti_implemented_mask1 & ~exti_configurable_mask1;
    const uint32_t direct2 = exti_implemented_mask2 & ~exti_configurable_mask2;
    print(serial, "  at boot: IMR1=", hex(boot_imr1), " (", hex(direct1),
          " = the direct lines), IMR2=", hex(boot_imr2), " (", hex(direct2),
          "), EMR1=", hex(boot_emr1), " RTSR1=", hex(boot_rtsr1),
          " EXTICR1=", hex(boot_exticr0), crlf);
    bench.verdict("IMR1 came up with EXACTLY the direct lines unmasked - the "
                  "header's implemented and configurable masks predict the "
                  "reset value bit for bit, where 13.5.12's printed "
                  "0xFFF80000 would leave line 20 (COMP3, a CONFIGURABLE "
                  "line here) unmasked",
                  boot_imr1 == direct1);
    bench.verdict("and so did IMR2, where the printed 0x1B and the rule do "
                  "agree (line 34 is the only configurable one up there)",
                  boot_imr2 == direct2);
    bench.verdict("EMR1 came up at zero (no CPU event is unmasked)",
                  boot_emr1 == 0u);
    bench.verdict("no trigger was selected before this suite ran",
                  boot_rtsr1 == 0u);
    bench.verdict("and every line's multiplexer was at its reset code, "
                  "port A", boot_exticr0 == 0u);

    // The three vectors.
    bench.verdict("lines 0..1 interrupt on EXTI0_1, 2..3 on EXTI2_3, the "
                  "rest on EXTI4_15",
                  Exti::irq(0) == EXTI0_1_IRQn && Exti::irq(1) == EXTI0_1_IRQn &&
                      Exti::irq(2) == EXTI2_3_IRQn && Exti::irq(3) == EXTI2_3_IRQn &&
                      Exti::irq(4) == EXTI4_15_IRQn && Exti::irq(15) == EXTI4_15_IRQn);
    bench.verdict("the three vectors partition the sixteen lines exactly",
                  (Exti::vector_lines(EXTI0_1_IRQn) ^
                   Exti::vector_lines(EXTI2_3_IRQn) ^
                   Exti::vector_lines(EXTI4_15_IRQn)) == 0xFFFFu &&
                      (Exti::vector_lines(EXTI0_1_IRQn) &
                       Exti::vector_lines(EXTI4_15_IRQn)) == 0u);

    // The refusals.
    bench.verdict("a DIRECT line refuses a trigger selection",
                  !Exti::sense(19, ExtiSense::rising));
    bench.verdict("a DIRECT line refuses a software trigger",
                  !Exti::trigger(19));
    bench.verdict("a DIRECT line has no pending bit to clear",
                  !Exti::clear(19));
    bench.verdict("but its interrupt mask is writable - that is the one "
                  "thing the EXTI does for a direct line",
                  Exti::interrupt(19, Exti::interrupt(19)));
    bench.verdict("a line this part does not implement is refused everywhere",
                  !Exti::interrupt(63, true) && !Exti::sense(63, ExtiSense::rising) &&
                      !Exti::trigger(63) && !Exti::implemented(63));
    bench.verdict("the multiplexer refuses a port this device does not bond",
                  !Exti::select(3, 'G') && !Exti::select(3, 'Z'));
    bench.verdict("and refuses a line that has no multiplexer at all",
                  !Exti::select(16, 'A') && Exti::selected(16) == 0);

    // The multiplexer, read back through the driver.
    bench.verdict("line 7 can be pointed at port B and says so",
                  Exti::select(7, 'B') && Exti::selected(7) == 'B');
    bench.verdict("and back at port A", Exti::select(7, 'A') &&
                      Exti::selected(7) == 'A');

    // The sense pair, written and read back as one value.
    bench.verdict("a sense is the two trigger bits and reads back as one "
                  "value",
                  Exti::sense(7, ExtiSense::both) &&
                      Exti::sense(7) == ExtiSense::both &&
                      Exti::sense(7, ExtiSense::falling) &&
                      Exti::sense(7) == ExtiSense::falling &&
                      Exti::sense(7, ExtiSense::none) &&
                      Exti::sense(7) == ExtiSense::none);

    quiet_everything();
}

// =============================================================================
// b - the stimulus, measured rather than assumed
// =============================================================================
//
// Two questions no chapter answers for a wireless bench. First: is each
// pad electrically free, so that its own pull moves it? Second - and
// this is where the STM32 differs from the SAM, whose PMUXEN takes a
// pad away from PORT's output driver - does the EXTI see a pad its
// owner is DRIVING? 7.3.1 says the input buffer is on in output mode
// too, and the EXTI's multiplexer selects a PORT and not a pin
// function, so the answer should be yes; everything after this letter
// uses the cheaper, sharper stimulus if it is.
void tb_stimulus() {
    bench.verdict("PA0 is electrically free (it follows its own pull)",
                  pad_follows_pull<PadA0>());
    bench.verdict("so is PB0", pad_follows_pull<PadB0>());
    bench.verdict("so is PB3", pad_follows_pull<PadB3>());
    bench.verdict("so is PB7", pad_follows_pull<PadB7>());
    bench.verdict("and so is PA8", pad_follows_pull<PadA8>());

    // The two line-0 pads must be electrically independent or letter e
    // measures nothing.
    PadA0::output(true);
    PadB0::input(PinPull::down);
    settle();
    const bool b0_low_while_a0_high = !PadB0::read();
    PadA0::output(false);
    PadB0::input(PinPull::up);
    settle();
    const bool b0_high_while_a0_low = PadB0::read();
    bench.verdict("PA0 and PB0 are two separate pins (one driven does not "
                  "move the other)",
                  b0_low_while_a0_high && b0_high_while_a0_low);

    // --- technique 1: the internal pull
    PadB3::input(PinPull::down);
    (void)poll_line<IntB3, PadB3>(ExtiSense::rising);
    settle();
    (void)IntB3::clear();
    walk_to<PadB3>(true);
    const bool pull_rising = IntB3::rising_pending();
    (void)IntB3::clear();
    walk_to<PadB3>(false);
    const bool pull_falling_on_rising_sense = IntB3::pending();
    print(serial, "  pull-walked PB3: a rising sense saw the up-flip -> ",
          pull_rising ? "1" : "0", ", the down-flip -> ",
          pull_falling_on_rising_sense ? "1" : "0", crlf);
    bench.verdict("THE INTERNAL PULL IS A STIMULUS: flipping PUPDR moves the "
                  "pad between the rails and the line sees the edge",
                  pull_rising);
    bench.verdict("and it is an EDGE and not a level: the opposite flip "
                  "leaves a rising-only line silent",
                  !pull_falling_on_rising_sense);

    // --- technique 2: the pad's own output driver
    park_driven<PadB3>(false);
    (void)IntB3::clear();
    drive_to<PadB3>(true);
    const bool driven_rising = IntB3::rising_pending();
    (void)IntB3::clear();
    drive_to<PadB3>(false);
    const bool driven_falling_seen = IntB3::pending();
    print(serial, "  PORT-driven PB3 (output mode): rising -> ",
          driven_rising ? "1" : "0", ", falling on a rising sense -> ",
          driven_falling_seen ? "1" : "0", crlf);
    bench.verdict("THE EXTI SEES A PAD ITS OWNER IS DRIVING: the input "
                  "buffer stays live in output mode (7.3.1) and the "
                  "multiplexer selects a PORT, not a pin function - so this "
                  "is not the SAM's PMUXEN situation at all",
                  driven_rising && !driven_falling_seen);

    // --- and the one mode that hides a pad: analog, buffer off
    PadB3::analog();
    settle();
    (void)IntB3::clear();
    // Nothing can drive an analog pad from inside, so the witness is
    // that a pull-flip - which still writes PUPDR - produces no edge.
    PadB3::pull(PinPull::up);
    settle();
    PadB3::pull(PinPull::down);
    settle();
    const bool analog_silent = !IntB3::pending();
    bench.verdict("ANALOG MODE BLINDS THE LINE: with the input buffer off "
                  "the pad's own pull moves nothing the EXTI can see",
                  analog_silent);

    quiet_everything();
}

// =============================================================================
// c - the four senses, counted
// =============================================================================
void tc_senses() {
    constexpr uint8_t walks = 8;

    struct Row {
        ExtiSense sense;
        const char* name;
        uint8_t rising;
        uint8_t falling;
    };
    Row rows[] = {
        {ExtiSense::rising, "rising", 0, 0},
        {ExtiSense::falling, "falling", 0, 0},
        {ExtiSense::both, "both", 0, 0},
        {ExtiSense::none, "none", 0, 0},
    };

    for (Row& r : rows) {
        park_driven<PadB7>(false);
        bench.verdict("the line takes its sense",
                      poll_line<IntB7, PadB7>(r.sense));
        (void)IntB7::clear();
        for (uint8_t i = 0; i < walks; ++i) {
            drive_to<PadB7>(true);
            if (IntB7::rising_pending()) {
                ++r.rising;
            }
            if (IntB7::falling_pending()) {
                ++r.falling;
            }
            (void)IntB7::clear();
            drive_to<PadB7>(false);
            if (IntB7::rising_pending()) {
                ++r.rising;
            }
            if (IntB7::falling_pending()) {
                ++r.falling;
            }
            (void)IntB7::clear();
        }
        print(serial, "  ", r.name, ": ", walks, " up + ", walks,
              " down edges -> ", r.rising, " rising pending, ", r.falling,
              " falling pending", crlf);
    }

    bench.verdict("a RISING line flags every up edge and no down edge",
                  rows[0].rising == walks && rows[0].falling == 0u);
    bench.verdict("a FALLING line flags every down edge and no up edge",
                  rows[1].falling == walks && rows[1].rising == 0u);
    bench.verdict("a BOTH line flags each edge in ITS OWN register - the "
                  "two pending bits are two, so a handler knows which edge "
                  "arrived without reading the pad",
                  rows[2].rising == walks && rows[2].falling == walks);
    bench.verdict("a line with no trigger selected flags nothing at all",
                  rows[3].rising == 0u && rows[3].falling == 0u);
    bench.verdict("and no edge was ever counted twice (an unfiltered "
                  "edge detector on a driven pad does not bounce)",
                  rows[2].rising + rows[2].falling == 2u * walks);

    // The pull-walked version of the same count, to show the softer
    // stimulus is as clean as the driven one.
    PadB7::input(PinPull::down);
    (void)poll_line<IntB7, PadB7>(ExtiSense::both);
    (void)IntB7::clear();
    uint8_t walked = 0;
    for (uint8_t i = 0; i < walks; ++i) {
        walk_to<PadB7>(true);
        if (IntB7::rising_pending()) {
            ++walked;
        }
        (void)IntB7::clear();
        walk_to<PadB7>(false);
        if (IntB7::falling_pending()) {
            ++walked;
        }
        (void)IntB7::clear();
    }
    print(serial, "  pull-walked: ", walked, " of ", 2u * walks,
          " edges seen", crlf);
    bench.verdict("the pull-driven stimulus is as sharp as the driven one",
                  walked == 2u * walks);

    quiet_everything();
}

// =============================================================================
// d - the software trigger, through the real vector
// =============================================================================
void td_software() {
    clear_counts();
    // No pad at all here: SWIER is the one stimulus that needs none.
    bench.verdict("line 3 arms with NO trigger edge selected",
                  IntB3::select() && IntB3::configure(ExtiSense::none) &&
                      IntB3::arm(true));
    (void)IntB3::clear();
    Nvic::clear_pending(EXTI2_3_IRQn);
    Nvic::enable(EXTI2_3_IRQn);

    constexpr uint8_t shots = 5;
    for (uint8_t i = 0; i < shots; ++i) {
        bench.verdict("SWIER takes the write", IntB3::trigger());
        settle();
    }

    print(serial, "  ", shots, " software triggers -> ", line_rising[3],
          " rising, ", line_falling[3], " falling, vector ", line_vector[3],
          ", ", vector_calls[2], " handler calls", crlf);
    bench.verdict("a software trigger is a RISING edge and reaches the "
                  "handler through the real vector, with no trigger edge "
                  "enabled and no pad involved (13.5.3)",
                  line_rising[3] == shots && line_falling[3] == 0u);
    bench.verdict("it arrived on EXTI2_3, the vector line 3 belongs to",
                  line_vector[3] == 2u);
    bench.verdict("the bit clears itself: SWIER reads back zero",
                  (Exti::regs().SWIER1 & IntB3::mask) == 0u);
    bench.verdict("and nothing is left pending after the handler",
                  !IntB3::pending());

    // A software trigger on a line whose EXTI interrupt is MASKED: the
    // other half of 13.3.1's rule, and the letter that reads it fully
    // is g.
    Nvic::disable(EXTI2_3_IRQn);
    bench.verdict("the line masks", IntB3::arm(false));
    (void)IntB3::clear();
    (void)IntB3::trigger();
    settle();
    const bool masked_pending = IntB3::pending();
    print(serial, "  a software trigger with the interrupt masked left "
          "pending = ", masked_pending ? "1" : "0", crlf);
    bench.verdict("a software trigger with EXTI_IMR clear leaves NO pending "
                  "bit either - 13.3.1's rule is about the pending register "
                  "and not about the source of the edge",
                  !masked_pending);

    quiet_everything();
}

// =============================================================================
// e - the multiplexer: one line, two ports
// =============================================================================
void te_mux() {
    park_driven<PadA0>(false);
    park_driven<PadB0>(false);

    // Line 0 pointed at port A.
    bench.verdict("line 0 takes port A", IntA0::select() && IntA0::selected());
    bench.verdict("and PB0's own view of the same line says NOT mine",
                  !IntB0::selected());
    bench.verdict("the line arms on a rising edge",
                  IntA0::configure(ExtiSense::rising) && IntA0::arm(true));
    Nvic::disable(EXTI0_1_IRQn);
    (void)IntA0::clear();

    drive_to<PadA0>(true);
    const bool a_seen = IntA0::rising_pending();
    (void)IntA0::clear();
    drive_to<PadA0>(false);
    drive_to<PadB0>(true);
    const bool b_seen_while_a_selected = IntA0::rising_pending();
    (void)IntA0::clear();
    drive_to<PadB0>(false);

    // The same line pointed at port B; nothing else changes.
    bench.verdict("line 0 moves to port B", IntB0::select() && IntB0::selected());
    bench.verdict("and PA0's view now says NOT mine", !IntA0::selected());
    (void)IntB0::clear();
    drive_to<PadB0>(true);
    const bool b_seen = IntB0::rising_pending();
    (void)IntB0::clear();
    drive_to<PadB0>(false);
    drive_to<PadA0>(true);
    const bool a_seen_while_b_selected = IntB0::rising_pending();
    (void)IntB0::clear();
    drive_to<PadA0>(false);

    print(serial, "  EXTICR at port A: PA0 edge -> ", a_seen ? "1" : "0",
          ", PB0 edge -> ", b_seen_while_a_selected ? "1" : "0",
          "; at port B: PB0 edge -> ", b_seen ? "1" : "0", ", PA0 edge -> ",
          a_seen_while_b_selected ? "1" : "0", crlf);

    bench.verdict("with the multiplexer on port A, PA0's edge is the line's",
                  a_seen);
    bench.verdict("and PB0's edge is invisible - ONE PIN PER LINE is the "
                  "silicon's rule, not a convention",
                  !b_seen_while_a_selected);
    bench.verdict("with it on port B the answers swap exactly",
                  b_seen && !a_seen_while_b_selected);
    bench.verdict("the sense, the mask and the pending bits belong to the "
                  "LINE and survive the port change untouched",
                  Exti::sense(0) == ExtiSense::rising && Exti::interrupt(0));

    quiet_everything();
}

// =============================================================================
// f - the three shared vectors
// =============================================================================
void tf_vectors() {
    clear_counts();
    park_driven<PadA0>(false);   // line 0  -> EXTI0_1
    park_driven<PadB3>(false);   // line 3  -> EXTI2_3
    park_driven<PadB7>(false);   // line 7  -> EXTI4_15
    park_driven<PadA8>(false);   // line 8  -> EXTI4_15

    bench.verdict("four lines on three vectors arm together",
                  IntA0::select() && IntA0::configure(ExtiSense::both) &&
                      IntA0::arm(true) && IntB3::select() &&
                      IntB3::configure(ExtiSense::both) && IntB3::arm(true) &&
                      IntB7::select() && IntB7::configure(ExtiSense::both) &&
                      IntB7::arm(true) && IntA8::select() &&
                      IntA8::configure(ExtiSense::both) && IntA8::arm(true));
    Exti::clear_rising(0xFFFFu);
    Exti::clear_falling(0xFFFFu);
    Nvic::clear_pending(EXTI0_1_IRQn);
    Nvic::clear_pending(EXTI2_3_IRQn);
    Nvic::clear_pending(EXTI4_15_IRQn);
    Nvic::enable(EXTI0_1_IRQn);
    Nvic::enable(EXTI2_3_IRQn);
    Nvic::enable(EXTI4_15_IRQn);

    constexpr uint8_t rounds = 4;
    for (uint8_t i = 0; i < rounds; ++i) {
        drive_to<PadA0>(true);  drive_to<PadA0>(false);
        drive_to<PadB3>(true);  drive_to<PadB3>(false);
        drive_to<PadB7>(true);  drive_to<PadB7>(false);
        drive_to<PadA8>(true);  drive_to<PadA8>(false);
    }

    print(serial, "  line 0: ", line_rising[0], "+", line_falling[0],
          " on vector ", line_vector[0], "; line 3: ", line_rising[3], "+",
          line_falling[3], " on vector ", line_vector[3], "; line 7: ",
          line_rising[7], "+", line_falling[7], " on vector ", line_vector[7],
          "; line 8: ", line_rising[8], "+", line_falling[8], " on vector ",
          line_vector[8], crlf);
    print(serial, "  handler calls: EXTI0_1 ", vector_calls[1], ", EXTI2_3 ",
          vector_calls[2], ", EXTI4_15 ", vector_calls[3], crlf);

    bench.verdict("every edge of every line reached a handler",
                  line_rising[0] == rounds && line_falling[0] == rounds &&
                      line_rising[3] == rounds && line_falling[3] == rounds &&
                      line_rising[7] == rounds && line_falling[7] == rounds &&
                      line_rising[8] == rounds && line_falling[8] == rounds);
    bench.verdict("line 0 was served by EXTI0_1 and nothing else",
                  line_vector[0] == 1u);
    bench.verdict("line 3 by EXTI2_3", line_vector[3] == 2u);
    bench.verdict("lines 7 and 8 by EXTI4_15, the vector that carries twelve",
                  line_vector[7] == 3u && line_vector[8] == 3u);
    bench.verdict("and the three handlers ran the number of times their own "
                  "lines fired",
                  vector_calls[1] == 2u * rounds && vector_calls[2] == 2u * rounds &&
                      vector_calls[3] == 4u * rounds);

    // THE CONFINEMENT, staged deliberately: with EXTI4_15 shut at the
    // NVIC, raise line 7 and leave it pending; then let EXTI0_1 run.
    // Its body is handed vector_lines(EXTI0_1) and must leave the
    // neighbour's bit exactly where it found it.
    Nvic::disable(EXTI4_15_IRQn);
    clear_counts();
    Exti::clear_rising(0xFFFFu);
    Exti::clear_falling(0xFFFFu);
    bench.verdict("line 7 is left pending with its vector shut",
                  IntB7::trigger() && (settle(), IntB7::rising_pending()));
    (void)IntA0::trigger();
    settle();
    print(serial, "  EXTI0_1 ran with line 7 pending: it SAW ",
          hex(vector_saw_outside[1]), " outside its own lines and line 7 is ",
          IntB7::rising_pending() ? "still pending" : "GONE", crlf);
    bench.verdict("the EXTI0_1 handler ran and served line 0",
                  line_rising[0] == 1u && vector_calls[1] == 1u);
    bench.verdict("it could SEE line 7's pending bit (one register serves "
                  "every line)",
                  (vector_saw_outside[1] & IntB7::mask) != 0u);
    bench.verdict("and Exti::isr(vector_lines) LEFT IT ALONE: a shared "
                  "pending register does not make a shared handler",
                  IntB7::rising_pending());
    bench.verdict("nor did it touch line 7's counters",
                  line_rising[7] == 0u && line_falling[7] == 0u);

    quiet_everything();
}

// =============================================================================
// g - the two masks: a masked line, and an EXTI EVENT out of WFE
// =============================================================================
void tg_masks() {
    clear_counts();
    park_driven<PadB7>(false);

    // --- 13.3.1's claim, measured: no interrupt, no pending bit.
    bench.verdict("line 7 takes a both-edges sense with its interrupt MASKED",
                  IntB7::select() && IntB7::configure(ExtiSense::both) &&
                      IntB7::arm(false) && !IntB7::armed());
    (void)IntB7::clear();
    Nvic::disable(EXTI4_15_IRQn);
    for (uint8_t i = 0; i < 4; ++i) {
        drive_to<PadB7>(true);
        drive_to<PadB7>(false);
    }
    const bool masked_pending = IntB7::pending();
    print(serial, "  8 edges with EXTI_IMR clear left pending = ",
          masked_pending ? "1" : "0", " (RPR1=", hex(Exti::rising_pending()),
          " FPR1=", hex(Exti::falling_pending()), ")", crlf);
    bench.verdict("THE PENDING BIT IS ONLY SET FOR AN UNMASKED INTERRUPT "
                  "(13.3.1): with EXTI_IMR clear eight edges leave no trace "
                  "at all - so a line cannot be POLLED without arming it, "
                  "which both other brio targets allow",
                  !masked_pending);
    bench.verdict("and unmasking afterwards does not resurrect them",
                  IntB7::arm(true) && !IntB7::pending());
    (void)IntB7::arm(false);
    (void)IntB7::clear();

    // --- the CPU event: WFE returns, with no interrupt and no flag.
    //
    // The measurement is a TIME. The CPU's event register latches an
    // EXTI event while the core is awake, so a WFE executed after the
    // edge returns AT ONCE if the event was taken and waits for the
    // next SysTick interrupt if it was not. Both legs start right after
    // a tick, so the "not taken" leg has a whole millisecond to wait
    // and the comparison is not a coin toss.
    // NOT ONE PRINT BETWEEN HERE AND THE MEASUREMENT. A verdict line is
    // milliseconds of console, and every byte of it is a USART
    // interrupt: an interrupt PENDING IN THE NVIC returns WFE too
    // (4.2.2's first bullet), so a talking console makes both legs
    // measure zero. The verdicts below are computed now and PRINTED
    // after the two sleeps.
    const bool event_unmasked =
        IntB7::event(true) && IntB7::event() && !IntB7::armed();
    console_drain();

    // THE ORDER BELOW IS THE MEASUREMENT, and the first version of this
    // letter got it wrong: on this core the event register is set by AN
    // EXCEPTION ENTRY as well as by SEV and by an external event, so
    // the loop that waits for a tick edge - which ends by returning
    // from SysTick_Handler - leaves the register SET. A WFE right after
    // it returns at once whatever the EXTI did, and the control leg
    // measured 0 us for the wrong reason. So: align to the tick FIRST,
    // then clear the event register with SEV + WFE, then make the edge,
    // then sleep. Nothing takes an exception in between - delay_us and
    // the stopwatch only READ SysTick - so the register's state is the
    // EXTI's doing alone.
    uint32_t t = Ticker::ticks();
    while (Ticker::ticks() == t) {
    }
    __SEV();
    __WFE();                       // consumes it: the register is now clear
    drive_to<PadB7>(true);         // the edge under test
    uint32_t c0 = cycles_now();
    __WFE();
    const uint32_t event_us = cycles_to_us(cycles_now() - c0);
    const bool event_pending = IntB7::pending();

    // The control: the same edge with the CPU event masked, staged
    // exactly the same way. Nothing but the next tick can end it.
    (void)IntB7::event(false);
    (void)IntB7::clear();
    drive_to<PadB7>(false);
    t = Ticker::ticks();
    while (Ticker::ticks() == t) {
    }
    __SEV();
    __WFE();
    drive_to<PadB7>(true);
    c0 = cycles_now();
    __WFE();
    const uint32_t control_us = cycles_to_us(cycles_now() - c0);

    print(serial, "  WFE after an edge: ", event_us,
          " us with the CPU event unmasked, ", control_us,
          " us with it masked (the tick is the only other wake)", crlf);
    bench.verdict("line 7's CPU EVENT unmasks (and its interrupt stays "
                  "masked)",
                  event_unmasked);
    bench.verdict("AN EXTI EVENT RETURNS THE CORE FROM WFE with no handler, "
                  "no NVIC line and nothing to acknowledge (4.2.2)",
                  event_us < 100u);
    bench.verdict("with the event masked the same edge does nothing and the "
                  "tick is what ends the wait, most of a millisecond later",
                  control_us > 500u);
    bench.verdict("and an EVENT SETS NO PENDING BIT: 13.4 says the pending "
                  "register is not set for an unmasked CPU event, so there "
                  "is nothing for a woken program to clear",
                  !event_pending);

    quiet_everything();
}

// =============================================================================
// h - ExtInt inside a real kernel
// =============================================================================
//
// The point of the letter: an edge on a pad becomes an EVENT an active
// object receives, through the kernel's own queue, with the handler
// doing nothing but post() - which is legal from an ISR by contract.

struct Edges {
    uint8_t line;
    bool rising;
};

struct Counter {
    using Event = std::variant<Edges>;
    static inline EventQueue<Event, 16, Stm32Platform> queue;

    static inline uint16_t rising = 0;
    static inline uint16_t falling = 0;
    static inline uint8_t last_line = 0xFF;

    static void init() {
        rising = 0;
        falling = 0;
        last_line = 0xFF;
    }
    static void dispatch(const Event& e) {
        match(e, [](Edges x) {
            last_line = x.line;
            if (x.rising) {
                ++rising;
            } else {
                ++falling;
            }
        });
    }
};

using EdgeKernel = Kernel<Stm32Platform, Counter>;

/// Raised only while letter h runs: the shared EXTI4_15 handler posts
/// to the kernel instead of counting, and every other letter would be
/// posting into a queue nobody pumps.
volatile bool kernel_mode = false;

void th_kernel() {
    clear_counts();
    park_driven<PadA8>(false);
    EdgeKernel::init_all();

    bench.verdict("PA8's line arms for both edges",
                  IntA8::select() && IntA8::configure(ExtiSense::both) &&
                      IntA8::arm(true));
    (void)IntA8::clear();
    Nvic::clear_pending(EXTI4_15_IRQn);
    Nvic::enable(EXTI4_15_IRQn);

    constexpr uint8_t rounds = 6;
    kernel_mode = true;
    for (uint8_t i = 0; i < rounds; ++i) {
        drive_to<PadA8>(true);
        while (EdgeKernel::step()) {
        }
        drive_to<PadA8>(false);
        while (EdgeKernel::step()) {
        }
    }
    kernel_mode = false;

    print(serial, "  ", rounds, " up + ", rounds, " down edges -> ",
          Counter::rising, " rising and ", Counter::falling,
          " falling events received, last line ", Counter::last_line,
          ", queue overflows ", Counter::queue.overflows(), crlf);

    bench.verdict("every edge became an event the active object received",
                  Counter::rising == rounds && Counter::falling == rounds);
    bench.verdict("each labelled with the line it came from",
                  Counter::last_line == IntA8::line);
    bench.verdict("and nothing was dropped on the way",
                  Counter::queue.overflows() == 0u);

    quiet_everything();
}

// =============================================================================
// u - the user button, as a plain input
// =============================================================================
//
// B1 cannot be pressed from here, so this letter judges only what the
// BOARD decides and prints the rest. It is worth a letter because the
// pad's resting state is a board fact this suite is the first thing to
// measure, and because it is the one EXTI line on this Nucleo that a
// human can exercise by hand.
void tu_button() {
    PadButton::input(PinPull::down);
    settle();
    const bool with_pulldown = PadButton::read();
    PadButton::input(PinPull::up);
    settle();
    const bool with_pullup = PadButton::read();
    PadButton::input(PinPull::none);
    settle();
    const bool floating = PadButton::read();

    print(serial, "  PC13 (B1) reads ", with_pulldown ? "HIGH" : "low",
          " against an internal pull-down, ", with_pullup ? "HIGH" : "low",
          " against a pull-up, ", floating ? "HIGH" : "low",
          " with no pull - press B1 and run u again to see it fall", crlf);

    bench.verdict("B1's pad is held HIGH by the BOARD: an internal "
                  "pull-down does not move it, so the Nucleo's own pull-up "
                  "is the stronger one and a press is a FALLING edge",
                  with_pulldown && with_pullup && floating);

    // The line itself, armed on the edge a press would make. No verdict
    // depends on a press; what is judged is that the line takes the
    // configuration and that nothing fires on its own.
    bench.verdict("line 13 takes port C and a falling sense",
                  ExtInt<PadButton>::select() && ExtInt<PadButton>::selected() &&
                      ExtInt<PadButton>::configure(ExtiSense::falling) &&
                      ExtInt<PadButton>::arm(true));
    (void)ExtInt<PadButton>::clear();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 50u) {
    }
    const bool spontaneous = ExtInt<PadButton>::pending();
    print(serial, "  50 ms armed with nobody pressing: pending = ",
          spontaneous ? "1" : "0", crlf);
    bench.verdict("and an untouched button raises nothing by itself",
                  !spontaneous);

    (void)ExtInt<PadButton>::arm(false);
    (void)Exti::release(13);
    PadButton::input(PinPull::none);
    quiet_everything();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf,
          "test_stm32_exti - STM32G0B1RE EXTI (RM0444 ch. 13): the pin senses "
          "GPIO does not have, wireless, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler
// is a spin loop - so all three EXTI vectors are bound, whether or not a
// letter is using them.
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void EXTI0_1_IRQHandler() { serve(EXTI0_1_IRQn, 1); }

extern "C" void EXTI2_3_IRQHandler() { serve(EXTI2_3_IRQn, 2); }

/// The twelve-line vector, and the one handler that has two jobs:
/// letter h wants the edge POSTED to an active object, every other
/// letter wants it counted. `kernel_mode` is what says which - raised
/// only around letter h's own loop, so nothing else posts into a queue
/// nobody is pumping.
extern "C" void EXTI4_15_IRQHandler() {
    if (kernel_mode) {
        const brio::ExtiPending p =
            brio::Exti::isr(brio::Exti::vector_lines(EXTI4_15_IRQn));
        for (uint8_t l = 4; l < 16; ++l) {
            const uint32_t b = static_cast<uint32_t>(1u) << l;
            if ((p.rising & b) != 0u) {
                brio::post<Counter>(Edges{l, true});
            }
            if ((p.falling & b) != 0u) {
                brio::post<Counter>(Edges{l, false});
            }
        }
        return;
    }
    serve(EXTI4_15_IRQn, 3);
}

int main() {
    // Sampled BEFORE anything can disturb them: letter a judges the
    // EXTI's reset values, and every verb of this suite writes some of
    // these registers.
    boot_imr1 = brio::Exti::regs().IMR1;
    boot_imr2 = brio::Exti::regs().IMR2;
    boot_emr1 = brio::Exti::regs().EMR1;
    boot_rtsr1 = brio::Exti::regs().RTSR1;
    boot_exticr0 = brio::Exti::regs().EXTICR[0];

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the block: geometry, reset values, vectors, refusals",
                 ta_block);
    bench.letter('b', "the stimulus: a pad walked by its own pull, and driven",
                 tb_stimulus);
    bench.letter('c', "the four senses counted, and the two pending registers",
                 tc_senses);
    bench.letter('d', "the software trigger through the real vector", td_software);
    bench.letter('e', "the multiplexer: one line, two ports", te_mux);
    bench.letter('f', "the three shared vectors and their confinement",
                 tf_vectors);
    bench.letter('g', "the masks: a masked line, and an event out of WFE",
                 tg_masks);
    bench.letter('h', "ExtInt inside a real kernel", th_kernel);
    bench.letter('u', "the user button as a plain input", tu_button);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=",
                    clock_ok ? "PLL64" : "FAILED", " tick=",
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
