/*
 * exti.hpp
 *
 * The STM32F4's external interrupt/event controller (RM0090 ch. 12,
 * RM0390 ch. 10, RM0383 ch. 10): the peripheral where THIS family keeps
 * its pin interrupts. There are none in GPIO - stm32f4/pin.hpp says so
 * and stops there - so every edge sense, every pending bit and every
 * wake-up line lives here.
 *
 *  Exti           the block: the trigger selection, the software trigger,
 *                 the one pending register, the interrupt and event
 *                 masks, the NVIC line each EXTI line reaches, and the
 *                 ISR body a vector calls.
 *
 *  ExtInt<Pin>    one GPIO line reached through the pad that carries it:
 *                 the line number IS the pin number, and the port is what
 *                 the multiplexer has to be told.
 *
 * EIGHT FACTS THAT SHAPE THE FILE.
 *
 * 1. THE LINE NUMBER IS THE PIN NUMBER, AND THE PORT IS A CHOICE. PA3,
 *    PB3 ... PK3 all reach line 3, and SYSCFG_EXTICR1's fourth nibble says
 *    which one of them does (RM0090 12.2.5, 9.2.3). So there is no
 *    pad-to-line table to read - and THE SIXTEEN LINES ARE A SCARCE
 *    RESOURCE SHARED ACROSS UP TO ELEVEN PORTS: PA5 and PB5 cannot both
 *    raise interrupts, ever. The silicon has no arbitration: the last
 *    EXTICR write simply takes the line and the previous pad goes quiet.
 *    THIS DRIVER REFUSES THAT WRITE - `select()` fails when the line is in
 *    use by another port (a sense selected, or an interrupt or an event
 *    unmasked), and `steal()` is the deliberate override that says so in
 *    its name. `exti_lines_distinct<...>()` is the compile-time check an
 *    application can put on ITS OWN set of lines, and `selected()` reads
 *    back who owns one now.
 *
 * 2. THE MULTIPLEXER IS ANOTHER PERIPHERAL'S REGISTER. Not the EXTI's, as
 *    on the STM32G0, but SYSCFG's - a block on APB2 behind a clock gate
 *    that is CLOSED out of reset (stm32f4/syscfg.hpp, which every verb
 *    here reaches the multiplexer through). With the gate shut an EXTICR
 *    write is dropped in silence, which is why the pad-level `claim()`
 *    opens it before anything else.
 *
 * 3. THERE IS NO LEVEL SENSE. The lines are EDGE triggered and nothing
 *    else (12.3.3's own note: "the external wake-up lines are edge
 *    triggered, no glitch must be generated on these lines"), so
 *    `ExtiSense` has four values and none of them is a level. A
 *    level-triggered input is the application's to build - sense both
 *    edges and read the pad.
 *
 * 4. ONE PENDING BIT PER LINE, FOR BOTH EDGES. EXTI_PR (12.3.6) is a
 *    single rc_w1 register: unlike the STM32G0's RPR/FPR pair, a
 *    both-edges line here does NOT say which edge arrived - the pad's
 *    level at handler time is the only clue, and it is a racy one. That is
 *    why `isr()` returns a plain mask of LINES and not a pair.
 *
 * 5. THE PENDING BIT IS ONLY SET FOR AN UNMASKED INTERRUPT. The chapter
 *    does not say so in words the way the STM32G0's does, but the block
 *    diagram (figure 41) puts the interrupt mask between the edge detector
 *    and the pending request register, and the bench agrees: with
 *    EXTI_IMR clear, edges on an enabled trigger leave PR at zero and
 *    unmasking afterwards does not resurrect them. SO A LINE CANNOT BE
 *    WATCHED WITHOUT BEING ARMED - what replaces the "poll the flag with
 *    the interrupt off" idiom is arming the EXTI's mask and leaving the
 *    NVIC line disabled. And when a line IS watched that way, the request
 *    reaches the NVIC and LATCHES THERE: clearing PR does not clear the
 *    NVIC's own pending bit, so `Nvic::clear_pending()` is the second half
 *    of any teardown that leaves a line armed with its vector shut.
 *
 * 6. EVERY IMPLEMENTED LINE IS CONFIGURABLE. The register maps carry bits
 *    22:0 (or 23:0) in RTSR, FTSR, SWIER and PR alike, so every line of
 *    this family has a trigger selection, a software trigger and a pending
 *    bit - there is no "direct line" class as on the STM32G0, and the
 *    peripheral wake-ups above 15 are edge-detected lines like the pins.
 *
 * 7. THE SOFTWARE TRIGGER IS NOT SELF-CLEARING, AND IT OBEYS THE MASK.
 *    SWIER's bit "is cleared by clearing the corresponding bit in EXTI_PR"
 *    (12.3.5) - so a second software trigger on a line needs the first one
 *    acknowledged, and `trigger()` is a read-modify-write of one bit
 *    rather than the plain store the STM32G0's self-clearing SWIER allows.
 *    12.3.5 also makes the flag conditional on IMR in so many words, and
 *    the bench agrees: a trigger with the interrupt masked raises nothing
 *    and leaves its SWIER bit standing, so a line is cleared before it is
 *    armed.
 *
 * 8. THE BLOCK HAS NO CLOCK GATE AND NO RESET OF ITS OWN. The EXTI is on
 *    APB2 and no device header of the family declares an EXTIEN or an
 *    EXTIRST, so nothing here has to be turned on - only SYSCFG, next
 *    door, does. The edge detection is asynchronous, which is what makes
 *    an EXTI line a wake-up source with the clocks stopped.
 *
 * WHAT THIS DRIVER OWNS, AND WHAT IT DOES NOT. It owns the FABRIC - lines,
 * triggers, the pending bits, the masks, the multiplexer, the vectors -
 * and NOT the vocabulary of what is wired to a line above 15. That list is
 * PER PART (RM0090 12.2.5 has seven entries, RM0390 six - no Ethernet -
 * and RM0383 five - no Ethernet and no USB OTG HS), so the reserve builds
 * the implemented mask from the PERIPHERALS' presence and a peripheral
 * driver that owns a wake-up publishes its own line number.
 * `Exti::implemented()` is how such a number is checked against the
 * device.
 *
 * WAKE-UP is stated here and NOT built: an EXTI line with its IMR or EMR
 * bit set wakes the CPU sub-system and, through PWR, the system clocks
 * (12.2.3). What that costs and how the clocks come back is the power
 * chapter's, and there is no SleepSite in this stratum yet - so the only
 * sleep this file has anything to say about is the one the core can enter
 * unaided: WFE, which returns on an EXTI CPU EVENT (an EMR bit set, no
 * interrupt, no NVIC line, no pending bit and nothing to clear).
 *
 * ERRATA. No item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is filed
 * against the EXTI or SYSCFG. One item filed under the RTC is about this
 * peripheral all the same, and it applies to every revision of all three
 * parts (ES0206 2.9.3, ES0298 2.10.3, ES0287 2.8.3, "RTC interrupt can be
 * masked by another RTC interrupt"): THE EFFECTIVE CLEAR OF A PENDING BIT
 * IS DELAYED with respect to the store that clears it. A handler that
 * clears its line last therefore returns while the request still stands
 * and is entered a second time - which is why `isr()` reads AND clears at
 * the top, before the handler does anything with what it read.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/syscfg.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/**
 * Which edge triggers a line: EXTI_RTSR and EXTI_FTSR, one bit each, and
 * `both` is simply both bits (12.3.3's note says so explicitly - "rising
 * and falling edge triggers can be set for the same interrupt line").
 *
 * THERE IS NO LEVEL HERE: this controller detects edges and nothing else.
 * `none` disables the line's detection altogether, which is also its reset
 * state.
 */
enum class ExtiSense : uint8_t { none = 0, rising = 1, falling = 2, both = 3 };

constexpr bool exti_sense_has_rising(ExtiSense s) {
    return (static_cast<uint8_t>(s) & static_cast<uint8_t>(ExtiSense::rising)) != 0u;
}
constexpr bool exti_sense_has_falling(ExtiSense s) {
    return (static_cast<uint8_t>(s) & static_cast<uint8_t>(ExtiSense::falling)) != 0u;
}

// =============================================================================
// The block
// =============================================================================

class Exti {
public:
    Exti() = delete;

    /// Sixteen pin lines on every part of the family, always lines 0..15.
    static constexpr uint8_t gpio_lines = exti_gpio_lines;

    /// Every line this device has, pins and peripheral wake-ups alike.
    static constexpr uint32_t implemented_mask = exti_implemented_mask();

    static EXTI_TypeDef& regs() { return *EXTI; }

    /// Does this device have line `n`? The peripheral wired to a line
    /// above 15 is what decides (stm32f4/device_tables.hpp).
    static constexpr bool implemented(uint8_t line) { return exti_line_implemented(line); }
    /// Is it one of the pin lines, the only ones with a multiplexer?
    static constexpr bool gpio(uint8_t line) { return line < gpio_lines; }

    /// The NVIC line this EXTI line interrupts on, and the lines that
    /// vector answers for. Both live in the reserve (IRQn values are
    /// enumerators the preprocessor cannot probe).
    static constexpr IRQn_Type irq(uint8_t line) { return exti_line_irq(line); }
    static constexpr uint32_t vector_lines(IRQn_Type v) { return exti_vector_lines(v); }

    // ---- the trigger selection (12.3.3, 12.3.4) ----------------------------

    /**
     * Enable this line's rising and/or falling edge. Refuses a line the
     * device does not implement.
     *
     * The two registers are written as one configuration, because the pair
     * IS the sense: setting them separately is how a line spends a moment
     * sensing an edge nobody asked for. 12.3.3's note is worth knowing when
     * a line is armed live - a RISING edge that arrives while RTSR is being
     * written DOES set the pending bit, a falling one during an FTSR write
     * does not - so a line armed under an already-moving pad can start with
     * a flag standing, and clear() before arm() is the discipline.
     */
    static bool sense(uint8_t line, ExtiSense s) {
        if (!implemented(line)) {
            return false;
        }
        const uint32_t b = bit(line);
        regs().RTSR = exti_sense_has_rising(s) ? (regs().RTSR | b) : (regs().RTSR & ~b);
        regs().FTSR = exti_sense_has_falling(s) ? (regs().FTSR | b) : (regs().FTSR & ~b);
        return true;
    }

    /// Read the pair back out of the silicon.
    static ExtiSense sense(uint8_t line) {
        if (!implemented(line)) {
            return ExtiSense::none;
        }
        const uint32_t b = bit(line);
        const uint8_t v = static_cast<uint8_t>(((regs().RTSR & b) != 0u ? 1u : 0u) |
                                               ((regs().FTSR & b) != 0u ? 2u : 0u));
        return static_cast<ExtiSense>(v);
    }

    // ---- the software trigger (12.3.5) -------------------------------------

    /**
     * EXTI_SWIER: raise an edge on a line by hand, with no pad and no
     * regard for RTSR/FTSR - the line needs no trigger selected at all.
     *
     * TWO THINGS THE STM32G0'S SELF-CLEARING SWIER DOES NOT ASK FOR. The
     * bit is set until the line's PENDING bit is cleared (12.3.5), so a
     * second trigger needs the first acknowledged - `clear(line)` then
     * `trigger(line)`; and because the bit stands, this is a
     * read-modify-write of one bit and not a plain store, so a trigger from
     * an interrupt racing one in the loop can be lost.
     */
    static bool trigger(uint8_t line) {
        if (!implemented(line)) {
            return false;
        }
        regs().SWIER |= bit(line);
        return true;
    }

    /// Is a software trigger of this line still unacknowledged? (SWIER
    /// reads back what was written until PR is cleared.)
    static bool triggered(uint8_t line) {
        return implemented(line) && (regs().SWIER & bit(line)) != 0u;
    }

    // ---- pending (12.3.6) --------------------------------------------------
    //
    // ONE register for both edges, rc_w1: a plain store of the bits to
    // clear, never a read-modify-write - writing zeros is what a
    // read-modify-write would do to every OTHER line's flag, and on an rc_w1
    // register a zero is exactly the "leave it alone" value.

    static uint32_t pending() { return regs().PR; }
    static bool pending(uint8_t line) {
        return implemented(line) && (regs().PR & bit(line)) != 0u;
    }
    static void clear_lines(uint32_t mask) { regs().PR = mask; }
    static bool clear(uint8_t line) {
        if (!implemented(line)) {
            return false;
        }
        regs().PR = bit(line);
        return true;
    }

    // ---- the two masks (12.3.1, 12.3.2) ------------------------------------
    //
    // Both come out of reset at zero on this family - nothing is unmasked
    // behind an application's back - but the verbs still write ONE BIT at a
    // time, because the register is shared with every other line and with
    // whatever peripheral owns a wake-up above 15.

    /// EXTI_IMR: this line's edge raises a CPU interrupt (and wakes the CPU
    /// sub-system). It is also what makes the line's PENDING bit appear at
    /// all - see the file header, fact 5 - so a line cannot be watched
    /// without being armed.
    static bool interrupt(uint8_t line, bool on) { return mask_bit(regs().IMR, line, on); }
    static bool interrupt(uint8_t line) { return mask_read(regs().IMR, line); }

    /// EXTI_EMR: this line's edge raises a CPU EVENT - it returns the core
    /// from WFE with no handler, no NVIC and no pending bit to clear
    /// (12.2.3, 12.2.4). Interrupt and event are independent: a line can
    /// have both, either or neither.
    static bool event(uint8_t line, bool on) { return mask_bit(regs().EMR, line, on); }
    static bool event(uint8_t line) { return mask_read(regs().EMR, line); }

    // ---- the GPIO multiplexer (SYSCFG_EXTICR, 9.2.3) -----------------------

    /**
     * Point line `line` at port `port` - the ONE choice the sixteen pin
     * lines offer - REFUSING to take a line another port is using, since
     * the silicon will not: the write would land, the line would change
     * hands and the pad that had it would simply stop raising edges.
     *
     * "In use" is what this driver can see of a claim: a sense selected, or
     * the interrupt or the event unmasked. A line merely POINTED at another
     * port and otherwise untouched is free, which is what makes EXTICR's
     * reset value (port A on every line) not a claim by port A.
     */
    static bool select(uint8_t line, char port) {
        if (!gpio(line) || exti_port_code(port) == 0xFFu) {
            return false;
        }
        if (in_use(line) && Syscfg::exti_source(line) != port) {
            return false;
        }
        return Syscfg::exti_source(line, port);
    }

    /// Take the line whatever it was doing - the deliberate override of
    /// select()'s refusal, for a program that is reconfiguring its own
    /// lines and knows the pad it is taking them from.
    static bool steal(uint8_t line, char port) { return Syscfg::exti_source(line, port); }

    /// Which port currently feeds this line, as a letter; 0 for a line that
    /// is not a GPIO line or whose field holds a code this device has no
    /// port for.
    static char selected(uint8_t line) { return Syscfg::exti_source(line); }

    /// Is anything relying on this line right now - a sense, an interrupt
    /// or an event? What select() refuses to walk over.
    static bool in_use(uint8_t line) {
        return implemented(line) &&
               (sense(line) != ExtiSense::none || interrupt(line) || event(line));
    }

    // ---- the ISR body ------------------------------------------------------

    /**
     * The body of one EXTI vector. `lines` is the mask of lines THIS vector
     * answers for - `Exti::vector_lines(EXTI9_5_IRQn)` and friends - so a
     * handler can neither consume nor be confused by another vector's
     * pending bits. The returned mask is the lines that fired; `served()`
     * asks it about one of them.
     *
     * READ AND CLEAR AT THE TOP, ALWAYS. The pending bit's clear takes
     * effect a little after the store that asks for it (the errata item in
     * the file header), so a handler that clears last returns while the
     * request still stands and the NVIC re-enters it. Clearing first costs
     * nothing and closes that door; the edge that arrives between the read
     * and the store is not lost either, because it sets the bit again after
     * the clear lands.
     */
    [[gnu::always_inline]] static uint32_t isr(uint32_t lines) {
        const uint32_t fired = regs().PR & lines;
        if (fired != 0u) {
            regs().PR = fired;
        }
        return fired;
    }

    /// Did line `line` fire in the mask a handler got from isr()?
    static constexpr bool served(uint32_t fired, uint8_t line) {
        return line < 32u && (fired & (1UL << line)) != 0u;
    }

    // ---- teardown ----------------------------------------------------------

    /**
     * One line back to its reset state: no trigger, no interrupt, no event,
     * nothing pending. The multiplexer is left alone on purpose - EXTICR's
     * reset value is port A, and "give line 3 back to PA3" is a claim and
     * not a release.
     */
    static bool release(uint8_t line) {
        if (!implemented(line)) {
            return false;
        }
        (void)interrupt(line, false);
        (void)event(line, false);
        (void)sense(line, ExtiSense::none);
        (void)clear(line);
        return true;
    }

private:
    static constexpr uint32_t bit(uint8_t line) { return 1UL << (line & 31u); }

    static bool mask_bit(volatile uint32_t& reg, uint8_t line, bool on) {
        if (!implemented(line)) {
            return false;
        }
        const uint32_t b = bit(line);
        reg = on ? (reg | b) : (reg & ~b);
        return true;
    }
    static bool mask_read(const volatile uint32_t& reg, uint8_t line) {
        return implemented(line) && (reg & bit(line)) != 0u;
    }
};

// =============================================================================
// One line named at compile time
// =============================================================================

/**
 * A line the program names as a CONSTANT - which is what a peripheral
 * wake-up above 15 always is, since a driver publishes its number
 * (`Rtc::exti_line`, and the like) and an application spells it once:
 *
 *   using RtcWake = brio::ExtiLine<22>;          // RTC wake-up
 *   RtcWake::configure(brio::ExtiSense::rising);
 *   RtcWake::arm(true);
 *   brio::Nvic::enable(RtcWake::irq());          // RTC_WKUP
 *
 * The point of the type is the static_assert: a line the part has not got
 * - the Ethernet wake-up on a part with no MAC - is a COMPILE ERROR here,
 * where `Exti`'s run-time verbs can only answer false. Both faces exist
 * because both questions are real: a driver that walks a line number
 * computed at run time needs the second.
 */
template <uint8_t Line>
struct ExtiLine {
    ExtiLine() = delete;

    static_assert(Exti::implemented(Line),
                  "brio ExtiLine: this device does not implement that EXTI line - the "
                  "sixteen pin lines exist everywhere, and each line above them exists "
                  "only where the peripheral wired to it does (no Ethernet wake-up "
                  "without an Ethernet MAC, no USB OTG HS wake-up without that "
                  "controller)");

    static constexpr uint8_t line = Line;
    static constexpr uint32_t mask = 1UL << Line;

    static constexpr IRQn_Type irq() { return Exti::irq(Line); }

    static bool configure(ExtiSense s) { return Exti::sense(Line, s); }
    static ExtiSense sense() { return Exti::sense(Line); }
    static bool arm(bool on) { return Exti::interrupt(Line, on); }
    static bool armed() { return Exti::interrupt(Line); }
    static bool event(bool on) { return Exti::event(Line, on); }
    static bool event() { return Exti::event(Line); }
    static bool trigger() { return Exti::trigger(Line); }
    static bool triggered() { return Exti::triggered(Line); }
    static bool pending() { return Exti::pending(Line); }
    static bool clear() { return Exti::clear(Line); }
    static bool served(uint32_t fired) { return Exti::served(fired, Line); }
    static bool release() { return Exti::release(Line); }
};

// =============================================================================
// One GPIO line, reached through its pad
// =============================================================================

/**
 * The line a pad can raise, with the pin number for a line number and the
 * port letter for the multiplexer's code.
 *
 *   using Button = brio::Pin<'C', 13>;
 *   using ButtonInt = brio::ExtInt<Button>;      // line 13, port C
 *   ButtonInt::claim(brio::PinPull::up);         // input + pull + EXTICR
 *   ButtonInt::configure(brio::ExtiSense::falling);
 *   ButtonInt::arm(true);
 *   brio::Nvic::enable(ButtonInt::irq());        // EXTI15_10
 *
 * THE PAD KEEPS ITS GPIO MODE. This is not an alternate function: the EXTI
 * watches the port's INPUT, which GPIO leaves live in input, output and
 * alternate modes alike (RM0090 8.3.10, and measured), so a line can watch
 * a pad the application is DRIVING or one a peripheral owns - the reason
 * `claim()` and `select()` are separate verbs. Only ANALOG mode, where the
 * input buffer is off, hides a pad from its line.
 */
template <class P>
struct ExtInt {
    ExtInt() = delete;

    static_assert(exti_port_code(P::port_letter) != 0xFFu,
                  "brio ExtInt: this device has no GPIO port of that letter, so no "
                  "SYSCFG_EXTICR code selects it (ports A, B, C and H exist on every "
                  "STM32F4, the rest by bonding)");
    static_assert(P::pin_number < exti_gpio_lines,
                  "brio ExtInt: the EXTI has one line per PIN NUMBER and there are "
                  "sixteen of them");

    using pin = P;

    /// The line IS the pin number (12.2.5): no table, no formula to get
    /// wrong, and no per-package gate - every bonded pad of every port
    /// reaches the line of its own number.
    static constexpr uint8_t line = P::pin_number;
    static constexpr uint32_t mask = 1UL << line;
    static constexpr char port = P::port_letter;

    static constexpr IRQn_Type irq() { return Exti::irq(line); }

    /// Point the line's multiplexer at THIS pad's port, leaving the pad's
    /// own mode alone - for a pad an application or a peripheral already
    /// owns. Refuses a line another port is using (see Exti::select).
    static bool select() { return Exti::select(line, port); }
    /// Take the line from whoever has it.
    static bool steal() { return Exti::steal(line, port); }
    /// Is this pad's port the one the line listens to right now?
    static bool selected() { return Exti::selected(line) == port; }

    /// The common case: put the pad in input mode with `pull`, then point
    /// the line at its port.
    static bool claim(PinPull pull = PinPull::none) {
        P::input(pull);
        return select();
    }

    static bool configure(ExtiSense s) { return Exti::sense(line, s); }
    static ExtiSense sense() { return Exti::sense(line); }

    static bool arm(bool on) { return Exti::interrupt(line, on); }
    static bool armed() { return Exti::interrupt(line); }
    static bool event(bool on) { return Exti::event(line, on); }
    static bool event() { return Exti::event(line); }

    static bool trigger() { return Exti::trigger(line); }
    static bool triggered() { return Exti::triggered(line); }
    static bool pending() { return Exti::pending(line); }
    static bool clear() { return Exti::clear(line); }
    static bool served(uint32_t fired) { return Exti::served(fired, line); }

    /// The line back to reset (the multiplexer left alone, see
    /// Exti::release) and the pad back to analog.
    static void release() {
        (void)Exti::release(line);
        P::release();
    }
};

/**
 * THE ONE-PIN-PER-LINE RULE AS A COMPILE-TIME CHECK. Sixteen lines are
 * shared by up to eleven ports, so an application that arms PA3 and PB3 has
 * written a bug - `Exti::select()` refuses the second claim at run time,
 * but a refusal is an answer nobody has to read. The application can state
 * the whole set instead, and be told at compile time:
 *
 *   static_assert(brio::exti_lines_distinct<ButtonInt, SensorInt>(),
 *                 "two pads on one EXTI line");
 *
 * Nothing enforces that the assertion is written - it is the same kind of
 * claim as a peripheral's AF number, which no header of this family can
 * check either.
 */
template <class... Ints>
constexpr bool exti_lines_distinct() {
    uint32_t seen = 0;
    bool ok = true;
    ((ok = ok && (seen & Ints::mask) == 0u, seen |= Ints::mask), ...);
    (void)seen;   // an empty pack sets it and reads it nowhere
    return ok;
}

} // namespace brio
