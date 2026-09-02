/*
 * exti.hpp
 *
 * The STM32G0's Extended interrupt and event controller (RM0444 ch. 13):
 * the peripheral where THIS family keeps its pin interrupts. There are
 * none in GPIO - stm32g0/pin.hpp says so and stops there - so everything
 * the AVR spells in PINnCTRL.ISC, and everything the SAM keeps in its
 * EIC, lives here.
 *
 *  Exti           the block: the trigger selection, the software
 *                 trigger, the two pending registers, the interrupt and
 *                 event masks, the GPIO port multiplexer, and the ISR
 *                 body the three shared vectors call.
 *
 *  ExtInt<Pin>    one GPIO line reached through the pad that carries
 *                 it: the line number IS the pin number, and the port
 *                 is what the multiplexer has to be told.
 *
 * SIX FACTS THAT SHAPE THE FILE.
 *
 * 1. THE LINE NUMBER IS THE PIN NUMBER, AND THE PORT IS A CHOICE. PA3,
 *    PB3, PC3, PD3, PE3 and PF3 all reach line 3 and EXTI_EXTICR1's
 *    third field says which one of them does (13.3.3, figure 29). So
 *    unlike the SAM's irregular pad-to-EXTINT map there is no table to
 *    read - and unlike it, THE SIXTEEN LINES ARE A SCARCE RESOURCE
 *    SHARED ACROSS SIX PORTS: PA3 and PB3 cannot both raise interrupts,
 *    ever. The last write to EXTICR wins and nothing in the silicon
 *    warns; `exti_lines_distinct<...>()` is the compile-time check an
 *    application can put on ITS OWN set of lines, and `selected()` is
 *    the run-time readback of who owns one now.
 *
 * 2. THERE IS NO LEVEL SENSE. The configurable lines are EDGE triggered
 *    and nothing else (13.5.1's own note: "the configurable lines are
 *    edge triggered, no glitch must be generated on these inputs"), so
 *    `ExtiSense` has four values where the AVR's PinSense has six and
 *    the SAM's EicSense has six. A level-triggered input is the
 *    application's to build - sense both edges and read the pad.
 *
 * 3. RISING AND FALLING ARE TWO SEPARATE PENDING BITS, in two separate
 *    registers (EXTI_RPR1 and EXTI_FPR1, 13.5.4/13.5.5), both W1C. A
 *    both-edges line therefore says WHICH edge arrived without reading
 *    the pad - the AVR and the SAM each have one flag per line and
 *    cannot. That is why the ISR body returns a PAIR of masks.
 *
 * 4. THE PENDING BIT IS ONLY SET FOR AN UNMASKED INTERRUPT. 13.3.1 says
 *    it in those words and 13.4 repeats it: with EXTI_IMR clear, an
 *    edge on an enabled trigger leaves no trace in RPR/FPR at all - the
 *    "flag standing while the interrupt is masked" that both other brio
 *    targets offer does not exist here, and neither does polling a line
 *    without arming it. Measured, and it is exactly what the chapter
 *    says (docs/stm32g0/exti.md).
 *
 * 5. THREE VECTORS FOR SIXTEEN LINES: EXTI0_1 serves lines 0 and 1,
 *    EXTI2_3 lines 2 and 3, EXTI4_15 the other twelve (table 61). A
 *    handler is therefore always a dispatcher, and `Exti::isr(lines)`
 *    takes the mask of lines its vector answers for - which
 *    `exti_vector_lines()` gives - so a handler cannot accidentally
 *    consume another vector's pending bits.
 *
 * 6. THE BLOCK HAS NO CLOCK GATE AND NO RESET. It sits on the AHB and
 *    runs on hclk; there is no RCC_AHBENR.EXTIEN and no EXTIRST in any
 *    device header of the family, so nothing here has to be turned on
 *    before it is used - the opposite of GPIO, whose port clock every
 *    configuring verb in pin.hpp must open first. The edge detection
 *    itself is ASYNCHRONOUS (13.3.1), which is why it works with the
 *    clock stopped and is what makes an EXTI line a wake-up source.
 *
 * WHAT THIS DRIVER OWNS, AND WHAT IT DOES NOT. It owns the FABRIC -
 * lines, triggers, pending bits, masks, the port multiplexer, the
 * vectors - and NOT the vocabulary of what is wired to a line above 15.
 * Table 65 lists the PVD on 16, the comparators on 17/18/20, the RTC on
 * 19, sixteen peripheral wake-ups on 21..36 - and that list is PER
 * PART: the G031 implements neither 20 nor 22 nor 24, the G071 neither
 * 20 nor 22, and a line number that means "USART3 wake-up" on one part
 * means nothing on another. So a peripheral driver that owns a wake-up
 * publishes ITS OWN line number, exactly as this stratum's peripherals
 * will publish their DMAMUX requests and as samc/'s publish their EVSYS
 * codes; `exti_line_implemented()` and `exti_line_configurable()` are
 * how such a number is checked against the device header. What lives
 * here is what is uniform: the sixteen GPIO lines, which are lines
 * 0..15 on every part of the family.
 *
 * DIRECT LINES, for the record (13.3.2, table 64): a direct line has no
 * trigger selection, no software trigger and NO PENDING BIT IN THE EXTI
 * at all - the flag to clear is the peripheral's own, the interrupt
 * that reaches the CPU is the peripheral's own, and all the EXTI does
 * is wake the system up. IMR/EMR are the only registers of this file
 * that touch one, which is why every mask verb here is a
 * read-modify-write of ONE bit: EXTI_IMR1 comes out of reset at
 * 0xFFF80000 with every direct line's interrupt already unmasked
 * (13.5.12), and a driver that wrote the whole register would silently
 * turn off a wake-up somebody else is relying on.
 *
 * WAKE-UP is stated here and NOT built: an EXTI line with its IMR or
 * EMR bit set wakes the CPU sub-system and, through PWR, the system
 * clocks (13.2.1, 13.3.1). What that costs and how the clocks come back
 * is chapter 4's, and there is no PWR driver or SleepSite in this
 * stratum yet - so the only sleep this file has anything to say about
 * is the one the core can enter unaided: WFE, which returns on an EXTI
 * CPU EVENT (an EMR bit set, no interrupt, no pending bit, nothing to
 * clear - 4.2.2's "Configuring a EXTI line in event mode"). That half
 * is measured; Stop and Standby wait for the power pass.
 *
 * ERRATA, ES0548 Rev 3 read on the bench chip's REVISION Z column:
 * nothing in the sheet touches the EXTI itself. The one item this
 * driver's users must know is 2.3.1, GPIO after a Standby wake-up (a
 * pad configured before Standby may not be restored as expected), and
 * it is unreachable here because nothing in this stratum enters
 * Standby - stated in docs/stm32g0/exti.md rather than coded, since
 * there is nothing to code against.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"

namespace brio {

// =============================================================================
// Vocabulary
// =============================================================================

/**
 * Which edge of a configurable line triggers it: EXTI_RTSR and
 * EXTI_FTSR, one bit each, and `both` is simply both bits (13.5.1's
 * note says so explicitly - "rising edge trigger can be set for a line
 * with falling edge trigger enabled").
 *
 * THERE IS NO LEVEL HERE, unlike avrdx/pin.hpp's PinSense and the SAM's
 * EicSense: this controller detects edges and nothing else. `none`
 * disables the line's detection altogether, which is also its reset
 * state.
 */
enum class ExtiSense : uint8_t { none = 0, rising = 1, falling = 2, both = 3 };

constexpr bool exti_sense_has_rising(ExtiSense s) {
    return (static_cast<uint8_t>(s) & static_cast<uint8_t>(ExtiSense::rising)) != 0u;
}
constexpr bool exti_sense_has_falling(ExtiSense s) {
    return (static_cast<uint8_t>(s) & static_cast<uint8_t>(ExtiSense::falling)) != 0u;
}

/**
 * What one vector's worth of pending bits looks like: the rising and
 * falling halves separately, because the silicon keeps them apart
 * (13.5.4, 13.5.5). Bit x of either mask is line x.
 */
struct ExtiPending {
    uint32_t rising = 0;
    uint32_t falling = 0;

    /// The lines that fired at all, whichever edge did it.
    constexpr uint32_t lines() const { return rising | falling; }
    constexpr bool any() const { return lines() != 0u; }
};

// =============================================================================
// The block
// =============================================================================

class Exti {
public:
    Exti() = delete;

    /// Sixteen on every part of the family, and always lines 0..15
    /// (stm32g0/device_tables.hpp reads the count off EXTICR's own
    /// geometry).
    static constexpr uint8_t gpio_lines = exti_gpio_lines;

    static EXTI_TypeDef& regs() { return *EXTI; }

    /// Does this device have line `n` at all (any kind)?
    static constexpr bool implemented(uint8_t line) { return exti_line_implemented(line); }
    /// Is it a CONFIGURABLE line - trigger selection, software trigger,
    /// pending bits - rather than a direct one?
    static constexpr bool configurable(uint8_t line) { return exti_line_configurable(line); }
    /// Is it one of the GPIO lines, the only ones with a multiplexer?
    static constexpr bool gpio(uint8_t line) { return line < gpio_lines; }

    /// The NVIC line a GPIO line interrupts on, and the lines that
    /// vector answers for. Both live in the reserve (IRQn values are
    /// enumerators the preprocessor cannot probe).
    static constexpr IRQn_Type irq(uint8_t line) { return exti_gpio_irq(line); }
    static constexpr uint32_t vector_lines(IRQn_Type v) { return exti_vector_lines(v); }

    // ---- the trigger selection (13.5.1, 13.5.2) ----------------------------

    /**
     * Enable this line's rising and/or falling edge. Refuses a line the
     * device does not implement or that is DIRECT (a direct line has no
     * trigger bits at all - it is enabled in its own peripheral).
     *
     * The two registers are written as one configuration, because the
     * pair IS the sense: setting them separately is how a line spends a
     * moment sensing an edge nobody asked for.
     */
    static bool sense(uint8_t line, ExtiSense s) {
        volatile uint32_t* const rt = rtsr(line);
        volatile uint32_t* const ft = ftsr(line);
        if (!configurable(line) || rt == nullptr || ft == nullptr) {
            return false;
        }
        const uint32_t b = bit(line);
        *rt = exti_sense_has_rising(s) ? (*rt | b) : (*rt & ~b);
        *ft = exti_sense_has_falling(s) ? (*ft | b) : (*ft & ~b);
        return true;
    }

    /// Read the pair back out of the silicon.
    static ExtiSense sense(uint8_t line) {
        const volatile uint32_t* const rt = rtsr(line);
        const volatile uint32_t* const ft = ftsr(line);
        if (!configurable(line) || rt == nullptr || ft == nullptr) {
            return ExtiSense::none;
        }
        const uint32_t b = bit(line);
        const uint8_t v = static_cast<uint8_t>(((*rt & b) != 0u ? 1u : 0u) |
                                               ((*ft & b) != 0u ? 2u : 0u));
        return static_cast<ExtiSense>(v);
    }

    // ---- the software trigger (13.5.3) -------------------------------------

    /**
     * EXTI_SWIER: raise a rising-edge event on a configurable line by
     * hand, "independently of EXTI_RTSR and EXTI_FTSR". The bit is
     * cleared by the hardware and reads back as zero, so this is a
     * plain store and there is nothing to poll.
     *
     * It is a RISING edge and only a rising one; there is no software
     * falling edge in this controller.
     */
    static bool trigger(uint8_t line) {
        volatile uint32_t* const sw = swier(line);
        if (!configurable(line) || sw == nullptr) {
            return false;
        }
        *sw = bit(line);
        return true;
    }

    // ---- pending (13.5.4, 13.5.5) ------------------------------------------
    //
    // Both registers are rc_w1: a plain store of the bits to clear, no
    // read-modify-write, and reading them costs nothing.

    static uint32_t rising_pending() { return regs().RPR1; }
    static uint32_t falling_pending() { return regs().FPR1; }
    static void clear_rising(uint32_t mask) { regs().RPR1 = mask; }
    static void clear_falling(uint32_t mask) { regs().FPR1 = mask; }

    static bool rising_pending(uint8_t line) {
        const volatile uint32_t* const r = rpr(line);
        return r != nullptr && configurable(line) && (*r & bit(line)) != 0u;
    }
    static bool falling_pending(uint8_t line) {
        const volatile uint32_t* const f = fpr(line);
        return f != nullptr && configurable(line) && (*f & bit(line)) != 0u;
    }
    static bool pending(uint8_t line) {
        return rising_pending(line) || falling_pending(line);
    }

    /// Clear both edges' pending bits of one line, whatever group it
    /// lives in.
    static bool clear(uint8_t line) {
        volatile uint32_t* const r = rpr(line);
        volatile uint32_t* const f = fpr(line);
        if (!configurable(line) || r == nullptr || f == nullptr) {
            return false;
        }
        const uint32_t b = bit(line);
        *r = b;
        *f = b;
        return true;
    }

    // ---- the two masks (13.5.12 .. 13.5.15) --------------------------------
    //
    // ONE BIT AT A TIME, ALWAYS. IMR1 comes out of reset at 0xFFF80000:
    // every DIRECT line's interrupt is already unmasked there, because
    // that is how a peripheral's own interrupt reaches the CPU at all.
    // A verb that wrote the whole register would turn those off behind
    // the owner's back, so nothing in this file ever does.

    /// EXTI_IMR: this line's edge raises a CPU interrupt (and wakes the
    /// CPU sub-system). It is also what makes the line's PENDING bit
    /// appear at all - see the file header, fact 4.
    static bool interrupt(uint8_t line, bool on) { return mask_bit(imr(line), line, on); }
    static bool interrupt(uint8_t line) { return mask_read(imr(line), line); }

    /// EXTI_EMR: this line's edge raises a CPU EVENT - it returns the
    /// core from WFE with no handler, no NVIC and no pending bit to
    /// clear (13.4, 4.2.2). Interrupt and event are independent: a line
    /// can have both, either or neither.
    static bool event(uint8_t line, bool on) { return mask_bit(emr(line), line, on); }
    static bool event(uint8_t line) { return mask_read(emr(line), line); }

    // ---- the GPIO multiplexer (13.3.3, 13.5.11) ----------------------------

    /**
     * Point line `line` at port `port` - the ONE choice the sixteen GPIO
     * lines offer, and the reason two pads with the same pin number
     * cannot both raise interrupts. Refuses a line that is not a GPIO
     * line and a port this device does not bond.
     *
     * There is no arbitration in the silicon and none here: the last
     * writer owns the line. `selected()` reads back who that is.
     */
    static bool select(uint8_t line, char port) {
        const uint8_t code = exti_port_code(port);
        if (!gpio(line) || code == 0xFFu) {
            return false;
        }
        const uint8_t reg = static_cast<uint8_t>(line / 4u);
        const uint32_t shift = static_cast<uint32_t>(line % 4u) * 8u;
        regs().EXTICR[reg] = (regs().EXTICR[reg] & ~(0xFFu << shift)) |
                             (static_cast<uint32_t>(code) << shift);
        return true;
    }

    /// Which port currently feeds this line, as a letter; 0 for a line
    /// that is not a GPIO line or whose field holds a code this device
    /// has no port for.
    static char selected(uint8_t line) {
        if (!gpio(line)) {
            return 0;
        }
        const uint8_t reg = static_cast<uint8_t>(line / 4u);
        const uint32_t shift = static_cast<uint32_t>(line % 4u) * 8u;
        const uint8_t code = static_cast<uint8_t>((regs().EXTICR[reg] >> shift) & 0xFFu);
        const char letter = static_cast<char>('A' + code);
        return exti_port_code(letter) == code ? letter : static_cast<char>(0);
    }

    // ---- the ISR body ------------------------------------------------------

    /**
     * The body of one of the three EXTI vectors. `lines` is the mask of
     * lines THIS vector answers for - `exti_vector_lines(EXTI2_3_IRQn)`
     * and friends - so a handler can neither consume nor be confused by
     * another vector's pending bits.
     *
     * Read-and-clear, both registers, and the two halves come back
     * apart: on a both-edges line the returned pair says which edge it
     * was without reading the pad, which is this controller's one real
     * advantage over the AVR's and the SAM's single flag.
     *
     * There is no mask to apply beyond `lines`: a pending bit only
     * exists for an unmasked interrupt in the first place (13.3.1), so
     * unlike the SERCOM's and the EIC's bodies this one has no
     * INTENSET-equivalent to AND with.
     */
    [[gnu::always_inline]] static ExtiPending isr(uint32_t lines) {
        ExtiPending p{regs().RPR1 & lines, regs().FPR1 & lines};
        if (p.rising != 0u) {
            regs().RPR1 = p.rising;
        }
        if (p.falling != 0u) {
            regs().FPR1 = p.falling;
        }
        return p;
    }

    // ---- teardown ----------------------------------------------------------

    /**
     * One line back to its reset state: no trigger, no interrupt, no
     * event, nothing pending. The multiplexer is left alone on purpose -
     * EXTICR's reset value is port A, and "give line 3 back to PA3" is
     * a claim and not a release.
     */
    static bool release(uint8_t line) {
        const bool was_configurable = configurable(line);
        (void)interrupt(line, false);
        (void)event(line, false);
        if (was_configurable) {
            (void)sense(line, ExtiSense::none);
            (void)clear(line);
        }
        return implemented(line);
    }

private:
    static constexpr uint32_t bit(uint8_t line) {
        return static_cast<uint32_t>(1u) << (line & 31u);
    }

    // The second register group is a struct-member question, answered
    // by the reserve with a pointer that is null where the register
    // does not exist on this part.
    static volatile uint32_t* rtsr(uint8_t line) {
        return line < 32u ? &regs().RTSR1 : exti_rtsr2();
    }
    static volatile uint32_t* ftsr(uint8_t line) {
        return line < 32u ? &regs().FTSR1 : exti_ftsr2();
    }
    static volatile uint32_t* swier(uint8_t line) {
        return line < 32u ? &regs().SWIER1 : exti_swier2();
    }
    static volatile uint32_t* rpr(uint8_t line) {
        return line < 32u ? &regs().RPR1 : exti_rpr2();
    }
    static volatile uint32_t* fpr(uint8_t line) {
        return line < 32u ? &regs().FPR1 : exti_fpr2();
    }
    static volatile uint32_t* imr(uint8_t line) {
        return line < 32u ? &regs().IMR1 : exti_imr2();
    }
    static volatile uint32_t* emr(uint8_t line) {
        return line < 32u ? &regs().EMR1 : exti_emr2();
    }

    static bool mask_bit(volatile uint32_t* reg, uint8_t line, bool on) {
        if (reg == nullptr || !implemented(line)) {
            return false;
        }
        const uint32_t b = bit(line);
        *reg = on ? (*reg | b) : (*reg & ~b);
        return true;
    }
    static bool mask_read(const volatile uint32_t* reg, uint8_t line) {
        return reg != nullptr && implemented(line) && (*reg & bit(line)) != 0u;
    }
};

// =============================================================================
// One GPIO line, reached through its pad
// =============================================================================

/**
 * The line a pad can raise, with the pin number for a line number and
 * the port letter for the multiplexer's code.
 *
 *   using Button = brio::Pin<'C', 13>;
 *   using ButtonInt = brio::ExtInt<Button>;      // line 13, port C
 *   ButtonInt::claim(brio::PinPull::up);         // input + pull + EXTICR
 *   ButtonInt::configure(brio::ExtiSense::falling);
 *   ButtonInt::arm(true);
 *   brio::Nvic::enable(ButtonInt::irq());        // EXTI4_15
 *
 * THE PAD KEEPS ITS GPIO MODE. This is not a peripheral multiplexer:
 * the EXTI watches the port's INPUT, which GPIO leaves live in input,
 * output and alternate modes alike (7.3.1), so a line can watch a pad
 * that an application is driving or that a peripheral owns - measured,
 * and the reason `claim()` and `select()` are separate verbs. Only
 * ANALOG mode, where the input buffer is off, hides a pad from its
 * line.
 */
template <class P>
struct ExtInt {
    ExtInt() = delete;

    static_assert(exti_port_code(P::port_letter) != 0xFFu,
                  "brio ExtInt: this device has no GPIO port of that letter, so "
                  "no EXTICR code selects it (ports A..D and F exist on every "
                  "STM32G0, E only on the G0B1/G0C1 class)");
    static_assert(P::pin_number < exti_gpio_lines,
                  "brio ExtInt: the EXTI has one line per PIN NUMBER and there "
                  "are sixteen of them");

    using pin = P;

    /// The line IS the pin number (13.3.3): no table, no formula to get
    /// wrong, and no per-package gate - every bonded pad of every port
    /// reaches the line of its own number.
    static constexpr uint8_t line = P::pin_number;
    static constexpr uint32_t mask = static_cast<uint32_t>(1u) << line;
    static constexpr char port = P::port_letter;

    static constexpr IRQn_Type irq() { return Exti::irq(line); }

    /// Point the line's multiplexer at THIS pad's port, leaving the
    /// pad's own mode alone - for a pad an application or a peripheral
    /// already owns.
    static bool select() { return Exti::select(line, port); }
    /// Is this pad's port the one the line listens to right now?
    static bool selected() { return Exti::selected(line) == port; }

    /// The common case: put the pad in input mode with `pull`, then
    /// point the line at its port.
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
    static bool rising_pending() { return Exti::rising_pending(line); }
    static bool falling_pending() { return Exti::falling_pending(line); }
    static bool pending() { return Exti::pending(line); }
    static bool clear() { return Exti::clear(line); }

    /// The line back to reset (the multiplexer left alone, see
    /// Exti::release) and the pad back to analog.
    static void release() {
        (void)Exti::release(line);
        P::release();
    }
};

/**
 * THE ONE-PIN-PER-LINE RULE AS A COMPILE-TIME CHECK. Sixteen lines are
 * shared by six ports, so an application that arms PA3 and PB3 has
 * written a bug the silicon will not report: the second EXTICR write
 * simply takes the line, and the first pad goes quiet.
 *
 * The driver cannot see an application's whole set of lines, but the
 * application can say what it is:
 *
 *   static_assert(brio::exti_lines_distinct<ButtonInt, SensorInt>(),
 *                 "two pads on one EXTI line");
 *
 * Nothing enforces that the assertion is written - it is the same kind
 * of claim as a peripheral's AF number, which no header on this family
 * can check either.
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
