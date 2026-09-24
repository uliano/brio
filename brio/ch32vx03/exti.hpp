/*
 * exti.hpp
 *
 * The external interrupt/event controller of the CH32V203 (RM 9.4, 9.5.1):
 * twenty-two lines, sixteen of them pins and the rest peripheral wake-ups,
 * with the edge detection, the two enables, the software trigger and one
 * flag apiece.
 *
 *   Exti           the block: the senses, the interrupt and event enables,
 *                  the software trigger, the flags, the multiplexer, the
 *                  vector each line reaches, and the ISR body a handler
 *                  calls.
 *   ExtiLine<n>    one line named as a constant - what a peripheral's
 *                  wake-up is - refused at compile time where this part
 *                  has no such line.
 *   ExtInt<Pin>    one line reached through the pad that raises it.
 *
 * SEVEN FACTS THAT SHAPE THE FILE.
 *
 * 1. THE LINE NUMBER IS THE PIN NUMBER, AND THE PORT IS A CHOICE. 10.2.3
 *    says it in so many words: PA1, PB1, PC1, PD1 and PE1 all reach EXTI1
 *    and "EXTI1 can only accept mappings from one of" them. The choice is
 *    AFIO_EXTICR's four bits per line (afio.hpp), so THE SIXTEEN LINES ARE
 *    A SCARCE RESOURCE SHARED ACROSS THE PORTS: PA5 and PB5 cannot both
 *    raise an edge. The silicon does not arbitrate - the last write takes
 *    the line and the pad that had it goes quiet - so `select()` REFUSES a
 *    line another port is using, `steal()` is the deliberate override that
 *    says so in its name, and `exti_lines_distinct<...>()` is the
 *    compile-time check an application puts on its own set.
 *
 * 2. THE MULTIPLEXER IS ANOTHER BLOCK'S REGISTER, behind a clock gate that
 *    is closed out of reset. Every verb here that touches it goes through
 *    afio.hpp, which opens the gate first; with the gate shut the write is
 *    dropped in silence.
 *
 * 3. THERE IS NO LEVEL SENSE, and no register for one: RTENR and FTENR are
 *    the whole of it, so `ExtiSense` has four values and none of them is a
 *    level. A line with neither edge enabled detects nothing - which is
 *    its reset state, and what `release()` puts it back to.
 *
 * 4. ONE FLAG PER LINE, FOR BOTH EDGES, WRITE-ONE-CLEAR. EXTI_INTFR is a
 *    single register with no edge in it, so a both-edges line does not say
 *    WHICH edge arrived: the pad's level at handler time is the only clue
 *    and it is a racy one. `isr()` therefore returns a mask of LINES. The
 *    flags are cleared by a plain store of the bits to clear, never a
 *    read-modify-write: on a write-one-clear register a zero is exactly
 *    the "leave it alone" value, and a read-modify-write would acknowledge
 *    every other line's edge.
 *
 * 5. A LINE ENABLED NOWHERE RAISES NO FLAG - measured, for a pad's edge and
 *    for the software trigger alike, though 9.5.1.5 says a trigger sets the
 *    flag whatever the enables hold. So a line cannot be watched without
 *    being armed: what replaces the "poll the flag with the interrupt off"
 *    idiom is INTENR set with the line's vector left shut at the PFIC, and
 *    the flag then stands until it is written one (measured).
 *
 * 6. THE SOFTWARE TRIGGER'S BIT STANDS until the flag is cleared - measured:
 *    after a write, SWIEVR reads its bit back, and clearing the line's flag
 *    clears it. That is the STM32F1's rule for this register, this one's
 *    ancestor, and NOT the sister family's, whose SWIEVR takes a plain
 *    store. `trigger()` is a read-modify-write for that reason, so a second
 *    trigger needs the first acknowledged and a trigger raised from an
 *    interrupt can be lost against one in the loop.
 *
 * 7. THE BLOCK HAS NO CLOCK GATE AND NO RESET OF ITS OWN. There is no
 *    EXTIEN bit in RCC_PB2PCENR (3.4.7) - the chapter says the registers
 *    are reached "through the PB2 interface" and that is all - so nothing
 *    here has to be turned on, and a line's edge detection is asynchronous,
 *    which is what makes it a wake-up source with the clocks stopped.
 *
 * THE WAKE-UP PATH, stated and half built. 9.4.2 gives two ways to end a
 * WFE: a line in EVENR, which needs no handler and leaves no flag; or a
 * line in INTENR with the vector masked at the PFIC and SEVONPEND set, in
 * which case the flag and the pending bit must both be cleared afterwards.
 * The platform's idle() is a WFE with SEVONPEND and WFITOWFE already set
 * (platform.hpp), so an armed line ends it either way. What a real SLEEP
 * costs and how the clocks come back is the power chapter's, and there is
 * no SleepSite in this stratum yet: this file provides the source and
 * measures only that the event latches.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pin.hpp"

namespace brio {

/// The block's six registers (RM 9.5.1), all with bits [21:0].
struct ExtiRegs {
    volatile uint32_t INTENR;   ///< 0x00 interrupt enable, one bit per line
    volatile uint32_t EVENR;    ///< 0x04 event enable
    volatile uint32_t RTENR;    ///< 0x08 rising-edge trigger enable
    volatile uint32_t FTENR;    ///< 0x0c falling-edge trigger enable
    volatile uint32_t SWIEVR;   ///< 0x10 software interrupt/event
    volatile uint32_t INTFR;    ///< 0x14 the flags, write one to clear
};

inline ExtiRegs* exti() { return reinterpret_cast<ExtiRegs*>(pb2_base + 0x0400); }

/**
 * Which edge raises a line: EXTI_RTENR and EXTI_FTENR, one bit each, and
 * `both` is simply both bits. `none` is the reset state, in which the
 * line detects nothing - a pad's edges included.
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

    /// The pin lines, 0..15 on every part: a line per pin NUMBER, its
    /// port chosen in the multiplexer.
    static constexpr uint8_t gpio_lines = 16;

    /// The lines above them, by what is wired to each (table 9-3). Which
    /// of them this part HAS follows the peripheral: the two USB
    /// controllers and the Ethernet are per part, line 21 is the
    /// CH32V20x_D8's 32 kHz calibration, and on the CH32V30x_D8, which
    /// has no USBD, the table gives line 18 to the USBFS/OTG controller
    /// as well as line 20.
    static constexpr uint8_t line_pvd = 16;
    static constexpr uint8_t line_rtc_alarm = 17;
    static constexpr uint8_t line_usbd_wakeup = 18;
    static constexpr uint8_t line_eth_wakeup = 19;
    static constexpr uint8_t line_usbfs_wakeup = 20;
    static constexpr uint8_t line_osc32k_wakeup = 21;
    static constexpr uint8_t line_count = 22;

    static ExtiRegs& regs() { return *exti(); }

    /// Does this part have line `n`? The peripheral wired to it decides,
    /// and the sixteen pin lines exist everywhere.
    static constexpr bool implemented(uint8_t line) {
        return line < gpio_lines || line == line_pvd || line == line_rtc_alarm ||
               (line == line_usbd_wakeup &&
                (device::has_usbd || device::device_class == DeviceClass::v30x_d8)) ||
               (line == line_eth_wakeup && device::has_ethernet) ||
               (line == line_usbfs_wakeup && device::has_usbfs) ||
               (line == line_osc32k_wakeup && device::device_class == DeviceClass::v20x_d8);
    }

    /// Every line this part has, as a mask.
    static constexpr uint32_t implemented_mask() {
        uint32_t m = 0;
        for (uint8_t line = 0; line < line_count; ++line) {
            if (implemented(line)) {
                m |= 1UL << line;
            }
        }
        return m;
    }

    /// Is it one of the pin lines - the only ones with a multiplexer?
    static constexpr bool gpio(uint8_t line) { return line < gpio_lines; }

    /// A vector as an optional: nothing where this part's device class
    /// has no such line (device.hpp's irq_none).
    static constexpr std::optional<Irq> vector_if_any(Irq v) {
        return irq_exists(v) ? std::optional<Irq>(v) : std::nullopt;
    }

    /**
     * The vector this line interrupts on: the first five lines have one
     * each, then two shared vectors carry 9..5 and 15..10, and the
     * peripheral wake-ups take the vector of the peripheral they belong
     * to (table 9-2).
     *
     * Nothing for a line this part has not got - and nothing for the two
     * the CH32V203RB adds, the Ethernet's wake-up and the 32 kHz
     * calibration's, which this driver does not arm. On the CH32V303 the
     * two USB wake-ups interrupt on 58 and 84 (device.hpp's Irq table:
     * measured, where WCH's own startup file for the class has zeros).
     */
    static constexpr std::optional<Irq> irq(uint8_t line) {
        if (!implemented(line)) {
            return std::nullopt;
        }
        switch (line) {
            case 0:  return Irq::exti0;
            case 1:  return Irq::exti1;
            case 2:  return Irq::exti2;
            case 3:  return Irq::exti3;
            case 4:  return Irq::exti4;
            case line_pvd: return Irq::pvd;
            case line_rtc_alarm: return Irq::rtc_alarm;
            case line_usbd_wakeup: return vector_if_any(Irq::usb_wakeup);
            case line_usbfs_wakeup: return vector_if_any(Irq::usbfs_wakeup);
            default: break;
        }
        if (line <= 9u) {
            return Irq::exti9_5;
        }
        if (line <= 15u) {
            return Irq::exti15_10;
        }
        return std::nullopt;
    }

    /// The lines one vector answers for - what a shared handler passes to
    /// isr() so it can neither consume nor be confused by another
    /// vector's flags. The two USB wake-ups are asked apart from the
    /// switch because a class without one of them states it as irq_none,
    /// and a switch cannot hold one value twice.
    static constexpr uint32_t vector_lines(Irq vector) {
        switch (vector) {
            case Irq::exti0: return 1UL << 0;
            case Irq::exti1: return 1UL << 1;
            case Irq::exti2: return 1UL << 2;
            case Irq::exti3: return 1UL << 3;
            case Irq::exti4: return 1UL << 4;
            case Irq::exti9_5: return 0x3E0UL;      // 9..5
            case Irq::exti15_10: return 0xFC00UL;   // 15..10
            case Irq::pvd: return 1UL << line_pvd;
            case Irq::rtc_alarm: return 1UL << line_rtc_alarm;
            default: break;
        }
        if (irq_exists(vector) && vector == Irq::usb_wakeup) {
            return 1UL << line_usbd_wakeup;
        }
        if (irq_exists(vector) && vector == Irq::usbfs_wakeup) {
            return 1UL << line_usbfs_wakeup;
        }
        return 0;
    }

    // ---- the senses (9.5.1.3, 9.5.1.4) -------------------------------------

    /**
     * Enable this line's rising and/or falling edge. The two registers
     * are written as one configuration, because the pair IS the sense:
     * setting them one at a time is how a line spends a moment sensing an
     * edge nobody asked for. False for a line this part has not got.
     */
    static bool sense(uint8_t line, ExtiSense s) {
        if (!implemented(line)) {
            return false;
        }
        const uint32_t b = bit(line);
        regs().RTENR = exti_sense_has_rising(s) ? (regs().RTENR | b) : (regs().RTENR & ~b);
        regs().FTENR = exti_sense_has_falling(s) ? (regs().FTENR | b) : (regs().FTENR & ~b);
        return true;
    }

    /// Read the pair back out of the silicon.
    static ExtiSense sense(uint8_t line) {
        if (!implemented(line)) {
            return ExtiSense::none;
        }
        const uint32_t b = bit(line);
        const uint8_t v = static_cast<uint8_t>(((regs().RTENR & b) != 0u ? 1u : 0u) |
                                               ((regs().FTENR & b) != 0u ? 2u : 0u));
        return static_cast<ExtiSense>(v);
    }

    // ---- the two enables (9.5.1.1, 9.5.1.2) --------------------------------

    /// EXTI_INTENR: this line's edge raises its flag and, through the
    /// PFIC, its vector.
    static bool interrupt(uint8_t line, bool on) { return mask_bit(regs().INTENR, line, on); }
    static bool interrupt(uint8_t line) { return mask_read(regs().INTENR, line); }

    /// EXTI_EVENR: this line's edge raises a WAKE EVENT instead - what
    /// ends a WFE with no handler run and no flag to clear (9.4.2).
    /// Interrupt and event are independent: a line can have both, either
    /// or neither.
    static bool event(uint8_t line, bool on) { return mask_bit(regs().EVENR, line, on); }
    static bool event(uint8_t line) { return mask_read(regs().EVENR, line); }

    // ---- the software trigger (9.5.1.5) ------------------------------------

    /// Raise a line by hand, with no pad and no regard for the senses.
    /// A read-modify-write of one bit: see the file header, fact 5.
    static bool trigger(uint8_t line) {
        if (!implemented(line)) {
            return false;
        }
        regs().SWIEVR |= bit(line);
        return true;
    }

    /// Does the trigger bit still stand? (What says whether this register
    /// self-clears on this silicon or holds until the flag is cleared.)
    static bool triggered(uint8_t line) {
        return implemented(line) && (regs().SWIEVR & bit(line)) != 0u;
    }

    // ---- the flags (9.5.1.6) -----------------------------------------------

    static uint32_t pending() { return regs().INTFR; }
    static bool pending(uint8_t line) {
        return implemented(line) && (regs().INTFR & bit(line)) != 0u;
    }
    /// Write-one-clear, as a plain store (see the file header, fact 4).
    static void clear_lines(uint32_t mask) { regs().INTFR = mask; }
    static bool clear(uint8_t line) {
        if (!implemented(line)) {
            return false;
        }
        regs().INTFR = bit(line);
        return true;
    }

    // ---- the multiplexer (AFIO_EXTICR, through afio.hpp) -------------------

    /**
     * Point line `line` at port `port` - the one choice a pin line offers
     * - REFUSING to take a line another port is using, since the silicon
     * will not: the write would land, the line would change hands and the
     * pad that had it would simply stop raising edges.
     *
     * "In use" is what this driver can see of a claim: a sense selected,
     * or the interrupt or the event enabled. A line merely POINTED at a
     * port and otherwise untouched is free, which is what makes EXTICR's
     * reset value - port A on every line - not a claim by port A.
     */
    static bool select(uint8_t line, char port) {
        if (!gpio(line) || exti_port_code(port) == 0xFFu) {
            return false;
        }
        if (in_use(line) && Afio::exti_source(line) != port) {
            return false;
        }
        return Afio::exti_source(line, port);
    }

    /// Take the line whatever it was doing - the deliberate override of
    /// select()'s refusal, for a program that is reconfiguring its own
    /// lines and knows the pad it takes them from.
    static bool steal(uint8_t line, char port) { return Afio::exti_source(line, port); }

    /// Which port feeds this line now, as a letter; 0 for a line that is
    /// not a pin line.
    static char selected(uint8_t line) { return gpio(line) ? Afio::exti_source(line) : '\0'; }

    /// Is anything relying on this line right now - a sense, an interrupt
    /// or an event? What select() refuses to walk over.
    static bool in_use(uint8_t line) {
        return implemented(line) &&
               (sense(line) != ExtiSense::none || interrupt(line) || event(line));
    }

    // ---- the ISR body ------------------------------------------------------

    /**
     * The body of one EXTI vector. `lines` is the mask of lines THIS
     * vector answers for (`Exti::vector_lines(Irq::exti9_5)` and
     * friends), and the return is the lines that fired; `served()` asks
     * it about one of them.
     *
     * READ AND CLEAR AT THE TOP. A handler that clears its flag last
     * returns while the request may still stand; clearing first costs
     * nothing, and an edge arriving between the read and the store is not
     * lost - it raises the flag again after the clear lands.
     */
    [[gnu::always_inline]] static uint32_t isr(uint32_t lines) {
        const uint32_t fired = regs().INTFR & lines;
        if (fired != 0u) {
            regs().INTFR = fired;
        }
        return fired;
    }

    /// Did line `line` fire in the mask a handler got from isr()?
    static constexpr bool served(uint32_t fired, uint8_t line) {
        return line < 32u && (fired & (1UL << line)) != 0u;
    }

    // ---- teardown ----------------------------------------------------------

    /**
     * One line back to its reset state: no sense, no interrupt, no event,
     * no flag, no standing software trigger. The multiplexer is left
     * alone on purpose - EXTICR's reset value is port A, and "give line 3
     * back to PA3" is a claim and not a release.
     */
    static bool release(uint8_t line) {
        if (!implemented(line)) {
            return false;
        }
        (void)interrupt(line, false);
        (void)event(line, false);
        (void)sense(line, ExtiSense::none);
        (void)clear(line);
        regs().SWIEVR &= ~bit(line);
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
 * A line the program names as a CONSTANT - which is what a peripheral's
 * wake-up always is:
 *
 *   using UsbWake = brio::ExtiLine<brio::Exti::line_usbd_wakeup>;
 *   UsbWake::configure(brio::ExtiSense::rising);
 *   UsbWake::arm(true);
 *   brio::Pfic::enable(*UsbWake::irq());
 *
 * The point of the type is the static_assert: a line this part has not
 * got - the Ethernet's wake-up on a part with no MAC, the 32 kHz
 * calibration's on the wrong device class - is a COMPILE ERROR here,
 * where `Exti`'s run-time verbs can only answer false. Both faces exist
 * because both questions are real: a driver that walks a line number
 * computed at run time needs the second.
 */
template <uint8_t Line>
struct ExtiLine {
    ExtiLine() = delete;

    static_assert(Exti::implemented(Line),
                  "brio ExtiLine: this part does not have that EXTI line - the sixteen pin "
                  "lines and the PVD's and the RTC alarm's exist everywhere, and each of the "
                  "four above them exists only where the peripheral wired to it does (no USB "
                  "wake-up without that controller, no Ethernet wake-up without a MAC, and the "
                  "32 kHz calibration's line is the other device class's)");

    static constexpr uint8_t line = Line;
    static constexpr uint32_t mask = 1UL << Line;

    static constexpr std::optional<Irq> irq() { return Exti::irq(Line); }

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
// One pin line, reached through its pad
// =============================================================================

/**
 * The line a pad can raise, with the pin number for a line number and the
 * port letter for the multiplexer's code:
 *
 *   using Button = brio::Pin<'C', 13>;
 *   using ButtonInt = brio::ExtInt<Button>;      // line 13, port C
 *   ButtonInt::claim(brio::PinPull::up);         // input + pull + EXTICR
 *   ButtonInt::configure(brio::ExtiSense::falling);
 *   ButtonInt::arm(true);
 *   brio::Pfic::enable(*ButtonInt::irq());       // exti15_10
 *
 * THE PAD KEEPS ITS GPIO MODE. This is not an alternate function: the
 * EXTI watches the port's INPUT, which the pad drives in input, output
 * and alternate modes alike (10.2.6..10.2.8: the level "is sampled to the
 * input data register every PB2 clock" in all three), so a line can watch
 * a pad the application is DRIVING or one a peripheral owns - which is
 * why `claim()` and `select()` are separate verbs. Only ANALOG mode,
 * where the input driver is off (10.2.9), hides a pad from its line.
 */
template <class P>
struct ExtInt {
    ExtInt() = delete;

    static_assert(exti_port_code(P::port_letter) != 0xFFu,
                  "brio ExtInt: this family has no GPIO port of that letter, so no "
                  "AFIO_EXTICR code selects it (ports A..E)");
    static_assert(P::pin_number < Exti::gpio_lines,
                  "brio ExtInt: the EXTI has one line per PIN NUMBER and there are sixteen "
                  "of them");

    using pin = P;

    /// The line IS the pin number (10.2.3): no table, no formula to get
    /// wrong, and no per-package gate - every bonded pad of every port
    /// reaches the line of its own number.
    static constexpr uint8_t line = P::pin_number;
    static constexpr uint32_t mask = 1UL << line;
    static constexpr char port = P::port_letter;

    static constexpr std::optional<Irq> irq() { return Exti::irq(line); }

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
    /// Exti::release) and the pad back to a floating input.
    static void release() {
        (void)Exti::release(line);
        P::release();
    }
};

/**
 * THE ONE-PAD-PER-LINE RULE AS A COMPILE-TIME CHECK. Sixteen lines are
 * shared by up to five ports, so an application that arms PA3 and PB3 has
 * written a bug - `Exti::select()` refuses the second claim at run time,
 * but a refusal is an answer nobody has to read. The application can
 * state the whole set instead, and be told at compile time:
 *
 *   static_assert(brio::exti_lines_distinct<ButtonInt, SensorInt>(),
 *                 "two pads on one EXTI line");
 *
 * Nothing enforces that the assertion is written - it is the same kind of
 * claim as a remap code, which no header of this family can check either.
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
