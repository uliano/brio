#pragma once
#include <stdint.h>
#include <avr/io.h>

#include "util/pwm_channel.hpp"

// Compile-time GPIO pin abstraction for the AVR Dx families, register-level.
// Ported from uliano/AVR-Multislope (lib/core/src/pin.hpp).
//
// Uses VPORT for single-cycle SBI/CBI instructions on set/clear/read, and the
// regular PORT for atomic OUTTGL/DIRSET/DIRCLR operations. Use invert(true)
// for active-low signals (hardware inversion via INVEN).
//
//   using Led = brio::Pin<'A', 5>;   // PA5
//   Led::output();                 // PORTA.DIRSET = PIN5_bm
//   Led::toggle();                 // PORTA.OUTTGL = PIN5_bm

namespace brio {

/// Runtime pin descriptor (3 bytes): lets a pin chosen at compile time
/// travel inside a request event (e.g. the CS/DC of a SPI transaction,
/// asserted by the bus AO, not by the client). A null PinRef (default)
/// means "no such pin": set/clear are no-ops, so optional pins cost one
/// branch. Build one with Pin<...>::ref().
struct PinRef {
    volatile VPORT_t* vport = nullptr;
    uint8_t mask = 0;

    void set() const {
        if (vport != nullptr) {
            vport->OUT |= mask;
        }
    }
    void clear() const {
        if (vport != nullptr) {
            vport->OUT &= ~mask;
        }
    }
    constexpr bool valid() const { return vport != nullptr; }
};

/// The PORT of a letter chosen at RUN time (for drivers whose pin is a
/// configuration value: a route's port, a comparator's input). Folds
/// to one address when the letter is a constant; an absent port (by
/// package) falls back to PORTA - callers validate the letter first.
constexpr volatile PORT_t& port_by_letter(char p) {
    switch (p) {
#ifdef PORTB
        case 'B': return PORTB;
#endif
        case 'C': return PORTC;
        case 'D': return PORTD;
#ifdef PORTE
        case 'E': return PORTE;
#endif
        case 'F': return PORTF;
#ifdef PORTG
        case 'G': return PORTG;
#endif
        default: return PORTA;
    }
}

/// PINnCTRL of (port letter, pin number) at run time.
inline volatile uint8_t& pinctrl_of(char p, uint8_t n) {
    return (&port_by_letter(p).PIN0CTRL)[n];
}

template <char PortLetter, uint8_t PinNum>
struct Pin {
    static_assert(PortLetter >= 'A' && PortLetter <= 'G', "Invalid port");
    static_assert(PinNum <= 7, "Invalid pin number");

    static constexpr char port_letter = PortLetter;   ///< 'A'..'G'
    static constexpr uint8_t pin_number = PinNum;      ///< 0..7
    static constexpr uint8_t mask = (1 << PinNum);

    // Regular PORT for atomic set/clear/toggle registers.
    // NOTE: which ports exist depends on the package (e.g. no PORTB/PORTE on
    // 28/32-pin parts, PORTG only on 64-pin), hence the #ifdef guards.
    // Requesting a port the package lacks is a compile error (C++23
    // static_assert(false) fires only in the instantiated branch).
    static constexpr volatile PORT_t& port() {
        if constexpr (PortLetter == 'A') return PORTA;
#ifdef PORTB
        else if constexpr (PortLetter == 'B') return PORTB;
#endif
        else if constexpr (PortLetter == 'C') return PORTC;
        else if constexpr (PortLetter == 'D') return PORTD;
#ifdef PORTE
        else if constexpr (PortLetter == 'E') return PORTE;
#endif
        else if constexpr (PortLetter == 'F') return PORTF;
#ifdef PORTG
        else if constexpr (PortLetter == 'G') return PORTG;
#endif
        else static_assert(false, "this port does not exist on this device");
    }

    // Virtual PORT for single-cycle bit access (SBI/CBI)
    static constexpr volatile VPORT_t& vport() {
        if constexpr (PortLetter == 'A') return VPORTA;
#ifdef VPORTB
        else if constexpr (PortLetter == 'B') return VPORTB;
#endif
        else if constexpr (PortLetter == 'C') return VPORTC;
        else if constexpr (PortLetter == 'D') return VPORTD;
#ifdef VPORTE
        else if constexpr (PortLetter == 'E') return VPORTE;
#endif
        else if constexpr (PortLetter == 'F') return VPORTF;
#ifdef VPORTG
        else if constexpr (PortLetter == 'G') return VPORTG;
#endif
        else static_assert(false, "this port does not exist on this device");
    }

    // Access PINnCTRL register for this pin
    static volatile uint8_t& pinctrl() {
        return (&port().PIN0CTRL)[PinNum];
    }

    /// Runtime descriptor of this pin (for request events - see PinRef).
    static constexpr PinRef ref() { return {&vport(), mask}; }

    // A pin is the degenerate PwmChannel (max = 1: any non-zero duty is
    // "on"), so generic actuators written over that concept (RgbLamp)
    // drive bare pins and timer channels with the same code.
    static constexpr uint16_t max = 1;
    static void duty(uint16_t v) { if (v) set(); else clear(); }

    // Basic I/O
    static void toggle() { port().OUTTGL = mask; }
    static void set()    { vport().OUT |= mask; }
    static void clear()  { vport().OUT &= ~mask; }
    static void output() { port().DIRSET = mask; }
    static void input()  { port().DIRCLR = mask; }
    static bool read()   { return vport().IN & mask; }

    // Hardware inversion (INVEN) - inverts both input and output
    static void invert(bool enable) {
        if (enable) pinctrl() |= PORT_INVEN_bm;
        else pinctrl() &= ~PORT_INVEN_bm;
    }

    // Internal pull-up (only effective when pin is input)
    static void pullup(bool enable) {
        if (enable) pinctrl() |= PORT_PULLUPEN_bm;
        else pinctrl() &= ~PORT_PULLUPEN_bm;
    }

    // Disable digital input buffer (saves power for analog pins)
    static void disable_digital_input() {
        pinctrl() = (pinctrl() & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc;
    }

    // Enable digital input buffer (default state)
    static void enable_digital_input() {
        pinctrl() = (pinctrl() & ~PORT_ISC_gm) | PORT_ISC_INTDISABLE_gc;
    }
};

static_assert(PwmChannel<Pin<'A', 0>>);

/**
 * PinSet<Pins...>: up to 8 pins, on any ports, handled as one bit mask
 * (bit i = the i-th pin of the pack). For a row of buttons, a bus of
 * select lines, a DIP switch. Every operation is a fold over the pack,
 * so a set on one port costs the same single-cycle VPORT accesses as
 * writing the port by hand - and the pins need not share a port.
 *
 *   using Keys = brio::PinSet<brio::Pin<'A', 2>, brio::Pin<'A', 3>>;
 *   Keys::input(true);                 // inputs with pull-ups
 *   const uint8_t raw = ~Keys::read() & Keys::mask;   // active-low
 */
template <typename... Pins>
struct PinSet {
    static_assert(sizeof...(Pins) >= 1 && sizeof...(Pins) <= 8, "1..8 pins");
    PinSet() = delete;

    static constexpr uint8_t count = sizeof...(Pins);
    static constexpr uint8_t mask = static_cast<uint8_t>((1u << count) - 1u);

    static void input(bool pullup = false) {
        (Pins::input(), ...);
        (Pins::pullup(pullup), ...);
    }
    static void output() { (Pins::output(), ...); }

    /// Bit i = level of pin i.
    static uint8_t read() {
        uint8_t v = 0;
        uint8_t bit = 0;
        ((v |= static_cast<uint8_t>(Pins::read() ? (1u << bit) : 0u), ++bit), ...);
        return v;
    }

    /// Drive pin i to bit i of `value`.
    static void write(uint8_t value) {
        uint8_t bit = 0;
        (((value & (1u << bit)) ? Pins::set() : Pins::clear(), ++bit), ...);
    }
};

} // namespace brio
