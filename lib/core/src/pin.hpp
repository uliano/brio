#pragma once
#include <avr/io.h>

// Compile-time GPIO pin abstraction for the AVR Dx families, register-level.
// Ported from uliano/AVR-Multislope (lib/core/src/pin.hpp).
//
// Uses VPORT for single-cycle SBI/CBI instructions on set/clear/read, and the
// regular PORT for atomic OUTTGL/DIRSET/DIRCLR operations. Use invert(true)
// for active-low signals (hardware inversion via INVEN).
//
//   using Led = dx::Pin<'A', 5>;   // PA5
//   Led::output();                 // PORTA.DIRSET = PIN5_bm
//   Led::toggle();                 // PORTA.OUTTGL = PIN5_bm

namespace dx {

template <char PortLetter, uint8_t PinNum>
struct Pin {
    static_assert(PortLetter >= 'A' && PortLetter <= 'G', "Invalid port");
    static_assert(PinNum <= 7, "Invalid pin number");

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

} // namespace dx
