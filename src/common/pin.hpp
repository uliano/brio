#pragma once
#include <avr/io.h>

// Compile-time GPIO pin abstraction for the AVR-Dx (AVR128DB28), register-level.
// Ported verbatim from uliano/AVR-Multislope (lib/core/src/pin.hpp).
//
// Uses VPORT for single-cycle SBI/CBI instructions on set/clear/read, and the
// regular PORT for atomic OUTTGL/DIRSET/DIRCLR operations. Use invert(true) for
// active-low signals (hardware inversion via INVEN).
//
//   using Led = Pin<'A', 5>;   // PA5
//   Led::output();             // PORTA.DIRSET = PIN5_bm
//   Led::toggle();             // PORTA.OUTTGL = PIN5_bm
template<char PortLetter, uint8_t PinNum>
struct Pin {
    static_assert(PortLetter >= 'A' && PortLetter <= 'F', "Invalid port");
    static_assert(PinNum <= 7, "Invalid pin number");

    static constexpr uint8_t mask = (1 << PinNum);

    // Regular PORT for atomic set/clear/toggle registers.
    // NOTE: PORTB and PORTE do not exist on 28/32-pin AVR-Dx parts (only on the
    // 48/64-pin packages), so those branches are #ifdef-guarded. On the
    // AVR128DB28 only A, C, D, F are present - do not request a port your
    // package lacks (it would fall through to PORTF).
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
        else return PORTF;
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
        else return VPORTF;
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
    static void disableDigitalInput() {
        pinctrl() = (pinctrl() & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc;
    }

    // Enable digital input buffer (default state)
    static void enableDigitalInput() {
        pinctrl() = (pinctrl() & ~PORT_ISC_gm) | PORT_ISC_INTDISABLE_gc;
    }
};
