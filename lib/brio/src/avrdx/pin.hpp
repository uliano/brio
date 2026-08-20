#pragma once
#include <stdint.h>
#include <avr/io.h>

#include "util/pwm_channel.hpp"

// The AVR DA/DB I/O pins (PORT, DS40002247B ch. 18), register-level:
//
//  Port<'C'>       the port RESOURCE - what a pin does not possess:
//                  the shared interrupt's flags (one vector per port,
//                  take_flags() is the ISR body), the port-wide slew
//                  limit, the multi-pin configuration engine, native
//                  mask operations;
//  Pin<'A', 5>     the per-pin face: direction/value on VPORT (SBI/
//                  CBI), configure(PinConfig) as ONE PINnCTRL store
//                  (invert, pullup, input level, sense), the single-
//                  field RMW verbs, flag()/clear_flag(); a PwmChannel
//                  (max 1) and a PinRef factory;
//  PinSet<...>     up to 8 pins on any ports as one bit mask; its
//                  configure() groups by port at compile time and
//                  rides the multi-pin engine.
//
//   using Led = brio::Pin<'A', 5>;   // PA5
//   Led::output();                 // PORTA.DIRSET = PIN5_bm
//   Led::toggle();                 // PORTA.OUTTGL = PIN5_bm
//   Led::configure({.pullup = true, .sense = brio::PinSense::falling});
//   ISR(PORTA_PORT_vect) { const uint8_t who = brio::Port<'A'>::take_flags(); ... }

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

/// Whether this package bonds out the port at all: 28/32-pin parts
/// lack PORTB and PORTE, PORTG exists on 64-pin only (the device
/// header is the authority, hence the #ifdefs). Port-level only -
/// a pin missing from an EXISTING port (e.g. PF4 on 28-pin parts)
/// is the per-family device tables' job. Drivers use this to keep an
/// instance usable when only one of its pin POSITIONS is absent
/// (an `if constexpr` on the missing branch), and to refuse a config
/// that asks for it.
constexpr bool port_exists(char p) {
    switch (p) {
        case 'A': case 'C': case 'D': case 'F': return true;
#ifdef PORTB
        case 'B': return true;
#endif
#ifdef PORTE
        case 'E': return true;
#endif
#ifdef PORTG
        case 'G': return true;
#endif
        default: return false;
    }
}

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

// ---- pin configuration vocabulary (18.5.16) ---------------------------------

/// ISC: what the input stage does. `none` = buffer on, no interrupt
/// (the reset state); the edge/level senses raise the PORT's one
/// interrupt (vector PORTx_PORT_vect, flags by pin in INTFLAGS);
/// `level_low` re-fires for as long as the pin reads low;
/// `input_disable` turns the digital input buffer OFF - IN freezes,
/// interrupts and events from the pin die, the pad keeps feeding the
/// analog mux (what ADC/AC inputs want: no mid-rail current, no
/// digital noise).
enum class PinSense : uint8_t {
    none = PORT_ISC_INTDISABLE_gc,
    both = PORT_ISC_BOTHEDGES_gc,
    rising = PORT_ISC_RISING_gc,
    falling = PORT_ISC_FALLING_gc,
    input_disable = PORT_ISC_INPUT_DISABLE_gc,
    level_low = PORT_ISC_LEVEL_gc,
};

#ifdef PORT_INLVL_bm
/// Input threshold (DB only): Schmitt from the supply, or TTL levels
/// (an MVIO companion). Change it only with the pin's interrupts and
/// peripherals quiet: the switch can fake a transition (18.3.3).
enum class PinLevel : uint8_t { schmitt = 0, ttl = PORT_INLVL_bm };
#endif

/// The whole PINnCTRL as one value, written in ONE store by
/// `Pin::configure` / the multi-pin paths: the datasheet warns that
/// INVEN toggled in the same cycle as an ISC change does not raise
/// the inversion edge's interrupt - for configuration that is exactly
/// the desired quietness (and single fields keep their RMW verbs).
struct PinConfig {
    bool invert = false;
    bool pullup = false;      ///< only effective while the pin is an input
#ifdef PORT_INLVL_bm
    PinLevel input_level = PinLevel::schmitt;
#endif
    PinSense sense = PinSense::none;
};

constexpr uint8_t pin_ctrl_byte(const PinConfig& c) {
    return static_cast<uint8_t>(
        (c.invert ? PORT_INVEN_bm : 0) |
        (c.pullup ? PORT_PULLUPEN_bm : 0) |
#ifdef PORT_INLVL_bm
        static_cast<uint8_t>(c.input_level) |
#endif
        static_cast<uint8_t>(c.sense));
}

// ---- the port resource ------------------------------------------------------

/// Port<'C'>: the typed view of ONE port's own registers - what a pin
/// does not possess: the interrupt flags of the shared PORTx_PORT_vect
/// (several pins can fire together: the ISR reads the MASK), the
/// port-wide slew-rate limit, the multi-pin configuration engine and
/// the native mask operations. Pin<letter, n> stays the per-pin face
/// and delegates its register access here.
template <char L>
struct Port {
    Port() = delete;
    static_assert(L >= 'A' && L <= 'G', "ports: 'A'..'G'");
    static_assert(port_exists(L), "this port does not exist on this device");

    static constexpr char letter = L;

    static constexpr volatile PORT_t& regs() {
        if constexpr (L == 'A') return PORTA;
#ifdef PORTB
        else if constexpr (L == 'B') return PORTB;
#endif
        else if constexpr (L == 'C') return PORTC;
        else if constexpr (L == 'D') return PORTD;
#ifdef PORTE
        else if constexpr (L == 'E') return PORTE;
#endif
        else if constexpr (L == 'F') return PORTF;
#ifdef PORTG
        else if constexpr (L == 'G') return PORTG;
#endif
    }
    static constexpr volatile VPORT_t& vregs() {
        if constexpr (L == 'A') return VPORTA;
#ifdef VPORTB
        else if constexpr (L == 'B') return VPORTB;
#endif
        else if constexpr (L == 'C') return VPORTC;
        else if constexpr (L == 'D') return VPORTD;
#ifdef VPORTE
        else if constexpr (L == 'E') return VPORTE;
#endif
        else if constexpr (L == 'F') return VPORTF;
#ifdef VPORTG
        else if constexpr (L == 'G') return VPORTG;
#endif
    }

    // Mask operations (bit n = pin n).
    static uint8_t in() { return vregs().IN; }
    static void dir_set(uint8_t m) { regs().DIRSET = m; }
    static void dir_clear(uint8_t m) { regs().DIRCLR = m; }
    static void out_set(uint8_t m) { regs().OUTSET = m; }
    static void out_clear(uint8_t m) { regs().OUTCLR = m; }
    static void out_toggle(uint8_t m) { regs().OUTTGL = m; }

    // Interrupt flags: write-1-to-clear - ALWAYS plain stores (an RMW
    // or an SBI would read every pending flag back as 1 and clear the
    // lot).
    static uint8_t flags() { return vregs().INTFLAGS; }
    static void clear_flags(uint8_t m) { regs().INTFLAGS = m; }
    /// ISR body for PORTx_PORT_vect: which pins fired (bit n), cleared.
    [[gnu::always_inline]] static uint8_t take_flags() {
        const uint8_t f = regs().INTFLAGS;
        regs().INTFLAGS = f;
        return f;
    }

    /// PORTCTRL.SRL: slew-rate limitation for EVERY pin of this port.
    static void slew_limit(bool on) { regs().PORTCTRL = on ? PORT_SRL_bm : 0; }
    static bool slew_limit() { return (regs().PORTCTRL & PORT_SRL_bm) != 0; }

    /// Multi-pin configuration (18.3.2.4): the PINnCTRL of every pin
    /// in `pins` written to `cfg` in one operation (PINCONFIG is
    /// mirrored across ports; the update mask is this port's).
    static void configure_mask(uint8_t pins, const PinConfig& cfg) {
        regs().PINCONFIG = pin_ctrl_byte(cfg);
        regs().PINCTRLUPD = pins;
    }
};

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
    static constexpr volatile PORT_t& port() { return Port<PortLetter>::regs(); }

    // Virtual PORT for single-cycle bit access (SBI/CBI)
    static constexpr volatile VPORT_t& vport() { return Port<PortLetter>::vregs(); }

    /// Px2 and Px6 sense FULLY asynchronously (I/O mux note 2): they
    /// wake from every sleep mode on every sense, catch sub-CLK_PER
    /// pulses and have no three-cycle dead-time; the other pins need
    /// CLK_PER (or BOTHEDGES/LEVEL held long enough) to wake.
    static constexpr bool fully_async = PinNum == 2 || PinNum == 6;

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

    /// The whole PINnCTRL in ONE store (invert + pullup + input level
    /// + sense): the way to (re)configure - no intermediate states,
    /// no INVEN/ISC same-cycle surprises. The single-field verbs
    /// below RMW the same register: main-vs-ISR discipline is the
    /// caller's (like every shared register).
    static void configure(const PinConfig& cfg) { pinctrl() = pin_ctrl_byte(cfg); }

    /// ISC alone (sense or input_disable), keeping INVEN/PULLUPEN.
    /// The datasheet's caveat: changing ISC while an interrupt is
    /// synchronizing can fire a spurious one or miss one - change it
    /// with the pin quiet, or clear_flag() after.
    static void sense(PinSense s) {
        pinctrl() = static_cast<uint8_t>((pinctrl() & ~PORT_ISC_gm) | static_cast<uint8_t>(s));
    }

    /// This pin's bit in the port's interrupt flags (write-1-to-clear:
    /// clear_flag is a plain store, never an SBI - an RMW would clear
    /// every pending flag of the port).
    static bool flag() { return (vport().INTFLAGS & mask) != 0; }
    static void clear_flag() { port().INTFLAGS = mask; }

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

    /// One PinConfig into every pin's PINnCTRL through the multi-pin
    /// engine (18.3.2.4): the pins are grouped BY PORT at compile
    /// time, PINCONFIG is written once (mirrored across ports) and
    /// each involved port gets one PINCTRLUPD mask - the set behaves
    /// like a single register even when its pins span ports, and the
    /// pins of one port change in the same cycle.
    static void configure(const PinConfig& cfg) {
        Port<'A'>::regs().PINCONFIG = pin_ctrl_byte(cfg);   // mirrored everywhere
        apply_update<'A'>(); apply_update<'B'>(); apply_update<'C'>();
        apply_update<'D'>(); apply_update<'E'>(); apply_update<'F'>();
        apply_update<'G'>();
    }

    /// The pins of this set living on port L, as that port's mask.
    template <char L>
    static constexpr uint8_t port_mask() {
        uint8_t m = 0;
        ((m |= (Pins::port_letter == L ? Pins::mask : 0)), ...);
        return m;
    }

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

private:
    template <char L>
    static void apply_update() {
        if constexpr (port_mask<L>() != 0) {
            Port<L>::regs().PINCTRLUPD = port_mask<L>();
        }
    }
};

} // namespace brio
