/*
 * evsys.hpp
 *
 * The AVR DA/DB event system (EVSYS, DS40002247B ch. 16) as brio sees
 * it - see docs/avrdx/evsys.md for the analysis and the model:
 *
 *  - a GENERATOR is a type: its CHANNELn code and the channels it may
 *    legally drive are constexpr facts (EvPitDiv<64>, EvPin<Pin<'A',2>>,
 *    EvRtcOvf...); the concept EventGenerator names the contract;
 *  - a USER is a type that knows its USERxxx register (EvOut<Pin<'D',2>>,
 *    EvAdc0Start, EvTcbCaptIn<0>...); the concept EventUser;
 *  - a CHANNEL is a resource handle, EventChannel<n>.
 *
 * Run-time primitives - the layer that IS the peripheral, one register
 * write each, callable from any handler (a rewire is an action of a
 * state: Entry routes, Exit disconnects):
 *
 *   EventChannel<1>::source(EvPitDiv<64>{});   // static_assert: odd channel
 *   EvOut<Pin<'D', 2>>::listen(EventChannel<1>{});
 *   ...
 *   EvOut<Pin<'D', 2>>::unlisten();
 *   EventChannel<1>::off();
 *   EventChannel<1>::pulse();                  // software event, one CLK_PER
 *
 * The compiler checks LEGALITY (this generator on this channel, this
 * pin as an EVOUT); it does not check EXCLUSIVITY of a channel that
 * several states rewire - that is ownership, the business of one AO's
 * FSM. The tables below hold what the current steps need (RTC/PIT
 * generators, port pins, EVOUT users) and grow on demand: a missing
 * generator or user is added here, three lines, never worked around
 * with a raw register write. A static allocator (EventSystem<Route...>)
 * is planned as sugar over these primitives when an app has more than
 * a couple of fixed routes.
 *
 * Facts worth remembering (16.3.2): a pin generator is the pin LEVEL
 * (the user's edge detection makes the edge; the pin's input driver
 * must be enabled); PIT_DIVn are levels - the prescaled RTC clock
 * divided, a free square wave (32768 / n Hz with the RTC prescaler at
 * 1, as Ticker leaves it); software events need CLK_PER (not in
 * standby); async users respond in standby without a clock. Errata
 * DS80000915F: no EVSYS items.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <concepts>

#include "avrdx/pin.hpp"

namespace brio {

// ---- contracts --------------------------------------------------------------

/// A generator: the CHANNELn code and the channels it may drive.
template <typename G>
concept EventGenerator = requires(uint8_t ch) {
    { G::code } -> std::convertible_to<uint8_t>;
    { G::legal_on(ch) } -> std::same_as<bool>;
};

/// A user: exposes its USER register (the channel mux) and the
/// listen/unlisten pair (usually inherited from EventUserBase).
template <typename U>
concept EventUser = requires {
    { U::reg() } -> std::same_as<volatile uint8_t&>;
    U::unlisten();
};

// ---- channels ---------------------------------------------------------------

/// Number of event channels on this device (CHANNEL0..9 on 48/64-pin
/// DA/DB; 28/32-pin parts have 8 - a fact for the device table when it
/// exists).
inline constexpr uint8_t event_channels = 10;

/// Resource handle for event channel n (also usable as an empty tag).
template <uint8_t n>
struct EventChannel {
    static_assert(n < event_channels, "no such event channel on this device");
    // Constructible: an empty tag object names the channel in calls
    // (`listen(EventChannel<1>{})`), like `clock` and `serial`.

    static constexpr uint8_t index = n;

    /// Drive the channel from generator G. Legality checked at compile time.
    template <EventGenerator G>
    static void source(G) {
        static_assert(G::legal_on(n),
                      "this event generator cannot be routed onto this channel "
                      "(PIT_DIV8192..1024: even channels; PIT_DIV512..64: odd; "
                      "PORTA/B pins: channels 0-1, PORTC/D: 2-3, PORTE/F: 4-5)");
        reg() = G::code;
    }

    /// Disconnect the generator (channel idle, users see nothing).
    static void off() { reg() = 0; }

    /// Software event: one CLK_PER pulse on the channel (needs CLK_PER,
    /// i.e. not in standby sleep). Users see it as any other event.
    static void pulse() {
        if constexpr (n < 8) {
            EVSYS.SWEVENTA = static_cast<uint8_t>(1u << n);
        } else {
            EVSYS.SWEVENTB = static_cast<uint8_t>(1u << (n - 8));
        }
    }

private:
    static volatile uint8_t& reg() { return (&EVSYS.CHANNEL0)[n]; }
};

// ---- generators (16.5.2) ------------------------------------------------------

/// The prescaled RTC clock divided by div: 8192/4096/2048/1024 on EVEN
/// channels only, 512/256/128/64 on ODD channels only (the two families
/// share codes 0x08..0x0B - the parity of the channel picks the family).
template <uint16_t div>
struct EvPitDiv {
    static_assert(div == 64 || div == 128 || div == 256 || div == 512 ||
                  div == 1024 || div == 2048 || div == 4096 || div == 8192,
                  "PIT event dividers: 64, 128, 256, 512 (odd channels), "
                  "1024, 2048, 4096, 8192 (even channels)");
    static constexpr bool even_family = div >= 1024;
    static constexpr uint8_t code =
        div == 8192 || div == 512 ? 0x08 :
        div == 4096 || div == 256 ? 0x09 :
        div == 2048 || div == 128 ? 0x0A : 0x0B;
    static constexpr bool legal_on(uint8_t ch) {
        return even_family ? (ch % 2 == 0) : (ch % 2 == 1);
    }
};

/// RTC counter overflow (pulse, CLK_RTC). All channels.
struct EvRtcOvf {
    static constexpr uint8_t code = 0x06;
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// RTC compare match (pulse, CLK_RTC). All channels.
struct EvRtcCmp {
    static constexpr uint8_t code = 0x07;
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// ADC0 result ready (pulse, CLK_PER). All channels.
struct EvAdc0Ready {
    static constexpr uint8_t code = 0x24;
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCA n overflow (normal mode) / low-byte underflow (split mode):
/// pulse, CLK_PER, all channels. Codes 0x80 (TCA0), 0x88 (TCA1).
template <uint8_t n>
struct EvTcaOvf {
    static_assert(n <= 1, "TCA0 and (48/64-pin parts) TCA1");
    static constexpr uint8_t code = static_cast<uint8_t>(0x80 + 8 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCA n high-byte underflow (split mode only). 0x81 / 0x89.
template <uint8_t n>
struct EvTcaHunf {
    static_assert(n <= 1, "TCA0 and (48/64-pin parts) TCA1");
    static constexpr uint8_t code = static_cast<uint8_t>(0x81 + 8 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCA n compare channel ch match (normal) / low-byte compare ch
/// (split): pulse, all channels. 0x84 + ch (TCA0), 0x8C + ch (TCA1).
template <uint8_t n, uint8_t ch>
struct EvTcaCmp {
    static_assert(n <= 1, "TCA0 and (48/64-pin parts) TCA1");
    static_assert(ch <= 2, "TCA compare channels: 0..2");
    static constexpr uint8_t code = static_cast<uint8_t>(0x84 + 8 * n + ch);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCB n CAPT flag set (the condition depends on the mode, tcb.hpp):
/// pulse, CLK_PER, all channels. 0xA0 + 2n.
template <uint8_t n>
struct EvTcbCapt {
    static_assert(n <= 4, "TCB0..TCB4");
    static constexpr uint8_t code = static_cast<uint8_t>(0xA0 + 2 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCB n counter overflow (MAX -> BOTTOM): pulse, all channels. 0xA1 + 2n.
/// The carry of a 32-bit cascade.
template <uint8_t n>
struct EvTcbOvf {
    static_assert(n <= 4, "TCB0..TCB4");
    static constexpr uint8_t code = static_cast<uint8_t>(0xA1 + 2 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// CCL LUT n output LEVEL (async). 0x10 + n.
template <uint8_t n>
struct EvLut {
    static_assert(n <= 5, "LUT0..LUT5");
    static constexpr uint8_t code = static_cast<uint8_t>(0x10 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// Analog comparator n output LEVEL (async). 0x20 + n.
template <uint8_t n>
struct EvAcOut {
    static_assert(n <= 2, "AC0..AC2");
    static constexpr uint8_t code = static_cast<uint8_t>(0x20 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// A port pin's LEVEL as an event (async; zero if the input driver is
/// disabled). PORTA/PORTB: channels 0-1; PORTC/PORTD: 2-3; PORTE/PORTF:
/// 4-5. The two ports of a pair share the code space: 0x40+n for the
/// first, 0x48+n for the second.
template <typename P>
struct EvPin {
    static constexpr char port = P::port_letter;
    static constexpr uint8_t pin = P::pin_number;
    static_assert(port >= 'A' && port <= 'F', "pin events: PORTA..PORTF");
    static constexpr uint8_t code =
        static_cast<uint8_t>(((port - 'A') % 2 == 0 ? 0x40 : 0x48) + pin);
    static constexpr bool legal_on(uint8_t ch) {
        const uint8_t pair = static_cast<uint8_t>((port - 'A') / 2);   // A/B=0, C/D=1, E/F=2
        return ch / 2 == pair;
    }
};

// ---- users (16.5.3) -----------------------------------------------------------

/// listen/unlisten over Derived::reg(). USER = channel + 1, 0 = off.
template <typename Derived>
struct EventUserBase {
    EventUserBase() = delete;

    template <uint8_t n>
    static void listen(EventChannel<n>) {
        Derived::reg() = static_cast<uint8_t>(n + 1);
    }
    static void unlisten() { Derived::reg() = 0; }
};

/// EVOUT: the channel's signal on a pin. Pins with an EVOUT function
/// (PORTMUX.EVSYSROUTEA): PA2 (alt PA7), PB2, PC2 (alt PC7), PD2 (alt
/// PD7), PE2, PF2. listen() selects the pin position, drives the pin
/// as output, connects. Any generator becomes a signal on a pin: the
/// test instrument of this peripheral.
template <typename P>
struct EvOut : EventUserBase<EvOut<P>> {
    static constexpr char port = P::port_letter;
    static constexpr uint8_t pin = P::pin_number;
    static_assert(port >= 'A' && port <= 'F', "EVOUT: PORTA..PORTF");
    static_assert(pin == 2 || (pin == 7 && (port == 'A' || port == 'C' || port == 'D')),
                  "EVOUT pins: Px2 on every port, Px7 (ALT1) on PORTA/PORTC/PORTD "
                  "(48-pin: PB7/PE7 absent, PF7 not an EVOUT)");

    static volatile uint8_t& reg() {
        if constexpr (port == 'A') return EVSYS.USEREVSYSEVOUTA;
#ifdef PORTB
        else if constexpr (port == 'B') return EVSYS.USEREVSYSEVOUTB;
#endif
        else if constexpr (port == 'C') return EVSYS.USEREVSYSEVOUTC;
        else if constexpr (port == 'D') return EVSYS.USEREVSYSEVOUTD;
#ifdef PORTE
        else if constexpr (port == 'E') return EVSYS.USEREVSYSEVOUTE;
#endif
        else if constexpr (port == 'F') return EVSYS.USEREVSYSEVOUTF;
        else static_assert(false, "EVOUT: this port is not present on this device");
    }

    template <uint8_t n>
    static void listen(EventChannel<n> ch) {
        constexpr uint8_t bit = static_cast<uint8_t>(1u << (port - 'A'));
        if constexpr (pin == 7) {
            PORTMUX.EVSYSROUTEA |= bit;      // ALT1 position
        } else {
            PORTMUX.EVSYSROUTEA &= static_cast<uint8_t>(~bit);
        }
        P::output();                          // the event drives the pin's output
        EventUserBase<EvOut<P>>::listen(ch);
    }
};

/// ADC0 start-conversion on the channel's rising edge (async). The ADC
/// driver's start_on() also sets STARTEI; listening alone arms nothing.
struct EvAdc0Start : EventUserBase<EvAdc0Start> {
    static volatile uint8_t& reg() { return EVSYS.USERADC0START; }
};

/// TCA n event input A: count on edge / while high, or direction from
/// the level - the action is EVACTA in the TCA (tca.hpp). Sync.
template <uint8_t n>
struct EvTcaCntA : EventUserBase<EvTcaCntA<n>> {
    static_assert(n <= 1, "TCA0 and (48/64-pin parts) TCA1");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCA0CNTA)[2 * n]; }
};

/// TCA n event input B: restart on edge / while high, or direction.
template <uint8_t n>
struct EvTcaCntB : EventUserBase<EvTcaCntB<n>> {
    static_assert(n <= 1, "TCA0 and (48/64-pin parts) TCA1");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCA0CNTB)[2 * n]; }
};

/// TCB n capture input: start/stop/capture/restart per the TCB's mode
/// and EDGE bit (tcb.hpp); sync, async in single-shot with ASYNC.
/// The TCB must also set CAPTEI - listening alone arms nothing.
template <uint8_t n>
struct EvTcbCaptIn : EventUserBase<EvTcbCaptIn<n>> {
    static_assert(n <= 3, "TCB0..TCB3 (TCB4 is a 64-pin part: its user registers follow after a gap)");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCB0CAPT)[2 * n]; }
};

/// TCB n count input: the positive edge of the event is the counter
/// clock (CLKSEL = EVENT in the TCB). Sync.
template <uint8_t n>
struct EvTcbCountIn : EventUserBase<EvTcbCountIn<n>> {
    static_assert(n <= 3, "TCB0..TCB3 (TCB4 is a 64-pin part: its user registers follow after a gap)");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCB0COUNT)[2 * n]; }
};

static_assert(EventGenerator<EvTcbCapt<0>>);
static_assert(EventUser<EvTcbCaptIn<0>>);
/// CCL LUT n event input A ('A') or B ('B'): the channel's signal
/// reaches the truth table as is (no detection, async) when the LUT
/// selects EVENTA/EVENTB on one of its inputs (ccl.hpp). 0x00 + 2n (+1).
template <uint8_t n, char which>
struct EvLutIn : EventUserBase<EvLutIn<n, which>> {
    static_assert(n <= 5, "LUT0..LUT5");
    static_assert(which == 'A' || which == 'B', "LUT event inputs: 'A' or 'B'");
    static volatile uint8_t& reg() { return (&EVSYS.USERCCLLUT0A)[2 * n + (which == 'B' ? 1 : 0)]; }
};

static_assert(EventGenerator<EvPitDiv<64>>);
static_assert(EventGenerator<EvPin<Pin<'A', 2>>>);
static_assert(EventUser<EvOut<Pin<'D', 2>>>);

} // namespace brio
