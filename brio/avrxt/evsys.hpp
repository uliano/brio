/*
 * evsys.hpp
 *
 * The event system (EVSYS) of the AVR DA/DB (DS40002247B ch. 16) and of
 * the AVR EA (DS40002443A ch. 16) as brio sees it - see
 * docs/avrxt/evsys.md for the analysis and the model:
 *
 *  - a GENERATOR is a type: its CHANNELn code and the channels it may
 *    legally drive are constexpr facts (EvPitDiv<64>, EvPin<Pin<'A',2>>,
 *    EvRtcOvf...); the concept EventGenerator names the contract. A
 *    generator that is a SLOT of another peripheral (the AVR EA's pin
 *    and PIT generators) carries the slot in its type and a prepare()
 *    that selects what the slot carries; source() calls it first;
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
 * THE TWO MODELS. On the AVR DA/DB every pin and eight fixed PIT
 * dividers are generators of their own, confined to channel pairs and
 * parities (16.5.2). On the AVR EA (16.4, 16.5.2) six channels take EVERY
 * generator, but pins and PIT dividers reach them through SLOTS: two per
 * port (PORTx.EVGENCTRLA picks which pin each carries, 18.5.17) and two
 * in the RTC (RTC.PITEVGENCTRLA picks which divider, DIV4..DIV32768,
 * 24.13.17) - a resource the app names as it names a channel:
 * EvPin<Pin<'A', 4>, 1> is "PA4 through PORTA's slot 1",
 * EvPitDiv<1024, 0> "CLK_RTC/1024 through the PIT's slot 0". The compiler
 * checks the slot exists; that two routes wanting two pins of one port
 * took different slots is ownership, like a channel's. On the DA/DB the
 * slot is 0 and nothing else (no slot register exists there).
 *
 * Facts worth remembering (16.3.2): a pin generator is the pin LEVEL
 * (the user's edge detection makes the edge; the pin's input driver
 * must be enabled); PIT_DIVn are levels - a free square wave off the
 * RTC's prescaler chain, n cycles of CLK_RTC per period (32768 / n Hz
 * on a 32.768 kHz source), independent of the RTC counter's own
 * PRESCALER (rtc.hpp); software events need CLK_PER (not in standby);
 * async users respond in standby without a clock. Errata DS80000915F:
 * no EVSYS items; DS80000882C 2.6.1 (every AVR DA revision): PB6/PB7
 * and PE4..PE7 are not connected to the event system, neither as a pin
 * generator nor as EVOUT - refused at compile time on a DA (the die,
 * avrxt/device_tables.hpp's erratum_da_evsys_pins), which bonds them on
 * 64 pins. DS80001048D (the AVR EA) has no EVSYS, PORT or RTC item. The
 * channel and instance counts the types below assert against are the
 * device tables' too.
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>
#include <concepts>

#include "avrxt/device_tables.hpp"
#include "avrxt/pin.hpp"

namespace brio {

/// DS80000882C 2.6.1: a pin the event system does not reach on a DA.
constexpr bool evsys_pin_unconnected(char port, uint8_t pin) {
    return erratum_da_evsys_pins && ((port == 'B' && pin >= 6) || (port == 'E' && pin >= 4));
}

// ---- contracts --------------------------------------------------------------

/// A generator: the CHANNELn code and the channels it may drive.
template <typename G>
concept EventGenerator = requires(uint8_t ch) {
    { G::code } -> std::convertible_to<uint8_t>;
    { G::legal_on(ch) } -> std::same_as<bool>;
};

/// A generator that is a slot of its peripheral: prepare() selects what
/// the slot carries, and EventChannel::source() calls it BEFORE writing
/// CHANNELn, so a channel already listened to never carries the slot's
/// previous selection after the route is made.
template <typename G>
concept SlottedEventGenerator = EventGenerator<G> && requires { G::prepare(); };

/// A user: exposes its USER register (the channel mux) and the
/// listen/unlisten pair (usually inherited from EventUserBase).
template <typename U>
concept EventUser = requires {
    { U::reg() } -> std::same_as<volatile uint8_t&>;
    U::unlisten();
};

// ---- channels ---------------------------------------------------------------

// The number of event channels is device_tables.hpp's event_channels,
// read off the EVSYS struct: CHANNEL0..9 with the software events split
// SWEVENTA/SWEVENTB on 48/64-pin DA/DB, 8 channels and SWEVENTA only on
// 28/32-pin parts.

/// Software event on a channel chosen at RUN time (for drivers that
/// store their channel as a value: a one-shot's trigger, a cascade's
/// snapshot). The compile-time form is EventChannel<n>::pulse().
inline void evsys_pulse(uint8_t ch) {
#ifdef EVSYS_SWEVENTB_gm
    if (ch >= 8) { EVSYS.SWEVENTB = static_cast<uint8_t>(1u << (ch - 8)); return; }
#endif
    EVSYS.SWEVENTA = static_cast<uint8_t>(1u << ch);
}

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
                      "(AVR DA/DB: PIT_DIV8192..1024 on even channels, PIT_DIV512..64 on "
                      "odd; PORTA/B pins on channels 0-1, PORTC/D 2-3, PORTE/F 4-5)");
        if constexpr (SlottedEventGenerator<G>) {
            G::prepare();
        }
        reg() = G::code;
    }

    /// Disconnect the generator (channel idle, users see nothing).
    static void off() { reg() = 0; }

    /// Software event: one CLK_PER pulse on the channel (needs CLK_PER,
    /// i.e. not in standby sleep). Users see it as any other event.
    static void pulse() {
        if constexpr (n < 8) {
            EVSYS.SWEVENTA = static_cast<uint8_t>(1u << n);
        }
#ifdef EVSYS_SWEVENTB_gm
        else {
            EVSYS.SWEVENTB = static_cast<uint8_t>(1u << (n - 8));
        }
#endif
    }

private:
    static volatile uint8_t& reg() { return (&EVSYS.CHANNEL0)[n]; }
};

// ---- generators (16.5.2) ------------------------------------------------------

/// The PIT's divided RTC clock as an event (a level: a free square wave
/// off the RTC's prescaler chain, div cycles of CLK_RTC per period).
///
/// AVR DA/DB: eight fixed generators, 8192/4096/2048/1024 on EVEN channels
/// only, 512/256/128/64 on ODD channels only (the two families share
/// codes 0x08..0x0B - the parity of the channel picks the family); the
/// slot must be 0.
///
/// AVR EA: two SLOTS in the RTC, each carrying one divider 4..32768
/// chosen in RTC.PITEVGENCTRLA (code log2(div) - 1, DS40002443A
/// 24.13.17), on any channel; the channel code names the slot (0x08 +
/// slot). A slot carries its divider only while the PIT is enabled
/// (PITEN): with the counter alone running the shared prescaler, which
/// 24.5.2.2 says runs while either function is, the slots stay still
/// (measured, docs/avrxt/evsys.md).
template <uint16_t div, uint8_t slot = 0>
struct EvPitDiv {
    static constexpr uint8_t log2_div =
        static_cast<uint8_t>(div == 0 ? 0 : 15 - __builtin_clz(static_cast<unsigned>(div)));
    static constexpr bool power_of_two = div != 0 && (div & (div - 1)) == 0;
    static_assert(slot <= 1, "two PIT event slots: 0 and 1");
    static_assert(has_pit_evgen || slot == 0,
                  "this part has no PIT event slots (RTC.PITEVGENCTRLA is the AVR EA's): "
                  "its PIT dividers are fixed generators - leave the slot 0");
    static_assert(has_pit_evgen || (power_of_two && div >= 64 && div <= 8192),
                  "PIT event dividers: 64, 128, 256, 512 (odd channels), "
                  "1024, 2048, 4096, 8192 (even channels)");
    static_assert(!has_pit_evgen || (power_of_two && div >= 4),
                  "PIT event slot dividers: 4, 8, ..., 32768 (DS40002443A 24.13.17)");
    static constexpr bool even_family = div >= 1024;
    static constexpr uint8_t code =
        has_pit_evgen ? static_cast<uint8_t>(0x08 + slot) :
        div == 8192 || div == 512 ? 0x08 :
        div == 4096 || div == 256 ? 0x09 :
        div == 2048 || div == 128 ? 0x0A : 0x0B;
    static constexpr bool legal_on(uint8_t ch) {
        if constexpr (has_pit_evgen) {
            (void)ch;
            return true;
        } else {
            return even_family ? (ch % 2 == 0) : (ch % 2 == 1);
        }
    }
#if defined(RTC_EVGEN0SEL_gm)
    /// The slot's divider: a read-modify-write of its nibble in
    /// RTC.PITEVGENCTRLA (the other slot untouched; no synchronization
    /// is documented for this register).
    static void prepare() {
        constexpr uint8_t gp = slot == 0 ? RTC_EVGEN0SEL_gp : RTC_EVGEN1SEL_gp;
        constexpr uint8_t gm = slot == 0 ? RTC_EVGEN0SEL_gm : RTC_EVGEN1SEL_gm;
        RTC.PITEVGENCTRLA = static_cast<uint8_t>((RTC.PITEVGENCTRLA & ~gm) |
                                                 ((log2_div - 1u) << gp));
    }
    /// Stop the slot (OFF): its channel then carries a constant low.
    static void release() {
        constexpr uint8_t gm = slot == 0 ? RTC_EVGEN0SEL_gm : RTC_EVGEN1SEL_gm;
        RTC.PITEVGENCTRLA = static_cast<uint8_t>(RTC.PITEVGENCTRLA & ~gm);
    }
#endif
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

#if defined(ADC_PGAEN_bm)
/// ADC0 sample ready and window compare (the AVR EA's differential ADC,
/// DS40002443A 16.5.2): pulses, all channels. 0x25, 0x26.
struct EvAdc0Sample {
    static constexpr uint8_t code = 0x25;
    static constexpr bool legal_on(uint8_t) { return true; }
};
struct EvAdc0Window {
    static constexpr uint8_t code = 0x26;
    static constexpr bool legal_on(uint8_t) { return true; }
};
#endif

/// UPDI SYNCH character seen (pulse). All channels. 0x01.
struct EvUpdiSynch {
    static constexpr uint8_t code = 0x01;
    static constexpr bool legal_on(uint8_t) { return true; }
};

#ifdef MVIO
/// MVIO VDDIO2 OK level (async, DB only). All channels. 0x05.
struct EvMvioOk {
    static constexpr uint8_t code = 0x05;
    static constexpr bool legal_on(uint8_t) { return true; }
};
#endif

#ifdef ZCD0
/// ZCD n zero-cross output LEVEL (async). All channels. 0x30 + n.
template <uint8_t n>
struct EvZcdOut {
    static_assert(n < zcd_count, "no such ZCD on this package (ZCD0 on 28/32 pins; "
                                 "ZCD0..1 on the AVR DA48, ZCD0..2 on the DB48 and the 64-pin parts)");
    static constexpr uint8_t code = static_cast<uint8_t>(0x30 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};
#endif

// How many op amps this package has is device_tables.hpp's
// opamp_count: none on the DA family (no OPAMP peripheral at all), two
// on the 28/32-pin DB parts, three on 48 and 64 pins - the OP2
// registers are simply absent from the OPAMP_t of the smaller packages.

#ifdef OPAMP
/// OPAMP n ready (DB only). All channels. 0x34 + n.
template <uint8_t n>
struct EvOpampReady {
    static_assert(n < opamp_count, "no such op amp on this package (OP2 on 48/64 pins)");
    static constexpr uint8_t code = static_cast<uint8_t>(0x34 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};
#endif

// How many USART instances this package has is device_tables.hpp's
// usart_count: 3 on 28/32-pin, 5 on 48, 6 on 64.

/// USART n XCK LEVEL (sync host mode). All channels. 0x60 + n.
template <uint8_t n>
struct EvUsartXck {
    static_assert(n < usart_count, "no such USART on this package");
    static constexpr uint8_t code = static_cast<uint8_t>(0x60 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// SPI n SCK LEVEL (host mode). All channels. 0x68 + n.
template <uint8_t n>
struct EvSpiSck {
    static_assert(n < spi_count, "no such SPI on this part");
    static constexpr uint8_t code = static_cast<uint8_t>(0x68 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

#ifdef TCD0
/// TCD0 compare events (async). All channels. 0xB0..0xB3.
struct EvTcdCmpBClr { static constexpr uint8_t code = 0xB0; static constexpr bool legal_on(uint8_t) { return true; } };
struct EvTcdCmpASet { static constexpr uint8_t code = 0xB1; static constexpr bool legal_on(uint8_t) { return true; } };
struct EvTcdCmpBSet { static constexpr uint8_t code = 0xB2; static constexpr bool legal_on(uint8_t) { return true; } };
struct EvTcdProgEv  { static constexpr uint8_t code = 0xB3; static constexpr bool legal_on(uint8_t) { return true; } };
#endif

/// TCA n overflow (normal mode) / low-byte underflow (split mode):
/// pulse, CLK_PER, all channels. Codes 0x80 (TCA0), 0x88 (TCA1).
template <uint8_t n>
struct EvTcaOvf {
    static_assert(n < tca_count, "TCA0 and (48/64-pin parts) TCA1");
    static constexpr uint8_t code = static_cast<uint8_t>(0x80 + 8 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCA n high-byte underflow (split mode only). 0x81 / 0x89.
template <uint8_t n>
struct EvTcaHunf {
    static_assert(n < tca_count, "TCA0 and (48/64-pin parts) TCA1");
    static constexpr uint8_t code = static_cast<uint8_t>(0x81 + 8 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCA n compare channel ch match (normal) / low-byte compare ch
/// (split): pulse, all channels. 0x84 + ch (TCA0), 0x8C + ch (TCA1).
template <uint8_t n, uint8_t ch>
struct EvTcaCmp {
    static_assert(n < tca_count, "TCA0 and (48/64-pin parts) TCA1");
    static_assert(ch <= 2, "TCA compare channels: 0..2");
    static constexpr uint8_t code = static_cast<uint8_t>(0x84 + 8 * n + ch);
    static constexpr bool legal_on(uint8_t) { return true; }
};

// How many TCB instances this package has is device_tables.hpp's
// tcb_count: 3 on 28/32-pin, 4 on 48, 5 on 64.

/// TCB n CAPT flag set (the condition depends on the mode, tcb.hpp):
/// pulse, CLK_PER, all channels. 0xA0 + 2n.
template <uint8_t n>
struct EvTcbCapt {
    static_assert(n < tcb_count, "no such TCB on this package");
    static constexpr uint8_t code = static_cast<uint8_t>(0xA0 + 2 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// TCB n counter overflow (MAX -> BOTTOM): pulse, all channels. 0xA1 + 2n.
/// The carry of a 32-bit cascade.
template <uint8_t n>
struct EvTcbOvf {
    static_assert(n < tcb_count, "no such TCB on this package");
    static constexpr uint8_t code = static_cast<uint8_t>(0xA1 + 2 * n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

// How many CCL LUTs this package has is device_tables.hpp's
// lut_count: the LUT4 register macros exist on 48/64-pin parts only.

/// CCL LUT n output LEVEL (async). 0x10 + n.
template <uint8_t n>
struct EvLut {
    static_assert(n < lut_count, "no such CCL LUT on this package");
    static constexpr uint8_t code = static_cast<uint8_t>(0x10 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// Analog comparator n output LEVEL (async). 0x20 + n.
template <uint8_t n>
struct EvAcOut {
    static_assert(n < ac_count, "no such analog comparator on this part");
    static constexpr uint8_t code = static_cast<uint8_t>(0x20 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// A port pin's LEVEL as an event (async; zero if the input driver is
/// disabled).
///
/// AVR DA/DB: every pin is a generator of its own, confined to a channel
/// pair - PORTA/PORTB on channels 0-1, PORTC/PORTD 2-3, PORTE/PORTF 4-5;
/// the two ports of a pair share the code space (0x40 + n for the first,
/// 0x48 + n for the second). The slot must be 0.
///
/// AVR EA: a port has two event SLOTS (PORTx.EVGENCTRLA, DS40002443A
/// 18.5.17), each carrying ONE of its pins, on any channel; the channel
/// code names the port and the slot (0x40 + 2 x port + slot, 16.5.2).
template <typename P, uint8_t slot = 0>
struct EvPin {
    static constexpr char port = P::port_letter;
    static constexpr uint8_t pin = P::pin_number;
    static_assert(port >= 'A' && port <= 'G', "pin events: PORTA..PORTG");
    static_assert(port_exists(port), "this port does not exist on this package");
    static_assert(!evsys_pin_unconnected(port, pin),
                  "PB6/PB7 and PE4..PE7 are not connected to the event system on an "
                  "AVR DA (DS80000882C 2.6.1)");
    static_assert(slot <= 1, "two pin event slots per port: 0 and 1");
    static_assert(has_port_evgen || slot == 0,
                  "this part has no pin event slots (PORTx.EVGENCTRLA is the AVR EA's): "
                  "every pin is a generator of its own - leave the slot 0");
    static constexpr uint8_t code =
        has_port_evgen ? static_cast<uint8_t>(0x40 + 2 * (port - 'A') + slot)
                       : static_cast<uint8_t>(((port - 'A') % 2 == 0 ? 0x40 : 0x48) + pin);
    static constexpr bool legal_on(uint8_t ch) {
        if constexpr (has_port_evgen) {
            (void)ch;
            return true;
        } else {
            const uint8_t pair = static_cast<uint8_t>((port - 'A') / 2);   // A/B=0, C/D=1, E/F=2, G=3
            return ch / 2 == pair;
        }
    }
#if defined(PORT_EVGEN0SEL_gm)
    /// Select this pin in the port's slot: a read-modify-write of the
    /// slot's field in PORTx.EVGENCTRLA, the other slot untouched. Two
    /// contexts re-selecting the two slots of one port would race on the
    /// shared register; one owner per port's slots is the app's.
    static void prepare() {
        constexpr uint8_t gp = slot == 0 ? PORT_EVGEN0SEL_gp : PORT_EVGEN1SEL_gp;
        constexpr uint8_t gm = slot == 0 ? PORT_EVGEN0SEL_gm : PORT_EVGEN1SEL_gm;
        volatile uint8_t& r = P::port().EVGENCTRLA;
        r = static_cast<uint8_t>((r & ~gm) | (pin << gp));
    }
#endif
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
    static_assert(port >= 'A' && port <= 'G', "EVOUT: PORTA..PORTG");
    static_assert(port_exists(port), "this port does not exist on this package");
    static_assert(!(port == 'F' && pin == 7 && avr_die == AvrDie::avr64ea),
                  "EVOUTF's ALT1 position on the AVR EA is PF7, the UPDI pin: a GPIO only "
                  "with FUSE.SYSCFG0.UPDIPINCFG = 0, which leaves the part unreachable "
                  "without a high-voltage programmer - refused");
    static_assert(pin == 2 || (pin == 7 && (avr_die == AvrDie::avr64ea
                                                ? (port == 'A' || port == 'C' || port == 'D' ||
                                                   port == 'F')
                                                : port != 'F')),
                  "EVOUT pins: Px2 in the default position on every port, Px7 "
                  "(ALT1) on every port except PORTF on the AVR DA/DB (DS40002247B "
                  "17.5.1), on PORTA, PORTC and PORTD on the AVR EA (DS40002443A "
                  "17.5.1); whether THIS package bonds pin 7 of an existing port is "
                  "a pin-level fact for the device tables");
    static_assert(!evsys_pin_unconnected(port, pin),
                  "EVOUT on PB7/PE7 does not reach its pin on an AVR DA "
                  "(DS80000882C 2.6.1)");

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
#ifdef PORTG
        else if constexpr (port == 'G') return EVSYS.USEREVSYSEVOUTG;
#endif
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

    /// Disconnect AND tear down what listen() set up: the pin back to
    /// an input, the PORTMUX position back to default (the "Entry
    /// routes, Exit disconnects" idiom leaves no trace).
    static void unlisten() {
        EventUserBase<EvOut<P>>::unlisten();
        P::input();
        if constexpr (pin == 7) {
            PORTMUX.EVSYSROUTEA &= static_cast<uint8_t>(~(1u << (port - 'A')));
        }
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
    static_assert(n < tca_count, "TCA0 and (48/64-pin parts) TCA1");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCA0CNTA)[2 * n]; }
};

/// TCA n event input B: restart on edge / while high, or direction.
template <uint8_t n>
struct EvTcaCntB : EventUserBase<EvTcaCntB<n>> {
    static_assert(n < tca_count, "TCA0 and (48/64-pin parts) TCA1");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCA0CNTB)[2 * n]; }
};

/// TCB n capture input: start/stop/capture/restart per the TCB's mode
/// and EDGE bit (tcb.hpp); sync, async in single-shot with ASYNC.
/// The TCB must also set CAPTEI - listening alone arms nothing.
template <uint8_t n>
struct EvTcbCaptIn : EventUserBase<EvTcbCaptIn<n>> {
    static_assert(n < tcb_count, "no such TCB on this package");
    // USER registers 0x1E..0x27 are contiguous through TCB4 (16.5.3).
    static volatile uint8_t& reg() { return (&EVSYS.USERTCB0CAPT)[2 * n]; }
};

/// TCB n count input: the positive edge of the event is the counter
/// clock (CLKSEL = EVENT in the TCB). Sync.
template <uint8_t n>
struct EvTcbCountIn : EventUserBase<EvTcbCountIn<n>> {
    static_assert(n < tcb_count, "no such TCB on this package");
    static volatile uint8_t& reg() { return (&EVSYS.USERTCB0COUNT)[2 * n]; }
};

static_assert(EventGenerator<EvTcbCapt<0>>);
static_assert(EventUser<EvTcbCaptIn<0>>);
/// CCL LUT n event input A ('A') or B ('B'): the channel's signal
/// reaches the truth table as is (no detection, async) when the LUT
/// selects EVENTA/EVENTB on one of its inputs (ccl.hpp). 0x00 + 2n (+1).
template <uint8_t n, char which>
struct EvLutIn : EventUserBase<EvLutIn<n, which>> {
    static_assert(n < lut_count, "no such CCL LUT on this package");
    static_assert(which == 'A' || which == 'B', "LUT event inputs: 'A' or 'B'");
    static volatile uint8_t& reg() { return (&EVSYS.USERCCLLUT0A)[2 * n + (which == 'B' ? 1 : 0)]; }
};

/// EVOUT on PORTG's pins (64-pin parts): the same EvOut, its port
/// gated above. USART n IRDA event input (sync). 0x14 + n.
template <uint8_t n>
struct EvUsartIrda : EventUserBase<EvUsartIrda<n>> {
    static_assert(n < usart_count, "no such USART on this package");
    static volatile uint8_t& reg() { return (&EVSYS.USERUSART0IRDA)[n]; }
};

#ifdef TCD0
/// TCD0 event inputs A and B (the action is in the TCD's config).
struct EvTcdInputA : EventUserBase<EvTcdInputA> {
    static volatile uint8_t& reg() { return EVSYS.USERTCD0INPUTA; }
};
struct EvTcdInputB : EventUserBase<EvTcdInputB> {
    static volatile uint8_t& reg() { return EVSYS.USERTCD0INPUTB; }
};
#endif

#ifdef OPAMP
/// OPAMP n event-driven controls (DB only): enable, disable, dump the
/// integrator, drive the output. Four users per instance, contiguous.
/// ENABLEn and DISABLEn are EDGE-detected, DUMPn and DRIVEn are LEVELS
/// (table 35-2): a channel driving a dump or a drive must HOLD its
/// level, a software pulse does nothing there.
enum class OpampAction : uint8_t { enable = 0, disable = 1, dump = 2, drive = 3 };
template <uint8_t n, OpampAction a>
struct EvOpampCtl : EventUserBase<EvOpampCtl<n, a>> {
    static_assert(n < opamp_count, "no such op amp on this package (OP2 on 48/64 pins)");
    static volatile uint8_t& reg() {
        return (&EVSYS.USEROPAMP0ENABLE)[4 * n + static_cast<uint8_t>(a)];
    }
};
#endif

static_assert(EventGenerator<EvPitDiv<1024>>);
static_assert(EventGenerator<EvPin<Pin<'A', 2>>>);
static_assert(EventGenerator<EvUpdiSynch>);
#ifdef ZCD0
static_assert(EventGenerator<EvZcdOut<0>>);
#endif
static_assert(EventUser<EvOut<Pin<'D', 2>>>);
static_assert(EventUser<EvUsartIrda<0>>);

} // namespace brio
