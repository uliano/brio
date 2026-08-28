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
 * must be enabled); PIT_DIVn are levels - a free square wave off the
 * RTC's prescaler chain, n cycles of CLK_RTC per period (32768 / n Hz
 * on a 32.768 kHz source), independent of the RTC counter's own
 * PRESCALER (rtc.hpp); software events need CLK_PER (not in standby);
 * async users respond in standby without a clock. Errata DS80000915F:
 * no EVSYS items.
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

/// Number of event channels on this device: CHANNEL0..9 with the
/// software events split SWEVENTA/SWEVENTB on 48/64-pin DA/DB;
/// 8 channels and SWEVENTA only on 28/32-pin parts (the device header
/// is the authority).
#ifdef EVSYS_SWEVENTB_gm
inline constexpr uint8_t event_channels = 10;
#else
inline constexpr uint8_t event_channels = 8;
#endif

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

/// ZCD n zero-cross output LEVEL (async). All channels. 0x30 + n.
template <uint8_t n>
struct EvZcdOut {
    static_assert(n <= 2, "ZCD0..ZCD2");
    static constexpr uint8_t code = static_cast<uint8_t>(0x30 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};

/// How many op amps this package has: none on the DA family (no OPAMP
/// peripheral at all), two on the 28/32-pin DB parts, three on 48 and
/// 64 pins. The device header is the authority - the OP2 registers are
/// simply absent from the OPAMP_t of the smaller packages, and that is
/// what the detection below asks about.
#ifdef OPAMP
/// The concept is what makes the member lookup a substitution rather
/// than a hard error: a requires-expression over a NON-dependent type
/// is checked immediately and would not compile where OP2 is absent.
template <typename T>
concept HasOpamp2Registers = requires(T& o) { o.OP2CTRLA; };
inline constexpr uint8_t opamp_count = HasOpamp2Registers<OPAMP_t> ? 3 : 2;
#else
inline constexpr uint8_t opamp_count = 0;
#endif

#ifdef OPAMP
/// OPAMP n ready (DB only). All channels. 0x34 + n.
template <uint8_t n>
struct EvOpampReady {
    static_assert(n < opamp_count, "no such op amp on this package (OP2 on 48/64 pins)");
    static constexpr uint8_t code = static_cast<uint8_t>(0x34 + n);
    static constexpr bool legal_on(uint8_t) { return true; }
};
#endif

/// How many USART instances this package has (the device header is
/// the authority): 3 on 28/32-pin, 5 on 48, 6 on 64.
#if defined(USART5)
inline constexpr uint8_t usart_count = 6;
#elif defined(USART3)
inline constexpr uint8_t usart_count = 5;
#else
inline constexpr uint8_t usart_count = 3;
#endif

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
    static_assert(n <= 1, "SPI0..SPI1");
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

/// How many TCB instances this package has (the device header is the
/// authority): 3 on 28/32-pin, 4 on 48, 5 on 64.
#if defined(TCB4)
inline constexpr uint8_t tcb_count = 5;
#elif defined(TCB3)
inline constexpr uint8_t tcb_count = 4;
#else
inline constexpr uint8_t tcb_count = 3;
#endif

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

/// How many CCL LUTs this package has (the device header is the
/// authority: the LUT4 register macros exist on 48/64-pin parts only).
#ifdef CCL_LUT4CTRLA
inline constexpr uint8_t lut_count = 6;
#else
inline constexpr uint8_t lut_count = 4;
#endif

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
    static_assert(port >= 'A' && port <= 'G', "pin events: PORTA..PORTG");
    static_assert(port_exists(port), "this port does not exist on this package");
    static constexpr uint8_t code =
        static_cast<uint8_t>(((port - 'A') % 2 == 0 ? 0x40 : 0x48) + pin);
    static constexpr bool legal_on(uint8_t ch) {
        const uint8_t pair = static_cast<uint8_t>((port - 'A') / 2);   // A/B=0, C/D=1, E/F=2, G=3
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
    static_assert(port >= 'A' && port <= 'G', "EVOUT: PORTA..PORTG");
    static_assert(port_exists(port), "this port does not exist on this package");
    static_assert(pin == 2 || (pin == 7 && port != 'F'),
                  "EVOUT pins: Px2 in the default position on every port, Px7 "
                  "(ALT1) on every port except PORTF (17.5.1; whether THIS "
                  "package bonds pin 7 of an existing port is a pin-level "
                  "fact for the device tables)");

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

static_assert(EventGenerator<EvPitDiv<64>>);
static_assert(EventGenerator<EvPin<Pin<'A', 2>>>);
static_assert(EventGenerator<EvUpdiSynch>);
static_assert(EventGenerator<EvZcdOut<0>>);
static_assert(EventUser<EvOut<Pin<'D', 2>>>);
static_assert(EventUser<EvUsartIrda<0>>);

} // namespace brio
